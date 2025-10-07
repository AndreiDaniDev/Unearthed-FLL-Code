# ---> Written by AndreiDani <---
from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Direction, Stop, Axis, Button, Color
from pybricks.pupdevices import Motor 
from pybricks.tools import wait, run_task, multitask, StopWatch
from pybricks.robotics import DriveBase

timer = StopWatch()

# ---> Max value for int = 2 ^ 31 - 1 = 1073741823 <--- #
def limitint(value: int, valuemin: int, valuemax: int) -> int:
    return min(max(value, valuemin), valuemax)

def inrange(value: int, valuemin: int, valuemax: int) -> int:
    return (valuemin <= value and value <= valuemax)

def myabs(value: int) -> int:
    return (-value if (value < 0) else value)

async def delayfunction(value: int, myfunc) -> None:
    await wait(value)
    await myfunc()

async def rundegrees(mymotor: Motor, angle: int, speed: int) -> None:
    await mymotor.run_angle(speed, angle)
    mymotor.brake(); return None

# pybricksdev run ble "Unearthed - FLL/unearthedfunctions.py"

# ---> Proportional - Integral - Derivative Controller <--- #
class PID_Controller(object):
    def __init__(self) -> None:

        # ---> Base variables (int) <--- #
        self.kp: int = 0; self.kd: int = 0; self.ki: int = 0
        self.cstscale: int = 1000

        self.error: int = 0; self.lasterror: int = 0; self.dt: int = 10
        self.proportional: int = 0; self.derivative: int = 0; self.integral: int = 0

        # ---> Variables for modifications to the PID controller <---
        self.maxchange: int = 50 # ---> Max error change <---
        self.integralwindup = (1 << 24) # ---> Integral windup <---
        
        # ---> Cost function for noise filtering and gradient ascent tuning algorithm <---
        self.costf: int = 0 # ---> f(x) = alpha * error * error + (1 - alpha) * f(x - 1) <---
        self.errorsmooth: int = 0 # ---> g(x) = alpha * error + (1 - alpha) * f(x - 1) <---
        self.alpha: int = 625 # ---> alpha constant (int -> cstscale) <---

        self.deadzone: int = 5 # ---> If error <= deadzone -> error = 0 <---

        return None

    def setconstants(self, dkp: int, dkd: int, dki: int) -> None:
        self.kp = dkp; self.kd = dkd; self.ki = dki; return None
    
    def noisefiltering(self) -> None:
        self.errorsmooth = (self.alpha * self.error + (self.cstscale - self.alpha) * self.errorsmooth) // self.cstscale
        self.error = self.errorsmooth # ---> Reassign filtered error to raw error to be applied in the controller <--- 

    # def gradientascent(self) -> None:
    #     self.costf = (self.alpha * self.error * self.error + (self.cstscale - self.alpha) * self.costf) / self.cstscale

    def compute(self, error: int, sign: int) -> int:
        
        # ---> Limiting the error to be in range of [lasterror +- maxchange] <--- #
        self.error = 0 if(abs(self.error) <= self.deadzone) else error # ---> Deadzone for error <--- # 
        self.error = limitint(error, self.lasterror - self.maxchange, self.lasterror + self.maxchange)

        self.noisefiltering() # ---> Filter errors and noise from the input <---
        # self.gradientascent() # ---> Tune the constants according to the gradient ascent algorithm <---

        self.proportional = self.error
        self.derivative = (self.error - self.lasterror)

        self.integral += self.dt * self.error
        self.integral = limitint(self.integral, -self.integralwindup, self.integralwindup)

        self.controlleroutput = int(
            self.proportional * self.kp + 
            self.derivative * self.kd / self.dt + 
            self.integral * self.ki
        ) // self.cstscale * sign

        self.lasterror = self.error # ---> Changing last error <--- #

        return 
   
class DriveBase_Contorller(object):
    def __init__(self, lfmotor: Motor, rgmotor: Motor, syslfmotor: Motor, sysrgmotor: Motor, oneunit: int, scaleunit: int, wheelerror: int = None, distbetweenwheel: int = 0, wheelcircumference: int = 0) -> None:

        # ---> Hub and Gyro sensor initialization <--- #
        self.hub = PrimeHub() 
        self.gyro = self.hub.imu
        self.gyro.reset_heading(0)

        # print(self.hub.battery.voltage())
        if(self.hub.battery.voltage() <= 7500):
            print("Failed to initialize - Battery too low\n")

        # ---> Drivebase motors initializations <--- #
        self.lefttmotor: Motor = lfmotor
        self.righttmotor: Motor = rgmotor

        self.syslefttmotor: Motor = syslfmotor
        self.sysrighttmotor: Motor = sysrgmotor

        self.lefttmotor.control.target_tolerances(None, wheelerror)
        self.righttmotor.control.target_tolerances(None, wheelerror)

        # ---> Formulas used for calculating the speed for turning <--- #
        self.distwheels: int = distbetweenwheel
        self.circumferencerobot: int = int(2 * 314 * distbetweenwheel // 100)
        self.wheelcircumference = int(wheelcircumference)

        # ---> Object controller for swaps in programs <--- #
        self.PID = PID_Controller()
        self.updatetime: int = 100

        # ---> Transforming Distance <--- #
        self.oneunit = oneunit; self.scaleunit = scaleunit

        # ---> Moving the robot <--- #
        self.lfspeed: int = 0; self.rgspeed: int = 0
        self.speedlow: int = 0; self.speedhigh: int = 0
        self.lfsign: int = 0; self.rgsign: int = 0

        self.minspeed: int = 50; self.maxspeed: int = 1100
        self.mingoodspeed: int = 150; self.maxgoodspeed: int = 1000

        self.error: int = 0; self.distance: int = 0
        self.angle: int = 0; self.targetangle: int = 0

        self.position: int = 0; 

        # ---> Acceptable error for turns <--- #
        self.epsilon: int = 2
        self.epsilonlowwspeed: int = 32

    async def __initrun__(self) -> None:

        self.gyro.reset_heading(0)
        self.PID.__init__() 

        await wait(250)

        return None

    # ---> Pair motor functions (Move tank and stop) <--- #
    def movetank(self, speedlf: int, speedrg: int) -> None:
        self.lefttmotor.run(speedlf)
        self.righttmotor.run(speedrg)

    # ---> Stop tank functions <---
    def stoptank(self)  -> None: self.lefttmotor.stop();  self.righttmotor.stop()
    def braketank(self) -> None: self.lefttmotor.brake(); self.righttmotor.brake()
    def holdtank(self)  -> None: self.lefttmotor.hold();  self.righttmotor.hold()
    def stalledtank(self) -> int: return (self.lefttmotor.stalled() & self.righttmotor.stalled()) 

    # ---> Helper functions for travelling distances <--- #
    def getdistance(self, dist: int) -> int: return dist * self.oneunit // self.scaleunit
    def getmotorsangle(self) -> int: return int(myabs(self.lefttmotor.angle() + self.righttmotor.angle()))
    def getangle(self) -> int: return int(self.gyro.heading())

    def computeturnspeed(self, speedangle: int) -> int:
        # ---> Formulas: https://www.desmos.com/calculator/ww29qyn3yt <---
        speed = (self.circumferencerobot * speedangle // 360) 
        speed = self.getdistance(speed)
        return int(speed)

    def computespeedmx(self, speed: int) -> None:
        if(speed == 0): return 0; 

        if(speed < 0): return limitint(speed, -self.maxspeed, -self.minspeed)
        elif(speed > 0):  return limitint(speed, self.minspeed, self.maxspeed)

    def computeturnsigns(self) -> None:
        self.lfsign = myabs(self.lfsign) * (-1 if self.getangle() > self.targetangle else  1)
        self.rgsign = myabs(self.rgsign) * ( 1 if self.getangle() > self.targetangle else -1)

    # ---> Here comes the big boi (the real functions) <--- #
    async def gyroforwards(self, distance: int, speed1: int, typestop, offsetlf: int = 0, offsetrg: int = 0) -> None:

        if(typestop == None): print("Typestop: NULL\n")

        # ---> Parameters initialization <--- #
        self.distance = 2 * self.getdistance(distance)
        self.speedlow = speed1; # self.speedhigh = speed2

        # ---> Need to implement time and speeding functions (no need for speed functions anymore :DDDD)<---
        self.speed = self.speedlow 

        # ---> Reset relative position and gyro <--- #
        self.lefttmotor.reset_angle(0)
        self.righttmotor.reset_angle(0)

        self.position = 0; self.lasttposition = 0

        while(self.position < self.distance):

            self.error = self.targetangle - self.getangle()
            self.PID.compute(self.error, 1)

            # print(self.error, self.PID.error)

            self.lfspeed = limitint(self.speed + self.PID.controlleroutput + offsetlf, self.minspeed, self.maxspeed)
            self.rgspeed = limitint(self.speed - self.PID.controlleroutput + offsetrg, self.minspeed, self.maxspeed)

            self.movetank(self.lfspeed, self.rgspeed)
            await wait(self.updatetime)

            if(self.stalledtank()): typestop(); return; 
            self.position = self.getmotorsangle()
            
        if(typestop != None): typestop()

        return None
    
    async def gyrobackwards(self, distance: int, speed1: int, typestop, offsetlf: int = 0, offsetrg: int = 0) -> None:

        if(typestop == None): print("Typestop: NULL\n")

        # ---> Parameters initialization <--- #
        self.distance = 2 * self.getdistance(distance)
        self.speedlow = speed1; # self.speedhigh = speed2

        # ---> Need to implement time and speeding functions <---
        self.speed = self.speedlow 

        # ---> Reset relative position and gyro <--- #
        self.lefttmotor.reset_angle(0)
        self.righttmotor.reset_angle(0)

        self.position = 0; self.lasttposition = 0

        while(self.position < self.distance):

            self.error = self.targetangle - self.getangle()
            self.PID.compute(self.error, 1)

            # print(self.error, self.PID.error)

            self.lfspeed = -limitint(self.speed + self.PID.controlleroutput + offsetlf, self.minspeed, self.maxspeed)
            self.rgspeed = -limitint(self.speed - self.PID.controlleroutput + offsetrg, self.minspeed, self.maxspeed)

            self.movetank(self.lfspeed, self.rgspeed)
            await wait(self.updatetime)

            if(self.stalledtank()): typestop(); return; 
            self.position = self.getmotorsangle()
            
        if(typestop != None): typestop()

        return None

    async def turnleftt(self, angle: int, speedangle: int, *, lfsign: int = -1, rgsign: int = 1, typestop) -> None:

        # ---> Recommended speed angle E {75, 225}, angle = 160 <---
        speedangle = limitint(speedangle, 45, 360) # ---> Too small -> setpoint ramping = 0 <---
        self.speed = self.computeturnspeed(speedangle) // ((lfsign != 0) + (rgsign != 0))

        # ---> Compute wheels directions <--- #
        angle = -myabs(angle)
        self.lfsign = min(lfsign, 0)
        self.rgsign = max(rgsign, 0)

        self.targetangle += angle

        # ---> Problem - Not stopping if it overshoots <--- #
        while(not inrange(self.getangle(), self.targetangle - self.epsilon, self.targetangle + self.epsilon)):

            # ---> Change speed to slow down <--- #
            if(inrange(self.getangle(), self.targetangle - self.epsilonlowwspeed, self.targetangle + self.epsilonlowwspeed)):
                self.speed = min(self.speed, self.mingoodspeed)

            self.computeturnsigns() # Switch turn signs in case of overshooting
            self.lfspeed = self.computespeedmx(self.lfsign * self.speed)
            self.rgspeed = self.computespeedmx(self.rgsign * self.speed)
    
            self.movetank(self.lfspeed, self.rgspeed) 
        
            await wait(self.updatetime)

        if(typestop != None): typestop()

        return None

    async def turnrightt(self, angle: int, speedangle: int, *, lfsign: int = 1, rgsign: int = -1, typestop) -> None:

        # ---> Recommended speed angle E {75, 225}, angle = 160 <---
        speedangle = limitint(speedangle, 45, 360) # ---> Too small -> setpoint ramping = 0 <---
        self.speed = self.computeturnspeed(speedangle) // ((lfsign != 0) + (rgsign != 0))

        # ---> Compute wheels directions <--- #
        angle = myabs(angle); 
        self.lfsign = max(lfsign, 0)
        self.rgsign = min(rgsign, 0)

        self.targetangle += angle

        while(not inrange(self.getangle(), self.targetangle - self.epsilon, self.targetangle + self.epsilon)):

            # ---> Change speed to slow down <--- #
            if(inrange(self.getangle(), self.targetangle - self.epsilonlowwspeed, self.targetangle + self.epsilonlowwspeed)):
                self.speed = min(self.speed, self.mingoodspeed)

            self.computeturnsigns() # Switch turn signs in case of overshooting
            self.lfspeed = self.computespeedmx(self.lfsign * self.speed)
            self.rgspeed = self.computespeedmx(self.rgsign * self.speed)
    
            self.movetank(self.lfspeed, self.rgspeed) 
        
            await wait(self.updatetime)

        if(typestop != None): typestop()

        return None

# ---> All units of measurement are in milimeters <---
mydrivebase = DriveBase_Contorller(
    Motor(Port.E, Direction.COUNTERCLOCKWISE), 
    Motor(Port.A, Direction.CLOCKWISE), 

    Motor(Port.B, Direction.CLOCKWISE), 
    Motor(Port.F, Direction.COUNTERCLOCKWISE), 

    2316, 1000, None, distbetweenwheel = 80,
    wheelcircumference = 49
)

# My drivebase with two spike prime wheels 
# mydrivebase = DriveBase_Contorller(
#     Motor(Port.A, Direction.COUNTERCLOCKWISE), 
#     Motor(Port.C, Direction.CLOCKWISE), 
#     2045, 1000, None, distbetweenwheel = 112,
#     wheelcircumference = 176
# )

# Motor Port A = rg db wheel
# Motor Port B = lf sys wheel
# Motor Port E = lf db sheel
# Motor Port F = rg sys wheel

class runmethods(object):
    def __init__(self): return None
    
    async def runk1(self): 
        
        mydrivebase.PID.setconstants(200, 0, 1); await wait(200)

        mydrivebase.sysrighttmotor.run_angle(250, 25)
        await mydrivebase.gyroforwards(415, 750, typestop = None, offsetlf = 32); await wait(100)
        await mydrivebase.gyroforwards(100, 500, typestop = mydrivebase.braketank, offsetlf = 32)
        await rundegrees(mydrivebase.sysrighttmotor, -250, 1000)
        await mydrivebase.gyrobackwards(500, 875, typestop = mydrivebase.braketank, offsetrg = 32); await wait(250)

        return None
    
    async def runk2(self): 
        
        mydrivebase.PID.setconstants(1400, 0, 3); await wait(200)
        
        # ---> Rethink trajecory <--- #

        # mydrivebase.syslefttmotor.run_angle(1000, -100, Stop = Stop.BRAKE)
        await mydrivebase.gyroforwards(300, 600, typestop = mydrivebase.braketank); await wait(500)
        await mydrivebase.turnrightt(35, 45, typestop = mydrivebase.braketank); await wait(500)
        await mydrivebase.gyroforwards(348, 600, typestop = mydrivebase.braketank); await wait(500)

        await mydrivebase.turnleftt(130, 50, typestop = mydrivebase.braketank, lfsign = 0); await wait(500)
        await mydrivebase.gyroforwards(300, 600, typestop = mydrivebase.braketank); await wait(500)

        await rundegrees(mydrivebase.syslefttmotor,  750, 1000); await wait(500)
        await rundegrees(mydrivebase.syslefttmotor, -750, 1000); await wait(500)

        await rundegrees(mydrivebase.sysrighttmotor, 800, 1100); await wait(500)
        await rundegrees(mydrivebase.sysrighttmotor, -1325, 1100); await wait(500)
        await rundegrees(mydrivebase.sysrighttmotor, 400, 1100); await wait(250)

        mydrivebase.PID.integral = 0
        await mydrivebase.gyrobackwards(300, 400)

        return None#
    async def runk3(self): 
        
        await mydrivebase.__initrun__()              
        mydrivebase.targetangle = 0                  
        mydrivebase.PID.setconstants(1400, 0, 3); await wait(200)

        await mydrivebase.gyroforwards(630, 650, typestop = mydrivebase.stoptank); await wait(500)
        await mydrivebase.turnrightt(90, 50, typestop = mydrivebase.braketank); await wait(500)
        await mydrivebase.gyroforwards(240, 450, typestop = mydrivebase.stoptank); await wait(500)

        await mydrivebase.turnleftt(90, 50, typestop = mydrivebase.braketank, lfsign = 0); await wait(500)
             
        mydrivebase.targetangle = 0
        mydrivebase.PID.integral = 0
        await mydrivebase.gyrobackwards(95, 300, typestop = mydrivebase.braketank)

        await rundegrees(mydrivebase.sysrighttmotor, -380, 500); await wait(500)
        await mydrivebase.gyroforwards(12, 450, typestop = mydrivebase.stoptank); await wait(500)
        await rundegrees(mydrivebase.sysrighttmotor, 450, 1000); await wait(500)
        #await rundegrees(mydrivebase.sysrighttmotor, -400, 500); await wait(500)
        #await multitask(
        #mydrivebase.gyroforwards(70, 305, typestop=mydrivebase.stoptank),
    #delayfunction(110, lambda: rundegrees(mydrivebase.sysrighttmotor, 250, 1000))
#)
    
        return None
    async def runk4(self): 

        await mydrivebase.__initrun__()              
        mydrivebase.targetangle = 0                  
        mydrivebase.PID.setconstants(1400, 0, 3); await wait(200)
        await rundegrees(mydrivebase.sysrighttmotor, 200, 1000); await wait(500)
    # async def runk3(self): return None
    # async def runk4(self): return None
    # async def runk5(self): return None
    # async def runk6(self): return None
    # async def runk7(self): return None
    # async def runk8(self): return None
    # async def runk9(self): return None

class runmanager(runmethods):
    def __init__(self): 
        self.runkk: int = 0; 
        self.rundelay: int = 200
        self.maxrun: int = 9

        self.computedelay: int = 250

        return None

    # ---> Modify local memory to save the lastt idx <--- #
    def savenumber(self, runnnumber: int) -> None:
        # print(bytes([runnnumber])) # print the bytes number 
        mydrivebase.hub.system.storage(0, write = bytes([runnnumber]));

    def loadnumber(self) -> int:
        savedata = mydrivebase.hub.system.storage(0, read = 1)
        if(savedata == None): return 1;

        return max(1, savedata[0])

    # ---> Get run number and do this <--- #
    async def runtask(self, runnumber: int) -> None:
        runname: str = "runk" + str(runnumber)
        method = getattr(self, runname, None)

        if(method == None):
            return None

        self.savenumber(runnumber)
        await method()

    def nextrun(self):

        self.runkk += 1
        if(self.runkk > self.maxrun):
            self.runkk = 1
        
    def prevrun(self): 

        self.runkk -= 1
        if(self.runkk <= 0):
            self.runkk = self.maxrun

    async def computeupdate(self) -> None:
        
        # ---> Load lastt run in program <--- #
        self.runkk = self.loadnumber()
        mydrivebase.hub.display.number(self.runkk);
        mydrivebase.hub.light.on(Color.ORANGE)

        while(1): # ---> Select Programs Function <--- #
            pressed = mydrivebase.hub.buttons.pressed()

            if(Button.CENTER in pressed):
                
                self.savenumber(self.runkk)

                mydrivebase.hub.light.on(Color.RED)

                mydrivebase.__initrun__()
                # await wait(self.rundelay) # Delay in __initrun__()

                timer.reset()
                await self.runtask(self.runkk)

                ttime = int(timer.time() / 10)
                print("Run: ", self.runkk, " -> ", ttime, " seconds", sep = "")
                
                await wait(self.rundelay)

                mydrivebase.hub.light.on(Color.ORANGE)

            if(Button.LEFT in pressed): self.prevrun(); mydrivebase.hub.display.number(self.runkk); await wait(self.computedelay)
            if(Button.RIGHT in pressed): self.nextrun(); mydrivebase.hub.display.number(self.runkk); await wait(self.computedelay)

        return None

taskmanager = runmanager()

async def main() -> None:

    print(":DD - Run k\n"); # Init run K
    # mydrivebase.PID.setconstants(400, 0, 2)

    mydrivebase.hub.system.set_stop_button(Button.BLUETOOTH)
    await taskmanager.computeupdate()

    # timer.reset(); timer.resume()
    # await mydrivebase.turnleft(90, 180)
    # timer.pause(); print("For: 90, 180 -> ", timer.time())

    return None

run_task(main())

# https://www.first-lego-league.org/en/2025-26-season/challenge-resources/evaluation
# https://www.first-lego-league.org/en/2025-26-season/challenge-resources/season-documents
