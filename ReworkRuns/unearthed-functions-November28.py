# ---> Written by AndreiDani <---
from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Direction, Stop, Axis, Button, Color
from pybricks.pupdevices import Motor 
from pybricks.tools import wait, run_task, multitask, StopWatch
from pybricks.robotics import DriveBase

timer = StopWatch()
freqreset: int = 256; # ms
ttimeset: int = 64; # ms

# ---> Max value for int = 2 ^ 31 - 1 = 1073741823 <--- #
def limitint(value: int, valuemin: int, valuemax: int) -> int:
    return min(max(value, valuemin), valuemax)

def inrange(value: int, valuemin: int, valuemax: int) -> int:
    return (valuemin <= value and value <= valuemax)

def myabs(value: int) -> int:
    return (-value if (value < 0) else value)

async def delayfunction(value: int, func) -> int:
    await wait(value); await func(); return
# ---> delayfunction(value, lambda: function with parameters) <--- #

async def rundegrees(mymotor: Motor, angle: int, speed: int) -> None:
    await mymotor.run_angle(speed, angle)
    mymotor.brake(); return None

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

    def __initvalues__(self) -> None:

        self.error = 0; self.lasterror = 0
        self.costf = 0; self.errorsmooth = 0

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
        self.error = 0 if(abs(error) <= self.deadzone) else error # ---> Deadzone for error <--- # 
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
   
class DriveBase_Controller(object):
    def __init__(self, lfmotor: Motor, rgmotor: Motor, syslfmotor: Motor, sysrgmotor: Motor, oneunit: int, scaleunit: int, wheelerror: int = None, distbetweenwheel: int = 0, wheelcircumference: int = 0) -> None:

        # ---> Hub and Gyro sensor initialization <--- #
        self.hub = PrimeHub() 
        self.gyro = self.hub.imu
        self.gyro.reset_heading(0)

        # print(self.hub.battery.voltage()) # Only in practice runs
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
        self.PID = PID_Controller(); self.updatetime: int = 100

        # ---> Transforming Distance <--- #
        self.oneunit = oneunit; self.scaleunit = scaleunit
        
        # ---> Constants for speeds <--- #
        self.minspeed: int = 50; self.maxspeed: int = 1100
        self.mingoodspeed: int = 150; self.maxgoodspeed: int = 1000

        # ---> Moving the robot <--- #
        self.lfspeed: int = 0; self.rgspeed: int = 0
        self.speedlow: int = 0; self.speedhigh: int = 0
        self.lfsign: int = 0; self.rgsign: int = 0

        self.error: int = 0; self.distance: int = 0
        self.angle: int = 0; self.targetangle: int = 0

        self.position: int = 0; 

        # ---> Acceptable error for turns <--- #
        self.epsilon: int = 2
        self.epsilonlowwspeed: int = 32

        # ---> Reset PID flagg <---
        self.lasttfunction: int = 0; 

    async def __initrun__(self) -> None:

        # self.gyro.reset_heading(0); # ---> Trashy function (i think) <--- #
        self.PID.__init__() # ---> Init full PID, but __initvalues__() might also work <--- #
        self.targetangle = int(self.gyro.heading()) # ---> Save state Gyroscope, instead of setting it to 0 <--- #

        await wait(freqreset)

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

    def computespeedmx(self, speed: int) -> int:
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
        
        # ---> Reset PID Values after some time <--- #
        if(self.lasttfunction == 2): self.PID.__initvalues__()

        # ---> Need to implement time and speeding functions (no need for speed functions anymore :DDDD)<---
        self.speed = self.speedlow 

        # ---> Reset relative position and gyro <--- #
        self.lefttmotor.reset_angle(0)
        self.righttmotor.reset_angle(0)

        self.position = 0; self.lasttposition = 0

        await wait(ttimeset); # wait x ms to set values

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

        self.lasttfunction = 1; # gyro forwards

        return None
    
    async def gyrobackwards(self, distance: int, speed1: int, typestop, offsetlf: int = 0, offsetrg: int = 0) -> None:

        if(typestop == None): print("Typestop: NULL\n")

        # ---> Parameters initialization <--- #
        self.distance = 2 * self.getdistance(distance)
        self.speedlow = speed1; # self.speedhigh = speed2

        # ---> Reset PID Values after some time <--- #
        if(self.lasttfunction == 1): self.PID.__initvalues__()

        # ---> Need to implement time and speeding functions <---
        self.speed = self.speedlow 

        # ---> Reset relative position and gyro <--- #
        self.lefttmotor.reset_angle(0)
        self.righttmotor.reset_angle(0)

        self.position = 0; self.lasttposition = 0

        await wait(ttimeset); # wait x ms to set values

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

        self.lasttfunction = 2; # gyro backwards

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

        await wait(ttimeset); # wait x ms to set values

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

        self.lasttfunction = 3; # turn leftt

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

        await wait(ttimeset); # wait x ms to set values

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

        self.lasttfunction = 4; # turn rightt

        return None

# ---> All units of measurement are in milimeters <---
mydrivebase = DriveBase_Controller(
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
    
    async def waituserinput(self): 

        while(1): # ---> wait until a button is pressed <--- #
            pressed = mydrivebase.hub.buttons.pressed()
            if(len(pressed) != 0): return None

            await wait(ttimeset)
        return None; # ---> exit corutine <--- 

    async def runk1(self): 
        
        mydrivebase.PID.setconstants(200, 0, 1); await wait(200)

        # ------------------------------------------------------------------------------------------ #

        mydrivebase.sysrighttmotor.run_angle(250, 25)
        await mydrivebase.gyroforwards(415, 750, typestop = None, offsetlf = 32); await wait(100)
        await mydrivebase.gyroforwards(100, 500, typestop = mydrivebase.braketank, offsetlf = 32)
        await rundegrees(mydrivebase.sysrighttmotor, -250, 1000)
        await mydrivebase.gyrobackwards(500, 875, typestop = mydrivebase.braketank, offsetrg = 32); await wait(250)
        
        # ------------------------------------------------------------------------------------------ #

        return None
    
    async def runk2(self): 
        
        mydrivebase.PID.setconstants(775, 0, 2); await wait(200)
    
        mydrivebase.sysrighttmotor.run_angle(1000, -550); 
        await mydrivebase.gyrobackwards(760, 910, typestop = mydrivebase.braketank); await wait(250)
        
        await mydrivebase.turnleftt(90, 42, lfsign = -1, rgsign = 1, typestop = mydrivebase.braketank); await wait(250)
        
        # await mydrivebase.gyrobackwards(817, 825, typestop = mydrivebase.braketank); await wait(250)
        # await mydrivebase.turnleftt(90, 45, lfsign = 0, rgsign = 1, typestop = mydrivebase.braketank); await wait(250)
        
        # ---> Set PID for smaller speeds <--- #
        mydrivebase.PID.setconstants(675, 0, 1); await wait(200)
        await mydrivebase.gyroforwards(170, 225, typestop = mydrivebase.braketank); await wait(200)
        
        await mydrivebase.sysrighttmotor.run_angle(1000, 200)
        await mydrivebase.sysrighttmotor.run_angle(1000, -200)
    
        await rundegrees(mydrivebase.syslefttmotor, -115 , 400);await wait(200)
        await mydrivebase.gyrobackwards(170,225, typestop = mydrivebase.braketank); await wait(200)
        
        # ---> Reinit PID <--- #
        mydrivebase.PID.setconstants(1125, 0, 2); await wait(200)
        
        await mydrivebase.turnrightt(100,65,lfsign = 1, rgsign = -1, typestop = mydrivebase.braketank); await wait(200)
        await mydrivebase.gyroforwards(800, 1100, typestop = mydrivebase.braketank); await wait(200)

        await rundegrees(mydrivebase.syslefttmotor, 300, 400);await wait (200)

        return None
        
    async def runk3(self):
    
        mydrivebase.PID.setconstants(1250, 0, 2); await wait(250)

        # ------------------------------------------------------------------------------------------ (don't know the state of this) #

        await mydrivebase.gyroforwards(630, 600, typestop = mydrivebase.braketank);await wait(200)
        await mydrivebase.gyrobackwards(70, 700, typestop = mydrivebase.braketank);await wait(200)

        await multitask(
            mydrivebase.gyroforwards(20, 400, typestop=mydrivebase.braketank),
            rundegrees(mydrivebase.syslefttmotor, 200, 200)
        ); await wait(200)

        await rundegrees(mydrivebase.syslefttmotor, -210, 600);await wait(200)
        await rundegrees(mydrivebase.syslefttmotor, 200, 200);await wait(200)

        await mydrivebase.gyroforwards(210, 925, typestop=mydrivebase.braketank);await wait(200)
        await mydrivebase.turnleftt(35, 70, rgsign=0, typestop = mydrivebase.braketank); await wait(200)

        await mydrivebase.gyrobackwards(140, 600, typestop = mydrivebase.braketank); await wait(200)
        await rundegrees(mydrivebase.sysrighttmotor, 170, 400) ; await wait(200)
        

        await mydrivebase.gyroforwards(300, 600, typestop = mydrivebase.braketank);await wait(200)
        await rundegrees(mydrivebase.sysrighttmotor, -170, 500)

        # # ---> Smaller turn speeds (~32 - ~45) and wait(~250 - ~500) after each movement) <---
        await mydrivebase.gyrobackwards(40, 600, typestop = mydrivebase.braketank); await wait(200)
        await mydrivebase.turnrightt(40, 60, lfsign=1, rgsign=-1, typestop=mydrivebase.braketank)
        await mydrivebase.gyrobackwards(640, 1100, typestop=mydrivebase.braketank)

        return None

    async def runk4(self):

        mydrivebase.PID.setconstants(750, 0, 2); await wait(200)

        # ------------------------------------------------------------------------------------------ #

        #await mydrivebase.gyroforwards(125, 425, typestop = mydrivebase.braketank); await wait(250) # with alligning
        await mydrivebase.turnrightt(42, 32, lfsign = 1, rgsign = 0, typestop = mydrivebase.braketank); await wait(250)
        await mydrivebase.gyroforwards(305, 425, typestop = mydrivebase.braketank); await wait(500)

        await mydrivebase.gyrobackwards(205, 625, typestop = mydrivebase.braketank); await wait(250)
        await mydrivebase.turnrightt(42, 32, lfsign = 0, rgsign = -1, typestop = mydrivebase.braketank); await wait(250)

        await mydrivebase.gyroforwards(740, 725, typestop = mydrivebase.braketank); await wait(500)

        await mydrivebase.turnrightt(97, 32, lfsign = 1, rgsign = 0, typestop = mydrivebase.braketank); await wait(250)
        await mydrivebase.gyroforwards(135, 675, typestop = mydrivebase.braketank); await wait(500)

        await mydrivebase.sysrighttmotor.run_angle(1000, -1250); 

        await mydrivebase.gyrobackwards(318, 675, typestop = mydrivebase.braketank); await wait(250)
        await mydrivebase.gyroforwards(100, 1000, typestop = mydrivebase.braketank); await wait(250)

        await mydrivebase.turnleftt(60, 175, lfsign = 0, rgsign = 1, typestop = mydrivebase.braketank); await wait(250)
        await mydrivebase.gyroforwards(700, 1000, typestop = mydrivebase.braketank); await wait(250)
        
        # ------------------------------------------------------------------------------------------ #

        return None

    async def runk5(self): # ---> Stones (needs a lot of tuning with the new sistem) <--- #
        
        mydrivebase.PID.setconstants(925, 0, 2); await wait(200)

        # ------------------------------------------------------------------------------------------ # (finised)

        mydrivebase.syslefttmotor.run_angle(1000, 200)
        await mydrivebase.gyroforwards(150, 675, typestop = mydrivebase.braketank); await wait(200)
        await mydrivebase.turnleftt(30, 30, lfsign = 0, rgsign = 1, typestop = mydrivebase.braketank); await wait(250)
            
        await mydrivebase.gyroforwards(475, 875, typestop = mydrivebase.braketank); await wait(250)
        
        await mydrivebase.turnrightt(71, 38, lfsign = 0, rgsign = -1, typestop = mydrivebase.braketank); await wait(250)
        mydrivebase.syslefttmotor.run_angle(1000, -325); await wait(250) # return to

        await mydrivebase.gyroforwards(225, 925, typestop = mydrivebase.braketank); await wait(200)
        
        await multitask(
            mydrivebase.syslefttmotor.run_angle(1000, 325),
            mydrivebase.sysrighttmotor.run_angle(1000, 1125)
        ); await wait(250)

        # ---> Retrieve to base <--- #
        mydrivebase.syslefttmotor.run_angle(1000, -275); await wait(275) # wait a little bit

        await mydrivebase.gyrobackwards(125, 1100, typestop = mydrivebase.braketank); await wait(250)
        await mydrivebase.turnleftt(75, 75, lfsign = -1, rgsign = 0, typestop = mydrivebase.braketank); await wait(250)
        await mydrivebase.gyrobackwards(650, 1100, typestop = mydrivebase.braketank); await wait(250)

        # ------------------------------------------------------------------------------------------ #

        return None

    async def runk6(self): # ---> misiune distrusa <--- # (swapped with runkx with stones (next one)) 

        mydrivebase.PID.setconstants(1150, 0, 2); await wait(200)

        await multitask(
            mydrivebase.sysrighttmotor.run_angle(1000, -100),
            mydrivebase.syslefttmotor.run_angle(1000, 150)
        ); 

        await self.waituserinput(); await wait(250)

        # ------------------------------------------------------------------------------------------ #  (modif version - faster)

        await mydrivebase.gyroforwards(322, 775, typestop = mydrivebase.braketank); await wait(250)
        await mydrivebase.sysrighttmotor.run_angle(900, -2000); await wait(250)

        await mydrivebase.gyrobackwards(125, 875, typestop = mydrivebase.braketank); await wait(250)
        await mydrivebase.turnleftt(47, 32, lfsign = 0, rgsign = 1, typestop = mydrivebase.braketank); await wait(250)

        # ---> Go to mission <--- #
        await mydrivebase.gyroforwards(224, 875, typestop = mydrivebase.braketank); await wait(250)
        
        await mydrivebase.syslefttmotor.run_angle(1100, -425); # await wait(250)
        
        await mydrivebase.gyrobackwards(64, 875, typestop = mydrivebase.braketank); await wait(250)
        await mydrivebase.gyroforwards(25, 625, typestop = mydrivebase.braketank); await wait(250)
        
        await mydrivebase.syslefttmotor.run_angle(1100, +450)

        await mydrivebase.gyrobackwards(325, 1100, typestop = mydrivebase.braketank); await wait(250)

        # ------------------------------------------------------------------------------------------ #

        return None

    async def runk7(self):
        
        # ---> NEEDED PID INITIALIZATION NOWWWWWWWWWWWWWWWWWWWWW <---

        await rundegrees(mydrivebase.sysrighttmotor, 100, 400)
        await self.waituserinput()
        await mydrivebase.gyroforwards(510, 750, typestop = mydrivebase.stoptank); await wait(200)
        await mydrivebase.turnleftt(60, 30, lfsign = 0, rgsign = 1, typestop = mydrivebase.braketank); await wait(250)
        await mydrivebase.gyroforwards(300, 600, typestop = mydrivebase.braketank); await wait(200)
        await mydrivebase.turnrightt(50, 20, lfsign = 1, rgsign = 0, typestop = mydrivebase.braketank); await wait(250)

        await mydrivebase.gyroforwards(39, 200, typestop = mydrivebase.stoptank); await wait(200)
        await mydrivebase.turnleftt(30, 20, lfsign = 0, rgsign = 1, typestop = mydrivebase.braketank); await wait(250)
        await mydrivebase.gyrobackwards(100, 500, typestop = mydrivebase.braketank); await wait(250)

        await mydrivebase.turnleftt(55, 30, lfsign = 0, rgsign = 1, typestop = mydrivebase.braketank); await wait(250)
        await mydrivebase.gyroforwards(570, 750, typestop = mydrivebase.stoptank); await wait(200)
        await rundegrees(mydrivebase.sysrighttmotor, -200, 1000)
        await mydrivebase.turnleftt(70, 450, lfsign = 0, rgsign = 1, typestop = mydrivebase.braketank); await wait(250)
        return None
    
    # async def runk8(self): return None
    # async def runk9(self): return None

    async def runk11(self):
        mydrivebase.lefttmotor.dc(100)
        mydrivebase.righttmotor.dc(100)
        mydrivebase.syslefttmotor.dc(100)
        mydrivebase.sysrighttmotor.dc(100)

        while(1): await wait(100)

    async def runk12(self):
        mydrivebase.lefttmotor.dc(-100)
        mydrivebase.righttmotor.dc(-100)
        mydrivebase.syslefttmotor.dc(-100)
        mydrivebase.sysrighttmotor.dc(-100)

        while(1): await wait(100)

class runmanager(runmethods):
    def __init__(self): 
        self.runkk: int = 0; 
        self.rundelay: int = 200
        self.maxrun: int = 13

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

        print(runname);

        if(method == None):
            return None

        self.savenumber(runnumber)
        await method()

    def nextrun(self): self.runkk = (1 if(self.runkk >= self.maxrun) else (self.runkk + 1))
    def prevrun(self): self.runkk = (self.maxrun if(self.runkk <= 1) else (self.runkk - 1))

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

                await mydrivebase.__initrun__()
                await self.runtask(self.runkk)
                await wait(self.rundelay)

                mydrivebase.hub.light.on(Color.ORANGE)

            if(Button.LEFT in pressed): self.prevrun(); mydrivebase.hub.display.number(self.runkk); await wait(self.computedelay)
            if(Button.RIGHT in pressed): self.nextrun(); mydrivebase.hub.display.number(self.runkk); await wait(self.computedelay)

            await wait(ttimeset); # wait x ms after each update to have some time to contemplate about some competitive programming problems :D

        return None

taskmanager = runmanager()

async def main() -> None:

    print(":DD - Run k \n"); # Init run K
    # mydrivebase.PID.setconstants(400, 0, 2)

    mydrivebase.hub.system.set_stop_button(Button.BLUETOOTH)
    await taskmanager.computeupdate()

    # timer.reset(); timer.resume()
    # await mydrivebase.turnleft(90, 180)
    # timer.pause(); print("For: 90, 180 -> ", timer.time())

    # timer.reset()
    # ttime = int(timer.time() / 10)
    # print("Run: ", self.runkk, " -> ", ttime, " seconds", sep = "")

    return None

run_task(main())

# https://www.first-lego-league.org/en/2025-26-season/challenge-resources/evaluation
# https://www.first-lego-league.org/en/2025-26-season/challenge-resources/season-documents 
