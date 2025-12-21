# ---> Written by AndreiDani <---
from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Direction, Stop, Axis, Button, Color
from pybricks.pupdevices import Motor 
from pybricks.tools import wait, run_task, multitask, StopWatch
from micropython import const

# timer = StopWatch() # -> we don't use this frequently <- #
__freqreset = const(256); # miliseconds to wait
__ttimeset = const(64); # miliseconds to wait
__maxint = const(1 << 24); # relative max int 
__constantscale = const(1000); # used in PID constants

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

# ---> Speed Controller to manage easier braking :D <--- #
class Speed_Controller(object):
    def __init__(self, kk: int = 0) -> None:    
    
        self.kk: int = kk; 
        self.msk: int = 0; # Type of function

        self.middle: int = 0; # Half of xx2

        return None

    def makefunction(self, xx: float = 0, oneminus: int = 0):
        if(oneminus): xx = (1 - xx); # -> Second half <- #
        return max(0, min(1, self.kk * xx)); # -> Compute function <- #

    def computefunction(self, xx1: int = 0, xx2: int = 1, scalevalue: int = 0) -> int:

        xx1 = myabs(xx1); # If we have negative it boom

        if(xx1 > xx2 or xx2 == 0): return 0; # ---> Invalid parameters <--- #

        xx: float = float(xx1) / float(xx2); # >>>> For now we convert <<<< #

        if(xx <= self.middle):
            return int(scalevalue * self.makefunction(xx, oneminus = 0)) if(self.msk & 1) else (scalevalue); 
        return int(scalevalue * self.makefunction(xx, oneminus = 1)) if(self.msk & 2) else (scalevalue); 

# ---> Proportional - Integral - Derivative Controller <--- #
class PID_Controller(object): 
    def __init__(self) -> None: 

        # ---> Base variables (int) + __constantscale = 1000 used by kp, ki, kd <--- #
        self.kp: int = 0; self.kd: int = 0; self.ki: int = 0

        self.error: int = 0; self.lasterror: int = 0; self.dt: int = 10
        self.proportional: int = 0; self.derivative: int = 0; self.integral: int = 0

        # ---> Variables for modifications to the PID controller <---
        self.maxchange: int = 50 # ---> Max error change <---
        self.integralwindup = __maxint # ---> Integral windup <---
        
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

    def setconstants(self, dkp: int, dkd: int, dki: int, deadzone: int = 5) -> None:
        self.__initvalues__(); self.kp = dkp; self.kd = dkd; self.ki = dki; self.deadzone = deadzone; return None
    
    def noisefiltering(self) -> None:
        self.errorsmooth = (self.alpha * self.error + (__constantscale - self.alpha) * self.errorsmooth) // __constantscale
        self.error = self.errorsmooth # ---> Reassign filtered error to raw error to be applied in the controller <--- 

    def compute(self, error: int, sign: int) -> None:
        
        # ---> Limiting the error to be in range of [lasterror +- maxchange] <--- #
        self.error = 0 if(abs(error) <= self.deadzone) else error # ---> Deadzone for error <--- # 
        self.error = limitint(self.error, self.lasterror - self.maxchange, self.lasterror + self.maxchange)

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
        ) // __constantscale * sign

        self.lasterror = self.error # ---> Changing last error <--- #

        return; # >>> finish computing controlleroutput <<< #
   
class DriveBase_Controller(object):
    def __init__(self, lfmotor: Motor, rgmotor: Motor, syslfmotor: Motor, sysrgmotor: Motor, oneunit: int, scaleunit: int, wheelerror: int = None, syswheelerror: int = None, signencoder: int = 1, distbetweenwheel: int = 0, wheelcircumference: int = 0) -> None:

        # ---> Hub and Gyro sensor initialization <--- #
        self.hub = PrimeHub() 
        self.gyro = self.hub.imu
        self.gyro.reset_heading(0)

        print(self.hub.battery.voltage()) # Only in practice runs
        if(self.hub.battery.voltage() <= 7500):
            print("Failed to initialize - Battery too low\n")

        # ---> Drivebase motors initializations <--- #
        self.lefttmotor: Motor = lfmotor
        self.righttmotor: Motor = rgmotor

        self.syslefttmotor: Motor = syslfmotor
        self.sysrighttmotor: Motor = sysrgmotor

        self.lefttmotor.control.target_tolerances(None, wheelerror)
        self.righttmotor.control.target_tolerances(None, wheelerror)

        if(self.syslefttmotor != None): self.syslefttmotor.control.target_tolerances(None, syswheelerror)
        if(self.sysrighttmotor != None): self.sysrighttmotor.control.target_tolerances(None, syswheelerror)

        # ---> Formulas used for calculating the speed for turning <--- #
        self.distwheels: int = distbetweenwheel
        self.circumferencerobot: int = int(2 * 314 * distbetweenwheel // 100) 
        self.wheelcircumference = int(wheelcircumference)

        self.signencoderwheels: int = signencoder

        # ---> Object controller for swaps in programs <--- #
        self.PID = PID_Controller(); self.updatetime: int = 100

        # ---> Transforming Distance <--- #
        self.oneunit = oneunit; self.scaleunit = scaleunit
        
        # ---> Constants for speeds <--- #
        self.minspeed: int = const(50); self.maxspeed: int = const(1100)
        self.mingoodspeed: int = const(150); self.maxgoodspeed: int = const(1000)

        # ---> Values used by speed controller <--- #
        self.speedcontroller = Speed_Controller(); 
        self.speedcontroller.kk = 4; # >> Slope of function << #
        self.goodbrakingspeed: int = const(250); 
        self.previousposition: int; # -> get xx1 <- #
        self.diffspeed: int; # -> get diff to compute speed <- #

        # ---> Moving the robot <--- #
        self.lfspeed: int = 0; self.rgspeed: int = 0
        self.speedlow: int = 0; self.speedhigh: int = 0
        self.lfsign: int = 0; self.rgsign: int = 0

        self.error: int = 0; self.distance: int = 0
        self.angle: int = 0; self.targetangle: int = 0

        self.position: int = 0; self.targetposition: int = 0

        # ---> Acceptable error for turns <--- #
        self.epsilon: int = 2
        self.epsilonlowwspeed: int = 32

    async def __initrun__(self) -> None:

        # ---> Reinit PID class and trick to reset gyro and position of the wheels <--- #
        self.PID.__init__(); self.targetangle = int(self.gyro.heading()) 
        self.lefttmotor.reset_angle(0); self.righttmotor.reset_angle(0)

        # ---> Reinit constant values <--- #
        self.epsilon = 2; self.epsilonlowwspeed = 32

        await wait(__freqreset); return None; # >>> return <<< #

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
    def getmotorsangle(self) -> int: return int(self.signencoderwheels * (self.lefttmotor.angle() + self.righttmotor.angle()))
    def getangle(self) -> int: return int(self.gyro.heading())

    def computeturnspeed(self, speedangle: int) -> int:
        # ---> Formulas: https://www.desmos.com/calculator/ww29qyn3yt <---
        speed = (self.circumferencerobot * speedangle // 360) 
        speed = self.getdistance(speed)
        return int(speed)

    def computespeedmx(self, speed: int) -> int:
        if(speed == 0): return 0; 

        if(speed < 0): return limitint(speed, -self.maxspeed, -self.minspeed)
        elif(speed > 0): return limitint(speed, self.minspeed, self.maxspeed)

    def computeturnsigns(self) -> None:
        self.lfsign = myabs(self.lfsign) * (-1 if self.getangle() > self.targetangle else  1)
        self.rgsign = myabs(self.rgsign) * ( 1 if self.getangle() > self.targetangle else -1)

    # -------------------------> Here comes the big boi (the real and based functions) <------------------------- #

    async def gyroforwards(self, distance: int, speed1: int, typestop, offsetlf: int = 0, offsetrg: int = 0, initpid: int = 0, typeemsk: int = 0) -> None:

        if(typestop == None): print("Typestop: NULL .~.\n")

        stallbrake = (self.braketank if (typestop == None) else typestop)

        # ---> Parameters initialization <--- #
        self.distance = 2 * self.getdistance(distance)

        self.speedhigh = min(self.maxspeed, speed1); 
        self.speedlow = min(self.speedhigh, max((self.speedhigh // 2), self.goodbrakingspeed)); 

        self.diffspeed = self.speedhigh - self.speedlow

        # ---> Reset PID Values after some time / after alligning to a mission <--- #
        if(initpid): self.PID.__initvalues__()

        # ---> These are used for speed controllers <---
        self.speed = self.speedlow 
        self.speedcontroller.msk = typeemsk 

        # ---> Reset relative position and gyro (tricks) <--- #
        self.position = self.getmotorsangle(); 
        self.previousposition = self.position; 
        self.targetposition = self.position + self.distance; 

        print("Typee move: ", typeemsk, " -> ", (typeemsk & 2), (typeemsk & 1), sep = "")

        await wait(__ttimeset); # wait x ms to set values

        while(self.position < self.targetposition):

            self.error = self.targetangle - self.getangle()
            self.PID.compute(self.error, 1); # >>> ahh sign (need +1) <<< #

            # print(self.error, self.PID.error)

            self.speed = self.speedlow + self.speedcontroller.computefunction(myabs(self.position - self.previousposition), self.distance, self.diffspeed); 

            print("Loop: ", myabs(self.position - self.previousposition), " -> ", self.distance, " | ", self.speed, sep = "");

            self.lfspeed = limitint(self.speed + self.PID.controlleroutput + offsetlf, self.minspeed, self.maxspeed)
            self.rgspeed = limitint(self.speed - self.PID.controlleroutput + offsetrg, self.minspeed, self.maxspeed)

            self.movetank(self.lfspeed, self.rgspeed)
            await wait(self.updatetime)

            if(self.stalledtank()): stallbrake(); return; 
            self.position = self.getmotorsangle()
            
        if(typestop != None): typestop()

        return None
    
    async def gyrobackwards(self, distance: int, speed1: int, typestop, offsetlf: int = 0, offsetrg: int = 0, initpid: int = 0, typeemsk: int = 0) -> None:

        if(typestop == None): print("Typestop: NULL .~.\n")

        stallbrake = (self.braketank if (typestop == None) else typestop)

        # ---> Parameters initialization <--- #
        self.distance = 2 * self.getdistance(distance)
        
        self.speedhigh = min(self.maxspeed, speed1); 
        self.speedlow = min(self.speedhigh, max((self.speedhigh // 2), self.goodbrakingspeed)); 

        self.diffspeed = self.speedhigh - self.speedlow

        # ---> Reset PID Values after some time <--- #
        if(initpid): self.PID.__initvalues__()

        # ---> These are used for speed controllers <---
        self.speed = self.speedlow 
        self.speedcontroller.msk = typeemsk 

        # ---> Reset relative position and gyro (tricks) <--- #
        self.position = self.getmotorsangle(); 
        self.previousposition = self.position; 
        self.targetposition = self.position - self.distance; 

        print("Typee move: ", typeemsk, " -> ", (typeemsk & 2), (typeemsk & 1), sep = "")

        await wait(__ttimeset); # wait x ms to set values

        while(self.position > self.targetposition):

            self.error = self.targetangle - self.getangle()
            self.PID.compute(self.error, -1); # >>> ahh sign (need -1) <<< #

            # print(self.error, self.PID.error)

            self.speed = self.speedlow + self.speedcontroller.computefunction(myabs(self.position - self.previousposition), self.distance, self.diffspeed); 

            print("Loop: ", myabs(self.position - self.previousposition), " -> ", self.distance, " | ", self.speed, sep = "");

            self.lfspeed = -limitint(self.speed + self.PID.controlleroutput + offsetlf, self.minspeed, self.maxspeed)
            self.rgspeed = -limitint(self.speed - self.PID.controlleroutput + offsetrg, self.minspeed, self.maxspeed)

            self.movetank(self.lfspeed, self.rgspeed)
            await wait(self.updatetime)

            if(self.stalledtank()): stallbrake(); return; 
            self.position = self.getmotorsangle()
            
        if(typestop != None): typestop()

        return None

    async def turnleftt(self, angle: int, speedangle: int, *, lfsign: int = -1, rgsign: int = 1, typestop) -> None:

        if(typestop == None): print("Typestop: NULL .~.\n")

        # ---> Recommended speed angle E {75, 225}, angle = 160 <---
        speedangle = limitint(speedangle, 45, 360) # ---> Too small -> setpoint ramping = 0 <---
        self.speed = self.computeturnspeed(speedangle) // ((lfsign != 0) + (rgsign != 0))

        # ---> Compute wheels directions <--- #
        angle = -myabs(angle)
        self.lfsign = min(lfsign, 0)
        self.rgsign = max(rgsign, 0)

        self.targetangle += angle

        await wait(__ttimeset); # wait x ms to set values

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

        if(typestop == None): print("Typestop: NULL .~.\n")

        # ---> Recommended speed angle E {75, 225}, angle = 160 <---
        speedangle = limitint(speedangle, 45, 360) # ---> Too small -> setpoint ramping = 0 <---
        self.speed = self.computeturnspeed(speedangle) // ((lfsign != 0) + (rgsign != 0))

        # ---> Compute wheels directions <--- #
        angle = myabs(angle); 
        self.lfsign = max(lfsign, 0)
        self.rgsign = min(rgsign, 0)

        self.targetangle += angle

        await wait(__ttimeset); # wait x ms to set values

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
# mydrivebase = DriveBase_Controller(
#     Motor(Port.E, Direction.COUNTERCLOCKWISE), 
#     Motor(Port.A, Direction.CLOCKWISE), 
#     Motor(Port.B, Direction.CLOCKWISE), 
#     Motor(Port.F, Direction.COUNTERCLOCKWISE), 
#     2316, 1000, 0, syswheelerror = 5, distbetweenwheel = 80,
#     wheelcircumference = 49
# )

# My drivebase with two spike prime wheels 
mydrivebase = DriveBase_Controller(
    Motor(Port.A, Direction.COUNTERCLOCKWISE), 
    Motor(Port.C, Direction.CLOCKWISE),
    None, None, # we don't have sys motors and we don't call them 
    2045, 1000, 0, syswheelerror = 5, signencoder = 1,
    distbetweenwheel = 112, wheelcircumference = 176
)

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

            await wait(__ttimeset)
        return None; # ---> exit corutine <--- 

    async def runk1(self) -> None:

        mydrivebase.PID.setconstants(750, 0, 2); await wait(250); # ---> Init PID values <--- #

        await mydrivebase.gyroforwards(250, 625, typestop = mydrivebase.braketank, initpid = 1, typeemsk = 0); await wait(250)
        await mydrivebase.gyroforwards(250, 625, typestop = mydrivebase.braketank, initpid = 1, typeemsk = 1); await wait(250)
        await mydrivebase.gyroforwards(250, 625, typestop = mydrivebase.braketank, initpid = 1, typeemsk = 2); await wait(250)
        await mydrivebase.gyroforwards(250, 625, typestop = mydrivebase.braketank, initpid = 1, typeemsk = 3); await wait(250)
        await mydrivebase.gyrobackwards(250, 625, typestop = mydrivebase.braketank, initpid = 0, typeemsk = 0); await wait(250)

        return None

class runmanager(runmethods):
    def __init__(self): 
        self.runkk: int = 0; 
        self.rundelay: int = 200
        self.maxrun: int = 13

        self.computedelay: int = 250

        return None

    # ---> Modify local memory (bytes) to save the lastt idx <--- #
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

        print(mydrivebase.hub.battery.voltage(), " -> ", runname, sep = "", end = "^~^ \n"); 

        if(method == None): return None

        self.savenumber(runnumber)
        await method(); return None; 

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

            await wait(__ttimeset); # wait x ms after each update to have some time to contemplate about some competitive programming problems :D

        return None

taskmanager = runmanager()

async def main() -> None:

    print(":DD - Run k \n"); # Init run K
    # -> mydrivebase.PID.setconstants(kp, ki, kd, dz) <- #

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
