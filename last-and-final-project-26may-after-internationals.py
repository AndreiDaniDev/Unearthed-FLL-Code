# ---> Written by AndreiDani <---
from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Direction, Stop, Axis, Button, Color
from pybricks.pupdevices import Motor 
from pybricks.tools import wait, run_task, multitask, StopWatch
from micropython import const

timer = StopWatch() # -> we don't use this frequently <- #
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

async def waituntilstall(mymotor: Motor) -> None:
    while(not mymotor.stalled()): await wait(__ttimeset)
    return None; # ---> Finish corutine <--- #

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

    def setconstants(self, dkp: int, dkd: int, dki: int, deadzone: int = 5, alphanoise: int = 625) -> None:
        self.kp = dkp; self.kd = dkd; self.ki = dki; self.deadzone = deadzone; self.alpha = alphanoise; self.__initvalues__(); return None
    
    def noisefiltering(self) -> None:
        self.errorsmooth = (self.alpha * self.error + (__constantscale - self.alpha) * self.errorsmooth) // __constantscale
        self.error = self.errorsmooth # ---> Reassign filtered error to raw error to be applied in the controller <--- 

    def compute(self, error: int, sign: int) -> None:
        
        # ---> Limiting the error to be in range of [lasterror +- maxchange] <--- #
        self.error = 0 if(abs(error) <= self.deadzone) else error # ---> Deadzone for error <--- # 
        self.error = limitint(self.error, self.lasterror - self.maxchange, self.lasterror + self.maxchange)

        self.noisefiltering() # ---> Filter errors and noise from the input <---
        # self.gradientascent() # ---> Tune the constants according to the gradient ascent algorithm <--- (nah)

        self.proportional = self.error
        self.derivative = (self.error - self.lasterror)

        self.integral += self.dt * self.error
        self.integral = limitint(self.integral, -self.integralwindup, self.integralwindup)

        self.controlleroutput = int(
            self.proportional * self.kp + 
            self.derivative * self.kd // self.dt + 
            self.integral * self.ki
        ) // __constantscale * sign

        self.lasterror = self.error # ---> Changing last error <--- #

        return; # >>> finish computing controlleroutput <<< #
   
class DriveBase_Controller(object):
    def __init__(self, lfmotor: Motor, rgmotor: Motor, syslfmotor: Motor, sysrgmotor: Motor, oneunit: int, scaleunit: int, wheelerror: int = None, syswheelerror: int = None, stallspeed: int = 50, ttimestall: int = 425, signencoder: int = 1, distbetweenwheel: int = 0, wheelcircumference: int = 0) -> None:

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

        if(self.syslefttmotor != None): 
            self.syslefttmotor.control.target_tolerances(None, syswheelerror)
            self.syslefttmotor.control.stall_tolerances(stallspeed, ttimestall)
        if(self.sysrighttmotor != None): 
            self.sysrighttmotor.control.target_tolerances(None, syswheelerror)
            self.sysrighttmotor.control.stall_tolerances(stallspeed, ttimestall)

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
        self.goodbrakingspeed: int = const(250); 
        self.previousposition: int; # -> get xx1 <- #

        # ---> Moving the robot <--- #
        self.lfspeed: int = 0; self.rgspeed: int = 0
        self.speedinit: int = 0; 
        self.lfsign: int = 0; self.rgsign: int = 0

        self.error: int = 0; self.distance: int = 0
        self.angle: int = 0; self.targetangle: int = 0

        self.position: int = 0; self.targetposition: int = 0

        # ---> Acceptable error for turns <--- #
        self.epsilon: int = 2
        self.epsilonlowwspeed: int = 32

    async def __initrun__(self, initpid: int = 1) -> None:

        # ---> Reinit PID class and trick to reset gyro and position of the wheels <--- #
        if(initpid): self.PID.__init__(); # ---> made with flag in case of wait for user input <--- # 
        self.targetangle = int(self.gyro.heading()) 
        self.lefttmotor.reset_angle(0); self.righttmotor.reset_angle(0)

        # ---> Reinit constant values <--- #
        self.epsilon = 2; self.epsilonlowwspeed = 32

        await wait(__freqreset); return None; # >>> return <<< #

    # ---> Pair motor functions (Move tank and stop) <--- #
    def movetank(self, speedlf: int, speedrg: int) -> None:
        self.lefttmotor.run(speedlf)
        self.righttmotor.run(speedrg)
        return None

    # ---> Stop tank functions <---
    def stoptank(self)  -> None: self.lefttmotor.stop();  self.righttmotor.stop();  return None
    def braketank(self) -> None: self.lefttmotor.brake(); self.righttmotor.brake(); return None
    def holdtank(self)  -> None: self.lefttmotor.hold();  self.righttmotor.hold();  return None
    def stalledtank(self) -> int: return (self.lefttmotor.stalled() & self.righttmotor.stalled()) 

    # ---> Helper functions for travelling distances <--- #
    def getdistance(self, dist: int) -> int: return int(dist * self.oneunit // self.scaleunit)
    def getmotorsangle(self) -> int: return int(self.signencoderwheels * (self.lefttmotor.angle() + self.righttmotor.angle()))
    def getangle(self) -> int: return int(self.gyro.heading())

    def computeturnspeed(self, speedangle: int) -> int:
        # ---> Formulas: https://www.desmos.com/calculator/ww29qyn3yt <---
        speed = (self.circumferencerobot * speedangle // 360) 
        speed = self.getdistance(speed)
        return int(speed)
    
    def computeturnsigns(self) -> None:    
        self.lfsign = myabs(self.lfsign) * (-1 if self.getangle() > self.targetangle else  1)
        self.rgsign = myabs(self.rgsign) * ( 1 if self.getangle() > self.targetangle else -1)

    # -------------------------> Here comes the big boi (the real and based functions) <------------------------- #

    async def gyroforwards(self, distance: int, speed1: int, typestop, offsetlf: int = 0, offsetrg: int = 0, initpid: int = 0, initgyro: int = 0) -> None:

        if(typestop == None): print("Typestop: NULL .~.\n")

        stallbrake = (self.braketank if (typestop == None) else typestop)

        # ---> Parameters initialization <--- #
        self.distance = 2 * self.getdistance(distance); 
        self.speedinit = min(self.maxspeed, max(self.minspeed, speed1)); 

        # ---> Reset PID Values after some time / after alligning to a mission <--- #
        if(initpid): self.PID.__initvalues__()
        if(initgyro): self.targetangle = int(self.gyro.heading()) 

        # ---> Reset relative position and gyro (tricks) <--- #
        self.position = self.getmotorsangle(); 
        self.previousposition = self.position; 
        self.targetposition = self.position + self.distance; 

        await wait(__ttimeset); # wait x ms to set values

        while(self.position < self.targetposition):

            self.error = self.targetangle - self.getangle()
            self.PID.compute(self.error, 1); # >>> ahh sign (need +1) <<< #

            self.lfspeed = limitint(self.speedinit + self.PID.controlleroutput + offsetlf, self.minspeed, self.maxspeed)
            self.rgspeed = limitint(self.speedinit - self.PID.controlleroutput + offsetrg, self.minspeed, self.maxspeed)

            self.movetank(self.lfspeed, self.rgspeed)
            await wait(self.updatetime)

            if(self.stalledtank()): stallbrake(); return; 
            self.position = self.getmotorsangle()
            
        if(typestop != None): typestop()

        return None
    
    async def gyrobackwards(self, distance: int, speed1: int, typestop, offsetlf: int = 0, offsetrg: int = 0, initpid: int = 0, initgyro: int = 0) -> None:

        if(typestop == None): print("Typestop: NULL .~.\n")

        stallbrake = (self.braketank if (typestop == None) else typestop)

        # ---> Parameters initialization <--- #
        self.distance = 2 * self.getdistance(distance); 
        self.speedinit = min(self.maxspeed, max(self.minspeed, speed1)); 
        
        # ---> Reset PID Values after some time / after alligning to a mission <--- #
        if(initpid): self.PID.__initvalues__()
        if(initgyro): self.targetangle = int(self.gyro.heading())

        # ---> Reset relative position and gyro (tricks) <--- #
        self.position = self.getmotorsangle(); 
        self.previousposition = self.position; 
        self.targetposition = self.position - self.distance; 

        await wait(__ttimeset); # wait x ms to set values

        while(self.position > self.targetposition):

            self.error = self.targetangle - self.getangle()
            self.PID.compute(self.error, -1); # >>> ahh sign (need -1) <<< #

            self.lfspeed = -limitint(self.speedinit + self.PID.controlleroutput + offsetlf, self.minspeed, self.maxspeed)
            self.rgspeed = -limitint(self.speedinit - self.PID.controlleroutput + offsetrg, self.minspeed, self.maxspeed)

            self.movetank(self.lfspeed, self.rgspeed)
            await wait(self.updatetime)

            if(self.stalledtank()): stallbrake(); return; 
            self.position = self.getmotorsangle()
            
        if(typestop != None): typestop()

        return None

    async def turnleftt(self, angle: int, speedangle: int, *, lfsign: int = -1, rgsign: int = 1, typestop) -> None:

        if(typestop == None): print("Typestop: NULL .~.\n")

        # ---> Recommended speed angle E {75, 225}, angle = 160 (if we have speed angle we can use this for setpoint ramping - different concept for pid) <---
        self.speed = self.computeturnspeed(speedangle) // ((lfsign != 0) + (rgsign != 0))
        self.speed = limitint(self.speed, self.minspeed, self.maxspeed); 

        # ---> Compute wheels directions <--- #
        angle = -myabs(angle); self.targetangle += angle; 
        self.lfsign = lfsign; self.rgsign = rgsign; 

        await wait(__ttimeset); # wait x ms to set values

        # ---> Problem - Not stopping if it overshoots <--- #
        while(not inrange(self.getangle(), self.targetangle - self.epsilon, self.targetangle + self.epsilon)):

            if(inrange(self.getangle(), self.targetangle - self.epsilonlowwspeed, self.targetangle + self.epsilonlowwspeed)):
                self.speed = min(self.speed, self.mingoodspeed); # simple for improving precision #

            self.computeturnsigns() # Switch turn signs in case of overshooting
            self.lfspeed = self.lfsign * self.speed; 
            self.rgspeed = self.rgsign * self.speed; 
    
            self.movetank(self.lfspeed, self.rgspeed) 
        
            await wait(self.updatetime)

        if(typestop != None): typestop()

        return None

    async def turnrightt(self, angle: int, speedangle: int, *, lfsign: int = 1, rgsign: int = -1, typestop) -> None:

        if(typestop == None): print("Typestop: NULL .~.\n")

        # ---> Recommended speed angle E {75, 225}, angle = 160 <---
        self.speed = self.computeturnspeed(speedangle) // ((lfsign != 0) + (rgsign != 0))
        self.speed = limitint(self.speed, self.minspeed, self.maxspeed); 

        # ---> Compute wheels directions <--- #
        angle = myabs(angle); self.targetangle += angle; 
        self.lfsign = lfsign; self.rgsign = rgsign;       

        await wait(__ttimeset); # wait x ms to set values

        while(not inrange(self.getangle(), self.targetangle - self.epsilon, self.targetangle + self.epsilon)):

            if(inrange(self.getangle(), self.targetangle - self.epsilonlowwspeed, self.targetangle + self.epsilonlowwspeed)):
                self.speed = min(self.speed, self.mingoodspeed); # simple for improving precision #

            self.computeturnsigns() # Switch turn signs in case of overshooting
            self.lfspeed = self.lfsign * self.speed; 
            self.rgspeed = self.rgsign * self.speed; 
    
            self.movetank(self.lfspeed, self.rgspeed) 
        
            await wait(self.updatetime)

        if(typestop != None): typestop()

        return None

    async def trytimelimit(self, func, ttime: int, typestop) -> None:

        if(func == None):  print("# invalid tle try #"); return None;

        await multitask(func(), wait(ttime), race = True);

        if(typestop != None): typestop(); # ---> stop if we got time limit exceeded <--- #

        return None; 

# ---> All units of measurement are in milimeters <---
mydrivebase = DriveBase_Controller(
    Motor(Port.E, Direction.COUNTERCLOCKWISE), 
    Motor(Port.A, Direction.CLOCKWISE), 
    Motor(Port.B, Direction.CLOCKWISE), 
    Motor(Port.F, Direction.COUNTERCLOCKWISE), 
    2316, 1000, wheelerror = 0, syswheelerror = 5, 
    stallspeed = 20, ttimestall = 100, signencoder = 1,
    distbetweenwheel = 80, wheelcircumference = 49
)

# My drivebase with two spike prime wheels 
# mydrivebase = DriveBase_Controller(
#     Motor(Port.A, Direction.COUNTERCLOCKWISE), 
#     Motor(Port.C, Direction.CLOCKWISE),
#     None, None, # we don't have sys motors and we don't call them 
#     2045, 1000, 0, syswheelerror = 5, signencoder = 1,
#     distbetweenwheel = 112, wheelcircumference = 176
# )

class runmethods(object):
    def __init__(self): return None
    
    # ---> template: mydrivebase.trytimelimit(lambda: mydrivebase.sysmotorr.run_angle(-, -), ttimelimit, lambda: mydrivebase.sysmotorr.hold()) <---

    async def waituserinput(self): 

        while(1): # ---> wait until a button is pressed <--- #
            pressed = mydrivebase.hub.buttons.pressed()
            if(len(pressed) != 0): break; 

            await wait(__ttimeset)
            
        await mydrivebase.__initrun__(initpid = 0);

        return None; # ---> exit corutine <--- 

    async def runk1(self): 
        
        # ---> all parameters are explained in the tehnical notebook <--- #
        # mydrivebase.PID.setconstants(kp, kd, ki, alphanoise = -, deadzone = -); await wait(200)
        
        # ------------------------------------------------------------------------------------------ #

        

        # ------------------------------------------------------------------------------------------ #
        
        return None

    # ---------------------------------------------------------------------------------- #

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
        self.maxrun: int = 12

        self.changerundelay: int = 250

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

                timer.reset(); timer.resume()

                await mydrivebase.__initrun__()
                await self.runtask(self.runkk)
                await wait(self.rundelay)

                timer.pause(); print("ttime: ", timer.time());

                mydrivebase.stoptank(); # allow them to move freely

                mydrivebase.hub.light.on(Color.ORANGE)

            if(Button.LEFT in pressed): self.prevrun(); mydrivebase.hub.display.number(self.runkk); await wait(self.changerundelay)
            if(Button.RIGHT in pressed): self.nextrun(); mydrivebase.hub.display.number(self.runkk); await wait(self.changerundelay)

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
