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
        self.__initvalues__(); self.kp = dkp; self.kd = dkd; self.ki = dki; self.deadzone = deadzone; self.alpha = alphanoise; self.__initvalues__(); return None
    
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

    def computespeedmx(self, speed: int) -> int:
        if(speed == 0): return 0; 

        if(speed < 0): return limitint(speed, -self.maxspeed, -self.minspeed)
        elif(speed > 0): return limitint(speed, self.minspeed, self.maxspeed)

    def computeturnsigns(self, samesign: bool = 0) -> None:
        if(samesign):
            self.lfsign = myabs(self.lfsign) * (-1 if self.getangle() > self.targetangle else 1)
            self.rgsign = myabs(self.rgsign) * (-1 if self.getangle() > self.targetangle else 1)
            return;
        
        self.lfsign = myabs(self.lfsign) * (-1 if self.getangle() > self.targetangle else  1)
        self.rgsign = myabs(self.rgsign) * ( 1 if self.getangle() > self.targetangle else -1)

    # -------------------------> Here comes the big boi (the real and based functions) <------------------------- #

    async def gyroforwards(self, distance: int, speed1: int, typestop, offsetlf: int = 0, offsetrg: int = 0, initpid: int = 0, initgyro: int = 0, typeemsk: int = 0) -> None:

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
    
    async def gyrobackwards(self, distance: int, speed1: int, typestop, offsetlf: int = 0, offsetrg: int = 0, initpid: int = 0, initgyro: int = 0, typeemsk: int = 0) -> None:

        if(typestop == None): print("Typestop: NULL .~.\n")

        stallbrake = (self.braketank if (typestop == None) else typestop)

        # ---> Parameters initialization <--- #
        self.distance = 2 * self.getdistance(distance); 
        self.speedinit = min(self.maxspeed, max(self.minspeed, speed1)); 

        # ---> Reset PID Values after some time <--- #
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

    async def turnleftt(self, angle: int, speedangle: int, *, lfsign: int = -1, rgsign: int = 1, lfdiv: int = 1, rgdiv: int = 1, typestop, lowerspeedbound: int = 45) -> None:

        if(typestop == None): print("Typestop: NULL .~.\n")

        # ---> Recommended speed angle E {75, 225}, angle = 160 <---
        speedangle = limitint(speedangle, lowerspeedbound, __maxint) # ---> Too small -> setpoint ramping = 0 <---
        self.speed = self.computeturnspeed(speedangle) // ((lfsign != 0) + (rgsign != 0))

        # ---> Compute wheels directions <--- #
        angle = -myabs(angle); self.targetangle += angle; 
        self.lfsign = lfsign; self.rgsign = rgsign;
        samesign: bool = bool((lfsign > 0 and rgsign > 0) or (lfsign < 0 and rgsign < 0));

        await wait(__ttimeset); # wait x ms to set values

        # ---> Problem - Not stopping if it overshoots <--- #
        while(not inrange(self.getangle(), self.targetangle - self.epsilon, self.targetangle + self.epsilon)):

            # ---> Change speed to slow down <--- #
            if(inrange(self.getangle(), self.targetangle - self.epsilonlowwspeed, self.targetangle + self.epsilonlowwspeed)):
                self.speed = min(self.speed, self.mingoodspeed)

            self.computeturnsigns(samesign) # Switch turn signs in case of overshooting
            self.lfspeed = self.computespeedmx(self.lfsign * self.speed) // lfdiv
            self.rgspeed = self.computespeedmx(self.rgsign * self.speed) // rgdiv
    
            self.movetank(self.lfspeed, self.rgspeed) 
        
            await wait(self.updatetime)

        if(typestop != None): typestop()

        return None

    async def turnrightt(self, angle: int, speedangle: int, *, lfsign: int = 1, rgsign: int = -1, lfdiv: int = 1, rgdiv: int = 1, typestop, lowerspeedbound: int = 45) -> None:

        if(typestop == None): print("Typestop: NULL .~.\n")

        # ---> Recommended speed angle E {75, 225}, angle = 160 <---
        speedangle = limitint(speedangle, lowerspeedbound, __maxint) # ---> Too small -> setpoint ramping = 0 <---
        self.speed = self.computeturnspeed(speedangle) // ((lfsign != 0) + (rgsign != 0))

        # ---> Compute wheels directions <--- #
        angle = myabs(angle); self.targetangle += angle; 
        self.lfsign = lfsign; self.rgsign = rgsign;       
        samesign: bool = bool((lfsign > 0 and rgsign > 0) or (lfsign < 0 and rgsign < 0));

        await wait(__ttimeset); # wait x ms to set values

        while(not inrange(self.getangle(), self.targetangle - self.epsilon, self.targetangle + self.epsilon)):

            # ---> Change speed to slow down <--- #
            if(inrange(self.getangle(), self.targetangle - self.epsilonlowwspeed, self.targetangle + self.epsilonlowwspeed)):
                self.speed = min(self.speed, self.mingoodspeed)

            self.computeturnsigns(samesign) # Switch turn signs in case of overshooting
            self.lfspeed = self.computespeedmx(self.lfsign * self.speed) // lfdiv
            self.rgspeed = self.computespeedmx(self.rgsign * self.speed) // rgdiv
    
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

# Motor Port A = rg db wheel
# Motor Port B = lf sys wheel
# Motor Port E = lf db sheel
# Motor Port F = rg sys wheel

class runmethods(object):
    def __init__(self): return None
    
    # ---> template: mydrivebase.trytimelimit(lambda: mydrivebase.syslefttmotor.run_angle(1000, 325), 2000, lambda: mydrivebase.sysrighttmotor.hold()) <---

    async def waituserinput(self): 

        while(1): # ---> wait until a button is pressed <--- #
            pressed = mydrivebase.hub.buttons.pressed()
            if(len(pressed) != 0): break; 

            await wait(__ttimeset)
            
        await mydrivebase.__initrun__(initpid = 0);

        return None; # ---> exit corutine <--- 

    async def runk1(self): 
        
        mydrivebase.PID.setconstants(1750, 1200, 1, alphanoise = 1000, deadzone = 0); await wait(200)
        
        # ------------------------------------------------------------------------------------------ #

        mydrivebase.sysrighttmotor.run_angle(250, -25); # -50 init, -325 lift, 625 lower

        await mydrivebase.gyroforwards(12, 1100, typestop = mydrivebase.braketank, initpid = 1); await wait(250)

        # ---> old 728 too low because it hit the mission and it couldn't go through <--- #
        await mydrivebase.gyrobackwards(742, 625, typestop = mydrivebase.braketank, initpid = 1); await wait(250)
        
        mydrivebase.sysrighttmotor.run_angle(750, 475); # -50 init, -325 lift, 625 lower
        await mydrivebase.turnleftt(90, 47, lfsign = -1, rgsign = 1, typestop = mydrivebase.braketank, lowerspeedbound = 45); await wait(250)

        # ---> reached mission <--- #
        await mydrivebase.gyroforwards(165, 225, typestop = mydrivebase.braketank, initpid = 0)
        
        mydrivebase.syslefttmotor.run_angle(1100, -550); # collect
        await mydrivebase.sysrighttmotor.run_angle(300, -425); await wait(250) # -50 init, -325 lift, 625 lower     
    
        await mydrivebase.__initrun__(); # i want to reset the angle to zero (relative angle perpendicular to the mission)
        await mydrivebase.gyrobackwards(105, 975, typestop = mydrivebase.braketank, initpid = 0); await wait(250)
        
        # ---> complete the sharky in the middle <--- #
        await mydrivebase.turnrightt(45, 36, lfsign = 1, rgsign = 0, typestop = mydrivebase.braketank, lowerspeedbound = 32); await wait(250)

        mydrivebase.sysrighttmotor.run_angle(1100, 475); # -50 init, -325 lift, 625 lower
        await mydrivebase.gyroforwards(280, 825, typestop = mydrivebase.braketank, initpid = 1, offsetlf = 55); await wait(250)
        
        await multitask(
            mydrivebase.trytimelimit(lambda: mydrivebase.sysrighttmotor.run_angle(275, -232), 3000, typestop = mydrivebase.sysrighttmotor.hold),# ---> if we have a bad run and can't complete the shark we will get tle and exit :D <--- #
            delayfunction(750, lambda: mydrivebase.trytimelimit(lambda: mydrivebase.turnleftt(20, 32, lfsign = -1, rgsign = 1, typestop = mydrivebase.braketank, lowerspeedbound = 32), 1500, mydrivebase.braketank))
        ); await wait(500); # wait one second to stabilize the shark ;)

        # ---> return to the base <--- #
        await mydrivebase.gyrobackwards(275, 1100, offsetlf = -100, typestop = mydrivebase.braketank, initpid = 0); await wait(250)
        await mydrivebase.turnrightt(75, 45, lfsign = 1, rgsign = -1, typestop = mydrivebase.braketank); await wait(250)

        await mydrivebase.gyroforwards(750, 1100, typestop = mydrivebase.braketank, initpid = 0, offsetrg = -200); await wait(250)

        # ------------------------------------------------------------------------------------------ #
        
        return None
                                                         
    async def runk2(self): 
        
        mydrivebase.PID.setconstants(1750, 0, 2, alphanoise = 1000, deadzone = 0); await wait(200)
        
        # --------------------------------------------------------------------------------------- #

        mydrivebase.syslefttmotor.run_angle(750, -750); # init arm
        
        await multitask( # before dist 715, now 708 (blind modification) #        
            mydrivebase.gyroforwards(695, 675, typestop = mydrivebase.braketank, initpid = 1), 
            delayfunction(500, lambda: mydrivebase.sysrighttmotor.run_angle(1100, -150))
        ); await wait(250)
        
        await mydrivebase.turnleftt(49, 32, lfsign = 0, rgsign = 1, typestop = mydrivebase.braketank); await wait(250);
        
        await mydrivebase.sysrighttmotor.run_angle(1100, +165); await wait(200); # force motor
    
        # ---> go to elements <--- #
        await mydrivebase.gyroforwards(230, 925, typestop = None, initpid = 0); await wait(250); # viata si sufletul meu :| #
        await mydrivebase.sysrighttmotor.run_angle(1100, -150); # lift arm
    
        await mydrivebase.gyrobackwards(215, 475, typestop = mydrivebase.braketank, initpid = 1); await wait(250)   
        await mydrivebase.gyroforwards(150, 675, typestop = mydrivebase.braketank, initpid = 0); await wait(250)

        # await mydrivebase.syslefttmotor.run_angle(400, +400); await wait(250)
        await mydrivebase.trytimelimit(lambda: mydrivebase.syslefttmotor.run_angle(400, 400), 1250, lambda: mydrivebase.syslefttmotor.hold()); await wait(250)

        await mydrivebase.gyrobackwards(5, 250, typestop = mydrivebase.braketank, initpid = 0); await wait(250)   
        await mydrivebase.turnrightt(48, 38, lfsign = 1, rgsign = -1, typestop = mydrivebase.braketank, lowerspeedbound = 38); await wait(250)
        await mydrivebase.gyrobackwards(750, 1100, offsetlf = -175, typestop = mydrivebase.braketank, initpid = 0); await wait(250)   

        # --------------------------------------------------------------------------------------- #

        return None
        
    async def runk3(self):
    
        mydrivebase.PID.setconstants(200, 0, 1); await wait(250)

        # ------------------------------------------------------------------------------------------ #

        mydrivebase.sysrighttmotor.run_angle(250, 30)
        await mydrivebase.gyroforwards(360, 1100, typestop = None, initpid = 1)
        mydrivebase.sysrighttmotor.run_angle(1100, -140)
        await mydrivebase.gyrobackwards(400, 1100, typestop = mydrivebase.braketank, offsetlf = 32, initpid = 0)
        
        return None

    async def runk4(self): 

        mydrivebase.PID.setconstants(2250, 1600, 0, alphanoise = 1000, deadzone = 0); await wait(200)

        # ------------------------------------------------------------------------------------------ #

        await mydrivebase.gyroforwards(70, 525, typestop = mydrivebase.braketank, initpid = 1); await wait(500)
        await mydrivebase.turnrightt(83, 38, lfsign = 0, rgsign = -1, typestop = mydrivebase.braketank, lowerspeedbound = 0); await wait(250)
        await mydrivebase.gyroforwards(825, 825, typestop = mydrivebase.braketank, initpid = 1); await wait(500)

        # ---> go to the mission <--- #
        await mydrivebase.turnrightt(95, 50, lfsign = 1, rgsign = 0, typestop = mydrivebase.braketank); await wait(250)
        
        mydrivebase.sysrighttmotor.run(-275); # rotate with a super small speed to allign even if they don't match :)
        await mydrivebase.gyroforwards(200, 975, typestop = None, initpid = 0); # await wait(500)
        
        await mydrivebase.trytimelimit(lambda: mydrivebase.sysrighttmotor.run_angle(1100, -900), 1000, typestop = mydrivebase.sysrighttmotor.brake()); 
 
        await mydrivebase.gyrobackwards(270, 750, typestop = mydrivebase.braketank, initpid = 0); await wait(250)
        await mydrivebase.gyroforwards(175, 1100, typestop = mydrivebase.braketank, initpid = 0); await wait(250)

        await mydrivebase.turnleftt(60, 60, lfsign = 0, rgsign = 1, typestop = mydrivebase.braketank); await wait(250)
        await mydrivebase.gyroforwards(750, 1100, typestop = mydrivebase.braketank, initpid = 1); await wait(250)

        # ------------------------------------------------------------------------------------------ #

        return None

    async def runk5(self): # ---> Stones (needs a lot of tuning with the new sistem) <--- #
        
        mydrivebase.PID.setconstants(2250, 2500, 4, deadzone = 0, alphanoise = 1000); await wait(200)

        # ------------------------------------------------------------------------------------------ # (finised)

        mydrivebase.syslefttmotor.run_angle(1000, 200)
        await mydrivebase.gyrobackwards(20, 625, typestop = mydrivebase.braketank, initpid = 1); await wait(200)
        await mydrivebase.gyroforwards(210, 675, typestop = mydrivebase.braketank, initpid = 1, initgyro = 1); await wait(200)

        await mydrivebase.turnleftt(20, 30, lfsign = 0, rgsign = 1, typestop = mydrivebase.braketank); await wait(250)
            
        await mydrivebase.gyroforwards(315, 775, typestop = mydrivebase.braketank, initpid = 0); await wait(250)
        
        await mydrivebase.turnrightt(66, 51, lfsign = 1, rgsign = 0, typestop = mydrivebase.braketank); await wait(250)

        mydrivebase.sysrighttmotor.run_angle(1100, 1575); # lower long arm
        mydrivebase.syslefttmotor.run_angle(1000, -325); await wait(250)
        
        # ---> lower long arm while moving forwards <--- #
        await mydrivebase.gyroforwards(250, 280, typestop = mydrivebase.braketank, initpid = 0); await wait(250); 

        mydrivebase.sysrighttmotor.reset_angle(0); # run target for relative angle
        
        await multitask(
            mydrivebase.trytimelimit(lambda: mydrivebase.syslefttmotor.run_angle(1100, 370), 1000, lambda: mydrivebase.sysrighttmotor.hold()),
            mydrivebase.trytimelimit(lambda: mydrivebase.sysrighttmotor.run_target(1100, -1775), 2500, None)
        ); await wait(250)

        mydrivebase.sysrighttmotor.run_target(1100, -1775);

        await mydrivebase.gyrobackwards(125, 625, offsetrg = -100, typestop = mydrivebase.braketank, initpid = 1); await wait(250)

        # ---> Retrieve to base <--- #
        await mydrivebase.turnleftt(60, 75, lfsign = -1, rgsign = 0, typestop = mydrivebase.braketank); await wait(250)
        await mydrivebase.gyrobackwards(650, 1100, offsetrg = -250, typestop = mydrivebase.braketank, initpid = 0); await wait(250)

        # ------------------------------------------------------------------------------------------ #

        return None

    async def runk6(self): # ---> misiune distrusa <--- # (swapped with runkx with stones (next one)) 

        mydrivebase.PID.setconstants(1250, 1750, 4, deadzone = 0, alphanoise = 1000); await wait(200)

        await multitask(
            mydrivebase.trytimelimit(lambda: mydrivebase.sysrighttmotor.run_angle(1100, -190), 1000, lambda: mydrivebase.sysrighttmotor.hold()),
            mydrivebase.trytimelimit(lambda: mydrivebase.syslefttmotor.run_angle(1000, 125), 1000, lambda: mydrivebase.syslefttmotor.hold())
        )

        await self.waituserinput(); await wait(250)

        # ------------------------------------------------------------------------------------------ #  (modif version - faster)

        await mydrivebase.gyrobackwards(10, 1100, typestop = mydrivebase.braketank); await wait(250)

        await mydrivebase.gyroforwards(317, 775, typestop = mydrivebase.braketank, initpid = 1, typeemsk = 3); await wait(250)
        await mydrivebase.trytimelimit(lambda: mydrivebase.sysrighttmotor.run_angle(1100, -4000), 4000, lambda: mydrivebase.sysrighttmotor.hold()); await wait(250);

        await mydrivebase.gyrobackwards(141, 875, typestop = mydrivebase.braketank, initpid = 0); await wait(250)
        await mydrivebase.turnleftt(42, 45, lfsign = 0, rgsign = 1, typestop = mydrivebase.braketank); await wait(250)

        # ---> Go to mission <--- #
        await mydrivebase.gyroforwards(224, 875, typestop = mydrivebase.braketank, initpid = 0); await wait(250)

        await mydrivebase.trytimelimit(lambda: mydrivebase.syslefttmotor.run_angle(1100, -425), 650, lambda: mydrivebase.syslefttmotor.hold())

        await mydrivebase.trytimelimit(lambda: mydrivebase.gyrobackwards(80, 875, typestop = mydrivebase.braketank, initpid = 0), 2000, mydrivebase.braketank); await wait(250)
        await mydrivebase.gyroforwards(25, 625, typestop = mydrivebase.braketank, initpid = 0); await wait(250)
        
        await mydrivebase.trytimelimit(lambda: mydrivebase.syslefttmotor.run_angle(1100, +450), 750, lambda: mydrivebase.syslefttmotor.hold())

        await mydrivebase.gyrobackwards(340, 1100, typestop = mydrivebase.braketank, initpid = 1); await wait(250)

        # ------------------------------------------------------------------------------------------ #

        return None

    async def runk7(self): # e bomba runul asta #
        
        # ---> Transport all elements and put a flag + get minecart :) <---
        mydrivebase.PID.setconstants(2150, 1750, 2, deadzone = 0, alphanoise = 1000); await wait(200); # ~small deadzone, because we need to be really precise with all movements (tested)
                                                                                    
        # ------------------------------------------------------------------------------------------ #
                                                                                    
        # ---> force the transmision closed at max super fast ;) <--- #
        await mydrivebase.trytimelimit(lambda: mydrivebase.sysrighttmotor.run_angle(1100, -590), 375, lambda: mydrivebase.sysrighttmotor.brake()); 

        await mydrivebase.gyrobackwards(25, 750, typestop = mydrivebase.braketank, initpid = 1); await wait(250)
        await mydrivebase.gyroforwards(590, 750, typestop = mydrivebase.braketank, initpid = 1, initgyro = 1); await wait(250)

        # ---> go and get the minecart <--- #
        await mydrivebase.turnleftt(65, 40, lfsign = 0, rgsign = 1, typestop = mydrivebase.braketank, lowerspeedbound = 0); await wait(250)
        
        # ---> go on a curved trajectory <--- # 
        mydrivebase.PID.deadzone = __maxint; await wait(200); 
        await mydrivebase.gyroforwards(400, 675, offsetrg = 48, typestop = mydrivebase.braketank, initpid = 0); await wait(250)
        mydrivebase.PID.deadzone = 0; await wait(200); 

        # ---> collect minecart <--- #
        await mydrivebase.trytimelimit(lambda: mydrivebase.sysrighttmotor.run_angle(1100, 700), 1250, lambda: mydrivebase.sysrighttmotor.hold()); await wait(250)
        await mydrivebase.turnleftt(35, 40, lfsign = 0, rgsign = 1, typestop = mydrivebase.braketank, lowerspeedbound = 0); await wait(250)

        mydrivebase.sysrighttmotor.run_angle(575, -575); await wait(250); # close while moving 
        await mydrivebase.gyroforwards(520, 675, typestop = mydrivebase.braketank, initpid = 0); await wait(250)

        # ------------------------------------------------------------------------------------------ #
        
        await mydrivebase.turnleftt(80, 40, lfsign = 0, rgsign = 1, typestop = mydrivebase.braketank, lowerspeedbound = 0);
        
        await mydrivebase.syslefttmotor.run_angle(660, 660); await wait(500);
        await mydrivebase.trytimelimit(lambda: mydrivebase.sysrighttmotor.run_angle(425, -1100), 2000, lambda: mydrivebase.sysrighttmotor.brake()); 
        
        await mydrivebase.gyroforwards(50, 375, typestop = mydrivebase.braketank, initpid = 0); await wait(250)

        await mydrivebase.gyrobackwards(75, 1100, typestop = mydrivebase.braketank, initpid = 0); await wait(250)

        # ------------------------------------------------------------------------------------------ #

        return None; # Final of run 7

    # ---------------------------------------------------------------------------------------------------------------------------------------------------- #

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

                timer.reset(); timer.resume()

                await mydrivebase.__initrun__()
                await self.runtask(self.runkk)
                await wait(self.rundelay)

                timer.pause(); print("ttime: ", timer.time());

                mydrivebase.stoptank(); # allow them to move freely

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
