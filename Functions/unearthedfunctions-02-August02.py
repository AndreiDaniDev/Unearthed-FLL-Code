# ---> Written by AndreiDani <---
from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Direction, Stop, Axis, Button
from pybricks.pupdevices import Motor 
from pybricks.tools import wait, run_task, multitask, StopWatch
from pybricks.robotics import DriveBase

timer = StopWatch()

# ---> Max value for int = 2 ^ 31 - 1 = 1073741823 <--- #
def limitint(value: int, valuemin: int, valuemax: int) -> int:
    return min(max(value, valuemin), valuemax)

def inlimit(value: int, valuemin: int, valuemax: int) -> int:
    return (valuemin <= value and value <= valuemax)

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
        self.alpha: int = 1000 # ---> alpha constant (int -> cstscale) <---

        self.deadzone: int = 5 # ---> If error <= deadzone -> error = 0 <---

        return None

    def setconstants(self, dkp: int, dkd: int, dki: int) -> None:
        self.kp = dkp 
        self.kd = dkd
        self.ki = dki
        return None
    
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

        self.controlleroutput = (
            self.proportional * self.kp + 
            self.derivative * self.kd / self.dt + 
            self.integral * self.ki
        ) // self.cstscale * sign

        self.lasterror = self.error # ---> Changing last error <--- #

        return 
   
class DriveBase_Contorller(object):
    def __init__(self, lfmotor: Motor, rgmotor: Motor, oneunit: int, scaleunit: int, wheelerror: int = None, distbetweenwheel: int = 0, wheelcircumference: int = 0) -> None:

        # ---> Hub and Gyro sensor initialization <--- #
        self.hub = PrimeHub() 
        self.gyro = self.hub.imu
        self.gyro.reset_heading(0)

        if(self.hub.battery.voltage() <= 7500):
            print("Failed to initialize - Battery too low\n")

        # ---> Drivebase motors initializations <--- #
        self.lefttmotor: Motor = lfmotor
        self.righttmotor: Motor = rgmotor

        self.lefttmotor.control.target_tolerances(None, wheelerror)
        self.righttmotor.control.target_tolerances(None, wheelerror)

        # ---> Formulas used for calculating the speed for turning <--- #
        self.distwheels: int = distbetweenwheel
        self.circumferencerobot: int = int(2 * 314 * distbetweenwheel // 100)
        self.wheelcircumference = int(wheelcircumference)

        # ---> Object controller for swaps in programs <--- #
        self.PID = PID_Controller(); 
        self.PID_turn = PID_Controller(); 
        self.updatetime: int = 25

        # ---> Transforming Distance <--- #
        self.oneunit = oneunit; self.scaleunit = scaleunit

        # ---> Moving the robot <--- #
        self.lfspeed: int = 0; self.rgspeed: int = 0
        self.speedlow: int = 0; self.speedhigh: int = 0

        self.minspeed: int = 200; self.maxspeed: int = 1100

        self.error: int = 0; self.distance: int = 0

        self.angle: int = 0
        self.targetangle: int = 0

        self.position: int = 0
        self.lasttposition: int = 0

        self.setpoint: int = 0
        self.setpointramping: int = 0

        # ---> Acceptable error for turns <--- #
        self.epsilon: int = 5

    # ---> Pair motor functions (Move tank and stop) <--- #
    def movetank(self, speedlf: int, speedrg: int) -> None:
        self.lefttmotor.run(speedlf)
        self.righttmotor.run(speedrg)

    # ---> Stop tank functions <---
    def stoptank(self)  -> None: self.lefttmotor.stop();  self.righttmotor.stop()
    def braketank(self) -> None: self.lefttmotor.brake(); self.righttmotor.brake()
    def holdtank(self)  -> None: self.lefttmotor.hold();  self.righttmotor.hold()

    # ---> Helper functions for travelling distances <--- #
    def getdistance(self, dist: int) -> int: return dist * self.oneunit // self.scaleunit
    def getmotorsangle(self) -> int: return int(self.lefttmotor.angle() + self.righttmotor.angle())
    def getangle(self) -> int: return int(self.gyro.heading())

    def computeturnspeed(self, speedangle: int) -> int:
        # ---> Formulas: https://www.desmos.com/calculator/ww29qyn3yt <---
        speed = (self.circumferencerobot * speedangle // 360) 
        speed = self.getdistance(self.speed)
        return int(speed)

    # ---> Here comes the big boi (the real functions) <--- #
    async def gyroforwards(self, distance: int, speed1: int, speed2: int) -> None:

        # ---> Parameters initialization <--- #
        self.distance = 2 * self.getdistance(distance)
        self.speedlow = speed1; self.speedhigh = speed2

        # ---> Need to implement time and speeding functions <---
        self.speed = self.speedlow 

        # ---> Reset relative position and gyro <--- #
        self.lefttmotor.reset_angle(0)
        self.righttmotor.reset_angle(0)

        self.position = 0; self.lasttposition = 0

        while(self.position < self.distance):

            self.error = self.targetangle - self.getangle()
            self.PID.compute(self.error, 1)

            print(self.error, self.PID.error)

            self.lfspeed = limitint(self.speed + self.PID.controlleroutput, self.minspeed, self.maxspeed)
            self.rgspeed = limitint(self.speed - self.PID.controlleroutput, self.minspeed, self.maxspeed)

            self.movetank(self.lfspeed, self.rgspeed)
            await wait(self.updatetime)

            self.lasttposition = self.position
            self.position = self.getmotorsangle()

        self.stoptank()

        return None
    
    # ---> Need to decide between PID for angle or Speed Controller to change speed + error estimation <--- #
    async def turnleftspeed(self, angle: int, speedangle: int, *, error: int = 5, lfsign: int = -1, rgsign: int = 1) -> None:
        
        self.speed = self.computeturnspeed(speedangle) // ((lfsign != 0) + (rgsign != 0))
        
        angle = -abs(angle); error = -abs(error)
        self.targetangle += angle

        while(self.getangle() + error > self.targetangle):

            self.lfspeed = lfsign * self.speed
            self.rgspeed = rgsign * self.speed

            self.movetank(self.lfspeed, self.rgspeed)
            await wait(self.updatetime)

        self.stoptank()

        return None

    async def turnleftpid(self, angle: int, speedangle: int) -> None:
        return None

pid = PID_Controller()

# ---> All units of measurement are in milimeters <---
mydrivebase = DriveBase_Contorller(
    Motor(Port.A, Direction.COUNTERCLOCKWISE), 
    Motor(Port.C, Direction.CLOCKWISE), 
    2045, 1000, None, distbetweenwheel = 112,
    wheelcircumference = 176
)

async def main() -> None:

    mydrivebase.PID.setconstants(1250, 50, 2)
    # await wait(1000)
    # await mydrivebase.gyroforwards(250, 700, 200)
    
    timer.reset(); timer.resume()
    await mydrivebase.turnleft(90, 180)
    timer.pause(); print("For: 90, 180 -> ", timer.time())

    await wait(500)

    timer.reset(); timer.resume()
    await mydrivebase.turnleft(90, 180, lfsign = 0, rgsign = 1)
    timer.pause(); print("For: 90, 180 -> ", timer.time())

    await wait(500)

    # await mydrivebase.turnleftpid(90, 20) # 20 degrees per second

    # await wait(1000)
    # print(mydrivebase.targetangle, mydrivebase.getangle())
    # await wait(2500)
    # await mydrivebase.gyroforwards(250, 700, 200)

    # mydrivebase.movetank(200, 800)
    # await wait(250)
    # mydrivebase.stoptank()
    # while(1): pass

    # pid.setconstants(4,6,1)
    # print(pid.kp, pid.kd, pid.ki)
    # pid.__init__()
    # print(pid.kp, pid.kd, pid.ki)

    '''while pid.integral < pid.integralwindup:
        pid.compute(100000, 1)
        print(pid.integral, " | ", pid.error) 

    while pid.integral > -pid.integralwindup:
        pid.compute(-100000, 1)
        print(pid.integral, " | ", pid.error)'''

    return None

run_task(main())
