# ---> Written by AndreiDani <---
from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Direction, Stop, Axis, Button
from pybricks.pupdevices import Motor 
from pybricks.tools import wait, run_task, multitask
from pybricks.robotics import DriveBase

# ---> Max value for int = 2 ^ 31 - 1 = 1073741823 <--- #
def limitint(value: int, valuemin: int, valuemax: int) -> int:
    return min(max(value, valuemin), valuemax)

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
        self.error = limitint(error, self.lasterror - self.maxchange, self.lasterror + self.maxchange)
        self.lasterror = self.error # ---> Changing last error <--- #

        self.noisefiltering() # ---> Filter errors and noise from the input <---
        # self.gradientascent() # ---> Tune the constants according to the gradient ascent algorithm <---

        self.proportional = self.error
        self.derivative = (self.error - self.lasterror)

        self.integral += self.error
        self.integral = limitint(self.integral, -self.integralwindup, self.integralwindup)

        self.controlleroutput = (
            self.proportional * self.kp + 
            self.derivative * self.kd / self.dt + 
            self.integral * self.ki
        ) // self.cstscale * sign

        return 
   
class DriveBase_Contorller(object):
    def __init__(self, lfmotor: Motor, rgmotor: Motor, oneunit: int, scaleunit: int, wheelerror: int = None) -> None:

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

        # ---> Object controller for swaps in programs <--- #
        self.PID = PID_Controller()

        # ---> Transforming Distance <--- #
        self.oneunit = oneunit; self.scaleunit = scaleunit

        # ---> Moving the robot <--- #
        self.speed: int = 0; self.distance: int = 0
        self.speedlow: int = 0; self.speedhigh: int = 0



    # ---> Pair motor functions (Move tank and stop) <--- #
    def movetank(self, speedlf: int, speedrg: int) -> None:
        self.lefttmotor.run(speedlf)
        self.righttmotor.run(speedrg)
        return None

    # ---> Stop tank functions <---
    def stoptank(self)  -> None: self.lefttmotor.stop();  self.righttmotor.stop()
    def braketank(self) -> None: self.lefttmotor.brake(); self.righttmotor.brake()
    def holdtank(self)  -> None: self.lefttmotor.hold();  self.righttmotor.hold()

    # ---> Helper functions for travelling distances <--- #
    def getdistance(self, dist: int) -> int:
        return dist * self.oneunit // self.scaleunit
    
    def getmotorsangle(self) -> int:
        return int(self.lefttmotor.angle() + self.righttmotor.angle())

    # ---> Here comes the big boi (the real functions) <--- #
    async def gyroforwards(self, distance: int, speed1: int, speed2: int) -> None:
        
        # ---> Parameters initialization <--- #
        self.distance = 2 * self.getdistance(distance)
        self.speedlow = speed1; self.speedhigh = speed2

        self.speed = self.speedlow # ---> Need to implement time and speeding functions <---

        # ---> Reset relative position and gyro <--- #
        self.lefttmotor.reset_angle(0)
        self.righttmotor.reset_angle(0)

        while(self.getmotorsangle() < self.distance):
            self.movetank(self.speed, self.speed)
        self.stoptank()

        return None

pid = PID_Controller()

mydrivebase = DriveBase_Contorller(
    Motor(Port.A, Direction.COUNTERCLOCKWISE), 
    Motor(Port.C, Direction.CLOCKWISE), 
    2045, 1000, None, 0
)

async def main() -> None:

    await mydrivebase.gyroforwards(100, 400, 200)

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
