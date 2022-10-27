import Sofa
from splib3.numerics import RigidDof, Quat
from splib3.animation import animate
from splib3.constants import Key
from stlib3.scene import Scene
from math import *
from wheels_angles_compute import twistToWheelsAngularSpeed, move
from functional_test import test

msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    p
   j    k    l
   m    ,    .

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'p': (-1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1)
   }

speedBindings = {
    'q': (0.1, 0.1),
    'z': (-0.1, -0.1),
    'w': (0.1, 0),
    'x': (-0.1, 0),
    'e': (0, 0.1),
    'c': (0, -0.1),
   }


def vels(speed, turn):
       return 'currently:\tspeed %s\tturn %s ' % (speed, turn)
class SummitxlController(Sofa.Core.Controller):
    """A Simple keyboard controller for the SummitXL
       Key UP, DOWN, LEFT, RIGHT to move
    """
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.robot = kwargs["robot"]
        # self.test_angular_speed = kwargs["angular_speed"]
        # self.test_linear_speed = kwargs["linear_speed"]
        # self.test_duration = kwargs["duration"]
        self.test_flag = kwargs["test"]
        self.dt = 0
        self.status = 0.
        self.speed = 1
        self.turn = 2
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.robot.simrobot_angular_vel = [0., 0., 0.]
        self.robot.simrobot_linear_vel = [0., 0., 0.]
        self.elapsed_time = 0
        self.deplacement_ctrl = 0

    def onAnimateBeginEvent(self, event):
        """At each time step we move the robot by the given forward_speed and angular_speed)
        """
        
        self.dt = event['dt']
        if not self.test_flag:
            wheels_angular_speed = twistToWheelsAngularSpeed(self.robot.simrobot_angular_vel[2],
                                                            self.robot.simrobot_linear_vel[0])
            move(self.robot.Chassis.WheelsMotors.angles.rest_position,
                wheels_angular_speed, self.dt)

        elif self.test_flag:
                print("dt = ", self.elapsed_time)
                self.elapsed_time +=self.dt
                self.elapsed_time += self.dt
                deplacement = self.test_linear_speed * self.dt
                self.deplacement_ctrl +=deplacement

                if self.elapsed_time >= self.test_duration:
                    self.test_linear_speed = 0
                    self.test_angular_speed = 0
                    final_pos =[self.robot.Chassis.Base.position.position.value[0][0],
                                self.robot.Chassis.Base.position.position.value[0][1],
                                self.robot.Chassis.Base.position.position.value[0][2]]
                    print("final position = ", final_pos, "traveled distance = ", self.deplacement_ctrl)
                
                else : 
                    wheels_angular_speed = twistToWheelsAngularSpeed(self.test_angular_speed, self.test_linear_speed)
                    move(self.robot.Chassis.WheelsMotors.angles.rest_position, wheels_angular_speed, self.dt)


    def onKeypressedEvent(self, event):
        key = event['key']
        key = key.lower()

        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.th = moveBindings[key][3]
            self.robot.simrobot_linear_vel[0] = self.x * self.speed
            self.robot.simrobot_angular_vel[2] = self.th * self.turn
        elif  key in speedBindings.keys():
            self.speed = self.speed + speedBindings[key][0]
            self.turn = self.turn  + speedBindings[key][1]

            if (self.status == 14):
                print(msg)
            self.status = (self.status + 1) % 15
        else:
            self.x = 0.0
            self.th = 0.0


    def onKeyreleasedEvent(self, event):
        key = event['key']
        key = key.lower()
        if key in moveBindings.keys() or key in speedBindings.keys():
            self.robot.simrobot_linear_vel[0]= 0
            self.robot.simrobot_angular_vel[2] = 0