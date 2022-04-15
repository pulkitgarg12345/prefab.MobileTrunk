import Sofa
from splib3.numerics import RigidDof, Quat
from splib3.animation import animate
from splib3.constants import Key
from stlib3.scene import Scene
from math import *

class SummitxlController(Sofa.Core.Controller):
    """A Simple keyboard controller for the SummitXL
       Key UP, DOWN, LEFT, RIGHT to move
    """
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.robot = kwargs["robot"]
        self.max_angular_vel = 3
        self.robot.linear_vel[0] = 0.1
        self.robot.angular_vel[2] = 0
        self.fwd = 0
        self.dt = 0
        self.wheel_ray = 0.0015

    def move(self, fwd, angle):
        """Move the robot using the forward speed and angular speed)"""
        robot = RigidDof(self.robot.Chassis.position)
        robot.translate(robot.forward * fwd)
        robot.rotateAround([0, 1, 0], angle)
        with self.robot.Chassis.WheelsMotors.angles.position.writeable() as angles:
            #Make the wheel turn according to forward speed
            # TODO: All the value are random, need to be really calculated
            angles += (fwd/self.wheel_ray)
            print(angles)
            #Make the wheel turn in reverse mode according to turning speed
            # TODO: the value are random, need to be really calculated
            angles[0] += (angle*10)
            angles[2] += (angle*10)
            angles[1] -= (angle*10)
            angles[3] -= (angle*10)

    def onAnimateBeginEvent(self, event):
        """At each time step we move the robot by the given forward_speed and angular_speed)
           TODO: normalize the speed by the dt so it is a real speed
        """
        self.dt = event['dt']
        self.move(self.fwd , self.robot.angular_vel[2])



    def onKeypressedEvent(self, event):
        key = event['key']
        if key == Key.downarrow:
            self.fwd = self.robot.linear_vel[0] * self.dt
        elif key == Key.uparrow:
            self.fwd = -self.robot.linear_vel[0] * self.dt
        if key == Key.leftarrow:
            self.robot.angular_vel[2] = self.robot.linear_vel[0] * self.dt * self.max_angular_vel
        elif key == Key.rightarrow:
            self.robot.angular_vel[2] = -self.robot.linear_vel[0] * self.dt * self.max_angular_vel

    def onKeyreleasedEvent(self, event):
        key = event['key']
        if key == Key.downarrow:
            self.fwd = 0
        elif key == Key.uparrow:
            self.fwd = 0
        if key == Key.leftarrow:
            self.robot.angular_vel[2] = 0
        elif key == Key.rightarrow:
            self.robot.angular_vel[2] = 0
