import Sofa
from splib3.numerics import RigidDof, Quat
from splib3.animation import animate
from splib3.constants import Key
from stlib3.scene import Scene
from math import *



class SummitxlController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.chassis = kwargs["chassis"]
        self.robot = kwargs["robot"]
        self.wheels = kwargs["wheels"]
        self.ray = 0.0015
        self.dt = None
        self.dx = 0
        self.forward_speed = 0
        self.angular_speed = 0
        self.speed = 1
        self.w = 0

    def move(self, fwd, angle):
        robot = RigidDof(self.robot.dofs)
        robot.translate(robot.left * fwd)
        robot.rotateAround([0, 0, 1], angle)
        
    def onAnimateBeginEvent(self, event):        
        self.move(self.forward_speed, self.angular_speed)

    def onKeypressedEvent(self, event):
        key = event['key']
        if key == Key.downarrow:
            self.forward_speed = -0.01
        elif key == Key.uparrow:
            self.forward_speed = 0.01
        if key == Key.leftarrow:
            self.angular_speed = 0.05
        elif key == Key.rightarrow:
            self.angular_speed = -0.05
             
    def onKeyreleasedEvent(self, event):
        key = event['key']
        if key == Key.downarrow:
            self.forward_speed = 0
        elif key == Key.uparrow:
            self.forward_speed = 0
        if key == Key.leftarrow:
            self.angular_speed = 0
        elif key == Key.rightarrow:
            self.angular_speed = 0
        
            
            #for i in range(0,4):
            #    wheel_rigid = RigidDof(self.wheels[i].dofs)
            #    wheel_rigid.rotateAround([0, 0, 1], self.rotation_angle)

        #elif key == Key.downarrow:
            #chassis_rigid = RigidDof(self.chassis.dofs)
            #chassis_rigid.rotateAround([0, 0, 1],-self.rotation_angle)
            #for i in range(0,4):
                #wheel_rigid = RigidDof(self.wheels[i].dofs)
                #wheel_rigid.rotateAround([0, 0, 1],-self.rotation_angle)
