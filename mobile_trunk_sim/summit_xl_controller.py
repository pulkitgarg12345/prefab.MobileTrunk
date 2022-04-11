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
        self.speed = 1
        self.w = 0

    def onAnimateBeginEvent(self, event):
        self.dt = event['dt']
        self.dx = self.robot.velocity[0] * self.dt
        self.w = self.speed/self.ray

    def onKeypressedEvent(self, event):
        key = event['key']
        if key == Key.leftarrow:
            self.dx-=0.005
            self.robot.dofs.position[0][0]+= self.dx
            self.chassis.dofs.position = self.robot.dofs.position
            for i in range(0,4):
                wheel_rigid = RigidDof(self.wheels[i].dofs)
                wheel_rigid.translate([self.dx ,0.0, 0.0])
                wheel_rigid.rotateAround([0, 1, 0],-self.w)

        elif key == Key.rightarrow:
            self.dx+=0.005
            self.robot.dofs.position[0][0]+= self.dx
            self.chassis.dofs.position = self.robot.dofs.position
            for i in range(0,4):
                wheel_rigid = RigidDof(self.wheels[i].dofs)
                wheel_rigid.translate([self.dx, 0.0, 0.0])
                wheel_rigid.rotateAround([0, 1, 0],self.w)

        #elif key == Key.uparrow:
            #chassis_rigid = RigidDof(self.chassis.dofs)
            #chassis_rigid.rotateAround([0, 0, 1],self.rotation_angle)
            #for i in range(0,4):
                #wheel_rigid = RigidDof(self.wheels[i].dofs)
                #wheel_rigid.rotateAround([0, 0, 1],self.rotation_angle)

        #elif key == Key.downarrow:
            #chassis_rigid = RigidDof(self.chassis.dofs)
            #chassis_rigid.rotateAround([0, 0, 1],-self.rotation_angle)
            #for i in range(0,4):
                #wheel_rigid = RigidDof(self.wheels[i].dofs)
                #wheel_rigid.rotateAround([0, 0, 1],-self.rotation_angle)
