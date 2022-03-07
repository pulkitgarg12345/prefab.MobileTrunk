import Sofa
from splib3.numerics import RigidDof, Quat
from splib3.animation import animate
from splib3.constants import Key
from stlib3.scene import Scene
from math import *



class SummitxlController(Sofa.Core.Controller):
    
    def __init__(self, *args, **kwargs):
    
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        speed = 1
        ray = 0.001 #rayon de la roue
        dx = 2*ray*pi # deplacement pour un tour de roue
        w = speed/ray #rotation des roues
        self.move_forward = dx
        self.move_backward = -dx
        self.wheels = kwargs["wheels"]
        self.chassis = kwargs["chassis"]
        self.angle = w

    def onKeypressedEvent(self, event):
        key = event['key']
        print("press on {}".format(key))

        if key == Key.leftarrow:
            chassis_rigid = RigidDof(self.chassis.dofs)
            chassis_rigid.translate([self.move_backward,0.0,0.0])
            for i in range(0,4):
                wheel_rigid = RigidDof(self.wheels[i].dofs)
                wheel_rigid.translate([self.move_backward, 0.0, 0.0])
                wheel_rigid.rotateAround([0, 1, 0],-self.angle)
        
        elif key == Key.rightarrow:
            chassis_rigid = RigidDof(self.chassis.dofs)
            chassis_rigid.translate([self.move_forward,0.0,0.0])
            for i in range(0,4):
                wheel_rigid = RigidDof(self.wheels[i].dofs)
                wheel_rigid.translate([self.move_forward, 0.0, 0.0])
                wheel_rigid.rotateAround([0, 1, 0],self.angle)
        
        elif key == Key.uparrow: 
            chassis_rigid = RigidDof(self.chassis.dofs)
            #chassis_rigid.rotateAround([0, 0, 1],self.angle)
            for i in range(0,4):
                wheel_rigid = RigidDof(self.wheels[i].dofs)
                wheel_rigid.rotateAround([0, 0, 1],self.angle)

        elif key == Key.downarrow:
            chassis_rigid = RigidDof(self.chassis.dofs)
            #chassis_rigid.rotateAround([0, 0, 1],-self.angle)
            for i in range(0,4):
                wheel_rigid = RigidDof(self.wheels[i].dofs)
                wheel_rigid.rotateAround([0, 0, 1],-self.angle)

    
    