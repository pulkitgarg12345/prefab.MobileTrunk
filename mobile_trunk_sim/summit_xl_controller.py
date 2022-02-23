import Sofa
from splib3.numerics import RigidDof, Quat
from splib3.animation import animate
from splib3.constants import Key
from stlib3.scene import Scene
from math import *



class SummitxlController(Sofa.Core.Controller):
    
    def __init__(self, *args, **kwargs):
    
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.stepsize = 0.0005
        self.wheels = kwargs["wheels"]
        self.chassis = kwargs["chassis"]
        self.angle = pi/64

    def onKeypressedEvent(self, event):
        key = event['key']
        print("press on {}".format(key))
        #self.initSummit(key)
        #self.animateSummit(key)

        if key == "A":
            chassis_rigid = RigidDof(self.chassis.dofs)
            chassis_rigid.translate([self.stepsize,0.0,0.0])
            chassis_rigid.rotateAround([0, 1, 0],self.angle)

            for i in range(0,4):
                wheel_rigid = RigidDof(self.wheels[i].dofs)
                wheel_rigid.translate([self.stepsize, 0.0, 0.0])
                wheel_rigid.rotateAround([0, 1, 0],self.angle)

    
    