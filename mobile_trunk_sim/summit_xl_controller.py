import Sofa
from splib3.numerics import RigidDof, Quat
from splib3.animation import animate
from splib3.constants import Key
from stlib3.scene import Scene
from math import *


def setup_chassis_animation(chassis, vector):
    rigid = RigidDof(chassis.dofs)
    print("------->", vector)
    rigid.translate(vector)

def setup_wheels_animation(wheels, vector):
    vector = sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)
    rigid = RigidDof(wheels.dofs)
    rigid.setPosition(rigid.rest_position + rigid.translate(vector, "position"))


class SummitxlController(Sofa.Core.Controller):
    
    def __init__(self, *args, **kwargs):
    
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.stepsize = 0.05
        self.wheels = kwargs["wheels"]
        self.chassis = kwargs["chassis"]

    def onKeypressedEvent(self, event):
        key = event['key']
        print("press on {}".format(key))
        #self.initSummit(key)
        #self.animateSummit(key)

        if key == "A":
            rigid = RigidDof(self.chassis.dofs)
            rigid.translate([self.stepsize,0.0,0.0])
            for i in range(0,4):
                rigid = RigidDof(self.wheels[i].dofs)
                rigid.translate([self.stepsize, 0.0, 0.0])
        
        
        elif key =='I':
            rigid = RigidDof(self.chassis.dofs)
            rigid.rotateAround("sx", 3)
            
        
    #def animateSummit(self, key):
        
    #    if key == Key.C:
    #        animate(setup_chassis_animation,
    #                {"chassis":self.chassis, "vector": [0.05, 0., 0.]}, duration=0.2)
            
            #for i in range(0,4):
                    #animate(setup_wheels_animation,
                        #{"wheelss":self.wheels[i], "vector": [0, 0, 4]}, duration=0.2)
    
    