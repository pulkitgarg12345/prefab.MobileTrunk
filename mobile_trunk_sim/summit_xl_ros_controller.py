# coding: utf8
#!/usr/bin/env python3

import Sofa
from splib3.numerics import RigidDof
from std_msgs.msg import Float32MultiArray 
from math import *

global ray , speed
speed = 1
ray = 0.001 #rayon de la roue
dx = 2*ray*pi # deplacement pour un tour de roue
w = speed/ray #rotation des roues

def send(data):
    msg = Float32MultiArray()
    msg.data = list(data.value[0])
    return msg  

def recv(data, datafield):
    t = data.tolist()
    datafield.value = [[t[0] + dx, t[1], t[2], t[3], t[4],
                        t[5], t[6]]]

class SummitxlrosController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.chassis = kwargs["chassis"]
        self.robot = kwargs["robot"]
        self.wheels = kwargs["wheels"]

    def onAnimateBeginEvent(self, event):
        self.chassis.dofs.position = self.robot.dofs.position
        for i in range(0,4):
            wheel_rigid = RigidDof(self.wheels[i].dofs)
            wheel_rigid.translate([dx, 0.0, 0.0])
            wheel_rigid.rotateAround([0, 1, 0], w)
