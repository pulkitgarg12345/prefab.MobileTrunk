# coding: utf8
#!/usr/bin/env python3

import Sofa
from splib3.numerics import RigidDof
from math import *

global ray , speed
speed = 1
ray = 0.0015 #rayon de la roue
dx = 2*ray*pi # deplacement pour un tour de roue
w = speed/ray #rotation des roues

def recv(data, datafield):
    t = data.tolist()
    datafield[0].value = [t[0], t[1], t[2]]
    datafield[1].value = [t[0], t[1], t[2]]


class SummitxlrosController(Sofa.Core.Controller):
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
        print(self.robot.linear_vel[0], self.robot.linear_vel[1], self.robot.linear_vel[2])
        self.dx = self.robot.linear_vel[0] * self.dt
        self.robot.dofs.position[0][0]+= self.dx
        self.w = self.speed/self.ray
        self.chassis.dofs.position = self.robot.dofs.position
        for i in range(0,4):
            wheel_rigid = RigidDof(self.wheels[i].dofs)
            wheel_rigid.translate([self.dx , 0.0, 0.0])
            wheel_rigid.rotateAround([0, 1, 0], self.w)