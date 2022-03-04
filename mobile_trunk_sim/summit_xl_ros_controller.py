# coding: utf8
#!/usr/bin/env python3
import Sofa
import sofaros
from sofaros import *
from splib3.numerics import RigidDof, Quat
from splib3.animation import animate
from splib3.constants import Key
from stlib3.scene import Scene
from math import *
from std_msgs.msg import Float32MultiArray 
from summit_xl_controller import *


def send(data):
    msg = Float32MultiArray()
    msg.data = list(data.value[0])
    return msg  


def recv(data, datafield):
    pass

