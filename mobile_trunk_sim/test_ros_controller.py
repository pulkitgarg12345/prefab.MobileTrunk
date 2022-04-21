from turtle import position
import Sofa
from stlib3.scene import Scene
from splib3.numerics import Quat
from math import pi
from summit_xl_controller import *
import sofaros
from sofaros import *
from std_msgs.msg import Float32MultiArray
from summit_xl import SummitXL, Floor
from summit_xl_teleop_key import *
from summit_xl_ros_controller import *
rosNode = sofaros.init("SofaNode")


def createScene(rootNode):
    scene = Scene(rootNode)
    scene.addMainHeader()
    scene.dt = 0.01
    scene.gravity = [0., -9810., 0.]

    SummitXL(scene.Modelling)
    Floor(scene.Modelling, rotation=[90,0,0], translation=[-2,-0.12,-2], scale=4)
    robot=scene.Modelling.SummitXL
    scene.Modelling.SummitXL.addObject(SummitxlController(name="KeyboardController", robot=scene.Modelling.SummitXL))


    scene.Modelling.SummitXL.addObject(sofaros.RosReceiver(rosNode, "/summit_xl_control/cmd_vel",
                                           [robot.findData('linear_vel'),robot.findData('angular_vel')],
                                           Float32MultiArray, recv))
    scene.Simulation.addChild(scene.Modelling)

    return rootNode
