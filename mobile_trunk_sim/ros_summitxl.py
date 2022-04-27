from turtle import position
import Sofa
from stlib3.scene import Scene
from splib3.numerics import Quat
from math import pi
import sofaros
from sofaros import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu
from summit_xl import SummitXL, Floor
from summitxl_roscontroller import *
rosNode = sofaros.init("SofaNode")


def createScene(rootNode):
    scene = Scene(rootNode)
    scene.addMainHeader()
    scene.dt = 0.01
    scene.gravity = [0., -9810., 0.]

    SummitXL(scene.Modelling)
    Floor(scene.Modelling, rotation=[90,0,0], translation=[-2,-0.12,-2], scale=4)
    robot=scene.Modelling.SummitXL
    scene.Modelling.SummitXL.addObject(SummitxlROSController(name="KeyboardController", robot=scene.Modelling.SummitXL))


    scene.Modelling.SummitXL.addObject(sofaros.RosReceiver(rosNode, "/summit_xl/robotnik_base_control/cmd_vel",
                                           [robot.findData('linear_vel'),robot.findData('angular_vel')],
                                           Twist, vel_recv))

    scene.Modelling.SummitXL.addObject(sofaros.RosReceiver(rosNode, "/summit_xl/clock",robot.findData('timestamp'),
                                           Int32MultiArray, time_recv))

    scene.Modelling.SummitXL.addObject(sofaros.RosSender(rosNode, "/summit_xl/mavros/imu/data",[robot.findData('orientation'),
                                                        robot.findData('angular_vel'), robot.findData('linear_acceleration'),
                                                        robot.findData('timestamp')],Imu, send))
    scene.Simulation.addChild(scene.Modelling)

    return rootNode
