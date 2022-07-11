import Sofa
from stlib3.scene import Scene
from stlib3.scene import ContactHeader
from stlib3.physics.rigid import Floor
from math import pi
import sofaros
from sofaros import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from summit_xl import SummitXL, Floor
from summitxl_roscontroller import *
from nav_msgs.msg import Odometry

rosNode = sofaros.init("SofaNode")


def createScene(rootNode):
    ContactHeader(rootNode, alarmDistance=0.5, contactDistance=0.07)

    scene = Scene(rootNode)
    scene.addMainHeader()
    scene.VisualStyle.displayFlags = 'showCollisionModels'
    scene.addObject('DefaultVisualManagerLoop')
    scene.dt = 0.001
    scene.gravity = [0., -9.810, 0.]

    floor = Floor(rootNode,
                  name="Floor",
                  translation=[-2, -0.12, -2],
                  uniformScale=0.3,
                  isAStaticObject=True)

    scene.addObject('EulerImplicitSolver')
    scene.addObject('SparseLDLSolver')

    SummitXL(scene.Modelling)

    robot = scene.Modelling.SummitXL
    scene.Modelling.SummitXL.addObject(SummitxlROSController(name="KeyboardController", robot=scene.Modelling.SummitXL))

    scene.Modelling.SummitXL.addObject(sofaros.RosReceiver(rosNode, "/summit_xl/cmd_vel",
                                                           [robot.findData('robot_linear_vel'),
                                                            robot.findData('robot_angular_vel')],
                                                           Twist, vel_recv))

    #scene.Modelling.SummitXL.addObject(
    #    sofaros.RosSender(rosNode, "/sofa_sim/imu/data", [robot.findData('sim_orientation'),
    #                                                      robot.findData('robot_angular_vel'),
    #                                                      robot.findData('linear_acceleration'),
    #                                                      robot.findData('timestamp')], Imu, send))

    scene.Modelling.SummitXL.addObject(sofaros.RosSender(rosNode, "/summit_xl/robotnik_base_control/odom",
                                                                                    [robot.findData('timestamp'), 
                                                                                    robot.findData('sim_position'), 
                                                                                    robot.findData('sim_orientation'),
                                                                                    robot.findData('robot_linear_vel'), 
                                                                                    robot.findData('robot_angular_vel')], 
                                                                                    Odometry, odom_send))

    #scene.Modelling.SummitXL.addObject(
    #    sofaros.RosReceiver(rosNode, "/summit_xl/robotnik_base_control/odom", [robot.findData('timestamp'),
    #                                                                           robot.findData('reel_position'),
    #                                                                           robot.findData('reel_orientation')],
    #                        Odometry, odom_recv))

    scene.Modelling.SummitXL.addObject(sofaros.RosSender(rosNode, "/summit_xl/robotnik_base_control/cmd_vel",
                                           [robot.findData('robot_linear_vel'),robot.findData('robot_angular_vel')],
                                           Twist, vel_send))
    scene.Simulation.addChild(scene.Modelling)

    return rootNode
