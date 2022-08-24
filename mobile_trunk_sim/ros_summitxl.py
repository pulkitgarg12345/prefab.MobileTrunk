import Sofa
from stlib3.scene import Scene
import sofaros
from sofaros import *
from stlib3.physics.rigid import Floor
from summit_xl import SummitXL
from echelon3.parameters import *
from echelon3.createEchelon import *

#from geometry_msgs.msg import Twist
#from sensor_msgs.msg import Imu
from summitxl_roscontroller import *
#from nav_msgs.msg import Odometry



rosNode = sofaros.init("SofaNode")


def createScene(rootNode):

    #########################################
    # Plugins, data and Solvers
    ######################################### 

    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight");
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SofaMeshCollision')
    rootNode.addObject('RequiredPlugin', name='SofaPlugins', pluginName='SofaGeneralRigid SofaGeneralEngine SofaConstraint SofaImplicitOdeSolver SofaSparseSolver SofaDeformable SofaEngine SofaBoundaryCondition SofaRigid SofaTopologyMapping SofaOpenglVisual SofaMiscCollision')

    scale = 1000
    scene = Scene(rootNode, iterative=False)
    scene.addMainHeader()
    scene.addContact(alarmDistance=0.2*scale, contactDistance=0.005*scale)
    scene.VisualStyle.displayFlags = 'showCollisionModels showForceFields'
    scene.addObject('DefaultVisualManagerLoop')
    scene.dt = 0.001
    scene.gravity = [0., -9810., 0.]

    scene.Simulation.TimeIntegrationSchema.vdamping.value = 0.1
    scene.Simulation.TimeIntegrationSchema.rayleighStiffness = 0.01
    scene.Simulation.addObject('GenericConstraintCorrection' , solverName='LinearSolver', ODESolverName='GenericConstraintSolver')
    
    #########################################
    # create summit
    #########################################

    SummitXL(scene.Modelling, 1000)
    floor = Floor(rootNode,
                  name="Floor",
                  translation=[-2*scale, -0.12*scale, -2*scale],
                  uniformScale=0.3*scale,
                  isAStaticObject=True)

    robot=scene.Modelling.SummitXL
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

    ########################################
    # createEchelon
    ######################################## 

    trunk = scene.Modelling.SummitXL.Chassis.addChild("Trunk")
    trunk.addObject("MechanicalObject", name = "position", template="Rigid3d",
                    position=[0., 0.26*scale, 0.32*scale,-0.5, -0.5, -0.5 , 0.5 ],
                     showObject=True,showObjectScale = 30)    
    trunk.addObject('RigidRigidMapping',name='mapping', input=scene.Modelling.SummitXL.Chassis.position.getLinkPath(),
                                                index=0)

    scene.Modelling.SummitXL.Chassis.addChild('Arm')

    arm = scene.Simulation.addChild(scene.Modelling.SummitXL.Chassis.Arm)
    
    connection = rootNode.Modelling.SummitXL.Chassis.Trunk.position
    
    parameters, cables = createEchelon(arm,connection,0,[0., 0.26*scale, 0.32*scale],[-90,-90,0])

    return rootNode
