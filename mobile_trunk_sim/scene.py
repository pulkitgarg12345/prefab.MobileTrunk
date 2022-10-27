import Sofa
import sys
from stlib3.scene import Scene
from stlib3.physics.rigid import Cube
from summitxl_controller import *
from summitxl_roscontroller import *
from stlib3.physics.rigid import Floor
#from floor import Floor
from mobile_trunk import mobileTrunk
from addCamera import addCamera
import sofaros
from sofaros import *



def createScene(rootNode):

    print("PYTHON: ", sys.argv)

    #########################################
    # Plugins, data and Solvers
    ######################################### 

    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SofaMeshCollision')
    rootNode.addObject('RequiredPlugin', name='SofaPlugins', pluginName='SofaGeneralRigid SofaGeneralEngine SofaConstraint SofaImplicitOdeSolver SofaSparseSolver SofaDeformable SofaEngine SofaBoundaryCondition SofaRigid SofaTopologyMapping SofaOpenglVisual SofaMiscCollision')


    scene = Scene(rootNode, iterative=False)
    scene.addMainHeader()
    scene.addContact(alarmDistance=0.1*1000, contactDistance=0.05*1000, frictionCoef=8)
    scene.VisualStyle.displayFlags = 'hideBehaviorModels showForceFields showCollisionModels showInteractionForceFields'
    scene.addObject('DefaultVisualManagerLoop')
    scene.dt = 0.01
    scene.gravity = [0., -9810., 0.]
    scene.Settings.removeObject(scene.Settings.OglSceneFrame)

    scene.Simulation.addObject('GenericConstraintCorrection', solverName='LinearSolver', ODESolverName='GenericConstraintSolver')

    #scene.Simulation.addObject('GenericConstraintCorrection', linearSolver='@LinearSolver', ODESolver='@GenericConstraintSolver')
    scene.Simulation.TimeIntegrationSchema.vdamping.value = 0.1
    scene.Simulation.TimeIntegrationSchema.rayleighStiffness = 0.01

    #########################################
    # add robot
    #########################################
    scale = 1000
    self, trunk= mobileTrunk(scene.Modelling)
    scene.Simulation.addChild(trunk)
    scene.Simulation.addChild(self)

    floor = Floor(rootNode,
                  name="Floor",
                  translation=[-2*1000, -0.1*1000, 2*1000],
                  uniformScale=0.1*1000,
                  isAStaticObject=True)
    
    regle = rootNode.addChild("Regle")
    regle.addObject('MeshSTLLoader', name='loader', filename='meshes/reglette.stl', 
                                   rotation=[-90,-90,0], scale=1000, translation=[-1*1000, 0.1*1000, -2.4*1000])
    regle.addObject('MeshTopology', src='@loader')
    regle.addObject('OglModel', name="renderer", src='@loader', color="blue")
    regle.addObject('MechanicalObject')
    # ##########################################
    # add Floor
    ##########################################
    #Room(rootNode, translation=[-2*scale, 2.7*scale, -2*scale])

    # floor = Floor(rootNode,
    #               name="Floor",
    #               translation=[5*scale, -3.5*scale, -2*scale])

    ##########################################
    # add Cube
    ##########################################
#     Cube(rootNode,
#          translation=[-1*scale, 10, -2*scale],
#          uniformScale=0.5*1000)

    #def myAnimation(target, body, factor):
    #    body.position += [[0.0,0.0,0.001,0.0,0,0,1]]
    #    target.position = [[factor* 3.14 * 2]]*len(target.position.value)

    #animate(myAnimation, {
    #        "body" : scene.Modelling.SummitXL.Chassis.position,
    #        "target": scene.Modelling.SummitXL.Chassis.WheelsMotors.angles}, duration=2, mode="loop")

    if sys.argv[1] == "KeyboardController":
        scene.Modelling.SummitXL.addObject(SummitxlController(name="KeyboardController", robot=scene.Modelling.SummitXL))

    elif sys.argv[1] == "SimToRobot":
            rosNode = sofaros.init("SofaNode")
            robot=scene.Modelling.SummitXL
            scene.Modelling.SummitXL.addObject(SummitxlROSController(name="KeyboardController", robot=robot, robotToSim=False))

            scene.Modelling.SummitXL.addObject(sofaros.RosSender(rosNode, "/summit_xl/robotnik_base_control/odom",
                                                                                    [robot.findData('timestamp'), 
                                                                                    robot.findData('sim_position'), 
                                                                                    robot.findData('sim_orientation'),
                                                                                    robot.findData('robot_linear_vel'), 
                                                                                    robot.findData('robot_angular_vel')], 
                                                                                    Odometry, odom_send))

            scene.Modelling.SummitXL.addObject(sofaros.RosSender(rosNode, "/summit_xl/robotnik_base_control/cmd_vel",
                                           [robot.findData('robot_linear_vel'),robot.findData('robot_angular_vel')],
                                           Twist, vel_send))
            # scene.Modelling.SummitXL.addObject(
            # sofaros.RosReceiver(rosNode, "/summit_xl/robotnik_base_control/odom", [robot.findData('timestamp'),
            #                                                                   robot.findData('reel_position'),
            #                                                                   robot.findData('reel_orientation')],
            #                Odometry, odom_recv))

    elif sys.argv[1] == "RobotToSim":
        rosNode = sofaros.init("SofaNode")
        robot=scene.Modelling.SummitXL
        scene.Modelling.SummitXL.addObject(SummitxlROSController(name="KeyboardController", robot=robot, robotToSim=True))
        scene.Modelling.SummitXL.addObject(sofaros.RosReceiver(rosNode, "/summit_xl/robotnik_base_control/odom",
                                                                             [robot.findData('timestamp'),
                                                                              robot.findData('reel_position'),
                                                                              robot.findData('reel_orientation'),
                                                                              robot.findData('robot_linear_vel'),
                                                                              robot.findData('robot_angular_vel')],
                                                                                Odometry, odom_recv))
        
        scene.Modelling.SummitXL.addObject(sofaros.RosSender(rosNode, "/digital_twin/odom", [robot.findData('sim_position'),
                                                              robot.findData('sim_orientation')],Odometry, 
                                                              position_and_orientation_send))
        scene.Modelling.SummitXL.addObject(sofaros.RosSender(rosNode, "/digital_twin/cmd_vel",
                                           [robot.findData('robot_linear_vel'),robot.findData('robot_angular_vel')],
                                           Twist, vel_send))


    return rootNode