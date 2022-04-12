import Sofa
from stlib3.scene import Scene
from summit_xl_controller import *
from summit_xl_description import *




def createScene(rootNode):
    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('DefaultAnimationLoop')

    rootNode.dt = 0.01
    rootNode.gravity = [0., -9810., 0.]

    robot = rootNode.addChild("Summit_xl")
    robot.addObject('MechanicalObject', name = 'dofs', template ='Rigid3', position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.]
                    , showObject=True, showObjectScale=0.09)
    robot.addData(name="velocity", value=[0.0, 0.0, 0.0], type="Vec3d", help="Summit_xl velocity", group="Summitxl_cmd_vel")

    chassis = robot.addChild("Chassis")
    chassis.addObject('MechanicalObject', name='dofs', template='Rigid3',
                             position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.],
                             showObject=True, showObjectScale=0.09)
    visual = chassis.addChild("VisualModel")
    visual.addObject('MeshSTLLoader', name='loader', filename='meshes/summit_xl_chassis_simple.stl')
    visual.addObject('MeshTopology', src='@loader')
    visual.addObject('OglModel', name='renderer',
                        src='@loader',
                        color="0.5 0.5 0.5 1")
    visual.addObject('RigidMapping',
                        input=chassis.dofs.getLinkPath(),
                        output=visual.renderer.getLinkPath())

    wheel1 = createWheel(robot, 'front_left_wheel',
                         front_left_wheel_position, front_left_wheel_orientation)

    wheel2 = createWheel(robot, 'back_left_wheel',
                         back_left_wheel_position, back_left_wheel_orientation)

    wheel3 = createWheel(robot, 'front_right_wheel',
                         front_right_wheel_position, front_right_wheel_orientation)

    wheel4 = createWheel(robot, 'back_right_wheel',
                         back_right_wheel_position, back_right_wheel_orientation)

    camera = create_sensor(robot, "front_rgbd_camera_offset", camera_orientation)
    antenna = create_sensor(robot, "imu_offset")
    #gps = create_sensor(robot, "gps_offset")
    floor = Floor(rootNode)

    robot.addObject(SummitxlController(rootNode, robot=robot, camera=camera,
                                       antenna=antenna, chassis=chassis, wheels = [wheel1, wheel2, wheel3, wheel4]))

    return rootNode
