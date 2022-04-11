import sys
import Sofa
from stlib3.scene import Scene
from splib3.numerics import Quat
from math import pi
import sofaros
from sofaros import *
from std_msgs.msg import Float32MultiArray
from summit_xl_ros_controller import *

front_left_wheel_position = [0.229, 0.235, 0.0]
back_left_wheel_position = [-0.229, 0.235, 0.0]
front_right_wheel_position = [0.229, -0.235, 0.0]
back_right_wheel_position = [-0.229, -0.235, 0.0]

front_left_wheel_orientation = [0.0, 0.0, 0.0, 0.0]
front_right_wheel_orientation = [0.0, 0.0, 0.0, 0.0]
back_left_wheel_orientation = [0.0, 0.0, 0.0, 0.0]
back_right_wheel_orientation = [0.0, 0.0, 0.0, 0.0]

rosNode = sofaros.init("SofaNode")




def wheel_orientation(angle):
    wheel_orientation = [0.0, 0.0, 0.0, 0.0]
    wheel_orientation = Quat.createFromAxisAngle([1.0, 0., 0], angle)
    wheel_orientation.rotateFromEuler([angle, 0. , 0.])

    return wheel_orientation

def Floor(parentNode, color=[0.5, 0.5, 0.5, 1.], rotation=[0, 0, 0],
          position=[0.0, 0.0, 0, 0.0, 0.0, 0.0, 1.], translation=[-1, -1, -0.15]):
    floor = parentNode.addChild('Floor')
    floor.addObject('MeshObjLoader', name='loader', filename='mesh/square1.obj', scale=2, rotation=rotation, translation=translation)
    floor.addObject('OglModel', src='@loader', color=color)
    floor.addObject('MeshTopology', src='@loader', name='topo')
    floor.addObject('MechanicalObject')
    floor.addObject('TriangleCollisionModel')
    floor.addObject('LineCollisionModel')
    floor.addObject('PointCollisionModel')
    return floor

def createWheel(parent, name, wheel_position, wheel_orientation):
    body = parent.addChild(name)
    body.addObject('MechanicalObject', name='dofs', showObject=True, template='Rigid3',
                    position=[wheel_position[0], wheel_position[1], wheel_position[2],
                              wheel_orientation[0], wheel_orientation[1], wheel_orientation[2],
                              wheel_orientation[3]],showObjectScale=0.09)
    visual = body.addChild('VisualModel')

    visual.addObject('MeshSTLLoader', name='loader1', filename='meshes/wheel.stl')

    visual.addObject('MeshTopology', src='@loader1')

    visual.addObject('OglModel', name='renderer', src='@loader1', color="Black")

    visual.addObject('RigidMapping',
                        input=body.dofs.getLinkPath(),
                        output=visual.renderer.getLinkPath())

    return body

def createScene(rootNode):
    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('DefaultAnimationLoop')

    rootNode.dt = 0.01
    rootNode.gravity = [0., -9810., 0.]

    robot = rootNode.addChild("Summit_xl")
    chassis = robot.addChild("Chassis")
    robot.addObject('MechanicalObject', name = 'dofs', template ='Rigid3', position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.], showObject=True, showObjectScale=0.09)
    robot.addData(name="velocity", value=[0.0, 0.0, 0.0], type="Vec3d", help="Summit_xl velocity", group="Summitxl_cmd_vel")

    chassis.addObject('MechanicalObject', name='dofs', template='Rigid3',
                             position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.],
                             showObject=False, showObjectScale=0.09)

    visual = chassis.addChild("VisualModel")
    visual.addObject('MeshSTLLoader', name='loader', filename='meshes/summit_xl_chassis_simple.stl')
    visual.addObject('MeshTopology', src='@loader')
    visual.addObject('OglModel', name='renderer',
                        src='@loader',
                        color="Ebony")
    visual.addObject('RigidMapping',
                        input=chassis.dofs.getLinkPath(),
                        output=visual.renderer.getLinkPath())

    front_right_wheel_orientation = wheel_orientation(pi/2)

    back_right_wheel_orientation = wheel_orientation(pi/2)

    front_left_wheel_orientation = wheel_orientation(pi)

    back_left_wheel_orientation = wheel_orientation(pi)

    wheel1 = createWheel(robot, 'front_left_wheel',
                         front_left_wheel_position, front_left_wheel_orientation)

    wheel2 = createWheel(robot, 'back_left_wheel',
                         back_left_wheel_position, back_left_wheel_orientation)

    wheel3 = createWheel(robot, 'front_right_wheel',
                         front_right_wheel_position, front_right_wheel_orientation)

    wheel4 = createWheel(robot, 'back_right_wheel',
                         back_right_wheel_position, back_right_wheel_orientation)

    floor = Floor(rootNode)

    rootNode.addObject(sofaros.RosReceiver(rosNode, "/summit_xl_control/cmd_vel",
                                           robot.findData("velocity"), Float32MultiArray, recv))

    #robots.finData("velocity")
    robot.addObject(SummitxlrosController(rootNode, chassis=chassis, robot=robot,wheels = [wheel1, wheel2, wheel3, wheel4]))

    return rootNode
