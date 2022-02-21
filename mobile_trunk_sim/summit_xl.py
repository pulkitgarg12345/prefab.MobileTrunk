import os
import Sofa
from stlib3.scene import Scene
from splib3.numerics import Quat
from math import pi

front_left_wheel_position = [0.229, 0.235, 0.0]
back_left_wheel_position = [-0.229, 0.235, 0.0]
front_right_wheel_position = [0.229, -0.235, 0.0]
back_right_wheel_position = [-0.229, -0.235, 0.0]

front_left_wheel_orientation = [0.0, 0.0, 0.0, 0.0]
front_right_wheel_orientation = [0.0, 0.0, 0.0, 0.0]
back_left_wheel_orientation = [0.0, 0.0, 0.0, 0.0]
back_right_wheel_orientation = [0.0, 0.0, 0.0, 0.0]


def wheel_orientation():
    wheel_orientation = [0.0, 0.0, 0.0, 0.0]
    wheel_orientation = Quat.createFromAxisAngle([1.0, 0., 0], pi/2.)
    wheel_orientation.rotateFromEuler([pi/2, 0. , 0.])
    
    return wheel_orientation


def createWheel(parent, name, wheel_position, wheel_orientation):
    body = parent.addChild(name)
    body.addObject('MechanicalObject', name='dofs', showObject=True, template='Rigid3',
                    position=[wheel_position[0], wheel_position[1], wheel_position[2],
                              wheel_orientation[0], wheel_orientation[1], wheel_orientation[2],
                              wheel_orientation[3]],showObjectScale=0.09)
    body.addObject('UniformMass', totalMass=0.01)
    visual = body.addChild('VisualModel')

    visual.addObject('MeshSTLLoader', name='loader1', filename='meshes/wheel.stl')

    visual.addObject('MeshTopology', src='@loader1')

    visual.addObject('OglModel', name='renderer', src='@loader1', color=[0.15, 0.45, 0.75, 0.7])

    visual.addObject('RigidMapping',
                        input=body.dofs.getLinkPath(),
                        output=visual.renderer.getLinkPath())

    return body
def createScene(rootNode):
    
    scene = Scene(rootNode, plugins=['SofaConstraint', 'SofaGeneralRigid', 'SofaOpenglVisual', 'SofaRigid'], iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=1e3, tolerance=1e-5)
    scene.Simulation.addObject('GenericConstraintCorrection')

    scene.dt = 0.01
    scene.gravity = [0., -9810., 0.]

    chassis = rootNode.Simulation.addChild("Chassis")
    chassis.addObject('MechanicalObject', name='dofs', template='Rigid3',
                             position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.],
                             showObject=False, showObjectScale=5.0,
                            )
    chassis.addObject('UniformMass', totalMass=0.01)
    visual = chassis.addChild("VisualModel")
    visual.addObject('MeshSTLLoader', name='loader', filename='meshes/summit_xl_chassis_simple.stl')
    visual.addObject('MeshTopology', src='@loader')
    visual.addObject('OglModel', name='renderer',
                        src='@loader',
                        color=[0.6, 0.6, 0.6, 0.6])
    

    front_right_wheel_orientation = wheel_orientation()
   
    back_right_wheel_orientation = wheel_orientation()
    
    wheel1 = createWheel(rootNode.Simulation, 'front_left_wheel',
                         front_left_wheel_position, front_left_wheel_orientation)
    
    wheel2 = createWheel(rootNode.Simulation, 'back_left_wheel',
                         back_left_wheel_position, back_left_wheel_orientation)
    
    wheel3 = createWheel(rootNode.Simulation, 'front_right_wheel',
                         front_right_wheel_position, front_right_wheel_orientation)
    
    wheel4 = createWheel(rootNode.Simulation, 'back_right_wheel',
                         back_right_wheel_position, back_right_wheel_orientation)



    


    
    return rootNode
