import os
import Sofa
from Sofa import *

from stlib3.scene import Scene





def createWheel(parent, name, x, y, z):
    body = parent.addChild(name)
    body.addObject('MechanicalObject', name='dofs', showObject=True, template='Rigid3',
                    position=[x , y, z, 0., 0., 0., 1.], showObjectScale=0.09)
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
    print("-------------->", type(chassis))
    
    wheel1 = createWheel(rootNode.Simulation, 'front_left_wheel', 0.229, 0.235, 0.0)
    wheel2 = createWheel(rootNode.Simulation, 'back_left_wheel', -0.229, 0.235, 0.0)
    wheel3 = createWheel(rootNode.Simulation, 'front_right_wheel', 0.229, -0.235, 0.0)
    wheel4 = createWheel(rootNode.Simulation, 'back_right_wheel',-0.229, -0.235, 0.0)



    


    
    return rootNode
