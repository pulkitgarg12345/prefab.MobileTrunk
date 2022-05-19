from signal import alarm
import Sofa
from stlib3.scene import Scene
from stlib3.scene import ContactHeader
from stlib3.physics.rigid import Floor

# def Floor(parentNode, color=[0.5, 0.5, 0.5, 1.], rotation=[0, 0, 0], translation=[0, 0, 0], scale=1):
#     floor = parentNode.addChild('Floor')
#     floor.addObject('MeshObjLoader', name='loader', filename='mesh/square1.obj', scale=scale, rotation=rotation, translation=translation)
#     floor.addObject('OglModel', src='@loader', color=color)
#     floor.addObject('MeshTopology', src='@loader', name='topo')
#     floor.addObject('MechanicalObject')
#     floor.addObject('TriangleCollisionModel')
#     floor.addObject('LineCollisionModel')
#     floor.addObject('PointCollisionModel')
#     return floor

def createScene(rootNode):
    ContactHeader(rootNode, alarmDistance=20, contactDistance=6)
    scene = Scene(rootNode)
    scene.addMainHeader()
    scene.VisualStyle.displayFlags = 'showCollisionModels'
    scene.dt = 0.001
    scene.gravity = [0., -9810., 0.]
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('DefaultVisualManagerLoop')

    #adding colision pipeline
    # scene.addObject('DefaultPipeline')
    # scene.addObject('BruteForceBroadPhase', name="BroadPhase")
    # scene.addObject('BVHNarrowPhase')
    # scene.addObject('LocalMinDistance', alarmDistance=0.1, contactDistance=0.05)
    # scene.addObject('RuleBasedContactManager', response="FrictionContact", responseParams="mu=0.8")
    # scene.addObject('GenericConstraintSolver')

    totalMass = 1.0
    volume = 1.0
    inertiaMatrix = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    wheel = scene.Modelling.addChild('Wheel')
    wheel.addObject("MechanicalObject", name = "position", template="Rigid3d",
                    position=[0., 20., 0., 0., 0., 0., 1.],showObject=True)

    wheel.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    wheel.addObject('UncoupledConstraintCorrection')
    #visual model
    visual = wheel.addChild('VisualModel')
    visual.addObject('MeshSTLLoader', name='loader1', filename='meshes/wheel.stl', rotation=[-90, 90, 0])
    visual.addObject('MeshTopology', src='@loader1')

    visual.addObject('OglModel', name='renderer', src='@loader1', color=[0.15, 0.45, 0.75, 0.7], scale=30)
    visual.addObject('RigidMapping',
                        input=wheel.position.getLinkPath(),
                        output=visual.renderer.getLinkPath())

    #collision model
    collision = wheel.addChild("Collision")
    collision.addObject('MeshSTLLoader', name='loader', filename='meshes/wheel.stl', rotation=[-90, -90, 0])
    collision.addObject('MeshTopology', src='@loader')
    collision.addObject('MechanicalObject')
    collision.addObject('TriangleCollisionModel')
    collision.addObject('LineCollisionModel')
    collision.addObject('PointCollisionModel')
    collision.addObject('RigidMapping')

    # Floor(scene.Modelling, rotation=[90,0,0], translation=[-2,-0.12,-2], scale=4)
    floor = Floor(rootNode,
                  name="Floor",
                  translation=[-2, -0.12, -2],
                  uniformScale=1.0,
                  isAStaticObject=True)
    scene.Simulation.addChild(scene.Modelling)

    return rootNode