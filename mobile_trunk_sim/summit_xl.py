import Sofa
from stlib3.scene import Scene
from splib3.numerics import Quat
from math import pi
from summit_xl_controller import *
from summit_xl_description import *

def Chassis():
    """The summitXL chassis description.
       The chassis is composed of:
            - a rigid frame for its main position
            - four wheels motors each with an angles describing the angular position of each wheel.
    """
    self = Sofa.Core.Node("Chassis")
    self.addObject("MechanicalObject", name="position", template="Rigid3d", position=[[0,0,0,0,0,0,1]])

    chain = self.addChild("WheelsMotors")
    chain.addObject('MechanicalObject', name="angles", template="Vec1d", position=[0,0,0,0,0])

    ## The following is needed to describe the articulated chain from the chass's position
    ## to the wheel's one.
    chain.addObject('ArticulatedHierarchyContainer')
    wheelPositions = [[0.229, 0,0.235],
                      [-0.229, 0,0.235],
                      [0.229, 0,-0.235],
                      [-0.229, 0,-0.235]]
    for i in range(4):
        ac = chain.addChild("MotorToWheel{0}".format(i))
        ac.addObject('ArticulationCenter', parentIndex=0, childIndex=1+i, posOnParent=wheelPositions[i])
        a = ac.addChild("Articulation")
        a.addObject('Articulation', translation=False, rotation=True, rotationAxis=[1, 0, 0], articulationIndex=i)

    wheels = self.addChild("Wheels")
    # There is one extra position in this mechanical object because there the articulated chain
    # Needs a root one (in addition to the four wheels)
    wheels.addObject("MechanicalObject", name="position", template="Rigid3d",
                          position=[[0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1]],
                          showObject=True)

    wheels.addObject('ArticulatedSystemMapping',
                          input1=chain.angles.getLinkPath(),
                          input2=self.position.getLinkPath(),
                          output=wheels.position.getLinkPath())

    ## Adds VisualModel for the chassis's body
    visual = self.addChild("VisualModel")
    parts = {
        "Chassis" : ('meshes/summit_xl_chassis.stl', [0.5,0.5,0.5,1.0]) ,
        "ChassisCover" : ('meshes/summit_xl_covers.stl', [0.1,0.1,0.1,1.0])
    }
    for name, (filepath, color) in parts.items():
        part = visual.addChild(name)
        part.addObject('MeshSTLLoader', name='loader', filename=filepath, rotation=[-90,-90,0])
        part.addObject('MeshTopology', src='@loader')
        part.addObject('OglModel', name="renderer", src='@loader', color=color)
        part.addObject('RigidMapping', input=self.Wheels.position.getLinkPath(), index=0)

    ## Add VisualModel for the wheels
    visual = wheels.addChild("VisualModel")
    visual.addObject('MeshSTLLoader', name='loader', filename='meshes/wheel.stl', rotation=[0,0,90])
    visual.addObject('MeshTopology', name='geometry', src='@loader')
    for i in range(4):
        wheel = visual.addChild("Wheel{0}".format(i))
        wheel.addObject("OglModel", src=visual.geometry.getLinkPath(), color=[0.2,0.2,0.2,1.0])
        wheel.addObject("RigidMapping", input=self.Wheels.position.getLinkPath(), index=i+1)


    return self

def SummitXL(parentNode, name="SummitXL"):
    self = parentNode.addChild(name)
    self.addData(name="velocity", value=[0.0, 0.0, 0.0],
                 type="Vec3d", help="Summit_xl velocity", group="Summitxl_cmd_vel")
    self.addChild(Chassis())
    return self

def Floor(parentNode, color=[0.5, 0.5, 0.5, 1.], rotation=[0, 0, 0], translation=[0, 0, 0], scale=1):
    floor = parentNode.addChild('Floor')
    floor.addObject('MeshObjLoader', name='loader', filename='mesh/square1.obj', scale=scale, rotation=rotation, translation=translation)
    floor.addObject('OglModel', src='@loader', color=color)
    floor.addObject('MeshTopology', src='@loader', name='topo')
    floor.addObject('MechanicalObject')
    floor.addObject('TriangleCollisionModel')
    floor.addObject('LineCollisionModel')
    floor.addObject('PointCollisionModel')
    return floor

def createScene(rootNode):
    scene = Scene(rootNode)
    scene.addMainHeader()
    scene.dt = 0.01
    scene.gravity = [0., -9810., 0.]

    SummitXL(scene.Modelling)
    Floor(scene.Modelling, rotation=[90,0,0], translation=[-2,-0.12,-2], scale=4)

    #def myAnimation(target, body, factor):
    #    body.position += [[0.0,0.0,0.001,0.0,0,0,1]]
    #    target.position = [[factor* 3.14 * 2]]*len(target.position.value)

    #animate(myAnimation, {
    #        "body" : scene.Modelling.SummitXL.Chassis.position,
    #        "target": scene.Modelling.SummitXL.Chassis.WheelsMotors.angles}, duration=2, mode="loop")

    scene.Modelling.SummitXL.addObject(SummitxlController(name="KeyboardController", robot=scene.Modelling.SummitXL))

    scene.Simulation.addChild(scene.Modelling)

    return rootNode
