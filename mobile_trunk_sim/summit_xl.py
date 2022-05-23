import Sofa

from stlib3.scene import Scene
from stlib3.scene import ContactHeader
from stlib3.physics.rigid import Floor
from splib3.numerics import Quat

from summitxl_controller import *

from parameters import *
from SpringConnection import *

def Chassis():
    """The summitXL chassis description.
       The chassis is composed of:
            - a rigid frame for its main position
            - four wheels motors each with an angle describing the angular position of each wheel.
            - 3 sensors  linked to the chassis
                *lazer
                *imu
                *camera
    """
    totalMass = 1.0
    volume = 1.0
    inertiaMatrix = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    self = Sofa.Core.Node("Chassis")
    self.addObject("MechanicalObject", name="position", template="Rigid3d", position=[[0,0.01,0,0,0,0,1]])
    self.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])
    self.addObject('UncoupledConstraintCorrection')
    #debug
    # debug = self.addChild("Debug")
    # debug.addObject("MechanicalObject", name="position", template="Rigid3d", position=self.position.position.value)
    # debug.position.showObject = True
    # debug.position.showObjectScale = 0.5
    # debug.position.drawMode = 1
    # debug.position.showColor = [0,1,0,1]

    # reel_robot = self.addChild("Reel_robot")
    # reel_robot.addObject("MechanicalObject", name="position", template="Rigid3d", position=self.position.position.value)
    # reel_robot.position.showObject = True
    # reel_robot.position.showObjectScale = 0.5
    # reel_robot.position.drawMode = 1
    # reel_robot.position.showColor =[10., 8., 2., 0.75]
    #
    chain = self.addChild("WheelsMotors")
    chain.addObject('MechanicalObject', name="angles", template="Vec1d", position=[0,0,0,0,0])
    chain.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    sensor =  self.addChild("FixedSensor")
    sensor.addObject('MechanicalObject', name="angles", template="Vec1d", position=[0,0,0,0,0])
    sensor.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    trunk  = self.addChild("FixedTrunk")
    trunk.addObject('MechanicalObject', name="angles", template="Vec1d", position=[0,0,0,0,0])
    trunk.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])
    #[0., 0.26, 0.19],        # front_ptz_camera

    ## The following is needed to describe the articulated chain from the chass's position
    ## to the wheel's , the sensor's  and the trunk one .
    chain.addObject('ArticulatedHierarchyContainer')
    wheelPositions = [[0.229, 0,0.235],
                      [-0.229, 0,0.235],
                      [0.229, 0,-0.235],
                      [-0.229, 0,-0.235]]
    sensorName=["lazer", "gps", "camera_RGBD"]
    sensor.addObject('ArticulatedHierarchyContainer')
    sensorPositions = [[0., 0.28 ,0.],          # 2d lazer
                       [0,0.275,-0.22],         # gps
                       [0.012, 0.172, 0.324]    # front_rgbd_camera
                       ]

    trunk.addObject('ArticulatedHierarchyContainer')
    tc = sensor.addChild('Trunk')
    tc.addObject('ArticulationCenter', parentIndex=0, childIndex=1, posOnParent=[0., 0.26, 0.19])
    t = tc.addChild("Articulation")
    t.addObject('Articulation', translation=False, rotation=True, rotationAxis=[1, 0, 0], articulationIndex=0)

    for i in range(3):
        sc = sensor.addChild(sensorName[i])
        sc.addObject('ArticulationCenter', parentIndex=0, childIndex=1+i, posOnParent=sensorPositions[i])
        s = sc.addChild("Articulation")
        s.addObject('Articulation', translation=False, rotation=True, rotationAxis=[1, 0, 0], articulationIndex=i)

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
    wheels.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    wheels.addObject('ArticulatedSystemMapping',
                          input1=chain.angles.getLinkPath(),
                          input2=self.position.getLinkPath(),
                          output=wheels.position.getLinkPath())
    #Add sensors
    sensors = self.addChild("Sensors")
    sensors.addObject("MechanicalObject", name = "position", template="Rigid3d",
                    position=[[0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1]],
                     showObject=True)
    sensors.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    sensors.addObject('ArticulatedSystemMapping',
                        input1=sensor.angles.getLinkPath(),
                        input2=self.position.getLinkPath(),
                        output=sensors.position.getLinkPath())

    #Add trunk
    echelon = self.addChild("Echelon")
    echelon.addObject("MechanicalObject", name = "position", template="Rigid3d",
                    position=[[0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1]],
                     showObject=True)
    echelon.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    echelon.addObject('ArticulatedSystemMapping',
                        input1=trunk.angles.getLinkPath(),
                        input2=self.position.getLinkPath(),
                        output=echelon.position.getLinkPath())

    ## Adds VisualModel for the chassis's body
    visual = self.addChild("VisualModel")
    parts = {
        "Chassis" : ('meshes/summit_xl_chassis.stl', [0.1,0.1,0.1,1.0]) ,
        "ChassisCover" : ('meshes/summit_xl_covers.stl', [0.8,0.8,0.8,1.0]),
        "chassisSimple" : ('meshes/summit_xl_chassis_simple.stl', [0.5,0.5,0.5,1.0])
    }
    for name, (filepath, color) in parts.items():
        part = visual.addChild(name)
        part.addObject('MeshSTLLoader', name='loader', filename=filepath, rotation=[-90,-90,0])
        part.addObject('MeshTopology', src='@loader')
        part.addObject('OglModel', name="renderer", src='@loader', color=color)
        part.addObject('RigidMapping', input=self.Wheels.position.getLinkPath(), index=0)

    #chassis collision model
    # collision_model = self.addChild("CollisionModel")
    # for name, (filepath, color) in parts.items():
    #     chassis_collision = collision_model.addChild(name+"Collision")
    #     chassis_collision.addObject('MeshSTLLoader', name='loader', filename=filepath, rotation=[-90,-90,0])
    #     chassis_collision.addObject('MeshTopology', src='@loader')
    #     chassis_collision.addObject('MechanicalObject')
    #     chassis_collision.addObject('TriangleCollisionModel', group=0)
    #     chassis_collision.addObject('LineCollisionModel', group=0)
    #     chassis_collision.addObject('PointCollisionModel', group=0)
    #     chassis_collision.addObject('RigidMapping', input=self.Wheels.position.getLinkPath(), index=0)

    ## Add VisualModel for the wheels
    visual = wheels.addChild("VisualModel")
    visual.addObject('MeshSTLLoader', name='loader', filename='meshes/wheel.stl', rotation=[0,0,90])
    visual.addObject('MeshTopology', name='geometry', src='@loader')
    for i in range(4):
        wheel = visual.addChild("Wheel{0}".format(i))
        wheel.addObject("OglModel", src=visual.geometry.getLinkPath(), color=[0.2,0.2,0.2,1.0])
        wheel.addObject("RigidMapping", input=self.Wheels.position.getLinkPath(), index=i+1)

    # #wheel collision model
    collison_model = wheels.addChild("CollisionModel")
    for i in range(0,4):
        wheel_collision = collison_model.addChild("WheelCollision{0}".format(i))
        wheel_collision.addObject('MeshSTLLoader', name='loader', filename='meshes/collision_wheel.stl', rotation=[0, 90, 0])
        wheel_collision.addObject('MeshTopology', src='@loader')
        wheel_collision.addObject('MechanicalObject')
        wheel_collision.addObject('TriangleCollisionModel', group=0)
        wheel_collision.addObject('LineCollisionModel',group=0)
        wheel_collision.addObject('PointCollisionModel', group=0)
        wheel_collision.addObject('RigidMapping', input=self.Wheels.position.getLinkPath(), index=i+1)


    ## Add VisualModel for the sensors
    visual = sensors.addChild("VisualModel")
    sensorfilepath = {
        "lazer" : ('meshes/hokuyo_urg_04lx.stl', 1) ,
        "gps" : ('meshes/antenna_3GO16.stl', 2),
        # "camera" : ('meshes/axis_p5514.stl',3),
        "camera-RGBD" : ('meshes/orbbec_astra_embedded_s.stl', 3)
    }

    for name, (filepath, index) in sensorfilepath.items():
        visual_body = visual.addChild(name)
        visual_body.addObject('MeshSTLLoader', name=name+'_loader', filename=filepath, rotation=[0,90,90])
        visual_body.addObject('MeshTopology', src='@'+name+'_loader')
        visual_body.addObject('OglModel', name=name+"_renderer", src='@'+name+'_loader', color=[0.2,0.2,0.2,1.0])
        visual_body.addObject('RigidMapping', input=self.Sensors.position.getLinkPath(),index=index)

    # sensor collison model
    # collison_model = sensors.addChild('CollisionModel')
    # for name, (filepath, index) in sensorfilepath.items():
    #     sensor_collision = collison_model.addChild(name+"Collision")
    #     sensor_collision.addObject('MeshSTLLoader', name='loader', filename=filepath, rotation=[0,90,90])
    #     sensor_collision.addObject('MeshTopology', src='@loader')
    #     sensor_collision.addObject('MechanicalObject')
    #     sensor_collision.addObject('TriangleCollisionModel', group=0)
    #     sensor_collision.addObject('LineCollisionModel', group=0)
    #     sensor_collision.addObject('PointCollisionModel', group=0)
    #     sensor_collision.addObject('RigidMapping', input=self.Sensors.position.getLinkPath(), index=index)
    return self

def SummitXL(parentNode, name="SummitXL"):
    self = parentNode.addChild(name)
    self.addData(name="robot_linear_vel", value=[0.0, 0.0, 0.0],
                 type="Vec3d", help="Summit_xl velocity", group="Summitxl_cmd_vel")

    self.addData(name="robot_angular_vel", value=[0.0, 0.0, 0.0],
                 type="Vec3d", help="Summit_xl velocity", group="Summitxl_cmd_vel")

    self.addData(name="sim_orientation", value=[0., 0., 0., 0.],
                 type="Vec4d", help="Summit_xl imu", group="Summitxl_cmd_vel")

    self.addData(name="reel_orientation", value=[0., 0., 0., 0.],
                 type="Vec4d", help="Summit_xl imu", group="Summitxl_cmd_vel")

    self.addData(name="linear_acceleration", value=[0.0, 0.0, 0.0],
                 type="Vec3d", help="Summit_xl imu", group="Summitxl_cmd_vel")

    self.addData(name="timestamp",value=[0, 0], type="vector<int>", help="Summit_xl imu",
                 group="Summitxl_cmd_vel")

    self.addData(name="sim_position",  value=[0.0, 0.0, 0.0],type="Vec3d",
                 help="Summit_xl odom", group="Summitxl_cmd_vel")

    self.addData(name="reel_position",  value=[0.0, 0.0, 0.0],type="Vec3d",
                 help="Summit_xl odom", group="Summitxl_cmd_vel")

    self.addChild(Chassis())
    return self


def createScene(rootNode):

    ContactHeader(rootNode, alarmDistance=0.2, contactDistance=0.05)

    #########################################
    # Plugins, data and Solvers
    ######################################### 

    rootNode.addObject('VisualStyle', displayFlags='hideBehaviorModels showForceFields showCollisionModels showInteractionForceFields');

    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight");
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    rootNode.addObject('RequiredPlugin', name='EigenLinearSolvers')
    rootNode.addObject('RequiredPlugin', name='SofaMeshCollision')
    rootNode.addObject('RequiredPlugin', name='SofaPlugins', pluginName='SofaGeneralRigid SofaGeneralEngine SofaConstraint SofaImplicitOdeSolver SofaSparseSolver SofaDeformable SofaEngine SofaBoundaryCondition SofaRigid SofaTopologyMapping SofaOpenglVisual')


    scene = Scene(rootNode)
    scene.addMainHeader()
    scene.VisualStyle.displayFlags = 'showCollisionModels showForceFields'
    scene.addObject('DefaultVisualManagerLoop')
    scene.dt = 0.001
    scene.gravity = [0., -9810., 0.]

    scene.addObject('EulerImplicitSolver')
    scene.addObject('SparseLDLSolver',template="CompressedRowSparseMatrixMat3x3d")

    #########################################
    # create summit
    ######################################### 

    SummitXL(scene.Modelling)
    floor = Floor(rootNode,
                  name="Floor",
                  translation=[-2, -0.12, -2],
                  uniformScale=0.1,
                  isAStaticObject=True)

    #def myAnimation(target, body, factor):
    #    body.position += [[0.0,0.0,0.001,0.0,0,0,1]]
    #    target.position = [[factor* 3.14 * 2]]*len(target.position.value)

    #animate(myAnimation, {
    #        "body" : scene.Modelling.SummitXL.Chassis.position,
    #        "target": scene.Modelling.SummitXL.Chassis.WheelsMotors.angles}, duration=2, mode="loop")

    scene.Modelling.SummitXL.addObject(SummitxlController(name="KeyboardController", robot=scene.Modelling.SummitXL))

    #########################################
    # createEchelon
    ######################################### 

    echelon = scene.Modelling.SummitXL.Chassis.addChild('Echelon')
    connection = scene.Modelling.SummitXL.Chassis.Echelon.position
    createEchelon(echelon,connection,3,[0,0,0],[-90,-90,0])

    scene.Simulation.addChild(scene.Modelling)

    return rootNode
