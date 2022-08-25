import Sofa
from stlib3.scene import Scene
from stlib3.physics.rigid import Cube
from summitxl_controller import *
from summit_xl import SummitXL
from echelon3.parameters import *
from echelon3.createEchelon import *


def Floor(parentNode, name,  translation, color=[0.5, 0.5, 0.5, 1.]):

    floor = parentNode.addChild(name)
    floor.addObject('MeshSTLLoader', name='loader', filename='meshes/Assembly.stl', 
                                    rotation=[-90, 0, 90],scale=2000, translation=translation )
    floor.addObject('MeshTopology', src='@loader')
    floor.addObject('OglModel', name="renderer", src='@loader', color=color)
    floor.addObject('MechanicalObject')


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

    scene = Scene(rootNode, iterative=False)
    scene.addMainHeader()
    scene.addContact(alarmDistance=0.1*1000, contactDistance=0.0005*1000)
    scene.VisualStyle.displayFlags = 'hideBehaviorModels showForceFields showCollisionModels showInteractionForceFields'
    scene.addObject('DefaultVisualManagerLoop')
    scene.dt = 0.001
    scene.gravity = [0., -9810., 0.]


    #scene.Modelling.addObject('EulerImplicitSolver',rayleighStiffness=0.01, rayleighMass=0, vdamping=0.1)
    #solver = scene.Modelling.addObject('SparseLDLSolver',name = 'SparseLDLSolver',template="CompressedRowSparseMatrixMat3x3d")

    scene.Simulation.TimeIntegrationSchema.vdamping.value = 0.1
    scene.Simulation.TimeIntegrationSchema.rayleighStiffness = 0.01
    scene.Simulation.addObject('GenericConstraintCorrection' , solverName='LinearSolver', ODESolverName='GenericConstraintSolver')

    #########################################
    # create summit
    #########################################
    #scene.Simulation.addChild(scene.Modelling)
    scale = 1000
    SummitXL(scene.Modelling, scale)
    floor = Floor(rootNode,
                  name="Floor",
                  translation=[5*scale, -3.5*scale, -2*scale])


    Cube(rootNode,
         translation=[-2*scale, -0.12*scale, -2*scale],
         uniformScale=0.4*1000)

    #def myAnimation(target, body, factor):
    #    body.position += [[0.0,0.0,0.001,0.0,0,0,1]]
    #    target.position = [[factor* 3.14 * 2]]*len(target.position.value)

    #animate(myAnimation, {
    #        "body" : scene.Modelling.SummitXL.Chassis.position,
    #        "target": scene.Modelling.SummitXL.Chassis.WheelsMotors.angles}, duration=2, mode="loop")

    scene.Modelling.SummitXL.addObject(SummitxlController(name="KeyboardController", robot=scene.Modelling.SummitXL, scale=scale))

    ########################################
    # createEchelon
    ######################################## 

    AttachedArm= scene.Modelling.SummitXL.Chassis.addChild("AttachedArm")
    AttachedArm.addObject("MechanicalObject", name = "position", template="Rigid3d",
                    position=[0., 0.26*1000, 0.32*1000,-0.5, -0.5, -0.5 , 0.5 ],
                     showObject=True,showObjectScale = 30)    
    AttachedArm.addObject('RigidRigidMapping',name='mapping', input=scene.Modelling.SummitXL.Chassis.position.getLinkPath(),
                                                index=0)

    trunk = scene.Modelling.SummitXL.Chassis.addChild('Trunk')
    base_position = rootNode.Modelling.SummitXL.Chassis.AttachedArm.position
    parameters, cables = createEchelon(trunk,base_position,0,[0., 0.26*1000, 0.32*1000],[-90,-90,0])

    if typeControl == 'displacement':
        trunk.addObject(CableController(cables, name = 'Cablecontroller'))
    elif typeControl == 'force' :
        trunk.addObject(ForceController(cables,dt,name = 'ForceController'))
    
    scene.Simulation.addChild(trunk)

    return rootNode