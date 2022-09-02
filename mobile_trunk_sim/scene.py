import Sofa
from stlib3.scene import Scene
from stlib3.physics.rigid import Cube
from summitxl_controller import *
from floor import Floor
from mobile_trunk import mobileTrunk
from addCamera import addCamera

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

    scene.Simulation.TimeIntegrationSchema.vdamping.value = 0.1
    scene.Simulation.TimeIntegrationSchema.rayleighStiffness = 0.01
    scene.Simulation.addObject('GenericConstraintCorrection' , linearSolver='@LinearSolver', ODESolverName='GenericConstraintSolver')

    #########################################
    # add robot
    #########################################
    scale = 1000
    trunk= mobileTrunk(scene.Modelling, scene.Simulation, scale)

    ##########################################
    # add Floor
    ##########################################
    floor = Floor(rootNode,
                  name="Floor",
                  translation=[5*scale, -3.5*scale, -2*scale])

    ##########################################
    # add Cube
    ##########################################
    Cube(rootNode,
         translation=[-1*scale, 10, -2*scale],
         uniformScale=0.5*1000)




    #def myAnimation(target, body, factor):
    #    body.position += [[0.0,0.0,0.001,0.0,0,0,1]]
    #    target.position = [[factor* 3.14 * 2]]*len(target.position.value)

    #animate(myAnimation, {
    #        "body" : scene.Modelling.SummitXL.Chassis.position,
    #        "target": scene.Modelling.SummitXL.Chassis.WheelsMotors.angles}, duration=2, mode="loop")

    scene.Modelling.SummitXL.addObject(SummitxlController(name="KeyboardController", robot=scene.Modelling.SummitXL, scale=scale))
    
    scene.Simulation.addChild(trunk)
    return rootNode