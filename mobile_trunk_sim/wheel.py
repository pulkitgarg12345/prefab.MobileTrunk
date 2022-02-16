import os
from stlib3.scene import Scene
dirPath = os.path.dirname(os.path.abspath(__file__))+'/'


def createScene(rootNode):
    
    scene = Scene(rootNode, plugins=['SofaConstraint', 'SofaGeneralRigid', 'SofaOpenglVisual', 'SofaRigid'], iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=1e3, tolerance=1e-5)
    scene.Simulation.addObject('GenericConstraintCorrection')
    
    scene.dt = 0.01
    scene.gravity = [0., -9810., 0.]
    
    rootNode.addObject('MeshSTLLoader', name='loader2', filename='meshes/wheel.stl')


    visual = rootNode.addChild('Visual')
    
    visual.addObject('OglModel', name='renderer',
                        src='@../loader2',
                        color=[1.0, 2.0, 1.0, 0.5])

    return rootNode