from stlib3.scene import Scene

def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0], plugins=['SofaSparseSolver', 'SofaOpenglVisual'], iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultAnimationLoop')
    scene.addObject('DefaultVisualManagerLoop')
    
    rootNode.dt = 0.01
    rootNode.addObject('MeshSTLLoader', name='loader1', filename='meshes/summit_xl_chassis_simple.stl')
    rootNode.addObject('OglModel', src='@loader1')
    chassis = rootNode.Simulation.addChild('MechanicalBody')

    chassis.addObject('MechanicalObject', name='dofs',
                             position=rootNode.loader1.position.getLinkPath(),
                             showObject=False, showObjectScale=5.0,
                            )
    
    chassis.addObject('UniformMass')

    visual = rootNode.addChild('Visual')
    visual.addObject('OglModel', name='renderer',
                        src='@../loader1',
                        color=[1.0, 1.0, 1.0, 0.5])
    
    chassis.addObject('IdentityMapping',
                             input=chassis.dofs.getLinkPath(),
                             output=visual.renderer.getLinkPath())
    