def addCamera(rootNode):

    camera = rootNode.addChild('Camera')
    camera.addObject('MechanicalObject' , template='Rigid3d', position=[1.13687e-13, 260, 1064.25, -0.5, -0.5, -0.5, 0.5],
                          showObject=False, showObjectScale=10)

    camera.addObject('RigidRigidMapping', name="mapping", input = rootNode.Trunk.framesNode.frames.getLinkPath() ,index=100)

    rootNode.getRoot().addObject('OglViewport', screenSize=[750, 450], name='Camera', swapMainView=True, zNear=5,
                                      zFar=-10, fovy=55,
                                      cameraRigid=camera.MechanicalObject.position.getLinkPath(), useFBO=False)
