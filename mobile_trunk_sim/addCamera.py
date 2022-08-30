def addCamera(rootNode, scale):

    camera = rootNode.addChild('Camera')
    camera.addObject('MechanicalObject' , template='Rigid3d', rotation=[180, 0, 0],position=[1.13687e-13/scale, 260/scale, 1078.45/scale, -0.5, -0.5, 0.5, 0.5],
                          showObject=False, showObjectScale=100)

    camera.addObject('RigidRigidMapping', name="mapping", input = rootNode.Trunk.framesNode.frames.getLinkPath() ,index=100)

    rootNode.getRoot().addObject('OglViewport', screenSize=[750, 450], name='Camera', swapMainView=True, zNear=5,
                                      zFar=-10, fovy=55,
                                      cameraRigid=camera.MechanicalObject.position.getLinkPath(), useFBO=False)