
from parameters import *
from classEchelon import *

if base_translation :
    from BaseController import *
if not inverse :
    from CableController import *



def createEchelon(echelon,base,index,translation,rotation):
    """
    Creates topology and cables for echelon3 arm
    Parameters :
        echelon : Sofa child on which we add everything
        base : mechanical object on which we attach the arm
        index : index of the mechanical point we attach the arm
        translation : from 0,0,0 where we creat the arm
        rotation : from 0,0,0 where we creat the arm
    """
    parameters = Echelon3([6,6,8],[12,11,7.375],[100,50.0*1.33, 35.0,35],[75,50.0,35, 35.0])

    position =  echelon.addObject('TransformEngine',name="transform" ,template="Vec3d" ,translation=translation ,rotation=rotation ,input_position=parameters.position)
    positionRigid =  echelon.addObject('TransformEngine',name="transformRigid" ,template="Rigid3d" ,translation=translation ,rotation=rotation ,input_position=parameters.positionRigid)
    # position =  echelon.addObject('TransformEngine',name="transform" ,template="Vec3d" ,translation=[0,0,0] ,rotation=[45,0,0] ,input_position=position.output_position)
    # positionRigid =  echelon.addObject('TransformEngine',name="transformRigid" ,template="Rigid3d" ,translation=[0,0,0] ,rotation=[45,0,0] ,input_position=positionRigid.output_position)
    position.init()
    positionRigid.init()
    #########################################
    # adding Points and topology
    ######################################### 

    framesNode = echelon 
    
    framesNode.addObject('PointSetTopologyContainer', position=position.output_position);
    frames = framesNode.addObject('MechanicalObject', name='frames', template='Rigid3d',  showObject=1, showObjectScale=2, position=positionRigid.output_position)

    topoTubeNode=framesNode.addChild('topoTube')
    topoTubeNode.addObject('EdgeSetTopologyContainer', edges=parameters.backboneEdges, name='edgeContainer')
    topoTubeNode.addObject('EdgeSetTopologyModifier', name='edgeModif')
    topoTubeNode.addObject('BeamInterpolation', name='TubeInterpolation', radius=parameters.sections[0].backbone.external_radius, innerRadius=parameters.sections[0].backbone.inner_radius, straight=1, defaultYoungModulus=parameters.sections[0].backbone.youngModulus, defaultPoissonRatio = 0.3)
    topoTubeNode.addObject('AdaptiveBeamForceFieldAndMass', massDensity=parameters.sections[0].backbone.volume_mass)
   
    topoRibNode=framesNode.addChild('topoRibs')
    topoRibNode.addObject('EdgeSetTopologyContainer',  edges=parameters.ribsEdges,name='edgecont')
    topoRibNode.addObject('BeamInterpolation',name = 'RibsInterpolation', crossSectionShape='rectangular', lengthY=parameters.sections[0].ribs[0].width, lengthZ=parameters.sections[0].ribs[0].thickness, straight=0, defaultYoungModulus=parameters.sections[0].ribs[0].youngModulus, DOF0TransformNode0= parameters.dof0TransformeNode0, DOF1TransformNode1 = parameters.dof1TransformeNode1, dofsAndBeamsAligned=0)
    topoRibNode.addObject('AdaptiveBeamForceFieldAndMass', massDensity=parameters.sections[0].ribs[0].volumMass)

    #########################################
    # Spring to connect the arm to a point
    ######################################### 

    framesNode.addObject('RestShapeSpringsForceField', points=parameters.backboneEdges[0][0], external_points = index, external_rest_shape = base.getLinkPath() ,stiffness=1e10 , angularStiffness=1e14)

    #########################################
    # Base
    ######################################### 

    BaseNode=framesNode.addChild('Base')
    box = [-0.5+translation[0], -parameters.ribsSide[0]-0.5++translation[1], -parameters.ribsHeight[0]-0.5+translation[2],   0.5++translation[0],   parameters.ribsSide[0]+0.5+translation[1] , parameters.ribsHeight[0]+0.5+translation[2] ]
    BaseNode.addObject('BoxROI', name='box', box=box, drawBoxes=1,drawSize = 0.2)

    if (base_translation):
        BaseNode.addObject('PartialFixedConstraint', indices='@box.indices', fixedDirections=[ 0, 1, 1, 1, 1, 1], template='Rigid3d')
    else:
        BaseNode.addObject('FixedConstraint', indices='@box.indices', template='Rigid3d')

    # createion of a sliding actuator at the base of the robot
    mobileBase = framesNode.addChild('mobileBase')
    mobileBase.addObject('MechanicalObject',name ='base', position=translation, template='Vec3')
    
    if not inverse :
        mobileBase.addObject('RestShapeSpringsForceField', points='@box.indices', stiffness=1e5 , angularStiffness=1e14)

    mobileBase.addObject('RigidMapping', index=parameters.backboneEdges[0][0])
    
    if (base_translation):
        if (inverse):
            mobileBase.addObject('SlidingActuator', direction=[ 1, 0, 0], indices=0, template='Vec3', maxPositiveDisp=parameters.base.maxDisplacement, maxNegativeDisp=parameters.base.maxDisplacement, maxDispVariation=parameters.base.maxSpeed*dt, displacement=-parameters.base.maxDisplacement)
        # else:
            # mobileBase.addObject('SlidingConstraint', direction=[ 1, 0, 0], indices=0,template='Vec3')  # direction=[ 1, 0, 0], indices=0,      

    #########################################
    # Cables
    ######################################### 
    
    # creation of the cables
    
    constraintPoints  = framesNode.addChild('constraintPoints')
    constraintPoints.addObject('MechanicalObject', position=parameters.positionConstraint)
    test= constraintPoints.addObject('RigidMapping', rigidIndexPerPoint = parameters.rigidMapIndices, globalToLocalCoords=0) 
    test.init()

    if all_sections:
        a = len(parameters.constraintIndices)
    else :
        a = parameters.numCables[0]

    for c in range(a):
        if (inverse):
            cable = constraintPoints.addObject('CableActuator', name='Section'+str(c//3+1)+'Cable'+str(c%3+1), indices=parameters.constraintIndices[c],  hasPullPoint=0, maxForce=parameters.cables.forceMax, minForce=0,maxPositiveDisp=parameters.cables.stroke/2-parameters.cables.neutralPosition[c],maxNegativeDisp=parameters.cables.stroke/2+parameters.cables.neutralPosition[c])
        else:
            cable = constraintPoints.addObject('CableConstraint', name ='Section'+str(c//3+1)+'Cable'+str(c%3+1), indices=parameters.constraintIndices[c], hasPullPoint=0, valueType=typeControl, value=0)        

    return parameters