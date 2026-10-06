#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is the Exudyn item interface
# 
# Details:  automatically generated file for conversion of item (node, object, marker, ...) data to dictionaries
# 
# Author:   Johannes Gerstmayr
# Date:     2019-07-01 (first created)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn #for exudyn.InvalidIndex() and other exudyn native structures needed in RigidBodySpringDamper
import numpy as np
import copy
from typing import Protocol, Union 


#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'CopyDictLevel1', 'IIDiagMatrix', 'CheckForValidUInt', 'CheckForValidPInt',
    'CheckForValidUReal', 'CheckForValidPReal', 'IsValidNumber', 'CheckForValidNumpyArray',
    'userFunctionArgsDict', 'ObjectGroundGraphicsDataUserFunction',
    'ObjectRigidBodyGraphicsDataUserFunction', 'ObjectRigidBody2DGraphicsDataUserFunction',
    'ObjectGenericODE2ForceUserFunction', 'ObjectGenericODE2MassMatrixUserFunction',
    'ObjectGenericODE2JacobianUserFunction', 'ObjectGenericODE2GraphicsDataUserFunction',
    'ObjectGenericODE1RhsUserFunction', 'ObjectKinematicTreeForceUserFunction',
    'ObjectFFRFForceUserFunction', 'ObjectFFRFMassMatrixUserFunction',
    'ObjectFFRFreducedOrderForceUserFunction', 'ObjectFFRFreducedOrderMassMatrixUserFunction',
    'ObjectANCFCable2DAxialForceUserFunction', 'ObjectANCFCable2DBendingMomentUserFunction',
    'ObjectConnectorSpringDamperSpringForceUserFunction',
    'ObjectConnectorCartesianSpringDamperSpringForceUserFunction',
    'ObjectConnectorRigidBodySpringDamperSpringForceTorqueUserFunction',
    'ObjectConnectorRigidBodySpringDamperPostNewtonStepUserFunction',
    'ObjectConnectorLinearSpringDamperSpringForceUserFunction',
    'ObjectConnectorTorsionalSpringDamperSpringTorqueUserFunction',
    'ObjectConnectorCoordinateSpringDamperSpringForceUserFunction',
    'ObjectConnectorCoordinateSpringDamperExtSpringForceUserFunction',
    'ObjectConnectorCoordinateOffsetUserFunction', 'ObjectConnectorCoordinateOffsetUserFunction_t',
    'ObjectConnectorCoordinateVectorConstraintUserFunction',
    'ObjectConnectorCoordinateVectorJacobianUserFunction', 'ObjectJointGenericOffsetUserFunction',
    'ObjectJointGenericOffsetUserFunction_t', 'LoadForceVectorLoadVectorUserFunction',
    'LoadTorqueVectorLoadVectorUserFunction', 'LoadMassProportionalLoadVectorUserFunction',
    'LoadCoordinateLoadUserFunction', 'SensorUserFunctionSensorUserFunction', 'VNodePoint',
    'NodePoint', 'Point', 'VPoint', 'VNodePoint2D', 'NodePoint2D', 'Point2D', 'VPoint2D',
    'VNodeRigidBodyEP', 'NodeRigidBodyEP', 'RigidEP', 'VRigidEP', 'VNodeRigidBodyRxyz',
    'NodeRigidBodyRxyz', 'RigidRxyz', 'VRigidRxyz', 'VNodeRigidBodyRotVecLG',
    'NodeRigidBodyRotVecLG', 'RigidRotVecLG', 'VRigidRotVecLG', 'VNodeRigidBody2D',
    'NodeRigidBody2D', 'Rigid2D', 'VRigid2D', 'VNode1D', 'Node1D', 'VNodePoint2DSlope1',
    'NodePoint2DSlope1', 'Point2DS1', 'VPoint2DS1', 'VNodePointSlope1', 'NodePointSlope1',
    'VNodePointSlope12', 'NodePointSlope12', 'VNodePointSlope23', 'NodePointSlope23',
    'VNodeGenericODE2', 'NodeGenericODE2', 'VNodeGenericODE1', 'NodeGenericODE1', 'VNodeGenericAE',
    'NodeGenericAE', 'VNodeGenericData', 'NodeGenericData', 'VNodePointGround', 'NodePointGround',
    'PointGround', 'VPointGround', 'VObjectGround', 'ObjectGround', 'VObjectMassPoint',
    'ObjectMassPoint', 'MassPoint', 'VMassPoint', 'VObjectMassPoint2D', 'ObjectMassPoint2D',
    'MassPoint2D', 'VMassPoint2D', 'VObjectMass1D', 'ObjectMass1D', 'Mass1D', 'VMass1D',
    'VObjectRotationalMass1D', 'ObjectRotationalMass1D', 'Rotor1D', 'VRotor1D', 'VObjectRigidBody',
    'ObjectRigidBody', 'RigidBody', 'VRigidBody', 'VObjectRigidBody2D', 'ObjectRigidBody2D',
    'RigidBody2D', 'VRigidBody2D', 'VObjectGenericODE2', 'ObjectGenericODE2', 'VObjectGenericODE1',
    'ObjectGenericODE1', 'VObjectKinematicTree', 'ObjectKinematicTree', 'KinematicTree',
    'VKinematicTree', 'VObjectFFRF', 'ObjectFFRF', 'VObjectFFRFreducedOrder',
    'ObjectFFRFreducedOrder', 'CMSobject', 'VCMSobject', 'VObjectANCFCable', 'ObjectANCFCable',
    'Cable', 'VCable', 'VObjectANCFCable2D', 'ObjectANCFCable2D', 'Cable2D', 'VCable2D',
    'VObjectALEANCFCable2D', 'ObjectALEANCFCable2D', 'ALECable2D', 'VALECable2D', 'VObjectANCFBeam',
    'ObjectANCFBeam', 'ANCFBeam', 'VANCFBeam', 'VObjectBeamGeometricallyExact2D',
    'ObjectBeamGeometricallyExact2D', 'Beam2D', 'VBeam2D', 'VObjectBeamGeometricallyExact',
    'ObjectBeamGeometricallyExact', 'Beam3D', 'VBeam3D', 'VObjectANCFThinPlate',
    'ObjectANCFThinPlate', 'VObjectConnectorSpringDamper', 'ObjectConnectorSpringDamper',
    'SpringDamper', 'VSpringDamper', 'VObjectConnectorCartesianSpringDamper',
    'ObjectConnectorCartesianSpringDamper', 'CartesianSpringDamper', 'VCartesianSpringDamper',
    'VObjectConnectorRigidBodySpringDamper', 'ObjectConnectorRigidBodySpringDamper',
    'RigidBodySpringDamper', 'VRigidBodySpringDamper', 'VObjectConnectorLinearSpringDamper',
    'ObjectConnectorLinearSpringDamper', 'LinearSpringDamper', 'VLinearSpringDamper',
    'VObjectConnectorTorsionalSpringDamper', 'ObjectConnectorTorsionalSpringDamper',
    'TorsionalSpringDamper', 'VTorsionalSpringDamper', 'VObjectConnectorCoordinateSpringDamper',
    'ObjectConnectorCoordinateSpringDamper', 'CoordinateSpringDamper', 'VCoordinateSpringDamper',
    'VObjectConnectorCoordinateSpringDamperExt', 'ObjectConnectorCoordinateSpringDamperExt',
    'CoordinateSpringDamperExt', 'VCoordinateSpringDamperExt', 'VObjectConnectorGravity',
    'ObjectConnectorGravity', 'ConnectorGravity', 'VConnectorGravity',
    'VObjectConnectorHydraulicActuatorSimple', 'ObjectConnectorHydraulicActuatorSimple',
    'HydraulicActuatorSimple', 'VHydraulicActuatorSimple', 'VObjectConnectorReevingSystemSprings',
    'ObjectConnectorReevingSystemSprings', 'ReevingSystemSprings', 'VReevingSystemSprings',
    'VObjectConnectorDistance', 'ObjectConnectorDistance', 'DistanceConstraint',
    'VDistanceConstraint', 'VObjectConnectorCoordinate', 'ObjectConnectorCoordinate',
    'CoordinateConstraint', 'VCoordinateConstraint', 'VObjectConnectorCoordinateVector',
    'ObjectConnectorCoordinateVector', 'CoordinateVectorConstraint', 'VCoordinateVectorConstraint',
    'VObjectConnectorRollingDiscPenalty', 'ObjectConnectorRollingDiscPenalty', 'RollingDiscPenalty',
    'VRollingDiscPenalty', 'VObjectContactConvexRoll', 'ObjectContactConvexRoll',
    'VObjectContactCoordinate', 'ObjectContactCoordinate', 'VObjectContactCircleCable2D',
    'ObjectContactCircleCable2D', 'VObjectContactFrictionCircleCable2D',
    'ObjectContactFrictionCircleCable2D', 'VObjectContactSphereSphere', 'ObjectContactSphereSphere',
    'VObjectContactSphereTorus', 'ObjectContactSphereTorus', 'VObjectContactSphereTriangle',
    'ObjectContactSphereTriangle', 'VObjectContactCurveCircles', 'ObjectContactCurveCircles',
    'CamFollowerContactPlanar', 'VCamFollowerContactPlanar', 'VObjectJointGeneric',
    'ObjectJointGeneric', 'GenericJoint', 'VGenericJoint', 'VObjectJointRevoluteZ',
    'ObjectJointRevoluteZ', 'RevoluteJointZ', 'VRevoluteJointZ', 'VObjectJointPrismaticX',
    'ObjectJointPrismaticX', 'PrismaticJointX', 'VPrismaticJointX', 'VObjectJointSpherical',
    'ObjectJointSpherical', 'SphericalJoint', 'VSphericalJoint', 'VObjectJointRollingDisc',
    'ObjectJointRollingDisc', 'RollingDiscJoint', 'VRollingDiscJoint', 'VObjectJointRevolute2D',
    'ObjectJointRevolute2D', 'RevoluteJoint2D', 'VRevoluteJoint2D', 'VObjectJointPrismatic2D',
    'ObjectJointPrismatic2D', 'PrismaticJoint2D', 'VPrismaticJoint2D', 'VObjectJointSliding',
    'ObjectJointSliding', 'SlidingJoint', 'VSlidingJoint', 'VObjectJointSliding2D',
    'ObjectJointSliding2D', 'SlidingJoint2D', 'VSlidingJoint2D', 'VObjectJointALEMoving2D',
    'ObjectJointALEMoving2D', 'ALEMovingJoint2D', 'VALEMovingJoint2D', 'VMarkerBodyMass',
    'MarkerBodyMass', 'VMarkerBodyPosition', 'MarkerBodyPosition', 'VMarkerBodyRigid',
    'MarkerBodyRigid', 'VMarkerNodePosition', 'MarkerNodePosition', 'VMarkerNodeRigid',
    'MarkerNodeRigid', 'VMarkerNodeCoordinate', 'MarkerNodeCoordinate', 'VMarkerNodeCoordinates',
    'MarkerNodeCoordinates', 'VMarkerNodeODE1Coordinate', 'MarkerNodeODE1Coordinate',
    'VMarkerNodeRotationCoordinate', 'MarkerNodeRotationCoordinate',
    'VMarkerBodiesRelativeTranslationCoordinate', 'MarkerBodiesRelativeTranslationCoordinate',
    'VMarkerBodiesRelativeRotationCoordinate', 'MarkerBodiesRelativeRotationCoordinate',
    'VMarkerSuperElementPosition', 'MarkerSuperElementPosition', 'VMarkerSuperElementRigid',
    'MarkerSuperElementRigid', 'VMarkerKinematicTreeRigid', 'MarkerKinematicTreeRigid',
    'VMarkerObjectODE2Coordinates', 'MarkerObjectODE2Coordinates', 'VMarkerBodyCable2DShape',
    'MarkerBodyCable2DShape', 'VMarkerBodyCable2DCoordinates', 'MarkerBodyCable2DCoordinates',
    'VMarkerBodyBeamShape', 'MarkerBodyBeamShape', 'VLoadForceVector', 'LoadForceVector', 'Force',
    'VForce', 'VLoadTorqueVector', 'LoadTorqueVector', 'Torque', 'VTorque', 'VLoadMassProportional',
    'LoadMassProportional', 'Gravity', 'VGravity', 'VLoadCoordinate', 'LoadCoordinate',
    'VSensorNode', 'SensorNode', 'VSensorObject', 'SensorObject', 'VSensorBody', 'SensorBody',
    'VSensorSuperElement', 'SensorSuperElement', 'VSensorKinematicTree', 'SensorKinematicTree',
    'VSensorMarker', 'SensorMarker', 'VSensorLoad', 'SensorLoad', 'VSensorUserFunction',
    'SensorUserFunction',
    ]


#helper function for level-1 copy of dicts (for visualization default args!)
#visualization dictionaries (which may be huge, are only flat copied, which is sufficient)
def CopyDictLevel1(originalDict):
    if isinstance(originalDict,dict): #copy only required if default dict is used
        copyDict = {}
        for key, value in originalDict.items():
            copyDict[key] = copy.copy(value)
        return copyDict
    else:
        return originalDict #fast track for everything else

#helper function diagonal matrices, not needing numpy
def IIDiagMatrix(rowsColumns, value):
    m = []
    for i in range(rowsColumns):
        m += [rowsColumns*[0]]
        m[i][i] = value
    return m

    
#helper function to check valid range
def CheckForValidUInt(value, parameterName, objectName):
    if value < 0:
        raise ValueError("Error in "+objectName+": (int) parameter "+parameterName + " may not be negative, but received "+str(value))
        return 0
    return value

#helper function to check valid range
def CheckForValidPInt(value, parameterName, objectName):
    if value <= 0:
        raise ValueError("Error in "+objectName+": (int) parameter "+parameterName + " must be positive (> 0), but received "+str(value))
        return 1 #this position is usually not reached
    return value
    
#helper function to check valid range
def CheckForValidUReal(value, parameterName, objectName):
    if value < 0:
        raise ValueError("Error in "+objectName+": (float) parameter "+parameterName + " may not be negative, but received "+str(value))
        return 0.
    return value

#helper function to check valid range
def CheckForValidPReal(value, parameterName, objectName):
    if value <= 0:
        raise ValueError("Error in "+objectName+": (float) parameter "+parameterName + " must be positive (> 0), but received "+str(value))
        return 1. #this position is usually not reached
    return value

#helper: return True, if x is int, float, np.double, np.integer or similar types that can be automatically casted to pybind11
def IsValidNumber(x):
    if (isinstance(x, float) 
        or isinstance(x, int)
        or isinstance(x, np.double)
        or isinstance(x, np.integer)
        ):
        return True
    return False

#helper function to check valid range
def CheckForValidNumpyArray(value):
    if IsValidNumber(value): 
        return value
    else:
        return np.array(value)


userFunctionArgsDict = {'MainSystem,preStepUserFunction': [['MainSystem', 'Real'], ['mbs', 'arg0'], ['bool']],
        'MainSystem,postStepUserFunction': [['MainSystem', 'Real'], ['mbs', 'arg0'], ['bool']],
        'MainSystem,postNewtonFunction': [['MainSystem', 'Real'], ['mbs', 'arg0'], ['StdVector2D']],
        'ObjectGround,graphicsDataUserFunction': [['MainSystem', 'Index'], ['mbs', 'itemNumber'], ['py::object'], ['ObjectGroundGraphicsDataUserFunction']],
        'ObjectRigidBody,graphicsDataUserFunction': [['MainSystem', 'Index'], ['mbs', 'itemNumber'], ['py::object'], ['ObjectRigidBodyGraphicsDataUserFunction']],
        'ObjectRigidBody2D,graphicsDataUserFunction': [['MainSystem', 'Index'], ['mbs', 'itemNumber'], ['py::object'], ['ObjectRigidBody2DGraphicsDataUserFunction']],
        'ObjectGenericODE2,forceUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector'], ['mbs', 't', 'itemNumber', 'q', 'q_t'], ['StdVector'], ['ObjectGenericODE2ForceUserFunction']],
        'ObjectGenericODE2,massMatrixUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector'], ['mbs', 't', 'itemNumber', 'q', 'q_t'], ['py::object'], ['ObjectGenericODE2MassMatrixUserFunction']],
        'ObjectGenericODE2,jacobianUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector', 'Real', 'Real'], ['mbs', 't', 'itemNumber', 'q', 'q_t', 'fODE2', 'fODE2_t'], ['py::object'], ['ObjectGenericODE2JacobianUserFunction']],
        'ObjectGenericODE2,graphicsDataUserFunction': [['MainSystem', 'Index'], ['mbs', 'itemNumber'], ['py::object'], ['ObjectGenericODE2GraphicsDataUserFunction']],
        'ObjectGenericODE1,rhsUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector'], ['mbs', 't', 'itemNumber', 'q'], ['StdVector'], ['ObjectGenericODE1RhsUserFunction']],
        'ObjectKinematicTree,forceUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector'], ['mbs', 't', 'itemNumber', 'q', 'q_t'], ['StdVector'], ['ObjectKinematicTreeForceUserFunction']],
        'ObjectFFRF,forceUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector'], ['mbs', 't', 'itemNumber', 'q', 'q_t'], ['StdVector'], ['ObjectFFRFForceUserFunction']],
        'ObjectFFRF,massMatrixUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector'], ['mbs', 't', 'itemNumber', 'q', 'q_t'], ['NumpyMatrix'], ['ObjectFFRFMassMatrixUserFunction']],
        'ObjectFFRFreducedOrder,forceUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector'], ['mbs', 't', 'itemNumber', 'q', 'q_t'], ['StdVector'], ['ObjectFFRFreducedOrderForceUserFunction']],
        'ObjectFFRFreducedOrder,massMatrixUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector'], ['mbs', 't', 'itemNumber', 'q', 'q_t'], ['NumpyMatrix'], ['ObjectFFRFreducedOrderMassMatrixUserFunction']],
        'ObjectANCFCable2D,axialForceUserFunction': [['MainSystem', 'Real', 'Index', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real'], ['mbs', 't', 'itemNumber', 'axialPositionNormalized', 'axialStrain', 'axialStrain_t', 'axialStrainRef', 'axialStiffness', 'axialDamping', 'curvature', 'curvature_t', 'curvatureRef'], ['Real'], ['ObjectANCFCable2DAxialForceUserFunction']],
        'ObjectANCFCable2D,bendingMomentUserFunction': [['MainSystem', 'Real', 'Index', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real'], ['mbs', 't', 'itemNumber', 'axialPositionNormalized', 'curvature', 'curvature_t', 'curvatureRef', 'bendingStiffness', 'bendingDamping', 'axialStrain', 'axialStrain_t', 'axialStrainRef'], ['Real'], ['ObjectANCFCable2DBendingMomentUserFunction']],
        'ObjectConnectorSpringDamper,springForceUserFunction': [['MainSystem', 'Real', 'Index', 'Real', 'Real', 'Real', 'Real', 'Real'], ['mbs', 't', 'itemNumber', 'deltaL', 'deltaL_t', 'stiffness', 'damping', 'force'], ['Real'], ['ObjectConnectorSpringDamperSpringForceUserFunction']],
        'ObjectConnectorCartesianSpringDamper,springForceUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector3D', 'StdVector3D', 'StdVector3D', 'StdVector3D', 'StdVector3D'], ['mbs', 't', 'itemNumber', 'displacement', 'velocity', 'stiffness', 'damping', 'offset'], ['StdVector3D'], ['ObjectConnectorCartesianSpringDamperSpringForceUserFunction']],
        'ObjectConnectorRigidBodySpringDamper,springForceTorqueUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector3D', 'StdVector3D', 'StdVector3D', 'StdVector3D', 'StdMatrix6D', 'StdMatrix6D', 'StdMatrix3D', 'StdMatrix3D', 'StdVector6D'], ['mbs', 't', 'itemNumber', 'displacement', 'rotation', 'velocity', 'angularVelocity', 'stiffness', 'damping', 'rotJ0', 'rotJ1', 'offset'], ['StdVector6D'], ['ObjectConnectorRigidBodySpringDamperSpringForceTorqueUserFunction']],
        'ObjectConnectorRigidBodySpringDamper,postNewtonStepUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector3D', 'StdVector3D', 'StdVector3D', 'StdVector3D', 'StdMatrix6D', 'StdMatrix6D', 'StdMatrix3D', 'StdMatrix3D', 'StdVector6D'], ['mbs', 't', 'itemNumber', 'dataCoordinates', 'displacement', 'rotation', 'velocity', 'angularVelocity', 'stiffness', 'damping', 'rotJ0', 'rotJ1', 'offset'], ['StdVector'], ['ObjectConnectorRigidBodySpringDamperPostNewtonStepUserFunction']],
        'ObjectConnectorLinearSpringDamper,springForceUserFunction': [['MainSystem', 'Real', 'Index', 'Real', 'Real', 'Real', 'Real', 'Real'], ['mbs', 't', 'itemNumber', 'displacement', 'velocity', 'stiffness', 'damping', 'offset'], ['Real'], ['ObjectConnectorLinearSpringDamperSpringForceUserFunction']],
        'ObjectConnectorTorsionalSpringDamper,springTorqueUserFunction': [['MainSystem', 'Real', 'Index', 'Real', 'Real', 'Real', 'Real', 'Real'], ['mbs', 't', 'itemNumber', 'rotation', 'angularVelocity', 'stiffness', 'damping', 'offset'], ['Real'], ['ObjectConnectorTorsionalSpringDamperSpringTorqueUserFunction']],
        'ObjectConnectorCoordinateSpringDamper,springForceUserFunction': [['MainSystem', 'Real', 'Index', 'Real', 'Real', 'Real', 'Real', 'Real'], ['mbs', 't', 'itemNumber', 'displacement', 'velocity', 'stiffness', 'damping', 'offset'], ['Real'], ['ObjectConnectorCoordinateSpringDamperSpringForceUserFunction']],
        'ObjectConnectorCoordinateSpringDamperExt,springForceUserFunction': [['MainSystem', 'Real', 'Index', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real'], ['mbs', 't', 'itemNumber', 'displacement', 'velocity', 'stiffness', 'damping', 'offset', 'velocityOffset', 'dynamicFrictionForce', 'staticFrictionOffsetForce', 'exponentialDecayStatic', 'viscousFrictionFactor', 'frictionProportionalZone'], ['Real'], ['ObjectConnectorCoordinateSpringDamperExtSpringForceUserFunction']],
        'ObjectConnectorCoordinate,offsetUserFunction': [['MainSystem', 'Real', 'Index', 'Real'], ['mbs', 't', 'itemNumber', 'lOffset'], ['Real'], ['ObjectConnectorCoordinateOffsetUserFunction']],
        'ObjectConnectorCoordinate,offsetUserFunction_t': [['MainSystem', 'Real', 'Index', 'Real'], ['mbs', 't', 'itemNumber', 'lOffset'], ['Real'], ['ObjectConnectorCoordinateOffsetUserFunction_t']],
        'ObjectConnectorCoordinateVector,constraintUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector', 'bool'], ['mbs', 't', 'itemNumber', 'q', 'q_t', 'velocityLevel'], ['StdVector'], ['ObjectConnectorCoordinateVectorConstraintUserFunction']],
        'ObjectConnectorCoordinateVector,jacobianUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector', 'bool'], ['mbs', 't', 'itemNumber', 'q', 'q_t', 'velocityLevel'], ['py::object'], ['ObjectConnectorCoordinateVectorJacobianUserFunction']],
        'ObjectJointGeneric,offsetUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector6D'], ['mbs', 't', 'itemNumber', 'offsetUserFunctionParameters'], ['StdVector6D'], ['ObjectJointGenericOffsetUserFunction']],
        'ObjectJointGeneric,offsetUserFunction_t': [['MainSystem', 'Real', 'Index', 'StdVector6D'], ['mbs', 't', 'itemNumber', 'offsetUserFunctionParameters'], ['StdVector6D'], ['ObjectJointGenericOffsetUserFunction_t']],
        'LoadForceVector,loadVectorUserFunction': [['MainSystem', 'Real', 'StdVector3D'], ['mbs', 't', 'loadVector'], ['StdVector3D'], ['LoadForceVectorLoadVectorUserFunction']],
        'LoadTorqueVector,loadVectorUserFunction': [['MainSystem', 'Real', 'StdVector3D'], ['mbs', 't', 'loadVector'], ['StdVector3D'], ['LoadTorqueVectorLoadVectorUserFunction']],
        'LoadMassProportional,loadVectorUserFunction': [['MainSystem', 'Real', 'StdVector3D'], ['mbs', 't', 'loadVector'], ['StdVector3D'], ['LoadMassProportionalLoadVectorUserFunction']],
        'LoadCoordinate,loadUserFunction': [['MainSystem', 'Real', 'Real'], ['mbs', 't', 'load'], ['Real'], ['LoadCoordinateLoadUserFunction']],
        'SensorUserFunction,sensorUserFunction': [['MainSystem', 'Real', 'StdArrayIndex', 'StdVector', 'ConfigurationType'], ['mbs', 't', 'sensorNumbers', 'factors', 'configuration'], ['StdVector'], ['SensorUserFunctionSensorUserFunction']]}


class ObjectGroundGraphicsDataUserFunction(Protocol):
    """A user function, which is called by the visualization thread in order to draw user-defined objects.
    
    The function can be used to generate any ``BodyGraphicsData``, see Section sec-graphicsdata.
    Use ``exudyn.graphics`` functions, see Section sec-module-graphics, to create more complicated objects.
    Note that ``graphicsDataUserFunction`` needs to copy lots of data and is therefore
    inefficient and only designed to enable simpler tests, but not large scale problems.
    
    Args:
        mbs (exudyn.MainSystem): provides reference to mbs, which can be used in user function to access all data of the object

        itemNumber (int): integer number of the object in mbs, allowing easy access

    Returns:
        list: list of ``GraphicsData`` dictionaries, see Section sec-graphicsdata
    """
    def __call__(self, mbs: exudyn.MainSystem, itemNumber: int) -> list: ...

class ObjectRigidBodyGraphicsDataUserFunction(Protocol):
    """A user function, which is called by the visualization thread in order to draw user-defined objects.
    
    The function can be used to generate any ``BodyGraphicsData``, see Section sec-graphicsdata.
    Use ``exudyn.graphics`` functions, see Section sec-module-graphics, to create more complicated objects.
    Note that ``graphicsDataUserFunction`` needs to copy lots of data and is therefore
    inefficient and only designed to enable simpler tests, but not large scale problems.
    
    For an example for ``graphicsDataUserFunction`` see ObjectGround, sec-item-objectground.
    
    Args:
        mbs (exudyn.MainSystem): provides reference to mbs, which can be used in user function to access all data of the object

        itemNumber (int): integer number of the object in mbs, allowing easy access

    Returns:
        list: list of ``GraphicsData`` dictionaries, see Section sec-graphicsdata
    """
    def __call__(self, mbs: exudyn.MainSystem, itemNumber: int) -> list: ...

class ObjectRigidBody2DGraphicsDataUserFunction(Protocol):
    """A user function, which is called by the visualization thread in order to draw user-defined objects.
    
    The function can be used to generate any ``BodyGraphicsData``, see Section sec-graphicsdata.
    Use ``exudyn.graphics`` functions, see Section sec-module-graphics, to create more complicated objects.
    Note that ``graphicsDataUserFunction`` needs to copy lots of data and is therefore
    inefficient and only designed to enable simpler tests, but not large scale problems.
    
    For an example for ``graphicsDataUserFunction`` see ObjectGround, sec-item-objectground.
    
    Args:
        mbs (exudyn.MainSystem): provides reference to mbs, which can be used in user function to access all data of the object

        itemNumber (int): integer number of the object in mbs, allowing easy access

    Returns:
        list: list of ``GraphicsData`` dictionaries, see Section sec-graphicsdata
    """
    def __call__(self, mbs: exudyn.MainSystem, itemNumber: int) -> list: ...

class ObjectGenericODE2ForceUserFunction(Protocol):
    """A user function, which computes a force vector depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.
    
    Note that itemNumber represents the index of the ObjectGenericODE2 object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which object belongs

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        q (np.ndarray): object coordinates (e.g., nodal displacement coordinates) in current configuration, without reference values

        q_t (np.ndarray): object velocity coordinates (time derivative of ``q``) in current configuration

    Returns:
        np.ndarray: returns force vector for object
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, q: np.ndarray, q_t: np.ndarray) -> np.ndarray: ...

class ObjectGenericODE2MassMatrixUserFunction(Protocol):
    """A user function, which computes a mass matrix depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which object belongs to

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        q (np.ndarray): object coordinates (e.g., nodal displacement coordinates) in current configuration, without reference values

        q_t (np.ndarray): object velocity coordinates (time derivative of ``q``) in current configuration

    Returns:
        exudyn.MatrixContainer: returns mass matrix for object, as exu.MatrixContainer, numpy array or list of lists; use MatrixContainer sparse format for larger matrices to speed up computations.
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, q: np.ndarray, q_t: np.ndarray) -> exudyn.MatrixContainer: ...

class ObjectGenericODE2JacobianUserFunction(Protocol):
    """A user function, which computes the jacobian of the LHS of the equations of motion, depending on current time, states of object and two.
    
    factors which are used to distinguish between position level and velocity level derivatives.
    Can be used to create any kind of mechanical system by using the object states.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which object belongs to

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        q (np.ndarray): object coordinates (e.g., nodal displacement coordinates) in current configuration, without reference values

        q_t (np.ndarray): object velocity coordinates (time derivative of ``q``) in current configuration

        fODE2 (float): factor to be multiplied with the position level jacobian, see {eq}``eq-objectgenericode2-jac``

        fODE2_t (float): factor to be multiplied with the velocity level jacobian, see {eq}``eq-objectgenericode2-jac``

    Returns:
        exudyn.MatrixContainer: returns special jacobian for object, as exu.MatrixContainer, numpy array or list of lists; use MatrixContainer sparse format for larger matrices to speed up computations; NOTE that the format of returnValue must AGREE with (dense/sparse triplet) format of stiffnessMatrix and dampingMatrix; sparse triplets MAY NOT contain zero values!
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, q: np.ndarray, q_t: np.ndarray, fODE2: float, fODE2_t: float) -> exudyn.MatrixContainer: ...

class ObjectGenericODE2GraphicsDataUserFunction(Protocol):
    """A user function, which is called by the visualization thread in order to draw user-defined objects.
    
    The function can be used to generate any ``BodyGraphicsData``, see Section sec-graphicsdata.
    Use ``exudyn.graphics`` functions, see Section sec-module-graphics, to create more complicated objects.
    Note that ``graphicsDataUserFunction`` needs to copy lots of data and is therefore
    inefficient and only designed to enable simpler tests, but not large scale problems.
    
    For an example for ``graphicsDataUserFunction`` see ObjectGround, sec-item-objectground.
    
    Args:
        mbs (exudyn.MainSystem): provides reference to mbs, which can be used in user function to access all data of the object

        itemNumber (int): integer number of the object in mbs, allowing easy access

    Returns:
        list: list of ``GraphicsData`` dictionaries, see Section sec-graphicsdata
    """
    def __call__(self, mbs: exudyn.MainSystem, itemNumber: int) -> list: ...

class ObjectGenericODE1RhsUserFunction(Protocol):
    """A user function, which computes a RHS vector depending on current time and states of the object.
    
    Can be used to create any kind of first order system, especially state space equations (inputs are added via CoordinateLoads to every node).
    Note that itemNumber represents the index of the ObjectGenericODE1 object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which object belongs

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        q (np.ndarray): object coordinates (composed from ODE1 nodal coordinates) in current configuration, without reference values

    Returns:
        np.ndarray: returns force vector for object
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, q: np.ndarray) -> np.ndarray: ...

class ObjectKinematicTreeForceUserFunction(Protocol):
    """A user function, which computes a force vector applied to the joint coordinates depending on current time and states of object.
    
    Note that itemNumber represents the index of the ObjectKinematicTree object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which object belongs

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        q (np.ndarray): object coordinates (e.g., nodal displacement coordinates) in current configuration, without reference values

        q_t (np.ndarray): object velocity coordinates (time derivative of ``q``) in current configuration

    Returns:
        np.ndarray: returns force vector for object
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, q: np.ndarray, q_t: np.ndarray) -> np.ndarray: ...

class ObjectFFRFForceUserFunction(Protocol):
    """A user function, which computes a force vector depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which object belongs

        t (float): current time in mbs

        itemNumber (int): integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        q (np.ndarray): object coordinates (nodal displacement coordinates of rigid body and mesh nodes) in current configuration, without reference values

        q_t (np.ndarray): object velocity coordinates (time derivative of ``q``) in current configuration

    Returns:
        np.ndarray: returns force vector for object
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, q: np.ndarray, q_t: np.ndarray) -> np.ndarray: ...

class ObjectFFRFMassMatrixUserFunction(Protocol):
    """A user function, which computes a mass matrix depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which object belongs

        t (float): current time in mbs

        itemNumber (int): integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        q (np.ndarray): object coordinates (nodal displacement coordinates of rigid body and mesh nodes) in current configuration, without reference values

        q_t (np.ndarray): object velocity coordinates (time derivative of ``q``) in current configuration

    Returns:
        np.ndarray: returns mass matrix for object
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, q: np.ndarray, q_t: np.ndarray) -> np.ndarray: ...

class ObjectFFRFreducedOrderForceUserFunction(Protocol):
    """A user function, which computes a force vector depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.
    
    Note that itemNumber represents the index of the ObjectFFRFreducedOrder object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which object belongs

        t (float): current time in mbs

        itemNumber (int): integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        q (np.ndarray): FFRF object coordinates (rigid body coordinates and reduced coordinates in a list) in current configuration, without reference values

        q_t (np.ndarray): object velocity coordinates (time derivatives of ``q``) in current configuration

    Returns:
        np.ndarray: returns force vector for object
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, q: np.ndarray, q_t: np.ndarray) -> np.ndarray: ...

class ObjectFFRFreducedOrderMassMatrixUserFunction(Protocol):
    """A user function, which computes a mass matrix depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which object belongs

        t (float): current time in mbs

        itemNumber (int): integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        q (np.ndarray): FFRF object coordinates (rigid body coordinates and reduced coordinates in a list) in current configuration, without reference values

        q_t (np.ndarray): object velocity coordinates (time derivatives of ``q``) in current configuration

    Returns:
        np.ndarray: returns mass matrix for object
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, q: np.ndarray, q_t: np.ndarray) -> np.ndarray: ...

class ObjectANCFCable2DAxialForceUserFunction(Protocol):
    r"""A user function, which computes the axial force depending on time, strains and curvatures and.
    
    object parameters (stiffness, damping).
    The object variables are provided to the function using the current values of the ANCFCable2D object.
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    **NOTE:** this function has a different interface as compared to the bending moment function.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which object belongs

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        axialPositionNormalized (float): axial position at the cable where the user function is evaluated; range is [0,1]

        axialStrain (float): :math:`\varepsilon`

        axialStrain_t (float): :math:`\varepsilon_t`

        axialStrainRef (float): :math:`\varepsilon_0 + f\cRef \cdot \varepsilon\cRef`

        axialStiffness (float): as given in object parameters

        axialDamping (float): as given in object parameters

        curvature (float): :math:`K`

        curvature_t (float): :math:`\dot K`

        curvatureRef (float): :math:`K_0 + f\cRef \cdot K\cRef`

    Returns:
        float: scalar value of computed axial force
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, axialPositionNormalized: float, axialStrain: float, axialStrain_t: float, axialStrainRef: float, axialStiffness: float, axialDamping: float, curvature: float, curvature_t: float, curvatureRef: float) -> float: ...

class ObjectANCFCable2DBendingMomentUserFunction(Protocol):
    r"""A user function, which computes the bending moment depending on time, strains and curvatures and.
    
    object parameters (stiffness, damping).
    The object variables are provided to the function using the current values of the ANCFCable2D object.
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    **NOTE:** this function has a different interface as compared to the axial force function.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which object belongs

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        axialPositionNormalized (float): axial position at the cable where the user function is evaluated; range is [0,1]

        curvature (float): :math:`K`

        curvature_t (float): :math:`\dot K`

        curvatureRef (float): :math:`K_0 + f\cRef \cdot K\cRef`

        bendingStiffness (float): as given in object parameters

        bendingDamping (float): as given in object parameters

        axialStrain (float): :math:`\varepsilon`

        axialStrain_t (float): :math:`\varepsilon_t`

        axialStrainRef (float): :math:`\varepsilon_0 + f\cRef \cdot \varepsilon\cRef`

    Returns:
        float: scalar value of computed bending moment
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, axialPositionNormalized: float, curvature: float, curvature_t: float, curvatureRef: float, bendingStiffness: float, bendingDamping: float, axialStrain: float, axialStrain_t: float, axialStrainRef: float) -> float: ...

class ObjectConnectorSpringDamperSpringForceUserFunction(Protocol):
    r"""A user function, which computes the spring force depending on time, object variables (deltaL, deltaL_t) and.
    
    object parameters (stiffness, damping, force).
    The object variables are provided to the function using the current values of the SpringDamper object.
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which object belongs

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        deltaL (float): :math:`L-L_0`, spring elongation

        deltaL_t (float): :math:`(\dot L - \dot L_0)`, spring velocity, including offset

        stiffness (float): copied from object

        damping (float): copied from object

        force (float): copied from object; constant force

    Returns:
        float: scalar value of computed spring force
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, deltaL: float, deltaL_t: float, stiffness: float, damping: float, force: float) -> float: ...

class ObjectConnectorCartesianSpringDamperSpringForceUserFunction(Protocol):
    r"""A user function, which computes the 3D spring force vector depending on time, object variables (deltaL, deltaL_t) and object parameters.
    
    (stiffness, damping, force).
    The object variables are provided to the function using the current values of the SpringDamper object.
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs in which underlying item is defined

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        displacement (np.ndarray): :math:`\Delta\! {}^{0}{\mathbf{p}}`

        velocity (np.ndarray): :math:`\Delta\! {}^{0}{\vv}`

        stiffness (np.ndarray): copied from object

        damping (np.ndarray): copied from object

        offset (np.ndarray): copied from object

    Returns:
        np.ndarray: list or numpy array of computed spring force
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, displacement: np.ndarray, velocity: np.ndarray, stiffness: np.ndarray, damping: np.ndarray, offset: np.ndarray) -> np.ndarray: ...

class ObjectConnectorRigidBodySpringDamperSpringForceTorqueUserFunction(Protocol):
    r"""A user function, which computes the 6D spring-damper force-torque vector depending on mbs, time, local quantities.
    
    (displacement, rotation, velocity, angularVelocity, stiffness), which are evaluated at current time, which are relative quantities between
    both markers and which are defined in joint J0 coordinates.
    As relative rotations are defined by Tait-Bryan rotation parameters, it is recommended to use this connector for small relative rotations only
    (except for rotations about one axis).
    Furthermore, the user function contains object parameters (stiffness, damping, rotationMarker0/1, offset).
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    
    Detailed description of the arguments and local quantities:
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs in which underlying item is defined

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        displacement (np.ndarray): :math:`{}^{J0}{\Delta\mathbf{p}}`

        rotation (np.ndarray): :math:`{}^{J0}{\ttheta}`

        velocity (np.ndarray): :math:`{}^{J0}{\Delta\vv}`

        angularVelocity (np.ndarray): :math:`{}^{J0}{\Delta\tomega}`

        stiffness (np.ndarray): copied from object

        damping (np.ndarray): copied from object

        rotJ0 (np.ndarray): rotationMarker0 copied from object

        rotJ1 (np.ndarray): rotationMarker1 copied from object

        offset (np.ndarray): copied from object

    Returns:
        np.ndarray: list or numpy array of computed spring force-torque
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, displacement: np.ndarray, rotation: np.ndarray, velocity: np.ndarray, angularVelocity: np.ndarray, stiffness: np.ndarray, damping: np.ndarray, rotJ0: np.ndarray, rotJ1: np.ndarray, offset: np.ndarray) -> np.ndarray: ...

class ObjectConnectorRigidBodySpringDamperPostNewtonStepUserFunction(Protocol):
    r"""A user function which computes the error of the PostNewtonStep :math:`\varepsilon_{PN}`, a recommended for stepsize reduction :math:`t_{recom}` (use values > 0 to recommend step size or values < 0 else; 0 gives minimum step size).
    
    and the updated dataCoordinates :math:`\dv^k` of ``NodeGenericData`` :math:`n_d`.
    Except from ``dataCoordinates``, the arguments are the same as in ``springForceTorqueUserFunction``.
    The ``postNewtonStepUserFunction`` should be used together with the dataCoordinates in order to implement a active set or switching strategy
    for discontinuous events, such as in contact, friction, plasticity, fracture or similar.
    
    Detailed description of the arguments and local quantities:
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs in which underlying item is defined

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        dataCoordinates (np.ndarray): :math:`\dv^{k-1} = [d_0^{k-1},\; d_1^{k-1},\; \ldots]` for previous post Newton step :math:`k-1`

        displacement (np.ndarray): :math:`{}^{J0}{\Delta\mathbf{p}}`

        rotation (np.ndarray): :math:`{}^{J0}{\ttheta}`

        velocity (np.ndarray): :math:`{}^{J0}{\Delta\vv}`

        angularVelocity (np.ndarray): :math:`{}^{J0}{\Delta\tomega}`

        stiffness (np.ndarray): copied from object

        damping (np.ndarray): copied from object

        rotJ0 (np.ndarray): rotationMarker0 copied from object

        rotJ1 (np.ndarray): rotationMarker1 copied from object

        offset (np.ndarray): copied from object

    Returns:
        np.ndarray: :math:`\left[\varepsilon_{PN},\; t_{recom},\; d_0^{k},\; d_1^{k}, ...\right]` where :math:`k` indicates the current step
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, dataCoordinates: np.ndarray, displacement: np.ndarray, rotation: np.ndarray, velocity: np.ndarray, angularVelocity: np.ndarray, stiffness: np.ndarray, damping: np.ndarray, rotJ0: np.ndarray, rotJ1: np.ndarray, offset: np.ndarray) -> np.ndarray: ...

class ObjectConnectorLinearSpringDamperSpringForceUserFunction(Protocol):
    r"""A user function, which computes the scalar torque depending on mbs, time, local quantities.
    
    (relative displacement, relative velocity), which are evaluated at current time.
    Furthermore, the user function contains object parameters (stiffness, damping, offset).
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    
    Detailed description of the arguments and local quantities:
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs in which underlying item is defined

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        displacement (float): :math:`\Delta x`

        velocity (float): :math:`\Delta v`

        stiffness (float): copied from object

        damping (float): copied from object

        offset (float): copied from object

    Returns:
        float: computed force
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, displacement: float, velocity: float, stiffness: float, damping: float, offset: float) -> float: ...

class ObjectConnectorTorsionalSpringDamperSpringTorqueUserFunction(Protocol):
    r"""A user function, which computes the scalar torque depending on mbs, time, local quantities.
    
    (relative rotation, relative angularVelocity), which are evaluated at current time.
    Furthermore, the user function contains object parameters (stiffness, damping, offset).
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    
    Detailed description of the arguments and local quantities:
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs in which underlying item is defined

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        rotation (float): :math:`\Delta \theta`

        angularVelocity (float): :math:`\Delta \omega`

        stiffness (float): copied from object

        damping (float): copied from object

        offset (float): copied from object

    Returns:
        float: computed torque
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, rotation: float, angularVelocity: float, stiffness: float, damping: float, offset: float) -> float: ...

class ObjectConnectorCoordinateSpringDamperSpringForceUserFunction(Protocol):
    r"""A user function, which computes the scalar spring force depending on time, object variables (displacement, velocity).
    
    and object parameters .
    The object variables are passed to the function using the current values of the CoordinateSpringDamper object.
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs in which underlying item is defined

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        displacement (float): :math:`\Delta q`

        velocity (float): :math:`\Delta v`

        stiffness (float): copied from object

        damping (float): copied from object

        offset (float): copied from object

    Returns:
        float: scalar value of computed force
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, displacement: float, velocity: float, stiffness: float, damping: float, offset: float) -> float: ...

class ObjectConnectorCoordinateSpringDamperExtSpringForceUserFunction(Protocol):
    r"""A user function, which computes the scalar spring force depending on time, object variables (displacement, velocity).
    
    and several object parameters.
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    
    Only a subset of object variables is passed to the function using the current values of the CoordinateSpringDamperExt object.
    For parameters that are not passed via the user function interface, use mbs.GetObject(itemNumber) or, e.g.,
    mbs.GetObjectParameter(itemNumber, 'limitStopsUpper') to obtain these parameters inside the user function.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs in which underlying item is defined

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        displacement (float): :math:`\Delta q`

        velocity (float): :math:`\Delta v`

        stiffness (float): copied from object

        damping (float): copied from object

        offset (float): copied from object

        velocityOffset (float): copied from object

        dynamicFrictionForce (float): copied from object

        staticFrictionOffsetForce (float): copied from object

        exponentialDecayStatic (float): copied from object

        viscousFrictionFactor (float): copied from object

        frictionProportionalZone (float): copied from object, also called regularization velocity or regVel

    Returns:
        float: scalar value of computed force
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, displacement: float, velocity: float, stiffness: float, damping: float, offset: float, velocityOffset: float, dynamicFrictionForce: float, staticFrictionOffsetForce: float, exponentialDecayStatic: float, viscousFrictionFactor: float, frictionProportionalZone: float) -> float: ...

class ObjectConnectorCoordinateOffsetUserFunction(Protocol):
    r"""A user function, which computes scalar offset for the coordinate constraint, e.g., in order to move a node on a prescribed trajectory.
    
    It is NECESSARY to use sufficiently smooth functions, having **initial offsets** consistent with **initial configuration** of bodies,
    either zero or compatible initial offset-velocity, and no initial accelerations.
    The ``offsetUserFunction`` is **ONLY used** in case of static computation or index3 (generalizedAlpha) time integration.
    In order to be on the safe side, provide both  ``offsetUserFunction`` and  ``offsetUserFunction_t``.
    
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    
    The user function gets time and the offset parameter as an input and returns the computed offset:
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs in which underlying item is defined

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        lOffset (float): :math:`l_\mathrm{off}`

    Returns:
        float: computed offset for given time
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, lOffset: float) -> float: ...

class ObjectConnectorCoordinateOffsetUserFunction_t(Protocol):
    r"""A user function, which computes scalar offset **velocity** for the coordinate constraint.
    
    It is NECESSARY to use sufficiently smooth functions, having **initial offset velocities** consistent with **initial velocities** of bodies.
    The ``offsetUserFunction_t`` is used instead of ``offsetUserFunction`` in case of ``velocityLevel = True``,
    or for index2 time integration and needed for computation of initial accelerations in second order implicit time integrators.
    
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    
    The user function gets time and the offset parameter as an input and returns the computed offset velocity:
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs in which underlying item is defined

        t (float): current time in mbs

        itemNumber (int): integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        lOffset (float): :math:`l_\mathrm{off}`

    Returns:
        float: computed offset velocity for given time
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, lOffset: float) -> float: ...

class ObjectConnectorCoordinateVectorConstraintUserFunction(Protocol):
    """A user function, which computes algebraic equations for the connector based on the marker coordinates stored in ``q`` and ``q_t``.
    
    Depending on ``velocityLevel``, the user function needs to compute either the position-level (``velocityLevel=False``) or
    the velocity level (``velocityLevel=True``) constraint equations.
    Note that for Index 2 solvers, the ``constraintUserFunction`` may be called with ``velocityLevel=True`` but ``jacobianUserFunction``
    is called with ``velocityLevel=False``.
    To define the number of algebraic equations, set ``scalingMarker0`` as a ``numpy.zeros((nAE,1))`` array with ``nAE`` being the number algebraic equations.
    The returned vector of ``constraintUserFunction`` must have size ``nAE``.
    
    Note that itemNumber represents the index of the ObjectGenericODE2 object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which object belongs to

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        q (np.ndarray): connector coordinates, subsequently for marker :math:`m0` and marker :math:`m1`, in current configuration

        q_t (np.ndarray): connector velocity coordinates in current configuration

        velocityLevel (bool): velocityLevel as currently stored in connector

    Returns:
        np.ndarray: returns vector (numpy array or list) of evaluated constraint equations for connector
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, q: np.ndarray, q_t: np.ndarray, velocityLevel: bool) -> np.ndarray: ...

class ObjectConnectorCoordinateVectorJacobianUserFunction(Protocol):
    """A user function, which computes the jacobian of the algebraic equations w.r.t. the ODE2 coordiantes (ODE2_t velocity coordinates if ``velocityLevel=True``).
    
    The jacobian needs to exactly represent the derivative of the constraintUserFunction.
    The returned matrix of ``jacobianUserFunction`` must have ``nAE`` rows and ``len(q)`` columns.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which object belongs to

        t (float): current time in mbs

        itemNumber (int): integer number :math:`i_N` of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        q (np.ndarray): connector coordinates, subsequently for marker :math:`m0` and marker :math:`m1`, in current configuration

        q_t (np.ndarray): connector velocity coordinates in current configuration

        velocityLevel (bool): velocityLevel as currently stored in connector

    Returns:
        exudyn.MatrixContainer: returns special jacobian for connector, as exu.MatrixContainer, numpy array or list of lists; use MatrixContainer sparse format for larger matrices to speed up computations; sparse triplets MAY NOT contain zero values!
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, q: np.ndarray, q_t: np.ndarray, velocityLevel: bool) -> exudyn.MatrixContainer: ...

class ObjectJointGenericOffsetUserFunction(Protocol):
    r"""A user function, which computes scalar offset for relative joint translation and joint rotation for the GenericJoint,.
    
    e.g., in order to move or rotate a body on a prescribed trajectory.
    It is NECESSARY to use sufficiently smooth functions, having **initial offsets** consistent with **initial configuration** of bodies,
    either zero or compatible initial offset-velocity, and no initial accelerations.
    The ``offsetUserFunction`` is **ONLY used** in case of static computation or index3 (generalizedAlpha) time integration.
    In order to be on the safe side, provide both  ``offsetUserFunction`` and  ``offsetUserFunction_t``.
    
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    
    The user function gets time and the offsetUserFunctionParameters as an input and returns the computed offset vector
    for all relative translational and rotational joint coordinates:
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs in which underlying item is defined

        t (float): current time in mbs

        itemNumber (int): integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        offsetUserFunctionParameters (np.ndarray): :math:`\mathbf{p}_{par}`, set of parameters which can be freely used in user function

    Returns:
        np.ndarray: computed offset vector for given time
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, offsetUserFunctionParameters: np.ndarray) -> np.ndarray: ...

class ObjectJointGenericOffsetUserFunction_t(Protocol):
    r"""A user function, which computes an offset **velocity** vector for the GenericJoint.
    
    It is NECESSARY to use sufficiently smooth functions, having **initial offset velocities** consistent with **initial velocities** of bodies.
    The ``offsetUserFunction_t`` is used instead of ``offsetUserFunction`` in case of ``velocityLevel = True``,
    or for index2 time integration and needed for computation of initial accelerations in second order implicit time integrators.
    
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    ``mbs.GetObjectParameter(itemNumber, ...)``, see the according description of ``GetObjectParameter``.
    
    The user function gets time and the offsetUserFunctionParameters as an input and returns the computed offset velocity vector
    for all relative translational and rotational joint coordinates:
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs in which underlying item is defined

        t (float): current time in mbs

        itemNumber (int): integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)

        offsetUserFunctionParameters (np.ndarray): :math:`\mathbf{p}_{par}`, set of parameters which can be freely used in user function

    Returns:
        np.ndarray: computed offset velocity vector for given time
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, itemNumber: int, offsetUserFunctionParameters: np.ndarray) -> np.ndarray: ...

class LoadForceVectorLoadVectorUserFunction(Protocol):
    r"""A user function, which computes the force vector depending on time and object parameters, which is hereafter applied to object or node.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which load belongs

        t (float): current time in mbs

        loadVector (np.ndarray): :math:`\fv` copied from object; WARNING: this parameter does not work in combination with static computation, as it is changed by the solver over step time

    Returns:
        np.ndarray: computed force vector
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, loadVector: np.ndarray) -> np.ndarray: ...

class LoadTorqueVectorLoadVectorUserFunction(Protocol):
    r"""A user function, which computes the torque vector depending on time and object parameters, which is hereafter applied to object or node.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which load belongs

        t (float): current time in mbs

        loadVector (np.ndarray): :math:`\ttau` copied from object; WARNING: this parameter does not work in combination with static computation, as it is changed by the solver over step time

    Returns:
        np.ndarray: computed torque vector
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, loadVector: np.ndarray) -> np.ndarray: ...

class LoadMassProportionalLoadVectorUserFunction(Protocol):
    r"""A user function, which computes the mass proporitional load vector depending on time and object parameters, which is hereafter applied to object or node.
    
    Example of user function: functionality same as in ``LoadForceVector``
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which load belongs

        t (float): current time in mbs

        loadVector (np.ndarray): :math:`\bv` copied from object; WARNING: this parameter does not work in combination with static computation, as it is changed by the solver over step time

    Returns:
        np.ndarray: computed load vector
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, loadVector: np.ndarray) -> np.ndarray: ...

class LoadCoordinateLoadUserFunction(Protocol):
    r"""A user function, which computes the scalar load depending on time and the object's ``load`` parameter.
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which load belongs

        t (float): current time in mbs

        load (float): :math:`\bv` copied from object; WARNING: this parameter does not work in combination with static computation, as it is changed by the solver over step time

    Returns:
        float: computed load
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, load: float) -> float: ...

class SensorUserFunctionSensorUserFunction(Protocol):
    """A user function, which computes a sensor output from other sensor outputs (or from generic time dependent functions).
    
    The configuration in general will be the exudyn.ConfigurationType.Current, but others could be used as well except for SensorMarker.
    
    The user function arguments are as follows:
    
    Args:
        mbs (exudyn.MainSystem): provides MainSystem mbs to which object belongs

        t (float): current time in mbs

        sensorNumbers (np.ndarray): list of sensor numbers

        factors (np.ndarray): list of factors that can be freely used for the user function

        configuration (exudyn.ConfigurationType): usually the exudyn.ConfigurationType.Current, but could also be different in user defined functions.

    Returns:
        np.ndarray: returns list or numpy array of sensor output values; size :math:`n_r` is implicitly defined by the returned list and may not be changed during simulation.
    """
    def __call__(self, mbs: exudyn.MainSystem, t: float, sensorNumbers: np.ndarray, factors: np.ndarray, configuration: exudyn.ConfigurationType) -> np.ndarray: ...

#+++++++++++++++++++++++++++++++
#NODE
class VNodePoint:
    """Visualization data for NodePoint.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used; type: float

        color: Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class NodePoint:
    """A 3D point node for point masses or solid finite elements which has 3 displacement degrees of freedom for ODE2.
    
    Args:
        name: node's unique name; type: str

        referenceCoordinates: reference coordinates of node, e.g. ref. coordinates for finite elements; global position of node without displacement; type: [float,float,float]

        initialCoordinates: initial displacement coordinate; type: [float,float,float]

        initialVelocities: initial velocity coordinate; type: [float,float,float]

        visualization: visualization data, see VNodePoint

    Notes:
        Node has/provides the following types: ``Position``

    """
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0.], initialCoordinates = [0.,0.,0.], initialVelocities = [0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.initialVelocities = np.array(initialVelocities)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'Point'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
Point = NodePoint
VPoint = VNodePoint

class VNodePoint2D:
    """Visualization data for NodePoint2D.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used; type: float

        color: Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class NodePoint2D:
    """A 2D point node for point masses or solid finite elements which has 2 displacement degrees of freedom for ODE2.
    
    Args:
        name: node's unique name; type: str

        referenceCoordinates: reference coordinates of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement; type: [float,float]

        initialCoordinates: initial displacement coordinate; type: [float,float]

        initialVelocities: initial velocity coordinate; type: [float,float]

        visualization: visualization data, see VNodePoint2D

    Notes:
        Node has/provides the following types: ``Position2D``, ``Position``

    """
    def __init__(self, name = '', referenceCoordinates = [0.,0.], initialCoordinates = [0.,0.], initialVelocities = [0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.initialVelocities = np.array(initialVelocities)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'Point2D'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
Point2D = NodePoint2D
VPoint2D = VNodePoint2D

class VNodeRigidBodyEP:
    """Visualization data for NodeRigidBodyEP.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used; type: float

        color: Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class NodeRigidBodyEP:
    r"""A 3D rigid body node based on Euler parameters for rigid bodies or beams.
    
    The node has 3 displacement coordinates (representing displacement of reference point :math:`{}^{0}{\rv}`) and four rotation coordinates (Euler parameters = unit quaternions).
    
    Args:
        name: node's unique name; type: str

        referenceCoordinates: reference coordinates (3 position coordinates and 4 Euler parameters) of node ==> e.g. ref. coordinates for finite elements or reference position of rigid body (e.g. for definition of joints); type: array_like

        initialCoordinates: initial displacement coordinates and 4 Euler parameters relative to reference coordinates; type: array_like

        initialVelocities: initial velocity coordinates: time derivatives of initial displacements and Euler parameters; type: array_like

        addConstraintEquation: True: automatically add Euler parameter constraint for node; False: Euler parameter constraint is not added, must be done manually (e.g., with CoordinateVectorConstraint); type: bool

        visualization: visualization data, see VNodeRigidBodyEP

    Notes:
        Node has/provides the following types: ``Position``, ``Orientation``, ``RigidBody``, ``RotationEulerParameters``

    """
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0., 0.,0.,0.,0.], initialCoordinates = [0.,0.,0., 0.,0.,0.,0.], initialVelocities = [0.,0.,0., 0.,0.,0.,0.], addConstraintEquation = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.initialVelocities = np.array(initialVelocities)
        self.addConstraintEquation = addConstraintEquation
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'RigidBodyEP'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'addConstraintEquation', self.addConstraintEquation
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
RigidEP = NodeRigidBodyEP
VRigidEP = VNodeRigidBodyEP

class VNodeRigidBodyRxyz:
    """Visualization data for NodeRigidBodyRxyz.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used; type: float

        color: Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class NodeRigidBodyRxyz:
    r"""A 3D rigid body node based on Euler / Tait-Bryan angles for rigid bodies or beams.
    
    All coordinates lead to second order differential equations; NOTE: this node has a singularity if the second rotation parameter reaches :math:`\psi_1 = (2k-1) \pi/2`, with :math:`k \in \Ncal` or :math:`-k \in \Ncal`.
    
    Args:
        name: node's unique name; type: str

        referenceCoordinates: reference coordinates (3 position and 3 xyz Euler angles) of node ==> e.g. ref. coordinates for finite elements or reference position of rigid body (e.g. for definition of joints); type: array_like

        initialCoordinates: initial displacement coordinates: ux,uy,uz and 3 Euler angles (xyz) relative to reference coordinates; type: array_like

        initialVelocities: initial velocity coordinate: time derivatives of ux,uy,uz and of 3 Euler angles (xyz); type: array_like

        visualization: visualization data, see VNodeRigidBodyRxyz

    Notes:
        Node has/provides the following types: ``Position``, ``Orientation``, ``RigidBody``, ``RotationRxyz``

    """
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0., 0.,0.,0.], initialCoordinates = [0.,0.,0., 0.,0.,0.], initialVelocities = [0.,0.,0., 0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.initialVelocities = np.array(initialVelocities)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'RigidBodyRxyz'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
RigidRxyz = NodeRigidBodyRxyz
VRigidRxyz = VNodeRigidBodyRxyz

class VNodeRigidBodyRotVecLG:
    """Visualization data for NodeRigidBodyRotVecLG.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used; type: float

        color: Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class NodeRigidBodyRotVecLG:
    r"""A 3D rigid body node based on rotation vector and Lie group methods for rigid bodies.
    
    The node has 3 displacement coordinates and three rotation coordinates and can be used in combination with explicit Lie Group time integration methods.
    
    Args:
        name: node's unique name; type: str

        referenceCoordinates: reference coordinates (position and rotation vector :math:`\nu`) of node ==> e.g. ref. coordinates for finite elements or reference position of rigid body (e.g. for definition of joints); type: array_like

        initialCoordinates: initial displacement coordinates :math:`\mathbf{u}` and rotation vector :math:`\nu` relative to reference coordinates; type: array_like

        initialVelocities: initial velocity coordinate: time derivatives of displacement and angular velocity vector; type: array_like

        visualization: visualization data, see VNodeRigidBodyRotVecLG

    Notes:
        Node has/provides the following types: ``Position``, ``Orientation``, ``RigidBody``, ``RotationRotationVector``

    """
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0., 0.,0.,0.], initialCoordinates = [0.,0.,0., 0.,0.,0.], initialVelocities = [0.,0.,0., 0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.initialVelocities = np.array(initialVelocities)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'RigidBodyRotVecLG'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
RigidRotVecLG = NodeRigidBodyRotVecLG
VRigidRotVecLG = VNodeRigidBodyRotVecLG

class VNodeRigidBody2D:
    """Visualization data for NodeRigidBody2D.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used; type: float

        color: Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class NodeRigidBody2D:
    r"""A 2D rigid body node for rigid bodies or beams.
    
    The node has 2 displacement degrees of freedom and one rotation coordinate (rotation around z-axis: :math:`\psi_0`). All coordinates are ODE2, used for second order differetial equations.
    
    Args:
        name: node's unique name; type: str

        referenceCoordinates: reference coordinates (x-pos,y-pos and rotation) of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement; type: [float,float,float]

        initialCoordinates: initial displacement coordinates and angle (relative to reference coordinates); type: [float,float,float]

        initialVelocities: initial velocity coordinates; type: [float,float,float]

        visualization: visualization data, see VNodeRigidBody2D

    Notes:
        Node has/provides the following types: ``Position2D``, ``Orientation2D``, ``Position``, ``Orientation``, ``RigidBody``

    """
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0.], initialCoordinates = [0.,0.,0.], initialVelocities = [0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.initialVelocities = np.array(initialVelocities)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'RigidBody2D'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
Rigid2D = NodeRigidBody2D
VRigid2D = VNodeRigidBody2D

class VNode1D:
    """Visualization data for Node1D."""
    def __init__(self):
        pass

    def __iter__(self):
        yield from ()

    def __repr__(self):
        return str(dict(self))

class Node1D:
    """A node with one ODE2 coordinate for one dimensional (1D) problems.
    
    Use e.g. for scalar dynamic equations (Mass1D) and mass-spring-damper mechanisms, representing either translational or rotational degrees of freedom: in most cases, Node1D is equivalent to NodeGenericODE2 using one coordinate, however, it offers a transformation to 3D translational or rotational motion and allows to couple this node to 2D or 3D bodies.
    
    Args:
        name: node's unique name; type: str

        referenceCoordinates: reference coordinate of node (in vector form); type: array_like

        initialCoordinates: initial displacement coordinate (in vector form); type: array_like

        initialVelocities: initial velocity coordinate (in vector form); type: array_like

        visualization: visualization data, see VNode1D

    Notes:
        Node has/provides the following types: ``GenericODE2``

    """
    def __init__(self, name = '', referenceCoordinates = [0.], initialCoordinates = [0.], initialVelocities = [0.], visualization = {}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.initialVelocities = np.array(initialVelocities)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', '1D'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities

    def __repr__(self):
        return str(dict(self))

class VNodePoint2DSlope1:
    """Visualization data for NodePoint2DSlope1.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used; type: float

        color: Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class NodePoint2DSlope1:
    r"""A 2D point/slope vector node for planar Bernoulli-Euler ANCF (absolute nodal coordinate formulation) beam elements.
    
    The node has 4 displacement degrees of freedom (2 for displacement of point node and 2 for the slope vector 'slopex'); all coordinates lead to second order differential equations; the slope vector defines the directional derivative w.r.t the local axial (x) coordinate, denoted as :math:`()^\prime`; in straight configuration aligned at the global x-axis, the slope vector reads :math:`\rv^\prime=[r_x^\prime\;\;r_y^\prime]^T=[1\;\;0]^T`.
    
    Args:
        name: node's unique name; type: str

        referenceCoordinates: reference coordinates (x-pos,y-pos; x-slopex, y-slopex) of node; global position of node without displacement; type: [float,float,float,float]

        initialCoordinates: initial displacement coordinates: ux, uy and x/y 'displacements' of slopex; type: [float,float,float,float]

        initialVelocities: initial velocity coordinates; type: [float,float,float,float]

        visualization: visualization data, see VNodePoint2DSlope1

    Notes:
        Node has/provides the following types: ``Position2D``, ``Orientation2D``, ``Point2DSlope1``, ``Position``, ``Orientation``

    """
    def __init__(self, name = '', referenceCoordinates = [0.,0.,1.,0.], initialCoordinates = [0.,0.,0.,0.], initialVelocities = [0.,0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.initialVelocities = np.array(initialVelocities)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'Point2DSlope1'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
Point2DS1 = NodePoint2DSlope1
VPoint2DS1 = VNodePoint2DSlope1

class VNodePointSlope1:
    """Visualization data for NodePointSlope1.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used; type: float

        color: Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class NodePointSlope1:
    r"""A 3D point/slope vector node for spatial Bernoulli-Euler ANCF (absolute nodal coordinate formulation) beam elements, with 3 position and 3 slope coordinates, all ODE2; the slope vector is the derivative of the position with respect to the axial coordinate, :math:`[1,\;0,\;0]\tp` for a straight beam along the global :math:`x`-axis.
    
    Args:
        name: node's unique name; type: str

        referenceCoordinates: reference coordinates (x-pos,y-pos,z-pos; x-slopex, y-slopex, z-slopex) of node; global position of node without displacement; type: array_like

        initialCoordinates: initial displacement coordinates: ux, uy, uz and x/y/z 'displacements' of slopex; type: array_like

        initialVelocities: initial velocity coordinates; type: array_like

        visualization: visualization data, see VNodePointSlope1

    Notes:
        Node has/provides the following types: ``Position``

    """
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0.,1.,0.,0.], initialCoordinates = [0.,0.,0.,0.,0.,0.], initialVelocities = [0.,0.,0.,0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.initialVelocities = np.array(initialVelocities)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'PointSlope1'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

class VNodePointSlope12:
    """Visualization data for NodePointSlope12.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used; type: float

        color: Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class NodePointSlope12:
    r"""A 3D point/slope vector node for thin ANCF (absolute nodal coordinate formulation) plate elements, with 3 position and 2 :math:`\times` 3 slope coordinates, all ODE2; the slope vectors are the derivatives of the position with respect to the two in-plane coordinates of the plate.
    
    Args:
        name: node's unique name; type: str

        referenceCoordinates: reference coordinates (x-pos,y-pos,z-pos; x-slopeX, y-slopeX, z-slopeX; x-slopeY, y-slopeY, z-slopeY) of node; global position of node without displacement; type: array_like

        initialCoordinates: initial displacement coordinates relative to reference coordinates; type: array_like

        initialVelocities: initial velocity coordinates; type: array_like

        visualization: visualization data, see VNodePointSlope12

    Notes:
        Node has/provides the following types: ``Position``, ``Orientation``

    """
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0.,1.,0.,0.,0.,1.,0.], initialCoordinates = [0.,0.,0.,0.,0.,0.,0.,0.,0.], initialVelocities = [0.,0.,0.,0.,0.,0.,0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.initialVelocities = np.array(initialVelocities)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'PointSlope12'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

class VNodePointSlope23:
    """Visualization data for NodePointSlope23.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used; type: float

        color: Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class NodePointSlope23:
    r"""A 3D point/slope vector node for spatial, shear and cross-section deformable ANCF (absolute nodal coordinate formulation) beam elements, with 3 position and 2 :math:`\times` 3 slope coordinates, all ODE2; the slope vectors are the derivatives of the position with respect to the two cross section coordinates :math:`y` and :math:`z`.
    
    Args:
        name: node's unique name; type: str

        referenceCoordinates: reference coordinates (x-pos,y-pos,z-pos; x-slopey, y-slopey, z-slopey; x-slopez, y-slopez, z-slopez) of node; global position of node without displacement; type: array_like

        initialCoordinates: initial displacement coordinates relative to reference coordinates; type: array_like

        initialVelocities: initial velocity coordinates; type: array_like

        visualization: visualization data, see VNodePointSlope23

    Notes:
        Node has/provides the following types: ``Position``, ``Orientation``

    """
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0.,0.,1.,0.,0.,0.,1.], initialCoordinates = [0.,0.,0.,0.,0.,0.,0.,0.,0.], initialVelocities = [0.,0.,0.,0.,0.,0.,0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.initialVelocities = np.array(initialVelocities)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'PointSlope23'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

class VNodeGenericODE2:
    """Visualization data for NodeGenericODE2."""
    def __init__(self):
        pass

    def __iter__(self):
        yield from ()

    def __repr__(self):
        return str(dict(self))

class NodeGenericODE2:
    """A node containing a number of ODE2 variables.
    
    Use this node e.g. for scalar dynamic equations (Mass1D), for ObjectGenericODE2 or for the Eulerian coordinate in the ALECable element. NOTE: referenceCoordinates and all initialCoordinates(_t) must be initialized, because no default values exist.
    
    Args:
        name: node's unique name; type: str

        referenceCoordinates: generic reference coordinates of node; must be consistent with numberOfODE2Coordinates; type: array_like

        initialCoordinates: initial displacement coordinates; must be consistent with numberOfODE2Coordinates; type: array_like

        initialCoordinates_t: initial velocity coordinates; must be consistent with numberOfODE2Coordinates; type: array_like

        numberOfODE2Coordinates: number of generic ODE2 coordinates; type: int

        visualization: visualization data, see VNodeGenericODE2

    Notes:
        Node has/provides the following types: ``GenericODE2``

    """
    def __init__(self, name = '', referenceCoordinates = [], initialCoordinates = [], initialCoordinates_t = [], numberOfODE2Coordinates = 0, visualization = {}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.initialCoordinates_t = np.array(initialCoordinates_t)
        self.numberOfODE2Coordinates = numberOfODE2Coordinates
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'GenericODE2'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialCoordinates_t', self.initialCoordinates_t
        yield 'numberOfODE2Coordinates', self.numberOfODE2Coordinates

    def __repr__(self):
        return str(dict(self))

class VNodeGenericODE1:
    """Visualization data for NodeGenericODE1."""
    def __init__(self):
        pass

    def __iter__(self):
        yield from ()

    def __repr__(self):
        return str(dict(self))

class NodeGenericODE1:
    """A node containing a number of ODE1 variables.
    
    Use this node e.g. for linear state space systems. NOTE: referenceCoordinates and initialCoordinates must be initialized, because no default values exist.
    
    Args:
        name: node's unique name; type: str

        referenceCoordinates: generic reference coordinates of node; must be consistent with numberOfODE1Coordinates; type: array_like

        initialCoordinates: initial displacement coordinates; must be consistent with numberOfODE1Coordinates; type: array_like

        numberOfODE1Coordinates: number of generic ODE1 coordinates; type: int

        visualization: visualization data, see VNodeGenericODE1

    """
    def __init__(self, name = '', referenceCoordinates = [], initialCoordinates = [], numberOfODE1Coordinates = 0, visualization = {}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.numberOfODE1Coordinates = numberOfODE1Coordinates
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'GenericODE1'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'numberOfODE1Coordinates', self.numberOfODE1Coordinates

    def __repr__(self):
        return str(dict(self))

class VNodeGenericAE:
    """Visualization data for NodeGenericAE."""
    def __init__(self):
        pass

    def __iter__(self):
        yield from ()

    def __repr__(self):
        return str(dict(self))

class NodeGenericAE:
    """A node containing a number of AE variables.
    
    Use e.g. linear state space systems. NOTE: referenceCoordinates and initialCoordinates must be initialized, because no default values exist.
    
    Args:
        name: node's unique name; type: str

        referenceCoordinates: generic reference coordinates of node; must be consistent with numberOfAECoordinates; type: array_like

        initialCoordinates: initial displacement coordinates; must be consistent with numberOfAECoordinates; type: array_like

        numberOfAECoordinates: number of generic AE coordinates; type: int

        visualization: visualization data, see VNodeGenericAE

    """
    def __init__(self, name = '', referenceCoordinates = [], initialCoordinates = [], numberOfAECoordinates = 0, visualization = {}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.numberOfAECoordinates = numberOfAECoordinates
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'GenericAE'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'numberOfAECoordinates', self.numberOfAECoordinates

    def __repr__(self):
        return str(dict(self))

class VNodeGenericData:
    """Visualization data for NodeGenericData."""
    def __init__(self):
        pass

    def __iter__(self):
        yield from ()

    def __repr__(self):
        return str(dict(self))

class NodeGenericData:
    """A node containing a number of data (history) variables.
    
    Use this node e.g. for contact (active set), friction or plasticity (history variables).
    
    Args:
        name: node's unique name; type: str

        initialCoordinates: initial data coordinates; type: array_like

        numberOfDataCoordinates: number of generic data coordinates (history variables); type: int

        visualization: visualization data, see VNodeGenericData

    Notes:
        Node has/provides the following types: ``GenericData``

    """
    def __init__(self, name = '', initialCoordinates = [], numberOfDataCoordinates = 0, visualization = {}):
        self.name = name
        self.initialCoordinates = np.array(initialCoordinates)
        self.numberOfDataCoordinates = numberOfDataCoordinates
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'GenericData'
        yield 'name', self.name
        yield 'initialCoordinates', self.initialCoordinates
        yield 'numberOfDataCoordinates', self.numberOfDataCoordinates

    def __repr__(self):
        return str(dict(self))

class VNodePointGround:
    """Visualization data for NodePointGround.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used; type: float

        color: Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class NodePointGround:
    """A 3D point node fixed to ground which is similar to NodePoint, but it does not generate coordinates.
    
    Applied or reaction forces do not have any effect. This node can be used for 'blind' or 'dummy' ODE2 and ODE1 coordinates to which CoordinateSpringDamper or CoordinateConstraint objects are attached to.
    
    Args:
        name: node's unique name; type: str

        referenceCoordinates: reference coordinates of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement; type: [float,float,float]

        visualization: visualization data, see VNodePointGround

    Notes:
        Node has/provides the following types: ``Ground``, ``Position2D``, ``Position``, ``Orientation``, ``GenericODE2``

    """
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'PointGround'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
PointGround = NodePointGround
VPointGround = VNodePointGround

#+++++++++++++++++++++++++++++++
#OBJECT
class VObjectGround:
    """Visualization data for ObjectGround.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        graphicsDataUserFunction: A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; type: ObjectGroundGraphicsDataUserFunction

        graphicsData: Structure contains data for body visualization; data is defined in special list / dictionary structure; type: BodyGraphicsData

    """
    def __init__(self, show = True, graphicsDataUserFunction: Union[ObjectGroundGraphicsDataUserFunction, int] = 0, graphicsData = []):
        self.show = show
        self.graphicsDataUserFunction = graphicsDataUserFunction
        self.graphicsData = copy.copy(graphicsData)

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsDataUserFunction', self.graphicsDataUserFunction
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))

class ObjectGround:
    """A ground object behaving like a rigid body, but having no degrees of freedom.
    
    Used to attach body-connectors without an action. For examples see spring dampers and joints.
    
    Args:
        name: objects's unique name; type: str

        referencePosition: reference point = reference position for ground object; local position is added on top of reference position for a ground object; the translation of referenceHT; type: [float,float,float]

        referenceRotation: the constant ground rotation matrix, which transforms body-fixed (b) to global (0) coordinates; the rotation of referenceHT; type: array_like

        referenceHT: the reference frame of the ground as homogeneous transformation, composed of referenceRotation and referencePosition: a 4x4 matrix, its 16 values row by row or an exu.HT; given together with one of them, both must agree; type: array_like (4x4) or exudyn.HT

        visualization: visualization data, see VObjectGround

    Notes:
        Object has/provides the following types: ``Ground``, ``Body``

    """
    def __init__(self, name = '', referencePosition = None, referenceRotation = None, referenceHT = None, visualization = {'show': True, 'graphicsDataUserFunction': 0, 'graphicsData': []}):
        self.name = name
        self.referencePosition = None if referencePosition is None else np.array(referencePosition)
        self.referenceRotation = None if referenceRotation is None else np.array(referenceRotation)
        self.referenceHT = referenceHT
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'Ground'
        yield 'name', self.name
        yield 'referencePosition', self.referencePosition
        yield 'referenceRotation', self.referenceRotation
        yield 'referenceHT', self.referenceHT
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsDataUserFunction', dict(self.visualization)["graphicsDataUserFunction"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

    def __repr__(self):
        return str(dict(self))

class VObjectMassPoint:
    """Visualization data for ObjectMassPoint.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        graphicsData: Structure contains data for body visualization; data is defined in special list / dictionary structure; type: BodyGraphicsData

    """
    def __init__(self, show = True, graphicsData = []):
        self.show = show
        self.graphicsData = copy.copy(graphicsData)

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))

class ObjectMassPoint:
    """A 3D mass point which is attached to a position-based node, usually NodePoint.
    
    Args:
        name: objects's unique name; type: str

        mass: mass [SI:kg] of mass point; type: float

        nodeNumber: node number (type NodeIndex) for mass point

        physicsMass: deprecated since 1.12.258, removed in 2031: use mass

        visualization: visualization data, see VObjectMassPoint

    Notes:
        Object has/provides the following types: ``Body``, ``SingleNoded``

        Requested Node type: ``Position``

    """
    def __init__(self, name = '', mass = 0., nodeNumber = exudyn.InvalidIndex(), physicsMass = None, visualization = {'show': True, 'graphicsData': []}):
        self.name = name
        self.mass = mass
        self.nodeNumber = nodeNumber
        self.physicsMass = physicsMass
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'MassPoint'
        yield 'name', self.name
        yield 'mass', self.mass
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]
        if self.physicsMass is not None:
            yield 'physicsMass', self.physicsMass

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
MassPoint = ObjectMassPoint
VMassPoint = VObjectMassPoint

class VObjectMassPoint2D:
    """Visualization data for ObjectMassPoint2D.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        graphicsData: Structure contains data for body visualization; data is defined in special list / dictionary structure; type: BodyGraphicsData

    """
    def __init__(self, show = True, graphicsData = []):
        self.show = show
        self.graphicsData = copy.copy(graphicsData)

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))

class ObjectMassPoint2D:
    """A 2D mass point which is attached to a position-based 2D node.
    
    Args:
        name: objects's unique name; type: str

        mass: mass [SI:kg] of mass point; type: float

        nodeNumber: node number (type NodeIndex) for mass point

        physicsMass: deprecated since 1.12.258, removed in 2031: use mass

        visualization: visualization data, see VObjectMassPoint2D

    Notes:
        Object has/provides the following types: ``Body``, ``SingleNoded``

        Requested Node type: ``Position2D`` + ``Position``

    """
    def __init__(self, name = '', mass = 0., nodeNumber = exudyn.InvalidIndex(), physicsMass = None, visualization = {'show': True, 'graphicsData': []}):
        self.name = name
        self.mass = mass
        self.nodeNumber = nodeNumber
        self.physicsMass = physicsMass
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'MassPoint2D'
        yield 'name', self.name
        yield 'mass', self.mass
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]
        if self.physicsMass is not None:
            yield 'physicsMass', self.physicsMass

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
MassPoint2D = ObjectMassPoint2D
VMassPoint2D = VObjectMassPoint2D

class VObjectMass1D:
    """Visualization data for ObjectMass1D.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        graphicsData: Structure contains data for body visualization; data is defined in special list / dictionary structure; type: BodyGraphicsData

    """
    def __init__(self, show = True, graphicsData = []):
        self.show = show
        self.graphicsData = copy.copy(graphicsData)

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))

class ObjectMass1D:
    """A 1D (translational) mass which is attached to Node1D.
    
    Note, that the mass does not need to have the interpretation as a translational mass.
    
    Args:
        name: objects's unique name; type: str

        mass: mass [SI:kg] of mass; type: float

        nodeNumber: node number (type NodeIndex) for Node1D

        referencePosition: a reference position, used to transform the 1D coordinate to a position; type: [float,float,float]

        referenceRotation: the constant body rotation matrix, which transforms body-fixed (b) to global (0) coordinates; type: array_like

        physicsMass: deprecated since 1.12.258, removed in 2031: use mass

        visualization: visualization data, see VObjectMass1D

    Notes:
        Object has/provides the following types: ``Body``, ``SingleNoded``

        Requested Node type: ``GenericODE2``

    """
    def __init__(self, name = '', mass = 0., nodeNumber = exudyn.InvalidIndex(), referencePosition = [0.,0.,0.], referenceRotation = IIDiagMatrix(rowsColumns=3,value=1), physicsMass = None, visualization = {'show': True, 'graphicsData': []}):
        self.name = name
        self.mass = mass
        self.nodeNumber = nodeNumber
        self.referencePosition = np.array(referencePosition)
        self.referenceRotation = np.array(referenceRotation)
        self.physicsMass = physicsMass
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'Mass1D'
        yield 'name', self.name
        yield 'mass', self.mass
        yield 'nodeNumber', self.nodeNumber
        yield 'referencePosition', self.referencePosition
        yield 'referenceRotation', self.referenceRotation
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]
        if self.physicsMass is not None:
            yield 'physicsMass', self.physicsMass

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
Mass1D = ObjectMass1D
VMass1D = VObjectMass1D

class VObjectRotationalMass1D:
    """Visualization data for ObjectRotationalMass1D.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        graphicsData: Structure contains data for body visualization; data is defined in special list / dictionary structure; type: BodyGraphicsData

    """
    def __init__(self, show = True, graphicsData = []):
        self.show = show
        self.graphicsData = copy.copy(graphicsData)

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))

class ObjectRotationalMass1D:
    r"""A 1D rotational inertia (mass) which is attached to Node1D.
    
    Args:
        name: objects's unique name; type: str

        inertia: inertia components [SI:kgm:math:`^2`] of rotor / rotational mass; type: float

        nodeNumber: node number (type NodeIndex) of Node1D, providing rotation coordinate :math:`\psi_0 = c_0`

        referencePosition: a constant reference position = reference point, used to assign joint constraints accordingly and for drawing; type: [float,float,float]

        referenceRotation: an intermediate rotation matrix, which transforms the 1D coordinate into 3D, see description; type: array_like

        physicsInertia: deprecated since 1.12.258, removed in 2031: use inertia

        visualization: visualization data, see VObjectRotationalMass1D

    Notes:
        Object has/provides the following types: ``Body``, ``SingleNoded``

        Requested Node type: ``GenericODE2``

    """
    def __init__(self, name = '', inertia = 0., nodeNumber = exudyn.InvalidIndex(), referencePosition = [0.,0.,0.], referenceRotation = IIDiagMatrix(rowsColumns=3,value=1), physicsInertia = None, visualization = {'show': True, 'graphicsData': []}):
        self.name = name
        self.inertia = inertia
        self.nodeNumber = nodeNumber
        self.referencePosition = np.array(referencePosition)
        self.referenceRotation = np.array(referenceRotation)
        self.physicsInertia = physicsInertia
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'RotationalMass1D'
        yield 'name', self.name
        yield 'inertia', self.inertia
        yield 'nodeNumber', self.nodeNumber
        yield 'referencePosition', self.referencePosition
        yield 'referenceRotation', self.referenceRotation
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]
        if self.physicsInertia is not None:
            yield 'physicsInertia', self.physicsInertia

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
Rotor1D = ObjectRotationalMass1D
VRotor1D = VObjectRotationalMass1D

class VObjectRigidBody:
    """Visualization data for ObjectRigidBody.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        graphicsDataUserFunction: A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics elements need to be defined in the local body coordinates and are transformed by mbs to global coordinates; type: ObjectRigidBodyGraphicsDataUserFunction

        graphicsData: Structure contains data for body visualization; data is defined in special list / dictionary structure; type: BodyGraphicsData

    """
    def __init__(self, show = True, graphicsDataUserFunction: Union[ObjectRigidBodyGraphicsDataUserFunction, int] = 0, graphicsData = []):
        self.show = show
        self.graphicsDataUserFunction = graphicsDataUserFunction
        self.graphicsData = copy.copy(graphicsData)

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsDataUserFunction', self.graphicsDataUserFunction
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))

class ObjectRigidBody:
    """A 3D rigid body which is attached to a 3D rigid body node.
    
    The rotation parametrization of the rigid body follows the rotation parametrization of the node. Use Euler parameters in the general case (no singularities) in combination with implicit solvers (GeneralizedAlpha or TrapezoidalIndex2), Tait-Bryan angles for special cases, e.g., rotors where no singularities occur if you rotate about :math:`x` or :math:`z` axis, or use Lie-group formulation with rotation vector together with explicit solvers. REMARK: Use the class ``RigidBodyInertia``, see sec-rigidbodyutilities-rigidbodyinertia---init-- of ``exudyn.rigidBodyUtilities`` to handle inertia, COM and mass.
    
    Args:
        name: objects's unique name; type: str

        mass: mass [SI:kg] of rigid body; type: float

        inertia: inertia components [SI:kgm:math:`^2`]: :math:`[J_{xx}, J_{yy}, J_{zz}, J_{yz}, J_{xz}, J_{xy}]` in body-fixed coordinate system and w.r.t. to the reference point of the body, NOT necessarily w.r.t. to COM; use the class RigidBodyInertia of exudynRigidBodyUtilities.py to handle inertia, COM and mass; type: array_like

        centerOfMass: local position of COM relative to the body's reference point; if the vector of the COM is [0,0,0], the computation will not consider additional terms for the COM and it is faster; type: [float,float,float]

        nodeNumber: node number (type NodeIndex) for rigid body node

        physicsMass: deprecated since 1.12.258, removed in 2031: use mass

        physicsInertia: deprecated since 1.12.258, removed in 2031: use inertia

        physicsCenterOfMass: deprecated since 1.12.258, removed in 2031: use centerOfMass

        visualization: visualization data, see VObjectRigidBody

    Notes:
        Object has/provides the following types: ``Body``, ``SingleNoded``

        Requested Node type: ``Position`` + ``Orientation`` + ``RigidBody``

    """
    def __init__(self, name = '', mass = 0., inertia = [0.,0.,0., 0.,0.,0.], centerOfMass = [0.,0.,0.], nodeNumber = exudyn.InvalidIndex(), physicsMass = None, physicsInertia = None, physicsCenterOfMass = None, visualization = {'show': True, 'graphicsDataUserFunction': 0, 'graphicsData': []}):
        self.name = name
        self.mass = mass
        self.inertia = np.array(inertia)
        self.centerOfMass = np.array(centerOfMass)
        self.nodeNumber = nodeNumber
        self.physicsMass = physicsMass
        self.physicsInertia = physicsInertia
        self.physicsCenterOfMass = physicsCenterOfMass
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'RigidBody'
        yield 'name', self.name
        yield 'mass', self.mass
        yield 'inertia', self.inertia
        yield 'centerOfMass', self.centerOfMass
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsDataUserFunction', dict(self.visualization)["graphicsDataUserFunction"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]
        if self.physicsMass is not None:
            yield 'physicsMass', self.physicsMass
        if self.physicsInertia is not None:
            yield 'physicsInertia', self.physicsInertia
        if self.physicsCenterOfMass is not None:
            yield 'physicsCenterOfMass', self.physicsCenterOfMass

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
RigidBody = ObjectRigidBody
VRigidBody = VObjectRigidBody

class VObjectRigidBody2D:
    """Visualization data for ObjectRigidBody2D.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        graphicsDataUserFunction: A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics elements need to be defined in the local body coordinates and are transformed by mbs to global coordinates; type: ObjectRigidBody2DGraphicsDataUserFunction

        graphicsData: Structure contains data for body visualization; data is defined in special list / dictionary structure; type: BodyGraphicsData

    """
    def __init__(self, show = True, graphicsDataUserFunction: Union[ObjectRigidBody2DGraphicsDataUserFunction, int] = 0, graphicsData = []):
        self.show = show
        self.graphicsDataUserFunction = graphicsDataUserFunction
        self.graphicsData = copy.copy(graphicsData)

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsDataUserFunction', self.graphicsDataUserFunction
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))

class ObjectRigidBody2D:
    """A 2D rigid body which is attached to a rigid body 2D node.
    
    The body obtains coordinates, position, velocity, etc. from the underlying 2D node.
    
    Args:
        name: objects's unique name; type: str

        mass: mass [SI:kg] of rigid body; type: float

        inertia: inertia [SI:kgm:math:`^2`] of rigid body w.r.t. reference point; this is equal to the center of mass, if centerOfMass = 0; type: float

        centerOfMass: local position of COM relative to the body's reference point; if the vector of the COM is [0,0], the computation will not consider additional terms for the COM and it is faster; type: [float,float]

        nodeNumber: node number (type NodeIndex) for 2D rigid body node

        physicsMass: deprecated since 1.12.258, removed in 2031: use mass

        physicsInertia: deprecated since 1.12.258, removed in 2031: use inertia

        physicsCenterOfMass: deprecated since 1.12.258, removed in 2031: use centerOfMass

        visualization: visualization data, see VObjectRigidBody2D

    Notes:
        Object has/provides the following types: ``Body``, ``SingleNoded``

        Requested Node type: ``Position2D`` + ``Orientation2D`` + ``Position`` + ``Orientation``

    """
    def __init__(self, name = '', mass = 0., inertia = 0., centerOfMass = [0.,0.], nodeNumber = exudyn.InvalidIndex(), physicsMass = None, physicsInertia = None, physicsCenterOfMass = None, visualization = {'show': True, 'graphicsDataUserFunction': 0, 'graphicsData': []}):
        self.name = name
        self.mass = mass
        self.inertia = inertia
        self.centerOfMass = np.array(centerOfMass)
        self.nodeNumber = nodeNumber
        self.physicsMass = physicsMass
        self.physicsInertia = physicsInertia
        self.physicsCenterOfMass = physicsCenterOfMass
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'RigidBody2D'
        yield 'name', self.name
        yield 'mass', self.mass
        yield 'inertia', self.inertia
        yield 'centerOfMass', self.centerOfMass
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsDataUserFunction', dict(self.visualization)["graphicsDataUserFunction"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]
        if self.physicsMass is not None:
            yield 'physicsMass', self.physicsMass
        if self.physicsInertia is not None:
            yield 'physicsInertia', self.physicsInertia
        if self.physicsCenterOfMass is not None:
            yield 'physicsCenterOfMass', self.physicsCenterOfMass

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
RigidBody2D = ObjectRigidBody2D
VRigidBody2D = VObjectRigidBody2D

class VObjectGenericODE2:
    """Visualization data for ObjectGenericODE2.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        color: RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used; type: [float,float,float,float]

        triangleMesh: a matrix of node numbers referring to the mesh nodes of the object, one triangle per row: 3 columns for flat triangles, or 6 for 6-node triangles drawn curved - the corners counterclockwise seen from outside, then the mid nodes of the edges 0-1, 1-2 and 2-0, as in GraphicsData triangles6; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!; type: array_like

        showNodes: set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'; type: bool

        graphicsDataUserFunction: A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics data is draw in global coordinates; it can be used to implement user element visualization, e.g., beam elements or simple mechanical systems; note that this user function may significantly slow down visualization; type: ObjectGenericODE2GraphicsDataUserFunction

    """
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.], triangleMesh = [], showNodes = False, graphicsDataUserFunction: Union[ObjectGenericODE2GraphicsDataUserFunction, int] = 0):
        self.show = show
        self.color = np.array(color)
        self.triangleMesh = np.array(triangleMesh)
        self.showNodes = showNodes
        self.graphicsDataUserFunction = graphicsDataUserFunction

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color
        yield 'triangleMesh', self.triangleMesh
        yield 'showNodes', self.showNodes
        yield 'graphicsDataUserFunction', self.graphicsDataUserFunction

    def __repr__(self):
        return str(dict(self))

class ObjectGenericODE2:
    r"""A system of :math:`n` second order ordinary differential equations (ODE2), having a mass matrix, damping/gyroscopic matrix, stiffness matrix and generalized forces.
    
    It can combine generic nodes, or node points. User functions can be used to compute mass matrix and generalized forces depending on given coordinates. NOTE: all matrices, vectors, etc. must have the same dimensions :math:`n` or :math:`(n \times n)`, or they must be empty :math:`(0 \times 0)`, except for the mass matrix which always needs to have dimensions :math:`(n \times n)`.
    
    Args:
        name: objects's unique name; type: str

        nodeNumbers: node numbers which provide the coordinates for the object (consecutively as provided in this list); type: ArrayNodeIndex

        massMatrix: mass matrix of object as MatrixContainer (or numpy array / list of lists); type: PyMatrixContainer

        stiffnessMatrix: stiffness matrix of object as MatrixContainer (or numpy array / list of lists); NOTE that (dense/sparse triplets) format must agree with dampingMatrix and jacobianUserFunction; type: PyMatrixContainer

        dampingMatrix: damping matrix of object as MatrixContainer (or numpy array / list of lists); NOTE that (dense/sparse triplets) format must agree with stiffnessMatrix and jacobianUserFunction; type: PyMatrixContainer

        forceVector: generalized force vector added to RHS; type: array_like

        forceUserFunction: A Python user function which computes the generalized user force vector for the ODE2 equations; see description below; type: ObjectGenericODE2ForceUserFunction

        massMatrixUserFunction: A Python user function which computes the mass matrix instead of the constant mass matrix given in :math:`\Mm`; return numpy array or MatrixContainer; see description below; type: ObjectGenericODE2MassMatrixUserFunction

        jacobianUserFunction: A Python user function which computes the jacobian, i.e., the derivative of the left-hand-side object equation w.r.t. the coordinates (times :math:`f_{ODE2}`) and w.r.t. the velocities (times :math:`f_{ODE2_t}`). Terms on the RHS must be subtracted from the LHS equation; the respective terms for the stiffness matrix and damping matrix are automatically added; see description below; type: ObjectGenericODE2JacobianUserFunction

        visualization: visualization data, see VObjectGenericODE2

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``, ``SuperElement``

    """
    def __init__(self, name = '', nodeNumbers = [], massMatrix = None, stiffnessMatrix = None, dampingMatrix = None, forceVector = [], forceUserFunction: Union[ObjectGenericODE2ForceUserFunction, int] = 0, massMatrixUserFunction: Union[ObjectGenericODE2MassMatrixUserFunction, int] = 0, jacobianUserFunction: Union[ObjectGenericODE2JacobianUserFunction, int] = 0, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.], 'triangleMesh': [], 'showNodes': False, 'graphicsDataUserFunction': 0}):
        self.name = name
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.massMatrix = massMatrix
        self.stiffnessMatrix = stiffnessMatrix
        self.dampingMatrix = dampingMatrix
        self.forceVector = CheckForValidNumpyArray(forceVector)
        self.forceUserFunction = forceUserFunction
        self.massMatrixUserFunction = massMatrixUserFunction
        self.jacobianUserFunction = jacobianUserFunction
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'GenericODE2'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'massMatrix', self.massMatrix
        yield 'stiffnessMatrix', self.stiffnessMatrix
        yield 'dampingMatrix', self.dampingMatrix
        yield 'forceVector', self.forceVector
        yield 'forceUserFunction', self.forceUserFunction
        yield 'massMatrixUserFunction', self.massMatrixUserFunction
        yield 'jacobianUserFunction', self.jacobianUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]
        yield 'VtriangleMesh', dict(self.visualization)["triangleMesh"]
        yield 'VshowNodes', dict(self.visualization)["showNodes"]
        yield 'VgraphicsDataUserFunction', dict(self.visualization)["graphicsDataUserFunction"]

    def __repr__(self):
        return str(dict(self))

class VObjectGenericODE1:
    """Visualization data for ObjectGenericODE1."""
    def __init__(self):
        pass

    def __iter__(self):
        yield from ()

    def __repr__(self):
        return str(dict(self))

class ObjectGenericODE1:
    r"""A system of :math:`n` ODE1, having a system matrix, a rhs vector, but mostly it will use a user function to describe special ODE1 systems.
    
    It is based on NodeGenericODE1 nodes. NOTE that all matrices, vectors, etc. must have the same dimensions :math:`n` or :math:`(n \times n)`, or they must be empty :math:`(0 \times 0)`, using [] in Python.
    
    Args:
        name: objects's unique name; type: str

        nodeNumbers: node numbers which provide the coordinates for the object (consecutively as provided in this list); type: ArrayNodeIndex

        systemMatrix: system matrix (state space matrix) of first order ODE; type: array_like

        rhsVector: a constant rhs vector (e.g., for constant input); type: array_like

        rhsUserFunction: A Python user function which computes the right-hand-side (rhs) of the first order ODE; see description below; type: ObjectGenericODE1RhsUserFunction

        visualization: visualization data, see VObjectGenericODE1

    Notes:
        Object has/provides the following types: ``MultiNoded``

    """
    def __init__(self, name = '', nodeNumbers = [], systemMatrix = [], rhsVector = [], rhsUserFunction: Union[ObjectGenericODE1RhsUserFunction, int] = 0, visualization = {}):
        self.name = name
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.systemMatrix = CheckForValidNumpyArray(systemMatrix)
        self.rhsVector = CheckForValidNumpyArray(rhsVector)
        self.rhsUserFunction = rhsUserFunction
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'GenericODE1'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'systemMatrix', self.systemMatrix
        yield 'rhsVector', self.rhsVector
        yield 'rhsUserFunction', self.rhsUserFunction

    def __repr__(self):
        return str(dict(self))

class VObjectKinematicTree:
    """Visualization data for ObjectKinematicTree.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        showLinks: set true, if links shall be shown; if graphicsDataList is empty, a standard drawing for links is used (drawing a cylinder from previous joint or base to next joint; size relative to frame size in KinematicTree visualization settings); else graphicsDataList are used per link; NOTE visualization of joint and COM frames can be modified via visualizationSettings.bodies.kinematicTree; type: bool

        showJoints: set true, if joints shall be shown; if graphicsDataList is empty, a standard drawing for joints is used (drawing a cylinder for revolute joints; size relative to frame size in KinematicTree visualization settings); type: bool

        color: RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used; type: [float,float,float,float]

        graphicsDataList: Structure contains data for link/joint visualization; data is defined as list of BodyGraphicsData where every BodyGraphicsData corresponds to one link/joint; must either be emtpy list or length must agree with number of links; type: BodyGraphicsDataList

    """
    def __init__(self, show = True, showLinks = True, showJoints = True, color = [-1.,-1.,-1.,-1.], graphicsDataList = []):
        self.show = show
        self.showLinks = showLinks
        self.showJoints = showJoints
        self.color = np.array(color)
        self.graphicsDataList = copy.copy(graphicsDataList)

    def __iter__(self):
        yield 'show', self.show
        yield 'showLinks', self.showLinks
        yield 'showJoints', self.showJoints
        yield 'color', self.color
        yield 'graphicsDataList', self.graphicsDataList

    def __repr__(self):
        return str(dict(self))

class ObjectKinematicTree:
    r"""A special object to represent open kinematic trees using minimal coordinate formulation.
    
    The kinematic tree is defined by lists of joint types, parents, inertia parameters (w.r.t. COM), etc. per link (body) and given joint (pre) transformations from the previous joint. Every joint / link is defined by the position and orientation of the previous joint and a coordinate transformation (incl. translation) from the previous link's to this link's joint coordinates. The joint can be combined with a marker, which allows to attach connectors as well as joints to represent closed loop mechanisms. Efficient models can be created by using tree structures in combination with constraints and very long chains should be avoided and replaced by (smaller) jointed chains if possible. The class Robot from exudyn.robotics can also be used to create kinematic trees, which are then exported as KinematicTree or as redundant multibody system. Use specialized settings in VisualizationSettings.bodies.kinematicTree for showing joint frames and other properties.
    
    Args:
        name: objects's unique name; type: str

        nodeNumber: node number (type NodeIndex) of GenericODE2 node containing the coordinates for the kinematic tree; :math:`n` being the number of minimal coordinates

        gravity: gravity vector in inertial coordinates; used to simply apply gravity as LoadMassProportional is not available for KinematicTree; type: [float,float,float]

        baseOffset: offset vector for base, in global coordinates; type: [float,float,float]

        jointTypes: joint types of kinematic Tree joints, using exu.JointType, like exu.JointType.RevoluteZ; must be always set; type: JointTypeList

        linkParents: index of parent joint/link; if no parent exists, the value is :math:`-1`; by default, :math:`p_0=-1` because the :math:`i`th parent index must always fulfill :math:`p_i<i`; must be always set; type: array_like

        jointTransformations: list of constant joint transformations from parent joint coordinates :math:`p_0` to this joint coordinates :math:`j_0`; this allows to adjust the orientation of the joint axes (but it does not affect the joint offset); if no parent exists (:math:`-1`), the base coordinate system :math:`0` is used; must be always set; type: Matrix3DList

        jointOffsets: list of constant joint offsets from parent joint to this joint; :math:`p_0`, :math:`p_1`, :math:`\ldots` denote the parent coordinate systems; this means that the joint offset is added prior to performing the joint transformation; if no parent exists (:math:`-1`), the base coordinate system :math:`0` is used; must be always set; type: Vector3DList

        jointHTs: the joint transformations and the joint offsets at once, as a list of homogeneous transformations from the parent joint to this joint - :math:`\Hm_i` with the rotation :math:`\Tm_i` and the translation :math:`{}^{p_i}{o_i}`, each a 4x4 matrix, its 16 values or an exu.HT; None: not given; given together with jointTransformations or jointOffsets, they must agree; type: list of array_like (4x4) or exudyn.HT

        linkInertiasCOM: list of link inertia tensors w.r.t. COM in joint/link :math:`j_i` coordinates; must be always set; type: Matrix3DList

        linkCOMs: list of vectors for center of mass (COM) in joint/link :math:`j_i` coordinates; must be always set; type: Vector3DList

        linkMasses: masses of links; must be always set; type: array_like

        linkForces: list of 3D force vectors per link in global coordinates acting on joint frame origin; use force-torque couple to realize off-origin forces; defaults to empty list :math:`[]`, adding no forces; type: Vector3DList

        linkTorques: list of 3D torque vectors per link in global coordinates; defaults to empty list :math:`[]`, adding no torques; type: Vector3DList

        jointForceVector: generalized force vector per coordinate added to RHS of EOM; represents a torque around the axis of rotation in revolute joints and a force in prismatic joints; for a revolute joint :math:`i`, the torque :math:`f[i]` acts positive (w.r.t. rotation axis) on link :math:`i` and negative on parent link :math:`p_i`; must be either empty list/array :math:`[]` (default) or have size :math:`n`; type: array_like

        jointPositionOffsetVector: offset for joint coordinates used in P(D) control; acts in positive joint direction similar to jointForceVector; should be modified, e.g., in preStepUserFunction; must be either empty list/array :math:`[]` (default) or have size :math:`n`; type: array_like

        jointVelocityOffsetVector: velocity offset for joint coordinates used in (P)D control; acts in positive joint direction similar to jointForceVector; should be modified, e.g., in preStepUserFunction; must be either empty list/array :math:`[]` (default) or have size :math:`n`; type: array_like

        jointPControlVector: proportional (P) control values per joint (multiplied with position error between joint value and offset :math:`\mathbf{u}_o`); note that more complicated control laws must be implemented with user functions; must be either empty list/array :math:`[]` (default) or have size :math:`n`; type: array_like

        jointDControlVector: derivative (D) control values per joint (multiplied with velocity error between joint velocity and velocity offset :math:`\vv_o`); note that more complicated control laws must be implemented with user functions; must be either empty list/array :math:`[]` (default) or have size :math:`n`; type: array_like

        forceUserFunction: A Python user function which computes the generalized force vector on RHS with identical action as jointForceVector; see description below; type: ObjectKinematicTreeForceUserFunction

        visualization: visualization data, see VObjectKinematicTree

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``, ``SuperElement``

        Requested Node type: ``GenericODE2``

    """
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), gravity = [0.,0.,0.], baseOffset = [0.,0.,0.], jointTypes = [], linkParents = [], jointTransformations = None, jointOffsets = None, jointHTs = None, linkInertiasCOM = None, linkCOMs = None, linkMasses = [], linkForces = None, linkTorques = None, jointForceVector = [], jointPositionOffsetVector = [], jointVelocityOffsetVector = [], jointPControlVector = [], jointDControlVector = [], forceUserFunction: Union[ObjectKinematicTreeForceUserFunction, int] = 0, visualization = {'show': True, 'showLinks': True, 'showJoints': True, 'color': [-1.,-1.,-1.,-1.], 'graphicsDataList': []}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.gravity = np.array(gravity)
        self.baseOffset = np.array(baseOffset)
        self.jointTypes = copy.copy(jointTypes)
        self.linkParents = copy.copy(linkParents)
        self.jointTransformations = jointTransformations
        self.jointOffsets = jointOffsets
        self.jointHTs = jointHTs
        self.linkInertiasCOM = linkInertiasCOM
        self.linkCOMs = linkCOMs
        self.linkMasses = np.array(linkMasses)
        self.linkForces = linkForces
        self.linkTorques = linkTorques
        self.jointForceVector = np.array(jointForceVector)
        self.jointPositionOffsetVector = np.array(jointPositionOffsetVector)
        self.jointVelocityOffsetVector = np.array(jointVelocityOffsetVector)
        self.jointPControlVector = np.array(jointPControlVector)
        self.jointDControlVector = np.array(jointDControlVector)
        self.forceUserFunction = forceUserFunction
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'KinematicTree'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'gravity', self.gravity
        yield 'baseOffset', self.baseOffset
        yield 'jointTypes', self.jointTypes
        yield 'linkParents', self.linkParents
        yield 'jointTransformations', self.jointTransformations
        yield 'jointOffsets', self.jointOffsets
        yield 'jointHTs', self.jointHTs
        yield 'linkInertiasCOM', self.linkInertiasCOM
        yield 'linkCOMs', self.linkCOMs
        yield 'linkMasses', self.linkMasses
        yield 'linkForces', self.linkForces
        yield 'linkTorques', self.linkTorques
        yield 'jointForceVector', self.jointForceVector
        yield 'jointPositionOffsetVector', self.jointPositionOffsetVector
        yield 'jointVelocityOffsetVector', self.jointVelocityOffsetVector
        yield 'jointPControlVector', self.jointPControlVector
        yield 'jointDControlVector', self.jointDControlVector
        yield 'forceUserFunction', self.forceUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VshowLinks', dict(self.visualization)["showLinks"]
        yield 'VshowJoints', dict(self.visualization)["showJoints"]
        yield 'Vcolor', dict(self.visualization)["color"]
        yield 'VgraphicsDataList', dict(self.visualization)["graphicsDataList"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
KinematicTree = ObjectKinematicTree
VKinematicTree = VObjectKinematicTree

class VObjectFFRF:
    """Visualization data for ObjectFFRF.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; use visualizationSettings.bodies.deformationScaleFactor to draw scaled (local) deformations; the reference frame node is shown with additional letters RF; type: bool

        color: RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used; type: [float,float,float,float]

        triangleMesh: a matrix of node numbers referring to the mesh nodes of the object, one triangle per row: 3 columns for flat triangles, or 6 for 6-node triangles drawn curved - the corners counterclockwise seen from outside, then the mid nodes of the edges 0-1, 1-2 and 2-0, as in GraphicsData triangles6; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!; type: array_like

        showNodes: set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'; type: bool

    """
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.], triangleMesh = [], showNodes = False):
        self.show = show
        self.color = np.array(color)
        self.triangleMesh = np.array(triangleMesh)
        self.showNodes = showNodes

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color
        yield 'triangleMesh', self.triangleMesh
        yield 'showNodes', self.showNodes

    def __repr__(self):
        return str(dict(self))

class ObjectFFRF:
    r"""This object is used to represent equations modelled by the FFRF.
    
    It contains a RigidBodyNode (always node 0) and a list of other nodes representing the finite element nodes used in the FFRF. Note that temporary matrices and vectors are subject of change in future. NOTE: Usually you SHOULD NOT USE THIS OBJECT - use the much more efficient ObjectFFRFreducedOrder object with modal reduction instead.
    
    Args:
        name: objects's unique name; type: str

        nodeNumbers: node numbers which provide the coordinates for the object (consecutively as provided in this list); the :math:`(n_\mathrm{nf}+1)` nodes represent the nodes of the FE mesh (except for node 0); the global nodal position needs to be reconstructed from the rigid-body motion of the reference frame; type: ArrayNodeIndex

        massMatrixFF: body-fixed and ONLY flexible coordinates part of mass matrix of object given in Python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format; type: PyMatrixContainer

        stiffnessMatrixFF: body-fixed and ONLY flexible coordinates part of stiffness matrix of object in Python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format; type: PyMatrixContainer

        dampingMatrixFF: body-fixed and ONLY flexible coordinates part of damping matrix of object in Python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format; type: PyMatrixContainer

        forceVector: generalized, force vector added to RHS; the rigid body part :math:`\fv_r` is directly applied to rigid body coordinates while the flexible part :math:`\fv\indf` is transformed from global to local coordinates; note that this force vector only allows to add gravity forces for bodies with COM at the origin of the reference frame; type: array_like

        forceUserFunction: A Python user function which computes the generalized user force vector for the ODE2 equations; note the different coordinate systems for rigid body and flexible part; The function args are mbs, time, objectNumber, coordinates q (without reference values) and coordinate velocities q_t; see description below; type: ObjectFFRFForceUserFunction

        massMatrixUserFunction: A Python user function which computes the TOTAL mass matrix (including reference node) and adds the local constant mass matrix; note the different coordinate systems as described in the FFRF mass matrix; see description below; type: ObjectFFRFMassMatrixUserFunction

        computeFFRFterms: flag decides whether the standard FFRF terms are computed; use this flag for user-defined definition of FFRF terms in mass matrix and quadratic velocity vector; type: bool

        objectIsInitialized: ALWAYS set to False! flag used to correctly initialize all FFRF matrices; as soon as this flag is False, internal (constant) FFRF matrices are recomputed during Assemble(); type: bool

        visualization: visualization data, see VObjectFFRF

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``, ``SuperElement``

    """
    def __init__(self, name = '', nodeNumbers = [], massMatrixFF = None, stiffnessMatrixFF = None, dampingMatrixFF = None, forceVector = [], forceUserFunction: Union[ObjectFFRFForceUserFunction, int] = 0, massMatrixUserFunction: Union[ObjectFFRFMassMatrixUserFunction, int] = 0, computeFFRFterms = True, objectIsInitialized = False, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.], 'triangleMesh': [], 'showNodes': False}):
        self.name = name
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.massMatrixFF = massMatrixFF
        self.stiffnessMatrixFF = stiffnessMatrixFF
        self.dampingMatrixFF = dampingMatrixFF
        self.forceVector = CheckForValidNumpyArray(forceVector)
        self.forceUserFunction = forceUserFunction
        self.massMatrixUserFunction = massMatrixUserFunction
        self.computeFFRFterms = computeFFRFterms
        self.objectIsInitialized = objectIsInitialized
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'FFRF'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'massMatrixFF', self.massMatrixFF
        yield 'stiffnessMatrixFF', self.stiffnessMatrixFF
        yield 'dampingMatrixFF', self.dampingMatrixFF
        yield 'forceVector', self.forceVector
        yield 'forceUserFunction', self.forceUserFunction
        yield 'massMatrixUserFunction', self.massMatrixUserFunction
        yield 'computeFFRFterms', self.computeFFRFterms
        yield 'objectIsInitialized', self.objectIsInitialized
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]
        yield 'VtriangleMesh', dict(self.visualization)["triangleMesh"]
        yield 'VshowNodes', dict(self.visualization)["showNodes"]

    def __repr__(self):
        return str(dict(self))

class VObjectFFRFreducedOrder:
    """Visualization data for ObjectFFRFreducedOrder.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; use visualizationSettings.bodies.deformationScaleFactor to draw scaled (local) deformations; the reference frame node is shown with additional letters RF; type: bool

        color: RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used; type: [float,float,float,float]

        triangleMesh: a matrix of node numbers referring to the mesh nodes of the object, one triangle per row: 3 columns for flat triangles, or 6 for 6-node triangles drawn curved - the corners counterclockwise seen from outside, then the mid nodes of the edges 0-1, 1-2 and 2-0, as in GraphicsData triangles6; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!; type: array_like

        showNodes: set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'; type: bool

    """
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.], triangleMesh = [], showNodes = False):
        self.show = show
        self.color = np.array(color)
        self.triangleMesh = np.array(triangleMesh)
        self.showNodes = showNodes

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color
        yield 'triangleMesh', self.triangleMesh
        yield 'showNodes', self.showNodes

    def __repr__(self):
        return str(dict(self))

class ObjectFFRFreducedOrder:
    r"""This object is used to represent modally reduced flexible bodies using the FFRF and the CMS.
    
    It can be used to model real-life mechanical systems imported from finite element codes or Python tools such as NETGEN/NGsolve, see the ``FEMinterface`` in sec-fem-feminterface---init--. It contains a RigidBodyNode (always node 0) and a NodeGenericODE2 representing the modal coordinates. Currently, equations must be defined within user functions, which are available in the FEM module, see class ``ObjectFFRFreducedOrderInterface``, especially the user functions ``UFmassFFRFreducedOrder`` and ``UFforceFFRFreducedOrder``, sec-fem-objectffrfreducedorderinterface-addobjectffrfreducedorderwithuserfunctions.
    
    Args:
        name: objects's unique name; type: str

        nodeNumbers: node numbers of rigid body node and NodeGenericODE2 for modal coordinates; the global nodal position needs to be reconstructed from the rigid-body motion of the reference frame, the modal coordinates and the mode basis; type: ArrayNodeIndex

        massMatrixReduced: body-fixed and ONLY flexible coordinates part of reduced mass matrix; provided as MatrixContainer(sparse/dense matrix); type: PyMatrixContainer

        stiffnessMatrixReduced: body-fixed and ONLY flexible coordinates part of reduced stiffness matrix; provided as MatrixContainer(sparse/dense matrix); type: PyMatrixContainer

        dampingMatrixReduced: body-fixed and ONLY flexible coordinates part of reduced damping matrix; provided as MatrixContainer(sparse/dense matrix); type: PyMatrixContainer

        forceUserFunction: A Python user function which computes the generalized user force vector for the ODE2 equations; see description below; type: ObjectFFRFreducedOrderForceUserFunction

        massMatrixUserFunction: A Python user function which computes the TOTAL mass matrix (including reference node) and adds the local constant mass matrix; see description below; type: ObjectFFRFreducedOrderMassMatrixUserFunction

        computeFFRFterms: flag decides whether the standard FFRF/CMS terms are computed; use this flag for user-defined definition of FFRF terms in mass matrix and quadratic velocity vector; type: bool

        modeBasis: mode basis, which transforms reduced coordinates to (full) nodal coordinates, written as a single vector :math:`[u_{x,n_0},\,u_{y,n_0},\,u_{z,n_0},\,\ldots,\,u_{x,n_n},\,u_{y,n_n},\,u_{z,n_n}]\tp`; type: array_like

        outputVariableModeBasis: mode basis, which transforms reduced coordinates to output variables per mode and per node; :math:`s_{OV}` is the size of the output variable, e.g., 6 for stress modes (:math:`S_{xx},...,S_{xy}`); type: array_like

        outputVariableTypeModeBasis: this must be the output variable type of the outputVariableModeBasis, e.g. exu.OutputVariableType.Stress

        referencePositions: vector containing the reference positions of all flexible nodes, needed for graphics; type: array_like

        objectIsInitialized: ALWAYS set to False! flag used to correctly initialize all FFRF matrices; as soon as this flag is False, some internal (constant) FFRF matrices are recomputed during Assemble(); type: bool

        mass: total mass [SI:kg] of FFRFreducedOrder object; type: float

        inertia: inertia tensor [SI:kgm:math:`^2`] of rigid body w.r.t. to the reference point of the body; type: array_like

        centerOfMass: local position of center of mass (COM); type: [float,float,float]

        mPsiTildePsi: special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface; type: array_like

        mPsiTildePsiTilde: special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface; type: array_like

        mPhitTPsi: special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface; type: array_like

        mPhitTPsiTilde: special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface; type: array_like

        mXRefTildePsi: special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface; type: array_like

        mXRefTildePsiTilde: special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface; type: array_like

        centerOfMassTilde: tilde matrix from local position of COM; autocomputed during initialization; type: array_like

        physicsMass: deprecated since 1.12.258, removed in 2031: use mass

        physicsInertia: deprecated since 1.12.258, removed in 2031: use inertia

        physicsCenterOfMass: deprecated since 1.12.258, removed in 2031: use centerOfMass

        physicsCenterOfMassTilde: deprecated since 1.12.258, removed in 2031: use centerOfMassTilde

        visualization: visualization data, see VObjectFFRFreducedOrder

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``, ``SuperElement``

    """
    def __init__(self, name = '', nodeNumbers = [], massMatrixReduced = None, stiffnessMatrixReduced = None, dampingMatrixReduced = None, forceUserFunction: Union[ObjectFFRFreducedOrderForceUserFunction, int] = 0, massMatrixUserFunction: Union[ObjectFFRFreducedOrderMassMatrixUserFunction, int] = 0, computeFFRFterms = True, modeBasis = [], outputVariableModeBasis = [], outputVariableTypeModeBasis = 0, referencePositions = [], objectIsInitialized = False, mass = 0., inertia = IIDiagMatrix(rowsColumns=3,value=1), centerOfMass = [0.,0.,0.], mPsiTildePsi = [], mPsiTildePsiTilde = [], mPhitTPsi = [], mPhitTPsiTilde = [], mXRefTildePsi = [], mXRefTildePsiTilde = [], centerOfMassTilde = IIDiagMatrix(rowsColumns=3,value=0), physicsMass = None, physicsInertia = None, physicsCenterOfMass = None, physicsCenterOfMassTilde = None, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.], 'triangleMesh': [], 'showNodes': False}):
        self.name = name
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.massMatrixReduced = massMatrixReduced
        self.stiffnessMatrixReduced = stiffnessMatrixReduced
        self.dampingMatrixReduced = dampingMatrixReduced
        self.forceUserFunction = forceUserFunction
        self.massMatrixUserFunction = massMatrixUserFunction
        self.computeFFRFterms = computeFFRFterms
        self.modeBasis = CheckForValidNumpyArray(modeBasis)
        self.outputVariableModeBasis = CheckForValidNumpyArray(outputVariableModeBasis)
        self.outputVariableTypeModeBasis = outputVariableTypeModeBasis
        self.referencePositions = CheckForValidNumpyArray(referencePositions)
        self.objectIsInitialized = objectIsInitialized
        self.mass = mass
        self.inertia = np.array(inertia)
        self.centerOfMass = np.array(centerOfMass)
        self.mPsiTildePsi = CheckForValidNumpyArray(mPsiTildePsi)
        self.mPsiTildePsiTilde = CheckForValidNumpyArray(mPsiTildePsiTilde)
        self.mPhitTPsi = CheckForValidNumpyArray(mPhitTPsi)
        self.mPhitTPsiTilde = CheckForValidNumpyArray(mPhitTPsiTilde)
        self.mXRefTildePsi = CheckForValidNumpyArray(mXRefTildePsi)
        self.mXRefTildePsiTilde = CheckForValidNumpyArray(mXRefTildePsiTilde)
        self.centerOfMassTilde = np.array(centerOfMassTilde)
        self.physicsMass = physicsMass
        self.physicsInertia = physicsInertia
        self.physicsCenterOfMass = physicsCenterOfMass
        self.physicsCenterOfMassTilde = physicsCenterOfMassTilde
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'FFRFreducedOrder'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'massMatrixReduced', self.massMatrixReduced
        yield 'stiffnessMatrixReduced', self.stiffnessMatrixReduced
        yield 'dampingMatrixReduced', self.dampingMatrixReduced
        yield 'forceUserFunction', self.forceUserFunction
        yield 'massMatrixUserFunction', self.massMatrixUserFunction
        yield 'computeFFRFterms', self.computeFFRFterms
        yield 'modeBasis', self.modeBasis
        yield 'outputVariableModeBasis', self.outputVariableModeBasis
        yield 'outputVariableTypeModeBasis', self.outputVariableTypeModeBasis
        yield 'referencePositions', self.referencePositions
        yield 'objectIsInitialized', self.objectIsInitialized
        yield 'mass', self.mass
        yield 'inertia', self.inertia
        yield 'centerOfMass', self.centerOfMass
        yield 'mPsiTildePsi', self.mPsiTildePsi
        yield 'mPsiTildePsiTilde', self.mPsiTildePsiTilde
        yield 'mPhitTPsi', self.mPhitTPsi
        yield 'mPhitTPsiTilde', self.mPhitTPsiTilde
        yield 'mXRefTildePsi', self.mXRefTildePsi
        yield 'mXRefTildePsiTilde', self.mXRefTildePsiTilde
        yield 'centerOfMassTilde', self.centerOfMassTilde
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]
        yield 'VtriangleMesh', dict(self.visualization)["triangleMesh"]
        yield 'VshowNodes', dict(self.visualization)["showNodes"]
        if self.physicsMass is not None:
            yield 'physicsMass', self.physicsMass
        if self.physicsInertia is not None:
            yield 'physicsInertia', self.physicsInertia
        if self.physicsCenterOfMass is not None:
            yield 'physicsCenterOfMass', self.physicsCenterOfMass
        if self.physicsCenterOfMassTilde is not None:
            yield 'physicsCenterOfMassTilde', self.physicsCenterOfMassTilde

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
CMSobject = ObjectFFRFreducedOrder
VCMSobject = VObjectFFRFreducedOrder

class VObjectANCFCable:
    """Visualization data for ObjectANCFCable.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; note that all quantities are computed at the beam centerline, even if drawn on surface of cylinder of beam; this effects, e.g., Displacement or Velocity, which is drawn constant over cross section; type: bool

        radius: if radius==0, only the centerline is drawn; else, a cylinder with radius is drawn; circumferential tiling follows general.cylinderTiling and beam axis tiling follows bodies.beams.axialTiling; type: float

        color: RGBA color of the object; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, radius = 0., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.radius = radius
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'radius', self.radius
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectANCFCable:
    r"""A 3D cable finite element using 2 nodes of type NodePointSlope1.
    
    The localPosition of the beam with length :math:`L`=length and height :math:`h` ranges in :math:`X`-direction in range :math:`[0, L]` and in :math:`Y`-direction in range :math:`[-h/2,h/2]` (which is in fact not needed in the EOM). For description see ObjectANCFCable2D, which is almost identical to 3D case. NOTE: this element does not include torsion, therfore a torque cannot be applied along the local x-axis.
    
    Args:
        name: objects's unique name; type: str

        length: [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives :math:`\rho A L`; must be positive; type: float

        massPerLength: [SI:kg/m] mass per length of beam; type: float

        bendingStiffness: [SI:Nm:math:`^2`] bending stiffness of beam; the bending moment is :math:`m = EI (\kappa - \kappa_0)`, in which :math:`\kappa` is the material measure of curvature; type: float

        axialStiffness: [SI:N] axial stiffness of beam; the axial force is :math:`f_{ax} = EA (\varepsilon -\varepsilon_0)`, in which :math:`\varepsilon = |\rv^\prime|-1` is the axial strain; type: float

        bendingDamping: [SI:Nm:math:`^2`/s] bending damping of beam ; the additional virtual work due to damping is :math:`\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx`; type: float

        axialDamping: [SI:N/s] axial damping of beam; the additional virtual work due to damping is :math:`\delta W_{\dot\varepsilon} = \int_0^L \dot \varepsilon \delta \varepsilon dx`; type: float

        referenceAxialStrain: [SI:1] reference axial strain of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference axial strain value; type: float

        strainIsRelativeToReference: if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration plus the values of :math:`\varepsilon_0` and :math:`\kappa_0` serve as a reference geometry; allows also values between 0. and 1.; type: float

        nodeNumbers: two node numbers ANCF cable element; type: NodeIndex2

        useReducedOrderIntegration: 0/false: use Gauss order 9 integration for virtual work of axial forces, order 5 for virtual work of bending moments; 1/true: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments

        physicsLength: deprecated since 1.12.258, removed in 2031: use length

        physicsMassPerLength: deprecated since 1.12.258, removed in 2031: use massPerLength

        physicsBendingStiffness: deprecated since 1.12.258, removed in 2031: use bendingStiffness

        physicsAxialStiffness: deprecated since 1.12.258, removed in 2031: use axialStiffness

        physicsBendingDamping: deprecated since 1.12.258, removed in 2031: use bendingDamping

        physicsAxialDamping: deprecated since 1.12.258, removed in 2031: use axialDamping

        physicsReferenceAxialStrain: deprecated since 1.12.258, removed in 2031: use referenceAxialStrain

        visualization: visualization data, see VObjectANCFCable

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``

        Requested Node type: ``Position``

    """
    def __init__(self, name = '', length = 0., massPerLength = 0., bendingStiffness = 0., axialStiffness = 0., bendingDamping = 0., axialDamping = 0., referenceAxialStrain = 0., strainIsRelativeToReference = 0., nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex()], useReducedOrderIntegration = 0, physicsLength = None, physicsMassPerLength = None, physicsBendingStiffness = None, physicsAxialStiffness = None, physicsBendingDamping = None, physicsAxialDamping = None, physicsReferenceAxialStrain = None, visualization = {'show': True, 'radius': 0., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.length = length
        self.massPerLength = massPerLength
        self.bendingStiffness = bendingStiffness
        self.axialStiffness = axialStiffness
        self.bendingDamping = bendingDamping
        self.axialDamping = axialDamping
        self.referenceAxialStrain = referenceAxialStrain
        self.strainIsRelativeToReference = strainIsRelativeToReference
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.useReducedOrderIntegration = useReducedOrderIntegration
        self.physicsLength = physicsLength
        self.physicsMassPerLength = physicsMassPerLength
        self.physicsBendingStiffness = physicsBendingStiffness
        self.physicsAxialStiffness = physicsAxialStiffness
        self.physicsBendingDamping = physicsBendingDamping
        self.physicsAxialDamping = physicsAxialDamping
        self.physicsReferenceAxialStrain = physicsReferenceAxialStrain
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ANCFCable'
        yield 'name', self.name
        yield 'length', self.length
        yield 'massPerLength', self.massPerLength
        yield 'bendingStiffness', self.bendingStiffness
        yield 'axialStiffness', self.axialStiffness
        yield 'bendingDamping', self.bendingDamping
        yield 'axialDamping', self.axialDamping
        yield 'referenceAxialStrain', self.referenceAxialStrain
        yield 'strainIsRelativeToReference', self.strainIsRelativeToReference
        yield 'nodeNumbers', self.nodeNumbers
        yield 'useReducedOrderIntegration', self.useReducedOrderIntegration
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vradius', dict(self.visualization)["radius"]
        yield 'Vcolor', dict(self.visualization)["color"]
        if self.physicsLength is not None:
            yield 'physicsLength', self.physicsLength
        if self.physicsMassPerLength is not None:
            yield 'physicsMassPerLength', self.physicsMassPerLength
        if self.physicsBendingStiffness is not None:
            yield 'physicsBendingStiffness', self.physicsBendingStiffness
        if self.physicsAxialStiffness is not None:
            yield 'physicsAxialStiffness', self.physicsAxialStiffness
        if self.physicsBendingDamping is not None:
            yield 'physicsBendingDamping', self.physicsBendingDamping
        if self.physicsAxialDamping is not None:
            yield 'physicsAxialDamping', self.physicsAxialDamping
        if self.physicsReferenceAxialStrain is not None:
            yield 'physicsReferenceAxialStrain', self.physicsReferenceAxialStrain

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
Cable = ObjectANCFCable
VCable = VObjectANCFCable

class VObjectANCFCable2D:
    """Visualization data for ObjectANCFCable2D.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawHeight: if beam is drawn with rectangular shape, this is the drawing height; type: float

        color: RGBA color of the object; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawHeight = 0., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawHeight = drawHeight
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawHeight', self.drawHeight
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectANCFCable2D:
    r"""A 2D cable finite element using 2 nodes of type NodePoint2DSlope1.
    
    The localPosition of the beam with length :math:`L`=length and height :math:`h` ranges in :math:`X`-direction in range :math:`[0, L]` and in :math:`Y`-direction in range :math:`[-h/2,h/2]` (which is in fact not needed in the EOM).
    
    Args:
        name: objects's unique name; type: str

        length: [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives :math:`\rho A L`; must be positive; type: float

        massPerLength: [SI:kg/m] mass per length of beam; type: float

        bendingStiffness: [SI:Nm:math:`^2`] bending stiffness of beam; the bending moment is :math:`m = EI (\kappa - \kappa_0)`, in which :math:`\kappa` is the material measure of curvature; type: float

        axialStiffness: [SI:N] axial stiffness of beam; the axial force is :math:`f_{ax} = EA (\varepsilon -\varepsilon_0)`, in which :math:`\varepsilon = |\rv^\prime|-1` is the axial strain; type: float

        bendingDamping: [SI:Nm:math:`^2`/s] bending damping of beam ; the additional virtual work due to damping is :math:`\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx`; type: float

        axialDamping: [SI:N/s] axial damping of beam; the additional virtual work due to damping is :math:`\delta W_{\dot\varepsilon} = \int_0^L \dot \varepsilon \delta \varepsilon dx`; type: float

        referenceAxialStrain: [SI:1] reference axial strain of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference axial strain value; type: float

        referenceCurvature: [SI:1/m] reference curvature of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference curvature value; type: float

        strainIsRelativeToReference: if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration plus the values of :math:`\varepsilon_0` and :math:`\kappa_0` serve as a reference geometry; allows also values between 0. and 1.; type: float

        nodeNumbers: two node numbers ANCF cable element; type: NodeIndex2

        useReducedOrderIntegration: 0/false: use Gauss order 9 integration for virtual work of axial forces, order 5 for virtual work of bending moments; 1/True: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments; 2: use mixed Lobatto/Gauss integration with exceptional quality of axial strain, however, spurious (hourglass) modes may occur!

        axialForceUserFunction: A Python function which defines the (nonlinear relations) of local strains (including axial strain and bending strain) as well as time derivatives to the local axial force; see description below; type: ObjectANCFCable2DAxialForceUserFunction

        bendingMomentUserFunction: A Python function which defines the (nonlinear relations) of local strains (including axial strain and bending strain) as well as time derivatives to the local bending moment; see description below; type: ObjectANCFCable2DBendingMomentUserFunction

        physicsLength: deprecated since 1.12.258, removed in 2031: use length

        physicsMassPerLength: deprecated since 1.12.258, removed in 2031: use massPerLength

        physicsBendingStiffness: deprecated since 1.12.258, removed in 2031: use bendingStiffness

        physicsAxialStiffness: deprecated since 1.12.258, removed in 2031: use axialStiffness

        physicsBendingDamping: deprecated since 1.12.258, removed in 2031: use bendingDamping

        physicsAxialDamping: deprecated since 1.12.258, removed in 2031: use axialDamping

        physicsReferenceAxialStrain: deprecated since 1.12.258, removed in 2031: use referenceAxialStrain

        physicsReferenceCurvature: deprecated since 1.12.258, removed in 2031: use referenceCurvature

        visualization: visualization data, see VObjectANCFCable2D

    Notes:
        Requested Node type: ``Position2D`` + ``Orientation2D`` + ``Point2DSlope1`` + ``Position`` + ``Orientation``

    """
    def __init__(self, name = '', length = 0., massPerLength = 0., bendingStiffness = 0., axialStiffness = 0., bendingDamping = 0., axialDamping = 0., referenceAxialStrain = 0., referenceCurvature = 0., strainIsRelativeToReference = 0., nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex()], useReducedOrderIntegration = 0, axialForceUserFunction: Union[ObjectANCFCable2DAxialForceUserFunction, int] = 0, bendingMomentUserFunction: Union[ObjectANCFCable2DBendingMomentUserFunction, int] = 0, physicsLength = None, physicsMassPerLength = None, physicsBendingStiffness = None, physicsAxialStiffness = None, physicsBendingDamping = None, physicsAxialDamping = None, physicsReferenceAxialStrain = None, physicsReferenceCurvature = None, visualization = {'show': True, 'drawHeight': 0., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.length = length
        self.massPerLength = massPerLength
        self.bendingStiffness = bendingStiffness
        self.axialStiffness = axialStiffness
        self.bendingDamping = bendingDamping
        self.axialDamping = axialDamping
        self.referenceAxialStrain = referenceAxialStrain
        self.referenceCurvature = referenceCurvature
        self.strainIsRelativeToReference = strainIsRelativeToReference
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.useReducedOrderIntegration = useReducedOrderIntegration
        self.axialForceUserFunction = axialForceUserFunction
        self.bendingMomentUserFunction = bendingMomentUserFunction
        self.physicsLength = physicsLength
        self.physicsMassPerLength = physicsMassPerLength
        self.physicsBendingStiffness = physicsBendingStiffness
        self.physicsAxialStiffness = physicsAxialStiffness
        self.physicsBendingDamping = physicsBendingDamping
        self.physicsAxialDamping = physicsAxialDamping
        self.physicsReferenceAxialStrain = physicsReferenceAxialStrain
        self.physicsReferenceCurvature = physicsReferenceCurvature
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ANCFCable2D'
        yield 'name', self.name
        yield 'length', self.length
        yield 'massPerLength', self.massPerLength
        yield 'bendingStiffness', self.bendingStiffness
        yield 'axialStiffness', self.axialStiffness
        yield 'bendingDamping', self.bendingDamping
        yield 'axialDamping', self.axialDamping
        yield 'referenceAxialStrain', self.referenceAxialStrain
        yield 'referenceCurvature', self.referenceCurvature
        yield 'strainIsRelativeToReference', self.strainIsRelativeToReference
        yield 'nodeNumbers', self.nodeNumbers
        yield 'useReducedOrderIntegration', self.useReducedOrderIntegration
        yield 'axialForceUserFunction', self.axialForceUserFunction
        yield 'bendingMomentUserFunction', self.bendingMomentUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawHeight', dict(self.visualization)["drawHeight"]
        yield 'Vcolor', dict(self.visualization)["color"]
        if self.physicsLength is not None:
            yield 'physicsLength', self.physicsLength
        if self.physicsMassPerLength is not None:
            yield 'physicsMassPerLength', self.physicsMassPerLength
        if self.physicsBendingStiffness is not None:
            yield 'physicsBendingStiffness', self.physicsBendingStiffness
        if self.physicsAxialStiffness is not None:
            yield 'physicsAxialStiffness', self.physicsAxialStiffness
        if self.physicsBendingDamping is not None:
            yield 'physicsBendingDamping', self.physicsBendingDamping
        if self.physicsAxialDamping is not None:
            yield 'physicsAxialDamping', self.physicsAxialDamping
        if self.physicsReferenceAxialStrain is not None:
            yield 'physicsReferenceAxialStrain', self.physicsReferenceAxialStrain
        if self.physicsReferenceCurvature is not None:
            yield 'physicsReferenceCurvature', self.physicsReferenceCurvature

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
Cable2D = ObjectANCFCable2D
VCable2D = VObjectANCFCable2D

class VObjectALEANCFCable2D:
    """Visualization data for ObjectALEANCFCable2D.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawHeight: if beam is drawn with rectangular shape, this is the drawing height; type: float

        color: RGBA color of the object; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawHeight = 0., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawHeight = drawHeight
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawHeight', self.drawHeight
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectALEANCFCable2D:
    r"""A 2D cable finite element using 2 nodes of type NodePoint2DSlope1 and a axially moving coordinate of type NodeGenericODE2, which adds additional (redundant) motion in axial direction of the beam.
    
    This allows modeling pipes but also axially moving beams. The localPosition of the beam with length :math:`L`=length and height :math:`h` ranges in :math:`X`-direction in range :math:`[0, L]` and in :math:`Y`-direction in range :math:`[-h/2,h/2]` (which is in fact not needed in the EOM).
    
    Args:
        name: objects's unique name; type: str

        length: [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives :math:`\rho A L`; must be positive; type: float

        massPerLength: [SI:kg/m] total mass per length of beam (including axially moving parts / fluid); type: float

        movingMassFactor: this factor denotes the amount of :math:`\rho A` which is moving; movingMassFactor=1 means, that all mass is moving; movingMassFactor=0 means, that no mass is moving; factor can be used to simulate e.g. pipe conveying fluid, in which :math:`\rho A` is the mass of the pipe+fluid, while :math:`movingMassFactor \cdot \rho A` is the mass per unit length of the fluid; type: float

        bendingStiffness: [SI:Nm:math:`^2`] bending stiffness of beam; the bending moment is :math:`m = EI (\kappa - \kappa_0)`, in which :math:`\kappa` is the material measure of curvature; type: float

        axialStiffness: [SI:N] axial stiffness of beam; the axial force is :math:`f_{ax} = EA (\varepsilon -\varepsilon_0)`, in which :math:`\varepsilon = |\rv^\prime|-1` is the axial strain; type: float

        bendingDamping: [SI:Nm:math:`^2`/s] bending damping of beam ; the additional virtual work due to damping is :math:`\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx`; type: float

        axialDamping: [SI:N/s] axial damping of beam; the additional virtual work due to damping is :math:`\delta W_{\dot\varepsilon} = \int_0^L \dot \varepsilon \delta \varepsilon dx`; type: float

        referenceAxialStrain: [SI:1] reference axial strain of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference axial strain value; type: float

        referenceCurvature: [SI:1/m] reference curvature of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference curvature value; type: float

        useCouplingTerms: true: correct case, where all coupling terms due to moving mass are respected; false: only include constant mass for ALE node coordinate, but deactivate other coupling terms (behaves like ANCFCable2D then); type: bool

        addALEvariation: true: correct case, where additional terms related to variation of strain and curvature are added; type: bool

        nodeNumbers: two node numbers ANCF cable element, third node=ALE GenericODE2 node; type: NodeIndex3

        useReducedOrderIntegration: 0/false: use Gauss order 9 integration for virtual work of axial forces, order 5 for virtual work of bending moments; 1/true: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments

        strainIsRelativeToReference: if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration plus the values of :math:`\varepsilon_0` and :math:`\kappa_0` serve as a reference geometry; allows also values between 0. and 1.; type: float

        physicsLength: deprecated since 1.12.258, removed in 2031: use length

        physicsMassPerLength: deprecated since 1.12.258, removed in 2031: use massPerLength

        physicsMovingMassFactor: deprecated since 1.12.258, removed in 2031: use movingMassFactor

        physicsBendingStiffness: deprecated since 1.12.258, removed in 2031: use bendingStiffness

        physicsAxialStiffness: deprecated since 1.12.258, removed in 2031: use axialStiffness

        physicsBendingDamping: deprecated since 1.12.258, removed in 2031: use bendingDamping

        physicsAxialDamping: deprecated since 1.12.258, removed in 2031: use axialDamping

        physicsReferenceAxialStrain: deprecated since 1.12.258, removed in 2031: use referenceAxialStrain

        physicsReferenceCurvature: deprecated since 1.12.258, removed in 2031: use referenceCurvature

        physicsUseCouplingTerms: deprecated since 1.12.258, removed in 2031: use useCouplingTerms

        physicsAddALEvariation: deprecated since 1.12.258, removed in 2031: use addALEvariation

        visualization: visualization data, see VObjectALEANCFCable2D

    """
    def __init__(self, name = '', length = 0., massPerLength = 0., movingMassFactor = 1., bendingStiffness = 0., axialStiffness = 0., bendingDamping = 0., axialDamping = 0., referenceAxialStrain = 0., referenceCurvature = 0., useCouplingTerms = True, addALEvariation = True, nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex(), exudyn.InvalidIndex()], useReducedOrderIntegration = 0, strainIsRelativeToReference = 0., physicsLength = None, physicsMassPerLength = None, physicsMovingMassFactor = None, physicsBendingStiffness = None, physicsAxialStiffness = None, physicsBendingDamping = None, physicsAxialDamping = None, physicsReferenceAxialStrain = None, physicsReferenceCurvature = None, physicsUseCouplingTerms = None, physicsAddALEvariation = None, visualization = {'show': True, 'drawHeight': 0., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.length = length
        self.massPerLength = massPerLength
        self.movingMassFactor = movingMassFactor
        self.bendingStiffness = bendingStiffness
        self.axialStiffness = axialStiffness
        self.bendingDamping = bendingDamping
        self.axialDamping = axialDamping
        self.referenceAxialStrain = referenceAxialStrain
        self.referenceCurvature = referenceCurvature
        self.useCouplingTerms = useCouplingTerms
        self.addALEvariation = addALEvariation
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.useReducedOrderIntegration = useReducedOrderIntegration
        self.strainIsRelativeToReference = strainIsRelativeToReference
        self.physicsLength = physicsLength
        self.physicsMassPerLength = physicsMassPerLength
        self.physicsMovingMassFactor = physicsMovingMassFactor
        self.physicsBendingStiffness = physicsBendingStiffness
        self.physicsAxialStiffness = physicsAxialStiffness
        self.physicsBendingDamping = physicsBendingDamping
        self.physicsAxialDamping = physicsAxialDamping
        self.physicsReferenceAxialStrain = physicsReferenceAxialStrain
        self.physicsReferenceCurvature = physicsReferenceCurvature
        self.physicsUseCouplingTerms = physicsUseCouplingTerms
        self.physicsAddALEvariation = physicsAddALEvariation
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ALEANCFCable2D'
        yield 'name', self.name
        yield 'length', self.length
        yield 'massPerLength', self.massPerLength
        yield 'movingMassFactor', self.movingMassFactor
        yield 'bendingStiffness', self.bendingStiffness
        yield 'axialStiffness', self.axialStiffness
        yield 'bendingDamping', self.bendingDamping
        yield 'axialDamping', self.axialDamping
        yield 'referenceAxialStrain', self.referenceAxialStrain
        yield 'referenceCurvature', self.referenceCurvature
        yield 'useCouplingTerms', self.useCouplingTerms
        yield 'addALEvariation', self.addALEvariation
        yield 'nodeNumbers', self.nodeNumbers
        yield 'useReducedOrderIntegration', self.useReducedOrderIntegration
        yield 'strainIsRelativeToReference', self.strainIsRelativeToReference
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawHeight', dict(self.visualization)["drawHeight"]
        yield 'Vcolor', dict(self.visualization)["color"]
        if self.physicsLength is not None:
            yield 'physicsLength', self.physicsLength
        if self.physicsMassPerLength is not None:
            yield 'physicsMassPerLength', self.physicsMassPerLength
        if self.physicsMovingMassFactor is not None:
            yield 'physicsMovingMassFactor', self.physicsMovingMassFactor
        if self.physicsBendingStiffness is not None:
            yield 'physicsBendingStiffness', self.physicsBendingStiffness
        if self.physicsAxialStiffness is not None:
            yield 'physicsAxialStiffness', self.physicsAxialStiffness
        if self.physicsBendingDamping is not None:
            yield 'physicsBendingDamping', self.physicsBendingDamping
        if self.physicsAxialDamping is not None:
            yield 'physicsAxialDamping', self.physicsAxialDamping
        if self.physicsReferenceAxialStrain is not None:
            yield 'physicsReferenceAxialStrain', self.physicsReferenceAxialStrain
        if self.physicsReferenceCurvature is not None:
            yield 'physicsReferenceCurvature', self.physicsReferenceCurvature
        if self.physicsUseCouplingTerms is not None:
            yield 'physicsUseCouplingTerms', self.physicsUseCouplingTerms
        if self.physicsAddALEvariation is not None:
            yield 'physicsAddALEvariation', self.physicsAddALEvariation

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
ALECable2D = ObjectALEANCFCable2D
VALECable2D = VObjectALEANCFCable2D

class VObjectANCFBeam:
    """Visualization data for ObjectANCFBeam.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; geometry is defined by sectionGeometry; type: bool

        sectionGeometry: defines cross section shape used for visualization and contact; type: BeamSectionGeometry

        color: RGBA color of the object; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, sectionGeometry = exudyn.BeamSectionGeometry(), color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.sectionGeometry = sectionGeometry
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'sectionGeometry', self.sectionGeometry
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectANCFBeam:
    r"""A 3D beam finite element based on the absolute nodal coordinate formulation, using two nodes.
    
    The localPosition :math:`x` of the beam ranges from :math:`-L/2` (at node 0) to :math:`L/2` (at node 1). The axial coordinate is :math:`x` (first coordinate) and the cross section is spanned by local :math:`y`/:math:`z` axes; assuming dimensions :math:`w_y` and :math:`w_z` in cross section, the local position range is :math:`\in [[-L/2,L/2],\, [-wy/2,wy/2],\, [-wz/2,wz/2] ]`. NOTE: Requires further development and tests!
    
    Args:
        name: objects's unique name; type: str

        nodeNumbers: two node numbers for beam element; type: NodeIndex2

        length: [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives :math:`\rho A L`; must be positive; type: float

        sectionData: data as given by exudyn.BeamSection(), defining inertial, stiffness and damping parameters of beam section.

        crossSectionPenaltyFactor: [SI:1] additional penalty factors for cross section deformation, which are in total :math:`k_{cs} = [f_{yy}\cdot EA,\, f_{zz}\cdot EA,\, f_{yz}\cdot (GA_y+GA_z)]\tp`; type: [float,float,float]

        crossSectionDamping: [SI:1] viscous damping according to penalty factors for cross section deformation; the damping is relative to the stiffness and should be thus usually much smaller than 1; the viscous damping factors read  :math:`d_{cs} = [d_{fyy}\cdot EA,\, d_{fzz}\cdot EA,\, d_{fyz}\cdot (GA_y+GA_z)]\tp`; type: [float,float,float]

        physicsLength: deprecated since 1.12.258, removed in 2031: use length

        visualization: visualization data, see VObjectANCFBeam

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``

        Requested Node type: ``Position`` + ``Orientation``

    """
    def __init__(self, name = '', nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex()], length = 0., sectionData = exudyn.BeamSection(), crossSectionPenaltyFactor = [1.,1.,1.], crossSectionDamping = [0.,0.,0.], physicsLength = None, visualization = {'show': True, 'sectionGeometry': exudyn.BeamSectionGeometry(), 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.length = length
        self.sectionData = sectionData
        self.crossSectionPenaltyFactor = np.array(crossSectionPenaltyFactor)
        self.crossSectionDamping = np.array(crossSectionDamping)
        self.physicsLength = physicsLength
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ANCFBeam'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'length', self.length
        yield 'sectionData', self.sectionData
        yield 'crossSectionPenaltyFactor', self.crossSectionPenaltyFactor
        yield 'crossSectionDamping', self.crossSectionDamping
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VsectionGeometry', dict(self.visualization)["sectionGeometry"]
        yield 'Vcolor', dict(self.visualization)["color"]
        if self.physicsLength is not None:
            yield 'physicsLength', self.physicsLength

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
ANCFBeam = ObjectANCFBeam
VANCFBeam = VObjectANCFBeam

class VObjectBeamGeometricallyExact2D:
    """Visualization data for ObjectBeamGeometricallyExact2D.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawHeight: if beam is drawn with rectangular shape, this is the drawing height; type: float

        color: RGBA color of the object; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawHeight = 0., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawHeight = drawHeight
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawHeight', self.drawHeight
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectBeamGeometricallyExact2D:
    r"""A 2D geometrically exact beam finite element, using 2 or 3 nodes of type NodeRigidBody2D.
    
    Note that the orientation of the nodes need to follow the cross section orientation in case that includeReferenceRotations=True; e.g., an angle 0 represents the cross section aligned with the :math:`y`-axis, while and angle :math:`\pi/2` means that the cross section points in negative :math:`x`-direction. Pre-curvature can be included with referenceCurvature and axial pre-stress can be considered by using a length different from the reference configuration of the nodes. The localPosition of the beam with length :math:`L`=length and height :math:`h` ranges in :math:`X`-direction in range :math:`[-L/2, L/2]` and in :math:`Y`-direction in range :math:`[-h/2,h/2]` (which is in fact not needed in the EOM).
    
    Args:
        name: objects's unique name; type: str

        nodeNumbers: two node numbers for beam element; type: ArrayNodeIndex

        length: [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives :math:`\rho A L`; must be positive; type: float

        massPerLength: [SI:kg/m] mass per length of beam; type: float

        crossSectionInertia: [SI:kg m] cross section mass moment of inertia; inertia acting against rotation of cross section; type: float

        bendingStiffness: [SI:Nm:math:`^2`] bending stiffness of beam; the bending moment is :math:`m = EI (\kappa - \kappa_0)`, in which :math:`\kappa` is the material measure of curvature; type: float

        axialStiffness: [SI:N] axial stiffness of beam; the axial force is :math:`f_{ax} = EA (\varepsilon -\varepsilon_0)`, in which :math:`\varepsilon` is the axial strain; type: float

        shearStiffness: [SI:N] effective shear stiffness of beam, including stiffness correction; type: float

        bendingDamping: [SI:Nm:math:`^2`/s] viscous damping of bending deformation; the additional virtual work due to damping is :math:`\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx`; type: float

        axialDamping: [SI:N/s] viscous damping of axial deformation; type: float

        shearDamping: [SI:N/s] viscous damping of shear deformation; type: float

        referenceCurvature: [SI:1/m] reference curvature of beam (pre-deformation) of beam; type: float

        includeReferenceRotations: if True, rotation of the cross section at the nodes includes node reference rotations (within referenceCoordinates of NodeRigidBody2D), which are used for the computation of bending strains (this means that a pre-curved beam is stress-free); if False, the reference rotation of the cross section is orthogonal to the reference slope vector. This allows to easily share nodes among several beams with different reference cross section orientation (i.e., only the change of rotation counts).; type: bool

        physicsLength: deprecated since 1.12.258, removed in 2031: use length

        physicsMassPerLength: deprecated since 1.12.258, removed in 2031: use massPerLength

        physicsCrossSectionInertia: deprecated since 1.12.258, removed in 2031: use crossSectionInertia

        physicsBendingStiffness: deprecated since 1.12.258, removed in 2031: use bendingStiffness

        physicsAxialStiffness: deprecated since 1.12.258, removed in 2031: use axialStiffness

        physicsShearStiffness: deprecated since 1.12.258, removed in 2031: use shearStiffness

        physicsBendingDamping: deprecated since 1.12.258, removed in 2031: use bendingDamping

        physicsAxialDamping: deprecated since 1.12.258, removed in 2031: use axialDamping

        physicsShearDamping: deprecated since 1.12.258, removed in 2031: use shearDamping

        physicsReferenceCurvature: deprecated since 1.12.258, removed in 2031: use referenceCurvature

        visualization: visualization data, see VObjectBeamGeometricallyExact2D

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``

        Requested Node type: ``Position2D`` + ``Orientation2D`` + ``Position`` + ``Orientation``

    """
    def __init__(self, name = '', nodeNumbers = [], length = 0., massPerLength = 0., crossSectionInertia = 0., bendingStiffness = 0., axialStiffness = 0., shearStiffness = 0., bendingDamping = 0., axialDamping = 0., shearDamping = 0., referenceCurvature = 0., includeReferenceRotations = False, physicsLength = None, physicsMassPerLength = None, physicsCrossSectionInertia = None, physicsBendingStiffness = None, physicsAxialStiffness = None, physicsShearStiffness = None, physicsBendingDamping = None, physicsAxialDamping = None, physicsShearDamping = None, physicsReferenceCurvature = None, visualization = {'show': True, 'drawHeight': 0., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.length = length
        self.massPerLength = massPerLength
        self.crossSectionInertia = crossSectionInertia
        self.bendingStiffness = bendingStiffness
        self.axialStiffness = axialStiffness
        self.shearStiffness = shearStiffness
        self.bendingDamping = bendingDamping
        self.axialDamping = axialDamping
        self.shearDamping = shearDamping
        self.referenceCurvature = referenceCurvature
        self.includeReferenceRotations = includeReferenceRotations
        self.physicsLength = physicsLength
        self.physicsMassPerLength = physicsMassPerLength
        self.physicsCrossSectionInertia = physicsCrossSectionInertia
        self.physicsBendingStiffness = physicsBendingStiffness
        self.physicsAxialStiffness = physicsAxialStiffness
        self.physicsShearStiffness = physicsShearStiffness
        self.physicsBendingDamping = physicsBendingDamping
        self.physicsAxialDamping = physicsAxialDamping
        self.physicsShearDamping = physicsShearDamping
        self.physicsReferenceCurvature = physicsReferenceCurvature
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'BeamGeometricallyExact2D'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'length', self.length
        yield 'massPerLength', self.massPerLength
        yield 'crossSectionInertia', self.crossSectionInertia
        yield 'bendingStiffness', self.bendingStiffness
        yield 'axialStiffness', self.axialStiffness
        yield 'shearStiffness', self.shearStiffness
        yield 'bendingDamping', self.bendingDamping
        yield 'axialDamping', self.axialDamping
        yield 'shearDamping', self.shearDamping
        yield 'referenceCurvature', self.referenceCurvature
        yield 'includeReferenceRotations', self.includeReferenceRotations
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawHeight', dict(self.visualization)["drawHeight"]
        yield 'Vcolor', dict(self.visualization)["color"]
        if self.physicsLength is not None:
            yield 'physicsLength', self.physicsLength
        if self.physicsMassPerLength is not None:
            yield 'physicsMassPerLength', self.physicsMassPerLength
        if self.physicsCrossSectionInertia is not None:
            yield 'physicsCrossSectionInertia', self.physicsCrossSectionInertia
        if self.physicsBendingStiffness is not None:
            yield 'physicsBendingStiffness', self.physicsBendingStiffness
        if self.physicsAxialStiffness is not None:
            yield 'physicsAxialStiffness', self.physicsAxialStiffness
        if self.physicsShearStiffness is not None:
            yield 'physicsShearStiffness', self.physicsShearStiffness
        if self.physicsBendingDamping is not None:
            yield 'physicsBendingDamping', self.physicsBendingDamping
        if self.physicsAxialDamping is not None:
            yield 'physicsAxialDamping', self.physicsAxialDamping
        if self.physicsShearDamping is not None:
            yield 'physicsShearDamping', self.physicsShearDamping
        if self.physicsReferenceCurvature is not None:
            yield 'physicsReferenceCurvature', self.physicsReferenceCurvature

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
Beam2D = ObjectBeamGeometricallyExact2D
VBeam2D = VObjectBeamGeometricallyExact2D

class VObjectBeamGeometricallyExact:
    """Visualization data for ObjectBeamGeometricallyExact.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; geometry is defined by sectionGeometry; type: bool

        sectionGeometry: defines cross section shape used for visualization and contact; type: BeamSectionGeometry

        color: RGBA color of the object; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, sectionGeometry = exudyn.BeamSectionGeometry(), color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.sectionGeometry = sectionGeometry
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'sectionGeometry', self.sectionGeometry
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectBeamGeometricallyExact:
    r"""A 3D geometrically exact (shear deformable) beam finite element with two 3D rigid body nodes, interpolated on SE(3).
    
    The localPosition :math:`x` of the beam ranges from :math:`-L/2` (at node 0) to :math:`L/2` (at node 1); the axial coordinate is :math:`x` (first coordinate) and the cross section is spanned by the local :math:`y`- and :math:`z`-axes.
    
    Args:
        name: objects's unique name; type: str

        nodeNumbers: two node numbers for beam element; type: NodeIndex2

        length: [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives :math:`\rho A L`; must be positive; type: float

        sectionData: data as given by exudyn.BeamSection(), defining inertial, stiffness and damping parameters of beam section.

        physicsLength: deprecated since 1.12.258, removed in 2031: use length

        visualization: visualization data, see VObjectBeamGeometricallyExact

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``

        Requested Node type: ``Position`` + ``Orientation``

    """
    def __init__(self, name = '', nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex()], length = 0., sectionData = exudyn.BeamSection(), physicsLength = None, visualization = {'show': True, 'sectionGeometry': exudyn.BeamSectionGeometry(), 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.length = length
        self.sectionData = sectionData
        self.physicsLength = physicsLength
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'BeamGeometricallyExact'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'length', self.length
        yield 'sectionData', self.sectionData
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VsectionGeometry', dict(self.visualization)["sectionGeometry"]
        yield 'Vcolor', dict(self.visualization)["color"]
        if self.physicsLength is not None:
            yield 'physicsLength', self.physicsLength

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
Beam3D = ObjectBeamGeometricallyExact
VBeam3D = VObjectBeamGeometricallyExact

class VObjectANCFThinPlate:
    """Visualization data for ObjectANCFThinPlate.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; the plate is drawn as n x n quads per element, n = bodies.beams.axialTiling/2, at least 2, on its surfaces if it has a thickness; the outline of the element is drawn as lines if view0.scene.showMeshEdges is set when the graphics data is built (the curved surface is drawn as flat quads, so the renderer cannot find the element edges itself); type: bool

        color: RGBA color of the object; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectANCFThinPlate:
    r"""OBJECT UNDER CONSTRUCTION: A 3D thin Kirchhoff plate finite element based on the absolute nodal coordinate formulation, using 4 nodes of type NodePointSlope12.
    
    The geometry as well as (deformed and distorted) reference configuration is given by the nodes. The localPosition follows unit-coordinates in the range [-1,1] for X, Y and Z coordinates; the thickness of the plate is h; This element is under construction.
    
    Args:
        name: objects's unique name; type: str

        thickness: [SI:m] thickness of the plate: one value for a constant thickness; 4 values, the thicknesses at the nodes in their order, interpolated bilinearly; or 12 values :math:`[h_0,\, h_{,s,0},\, h_{,t,0},\, \ldots,\, h_3,\, h_{,s,3},\, h_{,t,3}]`, the thickness and its gradients along the slopes of each node, interpolated with the 12 shape functions of the position; with 4 or 12 values, the stiffness is computed from the local thickness, see strainCoefficients; type: array_like

        density: [SI:kg/m:math:`^3`] density of the plate, possibly averaged over thickness; type: float

        massProportionalDamping: mass-proportional damping coefficient :math:`\alpha` [SI:1/s]; adds massmatrix proportional damping forces :math:`\fv_d = \alpha \Mm \dot{\qv}`; type: float

        stiffnessProportionalDamping: membrane stiffness-proportional damping coefficient :math:`\beta_\varepsilon` [SI:s]: Kelvin-Voigt damping :math:`\beta_\varepsilon\, \Dm_\varepsilon\, \dot\teps` added to the membrane forces, in the current configuration; it does not damp a rigid-body motion; type: float

        bendingStiffnessProportionalDamping: bending stiffness-proportional damping coefficient :math:`\beta_\kappa` [SI:s]: Kelvin-Voigt damping :math:`\beta_\kappa\, \Dm_\kappa\, \dot\tkappa` added to the bending moments; if negative (default), :math:`\beta_\varepsilon` of stiffnessProportionalDamping is used, 0 switches it off; type: float

        strainCoefficients: [SI:N/m] stiffness coefficients related to inplane normal and shear strains, integrated over height of the plate, as a list of 3D matrices; for a constant thickness one matrix; for 4 or 12 thickness values, the first matrix divided by thickness[0] is the material matrix of a homogeneous isotropic plate, :math:`\Dm_\varepsilon = \Dm_b\, h` and :math:`\Dm_\kappa = \Dm_b\, h^3/12` at each point, and further matrices are not used; type: Matrix3DList

        curvatureCoefficients: [SI:Nm] stiffness coefficients related to curvatures, integrated over height of the plate, as a list of 3D matrices; used for a constant thickness (one matrix); for 4 or 12 thickness values :math:`\Dm_\kappa` follows from strainCoefficients and the local thickness; type: Matrix3DList

        slopesScalingX: scaling of x-slopes at each element node; flat elements: half of the side length of the element; curved: optimal values such that curved geometry is best approximated; if negative (default) values are used, length is computed from node distances.; type: [float,float,float,float]

        slopesScalingY: scaling of y-slopes at each element node; flat elements: half of the side length of the element; curved: optimal values such that curved geometry is best approximated; if negative (default) values are used, length is computed from node distances.; type: [float,float,float,float]

        nodeNumbers: 4 NodePointSlope12 node numbers, with local (xi,eta) coordinates as [(-1,-1),(1,-1),(1,1),(-1,1)]; type: NodeIndex4

        useReducedOrderIntegration: integration of the virtual work: 0 - Gauss 5 x 5 points for the membrane and the bending terms; 1 - Lobatto 3 x 3 points for the membrane and Gauss 2 x 2 for the bending terms (disjoint points, against membrane locking); 2 - the same as 1

        physicsThickness: deprecated since 1.12.258, removed in 2031: use thickness

        physicsDensity: deprecated since 1.12.258, removed in 2031: use density

        physicsMassProportionalDamping: deprecated since 1.12.258, removed in 2031: use massProportionalDamping

        physicsStrainCoefficients: deprecated since 1.12.258, removed in 2031: use strainCoefficients

        physicsCurvatureCoefficients: deprecated since 1.12.258, removed in 2031: use curvatureCoefficients

        visualization: visualization data, see VObjectANCFThinPlate

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``

        Requested Node type: ``Position``

    """
    def __init__(self, name = '', thickness = [], density = 0., massProportionalDamping = 0., stiffnessProportionalDamping = 0., bendingStiffnessProportionalDamping = -1., strainCoefficients = None, curvatureCoefficients = None, slopesScalingX = [-1.,-1.,-1.,-1.], slopesScalingY = [-1.,-1.,-1.,-1.], nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex(), exudyn.InvalidIndex(), exudyn.InvalidIndex()], useReducedOrderIntegration = 0, physicsThickness = None, physicsDensity = None, physicsMassProportionalDamping = None, physicsStrainCoefficients = None, physicsCurvatureCoefficients = None, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.thickness = CheckForValidNumpyArray(thickness)
        self.density = density
        self.massProportionalDamping = massProportionalDamping
        self.stiffnessProportionalDamping = stiffnessProportionalDamping
        self.bendingStiffnessProportionalDamping = bendingStiffnessProportionalDamping
        self.strainCoefficients = strainCoefficients
        self.curvatureCoefficients = curvatureCoefficients
        self.slopesScalingX = np.array(slopesScalingX)
        self.slopesScalingY = np.array(slopesScalingY)
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.useReducedOrderIntegration = useReducedOrderIntegration
        self.physicsThickness = physicsThickness
        self.physicsDensity = physicsDensity
        self.physicsMassProportionalDamping = physicsMassProportionalDamping
        self.physicsStrainCoefficients = physicsStrainCoefficients
        self.physicsCurvatureCoefficients = physicsCurvatureCoefficients
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ANCFThinPlate'
        yield 'name', self.name
        yield 'thickness', self.thickness
        yield 'density', self.density
        yield 'massProportionalDamping', self.massProportionalDamping
        yield 'stiffnessProportionalDamping', self.stiffnessProportionalDamping
        yield 'bendingStiffnessProportionalDamping', self.bendingStiffnessProportionalDamping
        yield 'strainCoefficients', self.strainCoefficients
        yield 'curvatureCoefficients', self.curvatureCoefficients
        yield 'slopesScalingX', self.slopesScalingX
        yield 'slopesScalingY', self.slopesScalingY
        yield 'nodeNumbers', self.nodeNumbers
        yield 'useReducedOrderIntegration', self.useReducedOrderIntegration
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]
        if self.physicsThickness is not None:
            yield 'physicsThickness', self.physicsThickness
        if self.physicsDensity is not None:
            yield 'physicsDensity', self.physicsDensity
        if self.physicsMassProportionalDamping is not None:
            yield 'physicsMassProportionalDamping', self.physicsMassProportionalDamping
        if self.physicsStrainCoefficients is not None:
            yield 'physicsStrainCoefficients', self.physicsStrainCoefficients
        if self.physicsCurvatureCoefficients is not None:
            yield 'physicsCurvatureCoefficients', self.physicsCurvatureCoefficients

    def __repr__(self):
        return str(dict(self))

class VObjectConnectorSpringDamper:
    """Visualization data for ObjectConnectorSpringDamper.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size = diameter of spring; size == -1.f means that default connector size is used; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectConnectorSpringDamper:
    """An simple spring-damper element with additional force, connecting to position-based markers.
    
    Args:
        name: connector's unique name; type: str

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        referenceLength: reference length [SI:m] of spring; type: float

        stiffness: stiffness [SI:N/m] of spring; force acts against (length-initialLength); type: float

        damping: damping [SI:N/(m s)] of damper; force acts against d/dt(length); type: float

        force: added constant force [SI:N] of spring; scalar force; f=1 is equivalent to reducing initialLength by 1/stiffness; f > 0: tension; f < 0: compression; can be used to model actuator force; type: float

        velocityOffset: velocity offset [SI:m/s] of damper, being equivalent to time change of reference length; type: float

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        springForceUserFunction: A Python function which defines the spring force with parameters; the Python function will only be evaluated, if activeConnector is true, otherwise the SpringDamper is inactive; see description below; type: ObjectConnectorSpringDamperSpringForceUserFunction

        visualization: visualization data, see VObjectConnectorSpringDamper

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], referenceLength = 0., stiffness = 0., damping = 0., force = 0., velocityOffset = 0., activeConnector = True, springForceUserFunction: Union[ObjectConnectorSpringDamperSpringForceUserFunction, int] = 0, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.referenceLength = referenceLength
        self.stiffness = stiffness
        self.damping = damping
        self.force = force
        self.velocityOffset = velocityOffset
        self.activeConnector = activeConnector
        self.springForceUserFunction = springForceUserFunction
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ConnectorSpringDamper'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'referenceLength', self.referenceLength
        yield 'stiffness', self.stiffness
        yield 'damping', self.damping
        yield 'force', self.force
        yield 'velocityOffset', self.velocityOffset
        yield 'activeConnector', self.activeConnector
        yield 'springForceUserFunction', self.springForceUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
SpringDamper = ObjectConnectorSpringDamper
VSpringDamper = VObjectConnectorSpringDamper

class VObjectConnectorCartesianSpringDamper:
    """Visualization data for ObjectConnectorCartesianSpringDamper.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size = diameter of spring; size == -1.f means that default connector size is used; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectConnectorCartesianSpringDamper:
    """An 3D spring-damper element, providing springs and dampers in three (global) directions (x,y,z); the connector can be attached to position-based markers.
    
    Args:
        name: connector's unique name; type: str

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        stiffness: stiffness [SI:N/m] of springs; act against relative displacements in 0, 1, and 2-direction; type: [float,float,float]

        damping: damping [SI:N/(m s)] of dampers; act against relative velocities in 0, 1, and 2-direction; type: [float,float,float]

        offset: offset between two springs; type: [float,float,float]

        springForceUserFunction: A Python function which computes the 3D force vector between the two marker points, if activeConnector=True; see description below; type: ObjectConnectorCartesianSpringDamperSpringForceUserFunction

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectConnectorCartesianSpringDamper

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], stiffness = [0.,0.,0.], damping = [0.,0.,0.], offset = [0.,0.,0.], springForceUserFunction: Union[ObjectConnectorCartesianSpringDamperSpringForceUserFunction, int] = 0, activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.stiffness = np.array(stiffness)
        self.damping = np.array(damping)
        self.offset = np.array(offset)
        self.springForceUserFunction = springForceUserFunction
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ConnectorCartesianSpringDamper'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'stiffness', self.stiffness
        yield 'damping', self.damping
        yield 'offset', self.offset
        yield 'springForceUserFunction', self.springForceUserFunction
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
CartesianSpringDamper = ObjectConnectorCartesianSpringDamper
VCartesianSpringDamper = VObjectConnectorCartesianSpringDamper

class VObjectConnectorRigidBodySpringDamper:
    """Visualization data for ObjectConnectorRigidBodySpringDamper.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size = diameter of spring; size == -1.f means that default connector size is used; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectConnectorRigidBodySpringDamper:
    """An 3D spring-damper element acting on relative displacements and relative rotations of two rigid body (position+orientation) markers.
    
    It represents a penalty-based rigid joint (or prismatic, revolute, etc.)
    
    Args:
        name: connector's unique name; type: str

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        nodeNumber: node number of a NodeGenericData (size depends on application) for dataCoordinates for user functions (e.g., implementing contact/friction user function); type: NodeIndex

        stiffness: stiffness [SI:N/m or Nm/rad] of translational, torsional and coupled springs; act against relative displacements in x, y, and z-direction as well as the relative angles (calculated as Euler angles); in the simplest case, the first 3 diagonal values correspond to the local stiffness in x,y,z direction and the last 3 diagonal values correspond to the rotational stiffness around x,y and z axis; type: array_like

        damping: damping [SI:N/(m/s) or Nm/(rad/s)] of translational, torsional and coupled dampers; very similar to stiffness, however, the rotational velocity is computed from the angular velocity vector; type: array_like

        rotationMarker0: local rotation matrix for marker 0; stiffness, damping, etc. components are measured in local coordinates relative to rotationMarker0; type: array_like; deprecated since 1.12.244, removed in 2031: give the rotation to marker 0 as its localHT, e.g. MarkerBodyRigid(bodyNumber=b, localHT=exu.HT(rotation=A, translation=p))

        rotationMarker1: local rotation matrix for marker 1; stiffness, damping, etc. components are measured in local coordinates relative to rotationMarker1; type: array_like; deprecated since 1.12.244, removed in 2031: give the rotation to marker 1 as its localHT, e.g. MarkerBodyRigid(bodyNumber=b, localHT=exu.HT(rotation=A, translation=p))

        offset: translational and rotational offset considered in the spring force calculation; type: array_like

        useIntrinsicFormulation: if True, the joint uses the intrinsic formulation, which is independent on order of markers, using a mid-point and mid-rotation for evaluation and application of connector forces and torques; this uses a Lie group formulation; in this case, the force/torque vector is computed from the stiffness matrix times the 6-vector of the SE3 matrix logarithm between the two marker positions/rotations, see the equations; type: bool

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        springForceTorqueUserFunction: A Python function which computes the 6D force-torque vector (3D force + 3D torque) between the two rigid body markers, if activeConnector=True; see description below; type: ObjectConnectorRigidBodySpringDamperSpringForceTorqueUserFunction

        postNewtonStepUserFunction: A Python function which computes the error of the PostNewtonStep; see description below; type: ObjectConnectorRigidBodySpringDamperPostNewtonStepUserFunction

        intrinsicFormulation: deprecated since 1.12.258, removed in 2031: use useIntrinsicFormulation

        visualization: visualization data, see VObjectConnectorRigidBodySpringDamper

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position`` + ``Orientation``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), stiffness = IIDiagMatrix(rowsColumns=6,value=0.), damping = IIDiagMatrix(rowsColumns=6,value=0.), rotationMarker0 = IIDiagMatrix(rowsColumns=3,value=1), rotationMarker1 = IIDiagMatrix(rowsColumns=3,value=1), offset = [0.,0.,0.,0.,0.,0.], useIntrinsicFormulation = False, activeConnector = True, springForceTorqueUserFunction: Union[ObjectConnectorRigidBodySpringDamperSpringForceTorqueUserFunction, int] = 0, postNewtonStepUserFunction: Union[ObjectConnectorRigidBodySpringDamperPostNewtonStepUserFunction, int] = 0, intrinsicFormulation = None, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.stiffness = np.array(stiffness)
        self.damping = np.array(damping)
        self.rotationMarker0 = np.array(rotationMarker0)
        self.rotationMarker1 = np.array(rotationMarker1)
        self.offset = np.array(offset)
        self.useIntrinsicFormulation = useIntrinsicFormulation
        self.activeConnector = activeConnector
        self.springForceTorqueUserFunction = springForceTorqueUserFunction
        self.postNewtonStepUserFunction = postNewtonStepUserFunction
        self.intrinsicFormulation = intrinsicFormulation
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ConnectorRigidBodySpringDamper'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'stiffness', self.stiffness
        yield 'damping', self.damping
        yield 'rotationMarker0', self.rotationMarker0
        yield 'rotationMarker1', self.rotationMarker1
        yield 'offset', self.offset
        yield 'useIntrinsicFormulation', self.useIntrinsicFormulation
        yield 'activeConnector', self.activeConnector
        yield 'springForceTorqueUserFunction', self.springForceTorqueUserFunction
        yield 'postNewtonStepUserFunction', self.postNewtonStepUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]
        if self.intrinsicFormulation is not None:
            yield 'intrinsicFormulation', self.intrinsicFormulation

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
RigidBodySpringDamper = ObjectConnectorRigidBodySpringDamper
VRigidBodySpringDamper = VObjectConnectorRigidBodySpringDamper

class VObjectConnectorLinearSpringDamper:
    """Visualization data for ObjectConnectorLinearSpringDamper.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size = diameter of spring; size == -1.f means that default connector size is used; type: float

        drawAsCylinder: if this flag is True, the spring-damper is represented as cylinder; this may fit better if the spring-damper represents an actuator; type: bool

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., drawAsCylinder = False, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.drawAsCylinder = drawAsCylinder
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'drawAsCylinder', self.drawAsCylinder
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectConnectorLinearSpringDamper:
    """An linear spring-damper element acting on relative translations along given axis of local joint0 coordinate system.
    
    It connects to position and orientation-based markers; the linear spring-damper is intended to act within prismatic joints or in situations where only one translational axis is free; if the two markers rotate relative to each other, the spring-damper will always act in the local joint0 coordinate system.
    
    Args:
        name: connector's unique name; type: str

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        stiffness: torsional stiffness [SI:Nm/rad] against relative rotation; type: float

        damping: torsional damping [SI:Nm/(rad/s)]; type: float

        axisMarker0: local axis of spring-damper in marker 0 coordinates; this axis will co-move with marker :math:`m0`; if marker m0 is attached to ground, the spring-damper represents linear equations; type: [float,float,float]

        offset: translational offset considered in the spring force calculation (this can be used as position control input!); type: float

        velocityOffset: velocity offset considered in the damper force calculation (this can be used as velocity control input!); type: float

        force: additional constant force [SI:Nm] added to spring-damper; this can be used to prescribe a force between the two attached bodies (e.g., for actuation and control); type: float

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        springForceUserFunction: A Python function which computes the scalar force between the two rigid body markers along axisMarker0 in :math:`m0` coordinates, if activeConnector=True; see description below; type: ObjectConnectorLinearSpringDamperSpringForceUserFunction

        visualization: visualization data, see VObjectConnectorLinearSpringDamper

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position`` + ``Orientation``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], stiffness = 0., damping = 0., axisMarker0 = [1,0,0], offset = 0., velocityOffset = 0., force = 0., activeConnector = True, springForceUserFunction: Union[ObjectConnectorLinearSpringDamperSpringForceUserFunction, int] = 0, visualization = {'show': True, 'drawSize': -1., 'drawAsCylinder': False, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.stiffness = stiffness
        self.damping = damping
        self.axisMarker0 = np.array(axisMarker0)
        self.offset = offset
        self.velocityOffset = velocityOffset
        self.force = force
        self.activeConnector = activeConnector
        self.springForceUserFunction = springForceUserFunction
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ConnectorLinearSpringDamper'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'stiffness', self.stiffness
        yield 'damping', self.damping
        yield 'axisMarker0', self.axisMarker0
        yield 'offset', self.offset
        yield 'velocityOffset', self.velocityOffset
        yield 'force', self.force
        yield 'activeConnector', self.activeConnector
        yield 'springForceUserFunction', self.springForceUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'VdrawAsCylinder', dict(self.visualization)["drawAsCylinder"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
LinearSpringDamper = ObjectConnectorLinearSpringDamper
VLinearSpringDamper = VObjectConnectorLinearSpringDamper

class VObjectConnectorTorsionalSpringDamper:
    """Visualization data for ObjectConnectorTorsionalSpringDamper.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size = diameter of spring; size == -1.f means that default connector size is used; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectConnectorTorsionalSpringDamper:
    r"""An torsional spring-damper element acting on relative rotations around Z-axis of local joint0 coordinate system.
    
    It connects to orientation-based markers; if other rotation axis than the local joint0 Z axis shall be used, the joint rotationMarker0 / rotationMarker1 may be used. The joint perfectly extends a RevoluteJoint with a spring-damper, which can also be used to represent feedback control in an elegant and efficient way, by chosing appropriate user functions. It also allows to measure continuous / infinite rotations by making use of a NodeGeneric which compensates :math:`\pm \pi` jumps in the measured rotation (``OutputVariableType.Rotation``).
    
    Args:
        name: connector's unique name; type: str

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        nodeNumber: node number of a NodeGenericData with 1 dataCoordinate for continuous rotation reconstruction; if this node is left to invalid index, it will not be used; type: NodeIndex

        stiffness: torsional stiffness [SI:Nm/rad] against relative rotation; type: float

        damping: torsional damping [SI:Nm/(rad/s)]; type: float

        rotationMarker0: local rotation matrix for marker 0; transforms joint into marker coordinates; type: array_like; deprecated since 1.12.244, removed in 2031: give the rotation to marker 0 as its localHT, e.g. MarkerBodyRigid(bodyNumber=b, localHT=exu.HT(rotation=A, translation=p))

        rotationMarker1: local rotation matrix for marker 1; transforms joint into marker coordinates; type: array_like; deprecated since 1.12.244, removed in 2031: give the rotation to marker 1 as its localHT, e.g. MarkerBodyRigid(bodyNumber=b, localHT=exu.HT(rotation=A, translation=p))

        offset: rotational offset considered in the spring torque calculation (this can be used as rotation control input!); type: float

        velocityOffset: angular velocity offset considered in the damper torque calculation (this can be used as angular velocity control input!); type: float

        torque: additional constant torque [SI:Nm] added to spring-damper; this can be used to prescribe a torque between the two attached bodies (e.g., for actuation and control); type: float

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        springTorqueUserFunction: A Python function which computes the scalar torque between the two rigid body markers in local joint0 coordinates, if activeConnector=True; see description below; type: ObjectConnectorTorsionalSpringDamperSpringTorqueUserFunction

        visualization: visualization data, see VObjectConnectorTorsionalSpringDamper

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Orientation``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), stiffness = 0., damping = 0., rotationMarker0 = IIDiagMatrix(rowsColumns=3,value=1), rotationMarker1 = IIDiagMatrix(rowsColumns=3,value=1), offset = 0., velocityOffset = 0., torque = 0., activeConnector = True, springTorqueUserFunction: Union[ObjectConnectorTorsionalSpringDamperSpringTorqueUserFunction, int] = 0, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.stiffness = stiffness
        self.damping = damping
        self.rotationMarker0 = np.array(rotationMarker0)
        self.rotationMarker1 = np.array(rotationMarker1)
        self.offset = offset
        self.velocityOffset = velocityOffset
        self.torque = torque
        self.activeConnector = activeConnector
        self.springTorqueUserFunction = springTorqueUserFunction
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ConnectorTorsionalSpringDamper'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'stiffness', self.stiffness
        yield 'damping', self.damping
        yield 'rotationMarker0', self.rotationMarker0
        yield 'rotationMarker1', self.rotationMarker1
        yield 'offset', self.offset
        yield 'velocityOffset', self.velocityOffset
        yield 'torque', self.torque
        yield 'activeConnector', self.activeConnector
        yield 'springTorqueUserFunction', self.springTorqueUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
TorsionalSpringDamper = ObjectConnectorTorsionalSpringDamper
VTorsionalSpringDamper = VObjectConnectorTorsionalSpringDamper

class VObjectConnectorCoordinateSpringDamper:
    """Visualization data for ObjectConnectorCoordinateSpringDamper.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size = diameter of spring; size == -1.f means that default connector size is used; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectConnectorCoordinateSpringDamper:
    """A 1D (scalar) spring-damper element acting on single ODE2 coordinates and connecting to coordinate-based markers.
    
    NOTE that the coordinate markers only measure the coordinate (=displacement), but the reference position is not included as compared to position-based markers!; the spring-damper can also act on rotational coordinates.
    
    Args:
        name: connector's unique name; type: str

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        stiffness: stiffness [SI:N/m] of spring; acts against relative value of coordinates; type: float

        damping: damping [SI:N/(m s)] of damper; acts against relative velocity of coordinates; type: float

        offset: offset between two coordinates (reference length of springs), see equation; type: float

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        springForceUserFunction: A Python function which defines the spring force with 8 parameters, see equations section / see description below; type: ObjectConnectorCoordinateSpringDamperSpringForceUserFunction

        visualization: visualization data, see VObjectConnectorCoordinateSpringDamper

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Coordinate``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], stiffness = 0., damping = 0., offset = 0., activeConnector = True, springForceUserFunction: Union[ObjectConnectorCoordinateSpringDamperSpringForceUserFunction, int] = 0, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.stiffness = stiffness
        self.damping = damping
        self.offset = offset
        self.activeConnector = activeConnector
        self.springForceUserFunction = springForceUserFunction
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ConnectorCoordinateSpringDamper'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'stiffness', self.stiffness
        yield 'damping', self.damping
        yield 'offset', self.offset
        yield 'activeConnector', self.activeConnector
        yield 'springForceUserFunction', self.springForceUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
CoordinateSpringDamper = ObjectConnectorCoordinateSpringDamper
VCoordinateSpringDamper = VObjectConnectorCoordinateSpringDamper

class VObjectConnectorCoordinateSpringDamperExt:
    """Visualization data for ObjectConnectorCoordinateSpringDamperExt.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size = diameter of spring; size == -1.f means that default connector size is used; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectConnectorCoordinateSpringDamperExt:
    r"""A 1D (scalar) spring-damper element acting on single ODE2 coordinates, same as ObjectConnectorCoordinateSpringDamper but with extended features, such as limit stop and improved friction.
    
    It has different user function interface and additional data node as compared to ObjectConnectorCoordinateSpringDamper, but otherwise behaves very similar. The CoordinateSpringDamperExt is very useful for a single axis of a robot or similar machine modelled with a KinematicTree, as it can add friction and limits based on physical properties. It is highly recommended, to use the bristle model for friction with frictionProportionalZone=0 in case of implicit integrators (GeneralizedAlpha) as it converges better.
    
    Args:
        name: connector's unique name; type: str

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        nodeNumber: node number of a NodeGenericData for 3 data coordinates (friction mode, last sticking position, limit stop state), see description for details; must exist in case of bristle friction model or limit stops; type: NodeIndex

        stiffness: stiffness [SI:N/m] of spring; acts against relative value of coordinates; type: float

        damping: damping [SI:N/(m s)] of damper; acts against relative velocity of coordinates; type: float

        offset: offset between two coordinates (reference length of springs), see equation; it can be used to represent the pre-scribed drive coordinate; type: float

        velocityOffset: velocity offset of the damper force, see equation; also passed to springForceUserFunction; type: float

        factor0: marker 0 coordinate is multiplied with factor0; type: float

        factor1: marker 1 coordinate is multiplied with factor1; type: float

        dynamicFrictionForce: dynamic (viscous) friction force [SI:N] against relative velocity when sliding; assuming a normal force :math:`f_N`, the friction force can be interpreted as :math:`f_\mu = \mu f_N`; type: float

        staticFrictionOffsetForce: static (dry) friction offset force [SI:N]; assuming a normal force :math:`f_N`, the friction force is limited by :math:`f_\mu \le (\mu_{so} + \mu_d) f_N = f_{\mu_d} + f_{\mu_{so}}`; type: float

        stickingStiffness: stiffness of bristles in sticking case  [SI:N/m]; type: float

        stickingDamping: damping of bristles in sticking case  [SI:N/(m/s)]; type: float

        exponentialDecayStatic: relative velocity for exponential decay of static friction offset force [SI:m/s] against relative velocity; at :math:`\Delta v = v_\mathrm{exp}`, the static friction offset force is reduced to 36.8%; type: float

        viscousFrictionFactor: viscous friction factor [SI:N s/m]: the friction force part proportional to the relative velocity, acting against it in the sliding case; type: float

        frictionProportionalZone: if non-zero, a regularized Stribeck model is used, regularizing friction force around zero velocity - leading to zero friction force in case of zero velocity; this does not require a data node at all; if zero, the bristle model is used, which requires a data node which contains previous friction state and last sticking position; type: float

        limitStopsUpper: upper (maximum) value [SI:m] of coordinate before limit is activated; defined relative to the two marker coordinates; type: float

        limitStopsLower: lower (minimum) value [SI:m] of coordinate before limit is activated; defined relative to the two marker coordinates; type: float

        limitStopsStiffness: stiffness [SI:N/m] of limit stop (contact stiffness); following a linear contact model; type: float

        limitStopsDamping: damping [SI:N/(m/s)] of limit stop (contact damping); following a linear contact model; type: float

        useLimitStops: if True, limit stops are considered and parameters must be set accordingly; furthermore, the NodeGenericData must have 3 data coordinates; type: bool

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        springForceUserFunction: A Python function which defines the spring force with 8 parameters, see equations section / see description below; type: ObjectConnectorCoordinateSpringDamperExtSpringForceUserFunction

        fDynamicFriction: deprecated since 1.12.258, removed in 2031: use dynamicFrictionForce

        fStaticFrictionOffset: deprecated since 1.12.258, removed in 2031: use staticFrictionOffsetForce

        fViscousFriction: deprecated since 1.12.258, removed in 2031: use viscousFrictionFactor

        visualization: visualization data, see VObjectConnectorCoordinateSpringDamperExt

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Coordinate``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), stiffness = 0., damping = 0., offset = 0., velocityOffset = 0., factor0 = 1., factor1 = 1., dynamicFrictionForce = 0., staticFrictionOffsetForce = 0., stickingStiffness = 0., stickingDamping = 0., exponentialDecayStatic = 0.001, viscousFrictionFactor = 0., frictionProportionalZone = 0., limitStopsUpper = 0., limitStopsLower = 0., limitStopsStiffness = 0., limitStopsDamping = 0., useLimitStops = False, activeConnector = True, springForceUserFunction: Union[ObjectConnectorCoordinateSpringDamperExtSpringForceUserFunction, int] = 0, fDynamicFriction = None, fStaticFrictionOffset = None, fViscousFriction = None, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.stiffness = stiffness
        self.damping = damping
        self.offset = offset
        self.velocityOffset = velocityOffset
        self.factor0 = factor0
        self.factor1 = factor1
        self.dynamicFrictionForce = dynamicFrictionForce
        self.staticFrictionOffsetForce = staticFrictionOffsetForce
        self.stickingStiffness = stickingStiffness
        self.stickingDamping = stickingDamping
        self.exponentialDecayStatic = exponentialDecayStatic
        self.viscousFrictionFactor = viscousFrictionFactor
        self.frictionProportionalZone = frictionProportionalZone
        self.limitStopsUpper = limitStopsUpper
        self.limitStopsLower = limitStopsLower
        self.limitStopsStiffness = limitStopsStiffness
        self.limitStopsDamping = limitStopsDamping
        self.useLimitStops = useLimitStops
        self.activeConnector = activeConnector
        self.springForceUserFunction = springForceUserFunction
        self.fDynamicFriction = fDynamicFriction
        self.fStaticFrictionOffset = fStaticFrictionOffset
        self.fViscousFriction = fViscousFriction
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ConnectorCoordinateSpringDamperExt'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'stiffness', self.stiffness
        yield 'damping', self.damping
        yield 'offset', self.offset
        yield 'velocityOffset', self.velocityOffset
        yield 'factor0', self.factor0
        yield 'factor1', self.factor1
        yield 'dynamicFrictionForce', self.dynamicFrictionForce
        yield 'staticFrictionOffsetForce', self.staticFrictionOffsetForce
        yield 'stickingStiffness', self.stickingStiffness
        yield 'stickingDamping', self.stickingDamping
        yield 'exponentialDecayStatic', self.exponentialDecayStatic
        yield 'viscousFrictionFactor', self.viscousFrictionFactor
        yield 'frictionProportionalZone', self.frictionProportionalZone
        yield 'limitStopsUpper', self.limitStopsUpper
        yield 'limitStopsLower', self.limitStopsLower
        yield 'limitStopsStiffness', self.limitStopsStiffness
        yield 'limitStopsDamping', self.limitStopsDamping
        yield 'useLimitStops', self.useLimitStops
        yield 'activeConnector', self.activeConnector
        yield 'springForceUserFunction', self.springForceUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]
        if self.fDynamicFriction is not None:
            yield 'fDynamicFriction', self.fDynamicFriction
        if self.fStaticFrictionOffset is not None:
            yield 'fStaticFrictionOffset', self.fStaticFrictionOffset
        if self.fViscousFriction is not None:
            yield 'fViscousFriction', self.fViscousFriction

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
CoordinateSpringDamperExt = ObjectConnectorCoordinateSpringDamperExt
VCoordinateSpringDamperExt = VObjectConnectorCoordinateSpringDamperExt

class VObjectConnectorGravity:
    """Visualization data for ObjectConnectorGravity.
    
    Args:
        show: set true to draw a line between the two markers, e.g. to see which bodies attract each other; type: bool

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = False, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectConnectorGravity:
    """A connector for additing forces due to gravitational fields beween two bodies, which can be used for aerospace and small-scale astronomical problems.
    
    NOTE: DO NOT USE this connector for adding gravitational forces (loads), which should be using LoadMassProportional, which is acting global and always in the same direction.
    
    Args:
        name: connector's unique name; type: str

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        gravitationalConstant: gravitational constant [SI:m:math:`^3`kg:math:`^{-1}`s:math:`^{-2}`)]; while not recommended, a negative constant gan represent a repulsive force; type: float

        mass0: mass [SI:kg] of object attached to marker :math:`m0`; type: float

        mass1: mass [SI:kg] of object attached to marker :math:`m1`; type: float

        minDistanceRegularization: distance [SI:m] at which a regularization is added in order to avoid singularities, if objects come close; type: float

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectConnectorGravity

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], gravitationalConstant = 6.6743e-11, mass0 = 0., mass1 = 0., minDistanceRegularization = 0., activeConnector = True, visualization = {'show': False, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.gravitationalConstant = gravitationalConstant
        self.mass0 = mass0
        self.mass1 = mass1
        self.minDistanceRegularization = minDistanceRegularization
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ConnectorGravity'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'gravitationalConstant', self.gravitationalConstant
        yield 'mass0', self.mass0
        yield 'mass1', self.mass1
        yield 'minDistanceRegularization', self.minDistanceRegularization
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
ConnectorGravity = ObjectConnectorGravity
VConnectorGravity = VObjectConnectorGravity

class VObjectConnectorHydraulicActuatorSimple:
    """Visualization data for ObjectConnectorHydraulicActuatorSimple.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        cylinderRadius: radius for drawing of cylinder; type: float

        rodRadius: radius for drawing of rod; type: float

        pistonRadius: radius for drawing of piston (if drawn transparent); type: float

        pistonLength: radius for drawing of piston (if drawn transparent); type: float

        rodMountRadius: radius for drawing of rod mount sphere; type: float

        baseMountRadius: radius for drawing of base mount sphere; type: float

        baseMountLength: radius for drawing of base mount sphere; type: float

        colorCylinder: RGBA cylinder color; if R==-1, use default connector color; type: [float,float,float,float]

        colorPiston: RGBA piston color; type: [float,float,float,float]

    """
    def __init__(self, show = True, cylinderRadius = 0.05, rodRadius = 0.03, pistonRadius = 0.04, pistonLength = 0.001, rodMountRadius = 0., baseMountRadius = 0., baseMountLength = 0., colorCylinder = [-1.,-1.,-1.,-1.], colorPiston = [0.8,0.8,0.8,1.]):
        self.show = show
        self.cylinderRadius = cylinderRadius
        self.rodRadius = rodRadius
        self.pistonRadius = pistonRadius
        self.pistonLength = pistonLength
        self.rodMountRadius = rodMountRadius
        self.baseMountRadius = baseMountRadius
        self.baseMountLength = baseMountLength
        self.colorCylinder = np.array(colorCylinder)
        self.colorPiston = np.array(colorPiston)

    def __iter__(self):
        yield 'show', self.show
        yield 'cylinderRadius', self.cylinderRadius
        yield 'rodRadius', self.rodRadius
        yield 'pistonRadius', self.pistonRadius
        yield 'pistonLength', self.pistonLength
        yield 'rodMountRadius', self.rodMountRadius
        yield 'baseMountRadius', self.baseMountRadius
        yield 'baseMountLength', self.baseMountLength
        yield 'colorCylinder', self.colorCylinder
        yield 'colorPiston', self.colorPiston

    def __repr__(self):
        return str(dict(self))

class ObjectConnectorHydraulicActuatorSimple:
    r"""A basic hydraulic actuator with pressure build up equations.
    
    The actuator follows a valve input value, which results in a in- or outflow of fluid depending on the pressure difference. Valve values can be prescribed by user functions (not yet available) or with the ``MainSystem`` ``PreStepUserFunction(...)``.
    
    Args:
        name: connector's unique name; type: str

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        nodeNumbers: currently a list with one node number of NodeGenericODE1 for 2 hydraulic pressures (reference values for this node must be zero); data node may be added in future for switching; type: ArrayNodeIndex

        offsetLength: offset length [SI:m] of cylinder, representing minimal distance between the two bushings at stroke=0; type: float

        strokeLength: stroke length [SI:m] of cylinder, representing maximum extension relative to :math:`L_o`; the measured distance between the markers is :math:`L_s+L_o`; type: float

        chamberCrossSection0: cross section [SI:m:math:`^2`] of chamber (inner cylinder) at piston head (nut) side (0); type: float

        chamberCrossSection1: cross section [SI:m:math:`^2`] of chamber at piston rod side (1); usually smaller than chamberCrossSection0; type: float

        hoseVolume0: hose volume [SI:m:math:`^3`] at piston head (nut) side (0); as the effective bulk modulus would go to infinity at stroke length zero, the hose volume must be greater than zero; type: float

        hoseVolume1: hose volume [SI:m:math:`^3`] at piston rod side (1); as the effective bulk modulus would go to infinity at max. stroke length, the hose volume must be greater than zero; type: float

        valveOpening0: relative opening of valve :math:`[-1 \ldots 1]` [SI:1] at piston head (nut) side (0); positive value is valve opening towards system pressure, negative value is valve opening towards tank pressure; zero means closed valve; type: float

        valveOpening1: relative opening of valve :math:`[-1 \ldots 1]` [SI:1] at piston rod side (1); positive value is valve opening towards system pressure, negative value is valve opening towards tank pressure; zero means closed valve; type: float

        actuatorDamping: damping [SI:N/(m:math:`\,`s)] of hydraulic actuator (against actuator axial velocity); type: float

        oilBulkModulus: bulk modulus of oil [SI:N/(m:math:`^2`)]; type: float

        cylinderBulkModulus: bulk modulus of cylinder [SI:N/(m:math:`^2`)]; in fact, this is value represents the effect of the cylinder stiffness on the effective bulk modulus; type: float

        hoseBulkModulus: bulk modulus of hose [SI:N/(m:math:`^2`)]; in fact, this is value represents the effect of the hose stiffness on the effective bulk modulus; type: float

        nominalFlow: nominal flow of oil through valve [SI:m:math:`^3`/s]; type: float

        systemPressure: system pressure [SI:N/(m:math:`^2`)]; type: float

        tankPressure: tank pressure [SI:N/(m:math:`^2`)]; type: float

        useChamberVolumeChange: if True, the pressure build up equations include the change of oil stiffness due to change of chamber volume; type: bool

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectConnectorHydraulicActuatorSimple

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumbers = [], offsetLength = 0., strokeLength = 0., chamberCrossSection0 = 0., chamberCrossSection1 = 0., hoseVolume0 = 0., hoseVolume1 = 0., valveOpening0 = 0., valveOpening1 = 0., actuatorDamping = 0., oilBulkModulus = 0., cylinderBulkModulus = 0., hoseBulkModulus = 0., nominalFlow = 0., systemPressure = 0., tankPressure = 0., useChamberVolumeChange = False, activeConnector = True, visualization = {'show': True, 'cylinderRadius': 0.05, 'rodRadius': 0.03, 'pistonRadius': 0.04, 'pistonLength': 0.001, 'rodMountRadius': 0., 'baseMountRadius': 0., 'baseMountLength': 0., 'colorCylinder': [-1.,-1.,-1.,-1.], 'colorPiston': [0.8,0.8,0.8,1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.offsetLength = offsetLength
        self.strokeLength = strokeLength
        self.chamberCrossSection0 = chamberCrossSection0
        self.chamberCrossSection1 = chamberCrossSection1
        self.hoseVolume0 = hoseVolume0
        self.hoseVolume1 = hoseVolume1
        self.valveOpening0 = valveOpening0
        self.valveOpening1 = valveOpening1
        self.actuatorDamping = actuatorDamping
        self.oilBulkModulus = oilBulkModulus
        self.cylinderBulkModulus = cylinderBulkModulus
        self.hoseBulkModulus = hoseBulkModulus
        self.nominalFlow = nominalFlow
        self.systemPressure = systemPressure
        self.tankPressure = tankPressure
        self.useChamberVolumeChange = useChamberVolumeChange
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ConnectorHydraulicActuatorSimple'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumbers', self.nodeNumbers
        yield 'offsetLength', self.offsetLength
        yield 'strokeLength', self.strokeLength
        yield 'chamberCrossSection0', self.chamberCrossSection0
        yield 'chamberCrossSection1', self.chamberCrossSection1
        yield 'hoseVolume0', self.hoseVolume0
        yield 'hoseVolume1', self.hoseVolume1
        yield 'valveOpening0', self.valveOpening0
        yield 'valveOpening1', self.valveOpening1
        yield 'actuatorDamping', self.actuatorDamping
        yield 'oilBulkModulus', self.oilBulkModulus
        yield 'cylinderBulkModulus', self.cylinderBulkModulus
        yield 'hoseBulkModulus', self.hoseBulkModulus
        yield 'nominalFlow', self.nominalFlow
        yield 'systemPressure', self.systemPressure
        yield 'tankPressure', self.tankPressure
        yield 'useChamberVolumeChange', self.useChamberVolumeChange
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VcylinderRadius', dict(self.visualization)["cylinderRadius"]
        yield 'VrodRadius', dict(self.visualization)["rodRadius"]
        yield 'VpistonRadius', dict(self.visualization)["pistonRadius"]
        yield 'VpistonLength', dict(self.visualization)["pistonLength"]
        yield 'VrodMountRadius', dict(self.visualization)["rodMountRadius"]
        yield 'VbaseMountRadius', dict(self.visualization)["baseMountRadius"]
        yield 'VbaseMountLength', dict(self.visualization)["baseMountLength"]
        yield 'VcolorCylinder', dict(self.visualization)["colorCylinder"]
        yield 'VcolorPiston', dict(self.visualization)["colorPiston"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
HydraulicActuatorSimple = ObjectConnectorHydraulicActuatorSimple
VHydraulicActuatorSimple = VObjectConnectorHydraulicActuatorSimple

class VObjectConnectorReevingSystemSprings:
    """Visualization data for ObjectConnectorReevingSystemSprings.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        ropeRadius: radius of rope, drawn as one tube along the free spans and the arcs on the sheaves; visualizationSettings.general.cylinderTiling segments around it, connectors.curveTiling segments per full turn of an arc; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, ropeRadius = 0.001, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.ropeRadius = ropeRadius
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'ropeRadius', self.ropeRadius
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectConnectorReevingSystemSprings:
    r"""A rD reeving system defined by a list of torque-free and friction-free sheaves or points that are connected with one rope (modelled as massless spring).
    
    NOTE that the spring can undergo tension AND compression (in order to avoid compression, use a PreStepUserFunction to turn off stiffness and damping in this case!). The force is assumed to be constant all over the rope. The sheaves or connection points are defined by :math:`nr` rigid body markers :math:`[m_0, \, m_1, \, \ldots, \, m_{nr-1}]`. At both ends of the rope there may be a prescribed motion coupled to a coordinate marker each, given by :math:`m_{c0}` and :math:`m_{c1}` .
    
    Args:
        name: connector's unique name; type: str

        markerNumbers: list of position or rigid body markers used in reeving system and optional two coordinate markers (:math:`m_{c0}, \, m_{c1}`); the first marker :math:`m_0` and the last rigid body marker :math:`m_{nr-1}` represent the ends of the rope and are directly connected to a position; the markers :math:`m_1, \, \ldots, \, m_{nr-2}` can be connected to sheaves, for which a radius and an axis can be prescribed. The coordinate markers are optional and represent prescribed length at the rope ends (marker :math:`m_{c0}` is added length at start, marker :math:`m_{c1}` is added length at end of the rope in the reeving system); type: ArrayMarkerIndex

        hasCoordinateMarkers: flag, which determines, the list of markers (markerNumbers) contains two coordinate markers at the end of the list, representing the prescribed change of length at both ends; type: bool

        coordinateFactors: factors which are multiplied with the values of coordinate markers; this can be used, e.g., to change directions or to transform rotations (revolutions of a sheave) into change of length; type: [float,float]

        stiffnessPerLength: stiffness per length [SI:N/m/m] of rope; in case of cross section :math:`A` and Young's modulus :math:`E`, this parameter results in :math:`E\cdot A`; the effective stiffness of the reeving system is computed as :math:`EA/L` in which :math:`L` is the current length of the rope; type: float

        dampingPerLength: axial damping per length [SI:N/(m/s)/m] of rope; the effective damping coefficient of the reeving system is computed as :math:`DA/L` in which :math:`L` is the current length of the rope; type: float

        dampingTorsional: torsional damping [SI:Nms] between sheaves; this effect can damp rotations around the rope axis, pairwise between sheaves; this parameter is experimental; type: float

        dampingShear: damping of shear motion [SI:Ns] between sheaves; this effect can damp motion perpendicular to the rope between each pair of sheaves; this parameter is experimental; type: float

        regularizationForce: small regularization force [SI:N] in order to avoid large compressive forces; this regularization force can either be :math:`<0` (using a linear tension/compression spring model) or :math:`>0`, which restricts forces in the rope to be always :math:`\ge -F_{reg}`. Note that smaller forces lead to problems in implicit integrators and smaller time steps. For explicit integrators, this force can be chosen close to zero.; type: float

        referenceLength: reference length for computation of roped force; type: float

        sheavesAxes: list of local vectors axes of sheaves; vectors refer to rigid body markers given in list of markerNumbers; first and last axes are ignored, as they represent the attachment of the rope ends; type: Vector3DList

        sheavesRadii: radius for each sheave, related to list of markerNumbers and list of sheaveAxes; first and last radii must always be zero.; type: array_like

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectConnectorReevingSystemSprings

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``_None``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], hasCoordinateMarkers = False, coordinateFactors = [1,1], stiffnessPerLength = 0., dampingPerLength = 0., dampingTorsional = 0., dampingShear = 0., regularizationForce = 0.1, referenceLength = 0., sheavesAxes = None, sheavesRadii = [], activeConnector = True, visualization = {'show': True, 'ropeRadius': 0.001, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.hasCoordinateMarkers = hasCoordinateMarkers
        self.coordinateFactors = np.array(coordinateFactors)
        self.stiffnessPerLength = stiffnessPerLength
        self.dampingPerLength = dampingPerLength
        self.dampingTorsional = dampingTorsional
        self.dampingShear = dampingShear
        self.regularizationForce = regularizationForce
        self.referenceLength = referenceLength
        self.sheavesAxes = sheavesAxes
        self.sheavesRadii = np.array(sheavesRadii)
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ConnectorReevingSystemSprings'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'hasCoordinateMarkers', self.hasCoordinateMarkers
        yield 'coordinateFactors', self.coordinateFactors
        yield 'stiffnessPerLength', self.stiffnessPerLength
        yield 'dampingPerLength', self.dampingPerLength
        yield 'dampingTorsional', self.dampingTorsional
        yield 'dampingShear', self.dampingShear
        yield 'regularizationForce', self.regularizationForce
        yield 'referenceLength', self.referenceLength
        yield 'sheavesAxes', self.sheavesAxes
        yield 'sheavesRadii', self.sheavesRadii
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VropeRadius', dict(self.visualization)["ropeRadius"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
ReevingSystemSprings = ObjectConnectorReevingSystemSprings
VReevingSystemSprings = VObjectConnectorReevingSystemSprings

class VObjectConnectorDistance:
    """Visualization data for ObjectConnectorDistance.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: the diameter of the rod drawn between the markers if visualizationSettings.connectors.drawSimplified is False; -1 means a tenth of connectors.defaultSize; with drawSimplified, the connector is a line; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectConnectorDistance:
    """Connector which enforces constant or prescribed distance between two bodies/nodes.
    
    Args:
        name: constraints's unique name

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        distance: prescribed distance [SI:m] of the used markers; must by greater than zero; type: float

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectConnectorDistance

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``Position``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], distance = 0., activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.distance = distance
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ConnectorDistance'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'distance', self.distance
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
DistanceConstraint = ObjectConnectorDistance
VDistanceConstraint = VObjectConnectorDistance

class VObjectConnectorCoordinate:
    """Visualization data for ObjectConnectorCoordinate.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size = link size; size == -1.f means that default connector size is used; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectConnectorCoordinate:
    """A coordinate constraint which constrains two (scalar) coordinates of Marker[Node|Body]Coordinates attached to nodes or bodies.
    
    The constraint acts directly on coordinates, but does not include reference values, e.g., of nodal values. This constraint is computationally efficient and should be used to constrain nodal coordinates.
    
    Args:
        name: constraints's unique name

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        offset: An offset between the two values; type: float

        factor1: An additional factor multiplied with value1 used in algebraic equation; type: float

        velocityLevel: If true: connector constrains velocities (only works for ODE2 coordinates!); offset is used between velocities; in this case, the offsetUserFunction_t is considered and offsetUserFunction is ignored; type: bool

        offsetUserFunction: A Python function which defines the time-dependent offset; see description below; type: ObjectConnectorCoordinateOffsetUserFunction

        offsetUserFunction_t: time derivative of offsetUserFunction; needed for velocity level constraints; see description below; type: ObjectConnectorCoordinateOffsetUserFunction_t

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        factorValue1: deprecated since 1.12.258, removed in 2031: use factor1

        visualization: visualization data, see VObjectConnectorCoordinate

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``Coordinate``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], offset = 0., factor1 = 1., velocityLevel = False, offsetUserFunction: Union[ObjectConnectorCoordinateOffsetUserFunction, int] = 0, offsetUserFunction_t: Union[ObjectConnectorCoordinateOffsetUserFunction_t, int] = 0, activeConnector = True, factorValue1 = None, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.offset = offset
        self.factor1 = factor1
        self.velocityLevel = velocityLevel
        self.offsetUserFunction = offsetUserFunction
        self.offsetUserFunction_t = offsetUserFunction_t
        self.activeConnector = activeConnector
        self.factorValue1 = factorValue1
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ConnectorCoordinate'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'offset', self.offset
        yield 'factor1', self.factor1
        yield 'velocityLevel', self.velocityLevel
        yield 'offsetUserFunction', self.offsetUserFunction
        yield 'offsetUserFunction_t', self.offsetUserFunction_t
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]
        if self.factorValue1 is not None:
            yield 'factorValue1', self.factorValue1

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
CoordinateConstraint = ObjectConnectorCoordinate
VCoordinateConstraint = VObjectConnectorCoordinate

class VObjectConnectorCoordinateVector:
    """Visualization data for ObjectConnectorCoordinateVector."""
    def __init__(self):
        pass

    def __iter__(self):
        yield from ()

    def __repr__(self):
        return str(dict(self))

class ObjectConnectorCoordinateVector:
    """A constraint which constrains the coordinate vectors of two markers Marker[Node|Object|Body]Coordinates attached to nodes or bodies.
    
    The marker uses the objects LTG-lists to build the according coordinate mappings.
    
    Args:
        name: constraints's unique name

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        scalingMarker0: linear scaling matrix for coordinate vector of marker 0; matrix provided in Python numpy format; type: array_like

        scalingMarker1: linear scaling matrix for coordinate vector of marker 1; matrix provided in Python numpy format; type: array_like

        quadraticTermMarker0: quadratic scaling matrix for coordinate vector of marker 0; matrix provided in Python numpy format; type: array_like

        quadraticTermMarker1: quadratic scaling matrix for coordinate vector of marker 1; matrix provided in Python numpy format; type: array_like

        offset: offset added to constraint equation; only active, if no userFunction is defined; type: array_like

        velocityLevel: If true: connector constrains velocities (only works for ODE2 coordinates!); offset is used between velocities; in this case, the offsetUserFunction_t is considered and offsetUserFunction is ignored; type: bool

        constraintUserFunction: A Python user function which computes the constraint equations; to define the number of algebraic equations, set scalingMarker0 as a numpy.zeros((nAE,1)) array with nAE being the number algebraic equations; see description below; type: ObjectConnectorCoordinateVectorConstraintUserFunction

        jacobianUserFunction: A Python user function which computes the jacobian, i.e., the derivative of the left-hand-side object equation w.r.t. the coordinates (times :math:`f_{ODE2}`) and w.r.t. the velocities (times :math:`f_{ODE2_t}`). Terms on the RHS must be subtracted from the LHS equation; the respective terms for the stiffness matrix and damping matrix are automatically added; see description below; type: ObjectConnectorCoordinateVectorJacobianUserFunction

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectConnectorCoordinateVector

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``Coordinate``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], scalingMarker0 = [], scalingMarker1 = [], quadraticTermMarker0 = [], quadraticTermMarker1 = [], offset = [], velocityLevel = False, constraintUserFunction: Union[ObjectConnectorCoordinateVectorConstraintUserFunction, int] = 0, jacobianUserFunction: Union[ObjectConnectorCoordinateVectorJacobianUserFunction, int] = 0, activeConnector = True, visualization = {}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.scalingMarker0 = CheckForValidNumpyArray(scalingMarker0)
        self.scalingMarker1 = CheckForValidNumpyArray(scalingMarker1)
        self.quadraticTermMarker0 = CheckForValidNumpyArray(quadraticTermMarker0)
        self.quadraticTermMarker1 = CheckForValidNumpyArray(quadraticTermMarker1)
        self.offset = CheckForValidNumpyArray(offset)
        self.velocityLevel = velocityLevel
        self.constraintUserFunction = constraintUserFunction
        self.jacobianUserFunction = jacobianUserFunction
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ConnectorCoordinateVector'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'scalingMarker0', self.scalingMarker0
        yield 'scalingMarker1', self.scalingMarker1
        yield 'quadraticTermMarker0', self.quadraticTermMarker0
        yield 'quadraticTermMarker1', self.quadraticTermMarker1
        yield 'offset', self.offset
        yield 'velocityLevel', self.velocityLevel
        yield 'constraintUserFunction', self.constraintUserFunction
        yield 'jacobianUserFunction', self.jacobianUserFunction
        yield 'activeConnector', self.activeConnector

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
CoordinateVectorConstraint = ObjectConnectorCoordinateVector
VCoordinateVectorConstraint = VObjectConnectorCoordinateVector

class VObjectConnectorRollingDiscPenalty:
    """Visualization data for ObjectConnectorRollingDiscPenalty.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        discWidth: width of disc for drawing; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, discWidth = 0.1, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.discWidth = discWidth
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'discWidth', self.discWidth
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectConnectorRollingDiscPenalty:
    r"""A (flexible) connector representing a rolling rigid disc (marker 1) on a flat surface (marker 0, ground body, not moving) in global :math:`x`-:math:`y` plane.
    
    The connector is based on a penalty formulation and adds friction and slipping. The contraints works for discs as long as the disc axis and the plane normal vector are not parallel. Parameters may need to be adjusted for better convergence (e.g., dryFrictionProportionalZone). The formulation for the arbitrary disc axis is still under development and needs further testing. Note that the rolling body must have the reference point at the center of the disc.
    
    Args:
        name: constraints's unique name

        markerNumbers: list of markers used in connector; :math:`m0` represents a point at the plane surface (normal of surface plane defined by planeNormal); the ground can also be a moving rigid body; :math:`m1` represents the rolling body, which has its reference point (=local position [0,0,0]) at the disc center point; type: ArrayMarkerIndex

        nodeNumber: node number of a NodeGenericData (size=3) for 3 dataCoordinates, needed for discontinuous iteration (friction and contact); type: NodeIndex

        discRadius: defines the disc radius; type: float

        discAxis: axis of disc defined in marker :math:`m1` frame; type: [float,float,float]

        planeNormal: normal to the contact / rolling plane (ground); note that the plane reference point can be arbitrarily chosen by the location of the marker :math:`m0`; type: [float,float,float]

        dryFrictionAngle: angle [SI:1 (rad)] which defines a rotation of the local tangential coordinates dry friction; this allows to model Mecanum wheels with specified roll angle; type: float

        contactStiffness: normal contact stiffness [SI:N/m]; type: float

        contactDamping: normal contact damping [SI:N/(m s)]; type: float

        dryFriction: dry friction coefficients [SI:1] in local marker 1 joint :math:`J1` coordinates; if :math:`\alpha_t==0`, lateral direction :math:`l=x` and forward direction :math:`f=y`; assuming a normal force :math:`f_n`, the local friction force can be computed as :math:`{}^{J1}{\vp{f_{t,x}}{f_{t,y}}} = \vp{\mu_x f_n}{\mu_y f_n}`; type: [float,float]

        dryFrictionProportionalZone: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations); type: float

        viscousFriction: viscous friction coefficients [SI:1/(m/s)] in local marker 1 joint :math:`J1` coordinates; proportional to slipping velocity, leading to increasing slipping friction force for increasing slipping velocity; type: [float,float]

        rollingViscousFriction: viscous rolling friction [SI:s/m]: the force acts against the velocity of the trail on ground and is proportional to this velocity and to the contact normal force; currently, only implemented for disc axis parallel to ground!; type: float

        useLinearProportionalZone: if True, a linear proportional zone is used; the linear zone performs better in implicit time integration as the Jacobian has a constant tangent in the sticking case; type: bool

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        rollingFrictionViscous: deprecated since 1.12.258, removed in 2031: use rollingViscousFriction

        visualization: visualization data, see VObjectConnectorRollingDiscPenalty

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position`` + ``Orientation``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), discRadius = 0., discAxis = [1,0,0], planeNormal = [0,0,1], dryFrictionAngle = 0., contactStiffness = 0., contactDamping = 0., dryFriction = [0,0], dryFrictionProportionalZone = 0., viscousFriction = [0,0], rollingViscousFriction = 0., useLinearProportionalZone = False, activeConnector = True, rollingFrictionViscous = None, visualization = {'show': True, 'discWidth': 0.1, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.discRadius = discRadius
        self.discAxis = np.array(discAxis)
        self.planeNormal = np.array(planeNormal)
        self.dryFrictionAngle = dryFrictionAngle
        self.contactStiffness = contactStiffness
        self.contactDamping = contactDamping
        self.dryFriction = np.array(dryFriction)
        self.dryFrictionProportionalZone = dryFrictionProportionalZone
        self.viscousFriction = np.array(viscousFriction)
        self.rollingViscousFriction = rollingViscousFriction
        self.useLinearProportionalZone = useLinearProportionalZone
        self.activeConnector = activeConnector
        self.rollingFrictionViscous = rollingFrictionViscous
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ConnectorRollingDiscPenalty'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'discRadius', self.discRadius
        yield 'discAxis', self.discAxis
        yield 'planeNormal', self.planeNormal
        yield 'dryFrictionAngle', self.dryFrictionAngle
        yield 'contactStiffness', self.contactStiffness
        yield 'contactDamping', self.contactDamping
        yield 'dryFriction', self.dryFriction
        yield 'dryFrictionProportionalZone', self.dryFrictionProportionalZone
        yield 'viscousFriction', self.viscousFriction
        yield 'rollingViscousFriction', self.rollingViscousFriction
        yield 'useLinearProportionalZone', self.useLinearProportionalZone
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdiscWidth', dict(self.visualization)["discWidth"]
        yield 'Vcolor', dict(self.visualization)["color"]
        if self.rollingFrictionViscous is not None:
            yield 'rollingFrictionViscous', self.rollingFrictionViscous

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
RollingDiscPenalty = ObjectConnectorRollingDiscPenalty
VRollingDiscPenalty = VObjectConnectorRollingDiscPenalty

class VObjectContactConvexRoll:
    """Visualization data for ObjectContactConvexRoll.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectContactConvexRoll:
    r"""A contact connector representing a convex roll (marker 1) on a flat surface (marker 0, ground body, not moving) in global :math:`x`-:math:`y` plane.
    
    The connector is similar to ObjectConnectorRollingDiscPenalty, but includes a (strictly) convex shape of the roll defined by a polynomial. It is based on a penalty formulation and adds friction and slipping. The formulation is still under development and needs further testing. Note that the rolling body must have the reference point at the center of the disc.
    
    Args:
        name: constraints's unique name

        markerNumbers: list of markers used in connector; :math:`m0` represents the ground, which can undergo translations but not rotations, and :math:`m1` represents the rolling body, which has its reference point (=local position [0,0,0]) at the roll's center point; type: ArrayMarkerIndex

        nodeNumber: node number of a NodeGenericData (size=3) for 3 dataCoordinates, needed for discontinuous iteration (friction and contact); type: NodeIndex

        contactStiffness: normal contact stiffness [SI:N/m]; type: float

        contactDamping: normal contact damping [SI:N/(m s)]; type: float

        dynamicFriction: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, sec-module-physics; type: float

        staticFrictionOffset: static friction offset for friction model (static friction = dynamic friction + static offset), see StribeckFunction in exudyn.physics, sec-module-physics; type: float

        viscousFriction: viscous friction coefficient (velocity dependent part) for friction model, see StribeckFunction in exudyn.physics, sec-module-physics; type: float

        exponentialDecayStatic: exponential decay of static friction offset (must not be zero!), see StribeckFunction in exudyn.physics (named expVel there!), sec-module-physics; type: float

        frictionProportionalZone: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), sec-module-physics; type: float

        rollLength: roll length [m], symmetric w.r.t. centerpoint; type: float

        coefficientsHull: a vector of polynomial coefficients, which provides the polynomial of the CONVEX hull of the roll; :math:`\mathrm{hull}(x) = k_0 x^{n_p-1} + k x^{n_p-2} + \ldots + k_{n_p-2} x  + k_{n_p-1}`; type: array_like

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectContactConvexRoll

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position`` + ``Orientation``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), contactStiffness = 0., contactDamping = 0., dynamicFriction = 0., staticFrictionOffset = 0., viscousFriction = 0., exponentialDecayStatic = 0.001, frictionProportionalZone = 0.001, rollLength = 0., coefficientsHull = [], activeConnector = True, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.contactStiffness = contactStiffness
        self.contactDamping = contactDamping
        self.dynamicFriction = dynamicFriction
        self.staticFrictionOffset = staticFrictionOffset
        self.viscousFriction = viscousFriction
        self.exponentialDecayStatic = exponentialDecayStatic
        self.frictionProportionalZone = frictionProportionalZone
        self.rollLength = rollLength
        self.coefficientsHull = CheckForValidNumpyArray(coefficientsHull)
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ContactConvexRoll'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'contactStiffness', self.contactStiffness
        yield 'contactDamping', self.contactDamping
        yield 'dynamicFriction', self.dynamicFriction
        yield 'staticFrictionOffset', self.staticFrictionOffset
        yield 'viscousFriction', self.viscousFriction
        yield 'exponentialDecayStatic', self.exponentialDecayStatic
        yield 'frictionProportionalZone', self.frictionProportionalZone
        yield 'rollLength', self.rollLength
        yield 'coefficientsHull', self.coefficientsHull
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

class VObjectContactCoordinate:
    """Visualization data for ObjectContactCoordinate.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size = diameter of spring; size == -1.f means that default connector size is used; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectContactCoordinate:
    """A penalty-based contact condition for one coordinate: a force upon penetration of the gap between the coordinates of two markers, with the contact law of ObjectContactSphereSphere - linear by default, with a stiffness exponent and impact models; the contact state is kept in a data node (active set strategy).
    
    Args:
        name: connector's unique name; type: str

        markerNumbers: markers define contact gap; type: ArrayMarkerIndex

        nodeNumber: node number of a NodeGenericData with 1 data coordinate, the gap of the last discontinuous iteration (active set strategy), and a second one, the last impact velocity, if impactModel is not 0; type: NodeIndex

        contactStiffness: contact (penalty) stiffness [SI:N/m]; acts only upon penetration; type: float

        contactDamping: contact damping [SI:N/(m s)]; acts only upon penetration; type: float

        contactStiffnessExponent: exponent in the contact law [SI:1], as in ObjectContactSphereSphere; 1 is linear; type: float

        restitutionCoefficient: coefficient of restitution [SI:1], used by impactModel 1 and 2; must be > 0; type: float

        minimumImpactVelocity: lower bound [SI:m/s] of the impact velocity in the impact models; a larger damping at low impact velocities and in permanent contact; type: float

        impactModel: impact model, as in ObjectContactSphereSphere: 0) linear damping only; 1) Hunt-Crossley; 2) Gonthier et al. / Carvalho-Martins; contactDamping is added in all of them; type: int

        offset: offset [SI:m] of contact; type: float

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectContactCoordinate

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Coordinate``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1., restitutionCoefficient = 1., minimumImpactVelocity = 0., impactModel = 0, offset = 0., activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.contactStiffness = contactStiffness
        self.contactDamping = contactDamping
        self.contactStiffnessExponent = contactStiffnessExponent
        self.restitutionCoefficient = restitutionCoefficient
        self.minimumImpactVelocity = minimumImpactVelocity
        self.impactModel = impactModel
        self.offset = offset
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ContactCoordinate'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'contactStiffness', self.contactStiffness
        yield 'contactDamping', self.contactDamping
        yield 'contactStiffnessExponent', self.contactStiffnessExponent
        yield 'restitutionCoefficient', self.restitutionCoefficient
        yield 'minimumImpactVelocity', self.minimumImpactVelocity
        yield 'impactModel', self.impactModel
        yield 'offset', self.offset
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

class VObjectContactCircleCable2D:
    """Visualization data for ObjectContactCircleCable2D.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        showContactCircle: if True and show=True, the underlying contact circle is shown; uses circleTiling*4 for tiling (from VisualizationSettings.general); type: bool

        drawSize: drawing size = diameter of spring; size == -1.f means that default connector size is used; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, showContactCircle = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.showContactCircle = showContactCircle
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'showContactCircle', self.showContactCircle
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectContactCircleCable2D:
    """A very specialized penalty-based contact condition between a 2D circle (=marker0, any Position-marker) on a body and an ANCFCable2DShape (=marker1, Marker: BodyCable2DShape), in xy-plane.
    
    A node NodeGenericData is required with the number of cordinates according to the number of contact segments; the contact gap :math:`g` is integrated (piecewise linear) along the cable and circle; the contact force :math:`f_c` is zero for :math:`gap>0` and otherwise computed from :math:`f_c = g*contactStiffness`, without damping; during Newton iterations, the contact force is actived only, if :math:`dataCoordinate[0] <= 0`; dataCoordinate is set equal to gap in nonlinear iterations, but not modified in Newton iterations.
    
    Args:
        name: connector's unique name; type: str

        markerNumbers: markers define contact gap; type: ArrayMarkerIndex

        nodeNumber: node number of a NodeGenericData for nSegments dataCoordinates (used for active set strategy ==> hold the gap of the last discontinuous iteration and the friction state); type: NodeIndex

        numberOfContactSegments: number of linear contact segments to determine contact; each segment is a line and is associated to a data (history) variable; must be same as in according marker; type: int

        contactStiffness: contact (penalty) stiffness [SI:N/m/(contact segment)]; the stiffness is per contact segment; specific contact forces (per length) :math:`f_N` act in contact normal direction only upon penetration; type: float

        circleRadius: radius [SI:m] of contact circle; type: float

        offset: offset [SI:m] of contact, e.g. to include thickness of cable element; type: float

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectContactCircleCable2D

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``_None``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), numberOfContactSegments = 3, contactStiffness = 0., circleRadius = 0., offset = 0., activeConnector = True, visualization = {'show': True, 'showContactCircle': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.numberOfContactSegments = numberOfContactSegments
        self.contactStiffness = contactStiffness
        self.circleRadius = circleRadius
        self.offset = offset
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ContactCircleCable2D'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'numberOfContactSegments', self.numberOfContactSegments
        yield 'contactStiffness', self.contactStiffness
        yield 'circleRadius', self.circleRadius
        yield 'offset', self.offset
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VshowContactCircle', dict(self.visualization)["showContactCircle"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

class VObjectContactFrictionCircleCable2D:
    r"""Visualization data for ObjectContactFrictionCircleCable2D.
    
    Args:
        show: set True, if item is shown in visualization and false if it is not shown; note that only normal contact forces can be  drawn, which are approximated by :math:`k_c \cdot g` (neglecting damping term); type: bool

        showContactCircle: if True and show=True, the underlying contact circle is shown; uses circleTiling*4 for tiling (from VisualizationSettings.general); type: bool

        drawSize: drawing size = diameter of spring; size == -1.f means that default connector size is used; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, showContactCircle = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.showContactCircle = showContactCircle
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'showContactCircle', self.showContactCircle
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectContactFrictionCircleCable2D:
    r"""A very specialized penalty-based contact/friction condition between a 2D circle in the local x/y plane (=marker0, a RigidBody Marker, from node or object) on a body and an ANCFCable2DShape (=marker1, Marker: BodyCable2DShape), in xy-plane.
    
    A node NodeGenericData is required with 3:math:`\times`(number of contact segments) -- containing per segment: [contact gap, stick/slip (stick=0, slip=+-1, undefined=-2), last friction position]. The connector works with Cable2D and ALECable2D, HOWEVER, due to conceptual differences the (tangential) frictionStiffness cannot be used with ALECable2D; if using, it gives wrong tangential stresses, even though it may work in general.
    
    Args:
        name: connector's unique name; type: str

        markerNumbers: a marker :math:`m0` with position and orientation and a marker :math:`m1` of type BodyCable2DShape; together defining the contact geometry; type: ArrayMarkerIndex

        nodeNumber: node number of a NodeGenericData with 3 :math:`\times n_{cs}`  dataCoordinates (used for active set strategy → hold the gap of the last discontinuous iteration, friction state (+-1=slip, 0=stick, -2=undefined) and the last sticking position; initialize coordinates with list [0.1]*:math:`n_{cs}`+[-2]*:math:`n_{cs}`+[0.]*:math:`n_{cs}`, meaning that there is no initial contact with undefined slip/stick; type: NodeIndex

        numberOfContactSegments: number of linear contact segments to determine contact; each segment is a line and is associated to a data (history) variable; must be same as in according marker; type: int

        contactStiffness: contact (penalty) stiffness [SI:N/m/(contact segment)]; the stiffness is per contact segment; specific contact forces (per length) :math:`f_n` act in contact normal direction only upon penetration; type: float

        contactDamping: contact damping [SI:N/(m s)/(contact segment)]; the damping is per contact segment; acts in contact normal direction only upon penetration; type: float

        frictionVelocityPenalty: tangential velocity dependent penalty coefficient for friction [SI:N/(m s)/(contact segment)]; the coefficient causes tangential (contact) forces against relative tangential velocities in the contact area; type: float

        frictionStiffness: tangential displacement dependent penalty/stiffness coefficient for friction [SI:N/m/(contact segment)]; the coefficient causes tangential (contact) forces against relative tangential displacements in the contact area; type: float

        frictionCoefficient: friction coefficient [SI: 1]; tangential specific friction forces (per length) :math:`f_t` must fulfill the condition :math:`f_t \le \mu f_n`; type: float

        circleRadius: radius [SI:m] of contact circle; type: float

        useSegmentNormals: True: use normal and tangent according to linear segment; this is appropriate for very long (compared to circle) segments; False: use normals at segment points according to vector to circle center; this is more consistent for short segments, as forces are only applied in beam tangent and normal direction; type: bool

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectContactFrictionCircleCable2D

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``_None``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), numberOfContactSegments = 3, contactStiffness = 0., contactDamping = 0., frictionVelocityPenalty = 0., frictionStiffness = 0., frictionCoefficient = 0., circleRadius = 0., useSegmentNormals = True, activeConnector = True, visualization = {'show': True, 'showContactCircle': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.numberOfContactSegments = numberOfContactSegments
        self.contactStiffness = contactStiffness
        self.contactDamping = contactDamping
        self.frictionVelocityPenalty = frictionVelocityPenalty
        self.frictionStiffness = frictionStiffness
        self.frictionCoefficient = frictionCoefficient
        self.circleRadius = circleRadius
        self.useSegmentNormals = useSegmentNormals
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ContactFrictionCircleCable2D'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'numberOfContactSegments', self.numberOfContactSegments
        yield 'contactStiffness', self.contactStiffness
        yield 'contactDamping', self.contactDamping
        yield 'frictionVelocityPenalty', self.frictionVelocityPenalty
        yield 'frictionStiffness', self.frictionStiffness
        yield 'frictionCoefficient', self.frictionCoefficient
        yield 'circleRadius', self.circleRadius
        yield 'useSegmentNormals', self.useSegmentNormals
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VshowContactCircle', dict(self.visualization)["showContactCircle"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

class VObjectContactSphereSphere:
    """Visualization data for ObjectContactSphereSphere.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; draws spheres by given radii; type: bool

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = False, color = [0.7,0.7,0.7,1.]):
        self.show = show
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectContactSphereSphere:
    r"""A simple contact connector between two spheres, using various contact models and the option for contact of sphere inside hollow sphere (marker1).
    
    The connector implements at least the same functionality as in GeneralContact and is intended for simple setups and for testing, while GeneralContact is much more efficient due to parallelization approaches and efficient contact search.
    
    Args:
        name: constraints's unique name

        markerNumbers: list of markers representing centers of spheres, used in connector; type: ArrayMarkerIndex

        nodeNumber: node number of a NodeGenericData with numberOfDataCoordinates = 4 dataCoordinates, needed for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the  gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is the plastic overlap of the Edinburgh Adhesive Elasto-Plastic Model, initialized usually with 0 and set back to 0 in case that spheres have been separated.; type: NodeIndex

        spheresRadii: list containing radius of sphere 0 and radius of sphere 1 [SI:m].; type: [float,float]

        isHollowSphere1: flag, which determines, if sphere attached to marker 1 (radius 1) is a hollow sphere.; type: bool

        dynamicFriction: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, sec-module-physics; type: float

        frictionProportionalZone: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), sec-module-physics; type: float

        contactStiffness: normal contact stiffness [SI:N/m] (units in case that :math:`n_\mathrm{exp}=1`); type: float

        contactDamping: linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.; type: float

        contactStiffnessExponent: exponent in normal contact model [SI:1]; type: float

        constantPullOffForce: constant adhesion force [SI:N]; Edinburgh Adhesive Elasto-Plastic Model; type: float

        contactPlasticityRatio: ratio of contact stiffness for first loading and unloading/reloading [SI:1]; Edinburgh Adhesive Elasto-Plastic Model; :math:`\lambda_\mathrm{P}=1-k_c/K2`, which gives the contact stiffness for unloading/reloading :math:`K2 = k_c/(1-\lambda_\mathrm{P})`; set to 0 in order to fully deactivate Edinburgh Adhesive Elasto-Plastic Model model; type: float

        adhesionCoefficient: coefficient for adhesion [SI:N/m] (units in case that :math:`n_\mathrm{adh}=1`); Edinburgh Adhesive Elasto-Plastic Model; set to 0 to deactivate adhesion model; type: float

        adhesionExponent: exponent for adhesion coefficient [SI:1]; Edinburgh Adhesive Elasto-Plastic Model; type: float

        restitutionCoefficient: coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems); type: float

        minimumImpactVelocity: minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact); type: float

        impactModel: number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!; type: int

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectContactSphereSphere

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position`` + ``Orientation``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), spheresRadii = [-1.,-1.], isHollowSphere1 = False, dynamicFriction = 0., frictionProportionalZone = 0.001, contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1., constantPullOffForce = 0., contactPlasticityRatio = 0., adhesionCoefficient = 0., adhesionExponent = 1., restitutionCoefficient = 1., minimumImpactVelocity = 0., impactModel = 0, activeConnector = True, visualization = {'show': False, 'color': [0.7,0.7,0.7,1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.spheresRadii = np.array(spheresRadii)
        self.isHollowSphere1 = isHollowSphere1
        self.dynamicFriction = dynamicFriction
        self.frictionProportionalZone = frictionProportionalZone
        self.contactStiffness = contactStiffness
        self.contactDamping = contactDamping
        self.contactStiffnessExponent = contactStiffnessExponent
        self.constantPullOffForce = constantPullOffForce
        self.contactPlasticityRatio = contactPlasticityRatio
        self.adhesionCoefficient = adhesionCoefficient
        self.adhesionExponent = adhesionExponent
        self.restitutionCoefficient = restitutionCoefficient
        self.minimumImpactVelocity = minimumImpactVelocity
        self.impactModel = impactModel
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ContactSphereSphere'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'spheresRadii', self.spheresRadii
        yield 'isHollowSphere1', self.isHollowSphere1
        yield 'dynamicFriction', self.dynamicFriction
        yield 'frictionProportionalZone', self.frictionProportionalZone
        yield 'contactStiffness', self.contactStiffness
        yield 'contactDamping', self.contactDamping
        yield 'contactStiffnessExponent', self.contactStiffnessExponent
        yield 'constantPullOffForce', self.constantPullOffForce
        yield 'contactPlasticityRatio', self.contactPlasticityRatio
        yield 'adhesionCoefficient', self.adhesionCoefficient
        yield 'adhesionExponent', self.adhesionExponent
        yield 'restitutionCoefficient', self.restitutionCoefficient
        yield 'minimumImpactVelocity', self.minimumImpactVelocity
        yield 'impactModel', self.impactModel
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

class VObjectContactSphereTorus:
    """Visualization data for ObjectContactSphereTorus.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; draws spheres by given radii; type: bool

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = False, color = [0.7,0.7,0.7,1.]):
        self.show = show
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectContactSphereTorus:
    r"""A simple contact connector between a sphere (marker0) and a torus (marker1).
    
    The sphere is assumed to be placed inside of the torus (outer contact of sphere with torus currently not implemented!).
    
    Args:
        name: constraints's unique name

        markerNumbers: list of markers representing centers of sphere (marker 0) and center of torus (marker 1); type: ArrayMarkerIndex

        nodeNumber: node number of a NodeGenericData with numberOfDataCoordinates = 4 dataCoordinates, needed for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the  gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused.; type: NodeIndex

        sphereRadius: radius of sphere [SI:m]; type: float

        torusMajorRadius: major radius of torus [SI:m], representing center of rotated circle; type: float

        torusMinorRadius: minor radius of torus [SI:m], representing radius of circle of ring; type: float

        torusAxis: Vector containing rotation axis of torus; must be a unit vector.; type: [float,float,float]

        dynamicFriction: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, sec-module-physics; type: float

        frictionProportionalZone: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), sec-module-physics; type: float

        contactStiffness: normal contact stiffness [SI:N/m] (units in case that :math:`n_\mathrm{exp}=1`); type: float

        contactDamping: linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.; type: float

        contactStiffnessExponent: exponent in normal contact model [SI:1]; type: float

        restitutionCoefficient: coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems); type: float

        minimumImpactVelocity: minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact); type: float

        impactModel: number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!; type: int

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        radiusSphere: deprecated since 1.12.258, removed in 2031: use sphereRadius

        visualization: visualization data, see VObjectContactSphereTorus

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position`` + ``Orientation``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), sphereRadius = 0., torusMajorRadius = 0., torusMinorRadius = 0., torusAxis = [0,0,0], dynamicFriction = 0., frictionProportionalZone = 0.001, contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1., restitutionCoefficient = 1., minimumImpactVelocity = 0., impactModel = 0, activeConnector = True, radiusSphere = None, visualization = {'show': False, 'color': [0.7,0.7,0.7,1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.sphereRadius = sphereRadius
        self.torusMajorRadius = torusMajorRadius
        self.torusMinorRadius = torusMinorRadius
        self.torusAxis = np.array(torusAxis)
        self.dynamicFriction = dynamicFriction
        self.frictionProportionalZone = frictionProportionalZone
        self.contactStiffness = contactStiffness
        self.contactDamping = contactDamping
        self.contactStiffnessExponent = contactStiffnessExponent
        self.restitutionCoefficient = restitutionCoefficient
        self.minimumImpactVelocity = minimumImpactVelocity
        self.impactModel = impactModel
        self.activeConnector = activeConnector
        self.radiusSphere = radiusSphere
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ContactSphereTorus'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'sphereRadius', self.sphereRadius
        yield 'torusMajorRadius', self.torusMajorRadius
        yield 'torusMinorRadius', self.torusMinorRadius
        yield 'torusAxis', self.torusAxis
        yield 'dynamicFriction', self.dynamicFriction
        yield 'frictionProportionalZone', self.frictionProportionalZone
        yield 'contactStiffness', self.contactStiffness
        yield 'contactDamping', self.contactDamping
        yield 'contactStiffnessExponent', self.contactStiffnessExponent
        yield 'restitutionCoefficient', self.restitutionCoefficient
        yield 'minimumImpactVelocity', self.minimumImpactVelocity
        yield 'impactModel', self.impactModel
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]
        if self.radiusSphere is not None:
            yield 'radiusSphere', self.radiusSphere

    def __repr__(self):
        return str(dict(self))

class VObjectContactSphereTriangle:
    """Visualization data for ObjectContactSphereTriangle.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; draws spheres by given radii; type: bool

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = False, color = [0.7,0.7,0.7,1.]):
        self.show = show
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectContactSphereTriangle:
    r"""A simple contact connector between a sphere (marker0) and a triangle (marker1).
    
    Penalty-based contact is computed from penetration of the sphere with the triangle, including contact with edges if desired.
    
    Args:
        name: constraints's unique name

        markerNumbers: list of markers representing the center of the sphere (marker 0) and the reference point of the triangle (marker 1), where triangle nodal positions are defined in the local coordinates of marker 1.; type: ArrayMarkerIndex

        nodeNumber: node number of a NodeGenericData with numberOfDataCoordinates = 4 dataCoordinates, needed for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the  gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused.; type: NodeIndex

        sphereRadius: radius of sphere [SI:m]; type: float

        trianglePoints: triangle points, defined in marker 1 local coordinates; type: Vector3DList

        includeEdges: Binary flag, where 1 defines contact with edges 0, 2 with edge 1 and 4 with edge 2; 7 means that contact with all edges is included; edge 0 is the edge between node 0 and node 1; type: int

        dynamicFriction: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, sec-module-physics; type: float

        frictionProportionalZone: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), sec-module-physics; type: float

        contactStiffness: normal contact stiffness [SI:N/m] (units in case that :math:`n_\mathrm{exp}=1`); type: float

        contactDamping: linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.; type: float

        contactStiffnessExponent: exponent in normal contact model [SI:1]; type: float

        restitutionCoefficient: coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems); type: float

        minimumImpactVelocity: minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact); type: float

        impactModel: number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!; type: int

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        radiusSphere: deprecated since 1.12.258, removed in 2031: use sphereRadius

        visualization: visualization data, see VObjectContactSphereTriangle

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position`` + ``Orientation``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), sphereRadius = 0., trianglePoints = None, includeEdges = 7, dynamicFriction = 0., frictionProportionalZone = 0.001, contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1., restitutionCoefficient = 1., minimumImpactVelocity = 0., impactModel = 0, activeConnector = True, radiusSphere = None, visualization = {'show': False, 'color': [0.7,0.7,0.7,1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.sphereRadius = sphereRadius
        self.trianglePoints = trianglePoints
        self.includeEdges = includeEdges
        self.dynamicFriction = dynamicFriction
        self.frictionProportionalZone = frictionProportionalZone
        self.contactStiffness = contactStiffness
        self.contactDamping = contactDamping
        self.contactStiffnessExponent = contactStiffnessExponent
        self.restitutionCoefficient = restitutionCoefficient
        self.minimumImpactVelocity = minimumImpactVelocity
        self.impactModel = impactModel
        self.activeConnector = activeConnector
        self.radiusSphere = radiusSphere
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ContactSphereTriangle'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'sphereRadius', self.sphereRadius
        yield 'trianglePoints', self.trianglePoints
        yield 'includeEdges', self.includeEdges
        yield 'dynamicFriction', self.dynamicFriction
        yield 'frictionProportionalZone', self.frictionProportionalZone
        yield 'contactStiffness', self.contactStiffness
        yield 'contactDamping', self.contactDamping
        yield 'contactStiffnessExponent', self.contactStiffnessExponent
        yield 'restitutionCoefficient', self.restitutionCoefficient
        yield 'minimumImpactVelocity', self.minimumImpactVelocity
        yield 'impactModel', self.impactModel
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]
        if self.radiusSphere is not None:
            yield 'radiusSphere', self.radiusSphere

    def __repr__(self):
        return str(dict(self))

class VObjectContactCurveCircles:
    """Visualization data for ObjectContactCurveCircles.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; draws curve and circles with given radii; uses visualizationSettings circleTiling for circles and circleTiling/2 for tiling of non-straight segments; type: bool

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectContactCurveCircles:
    r"""A contact model between a curve defined by piecewise segments and a set of circles.
    
    The 2D curve may corotate in 3D with the underlying marker and also defines the plane of action for the circles. [REQUIRES FURTHER TESTING]
    
    Args:
        name: constraints's unique name

        markerNumbers: list of :math:`n_c+1` markers; marker :math:`m0` represents the marker carrying the curve; all other markers represent centers of :math:`n_c` circles, used in connector; type: ArrayMarkerIndex

        nodeNumber: node number of a NodeGenericData with nDataVariablesPerSegment dataCoordinates per segment, needed for discontinuous iteration; data variables contain values from last PostNewton iteration: data[0+3*i] is the circle number, data[1+3*i] is the gap, data[2+3*i] is the tangential velocity (and thus contains information if it is stick or slip); type: NodeIndex

        circlesRadii: Vector containing radii of :math:`n_c` circles [SI:m]; number according to size of markerNumbers-1; type: array_like

        segmentsData: matrix containing a set of two planar point coordinates in each row, representing segments attached to marker :math:`m0` and undergoing contact with the circles; for segment :math:`s0` row 0 reads :math:`[p_{0x,s0},\,p_{0y,s0},\,p_{1x,s0},\,p_{1y,s0}]`; note that the segments must be ordered such that going from :math:`\mathbf{p}_0` to :math:`\mathbf{p}_1`, the exterior lies on the right (positive) side. MatrixContainer has to be provided in dense mode!; type: PyMatrixContainer

        polynomialData: matrix containing coefficients for special polynomial enhancements of the linear segments; each row contains coefficients for polynomials for the according segment, prescribing slopes at beginning and end of segment as well as curvature at beginning and end of segment; slopes and curvatures are defined in a local x/y coordinate system where x is the segment axis (start: x=0; x-axis points towards end point) and the segment normal is in y-direction; MatrixContainer has to be provided in dense mode!; type: PyMatrixContainer

        dynamicFriction: dynamic friction coefficient: the friction force is :math:`\mu_d |f_N|`, regularized below frictionProportionalZone, see the equation; 0: no friction; type: float

        frictionProportionalZone: limit velocity [SI:m/s] up to which the friction force is proportional to the tangential velocity (regularization, against numerical oscillations); 0: no regularization; type: float

        contactStiffness: normal contact stiffness [SI:N/(m*m)]; type: float

        contactDamping: linear normal contact damping [SI:N/(m s)]; this damping is a simplification of real contact dissipation and should be used with care.; type: float

        contactModel: number of contact model: 0) linear model for stiffness and damping, only proportional to penetration; contact force is computed from :math:`l_\mathrm{seg}\left(p \cdot  \cdot k_c + \dot p \cdot d_c \right)` as long as :math:`p>0`; while this is numerically more stable, it gives jumps in forces when sliding over contact geometry 1) contact force proportional to integral over penetration area of circle with segments, giving a smoother contact force when sliding over geometry;

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectContactCurveCircles

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position`` + ``Orientation``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), circlesRadii = [], segmentsData = None, polynomialData = None, dynamicFriction = 0., frictionProportionalZone = 0.001, contactStiffness = 0., contactDamping = 0., contactModel = 0, activeConnector = True, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.circlesRadii = CheckForValidNumpyArray(circlesRadii)
        self.segmentsData = segmentsData
        self.polynomialData = polynomialData
        self.dynamicFriction = dynamicFriction
        self.frictionProportionalZone = frictionProportionalZone
        self.contactStiffness = contactStiffness
        self.contactDamping = contactDamping
        self.contactModel = contactModel
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ContactCurveCircles'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'circlesRadii', self.circlesRadii
        yield 'segmentsData', self.segmentsData
        yield 'polynomialData', self.polynomialData
        yield 'dynamicFriction', self.dynamicFriction
        yield 'frictionProportionalZone', self.frictionProportionalZone
        yield 'contactStiffness', self.contactStiffness
        yield 'contactDamping', self.contactDamping
        yield 'contactModel', self.contactModel
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
CamFollowerContactPlanar = ObjectContactCurveCircles
VCamFollowerContactPlanar = VObjectContactCurveCircles

class VObjectJointGeneric:
    """Visualization data for ObjectJointGeneric.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        axesRadius: radius of joint axes to draw; type: float

        axesLength: length of joint axes to draw; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, axesRadius = 0.1, axesLength = 0.4, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.axesRadius = axesRadius
        self.axesLength = axesLength
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'axesRadius', self.axesRadius
        yield 'axesLength', self.axesLength
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectJointGeneric:
    r"""A generic joint in 3D; constrains components of the absolute position and rotations of two points given by PointMarkers or RigidMarkers.
    
    The three rotation axes and sliding axes are those of the markers' frames; a rotation of these frames is given to the markers as their localHT.
    
    Args:
        name: constraints's unique name

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        constrainedAxes: flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; for :math:`j_i`, two values are possible: 0=free axis, 1=constrained axis; type: array_like

        rotationMarker0: local rotation matrix for marker :math:`m0`; translation and rotation axes for marker :math:`m0` are defined in the local body coordinate system and additionally transformed by rotationMarker0; type: array_like; deprecated since 1.12.244, removed in 2031: give the rotation to marker 0 as its localHT, e.g. MarkerBodyRigid(bodyNumber=b, localHT=exu.HT(rotation=A, translation=p))

        rotationMarker1: local rotation matrix for marker :math:`m1`; translation and rotation axes for marker :math:`m1` are defined in the local body coordinate system and additionally transformed by rotationMarker1; type: array_like; deprecated since 1.12.244, removed in 2031: give the rotation to marker 1 as its localHT, e.g. MarkerBodyRigid(bodyNumber=b, localHT=exu.HT(rotation=A, translation=p))

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        offsetUserFunctionParameters: vector of 6 parameters for joint's offsetUserFunction; type: array_like

        offsetUserFunction: A Python function which defines the time-dependent (fixed) offset of translation (indices 0,1,2) and rotation (indices 3,4,5) joint coordinates with parameters (mbs, t, offsetUserFunctionParameters); type: ObjectJointGenericOffsetUserFunction

        offsetUserFunction_t: (NOT IMPLEMENTED YET)time derivative of offsetUserFunction using the same parameters; type: ObjectJointGenericOffsetUserFunction_t

        alternativeConstraints: this is an experimental flag, may change in future: if uses alternative contraint equations for rotations, currently in case of 3 locked rotations: :math:`{}^{0}{\mathbf{t}}_{x0}\tp ({}^{0}{\mathbf{t}}_{y1} \times {}^{0}{\mathbf{t}}_{z0})`, :math:`{}^{0}{\mathbf{t}}_{y0}\tp ({}^{0}{\mathbf{t}}_{z1} \times {}^{0}{\mathbf{t}}_{x0})`, :math:`{}^{0}{\mathbf{t}}_{z0}\tp ({}^{0}{\mathbf{t}}_{x1} \times {}^{0}{\mathbf{t}}_{y0})`; this avoids 180° flips of the standard configuration in static computations, but leads to different values in Lagrange multipliers; type: bool

        visualization: visualization data, see VObjectJointGeneric

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``Position`` + ``Orientation``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], constrainedAxes = [1,1,1,1,1,1], rotationMarker0 = IIDiagMatrix(rowsColumns=3,value=1), rotationMarker1 = IIDiagMatrix(rowsColumns=3,value=1), activeConnector = True, offsetUserFunctionParameters = [0.,0.,0.,0.,0.,0.], offsetUserFunction: Union[ObjectJointGenericOffsetUserFunction, int] = 0, offsetUserFunction_t: Union[ObjectJointGenericOffsetUserFunction_t, int] = 0, alternativeConstraints = False, visualization = {'show': True, 'axesRadius': 0.1, 'axesLength': 0.4, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.constrainedAxes = copy.copy(constrainedAxes)
        self.rotationMarker0 = np.array(rotationMarker0)
        self.rotationMarker1 = np.array(rotationMarker1)
        self.activeConnector = activeConnector
        self.offsetUserFunctionParameters = np.array(offsetUserFunctionParameters)
        self.offsetUserFunction = offsetUserFunction
        self.offsetUserFunction_t = offsetUserFunction_t
        self.alternativeConstraints = alternativeConstraints
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'JointGeneric'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'constrainedAxes', self.constrainedAxes
        yield 'rotationMarker0', self.rotationMarker0
        yield 'rotationMarker1', self.rotationMarker1
        yield 'activeConnector', self.activeConnector
        yield 'offsetUserFunctionParameters', self.offsetUserFunctionParameters
        yield 'offsetUserFunction', self.offsetUserFunction
        yield 'offsetUserFunction_t', self.offsetUserFunction_t
        yield 'alternativeConstraints', self.alternativeConstraints
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VaxesRadius', dict(self.visualization)["axesRadius"]
        yield 'VaxesLength', dict(self.visualization)["axesLength"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
GenericJoint = ObjectJointGeneric
VGenericJoint = VObjectJointGeneric

class VObjectJointRevoluteZ:
    """Visualization data for ObjectJointRevoluteZ.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        axisRadius: radius of joint axis to draw; type: float

        axisLength: length of joint axis to draw; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, axisRadius = 0.1, axisLength = 0.4, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.axisRadius = axisRadius
        self.axisLength = axisLength
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'axisRadius', self.axisRadius
        yield 'axisLength', self.axisLength
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectJointRevoluteZ:
    """A revolute joint in 3D; constrains the position of two rigid body markers and the rotation about two axes, while the joint :math:`z`-rotation axis (defined in local coordinates of marker 0 / joint J0 coordinates) can freely rotate.
    
    The joint coordinate system is the frame of the markers; a rotation of it is given to the markers as their localHT. For easier definition of the joint, use ``mbs.CreateRevoluteJoint(...)`` for two rigid bodies (or ground).
    
    Args:
        name: constraints's unique name

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        rotationMarker0: local rotation matrix for marker :math:`m0`; translation and rotation axes for marker :math:`m0` are defined in the local body coordinate system and additionally transformed by rotationMarker0; type: array_like; deprecated since 1.12.244, removed in 2031: give the rotation to marker 0 as its localHT, e.g. MarkerBodyRigid(bodyNumber=b, localHT=exu.HT(rotation=A, translation=p))

        rotationMarker1: local rotation matrix for marker :math:`m1`; translation and rotation axes for marker :math:`m1` are defined in the local body coordinate system and additionally transformed by rotationMarker1; type: array_like; deprecated since 1.12.244, removed in 2031: give the rotation to marker 1 as its localHT, e.g. MarkerBodyRigid(bodyNumber=b, localHT=exu.HT(rotation=A, translation=p))

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectJointRevoluteZ

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``Position`` + ``Orientation``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], rotationMarker0 = IIDiagMatrix(rowsColumns=3,value=1), rotationMarker1 = IIDiagMatrix(rowsColumns=3,value=1), activeConnector = True, visualization = {'show': True, 'axisRadius': 0.1, 'axisLength': 0.4, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.rotationMarker0 = np.array(rotationMarker0)
        self.rotationMarker1 = np.array(rotationMarker1)
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'JointRevoluteZ'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'rotationMarker0', self.rotationMarker0
        yield 'rotationMarker1', self.rotationMarker1
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VaxisRadius', dict(self.visualization)["axisRadius"]
        yield 'VaxisLength', dict(self.visualization)["axisLength"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
RevoluteJointZ = ObjectJointRevoluteZ
VRevoluteJointZ = VObjectJointRevoluteZ

class VObjectJointPrismaticX:
    """Visualization data for ObjectJointPrismaticX.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        axisRadius: radius of joint axis to draw; type: float

        axisLength: length of joint axis to draw; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, axisRadius = 0.1, axisLength = 0.4, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.axisRadius = axisRadius
        self.axisLength = axisLength
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'axisRadius', self.axisRadius
        yield 'axisLength', self.axisLength
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectJointPrismaticX:
    """A prismatic joint in 3D; constrains the relative rotation of two rigid body markers and relative motion w.r.t.
    
    the joint :math:`y` and :math:`z` axes, allowing a relative motion along the joint :math:`x` axis (defined in local coordinates of marker 0 / joint J0 coordinates). The joint coordinate system is the frame of the markers; a rotation of it is given to the markers as their localHT. For easier definition of the joint, use ``mbs.CreatePrismaticJoint(...)`` for two rigid bodies (or ground).
    
    Args:
        name: constraints's unique name

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        rotationMarker0: local rotation matrix for marker :math:`m0`; translation and rotation axes for marker :math:`m0` are defined in the local body coordinate system and additionally transformed by rotationMarker0; type: array_like; deprecated since 1.12.244, removed in 2031: give the rotation to marker 0 as its localHT, e.g. MarkerBodyRigid(bodyNumber=b, localHT=exu.HT(rotation=A, translation=p))

        rotationMarker1: local rotation matrix for marker :math:`m1`; translation and rotation axes for marker :math:`m1` are defined in the local body coordinate system and additionally transformed by rotationMarker1; type: array_like; deprecated since 1.12.244, removed in 2031: give the rotation to marker 1 as its localHT, e.g. MarkerBodyRigid(bodyNumber=b, localHT=exu.HT(rotation=A, translation=p))

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectJointPrismaticX

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``Position`` + ``Orientation``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], rotationMarker0 = IIDiagMatrix(rowsColumns=3,value=1), rotationMarker1 = IIDiagMatrix(rowsColumns=3,value=1), activeConnector = True, visualization = {'show': True, 'axisRadius': 0.1, 'axisLength': 0.4, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.rotationMarker0 = np.array(rotationMarker0)
        self.rotationMarker1 = np.array(rotationMarker1)
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'JointPrismaticX'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'rotationMarker0', self.rotationMarker0
        yield 'rotationMarker1', self.rotationMarker1
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VaxisRadius', dict(self.visualization)["axisRadius"]
        yield 'VaxisLength', dict(self.visualization)["axisLength"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
PrismaticJointX = ObjectJointPrismaticX
VPrismaticJointX = VObjectJointPrismaticX

class VObjectJointSpherical:
    """Visualization data for ObjectJointSpherical.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        jointRadius: radius of joint to draw; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, jointRadius = 0.1, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.jointRadius = jointRadius
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'jointRadius', self.jointRadius
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectJointSpherical:
    """A spherical joint, which constrains the relative translation between two position based markers.
    
    Args:
        name: constraints's unique name

        markerNumbers: list of markers used in connector; :math:`m1` is the moving coin rigid body and :math:`m0` is the marker for the ground body, which use the localPosition=[0,0,0] for this marker!; type: ArrayMarkerIndex

        constrainedAxes: flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; for :math:`j_i`, two values are possible: 0=free axis, 1=constrained axis; type: array_like

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectJointSpherical

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``Position``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], constrainedAxes = [1,1,1], activeConnector = True, visualization = {'show': True, 'jointRadius': 0.1, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.constrainedAxes = copy.copy(constrainedAxes)
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'JointSpherical'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'constrainedAxes', self.constrainedAxes
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VjointRadius', dict(self.visualization)["jointRadius"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
SphericalJoint = ObjectJointSpherical
VSphericalJoint = VObjectJointSpherical

class VObjectJointRollingDisc:
    """Visualization data for ObjectJointRollingDisc.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        discWidth: width of disc for drawing; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, discWidth = 0.1, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.discWidth = discWidth
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'discWidth', self.discWidth
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectJointRollingDisc:
    """A joint representing a rolling rigid disc (marker 1) on a flat surface (marker 0, ground body) in global :math:`x`-:math:`y` plane.
    
    The contraint is based on an idealized rolling formulation with no slip. The contraints works for discs as long as the disc axis and the plane normal vector are not parallel. It must be assured that the disc has contact to ground in the initial configuration (adjust z-position of body accordingly). The ground body can be a rigid body which is moving. In this case, the flat surface is assumed to be in the :math:`x`-:math:`y`-plane at :math:`z=0`. Note that the rolling body must have the reference point at the center of the disc. NOTE: the cases of normal other than :math:`z`-direction, wheel axis other than :math:`x`-axis and moving ground body needs to be tested further, check your results!
    
    Args:
        name: constraints's unique name

        markerNumbers: list of markers used in connector; :math:`m0` represents the ground and :math:`m1` represents the rolling body, which has its reference point (=local position [0,0,0]) at the disc center point; type: ArrayMarkerIndex

        constrainedAxes: flags, which determine which constraints are active, in which :math:`j_0` represents lateral motion, :math:`j_1` longitudinal (forward/backward) motion and :math:`j_2` represents the normal (contact) direction; type: array_like

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        discRadius: defines the disc radius; type: float

        discAxis: axis of disc defined in marker :math:`m1` frame; type: [float,float,float]

        planeNormal: normal to the contact / rolling plane defined in marker :math:`m0` coordinates; type: [float,float,float]

        visualization: visualization data, see VObjectJointRollingDisc

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``Position`` + ``Orientation``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], constrainedAxes = [1,1,1], activeConnector = True, discRadius = 0, discAxis = [1,0,0], planeNormal = [0,0,1], visualization = {'show': True, 'discWidth': 0.1, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.constrainedAxes = copy.copy(constrainedAxes)
        self.activeConnector = activeConnector
        self.discRadius = discRadius
        self.discAxis = np.array(discAxis)
        self.planeNormal = np.array(planeNormal)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'JointRollingDisc'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'constrainedAxes', self.constrainedAxes
        yield 'activeConnector', self.activeConnector
        yield 'discRadius', self.discRadius
        yield 'discAxis', self.discAxis
        yield 'planeNormal', self.planeNormal
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdiscWidth', dict(self.visualization)["discWidth"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
RollingDiscJoint = ObjectJointRollingDisc
VRollingDiscJoint = VObjectJointRollingDisc

class VObjectJointRevolute2D:
    """Visualization data for ObjectJointRevolute2D.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size = radius of revolute joint; size == -1.f means that default connector size is used; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectJointRevolute2D:
    """A revolute joint in 2D; constrains the absolute 2D position of two points given by PointMarkers or RigidMarkers.
    
    Args:
        name: constraints's unique name

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectJointRevolute2D

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``Position``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'JointRevolute2D'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
RevoluteJoint2D = ObjectJointRevolute2D
VRevoluteJoint2D = VObjectJointRevolute2D

class VObjectJointPrismatic2D:
    """Visualization data for ObjectJointPrismatic2D.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size = radius of revolute joint; size == -1.f means that default connector size is used; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectJointPrismatic2D:
    """A prismatic joint in 2D; allows the relative motion of two bodies, using two RigidMarkers.
    
    Args:
        name: constraints's unique name

        markerNumbers: list of markers used in connector; type: ArrayMarkerIndex

        axisMarker0: direction of prismatic axis, given as a 3D vector in Marker0 frame; type: [float,float,float]

        normalMarker1: direction of normal to prismatic axis, given as a 3D vector in Marker1 frame; type: [float,float,float]

        constrainRotation: flag, which determines, if the connector also constrains the relative rotation of the two objects; if set to false, the constraint will keep an algebraic equation set equal zero; type: bool

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectJointPrismatic2D

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``Position`` + ``Orientation``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], axisMarker0 = [1.,0.,0.], normalMarker1 = [0.,1.,0.], constrainRotation = True, activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.axisMarker0 = np.array(axisMarker0)
        self.normalMarker1 = np.array(normalMarker1)
        self.constrainRotation = constrainRotation
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'JointPrismatic2D'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'axisMarker0', self.axisMarker0
        yield 'normalMarker1', self.normalMarker1
        yield 'constrainRotation', self.constrainRotation
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
PrismaticJoint2D = ObjectJointPrismatic2D
VPrismaticJoint2D = VObjectJointPrismatic2D

class VObjectJointSliding:
    """Visualization data for ObjectJointSliding.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size = radius of revolute joint; size == -1.f means that default connector size is used; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectJointSliding:
    """A specialized 3D sliding joint between a list of beam elements (updated marker1) and a position-based marker (marker0); the data coordinate x[0] provides the current index in slidingMarkerNumbers, and x[1] the local position in the cable element at the beginning of the timestep.
    
    Args:
        name: constraints's unique name

        markerNumbers: marker m0: position or rigid body marker of mass point or rigid body; marker m1: updated marker to Cable2D element, where the sliding joint currently is attached to; must be initialized with an appropriate (global) marker number according to the starting position of the sliding object; this marker changes with time (PostNewtonStep); type: ArrayMarkerIndex

        slidingMarkerNumbers: these markers are used to update marker m1, if the sliding position exceeds the current cable's range; the markers must be sorted such that marker :math:`m_{si}` at x=cable(i).length is equal to marker(i+1) at x=0 of cable(i+1); type: ArrayMarkerIndex

        slidingMarkerOffsets: this list contains the offsets of every sliding object (given by slidingMarkerNumbers) w.r.t. to the initial position (0): marker m0: offset=0, marker m1: offset=Length(cable0), marker m2: offset=Length(cable0)+Length(cable1), ...; type: array_like

        nodeNumber: node number of a NodeGenericData for 1 dataCoordinate showing the according marker number which is currently active and the start-of-step (global) sliding position; type: NodeIndex

        constrainRotations: flags for constrained rotation about x, y and z-axis: if flag=1, add constraint on rotation of marker m0 relative to respective axis; flag=0: sliding body can rotate freely about this axis; for ANCFCable, rotation about x-axis cannot be constrained; type: array_like

        constrainTranslations: flags for constrained translation in x, y and z-direction: if flag=1, add constraint on translation of marker m0 relative to respective axis; flag=0: sliding body can translate freely about this axis; along x-axis this should be usually 0, except for driven motion; type: array_like

        axialForce: ONLY APPLIES if useClassicalFormulation==True; axialForce represents an additional sliding force acting between beam and marker m0 body in axial (beam) direction; this force can be used to drive a body on a beam, but can only be changed with user functions.; type: float

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectJointSliding

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``_None``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], slidingMarkerNumbers = [], slidingMarkerOffsets = [], nodeNumber = exudyn.InvalidIndex(), constrainRotations = [1,1,1], constrainTranslations = [1,1,1], axialForce = 0, activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.slidingMarkerNumbers = copy.copy(slidingMarkerNumbers)
        self.slidingMarkerOffsets = np.array(slidingMarkerOffsets)
        self.nodeNumber = nodeNumber
        self.constrainRotations = copy.copy(constrainRotations)
        self.constrainTranslations = copy.copy(constrainTranslations)
        self.axialForce = axialForce
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'JointSliding'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'slidingMarkerNumbers', self.slidingMarkerNumbers
        yield 'slidingMarkerOffsets', self.slidingMarkerOffsets
        yield 'nodeNumber', self.nodeNumber
        yield 'constrainRotations', self.constrainRotations
        yield 'constrainTranslations', self.constrainTranslations
        yield 'axialForce', self.axialForce
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
SlidingJoint = ObjectJointSliding
VSlidingJoint = VObjectJointSliding

class VObjectJointSliding2D:
    """Visualization data for ObjectJointSliding2D.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size = radius of revolute joint; size == -1.f means that default connector size is used; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectJointSliding2D:
    """A specialized sliding joint (without rotation) in 2D between a Cable2D (marker1) and a position-based marker (marker0); the data coordinate x[0] provides the current index in slidingMarkerNumbers, and x[1] the local position in the cable element at the beginning of the timestep.
    
    Args:
        name: constraints's unique name

        markerNumbers: marker m0: position or rigid body marker of mass point or rigid body; marker m1: updated marker to Cable2D element, where the sliding joint currently is attached to; must be initialized with an appropriate (global) marker number according to the starting position of the sliding object; this marker changes with time (PostNewtonStep); type: ArrayMarkerIndex

        slidingMarkerNumbers: these markers are used to update marker m1, if the sliding position exceeds the current cable's range; the markers must be sorted such that marker :math:`m_{si}` at x=cable(i).length is equal to marker(i+1) at x=0 of cable(i+1); type: ArrayMarkerIndex

        slidingMarkerOffsets: this list contains the offsets of every sliding object (given by slidingMarkerNumbers) w.r.t. to the initial position (0): marker m0: offset=0, marker m1: offset=Length(cable0), marker m2: offset=Length(cable0)+Length(cable1), ...; type: array_like

        nodeNumber: node number of a NodeGenericData for 1 dataCoordinate showing the according marker number which is currently active and the start-of-step (global) sliding position; type: NodeIndex

        useClassicalFormulation: True: uses a formulation with 3 (+1) equations, including the force in sliding direction to be zero; forces in global coordinates, only index 3; False: use local formulation, which only needs 2 (+1) equations and can be used with index 2 formulation; type: bool

        constrainRotation: True: add constraint on rotation of marker m0 relative to slope (if True, marker m0 must be a rigid body marker); False: marker m0 body can rotate freely; type: bool

        axialForce: ONLY APPLIES if useClassicalFormulation==True; axialForce represents an additional sliding force acting between beam and marker m0 body in axial (beam) direction; this force can be used to drive a body on a beam, but can only be changed with user functions.; type: float

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        classicalFormulation: deprecated since 1.12.258, removed in 2031: use useClassicalFormulation

        visualization: visualization data, see VObjectJointSliding2D

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``_None``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], slidingMarkerNumbers = [], slidingMarkerOffsets = [], nodeNumber = exudyn.InvalidIndex(), useClassicalFormulation = True, constrainRotation = False, axialForce = 0, activeConnector = True, classicalFormulation = None, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.slidingMarkerNumbers = copy.copy(slidingMarkerNumbers)
        self.slidingMarkerOffsets = np.array(slidingMarkerOffsets)
        self.nodeNumber = nodeNumber
        self.useClassicalFormulation = useClassicalFormulation
        self.constrainRotation = constrainRotation
        self.axialForce = axialForce
        self.activeConnector = activeConnector
        self.classicalFormulation = classicalFormulation
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'JointSliding2D'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'slidingMarkerNumbers', self.slidingMarkerNumbers
        yield 'slidingMarkerOffsets', self.slidingMarkerOffsets
        yield 'nodeNumber', self.nodeNumber
        yield 'useClassicalFormulation', self.useClassicalFormulation
        yield 'constrainRotation', self.constrainRotation
        yield 'axialForce', self.axialForce
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]
        if self.classicalFormulation is not None:
            yield 'classicalFormulation', self.classicalFormulation

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
SlidingJoint2D = ObjectJointSliding2D
VSlidingJoint2D = VObjectJointSliding2D

class VObjectJointALEMoving2D:
    """Visualization data for ObjectJointALEMoving2D.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        drawSize: drawing size = radius of revolute joint; size == -1.f means that default connector size is used; type: float

        color: RGBA connector color; if R==-1, use default color; type: [float,float,float,float]

    """
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectJointALEMoving2D:
    """A specialized axially moving joint (without rotation) in 2D between a ALE Cable2D (marker1) and a position-based marker (marker0); ALE=Arbitrary Lagrangian Eulerian; the data coordinate x[0] provides the current index in slidingMarkerNumbers, and the ODE2 coordinate q[0] provides the (given) moving coordinate in the cable element.
    
    Args:
        name: constraints's unique name

        markerNumbers: marker m0: position-marker of mass point or rigid body; marker m1: updated marker to ANCF Cable2D element, where the sliding joint currently is attached to; must be initialized with an appropriate (global) marker number according to the starting position of the sliding object; this marker changes with time (PostNewtonStep); type: ArrayMarkerIndex

        slidingMarkerNumbers: a list of sn (global) marker numbers which are are used to update marker1; type: ArrayMarkerIndex

        slidingMarkerOffsets: this list contains the offsets of every sliding object (given by slidingMarkerNumbers) w.r.t. to the initial position (0): marker0: offset=0, marker1: offset=Length(cable0), marker2: offset=Length(cable0)+Length(cable1), ...; type: array_like

        slidingOffset: sliding offset [SI:m]: a scalar offset, which represents the (reference arc) length of all previous sliding cable elements; type: float

        nodeNumbers: node number of NodeGenericData (GD) with one data coordinate and of NodeGenericODE2 (ALE) with one ODE2 coordinate; type: ArrayNodeIndex

        usePenaltyFormulation: flag, which determines, if the connector is formulated with penalty, but still using algebraic equations (IsPenaltyConnector() still false); type: bool

        penaltyStiffness: penalty stiffness [SI:N/m] used if usePenaltyFormulation=True; type: float

        activeConnector: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint; type: bool

        visualization: visualization data, see VObjectJointALEMoving2D

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``_None``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], slidingMarkerNumbers = [], slidingMarkerOffsets = [], slidingOffset = 0., nodeNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], usePenaltyFormulation = False, penaltyStiffness = 0., activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.slidingMarkerNumbers = copy.copy(slidingMarkerNumbers)
        self.slidingMarkerOffsets = np.array(slidingMarkerOffsets)
        self.slidingOffset = slidingOffset
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.usePenaltyFormulation = usePenaltyFormulation
        self.penaltyStiffness = penaltyStiffness
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'JointALEMoving2D'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'slidingMarkerNumbers', self.slidingMarkerNumbers
        yield 'slidingMarkerOffsets', self.slidingMarkerOffsets
        yield 'slidingOffset', self.slidingOffset
        yield 'nodeNumbers', self.nodeNumbers
        yield 'usePenaltyFormulation', self.usePenaltyFormulation
        yield 'penaltyStiffness', self.penaltyStiffness
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
ALEMovingJoint2D = ObjectJointALEMoving2D
VALEMovingJoint2D = VObjectJointALEMoving2D

#+++++++++++++++++++++++++++++++
#MARKER
class VMarkerBodyMass:
    """Visualization data for MarkerBodyMass.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class MarkerBodyMass:
    """A marker attached to the body mass; use this marker to apply a body-load (e.g. gravitational force).
    
    Args:
        name: marker's unique name; type: str

        bodyNumber: body number to which marker is attached to; type: ObjectIndex

        visualization: visualization data, see VMarkerBodyMass

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``, ``BodyMass``

    """
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'BodyMass'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VMarkerBodyPosition:
    """Visualization data for MarkerBodyPosition.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class MarkerBodyPosition:
    r"""A position body-marker attached to a local (body-fixed) position :math:`{}^{b}{\mathbf{b}} = [b_0,\; b_1,\; b_2]` (:math:`x`, :math:`y`, and :math:`z` coordinates) of the body.
    
    It provides position information as well as the according derivatives (=velocity and derivative of position w.r.t. body coordinates). It can be used for connectors, joints or loads where position is required. If connectors also require orientation information, use a MarkerBodyRigid.
    
    Args:
        name: marker's unique name; type: str

        bodyNumber: body number to which marker is attached to; type: ObjectIndex

        localPosition: local body position of marker; e.g. local (body-fixed) position where force is applied to; type: [float,float,float]

        visualization: visualization data, see VMarkerBodyPosition

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``, ``Position``

    """
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), localPosition = [0.,0.,0.], visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.localPosition = np.array(localPosition)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'BodyPosition'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'localPosition', self.localPosition
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VMarkerBodyRigid:
    """Visualization data for MarkerBodyRigid.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class MarkerBodyRigid:
    r"""A rigid-body (position+orientation) body-marker attached to a local (body-fixed) position :math:`{}^{b}{\mathbf{b}} = [b_0,\; b_1,\; b_2]` (:math:`x`, :math:`y`, and :math:`z` coordinates) of the body.
    
    It provides position and orientation (rotation), as well as the according derivatives. It can be used for most connectors, joints or loads where either position, position and orientation, or orientation are required.
    
    Args:
        name: marker's unique name; type: str

        bodyNumber: body number to which marker is attached to; type: ObjectIndex

        localPosition: local body position of marker; e.g. local (body-fixed) position where force is applied to; the translation of localHT; type: [float,float,float]

        localHT: the frame of the marker in the body frame, as homogeneous transformation: its translation is localPosition, its rotation :math:`{}^{bm}{\Rot}` turns the marker frame against the body; a 4x4 matrix, its 16 values row by row or an exu.HT; None: not given; given together with localPosition, both must agree; type: array_like (4x4) or exudyn.HT

        visualization: visualization data, see VMarkerBodyRigid

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``, ``Position``, ``Orientation``

    """
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), localPosition = None, localHT = None, visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.localPosition = None if localPosition is None else np.array(localPosition)
        self.localHT = localHT
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'BodyRigid'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'localPosition', self.localPosition
        yield 'localHT', self.localHT
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VMarkerNodePosition:
    """Visualization data for MarkerNodePosition.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class MarkerNodePosition:
    """A node-Marker attached to a position-based node.
    
    It can be used for connectors, joints or loads where position is required. If connectors also require orientation information, use a MarkerNodeRigid.
    
    Args:
        name: marker's unique name; type: str

        nodeNumber: node number to which marker is attached to; type: NodeIndex

        visualization: visualization data, see VMarkerNodePosition

    Notes:
        Marker has/provides the following types: ``Node``, ``Position``

    """
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'NodePosition'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VMarkerNodeRigid:
    """Visualization data for MarkerNodeRigid.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class MarkerNodeRigid:
    """A rigid-body (position+orientation) node-marker attached to a rigid-body node.
    
    It provides position and orientation (rotation), as well as the according derivatives. It can be used for most connectors, joints or loads where either position, position and orientation, or orientation are required.
    
    Args:
        name: marker's unique name; type: str

        nodeNumber: node number to which marker is attached to; type: NodeIndex

        localHT: the frame of the marker in the node frame, as homogeneous transformation: its rotation turns the marker frame against the node; its translation must be zero for now; a 4x4 matrix, its 16 values row by row or an exu.HT; None: the node frame; type: array_like (4x4) or exudyn.HT

        visualization: visualization data, see VMarkerNodeRigid

    Notes:
        Marker has/provides the following types: ``Node``, ``Position``, ``Orientation``

    """
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), localHT = None, visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.localHT = localHT
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'NodeRigid'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'localHT', self.localHT
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VMarkerNodeCoordinate:
    """Visualization data for MarkerNodeCoordinate."""
    def __init__(self):
        pass

    def __iter__(self):
        yield from ()

    def __repr__(self):
        return str(dict(self))

class MarkerNodeCoordinate:
    """A node-Marker attached to a ODE2 coordinate of a node; this marker allows to connect a coordinate-based constraint or connector to a nodal coordinate (also NodeGround); for ODE1 coordinates use ``MarkerNodeODE1Coordinate``.
    
    Args:
        name: marker's unique name; type: str

        nodeNumber: node number to which marker is attached to; type: NodeIndex

        coordinate: coordinate of node to which marker is attached to; type: int

        visualization: visualization data, see VMarkerNodeCoordinate

    Notes:
        Marker has/provides the following types: ``Node``, ``Coordinate``

    """
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), coordinate = exudyn.InvalidIndex(), visualization = {}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.coordinate = coordinate
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'NodeCoordinate'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'coordinate', self.coordinate

    def __repr__(self):
        return str(dict(self))

class VMarkerNodeCoordinates:
    """Visualization data for MarkerNodeCoordinates."""
    def __init__(self):
        pass

    def __iter__(self):
        yield from ()

    def __repr__(self):
        return str(dict(self))

class MarkerNodeCoordinates:
    """A node-Marker attached to all ODE2 coordinates of a node.
    
    IN CONTRAST to MarkerNodeCoordinate, the marker coordinates INCLUDE the reference values! For ODE1 coordinates use ``MarkerNodeODE1Coordinates``.
    
    Args:
        name: marker's unique name; type: str

        nodeNumber: node number to which marker is attached to; type: NodeIndex

        visualization: visualization data, see VMarkerNodeCoordinates

    Notes:
        Marker has/provides the following types: ``Node``, ``Coordinate``

    """
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), visualization = {}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'NodeCoordinates'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber

    def __repr__(self):
        return str(dict(self))

class VMarkerNodeODE1Coordinate:
    """Visualization data for MarkerNodeODE1Coordinate."""
    def __init__(self):
        pass

    def __iter__(self):
        yield from ()

    def __repr__(self):
        return str(dict(self))

class MarkerNodeODE1Coordinate:
    """A node-Marker attached to a ODE1 coordinate of a node.
    
    Args:
        name: marker's unique name; type: str

        nodeNumber: node number to which marker is attached to; type: NodeIndex

        coordinate: coordinate of node to which marker is attached to; type: int

        visualization: visualization data, see VMarkerNodeODE1Coordinate

    Notes:
        Marker has/provides the following types: ``Node``, ``Coordinate``

    """
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), coordinate = exudyn.InvalidIndex(), visualization = {}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.coordinate = coordinate
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'NodeODE1Coordinate'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'coordinate', self.coordinate

    def __repr__(self):
        return str(dict(self))

class VMarkerNodeRotationCoordinate:
    """Visualization data for MarkerNodeRotationCoordinate."""
    def __init__(self):
        pass

    def __iter__(self):
        yield from ()

    def __repr__(self):
        return str(dict(self))

class MarkerNodeRotationCoordinate:
    """A node-Marker attached to a a node containing rotation; the Marker measures a rotation coordinate (Tait-Bryan angles) or angular velocities on the velocity level.
    
    Args:
        name: marker's unique name; type: str

        nodeNumber: node number to which marker is attached to; type: NodeIndex

        rotationCoordinate: rotation coordinate: 0=x, 1=y, 2=z; type: int

        visualization: visualization data, see VMarkerNodeRotationCoordinate

    Notes:
        Marker has/provides the following types: ``Node``, ``Coordinate``

    """
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), rotationCoordinate = exudyn.InvalidIndex(), visualization = {}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.rotationCoordinate = rotationCoordinate
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'NodeRotationCoordinate'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'rotationCoordinate', self.rotationCoordinate

    def __repr__(self):
        return str(dict(self))

class VMarkerBodiesRelativeTranslationCoordinate:
    """Visualization data for MarkerBodiesRelativeTranslationCoordinate.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class MarkerBodiesRelativeTranslationCoordinate:
    """A coordinate-based Marker attached to two rigid bodies or beams which computes the relative translation between the bodies according to the given axis.
    
    This marker can be used together with coordinate-based constraints and connectors (e.g., CoordinateSpringDamper and CoordinateConstraint). NOTE: it is assumed that the two bodies can only move along the given axis (e.g., constrained by a prismatic joint) -- otherwise results may be unexpected. NOTE: this approach is not compatible with FFRF-based flexible bodies and currently requires and intermediate rigid body.
    
    Args:
        name: marker's unique name; type: str

        bodyNumbers: list of body numbers for which relative coordinate is computed; type: ArrayObjectIndex

        localPosition0: local position on body 0; i.e. local (body-fixed) position where position is measured and force is applied to; type: [float,float,float]

        localPosition1: local position on body 1; i.e. local (body-fixed) position where position is measured and force is applied to; type: [float,float,float]

        axis0: axis defined in body 0, along which the relative translation is measured; type: [float,float,float]

        offset: translation offset [SI:m] subtracted from the translation; can be used to change the zero position; type: float

        visualization: visualization data, see VMarkerBodiesRelativeTranslationCoordinate

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``, ``Coordinate``

    """
    def __init__(self, name = '', bodyNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], localPosition0 = [0.,0.,0.], localPosition1 = [0.,0.,0.], axis0 = [1.,0.,0.], offset = 0., visualization = {'show': True}):
        self.name = name
        self.bodyNumbers = copy.copy(bodyNumbers)
        self.localPosition0 = np.array(localPosition0)
        self.localPosition1 = np.array(localPosition1)
        self.axis0 = np.array(axis0)
        self.offset = offset
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'BodiesRelativeTranslationCoordinate'
        yield 'name', self.name
        yield 'bodyNumbers', self.bodyNumbers
        yield 'localPosition0', self.localPosition0
        yield 'localPosition1', self.localPosition1
        yield 'axis0', self.axis0
        yield 'offset', self.offset
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VMarkerBodiesRelativeRotationCoordinate:
    """Visualization data for MarkerBodiesRelativeRotationCoordinate.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class MarkerBodiesRelativeRotationCoordinate:
    r"""A coordinate-based Marker attached to two rigid bodies or beams which computes the relative rotation between the bodies according to the given axis; this marker can be used together with coordinate-based constraints and connectors (e.g., CoordinateSpringDamper and CoordinateConstraint).
    
    NOTE: it is assumed that the two bodies can only rotate about the given axis (e.g., constrained by a revolute joint) -- otherwise results may be unexpected. NOTE: this approach is not compatible with FFRF-based flexible bodies and currently requires and intermediate rigid body.
    
    Args:
        name: marker's unique name; type: str

        bodyNumbers: list of body numbers for which relative coordinate is computed; type: ArrayObjectIndex

        nodeNumber: node number of NodeGenericData with 1 coordinate which contains previous angle for continuation of angles (initialize accordingly if needed); if node is not supplied, angles will have jump outside :math:`\pm \pi`; type: NodeIndex

        localPosition0: local position on body 0; i.e. local (body-fixed) position where position is measured and force is applied to; type: [float,float,float]

        localPosition1: local position on body 1; i.e. local (body-fixed) position where position is measured and force is applied to; type: [float,float,float]

        axis0: axis defined in body 0, along which the relative rotation is measured; type: [float,float,float]

        offset: rotation offset [SI:1] subtracted from the measured rotation; can be used to change the zero rotation; type: float

        visualization: visualization data, see VMarkerBodiesRelativeRotationCoordinate

    Notes:
        Marker has/provides the following types: ``Node``, ``Object``, ``Body``, ``Coordinate``

    """
    def __init__(self, name = '', bodyNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), localPosition0 = [0.,0.,0.], localPosition1 = [0.,0.,0.], axis0 = [1.,0.,0.], offset = 0., visualization = {'show': True}):
        self.name = name
        self.bodyNumbers = copy.copy(bodyNumbers)
        self.nodeNumber = nodeNumber
        self.localPosition0 = np.array(localPosition0)
        self.localPosition1 = np.array(localPosition1)
        self.axis0 = np.array(axis0)
        self.offset = offset
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'BodiesRelativeRotationCoordinate'
        yield 'name', self.name
        yield 'bodyNumbers', self.bodyNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'localPosition0', self.localPosition0
        yield 'localPosition1', self.localPosition1
        yield 'axis0', self.axis0
        yield 'offset', self.offset
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VMarkerSuperElementPosition:
    """Visualization data for MarkerSuperElementPosition.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        showMarkerNodes: set true, if all nodes are shown (similar to marker, but with less intensity); type: bool

    """
    def __init__(self, show = True, showMarkerNodes = True):
        self.show = show
        self.showMarkerNodes = showMarkerNodes

    def __iter__(self):
        yield 'show', self.show
        yield 'showMarkerNodes', self.showMarkerNodes

    def __repr__(self):
        return str(dict(self))

class MarkerSuperElementPosition:
    """A position marker attached to a SuperElement, such as ObjectFFRF, ObjectGenericODE2 and ObjectFFRFreducedOrder (for which it is in its current implementation inefficient for large number of meshNodeNumbers).
    
    The marker acts on the mesh (interface) nodes, not on the underlying nodes of the object.
    
    Args:
        name: marker's unique name; type: str

        bodyNumber: body number to which marker is attached to; type: ObjectIndex

        meshNodeNumbers: a list of :math:`n_m` mesh node numbers of superelement (=interface nodes) which are used to compute the body-fixed marker position; the related nodes must provide 3D position information, such as NodePoint, NodePoint2D, NodeRigidBody[..]; in order to retrieve the global node number, the generic body needs to convert local into global node numbers; type: array_like

        weightingFactors: a list of :math:`n_m` weighting factors per node to compute the final local position; the sum of these weights shall be 1, such that a summation of all nodal positions times weights gives the average position of the marker; type: array_like

        visualization: visualization data, see VMarkerSuperElementPosition

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``, ``Position``

    """
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), meshNodeNumbers = [], weightingFactors = [], visualization = {'show': True, 'showMarkerNodes': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.meshNodeNumbers = copy.copy(meshNodeNumbers)
        self.weightingFactors = np.array(weightingFactors)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'SuperElementPosition'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'meshNodeNumbers', self.meshNodeNumbers
        yield 'weightingFactors', self.weightingFactors
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VshowMarkerNodes', dict(self.visualization)["showMarkerNodes"]

    def __repr__(self):
        return str(dict(self))

class VMarkerSuperElementRigid:
    """Visualization data for MarkerSuperElementRigid.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

        showMarkerNodes: set true, if all nodes are shown (similar to marker, but with less intensity); type: bool

    """
    def __init__(self, show = True, showMarkerNodes = True):
        self.show = show
        self.showMarkerNodes = showMarkerNodes

    def __iter__(self):
        yield 'show', self.show
        yield 'showMarkerNodes', self.showMarkerNodes

    def __repr__(self):
        return str(dict(self))

class MarkerSuperElementRigid:
    """A position and orientation (rigid-body) marker attached to a SuperElement, such as ObjectFFRF, ObjectGenericODE2 and ObjectFFRFreducedOrder (for which it may be inefficient).
    
    The marker acts on the mesh nodes, not on the underlying nodes of the object. Note that in contrast to the MarkerSuperElementPosition, this marker needs a set of interface nodes which are not aligned at one line, such that these node points can represent a rigid body motion. Note that definitions of marker positions are slightly different from MarkerSuperElementPosition.
    
    Args:
        name: marker's unique name; type: str

        bodyNumber: body number to which marker is attached to; type: ObjectIndex

        offset: local marker SuperElement reference position offset used to correct the center point of the marker, which is computed from the weighted average of reference node positions (which may have some offset to the desired joint position). Note that this offset shall be small and larger offsets can cause instability in simulation models (better to have symmetric meshes at joints). The translation of localHT.; type: [float,float,float]

        localHT: the frame of the marker against the frame the marker computes from the mesh nodes, as homogeneous transformation: its translation is offset, its rotation turns the marker frame; a 4x4 matrix, its 16 values row by row or an exu.HT; None: not given; given together with offset, both must agree; type: array_like (4x4) or exudyn.HT

        meshNodeNumbers: a list of :math:`n_m` mesh node numbers of superelement (=interface nodes) which are used to compute the body-fixed marker position and orientation; the related nodes must provide 3D position information, such as NodePoint, NodePoint2D, NodeRigidBody[..]; in order to retrieve the global node number, the generic body needs to convert local into global node numbers; type: array_like

        weightingFactors: a list of :math:`n_m` weighting factors per node to compute the final local position and orientation; these factors could be based on surface integrals of the constrained mesh faces; type: array_like

        useAlternativeApproach: this flag switches between two versions for the computation of the rotation and angular velocity of the marker; alternative approach uses skew symmetric matrix of reference position; follows the inertia concept; type: bool

        rotationsExponentialMap: Experimental flag (2 is the correct value and will be used in future, removing this flag): This value switches different behavior for computation of rotations and angular velocities: 0 uses linearized rotations and angular velocities, 1 uses the exponential map for rotations but linear angular velocities, 2 uses the exponential map for rotations and the according tangent map for angular velocities; type: int

        visualization: visualization data, see VMarkerSuperElementRigid

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``, ``Position``, ``Orientation``

    """
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), offset = None, localHT = None, meshNodeNumbers = [], weightingFactors = [], useAlternativeApproach = True, rotationsExponentialMap = 2, visualization = {'show': True, 'showMarkerNodes': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.offset = None if offset is None else np.array(offset)
        self.localHT = localHT
        self.meshNodeNumbers = copy.copy(meshNodeNumbers)
        self.weightingFactors = np.array(weightingFactors)
        self.useAlternativeApproach = useAlternativeApproach
        self.rotationsExponentialMap = rotationsExponentialMap
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'SuperElementRigid'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'offset', self.offset
        yield 'localHT', self.localHT
        yield 'meshNodeNumbers', self.meshNodeNumbers
        yield 'weightingFactors', self.weightingFactors
        yield 'useAlternativeApproach', self.useAlternativeApproach
        yield 'rotationsExponentialMap', self.rotationsExponentialMap
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VshowMarkerNodes', dict(self.visualization)["showMarkerNodes"]

    def __repr__(self):
        return str(dict(self))

class VMarkerKinematicTreeRigid:
    """Visualization data for MarkerKinematicTreeRigid.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class MarkerKinematicTreeRigid:
    """A position and orientation (rigid-body) marker attached to a kinematic tree.
    
    The marker is attached to the ObjectKinematicTree object and additionally needs a link number as well as a local position, similar to the SensorKinematicTree. The marker allows to attach loads (LoadForceVector and LoadTorqueVector) at arbitrary links or position. It also allows to attach connectors (e.g., spring dampers or actuators) to the kinematic tree. Finally, joint constraints can be attached, which allows for realization of closed loop structures. NOTE, however, that it is less efficient to attach many markers to a kinematic tree, therefor for forces or joint control use the structures available in kinematic tree whenever possible.
    
    Args:
        name: marker's unique name; type: str

        objectNumber: body number to which marker is attached to; type: ObjectIndex

        linkNumber: number of link in KinematicTree to which marker is attached to; type: int

        localPosition: local (link-fixed) position of marker at link :math:`n_l`, using the link (:math:`n_l`) coordinate system; the translation of localHT; type: [float,float,float]

        localHT: the frame of the marker in the link frame, as homogeneous transformation: its translation is localPosition, its rotation turns the marker frame against the link; a 4x4 matrix, its 16 values row by row or an exu.HT; None: not given; given together with localPosition, both must agree; type: array_like (4x4) or exudyn.HT

        visualization: visualization data, see VMarkerKinematicTreeRigid

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``, ``Position``, ``Orientation``

    """
    def __init__(self, name = '', objectNumber = exudyn.InvalidIndex(), linkNumber = exudyn.InvalidIndex(), localPosition = None, localHT = None, visualization = {'show': True}):
        self.name = name
        self.objectNumber = objectNumber
        self.linkNumber = linkNumber
        self.localPosition = None if localPosition is None else np.array(localPosition)
        self.localHT = localHT
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'KinematicTreeRigid'
        yield 'name', self.name
        yield 'objectNumber', self.objectNumber
        yield 'linkNumber', self.linkNumber
        yield 'localPosition', self.localPosition
        yield 'localHT', self.localHT
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VMarkerObjectODE2Coordinates:
    """Visualization data for MarkerObjectODE2Coordinates."""
    def __init__(self):
        pass

    def __iter__(self):
        yield from ()

    def __repr__(self):
        return str(dict(self))

class MarkerObjectODE2Coordinates:
    """A Marker attached to all coordinates of an object (currently only body is possible), e.g. to apply special constraints or loads on all coordinates.
    
    The measured coordinates INCLUDE reference + current coordinates.
    
    Args:
        name: marker's unique name; type: str

        objectNumber: body number to which marker is attached to; type: ObjectIndex

        visualization: visualization data, see VMarkerObjectODE2Coordinates

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``, ``Coordinate``

    """
    def __init__(self, name = '', objectNumber = exudyn.InvalidIndex(), visualization = {}):
        self.name = name
        self.objectNumber = objectNumber
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'ObjectODE2Coordinates'
        yield 'name', self.name
        yield 'objectNumber', self.objectNumber

    def __repr__(self):
        return str(dict(self))

class VMarkerBodyCable2DShape:
    """Visualization data for MarkerBodyCable2DShape.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class MarkerBodyCable2DShape:
    """A special Marker attached to a 2D ANCF beam finite element with cubic interpolation and 8 coordinates.
    
    Args:
        name: marker's unique name; type: str

        bodyNumber: body number to which marker is attached to; type: ObjectIndex

        numberOfSegments: number of number of segments; each segment is a line and is associated to a data (history) variable; must be same as in according contact element; type: int

        verticalOffset: vertical offset from beam axis in positive (local) Y-direction; this offset accounts for consistent computation of positions and velocities at the surface of the beam; type: float

        visualization: visualization data, see VMarkerBodyCable2DShape

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``, ``Coordinate``

    """
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), numberOfSegments = 3, verticalOffset = 0., visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.numberOfSegments = numberOfSegments
        self.verticalOffset = verticalOffset
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'BodyCable2DShape'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'numberOfSegments', self.numberOfSegments
        yield 'verticalOffset', self.verticalOffset
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VMarkerBodyCable2DCoordinates:
    """Visualization data for MarkerBodyCable2DCoordinates.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class MarkerBodyCable2DCoordinates:
    """A special Marker attached to the coordinates of a 2D ANCF beam finite element with cubic interpolation.
    
    Args:
        name: marker's unique name; type: str

        bodyNumber: body number to which marker is attached to; type: ObjectIndex

        visualization: visualization data, see VMarkerBodyCable2DCoordinates

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``, ``Coordinate``

    """
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'BodyCable2DCoordinates'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VMarkerBodyBeamShape:
    """Visualization data for MarkerBodyBeamShape.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class MarkerBodyBeamShape:
    """A special Marker attached to a 3D beam finite element which provides at least position and tangent to the beam axis.
    
    Args:
        name: marker's unique name; type: str

        bodyNumber: body number to which marker is attached to (beam type); type: ObjectIndex

        visualization: visualization data, see VMarkerBodyBeamShape

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``

    """
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'BodyBeamShape'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

#+++++++++++++++++++++++++++++++
#LOAD
class VLoadForceVector:
    """Visualization data for LoadForceVector.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class LoadForceVector:
    """Load with (3D) force vector; attached to position-based marker.
    
    Args:
        name: load's unique name; type: str

        markerNumber: marker's number to which load is applied; type: MarkerIndex

        loadVector: vector-valued load [SI:N]; in case of a user function, this vector is ignored; type: [float,float,float]

        bodyFixed: if bodyFixed is true, the load is defined in body-fixed (local) coordinates, leading to a follower force; if false: global coordinates are used; type: bool

        loadVectorUserFunction: A Python function which defines the time-dependent load and replaces loadVector; see description below; NOTE that in static computations, the loadFactor is always 1 for forces computed by user functions (this means for the static computation, that a user function returning [t*5,t*1,0] corresponds to loadVector=[5,1,0] without a user function); the render window draws the load with the value of the user function if visualizationSettings.loads.drawWithUserFunction is True and the user function is symbolic, or a Python function with visualizationSettings.general.useMultiThreadedRendering = False - the render thread cannot call Python -, otherwise with loadVector; a sensor (SensorLoad) returns the force of the user function in any case; type: LoadForceVectorLoadVectorUserFunction

        visualization: visualization data, see VLoadForceVector

    Notes:
        Requested Marker type: ``Position``

    """
    def __init__(self, name = '', markerNumber = exudyn.InvalidIndex(), loadVector = [0.,0.,0.], bodyFixed = False, loadVectorUserFunction: Union[LoadForceVectorLoadVectorUserFunction, int] = 0, visualization = {'show': True}):
        self.name = name
        self.markerNumber = markerNumber
        self.loadVector = np.array(loadVector)
        self.bodyFixed = bodyFixed
        self.loadVectorUserFunction = loadVectorUserFunction
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'loadType', 'ForceVector'
        yield 'name', self.name
        yield 'markerNumber', self.markerNumber
        yield 'loadVector', self.loadVector
        yield 'bodyFixed', self.bodyFixed
        yield 'loadVectorUserFunction', self.loadVectorUserFunction
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
Force = LoadForceVector
VForce = VLoadForceVector

class VLoadTorqueVector:
    """Visualization data for LoadTorqueVector.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class LoadTorqueVector:
    """Load with (3D) torque vector; attached to rigidbody-based marker.
    
    Args:
        name: load's unique name; type: str

        markerNumber: marker's number to which load is applied; type: MarkerIndex

        loadVector: vector-valued load [SI:N]; in case of a user function, this vector is ignored; type: [float,float,float]

        bodyFixed: if bodyFixed is true, the load is defined in body-fixed (local) coordinates, leading to a follower torque; if false: global coordinates are used; type: bool

        loadVectorUserFunction: A Python function which defines the time-dependent load and replaces loadVector; see description below; see also notes on loadFactor and drawing in LoadForceVector! Example for Python function: def f(mbs, t, loadVector): return [loadVector[0]*np.sin(t*10*2*3.1415),0,0]; type: LoadTorqueVectorLoadVectorUserFunction

        visualization: visualization data, see VLoadTorqueVector

    Notes:
        Requested Marker type: ``Orientation``

    """
    def __init__(self, name = '', markerNumber = exudyn.InvalidIndex(), loadVector = [0.,0.,0.], bodyFixed = False, loadVectorUserFunction: Union[LoadTorqueVectorLoadVectorUserFunction, int] = 0, visualization = {'show': True}):
        self.name = name
        self.markerNumber = markerNumber
        self.loadVector = np.array(loadVector)
        self.bodyFixed = bodyFixed
        self.loadVectorUserFunction = loadVectorUserFunction
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'loadType', 'TorqueVector'
        yield 'name', self.name
        yield 'markerNumber', self.markerNumber
        yield 'loadVector', self.loadVector
        yield 'bodyFixed', self.bodyFixed
        yield 'loadVectorUserFunction', self.loadVectorUserFunction
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
Torque = LoadTorqueVector
VTorque = VLoadTorqueVector

class VLoadMassProportional:
    """Visualization data for LoadMassProportional.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class LoadMassProportional:
    """Load attached to MarkerBodyMass marker, applying a 3D vector load (e.g. the vector [0,-g,0] is used to apply gravitational loading of size g in negative y-direction).
    
    Args:
        name: load's unique name; type: str

        markerNumber: marker's number to which load is applied; type: MarkerIndex

        loadVector: vector-valued load [SI:N/kg = m/s:math:`^2`]; typically, this will be the gravity vector in global coordinates; in case of a user function, this v is ignored; type: [float,float,float]

        loadVectorUserFunction: A Python function which defines the time-dependent load; see description below; see also notes on loadFactor and drawing in LoadForceVector!; type: LoadMassProportionalLoadVectorUserFunction

        visualization: visualization data, see VLoadMassProportional

    Notes:
        Requested Marker type: ``Body`` + ``BodyMass``

    """
    def __init__(self, name = '', markerNumber = exudyn.InvalidIndex(), loadVector = [0.,0.,0.], loadVectorUserFunction: Union[LoadMassProportionalLoadVectorUserFunction, int] = 0, visualization = {'show': True}):
        self.name = name
        self.markerNumber = markerNumber
        self.loadVector = np.array(loadVector)
        self.loadVectorUserFunction = loadVectorUserFunction
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'loadType', 'MassProportional'
        yield 'name', self.name
        yield 'markerNumber', self.markerNumber
        yield 'loadVector', self.loadVector
        yield 'loadVectorUserFunction', self.loadVectorUserFunction
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
Gravity = LoadMassProportional
VGravity = VLoadMassProportional

class VLoadCoordinate:
    """Visualization data for LoadCoordinate."""
    def __init__(self):
        pass

    def __iter__(self):
        yield from ()

    def __repr__(self):
        return str(dict(self))

class LoadCoordinate:
    """Load with scalar value, which is attached to a coordinate-based marker; the load can be used e.g. to apply a force to a single axis of a body, a nodal coordinate of a finite element  or a torque to the rotatory DOF of a rigid body.
    
    Args:
        name: load's unique name; type: str

        markerNumber: marker's number to which load is applied; type: MarkerIndex

        load: scalar load [SI:N]; in case of a user function, this value is ignored; type: float

        loadUserFunction: A Python function which defines the time-dependent load and replaces the load; see description below; see also notes on loadFactor and drawing in LoadForceVector!; type: LoadCoordinateLoadUserFunction

        visualization: visualization data, see VLoadCoordinate

    Notes:
        Requested Marker type: ``Coordinate``

    """
    def __init__(self, name = '', markerNumber = exudyn.InvalidIndex(), load = 0., loadUserFunction: Union[LoadCoordinateLoadUserFunction, int] = 0, visualization = {}):
        self.name = name
        self.markerNumber = markerNumber
        self.load = load
        self.loadUserFunction = loadUserFunction
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'loadType', 'Coordinate'
        yield 'name', self.name
        yield 'markerNumber', self.markerNumber
        yield 'load', self.load
        yield 'loadUserFunction', self.loadUserFunction

    def __repr__(self):
        return str(dict(self))

#+++++++++++++++++++++++++++++++
#SENSOR
class VSensorNode:
    """Visualization data for SensorNode.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class SensorNode:
    """A sensor attached to a node, which measures one of the output variables of the node.
    
    Args:
        name: sensor's unique name; type: str

        nodeNumber: node number to which sensor is attached to; type: NodeIndex

        writeToFile: True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''; type: bool

        fileName: directory and file name for sensor file output; empty: no file is written; a relative name is placed in ``exudyn.config.outputDirectory`` if that is set; the directory is created if it does not exist; type: str

        outputVariableType: OutputVariableType for sensor

        storeInternal: true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available; type: bool

        visualization: visualization data, see VSensorNode

    """
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), writeToFile = True, fileName = '', outputVariableType = 0, storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.outputVariableType = outputVariableType
        self.storeInternal = storeInternal
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'sensorType', 'Node'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'outputVariableType', self.outputVariableType
        yield 'storeInternal', self.storeInternal
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VSensorObject:
    """Visualization data for SensorObject.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; sensors can be shown at the position assiciated with the object - note that in some cases, there might be no such position (e.g. data object)!; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class SensorObject:
    """A sensor attached to an object other than a body - a connector, a constraint, a joint - which measures one of the output variables of the object; a body is measured at a point, with SensorBody.
    
    Args:
        name: sensor's unique name; type: str

        objectNumber: object (e.g. connector) number to which sensor is attached to; type: ObjectIndex

        writeToFile: True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''; type: bool

        fileName: directory and file name for sensor file output; empty: no file is written; a relative name is placed in ``exudyn.config.outputDirectory`` if that is set; the directory is created if it does not exist; type: str

        outputVariableType: OutputVariableType for sensor

        storeInternal: true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available; type: bool

        visualization: visualization data, see VSensorObject

    """
    def __init__(self, name = '', objectNumber = exudyn.InvalidIndex(), writeToFile = True, fileName = '', outputVariableType = 0, storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.objectNumber = objectNumber
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.outputVariableType = outputVariableType
        self.storeInternal = storeInternal
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'sensorType', 'Object'
        yield 'name', self.name
        yield 'objectNumber', self.objectNumber
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'outputVariableType', self.outputVariableType
        yield 'storeInternal', self.storeInternal
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VSensorBody:
    """Visualization data for SensorBody.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class SensorBody:
    r"""A sensor attached to a body at a local position :math:`{}^{b}{\mathbf{b}}`, which measures one of the output variables of the body at that point.
    
    Args:
        name: sensor's unique name; type: str

        bodyNumber: body (=object) number to which sensor is attached to; type: ObjectIndex

        localPosition: local (body-fixed) body position of sensor; type: [float,float,float]

        writeToFile: True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''; type: bool

        fileName: directory and file name for sensor file output; empty: no file is written; a relative name is placed in ``exudyn.config.outputDirectory`` if that is set; the directory is created if it does not exist; type: str

        outputVariableType: OutputVariableType for sensor

        storeInternal: true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available; type: bool

        visualization: visualization data, see VSensorBody

    """
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), localPosition = [0.,0.,0.], writeToFile = True, fileName = '', outputVariableType = 0, storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.localPosition = np.array(localPosition)
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.outputVariableType = outputVariableType
        self.storeInternal = storeInternal
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'sensorType', 'Body'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'localPosition', self.localPosition
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'outputVariableType', self.outputVariableType
        yield 'storeInternal', self.storeInternal
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VSensorSuperElement:
    """Visualization data for SensorSuperElement.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class SensorSuperElement:
    """A sensor attached to a mesh node of a superelement, which measures one of the output variables of the superelement at that mesh node.
    
    Args:
        name: sensor's unique name; type: str

        bodyNumber: body (=object) number to which sensor is attached to; type: ObjectIndex

        meshNodeNumber: mesh node number, which is a local node number with in the object (starting with 0); the node number may represent a real Node in mbs, or may be virtual and reconstructed from the object coordinates such as in ObjectFFRFreducedOrder; type: int

        writeToFile: True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''; type: bool

        fileName: directory and file name for sensor file output; empty: no file is written; a relative name is placed in ``exudyn.config.outputDirectory`` if that is set; the directory is created if it does not exist; type: str

        outputVariableType: OutputVariableType for sensor, based on the output variables available for the mesh nodes (see special section for super element output variables, e.g, in ObjectFFRFreducedOrder, sec-objectffrfreducedorder-superelementoutput)

        storeInternal: true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available; type: bool

        visualization: visualization data, see VSensorSuperElement

    """
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), meshNodeNumber = exudyn.InvalidIndex(), writeToFile = True, fileName = '', outputVariableType = 0, storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.meshNodeNumber = meshNodeNumber
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.outputVariableType = outputVariableType
        self.storeInternal = storeInternal
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'sensorType', 'SuperElement'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'meshNodeNumber', self.meshNodeNumber
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'outputVariableType', self.outputVariableType
        yield 'storeInternal', self.storeInternal
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VSensorKinematicTree:
    """Visualization data for SensorKinematicTree.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class SensorKinematicTree:
    r"""A sensor attached to a link :math:`n_l` of an ObjectKinematicTree at a local position :math:`{}^{b}{\mathbf{b}}` in the frame of the link, which measures one of the output variables of the kinematic tree at that point.
    
    Args:
        name: sensor's unique name; type: str

        objectNumber: object number of KinematicTree to which sensor is attached to; type: ObjectIndex

        linkNumber: number of link in KinematicTree to measure quantities; type: int

        localPosition: local (link-fixed) position of sensor, defined in link (:math:`n_l`) coordinate system; type: [float,float,float]

        writeToFile: True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''; type: bool

        fileName: directory and file name for sensor file output; empty: no file is written; a relative name is placed in ``exudyn.config.outputDirectory`` if that is set; the directory is created if it does not exist; type: str

        outputVariableType: OutputVariableType for sensor

        storeInternal: true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available; type: bool

        visualization: visualization data, see VSensorKinematicTree

    """
    def __init__(self, name = '', objectNumber = exudyn.InvalidIndex(), linkNumber = exudyn.InvalidIndex(), localPosition = [0.,0.,0.], writeToFile = True, fileName = '', outputVariableType = 0, storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.objectNumber = objectNumber
        self.linkNumber = linkNumber
        self.localPosition = np.array(localPosition)
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.outputVariableType = outputVariableType
        self.storeInternal = storeInternal
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'sensorType', 'KinematicTree'
        yield 'name', self.name
        yield 'objectNumber', self.objectNumber
        yield 'linkNumber', self.linkNumber
        yield 'localPosition', self.localPosition
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'outputVariableType', self.outputVariableType
        yield 'storeInternal', self.storeInternal
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VSensorMarker:
    """Visualization data for SensorMarker.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class SensorMarker:
    """A sensor attached to a marker, which measures what the marker provides, in the current configuration.
    
    Args:
        name: sensor's unique name; type: str

        markerNumber: marker number to which sensor is attached to; type: MarkerIndex

        writeToFile: True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''; type: bool

        fileName: directory and file name for sensor file output; empty: no file is written; a relative name is placed in ``exudyn.config.outputDirectory`` if that is set; the directory is created if it does not exist; type: str

        outputVariableType: OutputVariableType for sensor; output variables are only possible according to markertype, see general description of SensorMarker

        storeInternal: true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available; type: bool

        visualization: visualization data, see VSensorMarker

    """
    def __init__(self, name = '', markerNumber = exudyn.InvalidIndex(), writeToFile = True, fileName = '', outputVariableType = 0, storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.markerNumber = markerNumber
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.outputVariableType = outputVariableType
        self.storeInternal = storeInternal
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'sensorType', 'Marker'
        yield 'name', self.name
        yield 'markerNumber', self.markerNumber
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'outputVariableType', self.outputVariableType
        yield 'storeInternal', self.storeInternal
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VSensorLoad:
    """Visualization data for SensorLoad.
    
    Args:
        show: set true, if item is shown in visualization and false if it is not shown; the sensor is drawn at the position of the marker of its load, if the marker has a position; type: bool

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class SensorLoad:
    """A sensor attached to a load, which measures the value of the load.
    
    Args:
        name: sensor's unique name; type: str

        loadNumber: load number to which sensor is attached to; type: LoadIndex

        writeToFile: True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''; type: bool

        fileName: directory and file name for sensor file output; empty: no file is written; a relative name is placed in ``exudyn.config.outputDirectory`` if that is set; the directory is created if it does not exist; type: str

        storeInternal: true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available; type: bool

        visualization: visualization data, see VSensorLoad

    """
    def __init__(self, name = '', loadNumber = exudyn.InvalidIndex(), writeToFile = True, fileName = '', storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.loadNumber = loadNumber
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.storeInternal = storeInternal
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'sensorType', 'Load'
        yield 'name', self.name
        yield 'loadNumber', self.loadNumber
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'storeInternal', self.storeInternal
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VSensorUserFunction:
    """Visualization data for SensorUserFunction."""
    def __init__(self):
        pass

    def __iter__(self):
        yield from ()

    def __repr__(self):
        return str(dict(self))

class SensorUserFunction:
    """A sensor defined by a user function.
    
    The sensor is intended to collect sensor values of a list of given sensors and recombine the output into a new value for output or control purposes. It is also possible to use this sensor without any dependence on other sensors in order to generate output for, e.g., any quantities in mbs or solvers.
    
    Args:
        name: sensor's unique name; type: str

        sensorNumbers: optional list of :math:`n` sensor numbers for use in user function; type: ArraySensorIndex

        factors: optional list of :math:`m` factors which can be used, e.g., for weighting sensor values; type: array_like

        writeToFile: True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''; type: bool

        fileName: directory and file name for sensor file output; empty: no file is written; a relative name is placed in ``exudyn.config.outputDirectory`` if that is set; the directory is created if it does not exist; type: str

        sensorUserFunction: A Python function which defines the time-dependent user function, which usually evaluates one or several sensors and computes a new sensor value, see example; type: SensorUserFunctionSensorUserFunction

        storeInternal: true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available; type: bool

        visualization: visualization data, see VSensorUserFunction

    """
    def __init__(self, name = '', sensorNumbers = [], factors = [], writeToFile = True, fileName = '', sensorUserFunction: Union[SensorUserFunctionSensorUserFunction, int] = 0, storeInternal = False, visualization = {}):
        self.name = name
        self.sensorNumbers = copy.copy(sensorNumbers)
        self.factors = np.array(factors)
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.sensorUserFunction = sensorUserFunction
        self.storeInternal = storeInternal
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'sensorType', 'UserFunction'
        yield 'name', self.name
        yield 'sensorNumbers', self.sensorNumbers
        yield 'factors', self.factors
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'sensorUserFunction', self.sensorUserFunction
        yield 'storeInternal', self.storeInternal

    def __repr__(self):
        return str(dict(self))

