#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example 
# 
# Details:  automatically generated file for conversion of item (node, object, marker, ...) data to dictionaries
# 
# Author:   Johannes Gerstmayr
# Date:     2019-07-01
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#item interface diagonal matrix creator

import exudyn #for exudyn.InvalidIndex() and other exudyn native structures needed in RigidBodySpringDamper
import numpy as np
import copy 

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

userFunctionArgsDict = {'MainSystem,preStepUserFunction': [['MainSystem', 'Real'], ['mbs', 'arg0'], ['bool']],
        'MainSystem,postStepUserFunction': [['MainSystem', 'Real'], ['mbs', 'arg0'], ['bool']],
        'MainSystem,postNewtonFunction': [['MainSystem', 'Real'], ['mbs', 'arg0'], ['StdVector2D']],
        'ObjectGround,graphicsDataUserFunction': [['MainSystem', 'Index'], ['mbs', 'arg0'], ['py::object']],
        'ObjectRigidBody,graphicsDataUserFunction': [['MainSystem', 'Index'], ['mbs', 'arg0'], ['py::object']],
        'ObjectRigidBody2D,graphicsDataUserFunction': [['MainSystem', 'Index'], ['mbs', 'arg0'], ['py::object']],
        'ObjectGenericODE2,forceUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3'], ['StdVector']],
        'ObjectGenericODE2,massMatrixUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3'], ['py::object']],
        'ObjectGenericODE2,jacobianUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector', 'Real', 'Real'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3', 'arg4', 'arg5'], ['py::object']],
        'ObjectGenericODE2,graphicsDataUserFunction': [['MainSystem', 'Index'], ['mbs', 'arg0'], ['py::object']],
        'ObjectGenericODE1,rhsUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector'], ['mbs', 'arg0', 'arg1', 'arg2'], ['StdVector']],
        'ObjectKinematicTree,forceUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3'], ['StdVector']],
        'ObjectFFRF,forceUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3'], ['StdVector']],
        'ObjectFFRF,massMatrixUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3'], ['NumpyMatrix']],
        'ObjectFFRFreducedOrder,forceUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3'], ['StdVector']],
        'ObjectFFRFreducedOrder,massMatrixUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3'], ['NumpyMatrix']],
        'ObjectANCFCable2D,axialForceUserFunction': [['MainSystem', 'Real', 'Index', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3', 'arg4', 'arg5', 'arg6', 'arg7', 'arg8', 'arg9', 'arg10'], ['Real']],
        'ObjectANCFCable2D,bendingMomentUserFunction': [['MainSystem', 'Real', 'Index', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3', 'arg4', 'arg5', 'arg6', 'arg7', 'arg8', 'arg9', 'arg10'], ['Real']],
        'ObjectConnectorSpringDamper,springForceUserFunction': [['MainSystem', 'Real', 'Index', 'Real', 'Real', 'Real', 'Real', 'Real'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3', 'arg4', 'arg5', 'arg6'], ['Real']],
        'ObjectConnectorCartesianSpringDamper,springForceUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector3D', 'StdVector3D', 'StdVector3D', 'StdVector3D', 'StdVector3D'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3', 'arg4', 'arg5', 'arg6'], ['StdVector3D']],
        'ObjectConnectorRigidBodySpringDamper,springForceTorqueUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector3D', 'StdVector3D', 'StdVector3D', 'StdVector3D', 'StdMatrix6D', 'StdMatrix6D', 'StdMatrix3D', 'StdMatrix3D', 'StdVector6D'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3', 'arg4', 'arg5', 'arg6', 'arg7', 'arg8', 'arg9', 'arg10'], ['StdVector6D']],
        'ObjectConnectorRigidBodySpringDamper,postNewtonStepUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector3D', 'StdVector3D', 'StdVector3D', 'StdVector3D', 'StdMatrix6D', 'StdMatrix6D', 'StdMatrix3D', 'StdMatrix3D', 'StdVector6D'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3', 'arg4', 'arg5', 'arg6', 'arg7', 'arg8', 'arg9', 'arg10', 'arg11'], ['StdVector']],
        'ObjectConnectorLinearSpringDamper,springForceUserFunction': [['MainSystem', 'Real', 'Index', 'Real', 'Real', 'Real', 'Real', 'Real'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3', 'arg4', 'arg5', 'arg6'], ['Real']],
        'ObjectConnectorTorsionalSpringDamper,springTorqueUserFunction': [['MainSystem', 'Real', 'Index', 'Real', 'Real', 'Real', 'Real', 'Real'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3', 'arg4', 'arg5', 'arg6'], ['Real']],
        'ObjectConnectorCoordinateSpringDamper,springForceUserFunction': [['MainSystem', 'Real', 'Index', 'Real', 'Real', 'Real', 'Real', 'Real'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3', 'arg4', 'arg5', 'arg6'], ['Real']],
        'ObjectConnectorCoordinateSpringDamperExt,springForceUserFunction': [['MainSystem', 'Real', 'Index', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real', 'Real'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3', 'arg4', 'arg5', 'arg6', 'arg7', 'arg8', 'arg9', 'arg10', 'arg11', 'arg12'], ['Real']],
        'ObjectConnectorCoordinate,offsetUserFunction': [['MainSystem', 'Real', 'Index', 'Real'], ['mbs', 'arg0', 'arg1', 'arg2'], ['Real']],
        'ObjectConnectorCoordinate,offsetUserFunction_t': [['MainSystem', 'Real', 'Index', 'Real'], ['mbs', 'arg0', 'arg1', 'arg2'], ['Real']],
        'ObjectConnectorCoordinateVector,constraintUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector', 'bool'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3', 'arg4'], ['StdVector']],
        'ObjectConnectorCoordinateVector,jacobianUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector', 'StdVector', 'bool'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3', 'arg4'], ['py::object']],
        'ObjectJointGeneric,offsetUserFunction': [['MainSystem', 'Real', 'Index', 'StdVector6D'], ['mbs', 'arg0', 'arg1', 'arg2'], ['StdVector6D']],
        'ObjectJointGeneric,offsetUserFunction_t': [['MainSystem', 'Real', 'Index', 'StdVector6D'], ['mbs', 'arg0', 'arg1', 'arg2'], ['StdVector6D']],
        'LoadForceVector,loadVectorUserFunction': [['MainSystem', 'Real', 'StdVector3D'], ['mbs', 'arg0', 'arg1'], ['StdVector3D']],
        'LoadTorqueVector,loadVectorUserFunction': [['MainSystem', 'Real', 'StdVector3D'], ['mbs', 'arg0', 'arg1'], ['StdVector3D']],
        'LoadMassProportional,loadVectorUserFunction': [['MainSystem', 'Real', 'StdVector3D'], ['mbs', 'arg0', 'arg1'], ['StdVector3D']],
        'LoadCoordinate,loadUserFunction': [['MainSystem', 'Real', 'Real'], ['mbs', 'arg0', 'arg1'], ['Real']],
        'SensorUserFunction,sensorUserFunction': [['MainSystem', 'Real', 'StdArrayIndex', 'StdVector', 'ConfigurationType'], ['mbs', 'arg0', 'arg1', 'arg2', 'arg3'], ['StdVector']]}


#+++++++++++++++++++++++++++++++
#NODE
class VNodePoint:
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
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class Node1D:
    def __init__(self, name = '', referenceCoordinates = [0.], initialCoordinates = [0.], initialVelocities = [0.], visualization = {'show': False}):
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
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VNodePoint2DSlope1:
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
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0.,1.,0.,0.,1.,0.,0.], initialCoordinates = [0.,0.,0.,0.,0.,0.,0.,0.,0.], initialVelocities = [0.,0.,0.,0.,0.,0.,0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
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
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0.,1.,0.,0.,1.,0.,0.], initialCoordinates = [0.,0.,0.,0.,0.,0.,0.,0.,0.], initialVelocities = [0.,0.,0.,0.,0.,0.,0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
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
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class NodeGenericODE2:
    def __init__(self, name = '', referenceCoordinates = [], initialCoordinates = [], initialCoordinates_t = [], numberOfODE2Coordinates = 0, visualization = {'show': False}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.initialCoordinates_t = np.array(initialCoordinates_t)
        self.numberOfODE2Coordinates = CheckForValidPInt(numberOfODE2Coordinates,"numberOfODE2Coordinates","NodeGenericODE2")
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'GenericODE2'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialCoordinates_t', self.initialCoordinates_t
        yield 'numberOfODE2Coordinates', self.numberOfODE2Coordinates
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VNodeGenericODE1:
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class NodeGenericODE1:
    def __init__(self, name = '', referenceCoordinates = [], initialCoordinates = [], numberOfODE1Coordinates = 0, visualization = {'show': False}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.numberOfODE1Coordinates = CheckForValidPInt(numberOfODE1Coordinates,"numberOfODE1Coordinates","NodeGenericODE1")
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'GenericODE1'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'numberOfODE1Coordinates', self.numberOfODE1Coordinates
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VNodeGenericAE:
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class NodeGenericAE:
    def __init__(self, name = '', referenceCoordinates = [], initialCoordinates = [], numberOfAECoordinates = 0, visualization = {'show': False}):
        self.name = name
        self.referenceCoordinates = np.array(referenceCoordinates)
        self.initialCoordinates = np.array(initialCoordinates)
        self.numberOfAECoordinates = CheckForValidPInt(numberOfAECoordinates,"numberOfAECoordinates","NodeGenericAE")
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'GenericAE'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'numberOfAECoordinates', self.numberOfAECoordinates
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VNodeGenericData:
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class NodeGenericData:
    def __init__(self, name = '', initialCoordinates = [], numberOfDataCoordinates = 0, visualization = {'show': False}):
        self.name = name
        self.initialCoordinates = np.array(initialCoordinates)
        self.numberOfDataCoordinates = CheckForValidUInt(numberOfDataCoordinates,"numberOfDataCoordinates","NodeGenericData")
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'nodeType', 'GenericData'
        yield 'name', self.name
        yield 'initialCoordinates', self.initialCoordinates
        yield 'numberOfDataCoordinates', self.numberOfDataCoordinates
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VNodePointGround:
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
    def __init__(self, show = True, graphicsDataUserFunction = 0, graphicsData = []):
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
    def __init__(self, name = '', referencePosition = [0.,0.,0.], referenceRotation = IIDiagMatrix(rowsColumns=3,value=1), visualization = {'show': True, 'graphicsDataUserFunction': 0, 'graphicsData': []}):
        self.name = name
        self.referencePosition = np.array(referencePosition)
        self.referenceRotation = np.array(referenceRotation)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'Ground'
        yield 'name', self.name
        yield 'referencePosition', self.referencePosition
        yield 'referenceRotation', self.referenceRotation
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsDataUserFunction', dict(self.visualization)["graphicsDataUserFunction"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

    def __repr__(self):
        return str(dict(self))
class VObjectMassPoint:
    def __init__(self, show = True, graphicsData = []):
        self.show = show
        self.graphicsData = copy.copy(graphicsData)

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))
class ObjectMassPoint:
    def __init__(self, name = '', physicsMass = 0., nodeNumber = exudyn.InvalidIndex(), visualization = {'show': True, 'graphicsData': []}):
        self.name = name
        self.physicsMass = CheckForValidUReal(physicsMass,"physicsMass","ObjectMassPoint")
        self.nodeNumber = nodeNumber
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'MassPoint'
        yield 'name', self.name
        yield 'physicsMass', self.physicsMass
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
MassPoint = ObjectMassPoint
VMassPoint = VObjectMassPoint

class VObjectMassPoint2D:
    def __init__(self, show = True, graphicsData = []):
        self.show = show
        self.graphicsData = copy.copy(graphicsData)

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))
class ObjectMassPoint2D:
    def __init__(self, name = '', physicsMass = 0., nodeNumber = exudyn.InvalidIndex(), visualization = {'show': True, 'graphicsData': []}):
        self.name = name
        self.physicsMass = CheckForValidUReal(physicsMass,"physicsMass","ObjectMassPoint2D")
        self.nodeNumber = nodeNumber
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'MassPoint2D'
        yield 'name', self.name
        yield 'physicsMass', self.physicsMass
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
MassPoint2D = ObjectMassPoint2D
VMassPoint2D = VObjectMassPoint2D

class VObjectMass1D:
    def __init__(self, show = True, graphicsData = []):
        self.show = show
        self.graphicsData = copy.copy(graphicsData)

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))
class ObjectMass1D:
    def __init__(self, name = '', physicsMass = 0., nodeNumber = exudyn.InvalidIndex(), referencePosition = [0.,0.,0.], referenceRotation = IIDiagMatrix(rowsColumns=3,value=1), visualization = {'show': True, 'graphicsData': []}):
        self.name = name
        self.physicsMass = CheckForValidUReal(physicsMass,"physicsMass","ObjectMass1D")
        self.nodeNumber = nodeNumber
        self.referencePosition = np.array(referencePosition)
        self.referenceRotation = np.array(referenceRotation)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'Mass1D'
        yield 'name', self.name
        yield 'physicsMass', self.physicsMass
        yield 'nodeNumber', self.nodeNumber
        yield 'referencePosition', self.referencePosition
        yield 'referenceRotation', self.referenceRotation
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Mass1D = ObjectMass1D
VMass1D = VObjectMass1D

class VObjectRotationalMass1D:
    def __init__(self, show = True, graphicsData = []):
        self.show = show
        self.graphicsData = copy.copy(graphicsData)

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))
class ObjectRotationalMass1D:
    def __init__(self, name = '', physicsInertia = 0., nodeNumber = exudyn.InvalidIndex(), referencePosition = [0.,0.,0.], referenceRotation = IIDiagMatrix(rowsColumns=3,value=1), visualization = {'show': True, 'graphicsData': []}):
        self.name = name
        self.physicsInertia = CheckForValidUReal(physicsInertia,"physicsInertia","ObjectRotationalMass1D")
        self.nodeNumber = nodeNumber
        self.referencePosition = np.array(referencePosition)
        self.referenceRotation = np.array(referenceRotation)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'RotationalMass1D'
        yield 'name', self.name
        yield 'physicsInertia', self.physicsInertia
        yield 'nodeNumber', self.nodeNumber
        yield 'referencePosition', self.referencePosition
        yield 'referenceRotation', self.referenceRotation
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Rotor1D = ObjectRotationalMass1D
VRotor1D = VObjectRotationalMass1D

class VObjectRigidBody:
    def __init__(self, show = True, graphicsDataUserFunction = 0, graphicsData = []):
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
    def __init__(self, name = '', physicsMass = 0., physicsInertia = [0.,0.,0., 0.,0.,0.], physicsCenterOfMass = [0.,0.,0.], nodeNumber = exudyn.InvalidIndex(), visualization = {'show': True, 'graphicsDataUserFunction': 0, 'graphicsData': []}):
        self.name = name
        self.physicsMass = CheckForValidUReal(physicsMass,"physicsMass","ObjectRigidBody")
        self.physicsInertia = np.array(physicsInertia)
        self.physicsCenterOfMass = np.array(physicsCenterOfMass)
        self.nodeNumber = nodeNumber
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'RigidBody'
        yield 'name', self.name
        yield 'physicsMass', self.physicsMass
        yield 'physicsInertia', self.physicsInertia
        yield 'physicsCenterOfMass', self.physicsCenterOfMass
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsDataUserFunction', dict(self.visualization)["graphicsDataUserFunction"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
RigidBody = ObjectRigidBody
VRigidBody = VObjectRigidBody

class VObjectRigidBody2D:
    def __init__(self, show = True, graphicsDataUserFunction = 0, graphicsData = []):
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
    def __init__(self, name = '', physicsMass = 0., physicsInertia = 0., nodeNumber = exudyn.InvalidIndex(), visualization = {'show': True, 'graphicsDataUserFunction': 0, 'graphicsData': []}):
        self.name = name
        self.physicsMass = CheckForValidUReal(physicsMass,"physicsMass","ObjectRigidBody2D")
        self.physicsInertia = CheckForValidUReal(physicsInertia,"physicsInertia","ObjectRigidBody2D")
        self.nodeNumber = nodeNumber
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'RigidBody2D'
        yield 'name', self.name
        yield 'physicsMass', self.physicsMass
        yield 'physicsInertia', self.physicsInertia
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsDataUserFunction', dict(self.visualization)["graphicsDataUserFunction"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
RigidBody2D = ObjectRigidBody2D
VRigidBody2D = VObjectRigidBody2D

class VObjectGenericODE2:
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.], triangleMesh = [], showNodes = False, graphicsDataUserFunction = 0):
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
    def __init__(self, name = '', nodeNumbers = [], massMatrix = None, stiffnessMatrix = None, dampingMatrix = None, forceVector = [], forceUserFunction = 0, massMatrixUserFunction = 0, jacobianUserFunction = 0, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.], 'triangleMesh': [], 'showNodes': False, 'graphicsDataUserFunction': 0}):
        self.name = name
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.massMatrix = massMatrix
        self.stiffnessMatrix = stiffnessMatrix
        self.dampingMatrix = dampingMatrix
        self.forceVector = np.array(forceVector)
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
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class ObjectGenericODE1:
    def __init__(self, name = '', nodeNumbers = [], systemMatrix = [], rhsVector = [], rhsUserFunction = 0, visualization = {'show': True}):
        self.name = name
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.systemMatrix = np.array(systemMatrix)
        self.rhsVector = np.array(rhsVector)
        self.rhsUserFunction = rhsUserFunction
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'GenericODE1'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'systemMatrix', self.systemMatrix
        yield 'rhsVector', self.rhsVector
        yield 'rhsUserFunction', self.rhsUserFunction
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VObjectKinematicTree:
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
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), gravity = [0.,0.,0.], baseOffset = [0.,0.,0.], jointTypes = [], linkParents = [], jointTransformations = None, jointOffsets = None, linkInertiasCOM = None, linkCOMs = None, linkMasses = [], linkForces = None, linkTorques = None, jointForceVector = [], jointPositionOffsetVector = [], jointVelocityOffsetVector = [], jointPControlVector = [], jointDControlVector = [], forceUserFunction = 0, visualization = {'show': True, 'showLinks': True, 'showJoints': True, 'color': [-1.,-1.,-1.,-1.], 'graphicsDataList': []}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.gravity = np.array(gravity)
        self.baseOffset = np.array(baseOffset)
        self.jointTypes = copy.copy(jointTypes)
        self.linkParents = copy.copy(linkParents)
        self.jointTransformations = jointTransformations
        self.jointOffsets = jointOffsets
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
    def __init__(self, name = '', nodeNumbers = [], massMatrixFF = None, stiffnessMatrixFF = None, dampingMatrixFF = None, forceVector = [], forceUserFunction = 0, massMatrixUserFunction = 0, computeFFRFterms = True, objectIsInitialized = False, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.], 'triangleMesh': [], 'showNodes': False}):
        self.name = name
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.massMatrixFF = massMatrixFF
        self.stiffnessMatrixFF = stiffnessMatrixFF
        self.dampingMatrixFF = dampingMatrixFF
        self.forceVector = np.array(forceVector)
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
    def __init__(self, name = '', nodeNumbers = [], massMatrixReduced = None, stiffnessMatrixReduced = None, dampingMatrixReduced = None, forceUserFunction = 0, massMatrixUserFunction = 0, computeFFRFterms = True, modeBasis = [], outputVariableModeBasis = [], outputVariableTypeModeBasis = 0, referencePositions = [], objectIsInitialized = False, physicsMass = 0., physicsInertia = IIDiagMatrix(rowsColumns=3,value=1), physicsCenterOfMass = [0.,0.,0.], mPsiTildePsi = [], mPsiTildePsiTilde = [], mPhitTPsi = [], mPhitTPsiTilde = [], mXRefTildePsi = [], mXRefTildePsiTilde = [], physicsCenterOfMassTilde = IIDiagMatrix(rowsColumns=3,value=0), visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.], 'triangleMesh': [], 'showNodes': False}):
        self.name = name
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.massMatrixReduced = massMatrixReduced
        self.stiffnessMatrixReduced = stiffnessMatrixReduced
        self.dampingMatrixReduced = dampingMatrixReduced
        self.forceUserFunction = forceUserFunction
        self.massMatrixUserFunction = massMatrixUserFunction
        self.computeFFRFterms = computeFFRFterms
        self.modeBasis = np.array(modeBasis)
        self.outputVariableModeBasis = np.array(outputVariableModeBasis)
        self.outputVariableTypeModeBasis = outputVariableTypeModeBasis
        self.referencePositions = np.array(referencePositions)
        self.objectIsInitialized = objectIsInitialized
        self.physicsMass = CheckForValidUReal(physicsMass,"physicsMass","ObjectFFRFreducedOrder")
        self.physicsInertia = np.array(physicsInertia)
        self.physicsCenterOfMass = np.array(physicsCenterOfMass)
        self.mPsiTildePsi = np.array(mPsiTildePsi)
        self.mPsiTildePsiTilde = np.array(mPsiTildePsiTilde)
        self.mPhitTPsi = np.array(mPhitTPsi)
        self.mPhitTPsiTilde = np.array(mPhitTPsiTilde)
        self.mXRefTildePsi = np.array(mXRefTildePsi)
        self.mXRefTildePsiTilde = np.array(mXRefTildePsiTilde)
        self.physicsCenterOfMassTilde = np.array(physicsCenterOfMassTilde)
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
        yield 'physicsMass', self.physicsMass
        yield 'physicsInertia', self.physicsInertia
        yield 'physicsCenterOfMass', self.physicsCenterOfMass
        yield 'mPsiTildePsi', self.mPsiTildePsi
        yield 'mPsiTildePsiTilde', self.mPsiTildePsiTilde
        yield 'mPhitTPsi', self.mPhitTPsi
        yield 'mPhitTPsiTilde', self.mPhitTPsiTilde
        yield 'mXRefTildePsi', self.mXRefTildePsi
        yield 'mXRefTildePsiTilde', self.mXRefTildePsiTilde
        yield 'physicsCenterOfMassTilde', self.physicsCenterOfMassTilde
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]
        yield 'VtriangleMesh', dict(self.visualization)["triangleMesh"]
        yield 'VshowNodes', dict(self.visualization)["showNodes"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
CMSobject = ObjectFFRFreducedOrder
VCMSobject = VObjectFFRFreducedOrder

class VObjectANCFCable:
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
    def __init__(self, name = '', physicsLength = 0., physicsMassPerLength = 0., physicsBendingStiffness = 0., physicsAxialStiffness = 0., physicsBendingDamping = 0., physicsAxialDamping = 0., physicsReferenceAxialStrain = 0., strainIsRelativeToReference = 0., nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex()], useReducedOrderIntegration = 0, visualization = {'show': True, 'radius': 0., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.physicsLength = CheckForValidUReal(physicsLength,"physicsLength","ObjectANCFCable")
        self.physicsMassPerLength = CheckForValidUReal(physicsMassPerLength,"physicsMassPerLength","ObjectANCFCable")
        self.physicsBendingStiffness = CheckForValidUReal(physicsBendingStiffness,"physicsBendingStiffness","ObjectANCFCable")
        self.physicsAxialStiffness = CheckForValidUReal(physicsAxialStiffness,"physicsAxialStiffness","ObjectANCFCable")
        self.physicsBendingDamping = CheckForValidUReal(physicsBendingDamping,"physicsBendingDamping","ObjectANCFCable")
        self.physicsAxialDamping = CheckForValidUReal(physicsAxialDamping,"physicsAxialDamping","ObjectANCFCable")
        self.physicsReferenceAxialStrain = physicsReferenceAxialStrain
        self.strainIsRelativeToReference = strainIsRelativeToReference
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.useReducedOrderIntegration = useReducedOrderIntegration
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ANCFCable'
        yield 'name', self.name
        yield 'physicsLength', self.physicsLength
        yield 'physicsMassPerLength', self.physicsMassPerLength
        yield 'physicsBendingStiffness', self.physicsBendingStiffness
        yield 'physicsAxialStiffness', self.physicsAxialStiffness
        yield 'physicsBendingDamping', self.physicsBendingDamping
        yield 'physicsAxialDamping', self.physicsAxialDamping
        yield 'physicsReferenceAxialStrain', self.physicsReferenceAxialStrain
        yield 'strainIsRelativeToReference', self.strainIsRelativeToReference
        yield 'nodeNumbers', self.nodeNumbers
        yield 'useReducedOrderIntegration', self.useReducedOrderIntegration
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vradius', dict(self.visualization)["radius"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Cable = ObjectANCFCable
VCable = VObjectANCFCable

class VObjectANCFCable2D:
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
    def __init__(self, name = '', physicsLength = 0., physicsMassPerLength = 0., physicsBendingStiffness = 0., physicsAxialStiffness = 0., physicsBendingDamping = 0., physicsAxialDamping = 0., physicsReferenceAxialStrain = 0., physicsReferenceCurvature = 0., strainIsRelativeToReference = 0., nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex()], useReducedOrderIntegration = 0, axialForceUserFunction = 0, bendingMomentUserFunction = 0, visualization = {'show': True, 'drawHeight': 0., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.physicsLength = CheckForValidUReal(physicsLength,"physicsLength","ObjectANCFCable2D")
        self.physicsMassPerLength = CheckForValidUReal(physicsMassPerLength,"physicsMassPerLength","ObjectANCFCable2D")
        self.physicsBendingStiffness = CheckForValidUReal(physicsBendingStiffness,"physicsBendingStiffness","ObjectANCFCable2D")
        self.physicsAxialStiffness = CheckForValidUReal(physicsAxialStiffness,"physicsAxialStiffness","ObjectANCFCable2D")
        self.physicsBendingDamping = CheckForValidUReal(physicsBendingDamping,"physicsBendingDamping","ObjectANCFCable2D")
        self.physicsAxialDamping = CheckForValidUReal(physicsAxialDamping,"physicsAxialDamping","ObjectANCFCable2D")
        self.physicsReferenceAxialStrain = physicsReferenceAxialStrain
        self.physicsReferenceCurvature = physicsReferenceCurvature
        self.strainIsRelativeToReference = strainIsRelativeToReference
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.useReducedOrderIntegration = useReducedOrderIntegration
        self.axialForceUserFunction = axialForceUserFunction
        self.bendingMomentUserFunction = bendingMomentUserFunction
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ANCFCable2D'
        yield 'name', self.name
        yield 'physicsLength', self.physicsLength
        yield 'physicsMassPerLength', self.physicsMassPerLength
        yield 'physicsBendingStiffness', self.physicsBendingStiffness
        yield 'physicsAxialStiffness', self.physicsAxialStiffness
        yield 'physicsBendingDamping', self.physicsBendingDamping
        yield 'physicsAxialDamping', self.physicsAxialDamping
        yield 'physicsReferenceAxialStrain', self.physicsReferenceAxialStrain
        yield 'physicsReferenceCurvature', self.physicsReferenceCurvature
        yield 'strainIsRelativeToReference', self.strainIsRelativeToReference
        yield 'nodeNumbers', self.nodeNumbers
        yield 'useReducedOrderIntegration', self.useReducedOrderIntegration
        yield 'axialForceUserFunction', self.axialForceUserFunction
        yield 'bendingMomentUserFunction', self.bendingMomentUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawHeight', dict(self.visualization)["drawHeight"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Cable2D = ObjectANCFCable2D
VCable2D = VObjectANCFCable2D

class VObjectALEANCFCable2D:
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
    def __init__(self, name = '', physicsLength = 0., physicsMassPerLength = 0., physicsMovingMassFactor = 1., physicsBendingStiffness = 0., physicsAxialStiffness = 0., physicsBendingDamping = 0., physicsAxialDamping = 0., physicsReferenceAxialStrain = 0., physicsReferenceCurvature = 0., physicsUseCouplingTerms = True, physicsAddALEvariation = True, nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex(), exudyn.InvalidIndex()], useReducedOrderIntegration = 0, strainIsRelativeToReference = 0., visualization = {'show': True, 'drawHeight': 0., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.physicsLength = CheckForValidUReal(physicsLength,"physicsLength","ObjectALEANCFCable2D")
        self.physicsMassPerLength = CheckForValidUReal(physicsMassPerLength,"physicsMassPerLength","ObjectALEANCFCable2D")
        self.physicsMovingMassFactor = CheckForValidUReal(physicsMovingMassFactor,"physicsMovingMassFactor","ObjectALEANCFCable2D")
        self.physicsBendingStiffness = CheckForValidUReal(physicsBendingStiffness,"physicsBendingStiffness","ObjectALEANCFCable2D")
        self.physicsAxialStiffness = CheckForValidUReal(physicsAxialStiffness,"physicsAxialStiffness","ObjectALEANCFCable2D")
        self.physicsBendingDamping = CheckForValidUReal(physicsBendingDamping,"physicsBendingDamping","ObjectALEANCFCable2D")
        self.physicsAxialDamping = CheckForValidUReal(physicsAxialDamping,"physicsAxialDamping","ObjectALEANCFCable2D")
        self.physicsReferenceAxialStrain = physicsReferenceAxialStrain
        self.physicsReferenceCurvature = physicsReferenceCurvature
        self.physicsUseCouplingTerms = physicsUseCouplingTerms
        self.physicsAddALEvariation = physicsAddALEvariation
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.useReducedOrderIntegration = useReducedOrderIntegration
        self.strainIsRelativeToReference = strainIsRelativeToReference
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ALEANCFCable2D'
        yield 'name', self.name
        yield 'physicsLength', self.physicsLength
        yield 'physicsMassPerLength', self.physicsMassPerLength
        yield 'physicsMovingMassFactor', self.physicsMovingMassFactor
        yield 'physicsBendingStiffness', self.physicsBendingStiffness
        yield 'physicsAxialStiffness', self.physicsAxialStiffness
        yield 'physicsBendingDamping', self.physicsBendingDamping
        yield 'physicsAxialDamping', self.physicsAxialDamping
        yield 'physicsReferenceAxialStrain', self.physicsReferenceAxialStrain
        yield 'physicsReferenceCurvature', self.physicsReferenceCurvature
        yield 'physicsUseCouplingTerms', self.physicsUseCouplingTerms
        yield 'physicsAddALEvariation', self.physicsAddALEvariation
        yield 'nodeNumbers', self.nodeNumbers
        yield 'useReducedOrderIntegration', self.useReducedOrderIntegration
        yield 'strainIsRelativeToReference', self.strainIsRelativeToReference
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawHeight', dict(self.visualization)["drawHeight"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
ALECable2D = ObjectALEANCFCable2D
VALECable2D = VObjectALEANCFCable2D

class VObjectANCFBeam:
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
    def __init__(self, name = '', nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex()], physicsLength = 0., sectionData = exudyn.BeamSection(), crossSectionPenaltyFactor = [1.,1.,1.], visualization = {'show': True, 'sectionGeometry': exudyn.BeamSectionGeometry(), 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.physicsLength = CheckForValidPReal(physicsLength,"physicsLength","ObjectANCFBeam")
        self.sectionData = sectionData
        self.crossSectionPenaltyFactor = np.array(crossSectionPenaltyFactor)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ANCFBeam'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'physicsLength', self.physicsLength
        yield 'sectionData', self.sectionData
        yield 'crossSectionPenaltyFactor', self.crossSectionPenaltyFactor
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VsectionGeometry', dict(self.visualization)["sectionGeometry"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
ANCFBeam = ObjectANCFBeam
VANCFBeam = VObjectANCFBeam

class VObjectBeamGeometricallyExact2D:
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
    def __init__(self, name = '', nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex()], physicsLength = 0., physicsMassPerLength = 0., physicsCrossSectionInertia = 0., physicsBendingStiffness = 0., physicsAxialStiffness = 0., physicsShearStiffness = 0., physicsBendingDamping = 0., physicsAxialDamping = 0., physicsShearDamping = 0., physicsReferenceCurvature = 0., includeReferenceRotations = False, visualization = {'show': True, 'drawHeight': 0., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.physicsLength = CheckForValidUReal(physicsLength,"physicsLength","ObjectBeamGeometricallyExact2D")
        self.physicsMassPerLength = CheckForValidUReal(physicsMassPerLength,"physicsMassPerLength","ObjectBeamGeometricallyExact2D")
        self.physicsCrossSectionInertia = CheckForValidUReal(physicsCrossSectionInertia,"physicsCrossSectionInertia","ObjectBeamGeometricallyExact2D")
        self.physicsBendingStiffness = CheckForValidUReal(physicsBendingStiffness,"physicsBendingStiffness","ObjectBeamGeometricallyExact2D")
        self.physicsAxialStiffness = CheckForValidUReal(physicsAxialStiffness,"physicsAxialStiffness","ObjectBeamGeometricallyExact2D")
        self.physicsShearStiffness = CheckForValidUReal(physicsShearStiffness,"physicsShearStiffness","ObjectBeamGeometricallyExact2D")
        self.physicsBendingDamping = CheckForValidUReal(physicsBendingDamping,"physicsBendingDamping","ObjectBeamGeometricallyExact2D")
        self.physicsAxialDamping = CheckForValidUReal(physicsAxialDamping,"physicsAxialDamping","ObjectBeamGeometricallyExact2D")
        self.physicsShearDamping = CheckForValidUReal(physicsShearDamping,"physicsShearDamping","ObjectBeamGeometricallyExact2D")
        self.physicsReferenceCurvature = physicsReferenceCurvature
        self.includeReferenceRotations = includeReferenceRotations
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'BeamGeometricallyExact2D'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'physicsLength', self.physicsLength
        yield 'physicsMassPerLength', self.physicsMassPerLength
        yield 'physicsCrossSectionInertia', self.physicsCrossSectionInertia
        yield 'physicsBendingStiffness', self.physicsBendingStiffness
        yield 'physicsAxialStiffness', self.physicsAxialStiffness
        yield 'physicsShearStiffness', self.physicsShearStiffness
        yield 'physicsBendingDamping', self.physicsBendingDamping
        yield 'physicsAxialDamping', self.physicsAxialDamping
        yield 'physicsShearDamping', self.physicsShearDamping
        yield 'physicsReferenceCurvature', self.physicsReferenceCurvature
        yield 'includeReferenceRotations', self.includeReferenceRotations
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawHeight', dict(self.visualization)["drawHeight"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Beam2D = ObjectBeamGeometricallyExact2D
VBeam2D = VObjectBeamGeometricallyExact2D

class VObjectBeamGeometricallyExact:
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
    def __init__(self, name = '', nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex()], physicsLength = 0., sectionData = exudyn.BeamSection(), visualization = {'show': True, 'sectionGeometry': exudyn.BeamSectionGeometry(), 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.physicsLength = CheckForValidPReal(physicsLength,"physicsLength","ObjectBeamGeometricallyExact")
        self.sectionData = sectionData
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'BeamGeometricallyExact'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'physicsLength', self.physicsLength
        yield 'sectionData', self.sectionData
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VsectionGeometry', dict(self.visualization)["sectionGeometry"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Beam3D = ObjectBeamGeometricallyExact
VBeam3D = VObjectBeamGeometricallyExact

class VObjectANCFThinPlate:
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectANCFThinPlate:
    def __init__(self, name = '', physicsThickness = 0., physicsDensity = 0., physicsStrainCoefficients = IIDiagMatrix(rowsColumns=3,value=1), physicsCurvatureCoefficients = IIDiagMatrix(rowsColumns=3,value=1), strainIsRelativeToReference = 1., nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex(), exudyn.InvalidIndex(), exudyn.InvalidIndex()], useReducedOrderIntegration = 0, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.physicsThickness = CheckForValidUReal(physicsThickness,"physicsThickness","ObjectANCFThinPlate")
        self.physicsDensity = CheckForValidUReal(physicsDensity,"physicsDensity","ObjectANCFThinPlate")
        self.physicsStrainCoefficients = np.array(physicsStrainCoefficients)
        self.physicsCurvatureCoefficients = np.array(physicsCurvatureCoefficients)
        self.strainIsRelativeToReference = strainIsRelativeToReference
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.useReducedOrderIntegration = useReducedOrderIntegration
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ANCFThinPlate'
        yield 'name', self.name
        yield 'physicsThickness', self.physicsThickness
        yield 'physicsDensity', self.physicsDensity
        yield 'physicsStrainCoefficients', self.physicsStrainCoefficients
        yield 'physicsCurvatureCoefficients', self.physicsCurvatureCoefficients
        yield 'strainIsRelativeToReference', self.strainIsRelativeToReference
        yield 'nodeNumbers', self.nodeNumbers
        yield 'useReducedOrderIntegration', self.useReducedOrderIntegration
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
class VObjectConnectorSpringDamper:
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], referenceLength = 0., stiffness = 0., damping = 0., force = 0., velocityOffset = 0., activeConnector = True, springForceUserFunction = 0, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.referenceLength = CheckForValidUReal(referenceLength,"referenceLength","ObjectConnectorSpringDamper")
        self.stiffness = CheckForValidUReal(stiffness,"stiffness","ObjectConnectorSpringDamper")
        self.damping = CheckForValidUReal(damping,"damping","ObjectConnectorSpringDamper")
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], stiffness = [0.,0.,0.], damping = [0.,0.,0.], offset = [0.,0.,0.], springForceUserFunction = 0, activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), stiffness = IIDiagMatrix(rowsColumns=6,value=0.), damping = IIDiagMatrix(rowsColumns=6,value=0.), rotationMarker0 = IIDiagMatrix(rowsColumns=3,value=1), rotationMarker1 = IIDiagMatrix(rowsColumns=3,value=1), offset = [0.,0.,0.,0.,0.,0.], intrinsicFormulation = False, activeConnector = True, springForceTorqueUserFunction = 0, postNewtonStepUserFunction = 0, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.stiffness = np.array(stiffness)
        self.damping = np.array(damping)
        self.rotationMarker0 = np.array(rotationMarker0)
        self.rotationMarker1 = np.array(rotationMarker1)
        self.offset = np.array(offset)
        self.intrinsicFormulation = intrinsicFormulation
        self.activeConnector = activeConnector
        self.springForceTorqueUserFunction = springForceTorqueUserFunction
        self.postNewtonStepUserFunction = postNewtonStepUserFunction
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
        yield 'intrinsicFormulation', self.intrinsicFormulation
        yield 'activeConnector', self.activeConnector
        yield 'springForceTorqueUserFunction', self.springForceTorqueUserFunction
        yield 'postNewtonStepUserFunction', self.postNewtonStepUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
RigidBodySpringDamper = ObjectConnectorRigidBodySpringDamper
VRigidBodySpringDamper = VObjectConnectorRigidBodySpringDamper

class VObjectConnectorLinearSpringDamper:
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], stiffness = 0., damping = 0., axisMarker0 = [1,0,0], offset = 0., velocityOffset = 0., force = 0., activeConnector = True, springForceUserFunction = 0, visualization = {'show': True, 'drawSize': -1., 'drawAsCylinder': False, 'color': [-1.,-1.,-1.,-1.]}):
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), stiffness = 0., damping = 0., rotationMarker0 = IIDiagMatrix(rowsColumns=3,value=1), rotationMarker1 = IIDiagMatrix(rowsColumns=3,value=1), offset = 0., velocityOffset = 0., torque = 0., activeConnector = True, springTorqueUserFunction = 0, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], stiffness = 0., damping = 0., offset = 0., activeConnector = True, springForceUserFunction = 0, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), stiffness = 0., damping = 0., offset = 0., velocityOffset = 0., factor0 = 1., factor1 = 1., fDynamicFriction = 0., fStaticFrictionOffset = 0., stickingStiffness = 0., stickingDamping = 0., exponentialDecayStatic = 1.e-3, fViscousFriction = 0., frictionProportionalZone = 0., limitStopsUpper = 0., limitStopsLower = 0., limitStopsStiffness = 0., limitStopsDamping = 0., useLimitStops = False, activeConnector = True, springForceUserFunction = 0, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.stiffness = stiffness
        self.damping = damping
        self.offset = offset
        self.velocityOffset = velocityOffset
        self.factor0 = factor0
        self.factor1 = factor1
        self.fDynamicFriction = CheckForValidUReal(fDynamicFriction,"fDynamicFriction","ObjectConnectorCoordinateSpringDamperExt")
        self.fStaticFrictionOffset = CheckForValidUReal(fStaticFrictionOffset,"fStaticFrictionOffset","ObjectConnectorCoordinateSpringDamperExt")
        self.stickingStiffness = CheckForValidUReal(stickingStiffness,"stickingStiffness","ObjectConnectorCoordinateSpringDamperExt")
        self.stickingDamping = CheckForValidUReal(stickingDamping,"stickingDamping","ObjectConnectorCoordinateSpringDamperExt")
        self.exponentialDecayStatic = CheckForValidPReal(exponentialDecayStatic,"exponentialDecayStatic","ObjectConnectorCoordinateSpringDamperExt")
        self.fViscousFriction = fViscousFriction
        self.frictionProportionalZone = CheckForValidUReal(frictionProportionalZone,"frictionProportionalZone","ObjectConnectorCoordinateSpringDamperExt")
        self.limitStopsUpper = limitStopsUpper
        self.limitStopsLower = limitStopsLower
        self.limitStopsStiffness = CheckForValidUReal(limitStopsStiffness,"limitStopsStiffness","ObjectConnectorCoordinateSpringDamperExt")
        self.limitStopsDamping = CheckForValidUReal(limitStopsDamping,"limitStopsDamping","ObjectConnectorCoordinateSpringDamperExt")
        self.useLimitStops = useLimitStops
        self.activeConnector = activeConnector
        self.springForceUserFunction = springForceUserFunction
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
        yield 'fDynamicFriction', self.fDynamicFriction
        yield 'fStaticFrictionOffset', self.fStaticFrictionOffset
        yield 'stickingStiffness', self.stickingStiffness
        yield 'stickingDamping', self.stickingDamping
        yield 'exponentialDecayStatic', self.exponentialDecayStatic
        yield 'fViscousFriction', self.fViscousFriction
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

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
CoordinateSpringDamperExt = ObjectConnectorCoordinateSpringDamperExt
VCoordinateSpringDamperExt = VObjectConnectorCoordinateSpringDamperExt

class VObjectConnectorGravity:
    def __init__(self, show = False, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectConnectorGravity:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], gravitationalConstant = 6.67430e-11, mass0 = 0., mass1 = 0., minDistanceRegularization = 0., activeConnector = True, visualization = {'show': False, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.gravitationalConstant = gravitationalConstant
        self.mass0 = CheckForValidUReal(mass0,"mass0","ObjectConnectorGravity")
        self.mass1 = CheckForValidUReal(mass1,"mass1","ObjectConnectorGravity")
        self.minDistanceRegularization = CheckForValidUReal(minDistanceRegularization,"minDistanceRegularization","ObjectConnectorGravity")
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
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
ConnectorGravity = ObjectConnectorGravity
VConnectorGravity = VObjectConnectorGravity

class VObjectConnectorHydraulicActuatorSimple:
    def __init__(self, show = True, cylinderRadius = 0.05, rodRadius = 0.03, pistonRadius = 0.04, pistonLength = 0.001, rodMountRadius = 0.0, baseMountRadius = 0.0, baseMountLength = 0.0, colorCylinder = [-1.,-1.,-1.,-1.], colorPiston = [0.8,0.8,0.8,1.]):
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumbers = [], offsetLength = 0., strokeLength = 0., chamberCrossSection0 = 0., chamberCrossSection1 = 0., hoseVolume0 = 0., hoseVolume1 = 0., valveOpening0 = 0., valveOpening1 = 0., actuatorDamping = 0., oilBulkModulus = 0., cylinderBulkModulus = 0., hoseBulkModulus = 0., nominalFlow = 0., systemPressure = 0., tankPressure = 0., useChamberVolumeChange = False, activeConnector = True, visualization = {'show': True, 'cylinderRadius': 0.05, 'rodRadius': 0.03, 'pistonRadius': 0.04, 'pistonLength': 0.001, 'rodMountRadius': 0.0, 'baseMountRadius': 0.0, 'baseMountLength': 0.0, 'colorCylinder': [-1.,-1.,-1.,-1.], 'colorPiston': [0.8,0.8,0.8,1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.offsetLength = CheckForValidUReal(offsetLength,"offsetLength","ObjectConnectorHydraulicActuatorSimple")
        self.strokeLength = CheckForValidPReal(strokeLength,"strokeLength","ObjectConnectorHydraulicActuatorSimple")
        self.chamberCrossSection0 = CheckForValidPReal(chamberCrossSection0,"chamberCrossSection0","ObjectConnectorHydraulicActuatorSimple")
        self.chamberCrossSection1 = CheckForValidPReal(chamberCrossSection1,"chamberCrossSection1","ObjectConnectorHydraulicActuatorSimple")
        self.hoseVolume0 = CheckForValidPReal(hoseVolume0,"hoseVolume0","ObjectConnectorHydraulicActuatorSimple")
        self.hoseVolume1 = CheckForValidPReal(hoseVolume1,"hoseVolume1","ObjectConnectorHydraulicActuatorSimple")
        self.valveOpening0 = valveOpening0
        self.valveOpening1 = valveOpening1
        self.actuatorDamping = CheckForValidUReal(actuatorDamping,"actuatorDamping","ObjectConnectorHydraulicActuatorSimple")
        self.oilBulkModulus = CheckForValidPReal(oilBulkModulus,"oilBulkModulus","ObjectConnectorHydraulicActuatorSimple")
        self.cylinderBulkModulus = CheckForValidUReal(cylinderBulkModulus,"cylinderBulkModulus","ObjectConnectorHydraulicActuatorSimple")
        self.hoseBulkModulus = CheckForValidUReal(hoseBulkModulus,"hoseBulkModulus","ObjectConnectorHydraulicActuatorSimple")
        self.nominalFlow = CheckForValidPReal(nominalFlow,"nominalFlow","ObjectConnectorHydraulicActuatorSimple")
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], hasCoordinateMarkers = False, coordinateFactors = [1,1], stiffnessPerLength = 0., dampingPerLength = 0., dampingTorsional = 0., dampingShear = 0., regularizationForce = 0.1, referenceLength = 0., sheavesAxes = None, sheavesRadii = [], activeConnector = True, visualization = {'show': True, 'ropeRadius': 0.001, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.hasCoordinateMarkers = hasCoordinateMarkers
        self.coordinateFactors = np.array(coordinateFactors)
        self.stiffnessPerLength = CheckForValidUReal(stiffnessPerLength,"stiffnessPerLength","ObjectConnectorReevingSystemSprings")
        self.dampingPerLength = CheckForValidUReal(dampingPerLength,"dampingPerLength","ObjectConnectorReevingSystemSprings")
        self.dampingTorsional = CheckForValidUReal(dampingTorsional,"dampingTorsional","ObjectConnectorReevingSystemSprings")
        self.dampingShear = CheckForValidUReal(dampingShear,"dampingShear","ObjectConnectorReevingSystemSprings")
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], distance = 0., activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.distance = CheckForValidPReal(distance,"distance","ObjectConnectorDistance")
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], offset = 0., factorValue1 = 1., velocityLevel = False, offsetUserFunction = 0, offsetUserFunction_t = 0, activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.offset = offset
        self.factorValue1 = factorValue1
        self.velocityLevel = velocityLevel
        self.offsetUserFunction = offsetUserFunction
        self.offsetUserFunction_t = offsetUserFunction_t
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ConnectorCoordinate'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'offset', self.offset
        yield 'factorValue1', self.factorValue1
        yield 'velocityLevel', self.velocityLevel
        yield 'offsetUserFunction', self.offsetUserFunction
        yield 'offsetUserFunction_t', self.offsetUserFunction_t
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
CoordinateConstraint = ObjectConnectorCoordinate
VCoordinateConstraint = VObjectConnectorCoordinate

class VObjectConnectorCoordinateVector:
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectConnectorCoordinateVector:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], scalingMarker0 = [], scalingMarker1 = [], quadraticTermMarker0 = [], quadraticTermMarker1 = [], offset = [], velocityLevel = False, constraintUserFunction = 0, jacobianUserFunction = 0, activeConnector = True, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.scalingMarker0 = np.array(scalingMarker0)
        self.scalingMarker1 = np.array(scalingMarker1)
        self.quadraticTermMarker0 = np.array(quadraticTermMarker0)
        self.quadraticTermMarker1 = np.array(quadraticTermMarker1)
        self.offset = np.array(offset)
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
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
CoordinateVectorConstraint = ObjectConnectorCoordinateVector
VCoordinateVectorConstraint = VObjectConnectorCoordinateVector

class VObjectConnectorRollingDiscPenalty:
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), discRadius = 0., discAxis = [1,0,0], planeNormal = [0,0,1], dryFrictionAngle = 0., contactStiffness = 0., contactDamping = 0., dryFriction = [0,0], dryFrictionProportionalZone = 0., viscousFriction = [0,0], rollingFrictionViscous = 0., useLinearProportionalZone = False, activeConnector = True, visualization = {'show': True, 'discWidth': 0.1, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.discRadius = CheckForValidPReal(discRadius,"discRadius","ObjectConnectorRollingDiscPenalty")
        self.discAxis = np.array(discAxis)
        self.planeNormal = np.array(planeNormal)
        self.dryFrictionAngle = dryFrictionAngle
        self.contactStiffness = CheckForValidUReal(contactStiffness,"contactStiffness","ObjectConnectorRollingDiscPenalty")
        self.contactDamping = CheckForValidUReal(contactDamping,"contactDamping","ObjectConnectorRollingDiscPenalty")
        self.dryFriction = np.array(dryFriction)
        self.dryFrictionProportionalZone = dryFrictionProportionalZone
        self.viscousFriction = np.array(viscousFriction)
        self.rollingFrictionViscous = rollingFrictionViscous
        self.useLinearProportionalZone = useLinearProportionalZone
        self.activeConnector = activeConnector
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
        yield 'rollingFrictionViscous', self.rollingFrictionViscous
        yield 'useLinearProportionalZone', self.useLinearProportionalZone
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdiscWidth', dict(self.visualization)["discWidth"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
RollingDiscPenalty = ObjectConnectorRollingDiscPenalty
VRollingDiscPenalty = VObjectConnectorRollingDiscPenalty

class VObjectContactConvexRoll:
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectContactConvexRoll:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), contactStiffness = 0., contactDamping = 0., dynamicFriction = 0., staticFrictionOffset = 0., viscousFriction = 0., exponentialDecayStatic = 1e-3, frictionProportionalZone = 1e-3, rollLength = 0., coefficientsHull =  [], rBoundingSphere = 0, activeConnector = True, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.contactStiffness = contactStiffness
        self.contactDamping = contactDamping
        self.dynamicFriction = CheckForValidUReal(dynamicFriction,"dynamicFriction","ObjectContactConvexRoll")
        self.staticFrictionOffset = CheckForValidUReal(staticFrictionOffset,"staticFrictionOffset","ObjectContactConvexRoll")
        self.viscousFriction = CheckForValidUReal(viscousFriction,"viscousFriction","ObjectContactConvexRoll")
        self.exponentialDecayStatic = CheckForValidPReal(exponentialDecayStatic,"exponentialDecayStatic","ObjectContactConvexRoll")
        self.frictionProportionalZone = CheckForValidUReal(frictionProportionalZone,"frictionProportionalZone","ObjectContactConvexRoll")
        self.rollLength = CheckForValidUReal(rollLength,"rollLength","ObjectContactConvexRoll")
        self.coefficientsHull = np.array(coefficientsHull)
        self.rBoundingSphere = CheckForValidUReal(rBoundingSphere,"rBoundingSphere","ObjectContactConvexRoll")
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
        yield 'rBoundingSphere', self.rBoundingSphere
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
class VObjectContactCoordinate:
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), contactStiffness = 0., contactDamping = 0., offset = 0., activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.contactStiffness = CheckForValidUReal(contactStiffness,"contactStiffness","ObjectContactCoordinate")
        self.contactDamping = CheckForValidUReal(contactDamping,"contactDamping","ObjectContactCoordinate")
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
        yield 'offset', self.offset
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
class VObjectContactCircleCable2D:
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), numberOfContactSegments = 3, contactStiffness = 0., contactDamping = 0., circleRadius = 0., offset = 0., activeConnector = True, visualization = {'show': True, 'showContactCircle': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.numberOfContactSegments = numberOfContactSegments
        self.contactStiffness = CheckForValidUReal(contactStiffness,"contactStiffness","ObjectContactCircleCable2D")
        self.contactDamping = CheckForValidUReal(contactDamping,"contactDamping","ObjectContactCircleCable2D")
        self.circleRadius = CheckForValidUReal(circleRadius,"circleRadius","ObjectContactCircleCable2D")
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
        yield 'contactDamping', self.contactDamping
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), numberOfContactSegments = 3, contactStiffness = 0., contactDamping = 0., frictionVelocityPenalty = 0., frictionStiffness = 0., frictionCoefficient = 0., circleRadius = 0., useSegmentNormals = True, activeConnector = True, visualization = {'show': True, 'showContactCircle': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.numberOfContactSegments = CheckForValidPInt(numberOfContactSegments,"numberOfContactSegments","ObjectContactFrictionCircleCable2D")
        self.contactStiffness = CheckForValidUReal(contactStiffness,"contactStiffness","ObjectContactFrictionCircleCable2D")
        self.contactDamping = CheckForValidUReal(contactDamping,"contactDamping","ObjectContactFrictionCircleCable2D")
        self.frictionVelocityPenalty = CheckForValidUReal(frictionVelocityPenalty,"frictionVelocityPenalty","ObjectContactFrictionCircleCable2D")
        self.frictionStiffness = CheckForValidUReal(frictionStiffness,"frictionStiffness","ObjectContactFrictionCircleCable2D")
        self.frictionCoefficient = CheckForValidUReal(frictionCoefficient,"frictionCoefficient","ObjectContactFrictionCircleCable2D")
        self.circleRadius = CheckForValidUReal(circleRadius,"circleRadius","ObjectContactFrictionCircleCable2D")
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
class VObjectJointGeneric:
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], constrainedAxes = [1,1,1,1,1,1], rotationMarker0 = IIDiagMatrix(rowsColumns=3,value=1), rotationMarker1 = IIDiagMatrix(rowsColumns=3,value=1), activeConnector = True, offsetUserFunctionParameters = [0.,0.,0.,0.,0.,0.], offsetUserFunction = 0, offsetUserFunction_t = 0, alternativeConstraints = False, visualization = {'show': True, 'axesRadius': 0.1, 'axesLength': 0.4, 'color': [-1.,-1.,-1.,-1.]}):
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], constrainedAxes = [1,1,1], activeConnector = True, discRadius = 0, discAxis = [1,0,0], planeNormal = [0,0,1], visualization = {'show': True, 'discWidth': 0.1, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.constrainedAxes = copy.copy(constrainedAxes)
        self.activeConnector = activeConnector
        self.discRadius = CheckForValidPReal(discRadius,"discRadius","ObjectJointRollingDisc")
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

class VObjectJointSliding2D:
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
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], slidingMarkerNumbers = [], slidingMarkerOffsets = [], nodeNumber = exudyn.InvalidIndex(), classicalFormulation = True, constrainRotation = False, axialForce = 0, activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.slidingMarkerNumbers = copy.copy(slidingMarkerNumbers)
        self.slidingMarkerOffsets = np.array(slidingMarkerOffsets)
        self.nodeNumber = nodeNumber
        self.classicalFormulation = classicalFormulation
        self.constrainRotation = constrainRotation
        self.axialForce = axialForce
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'JointSliding2D'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'slidingMarkerNumbers', self.slidingMarkerNumbers
        yield 'slidingMarkerOffsets', self.slidingMarkerOffsets
        yield 'nodeNumber', self.nodeNumber
        yield 'classicalFormulation', self.classicalFormulation
        yield 'constrainRotation', self.constrainRotation
        yield 'axialForce', self.axialForce
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
SlidingJoint2D = ObjectJointSliding2D
VSlidingJoint2D = VObjectJointSliding2D

class VObjectJointALEMoving2D:
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

class VObjectContactFrictionCircleCable2DOld:
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
class ObjectContactFrictionCircleCable2DOld:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), numberOfContactSegments = 3, contactStiffness = 0., contactDamping = 0., frictionVelocityPenalty = 0., frictionStiffness = 0., frictionCoefficient = 0., circleRadius = 0., offset = 0., activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.numberOfContactSegments = CheckForValidPInt(numberOfContactSegments,"numberOfContactSegments","ObjectContactFrictionCircleCable2DOld")
        self.contactStiffness = CheckForValidUReal(contactStiffness,"contactStiffness","ObjectContactFrictionCircleCable2DOld")
        self.contactDamping = CheckForValidUReal(contactDamping,"contactDamping","ObjectContactFrictionCircleCable2DOld")
        self.frictionVelocityPenalty = CheckForValidUReal(frictionVelocityPenalty,"frictionVelocityPenalty","ObjectContactFrictionCircleCable2DOld")
        self.frictionStiffness = CheckForValidUReal(frictionStiffness,"frictionStiffness","ObjectContactFrictionCircleCable2DOld")
        self.frictionCoefficient = CheckForValidUReal(frictionCoefficient,"frictionCoefficient","ObjectContactFrictionCircleCable2DOld")
        self.circleRadius = CheckForValidUReal(circleRadius,"circleRadius","ObjectContactFrictionCircleCable2DOld")
        self.offset = offset
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ContactFrictionCircleCable2DOld'
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
        yield 'offset', self.offset
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#+++++++++++++++++++++++++++++++
#MARKER
class VMarkerBodyMass:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerBodyMass:
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
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerBodyPosition:
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
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerBodyRigid:
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), localPosition = [0.,0.,0.], visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.localPosition = np.array(localPosition)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'BodyRigid'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'localPosition', self.localPosition
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerNodePosition:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerNodePosition:
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
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerNodeRigid:
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'NodeRigid'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerNodeCoordinate:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerNodeCoordinate:
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), coordinate = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.coordinate = CheckForValidUInt(coordinate,"coordinate","MarkerNodeCoordinate")
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'NodeCoordinate'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'coordinate', self.coordinate
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerNodeCoordinates:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerNodeCoordinates:
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'NodeCoordinates'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerNodeODE1Coordinate:
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerNodeODE1Coordinate:
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), coordinate = exudyn.InvalidIndex(), visualization = {'show': False}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.coordinate = CheckForValidUInt(coordinate,"coordinate","MarkerNodeODE1Coordinate")
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'NodeODE1Coordinate'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'coordinate', self.coordinate
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerNodeRotationCoordinate:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerNodeRotationCoordinate:
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), rotationCoordinate = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.rotationCoordinate = CheckForValidUInt(rotationCoordinate,"rotationCoordinate","MarkerNodeRotationCoordinate")
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'NodeRotationCoordinate'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'rotationCoordinate', self.rotationCoordinate
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerSuperElementPosition:
    def __init__(self, show = True, showMarkerNodes = True):
        self.show = show
        self.showMarkerNodes = showMarkerNodes

    def __iter__(self):
        yield 'show', self.show
        yield 'showMarkerNodes', self.showMarkerNodes

    def __repr__(self):
        return str(dict(self))
class MarkerSuperElementPosition:
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
    def __init__(self, show = True, showMarkerNodes = True):
        self.show = show
        self.showMarkerNodes = showMarkerNodes

    def __iter__(self):
        yield 'show', self.show
        yield 'showMarkerNodes', self.showMarkerNodes

    def __repr__(self):
        return str(dict(self))
class MarkerSuperElementRigid:
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), offset = [0.,0.,0.], meshNodeNumbers = [], weightingFactors = [], useAlternativeApproach = True, rotationsExponentialMap = 2, visualization = {'show': True, 'showMarkerNodes': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.offset = np.array(offset)
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
        yield 'meshNodeNumbers', self.meshNodeNumbers
        yield 'weightingFactors', self.weightingFactors
        yield 'useAlternativeApproach', self.useAlternativeApproach
        yield 'rotationsExponentialMap', self.rotationsExponentialMap
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VshowMarkerNodes', dict(self.visualization)["showMarkerNodes"]

    def __repr__(self):
        return str(dict(self))
class VMarkerKinematicTreeRigid:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerKinematicTreeRigid:
    def __init__(self, name = '', objectNumber = exudyn.InvalidIndex(), linkNumber = exudyn.InvalidIndex(), localPosition = [0.,0.,0.], visualization = {'show': True}):
        self.name = name
        self.objectNumber = objectNumber
        self.linkNumber = CheckForValidUInt(linkNumber,"linkNumber","MarkerKinematicTreeRigid")
        self.localPosition = np.array(localPosition)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'KinematicTreeRigid'
        yield 'name', self.name
        yield 'objectNumber', self.objectNumber
        yield 'linkNumber', self.linkNumber
        yield 'localPosition', self.localPosition
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerObjectODE2Coordinates:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerObjectODE2Coordinates:
    def __init__(self, name = '', objectNumber = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.objectNumber = objectNumber
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'markerType', 'ObjectODE2Coordinates'
        yield 'name', self.name
        yield 'objectNumber', self.objectNumber
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerBodyCable2DShape:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerBodyCable2DShape:
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), numberOfSegments = 3, verticalOffset = 0., visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.numberOfSegments = CheckForValidPInt(numberOfSegments,"numberOfSegments","MarkerBodyCable2DShape")
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
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerBodyCable2DCoordinates:
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
#+++++++++++++++++++++++++++++++
#LOAD
class VLoadForceVector:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class LoadForceVector:
    def __init__(self, name = '', markerNumber = exudyn.InvalidIndex(), loadVector = [0.,0.,0.], bodyFixed = False, loadVectorUserFunction = 0, visualization = {'show': True}):
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
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class LoadTorqueVector:
    def __init__(self, name = '', markerNumber = exudyn.InvalidIndex(), loadVector = [0.,0.,0.], bodyFixed = False, loadVectorUserFunction = 0, visualization = {'show': True}):
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
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class LoadMassProportional:
    def __init__(self, name = '', markerNumber = exudyn.InvalidIndex(), loadVector = [0.,0.,0.], loadVectorUserFunction = 0, visualization = {'show': True}):
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
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class LoadCoordinate:
    def __init__(self, name = '', markerNumber = exudyn.InvalidIndex(), load = 0., loadUserFunction = 0, visualization = {'show': True}):
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
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
#+++++++++++++++++++++++++++++++
#SENSOR
class VSensorNode:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class SensorNode:
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
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class SensorObject:
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
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class SensorBody:
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
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class SensorSuperElement:
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), meshNodeNumber = exudyn.InvalidIndex(), writeToFile = True, fileName = '', outputVariableType = 0, storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.meshNodeNumber = CheckForValidUInt(meshNodeNumber,"meshNodeNumber","SensorSuperElement")
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
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class SensorKinematicTree:
    def __init__(self, name = '', objectNumber = exudyn.InvalidIndex(), linkNumber = exudyn.InvalidIndex(), localPosition = [0.,0.,0.], writeToFile = True, fileName = '', outputVariableType = 0, storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.objectNumber = objectNumber
        self.linkNumber = CheckForValidUInt(linkNumber,"linkNumber","SensorKinematicTree")
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
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class SensorMarker:
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
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class SensorLoad:
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
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class SensorUserFunction:
    def __init__(self, name = '', sensorNumbers = [], factors = [], writeToFile = True, fileName = '', sensorUserFunction = 0, storeInternal = False, visualization = {'show': True}):
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
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
