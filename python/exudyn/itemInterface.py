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
    """Visualization data for NodePoint.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used

        color ([float,float,float,float]): Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used

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
        name (str): node's unique name

        referenceCoordinates ([float,float,float]): reference coordinates of node, e.g. ref. coordinates for finite elements; global position of node without displacement

        initialCoordinates ([float,float,float]): initial displacement coordinate

        initialVelocities ([float,float,float]): initial velocity coordinate

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used

        color ([float,float,float,float]): Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used

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
        name (str): node's unique name

        referenceCoordinates ([float,float]): reference coordinates of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement

        initialCoordinates ([float,float]): initial displacement coordinate

        initialVelocities ([float,float]): initial velocity coordinate

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used

        color ([float,float,float,float]): Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used

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
        name (str): node's unique name

        referenceCoordinates (array_like): reference coordinates (3 position coordinates and 4 Euler parameters) of node ==> e.g. ref. coordinates for finite elements or reference position of rigid body (e.g. for definition of joints)

        initialCoordinates (array_like): initial displacement coordinates and 4 Euler parameters relative to reference coordinates

        initialVelocities (array_like): initial velocity coordinates: time derivatives of initial displacements and Euler parameters

        addConstraintEquation (bool): True: automatically add Euler parameter constraint for node; False: Euler parameter constraint is not added, must be done manually (e.g., with CoordinateVectorConstraint)

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used

        color ([float,float,float,float]): Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used

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
        name (str): node's unique name

        referenceCoordinates (array_like): reference coordinates (3 position and 3 xyz Euler angles) of node ==> e.g. ref. coordinates for finite elements or reference position of rigid body (e.g. for definition of joints)

        initialCoordinates (array_like): initial displacement coordinates: ux,uy,uz and 3 Euler angles (xyz) relative to reference coordinates

        initialVelocities (array_like): initial velocity coordinate: time derivatives of ux,uy,uz and of 3 Euler angles (xyz)

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used

        color ([float,float,float,float]): Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used

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
        name (str): node's unique name

        referenceCoordinates (array_like): reference coordinates (position and rotation vector :math:`\nu`) of node ==> e.g. ref. coordinates for finite elements or reference position of rigid body (e.g. for definition of joints)

        initialCoordinates (array_like): initial displacement coordinates :math:`\mathbf{u}` and rotation vector :math:`\nu` relative to reference coordinates

        initialVelocities (array_like): initial velocity coordinate: time derivatives of displacement and angular velocity vector

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used

        color ([float,float,float,float]): Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used

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
        name (str): node's unique name

        referenceCoordinates ([float,float,float]): reference coordinates (x-pos,y-pos and rotation) of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement

        initialCoordinates ([float,float,float]): initial displacement coordinates and angle (relative to reference coordinates)

        initialVelocities ([float,float,float]): initial velocity coordinates

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
    """Visualization data for Node1D.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown; The node1D is represented as reference position and displacement along the global x-axis, which must not agree with the representation in the object using the Node1D

    """
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class Node1D:
    """A node with one ODE2 coordinate for one dimensional (1D) problems.
    
    Use e.g. for scalar dynamic equations (Mass1D) and mass-spring-damper mechanisms, representing either translational or rotational degrees of freedom: in most cases, Node1D is equivalent to NodeGenericODE2 using one coordinate, however, it offers a transformation to 3D translational or rotational motion and allows to couple this node to 2D or 3D bodies.
    
    Args:
        name (str): node's unique name

        referenceCoordinates (array_like): reference coordinate of node (in vector form)

        initialCoordinates (array_like): initial displacement coordinate (in vector form)

        initialVelocities (array_like): initial velocity coordinate (in vector form)

    Notes:
        Node has/provides the following types: ``GenericODE2``

    """
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
    """Visualization data for NodePoint2DSlope1.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used

        color ([float,float,float,float]): Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used

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
        name (str): node's unique name

        referenceCoordinates ([float,float,float,float]): reference coordinates (x-pos,y-pos; x-slopex, y-slopex) of node; global position of node without displacement

        initialCoordinates ([float,float,float,float]): initial displacement coordinates: ux, uy and x/y 'displacements' of slopex

        initialVelocities ([float,float,float,float]): initial velocity coordinates

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used

        color ([float,float,float,float]): Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used

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
    r"""A 3D point/slope vector node for spatial Bernoulli-Euler ANCF (absolute nodal coordinate formulation) beam elements.
    
    The node has 6 displacement degrees of freedom (3 for displacement of point node and 3 for the slope vector 'slopex'); all coordinates lead to second order differential equations; the slope vector defines the directional derivative w.r.t the local axial (x) coordinate, denoted as :math:`()^\prime`; in straight configuration aligned at the global x-axis, the slope vector reads :math:`\rv^\prime=[r_x^\prime\;\;r_y^\prime\;\;r_z^\prime]^T=[1\;\;0]^T`.
    
    Args:
        name (str): node's unique name

        referenceCoordinates (array_like): reference coordinates (x-pos,y-pos,z-pos; x-slopex, y-slopex, z-slopex) of node; global position of node without displacement

        initialCoordinates (array_like): initial displacement coordinates: ux, uy, uz and x/y/z 'displacements' of slopex

        initialVelocities (array_like): initial velocity coordinates

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used

        color ([float,float,float,float]): Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used

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
    r"""A 3D point/slope vector node for thin ANCF (absolute nodal coordinate formulation) plate elements.
    
    The node has 9 ODE2 degrees of freedom (3 for displacement of point node and 2 :math:`\times` 3 for the slope vectors 'slopeX' and 'slopeY'); all coordinates lead to second order differential equations; the slopeX vector defines the directional derivative w.r.t the local axial (x) coordinate, etc.; in straight configuration aligned at the global x-axis, the slopeY vector reads :math:`\rv_y^\prime=[0\;\;1\;\;0]^T`.
    
    Args:
        name (str): node's unique name

        referenceCoordinates (array_like): reference coordinates (x-pos,y-pos,z-pos; x-slopeX, y-slopeX, z-slopeX; x-slopeY, y-slopeY, z-slopeY) of node; global position of node without displacement

        initialCoordinates (array_like): initial displacement coordinates relative to reference coordinates

        initialVelocities (array_like): initial velocity coordinates

    Notes:
        Node has/provides the following types: ``Position``, ``Orientation``

    """
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
    """Visualization data for NodePointSlope23.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used

        color ([float,float,float,float]): Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used

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
    r"""A 3D point/slope vector node for spatial, shear and cross-section deformable ANCF (absolute nodal coordinate formulation) beam elements.
    
    The node has 9 ODE2 degrees of freedom (3 for displacement of point node and 2 :math:`\times` 3 for the slope vectors 'slopeY' and 'slopeZ'); all coordinates lead to second order differential equations; the slopeY vector defines the directional derivative w.r.t the local axial (y) coordinate, etc.; the slopeY vector reads :math:`\rv_y^\prime=[0\;\;1\;\;0]^T` and slopeZ gets :math:`\rv_z^\prime=[0\;\;0\;\;1]^T`.
    
    Args:
        name (str): node's unique name

        referenceCoordinates (array_like): reference coordinates (x-pos,y-pos,z-pos; x-slopey, y-slopey, z-slopey; x-slopez, y-slopez, z-slopez) of node; global position of node without displacement

        initialCoordinates (array_like): initial displacement coordinates relative to reference coordinates

        initialVelocities (array_like): initial velocity coordinates

    Notes:
        Node has/provides the following types: ``Position``, ``Orientation``

    """
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
    """Visualization data for NodeGenericODE2.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

    """
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class NodeGenericODE2:
    """A node containing a number of ODE2 variables.
    
    Use this node e.g. for scalar dynamic equations (Mass1D), for ObjectGenericODE2 or for the Eulerian coordinate in the ALECable element. NOTE: referenceCoordinates and all initialCoordinates(_t) must be initialized, because no default values exist.
    
    Args:
        name (str): node's unique name

        referenceCoordinates (array_like): generic reference coordinates of node; must be consistent with numberOfODE2Coordinates

        initialCoordinates (array_like): initial displacement coordinates; must be consistent with numberOfODE2Coordinates

        initialCoordinates_t (array_like): initial velocity coordinates; must be consistent with numberOfODE2Coordinates

        numberOfODE2Coordinates (int): number of generic ODE2 coordinates

    Notes:
        Node has/provides the following types: ``GenericODE2``

    """
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
    """Visualization data for NodeGenericODE1.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

    """
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class NodeGenericODE1:
    """A node containing a number of ODE1 variables.
    
    Use this node e.g. for linear state space systems. NOTE: referenceCoordinates and initialCoordinates must be initialized, because no default values exist.
    
    Args:
        name (str): node's unique name

        referenceCoordinates (array_like): generic reference coordinates of node; must be consistent with numberOfODE1Coordinates

        initialCoordinates (array_like): initial displacement coordinates; must be consistent with numberOfODE1Coordinates

        numberOfODE1Coordinates (int): number of generic ODE1 coordinates

    """
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
    """Visualization data for NodeGenericAE.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

    """
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class NodeGenericAE:
    """A node containing a number of AE variables.
    
    Use e.g. linear state space systems. NOTE: referenceCoordinates and initialCoordinates must be initialized, because no default values exist.
    
    Args:
        name (str): node's unique name

        referenceCoordinates (array_like): generic reference coordinates of node; must be consistent with numberOfAECoordinates

        initialCoordinates (array_like): initial displacement coordinates; must be consistent with numberOfAECoordinates

        numberOfAECoordinates (int): number of generic AE coordinates

    """
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
    """Visualization data for NodeGenericData.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

    """
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class NodeGenericData:
    """A node containing a number of data (history) variables.
    
    Use this node e.g. for contact (active set), friction or plasticity (history variables).
    
    Args:
        name (str): node's unique name

        initialCoordinates (array_like): initial data coordinates

        numberOfDataCoordinates (int): number of generic data coordinates (history variables)

    Notes:
        Node has/provides the following types: ``GenericData``

    """
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
    """Visualization data for NodePointGround.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used

        color ([float,float,float,float]): Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used

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
        name (str): node's unique name

        referenceCoordinates ([float,float,float]): reference coordinates of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        graphicsDataUserFunction (PyFunctionGraphicsData): A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function

        graphicsData (BodyGraphicsData): Structure contains data for body visualization; data is defined in special list / dictionary structure

    """
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
    """A ground object behaving like a rigid body, but having no degrees of freedom.
    
    Used to attach body-connectors without an action. For examples see spring dampers and joints.
    
    Args:
        name (str): objects's unique name

        referencePosition ([float,float,float]): reference point = reference position for ground object; local position is added on top of reference position for a ground object

        referenceRotation (array_like): the constant ground rotation matrix, which transforms body-fixed (b) to global (0) coordinates

    Notes:
        Object has/provides the following types: ``Ground``, ``Body``

    """
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
    """Visualization data for ObjectMassPoint.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        graphicsData (BodyGraphicsData): Structure contains data for body visualization; data is defined in special list / dictionary structure

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
        name (str): objects's unique name

        physicsMass (float): mass [SI:kg] of mass point

        nodeNumber (NodeIndex): node number (type NodeIndex) for mass point

    Notes:
        Object has/provides the following types: ``Body``, ``SingleNoded``

        Requested Node type: ``Position``

    """
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
    """Visualization data for ObjectMassPoint2D.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        graphicsData (BodyGraphicsData): Structure contains data for body visualization; data is defined in special list / dictionary structure

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
        name (str): objects's unique name

        physicsMass (float): mass [SI:kg] of mass point

        nodeNumber (NodeIndex): node number (type NodeIndex) for mass point

    Notes:
        Object has/provides the following types: ``Body``, ``SingleNoded``

        Requested Node type: ``Position2D`` + ``Position``

    """
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
    """Visualization data for ObjectMass1D.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        graphicsData (BodyGraphicsData): Structure contains data for body visualization; data is defined in special list / dictionary structure

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
        name (str): objects's unique name

        physicsMass (float): mass [SI:kg] of mass

        nodeNumber (NodeIndex): node number (type NodeIndex) for Node1D

        referencePosition ([float,float,float]): a reference position, used to transform the 1D coordinate to a position

        referenceRotation (array_like): the constant body rotation matrix, which transforms body-fixed (b) to global (0) coordinates

    Notes:
        Object has/provides the following types: ``Body``, ``SingleNoded``

        Requested Node type: ``GenericODE2``

    """
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
    """Visualization data for ObjectRotationalMass1D.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        graphicsData (BodyGraphicsData): Structure contains data for body visualization; data is defined in special list / dictionary structure

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
        name (str): objects's unique name

        physicsInertia (float): inertia components [SI:kgm:math:`^2`] of rotor / rotational mass

        nodeNumber (NodeIndex): node number (type NodeIndex) of Node1D, providing rotation coordinate :math:`\psi_0 = c_0`

        referencePosition ([float,float,float]): a constant reference position = reference point, used to assign joint constraints accordingly and for drawing

        referenceRotation (array_like): an intermediate rotation matrix, which transforms the 1D coordinate into 3D, see description

    Notes:
        Object has/provides the following types: ``Body``, ``SingleNoded``

        Requested Node type: ``GenericODE2``

    """
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
    """Visualization data for ObjectRigidBody.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        graphicsDataUserFunction (PyFunctionGraphicsData): A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics elements need to be defined in the local body coordinates and are transformed by mbs to global coordinates

        graphicsData (BodyGraphicsData): Structure contains data for body visualization; data is defined in special list / dictionary structure

    """
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
    """A 3D rigid body which is attached to a 3D rigid body node.
    
    The rotation parametrization of the rigid body follows the rotation parametrization of the node. Use Euler parameters in the general case (no singularities) in combination with implicit solvers (GeneralizedAlpha or TrapezoidalIndex2), Tait-Bryan angles for special cases, e.g., rotors where no singularities occur if you rotate about :math:`x` or :math:`z` axis, or use Lie-group formulation with rotation vector together with explicit solvers. REMARK: Use the class ``RigidBodyInertia``, see theDoc.pdf and ``CreateRigidBody(...)``, see theDoc.pdf, of ``exudyn.rigidBodyUtilities`` to handle inertia, COM and mass. addExampleImage{ObjectRigidBody}
    
    Args:
        name (str): objects's unique name

        physicsMass (float): mass [SI:kg] of rigid body

        physicsInertia (array_like): inertia components [SI:kgm:math:`^2`]: :math:`[J_{xx}, J_{yy}, J_{zz}, J_{yz}, J_{xz}, J_{xy}]` in body-fixed coordinate system and w.r.t. to the reference point of the body, NOT necessarily w.r.t. to COM; use the class RigidBodyInertia of exudynRigidBodyUtilities.py and CreateRigidBody(...) of MainSystem to handle inertia, COM and mass

        physicsCenterOfMass ([float,float,float]): local position of COM relative to the body's reference point; if the vector of the COM is [0,0,0], the computation will not consider additional terms for the COM and it is faster

        nodeNumber (NodeIndex): node number (type NodeIndex) for rigid body node

    Notes:
        Object has/provides the following types: ``Body``, ``SingleNoded``

        Requested Node type: ``Position`` + ``Orientation`` + ``RigidBody``

    """
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
    """Visualization data for ObjectRigidBody2D.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        graphicsDataUserFunction (PyFunctionGraphicsData): A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics elements need to be defined in the local body coordinates and are transformed by mbs to global coordinates

        graphicsData (BodyGraphicsData): Structure contains data for body visualization; data is defined in special list / dictionary structure

    """
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
    """A 2D rigid body which is attached to a rigid body 2D node.
    
    The body obtains coordinates, position, velocity, etc. from the underlying 2D node.
    
    Args:
        name (str): objects's unique name

        physicsMass (float): mass [SI:kg] of rigid body

        physicsInertia (float): inertia [SI:kgm:math:`^2`] of rigid body w.r.t. reference point; this is equal to the center of mass, if physicsCenterOfMass = 0

        physicsCenterOfMass ([float,float]): local position of COM relative to the body's reference point; if the vector of the COM is [0,0], the computation will not consider additional terms for the COM and it is faster

        nodeNumber (NodeIndex): node number (type NodeIndex) for 2D rigid body node

    Notes:
        Object has/provides the following types: ``Body``, ``SingleNoded``

        Requested Node type: ``Position2D`` + ``Orientation2D`` + ``Position`` + ``Orientation``

    """
    def __init__(self, name = '', physicsMass = 0., physicsInertia = 0., physicsCenterOfMass = [0.,0.], nodeNumber = exudyn.InvalidIndex(), visualization = {'show': True, 'graphicsDataUserFunction': 0, 'graphicsData': []}):
        self.name = name
        self.physicsMass = CheckForValidUReal(physicsMass,"physicsMass","ObjectRigidBody2D")
        self.physicsInertia = CheckForValidUReal(physicsInertia,"physicsInertia","ObjectRigidBody2D")
        self.physicsCenterOfMass = np.array(physicsCenterOfMass)
        self.nodeNumber = nodeNumber
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'RigidBody2D'
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
RigidBody2D = ObjectRigidBody2D
VRigidBody2D = VObjectRigidBody2D

class VObjectGenericODE2:
    """Visualization data for ObjectGenericODE2.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        color ([float,float,float,float]): RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used

        triangleMesh (array_like): a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!

        showNodes (bool): set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'

        graphicsDataUserFunction (PyFunctionGraphicsData): A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics data is draw in global coordinates; it can be used to implement user element visualization, e.g., beam elements or simple mechanical systems; note that this user function may significantly slow down visualization

    """
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
    r"""A system of :math:`n` second order ordinary differential equations (ODE2), having a mass matrix, damping/gyroscopic matrix, stiffness matrix and generalized forces.
    
    It can combine generic nodes, or node points. User functions can be used to compute mass matrix and generalized forces depending on given coordinates. NOTE: all matrices, vectors, etc. must have the same dimensions :math:`n` or :math:`(n \times n)`, or they must be empty :math:`(0 \times 0)`, except for the mass matrix which always needs to have dimensions :math:`(n \times n)`.
    
    Args:
        name (str): objects's unique name

        nodeNumbers (ArrayNodeIndex): node numbers which provide the coordinates for the object (consecutively as provided in this list)

        massMatrix (PyMatrixContainer): mass matrix of object as MatrixContainer (or numpy array / list of lists)

        stiffnessMatrix (PyMatrixContainer): stiffness matrix of object as MatrixContainer (or numpy array / list of lists); NOTE that (dense/sparse triplets) format must agree with dampingMatrix and jacobianUserFunction

        dampingMatrix (PyMatrixContainer): damping matrix of object as MatrixContainer (or numpy array / list of lists); NOTE that (dense/sparse triplets) format must agree with stiffnessMatrix and jacobianUserFunction

        forceVector (array_like): generalized force vector added to RHS

        forceUserFunction (PyFunctionVectorMbsScalarIndex2Vector): A Python user function which computes the generalized user force vector for the ODE2 equations; see description below

        massMatrixUserFunction (PyFunctionMatrixContainerMbsScalarIndex2Vector): A Python user function which computes the mass matrix instead of the constant mass matrix given in :math:`\Mm`; return numpy array or MatrixContainer; see description below

        jacobianUserFunction (PyFunctionMatrixContainerMbsScalarIndex2Vector2Scalar): A Python user function which computes the jacobian, i.e., the derivative of the left-hand-side object equation w.r.t. the coordinates (times :math:`f_{ODE2}`) and w.r.t. the velocities (times :math:`f_{ODE2_t}`). Terms on the RHS must be subtracted from the LHS equation; the respective terms for the stiffness matrix and damping matrix are automatically added; see description below

        coordinateIndexPerNode (array_like): this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed

        tempCoordinates (array_like): temporary vector containing coordinates

        tempCoordinates_t (array_like): temporary vector containing velocity coordinates

        tempCoordinates_tt (array_like): temporary vector containing acceleration coordinates

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``, ``SuperElement``

    """
    def __init__(self, name = '', nodeNumbers = [], massMatrix = None, stiffnessMatrix = None, dampingMatrix = None, forceVector = [], forceUserFunction = 0, massMatrixUserFunction = 0, jacobianUserFunction = 0, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.], 'triangleMesh': [], 'showNodes': False, 'graphicsDataUserFunction': 0}):
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
    """Visualization data for ObjectGenericODE1.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class ObjectGenericODE1:
    r"""A system of :math:`n` acf{ODE1}, having a system matrix, a rhs vector, but mostly it will use a user function to describe special ODE1 systems.
    
    It is based on NodeGenericODE1 nodes. NOTE that all matrices, vectors, etc. must have the same dimensions :math:`n` or :math:`(n \times n)`, or they must be empty :math:`(0 \times 0)`, using [] in Python.
    
    Args:
        name (str): objects's unique name

        nodeNumbers (ArrayNodeIndex): node numbers which provide the coordinates for the object (consecutively as provided in this list)

        systemMatrix (array_like): system matrix (state space matrix) of first order ODE

        rhsVector (array_like): a constant rhs vector (e.g., for constant input)

        rhsUserFunction (PyFunctionVectorMbsScalarIndexVector): A Python user function which computes the right-hand-side (rhs) of the first order ODE; see description below

        coordinateIndexPerNode (array_like): this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed

        tempCoordinates (array_like): temporary vector containing coordinates

        tempCoordinates_t (array_like): temporary vector containing velocity coordinates

    Notes:
        Object has/provides the following types: ``MultiNoded``

    """
    def __init__(self, name = '', nodeNumbers = [], systemMatrix = [], rhsVector = [], rhsUserFunction = 0, visualization = {'show': True}):
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
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))

class VObjectKinematicTree:
    """Visualization data for ObjectKinematicTree.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        showLinks (bool): set true, if links shall be shown; if graphicsDataList is empty, a standard drawing for links is used (drawing a cylinder from previous joint or base to next joint; size relative to frame size in KinematicTree visualization settings); else graphicsDataList are used per link; NOTE visualization of joint and COM frames can be modified via visualizationSettings.bodies.kinematicTree

        showJoints (bool): set true, if joints shall be shown; if graphicsDataList is empty, a standard drawing for joints is used (drawing a cylinder for revolute joints; size relative to frame size in KinematicTree visualization settings)

        color ([float,float,float,float]): RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used

        graphicsDataList (BodyGraphicsDataList): Structure contains data for link/joint visualization; data is defined as list of BodyGraphicsData where every BodyGraphicsData corresponds to one link/joint; must either be emtpy list or length must agree with number of links

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
        name (str): objects's unique name

        nodeNumber (NodeIndex): node number (type NodeIndex) of GenericODE2 node containing the coordinates for the kinematic tree; :math:`n` being the number of minimal coordinates

        gravity ([float,float,float]): gravity vector in inertial coordinates; used to simply apply gravity as LoadMassProportional is not available for KinematicTree

        baseOffset ([float,float,float]): offset vector for base, in global coordinates

        jointTypes (JointTypeList): joint types of kinematic Tree joints, using exu.JointType, like exu.JointType.RevoluteZ; must be always set

        linkParents (array_like): index of parent joint/link; if no parent exists, the value is :math:`-1`; by default, :math:`p_0=-1` because the :math:`i`th parent index must always fulfill :math:`p_i<i`; must be always set

        jointTransformations (Matrix3DList): list of constant joint transformations from parent joint coordinates :math:`p_0` to this joint coordinates :math:`j_0`; this allows to adjust the orientation of the joint axes (but it does not affect the joint offset); if no parent exists (:math:`-1`), the base coordinate system :math:`0` is used; must be always set

        jointOffsets (Vector3DList): list of constant joint offsets from parent joint to this joint; :math:`p_0`, :math:`p_1`, :math:`\ldots` denote the parent coordinate systems; this means that the joint offset is added prior to performing the joint transformation; if no parent exists (:math:`-1`), the base coordinate system :math:`0` is used; must be always set

        linkInertiasCOM (Matrix3DList): list of link inertia tensors w.r.t. COM in joint/link :math:`j_i` coordinates; must be always set

        linkCOMs (Vector3DList): list of vectors for center of mass (COM) in joint/link :math:`j_i` coordinates; must be always set

        linkMasses (array_like): masses of links; must be always set

        linkForces (Vector3DList): list of 3D force vectors per link in global coordinates acting on joint frame origin; use force-torque couple to realize off-origin forces; defaults to empty list :math:`[]`, adding no forces

        linkTorques (Vector3DList): list of 3D torque vectors per link in global coordinates; defaults to empty list :math:`[]`, adding no torques

        jointForceVector (array_like): generalized force vector per coordinate added to RHS of EOM; represents a torque around the axis of rotation in revolute joints and a force in prismatic joints; for a revolute joint :math:`i`, the torque :math:`f[i]` acts positive (w.r.t. rotation axis) on link :math:`i` and negative on parent link :math:`p_i`; must be either empty list/array :math:`[]` (default) or have size :math:`n`

        jointPositionOffsetVector (array_like): offset for joint coordinates used in P(D) control; acts in positive joint direction similar to jointForceVector; should be modified, e.g., in preStepUserFunction; must be either empty list/array :math:`[]` (default) or have size :math:`n`

        jointVelocityOffsetVector (array_like): velocity offset for joint coordinates used in (P)D control; acts in positive joint direction similar to jointForceVector; should be modified, e.g., in preStepUserFunction; must be either empty list/array :math:`[]` (default) or have size :math:`n`

        jointPControlVector (array_like): proportional (P) control values per joint (multiplied with position error between joint value and offset :math:`\mathbf{u}_o`); note that more complicated control laws must be implemented with user functions; must be either empty list/array :math:`[]` (default) or have size :math:`n`

        jointDControlVector (array_like): derivative (D) control values per joint (multiplied with velocity error between joint velocity and velocity offset :math:`\vv_o`); note that more complicated control laws must be implemented with user functions; must be either empty list/array :math:`[]` (default) or have size :math:`n`

        forceUserFunction (PyFunctionVectorMbsScalarIndex2Vector): A Python user function which computes the generalized force vector on RHS with identical action as jointForceVector; see description below

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``, ``SuperElement``

        Requested Node type: ``GenericODE2``

    """
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
    """Visualization data for ObjectFFRF.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown; use visualizationSettings.bodies.deformationScaleFactor to draw scaled (local) deformations; the reference frame node is shown with additional letters RF

        color ([float,float,float,float]): RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used

        triangleMesh (array_like): a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!

        showNodes (bool): set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'

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
        name (str): objects's unique name

        nodeNumbers (ArrayNodeIndex): node numbers which provide the coordinates for the object (consecutively as provided in this list); the :math:`(n_\mathrm{nf}+1)` nodes represent the nodes of the FE mesh (except for node 0); the global nodal position needs to be reconstructed from the rigid-body motion of the reference frame

        massMatrixFF (PyMatrixContainer): body-fixed and ONLY flexible coordinates part of mass matrix of object given in Python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format

        stiffnessMatrixFF (PyMatrixContainer): body-fixed and ONLY flexible coordinates part of stiffness matrix of object in Python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format

        dampingMatrixFF (PyMatrixContainer): body-fixed and ONLY flexible coordinates part of damping matrix of object in Python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format

        forceVector (array_like): generalized, force vector added to RHS; the rigid body part :math:`\fv_r` is directly applied to rigid body coordinates while the flexible part :math:`\fv\indf` is transformed from global to local coordinates; note that this force vector only allows to add gravity forces for bodies with COM at the origin of the reference frame

        forceUserFunction (PyFunctionVectorMbsScalarIndex2Vector): A Python user function which computes the generalized user force vector for the ODE2 equations; note the different coordinate systems for rigid body and flexible part; The function args are mbs, time, objectNumber, coordinates q (without reference values) and coordinate velocities q_t; see description below

        massMatrixUserFunction (PyFunctionMatrixMbsScalarIndex2Vector): A Python user function which computes the TOTAL mass matrix (including reference node) and adds the local constant mass matrix; note the different coordinate systems as described in the FFRF mass matrix; see description below

        computeFFRFterms (bool): flag decides whether the standard FFRF terms are computed; use this flag for user-defined definition of FFRF terms in mass matrix and quadratic velocity vector

        coordinateIndexPerNode (array_like): this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed

        objectIsInitialized (bool): ALWAYS set to False! flag used to correctly initialize all FFRF matrices; as soon as this flag is False, internal (constant) FFRF matrices are recomputed during Assemble()

        physicsMass (float): total mass [SI:kg] of FFRF object, auto-computed from mass matrix :math:`{}^{b}{\Mm}`

        physicsInertia (array_like): inertia tensor [SI:kgm:math:`^2`] of rigid body w.r.t. to the reference point of the body, auto-computed from the mass matrix :math:`{}^{b}{\Mm}`

        physicsCenterOfMass ([float,float,float]): local position of center of mass (COM); auto-computed from mass matrix :math:`{}^{b}{\Mm}`

        PHItTM (array_like): projector matrix; may be removed in future

        referencePositions (array_like): vector containing the reference positions of all flexible nodes

        tempVector (array_like): temporary vector

        tempCoordinates (array_like): temporary vector containing coordinates

        tempCoordinates_t (array_like): temporary vector containing velocity coordinates

        tempRefPosSkew (array_like): temporary matrix with skew symmetric local (deformed) node positions

        tempVelSkew (array_like): temporary matrix with skew symmetric local node velocities

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``, ``SuperElement``

    """
    def __init__(self, name = '', nodeNumbers = [], massMatrixFF = None, stiffnessMatrixFF = None, dampingMatrixFF = None, forceVector = [], forceUserFunction = 0, massMatrixUserFunction = 0, computeFFRFterms = True, objectIsInitialized = False, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.], 'triangleMesh': [], 'showNodes': False}):
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
        show (bool): set true, if item is shown in visualization and false if it is not shown; use visualizationSettings.bodies.deformationScaleFactor to draw scaled (local) deformations; the reference frame node is shown with additional letters RF

        color ([float,float,float,float]): RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used

        triangleMesh (array_like): a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!

        showNodes (bool): set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'

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
    
    It can be used to model real-life mechanical systems imported from finite element codes or Python tools such as NETGEN/NGsolve, see the ``FEMinterface`` in theDoc.pdf. It contains a RigidBodyNode (always node 0) and a NodeGenericODE2 representing the modal coordinates. Currently, equations must be defined within user functions, which are available in the FEM module, see class ``ObjectFFRFreducedOrderInterface``, especially the user functions ``UFmassFFRFreducedOrder`` and ``UFforceFFRFreducedOrder``, theDoc.pdf.
    
    Args:
        name (str): objects's unique name

        nodeNumbers (ArrayNodeIndex): node numbers of rigid body node and NodeGenericODE2 for modal coordinates; the global nodal position needs to be reconstructed from the rigid-body motion of the reference frame, the modal coordinates and the mode basis

        massMatrixReduced (PyMatrixContainer): body-fixed and ONLY flexible coordinates part of reduced mass matrix; provided as MatrixContainer(sparse/dense matrix)

        stiffnessMatrixReduced (PyMatrixContainer): body-fixed and ONLY flexible coordinates part of reduced stiffness matrix; provided as MatrixContainer(sparse/dense matrix)

        dampingMatrixReduced (PyMatrixContainer): body-fixed and ONLY flexible coordinates part of reduced damping matrix; provided as MatrixContainer(sparse/dense matrix)

        forceUserFunction (PyFunctionVectorMbsScalarIndex2Vector): A Python user function which computes the generalized user force vector for the ODE2 equations; see description below

        massMatrixUserFunction (PyFunctionMatrixMbsScalarIndex2Vector): A Python user function which computes the TOTAL mass matrix (including reference node) and adds the local constant mass matrix; see description below

        computeFFRFterms (bool): flag decides whether the standard FFRF/CMS terms are computed; use this flag for user-defined definition of FFRF terms in mass matrix and quadratic velocity vector

        modeBasis (array_like): mode basis, which transforms reduced coordinates to (full) nodal coordinates, written as a single vector :math:`[u_{x,n_0},\,u_{y,n_0},\,u_{z,n_0},\,\ldots,\,u_{x,n_n},\,u_{y,n_n},\,u_{z,n_n}]\tp`

        outputVariableModeBasis (array_like): mode basis, which transforms reduced coordinates to output variables per mode and per node; :math:`s_{OV}` is the size of the output variable, e.g., 6 for stress modes (:math:`S_{xx},...,S_{xy}`)

        outputVariableTypeModeBasis (OutputVariableType): this must be the output variable type of the outputVariableModeBasis, e.g. exu.OutputVariableType.Stress

        referencePositions (array_like): vector containing the reference positions of all flexible nodes, needed for graphics

        objectIsInitialized (bool): ALWAYS set to False! flag used to correctly initialize all FFRF matrices; as soon as this flag is False, some internal (constant) FFRF matrices are recomputed during Assemble()

        physicsMass (float): total mass [SI:kg] of FFRFreducedOrder object

        physicsInertia (array_like): inertia tensor [SI:kgm:math:`^2`] of rigid body w.r.t. to the reference point of the body

        physicsCenterOfMass ([float,float,float]): local position of center of mass (COM)

        mPsiTildePsi (array_like): special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface

        mPsiTildePsiTilde (array_like): special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface

        mPhitTPsi (array_like): special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface

        mPhitTPsiTilde (array_like): special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface

        mXRefTildePsi (array_like): special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface

        mXRefTildePsiTilde (array_like): special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface

        physicsCenterOfMassTilde (array_like): tilde matrix from local position of COM; autocomputed during initialization

        tempUserFunctionForce (array_like): temporary vector for UF force

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``, ``SuperElement``

    """
    def __init__(self, name = '', nodeNumbers = [], massMatrixReduced = None, stiffnessMatrixReduced = None, dampingMatrixReduced = None, forceUserFunction = 0, massMatrixUserFunction = 0, computeFFRFterms = True, modeBasis = [], outputVariableModeBasis = [], outputVariableTypeModeBasis = 0, referencePositions = [], objectIsInitialized = False, physicsMass = 0., physicsInertia = IIDiagMatrix(rowsColumns=3,value=1), physicsCenterOfMass = [0.,0.,0.], mPsiTildePsi = [], mPsiTildePsiTilde = [], mPhitTPsi = [], mPhitTPsiTilde = [], mXRefTildePsi = [], mXRefTildePsiTilde = [], physicsCenterOfMassTilde = IIDiagMatrix(rowsColumns=3,value=0), visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.], 'triangleMesh': [], 'showNodes': False}):
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
        self.physicsMass = CheckForValidUReal(physicsMass,"physicsMass","ObjectFFRFreducedOrder")
        self.physicsInertia = np.array(physicsInertia)
        self.physicsCenterOfMass = np.array(physicsCenterOfMass)
        self.mPsiTildePsi = CheckForValidNumpyArray(mPsiTildePsi)
        self.mPsiTildePsiTilde = CheckForValidNumpyArray(mPsiTildePsiTilde)
        self.mPhitTPsi = CheckForValidNumpyArray(mPhitTPsi)
        self.mPhitTPsiTilde = CheckForValidNumpyArray(mPhitTPsiTilde)
        self.mXRefTildePsi = CheckForValidNumpyArray(mXRefTildePsi)
        self.mXRefTildePsiTilde = CheckForValidNumpyArray(mXRefTildePsiTilde)
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
    """Visualization data for ObjectANCFCable.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown; note that all quantities are computed at the beam centerline, even if drawn on surface of cylinder of beam; this effects, e.g., Displacement or Velocity, which is drawn constant over cross section

        radius (float): if radius==0, only the centerline is drawn; else, a cylinder with radius is drawn; circumferential tiling follows general.cylinderTiling and beam axis tiling follows bodies.beams.axialTiling

        color ([float,float,float,float]): RGBA color of the object; if R==-1, use default color

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
    
    The localPosition of the beam with length :math:`L`=physicsLength and height :math:`h` ranges in :math:`X`-direction in range :math:`[0, L]` and in :math:`Y`-direction in range :math:`[-h/2,h/2]` (which is in fact not needed in the EOM). For description see ObjectANCFCable2D, which is almost identical to 3D case. NOTE: this element does not include torsion, therfore a torque cannot be applied along the local x-axis.
    
    Args:
        name (str): objects's unique name

        physicsLength (float): [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives :math:`\rho A L`; must be positive

        physicsMassPerLength (float): [SI:kg/m] mass per length of beam

        physicsBendingStiffness (float): [SI:Nm:math:`^2`] bending stiffness of beam; the bending moment is :math:`m = EI (\kappa - \kappa_0)`, in which :math:`\kappa` is the material measure of curvature

        physicsAxialStiffness (float): [SI:N] axial stiffness of beam; the axial force is :math:`f_{ax} = EA (\varepsilon -\varepsilon_0)`, in which :math:`\varepsilon = |\rv^\prime|-1` is the axial strain

        physicsBendingDamping (float): [SI:Nm:math:`^2`/s] bending damping of beam ; the additional virtual work due to damping is :math:`\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx`

        physicsAxialDamping (float): [SI:N/s] axial damping of beam; the additional virtual work due to damping is :math:`\delta W_{\dot\varepsilon} = \int_0^L \dot \varepsilon \delta \varepsilon dx`

        physicsReferenceAxialStrain (float): [SI:1] reference axial strain of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference axial strain value

        strainIsRelativeToReference (float): if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration plus the values of :math:`\varepsilon_0` and :math:`\kappa_0` serve as a reference geometry; allows also values between 0. and 1.

        nodeNumbers (NodeIndex2): two node numbers ANCF cable element

        useReducedOrderIntegration (int): 0/false: use Gauss order 9 integration for virtual work of axial forces, order 5 for virtual work of bending moments; 1/true: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``

        Requested Node type: ``Position``

    """
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
    """Visualization data for ObjectANCFCable2D.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawHeight (float): if beam is drawn with rectangular shape, this is the drawing height

        color ([float,float,float,float]): RGBA color of the object; if R==-1, use default color

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
    
    The localPosition of the beam with length :math:`L`=physicsLength and height :math:`h` ranges in :math:`X`-direction in range :math:`[0, L]` and in :math:`Y`-direction in range :math:`[-h/2,h/2]` (which is in fact not needed in the EOM).
    
    Args:
        name (str): objects's unique name

        physicsLength (float): [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives :math:`\rho A L`; must be positive

        physicsMassPerLength (float): [SI:kg/m] mass per length of beam

        physicsBendingStiffness (float): [SI:Nm:math:`^2`] bending stiffness of beam; the bending moment is :math:`m = EI (\kappa - \kappa_0)`, in which :math:`\kappa` is the material measure of curvature

        physicsAxialStiffness (float): [SI:N] axial stiffness of beam; the axial force is :math:`f_{ax} = EA (\varepsilon -\varepsilon_0)`, in which :math:`\varepsilon = |\rv^\prime|-1` is the axial strain

        physicsBendingDamping (float): [SI:Nm:math:`^2`/s] bending damping of beam ; the additional virtual work due to damping is :math:`\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx`

        physicsAxialDamping (float): [SI:N/s] axial damping of beam; the additional virtual work due to damping is :math:`\delta W_{\dot\varepsilon} = \int_0^L \dot \varepsilon \delta \varepsilon dx`

        physicsReferenceAxialStrain (float): [SI:1] reference axial strain of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference axial strain value

        physicsReferenceCurvature (float): [SI:1/m] reference curvature of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference curvature value

        strainIsRelativeToReference (float): if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration plus the values of :math:`\varepsilon_0` and :math:`\kappa_0` serve as a reference geometry; allows also values between 0. and 1.

        nodeNumbers (NodeIndex2): two node numbers ANCF cable element

        useReducedOrderIntegration (int): 0/false: use Gauss order 9 integration for virtual work of axial forces, order 5 for virtual work of bending moments; 1/True: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments; 2: use mixed Lobatto/Gauss integration with exceptional quality of axial strain, however, spurious (hourglass) modes may occur!

        axialForceUserFunction (PyFunctionMbsScalarIndexScalar9): A Python function which defines the (nonlinear relations) of local strains (including axial strain and bending strain) as well as time derivatives to the local axial force; see description below

        bendingMomentUserFunction (PyFunctionMbsScalarIndexScalar9): A Python function which defines the (nonlinear relations) of local strains (including axial strain and bending strain) as well as time derivatives to the local bending moment; see description below

    Notes:
        Requested Node type: ``Position2D`` + ``Orientation2D`` + ``Point2DSlope1`` + ``Position`` + ``Orientation``

    """
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
    """Visualization data for ObjectALEANCFCable2D.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawHeight (float): if beam is drawn with rectangular shape, this is the drawing height

        color ([float,float,float,float]): RGBA color of the object; if R==-1, use default color

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
    
    This allows modeling pipes but also axially moving beams. The localPosition of the beam with length :math:`L`=physicsLength and height :math:`h` ranges in :math:`X`-direction in range :math:`[0, L]` and in :math:`Y`-direction in range :math:`[-h/2,h/2]` (which is in fact not needed in the EOM).
    
    Args:
        name (str): objects's unique name

        physicsLength (float): [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives :math:`\rho A L`; must be positive

        physicsMassPerLength (float): [SI:kg/m] total mass per length of beam (including axially moving parts / fluid)

        physicsMovingMassFactor (float): this factor denotes the amount of :math:`\rho A` which is moving; physicsMovingMassFactor=1 means, that all mass is moving; physicsMovingMassFactor=0 means, that no mass is moving; factor can be used to simulate e.g. pipe conveying fluid, in which :math:`\rho A` is the mass of the pipe+fluid, while :math:`physicsMovingMassFactor \cdot \rho A` is the mass per unit length of the fluid

        physicsBendingStiffness (float): [SI:Nm:math:`^2`] bending stiffness of beam; the bending moment is :math:`m = EI (\kappa - \kappa_0)`, in which :math:`\kappa` is the material measure of curvature

        physicsAxialStiffness (float): [SI:N] axial stiffness of beam; the axial force is :math:`f_{ax} = EA (\varepsilon -\varepsilon_0)`, in which :math:`\varepsilon = |\rv^\prime|-1` is the axial strain

        physicsBendingDamping (float): [SI:Nm:math:`^2`/s] bending damping of beam ; the additional virtual work due to damping is :math:`\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx`

        physicsAxialDamping (float): [SI:N/s] axial damping of beam; the additional virtual work due to damping is :math:`\delta W_{\dot\varepsilon} = \int_0^L \dot \varepsilon \delta \varepsilon dx`

        physicsReferenceAxialStrain (float): [SI:1] reference axial strain of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference axial strain value

        physicsReferenceCurvature (float): [SI:1/m] reference curvature of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference curvature value

        physicsUseCouplingTerms (bool): true: correct case, where all coupling terms due to moving mass are respected; false: only include constant mass for ALE node coordinate, but deactivate other coupling terms (behaves like ANCFCable2D then)

        physicsAddALEvariation (bool): true: correct case, where additional terms related to variation of strain and curvature are added

        nodeNumbers (NodeIndex3): two node numbers ANCF cable element, third node=ALE GenericODE2 node

        useReducedOrderIntegration (int): 0/false: use Gauss order 9 integration for virtual work of axial forces, order 5 for virtual work of bending moments; 1/true: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments

        strainIsRelativeToReference (float): if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration plus the values of :math:`\varepsilon_0` and :math:`\kappa_0` serve as a reference geometry; allows also values between 0. and 1.

    """
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
    """Visualization data for ObjectANCFBeam.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown; geometry is defined by sectionGeometry

        sectionGeometry (BeamSectionGeometry): defines cross section shape used for visualization and contact

        color ([float,float,float,float]): RGBA color of the object; if R==-1, use default color

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
        name (str): objects's unique name

        nodeNumbers (NodeIndex2): two node numbers for beam element

        physicsLength (float): [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives :math:`\rho A L`; must be positive

        sectionData (BeamSection): data as given by exudyn.BeamSection(), defining inertial, stiffness and damping parameters of beam section.

        crossSectionPenaltyFactor ([float,float,float]): [SI:1] additional penalty factors for cross section deformation, which are in total :math:`k_{cs} = [f_{yy}\cdot EA,\, f_{zz}\cdot EA,\, f_{yz}\cdot (GA_y+GA_z)]\tp`

        crossSectionDamping ([float,float,float]): [SI:1] viscous damping according to penalty factors for cross section deformation; the damping is relative to the stiffness and should be thus usually much smaller than 1; the viscous damping factors read  :math:`d_{cs} = [d_{fyy}\cdot EA,\, d_{fzz}\cdot EA,\, d_{fyz}\cdot (GA_y+GA_z)]\tp`

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``

        Requested Node type: ``Position`` + ``Orientation``

    """
    def __init__(self, name = '', nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex()], physicsLength = 0., sectionData = exudyn.BeamSection(), crossSectionPenaltyFactor = [1.,1.,1.], crossSectionDamping = [0.,0.,0.], visualization = {'show': True, 'sectionGeometry': exudyn.BeamSectionGeometry(), 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.physicsLength = CheckForValidPReal(physicsLength,"physicsLength","ObjectANCFBeam")
        self.sectionData = sectionData
        self.crossSectionPenaltyFactor = np.array(crossSectionPenaltyFactor)
        self.crossSectionDamping = np.array(crossSectionDamping)
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ANCFBeam'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'physicsLength', self.physicsLength
        yield 'sectionData', self.sectionData
        yield 'crossSectionPenaltyFactor', self.crossSectionPenaltyFactor
        yield 'crossSectionDamping', self.crossSectionDamping
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VsectionGeometry', dict(self.visualization)["sectionGeometry"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
ANCFBeam = ObjectANCFBeam
VANCFBeam = VObjectANCFBeam

class VObjectBeamGeometricallyExact2D:
    """Visualization data for ObjectBeamGeometricallyExact2D.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawHeight (float): if beam is drawn with rectangular shape, this is the drawing height

        color ([float,float,float,float]): RGBA color of the object; if R==-1, use default color

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
    
    Note that the orientation of the nodes need to follow the cross section orientation in case that includeReferenceRotations=True; e.g., an angle 0 represents the cross section aligned with the :math:`y`-axis, while and angle :math:`\pi/2` means that the cross section points in negative :math:`x`-direction. Pre-curvature can be included with physicsReferenceCurvature and axial pre-stress can be considered by using a physicsLength different from the reference configuration of the nodes. The localPosition of the beam with length :math:`L`=physicsLength and height :math:`h` ranges in :math:`X`-direction in range :math:`[-L/2, L/2]` and in :math:`Y`-direction in range :math:`[-h/2,h/2]` (which is in fact not needed in the EOM).
    
    Args:
        name (str): objects's unique name

        nodeNumbers (ArrayNodeIndex): two node numbers for beam element

        physicsLength (float): [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives :math:`\rho A L`; must be positive

        physicsMassPerLength (float): [SI:kg/m] mass per length of beam

        physicsCrossSectionInertia (float): [SI:kg m] cross section mass moment of inertia; inertia acting against rotation of cross section

        physicsBendingStiffness (float): [SI:Nm:math:`^2`] bending stiffness of beam; the bending moment is :math:`m = EI (\kappa - \kappa_0)`, in which :math:`\kappa` is the material measure of curvature

        physicsAxialStiffness (float): [SI:N] axial stiffness of beam; the axial force is :math:`f_{ax} = EA (\varepsilon -\varepsilon_0)`, in which :math:`\varepsilon` is the axial strain

        physicsShearStiffness (float): [SI:N] effective shear stiffness of beam, including stiffness correction

        physicsBendingDamping (float): [SI:Nm:math:`^2`/s] viscous damping of bending deformation; the additional virtual work due to damping is :math:`\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx`

        physicsAxialDamping (float): [SI:N/s] viscous damping of axial deformation

        physicsShearDamping (float): [SI:N/s] viscous damping of shear deformation

        physicsReferenceCurvature (float): [SI:1/m] reference curvature of beam (pre-deformation) of beam

        includeReferenceRotations (bool): if True, rotation of the cross section at the nodes includes node reference rotations (within referenceCoordinates of NodeRigidBody2D), which are used for the computation of bending strains (this means that a pre-curved beam is stress-free); if False, the reference rotation of the cross section is orthogonal to the reference slope vector. This allows to easily share nodes among several beams with different reference cross section orientation (i.e., only the change of rotation counts).

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``

        Requested Node type: ``Position2D`` + ``Orientation2D`` + ``Position`` + ``Orientation``

    """
    def __init__(self, name = '', nodeNumbers = [], physicsLength = 0., physicsMassPerLength = 0., physicsCrossSectionInertia = 0., physicsBendingStiffness = 0., physicsAxialStiffness = 0., physicsShearStiffness = 0., physicsBendingDamping = 0., physicsAxialDamping = 0., physicsShearDamping = 0., physicsReferenceCurvature = 0., includeReferenceRotations = False, visualization = {'show': True, 'drawHeight': 0., 'color': [-1.,-1.,-1.,-1.]}):
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
    """Visualization data for ObjectBeamGeometricallyExact.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown; geometry is defined by sectionGeometry

        sectionGeometry (BeamSectionGeometry): defines cross section shape used for visualization and contact

        color ([float,float,float,float]): RGBA color of the object; if R==-1, use default color

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
    r"""A 3D geometrically exact beam finite element, currently using two 3D rigid body nodes.
    
    The localPosition :math:`x` of the beam ranges from :math:`-L/2` (at node 0) to :math:`L/2` (at node 1). The axial coordinate is :math:`x` (first coordinate) and the cross section is spanned by local :math:`y`/:math:`z` axes. NOTE: Requires further development and tests!
    
    Args:
        name (str): objects's unique name

        nodeNumbers (NodeIndex2): two node numbers for beam element

        physicsLength (float): [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives :math:`\rho A L`; must be positive

        sectionData (BeamSection): data as given by exudyn.BeamSection(), defining inertial, stiffness and damping parameters of beam section.

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``

        Requested Node type: ``Position`` + ``Orientation``

    """
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
    """Visualization data for ObjectANCFThinPlate.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown; note that all quantities are computed at the beam centerline, even if drawn on surface of cylinder of beam; this effects, e.g., Displacement or Velocity, which is drawn constant over cross section

        color ([float,float,float,float]): RGBA color of the object; if R==-1, use default color

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
        name (str): objects's unique name

        physicsThickness (array_like): [SI:m] thickness of plate either provided as scalar or as vector (4 values, same order as local element node numbers) values that are linearly interpolated from nodal values; dimensionality must agree between thickness, strainCoefficients and curvatureCoefficients

        physicsDensity (float): [SI:kg/m:math:`^3`] density of the plate, possibly averaged over thickness

        physicsMassProportionalDamping (float): mass-proportional damping coefficient :math:`\alpha` [SI:1/s]; adds massmatrix proportional damping forces :math:`\fv_d = \alpha \Mm \dot{\qv}`

        physicsStrainCoefficients (Matrix3DList): [SI:N/m] stiffness coefficients related to inplane normal and shear strains, integrated over height of the plate; either given as 3D Matrix (numpy array), or a list of 3D matrices at each nodal point, see thickness; dimensionality must agree between thickness, strainCoefficients and curvatureCoefficients

        physicsCurvatureCoefficients (Matrix3DList): [SI:Nm] stiffness coefficients related to curvatures, integrated over height of the plate; either given as 3D Matrix (numpy array), or a list of 3D matrices at each nodal point, see thickness; dimensionality must agree between thickness, strainCoefficients and curvatureCoefficients

        strainIsRelativeToReference (float): if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration serves as a reference geometry; allows also values between 0. and 1. to perform a transition during static computation

        slopesScalingX ([float,float,float,float]): scaling of x-slopes at each element node; flat elements: half of the side length of the element; curved: optimal values such that curved geometry is best approximated; if negative (default) values are used, length is computed from node distances.

        slopesScalingY ([float,float,float,float]): scaling of y-slopes at each element node; flat elements: half of the side length of the element; curved: optimal values such that curved geometry is best approximated; if negative (default) values are used, length is computed from node distances.

        nodeNumbers (NodeIndex4): 4 NodePointSlope12 node numbers, with local (xi,eta) coordinates as [(-1,-1),(1,-1),(1,1),(-1,1)]

        useReducedOrderIntegration (int): 0/false: use highest Gauss integration for virtual work of strains

    Notes:
        Object has/provides the following types: ``Body``, ``MultiNoded``

        Requested Node type: ``Position``

    """
    def __init__(self, name = '', physicsThickness = [], physicsDensity = 0., physicsMassProportionalDamping = 0., physicsStrainCoefficients = None, physicsCurvatureCoefficients = None, strainIsRelativeToReference = 1., slopesScalingX = [-1.,-1.,-1.,-1.], slopesScalingY = [-1.,-1.,-1.,-1.], nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex(), exudyn.InvalidIndex(), exudyn.InvalidIndex()], useReducedOrderIntegration = 0, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.physicsThickness = CheckForValidNumpyArray(physicsThickness)
        self.physicsDensity = CheckForValidUReal(physicsDensity,"physicsDensity","ObjectANCFThinPlate")
        self.physicsMassProportionalDamping = physicsMassProportionalDamping
        self.physicsStrainCoefficients = physicsStrainCoefficients
        self.physicsCurvatureCoefficients = physicsCurvatureCoefficients
        self.strainIsRelativeToReference = strainIsRelativeToReference
        self.slopesScalingX = np.array(slopesScalingX)
        self.slopesScalingY = np.array(slopesScalingY)
        self.nodeNumbers = copy.copy(nodeNumbers)
        self.useReducedOrderIntegration = useReducedOrderIntegration
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ANCFThinPlate'
        yield 'name', self.name
        yield 'physicsThickness', self.physicsThickness
        yield 'physicsDensity', self.physicsDensity
        yield 'physicsMassProportionalDamping', self.physicsMassProportionalDamping
        yield 'physicsStrainCoefficients', self.physicsStrainCoefficients
        yield 'physicsCurvatureCoefficients', self.physicsCurvatureCoefficients
        yield 'strainIsRelativeToReference', self.strainIsRelativeToReference
        yield 'slopesScalingX', self.slopesScalingX
        yield 'slopesScalingY', self.slopesScalingY
        yield 'nodeNumbers', self.nodeNumbers
        yield 'useReducedOrderIntegration', self.useReducedOrderIntegration
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

class VObjectConnectorSpringDamper:
    """Visualization data for ObjectConnectorSpringDamper.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size = diameter of spring; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): connector's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        referenceLength (float): reference length [SI:m] of spring

        stiffness (float): stiffness [SI:N/m] of spring; force acts against (length-initialLength)

        damping (float): damping [SI:N/(m s)] of damper; force acts against d/dt(length)

        force (float): added constant force [SI:N] of spring; scalar force; f=1 is equivalent to reducing initialLength by 1/stiffness; f > 0: tension; f < 0: compression; can be used to model actuator force

        velocityOffset (float): velocity offset [SI:m/s] of damper, being equivalent to time change of reference length

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

        springForceUserFunction (PyFunctionMbsScalarIndexScalar5): A Python function which defines the spring force with parameters; the Python function will only be evaluated, if activeConnector is true, otherwise the SpringDamper is inactive; see description below

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position``

    """
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
    """Visualization data for ObjectConnectorCartesianSpringDamper.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size = diameter of spring; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): connector's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        stiffness ([float,float,float]): stiffness [SI:N/m] of springs; act against relative displacements in 0, 1, and 2-direction

        damping ([float,float,float]): damping [SI:N/(m s)] of dampers; act against relative velocities in 0, 1, and 2-direction

        offset ([float,float,float]): offset between two springs

        springForceUserFunction (PyFunctionVector3DmbsScalarIndexScalar4Vector3D): A Python function which computes the 3D force vector between the two marker points, if activeConnector=True; see description below

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position``

    """
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
    """Visualization data for ObjectConnectorRigidBodySpringDamper.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size = diameter of spring; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): connector's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        nodeNumber (NodeIndex): node number of a NodeGenericData (size depends on application) for dataCoordinates for user functions (e.g., implementing contact/friction user function)

        stiffness (array_like): stiffness [SI:N/m or Nm/rad] of translational, torsional and coupled springs; act against relative displacements in x, y, and z-direction as well as the relative angles (calculated as Euler angles); in the simplest case, the first 3 diagonal values correspond to the local stiffness in x,y,z direction and the last 3 diagonal values correspond to the rotational stiffness around x,y and z axis

        damping (array_like): damping [SI:N/(m/s) or Nm/(rad/s)] of translational, torsional and coupled dampers; very similar to stiffness, however, the rotational velocity is computed from the angular velocity vector

        rotationMarker0 (array_like): local rotation matrix for marker 0; stiffness, damping, etc. components are measured in local coordinates relative to rotationMarker0

        rotationMarker1 (array_like): local rotation matrix for marker 1; stiffness, damping, etc. components are measured in local coordinates relative to rotationMarker1

        offset (array_like): translational and rotational offset considered in the spring force calculation

        intrinsicFormulation (bool): if True, the joint uses the intrinsic formulation, which is independent on order of markers, using a mid-point and mid-rotation for evaluation and application of connector forces and torques; this uses a Lie group formulation; in this case, the force/torque vector is computed from the stiffness matrix times the 6-vector of the SE3 matrix logarithm between the two marker positions/rotations, see the equations

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

        springForceTorqueUserFunction (PyFunctionVector6DmbsScalarIndex4Vector3D2Matrix6D2Matrix3DVector6D): A Python function which computes the 6D force-torque vector (3D force + 3D torque) between the two rigid body markers, if activeConnector=True; see description below

        postNewtonStepUserFunction (PyFunctionVectorMbsScalarIndex4VectorVector3D2Matrix6D2Matrix3DVector6D): A Python function which computes the error of the PostNewtonStep; see description below

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position`` + ``Orientation``

        Requested Node type: ``GenericData``

    """
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
    """Visualization data for ObjectConnectorLinearSpringDamper.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size = diameter of spring; size == -1.f means that default connector size is used

        drawAsCylinder (bool): if this flag is True, the spring-damper is represented as cylinder; this may fit better if the spring-damper represents an actuator

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): connector's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        stiffness (float): torsional stiffness [SI:Nm/rad] against relative rotation

        damping (float): torsional damping [SI:Nm/(rad/s)]

        axisMarker0 ([float,float,float]): local axis of spring-damper in marker 0 coordinates; this axis will co-move with marker :math:`m0`; if marker m0 is attached to ground, the spring-damper represents linear equations

        offset (float): translational offset considered in the spring force calculation (this can be used as position control input!)

        velocityOffset (float): velocity offset considered in the damper force calculation (this can be used as velocity control input!)

        force (float): additional constant force [SI:Nm] added to spring-damper; this can be used to prescribe a force between the two attached bodies (e.g., for actuation and control)

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

        springForceUserFunction (PyFunctionMbsScalarIndexScalar5): A Python function which computes the scalar force between the two rigid body markers along axisMarker0 in :math:`m0` coordinates, if activeConnector=True; see description below

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position`` + ``Orientation``

    """
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
    """Visualization data for ObjectConnectorTorsionalSpringDamper.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size = diameter of spring; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): connector's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        nodeNumber (NodeIndex): node number of a NodeGenericData with 1 dataCoordinate for continuous rotation reconstruction; if this node is left to invalid index, it will not be used

        stiffness (float): torsional stiffness [SI:Nm/rad] against relative rotation

        damping (float): torsional damping [SI:Nm/(rad/s)]

        rotationMarker0 (array_like): local rotation matrix for marker 0; transforms joint into marker coordinates

        rotationMarker1 (array_like): local rotation matrix for marker 1; transforms joint into marker coordinates

        offset (float): rotational offset considered in the spring torque calculation (this can be used as rotation control input!)

        velocityOffset (float): angular velocity offset considered in the damper torque calculation (this can be used as angular velocity control input!)

        torque (float): additional constant torque [SI:Nm] added to spring-damper; this can be used to prescribe a torque between the two attached bodies (e.g., for actuation and control)

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

        springTorqueUserFunction (PyFunctionMbsScalarIndexScalar5): A Python function which computes the scalar torque between the two rigid body markers in local joint0 coordinates, if activeConnector=True; see description below

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Orientation``

        Requested Node type: ``GenericData``

    """
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
    """Visualization data for ObjectConnectorCoordinateSpringDamper.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size = diameter of spring; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): connector's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        stiffness (float): stiffness [SI:N/m] of spring; acts against relative value of coordinates

        damping (float): damping [SI:N/(m s)] of damper; acts against relative velocity of coordinates

        offset (float): offset between two coordinates (reference length of springs), see equation

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

        springForceUserFunction (PyFunctionMbsScalarIndexScalar5): A Python function which defines the spring force with 8 parameters, see equations section / see description below

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Coordinate``

    """
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
    """Visualization data for ObjectConnectorCoordinateSpringDamperExt.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size = diameter of spring; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): connector's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        nodeNumber (NodeIndex): node number of a NodeGenericData for 3 data coordinates (friction mode, last sticking position, limit stop state), see description for details; must exist in case of bristle friction model or limit stops

        stiffness (float): stiffness [SI:N/m] of spring; acts against relative value of coordinates

        damping (float): damping [SI:N/(m s)] of damper; acts against relative velocity of coordinates

        offset (float): offset between two coordinates (reference length of springs), see equation; it can be used to represent the pre-scribed drive coordinate

        velocityOffset (float): offset between two coordinates; used to model D-control of a drive, where damping is not acting against prescribed velocity

        factor0 (float): marker 0 coordinate is multiplied with factor0

        factor1 (float): marker 1 coordinate is multiplied with factor1

        fDynamicFriction (float): dynamic (viscous) friction force [SI:N] against relative velocity when sliding; assuming a normal force :math:`f_N`, the friction force can be interpreted as :math:`f_\mu = \mu f_N`

        fStaticFrictionOffset (float): static (dry) friction offset force [SI:N]; assuming a normal force :math:`f_N`, the friction force is limited by :math:`f_\mu \le (\mu_{so} + \mu_d) f_N = f_{\mu_d} + f_{\mu_{so}}`

        stickingStiffness (float): stiffness of bristles in sticking case  [SI:N/m]

        stickingDamping (float): damping of bristles in sticking case  [SI:N/(m/s)]

        exponentialDecayStatic (float): relative velocity for exponential decay of static friction offset force [SI:m/s] against relative velocity; at :math:`\Delta v = v_\mathrm{exp}`, the static friction offset force is reduced to 36.8%

        fViscousFriction (float): viscous friction force part [SI:N/(m s)], acting against relative velocity in sliding case

        frictionProportionalZone (float): if non-zero, a regularized Stribeck model is used, regularizing friction force around zero velocity - leading to zero friction force in case of zero velocity; this does not require a data node at all; if zero, the bristle model is used, which requires a data node which contains previous friction state and last sticking position

        limitStopsUpper (float): upper (maximum) value [SI:m] of coordinate before limit is activated; defined relative to the two marker coordinates

        limitStopsLower (float): lower (minimum) value [SI:m] of coordinate before limit is activated; defined relative to the two marker coordinates

        limitStopsStiffness (float): stiffness [SI:N/m] of limit stop (contact stiffness); following a linear contact model

        limitStopsDamping (float): damping [SI:N/(m/s)] of limit stop (contact damping); following a linear contact model

        useLimitStops (bool): if True, limit stops are considered and parameters must be set accordingly; furthermore, the NodeGenericData must have 3 data coordinates

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

        springForceUserFunction (PyFunctionMbsScalarIndexScalar11): A Python function which defines the spring force with 8 parameters, see equations section / see description below

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Coordinate``

        Requested Node type: ``GenericData``

    """
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
    """Visualization data for ObjectConnectorGravity.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size = diameter of spring; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

    """
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
    """A connector for additing forces due to gravitational fields beween two bodies, which can be used for aerospace and small-scale astronomical problems.
    
    NOTE: DO NOT USE this connector for adding gravitational forces (loads), which should be using LoadMassProportional, which is acting global and always in the same direction.
    
    Args:
        name (str): connector's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        gravitationalConstant (float): gravitational constant [SI:m:math:`^3`kg:math:`^{-1}`s:math:`^{-2}`)]; while not recommended, a negative constant gan represent a repulsive force

        mass0 (float): mass [SI:kg] of object attached to marker :math:`m0`

        mass1 (float): mass [SI:kg] of object attached to marker :math:`m1`

        minDistanceRegularization (float): distance [SI:m] at which a regularization is added in order to avoid singularities, if objects come close

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position``

    """
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
    """Visualization data for ObjectConnectorHydraulicActuatorSimple.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        cylinderRadius (float): radius for drawing of cylinder

        rodRadius (float): radius for drawing of rod

        pistonRadius (float): radius for drawing of piston (if drawn transparent)

        pistonLength (float): radius for drawing of piston (if drawn transparent)

        rodMountRadius (float): radius for drawing of rod mount sphere

        baseMountRadius (float): radius for drawing of base mount sphere

        baseMountLength (float): radius for drawing of base mount sphere

        colorCylinder ([float,float,float,float]): RGBA cylinder color; if R==-1, use default connector color

        colorPiston ([float,float,float,float]): RGBA piston color

    """
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
    r"""A basic hydraulic actuator with pressure build up equations.
    
    The actuator follows a valve input value, which results in a in- or outflow of fluid depending on the pressure difference. Valve values can be prescribed by user functions (not yet available) or with the ``MainSystem`` ``PreStepUserFunction(...)``.
    
    Args:
        name (str): connector's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        nodeNumbers (ArrayNodeIndex): currently a list with one node number of NodeGenericODE1 for 2 hydraulic pressures (reference values for this node must be zero); data node may be added in future for switching

        offsetLength (float): offset length [SI:m] of cylinder, representing minimal distance between the two bushings at stroke=0

        strokeLength (float): stroke length [SI:m] of cylinder, representing maximum extension relative to :math:`L_o`; the measured distance between the markers is :math:`L_s+L_o`

        chamberCrossSection0 (float): cross section [SI:m:math:`^2`] of chamber (inner cylinder) at piston head (nut) side (0)

        chamberCrossSection1 (float): cross section [SI:m:math:`^2`] of chamber at piston rod side (1); usually smaller than chamberCrossSection0

        hoseVolume0 (float): hose volume [SI:m:math:`^3`] at piston head (nut) side (0); as the effective bulk modulus would go to infinity at stroke length zero, the hose volume must be greater than zero

        hoseVolume1 (float): hose volume [SI:m:math:`^3`] at piston rod side (1); as the effective bulk modulus would go to infinity at max. stroke length, the hose volume must be greater than zero

        valveOpening0 (float): relative opening of valve :math:`[-1 \ldots 1]` [SI:1] at piston head (nut) side (0); positive value is valve opening towards system pressure, negative value is valve opening towards tank pressure; zero means closed valve

        valveOpening1 (float): relative opening of valve :math:`[-1 \ldots 1]` [SI:1] at piston rod side (1); positive value is valve opening towards system pressure, negative value is valve opening towards tank pressure; zero means closed valve

        actuatorDamping (float): damping [SI:N/(m:math:`\,`s)] of hydraulic actuator (against actuator axial velocity)

        oilBulkModulus (float): bulk modulus of oil [SI:N/(m:math:`^2`)]

        cylinderBulkModulus (float): bulk modulus of cylinder [SI:N/(m:math:`^2`)]; in fact, this is value represents the effect of the cylinder stiffness on the effective bulk modulus

        hoseBulkModulus (float): bulk modulus of hose [SI:N/(m:math:`^2`)]; in fact, this is value represents the effect of the hose stiffness on the effective bulk modulus

        nominalFlow (float): nominal flow of oil through valve [SI:m:math:`^3`/s]

        systemPressure (float): system pressure [SI:N/(m:math:`^2`)]

        tankPressure (float): tank pressure [SI:N/(m:math:`^2`)]

        useChamberVolumeChange (bool): if True, the pressure build up equations include the change of oil stiffness due to change of chamber volume

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position``

    """
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
    """Visualization data for ObjectConnectorReevingSystemSprings.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        ropeRadius (float): radius of rope

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): connector's unique name

        markerNumbers (ArrayMarkerIndex): list of position or rigid body markers used in reeving system and optional two coordinate markers (:math:`m_{c0}, \, m_{c1}`); the first marker :math:`m_0` and the last rigid body marker :math:`m_{nr-1}` represent the ends of the rope and are directly connected to a position; the markers :math:`m_1, \, \ldots, \, m_{nr-2}` can be connected to sheaves, for which a radius and an axis can be prescribed. The coordinate markers are optional and represent prescribed length at the rope ends (marker :math:`m_{c0}` is added length at start, marker :math:`m_{c1}` is added length at end of the rope in the reeving system)

        hasCoordinateMarkers (bool): flag, which determines, the list of markers (markerNumbers) contains two coordinate markers at the end of the list, representing the prescribed change of length at both ends

        coordinateFactors ([float,float]): factors which are multiplied with the values of coordinate markers; this can be used, e.g., to change directions or to transform rotations (revolutions of a sheave) into change of length

        stiffnessPerLength (float): stiffness per length [SI:N/m/m] of rope; in case of cross section :math:`A` and Young's modulus :math:`E`, this parameter results in :math:`E\cdot A`; the effective stiffness of the reeving system is computed as :math:`EA/L` in which :math:`L` is the current length of the rope

        dampingPerLength (float): axial damping per length [SI:N/(m/s)/m] of rope; the effective damping coefficient of the reeving system is computed as :math:`DA/L` in which :math:`L` is the current length of the rope

        dampingTorsional (float): torsional damping [SI:Nms] between sheaves; this effect can damp rotations around the rope axis, pairwise between sheaves; this parameter is experimental

        dampingShear (float): damping of shear motion [SI:Ns] between sheaves; this effect can damp motion perpendicular to the rope between each pair of sheaves; this parameter is experimental

        regularizationForce (float): small regularization force [SI:N] in order to avoid large compressive forces; this regularization force can either be :math:`<0` (using a linear tension/compression spring model) or :math:`>0`, which restricts forces in the rope to be always :math:`\ge -F_{reg}`. Note that smaller forces lead to problems in implicit integrators and smaller time steps. For explicit integrators, this force can be chosen close to zero.

        referenceLength (float): reference length for computation of roped force

        sheavesAxes (Vector3DList): list of local vectors axes of sheaves; vectors refer to rigid body markers given in list of markerNumbers; first and last axes are ignored, as they represent the attachment of the rope ends

        sheavesRadii (array_like): radius for each sheave, related to list of markerNumbers and list of sheaveAxes; first and last radii must always be zero.

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``_None``

    """
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
    """Visualization data for ObjectConnectorDistance.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size = link size; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        distance (float): prescribed distance [SI:m] of the used markers; must by greater than zero

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``Position``

    """
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
    """Visualization data for ObjectConnectorCoordinate.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size = link size; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        offset (float): An offset between the two values

        factorValue1 (float): An additional factor multiplied with value1 used in algebraic equation

        velocityLevel (bool): If true: connector constrains velocities (only works for ODE2 coordinates!); offset is used between velocities; in this case, the offsetUserFunction_t is considered and offsetUserFunction is ignored

        offsetUserFunction (PyFunctionMbsScalarIndexScalar): A Python function which defines the time-dependent offset; see description below

        offsetUserFunction_t (PyFunctionMbsScalarIndexScalar): time derivative of offsetUserFunction; needed for velocity level constraints; see description below

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``Coordinate``

    """
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
    """Visualization data for ObjectConnectorCoordinateVector.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

    """
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.color = np.array(color)

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))

class ObjectConnectorCoordinateVector:
    """A constraint which constrains the coordinate vectors of two markers Marker[Node|Object|Body]Coordinates attached to nodes or bodies.
    
    The marker uses the objects LTG-lists to build the according coordinate mappings.
    
    Args:
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        scalingMarker0 (array_like): linear scaling matrix for coordinate vector of marker 0; matrix provided in Python numpy format

        scalingMarker1 (array_like): linear scaling matrix for coordinate vector of marker 1; matrix provided in Python numpy format

        quadraticTermMarker0 (array_like): quadratic scaling matrix for coordinate vector of marker 0; matrix provided in Python numpy format

        quadraticTermMarker1 (array_like): quadratic scaling matrix for coordinate vector of marker 1; matrix provided in Python numpy format

        offset (array_like): offset added to constraint equation; only active, if no userFunction is defined

        velocityLevel (bool): If true: connector constrains velocities (only works for ODE2 coordinates!); offset is used between velocities; in this case, the offsetUserFunction_t is considered and offsetUserFunction is ignored

        constraintUserFunction (PyFunctionVectorMbsScalarIndex2VectorBool): A Python user function which computes the constraint equations; to define the number of algebraic equations, set scalingMarker0 as a numpy.zeros((nAE,1)) array with nAE being the number algebraic equations; see description below

        jacobianUserFunction (PyFunctionMatrixContainerMbsScalarIndex2VectorBool): A Python user function which computes the jacobian, i.e., the derivative of the left-hand-side object equation w.r.t. the coordinates (times :math:`f_{ODE2}`) and w.r.t. the velocities (times :math:`f_{ODE2_t}`). Terms on the RHS must be subtracted from the LHS equation; the respective terms for the stiffness matrix and damping matrix are automatically added; see description below

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``Coordinate``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], scalingMarker0 = [], scalingMarker1 = [], quadraticTermMarker0 = [], quadraticTermMarker1 = [], offset = [], velocityLevel = False, constraintUserFunction = 0, jacobianUserFunction = 0, activeConnector = True, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.]}):
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
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))

#add typedef for short usage:
CoordinateVectorConstraint = ObjectConnectorCoordinateVector
VCoordinateVectorConstraint = VObjectConnectorCoordinateVector

class VObjectConnectorRollingDiscPenalty:
    """Visualization data for ObjectConnectorRollingDiscPenalty.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        discWidth (float): width of disc for drawing

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector; :math:`m0` represents a point at the plane surface (normal of surface plane defined by planeNormal); the ground can also be a moving rigid body; :math:`m1` represents the rolling body, which has its reference point (=local position [0,0,0]) at the disc center point

        nodeNumber (NodeIndex): node number of a NodeGenericData (size=3) for 3 dataCoordinates, needed for discontinuous iteration (friction and contact)

        discRadius (float): defines the disc radius

        discAxis ([float,float,float]): axis of disc defined in marker :math:`m1` frame

        planeNormal ([float,float,float]): normal to the contact / rolling plane (ground); note that the plane reference point can be arbitrarily chosen by the location of the marker :math:`m0`

        dryFrictionAngle (float): angle [SI:1 (rad)] which defines a rotation of the local tangential coordinates dry friction; this allows to model Mecanum wheels with specified roll angle

        contactStiffness (float): normal contact stiffness [SI:N/m]

        contactDamping (float): normal contact damping [SI:N/(m s)]

        dryFriction ([float,float]): dry friction coefficients [SI:1] in local marker 1 joint :math:`J1` coordinates; if :math:`\alpha_t==0`, lateral direction :math:`l=x` and forward direction :math:`f=y`; assuming a normal force :math:`f_n`, the local friction force can be computed as :math:`{}^{J1}{\vp{f_{t,x}}{f_{t,y}}} = \vp{\mu_x f_n}{\mu_y f_n}`

        dryFrictionProportionalZone (float): limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations)

        viscousFriction ([float,float]): viscous friction coefficients [SI:1/(m/s)] in local marker 1 joint :math:`J1` coordinates; proportional to slipping velocity, leading to increasing slipping friction force for increasing slipping velocity

        rollingFrictionViscous (float): rolling friction [SI:1], which acts against the velocity of the trail on ground and leads to a force proportional to the contact normal force; currently, only implemented for disc axis parallel to ground!

        useLinearProportionalZone (bool): if True, a linear proportional zone is used; the linear zone performs better in implicit time integration as the Jacobian has a constant tangent in the sticking case

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position`` + ``Orientation``

        Requested Node type: ``GenericData``

    """
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
    """Visualization data for ObjectContactConvexRoll.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector; :math:`m0` represents the ground, which can undergo translations but not rotations, and :math:`m1` represents the rolling body, which has its reference point (=local position [0,0,0]) at the roll's center point

        nodeNumber (NodeIndex): node number of a NodeGenericData (size=3) for 3 dataCoordinates, needed for discontinuous iteration (friction and contact)

        contactStiffness (float): normal contact stiffness [SI:N/m]

        contactDamping (float): normal contact damping [SI:N/(m s)]

        dynamicFriction (float): dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, theDoc.pdf

        staticFrictionOffset (float): static friction offset for friction model (static friction = dynamic friction + static offset), see StribeckFunction in exudyn.physics, theDoc.pdf

        viscousFriction (float): viscous friction coefficient (velocity dependent part) for friction model, see StribeckFunction in exudyn.physics, theDoc.pdf

        exponentialDecayStatic (float): exponential decay of static friction offset (must not be zero!), see StribeckFunction in exudyn.physics (named expVel there!), theDoc.pdf

        frictionProportionalZone (float): limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), theDoc.pdf

        rollLength (float): roll length [m], symmetric w.r.t. centerpoint

        coefficientsHull (array_like): a vector of polynomial coefficients, which provides the polynomial of the CONVEX hull of the roll; :math:`\mathrm{hull}(x) = k_0 x^{n_p-1} + k x^{n_p-2} + \ldots + k_{n_p-2} x  + k_{n_p-1}`

        coefficientsHullDerivative (array_like): polynomial coefficients of the polynomial :math:`\mathrm{hull}^\prime(x)`

        coefficientsHullDDerivative (array_like): second derivative of the hull polynomial.

        rBoundingSphere (float): The  radius of the bounding sphere for the contact pre-check, calculated from the polynomial coefficients of the hull

        pContact ([float,float,float]): The  current potential contact point. Contact occures if pContact[2] < 0.

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position`` + ``Orientation``

        Requested Node type: ``GenericData``

    """
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
        self.coefficientsHull = CheckForValidNumpyArray(coefficientsHull)
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
    """Visualization data for ObjectContactCoordinate.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size = diameter of spring; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
    r"""A penalty-based contact condition for one coordinate; the contact gap :math:`g` is defined as :math:`g=marker.value[1]- marker.value[0] - offset`; the contact force :math:`f_c` is zero for :math:`gap>0` and otherwise computed from :math:`f_c = g*contactStiffness + \dot g*contactDamping`; during Newton iterations, the contact force is actived only, if :math:`dataCoordinate[0] <= 0`; dataCoordinate is set equal to gap in nonlinear iterations, but not modified in Newton iterations.
    
    Args:
        name (str): connector's unique name

        markerNumbers (ArrayMarkerIndex): markers define contact gap

        nodeNumber (NodeIndex): node number of a NodeGenericData for 1 dataCoordinate (used for active set strategy ==> holds the gap of the last discontinuous iteration)

        contactStiffness (float): contact (penalty) stiffness [SI:N/m]; acts only upon penetration

        contactDamping (float): contact damping [SI:N/(m s)]; acts only upon penetration

        offset (float): offset [SI:m] of contact

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Coordinate``

        Requested Node type: ``GenericData``

    """
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
    """Visualization data for ObjectContactCircleCable2D.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        showContactCircle (bool): if True and show=True, the underlying contact circle is shown; uses circleTiling*4 for tiling (from VisualizationSettings.general)

        drawSize (float): drawing size = diameter of spring; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
    r"""A very specialized penalty-based contact condition between a 2D circle (=marker0, any Position-marker) on a body and an ANCFCable2DShape (=marker1, Marker: BodyCable2DShape), in xy-plane.
    
    A node NodeGenericData is required with the number of cordinates according to the number of contact segments; the contact gap :math:`g` is integrated (piecewise linear) along the cable and circle; the contact force :math:`f_c` is zero for :math:`gap>0` and otherwise computed from :math:`f_c = g*contactStiffness + \dot g*contactDamping`; during Newton iterations, the contact force is actived only, if :math:`dataCoordinate[0] <= 0`; dataCoordinate is set equal to gap in nonlinear iterations, but not modified in Newton iterations.
    
    Args:
        name (str): connector's unique name

        markerNumbers (ArrayMarkerIndex): markers define contact gap

        nodeNumber (NodeIndex): node number of a NodeGenericData for nSegments dataCoordinates (used for active set strategy ==> hold the gap of the last discontinuous iteration and the friction state)

        numberOfContactSegments (int): number of linear contact segments to determine contact; each segment is a line and is associated to a data (history) variable; must be same as in according marker

        contactStiffness (float): contact (penalty) stiffness [SI:N/m/(contact segment)]; the stiffness is per contact segment; specific contact forces (per length) :math:`f_N` act in contact normal direction only upon penetration

        contactDamping (float): contact damping [SI:N/(m s)/(contact segment)]; the damping is per contact segment; acts in contact normal direction only upon penetration

        circleRadius (float): radius [SI:m] of contact circle

        offset (float): offset [SI:m] of contact, e.g. to include thickness of cable element

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``_None``

        Requested Node type: ``GenericData``

    """
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
    r"""Visualization data for ObjectContactFrictionCircleCable2D.
    
    Args:
        show (bool): set True, if item is shown in visualization and false if it is not shown; note that only normal contact forces can be  drawn, which are approximated by :math:`k_c \cdot g` (neglecting damping term)

        showContactCircle (bool): if True and show=True, the underlying contact circle is shown; uses circleTiling*4 for tiling (from VisualizationSettings.general)

        drawSize (float): drawing size = diameter of spring; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): connector's unique name

        markerNumbers (ArrayMarkerIndex): a marker :math:`m0` with position and orientation and a marker :math:`m1` of type BodyCable2DShape; together defining the contact geometry

        nodeNumber (NodeIndex): node number of a NodeGenericData with 3 :math:`\times n_{cs}`  dataCoordinates (used for active set strategy → hold the gap of the last discontinuous iteration, friction state (+-1=slip, 0=stick, -2=undefined) and the last sticking position; initialize coordinates with list [0.1]*:math:`n_{cs}`+[-2]*:math:`n_{cs}`+[0.]*:math:`n_{cs}`, meaning that there is no initial contact with undefined slip/stick

        numberOfContactSegments (int): number of linear contact segments to determine contact; each segment is a line and is associated to a data (history) variable; must be same as in according marker

        contactStiffness (float): contact (penalty) stiffness [SI:N/m/(contact segment)]; the stiffness is per contact segment; specific contact forces (per length) :math:`f_n` act in contact normal direction only upon penetration

        contactDamping (float): contact damping [SI:N/(m s)/(contact segment)]; the damping is per contact segment; acts in contact normal direction only upon penetration

        frictionVelocityPenalty (float): tangential velocity dependent penalty coefficient for friction [SI:N/(m s)/(contact segment)]; the coefficient causes tangential (contact) forces against relative tangential velocities in the contact area

        frictionStiffness (float): tangential displacement dependent penalty/stiffness coefficient for friction [SI:N/m/(contact segment)]; the coefficient causes tangential (contact) forces against relative tangential displacements in the contact area

        frictionCoefficient (float): friction coefficient [SI: 1]; tangential specific friction forces (per length) :math:`f_t` must fulfill the condition :math:`f_t \le \mu f_n`

        circleRadius (float): radius [SI:m] of contact circle

        useSegmentNormals (bool): True: use normal and tangent according to linear segment; this is appropriate for very long (compared to circle) segments; False: use normals at segment points according to vector to circle center; this is more consistent for short segments, as forces are only applied in beam tangent and normal direction

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``_None``

        Requested Node type: ``GenericData``

    """
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

class VObjectContactSphereSphere:
    """Visualization data for ObjectContactSphereSphere.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown; draws spheres by given radii

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): list of markers representing centers of spheres, used in connector

        nodeNumber (NodeIndex): node number of a NodeGenericData with numberOfDataCoordinates = 4 dataCoordinates, needed for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the  gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is the plastic overlap of the Edinburgh Adhesive Elasto-Plastic Model, initialized usually with 0 and set back to 0 in case that spheres have been separated.

        spheresRadii ([float,float]): list containing radius of sphere 0 and radius of sphere 1 [SI:m].

        isHollowSphere1 (bool): flag, which determines, if sphere attached to marker 1 (radius 1) is a hollow sphere.

        dynamicFriction (float): dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, theDoc.pdf

        frictionProportionalZone (float): limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), theDoc.pdf

        contactStiffness (float): normal contact stiffness [SI:N/m] (units in case that :math:`n_\mathrm{exp}=1`)

        contactDamping (float): linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.

        contactStiffnessExponent (float): exponent in normal contact model [SI:1]

        constantPullOffForce (float): constant adhesion force [SI:N]; Edinburgh Adhesive Elasto-Plastic Model

        contactPlasticityRatio (float): ratio of contact stiffness for first loading and unloading/reloading [SI:1]; Edinburgh Adhesive Elasto-Plastic Model; :math:`\lambda_\mathrm{P}=1-k_c/K2`, which gives the contact stiffness for unloading/reloading :math:`K2 = k_c/(1-\lambda_\mathrm{P})`; set to 0 in order to fully deactivate Edinburgh Adhesive Elasto-Plastic Model model

        adhesionCoefficient (float): coefficient for adhesion [SI:N/m] (units in case that :math:`n_\mathrm{adh}=1`); Edinburgh Adhesive Elasto-Plastic Model; set to 0 to deactivate adhesion model

        adhesionExponent (float): exponent for adhesion coefficient [SI:1]; Edinburgh Adhesive Elasto-Plastic Model

        restitutionCoefficient (float): coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)

        minimumImpactVelocity (float): minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)

        impactModel (int): number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position`` + ``Orientation``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), spheresRadii = [-1.,-1.], isHollowSphere1 = False, dynamicFriction = 0., frictionProportionalZone = 1e-3, contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1., constantPullOffForce = 0., contactPlasticityRatio = 0., adhesionCoefficient = 0., adhesionExponent = 1., restitutionCoefficient = 1., minimumImpactVelocity = 0., impactModel = 0, activeConnector = True, visualization = {'show': False, 'color': [0.7,0.7,0.7,1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.spheresRadii = np.array(spheresRadii)
        self.isHollowSphere1 = isHollowSphere1
        self.dynamicFriction = CheckForValidUReal(dynamicFriction,"dynamicFriction","ObjectContactSphereSphere")
        self.frictionProportionalZone = CheckForValidUReal(frictionProportionalZone,"frictionProportionalZone","ObjectContactSphereSphere")
        self.contactStiffness = CheckForValidUReal(contactStiffness,"contactStiffness","ObjectContactSphereSphere")
        self.contactDamping = CheckForValidUReal(contactDamping,"contactDamping","ObjectContactSphereSphere")
        self.contactStiffnessExponent = CheckForValidPReal(contactStiffnessExponent,"contactStiffnessExponent","ObjectContactSphereSphere")
        self.constantPullOffForce = CheckForValidUReal(constantPullOffForce,"constantPullOffForce","ObjectContactSphereSphere")
        self.contactPlasticityRatio = CheckForValidUReal(contactPlasticityRatio,"contactPlasticityRatio","ObjectContactSphereSphere")
        self.adhesionCoefficient = CheckForValidUReal(adhesionCoefficient,"adhesionCoefficient","ObjectContactSphereSphere")
        self.adhesionExponent = CheckForValidUReal(adhesionExponent,"adhesionExponent","ObjectContactSphereSphere")
        self.restitutionCoefficient = CheckForValidPReal(restitutionCoefficient,"restitutionCoefficient","ObjectContactSphereSphere")
        self.minimumImpactVelocity = CheckForValidUReal(minimumImpactVelocity,"minimumImpactVelocity","ObjectContactSphereSphere")
        self.impactModel = CheckForValidUInt(impactModel,"impactModel","ObjectContactSphereSphere")
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
        show (bool): set true, if item is shown in visualization and false if it is not shown; draws spheres by given radii

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): list of markers representing centers of sphere (marker 0) and center of torus (marker 1)

        nodeNumber (NodeIndex): node number of a NodeGenericData with numberOfDataCoordinates = 4 dataCoordinates, needed for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the  gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused.

        radiusSphere (float): radius of sphere [SI:m]

        torusMajorRadius (float): major radius of torus [SI:m], representing center of rotated circle

        torusMinorRadius (float): minor radius of torus [SI:m], representing radius of circle of ring

        torusAxis ([float,float,float]): Vector containing rotation axis of torus; must be a unit vector.

        dynamicFriction (float): dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, theDoc.pdf

        frictionProportionalZone (float): limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), theDoc.pdf

        contactStiffness (float): normal contact stiffness [SI:N/m] (units in case that :math:`n_\mathrm{exp}=1`)

        contactDamping (float): linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.

        contactStiffnessExponent (float): exponent in normal contact model [SI:1]

        restitutionCoefficient (float): coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)

        minimumImpactVelocity (float): minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)

        impactModel (int): number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position`` + ``Orientation``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), radiusSphere = 0., torusMajorRadius = 0., torusMinorRadius = 0., torusAxis = [0,0,0], dynamicFriction = 0., frictionProportionalZone = 1e-3, contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1., restitutionCoefficient = 1., minimumImpactVelocity = 0., impactModel = 0, activeConnector = True, visualization = {'show': False, 'color': [0.7,0.7,0.7,1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.radiusSphere = CheckForValidPReal(radiusSphere,"radiusSphere","ObjectContactSphereTorus")
        self.torusMajorRadius = CheckForValidPReal(torusMajorRadius,"torusMajorRadius","ObjectContactSphereTorus")
        self.torusMinorRadius = CheckForValidPReal(torusMinorRadius,"torusMinorRadius","ObjectContactSphereTorus")
        self.torusAxis = np.array(torusAxis)
        self.dynamicFriction = CheckForValidUReal(dynamicFriction,"dynamicFriction","ObjectContactSphereTorus")
        self.frictionProportionalZone = CheckForValidUReal(frictionProportionalZone,"frictionProportionalZone","ObjectContactSphereTorus")
        self.contactStiffness = CheckForValidUReal(contactStiffness,"contactStiffness","ObjectContactSphereTorus")
        self.contactDamping = CheckForValidUReal(contactDamping,"contactDamping","ObjectContactSphereTorus")
        self.contactStiffnessExponent = CheckForValidPReal(contactStiffnessExponent,"contactStiffnessExponent","ObjectContactSphereTorus")
        self.restitutionCoefficient = CheckForValidPReal(restitutionCoefficient,"restitutionCoefficient","ObjectContactSphereTorus")
        self.minimumImpactVelocity = CheckForValidUReal(minimumImpactVelocity,"minimumImpactVelocity","ObjectContactSphereTorus")
        self.impactModel = CheckForValidUInt(impactModel,"impactModel","ObjectContactSphereTorus")
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ContactSphereTorus'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'radiusSphere', self.radiusSphere
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

    def __repr__(self):
        return str(dict(self))

class VObjectContactSphereTriangle:
    """Visualization data for ObjectContactSphereTriangle.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown; draws spheres by given radii

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): list of markers representing the center of the sphere (marker 0) and the reference point of the triangle (marker 1), where triangle nodal positions are defined in the local coordinates of marker 1.

        nodeNumber (NodeIndex): node number of a NodeGenericData with numberOfDataCoordinates = 4 dataCoordinates, needed for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the  gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused.

        radiusSphere (float): radius of sphere [SI:m]

        trianglePoints (Vector3DList): triangle points, defined in marker 1 local coordinates

        includeEdges (int): Binary flag, where 1 defines contact with edges 0, 2 with edge 1 and 4 with edge 2; 7 means that contact with all edges is included; edge 0 is the edge between node 0 and node 1

        dynamicFriction (float): dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, theDoc.pdf

        frictionProportionalZone (float): limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), theDoc.pdf

        contactStiffness (float): normal contact stiffness [SI:N/m] (units in case that :math:`n_\mathrm{exp}=1`)

        contactDamping (float): linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.

        contactStiffnessExponent (float): exponent in normal contact model [SI:1]

        restitutionCoefficient (float): coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)

        minimumImpactVelocity (float): minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)

        impactModel (int): number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position`` + ``Orientation``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), radiusSphere = 0., trianglePoints = None, includeEdges = 7, dynamicFriction = 0., frictionProportionalZone = 1e-3, contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1., restitutionCoefficient = 1., minimumImpactVelocity = 0., impactModel = 0, activeConnector = True, visualization = {'show': False, 'color': [0.7,0.7,0.7,1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.radiusSphere = CheckForValidPReal(radiusSphere,"radiusSphere","ObjectContactSphereTriangle")
        self.trianglePoints = trianglePoints
        self.includeEdges = CheckForValidUInt(includeEdges,"includeEdges","ObjectContactSphereTriangle")
        self.dynamicFriction = CheckForValidUReal(dynamicFriction,"dynamicFriction","ObjectContactSphereTriangle")
        self.frictionProportionalZone = CheckForValidUReal(frictionProportionalZone,"frictionProportionalZone","ObjectContactSphereTriangle")
        self.contactStiffness = CheckForValidUReal(contactStiffness,"contactStiffness","ObjectContactSphereTriangle")
        self.contactDamping = CheckForValidUReal(contactDamping,"contactDamping","ObjectContactSphereTriangle")
        self.contactStiffnessExponent = CheckForValidPReal(contactStiffnessExponent,"contactStiffnessExponent","ObjectContactSphereTriangle")
        self.restitutionCoefficient = CheckForValidPReal(restitutionCoefficient,"restitutionCoefficient","ObjectContactSphereTriangle")
        self.minimumImpactVelocity = CheckForValidUReal(minimumImpactVelocity,"minimumImpactVelocity","ObjectContactSphereTriangle")
        self.impactModel = CheckForValidUInt(impactModel,"impactModel","ObjectContactSphereTriangle")
        self.activeConnector = activeConnector
        self.visualization = CopyDictLevel1(visualization)

    def __iter__(self):
        yield 'objectType', 'ContactSphereTriangle'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'radiusSphere', self.radiusSphere
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

    def __repr__(self):
        return str(dict(self))

class VObjectContactCurveCircles:
    """Visualization data for ObjectContactCurveCircles.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown; draws curve and circles with given radii; uses visualizationSettings circleTiling for circles and circleTiling/2 for tiling of non-straight segments

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
    
    The 2D curve may corotate in 3D with the underlying marker and also defines the plane of action for the circles. [REQUIRES FURTHER TESTING; friction not yet available]
    
    Args:
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): list of :math:`n_c+1` markers; marker :math:`m0` represents the marker carrying the curve; all other markers represent centers of :math:`n_c` circles, used in connector

        nodeNumber (NodeIndex): node number of a NodeGenericData with nDataVariablesPerSegment dataCoordinates per segment, needed for discontinuous iteration; data variables contain values from last PostNewton iteration: data[0+3*i] is the circle number, data[1+3*i] is the gap, data[2+3*i] is the tangential velocity (and thus contains information if it is stick or slip)

        circlesRadii (array_like): Vector containing radii of :math:`n_c` circles [SI:m]; number according to size of markerNumbers-1

        segmentsData (PyMatrixContainer): matrix containing a set of two planar point coordinates in each row, representing segments attached to marker :math:`m0` and undergoing contact with the circles; for segment :math:`s0` row 0 reads :math:`[p_{0x,s0},\,p_{0y,s0},\,p_{1x,s0},\,p_{1y,s0}]`; note that the segments must be ordered such that going from :math:`\mathbf{p}_0` to :math:`\mathbf{p}_1`, the exterior lies on the right (positive) side. MatrixContainer has to be provided in dense mode!

        polynomialData (PyMatrixContainer): matrix containing coefficients for special polynomial enhancements of the linear segments; each row contains coefficients for polynomials for the according segment, prescribing slopes at beginning and end of segment as well as curvature at beginning and end of segment; slopes and curvatures are defined in a local x/y coordinate system where x is the segment axis (start: x=0; x-axis points towards end point) and the segment normal is in y-direction; MatrixContainer has to be provided in dense mode!

        rotationMarker0 (array_like): local rotation matrix for marker 0; used to rotate marker coordinates such that the curve lies in the :math:`x-y`-plane

        dynamicFriction (float): dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, theDoc.pdf

        frictionProportionalZone (float): limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), theDoc.pdf

        contactStiffness (float): normal contact stiffness [SI:N/(m*m)]

        contactDamping (float): linear normal contact damping [SI:N/(m s)]; this damping is a simplification of real contact dissipation and should be used with care.

        contactModel (int): number of contact model: 0) linear model for stiffness and damping, only proportional to penetration; contact force is computed from :math:`l_\mathrm{seg}\left(p \cdot  \cdot k_c + \dot p \cdot d_c \right)` as long as :math:`p>0`; while this is numerically more stable, it gives jumps in forces when sliding over contact geometry 1) contact force proportional to integral over penetration area of circle with segments, giving a smoother contact force when sliding over geometry;

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

        gapPerSegment (array_like): temporary vector for computed gap

        gapPerSegment_t (array_like): temporary vector for computed gap velocity

        segmentsForceLocalX (array_like): temporary vector for contact force per segment in local X-direction

        segmentsForceLocalY (array_like): temporary vector for contact force per segment in local Y-direction

    Notes:
        Object has/provides the following types: ``Connector``

        Requested Marker type: ``Position`` + ``Orientation``

        Requested Node type: ``GenericData``

    """
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), circlesRadii = [], segmentsData = None, polynomialData = None, rotationMarker0 = IIDiagMatrix(rowsColumns=3,value=1), dynamicFriction = 0., frictionProportionalZone = 1e-3, contactStiffness = 0., contactDamping = 0., contactModel = 0, activeConnector = True, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = copy.copy(markerNumbers)
        self.nodeNumber = nodeNumber
        self.circlesRadii = CheckForValidNumpyArray(circlesRadii)
        self.segmentsData = segmentsData
        self.polynomialData = polynomialData
        self.rotationMarker0 = np.array(rotationMarker0)
        self.dynamicFriction = CheckForValidUReal(dynamicFriction,"dynamicFriction","ObjectContactCurveCircles")
        self.frictionProportionalZone = CheckForValidUReal(frictionProportionalZone,"frictionProportionalZone","ObjectContactCurveCircles")
        self.contactStiffness = contactStiffness
        self.contactDamping = contactDamping
        self.contactModel = CheckForValidUInt(contactModel,"contactModel","ObjectContactCurveCircles")
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
        yield 'rotationMarker0', self.rotationMarker0
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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        axesRadius (float): radius of joint axes to draw

        axesLength (float): length of joint axes to draw

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
    
    An additional local rotation (rotationMarker) can be used to adjust the three rotation axes and/or sliding axes.
    
    Args:
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        constrainedAxes (array_like): flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; for :math:`j_i`, two values are possible: 0=free axis, 1=constrained axis

        rotationMarker0 (array_like): local rotation matrix for marker :math:`m0`; translation and rotation axes for marker :math:`m0` are defined in the local body coordinate system and additionally transformed by rotationMarker0

        rotationMarker1 (array_like): local rotation matrix for marker :math:`m1`; translation and rotation axes for marker :math:`m1` are defined in the local body coordinate system and additionally transformed by rotationMarker1

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

        offsetUserFunctionParameters (array_like): vector of 6 parameters for joint's offsetUserFunction

        offsetUserFunction (PyFunctionVector6DmbsScalarIndexVector6D): A Python function which defines the time-dependent (fixed) offset of translation (indices 0,1,2) and rotation (indices 3,4,5) joint coordinates with parameters (mbs, t, offsetUserFunctionParameters)

        offsetUserFunction_t (PyFunctionVector6DmbsScalarIndexVector6D): (NOT IMPLEMENTED YET)time derivative of offsetUserFunction using the same parameters

        alternativeConstraints (bool): this is an experimental flag, may change in future: if uses alternative contraint equations for rotations, currently in case of 3 locked rotations: :math:`{}^{0}{\mathbf{t}}_{x0}\tp ({}^{0}{\mathbf{t}}_{y1} \times {}^{0}{\mathbf{t}}_{z0})`, :math:`{}^{0}{\mathbf{t}}_{y0}\tp ({}^{0}{\mathbf{t}}_{z1} \times {}^{0}{\mathbf{t}}_{x0})`, :math:`{}^{0}{\mathbf{t}}_{z0}\tp ({}^{0}{\mathbf{t}}_{x1} \times {}^{0}{\mathbf{t}}_{y0})`; this avoids 180textdegree flips of the standard configuration in static computations, but leads to different values in Lagrange multipliers

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``Position`` + ``Orientation``

    """
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
    """Visualization data for ObjectJointRevoluteZ.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        axisRadius (float): radius of joint axis to draw

        axisLength (float): length of joint axis to draw

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
    
    An additional local rotation (rotationMarker) can be used to transform the markers' coordinate systems into the joint coordinate system. For easier definition of the joint, use the exudyn.rigidbodyUtilities function AddRevoluteJoint(...), theDoc.pdf, for two rigid bodies (or ground). addExampleImage{RevoluteJointZ}
    
    Args:
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        rotationMarker0 (array_like): local rotation matrix for marker :math:`m0`; translation and rotation axes for marker :math:`m0` are defined in the local body coordinate system and additionally transformed by rotationMarker0

        rotationMarker1 (array_like): local rotation matrix for marker :math:`m1`; translation and rotation axes for marker :math:`m1` are defined in the local body coordinate system and additionally transformed by rotationMarker1

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        axisRadius (float): radius of joint axis to draw

        axisLength (float): length of joint axis to draw

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
    
    the joint :math:`y` and :math:`z` axes, allowing a relative motion along the joint :math:`x` axis (defined in local coordinates of marker 0 / joint J0 coordinates). An additional local rotation (rotationMarker) can be used to transform the markers' coordinate systems into the joint coordinate system. For easier definition of the joint, use the exudyn.rigidbodyUtilities function AddPrismaticJoint(...), theDoc.pdf, for two rigid bodies (or ground). addExampleImage{PrismaticJointX}
    
    Args:
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        rotationMarker0 (array_like): local rotation matrix for marker :math:`m0`; translation and rotation axes for marker :math:`m0` are defined in the local body coordinate system and additionally transformed by rotationMarker0

        rotationMarker1 (array_like): local rotation matrix for marker :math:`m1`; translation and rotation axes for marker :math:`m1` are defined in the local body coordinate system and additionally transformed by rotationMarker1

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        jointRadius (float): radius of joint to draw

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector; :math:`m1` is the moving coin rigid body and :math:`m0` is the marker for the ground body, which use the localPosition=[0,0,0] for this marker!

        constrainedAxes (array_like): flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; for :math:`j_i`, two values are possible: 0=free axis, 1=constrained axis

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        discWidth (float): width of disc for drawing

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector; :math:`m0` represents the ground and :math:`m1` represents the rolling body, which has its reference point (=local position [0,0,0]) at the disc center point

        constrainedAxes (array_like): flags, which determine which constraints are active, in which :math:`j_0` represents lateral motion, :math:`j_1` longitudinal (forward/backward) motion and :math:`j_2` represents the normal (contact) direction

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

        discRadius (float): defines the disc radius

        discAxis ([float,float,float]): axis of disc defined in marker :math:`m1` frame

        planeNormal ([float,float,float]): normal to the contact / rolling plane defined in marker :math:`m0` coordinates

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``Position`` + ``Orientation``

    """
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
    """Visualization data for ObjectJointRevolute2D.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size = radius of revolute joint; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size = radius of revolute joint; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): list of markers used in connector

        axisMarker0 ([float,float,float]): direction of prismatic axis, given as a 3D vector in Marker0 frame

        normalMarker1 ([float,float,float]): direction of normal to prismatic axis, given as a 3D vector in Marker1 frame

        constrainRotation (bool): flag, which determines, if the connector also constrains the relative rotation of the two objects; if set to false, the constraint will keep an algebraic equation set equal zero

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size = radius of revolute joint; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): marker m0: position or rigid body marker of mass point or rigid body; marker m1: updated marker to Cable2D element, where the sliding joint currently is attached to; must be initialized with an appropriate (global) marker number according to the starting position of the sliding object; this marker changes with time (PostNewtonStep)

        slidingMarkerNumbers (ArrayMarkerIndex): these markers are used to update marker m1, if the sliding position exceeds the current cable's range; the markers must be sorted such that marker :math:`m_{si}` at x=cable(i).length is equal to marker(i+1) at x=0 of cable(i+1)

        slidingMarkerOffsets (array_like): this list contains the offsets of every sliding object (given by slidingMarkerNumbers) w.r.t. to the initial position (0): marker m0: offset=0, marker m1: offset=Length(cable0), marker m2: offset=Length(cable0)+Length(cable1), ...

        nodeNumber (NodeIndex): node number of a NodeGenericData for 1 dataCoordinate showing the according marker number which is currently active and the start-of-step (global) sliding position

        constrainRotations (array_like): flags for constrained rotation about x, y and z-axis: if flag=1, add constraint on rotation of marker m0 relative to respective axis; flag=0: sliding body can rotate freely about this axis; for ANCFCable, rotation about x-axis cannot be constrained

        constrainTranslations (array_like): flags for constrained translation in x, y and z-direction: if flag=1, add constraint on translation of marker m0 relative to respective axis; flag=0: sliding body can translate freely about this axis; along x-axis this should be usually 0, except for driven motion

        axialForce (float): ONLY APPLIES if classicalFormulation==True; axialForce represents an additional sliding force acting between beam and marker m0 body in axial (beam) direction; this force can be used to drive a body on a beam, but can only be changed with user functions.

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size = radius of revolute joint; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): marker m0: position or rigid body marker of mass point or rigid body; marker m1: updated marker to Cable2D element, where the sliding joint currently is attached to; must be initialized with an appropriate (global) marker number according to the starting position of the sliding object; this marker changes with time (PostNewtonStep)

        slidingMarkerNumbers (ArrayMarkerIndex): these markers are used to update marker m1, if the sliding position exceeds the current cable's range; the markers must be sorted such that marker :math:`m_{si}` at x=cable(i).length is equal to marker(i+1) at x=0 of cable(i+1)

        slidingMarkerOffsets (array_like): this list contains the offsets of every sliding object (given by slidingMarkerNumbers) w.r.t. to the initial position (0): marker m0: offset=0, marker m1: offset=Length(cable0), marker m2: offset=Length(cable0)+Length(cable1), ...

        nodeNumber (NodeIndex): node number of a NodeGenericData for 1 dataCoordinate showing the according marker number which is currently active and the start-of-step (global) sliding position

        classicalFormulation (bool): True: uses a formulation with 3 (+1) equations, including the force in sliding direction to be zero; forces in global coordinates, only index 3; False: use local formulation, which only needs 2 (+1) equations and can be used with index 2 formulation

        constrainRotation (bool): True: add constraint on rotation of marker m0 relative to slope (if True, marker m0 must be a rigid body marker); False: marker m0 body can rotate freely

        axialForce (float): ONLY APPLIES if classicalFormulation==True; axialForce represents an additional sliding force acting between beam and marker m0 body in axial (beam) direction; this force can be used to drive a body on a beam, but can only be changed with user functions.

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

    Notes:
        Object has/provides the following types: ``Connector``, ``Constraint``

        Requested Marker type: ``_None``

        Requested Node type: ``GenericData``

    """
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
    """Visualization data for ObjectJointALEMoving2D.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

        drawSize (float): drawing size = radius of revolute joint; size == -1.f means that default connector size is used

        color ([float,float,float,float]): RGBA connector color; if R==-1, use default color

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
        name (str): constraints's unique name

        markerNumbers (ArrayMarkerIndex): marker m0: position-marker of mass point or rigid body; marker m1: updated marker to ANCF Cable2D element, where the sliding joint currently is attached to; must be initialized with an appropriate (global) marker number according to the starting position of the sliding object; this marker changes with time (PostNewtonStep)

        slidingMarkerNumbers (ArrayMarkerIndex): a list of sn (global) marker numbers which are are used to update marker1

        slidingMarkerOffsets (array_like): this list contains the offsets of every sliding object (given by slidingMarkerNumbers) w.r.t. to the initial position (0): marker0: offset=0, marker1: offset=Length(cable0), marker2: offset=Length(cable0)+Length(cable1), ...

        slidingOffset (float): sliding offset [SI:m]: a scalar offset, which represents the (reference arc) length of all previous sliding cable elements

        nodeNumbers (ArrayNodeIndex): node number of NodeGenericData (GD) with one data coordinate and of NodeGenericODE2 (ALE) with one ODE2 coordinate

        usePenaltyFormulation (bool): flag, which determines, if the connector is formulated with penalty, but still using algebraic equations (IsPenaltyConnector() still false)

        penaltyStiffness (float): penalty stiffness [SI:N/m] used if usePenaltyFormulation=True

        activeConnector (bool): flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

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
        name (str): marker's unique name

        bodyNumber (ObjectIndex): body number to which marker is attached to

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

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
        name (str): marker's unique name

        bodyNumber (ObjectIndex): body number to which marker is attached to

        localPosition ([float,float,float]): local body position of marker; e.g. local (body-fixed) position where force is applied to

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

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
        name (str): marker's unique name

        bodyNumber (ObjectIndex): body number to which marker is attached to

        localPosition ([float,float,float]): local body position of marker; e.g. local (body-fixed) position where force is applied to

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``, ``Position``, ``Orientation``

    """
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
    """Visualization data for MarkerNodePosition.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

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
        name (str): marker's unique name

        nodeNumber (NodeIndex): node number to which marker is attached to

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

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
        name (str): marker's unique name

        nodeNumber (NodeIndex): node number to which marker is attached to

    Notes:
        Marker has/provides the following types: ``Node``, ``Position``, ``Orientation``

    """
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
    """Visualization data for MarkerNodeCoordinate.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class MarkerNodeCoordinate:
    """A node-Marker attached to a ODE2 coordinate of a node; this marker allows to connect a coordinate-based constraint or connector to a nodal coordinate (also NodeGround); for ODE1 coordinates use ``MarkerNodeODE1Coordinate``.
    
    Args:
        name (str): marker's unique name

        nodeNumber (NodeIndex): node number to which marker is attached to

        coordinate (int): coordinate of node to which marker is attached to

    Notes:
        Marker has/provides the following types: ``Node``, ``Coordinate``

    """
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
    """Visualization data for MarkerNodeCoordinates.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class MarkerNodeCoordinates:
    """A node-Marker attached to all ODE2 coordinates of a node.
    
    IN CONTRAST to MarkerNodeCoordinate, the marker coordinates INCLUDE the reference values! For ODE1 coordinates use ``MarkerNodeODE1Coordinates``.
    
    Args:
        name (str): marker's unique name

        nodeNumber (NodeIndex): node number to which marker is attached to

    Notes:
        Marker has/provides the following types: ``Node``, ``Coordinate``

    """
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
    """Visualization data for MarkerNodeODE1Coordinate.
    
    Args:
        show (bool): currently not available; set true, if item is shown in visualization and false if it is not shown

    """
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class MarkerNodeODE1Coordinate:
    """A node-Marker attached to a ODE1 coordinate of a node.
    
    Args:
        name (str): marker's unique name

        nodeNumber (NodeIndex): node number to which marker is attached to

        coordinate (int): coordinate of node to which marker is attached to

    Notes:
        Marker has/provides the following types: ``Node``, ``Coordinate``

    """
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
    """Visualization data for MarkerNodeRotationCoordinate.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class MarkerNodeRotationCoordinate:
    """A node-Marker attached to a a node containing rotation; the Marker measures a rotation coordinate (Tait-Bryan angles) or angular velocities on the velocity level.
    
    Args:
        name (str): marker's unique name

        nodeNumber (NodeIndex): node number to which marker is attached to

        rotationCoordinate (int): rotation coordinate: 0=x, 1=y, 2=z

    Notes:
        Marker has/provides the following types: ``Node``, ``Coordinate``

    """
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

class VMarkerBodiesRelativeTranslationCoordinate:
    """Visualization data for MarkerBodiesRelativeTranslationCoordinate.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

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
        name (str): marker's unique name

        bodyNumbers (ArrayObjectIndex): list of body numbers for which relative coordinate is computed

        localPosition0 ([float,float,float]): local position on body 0; i.e. local (body-fixed) position where position is measured and force is applied to

        localPosition1 ([float,float,float]): local position on body 1; i.e. local (body-fixed) position where position is measured and force is applied to

        axis0 ([float,float,float]): axis defined in body 0, along which the relative translation is measured

        offset (float): translation offset [SI:m] subtracted from the translation; can be used to change the zero position

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``, ``Position``, ``Orientation``, ``Coordinate``

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

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
        name (str): marker's unique name

        bodyNumbers (ArrayObjectIndex): list of body numbers for which relative coordinate is computed

        nodeNumber (NodeIndex): node number of NodeGenericData with 1 coordinate which contains previous angle for continuation of angles (initialize accordingly if needed); if node is not supplied, angles will have jump outside :math:`\pm \pi`

        localPosition0 ([float,float,float]): local position on body 0; i.e. local (body-fixed) position where position is measured and force is applied to

        localPosition1 ([float,float,float]): local position on body 1; i.e. local (body-fixed) position where position is measured and force is applied to

        axis0 ([float,float,float]): axis defined in body 0, along which the relative rotation is measured

        offset (float): rotation offset [SI:1] subtracted from the measured rotation; can be used to change the zero rotation

    Notes:
        Marker has/provides the following types: ``Node``, ``Object``, ``Body``, ``Position``, ``Orientation``, ``Coordinate``

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        showMarkerNodes (bool): set true, if all nodes are shown (similar to marker, but with less intensity)

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
        name (str): marker's unique name

        bodyNumber (ObjectIndex): body number to which marker is attached to

        meshNodeNumbers (array_like): a list of :math:`n_m` mesh node numbers of superelement (=interface nodes) which are used to compute the body-fixed marker position; the related nodes must provide 3D position information, such as NodePoint, NodePoint2D, NodeRigidBody[..]; in order to retrieve the global node number, the generic body needs to convert local into global node numbers

        weightingFactors (array_like): a list of :math:`n_m` weighting factors per node to compute the final local position; the sum of these weights shall be 1, such that a summation of all nodal positions times weights gives the average position of the marker

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

        showMarkerNodes (bool): set true, if all nodes are shown (similar to marker, but with less intensity)

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
        name (str): marker's unique name

        bodyNumber (ObjectIndex): body number to which marker is attached to

        offset ([float,float,float]): local marker SuperElement reference position offset used to correct the center point of the marker, which is computed from the weighted average of reference node positions (which may have some offset to the desired joint position). Note that this offset shall be small and larger offsets can cause instability in simulation models (better to have symmetric meshes at joints).

        meshNodeNumbers (array_like): a list of :math:`n_m` mesh node numbers of superelement (=interface nodes) which are used to compute the body-fixed marker position and orientation; the related nodes must provide 3D position information, such as NodePoint, NodePoint2D, NodeRigidBody[..]; in order to retrieve the global node number, the generic body needs to convert local into global node numbers

        weightingFactors (array_like): a list of :math:`n_m` weighting factors per node to compute the final local position and orientation; these factors could be based on surface integrals of the constrained mesh faces

        useAlternativeApproach (bool): this flag switches between two versions for the computation of the rotation and angular velocity of the marker; alternative approach uses skew symmetric matrix of reference position; follows the inertia concept

        rotationsExponentialMap (int): Experimental flag (2 is the correct value and will be used in future, removing this flag): This value switches different behavior for computation of rotations and angular velocities: 0 uses linearized rotations and angular velocities, 1 uses the exponential map for rotations but linear angular velocities, 2 uses the exponential map for rotations and the according tangent map for angular velocities

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``, ``Position``, ``Orientation``

    """
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
    """Visualization data for MarkerKinematicTreeRigid.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

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
        name (str): marker's unique name

        objectNumber (ObjectIndex): body number to which marker is attached to

        linkNumber (int): number of link in KinematicTree to which marker is attached to

        localPosition ([float,float,float]): local (link-fixed) position of marker at link :math:`n_l`, using the link (:math:`n_l`) coordinate system

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``, ``Position``, ``Orientation``

    """
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
    """Visualization data for MarkerObjectODE2Coordinates.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class MarkerObjectODE2Coordinates:
    """A Marker attached to all coordinates of an object (currently only body is possible), e.g. to apply special constraints or loads on all coordinates.
    
    The measured coordinates INCLUDE reference + current coordinates.
    
    Args:
        name (str): marker's unique name

        objectNumber (ObjectIndex): body number to which marker is attached to

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``, ``Coordinate``

    """
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
    """Visualization data for MarkerBodyCable2DShape.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

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
        name (str): marker's unique name

        bodyNumber (ObjectIndex): body number to which marker is attached to

        numberOfSegments (int): number of number of segments; each segment is a line and is associated to a data (history) variable; must be same as in according contact element

        verticalOffset (float): vertical offset from beam axis in positive (local) Y-direction; this offset accounts for consistent computation of positions and velocities at the surface of the beam

    Notes:
        Marker has/provides the following types: ``Object``, ``Body``, ``Coordinate``

    """
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
    """Visualization data for MarkerBodyCable2DCoordinates.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

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
        name (str): marker's unique name

        bodyNumber (ObjectIndex): body number to which marker is attached to

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

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
        name (str): marker's unique name

        bodyNumber (ObjectIndex): body number to which marker is attached to (beam type)

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

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
        name (str): load's unique name

        markerNumber (MarkerIndex): marker's number to which load is applied

        loadVector ([float,float,float]): vector-valued load [SI:N]; in case of a user function, this vector is ignored

        bodyFixed (bool): if bodyFixed is true, the load is defined in body-fixed (local) coordinates, leading to a follower force; if false: global coordinates are used

        loadVectorUserFunction (PyFunctionVector3DmbsScalarVector3D): A Python function which defines the time-dependent load and replaces loadVector; see description below; NOTE that in static computations, the loadFactor is always 1 for forces computed by user functions (this means for the static computation, that a user function returning [t*5,t*1,0] corresponds to loadVector=[5,1,0] without a user function); NOTE that forces are drawn using the value of loadVector; thus the current values according to the user function are NOT shown in the render window; however, a sensor (SensorLoad) returns the user function force which is applied to the object; to draw forces with current user function values, use a graphicsDataUserFunction of a ground object

    Notes:
        Requested Marker type: ``Position``

    """
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
    """Visualization data for LoadTorqueVector.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

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
        name (str): load's unique name

        markerNumber (MarkerIndex): marker's number to which load is applied

        loadVector ([float,float,float]): vector-valued load [SI:N]; in case of a user function, this vector is ignored

        bodyFixed (bool): if bodyFixed is true, the load is defined in body-fixed (local) coordinates, leading to a follower torque; if false: global coordinates are used

        loadVectorUserFunction (PyFunctionVector3DmbsScalarVector3D): A Python function which defines the time-dependent load and replaces loadVector; see description below; see also notes on loadFactor and drawing in LoadForceVector! Example for Python function: def f(mbs, t, loadVector): return [loadVector[0]*np.sin(t*10*2*3.1415),0,0]

    Notes:
        Requested Marker type: ``Orientation``

    """
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
    """Visualization data for LoadMassProportional.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

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
        name (str): load's unique name

        markerNumber (MarkerIndex): marker's number to which load is applied

        loadVector ([float,float,float]): vector-valued load [SI:N/kg = m/s:math:`^2`]; typically, this will be the gravity vector in global coordinates; in case of a user function, this v is ignored

        loadVectorUserFunction (PyFunctionVector3DmbsScalarVector3D): A Python function which defines the time-dependent load; see description below; see also notes on loadFactor and drawing in LoadForceVector!

    Notes:
        Requested Marker type: ``Body`` + ``BodyMass``

    """
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
    """Visualization data for LoadCoordinate.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class LoadCoordinate:
    """Load with scalar value, which is attached to a coordinate-based marker; the load can be used e.g. to apply a force to a single axis of a body, a nodal coordinate of a finite element  or a torque to the rotatory DOF of a rigid body.
    
    Args:
        name (str): load's unique name

        markerNumber (MarkerIndex): marker's number to which load is applied

        load (float): scalar load [SI:N]; in case of a user function, this value is ignored

        loadUserFunction (PyFunctionMbsScalar2): A Python function which defines the time-dependent load and replaces the load; see description below; see also notes on loadFactor and drawing in LoadForceVector!

    Notes:
        Requested Marker type: ``Coordinate``

    """
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
    """Visualization data for SensorNode.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class SensorNode:
    """A sensor attached to a ODE2 or ODE1 node.
    
    The sensor measures OutputVariables and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.
    
    Args:
        name (str): sensor's unique name

        nodeNumber (NodeIndex): node number to which sensor is attached to

        writeToFile (bool): True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''

        fileName (str): directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist

        outputVariableType (OutputVariableType): OutputVariableType for sensor

        storeInternal (bool): true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available

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
        show (bool): set true, if item is shown in visualization and false if it is not shown; sensors can be shown at the position assiciated with the object - note that in some cases, there might be no such position (e.g. data object)!

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class SensorObject:
    """A sensor attached to any object except bodies  (connectors, constraint, spring-damper, etc).
    
    As a difference to other SensorBody, the connector sensor measures quantities without a local position. The sensor measures OutputVariable and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.
    
    Args:
        name (str): sensor's unique name

        objectNumber (ObjectIndex): object (e.g. connector) number to which sensor is attached to

        writeToFile (bool): True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''

        fileName (str): directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist

        outputVariableType (OutputVariableType): OutputVariableType for sensor

        storeInternal (bool): true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class SensorBody:
    r"""A sensor attached to a body-object with local position :math:`{}^{b}{\mathbf{b}}`.
    
    As a difference to SensorObject, the body sensor needs a local position at which the sensor is attached to. The sensor measures OutputVariableBody and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.
    
    Args:
        name (str): sensor's unique name

        bodyNumber (ObjectIndex): body (=object) number to which sensor is attached to

        localPosition ([float,float,float]): local (body-fixed) body position of sensor

        writeToFile (bool): True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''

        fileName (str): directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist

        outputVariableType (OutputVariableType): OutputVariableType for sensor

        storeInternal (bool): true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available

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
        show (bool): set true, if item is shown in visualization and false if it is not shown

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class SensorSuperElement:
    """A sensor attached to a SuperElement-object with mesh node number.
    
    As a difference to other ObjectSensors, the SuperElement sensor has a mesh node number at which the sensor is attached to. The sensor measures OutputVariableSuperElement and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.
    
    Args:
        name (str): sensor's unique name

        bodyNumber (ObjectIndex): body (=object) number to which sensor is attached to

        meshNodeNumber (int): mesh node number, which is a local node number with in the object (starting with 0); the node number may represent a real Node in mbs, or may be virtual and reconstructed from the object coordinates such as in ObjectFFRFreducedOrder

        writeToFile (bool): True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''

        fileName (str): directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist

        outputVariableType (OutputVariableType): OutputVariableType for sensor, based on the output variables available for the mesh nodes (see special section for super element output variables, e.g, in ObjectFFRFreducedOrder, theDoc.pdf)

        storeInternal (bool): true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available

    """
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
    """Visualization data for SensorKinematicTree.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class SensorKinematicTree:
    r"""A sensor attached to a KinematicTree with local position :math:`{}^{b}{\mathbf{b}}` and link number :math:`n_l`.
    
    As a difference to SensorBody, the KinematicTree sensor needs a local position and a link number, which defines the sub-body at which the sensor values are evaluated. The local position is given in sub-body (link) local coordinates. The sensor measures OutputVariableKinematicTree and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.
    
    Args:
        name (str): sensor's unique name

        objectNumber (ObjectIndex): object number of KinematicTree to which sensor is attached to

        linkNumber (int): number of link in KinematicTree to measure quantities

        localPosition ([float,float,float]): local (link-fixed) position of sensor, defined in link (:math:`n_l`) coordinate system

        writeToFile (bool): True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''

        fileName (str): directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist

        outputVariableType (OutputVariableType): OutputVariableType for sensor

        storeInternal (bool): true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available

    """
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
    """Visualization data for SensorMarker.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class SensorMarker:
    """A sensor attached to a marker.
    
    The sensor measures the selected marker values and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Depending on markers, it can measure Coordinates (MarkerNodeCoordinate), Position and Velocity (MarkerXXXPosition), Position, Velocity, Rotation and AngularVelocityLocal (MarkerXXXRigid). Note that marker values are only available for the current configuration. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file
    
    Args:
        name (str): sensor's unique name

        markerNumber (MarkerIndex): marker number to which sensor is attached to

        writeToFile (bool): True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''

        fileName (str): directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist

        outputVariableType (OutputVariableType): OutputVariableType for sensor; output variables are only possible according to markertype, see general description of SensorMarker

        storeInternal (bool): true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available

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
        show (bool): set true, if item is shown in visualization and false if it is not shown; sensor visualization CURRENTLY NOT IMPLEMENTED

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class SensorLoad:
    """A sensor attached to a load.
    
    The sensor measures the load values and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.
    
    Args:
        name (str): sensor's unique name

        loadNumber (LoadIndex): load number to which sensor is attached to

        writeToFile (bool): True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''

        fileName (str): directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist

        storeInternal (bool): true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available

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
    """Visualization data for SensorUserFunction.
    
    Args:
        show (bool): set true, if item is shown in visualization and false if it is not shown; sensor visualization CURRENTLY NOT IMPLEMENTED

    """
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))

class SensorUserFunction:
    """A sensor defined by a user function.
    
    The sensor is intended to collect sensor values of a list of given sensors and recombine the output into a new value for output or control purposes. It is also possible to use this sensor without any dependence on other sensors in order to generate output for, e.g., any quantities in mbs or solvers.
    
    Args:
        name (str): sensor's unique name

        sensorNumbers (ArraySensorIndex): optional list of :math:`n` sensor numbers for use in user function

        factors (array_like): optional list of :math:`m` factors which can be used, e.g., for weighting sensor values

        writeToFile (bool): True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''

        fileName (str): directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist

        sensorUserFunction (PyFunctionVectorMbsScalarArrayIndexVectorConfiguration): A Python function which defines the time-dependent user function, which usually evaluates one or several sensors and computes a new sensor value, see example

        storeInternal (bool): true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available

    """
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

