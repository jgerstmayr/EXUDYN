#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  This module provides an extension interface to the C++ class MainSystem;
#           MainSystem is extended by Python interface functions to easily create
#           bodies and point masses without the need to create an according node and
#           connectors and joints without the need to create markers.
#           Extensions are activated in __init__.py
#
# Author:   Johannes Gerstmayr and others (see functions)
# Date:     2023-05-07 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#import exudyn #does not work out of exudyn.__init__.py
from typing import Union
from exudyn.misc.docmeta import docmeta
from exudyn.misc.deprecation import DeprecatedArgument #the deprecations of the library (#2807)
import exudyn as exu
from exudyn.misc.extensionRegistry import extends, install
import exudyn.plot
import exudyn.solver
import exudyn.systemUtilities #its @extends functions: mbs.ComputeConstraintResiduals, mbs.DrawSystemGraph
import exudyn.interactive
import exudyn.graphics
from exudyn.basicUtilities import Normalize

from exudyn.rigidBodyUtilities import _MarkerWithRotation, _RotationMarkerArgs
import exudyn.cMath as cMath #the math functions of the C++ code, the same bits on every platform (#2962)
from exudyn.rigidBodyUtilities import ComputeOrthonormalBasis, \
    RotationMatrix2EulerParameters, AngularVelocity2EulerParameters_t, RotationMatrix2RotXYZ, AngularVelocity2RotXYZ_t, \
    RotationMatrix2RotationVector, StrNodeType2NodeType

import exudyn.itemInterface as eii
from exudyn.itemInterface import ObjectGround, VObjectGround, SensorUserFunction
from exudyn.advancedUtilities import RaiseTypeError, IsVector, ExpectedType, IsValidObjectIndex, IsValidRealInt, IsValidPRealInt, IsValidURealInt, IsIntVector, \
                                    IsValidBool, IsSquareMatrix, IsNone, IsNotNone, IsInteger, IsValidInt

import numpy as np
import copy


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#add helpful Python extensions for MainSystem, regarding creation of bodies, point masses, connectors and joints


#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'MainSystemCreateGround', 'MainSystemCreateMassPoint', 'MainSystemCreateRigidBody',
    'MainSystemCreateSpringDamper', 'MainSystemCreateCartesianSpringDamper',
    'MainSystemCreateRigidBodySpringDamper', 'MainSystemCreateTorsionalSpringDamper', 'MainSystemCreateLinearSpringDamper',
    'MainSystemCreateRevoluteJoint', 'MainSystemCreatePrismaticJoint',
    'MainSystemCreateSphericalJoint', 'MainSystemCreateGenericJoint',
    'MainSystemCreateDistanceConstraint', 'MainSystemCreateCoordinateConstraint',
    'MainSystemCreateRollingDisc', 'MainSystemCreateRollingDiscPenalty',
    'MainSystemCreateSphereSphereContact', 'MainSystemCreateSphereQuadContact',
    'MainSystemCreateSphereTriangleContact', 'MainSystemCreateKinematicTree',
    'MainSystemCreateFFRFReducedOrderObject', 'MainSystemCreateForce', 'MainSystemCreateTorque',
    'CreateDistanceSensorGeometry', 'CreateDistanceSensor',
    ]

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#WHY THESE TWO EXIST (#2502). Every Create*Joint below converts the joint
#position and orientation into body coordinates with a 3x3 product. numpy does not guarantee the
#summation order of a small matrix product, and it CHANGED between releases: for a component that
#is analytically zero, numpy 2.2.4 returns exactly 0.0 while numpy 2.4.6 returns -2.9e-19. Those
#last bits become the localPosition of a marker - solver INPUT - and a sensitive model amplifies
#them: sliderCrank3Dbenchmark.py moved by a relative 2.7e-10 against a suite tolerance of 5e-14,
#with a byte-identical C++ binary. A committed reference value was therefore only reproducible
#with the numpy version that produced it (information document, fact 28).
#
#The products below are written out, so the order is fixed by the source and not by which kernel
#numpy picks. Each term is one IEEE multiply and one IEEE add, which makes the result identical on
#every numpy version - and exact where the answer is exact. The cost is a Python-level loop over
#nine terms, paid once per joint at model-build time.
#
#Kept private and local: these twenty-odd call sites are the ones measured to matter. The same
#pattern appears elsewhere in the package (FEM.py, kinematicTree.py); those are not touched here
#because nothing measured makes them matter, and a public utility would be new API.
@docmeta(public=False)
def _MatVec3(A, v):
    """A @ v for 3x3 by 3, with a summation order fixed by this source; see the note above."""
    return np.array([A[i][0]*v[0] + A[i][1]*v[1] + A[i][2]*v[2] for i in range(3)])

@docmeta(public=False)
def _MatMul3x3(A, B):
    """A @ B for 3x3 by 3x3, with a summation order fixed by this source; see the note above."""
    return np.array([[A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j]
                      for j in range(3)] for i in range(3)])


#THE ITEMS A CREATE FUNCTION CONNECTS (#2863): itemNumbers, two entries, each a body (ObjectIndex, or an int) at a local
#position, a node (NodeIndex) or a marker (MarkerIndex); which kinds a function takes, its docstring says and it passes
#to CheckItemNumbers. bodyNumbers is the deprecated name of itemNumbers - each function reports it itself, so that the
#declared deprecations of the library name the function -, and bodyOrNodeList is not accepted.
_bodyNodeMarker = ('body', 'node', 'marker')
_kindNames = {'body': 'ObjectIndex', 'node': 'NodeIndex', 'marker': 'MarkerIndex', 'ground': 'None'}

@docmeta(public=False)
def ItemKind(item):
    """'body', 'node', 'marker', or None: what an entry of itemNumbers is; an int is a body"""
    if isinstance(item, exudyn.MarkerIndex):
        return 'marker'
    if isinstance(item, exudyn.NodeIndex):
        return 'node'
    if IsValidObjectIndex(item):
        return 'body'
    return None

@docmeta(public=False)
def ItemNumbersArgument(where, itemNumbers, bodyNumbers, bodyOrNodeList=None):
    """the items given to a Create function: itemNumbers, or bodyNumbers, its deprecated name, which the caller reports;
    a TypeError for both at once, and for bodyOrNodeList"""
    if bodyOrNodeList is not None:
        raise TypeError('ERROR in ' + where + ': the argument bodyOrNodeList is not accepted; use itemNumbers')
    if bodyNumbers is None:
        return itemNumbers
    if not (isinstance(itemNumbers, list) and all(item is None for item in itemNumbers)):
        raise TypeError('ERROR in ' + where + ': bodyNumbers and itemNumbers are given; use itemNumbers only')
    return bodyNumbers

@docmeta(public=False)
def CheckItemNumbers(where, itemNumbers, kinds, localPositions=None):
    """a TypeError unless itemNumbers is a list of two items, entry i of one of the kinds in kinds[i] ('body', 'node',
    'marker', 'ground' for None); a node or a marker has no local position, so localPositions[i], if given, is [0,0,0]"""
    if exudyn.__useExudynFast:
        return
    if not isinstance(itemNumbers, list) or len(itemNumbers) != 2:
        RaiseTypeError(where=where, argumentName='itemNumbers', received = itemNumbers, expectedType = 'list of 2 item numbers')
    for i in range(2):
        kind = 'ground' if itemNumbers[i] is None else ItemKind(itemNumbers[i])
        if kind not in kinds[i]:
            names = [_kindNames[k] for k in kinds[i]]
            RaiseTypeError(where=where, argumentName='itemNumbers['+str(i)+']', received = itemNumbers[i],
                           expectedType = ', '.join(names[:-1]) + (' or ' if len(names) > 1 else '') + names[-1])
        if (kind in ('node', 'marker') and localPositions is not None and IsVector(localPositions[i], 3)
                and any(x != 0 for x in localPositions[i])):
            RaiseTypeError(where=where, argumentName='localPosition'+str(i), received = localPositions[i],
                           expectedType = '[0,0,0] for a ' + _kindNames[kind])

@docmeta(public=False)
def ItemOutput(mbs, item, localPosition, variableType):
    """an output of a body at its local position, of a node or of a marker, in the reference configuration"""
    kind = ItemKind(item)
    if kind == 'body':
        return mbs.GetObjectOutputBody(item, variableType, localPosition=localPosition,
                                       configuration=exudyn.ConfigurationType.Reference)
    if kind == 'node':
        return mbs.GetNodeOutput(item, variableType, configuration=exudyn.ConfigurationType.Reference)
    return mbs.GetMarkerOutput(item, variableType, configuration=exudyn.ConfigurationType.Reference)


#internal function: do some pre-checks and calculations for joint
#extended function which also accepts markers in itemNumbers and returns new or existing markers
#marker0 overrides the joint "position"
@docmeta(public=False)
def JointPreCheckCalcBodyMarkers(where, mbs, name, itemNumbers, position, show, useGlobalFrame, requireRotMat=True):
    """Helper to calculate markers."""
    CheckItemNumbers(where, itemNumbers, [('body', 'marker'), ('body', 'marker')])
    if not exudyn.__useExudynFast:
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsValidBool(useGlobalFrame):
            RaiseTypeError(where=where, argumentName='useGlobalFrame', received = useGlobalFrame, expectedType = ExpectedType.Bool)
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

    mBody0 = itemNumbers[0] if isinstance(itemNumbers[0], exudyn.MarkerIndex) else None
    mBody1 = itemNumbers[1] if isinstance(itemNumbers[1], exudyn.MarkerIndex) else None

    if not exudyn.__useExudynFast:
        if mBody0 is not None or mBody1 is not None:
            if not IsVector(position) or len(position) != 0:
                raise ValueError('ERROR in ' + where + ' in argument "position": ' +
                                 'in case that a marker number is provided, position must be []')
        elif not IsVector(position, 3):
            RaiseTypeError(where=where, argumentName='position', received = position, expectedType = ExpectedType.Vector, dim=3)

    p0 = ItemOutput(mbs, itemNumbers[0], [0,0,0], exudyn.OutputVariableType.Position)
    A0 = ItemOutput(mbs, itemNumbers[0], [0,0,0], exudyn.OutputVariableType.RotationMatrix).reshape((3,3))
    p1 = ItemOutput(mbs, itemNumbers[1], [0,0,0], exudyn.OutputVariableType.Position)
    A1 = ItemOutput(mbs, itemNumbers[1], [0,0,0], exudyn.OutputVariableType.RotationMatrix).reshape((3,3))

    if mBody0 is not None:
        pJoint = p0 #marker sets the global joint position!
    elif mBody1 is not None:
        pJoint = p1
    elif useGlobalFrame:
        pJoint = copy.copy(position)
    else: #transform into global coordinates, then everything works same
        pJoint = A0 @ position + p0

    return [p0, A0, p1, A1, mBody0, mBody1, pJoint]


@docmeta(public=False)
def ItemMarker(mbs, name, i, item, localPosition, useRigidMarker):
    """the marker of entry i of itemNumbers: a new one for a body or a node, a rigid one or a position one, named
    Marker<i>:name; a marker is used as it is"""
    markerName = 'Marker'+str(i)+':'+name if name != '' else ''
    kind = ItemKind(item)
    if kind == 'body':
        MarkerBodyType = eii.MarkerBodyRigid if useRigidMarker else eii.MarkerBodyPosition
        return mbs.AddMarker(MarkerBodyType(name=markerName, bodyNumber=item, localPosition=localPosition))
    if kind == 'node':
        MarkerNodeType = eii.MarkerNodeRigid if useRigidMarker else eii.MarkerNodePosition
        return mbs.AddMarker(MarkerNodeType(name=markerName, nodeNumber=item))
    return item


#internal: the markers of the two items - a body or a node gets a new one, a marker is used as it is - and, if asked for,
#their positions and rotation matrices in the reference configuration
@docmeta(public=False)
def GetMarkersPosRot(mbs, name, itemNumbers, localPosition0, localPosition1,
                     getPosition=False, getRotationMatrix=False, useRigidMarker=False):
    """Helper to calculate marker pose."""
    markers = [None, None]
    positions = [None, None]
    rotations = [None, None]
    for (i, localPosition) in enumerate([localPosition0, localPosition1]):
        item = itemNumbers[i]
        markers[i] = ItemMarker(mbs, name, i, item, localPosition, useRigidMarker)
        if getPosition:
            positions[i] = ItemOutput(mbs, item, localPosition, exudyn.OutputVariableType.Position)
        if getRotationMatrix:
            rotations[i] = ItemOutput(mbs, item, localPosition, exudyn.OutputVariableType.RotationMatrix).reshape((3,3))

    return [markers[0], markers[1], positions[0], positions[1], rotations[0], rotations[1]]


def _RotationIntoMarker(mbs, marker, bodyNodeMarker, localPosition, rotation):
    """the rotation of a joint or connector on its side, given to the marker as localHT if the marker was created here
    (a MarkerBodyRigid or MarkerNodeRigid); returns the unit matrix, or the rotation itself for a marker the caller gave,
    which is not changed - _MarkerWithRotation then adds a turned copy of it (#2745, #2804)"""
    if isinstance(bodyNodeMarker, exudyn.MarkerIndex):
        return rotation
    if isinstance(bodyNodeMarker, exudyn.NodeIndex):
        mbs.SetMarkerParameter(marker, 'localHT', exu.HT(rotation=rotation))
    else:
        mbs.SetMarkerParameter(marker, 'localHT', exu.HT(rotation=rotation, translation=localPosition))
    return np.eye(3)


#internal: convert exudyn jointType to axis vector
@docmeta(public=False)
def JointTypeToAxis(jointType):
    """Helper for joint types."""
    if (jointType == exu.JointType.PrismaticX or jointType == exu.JointType.RevoluteX):
        axis = np.array([1,0,0])
    if (jointType == exu.JointType.PrismaticY or jointType == exu.JointType.RevoluteY):
        axis = np.array([0,1,0])
    if (jointType == exu.JointType.PrismaticZ or jointType == exu.JointType.RevoluteZ):
        axis = np.array([0,0,1])
    else:
        ValueError('JointTypeToAxis: invalid joint type:'+str(jointType))
    return axis





#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def _FrameFromHT(where, htName, ht, positionName, position, rotationName, rotation):
    """position and rotation matrix from an HT argument and its two parts, None meaning not given (#2794):
    the HT (a 4x4 matrix, its 16 values or an exu.HT) or the parts, not both; returns (position, rotation),
    each None if not given"""
    if ht is None:
        return position, rotation
    if position is not None or rotation is not None:
        raise ValueError(where + ': ' + htName + ' and ' + positionName + ' or ' + rotationName
                         + ' are given; give one of them, the other None')
    if not isinstance(ht, exu.HT):
        if np.array(ht).shape not in [(4, 4), (16,)]:
            raise ValueError(where + ': ' + htName + ' must be an exu.HT, a 4x4 matrix or its 16 values')
        ht = exu.HT(ht)
    return ht.translation, ht.rotation


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateGround(mbs,
                           name = '',   
                           referencePosition = None,
                           referenceRotationMatrix = None,
                           graphicsDataList = [],
                           graphicsDataUserFunction = 0,
                           show = True,
                           referenceHT = None) -> exudyn.ObjectIndex: 
    """helper function to create a ground object, using arguments of ObjectGround; this function is mainly added for consistency with other mainSystemExtensions

    Args:
        mbs: the MainSystem where items are created
        name: name string for object
        referencePosition: reference position of the ground (a 3D vector); None: zero
        referenceRotationMatrix: reference rotation matrix of the ground (a 3D matrix); None: the unit matrix
        referenceHT: referenceRotationMatrix and referencePosition at once, as homogeneous transformation: an exu.HT, a 4x4 matrix or its 16 values; None: not given; it raises if given together with referencePosition or referenceRotationMatrix
        graphicsDataList: list of GraphicsData for optional ground visualization
        graphicsDataUserFunction: a user function graphicsDataUserFunction(mbs, itemNumber)->BodyGraphicsData (list of GraphicsData), which can be used to draw user-defined graphics; this is much slower than regular GraphicsData
        color: color of node
        show: True: show ground object;

    Returns:
        :ObjectIndex: returns ground object index

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        ground=mbs.CreateGround(referencePosition = [2,0,0],
                                graphicsDataList = [exu.graphics.CheckerBoard(point=[0,0,0], normal=[0,1,0],size=4)])
    """
    referencePosition, referenceRotationMatrix = _FrameFromHT('MainSystem.CreateGround(...)', 'referenceHT', referenceHT,
        'referencePosition', referencePosition, 'referenceRotationMatrix', referenceRotationMatrix)
    if referencePosition is None:
        referencePosition = [0.,0.,0.]
    if referenceRotationMatrix is None:
        referenceRotationMatrix = np.eye(3)

    #error checks:
    if not exudyn.__useExudynFast:
        where='MainSystem.CreateGround(...)'
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)
        if not IsVector(referencePosition, 3):
            RaiseTypeError(where=where, argumentName='referencePosition', received = referencePosition, expectedType = ExpectedType.Vector, dim=3)

        if not IsSquareMatrix(referenceRotationMatrix, 3):
            RaiseTypeError(where=where, argumentName='referenceRotationMatrix', received = referenceRotationMatrix, expectedType = ExpectedType.Matrix, dim=3)
    
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
    
        if type(graphicsDataList) != list:
            raise ValueError(where+': graphicsDataList must be a (possibly empty) list of dictionaries of graphics data!')

    groundNumber = mbs.AddObject(eii.ObjectGround(name = name,
                                    referencePosition=referencePosition,
                                    referenceRotation=referenceRotationMatrix,
                                    visualization = eii.VObjectGround(show = show, 
                                                        graphicsDataUserFunction=graphicsDataUserFunction,
                                                        graphicsData = graphicsDataList) ))
    return groundNumber


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateMassPoint(mbs,
                           name = '',
                           referencePosition = [0.,0.,0.],
                           initialDisplacement = [0.,0.,0.],
                           initialVelocity = [0.,0.,0.],
                           mass=0,
                           gravity = [0.,0.,0.],
                           graphicsDataList = [],
                           drawSize = -1,
                           color =  [-1.,-1.,-1.,-1.],
                           show = True, 
                           create2D = False, 
                           returnDict = False, physicsMass=None) -> Union[dict, exudyn.ObjectIndex]: 
    """helper function to create 2D or 3D mass point object and node, using arguments as in NodePoint and MassPoint

    Args:
        mbs: the MainSystem where items are created
        name: name string for object, node is 'Node:'+name
        referencePosition: reference coordinates for point node (always a 3D vector, no matter if 2D or 3D mass)
        initialDisplacement: initial displacements for point node (always a 3D vector, no matter if 2D or 3D mass)
        initialVelocity: initial velocities for point node (always a 3D vector, no matter if 2D or 3D mass)
        mass: mass of mass point
        gravity: gravity vevtor applied (always a 3D vector, no matter if 2D or 3D mass)
        graphicsDataList: list of GraphicsData for optional mass visualization
        drawSize: general drawing size of node
        color: color of node
        show: True: if graphicsData list is empty, node is shown, otherwise body is shown; False: nothing is shown
        create2D: if True, create NodePoint2D and MassPoint2D
        returnDict: if False, returns object index; if True, returns dict of all information on created object and node
        physicsMass: deprecated name of mass

    Returns:
        :Union[dict, ObjectIndex]: returns mass point object index or dict with all data on request (if returnDict=True)

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        b0=mbs.CreateMassPoint(referencePosition = [0,0,0],
                               initialVelocity = [2,5,0],
                               mass = 1, gravity = [0,-9.81,0],
                               drawSize = 0.5, color=exu.graphics.color.blue)
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    if physicsMass is not None: #the old name of the argument (#2814)
        DeprecatedArgument('physicsMass', '1.12.258', 2031, use='mass', function='MainSystem.CreateMassPoint')
        mass = physicsMass
    #error checks:        
    if not exudyn.__useExudynFast:
        where='MainSystem.CreateMassPoint(...)'
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)
        if not IsVector(referencePosition, 3):
            RaiseTypeError(where=where, argumentName='referencePosition', received = referencePosition, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(initialDisplacement, 3):
            RaiseTypeError(where=where, argumentName='initialDisplacement', received = initialDisplacement, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(initialVelocity, 3):
            RaiseTypeError(where=where, argumentName='initialVelocity', received = initialVelocity, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(gravity, 3):
            RaiseTypeError(where=where, argumentName='gravity', received = gravity, expectedType = ExpectedType.Vector, dim=3)
    
        if not IsValidRealInt(drawSize):
            RaiseTypeError(where=where, argumentName='drawSize', received = drawSize, expectedType = ExpectedType.Real)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsValidBool(create2D):
            RaiseTypeError(where=where, argumentName='create2D', received = create2D, expectedType = ExpectedType.Bool)
        if not IsValidBool(returnDict):
            RaiseTypeError(where=where, argumentName='returnDict', received = returnDict, expectedType = ExpectedType.Bool)
    
        if type(graphicsDataList) != list:
            raise ValueError(where+': graphicsDataList must be a (possibly empty) list of dictionaries of graphics data!')

    nodeName = ''
    if name != '':
        nodeName = 'Node:'+name

    if len(graphicsDataList) != 0: 
        drawSize = 0 #this makes the node to be shown (number, basis), but not drawn

    if not create2D:
        nodeNumber = mbs.AddNode(eii.NodePoint(name = nodeName,
                         referenceCoordinates = referencePosition,
                         initialCoordinates=initialDisplacement,
                         initialVelocities=initialVelocity,
                         visualization = eii.VNodePoint(show = show, drawSize = drawSize, color = color),
                         ))
        bodyNumber = mbs.AddObject(eii.MassPoint(name = name,
                                                mass=mass,
                                                nodeNumber = nodeNumber,
                                                visualization = eii.VMassPoint(show = graphicsDataList != [], 
                                                                           graphicsData = graphicsDataList) ))
    else:
        nodeNumber = mbs.AddNode(eii.NodePoint2D(name = nodeName,
                         referenceCoordinates = referencePosition[0:2],
                         initialCoordinates=initialDisplacement[0:2],
                         initialVelocities=initialVelocity[0:2],
                         visualization = eii.VNodePoint2D(show = show, drawSize = drawSize, color = color),
                         ))
        bodyNumber = mbs.AddObject(eii.MassPoint2D(name = name, 
                                                mass=mass,
                                                nodeNumber = nodeNumber,
                                                visualization = eii.VMassPoint(show = graphicsDataList != [], 
                                                                           graphicsData = graphicsDataList) ))
        
    if returnDict:
        rDict = {'nodeNumber':nodeNumber, 'bodyNumber': bodyNumber}
    
    if list(gravity) != [0.,0.,0.]: #        if np.linalg.norm(gravity) != 0.:
        markerNumber = mbs.AddMarker(eii.MarkerBodyMass(bodyNumber=bodyNumber))
        loadNumber = mbs.AddLoad(eii.LoadMassProportional(markerNumber=markerNumber, loadVector=gravity))
        if returnDict:
            rDict['markerBodyMass'] = markerNumber
            rDict['loadNumber'] = loadNumber

    if returnDict:
        return rDict
    else:
        return bodyNumber


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateRigidBody(mbs,
                           name = '',
                           referencePosition = None,
                           referenceRotationMatrix = None,
                           initialVelocity = [0.,0.,0.],
                           initialAngularVelocity = [0.,0.,0.],
                           initialDisplacement = None,
                           initialRotationMatrix = None,
                           inertia=None,
                           gravity = [0.,0.,0.],
                           nodeType=exudyn.NodeType.RotationEulerParameters,
                           graphicsDataList = [],
                           graphicsDataUserFunction = 0,
                           drawSize = -1,
                           color =  [-1.,-1.,-1.,-1.],
                           show = True, 
                           create2D = False, 
                           returnDict = False,
                           referenceHT = None,
                           initialHT = None) -> Union[dict, exudyn.ObjectIndex]: 
    """helper function to create 3D (or 2D) rigid body object and node; all quantities are global (angular velocity, etc.); use this function to easily create a rigid body; graphics can be directly obtained from inertia object, e.g. in case of cylindrical or cuboid shape

    Args:
        mbs: the MainSystem where items are created
        name: name string for object, node is 'Node:'+name
        referencePosition: reference position vector for rigid body node (always a 3D vector, no matter if 2D or 3D body); None: zero
        referenceRotationMatrix: reference rotation matrix for rigid body node (always 3D matrix, no matter if 2D or 3D body); None: the unit matrix
        initialVelocity: initial translational velocity vector for node (always a 3D vector, no matter if 2D or 3D body)
        initialAngularVelocity: initial angular velocity vector for node (always a 3D vector, no matter if 2D or 3D body)
        initialDisplacement: initial translational displacement vector for node (always a 3D vector, no matter if 2D or 3D body); these displacements are deviations from reference position, e.g. for a finite element node [None: unused]
        initialRotationMatrix: initial rotation provided as matrix (always a 3D matrix, no matter if 2D or 3D body); this rotation is superimposed to reference rotation [None: unused]
        inertia: an instance of class RigidBodyInertia, see rigidBodyUtilities; may also be from derived class (InertiaCuboid, InertiaMassPoint, InertiaCylinder, ...)
        gravity: gravity vevtor applied (always a 3D vector, no matter if 2D or 3D mass)
        nodeType: optional exudyn.NodeType to define the rotation parameterization: RotationEulerParameters, RotationRotationVector or RotationRxyz
        graphicsDataList: list of GraphicsData for rigid body visualization; use exudyn.graphics functions to create GraphicsData for basic solids
        graphicsDataUserFunction: a user function graphicsDataUserFunction(mbs, itemNumber)->BodyGraphicsData (list of GraphicsData), which can be used to draw user-defined graphics; this is much slower than regular GraphicsData
        drawSize: general drawing size of node
        color: color of node
        show: True: if graphicsData list is empty, node is shown, otherwise body is shown; False: nothing is shown
        create2D: if True, create NodeRigidBody2D and ObjectRigidBody2D
        returnDict: if False, returns object index; if True, returns dict of all information on created object and node
        referenceHT: referenceRotationMatrix and referencePosition at once, as homogeneous transformation: an exu.HT, a 4x4 matrix or its 16 values; None: not given; it raises if given together with referencePosition or referenceRotationMatrix
        initialHT: initialRotationMatrix and initialDisplacement at once, the transformation added to the reference (the rotation superimposed to the reference rotation, the displacement added to the reference position); an exu.HT, a 4x4 matrix or its 16 values; None: not given; it raises if given together with initialDisplacement or initialRotationMatrix

    Returns:
        :Union[dict, ObjectIndex]: returns rigid body object index (or dict with 'nodeNumber', 'objectNumber' and possibly 'loadNumber' and 'markerBodyMass' if returnDict=True)

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                         sideLengths=[1,0.1,0.1]),
                                 referencePosition = [1,0,0],
                                 initialVelocity = [2,5,0],
                                 initialAngularVelocity = [5,0.5,0.7],
                                 gravity = [0,-9.81,0],
                                 graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                              color=exu.graphics.color.red)])
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    where='MainSystem.CreateRigidBody(...)'
    referencePosition, referenceRotationMatrix = _FrameFromHT(where, 'referenceHT', referenceHT,
        'referencePosition', referencePosition, 'referenceRotationMatrix', referenceRotationMatrix)
    initialDisplacement, initialRotationMatrix = _FrameFromHT(where, 'initialHT', initialHT,
        'initialDisplacement', initialDisplacement, 'initialRotationMatrix', initialRotationMatrix)
    if referencePosition is None:
        referencePosition = [0.,0.,0.]
    if referenceRotationMatrix is None:
        referenceRotationMatrix = np.eye(3)

    #error checks:
    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)
        if not IsVector(referencePosition, 3):
            RaiseTypeError(where=where, argumentName='referencePosition', received = referencePosition, expectedType = ExpectedType.Vector, dim=3)
        if not IsSquareMatrix(referenceRotationMatrix, 3):
            RaiseTypeError(where=where, argumentName='referenceRotationMatrix', received = referenceRotationMatrix, expectedType = ExpectedType.Matrix, dim=3)


        if not IsVector(initialVelocity, 3):
            RaiseTypeError(where=where, argumentName='initialVelocity', received = initialVelocity, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(initialAngularVelocity, 3):
            RaiseTypeError(where=where, argumentName='initialAngularVelocity', received = initialAngularVelocity, expectedType = ExpectedType.Vector, dim=3)
        if IsNotNone(initialDisplacement) and not IsVector(initialDisplacement, 3):
            RaiseTypeError(where=where, argumentName='initialDisplacement', received = initialDisplacement, expectedType = ExpectedType.Vector, dim=3)
        if IsNotNone(initialRotationMatrix) and not IsSquareMatrix(initialRotationMatrix, 3):
            RaiseTypeError(where=where, argumentName='initialRotationMatrix', received = initialRotationMatrix, expectedType = ExpectedType.Matrix, dim=3)

        if not IsVector(gravity, 3):
            RaiseTypeError(where=where, argumentName='gravity', received = gravity, expectedType = ExpectedType.Vector, dim=3)
    
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsValidRealInt(drawSize):
            RaiseTypeError(where=where, argumentName='drawSize', received = drawSize, expectedType = ExpectedType.Real)
        if not IsValidBool(create2D):
            RaiseTypeError(where=where, argumentName='create2D', received = create2D, expectedType = ExpectedType.Bool)
        if not IsValidBool(returnDict):
            RaiseTypeError(where=where, argumentName='returnDict', received = returnDict, expectedType = ExpectedType.Bool)
    
        if type(graphicsDataList) != list:
            raise ValueError(where+': graphicsDataList must be a (possibly empty) list of dictionaries of graphics data!')

        # if create2D:
        #     raise ValueError('MainSystem.CreateRigidBody(...): create2D=True currently not supported')

    nodeName = ''
    if name != '':
        nodeName = 'Node:'+name

    #try to get graphics from inertia, if no graphics provided
    graphicsDataList0 = graphicsDataList
    if len(graphicsDataList) == 0 and inertia is not None:
        graphicsDataList0 = [inertia.GetGraphics(color)]
        if graphicsDataList0 is None: 
            graphicsDataList0=[]
            
    if len(graphicsDataList0) != 0: 
        drawSize = 0 #this makes the node to be shown (number, basis), but not drawn

    #++++++++++++++++        
    if not create2D:
        RotationMatrix2parameters = None
        AngularVelocity2parameters_t = None
        NodeClass = None
        VNodeClass = None
        if nodeType == exudyn.NodeType.RotationEulerParameters:
            RotationMatrix2parameters = RotationMatrix2EulerParameters
            AngularVelocity2parameters_t = AngularVelocity2EulerParameters_t
            NodeClass = eii.NodeRigidBodyEP
            VNodeClass = eii.VNodeRigidBodyEP
        elif nodeType == exudyn.NodeType.RotationRxyz:
            RotationMatrix2parameters = RotationMatrix2RotXYZ
            AngularVelocity2parameters_t = AngularVelocity2RotXYZ_t
            NodeClass = eii.NodeRigidBodyRxyz
            VNodeClass = eii.VNodeRigidBodyRxyz
        elif nodeType == exudyn.NodeType.RotationRotationVector:
            def AngularVelocity2RotationVector_t(angularVelocity, rotMatrix):
                return np.dot(rotMatrix.transpose(),angularVelocity)
                
            RotationMatrix2parameters = RotationMatrix2RotationVector
            AngularVelocity2parameters_t = AngularVelocity2RotationVector_t
            NodeClass = eii.NodeRigidBodyRotVecLG
            VNodeClass = eii.VNodeRigidBodyRotVecLG
        else:
            raise ValueError('MainSystem.CreateRigidBody(...): invalid nodeType!')
        #++++++++++++++++        
        referenceRot = RotationMatrix2parameters(referenceRotationMatrix)
        if nodeType != exudyn.NodeType.RotationRotationVector:
            rot0_t = AngularVelocity2parameters_t(initialAngularVelocity, referenceRot)
        else:
            rot0_t = AngularVelocity2parameters_t(initialAngularVelocity, referenceRotationMatrix)
    
        initCoordinates = [0] * (3+len(referenceRot))
        if IsNotNone(initialDisplacement) or IsNotNone(initialRotationMatrix):
            if IsNone(initialDisplacement):
                initialDisplacement = [0.,0.,0.]
            if IsNone(initialRotationMatrix):
                initialRotationMatrix = np.eye(3)
            
            rotInit = RotationMatrix2parameters(referenceRotationMatrix @ initialRotationMatrix) - referenceRot #relative to reference!
            initCoordinates  = list(initialDisplacement)+list(rotInit)
            
    
        nodeItem = NodeClass(name = nodeName,
                             referenceCoordinates=list(referencePosition) + list(referenceRot), 
                             initialVelocities=list(initialVelocity)+list(rot0_t),
                             initialCoordinates=initCoordinates,
                             visualization = VNodeClass(show = show, drawSize = drawSize, color = color)
                             )
        nodeNumber = mbs.AddNode(nodeItem)
        bodyNumber = mbs.AddObject(eii.ObjectRigidBody(name=name, mass=inertia.mass, inertia=inertia.GetInertia6D(), 
                                                       centerOfMass=inertia.com,
                                                       nodeNumber=nodeNumber, 
                                                       visualization=eii.VObjectRigidBody(show = show, 
                                                                                          graphicsDataUserFunction = graphicsDataUserFunction,
                                                                                          graphicsData=graphicsDataList0)))
    else: #2D
        A = np.array(referenceRotationMatrix)
        if not exudyn.__useExudynFast:
            if abs(referencePosition[2]) > 1e-14:
                raise ValueError('MainSystem.CreateRigidBody(...): in case of 2D rigid body, referencePosition may not have a Z-component')
            if (abs(A[2,0]) + abs(A[2,1]) + abs(A[0,2]) + abs(A[1,2]) + abs(A[2,2]-1)) > 1e-13:
                raise ValueError('MainSystem.CreateRigidBody(...): in case of 2D rigid body, referenceRotationMatrix must only have a rotation around Z-axis')
            if (abs(initialVelocity[2])) > 1e-14:
                raise ValueError('MainSystem.CreateRigidBody(...): in case of 2D rigid body, initialVelocity must not have a Z-component')
            if (abs(initialAngularVelocity[0]) + abs(initialAngularVelocity[1])) > 1e-14:
                raise ValueError('MainSystem.CreateRigidBody(...): in case of 2D rigid body, initialAngularVelocity must only have a Z-component')
            if cMath.norm(inertia.com) > 1e-14:
                raise ValueError('MainSystem.CreateRigidBody(...): in case of 2D rigid body, the center of mass in inertia must be [0,0,0] (will be fixed in future)')


        referenceRot = cMath.atan2(A[1,0],A[0,0])
    
        initCoordinates = [0.,0.,0.]
        if IsNotNone(initialDisplacement) or IsNotNone(initialRotationMatrix):
            if IsNotNone(initialDisplacement):
                if abs(initialDisplacement[2]) > 1e-14:
                    raise ValueError('MainSystem.CreateRigidBody(...): in case of 2D rigid body, initialDisplacement may not have a Z-component')
                initCoordinates[0] = initialDisplacement[0]
                initCoordinates[1] = initialDisplacement[1]
            if IsNotNone(initialRotationMatrix):
                A0 = np.array(initialRotationMatrix)
                if (abs(A0[2,0]) + abs(A0[2,1]) + abs(A0[0,2]) + abs(A0[1,2]) + abs(A0[2,2]-1)) > 1e-13:
                    raise ValueError('MainSystem.CreateRigidBody(...): in case of 2D rigid body, initialRotationMatrix must only have a rotation around Z-axis')
                phi0 = cMath.atan2(A0[1,0],A0[0,0]) - referenceRot
                initCoordinates[2] = phi0
            
        nodeItem = eii.NodeRigidBody2D(name = nodeName,
                             referenceCoordinates=[referencePosition[0],referencePosition[1],referenceRot], 
                             initialCoordinates=initCoordinates,
                             initialVelocities=[initialVelocity[0],initialVelocity[1],initialAngularVelocity[2]],
                             visualization = eii.VNodeRigidBody2D(show = show, drawSize = drawSize, color = color)
                             )
        nodeNumber = mbs.AddNode(nodeItem)
        bodyNumber = mbs.AddObject(eii.ObjectRigidBody2D(name=name, mass=inertia.mass, inertia=inertia.GetInertia6D()[2],
                                                       #centerOfMass=inertia.com,
                                                       nodeNumber=nodeNumber,
                                                       visualization=eii.VObjectRigidBody(show = show,
                                                                                          graphicsDataUserFunction=graphicsDataUserFunction,
                                                                                          graphicsData=graphicsDataList0)))
        
    if returnDict:
        rDict = {'nodeNumber':nodeNumber, 'bodyNumber': bodyNumber}

    if cMath.norm(gravity) != 0.:
        markerNumber = mbs.AddMarker(eii.MarkerBodyMass(bodyNumber=bodyNumber))
        loadNumber = mbs.AddLoad(eii.LoadMassProportional(markerNumber=markerNumber, loadVector=gravity))

        if returnDict:
            rDict['markerBodyMass'] = markerNumber
            rDict['loadNumber'] = loadNumber

    if returnDict:
        return rDict
    else:
        return bodyNumber


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateSpringDamper(mbs,
                                 name='',
                                 itemNumbers=[None, None], 
                                 localPosition0 = [0.,0.,0.],
                                 localPosition1 = [0.,0.,0.], 
                                 referenceLength = None, 
                                 stiffness = 0., damping = 0., force = 0.,
                                 velocityOffset = 0., 
                                 springForceUserFunction = 0,
                                 show=True, drawSize=-1, color=exudyn.graphics.color.default, bodyNumbers=None, bodyOrNodeList=None) -> exudyn.ObjectIndex:
    """helper function to create SpringDamper connector, using arguments from ObjectConnectorSpringDamper; similar interface as CreateDistanceConstraint(...), see there for for further information

    Args:
        mbs: the MainSystem where items are created
        name: name string for connector; markers get Marker0:name and Marker1:name
        itemNumbers: a list of two items to be connected, each a body (ObjectIndex) at localPosition0/1, a node (NodeIndex) or a marker (MarkerIndex); for a node or a marker, localPosition0/1 is [0,0,0]
        localPosition0: local position (as 3D list or numpy array) on body0, if not a node of marker number
        localPosition1: local position (as 3D list or numpy array) on body1, if not a node of marker number
        referenceLength: if None, length is computed from reference position of bodies or nodes; if not None, this scalar reference length is used for spring
        stiffness: scalar stiffness coefficient
        damping: scalar damping coefficient
        force: scalar additional force applied
        velocityOffset: scalar offset: if referenceLength is changed over time, the velocityOffset may be changed accordingly to emulate a reference motion
        springForceUserFunction: a user function springForceUserFunction(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force)->float ; this function replaces the internal connector force computation
        show: if True, connector visualization is drawn
        drawSize: general drawing size of connector
        color: color of connector
        bodyNumbers: deprecated name of itemNumbers
        bodyOrNodeList: not accepted, a TypeError: the items are given in itemNumbers

    Returns:
        :ObjectIndex: returns index of newly created object

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        b0 = mbs.CreateMassPoint(referencePosition = [2,0,0],
                                 initialVelocity = [2,5,0],
                                 mass = 1, gravity = [0,-9.81,0],
                                 drawSize = 0.5, color=exu.graphics.color.blue)
        oGround = mbs.AddObject(ObjectGround())
        #add vertical spring
        oSD = mbs.CreateSpringDamper(itemNumbers=[oGround, b0],
                                     localPosition0=[2,1,0],
                                     localPosition1=[0,0,0],
                                     stiffness=1e4, damping=1e2,
                                     drawSize=0.2)
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        SC.visualizationSettings.nodes.drawNodesAsPoint=False
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    #perform some checks:
    where='MainSystem.CreateSpringDamper(...)'
    if bodyNumbers is not None: #the old name of the argument (#2863)
        DeprecatedArgument('bodyNumbers', '1.12.337', 2031, use='itemNumbers', function='MainSystem.CreateSpringDamper')
    itemNumbers = ItemNumbersArgument(where, itemNumbers, bodyNumbers, bodyOrNodeList)
    CheckItemNumbers(where, itemNumbers, [_bodyNodeMarker, _bodyNodeMarker], [localPosition0, localPosition1])
    
    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)
                
        if not IsVector(localPosition0, 3):
            RaiseTypeError(where=where, argumentName='localPosition0', received = localPosition0, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(localPosition1, 3):
            RaiseTypeError(where=where, argumentName='localPosition1', received = localPosition1, expectedType = ExpectedType.Vector, dim=3)
    
        if IsNotNone(referenceLength) and not IsValidURealInt(referenceLength):
            RaiseTypeError(where=where, argumentName='referenceLength', received = referenceLength, expectedType = ExpectedType.PReal)
        if not IsValidRealInt(stiffness):
            RaiseTypeError(where=where, argumentName='stiffness', received = stiffness, expectedType = ExpectedType.Real)
        if not IsValidRealInt(damping):
            RaiseTypeError(where=where, argumentName='damping', received = damping, expectedType = ExpectedType.Real)
        if not IsValidRealInt(force):
            RaiseTypeError(where=where, argumentName='force', received = force, expectedType = ExpectedType.Real)
        if not IsValidRealInt(velocityOffset):
            RaiseTypeError(where=where, argumentName='velocityOffset', received = velocityOffset, expectedType = ExpectedType.Real)
    
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsValidRealInt(drawSize):
            RaiseTypeError(where=where, argumentName='drawSize', received = drawSize, expectedType = ExpectedType.Real)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)


    [mBody0, mBody1, p0, p1, A0, A1] = GetMarkersPosRot(mbs, name, itemNumbers, localPosition0, localPosition1, 
                                                        getPosition=True, getRotationMatrix=False, useRigidMarker=False)
        
    if IsNone(referenceLength): #automatically compute reference length
        referenceLength = cMath.norm(np.array(p1)-p0)
    
    oConnector = mbs.AddObject(eii.ObjectConnectorSpringDamper(name=name,markerNumbers = [mBody0,mBody1],
                                                                      referenceLength = referenceLength,
                                                                      stiffness = stiffness,
                                                                      damping = damping,
                                                                      force = force, 
                                                                      velocityOffset = velocityOffset,
                                                                      springForceUserFunction=springForceUserFunction,
                                                                      visualization=eii.VSpringDamper(show=show, drawSize=drawSize,
                                                                                                      color=color)
                                                                      ))

    return oConnector


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateCartesianSpringDamper(mbs,
                                 name='',
                                 itemNumbers=[None, None], 
                                 localPosition0 = [0.,0.,0.],
                                 localPosition1 = [0.,0.,0.], 
                                 stiffness = [0.,0.,0.], damping = [0.,0.,0.], 
                                 offset = [0.,0.,0.],
                                 springForceUserFunction = 0,
                                 show=True, drawSize=-1, color=exudyn.graphics.color.default, bodyNumbers=None, bodyOrNodeList=None) -> exudyn.ObjectIndex:
    """helper function to create CartesianSpringDamper connector, using arguments from ObjectConnectorCartesianSpringDamper

    Args:
        mbs: the MainSystem where items are created
        name: name string for connector; markers get Marker0:name and Marker1:name
        itemNumbers: a list of two items to be connected, each a body (ObjectIndex) at localPosition0/1, a node (NodeIndex) or a marker (MarkerIndex); for a node or a marker, localPosition0/1 is [0,0,0]
        localPosition0: local position (as 3D list or numpy array) on body0, if not a node of marker number
        localPosition1: local position (as 3D list or numpy array) on body1, if not a node of marker number
        stiffness: stiffness coefficients (as 3D list or numpy array)
        damping: damping coefficients (as 3D list or numpy array)
        offset: offset vector (as 3D list or numpy array)
        springForceUserFunction: a user function springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset)->[float,float,float] ; this function replaces the internal connector force computation
        show: if True, connector visualization is drawn
        drawSize: general drawing size of connector
        color: color of connector
        bodyNumbers: deprecated name of itemNumbers
        bodyOrNodeList: not accepted, a TypeError: the items are given in itemNumbers

    Returns:
        :ObjectIndex: returns index of newly created object

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        b0 = mbs.CreateMassPoint(referencePosition = [7,0,0],
                                  mass = 1, gravity = [0,-9.81,0],
                                  drawSize = 0.5, color=exu.graphics.color.blue)
        oGround = mbs.AddObject(ObjectGround())
        oSD = mbs.CreateCartesianSpringDamper(itemNumbers=[oGround, b0],
                                      localPosition0=[7.5,1,0],
                                      localPosition1=[0,0,0],
                                      stiffness=[200,2000,0], damping=[2,20,0],
                                      drawSize=0.2)
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        SC.visualizationSettings.nodes.drawNodesAsPoint=False
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    where='MainSystem.CreateCartesianSpringDamper(...)'
    if bodyNumbers is not None: #the old name of the argument (#2863)
        DeprecatedArgument('bodyNumbers', '1.12.337', 2031, use='itemNumbers', function='MainSystem.CreateCartesianSpringDamper')
    itemNumbers = ItemNumbersArgument(where, itemNumbers, bodyNumbers, bodyOrNodeList)
    CheckItemNumbers(where, itemNumbers, [_bodyNodeMarker, _bodyNodeMarker], [localPosition0, localPosition1])

    #perform some checks:
    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)
    
        if not IsVector(localPosition0, 3):
            RaiseTypeError(where=where, argumentName='localPosition0', received = localPosition0, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(localPosition1, 3):
            RaiseTypeError(where=where, argumentName='localPosition1', received = localPosition1, expectedType = ExpectedType.Vector, dim=3)
    
        if not IsVector(stiffness, 3):
            RaiseTypeError(where=where, argumentName='stiffness', received = stiffness, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(damping, 3):
            RaiseTypeError(where=where, argumentName='damping', received = damping, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(offset, 3):
            RaiseTypeError(where=where, argumentName='offset', received = offset, expectedType = ExpectedType.Vector, dim=3)

        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsValidRealInt(drawSize):
            RaiseTypeError(where=where, argumentName='drawSize', received = drawSize, expectedType = ExpectedType.Real)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)

    [mBody0, mBody1, p0, p1, A0, A1] = GetMarkersPosRot(mbs, name, itemNumbers, localPosition0, localPosition1, 
                                                        getPosition=False, getRotationMatrix=False, useRigidMarker=False)
                
    oConnector = mbs.AddObject(eii.ObjectConnectorCartesianSpringDamper(name=name,markerNumbers = [mBody0,mBody1],
                                                                        stiffness = stiffness, damping = damping, offset = offset,
                                                                        springForceUserFunction=springForceUserFunction,
                                                                        visualization=eii.VCartesianSpringDamper(show=show, 
                                                                                      drawSize=drawSize, color=color)
                                                                      ))

    return oConnector

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateRigidBodySpringDamper(mbs,
                                 name='',
                                 itemNumbers=[None, None], 
                                 localPosition0 = [0.,0.,0.],
                                 localPosition1 = [0.,0.,0.], 
                                 stiffness = np.zeros((6,6)), 
                                 damping = np.zeros((6,6)), 
                                 offset = [0.,0.,0.,0.,0.,0.],
                                 rotationMatrixJoint=np.eye(3),
                                 useGlobalFrame=True,
                                 useIntrinsicFormulation=True,
                                 springForceTorqueUserFunction=0,
                                 postNewtonStepUserFunction=0,
                                 show=True, drawSize=-1, color=exudyn.graphics.color.default, intrinsicFormulation=None, bodyNumbers=None, bodyOrNodeList=None) -> exudyn.ObjectIndex:
    """helper function to create RigidBodySpringDamper connector, using arguments from ObjectConnectorRigidBodySpringDamper, see there for the full documentation

    Args:
        mbs: the MainSystem where items are created
        name: name string for connector; markers get Marker0:name and Marker1:name
        itemNumbers: a list of two items to be connected, each a body (ObjectIndex) at localPosition0/1, a rigid node (NodeIndex) or a rigid marker (MarkerIndex); for a node or a marker, localPosition0/1 is [0,0,0]
        localPosition0: local position (as 3D list or numpy array) on body0, if not a node of marker number
        localPosition1: local position (as 3D list or numpy array) on body1, if not a node of marker number
        stiffness: stiffness coefficients (as 6D matrix or numpy array)
        damping: damping coefficients (as 6D matrix or numpy array)
        offset: offset vector (as 6D list or numpy array)
        rotationMatrixJoint: additional rotation matrix; in case  useGlobalFrame=False, it transforms body0/node0 local frame to joint frame; if useGlobalFrame=True, it transforms global frame to joint frame
        useGlobalFrame: if False, the rotationMatrixJoint is defined in the local coordinate system of body0
        useIntrinsicFormulation: if True, uses intrinsic formulation of Maserati and Morandini, which uses matrix logarithm and is independent of order of markers (preferred formulation); otherwise, Tait-Bryan angles are used for computation of torque, see documentation
        springForceTorqueUserFunction: a user function springForceTorqueUserFunction(mbs, t, itemNumber, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1, offset)->[float,float,float, float,float,float] ; this function replaces the internal connector force / torque computation
        postNewtonStepUserFunction: a special user function postNewtonStepUserFunction(mbs, t, Index itemIndex, dataCoordinates, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1, offset)->[PNerror, recommendedStepSize, data[0], data[1], ...] ; for details, see RigidBodySpringDamper for full docu
        show: if True, connector visualization is drawn
        drawSize: general drawing size of connector
        color: color of connector
        intrinsicFormulation: deprecated name of useIntrinsicFormulation
        bodyNumbers: deprecated name of itemNumbers
        bodyOrNodeList: not accepted, a TypeError: the items are given in itemNumbers

    Returns:
        :ObjectIndex: returns index of newly created object

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                         sideLengths=[1,0.1,0.1]),
                                 referencePosition = [8,0,0],
                                 initialAngularVelocity = [0,1,2],
                                 gravity = [0,-9.81,0],
                                 graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                          color=exu.graphics.color.green)])
        oGround = mbs.AddObject(ObjectGround())
        #a bushing at the left end of the bar
        mbs.CreateRigidBodySpringDamper(itemNumbers=[oGround, b0],
                                        localPosition0=[7.5,0,0],
                                        localPosition1=[-0.5,0,0],
                                        stiffness=np.diag([1e4,1e4,1e4,1e3,1e3,1e3]),
                                        damping=np.diag([10,10,10,1,1,1]))
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    if intrinsicFormulation is not None: #the old name of the argument (#2814)
        DeprecatedArgument('intrinsicFormulation', '1.12.258', 2031, use='useIntrinsicFormulation', function='MainSystem.CreateRigidBodySpringDamper')
        useIntrinsicFormulation = intrinsicFormulation
    where='MainSystem.CreateRigidBodySpringDamper(...)'
    if bodyNumbers is not None: #the old name of the argument (#2863)
        DeprecatedArgument('bodyNumbers', '1.12.337', 2031, use='itemNumbers', function='MainSystem.CreateRigidBodySpringDamper')
    itemNumbers = ItemNumbersArgument(where, itemNumbers, bodyNumbers, bodyOrNodeList)
    CheckItemNumbers(where, itemNumbers, [_bodyNodeMarker, _bodyNodeMarker], [localPosition0, localPosition1])

    #perform some checks:
    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)
                
        if not IsVector(localPosition0, 3):
            RaiseTypeError(where=where, argumentName='localPosition0', received = localPosition0, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(localPosition1, 3):
            RaiseTypeError(where=where, argumentName='localPosition1', received = localPosition1, expectedType = ExpectedType.Vector, dim=3)
    
        if not IsSquareMatrix(stiffness, 6):
            RaiseTypeError(where=where, argumentName='stiffness', received = stiffness, expectedType = ExpectedType.Matrix, dim=6)
        if not IsSquareMatrix(damping, 6):
            RaiseTypeError(where=where, argumentName='damping', received = damping, expectedType = ExpectedType.Matrix, dim=6)
        if not IsVector(offset, 6):
            RaiseTypeError(where=where, argumentName='offset', received = offset, expectedType = ExpectedType.Vector, dim=3)


        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsValidBool(useGlobalFrame):
            RaiseTypeError(where=where, argumentName='useGlobalFrame', received = useGlobalFrame, expectedType = ExpectedType.Bool)

        if not IsValidRealInt(drawSize):
            RaiseTypeError(where=where, argumentName='drawSize', received = drawSize, expectedType = ExpectedType.Real)

        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)

        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)


    [mBody0, mBody1, p0, p1, A0, A1] = GetMarkersPosRot(mbs, name, itemNumbers, localPosition0, localPosition1, 
                                                        getPosition=False, getRotationMatrix=True, useRigidMarker=True)

    if useGlobalFrame:
        #compute joint marker orientations, rotationMatrixAxes represents global frame:
        MR0 = _MatMul3x3(A0.T, rotationMatrixJoint)
        MR1 = _MatMul3x3(A1.T, rotationMatrixJoint)
    else: #transform into global coordinates, then everything works same
        #compute joint marker orientations, rotationMatrixAxes represents local frame:
        MR0 = rotationMatrixJoint
        MR1 = _MatMul3x3(_MatMul3x3(A1.T, A0), rotationMatrixJoint)

    #the rotations are the markers' (#2745)
    MR0 = _RotationIntoMarker(mbs, mBody0, itemNumbers[0], localPosition0, MR0)
    MR1 = _RotationIntoMarker(mbs, mBody1, itemNumbers[1], localPosition1, MR1)
    (mBody0, MR0) = _MarkerWithRotation(mbs, mBody0, MR0) #a marker the caller gave: a copy turned by the rotation (#2804)
    (mBody1, MR1) = _MarkerWithRotation(mbs, mBody1, MR1)

    oConnector = mbs.AddObject(eii.ObjectConnectorRigidBodySpringDamper(name=name,markerNumbers = [mBody0,mBody1],
                                                                        stiffness = stiffness, damping = damping, 
                                                                        offset = offset,
                                                                        **_RotationMarkerArgs(MR0, MR1), #only a rotation a marker cannot take (#2820)
                                                                        useIntrinsicFormulation=useIntrinsicFormulation,
                                                                        springForceTorqueUserFunction=springForceTorqueUserFunction, 
                                                                        postNewtonStepUserFunction=postNewtonStepUserFunction,
                                                                        visualization=eii.VRigidBodySpringDamper(show=show, 
                                                                                      drawSize=drawSize, color=color)
                                                                      ))

    return oConnector




#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateTorsionalSpringDamper(mbs,
                                          name='',
                                          itemNumbers=[None, None], 
                                          position = [0.,0.,0.],
                                          axis = [0.,0.,0.],
                                          stiffness = 0., 
                                          damping = 0., 
                                          offset = 0.,
                                          velocityOffset = 0.,
                                          torque = 0.,
                                          useGlobalFrame=True,
                                          springTorqueUserFunction=0,
                                          unlimitedRotations = True,
                                          show=True, drawSize=-1, color=exudyn.graphics.color.default, bodyNumbers=None, bodyOrNodeList=None) -> exudyn.ObjectIndex:
    """helper function to create TorsionalSpringDamper connector, using arguments from ObjectConnectorTorsionalSpringDamper, see there for the full documentation

    Args:
        mbs: the MainSystem where items are created
        name: name string for connector; markers get Marker0:name and Marker1:name
        itemNumbers: a list of two items to be connected, each a rigid body or a ground object (ObjectIndex) or a rigid marker (MarkerIndex); with a marker, position is [], and the marker sets the position of the spring-damper
        position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
        axis: a 3D vector as list or np.array containing the axis around which the spring acts, either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True)
        stiffness: scalar stiffness of spring
        damping: scalar damping added to spring
        offset: scalar offset, which can be used to realize a P-controlled actuator
        velocityOffset: scalar velocity offset, which can be used to realize a D-controlled actuator
        torque: additional constant torque added to spring-damper, acting between the two bodies
        useGlobalFrame: if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
        springTorqueUserFunction : a user function springTorqueUserFunction(mbs, t, itemNumber, rotation, angularVelocity, stiffness, damping, offset)->float ; this function replaces the internal connector torque computation
        unlimitedRotations: if True, an additional generic data node is added to enable measurement of rotations beyond +/- pi; this also allows the spring to cope with multiple turns.
        show: if True, connector visualization is drawn
        drawSize: general drawing size of connector
        color: color of connector
        bodyNumbers: deprecated name of itemNumbers
        bodyOrNodeList: not accepted, a TypeError: the items are given in itemNumbers

    Returns:
        :ObjectIndex: returns index of newly created object

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                         sideLengths=[1,0.1,0.1]),
                                 referencePosition = [9.5,0,0],
                                 initialAngularVelocity = [0,0,2],
                                 graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                          color=exu.graphics.color.steelblue)])
        oGround = mbs.AddObject(ObjectGround())
        mbs.CreateRevoluteJoint(itemNumbers=[oGround, b0], position=[9,0,0], axis=[0,0,1],
                                useGlobalFrame=True, axisRadius=0.02, axisLength=0.14)
        #a torsional spring-damper about the axis of the revolute joint
        mbs.CreateTorsionalSpringDamper(itemNumbers=[oGround, b0], position=[9,0,0], axis=[0,0,1],
                                        stiffness=100, damping=1)
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    where='MainSystem.CreateTorsionalSpringDamper(...)'
    if bodyNumbers is not None: #the old name of the argument (#2863)
        DeprecatedArgument('bodyNumbers', '1.12.337', 2031, use='itemNumbers', function='MainSystem.CreateTorsionalSpringDamper')
    itemNumbers = ItemNumbersArgument(where, itemNumbers, bodyNumbers, bodyOrNodeList)

    #perform some checks:
    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)
                
        if not IsVector(axis, 3):
            RaiseTypeError(where=where, argumentName='axis', received = axis, expectedType = ExpectedType.Vector, dim=3)
    
        if not IsValidURealInt(stiffness):
            RaiseTypeError(where=where, argumentName='stiffness', received = stiffness, expectedType = ExpectedType.UReal)
        if not IsValidURealInt(damping):
            RaiseTypeError(where=where, argumentName='damping', received = damping, expectedType = ExpectedType.UReal)
        if not IsValidRealInt(offset):
            RaiseTypeError(where=where, argumentName='offset', received = offset, expectedType = ExpectedType.Real)
        if not IsValidRealInt(velocityOffset):
            RaiseTypeError(where=where, argumentName='velocityOffset', received = velocityOffset, expectedType = ExpectedType.Real)
        if not IsValidRealInt(torque):
            RaiseTypeError(where=where, argumentName='torque', received = torque, expectedType = ExpectedType.Real)


        if not IsValidBool(unlimitedRotations):
            RaiseTypeError(where=where, argumentName='unlimitedRotations', received = unlimitedRotations, expectedType = ExpectedType.Bool)

        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)


        if not IsValidRealInt(drawSize):
            RaiseTypeError(where=where, argumentName='drawSize', received = drawSize, expectedType = ExpectedType.Real)

        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)

        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)


    #similar to RevoluteJoint!
    [p0, A0, p1, A1, mBody0, mBody1, pJoint] = JointPreCheckCalcBodyMarkers(where, mbs, name, itemNumbers, position, show, useGlobalFrame)
        
    if useGlobalFrame:
        vAxis = copy.copy(axis)
    else: #transform into global coordinates, then everything works same
        vAxis = A0 @ axis

    #compute joint frame (not unique, only rotation axis must coincide)
    B = ComputeOrthonormalBasis(vAxis) #axis = x-axis

    #interchange z and x axis (needs sign change, otherwise det(A)=-1)
    AJ = np.eye(3)
    AJ[:,0]=-B[:,2]
    AJ[:,1]= B[:,1]
    AJ[:,2]= B[:,0] #axis ==> rotation axis z for revolute joint ... 
    
    #compute joint position and axis in bodyNumber0 / 1 coordinates:
    pJ0 = _MatVec3(A0.T, np.array(pJoint) - p0)
    pJ1 = _MatVec3(A1.T, np.array(pJoint) - p1)

    #compute joint marker orientations:
    MR0 = _MatMul3x3(A0.T, AJ)  
    MR1 = _MatMul3x3(A1.T, AJ)  
    
    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name

    if mBody0 is None: #the rotation is the marker's (#2745)
        mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=itemNumbers[0], localHT=exu.HT(rotation=MR0, translation=pJ0)))
        MR0 = np.eye(3)
    if mBody1 is None:
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=itemNumbers[1], localHT=exu.HT(rotation=MR1, translation=pJ1)))
        MR1 = np.eye(3)
    (mBody0, MR0) = _MarkerWithRotation(mbs, mBody0, MR0) #a marker the caller gave: a copy turned by the rotation (#2804)
    (mBody1, MR1) = _MarkerWithRotation(mbs, mBody1, MR1)

    if unlimitedRotations:
        nGeneric = mbs.AddNode(eii.NodeGenericData(initialCoordinates=[0], 
                                             numberOfDataCoordinates=1)) #for infinite rotations
    else:
        nGeneric = exudyn.InvalidIndex()

    oConnector = mbs.AddObject(eii.ObjectConnectorTorsionalSpringDamper(name=name,
                                                                        markerNumbers = [mBody0,mBody1],
                                                                        nodeNumber = nGeneric,
                                                                        stiffness = stiffness, 
                                                                        damping = damping, 
                                                                        offset = offset,
                                                                        velocityOffset = velocityOffset,
                                                                        torque = torque,
                                                                        **_RotationMarkerArgs(MR0, MR1), #only a rotation a marker cannot take (#2820)
                                                                        springTorqueUserFunction=springTorqueUserFunction, 
                                                                        visualization=eii.VTorsionalSpringDamper(show=show, 
                                                                                      drawSize=drawSize, color=color)
                                                                      ))

    
    return oConnector


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateLinearSpringDamper(mbs,
                                       name='',
                                       itemNumbers=[None, None],
                                       position = [0.,0.,0.],
                                       axis = [1.,0.,0.],
                                       stiffness = 0.,
                                       damping = 0.,
                                       offset = 0.,
                                       velocityOffset = 0.,
                                       force = 0.,
                                       useGlobalFrame=True,
                                       springForceUserFunction=0,
                                       show=True, drawSize=-1, color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """helper function to create a LinearSpringDamper connector, acting along an axis between two rigid bodies, using arguments from ObjectConnectorLinearSpringDamper, see there for the full documentation; usually used together with a prismatic joint

    Args:
        mbs: the MainSystem where items are created
        name: name string for connector; markers get Marker0:name and Marker1:name
        itemNumbers: a list of two items to be connected, each a rigid body or a ground object (ObjectIndex) or a rigid marker (MarkerIndex); with a marker, position is [], and the marker sets the position of the spring-damper
        position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the spring-damper in reference configuration; else: local position in body0
        axis: a 3D vector as list or np.array containing the axis along which the spring acts, either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True); it is normalized
        stiffness: scalar stiffness of spring
        damping: scalar damping added to spring
        offset: scalar offset of the displacement along the axis, at which the spring force is zero
        velocityOffset: scalar velocity offset, which can be used to realize a D-controlled actuator
        force: additional constant force added to spring-damper, acting between the two bodies along the axis
        useGlobalFrame: if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
        springForceUserFunction: a user function springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset)->float ; this function replaces the internal connector force computation
        show: if True, connector visualization is drawn
        drawSize: general drawing size of connector
        color: color of connector

    Returns:
        :ObjectIndex: returns index of newly created object

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        oGround = mbs.CreateGround()
        b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=1000, sideLengths=[0.2,0.1,0.1]),
                                 referencePosition = [1,0,0], gravity = [-9.81,0,0])
        mbs.CreatePrismaticJoint(itemNumbers=[oGround, b0], position=[1,0,0], axis=[1,0,0])
        mbs.CreateLinearSpringDamper(itemNumbers=[oGround, b0], position=[1,0,0], axis=[1,0,0],
                                     stiffness=1e3, damping=10)
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    where='MainSystem.CreateLinearSpringDamper(...)'

    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)
        if not IsVector(axis, 3) or cMath.norm(axis) == 0:
            RaiseTypeError(where=where, argumentName='axis', received = axis, expectedType = 'a vector of length 3, not zero')
        if not IsValidURealInt(stiffness):
            RaiseTypeError(where=where, argumentName='stiffness', received = stiffness, expectedType = ExpectedType.UReal)
        if not IsValidURealInt(damping):
            RaiseTypeError(where=where, argumentName='damping', received = damping, expectedType = ExpectedType.UReal)
        if not IsValidRealInt(offset):
            RaiseTypeError(where=where, argumentName='offset', received = offset, expectedType = ExpectedType.Real)
        if not IsValidRealInt(velocityOffset):
            RaiseTypeError(where=where, argumentName='velocityOffset', received = velocityOffset, expectedType = ExpectedType.Real)
        if not IsValidRealInt(force):
            RaiseTypeError(where=where, argumentName='force', received = force, expectedType = ExpectedType.Real)
        if not IsValidRealInt(drawSize):
            RaiseTypeError(where=where, argumentName='drawSize', received = drawSize, expectedType = ExpectedType.Real)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)

    #as the joints: a marker of each side at the position, the axis in the frame of marker 0 (#1953)
    [p0, A0, p1, A1, mBody0, mBody1, pPosition] = JointPreCheckCalcBodyMarkers(where, mbs, name, itemNumbers, position, show, useGlobalFrame)

    vAxis = np.array(axis, dtype=float) if useGlobalFrame else _MatVec3(A0, axis)
    vAxis = vAxis / cMath.norm(vAxis)

    mName0 = 'Marker0:'+name if name != '' else ''
    mName1 = 'Marker1:'+name if name != '' else ''
    if mBody0 is None:
        mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0, bodyNumber=itemNumbers[0],
                                                   localPosition=_MatVec3(A0.T, np.array(pPosition) - p0)))
    if mBody1 is None:
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1, bodyNumber=itemNumbers[1],
                                                   localPosition=_MatVec3(A1.T, np.array(pPosition) - p1)))

    oConnector = mbs.AddObject(eii.ObjectConnectorLinearSpringDamper(name=name,
                                                                     markerNumbers = [mBody0,mBody1],
                                                                     axisMarker0 = _MatVec3(A0.T, vAxis),
                                                                     stiffness = stiffness,
                                                                     damping = damping,
                                                                     offset = offset,
                                                                     velocityOffset = velocityOffset,
                                                                     force = force,
                                                                     springForceUserFunction=springForceUserFunction,
                                                                     visualization=eii.VLinearSpringDamper(show=show,
                                                                                   drawSize=drawSize, color=color)
                                                                     ))
    return oConnector




#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateRevoluteJoint(mbs, name='', itemNumbers=[None, None], 
                                  position=[], axis=[], useGlobalFrame=True, 
                                  show=True, axisRadius=0.1, axisLength=0.4, color=exudyn.graphics.color.default, bodyNumbers=None, bodyOrNodeList=None) -> exudyn.ObjectIndex:
    """Create revolute joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        itemNumbers: a list of two items to be connected, each a rigid body or a ground object (ObjectIndex) or a rigid marker (MarkerIndex); with a marker, position is [], and the marker sets the position of the joint
        position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
        axis: a 3D vector as list or np.array containing the joint axis either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True)
        useGlobalFrame: if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
        show: if True, connector visualization is drawn
        axisRadius: radius of axis for connector graphical representation
        axisLength: length of axis for connector graphical representation
        color: color of connector
        bodyNumbers: deprecated name of itemNumbers
        bodyOrNodeList: not accepted, a TypeError: the items are given in itemNumbers

    Returns:
        :ObjectIndex: returns index of created joint

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                         sideLengths=[1,0.1,0.1]),
                                 referencePosition = [3,0,0],
                                 gravity = [0,-9.81,0],
                                 graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                              color=exu.graphics.color.steelblue)])
        oGround = mbs.AddObject(ObjectGround())
        mbs.CreateRevoluteJoint(itemNumbers=[oGround, b0], position=[2.5,0,0], axis=[0,0,1],
                                useGlobalFrame=True, axisRadius=0.02, axisLength=0.14)
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    where = 'MainSystem.CreateRevoluteJoint(...)'
    if bodyNumbers is not None: #the old name of the argument (#2863)
        DeprecatedArgument('bodyNumbers', '1.12.337', 2031, use='itemNumbers', function='MainSystem.CreateRevoluteJoint')
    itemNumbers = ItemNumbersArgument(where, itemNumbers, bodyNumbers, bodyOrNodeList)
    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsVector(axis, 3):
            RaiseTypeError(where=where, argumentName='axis', received = axis, expectedType = ExpectedType.Vector, dim=3)
    
        if not IsValidRealInt(axisRadius):
            RaiseTypeError(where=where, argumentName='axisRadius', received = axisRadius, expectedType = ExpectedType.Real)
        if not IsValidRealInt(axisLength):
            RaiseTypeError(where=where, argumentName='axisLength', received = axisLength, expectedType = ExpectedType.Real)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)

    #similar to RevoluteJoint!
    [p0, A0, p1, A1, mBody0, mBody1, pJoint] = JointPreCheckCalcBodyMarkers(where, mbs, name, itemNumbers, position, show, useGlobalFrame)
        
    if useGlobalFrame:
        vAxis = copy.copy(axis)
    else: #transform into global coordinates, then everything works same
        vAxis = A0 @ axis

    #compute joint frame (not unique, only rotation axis must coincide)
    B = ComputeOrthonormalBasis(vAxis) #axis = x-axis
    
    #interchange z and x axis (needs sign change, otherwise det(A)=-1)
    AJ = np.eye(3)
    AJ[:,0]=-B[:,2]
    AJ[:,1]= B[:,1]
    AJ[:,2]= B[:,0] #axis ==> rotation axis z for revolute joint ... 
    
    #compute joint position and axis in bodyNumber0 / 1 coordinates:
    pJ0 = _MatVec3(A0.T, np.array(pJoint) - p0)
    pJ1 = _MatVec3(A1.T, np.array(pJoint) - p1)

    #compute joint marker orientations:
    MR0 = _MatMul3x3(A0.T, AJ)  
    MR1 = _MatMul3x3(A1.T, AJ)  
    
    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name

    if mBody0 is None: #the rotation is the marker's (#2745)
        mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=itemNumbers[0], localHT=exu.HT(rotation=MR0, translation=pJ0)))
        MR0 = np.eye(3)
    if mBody1 is None:
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=itemNumbers[1], localHT=exu.HT(rotation=MR1, translation=pJ1)))
        MR1 = np.eye(3)
    (mBody0, MR0) = _MarkerWithRotation(mbs, mBody0, MR0) #a marker the caller gave: a copy turned by the rotation (#2804)
    (mBody1, MR1) = _MarkerWithRotation(mbs, mBody1, MR1)
    
    oJoint = mbs.AddObject(eii.ObjectJointRevoluteZ(name=name,markerNumbers=[mBody0,mBody1],
                                                **_RotationMarkerArgs(MR0, MR1), #only a rotation a marker cannot take (#2820)
             visualization=eii.VRevoluteJointZ(show=show, axisRadius=axisRadius, axisLength=axisLength, color=color) ))

    return oJoint


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreatePrismaticJoint(mbs, name='', itemNumbers=[None, None], 
                                  position=[], axis=[], useGlobalFrame=True, 
                                  show=True, axisRadius=0.1, axisLength=0.4, color=exudyn.graphics.color.default, bodyNumbers=None, bodyOrNodeList=None) -> exudyn.ObjectIndex:
    """Create prismatic joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        itemNumbers: a list of two items to be connected, each a rigid body or a ground object (ObjectIndex) or a rigid marker (MarkerIndex); with a marker, position is [], and the marker sets the position of the joint
        position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
        axis: a 3D vector as list or np.array containing the joint axis either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True)
        useGlobalFrame: if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
        show: if True, connector visualization is drawn
        axisRadius: radius of axis for connector graphical representation
        axisLength: length of axis for connector graphical representation
        color: color of connector
        bodyNumbers: deprecated name of itemNumbers
        bodyOrNodeList: not accepted, a TypeError: the items are given in itemNumbers

    Returns:
        :ObjectIndex: returns index of created joint

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                         sideLengths=[1,0.1,0.1]),
                                 referencePosition = [4,0,0],
                                 initialVelocity = [0,4,0],
                                 gravity = [0,-9.81,0],
                                 graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                              color=exu.graphics.color.steelblue)])
        oGround = mbs.AddObject(ObjectGround())
        mbs.CreatePrismaticJoint(itemNumbers=[oGround, b0], position=[3.5,0,0], axis=[0,1,0],
                                 useGlobalFrame=True, axisRadius=0.02, axisLength=1)
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    where = 'MainSystem.CreatePrismaticJoint(...)'
    if bodyNumbers is not None: #the old name of the argument (#2863)
        DeprecatedArgument('bodyNumbers', '1.12.337', 2031, use='itemNumbers', function='MainSystem.CreatePrismaticJoint')
    itemNumbers = ItemNumbersArgument(where, itemNumbers, bodyNumbers, bodyOrNodeList)
    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsVector(axis, 3):
            RaiseTypeError(where=where, argumentName='axis', received = axis, expectedType = ExpectedType.Vector, dim=3)
    
        if not IsValidRealInt(axisRadius):
            RaiseTypeError(where=where, argumentName='axisRadius', received = axisRadius, expectedType = ExpectedType.Real)
        if not IsValidRealInt(axisLength):
            RaiseTypeError(where=where, argumentName='axisLength', received = axisLength, expectedType = ExpectedType.Real)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)

    [p0, A0, p1, A1, mBody0, mBody1, pJoint] = JointPreCheckCalcBodyMarkers(where, mbs, name, itemNumbers, position, show, useGlobalFrame)
        
    if useGlobalFrame:
        vAxis = copy.copy(axis)
    else: #transform into global coordinates, then everything works same
        vAxis = A0 @ axis

    #compute joint frame (not unique, only rotation axis must coincide)
    AJ = ComputeOrthonormalBasis(vAxis) #axis = x-axis
    
    #compute joint position and axis in bodyNumber0 / 1 coordinates:
    pJ0 = _MatVec3(A0.T, np.array(pJoint) - p0)
    pJ1 = _MatVec3(A1.T, np.array(pJoint) - p1)

    #compute joint marker orientations:
    MR0 = _MatMul3x3(A0.T, AJ)  
    MR1 = _MatMul3x3(A1.T, AJ)  
    
    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name

    if mBody0 is None: #the rotation is the marker's (#2745)
        mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=itemNumbers[0], localHT=exu.HT(rotation=MR0, translation=pJ0)))
        MR0 = np.eye(3)
    if mBody1 is None:
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=itemNumbers[1], localHT=exu.HT(rotation=MR1, translation=pJ1)))
        MR1 = np.eye(3)
    (mBody0, MR0) = _MarkerWithRotation(mbs, mBody0, MR0) #a marker the caller gave: a copy turned by the rotation (#2804)
    (mBody1, MR1) = _MarkerWithRotation(mbs, mBody1, MR1)
    
    oJoint = mbs.AddObject(eii.ObjectJointPrismaticX(name=name,markerNumbers=[mBody0,mBody1],
                                                **_RotationMarkerArgs(MR0, MR1), #only a rotation a marker cannot take (#2820)
             visualization=eii.VPrismaticJointX(show=show, axisRadius=axisRadius, axisLength=axisLength, color=color) ))

    return oJoint


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateSphericalJoint(mbs, name='', itemNumbers=[None, None], 
                                  position=[], constrainedAxes=[1,1,1], useGlobalFrame=True, 
                                  show=True, jointRadius=0.1, color=exudyn.graphics.color.default, bodyNumbers=None, bodyOrNodeList=None) -> exudyn.ObjectIndex:
    """Create spherical joint between two bodies; definition of joint position in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers are automatically computed

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        itemNumbers: a list of two items to be connected, each a mass point, a rigid body or a ground object (ObjectIndex) or a position marker (MarkerIndex); with a marker, position is [], and the marker sets the position of the joint
        position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
        constrainedAxes: flags, which determines which (global) translation axes are constrained; each entry may only be 0 (=free) axis or 1 (=constrained axis)
        useGlobalFrame: if False, the point and axis vectors are defined in the local coordinate system of body0
        show: if True, connector visualization is drawn
        jointRadius: radius of sphere for connector graphical representation
        color: color of connector
        bodyNumbers: deprecated name of itemNumbers
        bodyOrNodeList: not accepted, a TypeError: the items are given in itemNumbers

    Returns:
        :ObjectIndex: returns index of created joint

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                         sideLengths=[1,0.1,0.1]),
                                 referencePosition = [5,0,0],
                                 initialAngularVelocity = [5,0,0],
                                 gravity = [0,-9.81,0],
                                 graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                              color=exu.graphics.color.orange)])
        oGround = mbs.AddObject(ObjectGround())
        mbs.CreateSphericalJoint(itemNumbers=[oGround, b0], position=[5.5,0,0],
                                 useGlobalFrame=True, jointRadius=0.06)
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    where = 'MainSystem.CreateSphericalJoint(...)'
    if bodyNumbers is not None: #the old name of the argument (#2863)
        DeprecatedArgument('bodyNumbers', '1.12.337', 2031, use='itemNumbers', function='MainSystem.CreateSphericalJoint')
    itemNumbers = ItemNumbersArgument(where, itemNumbers, bodyNumbers, bodyOrNodeList)
    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsIntVector(constrainedAxes, 3):
            RaiseTypeError(where=where, argumentName='constrainedAxes', received = constrainedAxes, expectedType = ExpectedType.IntVector, dim=3)
        if not IsValidRealInt(jointRadius):
            RaiseTypeError(where=where, argumentName='jointRadius', received = jointRadius, expectedType = ExpectedType.Real)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)

    #similar to RevoluteJoint!
    [p0, A0, p1, A1, mBody0, mBody1, pJoint] = JointPreCheckCalcBodyMarkers(where, mbs, name, itemNumbers, position, show, useGlobalFrame, requireRotMat=False)
        
    #compute joint position and axis in bodyNumber0 / 1 coordinates:
    pJ0 = _MatVec3(A0.T, np.array(pJoint) - p0)
    pJ1 = _MatVec3(A1.T, np.array(pJoint) - p1)

    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name

    if mBody0 is None: mBody0 = mbs.AddMarker(eii.MarkerBodyPosition(name=mName0,bodyNumber=itemNumbers[0], localPosition=pJ0))
    if mBody1 is None: mBody1 = mbs.AddMarker(eii.MarkerBodyPosition(name=mName1,bodyNumber=itemNumbers[1], localPosition=pJ1))
    
    oJoint = mbs.AddObject(eii.ObjectJointSpherical(name=name,markerNumbers=[mBody0,mBody1], 
                                                    constrainedAxes=constrainedAxes,
             visualization=eii.VObjectJointSpherical(show=show, jointRadius=jointRadius, color=color) ))

    return oJoint



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateGenericJoint(mbs, name='', itemNumbers=[None, None], 
                                 position=[], 
                                 rotationMatrixAxes=np.eye(3), 
                                 constrainedAxes=[1,1,1, 1,1,1], 
                                 useGlobalFrame=True,
                                 offsetUserFunction=0, offsetUserFunction_t=0,
                                 show=True, axesRadius=0.1, axesLength=0.4, color=exudyn.graphics.color.default, bodyNumbers=None, bodyOrNodeList=None) -> exudyn.ObjectIndex:
    """Create generic joint between two bodies; definition of joint position (position) and axes (rotationMatrixAxes) in global coordinates (useGlobalFrame=True) or in local coordinates of body0 (useGlobalFrame=False), where rotationMatrixAxes is an additional rotation to body0; all markers, markerRotation and other quantities are automatically computed

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        itemNumbers: a list of two items to be connected, each a rigid body or a ground object (ObjectIndex) or a rigid marker (MarkerIndex); with a marker, position is [], and the marker sets the position of the joint
        position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
        rotationMatrixAxes: rotation matrix which defines orientation of constrainedAxes; if useGlobalFrame, this rotation matrix is global, else the rotation matrix is post-multiplied with the rotation of body0, identical with rotationMarker0 in the joint
        constrainedAxes: flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; each entry may only be 0 (=free) axis or 1 (=constrained axis); ALL constrained Axes are defined relative to reference rotation of body0 times rotation0
        useGlobalFrame: if False, the position is defined in the local coordinate system of body0, otherwise it is defined in global coordinates
        offsetUserFunction: a user function offsetUserFunction(mbs, t, itemNumber, offsetUserFunctionParameters)->float ; this function replaces the internal (constant) by a user-defined offset. This allows to realize rheonomic joints and allows kinematic simulation
        offsetUserFunction_t: a user function offsetUserFunction_t(mbs, t, itemNumber, offsetUserFunctionParameters)->float ; this function replaces the internal (constant) by a user-defined offset velocity; this function is used instead of offsetUserFunction, if velocityLevel (index2) time integration
        show: if True, connector visualization is drawn
        axesRadius: radius of axes for connector graphical representation
        axesLength: length of axes for connector graphical representation
        color: color of connector
        bodyNumbers: deprecated name of itemNumbers
        bodyOrNodeList: not accepted, a TypeError: the items are given in itemNumbers

    Returns:
        :ObjectIndex: returns index of created joint

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                         sideLengths=[1,0.1,0.1]),
                                 referencePosition = [6,0,0],
                                 initialAngularVelocity = [0,8,0],
                                 gravity = [0,-9.81,0],
                                 graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                              color=exu.graphics.color.orange)])
        oGround = mbs.AddObject(ObjectGround())
        mbs.CreateGenericJoint(itemNumbers=[oGround, b0], position=[5.5,0,0],
                               constrainedAxes=[1,1,1, 1,0,0],
                               rotationMatrixAxes=RotationMatrixX(0.125*pi), #tilt axes
                               useGlobalFrame=True, axesRadius=0.02, axesLength=0.2)
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    where = 'MainSystem.CreateGenericJoint(...)'
    if bodyNumbers is not None: #the old name of the argument (#2863)
        DeprecatedArgument('bodyNumbers', '1.12.337', 2031, use='itemNumbers', function='MainSystem.CreateGenericJoint')
    itemNumbers = ItemNumbersArgument(where, itemNumbers, bodyNumbers, bodyOrNodeList)
    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsIntVector(constrainedAxes, 6):
            RaiseTypeError(where=where, argumentName='constrainedAxes', received = constrainedAxes, expectedType = ExpectedType.IntVector, dim=6)
    
        if not IsValidRealInt(axesRadius):
            RaiseTypeError(where=where, argumentName='axesRadius', received = axesRadius, expectedType = ExpectedType.Real)
        if not IsValidRealInt(axesLength):
            RaiseTypeError(where=where, argumentName='axesLength', received = axesLength, expectedType = ExpectedType.Real)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)

    [p0, A0, p1, A1, mBody0, mBody1, pJoint] = JointPreCheckCalcBodyMarkers(where, mbs, name, itemNumbers, position, show, useGlobalFrame)
        
    if useGlobalFrame:
        #compute joint marker orientations, rotationMatrixAxes represents global frame:
        MR0 = _MatMul3x3(A0.T, rotationMatrixAxes)
        MR1 = _MatMul3x3(A1.T, rotationMatrixAxes)
    else: #transform into global coordinates, then everything works same
        #compute joint marker orientations, rotationMatrixAxes represents local frame:
        MR0 = copy.copy(rotationMatrixAxes)
        MR1 = _MatMul3x3(_MatMul3x3(A1.T, A0), rotationMatrixAxes)

    
    #compute joint position and axis in bodyNumber0 / 1 coordinates:
    pJ0 = _MatVec3(A0.T, np.array(pJoint) - p0)
    pJ1 = _MatVec3(A1.T, np.array(pJoint) - p1)

    
    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name

    if mBody0 is None: #the rotation is the marker's (#2745)
        mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=itemNumbers[0], localHT=exu.HT(rotation=MR0, translation=pJ0)))
        MR0 = np.eye(3)
    if mBody1 is None:
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=itemNumbers[1], localHT=exu.HT(rotation=MR1, translation=pJ1)))
        MR1 = np.eye(3)
    (mBody0, MR0) = _MarkerWithRotation(mbs, mBody0, MR0) #a marker the caller gave: a copy turned by the rotation (#2804)
    (mBody1, MR1) = _MarkerWithRotation(mbs, mBody1, MR1)
    
    oJoint = mbs.AddObject(eii.ObjectJointGeneric(name=name,markerNumbers=[mBody0,mBody1],
                                                  constrainedAxes = constrainedAxes,
                                                  **_RotationMarkerArgs(MR0, MR1), #only a rotation a marker cannot take (#2820)
                                                  offsetUserFunction=offsetUserFunction,
                                                  offsetUserFunction_t=offsetUserFunction_t,
             visualization=eii.VObjectJointGeneric(show=show, axesRadius=axesRadius, axesLength=axesLength, color=color) ))

    return oJoint


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateDistanceConstraint(mbs, name='', 
                                       itemNumbers=[None, None], 
                                       localPosition0 = [0.,0.,0.],
                                       localPosition1 = [0.,0.,0.], 
                                       distance=None, 
                                       show=True, drawSize=-1., color=exudyn.graphics.color.default, bodyNumbers=None, bodyOrNodeList=None) -> exudyn.ObjectIndex:
    """Create distance joint between two bodies; definition of joint positions in local coordinates of bodies or nodes; if distance=None, it is computed automatically from reference length; all markers are automatically computed

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        itemNumbers: a list of two items to be connected, each a body (ObjectIndex) at localPosition0/1, a node (NodeIndex) or a marker (MarkerIndex); for a node or a marker, localPosition0/1 is [0,0,0]
        localPosition0: local position (as 3D list or numpy array) on body0, if not a node or marker number
        localPosition1: local position (as 3D list or numpy array) on body1, if not a node or marker number
        distance: if None, distance is computed from reference position of bodies or nodes; if not None, this distance is prescribed between the two positions; if distance = 0, it will create a SphericalJoint as this case is not possible with a DistanceConstraint
        show: if True, connector visualization is drawn
        drawSize: general drawing size of node
        color: color of connector
        bodyNumbers: deprecated name of itemNumbers
        bodyOrNodeList: not accepted, a TypeError: the items are given in itemNumbers

    Returns:
        :ObjectIndex: returns index of created joint

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                          sideLengths=[1,0.1,0.1]),
                                  referencePosition = [6,0,0],
                                  gravity = [0,-9.81,0],
                                  graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                              color=exu.graphics.color.orange)])
        m1 = mbs.CreateMassPoint(referencePosition=[5.5,-1,0],
                                 mass=1, drawSize = 0.2)
        n1 = mbs.GetObject(m1)['nodeNumber']
        oGround = mbs.AddObject(ObjectGround())
        mbs.CreateDistanceConstraint(itemNumbers=[oGround, b0],
                                     localPosition0 = [6.5,1,0],
                                     localPosition1 = [0.5,0,0],
                                     distance=None, #automatically computed
                                     drawSize=0.06)
        mbs.CreateDistanceConstraint(itemNumbers=[b0, n1],
                                     localPosition0 = [-0.5,0,0],
                                     localPosition1 = [0.,0.,0.], #must be [0,0,0] for Node
                                     distance=None, #automatically computed
                                     drawSize=0.06)
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    where = 'MainSystem.CreateDistanceConstraint(...)'
    if bodyNumbers is not None: #the old name of the argument (#2863)
        DeprecatedArgument('bodyNumbers', '1.12.337', 2031, use='itemNumbers', function='MainSystem.CreateDistanceConstraint')
    itemNumbers = ItemNumbersArgument(where, itemNumbers, bodyNumbers, bodyOrNodeList)
    CheckItemNumbers(where, itemNumbers, [_bodyNodeMarker, _bodyNodeMarker], [localPosition0, localPosition1])
        
    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)
            
        if not IsVector(localPosition0, 3):
            RaiseTypeError(where=where, argumentName='localPosition0', received = localPosition0, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(localPosition1, 3):
            RaiseTypeError(where=where, argumentName='localPosition1', received = localPosition1, expectedType = ExpectedType.Vector, dim=3)
    
        if IsNotNone(distance) and not IsValidURealInt(distance):
            RaiseTypeError(where=where, argumentName='distance', received = distance, expectedType = ExpectedType.PReal)

        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsValidRealInt(drawSize):
            RaiseTypeError(where=where, argumentName='drawSize', received = drawSize, expectedType = ExpectedType.Real)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)


    [mBody0, mBody1, p0, p1, A0, A1] = GetMarkersPosRot(mbs, name, itemNumbers, localPosition0, localPosition1,
                                                        getPosition=IsNone(distance))
    if IsNone(distance): #automatically compute distance
        distance = cMath.norm(np.array(p1)-p0)
    
    if distance != 0:
        oJoint = mbs.AddObject(eii.ObjectConnectorDistance(name=name,markerNumbers=[mBody0,mBody1], distance=distance,
                 visualization=eii.VObjectConnectorDistance(show=show, drawSize=drawSize, color=color) ))
    else:
        #VERY SPECIAL case, which should help to resolve problems if distance=0 is used ... 
        exu.Print('WARNING: CreateDistanceConstraint called with distance=0; creating SphericalJoint instead')
        constrainedAxes = [1,1,1]
        for item in itemNumbers:
            if ItemKind(item) == 'body' and '2D' in mbs.GetObject(item)['objectType']:
                constrainedAxes[2] = 0
        oJoint = mbs.AddObject(eii.SphericalJoint(name=name,markerNumbers=[mBody0,mBody1], 
                                                  constrainedAxes=constrainedAxes,
                                                  visualization=eii.VSphericalJoint(show=show, jointRadius=0.5*drawSize, color=color) ))
        

    return oJoint



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateCoordinateConstraint(mbs, name='', 
                                        itemNumbers=[None, None], 
                                        coordinates=[None, None], 
                                        offset = 0.,
                                        factor1 = 1.,
                                        velocityLevel = False,
                                        offsetUserFunction = 0,
                                        offsetUserFunction_t = 0,
                                        show=True, drawSize=-1., color=exudyn.graphics.color.default,
                                        includeReferenceCoordinates=[False, False],
                                        factorValue1=None, bodyNumbers=None, bodyOrNodeList=None) -> exudyn.ObjectIndex:
    """Create coordinate constraint between two bodies or nodes, or one of them and the ground; markers and NodePointGround are automatically created when needed

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        itemNumbers: a list of the two items whose coordinates are constrained, each a body (ObjectIndex), whose coordinates are those of its nodes in their order, a node (NodeIndex), or None or a ground object for the ground
        coordinates: a list of the two coordinate indices: of the node, or of the body, counted over its nodes in their order; None for the ground
        offset: an fixed offset between the two coordinate values
        factor1: an additional factor multiplied with coordinate value1 used in algebraic equation, to enable (e.g. gear) ratio between coordinates
        velocityLevel: If true: connector constrains velocities (only works for ODE2 coordinates!); offset is used between velocities; if True, the offsetUserFunction_t is considered and offsetUserFunction is ignored
        offsetUserFunction: a Python function which defines the time-dependent offset; see description in CoordinateConstraint
        offsetUserFunction_t: time derivative of offsetUserFunction; needed for velocity level constraints; see description in CoordinateConstraint
        show: if True, connector visualization is drawn
        drawSize: general drawing size of node
        color: color of connector
        includeReferenceCoordinates: a list of two bools, one per item: True constrains the coordinate including its
            reference value, e.g. the absolute position instead of the displacement (MarkerNodeCoordinate, #990)
        factorValue1: deprecated name of factor1
        bodyNumbers: deprecated name of itemNumbers
        bodyOrNodeList: not accepted, a TypeError: the items are given in itemNumbers

    Returns:
        :ObjectIndex: returns index of created joint

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                          sideLengths=[1,0.1,0.1]),
                                  referencePosition = [6,0,0],
                                  gravity = [0,-9.81,0],
                                  graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                              color=exu.graphics.color.orange)])
        m1 = mbs.CreateMassPoint(referencePosition=[5.5,-1,0],
                                 mass=1, drawSize = 0.2)
        mbs.CreateCoordinateConstraint(itemNumbers=[None, b0],
                                       coordinates=[None, 0]) #constrains X-coordinate
        #constrain Y-coordinate of b0 to Z-coordinate of m1:
        mbs.CreateCoordinateConstraint(itemNumbers=[b0, m1],
                                       coordinates=[1, 2])
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    if factorValue1 is not None: #the old name of the argument (#2814)
        DeprecatedArgument('factorValue1', '1.12.258', 2031, use='factor1', function='MainSystem.CreateCoordinateConstraint')
        factor1 = factorValue1
    where = 'MainSystem.CreateCoordinateConstraint(...)'
    if bodyNumbers is not None: #the old name of the argument (#2863)
        DeprecatedArgument('bodyNumbers', '1.12.337', 2031, use='itemNumbers', function='MainSystem.CreateCoordinateConstraint')
    itemNumbers = ItemNumbersArgument(where, itemNumbers, bodyNumbers, bodyOrNodeList)
    CheckItemNumbers(where, itemNumbers, [('body', 'node', 'ground'), ('body', 'node', 'ground')])

    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)
        if not isinstance(coordinates, list) or len(coordinates) != 2:
            RaiseTypeError(where=where, argumentName='coordinates', received = coordinates, expectedType = 'list of 2 coordinate indices of the respective items')

        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsValidRealInt(drawSize):
            RaiseTypeError(where=where, argumentName='drawSize', received = drawSize, expectedType = ExpectedType.Real)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)
        if (not isinstance(includeReferenceCoordinates, (list, tuple)) or len(includeReferenceCoordinates) != 2
                or not all(IsValidBool(flag) for flag in includeReferenceCoordinates)):
            RaiseTypeError(where=where, argumentName='includeReferenceCoordinates', received = includeReferenceCoordinates,
                           expectedType = 'list of 2 bools')

    errStr = 'ERROR in ' + where + ': '
    kinds = ['ground' if item is None else ItemKind(item) for item in itemNumbers]
    kinds = ['ground' if kind == 'body' and mbs.GetObject(item)['objectType'] == 'Ground' else kind
             for (kind, item) in zip(kinds, itemNumbers)]
    if kinds == ['ground', 'ground']:
        raise ValueError(errStr+'one of the two items must be a body or a node, but received: '+str(itemNumbers))

    markerNumbers = [None,None]
    for (i, item, kind) in zip(range(2), itemNumbers, kinds):
        markerName = 'Marker'+str(i)+':'+name if name != '' else ''
        coordinate = coordinates[i]
        if kind == 'ground':
            nPointGround = mbs.AddNode(eii.NodePointGround(visualization=eii.VNodePointGround(show=False)))
            markerNumbers[i] = mbs.AddMarker(eii.MarkerNodeCoordinate(name=markerName, nodeNumber=nPointGround, coordinate=0,
                                                                      includeReferenceCoordinate=bool(includeReferenceCoordinates[i])))
            continue

        if not IsInteger(coordinate):
            raise ValueError(errStr+f'coordinates[{i}] = {coordinate} is no valid coordinate index')
        if kind == 'node':
            (node, nodeCoordinate) = (item, coordinate)
        else: #the coordinate of a body counts over its nodes
            objectDict = mbs.GetObject(item)
            nodeNumbers = [objectDict['nodeNumber']] if 'nodeNumber' in objectDict else objectDict['nodeNumbers']
            (node, nodeCoordinate) = (None, coordinate)
            for bodyNode in nodeNumbers:
                numberOfCoordinates = len(mbs.systemData.GetNodeLTGODE2(bodyNode))
                if nodeCoordinate < numberOfCoordinates:
                    node = bodyNode
                    break
                nodeCoordinate -= numberOfCoordinates
            if node is None:
                raise ValueError(errStr+f'itemNumbers[{i}] = {item}: the body has no coordinate {coordinate}')
        markerNumbers[i] = mbs.AddMarker(eii.MarkerNodeCoordinate(name=markerName, nodeNumber=node, coordinate=nodeCoordinate,
                                                                  includeReferenceCoordinate=bool(includeReferenceCoordinates[i])))

    #now we should have two markers
    oJoint = mbs.AddObject(eii.ObjectConnectorCoordinate( name=name,
                                                         markerNumbers=markerNumbers, 
                                                         offset = offset,
                                                         factor1 = factor1,
                                                         velocityLevel = velocityLevel,
                                                         offsetUserFunction = offsetUserFunction,
                                                         offsetUserFunction_t = offsetUserFunction_t,
                           visualization=eii.VObjectConnectorCoordinate(show=show, drawSize=drawSize, color=color) ))
       

    return oJoint


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateRollingDisc(mbs, name='', bodyNumbers=[None, None], 
                                axisPosition=[], axisVector = [1,0,0],
                                discRadius = 0., planePosition = [0,0,0], planeNormal = [0,0,1], 
                                constrainedAxes = [1,1,1],
                                activeConnector = True,
                                show=True, discWidth=0.1, color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """Create an ideal rolling disc joint between wheel rigid body and ground; the disc is infinitely thin and the ground is a perfectly flat plane; the wheel may lift off; definition of joint position and axis in global coordinates (alternatively in wheel (body1) local coordinates) for reference configuration of bodies; all markers and other quantities are automatically computed; some constraint conditions may be deactivated, e.g. to resolve redundancy of constraints for multi-wheel vehicles

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        bodyNumbers: a list of object numbers for body0=ground and body1=wheel; must be rigid body or ground object
        axisPosition: a 3D vector as list or np.array: position of wheel axis in local body1=wheel coordinates
        axisVector: a 3D vector as list or np.array containing the joint (=wheel) axis in local body1=wheel coordinates
        discRadius: radius of the disc
        planePosition: any 3D position vector of plane in ground object; given as local coordinates in ground object
        planeNormal: 3D normal vector of the rolling (contact) plane on ground; given as local coordinates in ground object
        constrainedAxes: [j0,j1,j2] flags, which determine which constraints are active, in which j0 represents the constraint for lateral motion, j1 longitudinal (forward/backward) motion and j2 represents the normal (contact) direction
        activeConnector: flag to activate or deactivate the joint
        show: if True, connector visualization is drawn
        discWidth: disc with, only used for drawing
        color: color of connector

    Returns:
        :ObjectIndex: returns index of created joint

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        r = 0.2
        oDisc = mbs.CreateRigidBody(inertia = InertiaCylinder(density=5000, length=0.1, outerRadius=r, axis=0),
                                  referencePosition = [1,0,r],
                                  initialAngularVelocity = [-3*2*pi,0,0],
                                  initialVelocity = [0,r*3*2*pi,0],
                                  gravity = [0,0,-9.81],
                                  graphicsDataList = [exu.graphics.Cylinder(pAxis = [-0.05,0,0], vAxis = [0.1,0,0], radius = r*0.99,
                                                                            color=exu.graphics.color.blue),
                                                      exu.graphics.Basis(length=2*r)])
        oGround = mbs.CreateGround(graphicsDataList=[exu.graphics.CheckerBoard(size=4)])
        mbs.CreateRollingDisc(bodyNumbers=[oGround, oDisc],
                              axisPosition=[0,0,0], axisVector=[1,0,0], #on local wheel frame
                              planePosition = [0,0,0], planeNormal = [0,0,1],  #in ground frame
                              discRadius = r,
                              discWidth=0.01, color=exu.graphics.color.steelblue)
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings()
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    where = 'MainSystem.CreateRollingDisc(...)'
    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsVector(axisPosition, 3):
            RaiseTypeError(where=where, argumentName='axisPosition', received = axisPosition, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(axisVector, 3):
            RaiseTypeError(where=where, argumentName='axisVector', received = axisVector, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(planePosition, 3):
            RaiseTypeError(where=where, argumentName='planePosition', received = planePosition, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(planeNormal, 3):
            RaiseTypeError(where=where, argumentName='planeNormal', received = planeNormal, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(constrainedAxes, 3):
            RaiseTypeError(where=where, argumentName='constrainedAxes', received = constrainedAxes, expectedType = ExpectedType.IntVector, dim=3)
    
        if not IsValidRealInt(discRadius):
            RaiseTypeError(where=where, argumentName='discRadius', received = discRadius, expectedType = ExpectedType.Real)
        if not IsValidRealInt(discWidth):
            RaiseTypeError(where=where, argumentName='discWidth', received = discWidth, expectedType = ExpectedType.Real)

        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)

        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)

    
    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name

    mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=bodyNumbers[0], localPosition=planePosition)) #ground
    mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=bodyNumbers[1], localPosition=axisPosition)) #wheel

    oJoint = mbs.AddObject(eii.ObjectJointRollingDisc(name=name,markerNumbers=[mBody0,mBody1],
                                                      constrainedAxes=constrainedAxes, discRadius = discRadius, 
                                                      discAxis = axisVector, planeNormal = planeNormal, 
                                                      activeConnector = activeConnector, 
                                                      visualization = eii.VObjectJointRollingDisc(show=show, 
                                                                                                  discWidth=discWidth, 
                                                                                                  color=color) ))

    return oJoint


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




@extends(exudyn.MainSystem)
def MainSystemCreateRollingDiscPenalty(mbs, name='', bodyNumbers=[None, None], 
                                  axisPosition=[], axisVector = [1,0,0],
                                  discRadius = 0., planePosition = [0,0,0], planeNormal = [0,0,1], 
                                  contactStiffness = 0., contactDamping = 0., 
                                  dryFriction = [0,0], dryFrictionAngle = 0., 
                                  dryFrictionProportionalZone = 0., viscousFriction = [0,0], 
                                  rollingViscousFriction = 0., useLinearProportionalZone = False, 
                                  activeConnector = True, 
                                  show=True, discWidth=0.1, color=exudyn.graphics.color.default, rollingFrictionViscous=None) -> exudyn.ObjectIndex:
    """Create penalty-based rolling disc joint between wheel rigid body and ground; the disc is infinitely thin and the ground is a perfectly flat plane; the wheel may lift off; definition of joint position and axis in global coordinates (alternatively in wheel (body1) local coordinates) for reference configuration of bodies; all markers and other quantities are automatically computed

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        bodyNumbers: a list of object numbers for body0=ground and body1=wheel; must be rigid body or ground object
        axisPosition: a 3D vector as list or np.array: position of wheel axis in local body1=wheel coordinates
        axisVector: a 3D vector as list or np.array containing the joint (=wheel) axis in local body1=wheel coordinates
        discRadius: radius of the disc
        planePosition: any 3D position vector of plane in ground object; given as local coordinates in ground object
        planeNormal: 3D normal vector of the rolling (contact) plane on ground; given as local coordinates in ground object
        dryFrictionAngle: angle (radiant) which defines a rotation of the local tangential coordinates dry friction; this allows to model Mecanum wheels with specified roll angle
        contactStiffness: normal contact stiffness
        contactDamping: normal contact damping
        dryFriction: 2D list of friction parameters; dry friction coefficients in local wheel coordinates, where for dryFrictionAngle=0, the first parameter refers to forward direction and the second parameter to lateral direction
        viscousFriction: 2D list of viscous friction coefficients [SI:1/(m/s)] in local wheel coordinates; proportional to slipping velocity, leading to increasing slipping friction force for increasing slipping velocity; directions are same as in dryFriction
        dryFrictionProportionalZone: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations)
        rollingViscousFriction: viscous rolling friction [SI:s/m]: the force acts against the velocity of the trail on ground and is proportional to this velocity and to the contact normal force;
        useLinearProportionalZone: if True, a linear proportional zone is used; the linear zone performs better in implicit time integration as the Jacobian has a constant tangent in the sticking case
        activeConnector: flag to activate or deactivate the connector
        show: if True, connector visualization is drawn
        discWidth: disc with, only used for drawing
        color: color of connector
        rollingFrictionViscous: deprecated name of rollingViscousFriction

    Returns:
        :ObjectIndex: returns index of created joint

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        r = 0.2
        oDisc = mbs.CreateRigidBody(inertia = InertiaCylinder(density=5000, length=0.1, outerRadius=r, axis=0),
                                  referencePosition = [1,0,r],
                                  initialAngularVelocity = [-3*2*pi,0,0],
                                  initialVelocity = [0,r*3*2*pi,0],
                                  gravity = [0,0,-9.81],
                                  graphicsDataList = [exu.graphics.Cylinder(pAxis = [-0.05,0,0], vAxis = [0.1,0,0], radius = r*0.99,
                                                                            color=exu.graphics.color.blue),
                                                      exu.graphics.Basis(length=2*r)])
        oGround = mbs.CreateGround(graphicsDataList=[exu.graphics.CheckerBoard(size=4)])
        mbs.CreateRollingDiscPenalty(bodyNumbers=[oGround, oDisc], axisPosition=[0,0,0], axisVector=[1,0,0],
                                      discRadius = r, planePosition = [0,0,0], planeNormal = [0,0,1],
                                      dryFriction = [0.2,0.2],
                                      contactStiffness = 1e5, contactDamping = 2e3,
                                      discWidth=0.01, color=exu.graphics.color.steelblue)
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings()
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    if rollingFrictionViscous is not None: #the old name of the argument (#2814)
        DeprecatedArgument('rollingFrictionViscous', '1.12.258', 2031, use='rollingViscousFriction', function='MainSystem.CreateRollingDiscPenalty')
        rollingViscousFriction = rollingFrictionViscous
    where = 'MainSystem.CreateRollingDiscPenalty(...)'
    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsVector(axisPosition, 3):
            RaiseTypeError(where=where, argumentName='axisPosition', received = axisPosition, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(axisVector, 3):
            RaiseTypeError(where=where, argumentName='axisVector', received = axisVector, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(planePosition, 3):
            RaiseTypeError(where=where, argumentName='planePosition', received = planePosition, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(planeNormal, 3):
            RaiseTypeError(where=where, argumentName='planeNormal', received = planeNormal, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(dryFriction, 2):
            RaiseTypeError(where=where, argumentName='dryFriction', received = dryFriction, expectedType = ExpectedType.Vector, dim=2)
        if not IsVector(viscousFriction, 2):
            RaiseTypeError(where=where, argumentName='viscousFriction', received = viscousFriction, expectedType = ExpectedType.Vector, dim=2)
    
        if not IsValidRealInt(discRadius):
            RaiseTypeError(where=where, argumentName='discRadius', received = discRadius, expectedType = ExpectedType.Real)
        if not IsValidRealInt(contactStiffness):
            RaiseTypeError(where=where, argumentName='contactStiffness', received = contactStiffness, expectedType = ExpectedType.Real)
        if not IsValidRealInt(contactDamping):
            RaiseTypeError(where=where, argumentName='contactDamping', received = contactDamping, expectedType = ExpectedType.Real)
        if not IsValidRealInt(dryFrictionAngle):
            RaiseTypeError(where=where, argumentName='dryFrictionAngle', received = dryFrictionAngle, expectedType = ExpectedType.Real)
        if not IsValidRealInt(dryFrictionProportionalZone):
            RaiseTypeError(where=where, argumentName='dryFrictionProportionalZone', received = dryFrictionProportionalZone, expectedType = ExpectedType.Real)
        if not IsValidRealInt(rollingViscousFriction):
            RaiseTypeError(where=where, argumentName='rollingViscousFriction', received = rollingViscousFriction, expectedType = ExpectedType.Real)
        if not IsValidRealInt(useLinearProportionalZone):
            RaiseTypeError(where=where, argumentName='useLinearProportionalZone', received = useLinearProportionalZone, expectedType = ExpectedType.Real)
        if not IsValidRealInt(discWidth):
            RaiseTypeError(where=where, argumentName='discWidth', received = discWidth, expectedType = ExpectedType.Real)

        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)

        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)

    
    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name

    mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=bodyNumbers[0], localPosition=planePosition)) #ground
    mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=bodyNumbers[1], localPosition=axisPosition)) #wheel
    nGeneric = mbs.AddNode(eii.NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3) )
    
    oJoint = mbs.AddObject(eii.ObjectConnectorRollingDiscPenalty(name=name,markerNumbers=[mBody0,mBody1],
                                                                 nodeNumber = nGeneric, 
                                                                 discRadius = discRadius, discAxis = axisVector, planeNormal = planeNormal, 
                                                                 contactStiffness = contactStiffness, contactDamping = contactDamping, 
                                                                 dryFriction = dryFriction, dryFrictionAngle = dryFrictionAngle, 
                                                                 dryFrictionProportionalZone = dryFrictionProportionalZone, viscousFriction = viscousFriction, 
                                                                 rollingViscousFriction = rollingViscousFriction, useLinearProportionalZone = useLinearProportionalZone, 
                                                                 activeConnector = activeConnector, 
                                                                 visualization = eii.VObjectConnectorRollingDiscPenalty(show=show, discWidth=discWidth, 
                                                                                                                        color=color) ))
                           
    return oJoint



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateSphereSphereContact(mbs, name='', itemNumbers=[None, None], 
                                       localPosition0 = [0.,0.,0.], localPosition1 = [0.,0.,0.], 
                                       spheresRadii = [-1,-1], isHollowSphere1 = False,
                                       dynamicFriction = 0., frictionProportionalZone = 1e-3,
                                       contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1,
                                       constantPullOffForce = 0, contactPlasticityRatio = 0, adhesionCoefficient = 0, adhesionExponent = 1,
                                       restitutionCoefficient = 1, minimumImpactVelocity = 0,
                                       impactModel = exu.ContactImpactModel.Linear,
                                       dataInitialCoordinates = [0,0,0,0],
                                       activeConnector=True,
                                       show=False, color=exudyn.graphics.color.default, bodyNumbers=None, bodyOrNodeList=None) -> exudyn.ObjectIndex:
    """Create penalty-based sphere-sphere contact between two rigid bodies, mass points (if friction coefficient is zero) or according nodes; the contact is based on ObjectContactSphereSphere; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        itemNumbers: a list of the two items the spheres are attached to, each a body (ObjectIndex) at localPosition0/1, a node (NodeIndex) or a marker (MarkerIndex); for a node or a marker, localPosition0/1 is [0,0,0]; with friction, a rigid body, node or marker; for a mass point, friction due to rolling is not accounted for
        localPosition0: local position (as 3D list or numpy array) of sphere0 on body0, if not a node number
        localPosition1: local position (as 3D list or numpy array) of sphere1 on body1, if not a node number
        spheresRadii: list containing radius of sphere 0 and radius of sphere 1 [SI:m].
        isHollowSphere1: flag, which determines, if sphere attached to marker 1 (radius 1) is a hollow sphere.
        dynamicFriction: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, Section Module: physics
        frictionProportionalZone: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), Section Module: physics
        contactStiffness: normal contact stiffness
        contactDamping: linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
        contactStiffnessExponent: exponent in normal contact model [SI:1]
        constantPullOffForce: constant adhesion force [SI:N]; Edinburgh Adhesive Elasto-Plastic Model
        contactPlasticityRatio: ratio of contact stiffness for first loading and unloading/reloading [SI:1]; Edinburgh Adhesive Elasto-Plastic Model; see ObjectContactSphereSphere
        adhesionCoefficient: coefficient for adhesion [SI:N/m]; Edinburgh Adhesive Elasto-Plastic Model; set to 0 to deactivate adhesion model
        adhesionExponent: exponent for adhesion coefficient [SI:1]; Edinburgh Adhesive Elasto-Plastic Model
        restitutionCoefficient: coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)
        minimumImpactVelocity: minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)
        impactModel: the impact model, exu.ContactImpactModel: Linear (only the linear contactDamping), HuntCrossley or GonthierCarvalhoMartins, the latter much more accurate regarding the coefficient of restitution in the full range (0, 1]; the integers 0, 1, 2 are accepted as well; in all models, the linear contactDamping is added, if not set to zero (#3327)
        dataInitialCoordinates: a list of four values for initialization of the data node, used for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused
        activeConnector: flag to activate or deactivate the connector
        show: if True, connector visualization is drawn
        color: color of connector
        bodyNumbers: deprecated name of itemNumbers
        bodyOrNodeList: not accepted, a TypeError: the items are given in itemNumbers

    Returns:
        :ObjectIndex: returns index of created joint

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        oGround = mbs.AddObject(ObjectGround())
        b0 = mbs.CreateMassPoint(referencePosition = [11,1,0],
                                 initialVelocity = [0.5,0,0],
                                 mass = 1, gravity = [0,-9.81,0],
                                 drawSize = 0.4, color=exu.graphics.color.blue)
        #a sphere of radius 0.2 falls onto a sphere of radius 0.5 fixed to the ground
        mbs.CreateSphereSphereContact(itemNumbers=[oGround, b0],
                                      localPosition0=[11,0,0],
                                      spheresRadii=[0.5,0.2],
                                      contactStiffness=1e4, contactDamping=10)
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    where = 'MainSystem.CreateSphereSphereContact(...)'
    if bodyNumbers is not None: #the old name of the argument (#2863)
        DeprecatedArgument('bodyNumbers', '1.12.337', 2031, use='itemNumbers', function='MainSystem.CreateSphereSphereContact')
    itemNumbers = ItemNumbersArgument(where, itemNumbers, bodyNumbers, bodyOrNodeList)

    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsVector(localPosition0, 3):
            RaiseTypeError(where=where, argumentName='localPosition0', received = localPosition0, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(localPosition1, 3):
            RaiseTypeError(where=where, argumentName='localPosition1', received = localPosition1, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(spheresRadii, 2):
            RaiseTypeError(where=where, argumentName='spheresRadii', received = spheresRadii, expectedType = ExpectedType.Vector, dim=2)

        if not IsValidBool(isHollowSphere1):
            RaiseTypeError(where=where, argumentName='isHollowSphere1', received = isHollowSphere1, expectedType = ExpectedType.Bool)
        if not IsValidURealInt(dynamicFriction):
            RaiseTypeError(where=where, argumentName='dynamicFriction', received = dynamicFriction, expectedType = ExpectedType.Real)
        if not IsValidURealInt(frictionProportionalZone):
            RaiseTypeError(where=where, argumentName='frictionProportionalZone', received = frictionProportionalZone, expectedType = ExpectedType.Real)

        if not IsValidURealInt(contactStiffness):
            RaiseTypeError(where=where, argumentName='contactStiffness', received = contactStiffness, expectedType = ExpectedType.Real)
        if not IsValidURealInt(contactDamping):
            RaiseTypeError(where=where, argumentName='contactDamping', received = contactDamping, expectedType = ExpectedType.Real)
        if not IsValidPRealInt(contactStiffnessExponent):
            RaiseTypeError(where=where, argumentName='contactStiffnessExponent', received = contactStiffnessExponent, expectedType = ExpectedType.Real)

        if not IsValidURealInt(constantPullOffForce):
            RaiseTypeError(where=where, argumentName='constantPullOffForce', received = constantPullOffForce, expectedType = ExpectedType.Real)
        if not IsValidURealInt(contactPlasticityRatio):
            RaiseTypeError(where=where, argumentName='contactPlasticityRatio', received = contactPlasticityRatio, expectedType = ExpectedType.Real)

        if not IsValidURealInt(adhesionCoefficient):
            RaiseTypeError(where=where, argumentName='adhesionCoefficient', received = adhesionCoefficient, expectedType = ExpectedType.Real)
        if not IsValidPRealInt(adhesionExponent):
            RaiseTypeError(where=where, argumentName='adhesionExponent', received = adhesionExponent, expectedType = ExpectedType.Real)
        if not IsValidPRealInt(restitutionCoefficient):
            RaiseTypeError(where=where, argumentName='restitutionCoefficient', received = restitutionCoefficient, expectedType = ExpectedType.Real)
        if not IsValidURealInt(minimumImpactVelocity):
            RaiseTypeError(where=where, argumentName='minimumImpactVelocity', received = minimumImpactVelocity, expectedType = ExpectedType.Real)
        if not isinstance(impactModel, exu.ContactImpactModel) and (not IsValidInt(impactModel) or impactModel < 0 or impactModel > 2):
            RaiseTypeError(where=where, argumentName='impactModel', received = impactModel, expectedType = 'expected exu.ContactImpactModel or int in range [0,2]')

        if not IsVector(dataInitialCoordinates, 4):
            RaiseTypeError(where=where, argumentName='dataInitialCoordinates', received = dataInitialCoordinates, expectedType = ExpectedType.Vector, dim=4)

        if not IsValidBool(activeConnector):
            RaiseTypeError(where=where, argumentName='activeConnector', received = activeConnector, expectedType = ExpectedType.Bool)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)

    
    CheckItemNumbers(where, itemNumbers, [_bodyNodeMarker, _bodyNodeMarker], [localPosition0, localPosition1])
    mBody0 = ItemMarker(mbs, name, 0, itemNumbers[0], localPosition0, useRigidMarker=dynamicFriction != 0)
    mBody1 = ItemMarker(mbs, name, 1, itemNumbers[1], localPosition1, useRigidMarker=dynamicFriction != 0)

    nGeneric = mbs.AddNode(eii.NodeGenericData(initialCoordinates=dataInitialCoordinates,
                                         numberOfDataCoordinates=len(dataInitialCoordinates)))
    oContact = mbs.AddObject(eii.ObjectContactSphereSphere(markerNumbers=[mBody0, mBody1],
                                                    nodeNumber=nGeneric,
                                                    spheresRadii=spheresRadii,
                                                    isHollowSphere1 = isHollowSphere1,
                                                    dynamicFriction = dynamicFriction,
                                                    frictionProportionalZone = frictionProportionalZone,
                                                    contactStiffness = contactStiffness,
                                                    contactDamping = contactDamping,
                                                    contactStiffnessExponent = contactStiffnessExponent,
                                                    constantPullOffForce = constantPullOffForce,
                                                    contactPlasticityRatio = contactPlasticityRatio,
                                                    adhesionCoefficient = adhesionCoefficient,
                                                    adhesionExponent = adhesionExponent,
                                                    restitutionCoefficient = restitutionCoefficient,
                                                    minimumImpactVelocity = minimumImpactVelocity,
                                                    impactModel = impactModel,
                                                    activeConnector = activeConnector,
                                                    visualization=eii.VObjectContactSphereSphere(show=show, color=color),
                                                    ))
    
    return oContact #nGeneric can be retrieved from oJoint easily via mbs.GetObject(oJoint)['nodeNumber']!


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateSphereQuadContact(mbs, name='', itemNumbers=[None, None], 
                                       localPosition0 = [0.,0.,0.], sphereRadius = 0,
                                       quadPoints = exudyn.Vector3DList([[0,0,0],[1,0,0],[1,1,0],[0,1,0]]),
                                       includeEdges = 15, dynamicFriction = 0., frictionProportionalZone = 1e-3,
                                       contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1,
                                       restitutionCoefficient = 1, minimumImpactVelocity = 0,
                                       impactModel = exu.ContactImpactModel.Linear,
                                       dataInitialCoordinates = [0,0,0,0],
                                       activeConnector=True,
                                       localPosition1 = [0.,0.,0.], 
                                       show=False, color=exudyn.graphics.color.default, radiusSphere=None, bodyNumbers=None, bodyOrNodeList=None) -> dict:
    """Create penalty-based sphere-quad contact between two rigid bodies, mass points or according nodes; the contact is based on two ObjectContactSphereTriangle; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        itemNumbers: the item the sphere is attached to - a body (ObjectIndex) at localPosition0, a node (NodeIndex) or a marker (MarkerIndex), for which localPosition0 is [0,0,0]; with friction, a rigid one - and the body (ObjectIndex) of the quad
        localPosition0: local position (as 3D list or numpy array) of sphere0 on body0, if not a node number
        sphereRadius: radius of sphere 0 [SI:m].
        quadPoints: 4 points as Vector3DList, list or numpy array to define the quad, defined in body1 local coordinates; note that the quad is split into two triangles with point indices [0,1,3] and [1,2,3]
        includeEdges: binary flag, where 1 defines contact with edges 0, 2 with edge 1, 4 with edge 2 and 8 with edge 3; 15 means that contact with all edges is included; edge 0 is the edge between node 0 and node 1, etc.
        dynamicFriction: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, Section Module: physics
        frictionProportionalZone: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), Section Module: physics
        contactStiffness: normal contact stiffness
        contactDamping: linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
        contactStiffnessExponent: exponent in normal contact model [SI:1]
        restitutionCoefficient: coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)
        minimumImpactVelocity: minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)
        impactModel: the impact model, exu.ContactImpactModel: Linear (only the linear contactDamping), HuntCrossley or GonthierCarvalhoMartins, the latter much more accurate regarding the coefficient of restitution in the full range (0, 1]; the integers 0, 1, 2 are accepted as well; in all models, the linear contactDamping is added, if not set to zero (#3327)
        dataInitialCoordinates: a list of four values for initialization of the data node, used for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused
        activeConnector: flag to activate or deactivate the connector
        localPosition1: local position (as 3D list or numpy array) of quad1 on body1; this is usually not needed and adds simply an offset to the quad coordinates
        show: if True, connector visualization is drawn
        color: color of connector
        radiusSphere: deprecated name of sphereRadius
        bodyNumbers: deprecated name of itemNumbers
        bodyOrNodeList: not accepted, a TypeError: the items are given in itemNumbers

    Returns:
        :dict: dictionary containing oContact0 and oContact1 with ObjectIndex of each contact object
    """
    if radiusSphere is not None: #the old name of the argument (#2814)
        DeprecatedArgument('radiusSphere', '1.12.258', 2031, use='sphereRadius', function='MainSystem.CreateSphereQuadContact')
        sphereRadius = radiusSphere
    where = 'MainSystem.CreateSphereQuadContact(...)'
    if bodyNumbers is not None: #the old name of the argument (#2863)
        DeprecatedArgument('bodyNumbers', '1.12.337', 2031, use='itemNumbers', function='MainSystem.CreateSphereQuadContact')
    itemNumbers = ItemNumbersArgument(where, itemNumbers, bodyNumbers, bodyOrNodeList)

    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsVector(localPosition0, 3):
            RaiseTypeError(where=where, argumentName='localPosition0', received = localPosition0, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(localPosition1, 3):
            RaiseTypeError(where=where, argumentName='localPosition1', received = localPosition1, expectedType = ExpectedType.Vector, dim=3)
        if not IsValidPRealInt(sphereRadius):
            RaiseTypeError(where=where, argumentName='sphereRadius', received = sphereRadius, expectedType = ExpectedType.Real)
        if (type(quadPoints) != exudyn.Vector3DList and not isinstance(quadPoints, (list,np.ndarray))) or len(quadPoints) != 4:
            RaiseTypeError(where=where, argumentName='quadPoints', received = quadPoints, expectedType = 'expected type=exudyn.Vector3DList or list with length 4, or numpy array with position vectors in rows')
        if not IsValidInt(includeEdges) or includeEdges < 0 or includeEdges > 15:
            RaiseTypeError(where=where, argumentName='includeEdges', received = includeEdges, expectedType = 'expected type=int in range[0,15]')
        if not IsValidURealInt(dynamicFriction):
            RaiseTypeError(where=where, argumentName='dynamicFriction', received = dynamicFriction, expectedType = ExpectedType.Real)
        if not IsValidURealInt(frictionProportionalZone):
            RaiseTypeError(where=where, argumentName='frictionProportionalZone', received = frictionProportionalZone, expectedType = ExpectedType.Real)

        if not IsValidURealInt(contactStiffness):
            RaiseTypeError(where=where, argumentName='contactStiffness', received = contactStiffness, expectedType = ExpectedType.Real)
        if not IsValidURealInt(contactDamping):
            RaiseTypeError(where=where, argumentName='contactDamping', received = contactDamping, expectedType = ExpectedType.Real)
        if not IsValidPRealInt(contactStiffnessExponent):
            RaiseTypeError(where=where, argumentName='contactStiffnessExponent', received = contactStiffnessExponent, expectedType = ExpectedType.Real)
        if not IsValidPRealInt(restitutionCoefficient):
            RaiseTypeError(where=where, argumentName='restitutionCoefficient', received = restitutionCoefficient, expectedType = ExpectedType.Real)
        if not IsValidURealInt(minimumImpactVelocity):
            RaiseTypeError(where=where, argumentName='minimumImpactVelocity', received = minimumImpactVelocity, expectedType = ExpectedType.Real)
        if not isinstance(impactModel, exu.ContactImpactModel) and (not IsValidInt(impactModel) or impactModel < 0 or impactModel > 2):
            RaiseTypeError(where=where, argumentName='impactModel', received = impactModel, expectedType = 'expected exu.ContactImpactModel or int in range [0,2]')

        if not IsVector(dataInitialCoordinates, 4):
            RaiseTypeError(where=where, argumentName='dataInitialCoordinates', received = dataInitialCoordinates, expectedType = ExpectedType.Vector, dim=4)

        if not IsValidBool(activeConnector):
            RaiseTypeError(where=where, argumentName='activeConnector', received = activeConnector, expectedType = ExpectedType.Bool)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)

    
    CheckItemNumbers(where, itemNumbers, [_bodyNodeMarker, ('body',)], [localPosition0, localPosition1])
    mBody0 = ItemMarker(mbs, name, 0, itemNumbers[0], localPosition0, useRigidMarker=dynamicFriction != 0) #the sphere
    mBody1 = ItemMarker(mbs, name, 1, itemNumbers[1], localPosition1, useRigidMarker=True) #the quad

    trigIndices = [[0,1,3], [1,2,3]] #this is how the quad is split into two triangles
    #compute edges flags from quad edges flags
    edges0 = (includeEdges&1) + 0*(includeEdges&2) + (includeEdges&8)//2   #braces NEEDED!!!
    edges1 = ((includeEdges&2) + (includeEdges&4) + 0*(includeEdges&8))//2 #braces NEEDED!!!
    includeEdgesList = [edges0, edges1] #for quad, would be usually [5,3] in order that all quad edges are used
    
    returnDict = {}
    for k, trig in enumerate(trigIndices):
        trianglePoints = exudyn.Vector3DList([quadPoints[trig[0]],quadPoints[trig[1]],quadPoints[trig[2]]])
        nGeneric = mbs.AddNode(eii.NodeGenericData(initialCoordinates=dataInitialCoordinates,
                                             numberOfDataCoordinates=len(dataInitialCoordinates)))
        oContact = mbs.AddObject(eii.ObjectContactSphereTriangle(markerNumbers=[mBody0, mBody1],
                                                        nodeNumber=nGeneric,
                                                        sphereRadius=sphereRadius,
                                                        trianglePoints=trianglePoints,
                                                        includeEdges=includeEdgesList[k],
                                                        dynamicFriction = dynamicFriction,
                                                        frictionProportionalZone = frictionProportionalZone,
                                                        contactStiffness = contactStiffness,
                                                        contactDamping = contactDamping,
                                                        contactStiffnessExponent = contactStiffnessExponent,
                                                        restitutionCoefficient = restitutionCoefficient,
                                                        minimumImpactVelocity = minimumImpactVelocity,
                                                        impactModel = impactModel,
                                                        activeConnector = activeConnector,
                                                        visualization=eii.VObjectContactSphereTriangle(show=show, color=color),
                                                        ))
        returnDict['oContact'+str(k)] = oContact
    
    return returnDict #nGeneric node numbers can be retrieved from oJoint easily via mbs.GetObject(oContact0)['nodeNumber']!


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateSphereTriangleContact(mbs, name='', itemNumbers=[None, None], 
                                       localPosition0 = [0.,0.,0.], sphereRadius = 0,
                                       trianglePoints = exudyn.Vector3DList([[0,0,0],[1,0,0],[0,1,0]]),
                                       includeEdges = 7, dynamicFriction = 0., frictionProportionalZone = 1e-3,
                                       contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1,
                                       restitutionCoefficient = 1, minimumImpactVelocity = 0,
                                       impactModel = exu.ContactImpactModel.Linear,
                                       dataInitialCoordinates = [0,0,0,0],
                                       activeConnector=True,
                                       localPosition1 = [0.,0.,0.], 
                                       show=False, color=exudyn.graphics.color.default, radiusSphere=None, bodyNumbers=None, bodyOrNodeList=None) -> exudyn.ObjectIndex:
    """Create penalty-based sphere-triangle contact between two rigid bodies, mass points or according nodes; the contact is based on ObjectContactSphereTriangle; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        itemNumbers: the item the sphere is attached to - a body (ObjectIndex) at localPosition0, a node (NodeIndex) or a marker (MarkerIndex), for which localPosition0 is [0,0,0]; with friction, a rigid one - and the body (ObjectIndex) of the triangle
        localPosition0: local position (as 3D list or numpy array) of sphere0 on body0, if not a node number
        sphereRadius: radius of sphere 0 [SI:m].
        trianglePoints: triangle points as Vector3DList, list or numpy array to define the quad, defined in body1 local coordinates
        includeEdges: binary flag, where 1 defines contact with edges 0, 2 with edge 1 and 4 with edge 2; 7 means that contact with all edges is included; edge 0 is the edge between node 0 and node 1, etc.
        dynamicFriction: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, Section Module: physics
        frictionProportionalZone: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), Section Module: physics
        contactStiffness: normal contact stiffness
        contactDamping: linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
        contactStiffnessExponent: exponent in normal contact model [SI:1]
        restitutionCoefficient: coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)
        minimumImpactVelocity: minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)
        impactModel: the impact model, exu.ContactImpactModel: Linear (only the linear contactDamping), HuntCrossley or GonthierCarvalhoMartins, the latter much more accurate regarding the coefficient of restitution in the full range (0, 1]; the integers 0, 1, 2 are accepted as well; in all models, the linear contactDamping is added, if not set to zero (#3327)
        dataInitialCoordinates: a list of four values for initialization of the data node, used for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused
        activeConnector: flag to activate or deactivate the connector
        localPosition1: local position (as 3D list or numpy array) of triangle1 on body1; this is usually not needed and adds simply an offset to the triangle coordinates
        show: if True, connector visualization is drawn
        color: color of connector
        radiusSphere: deprecated name of sphereRadius
        bodyNumbers: deprecated name of itemNumbers
        bodyOrNodeList: not accepted, a TypeError: the items are given in itemNumbers

    Returns:
        :ObjectIndex: returns index of created joint
    """
    if radiusSphere is not None: #the old name of the argument (#2814)
        DeprecatedArgument('radiusSphere', '1.12.258', 2031, use='sphereRadius', function='MainSystem.CreateSphereTriangleContact')
        sphereRadius = radiusSphere
    where = 'MainSystem.CreateSphereTriangleContact(...)'
    if bodyNumbers is not None: #the old name of the argument (#2863)
        DeprecatedArgument('bodyNumbers', '1.12.337', 2031, use='itemNumbers', function='MainSystem.CreateSphereTriangleContact')
    itemNumbers = ItemNumbersArgument(where, itemNumbers, bodyNumbers, bodyOrNodeList)

    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsVector(localPosition0, 3):
            RaiseTypeError(where=where, argumentName='localPosition0', received = localPosition0, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(localPosition1, 3):
            RaiseTypeError(where=where, argumentName='localPosition1', received = localPosition1, expectedType = ExpectedType.Vector, dim=3)
        if not IsValidPRealInt(sphereRadius):
            RaiseTypeError(where=where, argumentName='sphereRadius', received = sphereRadius, expectedType = ExpectedType.Real)
        if (type(trianglePoints) != exudyn.Vector3DList and not isinstance(trianglePoints, (list,np.ndarray))) or len(trianglePoints) != 3:
            RaiseTypeError(where=where, argumentName='trianglePoints', received = trianglePoints, expectedType = 'expected type=exudyn.Vector3DList or list with length 3, or numpy array with position vectors in rows')
        if not IsValidInt(includeEdges) or includeEdges < 0 or includeEdges > 7:
            RaiseTypeError(where=where, argumentName='includeEdges', received = includeEdges, expectedType = 'expected type=int in range[0,7]')
        if not IsValidURealInt(dynamicFriction):
            RaiseTypeError(where=where, argumentName='dynamicFriction', received = dynamicFriction, expectedType = ExpectedType.Real)
        if not IsValidURealInt(frictionProportionalZone):
            RaiseTypeError(where=where, argumentName='frictionProportionalZone', received = frictionProportionalZone, expectedType = ExpectedType.Real)

        if not IsValidURealInt(contactStiffness):
            RaiseTypeError(where=where, argumentName='contactStiffness', received = contactStiffness, expectedType = ExpectedType.Real)
        if not IsValidURealInt(contactDamping):
            RaiseTypeError(where=where, argumentName='contactDamping', received = contactDamping, expectedType = ExpectedType.Real)
        if not IsValidPRealInt(contactStiffnessExponent):
            RaiseTypeError(where=where, argumentName='contactStiffnessExponent', received = contactStiffnessExponent, expectedType = ExpectedType.Real)
        if not IsValidPRealInt(restitutionCoefficient):
            RaiseTypeError(where=where, argumentName='restitutionCoefficient', received = restitutionCoefficient, expectedType = ExpectedType.Real)
        if not IsValidURealInt(minimumImpactVelocity):
            RaiseTypeError(where=where, argumentName='minimumImpactVelocity', received = minimumImpactVelocity, expectedType = ExpectedType.Real)
        if not isinstance(impactModel, exu.ContactImpactModel) and (not IsValidInt(impactModel) or impactModel < 0 or impactModel > 2):
            RaiseTypeError(where=where, argumentName='impactModel', received = impactModel, expectedType = 'expected exu.ContactImpactModel or int in range [0,2]')

        if not IsVector(dataInitialCoordinates, 4):
            RaiseTypeError(where=where, argumentName='dataInitialCoordinates', received = dataInitialCoordinates, expectedType = ExpectedType.Vector, dim=4)

        if not IsValidBool(activeConnector):
            RaiseTypeError(where=where, argumentName='activeConnector', received = activeConnector, expectedType = ExpectedType.Bool)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)

    
    CheckItemNumbers(where, itemNumbers, [_bodyNodeMarker, ('body',)], [localPosition0, localPosition1])
    mBody0 = ItemMarker(mbs, name, 0, itemNumbers[0], localPosition0, useRigidMarker=True) #the sphere
    mBody1 = ItemMarker(mbs, name, 1, itemNumbers[1], localPosition1, useRigidMarker=True) #the triangle

    nGeneric = mbs.AddNode(eii.NodeGenericData(initialCoordinates=dataInitialCoordinates,
                                         numberOfDataCoordinates=len(dataInitialCoordinates)))
    oContact = mbs.AddObject(eii.ObjectContactSphereTriangle(markerNumbers=[mBody0, mBody1],
                                                    nodeNumber=nGeneric,
                                                    sphereRadius=sphereRadius,
                                                    trianglePoints=trianglePoints,
                                                    includeEdges=includeEdges,
                                                    dynamicFriction = dynamicFriction,
                                                    frictionProportionalZone = frictionProportionalZone,
                                                    contactStiffness = contactStiffness,
                                                    contactDamping = contactDamping,
                                                    contactStiffnessExponent = contactStiffnessExponent,
                                                    restitutionCoefficient = restitutionCoefficient,
                                                    minimumImpactVelocity = minimumImpactVelocity,
                                                    impactModel = impactModel,
                                                    activeConnector = activeConnector,
                                                    visualization=eii.VObjectContactSphereTriangle(show=show, color=color),
                                                    ))
    
    return oContact #nGeneric can be retrieved from oJoint easily via mbs.GetObject(oJoint)['nodeNumber']!




#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the Create functions of the contacts between primitive shapes (#3342): the markers, the data node of the size the item
#needs, apart at the start, and the item

@docmeta(public=False)
def AxisHT(localPosition, localAxis):
    """the local frame of a cylinder or a cone on its item: at localPosition, its z axis along localAxis"""
    A = np.array(ComputeOrthonormalBasis(localAxis)) #the first column along localAxis
    HT = np.eye(4)
    HT[0:3, 0:3] = np.column_stack([A[:, 1], A[:, 2], A[:, 0]])
    HT[0:3, 3] = localPosition
    return HT

@docmeta(public=False)
def ShapeMarker(mbs, name, i, item, HT):
    """the rigid marker of the shape of entry i of itemNumbers: a new one in the frame HT of a body or a node, named
    Marker<i>:name; a marker is used as it is"""
    markerName = 'Marker'+str(i)+':'+name if name != '' else ''
    kind = ItemKind(item)
    if kind == 'body':
        return mbs.AddMarker(eii.MarkerBodyRigid(name=markerName, bodyNumber=item, localHT=HT))
    if kind == 'node':
        return mbs.AddMarker(eii.MarkerNodeRigid(name=markerName, nodeNumber=item, localHT=HT))
    return item

@docmeta(public=False)
def TrianglePointsArgument(where, trianglePoints):
    """the 3 points of a triangle, given as Vector3DList, list or numpy array, as a Vector3DList; a ValueError unless 3 x 3"""
    points = np.array(trianglePoints.GetPythonObject() if isinstance(trianglePoints, exudyn.Vector3DList) else trianglePoints, dtype=float)
    if not exudyn.__useExudynFast and points.shape != (3, 3):
        RaiseTypeError(where=where, argumentName='trianglePoints', received = trianglePoints, expectedType = '3 points: a Vector3DList, a list or a numpy array of 3 x 3')
    return exudyn.Vector3DList(points.tolist())

@docmeta(public=False)
def CheckPrimitiveContact(where, name, itemNumbers, localPositions, localAxes, positive, law, show, color):
    """the ValueErrors of the arguments shared by the Create functions of the primitive contacts: the items and their local
    frames, the sizes in positive (name: value, all > 0), the contact law and the drawing"""
    CheckItemNumbers(where, itemNumbers, [_bodyNodeMarker, _bodyNodeMarker], localPositions)
    if exudyn.__useExudynFast:
        return
    if not isinstance(name, str):
        RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)
    for (i, localPosition) in enumerate(localPositions):
        if not IsVector(localPosition, 3):
            RaiseTypeError(where=where, argumentName='localPosition'+str(i), received = localPosition, expectedType = ExpectedType.Vector, dim=3)
    for (i, localAxis) in localAxes.items():
        if not IsVector(localAxis, 3) or all(x == 0 for x in localAxis):
            RaiseTypeError(where=where, argumentName='localAxis'+str(i), received = localAxis, expectedType = 'a 3D vector, not zero')
        if ItemKind(itemNumbers[i]) == 'marker' and list(localAxis) != [0, 0, 1]:
            RaiseTypeError(where=where, argumentName='localAxis'+str(i), received = localAxis, expectedType = '[0,0,1] for a MarkerIndex, whose z axis is the axis')
    for (key, value) in positive.items():
        values = value if isinstance(value, (list, tuple, np.ndarray)) else [value]
        if not all(IsValidPRealInt(x) for x in values):
            RaiseTypeError(where=where, argumentName=key, received = value, expectedType = 'positive real' + (' values' if len(values) > 1 else ''))
    for key in ['dynamicFriction', 'frictionProportionalZone', 'contactStiffness', 'contactDamping', 'minimumImpactVelocity',
                'contactPlasticityRatio', 'adhesionCoefficient', 'constantPullOffForce']:
        if key in law and not IsValidURealInt(law[key]):
            RaiseTypeError(where=where, argumentName=key, received = law[key], expectedType = ExpectedType.Real)
    for key in ['contactStiffnessExponent', 'restitutionCoefficient', 'adhesionExponent']:
        if key in law and not IsValidPRealInt(law[key]):
            RaiseTypeError(where=where, argumentName=key, received = law[key], expectedType = ExpectedType.Real)
    impactModel = law['impactModel']
    if not isinstance(impactModel, exu.ContactImpactModel) and (not IsValidInt(impactModel) or impactModel < 0 or impactModel > 2):
        RaiseTypeError(where=where, argumentName='impactModel', received = impactModel, expectedType = 'expected exu.ContactImpactModel or int in range [0,2]')
    if not IsValidBool(law['activeConnector']):
        RaiseTypeError(where=where, argumentName='activeConnector', received = law['activeConnector'], expectedType = ExpectedType.Bool)
    if not IsValidBool(show):
        RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
    if not IsVector(color, 4):
        RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)

@docmeta(public=False)
def AddPrimitiveContact(mbs, ItemType, VisuType, markers, numberOfPoints, law, show, color, shape):
    """the data node of numberOfPoints contact points - blocks of 3 coordinates, of 4 with contactPlasticityRatio > 0,
    apart at the start - and the contact item"""
    blockSize = 4 if law.get('contactPlasticityRatio', 0) != 0 else 3
    nData = mbs.AddNode(eii.NodeGenericData(numberOfDataCoordinates=numberOfPoints*blockSize,
                                            initialCoordinates=([1] + [0]*(blockSize - 1))*numberOfPoints))
    return mbs.AddObject(ItemType(markerNumbers=markers, nodeNumber=nData, visualization=VisuType(show=show, color=color),
                                  **shape, **law))

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateSphereCylinderContact(mbs, name='', itemNumbers=[None, None],
                                         localPosition0=[0.,0.,0.], localPosition1=[0.,0.,0.], localAxis1=[0.,0.,1.],
                                         sphereRadius=0., cylinderRadius=0., cylinderLength=0.,
                                         contactStiffness=0., contactDamping=0., dynamicFriction=0., frictionProportionalZone=1e-3,
                                         contactStiffnessExponent=1, restitutionCoefficient=1, minimumImpactVelocity=0,
                                         impactModel=exu.ContactImpactModel.Linear, contactForceUserFunction=0,
                                         activeConnector=True, show=False, color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """Create the penalty contact of a sphere and a finite cylinder, ObjectContactSphereCylinder, with its markers and its
    data node (#3342); for a few contacts, while GeneralContact is made for many bodies

    Args:
        mbs: the MainSystem where the markers, the data node and the contact are created
        name: name string of the contact; the markers get Marker0:name and Marker1:name
        itemNumbers: the items of the sphere and of the cylinder, each a body (ObjectIndex), a node (NodeIndex) or a marker (MarkerIndex), which is used as it is; with friction, a rigid one for the sphere
        localPosition0: the center of the sphere on item 0 [SI:m]; [0,0,0] for a node or a marker
        localPosition1: the center of the cylinder on item 1 [SI:m]; [0,0,0] for a node or a marker
        localAxis1: the axis of the cylinder on item 1, need not have unit length; [0,0,1] for a marker, whose z axis is the axis
        sphereRadius: radius of the sphere [SI:m]
        cylinderRadius: radius of the cylinder [SI:m]
        cylinderLength: length of the cylinder [SI:m], centered at localPosition1
        contactStiffness: normal contact stiffness [SI:N/m]
        contactDamping: linear normal contact damping [SI:N/(m s)]
        dynamicFriction: dynamic friction coefficient of the regularized Coulomb friction
        frictionProportionalZone: velocity [SI:m/s] up to which the friction is proportional to the sliding velocity
        contactStiffnessExponent: exponent of the penetration in the normal contact force
        restitutionCoefficient: coefficient of restitution of the impact model
        minimumImpactVelocity: lower bound of the impact velocity of the impact model [SI:m/s]
        impactModel: exu.ContactImpactModel: Linear, HuntCrossley or GonthierCarvalhoMartins; the integers 0, 1, 2 are accepted as well
        contactForceUserFunction: a Python or symbolic user function replacing the contact law, see ObjectContactSphereCylinder
        activeConnector: flag to activate or deactivate the contact
        show: if True, the contact item is drawn with the settings visualizationSettings.contact
        color: color of the drawn shapes, unless the default

    Returns:
        :ObjectIndex: returns the index of the contact item; its data node is mbs.GetObject(oContact)['nodeNumber']

    Example:
        import exudyn as exu
        from exudyn.utilities import *
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        oGround = mbs.CreateGround()
        #a ball resting on a cylinder lying along x
        oBall = mbs.CreateRigidBody(inertia=InertiaSphere(mass=1, radius=0.1), referencePosition=[0,0,0.3], gravity=[0,0,-9.81])
        mbs.CreateSphereCylinderContact(itemNumbers=[oBall, oGround], localPosition1=[0,0,0], localAxis1=[1,0,0],
                                        sphereRadius=0.1, cylinderRadius=0.2, cylinderLength=1,
                                        contactStiffness=1e5, contactDamping=500)
        mbs.Assemble()
        mbs.SolveDynamic()
    """
    where = 'MainSystem.CreateSphereCylinderContact(...)'
    law = dict(contactStiffness=contactStiffness, contactDamping=contactDamping, dynamicFriction=dynamicFriction,
               frictionProportionalZone=frictionProportionalZone, contactStiffnessExponent=contactStiffnessExponent,
               restitutionCoefficient=restitutionCoefficient, minimumImpactVelocity=minimumImpactVelocity, impactModel=impactModel,
               contactForceUserFunction=contactForceUserFunction, activeConnector=activeConnector)
    CheckPrimitiveContact(where, name, itemNumbers, [localPosition0, localPosition1], {1: localAxis1},
                          {'sphereRadius': sphereRadius, 'cylinderRadius': cylinderRadius, 'cylinderLength': cylinderLength}, law, show, color)
    markers = [ItemMarker(mbs, name, 0, itemNumbers[0], localPosition0, useRigidMarker=dynamicFriction != 0),
               ShapeMarker(mbs, name, 1, itemNumbers[1], AxisHT(localPosition1, localAxis1))]
    return AddPrimitiveContact(mbs, eii.ObjectContactSphereCylinder, eii.VObjectContactSphereCylinder, markers, 1, law, show, color,
                               dict(sphereRadius=sphereRadius, cylinderRadius=cylinderRadius, cylinderLength=cylinderLength))

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateCylinderCylinderContact(mbs, name='', itemNumbers=[None, None],
                                           localPosition0=[0.,0.,0.], localAxis0=[0.,0.,1.], localPosition1=[0.,0.,0.], localAxis1=[0.,0.,1.],
                                           cylindersRadii=[0., 0.], cylindersLengths=[0., 0.], isHollowCylinder1=False,
                                           contactStiffness=0., contactDamping=0., dynamicFriction=0., frictionProportionalZone=1e-3,
                                           contactStiffnessExponent=1, restitutionCoefficient=1, minimumImpactVelocity=0,
                                           impactModel=exu.ContactImpactModel.Linear, contactPlasticityRatio=0., adhesionCoefficient=0.,
                                           adhesionExponent=1., constantPullOffForce=0., contactForceUserFunction=0,
                                           activeConnector=True, show=False, color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """Create the penalty contact of two finite cylinders, or of a cylinder inside a hollow one,
    ObjectContactCylinderCylinder, with its markers and its data node of 4 contact points, 8 with isHollowCylinder1 (#3342)

    Args:
        mbs: the MainSystem where the markers, the data node and the contact are created
        name: name string of the contact; the markers get Marker0:name and Marker1:name
        itemNumbers: the items of cylinder 0 and cylinder 1, each a body (ObjectIndex), a rigid node (NodeIndex) or a rigid marker (MarkerIndex), which is used as it is
        localPosition0: the center of cylinder 0 on item 0 [SI:m]; [0,0,0] for a node or a marker
        localAxis0: the axis of cylinder 0 on item 0, need not have unit length; [0,0,1] for a marker, whose z axis is the axis
        localPosition1: the center of cylinder 1 on item 1 [SI:m]; [0,0,0] for a node or a marker
        localAxis1: the axis of cylinder 1 on item 1; [0,0,1] for a marker
        cylindersRadii: the radii of cylinder 0 and cylinder 1 [SI:m]; of the bore with isHollowCylinder1
        cylindersLengths: the lengths of cylinder 0 and cylinder 1 [SI:m], each centered at its local position
        isHollowCylinder1: if True, cylinder 0 moves inside cylinder 1, a hollow cylinder open at both ends
        contactStiffness: normal contact stiffness of each contact point [SI:N/m]
        contactDamping: linear normal contact damping of each contact point [SI:N/(m s)]
        dynamicFriction: dynamic friction coefficient of the regularized Coulomb friction
        frictionProportionalZone: velocity [SI:m/s] up to which the friction is proportional to the sliding velocity
        contactStiffnessExponent: exponent of the penetration in the normal contact force
        restitutionCoefficient: coefficient of restitution of the impact model
        minimumImpactVelocity: lower bound of the impact velocity of the impact model [SI:m/s]
        impactModel: exu.ContactImpactModel: Linear, HuntCrossley or GonthierCarvalhoMartins; the integers 0, 1, 2 are accepted as well
        contactPlasticityRatio: ratio of the stiffness of first loading and of unloading of the adhesive elasto-plastic model; 0: elastic
        adhesionCoefficient: coefficient of the adhesion of the adhesive elasto-plastic model [SI:N/m]
        adhesionExponent: exponent of the adhesion of the adhesive elasto-plastic model
        constantPullOffForce: constant adhesion force of each contact point of the adhesive elasto-plastic model [SI:N]
        contactForceUserFunction: a Python or symbolic user function replacing the contact law, see ObjectContactCylinderCylinder
        activeConnector: flag to activate or deactivate the contact
        show: if True, the contact item is drawn with the settings visualizationSettings.contact
        color: color of the drawn shapes, unless the default

    Returns:
        :ObjectIndex: returns the index of the contact item; its data node is mbs.GetObject(oContact)['nodeNumber']

    Example:
        import exudyn as exu
        from exudyn.utilities import *
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        oGround = mbs.CreateGround()
        #a shaft along x resting in a fixed ring
        oShaft = mbs.CreateRigidBody(inertia=InertiaCylinder(density=1000, length=0.5, outerRadius=0.05, axis=0),
                                     referencePosition=[0,0,0], gravity=[0,0,-9.81])
        mbs.CreateCylinderCylinderContact(itemNumbers=[oShaft, oGround], localAxis0=[1,0,0], localAxis1=[1,0,0],
                                          cylindersRadii=[0.05, 0.06], cylindersLengths=[0.5, 0.1], isHollowCylinder1=True,
                                          contactStiffness=1e5, contactDamping=500)
        mbs.Assemble()
        mbs.SolveDynamic()
    """
    where = 'MainSystem.CreateCylinderCylinderContact(...)'
    law = dict(contactStiffness=contactStiffness, contactDamping=contactDamping, dynamicFriction=dynamicFriction,
               frictionProportionalZone=frictionProportionalZone, contactStiffnessExponent=contactStiffnessExponent,
               restitutionCoefficient=restitutionCoefficient, minimumImpactVelocity=minimumImpactVelocity, impactModel=impactModel,
               contactPlasticityRatio=contactPlasticityRatio, adhesionCoefficient=adhesionCoefficient, adhesionExponent=adhesionExponent,
               constantPullOffForce=constantPullOffForce, contactForceUserFunction=contactForceUserFunction, activeConnector=activeConnector)
    if not exudyn.__useExudynFast and not (IsVector(cylindersRadii, 2) and IsVector(cylindersLengths, 2)):
        RaiseTypeError(where=where, argumentName='cylindersRadii, cylindersLengths', received = [cylindersRadii, cylindersLengths], expectedType = ExpectedType.Vector, dim=2)
    if not exudyn.__useExudynFast and not IsValidBool(isHollowCylinder1):
        RaiseTypeError(where=where, argumentName='isHollowCylinder1', received = isHollowCylinder1, expectedType = ExpectedType.Bool)
    CheckPrimitiveContact(where, name, itemNumbers, [localPosition0, localPosition1], {0: localAxis0, 1: localAxis1},
                          {'cylindersRadii': cylindersRadii, 'cylindersLengths': cylindersLengths}, law, show, color)
    markers = [ShapeMarker(mbs, name, 0, itemNumbers[0], AxisHT(localPosition0, localAxis0)),
               ShapeMarker(mbs, name, 1, itemNumbers[1], AxisHT(localPosition1, localAxis1))]
    return AddPrimitiveContact(mbs, eii.ObjectContactCylinderCylinder, eii.VObjectContactCylinderCylinder, markers,
                               8 if isHollowCylinder1 else 4, law, show, color,
                               dict(cylindersRadii=cylindersRadii, cylindersLengths=cylindersLengths, isHollowCylinder1=isHollowCylinder1))

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateCylinderTriangleContact(mbs, name='', itemNumbers=[None, None],
                                           localPosition0=[0.,0.,0.], localAxis0=[0.,0.,1.], cylinderRadius=0., cylinderLength=0.,
                                           trianglePoints=[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]], infinitePlane=False, localPosition1=[0.,0.,0.],
                                           contactStiffness=0., contactDamping=0., dynamicFriction=0., frictionProportionalZone=1e-3,
                                           contactStiffnessExponent=1, restitutionCoefficient=1, minimumImpactVelocity=0,
                                           impactModel=exu.ContactImpactModel.Linear, contactPlasticityRatio=0., adhesionCoefficient=0.,
                                           adhesionExponent=1., constantPullOffForce=0., contactForceUserFunction=0,
                                           activeConnector=True, show=False, color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """Create the penalty contact of a finite cylinder and a triangle or its plane, ObjectContactCylinderTriangle, with its
    markers and its data node of 4 contact points (#3342)

    Args:
        mbs: the MainSystem where the markers, the data node and the contact are created
        name: name string of the contact; the markers get Marker0:name and Marker1:name
        itemNumbers: the items of the cylinder and of the triangle, each a body (ObjectIndex), a rigid node (NodeIndex) or a rigid marker (MarkerIndex), which is used as it is
        localPosition0: the center of the cylinder on item 0 [SI:m]; [0,0,0] for a node or a marker
        localAxis0: the axis of the cylinder on item 0, need not have unit length; [0,0,1] for a marker, whose z axis is the axis
        cylinderRadius: radius of the cylinder [SI:m]
        cylinderLength: length of the cylinder [SI:m], centered at localPosition0
        trianglePoints: the 3 points of the triangle on item 1 [SI:m], relative to localPosition1, as list, numpy array or Vector3DList; the cylinder is on the side of the normal by the order of the points
        infinitePlane: if True, the contact is with the plane of the triangle, of any size
        localPosition1: the position on item 1 which the triangle points are relative to [SI:m]; [0,0,0] for a node or a marker
        contactStiffness: normal contact stiffness of each contact point [SI:N/m]
        contactDamping: linear normal contact damping of each contact point [SI:N/(m s)]
        dynamicFriction: dynamic friction coefficient of the regularized Coulomb friction
        frictionProportionalZone: velocity [SI:m/s] up to which the friction is proportional to the sliding velocity
        contactStiffnessExponent: exponent of the penetration in the normal contact force
        restitutionCoefficient: coefficient of restitution of the impact model
        minimumImpactVelocity: lower bound of the impact velocity of the impact model [SI:m/s]
        impactModel: exu.ContactImpactModel: Linear, HuntCrossley or GonthierCarvalhoMartins; the integers 0, 1, 2 are accepted as well
        contactPlasticityRatio: ratio of the stiffness of first loading and of unloading of the adhesive elasto-plastic model; 0: elastic
        adhesionCoefficient: coefficient of the adhesion of the adhesive elasto-plastic model [SI:N/m]
        adhesionExponent: exponent of the adhesion of the adhesive elasto-plastic model
        constantPullOffForce: constant adhesion force of each contact point of the adhesive elasto-plastic model [SI:N]
        contactForceUserFunction: a Python or symbolic user function replacing the contact law, see ObjectContactCylinderTriangle
        activeConnector: flag to activate or deactivate the contact
        show: if True, the contact item is drawn with the settings visualizationSettings.contact
        color: color of the drawn shapes, unless the default

    Returns:
        :ObjectIndex: returns the index of the contact item; its data node is mbs.GetObject(oContact)['nodeNumber']

    Example:
        import exudyn as exu
        from exudyn.utilities import *
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        oGround = mbs.CreateGround()
        #a roller along y dropped on the floor, the plane z = 0
        oRoller = mbs.CreateRigidBody(inertia=InertiaCylinder(density=1000, length=0.2, outerRadius=0.1, axis=1),
                                      referencePosition=[0,0,0.12], gravity=[0,0,-9.81])
        mbs.CreateCylinderTriangleContact(itemNumbers=[oRoller, oGround], localAxis0=[0,1,0], cylinderRadius=0.1, cylinderLength=0.2,
                                          trianglePoints=[[0,0,0],[1,0,0],[0,1,0]], infinitePlane=True,
                                          contactStiffness=1e5, contactDamping=500)
        mbs.Assemble()
        mbs.SolveDynamic()
    """
    where = 'MainSystem.CreateCylinderTriangleContact(...)'
    law = dict(contactStiffness=contactStiffness, contactDamping=contactDamping, dynamicFriction=dynamicFriction,
               frictionProportionalZone=frictionProportionalZone, contactStiffnessExponent=contactStiffnessExponent,
               restitutionCoefficient=restitutionCoefficient, minimumImpactVelocity=minimumImpactVelocity, impactModel=impactModel,
               contactPlasticityRatio=contactPlasticityRatio, adhesionCoefficient=adhesionCoefficient, adhesionExponent=adhesionExponent,
               constantPullOffForce=constantPullOffForce, contactForceUserFunction=contactForceUserFunction, activeConnector=activeConnector)
    points = TrianglePointsArgument(where, trianglePoints)
    if not exudyn.__useExudynFast and not IsValidBool(infinitePlane):
        RaiseTypeError(where=where, argumentName='infinitePlane', received = infinitePlane, expectedType = ExpectedType.Bool)
    CheckPrimitiveContact(where, name, itemNumbers, [localPosition0, localPosition1], {0: localAxis0},
                          {'cylinderRadius': cylinderRadius, 'cylinderLength': cylinderLength}, law, show, color)
    markers = [ShapeMarker(mbs, name, 0, itemNumbers[0], AxisHT(localPosition0, localAxis0)),
               ShapeMarker(mbs, name, 1, itemNumbers[1], AxisHT(localPosition1, [0, 0, 1]))]
    return AddPrimitiveContact(mbs, eii.ObjectContactCylinderTriangle, eii.VObjectContactCylinderTriangle, markers, 4, law, show, color,
                               dict(cylinderRadius=cylinderRadius, cylinderLength=cylinderLength, trianglePoints=points, infinitePlane=infinitePlane))

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateConeConeContact(mbs, name='', itemNumbers=[None, None],
                                   localPosition0=[0.,0.,0.], localAxis0=[0.,0.,1.], localPosition1=[0.,0.,0.], localAxis1=[0.,0.,1.],
                                   conesRadii0=[0., 0.], conesRadii1=[0., 0.], conesLengths=[0., 0.], isHollowCone1=False,
                                   contactStiffness=0., contactDamping=0., dynamicFriction=0., frictionProportionalZone=1e-3,
                                   contactStiffnessExponent=1, restitutionCoefficient=1, minimumImpactVelocity=0,
                                   impactModel=exu.ContactImpactModel.Linear, contactPlasticityRatio=0., adhesionCoefficient=0.,
                                   adhesionExponent=1., constantPullOffForce=0., contactForceUserFunction=0,
                                   activeConnector=True, show=False, color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """Create the penalty contact of two truncated cones, or of a cone inside a hollow one, ObjectContactConeCone, with its
    markers and its data node of 4 contact points, 8 with isHollowCone1 (#3342)

    Args:
        mbs: the MainSystem where the markers, the data node and the contact are created
        name: name string of the contact; the markers get Marker0:name and Marker1:name
        itemNumbers: the items of cone 0 and cone 1, each a body (ObjectIndex), a rigid node (NodeIndex) or a rigid marker (MarkerIndex), which is used as it is
        localPosition0: the center of cone 0 on item 0 [SI:m]; [0,0,0] for a node or a marker
        localAxis0: the axis of cone 0 on item 0, from its first to its second radius, need not have unit length; [0,0,1] for a marker, whose z axis is the axis
        localPosition1: the center of cone 1 on item 1 [SI:m]; [0,0,0] for a node or a marker
        localAxis1: the axis of cone 1 on item 1; [0,0,1] for a marker
        conesRadii0: the radii of cone 0 at the start and at the end of its axis [SI:m]; one may be 0, a pointed cone
        conesRadii1: the radii of cone 1 at the start and at the end of its axis [SI:m]; of the bore with isHollowCone1
        conesLengths: the lengths of cone 0 and cone 1 [SI:m], each centered at its local position
        isHollowCone1: if True, cone 0 moves inside cone 1, a hollow cone open at both ends
        contactStiffness: normal contact stiffness of each contact point [SI:N/m]
        contactDamping: linear normal contact damping of each contact point [SI:N/(m s)]
        dynamicFriction: dynamic friction coefficient of the regularized Coulomb friction
        frictionProportionalZone: velocity [SI:m/s] up to which the friction is proportional to the sliding velocity
        contactStiffnessExponent: exponent of the penetration in the normal contact force
        restitutionCoefficient: coefficient of restitution of the impact model
        minimumImpactVelocity: lower bound of the impact velocity of the impact model [SI:m/s]
        impactModel: exu.ContactImpactModel: Linear, HuntCrossley or GonthierCarvalhoMartins; the integers 0, 1, 2 are accepted as well
        contactPlasticityRatio: ratio of the stiffness of first loading and of unloading of the adhesive elasto-plastic model; 0: elastic
        adhesionCoefficient: coefficient of the adhesion of the adhesive elasto-plastic model [SI:N/m]
        adhesionExponent: exponent of the adhesion of the adhesive elasto-plastic model
        constantPullOffForce: constant adhesion force of each contact point of the adhesive elasto-plastic model [SI:N]
        contactForceUserFunction: a Python or symbolic user function replacing the contact law, see ObjectContactConeCone
        activeConnector: flag to activate or deactivate the contact
        show: if True, the contact item is drawn with the settings visualizationSettings.contact
        color: color of the drawn shapes, unless the default

    Returns:
        :ObjectIndex: returns the index of the contact item; its data node is mbs.GetObject(oContact)['nodeNumber']

    Example:
        import exudyn as exu
        from exudyn.utilities import *
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        oGround = mbs.CreateGround()
        #a small cone standing on the top of a fixed cone
        oCone = mbs.CreateRigidBody(inertia=InertiaCylinder(density=1000, length=0.1, outerRadius=0.04, axis=2),
                                    referencePosition=[0,0,0.16], gravity=[0,0,-9.81])
        mbs.CreateConeConeContact(itemNumbers=[oCone, oGround], conesRadii0=[0.05, 0.02], conesRadii1=[0.2, 0.1],
                                  conesLengths=[0.1, 0.2], contactStiffness=1e5, contactDamping=500)
        mbs.Assemble()
        mbs.SolveDynamic()
    """
    where = 'MainSystem.CreateConeConeContact(...)'
    law = dict(contactStiffness=contactStiffness, contactDamping=contactDamping, dynamicFriction=dynamicFriction,
               frictionProportionalZone=frictionProportionalZone, contactStiffnessExponent=contactStiffnessExponent,
               restitutionCoefficient=restitutionCoefficient, minimumImpactVelocity=minimumImpactVelocity, impactModel=impactModel,
               contactPlasticityRatio=contactPlasticityRatio, adhesionCoefficient=adhesionCoefficient, adhesionExponent=adhesionExponent,
               constantPullOffForce=constantPullOffForce, contactForceUserFunction=contactForceUserFunction, activeConnector=activeConnector)
    if not exudyn.__useExudynFast:
        for (key, value) in [('conesRadii0', conesRadii0), ('conesRadii1', conesRadii1), ('conesLengths', conesLengths)]:
            if not IsVector(value, 2) or (key != 'conesLengths' and (min(value) < 0 or max(value) <= 0)):
                RaiseTypeError(where=where, argumentName=key, received = value, expectedType = 'a vector of 2 values, not negative, one of them positive' if key != 'conesLengths' else ExpectedType.Vector, dim=2)
        if not IsValidBool(isHollowCone1):
            RaiseTypeError(where=where, argumentName='isHollowCone1', received = isHollowCone1, expectedType = ExpectedType.Bool)
    CheckPrimitiveContact(where, name, itemNumbers, [localPosition0, localPosition1], {0: localAxis0, 1: localAxis1},
                          {'conesLengths': conesLengths}, law, show, color)
    markers = [ShapeMarker(mbs, name, 0, itemNumbers[0], AxisHT(localPosition0, localAxis0)),
               ShapeMarker(mbs, name, 1, itemNumbers[1], AxisHT(localPosition1, localAxis1))]
    return AddPrimitiveContact(mbs, eii.ObjectContactConeCone, eii.VObjectContactConeCone, markers, 8 if isHollowCone1 else 4, law, show, color,
                               dict(conesRadii0=conesRadii0, conesRadii1=conesRadii1, conesLengths=conesLengths, isHollowCone1=isHollowCone1))

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateConeTriangleContact(mbs, name='', itemNumbers=[None, None],
                                       localPosition0=[0.,0.,0.], localAxis0=[0.,0.,1.], coneRadii=[0., 0.], coneLength=0.,
                                       trianglePoints=[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]], infinitePlane=False, localPosition1=[0.,0.,0.],
                                       contactStiffness=0., contactDamping=0., dynamicFriction=0., frictionProportionalZone=1e-3,
                                       contactStiffnessExponent=1, restitutionCoefficient=1, minimumImpactVelocity=0,
                                       impactModel=exu.ContactImpactModel.Linear, contactPlasticityRatio=0., adhesionCoefficient=0.,
                                       adhesionExponent=1., constantPullOffForce=0., contactForceUserFunction=0,
                                       activeConnector=True, show=False, color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """Create the penalty contact of a truncated cone and a triangle or its plane, ObjectContactConeTriangle, with its markers
    and its data node of 4 contact points (#3342)

    Args:
        mbs: the MainSystem where the markers, the data node and the contact are created
        name: name string of the contact; the markers get Marker0:name and Marker1:name
        itemNumbers: the items of the cone and of the triangle, each a body (ObjectIndex), a rigid node (NodeIndex) or a rigid marker (MarkerIndex), which is used as it is
        localPosition0: the center of the cone on item 0 [SI:m]; [0,0,0] for a node or a marker
        localAxis0: the axis of the cone on item 0, from its first to its second radius, need not have unit length; [0,0,1] for a marker, whose z axis is the axis
        coneRadii: the radii of the cone at the start and at the end of its axis [SI:m]; one may be 0, a pointed cone
        coneLength: length of the cone [SI:m], centered at localPosition0
        trianglePoints: the 3 points of the triangle on item 1 [SI:m], relative to localPosition1, as list, numpy array or Vector3DList; the cone is on the side of the normal by the order of the points
        infinitePlane: if True, the contact is with the plane of the triangle, of any size
        localPosition1: the position on item 1 which the triangle points are relative to [SI:m]; [0,0,0] for a node or a marker
        contactStiffness: normal contact stiffness of each contact point [SI:N/m]
        contactDamping: linear normal contact damping of each contact point [SI:N/(m s)]
        dynamicFriction: dynamic friction coefficient of the regularized Coulomb friction
        frictionProportionalZone: velocity [SI:m/s] up to which the friction is proportional to the sliding velocity
        contactStiffnessExponent: exponent of the penetration in the normal contact force
        restitutionCoefficient: coefficient of restitution of the impact model
        minimumImpactVelocity: lower bound of the impact velocity of the impact model [SI:m/s]
        impactModel: exu.ContactImpactModel: Linear, HuntCrossley or GonthierCarvalhoMartins; the integers 0, 1, 2 are accepted as well
        contactPlasticityRatio: ratio of the stiffness of first loading and of unloading of the adhesive elasto-plastic model; 0: elastic
        adhesionCoefficient: coefficient of the adhesion of the adhesive elasto-plastic model [SI:N/m]
        adhesionExponent: exponent of the adhesion of the adhesive elasto-plastic model
        constantPullOffForce: constant adhesion force of each contact point of the adhesive elasto-plastic model [SI:N]
        contactForceUserFunction: a Python or symbolic user function replacing the contact law, see ObjectContactConeTriangle
        activeConnector: flag to activate or deactivate the contact
        show: if True, the contact item is drawn with the settings visualizationSettings.contact
        color: color of the drawn shapes, unless the default

    Returns:
        :ObjectIndex: returns the index of the contact item; its data node is mbs.GetObject(oContact)['nodeNumber']

    Example:
        import exudyn as exu
        from exudyn.utilities import *
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        oGround = mbs.CreateGround()
        #a cone standing on its base on the floor, the plane z = 0
        oCone = mbs.CreateRigidBody(inertia=InertiaCylinder(density=1000, length=0.2, outerRadius=0.07, axis=2),
                                    referencePosition=[0,0,0.11], gravity=[0,0,-9.81])
        mbs.CreateConeTriangleContact(itemNumbers=[oCone, oGround], coneRadii=[0.1, 0.03], coneLength=0.2,
                                      trianglePoints=[[0,0,0],[1,0,0],[0,1,0]], infinitePlane=True,
                                      contactStiffness=1e5, contactDamping=500)
        mbs.Assemble()
        mbs.SolveDynamic()
    """
    where = 'MainSystem.CreateConeTriangleContact(...)'
    law = dict(contactStiffness=contactStiffness, contactDamping=contactDamping, dynamicFriction=dynamicFriction,
               frictionProportionalZone=frictionProportionalZone, contactStiffnessExponent=contactStiffnessExponent,
               restitutionCoefficient=restitutionCoefficient, minimumImpactVelocity=minimumImpactVelocity, impactModel=impactModel,
               contactPlasticityRatio=contactPlasticityRatio, adhesionCoefficient=adhesionCoefficient, adhesionExponent=adhesionExponent,
               constantPullOffForce=constantPullOffForce, contactForceUserFunction=contactForceUserFunction, activeConnector=activeConnector)
    points = TrianglePointsArgument(where, trianglePoints)
    if not exudyn.__useExudynFast:
        if not IsVector(coneRadii, 2) or min(coneRadii) < 0 or max(coneRadii) <= 0:
            RaiseTypeError(where=where, argumentName='coneRadii', received = coneRadii, expectedType = 'a vector of 2 values, not negative, one of them positive')
        if not IsValidBool(infinitePlane):
            RaiseTypeError(where=where, argumentName='infinitePlane', received = infinitePlane, expectedType = ExpectedType.Bool)
    CheckPrimitiveContact(where, name, itemNumbers, [localPosition0, localPosition1], {0: localAxis0},
                          {'coneLength': coneLength}, law, show, color)
    markers = [ShapeMarker(mbs, name, 0, itemNumbers[0], AxisHT(localPosition0, localAxis0)),
               ShapeMarker(mbs, name, 1, itemNumbers[1], AxisHT(localPosition1, [0, 0, 1]))]
    return AddPrimitiveContact(mbs, eii.ObjectContactConeTriangle, eii.VObjectContactConeTriangle, markers, 4, law, show, color,
                               dict(coneRadii=coneRadii, coneLength=coneLength, trianglePoints=points, infinitePlane=infinitePlane))

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateConvexConvexContact(mbs, name='', itemNumbers=[None, None],
                                       points0=None, triangles0=None, points1=None, triangles1=None,
                                       localPosition0=[0.,0.,0.], localPosition1=[0.,0.,0.],
                                       contactStiffness=0., contactDamping=0., dynamicFriction=0., frictionProportionalZone=1e-3,
                                       contactStiffnessExponent=1, restitutionCoefficient=1, minimumImpactVelocity=0,
                                       impactModel=exu.ContactImpactModel.Linear, contactPlasticityRatio=0., adhesionCoefficient=0.,
                                       adhesionExponent=1., constantPullOffForce=0., contactForceUserFunction=0,
                                       activeConnector=True, show=False, color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """Create the penalty contact of two convex polytopes, ObjectContactConvexConvex, with its markers and its data node of 8
    contact points (#3342); the points and triangles may come from graphics.ToPointsAndTrigs of the graphics of a body

    Args:
        mbs: the MainSystem where the markers, the data node and the contact are created
        name: name string of the contact; the markers get Marker0:name and Marker1:name
        itemNumbers: the items of polytope 0 and polytope 1, each a body (ObjectIndex), a rigid node (NodeIndex) or a rigid marker (MarkerIndex), which is used as it is
        points0: the points of polytope 0 relative to localPosition0 on item 0 [SI:m], a list or a numpy array of n x 3; their convex hull is the shape
        triangles0: the triangles of the surface of polytope 0, with outward normals, a list or a numpy array of m x 3 indices of points0
        points1: the points of polytope 1 relative to localPosition1 on item 1 [SI:m]
        triangles1: the triangles of the surface of polytope 1, with outward normals
        localPosition0: the position on item 0 which points0 are relative to [SI:m]; [0,0,0] for a node or a marker
        localPosition1: the position on item 1 which points1 are relative to [SI:m]; [0,0,0] for a node or a marker
        contactStiffness: normal contact stiffness of each contact point [SI:N/m]
        contactDamping: linear normal contact damping of each contact point [SI:N/(m s)]
        dynamicFriction: dynamic friction coefficient of the regularized Coulomb friction
        frictionProportionalZone: velocity [SI:m/s] up to which the friction is proportional to the sliding velocity
        contactStiffnessExponent: exponent of the penetration in the normal contact force
        restitutionCoefficient: coefficient of restitution of the impact model
        minimumImpactVelocity: lower bound of the impact velocity of the impact model [SI:m/s]
        impactModel: exu.ContactImpactModel: Linear, HuntCrossley or GonthierCarvalhoMartins; the integers 0, 1, 2 are accepted as well
        contactPlasticityRatio: ratio of the stiffness of first loading and of unloading of the adhesive elasto-plastic model; 0: elastic
        adhesionCoefficient: coefficient of the adhesion of the adhesive elasto-plastic model [SI:N/m]
        adhesionExponent: exponent of the adhesion of the adhesive elasto-plastic model
        constantPullOffForce: constant adhesion force of each contact point of the adhesive elasto-plastic model [SI:N]
        contactForceUserFunction: a Python or symbolic user function replacing the contact law, see ObjectContactConvexConvex
        activeConnector: flag to activate or deactivate the contact
        show: if True, the contact item is drawn with the settings visualizationSettings.contact
        color: color of the drawn shapes, unless the default

    Returns:
        :ObjectIndex: returns the index of the contact item; its data node is mbs.GetObject(oContact)['nodeNumber']

    Example:
        import exudyn as exu
        from exudyn.utilities import *
        import exudyn.graphics as graphics
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        oGround = mbs.CreateGround()
        #a cube dropped on a fixed slab, both from the points and triangles of their graphics
        cube = graphics.Brick(size=[0.2]*3)
        oCube = mbs.CreateRigidBody(inertia=InertiaCuboid(density=1000, sideLengths=[0.2]*3), referencePosition=[0,0,0.12],
                                    gravity=[0,0,-9.81], graphicsDataList=[cube])
        (points0, triangles0) = graphics.ToPointsAndTrigs(cube)
        (points1, triangles1) = graphics.ToPointsAndTrigs(graphics.Brick(centerPoint=[0,0,-0.05], size=[1,1,0.1]))
        mbs.CreateConvexConvexContact(itemNumbers=[oCube, oGround], points0=points0, triangles0=triangles0,
                                      points1=points1, triangles1=triangles1, contactStiffness=1e5, contactDamping=500)
        mbs.Assemble()
        mbs.SolveDynamic()
    """
    where = 'MainSystem.CreateConvexConvexContact(...)'
    law = dict(contactStiffness=contactStiffness, contactDamping=contactDamping, dynamicFriction=dynamicFriction,
               frictionProportionalZone=frictionProportionalZone, contactStiffnessExponent=contactStiffnessExponent,
               restitutionCoefficient=restitutionCoefficient, minimumImpactVelocity=minimumImpactVelocity, impactModel=impactModel,
               contactPlasticityRatio=contactPlasticityRatio, adhesionCoefficient=adhesionCoefficient, adhesionExponent=adhesionExponent,
               constantPullOffForce=constantPullOffForce, contactForceUserFunction=contactForceUserFunction, activeConnector=activeConnector)
    shape = {}
    for (key, value, columns) in [('points0', points0, 'real'), ('triangles0', triangles0, 'int'), ('points1', points1, 'real'), ('triangles1', triangles1, 'int')]:
        array = np.array(value) if value is not None else np.zeros((0, 3))
        if not exudyn.__useExudynFast and (array.ndim != 2 or array.shape[1] != 3 or array.shape[0] < (4 if columns == 'real' else 4)
                                           or (columns == 'int' and not np.issubdtype(array.dtype, np.integer))):
            RaiseTypeError(where=where, argumentName=key, received = value, expectedType = 'a list or numpy array of n x 3 ' + ('points, n >= 4' if columns == 'real' else 'integer indices, n >= 4'))
        shape[key] = array if columns == 'int' else array + np.array(localPosition0 if key == 'points0' else localPosition1)
    CheckPrimitiveContact(where, name, itemNumbers, [localPosition0, localPosition1], {}, {}, law, show, color)
    markers = [ShapeMarker(mbs, name, 0, itemNumbers[0], np.eye(4)), ShapeMarker(mbs, name, 1, itemNumbers[1], np.eye(4))]
    return AddPrimitiveContact(mbs, eii.ObjectContactConvexConvex, eii.VObjectContactConvexConvex, markers, 8, law, show, color, shape)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

@extends(exudyn.MainSystem)
def MainSystemCreateKinematicTree(mbs,
                           name = '',
                           listOfTreeLinks = [],
                           referenceCoordinates = None,
                           initialCoordinates = None,
                           initialCoordinates_t = None,
                           gravity = [0.,0.,0.],
                           baseOffset = [0.,0.,0.],
                           linkForces  = None,
                           linkTorques  = None,
                           jointForceVector = None,
                           jointPositionOffsetVector = None,
                           jointVelocityOffsetVector  = None,
                           forceUserFunction = 0,
                           jointRadius = 0.05,
                           jointWidth = 0.12,
                           colors = exudyn.graphics.color.default,
                           colorsJoints = exudyn.graphics.color.default,
                           baseGraphicsDataList = None,
                           linkRoundness = 0.2,
                           show = True, 
                           ) -> exudyn.ObjectIndex: 
    """helper function to create 2D or 3D mass point object and node, using arguments as in NodePoint and MassPoint; uses TreeLink as defined in exudyn.rigidBodyUtilities

    Args:
        mbs: the MainSystem where items are created
        name: name string for object, node is 'Node:'+name
        listOfTreeLinks: list of TreeLink (from exudyn.rigidBodyUtilities) which characterize the KinematicTree
        referenceCoordinates: reference coordinates all kinematic tree coordinates (e.g., joint angles); i.e., configuration where displacements are zero
        initialCoordinates: initial deviation from reference coordinates (= displacements)
        initialCoordinates_t: initial velocities (e.g., of joint angles)
        gravity: gravity vevtor applied to kinematic tree (always a 3D vector, no matter if 2D or 3D mass)
        baseOffset: constant 3D vector representing the origin of the kinematic tree
        linkForces: Vector3DList of forces per link (at joint origin) or None
        linkTorques: Vector3DList of torques per link or None
        jointForceVector: a list or numpy array of scalar forces per joint, representing joint forces (prismatic joint) or joint torques (revolute joint)
        jointPositionOffsetVector: a list or numpy array of scalar set coordinates per joint; use PreStepUserFunction to change values over time
        jointVelocityOffsetVector: a list or numpy array of scalar set velocities per joint; use PreStepUserFunction to change values over time
        forceUserFunction: A Python user function which computes the generalized force vector on RHS with identical action as jointForceVector; for description see ObjectKinematicTree
        show: show kinematic tree
        showLinks: set true, if links shall be shown; if graphicsDataList is empty, a standard drawing for links is used (drawing a cylinder from previous joint or base to next joint; size relative to frame size in KinematicTree visualization settings); else graphicsDataList are used per link; NOTE visualization of joint and COM frames can be modified via visualizationSettings.bodies.kinematicTree
        showJoints: set true, if joints shall be shown; if graphicsDataList is empty, a standard drawing for joints is used (drawing a cylinder for revolute joints; size relative to frame size in KinematicTree visualization settings)
        jointRadius: for generic visualization of joints and links
        jointWidth: for generic visualization of joints and links
        colors: either one general color for kinematic tree, or list with one color per link
        colorsJoints: either one color for all joints or list with one color per joint
        baseGraphicsDataList: graphics for base; if None, it is computed automatically; otherwise a list of graphicsData or empty list
        linkRoundness: for automatic generation of graphics for links, roundness=0 give brick-shape, roundness<1 give transition of brick to ellipsoid and roundness=1 give cylinders
        show: show kinematic tree

    Returns:
        :ObjectIndex: returns kinematic tree object index
    """
    nLinks = len(listOfTreeLinks)

    #error checks:        
    if not exudyn.__useExudynFast:
        where='MainSystem.CreateKinematicTree(...)'
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)
        if not IsVector(baseOffset, 3):
            RaiseTypeError(where=where, argumentName='baseOffset', received = baseOffset, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(gravity, 3):
            RaiseTypeError(where=where, argumentName='gravity', received = gravity, expectedType = ExpectedType.Vector, dim=3)
    
        if not IsValidRealInt(jointRadius):
            RaiseTypeError(where=where, argumentName='jointRadius', received = jointRadius, expectedType = ExpectedType.Real)
        if not IsValidRealInt(jointWidth):
            RaiseTypeError(where=where, argumentName='jointWidth', received = jointWidth, expectedType = ExpectedType.Real)
        if not IsValidRealInt(linkRoundness):
            RaiseTypeError(where=where, argumentName='linkRoundness', received = linkRoundness, expectedType = ExpectedType.Real)

        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)

    
    nodeName = ''
    if name != '':
        nodeName = 'Node:'+name

    nLinks = len(listOfTreeLinks)

    def CheckAndGetDefault(var, argName, default):
        if var is None:
            return default
        elif not IsVector(var,nLinks):
            addStr = ' but received: '+str(var)
            if IsVector(var): 
                addStr = ' but received length '+str(len(var))
            raise ValueError(where+': arg "'+argName+'" is expected to be either None or a list / numpy array with length '+str(nLinks)+' (length of listOfTreeLinks)'+addStr)
        return var

    referenceCoordinates = CheckAndGetDefault(referenceCoordinates, 'referenceCoordinates', np.zeros(nLinks))
    initialCoordinates = CheckAndGetDefault(initialCoordinates, 'initialCoordinates', np.zeros(nLinks))
    initialCoordinates_t = CheckAndGetDefault(initialCoordinates_t, 'initialCoordinates_t', np.zeros(nLinks))

    jointForceVector = CheckAndGetDefault(jointForceVector, 'jointForceVector', [])
    jointPositionOffsetVector = CheckAndGetDefault(jointPositionOffsetVector, 'jointPositionOffsetVector', [])
    jointVelocityOffsetVector = CheckAndGetDefault(jointVelocityOffsetVector, 'jointVelocityOffsetVector', [])

    linkMasses = []
    linkCOMs = exu.Vector3DList()
    linkInertiasCOM=exu.Matrix3DList()
    
    jointTypes = []
    jointHTs = [] #the joint transformations and offsets, one exu.HT per link (#2824)

    graphicsDataList = []
    autoComputeBaseGraphics = True if baseGraphicsDataList is None else False
    baseGraphicsDataList0 = [] if baseGraphicsDataList is None else baseGraphicsDataList
    
    jointPControlVector = []
    jointDControlVector = []

    linkParents = []
    
    linkColors = colors
    if type(colors) is not list:
        raise ValueError(where+': arg "colors" is expected to be either single RGBA color (list) or list of RGBA colors (list of lists)')
    if type(colorsJoints) is not list:
        raise ValueError(where+': arg "colorsJoints" is expected to be either single RGBA color (list) or list of RGBA colors (list of lists)')

    if type(colors[0]) is not list:
        if len(colors)!=4:
            raise ValueError(where+': arg "colors" must be a list of 4 RGBA components or a list of RGBA colors (list of lists)')
        color0 = colors if colors[0] != -1 else exudyn.graphics.color.defaultBody
        linkColors = [color0]*nLinks
    else:
        if len(colors) != nLinks:
            raise ValueError(where+': arg "colors" must be a list of 4 RGBA components or a list of RGBA colors with '+str(nLinks)+' colors')
        for color in colors:
            if len(color)!=4:
                raise ValueError(where+': arg "colors" must be a list of 4 RGBA components or a list of RGBA colors (list of lists with 4 components), but received color: '+str(color))
            
    jointColors = colorsJoints
    if type(colorsJoints[0]) is not list:
        if len(colorsJoints)!=4:
            raise ValueError(where+': arg "colorsJoints" must be a list of 4 RGBA components or a list of RGBA colors (list of lists)')
        color0 = colorsJoints if jointColors[0] != -1 else exudyn.graphics.color.defaultJoint
        jointColors = [color0]*nLinks
    else:
        if len(jointColors) != nLinks:
            raise ValueError(where+': arg "jointColors" must be a list of 4 RGBA components or a list of RGBA colors with '+str(nLinks)+' colors')
        for color in jointColors:
            if len(color)!=4:
                raise ValueError(where+': arg "jointColors" must be a list of 4 RGBA components or a list of RGBA colors (list of lists with 4 components), but received color: '+str(color))
    
    parentsNoneType = False
    parentsNumberType = False
    hasPDcontrol = False
    leaveLinks = [True]*nLinks #contains True if is leave
    for i in range(nLinks):

        link = listOfTreeLinks[i]
        if link.parent is None:
            parentsNoneType = True
            linkParents.append(i-1)
        else:
            parentsNumberType = True
            if link.parent >= i:
                raise ValueError(where+': TreeLink parents must always have smaller index than current link')
            linkParents.append(link.parent)
        if linkParents[-1] != -1:
            leaveLinks[linkParents[-1]] = False

        graphicsDataList.append([]) #add empty list that is filled lateron

        jointTypes.append(link.jointType)
        linkMasses.append(link.linkInertia.Mass())
        linkCOMs.Append(link.linkInertia.COM())
        linkInertiasCOM.Append(link.linkInertia.InertiaCOM())
        jointHTs.append(exu.HT(link.jointHT))
    
        if link.PDcontrol is not None:
            hasPDcontrol = True
            jointPControlVector.append(link.PDcontrol[0])
            jointDControlVector.append(link.PDcontrol[1])
        else:
            jointPControlVector.append(0)
            jointDControlVector.append(0)
        
    for i in range(nLinks):
        link = listOfTreeLinks[i]
        #add graphics or create accoring graphics
        if link.graphicsDataList is not None:
            for graphicsData in link.graphicsDataList:
                graphicsDataList[i].append(graphicsData)

        axis = JointTypeToAxis(link.jointType)

        if leaveLinks[i] and link.graphicsDataList is None: #if leave link without graphics, add automatically
            vAxis = np.array([jointWidth,0,0]) if axis[0] == 0 else np.array([0,jointWidth,0])
            if linkRoundness < 1:
                gLink = exudyn.graphics.Brick(centerPoint=vAxis,
                                              size=vAxis + [jointWidth, jointWidth, jointWidth],
                                              color=linkColors[i],
                                              roundness=linkRoundness,
                                              nTiles=24)
            else:
                
                gLink = exudyn.graphics.Cylinder(pAxis=[0,0,0], vAxis=vAxis*2, 
                                                 radius=jointWidth/1.2,
                                                 color=linkColors[i])
            graphicsDataList[i].append(gLink)

        addGraphics = False #autocompute graphics
        linkColor = exudyn.graphics.color.defaultBody
        if linkParents[i] == -1:
            gDataList = baseGraphicsDataList0
            addGraphics = autoComputeBaseGraphics
            parentAxis = [1,0,0] #any axis
        else:
            addGraphics = listOfTreeLinks[linkParents[i]].graphicsDataList is None
            gDataList = graphicsDataList[linkParents[i]]
            parentAxis = JointTypeToAxis(listOfTreeLinks[linkParents[i]].jointType)
            linkColor = linkColors[linkParents[i]]
            
        v = jointHTs[i].translation
        if addGraphics:
            #joints:
            if listOfTreeLinks[i].graphicsDataList is None:
                gJoint = exudyn.graphics.Cylinder(pAxis=-0.5*jointWidth*axis, vAxis=jointWidth*axis, 
                                                  radius=jointRadius,
                                                  color=jointColors[i])
                graphicsDataList[i].append(gJoint)

            if cMath.norm(v) > 0:
                #links:
                if linkRoundness < 1:
                    axis0 = Normalize(v)
                    axis2 = np.cross(axis0, parentAxis)
                    axis1 = -np.cross(axis0, axis2)
                    lenV = cMath.norm(v) #will always have some extension
                    gLink = exudyn.graphics.Brick(centerPoint=[0.5*lenV,0,0],
                                                  size=[lenV + 1.6*jointRadius, jointWidth, 2*jointRadius],
                                                  color=linkColor,
                                                  roundness=linkRoundness,
                                                  nTiles=24)
                    rot = np.stack((axis0,axis1,axis2),axis=1)
                    p = [0,0,0]
                    gLink = exudyn.graphics.Move(gLink, p, rot)
                else:
                    gLink = exudyn.graphics.Cylinder(pAxis=[0,0,0], vAxis=v, 
                                                     radius=jointWidth/1.2,
                                                     color=linkColor)
                gDataList.append(gLink) #only a link of some length is drawn (#2827)
                
    if parentsNoneType and parentsNumberType:
        raise ValueError(where+': either all TreeLink parents are None and automatically computed or all parents are given as number')

    if len(jointPControlVector) != 0:
        if len(jointPositionOffsetVector)==0:
            jointPositionOffsetVector = np.zeros(nLinks)
    else:
        if len(jointPositionOffsetVector)!=0:
            raise ValueError(where+': arg jointPositionOffsetVector must None if no PDcontrol given in TreeLinks')
    if len(jointPControlVector) != 0:
        if len(jointVelocityOffsetVector)==0:
            jointVelocityOffsetVector = np.zeros(nLinks)
    else:
        if len(jointVelocityOffsetVector)!=0:
            raise ValueError(where+': arg jointVelocityOffsetVector must None if no PDcontrol given in TreeLinks')


    if len(baseGraphicsDataList0) != 0:
        mbs.CreateGround(referencePosition=baseOffset,
                         graphicsDataList=baseGraphicsDataList0)

    #create node for unknowns of KinematicTree
    nGeneric = mbs.AddNode(eii.NodeGenericODE2(name=nodeName,
                                               referenceCoordinates=referenceCoordinates,
                                               initialCoordinates=initialCoordinates,
                                               initialCoordinates_t=initialCoordinates_t,
                                               numberOfODE2Coordinates=nLinks))
    
    #create KinematicTree
    oKT = mbs.AddObject(eii.ObjectKinematicTree(name=name,
                                                nodeNumber=nGeneric, 
                                                jointTypes=jointTypes, 
                                                linkParents=linkParents,
                                                jointHTs=jointHTs,
                                                linkInertiasCOM=linkInertiasCOM, 
                                                linkCOMs=linkCOMs, 
                                                linkMasses=linkMasses,
                                                jointPControlVector = jointPControlVector if hasPDcontrol else [],
                                                jointDControlVector = jointDControlVector if hasPDcontrol else [],
                                                jointPositionOffsetVector=jointPositionOffsetVector if hasPDcontrol else [],
                                                jointVelocityOffsetVector=jointVelocityOffsetVector if hasPDcontrol else [],
                                                jointForceVector = jointForceVector,
                                                baseOffset = baseOffset, 
                                                gravity=gravity,
                                                visualization=eii.VObjectKinematicTree(graphicsDataList = graphicsDataList)
                                                ))

    return oKT


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

@docmeta(author='Sebastian Weyrer')
@extends(exudyn.MainSystem)
def MainSystemCreateFFRFReducedOrderObject(mbs, name, femInterface,
                                           referencePosition=[0., 0., 0.],
                                           initialVelocity=[0., 0., 0.],
                                           referenceRotationMatrix=np.eye(3),
                                           initialAngularVelocity=[0., 0., 0.],
                                           massProportionalDamping=0.,
                                           stiffnessProportionalDamping=0.,
                                           gravity=[0., 0., 0.],
                                           color=exudyn.graphics.color.defaultFFRF,
                                           rigidBodyNodeType=exu.NodeType.RotationEulerParameters,
                                           superElementRigidMarkersOffsets=None,
                                           superElementRigidMarkersLocalHTs=None,
                                           showMarkers=True,
                                           verbose=False) -> dict:
    """Create an FFRF reduced order object; the function adds SuperElementRigid markers if boundaries are defined in the given femInterface and thus enables straightforward integration of flexible bodies into a multibody system

    Args:
        mbs: the MainSystem to which the FFRF reduced order object and the SuperElementRigid markers are added
        name: name of the FFRF reduced order object; used to name the created SuperElementRigid markers (name + ':' + boundaryName), the rigid body node ('NodeRigidBody:' + name), and the generic ODE2 node ('NodeGeneric:' + name); if no name is available, set name=None
        femInterface: an instance of EXUDYN's FEMinterface class; this instance must hold at least a position-based mesh and eigenmodes of the system (for model reduction); usually, also boundaries named [boundaryName0, boundaryName1, ...] are defined within the femInterface; if no boundaries are defined, no SuperElementRigid markers are added
        referencePosition: reference position of the floating frame (i.e. of the rigid body node) (always a 3D vector)
        initialVelocity: initial velocity of the floating frame (i.e. of the rigid body node) (always a 3D vector)
        referenceRotationMatrix: reference rotation matrix for the floating frame (i.e. of the rigid body node) (always a 3D matrix)
        initialAngularVelocity: initial angular velocity of the floating frame (i.e. of the rigid body node) (always a 3D vector)
        massProportionalDamping: Rayleigh damping factor for mass proportional damping (multiplied with reduced mass matrix), added to floating frame/modal coordinates only
        stiffnessProportionalDamping: Rayleigh damping factor for stiffness proportional damping (multiplied with reduced stiffness matrix), added to floating frame/modal coordinates only
        gravity: gravity applied to the FFRF reduced order object (always a 3D vector)
        color: color with which the FFRF reduced order object is drawn (if no contour is set in the visualization settings)
        rigidBodyNodeType: node type of the rigid body node of the floating frame: exu.NodeType.RotationEulerParameters, exu.NodeType.RotationRxyz or exu.NodeType.RotationRotationVector, or one of them as string, e.g., 'NodeType.RotationRxyz'
        superElementRigidMarkersOffsets: if not None, adds local offsets to the created SuperElementRigid markers; if N boundaries are defined in the femInterface, a N x 3 list or N x 3 np.array sets an offset for each added marker, for N=1 also a vector with 3 entries; the order of the offsets follows the order in [boundaryName0, boundaryName1, ...] used when setting up the femInterface
        superElementRigidMarkersLocalHTs: if not None, sets the local homogeneous transformations of the created SuperElementRigid markers: the frame of a marker against the frame the marker computes from the mesh nodes; if N boundaries are defined in the femInterface, a list of N transformations, each an exu.HT or a 4x4 matrix given as list of lists or np.array, sets one transformation for each added marker; the order of the transformations follows the order in [boundaryName0, boundaryName1, ...] used when setting up the femInterface; the translation of a transformation is the offset of the marker, so if superElementRigidMarkersOffsets is also given, both must agree
        showMarkers: if True, SuperElementRigid markers are drawn
        verbose: if True, additional information will be printed in the console upon calling the function

    Returns:
        :dict: dictionary mapping each created SuperElementRigid marker name to its marker number, plus an additional entry under 'FFRFReducedOrderObjectDict' containing information about the created FFRF reduced order object

    Example:
        import exudyn as exu
        from exudyn.FEM import * # includes fem functionality
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        from netgen import occ
        import ngsolve as ngs
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        materials = {'steel':{'youngsModulus':2e11, 'poissonsRatio':0.3, 'density':7850}}
        cuboid = occ.Box((0, -0.1/2, -0.1/2), (1, 0.1/2, 0.1/2))
        boundaryNamesList = ['boundary0', 'boundary1']
        cuboid.faces.Min((1, 0, 0)).name = boundaryNamesList[0]
        cuboid.faces.Max((1, 0, 0)).name = boundaryNamesList[1]
        cuboid.name = 'steel'
        geo = occ.OCCGeometry(cuboid)
        mesh = ngs.Mesh(geo.GenerateMesh(maxh=0.05))
        cuboidFemInterface = FEMinterface()
        cuboidFemInterface.ImportMeshFromNGsolve(mesh=mesh,
                                                 materials=materials,
                                                 boundaryNamesList=boundaryNamesList,
                                                 meshOrder=1)
        [boundaryNodesList, boundaryWeightsList] = cuboidFemInterface.GetBoundaryNodeSetsAsLists()
        cuboidFemInterface.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryNodesList,
                                                         nEigenModes=6,
                                                         boundaryNodesWeights=boundaryWeightsList)
        createFFRFObjectDict = mbs.CreateFFRFReducedOrderObject(name='cuboid',
                                                                femInterface=cuboidFemInterface)
        mboundary0 = createFFRFObjectDict['cuboid:boundary0']
        mboundary1 = createFFRFObjectDict['cuboid:boundary1']
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        SC.visualizationSettings.nodes.show = False
        mbs.SolveDynamic(simulationSettings)
    """
    from exudyn.FEM import FEMinterface, ObjectFFRFreducedOrderInterface
    where = 'MainSystem.CreateFFRFReducedOrderObject(...)'
    errStr = 'ERROR in ' + where + ': '
    # check all the received arguments
    if not exudyn.__useExudynFast:
        # must-have parameters
        nameInvalid = False
        if not isinstance(name, str) and IsNotNone(name): # the data type is not valid
            nameInvalid = True
        elif name == '': # the data type is valid but empty string is not allowed
            nameInvalid = True
        if nameInvalid:
            raise ValueError(errStr + 'Name must be a non-empty string or "None".')
        if not isinstance(femInterface, FEMinterface):
            RaiseTypeError(where=where, argumentName='femInterface', received=femInterface, expectedType='FEMinterface')
        # cms object parameters
        nodeType = StrNodeType2NodeType(rigidBodyNodeType) if isinstance(rigidBodyNodeType, str) else rigidBodyNodeType # gives error if not convertible
        if nodeType not in [exu.NodeType.RotationEulerParameters, exu.NodeType.RotationRxyz, exu.NodeType.RotationRotationVector]:
            RaiseTypeError(where=where, argumentName='rigidBodyNodeType', received=rigidBodyNodeType, expectedType='exu.NodeType.RotationEulerParameters, RotationRxyz or RotationRotationVector, or one of them as string')
        # FFRF object parameters
        if not IsVector(referencePosition, 3):
            RaiseTypeError(where=where, argumentName='referencePosition', received=referencePosition, expectedType=ExpectedType.Vector, dim=3)
        if not IsVector(initialVelocity, 3):
            RaiseTypeError(where=where, argumentName='initialVelocity', received=initialVelocity, expectedType=ExpectedType.Vector, dim=3)
        if not IsSquareMatrix(referenceRotationMatrix, 3):
            RaiseTypeError(where=where, argumentName='referenceRotationMatrix', received=referenceRotationMatrix, expectedType=ExpectedType.Matrix, dim=3)
        if not IsVector(initialAngularVelocity, 3):
            RaiseTypeError(where=where, argumentName='initialAngularVelocity', received=initialAngularVelocity, expectedType=ExpectedType.Vector, dim=3)
        if not IsValidRealInt(massProportionalDamping):
            RaiseTypeError(where=where, argumentName='massProportionalDamping', received=massProportionalDamping, expectedType=ExpectedType.Real)
        if not IsValidRealInt(stiffnessProportionalDamping):
            RaiseTypeError(where=where, argumentName='stiffnessProportionalDamping', received=stiffnessProportionalDamping, expectedType=ExpectedType.Real)
        if not IsVector(gravity, 3):
            RaiseTypeError(where=where, argumentName='gravity', received=gravity, expectedType=ExpectedType.Vector, dim=3)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received=color, expectedType=ExpectedType.Vector, dim=4)
        # marker parameters
        if IsNotNone(superElementRigidMarkersOffsets) and not isinstance(superElementRigidMarkersOffsets, list) and not isinstance(superElementRigidMarkersOffsets, np.ndarray):
            raise ValueError(errStr + 'superElementRigidMarkersOffsets must be "None", a list or a np.array.')
        if IsNotNone(superElementRigidMarkersLocalHTs):
            if not isinstance(superElementRigidMarkersLocalHTs, list):
                RaiseTypeError(where=where, argumentName='superElementRigidMarkersLocalHTs', received=superElementRigidMarkersLocalHTs, expectedType='None or a list of exu.HT or 4x4 matrices')
            for i, localHT in enumerate(superElementRigidMarkersLocalHTs):
                if not isinstance(localHT, exu.HT) and not IsSquareMatrix(localHT, 4):
                    RaiseTypeError(where=where, argumentName='superElementRigidMarkersLocalHTs[' + str(i) + ']', received=localHT, expectedType='exu.HT or 4x4 matrix (as list of lists or numpy array)')
        if not IsValidBool(showMarkers):
            RaiseTypeError(where=where, argumentName='showMarkers', received=showMarkers, expectedType=ExpectedType.Bool)
        # verbose
        if not IsValidBool(verbose):
            RaiseTypeError(where=where, argumentName='verbose', received=verbose, expectedType=ExpectedType.Bool)
        # check whether eigenmodes have been computed
        if femInterface.modeBasis == {}: # this would not be empty if free-free or HCB modes were computed
            raise ValueError(errStr + 'The given femInterface does not hold modes which are needed for component mode synthesis. Use e.g. "femInterface.ComputeHurtyCraigBamptonModes()" before creating an FFRF object.')
    # whether boundaries are present decides whether markers are created, so it is no check and is needed in the fast module as well
    freeEigenmodes = (femInterface.nodeSets == [])
    if freeEigenmodes and verbose:
        # make a print since no boundary conditions here is not very usual; warn the user
        exu.Print('WARNING: The given femInterface does not hold nodeSets: Free eigenmodes are used and no markers will be created.')
    # before doing costly computations, initialize markerNameslist and check whether the marker names and offsets are valid
    # the order of marker names in markerNamesList follows the order in boundaryNamesList upon creating the femInterface
    markerNamesList = []
    if not freeEigenmodes:
        for nodeSet in femInterface.nodeSets: # nodeSets is a list
            if IsNone(name):
                markerName = nodeSet['Name']
            else:
                markerName = name + ':' + nodeSet['Name']
            if not exudyn.__useExudynFast:
                # for the following check, cast the number to integer for comparison since ususally it is of class 'exudyn.exudynCPP.MarkerIndex'
                if int(mbs.GetMarkerNumber(markerName)) != -1: # the marker already exists
                    raise ValueError(errStr + 'The marker ' + markerName + ' already exists.')
            markerNamesList += [markerName]
    nMarkers = len(markerNamesList) # if free eigenmodes: nMarkers = 0
    # define the offsets and local HTs of the markers if there are any markers
    # check if the given offsets and localHTs are valid and refuse them if they are not
    if nMarkers != 0:
        if IsNotNone(superElementRigidMarkersOffsets):
            # always make a np.array that has two dimensions: nMarkers x 3
            superElementRigidMarkersOffsets = np.array(superElementRigidMarkersOffsets)
            if superElementRigidMarkersOffsets.ndim == 1: # only has one dimension, so add the first
                superElementRigidMarkersOffsets = superElementRigidMarkersOffsets[np.newaxis, :]
            if not exudyn.__useExudynFast:
                if superElementRigidMarkersOffsets.shape[0] != nMarkers:
                    raise ValueError(errStr + 'Number of rows in "superElementRigidMarkersOffsets" must be number of interfaces.')
                for i, offset in enumerate(superElementRigidMarkersOffsets):
                    if not IsVector(offset, 3):
                        RaiseTypeError(where=where, argumentName='superElementRigidMarkersOffsets[' + str(i) + ']', received=offset, expectedType=ExpectedType.Vector, dim=3)
        if IsNotNone(superElementRigidMarkersLocalHTs) and not exudyn.__useExudynFast:
            if len(superElementRigidMarkersLocalHTs) != nMarkers:
                raise ValueError(errStr + 'Number of entries in "superElementRigidMarkersLocalHTs" must be number of interfaces.')
            if IsNotNone(superElementRigidMarkersOffsets): # the marker refuses disagreeing values too, but only after the costly computations
                for i, localHT in enumerate(superElementRigidMarkersLocalHTs):
                    translation = exu.HT(localHT).translation
                    if cMath.norm(superElementRigidMarkersOffsets[i] - translation) > 1e-12*(1. + cMath.norm(translation)): # done as in C++
                        raise ValueError(errStr + 'superElementRigidMarkersOffsets[' + str(i) + '] and the translation of superElementRigidMarkersLocalHTs[' + str(i) + '] must agree.')
        # the marker takes None for what is not given
        if IsNone(superElementRigidMarkersOffsets):
            superElementRigidMarkersOffsets = [None]*nMarkers
        if IsNone(superElementRigidMarkersLocalHTs):
            superElementRigidMarkersLocalHTs = [None]*nMarkers
    # use the eigenmodes for component mode synthessis
    cms = ObjectFFRFreducedOrderInterface(femInterface, rigidBodyNodeType=rigidBodyNodeType)
    # here, name can be empty string since this is the default option in AddObjectFFRFreducedOrder
    if IsNone(name):
        name = ''
    objFFRF = cms.AddObjectFFRFreducedOrder(mbs, name=name,
                                        positionRef=referencePosition,
                                        initialVelocity=initialVelocity, 
                                        rotationMatrixRef=referenceRotationMatrix,
                                        initialAngularVelocity=initialAngularVelocity,
                                        massProportionalDamping=massProportionalDamping,
                                        stiffnessProportionalDamping=stiffnessProportionalDamping,
                                        gravity=gravity,
                                        color=color)
    returnDict = {} # initialize dictionary that is returned
    returnDict['FFRFReducedOrderObjectDict'] = objFFRF # and already add information about the FFRF object
    if nMarkers != 0:
        # the order in the following two lists is the same as the order of the nodeSets names and thus in boundaryNamesList
        # it is ensured that marker name and the real marker correspond to each other
        # is is ensured that the offsets are assigend to the correct markers
        [boundaryNodesList, boundaryWeightsList] = femInterface.GetBoundaryNodeSetsAsLists()
        if not exudyn.__useExudynFast:
            if boundaryNodesList == [] or boundaryWeightsList == []:
                 raise ValueError(errStr + 'No boundary nodes and/or boundary weights are found in the femInterface.')
        # add markers according to the markerNamesList
        for i, markerName in enumerate(markerNamesList):
            marker = mbs.AddMarker(eii.MarkerSuperElementRigid(name=markerName,
                                                               bodyNumber=objFFRF['oFFRFreducedOrder'],
                                                               meshNodeNumbers=boundaryNodesList[i],
                                                               weightingFactors=boundaryWeightsList[i],
                                                               offset=superElementRigidMarkersOffsets[i],
                                                               localHT=superElementRigidMarkersLocalHTs[i]))
            returnDict[markerName] = marker # add the marker number to the dict under the key of the marker name
            if verbose:
                exu.Print('Added super element rigid body marker "' + markerName + '" that is marker number ' + str(marker) + ' in the MainSystem.')
    return returnDict




#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateForce(mbs,
                name = '',   
                itemNumber = None,
                loadVector = [0.,0.,0.], 
                localPosition = [0.,0.,0.], 
                bodyFixed = False,
                loadVectorUserFunction = 0,
                show = True, bodyNumber = None) -> exudyn.LoadIndex:
    """helper function to create force applied to a body, a node or a marker

    Args:
        mbs: the MainSystem where items are created
        name: name string for object
        itemNumber: the item the force is applied to: a body (ObjectIndex) at localPosition, a node (NodeIndex) or a marker (MarkerIndex), for which localPosition is [0,0,0]
        loadVector: force vector (as 3D list or numpy array)
        localPosition: local position (as 3D list or numpy array) on the body where the force is applied
        bodyFixed: if True, the force is corotated with the body or node; else, the force is global
        loadVectorUserFunction: A Python function f(mbs, t, load)->loadVector which defines the time-dependent load and replaces loadVector in every time step; the arg load is the static loadVector
        show: if True, load is drawn
        bodyNumber: deprecated name of itemNumber

    Returns:
        :LoadIndex: returns load index

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        b0=mbs.CreateMassPoint(referencePosition = [0,0,0],
                               initialVelocity = [2,5,0],
                               mass = 1, gravity = [0,-9.81,0],
                               drawSize = 0.5, color=exu.graphics.color.blue)
        f0=mbs.CreateForce(itemNumber=b0, loadVector=[100,0,0],
                           localPosition=[0,0,0])
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    where='MainSystem.CreateForce(...)'
    if bodyNumber is not None: #the old name of the argument (#2863)
        DeprecatedArgument('bodyNumber', '1.12.337', 2031, use='itemNumber', function='MainSystem.CreateForce')
        if itemNumber is not None:
            raise TypeError('ERROR in ' + where + ': bodyNumber and itemNumber are given; use itemNumber only')
        itemNumber = bodyNumber
    #error checks:
    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsVector(localPosition, 3):
            RaiseTypeError(where=where, argumentName='localPosition', received = localPosition, expectedType = ExpectedType.Vector, dim=3)

        if ItemKind(itemNumber) is None:
            RaiseTypeError(where=where, argumentName='itemNumber', received = itemNumber, expectedType = 'ObjectIndex, NodeIndex or MarkerIndex')
        elif ItemKind(itemNumber) != 'body' and IsVector(localPosition, 3) and any(x != 0 for x in localPosition):
            RaiseTypeError(where=where, argumentName='localPosition', received = localPosition, expectedType = '[0,0,0] for a node or a marker')

        if not IsVector(loadVector, 3):
            RaiseTypeError(where=where, argumentName='loadVector', received = loadVector, expectedType = ExpectedType.Vector, dim=3)
    
        if not IsValidRealInt(bodyFixed):
            RaiseTypeError(where=where, argumentName='bodyFixed', received = bodyFixed, expectedType = ExpectedType.Bool)
        
        # if not IsUserFunction(loadVectorUserFunction):
        #     RaiseTypeError(where=where, argumentName='loadVectorUserFunction', received = loadVectorUserFunction, expectedType = ExpectedType.UserFunction)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
    
    markerNumber = ItemMarker(mbs, '', 0, itemNumber, localPosition, useRigidMarker=bodyFixed)

    loadNumber = mbs.AddLoad(eii.LoadForceVector(markerNumber=markerNumber, 
                                                 loadVector=loadVector,
                                                 bodyFixed=bodyFixed, 
                                                 loadVectorUserFunction=loadVectorUserFunction))

    return loadNumber


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateTorque(mbs,
                name = '',
                itemNumber = None,
                loadVector = [0.,0.,0.], 
                localPosition = [0.,0.,0.], 
                bodyFixed = False,
                loadVectorUserFunction = 0,
                show = True, bodyNumber = None) -> exudyn.LoadIndex:
    """helper function to create torque applied to a body, a node or a marker

    Args:
        mbs: the MainSystem where items are created
        name: name string for object
        itemNumber: the item the torque is applied to: a body (ObjectIndex) at localPosition, a rigid node (NodeIndex) or a rigid marker (MarkerIndex), for which localPosition is [0,0,0]
        loadVector: torque vector (as 3D list or numpy array)
        localPosition: local position (as 3D list or numpy array) on the body where the torque is applied
        bodyFixed: if True, the torque is corotated with the body or node; else, the torque is global
        loadVectorUserFunction: A Python function f(mbs, t, load)->loadVector which defines the time-dependent load and replaces loadVector in every time step; the arg load is the static loadVector
        show: if True, load is drawn
        bodyNumber: deprecated name of itemNumber

    Returns:
        :LoadIndex: returns load index

    Example:
        import exudyn as exu
        from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
        import numpy as np
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                         sideLengths=[1,0.1,0.1]),
                                 referencePosition = [1,3,0],
                                 gravity = [0,-9.81,0],
                                 graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                              color=exu.graphics.color.red)])
        f0=mbs.CreateTorque(itemNumber=b0, loadVector=[0,100,0])
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    where='MainSystem.CreateTorque(...)'
    if bodyNumber is not None: #the old name of the argument (#2863)
        DeprecatedArgument('bodyNumber', '1.12.337', 2031, use='itemNumber', function='MainSystem.CreateTorque')
        if itemNumber is not None:
            raise TypeError('ERROR in ' + where + ': bodyNumber and itemNumber are given; use itemNumber only')
        itemNumber = bodyNumber
    #error checks:
    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsVector(loadVector, 3):
            RaiseTypeError(where=where, argumentName='loadVector', received = loadVector, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(localPosition, 3):
            RaiseTypeError(where=where, argumentName='localPosition', received = localPosition, expectedType = ExpectedType.Vector, dim=3)


        if ItemKind(itemNumber) is None:
            RaiseTypeError(where=where, argumentName='itemNumber', received = itemNumber, expectedType = 'ObjectIndex, NodeIndex or MarkerIndex')
        elif ItemKind(itemNumber) != 'body' and IsVector(localPosition, 3) and any(x != 0 for x in localPosition):
            RaiseTypeError(where=where, argumentName='localPosition', received = localPosition, expectedType = '[0,0,0] for a node or a marker')
    
        if not IsValidRealInt(bodyFixed):
            RaiseTypeError(where=where, argumentName='bodyFixed', received = bodyFixed, expectedType = ExpectedType.Bool)
        # if not IsUserFunction(loadVectorUserFunction):
        #     RaiseTypeError(where=where, argumentName='loadVectorUserFunction', received = loadVectorUserFunction, expectedType = ExpectedType.UserFunction)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
    
    markerNumber = ItemMarker(mbs, '', 0, itemNumber, localPosition, useRigidMarker=True)

    loadNumber = mbs.AddLoad(eii.LoadTorqueVector(markerNumber=markerNumber, 
                                                  loadVector=loadVector,
                                                  bodyFixed=bodyFixed,
                                                  loadVectorUserFunction=loadVectorUserFunction))

    return loadNumber




#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# exudyn.MainSystem.CreateMassPoint = MainSystemCreateMassPoint
# exudyn.MainSystem.CreateSpringDamper = MainSystemCreateSpringDamper
# exudyn.MainSystem.CreateRevoluteJoint = MainSystemCreateRevoluteJoint
# exudyn.MainSystem.CreatePrismaticJoint = MainSystemCreatePrismaticJoint
# exudyn.MainSystem.CreateGenericJoint = MainSystemCreateGenericJoint

#missing:
#LinearSpringDamper
#TorsionalSpringDamper
#RollingDiscPenalty
#2x rolling disc
#CreateBeamsStraight[2D](...) #ANCF, GE with types?
#CreateBeamsCurved[2D](...)   #ANCF, GE


# #FUTURE:
# #def InitializeFromRestartFile(mbs, simulationSettings, restartFileName, verbose=True):

     
#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#distance sensor and system graph (moved from utilities.py)

def __UFsensorDistance(mbs, t, sensorNumbers, factors, configuration):
    """internal function used for CreateDistanceSensor
    """

    generalContactIndex = int(factors[0])
    dirSensor = factors[5:8]
    markerNumber = int(factors[1])
    hasRotation = False
    if markerNumber != -1:
        p0 = mbs.GetMarkerOutput(markerNumber, variableType=exudyn.OutputVariableType.Position)
        hasRotation = ('Rigid' in mbs.GetMarker(markerNumber)['markerType'])
        if hasRotation:
            A0 = mbs.GetMarkerOutput(markerNumber, variableType=exudyn.OutputVariableType.RotationMatrix).reshape((3,3))
            dirSensor = A0 @ dirSensor
    else:
        p0 = np.array(factors[2:5])

    [minDistance, maxDistance, cylinderRadius, selectedTypeIndex, measureVelocity, graphicsObject, flags] = factors[8:15]
    selectedTypeIndex = exudyn.ContactTypeIndex(int(selectedTypeIndex)) #only converts from int
    measureVelocity = bool(measureVelocity)
    graphicsObject = int(graphicsObject)

    gContact = mbs.GetGeneralContact(generalContactIndex)
    data = gContact.ShortestDistanceAlongLine(pStart = p0, direction = dirSensor, 
                                           minDistance=minDistance, maxDistance=maxDistance,
                                           cylinderRadius=cylinderRadius, asDictionary=(measureVelocity),
                                           typeIndex=selectedTypeIndex,
                                           )
    if measureVelocity:
        d = data['distance']
        v = data['velocityAlongLine']
        rv = [d, v]
    else:
        d = data
        rv = [d]

    if graphicsObject != -1:
        factLen = 1
        if int(flags) == 0:
            factLen = 0

        mbs.SetObjectParameter(graphicsObject,'referencePosition',p0 + factLen*(d*np.array(Normalize(dirSensor))) )
        if hasRotation:
            mbs.SetObjectParameter(graphicsObject,'referenceRotation', A0)
            
    return rv


@extends(exudyn.MainSystem)
def CreateDistanceSensorGeometry(mbs, meshPoints, meshTrigs, rigidBodyMarkerIndex, searchTreeCellSize=[8,8,8]) -> int:
    """Add geometry for distance sensor given by points and triangles (point indices) to mbs; use a rigid body marker where the geometry is put on;
    Creates a GeneralContact for efficient search on background. If you have several sets of points and trigs, first merge them or add them manually to the contact

    Args:
        mbs: MainSystem where contact is created
        meshPoints: list of points (3D), as returned by graphics.ToPointsAndTrigs()
        meshTrigs: list of trigs (3 node indices each), as returned by graphics.ToPointsAndTrigs()
        rigidBodyMarkerIndex: rigid body marker to which the triangles are fixed on (ground or moving object)
        searchTreeCellSize: size of search tree (X,Y,Z); use larger values in directions where more triangles are located

    Returns:
        :int: returns ngc, which is the number of GeneralContact in mbs, to be used in CreateDistanceSensor(...); keep the gContact as deletion may corrupt data

    Note:
        should be used by CreateDistanceSensor(...) and AddLidar(...) for simple initialization of GeneralContact; old name: DistanceSensorSetupGeometry(...)
    """
    gContact = mbs.AddGeneralContact()
    gContact.SetFrictionPairings(0*np.eye(1)) #may not be empty
    gContact.SetSearchTreeCellSize(numberOfCells=searchTreeCellSize)
    # [meshPoints, meshTrigs] = RefineMesh(meshPoints, meshTrigs) #just to have more triangles on floor
    gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=rigidBodyMarkerIndex,
                                        contactStiffness=1, contactDamping=1, #dummy values
                                        frictionMaterialIndex=0, pointList=meshPoints, triangleList=meshTrigs)
    gContact.isActive=False #no contact computation; could also be done later on, if many moving objects are used ...
    ngc = mbs.NumberOfGeneralContacts()-1
    gContact = mbs.GetGeneralContact(ngc) #keeps reference to gContact, while other functions work with automatic ...

    return ngc


@extends(exudyn.MainSystem)
def CreateDistanceSensor(mbs, generalContactIndex,
                      positionOrMarker, dirSensor, minDistance=-1e7, 
                      maxDistance=1e7, cylinderRadius=0, 
                      selectedTypeIndex=exudyn.ContactTypeIndex.IndexEndOfEnumList,
                      storeInternal = False, fileName = '', measureVelocity = False,
                      addGraphicsObject=False, drawDisplaced=True, color=exudyn.graphics.color.red) -> exudyn.SensorIndex:
    """Function to create distance sensor based on GeneralContact in mbs; sensor can be either placed on absolute position or attached to rigid body marker; in case of marker, dirSensor is relative to the marker

    Args:
        mbs: the MainSystem where distance sensor is created
        generalContactIndex: the number of the GeneralContact object in mbs; the index of the GeneralContact object which has been added with last AddGeneralContact(...) command is generalContactIndex=mbs.NumberOfGeneralContacts()-1
        positionOrMarker: either a 3D position as list or np.array, or a MarkerIndex with according rigid body marker
        dirSensor: the direction (no need to normalize) along which the distance is measured (must not be normalized); in case of marker, the direction is relative to marker orientation if marker contains orientation (BodyRigid, NodeRigid)
        minDistance: the minimum distance which is accepted; smaller distance will be ignored
        maxDistance: the maximum distance which is accepted; items being at maxDistance or futher are ignored; if no items are found, the function returns maxDistance
        cylinderRadius: in case of spheres (selectedTypeIndex=ContactTypeIndex.IndexSpheresMarkerBased), a cylinder can be used which measures the shortest distance at a certain radius (geometrically interpreted as cylinder)
        selectedTypeIndex: either this type has default value, meaning that all items in GeneralContact are measured, or there is a specific type index, which is the only type that is considered during measurement
        storeInternal: like with any SensorUserFunction, setting to True stores sensor data internally
        fileName: if defined, recorded data of SensorUserFunction is written to specified file
        measureVelocity: if True, the sensor measures additionally the velocity (component 0=distance, component 1=velocity); velocity is the velocity in direction 'dirSensor' and does not account for changes in geometry, thus it may be different from the time derivative of the distance!
        addGraphicsObject: if True, the distance sensor is also visualized graphically in a simplified manner with a red line having the length of dirSensor; NOTE that updates are ONLY performed during computation, not in visualization; for this reason, solution.sensors.writePeriod should be accordingly small
        drawDisplaced: if True, the red line is drawn backwards such that it moves along the measured surface; if False, the beam is fixed to marker or position
        color: optional color for 'laser beam' to be drawn

    Returns:
        :SensorIndex: creates sensor and returns according sensor number of SensorUserFunction

    Note:
        use generalContactIndex = CreateDistanceSensorGeometry(...) before to create GeneralContact module containing geometry; old name: AddDistanceSensor(...)
    """
    
    markerNumber = -1
    p0list = [0,0,0]
    if type(positionOrMarker) == list or type(positionOrMarker) == np.ndarray:
        p0list = list(positionOrMarker)
    elif type(positionOrMarker)==exudyn.MarkerIndex:
        markerNumber = float(int(positionOrMarker))
        #the reference position of the marker, if it has one; else zeros
        if exudyn.OutputVariableType.Position in mbs.Inspect(positionOrMarker, exudyn.InspectType.OutputVariables):
            p0list = list(mbs.GetMarkerOutput(markerNumber=positionOrMarker,
                                              variableType=exudyn.OutputVariableType.Position,
                                              configuration=exudyn.ConfigurationType.Reference))
    else:
        raise ValueError('CreateDistanceSensor: positionOrMarker must be either MarkerIndex or 3D position as list or numpy.array')

    graphicsObject = -1 #signals that there is no graphics object
    sign = 1.
    if drawDisplaced:
        sign = -1.
    if addGraphicsObject:
        if cylinderRadius == 0:
            gData = exudyn.graphics.Lines([[0,0,0],list(sign*np.array(dirSensor))], color = color)
        else: 
            gData = exudyn.graphics.Cylinder([0,0,0],sign*np.array(dirSensor), radius=cylinderRadius, color = color)
            
        graphicsObject=mbs.AddObject(ObjectGround(referencePosition= p0list,
                                      visualization=VObjectGround(graphicsData=[gData])))

    flags = int(drawDisplaced)
    dataUF = [float(generalContactIndex)]
    dataUF += [markerNumber] + p0list + list(dirSensor)
    dataUF += [ minDistance, maxDistance, cylinderRadius, float(int(selectedTypeIndex)), float(measureVelocity), float(int(graphicsObject)), float(flags)] 


    sUF = mbs.AddSensor(SensorUserFunction(sensorNumbers=[], factors=dataUF,
                                              storeInternal=storeInternal,
                                              fileName=fileName,
                                              sensorUserFunction=__UFsensorDistance))

    return sUF


#bind all functions marked with @extends (in this module, plot, solver, systemUtilities, interactive) to their classes
install()
