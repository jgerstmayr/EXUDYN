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
import exudyn.interactive
import exudyn.graphics
from exudyn.basicUtilities import Normalize

from exudyn.rigidBodyUtilities import _MarkerWithRotation, _RotationMarkerArgs
from exudyn.rigidBodyUtilities import ComputeOrthonormalBasis, \
    RotationMatrix2EulerParameters, AngularVelocity2EulerParameters_t, RotationMatrix2RotXYZ, AngularVelocity2RotXYZ_t, \
    RotationMatrix2RotationVector

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
    'MainSystemCreateRigidBodySpringDamper', 'MainSystemCreateTorsionalSpringDamper',
    'MainSystemCreateRevoluteJoint', 'MainSystemCreatePrismaticJoint',
    'MainSystemCreateSphericalJoint', 'MainSystemCreateGenericJoint',
    'MainSystemCreateDistanceConstraint', 'MainSystemCreateCoordinateConstraint',
    'MainSystemCreateRollingDisc', 'MainSystemCreateRollingDiscPenalty',
    'MainSystemCreateSphereSphereContact', 'MainSystemCreateSphereQuadContact',
    'MainSystemCreateSphereTriangleContact', 'MainSystemCreateKinematicTree',
    'MainSystemCreateFFRFReducedOrderObject', 'MainSystemCreateForce', 'MainSystemCreateTorque',
    'CreateDistanceSensorGeometry', 'CreateDistanceSensor', 'DrawSystemGraph',
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


#internal function: do some pre-checks and calculations for joint
#extended function which also accepts markers in bodyNumbers and returns new or existing markers
#marker0 overrides the joint "position"
@docmeta(public=False)
def JointPreCheckCalcBodyMarkers(where, mbs, name, bodyNumbers, position, show, useGlobalFrame, requireRotMat=True):
    """Helper to calculate markers."""
    if not exudyn.__useExudynFast:
        if not isinstance(bodyNumbers, list) or len(bodyNumbers) != 2:
            RaiseTypeError(where=where, argumentName='bodyNumbers', received = bodyNumbers, expectedType = 'list of 2 body or marker numbers')
        if not IsValidObjectIndex(bodyNumbers[0]):
            if not isinstance(bodyNumbers[0], exudyn.MarkerIndex): #also accept marker
                RaiseTypeError(where=where, argumentName='bodyNumbers[0]', received = bodyNumbers[0], expectedType = 'ObjectIndex or MarkerIndex')
            elif np.linalg.norm(position) != 0: #for marker, position must be zero!
                RaiseTypeError(where=where, argumentName='position', received = position, expectedType = '[0,0,0]')
                
        if not IsValidObjectIndex(bodyNumbers[1]):
            if not isinstance(bodyNumbers[1], exudyn.MarkerIndex): #also accept marker
                RaiseTypeError(where=where, argumentName='bodyNumbers[1]', received = bodyNumbers[1], expectedType = 'ObjectIndex or MarkerIndex')
            elif np.linalg.norm(position) != 0: #for marker, position must be zero!
                RaiseTypeError(where=where, argumentName='position', received = position, expectedType = '[0,0,0]')
    
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsValidBool(useGlobalFrame):
            RaiseTypeError(where=where, argumentName='useGlobalFrame', received = useGlobalFrame, expectedType = ExpectedType.Bool)
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

    mBody0 = bodyNumbers[0] if isinstance(bodyNumbers[0], exudyn.MarkerIndex) else None
    mBody1 = bodyNumbers[1] if isinstance(bodyNumbers[1], exudyn.MarkerIndex) else None

    if not exudyn.__useExudynFast:
        if mBody0 is not None or mBody1 is not None:
            if not IsVector(position) or len(position) != 0:
                raise ValueError('ERROR in ' + where + ' in argument "position": ' +
                                 'in case that a marker number is provided, position must be []')
        elif not IsVector(position, 3):
            RaiseTypeError(where=where, argumentName='position', received = position, expectedType = ExpectedType.Vector, dim=3)

    pJoint = None

    if mBody0 is None:
        p0 = mbs.GetObjectOutputBody(bodyNumbers[0],exudyn.OutputVariableType.Position,
                                     localPosition=[0,0,0],
                                     configuration=exudyn.ConfigurationType.Reference)
        A0 = mbs.GetObjectOutputBody(bodyNumbers[0],exudyn.OutputVariableType.RotationMatrix,
                                     localPosition=[0,0,0],
                                     configuration=exudyn.ConfigurationType.Reference).reshape((3,3))
    else:
        p0 = mbs.GetMarkerOutput(bodyNumbers[0],
                                 exudyn.OutputVariableType.Position,
                                 configuration=exudyn.ConfigurationType.Reference)
        A0 = mbs.GetMarkerOutput(bodyNumbers[0],
                                 exudyn.OutputVariableType.RotationMatrix,
                                 configuration=exudyn.ConfigurationType.Reference).reshape((3,3))
        pJoint = p0 #marker sets the global joint position!
        
    if mBody1 is None:
        p1 = mbs.GetObjectOutputBody(bodyNumbers[1],exudyn.OutputVariableType.Position,
                                     localPosition=[0,0,0],
                                     configuration=exudyn.ConfigurationType.Reference)
        A1 = mbs.GetObjectOutputBody(bodyNumbers[1],exudyn.OutputVariableType.RotationMatrix,
                                     localPosition=[0,0,0],
                                     configuration=exudyn.ConfigurationType.Reference).reshape((3,3))
    else:
        p1 = mbs.GetMarkerOutput(bodyNumbers[1],
                                 exudyn.OutputVariableType.Position,
                                 configuration=exudyn.ConfigurationType.Reference)
        A1 = mbs.GetMarkerOutput(bodyNumbers[1],
                                 exudyn.OutputVariableType.RotationMatrix,
                                 configuration=exudyn.ConfigurationType.Reference).reshape((3,3))
        if pJoint is None:
            pJoint = p1 #marker sets the global joint position!

    if pJoint is None:
        if useGlobalFrame:
            pJoint = copy.copy(position)
        else: #transform into global coordinates, then everything works same
            pJoint = A0 @ position + p0
    
    return [p0, A0, p1, A1, mBody0, mBody1, pJoint]


#internal function, which checks bodyList and bodyOrNodeList and returns appropriate bodyOrNodeList
@docmeta(public=False)
def ProcessBodyNodeMarkerLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where, bodyList=[None,None]):
    """Helper to check which items to refer to."""
    if not exudyn.__useExudynFast:
        if not isinstance(bodyList, list) or len(bodyList) != 2:
            RaiseTypeError(where=where, argumentName='bodyList', received = bodyList, expectedType = 'list of 2 body numbers')
        if not isinstance(bodyNumbers, list) or len(bodyNumbers) != 2:
            RaiseTypeError(where=where, argumentName='bodyNumbers', received = bodyNumbers, expectedType = 'list of 2 body, node or marker numbers')

    causingArgName = 'bodyOrNodeList'
    if IsNotNone(bodyNumbers[0]) or IsNotNone(bodyNumbers[1]):
        bodyOrNodeList = [bodyNumbers[0],bodyNumbers[1]] #flat copy, but otherwise would lead to change of args (mutable args!)
        causingArgName = 'bodyNumbers'
    elif IsNotNone(bodyList[0]) or IsNotNone(bodyList[1]):
        #reported for the Create function, one stack level further up than the helper
        DeprecatedArgument('bodyList', '1.11.0', 2029, use='bodyNumbers', function=where.replace('(...)', ''), stackLevel=4)
        bodyOrNodeList = [bodyList[0],bodyList[1]] #flat copy, but otherwise would lead to change of args (mutable args!)
        causingArgName = 'bodyList'

    if not exudyn.__useExudynFast:
        if not isinstance(bodyOrNodeList, list) or len(bodyOrNodeList) != 2:
            RaiseTypeError(where=where, argumentName='bodyOrNodeList', received = bodyOrNodeList, expectedType = 'list of 2 body or node numbers')
    
        if (not (IsValidObjectIndex(bodyOrNodeList[0]) 
                 or (isinstance(bodyOrNodeList[0], exudyn.NodeIndex) and localPosition0==[0.,0.,0.]) 
                 or (isinstance(bodyOrNodeList[0], exudyn.MarkerIndex) and localPosition0==[0.,0.,0.]) ) ):
            RaiseTypeError(where=where, argumentName=''+causingArgName+'[0]', received = bodyOrNodeList[0], 
                           expectedType = 'expected either ObjectIndex, or NodeIndex/MarkerIndex AND localPosition0=[0.,0.,0.]')
            
        if (not (IsValidObjectIndex(bodyOrNodeList[1]) 
                 or (isinstance(bodyOrNodeList[1], exudyn.NodeIndex) and localPosition1==[0.,0.,0.]) 
                 or (isinstance(bodyOrNodeList[1], exudyn.MarkerIndex) and localPosition1==[0.,0.,0.]) ) ):
            RaiseTypeError(where=where, argumentName=''+causingArgName+'[1]', received = bodyOrNodeList[1], 
                           expectedType = 'expected either ObjectIndex, or NodeIndex/MarkerIndex AND localPosition1=[0.,0.,0.]')
    
    return bodyOrNodeList


#internal: get markers, positions and orientations
@docmeta(public=False)
def GetMarkersPosRot(mbs, name, internBodyNodeMarkerList, localPosition0, localPosition1, 
                     getPosition=False, getRotationMatrix=False, useRigidMarker=False):
    """Helper to calculate marker pose."""
    MarkerBodyType = eii.MarkerBodyRigid if useRigidMarker else eii.MarkerBodyPosition
    MarkerNodeType = eii.MarkerNodeRigid if useRigidMarker else eii.MarkerNodePosition
    
    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name
    
    if IsValidObjectIndex(internBodyNodeMarkerList[0]):
        mBody0 = mbs.AddMarker(MarkerBodyType(name=mName0,bodyNumber=internBodyNodeMarkerList[0], localPosition=localPosition0))
    elif isinstance(internBodyNodeMarkerList[0], exudyn.NodeIndex):
        mBody0 = mbs.AddMarker(MarkerNodeType(name=mName0,nodeNumber=internBodyNodeMarkerList[0]))
    elif isinstance(internBodyNodeMarkerList[0], exudyn.MarkerIndex):
        mBody0 = internBodyNodeMarkerList[0]

    if IsValidObjectIndex(internBodyNodeMarkerList[1]):
        mBody1 = mbs.AddMarker(MarkerBodyType(name=mName1,bodyNumber=internBodyNodeMarkerList[1], localPosition=localPosition1))
    elif isinstance(internBodyNodeMarkerList[1], exudyn.NodeIndex):
        mBody1 = mbs.AddMarker(MarkerNodeType(name=mName1,nodeNumber=internBodyNodeMarkerList[1]))
    elif isinstance(internBodyNodeMarkerList[1], exudyn.MarkerIndex):
        mBody1 = internBodyNodeMarkerList[1]
    
    p0 = None
    p1 = None
    A0 = None
    A1 = None
    if getPosition:
        if isinstance(internBodyNodeMarkerList[0], exudyn.ObjectIndex):
            p0 = mbs.GetObjectOutputBody(internBodyNodeMarkerList[0],exudyn.OutputVariableType.Position,
                                         localPosition=localPosition0, configuration=exudyn.ConfigurationType.Reference)
        elif isinstance(internBodyNodeMarkerList[0], exudyn.NodeIndex):
            p0 = mbs.GetNodeOutput(internBodyNodeMarkerList[0],exudyn.OutputVariableType.Position, configuration=exudyn.ConfigurationType.Reference)
        else:
            p0 = mbs.GetMarkerOutput(internBodyNodeMarkerList[0],exudyn.OutputVariableType.Position, configuration=exudyn.ConfigurationType.Reference)

        if isinstance(internBodyNodeMarkerList[1], exudyn.ObjectIndex):
            p1 = mbs.GetObjectOutputBody(internBodyNodeMarkerList[1],exudyn.OutputVariableType.Position,
                                         localPosition=localPosition1, configuration=exudyn.ConfigurationType.Reference)
        elif isinstance(internBodyNodeMarkerList[1], exudyn.NodeIndex):
            p1 = mbs.GetNodeOutput(internBodyNodeMarkerList[1],exudyn.OutputVariableType.Position, configuration=exudyn.ConfigurationType.Reference)
        else:
            p1 = mbs.GetMarkerOutput(internBodyNodeMarkerList[1],exudyn.OutputVariableType.Position, configuration=exudyn.ConfigurationType.Reference)

    if getRotationMatrix:
        if isinstance(internBodyNodeMarkerList[0], exudyn.ObjectIndex):
            A0 = mbs.GetObjectOutputBody(objectNumber=internBodyNodeMarkerList[0],variableType=exudyn.OutputVariableType.RotationMatrix,
                                         localPosition=localPosition0,
                                         configuration=exudyn.ConfigurationType.Reference).reshape((3,3))
        elif isinstance(internBodyNodeMarkerList[0], exudyn.NodeIndex):
            A0 = mbs.GetNodeOutput(nodeNumber=internBodyNodeMarkerList[0], variableType=exudyn.OutputVariableType.RotationMatrix,
                                   configuration=exudyn.ConfigurationType.Reference).reshape((3,3))
        else:
            A0 = mbs.GetMarkerOutput(internBodyNodeMarkerList[0], variableType=exudyn.OutputVariableType.RotationMatrix,
                                     configuration=exudyn.ConfigurationType.Reference).reshape((3,3))
    
        if isinstance(internBodyNodeMarkerList[1], exudyn.ObjectIndex):
            mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=internBodyNodeMarkerList[1], localPosition=localPosition1))
            A1 = mbs.GetObjectOutputBody(objectNumber=internBodyNodeMarkerList[1],variableType=exudyn.OutputVariableType.RotationMatrix,
                                         localPosition=localPosition1,
                                         configuration=exudyn.ConfigurationType.Reference).reshape((3,3))
        elif isinstance(internBodyNodeMarkerList[1], exudyn.NodeIndex):
            mBody1 = mbs.AddMarker(eii.MarkerNodeRigid(name=mName1,nodeNumber=internBodyNodeMarkerList[1]))
            A1 = mbs.GetNodeOutput(nodeNumber=internBodyNodeMarkerList[1], variableType=exudyn.OutputVariableType.RotationMatrix,
                                   configuration=exudyn.ConfigurationType.Reference).reshape((3,3))
        else:
            A1 = mbs.GetMarkerOutput(internBodyNodeMarkerList[1], variableType=exudyn.OutputVariableType.RotationMatrix,
                                     configuration=exudyn.ConfigurationType.Reference).reshape((3,3))
    
    return [mBody0, mBody1, p0, p1, A0, A1]


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
            if np.linalg.norm(inertia.com) > 1e-14:
                raise ValueError('MainSystem.CreateRigidBody(...): in case of 2D rigid body, the center of mass in inertia must be [0,0,0] (will be fixed in future)')


        referenceRot = np.arctan2(A[1,0],A[0,0])
    
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
                phi0 = np.arctan2(A0[1,0],A0[0,0]) - referenceRot
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

    if np.linalg.norm(gravity) != 0.:
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
                                 bodyNumbers=[None, None], 
                                 localPosition0 = [0.,0.,0.],
                                 localPosition1 = [0.,0.,0.], 
                                 referenceLength = None, 
                                 stiffness = 0., damping = 0., force = 0.,
                                 velocityOffset = 0., 
                                 springForceUserFunction = 0,
                                 bodyOrNodeList=[None, None], 
                                 bodyList=[None, None],
                                 show=True, drawSize=-1, color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """helper function to create SpringDamper connector, using arguments from ObjectConnectorSpringDamper; similar interface as CreateDistanceConstraint(...), see there for for further information

    Args:
        mbs: the MainSystem where items are created
        name: name string for connector; markers get Marker0:name and Marker1:name
        bodyNumbers: a list of two body numbers (ObjectIndex) to be connected; alternatively, MarkerIndex or NodeIndex can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
        localPosition0: local position (as 3D list or numpy array) on body0, if not a node of marker number
        localPosition1: local position (as 3D list or numpy array) on body1, if not a node of marker number
        referenceLength: if None, length is computed from reference position of bodies or nodes; if not None, this scalar reference length is used for spring
        stiffness: scalar stiffness coefficient
        damping: scalar damping coefficient
        force: scalar additional force applied
        velocityOffset: scalar offset: if referenceLength is changed over time, the velocityOffset may be changed accordingly to emulate a reference motion
        springForceUserFunction: a user function springForceUserFunction(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force)->float ; this function replaces the internal connector force computation
        bodyOrNodeList: alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
        show: if True, connector visualization is drawn
        drawSize: general drawing size of connector
        color: color of connector
        bodyList: DEPRECATED

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
        oSD = mbs.CreateSpringDamper(bodyNumbers=[oGround, b0],
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
    internBodyNodeMarkerList = ProcessBodyNodeMarkerLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where, bodyList)
    
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


    [mBody0, mBody1, p0, p1, A0, A1] = GetMarkersPosRot(mbs, name, internBodyNodeMarkerList, localPosition0, localPosition1, 
                                                        getPosition=True, getRotationMatrix=False, useRigidMarker=False)
        
    if IsNone(referenceLength): #automatically compute reference length
        referenceLength = np.linalg.norm(np.array(p1)-p0)
    
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
                                 bodyNumbers=[None, None], 
                                 localPosition0 = [0.,0.,0.],
                                 localPosition1 = [0.,0.,0.], 
                                 stiffness = [0.,0.,0.], damping = [0.,0.,0.], 
                                 offset = [0.,0.,0.],
                                 springForceUserFunction = 0,
                                 bodyOrNodeList=[None, None],
                                 bodyList=[None, None],
                                 show=True, drawSize=-1, color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """helper function to create CartesianSpringDamper connector, using arguments from ObjectConnectorCartesianSpringDamper

    Args:
        mbs: the MainSystem where items are created
        name: name string for connector; markers get Marker0:name and Marker1:name
        bodyNumbers: a list of two body numbers (ObjectIndex) to be connected; alternatively, MarkerIndex or NodeIndex can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
        localPosition0: local position (as 3D list or numpy array) on body0, if not a node of marker number
        localPosition1: local position (as 3D list or numpy array) on body1, if not a node of marker number
        stiffness: stiffness coefficients (as 3D list or numpy array)
        damping: damping coefficients (as 3D list or numpy array)
        offset: offset vector (as 3D list or numpy array)
        springForceUserFunction: a user function springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset)->[float,float,float] ; this function replaces the internal connector force computation
        bodyOrNodeList: alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
        bodyList: DEPRECATED
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
        b0 = mbs.CreateMassPoint(referencePosition = [7,0,0],
                                  mass = 1, gravity = [0,-9.81,0],
                                  drawSize = 0.5, color=exu.graphics.color.blue)
        oGround = mbs.AddObject(ObjectGround())
        oSD = mbs.CreateCartesianSpringDamper(bodyNumbers=[oGround, b0],
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
    internBodyNodeMarkerList = ProcessBodyNodeMarkerLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where, bodyList)

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

    [mBody0, mBody1, p0, p1, A0, A1] = GetMarkersPosRot(mbs, name, internBodyNodeMarkerList, localPosition0, localPosition1, 
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
                                 bodyNumbers=[None, None], 
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
                                 bodyOrNodeList=[None, None],
                                 bodyList=[None, None],
                                 show=True, drawSize=-1, color=exudyn.graphics.color.default, intrinsicFormulation=None) -> exudyn.ObjectIndex:
    """helper function to create RigidBodySpringDamper connector, using arguments from ObjectConnectorRigidBodySpringDamper, see there for the full documentation

    Args:
        mbs: the MainSystem where items are created
        name: name string for connector; markers get Marker0:name and Marker1:name
        bodyNumbers: a list of two body numbers (ObjectIndex) to be connected; alternatively, MarkerIndex or NodeIndex can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
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
        bodyOrNodeList: alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
        bodyList: DEPRECATED
        show: if True, connector visualization is drawn
        drawSize: general drawing size of connector
        color: color of connector
        intrinsicFormulation: deprecated name of useIntrinsicFormulation

    Returns:
        :ObjectIndex: returns index of newly created object

    Example:
        #coming later
    """
    if intrinsicFormulation is not None: #the old name of the argument (#2814)
        DeprecatedArgument('intrinsicFormulation', '1.12.258', 2031, use='useIntrinsicFormulation', function='MainSystem.CreateRigidBodySpringDamper')
        useIntrinsicFormulation = intrinsicFormulation
    where='MainSystem.CreateRigidBodySpringDamper(...)'
    internBodyNodeMarkerList = ProcessBodyNodeMarkerLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where, bodyList)

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


    [mBody0, mBody1, p0, p1, A0, A1] = GetMarkersPosRot(mbs, name, internBodyNodeMarkerList, localPosition0, localPosition1, 
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
    MR0 = _RotationIntoMarker(mbs, mBody0, internBodyNodeMarkerList[0], localPosition0, MR0)
    MR1 = _RotationIntoMarker(mbs, mBody1, internBodyNodeMarkerList[1], localPosition1, MR1)
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
                                          bodyNumbers=[None, None], 
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
                                          show=True, drawSize=-1, color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """helper function to create TorsionalSpringDamper connector, using arguments from ObjectConnectorTorsionalSpringDamper, see there for the full documentation

    Args:
        mbs: the MainSystem where items are created
        name: name string for connector; markers get Marker0:name and Marker1:name
        bodyNumbers: a list of two body numbers (ObjectIndex) to be connected; alternatively, MarkerIndex can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
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

    Returns:
        :ObjectIndex: returns index of newly created object

    Example:
        #coming later
    """
    where='MainSystem.CreateTorsionalSpringDamper(...)'

    #perform some checks:
    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)
                
        if not IsVector(position, 3):
            RaiseTypeError(where=where, argumentName='position', received = position, expectedType = ExpectedType.Vector, dim=3)
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
    [p0, A0, p1, A1, mBody0, mBody1, pJoint] = JointPreCheckCalcBodyMarkers(where, mbs, name, bodyNumbers, position, show, useGlobalFrame)
        
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
        mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=bodyNumbers[0], localHT=exu.HT(rotation=MR0, translation=pJ0)))
        MR0 = np.eye(3)
    if mBody1 is None:
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=bodyNumbers[1], localHT=exu.HT(rotation=MR1, translation=pJ1)))
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
def MainSystemCreateRevoluteJoint(mbs, name='', bodyNumbers=[None, None], 
                                  position=[], axis=[], useGlobalFrame=True, 
                                  show=True, axisRadius=0.1, axisLength=0.4, color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """Create revolute joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        bodyNumbers: a list of two body numbers (ObjectIndex) to be connected; must be rigid body or ground object; alternatively, MarkerIndex (Rigid) can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
        position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
        axis: a 3D vector as list or np.array containing the joint axis either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True)
        useGlobalFrame: if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
        show: if True, connector visualization is drawn
        axisRadius: radius of axis for connector graphical representation
        axisLength: length of axis for connector graphical representation
        color: color of connector

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
        mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b0], position=[2.5,0,0], axis=[0,0,1],
                                useGlobalFrame=True, axisRadius=0.02, axisLength=0.14)
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    where = 'MainSystem.CreateRevoluteJoint(...)'
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
    [p0, A0, p1, A1, mBody0, mBody1, pJoint] = JointPreCheckCalcBodyMarkers(where, mbs, name, bodyNumbers, position, show, useGlobalFrame)
        
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
        mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=bodyNumbers[0], localHT=exu.HT(rotation=MR0, translation=pJ0)))
        MR0 = np.eye(3)
    if mBody1 is None:
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=bodyNumbers[1], localHT=exu.HT(rotation=MR1, translation=pJ1)))
        MR1 = np.eye(3)
    (mBody0, MR0) = _MarkerWithRotation(mbs, mBody0, MR0) #a marker the caller gave: a copy turned by the rotation (#2804)
    (mBody1, MR1) = _MarkerWithRotation(mbs, mBody1, MR1)
    
    oJoint = mbs.AddObject(eii.ObjectJointRevoluteZ(name=name,markerNumbers=[mBody0,mBody1],
                                                **_RotationMarkerArgs(MR0, MR1), #only a rotation a marker cannot take (#2820)
             visualization=eii.VRevoluteJointZ(show=show, axisRadius=axisRadius, axisLength=axisLength, color=color) ))

    return oJoint


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreatePrismaticJoint(mbs, name='', bodyNumbers=[None, None], 
                                  position=[], axis=[], useGlobalFrame=True, 
                                  show=True, axisRadius=0.1, axisLength=0.4, color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """Create prismatic joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        bodyNumbers: a list of two body numbers (ObjectIndex) to be connected; must be rigid body or ground object; alternatively, MarkerIndex (Rigid) can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
        position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
        axis: a 3D vector as list or np.array containing the joint axis either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True)
        useGlobalFrame: if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
        show: if True, connector visualization is drawn
        axisRadius: radius of axis for connector graphical representation
        axisLength: length of axis for connector graphical representation
        color: color of connector

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
        mbs.CreatePrismaticJoint(bodyNumbers=[oGround, b0], position=[3.5,0,0], axis=[0,1,0],
                                 useGlobalFrame=True, axisRadius=0.02, axisLength=1)
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    where = 'MainSystem.CreatePrismaticJoint(...)'
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

    [p0, A0, p1, A1, mBody0, mBody1, pJoint] = JointPreCheckCalcBodyMarkers(where, mbs, name, bodyNumbers, position, show, useGlobalFrame)
        
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
        mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=bodyNumbers[0], localHT=exu.HT(rotation=MR0, translation=pJ0)))
        MR0 = np.eye(3)
    if mBody1 is None:
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=bodyNumbers[1], localHT=exu.HT(rotation=MR1, translation=pJ1)))
        MR1 = np.eye(3)
    (mBody0, MR0) = _MarkerWithRotation(mbs, mBody0, MR0) #a marker the caller gave: a copy turned by the rotation (#2804)
    (mBody1, MR1) = _MarkerWithRotation(mbs, mBody1, MR1)
    
    oJoint = mbs.AddObject(eii.ObjectJointPrismaticX(name=name,markerNumbers=[mBody0,mBody1],
                                                **_RotationMarkerArgs(MR0, MR1), #only a rotation a marker cannot take (#2820)
             visualization=eii.VPrismaticJointX(show=show, axisRadius=axisRadius, axisLength=axisLength, color=color) ))

    return oJoint


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateSphericalJoint(mbs, name='', bodyNumbers=[None, None], 
                                  position=[], constrainedAxes=[1,1,1], useGlobalFrame=True, 
                                  show=True, jointRadius=0.1, color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """Create spherical joint between two bodies; definition of joint position in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers are automatically computed

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        bodyNumbers: a list of two body numbers (ObjectIndex) to be connected; must be point mass, rigid body or ground object; alternatively, MarkerIndex can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
        position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
        constrainedAxes: flags, which determines which (global) translation axes are constrained; each entry may only be 0 (=free) axis or 1 (=constrained axis)
        useGlobalFrame: if False, the point and axis vectors are defined in the local coordinate system of body0
        show: if True, connector visualization is drawn
        jointRadius: radius of sphere for connector graphical representation
        color: color of connector

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
        mbs.CreateSphericalJoint(bodyNumbers=[oGround, b0], position=[5.5,0,0],
                                 useGlobalFrame=True, jointRadius=0.06)
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    where = 'MainSystem.CreateSphericalJoint(...)'
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
    [p0, A0, p1, A1, mBody0, mBody1, pJoint] = JointPreCheckCalcBodyMarkers(where, mbs, name, bodyNumbers, position, show, useGlobalFrame, requireRotMat=False)
        
    #compute joint position and axis in bodyNumber0 / 1 coordinates:
    pJ0 = _MatVec3(A0.T, np.array(pJoint) - p0)
    pJ1 = _MatVec3(A1.T, np.array(pJoint) - p1)

    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name

    if mBody0 is None: mBody0 = mbs.AddMarker(eii.MarkerBodyPosition(name=mName0,bodyNumber=bodyNumbers[0], localPosition=pJ0))
    if mBody1 is None: mBody1 = mbs.AddMarker(eii.MarkerBodyPosition(name=mName1,bodyNumber=bodyNumbers[1], localPosition=pJ1))
    
    oJoint = mbs.AddObject(eii.ObjectJointSpherical(name=name,markerNumbers=[mBody0,mBody1], 
                                                    constrainedAxes=constrainedAxes,
             visualization=eii.VObjectJointSpherical(show=show, jointRadius=jointRadius, color=color) ))

    return oJoint



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateGenericJoint(mbs, name='', bodyNumbers=[None, None], 
                                 position=[], 
                                 rotationMatrixAxes=np.eye(3), 
                                 constrainedAxes=[1,1,1, 1,1,1], 
                                 useGlobalFrame=True,
                                 offsetUserFunction=0, offsetUserFunction_t=0,
                                 show=True, axesRadius=0.1, axesLength=0.4, color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """Create generic joint between two bodies; definition of joint position (position) and axes (rotationMatrixAxes) in global coordinates (useGlobalFrame=True) or in local coordinates of body0 (useGlobalFrame=False), where rotationMatrixAxes is an additional rotation to body0; all markers, markerRotation and other quantities are automatically computed

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        bodyNumbers: a list of two body numbers (ObjectIndex) to be connected; must be rigid body or ground object; alternatively, MarkerIndex (Rigid) can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
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
        mbs.CreateGenericJoint(bodyNumbers=[oGround, b0], position=[5.5,0,0],
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

    [p0, A0, p1, A1, mBody0, mBody1, pJoint] = JointPreCheckCalcBodyMarkers(where, mbs, name, bodyNumbers, position, show, useGlobalFrame)
        
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
        mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=bodyNumbers[0], localHT=exu.HT(rotation=MR0, translation=pJ0)))
        MR0 = np.eye(3)
    if mBody1 is None:
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=bodyNumbers[1], localHT=exu.HT(rotation=MR1, translation=pJ1)))
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
                                       bodyNumbers=[None, None], 
                                       localPosition0 = [0.,0.,0.],
                                       localPosition1 = [0.,0.,0.], 
                                       distance=None, 
                                       bodyOrNodeList=[None, None],
                                       bodyList=[None, None],
                                       show=True, drawSize=-1., color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """Create distance joint between two bodies; definition of joint positions in local coordinates of bodies or nodes; if distance=None, it is computed automatically from reference length; all markers are automatically computed

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        bodyNumbers: a list of two body numbers (ObjectIndex) to be connected; alternatively, MarkerIndex can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
        localPosition0: local position (as 3D list or numpy array) on body0, if not a node or marker number
        localPosition1: local position (as 3D list or numpy array) on body1, if not a node or marker number
        distance: if None, distance is computed from reference position of bodies or nodes; if not None, this distance is prescribed between the two positions; if distance = 0, it will create a SphericalJoint as this case is not possible with a DistanceConstraint
        bodyOrNodeList: alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
        bodyList: DEPRECATED
        show: if True, connector visualization is drawn
        drawSize: general drawing size of node
        color: color of connector

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
        mbs.CreateDistanceConstraint(bodyNumbers=[oGround, b0],
                                     localPosition0 = [6.5,1,0],
                                     localPosition1 = [0.5,0,0],
                                     distance=None, #automatically computed
                                     drawSize=0.06)
        mbs.CreateDistanceConstraint(bodyOrNodeList=[b0, n1],
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
    internBodyNodeMarkerList = ProcessBodyNodeMarkerLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where, bodyList)
        
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


    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name
        
    if isinstance(internBodyNodeMarkerList[0], exudyn.ObjectIndex):
        mBody0 = mbs.AddMarker(eii.MarkerBodyPosition(name=mName0,bodyNumber=internBodyNodeMarkerList[0], localPosition=localPosition0))
    else:
        mBody0 = mbs.AddMarker(eii.MarkerNodePosition(name=mName0,nodeNumber=internBodyNodeMarkerList[0]))

    if isinstance(internBodyNodeMarkerList[1], exudyn.ObjectIndex):
        mBody1 = mbs.AddMarker(eii.MarkerBodyPosition(name=mName1,bodyNumber=internBodyNodeMarkerList[1], localPosition=localPosition1))
    else:
        mBody1 = mbs.AddMarker(eii.MarkerNodePosition(name=mName1,nodeNumber=internBodyNodeMarkerList[1]))
        
    if IsNone(distance): #automatically compute distance
        
        if isinstance(internBodyNodeMarkerList[0], exudyn.ObjectIndex):
            p0 = mbs.GetObjectOutputBody(internBodyNodeMarkerList[0],exudyn.OutputVariableType.Position,
                                         localPosition=localPosition0, configuration=exudyn.ConfigurationType.Reference)
        else:
            p0 = mbs.GetNodeOutput(internBodyNodeMarkerList[0],exudyn.OutputVariableType.Position, configuration=exudyn.ConfigurationType.Reference)
            
        if isinstance(internBodyNodeMarkerList[1], exudyn.ObjectIndex):
            p1 = mbs.GetObjectOutputBody(internBodyNodeMarkerList[1],exudyn.OutputVariableType.Position,
                                         localPosition=localPosition1, configuration=exudyn.ConfigurationType.Reference)
        else:
            p1 = mbs.GetNodeOutput(internBodyNodeMarkerList[1],exudyn.OutputVariableType.Position, configuration=exudyn.ConfigurationType.Reference)
        
        distance = np.linalg.norm(np.array(p1)-p0)
    
    if distance != 0:
        oJoint = mbs.AddObject(eii.ObjectConnectorDistance(name=name,markerNumbers=[mBody0,mBody1], distance=distance,
                 visualization=eii.VObjectConnectorDistance(show=show, drawSize=drawSize, color=color) ))
    else:
        #VERY SPECIAL case, which should help to resolve problems if distance=0 is used ... 
        exu.Print('WARNING: CreateDistanceConstraint called with distance=0; creating SphericalJoint instead')
        constrainedAxes = [1,1,1]
        if isinstance(internBodyNodeMarkerList[0], exudyn.ObjectIndex):
            if '2D' in mbs.GetObject(internBodyNodeMarkerList[0])['objectType']:
                constrainedAxes[2] = 0
        if isinstance(internBodyNodeMarkerList[1], exudyn.ObjectIndex):
            if '2D' in mbs.GetObject(internBodyNodeMarkerList[1])['objectType']:
                constrainedAxes[2] = 0
        oJoint = mbs.AddObject(eii.SphericalJoint(name=name,markerNumbers=[mBody0,mBody1], 
                                                  constrainedAxes=constrainedAxes,
                                                  visualization=eii.VSphericalJoint(show=show, jointRadius=0.5*drawSize, color=color) ))
        

    return oJoint



#NOTE: could be added in future for CreateCoordinateConstraint:
#  bodyOrNodeList: alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateCoordinateConstraint(mbs, name='', 
                                        bodyNumbers=[None, None], 
                                        coordinates=[None, None], 
                                        offset = 0.,
                                        factor1 = 1.,
                                        velocityLevel = False,
                                        offsetUserFunction = 0,
                                        offsetUserFunction_t = 0,
                                        show=True, drawSize=-1., color=exudyn.graphics.color.default, factorValue1=None) -> exudyn.ObjectIndex:
    """Create coordinate constraint for two bodies, or body on ground; markers and NodePointGround are automatically created when needed

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        bodyNumbers: a list of two body numbers (ObjectIndex) to be constrained
        coordinates: a list of two coordinates for the respective bodies (in case of ground, it shall be None)
        offset: an fixed offset between the two coordinate values
        factor1: an additional factor multiplied with coordinate value1 used in algebraic equation, to enable (e.g. gear) ratio between coordinates
        velocityLevel: If true: connector constrains velocities (only works for ODE2 coordinates!); offset is used between velocities; if True, the offsetUserFunction_t is considered and offsetUserFunction is ignored
        offsetUserFunction: a Python function which defines the time-dependent offset; see description in CoordinateConstraint
        offsetUserFunction_t: time derivative of offsetUserFunction; needed for velocity level constraints; see description in CoordinateConstraint
        show: if True, connector visualization is drawn
        drawSize: general drawing size of node
        color: color of connector
        factorValue1: deprecated name of factor1

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
        mbs.CreateCoordinateConstraint(bodyNumbers=[None, b0],
                                       coordinates=[None, 0]) #constrains X-coordinate
        #constrain Y-coordinate of b0 to Z-coordinate of m1:
        mbs.CreateCoordinateConstraint(bodyNumbers=[b0, m1],
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
        
    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)
            
        if not isinstance(bodyNumbers, list) or len(bodyNumbers) != 2:
            RaiseTypeError(where=where, argumentName='bodyNumbers', received = bodyNumbers, expectedType = 'list of 2 body numbers')
        if not isinstance(coordinates, list) or len(coordinates) != 2:
            RaiseTypeError(where=where, argumentName='coordinates', received = coordinates, expectedType = 'list of 2 coordinate indices of the respective bodies')

        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsValidRealInt(drawSize):
            RaiseTypeError(where=where, argumentName='drawSize', received = drawSize, expectedType = ExpectedType.Real)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)


    mNames = ['','']
    if name != '':
        mNames[0] = 'Marker0:'+name
        mNames[1] = 'Marker1:'+name

    #loop over both bodies to find nodes
    # nodeNumbers = [None,None]
    markerNumbers = [None,None]
    firstBodyIsNone = False

    errStr = 'ERROR in ' + where + ': '

    for i, body in enumerate(bodyNumbers):
        coordinate = coordinates[i]
        if body is not None and not isinstance(body, exudyn.ObjectIndex):
            raise ValueError(errStr+f'bodyNumber {body} is no valid ObjectIndex')

        if body is None or mbs.GetObject(body)['objectType'] == 'Ground':
            #use ground
            if i == 1 and firstBodyIsNone:
                raise ValueError(errStr+'one of the two bodyNumbers must be a valid ObjectIndex, but received:'+str(bodyNumbers))

            nPointGround = mbs.AddNode(eii.NodePointGround(visualization=eii.VNodePointGround(show=False)))
            markerNumbers[i] = mbs.AddMarker(eii.MarkerNodeCoordinate(name=mNames[i],nodeNumber=nPointGround, 
                                                                      coordinate=0))
            firstBodyIsNone = True
        else:
            if not isinstance(body, exudyn.ObjectIndex):
                raise ValueError(errStr+f'bodyNumber {body} is no valid ObjectIndex')
            if not IsInteger(coordinate):
                raise ValueError(errStr+f'coordinates[{i}] = {coordinate} is no valid coordinate index')
            
            #get node
            if int(body) >= mbs.systemData.NumberOfObjects():
                raise ValueError(errStr+f'bodyNumber {body} is not available in MainSystem')
            
            objectDict = mbs.GetObject(body)
            if 'nodeNumber' in objectDict:
                nodeNumbers = [objectDict['nodeNumber']]
            else:
                nodeNumbers = objectDict['nodeNumbers']
            
            coordinateOffset = 0
            for node in nodeNumbers:
                nodeLTG = mbs.systemData.GetNodeLTGODE2(node)
                coordinateOffset += len(nodeLTG)
                if coordinate < coordinateOffset:
                    markerNumbers[i] = mbs.AddMarker(eii.MarkerNodeCoordinate(name=mNames[i],nodeNumber=node, 
                                                                    coordinate=coordinates[i]))
            if markerNumbers[i] is None:
                raise ValueError(errStr+f'bodyNumber {body}: requested nodal coordinate {coordinate} not available')

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
def MainSystemCreateSphereSphereContact(mbs, name='', bodyNumbers=[None, None], 
                                       localPosition0 = [0.,0.,0.], localPosition1 = [0.,0.,0.], 
                                       spheresRadii = [-1,-1], isHollowSphere1 = False,
                                       dynamicFriction = 0., frictionProportionalZone = 1e-3,
                                       contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1,
                                       constantPullOffForce = 0, contactPlasticityRatio = 0, adhesionCoefficient = 0, adhesionExponent = 1,
                                       restitutionCoefficient = 1, minimumImpactVelocity = 0,
                                       impactModel = 0,
                                       dataInitialCoordinates = [0,0,0,0],
                                       activeConnector=True,
                                       bodyOrNodeList=[None, None], 
                                       show=False, color=exudyn.graphics.color.default) -> exudyn.ObjectIndex:
    """Create penalty-based sphere-sphere contact between two rigid bodies, mass points (if friction coefficient is zero) or according nodes; the contact is based on ObjectContactSphereSphere; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        bodyNumbers: a list of object numbers for sphere0 and sphere1; Note that if body is a mass point, friction due to rolling is not accounted for!
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
        impactModel: number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!
        dataInitialCoordinates: a list of four values for initialization of the data node, used for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused
        activeConnector: flag to activate or deactivate the connector
        bodyOrNodeList: alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
        show: if True, connector visualization is drawn
        color: color of connector

    Returns:
        :ObjectIndex: returns index of created joint
    """
    where = 'MainSystem.CreateSphereSphereContact(...)'
    internBodyNodeMarkerList = ProcessBodyNodeMarkerLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where)

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
        if not IsValidInt(impactModel) or impactModel < 0 or impactModel > 2:
            RaiseTypeError(where=where, argumentName='impactModel', received = impactModel, expectedType = 'expected type=int, in range [0,2]')

        if not IsVector(dataInitialCoordinates, 4):
            RaiseTypeError(where=where, argumentName='dataInitialCoordinates', received = dataInitialCoordinates, expectedType = ExpectedType.Vector, dim=4)

        if not IsValidBool(activeConnector):
            RaiseTypeError(where=where, argumentName='activeConnector', received = activeConnector, expectedType = ExpectedType.Bool)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)

    
    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name
        
    NewMarkerBody = eii.MarkerBodyRigid if dynamicFriction != 0 else eii.MarkerBodyPosition
    NewMarkerNode = eii.MarkerNodeRigid if dynamicFriction != 0 else eii.MarkerNodePosition
        
    if isinstance(internBodyNodeMarkerList[0], exudyn.ObjectIndex):
        mBody0 = mbs.AddMarker(NewMarkerBody(name=mName0,bodyNumber=internBodyNodeMarkerList[0], localPosition=localPosition0))
    else:
        mBody0 = mbs.AddMarker(NewMarkerNode(name=mName0,nodeNumber=internBodyNodeMarkerList[0]))

    if isinstance(internBodyNodeMarkerList[1], exudyn.ObjectIndex):
        mBody1 = mbs.AddMarker(NewMarkerBody(name=mName1,bodyNumber=internBodyNodeMarkerList[1], localPosition=localPosition1))
    else:
        mBody1 = mbs.AddMarker(NewMarkerNode(name=mName1,nodeNumber=internBodyNodeMarkerList[1]))
    
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
def MainSystemCreateSphereQuadContact(mbs, name='', bodyNumbers=[None, None], 
                                       localPosition0 = [0.,0.,0.], sphereRadius = 0,
                                       quadPoints = exudyn.Vector3DList([[0,0,0],[1,0,0],[1,1,0],[0,1,0]]),
                                       includeEdges = 15, dynamicFriction = 0., frictionProportionalZone = 1e-3,
                                       contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1,
                                       restitutionCoefficient = 1, minimumImpactVelocity = 0,
                                       impactModel = 0,
                                       dataInitialCoordinates = [0,0,0,0],
                                       activeConnector=True,
                                       bodyOrNodeList=[None, None], 
                                       localPosition1 = [0.,0.,0.], 
                                       show=False, color=exudyn.graphics.color.default, radiusSphere=None) -> dict:
    """Create penalty-based sphere-quad contact between two rigid bodies, mass points or according nodes; the contact is based on two ObjectContactSphereTriangle; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        bodyNumbers: a list of object numbers for sphere (0) and quad (1); Note that if body is a mass point, friction due to rolling is not accounted for!
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
        impactModel: number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!
        dataInitialCoordinates: a list of four values for initialization of the data node, used for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused
        activeConnector: flag to activate or deactivate the connector
        bodyOrNodeList: alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
        localPosition1: local position (as 3D list or numpy array) of quad1 on body1; this is usually not needed and adds simply an offset to the quad coordinates
        show: if True, connector visualization is drawn
        color: color of connector
        radiusSphere: deprecated name of sphereRadius

    Returns:
        :dict: dictionary containing oContact0 and oContact1 with ObjectIndex of each contact object
    """
    if radiusSphere is not None: #the old name of the argument (#2814)
        DeprecatedArgument('radiusSphere', '1.12.258', 2031, use='sphereRadius', function='MainSystem.CreateSphereQuadContact')
        sphereRadius = radiusSphere
    where = 'MainSystem.CreateSphereQuadContact(...)'
    internBodyNodeMarkerList = ProcessBodyNodeMarkerLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where)

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

            RaiseTypeError(where=where, argumentName='restitutionCoefficient', received = restitutionCoefficient, expectedType = ExpectedType.Real)
        if not IsValidURealInt(minimumImpactVelocity):
            RaiseTypeError(where=where, argumentName='minimumImpactVelocity', received = minimumImpactVelocity, expectedType = ExpectedType.Real)
        if not IsValidInt(impactModel) or impactModel < 0 or impactModel > 2:
            RaiseTypeError(where=where, argumentName='impactModel', received = impactModel, expectedType = ExpectedType.Real)

        if not IsVector(dataInitialCoordinates, 4):
            RaiseTypeError(where=where, argumentName='dataInitialCoordinates', received = dataInitialCoordinates, expectedType = ExpectedType.Vector, dim=4)

        if not IsValidBool(activeConnector):
            RaiseTypeError(where=where, argumentName='activeConnector', received = activeConnector, expectedType = ExpectedType.Bool)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)

    
    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name

    #new marker for sphere can be Position or Rigid
    NewMarkerBody = eii.MarkerBodyRigid if dynamicFriction != 0 else eii.MarkerBodyPosition
    NewMarkerNode = eii.MarkerNodeRigid if dynamicFriction != 0 else eii.MarkerNodePosition
        
    if isinstance(internBodyNodeMarkerList[0], exudyn.ObjectIndex):
        mBody0 = mbs.AddMarker(NewMarkerBody(name=mName0,bodyNumber=internBodyNodeMarkerList[0], localPosition=localPosition0))
    else:
        mBody0 = mbs.AddMarker(NewMarkerNode(name=mName0,nodeNumber=internBodyNodeMarkerList[0]))

    if isinstance(internBodyNodeMarkerList[1], exudyn.ObjectIndex):
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=internBodyNodeMarkerList[1], localPosition=localPosition1))
    else:
        mBody1 = mbs.AddMarker(eii.MarkerNodeRigid(name=mName1,nodeNumber=internBodyNodeMarkerList[1]))

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
def MainSystemCreateSphereTriangleContact(mbs, name='', bodyNumbers=[None, None], 
                                       localPosition0 = [0.,0.,0.], sphereRadius = 0,
                                       trianglePoints = exudyn.Vector3DList([[0,0,0],[1,0,0],[0,1,0]]),
                                       includeEdges = 7, dynamicFriction = 0., frictionProportionalZone = 1e-3,
                                       contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1,
                                       restitutionCoefficient = 1, minimumImpactVelocity = 0,
                                       impactModel = 0,
                                       dataInitialCoordinates = [0,0,0,0],
                                       activeConnector=True,
                                       bodyOrNodeList=[None, None], 
                                       localPosition1 = [0.,0.,0.], 
                                       show=False, color=exudyn.graphics.color.default, radiusSphere=None) -> exudyn.ObjectIndex:
    """Create penalty-based sphere-triangle contact between two rigid bodies, mass points or according nodes; the contact is based on ObjectContactSphereTriangle; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems

    Args:
        mbs: the MainSystem where joint and markers shall be created
        name: name string for joint; markers get Marker0:name and Marker1:name
        bodyNumbers: a list of object numbers for sphere (0) and triangle (1); Note that if body is a mass point, friction due to rolling is not accounted for!
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
        impactModel: number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!
        dataInitialCoordinates: a list of four values for initialization of the data node, used for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused
        activeConnector: flag to activate or deactivate the connector
        bodyOrNodeList: alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
        localPosition1: local position (as 3D list or numpy array) of triangle1 on body1; this is usually not needed and adds simply an offset to the triangle coordinates
        show: if True, connector visualization is drawn
        color: color of connector
        radiusSphere: deprecated name of sphereRadius

    Returns:
        :ObjectIndex: returns index of created joint
    """
    if radiusSphere is not None: #the old name of the argument (#2814)
        DeprecatedArgument('radiusSphere', '1.12.258', 2031, use='sphereRadius', function='MainSystem.CreateSphereTriangleContact')
        sphereRadius = radiusSphere
    where = 'MainSystem.CreateSphereTriangleContact(...)'
    internBodyNodeMarkerList = ProcessBodyNodeMarkerLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where)

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

            RaiseTypeError(where=where, argumentName='restitutionCoefficient', received = restitutionCoefficient, expectedType = ExpectedType.Real)
        if not IsValidURealInt(minimumImpactVelocity):
            RaiseTypeError(where=where, argumentName='minimumImpactVelocity', received = minimumImpactVelocity, expectedType = ExpectedType.Real)
        if not IsValidInt(impactModel) or impactModel < 0 or impactModel > 2:
            RaiseTypeError(where=where, argumentName='impactModel', received = impactModel, expectedType = ExpectedType.Real)

        if not IsVector(dataInitialCoordinates, 4):
            RaiseTypeError(where=where, argumentName='dataInitialCoordinates', received = dataInitialCoordinates, expectedType = ExpectedType.Vector, dim=4)

        if not IsValidBool(activeConnector):
            RaiseTypeError(where=where, argumentName='activeConnector', received = activeConnector, expectedType = ExpectedType.Bool)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)

    
    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name
        
    if isinstance(internBodyNodeMarkerList[0], exudyn.ObjectIndex):
        mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=internBodyNodeMarkerList[0], localPosition=localPosition0))
    else:
        mBody0 = mbs.AddMarker(eii.MarkerNodeRigid(name=mName0,nodeNumber=internBodyNodeMarkerList[0]))

    if isinstance(internBodyNodeMarkerList[1], exudyn.ObjectIndex):
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=internBodyNodeMarkerList[1], localPosition=localPosition1))
    else:
        mBody1 = mbs.AddMarker(eii.MarkerNodeRigid(name=mName1,nodeNumber=internBodyNodeMarkerList[1]))
    
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

            if np.linalg.norm(v) > 0:
                #links:
                if linkRoundness < 1:
                    axis0 = Normalize(v)
                    axis2 = np.cross(axis0, parentAxis)
                    axis1 = -np.cross(axis0, axis2)
                    lenV = np.linalg.norm(v) #will always have some extension
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
                                           superElementRigidMarkersOffsets=None,
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
        superElementRigidMarkersOffsets: if not None, adds local offsets to the created SuperElementRigid markers; if N boundaries are defined in the femInterface, a N x 3 list or np.array sets an offset for each added marker; the order of the offsets follows the order in [boundaryName0, boundaryName1, ...] used when setting up the femInterface
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
        if not IsValidBool(showMarkers):
            RaiseTypeError(where=where, argumentName='showMarkers', received=showMarkers, expectedType=ExpectedType.Bool)
        # verbose
        if not IsValidBool(verbose):
            RaiseTypeError(where=where, argumentName='verbose', received=verbose, expectedType=ExpectedType.Bool)
        # check whether eigenmodes have been computed
        if femInterface.modeBasis == {}: # this would not be empty if free-free or HCB modes were computed
            raise ValueError(errStr + 'The given femInterface does not hold modes which are needed for component mode synthesis. Use e.g. "femInterface.ComputeHurtyCraigBamptonModes()" before creating an FFRF object.')
        # check whether boundaries are present
        freeEigenmodes = False
        if femInterface.nodeSets == []:
            freeEigenmodes = True
            if verbose:
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
    # define the offsets of the markers if there are any markers
    if nMarkers != 0:
        if IsNotNone(superElementRigidMarkersOffsets):
            if not exudyn.__useExudynFast:
                # when offsets are given, we must make several checks
                # always make a np.array that has two dimensions: nMarkers x 3
                if isinstance(superElementRigidMarkersOffsets, np.ndarray):
                    if superElementRigidMarkersOffsets.ndim == 1: # only has one dimension, so add the first
                        superElementRigidMarkersOffsets = superElementRigidMarkersOffsets[np.newaxis, :]
                else:
                    if not any(isinstance(el, list) for el in superElementRigidMarkersOffsets): # no elements in the list is a list
                        superElementRigidMarkersOffsets = np.array([superElementRigidMarkersOffsets]) # make a np.array with two dimensions
                    else: # we already have a list of lists
                        superElementRigidMarkersOffsets = np.array(superElementRigidMarkersOffsets)
                    if superElementRigidMarkersOffsets.shape[0] != nMarkers:
                        raise ValueError(errStr + 'Number of rows in np.array "superElementRigidMarkersOffsets" must be number of interfaces.')
                    if superElementRigidMarkersOffsets.shape[1] != 3:
                        raise ValueError(errStr + 'Number of columns in np.array "superElementRigidMarkersOffsets" must be 3: local reference position offset(s).')
        else: # if no offsets are given, just set them zero
            superElementRigidMarkersOffsets = np.zeros([nMarkers, 3])
    # use the eigenmodes for component mode synthessis
    cms = ObjectFFRFreducedOrderInterface(femInterface)
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
                                                               offset=superElementRigidMarkersOffsets[i]))
            returnDict[markerName] = marker # add the marker number to the dict under the key of the marker name
            if verbose:
                exu.Print('Added super element rigid body marker "' + markerName + '" that is marker number ' + str(marker) + ' in the MainSystem.')
    return returnDict




#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateForce(mbs,
                name = '',   
                bodyNumber = None,
                loadVector = [0.,0.,0.], 
                localPosition = [0.,0.,0.], 
                bodyFixed = False,
                loadVectorUserFunction = 0,
                show = True) -> exudyn.LoadIndex:
    """helper function to create force applied to given body

    Args:
        mbs: the MainSystem where items are created
        name: name string for object
        bodyNumber: body number (ObjectIndex) at which the force is applied to
        loadVector: force vector (as 3D list or numpy array)
        localPosition: local position (as 3D list or numpy array) where force is applied
        bodyFixed: if True, the force is corotated with the body; else, the force is global
        loadVectorUserFunction: A Python function f(mbs, t, load)->loadVector which defines the time-dependent load and replaces loadVector in every time step; the arg load is the static loadVector
        show: if True, load is drawn

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
        f0=mbs.CreateForce(bodyNumber=b0, loadVector=[100,0,0],
                           localPosition=[0,0,0])
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    #error checks:        
    if not exudyn.__useExudynFast:
        where='MainSystem.CreateForce(...)'
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsVector(localPosition, 3):
            RaiseTypeError(where=where, argumentName='localPosition', received = localPosition, expectedType = ExpectedType.Vector, dim=3)

        # if not IsValidObjectIndex(bodyNumber):
            # RaiseTypeError(where=where, argumentName='bodyNumber', received = bodyNumber, expectedType = ExpectedType.ObjectIndex)
        if not IsValidObjectIndex(bodyNumber):
            if not isinstance(bodyNumber, exudyn.MarkerIndex): #also accept marker
                RaiseTypeError(where=where, argumentName='bodyNumber', received = bodyNumber, expectedType = 'ObjectIndex or MarkerIndex')
            elif np.linalg.norm(localPosition) != 0: #for marker, localPosition must be zero!
                RaiseTypeError(where=where, argumentName='localPosition', received = localPosition, expectedType = '[0,0,0]')

        if not IsVector(loadVector, 3):
            RaiseTypeError(where=where, argumentName='loadVector', received = loadVector, expectedType = ExpectedType.Vector, dim=3)
    
        if not IsValidRealInt(bodyFixed):
            RaiseTypeError(where=where, argumentName='bodyFixed', received = bodyFixed, expectedType = ExpectedType.Bool)
        
        # if not IsUserFunction(loadVectorUserFunction):
        #     RaiseTypeError(where=where, argumentName='loadVectorUserFunction', received = loadVectorUserFunction, expectedType = ExpectedType.UserFunction)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
    
    markerNumber = bodyNumber if isinstance(bodyNumber, exudyn.MarkerIndex) else None

    if markerNumber is None:
        if bodyFixed:
            markerNumber = mbs.AddMarker(eii.MarkerBodyRigid(bodyNumber=bodyNumber, localPosition=localPosition))
        else:
            markerNumber = mbs.AddMarker(eii.MarkerBodyPosition(bodyNumber=bodyNumber, localPosition=localPosition))
        
    loadNumber = mbs.AddLoad(eii.LoadForceVector(markerNumber=markerNumber, 
                                                 loadVector=loadVector,
                                                 bodyFixed=bodyFixed, 
                                                 loadVectorUserFunction=loadVectorUserFunction))

    return loadNumber


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@extends(exudyn.MainSystem)
def MainSystemCreateTorque(mbs,
                name = '',
                bodyNumber = None,
                loadVector = [0.,0.,0.], 
                localPosition = [0.,0.,0.], 
                bodyFixed = False,
                loadVectorUserFunction = 0,
                show = True) -> exudyn.LoadIndex:
    """helper function to create torque applied to given body

    Args:
        mbs: the MainSystem where items are created
        name: name string for object
        bodyNumber: body number (ObjectIndex) at which the torque is applied to
        loadVector: torque vector (as 3D list or numpy array)
        localPosition: local position (as 3D list or numpy array) where torque is applied
        bodyFixed: if True, the torque is corotated with the body; else, the torque is global
        loadVectorUserFunction: A Python function f(mbs, t, load)->loadVector which defines the time-dependent load and replaces loadVector in every time step; the arg load is the static loadVector
        show: if True, load is drawn

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
        f0=mbs.CreateTorque(bodyNumber=b0, loadVector=[0,100,0])
        mbs.Assemble()
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        simulationSettings.timeIntegration.numberOfSteps = 1000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings = simulationSettings)
    """
    #error checks:        
    if not exudyn.__useExudynFast:
        where='MainSystem.CreateTorque(...)'
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsVector(loadVector, 3):
            RaiseTypeError(where=where, argumentName='loadVector', received = loadVector, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(localPosition, 3):
            RaiseTypeError(where=where, argumentName='localPosition', received = localPosition, expectedType = ExpectedType.Vector, dim=3)

        # if not IsValidObjectIndex(bodyNumber):
            # RaiseTypeError(where=where, argumentName='bodyNumber', received = bodyNumber, expectedType = ExpectedType.ObjectIndex)

        if not IsValidObjectIndex(bodyNumber):
            if not isinstance(bodyNumber, exudyn.MarkerIndex): #also accept marker
                RaiseTypeError(where=where, argumentName='bodyNumber', received = bodyNumber, expectedType = 'ObjectIndex or MarkerIndex')
            elif np.linalg.norm(localPosition) != 0: #for marker, localPosition must be zero!
                RaiseTypeError(where=where, argumentName='localPosition', received = localPosition, expectedType = '[0,0,0]')
    
        if not IsValidRealInt(bodyFixed):
            RaiseTypeError(where=where, argumentName='bodyFixed', received = bodyFixed, expectedType = ExpectedType.Bool)
        # if not IsUserFunction(loadVectorUserFunction):
        #     RaiseTypeError(where=where, argumentName='loadVectorUserFunction', received = loadVectorUserFunction, expectedType = ExpectedType.UserFunction)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
    
    markerNumber = bodyNumber if isinstance(bodyNumber, exudyn.MarkerIndex) else mbs.AddMarker(eii.MarkerBodyRigid(bodyNumber=bodyNumber, localPosition=localPosition))
    
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
        try:
            p0list = mbs.GetMarkerOutput(markerNumber=positionOrMarker,
                                         variableType=exudyn.OutputVariableType.Position, 
                                         configuration=exudyn.ConfigurationType.Reference)
            p0list = list(p0list)
        except exudyn.ExudynError:
            p0list = [0,0,0] #this was just a trial, otherwise initialize with zeros (e.g. for special objects where this does not work)
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


@extends(exudyn.MainSystem)
def DrawSystemGraph(mbs, showLoads=True, showSensors=True, useItemNames = False, 
                    useItemTypes = False, addItemTypeNames=True, multiLine=True, fontSizeFactor=1., 
                    layoutDistanceFactor=3., layoutIterations=100, showLegend = True, tightLayout = True, 
                    showGraph = True, addItemData = False, addAnnotations = False) -> list:
    """helper function which draws system graph of a MainSystem (mbs); several options let adjust the appearance of the graph; the graph visualization uses randomizer, which results in different graphs after every run!

    Args:
        mbs: MainSystem to be operated with
        showLoads: toggle appearance of loads in mbs
        showSensors: toggle appearance of sensors in mbs
        useItemNames: if True, object names are shown instead of basic object types (Node, Load, ...)
        useItemTypes: if True, object type names (MassPoint, JointRevolute, ...) are shown instead of basic object types (Node, Load, ...); Note that Node, Object, is omitted at the beginning of itemName (as compared to the reference manual); item classes become clear from the legend
        addItemTypeNames: if True, type nymes (Node, Load, etc.) are added
        multiLine: if True, labels are multiline, improving readability; ignored if showGraph = False
        fontSizeFactor: use this factor to scale fonts, allowing to fit larger graphs on the screen with values < 1
        showLegend: shows legend for different item types
        layoutDistanceFactor: this factor influences the arrangement of labels; larger distance values lead to circle-like results
        layoutIterations: more iterations lead to better arrangement of the layout, but need more time for larger systems (use 1000-10000 to get good results)
        tightLayout: if True, uses matplotlib plt.tight_layout() which may raise warning
        showGraph: if True, graph is plotted with matplotlib
        addItemData: if True, specific data is added to the graph nodes, to be used for deeper analysis of system graphs
        addAnnotations: add data node graphs (not shown), except for graphics data, item numbers, names and types (which are already available in graph data or edges)

    Returns:
        :[Any, Any, Any]: returns [networkx, G, items] with nx being networkx, G the graph and item what is returned by nx.draw_networkx_labels(...)
    """
    
    try:
        #all imports are part of anaconda (e.g. anaconda 5.2.0, python 3.6.5)
        #import numpy as np
        import networkx as nx #for generating graphs and graph arrangement
        import matplotlib.pyplot as plt #for drawing
    except ImportError as e:
        raise ImportError("numpy, networkx and matplotlib required for DrawSystemGraph(...)") from e
    except :
        exudyn.Print("DrawSystemGraph(...): unexpected error during import of numpy, networkx and matplotlib")
        raise
    
    excludeAnnotations = ['V',
                          'nodeNumber', 'objectNumber', 'bodyNumber', 'markerNumber', 
                          'loadNumber', 'sensorNumber',
                          'nodeType','objectType','markerType','loadType','sensorType',
                          'name',
                          ]
    def GetAnnotations(itemDict):
        annotations = {}
        for key, value in itemDict.items():
            exclude = False
            for startWith in excludeAnnotations:
                if key.startswith(startWith):
                    exclude = True
                    break
            if not exclude:
                annotations[key] = value
        return annotations
                
    
    itemColors = {'Node':'red', 'Object':'skyblue', 'Oconnector':'dodgerblue', 'Ojoint':'dodgerblue', 'Ocontact':'dodgerblue', #turqoise, skyblue
                      'Marker': 'orange', 'Load': 'mediumorchid', 
                      'Sensor': 'forestgreen'} #https://matplotlib.org/examples/color/named_colors.html
    
    itemColorMap=[]     #color per item
    itemNames=[]        #name per item 
    itemTypes=[]        #name per item 
    edgeColorMap=[]
    nodesToItems=[]     #maps mbs-node numbers to item numbers
    markersToItems=[]   #maps mbs-marker numbers to item numbers
    objectsToItems=[]   #maps mbs-object numbers to item numbers
    loadsToItems=[]     #maps mbs-load numbers to item numbers
    sensorsToItems=[]   #maps mbs-sensor numbers to item numbers
    
    objectNodeColor = 'navy' #color for edges between nodes and objects, to be highlighted
            
    G = nx.Graph()
    
    #showLegend = False
    #addItemTypeNames = False #Object, Node, ... not added but legend added
    # if useItemTypes or useItemNames:
    #     showLegend = True

    sLineBreak = ''
    if multiLine and showGraph:
        sLineBreak = '-\n'

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
    itemType = 'Node'
    n = mbs.systemData.NumberOfNodes()
    for i in range(n):
        item = mbs.GetNode(i)
        itemName=itemType+str(i)
    
        nodeName = 'Node'
        if item['nodeType'].find('Ground') != -1:
            nodeName = nodeName + 'Ground'
    
        if useItemNames:
            itemName=item['name'] #+str(i)
        elif useItemTypes:
            itemName=item['nodeType']+str(i)
            if addItemTypeNames:
                itemName = nodeName + sLineBreak + itemName

            if sLineBreak != '':
                itemName=itemName.replace('Node'+sLineBreak+'Rigid','NodeRigid'+sLineBreak)
                itemName=itemName.replace('Node'+sLineBreak+'Generic','NodeGeneric'+sLineBreak)
    
        G.add_node(itemName)
        if addItemData:
            G.nodes[itemName].update({'type':itemType + item['nodeType'],
                                      'basicType': itemType,
                                      'ID': i,
                                      'name': item['name'],
                                      })
        if addAnnotations:
            G.nodes[itemName].update({'annotations': GetAnnotations(item)})

        nodesToItems += [len(itemColorMap)]
        itemColorMap += [itemColors[itemType]]
        itemNames += [itemName]
        itemTypes += [itemType]
    
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
    #add markers without edges
    itemType = 'Marker'
    n = mbs.systemData.NumberOfMarkers()
    for i in range(n):
        item = mbs.GetMarker(i)
        itemName=itemType+str(i)
        if useItemNames:
            itemName=item['name']#+str(i)
        elif useItemTypes:
            itemName=item['markerType']+str(i)
            if addItemTypeNames:
                itemName = 'Marker' + sLineBreak + itemName

            if sLineBreak != '':
                itemName=itemName.replace('Marker'+sLineBreak+'Body','MarkerBody'+sLineBreak)
                itemName=itemName.replace('Marker'+sLineBreak+'Object','MarkerObject'+sLineBreak)
                itemName=itemName.replace('Marker'+sLineBreak+'Node','MarkerNode'+sLineBreak)
                itemName=itemName.replace('Marker'+sLineBreak+'SuperElement','MarkerSuperElement'+sLineBreak)
                itemName=itemName.replace('Marker'+sLineBreak+'Kinematic','MarkerKinematic'+sLineBreak)
    
        G.add_node(itemName) #attributes: size, weight, ...
        if addItemData:
            G.nodes[itemName].update({'type':itemType + item['markerType'],
                                      'basicType': itemType,
                                      'ID': i,
                                      'name': item['name'],
                                      })
        if addAnnotations:
            G.nodes[itemName].update({'annotations': GetAnnotations(item)})
        markersToItems += [len(itemColorMap)]
        itemColorMap += [itemColors[itemType]]
        itemNames += [itemName]
        itemTypes += [itemType]
    
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
    itemType = 'Object'
    n = mbs.systemData.NumberOfObjects()
    for i in range(n):
        objectType = itemType
        item = mbs.GetObject(i)
        if item['objectType'].find('Connector') != -1:
            objectType = 'Oconnector'
        elif item['objectType'].find('Contact') != -1:
            objectType = 'Ocontact'
        elif item['objectType'].find('Joint') != -1:
            objectType = 'Ojoint'
        itemName=objectType+str(i)
    
        if useItemNames:
            itemName=item['name']#+str(i)
        elif useItemTypes:
            itemName=item['objectType']+str(i)
            if addItemTypeNames:
                itemName = 'Object' + sLineBreak + itemName
            
            if sLineBreak != '':
                itemName=itemName.replace('Object'+sLineBreak+'Joint','ObjectJoint'+sLineBreak)
                itemName=itemName.replace('Object'+sLineBreak+'Mass','ObjectMass'+sLineBreak)
                itemName=itemName.replace('Object'+sLineBreak+'Beam','ObjectBeam'+sLineBreak)
                itemName=itemName.replace('Object'+sLineBreak+'ANCF','ObjectANCF'+sLineBreak)
                itemName=itemName.replace('Object'+sLineBreak+'Contact','ObjectContact'+sLineBreak)
                itemName=itemName.replace('Object'+sLineBreak+'Connector','ObjectConnector'+sLineBreak)
                itemName=itemName.replace('Object'+sLineBreak+'Rigid','ObjectRigid'+sLineBreak)
                itemName=itemName.replace('Object'+sLineBreak+'FFRFr','ObjectFFRF'+sLineBreak+'r')
            
        G.add_node(itemName) #attributes: size, weight, ...
        if addItemData:
            G.nodes[itemName].update({'type':itemType + item['objectType'],
                                      'basicType': itemType,
                                      'ID': i,
                                      'name': item['name'],
                                      })
        if addAnnotations:
            G.nodes[itemName].update({'annotations': GetAnnotations(item)})
        objectsToItems += [len(itemColorMap)]
        itemNames += [itemName]
        itemTypes += [itemType]
        itemColorMap += [itemColors[objectType]]
    
    #    objectColor = ''
        #for objects: add edges to nodes
        nodeNumbers = []
        if 'nodeNumber' in item:
            nodeNumbers += [item['nodeNumber']]
        if 'nodeNumbers' in item:
            nodeNumbers += item['nodeNumbers']
    
        for j in range(len(nodeNumbers)):
            nodeNumbers[j] = int(nodeNumbers[j])
    
        for j in nodeNumbers:
            if j != exudyn.InvalidIndex(): #for RigidBodySpringDamper
                edge = (itemNames[objectsToItems[i]],itemNames[nodesToItems[j]])
                G.add_edge(*edge)
                # if addItemData:
                #     G.edges[edge].update({'type':itemType + item['objectType'],
                #                           'basicType': itemType,
                #                           'ID': i,
                #                           'name': item['name'],
                #                           })
                if showGraph:
                    G.edges[edge].update({'color':objectNodeColor})
    
        #for connectors, contact, joint: add edges to these objects
        markerNumbers = []
        if 'markerNumbers' in item: #should only be markerNumbers ...
            markerNumbers += item['markerNumbers']
    
        for j in range(len(markerNumbers)):
            markerNumbers[j] = int(markerNumbers[j])
    
        for j in markerNumbers:
            edge = (itemNames[objectsToItems[i]],itemNames[markersToItems[j]])
            G.add_edge(*edge)
            # if addItemData:
            #     G.edges[edge].update({'type':itemType + item['objectType'],
            #                           'basicType': itemType,
            #                           'ID': i,
            #                           'name': item['name'],
            #                           })
            if showGraph:
                G.edges[edge].update({'color':itemColors['Oconnector']})


            
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
    #now add only edges for markers:
    itemType = 'Marker'
    n = mbs.systemData.NumberOfMarkers()
    for i in range(n):
        objectType = itemType
        item = mbs.GetMarker(i)
    
        #for node markers:
        nodeNumbers = []
        if 'nodeNumber' in item:
            nodeNumbers += [item['nodeNumber']]
       
        for j in range(len(nodeNumbers)):
            nodeNumbers[j] = int(nodeNumbers[j])
    
        for j in nodeNumbers:
            edge = (itemNames[markersToItems[i]],itemNames[nodesToItems[j]])
            G.add_edge(*edge)
            # if addItemData:
            #     G.edges[edge].update({'type':itemType + item['markerType'],
            #                           'basicType': itemType,
            #                           'ID': i,
            #                           'name': item['name'],
            #                           })
            if showGraph:
                G.edges[edge].update({'color':'orange'})
    
        #for object markers:
        objectNumbers = []
        if 'objectNumber' in item: objectNumbers += [item['objectNumber']]
        if 'bodyNumber' in item: objectNumbers += [item['bodyNumber']]
       
        for j in range(len(objectNumbers)):
            objectNumbers[j] = int(objectNumbers[j])
    
        for j in objectNumbers:
            edge = (itemNames[markersToItems[i]],itemNames[objectsToItems[j]])
            G.add_edge(*edge)
            # if addItemData:
            #     G.edges[edge].update({'type':itemType + item['markerType'],
            #                           'basicType': itemType,
            #                           'ID': i,
            #                           'name': item['name'],
            #                           })
            if showGraph:
                G.edges[edge].update({'color':'orange'})
            
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
    #add loads
    if showLoads:
        itemType = 'Load'
        n = mbs.systemData.NumberOfLoads()
        for i in range(n):
            item = mbs.GetLoad(i)
            itemName=itemType+str(i)
            if useItemNames:
                itemName=item['name']#+str(i)
            elif useItemTypes:
                itemName=item['loadType']+str(i)
                if addItemTypeNames:
                    itemName = 'Load' + sLineBreak + itemName

                if sLineBreak != '':
                    itemName=itemName.replace('Load'+sLineBreak+'Mass','LoadMass'+sLineBreak)

        
            G.add_node(itemName) #attributes: size, weight, ...
            if addItemData:
                G.nodes[itemName].update({'type':itemType + item['loadType'],
                                          'basicType': itemType,
                                          'ID': i,
                                          'name': item['name'],
                                          })
            if addAnnotations:
                G.nodes[itemName].update({'annotations': GetAnnotations(item)})

            loadsToItems += [len(itemColorMap)]
            itemColorMap += [itemColors[itemType]]
            itemNames += [itemName]
            itemTypes += [itemType]
    
            markerNumbers = [int(item['markerNumber'])]
            
            for j in markerNumbers:
                G.add_edge(itemNames[loadsToItems[i]],itemNames[markersToItems[j]], color=itemColors['Load'])
    
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
    #add sensors
    if showSensors:
        itemType = 'Sensor'
        n = mbs.systemData.NumberOfSensors() #only available for Exudyn version >= 1.0.15
        for i in range(n):
            item = mbs.GetSensor(i)
            itemName=itemType+str(i)
            if useItemNames:
                itemName=item['name']#+str(i)
            elif useItemTypes:
                itemName=item['sensorType']+str(i)
                if addItemTypeNames:
                    itemName = 'Sensor' + sLineBreak + itemName
        
            G.add_node(itemName) #attributes: size, weight, ...
            if addItemData:
                G.nodes[itemName].update({'type':itemType + item['sensorType'],
                                          'basicType': itemType,
                                          'ID': i,
                                          'name': item['name'],
                                          })
            if addAnnotations:
                G.nodes[itemName].update({'annotations': GetAnnotations(item)})
            sensorsToItems += [len(itemColorMap)]
            itemColorMap += [itemColors[itemType]]
            itemNames += [itemName]
            itemTypes += [itemType]
    
            #for object sensors:
            objectNumbers = []
            if 'objectNumber' in item: objectNumbers += [item['objectNumber']]
            if 'bodyNumber' in item: objectNumbers += [item['bodyNumber']]
           
            for j in range(len(objectNumbers)):
                objectNumbers[j] = int(objectNumbers[j])
        
            for j in objectNumbers:
                G.add_edge(itemNames[sensorsToItems[i]],itemNames[objectsToItems[j]],color=itemColors[itemType])

            #for node sensors:
            nodeNumbers = []
            if 'nodeNumber' in item: nodeNumbers += [int(item['nodeNumber'])]
                   
            for j in nodeNumbers:
                G.add_edge(itemNames[sensorsToItems[i]],itemNames[nodesToItems[j]],color=itemColors[itemType])

    items = None #only assigned if shown
    if showGraph:
        plt.clf()
    
        if showLegend:
            legendColors = {'Node':'red', 'Object':'skyblue', 'Object(Connector)':'dodgerblue', 
                          'Marker': 'orange', 'Load': 'mediumorchid', 
                          'Sensor': 'forestgreen'} 
            #f = plt.figure(1)
            #ax = f.add_subplot(1,1,1)
            for label in legendColors:
                plt.plot([0],[0],linewidth=8,color=legendColors[label],label=label)
            
            fontSizeLegend = 10
            if fontSizeFactor > 1: #do not make font size smaller!
                fontSizeLegend *= fontSizeFactor
            plt.legend(fontsize=fontSizeLegend)
    
        
        #now get out the right sorting of colors ...
        edgeColorMap = []
        edgeWidths = []
        edges=G.edges()
        for item in edges.items(): 
            edgeColorMap += [item[1]['color']] #color is in item[1], which is a dictionary ...
            edgeWidth = 2
            if item[1]['color'] == objectNodeColor: #object-node should be emphasized
                edgeWidth = 4
            edgeWidths += [edgeWidth]
        
        pos = nx.drawing.spring_layout(G, scale=0.5, k=layoutDistanceFactor*1/np.sqrt(G.size()), 
                                       threshold = 1e-5, iterations = layoutIterations)
        nx.draw_networkx_nodes(G, pos, node_size=1)
        nx.draw_networkx_edges(G, pos, edge_color=edgeColorMap, width=edgeWidths)#width=2)
        
        #reproduce what draw_networkx_labels does, allowing different colors for nodes
        #check: https://networkx.github.io/documentation/stable/_modules/networkx/drawing/nx_pylab.html
        items = nx.draw_networkx_labels(G, pos, font_size=10*fontSizeFactor, clip_on=False, #clip at plot boundary
                                        bbox=dict(facecolor='skyblue', edgecolor='black', 
                                                  boxstyle='round,pad=0.1', lw=10*fontSizeFactor)) #lw is border size (no effect?)
    
        
        #now assign correct colors:
        for i in range(len(itemNames)):
            currentColor = itemColorMap[i]
            itemType = itemTypes[i]
            boxStyle = 'round,pad=0.2'
            fontSize = 10*fontSizeFactor
            if itemType == 'Object':
                boxStyle = 'round,pad=0.2'
                fontSize = 12*fontSizeFactor
            if itemType == 'Node':  
                boxStyle = 'square,pad=0.1'
                fontSize = 10*fontSizeFactor
            if itemType == 'Marker':  
                boxStyle = 'square,pad=0.1'
                fontSize = 8*fontSizeFactor
        
            items[itemNames[i]].set_bbox(dict(facecolor=currentColor,  
                  edgecolor=currentColor, boxstyle=boxStyle))
            items[itemNames[i]].set_fontsize(fontSize)
        
        plt.axis('off') #do not show frame, because usually some nodes are very close to frame ...
        if tightLayout:
            plt.tight_layout()
        plt.margins(x=0.1*fontSizeFactor, y=0.1*fontSizeFactor) #larger margin, to avoid clipping of long texts
        plt.draw() #force redraw after colors have changed
    
    return [nx, G, items]


#bind all functions marked with @extends (in this module, plot, solver, utilities, interactive) to their classes
install()
