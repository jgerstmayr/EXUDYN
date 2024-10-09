#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  This module provides an extension interface to the C++ class MainSystem;
#           MainSystem is extended by Python interface functions to easily create
#           bodies and point masses without the need to create an according node and
#           connectors and joints without the need to create markers.
#           For activation of Python extension in the mainSystem, 
#           just write: \\ \texttt{import exudyn.mainSystemExtensions} or 
#           import \texttt{exudyn.utilities}
#
# Author:   Johannes Gerstmayr
# Date:     2023-05-07 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#import exudyn #does not work out of exudyn.__init__.py
import exudyn as exu
import exudyn.plot
import exudyn.solver
import exudyn.interactive
import exudyn.graphics
import exudyn.utilities

from exudyn.rigidBodyUtilities import GetRigidBodyNode, ComputeOrthonormalBasis, \
    RotationMatrix2EulerParameters, AngularVelocity2EulerParameters_t, RotationMatrix2RotXYZ, AngularVelocity2RotXYZ_t, \
    RotationMatrix2RotationVector

import exudyn.itemInterface as eii 
from exudyn.advancedUtilities import RaiseTypeError, IsVector, ExpectedType, IsValidObjectIndex, IsValidNodeIndex, \
                                    IsValidRealInt, IsValidPRealInt, IsValidURealInt, IsIntVector, \
                                    IsValidBool, IsSquareMatrix, IsNone, IsNotNone

import numpy as np
import copy

#exudyn.Print('WARNING: mainSystemInterface is available only for testing; it may fail on certain architectures; use with care')


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#add helpful Python extensions for MainSystem, regarding creation of bodies, point masses, connectors and joints

#internal function: do some pre-checks and calculations for joint
def JointPreCheckCalc(where, mbs, name, bodyNumbers, position, show, useGlobalFrame):
    #perform some checks:
    if not exudyn.__useExudynFast:
        if not isinstance(bodyNumbers, list) or len(bodyNumbers) != 2:
            RaiseTypeError(where=where, argumentName='bodyNumbers', received = bodyNumbers, expectedType = 'list of 2 body numbers')
        if not IsValidObjectIndex(bodyNumbers[0]):
            RaiseTypeError(where=where, argumentName='bodyNumbers[0]', received = bodyNumbers[0], expectedType = ExpectedType.ObjectIndex)
        if not IsValidObjectIndex(bodyNumbers[1]):
            RaiseTypeError(where=where, argumentName='bodyNumbers[1]', received = bodyNumbers[1], expectedType = ExpectedType.ObjectIndex)
    
        if not IsVector(position, 3):
            RaiseTypeError(where=where, argumentName='position', received = position, expectedType = ExpectedType.Vector, dim=3)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsValidBool(useGlobalFrame):
            RaiseTypeError(where=where, argumentName='useGlobalFrame', received = useGlobalFrame, expectedType = ExpectedType.Bool)
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

    p0 = mbs.GetObjectOutputBody(bodyNumbers[0],exudyn.OutputVariableType.Position,
                                 localPosition=[0,0,0],
                                 configuration=exudyn.ConfigurationType.Reference)
    A0 = mbs.GetObjectOutputBody(bodyNumbers[0],exudyn.OutputVariableType.RotationMatrix,
                                 localPosition=[0,0,0],
                                 configuration=exudyn.ConfigurationType.Reference).reshape((3,3))
    p1 = mbs.GetObjectOutputBody(bodyNumbers[1],exudyn.OutputVariableType.Position,
                                 localPosition=[0,0,0],
                                 configuration=exudyn.ConfigurationType.Reference)
    A1 = mbs.GetObjectOutputBody(bodyNumbers[1],exudyn.OutputVariableType.RotationMatrix,
                                 localPosition=[0,0,0],
                                 configuration=exudyn.ConfigurationType.Reference).reshape((3,3))

    return [p0, A0, p1, A1] 

#internal function, which checks bodyList and bodyOrNodeList and returns appropriate bodyOrNodeList
def ProcessBodyNodeLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where, bodyList):
    if not exudyn.__useExudynFast:
        if not isinstance(bodyList, list) or len(bodyList) != 2:
            RaiseTypeError(where=where, argumentName='bodyList', received = bodyList, expectedType = 'list of 2 body numbers')
        if not isinstance(bodyNumbers, list) or len(bodyNumbers) != 2:
            RaiseTypeError(where=where, argumentName='bodyNumbers', received = bodyNumbers, expectedType = 'list of 2 body numbers')

    causingArgName = 'bodyOrNodeList'
    if IsNotNone(bodyNumbers[0]) or IsNotNone(bodyNumbers[1]):
        bodyOrNodeList = [bodyNumbers[0],bodyNumbers[1]] #flat copy, but otherwise would lead to change of args (mutable args!)
        causingArgName = 'bodyNumbers'
    elif IsNotNone(bodyList[0]) or IsNotNone(bodyList[1]):
        bodyOrNodeList = [bodyList[0],bodyList[1]] #flat copy, but otherwise would lead to change of args (mutable args!)
        causingArgName = 'bodyList'

    if not exudyn.__useExudynFast:
        if not isinstance(bodyOrNodeList, list) or len(bodyOrNodeList) != 2:
            RaiseTypeError(where=where, argumentName='bodyOrNodeList', received = bodyOrNodeList, expectedType = 'list of 2 body or node numbers')
    
        if not (isinstance(bodyOrNodeList[0], exudyn.ObjectIndex) or (isinstance(bodyOrNodeList[0], exudyn.NodeIndex) and localPosition0==[0.,0.,0.])):
            RaiseTypeError(where=where, argumentName=''+causingArgName+'[0]', received = bodyOrNodeList[0], 
                           expectedType = 'expected either ObjectIndex or NodeIndex and localPosition0=[0.,0.,0.]')
            
        if not (isinstance(bodyOrNodeList[1], exudyn.ObjectIndex) or (isinstance(bodyOrNodeList[1], exudyn.NodeIndex) and localPosition1==[0.,0.,0.])):
            RaiseTypeError(where=where, argumentName=''+causingArgName+'[1]', received = bodyOrNodeList[1], 
                           expectedType = 'expected either ObjectIndex or NodeIndex and localPosition1=[0.,0.,0.]')
    
    return bodyOrNodeList



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: helper function to create a ground object, using arguments of ObjectGround; this function is mainly added for consistency with other mainSystemExtensions
#**input: 
#  mbs: the MainSystem where items are created
#  name: name string for object
#  referencePosition: reference coordinates for point node (always a 3D vector, no matter if 2D or 3D mass)
#  referenceRotationMatrix: reference rotation matrix for rigid body node (always 3D matrix, no matter if 2D or 3D body)
#  graphicsDataList: list of GraphicsData for optional ground visualization
#  graphicsDataUserFunction: a user function graphicsDataUserFunction(mbs, itemNumber)->BodyGraphicsData (list of GraphicsData), which can be used to draw user-defined graphics; this is much slower than regular GraphicsData
#  color: color of node
#  show: True: show ground object; 
#**output: ObjectIndex; returns ground object index 
#**belongsTo: MainSystem
#**example:
# import exudyn as exu
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# 
# ground=mbs.CreateGround(referencePosition = [2,0,0],
#                         graphicsDataList = [exu.graphics.CheckerBoard(point=[0,0,0], normal=[0,1,0],size=4)])
# 
def MainSystemCreateGround(mbs,
                           name = '',   
                           referencePosition = [0.,0.,0.],
                           referenceRotationMatrix = np.eye(3),
                           graphicsDataList = [],
                           graphicsDataUserFunction = 0,
                           show = True): 

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
#**function: helper function to create 2D or 3D mass point object and node, using arguments as in NodePoint and MassPoint
#**input: 
#  mbs: the MainSystem where items are created
#  name: name string for object, node is 'Node:'+name
#  referencePosition: reference coordinates for point node (always a 3D vector, no matter if 2D or 3D mass)
#  initialDisplacement: initial displacements for point node (always a 3D vector, no matter if 2D or 3D mass)
#  initialVelocity: initial velocities for point node (always a 3D vector, no matter if 2D or 3D mass)
#  physicsMass: mass of mass point
#  gravity: gravity vevtor applied (always a 3D vector, no matter if 2D or 3D mass)
#  graphicsDataList: list of GraphicsData for optional mass visualization
#  drawSize: general drawing size of node
#  color: color of node
#  show: True: if graphicsData list is empty, node is shown, otherwise body is shown; otherwise, nothing is shown
#  create2D: if True, create NodePoint2D and MassPoint2D
#  returnDict: if False, returns object index; if True, returns dict of all information on created object and node
#**output: Union[dict, ObjectIndex]; returns mass point object index or dict with all data on request (if returnDict=True)
#**belongsTo: MainSystem
#**example:
# import exudyn as exu
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# 
# b0=mbs.CreateMassPoint(referencePosition = [0,0,0],
#                        initialVelocity = [2,5,0],
#                        physicsMass = 1, gravity = [0,-9.81,0],
#                        drawSize = 0.5, color=exu.graphics.color.blue)
# 
# mbs.Assemble()
# simulationSettings = exu.SimulationSettings() #takes currently set values or default values
# simulationSettings.timeIntegration.numberOfSteps = 1000
# simulationSettings.timeIntegration.endTime = 2
# mbs.SolveDynamic(simulationSettings = simulationSettings)
def MainSystemCreateMassPoint(mbs,
                           name = '',
                           referencePosition = [0.,0.,0.],
                           initialDisplacement = [0.,0.,0.],
                           initialVelocity = [0.,0.,0.],
                           physicsMass=0,
                           gravity = [0.,0.,0.],
                           graphicsDataList = [],
                           drawSize = -1,
                           color =  [-1.,-1.,-1.,-1.],
                           show = True, 
                           create2D = False, 
                           returnDict = False): 

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

    if not create2D:
        nodeNumber = mbs.AddNode(eii.NodePoint(name = nodeName,
                         referenceCoordinates = referencePosition,
                         initialCoordinates=initialDisplacement,
                         initialVelocities=initialVelocity,
                         visualization = eii.VNodePoint(show = show and (graphicsDataList == []), drawSize = drawSize, color = color),
                         ))
        bodyNumber = mbs.AddObject(eii.MassPoint(name = name,
                                                physicsMass=physicsMass,
                                                nodeNumber = nodeNumber,
                                                visualization = eii.VMassPoint(show = graphicsDataList != [], 
                                                                           graphicsData = graphicsDataList) ))
    else:
        nodeNumber = mbs.AddNode(eii.NodePoint2D(name = nodeName,
                         referenceCoordinates = referencePosition[0:2],
                         initialCoordinates=initialDisplacement[0:2],
                         initialVelocities=initialVelocity[0:2],
                         visualization = eii.VNodePoint2D(show = show and (graphicsDataList == []), drawSize = drawSize, color = color),
                         ))
        bodyNumber = mbs.AddObject(eii.MassPoint2D(name = name, 
                                                physicsMass=physicsMass,
                                                nodeNumber = nodeNumber,
                                                visualization = eii.VMassPoint(show = graphicsDataList != [], 
                                                                           graphicsData = graphicsDataList) ))
        
    if returnDict:
        rDict = {'nodeNumber':nodeNumber, 'bodyNumber': bodyNumber}
    
    if list(gravity) != [0.,0.,0.]: #        if NormL2(gravity) != 0.:
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
#**function: helper function to create 3D (or 2D) rigid body object and node; all quantities are global (angular velocity, etc.)
#**input: 
#  mbs: the MainSystem where items are created
#  name: name string for object, node is 'Node:'+name
#  referencePosition: reference position vector for rigid body node (always a 3D vector, no matter if 2D or 3D body)
#  referenceRotationMatrix: reference rotation matrix for rigid body node (always 3D matrix, no matter if 2D or 3D body)
#  initialVelocity: initial translational velocity vector for node (always a 3D vector, no matter if 2D or 3D body)
#  initialAngularVelocity: initial angular velocity vector for node (always a 3D vector, no matter if 2D or 3D body)
#  initialDisplacement: initial translational displacement vector for node (always a 3D vector, no matter if 2D or 3D body); these displacements are deviations from reference position, e.g. for a finite element node [None: unused]
#  initialRotationMatrix: initial rotation provided as matrix (always a 3D matrix, no matter if 2D or 3D body); this rotation is superimposed to reference rotation [None: unused]
#  inertia: an instance of class RigidBodyInertia, see rigidBodyUtilities; may also be from derived class (InertiaCuboid, InertiaMassPoint, InertiaCylinder, ...)
#  gravity: gravity vevtor applied (always a 3D vector, no matter if 2D or 3D mass)
#  graphicsDataList: list of GraphicsData for rigid body visualization; use exudyn.graphics functions to create GraphicsData for basic solids
#  graphicsDataUserFunction: a user function graphicsDataUserFunction(mbs, itemNumber)->BodyGraphicsData (list of GraphicsData), which can be used to draw user-defined graphics; this is much slower than regular GraphicsData
#  drawSize: general drawing size of node
#  color: color of node
#  show: True: if graphicsData list is empty, node is shown, otherwise body is shown; False: nothing is shown
#  create2D: if True, create NodeRigidBody2D and ObjectRigidBody2D
#  returnDict: if False, returns object index; if True, returns dict of all information on created object and node
#**output: Union[dict, ObjectIndex]; returns rigid body object index (or dict with 'nodeNumber', 'objectNumber' and possibly 'loadNumber' and 'markerBodyMass' if returnDict=True)
#**belongsTo: MainSystem
#**example:
# import exudyn as exu
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# 
# b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, 
#                                                  sideLengths=[1,0.1,0.1]),
#                          referencePosition = [1,0,0],
#                          initialVelocity = [2,5,0],
#                          initialAngularVelocity = [5,0.5,0.7],
#                          gravity = [0,-9.81,0],
#                          graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1], 
#                                                                       color=exu.graphics.color.red)])
# 
# mbs.Assemble()
# simulationSettings = exu.SimulationSettings() #takes currently set values or default values
# simulationSettings.timeIntegration.numberOfSteps = 1000
# simulationSettings.timeIntegration.endTime = 2
# 
# mbs.SolveDynamic(simulationSettings = simulationSettings)
def MainSystemCreateRigidBody(mbs,
                           name = '',   
                           referencePosition = [0.,0.,0.],
                           referenceRotationMatrix = np.eye(3),
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
                           returnDict = False): 

    #error checks:        
    if not exudyn.__useExudynFast:
        where='MainSystem.CreateRigidBody(...)'
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
                             visualization = VNodeClass(show = show and (graphicsDataList == []), drawSize = drawSize, color = color)
                             )
        nodeNumber = mbs.AddNode(nodeItem)
        bodyNumber = mbs.AddObject(eii.ObjectRigidBody(name=name, physicsMass=inertia.mass, physicsInertia=inertia.GetInertia6D(), 
                                                       physicsCenterOfMass=inertia.com,
                                                       nodeNumber=nodeNumber, 
                                                       visualization=eii.VObjectRigidBody(show = graphicsDataList != [], 
                                                                                          graphicsDataUserFunction = graphicsDataUserFunction,
                                                                                          graphicsData=graphicsDataList)))
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
                             visualization = eii.VNodeRigidBody2D(show = show and (graphicsDataList == []), drawSize = drawSize, color = color)
                             )
        nodeNumber = mbs.AddNode(nodeItem)
        bodyNumber = mbs.AddObject(eii.ObjectRigidBody2D(name=name, physicsMass=inertia.mass, physicsInertia=inertia.GetInertia6D()[2],
                                                       #physicsCenterOfMass=inertia.com,
                                                       nodeNumber=nodeNumber,
                                                       visualization=eii.VObjectRigidBody(show = graphicsDataList != [],
                                                                                          graphicsDataUserFunction=graphicsDataUserFunction,
                                                                                          graphicsData=graphicsDataList)))
        
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
#**function: helper function to create SpringDamper connector, using arguments from ObjectConnectorSpringDamper; similar interface as CreateDistanceConstraint(...), see there for for further information
#**input: 
#  mbs: the MainSystem where items are created
#  name: name string for connector; markers get Marker0:name and Marker1:name
#  bodyNumbers: a list of two body numbers (ObjectIndex) to be connected
#  localPosition0: local position (as 3D list or numpy array) on body0, if not a node number
#  localPosition1: local position (as 3D list or numpy array) on body1, if not a node number
#  referenceLength: if None, length is computed from reference position of bodies or nodes; if not None, this scalar reference length is used for spring
#  stiffness: scalar stiffness coefficient
#  damping: scalar damping coefficient
#  force: scalar additional force applied
#  velocityOffset: scalar offset: if referenceLength is changed over time, the velocityOffset may be changed accordingly to emulate a reference motion
#  springForceUserFunction: a user function springForceUserFunction(mbs, t, itemNumber, deltaL, deltaL\_t, stiffness, damping, force)->float ; this function replaces the internal connector force compuation
#  bodyOrNodeList: alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
#  show: if True, connector visualization is drawn
#  drawSize: general drawing size of connector
#  color: color of connector
#**output: ObjectIndex; returns index of newly created object
#**belongsTo: MainSystem
#**example:
# import exudyn as exu
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# 
# b0 = mbs.CreateMassPoint(referencePosition = [2,0,0],
#                          initialVelocity = [2,5,0],
#                          physicsMass = 1, gravity = [0,-9.81,0],
#                          drawSize = 0.5, color=exu.graphics.color.blue)
# 
# oGround = mbs.AddObject(ObjectGround())
# #add vertical spring
# oSD = mbs.CreateSpringDamper(bodyNumbers=[oGround, b0],
#                              localPosition0=[2,1,0],
#                              localPosition1=[0,0,0],
#                              stiffness=1e4, damping=1e2,
#                              drawSize=0.2)
# 
# mbs.Assemble()
# simulationSettings = exu.SimulationSettings() #takes currently set values or default values
# simulationSettings.timeIntegration.numberOfSteps = 1000
# simulationSettings.timeIntegration.endTime = 2
# SC.visualizationSettings.nodes.drawNodesAsPoint=False
# mbs.SolveDynamic(simulationSettings = simulationSettings)
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
                                 show=True, drawSize=-1, color=exudyn.graphics.color.default):
    #perform some checks:
    where='MainSystem.CreateSpringDamper(...)'
    internBodyNodeList = ProcessBodyNodeLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where, bodyList)
    
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

    
    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name
        
    if isinstance(internBodyNodeList[0], exudyn.ObjectIndex):
        mBody0 = mbs.AddMarker(eii.MarkerBodyPosition(name=mName0,bodyNumber=internBodyNodeList[0], localPosition=localPosition0))
    else:
        mBody0 = mbs.AddMarker(eii.MarkerNodePosition(name=mName0,nodeNumber=internBodyNodeList[0]))

    if isinstance(internBodyNodeList[1], exudyn.ObjectIndex):
        mBody1 = mbs.AddMarker(eii.MarkerBodyPosition(name=mName1,bodyNumber=internBodyNodeList[1], localPosition=localPosition1))
    else:
        mBody1 = mbs.AddMarker(eii.MarkerNodePosition(name=mName1,nodeNumber=internBodyNodeList[1]))
        
    if IsNone(referenceLength): #automatically compute reference length
        
        if isinstance(internBodyNodeList[0], exudyn.ObjectIndex):
            p0 = mbs.GetObjectOutputBody(internBodyNodeList[0],exudyn.OutputVariableType.Position,
                                         localPosition=localPosition0, configuration=exudyn.ConfigurationType.Reference)
        else:
            p0 = mbs.GetNodeOutput(internBodyNodeList[0],exudyn.OutputVariableType.Position, configuration=exudyn.ConfigurationType.Reference)
            
        if isinstance(internBodyNodeList[1], exudyn.ObjectIndex):
            p1 = mbs.GetObjectOutputBody(internBodyNodeList[1],exudyn.OutputVariableType.Position,
                                         localPosition=localPosition1, configuration=exudyn.ConfigurationType.Reference)
        else:
            p1 = mbs.GetNodeOutput(internBodyNodeList[1],exudyn.OutputVariableType.Position, configuration=exudyn.ConfigurationType.Reference)
        
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
#**function: helper function to create CartesianSpringDamper connector, using arguments from ObjectConnectorCartesianSpringDamper
#**input: 
#  mbs: the MainSystem where items are created
#  name: name string for connector; markers get Marker0:name and Marker1:name
#  bodyNumbers: a list of two body numbers (ObjectIndex) to be connected
#  localPosition0: local position (as 3D list or numpy array) on body0, if not a node number
#  localPosition1: local position (as 3D list or numpy array) on body1, if not a node number
#  stiffness: stiffness coefficients (as 3D list or numpy array)
#  damping: damping coefficients (as 3D list or numpy array)
#  offset: offset vector (as 3D list or numpy array)
#  springForceUserFunction: a user function springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset)->[float,float,float] ; this function replaces the internal connector force compuation
#  bodyOrNodeList: alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
#  show: if True, connector visualization is drawn
#  drawSize: general drawing size of connector
#  color: color of connector
#**output: ObjectIndex; returns index of newly created object
#**belongsTo: MainSystem
#**example:
# import exudyn as exu
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# 
# b0 = mbs.CreateMassPoint(referencePosition = [7,0,0],
#                           physicsMass = 1, gravity = [0,-9.81,0],
#                           drawSize = 0.5, color=exu.graphics.color.blue)
# 
# oGround = mbs.AddObject(ObjectGround())
# 
# oSD = mbs.CreateCartesianSpringDamper(bodyNumbers=[oGround, b0],
#                               localPosition0=[7.5,1,0],
#                               localPosition1=[0,0,0],
#                               stiffness=[200,2000,0], damping=[2,20,0],
#                               drawSize=0.2)
# 
# mbs.Assemble()
# simulationSettings = exu.SimulationSettings() #takes currently set values or default values
# simulationSettings.timeIntegration.numberOfSteps = 1000
# simulationSettings.timeIntegration.endTime = 2
# SC.visualizationSettings.nodes.drawNodesAsPoint=False
# 
# mbs.SolveDynamic(simulationSettings = simulationSettings)
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
                                 show=True, drawSize=-1, color=exudyn.graphics.color.default):

    where='MainSystem.CreateCartesianSpringDamper(...)'
    internBodyNodeList = ProcessBodyNodeLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where, bodyList)

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

    
    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name
        
    if isinstance(internBodyNodeList[0], exudyn.ObjectIndex):
        mBody0 = mbs.AddMarker(eii.MarkerBodyPosition(name=mName0,bodyNumber=internBodyNodeList[0], localPosition=localPosition0))
    else:
        mBody0 = mbs.AddMarker(eii.MarkerNodePosition(name=mName0,nodeNumber=internBodyNodeList[0]))

    if isinstance(internBodyNodeList[1], exudyn.ObjectIndex):
        mBody1 = mbs.AddMarker(eii.MarkerBodyPosition(name=mName1,bodyNumber=internBodyNodeList[1], localPosition=localPosition1))
    else:
        mBody1 = mbs.AddMarker(eii.MarkerNodePosition(name=mName1,nodeNumber=internBodyNodeList[1]))
            
    oConnector = mbs.AddObject(eii.ObjectConnectorCartesianSpringDamper(name=name,markerNumbers = [mBody0,mBody1],
                                                                        stiffness = stiffness, damping = damping, offset = offset,
                                                                        springForceUserFunction=springForceUserFunction,
                                                                        visualization=eii.VCartesianSpringDamper(show=show, 
                                                                                      drawSize=drawSize, color=color)
                                                                      ))

    return oConnector

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: helper function to create RigidBodySpringDamper connector, using arguments from ObjectConnectorRigidBodySpringDamper, see there for the full documentation
#**input: 
#  mbs: the MainSystem where items are created
#  name: name string for connector; markers get Marker0:name and Marker1:name
#  bodyNumbers: a list of two body numbers (ObjectIndex) to be connected
#  localPosition0: local position (as 3D list or numpy array) on body0, if not a node number
#  localPosition1: local position (as 3D list or numpy array) on body1, if not a node number
#  stiffness: stiffness coefficients (as 6D matrix or numpy array)
#  damping: damping coefficients (as 6D matrix or numpy array)
#  offset: offset vector (as 6D list or numpy array)
#  rotationMatrixJoint: additional rotation matrix; in case  useGlobalFrame=False, it transforms body0/node0 local frame to joint frame; if useGlobalFrame=True, it transforms global frame to joint frame
#  useGlobalFrame: if False, the rotationMatrixJoint is defined in the local coordinate system of body0
#  intrinsicFormulation: if True, uses intrinsic formulation of Maserati and Morandini, which uses matrix logarithm and is independent of order of markers (preferred formulation); otherwise, Tait-Bryan angles are used for computation of torque, see documentation
#  springForceTorqueUserFunction: a user function springForceTorqueUserFunction(mbs, t, itemNumber, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1, offset)->[float,float,float, float,float,float] ; this function replaces the internal connector force / torque compuation
#  postNewtonStepUserFunction: a special user function postNewtonStepUserFunction(mbs, t, Index itemIndex, dataCoordinates, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1, offset)->[PNerror, recommendedStepSize, data[0], data[1], ...] ; for details, see RigidBodySpringDamper for full docu
#  bodyOrNodeList: alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
#  show: if True, connector visualization is drawn
#  drawSize: general drawing size of connector
#  color: color of connector
#**output: ObjectIndex; returns index of newly created object
#**belongsTo: MainSystem
#**example:
# #coming later
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
                                 intrinsicFormulation=True,
                                 springForceTorqueUserFunction=0,
                                 postNewtonStepUserFunction=0,
                                 bodyOrNodeList=[None, None],
                                 bodyList=[None, None],
                                 show=True, drawSize=-1, color=exudyn.graphics.color.default):

    where='MainSystem.CreateRigidBodySpringDamper(...)'
    internBodyNodeList = ProcessBodyNodeLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where, bodyList)

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


    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name
    
    if isinstance(internBodyNodeList[0], exudyn.ObjectIndex):
        mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=internBodyNodeList[0], localPosition=localPosition0))
        A0 = mbs.GetObjectOutputBody(objectNumber=internBodyNodeList[0],variableType=exudyn.OutputVariableType.RotationMatrix,
                                     localPosition=localPosition0,
                                     configuration=exudyn.ConfigurationType.Reference).reshape((3,3))
    else:
        mBody0 = mbs.AddMarker(eii.MarkerNodePosition(name=mName0,nodeNumber=internBodyNodeList[0]))
        A0 = mbs.GetNodeOutput(nodeNumber=internBodyNodeList[0], variableType=exudyn.OutputVariableType.RotationMatrix,
                               configuration=exudyn.ConfigurationType.Reference).reshape((3,3))

    if isinstance(internBodyNodeList[1], exudyn.ObjectIndex):
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=internBodyNodeList[1], localPosition=localPosition1))
        A1 = mbs.GetObjectOutputBody(objectNumber=internBodyNodeList[1],variableType=exudyn.OutputVariableType.RotationMatrix,
                                     localPosition=localPosition1,
                                     configuration=exudyn.ConfigurationType.Reference).reshape((3,3))
    else:
        mBody1 = mbs.AddMarker(eii.MarkerNodePosition(name=mName1,nodeNumber=internBodyNodeList[1]))
        A1 = mbs.GetNodeOutput(nodeNumber=internBodyNodeList[1], variableType=exudyn.OutputVariableType.RotationMatrix,
                               configuration=exudyn.ConfigurationType.Reference).reshape((3,3))

    # print('A0=',A0)
    # print('A1=',A1)
    if useGlobalFrame:
        #compute joint marker orientations, rotationMatrixAxes represents global frame:
        MR0 = A0.T @ rotationMatrixJoint
        MR1 = A1.T @ rotationMatrixJoint
    else: #transform into global coordinates, then everything works same
        #compute joint marker orientations, rotationMatrixAxes represents local frame:
        MR0 = rotationMatrixJoint
        MR1 = A1.T @ A0 @ rotationMatrixJoint

            
    oConnector = mbs.AddObject(eii.ObjectConnectorRigidBodySpringDamper(name=name,markerNumbers = [mBody0,mBody1],
                                                                        stiffness = stiffness, damping = damping, 
                                                                        offset = offset,
                                                                        rotationMarker0=MR0, 
                                                                        rotationMarker1=MR1,
                                                                        intrinsicFormulation=intrinsicFormulation,
                                                                        springForceTorqueUserFunction=springForceTorqueUserFunction, 
                                                                        postNewtonStepUserFunction=postNewtonStepUserFunction,
                                                                        visualization=eii.VRigidBodySpringDamper(show=show, 
                                                                                      drawSize=drawSize, color=color)
                                                                      ))

    return oConnector



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Create revolute joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed
#**input:
#  mbs: the MainSystem where joint and markers shall be created
#  name: name string for joint; markers get Marker0:name and Marker1:name
#  bodyNumbers: a list of object numbers for body0 and body1; must be rigid body or ground object
#  position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
#  axis: a 3D vector as list or np.array containing the joint axis either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True)
#  useGlobalFrame: if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
#  show: if True, connector visualization is drawn
#  axisRadius: radius of axis for connector graphical representation
#  axisLength: length of axis for connector graphical representation
#  color: color of connector
#**output: [ObjectIndex, MarkerIndex, MarkerIndex]; returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
#**belongsTo: MainSystem
#**example:
# import exudyn as exu
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# 
# b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, 
#                                                  sideLengths=[1,0.1,0.1]),
#                          referencePosition = [3,0,0],
#                          gravity = [0,-9.81,0],
#                          graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1], 
#                                                                       color=exu.graphics.color.steelblue)])
# oGround = mbs.AddObject(ObjectGround())
# mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b0], position=[2.5,0,0], axis=[0,0,1],
#                         useGlobalFrame=True, axisRadius=0.02, axisLength=0.14)
# 
# mbs.Assemble()
# simulationSettings = exu.SimulationSettings() #takes currently set values or default values
# simulationSettings.timeIntegration.numberOfSteps = 1000
# simulationSettings.timeIntegration.endTime = 2
# 
# mbs.SolveDynamic(simulationSettings = simulationSettings)
def MainSystemCreateRevoluteJoint(mbs, name='', bodyNumbers=[None, None], 
                                  position=[], axis=[], useGlobalFrame=True, 
                                  show=True, axisRadius=0.1, axisLength=0.4, color=exudyn.graphics.color.default):
    
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

    [p0, A0, p1, A1] = JointPreCheckCalc(where, mbs, name, bodyNumbers, position, show, useGlobalFrame)
        
    if useGlobalFrame:
        pJoint = copy.copy(position)
        vAxis = copy.copy(axis)
    else: #transform into global coordinates, then everything works same
        pJoint = A0 @ position + p0
        vAxis = A0 @ axis

    #compute joint frame (not unique, only rotation axis must coincide)
    B = ComputeOrthonormalBasis(vAxis) #axis = x-axis
    #interchange z and x axis (needs sign change, otherwise det(A)=-1)
    AJ = np.eye(3)
    AJ[:,0]=-B[:,2]
    AJ[:,1]= B[:,1]
    AJ[:,2]= B[:,0] #axis ==> rotation axis z for revolute joint ... 
    #print(AJ)
    
    #compute joint position and axis in bodyNumber0 / 1 coordinates:
    pJ0 = A0.T @ (np.array(pJoint) - p0)
    pJ1 = A1.T @ (np.array(pJoint) - p1)

    #compute joint marker orientations:
    MR0 = A0.T @ AJ  
    MR1 = A1.T @ AJ  
    
    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name

    mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=bodyNumbers[0], localPosition=pJ0))
    mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=bodyNumbers[1], localPosition=pJ1))
    
    oJoint = mbs.AddObject(eii.ObjectJointRevoluteZ(name=name,markerNumbers=[mBody0,mBody1],
                                                rotationMarker0=MR0,
                                                rotationMarker1=MR1,
             visualization=eii.VRevoluteJointZ(show=show, axisRadius=axisRadius, axisLength=axisLength, color=color) ))

    return [oJoint, mBody0, mBody1]


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Create prismatic joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed
#**input:
#  mbs: the MainSystem where joint and markers shall be created
#  name: name string for joint; markers get Marker0:name and Marker1:name
#  bodyNumbers: a list of object numbers for body0 and body1; must be rigid body or ground object
#  position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
#  axis: a 3D vector as list or np.array containing the joint axis either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True)
#  useGlobalFrame: if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
#  show: if True, connector visualization is drawn
#  axisRadius: radius of axis for connector graphical representation
#  axisLength: length of axis for connector graphical representation
#  color: color of connector
#**output: [ObjectIndex, MarkerIndex, MarkerIndex]; returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
#**belongsTo: MainSystem
#**example:
# import exudyn as exu
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# 
# b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, 
#                                                  sideLengths=[1,0.1,0.1]),
#                          referencePosition = [4,0,0],
#                          initialVelocity = [0,4,0],
#                          gravity = [0,-9.81,0],
#                          graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1], 
#                                                                       color=exu.graphics.color.steelblue)])
# 
# oGround = mbs.AddObject(ObjectGround())
# mbs.CreatePrismaticJoint(bodyNumbers=[oGround, b0], position=[3.5,0,0], axis=[0,1,0], 
#                          useGlobalFrame=True, axisRadius=0.02, axisLength=1)
# 
# mbs.Assemble()
# simulationSettings = exu.SimulationSettings() #takes currently set values or default values
# simulationSettings.timeIntegration.numberOfSteps = 1000
# simulationSettings.timeIntegration.endTime = 2
# 
# mbs.SolveDynamic(simulationSettings = simulationSettings)
def MainSystemCreatePrismaticJoint(mbs, name='', bodyNumbers=[None, None], 
                                  position=[], axis=[], useGlobalFrame=True, 
                                  show=True, axisRadius=0.1, axisLength=0.4, color=exudyn.graphics.color.default):
        
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

    [p0, A0, p1, A1] = JointPreCheckCalc(where, mbs, name, bodyNumbers, position, show, useGlobalFrame)


    if useGlobalFrame:
        pJoint = copy.copy(position)
        vAxis = copy.copy(axis)
    else: #transform into global coordinates, then everything works same
        pJoint = A0 @ position + p0
        vAxis = A0 @ axis

    #compute joint frame (not unique, only rotation axis must coincide)
    AJ = ComputeOrthonormalBasis(vAxis) #axis = x-axis
    
    #compute joint position and axis in bodyNumber0 / 1 coordinates:
    pJ0 = A0.T @ (np.array(pJoint) - p0)
    pJ1 = A1.T @ (np.array(pJoint) - p1)

    #compute joint marker orientations:
    MR0 = A0.T @ AJ  
    MR1 = A1.T @ AJ  
    
    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name

    mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=bodyNumbers[0], localPosition=pJ0))
    mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=bodyNumbers[1], localPosition=pJ1))
    
    oJoint = mbs.AddObject(eii.ObjectJointPrismaticX(name=name,markerNumbers=[mBody0,mBody1],
                                                rotationMarker0=MR0,
                                                rotationMarker1=MR1,
             visualization=eii.VPrismaticJointX(show=show, axisRadius=axisRadius, axisLength=axisLength, color=color) ))

    return [oJoint, mBody0, mBody1]

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Create spherical joint between two bodies; definition of joint position in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers are automatically computed
#**input:
#  mbs: the MainSystem where joint and markers shall be created
#  name: name string for joint; markers get Marker0:name and Marker1:name
#  bodyNumbers: a list of object numbers for body0 and body1; must be mass point, rigid body or ground object
#  position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
#  constrainedAxes: flags, which determines which (global) translation axes are constrained; each entry may only be 0 (=free) axis or 1 (=constrained axis)
#  useGlobalFrame: if False, the point and axis vectors are defined in the local coordinate system of body0
#  show: if True, connector visualization is drawn
#  jointRadius: radius of sphere for connector graphical representation
#  color: color of connector
#**output: [ObjectIndex, MarkerIndex, MarkerIndex]; returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
#**belongsTo: MainSystem
#**example:
# import exudyn as exu
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# 
# b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, 
#                                                  sideLengths=[1,0.1,0.1]),
#                          referencePosition = [5,0,0],
#                          initialAngularVelocity = [5,0,0],
#                          gravity = [0,-9.81,0],
#                          graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1], 
#                                                                       color=exu.graphics.color.orange)])
# oGround = mbs.AddObject(ObjectGround())
# mbs.CreateSphericalJoint(bodyNumbers=[oGround, b0], position=[5.5,0,0], 
#                          useGlobalFrame=True, jointRadius=0.06)
# 
# mbs.Assemble()
# simulationSettings = exu.SimulationSettings() #takes currently set values or default values
# simulationSettings.timeIntegration.numberOfSteps = 1000
# simulationSettings.timeIntegration.endTime = 2
# 
# mbs.SolveDynamic(simulationSettings = simulationSettings)
def MainSystemCreateSphericalJoint(mbs, name='', bodyNumbers=[None, None], 
                                  position=[], constrainedAxes=[1,1,1], useGlobalFrame=True, 
                                  show=True, jointRadius=0.1, color=exudyn.graphics.color.default):
        
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

    [p0, A0, p1, A1] = JointPreCheckCalc(where, mbs, name, bodyNumbers, position, show, useGlobalFrame)

    if useGlobalFrame:
        pJoint = copy.copy(position)
    else: #transform into global coordinates, then everything works same
        pJoint = A0 @ position + p0

    
    #compute joint position and axis in bodyNumber0 / 1 coordinates:
    pJ0 = A0.T @ (np.array(pJoint) - p0)
    pJ1 = A1.T @ (np.array(pJoint) - p1)

    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name

    mBody0 = mbs.AddMarker(eii.MarkerBodyPosition(name=mName0,bodyNumber=bodyNumbers[0], localPosition=pJ0))
    mBody1 = mbs.AddMarker(eii.MarkerBodyPosition(name=mName1,bodyNumber=bodyNumbers[1], localPosition=pJ1))
    
    oJoint = mbs.AddObject(eii.ObjectJointSpherical(name=name,markerNumbers=[mBody0,mBody1], 
                                                    constrainedAxes=constrainedAxes,
             visualization=eii.VObjectJointSpherical(show=show, jointRadius=jointRadius, color=color) ))

    return [oJoint, mBody0, mBody1]



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Create generic joint between two bodies; definition of joint position (position) and axes (rotationMatrixAxes) in global coordinates (useGlobalFrame=True) or in local coordinates of body0 (useGlobalFrame=False), where rotationMatrixAxes is an additional rotation to body0; all markers, markerRotation and other quantities are automatically computed
#**input:
#  mbs: the MainSystem where joint and markers shall be created
#  name: name string for joint; markers get Marker0:name and Marker1:name
#  bodyNumber0: a object number for body0, must be rigid body or ground object
#  bodyNumber1: a object number for body1, must be rigid body or ground object
#  position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
#  rotationMatrixAxes: rotation matrix which defines orientation of constrainedAxes; if useGlobalFrame, this rotation matrix is global, else the rotation matrix is post-multiplied with the rotation of body0, identical with rotationMarker0 in the joint
#  constrainedAxes: flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; each entry may only be 0 (=free) axis or 1 (=constrained axis); ALL constrained Axes are defined relative to reference rotation of body0 times rotation0
#  useGlobalFrame: if False, the position is defined in the local coordinate system of body0, otherwise it is defined in global coordinates
#  offsetUserFunction: a user function offsetUserFunction(mbs, t, itemNumber, offsetUserFunctionParameters)->float ; this function replaces the internal (constant) by a user-defined offset. This allows to realize rheonomic joints and allows kinematic simulation
#  offsetUserFunction_t: a user function offsetUserFunction\_t(mbs, t, itemNumber, offsetUserFunctionParameters)->float ; this function replaces the internal (constant) by a user-defined offset velocity; this function is used instead of offsetUserFunction, if velocityLevel (index2) time integration
#  show: if True, connector visualization is drawn
#  axesRadius: radius of axes for connector graphical representation
#  axesLength: length of axes for connector graphical representation
#  color: color of connector
#**output: [ObjectIndex, MarkerIndex, MarkerIndex]; returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
#**belongsTo: MainSystem
#**example:
# import exudyn as exu
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# 
# b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, 
#                                                  sideLengths=[1,0.1,0.1]),
#                          referencePosition = [6,0,0],
#                          initialAngularVelocity = [0,8,0],
#                          gravity = [0,-9.81,0],
#                          graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1], 
#                                                                       color=exu.graphics.color.orange)])
# oGround = mbs.AddObject(ObjectGround())
# mbs.CreateGenericJoint(bodyNumbers=[oGround, b0], position=[5.5,0,0],
#                        constrainedAxes=[1,1,1, 1,0,0],
#                        rotationMatrixAxes=RotationMatrixX(0.125*pi), #tilt axes
#                        useGlobalFrame=True, axesRadius=0.02, axesLength=0.2)
# 
# mbs.Assemble()
# simulationSettings = exu.SimulationSettings() #takes currently set values or default values
# simulationSettings.timeIntegration.numberOfSteps = 1000
# simulationSettings.timeIntegration.endTime = 2
# 
# mbs.SolveDynamic(simulationSettings = simulationSettings)
def MainSystemCreateGenericJoint(mbs, name='', bodyNumbers=[None, None], 
                                 position=[], 
                                 rotationMatrixAxes=np.eye(3), 
                                 constrainedAxes=[1,1,1, 1,1,1], 
                                 useGlobalFrame=True,
                                 offsetUserFunction=0, offsetUserFunction_t=0,
                                 show=True, axesRadius=0.1, axesLength=0.4, color=exudyn.graphics.color.default):
        
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

    [p0, A0, p1, A1] = JointPreCheckCalc(where, mbs, name, bodyNumbers, position, show, useGlobalFrame)


    if useGlobalFrame:
        pJoint = copy.copy(position)
        #compute joint marker orientations, rotationMatrixAxes represents global frame:
        MR0 = A0.T @ rotationMatrixAxes
        MR1 = A1.T @ rotationMatrixAxes
    else: #transform into global coordinates, then everything works same
        pJoint = A0 @ position + p0
        #compute joint marker orientations, rotationMatrixAxes represents local frame:
        MR0 = copy.copy(rotationMatrixAxes)
        MR1 = A1.T @ A0 @ rotationMatrixAxes

    
    #compute joint position and axis in bodyNumber0 / 1 coordinates:
    pJ0 = A0.T @ (np.array(pJoint) - p0)
    pJ1 = A1.T @ (np.array(pJoint) - p1)

    
    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:'+name
        mName1 = 'Marker1:'+name

    mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=bodyNumbers[0], localPosition=pJ0))
    mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=bodyNumbers[1], localPosition=pJ1))
    
    oJoint = mbs.AddObject(eii.ObjectJointGeneric(name=name,markerNumbers=[mBody0,mBody1],
                                                  constrainedAxes = constrainedAxes,
                                                  rotationMarker0=MR0,
                                                  rotationMarker1=MR1, 
                                                  offsetUserFunction=offsetUserFunction,
                                                  offsetUserFunction_t=offsetUserFunction_t,
             visualization=eii.VObjectJointGeneric(show=show, axesRadius=axesRadius, axesLength=axesLength, color=color) ))

    return [oJoint, mBody0, mBody1]

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Create distance joint between two bodies; definition of joint positions in local coordinates of bodies or nodes; if distance=None, it is computed automatically from reference length; all markers are automatically computed
#**input:
#  mbs: the MainSystem where joint and markers shall be created
#  name: name string for joint; markers get Marker0:name and Marker1:name
#  bodyNumbers: a list of two body numbers (ObjectIndex) to be constrained
#  localPosition0: local position (as 3D list or numpy array) on body0, if not a node number
#  localPosition1: local position (as 3D list or numpy array) on body1, if not a node number
#  distance: if None, distance is computed from reference position of bodies or nodes; if not None, this distance is prescribed between the two positions; if distance = 0, it will create a SphericalJoint as this case is not possible with a DistanceConstraint
#  bodyOrNodeList: alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
#  show: if True, connector visualization is drawn
#  drawSize: general drawing size of node
#  color: color of connector
#**output: [ObjectIndex, MarkerIndex, MarkerIndex]; returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
#**belongsTo: MainSystem
#**example:
# import exudyn as exu
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# 
# b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, 
#                                                   sideLengths=[1,0.1,0.1]),
#                           referencePosition = [6,0,0],
#                           gravity = [0,-9.81,0],
#                           graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1], 
#                                                                       color=exu.graphics.color.orange)])
# m1 = mbs.CreateMassPoint(referencePosition=[5.5,-1,0],
#                          physicsMass=1, drawSize = 0.2)
# n1 = mbs.GetObject(m1)['nodeNumber']
#     
# oGround = mbs.AddObject(ObjectGround())
# mbs.CreateDistanceConstraint(bodyNumbers=[oGround, b0], 
#                              localPosition0 = [6.5,1,0],
#                              localPosition1 = [0.5,0,0],
#                              distance=None, #automatically computed
#                              drawSize=0.06)
# 
# mbs.CreateDistanceConstraint(bodyOrNodeList=[b0, n1], 
#                              localPosition0 = [-0.5,0,0],
#                              localPosition1 = [0.,0.,0.], #must be [0,0,0] for Node
#                              distance=None, #automatically computed
#                              drawSize=0.06)
# 
# mbs.Assemble()
# simulationSettings = exu.SimulationSettings() #takes currently set values or default values
# simulationSettings.timeIntegration.numberOfSteps = 1000
# simulationSettings.timeIntegration.endTime = 2
# 
# mbs.SolveDynamic(simulationSettings = simulationSettings)
def MainSystemCreateDistanceConstraint(mbs, name='', 
                                       bodyNumbers=[None, None], 
                                       localPosition0 = [0.,0.,0.],
                                       localPosition1 = [0.,0.,0.], 
                                       distance=None, 
                                       bodyOrNodeList=[None, None],
                                       bodyList=[None, None],
                                       show=True, drawSize=-1., color=exudyn.graphics.color.default):
    
    where = 'MainSystem.CreateDistanceConstraint(...)'
    internBodyNodeList = ProcessBodyNodeLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where, bodyList)
        
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
        
    if isinstance(internBodyNodeList[0], exudyn.ObjectIndex):
        mBody0 = mbs.AddMarker(eii.MarkerBodyPosition(name=mName0,bodyNumber=internBodyNodeList[0], localPosition=localPosition0))
    else:
        mBody0 = mbs.AddMarker(eii.MarkerNodePosition(name=mName0,nodeNumber=internBodyNodeList[0]))

    if isinstance(internBodyNodeList[1], exudyn.ObjectIndex):
        mBody1 = mbs.AddMarker(eii.MarkerBodyPosition(name=mName1,bodyNumber=internBodyNodeList[1], localPosition=localPosition1))
    else:
        mBody1 = mbs.AddMarker(eii.MarkerNodePosition(name=mName1,nodeNumber=internBodyNodeList[1]))
        
    if IsNone(distance): #automatically compute distance
        
        if isinstance(internBodyNodeList[0], exudyn.ObjectIndex):
            p0 = mbs.GetObjectOutputBody(internBodyNodeList[0],exudyn.OutputVariableType.Position,
                                         localPosition=localPosition0, configuration=exudyn.ConfigurationType.Reference)
        else:
            p0 = mbs.GetNodeOutput(internBodyNodeList[0],exudyn.OutputVariableType.Position, configuration=exudyn.ConfigurationType.Reference)
            
        if isinstance(internBodyNodeList[1], exudyn.ObjectIndex):
            p1 = mbs.GetObjectOutputBody(internBodyNodeList[1],exudyn.OutputVariableType.Position,
                                         localPosition=localPosition1, configuration=exudyn.ConfigurationType.Reference)
        else:
            p1 = mbs.GetNodeOutput(internBodyNodeList[1],exudyn.OutputVariableType.Position, configuration=exudyn.ConfigurationType.Reference)
        
        distance = np.linalg.norm(np.array(p1)-p0)
    
    if distance != 0:
        oJoint = mbs.AddObject(eii.ObjectConnectorDistance(name=name,markerNumbers=[mBody0,mBody1], distance=distance,
                 visualization=eii.VObjectConnectorDistance(show=show, drawSize=drawSize, color=color) ))
    else:
        #VERY SPECIAL case, which should help to resolve problems if distance=0 is used ... 
        exu.Print('WARNING: CreateDistanceConstraint called with distance=0; creating SphericalJoint instead')
        constrainedAxes = [1,1,1]
        if isinstance(internBodyNodeList[0], exudyn.ObjectIndex):
            if '2D' in mbs.GetObject(internBodyNodeList[0])['objectType']:
                constrainedAxes[2] = 0
        if isinstance(internBodyNodeList[1], exudyn.ObjectIndex):
            if '2D' in mbs.GetObject(internBodyNodeList[1])['objectType']:
                constrainedAxes[2] = 0
        oJoint = mbs.AddObject(eii.SphericalJoint(name=name,markerNumbers=[mBody0,mBody1], 
                                                  constrainedAxes=constrainedAxes,
                                                  visualization=eii.VSphericalJoint(show=show, jointRadius=0.5*drawSize, color=color) ))
        

    return [oJoint, mBody0, mBody1]





#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: helper function to create force applied to given body
#**input: 
#  mbs: the MainSystem where items are created
#  name: name string for object
#  bodyNumber: body number (ObjectIndex) at which the force is applied to
#  loadVector: force vector (as 3D list or numpy array)
#  localPosition: local position (as 3D list or numpy array) where force is applied
#  bodyFixed: if True, the force is corotated with the body; else, the force is global
#  loadVectorUserFunction: A Python function f(mbs, t, load)->loadVector which defines the time-dependent load and replaces loadVector in every time step; the arg load is the static loadVector
#  show: if True, load is drawn
#**output: LoadIndex; returns load index
#**belongsTo: MainSystem
#**example:
# import exudyn as exu
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# 
# b0=mbs.CreateMassPoint(referencePosition = [0,0,0],
#                        initialVelocity = [2,5,0],
#                        physicsMass = 1, gravity = [0,-9.81,0],
#                        drawSize = 0.5, color=exu.graphics.color.blue)
#
# f0=mbs.CreateForce(bodyNumber=b0, loadVector=[100,0,0],
#                    localPosition=[0,0,0])
# 
# mbs.Assemble()
# simulationSettings = exu.SimulationSettings() #takes currently set values or default values
# simulationSettings.timeIntegration.numberOfSteps = 1000
# simulationSettings.timeIntegration.endTime = 2
# mbs.SolveDynamic(simulationSettings = simulationSettings)
def MainSystemCreateForce(mbs,
                name = '',   
                bodyNumber = None,
                loadVector = [0.,0.,0.], 
                localPosition = [0.,0.,0.], 
                bodyFixed = False,
                loadVectorUserFunction = 0,
                show = True):

    #error checks:        
    if not exudyn.__useExudynFast:
        where='MainSystem.CreateForce(...)'
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsValidObjectIndex(bodyNumber):
            RaiseTypeError(where=where, argumentName='bodyNumber', received = bodyNumber, expectedType = ExpectedType.ObjectIndex)
        if not IsVector(loadVector, 3):
            RaiseTypeError(where=where, argumentName='loadVector', received = loadVector, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(localPosition, 3):
            RaiseTypeError(where=where, argumentName='localPosition', received = localPosition, expectedType = ExpectedType.Vector, dim=3)
    
        if not IsValidRealInt(bodyFixed):
            RaiseTypeError(where=where, argumentName='bodyFixed', received = bodyFixed, expectedType = ExpectedType.Bool)
        
        # if not IsUserFunction(loadVectorUserFunction):
        #     RaiseTypeError(where=where, argumentName='loadVectorUserFunction', received = loadVectorUserFunction, expectedType = ExpectedType.UserFunction)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
    

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
#**function: helper function to create torque applied to given body
#**input: 
#  mbs: the MainSystem where items are created
#  name: name string for object
#  bodyNumber: body number (ObjectIndex) at which the torque is applied to
#  loadVector: torque vector (as 3D list or numpy array)
#  localPosition: local position (as 3D list or numpy array) where torque is applied
#  bodyFixed: if True, the torque is corotated with the body; else, the torque is global
#  loadVectorUserFunction: A Python function f(mbs, t, load)->loadVector which defines the time-dependent load and replaces loadVector in every time step; the arg load is the static loadVector
#  show: if True, load is drawn
#**output: LoadIndex; returns load index
#**belongsTo: MainSystem
#**example:
# import exudyn as exu
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# 
# b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, 
#                                                  sideLengths=[1,0.1,0.1]),
#                          referencePosition = [1,3,0],
#                          gravity = [0,-9.81,0],
#                          graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1], 
#                                                                       color=exu.graphics.color.red)])
#
# f0=mbs.CreateTorque(bodyNumber=b0, loadVector=[0,100,0])
# 
# mbs.Assemble()
# simulationSettings = exu.SimulationSettings() #takes currently set values or default values
# simulationSettings.timeIntegration.numberOfSteps = 1000
# simulationSettings.timeIntegration.endTime = 2
# mbs.SolveDynamic(simulationSettings = simulationSettings)
def MainSystemCreateTorque(mbs,
                name = '',
                bodyNumber = None,
                loadVector = [0.,0.,0.], 
                localPosition = [0.,0.,0.], 
                bodyFixed = False,
                loadVectorUserFunction = 0,
                show = True):

    #error checks:        
    if not exudyn.__useExudynFast:
        where='MainSystem.CreateTorque(...)'
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsValidObjectIndex(bodyNumber):
            RaiseTypeError(where=where, argumentName='bodyNumber', received = bodyNumber, expectedType = ExpectedType.ObjectIndex)
        if not IsVector(loadVector, 3):
            RaiseTypeError(where=where, argumentName='loadVector', received = loadVector, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(localPosition, 3):
            RaiseTypeError(where=where, argumentName='localPosition', received = localPosition, expectedType = ExpectedType.Vector, dim=3)
    
        if not IsValidRealInt(bodyFixed):
            RaiseTypeError(where=where, argumentName='bodyFixed', received = bodyFixed, expectedType = ExpectedType.Bool)
        # if not IsUserFunction(loadVectorUserFunction):
        #     RaiseTypeError(where=where, argumentName='loadVectorUserFunction', received = loadVectorUserFunction, expectedType = ExpectedType.UserFunction)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received = show, expectedType = ExpectedType.Bool)
    
    
    markerNumber = mbs.AddMarker(eii.MarkerBodyRigid(bodyNumber=bodyNumber, localPosition=localPosition))
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
#RigidBodySpringDamper
#LinearSpringDamper
#TorsionalSpringDamper
#2x rolling disc
#CreateBeamsStraight[2D](...) #ANCF, GE with types?
#CreateBeamsCurved[2D](...)   #ANCF, GE


# #FUTURE:
# #def InitializeFromRestartFile(mbs, simulationSettings, restartFileName, verbose=True):

    
# #needs some extensions; really needed?
# # def AnimateModes(self, *args, **kwargs):
# #     SC = self.GetSystemContainer()
# #     return exudyn.interactive.AnimateModes(SC, self, *args, **kwargs)
# #makes no sense, as it is a class:
# # def InteractiveDialog(self, *args, **kwargs):
# #     return exudyn.interactive.InteractiveDialog(self, *args, **kwargs)
 
#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO 
#NOTE that the following text is autogenerated, do not modify!


#link MainSystem function to Python function:
exu.MainSystem.SolutionViewer=exu.interactive.SolutionViewer


#link MainSystem function to Python function:
exu.MainSystem.CreateGround=MainSystemCreateGround


#link MainSystem function to Python function:
exu.MainSystem.CreateMassPoint=MainSystemCreateMassPoint


#link MainSystem function to Python function:
exu.MainSystem.CreateRigidBody=MainSystemCreateRigidBody


#link MainSystem function to Python function:
exu.MainSystem.CreateSpringDamper=MainSystemCreateSpringDamper


#link MainSystem function to Python function:
exu.MainSystem.CreateCartesianSpringDamper=MainSystemCreateCartesianSpringDamper


#link MainSystem function to Python function:
exu.MainSystem.CreateRigidBodySpringDamper=MainSystemCreateRigidBodySpringDamper


#link MainSystem function to Python function:
exu.MainSystem.CreateRevoluteJoint=MainSystemCreateRevoluteJoint


#link MainSystem function to Python function:
exu.MainSystem.CreatePrismaticJoint=MainSystemCreatePrismaticJoint


#link MainSystem function to Python function:
exu.MainSystem.CreateSphericalJoint=MainSystemCreateSphericalJoint


#link MainSystem function to Python function:
exu.MainSystem.CreateGenericJoint=MainSystemCreateGenericJoint


#link MainSystem function to Python function:
exu.MainSystem.CreateDistanceConstraint=MainSystemCreateDistanceConstraint


#link MainSystem function to Python function:
exu.MainSystem.CreateForce=MainSystemCreateForce


#link MainSystem function to Python function:
exu.MainSystem.CreateTorque=MainSystemCreateTorque


#link MainSystem function to Python function:
exu.MainSystem.PlotSensor=exu.plot.PlotSensor


#link MainSystem function to Python function:
exu.MainSystem.SolveStatic=exu.solver.SolveStatic


#link MainSystem function to Python function:
exu.MainSystem.SolveDynamic=exu.solver.SolveDynamic


#link MainSystem function to Python function:
exu.MainSystem.ComputeLinearizedSystem=exu.solver.ComputeLinearizedSystem


#link MainSystem function to Python function:
exu.MainSystem.ComputeODE2Eigenvalues=exu.solver.ComputeODE2Eigenvalues


#link MainSystem function to Python function:
exu.MainSystem.ComputeSystemDegreeOfFreedom=exu.solver.ComputeSystemDegreeOfFreedom


#link MainSystem function to Python function:
exu.MainSystem.CreateDistanceSensorGeometry=exu.utilities.CreateDistanceSensorGeometry


#link MainSystem function to Python function:
exu.MainSystem.CreateDistanceSensor=exu.utilities.CreateDistanceSensor


#link MainSystem function to Python function:
exu.MainSystem.DrawSystemGraph=exu.utilities.DrawSystemGraph

