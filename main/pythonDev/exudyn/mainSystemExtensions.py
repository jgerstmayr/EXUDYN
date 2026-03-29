#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  This module provides an extension interface to the C++ class MainSystem;
#           MainSystem is extended by Python interface functions to easily create
#           bodies and point masses without the need to create an according node and
#           connectors and joints without the need to create markers.
#           Extensions are activated in __init__.py
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
from exudyn.utilities import NormL2, Normalize

from exudyn.rigidBodyUtilities import ComputeOrthonormalBasis, \
    RotationMatrix2EulerParameters, AngularVelocity2EulerParameters_t, RotationMatrix2RotXYZ, AngularVelocity2RotXYZ_t, \
    RotationMatrix2RotationVector, HT0, HT2translation, HT2rotationMatrix

import exudyn.itemInterface as eii
from exudyn.advancedUtilities import RaiseTypeError, IsVector, IsReal, ExpectedType, IsValidObjectIndex, IsValidNodeIndex, \
                                    IsValidRealInt, IsValidPRealInt, IsValidURealInt, IsIntVector, \
                                    IsValidBool, IsSquareMatrix, IsNone, IsNotNone, IsInteger, IsValidInt

import numpy as np
import copy


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#add helpful Python extensions for MainSystem, regarding creation of bodies, point masses, connectors and joints

#internal function: do some pre-checks and calculations for joint
def JointPreCheckCalc(where, mbs, name, bodyNumbers, position, show, useGlobalFrame, requireRotMat=True):
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
def ProcessBodyNodeLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where, bodyList=[None,None]):
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
        exu.Print('WARNING: bodyList in MainSystem Create functions is deprecated; use bodyNumbers instead!')
        bodyOrNodeList = [bodyList[0],bodyList[1]] #flat copy, but otherwise would lead to change of args (mutable args!)
        causingArgName = 'bodyList'

    if not exudyn.__useExudynFast:
        if not isinstance(bodyOrNodeList, list) or len(bodyOrNodeList) != 2:
            RaiseTypeError(where=where, argumentName='bodyOrNodeList', received = bodyOrNodeList, expectedType = 'list of 2 body or node numbers')
    
        if not (isinstance(bodyOrNodeList[0], exudyn.ObjectIndex) or (isinstance(bodyOrNodeList[0], exudyn.NodeIndex) and localPosition0==[0.,0.,0.])):
            RaiseTypeError(where=where, argumentName=''+causingArgName+'[0]', received = bodyOrNodeList[0], 
                           expectedType = 'expected either ObjectIndex, or NodeIndex AND localPosition0=[0.,0.,0.]')
            
        if not (isinstance(bodyOrNodeList[1], exudyn.ObjectIndex) or (isinstance(bodyOrNodeList[1], exudyn.NodeIndex) and localPosition1==[0.,0.,0.])):
            RaiseTypeError(where=where, argumentName=''+causingArgName+'[1]', received = bodyOrNodeList[1], 
                           expectedType = 'expected either ObjectIndex, or NodeIndex AND localPosition1=[0.,0.,0.]')
    
    return bodyOrNodeList

#internal: convert exudyn jointType to axis vector
def JointTypeToAxis(jointType):
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
#  show: True: if graphicsData list is empty, node is shown, otherwise body is shown; False: nothing is shown
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
                                                physicsMass=physicsMass,
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
#**function: helper function to create 3D (or 2D) rigid body object and node; all quantities are global (angular velocity, etc.); use this function to easily create a rigid body; graphics can be directly obtained from inertia object, e.g. in case of cylindrical or cuboid shape
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
        bodyNumber = mbs.AddObject(eii.ObjectRigidBody(name=name, physicsMass=inertia.mass, physicsInertia=inertia.GetInertia6D(), 
                                                       physicsCenterOfMass=inertia.com,
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
        bodyNumber = mbs.AddObject(eii.ObjectRigidBody2D(name=name, physicsMass=inertia.mass, physicsInertia=inertia.GetInertia6D()[2],
                                                       #physicsCenterOfMass=inertia.com,
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
#  springForceUserFunction: a user function springForceUserFunction(mbs, t, itemNumber, deltaL, deltaL\_t, stiffness, damping, force)->float ; this function replaces the internal connector force computation
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
#  springForceUserFunction: a user function springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset)->[float,float,float] ; this function replaces the internal connector force computation
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
#  springForceTorqueUserFunction: a user function springForceTorqueUserFunction(mbs, t, itemNumber, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1, offset)->[float,float,float, float,float,float] ; this function replaces the internal connector force / torque computation
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
#**function: helper function to create TorsionalSpringDamper connector, using arguments from ObjectConnectorTorsionalSpringDamper, see there for the full documentation
#**input: 
#  mbs: the MainSystem where items are created
#  name: name string for connector; markers get Marker0:name and Marker1:name
#  bodyNumbers: a list of two body numbers (ObjectIndex) to be connected
#  position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
#  axis: a 3D vector as list or np.array containing the axis around which the spring acts, either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True)
#  stiffness: scalar stiffness of spring
#  damping: scalar damping added to spring
#  offset: scalar offset, which can be used to realize a P-controlled actuator
#  velocityOffset: scalar velocity offset, which can be used to realize a D-controlled actuator
#  torque: additional constant torque added to spring-damper, acting between the two bodies
#  factorMarker0/factorMarker1: scaling factors applied to marker rotations to model gear ratios (effective error = f1*phi1 - f0*phi0)
#  backlash/backlashStiffnessScale/backlashDampingScale: parameters describing torsional backlash deadband and stiffness/damping scaling inside the gap
#  useGlobalFrame: if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
#  springTorqueUserFunction : a user function springTorqueUserFunction(mbs, t, itemNumber, rotation, angularVelocity, stiffness, damping, offset)->float ; this function replaces the internal connector torque computation
#  unlimitedRotations: if True, an additional generic data node is added to enable measurement of rotations beyond +/- pi; this also allows the spring to cope with multiple turns.
#  show: if True, connector visualization is drawn
#  drawSize: general drawing size of connector
#  color: color of connector
#**output: ObjectIndex; returns index of newly created object
#**belongsTo: MainSystem
#**example:
# #coming later
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
                                          factorMarker0 = 1.,
                                          factorMarker1 = 1.,
                                          backlash = 0.,
                                          backlashStiffnessScale = 0.,
                                          backlashDampingScale = 0.,
                                          useGlobalFrame=True,
                                          springTorqueUserFunction=0,
                                          unlimitedRotations = True,
                                          show=True, drawSize=-1, color=exudyn.graphics.color.default):

    where='MainSystem.CreateTorsionalSpringDamper(...)'
    #DELETE: internBodyNodeList = bodyNumbers

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
        if not IsValidURealInt(offset):
            RaiseTypeError(where=where, argumentName='offset', received = offset, expectedType = ExpectedType.UReal)
        if not IsValidURealInt(velocityOffset):
            RaiseTypeError(where=where, argumentName='velocityOffset', received = velocityOffset, expectedType = ExpectedType.UReal)
        if not IsValidURealInt(torque):
            RaiseTypeError(where=where, argumentName='torque', received = torque, expectedType = ExpectedType.UReal)


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

    useExtendedData = (factorMarker0 != 1.) or (factorMarker1 != 1.) or (backlash != 0.)
    if unlimitedRotations or useExtendedData:
        nCoords = 1 if not useExtendedData else 3
        nGeneric = mbs.AddNode(eii.NodeGenericData(initialCoordinates=[0]*nCoords, 
                                             numberOfDataCoordinates=nCoords))
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
                                                                        factorMarker0 = factorMarker0,
                                                                        factorMarker1 = factorMarker1,
                                                                        backlash = backlash,
                                                                        backlashStiffnessScale = backlashStiffnessScale,
                                                                        backlashDampingScale = backlashDampingScale,
                                                                        rotationMarker0=MR0, 
                                                                        rotationMarker1=MR1,
                                                                        springTorqueUserFunction=springTorqueUserFunction, 
                                                                        visualization=eii.VTorsionalSpringDamper(show=show, 
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
#**output: ObjectIndex; returns index of created joint
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

    return oJoint


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
#**output: ObjectIndex; returns index of created joint
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

    return oJoint


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
#**output: ObjectIndex; returns index of created joint
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

    [p0, A0, p1, A1] = JointPreCheckCalc(where, mbs, name, bodyNumbers, position, show, useGlobalFrame, requireRotMat=False)

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

    return oJoint



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
#**output: ObjectIndex; returns index of created joint
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

    return oJoint


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
#**output: ObjectIndex; returns index of created joint
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
        

    return oJoint



#NOTE: could be added in future for CreateCoordinateConstraint:
#  bodyOrNodeList: alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Create coordinate constraint for two bodies, or body on ground; markers and NodePointGround are automatically created when needed
#**input:
#  mbs: the MainSystem where joint and markers shall be created
#  name: name string for joint; markers get Marker0:name and Marker1:name
#  bodyNumbers: a list of two body numbers (ObjectIndex) to be constrained
#  coordinates: a list of two coordinates for the respective bodies (in case of ground, it shall be None)
#  offset: an fixed offset between the two coordinate values
#  factorValue1: an additional factor multiplied with coordinate value1 used in algebraic equation, to enable (e.g. gear) ratio between coordinates
#  velocityLevel: If true: connector constrains velocities (only works for ODE2 coordinates!); offset is used between velocities; if True, the offsetUserFunction\_t is considered and offsetUserFunction is ignored
#  offsetUserFunction: a Python function which defines the time-dependent offset; see description in CoordinateConstraint
#  offsetUserFunction_t: time derivative of offsetUserFunction; needed for velocity level constraints; see description in CoordinateConstraint
#  show: if True, connector visualization is drawn
#  drawSize: general drawing size of node
#  color: color of connector
#**output: ObjectIndex; returns index of created joint
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
# 
# mbs.CreateCoordinateConstraint(bodyNumbers=[None, b0],
#                                coordinates=[None, 0]) #constraints X-coordinate
# 
# #constrain Y-coordinate of b0 to Z-coordinate of m1:
# mbs.CreateCoordinateConstraint(bodyNumbers=[b0, m1], 
#                                coordinates=[1, 2]) 
# 
# mbs.Assemble()
# simulationSettings = exu.SimulationSettings() #takes currently set values or default values
# simulationSettings.timeIntegration.numberOfSteps = 1000
# simulationSettings.timeIntegration.endTime = 2
# 
# mbs.SolveDynamic(simulationSettings = simulationSettings)
def MainSystemCreateCoordinateConstraint(mbs, name='', 
                                        bodyNumbers=[None, None], 
                                        coordinates=[None, None], 
                                        offset = 0.,
                                        factorValue1 = 1.,
                                        velocityLevel = False,
                                        offsetUserFunction = 0,
                                        offsetUserFunction_t = 0,
                                        show=True, drawSize=-1., color=exudyn.graphics.color.default):
    
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
                                                         factorValue1 = factorValue1,
                                                         velocityLevel = velocityLevel,
                                                         offsetUserFunction = offsetUserFunction,
                                                         offsetUserFunction_t = offsetUserFunction_t,
                           visualization=eii.VObjectConnectorCoordinate(show=show, drawSize=drawSize, color=color) ))
       

    return oJoint


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Create an ideal rolling disc joint between wheel rigid body and ground; the disc is infinitely thin and the ground is a perfectly flat plane; the wheel may lift off; definition of joint position and axis in global coordinates (alternatively in wheel (body1) local coordinates) for reference configuration of bodies; all markers and other quantities are automatically computed; some constraint conditions may be deactivated, e.g. to resolve redundancy of constraints for multi-wheel vehicles
#**input:
#  mbs: the MainSystem where joint and markers shall be created
#  name: name string for joint; markers get Marker0:name and Marker1:name
#  bodyNumbers: a list of object numbers for body0=ground and body1=wheel; must be rigid body or ground object
#  axisPosition: a 3D vector as list or np.array: position of wheel axis in local body1=wheel coordinates
#  axisVector: a 3D vector as list or np.array containing the joint (=wheel) axis in local body1=wheel coordinates
#  discRadius: radius of the disc
#  planePosition: any 3D position vector of plane in ground object; given as local coordinates in ground object
#  planeNormal: 3D normal vector of the rolling (contact) plane on ground; given as local coordinates in ground object
#  constrainedAxes: [j0,j1,j2] flags, which determine which constraints are active, in which j0 represents the constraint for lateral motion, j1 longitudinal (forward/backward) motion and j2 represents the normal (contact) direction
#  activeConnector: flag to activate or deactivate the joint
#  show: if True, connector visualization is drawn
#  discWidth: disc with, only used for drawing
#  color: color of connector
#**output: ObjectIndex; returns index of created joint
#**belongsTo: MainSystem
#**example:
# import exudyn as exu
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
#
# r = 0.2
# oDisc = mbs.CreateRigidBody(inertia = InertiaCylinder(density=5000, length=0.1, outerRadius=r, axis=0),
#                           referencePosition = [1,0,r],
#                           initialAngularVelocity = [-3*2*pi,0,0],
#                           initialVelocity = [0,r*3*2*pi,0],
#                           gravity = [0,0,-9.81],
#                           graphicsDataList = [exu.graphics.Cylinder(pAxis = [-0.05,0,0], vAxis = [0.1,0,0], radius = r*0.99,
#                                                                     color=exu.graphics.color.blue),
#                                               exu.graphics.Basis(length=2*r)])
# oGround = mbs.CreateGround(graphicsDataList=[exu.graphics.CheckerBoard(size=4)])
#
# mbs.CreateRollingDisc(bodyNumbers=[oGround, oDisc], 
#                       axisPosition=[0,0,0], axisVector=[1,0,0], #on local wheel frame
#                       planePosition = [0,0,0], planeNormal = [0,0,1],  #in ground frame
#                       discRadius = r, 
#                       discWidth=0.01, color=exu.graphics.color.steelblue)
# mbs.Assemble()
# simulationSettings = exu.SimulationSettings()
# simulationSettings.timeIntegration.numberOfSteps = 1000
# simulationSettings.timeIntegration.endTime = 2
#
# mbs.SolveDynamic(simulationSettings = simulationSettings)
def MainSystemCreateRollingDisc(mbs, name='', bodyNumbers=[None, None], 
                                axisPosition=[], axisVector = [1,0,0],
                                discRadius = 0., planePosition = [0,0,0], planeNormal = [0,0,1], 
                                constrainedAxes = [1,1,1],
                                activeConnector = True,
                                show=True, discWidth=0.1, color=exudyn.graphics.color.default):
    
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
#**function: Create penalty-based rolling disc joint between wheel rigid body and ground; the disc is infinitely thin and the ground is a perfectly flat plane; the wheel may lift off; definition of joint position and axis in global coordinates (alternatively in wheel (body1) local coordinates) for reference configuration of bodies; all markers and other quantities are automatically computed
#**input:
#  mbs: the MainSystem where joint and markers shall be created
#  name: name string for joint; markers get Marker0:name and Marker1:name
#  bodyNumbers: a list of object numbers for body0=ground and body1=wheel; must be rigid body or ground object
#  axisPosition: a 3D vector as list or np.array: position of wheel axis in local body1=wheel coordinates
#  axisVector: a 3D vector as list or np.array containing the joint (=wheel) axis in local body1=wheel coordinates
#  discRadius: radius of the disc
#  planePosition: any 3D position vector of plane in ground object; given as local coordinates in ground object
#  planeNormal: 3D normal vector of the rolling (contact) plane on ground; given as local coordinates in ground object
#  dryFrictionAngle: angle (radiant) which defines a rotation of the local tangential coordinates dry friction; this allows to model Mecanum wheels with specified roll angle
#  contactStiffness: normal contact stiffness
#  contactDamping: normal contact damping
#  dryFriction: 2D list of friction parameters; dry friction coefficients in local wheel coordinates, where for dryFrictionAngle=0, the first parameter refers to forward direction and the second parameter to lateral direction
#  viscousFriction: 2D list of viscous friction coefficients [SI:1/(m/s)] in local wheel coordinates; proportional to slipping velocity, leading to increasing slipping friction force for increasing slipping velocity; directions are same as in dryFriction
#  dryFrictionProportionalZone: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations)
#  rollingFrictionViscous: rolling friction [SI:1], which acts against the velocity of the trail on ground and leads to a force proportional to the contact normal force; 
#  useLinearProportionalZone: if True, a linear proportional zone is used; the linear zone performs better in implicit time integration as the Jacobian has a constant tangent in the sticking case
#  activeConnector: flag to activate or deactivate the connector
#  show: if True, connector visualization is drawn
#  discWidth: disc with, only used for drawing
#  color: color of connector
#**output: ObjectIndex; returns index of created joint
#**belongsTo: MainSystem
#**example:
# import exudyn as exu
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()

# r = 0.2
# oDisc = mbs.CreateRigidBody(inertia = InertiaCylinder(density=5000, length=0.1, outerRadius=r, axis=0),
#                           referencePosition = [1,0,r],
#                           initialAngularVelocity = [-3*2*pi,0,0],
#                           initialVelocity = [0,r*3*2*pi,0],
#                           gravity = [0,0,-9.81],
#                           graphicsDataList = [exu.graphics.Cylinder(pAxis = [-0.05,0,0], vAxis = [0.1,0,0], radius = r*0.99,
#                                                                     color=exu.graphics.color.blue),
#                                               exu.graphics.Basis(length=2*r)])
# oGround = mbs.CreateGround(graphicsDataList=[exu.graphics.CheckerBoard(size=4)])

# mbs.CreateRollingDiscPenalty(bodyNumbers=[oGround, oDisc], axisPosition=[0,0,0], axisVector=[1,0,0],
#                               discRadius = r, planePosition = [0,0,0], planeNormal = [0,0,1], 
#                               dryFriction = [0.2,0.2],
#                               contactStiffness = 1e5, contactDamping = 2e3, 
#                               discWidth=0.01, color=exu.graphics.color.steelblue)

# mbs.Assemble()
# simulationSettings = exu.SimulationSettings()
# simulationSettings.timeIntegration.numberOfSteps = 1000
# simulationSettings.timeIntegration.endTime = 2

# mbs.SolveDynamic(simulationSettings = simulationSettings)
def MainSystemCreateRollingDiscPenalty(mbs, name='', bodyNumbers=[None, None], 
                                  axisPosition=[], axisVector = [1,0,0],
                                  discRadius = 0., planePosition = [0,0,0], planeNormal = [0,0,1], 
                                  contactStiffness = 0., contactDamping = 0., 
                                  dryFriction = [0,0], dryFrictionAngle = 0., 
                                  dryFrictionProportionalZone = 0., viscousFriction = [0,0], 
                                  rollingFrictionViscous = 0., useLinearProportionalZone = False, 
                                  activeConnector = True, 
                                  show=True, discWidth=0.1, color=exudyn.graphics.color.default):
    
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
        if not IsValidRealInt(rollingFrictionViscous):
            RaiseTypeError(where=where, argumentName='rollingFrictionViscous', received = rollingFrictionViscous, expectedType = ExpectedType.Real)
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
                                                                 rollingFrictionViscous = rollingFrictionViscous, useLinearProportionalZone = useLinearProportionalZone, 
                                                                 activeConnector = activeConnector, 
                                                                 visualization = eii.VObjectConnectorRollingDiscPenalty(show=show, discWidth=discWidth, 
                                                                                                                        color=color) ))
                           
    return oJoint



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Create penalty-based sphere-sphere contact between two rigid bodies, mass points or according nodes; the contact is based on ObjectContactSphereSphere; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems
#**input:
#  mbs: the MainSystem where joint and markers shall be created
#  name: name string for joint; markers get Marker0:name and Marker1:name
#  bodyNumbers: a list of object numbers for sphere0 and sphere1; Note that if body is a mass point, friction due to rolling is not accounted for!
#  localPosition0: local position (as 3D list or numpy array) of sphere0 on body0, if not a node number
#  localPosition1: local position (as 3D list or numpy array) of sphere1 on body1, if not a node number
#  spheresRadii: list containing radius of sphere 0 and radius of sphere 1 [SI:m].
#  isHollowSphere1: flag, which determines, if sphere attached to marker 1 (radius 1) is a hollow sphere.
#  dynamicFriction: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, Section Module: physics
#  frictionProportionalZone: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), Section Module: physics
#  contactStiffness: normal contact stiffness
#  contactDamping: linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
#  contactStiffnessExponent: exponent in normal contact model [SI:1]
#  constantPullOffForce: constant adhesion force [SI:N]; Edinburgh Adhesive Elasto-Plastic Model
#  contactPlasticityRatio: ratio of contact stiffness for first loading and unloading/reloading [SI:1]; Edinburgh Adhesive Elasto-Plastic Model; see ObjectContactSphereSphere
#  adhesionCoefficient: coefficient for adhesion [SI:N/m]; Edinburgh Adhesive Elasto-Plastic Model; set to 0 to deactivate adhesion model
#  adhesionExponent: exponent for adhesion coefficient [SI:1]; Edinburgh Adhesive Elasto-Plastic Model
#  restitutionCoefficient: coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)
#  minimumImpactVelocity: minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)
#  impactModel: number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!
#  dataInitialCoordinates: a list of four values for initialization of the data node, used for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused
#  activeConnector: flag to activate or deactivate the connector
#  bodyOrNodeList: alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
#  show: if True, connector visualization is drawn
#  color: color of connector
#**output: ObjectIndex; returns index of created joint
#**belongsTo: MainSystem
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
                                       show=False, color=exudyn.graphics.color.default):
    
    where = 'MainSystem.CreateSphereSphereContact(...)'
    internBodyNodeList = ProcessBodyNodeLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where)

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
        
    if isinstance(internBodyNodeList[0], exudyn.ObjectIndex):
        mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=internBodyNodeList[0], localPosition=localPosition0))
    else:
        mBody0 = mbs.AddMarker(eii.MarkerNodeRigid(name=mName0,nodeNumber=internBodyNodeList[0]))

    if isinstance(internBodyNodeList[1], exudyn.ObjectIndex):
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=internBodyNodeList[1], localPosition=localPosition1))
    else:
        mBody1 = mbs.AddMarker(eii.MarkerNodeRigid(name=mName1,nodeNumber=internBodyNodeList[1]))
    
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
#**function: Create penalty-based circle-circle contact between two rigid or point bodies in the plane; the contact is based on ObjectContactCircleCircle and supports convex (positive radius) as well as concave (negative radius) follower circles.
#**input:
#  mbs: the MainSystem where markers and the contact object shall be created
#  name: name string for the contact; markers get Marker0:name and Marker1:name
#  bodyNumbers: list of object numbers for circle0 and circle1; pass None entries and use bodyOrNodeList for mixed node/object references
#  localPosition0: local position (3D) of the first circle center on body0, if not a node reference
#  localPosition1: local position (3D) of the second circle center on body1, if not a node reference
#  radius1: radius [m] of the first circle; use negative value to model a concave surface
#  radius2: radius [m] of the second circle; negative values create an inward facing cavity
#  contactStiffness: normal contact stiffness [N/m]
#  contactDamping: normal contact damping [N s/m]
#  frictionCoefficient: Coulomb friction coefficient; set to 0 for frictionless contact
#  dataInitialCoordinates: initialization for the discontinuous iteration data node (length must equal 6)
#  activeConnector: flag to activate or deactivate the connector
#  bodyOrNodeList: alternative to bodyNumbers; use for node references or mixed cases
#  show: if True, connector visualization is drawn
#  color: RGBA color of connector visualization
#**output: ObjectIndex; returns index of created contact object
#**belongsTo: MainSystem
def MainSystemCreateCircleCircleContact(mbs, name='', bodyNumbers=[None, None],
                                        localPosition0 = [0.,0.,0.], localPosition1 = [0.,0.,0.],
                                        radius1 = 1., radius2 = 1.,
                                        contactStiffness = 0., contactDamping = 0.,
                                        frictionCoefficient = 0.,
                                        frictionVelocityPenalty = 0.,
                                        frictionProportionalZone = 0.001,
                                        frictionStiffness = 0.,
                                        dataInitialCoordinates = [0.,0.,0.,0.,0.,0.,0.,0.],
                                        activeConnector=True,
                                        bodyOrNodeList=[None, None],
                                        show=False, color=exudyn.graphics.color.default):

    where = 'MainSystem.CreateCircleCircleContact(...)'
    internBodyNodeList = ProcessBodyNodeLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where)

    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsVector(localPosition0, 3):
            RaiseTypeError(where=where, argumentName='localPosition0', received = localPosition0, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(localPosition1, 3):
            RaiseTypeError(where=where, argumentName='localPosition1', received = localPosition1, expectedType = ExpectedType.Vector, dim=3)

        if not IsValidRealInt(radius1):
            RaiseTypeError(where=where, argumentName='radius1', received = radius1, expectedType = ExpectedType.Real)
        if not IsValidRealInt(radius2):
            RaiseTypeError(where=where, argumentName='radius2', received = radius2, expectedType = ExpectedType.Real)

        if not IsValidURealInt(contactStiffness):
            RaiseTypeError(where=where, argumentName='contactStiffness', received = contactStiffness, expectedType = ExpectedType.Real)
        if not IsValidURealInt(contactDamping):
            RaiseTypeError(where=where, argumentName='contactDamping', received = contactDamping, expectedType = ExpectedType.Real)
        if not IsValidURealInt(frictionCoefficient):
            RaiseTypeError(where=where, argumentName='frictionCoefficient', received = frictionCoefficient, expectedType = ExpectedType.Real)
        if not IsValidURealInt(frictionVelocityPenalty):
            RaiseTypeError(where=where, argumentName='frictionVelocityPenalty', received = frictionVelocityPenalty, expectedType = ExpectedType.Real)
        if not IsValidURealInt(frictionProportionalZone):
            RaiseTypeError(where=where, argumentName='frictionProportionalZone', received = frictionProportionalZone, expectedType = ExpectedType.Real)
        if not IsValidURealInt(frictionStiffness):
            RaiseTypeError(where=where, argumentName='frictionStiffness', received = frictionStiffness, expectedType = ExpectedType.Real)

        if not IsVector(dataInitialCoordinates, 8):
            RaiseTypeError(where=where, argumentName='dataInitialCoordinates', received = dataInitialCoordinates, expectedType = ExpectedType.Vector, dim=6)

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

    if isinstance(internBodyNodeList[0], exudyn.ObjectIndex):
        mBody0 = mbs.AddMarker(eii.MarkerBodyPosition(name=mName0, bodyNumber=internBodyNodeList[0], localPosition=localPosition0))
    else:
        mBody0 = mbs.AddMarker(eii.MarkerNodePosition(name=mName0, nodeNumber=internBodyNodeList[0]))

    if isinstance(internBodyNodeList[1], exudyn.ObjectIndex):
        mBody1 = mbs.AddMarker(eii.MarkerBodyPosition(name=mName1, bodyNumber=internBodyNodeList[1], localPosition=localPosition1))
    else:
        mBody1 = mbs.AddMarker(eii.MarkerNodePosition(name=mName1, nodeNumber=internBodyNodeList[1]))

    nGeneric = mbs.AddNode(eii.NodeGenericData(initialCoordinates=dataInitialCoordinates,
                                         numberOfDataCoordinates=len(dataInitialCoordinates)))

    oContact = mbs.AddObject(eii.ObjectContactCircleCircle(markerNumbers=[mBody0, mBody1],
                                                   nodeNumber=nGeneric,
                                                   radius1=radius1,
                                                   radius2=radius2,
                                                   contactStiffness=contactStiffness,
                                                   contactDamping=contactDamping,
                                                   frictionCoefficient=frictionCoefficient,
                                                   frictionVelocityPenalty=frictionVelocityPenalty,
                                                   frictionProportionalZone=frictionProportionalZone,
                                                   frictionStiffness=frictionStiffness,
                                                   activeConnector=activeConnector,
                                                   visualization=eii.VObjectContactCircleCircle(show=show, color=color)))

    return oContact


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Create penalty-based curve-circle contact; wraps ObjectContactCurveCircles.
#**input:
#  name: name string for object
#  bodyNumberCurve: body number of the body carrying the curve (segments)
#  bodyNumberCircle: body number of the body carrying the circle
#  localPositionCurve: local position of curve marker on bodyNumberCurve
#  localPositionCircle: local position of circle marker on bodyNumberCircle
#  circleRadius: radius of the circle [SI:m]
#  segmentsData: matrix containing segment data, each row [x0,y0,x1,y1] defining a line segment
#  rotationMarker0: 3x3 rotation matrix for curve marker orientation (default: identity)
#  dynamicFriction: dynamic friction coefficient
#  frictionProportionalZone: limit velocity [m/s] for friction regularization
#  frictionVelocityPenalty: velocity penalty coefficient [SI:N/(m/s)]; if > 0, uses velocity penalty friction model
#  contactStiffness: normal contact stiffness [SI:N/m]
#  contactDamping: linear normal contact damping [SI:N/(m/s)]
#  contactModel: contact model type (0: linear, 1: integral)
#  activeConnector: flag to activate or deactivate the connector
#  show: if True, connector visualization is drawn
#  color: RGBA color of connector visualization
#**output: ObjectIndex; returns index of created contact object
#**belongsTo: MainSystem
def MainSystemCreateContactCurveCircles(mbs, name='', 
                                        bodyNumberCurve=None, bodyNumberCircle=None,
                                        localPositionCurve=[0.,0.,0.], localPositionCircle=[0.,0.,0.],
                                        circleRadius=1.,
                                        segmentsData=None,
                                        rotationMarker0=np.eye(3),
                                        dynamicFriction=0., 
                                        frictionProportionalZone=0.001,
                                        frictionVelocityPenalty=0.,
                                        frictionStiffness=0.,
                                        contactStiffness=0., contactDamping=0.,
                                        contactModel=0,
                                        activeConnector=True,
                                        # Hertz contact parameters
                                        useHertzContact=False,
                                        faceWidth=0.01,
                                        elasticModulus=2.1e11,
                                        poissonRatio=0.3,
                                        curvaturePerPoint=[],
                                        baseStiffnessPerPoint=[],
                                        show=True, color=exudyn.graphics.color.default):

    where = 'MainSystem.CreateContactCurveCircles(...)'

    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received=name, expectedType=ExpectedType.String)
        if bodyNumberCurve is None:
            raise ValueError(where + ": bodyNumberCurve must be provided")
        if bodyNumberCircle is None:
            raise ValueError(where + ": bodyNumberCircle must be provided")
        if segmentsData is None:
            raise ValueError(where + ": segmentsData must be provided")
        if not IsVector(localPositionCurve, 3):
            RaiseTypeError(where=where, argumentName='localPositionCurve', received=localPositionCurve, expectedType=ExpectedType.Vector, dim=3)
        if not IsVector(localPositionCircle, 3):
            RaiseTypeError(where=where, argumentName='localPositionCircle', received=localPositionCircle, expectedType=ExpectedType.Vector, dim=3)
        if not IsValidRealInt(circleRadius):
            RaiseTypeError(where=where, argumentName='circleRadius', received=circleRadius, expectedType=ExpectedType.Real)
        if not IsValidURealInt(dynamicFriction):
            RaiseTypeError(where=where, argumentName='dynamicFriction', received=dynamicFriction, expectedType=ExpectedType.Real)
        if not IsValidURealInt(frictionProportionalZone):
            RaiseTypeError(where=where, argumentName='frictionProportionalZone', received=frictionProportionalZone, expectedType=ExpectedType.Real)
        if not IsValidURealInt(frictionVelocityPenalty):
            RaiseTypeError(where=where, argumentName='frictionVelocityPenalty', received=frictionVelocityPenalty, expectedType=ExpectedType.Real)
        if not IsValidURealInt(frictionStiffness):
            RaiseTypeError(where=where, argumentName='frictionStiffness', received=frictionStiffness, expectedType=ExpectedType.Real)
        if not IsValidURealInt(contactStiffness):
            RaiseTypeError(where=where, argumentName='contactStiffness', received=contactStiffness, expectedType=ExpectedType.Real)
        if not IsValidURealInt(contactDamping):
            RaiseTypeError(where=where, argumentName='contactDamping', received=contactDamping, expectedType=ExpectedType.Real)
        if not IsValidBool(activeConnector):
            RaiseTypeError(where=where, argumentName='activeConnector', received=activeConnector, expectedType=ExpectedType.Bool)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received=show, expectedType=ExpectedType.Bool)

    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'MarkerCurve:' + name
        mName1 = 'MarkerCircle:' + name

    # Create markers for curve and circle bodies
    mCurve = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0, bodyNumber=bodyNumberCurve, localPosition=localPositionCurve))
    mCircle = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1, bodyNumber=bodyNumberCircle, localPosition=localPositionCircle))

    # Create data node for contact state (3 data variables per segment)
    nSegments = len(segmentsData) if hasattr(segmentsData, '__len__') else segmentsData.shape[0]
    initialContactData = []
    for _ in range(nSegments):
        initialContactData.extend([-1.0, 0.0, 0.0, 0.0, 0.0])  # circleIndex, gap, vTangent, stickPosX, stickPosY

    nGeneric = mbs.AddNode(eii.NodeGenericData(initialCoordinates=initialContactData,
                                               numberOfDataCoordinates=len(initialContactData)))

    # Create segments matrix container
    segmentsMatrix = exudyn.MatrixContainer(segmentsData)

    oContact = mbs.AddObject(eii.ObjectContactCurveCircles(
        name=name,
        markerNumbers=[mCurve, mCircle],
        nodeNumber=nGeneric,
        circlesRadii=[circleRadius],
        segmentsData=segmentsMatrix,
        rotationMarker0=rotationMarker0,
        dynamicFriction=dynamicFriction,
        frictionProportionalZone=frictionProportionalZone,
        frictionVelocityPenalty=frictionVelocityPenalty,
        frictionStiffness=frictionStiffness,
        contactStiffness=contactStiffness,
        contactDamping=contactDamping,
        contactModel=contactModel,
        activeConnector=activeConnector,
        # Hertz contact parameters
        useHertzContact=useHertzContact,
        faceWidth=faceWidth,
        elasticModulus=elasticModulus,
        poissonRatio=poissonRatio,
        curvaturePerPoint=curvaturePerPoint,
        baseStiffnessPerPoint=baseStiffnessPerPoint,
        visualization=eii.VObjectContactCurveCircles(show=show, color=color)))

    return oContact


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Create penalty-based polyline/polyline gear contact for few-tooth-difference transmissions; wraps ObjectContactFewTeeth.
#**input: see documentation of ObjectContactFewTeeth
#**output: ObjectIndex of newly created contact object
def MainSystemCreateFewTeethContact(mbs, name='', bodyNumbers=[None, None],
                                    outerAnalytic=None, innerAnalytic=None,
                                    contactStiffness=0., contactDamping=0.,
                                    stiffnessOuter=None, stiffnessInner=None,
                                    faceWidth=0.01, elasticModulus=2.1e11, poissonRatio=0.3,
                                    frictionCoefficient=0., frictionVelocityPenalty=0., frictionProportionalZone=0.001,
                                    frictionStiffness=0.,
                                    contactReferenceDistance=0., tipProbeLength=0.005,
                                    activeConnector=True,
                                    bodyOrNodeList=[None, None],
                                    show=False, color=exudyn.graphics.color.default):

    where = 'MainSystem.CreateFewTeethContact(...)'
    internBodyNodeList = ProcessBodyNodeLists(bodyNumbers, bodyOrNodeList, [0.,0.,0.], [0.,0.,0.], where)

    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received=name, expectedType=ExpectedType.String)
        for profName, profile in [('outerAnalytic', outerAnalytic), ('innerAnalytic', innerAnalytic)]:
            if profile is not None and not isinstance(profile, dict):
                RaiseTypeError(where=where, argumentName=profName, received=profile, expectedType='dict')
        if not IsValidURealInt(contactStiffness):
            RaiseTypeError(where=where, argumentName='contactStiffness', received=contactStiffness, expectedType=ExpectedType.Real)
        if not IsValidURealInt(contactDamping):
            RaiseTypeError(where=where, argumentName='contactDamping', received=contactDamping, expectedType=ExpectedType.Real)
        if not IsValidRealInt(contactReferenceDistance):
            RaiseTypeError(where=where, argumentName='contactReferenceDistance', received=contactReferenceDistance, expectedType=ExpectedType.Real)
        if not IsValidURealInt(faceWidth):
            RaiseTypeError(where=where, argumentName='faceWidth', received=faceWidth, expectedType=ExpectedType.Real)
        if not IsValidURealInt(elasticModulus):
            RaiseTypeError(where=where, argumentName='elasticModulus', received=elasticModulus, expectedType=ExpectedType.Real)
        if not IsValidURealInt(poissonRatio):
            RaiseTypeError(where=where, argumentName='poissonRatio', received=poissonRatio, expectedType=ExpectedType.Real)
        if not IsValidURealInt(frictionCoefficient):
            RaiseTypeError(where=where, argumentName='frictionCoefficient', received=frictionCoefficient, expectedType=ExpectedType.Real)
        if not IsValidURealInt(frictionVelocityPenalty):
            RaiseTypeError(where=where, argumentName='frictionVelocityPenalty', received=frictionVelocityPenalty, expectedType=ExpectedType.Real)
        if not IsValidURealInt(frictionProportionalZone):
            RaiseTypeError(where=where, argumentName='frictionProportionalZone', received=frictionProportionalZone, expectedType=ExpectedType.Real)
        if not IsValidURealInt(frictionStiffness):
            RaiseTypeError(where=where, argumentName='frictionStiffness', received=frictionStiffness, expectedType=ExpectedType.Real)
        if not IsValidURealInt(tipProbeLength):
            RaiseTypeError(where=where, argumentName='tipProbeLength', received=tipProbeLength, expectedType=ExpectedType.Real)
        if not IsValidBool(activeConnector):
            RaiseTypeError(where=where, argumentName='activeConnector', received=activeConnector, expectedType=ExpectedType.Bool)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received=show, expectedType=ExpectedType.Bool)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received=color, expectedType=ExpectedType.Vector, dim=4)

    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:' + name
        mName1 = 'Marker1:' + name

    if isinstance(internBodyNodeList[0], exudyn.ObjectIndex):
        mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0, bodyNumber=internBodyNodeList[0], localPosition=[0.,0.,0.]))
    else:
        mBody0 = mbs.AddMarker(eii.MarkerNodeRigid(name=mName0, nodeNumber=internBodyNodeList[0]))

    if isinstance(internBodyNodeList[1], exudyn.ObjectIndex):
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1, bodyNumber=internBodyNodeList[1], localPosition=[0.,0.,0.]))
    else:
        mBody1 = mbs.AddMarker(eii.MarkerNodeRigid(name=mName1, nodeNumber=internBodyNodeList[1]))

    nGeneric = mbs.AddNode(eii.NodeGenericData(initialCoordinates=[0.]*9, numberOfDataCoordinates=9))  # 增加2个用于Bristle粘滞位置

    oContact = mbs.AddObject(eii.ObjectContactFewTeeth(name=name,
                                                       markerNumbers=[mBody0, mBody1],
                                                       nodeNumber=nGeneric,
                                                       outerAnalytic=outerAnalytic,
                                                       innerAnalytic=innerAnalytic,
                                                       contactStiffness=contactStiffness,
                                                       contactDamping=contactDamping,
                                                       stiffnessOuter=stiffnessOuter,
                                                       stiffnessInner=stiffnessInner,
                                                       faceWidth=faceWidth,
                                                       elasticModulus=elasticModulus,
                                                       poissonRatio=poissonRatio,
                                                       frictionCoefficient=frictionCoefficient,
                                                       frictionVelocityPenalty=frictionVelocityPenalty,
                                                       frictionProportionalZone=frictionProportionalZone,
                                                       frictionStiffness=frictionStiffness,
                                                       contactReferenceDistance=contactReferenceDistance,
                                                       tipProbeLength=tipProbeLength,
                                                       activeConnector=activeConnector,
                                                       visualization=eii.VObjectContactFewTeeth(show=show, color=color)))

    return oContact


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Create penalty-based external involute gear contact for standard gear pairs; wraps ObjectContactExternalInvolute.
#**input: see documentation of ObjectContactExternalInvolute
#**output: ObjectIndex of newly created contact object
def MainSystemCreateExternalGearContact(mbs, name='', bodyNumbers=[None, None],
                                        gear1Analytic=None, gear2Analytic=None,
                                        contactDamping=0.,
                                        stiffnessGear1=None, stiffnessGear2=None,
                                        faceWidth=0.01, elasticModulus=2.1e11, poissonRatio=0.3,
                                        frictionCoefficient=0., frictionVelocityPenalty=0., frictionProportionalZone=0.001,
                                        frictionStiffness=0.,
                                        tipProbeLength=0.005,
                                        activeConnector=True,
                                        bodyOrNodeList=[None, None],
                                        show=False, color=exudyn.graphics.color.default):

    where = 'MainSystem.CreateExternalGearContact(...)'
    internBodyNodeList = ProcessBodyNodeLists(bodyNumbers, bodyOrNodeList, [0.,0.,0.], [0.,0.,0.], where)

    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received=name, expectedType=ExpectedType.String)
        for profName, profile in [('gear1Analytic', gear1Analytic), ('gear2Analytic', gear2Analytic)]:
            if profile is not None and not isinstance(profile, dict):
                RaiseTypeError(where=where, argumentName=profName, received=profile, expectedType='dict')
        if not IsValidURealInt(contactDamping):
            RaiseTypeError(where=where, argumentName='contactDamping', received=contactDamping, expectedType=ExpectedType.Real)
        if not IsValidURealInt(faceWidth):
            RaiseTypeError(where=where, argumentName='faceWidth', received=faceWidth, expectedType=ExpectedType.Real)
        if not IsValidURealInt(elasticModulus):
            RaiseTypeError(where=where, argumentName='elasticModulus', received=elasticModulus, expectedType=ExpectedType.Real)
        if not IsValidURealInt(poissonRatio):
            RaiseTypeError(where=where, argumentName='poissonRatio', received=poissonRatio, expectedType=ExpectedType.Real)
        if not IsValidURealInt(frictionCoefficient):
            RaiseTypeError(where=where, argumentName='frictionCoefficient', received=frictionCoefficient, expectedType=ExpectedType.Real)
        if not IsValidURealInt(frictionVelocityPenalty):
            RaiseTypeError(where=where, argumentName='frictionVelocityPenalty', received=frictionVelocityPenalty, expectedType=ExpectedType.Real)
        if not IsValidURealInt(frictionProportionalZone):
            RaiseTypeError(where=where, argumentName='frictionProportionalZone', received=frictionProportionalZone, expectedType=ExpectedType.Real)
        if not IsValidURealInt(frictionStiffness):
            RaiseTypeError(where=where, argumentName='frictionStiffness', received=frictionStiffness, expectedType=ExpectedType.Real)
        if not IsValidURealInt(tipProbeLength):
            RaiseTypeError(where=where, argumentName='tipProbeLength', received=tipProbeLength, expectedType=ExpectedType.Real)
        if not IsValidBool(activeConnector):
            RaiseTypeError(where=where, argumentName='activeConnector', received=activeConnector, expectedType=ExpectedType.Bool)
        if not IsValidBool(show):
            RaiseTypeError(where=where, argumentName='show', received=show, expectedType=ExpectedType.Bool)
        if not IsVector(color, 4):
            RaiseTypeError(where=where, argumentName='color', received=color, expectedType=ExpectedType.Vector, dim=4)

    mName0 = ''
    mName1 = ''
    if name != '':
        mName0 = 'Marker0:' + name
        mName1 = 'Marker1:' + name

    if isinstance(internBodyNodeList[0], exudyn.ObjectIndex):
        mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0, bodyNumber=internBodyNodeList[0], localPosition=[0.,0.,0.]))
    else:
        mBody0 = mbs.AddMarker(eii.MarkerNodeRigid(name=mName0, nodeNumber=internBodyNodeList[0]))

    if isinstance(internBodyNodeList[1], exudyn.ObjectIndex):
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1, bodyNumber=internBodyNodeList[1], localPosition=[0.,0.,0.]))
    else:
        mBody1 = mbs.AddMarker(eii.MarkerNodeRigid(name=mName1, nodeNumber=internBodyNodeList[1]))

    nGeneric = mbs.AddNode(eii.NodeGenericData(initialCoordinates=[0.]*11, numberOfDataCoordinates=11))

    oContact = mbs.AddObject(eii.ObjectContactExternalInvolute(name=name,
                                                       markerNumbers=[mBody0, mBody1],
                                                       nodeNumber=nGeneric,
                                                       gear1Analytic=gear1Analytic,
                                                       gear2Analytic=gear2Analytic,
                                                       contactDamping=contactDamping,
                                                       stiffnessGear1=stiffnessGear1,
                                                       stiffnessGear2=stiffnessGear2,
                                                       faceWidth=faceWidth,
                                                       elasticModulus=elasticModulus,
                                                       poissonRatio=poissonRatio,
                                                       frictionCoefficient=frictionCoefficient,
                                                       frictionVelocityPenalty=frictionVelocityPenalty,
                                                       frictionProportionalZone=frictionProportionalZone,
                                                       frictionStiffness=frictionStiffness,
                                                       tipProbeLength=tipProbeLength,
                                                       activeConnector=activeConnector,
                                                       visualization=eii.VObjectContactExternalInvolute(show=show, color=color)))

    return oContact


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Create penalty-based sphere-quad contact between two rigid bodies, mass points or according nodes; the contact is based on two ObjectContactSphereTriangle; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems
#**input:
#  mbs: the MainSystem where joint and markers shall be created
#  name: name string for joint; markers get Marker0:name and Marker1:name
#  bodyNumbers: a list of object numbers for sphere (0) and quad (1); Note that if body is a mass point, friction due to rolling is not accounted for!
#  localPosition0: local position (as 3D list or numpy array) of sphere0 on body0, if not a node number
#  radiusSphere: radius of sphere 0 [SI:m].
#  quadPoints: 4 points as Vector3DList to define the quad, defined in body1 local coordinates; note that the quad is split into two triangles with point indices [0,1,3] and [1,2,3]
#  includeEdges: binary flag, where 1 defines contact with edges 0, 2 with edge 1, 4 with edge 2 and 8 with edge 3; 15 means that contact with all edges is included; edge 0 is the edge between node 0 and node 1, etc.
#  dynamicFriction: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, Section Module: physics
#  frictionProportionalZone: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), Section Module: physics
#  contactStiffness: normal contact stiffness
#  contactDamping: linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
#  contactStiffnessExponent: exponent in normal contact model [SI:1]
#  restitutionCoefficient: coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)
#  minimumImpactVelocity: minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)
#  impactModel: number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!
#  dataInitialCoordinates: a list of four values for initialization of the data node, used for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused
#  activeConnector: flag to activate or deactivate the connector
#  bodyOrNodeList: alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
#  localPosition1: local position (as 3D list or numpy array) of quad1 on body1; this is usually not needed and adds simply an offset to the quad coordinates
#  show: if True, connector visualization is drawn
#  color: color of connector
#**output: dict containing oContact0 and oContact1 with ObjectIndex of each contact object
#**belongsTo: MainSystem
def MainSystemCreateSphereQuadContact(mbs, name='', bodyNumbers=[None, None], 
                                       localPosition0 = [0.,0.,0.], radiusSphere = 0,
                                       quadPoints = exudyn.Vector3DList([[0,0,0],[1,0,0],[1,1,0],[0,1,0]]),
                                       includeEdges = 15, dynamicFriction = 0., frictionProportionalZone = 1e-3,
                                       contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1,
                                       restitutionCoefficient = 1, minimumImpactVelocity = 0,
                                       impactModel = 0,
                                       dataInitialCoordinates = [0,0,0,0],
                                       activeConnector=True,
                                       bodyOrNodeList=[None, None], 
                                       localPosition1 = [0.,0.,0.], 
                                       show=False, color=exudyn.graphics.color.default):
    
    where = 'MainSystem.CreateSphereSphereContact(...)'
    internBodyNodeList = ProcessBodyNodeLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where)

    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsVector(localPosition0, 3):
            RaiseTypeError(where=where, argumentName='localPosition0', received = localPosition0, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(localPosition1, 3):
            RaiseTypeError(where=where, argumentName='localPosition1', received = localPosition1, expectedType = ExpectedType.Vector, dim=3)
        if not IsValidPRealInt(radiusSphere):
            RaiseTypeError(where=where, argumentName='radiusSphere', received = radiusSphere, expectedType = ExpectedType.Real)
        if type(quadPoints) != exudyn.Vector3DList or len(quadPoints) != 4:
            RaiseTypeError(where=where, argumentName='quadPoints', received = quadPoints, expectedType = 'expected type=exudyn.Vector3DList of length 4')
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
        
    if isinstance(internBodyNodeList[0], exudyn.ObjectIndex):
        mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=internBodyNodeList[0], localPosition=localPosition0))
    else:
        mBody0 = mbs.AddMarker(eii.MarkerNodeRigid(name=mName0,nodeNumber=internBodyNodeList[0]))

    if isinstance(internBodyNodeList[1], exudyn.ObjectIndex):
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=internBodyNodeList[1], localPosition=localPosition1))
    else:
        mBody1 = mbs.AddMarker(eii.MarkerNodeRigid(name=mName1,nodeNumber=internBodyNodeList[1]))

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
                                                        radiusSphere=radiusSphere,
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
#**function: Create penalty-based sphere-triangle contact between two rigid bodies, mass points or according nodes; the contact is based on ObjectContactSphereTriangle; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems
#**input:
#  mbs: the MainSystem where joint and markers shall be created
#  name: name string for joint; markers get Marker0:name and Marker1:name
#  bodyNumbers: a list of object numbers for sphere (0) and triangle (1); Note that if body is a mass point, friction due to rolling is not accounted for!
#  localPosition0: local position (as 3D list or numpy array) of sphere0 on body0, if not a node number
#  radiusSphere: radius of sphere 0 [SI:m].
#  trianglePoints: triangle points as Vector3DList, defined in body1 local coordinates
#  includeEdges: binary flag, where 1 defines contact with edges 0, 2 with edge 1 and 4 with edge 2; 7 means that contact with all edges is included; edge 0 is the edge between node 0 and node 1, etc.
#  dynamicFriction: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, Section Module: physics
#  frictionProportionalZone: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), Section Module: physics
#  contactStiffness: normal contact stiffness
#  contactDamping: linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
#  contactStiffnessExponent: exponent in normal contact model [SI:1]
#  restitutionCoefficient: coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)
#  minimumImpactVelocity: minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)
#  impactModel: number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!
#  dataInitialCoordinates: a list of four values for initialization of the data node, used for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused
#  activeConnector: flag to activate or deactivate the connector
#  bodyOrNodeList: alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
#  localPosition1: local position (as 3D list or numpy array) of triangle1 on body1; this is usually not needed and adds simply an offset to the triangle coordinates
#  show: if True, connector visualization is drawn
#  color: color of connector
#**output: ObjectIndex; returns index of created joint
#**belongsTo: MainSystem
def MainSystemCreateSphereTriangleContact(mbs, name='', bodyNumbers=[None, None], 
                                       localPosition0 = [0.,0.,0.], radiusSphere = 0,
                                       trianglePoints = exudyn.Vector3DList([[0,0,0],[1,0,0],[0,1,0]]),
                                       includeEdges = 7, dynamicFriction = 0., frictionProportionalZone = 1e-3,
                                       contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1,
                                       restitutionCoefficient = 1, minimumImpactVelocity = 0,
                                       impactModel = 0,
                                       dataInitialCoordinates = [0,0,0,0],
                                       activeConnector=True,
                                       bodyOrNodeList=[None, None], 
                                       localPosition1 = [0.,0.,0.], 
                                       show=False, color=exudyn.graphics.color.default):
    
    where = 'MainSystem.CreateSphereSphereContact(...)'
    internBodyNodeList = ProcessBodyNodeLists(bodyNumbers, bodyOrNodeList, localPosition0, localPosition1, where)

    if not exudyn.__useExudynFast:
        if not isinstance(name, str):
            RaiseTypeError(where=where, argumentName='name', received = name, expectedType = ExpectedType.String)

        if not IsVector(localPosition0, 3):
            RaiseTypeError(where=where, argumentName='localPosition0', received = localPosition0, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(localPosition1, 3):
            RaiseTypeError(where=where, argumentName='localPosition1', received = localPosition1, expectedType = ExpectedType.Vector, dim=3)
        if not IsValidPRealInt(radiusSphere):
            RaiseTypeError(where=where, argumentName='radiusSphere', received = radiusSphere, expectedType = ExpectedType.Real)
        if type(trianglePoints) != exudyn.Vector3DList or len(trianglePoints) != 3:
            RaiseTypeError(where=where, argumentName='trianglePoints', received = trianglePoints, expectedType = 'expected type=exudyn.Vector3DList of length 3')
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
        
    if isinstance(internBodyNodeList[0], exudyn.ObjectIndex):
        mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName0,bodyNumber=internBodyNodeList[0], localPosition=localPosition0))
    else:
        mBody0 = mbs.AddMarker(eii.MarkerNodeRigid(name=mName0,nodeNumber=internBodyNodeList[0]))

    if isinstance(internBodyNodeList[1], exudyn.ObjectIndex):
        mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(name=mName1,bodyNumber=internBodyNodeList[1], localPosition=localPosition1))
    else:
        mBody1 = mbs.AddMarker(eii.MarkerNodeRigid(name=mName1,nodeNumber=internBodyNodeList[1]))
    
    nGeneric = mbs.AddNode(eii.NodeGenericData(initialCoordinates=dataInitialCoordinates,
                                         numberOfDataCoordinates=len(dataInitialCoordinates)))
    oContact = mbs.AddObject(eii.ObjectContactSphereTriangle(markerNumbers=[mBody0, mBody1],
                                                    nodeNumber=nGeneric,
                                                    radiusSphere=radiusSphere,
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





#**function: helper function to create 2D or 3D mass point object and node, using arguments as in NodePoint and MassPoint; uses TreeLink as defined in exudyn.rigidBodyUtilities
#**input: 
#  mbs: the MainSystem where items are created
#  name: name string for object, node is 'Node:'+name
#  listOfTreeLinks: list of TreeLink (from exudyn.rigidBodyUtilities) which characterize the KinematicTree
#  referenceCoordinates: reference coordinates all kinematic tree coordinates (configuration when current coordinates are zero)
#  initialCoordinates: initial deviation from reference coordinates
#  initialVelocities: initial velocities for point node (always a 3D vector, no matter if 2D or 3D mass)
#  gravity: gravity vevtor applied to kinematic tree (always a 3D vector, no matter if 2D or 3D mass)
#  baseOffset: constant 3D vector representing the origin of the kinematic tree
#  linkForces: Vector3DList of forces per link (at joint origin) or None
#  linkTorques: Vector3DList of torques per link or None
#  jointForceVector: a list or numpy array of scalar forces per joint, representing joint forces (prismatic joint) or joint torques (revolute joint)
#  jointPositionOffsetVector: a list or numpy array of scalar set coordinates per joint; use PreStepUserFunction to change values over time
#  jointVelocityOffsetVector: a list or numpy array of scalar set velocities per joint; use PreStepUserFunction to change values over time
#  forceUserFunction: A Python user function which computes the generalized force vector on RHS with identical action as jointForceVector; for description see ObjectKinematicTree
#  show: show kinematic tree
#  showLinks: set true, if links shall be shown; if graphicsDataList is empty, a standard drawing for links is used (drawing a cylinder from previous joint or base to next joint; size relative to frame size in KinematicTree visualization settings); else graphicsDataList are used per link; NOTE visualization of joint and COM frames can be modified via visualizationSettings.bodies.kinematicTree
#  showJoints: set true, if joints shall be shown; if graphicsDataList is empty, a standard drawing for joints is used (drawing a cylinder for revolute joints; size relative to frame size in KinematicTree visualization settings)
#  jointRadius: for generic visualization of joints and links
#  jointWidth: for generic visualization of joints and links
#  colors: either one general color for kinematic tree, or list with one color per link
#  colorsJoints: either one color for all joints or list with one color per joint
#  baseGraphicsDataList: graphics for base; if None, it is computed automatically; otherwise a list of graphicsData or empty list
#  linkRoundness: for automatic generation of graphics for links, roundness=0 give brick-shape, roundness<1 give transition of brick to ellipsoid and roundness=1 give cylinders
#  show: show kinematic tree
#**output: ObjectIndex; returns kinematic tree object index
#**belongsTo: MainSystem
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
                           ): 

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
    jointTransformations=exu.Matrix3DList()
    jointOffsets = exu.Vector3DList()

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
        jointTransformations.Append(HT2rotationMatrix(link.jointHT) )
        jointOffsets.Append(HT2translation(link.jointHT) )
    
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
            
        v = HT2translation(link.jointHT)
        if addGraphics:
            #joints:
            if listOfTreeLinks[i].graphicsDataList is None:
                gJoint = exudyn.graphics.Cylinder(pAxis=-0.5*jointWidth*axis, vAxis=jointWidth*axis, 
                                                  radius=jointRadius,
                                                  color=jointColors[i])
                graphicsDataList[i].append(gJoint)

            if NormL2(v) > 0:
                #links:
                if linkRoundness < 1:
                    axis0 = Normalize(v)
                    axis2 = np.cross(axis0, parentAxis)
                    axis1 = -np.cross(axis0, axis2)
                    lenV = NormL2(v) #will always have some extension
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


            gDataList.append(gLink)
                
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
                                                jointTransformations=jointTransformations, 
                                                jointOffsets=jointOffsets,
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
#LinearSpringDamper
#TorsionalSpringDamper
#RollingDiscPenalty
#2x rolling disc
#CreateBeamsStraight[2D](...) #ANCF, GE with types?
#CreateBeamsCurved[2D](...)   #ANCF, GE


# #FUTURE:
# #def InitializeFromRestartFile(mbs, simulationSettings, restartFileName, verbose=True):

     
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
exu.MainSystem.CreateTorsionalSpringDamper=MainSystemCreateTorsionalSpringDamper


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
exu.MainSystem.CreateCoordinateConstraint=MainSystemCreateCoordinateConstraint


#link MainSystem function to Python function:
exu.MainSystem.CreateRollingDisc=MainSystemCreateRollingDisc


#link MainSystem function to Python function:
exu.MainSystem.CreateRollingDiscPenalty=MainSystemCreateRollingDiscPenalty


#link MainSystem function to Python function:
exu.MainSystem.CreateCircleCircleContact=MainSystemCreateCircleCircleContact


#link MainSystem function to Python function:
exu.MainSystem.CreateContactCurveCircles=MainSystemCreateContactCurveCircles


#link MainSystem function to Python function:
exu.MainSystem.CreateSphereSphereContact=MainSystemCreateSphereSphereContact


#link MainSystem function to Python function:
exu.MainSystem.CreateSphereQuadContact=MainSystemCreateSphereQuadContact


#link MainSystem function to Python function:
exu.MainSystem.CreateSphereTriangleContact=MainSystemCreateSphereTriangleContact


#link MainSystem function to Python function:
exu.MainSystem.CreateKinematicTree=MainSystemCreateKinematicTree


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
