#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details: 	This module provides an extension interface to the C++ class MainSystem;
#			MainSystem is extended by Python interface functions to easily create
#           bodies and point masses without the need to create an according node and
#           connectors and joints without the need to create markers.
#
# Author:   Johannes Gerstmayr
# Date:     2023-05-07 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.rigidBodyUtilities import GetRigidBodyNode
import exudyn.itemInterface as eii 
from exudyn.advancedUtilities import RaiseTypeError, IsVector, ExpectedType, IsValidObjectIndex, IsValidRealInt, IsValidPRealInt, IsValidURealInt,\
                                    IsValidBool
from exudyn.graphicsDataUtilities import color4default
import numpy as np

if not 'mainSystemInterface' in exu.sys:
    raise ValueError('mainSystemInterface is only for testing; it requires to resolve the problem of type completion prior to full implementation')

#steps to do:
#naming: mbs.CreateMassPoint(...), mbs.CreateRevoluteJoint(...)
#create autoGenerateMBScreators.py
#==>dicts to define arguments, type, defaults, description
#   automatically create checks
#   add code section or autocode
#   bodyNumber0=>index0

#**function: add helpful Python extensions for MainSystem, regarding creation of bodies, point masses, connectors and joints
def MainSystemAddPythonFunctions():
    
    #**function: helper function to create 2D or 3D mass point object and node
    def MainSystemAddMassPoint(self,#node quantities
                               referenceCoordinates = [0.,0.,0.],
                               initialCoordinates = [0.,0.,0.],
                               initialVelocities = [0.,0.,0.],
                               #object quantities:
                               physicsMass=0,
                               gravity = [0.,0.,0.],
                               graphicsDataList = [],
                               #graphics, mixed
                               drawSize = -1,
                               color =  [-1.,-1.,-1.,-1.],
                               show = True, #if graphicsDataList is empty, node is shown, otherwise body is shown
                               name = '',   #both for node and object
                               create2D = False, #NodePoint2D, MassPoint2D
                               returnDict = False, #if True, returns dictionary of all data
                               ):

        mainSystem = self
        #error checks:        
        if not IsVector(referenceCoordinates, 3):
            RaiseTypeError(where='MainSystem.AddMassPoint(...)', argumentName='referenceCoordinates', received = referenceCoordinates, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(initialCoordinates, 3):
            RaiseTypeError(where='MainSystem.AddMassPoint(...)', argumentName='initialCoordinates', received = initialCoordinates, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(initialVelocities, 3):
            RaiseTypeError(where='MainSystem.AddMassPoint(...)', argumentName='initialVelocities', received = initialVelocities, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(gravity, 3):
            RaiseTypeError(where='MainSystem.AddMassPoint(...)', argumentName='gravity', received = gravity, expectedType = ExpectedType.Vector, dim=3)
    
        if type(graphicsDataList) != list:
            raise ValueError('MainSystem.AddMassPoint(...): graphicsDataList must be a (possibly empty) list of dictionaries of graphics data!')
    
        nodeNumber = mainSystem.AddNode(eii.NodePoint(name = name,
                         referenceCoordinates = referenceCoordinates,
                         initialCoordinates=initialCoordinates,
                         initialVelocities=initialVelocities,
                         visualization = eii.VNodePoint (show = show and (graphicsDataList == []), drawSize = drawSize, color = color),
                         ))
        objectNumber = mainSystem.AddObject(eii.MassPoint(name = name, 
                                                physicsMass=physicsMass,
                                                nodeNumber = nodeNumber,
                                                visualization = eii.VMassPoint(show = graphicsDataList != [], 
                                                                           graphicsData = graphicsDataList)
                                                ))
        if returnDict:
            rDict = {'nodeNumber':nodeNumber, 'objectNumber': objectNumber}
        
        if list(gravity) != [0.,0.,0.]: #        if NormL2(gravity) != 0.:
            markerNumber = mainSystem.AddMarker(eii.MarkerBodyMass(bodyNumber=objectNumber))
            loadNumber = mainSystem.AddLoad(eii.LoadMassProportional(markerNumber=markerNumber, loadVector=gravity))
            if returnDict:
                rDict['markerBodyMass'] = markerNumber
                rDict['loadNumber'] = loadNumber

        if returnDict:
            return rDict
        else:
            return objectNumber

    def MainSystemAddSpringDamper(self,
                                  bodyNumber0, bodyNumber1, 
                                  localPosition0 = [0.,0.,0.],
                                  localPosition1 = [0.,0.,0.],
                                  referenceLength = None, 
                                  stiffness = 0., damping = 0., force = 0.,
                                  velocityOffset = 0., 
                                  show=True, drawSize=-1, color=color4default,
                                  ):
        mainSystem = self
        #perform some checks:
        if not IsValidObjectIndex(bodyNumber0):
            RaiseTypeError(where='MainSystem.AddSpringDamper(...)', argumentName='bodyNumber0', received = bodyNumber0, expectedType = ExpectedType.ObjectIndex)
        if not IsValidObjectIndex(bodyNumber1):
            RaiseTypeError(where='MainSystem.AddSpringDamper(...)', argumentName='bodyNumber1', received = bodyNumber1, expectedType = ExpectedType.ObjectIndex)

        if not IsVector(localPosition0, 3):
            RaiseTypeError(where='MainSystem.AddMassPoint(...)', argumentName='localPosition0', received = localPosition0, expectedType = ExpectedType.Vector, dim=3)
        if not IsVector(localPosition1, 3):
            RaiseTypeError(where='MainSystem.AddMassPoint(...)', argumentName='localPosition1', received = localPosition1, expectedType = ExpectedType.Vector, dim=3)

        if referenceLength != None and not IsValidPRealInt(referenceLength):
            RaiseTypeError(where='MainSystem.AddSpringDamper(...)', argumentName='referenceLength', received = referenceLength, expectedType = ExpectedType.PReal)
        if not IsValidRealInt(stiffness):
            RaiseTypeError(where='MainSystem.AddSpringDamper(...)', argumentName='stiffness', received = stiffness, expectedType = ExpectedType.Real)
        if not IsValidRealInt(damping):
            RaiseTypeError(where='MainSystem.AddSpringDamper(...)', argumentName='damping', received = damping, expectedType = ExpectedType.Real)
        if not IsValidRealInt(force):
            RaiseTypeError(where='MainSystem.AddSpringDamper(...)', argumentName='force', received = force, expectedType = ExpectedType.Real)
        if not IsValidRealInt(velocityOffset):
            RaiseTypeError(where='MainSystem.AddSpringDamper(...)', argumentName='velocityOffset', received = velocityOffset, expectedType = ExpectedType.Real)

        if not IsValidBool(show):
            RaiseTypeError(where='MainSystem.AddSpringDamper(...)', argumentName='show', received = show, expectedType = ExpectedType.Bool)
        if not IsValidRealInt(drawSize):
            RaiseTypeError(where='MainSystem.AddSpringDamper(...)', argumentName='drawSize', received = drawSize, expectedType = ExpectedType.Real)

        if not IsVector(color, 4):
            RaiseTypeError(where='MainSystem.AddSpringDamper(...)', argumentName='color', received = color, expectedType = ExpectedType.Vector, dim=4)
            
        mBody0 = mainSystem.AddMarker(eii.MarkerBodyPosition(bodyNumber=bodyNumber0, localPosition=localPosition0))
        mBody1 = mainSystem.AddMarker(eii.MarkerBodyPosition(bodyNumber=bodyNumber1, localPosition=localPosition1))

        if referenceLength == None: #automatically compute reference length
            p0 = mainSystem.GetObjectOutputBody(bodyNumber0,exu.OutputVariableType.Position,
                                         localPosition=localPosition0, configuration=exu.ConfigurationType.Reference)
            p1 = mainSystem.GetObjectOutputBody(bodyNumber1,exu.OutputVariableType.Position,
                                         localPosition=localPosition1, configuration=exu.ConfigurationType.Reference)
            referenceLength = np.linalg.norm(np.array(p1)-p0)
        
        oConnector = mainSystem.AddObject(eii.ObjectConnectorSpringDamper(markerNumbers = [mBody0,mBody1],
                                                                          referenceLength = referenceLength,
                                                                          stiffness = stiffness,
                                                                          damping = damping,
                                                                          force = force,
                                                                          velocityOffset = velocityOffset,
                                                                          visualization=eii.VSpringDamper(show=show, drawSize=drawSize,
                                                                                                          color=color)
                                                                          ))

        return oConnector


    exu.MainSystem.AddMassPoint = MainSystemAddMassPoint
    exu.MainSystem.AddSpringDamper = MainSystemAddSpringDamper


MainSystemAddPythonFunctions()

