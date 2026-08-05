#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test models for mainSystemExtensions; tests for functions except PlotSensor or SolutionViewer, 
#           which are tested already in other functions or cannot be tested with test suite;
#           all tests are self-contained and are included as examples for docu
#
# Author:   Johannes Gerstmayr
# Date:     2023-05-19
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
import numpy as np

useGraphics = True #without test
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#you can erase the following lines and all exudynTestGlobals related operations if this is not intended to be used as TestModel:
try: #only if called from test suite
    from modelUnitTests import exudynTestGlobals #for globally storing test results
    useGraphics = exudynTestGlobals.useGraphics
except:
    class ExudynTestGlobals:
        pass
    exudynTestGlobals = ExudynTestGlobals()
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

## set up MainSystem mbs
SC = exu.SystemContainer()
mbs = SC.AddSystem()

## overall parameters
L = 0.4 #length of bodies
d = 0.1 #diameter of bodies
p0 = [0.,0.,0] #reference position
vLoc = np.array([L,0,0]) #last to next joint
#g = [0,0,-9.81]
g = [0,-9.81,0]

## create ground and ground marker
oGround=mbs.AddObject(ObjectGround(referencePosition= [-0.5*L,0,0])) 
mPosLast = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oGround, localPosition=[0,0,0]))
A0 = np.eye(3)
Alast = A0 #previous marker
bodyLast = oGround

## set up rotation matrices for relative rotation of joints
A0 = RotationMatrixX(0)
A1 = RotationMatrixY(0.5*pi)
A2 = RotationMatrixZ(0.5*pi*0)
A3 = RotationMatrixX(-0.5*pi*0)
Alist=[A0,A1,A2,A3,A1]

## set up list of vectors defining global axes
v0=[0,0,1]
v1=[1,1,1]
v2=[0,1,0]
v3=[0,0,1]
v4=[1,1,0]

# v1=[0,0,1]
# v2=[0,1,0]

axisList=[v0,v1,v2,v3,v4]

markerList=[]
sensorList=[]
refValuesList=[]
rotJoints = [0,1,1,0,1]

## for loop to create a chain of 4 bodies under gravity connected with revolute joints
for i in range(4):
    ### create inertia for block with dimensions [L,d,d] and graphics for block
    inertia = InertiaCuboid(density=1000, sideLengths=[L,d,d])
    graphicsBody = graphics.Brick([0,0,0], [0.96*L,d,d], graphics.color.steelblue)

    #axis = [0,0,1]
    axis = np.array(axisList[i]) #global axis!
    axis = axis/np.linalg.norm(axis)
    #print('axis=',axis)

    ### create and add rigid body to mbs
    p0 += Alist[i] @ (0.5*vLoc)
    angVel = -2 * pi * (Alast @ np.array(axis))
    oRB = mbs.CreateRigidBody(inertia=inertia, 
                              referencePosition=p0,
                              # initialAngularVelocity=angVel,
                              referenceRotationMatrix=Alist[i],
                              gravity=g,
                              graphicsDataList=[graphicsBody])
    nRB= mbs.GetObject(oRB)['nodeNumber']

    body0 = bodyLast
    body1 = oRB
    ### retrieve reference position for simpler definition of global joint position
    point = mbs.GetObjectOutputBody(oRB,exu.OutputVariableType.Position,
                                    localPosition=[-0.5*L,0,0],
                                    configuration=exu.ConfigurationType.Reference)

    if rotJoints[i]:
        ### set up revolute joint between two bodies, at global position and with global axis
        oJoint = mbs.CreateRevoluteJoint(bodyNumbers=[body0, body1], 
                                position=point, axis=axis, useGlobalFrame=True, 
                                axisRadius=0.6*d, axisLength=1.2*d)
    else:
        ### set up prismatic joint between two bodies, at global position and with global axis
        oJoint = mbs.CreatePrismaticJoint(bodyNumbers=[body0, body1], position=point, axis=axis, 
                                          useGlobalFrame=True, axisRadius=0.6*d, axisLength=1.2*d)


    markers = mbs.GetObject(oJoint)['markerNumbers']
    localPosition0 = mbs.GetMarker(markers[0])['localPosition']
    localPosition1 = mbs.GetMarker(markers[1])['localPosition']
    
    if rotJoints[i]:
        nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[0], numberOfDataCoordinates=1))
        mRelative = mbs.AddMarker(MarkerBodiesRelativeRotationCoordinate(bodyNumbers=[body0, body1], 
                                                                         nodeNumber=nGeneric,
                                                                         localPosition0=localPosition0, localPosition1=localPosition1,
                                                                         axis0 = Alast.T@axis))
    else:
        mRelative = mbs.AddMarker(MarkerBodiesRelativeTranslationCoordinate(bodyNumbers=[body0, body1], 
                                                                localPosition0=localPosition0, localPosition1=localPosition1,
                                                                axis0 = Alast.T@axis))
    markerList.append(mRelative)

    bodyLast = oRB

    p0 += Alist[i] @ (0.5*vLoc)
    Alast = Alist[i]

    sMarker = mbs.AddSensor(SensorMarker(markerNumber=mRelative, storeInternal=True, 
                                   outputVariableType=exu.OutputVariableType.Coordinates))
    sensorList.append(sMarker)

#++++++++++++++++++++++++++++++++++++
mbs.Assemble() #needed to compute marker output for current configuration

#compute values in reference/initial configuration in order to avoid jumps in constraints
for mRelative in markerList:
    refValue = mbs.GetMarkerOutput(mRelative, exu.OutputVariableType.Coordinates)[0]
    print('refValue=',refValue)
    refValuesList.append(refValue)
    mbs.SetMarkerParameter(mRelative, 'offset', refValue)


#++++++++++++++++++++++++++++++++++++
#constraints to connect rotations/translations

def UFoffset0(mbs, t, itemNumber, lOffset):
    return 0.3*(1-cos(2*pi*t))
    # return 1.2*(sin(0.5*pi*t))

nGround = mbs.AddNode(NodePointGround())
mcGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))

cc0 = mbs.AddObject(CoordinateConstraint(markerNumbers=[mcGround, markerList[0]], 
                                          offset = 0.,
                                          offsetUserFunction=UFoffset0))

cc1 = mbs.AddObject(CoordinateConstraint(markerNumbers=[markerList[0], markerList[1]], 
                                          factorValue1=1./(2*pi),
                                          # velocityLevel=True
                                          ))

cc2 = mbs.AddObject(CoordinateConstraint(markerNumbers=[markerList[1], markerList[2]], 
# cc2 = mbs.AddObject(CoordinateConstraint(markerNumbers=[mcGround, markerList[2]], 
                                          factorValue1=-1,
                                          # velocityLevel=True
                                          offsetUserFunction=UFoffset0
                                          ))
cc3 = mbs.AddObject(CoordinateConstraint(markerNumbers=[markerList[2], markerList[3]], 
                                          factorValue1=3*(2*pi),
                                          # velocityLevel=True
                                          ))


#++++++++++++++++++++++++++++++++++++


## assemble and set up simulation settings
mbs.Assemble()

# print('cc2=',mbs.GetObjectOutput(cc2, exu.OutputVariableType.ConstraintEquation))

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 2
h=0.001  #use small step size to detext contact switching

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.solutionWritePeriod = 0.01
simulationSettings.solutionSettings.sensorsWritePeriod = 0.01
simulationSettings.timeIntegration.verboseMode = 1 #print some progress

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
simulationSettings.timeIntegration.newton.useModifiedNewton = True

SC.visualizationSettings.markers.defaultSize = 0.1
SC.visualizationSettings.markers.drawSimplified = False
SC.visualizationSettings.nodes.show = True
SC.visualizationSettings.nodes.drawNodesAsPoint  = False
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.nodes.basisSize = 0.015
SC.visualizationSettings.connectors.showJointAxes = True

SC.visualizationSettings.openGL.multiSampling=2
SC.visualizationSettings.openGL.lineWidth=2
SC.visualizationSettings.view0.window.renderWindowSize = [1600,1200]
SC.visualizationSettings.view0.scene.drawCoordinateSystem=True

SC.visualizationSettings.general.autoFitScene = False #use loaded render state
simulationSettings.displayComputationTime = True
simulationSettings.displayStatistics = True


if useGraphics:
    SC.renderer.Start()
    if 'renderState' in exu.sys:
        SC.renderer.SetState(exu.sys['renderState'])
    SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings = simulationSettings)

if useGraphics:
    SC.renderer.Stop()


if useGraphics:
    mbs.SolutionViewer()

    if len(sensorList):
        mbs.PlotSensor(sensorList)

#%%++++
testError = np.linalg.norm(mbs.systemData.GetODE2Coordinates())
exu.Print('solution of relativeRotationTranslationMechanism=',testError)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++

exudynTestGlobals.testError = testError - (0.0)   #2023-06-12: 4.172189649307425
exudynTestGlobals.testResult = testError


