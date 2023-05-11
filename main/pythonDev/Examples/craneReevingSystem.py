#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A crane model using the ReevingSystemSprings for the rope 
#
# Author:   Johannes Gerstmayr
# Date:     2022-06-16
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *

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

import numpy as np
from math import sin, cos, sqrt,pi

SC = exu.SystemContainer()
mbs = SC.AddSystem()

gGround = GraphicsDataCheckerBoard(point=[0,0,0], normal = [0,1,0], size=60, nTiles=12)
oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[gGround])))

tRoll = 0.05    #thickness rolls (graphics)
rHook = 0.2     #radius of Hook rolls
rCarr = 0.3     #radius of Carriage rolls
g = [0,-9.81,0]
colorRolls = color4red

#rope parameters:
rRope = 0.025 #drawing radius
A = rRope**2*pi
EArope = 1e9*A
print('EArope=',EArope)
stiffnessRope = EArope #stiffness per length
dampingRope = 0.1*stiffnessRope #dampiung per length
dampingRopeTorsional = 1e-4*dampingRope
dampingRopeShear = 0.1*dampingRope*0.001


#%% +++++++++++++++++++++++++++++++
#crane: (height=Y, arm=X)
H = 40
L = 30
Dtower = 1.5
Darm = 1
Lcarr = 1
Dcarr = 0.6
Lhook = 0.5
Dhook = 0.5

carrYoff = 0.3*Darm
hookZoff = 0.4*Dhook
hookYoff = 0.5*H
#hookZoffVec = np.array([0,0,hookZoff])

posTower = np.array([0,0.5*H,0])
posArm = 2*posTower + np.array([0.5*L,0,0])

sJoint = 0.5 #overal joint size for main joints

#++++++++++++++++++++++++++++
#node for prescribed motion
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))
mNodeGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))#andy coordinate is zero
nCoordCarr = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0], initialCoordinates=[0], 
                                         initialCoordinates_t=[0], numberOfODE2Coordinates=1))
nCoordHook = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0], initialCoordinates=[0], 
                                         initialCoordinates_t=[0], numberOfODE2Coordinates=1))

mbs.AddObject(Mass1D(physicsMass=1, nodeNumber=nCoordCarr))
mbs.AddObject(Mass1D(physicsMass=1, nodeNumber=nCoordHook))

mNodeCarr = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nCoordCarr, coordinate=0))
mNodeHook = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nCoordHook, coordinate=0))

ccCarr = mbs.AddObject(CoordinateConstraint(markerNumbers=[mNodeGround, mNodeCarr], offset=0))
ccHook = mbs.AddObject(CoordinateConstraint(markerNumbers=[mNodeGround, mNodeHook], offset=0))
#++++++++++++++++++++++++++++
#tower
Vtower = Dtower*Dtower*H
inertiaTower = InertiaCuboid(2000/Vtower, [Dtower,H,Dtower])

#model tower as body, may be moved as well ...
graphicsTower = [GraphicsDataOrthoCubePoint([0,0,0],[Dtower,H,Dtower],color=color4grey, addEdges = True)]
graphicsTower += [GraphicsDataCylinder([0,0.5*H-Darm*0.5,0],[0,0.5*Darm,0],radius=1.1*Darm, color=color4grey)]
[nTower,bTower]=AddRigidBody(mainSys = mbs, 
                      inertia = inertiaTower, 
                      nodeType = exu.NodeType.RotationEulerParameters, 
                      position = posTower,
                      gravity = g, 
                      graphicsDataList = graphicsTower)


markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
markerTowerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bTower, localPosition=[0,-0.5*H,0]))
oJointTower = mbs.AddObject(GenericJoint(markerNumbers=[markerGround, markerTowerGround],
                                    constrainedAxes=[1,1,1,1,1,1],
                                    visualization=VGenericJoint(axesRadius=0.5*sJoint, axesLength=1.5*sJoint)))


#++++++++++++++++++++++++++++
#carriage, initially located at midspan or arm
Vcarr = Dcarr*Dcarr*Lcarr

inertiaCarr = InertiaCuboid(100/Vcarr, [Lcarr,Dcarr,Dcarr])
posCarr = posArm + np.array([0,-carrYoff,0])

zRoll = np.array([0,0,tRoll])
pRollCarr =  [None,None,None,None,None]
pRollCarr[0] = [-0.5*Lcarr,0,-hookZoff]
pRollCarr[1] = [-0.5*Lcarr,0, hookZoff]
pRollCarr[2] = [ 0.5*Lcarr,0,-hookZoff]
pRollCarr[3] = [ 0.5*Lcarr,0, hookZoff]
pRollCarr[4] = [ 0.5*Lcarr,0, 0*hookZoff]

#model tower as body, may be moved as well ...
graphicsCarr = []
for p in pRollCarr:
    graphicsCarr += [GraphicsDataCylinder(p-0.5*zRoll,zRoll,radius=rHook, color=colorRolls, addEdges=True)]

graphicsCarr += [GraphicsDataOrthoCubePoint([0,0,0],[1.2*Lcarr,0.2*Dcarr,1.2*Dcarr],color=color4grey[0:3]+[0.5], addEdges = True)]
[nCarr,bCarr]=AddRigidBody(mainSys = mbs, 
                      inertia = inertiaCarr, 
                      nodeType = exu.NodeType.RotationEulerParameters, 
                      position = posCarr, 
                      angularVelocity=[0,0,0],
                      gravity = g, 
                      graphicsDataList = graphicsCarr)


#++++++++++++++++++++++++++++
#arm
Varm = Darm*Darm*L
inertiaArm = InertiaCuboid(2000/Varm, [L,Darm,Darm])

pRollArm =  [None,None,None]
pRollArm[0] = [-0.5*L, rCarr,0]
pRollArm[1] = [ 0.5*L,0     ,0]
pRollArm[2] = [-0.5*L,-rCarr,0]
rRollArm = [0,rCarr,0]

#model tower as body, may be moved as well ...
graphicsArm = []
for i,p in enumerate(pRollArm):
    graphicsArm += [GraphicsDataCylinder(p-0.5*zRoll,zRoll,radius=max(0.1*rCarr,rRollArm[i]), color=colorRolls, addEdges=True)]

graphicsArm += [GraphicsDataOrthoCubePoint([-0.1*L,0,0],[L*1.2,Darm,Darm],color=[0.3,0.3,0.9,0.5], addEdges = True)]
[nArm,bArm]=AddRigidBody(mainSys = mbs, 
                      inertia = inertiaArm, 
                      nodeType = exu.NodeType.RotationEulerParameters, 
                      position = posArm, 
                      angularVelocity=[0,0.1*0,0],
                      gravity = g, 
                      graphicsDataList = graphicsArm)

markerTowerArm = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bTower, localPosition=[0,0.5*H,0]))
markerArmTower = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bArm, localPosition=[-0.5*L,0,0]))
oJointArm = mbs.AddObject(GenericJoint(markerNumbers=[markerTowerArm, markerArmTower],
                                    constrainedAxes=[1,1,1,1,0,1],
                                    visualization=VGenericJoint(axesRadius=0.5*sJoint, axesLength=1.5*sJoint)))

markerArmCarr = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bArm, localPosition=[0,-carrYoff,0]))
markerCarrArm = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCarr, localPosition=[0,0,0]))
oJointCarr = mbs.AddObject(GenericJoint(markerNumbers=[markerArmCarr, markerCarrArm],
                                    constrainedAxes=[0,1,1,1,1,1],
                                    visualization=VGenericJoint(axesRadius=0.5*sJoint, axesLength=1.5*sJoint)))

#++++++++++++++++++++++++++++
#Reeving system for motion of carriage
markerListCarriage1 = []
markerListCarriage1+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bArm, localPosition=pRollArm[0]))]
markerListCarriage1+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bArm, localPosition=pRollArm[1]))]
markerListCarriage1+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCarr, localPosition=[ 0.5*Lcarr,0,0]))]
markerListCarriage1+=[mNodeCarr,mNodeGround]

markerListCarriage2 = []
markerListCarriage2+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bArm, localPosition=pRollArm[2]))]
markerListCarriage2+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCarr, localPosition=[-0.5*Lcarr,0,0]))]
markerListCarriage2+=[mNodeCarr,mNodeGround]

LrefRopeCarriage1 = L+pi*rCarr+0.5*L-0.5*Lcarr
LrefRopeCarriage2 = 0.5*L-0.5*Lcarr

sheavesAxes1 = exu.Vector3DList()
for i, radius in enumerate(rRollArm):
    sheavesAxes1.Append([0,0,-1])

#no radius needed, just two points:
sheavesAxes2 = exu.Vector3DList()
sheavesAxes2.Append([0,0,-1])
sheavesAxes2.Append([0,0,-1])

oRScarr1=mbs.AddObject(ReevingSystemSprings(markerNumbers=markerListCarriage1, 
                                            hasCoordinateMarkers=True, coordinateFactors=[-1,0],#negative direction X
                                            stiffnessPerLength=stiffnessRope, dampingPerLength=dampingRope, referenceLength = LrefRopeCarriage1,
                                            dampingTorsional = dampingRopeTorsional, dampingShear = dampingRopeShear*0,
                                            sheavesAxes=sheavesAxes1, sheavesRadii=rRollArm,
                                            visualization=VReevingSystemSprings(ropeRadius=rRope, color=color4lawngreen)))

oRScarr2=mbs.AddObject(ReevingSystemSprings(markerNumbers=markerListCarriage2, 
                                            hasCoordinateMarkers=True, coordinateFactors=[1,0], #positive direction X
                                            stiffnessPerLength=stiffnessRope, dampingPerLength=dampingRope, referenceLength = LrefRopeCarriage2,
                                            dampingTorsional = dampingRopeTorsional*0,
                                            sheavesAxes=sheavesAxes2, sheavesRadii=[0,0],
                                            visualization=VReevingSystemSprings(ropeRadius=rRope, color=color4lawngreen)))

#++++++++++++++++++++++++++++
#hook
Vhook = Dhook*Dhook*Lhook

inertiaHook = InertiaCuboid(100/Vhook, [Lhook,Dhook,Dhook])
inertiaHook = inertiaHook.Translated([0,-Dhook,0])
posHook = posCarr + np.array([0,-hookYoff,0])


pRollHook =  [None,None,None,None]
pRollHook[0] = [-0.5*Lhook,0,-hookZoff]
pRollHook[1] = [-0.5*Lhook,0, hookZoff]
pRollHook[2] = [ 0.5*Lhook,0,-hookZoff]
pRollHook[3] = [ 0.5*Lhook,0, hookZoff]

#model tower as body, may be moved as well ...
graphicsHook = []
for p in pRollHook:
    graphicsHook += [GraphicsDataCylinder(p-0.5*zRoll,zRoll,radius=rHook, color=colorRolls, addEdges=True)]

graphicsHook += [GraphicsDataOrthoCubePoint([0,0,0],[Lhook,0.2*Dhook,Dhook],color=color4grey[0:3]+[0.5], addEdges = True)]
graphicsHook += [GraphicsDataOrthoCubePoint([0,-Dhook,0],[4*Lhook,2*Dhook,2*Dhook],color=color4grey[0:3]+[0.5], addEdges = True)]
[nHook,bHook]=AddRigidBody(mainSys = mbs, 
                      inertia = inertiaHook, 
                      nodeType = exu.NodeType.RotationEulerParameters, 
                      position = posHook, 
                      angularVelocity=[0,0,0],
                      gravity = g, 
                      graphicsDataList = graphicsHook)


markerListHook = []
markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bArm, localPosition=[-0.5*L,-carrYoff+2*rHook,0]))]
markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCarr, localPosition=pRollCarr[0]))]
markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bHook, localPosition=pRollHook[0]))]
markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCarr, localPosition=pRollCarr[1]))]
markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bHook, localPosition=pRollHook[1]))]
markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCarr, localPosition=pRollCarr[3]))]
markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bHook, localPosition=pRollHook[3]))]
markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCarr, localPosition=pRollCarr[2]))]
markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bHook, localPosition=pRollHook[2]))]
markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCarr, localPosition=pRollCarr[4]))]
markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bArm, localPosition=[ 0.5*L,-carrYoff+2*rHook,0]))]
markerListHook+=[mNodeHook,mNodeGround]

LrefRopeHook = 8*0.5*H+L+8*pi*rHook

#no radius needed, just two points:
sheavesAxesHook = exu.Vector3DList()
sheavesAxesHook.Append([0,0,1])
sheavesAxesHook.Append([0,0,-1])
sheavesAxesHook.Append([0,0,1]) #Hook0
sheavesAxesHook.Append([0,0,1])
sheavesAxesHook.Append([0,0,1]) #Hook1
sheavesAxesHook.Append([0,0,-1])
sheavesAxesHook.Append([0,0,-1]) #Hook2
sheavesAxesHook.Append([0,0,-1])
sheavesAxesHook.Append([0,0,-1]) #Hook3
sheavesAxesHook.Append([0,0,-1])
sheavesAxesHook.Append([0,0,1])

radiiRollHook = []
for i in range(len(sheavesAxesHook)):
    radiiRollHook += [rHook]

oRScarr1=mbs.AddObject(ReevingSystemSprings(markerNumbers=markerListHook, 
                                            hasCoordinateMarkers=True, coordinateFactors=[1,0],
                                            stiffnessPerLength=stiffnessRope, dampingPerLength=dampingRope*0.1, referenceLength = LrefRopeHook,
                                            dampingTorsional = dampingRopeTorsional*0.1, dampingShear = dampingRopeShear,
                                            sheavesAxes=sheavesAxesHook, sheavesRadii=radiiRollHook,
                                            visualization=VReevingSystemSprings(ropeRadius=rRope, color=color4dodgerblue)))




#%% +++++++++++++++++++++++++++++++
# #add sensors 
# if True:
#     sPos1 = mbs.AddSensor(SensorNode(nodeNumber=nodeList[1], storeInternal=True,
#                                           outputVariableType=exu.OutputVariableType.Position))
#     sOmega1 = mbs.AddSensor(SensorNode(nodeNumber=nodeList[1], storeInternal=True,
#                                           outputVariableType=exu.OutputVariableType.AngularVelocity))
#     sLength= mbs.AddSensor(SensorObject(objectNumber=oRS, storeInternal=True,
#                                           outputVariableType=exu.OutputVariableType.Distance))
#     sLength_t= mbs.AddSensor(SensorObject(objectNumber=oRS, storeInternal=True,
#                                           outputVariableType=exu.OutputVariableType.VelocityLocal))


def PreStepUserFunction(mbs, t):
    if t <= 10:
        mbs.SetObjectParameter(ccCarr, 'offset', SmoothStep(t,  0, 10, 0, 0.45*L))
    elif t <= 20:
        mbs.SetObjectParameter(ccHook, 'offset', SmoothStep(t, 10, 20, 0, -8*0.40*H))
    elif t <= 30:
        mbs.SetObjectParameter(ccCarr, 'offset', SmoothStep(t, 20, 30, 0.45*L,-0.4*L))
    elif t <= 40:
        mbs.SetObjectParameter(ccHook, 'offset', SmoothStep(t, 30, 40, -8*0.40*H,8*0.45*H))
    else:
        mbs.SetObjectParameter(ccCarr, 'offset', SmoothStep(t, 40, 57, -0.4*L, 0.45*L))
        mbs.SetObjectParameter(ccHook, 'offset', SmoothStep(t, 40, 60, 8*0.45*H, 6*0.45*H))

    return True

mbs.SetPreStepUserFunction(PreStepUserFunction)

#%%++++++++++++++++++++++++++++++++++++++++++++++++
#simulate:
mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 80
h=0.001

solutionFile = 'solution/coordsCrane.txt'

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.writeSolutionToFile= True #set False for CPU performance measurement
simulationSettings.solutionSettings.solutionWritePeriod= 0.2
simulationSettings.solutionSettings.coordinatesSolutionFileName = solutionFile
simulationSettings.solutionSettings.sensorsWritePeriod = 0.01
# simulationSettings.timeIntegration.simulateInRealtime=True
# simulationSettings.timeIntegration.realtimeFactor=5
SC.visualizationSettings.general.graphicsUpdateInterval = 0.01
simulationSettings.parallel.numberOfThreads=4
simulationSettings.displayComputationTime = True

simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.useModifiedNewton = True

SC.visualizationSettings.nodes.show = True
SC.visualizationSettings.nodes.drawNodesAsPoint  = False
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.nodes.basisSize = 0.2

SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.openGL.shadow = 0.3*0
SC.visualizationSettings.openGL.light0position = [-50,200,100,0]

SC.visualizationSettings.window.renderWindowSize=[1920,1200]
#SC.visualizationSettings.general.autoFitScene = False #use loaded render state
useGraphics = True
if useGraphics:
    exu.StartRenderer()
    if 'renderState' in exu.sys:
        SC.SetRenderState(exu.sys[ 'renderState' ])
    mbs.WaitForUserToContinue()


exu.SolveDynamic(mbs, simulationSettings, 
                 #solverType=exu.DynamicSolverType.TrapezoidalIndex2 #in this case, drift shows up significantly!
                 )

if useGraphics:
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!


if True:
    #%%++++++++++++
    from exudyn.interactive import SolutionViewer
    SC.visualizationSettings.general.autoFitScene = False
    # solution = LoadSolutionFile(solutionFile)
    SolutionViewer(mbs) #loads solution file via name stored in mbs

#%%++++++++++++
if False:
    from exudyn.plot import PlotSensor

    PlotSensor(mbs, sPos1, components=[0,1,2], labels=['pos X','pos Y','pos Z'], closeAll=True)
    PlotSensor(mbs, sOmega1, components=[0,1,2], labels=['omega X','omega Y','omega Z'])
    PlotSensor(mbs, sLength, components=[0], labels=['length'])
    PlotSensor(mbs, sLength_t, components=[0], labels=['vel'])


#compute error for test suite:
sol2 = mbs.systemData.GetODE2Coordinates(); 
u = np.linalg.norm(sol2); 
exu.Print('solution of craneReevingSystem=',u)

exudynTestGlobals.testResult = u
