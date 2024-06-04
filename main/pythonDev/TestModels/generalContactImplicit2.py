#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  test implicit integration of contact with spheres and triangles, focus on normal contact
#
# Author:   Johannes Gerstmayr
# Date:     2024-06-02
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
#import exudyn.graphics as graphics #only import if it does not conflict


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

SC = exu.SystemContainer()
mbs = SC.AddSystem()

doImplicit = True
#%%+++++++++++++++++++++++++++++++++
#sphere-sphere with coordinate constraints, prestressed; fixed torque on one side, linear increasing torque on other side
#sphere on ground, rolling
#cube on ground, sliding (f=[f, f*mu*t, 0]), tangential force changing
#cube on ground with initial velocity
#cube-cube contact (meshed)


L = 1 #surrounding
a = 0.1 #base dimention of objects
r = 0.5*a #radius
t = 0.25*a #thickness

#contact coefficients:
mu = 0.1      #dry friction
m = 0.025     #mass
k = 1e3       #(linear) normal contact stiffness
d = 2e-4*k  #(linear) contact damping
gFact = 10
g = [0,0,-gFact]

gContact = mbs.AddGeneralContact()
gContact.verboseMode = 1
#gContact.sphereSphereContact = False
gContact.frictionProportionalZone = 1e-3*0.1
#gContact.excludeDuplicatedTrigSphereContactPoints = False
fricMat = mu*np.eye(1)
gContact.SetFrictionPairings(fricMat)
gContact.SetSearchTreeCellSize(numberOfCells=[4,4,4])

#%% ground
p0 = np.array([0,0,-0.5*t])
color4wall = [0.9,0.9,0.7,0.5]
addNormals = False
gFloor = GraphicsDataOrthoCubePoint(p0,[L,L,t],color4steelblue,addNormals)

gDataList = [gFloor]


nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0] ))
mGround = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGround))
mGroundC = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))

[meshPoints, meshTrigs] = GraphicsData2PointsAndTrigs(gFloor)
#[meshPoints, meshTrigs] = RefineMesh(meshPoints, meshTrigs) #just to have more triangles on floor
# [meshPoints, meshTrigs] = RefineMesh(meshPoints, meshTrigs) #just to have more triangles on floor
gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mGround, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,
    pointList=meshPoints,  triangleList=meshTrigs)

if True: #looses color
    gFloor = GraphicsDataFromPointsAndTrigs(meshPoints, meshTrigs, color=color4wall) #show refined mesh
    gDataList = [gFloor]

evalNodes = [] #collect nodes that are evaluated for test
#%%++++++++++++++++++++++++++++++++++++++++++++
#free rolling sphere:
gList = [GraphicsDataSphere(point=[0,0,0], radius=r, color= color4red, nTiles=24)]
gList += [GraphicsDataBasis(length=2*r)]

pRef = [-0.4*L,0*L,r - m*gFact/(0.5*k)] #need to use series of 2 k's!

RBinertia = InertiaSphere(m, r)
oMass = mbs.CreateRigidBody(inertia=RBinertia, 
                            #nodeType=exu.NodeType.RotationRotationVector,
                            nodeType=exu.NodeType.RotationRxyz,
                            referencePosition=pRef,
                            #referenceRotationMatrix=RotXYZ2RotationMatrix([1,2,3]),
                            initialVelocity=[1,0,0],
                            gravity=g, 
                            graphicsDataList=gList,
                            )

nMass = mbs.GetObject(oMass)['nodeNumber']
mbs.SetNodeParameter(nMass, 'Vshow', True) #node not shown with CreateRigidBody ...
mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))

gContact.AddSphereWithMarker(mNode, radius=r, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)

N = m*gFact
F = mu*N
print('contact force N =',N)
print('friction force F=',F)
print('acceleration    =',F/m)

evalNodes += [nMass] 

#add as last because of transparency
oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=gDataList)))

#only implicit:
# mbs.CreateGenericJoint(bodyNumbers=[oGround, oMass], position=pRef,
#                        constrainedAxes=[0,0,0, 1,1,1])

mRot0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=3))
mRot1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=4))
mRot2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=5))

ccR0 = mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundC, mRot0]))
ccR1 = mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundC, mRot1]))
ccR2 = mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundC, mRot2]))

if useGraphics:
    sNode0 = mbs.AddSensor(SensorNode(nodeNumber=nMass, storeInternal=True, 
                                      outputVariableType=exu.OutputVariableType.Displacement))
    vNode0 = mbs.AddSensor(SensorNode(nodeNumber=nMass, storeInternal=True, 
                                      outputVariableType=exu.OutputVariableType.Velocity))
    # omegaNode0 = mbs.AddSensor(SensorNode(nodeNumber=nMass, storeInternal=True, 
    #                                   outputVariableType=exu.OutputVariableType.AngularVelocity))
    
    #lagrange multiplier; only available with implicit integration
    sRot1 = mbs.AddSensor(SensorObject(objectNumber=ccR1, storeInternal=True, 
                                      outputVariableType=exu.OutputVariableType.Force))

#%%+++++++++++++++++++++++++++++++++
mbs.Assemble()

tEnd = 2 #tEnd = 0.8 for test suite
h= 0.0002  #h= 0.0002 for test suite
# h*=0.1
# tEnd*=3
simulationSettings = exu.SimulationSettings()
#simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
simulationSettings.solutionSettings.writeSolutionToFile = False
if useGraphics:
    simulationSettings.solutionSettings.solutionWritePeriod = 0.005
    simulationSettings.solutionSettings.writeSolutionToFile = True
    simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/coordinatesSolution.txt'
else:
    simulationSettings.solutionSettings.exportAccelerations = False
    simulationSettings.solutionSettings.exportVelocities = False
    
simulationSettings.solutionSettings.sensorsWritePeriod = h*10
simulationSettings.solutionSettings.outputPrecision = 8 #make files smaller
# simulationSettings.displayComputationTime = True
# simulationSettings.displayGlobalTimers = True
#simulationSettings.displayStatistics = True
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.parallel.numberOfThreads = 1

simulationSettings.timeIntegration.newton.numericalDifferentiation.forODE2 = False
simulationSettings.timeIntegration.newton.useModifiedNewton = True

SC.visualizationSettings.general.graphicsUpdateInterval=0.05
# SC.visualizationSettings.general.drawWorldBasis = True
SC.visualizationSettings.general.circleTiling=200
SC.visualizationSettings.general.drawCoordinateSystem=True
SC.visualizationSettings.loads.show=False
SC.visualizationSettings.bodies.show=True
SC.visualizationSettings.markers.show=False

SC.visualizationSettings.nodes.show=True
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.nodes.basisSize = 1
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.defaultSize = 0 #must not be -1, otherwise uses autocomputed size
SC.visualizationSettings.nodes.tiling = 4
SC.visualizationSettings.openGL.drawFaceNormals = False

SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.openGL.shadow = 0.25
SC.visualizationSettings.openGL.light0position = [-3,3,10,0]

if useGraphics:
    SC.visualizationSettings.general.autoFitScene = False
    exu.StartRenderer()
    if 'renderState' in exu.sys:
        SC.SetRenderState(exu.sys['renderState'])
    # mbs.WaitForUserToContinue()

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.explicitIntegration.computeEndOfStepAccelerations = False #increase performance, accelerations less accurate
#mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitEuler)
mbs.SolveDynamic(simulationSettings)

#compute error:
uSum=0
for node in evalNodes:
    u = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)
    exu.Print('coords node'+str(node)+' =',u)
    for c in u:
        uSum += abs(c) #add up all coordinates for comparison


exu.Print('solution of generalContactImplicit1=',uSum)
exudynTestGlobals.testError = uSum - (0) 

exudynTestGlobals.testResult = uSum

    
if useGraphics:
    # SC.WaitForRenderEngineStopFlag()

    if True:
        SC.visualizationSettings.general.autoFitScene = False
        SC.visualizationSettings.general.graphicsUpdateInterval=0.02
        
        mbs.SolutionViewer(timeout=0.01)

    exu.StopRenderer() #safely close rendering window!

if useGraphics:
    mbs.PlotSensor([], closeAll=True)
    mbs.PlotSensor([sNode0]*3, [0,1,2], figureName='sphere position')
    mbs.PlotSensor([vNode0]*3, [0,1,2], figureName='sphere velocity')
    mbs.PlotSensor([sRot1], [0], figureName='Torque y') #only available in implicit integrators!


