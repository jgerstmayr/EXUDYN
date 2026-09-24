#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test for AddRevoluteJoint utility function
#
# Author:   Johannes Gerstmayr 
# Date:     2021-07-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import testRunnerTools
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics

testIsActive = exu.sys.get('testIsActive', True)


from math import sin, cos, pi
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()


#background
color = [0.1,0.1,0.8,1]
L = 0.4 #length of bodies
d = 0.1 #diameter of bodies

oGround=mbs.AddObject(ObjectGround(referencePosition= [-0.5*L,0,0])) 
mPosLast = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oGround, localPosition=[0,0,0]))
A0 = np.eye(3)
Alast = A0 #previous marker
bodyLast = oGround

Alist=[]
axisList=[]
A=RotationMatrixX(0)

nBodies = 400
for i in range(nBodies):
    delta = 0.01*pi
    #A = RotationMatrixZ(delta)
    v0= A@[0,0,1]
    Alist+=[A]
    axisList+=[v0]
    A = A @ RotationMatrixX(delta)@RotationMatrixZ(2*delta)


p0 = [0.,0.,0] #reference position
vLoc = np.array([L,0,0]) #last to next joint
g = [0,0,9.81]
#g = [0,9.81,0]

#create a chain of bodies:
for i in range(nBodies):
    #print("Build Object", i)
    inertia = InertiaCuboid(density=1000, sideLengths=[L,d,d])
    p0 += Alist[i] @ (0.5*vLoc)
    #p0 += (0.5*vLoc)

    ep0 = eulerParameters0 #no rotation
    graphicsBody = graphics.Brick([0,0,0], [0.96*L,d,d], graphics.color.steelblue)
    # ep = RotationMatrix2EulerParameters(Alist[i])
    # ep = (1./np.linalg.norm(ep)) * ep
    oRB = mbs.CreateRigidBody(inertia=inertia, 
                              referencePosition=p0,
                              referenceRotationMatrix=Alist[i],
                              gravity=g,
                              graphicsDataList=[graphicsBody])

    body0 = bodyLast
    body1 = oRB
    axis = axisList[i]
    mbs.CreateRevoluteJoint(bodyNumbers=[body0, body1], position=[0.5*L,0,0], 
                            axis=Alast.T@axis, useGlobalFrame=False, 
                            axisRadius=0.6*d, axisLength=1.2*d)

    bodyLast = oRB
    
    p0 += Alist[i] @ (0.5*vLoc)
    #p0 += (0.5*vLoc)
    Alast = Alist[i]

#mbs.AddLoad(LoadForceVector(markerNumber=mPosLast, loadVector=[0,0,20]))

mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 0.7   #2026-09-16: shortened to stay inside the 1.5 - 5 s target (issue #2460)
h=0.001  #use small step size to detext contact switching

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.solutionWritePeriod = 0.005
simulationSettings.solutionSettings.sensorsWritePeriod = 0.01
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
simulationSettings.timeIntegration.newton.useModifiedNewton = True
#simulationSettings.timeIntegration.newton.modifiedNewtonJacUpdatePerStep = True
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
simulationSettings.displayComputationTime = True
simulationSettings.displayStatistics = True
simulationSettings.parallel.numberOfThreads = 4

SC.visualizationSettings.nodes.show = True
SC.visualizationSettings.nodes.drawNodesAsPoint  = False
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.nodes.basisSize = 0.015
SC.visualizationSettings.connectors.showJointAxes = True

#for snapshot:
SC.visualizationSettings.openGL.multiSampling=4
SC.visualizationSettings.openGL.lineWidth=2
SC.visualizationSettings.view0.window.renderWindowSize = [800,600]
SC.visualizationSettings.view0.scene.drawCoordinateSystem=False
SC.visualizationSettings.view0.scene.drawWorldBasis=True

SC.visualizationSettings.general.autoFitScene = False #use loaded render state
if not testIsActive:
    SC.renderer.Start()
    SC.renderer.RestoreSavedState()
    #SC.renderer.DoIdleTasks()
else:
    simulationSettings.solutionSettings.writeSolutionToFile = False

mbs.SolveDynamic(simulationSettings)

nRB= mbs.GetObject(oRB)['nodeNumber']
u0 = mbs.GetNodeOutput(nRB, exu.OutputVariableType.Displacement)
rot0 = mbs.GetNodeOutput(nRB, exu.OutputVariableType.Rotation)
exu.Print('u0=',u0,', rot0=', rot0)

result = (abs(u0)+abs(rot0)).sum()
exu.Print('solution of perf3DRigidBodies=',result)

exu.sys['testResult'] = result
#one run per file; the summary of runPerformanceTests.py reports the SOLVER time of it,
#without model build and Python overhead (issue #2460)
testRunnerTools.AddTiming('perf3DRigidBodies.py', mbs, result)




#%%+++++++++++++++++++++++++++++
if not testIsActive:
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!


