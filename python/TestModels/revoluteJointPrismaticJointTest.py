#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A chain of 3D rigid bodies connected with revolute and prismatic joints
#
# Author:   Johannes Gerstmayr
# Date:     2021-07-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

from math import sin, cos, pi
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


#background
color = [0.1,0.1,0.8,1]
L = 0.4 #length of bodies
d = 0.1 #diameter of bodies

#create background, in order to have according zoom all
zz=2*L
#background0 = GraphicsDataRectangle(-zz,-zz,zz,zz,graphics.color.white)
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0])) #,visualization=VObjectGround(graphicsData= [background0])))
mPosLast = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oGround, localPosition=[0,0,0]))
A0 = RotationMatrixX(0) 
Alast = A0 #previous marker

A0 = RotationMatrixX(0)
A1 = RotationMatrixY(0.5*pi)
A2 = RotationMatrixZ(0.5*pi)
A3 = RotationMatrixX(-0.5*pi)
A4 = RotationMatrixZ(0.5*pi)

Alist=[A0,A1,A2,A3,A4]

p0 = [0.,0.,0] #reference position
vLoc = np.array([L,0,0]) #last to next joint
#g = [0,0,-9.81]
g = [0,-9.81,0]

#create a chain of bodies:
for i in range(5):
    #print("Build Object", i)
    inertia = InertiaCuboid(density=1000, sideLengths=[L,d,d])
    #p0 += Alist[i] @ (0.5*vLoc)
    p0 += (0.5*vLoc)

    ep0 = eulerParameters0 #no rotation
    graphicsBody = graphics.Brick([0,0,0], [L,d,d], graphics.color.steelblue)
    oRB = mbs.CreateRigidBody(inertia=inertia, 
                              referencePosition=p0,
                              referenceRotationMatrix=A0,
                              gravity=g,
                              graphicsDataList=[graphicsBody])
    nRB= mbs.GetObject(oRB)['nodeNumber']

    mPos0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB, localPosition = [-0.5*L,0,0]))
    mPos1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB, localPosition = [ 0.5*L,0,0]))
    useGenericJoint = False #for comparison
    if True:
        if i != 2:
            if not useGenericJoint:
                mbs.AddObject(ObjectJointRevoluteZ(markerNumbers = [mPosLast, mPos0], 
                                                  rotationMarker0=Alist[i],
                                                  rotationMarker1=Alist[i],
                                                  visualization=VObjectJointRevoluteZ(axisRadius=0.5*d, axisLength=1.2*d)
                                                  )) 
            else: #compare to GenericJoint
                mbs.AddObject(ObjectJointGeneric(markerNumbers = [mPosLast, mPos0], 
                                                 constrainedAxes=[1,1,1,1,1,0],
                                                 rotationMarker0=Alist[i],
                                                 rotationMarker1=Alist[i],
                                                 visualization=VGenericJoint(axesRadius=0.5*d, axesLength=1.2*d)
                                                 )) 
        else:
            if not useGenericJoint:
                mbs.AddObject(ObjectJointPrismaticX(markerNumbers = [mPosLast, mPos0], 
                                                  rotationMarker0=Alist[i],
                                                  rotationMarker1=Alist[i],
                                                  visualization=VObjectJointPrismaticX(axisRadius=0.5*d, axisLength=1.2*d)
                                                  )) 
            else: #compare to GenericJoint
                mbs.AddObject(ObjectJointGeneric(markerNumbers = [mPosLast, mPos0], 
                                                 constrainedAxes=[0,1,1,1,1,1],
                                                 rotationMarker0=Alist[i],
                                                 rotationMarker1=Alist[i],
                                                 visualization=VGenericJoint(axesRadius=0.5*d, axesLength=1.2*d)
                                                 )) 
            #add spring, to limit motion in prismatic joint:
            k=500
            damp=k*0.02
            mbs.AddObject(CartesianSpringDamper(markerNumbers = [mPosLast, mPos0], 
                                       stiffness = [k]*3, damping = [damp]*3,
                                       visualization=VCartesianSpringDamper(show=False, drawSize=0.5*d)))
        
    mPosLast = mPos1
    
    # p0 += Alist[i] @ (0.5*vLoc)
    p0 += (0.5*vLoc)
    Alast = Alist[i]

mbs.AddLoad(LoadForceVector(markerNumber=mPosLast, loadVector=[0,0,20]))

mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 0.25
h=0.001  #use small step size to detext contact switching

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.solutionWritePeriod = 0.001
simulationSettings.solutionSettings.sensorsWritePeriod = 0.01
#simulationSettings.timeIntegration.simulateInRealtime = True
#simulationSettings.timeIntegration.realtimeFactor = 0.1
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
simulationSettings.timeIntegration.newton.useModifiedNewton = True
#simulationSettings.timeIntegration.newton.modifiedNewtonJacUpdatePerStep = True 

SC.visualizationSettings.nodes.show = True
SC.visualizationSettings.nodes.drawNodesAsPoint  = False
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.nodes.basisSize = 0.015
SC.visualizationSettings.connectors.showJointAxes = True
 
SC.visualizationSettings.general.autoFitScene = False #use loaded render state
#useGraphics = False
if useGraphics:
    simulationSettings.displayComputationTime = True
    simulationSettings.displayStatistics = True
    SC.renderer.Start()
    if 'renderState' in exu.sys:
        SC.renderer.SetState(exu.sys[ 'renderState' ])
    #SC.renderer.DoIdleTasks()
else:
    simulationSettings.solutionSettings.writeSolutionToFile = False

#mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)
mbs.SolveDynamic(simulationSettings, showHints=True)

if False: #use this to reload the solution and use SolutionViewer
    sol = LoadSolutionFile('coordinatesSolution.txt')
    
    mbs.SolutionViewer(sol)


u0 = mbs.GetNodeOutput(nRB, exu.OutputVariableType.Displacement)
rot0 = mbs.GetNodeOutput(nRB, exu.OutputVariableType.Rotation)
exu.Print('u0=',u0,', rot0=', rot0)

result = (abs(u0)+abs(rot0)).sum()
exu.Print('solution of revoluteJointprismaticJointTest=',result)

exudynTestGlobals.testError = result - (1.2538806799246283) #2020-07-01: 1.2538806799246283
exudynTestGlobals.testResult = result



#%%+++++++++++++++++++++++++++++
if useGraphics:
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!


