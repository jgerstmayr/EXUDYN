#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test for GenericJoint with userFunction
#
# Author:   Johannes Gerstmayr
# Date:     2020-04-22
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
from exudyn.FEM import *

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

nBodies = 5
color = [0.1,0.1,0.8,1]
s = 0.1 #width of cube
sx = 3*s #length of cube/body
cPosZ = 0. #offset of constraint in z-direction
zz = sx * (nBodies+1)*2 #max size of background

background0 = GraphicsDataRectangle(-zz,-zz,zz,2.5*sx,color)
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], 
                                   visualization=VObjectGround(graphicsData= [background0])))
mPosLast = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oGround, 
                                            localPosition=[-sx,0,0]))
oRB = -1 #for output value

def UFgenericJoint(mbs, t, itemIndex, param):
    phiZ = -2*pi*0.5*(1.-np.cos(t*2*pi / 4)) / nBodies
    phiX = -0.5*pi*0.5*(1.-np.cos(t*2*pi / 4)) / nBodies
    #exu.Print("phi=", phi)
    return [0,0,0, phiX,0,phiZ]

#create a chain of bodies:
for i in range(nBodies):
    f = 0 #factor for initial velocities
    omega0 = [0,50.*f,20*f] #arbitrary initial angular velocity
    ep0 = eulerParameters0 #no rotation
    ep_t0 = AngularVelocity2EulerParameters_t(omega0, ep0)

    p0 = [-sx*0+i*2*sx,0.,0] #reference position
    v0 = [0.2*f,0.,0.] #initial translational velocity

    nRB = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=p0+ep0, 
                                      initialVelocities=v0+list(ep_t0)))
    #nRB = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0,0,0,1,0,0,0], initialVelocities=[0,0,0,0,0,0,0]))
    oGraphics = GraphicsDataOrthoCubeLines(-sx,-s,-s, sx,s,s, [0.8,0.1,0.1,1])
    oRB = mbs.AddObject(ObjectRigidBody(physicsMass=2, 
                                        physicsInertia=[6,1,6,0,0,0], 
                                        nodeNumber=nRB, 
                                        physicsCenterOfMass=[0,0,0],
                                        visualization=VObjectRigidBody(graphicsData=[oGraphics])))

    mMassRB = mbs.AddMarker(MarkerBodyMass(bodyNumber = oRB))
    mbs.AddLoad(Gravity(markerNumber = mMassRB, loadVector=[0.,-9.81,0.])) #gravity in negative z-direction

    if i==0:
        mPos = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB, localPosition = [-sx,0.,cPosZ*0]))
    else:
        mPos = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB, localPosition = [-sx,0.,cPosZ]))

    val=1
    if i==0: val=0
    if i < 2:
        mbs.AddObject(GenericJoint(markerNumbers = [mPos, mPosLast], constrainedAxes=[val,1,1, 1,1,1],
                                    offsetUserFunctionParameters=[0,0,0, 0,0,0],
                                    offsetUserFunction=UFgenericJoint))
    else:
        if i==3:
            mbs.AddObject(GenericJoint(markerNumbers = [mPosLast, mPos], constrainedAxes=[1,1,1, 1,0,1]))
        elif i==4:
            mbs.AddObject(GenericJoint(markerNumbers = [mPosLast, mPos], constrainedAxes=[1,1,1, 1,1,0]))
        else:
            mbs.AddObject(SphericalJoint(markerNumbers = [mPosLast, mPos], constrainedAxes=[1,1,1]))

    #marker for next chain body
    mPosLast = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB, localPosition = [sx,0.,cPosZ]))


mbs.Assemble()
#exu.Print(mbs)

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

fact = 500
simulationSettings.timeIntegration.numberOfSteps = 1*fact
simulationSettings.timeIntegration.endTime = 0.005*fact
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/fact*10
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 

simulationSettings.solutionSettings.solutionInformation = "rigid body tests"
SC.visualizationSettings.nodes.defaultSize = 0.05
simulationSettings.displayComputationTime = False
#simulationSettings.displayStatistics = True



if useGraphics:
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings)#, experimentalNewSolver=True)


#+++++++++++++++++++++++++++++++++++++++++++++
#compute TestModel error for EulerParameters and index2 solver
pos = mbs.GetObjectOutputBody(oRB,exu.OutputVariableType.Position, localPosition=[0,0,0])
exu.Print('pos=',pos)
u = 0
for i in range(3): #take sum of all coordinates
    u += abs(pos[i])

exu.Print('solution of GenericJointTest=',u)

exudynTestGlobals.testError = u - (1.1878327690760586) #2020-04-22: 1.1878327690760586
exudynTestGlobals.testResult = u


if useGraphics:
    #SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

