#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test rigid body formulation for different center of mass (COM)
#
# Author:   Johannes Gerstmayr
# Date:     2023-11-30
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

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#intrinsic is independent of order of markers; force/torque computed at mid-frame of joint
intrinsicFormulation=True

k=10000
d= k*0.05 * 0
kr = 500*0.5
dr = kr*0.05 * 0

L = 1
#++++++++++++++++++++++++++++++++++++++++++++++
#create two ground and two bodies, rigid body spring damper using different marker order
gGround = graphics.Brick(size = [L,0.1,0.25], color=graphics.color.blue)
oGround = mbs.CreateGround(referencePosition = [0,0,0], graphicsDataList=[gGround])

gBody1 = graphics.Brick(size = [L,0.1,0.2], color=graphics.color.red)
oBody1 = mbs.CreateRigidBody(referencePosition = [L,0.,0.],
                       referenceRotationMatrix = np.eye(3),
                       initialVelocity = [0.,1.,2.],
                       initialAngularVelocity = 5.*np.array([1.7,2.1,3.2]),
                       inertia=InertiaCuboid(density=1000,sideLengths=[L,0.1,0.2]),
                       gravity = [0.,0.,0.],
                       graphicsDataList = [gBody1],)
 
gBody2 = graphics.Brick(size = [L,0.1,0.2], color=graphics.color.green)
oBody2 = mbs.CreateRigidBody(referencePosition = [L,0.,0.],
                       referenceRotationMatrix = np.eye(3),
                       initialVelocity = [0.,1.,2.],
                       initialAngularVelocity = 5.*np.array([1.7,2.1,3.2]),
                       inertia=InertiaCuboid(density=1000,sideLengths=[L,0.1,0.2]),
                       gravity = [0.,0.,0.],
                       graphicsDataList = [gBody2],)
 
oRBSD1 = mbs.CreateRigidBodySpringDamper(bodyList = [oGround, oBody1],
                                        localPosition0 = [0.5*L,0.,0.], #global position
                                        localPosition1 = [-0.5*L,0.,0.], #global position
                                        stiffness = np.diag([k*0.4,k*0.5,k*0.7, kr,kr*2,kr*1.3]), 
                                        damping = np.diag([d,d,d, dr,dr,dr]), 
                                        offset = [0.,0.,0.,0.,0.,0.],
                                         intrinsicFormulation=intrinsicFormulation,
                                        )

oRBSD2 = mbs.CreateRigidBodySpringDamper(
                                         bodyList = [oBody2, oGround],
                                         localPosition0 = [-0.5*L,0.,0.], #global position
                                         localPosition1 = [0.5*L,0.,0.], #global position
                                         stiffness = np.diag([k*0.4,k*0.5,k*0.7, kr,kr*2,kr*1.3]), 
                                         damping = np.diag([d,d,d, dr,dr,dr]), 
                                         offset = [0.,0.,0.,0.,0.,0.],
                                         intrinsicFormulation=intrinsicFormulation,
                                        )

#++++++++++++++++++++++++++++++++++++++++++++++
#create two connected bodies, freely rotating:
oBody3a = mbs.CreateRigidBody(referencePosition = [0,1.,0.],
                       referenceRotationMatrix = np.eye(3),
                       initialVelocity = 0.*np.array([0.,1.,2.]),
                       initialAngularVelocity = 5*np.array([1.7,2.1,3.2]),
                       inertia=InertiaCuboid(density=1000,sideLengths=[L,0.1,0.2]),
                       gravity = [0.,0.,0.],
                       graphicsDataList = [gBody1],)
 
oBody3b = mbs.CreateRigidBody(referencePosition = [L,1.,0.],
                       referenceRotationMatrix = np.eye(3),
                       initialVelocity = 0.*np.array([0.,-1.,-2.]),
                       initialAngularVelocity = -5*np.array([1.7,2.1,3.2]),
                       inertia=InertiaCuboid(density=1000,sideLengths=[L,0.1,0.2]),
                       gravity = [0.,0.,0.],
                       graphicsDataList = [gBody2],)

oRBSD3 = mbs.CreateRigidBodySpringDamper(
                                         bodyList = [oBody3a, oBody3b],
                                         localPosition0 = [0.5*L,0.,0.], #global position
                                         localPosition1 = [-0.5*L,0.,0.], #global position
                                         stiffness = 1*np.diag([k*0.4,k*0.5,k*0.7, kr,kr*2,kr*1.3]), 
                                         damping = np.diag([d,d,d, dr,dr,dr]), 
                                         offset = [0.,0.,0.,0.,0.,0.],
                                         intrinsicFormulation=intrinsicFormulation,
                                        )

# mbs.SetObjectParameter(oRBSD1, 'intrinsicFormulation', True)
# mbs.SetObjectParameter(oRBSD2, 'intrinsicFormulation', True)
# mbs.SetObjectParameter(oRBSD3, 'intrinsicFormulation', True)

sBody1 = mbs.AddSensor(SensorBody(bodyNumber=oBody1, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
sBody2 = mbs.AddSensor(SensorBody(bodyNumber=oBody2, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
sBody3 = mbs.AddSensor(SensorBody(bodyNumber=oBody3a, storeInternal=True, outputVariableType=exu.OutputVariableType.AngularVelocity))


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++
mbs.Assemble()

endTime = 1 #for stepSize=0.002: non-intrinsic formulation gets unstable around 7.5 seconds for body3 and for body1/2 around 73 seconds
if useGraphics:
    endTime = 100

stepSize = 0.002
simulationSettings = exu.SimulationSettings()
#simulationSettings.displayComputationTime = True
simulationSettings.timeIntegration.verboseMode = useGraphics
simulationSettings.solutionSettings.writeSolutionToFile = useGraphics

simulationSettings.timeIntegration.numberOfSteps = int(endTime/stepSize)
simulationSettings.timeIntegration.endTime = endTime
simulationSettings.timeIntegration.newton.useModifiedNewton = True


if useGraphics:
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings)

if useGraphics:
    SC.renderer.Stop() #safely close rendering window!


    mbs.PlotSensor([sBody1,sBody2], components=[1,1])
    mbs.PlotSensor([sBody1,sBody2], components=[2,2])
    mbs.PlotSensor([sBody3], components=[0,1,2])

    mbs.SolutionViewer()




p1=mbs.GetObjectOutputBody(oBody1, exu.OutputVariableType.Displacement)
exu.Print("p1=", p1)
p2=mbs.GetObjectOutputBody(oBody2, exu.OutputVariableType.Displacement)
exu.Print("p2=", p2)
omega3=mbs.GetObjectOutputBody(oBody3a, exu.OutputVariableType.AngularVelocity)
exu.Print("omega3=", omega3)


#+++++++++++++++++++++++++++++++++++++++++++++
u=NormL2(p1) + NormL2(p2) + NormL2(0.01*omega3)
exu.Print('solution of rigidBodySpringDamperIntrinsic test=',u)

exudynTestGlobals.testError = u - (0.5472368463500464)
exudynTestGlobals.testResult = u


if useGraphics:
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!





