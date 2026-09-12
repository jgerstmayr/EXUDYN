#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test model for MainSystem Create functions; the model itself is not very meaningful, but it uses many Create functions
#
# Author:   Johannes Gerstmayr
# Date:     2025-05-11
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


#set up new multibody system to work with
SC = exu.SystemContainer()
mbs = SC.AddSystem()

graphicsGround = graphics.CheckerBoard(point=[0,2,-1], size=8)
oGround = mbs.CreateGround(referencePosition=[0,0,0], graphicsDataList=[graphicsGround])

oMass = mbs.CreateMassPoint(physicsMass=5, referencePosition=[1,0,0], 
                            initialDisplacement=[0,0,0],
                            initialVelocity=[0,0.5,0],
                            gravity=[0,-9.81,0],
                            drawSize=0.3, color=graphics.color.orange,
                            )

oSpringDamper = mbs.CreateSpringDamper(bodyNumbers=[oGround, oMass], #[body0,body1]
                                      localPosition0=[0,0,0], #locally on body0
                                      localPosition1=[0,0,0], #locally on body1
                                      referenceLength=None, #usually set to None (default) => takes the (relaxed) length in reference configuration
                                      stiffness=1e2, damping=1)

oDistance = mbs.CreateDistanceConstraint(bodyNumbers=[oGround, oMass], #[body0,body1]
                                         localPosition0 = [0,0,0], #locally on body0
                                         localPosition1 = [0,0,0], #locally on body1
                                         distance=None)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++

mass = 10 
outerRadius = 0.45
length = 2
volume = np.pi * outerRadius**2 * length
inertiaCylinder = InertiaCylinder(density=mass/volume, length=length, outerRadius=outerRadius, 
                                  axis=0, innerRadius=0.7*outerRadius)

oBody = mbs.CreateRigidBody(inertia=inertiaCylinder,
                            referencePosition=[3,0,0],
                            initialVelocity=[0,0,0],
                            initialAngularVelocity=[2*np.pi,0,0],
                            gravity=[0,-9.81,0],
                            #graphicsDataList=[graphicsCyl]
                            )


rbX = 2
rbY = 0.5
rbZ = 0.2

#create inertia instance for cuboid (brick) shape
mass=10 #kg
volume=rbX*rbY*rbZ
inertiaCube2 = InertiaCuboid(density=mass/volume, sideLengths=[rbX,rbY,rbZ])
inertiaCube2 = inertiaCube2.Translated([0.5,0,0])

oBody2 = mbs.CreateRigidBody(inertia = inertiaCube2,
                             referencePosition = [4,-0.5*rbX,0], 
                             referenceRotationMatrix = RotationVector2RotationMatrix([0,0,-0.5*np.pi]),
                             initialAngularVelocity = [0,0,0.2*2*np.pi],
                             initialVelocity = [0.2*np.pi*0.5*rbX,0,0],
                             gravity = [0,-9.81,0],
                             color=graphics.color.blue,
                             #graphicsDataList=[graphicsCube, graphics.Basis(origin=inertiaCube2.COM())],
                             )

loadMassPoint = mbs.CreateForce(bodyNumber=oMass, loadVector=[10,0,0])
loadRigidBody = mbs.CreateForce(bodyNumber=oBody, localPosition=[0,0,0], loadVector=[10,0,0])

def UFforce(mbs, t, loadVector):
    return (10+5*np.sin(t*10*2*np.pi))*np.array([0,5,0])

mbs.CreateForce(bodyNumber=oBody,
                localPosition=[0,1.2,0.5], #position at body
                loadVectorUserFunction=UFforce)

def UserFunctionTorque(mbs, t, loadVector):
    return np.cos(t*2*np.pi)*np.array(loadVector)

#add torque with user function
mbs.CreateTorque(bodyNumber=oBody, loadVector=[5,0,0], 
                 loadVectorUserFunction=UserFunctionTorque)

#+++++++++
inertiaSphere = InertiaSphere(mass=2, radius=0.25)
gSphere = inertiaSphere.GetGraphics(graphics.color.red)

oMass1 = mbs.CreateMassPoint(physicsMass=2, referencePosition=[1,0,0],
                             gravity=[0,-9.81,0],
                             # drawSize=0.25, color=graphics.color.red,
                             graphicsDataList=[gSphere]
                             )

#create spherical joint between ground and mass point; could also be applied to two bodies; possible bodies: mass point or rigid body
mbs.CreateSphericalJoint(bodyNumbers=[oGround, oMass1],
                         position=[1,0,0], #global position of joint (in reference configuration)
                         constrainedAxes=[1,0,0]) #x,y,z directions: 1 = fixed, 0 = free motion

#constrain Y and Z coordinate of mass point to move z=10*y
mbs.CreateCoordinateConstraint(bodyNumbers=[oMass1, oMass1], 
                               coordinates=[2,1],
                               factorValue1=10)

mbs.CreatePrismaticJoint(bodyNumbers=[oGround, oBody], 
                         position=[3,0,0], #global position of joint
                         axis=[1,0,0], #global axis of joint, can move in global x-direction
                         useGlobalFrame=True) #use local coordinates for joint definition

mbs.CreateRevoluteJoint(bodyNumbers=[oBody, oBody2], 
                        position=[4,-0.5,0], #global position of joint
                        axis=[0,0,1], #rotation along global z-axis
                        useGlobalFrame=True)

mbs.CreateTorsionalSpringDamper(bodyNumbers=[oBody, oBody2],
                                position=[4,-0.5,0], #global position of spring-damper
                                axis=[0,0,1],        #global rotation axis
                                stiffness=1000,
                                damping=20,
                                useGlobalFrame=True)

mass = 5 
mu = 0.05 #static friction
rDisc = 0.5
length = 0.1
volume = np.pi * rDisc**2 * length
inertiaCylinder = InertiaCylinder(density=mass/volume, length=length, outerRadius=rDisc, axis=0)

oDisc = mbs.CreateRigidBody(inertia=inertiaCylinder,
                            nodeType=exu.NodeType.RotationRxyz, #this allows to constrain Z-rotation
                            referencePosition=[0,2,rDisc-1], #reference position x/y/z of COM
                            initialVelocity=[0,2*np.pi*rDisc,0],   
                            initialAngularVelocity=[-2*np.pi,0.2,0],
                            gravity=[0,0,-9.81])

mbs.CreateRollingDiscPenalty(bodyNumbers=[oGround, oDisc], discRadius = rDisc, 
                             axisPosition=[0,0,0], axisVector=[1,0,0],
                             planePosition = [0,0,-1], planeNormal = [0,0,1],
                             contactStiffness = 1e4, contactDamping = 2e2, 
                             dryFriction = [mu,mu], color=graphics.color.red)

#constrain Z and Y-rotation of disc, so it rotates only around X
mbs.CreateCoordinateConstraint(bodyNumbers=[oDisc, None], coordinates=[5,None])
mbs.CreateCoordinateConstraint(bodyNumbers=[oDisc, oGround], coordinates=[4,None])#also works with ground

mass = 5 
rDisc = 0.5
length = 0.1
volume = np.pi * rDisc**2 * length
inertiaCylinder = InertiaCylinder(density=mass/volume, length=length, outerRadius=rDisc, axis=0)

#create a free rigid body using the defined inertia properties and applies gravity (y-direction).
oDisc1 = mbs.CreateRigidBody(inertia=inertiaCylinder,
                            referencePosition=[1,2,rDisc-1], #reference position x/y/z of COM
                            initialVelocity=[0,2*np.pi*rDisc,0],   
                            initialAngularVelocity=[-2*np.pi,0.2,0],
                            gravity=[0,0,-9.81])

mbs.CreateForce(bodyNumber=oDisc1, loadVector=[10,0,0])

#create a 'rolling' joint between flat ground defined by plane, lying on oGround, and rigid body given as oDisc:
mbs.CreateRollingDisc(bodyNumbers=[oGround, oDisc1], discRadius = rDisc, 
                      axisPosition=[0,0,0], axisVector=[1,0,0], #relative to oDisc
                      planePosition = [0,0,-1], planeNormal = [0,0,1], #defines plane
                      constrainedAxes=[1,1,1] #constrain 3 axes: lateral motion, forward motion and normal contact
                      )

#constrain the two discs to have same X-motion
mbs.CreateCoordinateConstraint(bodyNumbers=[oDisc,oDisc1],
                               velocityLevel=True,
                               coordinates=[0,0])

mbs.CreateCartesianSpringDamper(
    bodyNumbers=[oGround, oBody2], #[body0,body1]
    localPosition0=[4,-0.5*rbX,0], #for body0
    localPosition1=[0,0,0],        #for body1
    stiffness = [100,10,10], #x,y,z stiffness
    damping = [5,2,2], #x,y,z damping
)

#%%++++++++++++++++++++++++++++++++++++++++++++++
mbs.Assemble()

endTime = 0.5 #simulation time in seconds
if useGraphics:
    endTime = 2.5

stepSize = 0.002 #should be small enough to achieve sufficient accuracy

#some simulation parameters:
simulationSettings = exu.SimulationSettings() #takes currently set values or default values
simulationSettings.timeIntegration.numberOfSteps = int(endTime/stepSize)
simulationSettings.timeIntegration.endTime = endTime
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.newton.useModifiedNewton = True

#for redundant constraints, use these two settings
simulationSettings.linearSolverSettings.ignoreSingularJacobian=True
simulationSettings.linearSolverType = exu.LinearSolverType.EigenDense #use EigenSparse for larger systems alternatively
#simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
# simulationSettings.timeIntegration.newton.useModifiedNewton = True
SC.visualizationSettings.openGL.multiSampling = 4

mbs.SolveDynamic(simulationSettings)


#%%
#TAG: visualization
#DIFFICULTY: 10000
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.showBasis = True

if useGraphics:
    mbs.SolutionViewer()

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
ode2 = mbs.systemData.GetODE2Coordinates()

u = 0.01*np.linalg.norm(ode2)
exu.Print('solution of createFunctionsTest=',u) 

exudynTestGlobals.testResult = u
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


