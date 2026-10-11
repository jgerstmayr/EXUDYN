#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ObjectUserBody as a rigid body (#2986): a body on a NodeRigidBodyRxyz whose mass matrix, quadratic
#           velocity terms, position, rotation matrix and Jacobians are Python user functions, written with
#           exudyn.cUtilities; it moves under gravity, a Cartesian spring-damper at a point off the center of
#           mass and a torque, side by side with an ObjectRigidBody under the same loads, and both must agree.
#           A rigid marker on a user body without rotationMatrixUserFunction is refused when it is evaluated.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-12
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.cUtilities as cu
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

inertia = InertiaCuboid(density=1000, sideLengths=[0.4, 0.2, 0.1])
m = inertia.Mass()
J = inertia.inertiaTensor
g = [0, -9.81, 0]
pSpring = [0.2, 0.1, 0.05] #local point of the spring-damper
torque = [0.2, 0, 0.5]

#the rigid body as user functions; q = [position, Tait-Bryan angles], including the reference values
def UFmass(mbs, t, itemNumber, q, q_t):
    G = cu.RotXYZ2GLocal(q[3:6])
    M = np.zeros((6, 6))
    M[0:3, 0:3] = m*np.eye(3)
    M[3:6, 3:6] = G.T @ J @ G
    return M

def UFforce(mbs, t, itemNumber, q, q_t):
    G = cu.RotXYZ2GLocal(q[3:6])
    G_t = cu.RotXYZ2GLocal_t(q[3:6], q_t[3:6])
    omegaLocal = G @ q_t[3:6]
    f = np.zeros(6)
    f[3:6] = -G.T @ (J @ (G_t @ q_t[3:6]) + np.cross(omegaLocal, J @ omegaLocal))
    return f

def UFposition(mbs, t, itemNumber, localPosition, q):
    return np.array(q[0:3]) + cu.RotXYZ2RotationMatrix(q[3:6]) @ localPosition

def UFpositionJacobian(mbs, t, itemNumber, localPosition, q):
    A = cu.RotXYZ2RotationMatrix(q[3:6])
    Jpos = np.zeros((3, 6))
    Jpos[:, 0:3] = np.eye(3)
    Jpos[:, 3:6] = -A @ cu.Skew(localPosition) @ cu.RotXYZ2GLocal(q[3:6])
    return Jpos

def UFrotationMatrix(mbs, t, itemNumber, localPosition, q):
    return cu.RotXYZ2RotationMatrix(q[3:6])

def UFrotationJacobian(mbs, t, itemNumber, localPosition, q):
    Jrot = np.zeros((3, 6))
    Jrot[:, 3:6] = cu.RotXYZ2RotationMatrix(q[3:6]) @ cu.RotXYZ2GLocal(q[3:6])
    return Jrot

oGround = mbs.AddObject(ObjectGround())
mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0.3, 0.2, 0]))

bodies = []
for isUserBody in [False, True]:
    node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0, 0, 0, 0.2, -0.1, 0.3],
                                         initialVelocities=[0.5, 0, 1, 1, 0.5, 2]))
    if isUserBody:
        body = mbs.AddObject(ObjectUserBody(nodeNumbers=[node], massMatrixUserFunction=UFmass, forceUserFunction=UFforce,
                                            positionUserFunction=UFposition, positionJacobianUserFunction=UFpositionJacobian,
                                            rotationMatrixUserFunction=UFrotationMatrix,
                                            rotationJacobianUserFunction=UFrotationJacobian))
    else:
        body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=m, inertia=inertia.GetInertia6D()))
    bodies += [body]
    mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerBodyPosition(bodyNumber=body)), loadVector=m*np.array(g)))
    mbs.AddObject(CartesianSpringDamper(markerNumbers=[mGround, mbs.AddMarker(MarkerBodyPosition(bodyNumber=body, localPosition=pSpring))],
                                        stiffness=[200, 200, 200], damping=[2, 2, 2]))
    mbs.AddLoad(LoadTorqueVector(markerNumber=mbs.AddMarker(MarkerBodyRigid(bodyNumber=body)), loadVector=torque))

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 500
simulationSettings.timeIntegration.endTime = 0.5
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.solution.file.write = False
mbs.SolveDynamic(simulationSettings)

#the output variables of both bodies at the point of the spring-damper
outputs = [exu.OutputVariableType.Position, exu.OutputVariableType.Displacement, exu.OutputVariableType.Velocity,
           exu.OutputVariableType.RotationMatrix, exu.OutputVariableType.AngularVelocity, exu.OutputVariableType.AngularVelocityLocal]
difference = 0
u = 0
for output in outputs:
    values = [np.array(mbs.GetObjectOutputBody(body, output, localPosition=pSpring)) for body in bodies]
    difference = max(difference, np.max(np.abs(values[1] - values[0])))
    u += np.sum(values[1])
energies = [mbs.GetObjectOutputBody(body, exu.OutputVariableType.KineticEnergy, localPosition=[0, 0, 0]) for body in bodies]
difference = max(difference, abs(energies[1] - energies[0]))
u += energies[1]
exu.Print('largest difference to ObjectRigidBody:', difference)

#a rigid marker on a user body without rotationMatrixUserFunction is refused when it is evaluated
mbs2 = SC.AddSystem()
node = mbs2.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0, 0, 0, 0, 0, 0]))
body = mbs2.AddObject(ObjectUserBody(nodeNumbers=[node], massMatrixUserFunction=UFmass, positionUserFunction=UFposition,
                                     positionJacobianUserFunction=UFpositionJacobian))
mbs2.AddLoad(LoadTorqueVector(markerNumber=mbs2.AddMarker(MarkerBodyRigid(bodyNumber=body)), loadVector=torque))
mbs2.Assemble()
refused = 0
try:
    mbs2.SolveDynamic(simulationSettings)
except ValueError as e:
    refused = int('rotationMatrixUserFunction is not given' in str(e))
exu.Print('missing rotationMatrixUserFunction refused:', refused)

u += int(difference > 1e-8) + (1 - refused)
exu.Print('solution of userRigidBodyTest=', u)

exu.sys['testResult'] = u
