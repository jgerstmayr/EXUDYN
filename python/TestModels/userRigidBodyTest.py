#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ObjectUserBody as a rigid body (#2986): a body on a NodeRigidBodyRxyz whose mass matrix, quadratic
#           velocity terms, position, rotation matrix and Jacobians are Python user functions, written with
#           exudyn.cUtilities; it moves under gravity, a Cartesian spring-damper at a point off the center of
#           mass and a torque, side by side with an ObjectRigidBody under the same loads, and both must agree.
#           A second user body has only symbolic user functions: the force and the position written once with
#           components (#3007), so that the same function computes with floats and records with exudyn.symbolic, the
#           mass matrix, the rotation matrix and the Jacobians as symbolic matrices with @ and .T (#3020, #3018).
#           The three bodies are solved one after the other, each alone, with the computation time shown.
#           The user body reuses the results of its position and rotation user functions for the same time,
#           coordinates and local position; a new user function clears them (#3028).
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
import exudyn.cMath as cMath
from exudyn.advancedUtilities import CreateSymbolicUserFunction
import numpy as np
esym = exu.symbolic

SC = exu.SystemContainer()

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

#the force and the position with components, for floats (cMath) and for the symbolic recording (esym) alike
def MatVec(A, v):
    return [A[i][0]*v[0] + A[i][1]*v[1] + A[i][2]*v[2] for i in range(3)]

def MatTVec(A, v):
    return [A[0][i]*v[0] + A[1][i]*v[1] + A[2][i]*v[2] for i in range(3)]

def Cross(a, b):
    return [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]]

def GLocal(functions, rot):
    #local G matrix of Tait-Bryan angles xyz, omegaLocal = G @ rot_t
    return [[functions.cos(rot[1])*functions.cos(rot[2]), functions.sin(rot[2]), 0],
            [-functions.cos(rot[1])*functions.sin(rot[2]), functions.cos(rot[2]), 0],
            [functions.sin(rot[1]), 0, 1]]

def GLocal_t(functions, rot, rot_t):
    c1, s1, c2, s2 = functions.cos(rot[1]), functions.sin(rot[1]), functions.cos(rot[2]), functions.sin(rot[2])
    return [[-s1*c2*rot_t[1] - c1*s2*rot_t[2], c2*rot_t[2], 0],
            [s1*s2*rot_t[1] - c1*c2*rot_t[2], -s2*rot_t[2], 0],
            [c1*rot_t[1], 0, 0]]

def RotationMatrix(functions, rot):
    #Rx(rot[0]) Ry(rot[1]) Rz(rot[2])
    c0, s0, c1, s1, c2, s2 = (functions.cos(rot[0]), functions.sin(rot[0]), functions.cos(rot[1]), functions.sin(rot[1]),
                              functions.cos(rot[2]), functions.sin(rot[2]))
    return [[c1*c2, -c1*s2, s1],
            [s0*s1*c2 + c0*s2, -s0*s1*s2 + c0*c2, -s0*c1],
            [-c0*s1*c2 + s0*s2, c0*s1*s2 + s0*c2, c0*c1]]

def ForceComponents(functions, q, q_t):
    rot = [q[3], q[4], q[5]]
    rot_t = [q_t[3], q_t[4], q_t[5]]
    G = GLocal(functions, rot)
    omegaLocal = MatVec(G, rot_t)
    JomegaLocal = MatVec(J, omegaLocal)
    inner = MatVec(J, MatVec(GLocal_t(functions, rot, rot_t), rot_t))
    gyroscopic = Cross(omegaLocal, JomegaLocal)
    torques = MatTVec(G, [inner[i] + gyroscopic[i] for i in range(3)])
    return [0*q[0], 0*q[0], 0*q[0], -torques[0], -torques[1], -torques[2]]

def PositionComponents(functions, localPosition, q):
    A = RotationMatrix(functions, [q[3], q[4], q[5]])
    p = MatVec(A, localPosition)
    return [q[0] + p[0], q[1] + p[1], q[2] + p[2]]

def UFforceSymbolic(mbs, t, itemNumber, q, q_t):
    return ForceComponents(esym, q, q_t)

def UFpositionSymbolic(mbs, t, itemNumber, localPosition, q):
    return PositionComponents(esym, localPosition, q)

#the matrix results as symbolic matrices (#3020): built from the components, multiplied with @ and transposed with .T
def Skew(v):
    return [[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]

def Blocks3x6(left, right):
    #the 3 x 6 matrix [left, right] of two 3 x 3 symbolic matrices; None is a zero block
    return esym.Matrix([[left[i, j] if left is not None else 0 for j in range(3)] + [right[i, j] for j in range(3)] for i in range(3)])

def UFmassSymbolic(mbs, t, itemNumber, q, q_t):
    G = esym.Matrix(GLocal(esym, [q[3], q[4], q[5]]))
    Mrot = G.T @ esym.Matrix(J) @ G
    return esym.Matrix([[m if i == j else 0 for j in range(3)] + [0, 0, 0] for i in range(3)]
                       + [[0, 0, 0] + [Mrot[i, j] for j in range(3)] for i in range(3)])

def UFpositionJacobianSymbolic(mbs, t, itemNumber, localPosition, q):
    rot = [q[3], q[4], q[5]]
    A = esym.Matrix(RotationMatrix(esym, rot))
    rotationPart = -(A @ esym.Matrix(Skew(localPosition)) @ esym.Matrix(GLocal(esym, rot)))
    return Blocks3x6(esym.Matrix(np.eye(3)), rotationPart)

def UFrotationMatrixSymbolic(mbs, t, itemNumber, localPosition, q):
    return esym.Matrix(RotationMatrix(esym, [q[3], q[4], q[5]]))

def UFrotationJacobianSymbolic(mbs, t, itemNumber, localPosition, q):
    rot = [q[3], q[4], q[5]]
    return Blocks3x6(None, esym.Matrix(RotationMatrix(esym, rot)) @ esym.Matrix(GLocal(esym, rot)))

#the component functions compute what the matrix functions above compute
qTest = [0.1, 0.2, 0.3, 0.2, -0.1, 0.3]
qTest_t = [0.5, 0, 1, 1, 0.5, 2]
componentsDifference = max(np.max(np.abs(np.array(ForceComponents(cMath, qTest, qTest_t)) - UFforce(0, 0, 0, np.array(qTest), np.array(qTest_t)))),
                           np.max(np.abs(np.array(PositionComponents(cMath, pSpring, qTest)) - UFposition(0, 0, 0, pSpring, np.array(qTest)))))
exu.Print('the component functions against the matrix functions:', componentsDifference)

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 500
simulationSettings.timeIntegration.endTime = 0.5
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.show.computationTime = True #shows what each kind of body costs
simulationSettings.solution.file.write = False

#the output variables at the point of the spring-damper and the kinetic energy, of each kind of body
outputs = [exu.OutputVariableType.Position, exu.OutputVariableType.Displacement, exu.OutputVariableType.Velocity,
           exu.OutputVariableType.RotationMatrix, exu.OutputVariableType.AngularVelocity, exu.OutputVariableType.AngularVelocityLocal]

def Solve(kind):
    """the body under gravity, a Cartesian spring-damper and a torque, as ObjectRigidBody ('standard'), as
    ObjectUserBody with Python user functions ('python') or with symbolic user functions ('symbolic'), solved alone"""
    exu.Print('+++ the body with ' + {'standard': 'ObjectRigidBody', 'python': 'Python user functions',
                                       'symbolic': 'symbolic user functions'}[kind] + ' +++')
    mbs = SC.AddSystem()
    oGround = mbs.AddObject(ObjectGround())
    mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0.3, 0.2, 0]))
    node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0, 0, 0, 0.2, -0.1, 0.3],
                                         initialVelocities=[0.5, 0, 1, 1, 0.5, 2]))
    keep = [] #the symbolic functions live as long as the model
    if kind == 'standard':
        body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=m, inertia=inertia.GetInertia6D()))
    else:
        body = mbs.AddObject(ObjectUserBody(nodeNumbers=[node], massMatrixUserFunction=UFmass, forceUserFunction=UFforce,
                                            positionUserFunction=UFposition, positionJacobianUserFunction=UFpositionJacobian,
                                            rotationMatrixUserFunction=UFrotationMatrix,
                                            rotationJacobianUserFunction=UFrotationJacobian))
        if kind == 'symbolic': #every user function of the body symbolic, the matrices too (#3020)
            for name, function in [('forceUserFunction', UFforceSymbolic), ('positionUserFunction', UFpositionSymbolic),
                                   ('massMatrixUserFunction', UFmassSymbolic),
                                   ('positionJacobianUserFunction', UFpositionJacobianSymbolic),
                                   ('rotationMatrixUserFunction', UFrotationMatrixSymbolic),
                                   ('rotationJacobianUserFunction', UFrotationJacobianSymbolic)]:
                keep += [CreateSymbolicUserFunction(mbs, function, name, body)]
                mbs.SetObjectParameter(body, name, keep[-1])
    mbs.variables['keep'] = keep
    mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerBodyPosition(bodyNumber=body)), loadVector=m*np.array(g)))
    mbs.AddObject(CartesianSpringDamper(markerNumbers=[mGround, mbs.AddMarker(MarkerBodyPosition(bodyNumber=body, localPosition=pSpring))],
                                        stiffness=[200, 200, 200], damping=[2, 2, 2]))
    mbs.AddLoad(LoadTorqueVector(markerNumber=mbs.AddMarker(MarkerBodyRigid(bodyNumber=body)), loadVector=torque))
    mbs.Assemble()
    mbs.SolveDynamic(simulationSettings)
    values = [np.array(mbs.GetObjectOutputBody(body, output, localPosition=pSpring)) for output in outputs]
    energy = mbs.GetObjectOutputBody(body, exu.OutputVariableType.KineticEnergy, localPosition=[0, 0, 0])
    return values, energy

results = [Solve(kind) for kind in ['standard', 'python', 'symbolic']]

difference = 0
differenceSymbolic = 0  #between the Python and the symbolic user body
u = 0
for i in range(len(outputs)):
    values = [result[0][i] for result in results]
    difference = max(difference, np.max(np.abs(values[1] - values[0])), np.max(np.abs(values[2] - values[0])))
    differenceSymbolic = max(differenceSymbolic, np.max(np.abs(values[2] - values[1])))
    u += np.sum(values[1])
energies = [result[1] for result in results]
difference = max(difference, abs(energies[1] - energies[0]), abs(energies[2] - energies[0]))
u += energies[1]
exu.Print('largest difference to ObjectRigidBody:', difference, ', between Python and symbolic user functions:',
          differenceSymbolic)

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
exu.Print('a rigid marker without rotationMatrixUserFunction is refused, as it must be:', bool(refused))

#the body reuses the results of its user functions for the same time, coordinates and local position; a new user
#function clears them (#3028)
p1 = np.array(mbs2.GetObjectOutputBody(body, exu.OutputVariableType.Position, localPosition=[0.1, 0, 0],
                                       configuration=exu.ConfigurationType.Reference))
mbs2.SetObjectParameter(body, 'positionUserFunction', lambda mbs, t, itemNumber, localPosition, q:
                        UFposition(mbs, t, itemNumber, localPosition, q) + np.array([1., 0, 0]))
p2 = np.array(mbs2.GetObjectOutputBody(body, exu.OutputVariableType.Position, localPosition=[0.1, 0, 0],
                                       configuration=exu.ConfigurationType.Reference))
newFunctionUsed = int(np.max(np.abs(p2 - p1 - np.array([1., 0, 0]))) < 1e-15)
exu.Print('a new positionUserFunction is used, not the stored result:', bool(newFunctionUsed))

u += int(difference > 1e-8) + (1 - refused) + (1 - newFunctionUsed) + int(differenceSymbolic > 1e-12) + int(componentsDifference > 1e-14)
exu.Print('solution of userRigidBodyTest=', u)

exu.sys['testResult'] = u
