#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ObjectUserBody as a rigid body (#2986): a body on a NodeRigidBodyRxyz whose mass matrix, quadratic
#           velocity terms, position, rotation matrix and Jacobians are Python user functions, written with
#           exudyn.cUtilities; it moves under gravity, a Cartesian spring-damper at a point off the center of
#           mass and a torque, side by side with an ObjectRigidBody under the same loads, and both must agree.
#           A second user body has only symbolic user functions, written with exudyn.symbolicUtilities (#3069) as
#           symbolic matrices and vectors with @ and .T (#3020, #3018); the force and the position are written once
#           for functions=exudyn.symbolic and exudyn.cMath, so that the same function records and computes numbers.
#           The mass and the inertia tensor are the parameters of the user bodies, named by a ParameterLayout and read
#           by the Python and the symbolic user functions alike (#3056).
#           The three bodies are solved one after the other, each alone, with the computation time shown.
#           A new position user function, set with SetObjectParameter, is used at once.
#           A rigid marker on a user body without rotationMatrixUserFunction is refused when it is evaluated.
#           Run alone, it draws the user bodies with their graphicsData and opens the SolutionViewer.
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
import exudyn.symbolicUtilities as su
from exudyn.advancedUtilities import CreateSymbolicUserFunction, ParameterLayout
import exudyn.graphics as graphics
import numpy as np
esym = exu.symbolic
testIsActive = exu.sys.get('testIsActive', False) #run alone: the renderer, the solution file and the SolutionViewer

SC = exu.SystemContainer()

inertia = InertiaCuboid(density=1000, sideLengths=[0.4, 0.2, 0.1])
m = inertia.Mass()
g = [0, -9.81, 0]

#the parameters of the user rigid body: its mass and inertia tensor, [Jxx, Jyy, Jzz, Jyz, Jxz, Jxy] as in Exudyn
layout = ParameterLayout([('mass', 1), ('inertia', 6)])
bodyParameters = layout.ToVector({'mass': m, 'inertia': inertia.GetInertia6D()})
pSpring = [0.2, 0.1, 0.05] #local point of the spring-damper
torque = [0.2, 0, 0.5]

#the rigid body as user functions; qTotal = [position, Tait-Bryan angles], the total coordinates
def UFmass(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    d = layout.ToDict(parameters)
    J = Inertia6D2InertiaTensor(d['inertia'])
    G = cu.RotXYZ2GLocal(qTotal[3:6])
    M = np.zeros((6, 6))
    M[0:3, 0:3] = d['mass']*np.eye(3)
    M[3:6, 3:6] = G.T @ J @ G
    return M

def UFforce(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    J = Inertia6D2InertiaTensor(layout.ToDict(parameters)['inertia'])
    G = cu.RotXYZ2GLocal(qTotal[3:6])
    G_t = cu.RotXYZ2GLocal_t(qTotal[3:6], q_t[3:6])
    omegaLocal = G @ q_t[3:6]
    f = np.zeros(6)
    f[3:6] = -G.T @ (J @ (G_t @ q_t[3:6]) + np.cross(omegaLocal, J @ omegaLocal))
    return f

def UFposition(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return np.array(qTotal[0:3]) + cu.RotXYZ2RotationMatrix(qTotal[3:6]) @ localPosition

def UFpositionJacobian(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    A = cu.RotXYZ2RotationMatrix(qTotal[3:6])
    Jpos = np.zeros((3, 6))
    Jpos[:, 0:3] = np.eye(3)
    Jpos[:, 3:6] = -A @ cu.Skew(localPosition) @ cu.RotXYZ2GLocal(qTotal[3:6])
    return Jpos

def UFrotationMatrix(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return cu.RotXYZ2RotationMatrix(qTotal[3:6])

def UFrotationJacobian(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    Jrot = np.zeros((3, 6))
    Jrot[:, 3:6] = cu.RotXYZ2RotationMatrix(qTotal[3:6]) @ cu.RotXYZ2GLocal(qTotal[3:6])
    return Jrot

#the symbolic user functions with exudyn.symbolicUtilities (#3069): the same calls as above, with functions=esym;
#the force and the position also compute numbers with functions=cMath
def ForceTorques(qTotal, q_t, inertia6D, functions):
    rot, rot_t = qTotal[3:6], q_t[3:6]
    J = su.Inertia6D2InertiaTensor(inertia6D, functions)
    G = su.RotXYZ2GLocal(rot, functions)
    omegaLocal = G @ rot_t
    torques = -(G.T @ (J @ (su.RotXYZ2GLocal_t(rot, rot_t, functions) @ rot_t) + su.Cross(omegaLocal, J @ omegaLocal, functions)))
    return [0, 0, 0, torques[0], torques[1], torques[2]]

def Position(localPosition, qTotal, functions):
    return qTotal[0:3] + su.RotXYZ2RotationMatrix(qTotal[3:6], functions) @ localPosition

def UFforceSymbolic(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return ForceTorques(qTotal, q_t, layout.ToDict(parameters)['inertia'], esym)

def UFpositionSymbolic(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return Position(localPosition, qTotal, esym)

def Blocks3x6(left, right):
    #the 3 x 6 matrix [left, right] of two 3 x 3 symbolic matrices; None is a zero block
    return esym.Matrix([[left[i, j] if left is not None else 0 for j in range(3)] + [right[i, j] for j in range(3)] for i in range(3)])

def UFmassSymbolic(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    d = layout.ToDict(parameters)
    G = su.RotXYZ2GLocal(qTotal[3:6])
    Mrot = G.T @ su.Inertia6D2InertiaTensor(d['inertia']) @ G
    return esym.Matrix([[d['mass'] if i == j else 0 for j in range(3)] + [0, 0, 0] for i in range(3)]
                       + [[0, 0, 0] + [Mrot[i, j] for j in range(3)] for i in range(3)])

def UFpositionJacobianSymbolic(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    rot = qTotal[3:6]
    rotationPart = -(su.RotXYZ2RotationMatrix(rot) @ su.Skew(localPosition) @ su.RotXYZ2GLocal(rot))
    return Blocks3x6(esym.Matrix(np.eye(3)), rotationPart)

def UFrotationMatrixSymbolic(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return su.RotXYZ2RotationMatrix(qTotal[3:6])

def UFrotationJacobianSymbolic(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    rot = qTotal[3:6]
    return Blocks3x6(None, su.RotXYZ2RotationMatrix(rot) @ su.RotXYZ2GLocal(rot))

#with functions=cMath, the force and the position of the symbolic user functions compute what the Python ones compute
qTest = np.array([0.1, 0.2, 0.3, 0.2, -0.1, 0.3])
qTest_t = np.array([0.5, 0, 1, 1, 0.5, 2])
componentsDifference = max(np.max(np.abs(np.array(ForceTorques(qTest, qTest_t, inertia.GetInertia6D(), cMath)) - UFforce(0, 0, 0, qTest, qTest, qTest_t, bodyParameters))),
                           np.max(np.abs(Position(np.array(pSpring), qTest, cMath) - UFposition(0, 0, 0, pSpring, qTest, qTest, qTest_t, bodyParameters))))
exu.Print('the functions of the symbolic user functions with cMath against the Python user functions:', componentsDifference)

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 500
simulationSettings.timeIntegration.endTime = 0.5
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.show.computationTime = True #shows what each kind of body costs
simulationSettings.solution.file.write = False

#the output variables at the point of the spring-damper and the kinetic energy, of each kind of body
outputs = [exu.OutputVariableType.Position, exu.OutputVariableType.Displacement, exu.OutputVariableType.Velocity,
           exu.OutputVariableType.RotationMatrix, exu.OutputVariableType.AngularVelocity, exu.OutputVariableType.AngularVelocityLocal]

bodyGraphics = [graphics.Brick(size=[0.4, 0.2, 0.1], color=graphics.color.steelblue, addEdges=True), graphics.Basis(length=0.15)]

def Solve(kind, show=False):
    """the body under gravity, a Cartesian spring-damper and a torque, as ObjectRigidBody ('standard'), as
    ObjectUserBody with Python user functions ('python') or with symbolic user functions ('symbolic'), solved alone;
    show: drawn when the model runs alone"""
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
        body = mbs.AddObject(ObjectUserBody(nodeNumbers=[node], parameters=bodyParameters,
                                            massMatrixUserFunction=UFmass, forceUserFunction=UFforce,
                                            positionUserFunction=UFposition, positionJacobianUserFunction=UFpositionJacobian,
                                            rotationMatrixUserFunction=UFrotationMatrix,
                                            rotationJacobianUserFunction=UFrotationJacobian,
                                            visualization=VObjectUserBody(graphicsData=bodyGraphics)))
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
    if show and not testIsActive:
        simulationSettings.solution.file.write = True
        SC.renderer.Start()
    mbs.SolveDynamic(simulationSettings)
    if show and not testIsActive:
        SC.renderer.Stop()
        simulationSettings.solution.file.write = False
        mbs.SolutionViewer()
    values = [np.array(mbs.GetObjectOutputBody(body, output, localPosition=pSpring)) for output in outputs]
    energy = mbs.GetObjectOutputBody(body, exu.OutputVariableType.KineticEnergy, localPosition=[0, 0, 0])
    return values, energy

results = [Solve(kind, show=(kind == 'symbolic')) for kind in ['standard', 'python', 'symbolic']]

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
body = mbs2.AddObject(ObjectUserBody(nodeNumbers=[node], parameters=bodyParameters, massMatrixUserFunction=UFmass, positionUserFunction=UFposition,
                                     positionJacobianUserFunction=UFpositionJacobian))
mbs2.AddLoad(LoadTorqueVector(markerNumber=mbs2.AddMarker(MarkerBodyRigid(bodyNumber=body)), loadVector=torque))
mbs2.Assemble()
refused = 0
try:
    mbs2.SolveDynamic(simulationSettings)
except ValueError as e:
    refused = int('rotationMatrixUserFunction is not given' in str(e))
exu.Print('a rigid marker without rotationMatrixUserFunction is refused, as it must be:', bool(refused))

#a new user function is used at once
p1 = np.array(mbs2.GetObjectOutputBody(body, exu.OutputVariableType.Position, localPosition=[0.1, 0, 0],
                                       configuration=exu.ConfigurationType.Reference))
mbs2.SetObjectParameter(body, 'positionUserFunction', lambda mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters:
                        UFposition(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters) + np.array([1., 0, 0]))
p2 = np.array(mbs2.GetObjectOutputBody(body, exu.OutputVariableType.Position, localPosition=[0.1, 0, 0],
                                       configuration=exu.ConfigurationType.Reference))
newFunctionUsed = int(np.max(np.abs(p2 - p1 - np.array([1., 0, 0]))) < 1e-15)
exu.Print('a new positionUserFunction is used, not the stored result:', bool(newFunctionUsed))

u += int(difference > 1e-8) + (1 - refused) + (1 - newFunctionUsed) + int(differenceSymbolic > 1e-12) + int(componentsDifference > 1e-14)
exu.Print('solution of userRigidBodyTest=', u)

exu.sys['testResult'] = u
