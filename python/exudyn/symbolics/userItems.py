#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  User items with the interfaces of the built-in items (#3111, #3125, design:
#           docs/dev/design/symbolicUserItems.md): UserItemCreator(mbs) has the Create functions of mbs with the same
#           arguments, and creates ObjectUserBody and ObjectConnectorUser instead of the built-in objects, with the same
#           nodes, markers and loads. The user functions are Python functions, so that the system runs in Exudyn as
#           built; Record() replaces them by one symbolic recording per kind of item, which SymbolicSystem and the
#           export take. A script that builds a system with creator = mbs builds the same system of user items with
#           creator = UserItemCreator(mbs).
#           The items: CreateGround, CreateMassPoint, CreateRigidBody (on NodeRigidBodyRxyz), CreateSpringDamper,
#           CreateTorsionalSpringDamper, CreateSphereSphereContact (#3187, with a data node #3197), CreateCartesianSpringDamper,
#           CreateRigidBodySpringDamper (#3193), CreateRevoluteJoint, CreatePrismaticJoint, CreateSphericalJoint,
#           CreateDistanceConstraint (ObjectJointUser, #3140), CreateCoordinateConstraint (ObjectGenericDAE, #3193),
#           CreateForce, CreateTorque. An argument a user item
#           does not support raises, naming it. A parameter of a mass point or a spring-damper may be a named
#           exudyn.symbolic.Real (mass=m1): the item gets its value, the creator keeps the variable in
#           parameterVariables, for SymbolicSystem(mbs, parameters=creator.parameterVariables) (#3152).
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-27 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import exudyn
import exudyn.itemInterface as eii
import exudyn.cMath as cMath
import exudyn.graphics as graphics
from exudyn.advancedUtilities import CreateSymbolicUserFunction
import exudyn.symbolics.kinematics as su
from exudyn.symbolics.utilities import IfThenElse
from exudyn.rigidBodyUtilities import RotationMatrix2RotXYZ, AngularVelocity2RotXYZ_t, ComputeOrthonormalBasis
from exudyn.misc.mainSystemExtensions import GetMarkersPosRot, JointPreCheckCalcBodyMarkers, _FrameFromHT

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = ['UserItemCreator']


#the mass point, ObjectUserBody on a NodePoint, parameters [mass]: as ObjectMassPoint, M = m I, the position of a local
#point is the node position plus the local position; it does not rotate
def _UFmassPointMass(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return parameters[0]*np.eye(3)

def _UFmassPointMassIntegral(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return parameters[0]*np.eye(3)

def _UFmassPointPosition(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return qTotal + localPosition

def _UFmassPointPositionJacobian(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return np.eye(3)

def _UFmassPointRotationMatrix(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return np.eye(3)

def _UFmassPointRotationJacobian(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return np.zeros((3, 3))

#the functions of exudyn.symbolics.kinematics for an argument: exudyn.symbolic while a function is recorded, else cMath,
#so that the rigid body computes numbers in Exudyn and is recorded by Record
def _Functions(argument):
    return exudyn.symbolic if isinstance(argument, (exudyn.symbolic.Vector, exudyn.symbolic.Matrix)) else cMath

def _Matrix(rows, functions):
    return exudyn.symbolic.Matrix(rows) if functions is exudyn.symbolic else np.array(rows, dtype=float)

def _Vector(components, functions):
    return exudyn.symbolic.Vector(components) if functions is exudyn.symbolic else np.array(components, dtype=float)

def _Blocks3x6(left, right, functions):
    #the 3 x 6 matrix [left, right] of two 3 x 3 matrices; None is a zero block
    return _Matrix([[left[i, j] if left is not None else 0 for j in range(3)]
                    + [right[i, j] if right is not None else 0 for j in range(3)] for i in range(3)], functions)

#the rigid body, ObjectUserBody on a NodeRigidBodyRxyz, qTotal = [position, Tait-Bryan angles], parameters [mass,
#inertia (6 components, about the node, as ObjectRigidBody), centerOfMass]: the mass matrix and the forces of
#src/ImplObjects/CObjectRigidBody.cpp with the local G matrix, the position of a local point p + A pLocal
def _UFrigidBodyMass(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    F = _Functions(qTotal)
    (m, A, G) = (parameters[0], su.RotXYZ2RotationMatrix(qTotal[3:6], F), su.RotXYZ2GLocal(qTotal[3:6], F))
    Mrot = G.T @ su.Inertia6D2InertiaTensor(parameters[1:7], F) @ G
    Mtr = (-m)*(A @ su.Skew(parameters[7:10], F) @ G) #the coupling of a center of mass off the node
    return _Matrix([[m if i == j else 0 for j in range(3)] + [Mtr[i, j] for j in range(3)] for i in range(3)]
                   + [[Mtr[j, i] for j in range(3)] + [Mrot[i, j] for j in range(3)] for i in range(3)], F)

def _UFrigidBodyForce(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    #the right-hand side of the quadratic velocity terms: -G^T (omega x J omega + J G_t rot_t) on the rotations, and
    #-A (omega x (U x omega) + U x G_t rot_t) on the translations, U = -m centerOfMass, omega local
    F = _Functions(qTotal)
    (rot, rot_t) = (qTotal[3:6], q_t[3:6])
    J = su.Inertia6D2InertiaTensor(parameters[1:7], F)
    G = su.RotXYZ2GLocal(rot, F)
    omega = G @ rot_t
    G_tRot_t = su.RotXYZ2GLocal_t(rot, rot_t, F) @ rot_t
    torques = G.T @ (su.Cross(omega, J @ omega, F) + J @ G_tRot_t)
    U = (-parameters[0])*parameters[7:10]
    forces = su.RotXYZ2RotationMatrix(rot, F) @ (su.Cross(omega, su.Cross(U, omega, F), F) + su.Cross(U, G_tRot_t, F))
    return _Vector([-forces[i] for i in range(3)] + [-torques[i] for i in range(3)], F)

def _UFrigidBodyMassIntegral(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    #m J_pos at the center of mass
    F = _Functions(qTotal)
    m = parameters[0]
    rot = qTotal[3:6]
    return _Blocks3x6(_Matrix([[m if i == j else 0 for j in range(3)] for i in range(3)], F),
                      (-m)*(su.RotXYZ2RotationMatrix(rot, F) @ su.Skew(parameters[7:10], F) @ su.RotXYZ2GLocal(rot, F)), F)

def _UFrigidBodyPosition(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return qTotal[0:3] + su.RotXYZ2RotationMatrix(qTotal[3:6], _Functions(qTotal)) @ localPosition

def _UFrigidBodyPositionJacobian(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    F = _Functions(qTotal)
    rot = qTotal[3:6]
    return _Blocks3x6(_Matrix(np.eye(3), F), -(su.RotXYZ2RotationMatrix(rot, F) @ su.Skew(localPosition, F) @ su.RotXYZ2GLocal(rot, F)), F)

def _UFrigidBodyRotationMatrix(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return su.RotXYZ2RotationMatrix(qTotal[3:6], _Functions(qTotal))

def _UFrigidBodyRotationJacobian(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    F = _Functions(qTotal)
    rot = qTotal[3:6]
    return _Blocks3x6(None, su.RotXYZ2RotationMatrix(rot, F) @ su.RotXYZ2GLocal(rot, F), F)

#the connectors and joints get the frame of each marker, an exudyn.HT, and the connectors also its twist, an
#exudyn.Twist; while recorded, a kinematics.HT and a kinematics.Twist of symbolic parts with the same methods (#3183)

#the spring-damper, ObjectConnectorUser on two position markers, parameters [stiffness, damping, referenceLength, force,
#velocityOffset]: as ObjectConnectorSpringDamper, the force k (L - L0) + d (dL/dt - v0) + f along the axis, pulling
#marker 0 towards marker 1 if positive; with cMath, it computes numbers and is recorded
def _UFspringDamperForce(mbs, t, itemNumber, HT0, twist0, HT1, twist1, dataCoordinates, parameters):
    delta = HT1.translation - HT0.translation
    length = cMath.sqrt(delta @ delta)
    axis = (1/length)*delta
    force = (parameters[0]*(length - parameters[2]) + parameters[1]*(axis @ (twist1.velocity - twist0.velocity) - parameters[4])
             + parameters[3])
    return np.hstack([force*axis, -force*axis])

#the torsional spring-damper, ObjectConnectorUser on two rigid markers, parameters [stiffness, damping, offset,
#velocityOffset, torque]: as ObjectConnectorTorsionalSpringDamper, the torque k (phi - phi0) + d (omega - omega0) + tau0
#about the z-axis of marker 0, phi the z-angle of the relative rotation A0^T A1 as Tait-Bryan angle, omega the z-component
#of the relative angular velocity in the frame of marker 0; no forces. As the forces of the spring-damper, the torques act
#on the markers: a positive torque turns marker 0 about the axis and marker 1 back
def _UFtorsionalSpringDamperForce(mbs, t, itemNumber, HT0, twist0, HT1, twist1, dataCoordinates, parameters):
    A0 = HT0.rotation
    F = _Functions(A0)
    relative = HT0.Relative(HT1).rotation
    angle = F.atan2(-relative[0, 1], relative[0, 0])
    omega = su.RelativeTwist(HT0, twist0, HT1, twist1).angularVelocity[2]
    torque = parameters[0]*(angle - parameters[2]) + parameters[1]*(omega - parameters[3]) + parameters[4]
    torque0 = [torque*A0[i, 2] for i in range(3)]
    return _Vector([0, 0, 0] + torque0 + [0, 0, 0] + [-x for x in torque0], F)

#the sphere-sphere contact, ObjectConnectorUser on two position markers with a NodeGenericData of the gap of the last
#iteration and the impact velocity, parameters [radius0, radius1, stiffness, damping, minimumImpactVelocity,
#impactFactor]: as ObjectContactSphereSphere without friction, plasticity and adhesion, the normal force k delta - d
#dg/dt on sphere 1 along the normal n from sphere 0 to sphere 1, delta = -g, g = |p1 - p0| - r0 - r1, while the gap of
#the data is negative; with an impact model the damping lambda delta dg/dt of the restitution coefficient, lambda =
#impactFactor k/v, v the impact velocity of the data or minimumImpactVelocity. Whether the item has an impact model is
#a Python constant of the recording, as it changes the computation: one force function for both, of one kind, and the
#term not recorded for impactModel 0 (#3203). The post-Newton step switches as
#CObjectContactSphereSphere::PostNewtonStep: the gap into the data, the error |k g| and the recommended step size
#|g0/(dg/dt)| of a switch, the impact velocity -dg/dt when the step begins open and ends in contact (#3197). Written
#with IfThenElse of exudyn.symbolics.utilities, which computes numbers and is recorded
def _SphereSphereContact(HT0, twist0, HT1, twist1):
    #the functions, the normal, the distance and the normal velocity of the two spheres
    delta = HT1.translation - HT0.translation
    F = _Functions(delta)
    distance = F.sqrt(delta @ delta)
    normal = (1/distance)*delta
    return (F, normal, distance, normal @ (twist1.velocity - twist0.velocity))

def _SphereSphereContactFunctions(restitution):
    #the user functions of the contact without or with an impact model, restitution a constant of the recording
    def UFforce(mbs, t, itemNumber, HT0, twist0, HT1, twist1, dataCoordinates, parameters):
        (F, normal, distance, vNormal) = _SphereSphereContact(HT0, twist0, HT1, twist1)
        gap = distance - parameters[0] - parameters[1]
        force = -parameters[2]*gap - parameters[3]*vNormal
        if restitution:
            vImpact = F.max(parameters[4], dataCoordinates[1]) if F is exudyn.symbolic else max(parameters[4], dataCoordinates[1])
            impact = vImpact > 0
            lambdaImpact = IfThenElse(impact, parameters[5]*parameters[2]/IfThenElse(impact, vImpact, 1.), 0.)
            force = force - lambdaImpact*gap*vNormal
        force = IfThenElse(dataCoordinates[0] < 0, force, 0.)
        return np.hstack([-force*normal, force*normal])
    return [('forceUserFunction', UFforce), ('postNewtonStepUserFunction', _UFsphereSphereContactPostNewton)]

def _UFsphereSphereContactPostNewton(mbs, t, itemNumber, HT0, twist0, HT1, twist1, dataCoordinates, dataCoordinatesStartOfStep,
                                     parameters):
    (F, normal, distance, vNormal) = _SphereSphereContact(HT0, twist0, HT1, twist1)
    gap = distance - parameters[0] - parameters[1]
    (lastGap, gapStartOfStep) = (dataCoordinates[0], dataCoordinatesStartOfStep[0])
    switched = (gap > 0) != (lastGap > 0)
    error = IfThenElse(switched, F.abs(gap*parameters[2]) if F is exudyn.symbolic else abs(gap*parameters[2]), 0.)
    stepKnown = (vNormal != 0)*(gapStartOfStep != 0) #a product of conditions, 0 or 1
    stepSize = IfThenElse(switched*stepKnown, abs(gapStartOfStep/IfThenElse(stepKnown, vNormal, 1.)) if F is not exudyn.symbolic
                          else F.abs(gapStartOfStep/IfThenElse(stepKnown, vNormal, 1.)), -1.)
    impactVelocity = IfThenElse((gapStartOfStep > 0)*(gap <= 0), -vNormal, dataCoordinatesStartOfStep[1])
    return [error, stepSize, gap, impactVelocity]

_sphereSphereContactFunctions = {restitution: _SphereSphereContactFunctions(restitution) for restitution in [False, True]}

#the Cartesian spring-damper, ObjectConnectorUser on two position markers, parameters [stiffness (3), damping (3),
#offset (3)]: as ObjectConnectorCartesianSpringDamper, f = k (p1 - p0 - offset) + d (v1 - v0) per global component,
#pulling marker 0 towards marker 1 for a positive component (#3193)
def _UFcartesianSpringDamperForce(mbs, t, itemNumber, HT0, twist0, HT1, twist1, dataCoordinates, parameters):
    (delta, velocity) = (HT1.translation - HT0.translation, twist1.velocity - twist0.velocity)
    force = [parameters[i]*(delta[i] - parameters[6 + i]) + parameters[3 + i]*velocity[i] for i in range(3)]
    return _Vector(force + [-x for x in force], _Functions(delta))

#the rigid-body spring-damper, ObjectConnectorUser on two rigid markers, parameters [stiffness (36, row by row),
#damping (36), offset (6), rotationMarker0 (9), rotationMarker1 (9)]: as ObjectConnectorRigidBodySpringDamper, the
#joint frame Aj - of marker 0 times rotationMarker0, or the mid-frame of the intrinsic formulation -, in it the relative
#displacement and rotation u - the Tait-Bryan angles of the relative rotation, or the logarithm on SE(3) of the
#relative frame - and the relative velocity and angular velocity v, f = K (u - offset) + D v, and the force Aj f[0:3]
#and torque Aj f[3:6] towards marker 1 on marker 0, the opposite on marker 1; intrinsic, the torque of the offset of
#the force adds to both (#3193). The formulation changes the structure of the computation, so it is a Python constant
#of the recording, not a parameter: one force function of each, both of one kind (#3203)
def _RigidBodySpringDamperForce(HT0, twist0, HT1, twist1, parameters, intrinsic):
    F = _Functions(HT0.rotation)
    (R0, R1) = (_Matrix([[parameters[78 + k + 3*i + j] for j in range(3)] for i in range(3)], F) for k in [0, 9])
    (H0, H1) = (su.HT(HT0.rotation @ R0, HT0.translation), su.HT(HT1.rotation @ R1, HT1.translation))
    relative = H0.Relative(H1)
    if intrinsic:
        motion = relative.LogSE3()
        u = [motion[i] for i in range(6)]
        Aj = (H0*su.HT().SetExpSE3([0.5*x for x in u])).rotation
    else:
        (rotation, Aj) = (relative.GetRxyz(), H0.rotation)
        u = [relative.translation[i] for i in range(3)] + [rotation[i] for i in range(3)]
    (velocity, angularVelocity) = (Aj.T @ (twist1.velocity - twist0.velocity), Aj.T @ (twist1.angularVelocity - twist0.angularVelocity))
    v = [velocity[i] for i in range(3)] + [angularVelocity[i] for i in range(3)]
    f = [sum(parameters[6*i + j]*(u[j] - parameters[72 + j]) + parameters[36 + 6*i + j]*v[j] for j in range(6)) for i in range(6)]
    (force, torque) = (Aj @ _Vector(f[0:3], F), Aj @ _Vector(f[3:6], F))
    offsetTorque = [0, 0, 0]
    if intrinsic:
        offsetTorque = su.Cross(Aj @ _Vector([-0.5*x for x in u[0:3]], F), force, F)
    return _Vector([force[i] for i in range(3)] + [torque[i] - offsetTorque[i] for i in range(3)]
                   + [-force[i] for i in range(3)] + [-torque[i] - offsetTorque[i] for i in range(3)], F)

def _RigidBodySpringDamperFunctions(intrinsic):
    #the user functions of a formulation, intrinsic a constant of the recording
    def UFforce(mbs, t, itemNumber, HT0, twist0, HT1, twist1, dataCoordinates, parameters):
        return _RigidBodySpringDamperForce(HT0, twist0, HT1, twist1, parameters, intrinsic)
    return [('forceUserFunction', UFforce)]

_rigidBodySpringDamperFunctions = {intrinsic: _RigidBodySpringDamperFunctions(intrinsic) for intrinsic in [False, True]}

#the joints, ObjectJointUser on two rigid markers whose frames are the joint frame: the equations of the built-in joint
#at position level - the global difference of the positions, as the Lagrange multipliers of the built-in joints are
#global -, and their derivative by the translations and global rotation increments [p0, theta0, p1, theta1] of the
#markers, R(theta) = (I + skew(theta)) R. For the scalar product u.v of an axis u of marker 0 and an axis v of
#marker 1 the rotation columns are u x v and v x u
def _Column(R, j):
    return [R[i, j] for i in range(3)]

def _Dot(u, v):
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]

def _Cross(u, v):
    return [u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0]]

def _JacobianRows(rows, functions):
    #the n x 12 matrix of rows of four blocks [p0, theta0, p1, theta1], None a zero block
    return _Matrix([[x for block in row for x in (block if block is not None else [0, 0, 0])] for row in rows], functions)

def _Unit(k, sign=1):
    return [sign if i == k else 0 for i in range(3)]

def _Distance(HT0, HT1):
    #the components of the global difference of the positions of the markers
    (p0, p1) = (HT0.translation, HT1.translation)
    return [p1[i] - p0[i] for i in range(3)]

#the revolute joint, as ObjectJointRevoluteZ: the relative position, and the z-axis of marker 0 perpendicular to the
#x- and y-axes of marker 1; no parameters
def _UFrevoluteJointConstraint(mbs, t, itemNumber, HT0, HT1, parameters):
    (A0, A1) = (HT0.rotation, HT1.rotation)
    z0 = _Column(A0, 2)
    return _Vector(_Distance(HT0, HT1) + [_Dot(z0, _Column(A1, 0)), _Dot(z0, _Column(A1, 1))], _Functions(A0))

def _UFrevoluteJointJacobian(mbs, t, itemNumber, HT0, HT1, parameters):
    (A0, A1) = (HT0.rotation, HT1.rotation)
    z0 = _Column(A0, 2)
    rows = [[_Unit(k, -1), None, _Unit(k), None] for k in range(3)]
    for j in [0, 1]:
        v = _Column(A1, j)
        rows.append([None, _Cross(z0, v), None, _Cross(v, z0)])
    return _JacobianRows(rows, _Functions(A0))

#the prismatic joint, as ObjectJointPrismaticX: the relative position along the y- and z-axes of marker 0, and the
#axes z0.y1, z0.x1, x0.y1; no parameters
def _UFprismaticJointConstraint(mbs, t, itemNumber, HT0, HT1, parameters):
    (A0, A1) = (HT0.rotation, HT1.rotation)
    d = _Distance(HT0, HT1)
    (x0, y0, z0) = (_Column(A0, 0), _Column(A0, 1), _Column(A0, 2))
    (x1, y1) = (_Column(A1, 0), _Column(A1, 1))
    return _Vector([_Dot(y0, d), _Dot(z0, d), _Dot(z0, y1), _Dot(z0, x1), _Dot(x0, y1)], _Functions(A0))

def _UFprismaticJointJacobian(mbs, t, itemNumber, HT0, HT1, parameters):
    (A0, A1) = (HT0.rotation, HT1.rotation)
    d = _Distance(HT0, HT1)
    (x0, y0, z0) = (_Column(A0, 0), _Column(A0, 1), _Column(A0, 2))
    (x1, y1) = (_Column(A1, 0), _Column(A1, 1))
    rows = [[[-x for x in u], _Cross(u, d), u, None] for u in [y0, z0]]
    rows += [[None, _Cross(u, v), None, _Cross(v, u)] for (u, v) in [(z0, y1), (z0, x1), (x0, y1)]]
    return _JacobianRows(rows, _Functions(A0))

#the spherical joint, as ObjectJointSpherical with all axes constrained: the relative position; no parameters
def _UFsphericalJointConstraint(mbs, t, itemNumber, HT0, HT1, parameters):
    return _Vector(_Distance(HT0, HT1), _Functions(HT0.translation))

def _UFsphericalJointJacobian(mbs, t, itemNumber, HT0, HT1, parameters):
    return _JacobianRows([[_Unit(k, -1), None, _Unit(k), None] for k in range(3)], _Functions(HT0.translation))

#the distance constraint, as ObjectConnectorDistance: |p1 - p0| - distance; parameters [distance]
def _UFdistanceConstraint(mbs, t, itemNumber, HT0, HT1, parameters):
    F = _Functions(HT0.translation)
    d = _Distance(HT0, HT1)
    return _Vector([F.sqrt(_Dot(d, d)) - parameters[0]], F)

def _UFdistanceConstraintJacobian(mbs, t, itemNumber, HT0, HT1, parameters):
    F = _Functions(HT0.translation)
    d = _Distance(HT0, HT1)
    axis = [x*(1/F.sqrt(_Dot(d, d))) for x in d]
    return _JacobianRows([[[-x for x in axis], None, axis, None]], F)

#the user functions of each kind of item, which Record records once per kind; a kind of variants - a Python constant
#of the Create function that changes the structure of the computation - has the functions of each variant in the item
#(UserItemCreator._variants), recorded once per variant; its entry here is the variant of the default arguments of the
#Create function (impactModel=0, useIntrinsicFormulation=True), which only names the kind, as every item of it keeps
#its own variant
_kindFunctions = {'massPoint': [('massMatrixUserFunction', _UFmassPointMass),
                                ('massIntegralUserFunction', _UFmassPointMassIntegral),
                                ('positionUserFunction', _UFmassPointPosition),
                                ('positionJacobianUserFunction', _UFmassPointPositionJacobian),
                                ('rotationMatrixUserFunction', _UFmassPointRotationMatrix),
                                ('rotationJacobianUserFunction', _UFmassPointRotationJacobian)],
                  'rigidBody': [('massMatrixUserFunction', _UFrigidBodyMass),
                                ('forceUserFunction', _UFrigidBodyForce),
                                ('massIntegralUserFunction', _UFrigidBodyMassIntegral),
                                ('positionUserFunction', _UFrigidBodyPosition),
                                ('positionJacobianUserFunction', _UFrigidBodyPositionJacobian),
                                ('rotationMatrixUserFunction', _UFrigidBodyRotationMatrix),
                                ('rotationJacobianUserFunction', _UFrigidBodyRotationJacobian)],
                  'springDamper': [('forceUserFunction', _UFspringDamperForce)],
                  'torsionalSpringDamper': [('forceUserFunction', _UFtorsionalSpringDamperForce)],
                  'sphereSphereContact': _sphereSphereContactFunctions[False],
                  'cartesianSpringDamper': [('forceUserFunction', _UFcartesianSpringDamperForce)],
                  'rigidBodySpringDamper': _rigidBodySpringDamperFunctions[True],
                  'coordinateConstraint': [],
                  'revoluteJoint': [('constraintUserFunction', _UFrevoluteJointConstraint),
                                    ('jacobianUserFunction', _UFrevoluteJointJacobian)],
                  'prismaticJoint': [('constraintUserFunction', _UFprismaticJointConstraint),
                                     ('jacobianUserFunction', _UFprismaticJointJacobian)],
                  'sphericalJoint': [('constraintUserFunction', _UFsphericalJointConstraint),
                                     ('jacobianUserFunction', _UFsphericalJointJacobian)],
                  'distanceConstraint': [('constraintUserFunction', _UFdistanceConstraint),
                                         ('jacobianUserFunction', _UFdistanceConstraintJacobian)]}
#the number of equations of each kind of joint
_jointConstraints = {'revoluteJoint': 5, 'prismaticJoint': 5, 'sphericalJoint': 3, 'distanceConstraint': 1}


def _Numbers(values):
    """the values of parameters given as numbers or named exudyn.symbolic.Real, for the item (#3152)"""
    return [float(value.Evaluate()) if isinstance(value, exudyn.symbolic.Real) else value for value in values]


def _Refuse(where, name, value, default, reason=''):
    """raise naming an argument the user item does not support, if it differs from its default"""
    if isinstance(value, (list, tuple, np.ndarray)) or isinstance(default, (list, tuple, np.ndarray)):
        differs = np.shape(value) != np.shape(default) or not np.array_equal(value, default)
    else:
        differs = value is not default and value != default
    if differs:
        raise ValueError('UserItemCreator.' + where + ': the argument ' + name + ' is not supported by the user item'
                         + (' (' + reason + ')' if reason else ''))


def _JointGraphics(markers, shape, size, length, color):
    """the graphicsDataUserFunction of a joint of UserItemCreator (#3147), a simplified drawing at its markers in the
    visualization configuration: 'revolute' a cylinder along the z-axis of marker 0, 'prismatic' along its x-axis,
    'spherical' a sphere at marker 0, 'distance' a line - or a cylinder of diameter size > 0 - between the markers"""
    drawColor = list(color) if color[0] >= 0 else graphics.color.grey

    def Frame(mbs, marker):
        visualization = exudyn.ConfigurationType.Visualization
        return (np.array(mbs.GetMarkerOutput(marker, exudyn.OutputVariableType.Position, configuration=visualization)),
                np.array(mbs.GetMarkerOutput(marker, exudyn.OutputVariableType.RotationMatrix,
                                             configuration=visualization)).reshape(3, 3))

    def UFgraphics(mbs, itemNumber):
        (p, A) = Frame(mbs, markers[0])
        if shape == 'spherical':
            return [graphics.Sphere(point=p, radius=size, color=drawColor, nTiles=8)]
        if shape == 'distance':
            p1 = Frame(mbs, markers[1])[0]
            if size > 0:
                return [graphics.Cylinder(pAxis=p, vAxis=p1 - p, radius=0.5*size, color=drawColor, nTiles=8)]
            return [graphics.Lines([p, p1], color=drawColor)]
        axis = A[:, 2] if shape == 'revolute' else A[:, 0]
        return [graphics.Cylinder(pAxis=p - 0.5*length*axis, vAxis=length*axis, radius=size, color=drawColor, nTiles=12)]
    return UFgraphics


class UserItemCreator:
    """creates user items with the Create functions of mbs and their arguments (#3125): ObjectUserBody and
    ObjectConnectorUser with Python user functions instead of the built-in objects, recorded as symbolic user functions
    by Record, one recording per kind of item; see the Details of this module

    Args:
        mbs: the MainSystem where the items are created

    Note:
        Its members: mbs; items, the dict of the item numbers of each kind ('massPoint', 'rigidBody', 'springDamper',
        'torsionalSpringDamper', 'revoluteJoint', 'prismaticJoint', 'sphericalJoint', 'distanceConstraint'); loads, the
        list of the loads with a user function created by CreateForce; recordings, the symbolic user functions of
        Record, which must live as long as the system; parameterVariables, the dict of the parameters of the items
        with a named exudyn.symbolic.Real among them - CreateMassPoint(mass=m1) -, by the int of the item number, as
        SymbolicSystem(mbs, parameters=...) takes it (#3152). Example:

        creator = UserItemCreator(mbs) #or creator = mbs: the same script with the built-in items
        b0 = creator.CreateMassPoint(referencePosition=[1,0,0], mass=2, gravity=[0,-9.81,0])
        creator.CreateSpringDamper(itemNumbers=[creator.CreateGround(), b0], stiffness=100, damping=1)
        creator.Record()
        mbs.Assemble()
    """
    def __init__(self, mbs):
        self.mbs = mbs
        self.items = {kind: [] for kind in _kindFunctions}
        self.loads = []
        self.recordings = []
        self._massPointNodes = {} #the node of each user mass point, by the int of its body number
        self._variants = {} #the user functions of an item of a kind of variants, by the int of its number (#3203)
        self.parameterVariables = {}

    def _KeepParameters(self, itemNumber, values):
        """keep the parameters of an item if a named exudyn.symbolic.Real is among them (#3152)"""
        if any(isinstance(value, exudyn.symbolic.Real) for value in values):
            self.parameterVariables[int(itemNumber)] = list(values)

    def CreateGround(self, name='', referencePosition=None, referenceRotationMatrix=None, graphicsDataList=[],
                     graphicsDataUserFunction=0, show=True, referenceHT=None):
        """the built-in ObjectGround, as mbs.CreateGround: it has no coordinates

        Args:
            name: as in mbs.CreateGround
            referencePosition: as in mbs.CreateGround
            referenceRotationMatrix: as in mbs.CreateGround
            graphicsDataList: as in mbs.CreateGround
            graphicsDataUserFunction: as in mbs.CreateGround
            show: as in mbs.CreateGround
            referenceHT: as in mbs.CreateGround

        Returns:
            the ObjectIndex of the ground
        """
        return self.mbs.CreateGround(name=name, referencePosition=referencePosition,
                                     referenceRotationMatrix=referenceRotationMatrix, graphicsDataList=graphicsDataList,
                                     graphicsDataUserFunction=graphicsDataUserFunction, show=show, referenceHT=referenceHT)

    def CreateMassPoint(self, name='', referencePosition=[0., 0., 0.], initialDisplacement=[0., 0., 0.],
                        initialVelocity=[0., 0., 0.], mass=0, gravity=[0., 0., 0.], graphicsDataList=[], drawSize=-1,
                        color=[-1., -1., -1., -1.], show=True, create2D=False, returnDict=False, physicsMass=None):
        """a mass point as ObjectUserBody on a NodePoint, with the nodes, markers and loads of mbs.CreateMassPoint and
        its arguments

        Args:
            name: as in mbs.CreateMassPoint
            referencePosition: as in mbs.CreateMassPoint
            initialDisplacement: as in mbs.CreateMassPoint
            initialVelocity: as in mbs.CreateMassPoint
            mass: as in mbs.CreateMassPoint; the parameter of the user body, a number or a named exudyn.symbolic.Real
            gravity: as in mbs.CreateMassPoint
            graphicsDataList: as in mbs.CreateMassPoint, the graphicsData of the user body
            drawSize: as in mbs.CreateMassPoint
            color: as in mbs.CreateMassPoint
            show: as in mbs.CreateMassPoint
            create2D: refused if True: a planar user mass point does not exist yet
            returnDict: as in mbs.CreateMassPoint
            physicsMass: refused if given: use mass

        Returns:
            the ObjectIndex of the body, or with returnDict the dict of mbs.CreateMassPoint
        """
        _Refuse('CreateMassPoint', 'create2D', create2D, False, 'a planar user mass point does not exist yet')
        _Refuse('CreateMassPoint', 'physicsMass', physicsMass, None, 'use mass')
        mbs = self.mbs
        if len(graphicsDataList) != 0:
            drawSize = 0 #the node is shown (number, basis), but not drawn, as mbs.CreateMassPoint does
        nodeNumber = mbs.AddNode(eii.NodePoint(name='Node:' + name if name != '' else '', referenceCoordinates=referencePosition,
                                               initialCoordinates=initialDisplacement, initialVelocities=initialVelocity,
                                               visualization=eii.VNodePoint(show=show, drawSize=drawSize, color=color)))
        bodyNumber = mbs.AddObject(eii.ObjectUserBody(name=name, nodeNumbers=[nodeNumber], parameters=_Numbers([mass]),
                                                      constantMassMatrix=True,
                                                      **{functionName: function for (functionName, function) in _kindFunctions['massPoint']},
                                                      visualization=eii.VObjectUserBody(show=graphicsDataList != [],
                                                                                        graphicsData=graphicsDataList)))
        self.items['massPoint'].append(bodyNumber)
        self._KeepParameters(bodyNumber, [mass])
        self._massPointNodes[int(bodyNumber)] = nodeNumber
        result = {'nodeNumber': nodeNumber, 'bodyNumber': bodyNumber}
        if list(gravity) != [0., 0., 0.]:
            result['markerBodyMass'] = mbs.AddMarker(eii.MarkerBodyMass(bodyNumber=bodyNumber))
            result['loadNumber'] = mbs.AddLoad(eii.LoadMassProportional(markerNumber=result['markerBodyMass'], loadVector=gravity))
        return result if returnDict else bodyNumber

    def CreateRigidBody(self, name='', referencePosition=None, referenceRotationMatrix=None, initialVelocity=[0., 0., 0.],
                        initialAngularVelocity=[0., 0., 0.], initialDisplacement=None, initialRotationMatrix=None,
                        inertia=None, gravity=[0., 0., 0.], nodeType=exudyn.NodeType.RotationEulerParameters,
                        graphicsDataList=[], graphicsDataUserFunction=0, drawSize=-1, color=[-1., -1., -1., -1.],
                        show=True, create2D=False, returnDict=False, referenceHT=None, initialHT=None):
        """a rigid body as ObjectUserBody on a NodeRigidBodyRxyz, with the node, markers and loads of
        mbs.CreateRigidBody and its arguments; nodeType must be exu.NodeType.RotationRxyz

        Args:
            name: as in mbs.CreateRigidBody
            referencePosition: as in mbs.CreateRigidBody
            referenceRotationMatrix: as in mbs.CreateRigidBody
            initialVelocity: as in mbs.CreateRigidBody
            initialAngularVelocity: as in mbs.CreateRigidBody
            initialDisplacement: as in mbs.CreateRigidBody
            initialRotationMatrix: as in mbs.CreateRigidBody
            inertia: as in mbs.CreateRigidBody; its mass, inertia about the node and center of mass are the parameters
                of the user body
            gravity: as in mbs.CreateRigidBody
            nodeType: exu.NodeType.RotationRxyz; the default RotationEulerParameters is refused (the user body does not
                give the algebraic equation of the Euler parameters), RotationRotationVector too (a Lie group node)
            graphicsDataList: as in mbs.CreateRigidBody, the graphicsData of the user body; empty: the graphics of inertia
            graphicsDataUserFunction: refused if given: the one of ObjectUserBody has other arguments
            drawSize: as in mbs.CreateRigidBody
            color: as in mbs.CreateRigidBody
            show: as in mbs.CreateRigidBody
            create2D: refused if True: a planar user rigid body does not exist yet
            returnDict: as in mbs.CreateRigidBody
            referenceHT: as in mbs.CreateRigidBody
            initialHT: as in mbs.CreateRigidBody

        Returns:
            the ObjectIndex of the body, or with returnDict the dict of mbs.CreateRigidBody
        """
        where = 'UserItemCreator.CreateRigidBody'
        _Refuse('CreateRigidBody', 'create2D', create2D, False, 'a planar user rigid body does not exist yet')
        _Refuse('CreateRigidBody', 'graphicsDataUserFunction', graphicsDataUserFunction, 0,
                'the one of ObjectUserBody has other arguments')
        if nodeType != exudyn.NodeType.RotationRxyz:
            raise ValueError(where + ': the argument nodeType ' + str(nodeType) + ' is not supported by the user item '
                             '(only exu.NodeType.RotationRxyz: Euler parameters need their algebraic equation, the '
                             'rotation vector node is a Lie group node)')
        if inertia is None:
            raise ValueError(where + ': inertia must be given')
        (referencePosition, referenceRotationMatrix) = _FrameFromHT(where, 'referenceHT', referenceHT, 'referencePosition',
                                                                    referencePosition, 'referenceRotationMatrix', referenceRotationMatrix)
        (initialDisplacement, initialRotationMatrix) = _FrameFromHT(where, 'initialHT', initialHT, 'initialDisplacement',
                                                                    initialDisplacement, 'initialRotationMatrix', initialRotationMatrix)
        referencePosition = [0., 0., 0.] if referencePosition is None else list(referencePosition)
        referenceRotationMatrix = np.eye(3) if referenceRotationMatrix is None else np.array(referenceRotationMatrix)
        mbs = self.mbs
        graphicsDataList0 = graphicsDataList
        if len(graphicsDataList) == 0:
            inertiaGraphics = inertia.GetGraphics(color)
            graphicsDataList0 = [inertiaGraphics] if inertiaGraphics is not None else []
        if len(graphicsDataList0) != 0:
            drawSize = 0 #the node is shown (number, basis), but not drawn, as mbs.CreateRigidBody does
        #the node as mbs.CreateRigidBody computes it for RotationRxyz
        referenceRotation = RotationMatrix2RotXYZ(referenceRotationMatrix)
        initialCoordinates = [0.]*6
        if initialDisplacement is not None or initialRotationMatrix is not None:
            initialDisplacement = [0., 0., 0.] if initialDisplacement is None else initialDisplacement
            initialRotationMatrix = np.eye(3) if initialRotationMatrix is None else initialRotationMatrix
            initialCoordinates = list(initialDisplacement) + list(RotationMatrix2RotXYZ(referenceRotationMatrix @ initialRotationMatrix)
                                                                 - referenceRotation)
        nodeNumber = mbs.AddNode(eii.NodeRigidBodyRxyz(name='Node:' + name if name != '' else '',
                                                       referenceCoordinates=referencePosition + list(referenceRotation),
                                                       initialCoordinates=initialCoordinates,
                                                       initialVelocities=list(initialVelocity) + list(AngularVelocity2RotXYZ_t(initialAngularVelocity, referenceRotation)),
                                                       visualization=eii.VNodeRigidBodyRxyz(show=show, drawSize=drawSize, color=color)))
        bodyNumber = mbs.AddObject(eii.ObjectUserBody(name=name, nodeNumbers=[nodeNumber],
                                                      parameters=[inertia.mass] + list(inertia.GetInertia6D()) + list(inertia.com),
                                                      centerOfMass=list(inertia.com),
                                                      **{functionName: function for (functionName, function) in _kindFunctions['rigidBody']},
                                                      visualization=eii.VObjectUserBody(show=show, graphicsData=graphicsDataList0)))
        self.items['rigidBody'].append(bodyNumber)
        result = {'nodeNumber': nodeNumber, 'bodyNumber': bodyNumber}
        if cMath.norm(gravity) != 0.:
            result['markerBodyMass'] = mbs.AddMarker(eii.MarkerBodyMass(bodyNumber=bodyNumber))
            result['loadNumber'] = mbs.AddLoad(eii.LoadMassProportional(markerNumber=result['markerBodyMass'], loadVector=gravity))
        return result if returnDict else bodyNumber

    def CreateSpringDamper(self, name='', itemNumbers=[None, None], localPosition0=[0., 0., 0.], localPosition1=[0., 0., 0.],
                           referenceLength=None, stiffness=0., damping=0., force=0., velocityOffset=0.,
                           springForceUserFunction=0, show=True, drawSize=-1, color=exudyn.graphics.color.default,
                           bodyNumbers=None, bodyOrNodeList=None):
        """a spring-damper as ObjectConnectorUser on the markers of mbs.CreateSpringDamper, with its arguments and its
        force; at the center of a user mass point the marker is on its node, which has the same position; drawn by a
        graphicsDataUserFunction

        Args:
            name: as in mbs.CreateSpringDamper
            itemNumbers: as in mbs.CreateSpringDamper
            localPosition0: as in mbs.CreateSpringDamper
            localPosition1: as in mbs.CreateSpringDamper
            referenceLength: as in mbs.CreateSpringDamper; a number or a named exudyn.symbolic.Real
            stiffness: as in mbs.CreateSpringDamper; a number or a named exudyn.symbolic.Real
            damping: as in mbs.CreateSpringDamper; a number or a named exudyn.symbolic.Real
            force: as in mbs.CreateSpringDamper; a number or a named exudyn.symbolic.Real
            velocityOffset: as in mbs.CreateSpringDamper; a number or a named exudyn.symbolic.Real
            springForceUserFunction: refused if given: the force is the forceUserFunction of the connector
            show: as in mbs.CreateSpringDamper
            drawSize: a line if not positive, else a cylinder of diameter drawSize
            color: the color of the drawing; the default: grey
            bodyNumbers: refused if given: use itemNumbers
            bodyOrNodeList: refused if given: use itemNumbers

        Returns:
            the ObjectIndex of the connector
        """
        _Refuse('CreateSpringDamper', 'springForceUserFunction', springForceUserFunction, 0,
                'the force is the forceUserFunction of the connector')
        _Refuse('CreateSpringDamper', 'bodyNumbers', bodyNumbers, None, 'use itemNumbers')
        _Refuse('CreateSpringDamper', 'bodyOrNodeList', bodyOrNodeList, None, 'use itemNumbers')
        mbs = self.mbs
        #at the center of a user mass point, the marker is on its node, which SymbolicSystem takes (#3121); the position
        #is the same as on the body
        itemNumbers = [self._massPointNodes[int(item)] if (isinstance(item, exudyn.ObjectIndex) and int(item) in self._massPointNodes
                                                           and not np.any(localPosition)) else item
                       for (item, localPosition) in zip(itemNumbers, [localPosition0, localPosition1])]
        [marker0, marker1, p0, p1, A0, A1] = GetMarkersPosRot(mbs, name, itemNumbers, localPosition0, localPosition1,
                                                              getPosition=True, getRotationMatrix=False, useRigidMarker=False)
        if referenceLength is None:
            referenceLength = cMath.norm(np.array(p1) - p0)
        parameters = [stiffness, damping, referenceLength, force, velocityOffset]
        drawColor = list(color) if color[0] >= 0 else graphics.color.grey

        def UFgraphics(mbs, itemNumber):
            positions = [np.array(mbs.GetMarkerOutput(marker, exudyn.OutputVariableType.Position,
                                                      configuration=exudyn.ConfigurationType.Visualization))
                         for marker in [marker0, marker1]]
            if drawSize > 0:
                return [graphics.Cylinder(pAxis=positions[0], vAxis=positions[1] - positions[0], radius=0.5*drawSize,
                                          color=drawColor, nTiles=8)]
            return [graphics.Lines(positions, color=drawColor)]

        connector = mbs.AddObject(eii.ObjectConnectorUser(name=name, markerNumbers=[marker0, marker1],
                                                          parameters=_Numbers(parameters),
                                                          forceUserFunction=_UFspringDamperForce,
                                                          visualization=eii.VObjectConnectorUser(show=show, graphicsDataUserFunction=UFgraphics)))
        self.items['springDamper'].append(connector)
        self._KeepParameters(connector, parameters)
        return connector

    def CreateSphereSphereContact(self, name='', itemNumbers=[None, None], localPosition0=[0., 0., 0.],
                                  localPosition1=[0., 0., 0.], spheresRadii=[-1, -1], isHollowSphere1=False,
                                  dynamicFriction=0., frictionProportionalZone=1e-3, contactStiffness=0., contactDamping=0.,
                                  contactStiffnessExponent=1, constantPullOffForce=0, contactPlasticityRatio=0,
                                  adhesionCoefficient=0, adhesionExponent=1, restitutionCoefficient=1,
                                  minimumImpactVelocity=0, impactModel=0, dataInitialCoordinates=[0, 0, 0, 0],
                                  activeConnector=True, show=False, color=exudyn.graphics.color.default, bodyNumbers=None,
                                  bodyOrNodeList=None):
        """a sphere-sphere contact as ObjectConnectorUser on the position markers of mbs.CreateSphereSphereContact, with
        the normal force of ObjectContactSphereSphere, k delta - d dg/dt, and with impactModel 1 or 2 the damping of the
        restitution coefficient - the model 2 is the recommended one -; switched as the built-in contact by the gap of
        a data node in the post-Newton step, which is recorded as the force (#3187, #3197); drawn as two spheres by a
        graphicsDataUserFunction

        Args:
            name: as in mbs.CreateSphereSphereContact
            itemNumbers: as in mbs.CreateSphereSphereContact
            localPosition0: as in mbs.CreateSphereSphereContact
            localPosition1: as in mbs.CreateSphereSphereContact
            spheresRadii: as in mbs.CreateSphereSphereContact; numbers or named exudyn.symbolic.Real
            isHollowSphere1: refused if True
            dynamicFriction: refused if not 0: friction needs the data node and the post-Newton step
            frictionProportionalZone: not used, as there is no friction
            contactStiffness: as in mbs.CreateSphereSphereContact; a number or a named exudyn.symbolic.Real
            contactDamping: as in mbs.CreateSphereSphereContact; a number or a named exudyn.symbolic.Real
            contactStiffnessExponent: refused if not 1: the linear contact law
            constantPullOffForce: refused if not 0: no adhesion
            contactPlasticityRatio: refused if not 0: no plasticity
            adhesionCoefficient: refused if not 0: no adhesion
            adhesionExponent: refused if not 1: no adhesion
            restitutionCoefficient: as in mbs.CreateSphereSphereContact, with impactModel 1 or 2
            minimumImpactVelocity: as in mbs.CreateSphereSphereContact, with impactModel 1 or 2
            impactModel: 0 (the default), 1 (Hunt-Crossley) or 2 (Gonthier and Carvalho-Martins, recommended), as in
                mbs.CreateSphereSphereContact; the items without and with an impact model share a recording each
            dataInitialCoordinates: as in mbs.CreateSphereSphereContact; the gap and the impact velocity (the components
                0 and 2) go into the data node of the connector, the others must be 0, as there is no friction and
                plasticity
            activeConnector: refused if False
            show: draw the two spheres
            color: the color of the drawing; the default: grey
            bodyNumbers: refused if given: use itemNumbers
            bodyOrNodeList: refused if given: use itemNumbers

        Returns:
            the ObjectIndex of the connector
        """
        where = 'CreateSphereSphereContact'
        for (argument, value, default, reason) in [
                ('isHollowSphere1', isHollowSphere1, False, ''),
                ('dynamicFriction', dynamicFriction, 0., 'friction needs the data node and the post-Newton step'),
                ('contactStiffnessExponent', contactStiffnessExponent, 1, 'the contact law is linear'),
                ('constantPullOffForce', constantPullOffForce, 0, 'no adhesion'),
                ('contactPlasticityRatio', contactPlasticityRatio, 0, 'no plasticity'),
                ('adhesionCoefficient', adhesionCoefficient, 0, 'no adhesion'),
                ('adhesionExponent', adhesionExponent, 1, 'no adhesion'),
                ('activeConnector', activeConnector, True, ''),
                ('bodyNumbers', bodyNumbers, None, 'use itemNumbers'),
                ('bodyOrNodeList', bodyOrNodeList, None, 'use itemNumbers')]:
            _Refuse(where, argument, value, default, reason)
        if impactModel not in [0, 1, 2]:
            raise ValueError('UserItemCreator.CreateSphereSphereContact: impactModel must be 0, 1 or 2, not ' + str(impactModel))
        if impactModel == 0:
            _Refuse(where, 'restitutionCoefficient', restitutionCoefficient, 1, 'the restitution coefficient needs impactModel 1 or 2')
            _Refuse(where, 'minimumImpactVelocity', minimumImpactVelocity, 0, 'the impact velocity needs impactModel 1 or 2')
        _Refuse(where, 'dataInitialCoordinates', [dataInitialCoordinates[1], dataInitialCoordinates[3]], [0, 0],
                'the tangent velocity and the plastic deformation: there is no friction and plasticity')
        #the factor of lambda of CObjectContactSphereSphere for the restitution coefficient e
        e = float(restitutionCoefficient)
        if impactModel != 0 and e <= 0:
            raise ValueError('UserItemCreator.CreateSphereSphereContact: the restitution coefficient must be > 0, not ' + str(e))
        impactFactor = 0.
        if impactModel == 1:
            impactFactor = 1.5*(e - 1.)
        elif impactModel == 2:
            impactFactor = 1.5*(e - 1.)*(11. - e)/(1. + 9.*e) if e > 1./3. else (e*e - 1.)/e
        mbs = self.mbs
        itemNumbers = [self._massPointNodes[int(item)] if (isinstance(item, exudyn.ObjectIndex) and int(item) in self._massPointNodes
                                                           and not np.any(localPosition)) else item
                       for (item, localPosition) in zip(itemNumbers, [localPosition0, localPosition1])]
        [marker0, marker1, p0, p1, A0, A1] = GetMarkersPosRot(mbs, name, itemNumbers, localPosition0, localPosition1,
                                                              getPosition=True, getRotationMatrix=False, useRigidMarker=False)
        parameters = [spheresRadii[0], spheresRadii[1], contactStiffness, contactDamping, minimumImpactVelocity, impactFactor]
        radii = _Numbers(spheresRadii)
        drawColor = list(color) if color[0] >= 0 else graphics.color.grey

        def UFgraphics(mbs, itemNumber):
            return [graphics.Sphere(point=mbs.GetMarkerOutput(marker, exudyn.OutputVariableType.Position,
                                                              configuration=exudyn.ConfigurationType.Visualization),
                                    radius=radius, color=drawColor, nTiles=12)
                    for (marker, radius) in zip([marker0, marker1], radii)]

        functions = _sphereSphereContactFunctions[impactModel != 0]
        data = mbs.AddNode(eii.NodeGenericData(initialCoordinates=[dataInitialCoordinates[0], dataInitialCoordinates[2]],
                                               numberOfDataCoordinates=2))
        connector = mbs.AddObject(eii.ObjectConnectorUser(name=name, markerNumbers=[marker0, marker1], nodeNumber=data,
                                                          parameters=_Numbers(parameters), **dict(functions),
                                                          visualization=eii.VObjectConnectorUser(show=show, graphicsDataUserFunction=UFgraphics)))
        self.items['sphereSphereContact'].append(connector)
        self._variants[int(connector)] = functions
        self._KeepParameters(connector, parameters)
        return connector

    def CreateTorsionalSpringDamper(self, name='', itemNumbers=[None, None], position=[0., 0., 0.], axis=[0., 0., 0.],
                                    stiffness=0., damping=0., offset=0., velocityOffset=0., torque=0., useGlobalFrame=True,
                                    springTorqueUserFunction=0, unlimitedRotations=True, show=True, drawSize=-1,
                                    color=exudyn.graphics.color.default, bodyNumbers=None, bodyOrNodeList=None):
        """a torsional spring-damper as ObjectConnectorUser on the rigid markers of mbs.CreateTorsionalSpringDamper,
        with its arguments and its torque about the axis; the angle is that of the relative rotation in (-pi, pi], so
        unlimitedRotations must be False; drawn by a graphicsDataUserFunction

        Args:
            name: as in mbs.CreateTorsionalSpringDamper
            itemNumbers: as in mbs.CreateTorsionalSpringDamper, two bodies or the ground
            position: as in mbs.CreateTorsionalSpringDamper
            axis: as in mbs.CreateTorsionalSpringDamper
            stiffness: as in mbs.CreateTorsionalSpringDamper; a number or a named exudyn.symbolic.Real
            damping: as in mbs.CreateTorsionalSpringDamper; a number or a named exudyn.symbolic.Real
            offset: as in mbs.CreateTorsionalSpringDamper; a number or a named exudyn.symbolic.Real
            velocityOffset: as in mbs.CreateTorsionalSpringDamper; a number or a named exudyn.symbolic.Real
            torque: as in mbs.CreateTorsionalSpringDamper; a number or a named exudyn.symbolic.Real
            useGlobalFrame: as in mbs.CreateTorsionalSpringDamper
            springTorqueUserFunction: refused if given: the torque is the forceUserFunction of the connector
            unlimitedRotations: must be False: the turns need a data coordinate, which the symbolic system does not take
            show: as in mbs.CreateTorsionalSpringDamper
            drawSize: the diameter of a cylinder along the axis; not positive: a line
            color: the color of the drawing; the default: grey
            bodyNumbers: refused if given: use itemNumbers
            bodyOrNodeList: refused if given: use itemNumbers

        Returns:
            the ObjectIndex of the connector
        """
        _Refuse('CreateTorsionalSpringDamper', 'springTorqueUserFunction', springTorqueUserFunction, 0,
                'the torque is the forceUserFunction of the connector')
        _Refuse('CreateTorsionalSpringDamper', 'unlimitedRotations', unlimitedRotations, False,
                'the turns need a data coordinate; pass unlimitedRotations=False')
        _Refuse('CreateTorsionalSpringDamper', 'bodyNumbers', bodyNumbers, None, 'use itemNumbers')
        _Refuse('CreateTorsionalSpringDamper', 'bodyOrNodeList', bodyOrNodeList, None, 'use itemNumbers')
        if any(isinstance(item, exudyn.MarkerIndex) for item in itemNumbers):
            raise ValueError('UserItemCreator.CreateTorsionalSpringDamper: the argument itemNumbers with a marker is not '
                             'supported by the user item (give the bodies)')
        mbs = self.mbs
        #the markers as mbs.CreateTorsionalSpringDamper adds them: the frame of the joint, its z-axis the axis
        [p0, A0, p1, A1, marker0, marker1, pJoint] = JointPreCheckCalcBodyMarkers('UserItemCreator.CreateTorsionalSpringDamper', mbs,
                                                                                  name, itemNumbers, position, show, useGlobalFrame)
        B = ComputeOrthonormalBasis(np.array(axis) if useGlobalFrame else A0 @ axis)
        AJ = np.array([-B[:, 2], B[:, 1], B[:, 0]]).T
        markers = [marker0, marker1]
        for (k, (p, A)) in enumerate([(p0, A0), (p1, A1)]):
            if markers[k] is None:
                markers[k] = mbs.AddMarker(eii.MarkerBodyRigid(name='Marker' + str(k) + ':' + name if name != '' else '',
                                                               bodyNumber=itemNumbers[k],
                                                               localHT=exudyn.HT(rotation=A.T @ AJ, translation=A.T @ (np.array(pJoint) - p))))
        length = drawSize if drawSize > 0 else 0.1
        drawColor = list(color) if color[0] >= 0 else graphics.color.grey

        def UFgraphics(mbs, itemNumber):
            p = np.array(mbs.GetMarkerOutput(markers[0], exudyn.OutputVariableType.Position,
                                             configuration=exudyn.ConfigurationType.Visualization))
            A = np.array(mbs.GetMarkerOutput(markers[0], exudyn.OutputVariableType.RotationMatrix,
                                             configuration=exudyn.ConfigurationType.Visualization)).reshape(3, 3)
            if drawSize > 0:
                return [graphics.Cylinder(pAxis=p - 0.5*length*A[:, 2], vAxis=length*A[:, 2], radius=0.5*drawSize,
                                          color=drawColor, nTiles=8)]
            return [graphics.Lines([p - 0.5*length*A[:, 2], p + 0.5*length*A[:, 2]], color=drawColor)]

        connector = mbs.AddObject(eii.ObjectConnectorUser(name=name, markerNumbers=markers,
                                                          parameters=_Numbers([stiffness, damping, offset, velocityOffset, torque]),
                                                          requestedMarkerType=exudyn.MarkerType.Position | exudyn.MarkerType.Orientation,
                                                          forceUserFunction=_UFtorsionalSpringDamperForce,
                                                          visualization=eii.VObjectConnectorUser(show=show, graphicsDataUserFunction=UFgraphics)))
        self.items['torsionalSpringDamper'].append(connector)
        self._KeepParameters(connector, [stiffness, damping, offset, velocityOffset, torque])
        return connector

    def _JointMarkers(self, where, name, itemNumbers, position, axis, useGlobalFrame, show, frame):
        """the two rigid markers of a joint at position, as mbs.Create...Joint computes them: their rotation the joint
        frame - frame 'z' its z-axis the axis (revolute), 'x' its x-axis the axis (prismatic), None the body frame"""
        if any(isinstance(item, exudyn.MarkerIndex) for item in itemNumbers):
            raise ValueError('UserItemCreator.' + where + ': the argument itemNumbers with a marker is not supported by the '
                             'user item (give the bodies)')
        mbs = self.mbs
        [p0, A0, p1, A1, _, _, pJoint] = JointPreCheckCalcBodyMarkers('UserItemCreator.' + where, mbs, name, itemNumbers,
                                                                      position, show, useGlobalFrame)
        AJ = np.eye(3)
        if frame is not None:
            B = ComputeOrthonormalBasis(np.array(axis) if useGlobalFrame else A0 @ axis)
            AJ = np.array([-B[:, 2], B[:, 1], B[:, 0]]).T if frame == 'z' else B
        return [mbs.AddMarker(eii.MarkerBodyRigid(name='Marker' + str(k) + ':' + name if name != '' else '', bodyNumber=itemNumbers[k],
                                                  localHT=exudyn.HT(rotation=(A.T @ AJ) if frame is not None else np.eye(3),
                                                                    translation=A.T @ (np.array(pJoint) - p))))
                for (k, (p, A)) in enumerate([(p0, A0), (p1, A1)])]

    def _Joint(self, kind, name, markers, parameters, show, drawing):
        """an ObjectJointUser of a kind of joint on two rigid markers, drawn by the graphicsDataUserFunction drawing"""
        joint = self.mbs.AddObject(eii.ObjectJointUser(name=name, markerNumbers=markers, numberOfConstraints=_jointConstraints[kind],
                                                       parameters=parameters,
                                                       **{functionName: function for (functionName, function) in _kindFunctions[kind]},
                                                       visualization=eii.VObjectJointUser(show=show, graphicsDataUserFunction=drawing)))
        self.items[kind].append(joint)
        return joint

    def CreateCartesianSpringDamper(self, name='', itemNumbers=[None, None], localPosition0=[0., 0., 0.],
                                    localPosition1=[0., 0., 0.], stiffness=[0., 0., 0.], damping=[0., 0., 0.],
                                    offset=[0., 0., 0.], springForceUserFunction=0, show=True, drawSize=-1,
                                    color=exudyn.graphics.color.default, bodyNumbers=None, bodyOrNodeList=None):
        """a Cartesian spring-damper as ObjectConnectorUser on the position markers of mbs.CreateCartesianSpringDamper,
        with its arguments and the force of ObjectConnectorCartesianSpringDamper; drawn as a line (#3193)

        Args:
            name: as in mbs.CreateCartesianSpringDamper
            itemNumbers: as in mbs.CreateCartesianSpringDamper
            localPosition0: as in mbs.CreateCartesianSpringDamper
            localPosition1: as in mbs.CreateCartesianSpringDamper
            stiffness: as in mbs.CreateCartesianSpringDamper; numbers or named exudyn.symbolic.Real
            damping: as in mbs.CreateCartesianSpringDamper; numbers or named exudyn.symbolic.Real
            offset: as in mbs.CreateCartesianSpringDamper; numbers or named exudyn.symbolic.Real
            springForceUserFunction: refused if given: the force is the forceUserFunction of the connector
            show: as in mbs.CreateCartesianSpringDamper
            drawSize: not used: a line between the markers
            color: the color of the line; the default: grey
            bodyNumbers: refused if given: use itemNumbers
            bodyOrNodeList: refused if given: use itemNumbers

        Returns:
            the ObjectIndex of the connector
        """
        where = 'CreateCartesianSpringDamper'
        _Refuse(where, 'springForceUserFunction', springForceUserFunction, 0, 'the force is the forceUserFunction of the connector')
        _Refuse(where, 'bodyNumbers', bodyNumbers, None, 'use itemNumbers')
        _Refuse(where, 'bodyOrNodeList', bodyOrNodeList, None, 'use itemNumbers')
        mbs = self.mbs
        itemNumbers = [self._massPointNodes[int(item)] if (isinstance(item, exudyn.ObjectIndex) and int(item) in self._massPointNodes
                                                           and not np.any(localPosition)) else item
                       for (item, localPosition) in zip(itemNumbers, [localPosition0, localPosition1])]
        [marker0, marker1, p0, p1, A0, A1] = GetMarkersPosRot(mbs, name, itemNumbers, localPosition0, localPosition1,
                                                              getPosition=False, getRotationMatrix=False, useRigidMarker=False)
        parameters = list(stiffness) + list(damping) + list(offset)
        connector = mbs.AddObject(eii.ObjectConnectorUser(name=name, markerNumbers=[marker0, marker1], parameters=_Numbers(parameters),
                                                          forceUserFunction=_UFcartesianSpringDamperForce,
                                                          visualization=eii.VObjectConnectorUser(show=show, graphicsDataUserFunction=
                                                              _JointGraphics([marker0, marker1], 'distance', 0, 0, color))))
        self.items['cartesianSpringDamper'].append(connector)
        self._KeepParameters(connector, parameters)
        return connector

    def CreateRigidBodySpringDamper(self, name='', itemNumbers=[None, None], localPosition0=[0., 0., 0.],
                                    localPosition1=[0., 0., 0.], stiffness=np.zeros((6, 6)), damping=np.zeros((6, 6)),
                                    offset=[0., 0., 0., 0., 0., 0.], rotationMatrixJoint=np.eye(3), useGlobalFrame=True,
                                    useIntrinsicFormulation=True, springForceTorqueUserFunction=0,
                                    postNewtonStepUserFunction=0, show=True, drawSize=-1, color=exudyn.graphics.color.default,
                                    intrinsicFormulation=None, bodyNumbers=None, bodyOrNodeList=None):
        """a rigid-body spring-damper as ObjectConnectorUser on the rigid markers of mbs.CreateRigidBodySpringDamper, with
        its arguments and the forces and torques of ObjectConnectorRigidBodySpringDamper in both formulations - the
        intrinsic one by the logarithm on SE(3) of the relative frame, the other by the Tait-Bryan angles of the relative
        rotation -; the joint rotations are parameters of the connector; drawn as a line (#3193)

        Args:
            name: as in mbs.CreateRigidBodySpringDamper
            itemNumbers: as in mbs.CreateRigidBodySpringDamper, two bodies, rigid nodes or the ground
            localPosition0: as in mbs.CreateRigidBodySpringDamper
            localPosition1: as in mbs.CreateRigidBodySpringDamper
            stiffness: as in mbs.CreateRigidBodySpringDamper, 6 x 6; numbers or named exudyn.symbolic.Real
            damping: as in mbs.CreateRigidBodySpringDamper, 6 x 6; numbers or named exudyn.symbolic.Real
            offset: as in mbs.CreateRigidBodySpringDamper; numbers or named exudyn.symbolic.Real
            rotationMatrixJoint: as in mbs.CreateRigidBodySpringDamper
            useGlobalFrame: as in mbs.CreateRigidBodySpringDamper
            useIntrinsicFormulation: as in mbs.CreateRigidBodySpringDamper; the items of each formulation share a
                recording
            springForceTorqueUserFunction: refused if given: the forces are the forceUserFunction of the connector
            postNewtonStepUserFunction: refused if given: a post-Newton step needs data coordinates
            show: as in mbs.CreateRigidBodySpringDamper
            drawSize: not used: a line between the markers
            color: the color of the line; the default: grey
            intrinsicFormulation: refused if given: use useIntrinsicFormulation
            bodyNumbers: refused if given: use itemNumbers
            bodyOrNodeList: refused if given: use itemNumbers

        Returns:
            the ObjectIndex of the connector
        """
        where = 'CreateRigidBodySpringDamper'
        for (argument, value, default, reason) in [
                ('springForceTorqueUserFunction', springForceTorqueUserFunction, 0, 'the forces are the forceUserFunction of the connector'),
                ('postNewtonStepUserFunction', postNewtonStepUserFunction, 0, 'a post-Newton step needs data coordinates'),
                ('intrinsicFormulation', intrinsicFormulation, None, 'use useIntrinsicFormulation'),
                ('bodyNumbers', bodyNumbers, None, 'use itemNumbers'),
                ('bodyOrNodeList', bodyOrNodeList, None, 'use itemNumbers')]:
            _Refuse(where, argument, value, default, reason)
        if any(isinstance(item, exudyn.MarkerIndex) for item in itemNumbers):
            raise ValueError('UserItemCreator.CreateRigidBodySpringDamper: the argument itemNumbers with a marker is not '
                             'supported by the user item (give the bodies or nodes)')
        mbs = self.mbs
        [marker0, marker1, p0, p1, A0, A1] = GetMarkersPosRot(mbs, name, itemNumbers, localPosition0, localPosition1,
                                                              getPosition=False, getRotationMatrix=True, useRigidMarker=True)
        rotationMatrixJoint = np.array(rotationMatrixJoint, dtype=float)
        if useGlobalFrame: #as mbs.CreateRigidBodySpringDamper
            (MR0, MR1) = (A0.T @ rotationMatrixJoint, A1.T @ rotationMatrixJoint)
        else:
            (MR0, MR1) = (rotationMatrixJoint, A1.T @ A0 @ rotationMatrixJoint)
        parameters = ([x for row in stiffness for x in row] + [x for row in damping for x in row] + list(offset)
                      + list(MR0.flatten()) + list(MR1.flatten()))
        functions = _rigidBodySpringDamperFunctions[bool(useIntrinsicFormulation)]
        connector = mbs.AddObject(eii.ObjectConnectorUser(name=name, markerNumbers=[marker0, marker1], parameters=_Numbers(parameters),
                                                          requestedMarkerType=exudyn.MarkerType.Position | exudyn.MarkerType.Orientation,
                                                          **dict(functions),
                                                          visualization=eii.VObjectConnectorUser(show=show, graphicsDataUserFunction=
                                                              _JointGraphics([marker0, marker1], 'distance', 0, 0, color))))
        self.items['rigidBodySpringDamper'].append(connector)
        self._variants[int(connector)] = functions
        self._KeepParameters(connector, parameters)
        return connector

    def CreateCoordinateConstraint(self, name='', itemNumbers=[None, None], coordinates=[None, None], offset=0.,
                                   factor1=1., velocityLevel=False, offsetUserFunction=0, offsetUserFunction_t=0, show=True,
                                   drawSize=-1., color=exudyn.graphics.color.default, includeReferenceCoordinates=[False, False],
                                   factorValue1=None, bodyNumbers=None, bodyOrNodeList=None):
        """a coordinate constraint as ObjectGenericDAE with the equation of ObjectConnectorCoordinate, factor1 q1 - q0 -
        offset = 0, on the coordinates of mbs.CreateCoordinateConstraint: the nodes of the coordinates are its ODE2 nodes,
        a NodeGenericAE its multiplier, the equation its constraintMatrixODE2 and constraintVector (#3193)

        Args:
            name: as in mbs.CreateCoordinateConstraint
            itemNumbers: as in mbs.CreateCoordinateConstraint: a body, a node, or None or a ground object for the ground
            coordinates: as in mbs.CreateCoordinateConstraint
            offset: as in mbs.CreateCoordinateConstraint
            factor1: as in mbs.CreateCoordinateConstraint
            velocityLevel: refused if True: the equation is at position level
            offsetUserFunction: refused if given: the offset is constant
            offsetUserFunction_t: refused if given: the offset is constant
            show: not used: the object draws nothing
            drawSize: not used
            color: not used
            includeReferenceCoordinates: as in mbs.CreateCoordinateConstraint
            factorValue1: refused if given: use factor1
            bodyNumbers: refused if given: use itemNumbers
            bodyOrNodeList: refused if given: use itemNumbers

        Returns:
            the ObjectIndex of the ObjectGenericDAE
        """
        where = 'CreateCoordinateConstraint'
        for (argument, value, default, reason) in [('velocityLevel', velocityLevel, False, 'the equation is at position level'),
                                                   ('offsetUserFunction', offsetUserFunction, 0, 'the offset is constant'),
                                                   ('offsetUserFunction_t', offsetUserFunction_t, 0, 'the offset is constant'),
                                                   ('factorValue1', factorValue1, None, 'use factor1'),
                                                   ('bodyNumbers', bodyNumbers, None, 'use itemNumbers'),
                                                   ('bodyOrNodeList', bodyOrNodeList, None, 'use itemNumbers')]:
            _Refuse(where, argument, value, default, reason)
        mbs = self.mbs
        (nodes, columns, factors) = ([], [], [-1., float(factor1)])
        c0 = float(offset)
        for (item, coordinate, factor, includeReference) in zip(itemNumbers, coordinates, factors, includeReferenceCoordinates):
            if item is None or (isinstance(item, exudyn.ObjectIndex) and mbs.GetObject(item)['objectType'] == 'Ground'):
                continue #the ground: the coordinate is 0
            if isinstance(item, exudyn.NodeIndex):
                (node, nodeCoordinate) = (item, int(coordinate))
            else: #the coordinate of a body counts over its nodes, as mbs.CreateCoordinateConstraint
                objectDict = mbs.GetObject(item)
                (node, nodeCoordinate) = (None, int(coordinate))
                for bodyNode in ([objectDict['nodeNumber']] if 'nodeNumber' in objectDict else objectDict['nodeNumbers']):
                    numberOfCoordinates = len(mbs.systemData.GetNodeLTGODE2(bodyNode))
                    if nodeCoordinate < numberOfCoordinates:
                        node = bodyNode
                        break
                    nodeCoordinate -= numberOfCoordinates
                if node is None:
                    raise ValueError('UserItemCreator.CreateCoordinateConstraint: the body ' + str(item) + ' has no coordinate '
                                     + str(coordinate))
            if node not in nodes:
                nodes.append(node)
            offsetOfNode = sum(len(mbs.systemData.GetNodeLTGODE2(n)) for n in nodes[:nodes.index(node)])
            columns.append((offsetOfNode + nodeCoordinate, factor))
            if includeReference: #the equation in the total coordinates: the reference values into the constant
                c0 -= factor*float(mbs.GetNode(node)['referenceCoordinates'][nodeCoordinate])
        if not nodes:
            raise ValueError('UserItemCreator.CreateCoordinateConstraint: one of the two items must be a body or a node, but '
                             'received: ' + str(itemNumbers))
        n = sum(len(mbs.systemData.GetNodeLTGODE2(node)) for node in nodes)
        constraintMatrix = np.zeros((1, n))
        for (column, factor) in columns:
            constraintMatrix[0, column] += factor
        multiplier = mbs.AddNode(eii.NodeGenericAE(referenceCoordinates=[0], initialCoordinates=[0], numberOfAECoordinates=1))
        constraint = mbs.AddObject(eii.ObjectGenericDAE(name=name, nodeNumbersODE2=nodes, nodeNumbersAE=[multiplier],
                                                        constraintMatrixODE2=constraintMatrix, constraintVector=[c0]))
        self.items['coordinateConstraint'].append(constraint)
        return constraint

    def CreateRevoluteJoint(self, name='', itemNumbers=[None, None], position=[], axis=[], useGlobalFrame=True, show=True,
                            axisRadius=0.1, axisLength=0.4, color=exudyn.graphics.color.default, bodyNumbers=None,
                            bodyOrNodeList=None):
        """a revolute joint as ObjectJointUser on the rigid markers of mbs.CreateRevoluteJoint, with its arguments and
        the equations of ObjectJointRevoluteZ; drawn as a cylinder along the axis (#3147)

        Args:
            name: as in mbs.CreateRevoluteJoint
            itemNumbers: as in mbs.CreateRevoluteJoint, two bodies or the ground
            position: as in mbs.CreateRevoluteJoint
            axis: as in mbs.CreateRevoluteJoint
            useGlobalFrame: as in mbs.CreateRevoluteJoint
            show: as in mbs.CreateRevoluteJoint
            axisRadius: the radius of the cylinder of the drawing
            axisLength: the length of the cylinder of the drawing
            color: the color of the drawing; the default: grey
            bodyNumbers: refused if given: use itemNumbers
            bodyOrNodeList: refused if given: use itemNumbers

        Returns:
            the ObjectIndex of the joint
        """
        _Refuse('CreateRevoluteJoint', 'bodyNumbers', bodyNumbers, None, 'use itemNumbers')
        _Refuse('CreateRevoluteJoint', 'bodyOrNodeList', bodyOrNodeList, None, 'use itemNumbers')
        markers = self._JointMarkers('CreateRevoluteJoint', name, itemNumbers, position, axis, useGlobalFrame, show, 'z')
        return self._Joint('revoluteJoint', name, markers, [], show, _JointGraphics(markers, 'revolute', axisRadius, axisLength, color))

    def CreatePrismaticJoint(self, name='', itemNumbers=[None, None], position=[], axis=[], useGlobalFrame=True, show=True,
                             axisRadius=0.1, axisLength=0.4, color=exudyn.graphics.color.default, bodyNumbers=None,
                             bodyOrNodeList=None):
        """a prismatic joint as ObjectJointUser on the rigid markers of mbs.CreatePrismaticJoint, with its arguments and
        the equations of ObjectJointPrismaticX; drawn as a cylinder along the axis (#3147)

        Args:
            name: as in mbs.CreatePrismaticJoint
            itemNumbers: as in mbs.CreatePrismaticJoint, two bodies or the ground
            position: as in mbs.CreatePrismaticJoint
            axis: as in mbs.CreatePrismaticJoint
            useGlobalFrame: as in mbs.CreatePrismaticJoint
            show: as in mbs.CreatePrismaticJoint
            axisRadius: the radius of the cylinder of the drawing
            axisLength: the length of the cylinder of the drawing
            color: the color of the drawing; the default: grey
            bodyNumbers: refused if given: use itemNumbers
            bodyOrNodeList: refused if given: use itemNumbers

        Returns:
            the ObjectIndex of the joint
        """
        _Refuse('CreatePrismaticJoint', 'bodyNumbers', bodyNumbers, None, 'use itemNumbers')
        _Refuse('CreatePrismaticJoint', 'bodyOrNodeList', bodyOrNodeList, None, 'use itemNumbers')
        markers = self._JointMarkers('CreatePrismaticJoint', name, itemNumbers, position, axis, useGlobalFrame, show, 'x')
        return self._Joint('prismaticJoint', name, markers, [], show, _JointGraphics(markers, 'prismatic', axisRadius, axisLength, color))

    def CreateSphericalJoint(self, name='', itemNumbers=[None, None], position=[], constrainedAxes=[1, 1, 1],
                             useGlobalFrame=True, show=True, jointRadius=0.1, color=exudyn.graphics.color.default,
                             bodyNumbers=None, bodyOrNodeList=None):
        """a spherical joint as ObjectJointUser on rigid markers at the points of mbs.CreateSphericalJoint, with its
        arguments and the equations of ObjectJointSpherical; drawn as a sphere (#3147)

        Args:
            name: as in mbs.CreateSphericalJoint
            itemNumbers: as in mbs.CreateSphericalJoint, two bodies or the ground
            position: as in mbs.CreateSphericalJoint
            constrainedAxes: must be [1, 1, 1]: an axis left free has the equation lambda = 0, which ObjectJointUser
                does not give
            useGlobalFrame: as in mbs.CreateSphericalJoint
            show: as in mbs.CreateSphericalJoint
            jointRadius: the radius of the sphere of the drawing
            color: the color of the drawing; the default: grey
            bodyNumbers: refused if given: use itemNumbers
            bodyOrNodeList: refused if given: use itemNumbers

        Returns:
            the ObjectIndex of the joint
        """
        _Refuse('CreateSphericalJoint', 'constrainedAxes', list(constrainedAxes), [1, 1, 1],
                'an axis left free has the equation lambda = 0, which ObjectJointUser does not give')
        _Refuse('CreateSphericalJoint', 'bodyNumbers', bodyNumbers, None, 'use itemNumbers')
        _Refuse('CreateSphericalJoint', 'bodyOrNodeList', bodyOrNodeList, None, 'use itemNumbers')
        markers = self._JointMarkers('CreateSphericalJoint', name, itemNumbers, position, None, useGlobalFrame, show, None)
        return self._Joint('sphericalJoint', name, markers, [], show, _JointGraphics(markers, 'spherical', jointRadius, 0., color))

    def CreateDistanceConstraint(self, name='', itemNumbers=[None, None], localPosition0=[0., 0., 0.],
                                 localPosition1=[0., 0., 0.], distance=None, show=True, drawSize=-1.,
                                 color=exudyn.graphics.color.default, bodyNumbers=None, bodyOrNodeList=None):
        """a distance constraint as ObjectJointUser on rigid markers of the two bodies, with the arguments of
        mbs.CreateDistanceConstraint and the equation of ObjectConnectorDistance; on a user mass point the marker is on
        the body; drawn as a line between the markers (#3147)

        Args:
            name: as in mbs.CreateDistanceConstraint
            itemNumbers: two bodies or the ground; a node or a marker is refused
            localPosition0: as in mbs.CreateDistanceConstraint
            localPosition1: as in mbs.CreateDistanceConstraint
            distance: as in mbs.CreateDistanceConstraint; None: the distance in the reference configuration; 0 is
                refused (mbs.CreateDistanceConstraint creates a spherical joint then)
            show: as in mbs.CreateDistanceConstraint
            drawSize: a line if not positive, else a cylinder of diameter drawSize
            color: the color of the drawing; the default: grey
            bodyNumbers: refused if given: use itemNumbers
            bodyOrNodeList: refused if given: use itemNumbers

        Returns:
            the ObjectIndex of the constraint
        """
        _Refuse('CreateDistanceConstraint', 'bodyNumbers', bodyNumbers, None, 'use itemNumbers')
        _Refuse('CreateDistanceConstraint', 'bodyOrNodeList', bodyOrNodeList, None, 'use itemNumbers')
        if not all(isinstance(item, exudyn.ObjectIndex) for item in itemNumbers):
            raise ValueError('UserItemCreator.CreateDistanceConstraint: the argument itemNumbers must be two bodies or the '
                             'ground; a node or a marker is not supported by the user item')
        [marker0, marker1, p0, p1, A0, A1] = GetMarkersPosRot(self.mbs, name, itemNumbers, localPosition0, localPosition1,
                                                              getPosition=distance is None, useRigidMarker=True)
        if distance is None:
            distance = cMath.norm(np.array(p1) - p0)
        if distance == 0:
            raise ValueError('UserItemCreator.CreateDistanceConstraint: the distance 0 is not supported by the user item '
                             '(use CreateSphericalJoint)')
        return self._Joint('distanceConstraint', name, [marker0, marker1], [distance], show,
                           _JointGraphics([marker0, marker1], 'distance', drawSize, 0., color))

    def CreateForce(self, name='', itemNumber=None, loadVector=[0., 0., 0.], localPosition=[0., 0., 0.], bodyFixed=False,
                    loadVectorUserFunction=0, show=True, bodyNumber=None):
        """the built-in LoadForceVector of mbs.CreateForce, with its arguments: on a user mass point, its marker acts by
        the Jacobian of the user body

        Args:
            name: as in mbs.CreateForce
            itemNumber: as in mbs.CreateForce
            loadVector: as in mbs.CreateForce
            localPosition: as in mbs.CreateForce
            bodyFixed: as in mbs.CreateForce
            loadVectorUserFunction: as in mbs.CreateForce; a Python function is recorded by Record
            show: as in mbs.CreateForce
            bodyNumber: refused if given: use itemNumber

        Returns:
            the LoadIndex of the force
        """
        _Refuse('CreateForce', 'bodyNumber', bodyNumber, None, 'use itemNumber')
        load = self.mbs.CreateForce(name=name, itemNumber=itemNumber, loadVector=loadVector, localPosition=localPosition,
                                    bodyFixed=bodyFixed, loadVectorUserFunction=loadVectorUserFunction, show=show)
        if callable(loadVectorUserFunction):
            self.loads.append((load, loadVectorUserFunction, 'LoadForceVector'))
        return load

    def CreateTorque(self, name='', itemNumber=None, loadVector=[0., 0., 0.], localPosition=[0., 0., 0.], bodyFixed=False,
                     loadVectorUserFunction=0, show=True, bodyNumber=None):
        """the built-in LoadTorqueVector of mbs.CreateTorque, with its arguments: on a user rigid body, its marker acts by
        the rotation Jacobian of the user body (#3193)

        Args:
            name: as in mbs.CreateTorque
            itemNumber: as in mbs.CreateTorque
            loadVector: as in mbs.CreateTorque
            localPosition: as in mbs.CreateTorque
            bodyFixed: as in mbs.CreateTorque
            loadVectorUserFunction: as in mbs.CreateTorque; a Python function is recorded by Record
            show: as in mbs.CreateTorque
            bodyNumber: refused if given: use itemNumber

        Returns:
            the LoadIndex of the torque
        """
        _Refuse('CreateTorque', 'bodyNumber', bodyNumber, None, 'use itemNumber')
        load = self.mbs.CreateTorque(name=name, itemNumber=itemNumber, loadVector=loadVector, localPosition=localPosition,
                                     bodyFixed=bodyFixed, loadVectorUserFunction=loadVectorUserFunction, show=show)
        if callable(loadVectorUserFunction):
            self.loads.append((load, loadVectorUserFunction, 'LoadTorqueVector'))
        return load

    def Record(self):
        """replace the Python user functions of the items by symbolic ones: each user function of a kind recorded once,
        with the first item of the kind, and set in every item of the kind - one recording per kind and variant, as
        SymbolicSystem groups them -; the loadVectorUserFunction of each force of CreateForce recorded on its own. The
        recordings are kept in recordings, as long as this object lives"""
        mbs = self.mbs
        for (kind, items) in self.items.items():
            variants = {} #the items of each variant, by the identity of its functions (#3203)
            for item in items:
                functions = self._variants.get(int(item), _kindFunctions[kind])
                variants.setdefault(id(functions), (functions, []))[1].append(item)
            for (functions, variantItems) in variants.values():
                for (functionName, function) in functions:
                    recording = CreateSymbolicUserFunction(mbs, function, functionName, variantItems[0])
                    self.recordings.append(recording)
                    for item in variantItems:
                        mbs.SetObjectParameter(item, functionName, recording)
        for (load, function, loadType) in self.loads:
            recording = CreateSymbolicUserFunction(mbs, function, 'loadVectorUserFunction', itemTypeName=loadType)
            self.recordings.append(recording)
            mbs.SetLoadParameter(load, 'loadVectorUserFunction', recording)
        self.loads = []
