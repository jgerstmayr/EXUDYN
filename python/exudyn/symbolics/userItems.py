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
#           CreateTorsionalSpringDamper, CreateRevoluteJoint, CreatePrismaticJoint, CreateSphericalJoint,
#           CreateDistanceConstraint (ObjectJointUser, #3140), CreateForce. An argument a user item does not support
#           raises, naming it. A parameter of a mass point or a spring-damper may be a named exudyn.symbolic.Real
#           (mass=m1): the item gets its value, the creator keeps the variable in parameterVariables, for
#           SymbolicSystem(mbs, parameters=creator.parameterVariables) (#3152).
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

#the spring-damper, ObjectConnectorUser on two position markers, parameters [stiffness, damping, referenceLength, force,
#velocityOffset]: as ObjectConnectorSpringDamper, the force k (L - L0) + d (dL/dt - v0) + f along the axis, pulling
#marker 0 towards marker 1 if positive; with cMath, it computes numbers and is recorded
def _UFspringDamperForce(mbs, t, itemNumber, position0, velocity0, rotationMatrix0, angularVelocity0,
                        position1, velocity1, rotationMatrix1, angularVelocity1, dataCoordinates, parameters):
    delta = position1 - position0
    length = cMath.sqrt(delta @ delta)
    axis = (1/length)*delta
    force = (parameters[0]*(length - parameters[2]) + parameters[1]*(axis @ (velocity1 - velocity0) - parameters[4])
             + parameters[3])
    return np.hstack([force*axis, -force*axis])

#the torsional spring-damper, ObjectConnectorUser on two rigid markers, parameters [stiffness, damping, offset,
#velocityOffset, torque]: as ObjectConnectorTorsionalSpringDamper, the torque k (phi - phi0) + d (omega - omega0) + tau0
#about the z-axis of marker 0, phi the z-angle of the relative rotation A0^T A1 as Tait-Bryan angle, omega the z-component
#of the relative angular velocity in the frame of marker 0; no forces. As the forces of the spring-damper, the torques act
#on the markers: a positive torque turns marker 0 about the axis and marker 1 back
def _UFtorsionalSpringDamperForce(mbs, t, itemNumber, position0, velocity0, rotationMatrix0, angularVelocity0,
                                  position1, velocity1, rotationMatrix1, angularVelocity1, dataCoordinates, parameters):
    F = _Functions(rotationMatrix0)
    A0 = rotationMatrix0
    relative = A0.T @ rotationMatrix1
    angle = F.atan2(-relative[0, 1], relative[0, 0])
    omega = (A0.T @ (angularVelocity1 - angularVelocity0))[2]
    torque = parameters[0]*(angle - parameters[2]) + parameters[1]*(omega - parameters[3]) + parameters[4]
    torque0 = [torque*A0[i, 2] for i in range(3)]
    return _Vector([0, 0, 0] + torque0 + [0, 0, 0] + [-x for x in torque0], F)

#the joints, ObjectJointUser on two rigid markers whose frames are the joint frame: the equations of the built-in joint
#at position level, and their derivative by the translations and global rotation increments [p0, theta0, p1, theta1] of
#the markers, R(theta) = (I + skew(theta)) R. For the scalar product u.v of an axis u of marker 0 and an axis v of
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

#the revolute joint, as ObjectJointRevoluteZ: the relative position, and the z-axis of marker 0 perpendicular to the
#x- and y-axes of marker 1; no parameters
def _UFrevoluteJointConstraint(mbs, t, itemNumber, position0, rotationMatrix0, position1, rotationMatrix1, parameters):
    F = _Functions(rotationMatrix0)
    z0 = _Column(rotationMatrix0, 2)
    return _Vector([position1[i] - position0[i] for i in range(3)]
                   + [_Dot(z0, _Column(rotationMatrix1, 0)), _Dot(z0, _Column(rotationMatrix1, 1))], F)

def _UFrevoluteJointJacobian(mbs, t, itemNumber, position0, rotationMatrix0, position1, rotationMatrix1, parameters):
    F = _Functions(rotationMatrix0)
    z0 = _Column(rotationMatrix0, 2)
    rows = [[_Unit(k, -1), None, _Unit(k), None] for k in range(3)]
    for j in [0, 1]:
        v = _Column(rotationMatrix1, j)
        rows.append([None, _Cross(z0, v), None, _Cross(v, z0)])
    return _JacobianRows(rows, F)

#the prismatic joint, as ObjectJointPrismaticX: the relative position along the y- and z-axes of marker 0, and the
#axes z0.y1, z0.x1, x0.y1; no parameters
def _UFprismaticJointConstraint(mbs, t, itemNumber, position0, rotationMatrix0, position1, rotationMatrix1, parameters):
    F = _Functions(rotationMatrix0)
    d = [position1[i] - position0[i] for i in range(3)]
    (x0, y0, z0) = (_Column(rotationMatrix0, 0), _Column(rotationMatrix0, 1), _Column(rotationMatrix0, 2))
    (x1, y1) = (_Column(rotationMatrix1, 0), _Column(rotationMatrix1, 1))
    return _Vector([_Dot(y0, d), _Dot(z0, d), _Dot(z0, y1), _Dot(z0, x1), _Dot(x0, y1)], F)

def _UFprismaticJointJacobian(mbs, t, itemNumber, position0, rotationMatrix0, position1, rotationMatrix1, parameters):
    F = _Functions(rotationMatrix0)
    d = [position1[i] - position0[i] for i in range(3)]
    (x0, y0, z0) = (_Column(rotationMatrix0, 0), _Column(rotationMatrix0, 1), _Column(rotationMatrix0, 2))
    (x1, y1) = (_Column(rotationMatrix1, 0), _Column(rotationMatrix1, 1))
    rows = [[[-x for x in u], _Cross(u, d), u, None] for u in [y0, z0]]
    rows += [[None, _Cross(u, v), None, _Cross(v, u)] for (u, v) in [(z0, y1), (z0, x1), (x0, y1)]]
    return _JacobianRows(rows, F)

#the spherical joint, as ObjectJointSpherical with all axes constrained: the relative position; no parameters
def _UFsphericalJointConstraint(mbs, t, itemNumber, position0, rotationMatrix0, position1, rotationMatrix1, parameters):
    return _Vector([position1[i] - position0[i] for i in range(3)], _Functions(rotationMatrix0))

def _UFsphericalJointJacobian(mbs, t, itemNumber, position0, rotationMatrix0, position1, rotationMatrix1, parameters):
    return _JacobianRows([[_Unit(k, -1), None, _Unit(k), None] for k in range(3)], _Functions(rotationMatrix0))

#the distance constraint, as ObjectConnectorDistance: |p1 - p0| - distance; parameters [distance]
def _UFdistanceConstraint(mbs, t, itemNumber, position0, rotationMatrix0, position1, rotationMatrix1, parameters):
    F = _Functions(rotationMatrix0)
    d = [position1[i] - position0[i] for i in range(3)]
    return _Vector([F.sqrt(_Dot(d, d)) - parameters[0]], F)

def _UFdistanceConstraintJacobian(mbs, t, itemNumber, position0, rotationMatrix0, position1, rotationMatrix1, parameters):
    F = _Functions(rotationMatrix0)
    d = [position1[i] - position0[i] for i in range(3)]
    axis = [x*(1/F.sqrt(_Dot(d, d))) for x in d]
    return _JacobianRows([[[-x for x in axis], None, axis, None]], F)

#the user functions of each kind of item, which Record records once per kind
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
            self.loads.append((load, loadVectorUserFunction))
        return load

    def Record(self):
        """replace the Python user functions of the items by symbolic ones: each user function of a kind recorded once,
        with the first item of the kind, and set in every item of the kind - one recording per kind, as SymbolicSystem
        groups them -; the loadVectorUserFunction of each force of CreateForce recorded on its own. The recordings are
        kept in recordings, as long as this object lives"""
        mbs = self.mbs
        for (kind, functions) in _kindFunctions.items():
            if not self.items[kind]:
                continue
            for (functionName, function) in functions:
                recording = CreateSymbolicUserFunction(mbs, function, functionName, self.items[kind][0])
                self.recordings.append(recording)
                for item in self.items[kind]:
                    mbs.SetObjectParameter(item, functionName, recording)
        for (load, function) in self.loads:
            recording = CreateSymbolicUserFunction(mbs, function, 'loadVectorUserFunction', itemTypeName='LoadForceVector')
            self.recordings.append(recording)
            mbs.SetLoadParameter(load, 'loadVectorUserFunction', recording)
        self.loads = []
