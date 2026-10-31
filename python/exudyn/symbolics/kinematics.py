#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Rotation and kinematics functions for symbolic user functions (#3069): a selection of
#           exudyn.cUtilities / exudyn.rigidBodyUtilities with the same names and arguments, plus a last argument
#           functions, the module whose sin, cos, sqrt, ... are used. With functions=exudyn.symbolic (the default)
#           the arguments may be symbolic and the result is an exudyn.symbolic.Matrix or Vector, which a symbolic
#           user function records; with functions=numpy, math or exudyn.cMath they are numbers and the result is a
#           numpy array, equal to the one of cUtilities. So a user function is written once for both; for numbers
#           alone, exudyn.cUtilities is faster.
#           Arguments are sequences indexed by component: lists, numpy arrays, symbolic vectors or their slices.
#           The classes HT and Twist hold a frame and its velocities with symbolic parts, with the methods of
#           exudyn.HT and exudyn.Twist; PointVelocity, LocalTwist and RelativeTwist take either (#3182).
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-23 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import exudyn as exu
import exudyn.cUtilities #the twist functions for numbers (#3174)

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'Skew', 'Cross', 'Inertia6D2InertiaTensor', 'RotationMatrixX', 'RotationMatrixY', 'RotationMatrixZ',
    'RotationMatrix2D', 'RotXYZ2RotationMatrix', 'RotXYZ2G', 'RotXYZ2G_t', 'RotXYZ2GLocal', 'RotXYZ2GLocal_t',
    'AngularVelocity2RotXYZ_t', 'RotXYZ2EulerParameters', 'EulerParameters2RotationMatrix', 'EulerParameters2G',
    'EulerParameters2GLocal', 'AngularVelocity2EulerParameters_t', 'RotationVector2RotationMatrix',
    'RotationVector2G', 'RotationVector2GLocal', 'HT', 'Twist', 'PointVelocity', 'LocalTwist', 'RelativeTwist',
]

#below this length, RotationVector2RotationMatrix takes the limits of its factors sin(a)/a and 2 sin(a/2)^2/a^2
_smallAngle = 1e-12

def _Matrix(rows, functions):
    """the result matrix: a symbolic matrix for exudyn.symbolic, a numpy array otherwise"""
    if functions is exu.symbolic:
        return exu.symbolic.Matrix(rows)
    return np.array(rows, dtype=float)

def _Vector(components, functions):
    """the result vector: a symbolic vector for exudyn.symbolic, a numpy array otherwise"""
    if functions is exu.symbolic:
        return exu.symbolic.Vector(components)
    return np.array(components, dtype=float)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#            BASICS
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def Skew(vector, functions=exu.symbolic):
    """skew symmetric 3 x 3 matrix of a 3D vector, Skew(a) @ b = a x b; equals exudyn.cUtilities.Skew for a 3D vector

    Args:
        vector: 3D vector
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3 x 3 matrix, symbolic for exudyn.symbolic, else as np.array
    """
    v = vector
    return _Matrix([[0, -v[2], v[1]],
                    [v[2], 0, -v[0]],
                    [-v[1], v[0], 0]], functions)

def Cross(vector0, vector1, functions=exu.symbolic):
    """cross product of two 3D vectors, as numpy.cross

    Args:
        vector0: first 3D vector
        vector1: second 3D vector
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3D vector vector0 x vector1, symbolic for exudyn.symbolic, else as np.array
    """
    a, b = vector0, vector1
    return _Vector([a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]], functions)

def Inertia6D2InertiaTensor(inertia6D, functions=exu.symbolic):
    """3 x 3 inertia tensor of its 6 components [J00, J11, J22, J12, J02, J01]; equals
    exudyn.rigidBodyUtilities.Inertia6D2InertiaTensor

    Args:
        inertia6D: the 6 components of the inertia tensor
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3 x 3 inertia tensor, symbolic for exudyn.symbolic, else as np.array
    """
    J = inertia6D
    return _Matrix([[J[0], J[5], J[4]],
                    [J[5], J[1], J[3]],
                    [J[4], J[3], J[2]]], functions)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#            ROTATIONS ABOUT AXES
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def RotationMatrixX(angleRad, functions=exu.symbolic):
    """rotation matrix about the x-axis; equals exudyn.cUtilities.RotationMatrixX

    Args:
        angleRad: angle in radiant
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3 x 3 rotation matrix, symbolic for exudyn.symbolic, else as np.array
    """
    c, s = functions.cos(angleRad), functions.sin(angleRad)
    return _Matrix([[1, 0, 0],
                    [0, c, -s],
                    [0, s, c]], functions)

def RotationMatrixY(angleRad, functions=exu.symbolic):
    """rotation matrix about the y-axis; equals exudyn.cUtilities.RotationMatrixY

    Args:
        angleRad: angle in radiant
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3 x 3 rotation matrix, symbolic for exudyn.symbolic, else as np.array
    """
    c, s = functions.cos(angleRad), functions.sin(angleRad)
    return _Matrix([[c, 0, s],
                    [0, 1, 0],
                    [-s, 0, c]], functions)

def RotationMatrixZ(angleRad, functions=exu.symbolic):
    """rotation matrix about the z-axis; equals exudyn.cUtilities.RotationMatrixZ

    Args:
        angleRad: angle in radiant
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3 x 3 rotation matrix, symbolic for exudyn.symbolic, else as np.array
    """
    c, s = functions.cos(angleRad), functions.sin(angleRad)
    return _Matrix([[c, -s, 0],
                    [s, c, 0],
                    [0, 0, 1]], functions)

def RotationMatrix2D(angleRad, functions=exu.symbolic):
    """2D rotation matrix; equals exudyn.cUtilities.RotationMatrix2D

    Args:
        angleRad: angle about the out-of-plane axis in radiant
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        2 x 2 rotation matrix, symbolic for exudyn.symbolic, else as np.array
    """
    c, s = functions.cos(angleRad), functions.sin(angleRad)
    return _Matrix([[c, -s],
                    [s, c]], functions)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#            TAIT-BRYAN ANGLES XYZ
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def RotXYZ2RotationMatrix(rot, functions=exu.symbolic):
    """rotation matrix Rx(rot[0]) Ry(rot[1]) Rz(rot[2]) of the Tait-Bryan angles; equals
    exudyn.cUtilities.RotXYZ2RotationMatrix

    Args:
        rot: 3 Tait-Bryan angles [X, Y, Z] in radiant
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3 x 3 rotation matrix, symbolic for exudyn.symbolic, else as np.array
    """
    c0, s0 = functions.cos(rot[0]), functions.sin(rot[0])
    c1, s1 = functions.cos(rot[1]), functions.sin(rot[1])
    c2, s2 = functions.cos(rot[2]), functions.sin(rot[2])
    return _Matrix([[c1*c2, -c1*s2, s1],
                    [s0*s1*c2 + c0*s2, -s0*s1*s2 + c0*c2, -s0*c1],
                    [-c0*s1*c2 + s0*s2, c0*s1*s2 + s0*c2, c0*c1]], functions)

def RotXYZ2G(rot, functions=exu.symbolic):
    """G matrix of the Tait-Bryan angles, omega = G @ rot_t with the global angular velocity omega; equals
    exudyn.cUtilities.RotXYZ2G

    Args:
        rot: 3 Tait-Bryan angles [X, Y, Z] in radiant
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3 x 3 matrix G, symbolic for exudyn.symbolic, else as np.array
    """
    c0, s0 = functions.cos(rot[0]), functions.sin(rot[0])
    c1, s1 = functions.cos(rot[1]), functions.sin(rot[1])
    return _Matrix([[1, 0, s1],
                    [0, c0, -c1*s0],
                    [0, s0, c0*c1]], functions)

def RotXYZ2G_t(rot, rot_t, functions=exu.symbolic):
    """time derivative of the G matrix of the Tait-Bryan angles; equals exudyn.cUtilities.RotXYZ2G_t

    Args:
        rot: 3 Tait-Bryan angles [X, Y, Z] in radiant
        rot_t: their time derivatives
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3 x 3 matrix G_t, symbolic for exudyn.symbolic, else as np.array
    """
    c0, s0 = functions.cos(rot[0]), functions.sin(rot[0])
    c1, s1 = functions.cos(rot[1]), functions.sin(rot[1])
    return _Matrix([[0, 0, rot_t[1]*c1],
                    [0, -rot_t[0]*s0, rot_t[1]*s0*s1 - rot_t[0]*c0*c1],
                    [0, rot_t[0]*c0, -rot_t[0]*c1*s0 - rot_t[1]*c0*s1]], functions)

def RotXYZ2GLocal(rot, functions=exu.symbolic):
    """local G matrix of the Tait-Bryan angles, omegaLocal = GLocal @ rot_t with the angular velocity in body
    coordinates; equals exudyn.cUtilities.RotXYZ2GLocal

    Args:
        rot: 3 Tait-Bryan angles [X, Y, Z] in radiant
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3 x 3 matrix GLocal, symbolic for exudyn.symbolic, else as np.array
    """
    c1, s1 = functions.cos(rot[1]), functions.sin(rot[1])
    c2, s2 = functions.cos(rot[2]), functions.sin(rot[2])
    return _Matrix([[c1*c2, s2, 0],
                    [-c1*s2, c2, 0],
                    [s1, 0, 1]], functions)

def RotXYZ2GLocal_t(rot, rot_t, functions=exu.symbolic):
    """time derivative of the local G matrix of the Tait-Bryan angles; equals exudyn.cUtilities.RotXYZ2GLocal_t

    Args:
        rot: 3 Tait-Bryan angles [X, Y, Z] in radiant
        rot_t: their time derivatives
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3 x 3 matrix GLocal_t, symbolic for exudyn.symbolic, else as np.array
    """
    c1, s1 = functions.cos(rot[1]), functions.sin(rot[1])
    c2, s2 = functions.cos(rot[2]), functions.sin(rot[2])
    return _Matrix([[-rot_t[2]*c1*s2 - rot_t[1]*c2*s1, rot_t[2]*c2, 0],
                    [rot_t[1]*s2*s1 - rot_t[2]*c2*c1, -rot_t[2]*s2, 0],
                    [rot_t[1]*c1, 0, 0]], functions)

def AngularVelocity2RotXYZ_t(angularVelocity, rotation, functions=exu.symbolic):
    """time derivatives of the Tait-Bryan angles of the global angular velocity, G^-1 @ angularVelocity; equals
    exudyn.cUtilities.AngularVelocity2RotXYZ_t; singular at cos(rotation[1]) = 0, where it divides by zero

    Args:
        angularVelocity: 3D global angular velocity
        rotation: 3 Tait-Bryan angles [X, Y, Z] in radiant
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3D vector of the time derivatives of the angles, symbolic for exudyn.symbolic, else as np.array
    """
    w = angularVelocity
    c0, s0 = functions.cos(rotation[0]), functions.sin(rotation[0])
    c1, s1 = functions.cos(rotation[1]), functions.sin(rotation[1])
    return _Vector([w[0] + (s0*s1*w[1] - c0*s1*w[2])/c1,
                    c0*w[1] + s0*w[2],
                    (-s0*w[1] + c0*w[2])/c1], functions)

def RotXYZ2EulerParameters(alpha, functions=exu.symbolic):
    """the 4 Euler parameters of the Tait-Bryan angles, see [Henderson1977], entry 0 the scalar part; equals
    exudyn.cUtilities.RotXYZ2EulerParameters

    Args:
        alpha: 3 Tait-Bryan angles [X, Y, Z] in radiant
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        4D vector of Euler parameters, symbolic for exudyn.symbolic, else as np.array
    """
    c0, s0 = functions.cos(0.5*alpha[0]), functions.sin(0.5*alpha[0])
    c1, s1 = functions.cos(0.5*alpha[1]), functions.sin(0.5*alpha[1])
    c2, s2 = functions.cos(0.5*alpha[2]), functions.sin(0.5*alpha[2])
    return _Vector([-s0*s1*s2 + c0*c1*c2,
                    s0*c1*c2 + s1*s2*c0,
                    -s0*s2*c1 + s1*c0*c2,
                    s0*s1*c2 + s2*c0*c1], functions)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#            EULER PARAMETERS
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def EulerParameters2RotationMatrix(eulerParameters, functions=exu.symbolic):
    """rotation matrix of the 4 Euler parameters; equals exudyn.cUtilities.EulerParameters2RotationMatrix

    Args:
        eulerParameters: 4 Euler parameters, entry 0 the scalar part
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3 x 3 rotation matrix, symbolic for exudyn.symbolic, else as np.array
    """
    p0, p1, p2, p3 = eulerParameters[0], eulerParameters[1], eulerParameters[2], eulerParameters[3]
    return _Matrix([[-2.*p3*p3 - 2.*p2*p2 + 1., -2.*p3*p0 + 2.*p2*p1, 2.*p3*p1 + 2.*p2*p0],
                    [2.*p3*p0 + 2.*p2*p1, -2.*p3*p3 - 2.*p1*p1 + 1., 2.*p3*p2 - 2.*p1*p0],
                    [-2.*p2*p0 + 2.*p3*p1, 2.*p3*p2 + 2.*p1*p0, -2.*p2*p2 - 2.*p1*p1 + 1.]], functions)

def EulerParameters2G(eulerParameters, functions=exu.symbolic):
    """G matrix of the Euler parameters, omega = G @ ep_t with the global angular velocity omega; equals
    exudyn.cUtilities.EulerParameters2G

    Args:
        eulerParameters: 4 Euler parameters, entry 0 the scalar part
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3 x 4 matrix G, symbolic for exudyn.symbolic, else as np.array
    """
    p0, p1, p2, p3 = eulerParameters[0], eulerParameters[1], eulerParameters[2], eulerParameters[3]
    return _Matrix([[-2.*p1, 2.*p0, -2.*p3, 2.*p2],
                    [-2.*p2, 2.*p3, 2.*p0, -2.*p1],
                    [-2.*p3, -2.*p2, 2.*p1, 2.*p0]], functions)

def EulerParameters2GLocal(eulerParameters, functions=exu.symbolic):
    """local G matrix of the Euler parameters, omegaLocal = GLocal @ ep_t with the angular velocity in body
    coordinates; equals exudyn.cUtilities.EulerParameters2GLocal

    Args:
        eulerParameters: 4 Euler parameters, entry 0 the scalar part
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3 x 4 matrix GLocal, symbolic for exudyn.symbolic, else as np.array
    """
    p0, p1, p2, p3 = eulerParameters[0], eulerParameters[1], eulerParameters[2], eulerParameters[3]
    return _Matrix([[-2.*p1, 2.*p0, 2.*p3, -2.*p2],
                    [-2.*p2, -2.*p3, 2.*p0, 2.*p1],
                    [-2.*p3, 2.*p2, -2.*p1, 2.*p0]], functions)

def AngularVelocity2EulerParameters_t(angularVelocity, eulerParameters, functions=exu.symbolic):
    """time derivatives of the Euler parameters of the global angular velocity, 0.25 G^T @ angularVelocity for unit
    Euler parameters; equals exudyn.cUtilities.AngularVelocity2EulerParameters_t

    Args:
        angularVelocity: 3D global angular velocity
        eulerParameters: 4 Euler parameters, entry 0 the scalar part
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        4D vector of the time derivatives of the Euler parameters, symbolic for exudyn.symbolic, else as np.array
    """
    w = angularVelocity
    p0, p1, p2, p3 = eulerParameters[0], eulerParameters[1], eulerParameters[2], eulerParameters[3]
    return _Vector([0.5*(-p1*w[0] - p2*w[1] - p3*w[2]),
                    0.5*(p0*w[0] + p3*w[1] - p2*w[2]),
                    0.5*(-p3*w[0] + p0*w[1] + p1*w[2]),
                    0.5*(p2*w[0] - p1*w[1] + p0*w[2])], functions)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#            ROTATION VECTOR
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def RotationVector2RotationMatrix(rotationVector, functions=exu.symbolic):
    """rotation matrix of the rotation vector v with the angle a = |v|, I + sin(a)/a Skew(v) + 2 sin(a/2)^2/a^2
    Skew(v)^2, see appendix B in [Simo1988]; below a = 1e-12 the factors are their limits 1 and 1/2, for symbolic
    arguments chosen by exudyn.symbolic.IfThenElse; equals exudyn.cUtilities.RotationVector2RotationMatrix

    Args:
        rotationVector: 3D rotation vector
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3 x 3 rotation matrix, symbolic for exudyn.symbolic, else as np.array
    """
    v = rotationVector
    angle = functions.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    if isinstance(angle, exu.symbolic.Real):
        small = angle < _smallAngle
        sHalf = functions.sin(0.5*angle)
        f1 = exu.symbolic.IfThenElse(small, exu.symbolic.Real(1.), functions.sin(angle)/angle)
        f2 = exu.symbolic.IfThenElse(small, exu.symbolic.Real(0.5), 2.*sHalf*sHalf/(angle*angle))
    elif angle < _smallAngle:
        f1, f2 = 1., 0.5
    else:
        sHalf = functions.sin(0.5*angle)
        f1, f2 = functions.sin(angle)/angle, 2.*sHalf*sHalf/(angle*angle)
    #Skew(v)^2 = v v^T - (v^T v) I
    vv = v[0]*v[0] + v[1]*v[1] + v[2]*v[2]
    return _Matrix([[1 + f2*(v[0]*v[0] - vv), -f1*v[2] + f2*v[0]*v[1], f1*v[1] + f2*v[0]*v[2]],
                    [f1*v[2] + f2*v[1]*v[0], 1 + f2*(v[1]*v[1] - vv), -f1*v[0] + f2*v[1]*v[2]],
                    [-f1*v[1] + f2*v[2]*v[0], f1*v[0] + f2*v[2]*v[1], 1 + f2*(v[2]*v[2] - vv)]], functions)

def RotationVector2G(rotationVector, functions=exu.symbolic):
    """G matrix of the rotation vector as the Lie group node NodeRigidBodyRotVecLG uses it, whose velocity
    coordinates are the local angular velocity: the rotation matrix; equals exudyn.rigidBodyUtilities.RotationVector2G

    Args:
        rotationVector: 3D rotation vector
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3 x 3 matrix G, symbolic for exudyn.symbolic, else as np.array
    """
    return RotationVector2RotationMatrix(rotationVector, functions)

def RotationVector2GLocal(rotationVector, functions=exu.symbolic):
    """local G matrix of the rotation vector as the Lie group node NodeRigidBodyRotVecLG uses it, whose velocity
    coordinates are the local angular velocity: the unit matrix; equals exudyn.rigidBodyUtilities.RotationVector2GLocal

    Args:
        rotationVector: 3D rotation vector
        functions: exudyn.symbolic (symbolic or numbers), numpy, math or exudyn.cMath (numbers)

    Returns:
        3 x 3 unit matrix, symbolic for exudyn.symbolic, else as np.array
    """
    return _Matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]], functions)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#            HOMOGENEOUS TRANSFORMATION AND TWIST
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def _IsSymbolic(*values):
    return any(isinstance(value, (exu.symbolic.Matrix, exu.symbolic.Vector, exu.symbolic.Real)) for value in values)

def _AsSymbolic(value):
    """a constant matrix or vector of numbers as symbolic one, a symbolic one as it is"""
    if _IsSymbolic(value):
        return value
    array = np.asarray(value, dtype=float)
    return exu.symbolic.Matrix(array.tolist()) if array.ndim == 2 else exu.symbolic.Vector(array.tolist())

def _MatMul(matrix, operand):
    """matrix @ operand; a constant is made symbolic if the other one is symbolic"""
    if _IsSymbolic(matrix, operand):
        return _AsSymbolic(matrix) @ _AsSymbolic(operand)
    return np.asarray(matrix) @ np.asarray(operand)

def _Plus(vector0, vector1, sign=1.):
    """vector0 + sign*vector1; a constant is made symbolic if the other one is symbolic"""
    if _IsSymbolic(vector0, vector1):
        (vector0, vector1) = (_AsSymbolic(vector0), _AsSymbolic(vector1))
        return vector0 + vector1 if sign > 0 else vector0 - vector1
    return np.asarray(vector0) + sign*np.asarray(vector1)

class HT:
    """the homogeneous transformation of exudyn.HT for symbolic parts (#3174): a rotation matrix and a translation,
    each an exudyn.symbolic.Matrix or Vector, or numbers; its methods give the matrix and vector expressions at once,
    so that a recording holds matrix products. A symbolic user function of ObjectConnectorUser and ObjectJointUser gets
    one per marker while it is recorded, an exudyn.HT when it computes with numbers; both have the same methods. An
    exudyn.HT is taken as constant in the products, in either order.

    Args:
        rotation: 3 x 3 rotation matrix, None for the unit matrix; or an exudyn.HT or HT, copied
        translation: 3D translation, None for zero
    """
    def __init__(self, rotation=None, translation=None):
        if isinstance(rotation, (HT, exu.HT)):
            if translation is not None:
                raise ValueError('kinematics.HT: with an HT, translation must be None')
            (rotation, translation) = (rotation.rotation, rotation.translation)
        self.rotation = np.eye(3) if rotation is None else rotation
        self.translation = np.zeros(3) if translation is None else translation

    def __mul__(self, other):
        """H1*H2, the composition, an HT; H*v, the transformed point A v + p of a 3D vector"""
        if isinstance(other, (HT, exu.HT)):
            return HT(_MatMul(self.rotation, other.rotation),
                      _Plus(_MatMul(self.rotation, other.translation), self.translation))
        return _Plus(_MatMul(self.rotation, other), self.translation)

    def __rmul__(self, other):
        """exudyn.HT * HT, the composition with the exudyn.HT as constant"""
        if isinstance(other, exu.HT):
            return HT(other).__mul__(self)
        return NotImplemented

    def __matmul__(self, other):
        """H1@H2, the composition as H1*H2"""
        if isinstance(other, (HT, exu.HT)):
            return self.__mul__(other)
        return NotImplemented

    def __rmatmul__(self, other):
        """exudyn.HT @ HT, the composition as exudyn.HT * HT"""
        return self.__rmul__(other)

    def Inverse(self):
        """the inverse transformation [A^T, -A^T p], an HT"""
        rotationT = self.rotation.T
        return HT(rotationT, -_MatMul(rotationT, self.translation))

    def Relative(self, other):
        """the frame of other seen from this one, H.Inverse()*other = [A^T A2, A^T (p2 - p)], without the inverse"""
        rotationT = self.rotation.T
        return HT(_MatMul(rotationT, other.rotation),
                  _MatMul(rotationT, _Plus(other.translation, self.translation, -1.)))

    def __repr__(self):
        return 'kinematics.HT(rotation=' + str(self.rotation) + ', translation=' + str(self.translation) + ')'

class Twist:
    """the twist of exudyn.Twist for symbolic parts (#3174): the velocity of the origin of a frame and its angular
    velocity, both in global components, each an exudyn.symbolic.Vector or numbers. A symbolic forceUserFunction of
    ObjectConnectorUser gets one per marker while it is recorded, an exudyn.Twist when it computes with numbers.

    Args:
        velocity: 3D velocity of the origin, None for zero; or an exudyn.Twist or Twist, copied
        angularVelocity: 3D angular velocity, None for zero
    """
    def __init__(self, velocity=None, angularVelocity=None):
        if isinstance(velocity, (Twist, exu.Twist)):
            if angularVelocity is not None:
                raise ValueError('kinematics.Twist: with a Twist, angularVelocity must be None')
            (velocity, angularVelocity) = (velocity.velocity, velocity.angularVelocity)
        self.velocity = np.zeros(3) if velocity is None else velocity
        self.angularVelocity = np.zeros(3) if angularVelocity is None else angularVelocity

    def __repr__(self):
        return 'kinematics.Twist(velocity=' + str(self.velocity) + ', angularVelocity=' + str(self.angularVelocity) + ')'

def _Numeric(*values):
    """true if the frames and twists are those of exudyn, which exudyn.cUtilities computes"""
    return all(isinstance(value, (exu.HT, exu.Twist)) for value in values)

def _CrossOf(vector0, vector1):
    functions = exu.symbolic if _IsSymbolic(vector0, vector1) else np
    return Cross(vector0, vector1, functions)

def PointVelocity(HT, twist, localPoint):
    """the velocity of a point fixed in the frame HT, given by its position in the frame, v + omega x (A localPoint);
    for an exudyn.HT and an exudyn.Twist computed by exudyn.cUtilities.PointVelocity

    Args:
        HT: the frame, an exudyn.HT or kinematics.HT
        twist: the twist of the frame, an exudyn.Twist or kinematics.Twist
        localPoint: 3D position of the point in the frame

    Returns:
        3D velocity of the point, global components; symbolic for symbolic parts, else as np.array
    """
    if _Numeric(HT, twist):
        return exu.cUtilities.PointVelocity(HT, twist, localPoint)
    return _Plus(twist.velocity, _CrossOf(twist.angularVelocity, _MatMul(HT.rotation, localPoint)))

def LocalTwist(HT, twist):
    """the twist in the components of the frame HT, [A^T v, A^T omega]; for an exudyn.HT and an exudyn.Twist computed
    by exudyn.cUtilities.LocalTwist

    Args:
        HT: the frame, an exudyn.HT or kinematics.HT
        twist: the twist of the frame, an exudyn.Twist or kinematics.Twist

    Returns:
        the twist in local components, an exudyn.Twist for exudyn.HT and exudyn.Twist, else a kinematics.Twist
    """
    if _Numeric(HT, twist):
        return exu.cUtilities.LocalTwist(HT, twist)
    rotationT = HT.rotation.T
    return Twist(_MatMul(rotationT, twist.velocity), _MatMul(rotationT, twist.angularVelocity))

def RelativeTwist(HT0, twist0, HT1, twist1):
    """the velocity of frame 1 relative to frame 0 in the components of frame 0, the time derivative of
    HT0.Relative(HT1): [A0^T (v1 - v0 - omega0 x (p1 - p0)), A0^T (omega1 - omega0)]; for exudyn.HT and exudyn.Twist
    computed by exudyn.cUtilities.RelativeTwist

    Args:
        HT0: frame 0, an exudyn.HT or kinematics.HT
        twist0: the twist of frame 0, an exudyn.Twist or kinematics.Twist
        HT1: frame 1
        twist1: the twist of frame 1

    Returns:
        the relative twist, an exudyn.Twist for exudyn.HT and exudyn.Twist, else a kinematics.Twist
    """
    if _Numeric(HT0, twist0, HT1, twist1):
        return exu.cUtilities.RelativeTwist(HT0, twist0, HT1, twist1)
    rotationT = HT0.rotation.T
    distance = _Plus(HT1.translation, HT0.translation, -1.)
    velocity = _Plus(_Plus(twist1.velocity, twist0.velocity, -1.), _CrossOf(twist0.angularVelocity, distance), -1.)
    return Twist(_MatMul(rotationT, velocity),
                 _MatMul(rotationT, _Plus(twist1.angularVelocity, twist0.angularVelocity, -1.)))
