#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Advanced utility/mathematical functions for reference frames, rigid body kinematics
#           and dynamics. Useful Euler parameter and Tait-Bryan angle conversion functions
#           are included. A class for rigid body inertia creating and transformation is available.
#
# Author:   Johannes Gerstmayr, Stefan Holzinger (rotation vector and Tait-Bryan angles)
# Date:     2020-03-10 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#constants and fixed structures:
from exudyn.misc.docmeta import docmeta
import numpy as np #LoadSolutionFile
import exudyn.itemInterface as eii
import exudyn as exu 
import exudyn.graphics as graphics
from exudyn.advancedUtilities import ExpectedType, RaiseTypeError, IsValidBool, IsValidRealInt, IsVector, IsSquareMatrix, IsValidObjectIndex
from math import sin, cos #, sqrt, atan2

import copy

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'eulerParameters0', 'ComputeOrthonormalBasisVectors', 'ComputeOrthonormalBasis', 'GramSchmidt',
    'Skew', 'Skew2Vec', 'ComputeSkewMatrix', 'EulerParameters2G', 'EulerParameters2GLocal',
    'EulerParameters2RotationMatrix', 'RotationMatrix2EulerParameters',
    'AngularVelocity2EulerParameters_t', 'RotationVector2RotationMatrix',
    'RotationMatrix2RotationVector', 'ComputeRotationAxisFromRotationVector', 'RotationVector2G',
    'RotationVector2GLocal', 'RotXYZ2RotationMatrix', 'RotationMatrix2RotXYZ', 'RotXYZ2G',
    'RotXYZ2G_t', 'RotXYZ2GLocal', 'RotXYZ2GLocal_t', 'AngularVelocity2RotXYZ_t',
    'RotXYZ2EulerParameters', 'RotationMatrix2RotZYZ', 'RotationMatrixX', 'RotationMatrixY',
    'RotationMatrixZ', 'RotationMatrix2D', 'HomogeneousTransformation', 'HT', 'HTtranslate',
    'HTtranslateX', 'HTtranslateY', 'HTtranslateZ', 'HT0', 'HTrotateX', 'HTrotateY', 'HTrotateZ',
    'HT2translation', 'HT2rotationMatrix', 'InverseHT', 'RotationX2T66', 'RotationY2T66',
    'RotationZ2T66', 'Translation2T66', 'TranslationX2T66', 'TranslationY2T66', 'TranslationZ2T66',
    'T66toRotationTranslation', 'InverseT66toRotationTranslation', 'RotationTranslation2T66',
    'RotationTranslation2T66Inverse', 'T66Inverse', 'T66toHT', 'HT2T66Inverse',
    'InertiaTensor2Inertia6D', 'Inertia6D2InertiaTensor', 'TreeLink', 'RigidBodyInertia',
    'InertiaCuboid', 'InertiaRodX', 'InertiaMassPoint', 'InertiaSphere', 'InertiaHollowSphere',
    'InertiaCylinder', 'StrNodeType2NodeType', 'GetRigidBodyNode', 'AddRigidBody',
    'AddRevoluteJoint', 'AddPrismaticJoint',
    ]

eulerParameters0 = [1.,0.,0.,0.] #Euler parameters for case where rotation angle is zero (rotation axis arbitrary)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def ComputeOrthonormalBasisVectors(vector0):
    """compute orthogonal basis vectors (normal1, normal2) for given vector0 (non-unique solution!); the length of vector0 must not be 1; if vector0 == [0,0,0], then any normal basis is returned

    Returns:
        returns [vector0normalized, normal1, normal2], in which vector0normalized is the normalized vector0 (has unit length); all vectors in numpy array format
    """
    v = np.array([vector0[0],vector0[1],vector0[2]])

    L0 = np.linalg.norm(v)
    if L0 == 0:
        n1 = np.array([1,0,0])
        n2 = np.array([0,1,0])
    else:
        v = (1. / L0)*v;
    
        if (abs(v[0]) > 0.5) and (abs(v[1]) < 0.1) and (abs(v[2]) < 0.1):
            n1 = np.array([0., 1., 0.])
        else:
            n1 = np.array([1., 0., 0.])
    
        h = np.dot(n1, v);
        n1 -= h * v;
        n1 = (1/np.linalg.norm(n1))*n1;
        n2 = np.cross(v,n1)

    return [v, n1, n2]

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def ComputeOrthonormalBasis(vector0):
    """compute orthogonal basis, in which the normalized vector0 is the first column and the other columns are normals to vector0 (non-unique solution!); the length of vector0 must not be 1; if vector0 == [0,0,0], then any normal basis is returned

    Returns:
        returns A, a rotation matrix, in which the first column is parallel to vector0; A is a 2D numpy array
    """
    return np.vstack(ComputeOrthonormalBasisVectors(vector0)).T
    

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def GramSchmidt(vector0, vector1):
    """compute Gram-Schmidt projection of given 3D vector 1 on vector 0 and return normalized triad (vector0, vector1, vector0 x vector1)
    """

    v0 = np.array([vector0[0],vector0[1],vector0[2]])
    L0 = np.linalg.norm(v0)
    v0 = (1. / L0)*v0;
    
    v1 = np.array([vector1[0],vector1[1],vector1[2]])
    L1 = np.linalg.norm(v1)
    v1 = (1. / L1)*v1;
    
    h = np.dot(v1, v0);
    v1 -= h * v0;
    v1 = (1/np.linalg.norm(v1))*v1;
    n2 = np.cross(v0,v1)

    return [v0, v1, n2]


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def Skew(vector):
    """compute skew symmetric 3x3-matrix from 3x1- or 1x3-vector
    """
    skewsymmetricMatrix = np.array([[ 0.,       -vector[2], vector[1]], 
                                    [ vector[2], 0.,       -vector[0]],
                                    [-vector[1], vector[0], 0.]])
    return skewsymmetricMatrix

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def Skew2Vec(skew):
    """convert skew symmetric matrix m to vector
    """
    shape = skew.shape
    if shape == (3,3):
        w1 = skew[2][1]
        w2 = skew[0][2]
        w3 = -skew[0][1]
        vec = np.array([w1, w2, w3])
    if shape == (4,4):
        w1 = skew[2][1]
        w2 = skew[0][2]
        w3 = -skew[0][1]
        u1 = skew[0][3]
        u2 = skew[1][3]
        u3 = skew[2][3]
        vec = np.array([u1, u2, u3, w1, w2, w3])       
    return vec


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def ComputeSkewMatrix(v):
    """compute skew matrix from vector or matrix; used for ObjectFFRF and CMS implementation

    Args:
        a vector v in np.array format, containing 3*n components or a matrix with m columns of same shape

    Returns:
        if v is a vector, output is (3*n x 3) skew matrix in np.array format; if v is a (n x m) matrix, the output is a (3*n x m) skew matrix in np.array format
    """
    if type(v) == list or v.ndim == 1:
        n = int(len(v)/3) #number of nodes
        sm = np.zeros((3*n,3))

        for i in range(n):
            off = 3*i
            x=v[off+0]
            y=v[off+1]
            z=v[off+2]
            mLoc = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
            sm[off:off+3,:] = mLoc[:,:]
    
        return sm
    elif v.ndim==2: #dim=2
        (nRows,nCols) = v.shape
        n = int(nRows/3) #number of nodes
        sm = np.zeros((3*n,3*nCols))

        for j in range(nCols):
            for i in range(n):
                off = 3*i
                x=v[off+0,j]
                y=v[off+1,j]
                z=v[off+2,j]
                mLoc = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
                sm[off:off+3,3*j:3*j+3] = mLoc[:,:]
    else: 
        exu.Print("ERROR: wrong dimension in ComputeSkewMatrix(...)")
    return sm

#tests for ComputeSkewMatrix
#x = np.array([1,2,3,4,5,6])
#exu.Print(ComputeSkewMatrix(x))
#x = np.array([[1,2],[3,4],[5,6],[1,2],[3,4],[5,6]])
#exu.Print(ComputeSkewMatrix(x))


# OLD / duplicate with less functionality!
# #**function: compute (3 x 3*n) skew matrix from (3*n) vector
# def ComputeSkewMatrix(v):
#     n = int(len(v)/3) #number of nodes
#     sm = np.zeros((3*n,3))

#     for i in range(n):
#         off = 3*i
#         x=v[off+0]
#         y=v[off+1]
#         z=v[off+2]
#         sm[off:off+3,:] = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
#         # mLoc = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
#         # sm[off:off+3,:] = mLoc[:,:]
    
#     return sm



#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#helper functions for RIGID BODY KINEMATICS:

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def EulerParameters2G(eulerParameters):
    r"""convert Euler parameters (ep) to G-matrix (=$\partial \tomega  / \partial \pv_t$)

    Args:
        vector of 4 eulerParameters as list or np.array

    Returns:
        3x4 matrix G as np.array
    """
    ep = eulerParameters
    return np.array([[-2.*ep[1], 2.*ep[0],-2.*ep[3], 2.*ep[2]],
                     [-2.*ep[2], 2.*ep[3], 2.*ep[0],-2.*ep[1]],
                     [-2.*ep[3],-2.*ep[2], 2.*ep[1], 2.*ep[0]] ])

def EulerParameters2GLocal(eulerParameters):
    r"""convert Euler parameters (ep) to local G-matrix (=$\partial \LU{b}{\tomega} / \partial \pv_t$)

    Args:
        vector of 4 eulerParameters as list or np.array

    Returns:
        3x4 matrix G as np.array
    """
    ep = eulerParameters
    return np.array([[-2.*ep[1], 2.*ep[0], 2.*ep[3],-2.*ep[2]],
                     [-2.*ep[2],-2.*ep[3], 2.*ep[0], 2.*ep[1]],
                     [-2.*ep[3], 2.*ep[2],-2.*ep[1], 2.*ep[0]] ])

def EulerParameters2RotationMatrix(eulerParameters):
    """compute rotation matrix from eulerParameters

    Args:
        vector of 4 eulerParameters as list or np.array

    Returns:
        3x3 rotation matrix as np.array
    """
    ep = eulerParameters
    return np.array([[-2.0*ep[3]*ep[3] - 2.0*ep[2]*ep[2] + 1.0, -2.0*ep[3]*ep[0] + 2.0*ep[2]*ep[1], 2.0*ep[3]*ep[1] + 2.0*ep[2]*ep[0]],
                     [ 2.0*ep[3]*ep[0] + 2.0*ep[2]*ep[1], -2.0*ep[3]*ep[3] - 2.0*ep[1]*ep[1] + 1.0, 2.0*ep[3]*ep[2] - 2.0*ep[1]*ep[0]],
                     [-2.0*ep[2]*ep[0] + 2.0*ep[3]*ep[1], 2.0*ep[3]*ep[2] + 2.0*ep[1]*ep[0], -2.0*ep[2]*ep[2] - 2.0*ep[1]*ep[1] + 1.0] ])

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def RotationMatrix2EulerParameters(rotationMatrix):
    """compute Euler parameters from given rotation matrix

    Args:
        3x3 rotation matrix as list of lists or as np.array

    Returns:
        vector of 4 eulerParameters as np.array
    """
    A=np.array(rotationMatrix)
    trace = A[0,0] + A[1,1] + A[2,2] + 1.0
    M_EPSILON = 1e-15 #small number to avoid division by zero

    if (abs(trace) > M_EPSILON):
        s = 0.5 / np.sqrt(abs(trace))
        ep0 = 0.25 / s
        ep1 = (A[2,1] - A[1,2]) * s
        ep2 = (A[0,2] - A[2,0]) * s
        ep3 = (A[1,0] - A[0,1]) * s
    else:
        if (A[0,0] > A[1,1]) and (A[0,0] > A[2,2]):
            s = 2.0 * np.sqrt(abs(1.0 + A[0,0] - A[1,1] - A[2,2]))
            ep1 = 0.25 * s
            ep2 = (A[0,1] + A[1,0]) / s
            ep3 = (A[0,2] + A[2,0]) / s
            ep0 = (A[1,2] - A[2,1]) / s
        elif A[1,1] > A[2,2]:
            s = 2.0 * np.sqrt(abs(1.0 + A[1,1] - A[0,0] - A[2,2]))
            ep1 = (A[0,1] + A[1,0]) / s
            ep2 = 0.25 * s
            ep3 = (A[1,2] + A[2,1]) / s
            ep0 = (A[0,2] - A[2,0]) / s
        else:
            s = 2.0 * np.sqrt(abs(1.0 + A[2,2] - A[0,0] - A[1,1]));
            ep1 = (A[0,2] + A[2,0]) / s
            ep2 = (A[1,2] + A[2,1]) / s
            ep3 = 0.25 * s
            ep0 = (A[0,1] - A[1,0]) / s

    ep=np.array([ep0,ep1,ep2,ep3])
    #normalize Euler parameters, if rotation matrix is inaccurate; otherwise, may lead to errors in checkPreAssemble
    epNorm = np.linalg.norm(ep)
    if epNorm != 0.:
        ep *= 1./epNorm
    return ep

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def AngularVelocity2EulerParameters_t(angularVelocity, eulerParameters):
    r"""compute time derivative of Euler parameters from (global) angular velocity vector
    note that for Euler parameters $\pv$, we have $\tomega=\Gm \dot \pv$ ==> $\Gm^T \tomega = \Gm^T\cdot \Gm\cdot \dot \pv$ ==> $\Gm^T \Gm=4(\Im_{4 \times 4} - \pv\cdot \pv^T)\dot\pv = 4 (\Im_{4x4}) \dot \pv$

    Args:
        angularVelocity: 3D vector of angular velocity in global frame, as lists or as np.array
        eulerParameters: vector of 4 eulerParameters as np.array or list

    Returns:
        vector of time derivatives of 4 eulerParameters as np.array
    """
    
    GT = np.transpose(EulerParameters2G(eulerParameters))
    return 0.25*(GT.dot(angularVelocity))


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#            ROTATION VECTOR
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def RotationVector2RotationMatrix(rotationVector):
    r"""rotaton matrix from rotation vector, see appendix B in [Simo1988]

    Args:
        3D rotation vector as list or np.array

    Returns:
        3x3 rotation matrix as np.array

    Note:
        gets inaccurate for very large rotations, $\phi \\gg 2*\pi$
    """
    phi = np.linalg.norm(rotationVector)
    if phi == 0.:
        R = np.eye(3)
    else:
        if phi > 2*np.pi: 
            phi = phi % (2 * np.pi)
        OmegaSkew = Skew(rotationVector)
        alpha = np.sin(phi)/phi
        beta = 2*(1-np.cos(phi))/phi**2 #the loss of digits in 1-np.cos(phi) is compensated by OmegaSkew@OmegaSkew
        R = np.eye(3) + alpha*OmegaSkew + 0.5*beta*np.matmul(OmegaSkew, OmegaSkew)

    
    return R  


def RotationMatrix2RotationVector(rotationMatrix):
    """compute rotation vector from rotation matrix

    Args:
        3x3 rotation matrix as list of lists or as np.array

    Returns:
        vector of 3 components of rotation vector as np.array
    """
    ep = RotationMatrix2EulerParameters(rotationMatrix)
    
    n = ep[1:]
    norm = np.linalg.norm(n)
    
    #phi = 2.*acos(ep[0])
    phi = 2.*np.arctan2(norm, ep[0])
    
    if norm != 0.:
        n = (1./norm)*n

    return phi*n
    
    # # compute a  rotation vector from given rotation matrix according to 
    # # 2015 - Sonneville - A geometrical local frame approach for flexible multibody systems, p45
    # if np.linalg.norm(rotationMatrix - np.eye(3)) == 0.:
    #     rotationVector = np.zeros(3)
    # else:
    #     theta = np.arccos(0.5*(np.trace(rotationMatrix)-1))
    #     if abs(theta) < np.pi and abs(theta) > 0:
    #         logR = (theta/(2*np.sin(theta)))*(rotationMatrix - np.transpose(rotationMatrix))
    #         rotationVector = Skew2Vec(logR)
    #     else:
    #         rotationVector = np.zeros(3)

    # return rotationVector


def ComputeRotationAxisFromRotationVector(rotationVector):
    """compute rotation axis from given rotation vector

    Args:
        3D rotation vector as np.array

    Returns:
        3D vector as np.array representing the rotation axis
    """
    
    # compute rotation angle
    rotationAngle = np.linalg.norm(rotationVector)
    
    # compute rotation axis
    if rotationAngle == 0.0:
        rotationAxis = np.zeros(3)
    else:
        rotationAxis = rotationVector/rotationAngle
    
    # return rotation axis 
    return rotationAxis


def RotationVector2G(rotationVector):
    r"""convert rotation vector (parameters) (v) to G-matrix (=$\partial \tomega  / \partial \dot \vv$)

    Args:
        vector of rotation vector (len=3) as list or np.array

    Returns:
        3x3 matrix G as np.array
    """
    return RotationVector2RotationMatrix(rotationVector)

def RotationVector2GLocal(eulerParameters):
    r"""convert rotation vector (parameters) (v) to local G-matrix (=$\partial \LU{b}{\tomega}   / \partial \vv_t$)

    Args:
        vector of rotation vector (len=3) as list or np.array

    Returns:
        3x3 matrix G as np.array
    """
    return np.eye(3)



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#            TAIT BRYAN ANGLES
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def RotXYZ2RotationMatrix(rot):
    """compute rotation matrix from consecutive xyz [Rots](#Rot) (Tait-Bryan angles); A=Ax*Ay*Az; rot=[rotX, rotY, rotZ]

    Args:
        3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant

    Returns:
        3x3 rotation matrix as np.array
    """
    c0 = np.cos(rot[0])
    s0 = np.sin(rot[0])
    c1 = np.cos(rot[1])
    s1 = np.sin(rot[1])
    c2 = np.cos(rot[2])
    s2 = np.sin(rot[2])
    
    return np.array([[ c1*c2           ,-c1*s2           , s1    ],
                     [ s0*s1*c2 + c0*s2,-s0*s1*s2 + c0*c2,-s0*c1 ],
                     [-c0*s1*c2 + s0*s2, c0*s1*s2 + s0*c2, c0*c1 ]]);

def RotationMatrix2RotXYZ(rotationMatrix):
    """convert rotation matrix to xyz Euler angles (Tait-Bryan angles);  A=Ax*Ay*Az;

    Args:
        3x3 rotation matrix as list of lists or np.array

    Returns:
        vector of Tait-Bryan rotation parameters [X,Y,Z] (in radiant) as np.array

    Note:
        due to gimbal lock / singularity at rot[1] = pi/2, -pi/2, ... the reconstruction of
        `RotationMatrix2RotXYZ( RotXYZ2RotationMatrix(rot) )` may fail, but
        `RotXYZ2RotationMatrix( RotationMatrix2RotXYZ( RotXYZ2RotationMatrix(rot) ) )` works always
    """
    R=np.array(rotationMatrix)
    #rot=np.array([0,0,0])
    rot=np.zeros(3)
    absC1 = np.sqrt((-R[1,2])**2+R[2,2]**2)
    rot[1] = np.arctan2(R[0,2], absC1)
    if absC1 > 1e-14:
        rot[0] = np.arctan2(-R[1,2], R[2,2])
        rot[2] = np.arctan2(-R[0,1], R[0,0])
    else: #rot[0] and rot[2] represent same axes, set one of them zero!
        rot[0] = 0.
        #c1=0,s0=0,c0=1
        #s0*s1*c2 + c0*s2,-s0*s1*s2 + c0*c2 => c0*s2, c0*c2
        rot[2] = np.arctan2(R[1,0], R[1,1])
        
    return rot

# #OLD, problems at rot[1]=pi/2: rotation represents different rotation matrix
# def RotationMatrix2RotXYZ(rotationMatrix):
#     R=np.array(rotationMatrix)
#     #rot=np.array([0,0,0])
#     rot=[0,0,0]
#     rot[0] = np.arctan2(-R[1,2], R[2,2])
#     rot[1] = np.arctan2(R[0,2], np.sqrt(abs(1. - R[0,2] * R[0,2]))) #fabs for safety, if small round up error in rotation matrix ...
#     rot[2] = np.arctan2(-R[0,1], R[0,0])
#     return np.array(rot);


def RotXYZ2G(rot):
    r"""compute (global-frame) G-matrix for xyz Euler angles (Tait-Bryan angles) ($\LU{0}{\Gm} = \partial \LU{0}{\tomega}  / \partial \dot \ttheta$)

    Args:
        3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant

    Returns:
        3x3 matrix G as np.array
    """
    c0 = cos(rot[0])
    s0 = sin(rot[0])
    c1 = cos(rot[1])
    s1 = sin(rot[1])

    return np.array([[1, 0, s1],
                     [0, c0, -c1*s0],
                     [0, s0,  c0*c1 ]])

def RotXYZ2G_t(rot, rot_t):
    r"""compute time derivative of (global-frame) G-matrix for xyz Euler angles (Tait-Bryan angles) ($\LU{0}{\Gm} = \partial \LU{0}{\tomega}  / \partial \dot \ttheta$)

    Args:
        rot: 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
        rot_t: 3D vector of time derivative of Tait-Bryan rotation parameters [X,Y,Z] in radiant/s

    Returns:
        3x3 matrix G_t as np.array
    """
    c0 = cos(rot[0])
    s0 = sin(rot[0])
    c1 = cos(rot[1])
    s1 = sin(rot[1])

    return np.array([[0, 0, rot_t[1]*c1],
                     [0, -rot_t[0]*s0, rot_t[1]*s0*s1 - rot_t[0]*c0*c1],
                     [0, rot_t[0]*c0, -rot_t[0]*c1*s0 - rot_t[1]*c0*s1]])


def RotXYZ2GLocal(rot):
    r"""compute local (body-fixed) G-matrix for xyz Euler angles (Tait-Bryan angles) ($\LU{b}{\Gm} = \partial \LU{b}{\tomega}  / \partial \ttheta_t$)

    Args:
        3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant

    Returns:
        3x3 matrix GLocal as np.array
    """
    c1 = cos(rot[1])
    s1 = sin(rot[1])
    c2 = cos(rot[2])
    s2 = sin(rot[2])

    return np.array([[ c1*c2, s2, 0],
                     [-c1*s2, c2, 0],
                     [ s1,     0,  1]])

def RotXYZ2GLocal_t(rot, rot_t):
    r"""compute time derivative of (body-fixed) G-matrix for xyz Euler angles (Tait-Bryan angles) ($\LU{b}{\Gm} = \partial \LU{b}{\tomega}  / \partial \ttheta_t$)

    Args:
        rot: 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
        rot_t: 3D vector of time derivative of Tait-Bryan rotation parameters [X,Y,Z] in radiant/s

    Returns:
        3x3 matrix GLocal_t as np.array
    """
    c1 = cos(rot[1])
    s1 = sin(rot[1])
    c2 = cos(rot[2])
    s2 = sin(rot[2])

    return np.array([[-rot_t[2]*c1*s2 - rot_t[1]*c2*s1, rot_t[2]*c2, 0],
                     [ rot_t[1]*s2*s1 - rot_t[2]*c2*c1, -rot_t[2]*s2, 0],
                     [ rot_t[1]*c1, 0, 0 ]])






def AngularVelocity2RotXYZ_t(angularVelocity, rotation):
    """compute time derivatives of angles RotXYZ from (global) angular velocity vector and given rotation

    Args:
        angularVelocity: global angular velocity vector as list or np.array
        rotation: 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant

    Returns:
        time derivative of vector of Tait-Bryan rotation parameters [X,Y,Z] (in radiant) as np.array
    """
    psi = rotation[0]
    theta = rotation[1]
    #phi = rotation[2] #not needed
    cTheta = np.cos(theta)
    if cTheta == 0:
        exu.Print('AngularVelocity2RotXYZ_t: not possible for rotation[1] == pi/2, 3*pi/2, ...')

    GInv = (1/cTheta)*np.array([[np.cos(theta), np.sin(psi)*np.sin(theta),-np.cos(psi)*np.sin(theta)],
                                [0            , np.cos(psi)*np.cos(theta)   , np.sin(psi)*np.cos(theta)],
                                [0            ,-np.sin(psi)              , np.cos(psi)]])
    return np.dot(GInv,angularVelocity)
  
    
def RotXYZ2EulerParameters(alpha):
    """compute four Euler parameters from given RotXYZ angles, see [Henderson1977]

    Args:
        alpha: 3D vector as np.array containing RotXYZ angles

    Returns:
        4D vector as np.array containing four Euler parameters
        entry zero of output represent the scalar part of Euler parameters
    """
    psi   = alpha[0]
    theta = alpha[1]
    phi   = alpha[2]   
    u = 0.5*psi
    v = 0.5*theta
    w = 0.5*phi    
    cPsi   = np.cos(u)
    cTheta = np.cos(v)
    cPhi   = np.cos(w)    
    sPsi   = np.sin(u)
    sTheta = np.sin(v)
    sPhi   = np.sin(w)    
    q0 = -sPsi*sTheta*sPhi + cPsi*cTheta*cPhi
    q1 =  sPsi*cTheta*cPhi + sTheta*sPhi*cPsi    
    q2 = -sPsi*sPhi*cTheta + sTheta*cPsi*cPhi 
    q3 =  sPsi*sTheta*cPhi + sPhi*cPsi*cTheta 
    return np.array([q0, q1, q2, q3])


#%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#            Euler ANGLES
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

@docmeta(author='Martin Sereinig')
def RotationMatrix2RotZYZ(rotationMatrix, flip):
    """convert rotation matrix to zyz Euler angles;  A=Az*Ay*Az;

    Args:
        rotationMatrix: 3x3 rotation matrix as list of lists or np.array
        flip:           argument to choose first Euler angle to be in quadrant 2 or 3.

    Returns:
        vector of Euler rotation parameters [Z,Y,Z] (in radiant) as np.array

    Note:
        tested (compared with Robotics, Vision and Control book of P. Corke)
    """
    R=np.array(rotationMatrix)
    # Method as per Paul, p 69.
    # euler = [phi theta psi]
    eulangles = np.zeros([3])
    eps = 10**(-14)

    if abs(R[0, 2]) < eps and abs(R[1, 2]) < eps:
        # singularity
        eulangles[0] = 0
        sp = 0
        cp = 1
        eulangles[1] = np.arctan2(
            cp*R[0, 2] + sp*R[1, 2], R[2, 2])
        eulangles[2] = np.arctan2(-sp * R[0, 0] + cp *
                                  R[1, 0], -sp*R[0, 1] + cp*R[1, 1])
    else:
        # non singular
        # Only positive phi is returned.
        if flip:
            eulangles[0] = np.arctan2(-R[1, 2], -R[0, 2])
        else:
            eulangles[0] = np.arctan2(R[1, 2], R[0, 2])

        sp = np.sin(eulangles[0])
        cp = np.cos(eulangles[0])
        eulangles[1] = np.arctan2(
            cp*R[0, 2] + sp*R[1, 2], R[2, 2])
        eulangles[2] = np.arctan2(-sp * R[0, 0] + cp *
                                  R[1, 0], -sp*R[0, 1] + cp*R[1, 1])
    return eulangles



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def RotationMatrixX(angleRad):
    """compute rotation matrix w.r.t. X-axis (first axis)

    Args:
        angle around X-axis in radiant

    Returns:
        3x3 rotation matrix as np.array
    """
    return np.array([[1, 0, 0],
                     [0, np.cos(angleRad),-np.sin(angleRad)],
                     [0, np.sin(angleRad), np.cos(angleRad)] ])

def RotationMatrixY(angleRad):
    """compute rotation matrix w.r.t. Y-axis (second axis)

    Args:
        angle around Y-axis in radiant

    Returns:
        3x3 rotation matrix as np.array
    """
    return np.array([ [ np.cos(angleRad), 0, np.sin(angleRad)],
                      [0,        1, 0],
                      [-np.sin(angleRad),0, np.cos(angleRad)] ])

def RotationMatrixZ(angleRad):
    """compute rotation matrix w.r.t. Z-axis (third axis)

    Args:
        angle around Z-axis in radiant

    Returns:
        3x3 rotation matrix as np.array
    """
    return np.array([ [np.cos(angleRad),-np.sin(angleRad), 0],
                      [np.sin(angleRad), np.cos(angleRad), 0],
                      [0,        0,        1] ]);

def RotationMatrix2D(angleRad):
    """compute 2D rotation matrix

    Args:
        angle around out-of-plane axis in radiant

    Returns:
        2x2 rotation matrix as np.array
    """
    return np.array([ [np.cos(angleRad),-np.sin(angleRad)],
                      [np.sin(angleRad), np.cos(angleRad)] ]);

    
#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#functions for homogeneous transformations (HT)
def HomogeneousTransformation(A, r):
    """compute [HT](#HT) matrix from rotation matrix A and translation vector r, as a 4x4 numpy array;
    exudyn.HT is the C++ class of the same transformation, faster in products, inverses and transformed points:
    exudyn.HT(rotation=A, translation=r) (#2780); the shortcut HT of this module is this function
    """
    T = np.zeros((4,4))
    T[0:3,0:3] = A
    T[0:3,3] = r
    T[3,3] = 1
    return T

HT = HomogeneousTransformation #shortcut

def HTtranslate(r):
    """[HT](#HT) for translation with vector r
    """
    T = np.eye(4)
    T[0:3,3] = r
    return T

def HTtranslateX(x):
    """[HT](#HT) for translation along x axis with value x
    """
    T = np.eye(4)
    T[0,3] = x
    return T

def HTtranslateY(y):
    """[HT](#HT) for translation along y axis with value y
    """
    T = np.eye(4)
    T[1,3] = y
    return T

def HTtranslateZ(z):
    """[HT](#HT) for translation along z axis with value z
    """
    T = np.eye(4)
    T[2,3] = z
    return T

def HT0():
    """identity [HT](#HT):
    """
    return np.eye(4)

def HTrotateX(angle):
    """[HT](#HT) for rotation around axis X (first axis)
    """
    T = np.eye(4)
    T[0:3,0:3] = RotationMatrixX(angle)
    return T
    
def HTrotateY(angle):
    """[HT](#HT) for rotation around axis X (first axis)
    """
    T = np.eye(4)
    T[0:3,0:3] = RotationMatrixY(angle)
    return T
    
def HTrotateZ(angle):
    """[HT](#HT) for rotation around axis X (first axis)
    """
    T = np.eye(4)
    T[0:3,0:3] = RotationMatrixZ(angle)
    return T

def HT2translation(T):
    """return translation part of [HT](#HT)
    """
    return T[0:3,3]

def HT2rotationMatrix(T):
    """return rotation matrix of [HT](#HT)
    """
    return T[0:3,0:3]

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def InverseHT(T):
    """return inverse [HT](#HT) such that inv(T)*T = np.eye(4)
    """
    Tinv = np.eye(4)
    Ainv = T[0:3,0:3].T #inverse rotation part
    Tinv[0:3,0:3] = Ainv
    r = T[0:3,3]        #translation part
    Tinv[0:3,3]  = -Ainv @ r       #inverse translation part
    return Tinv

################################################################################
#Test (compared with Robotcs, Vision and Control book of P. Corke:
#T=HTtranslate([1,0,0]) @ HTrotateX(np.pi/2) @ HTtranslate([0,1,0])
#exu.Print("T=",T.round(8))
#
#R = RotationMatrixZ(0.1) @ RotationMatrixY(0.2) @ RotationMatrixZ(0.3) 
#exu.Print("R=",R.round(4))

################################################################################

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#functions for 6x6 coordinate transformation matrices (\ac{T66}), see Featherstone / Handbook of robotics \cite{Siciliano2016}
def RotationX2T66(angle):
    """compute 6x6 coordinate transformation matrix for rotation around X axis; output: first 3 components for rotation, second 3 components for translation! See Featherstone / Handbook of robotics [Siciliano2016]
    """
    c = cos(angle);
    s = sin(angle);
    return np.array(
        [[1,  0,  0,  0,  0,  0],
         [0,  c, -s,  0,  0,  0],
         [0,  s,  c,  0,  0,  0],
         [0,  0,  0,  1,  0,  0],
         [0,  0,  0,  0,  c, -s],
         [0,  0,  0,  0,  s,  c]])

def RotationY2T66(angle):
    """compute 6x6 transformation matrix for rotation around Y axis; output: first 3 components for rotation, second 3 components for translation
    """
    c = cos(angle);
    s = sin(angle);
    return np.array(
        [[c,  0,  s,  0,  0,  0],
         [0,  1,  0,  0,  0,  0],
         [-s, 0,  c,  0,  0,  0],
         [0,  0,  0,  c,  0,  s],
         [0,  0,  0,  0,  1,  0],
         [0,  0,  0, -s,  0,  c]])

def RotationZ2T66(angle):
    """compute 6x6 transformation matrix for rotation around Z axis; output: first 3 components for rotation, second 3 components for translation
    """
    c = cos(angle);
    s = sin(angle);
    return np.array(
        [[ c, -s,  0,  0,  0,  0],
         [ s,  c,  0,  0,  0,  0],
         [ 0,  0,  1,  0,  0,  0],
         [ 0,  0,  0,  c, -s,  0],
         [ 0,  0,  0,  s,  c,  0],
         [ 0,  0,  0,  0,  0,  1]])

def Translation2T66(translation3D):
    """compute 6x6 transformation matrix for translation according to 3D vector translation3D; output: first 3 components for rotation, second 3 components for translation!
    """
    t = translation3D
    return np.array(
        [[    1,    0,    0,  0,  0,  0],
         [    0,    1,    0,  0,  0,  0],
         [    0,    0,    1,  0,  0,  0],
         [    0, t[2],-t[1],  1,  0,  0],
         [-t[2],    0, t[0],  0,  1,  0],
         [ t[1],-t[0],    0,  0,  0,  1]])

def TranslationX2T66(translation):
    """compute 6x6 transformation matrix for translation along X axis; output: first 3 components for rotation, second 3 components for translation!
    """
    return Translation2T66([translation,0,0])

def TranslationY2T66(translation):
    """compute 6x6 transformation matrix for translation along Y axis; output: first 3 components for rotation, second 3 components for translation!
    """
    return Translation2T66([0,translation,0])

def TranslationZ2T66(translation):
    """compute 6x6 transformation matrix for translation along Z axis; output: first 3 components for rotation, second 3 components for translation!
    """
    return Translation2T66([0,0,translation])

def T66toRotationTranslation(T66):
    """convert 6x6 coordinate transformation (Plücker transform) into rotation and translation

    Args:
        T66 given as  6x6 numpy array

    Returns:
        [A, v] with 3x3 rotation matrix A and 3D translation vector v
    """
    A = T66[0:3,0:3]
    v = Skew2Vec(T66[3:6,0:3]@A.T) #this leads to identical backtransformation
    return [A, v] 

def InverseT66toRotationTranslation(T66):
    """convert inverse 6x6 coordinate transformation (Plücker transform) into rotation and translation

    Args:
        inverse T66 given as  6x6 numpy array

    Returns:
        [A, v] with 3x3 rotation matrix A and 3D translation vector v
    """
    A = (T66[0:3,0:3]).T
    v = -Skew2Vec(A@T66[3:6,0:3])
    return [A, v] 

def RotationTranslation2T66(A, v):
    """convert rotation and translation into 6x6 coordinate transformation (Plücker transform)

    Args:
        A: 3x3 rotation matrix A
        v: 3D translation vector v

    Returns:
        return 6x6 transformation matrix 'T66'
    """
    return np.block([
        [A, np.zeros((3,3))], 
        [Skew(v)@A, A]]) 

def RotationTranslation2T66Inverse(A, v):
    """convert rotation and translation into INVERSE 6x6 coordinate transformation (Plücker transform)

    Args:
        A: 3x3 rotation matrix A
        v: 3D translation vector v

    Returns:
        return 6x6 transformation matrix 'T66'
    """
    return np.block([
        [A.T, np.zeros((3,3))], 
        [-A.T@Skew(v), A.T]]) 

def T66Inverse(T66):
    """compute inverse of 6x6 coordinate transformation (Plücker transform)

    Args:
        T66: 6x6 coordinate transformation (Plücker transform)

    Returns:
        return inverse 6x6 transformation matrix 'T66'

    Note:
        Skew(A@v) = A@Skew(v)@A.T; v=ApB: -BRA@Skew(ApB) = Skew(BpA)@BRA
    """
    A = T66[0:3,0:3] #BRA in Handbook of robotics
    v = Skew2Vec(T66[3:6,0:3]@A.T) #v=BpA in in Handbook of robotics ==> ApB=-BRA.T@BpA = -A.T@v
        
    return np.block([
        [         A.T, np.zeros((3,3))], 
        [-A.T@Skew(v), A.T            ]])
# #identical (using an inverse representation of v):
#     A = T66[0:3,0:3] #BRA in Handbook of robotics
#     v = -Skew2Vec(A.T @ T66[3:6,0:3]) #v=ApB in in Handbook of robotics ==> BpA=-BRA@ApB = -A@v
        
#     return np.block([
#         [A.T, np.zeros((3,3))], 
#         [A.T@Skew(A@v), A.T]])

def T66toHT(T66):
    """convert 6x6 coordinate transformation (Plücker transform) into 4x4 homogeneous transformation; NOTE that the homogeneous transformation is the inverse of what is computed in function pluho() of Featherstone

    Args:
        T66 given as 6x6 numpy array

    Returns:
        homogeneous transformation (4x4 numpy array)
    """
    A = T66[0:3,0:3]
    T = np.zeros((4,4))
    T[0:3,0:3] = A
    T[0:3,3] = Skew2Vec(T66[3:6,0:3] @ A.T)
    T[3,3] = 1
    return T

def HT2T66Inverse(T):
    """convert 4x4 homogeneous transformation into 6x6 coordinate transformation (Plücker transform); NOTE that the homogeneous transformation is the inverse of what is computed in function pluho() of Featherstone

    Args:
        T: 4x4 homogeneous transformation (numpy array)

    Returns:
        T66 (6x6 numpy array)
    """
    A = T[0:3,0:3].T 
    v = T[0:3,3]
    return np.block([
        [A, np.zeros((3,3))],
        [-A@Skew(v), A]])

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#inertia 6D functions

def InertiaTensor2Inertia6D(inertiaTensor):
    """convert a 3x3 matrix (list or numpy array) into a list with 6 inertia components, sorted as J00, J11, J22, J12, J02, J01
    """
    J = np.array(inertiaTensor)
    return [J[0,0], J[1,1], J[2,2],  J[1,2], J[0,2], J[0,1]]

def Inertia6D2InertiaTensor(inertia6D):
    """convert a list or numpy array with 6 inertia components (sorted as [J00, J11, J22, J12, J02, J01]) (list or numpy array) into a 3x3 matrix (np.array)
    """
    J = inertia6D
    return np.array([[J[0],J[5],J[4]],
                     [J[5],J[1],J[3]],
                     [J[4],J[3],J[2]]])


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class TreeLink:
    """helper class for CreateKinematicTree, representing a link on a joint within a kinematic tree

    Example:
        link3 = TreeLink(linkInertia = InertiaCuboid(2800, [0.25,0.08,0.08]).Translated([0.125,0,0]),
                         jointType = =exu.JointType.RevoluteZ,
                         parent = 1,
                         graphicsData = graphics.Brick(centerPoint=[0.125,0,0], size=[0.25,0.08,0.08],
                                                       color=graphics.color.blue),
                         )
    """
    def __init__(self, linkInertia, 
                 jointType=exu.JointType.RevoluteZ,
                 jointHT=HT0(),
                 parent=None, 
                 PDcontrol=None, 
                 graphicsDataList=None):
        """initialize inertia

        Args:
            linkInertia: RigidBodyInertia class, containing mass, inertia, and COM
            jointHT: transformation from previous link to this link's joint
            parent: index to parent link; if parent link is ground, use -1; if all parents in a serial kinematic tree are None, parent indices are computed automatically
            PDcontrol: tuple of PD control parameters
            graphicsData: graphicsDataList link; None automatically adds a suitable graphical object from next joint to this joint; use empty list [] to add no graphics for link
        """
        self.jointType = jointType
        self.linkInertia = linkInertia
        self.jointHT = jointHT
        self.parent = parent
        self.PDcontrol = PDcontrol
        self.graphicsDataList = graphicsDataList


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class RigidBodyInertia:
    """helper class for rigid body inertia (see also derived classes Inertia...).
    Provides a structure to define mass, inertia and center of mass (COM) of a rigid body.
    The inertia tensor and center of mass must correspond when initializing the body!

    Note:
        in the default mode, inertiaTensorAtCOM = False, the inertia tensor must be provided with respect to the reference point; otherwise, it is given at COM; internally, the inertia tensor is always with respect to the reference point, not w.r.t. to COM!

    Example:
        i0 = RigidBodyInertia(10,np.diag([1,2,3]))
        i1 = i0.Rotated(RotationMatrixX(np.pi/2))
        i2 = i1.Translated([1,0,0])
    """
    def __init__(self, mass=0, inertiaTensor=np.zeros([3,3]), com=np.zeros(3), inertiaTensorAtCOM = False):
        """initialize RigidBodyInertia with scalar mass, 3x3 inertiaTensor (w.r.t. reference point!!!) and center of mass com

        Args:
            mass: mass of rigid body (dimensions need to be consistent, should be in SI-units)
            inertiaTensor: tensor given w.r.t. reference point, NOT w.r.t. center of mass!
            com: center of mass relative to reference point, in same coordinate system as inertiaTensor
            inertiaTensorAtCOM: bool flag: if False (default), the inertiaTensor has to be provided w.r.t. the reference point; if True, it has to be provided at the center of mass
        """
        
        if not isinstance(inertiaTensorAtCOM, bool) and not isinstance(inertiaTensorAtCOM, int): 
            raise ValueError('RigidBodyInertia: inertiaTensorAtCOM must be bool or int (0/1), but received '+str(inertiaTensorAtCOM))
        if np.array(inertiaTensor).shape != (3,3): #shape is a tuple
            raise ValueError('RigidBodyInertia: inertiaTensor must have shape (3,3), but received '+str(inertiaTensor.shape))
        if np.array(com).shape != (3,): #shape is a tuple
            raise ValueError('RigidBodyInertia: com must have 3 components, but received '+str(np.array(inertiaTensor).shape))

        #default values for graphics
        self._nTilesGraphics = 16       #for spheres; for cylinders factor is multiplied by 2
        self._roundnessGraphics = 0.    #for InertiaCuboid, make it round

        self.data = {'type':'RigidBodyInertia'} #special data, like radius, etc. (used for drawing)
        
        #further checks if inertia tensor makes sense...
        Ix = inertiaTensor[0,0]
        Iy = inertiaTensor[1,1]
        Iz = inertiaTensor[2,2]
        if (Ix + Iy < Iz) or (Ix + Iz < Iy) or (Iy + Iz < Ix):
            exu.Print('WARNING: RigidBodyInertia: inertiaTensor does not fulfill triangle inequality! This may lead to unphysical and numerically unstable results!')
        if (Ix + Iy < Iz) or (Ix + Iz < Iy) or (Iy + Iz < Ix):
            exu.Print('WARNING: RigidBodyInertia: inertiaTensor does not fulfill triangle inequality! This may lead to unphysical and numerically unstable results!')

        norm = np.linalg.norm(inertiaTensor)
        if norm != 0: #in case of mass point, this shall be possible
            if np.linalg.norm(inertiaTensor - inertiaTensor.T)/norm > 1e-14:
                exu.Print('WARNING: RigidBodyInertia: inertiaTensor seems to be unsymmetric; This may lead to unphysical and numerically unstable results!')
            
        self.mass = mass
        self.inertiaTensor = np.array(inertiaTensor)
        self.com = np.array(com)
        if inertiaTensorAtCOM:
            self.inertiaTensor = self.inertiaTensor + self.mass*np.dot(Skew(self.com).transpose(),Skew(self.com))
        
    def __add__(self, otherBodyInertia):
        """add (+) operator allows adding another inertia information with SAME local coordinate system and reference point!
        only inertias with same center of rotation can be added!

        Example:
            J = InertiaSphere(2,0.1) + InertiaRodX(1,2)
        """
        sumMass = self.mass + otherBodyInertia.mass
        return RigidBodyInertia(mass=sumMass,
                                inertiaTensor = self.inertiaTensor + otherBodyInertia.inertiaTensor,
                                com=1./sumMass*(self.mass*self.com + otherBodyInertia.mass*otherBodyInertia.com))

    def __iadd__(self, otherBodyInertia):
        """+= operator allows adding another inertia information with SAME local coordinate system and reference point!
        only inertias with same center of rotation can be added!

        Example:
            J = InertiaSphere(2,0.1)
            J += InertiaRodX(1,2)
        """
        self = self + otherBodyInertia
        return self
        
    def SetWithCOMinertia(self, mass, inertiaTensorCOM, com):
        """set RigidBodyInertia with scalar mass, 3x3 inertiaTensor (w.r.t. com) and center of mass com

        Args:
            mass: mass of rigid body (dimensions need to be consistent, should be in SI-units)
            inertiaTensorCOM: tensor given w.r.t. reference point, NOT w.r.t. center of mass!
            com: center of mass relative to reference point, in same coordinate system as inertiaTensor
        """
        if np.array(inertiaTensorCOM).shape != (3,3): #shape is a tuple
            raise ValueError('RigidBodyInertia: inertiaTensorCOMmust have shape (3,3), but received '+str(np.array(inertiaTensorCOM).shape))
        if np.array(com).shape != (3,): #shape is a tuple
            raise ValueError('RigidBodyInertia: com must have 3 components, but received '+str(np.array(inertiaTensorCOM).shape))
        self.mass = mass
        self.com = np.array(com)
        self.inertiaTensor = np.array(inertiaTensorCOM) + self.mass*np.dot(Skew(self.com).transpose(),Skew(self.com))
        
        
    def Inertia(self):
        """returns 3x3 inertia tensor with respect to chosen reference point (not necessarily COM)
        """
        return self.inertiaTensor

    def InertiaCOM(self):
        """returns 3x3 inertia tensor with respect to COM
        """
        return self.inertiaTensor - self.mass*np.dot(Skew(self.com).transpose(),Skew(self.com))

    def COM(self):
        """returns center of mass (COM) w.r.t. chosen reference point
        """
        return self.com

    def Mass(self):
        """returns mass
        """
        return self.mass

    def Translated(self, vec):
        r"""returns a RigidBodyInertia with center of mass com shifted by vec; $\ra$ transforms the returned inertiaTensor to the new center of rotation
        """
        #transform inertia to com=[0,0,0]
        inertiaCOM = self.inertiaTensor - self.mass*np.dot(Skew(self.com).transpose(),Skew(self.com))
        try:
            newCOM = self.com + vec
        except (ValueError, TypeError):
            raise ValueError("ERROR in RigidBodyInertia.Translated(vec): vec must be a vector with 3 components")
        inertiaCOM += self.mass*np.dot(Skew(newCOM).transpose(),Skew(newCOM))
        rbi = RigidBodyInertia(mass=self.mass, 
                               inertiaTensor=inertiaCOM,
                               com=newCOM)
        rbi.data.update(self.data)
        if 'HT' in self.data:
            rbi.data['HT'] = self.data['HT'] @ HTtranslate(vec)
        else:
            rbi.data['HT'] = HTtranslate(vec)
        return rbi

    def Rotated(self, rot):
        """returns a RigidBodyInertia rotated by 3x3 rotation matrix rot, such that for a given J, the new inertia tensor reads Jnew = rot*J*rot.T

        Note:
            only allowed if COM=0 !
        """
        if np.linalg.norm(self.com) != 0:
            exu.Print("ERROR: RigidBodyInertia.Rotated only allowed in case of com=0")
            return 0
        try:
            inertia = np.dot(np.array(rot),np.dot(self.inertiaTensor,rot.transpose()))
        except (ValueError, TypeError, AttributeError):
            raise ValueError("ERROR in RigidBodyInertia.Rotated(rot): rot must be a 3x3 rotation matrix")
        return RigidBodyInertia(mass=self.mass, 
                                inertiaTensor=inertia,
                                com=self.com)

    def Transformed(self, HT):
        """return rigid body inertia transformed by homogeneous transformation HT
        """
        A = HT2rotationMatrix(HT)
        v = HT2translation(HT)
        
        inertiaCOM = self.inertiaTensor - self.mass*np.dot(Skew(self.com).transpose(),Skew(self.com))
        inertiaCOM = A @ inertiaCOM @ A.T #tested with general rigid body and shifted reference point

        newCOM = A @ self.com + v

        inertiaCOM += self.mass*np.dot(Skew(newCOM).transpose(),Skew(newCOM))
        rbi = RigidBodyInertia(mass=self.mass, 
                                inertiaTensor=inertiaCOM,
                                com=newCOM)
        rbi.data.update(self.data)
        if 'HT' in self.data:
            rbi.data['HT'] = self.data['HT'] @ HT
        else:
            rbi.data['HT'] = HT
        return rbi
    
    def GetInertia6D(self):
        """get vector with 6 inertia components (Jxx, Jyy, Jzz, Jyz, Jxz, Jxy) w.r.t. to reference point (not necessarily the COM), as needed in ObjectRigidBody
        """
        return InertiaTensor2Inertia6D(self.inertiaTensor)
        # J = self.inertiaTensor
        # return [J[0][0], J[1][1], J[2][2],  J[1][2], J[0][2], J[0][1]]

    def GetTypeName(self):
        """which returns str of type ('InertiaCylinder', 'InertiaCuboid', ...)
        """
        return self.__class__.__name__

    def GetSpecialData(self):
        """returns dictionary with further data of inertia, like cylinder radius, etc.
        """
        return self.data


    def GetGraphics(self, color, nTiles=None, roundness=None):
        """get graphicsData object from inertia; this simplifies the rigid body creation process and allows to check for consistency; currently does not include HT-rotations!
        """
    
        color0 = color
        if color[0] == -1:
            color0 = graphics.color.defaultBody #default body color; otherwise not visible
        
        nTiles = self._nTilesGraphics if nTiles is None else nTiles
        roundness = self._roundnessGraphics if roundness is None else roundness

        com = self.COM()
        typeName = self.data['type']
        #++++++++++++++++++++++++++++++++++++++++++++++
        if typeName == 'InertiaCylinder':
            length=self.data['length']
            outerRadius=self.data['outerRadius']
            innerRadius=self.data['innerRadius']
            axis=self.data['axis']
            axisVector = np.array([0,0,0])
            axisVector[axis] = 1
            gData = graphics.Cylinder(pAxis=-0.5*length*axisVector+com,
                                      vAxis=length*axisVector,
                                      radius=outerRadius,
                                      color=color0,
                                      nTiles=nTiles*2,
                                      radiusInner=innerRadius if innerRadius>0 else None
                                      )
        elif typeName == 'InertiaCuboid':
            sideLengths=self.data['sideLengths']
            gData = graphics.Brick(centerPoint=com, size=sideLengths, color=color0,
                                   roundness=roundness)
    
        elif typeName == 'InertiaRodX':
            length=self.data['length']
            gData = graphics.Brick(centerPoint=com, size=[length, length*0.01, length*0.01], 
                                   color=color0, roundness=roundness)
    
        elif typeName == 'InertiaMassPoint':
            mass=self.data['mass']
            #V=4./3.*np.pi*r**3 => m/rho = V => r
            r = (mass/2000*0.75/np.pi)**(1/3) #approximation for radius with rho=2000
            gData = graphics.Sphere(point=com, radius=r, nTiles=nTiles, color=color0) #size is not known
    
        elif typeName == 'InertiaSphere':
            radius=self.data['radius']
            gData = graphics.Sphere(point=com, radius=radius, nTiles=nTiles, color=color0) #size is not known
    
        elif typeName == 'InertiaSphere' or typeName == 'InertiaHollowSphere':
            radius=self.data['radius']
            gData = graphics.Sphere(point=com, radius=radius, nTiles=nTiles, color=color0) #size is not known

        else:
            return None #signals that no graphics data could be extracted
        return gData


    def __str__(self):
        s = 'mass = ' + str(self.mass)
        s += '\nCOM = ' + str(self.com)
        s += '\ninertiaTensorAtOrigin = \n' + str(self.inertiaTensor)
        s += '\ninertiaTensorAtCOM = \n' + str(self.InertiaCOM())
        return s
    def __repr__(self):
        return str(self)


class InertiaCuboid(RigidBodyInertia):
    """create RigidBodyInertia with moment of inertia and mass of a cuboid with density and side lengths sideLengths along local axes 1, 2, 3; inertia w.r.t. center of mass, com=[0,0,0]

    Example:
        InertiaCuboid(density=1000,sideLengths=[1,0.1,0.1])
    """
    def __init__(self, density, sideLengths):
        """initialize inertia
        """
        L1=sideLengths[0]
        L2=sideLengths[1]
        L3=sideLengths[2]
        newMass=density*L1*L2*L3
        RigidBodyInertia.__init__(self, mass=newMass,
                                  inertiaTensor=newMass/12.*np.diag([(L2**2 + L3**2),(L1**2 + L3**2),(L1**2 + L2**2)]),
                                  com=np.zeros(3))
        self.data = {'type':'InertiaCuboid', 'density':density, 'sideLengths':sideLengths}

class InertiaRodX(RigidBodyInertia):
    """create RigidBodyInertia with moment of inertia and mass of a rod with mass m and length L in local 1-direction (x-direction); inertia w.r.t. center of mass, com=[0,0,0]
    """
    def __init__(self, mass, length):
        """initialize inertia with mass and length of rod
        """
        RigidBodyInertia.__init__(self, mass=mass,
                                  inertiaTensor=mass/12.*np.diag([0.,length**2,length**2]),
                                  com=np.zeros(3))
        self.data = {'type':'InertiaRodX', 'mass':mass, 'length':length}
        
class InertiaMassPoint(RigidBodyInertia):
    """create RigidBodyInertia with moment of inertia and mass of mass point with given 'mass'; inertia w.r.t. center of mass, com=[0,0,0]; note that the inertia tensor gives zero and cannot be directly used in rigid bodies, however, it can be used to be added to another inertia tensor (e.g. to add unbalance)
    """
    def __init__(self, mass):
        """initialize inertia with mass of point
        """
        RigidBodyInertia.__init__(self, mass=mass,
                                  inertiaTensor=np.zeros([3,3]),
                                  com=np.zeros(3))
        self.data = {'type':'InertiaMassPoint', 'mass':mass}

class InertiaSphere(RigidBodyInertia):
    """create RigidBodyInertia with moment of inertia and mass of sphere with mass and radius; inertia w.r.t. center of mass, com=[0,0,0]
    """
    def __init__(self, mass=None, radius=None, density=None):
        """initialize inertia with mass and radius of sphere
        """
        volume = 4./3. * np.pi * radius**3
        if density is None and mass is not None:
            density = mass / volume if volume!=0 else 0 #to ignore cases where someone likes to use radius=0
        elif density is not None and mass is None:
            mass = density * volume
        else:
            raise ValueError('InertiaSphere: invalid args provided: either mass is a float number, then density has to be None, or density is a float number, then mass has to be None!')

        J = 2.*mass/5.*radius**2
        RigidBodyInertia.__init__(self, mass=mass,
                                  inertiaTensor=np.diag([J,J,J]),
                                  com=np.zeros(3))
        self.data = {'type':'InertiaSphere', 'mass':mass, 'radius':radius, 'density':density, 'volume':volume}
        
class InertiaHollowSphere(RigidBodyInertia):
    """create RigidBodyInertia with moment of inertia and mass of hollow sphere with mass (concentrated at circumference) and radius; inertia w.r.t. center of mass, com=0
    """
    def __init__(self, mass, radius):
        """initialize inertia with mass and (inner==outer) radius of hollow sphere
        """
        J = 2.*mass/3.*radius**2
        RigidBodyInertia.__init__(self, mass=mass,
                                  inertiaTensor=np.diag([J,J,J]),
                                  com=np.zeros(3))
        self.data = {'type':'InertiaHollowSphere', 'mass':mass, 'radius':radius}

class InertiaCylinder(RigidBodyInertia):
    """create RigidBodyInertia with moment of inertia and mass of cylinder with density, length and outerRadius; axis defines the orientation of the cylinder axis (0=x-axis, 1=y-axis, 2=z-axis); for hollow cylinder use innerRadius != 0; inertia w.r.t. center of mass, com=[0,0,0]
    """
    def __init__(self, density, length, outerRadius, axis, innerRadius=0):
        """initialize inertia with density, length, outer radius, axis (0=x-axis, 1=y-axis, 2=z-axis) and optional inner radius (for hollow cylinder)
        """
        m = density*length*np.pi*(outerRadius**2-innerRadius**2)
        Jaxis = 0.5*m*(outerRadius**2+innerRadius**2)
        Jtt = 1./12.*m*(3*(outerRadius**2+innerRadius**2)+length**2)

        if axis==0:
            RigidBodyInertia.__init__(self, mass=m,
                                      inertiaTensor=np.diag([Jaxis,Jtt,Jtt]),
                                      com=np.zeros(3))
        elif axis==1:
            RigidBodyInertia.__init__(self, mass=m,
                                      inertiaTensor=np.diag([Jtt,Jaxis,Jtt]),
                                      com=np.zeros(3))
        elif axis==2:
            RigidBodyInertia.__init__(self, mass=m,
                                      inertiaTensor=np.diag([Jtt,Jtt,Jaxis]),
                                      com=np.zeros(3))
        else:
            raise ValueError("InertiaCylinder: axis must be 0, 1 or 2!")

        self.data = {'type':'InertiaCylinder', 
                     'density':density, 'length':length, 'axis':axis, 
                     'outerRadius':outerRadius, 'innerRadius':innerRadius}
        

def StrNodeType2NodeType(sNodeType):
    """convert string into exudyn.NodeType; call e.g. with 'NodeType.RotationEulerParameters' or 'RotationEulerParameters'

    Note:
        function is not very fast, so should be avoided in time-critical situations
    """
    s = str(sNodeType) #if called with type
    s = s.replace('NodeType.','')
    nodeTypes = exu.NodeType.__members__
    if s in nodeTypes:
        return nodeTypes[s]
    else:
        raise ValueError('StrNodeType2NodeType: no valid NodeType: "'+s+'"')
    # for key in nodeTypes:
    #     if s == str(key) or s == str(nodeTypes[key]):
    #         return int(nodeTypes[key])
    
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def GetRigidBodyNode(nodeType, 
                 position=[0,0,0], 
                 velocity=[0,0,0], 
                 rotationMatrix= [],
                 rotationParameters = [],
                 angularVelocity=[0,0,0]):
    """get node item interface according to nodeType, using initialization with position, velocity, angularVelocity and rotationMatrix

    Args:
        nodeType: a node type according to exudyn.NodeType, or a string of it, e.g., 'NodeType.RotationEulerParameters' (fastest, but additional algebraic constraint equation), 'NodeType.RotationRxyz' (Tait-Bryan angles, singularity for second angle at +/- 90 degrees), 'NodeType.RotationRotationVector' (used for Lie group integration)
        position: reference position as list or numpy array with 3 components (in global/world frame)
        velocity: initial translational velocity as list or numpy array with 3 components (in global/world frame)
        rotationMatrix: 3x3 list or numpy matrix to define reference rotation; use EITHER rotationMatrix=[[...],[...],[...]] (while rotationParameters=[]) or rotationParameters=[...] (while rotationMatrix=[])
        rotationParameters: reference rotation parameters; use EITHER rotationMatrix=[[...],[...],[...]] (while rotationParameters=[]) or rotationParameters=[...] (while rotationMatrix=[])
        angularVelocity: initial angular velocity as list or numpy array with 3 components (in global/world frame)

    Returns:
        returns list containing node number and body number: [nodeNumber, bodyNumber]
    """

    rotationMatrixNew = copy.copy(rotationMatrix)
    if len(rotationMatrixNew) != 0 and len(rotationParameters) != 0:
        raise ValueError('GetRigidBodyNode: either rotationMatrixNew or rotationParameters must empty!')
    if len(rotationMatrixNew) == 0 and len(rotationParameters) == 0:
        rotationMatrixNew=np.eye(3)

    strNodeType = str(nodeType) #works both for nodeType and for strings (if exudyn not available)

    nodeItem = []
    if strNodeType == 'NodeType.RotationEulerParameters':
        if len(rotationParameters) == 0:
            ep0 = RotationMatrix2EulerParameters(rotationMatrixNew)
        else:
            ep0 = rotationParameters
           
        ep_t0 = AngularVelocity2EulerParameters_t(angularVelocity, ep0)
        nodeItem = eii.NodeRigidBodyEP(referenceCoordinates=list(position)+list(ep0),
                                   initialVelocities=list(velocity)+list(ep_t0))       
    elif strNodeType == 'NodeType.RotationRxyz':
        if len(rotationParameters) == 0:
            rot0 = RotationMatrix2RotXYZ(rotationMatrixNew)
        else:
            rot0 = rotationParameters

        rot_t0 = AngularVelocity2RotXYZ_t(angularVelocity, rot0)
        nodeItem = eii.NodeRigidBodyRxyz(referenceCoordinates=list(position)+list(rot0),
                                     initialVelocities=list(velocity)+list(rot_t0))
    elif strNodeType == 'NodeType.RotationRotationVector':
        if len(rotationParameters) == 0:
            #raise ValueError('NodeType.RotationRotationVector not implemented!')
            rot0 = RotationMatrix2RotationVector(rotationMatrixNew)
        else:
            rot0 = rotationParameters
        
        rotMatrix = RotationVector2RotationMatrix(rot0) #rotationMatrixNew needed!
        angularVelocityLocal = np.dot(rotMatrix.transpose(),angularVelocity)
            
        nodeItem = eii.NodeRigidBodyRotVecLG(referenceCoordinates=list(position) + list(rot0), 
                                         initialVelocities=list(velocity)+list(angularVelocityLocal))
        
    elif strNodeType == 'NodeType.LieGroupWithDirectUpdate':
        if len(rotationParameters) == 0:
            #raise ValueError('NodeType.RotationRotationVector not implemented!')
            rot0 = RotationMatrix2RotationVector(rotationMatrixNew)
        else:
            rot0 = rotationParameters
        
        rotMatrix = RotationVector2RotationMatrix(rot0) #rotationMatrixNew needed!
        angularVelocityLocal = np.dot(rotMatrix.transpose(),angularVelocity)
            
        nodeItem = eii.NodeRigidBodyRotVecLG(referenceCoordinates=list(position) + list(rot0), 
                                         initialVelocities=list(velocity)+list(angularVelocityLocal))  
        
    # elif strNodeType == 'NodeType.LieGroupWithDataCoordinates':
    #     if len(rotationParameters) == 0:
    #         #raise ValueError('NodeType.RotationRotationVector not implemented!')
    #         rot0 = RotationMatrix2RotationVector(rotationMatrixNew)
    #     else:
    #         rot0 = rotationParameters
        
    #     rotMatrix = RotationVector2RotationMatrix(rot0) #rotationMatrixNew needed!
    #     angularVelocityLocal = np.dot(rotMatrix.transpose(),angularVelocity)
            
    #     nodeItem = eii.NodeRigidBodyRotVecDataLG(referenceCoordinates=list(position) + list(rot0),
    #                                                     initialCoordinates=list(position)+list(rot0), #initializes data coordinates
    #                                                     initialVelocities=list(velocity)+list(angularVelocityLocal))  
        
    else:
        raise ValueError("GetRigidBodyNode: invalid node type:"+strNodeType)

    return nodeItem

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def AddRigidBody(mainSys, inertia, 
                 nodeType = exu.NodeType.RotationEulerParameters, 
                 position=[0,0,0], velocity=[0,0,0], 
                 rotationMatrix= [],
                 rotationParameters = [],
                 angularVelocity=[0,0,0],
                 gravity=[0,0,0],
                 graphicsDataList=[]):
    """DEPRECATED: adds a node (with str(exu.NodeType. ...)) and body for a given rigid body; all quantities (esp. velocity and angular velocity) are given in global coordinates!

    Args:
        inertia: an inertia object as created by class RigidBodyInertia; containing mass, COM and inertia
        nodeType: a node type according to exudyn.NodeType, or a string of it, e.g., 'NodeType.RotationEulerParameters' (fastest, but additional algebraic constraint equation), 'NodeType.RotationRxyz' (Tait-Bryan angles, singularity for second angle at +/- 90 degrees), 'NodeType.RotationRotationVector' (used for Lie group integration)
        position: reference position as list or numpy array with 3 components (in global/world frame)
        velocity: initial translational velocity as list or numpy array with 3 components (in global/world frame)
        rotationMatrix: 3x3 list or numpy matrix to define reference rotation; use EITHER rotationMatrix=[[...],[...],[...]] (while rotationParameters=[]) or rotationParameters=[...] (while rotationMatrix=[])
        rotationParameters: reference rotation parameters; use EITHER rotationMatrix=[[...],[...],[...]] (while rotationParameters=[]) or rotationParameters=[...] (while rotationMatrix=[])
        angularVelocity: initial angular velocity as list or numpy array with 3 components (in global/world frame)
        gravity: if provided as list or numpy array with 3 components, it adds gravity force to the body at the COM, i.e., fAdd = m*gravity
        graphicsDataList: list of graphicsData objects to define appearance of body

    Returns:
        returns list containing node number and body number: [nodeNumber, bodyNumber]

    Note:
        DEPRECATED and will be removed; use MainSystem.CreateRigidBody(...) instead!
    """

    rotationMatrixNew = copy.copy(rotationMatrix)

    if not isinstance(inertia, RigidBodyInertia): #do not use 'exu.rigidBodyUtilities.' in front, even not outside of module!
        RaiseTypeError(where='AddRigidBody', argumentName='inertia', received = inertia, expectedType = ExpectedType.RigidBodyInertia, dim=None)
    #MISSING: check for nodeType
    if not IsVector(position, 3):
        RaiseTypeError(where='AddRigidBody', argumentName='position', received = position, expectedType = ExpectedType.Vector, dim=3)
    if not IsVector(velocity, 3):
        RaiseTypeError(where='AddRigidBody', argumentName='velocity', received = velocity, expectedType = ExpectedType.Vector, dim=3)
    if not IsVector(angularVelocity, 3):
        RaiseTypeError(where='AddRigidBody', argumentName='angularVelocity', received = angularVelocity, expectedType = ExpectedType.Vector, dim=3)
    if not IsVector(gravity, 3):
        RaiseTypeError(where='AddRigidBody', argumentName='gravity', received = gravity, expectedType = ExpectedType.Vector, dim=3)

    if type(graphicsDataList) != list:
        raise ValueError('AddRigidBody: graphicsDataList must be a (possibly empty) list of dictionaries of graphics data!')


    if not IsSquareMatrix(rotationMatrixNew):
        raise ValueError('AddRigidBody: rotationMatrix must be a (possibly empty) list or numpy array!')
    if not IsVector(rotationParameters):
        raise ValueError('AddRigidBody: rotationParameters must be a (possibly empty) list or numpy array!')
    
    if len(rotationMatrixNew) != 0 and len(rotationParameters) != 0:
        raise ValueError('AddRigidBody: either rotationMatrix or rotationParameters must be empty list or numpy array!')
    if len(rotationMatrixNew) == 0 and len(rotationParameters) == 0:
        rotationMatrixNew=np.eye(3)
    else:
        if len(rotationMatrixNew) == 0:
            expectedSize = 3
            if str(nodeType) == 'NodeType.RotationEulerParameters': 
                expectedSize = 4
            if not IsVector(rotationParameters, expectedSize):
                RaiseTypeError(where='AddRigidBody', argumentName='rotationParameters', received = rotationParameters, expectedType = ExpectedType.Vector, dim=expectedSize)
        else:
            if not IsSquareMatrix(rotationMatrixNew, 3):
                RaiseTypeError(where='AddRigidBody', argumentName='rotationMatrix', received = rotationMatrixNew, expectedType = ExpectedType.Matrix, dim=3)
            
            
    nodeItem = GetRigidBodyNode(nodeType, position, velocity, rotationMatrixNew, rotationParameters, angularVelocity)
    nodeNumber = mainSys.AddNode(nodeItem)
    
    bodyNumber = mainSys.AddObject(eii.ObjectRigidBody(physicsMass=inertia.mass, physicsInertia=inertia.GetInertia6D(), 
                                                   physicsCenterOfMass=inertia.com,
                                                   nodeNumber=nodeNumber, 
                                                   visualization=eii.VObjectRigidBody(graphicsData=graphicsDataList)))
    
    if np.linalg.norm(gravity) != 0.:
        markerNumber = mainSys.AddMarker(eii.MarkerBodyMass(bodyNumber=bodyNumber))
        mainSys.AddLoad(eii.LoadMassProportional(markerNumber=markerNumber, loadVector=gravity))
    
    return [nodeNumber, bodyNumber]


def AddRevoluteJoint(mbs, body0, body1, point, axis, useGlobalFrame=True, 
                     showJoint=True, axisRadius=0.1, axisLength=0.4):
    """DEPRECATED (use MainSystem function instead): add revolute joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed

    Args:
        mbs: the MainSystem to which the joint and markers shall be added
        body0: a object number for body0, must be rigid body or ground object
        body1: a object number for body1, must be rigid body or ground object
        point: a 3D vector as list or np.array containing the global center point of the joint in reference configuration
        axis: a 3D vector as list or np.array containing the global rotation axis of the joint in reference configuration
        useGlobalFrame: if False, the point and axis vectors are defined in the local coordinate system of body0

    Returns:
        returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint

    Note:
        DEPRECATED and will be removed; use MainSystem.CreateRevoluteJoint(...) instead!
    """

    exu.Print('WARNING: AddRevoluteJoint is deprecated; use mbs.CreateRevoluteJoint instead!')
    
    #perform some checks:
    if not IsValidObjectIndex(body0):
        RaiseTypeError(where='AddRevoluteJoint', argumentName='body0', received = body0, expectedType = ExpectedType.ObjectIndex)
    if not IsValidObjectIndex(body1):
        RaiseTypeError(where='AddRevoluteJoint', argumentName='body1', received = body1, expectedType = ExpectedType.ObjectIndex)
        
    if not IsVector(point, 3):
        RaiseTypeError(where='AddRevoluteJoint', argumentName='point', received = point, expectedType = ExpectedType.Vector, dim=3)
    if not IsVector(axis, 3):
        RaiseTypeError(where='AddRevoluteJoint', argumentName='axis', received = axis, expectedType = ExpectedType.Vector, dim=3)

    if not IsValidBool(useGlobalFrame):
        RaiseTypeError(where='AddRevoluteJoint', argumentName='useGlobalFrame', received = useGlobalFrame, expectedType = ExpectedType.Bool)
    if not IsValidBool(showJoint):
        RaiseTypeError(where='AddRevoluteJoint', argumentName='showJoint', received = showJoint, expectedType = ExpectedType.Bool)

    if not IsValidRealInt(axisRadius):
        RaiseTypeError(where='AddRevoluteJoint', argumentName='axisRadius', received = axisRadius, expectedType = ExpectedType.Real)
    if not IsValidRealInt(axisLength):
        RaiseTypeError(where='AddRevoluteJoint', argumentName='axisLength', received = axisLength, expectedType = ExpectedType.Real)

    p0 = mbs.GetObjectOutputBody(body0,exu.OutputVariableType.Position,
                                 localPosition=[0,0,0],
                                 configuration=exu.ConfigurationType.Reference)
    A0 = mbs.GetObjectOutputBody(body0,exu.OutputVariableType.RotationMatrix,
                                 localPosition=[0,0,0],
                                 configuration=exu.ConfigurationType.Reference).reshape((3,3))
    p1 = mbs.GetObjectOutputBody(body1,exu.OutputVariableType.Position,
                                 localPosition=[0,0,0],
                                 configuration=exu.ConfigurationType.Reference)
    A1 = mbs.GetObjectOutputBody(body1,exu.OutputVariableType.RotationMatrix,
                                 localPosition=[0,0,0],
                                 configuration=exu.ConfigurationType.Reference).reshape((3,3))

    if useGlobalFrame:
        pJoint = point
        vAxis = copy.copy(axis)
    else: #transform into global coordinates, then everything works same
        pJoint = A0 @ point + p0
        vAxis = A0 @ axis

    #compute joint frame (not unique, only rotation axis must coincide)
    B = ComputeOrthonormalBasis(vAxis) #axis = x-axis
    #interchange z and x axis (needs sign change, otherwise det(A)=-1)
    AJ = np.eye(3)
    AJ[:,0]=-B[:,2]
    AJ[:,1]= B[:,1]
    AJ[:,2]= B[:,0] #axis ==> rotation axis z for revolute joint ... 
    
    #compute joint position and axis in body0 / 1 coordinates:
    pJ0 = A0.T @ (np.array(pJoint) - p0)
    pJ1 = A1.T @ (np.array(pJoint) - p1)

    #compute joint marker orientations:
    MR0 = A0.T @ AJ  
    MR1 = A1.T @ AJ  
    
    mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(bodyNumber=body0, localPosition=pJ0))
    mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(bodyNumber=body1, localPosition=pJ1))
    
    oJoint = mbs.AddObject(eii.ObjectJointRevoluteZ(markerNumbers=[mBody0,mBody1],
                                                rotationMarker0=MR0,
                                                rotationMarker1=MR1,
             visualization=eii.VRevoluteJointZ(show=showJoint, axisRadius=axisRadius, axisLength=axisLength) ))

    return [oJoint, mBody0, mBody1]


def AddPrismaticJoint(mbs, body0, body1, point, axis, useGlobalFrame=True, 
                     showJoint=True, axisRadius=0.1, axisLength=0.4):
    """DEPRECATED (use MainSystem function instead): add prismatic joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed

    Args:
        mbs: the MainSystem to which the joint and markers shall be added
        body0: a object number for body0, must be rigid body or ground object
        body1: a object number for body1, must be rigid body or ground object
        point: a 3D vector as list or np.array containing the global center point of the joint in reference configuration
        axis: a 3D vector as list or np.array containing the global translation axis of the joint in reference configuration
        useGlobalFrame: if False, the point and axis vectors are defined in the local coordinate system of body0

    Returns:
        returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint

    Note:
        DEPRECATED and will be removed; use MainSystem.CreatePrismaticJoint(...) instead!
    """

    exu.Print('WARNING: AddPrismaticJoint is deprecated; use mbs.CreateRevoluteJoint instead!')
        
    if not IsValidObjectIndex(body0):
        RaiseTypeError(where='AddPrismaticJoint', argumentName='body0', received = body0, expectedType = ExpectedType.ObjectIndex)
    if not IsValidObjectIndex(body1):
        RaiseTypeError(where='AddPrismaticJoint', argumentName='body1', received = body1, expectedType = ExpectedType.ObjectIndex)
        
    if not IsVector(point, 3):
        RaiseTypeError(where='AddPrismaticJoint', argumentName='point', received = point, expectedType = ExpectedType.Vector, dim=3)
    if not IsVector(axis, 3):
        RaiseTypeError(where='AddPrismaticJoint', argumentName='axis', received = axis, expectedType = ExpectedType.Vector, dim=3)

    if not IsValidBool(useGlobalFrame):
        RaiseTypeError(where='AddPrismaticJoint', argumentName='useGlobalFrame', received = useGlobalFrame, expectedType = ExpectedType.Bool)
    if not IsValidBool(showJoint):
        RaiseTypeError(where='AddPrismaticJoint', argumentName='showJoint', received = showJoint, expectedType = ExpectedType.Bool)

    if not IsValidRealInt(axisRadius):
        RaiseTypeError(where='AddPrismaticJoint', argumentName='axisRadius', received = axisRadius, expectedType = ExpectedType.Real)
    if not IsValidRealInt(axisLength):
        RaiseTypeError(where='AddPrismaticJoint', argumentName='axisLength', received = axisLength, expectedType = ExpectedType.Real)

    p0 = mbs.GetObjectOutputBody(body0,exu.OutputVariableType.Position,
                                 localPosition=[0,0,0],
                                 configuration=exu.ConfigurationType.Reference)
    A0 = mbs.GetObjectOutputBody(body0,exu.OutputVariableType.RotationMatrix,
                                 localPosition=[0,0,0],
                                 configuration=exu.ConfigurationType.Reference).reshape((3,3))
    p1 = mbs.GetObjectOutputBody(body1,exu.OutputVariableType.Position,
                                 localPosition=[0,0,0],
                                 configuration=exu.ConfigurationType.Reference)
    A1 = mbs.GetObjectOutputBody(body1,exu.OutputVariableType.RotationMatrix,
                                 localPosition=[0,0,0],
                                 configuration=exu.ConfigurationType.Reference).reshape((3,3))

    if useGlobalFrame:
        pJoint = point
        vAxis = copy.copy(axis)
    else: #transform into global coordinates, then everything works same
        pJoint = A0 @ point + p0
        vAxis = A0 @ axis

    #compute joint frame (not unique, only rotation axis must coincide)
    AJ = ComputeOrthonormalBasis(vAxis) #axis = x-axis
    
    #compute joint position and axis in body0 / 1 coordinates:
    pJ0 = A0.T @ (np.array(pJoint) - p0)
    pJ1 = A1.T @ (np.array(pJoint) - p1)

    #compute joint marker orientations:
    MR0 = A0.T @ AJ  
    MR1 = A1.T @ AJ  
    
    mBody0 = mbs.AddMarker(eii.MarkerBodyRigid(bodyNumber=body0, localPosition=pJ0))
    mBody1 = mbs.AddMarker(eii.MarkerBodyRigid(bodyNumber=body1, localPosition=pJ1))
    
    oJoint = mbs.AddObject(eii.ObjectJointPrismaticX(markerNumbers=[mBody0,mBody1],
                                                rotationMarker0=MR0,
                                                rotationMarker1=MR1,
             visualization=eii.VPrismaticJointX(show=showJoint, axisRadius=axisRadius, axisLength=axisLength) ))

    return [oJoint, mBody0, mBody1]




