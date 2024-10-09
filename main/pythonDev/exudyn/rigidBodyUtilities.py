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
import numpy as np #LoadSolutionFile
import exudyn.itemInterface as eii
import exudyn as exu 
from exudyn.basicUtilities import NormL2
from exudyn.advancedUtilities import ExpectedType, RaiseTypeError, IsValidBool, IsValidRealInt, IsVector, IsSquareMatrix, IsValidObjectIndex
from math import sin, cos #, sqrt, atan2

import copy

eulerParameters0 = [1.,0.,0.,0.] #Euler parameters for case where rotation angle is zero (rotation axis arbitrary)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute orthogonal basis vectors (normal1, normal2) for given vector0 (non-unique solution!); the length of vector0 must not be 1; if vector0 == [0,0,0], then any normal basis is returned
#**output: returns [vector0normalized, normal1, normal2], in which vector0normalized is the normalized vector0 (has unit length); all vectors in numpy array format
def ComputeOrthonormalBasisVectors(vector0):
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
    #print("basis=", v,n1,n2)
    return [v, n1, n2]

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute orthogonal basis, in which the normalized vector0 is the first column and the other columns are normals to vector0 (non-unique solution!); the length of vector0 must not be 1; if vector0 == [0,0,0], then any normal basis is returned
#**output: returns A, a rotation matrix, in which the first column is parallel to vector0; A is a 2D numpy array
def ComputeOrthonormalBasis(vector0):
    return np.vstack(ComputeOrthonormalBasisVectors(vector0)).T
    

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute Gram-Schmidt projection of given 3D vector 1 on vector 0 and return normalized triad (vector0, vector1, vector0 x vector1)
def GramSchmidt(vector0, vector1):

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
    #print("basis=", v,n1,n2)
    return [v0, v1, n2]


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute skew symmetric 3x3-matrix from 3x1- or 1x3-vector
def Skew(vector):
    skewsymmetricMatrix = np.array([[ 0.,       -vector[2], vector[1]], 
                                    [ vector[2], 0.,       -vector[0]],
                                    [-vector[1], vector[0], 0.]])
    return skewsymmetricMatrix

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: convert skew symmetric matrix m to vector
def Skew2Vec(skew):
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
#**function: compute skew matrix from vector or matrix; used for ObjectFFRF and CMS implementation
#**input: a vector v in np.array format, containing 3*n components or a matrix with m columns of same shape
#**output: if v is a vector, output is (3*n x 3) skew matrix in np.array format; if v is a (n x m) matrix, the output is a (3*n x m) skew matrix in np.array format
def ComputeSkewMatrix(v):
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
        print("ERROR: wrong dimension in ComputeSkewMatrix(...)")
    return sm

#tests for ComputeSkewMatrix
#x = np.array([1,2,3,4,5,6])
#print(ComputeSkewMatrix(x))
#x = np.array([[1,2],[3,4],[5,6],[1,2],[3,4],[5,6]])
#print(ComputeSkewMatrix(x))


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
#**function: convert Euler parameters (ep) to G-matrix (=$\partial \tomega  / \partial \pv_t$)
#**input: vector of 4 eulerParameters as list or np.array
#**output: 3x4 matrix G as np.array
def EulerParameters2G(eulerParameters):
    ep = eulerParameters
    return np.array([[-2.*ep[1], 2.*ep[0],-2.*ep[3], 2.*ep[2]],
                     [-2.*ep[2], 2.*ep[3], 2.*ep[0],-2.*ep[1]],
                     [-2.*ep[3],-2.*ep[2], 2.*ep[1], 2.*ep[0]] ])

#**function: convert Euler parameters (ep) to local G-matrix (=$\partial \LU{b}{\tomega} / \partial \pv_t$)
#**input: vector of 4 eulerParameters as list or np.array
#**output: 3x4 matrix G as np.array
def EulerParameters2GLocal(eulerParameters):
    ep = eulerParameters
    return np.array([[-2.*ep[1], 2.*ep[0], 2.*ep[3],-2.*ep[2]],
                     [-2.*ep[2],-2.*ep[3], 2.*ep[0], 2.*ep[1]],
                     [-2.*ep[3], 2.*ep[2],-2.*ep[1], 2.*ep[0]] ])

#**function: compute rotation matrix from eulerParameters    
#**input: vector of 4 eulerParameters as list or np.array
#**output: 3x3 rotation matrix as np.array
def EulerParameters2RotationMatrix(eulerParameters):
    ep = eulerParameters
    return np.array([[-2.0*ep[3]*ep[3] - 2.0*ep[2]*ep[2] + 1.0, -2.0*ep[3]*ep[0] + 2.0*ep[2]*ep[1], 2.0*ep[3]*ep[1] + 2.0*ep[2]*ep[0]],
                     [ 2.0*ep[3]*ep[0] + 2.0*ep[2]*ep[1], -2.0*ep[3]*ep[3] - 2.0*ep[1]*ep[1] + 1.0, 2.0*ep[3]*ep[2] - 2.0*ep[1]*ep[0]],
                     [-2.0*ep[2]*ep[0] + 2.0*ep[3]*ep[1], 2.0*ep[3]*ep[2] + 2.0*ep[1]*ep[0], -2.0*ep[2]*ep[2] - 2.0*ep[1]*ep[1] + 1.0] ])

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute Euler parameters from given rotation matrix
#**input: 3x3 rotation matrix as list of lists or as np.array
#**output: vector of 4 eulerParameters as np.array
def RotationMatrix2EulerParameters(rotationMatrix):
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
#**function: compute time derivative of Euler parameters from (global) angular velocity vector
#note that for Euler parameters $\pv$, we have $\tomega=\Gm \dot \pv$ ==> $\Gm^T \tomega = \Gm^T\cdot \Gm\cdot \dot \pv$ ==> $\Gm^T \Gm=4(\Im_{4 \times 4} - \pv\cdot \pv^T)\dot\pv = 4 (\Im_{4x4}) \dot \pv$
#**input: 
#  angularVelocity: 3D vector of angular velocity in global frame, as lists or as np.array
#  eulerParameters: vector of 4 eulerParameters as np.array or list
#**output: vector of time derivatives of 4 eulerParameters as np.array
def AngularVelocity2EulerParameters_t(angularVelocity, eulerParameters):
    
    GT = np.transpose(EulerParameters2G(eulerParameters))
    return 0.25*(GT.dot(angularVelocity))


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#            ROTATION VECTOR
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: rotaton matrix from rotation vector, see appendix B in \cite{Simo1988}
#**input: 3D rotation vector as list or np.array
#**output: 3x3 rotation matrix as np.array
#**notes: gets inaccurate for very large rotations, $\phi \\gg 2*\pi$
def RotationVector2RotationMatrix(rotationVector):
    phi = np.linalg.norm(rotationVector)
    if phi == 0.:
        R = np.eye(3)
    else:
        OmegaSkew = Skew(rotationVector)
        alpha = np.sin(phi)/phi
        beta = 2*(1-np.cos(phi))/phi**2 #the loss of digits in 1-np.cos(phi) is compensated by OmegaSkew@OmegaSkew
        R = np.eye(3) + alpha*OmegaSkew + 0.5*beta*np.matmul(OmegaSkew, OmegaSkew)

    
    return R  


#**function: compute rotation vector from rotation matrix
#**input: 3x3 rotation matrix as list of lists or as np.array
#**output: vector of 3 components of rotation vector as np.array
def RotationMatrix2RotationVector(rotationMatrix):
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


#**function: compute rotation axis from given rotation vector
#**input: 3D rotation vector as np.array
#**output: 3D vector as np.array representing the rotation axis
def ComputeRotationAxisFromRotationVector(rotationVector):
    
    # compute rotation angle
    rotationAngle = np.linalg.norm(rotationVector)
    
    # compute rotation axis
    if rotationAngle == 0.0:
        rotationAxis = np.zeros(3)
    else:
        rotationAxis = rotationVector/rotationAngle
    
    # return rotation axis 
    return rotationAxis


#**function: convert rotation vector (parameters) (v) to G-matrix (=$\partial \tomega  / \partial \dot \vv$)
#**input: vector of rotation vector (len=3) as list or np.array
#**output: 3x3 matrix G as np.array
def RotationVector2G(rotationVector):
    return RotationVector2RotationMatrix(rotationVector)

#**function: convert rotation vector (parameters) (v) to local G-matrix (=$\partial \LU{b}{\tomega}   / \partial \vv_t$)
#**input: vector of rotation vector (len=3) as list or np.array
#**output: 3x3 matrix G as np.array
def RotationVector2GLocal(eulerParameters):
    return np.eye(3)



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#            TAIT BRYAN ANGLES
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#**function: compute rotation matrix from consecutive xyz \acp{Rot} (Tait-Bryan angles); A=Ax*Ay*Az; rot=[rotX, rotY, rotZ]
#**input: 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
#**output: 3x3 rotation matrix as np.array
def RotXYZ2RotationMatrix(rot):
    c0 = np.cos(rot[0])
    s0 = np.sin(rot[0])
    c1 = np.cos(rot[1])
    s1 = np.sin(rot[1])
    c2 = np.cos(rot[2])
    s2 = np.sin(rot[2])
    
    return np.array([[ c1*c2           ,-c1*s2           , s1    ],
                     [ s0*s1*c2 + c0*s2,-s0*s1*s2 + c0*c2,-s0*c1 ],
                     [-c0*s1*c2 + s0*s2, c0*s1*s2 + s0*c2, c0*c1 ]]);

#**function: convert rotation matrix to xyz Euler angles (Tait-Bryan angles);  A=Ax*Ay*Az; 
#**input:  3x3 rotation matrix as list of lists or np.array
#**output: vector of Tait-Bryan rotation parameters [X,Y,Z] (in radiant) as np.array
#**notes: due to gimbal lock / singularity at rot[1] = pi/2, -pi/2, ... the reconstruction of 
#  \texttt{RotationMatrix2RotXYZ( RotXYZ2RotationMatrix(rot) )} may fail, but 
#  \texttt{RotXYZ2RotationMatrix( RotationMatrix2RotXYZ( RotXYZ2RotationMatrix(rot) ) )} works always
def RotationMatrix2RotXYZ(rotationMatrix):
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


#**function: compute (global-frame) G-matrix for xyz Euler angles (Tait-Bryan angles) ($\LU{0}{\Gm} = \partial \LU{0}{\tomega}  / \partial \dot \ttheta$)
#**input:  3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
#**output: 3x3 matrix G as np.array
def RotXYZ2G(rot):
    c0 = cos(rot[0])
    s0 = sin(rot[0])
    c1 = cos(rot[1])
    s1 = sin(rot[1])

    return np.array([[1, 0, s1],
                     [0, c0, -c1*s0],
                     [0, s0,  c0*c1 ]])

#**function: compute time derivative of (global-frame) G-matrix for xyz Euler angles (Tait-Bryan angles) ($\LU{0}{\Gm} = \partial \LU{0}{\tomega}  / \partial \dot \ttheta$)
#**input:  
#    rot: 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
#    rot_t: 3D vector of time derivative of Tait-Bryan rotation parameters [X,Y,Z] in radiant/s
#**output: 3x3 matrix G\_t as np.array
def RotXYZ2G_t(rot, rot_t):
    c0 = cos(rot[0])
    s0 = sin(rot[0])
    c1 = cos(rot[1])
    s1 = sin(rot[1])

    return np.array([[0, 0, rot_t[1]*c1],
                     [0, -rot_t[0]*s0, rot_t[1]*s0*s1 - rot_t[0]*c0*c1],
                     [0, rot_t[0]*c0, -rot_t[0]*c1*s0 - rot_t[1]*c0*s1]])


#**function: compute local (body-fixed) G-matrix for xyz Euler angles (Tait-Bryan angles) ($\LU{b}{\Gm} = \partial \LU{b}{\tomega}  / \partial \ttheta_t$)
#**input:  3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
#**output: 3x3 matrix GLocal as np.array
def RotXYZ2GLocal(rot):
    c1 = cos(rot[1])
    s1 = sin(rot[1])
    c2 = cos(rot[2])
    s2 = sin(rot[2])

    return np.array([[ c1*c2, s2, 0],
                     [-c1*s2, c2, 0],
                     [ s1,     0,  1]])

#**function: compute time derivative of (body-fixed) G-matrix for xyz Euler angles (Tait-Bryan angles) ($\LU{b}{\Gm} = \partial \LU{b}{\tomega}  / \partial \ttheta_t$)
#**input:  
#    rot: 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
#    rot_t: 3D vector of time derivative of Tait-Bryan rotation parameters [X,Y,Z] in radiant/s
#**output: 3x3 matrix GLocal\_t as np.array
def RotXYZ2GLocal_t(rot, rot_t):
    c1 = cos(rot[1])
    s1 = sin(rot[1])
    c2 = cos(rot[2])
    s2 = sin(rot[2])

    return np.array([[-rot_t[2]*c1*s2 - rot_t[1]*c2*s1, rot_t[2]*c2, 0],
                     [ rot_t[1]*s2*s1 - rot_t[2]*c2*c1, -rot_t[2]*s2, 0],
                     [ rot_t[1]*c1, 0, 0 ]])






#**function: compute time derivatives of angles RotXYZ from (global) angular velocity vector and given rotation
#**input:  
#  angularVelocity: global angular velocity vector as list or np.array
#  rotation: 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
#**output: time derivative of vector of Tait-Bryan rotation parameters [X,Y,Z] (in radiant) as np.array
def AngularVelocity2RotXYZ_t(angularVelocity, rotation):
    psi = rotation[0]
    theta = rotation[1]
    #phi = rotation[2] #not needed
    cTheta = np.cos(theta)
    if cTheta == 0:
        print('AngularVelocity2RotXYZ_t: not possible for rotation[1] == pi/2, 3*pi/2, ...')

    GInv = (1/cTheta)*np.array([[np.cos(theta), np.sin(psi)*np.sin(theta),-np.cos(psi)*np.sin(theta)],
                                [0            , np.cos(psi)*np.cos(theta)   , np.sin(psi)*np.cos(theta)],
                                [0            ,-np.sin(psi)              , np.cos(psi)]])
    return np.dot(GInv,angularVelocity)
  
    
#**function: compute four Euler parameters from given RotXYZ angles, see \cite{Henderson1977}
#**input: 
#   alpha: 3D vector as np.array containing RotXYZ angles
#**output: 4D vector as np.array containing four Euler parameters 
#          entry zero of output represent the scalar part of Euler parameters
def RotXYZ2EulerParameters(alpha):
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

#**function: convert rotation matrix to zyz Euler angles;  A=Az*Ay*Az;
#**input:
#  rotationMatrix: 3x3 rotation matrix as list of lists or np.array
#  flip:           argument to choose first Euler angle to be in quadrant 2 or 3.
#**output: vector of Euler rotation parameters [Z,Y,Z] (in radiant) as np.array
#**notes: tested (compared with Robotics, Vision and Control book of P. Corke)
#**author: Martin Sereinig
def RotationMatrix2RotZYZ(rotationMatrix, flip):
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
#**function: compute rotation matrix w.r.t. X-axis (first axis)
#**input: angle around X-axis in radiant
#**output: 3x3 rotation matrix as np.array
def RotationMatrixX(angleRad):
    return np.array([[1, 0, 0],
                     [0, np.cos(angleRad),-np.sin(angleRad)],
                     [0, np.sin(angleRad), np.cos(angleRad)] ])

#**function: compute rotation matrix w.r.t. Y-axis (second axis)
#**input: angle around Y-axis in radiant
#**output: 3x3 rotation matrix as np.array
def RotationMatrixY(angleRad):
    return np.array([ [ np.cos(angleRad), 0, np.sin(angleRad)],
                      [0,        1, 0],
                      [-np.sin(angleRad),0, np.cos(angleRad)] ])

#**function: compute rotation matrix w.r.t. Z-axis (third axis)
#**input: angle around Z-axis in radiant
#**output: 3x3 rotation matrix as np.array
def RotationMatrixZ(angleRad):
    return np.array([ [np.cos(angleRad),-np.sin(angleRad), 0],
                      [np.sin(angleRad), np.cos(angleRad), 0],
                      [0,        0,        1] ]);

    
#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#functions for homogeneous transformations (HT)
#**function: compute \ac{HT} matrix from rotation matrix A and translation vector r
def HomogeneousTransformation(A, r):
    T = np.zeros((4,4))
    T[0:3,0:3] = A
    T[0:3,3] = r
    T[3,3] = 1
    return T

HT = HomogeneousTransformation #shortcut

#**function: \ac{HT} for translation with vector r
def HTtranslate(r):
    T = np.eye(4)
    T[0:3,3] = r
    return T

#**function: \ac{HT} for translation along x axis with value x
def HTtranslateX(x):
    T = np.eye(4)
    T[0,3] = x
    return T

#**function: \ac{HT} for translation along y axis with value y
def HTtranslateY(y):
    T = np.eye(4)
    T[1,3] = y
    return T

#**function: \ac{HT} for translation along z axis with value z
def HTtranslateZ(z):
    T = np.eye(4)
    T[2,3] = z
    return T

#**function: identity \ac{HT}:
def HT0():
    return np.eye(4)

#**function: \ac{HT} for rotation around axis X (first axis)
def HTrotateX(angle):
    T = np.eye(4)
    T[0:3,0:3] = RotationMatrixX(angle)
    return T
    
#**function: \ac{HT} for rotation around axis X (first axis)
def HTrotateY(angle):
    T = np.eye(4)
    T[0:3,0:3] = RotationMatrixY(angle)
    return T
    
#**function: \ac{HT} for rotation around axis X (first axis)
def HTrotateZ(angle):
    T = np.eye(4)
    T[0:3,0:3] = RotationMatrixZ(angle)
    return T

#**function: return translation part of \ac{HT}
def HT2translation(T):
    return T[0:3,3]

#**function: return rotation matrix of \ac{HT}
def HT2rotationMatrix(T):
    return T[0:3,0:3]

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: return inverse \ac{HT} such that inv(T)*T = np.eye(4)
def InverseHT(T):
    Tinv = np.eye(4)
    Ainv = T[0:3,0:3].T #inverse rotation part
    Tinv[0:3,0:3] = Ainv
    r = T[0:3,3]        #translation part
    Tinv[0:3,3]  = -Ainv @ r       #inverse translation part
    return Tinv

################################################################################
#Test (compared with Robotcs, Vision and Control book of P. Corke:
#T=HTtranslate([1,0,0]) @ HTrotateX(np.pi/2) @ HTtranslate([0,1,0])
#print("T=",T.round(8))
#
#R = RotationMatrixZ(0.1) @ RotationMatrixY(0.2) @ RotationMatrixZ(0.3) 
#print("R=",R.round(4))

################################################################################

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#functions for 6x6 coordinate transformation matrices (\ac{T66}), see Featherstone / Handbook of robotics \cite{Siciliano2016}
#**function: compute 6x6 coordinate transformation matrix for rotation around X axis; output: first 3 components for rotation, second 3 components for translation! See Featherstone / Handbook of robotics \cite{Siciliano2016}
def RotationX2T66(angle):
    c = cos(angle);
    s = sin(angle);
    return np.array(
        [[1,  0,  0,  0,  0,  0],
         [0,  c, -s,  0,  0,  0],
         [0,  s,  c,  0,  0,  0],
         [0,  0,  0,  1,  0,  0],
         [0,  0,  0,  0,  c, -s],
         [0,  0,  0,  0,  s,  c]])

#**function: compute 6x6 transformation matrix for rotation around Y axis; output: first 3 components for rotation, second 3 components for translation
def RotationY2T66(angle):
    c = cos(angle);
    s = sin(angle);
    return np.array(
        [[c,  0,  s,  0,  0,  0],
         [0,  1,  0,  0,  0,  0],
         [-s, 0,  c,  0,  0,  0],
         [0,  0,  0,  c,  0,  s],
         [0,  0,  0,  0,  1,  0],
         [0,  0,  0, -s,  0,  c]])

#**function: compute 6x6 transformation matrix for rotation around Z axis; output: first 3 components for rotation, second 3 components for translation
def RotationZ2T66(angle):
    c = cos(angle);
    s = sin(angle);
    return np.array(
        [[ c, -s,  0,  0,  0,  0],
         [ s,  c,  0,  0,  0,  0],
         [ 0,  0,  1,  0,  0,  0],
         [ 0,  0,  0,  c, -s,  0],
         [ 0,  0,  0,  s,  c,  0],
         [ 0,  0,  0,  0,  0,  1]])

#**function: compute 6x6 transformation matrix for translation according to 3D vector translation3D; output: first 3 components for rotation, second 3 components for translation!
def Translation2T66(translation3D):
    t = translation3D
    return np.array(
        [[    1,    0,    0,  0,  0,  0],
         [    0,    1,    0,  0,  0,  0],
         [    0,    0,    1,  0,  0,  0],
         [    0, t[2],-t[1],  1,  0,  0],
         [-t[2],    0, t[0],  0,  1,  0],
         [ t[1],-t[0],    0,  0,  0,  1]])

#**function: compute 6x6 transformation matrix for translation along X axis; output: first 3 components for rotation, second 3 components for translation!
def TranslationX2T66(translation):
    return Translation2T66([translation,0,0])

#**function: compute 6x6 transformation matrix for translation along Y axis; output: first 3 components for rotation, second 3 components for translation!
def TranslationY2T66(translation):
    return Translation2T66([0,translation,0])

#**function: compute 6x6 transformation matrix for translation along Z axis; output: first 3 components for rotation, second 3 components for translation!
def TranslationZ2T66(translation):
    return Translation2T66([0,0,translation])

#**function convert 6x6 coordinate transformation (Pl\"ucker transform) into rotation and translation
#**input: T66 given as  6x6 numpy array
#**output: [A, v] with 3x3 rotation matrix A and 3D translation vector v
def T66toRotationTranslation(T66):
    A = T66[0:3,0:3]
    v = Skew2Vec(T66[3:6,0:3]@A.T) #this leads to identical backtransformation
    return [A, v] 

#**function convert inverse 6x6 coordinate transformation (Pl\"ucker transform) into rotation and translation
#**input: inverse T66 given as  6x6 numpy array
#**output: [A, v] with 3x3 rotation matrix A and 3D translation vector v
def InverseT66toRotationTranslation(T66):
    A = (T66[0:3,0:3]).T
    v = -Skew2Vec(A@T66[3:6,0:3])
    return [A, v] 

#**function convert rotation and translation into 6x6 coordinate transformation (Pl\"ucker transform)
#**input:
#  A: 3x3 rotation matrix A
#  v: 3D translation vector v
#**output: return 6x6 transformation matrix 'T66'
def RotationTranslation2T66(A, v):
    return np.block([
        [A, np.zeros((3,3))], 
        [Skew(v)@A, A]]) 

#**function convert rotation and translation into INVERSE 6x6 coordinate transformation (Pl\"ucker transform)
#**input:
#  A: 3x3 rotation matrix A
#  v: 3D translation vector v
#**output: return 6x6 transformation matrix 'T66'
def RotationTranslation2T66Inverse(A, v):
    return np.block([
        [A.T, np.zeros((3,3))], 
        [-A.T@Skew(v), A.T]]) 

#**compute inverse of 6x6 coordinate transformation (Pl\"ucker transform)
#**input:
#  T66: 6x6 coordinate transformation (Pl\"ucker transform)
#**output: return inverse 6x6 transformation matrix 'T66'
#**note: Skew(A@v) = A@Skew(v)@A.T; v=ApB: -BRA@Skew(ApB) = Skew(BpA)@BRA
def T66Inverse(T66):
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

#**function convert 6x6 coordinate transformation (Pl\"ucker transform) into 4x4 homogeneous transformation; NOTE that the homogeneous transformation is the inverse of what is computed in function pluho() of Featherstone
#**input: T66 given as 6x6 numpy array
#**output: homogeneous transformation (4x4 numpy array)
def T66toHT(T66):
    A = T66[0:3,0:3]
    T = np.zeros((4,4))
    T[0:3,0:3] = A
    T[0:3,3] = Skew2Vec(T66[3:6,0:3] @ A.T)
    T[3,3] = 1
    return T

#**function: convert 4x4 homogeneous transformation into 6x6 coordinate transformation (Pl\"ucker transform); NOTE that the homogeneous transformation is the inverse of what is computed in function pluho() of Featherstone
#**output: 4x4 homogeneous transformation in numpy array format
#**outputinput: T66 (6x6 numpy array)
def HT2T66Inverse(T):
    A = T[0:3,0:3].T 
    v = T[0:3,3]
    return np.block([
        [A, np.zeros((3,3))],
        [-A@Skew(v), A]])

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#inertia 6D functions

#**function: convert a 3x3 matrix (list or numpy array) into a list with 6 inertia components, sorted as J00, J11, J22, J12, J02, J01
def InertiaTensor2Inertia6D(inertiaTensor):
    J = np.array(inertiaTensor)
    return [J[0,0], J[1,1], J[2,2],  J[1,2], J[0,2], J[0,1]]

#**function: convert a list or numpy array with 6 inertia components (sorted as [J00, J11, J22, J12, J02, J01]) (list or numpy array) into a 3x3 matrix (np.array)
def Inertia6D2InertiaTensor(inertia6D):
    J = inertia6D
    return np.array([[J[0],J[5],J[4]],
                     [J[5],J[1],J[3]],
                     [J[4],J[3],J[2]]])

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**class: helper class for rigid body inertia (see also derived classes Inertia...).
#Provides a structure to define mass, inertia and center of mass (COM) of a rigid body.
#The inertia tensor and center of mass must correspond when initializing the body!
#**notes:
#   in the default mode, inertiaTensorAtCOM = False, the inertia tensor must be provided with respect to the reference point; otherwise, it is given at COM; internally, the inertia tensor is always with respect to the reference point, not w.r.t. to COM!
#**example:
#i0 = RigidBodyInertia(10,np.diag([1,2,3]))
#i1 = i0.Rotated(RotationMatrixX(np.pi/2))
#i2 = i1.Translated([1,0,0])
class RigidBodyInertia:
    #**classFunction: initialize RigidBodyInertia with scalar mass, 3x3 inertiaTensor (w.r.t. reference point!!!) and center of mass com
    #**input:
    #  mass: mass of rigid body (dimensions need to be consistent, should be in SI-units)
    #  inertiaTensor: tensor given w.r.t.\ reference point, NOT w.r.t.\ center of mass!
    #  com: center of mass relative to reference point, in same coordinate system as inertiaTensor
    def __init__(self, mass=0, inertiaTensor=np.zeros([3,3]), com=np.zeros(3), inertiaTensorAtCOM = False):
        
        if np.array(inertiaTensor).shape != (3,3): #shape is a tuple
            raise ValueError('RigidBodyInertia: inertiaTensor must have shape (3,3), but received '+str(inertiaTensor.shape))
        if np.array(com).shape != (3,): #shape is a tuple
            raise ValueError('RigidBodyInertia: com must have 3 components, but received '+str(np.array(inertiaTensor).shape))
        self.mass = mass
        self.inertiaTensor = np.array(inertiaTensor)
        self.com = np.array(com)
        if inertiaTensorAtCOM:
            self.inertiaTensor = self.inertiaTensor + self.mass*np.dot(Skew(self.com).transpose(),Skew(self.com))
        
    #**classFunction: add (+) operator allows adding another inertia information with SAME local coordinate system and reference point!
    #only inertias with same center of rotation can be added!
    #**example: 
    #J = InertiaSphere(2,0.1) + InertiaRodX(1,2)
    def __add__(self, otherBodyInertia):
        sumMass = self.mass + otherBodyInertia.mass
        return RigidBodyInertia(mass=sumMass,
                                inertiaTensor = self.inertiaTensor + otherBodyInertia.inertiaTensor,
                                com=1./sumMass*(self.mass*self.com + otherBodyInertia.mass*otherBodyInertia.com))

    #**classFunction: += operator allows adding another inertia information with SAME local coordinate system and reference point!
    #only inertias with same center of rotation can be added!
    #**example: 
    #J = InertiaSphere(2,0.1) 
    #J += InertiaRodX(1,2)
    def __iadd__(self, otherBodyInertia):
        self = self + otherBodyInertia
        return self
        
    #**classFunction: set RigidBodyInertia with scalar mass, 3x3 inertiaTensor (w.r.t.\ com) and center of mass com
    #**input:
    #  mass: mass of rigid body (dimensions need to be consistent, should be in SI-units)
    #  inertiaTensorCOM: tensor given w.r.t.\ reference point, NOT w.r.t.\ center of mass!
    #  com: center of mass relative to reference point, in same coordinate system as inertiaTensor
    def SetWithCOMinertia(self, mass, inertiaTensorCOM, com):
        if np.array(inertiaTensorCOM).shape != (3,3): #shape is a tuple
            raise ValueError('RigidBodyInertia: inertiaTensorCOMmust have shape (3,3), but received '+str(np.array(inertiaTensorCOM).shape))
        if np.array(com).shape != (3,): #shape is a tuple
            raise ValueError('RigidBodyInertia: com must have 3 components, but received '+str(np.array(inertiaTensorCOM).shape))
        self.mass = mass
        self.com = np.array(com)
        self.inertiaTensor = np.array(inertiaTensorCOM) + self.mass*np.dot(Skew(self.com).transpose(),Skew(self.com))
        
        
    #**classFunction: returns 3x3 inertia tensor with respect to chosen reference point (not necessarily COM)
    def Inertia(self):
        return self.inertiaTensor

    #**classFunction: returns 3x3 inertia tensor with respect to COM
    def InertiaCOM(self):
        return self.inertiaTensor - self.mass*np.dot(Skew(self.com).transpose(),Skew(self.com))

    #**classFunction: returns center of mass (COM) w.r.t. chosen reference point
    def COM(self):
        return self.com

    #**classFunction: returns mass
    def Mass(self):
        return self.mass

    #**classFunction: returns a RigidBodyInertia with center of mass com shifted by vec; $\ra$ transforms the returned inertiaTensor to the new center of rotation
    def Translated(self, vec):
        #transform inertia to com=[0,0,0]
        inertiaCOM = self.inertiaTensor - self.mass*np.dot(Skew(self.com).transpose(),Skew(self.com))
        try:
            newCOM = self.com + vec
        except:
            raise ValueError("ERROR in RigidBodyInertia.Translated(vec): vec must be a vector with 3 components")
        inertiaCOM += self.mass*np.dot(Skew(newCOM).transpose(),Skew(newCOM))
        return RigidBodyInertia(mass=self.mass, 
                                inertiaTensor=inertiaCOM,
                                com=newCOM)

    #**classFunction: returns a RigidBodyInertia rotated by 3x3 rotation matrix rot, such that for a given J, the new inertia tensor reads Jnew = rot*J*rot.T
    #**notes: only allowed if COM=0 !
    def Rotated(self, rot):
        if NormL2(self.com) != 0:
            print("ERROR: RigidBodyInertia.Rotated only allowed in case of com=0")
            return 0
        try:
            inertia = np.dot(np.array(rot),np.dot(self.inertiaTensor,rot.transpose()))
        except:
            raise ValueError("ERROR in RigidBodyInertia.Rotated(rot): rot must be a 3x3 rotation matrix")
        return RigidBodyInertia(mass=self.mass, 
                                inertiaTensor=inertia,
                                com=self.com)

    #**classFunction: return rigid body inertia transformed by homogeneous transformation HT
    def Transformed(self, HT):
        A = HT2rotationMatrix(HT)
        v = HT2translation(HT)
        
        inertiaCOM = self.inertiaTensor - self.mass*np.dot(Skew(self.com).transpose(),Skew(self.com))
        inertiaCOM = A @ inertiaCOM @ A.T #tested with general rigid body and shifted reference point

        newCOM = A @ self.com + v

        inertiaCOM += self.mass*np.dot(Skew(newCOM).transpose(),Skew(newCOM))
        return RigidBodyInertia(mass=self.mass, 
                                inertiaTensor=inertiaCOM,
                                com=newCOM)

    #**classFunction: get vector with 6 inertia components (Jxx, Jyy, Jzz, Jyz, Jxz, Jxy) as needed in ObjectRigidBody
    def GetInertia6D(self):
        return InertiaTensor2Inertia6D(self.inertiaTensor)
        # J = self.inertiaTensor
        # return [J[0][0], J[1][1], J[2][2],  J[1][2], J[0][2], J[0][1]]


    def __str__(self):
        s = 'mass = ' + str(self.mass)
        s += '\nCOM = ' + str(self.com)
        s += '\ninertiaTensorAtOrigin = \n' + str(self.inertiaTensor)
        s += '\ninertiaTensorAtCOM = \n' + str(self.InertiaCOM())
        return s
    def __repr__(self):
        return str(self)


#**class: create RigidBodyInertia with moment of inertia and mass of a cuboid with density and side lengths sideLengths along local axes 1, 2, 3; inertia w.r.t. center of mass, com=[0,0,0]
#**example: 
# InertiaCuboid(density=1000,sideLengths=[1,0.1,0.1])
class InertiaCuboid(RigidBodyInertia):
    #**classFunction: initialize inertia
    def __init__(self, density, sideLengths):
        L1=sideLengths[0]
        L2=sideLengths[1]
        L3=sideLengths[2]
        newMass=density*L1*L2*L3
        RigidBodyInertia.__init__(self, mass=newMass,
                                  inertiaTensor=newMass/12.*np.diag([(L2**2 + L3**2),(L1**2 + L3**2),(L1**2 + L2**2)]),
                                  com=np.zeros(3))

#**class: create RigidBodyInertia with moment of inertia and mass of a rod with mass m and length L in local 1-direction (x-direction); inertia w.r.t. center of mass, com=[0,0,0]
class InertiaRodX(RigidBodyInertia):
    #**classFunction: initialize inertia with mass and length of rod
    def __init__(self, mass, length):
        RigidBodyInertia.__init__(self, mass=mass,
                                  inertiaTensor=mass/12.*np.diag([0.,length**2,length**2]),
                                  com=np.zeros(3))
        
#**class: create RigidBodyInertia with moment of inertia and mass of mass point with 'mass'; inertia w.r.t. center of mass, com=[0,0,0]
class InertiaMassPoint(RigidBodyInertia):
    #**classFunction: initialize inertia with mass of point
    def __init__(self, mass):
        RigidBodyInertia.__init__(self, mass=mass,
                                  inertiaTensor=np.zeros([3,3]),
                                  com=np.zeros(3))

#**class: create RigidBodyInertia with moment of inertia and mass of sphere with mass and radius; inertia w.r.t. center of mass, com=[0,0,0]
class InertiaSphere(RigidBodyInertia):
    #**classFunction: initialize inertia with mass and radius of sphere
    def __init__(self, mass, radius):
        J = 2.*mass/5.*radius**2
        RigidBodyInertia.__init__(self, mass=mass,
                                  inertiaTensor=np.diag([J,J,J]),
                                  com=np.zeros(3))
        
#**class: create RigidBodyInertia with moment of inertia and mass of hollow sphere with mass (concentrated at circumference) and radius; inertia w.r.t. center of mass, com=0
class InertiaHollowSphere(RigidBodyInertia):
    #**classFunction: initialize inertia with mass and (inner==outer) radius of hollow sphere
    def __init__(self, mass, radius):
        J = 2.*mass/3.*radius**2
        RigidBodyInertia.__init__(self, mass=mass,
                                  inertiaTensor=np.diag([J,J,J]),
                                  com=np.zeros(3))

#**class: create RigidBodyInertia with moment of inertia and mass of cylinder with density, length and outerRadius; axis defines the orientation of the cylinder axis (0=x-axis, 1=y-axis, 2=z-axis); for hollow cylinder use innerRadius != 0; inertia w.r.t. center of mass, com=[0,0,0]
class InertiaCylinder(RigidBodyInertia):
    #**classFunction: initialize inertia with density, length, outer radius, axis (0=x-axis, 1=y-axis, 2=z-axis) and optional inner radius (for hollow cylinder)
    def __init__(self, density, length, outerRadius, axis, innerRadius=0):
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
        

#**function: convert string into exudyn.NodeType; call e.g. with 'NodeType.RotationEulerParameters' or 'RotationEulerParameters'
#**notes: function is not very fast, so should be avoided in time-critical situations
def StrNodeType2NodeType(sNodeType):
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
#**function: get node item interface according to nodeType, using initialization with position, velocity, angularVelocity and rotationMatrix
#**input:
#   nodeType: a node type according to exudyn.NodeType, or a string of it, e.g., 'NodeType.RotationEulerParameters' (fastest, but additional algebraic constraint equation), 'NodeType.RotationRxyz' (Tait-Bryan angles, singularity for second angle at +/- 90 degrees), 'NodeType.RotationRotationVector' (used for Lie group integration)
#   position: reference position as list or numpy array with 3 components (in global/world frame)
#   velocity: initial translational velocity as list or numpy array with 3 components (in global/world frame)
#   rotationMatrix: 3x3 list or numpy matrix to define reference rotation; use EITHER rotationMatrix=[[...],[...],[...]] (while rotationParameters=[]) or rotationParameters=[...] (while rotationMatrix=[]) 
#   rotationParameters: reference rotation parameters; use EITHER rotationMatrix=[[...],[...],[...]] (while rotationParameters=[]) or rotationParameters=[...] (while rotationMatrix=[]) 
#   angularVelocity: initial angular velocity as list or numpy array with 3 components (in global/world frame)
#**output: returns list containing node number and body number: [nodeNumber, bodyNumber]
def GetRigidBodyNode(nodeType, 
                 position=[0,0,0], 
                 velocity=[0,0,0], 
                 rotationMatrix= [],
                 rotationParameters = [],
                 angularVelocity=[0,0,0]):

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
#**function: DEPRECATED: adds a node (with str(exu.NodeType. ...)) and body for a given rigid body; all quantities (esp. velocity and angular velocity) are given in global coordinates!
#**input:
#   inertia: an inertia object as created by class RigidBodyInertia; containing mass, COM and inertia
#   nodeType: a node type according to exudyn.NodeType, or a string of it, e.g., 'NodeType.RotationEulerParameters' (fastest, but additional algebraic constraint equation), 'NodeType.RotationRxyz' (Tait-Bryan angles, singularity for second angle at +/- 90 degrees), 'NodeType.RotationRotationVector' (used for Lie group integration)
#   position: reference position as list or numpy array with 3 components (in global/world frame)
#   velocity: initial translational velocity as list or numpy array with 3 components (in global/world frame)
#   rotationMatrix: 3x3 list or numpy matrix to define reference rotation; use EITHER rotationMatrix=[[...],[...],[...]] (while rotationParameters=[]) or rotationParameters=[...] (while rotationMatrix=[]) 
#   rotationParameters: reference rotation parameters; use EITHER rotationMatrix=[[...],[...],[...]] (while rotationParameters=[]) or rotationParameters=[...] (while rotationMatrix=[]) 
#   angularVelocity: initial angular velocity as list or numpy array with 3 components (in global/world frame)
#   gravity: if provided as list or numpy array with 3 components, it adds gravity force to the body at the COM, i.e., fAdd = m*gravity
#   graphicsDataList: list of graphicsData objects to define appearance of body
#**output: returns list containing node number and body number: [nodeNumber, bodyNumber]
#**notes: DEPRECATED and will be removed; use MainSystem.CreateRigidBody(...) instead!
def AddRigidBody(mainSys, inertia, 
                 nodeType = exu.NodeType.RotationEulerParameters, 
                 position=[0,0,0], velocity=[0,0,0], 
                 rotationMatrix= [],
                 rotationParameters = [],
                 angularVelocity=[0,0,0],
                 gravity=[0,0,0],
                 graphicsDataList=[]):

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
    
    if NormL2(gravity) != 0.:
        markerNumber = mainSys.AddMarker(eii.MarkerBodyMass(bodyNumber=bodyNumber))
        mainSys.AddLoad(eii.LoadMassProportional(markerNumber=markerNumber, loadVector=gravity))
    
    return [nodeNumber, bodyNumber]


#**function: DEPRECATED (use MainSystem function instead): add revolute joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed
#**input:
#  mbs: the MainSystem to which the joint and markers shall be added
#  body0: a object number for body0, must be rigid body or ground object
#  body1: a object number for body1, must be rigid body or ground object
#  point: a 3D vector as list or np.array containing the global center point of the joint in reference configuration
#  axis: a 3D vector as list or np.array containing the global rotation axis of the joint in reference configuration
#  useGlobalFrame: if False, the point and axis vectors are defined in the local coordinate system of body0
#**output: returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
#**notes: DEPRECATED and will be removed; use MainSystem.CreateRevoluteJoint(...) instead!
def AddRevoluteJoint(mbs, body0, body1, point, axis, useGlobalFrame=True, 
                     showJoint=True, axisRadius=0.1, axisLength=0.4):

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
    #print(AJ)
    
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


#**function: DEPRECATED (use MainSystem function instead): add prismatic joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed
#**input:
#  mbs: the MainSystem to which the joint and markers shall be added
#  body0: a object number for body0, must be rigid body or ground object
#  body1: a object number for body1, must be rigid body or ground object
#  point: a 3D vector as list or np.array containing the global center point of the joint in reference configuration
#  axis: a 3D vector as list or np.array containing the global translation axis of the joint in reference configuration
#  useGlobalFrame: if False, the point and axis vectors are defined in the local coordinate system of body0
#**output: returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
#**notes: DEPRECATED and will be removed; use MainSystem.CreatePrismaticJoint(...) instead!
def AddPrismaticJoint(mbs, body0, body1, point, axis, useGlobalFrame=True, 
                     showJoint=True, axisRadius=0.1, axisLength=0.4):
        
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




