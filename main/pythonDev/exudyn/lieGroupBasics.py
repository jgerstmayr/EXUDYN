#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Lie group methods and formulas for Lie group integration.
# References:   \\
#               For details on Lie group methods used here, see the references \cite{Henderson1977, Simo1988, Bruels2011, Sonneville2014, Sonneville2017, Terze2016, Mueller2017}.
#               Lie group methods for rotation vector are described in Holzinger and Gerstmayr \cite{HolzingerGerstmayr2020, Holzinger2021}.
#               
# Author:   Stefan Holzinger
# Date:     2020-09-11
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



import numpy as np
from numpy.linalg import norm
from math import sin, cos, tan, atan2, acos, sqrt

import exudyn as exu
from exudyn.rigidBodyUtilities import EulerParameters2RotationMatrix
from exudyn.rigidBodyUtilities import RotXYZ2RotationMatrix
from exudyn.rigidBodyUtilities import HT2rotationMatrix
from exudyn.rigidBodyUtilities import HT2translation
from exudyn.rigidBodyUtilities import HomogeneousTransformation
from exudyn.rigidBodyUtilities import Skew, Skew2Vec
from exudyn.rigidBodyUtilities import ComputeRotationAxisFromRotationVector 





# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#       HELPER METHODS FOR BASIC LIE GROUP METHODS
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
#**function: compute the cardinal sine function in radians
#**input: scalar float or int value
#**output: float value in radians
def Sinc(x):
    if x == 0.:
        s = 1.0
    else:
        s = sin(x)/x
#    s = np.sinc(x/np.pi) # is not more accurate than our implementation!
    return s


#**function: compute the cotangent function cot(x)=1/tan(x) in radians
#**input: scalar float or int value
#**output: float value in radians
def Cot(x):
    return 1/tan(x)


#**function: computes 3x3 rotation matrix from 7x7 R3xSO(3) matrix, see \cite{Bruels2011}
#**input: 
#   G: 7x7 matrix as np.array
#**output: 3x3 rotation matrix as np.array
def R3xSO3Matrix2RotationMatrix(G): 
    return G[0:3,0:3]


#**function: computes translation part of R3xSO(3) matrix, see \cite{Bruels2011}
#**input: 
#   G: 7x7 matrix as np.array
#**output: 3D vector as np.array containg translational part of R3xSO(3)
def R3xSO3Matrix2Translation(G):
    return G[3:6,6]


#**function: builds 7x7 matrix as element of the Lie group R3xSO(3), see \cite{Bruels2011}
#**input: 
#   x: 3D vector as np.array representing the translation part corresponding to R3 
#   R: 3x3 rotation matrix as np.array
#**output: 7x7 matrix as np.array
def R3xSO3Matrix(x,R):
    G = np.eye(7)
    G[0:3,0:3] = R
    G[3:6,6] = x
    return G









#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#       EXPONENTIAL MAPS AND TANGENT OPERATORS
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#**function: compute the matrix exponential map on the Lie group SO(3), see \cite{Mueller2017}
#**input: 3D rotation vector as np.array
#**output: 3x3 matrix as np.array
def ExpSO3(Omega):
    phi = norm(Omega)
    I = np.eye(3)
    OmegaSkew = Skew(Omega)
    R = I + Sinc(phi)*OmegaSkew + 0.5*(Sinc(0.5*phi)**2)*np.matmul(OmegaSkew, OmegaSkew)
    return R  


#**function: compute the quaternion exponential map on the Lie group S(3), see \cite{Terze2016, Mueller2017}
#**input: 3D rotation vector as np.array
#**output: 4D vector as np.array containing four Euler parameters 
#          entry zero of output represent the scalar part of Euler parameters
def ExpS3(Omega):
    phi = norm(Omega)
    q0 = cos(0.5*phi)
    qV = 0.5*Sinc(0.5*phi)*Omega
    return np.array([q0, qV[0], qV[1], qV[2]])     


#**function: compute the matrix logarithmic map on the Lie group SO(3), see \cite{Sonneville2014, Sonneville2017}
#**input: 3x3 rotation matrix as np.array
#**output: 3x3 skew symmetric matrix as np.array
def LogSO3(R):
    val = 0.5*(np.trace(R)-1) #if slightly larger than 1, due to numerical differentiation
    if abs(val)>1:
        val = val/abs(val)
    phi = acos(val)
    if phi == 0.:
        X = np.zeros((3,3))
    else:
        X = (phi/(2*sin(phi)))*(R - np.transpose(R))
    return X



termExpanded = lambda x: 1/6 - (1/120)*x**2 + (1/5040)*x**4 - (1/362880)*x**6 + (1/39916800)*x**8 - (1/6227020800)*x**10 + (1/1307674368000)*x**12 
#**function: compute the tangent operator corresponding to ExpSO3, see \cite{Bruels2011}
#**input: 3D rotation vector as np.array
#**output: 3x3 matrix as np.array       
def TExpSO3(Omega):
    phi = norm(Omega)
    I = np.eye(3)
    if phi == 0.:
        T = I
    else:
        OmegaSkew = Skew(Omega)
        t1 = -0.5*Sinc(phi/2)**2 #(np.cos(phi)-1)/(phi**2)
        if phi < 0.01:
            t2 = termExpanded(phi)
        else:
            t2 = (1/(phi**2))*(1-(sin(phi)/phi))
        T = I + t1*OmegaSkew + t2*np.dot(OmegaSkew, OmegaSkew)
    return T


#**function: compute the inverse of the tangent operator TExpSO3, see \cite{Sonneville2014}
#            this function was improved, see coordinateMaps.pdf by Stefan Holzinger
#**input: 3D rotation vector as np.array
#**output: 3x3 matrix as np.array 
def TExpSO3Inv(Omega):
    phi = norm(Omega)
    if phi == 0.0: 
        Tinv = np.eye(3)
    elif phi <= 0.02: 
        c = (1/12) + (1/720)*phi**2 + (1/30240)*phi**4 # + (1/1209600)*phi**6 # + (1/47900160)*phi**8
        b = 1 - c*phi**2
        A = np.diag([b, b, b])
        OmegaSkew = Skew(0.5*Omega)
        Tinv = A + OmegaSkew + c*np.outer(Omega,Omega)
    else:
        OmegaSkew = Skew(0.5*Omega)
        epsilon = 0.5*phi
        beta = epsilon*Cot( epsilon )
        gamma = (1 - beta)/(phi**2)
        Tinv = np.diag([beta, beta, beta]) + OmegaSkew + gamma*np.outer(Omega,Omega)
    return Tinv 


#**function: compute the matrix exponential map on the Lie group SE(3), see \cite{Bruels2011}
#**input: 6D incremental motion vector as np.array
#**output: 4x4 homogeneous transformation matrix as np.array
def ExpSE3(x):
    U     = x[0:3]
    Omega = x[3:6]
    R = ExpSO3(Omega)
    x = np.dot(np.transpose(TExpSO3(Omega)), U)
    return HomogeneousTransformation(R, x)


#**function: compute the matrix logarithm on the Lie group SE(3), see \cite{Sonneville2014}
#**input: 4x4 homogeneous transformation matrix as np.array
#**output: 4x4 skew symmetric matrix as np.array
def LogSE3(H):
    R = HT2rotationMatrix(H)
    aSkew = LogSO3(R)
    a = Skew2Vec(aSkew)   
    A = np.transpose(TExpSO3Inv(a))
    x = np.dot(A,HT2translation(H))
    log = np.zeros((4,4))
    log[0:3,0:3] = aSkew
    log[0:3,3] = x
    return log
    

#**function: compute the tangent operator corresponding to ExpSE3, see \cite{Bruels2011}
#**input: 6D incremental motion vector as np.array
#**output: 6x6 matrix as np.array
def TExpSE3(x):
    U     = x[0:3]
    Omega = x[3:6]
    USkew     = Skew(U)
    OmegaSkew = Skew(Omega)
    phi = norm(Omega)
    phiOverTwo = phi/2
    if phi == 0.:
        TUOmegaPlus = -0.5*USkew 
    else:
        a = (2*sin(phiOverTwo)*cos(phiOverTwo))/phi
        b = 4*(sin(phiOverTwo)**2)/(phi**2)
        c1 = 0.5*(1-b)*USkew
        c2 = ((1-a)/(phi**2))*(np.dot(USkew,OmegaSkew) + np.dot(OmegaSkew,USkew))
        c3 = -((a-b)/(phi**2))*np.dot(np.dot(Omega,U), OmegaSkew)
        c4 = (1/(phi**2))*(0.5*b - (3/(phi**2))*(1-a))*np.dot(Omega,U)*np.dot(OmegaSkew,OmegaSkew)
        TUOmegaPlus = c1 + c2 + c3 + c4 - 0.5*USkew
    TexpSO3 = TExpSO3(Omega)
    T = np.block([[TexpSO3,         TUOmegaPlus],
                  [np.zeros((3,3)), TexpSO3]])
    return T


#**function: compute the inverse of tangent operator TExpSE3, see \cite{Sonneville2014}
#**input: 6D incremental motion vector as np.array
#**output: 6x6 matrix as np.array
def TExpSE3Inv(x):
    U     = x[0:3]
    Omega = x[3:6]
    phi = norm(Omega)
    if phi == 0.0:
        Tuwm = 0.5*Skew(U)
    else:
        alpha = Sinc(phi)
        beta = 2*(1 - cos(phi))/phi**2 
        USkew     = Skew(U)
        OmegaSkew = Skew(Omega)
        c1 = 0.5*USkew
        c2 = ((beta-alpha)/(beta*phi**2))*(np.matmul(USkew,OmegaSkew) + np.matmul(OmegaSkew,USkew))
        c3 = ((1 + alpha + -2*beta)/(beta*phi**4))*(np.dot(Omega,U))*np.matmul(OmegaSkew,OmegaSkew)
        Tuwm = c1 + c2 + c3
    TexpSO3Inv = TExpSO3Inv(Omega)
    Tinv = np.block([[TexpSO3Inv,      Tuwm],
                     [np.zeros((3,3)), TexpSO3Inv]])
    return Tinv
    

#**function: compute the matrix exponential map on the Lie group R3xSO(3), see \cite{Bruels2011}
#**input: 6D incremental motion vector as np.array
#**output: 7x7 matrix as np.array
def ExpR3xSO3(x):
    G = np.eye(7)
    G[0:3,0:3] = ExpSO3(x[3:6])
    G[3:6,6] = x[0:3]
    return G


#**function: compute the tangent operator corresponding to ExpR3xSO3, see \cite{Bruels2011}
#**input: 6D incremental motion vector as np.array
#**output: 6x6 matrix as np.array
def TExpR3xSO3(x):
    T = np.eye(6)
    T[3:6,3:6] = TExpSO3(x[3:6])
    return T


#**function: compute the inverse of tangent operator TExpR3xSO3
#**input: 6D incremental motion vector as np.array
#**output: 6x6 matrix as np.array
def TExpR3xSO3Inv(x):
    T = np.eye(6)
    T[3:6,3:6] = TExpSO3Inv(x[3:6])
    return T








#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#       COMPOSITION OPERATIONS FOR LIE GROUP TIME INTEGRATION METHODS 
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



#**function: compute composition operation for pairs in the Lie group R3xS3
#**input: 
#  q0: 7D vector as np.array containing position coordinates and Euler parameters
#  incrementalMotionVector: 6D incremental motion vector as np.array
#**output: 7D vector as np.array containing composed position coordinates and composed Euler parameters
def CompositionRuleDirectProductR3AndS3(q0, incrementalMotionVector):
    
    # pair (x0, theta0)
    x0     = q0[0:3]  # global COM position at time step t0
    theta0 = q0[3:7]  # Euler parameters at time step t0
    
    # pair (delta x, incremental rotation vector)
    delta_x   = incrementalMotionVector[0:3] # global position increment of COM at time step t
    incRotVec = incrementalMotionVector[3:6] # incremental rotation vector at time step t
    
    # compososition rule
    x   = x0 + delta_x
    theta = CompositionRuleForEulerParameters(theta0, ExpS3(incRotVec))
    
    return np.block([x, theta])


#**function: compute composition operation for pairs in the Lie group R3 semiTimes S3 (corresponds to SE(3))
#**input: 
#  q0: 7D vector as np.array containing position coordinates and Euler parameters
#  incrementalMotionVector: 6D incremental motion vector as np.array
#**output: 7D vector as np.array containing composed position coordinates and composed Euler parameters
def CompositionRuleSemiDirectProductR3AndS3(q0, incrementalMotionVector):
    
    # pair (x0, theta0)
    x0     = q0[0:3]  # global COM position at time step t0
    theta0 = q0[3:7]  # Euler parameters at time step t0
    
    # pair (delta x, incremental rotation vector)
    delta_x   = incrementalMotionVector[0:3] # global position increment of COM at time step t
    incRotVec = incrementalMotionVector[3:6] # incremental rotation vector at time step t
    
    # compososition rule
    R0  = EulerParameters2RotationMatrix(theta0)
    x   = x0 + np.dot(R0, np.dot(np.transpose(TExpSO3(incRotVec)), delta_x))
    theta = CompositionRuleForEulerParameters(theta0, ExpS3(incRotVec))
    
    return np.block([x, theta])


#**function: compute composition operation for pairs in the group obtained from the direct product of R3 and R3, see \cite{HolzingerGerstmayr2020}
#            the rotation vector is used as rotation parametrizations
#            this composition operation can be used in formulations which represent the translational velocities in the global (inertial) frame
#**input: 
#  q0: 6D vector as np.array containing position coordinates and rotation vector
#  incrementalMotionVector: 6D incremental motion vector as np.array
#**output: 7D vector as np.array containing composed position coordinates and composed rotation vector
def CompositionRuleDirectProductR3AndR3RotVec(q0, incrementalMotionVector):
    
    # pair (x0, psi0)
    x0   = q0[0:3]  # global COM position at time step t0
    psi0 = q0[3:6]  # rotation vector at time step t0
    
    # pair (delta x, delta theta)
    delta_x   = incrementalMotionVector[0:3] # global position increment of COM at time step t
    incRotVec = incrementalMotionVector[3:6] # incremental rotation vector at time step t
    
    # compososition rule
    x   = x0 + delta_x
    psi = CompositionRuleForRotationVectors(psi0, incRotVec)
    
    return np.block([x, psi])


#**function: compute composition operation for pairs in the group obtained from the direct product of R3 and R3.
#            the rotation vector is used as rotation parametrizations
#            this composition operation can be used in formulations which represent the translational velocities in the local (body-attached) frame
#**input: 
#  q0: 6D vector as np.array containing position coordinates and rotation vector
#  incrementalMotionVector: 6D incremental motion vector as np.array
#**output: 6D vector as np.array containing composed position coordinates and composed rotation vector
def CompositionRuleSemiDirectProductR3AndR3RotVec(q0, incrementalMotionVector):
    
    # pair (x0, psi0)
    x0   = q0[0:3]  # global COM position at time step t0
    psi0 = q0[3:6]  # rotation vector at time step t0
    
    # pair (delta x, delta theta)
    delta_x   = incrementalMotionVector[0:3] # global position increment of COM at time step t
    incRotVec = incrementalMotionVector[3:6] # incremental rotation vector at time step t
    
    # compososition rule
    R0  = ExpSO3(psi0)
    x   = x0 + np.dot(R0, np.dot(np.transpose(TExpSO3(incRotVec)), delta_x))
    psi = CompositionRuleForRotationVectors(psi0, incRotVec)
    
    return np.block([x, psi])


#**function: compute composition operation for pairs in the group obtained from the direct product of R3 and R3.
#            Cardan-Tait/Bryan (CTB) angles are used as rotation parametrizations
#            this composition operation can be used in formulations which represent the translational velocities in the global (inertial) frame
#**input: 
#  q0: 6D vector as np.array containing position coordinates and Cardan-Tait/Bryan angles
#  incrementalMotionVector: 6D incremental motion vector as np.array
#**output: 6D vector as np.array containing composed position coordinates and composed Cardan-Tait/Bryan angles
def CompositionRuleDirectProductR3AndR3RotXYZAngles(q0, incrementalMotionVector):
    
    # pair (x0, psi0)
    x0     = q0[0:3]  # global COM position at time step t0
    alpha0 = q0[3:6]  # Cardan-Tait/Bryan angles at time step t0
    
    # pair (delta x, delta theta)
    delta_x   = incrementalMotionVector[0:3] # global position increment of COM at time step t
    incRotVec = incrementalMotionVector[3:6] # incremental rotation vector at time step t
    
    # compososition rule
    x   = x0 + delta_x
    alpha = CompositionRuleRotXYZAnglesRotationVector(alpha0, incRotVec)
    
    return np.block([x, alpha])


#**function: compute composition operation for pairs in the group obtained from the direct product of R3 and R3.
#            Cardan-Tait/Bryan (CTB) angles are used as rotation parametrizations
#            this composition operation can be used in formulations which represent the translational velocities in the local (body-attached) frame
#**input: 
#  q0: 6D vector as np.array containing position coordinates and Cardan-Tait/Bryan angles
#  incrementalMotionVector: 6D incremental motion vector as np.array
#**output: 6D vector as np.array containing composed position coordinates and composed Cardan-Tait/Bryan angles
def CompositionRuleSemiDirectProductR3AndR3RotXYZAngles(q0, incrementalMotionVector):
    
    # pair (x0, psi0)
    x0     = q0[0:3]  # global COM position at time step t0
    alpha0 = q0[3:6]  # Cardan-Tait/Bryan angles at time step t0
    
    # pair (delta x, delta theta)
    delta_x   = incrementalMotionVector[0:3] # global position increment of COM at time step t
    incRotVec = incrementalMotionVector[3:6] # incremental rotation vector at time step t
    
    # compososition rule
    R0    = RotXYZ2RotationMatrix(alpha0)
    x     = x0 + np.dot(R0, np.dot(np.transpose(TExpSO3(incRotVec)), delta_x))
    alpha = CompositionRuleRotXYZAnglesRotationVector(alpha0, incRotVec)
    
    return np.block([x, alpha])
 

#**function: compute composition operation for Euler parameters (unit quaternions)
#            this composition operation is quaternion multiplication, see \cite{Terze2016}
#**input: 
#  q: 4D vector as np.array containing Euler parameters
#  p: 4D vector as np.array containing Euler parameters
#**output: 4D vector as np.array containing composed (multiplied) Euler parameters
def CompositionRuleForEulerParameters(q, p):
    p0 = p[0]
    pV = p[1:4]
    q0 = q[0]
    qV = q[1:4]
    x0 = q0*p0 - np.dot(qV,pV)
    xV = q0*pV + p0*qV + np.cross(qV,pV)
    return np.array([x0, xV[0], xV[1], xV[2]])


#**function: compute composition operation for rotation vectors v0 and Omega, see \cite{Holzinger2021}
#**input: 
#  v0: 3D rotation vector as np.array
#  Omega: 3D (incremental) rotation vector as np.array
#**output: 3D vector as np.array containing composed rotation vector v
def CompositionRuleForRotationVectors(v0, Omega):
    w1Half = 0.5*norm(v0)
    w2Half = 0.5*norm(Omega)
    c0 = cos(w1Half)
    c1 = cos(w2Half)
    s0 = Sinc(w1Half)
    s1 = Sinc(w2Half)
    x = c0*c1 - 0.25*s0*s1*np.dot(v0,Omega)
    xPower = x**2
    xTemp = sqrt(1 - xPower)
    w = np.pi - 2*atan2(x,xTemp)        
    rho = s0*c1*v0 + c0*s1*Omega + 0.5*s0*s1*np.cross(v0, Omega)
    n = ComputeRotationAxisFromRotationVector(rho)
    v = w*n
    return v


#**function: compute composition operation for RotXYZ angles, see \cite{Holzinger2021}
#**input: 
#  alpha0: 3D vector as np.array containing RotXYZ angles
#  Omega:  3D vector as np.array containing the (incremental) rotation vector
#**output: 3D vector as np.array containing composed RotXYZ angles
def CompositionRuleRotXYZAnglesRotationVector(alpha0, Omega):
  
    # Cardan-Tait/Bryan angles
    psi0   = alpha0[0]
    theta0 = alpha0[1]
    phi0   = alpha0[2]
    
    # define unit vectors
    e1 = np.array([1, 0, 0])
    e2 = np.array([0, 1, 0])
    e3 = np.array([0, 0, 1])
    
    # compute vectors u and v
    R0 = RotXYZ2RotationMatrix(alpha0)
    exp = ExpSO3(Omega)
    u1 = np.dot(e1, R0) 
    u2 = np.dot(e2, R0)
    u3 = np.dot(e3, R0)
    v1 = np.dot(exp, e1)
    v2 = np.dot(exp, e2)
    v3 = np.dot(exp, e3)
    u1v3 = np.dot(u1,v3)
    cosTheta = sqrt(1 - u1v3**2)
    
    # compute mu
    if cosTheta == 0.0:
        mu = 0
    else:
        mu = 1 / cosTheta
    
    # compute sine and cosine terms of incremental angle changes
    sinPsi0 = sin(psi0)
    cosPsi0 = cos(psi0)
    sinTheta0 = sin(theta0)
    cosTheta0 = cos(theta0)
    sinPhi0 = sin(phi0)
    cosPhi0 = cos(phi0)    
    u2v3 = np.dot(u2,v3)
    u3v3 = np.dot(u3,v3)
    u1v2 = np.dot(u1,v2)
    u1v1 = np.dot(u1,v1)    
    y1 = -u2v3*cosPsi0 - u3v3*sinPsi0
    x1 =  u3v3*cosPsi0 - u2v3*sinPsi0
    y2 = u1v3*cosTheta0 - cosTheta*sinTheta0
    x2 = cosTheta*cosTheta0 + u1v3*sinTheta0
    y3 = -u1v2*cosPhi0 - u1v1*sinPhi0
    x3 = u1v1*cosPhi0 - u1v2*sinPhi0
    
    # compute incremental angles changes
    deltaPsi   = atan2(mu*y1, mu*x1)
    deltaTheta = atan2(y2, x2)
    deltaPhi   = atan2(mu*y3, mu*x3)
    deltaAlpha = np.array([deltaPsi, deltaTheta, deltaPhi])
    
    return alpha0 + deltaAlpha




