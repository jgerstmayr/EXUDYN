#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Lie group methods and formulas for Lie group integration.
# References:   \\
#               For details on Lie group methods used here, see the references \cite{Henderson1977, Simo1988, Bruels2011, Sonneville2014, Sonneville2017, Terze2016, Mueller2017}.
#               Lie group methods for rotation vector are described in Holzinger and Gerstmayr \cite{HolzingerGerstmayr2020, Holzinger2021}.
#               
# Author:   Stefan Holzinger, Johannes Gerstmayr
# Date:     2020-09-11
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



import numpy as np
from numpy.linalg import norm
from math import sin, cos, tan, atan2, acos, sqrt

#import exudyn as exu
from exudyn.rigidBodyUtilities import EulerParameters2RotationMatrix, RotXYZ2RotationMatrix, HT2rotationMatrix, HT2translation, \
            HomogeneousTransformation, Skew, Skew2Vec, ComputeRotationAxisFromRotationVector, RotationMatrix2EulerParameters


# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#       HELPER METHODS FOR BASIC LIE GROUP METHODS
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
#**function: compute the cardinal sine function in radians
#**input: scalar float or int value
#**output: float value in radians
#**author: Stefan Holzinger
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
#**author: Stefan Holzinger
def Cot(x):
    return 1/tan(x)


#**function: computes 3x3 rotation matrix from 7x7 R3xSO(3) matrix, see \cite{Bruels2011}
#**input: 
#   G: 7x7 matrix as np.array
#**output: 3x3 rotation matrix as np.array
#**author: Stefan Holzinger
def R3xSO3Matrix2RotationMatrix(G): 
    return G[0:3,0:3]


#**function: computes translation part of R3xSO(3) matrix, see \cite{Bruels2011}
#**input: 
#   G: 7x7 matrix as np.array
#**output: 3D vector as np.array containg translational part of R3xSO(3)
#**author: Stefan Holzinger
def R3xSO3Matrix2Translation(G):
    return G[3:6,6]


#**function: builds 7x7 matrix as element of the Lie group R3xSO(3), see \cite{Bruels2011}
#**input: 
#   x: 3D vector as np.array representing the translation part corresponding to R3 
#   R: 3x3 rotation matrix as np.array
#**output: 7x7 matrix as np.array
#**author: Stefan Holzinger
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
#**author: Stefan Holzinger
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
#**author: Stefan Holzinger
def ExpS3(Omega):
    phi = norm(Omega)
    q0 = cos(0.5*phi)
    qV = 0.5*Sinc(0.5*phi)*Omega
    return np.array([q0, qV[0], qV[1], qV[2]])     


#**function: compute the matrix logarithmic map on the Lie group SO(3)
#**input: 3x3 rotation matrix as np.array
#**output: 3x3 skew symmetric matrix as np.array
#**author: Johannes Gerstmayr
#**notes: improved accuracy for very small angles as well as angles phi close to pi AS WELL AS at phi=pi
def LogSO3(R):
    ep = RotationMatrix2EulerParameters(R)
    
    n = ep[1:]
    norm = np.linalg.norm(n)
    
    #phi = 2.*acos(ep[0])
    phi = 2.*np.arctan2(norm, ep[0])
    
    if norm != 0.:
        n = (1./norm)*n

    return Skew(phi*n)
    


# #**function: compute the matrix logarithmic map on the Lie group SO(3), see \cite{Sonneville2014, Sonneville2017}
# #**input: 3x3 rotation matrix as np.array
# #**output: 3x3 skew symmetric matrix as np.array
# #**author: Stefan Holzinger
# def LogSO3(R):
#     val = 0.5*(np.trace(R)-1) #if slightly larger than 1, due to numerical differentiation
#     if abs(val)>1:
#         val = val/abs(val)
#     phi = acos(val)
#     if phi == 0.:
#         X = np.zeros((3,3))
#     else:
#         X = (phi/(2*sin(phi)))*(R - np.transpose(R))
#     return X


#**function: compute the tangent operator corresponding to ExpSO3, see \cite{Bruels2011}
#**input: 3D rotation vector as np.array
#**output: 3x3 matrix as np.array       
#**author: Stefan Holzinger
def TExpSO3(Omega):
    #not all of these terms are needed (as implemented in C++ code):

    phi = norm(Omega)
    I = np.eye(3)
    if phi == 0.:
        T = I
    else:
        OmegaSkew = Skew(Omega)
        t1 = -0.5*Sinc(phi/2)**2 #(np.cos(phi)-1)/(phi**2)
        if phi < 0.01:
            t2 = 1 / 6 - (1 / 120)*phi**2 + (1 / 5040)*phi**4
            #termExpanded = lambda x: 1/6 - (1/120)*x**2 + (1/5040)*x**4 - (1/362880)*x**6 + (1/39916800)*x**8 - (1/6227020800)*x**10 + (1/1307674368000)*x**12 
        else:
            t2 = (1/(phi**2))*(1-(sin(phi)/phi))
        T = I + t1*OmegaSkew + t2*np.dot(OmegaSkew, OmegaSkew)
    return T


#**function: compute the inverse of the tangent operator TExpSO3, see \cite{Sonneville2014}
#            this function was improved, see coordinateMaps.pdf by Stefan Holzinger
#**input: 3D rotation vector as np.array
#**output: 3x3 matrix as np.array 
#**author: Stefan Holzinger
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
#**author: Stefan Holzinger
def ExpSE3(x):
    U     = x[0:3]
    Omega = x[3:6]
    R = ExpSO3(Omega)
    x = np.dot(np.transpose(TExpSO3(Omega)), U)
    return HomogeneousTransformation(R, x)


#**function: compute the matrix logarithm on the Lie group SE(3), see \cite{Sonneville2014}
#**input: 4x4 homogeneous transformation matrix as np.array
#**output: 4x4 skew symmetric matrix as np.array
#**author: Stefan Holzinger
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
#**author: Stefan Holzinger
#**notes: improved accuracy for very small angles as well as angles phi 
def TExpSE3(x):
    U     = x[0:3]
    Omega = x[3:6]
    USkew     = Skew(U)
    OmegaSkew = Skew(Omega)
    phi = norm(Omega)
   # phiOverTwo = 0.5*phi
    
    # coefficient f2, see Phd thesis Stefan Hante, Table. 1, page 121
    if phi >= 1e-2:
        f2 = (np.cos(phi) - 1) / phi**2
    else:
        f2 = -0.5 + (1/24)*phi**2 - (1/720)*phi**4

    # coefficient f3, see Phd thesis Stefan Hante, Table. 1, page 121
    if phi >= 1e-4:
        f3 = (phi - np.sin(phi)) / phi**3
    else:
        f3 = 1/6 - (1/24)*phi**2 - (1/720)*phi**4     

    # coefficient f4, see Phd thesis Stefan Hante, Table. 1, page 121
    if phi >= 1e-1:
        f4 = (2 - 2*np.cos(phi) - phi*np.sin(phi)) / phi**4
    else:
        f4 = 1/12 - (1/180)*phi**2 + (1/6720)*phi**4 - (1/453600)*phi**6
        
    # coefficient f5, see Phd thesis Stefan Hante, Table. 1, page 121
    if phi >= 1e-1:
        f5 = ( phi*(2 + np.cos(phi)) - 3*np.sin(phi)) / phi**5
    else:
        f5 = 1/60 - (1/1260)*phi**2 + (1/60480)*phi**4 - (1/4989600)*phi**6     
    
    TUOmegaPlus = f2*USkew + f3*( USkew @ OmegaSkew + OmegaSkew @ USkew ) + f4*np.dot(Omega,U)*OmegaSkew - f5*np.dot(Omega,U)*OmegaSkew**2
    
    TexpSO3 = TExpSO3(Omega)
    T = np.block([[TexpSO3,         TUOmegaPlus],
                  [np.zeros((3,3)), TexpSO3]])
    return T



#**function: compute the inverse of tangent operator TExpSE3, see \cite{Sonneville2014}
#**input: 6D incremental motion vector as np.array
#**output: 6x6 matrix as np.array
#**author: Stefan Holzinger
#**notes: improved accuracy for very small angles as well as angles phi 
def TExpSE3Inv(x):
    U     = x[0:3]
    Omega = x[3:6]
    USkew     = Skew(U)
    OmegaSkew = Skew(Omega)
    phi = norm(Omega)
    phiOverTwo = 0.5*phi
    
    # coefficient f6, see Phd thesis Stefan Hante, Table. 1, page 121
    if phi >= 1e-2:
        f6 = (2 - Cot(phiOverTwo)) / (2*phi**2)
    else:
        f6 = 1/12 + (1/720)*phi**2 + (1/30240)*phi**4
    
    # coefficient f8, see Phd thesis Stefan Hante, Table. 1, page 121
    if phi >= 2e-1:
        f8 = (phi*np.sin(phi) + 4*np.cos(phi) + phi**2 - 4) / (4*np.sin(phiOverTwo)**2 * phi**4)
    else:
        f8 = 1/360 + (1/7560)*phi**2 + (1/201600)*phi**4 + (1/5987520)*phi**6 + (691/130767436800)*phi**8
    
    # Matrix C2, see Phd thesis Stefan Hante, Sect.A.3, page 117
    Tuwm = 0.5*USkew + f6*( USkew @ OmegaSkew + OmegaSkew @ USkew ) + f8*np.dot(Omega,U)*OmegaSkew**2
    TexpSO3Inv = TExpSO3Inv(Omega) #NOTE: overrides the function TexpSO3Inv
    Tinv = np.block([[TexpSO3Inv,      Tuwm],
                     [np.zeros((3,3)), TexpSO3Inv]])
    return Tinv
    

#**function: compute the matrix exponential map on the Lie group R3xSO(3), see \cite{Bruels2011}
#**input: 6D incremental motion vector as np.array
#**output: 7x7 matrix as np.array
#**author: Stefan Holzinger
def ExpR3xSO3(x):
    G = np.eye(7)
    G[0:3,0:3] = ExpSO3(x[3:6])
    G[3:6,6] = x[0:3]
    return G


#**function: compute the tangent operator corresponding to ExpR3xSO3, see \cite{Bruels2011}
#**input: 6D incremental motion vector as np.array
#**output: 6x6 matrix as np.array
#**author: Stefan Holzinger
def TExpR3xSO3(x):
    T = np.eye(6)
    T[3:6,3:6] = TExpSO3(x[3:6])
    return T


#**function: compute the inverse of tangent operator TExpR3xSO3
#**input: 6D incremental motion vector as np.array
#**output: 6x6 matrix as np.array
#**author: Stefan Holzinger
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
#**author: Stefan Holzinger
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
#**author: Stefan Holzinger
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
#**author: Stefan Holzinger
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
#**author: Stefan Holzinger
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
#**author: Stefan Holzinger
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
#**author: Stefan Holzinger
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
#**author: Stefan Holzinger
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
#**author: Stefan Holzinger
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
#**author: Stefan Holzinger
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




