# Utility functions and structures for Exudyn
"""
Created on Fri Jul 26 10:53:30 2019

@author: Johannes Gerstmayr

goal: support functions, which simplify the generation of models
"""
#constants and fixed structures:
import numpy as np #LoadSolutionFile
from itemInterface import *
#import exudyn as exu #do not import! causes troubles with exudynFast, etc.!!
from exudynBasicUtilities import NormL2


eulerParameters0 = [1.,0.,0.,0.] #Euler parameters for case where rotation angle is zero (rotation axis arbitrary)

#**function: compute orthogonal basis vectors (normal1, normal2) for given vector0 (non-unique solution!); if vector0 == [0,0,0], then any normal basis is returned
def ComputeOrthonormalBasis(vector0):
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


#**function: compute skew symmetric 3x3-matrix from 3x1- or 1x3-vector
def Skew(vector):
    skewsymmetricMatrix = np.array([[ 0.,       -vector[2], vector[1]], 
                                    [ vector[2], 0.,       -vector[0]],
                                    [-vector[1], vector[0], 0.]])
    return skewsymmetricMatrix

#**function: convert skew symmetric matrix m to vector
def Skew2Vec(m):
    return np.array([-m[1][2], m[0][2], -m[0][1]])

#**function: compute (3 x 3*n) skew matrix from (3*n) vector
def ComputeSkewMatrix(v):
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



#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#helper functions for RIGID BODY KINEMATICS:

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

#**function: compute Euler parameters from given rotation matrix
#**input: 3x3 rotation matrix as list of lists or as np.array
#**output: vector of 4 eulerParameters as np.array
def RotationMatrix2EulerParameters(rotationMatrix):
    A=rotationMatrix
    trace = A[0][0] + A[1][1] + A[2][2] + 1.0
    M_EPSILON = 1e-15 #small number to avoid division by zero

    if (abs(trace) > M_EPSILON):
        s = 0.5 / np.sqrt(abs(trace))
        ep0 = 0.25 / s
        ep1 = (A[2][1] - A[1][2]) * s
        ep2 = (A[0][2] - A[2][0]) * s
        ep3 = (A[1][0] - A[0][1]) * s
    else:
        if (A[0][0] > A[1][1]) and (A[0][0] > A[2][2]):
            s = 2.0 * np.sqrt(abs(1.0 + A[0][0] - A[1][1] - A[2][2]))
            ep1 = 0.25 * s
            ep2 = (A[0][1] + A[1][0]) / s
            ep3 = (A[0][2] + A[2][0]) / s
            ep0 = (A[1][2] - A[2][1]) / s
        elif A[1][1] > A[2][2]:
            s = 2.0 * np.sqrt(abs(1.0 + A[1][1] - A[0][0] - A[2][2]))
            ep1 = (A[0][1] + A[1][0]) / s
            ep2 = 0.25 * s
            ep3 = (A[1][2] + A[2][1]) / s
            ep0 = (A[0][2] - A[2][0]) / s
        else:
            s = 2.0 * np.sqrt(abs(1.0 + A[2][2] - A[0][0] - A[1][1]));
            ep1 = (A[0][2] + A[2][0]) / s
            ep2 = (A[1][2] + A[2][1]) / s
            ep3 = 0.25 * s
            ep0 = (A[0][1] - A[1][0]) / s

    return np.array([ep0,ep1,ep2,ep3])

#**function: compute time derivative of Euler parameters from (global) angular velocity vector
#note that for Euler parameters $\pv$, we have $\tomega=\Gm \pv_t$ ==> $\Gm^T \tomega = \Gm^T\cdot \Gm\cdot \pv_t$ ==> $\Gm^T \Gm=4(\Im_{4x4} - \pv\cdot \pv^T)\pv_t = 4 (\Im_{4x4}) \pv_t$
#**input: 
#  angularVelocity: 3D vector of angular velocity in global frame, as lists or as np.array
#  eulerParameters: vector of 4 eulerParameters as np.array or list
#**output: vector of time derivatives of 4 eulerParameters as np.array
def AngularVelocity2EulerParameters_t(angularVelocity, eulerParameters):
    
    GT = np.transpose(EulerParameters2G(eulerParameters))
    return 0.25*(GT.dot(angularVelocity))


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#            ROTATION VECTOR
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: rotaton matrix from rotation vector
#**input: 3D rotation vector as list or np.array
#**output: 3x3 rotation matrix as np.array
def RotationVector2RotationMatrix(rotationVector):
    rotationAngle = np.linalg.norm(rotationVector)
    I = np.diag([1, 1, 1])
    if rotationAngle == 0.0:
        R = I
    else:
        rotVecTilde = np.array(Skew(rotationVector))
        R0 = I + (np.sin(rotationAngle)/rotationAngle)*rotVecTilde
        R1 = ((1-np.cos(rotationAngle))/rotationAngle**2)*np.dot(rotVecTilde,rotVecTilde)
        R = R0 + R1
    return R

#**function: compute rotation vector from rotation matrix
#**input: 3x3 rotation matrix as list of lists or as np.array
#**output: vector of 3 components of rotation vector as np.array
def RotationMatrix2RotationVector(rotationMatrix):
    # compute a  rotation vector from given rotation matrix according to 
    # 2015 - Sonneville - A geometrical local frame approach for flexible multibody systems, p45
    theta = np.arccos(0.5*(np.trace(rotationMatrix)-1))
    if np.linalg.norm(theta) < np.pi:
        logR = (theta/(2*np.sin(theta)))*(rotationMatrix - np.transpose(rotationMatrix))
        rotationVector = Skew2Vec(logR)
    else:
        rotationVector = np.zeros(3)
    return rotationVector


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#            TAIT BRYAN ANGLES
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#**function: compute rotation matrix from consecutive xyz rotations (Tait-Bryan angles); A=Ax*Ay*Az; rot=[rotX, rotY, rotZ]
#**input: 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
#**output: 3x3 rotation matrix as np.array
def RotXYZ2RotationMatrix(rot):
    c0 = np.cos(rot[0])
    s0 = np.sin(rot[0])
    c1 = np.cos(rot[1])
    s1 = np.sin(rot[1])
    c2 = np.cos(rot[2])
    s2 = np.sin(rot[2])
    
    return np.array([[c1*c2,-c1 * s2,s1],
                  [s0*s1*c2 + c0 * s2, -s0 * s1*s2 + c0 * c2,-s0 * c1],
                  [-c0 * s1*c2 + s0 * s2,c0*s1*s2 + s0 * c2,c0*c1 ]]);

#**function: convert rotation matrix to xyz Euler angles (Tait-Bryan angles);  A=Ax*Ay*Az; 
#**input:  3x3 rotation matrix as list of lists or np.array
#**output: vector of Tait-Bryan rotation parameters [X,Y,Z] (in radiant) as np.array
def RotationMatrix2RotXYZ(rotationMatrix):
    R=rotationMatrix
    #rot=np.array([0,0,0])
    rot=[0,0,0]
    rot[0] = np.arctan2(-R[1][2], R[2][2])
    rot[1] = np.arctan2(R[0][2], np.sqrt(abs(1. - R[0][2] * R[0][2]))) #fabs for safety, if small round up error in rotation matrix ...
    rot[2] = np.arctan2(-R[0][1], R[0][0])
    return np.array(rot);

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
    

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
                      [0,	    0,        1] ]);

    
#**function: compute homogeneous transformation matrix from rotation matrix A and translation vector r
def HomogeneousTransformation(A, r):
    T = np.zeros((4,4))
    T[0:3,0:3] = A
    T[0:3,3] = r
    T[3,3] = 1
    return T

HT = HomogeneousTransformation #shortcut

#**function: homogeneous transformation for translation with vector r
def HTtranslate(r):
    T = np.eye(4)
    T[0:3,3] = r
    return T

#**function: identity homogeneous transformation:
def HT0():
    return np.eye(4)

#**function: homogeneous transformation for rotation around axis X (first axis)
def HTrotateX(angle):
    T = np.eye(4)
    T[0:3,0:3] = RotationMatrixX(angle)
    return T
    
#**function: homogeneous transformation for rotation around axis X (first axis)
def HTrotateY(angle):
    T = np.eye(4)
    T[0:3,0:3] = RotationMatrixY(angle)
    return T
    
#**function: homogeneous transformation for rotation around axis X (first axis)
def HTrotateZ(angle):
    T = np.eye(4)
    T[0:3,0:3] = RotationMatrixZ(angle)
    return T

#**function: return translation part of homogeneous transformation
def HT2translation(T):
    return T[0:3,3]

#**function: return rotation matrix of homogeneous transformation
def HT2rotationMatrix(T):
    return T[0:3,0:3]

#**function: return inverse homogeneous transformation such that inv(T)*T = np.eye(4)
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

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#helper classes for rigid body inertia

#structure to define mass, inertia and center of mass (com) of a rigid body
#the inertia tensor and center of mass must correspond when initializing the body!
#it is recommended to start with com=[0,0,0] and then to TranslateCenterOfRotation
#THIS METHOD IS NOT VERIFIED AND SUBJECT TO PROGRAMMING ERRORS ==> check your results!
class RigidBodyInertia:
    def __init__(self, mass=0, inertiaTensor=np.zeros([3,3]), com=np.zeros(3)):
        
        if inertiaTensor.shape != (3,3): #shape is a tuple
            raise ValueError('RigidBodyInertia: must have dimensions (3,3), but received'+str(inertiaTensor.shape))
        self.mass = np.array(mass)
        self.inertiaTensor = np.array(inertiaTensor)
        self.com = com
        
    #allows adding another inertia information with SAME local coordinate system
    #only inertias with same center of rotation can be added!
    def __add__(self, otherBodyInertia):
        sumMass = self.mass + otherBodyInertia.mass
        return RigidBodyInertia(mass=sumMass,
                                inertiaTensor = self.inertiaTensor + otherBodyInertia.inertiaTensor,
                                com=1./sumMass*(self.mass*self.com + otherBodyInertia.mass*otherBodyInertia.com))
        
    #shifts the com with vector vec and transforms the inertiaTensor to the new center of rotation
    def Translated(self, vec):
        #transform inertia to com=[0,0,0]
        inertia = self.inertiaTensor - self.mass*np.dot(Skew(self.com).transpose(),Skew(self.com))
        newCOM = self.com + vec
        inertia += self.mass*np.dot(Skew(newCOM).transpose(),Skew(newCOM))
        return RigidBodyInertia(mass=self.mass, 
                                inertiaTensor=inertia,
                                com=newCOM)

    #rotates the inertia tensor with transformation matrix rot; Jnew = rot*J*rot^T
    #only allowed if COM=0 !
    def Rotated(self, rot):
        if NormL2(self.com) != 0:
            print("ERROR: RigidBodyInertia.Rotated only allowed in case of com=0")
            return 0
        
        inertia = np.dot(rot,np.dot(self.inertiaTensor,rot.transpose()))
        return RigidBodyInertia(mass=self.mass, 
                                inertiaTensor=inertia,
                                com=self.com)

    #get vector with 6 inertia components: Jxx, Jyy, Jzz, Jyz, Jxz, Jxy
    def GetInertia6D(self):
        J = self.inertiaTensor
        return [J[0][0], J[1][1], J[2][2],  J[1][2], J[0][2], J[0][1]]

    def __str__(self):
        s = 'mass = ' + str(self.mass)
        s += '\nCOM = ' + str(self.com)
        s += '\ninertiaTensorAtOrigin = \n' + str(self.inertiaTensor)
        return s
    def __repr__(self):
        return str(self)


#set moment of inertia and mass of a cuboid with density and side lengths sideLengths along local axes 1, 2, 3; inertia w.r.t. center of mass, com=0
#example: InertiaCuboid(density=1000,sideLengths=[1,0.1,0.1])
class InertiaCuboid(RigidBodyInertia):
    def __init__(self, density, sideLengths):
        L1=sideLengths[0]
        L2=sideLengths[1]
        L3=sideLengths[2]
        newMass=density*L1*L2*L3
        RigidBodyInertia.__init__(self, mass=newMass,
                                  inertiaTensor=newMass/12.*np.diag([(L2**2 + L3**2),(L1**2 + L3**2),(L1**2 + L2**2)]),
                                  com=np.zeros(3))

#set moment of inertia and mass of a rod with mass m and length L in local 1-direction (x-direction); inertia w.r.t. center of mass, com=0
class InertiaRodX(RigidBodyInertia):
    def __init__(self, mass, length):
        RigidBodyInertia.__init__(self, mass=mass,
                                  inertiaTensor=mass/12.*np.diag([0.,length**2,length**2]),
                                  com=np.zeros(3))
        
#set moment of inertia and mass of mass point with 'mass'; inertia w.r.t. center of mass, com=0
class InertiaMassPoint(RigidBodyInertia):
    def __init__(self, mass):
        RigidBodyInertia.__init__(self, mass=mass,
                                  inertiaTensor=np.zeros([3,3]),
                                  com=np.zeros(3))

#set moment of inertia and mass of sphere with mass and radius; inertia w.r.t. center of mass, com=0
class InertiaSphere(RigidBodyInertia):
    def __init__(self, mass, radius):
        J = 2.*mass/5.*radius**2
        RigidBodyInertia.__init__(self, mass=mass,
                                  inertiaTensor=np.diag([J,J,J]),
                                  com=np.zeros(3))
        
#set moment of inertia and mass of hollow sphere with mass (concentrated at circumference) and radius; inertia w.r.t. center of mass, com=0
class InertiaHollowSphere(RigidBodyInertia):
    def __init__(self, mass, radius):
        J = 2.*mass/3.*radius**2
        RigidBodyInertia.__init__(self, mass=mass,
                                  inertiaTensor=np.diag([J,J,J]),
                                  com=np.zeros(3))

#set moment of inertia and mass of cylinder with density, length and outerRadius; axis defines the orientation of the cylinder axis (0=x-axis, 1=y-axis, 2=z-axis); for hollow cylinder use innerRadius != 0; inertia w.r.t. center of mass, com=0
class InertiaCylinder(RigidBodyInertia):
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
        
        
    


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: adds a node (with str(exu.NodeType. ...)) and body for a given rigid body
#either the initial rotation is given by the rotationMatrix (while rotationParameters=[]) or by rotationParameters (while rotationMatrix=[]) (non empty)
#position ... initial position, etc.
#all quantities (esp. velocity and angular velocity) are given in global coordinates!
#returns node number and body number
#adds gravity force, i.e., m*gravity
def AddRigidBody(mainSys, inertia, nodeType, 
                 position=[0,0,0], velocity=[0,0,0], 
                 rotationMatrix= [],
                 rotationParameters = [],
                 angularVelocity=[0,0,0],
                 gravity=[0,0,0],
                 graphicsDataList=[]):

    if len(rotationMatrix) != 0 and len(rotationParameters) != 0:
        raise ValueError('AddRigidBody: either rotationMatrix or rotationParameters must empty!')
    if len(rotationMatrix) == 0 and len(rotationParameters) == 0:
        rotationMatrix=np.diag([1,1,1])
        
    strNodeType = str(nodeType)
    nodeNumber = -1
    if strNodeType == 'NodeType.RotationEulerParameters':
        if len(rotationParameters) == 0:
            ep0 = RotationMatrix2EulerParameters(rotationMatrix)
        else:
            ep0 = rotationParameters
           
        ep_t0 = AngularVelocity2EulerParameters_t(angularVelocity, ep0)
        nodeNumber = mainSys.AddNode(NodeRigidBodyEP(referenceCoordinates=list(position)+list(ep0), 
                                                     initialVelocities=list(velocity)+list(ep_t0)))
    elif strNodeType == 'NodeType.RotationRxyz':
        if len(rotationParameters) == 0:
            rot0 = RotationMatrix2RotXYZ(rotationMatrix)
        else:
            rot0 = rotationParameters

        rot_t0 = AngularVelocity2RotXYZ_t(angularVelocity, rot0)
#        print('rot0=',rot0)
#        print('rot_t0=',rot_t0)
        nodeNumber = mainSys.AddNode(NodeRigidBodyRxyz(referenceCoordinates=list(position)+list(rot0), 
                                                       initialVelocities=list(velocity)+list(rot_t0)))
    elif strNodeType == 'NodeType.RotationRotationVector':
        if len(rotationParameters) == 0:
            #raise ValueError('NodeType.RotationRotationVector not implemented!')
            rot0 = RotationMatrix2RotationVector(rotationMatrix)
        else:
            rot0 = rotationParameters
        
        rotMatrix = RotationVector2RotationMatrix(rot0) #rotationMatrix needed!
        angularVelocityLocal = np.dot(rotMatrix.transpose(),angularVelocity)
            
        nodeNumber = mainSys.AddNode(NodeRigidBodyRotVecLG(referenceCoordinates=list(position) + list(rot0), 
                                                           initialVelocities=list(velocity)+list(angularVelocityLocal)))
    #if NormL2(inertia.com) != 0:
    #    print("AddRigidBody COM=", inertia.com)

    bodyNumber = mainSys.AddObject(ObjectRigidBody(physicsMass=inertia.mass, physicsInertia=inertia.GetInertia6D(), 
                                                   physicsCenterOfMass=inertia.com,
                                                   nodeNumber=nodeNumber, 
                                                   visualization=VObjectRigidBody(graphicsData=graphicsDataList)))
    
    if NormL2(gravity) != 0.:
        markerNumber = mainSys.AddMarker(MarkerBodyMass(bodyNumber=bodyNumber))
        mainSys.AddLoad(LoadMassProportional(markerNumber=markerNumber, loadVector=gravity))
    
    return [nodeNumber, bodyNumber]





    
