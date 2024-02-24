#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  A library for preparation of minimal coordinates (kinematic tree) formulation.
#           This library follows mostly the algorithms of Roy Featherstone, see http://royfeatherstone.org/
#           His code is availble in MATLAB as well as described in the Springer Handbook of Robotics \cite{Siciliano2016}.
#           The main formalisms are based on 6x6 matrices, so-called Pl\"ucker transformations, denoted as \ac{T66}, as defined by Featherstone.
#
# Author:   Johannes Gerstmayr
# Date:     2021-06-22
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#constants and fixed structures:
import numpy as np
#from math import pi, sin, cos#, sqrt
from copy import deepcopy

#import exudyn as exu
import exudyn.rigidBodyUtilities as erb
#from exudyn.robotics import *

#check with Corke Toolbox:
# https://github.com/petercorke/spatialmath-python
# https://github.com/petercorke/robotics-toolbox-python


#the following functions are defined here to fit into the original Featherstone algorithm
#rotations are transposed / inverse
def RotationX2T66Inverse(angle):
    return erb.RotationX2T66(angle).T

#the following functions are defined here to fit into the original Featherstone algorithm
#rotations are transposed / inverse
def RotationY2T66Inverse(angle):
    return erb.RotationY2T66(angle).T

#the following functions are defined here to fit into the original Featherstone algorithm
#rotations are transposed / inverse
def RotationZ2T66Inverse(angle):
    return erb.RotationZ2T66(angle).T

#compute inverse 6x6 transformation matrix for translation along X axis; output: first 3 components for rotation, second 3 components for translation!
def TranslationX2T66Inverse(translation):
    return erb.Translation2T66([-translation,0,0])

#compute inverse 6x6 transformation matrix for translation along Y axis; output: first 3 components for rotation, second 3 components for translation!
def TranslationY2T66Inverse(translation):
    return erb.Translation2T66([0,-translation,0])

#compute inverse 6x6 transformation matrix for translation along Z axis; output: first 3 components for rotation, second 3 components for translation!
def TranslationZ2T66Inverse(translation):
    return erb.Translation2T66([0,0,-translation])



#**function: convert mass, COM and inertia into 6x6 inertia matrix
#**input:
#  mass: scalar mass
#  centerOfMass: 3D vector (list/array)
#  inertia: 3x3 matrix (list of lists / 2D array) w.r.t. center of mass
#**output: 6x6 numpy array for further use in minimal coordinates formulation
def MassCOMinertia2T66(mass, centerOfMass, inertia):
    C = erb.Skew(centerOfMass)
    return np.block([
        [inertia + mass*(C @ C.T), mass*C],
        [mass*C.T, mass*np.eye(3) ]])

#**function: convert inertia as produced with RigidBodyInertia class into 6x6 inertia matrix (as used in KinematicTree66, Featherstone / Handbook of robotics \cite{Siciliano2016})
#**notes: within the 6x6 matrix, the inertia tensor is defined w.r.t.\ the center of mass, while RigidBodyInertia defines the inertia tensor w.r.t.\ the reference point; however, this function correctly transforms all quantities of inertia.
#**output: 6x6 numpy array for further use in minimal coordinates formulation
def Inertia2T66(inertia):
    C = erb.Skew(inertia.com)
    mass = inertia.mass
    #inertiaCOM = inertia.Translated(-inertia.com).inertiaTensor
    inertiaCOM = inertia.InertiaCOM()
    #alternatively, we could just use inertia.Inertia() in first block!
    return np.block([
        [inertiaCOM + mass*(C @ C.T), mass*C],
        [mass*C.T, mass*np.eye(3) ]])

#**function: convert 6x6 inertia matrix into mass, COM and inertia 
#**input: 6x6 numpy array containing rigid body inertia according to Featherstone / Handbook of robotics \cite{Siciliano2016}
#**output: [mass, centerOfMass, inertia]
#  mass: scalar mass
#  centerOfMass: 3D vector (list/array)
#  inertia: 3x3 matrix (list of lists / 2D array) w.r.t. center of mass
def Inertia66toMassCOMinertia(inertia66):
    mass = inertia66[5,5]
    massCOM = inertia66[0:3,3:6]
    centerOfMass = erb.Skew2Vec(massCOM)/mass
    inertia = inertia66[0:3,0:3] - massCOM @ massCOM.T/mass;

    return [mass, centerOfMass, inertia]


#define dictionary for joint transformations, as there is no switch case statement in Python
dictOfJointTransformMotionSubspace66 = {
    'Rx':[RotationX2T66Inverse,   np.array([1,0,0,0,0,0])], #revolute joint for local X axis
    'Ry':[RotationY2T66Inverse,   np.array([0,1,0,0,0,0])], #revolute joint for local Y axis
    'Rz':[RotationZ2T66Inverse,   np.array([0,0,1,0,0,0])], #revolute joint for local Z axis
    'Px':[TranslationX2T66Inverse, np.array([0,0,0,1,0,0])], #prismatic joint for local X axis
    'Py':[TranslationY2T66Inverse, np.array([0,0,0,0,1,0])], #prismatic joint for local Y axis
    'Pz':[TranslationZ2T66Inverse, np.array([0,0,0,0,0,1])], #prismatic joint for local Z axis
    #helical and planar joints available in Featherstone's implementation
    }

#**function: return 6x6 Pl\"ucker joint transformation matrix evaluated for scalar joint coordinate q and motion subspace ('free modes' in Table 2.6 in Handbook of robotics \cite{Siciliano2016})
def JointTransformMotionSubspace66(jointType, q):
    [T,MS] = dictOfJointTransformMotionSubspace66[jointType]
    return [T(q), MS]
        

#unit matrix needed for dictOfJointRotationMatrixAxis
# def UnitMatrix3D():
#     return np.eye(3)

#define dictionary for joint transformations, as there is no switch case statement in Python
#using rotation matrix, translation (for prismatic joints), joint axes (rot, trans)
dictOfJointRotationMatrixAxis = {
    'Rx':[erb.RotationMatrixX, np.array([0,0,0]), np.array([1,0,0]), np.array([0,0,0])], #revolute joint for local X axis
    'Ry':[erb.RotationMatrixY, np.array([0,0,0]), np.array([0,1,0]), np.array([0,0,0])], #revolute joint for local Y axis
    'Rz':[erb.RotationMatrixZ, np.array([0,0,0]), np.array([0,0,1]), np.array([0,0,0])], #revolute joint for local Z axis
    'Px':[0,               np.array([1,0,0]), np.array([0,0,0]), np.array([1,0,0])], #prismatic joint for local X axis
    'Py':[0,               np.array([0,1,0]), np.array([0,0,0]), np.array([0,1,0])], #prismatic joint for local Y axis
    'Pz':[0,               np.array([0,0,1]), np.array([0,0,0]), np.array([0,0,1])], #prismatic joint for local Z axis
    }

#**function: return list containing rotation matrix, translation vector, rotation axis and translation axis for joint transformation
def JointTransformMotionSubspace(jointType, q):
    [A, v, rotAxis, transAxis] = dictOfJointRotationMatrixAxis[jointType]
    if jointType[0] == 'R':
        return [A(q).T, v, rotAxis, transAxis] #transposed needed as compared to RotationMatrixX, as the algorithm requires coordinate transforms instead of active rotation matrices
    else: #prismatic
        return [np.eye(3), v*q, rotAxis, transAxis]
        

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#definition of a kinematic tree
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**class: class to define a kinematic tree in Python, which can be used for building serial or tree-structured multibody systems 
#         (or robots) with a minimal coordinates formulation, using rotation matrices and 3D offsets; for efficient computation, use the C++ ObjectKinematicTree
#**notes:
#   The formulation and structures widely follows the more efficient formulas (but still implemented in Python!) with 3D vectors and rotation matrices as proposed in Handbook of robotics \cite{Siciliano2016}, Chapter 3, but with the rotation matrices (\texttt{listOfRotations}) being transposed in the Python implementation as compared to the description in the book, being thus compliant with other Exudyn functions; the 3D vector/matrix Python implementation does not offer advantages as compared to the formulation with Pl\"ucker coordinates, BUT it reflects the formulas of the C++ implementation and is used for testing
class KinematicTree33:
    #**classFunction: initialize kinematic tree
    #**input:
    #  listOfJointTypes: mandatory list of joint types 'Rx', 'Ry', 'Rz' denoting revolute joints; 'Px', 'Py', 'Pz', denoting prismatic joints
    #  listOfRotations: per link rotation matrix, transforming coordinates of the joint coordinate system w.r.t. the previous coordinate system (this is the inverse of Pl\"ucker coordinate transforms (6x6))
    #  listOfOffsets: per link offset vector from pervious coordinate system to the joint coordinate system
    #  listOfInertia3D: per link 3D inertia matrix, w.r.t.\ reference point (not COM!)
    #  listOfCOM: per link vector from reference point to center of mass (COM), in link coordinates
    #  listOfMass: mass per link
    #  listOfParents: list of parent object indices (int), according to the index in jointTypes and transformations; use empty list for kinematic chain and use -1 if no parent exists (parent=base or world frame)
    #  gravity: a 3D list/array containing the gravity applied to the kinematic tree (in world frame)
    def __init__(self, 
                 listOfJointTypes,
                 listOfRotations,
                 listOfOffsets,
                 listOfInertia3D,
                 listOfCOM, 
                 listOfMass,
                 listOfParents=[],
                 gravity=[0,0,-9.81],
                 ):
        self.listOfJointTypes = listOfJointTypes
        self.listOfRotations = listOfRotations
        self.listOfOffsets = listOfOffsets
        self.listOfInertia3D = listOfInertia3D
        self.listOfCOM = listOfCOM
        self.listOfMass = listOfMass
        self.listOfParents = list(listOfParents)
        self.gravity = np.array(gravity)

        if len(self.listOfParents) == 0: #for kinematic chain, all joints are in a row
            self.listOfParents = np.arange(-1,len(self.listOfJointTypes)-1)
        else:
            self.listOfParents = np.array(self.listOfParents)

        if len(self.listOfJointTypes) != len(self.listOfRotations):
            raise ValueError('KinematicTree33: length of listOfJointTypes must agree with length of listOfRotations')
        if len(self.listOfJointTypes) != len(self.listOfOffsets):
            raise ValueError('KinematicTree33: length of listOfJointTypes must agree with length of listOfOffsets')
        if len(self.listOfJointTypes) != len(self.listOfParents):
            raise ValueError('KinematicTree33: length of listOfJointTypes must agree with length of parents (or parents may be empty)')
        if len(self.listOfJointTypes) != len(self.listOfInertia3D):
            raise ValueError('KinematicTree33: length of listOfJointTypes must agree with length of listOfInertia3D')
        if len(self.listOfJointTypes) != len(self.listOfMass):
            raise ValueError('KinematicTree33: length of listOfJointTypes must agree with length of listOfMass')
        if len(self.listOfJointTypes) != len(self.listOfCOM):
            raise ValueError('KinematicTree33: length of listOfJointTypes must agree with length of listOfCOM')

        for i in range(len(listOfParents)):
            if listOfParents[i] >= i:
                raise ValueError('KinematicTree33: link / parent indices must be sorted such that parent indices always refer to a link with an index smaller than the current link (parent[i] < i)')
    #**classFunction: return number of joints, defined by size of jointTypes
    def Size(self):
        return len(self.listOfJointTypes)
    
    #position of joint i is based on position of joint j, using offset %\pv_i%, $\LU{i}{\rv_i}=\LU{i}{\rv_j}+\LU{i}{\pv_i}$
    #rotation of joint i relative to joint j is $\LU{ij}{\Am}$, rotation axis $\jv_j$ transformed by $\LU{i}{\jv_j} = \LU{ij}{\Am} \LU{j}{\jv_j}$
    #**classFunction: return [A, p] containing rotation matrix and offset for joint j
    def XL(self,i):
        return [self.listOfRotations[i], self.listOfOffsets[i]]


    #**classFunction: compute forward dynamics using composite rigid body algorithm
    #**input:
    #  q: joint space coordinates for the model at which the forward dynamics is evaluated
    #  q_t: joint space velocity coordinates for the model at which the forward dynamics is evaluated
    #  torques: a vector of torques applied at joint coordinates or list/array with zero length
    #  forces: forces acting on the bodies using special format
    #**output: returns acceleration vector q\_tt of joint coordinates
    def ForwardDynamicsCRB(self, q=[], q_t=[], torques=[], forces=[]):
        if forces != []:
            raise ValueError('ForwardDynamicsCRB: forces not implemented')
        
        if len(torques) == 0:
            torques = np.zeros(len(q))
            
        [M, fGeneralized] = self.ComputeMassMatrixAndForceTerms(q, q_t)
        
        return np.linalg.solve(M, torques - fGeneralized)
    
    
    #**classFunction: compute generalized mass matrix M and generalized force terms for 
    #            kinematic tree, using current state (joint) variables q and 
    #            joint velocities q\_t. The generalized force terms f = fGeneralized
    #            contain Coriolis and gravity if given in the kinematicTree.
    #**input:
    #  q: current joint coordinates
    #  q_t: current joint velocities
    #  externalForces: list of torque/forces in global (world) frame per joint; may be empty list, containing 6D vectors or matrices with 6D vectors in columns that are summed up for each link
    #**output: mass matrix $\Mm$ and RHS vector $\fv_{RHS}$ for equations of motion $M(q) \cdot q_{tt} + f(q,q_t,externalForces) = \tau$; RHS is $\fv_{RHS}=\tau - f(q,q_t,externalForces)$; $\tau$ can be added outside of \texttt{ComputeMassMatrixAndForceTerms}
    def ComputeMassMatrixAndForceTerms(self, q, q_t, externalForces=[]):
        #gravity6D = np.hstack((np.zeros(3),self.gravity))
        n = self.Size()
        
        #initialize termporary matrices
        #Rot = [np.zeros((3,3))]*n       #store rotation matrices
        #Trans = [np.zeros(3)]*n     #store translation vectors
        MSrot = [np.zeros(3)]*n     #motion subspace for rotation
        MStrans = [np.zeros(3)]*n   #motion subspace for translation
        XupRot = [np.zeros((3,3))]*n    #store link-joint rotation matrices
        XupTrans = [np.zeros(3)]*n  #store link-joint translation vectors
        vRot = [np.zeros(3)]*n     #store 3D joint angular velocity vectors
        vTrans = [np.zeros(3)]*n     #store 3D joint velocity vectors
        avpRot = [np.zeros(3)]*n   #store 3D joint angular acceleration vectors (rotation)
        avpTrans = [np.zeros(3)]*n   #store 3D joint acceleration vectors (translation)
        fvpRot = [np.zeros(3)]*n   #store 3D joint torque vectors for every body
        fvpTrans = [np.zeros(3)]*n   #store 3D joint force vectors for every body
        
        #'recursively' compute transformations, accelerations and forces on parent joints
        for i in range(n):
            #[XJ, MS[i]] = JointTransformMotionSubspace(self.listOfJointTypes[i], q[i])
            [XJrot, XJtrans, MSrot[i], MStrans[i]] = JointTransformMotionSubspace(self.listOfJointTypes[i], q[i])

            #vJ = MS[i] * q_t[i]
            vJrot = MSrot[i] * q_t[i]
            vJtrans = MStrans[i] * q_t[i]

            #Xup[i] = XJ @ self.XL(i)
            [AL, tL] = self.XL(i)
            AL = AL.T #AL transposed!
            XupRot[i] = XJrot @ AL #X1*X2 = (R1*R2; p2+R2.T*p1)
            XupTrans[i] = tL + AL.T @ XJtrans 
            
            if self.listOfParents[i] == -1:
                #v[i] = vJ
                vRot[i] = vJrot
                vTrans[i] = vJtrans
                #avp[i] = Xup[i] @ (-gravity6D)
                avpRot[i] = np.zeros(3) #XupRot[i] @ omega = 0
                avpTrans[i] = XupRot[i] @ (-self.gravity)
            else:
                #v[i] = Xup[i] @ v[self.listOfParents[i]] + vJ
                vRot[i] = XupRot[i] @ vRot[self.listOfParents[i]] + vJrot
                vTrans[i] = XupRot[i] @ (vTrans[self.listOfParents[i]] - np.cross(XupTrans[i],vRot[self.listOfParents[i]]) ) + vJtrans
                #avp[i] = Xup[i] @ avp[self.listOfParents[i]] + CRM(v[i]) @ vJ
                avpRot[i] = XupRot[i] @ avpRot[self.listOfParents[i]] + np.cross(vRot[i], vJrot)
                avpTrans[i] = XupRot[i] @ (avpTrans[self.listOfParents[i]] - np.cross(XupTrans[i],avpRot[self.listOfParents[i]]) ) 
                avpTrans[i] += np.cross(vTrans[i], vJrot) + np.cross(vRot[i], vJtrans)
    
            # print('vRot['+str(i)+']=',  vRot[i].round(4))
            # print('vTra['+str(i)+']=',  vTrans[i].round(4))
            # print('avpRot['+str(i)+']=',avpRot[i].round(4))
            # print('avpTra['+str(i)+']=',avpTrans[i].round(4))
            #continue with adaptation herehere!
            #fvp[i] = self.inertias[i] @ avp[i] + CRF(v[i]) @ self.inertias[i] @ v[i] 
            J = self.listOfInertia3D[i]
            m = self.listOfMass[i]
            h = m * self.listOfCOM[i]
            JvRot = J @ vRot[i] + np.cross(h, vTrans[i])
            JvTrans = m * vTrans[i] - np.cross(h, vRot[i])
            
            fvpRot[i] = J @ avpRot[i] + np.cross(h, avpTrans[i])
            fvpRot[i] += np.cross(vRot[i], JvRot) + np.cross(vTrans[i], JvTrans)#CRF(v[i]) @ self.inertias[i] @ v[i] 
            fvpTrans[i] = m * avpTrans[i] - np.cross(h, avpRot[i])
            fvpTrans[i] += np.cross(vRot[i], JvTrans)  #CRF(v[i]) @ self.inertias[i] @ v[i] 
    
        # print('fvpRot =', fvpRot)
        # print('fvpTrans =', fvpTrans)
        #add external foces
        #to be implemented
        #fvp = self.AddExternalForces(Xup, fvp, externalForces)
    
        f = np.zeros(n)             #store joint forces
        M = np.zeros((n,n))         #store mass matrix
        #compute joint forces; REVERSED loop!
        for i in reversed(range(n)):
            f[i] = MSrot[i].T @ fvpRot[i] + MStrans[i].T @ fvpTrans[i]
            if self.listOfParents[i] != -1:
                #fvp[self.listOfParents[i]] += Xup[i].T @ fvp[i]
                R = XupRot[i]
                p = XupTrans[i]
                fvpRot[self.listOfParents[i]] += R.T @ fvpRot[i] + np.cross(p, R.T@fvpTrans[i])
                fvpTrans[self.listOfParents[i]] += R.T @ fvpTrans[i]

        #compute composite inertia
        #IC = deepcopy(self.inertias) #problems if identical np.array() used for several inertias!!!
        ICinertia = [np.zeros((3,3))]*n
        ICh = [np.zeros(3)]*n    
        ICm = [0]*n    

        for i in range(n):
            ICinertia[i] = deepcopy(self.listOfInertia3D[i]) #deepcopy very slow!
            ICm[i] = deepcopy(self.listOfMass[i])
            ICh[i] = ICm[i] * deepcopy(self.listOfCOM[i])

        # for i in range(n):
        #     ICh[i] = ICm[i] * ICh[i]  #compute h=m*com
        
        for i in reversed(range(n)):
            if self.listOfParents[i] != -1:
                #IC[self.listOfParents[i]] += Xup[i].T @ IC[i] @ Xup[i]
                R = XupRot[i]
                p = XupTrans[i]
                ICm[self.listOfParents[i]] += ICm[i]
                ICh[self.listOfParents[i]] += R.T @ ICh[i] + ICm[i]*p
                ICinertia[self.listOfParents[i]] += R.T @ ICinertia[i] @ R - erb.Skew(p)@erb.Skew(R.T @ ICh[i]) - erb.Skew(R.T@ICh[i] + ICm[i]*p) @ erb.Skew(p)

        # for i in range(n):
        #     print('IC'+str(i)+'m=',ICm[i])
        #     print('ICh'+str(i)+'=',ICh[i])
        #     print('ICinertia'+str(i)+'=',ICinertia[i])
    
        #compute generalized mass matrix and projected inertia
        for i in range(n):
            #fh = IC[i] @ MS[i]
            fhRot = ICinertia[i] @ MSrot[i] + np.cross(ICh[i], MStrans[i])
            fhTrans = m*MStrans[i] - np.cross(ICh[i], MSrot[i])
            #M[i,i] = MS[i].T @ fh
            M[i,i] = MSrot[i].T @ fhRot + MStrans[i].T @ fhTrans
            j = i
            while self.listOfParents[j] > -1:
                #fh = Xup[j].T @ fh
                R = XupRot[j]
                p = XupTrans[j]
                fhRot = R.T @ fhRot + np.cross(p, R.T@fhTrans)
                fhTrans = R.T @ fhTrans
                j = self.listOfParents[j]
                #M[i,j] = MS[j].T @ fh
                M[i,j] = MSrot[j].T @ fhRot + MStrans[j].T @ fhTrans
                M[j,i] = M[i,j]
    
        return [M, f]
    
    
    # #**classFunction: add action of external forces to forces fvp and return new composed vector of forces fvp
    # #**input:
    # #  Xup: 6x6 transformation matrices per joint; as computed in ComputeMassMatrixAndForceTerms
    # #  fvp: force (torque) per joint, as computed in ComputeMassMatrixAndForceTerms
    # #  externalForces: list of torque/forces in global (world) frame per joint; may be empty list, containing 6D vectors or matrices with 6D vectors in columns that are summed up for each link
    # def AddExternalForces(self, Xup, fvp, externalForces=[]):
    #     fvpOut = fvp #np.zeros(n)
    #     if len(externalForces): #only consider forces, if not empty list
    #         n = len(self.listOfParents)
    #         Xa = [np.zeros((6,6))]*n
    #         for i in range(n):
    #             if self.listOfParents[i] == -1:
    #                 Xa[i] = Xup[i]
    #             else:
    #                 Xa[i] = Xup[i] * Xa[self.listOfParents[i]];
    #             if len(externalForces[i].shape) == 1:
    #                 fvpOut[i] += np.linalg.solve(Xa[i].T, externalForces[i])
    #             else:
    #                 raise ValueError('KinematicTree33.AddExternalForces: unchecked code for multiple forces')
    #                 for j in range(len(externalForces[i].shape[1])): #loop over several vectors
    #                     fvpOut[i] += np.linalg.solve(Xa[i].T, externalForces[i][:,j])
    
    #     return fvpOut
        
    














#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: computes cross product operator for motion from 6D vector v; CRM(v) @ m computes the cross product of v and motion m
def CRM(v):
      return np.array([[0   , -v[2] , v[1]  , 0    , 0    , 0    ],
                       [ v[2],  0    ,-v[0]  , 0    , 0    , 0    ],
                       [-v[1],  v[0] , 0     , 0    , 0    , 0    ],
                       [ 0   , -v[5] , v[4]  , 0    ,-v[2] , v[1] ],
                       [ v[5],  0    ,-v[3]  , v[2] , 0    ,-v[0] ],
                       [-v[4],  v[3] , 0     ,-v[1] , v[0] , 0    ] ])

#**function: computes cross product operator for force from 6D vector v; CRF(v) @ f computes the cross product of v and force f
def CRF(v):
    return -CRM(v).T



#definition of a kinematic tree
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**class: class to define a kinematic tree, which can be used for building serial or tree-structured multibody systems 
#         (or robots) with a minimal coordinates formulation, using Pl\"ucker coordinate transforms (6x6); for efficient computation, use the C++ ObjectKinematicTree
#**notes:
#   The formulation and structures widely follow Roy Featherstone (http://royfeatherstone.org/) / Handbook of robotics \cite{Siciliano2016} 
class KinematicTree66:
    #**classFunction: initialize kinematic tree
    #**input:
    #  listOfJointTypes: mandatory list of joint types 'Rx', 'Ry', 'Rz' denoting revolute joints; 'Px', 'Py', 'Pz', denoting prismatic joints
    #  listOfTransformations: provide a list of Pl\"ucker coordinate transforms (6x6 numpy matrices), describing the (constant) link transformation from the link coordinate system (previous/parent joint) to this joint coordinate system
    #  listOfInertias: provide a list of inertias as (6x6 numpy matrices), as produced by the function MassCOMinertia2T66
    #  listOfParents: list of parent object indices (int), according to the index in jointTypes and transformations; use empty list for kinematic chain and use -1 if no parent exists (parent=base or world frame)
    #  gravity: a 3D list/array containing the gravity applied to the kinematic tree (in world frame)
    def __init__(self, 
                 listOfJointTypes,
                 listOfTransformations, 
                 listOfInertias, 
                 listOfParents=[],
                 gravity=[0,0,-9.81],
                 ):
        self.jointTypes = listOfJointTypes
        self.transformations = listOfTransformations
        self.inertias = listOfInertias
        self.parents = listOfParents
        self.gravity = np.array(gravity)

        if len(self.parents) == 0: #for kinematic chain, all joints are in a row
            self.parents = np.arange(-1,len(self.jointTypes)-1)
        else:
            self.parents = np.array(self.parents)

        if len(self.jointTypes) != len(self.transformations):
            raise ValueError('KinematicTree66: length of jointTypes must agree with length of transformations')
        if len(self.jointTypes) != len(self.parents):
            raise ValueError('KinematicTree66: length of jointTypes must agree with length of parents (or parents may be empty)')
        if len(self.jointTypes) != len(self.inertias):
            raise ValueError('KinematicTree66: length of jointTypes must agree with length of inertias')

        for i in range(len(listOfParents)):
            if listOfParents[i] >= i:
                raise ValueError('KinematicTree66: link / parent indices must be sorted such that parent indices always refer to a link with an index smaller than the current link (parent[i] < i)')
    #**classFunction: return number of joints, defined by size of jointTypes
    def Size(self):
        return len(self.jointTypes)
    
    #**classFunction: return 6D transformation of joint i, given by transformation
    def XL(self,i):
        return self.transformations[i]


    #**classFunction: compute forward dynamics using composite rigid body algorithm
    #**input:
    #  q: joint space coordinates for the model at which the forward dynamics is evaluated
    #  q_t: joint space velocity coordinates for the model at which the forward dynamics is evaluated
    #  torques: a vector of torques applied at joint coordinates or list/array with zero length
    #  forces: forces acting on the bodies using special format
    #**output: returns acceleration vector q\_tt of joint coordinates
    def ForwardDynamicsCRB(self, q=[], q_t=[], torques=[], forces=[]):
        if forces != []:
            raise ValueError('ForwardDynamicsCRB: forces not implemented')
        
        if len(torques) == 0:
            torques = np.zeros(len(q))
            
        [M, fGeneralized] = self.ComputeMassMatrixAndForceTerms(q, q_t)
        
        return np.linalg.solve(M, torques - fGeneralized)
    
    
    #**classFunction: compute generalized mass matrix M and generalized force terms for 
    #            kinematic tree, using current state (joint) variables q and 
    #            joint velocities q\_t. The generalized force terms f = fGeneralized
    #            contain Coriolis and gravity if given in the kinematicTree.
    #**input:
    #  q: current joint coordinates
    #  q_t: current joint velocities
    #  externalForces: list of torque/forces in global (world) frame per joint; may be empty list, containing 6D vectors or matrices with 6D vectors in columns that are summed up for each link
    #**output: mass matrix $\Mm$ and RHS vector $\fv_{RHS}$ for equations of motion $M(q) \cdot q_{tt} + f(q,q_t,externalForces) = \tau$; RHS is $\fv_{RHS}=\tau - f(q,q_t,externalForces)$; $\tau$ can be added outside of \texttt{ComputeMassMatrixAndForceTerms}
    def ComputeMassMatrixAndForceTerms(self, q, q_t, externalForces=[]):
        gravity6D = np.hstack((np.zeros(3),self.gravity))
        n = self.Size()
        
        #initialize termporary matrices
        MS = [np.zeros((6))]*n      #store motion supspaces
        Xup = [np.zeros((6,6))]*n   #store link-joint transformation matrices
        v = [np.zeros(6)]*n     #store 6D joint velocity vectors
        fvp = [np.zeros(6)]*n   #store 6D joint force vectors for every body
        avp = [np.zeros(6)]*n   #store 6D joint acceleration vectors
        
        #'recursively' compute transformations, accelerations and forces on parent joints
        for i in range(n):
            [XJ, MS[i]] = JointTransformMotionSubspace66(self.jointTypes[i], q[i])
            vJ = MS[i] * q_t[i]
            Xup[i] = XJ @ self.XL(i)
            if self.parents[i] == -1:
                v[i] = vJ
                avp[i] = Xup[i] @ (-gravity6D)
            else:
                v[i] = Xup[i] @ v[self.parents[i]] + vJ
                avp[i] = Xup[i] @ avp[self.parents[i]] + CRM(v[i]) @ vJ
    
            fvp[i] = self.inertias[i] @ avp[i] + CRF(v[i]) @ self.inertias[i] @ v[i] 

            # print("Xup"+str(i)+"=",Xup[i])
            # print("MS"+str(i)+"=",MS[i])
            # print("IC"+str(i)+"=",self.inertias[i])
            # print("CRF"+str(i)+"=",CRF(v[i]))
            # print("v"+str(i)+"=",v[i])
            # print("avp"+str(i)+"=",avp[i])
            # print('fvp['+str(i)+']=',fvp[i].round(8))
    
        #add external foces
        fvp = self.AddExternalForces(Xup, fvp, externalForces)
    
        f = np.zeros(n)             #store joint forces
        M = np.zeros((n,n))         #store mass matrix
        #compute joint forces; REVERSED loop!
        for i in reversed(range(n)):
            f[i] = MS[i].T @ fvp[i]
            if self.parents[i] != -1:
                fvp[self.parents[i]] += Xup[i].T @ fvp[i]
    
        #compute composite inertia
        #IC = deepcopy(self.inertias) #may cause problems if referenced np.arrays used!
        IC = [np.zeros((6,6))]*n

        for i in range(n):
            IC[i] = deepcopy(self.inertias[i])
        
        for i in reversed(range(n)):
            if self.parents[i] != -1:
                IC[self.parents[i]] += Xup[i].T @ IC[i] @ Xup[i]

        #compute generalized mass matrix and projected inertia
        for i in range(n):
            fh = IC[i] @ MS[i]
            M[i,i] = MS[i].T @ fh
            j = i
            while self.parents[j] > -1:
                fh = Xup[j].T @ fh
                j = self.parents[j]
                M[i,j] = MS[j].T @ fh
                M[j,i] = M[i,j]
    
        return [M, f]
    
    
    #**classFunction: add action of external forces to forces fvp and return new composed vector of forces fvp
    #**input:
    #  Xup: 6x6 transformation matrices per joint; as computed in ComputeMassMatrixAndForceTerms
    #  fvp: force (torque) per joint, as computed in ComputeMassMatrixAndForceTerms
    #  externalForces: list of torque/forces in global (world) frame per joint; may be empty list, containing 6D vectors or matrices with 6D vectors in columns that are summed up for each link
    def AddExternalForces(self, Xup, fvp, externalForces=[]):
        fvpOut = fvp #np.zeros(n)
        if len(externalForces): #only consider forces, if not empty list
            n = len(self.parents)
            Xa = [np.zeros((6,6))]*n
            for i in range(n):
                if self.parents[i] == -1:
                    Xa[i] = Xup[i]
                else:
                    Xa[i] = Xup[i] * Xa[self.parents[i]];
                if len(externalForces[i].shape) == 1:
                    fvpOut[i] += np.linalg.solve(Xa[i].T, externalForces[i])
                else:
                    raise ValueError('KinematicTree66.AddExternalForces: unchecked code for multiple forces')
                    for j in range(len(externalForces[i].shape[1])): #loop over several vectors
                        fvpOut[i] += np.linalg.solve(Xa[i].T, externalForces[i][:,j])
    
        return fvpOut
        
    

