#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library for robotics
#
# Details:  A library which includes support functions for robotics;
#           the library is built on standard Denavit-Hartenberg Parameters and
#           Homogeneous Transformations (HT) to describe transformations and coordinate systems;
#           import this library e.g. with import exudyn.robotics as robotics
#
# Author:   Johannes Gerstmayr
# Date:     2020-04-14
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#constants and fixed structures:
from exudyn.misc.docmeta import docmeta
from exudyn.misc.deprecation import Deprecated #the deprecations of the library (#2807)
import numpy as np

import exudyn

import exudyn.itemInterface as eii

import exudyn.rigidBodyUtilities as erb
import exudyn.graphics as graphics

from copy import copy, deepcopy
import exudyn.cMath as cMath #the math functions of the C++ code, the same bits on every platform (#2962)
import exudyn.cUtilities #KinematicChain, the kinematics of Robot in C++ (#3259)


#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'dictJointTypeText2Exudyn', 'dictJointType2HT', 'dictJointType2Axis',
    'dictJointType2coordinate6D', 'VRobotLink', 'RobotLink', 'VRobotTool', 'RobotTool',
    'VRobotBase', 'RobotBase', 'robotDictionaryVersion', 'Robot', 'StdDH2HT', 'ModDHKK2HT',
    'projectAngleToPMPi', 'InverseKinematicsNumerical',
    ]

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  Define robot link +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define convert text named joint types to exudyn joint types
dictJointTypeText2Exudyn = {
    'Rx':exudyn.JointType.RevoluteX, #revolute joint for local X axis
    'Ry':exudyn.JointType.RevoluteY, #revolute joint for local Y axis
    'Rz':exudyn.JointType.RevoluteZ, #revolute joint for local Z axis
    'Px':exudyn.JointType.PrismaticX, #prismatic joint for local X axis
    'Py':exudyn.JointType.PrismaticY, #prismatic joint for local Y axis
    'Pz':exudyn.JointType.PrismaticZ, #prismatic joint for local Z axis
    }

#define dictionary for joint transformations as homogeneous transformations (exudyn.HT), replacement for switch/case
dictJointType2HT = {
    'Rx':lambda angle: exudyn.HT().SetRotationX(angle), #revolute joint for local X axis
    'Ry':lambda angle: exudyn.HT().SetRotationY(angle), #revolute joint for local Y axis
    'Rz':lambda angle: exudyn.HT().SetRotationZ(angle), #revolute joint for local Z axis
    'Px':lambda x: exudyn.HT(translation=[x,0.,0.]), #prismatic joint for local X axis
    'Py':lambda y: exudyn.HT(translation=[0.,y,0.]), #prismatic joint for local Y axis
    'Pz':lambda z: exudyn.HT(translation=[0.,0.,z]), #prismatic joint for local Z axis
    }

def _ToHT(H):
    """an exudyn.HT from an exudyn.HT (the same object), a 4x4 matrix or None (the identity); the robotics classes store
    and compute with exudyn.HT (#2821)"""
    if isinstance(H, exudyn.HT):
        return H
    if H is None:
        return exudyn.HT()
    return exudyn.HT(np.array(H, dtype=float))

#define dictionary for joint transformations as homogeneous transformations, replacement for switch/case
dictJointType2Axis = {
    'Rx':np.array([1,0,0]), #revolute joint for local X axis
    'Ry':np.array([0,1,0]), #revolute joint for local Y axis
    'Rz':np.array([0,0,1]), #revolute joint for local Z axis
    'Px':np.array([1,0,0]), #prismatic joint for local X axis
    'Py':np.array([0,1,0]), #prismatic joint for local Y axis
    'Pz':np.array([0,0,1]), #prismatic joint for local Z axis
    }

#maps joint type to (constrained) coordinate 0..2: translation, 3..5: rotation
dictJointType2coordinate6D = {
    'Px':0, #revolute joint for local X axis
    'Py':1, #revolute joint for local Y axis
    'Pz':2, #revolute joint for local Z axis
    'Rx':3, #prismatic joint for local X axis
    'Ry':4, #prismatic joint for local Y axis
    'Rz':5, #prismatic joint for local Z axis
}

#RobotLink changes 2021-08-16:
#localerb.HT->localerb.HT (does not include rotation); this joint to this link erb.HT
#add preerb.HT: previous link to this joint erb.HT
#dictJointType2erb.HT[link.jointType](q[i]): defines joint erb.HT
#localerb.HT in MBS modeling (inertia, etc.) is kept same, while localerb.HT = ID4 for ModDH
#preerb.HT must be added for joint axes definitions and for currentHT
#inertia and COM need to be converted by localHT of StdDH into ModDH configuration!!!

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the analytic inverse kinematics (#3260): the robot as a product of exponentials, T(q) = E_1(q_1) ... E_n(q_n) T(0),
#E_i the rotation about the axis of joint i at q = 0 (direction omega_i through the point p_i); the subproblems of
#Paden and Kahan solve it for the geometries of _IKGeometry; KinematicChain.InverseKinematicsAnalytic is the C++
#version of the same algorithm

def _IKRotate(axis, angle, v):
    """v rotated about the unit vector axis by angle (Rodrigues)"""
    c = cMath.cos(angle)
    s = cMath.sin(angle)
    return v*c + np.cross(axis, v)*s + axis*(axis @ v)*(1.-c)

def _IKRotationMatrix(axis, angle):
    """the rotation matrix about the unit vector axis by angle"""
    return np.column_stack([_IKRotate(axis, angle, e) for e in np.eye(3)])

def _IKAngle(axis, u, v, fallback):
    """subproblem 1: the angle about axis that rotates u into v, both projected onto the plane normal to axis;
    fallback if a projection vanishes (any angle does it, a singular configuration)"""
    up = u - axis*(axis @ u)
    vp = v - axis*(axis @ v)
    if cMath.norm(up) < 1e-12*(1.+cMath.norm(u)) or cMath.norm(vp) < 1e-12*(1.+cMath.norm(v)):
        return fallback
    return cMath.atan2(axis @ np.cross(up, vp), up @ vp)

def _IKAnglesCosSin(a, b, e, fallback, tolerance):
    """the angles x of a cos(x) + b sin(x) = e: two, or none if |e| > |(a, b)|, or fallback if a = b = e = 0"""
    r = cMath.sqrt(a*a + b*b)
    if r < tolerance:
        return [fallback] if abs(e) < tolerance else []
    x = e/r
    if abs(x) > 1.+1e-10:
        return []
    phi = cMath.atan2(b, a)
    d = cMath.acos(min(1., max(-1., x)))
    return [phi + d, phi - d]

def _IKArm(omega, p, W0, X, fallback, tolerance):
    """the joint angles q_1, q_2, q_3 that move the point W0 (fixed in link 3, at q = 0) to X, for the axes 2 and 3
    parallel and axis 1 not parallel to them: q_1 puts X into the plane normal to axis 2 in which links 2 and 3 move W0,
    subproblem 3 gives q_3 from the distance to axis 2, subproblem 1 gives q_2; up to 4 solutions"""
    n = omega[1]
    w = omega[0]
    d = X - p[0]
    wd = w @ d
    q1List = _IKAnglesCosSin(n @ d - wd*(w @ n), -(n @ np.cross(w, d)), n @ (W0 - p[0]) - wd*(w @ n), fallback[0], tolerance)
    solutions = []
    for q1 in q1List:
        for (q2, q3) in _IKPlanar2R(omega[1], omega[2], p[1], p[2], W0, p[0] + _IKRotate(w, -q1, d), fallback, tolerance):
            solutions += [[q1, q2, q3]]
    return solutions

def _IKPlanar2R(omega2, omega3, p2, p3, W0, Y, fallback, tolerance):
    """the angles q_2, q_3 about the parallel axes 2 and 3 that move the point W0 to Y: subproblem 3 for q_3, the
    distance of the point from axis 2, then subproblem 1 for q_2; up to 2 solutions"""
    def Projected(x):
        return x - omega2*(omega2 @ x)
    u = Projected(W0 - p3)
    v = Projected(p2 - p3)
    delta = cMath.norm(Projected(Y - p2))
    nu = cMath.norm(u)
    nv = cMath.norm(v)
    if nu < tolerance or nv < tolerance:
        return []
    kappa = (nu*nu + nv*nv - delta*delta)/(2.*nu*nv)
    if abs(kappa) > 1.+1e-10:
        return []
    alpha = cMath.atan2(omega3 @ np.cross(v, u), v @ u)
    d = cMath.acos(min(1., max(-1., kappa)))
    solutions = []
    for q3 in [-alpha + d, -alpha - d]:
        Z = p3 + _IKRotate(omega3, q3, W0 - p3)
        solutions += [[_IKAngle(omega2, Z - p2, Y - p2, fallback[1]), q3]]
    return solutions

def _IKWrist(omega4, omega5, omega6, R, fallback):
    """the angles q_4, q_5, q_6 of R_4 R_5 R_6 = R for three axes through one point: subproblem 2 for q_4, q_5, which
    turn omega_6 into R omega_6, then subproblem 1 for q_6; up to 2 solutions"""
    u = omega6
    v = R @ omega6
    c = omega4 @ omega5
    cross = np.cross(omega4, omega5)
    if cMath.norm(cross) < 1e-12:
        return []
    alpha = (omega4 @ v - c*(omega5 @ u))/(1.-c*c)
    beta = (omega5 @ u - c*(omega4 @ v))/(1.-c*c)
    gamma2 = (u @ u - alpha*alpha - beta*beta - 2.*alpha*beta*c)/(cross @ cross)
    if gamma2 < -1e-10:
        return []
    gamma = cMath.sqrt(max(0., gamma2))
    #a vector normal to omega6 for q_6
    x = np.cross(omega6, cross) if cMath.norm(np.cross(omega6, cross)) > 1e-6 else np.cross(omega6, omega4)
    solutions = []
    for g in [gamma, -gamma]:
        z = alpha*omega4 + beta*omega5 + g*cross
        q5 = _IKAngle(omega5, u, z, fallback[4])
        q4 = _IKAngle(omega4, z, v, fallback[3])
        y = _IKRotationMatrix(omega5, q5).T @ _IKRotationMatrix(omega4, q4).T @ R @ x
        solutions += [[q4, q5, _IKAngle(omega6, x, y, fallback[5])]]
    return solutions

def _IKLineIntersection(p0, a0, p1, a1):
    """the point nearest to the lines p0 + s a0 and p1 + t a1 (unit vectors), and their distance; None for parallel lines"""
    w = p0 - p1
    b = a0 @ a1
    if 1.-b*b < 1e-12:
        return (None, np.inf)
    s = (b*(a1 @ w) - a0 @ w)/(1.-b*b)
    t = (a1 @ w - b*(a0 @ w))/(1.-b*b)
    x0 = p0 + s*a0
    x1 = p1 + t*a1
    return (0.5*(x0 + x1), cMath.norm(x0 - x1))


class VRobotLink:
    """class to define visualization of RobotLink
    """
    def __init__(self, jointRadius = 0.06, jointWidth = 0.12, linkWidth = 0.1, showMBSjoint = True, showCOM = True, 
                 linkColor = [0.4,0.4,0.4,1], graphicsData = [] ):
        """initialize robot link with parameters, being self-explaining

        Args:
            jointRadius: radius of joint to draw
            jointWidth: length or width of joint (depending on type of joint)
            showMBSjoint: if False, joint is not drawn
            linkWidth: width of link for default drawing
            linkColor: color of link for default drawing
            showCOM: if True, center of mass is marked with cube
            graphicsData: list of GraphicsData to represent link; if list is empty, link graphics will be generated from link geometry data; otherwise, drawing will be taken from graphicsData, and only showMBSjoint and showCOM flags will add additional graphics
        """
        self.jointRadius = jointRadius
        self.jointWidth = jointWidth 
        self.showMBSjoint = showMBSjoint
        self.linkWidth = linkWidth 
        self.linkColor = linkColor
        self.showCOM = showCOM
        self.graphicsData = graphicsData

    def __str__(self):
        s = '  jointRadius = ' + str(self.jointRadius)
        s += '\n  jointWidth = ' + str(self.jointWidth)
        s += '\n  showMBSjoint = ' + str(self.showMBSjoint)
        s += '\n  linkWidth = ' + str(self.linkWidth)
        s += '\n  linkColor = ' + str(self.linkColor)
        s += '\n  showCOM = ' + str(self.showCOM)
        gDataStr = ''
        if len(self.graphicsData) != 0:
            gDataStr = '...'
        s += '\n  graphicsData = [' + gDataStr + ']'
            
        return s
    def __repr__(self):
        return str(self)


class RobotLink:
    """class to define one link of a robot
    """
    def __init__(self, mass, COM, inertia, localHT=None, jointType='Rz', parent=-2, preHT=None, PDcontrol=(None,None), visualization=VRobotLink(),
                 jointLimits=None, viscousFriction=0., coulombFriction=0., endStopStiffness=0., endStopDamping=0.):
        """initialize robot link

        Args:
            mass: mass of robot link
            COM: center of mass in link coordinate system
            inertia: 3x3 matrix (list of lists / numpy array) containing inertia tensor in link coordinates, with respect to center of mass
            localHT: exudyn.HT or 4x4 matrix (list of lists / numpy array) containing homogeneous transformation from local joint to link coordinates, stored as exudyn.HT; default (None) = identity; currently, this transformation is not available in KinematicTree, therefore the link inertia and COM must be transformed accordingly
            preHT: exudyn.HT or 4x4 matrix (list of lists / numpy array) containing homogeneous transformation from previous link to this joint, stored as exudyn.HT; default (None) = identity
            jointType: string containing joint type, out of: 'Rx', 'Ry', 'Rz' for revolute joints and 'Px', 'Py', 'Pz' for prismatic joints around/along the respecitive local axes
            parent: for building robots as kinematic tree; use '-2' to automatically set parents for serial robot (on fixed base), use '-1' for ground-parent and any other 0-based index for connection to parent link
            PDcontrol: tuple of P and D control values, defining position (rotation) proportional value P and velocitiy proportional value D
            visualization: VRobotLink structure containing options for drawing of link and joints; see class VRobotLink
            jointLimits: None or [lower, upper], the range of the joint coordinate (rad or m); the end stops of the models of Robot(enableEndStops=True) act outside of it
            viscousFriction: the viscous friction of the joint, a torque (force) per joint velocity, in Nm/(rad/s) or N/(m/s)
            coulombFriction: the Coulomb friction torque (force) of the joint, in Nm or N, regularized below Robot.frictionRegularization
            endStopStiffness: the stiffness of the end stops at the jointLimits, in Nm/rad or N/m
            endStopDamping: the damping of the end stops, in Nm/(rad/s) or N/(m/s)
        """
        self.mass = mass
        self.COM = np.array(COM)
        self.inertia = np.array(inertia)
        self.localHT = localHT
        self.preHT = preHT
        self.jointType = jointType
        self.parent = parent
        self.visualization = deepcopy(visualization)
        if PDcontrol[0] is not None:
            self.PDcontrol = PDcontrol
        #friction and end stops, realized in both models of Robot by a CoordinateSpringDamperExt on the joint coordinate (#3270)
        self.jointLimits = None if jointLimits is None else [float(jointLimits[0]), float(jointLimits[1])]
        self.viscousFriction = viscousFriction
        self.coulombFriction = coulombFriction
        self.endStopStiffness = endStopStiffness
        self.endStopDamping = endStopDamping

    @property
    def localHT(self):
        """transformation from local joint to link coordinates, an exudyn.HT"""
        return self._localHT

    @localHT.setter
    def localHT(self, value):
        self._localHT = _ToHT(value)

    @property
    def preHT(self):
        """transformation from previous link to this joint, an exudyn.HT"""
        return self._preHT

    @preHT.setter
    def preHT(self, value):
        self._preHT = _ToHT(value)

    def SetPDcontrol(self, Pvalue, Dvalue):
        """set PD control values for drive of joint related to link using position-proportional value P and differential value (velocity proportional) D
        """
        self.PDcontrol = (Pvalue, Dvalue)

    def HasPDcontrol(self):
        """check if contrl is available
        """
        return hasattr(self, 'PDcontrol')

    def GetPDcontrol(self):
        """get PD control values
        """
        if not hasattr(self, 'PDcontrol'):
            raise ValueError('RobotLink: PDcontrol is not defined for link! Use SetPDcontrol to define parameters before using')
        return self.PDcontrol

    def __str__(self):
        s = '  mass = ' + str(self.mass)
        s += '\n  COM = ' + str(self.COM)
        s += '\n  inertia = ' + str(self.inertia)
        s += '\n  localHT = ' + str(self.localHT)
        s += '\n  preHT = ' + str(self.preHT)
        s += '\n  jointType = ' + str(self.jointType)
        s += '\n  parent = ' + str(self.parent)
        s += '\n  jointLimits = ' + str(self.jointLimits)
        s += '\n  friction (viscous, Coulomb) = ' + str((self.viscousFriction, self.coulombFriction))
        s += '\n  end stop (stiffness, damping) = ' + str((self.endStopStiffness, self.endStopDamping))
        s += '\n  visualization:\n' + str(self.visualization)
        if self.HasPDcontrol():
            s += '\n  PDcontrol = ' + str(self.PDcontrol)
        return s

    def __repr__(self):
        return str(self)
        
class VRobotTool:
    """class to define visualization of RobotTool
    """
    def __init__(self, graphicsData=[]):
        """initialize robot tool with parameters; currently only graphicsData, which is a list of GraphicsData same as in mbs Objects
        """
        self.graphicsData = copy(graphicsData)

class RobotTool:
    """define tool of robot: containing graphics and HT (may add features in future)
    """
    def __init__(self, HT=None, visualization=None):
        """initialize robot tool

        Args:
            HT: exudyn.HT or 4x4 matrix (list of lists / numpy array) containing homogeneous transformation to transform from the frame of the last link (after its localHT, as Robot.LinkHT) to the tool, stored as exudyn.HT; default (None) = identity
            graphicsData: dictionary containing a list of GraphicsData, same as in exudyn Objects
        """
        self.HT = HT
        self.visualization = VRobotTool() if visualization is None else visualization #a new one per tool: a default argument would be shared by all tools

    @property
    def HT(self):
        """the homogeneous transformation, an exudyn.HT"""
        return self._HT

    @HT.setter
    def HT(self, value):
        self._HT = _ToHT(value)
        
    def __str__(self):
        s = '  HT = ' + str(self.HT)
        return s
    def __repr__(self):
        return str(self)

class VRobotBase:
    """class to define visualization of RobotBase
    """
    def __init__(self, graphicsData=[]):
        """initialize robot base with parameters; currently only graphicsData, which is a list of GraphicsData same as in mbs Objects
        """
        self.graphicsData = copy(graphicsData)

    
class RobotBase:
    """define base of robot: containing graphics and HT (may add features in future)
    """
    def __init__(self, HT=None, visualization=None):
        """initialize robot base

        Args:
            HT: exudyn.HT or 4x4 matrix (list of lists / numpy array) containing homogeneous transformation to transform from world coordinates to base coordinates (changes orientation and position of robot), stored as exudyn.HT; default (None) = identity
            graphicsData: dictionary containing a list of GraphicsData, same as in exudyn Objects
        """
        self.HT = HT
        self.visualization = VRobotBase() if visualization is None else visualization

    @property
    def HT(self):
        """the homogeneous transformation, an exudyn.HT"""
        return self._HT

    @HT.setter
    def HT(self, value):
        self._HT = _ToHT(value)
    
    def __str__(self):
        s = '  HT = ' + str(self.HT)
        return s

    def __repr__(self):
        return str(self)


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  Define robot ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the version of the format of Robot.GetDictionary (#3273): 1, 2, ...; it is raised by 1 when the format changes between
#two releases, and SetDictionary reads every version up to it
robotDictionaryVersion = 1

class Robot:
    """class to define a robot
    """
    def __init__(self, 
                 gravity=[0,0,-9.81],
                 base = None,
                 tool = None,
                 referenceConfiguration = [],
                 q0 = None,
                 enableEndStops = False,
                 frictionRegularization = 0.01,
                 ):
        """initialize robot class

        Args:
            base: definition of base using RobotBase() class; None: a new RobotBase()
            tool: definition of tool using RobotTool() class; None: a new RobotTool()
            gravity: a list or 3D numpy array defining gravity
            referenceConfiguration: a list of scalar quantities defining the parameters for reference configuration
            q0: the initial joint coordinates of the models of CreateKinematicTree and CreateRedundantCoordinateMBS; None: the referenceConfiguration
            enableEndStops: if True, the models get end stops at the jointLimits of the links (with their endStopStiffness and endStopDamping)
            frictionRegularization: the joint velocity (rad/s or m/s) below which the Coulomb friction of the links is linear in the velocity
        """
        self.gravity = np.array(gravity)
        self.base = RobotBase() if base is None else base #new ones per robot: default arguments would be shared by all robots
        self.tool = RobotTool() if tool is None else tool
        self.referenceConfiguration = np.array(referenceConfiguration)
        self.q0 = q0
        self.enableEndStops = enableEndStops
        self.frictionRegularization = frictionRegularization
        self.links = [] #initialize list of link data
        self.isSerialRobot = True #this is true as long as parent = link index - 1

    @property
    def q0(self):
        """the initial joint coordinates of the models, a numpy array; the referenceConfiguration if not set (#3270)"""
        if self._q0 is None:
            return np.array(self.referenceConfiguration, dtype=float)
        return self._q0

    @q0.setter
    def q0(self, value):
        self._q0 = None if value is None else np.array(value, dtype=float)

    def SetJointFriction(self, viscous=0., coulomb=0.):
        """set the same viscous and Coulomb friction in every link (RobotLink viscousFriction, coulombFriction)

        Args:
            viscous: the viscous friction, a torque (force) per joint velocity
            coulomb: the Coulomb friction torque (force)
        """
        for link in self.links:
            link.viscousFriction = viscous
            link.coulombFriction = coulomb

    def SetEndStops(self, stiffness, damping, enable=True):
        """set the same end stop stiffness and damping in every link that has jointLimits, and enableEndStops

        Args:
            stiffness: the end stop stiffness, in Nm/rad or N/m
            damping: the end stop damping, in Nm/(rad/s) or N/(m/s)
            enable: the value of enableEndStops
        """
        for link in self.links:
            if link.jointLimits is not None:
                link.endStopStiffness = stiffness
                link.endStopDamping = damping
        self.enableEndStops = enable

    def __str__(self):
        s = 'gravity = ' + str(self.gravity)
        s += '\nreferenceConfiguration = ' + str(self.referenceConfiguration)
        s += '\nq0 = ' + str(self.q0)
        s += '\nbase: \n' + str(self.base)
        s += '\ntool: \n' + str(self.tool)
        s += '\nlinks: \n' + str(self.links)
        return s
    
    def __repr__(self):
        return str(self)

    def GetDictionary(self):
        """the robot as a dict of its parameters, links, base and tool with their graphics data (#3267) and the
        textures that the graphics data name, as renderer.GetTexture gives them (#3268), for SetDictionary; save and load
        it, e.g., with exudyn.advancedUtilities.SaveDictToHDF5 and LoadDictFromHDF5; the data of a created model
        (SetControl, the item numbers) are not included

        Returns:
            a dict of numbers, strings, lists and numpy arrays; 'dictionaryVersion' is the version of its format, which
            SetDictionary uses to read dicts of earlier versions (#3273)
        """
        linkVisualizationKeys = ['jointRadius', 'jointWidth', 'linkWidth', 'showMBSjoint', 'showCOM', 'linkColor']
        links = []
        for link in self.links:
            links += [{'mass': link.mass, 'COM': np.array(link.COM), 'inertia': np.array(link.inertia),
                       'localHT': link.localHT.HT44(), 'preHT': link.preHT.HT44(),
                       'jointType': link.jointType, 'parent': link.parent,
                       'PDcontrol': list(link.PDcontrol) if link.HasPDcontrol() else None,
                       'jointLimits': link.jointLimits, 'viscousFriction': link.viscousFriction,
                       'coulombFriction': link.coulombFriction, 'endStopStiffness': link.endStopStiffness,
                       'endStopDamping': link.endStopDamping,
                       'visualization': dict({key: getattr(link.visualization, key) for key in linkVisualizationKeys},
                                             graphicsData=list(link.visualization.graphicsData))}]
        #the textures of the graphics data; a name without texture is drawn without and left out
        textureNames = []
        for graphicsList in [self.base.visualization.graphicsData, self.tool.visualization.graphicsData] + [link.visualization.graphicsData for link in self.links]:
            for data in graphicsList:
                if isinstance(data, dict) and 'texture' in data and data['texture'] not in textureNames:
                    textureNames += [data['texture']]
        textures = []
        for name in textureNames:
            try:
                textures += [exudyn._compiledModule._GetTexture(name)]
            except ValueError:
                pass
        return {'dictionaryType': 'Robot', 'dictionaryVersion': robotDictionaryVersion,
                'gravity': np.array(self.gravity), 'referenceConfiguration': np.array(self.referenceConfiguration),
                'q0': self._q0, 'enableEndStops': self.enableEndStops,
                'frictionRegularization': self.frictionRegularization,
                'base': {'HT': self.base.HT.HT44(), 'graphicsData': list(self.base.visualization.graphicsData)},
                'tool': {'HT': self.tool.HT.HT44(), 'graphicsData': list(self.tool.visualization.graphicsData)},
                'links': links, 'textures': textures}

    def SetDictionary(self, robotDict):
        """set the robot from a dict of GetDictionary (#3267), replacing its links, base and tool; the textures of
        the dict are added as renderer.AddTexture adds them, replacing the images of their names (#3268)

        Args:
            robotDict: the dict, e.g. loaded with exudyn.advancedUtilities.LoadDictFromHDF5

        Example:
            from exudyn.advancedUtilities import SaveDictToHDF5, LoadDictFromHDF5
            SaveDictToHDF5('robot.h5', robot.GetDictionary())
            robot2 = Robot()
            robot2.SetDictionary(LoadDictFromHDF5('robot.h5'))
        """
        if robotDict.get('dictionaryType') != 'Robot':
            raise ValueError("Robot.SetDictionary: the dict is not one of Robot.GetDictionary (dictionaryType 'Robot')")
        version = robotDict.get('dictionaryVersion')
        if not isinstance(version, int) or version < 1 or version > robotDictionaryVersion:
            raise ValueError('Robot.SetDictionary: the dict has the format version ' + str(version) + ', but this Exudyn reads the versions 1 to '
                             + str(robotDictionaryVersion) + '; a dict of a later Exudyn needs that version')
        #the dicts of earlier versions are converted here, version by version, when the format changes
        self.gravity = np.array(robotDict['gravity'], dtype=float)
        self.base = RobotBase(HT=robotDict['base']['HT'], visualization=VRobotBase(graphicsData=robotDict['base']['graphicsData']))
        self.tool = RobotTool(HT=robotDict['tool']['HT'], visualization=VRobotTool(graphicsData=robotDict['tool']['graphicsData']))
        self.enableEndStops = bool(robotDict['enableEndStops'])
        self.frictionRegularization = robotDict['frictionRegularization']
        self.links = []
        self.isSerialRobot = True
        for linkDict in robotDict['links']:
            PDcontrol = (None, None) if linkDict['PDcontrol'] is None else tuple(linkDict['PDcontrol'])
            self.AddLink(RobotLink(mass=linkDict['mass'], COM=linkDict['COM'], inertia=linkDict['inertia'],
                                   localHT=linkDict['localHT'], preHT=linkDict['preHT'], jointType=linkDict['jointType'],
                                   parent=linkDict['parent'], PDcontrol=PDcontrol,
                                   visualization=VRobotLink(**linkDict['visualization']),
                                   jointLimits=linkDict['jointLimits'], viscousFriction=linkDict['viscousFriction'],
                                   coulombFriction=linkDict['coulombFriction'], endStopStiffness=linkDict['endStopStiffness'],
                                   endStopDamping=linkDict['endStopDamping']))
        self.referenceConfiguration = np.array(robotDict['referenceConfiguration'], dtype=float)
        self.q0 = robotDict['q0']
        for texture in robotDict.get('textures', []):
            exudyn._compiledModule._SetTexture(texture)

    def SaveToHDF5(self, fileName):
        """save the robot with its graphics and textures to an HDF5 file, the dict of GetDictionary (#3356)

        Args:
            fileName: the file name, including its ending, usually .h5

        Note:
            requires h5py and scipy
        """
        from exudyn.advancedUtilities import SaveDictToHDF5
        SaveDictToHDF5(fileName, self.GetDictionary())

    def LoadFromHDF5(self, fileName):
        """set the robot from an HDF5 file of SaveToHDF5, as SetDictionary does (#3356)

        Args:
            fileName: the file name, including its ending, usually .h5

        Note:
            requires h5py and scipy
        """
        from exudyn.advancedUtilities import LoadDictFromHDF5
        self.SetDictionary(LoadDictFromHDF5(fileName))

    def AddLink(self, robotLink):
        """add a link to serial robot
        """
        i = len(self.links) #current index
        if len(self.referenceConfiguration) == i: #extend reference configuration, if not specified by user during initialization
            self.referenceConfiguration = np.hstack((self.referenceConfiguration,[0]))

        self.links += [deepcopy(robotLink)]
        if self.links[i].parent == -2: #in this case, automatically set parents for serial robot (chain)
            self.links[i].parent = i-1
        elif self.links[i].parent >= i:
            raise ValueError('Robot.AddLink(...): link parent index must be always lower than link index')

        if  self.links[i].parent != i-1:
            self.isSerialRobot = False
        
        if not self.isSerialRobot and (cMath.norm(self.tool.HT.HT44() - np.eye(4)) >= 1e-15
            or self.tool.visualization.graphicsData != []):
            exudyn.Print('Warning: class Robot: tool defined in kinematic tree; currently tool is only allowed for serial robots')

        return i #return index of link

    def IsSerialRobot(self):
        """return True, if robot is a serial robot
        """
        return self.isSerialRobot

    def GetLink(self, i):
        """return Link object of link i
        """
        return self.links[i]

    def HasParent(self, i):
        """True if link has parent, False if not
        """
        return self.links[i].parent >= 0

    def GetParentIndex(self, i):
        """Get index of parent link; for serial robot this is simple, but for general trees, there is a index list
        """
        return self.links[i].parent

    
    def NumberOfLinks(self):
        """return number of links
        """
        return len(self.links)

    def GetBaseHT(self):
        """return base as homogeneous transformation
        """
        return self.base.HT

    def GetToolHT(self):
        """return the tool frame relative to the frame of the last link, as homogeneous transformation; GetGlobalToolHT
        gives it in global coordinates
        """
        return self.tool.HT

    def GetGlobalToolHT(self, q):
        """the tool frame (TCP) in global coordinates at the joint coordinates q, an exudyn.HT (#3273), computed in Python
        by LinkHT; for many q, e.g. along a trajectory, create the C++ chain once, chain = robot.GetKinematicChain(), and
        call chain.ToolHT(q), about 100 times faster (some 100000 evaluations per second)

        Args:
            q: the joint coordinates

        Returns:
            the exudyn.HT of the tool in global coordinates
        """
        return self.LinkHT(q)[-1] * self.tool.HT
    
    def SetControl(self, trajectory, compensation=None):
        """the joint control of the model of CreateKinematicTree or CreateRedundantCoordinateMBS, by PreStepControl
        (#3262, #3270): the PD control of the links (RobotLink PDcontrol) towards the joint coordinates and velocities of
        trajectory

        Args:
            trajectory: an exudyn.robotics.motion.Trajectory (or an object with Evaluate(t) returning [q, q_t, q_tt])
            compensation: None; 'static': the joint forces that hold the robot against gravity at the coordinates of the trajectory (-KinematicChain.StaticTorques); 'dynamic': the joint forces of the inverse dynamics with the velocities and accelerations of the trajectory (KinematicChain.InverseDynamics, #3276), a feed-forward that leaves the PD control the errors only
        """
        if compensation not in [None, 'static', 'dynamic']:
            raise ValueError("Robot.SetControl: compensation must be None, 'static' or 'dynamic', not " + str(compensation))
        self.controlTrajectory = trajectory
        self.controlCompensation = compensation
        self.controlChain = self.GetKinematicChain() if compensation is not None else None

    def _ModelData(self, functionName):
        """the data of the model created last by CreateKinematicTree or CreateRedundantCoordinateMBS"""
        data = getattr(self, 'modelData', None)
        if data is None:
            raise ValueError('Robot.' + functionName + ': create the model first, by CreateKinematicTree or CreateRedundantCoordinateMBS')
        return data

    def PreStepControl(self, mbs, t):
        """the PreStepUserFunction of the joint control of SetControl (#3262, #3270), for the model created last by
        CreateKinematicTree or CreateRedundantCoordinateMBS: the joint position and velocity offsets of the PD control
        from the trajectory at time t, and the joint forces of the compensation; pass it directly,
        mbs.SetPreStepUserFunction(robot.PreStepControl), or call it in the PreStepUserFunction of the script for
        several robots

        Args:
            mbs: the MainSystem of the model
            t: the time

        Returns:
            True
        """
        if getattr(self, 'controlTrajectory', None) is None:
            raise ValueError('Robot.PreStepControl: call SetControl(trajectory) first')
        data = self._ModelData('PreStepControl')
        [q, q_t, q_tt] = self.controlTrajectory.Evaluate(t)
        drive = np.zeros(len(self.links)) #the joint forces of the compensation, acting on the links of the joints
        if self.controlCompensation == 'static':
            drive = -self.controlChain.StaticTorques(q)
        elif self.controlCompensation == 'dynamic':
            drive = self.controlChain.InverseDynamics(q, q_t, q_tt)

        if data['modelType'] == 'kinematicTree':
            oKT = data['objectKinematicTree']
            mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector', q)
            mbs.SetObjectParameter(oKT, 'jointVelocityOffsetVector', q_t)
            if self.controlCompensation is not None:
                mbs.SetObjectParameter(oKT, 'jointForceVector', drive)
        else:
            #the spring-dampers measure the absolute joint coordinate; their force acts against it, the drive is -force
            for (i, objectSD) in enumerate(data['springDamperList']):
                if objectSD is None:
                    continue
                mbs.SetObjectParameter(objectSD, 'offset', q[i])
                mbs.SetObjectParameter(objectSD, 'velocityOffset', q_t[i])
                if self.controlCompensation is not None:
                    mbs.SetObjectParameter(objectSD, 'torque' if self.links[i].jointType[0] == 'R' else 'force', -drive[i])
        return True

    def AddSensors(self, mbs, outputVariables=[exudyn.OutputVariableType.Coordinates, exudyn.OutputVariableType.Coordinates_t],
                   storeInternal=True, fileName='', writeToFile=None):
        """add sensors of the joints to the model created last by CreateKinematicTree or CreateRedundantCoordinateMBS
        (#3270); each sensor gives the vector of one quantity over all joints, the same in both models

        Args:
            mbs: the MainSystem of the model
            outputVariables: a list of exudyn.OutputVariableType: Coordinates (the joint coordinates q), Coordinates_t (the joint velocities), Force (the joint torques and forces of the PD control and the compensation, acting on the link of the joint)
            storeInternal: if True, the sensors store their values in the mbs (mbs.GetSensorStoredData, PlotSensor)
            fileName: if not empty, the base of the file names; the name of the output variable is appended
            writeToFile: None: write to a file if fileName is given; True or False otherwise

        Returns:
            a dict of the sensor numbers, with the output variables as keys
        """
        data = self._ModelData('AddSensors')
        OVT = exudyn.OutputVariableType
        nLinks = len(self.links)
        sensors = {}
        for ovt in outputVariables:
            if ovt not in [OVT.Coordinates, OVT.Coordinates_t, OVT.Force]:
                raise ValueError('Robot.AddSensors: outputVariables must be Coordinates, Coordinates_t or Force, not ' + str(ovt))
            args = {'storeInternal': storeInternal,
                    'fileName': (fileName + str(ovt).split('.')[-1] + '.txt') if fileName != '' else '',
                    'writeToFile': (fileName != '') if writeToFile is None else writeToFile}
            if data['modelType'] == 'kinematicTree' and ovt != OVT.Force:
                sensors[ovt] = mbs.AddSensor(eii.SensorBody(bodyNumber=data['objectKinematicTree'], outputVariableType=ovt, **args))
                continue

            if data['modelType'] == 'kinematicTree': #the force of the PD control and the joint forces
                oKT = data['objectKinematicTree']
                def SensorUF(mbs, t, sensorNumbers, factors, configuration, oKT=oKT):
                    q = mbs.GetObjectOutputBody(oKT, OVT.Coordinates, configuration=configuration)
                    q_t = mbs.GetObjectOutputBody(oKT, OVT.Coordinates_t, configuration=configuration)
                    f = np.zeros(nLinks)
                    P = mbs.GetObjectParameter(oKT, 'jointPControlVector')
                    if len(P) != 0:
                        f += P*(mbs.GetObjectParameter(oKT, 'jointPositionOffsetVector') - q)
                    D = mbs.GetObjectParameter(oKT, 'jointDControlVector')
                    if len(D) != 0:
                        f += D*(mbs.GetObjectParameter(oKT, 'jointVelocityOffsetVector') - q_t)
                    fJoint = mbs.GetObjectParameter(oKT, 'jointForceVector')
                    if len(fJoint) != 0:
                        f += fJoint
                    return f
            elif ovt == OVT.Force: #the spring-dampers of the redundant model
                springDampers = [(i, objectSD, OVT.TorqueLocal if self.links[i].jointType[0] == 'R' else OVT.ForceLocal)
                                 for (i, objectSD) in enumerate(data['springDamperList']) if objectSD is not None]
                def SensorUF(mbs, t, sensorNumbers, factors, configuration, springDampers=springDampers):
                    f = np.zeros(nLinks)
                    for (i, objectSD, ovtSD) in springDampers:
                        f[i] = -mbs.GetObjectOutput(objectSD, ovtSD, configuration=configuration)
                    return f
            else: #the joint coordinates of the redundant model, measured by its coordinate markers
                markers = data['jointCoordinateMarkerList']
                ovtMarker = ovt
                def SensorUF(mbs, t, sensorNumbers, factors, configuration, markers=markers, ovtMarker=ovtMarker):
                    return np.array([mbs.GetMarkerOutput(m, ovtMarker, configuration=configuration)[0] for m in markers])

            sensors[ovt] = mbs.AddSensor(eii.SensorUserFunction(sensorUserFunction=SensorUF, **args))
        return sensors

    def _HasJointConnector(self, i):
        """True if link i needs the CoordinateSpringDamperExt of its friction or end stops (#3270)"""
        link = self.links[i]
        hasEndStops = self.enableEndStops and link.jointLimits is not None
        if hasEndStops and link.endStopStiffness == 0:
            raise ValueError('Robot: link ' + str(i) + ' has jointLimits but no endStopStiffness; set it, e.g. by Robot.SetEndStops, or enableEndStops=False')
        return hasEndStops or link.viscousFriction != 0 or link.coulombFriction != 0

    def _AddJointConnectors(self, mbs, i, groundMarker, marker, coordinateOffset):
        """the CoordinateSpringDamperExt of the friction and the one of the end stops of link i between the coordinate
        marker of its joint and a ground coordinate marker (#3270), each None if not needed; coordinateOffset is the
        value of the marker at the joint coordinate zero"""
        link = self.links[i]
        friction = None
        endStop = None
        if link.viscousFriction != 0 or link.coulombFriction != 0: #regularized, without a data node
            friction = mbs.AddObject(eii.CoordinateSpringDamperExt(markerNumbers=[groundMarker, marker],
                                                                   damping=link.viscousFriction,
                                                                   dynamicFrictionForce=link.coulombFriction,
                                                                   frictionProportionalZone=self.frictionRegularization,
                                                                   visualization=eii.VCoordinateSpringDamperExt(show=False)))
        if self.enableEndStops and link.jointLimits is not None: #a separate object, as the stops switch to the PostNewton iteration
            nData = mbs.AddNode(eii.NodeGenericData(numberOfDataCoordinates=3, initialCoordinates=[0,0,0]))
            endStop = mbs.AddObject(eii.CoordinateSpringDamperExt(markerNumbers=[groundMarker, marker], nodeNumber=nData,
                                                                  useLimitStops=True,
                                                                  limitStopsLower=link.jointLimits[0] + coordinateOffset,
                                                                  limitStopsUpper=link.jointLimits[1] + coordinateOffset,
                                                                  limitStopsStiffness=link.endStopStiffness,
                                                                  limitStopsDamping=link.endStopDamping,
                                                                  visualization=eii.VCoordinateSpringDamperExt(show=False)))
        return (friction, endStop)

    def GetKinematicChain(self):
        """the kinematics of the robot in C++, an exudyn.cUtilities.KinematicChain of its links, base, tool and gravity
        (#3259): JointHT, LinkHT, ToolHT, Jacobian and StaticTorques of the chain compute what the functions of the
        same names of Robot compute in Python, some hundred times faster; create it again after the robot is changed
        """
        return exudyn.cUtilities.KinematicChain(jointTypes=[link.jointType for link in self.links],
                                                preHTs=[link.preHT for link in self.links],
                                                localHTs=[link.localHT for link in self.links],
                                                parents=[self.GetParentIndex(i) for i in range(len(self.links))],
                                                masses=[link.mass for link in self.links],
                                                COMs=[link.COM for link in self.links],
                                                baseHT=self.base.HT, toolHT=self.tool.HT, gravity=self.gravity,
                                                inertias=[link.inertia for link in self.links])

    def LinkHT(self, q):
        """compute list of homogeneous transformations (exudyn.HT) for every link, using current joint coordinates q; leads to different results for standard and modified DH parameters because link coordinates are different! The C++ version is KinematicChain.LinkHT of GetKinematicChain()
        """
        HT = []


        for i in range(len(self.links)):
            link = self.links[i]

            T01 = link.preHT * dictJointType2HT[link.jointType](q[i]) * link.localHT
            if self.HasParent(i):
                pIndex = self.GetParentIndex(i)
                Tcurrent = HT[pIndex] * T01
            else:
                Tcurrent = self.base.HT * T01
            HT += [Tcurrent]
        
        return HT    

    def JointHT(self, q):
        """compute list of homogeneous transformations (exudyn.HT) for every joint (after rotation), using current joint coordinates q; the C++ version is KinematicChain.JointHT of GetKinematicChain()
        """
        HT = []

        for i in range(len(self.links)):
            link = self.links[i]

            T01 = link.preHT * dictJointType2HT[link.jointType](q[i])
            if self.HasParent(i):
                pIndex = self.GetParentIndex(i)
                Tcurrent = HT[pIndex] * self.links[pIndex].localHT * T01
            else:
                Tcurrent = self.base.HT * T01
            HT += [Tcurrent]
        return HT

    def COMHT(self, HT):
        """compute list of  homogeneous transformations HT (exudyn.HT) from base to every COM using HT list from Robot.JointHT(...)
        """
        HTCOM = []

        for i in range(len(self.links)):
            HTCOM += [_ToHT(HT[i]) * self.links[i].localHT * exudyn.HT(translation=self.links[i].COM)]
        
        return HTCOM
    
    def StaticTorques(self,HT):
        """compute the generalized forces of gravity on the joints (gravity and masses as given in robot), the sum over the links j of J_j^T m_j g with the Jacobian J_j of the center of mass of link j, taking HT from Robot.JointHT(); in one backward pass (#3259): the mass and its first moment summed over the links after each joint, then tau_i = axis . ((S_i - M_i o_i) x g) for a revolute joint at o_i, axis . (M_i g) for a prismatic one; the joint torques that hold the robot are their negative; the C++ version is KinematicChain.StaticTorques of GetKinematicChain()
        """
        n = len(self.links)
        HTcom = self.COMHT(HT)
        massSum = np.array([link.mass for link in self.links], dtype=float)
        momentSum = [self.links[i].mass * np.array(HTcom[i].translation) for i in range(n)]
        for i in reversed(range(n)): #the links after a joint come after it
            if self.HasParent(i):
                massSum[self.GetParentIndex(i)] += massSum[i]
                momentSum[self.GetParentIndex(i)] = momentSum[self.GetParentIndex(i)] + momentSum[i]
        gravity = np.array(self.gravity, dtype=float)
        jointTorques = np.zeros(n)
        for i in range(n):
            Hjoint = _ToHT(HT[i])
            axis = Hjoint.rotation @ dictJointType2Axis[self.links[i].jointType]
            if self.links[i].jointType[0] == 'R':
                jointTorques[i] = axis @ np.cross(momentSum[i] - massSum[i] * Hjoint.translation, gravity)
            else:
                jointTorques[i] = massSum[i] * (axis @ gravity)
        return jointTorques

    def InverseDynamics(self, q, q_t, q_tt):
        """the joint torques and forces that move the robot with the joint accelerations q_tt at the joint coordinates q
        and velocities q_t against gravity (#3276), M(q) q_tt + h(q, q_t) minus the generalized forces of gravity - at
        q_t = q_tt = 0 the negative of StaticTorques; the recursive Newton-Euler algorithm in global vectors: forward the
        angular velocity and acceleration of each link and the acceleration of its joint origin (the base accelerated by
        -g, which gives the weights), the force m a_c at the COM and the moment I alpha + omega x I omega; backward the
        force and the moment about the joint origin, summed over the subtree, projected on the joint axis; the C++
        version is KinematicChain.InverseDynamics of GetKinematicChain(), about 100 times faster

        Args:
            q: the joint coordinates
            q_t: the joint velocities
            q_tt: the joint accelerations

        Returns:
            the joint torques (revolute) and forces (prismatic), a numpy array
        """
        n = len(self.links)
        jointHTs = self.JointHT(q)
        linkHTs = self.LinkHT(q)
        (omega, alpha, accelerationOrigin, force, moment) = ([None]*n, [None]*n, [None]*n, [None]*n, [None]*n)
        origins = [np.array(H.translation) for H in jointHTs]
        axes = [np.array(jointHTs[i].rotation) @ dictJointType2Axis[self.links[i].jointType] for i in range(n)]
        for i in range(n):
            link = self.links[i]
            if self.HasParent(i):
                p = self.GetParentIndex(i)
                (omegaParent, alphaParent, accelerationParent, originParent) = (omega[p], alpha[p], accelerationOrigin[p], origins[p])
            else:
                (omegaParent, alphaParent, accelerationParent) = (np.zeros(3), np.zeros(3), -np.array(self.gravity, dtype=float))
                originParent = np.array(self.base.HT.translation)
            r = origins[i] - originParent
            accelerationOrigin[i] = accelerationParent + np.cross(alphaParent, r) + np.cross(omegaParent, np.cross(omegaParent, r))
            if link.jointType[0] == 'R':
                omega[i] = omegaParent + q_t[i]*axes[i]
                alpha[i] = alphaParent + q_tt[i]*axes[i] + np.cross(omegaParent, q_t[i]*axes[i])
            else: #the origin slides along the axis
                (omega[i], alpha[i]) = (omegaParent, alphaParent)
                accelerationOrigin[i] = accelerationOrigin[i] + q_tt[i]*axes[i] + 2.*np.cross(omegaParent, q_t[i]*axes[i])
            c = np.array(linkHTs[i].rotation) @ link.COM + np.array(linkHTs[i].translation) - origins[i]
            accelerationCOM = accelerationOrigin[i] + np.cross(alpha[i], c) + np.cross(omega[i], np.cross(omega[i], c))
            R = np.array(linkHTs[i].rotation)
            inertia = R @ link.inertia @ R.T
            force[i] = link.mass*accelerationCOM
            moment[i] = inertia @ alpha[i] + np.cross(omega[i], inertia @ omega[i]) + np.cross(c, force[i])
        for i in reversed(range(n)): #the links of the subtree come after their parent
            if self.HasParent(i):
                p = self.GetParentIndex(i)
                force[p] = force[p] + force[i]
                moment[p] = moment[p] + moment[i] + np.cross(origins[i] - origins[p], force[i])
        return np.array([axes[i] @ (moment[i] if self.links[i].jointType[0] == 'R' else force[i]) for i in range(n)])
    
    
    def Jacobian(self, HT, toolPosition=[], mode='all', linkIndex=None):
        """compute jacobian for translation and rotation at toolPosition using joint HT; this is using the Robot functions, but is inefficient for simulation purposes; the C++ version is KinematicChain.Jacobian of GetKinematicChain(), which takes q and a position in the link frame

        Args:
            HT: list of homogeneous transformations per joint , as computed by Robot.JointHT(...)
            toolPosition: global position at which the jacobian is evaluated (e.g., COM); if empty [], it uses the origin of the last link
            mode: 'all'...translation and rotation jacobian, 'trans'...only translation part, 'rot': only rotation part
            linkIndex: link index for which the jacobian is evaluated; if linkIndex==None, it uses the last link provided in HT

        Returns:
            returns jacobian with translation and rotation parts in rows (3 or 6) according to mode, and one column per HT; in the kinematic tree the columns not related to linkIndex remain zero
        """
        n = len(HT)
        if n > len(self.links):
            exudyn.Print("ERROR: number of homogeneous transformations (HT) greater than number of links")

        #link index is usually the last link contained in HT (all subsequent columns in jacobian would be anyway zero):
        if linkIndex is None:
            linkIndex = n-1
    
        Jomega = np.zeros((3,n))#rotation part of jacobian
        Jvel = np.zeros((3,n))  #translation part of jacobian
        
        vn = list(toolPosition)
        if len(vn) == 0:
            vn = (_ToHT(HT[linkIndex]) * self.links[linkIndex].localHT).translation #last link coordinates


        #for tree, we may only consider links in the chain from link to base!
        i = linkIndex
        endReached = False
        while not endReached:
            Hjoint = _ToHT(HT[i])
            A = Hjoint.rotation #rotation of joint i

            localAxis = dictJointType2Axis[self.links[i].jointType]

            axis = A @ localAxis #axis in global coordinates
            if self.links[i].jointType[0] == 'R':
                Jomega[0:3,i] = axis #only considered, if revolute joint
            
            vPrevious = Hjoint.translation
             
            #revolute joint:
            if self.links[i].jointType[0] == 'R': #revolute joint
                Jvel[0:3,i]   = erb.Skew(axis) @ (vn - vPrevious) #only considered, if revolute joint
            elif self.links[i].jointType[0] == 'P': #prismatic joint
                Jvel[0:3,i]   = axis
            else:
                raise ValueError('Robot.Jacobian(...): illegal jointType')
                
            if not self.HasParent(i):
                endReached = True
            else:
                i = self.GetParentIndex(i)
        
        if mode == 'rot':
            J = Jomega
        elif mode == 'trans':
            J = Jvel
        elif mode == 'all':
            J = np.zeros((6,n))
            J[0:3,0:n] = Jvel
            J[3:6,0:n] = Jomega
    
        return J

    def InverseKinematicsNumerical(self, HT, qInitial, weights=None, tolerance=1e-10, maxIterations=100, restarts=0):
        """the numerical inverse kinematics by Levenberg-Marquardt (#3269): joint coordinates q with the tool frame
        LinkHT(q)[-1]*tool.HT equal to HT (exu.HT or 4x4 matrix), starting from qInitial, for any serial robot (also
        with fewer or more than 6 joints). The error e is the position difference and the rotation vector of
        R_HT R(q)^T, multiplied by weights; the step solves (J^T J + lambda I) dq = J^T e, which lowers E = e.e/2; lambda
        divided by 10 after a step that lowers E and multiplied by 10 until one does

        Args:
            HT: the desired tool frame, exu.HT or 4x4 matrix
            qInitial: the joint coordinates to start from
            weights: None (ones), or 6 weights of the position and rotation errors; zeros for components that are free
            tolerance: converged for |e| <= tolerance
            maxIterations: the iterations per start
            restarts: further starts from qInitial shifted by the fixed sequence pi (2 frac(k sqrt(p_i)) - 1) with the
                primes p_i, for the revolute joints

        Returns:
            (q, success, iterations), q of the last start if none converged

        Note:
            the C++ version is KinematicChain.InverseKinematicsNumerical of GetKinematicChain(), about 90 times faster (10 us for the Puma560)
        """
        if not self.isSerialRobot:
            raise ValueError('Robot.InverseKinematicsNumerical: the numerical inverse kinematics needs a serial robot')
        n = len(self.links)
        T = _ToHT(HT)
        qInitial = np.array(qInitial, dtype=float)
        if len(qInitial) != n:
            raise ValueError('Robot.InverseKinematicsNumerical: qInitial must have '+str(n)+' components')
        weights = np.ones(6) if weights is None else np.array(weights, dtype=float)
        primes = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53]
        revolute = np.array([link.jointType[0] == 'R' for link in self.links])

        def PoseError(q):
            Tq = self.LinkHT(q)[-1]*self.tool.HT
            e = np.hstack([np.array(T.translation) - np.array(Tq.translation),
                           erb.RotationMatrix2RotationVector(np.array(T.rotation) @ np.array(Tq.rotation).T)])*weights
            return (e, 0.5*(e @ e), np.array(Tq.translation))

        iterations = 0
        success = False
        for start in range(restarts + 1):
            s = start*np.sqrt([primes[i % 16] for i in range(n)])
            q = qInitial + revolute*np.pi*(2.*(s - np.floor(s)) - 1.)*(start > 0)
            (e, E, p) = PoseError(q)
            lam = 1e-3
            for it in range(maxIterations):
                if np.sqrt(2.*E) <= tolerance:
                    success = True
                    break
                iterations += 1
                J = self.Jacobian(self.JointHT(q), toolPosition=p)*weights[:, None]
                A = J.T @ J
                g = J.T @ e
                accepted = False
                while not accepted and lam <= 1e10:
                    qNew = q + np.linalg.solve(A + lam*np.eye(n), g)
                    (eNew, ENew, pNew) = PoseError(qNew)
                    if ENew < E:
                        (q, e, E, p) = (qNew, eNew, ENew, pNew)
                        lam = max(0.1*lam, 1e-12)
                        accepted = True
                    else:
                        lam *= 10.
                if not accepted: #no descent: a local minimum
                    break
            if success or np.sqrt(2.*E) <= tolerance:
                success = True
                break
        return (q, success, iterations)

    def ArmConfiguration(self, q):
        """the configuration of the arm at the joint coordinates q, which tells the solutions of InverseKinematicsAnalytic
        apart (#3265): [shoulder, elbow, wrist], each +1 or -1, and 0 in the singular configuration between them; from
        the joint axes omega_i at q, independent of the DH convention and the zero angles of the joints, for the robots of
        InverseKinematicsAnalytic. The wrist center W is the point of the axes 5 and 6 (of 4, 5, 6 for a spherical
        wrist), or the tool position (3 links); a, b are the vectors from axis 2 to axis 3 and from axis 3 to W (to axis 4
        for the UR), normal to omega_2:
        shoulder = sign((W - p_1) . (omega_1 x omega_2)), W in front of (+1) or behind axis 1, 0 for W on axis 1;
        elbow = sign(omega_3 . (a x b)), the bend of the elbow about axis 3, 0 for the arm stretched;
        wrist = sign(omega_5 . (omega_4 x omega_6)), 0 for axis 6 parallel to axis 4 (always 0 for 3 links).
        The signs follow the directions of the axes; which sign is called up or down, left or right is that of the robot.

        Args:
            q: the joint coordinates

        Returns:
            a list of three ints

        Note:
            the C++ version is KinematicChain.ArmConfiguration of GetKinematicChain()
        """
        n = len(self.links)
        if not self.isSerialRobot or any(link.jointType[0] != 'R' for link in self.links) or n not in [3, 6]:
            raise ValueError('Robot.ArmConfiguration: the configuration needs 3 or 6 revolute joints in series')
        JH = [_ToHT(H) for H in self.JointHT(q)]
        omega = [np.array(H.rotation) @ dictJointType2Axis[link.jointType] for (H, link) in zip(JH, self.links)]
        p = [np.array(H.translation) for H in JH]
        scale = 1. + max(cMath.norm(x - p[0]) for x in p)
        if n == 3:
            W = np.array((self.LinkHT(q)[-1]*self.tool.HT).translation)
        else:
            W = _IKLineIntersection(p[4], omega[4], p[5], omega[5])[0]
            if W is None:
                raise ValueError('Robot.ArmConfiguration: the axes 5 and 6 must cross')
        E = W #the point after the elbow
        if n == 6 and cMath.norm(np.cross(omega[1], omega[3])) < 1e-10: #UR: axis 4 parallel to axis 2
            E = p[3]
        def Sign(x, tolerance):
            return 0 if abs(x) < tolerance else int(np.sign(x))
        n12 = np.cross(omega[0], omega[1])
        shoulder = Sign((W - p[0]) @ n12/cMath.norm(n12), 1e-10*scale)
        a = p[2] - p[1] - omega[1]*(omega[1] @ (p[2] - p[1]))
        b = E - p[2] - omega[1]*(omega[1] @ (E - p[2]))
        elbow = Sign(omega[2] @ np.cross(a, b)/max(cMath.norm(a)*cMath.norm(b), 1e-300), 1e-10)
        wrist = 0 if n == 3 else Sign(omega[4] @ np.cross(omega[3], omega[5]), 1e-10)
        return [shoulder, elbow, wrist]

    def InverseKinematicsAnalytic(self, HT, qReference=None, configuration=None):
        """the analytic inverse kinematics: all joint coordinates q with the tool frame LinkHT(q)[-1]*tool.HT equal to HT
        (an exu.HT or 4x4 matrix in global coordinates), for three geometries of revolute joints in series: 3 links with
        the axes 2 and 3 parallel (the position of the tool only, up to 4 solutions); 6 links with the axes 2 and 3
        parallel and the axes 4, 5, 6 through one point, a spherical wrist (the Puma560, KUKA, Staeubli; up to 8); 6
        links with the axes 2, 3 and 4 parallel and the axes 5 and 6 crossing (the UR robots; up to 8); a ValueError
        for other robots. The robot is a product of exponentials, T(q) = E_1(q_1) ... E_n(q_n) T(0), solved by the
        subproblems of Paden and Kahan (#3260); every solution is checked by the forward kinematics. In a singular
        configuration, where a joint angle is free, it takes the value of qReference (or 0).

        Args:
            HT: the desired tool frame, exu.HT or 4x4 matrix
            qReference: None, or joint coordinates; then the angles are shifted by multiples of 2 pi to be nearest to
                qReference and the solutions are sorted by their distance from it, the nearest first
            configuration: None, or [shoulder, elbow, wrist] of ArmConfiguration, each +1, -1, or 0 or None (any), e.g.
                ArmConfiguration(qReference) to keep the configuration of the robot; only the solutions of this
                configuration are returned, and those on its boundary (0)

        Returns:
            a numpy array with one solution per row, angles in (-pi, pi] without qReference; no rows if the pose cannot
            be reached

        Note:
            the C++ version is KinematicChain.InverseKinematicsAnalytic of GetKinematicChain(), about 100 times faster
        """
        n = len(self.links)
        if not self.isSerialRobot or any(link.jointType[0] != 'R' for link in self.links) or n not in [3, 6]:
            raise ValueError('Robot.InverseKinematicsAnalytic: the analytic inverse kinematics needs 3 or 6 revolute joints in series')
        fallback = np.zeros(n) if qReference is None else np.array(qReference, dtype=float)
        if len(fallback) != n:
            raise ValueError('Robot.InverseKinematicsAnalytic: qReference must have '+str(n)+' components')
        if configuration is not None and (len(configuration) != 3 or any(c not in [None, 0, 1, -1] for c in configuration)):
            raise ValueError('Robot.InverseKinematicsAnalytic: configuration must be None or [shoulder, elbow, wrist], each 1, -1, or 0 or None for any')
        JH0 = self.JointHT(np.zeros(n))
        omega = [_ToHT(H).rotation @ dictJointType2Axis[link.jointType] for (H, link) in zip(JH0, self.links)]
        p = [np.array(_ToHT(H).translation) for H in JH0]
        T0 = self.LinkHT(np.zeros(n))[-1]*self.tool.HT
        T = _ToHT(HT)
        scale = 1. + max(cMath.norm(x - p[0]) for x in p + [np.array(T0.translation)])
        tolerance = 1e-10*scale

        def Parallel(i, j):
            return cMath.norm(np.cross(omega[i], omega[j])) < 1e-10
        if not Parallel(1, 2) or Parallel(0, 1):
            raise ValueError('Robot.InverseKinematicsAnalytic: the axes of joints 2 and 3 must be parallel and that of joint 1 not parallel to them')

        G = T*T0.Inverse() #T(q) T(0)^-1 = E_1 ... E_n
        RG = np.array(G.rotation)
        solutions = []
        if n == 3:
            solutions = _IKArm(omega, p, np.array(T0.translation), np.array(T.translation), fallback, tolerance)
        else:
            (W0, distance) = _IKLineIntersection(p[3], omega[3], p[4], omega[4])
            if W0 is not None and distance < tolerance and cMath.norm(np.cross(W0 - p[5], omega[5])) < tolerance:
                #spherical wrist: the wrist center from q_1, q_2, q_3, then the wrist angles
                for q123 in _IKArm(omega, p, W0, G*W0, fallback, tolerance):
                    R123 = _IKRotationMatrix(omega[0], q123[0]) @ _IKRotationMatrix(omega[1], q123[1]) @ _IKRotationMatrix(omega[2], q123[2])
                    for q456 in _IKWrist(omega[3], omega[4], omega[5], R123.T @ RG, fallback):
                        solutions += [q123 + q456]
            else:
                (W0, distance) = _IKLineIntersection(p[4], omega[4], p[5], omega[5])
                if not Parallel(1, 3) or W0 is None or distance > tolerance or Parallel(3, 4):
                    raise ValueError('Robot.InverseKinematicsAnalytic: a robot of 6 joints needs a spherical wrist (the axes 4, 5, 6 '
                                     'through one point) or the axes 2, 3, 4 parallel and the axes 5 and 6 crossing (UR)')
                #UR: q_1 from the point W0 of the axes 5 and 6, q_5 from the axis 6, q_6, the sum of q_2, q_3, q_4, then
                #q_2, q_3 for a point of axis 4
                nAxis = omega[1]
                X = G*W0
                d = X - p[0]
                w = omega[0]
                wd = w @ d
                for q1 in _IKAnglesCosSin(nAxis @ d - wd*(w @ nAxis), -(nAxis @ np.cross(w, d)), nAxis @ (W0 - p[0]) - wd*(w @ nAxis),
                                          fallback[0], tolerance):
                    R1 = _IKRotationMatrix(w, q1)
                    Rrest = R1.T @ RG #R(n, q_2+q_3+q_4) R_5 R_6
                    b = Rrest @ omega[5]
                    c56 = omega[4] @ omega[5]
                    for q5 in _IKAnglesCosSin(nAxis @ omega[5] - c56*(nAxis @ omega[4]), nAxis @ np.cross(omega[4], omega[5]),
                                              nAxis @ b - c56*(nAxis @ omega[4]), fallback[4], 1e-10):
                        R5 = _IKRotationMatrix(omega[4], q5)
                        q6 = _IKAngle(omega[5], Rrest.T @ nAxis, R5.T @ nAxis, fallback[5])
                        Rn = Rrest @ _IKRotationMatrix(omega[5], q6).T @ R5.T
                        x = np.cross(nAxis, omega[4]) #normal to the parallel axes
                        q234 = _IKAngle(nAxis, x, Rn @ x, 0.)
                        Y = p[0] + _IKRotate(w, -q1, d)
                        target4 = Y - _IKRotate(nAxis, q234, W0 - p[3])
                        for (q2, q3) in _IKPlanar2R(omega[1], omega[2], p[1], p[2], p[3], target4, fallback, tolerance):
                            q4 = (q234 - (omega[1] @ nAxis)*q2 - (omega[2] @ nAxis)*q3)/(omega[3] @ nAxis)
                            solutions += [[q1, q2, q3, q4, q5, q6]]

        #the angles in (-pi, pi] or nearest to qReference, checked by the forward kinematics, without repetitions
        result = []
        for q in solutions:
            q = np.array(q)
            q = (q + np.pi) % (2*np.pi) - np.pi if qReference is None else q - 2*np.pi*np.round((q - fallback)/(2*np.pi))
            Tq = self.LinkHT(q)[-1]*self.tool.HT
            error = cMath.norm(np.array(Tq.translation) - np.array(T.translation))/scale
            if n == 6:
                error += cMath.norm(np.array(Tq.rotation) - np.array(T.rotation))
            if error < 1e-8 and all(cMath.norm(q - r) > 1e-8 for r in result):
                if configuration is None or all(not c or s == 0 or s == c
                                                for (s, c) in zip(self.ArmConfiguration(q), configuration)):
                    result += [q]
        if qReference is not None:
            result.sort(key=lambda q: cMath.norm(q - fallback))
        return np.array(result).reshape((len(result), n))


    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def CreateKinematicTree(self, mbs, name = '', forceUserFunction = 0, floatingBase = None):
        """Add a ObjectKinematicTree to existing mbs from the robot structure inside this robot class;
        Joints defined by the kinematics as well as links (and inertia) are transferred to the kinematic tree object;
        Current implementation only works for serial robots;
        Control can be realized simply by adding PDcontrol to RobotLink structures, then modifying jointPositionOffsetVector and jointVelocityOffsetVector in ObjectKinematicTree; force offsets (e.g., static or dynamic torque compensation) can be added to KinematicTree jointForceVector; more general control can be added by using KinematicTree forceUserFunction;
        The coordinates in KinematicTree (as well as jointPositionOffsetVector, etc.) are sorted in the order as the RobotLinks are added to the Robot class;
        Note that the ObjectKinematicTree is still under development and interfaces may change.

        Args:
            mbs: the multibody system, which will be extended
            name: object name in KinematicTree; transferred to KinematicTree, default = ''
            floatingBase: None for a base fixed at base.HT; else a RigidBodyInertia of the base, which becomes a free rigid body with a Lie group node
                at base.HT, with the graphics of the base, on which the tree moves (#3359); the initial velocity of the base is zero
            forceUserFunction: defines the user function for computation of joint forces in KinematicTree; transferred to KinematicTree, default = 0

        Returns:
            the function returns a dictionary containing 'nodeBase' and 'objectBase' with a floating base (else None), 'nodeGeneric': generic ODE2 node number ,'objectKinematicTree': the kinematic tree object, 'baseObject': the base object if created, otherwise None, 'jointFrictionList' and 'jointEndStopList': per link the CoordinateSpringDamperExt of its friction and of its end stops, or None

        Note:
            the initial joint coordinates are q0; the friction and the end stops of the links act by a CoordinateSpringDamperExt on the joint coordinate (#3270); PreStepControl and AddSensors act on this model until another one is created
        """

        #add graphics for base:
        baseObject = None #if it does not exist
        baseOffset = self.base.HT.translation
        nBase = None

        if floatingBase is not None: #the root links are placed in the frame of the base node, the graphics of the base are on its body (#3359)
            bodyDict = mbs.CreateRigidBody(inertia=floatingBase, referencePosition=baseOffset, referenceRotationMatrix=self.base.HT.rotation,
                                           gravity=self.gravity, nodeType=exudyn.NodeType.RotationRotationVector,
                                           graphicsDataList=list(self.base.visualization.graphicsData), returnDict=True)
            (nBase, baseObject) = (bodyDict['nodeNumber'], bodyDict['bodyNumber'])
            baseOffset = [0., 0., 0.]
        elif self.base.visualization.graphicsData != []:
            #add a ground object at base position
            graphicsDataBase = []
            pOff = baseOffset
            Aoff = self.base.HT.rotation
            for data in self.base.visualization.graphicsData:
                graphicsDataBase += [exudyn.graphics.Move(data, [0,0,0], Aoff)] #only rotated, translation is in ground

            baseObject = mbs.AddObject(eii.ObjectGround(referencePosition=pOff, 
                                                    visualization=eii.VObjectGround(graphicsData=graphicsDataBase)))

        #+++++++++++++++++++++++
        qRef = np.array(self.referenceConfiguration, dtype=float)
        q0 = self.q0
        
        nLinks = len(self.links)
        graphicsDataList = []   #list of graphicsData per link
        jointTypesList = []     #exudyn joint types
        linkParents = []

        jointHTs=[] #the joint transformations and offsets as HTs (#2798)
        linkInertiasCOM=[] 
        linkCOMs=[]
        linkMasses=[]

        #if PD control exists, this vector is kept, otherwise erased:
        jointPControlVector = [0]*nLinks
        jointDControlVector = [0]*nLinks
        jointPositionOffsetVector = list(q0) #the control holds the initial configuration
        jointVelocityOffsetVector = [0]*nLinks
        hasPDcontrol = False #True, if any link has control
        
        #create robot tree:
        for i in range(nLinks):
            link = self.links[i]
            linkParents += [self.GetParentIndex(i)]

            if link.jointType not in dictJointType2Axis:
                raise ValueError('CreateKinematicTree: found invalid joint type in link '+str(i)+':'+link.jointType)


            axis = dictJointType2Axis[link.jointType]
            
            if link.HasPDcontrol():
                jointPControlVector[i] = link.GetPDcontrol()[0]
                jointDControlVector[i] = link.GetPDcontrol()[1]
                hasPDcontrol = True
                
            #++++++++++++++++++++++++++++++++++
            jointTypesList += [dictJointTypeText2Exudyn[link.jointType]]
            
            parentLinkLocalHT = exudyn.HT()
            if self.HasParent(i):
                parentLinkLocalHT = self.links[self.GetParentIndex(i)].localHT
            jointHTs += [parentLinkLocalHT * link.preHT]

            #inertia is defined in link coordinates; but KinematicTree needs inertia w.r.t. joint coordinates:
            rbi = erb.RigidBodyInertia()
            rbi.SetWithCOMinertia(link.mass, link.inertia, link.COM)
    
            rbi = rbi.Transformed((link.localHT)) #inertia parameters need to be transformed to joint frame
            
            linkInertiasCOM += [rbi.InertiaCOM()] #KinematicTree needs inertia w.r.t. COM
            linkCOMs += [rbi.COM()] 
            linkMasses += [rbi.Mass()]

            
            #++++++++++++++++++++++++++++++++++
            #visualization:
            linkVisualization = link.visualization
            showMBSjoint = linkVisualization.showMBSjoint
            r = linkVisualization.jointRadius
            wJ = linkVisualization.jointWidth
            wL = linkVisualization.linkWidth
            color = linkVisualization.linkColor
            
            graphicsDataLink = []
            #draw COM:
            if linkVisualization.showCOM:
                dd = r*0.2
                colorCOM = copy(color)
                colorCOM[0] *= 0.7 #make COM a little darker
                colorCOM[1] *= 0.7
                colorCOM[2] *= 0.7
                graphicsDataLink += [exudyn.graphics.Brick(rbi.COM(), [dd,dd,dd], colorCOM)]


            #draw joint in this link
            if showMBSjoint:
                gJoint = exudyn.graphics.Cylinder(-0.5*wJ*axis, 0.5*wJ*axis, radius=r, color=graphics.color.grey)
                graphicsDataLink += [gJoint]
                
            #draw link from parent link origin to this link origin, defined by preHT of this link
            if self.HasParent(i):
                iParent = self.GetParentIndex(i)
                vParent = jointHTs[i].translation
                if len(linkVisualization.graphicsData) == 0:
                    gLink = exudyn.graphics.Cylinder([0.,0.,0.], vParent, radius=0.5*wL, color=color)
                    graphicsDataList[iParent] += [gLink]

            if len(linkVisualization.graphicsData) != 0:
                graphicsDataLink += linkVisualization.graphicsData

            #add transformed graphicsData of tool to link graphics; the tool is relative to the frame of the last link,
            #the graphics of the tree to its joint frame (#3255)
            if (i==len(self.links)-1 and #tool
                self.tool.visualization.graphicsData != []):
                toolHT = link.localHT * self.tool.HT
                for data in self.tool.visualization.graphicsData:
                    graphicsDataLink += [exudyn.graphics.Move(data, toolHT.translation, toolHT.rotation)] 

            graphicsDataList += [graphicsDataLink]
        
        #create node for unknowns of KinematicTree
        nGeneric = mbs.AddNode(eii.NodeGenericODE2(referenceCoordinates=qRef,
                                               initialCoordinates=q0-qRef,
                                               initialCoordinates_t=[0.]*nLinks,
                                               numberOfODE2Coordinates=nLinks))

        if not hasPDcontrol: #use empty lists, makes model more efficient
            jointPControlVector = []
            jointDControlVector = []
            jointPositionOffsetVector = [] #size must agree with jointPControlVector
            jointVelocityOffsetVector = [] #size must agree with jointDControlVector

        jointForceVector = [] #[0]*nLinks #not used right now


        #create KinematicTree
        baseNodeArguments = {} if nBase is None else {'baseNodeNumber': nBase}
        oKT = mbs.AddObject(eii.ObjectKinematicTree(nodeNumber=nGeneric, **baseNodeArguments, jointTypes=jointTypesList, linkParents=linkParents,
                                          jointHTs=jointHTs, 
                                          linkInertiasCOM=exudyn.Matrix3DList(linkInertiasCOM), 
                                          linkCOMs=exudyn.Vector3DList(linkCOMs), linkMasses=linkMasses, 
                                          baseOffset = baseOffset, gravity=self.gravity, 
                                          jointForceVector=jointForceVector, forceUserFunction=forceUserFunction,
                                          jointPositionOffsetVector = jointPositionOffsetVector, jointPControlVector = jointPControlVector,
                                          jointVelocityOffsetVector = jointVelocityOffsetVector, jointDControlVector = jointDControlVector,
                                          visualization=eii.VObjectKinematicTree(graphicsDataList = graphicsDataList)))

        #friction and end stops on the node coordinates, which are relative to the reference configuration (#3270)
        jointFrictionList = [None]*nLinks
        jointEndStopList = [None]*nLinks
        groundMarker = None
        for i in range(nLinks):
            if self._HasJointConnector(i):
                if groundMarker is None:
                    groundMarker = mbs.AddMarker(eii.MarkerNodeCoordinate(nodeNumber=mbs.AddNode(eii.NodePointGround()), coordinate=0))
                mCoordinate = mbs.AddMarker(eii.MarkerNodeCoordinate(nodeNumber=nGeneric, coordinate=i))
                (jointFrictionList[i], jointEndStopList[i]) = self._AddJointConnectors(mbs, i, groundMarker, mCoordinate, -qRef[i])

        #return some needed variables for further use
        d = {'nodeGeneric': nGeneric, 'objectKinematicTree': oKT, 'baseObject': baseObject, 'nodeBase': nBase, 'objectBase': None if nBase is None else baseObject,
             'jointFrictionList': jointFrictionList, 'jointEndStopList': jointEndStopList,
             }
        self.modelData = dict(d, modelType='kinematicTree') #for PreStepControl and AddSensors (#3262, #3270)
        return d

                      

    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def CreateRedundantCoordinateMBS(self, mbs, baseMarker, jointSpringDamperUserFunctionList= [], 
                                     jointLoadUserFunctionList=[], 
                                     createJointTorqueLoads=True, rotationMarkerBase=None,
                                     rigidBodyNodeType=exudyn.NodeType.RotationEulerParameters): 
        """Add items to existing mbs from the robot structure inside this robot class; robot is attached to baseMarker (can be ground object or moving/deformable body);
        The (serial) robot is built as rigid bodies (containing rigid body nodes), where bodies represent the links which are connected by joints;
        Add optional jointSpringDamperUserFunctionList for individual control of joints; otherwise use PDcontrol in RobotLink structure; additional joint torques/forces can be added via spring damper, using mbs.SetObjectParameter(...) function;
        See several Python examples, e.g., `serialRobotTestTSD.py`, in Examples or TestModels;
        For more efficient models, use CreateKinematicTree(...) function!

        Args:
            mbs: the multibody system, which will be extended
            baseMarker: a rigid body marker, at which the robot will be placed (usually ground); note that the local coordinate system of the base must be in accordance with the DH-parameters, i.e., the z-axis must be the first rotation axis. For correction of the base coordinate system, use rotationMarkerBase
            jointSpringDamperUserFunctionList: NOT IMPLEMENTED yet: jointSpringDamperUserFunctionLista list of user functions for actuation of joints with more efficient spring-damper based connector (spring-damper directly emulates PD-controller); uses torsional spring damper for revolute joints and linear spring damper for prismatic joints; can be empty list (no spring dampers); if entry of list is 0, no user function is created, just pure spring damper; parameters are taken from RobotLink PDcontrol structure, which MUST be defined using SetPDcontrol(...) in RobotLink
            jointLoadUserFunctionList: DEPRECATED: a list of user functions for actuation of joints according to a LoadTorqueVector userFunction, see serialRobotTest.py as an example; can be empty list
            createJointTorqueLoads: DEPRECATED: if True, independently of jointLoadUserFunctionList, joint loads are created; the load numbers are stored in lists jointTorque0List/ jointTorque1List; the loads contain zero torques and need to be updated in every computation step, e.g., using a preStepUserFunction; unitTorque0List/ unitTorque1List contain the unit torque vector for the according body(link) which needs to be applied on both bodies attached to the joint
            rotationMarkerBase: add a numpy 3x3 matrix for rotation of the base, in order that the robot can be attached to any rotated base marker; the rotationMarkerBase is according to the definition in GenericJoint; note, that for moving base, the static compensation does not work (base rotation must be updated)
            rigidBodyNodeType: specify node type of rigid body node, e.g., exudyn.NodeType.RotationEulerParameters, etc.

        Returns:
            the function returns a dictionary containing per link nodes and object (body) numbers, 'nodeList', 'bodyList', the object numbers for joints, 'jointList', list of load numbers for joint torques (jointTorque0List, jointTorque1List); unit torque vectors in local coordinates of the bodies to which the torques are applied (unitTorque0List, unitTorque1List); springDamperList contains the spring dampers if defined by PDcontrol of links; 'jointCoordinateMarkerList': per joint the marker of its coordinate (MarkerBodiesRelativeRotationCoordinate or MarkerBodiesRelativeTranslationCoordinate, the absolute joint coordinate), 'jointFrictionList' and 'jointEndStopList': per link the CoordinateSpringDamperExt of its friction and of its end stops, or None

        Note:
            the initial joint coordinates are q0; the friction and the end stops of the links act by a CoordinateSpringDamperExt on the joint coordinate marker (#3270), which needs a baseMarker on a body (e.g. MarkerBodyRigid of the ground); PreStepControl and AddSensors act on this model until another one is created
        """
        #build robot model:
        nodeList = []           #node number or rigid node for link
        bodyList = []           #body number or rigid body for link
        jointList = []          #joint which links to previous link or base
        jointCoordinateMarkerList = [] #the marker of each joint coordinate (#3270)
        jointFrictionList = [] #the CoordinateSpringDamperExt of the friction of each joint, or None
        jointEndStopList = [] #the CoordinateSpringDamperExt of the end stops of each joint, or None
        markerList0 = []        #contains n marker numbers per link (=body) which connect to previous/left link
        markerList1 = []        #contains n marker numbers per link (=body) which connect to next/right link
        jointTorque0List = []   #load number of joint torque at previous/left link (negative); 
        jointTorque1List = []   #load number of joint torque at next/right link (positive)
        unitTorque0List = []    #contains unit torque0 (previous/left link) for joint i, should be multiplied with according factor to represent joint torque
        unitTorque1List = []    #contains unit torque1 (next/right link) for joint i, should be multiplied with according factor to represent joint torque
        springDamperList = []   #contains torsional or linear spring dampers for control of joint axes
        
        HTlist = [] #list of homogeneous transformations for links stored (for parent link relation in tree)
        HTinitialList = [] #the same at the initial configuration q0
        groundMarker = None

        #delete: lastMarker = baseMarker
        lastMarkerRotation = np.identity(3) #base rotation included in marker
        if rotationMarkerBase is not None:
            lastMarkerRotation = rotationMarkerBase
            
        qRef = np.array(self.referenceConfiguration, dtype=float)
        q0 = self.q0
        
        baseObject = -1 #if it does not exist
        if self.base.visualization.graphicsData != []:
            #add a ground object at base position
            graphicsDataBase = []
            pOff = self.base.HT.translation
            Aoff = self.base.HT.rotation
            for data in self.base.visualization.graphicsData:
                graphicsDataBase += [exudyn.graphics.Move(data, [0,0,0], Aoff)] #only rotated, translation is in ground

            baseObject = mbs.AddObject(eii.ObjectGround(referencePosition=pOff, 
                                                    visualization=eii.VObjectGround(graphicsData=graphicsDataBase)))

        #create list of indices to next links
        nextLinks = [None]*self.NumberOfLinks()
        for i in range(self.NumberOfLinks()):
            nextLinks[i] = []
            link = self.links[i]
            if self.GetParentIndex(i) != -1:
               nextLinks[self.GetParentIndex(i)] += [i]
                
        #create robot nodes and bodies:
        for i in range(len(self.links)):
            link = self.links[i]
            if link.jointType not in dictJointType2Axis:
                raise ValueError('CreateRedundantCoordinateMBS: found invalid joint type in link '+str(i)+':'+link.jointType)
        
        
            T01 = link.preHT * dictJointType2HT[link.jointType](qRef[i]) * link.localHT #new bodies are placed at origin of link frame
            if not self.HasParent(i):
                Tcurrent = self.GetBaseHT()
            else:
                Tcurrent = HTlist[self.GetParentIndex(i)]

            Tcurrent = Tcurrent * T01
            HTlist += [Tcurrent]
            T01initial = link.preHT * dictJointType2HT[link.jointType](q0[i]) * link.localHT
            Tinitial = (self.GetBaseHT() if not self.HasParent(i) else HTinitialList[self.GetParentIndex(i)]) * T01initial
            HTinitialList += [Tinitial]

            #++++++++++++++++++++++++++++++++++++++++++++++
        
            localHTinv = link.localHT.Inverse()
            AthisT = localHTinv.rotation #transforms back to joint0
            pThis =  localHTinv.translation #AthisT @ np.array([-a,0,-d]) #needed for marker of next link
            
            #compute axis of previous link, for std DH, this is transformed back to previous joint
            jointAxis = dictJointType2Axis[link.jointType]
            axis0 = AthisT @ jointAxis #for drawing
            #++++++++++++++++++++++++++++++++++++++++++++++
            
            
            #rigid body parameters:
            com = link.COM #is defined within link==body frame
        
            inertiaLink = erb.RigidBodyInertia(mass=link.mass, inertiaTensor=link.inertia)
            inertiaLink = inertiaLink.Translated(com) #needs to be recomputed, because inertia in Robot is w.r.t. COM, but ObjectRigidBody needs inertia for reference point
            
            #++++++++++++++++++++++++
            #compute axis1 related to next link (for std DH, this is local z-axis in link coordinates)
            graphicsList = []
            for nextLinkIndex in nextLinks[i]:
                nextLink = self.links[nextLinkIndex]
                pNext = nextLink.preHT.translation #this defines the position for the local of the axis for next link
                axisNext = nextLink.preHT.rotation @ dictJointType2Axis[nextLink.jointType]

                if len(link.visualization.graphicsData) == 0:
                    graphicsList += self.GetLinkGraphicsData(i, pThis, pNext, axis0, axisNext, link.visualization)

            if len(link.visualization.graphicsData) != 0:
                graphicsList += link.visualization.graphicsData

            #add transformed graphicsData of tool to link graphics
            if (i==len(self.links)-1 and #tool
                self.tool.visualization.graphicsData != []):
                pOff = self.tool.HT.translation
                Aoff = self.tool.HT.rotation
                for data in self.tool.visualization.graphicsData:
                    graphicsList += [exudyn.graphics.Move(data, pOff, Aoff)] 

            
            #++++++++++++++++++++++++
            #now add body for link:
            initialArgs = {}
            if cMath.norm(Tinitial.HT44() - Tcurrent.HT44()) != 0: #the initial configuration q0 differs from the reference
                initialArgs['initialDisplacement'] = Tinitial.translation - Tcurrent.translation
                initialArgs['initialRotationMatrix'] = Tcurrent.rotation.T @ Tinitial.rotation #superimposed in the body frame
            dictLink = mbs.CreateRigidBody(referenceHT=Tcurrent, **initialArgs,
                                           inertia=inertiaLink,    
                                           gravity=self.gravity,  
                                           nodeType=rigidBodyNodeType,  
                                           graphicsDataList=graphicsList,  
                                           returnDict=True)  
            nodeList+=[dictLink['nodeNumber']  ]
            bodyList+=[dictLink['bodyNumber']]
        
            #++++++++++++++++++++++++
            #add markers and joints; the markers carry the rotation of the joint as localHT (#2745)
            mLink1 = mbs.AddMarker(eii.MarkerBodyRigid(bodyNumber=dictLink['bodyNumber'], localHT=localHTinv))

            if i == 0:
                lastMarkerRotation = link.preHT.rotation @ lastMarkerRotation #is rotationMarkerBase
                if cMath.norm(link.preHT.translation) != 0: #the first joint after the base marker, by the translation of preHT (#3270)
                    baseData = mbs.GetMarker(baseMarker)
                    if 'localHT' not in baseData:
                        raise ValueError('CreateRedundantCoordinateMBS: the preHT of the first link has a translation, which needs a baseMarker with localHT, e.g. MarkerBodyRigid')
                    baseData = {key: value for (key, value) in baseData.items() if key not in ['name', 'localPosition'] and key[0] != 'V'}
                    baseData['localHT'] = exudyn.HT(baseData['localHT']) * exudyn.HT(translation=link.preHT.translation)
                    baseMarker = mbs.AddMarker(baseData)
                (mLink0LastBody, rotationMarker0) = erb._MarkerWithRotation(mbs, baseMarker, lastMarkerRotation)
            else:
                lastMarkerRotation = link.preHT.rotation
                parentBody = bodyList[self.GetParentIndex(i)]
                mLink0LastBody = mbs.AddMarker(eii.MarkerBodyRigid(bodyNumber=parentBody,
                                                               localHT=link.preHT)) #preHT is defined in the parent link
                rotationMarker0 = np.eye(3)

            markerList0+=[mLink0LastBody]
            markerList1+=[mLink1]

            constrainedAxes = [1,1,1,1,1,1] #all axes constrained
            constrainedAxes[dictJointType2coordinate6D[link.jointType]] = 0 #this is the free axis

            r = link.visualization.jointRadius
            wJ = link.visualization.jointWidth
            showMBSjoint = link.visualization.showMBSjoint

            loadSize = 1
            
            marker0 = mLink0LastBody
            marker1 = mLink1
            rotationMarker1 = np.eye(3) #in the marker; rotationMarker0 only for a base marker that cannot take it

            jointLink = mbs.AddObject(eii.GenericJoint(markerNumbers=[marker0, marker1],
                                                    constrainedAxes=constrainedAxes,
                                                    **erb._RotationMarkerArgs(rotationMarker0, rotationMarker1), #only a rotation the base marker cannot take (#2820)
                                                    visualization=eii.VObjectJointGeneric(show=showMBSjoint, axesRadius = r*0.25, 
                                                                  axesLength=wJ*1.1, color=graphics.color.grey)))

            jointList+=[jointLink]
            objectSD = None #if not added

            if link.jointType[0] == 'R':
                #load on previous body, negative sign
                torque0 = rotationMarker0 @ (-loadSize*jointAxis) #np.array([0,0, -loadSize])
                torque1 = rotationMarker1 @ (loadSize*jointAxis) #rotated negative torque vector for current link, it is not the z-axis
                unitTorque0List += [torque0]
                unitTorque1List += [torque1]
    
                if i < len(jointLoadUserFunctionList):
                    load0 = mbs.AddLoad(eii.LoadTorqueVector(markerNumber=marker0, loadVector=torque0,
                                                                    bodyFixed=True, loadVectorUserFunction=jointLoadUserFunctionList[i]))
                    load1 = mbs.AddLoad(eii.LoadTorqueVector(markerNumber=marker1, loadVector=torque1, 
                                                                    bodyFixed=True, loadVectorUserFunction=jointLoadUserFunctionList[i]))
                    jointTorque0List += [load0]
                    jointTorque1List += [load1]
                elif createJointTorqueLoads: #loads then must be updated in, e.g., mbs.SetPreStepUserFunction(...)
                    load0 = mbs.AddLoad(eii.LoadTorqueVector(markerNumber=marker0, loadVector=[0,0,0], 
                                                                    bodyFixed=True))
                    load1 = mbs.AddLoad(eii.LoadTorqueVector(markerNumber=marker1, loadVector=[0,0,0],
                                                                    bodyFixed=True))
                    jointTorque0List += [load0]
                    jointTorque1List += [load1]
                else:
                    jointTorque0List += [None]
                    jointTorque1List += [None]
                
                if i < len(jointSpringDamperUserFunctionList) or link.HasPDcontrol():
                    PDcontrol = link.GetPDcontrol()
                    #generic node for infinite revolutions:
                    nGeneric=mbs.AddNode(eii.NodeGenericData(initialCoordinates=[q0[i]],
                                                         numberOfDataCoordinates=1)) #for infinite rotations
                    #the spring-damper acts about the z-axis of its marker frames: frames with z along the joint axis (#3270)
                    (markerSD0, markerSD1, rotationSD0) = (marker0, marker1, rotationMarker0)
                    if link.jointType != 'Rz':
                        Raxis = erb.RotationMatrixY(0.5*np.pi) if link.jointType == 'Rx' else erb.RotationMatrixX(-0.5*np.pi) #z to x or y
                        if i == 0:
                            (markerSD0, rotationSD0) = erb._MarkerWithRotation(mbs, baseMarker, lastMarkerRotation @ Raxis)
                        else:
                            markerSD0 = mbs.AddMarker(eii.MarkerBodyRigid(bodyNumber=parentBody, localHT=link.preHT*exudyn.HT(rotation=Raxis)))
                        markerSD1 = mbs.AddMarker(eii.MarkerBodyRigid(bodyNumber=dictLink['bodyNumber'], localHT=localHTinv*exudyn.HT(rotation=Raxis)))
                    #torsional spring-damper allows control of rotation
                    objectSD = mbs.AddObject(eii.TorsionalSpringDamper(markerNumbers=[markerSD0, markerSD1],
                                                        nodeNumber=nGeneric,
                                                        **erb._RotationMarkerArgs(rotationSD0, rotationMarker1), #only a rotation the base marker cannot take (#2820)
                                                        stiffness=PDcontrol[0], offset=q0[i], #holds the initial configuration
                                                        damping=PDcontrol[1],
                                                        visualization=eii.VTorsionalSpringDamper(show=False)
                                                        ))
            else:
                if (createJointTorqueLoads):
                    raise ValueError('CreateRedundantCoordinateMBS: createJointTorqueLoads only valid for Revolute joint')

                if i < len(jointSpringDamperUserFunctionList) or link.HasPDcontrol():
                    PDcontrol = link.GetPDcontrol()
                    #linear spring-damper allows control in translational direction
                    objectSD = mbs.AddObject(eii.LinearSpringDamper(markerNumbers=[marker0, marker1],
                                                                axisMarker0 = rotationMarker0 @ jointAxis, #the joint axis in marker 0
                                                                stiffness=PDcontrol[0], offset=q0[i],
                                                                damping=PDcontrol[1],
                                                                visualization=eii.VLinearSpringDamper(show=False)
                                                                ))
                    
            springDamperList += [objectSD]

            #the joint coordinate between the joint frames of the two bodies, absolute (#3270)
            if i == 0:
                baseData = mbs.GetMarker(baseMarker)
                if 'bodyNumber' not in baseData:
                    raise ValueError('CreateRedundantCoordinateMBS: the coordinate of the first joint needs a baseMarker on a body, e.g. MarkerBodyRigid')
                bodyPrevious = baseData['bodyNumber']
                baseHT = exudyn.HT(baseData['localHT']) if 'localHT' in baseData else exudyn.HT(translation=baseData.get('localPosition', [0,0,0]))
                localPosition0 = baseHT.translation
                localRotation0 = baseHT.rotation @ lastMarkerRotation
            else:
                bodyPrevious = bodyList[self.GetParentIndex(i)]
                localPosition0 = link.preHT.translation
                localRotation0 = link.preHT.rotation
            coordinateArgs = {'bodyNumbers': [bodyPrevious, dictLink['bodyNumber']], 'localPosition0': localPosition0,
                              'localPosition1': localHTinv.translation}
            if link.jointType[0] == 'R':
                mCoordinate = mbs.AddMarker(eii.MarkerBodiesRelativeRotationCoordinate(**coordinateArgs, axis0=jointAxis,
                    localRotation0=localRotation0, localRotation1=localHTinv.rotation,
                    nodeNumber=mbs.AddNode(eii.NodeGenericData(numberOfDataCoordinates=1, initialCoordinates=[q0[i]])))) #continuous angles
            else:
                mCoordinate = mbs.AddMarker(eii.MarkerBodiesRelativeTranslationCoordinate(**coordinateArgs, axis0=localRotation0 @ jointAxis))
            jointCoordinateMarkerList += [mCoordinate]
            connectors = (None, None)
            if self._HasJointConnector(i):
                if groundMarker is None:
                    groundMarker = mbs.AddMarker(eii.MarkerNodeCoordinate(nodeNumber=mbs.AddNode(eii.NodePointGround()), coordinate=0))
                connectors = self._AddJointConnectors(mbs, i, groundMarker, mCoordinate, 0.)
            jointFrictionList += [connectors[0]]
            jointEndStopList += [connectors[1]]

            
            #end loop over links
            #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        #return some needed variables for further use        
        d = {'nodeList': nodeList,'bodyList': bodyList,'jointList': jointList,
             'markerList0': markerList0,
             'markerList1': markerList1,
             'springDamperList': springDamperList,
             'jointTorque0List': jointTorque0List,
             'jointTorque1List': jointTorque1List,
             'unitTorque0List': unitTorque0List,
             'unitTorque1List': unitTorque1List,
             'jointCoordinateMarkerList': jointCoordinateMarkerList,
             'jointFrictionList': jointFrictionList, 'jointEndStopList': jointEndStopList,
             'baseObject':baseObject}
        self.modelData = dict(d, modelType='redundant') #for PreStepControl and AddSensors (#3270)
        return d
    
        
    

    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    def GetKinematicTree66(self):
        """export kinematicTree
        """
        from exudyn.kinematicTree import KinematicTree66, Inertia2T66
        
        jointTypes = []
        transformations = []
        inertias = []
        n = len(self.links)
        for i in range(n):
            link = self.links[i]
            jointTypes += [link.jointType]
            
            Amat = link.preHT.rotation
            vVec = link.preHT.translation
            X=erb.RotationTranslation2T66Inverse(A=Amat, v=vVec)
            if cMath.norm(link.localHT.HT44() - np.eye(4)) > 1e-15:
                raise ValueError('GetKinematicTree66(): not implemented for links with localHT other than the identity')
            
            transformations += [X] #defines transformation to joint in parent link
            J = erb.RigidBodyInertia(mass=link.mass, inertiaTensor=link.inertia) #link.inertia around COM
            J = J.Translated(link.COM)
            inertias += [Inertia2T66(J)]
        
        KT=KinematicTree66(listOfJointTypes=jointTypes, 
                           listOfTransformations=transformations, 
                           listOfInertias=inertias, 
                           gravity=self.gravity)
        return KT
    
    def GetLinkGraphicsData(self, i, p0, p1, axis0, axis1, linkVisualization):
        """create link GraphicsData (list) for link i; internally used in CreateRedundantCoordinateMBS(...); linkVisualization contains visualization dict of link
        """
        
        com = self.links[i].COM
        graphicsList = []

        r = linkVisualization.jointRadius
        wJ = linkVisualization.jointWidth
        wL = linkVisualization.linkWidth
        color = linkVisualization.linkColor

        #draw COM:
        if linkVisualization.showCOM:
            dd = r*0.2
            colorCOM = copy(color)
            colorCOM[0] *= 0.8 #make COM a little darker
            colorCOM[1] *= 0.8
            colorCOM[2] *= 0.8
            graphicsList += [exudyn.graphics.Brick(com, [dd,dd,dd], colorCOM)]

        #draw link:
        if r != 0:
            h0 = 0.5*wJ   #height of half axis, first joint
            h1 = 0.5*wJ   #height of half axis, second joint

            if i != 0:
                graphicsList += [exudyn.graphics.Cylinder(pAxis=p0, vAxis=-h0*np.array(axis0), 
                                                      radius=r, color=color)]
            
            graphicsList += [exudyn.graphics.Cylinder(pAxis=p1, vAxis= h1*np.array(axis1), 
                                                      radius=r, color=color)]

            #draw body as cylinder:
            if cMath.norm(np.array(p1) - p0) > 1e-15:
                graphicsList += [exudyn.graphics.Cylinder(pAxis=p1, vAxis=(np.array(p0) - p1), 
                                                      radius=0.5*wL, color=color)]
        
        return graphicsList

    
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  DH-PARAMETERS  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def StdDH2HT(DHparameters):
    """compute the homogeneous transformation HT, an exudyn.HT, from standard DHparameters=[theta, d, a, alpha]
    """
    #optimized version:
    [theta, d, a, alpha] = DHparameters
    ct = cMath.cos(theta)
    st = cMath.sin(theta)
    ca = cMath.cos(alpha)
    sa = cMath.sin(alpha)
    return exudyn.HT(rotation=[[ct,-st*ca, st*sa],
                               [st, ct*ca,-ct*sa],
                               [ 0, sa   , ca   ]], translation=[a*ct, a*st, d])

#\mfour{\cos \theta_j &-\sin \theta_j \cos \alpha_j & \sin \theta_j \sin \alpha_j & a_j \cos \theta_j}
#                                                        {\sin \theta_j & \cos \theta_j \cos \alpha_j &-\cos \theta_j \sin \alpha_j & a_j \sin \theta_j}
#                                                        {0             & \sin \alpha_j               & \cos \alpha_j               & d_j }
#                                                        {0 & 0 & 0 & 1}

def ModDHKK2HT(DHparameters):
    """compute pre- and post- homogeneous transformations (exudyn.HT) from modified Denavit-Hartenberg DHparameters=[alpha, d, theta, r]; returns [HTpre, HTpost]; HTpre is transformation before axis rotation, HTpost includes axis rotation and everything hereafter; modified DH-Parameters according to Khalil and Kleinfinger, 1986
    """
    [alpha, d, theta, r] = DHparameters
    return [exudyn.HT().SetRotationX(alpha) * exudyn.HT(translation=[d,0,0]), exudyn.HT().SetRotationZ(theta) * exudyn.HT(translation=[0,0,r])]

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@docmeta(author='Peter Manzl')
def projectAngleToPMPi(q0): 
    r"""This function projects an angle in the range $[-min_{float}, +max_{float}]$ fo the range $[-\pi, +\pi]$

    Args:
        q0: An angle either as scalar, list or array

    Returns:
        qProj: The angle projected into the range $[-\pi to \pi]$
    """
    if type(q0) == list: 
        q0 = np.array(q0) # cast to array for modulo to work
    q1 =  q0 % (2*np.pi) # in range 0 to 2*np.pi
    qProj = q1 - 2*np.pi*(q1 > np.pi)
    return qProj


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@docmeta(author='Peter Manzl, Johannes Gerstmayr')
class InverseKinematicsNumerical():
    """DEPRECATED: the numerical inverse kinematics of a serial robot, by the Levenberg-Marquardt iteration of
    KinematicChain.InverseKinematicsNumerical (#3269); it solved a static problem of a kinematic tree before; use
    Robot.InverseKinematicsNumerical or robot.GetKinematicChain().InverseKinematicsNumerical instead
    """
    @Deprecated('1.13.0', 2029, use='Robot.InverseKinematicsNumerical')
    def __init__(self, robot, jointStiffness = 1e0, useRenderer=False, flagDebug=False,
                 useAlternativeConstraints=False):
        """DEPRECATED: the inverse kinematics of robot

        Args:
            robot: robot class
            jointStiffness: no effect (of the former static solution)
            useRenderer: no effect (of the former static solution)
            flagDebug: no effect (of the former static solution)
            useAlternativeConstraints: no effect (of the former static solution)
        """
        self.robot = robot
        self.chain = robot.GetKinematicChain()
        self.q = np.zeros(len(robot.links)) if len(robot.referenceConfiguration) != len(robot.links) \
            else np.array(robot.referenceConfiguration, dtype=float)

    def GetCurrentRobotHT(self):
        """the HT of the tool at the last solution (or the reference configuration), an exudyn.HT"""
        return self.chain.ToolHT(self.q)

    @docmeta(author='Peter Manzl')
    def InterpolateHTs(self, T1, T2, rotStep=np.pi/16, minSteps = 1): 
        """

        Args:
            T1: 4x4 homogeneous transformation matrix representing the first Pose
            T2: 4x4 homogeneous transformation matrix representing the second Pose
            rotStep: the max. size of steps to take for the orientation
            minSteps: minimum number of substeps to interpolate

        Returns:
            T: a List of homogeneous transformations (exudyn.HT) for each step between

        Note:
            still under development; interpolation may be changed to using logSE3
        """
        (T1, T2) = (np.asarray(T1), np.asarray(T2)) #a 4x4 array or an exudyn.HT
        R1, t1 = T1[:3,:3], erb.HT2translation(T1)
        R2, t2 = T2[:3,:3], erb.HT2translation(T2)
        t12 = t2 - t1
        R12 = np.transpose(R1) @ R2
        rot12 = erb.RotationMatrix2RotationVector(R12)
        rotAng = cMath.norm(rot12, 2) # rotation vector is angle * normalized rotation axis 


        n = max(minSteps, 1+int(rotAng/rotStep)) # number of steps (#3255)
        T = []
        for i in range(n): 
            roti = rot12*(i+1)/n
            Ri = R1 @ erb.RotationVector2RotationMatrix(roti)
            ti = t1 + (t2-t1)*(i+1)/n 
            Ti = exudyn.HT(rotation=Ri, translation=ti)
            T += [Ti]
        T += [_ToHT(T2)] # to satisfy the boundry condition
        return T
    
    def SolveSafe(self, T, q0 = None):
        """the inverse kinematics of Solve with 10 restarts

        Args:
            T: the desired pose of the tool, an exudyn.HT or a 4x4 matrix
            q0: the joint coordinates to start from; None: the last solution

        Returns:
            [q, success]; q is None if no solution was found
        """
        return self._Solve(T, q0, 10)

    def Solve(self, T, q0 = None):
        """the inverse kinematics by KinematicChain.InverseKinematicsNumerical from q0

        Args:
            T: the desired pose of the tool, an exudyn.HT or a 4x4 matrix
            q0: the joint coordinates to start from; None: the last solution

        Returns:
            [q, success]; q is None if no solution was found
        """
        return self._Solve(T, q0, 0)

    def _Solve(self, T, q0, restarts):
        (q, success, iterations) = self.chain.InverseKinematicsNumerical(_ToHT(T), self.q if q0 is None else q0,
                                                                         restarts=restarts)
        if not success:
            return [None, False]
        self.q = projectAngleToPMPi(q)
        return [self.q, True]
