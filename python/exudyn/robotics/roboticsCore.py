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
# Example:    New robot model uses the class Robot with class RobotLink; the old dictionary structure is defined in the example in ComputeJointHT for the definition of the 'robot' dictionary.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#constants and fixed structures:
import numpy as np

import exudyn

import exudyn.itemInterface as eii
import exudyn.basicUtilities as ebu
import exudyn.advancedUtilities as eau

#import exudyn.utilities as eut
import exudyn.rigidBodyUtilities as erb
import exudyn.graphicsDataUtilities as egd
import exudyn.graphics as graphics

from copy import copy, deepcopy
import time #for timer in InverseKinematicsNumerical


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

#define dictionary for joint transformations as homogeneous transformations, replacement for switch/case
dictJointType2HT = {
    'Rx':erb.HTrotateX, #revolute joint for local X axis
    'Ry':erb.HTrotateY, #revolute joint for local Y axis
    'Rz':erb.HTrotateZ, #revolute joint for local Z axis
    'Px':erb.HTtranslateX, #prismatic joint for local X axis
    'Py':erb.HTtranslateY, #prismatic joint for local Y axis
    'Pz':erb.HTtranslateZ, #prismatic joint for local Z axis
    }

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

#**class: class to define visualization of RobotLink
class VRobotLink:
    #**classFunction: initialize robot link with parameters, being self-explaining
    #**input:
    #  jointRadius: radius of joint to draw
    #  jointWidth: length or width of joint (depending on type of joint)
    #  showMBSjoint: if False, joint is not drawn
    #  linkWidth: width of link for default drawing
    #  linkColor: color of link for default drawing
    #  showCOM: if True, center of mass is marked with cube
    #  graphicsData: list of GraphicsData to represent link; if list is empty, link graphics will be generated from link geometry data; otherwise, drawing will be taken from graphicsData, and only showMBSjoint and showCOM flags will add additional graphics
    def __init__(self, jointRadius = 0.06, jointWidth = 0.12, linkWidth = 0.1, showMBSjoint = True, showCOM = True, 
                 linkColor = [0.4,0.4,0.4,1], graphicsData = [] ):
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


#**class: class to define one link of a robot
class RobotLink:
    #**classFunction: initialize robot link
    #**input:
    #  mass: mass of robot link
    #  COM: center of mass in link coordinate system
    #  inertia: 3x3 matrix (list of lists / numpy array) containing inertia tensor in link coordinates, with respect to center of mass
    #  localHT: 4x4 matrix (list of lists / numpy array) containing homogeneous transformation from local joint to link coordinates; default = identity; currently, this transformation is not available in KinematicTree, therefore the link inertia and COM must be transformed accordingly
    #  preHT: 4x4 matrix (list of lists / numpy array) containing homogeneous transformation from previous link to this joint; default = identity
    #  jointType: string containing joint type, out of: 'Rx', 'Ry', 'Rz' for revolute joints and 'Px', 'Py', 'Pz' for prismatic joints around/along the respecitive local axes
    #  parent: for building robots as kinematic tree; use '-2' to automatically set parents for serial robot (on fixed base), use '-1' for ground-parent and any other 0-based index for connection to parent link
    #  PDcontrol: tuple of P and D control values, defining position (rotation) proportional value P and velocitiy proportional value D
    #  visualization: VRobotLink structure containing options for drawing of link and joints; see class VRobotLink
    def __init__(self, mass, COM, inertia, localHT=erb.HT0(), jointType='Rz', parent=-2, preHT=erb.HT0(), PDcontrol=(None,None), visualization=VRobotLink()):
        self.mass = mass
        self.COM = np.array(COM)
        self.inertia = np.array(inertia)
        self.localHT = np.array(localHT)
        self.preHT = np.array(preHT)
        self.jointType = jointType
        self.parent = parent
        self.visualization = deepcopy(visualization)
        if PDcontrol[0] != None:
            self.PDcontrol = PDcontrol

    #**classFunction: set PD control values for drive of joint related to link using position-proportional value P and differential value (velocity proportional) D
    def SetPDcontrol(self, Pvalue, Dvalue):
        self.PDcontrol = (Pvalue, Dvalue)

    #**classFunction: check if contrl is available
    def HasPDcontrol(self):
        return hasattr(self, 'PDcontrol')

    #**classFunction: get PD control values
    def GetPDcontrol(self):
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
        s += '\n  visualization:\n' + str(self.visualization)
        if self.HasPDcontrol():
            s += '\n  PDcontrol = ' + str(self.PDcontrol)
        return s

    def __repr__(self):
        return str(self)
        
#**class: class to define visualization of RobotTool
class VRobotTool:
    #**classFunction: initialize robot tool with parameters; currently only graphicsData, which is a list of GraphicsData same as in mbs Objects
    def __init__(self, graphicsData=[]):
        self.graphicsData = copy(graphicsData)

#**class: define tool of robot: containing graphics and HT (may add features in future)
class RobotTool:
    #**classFunction: initialize robot tool
    #**input:
    #  HT: 4x4 matrix (list of lists / numpy array) containing homogeneous transformation to transform from last link to tool
    #  graphicsData: dictionary containing a list of GraphicsData, same as in exudyn Objects
    def __init__(self, HT=erb.HT0(), visualization=VRobotTool()):
        self.HT = np.array(HT)
        self.visualization = visualization
        
    def __str__(self):
        s = '  HT = ' + str(self.HT)
        #s += '\nvisualization = ' + str(self.visualization)
        return s
    def __repr__(self):
        return str(self)

#**class: class to define visualization of RobotBase
class VRobotBase:
    #**classFunction: initialize robot base with parameters; currently only graphicsData, which is a list of GraphicsData same as in mbs Objects
    def __init__(self, graphicsData=[]):
        self.graphicsData = copy(graphicsData)

    
#**class: define base of robot: containing graphics and HT (may add features in future)
class RobotBase:
    #**classFunction: initialize robot base
    #**input:
    #  HT: 4x4 matrix (list of lists / numpy array) containing homogeneous transformation to transform from world coordinates to base coordinates (changes orientation and position of robot)
    #  graphicsData: dictionary containing a list of GraphicsData, same as in exudyn Objects
    def __init__(self, HT=erb.HT0(), visualization=VRobotBase()):
        self.HT = np.array(HT)
        self.visualization = visualization
    
    def __str__(self):
        s = '  HT = ' + str(self.HT)
        #s += '\nvisualization = ' + str(self.visualization)
        return s

    def __repr__(self):
        return str(self)


buildFromDictionaryWarned = False #do not use this variable, it is for deprecation warnings!
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  Define robot ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**class: class to define a robot
class Robot:
    #**classFunction: initialize robot class
    #**input:
    #  base: definition of base using RobotBase() class
    #  tool: definition of tool using RobotTool() class
    #  gravity: a list or 3D numpy array defining gravity
    #  referenceConfiguration: a list of scalar quantities defining the parameters for reference configuration
    def __init__(self, 
                 gravity=[0,0,-9.81],
                 base = RobotBase(),
                 tool = RobotTool(),
                 referenceConfiguration = []
                 ):
        self.gravity = np.array(gravity)
        self.base = base
        self.tool = tool
        self.referenceConfiguration = np.array(referenceConfiguration)
        self.links = [] #initialize list of link data
        self.isSerialRobot = True #this is true as long as parent = link index - 1

    def __str__(self):
        s = 'gravity = ' + str(self.gravity)
        s += '\nreferenceConfiguration = ' + str(self.referenceConfiguration)
        s += '\nbase: \n' + str(self.base)
        s += '\ntool: \n' + str(self.tool)
        s += '\nlinks: \n' + str(self.links)
        return s
    
    def __repr__(self):
        return str(self)
        

    #**classFunction: add a link to serial robot
    def AddLink(self, robotLink):
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
        
        if not self.isSerialRobot and (np.linalg.norm(self.tool.HT - erb.HT0()) >= 1e-15
            or self.tool.visualization.graphicsData != []):
            exudyn.Print('Warning: class Robot: tool defined in kinematic tree; currently tool is only allowed for serial robots')

        return i #return index of link

    #**classFunction: return True, if robot is a serial robot
    def IsSerialRobot(self):
        return self.isSerialRobot

    #**classFunction: return Link object of link i
    def GetLink(self, i):
        return self.links[i]

    #**classFunction: True if link has parent, False if not
    def HasParent(self, i):
        return self.links[i].parent >= 0

    #**classFunction: Get index of parent link; for serial robot this is simple, but for general trees, there is a index list
    def GetParentIndex(self, i):
        return self.links[i].parent

    
    #**classFunction: return number of links
    def NumberOfLinks(self):
        return len(self.links)

    #**classFunction: return base as homogeneous transformation
    def GetBaseHT(self):
        return self.base.HT

    #**classFunction: return base as homogeneous transformation
    def GetToolHT(self):
        return self.tool.HT
    
    #**classFunction: compute list of homogeneous transformations for every link, using current joint coordinates q; leads to different results for standard and modified DH parameters because link coordinates are different!
    def LinkHT(self, q):
        HT = []

        # #only for serial robots:
        # Tcurrent = self.base.HT
        # for i in range(len(self.links)):
        #     link = self.links[i]

        #     #call function to compute HT for joint rotation/translation:
        #     T01 = link.preHT @ dictJointType2HT[link.jointType](q[i]) @ link.localHT

        #     Tcurrent = Tcurrent @ T01
        #     HT += [copy(Tcurrent)]


        for i in range(len(self.links)):
            link = self.links[i]

            T01 = link.preHT @ dictJointType2HT[link.jointType](q[i]) @ link.localHT
            if self.HasParent(i):
                pIndex = self.GetParentIndex(i)
                Tcurrent = HT[pIndex] @ T01
            else:
                Tcurrent = self.base.HT @ T01
            HT += [copy(Tcurrent)]
        
        return HT    

    #**classFunction: compute list of homogeneous transformations for every joint (after rotation), using current joint coordinates q
    def JointHT(self, q):
        HT = []

        # #only for serial robots:
        # Tcurrent = self.base.HT
        # for i in range(len(self.links)):
        #     link = self.links[i]

        #     T01 = link.preHT @ dictJointType2HT[link.jointType](q[i])
        #     Tcurrent = Tcurrent @ T01
        #     HT += [copy(Tcurrent)]
            
        #     Tcurrent = Tcurrent @ link.localHT

        for i in range(len(self.links)):
            link = self.links[i]

            T01 = link.preHT @ dictJointType2HT[link.jointType](q[i])
            if self.HasParent(i):
                pIndex = self.GetParentIndex(i)
                Tcurrent = HT[pIndex] @ self.links[pIndex].localHT @ T01
            else:
                Tcurrent = self.base.HT @ T01
            HT += [copy(Tcurrent)]
        return HT

    #**classFunction: compute list of  homogeneous transformations HT from base to every COM using HT list from Robot.JointHT(...)
    def COMHT(self, HT):
        HTCOM = []
        
        for i in range(len(self.links)):
            HTCOM += [HT[i] @ self.links[i].localHT @ erb.HTtranslate(self.links[i].COM)]
        
        return HTCOM
    
    #**classFunction: compute list of joint torques for serial robot due to gravity (gravity and mass as given in robot), taking HT from Robot.JointHT()
    def StaticTorques(self,HT):
        jointTorques = np.zeros(np.size(self.links))
    
        #compute HTs for COM
        HTcom=self.COMHT(HT)
        
        #sum up the torques of all gravity loads:
        for i in range(len(HTcom)):
            p = erb.HT2translation(HTcom[i])
            Jcom = self.Jacobian(HT[0:i+1],toolPosition=p,mode='trans')
            fG = self.links[i].mass * self.gravity
            tau = Jcom.T @ fG
            jointTorques[0:i+1] += tau
        return jointTorques
    
    
    #**classFunction: compute jacobian for translation and rotation at toolPosition using joint HT; this is using the Robot functions, but is inefficient for simulation purposes
    #**input:
    #  HT: list of homogeneous transformations per joint , as computed by Robot.JointHT(...)
    #  toolPosition: global position at which the jacobian is evaluated (e.g., COM); if empty [], it uses the origin of the last link
    #  mode: 'all'...translation and rotation jacobian, 'trans'...only translation part, 'rot': only rotation part
    #  linkIndex: link index for which the jacobian is evaluated; if linkIndex==None, it uses the last link provided in HT
    #**output: returns jacobian with translation and rotation parts in rows (3 or 6) according to mode, and one column per HT; in the kinematic tree the columns not related to linkIndex remain zero
    def Jacobian(self, HT, toolPosition=[], mode='all', linkIndex=None):
        n = len(HT)
        if n > len(self.links):
            exudyn.Print("ERROR: number of homogeneous transformations (HT) greater than number of links")

        #link index is usually the last link contained in HT (all subsequent columns in jacobian would be anyway zero):
        if linkIndex == None:
            linkIndex = n-1
    
        Jomega = np.zeros((3,n))#rotation part of jacobian
        Jvel = np.zeros((3,n))  #translation part of jacobian
        #A = erb.HT2rotationMatrix(self.GetBaseHT())
        #vPrevious = erb.HT2translation(self.GetBaseHT())
        
        vn = list(toolPosition)
        if len(vn) == 0:
            vn = erb.HT2translation(HT[linkIndex] @ self.links[linkIndex].localHT) #last link coordinates


        #for tree, we may only consider links in the chain from link to base!
        i = linkIndex
        endReached = False
        while not endReached:
            #if i > 0:
            A = erb.HT2rotationMatrix(HT[i]) #rotation of joint i

            # localAxis = erb.HT2rotationMatrix(self.links[i].preHT) @ dictJointType2Axis[self.links[i].jointType]
            localAxis = dictJointType2Axis[self.links[i].jointType]

            axis = A @ localAxis #axis in global coordinates
            #OLD DH parameter based: Jomega[0:3,i] = robot['jointType'][i] * axis #only considered, if revolute joint
            if self.links[i].jointType[0] == 'R':
                Jomega[0:3,i] = axis #only considered, if revolute joint
            
            #vPrevious = erb.HT2translation(HT[i] @ self.links[i].preHT)
            vPrevious = erb.HT2translation(HT[i])
             
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


    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: Add a ObjectKinematicTree to existing mbs from the robot structure inside this robot class;
    #                 Joints defined by the kinematics as well as links (and inertia) are transferred to the kinematic tree object;
    #                 Current implementation only works for serial robots;
    #                 Control can be realized simply by adding PDcontrol to RobotLink structures, then modifying jointPositionOffsetVector and jointVelocityOffsetVector in ObjectKinematicTree; force offsets (e.g., static or dynamic torque compensation) can be added to KinematicTree jointForceVector; more general control can be added by using KinematicTree forceUserFunction;
    #                 The coordinates in KinematicTree (as well as jointPositionOffsetVector, etc.) are sorted in the order as the RobotLinks are added to the Robot class;
    #                 Note that the ObjectKinematicTree is still under development and interfaces may change.
    #**input: 
    #   mbs: the multibody system, which will be extended
    #   name: object name in KinematicTree; transferred to KinematicTree, default = ''
    #   forceUserFunction: defines the user function for computation of joint forces in KinematicTree; transferred to KinematicTree, default = 0
    #**output: the function returns a dictionary containing 'nodeGeneric': generic ODE2 node number ,'objectKinematicTree': the kinematic tree object, 'baseObject': the base object if created, otherwise None; further values will be added in future
    def CreateKinematicTree(self, mbs, name = '', forceUserFunction = 0):
        #def CreateKinematicTree(self, mbs, jointSpringDamperUserFunctionList=[]):

        #add graphics for base:
        baseObject = None #if it does not exist
        baseOffset = erb.HT2translation(self.base.HT)

        if self.base.visualization.graphicsData != []:
            #add a ground object at base position
            graphicsDataBase = []
            pOff = baseOffset
            Aoff = erb.HT2rotationMatrix(self.base.HT)
            for data in self.base.visualization.graphicsData:
                graphicsDataBase += [exudyn.graphics.Move(data, [0,0,0], Aoff)] #only rotated, translation is in ground

            baseObject = mbs.AddObject(eii.ObjectGround(referencePosition=pOff, 
                                                    visualization=eii.VObjectGround(graphicsData=graphicsDataBase)))

        #+++++++++++++++++++++++
        #Tcurrent = self.GetBaseHT()
        qRef = self.referenceConfiguration
        
        nLinks = len(self.links)
        graphicsDataList = []   #list of graphicsData per link
        jointTypesList = []     #exudyn joint types
        linkParents = []

        jointTransformations=[]
        jointOffsets=[]
        linkInertiasCOM=[] 
        linkCOMs=[]
        linkMasses=[]

        #if PD control exists, this vector is kept, otherwise erased:
        jointPControlVector = [0]*nLinks
        jointDControlVector = [0]*nLinks
        jointPositionOffsetVector = [0]*nLinks
        jointVelocityOffsetVector = [0]*nLinks
        hasPDcontrol = False #True, if any link has control
        
        #create robot tree:
        for i in range(nLinks):
            link = self.links[i]
            linkParents += [self.GetParentIndex(i)]

            # if np.linalg.norm(link.localHT - erb.HT0()) >= 1e-14: #now implemented
            #     raise ValueError('CreateKinematicTree: can only convert robots with localHT = identity')
            if link.jointType not in dictJointType2Axis:
                raise ValueError('CreateKinematicTree: found invalid joint type in link '+str(i)+':'+link.jointType)


            axis = dictJointType2Axis[link.jointType]
            
            if link.HasPDcontrol():
                jointPControlVector[i] = link.GetPDcontrol()[0]
                jointDControlVector[i] = link.GetPDcontrol()[1]
                hasPDcontrol = True
                
            # com = link.COM
            #++++++++++++++++++++++++++++++++++
            jointTypesList += [dictJointTypeText2Exudyn[link.jointType]]
            # jointTransformations += [erb.HT2rotationMatrix(link.preHT)] 
            # jointOffsets += [erb.HT2translation(link.preHT)]
            
            # 
            parentLinkLocalHT = erb.HT0()
            if self.HasParent(i):
                parentLinkLocalHT = self.links[self.GetParentIndex(i)].localHT
            jointTransformations += [erb.HT2rotationMatrix(parentLinkLocalHT @ link.preHT)] 
            jointOffsets += [erb.HT2translation(parentLinkLocalHT @ link.preHT)]

            #inertia is defined in link coordinates; but KinematicTree needs inertia w.r.t. joint coordinates:
            rbi = erb.RigidBodyInertia()
            rbi.SetWithCOMinertia(link.mass, link.inertia, link.COM)
    
            rbi = rbi.Transformed((link.localHT)) #inertia parameters need to be transformed to joint frame
            
            linkInertiasCOM += [rbi.InertiaCOM()] #KinematicTree needs inertia w.r.t. COM
            linkCOMs += [rbi.COM()] 
            linkMasses += [rbi.Mass()]
            # linkInertiasCOM += [link.inertia] #is already w.r.t. COM
            # linkCOMs += [link.COM]
            # linkMasses += [link.mass]

            
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
                #parentLink = self.GetLink(iParent)
                vParent = erb.HT2translation(parentLinkLocalHT@link.preHT)
                if len(linkVisualization.graphicsData) == 0:
                    gLink = exudyn.graphics.Cylinder([0.,0.,0.], vParent, radius=0.5*wL, color=color)
                    graphicsDataList[iParent] += [gLink]

            if len(linkVisualization.graphicsData) != 0:
                graphicsDataLink += linkVisualization.graphicsData

            #add transformed graphicsData of tool to link graphics
            if (i==len(self.links)-1 and #tool
                self.tool.visualization.graphicsData != []):
                pOff = erb.HT2translation(self.tool.HT)
                Aoff = erb.HT2rotationMatrix(self.tool.HT)
                for data in self.tool.visualization.graphicsData:
                    graphicsDataLink += [exudyn.graphics.Move(data, pOff, Aoff)] 

            graphicsDataList += [graphicsDataLink]
        
        #create node for unknowns of KinematicTree
        nGeneric = mbs.AddNode(eii.NodeGenericODE2(referenceCoordinates=qRef,
                                               initialCoordinates=[0.]*nLinks,
                                               initialCoordinates_t=[0.]*nLinks,
                                               numberOfODE2Coordinates=nLinks))

        if not hasPDcontrol: #use empty lists, makes model more efficient
            jointPControlVector = []
            jointDControlVector = []
            jointPositionOffsetVector = [] #size must agree with jointPControlVector
            jointVelocityOffsetVector = [] #size must agree with jointDControlVector

        jointForceVector = [] #[0]*nLinks #not used right now


        #create KinematicTree
        oKT = mbs.AddObject(eii.ObjectKinematicTree(nodeNumber=nGeneric, jointTypes=jointTypesList, linkParents=linkParents,
                                          jointTransformations=exudyn.Matrix3DList(jointTransformations), 
                                          jointOffsets=exudyn.Vector3DList(jointOffsets), 
                                          linkInertiasCOM=exudyn.Matrix3DList(linkInertiasCOM), 
                                          linkCOMs=exudyn.Vector3DList(linkCOMs), linkMasses=linkMasses, 
                                          baseOffset = baseOffset, gravity=self.gravity, 
                                          jointForceVector=jointForceVector, forceUserFunction=forceUserFunction,
                                          jointPositionOffsetVector = jointPositionOffsetVector, jointPControlVector = jointPControlVector,
                                          jointVelocityOffsetVector = jointVelocityOffsetVector, jointDControlVector = jointDControlVector,
                                          visualization=eii.VObjectKinematicTree(graphicsDataList = graphicsDataList)))

        #return some needed variables for further use        
        d = {'nodeGeneric': nGeneric, 'objectKinematicTree': oKT, 'baseObject': baseObject,
             }
        return d

                      

    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: Add items to existing mbs from the robot structure inside this robot class; robot is attached to baseMarker (can be ground object or moving/deformable body);
    #                 The (serial) robot is built as rigid bodies (containing rigid body nodes), where bodies represent the links which are connected by joints; 
    #                 Add optional jointSpringDamperUserFunctionList for individual control of joints; otherwise use PDcontrol in RobotLink structure; additional joint torques/forces can be added via spring damper, using mbs.SetObjectParameter(...) function;
    #                 See several Python examples, e.g., \texttt{serialRobotTestTSD.py}, in Examples or TestModels;
    #                 For more efficient models, use CreateKinematicTree(...) function!
    #**input: 
    #   mbs: the multibody system, which will be extended
    #   baseMarker: a rigid body marker, at which the robot will be placed (usually ground); note that the local coordinate system of the base must be in accordance with the DH-parameters, i.e., the z-axis must be the first rotation axis. For correction of the base coordinate system, use rotationMarkerBase
    #   jointSpringDamperUserFunctionList: NOT IMPLEMENTED yet: jointSpringDamperUserFunctionLista list of user functions for actuation of joints with more efficient spring-damper based connector (spring-damper directly emulates PD-controller); uses torsional spring damper for revolute joints and linear spring damper for prismatic joints; can be empty list (no spring dampers); if entry of list is 0, no user function is created, just pure spring damper; parameters are taken from RobotLink PDcontrol structure, which MUST be defined using SetPDcontrol(...) in RobotLink
    #   jointLoadUserFunctionList: DEPRECATED: a list of user functions for actuation of joints according to a LoadTorqueVector userFunction, see serialRobotTest.py as an example; can be empty list
    #   createJointTorqueLoads: DEPRECATED: if True, independently of jointLoadUserFunctionList, joint loads are created; the load numbers are stored in lists jointTorque0List/ jointTorque1List; the loads contain zero torques and need to be updated in every computation step, e.g., using a preStepUserFunction; unitTorque0List/ unitTorque1List contain the unit torque vector for the according body(link) which needs to be applied on both bodies attached to the joint
    #   rotationMarkerBase: add a numpy 3x3 matrix for rotation of the base, in order that the robot can be attached to any rotated base marker; the rotationMarkerBase is according to the definition in GenericJoint; note, that for moving base, the static compensation does not work (base rotation must be updated)
    #   rigidBodyNodeType: specify node type of rigid body node, e.g., exudyn.NodeType.RotationEulerParameters, etc.
    #**output: the function returns a dictionary containing per link nodes and object (body) numbers, 'nodeList', 'bodyList', the object numbers for joints, 'jointList', list of load numbers for joint torques (jointTorque0List, jointTorque1List); unit torque vectors in local coordinates of the bodies to which the torques are applied (unitTorque0List, unitTorque1List); springDamperList contains the spring dampers if defined by PDcontrol of links
    def CreateRedundantCoordinateMBS(self, mbs, baseMarker, jointSpringDamperUserFunctionList= [], 
                                     jointLoadUserFunctionList=[], 
                                     createJointTorqueLoads=True, rotationMarkerBase=None,
                                     rigidBodyNodeType=exudyn.NodeType.RotationEulerParameters): 
        #build robot model:
        nodeList = []           #node number or rigid node for link
        bodyList = []           #body number or rigid body for link
        jointList = []          #joint which links to previous link or base
        markerList0 = []        #contains n marker numbers per link (=body) which connect to previous/left link
        markerList1 = []        #contains n marker numbers per link (=body) which connect to next/right link
        jointTorque0List = []   #load number of joint torque at previous/left link (negative); 
        jointTorque1List = []   #load number of joint torque at next/right link (positive)
        unitTorque0List = []    #contains unit torque0 (previous/left link) for joint i, should be multiplied with according factor to represent joint torque
        unitTorque1List = []    #contains unit torque1 (next/right link) for joint i, should be multiplied with according factor to represent joint torque
        springDamperList = []   #contains torsional or linear spring dampers for control of joint axes
        
        HTlist = [] #list of homogeneous transformations for links stored (for parent link relation in tree)

        #delete: lastMarker = baseMarker
        lastMarkerRotation = np.identity(3) #base rotation included in marker
        if rotationMarkerBase != None:
            lastMarkerRotation = rotationMarkerBase
            
        qRef = self.referenceConfiguration
        
        baseObject = -1 #if it does not exist
        if self.base.visualization.graphicsData != []:
            #add a ground object at base position
            graphicsDataBase = []
            pOff = erb.HT2translation(self.base.HT)
            Aoff = erb.HT2rotationMatrix(self.base.HT)
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
        
        
            # T01 = DH2HT(DHparam) #transformation from last link to this link; it defines the orientation of the body
            T01 = link.preHT @ dictJointType2HT[link.jointType](qRef[i]) @ link.localHT #new bodies are placed at origin of link frame
            if not self.HasParent(i):
                Tcurrent = self.GetBaseHT()
            else:
                Tcurrent = HTlist[self.GetParentIndex(i)]

            Tcurrent = Tcurrent @ T01
            HTlist += [copy(Tcurrent)]

            #++++++++++++++++++++++++++++++++++++++++++++++
        
            localHTinv = erb.InverseHT(link.localHT)
            AthisT = erb.HT2rotationMatrix(localHTinv) #transforms back to joint0
            pThis =  erb.HT2translation(localHTinv) #AthisT @ np.array([-a,0,-d]) #needed for marker of next link
            
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
                # if i == len(self.links)-1:
                #     pNext = np.array([0,0,0.]) #use local position for final link
                #     axisNext=np.array([0,0,0]) #no axis to draw for last link
                nextLink = self.links[nextLinkIndex]
                pNext = erb.HT2translation(nextLink.preHT) #this defines the position for the local of the axis for next link
                axisNext = erb.HT2rotationMatrix(nextLink.preHT) @ dictJointType2Axis[nextLink.jointType] 

                if len(link.visualization.graphicsData) == 0:
                    graphicsList += self.GetLinkGraphicsData(i, pThis, pNext, axis0, axisNext, link.visualization)

            if len(link.visualization.graphicsData) != 0:
                graphicsList += link.visualization.graphicsData

            #add transformed graphicsData of tool to link graphics
            if (i==len(self.links)-1 and #tool
                self.tool.visualization.graphicsData != []):
                pOff = erb.HT2translation(self.tool.HT)
                Aoff = erb.HT2rotationMatrix(self.tool.HT)
                for data in self.tool.visualization.graphicsData:
                    graphicsList += [exudyn.graphics.Move(data, pOff, Aoff)] 

            
            #++++++++++++++++++++++++
            #now add body for link:
            dictLink = mbs.CreateRigidBody(referencePosition=erb.HT2translation(Tcurrent),  
                                           referenceRotationMatrix=erb.HT2rotationMatrix(Tcurrent),  
                                           inertia=inertiaLink,  
                                           gravity=self.gravity,  
                                           nodeType=rigidBodyNodeType,  
                                           graphicsDataList=graphicsList,  
                                           returnDict=True)  
            nodeList+=[dictLink['nodeNumber']  ]
            bodyList+=[dictLink['bodyNumber']]
        
            #++++++++++++++++++++++++
            #add markers and joints
            mLink1 = mbs.AddMarker(eii.MarkerBodyRigid(bodyNumber=dictLink['bodyNumber'], localPosition=pThis))

            if i == 0:
                lastMarkerRotation = erb.HT2rotationMatrix(link.preHT)@lastMarkerRotation #is rotationMarkerBase
                mLink0LastBody = baseMarker
            else:
                lastMarkerRotation = erb.HT2rotationMatrix(link.preHT)
                marker0Position = erb.HT2translation(link.preHT) #this is defined in the parent link!
                parentBody = bodyList[self.GetParentIndex(i)]
                mLink0LastBody = mbs.AddMarker(eii.MarkerBodyRigid(bodyNumber=parentBody, 
                                                               localPosition=marker0Position))

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
            rotationMarker0 = lastMarkerRotation
            rotationMarker1 = AthisT

            jointLink = mbs.AddObject(eii.GenericJoint(markerNumbers=[marker0, marker1],
                                                    constrainedAxes=constrainedAxes,
                                                    rotationMarker0=rotationMarker0,
                                                    rotationMarker1=rotationMarker1,
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
                    nGeneric=mbs.AddNode(eii.NodeGenericData(initialCoordinates=[0], 
                                                         numberOfDataCoordinates=1)) #for infinite rotations
                    #torsional spring-damper allows control of rotation
                    objectSD = mbs.AddObject(eii.TorsionalSpringDamper(markerNumbers=[marker0, marker1],
                                                        nodeNumber=nGeneric,
                                                        rotationMarker0=rotationMarker0,
                                                        rotationMarker1=rotationMarker1,                                            
                                                        stiffness=PDcontrol[0],
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
                                                                axisMarker0 = jointAxis,
                                                                stiffness=PDcontrol[0],
                                                                damping=PDcontrol[1],
                                                                visualization=eii.VLinearSpringDamper(show=False)
                                                                ))
                    
            springDamperList += [objectSD]
            
            #markerList0 += [mlink1next]
            # if i < len(self.links)-1: #not suitable for kinematicTree
            #     lastMarkerRotation = erb.HT2rotationMatrix(self.links[i+1].preHT) #needed for modified DH parameters
            
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
             'baseObject':baseObject}
        return d
    
        
    

    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    #**classFunction: export kinematicTree
    def GetKinematicTree66(self):
        from exudyn.kinematicTree import KinematicTree66, Inertia2T66
        
        jointTypes = []
        transformations = []
        inertias = []
        n = len(self.links)
        for i in range(n):
            link = self.links[i]
            jointTypes += [link.jointType]
            
            preHT = link.preHT
            Amat = erb.HT2rotationMatrix(preHT) 
            vVec = erb.HT2translation(preHT)
            X=erb.RotationTranslation2T66Inverse(A=Amat, v=vVec)
            if np.linalg.norm(link.localHT - erb.HT0()) > 1e-15:
                raise ValueError('GetKinematicTree66(): not implemented for links with localHT != HT0()')
            
            transformations += [X] #defines transformation to joint in parent link
            J = erb.RigidBodyInertia(mass=link.mass, inertiaTensor=link.inertia) #link.inertia around COM
            J = J.Translated(link.COM)
            inertias += [Inertia2T66(J)]
        
        KT=KinematicTree66(listOfJointTypes=jointTypes, 
                           listOfTransformations=transformations, 
                           listOfInertias=inertias, 
                           gravity=self.gravity)
        return KT
    
    #**classFunction: create link GraphicsData (list) for link i; internally used in CreateRedundantCoordinateMBS(...); linkVisualization contains visualization dict of link
    def GetLinkGraphicsData(self, i, p0, p1, axis0, axis1, linkVisualization):
        
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

            #first cylinder should be drawn at base:
            # if i == 0: #draw full cylinder for first joint
            #     h0 = wJ

            if i != 0:
                graphicsList += [exudyn.graphics.Cylinder(pAxis=p0, vAxis=-h0*np.array(axis0), 
                                                      radius=r, color=color)]
            
            graphicsList += [exudyn.graphics.Cylinder(pAxis=p1, vAxis= h1*np.array(axis1), 
                                                      radius=r, color=color)]

            #draw body as cylinder:
            if ebu.NormL2(ebu.VSub(p1,p0)) > 1e-15:
                graphicsList += [exudyn.graphics.Cylinder(pAxis=p1, vAxis=ebu.VSub(p0,p1), 
                                                      radius=0.5*wL, color=color)]
        
        return graphicsList

    
    #**classFunction: build robot structre from dictionary; this is a DEPRECATED function, which is used in older models; DO NOT USE
    def BuildFromDictionary(self, robotDict):
        global buildFromDictionaryWarned
        if not buildFromDictionaryWarned:
            buildFromDictionaryWarned = True
            exudyn.Print("WARNING: function BuildFromDictionary in class Robot is DEPRECATED; DO NOT USE")

        if 'base' in robotDict:
            self.base.HT = np.array(robotDict['base']['HT'])
        if 'tool' in robotDict:
            self.tool.HT = np.array(robotDict['tool']['HT'])
        if 'gravity' in robotDict:
            self.gravity = np.array(robotDict['gravity'])
        if 'referenceConfiguration' in robotDict:
            self.referenceConfiguration = np.array(robotDict['referenceConfiguration'])
        
        warnedModDH = False
        for i in range(len(robotDict['links'])):
            link = robotDict['links'][i]
            # qRef = 0
            # if len(robotDict['referenceConfiguration']) > i:
            #     qRef = robotDict['referenceConfiguration'][i]
            
            if 'stdDH' in link:
                localHT = StdDH2HT(link['stdDH']) #using theta=0
            #modified DH parameters are re-interpreted as standard DH parameters, which does not always work / needs additional adjustments!
            elif 'modDHcraig' in link: #according to Craig (1986) and modified DH parameters in Corke's toolbox (2017)
                if not warnedModDH:
                    warnedModDH = True
                    exudyn.Print('WARNING: BuildFromDictionary untested for modDH; missing transformation for inertia and COM?')
                [theta, d, a, alpha] = link['modDHcraig']
                a1 = 0 #if no succeeding link follows
                alpha1 = 0
                if i < len(robotDict['links'])-1: #there exists a next link, which we use as additional standard DH-parameters for this link
                    [theta1, d1, a1, alpha1] = link['modDHcraig']
                if i == 0: #put first two DH parameters to base
                    self.base.HT = self.base.HT @ erb.HTrotateX(alpha) @ erb.HTtranslate([a,0,0]) 
                
                #local HT re-interpreted as standard DH-parameters, as suggested by Corke 2017, page 219:
                localHT = erb.HTrotateZ(theta) @ erb.HTtranslate([0,0,d]) @ erb.HTtranslate([a1,0,0]) @ erb.HTrotateX(alpha1)
            else:
                raise ValueError('BuildFromDictionary in class Robot: only supports links with stdDH parameters')

            robotLink = RobotLink(mass=link['mass'], COM=link['COM'], inertia=link['inertia'], localHT=localHT)
            self.AddLink(robotLink)

        

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  DH-PARAMETERS  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#**function: compute homogeneous transformation matrix HT from standard DHparameters=[theta, d, a, alpha]
def StdDH2HT(DHparameters):
#    [theta, d, a, alpha] = DHparameters
#    return erb.HTrotateZ(theta) @ erb.HTtranslate([0,0,d]) @ erb.HTtranslate([a,0,0]) @ erb.HTrotateX(alpha)
    #optimized version:
    [theta, d, a, alpha] = DHparameters
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([[ct,-st*ca, st*sa, a*ct],
                     [st, ct*ca,-ct*sa, a*st],
                     [ 0, sa   , ca   , d   ],
                     [ 0, 0    , 0    , 1   ]])

#\mfour{\cos \theta_j &-\sin \theta_j \cos \alpha_j & \sin \theta_j \sin \alpha_j & a_j \cos \theta_j}
#                                                        {\sin \theta_j & \cos \theta_j \cos \alpha_j &-\cos \theta_j \sin \alpha_j & a_j \sin \theta_j}
#                                                        {0             & \sin \alpha_j               & \cos \alpha_j               & d_j }
#                                                        {0 & 0 & 0 & 1}
#Test (compared with Robotcs, Vision and Control book of P. Corke:
#exudyn.Print("std. DH =\n", DH2HT([0.5, 0.1, 0.2, np.pi/2]).round(4))

#**function: compute pre- and post- homogeneous transformation matrices from modified Denavit-Hartenberg DHparameters=[alpha, d, theta, r]; returns [HTpre, HTpost]; HTpre is transformation before axis rotation, HTpost includes axis rotation and everything hereafter; modified DH-Parameters according to Khalil and Kleinfinger, 1986
def ModDHKK2HT(DHparameters):
    [alpha, d, theta, r] = DHparameters
    return [erb.HTrotateX(alpha) @ erb.HTtranslate([d,0,0]) , erb.HTrotateZ(theta) @ erb.HTtranslate([0,0,r]) ] 

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: This function projects an angle in the range $[-min_{float}, +max_{float}]$ fo the range $[-\pi, +\pi]$
#**input:
#  q0: An angle either as scalar, list or array
#**output:
#  qProj: The angle projected into the range $[-\pi to \pi]$
#**author: Peter Manzl
def projectAngleToPMPi(q0): 
    if type(q0) == list: 
        q0 = np.array(q0) # cast to array for modulo to work
    q1 =  q0 % (2*np.pi) # in range 0 to 2*np.pi
    qProj = q1 - 2*np.pi*(q1 > np.pi)
    return qProj


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**class: This class can be used to solve the inverse kinematics problem using a multibody system 
#            by solving the static problem of a serial robot
#**author: Peter Manzl, Johannes Gerstmayr
#**notes: still under development; errors in orientations of solution may occure. proviedes mtehods to calculate inverse Kinematics 
class InverseKinematicsNumerical(): 
    # initialize system
    #**classFunction: initialize RigidBodyInertia with scalar mass, 3x3 inertiaTensor (w.r.t. reference point!!!) and center of mass com
    #**input:
    #  robot: robot class
    #  jointStiffness: the stiffness used for the robot's model joints
    #  useRenderer: when solving the inverse kinematics the renderer is used to show the starting/end 
    #               configuration of the robot using the graphics objects definded in the robot object
    #**author: Peter Manzl
    def __init__(self, robot, jointStiffness = 1e0, useRenderer=False, flagDebug=False, 
                 useAlternativeConstraints=False): 
        self.SC = exudyn.SystemContainer()
        self.mbsIK = self.SC.AddSystem()
        self.robot = robot
        self.nLinks = len(self.robot.links)
        self.useRenderer = useRenderer
        self.flagDebug = flagDebug
        self.epsRotationMatrix = 1e-14
        self.epsSolution = 1e-14
        self.useAlternativeConstraints = useAlternativeConstraints
        
        self.oGround = self.mbsIK.AddObject(eii.ObjectGround(referencePosition=erb.HT2translation(robot.GetBaseHT()), 
                                              #visualization=VObjectGround(graphicsData=graphicsBaseList)
                                                  ))
        
        self.baseMarker = self.mbsIK.AddMarker(eii.MarkerBodyRigid(bodyNumber=self.oGround, localPosition=[0,0,0]))
        self.mGroundEE = self.mbsIK.AddMarker(eii.MarkerBodyRigid(bodyNumber=self.oGround, localPosition=[0,0,0]))
        self.ToolHT = robot.tool.HT
        
        if eau.IsValidRealInt(jointStiffness): # build list from real
            jointStiffness = [jointStiffness] * self.nLinks 
        
        
        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # build kinemtic tree model of the robot
        self.robot.gravity = [0,0,0]    # no gravity needed
        # self.KinematicTree = self.robot.GetKinematicTree66() # not working for localHT (stdDH!)
        listOldControl = [None]*self.nLinks #store previous PDcontrol
        for i in range(self.nLinks):     
            listOldControl[i] = self.robot.links[i].PDcontrol 
            self.robot.links[i].PDcontrol = (jointStiffness[i],0)

        self.robotDict = self.robot.CreateKinematicTree(self.mbsIK)
        
        self.mTool = self.mbsIK.AddMarker(eii.MarkerKinematicTreeRigid(objectNumber=self.robotDict['objectKinematicTree'], linkNumber=self.nLinks-1, 
                                                                           localPosition=erb.HT2translation(self.robot.tool.HT)))
        
        self.sToolTrans = self.mbsIK.AddSensor(eii.SensorMarker(markerNumber=self.mTool, 
                                              outputVariableType=exudyn.OutputVariableType.Position, storeInternal=False))
        
        self.sToolRot = self.mbsIK.AddSensor(eii.SensorMarker(markerNumber=self.mTool, 
                                            outputVariableType=exudyn.OutputVariableType.RotationMatrix, storeInternal=False))

    
        if 1: # 
            self.constraintTool= self.mbsIK.AddObject(eii.GenericJoint(markerNumbers=[self.mGroundEE , self.mTool],
                                                   rotationMarker1=erb.HT2rotationMatrix(self.robot.tool.HT), 
                                                   alternativeConstraints = self.useAlternativeConstraints))

        else: 
            self.constraintTool= self.mbsIK.AddObject(eii.RigidBodySpringDamper(markerNumbers=[self.mGroundEE , self.mTool], 
                                                              stiffness=np.eye(6)*1e8 , damping = np.eye(6)*1e3, 
                                                              rotationMarker1=erb.HT2rotationMatrix(self.robot.tool.HT))) 

        #restore PDcontrol for robot!        
        for i in range(self.nLinks):     
            self.robot.links[i].PDcontrol = listOldControl[i]
        
        
        # set simulation settings for static solver 
        self.simulationSettings = exudyn.SimulationSettings()
        self.simulationSettings.solutionSettings.writeSolutionToFile = False
        self.simulationSettings.solutionSettings.binarySolutionFile = False
        self.simulationSettings.linearSolverSettings.ignoreSingularJacobian = True
        self.simulationSettings.displayComputationTime = False
        self.simulationSettings.displayStatistics = False
        
        self.simulationSettings.staticSolver.newton.maxIterations = 50 #original: 500
        self.simulationSettings.staticSolver.adaptiveStep = True
        self.simulationSettings.staticSolver.verboseMode = 0
        self.simulationSettings.displayGlobalTimers = 0
        #self.simulationSettings.staticSolver.stabilizerODE2term = 1e-1
        self.staticSolver = exudyn.MainSolverStatic()
        # sparse solver settings are faster for redundant mbs
        self.mbsIK.Assemble()   
        self.sysStateList = self.mbsIK.systemData.GetSystemState()

    def __def__(self):
        self.staticSolver.FinalizeSolver(self.mbsIK, self.simulationSettings)
        
    # debugging helper function 
    def createVector(p0, p01): 
        x = [p0[0], p0[0] + p01[0]]
        y = [p0[1], p0[1] + p01[1]]
        z = [p0[2], p0[2] + p01[2]]
        return x, y, z


    #**classFunction: Utility function to get current Homogeneous transformation of the robot to check inverse Kinematics solution
    # ** output: 
    #   T: 4x4 homogeneous Transformation matrix of the current TCP pose
    def GetCurrentRobotHT(self): 
        # self.robot.JointHT(q)[-1]  @ self.robot.tool.HT # proviedes same functionality as reading sensors...
        posFKine = self.mbsIK.GetSensorValues(self.sToolTrans) 
        RotFkine = self.mbsIK.GetSensorValues(self.sToolRot).reshape((3,3))
        T = erb.HomogeneousTransformation(RotFkine, posFKine) # global HT
        return T

    #**classFunction: 
    #**input:
    #  T1: 4x4 homogeneous transformation matrix representing the first Pose
    #  T2: 4x4 homogeneous transformation matrix representing the second Pose
    #  rotStep: the max. size of steps to take for the orientation
    #  minSteps: minimum number of substeps to interpolate
    #**output: 
    # T: a List of homogeneous Transformations for each step between
    #**author: Peter Manzl
    #**notes: still under development; interpolation may be changed to using logSE3
    def InterpolateHTs(self, T1, T2, rotStep=np.pi/16, minSteps = 1): 
        R1, t1 = T1[:3,:3], erb.HT2translation(T1)
        R2, t2 = T2[:3,:3], erb.HT2translation(T2)
        t12 = t2 - t1
        R12 = np.transpose(R1) @ R2
        rot12 = erb.RotationMatrix2RotationVector(R12)
        rotAng = np.linalg.norm(rot12, 2) # rotation vector is angle * normalized rotation axis 

        #DELETE, not needed any more due to improved RotationMatrix2RotationVector  
        # if this is the case, then two axes are flipped; 
        # this can happen in the generic joint with the static solver and correspond 
        # to a 180° rotation around the axis with entry 1 in the rotation matrix
        # if abs(np.trace(R12) + 1) <= self.epsRotationMatrix: 
        #     R12 = np.round(R12, 12)
        #     for i in range(3): 
        #         if abs(R12[i,i]-1) <= self.epsRotationMatrix: rot12[i] = 1
        #     rot12 *= np.pi
        #     rotAng = np.pi

        n = min(minSteps, 1+int(rotAng/rotStep)) # number of steps
        T = []
        for i in range(n): 
            roti = rot12*(i+1)/n
            Ri = R1 @ erb.RotationVector2RotationMatrix(roti)
            # Ri = Ri /np.linalg.det(Ri) # avoid
            ti = t1 + (t2-t1)*(i+1)/n 
            Ti = erb.HomogeneousTransformation(Ri, ti)    
            T += [Ti]
        T += [T2] # to satisfy the boundry condition
        return T
    
    #**classFunction: This Method can be used to solve the inverse kinematics problem by solving 
    #            the static problem of a serial robot using steps to interpolate between start and end position close to the function Solve. 
    #            This helps the function Solve() to find the correct solutions. 
    #**input:
    #  T: the 4x4 homogeneous transformation matrix representing the desired position and orientation of the Endeffector
    #  q0: The configuration (joint angles/positions) of the robot from which the numerical methods start so calculate the solution; q0=None indicates that the stored solution (from model or previous solution) shall be used for initialization
    #**output: [q, success]; q: The solution for the joint angles in which the robot's tool center point (TCP) reaches the desired homogeneous transformation matrix T; success=False indicates that all trials for inverse kinematics failed, leading to q=None
    # success: flag to indicate if method was successful
    #**author: Peter Manzl, Johannes Gerstmayr
    #**notes: still under development; errors in orientations of solution may occure. works similar to ikine\_LM function of the robotics toolbox from peter corke
    def SolveSafe(self, T, q0 = None):
        T0 = self.GetCurrentRobotHT()
        TInterp = self.InterpolateHTs(T0, T, rotStep=np.pi/3) # no steps in between needed!
        q = q0
        for Ti in TInterp:
            [q, success] = self.Solve(Ti, q)
            if not success: 
                if self.flagDebug: 
                    exudyn.Print('WARNING: InverseKinematics: SolveSafe failed to solve')
                break

        if success:
            TSol = self.GetCurrentRobotHT() # the forward kinematics after solving
        else:
            TSol = T0
    
        if ((TSol-T) >= self.epsSolution).any(): #*JG: 1e-12; try once again with even finer discretization ...
            if self.flagDebug:
                exudyn.Print('WARNING: InverseKinematics: SolveSafe refine')
                if success:
                    exudyn.Print(' at err = \n', (np.round((TSol-T), 10)) ) # round for better readability

            TInterp = self.InterpolateHTs(T0, T, rotStep = np.pi/20, minSteps=4) #*JG:2023-03-29: changed from TSol to T0
            q = q0
            for Ti in TInterp:
                [q, success] = self.Solve(Ti, q)
                if not success: 
                    break

            if success:
                TSol = self.GetCurrentRobotHT()
                if (np.abs(TSol-T) >= 1e-8).any(): 
                    if self.flagDebug: 
                        exudyn.Print('WARNING: InverseKinematics: SolveSafe refinement failed: err = ', (np.round((TSol-T), 10))) # round for better readability
                    success = False

        if not success:
            q = None

        return [q, success]
    
    #**classFunction: This Method can be used to solve the inverse kinematics problem by solving 
    #            the static problem of a serial robot using steps to interpolate between start and end position close to the function Solve. 
    #           T his helps the fucntion Solve to find the correct solutions. 
    #**input:
    #  T: the 4x4 homogeneous transformation matrix representing the desired position and orientation of the Endeffector
    #  q0: The configuration (joint angles/positions) of the robot from which the numerical methods start so calculate the solution; q0=None indicates that the stored solution (from model or previous solution) shall be used for initialization
    #**output: [q, success]; q: The solution for the joint angles in which the robot's tool center point (TCP) reaches the desired homogeneous transformation matrix T; success=False indicates that all trials for inverse kinematics failed, leading to q=None
    #**author: Peter Manzl, Johannes Gerstmayr
    #**notes: still under development; errors in orientations of solution may occure. works similar to ikine\_LM function of the robotics toolbox from peter corke
    def Solve(self, T, q0 = None): 
        # check type of T 
        T = np.array(T)
        if T.shape != (4,4) or round(np.linalg.det(T[0:3, 0:3]),10) != 1.0:  # check if is homogeneous TF
            raise ValueError('inverse Kinematics only possible for homogeneous transformations, represented by a 4x4 array with structure of [[R, t], [0,0,0,1]].')
        q = None
        if not(hasattr(q0, '__iter__')) and q0 == None: #replace with: q0 is None
            q0 = self.mbsIK.systemData.GetODE2Coordinates() # + (np.random.random(self.nLinks)-0.5)*0.1 # [0]*self.nLinks
        
        #always set q0 as zero-configuration for springs!
        q0 = projectAngleToPMPi(q0) 
        self.mbsIK.SetObjectParameter(self.robotDict['objectKinematicTree'], 'jointPositionOffsetVector', q0)

        
        self.mbsIK.systemData.SetODE2Coordinates(coordinates=q0, configuration=exudyn.ConfigurationType.Initial)
        R = erb.HT2rotationMatrix(T)
        trans = (erb.HT2translation(T))
        
        # set the position of the Ground to match the desired EE position (in global "ground" sytem)
        self.mbsIK.SetMarkerParameter(self.mGroundEE, 'localPosition', trans)
        # set the desired rotation 
        self.mbsIK.SetObjectParameter(self.constraintTool, 'rotationMarker0', R)
        
        try: 
            if self.useRenderer: 
                SC.renderer.Start()
                self.mbsIK.WaitForUserToContinue() #stop before simulating

            success = self.staticSolver.SolveSystem(self.mbsIK, self.simulationSettings)
            
            q = self.mbsIK.systemData.GetODE2Coordinates()
            q = projectAngleToPMPi(q) # solution of the inverse kinematics problem projected into -pi/pi range
            self.mbsIK.systemData.SetODE2Coordinates(coordinates=q, configuration=exudyn.ConfigurationType.Initial)
            
        except:
            if self.flagDebug: 
                exudyn.Print('WARNING: InverseKinematics: Solve: static solver failed')
            [q, success] = None, False
            
        if success:
            # read output from sensors to check if the solution of the inverse Kinematics was correct
            self.fKineSolved = self.GetCurrentRobotHT()
            if (np.abs(self.fKineSolved - T) <= self.epsSolution).all(): 
                success = True
            else: 
                success = False
                # forwards kinematics deviates from desired HT; happens when desired TCP is outside of the robot's working space
                # or when the orientation is not set correctly; can happen because of current implementation of generic joint 
                if self.flagDebug: 
                    exudyn.Print('\n'*1)
                    exudyn.Print('WARNING: InverseKinematics: solution incorrect:') 
                    p1 = erb.HT2translation(self.fKineSolved)
                    p2 = erb.HT2translation(T)
                    exudyn.Print('pos error: ', np.round(p1-p2, 17)) # if TF is in the workspace then the position works
                    # rotation may still be wrong
                    R1 = self.fKineSolved[0:3,0:3]
                    R2 = T[0:3,0:3]
                    rot1 = erb.RotationMatrix2RotXYZ(R1)
                    rot2 = erb.RotationMatrix2RotXYZ(R2) 
                    exudyn.Print('rot1 = {}, rot2 = {}'.format(rot1, rot2))
                    exudyn.Print('R1:\n{}, \nR2:\n{}'.format(R1, R2))
                    exudyn.Print('\n'*1)
                    # raise ValueError('no valid solution found for inverse kinematics')
        else:
            q = None

        if self.useRenderer:    
            self.simulationSettings.solutionSettings.solutionInformation = 'success = {}\nq={}'.format(success, np.round(q, 3))
            self.SC.renderer.DoIdleTasks() # stop before closing
            self.SC.renderer.Stop() # close rendering window! 

        return [q, success]

