#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  example for inverse kinematics of serial manipulator UR5
#
# Author:   Peter Manzel; Johannes Gerstmayr
# Date:     2019-07-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics
from exudyn.rigidBodyUtilities import *
from exudyn.robotics import *
import numpy as np

from exudyn.kinematicTree import KinematicTree66, JointTransformMotionSubspace
# from exudyn.robotics import InverseKinematicsNumerical

jointWidth=0.1
jointRadius=0.06
linkWidth=0.1
graphicsBaseList = [graphics.Brick([0,0,-0.15], [0.12,0.12,0.1], graphics.color.grey)]
graphicsBaseList +=[graphics.Cylinder([0,0,-jointWidth], [0,0,jointWidth], linkWidth*0.5, graphics.colorList[0])] #belongs to first body

from exudyn.robotics.models import ManipulatorPuma560, ManipulatorPANDA, ManipulatorUR5
# robotDef = ManipulatorPuma560()
robotDef = ManipulatorUR5()
# robotDef = ManipulatorPANDA()
flagStdDH = True
# LinkList2Robot() # todo: build robot using the utility function

toolGraphics = [graphics.Basis(length=0.3*0)]
robot2 = Robot(gravity=[0,0,-9.81],
              base = RobotBase(HT=exu.HT(), visualization=VRobotBase(graphicsData=graphicsBaseList)),
              tool = RobotTool(HT=exu.HT(translation=[0,0,0.1*0]), visualization=VRobotTool(graphicsData=toolGraphics)),
              referenceConfiguration = []) #referenceConfiguration created with 0s automatically

nLinks = len(robotDef['links'])
# save read DH-Parameters into variables for convenience
a,d,alpha,rz, dx = [0]*nLinks, [0]*nLinks, [0]*nLinks, [0]*nLinks, [0]*nLinks
for cnt, link in enumerate(robotDef['links']):
    robot2.AddLink(RobotLink(mass=link['mass'], 
                               COM=link['COM'], 
                               inertia=link['inertia'], 
                                localHT=StdDH2HT(link['stdDH']),
                                # localHT=StdDH2HT(link['modKKDH']),
                               PDcontrol=(10, 1),
                               visualization=VRobotLink(linkColor=graphics.colorList[cnt], showCOM=False, showMBSjoint=True)
                               ))                                                
    # save read DH-Parameters into variables for convenience
    if flagStdDH: # std-dh  
        # stdH = [theta, d, a, alpha] with Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)                                                                                          
        d[cnt], a[cnt], alpha[cnt] = link['stdDH'][1],link['stdDH'][2], link['stdDH'][3]
    else: 
        # modDH = [alpha, dx, theta, rz] as used by Khali: Rx(alpha) * Tx(d) * Rz(theta) * Tz(r)
        # Important note:  d(khali)=a(corke)  and r(khali)=d(corke)  
        alpha[cnt], dx[cnt], rz[cnt],  = link['stdDH'][0],link['stdDH'][1], link['stdDH'][3]

myIkine = InverseKinematicsNumerical(robot2, useRenderer=True)
#the target pose is the one of a known configuration, so it is reachable; the solver starts from another one
qTarget = [0.2, -0.9, -0.8, -0.7, 0.6, 1.4]
T3 = robot2.JointHT(qTarget)[-1] @ robot2.tool.HT #the tool frame in the target configuration
q0 = [0, -np.pi/4, -np.pi/4, -np.pi/4, np.pi/4, np.pi/2]
[q, success] = myIkine.Solve(T3, q0=q0)
print('success = {}\nq = {} rad'.format(success, None if q is None else np.round(q, 3)))
