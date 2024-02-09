
.. _examples-inversekinematicsnumericalexample:

************************************
InverseKinematicsNumericalExample.py
************************************

You can view and download this file on Github: `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_

.. code-block:: python
   :linenos:

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
   from exudyn.utilities import *
   from exudyn.rigidBodyUtilities import *
   from exudyn.graphicsDataUtilities import *
   from exudyn.robotics import *
   import numpy as np
   
   from exudyn.kinematicTree import KinematicTree66, JointTransformMotionSubspace
   # from exudyn.robotics import InverseKinematicsNumerical
   
   jointWidth=0.1
   jointRadius=0.06
   linkWidth=0.1
   graphicsBaseList = [GraphicsDataOrthoCubePoint([0,0,-0.15], [0.12,0.12,0.1], color4grey)]
   graphicsBaseList +=[GraphicsDataCylinder([0,0,-jointWidth], [0,0,jointWidth], linkWidth*0.5, color4list[0])] #belongs to first body
   
   from exudyn.robotics.models import ManipulatorPuma560, ManipulatorPANDA, ManipulatorUR5
   # robotDef = ManipulatorPuma560()
   robotDef = ManipulatorUR5()
   # robotDef = ManipulatorPANDA()
   flagStdDH = True
   # LinkList2Robot() # todo: build robot using the utility function
   
   toolGraphics = [GraphicsDataBasis(length=0.3*0)]
   robot2 = Robot(gravity=[0,0,-9.81],
                 base = RobotBase(HT=HTtranslate([0,0,0]), visualization=VRobotBase(graphicsData=graphicsBaseList)),
                 tool = RobotTool(HT=HTtranslate([0,0,0.1*0]), visualization=VRobotTool(graphicsData=toolGraphics)),
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
                                  visualization=VRobotLink(linkColor=color4list[cnt], showCOM=False, showMBSjoint=True)
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
   ## test 
   if 1:  # tests close to zero-configuration
       R = RotXYZ2RotationMatrix(np.array([np.pi,0.2*0,np.pi/8*0 ]))
       t = [0.4526, -0.1488, 0.5275] 
       T2 = [[1,0,0,0.3], [0,1,0,0.3], [0,0,1,0.3], [0,0,0,1]]
       T3 = HomogeneousTransformation(R, t)
       sol = myIkine.Solve(T3, q0 = [0, -np.pi/4, -np.pi/4, -np.pi/4, np.pi/4, np.pi/2])
       print('success = {}\nq = {} rad'.format(sol[1], np.round(sol[0], 3)))
       

