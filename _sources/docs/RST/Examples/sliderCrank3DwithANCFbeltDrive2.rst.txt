
.. _examples-slidercrank3dwithancfbeltdrive2:

**********************************
sliderCrank3DwithANCFbeltDrive2.py
**********************************

You can view and download this file on Github: `sliderCrank3DwithANCFbeltDrive2.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive2.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Create slider-crank mechanism with belt drive modeled with ANCF cable elements
   #
   # Authors: Martin Knapp and Lukas March; edited by Johannes Gerstmayr, in particular adapted to functionality of beams module and fixed initialization problems (2024-10-15)
   # Date: Created on Thu May  19 12:22:52 2020
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   # Notes: PROJECT Exercise:  Drive System + Crank System; VU Industrielle Mechatronik 2 - Robotics and Simulation
   #        The performance could be significantly improved, if GeneralContact would be used instead of ObjectContactFrictionCircleCable2D
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #
   #
   #AUTHORS: Martin Knapp & Lukas March
   #
   #
   #Copyright: This file is part of Exudyn. Exudyn is free software. 
   #You can redistribute it and/or modify it under the terms of the Exudyn license.
   #See 'LICENSE.txt' for more details.
   #"""
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Tested with EXUDYN Version 0.1.342.
   
   import sys
   import os
   
       
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   import sys
   import os
   import numpy as np
   import matplotlib.pyplot as plt
   import matplotlib.ticker as ticker
   
   
   import exudyn as exu 
   import numpy as np
   from exudyn.itemInterface import (MarkerNodeRotationCoordinate, ObjectConnectorCartesianSpringDamper, 
                              LoadTorqueVector, VObjectJointPrismatic2D, ObjectJointPrismatic2D, Torque, 
                              MassPoint2D, RigidBody2D, NodePoint2D, RevoluteJoint2D, CoordinateConstraint, 
                              ObjectGround, ObjectContactFrictionCircleCable2D, NodeGenericData, 
                              MarkerBodyCable2DShape, NodePoint2DSlope1, ObjectANCFCable2D, VObjectANCFCable2D, MarkerBodyPosition, 
                              VObjectJointRevolute2D, VObjectRigidBody2D, NodePointGround, MarkerNodePosition, 
                              MarkerNodeCoordinate, Force, SensorBody, NodeRigidBody2D, ObjectRigidBody2D, 
                              MarkerBodyRigid, ObjectJointRevolute2D, SensorLoad)
   from exudyn.utilities import AddRigidBody, RigidBodyInertia, ObjectConnectorCoordinate, InertiaCuboid
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.beams import *
   
   #import visHelper
   #visHelper.init()
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   PLOTS_PATH = "plots/"
   fontSize = 20
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # Change plot axis
   def set_axis(plt, equal = False):
       ax=plt.gca() #get current axes
       ax.grid(True , 'major', 'both')
       ax.xaxis.set_major_locator(ticker.MaxNLocator(10))
       ax.yaxis.set_major_locator(ticker.MaxNLocator(10))
       if equal:
           ax.set_aspect('equal')
       plt.legend() #show labels as legend
   
   def plotOmegaDisk0():
       ang_vel = np.loadtxt("angular_velocity_disk0.txt", comments='#', delimiter=',')
       
       fig = plt.figure(figsize=[13,5])
       plt.plot(ang_vel[:,0], ang_vel[:,3], 'r-', label='$\\omega_{disk0}$')
       set_axis(plt)
       ax=plt.gca()
       ax.set_xlabel('$t [s]$', fontsize=fontSize)
       ax.set_ylabel('$\\omega_{disk0} [rad/s]$', fontsize=fontSize)
       ax.grid(True, 'major', 'both')
       plt.legend()
       plt.tight_layout()
       plt.show()
       fig.savefig(PLOTS_PATH + 'angular_velocity_disk0.pdf', format='pdf')
   
   def plotOmegaDisk1():
       ang_vel = np.loadtxt("angular_velocity_disk1.txt", comments='#', delimiter=',')
       
       fig = plt.figure(figsize=[13,5])
       plt.plot(ang_vel[:,0], ang_vel[:,3], 'r-', label='$\\omega_{disk1}$')
       set_axis(plt)
       ax=plt.gca()
       ax.set_xlabel('$t [s]$', fontsize=fontSize)
       ax.set_ylabel('$\\omega_{disk1} [rad/s]$', fontsize=fontSize)
       ax.grid(True, 'major', 'both')
       plt.legend()
       plt.tight_layout()
       plt.show()
       fig.savefig(PLOTS_PATH + 'angular_velocity_disk1.pdf', format='pdf')
   
   def plotTorque():
       ang_vel = np.loadtxt("torque.txt", comments='#', delimiter=',')
       
       fig = plt.figure(figsize=[13,5])
       plt.plot(ang_vel[:,0], ang_vel[:,3], 'r-', label='$\\tau$')
       set_axis(plt)
       ax=plt.gca()
       ax.set_xlabel('$t [s]$', fontsize=fontSize)
       ax.set_ylabel('$\\tau [Nm]$', fontsize=fontSize)
       ax.grid(True, 'major', 'both')
       plt.legend()
       plt.tight_layout()
       plt.show()
       fig.savefig(PLOTS_PATH + 'torque.pdf', format='pdf')
   
   def plotCrankPos():
       ang_vel = np.loadtxt("crank_pos.txt", comments='#', delimiter=',')
       
       fig = plt.figure(figsize=[13,5])
       plt.plot(ang_vel[:,0], ang_vel[:,1], 'r-', label='$x_{Pos}$')
       plt.plot(ang_vel[:,0], ang_vel[:,2], 'b-', label='$y_{Pos}$')
       set_axis(plt)
       ax=plt.gca()
       ax.set_xlabel('$t [s]$', fontsize=fontSize)
       ax.set_ylabel('$x,y [m]$', fontsize=fontSize)
       ax.grid(True, 'major', 'both')
       plt.legend()
       plt.tight_layout()
       plt.show()
       fig.savefig(PLOTS_PATH + 'crank_pos.pdf', format='pdf')
   
   
   def vishelperInit():
       plt.close('all')
       if not os.path.isdir(PLOTS_PATH):
           os.mkdir(PLOTS_PATH)
       
   def visHelperPlot_all():
       plotOmegaDisk0()
       plotOmegaDisk1()
       plotTorque()
       plotCrankPos()
   
   #if __name__ is "__main__":
   #    init()
   #    plot_all()
   
   
   vishelperInit()
   
   
   
   print("Exudyn used:", exu.config.Version())
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #SYSTEM SETTINGS
   
   # 0 - belt-drive-system
   # 1 - crank system 2D
   # 2 - crank system 3D
   # 3 - belt-drive-system + crank system 2D
   # 4 - belt-drive-system + crank system 3D
   
   sys_set = 4
   
   useGraphics = True
   export_images = useGraphics
   PLOTS_PATH = "plots/"
   
   if export_images:
       if not os.path.isdir(PLOTS_PATH):
           os.mkdir(PLOTS_PATH)
   
   # belt-drive-system 
   enable_force = True          # Enable Preload Force in the belt
   enable_disk_friction = True  # When True the disks will have contact to the belt else the belt is free floating
   enable_controller = True     # If True the disk0 will get a torque regulated by a P-controller else a fixed torque
   nElements = 1*50             # Belt element count
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #geometry parameters
   L_0 = 0.5               #distance between the disks [m]
   L_A = 0.15              #length of crank [m]
   h_A = 0.025             #cross-section height and width of crank [m]
   L_B = 0.3               #length of connecting rod [m]
   h_B = 0.025             #cross-section height and width of connecting rod [m]
   r_0 = 0.05              #radius of disk0 [m]
   r_1 = 0.1               #radius of disk1 [m]
   b_a0 = 0.1              #section-length of crank [m]
   b_a1 = 0.05             #section-length of crank [m]
   b_a2 = 0.05             #section-length of crank [m]
   h_S = 0.1               #cross-section height and width of slider [m]
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #belt parameters
   d_belt = 0.01           #cross-section height and width [m]
   csa_belt = d_belt**2    #cross-section area [m^2]
   E_belt = 2e9            #E modulus [N/m^2]
   rho_belt = 1e3          #density [kg/m^3]
   F_p = 1e4               #preload force[N]
   
   contactStiffness=1e5*10    #contactStiffness between the belt and the disks ->
                           #holds the belt around disks
   contactDamping=200      #damping between the belt and the disks
   mu = 0.9                #friction coefficent between the belt and the disks
   velPenalty = 5e2        #frictionVelocityPenalty between tangential velocities
                           #of the belt against the disks
   controller_set_vel = 100 # Desired angular velocity in rad/s of the disk0: 100 rad/s = ~ 955 U/min
   controller_p = 30.0      # P-Factor of controller
   
   MassPerLength_belt = rho_belt * csa_belt
   bendStiffness_belt = E_belt * (d_belt**4 / 12)
   #Hook: F = E * A * epsilon
   axialStiffness_belt = E_belt * csa_belt
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #mass parameters
   m_disk0 = 0.5           #mass of disk0 [kg]
   m_disk1 = 1             #mass of disk1 [kg]
   m_crank = 0.5           #mass of crank [kg]
   density_conrod = 1000   #Density of the conrod [kg/m^3]
   m_slider = 0.2          #mass of slider [kg]
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #inertia parameters
   #inertia disks
   J_disk0 = 0.5 * m_disk0 * r_0**2                          #inertia disk0 [Nm^2]
   J_disk1 = 0.5 * m_disk1 * r_1**2                          #inertia disk1 [Nm^2]
   
   #inertia crank
   J_xx_crank = 5e-3                                         #inertia xx [Nm^2]
   J_yy_crank = 7e-3                                         #inertia yy [Nm^2]
   J_zz_crank = 2.5e-3                                       #inertia zz [Nm^2]
   inertia_crank = RigidBodyInertia(mass=m_crank, 
                                    inertiaTensor=np.diag([J_xx_crank,
                                                           J_yy_crank,
                                                           J_zz_crank]))
   #inertia conrod
   inertia_conrod = InertiaCuboid(density=density_conrod, 
                                  sideLengths = [L_B,h_B,h_B])
   
   #inertia slider 
   #Dummy inertia because MassPoint will result in System Jacobian not invertible!
   inertia_slider = InertiaCuboid(density=(m_slider/h_S**3),
                                  sideLengths = [h_S]*3)
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #bearing parameters
   kBearing = 50e3                 #stiffness of bearing [N/m]
   dBearing =  1e3                 #damping of bearing [N/ms]
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #load parameters
   M = 0.1                             #torque [Nm]
   g = [0,-9.81,0]                     #acceleration of earth [m/s^2]
   load_crank = [0,-9.81*m_crank,0]    #gravity [N]
   load_conrod = [0,-9.81*inertia_conrod.mass,0]  #gravity [N]
   load_slider = [0,-9.81*m_slider,0]  #gravity [N]
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()     
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #BELT DRIVE SYSTEM
   if sys_set == 0 or sys_set > 2:
   
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #grounds
       
       #ground-node and ground-marker in center of disk0
       nG_disk0 = mbs.AddNode(NodePointGround(referenceCoordinates = [L_0,0,0])) 
       mG_disk0 = mbs.AddMarker(MarkerNodePosition(nodeNumber = nG_disk0))
       
       #ground-node and ground-marker in center of disk1
       nG_disk1 = mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0])) 
       mG_disk1 = mbs.AddMarker(MarkerNodePosition(nodeNumber = nG_disk1))
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #disk0
       
       #initial coordinates disk0 
       u0_disk0 = [L_0,0,0]   
       #NodeRigidBody2D on center of gravity disk0 
       nRB2D_disk0 = mbs.AddNode(NodeRigidBody2D(referenceCoordinates = [0,0,0],               
                          initialCoordinates = u0_disk0,                      
                          initialVelocities = [0,0,0])) 
       #visualization of disk0
       bodyVis_disk0 = VObjectRigidBody2D(graphicsData=[{'type':'Circle',
                                                         'color':[0,0,0,1],
                                                         'radius':r_0, 
                                                         'position':[0,0,0],
                                                         'normal':[0,0,1]},
                                                        {'type':'Line',
                                                         'color':[1,0,0,1], 
                                                         'data':[0,0,0,r_0,0,0]}]) 
       #ObjectRigidBody2D disk0
       oRB2D_disk0 = mbs.AddObject(ObjectRigidBody2D(nodeNumber = nRB2D_disk0, 
                                                     physicsMass=m_disk0, 
                                                     physicsInertia=J_disk0, 
                                                     visualization = bodyVis_disk0))   
       #MarkerBodyRigid on center of disk0
       mNP_disk0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB2D_disk0, 
                                                 localPosition=[0,0,0]))    
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #disk1
       
       #initial coordinates disk1  
       u0_disk1 = [0,0,0]     
       #NodeRigidBody2D on center of disk1
       nRB2D_disk1 = mbs.AddNode(NodeRigidBody2D(referenceCoordinates = [0,0,0],               
                          initialCoordinates = u0_disk1,                      
                          initialVelocities = [0,0,0])) 
       #visualization of disk1
       bodyVis_disk1 = VObjectRigidBody2D(graphicsData=[{'type':'Circle',
                                                         'color':[0,0,0,1],
                                                         'radius':r_1,
                                                         'position':[0,0,0],
                                                         'normal':[0,0,1]},
                                                        {'type':'Line',
                                                         'color':[1,0,0,1], 
                                                         'data':[0,0,0,r_1,0,0]}])   
       #ObjectRigidBody2D disk1
       oRB2D_disk1 = mbs.AddObject(ObjectRigidBody2D(nodeNumber = nRB2D_disk1, 
                                                     physicsMass=m_disk1, 
                                                     physicsInertia=J_disk1, 
                                                     visualization = bodyVis_disk1))   
       #MarkerBodyRigid on center of disk1
       mNP_disk1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB2D_disk1, 
                                                 localPosition=[0,0,0]))   
        
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #joints
       
       #visualization of joint0 in the center of disk0
       jointVis_disk0 = VObjectJointRevolute2D(show=True, drawSize=0.02, 
                                               color=[0,0,0,1])   
       #ObjectJointRevolute2D between ground and disk0
       oJR2D_disk0 = mbs.AddObject(ObjectJointRevolute2D(markerNumbers = [mG_disk0,mNP_disk0], 
                                                         visualization = jointVis_disk0))  
       #visualization of joint1 in the center of disk1    
       jointVis_disk1 = VObjectJointRevolute2D(show=True, drawSize=0.02, color=[0,0,0,1])      
       #ObjectJointRevolute2D between ground and disk1
       oJR2D_disk1 = mbs.AddObject(ObjectJointRevolute2D(markerNumbers = [mG_disk1,mNP_disk1], 
                                                         visualization = jointVis_disk1))    
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #function to get length of the belt
       def belt_get_lengths(L_0, r_l, r_r):
           alpha = np.arcsin((r_l-r_r)/L_0)            #angle of the arc length
           b_belt = L_0*np.cos(alpha)                  #branch between the disks
           al_dr_belt = r_r*(np.pi-2*alpha)            #arc length disk0
           al_dl_belt = r_l*(np.pi+2*alpha)            #arc length disk1 
           len_belt = 2*b_belt+al_dr_belt+al_dl_belt   #belt length
           return [alpha, b_belt, al_dl_belt, al_dr_belt, len_belt]
       
           
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #belt cable elements
       [alpha, b_belt, al_dl_belt, al_dr_belt, len_belt] = belt_get_lengths(L_0, r_1, r_0)
       #calculate belt length with preload force:
       epsilon_belt = -F_p / axialStiffness_belt
       len_belt_p = len_belt * (1.0 + epsilon_belt)
       
       print("Belt length: ", len_belt, " and after preload force: ", len_belt_p)
       
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #new functionality with beams module:
       circleList = [[[L_0,0],r_0,'R'],
                     [[0,0],r_1,'R'],
                     [[L_0,0],r_0,'R'],
                     [[0,0],r_1,'R'],
                     ]
   
       reevingDict = CreateReevingCurve(circleList, drawingLinesPerCircle = 64,
                                        removeLastLine=True,
                                        numberOfANCFnodes=nElements, graphicsNodeSize= 0.01)
       
       del circleList[-1] #remove circles not needed for contact/visualization
       del circleList[-1] #remove circles not needed for contact/visualization
       
       gList=[]
       if False: #visualize reeving curve, without simulation
           gList = reevingDict['graphicsDataLines'] + reevingDict['graphicsDataCircles']
           
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #create ANCF elements:
       
       # dimZ = b #z.dimension
       hDraw = 0.002
       cableTemplate = ObjectANCFCable2D(
                               physicsBendingStiffness=bendStiffness_belt*0.1,
                               physicsMassPerLength=MassPerLength_belt, 
                               physicsAxialStiffness=axialStiffness_belt, 
                               useReducedOrderIntegration=2,  
                               physicsReferenceAxialStrain=epsilon_belt,
                               physicsBendingDamping = 0.005*bendStiffness_belt,
                               physicsAxialDamping = 0.02*axialStiffness_belt,
                               visualization=VObjectANCFCable2D(drawHeight=hDraw),
                               )
       
       ancf = PointsAndSlopes2ANCFCable2D(mbs, reevingDict['ancfPointsSlopes'], reevingDict['elementLengths'], 
                                          cableTemplate, 
                                          #massProportionalLoad=gVec,
                                          firstNodeIsLastNode=True, graphicsSizeConstraints=0.01)
   
       nNP2DS = ancf[0]
       oANCFC2D_b = ancf[1]
   
   
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #contact friction belt
       
       if enable_disk_friction:
           
           nCableGenData_disk0 = []
           contact_disk0 = []
           nCableGenData_disk1 = []
           contact_disk1 = []
           mCableShape = []
           
           for i in range(0,len(oANCFC2D_b)):
               nCS = 8
               initGap = 0.001 #positive gap gives fast initial settling of contact
               #NodeGenericData disk0
               nCableGenData_disk0.append(mbs.AddNode(NodeGenericData(initialCoordinates = [initGap]*nCS+[-2]*nCS+[0]*nCS, 
                                                                      numberOfDataCoordinates=3*nCS)))
               #NodeGenericData disk1
               nCableGenData_disk1.append(mbs.AddNode(NodeGenericData(initialCoordinates = [initGap]*nCS+[-2]*nCS+[0]*nCS, 
                                                                      numberOfDataCoordinates=3*nCS)))
               #MarkerBodyCable2DShape on cable
               mCableShape.append(mbs.AddMarker(MarkerBodyCable2DShape(bodyNumber=oANCFC2D_b[i], 
                                                                       numberOfSegments=nCS)))
               #contact friction to disk0
               contact_disk0.append(mbs.AddObject(ObjectContactFrictionCircleCable2D(markerNumbers=[mNP_disk0,mCableShape[-1]], 
                                                                                     nodeNumber=nCableGenData_disk0[-1], 
                                                                                     circleRadius=r_0, 
                                                                                     contactStiffness=contactStiffness, 
                                                                                     contactDamping=contactDamping, 
                                                                                     numberOfContactSegments=nCS, 
                                                                                     frictionCoefficient=mu,
                                                                                     frictionVelocityPenalty=velPenalty)))
               #contact friction to disk1
               contact_disk1.append(mbs.AddObject(ObjectContactFrictionCircleCable2D(markerNumbers=[mNP_disk1, mCableShape[-1]], 
                                                                                     nodeNumber=nCableGenData_disk1[-1], 
                                                                                     circleRadius=r_1,
                                                                                     contactStiffness=contactStiffness, 
                                                                                     contactDamping=contactDamping, 
                                                                                     numberOfContactSegments=nCS, 
                                                                                     frictionCoefficient=mu,
                                                                                     frictionVelocityPenalty=velPenalty)))
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       # Velocity controller
                   
       s_disk0 = mbs.AddSensor(SensorBody(bodyNumber=oRB2D_disk0, writeToFile=True, 
                                          fileName="angular_velocity_disk0.txt",
                                          outputVariableType=exu.OutputVariableType.AngularVelocity))
       
       def p_controller(mbs, t, loadVector):
           vel = mbs.GetSensorValues(s_disk0)[2]
           torque = controller_p * (controller_set_vel - vel)
           return [0,0,torque]
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #torque on disk0    
       if enable_controller:
           l_Torquedisk0 = mbs.AddLoad(LoadTorqueVector(markerNumber=mNP_disk0,
                                        loadVectorUserFunction=p_controller))
           
       else:
           l_Torquedisk0 = mbs.AddLoad(LoadTorqueVector(markerNumber=mNP_disk0,loadVector=[0,0,M]))
       
       s_disk1 = mbs.AddSensor(SensorBody(bodyNumber=oRB2D_disk1, writeToFile=True, 
                                      fileName="angular_velocity_disk1.txt",
                                      outputVariableType=exu.OutputVariableType.AngularVelocity))
       s_load = mbs.AddSensor(SensorLoad(loadNumber=l_Torquedisk0, writeToFile=True, 
                                      fileName="torque.txt"))
   
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #CRANK DRIVE SYSTEM 2D
   if sys_set == 1 or sys_set == 3:
           
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #ground
       nPG = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) 
       oG = mbs.AddObject(ObjectGround(referencePosition=[0,0,0])) 
       mBPG = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oG, localPosition=[0,0,0])) 
       mNCG = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nPG, coordinate=0))
   
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #crank
       
       #visualization of crank
       graphics_crank = graphics.RigidLink(p0=[-0.5*L_A,0,-h_A/2],
                                              p1=[0.5*L_A ,0,-h_A/2], 
                                              axis0=[0,0,1], axis1=[0,0,1], 
                                              radius=[0.5*h_A,0.5*h_A],
                                              thickness=h_A, width=[h_A,h_A], 
                                              color=[0.8,0.8,0.8,1.],nTiles=16)
       #node on center of gravity of crank
       nRB2D_crank = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[-L_A*0.5,0,0], 
                                                 initialVelocities=[0,0,0]))
       #RigidBody2D crank
       oRB2D_crank = mbs.AddObject(RigidBody2D(physicsMass=m_crank, 
                                           physicsInertia=J_zz_crank,
                                           nodeNumber=nRB2D_crank,
                                           visualization=VObjectRigidBody2D(graphicsData=[graphics_crank])))
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #connecting rod
       
       #visualization of connecting rod
       graphics_conrod = graphics.RigidLink(p0=[-0.5*L_B,0,h_B/2],
                                               p1=[0.5*L_B,0,h_B/2], 
                                               axis0=[0,0,1], axis1=[0,0,1],
                                               radius=[0.5*h_B,0.5*h_B],
                                               thickness=h_B, width=[h_B,h_B], 
                                               color=[0.7,0.7,0.7,1],nTiles=36)
       #node on center of gravity of connecting rod
       nRB2D_conrod = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[-(L_A+L_B*0.5),0,0], 
                                                  initialVelocities=[0,0,0]))
       #RigidBody2D connecting rod
       oRB2D_conrod = mbs.AddObject(RigidBody2D(physicsMass=inertia_conrod.mass, 
                                         physicsInertia=inertia_conrod.inertiaTensor[1,1],
                                         nodeNumber=nRB2D_conrod,
                                         visualization=VObjectRigidBody2D(graphicsData= [graphics_conrod])))
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
       #slider
       
       #visualization of slider
       c=0.025 #dimension of slider
       graphics_slider = graphics.BrickXYZ(-c,-c,-c*2,c,c,0,
                                               color=[0.2,0.2,0.2,0.9])
       #node on center of gravity of slider
       nP2D_slider = mbs.AddNode(NodePoint2D(referenceCoordinates=[-(L_A+L_B),0]))
       #MassPoint2D slider
       oMP2D_slider = mbs.AddObject(MassPoint2D(physicsMass=m_slider, 
                                                nodeNumber=nP2D_slider,
                                                visualization=VObjectRigidBody2D(graphicsData= [graphics_slider])))
   
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #markers for joints
       mBPLeft_crank = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRB2D_crank, 
                                                        localPosition=[-L_A*0.5,0.,0.])) 
       mBPRight_crank = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRB2D_crank, 
                                                         localPosition=[L_A*0.5,0.,0.])) 
       mBPLeft_conrod = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRB2D_conrod, 
                                                         localPosition=[-L_B*0.5,0.,0.])) 
       mBPRight_conrod = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRB2D_conrod, 
                                                          localPosition=[L_B*0.5,0.,0.])) 
       mBP_slider = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oMP2D_slider, 
                                                     localPosition=[ 0.,0.,0.]))
   
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #joints
       oRJ2D_ground_crank = mbs.AddObject(RevoluteJoint2D(markerNumbers=[mBPG,mBPRight_crank]))
       oRJ2D_crank_conrod = mbs.AddObject(RevoluteJoint2D(markerNumbers=[mBPLeft_crank,mBPRight_conrod]))
       oRJ2D_slider = mbs.AddObject(RevoluteJoint2D(markerNumbers=[mBP_slider,mBPLeft_conrod]))
         
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++                                             
       #markers for node constraints
       mNC_Y_slider = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nP2D_slider, coordinate=1)) #y-coordinate is constrained
   
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #coordinate constraints for slider (free motion in x-direction)
       oCC_ground_slider = mbs.AddObject(CoordinateConstraint(markerNumbers=[mNCG, mNC_Y_slider]))
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #markers for load
       mBR_crank_torque = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRB2D_crank, localPosition=[L_A/2,0.,0.])) 
       mBR_crank_gravity = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRB2D_crank, localPosition=[0.,0.,0.]))
       mBR_conrod_gravity = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRB2D_conrod, localPosition=[0.,0.,0.]))
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #loads and driving forces
       
        #gravity of crank       
       lC_crank_gravity = mbs.AddLoad(Force(markerNumber = mBR_crank_gravity, 
                                            loadVector = load_crank))  
       #gravity of conrod      
       lC_conrod_gravity = mbs.AddLoad(Force(markerNumber = mBR_conrod_gravity, 
                                             loadVector = load_conrod))  
       #gravity of slider    
       lC_slider_gravity = mbs.AddLoad(Force(markerNumber = mBP_slider, 
                                             loadVector = load_slider))  
       
       if sys_set == 1:
           #torque at crank
           lC_crank_torque = mbs.AddLoad(Torque(markerNumber = mBR_crank_torque, 
                                                loadVector = [0, 0, M]))   
           
       if sys_set == 3:
           #ConnectorCoordinate - crank gets torque of disk1
           mNC_disk1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRB2D_disk1,coordinate=2))
           mNC_crank = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRB2D_crank,coordinate=2))
           oCC_disk1_crank = mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mNC_disk1,mNC_crank],
                                                                     velocityLevel=True))
                   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #CRANK DRIVE SYSTEM 3D
   if sys_set == 2 or sys_set == 4:
           
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #ground
       oG_Left = mbs.AddObject(ObjectGround())
       oG_Right = mbs.AddObject(ObjectGround())
       oG_slider = mbs.AddObject(ObjectGround())
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #crank
       graphics_crank_1 = graphics.RigidLink(p0=[0,0,0],p1=[0,0,-b_a0], axis1=[0,0,1], radius=[h_A/2,h_A/1.3], 
                                          thickness = h_A, width=[0,h_A/2], color=[0.8,0.8,0.8,1])
       graphics_crank_2 = graphics.RigidLink(p0=[0,0,0],p1=[-L_A,0,0], radius=[h_A/2,h_A/2], 
                                          thickness = h_A, color=[0.8,0.8,0.8,1])
       graphics_crank_3 = graphics.RigidLink(p0=[-L_A,0,0],p1=[-L_A,0,b_a1], radius=[h_A/2,h_A/2], 
                                          thickness = h_A, color=[0.8,0.8,0.8,1])
       graphics_crank_4 = graphics.RigidLink(p0=[-L_A,0,b_a1],p1=[0,0,b_a1], radius=[h_A/2,h_A/2], 
                                          thickness = h_A, color=[0.8,0.8,0.8,1])
       graphics_crank_5 = graphics.RigidLink(p0=[0,0,b_a1],p1=[0,0,b_a1+b_a2],axis1=[0,0,1], radius=[h_A/2,h_A/1.3], 
                                          thickness = h_A, width=[0,h_A/2], color=[0.8,0.8,0.8,1])
   
       dictCrank = mbs.CreateRigidBody(
                     inertia=inertia_crank,
                     referencePosition=[0, 0, b_a0],
                     gravity=g,
                     graphicsDataList=[graphics_crank_1, graphics_crank_2, graphics_crank_3, graphics_crank_4, graphics_crank_5],
                     returnDict=True)
       [nRG_crank, oRB_crank] = [dictCrank['nodeNumber'], dictCrank['bodyNumber']]
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #connecting rod
       graphics_conrod = graphics.RigidLink(p0=[L_B/2, 0, 0], p1=[-L_B/2, 0, 0], axis0=[0, 0, 1], axis1=[0, 0, 1], 
                                            radius=[h_B/1.5, h_B/2], thickness=h_B, width=[h_B, h_B], color=[0.5, 0.5, 0.5, 1])
       dictConrod = mbs.CreateRigidBody(
                     inertia=inertia_conrod,
                     referencePosition=[-L_A - L_B/2, 0, b_a0 + b_a1/2],
                     gravity=g,
                     graphicsDataList=[graphics_conrod],
                     returnDict=True)
       [nRG_conrod, oRB_conrod] = [dictConrod['nodeNumber'], dictConrod['bodyNumber']]
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #slider
       d = 0.07
       graphics_slider = graphics.BrickXYZ(-d/2, -d/2, -d - h_B/2, d/2, d/2, -h_B/2, 
                                           color=[0.2, 0.2, 0.2, 0.9])
       dictSlider = mbs.CreateRigidBody(
                      inertia=inertia_slider,
                      initialAngularVelocity=[0, 0, 0],
                      referencePosition=[-(L_A + L_B), 0, b_a0 + b_a1/2],
                      gravity=g,
                      graphicsDataList=[graphics_slider],
                      returnDict=True)
       [nRB_slider, oRB_slider] = [dictSlider['nodeNumber'], dictSlider['bodyNumber']]
   
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #marker for joints
       mG_Left = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oG_Left, 
                                               localPosition=[0,0,0]))
       mG_Right = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oG_Right, 
                                                localPosition=[0,0,b_a0+b_a1+b_a2]))
       mG_slider = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oG_slider, 
                                                 localPosition=[-(L_A+L_B),0,b_a0+b_a1/2]))
       mBR_crank_Left = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRB_crank, 
                                                      localPosition=[0,0,-b_a0]))
       mBR_crank_Right = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRB_crank, 
                                                       localPosition=[0,0,b_a1+b_a2]))
       mBR_crank_conrod = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRB_crank, 
                                                        localPosition=[-L_A,0,b_a1/2]))
       mBR_conrod_crank = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRB_conrod, 
                                                        localPosition=[L_B/2,0,0]))
       mBR_conrod_slider = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRB_conrod, 
                                                         localPosition=[-L_B/2,0,0]))
       mBR_slider_conrod = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRB_slider, 
                                                         localPosition=[0,0,0]))
       mBR_slider = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRB_slider, 
                                                  localPosition=[0,0,0]))
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++q
       #joints
       oGJ_crank_Left = mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers=[mG_Left, mBR_crank_Left], 
                                                                           stiffness=[kBearing]*3, damping=[dBearing]*3))
       oGJ_crank_Right = mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers=[mG_Right, mBR_crank_Right], 
                                                                            stiffness=[kBearing]*3, damping=[dBearing]*3))
       oRJ2D_crank_conrod = mbs.AddObject(RevoluteJoint2D(markerNumbers=[mBR_crank_conrod,mBR_conrod_crank],
                                                          visualization=VObjectJointRevolute2D(drawSize=0.05)))
       oRJ2D_conrod_slider = mbs.AddObject(RevoluteJoint2D(markerNumbers=[mBR_conrod_slider,mBR_slider_conrod],
                                                           visualization=VObjectJointRevolute2D(drawSize=0.05)))
       oJP2D_slider = mbs.AddObject(ObjectJointPrismatic2D(markerNumbers=[mG_slider,mBR_slider], 
                                                           axisMarker0 = [1.,0.,0.],
                                                           normalMarker1 = [0.,1.,0.],
                                                           visualization=VObjectJointPrismatic2D(drawSize=0.01)))
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #load and driving forces
       
       if sys_set == 2:
           #markers for load
           mBR_crank_torque = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRB_crank, 
                                                            localPosition=[0.,0.,b_a1+b_a2])) 
           #driving forces
           lC_crank_torque = mbs.AddLoad(Torque(markerNumber = mBR_crank_torque, 
                                                loadVector = [0, 0, M/2]))  
           
       if sys_set == 4:
           #ConnectorCoordinate - crank gets torque of disk1
           #markers for Connector
           mNC_disk1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRB2D_disk1,coordinate=2))
           # disk1 is a NodeRigidBody2D -> coordinates = [q_0,q_1,psi] -> Rotation psi
           mNC_crank = mbs.AddMarker(MarkerNodeRotationCoordinate(nodeNumber=nRG_crank,rotationCoordinate=2))
           # crank is a 3D Node -> Rotation around z-axis
           #Connector Coordinate
           oCC_disk1_crank = mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mNC_disk1,mNC_crank],
                                                                     velocityLevel=True))
           
       s_crank = mbs.AddSensor(SensorBody(bodyNumber=oRB_crank, writeToFile=True, 
                                      fileName="crank_pos.txt", localPosition=[0,0,-b_a0],
                                      outputVariableType=exu.OutputVariableType.Position))
           
      
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++            
   
   mbs.Assemble()
   print(mbs)
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #simulation
   
   tEnd = 1              #end time of the simulation
   stepSize = 2.5e-5
   
   sims = exu.SimulationSettings()               
   sims.timeIntegration.generalizedAlpha.spectralRadius=0.8
   sims.pauseAfterEachStep = False     
   sims.displayStatistics = True             
   sims.displayComputationTime = True             
   sims.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   sims.timeIntegration.endTime = tEnd
   sims.timeIntegration.newton.absoluteTolerance = 1e-7
   sims.timeIntegration.newton.relativeTolerance = 1e-7
   sims.timeIntegration.newton.useModifiedNewton = True
   sims.timeIntegration.discontinuous.iterationTolerance = 0.1
   sims.timeIntegration.discontinuous.maxIterations = 3
   sims.solutionSettings.sensorsWritePeriod = 0.001
   sims.solutionSettings.solutionWritePeriod = stepSize*4*5
   sims.parallel.numberOfThreads = 1
   sims.timeIntegration.verboseMode = 1
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #visualizaion
   SC.visualizationSettings.general.circleTiling=128
   dSize = 0.02
   SC.visualizationSettings.nodes.defaultSize = dSize*0.25
   SC.visualizationSettings.markers.defaultSize = dSize*0.25
   SC.visualizationSettings.bodies.defaultSize = [dSize, dSize, dSize]
   SC.visualizationSettings.connectors.defaultSize = dSize
   SC.visualizationSettings.nodes.show=True
   SC.visualizationSettings.loads.show=False
   SC.visualizationSettings.loads.drawSimplified = False
   SC.visualizationSettings.loads.defaultSize = 0.1
   SC.visualizationSettings.markers.show=True
   SC.visualizationSettings.sensors.show=True
   SC.visualizationSettings.general.autoFitScene=False
   SC.visualizationSettings.openGL.initialCenterPoint = [0,0,0]
   SC.visualizationSettings.openGL.initialZoom = 0.5
   SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.ForceLocal
   
   if sys_set == 0:
       rot_alpha=0
       rot_beta=0
       rot_gamma=0
   elif sys_set > 0:
       rot_alpha=-0.6
       rot_beta=0.6
       rot_gamma=0.37
   
   R_x = np.array([[ 1, 0, 0],
                   [ 0, np.cos(rot_alpha), -np.sin(rot_alpha)],
                   [ 0, np.sin(rot_alpha), np.cos(rot_alpha)]])
   R_y = np.array([[ np.cos(rot_beta), 0, np.sin(rot_beta)],
                   [ 0, 1, 0],
                   [ -np.sin(rot_beta), 0, np.cos(rot_beta)]])
   R_z = np.array([[ np.cos(rot_gamma), -np.sin(rot_gamma), 0],
                   [ np.sin(rot_gamma), np.cos(rot_gamma), 0],
                   [ 0, 0, 1]])
   IMR = np.dot(R_x,R_y)
   IMR = np.dot(IMR,R_z)
   SC.visualizationSettings.openGL.initialModelRotation = [[IMR[0,0],IMR[0,1],IMR[0,2]],
                                                           [IMR[1,0],IMR[1,1],IMR[1,2]],   
                                                           [IMR[2,0],IMR[2,1],IMR[2,2]]]
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Rendering
   SC.renderer.Start()                 #start graphics visualization
   SC.renderer.DoIdleTasks()         #wait for pressing SPACE bar to continue
   # mbs.SolveStatic(sims)
   mbs.SolveDynamic(sims)
   SC.renderer.DoIdleTasks()    #wait for pressing 'Q' to quit
   SC.renderer.Stop()                  #safely close rendering window!
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   if export_images:
       visHelperPlot_all()


