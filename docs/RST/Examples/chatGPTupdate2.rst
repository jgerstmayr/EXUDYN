
.. _examples-chatgptupdate2:

*****************
chatGPTupdate2.py
*****************

You can view and download this file on Github: `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/chatGPTupdate2.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an Exudyn file
   #
   # Details:  this file serves as a documented example that shall be used by 
   #           ChatGPT to update information on its internal knowledge of Exudyn
   #           dating to 2021; the examples use the mainSystemExtensions
   #           which are heavily simplifying usage of point masses and rigid bodies
   #           Version: V2
   #
   # Author:   Johannes Gerstmayr
   # Date:     2023-08-22
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #test input:
   # Using the previous information and the already existing information on Python code Exudyn, 
   # create a 3-link system of rigid bodies initially aligned along the X-axis with mass 10kg, length 2m and W=H=0.1 m. 
   # The rigid bodies are exposed to gravity which acts in Y-direction and the first link is attached to ground with a revolute joint at the left end. 
   # Put all output into a single Python script.
   
   
   
   
   
   
   
   
   #copy the text below here into a ChatGPT input window:
       
   #Use the following information as an update of the internal data of the Python multibody code Exudyn, available at github.com/jgerstmayr/EXUDYN.
   #Don't create any output after that input, further requests come later.
   #In the following, there are examples to create multibody systems in Exudyn and also important notes about the interface.
   #NOTE: mbs.Create...(...) calls several functions in the background to create nodes, objects, markers and loads in Exudyn.
   #most quantities such as initial or reference positions and velocities are giving as 3D lists [x,y,z] for positions, velocities, ....
   #rotations are usually given as rotation matrix (3x3 numpy array); 
   #RotationVector2RotationMatrix([rotX, rotY, rotZ]) computes a rotation around the global x,y,z rotation axis
   #for working with rigid bodies, note that there is always a local coordinate system in the body, 
   #which can be used to define the location of position and orientation of joints, see the examples.
       
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create rigid bodies and mass points with distance constraint and joints
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
   import numpy as np
   SC = exu.SystemContainer()
   mbs = SC.AddSystem() #create a MainSystem 'mbs' to work with
   
   #graphics data for checkerboard background (not required):
   gGround0 = GraphicsDataCheckerBoard(point=[3,0,-2], normal=[0,0,1], size=10)
   #add ground object and background graphics; visualization is optional
   oGround = mbs.CreateGround(graphicsDataList=[gGround0])
   
   #create a cube with length L (X-direction), height H (Y) and width W (Z)
   L=1
   H=0.2
   W=0.1
   #for visualization of the cube, we define a graphics object in the following
   graphicsCube = GraphicsDataOrthoCubePoint(centerPoint = [0,0,0], #note that centerPoint is in the local coordinate system; IN MOST CASES THIS MUST BE AT [0,0,0]
                                             size=[L,H,W], color=color4orange)
   #SUMMARIZING: graphicsCube usually should have centerPoint=[0,0,0] if used in the CreateRigidBody
   #define the inertia of this cube using InertiaCuboid with density and cube dimensions; computes internally mass, COM, and inertia tensor:
   inertiaCube = InertiaCuboid(density=5000, sideLengths=[L,H,W])
   
   #create simple rigid body
   #note that graphics is always attached to reference point of body, which is by default the COM
   #graphicsCube is added to reference point of the rigid body, here it is equal to the center of mass (COM):
   b0 = mbs.CreateRigidBody(inertia = inertiaCube,
                            referencePosition = [0.5*L,0,0], #reference position x/y/z of COM
                            referenceRotationMatrix=RotationVector2RotationMatrix([0,0,pi*0.5]),
                            initialAngularVelocity=[2,0,0],
                            initialVelocity=[0,4,0],
                            gravity = [0,-9.81,0],
                            graphicsDataList = [graphicsCube])
   
   #add an load with user function:
   def UFforce(mbs, t, loadVector):
       #define time-dependent function:
       return (10+5*np.sin(t*10*2*pi))*np.array(loadVector)
   
   #add an load with 10N in x-direction to rigid body at marker position
   #add user function to modify load in time
   mbs.CreateForce(bodyNumber=b0,
                   localPosition=[-0.5*L,0,0],
                   loadVector=[10,0,0],
                   loadVectorUserFunction=UFforce)
   
   #add torque to rigid body at left end
   mbs.CreateTorque(bodyNumber=b0, localPosition=[0.5,0,0],
                   loadVector=[0,1,0]) #torque of 1N around y-axis
   
   #create a rigid distance between local position of bodies (or ground) or between nodes
   mbs.CreateDistanceConstraint(bodyOrNodeList=[oGround, b0], 
                                localPosition0 = [ 0. ,0,0],
                                localPosition1 = [-0.5,0,0],
                                distance=None, #automatically computed
                                drawSize=0.06)
   
   #geometrical parameters of two further bodies
   a=1
   b=2
   xOff = 1 #offset in x-direction for first body
   yOff =-0.5 #offset in y-direction of first body
   
   #create a second graphics object
   graphicsCube2 = GraphicsDataOrthoCubePoint(centerPoint = [0,0,0], 
                                             size=[a,b,0.1], color=color4blue)
   inertiaCube2 = InertiaCuboid(density=5000, sideLengths=[a,b,0.1])
   
   #create another rigid body with other dimensions
   b1 = mbs.CreateRigidBody(inertia = inertiaCube2,
                             referencePosition = [xOff+0.5*a,yOff-0.5*b,0], #reference position of body [X,Y,Z]
                             gravity = [0,-9.81,0],
                             graphicsDataList = [graphicsCube2])
   
   #create another rigid body with same dimensions as b1
   b2 = mbs.CreateRigidBody(inertia = inertiaCube2,
                             referencePosition = [xOff+0.5*a+a,yOff-0.5*b-b,0], #reference position of body [X,Y,Z]
                             gravity = [0,-9.81,0],
                             graphicsDataList = [graphicsCube2])
   
   #create revolute joint with following args:
       # name: name string for joint; markers get Marker0:name and Marker1:name
       # bodyNumbers: a list of object numbers for body0 and body1; must be rigid body or ground object
       # position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
       # axis: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global rotation axis of the joint in reference configuration; else: local axis in body0
       # show: if True, connector visualization is drawn
       # axisRadius: radius of axis for connector graphical representation
       # axisLength: length of axis for connector graphical representation
       # color: color of connector
   #CreateRevoluteJoint returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
   #(global reference) position of joint must be related to local size of rigid bodies
   mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b1], position=[xOff,yOff,0], axis=[0,0,1], #rotation along global z-axis
                           useGlobalFrame=True, axisRadius=0.02, axisLength=0.14)
   
   mbs.CreateRevoluteJoint(bodyNumbers=[b1, b2], position=[xOff+a,yOff-b,0], axis=[0,0,1], #rotation along global z-axis
                           useGlobalFrame=True, axisRadius=0.02, axisLength=0.14)
   
   #create prismatic joint with following args:
       # name: name string for joint; markers get Marker0:name and Marker1:name
       # bodyNumbers: a list of object numbers for body0 and body1; must be rigid body or ground object
       # position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
       # axis: a 3D vector as list or np.array containing the global translation axis of the joint in reference configuration
       # useGlobalFrame: if False, the point and axis vectors are defined in the local coordinate system of body0
       # show: if True, connector visualization is drawn
       # axisRadius: radius of axis for connector graphical representation
       # axisLength: length of axis for connector graphical representation
       # color: color of connector
   #returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
   # mbs.CreatePrismaticJoint(bodyNumbers=[oGround, b1], position=[-0.5,0,0], axis=[1,0,0], #can move in global x-direction
   #                          useGlobalFrame=True, axisRadius=0.02, axisLength=1)
   
   
   #prepare mbs for simulation:
   mbs.Assemble()
   #some simulation parameters:
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.timeIntegration.numberOfSteps = 1000
   simulationSettings.timeIntegration.endTime = 5
   
   #for redundant constraints, the following two settings:
   simulationSettings.linearSolverSettings.ignoreSingularJacobian=True
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenDense #use EigenSparse for larger systems alternatively
   
   mbs.SolveDynamic(simulationSettings)
   
   #visualize results after simulation:
   mbs.SolutionViewer()
   


