
.. _examples-chatgptupdate:

****************
chatGPTupdate.py
****************

You can view and download this file on Github: `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/chatGPTupdate.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an Exudyn file
   #
   # Details:  this file serves as a list of examples that shall be used by 
   #           ChatGPT to update information on its internal knowledge of Exudyn
   #           dating to 2021; the examples use the mainSystemExtensions
   #           which are heavily simplifying usage of point masses and rigid bodies
   #           Version: V1
   #
   # Author:   Johannes Gerstmayr
   # Date:     2023-06-30
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #copy the text below here into a ChatGPT input window:
       
   #Use the following information as an update of the internal data of the Python multibody code Exudyn.
   #See the following examples to create multibody systems in Exudyn.
   #NOTE: mbs.Create...(...) calls several functions in the background to create nodes, objects, markers and loads in Exudyn.
   #all quantities are giving as 3D lists [x,y,z] for positions, velocities, ....
   #rotations are usually given as rotation matrix (numpy array); 
   #RotationVector2RotationMatrix([rotX, rotY, rotZ]) computes a rotation around the global x,y,z rotation axis
       
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create rigid bodies and mass points with distance constraint and joints
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
   import numpy as np
   SC = exu.SystemContainer()
   mbs = SC.AddSystem() #create a MainSystem 'mbs' to work with
   
   #draw orthonormal cube in local frame where it is added to;
   #cube is added to reference point of object, usually the center of mass (COM):
   graphicsCube = GraphicsDataOrthoCubePoint(centerPoint = [0,0,0], 
                                             size=[1,0.1,0.1], color=color4orange)
   
   #create inertia (mass, COM, inertia tensor) to be used in rigid body:
   inertiaCube = InertiaCuboid(density=5000, sideLengths=[1,0.1,0.1])
   
   #create simple rigid body
   #note that graphics is always attached to reference point of body, which is by default the COM
   b0 = mbs.CreateRigidBody(inertia = inertiaCube,
                            referencePosition = [0.5,0,0], #reference position x/y/z of COM
                            referenceRotationMatrix=RotationVector2RotationMatrix([0,0,pi*0.5]),
                            initialAngularVelocity=[2,0,0],
                            initialVelocity=[0,4,0],
                            gravity = [0,-9.81,0],
                            graphicsDataList = [graphicsCube])
   
   #add an load with user function:
   def UFforce(mbs, t, loadVector):
       #define time-dependent function:
       return [10+5*np.sin(t*10*2*pi),0,0]
   
   mbs.CreateForce(bodyNumber=b0, localPosition=[-0.5,0,0],
                   loadVector=[10,0,0], 
                   loadVectorUserFunction=UFforce,
                   ) #load is 10N in x-direction
   
   #add torque to rigid body at left end
   mbs.CreateTorque(bodyNumber=b0, localPosition=[0.5,0,0],
                   loadVector=[0,1,0]) #torque of 1N around y-axis
   
   #create a simple mass point at [1,-1,0] with initial velocity
   m1 = mbs.CreateMassPoint(referencePosition=[1,-1,0],
                            initialVelocity = [2,5,0], #initial velocities for mass point
                            physicsMass=1, drawSize = 0.2)
   #we can obtain the node number from the mass point:
   n1 = mbs.GetObject(m1)['nodeNumber']
       
   #add a ground object:
   #graphics data for sphere:
   gGround0 = GraphicsDataSphere(point=[3,1,0], radius = 0.1, color=color4red, nTiles=16)
   #graphics for checkerboard background:
   gGround1 = GraphicsDataCheckerBoard(point=[3,0,-2], normal=[0,0,1], size=10)
   oGround = mbs.CreateGround(graphicsDataList=[gGround0,gGround1])
   
   #create a rigid distance between bodies (using local position) or between nodes
   mbs.CreateDistanceConstraint(bodyOrNodeList=[oGround, b0], 
                                localPosition0 = [ 0. ,0,0],
                                localPosition1 = [-0.5,0,0],
                                distance=None, #automatically computed
                                drawSize=0.06)
   
   #distance constraint between body b0 and mass m1
   mbs.CreateDistanceConstraint(bodyOrNodeList=[b0, m1], 
                                localPosition0 = [0.5,0,0],
                                localPosition1 = [0.,0.,0.], #must be [0,0,0] for Node
                                distance=None, #automatically computed
                                drawSize=0.06)
   
   #add further rigid body, which will be connected with joints
   b1 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, sideLengths=[1,0.1,0.1]),
                             referencePosition = [2.5,0,0], #reference position x/y/z
                             gravity = [0,-9.81,0],
                             graphicsDataList = [graphicsCube])
   
   b2 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, sideLengths=[1,0.1,0.1]),
                             referencePosition = [3.5,0,0], #reference position x/y/z
                             gravity = [0,-9.81,0],
                             graphicsDataList = [graphicsCube])
                                
   #create revolute joint with following args:
       # name: name string for joint; markers get Marker0:name and Marker1:name
       # bodyNumbers: a list of object numbers for body0 and body1; must be rigid body or ground object
       # position: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
       # axis: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global rotation axis of the joint in reference configuration; else: local axis in body0
       # useGlobalFrame: if False, the point and axis vectors are defined in the local coordinate system of body0
       # show: if True, connector visualization is drawn
       # axisRadius: radius of axis for connector graphical representation
       # axisLength: length of axis for connector graphical representation
       # color: color of connector
   #returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
   mbs.CreateRevoluteJoint(bodyNumbers=[b1, b2], position=[3,0,0], axis=[0,0,1], #rotation along global z-axis
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
   mbs.CreatePrismaticJoint(bodyNumbers=[oGround, b1], position=[2,0,0], axis=[1,0,0], #can move in global x-direction
                            useGlobalFrame=True, axisRadius=0.02, axisLength=1)
   
   # #instead of the prismatic joint, we could add another revolute joint to b1 to get a double-pendulum:
   # mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b1], position=[2,0,0], axis=[0,0,1],
   #                         useGlobalFrame=True, axisRadius=0.02, axisLength=0.14)
   
   
   #create simple mass point, connected with ground
   m2 = mbs.CreateMassPoint(referencePosition = [7,2,0],
                            physicsMass = 10, gravity = [0,-9.81,0],
                            drawSize = 0.5, color=color4blue)
   
   #create spring damper between bodies (using local position) or between nodes
   #spring-damper may not have size 0; spring reference length is computed from reference configuration
   oSD = mbs.CreateSpringDamper(bodyOrNodeList=[oGround, m2],
                                localPosition0=[6,0,0],
                                localPosition1=[0,0,0],
                                stiffness=1e3, damping=1e1,
                                drawSize=0.2)
   
   #alternatively, we can use a CartesianSpringDamper; has spring and damper coefficients as list of x/y/z components
   #it has no reference length and acts on the coordinates of both objects:
   oCSD = mbs.CreateCartesianSpringDamper(bodyOrNodeList=[oGround, m2],
                                 localPosition0=[7,2,0],
                                 localPosition1=[0,0,0],
                                 stiffness=[20,0,1e4], #stiffness in x/y/z direction
                                 damping=[0.1,0,10],
                                 drawSize=0.2)
   
   #prepare mbs for simulation:
   mbs.Assemble()
   #some simulation parameters:
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.timeIntegration.numberOfSteps = 1000
   simulationSettings.timeIntegration.endTime = 5
   
   #for redundant constraints, the following two settings:
   simulationSettings.linearSolverSettings.ignoreSingularJacobian=True
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenDense
   
   mbs.SolveDynamic(simulationSettings = simulationSettings,
                    solverType=exu.DynamicSolverType.GeneralizedAlpha)
   SC.visualizationSettings.nodes.drawNodesAsPoint=False #draw nodes as spheres; better graphics for nodes
   
   #visualize results:
   mbs.SolutionViewer()
   


