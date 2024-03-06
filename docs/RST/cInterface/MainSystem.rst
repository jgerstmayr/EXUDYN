
**********
MainSystem
**********




This is the class which defines a (multibody) system. The MainSystem shall only be created by \ ``SC.AddSystem()``\ , not with \ ``exu.MainSystem()``\ , as the latter one would not be linked to a SystemContainer. In some cases, you may use SC.Append(mbs). In C++, there is a MainSystem (the part which links to Python) and a System (computational part). For that reason, the name is MainSystem on the Python side, but it is often just called 'system'. For compatibility, it is recommended to denote the variable holding this system as mbs, the multibody dynamics system. It can be created, visualized and computed. Use the following functions for system manipulation.

.. code-block:: python
   
   import exudyn as exu
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()

\ The class **MainSystem** has the following **functions and structures**:

* | **Assemble**\ (): 
  | assemble items (nodes, bodies, markers, loads, ...) of multibody system; Calls CheckSystemIntegrity(...), AssembleCoordinates(), AssembleLTGLists(), AssembleInitializeSystemCoordinates(), and AssembleSystemInitialize()
* | **AssembleCoordinates**\ (): 
  | assemble coordinates: assign computational coordinates to nodes and constraints (algebraic variables)
* | **AssembleLTGLists**\ (): 
  | build \ :ref:`LTG <LTG>`\  coordinate lists for objects (used to build global ODE2RHS, MassMatrix, etc. vectors and matrices) and store special object lists (body, connector, constraint, ...)
* | **AssembleInitializeSystemCoordinates**\ (): 
  | initialize all system-wide coordinates based on initial values given in nodes
* | **AssembleSystemInitialize**\ (): 
  | initialize some system data, e.g., generalContact objects (searchTree, etc.)
* | **Reset**\ (): 
  | reset all lists of items (nodes, bodies, markers, loads, ...) and temporary vectors; deallocate memory
* | **GetSystemContainer**\ (): 
  | return the systemContainer where the mainSystem (mbs) was created
* | **WaitForUserToContinue**\ (\ *printMessage*\  = True): 
  | interrupt further computation until user input --> 'pause' function; this command runs a loop in the background to have active response of the render window, e.g., to open the visualization dialog or use the right-mouse-button; behaves similar as SC.WaitForRenderEngineStopFlagthis()
* | **SendRedrawSignal**\ (): 
  | this function is used to send a signal to the renderer that the scene shall be redrawn because the visualization state has been updated
* | **GetRenderEngineStopFlag**\ (): 
  | get the current stop simulation flag; True=user wants to stop simulation
* | **SetRenderEngineStopFlag**\ (\ *stopFlag*\ ): 
  | set the current stop simulation flag; set to False, in order to continue a previously user-interrupted simulation
* | **ActivateRendering**\ (\ *flag*\  = True): 
  | activate (flag=True) or deactivate (flag=False) rendering for this system
* | **SetPreStepUserFunction**\ (\ *value*\ ): 
  | Sets a user function PreStepUserFunction(mbs, t) executed at beginning of every computation step; in normal case return True; return False to stop simulation after current step; set to 0 (integer) in order to erase user function. Note that the time returned is already the end of the step, which allows to compute forces consistently with trapezoidal integrators; for higher order Runge-Kutta methods, step time will be available only in object-user functions.
  | *Example*:

  .. code-block:: python

     def PreStepUserFunction(mbs, t):
         print(mbs.systemData.NumberOfNodes())
         if(t>1): 
             return False 
         return True 
     mbs.SetPreStepUserFunction(PreStepUserFunction)

* | **GetPreStepUserFunction**\ (\ *asDict*\  = False): 
  | Returns the preStepUserFunction.
* | **SetPostStepUserFunction**\ (\ *value*\ ): 
  | Sets a user function PostStepUserFunction(mbs, t) executed at beginning of every computation step; in normal case return True; return False to stop simulation after current step; set to 0 (integer) in order to erase user function.
  | *Example*:

  .. code-block:: python

     def PostStepUserFunction(mbs, t):
         print(mbs.systemData.NumberOfNodes())
         if(t>1): 
             return False 
         return True 
     mbs.SetPostStepUserFunction(PostStepUserFunction)

* | **GetPostStepUserFunction**\ (\ *asDict*\  = False): 
  | Returns the postStepUserFunction.
* | **SetPostNewtonUserFunction**\ (\ *value*\ ): 
  | Sets a user function PostNewtonUserFunction(mbs, t) executed after successful Newton iteration in implicit or static solvers and after step update of explicit solvers, but BEFORE PostNewton functions are called by the solver; function returns list [discontinuousError, recommendedStepSize], containing a error of the PostNewtonStep, which is compared to [solver].discontinuous.iterationTolerance. The recommendedStepSize shall be negative, if no recommendation is given, 0 in order to enforce minimum step size or a specific value to which the current step size will be reduced and the step will be repeated; use this function, e.g., to reduce step size after impact or change of data variables; set to 0 (integer) in order to erase user function. Similar described by Flores and Ambrosio, https://doi.org/10.1007/s11044-010-9209-8
  | *Example*:

  .. code-block:: python

     def PostNewtonUserFunction(mbs, t):
         if(t>1): 
             return [0, 1e-6] 
         return [0,0] 
     mbs.SetPostNewtonUserFunction(PostNewtonUserFunction)

* | **GetPostNewtonUserFunction**\ (\ *asDict*\  = False): 
  | Returns the postNewtonUserFunction.
* | **AddGeneralContact**\ (): 
  | add a new general contact, used to enable efficient contact computation between objects (nodes or markers)
* | **GetGeneralContact**\ (\ *generalContactNumber*\ ): 
  | get read/write access to GeneralContact with index generalContactNumber stored in mbs; Examples shows how to access the GeneralContact object added with last AddGeneralContact() command:
  | *Example*:

  .. code-block:: python

     gc=mbs.GetGeneralContact(mbs.NumberOfGeneralContacts()-1)

* | **DeleteGeneralContact**\ (\ *generalContactNumber*\ ): 
  | delete GeneralContact with index generalContactNumber in mbs; other general contacts are resorted (index changes!)
* | **NumberOfGeneralContacts**\ (): 
  | Return number of GeneralContact objects in mbs
* | **GetAvailableFactoryItems**\ (): 
  | get all available items to be added (nodes, objects, etc.); this is useful in particular in case of additional user elements to check if they are available; the available items are returned as dictionary, containing lists of strings for Node, Object, etc.
* | **GetDictionary**\ (): 
  | [UNDER DEVELOPMENT]: return the dictionary of the system data (todo: and state), e.g., to copy the system or for pickling
* | **SetDictionary**\ (\ *systemDict*\ ): 
  | [UNDER DEVELOPMENT]: set system data (todo: and state) from given dictionary; used for pickling
* | **\_\_repr\_\_()**\ : 
  | return the representation of the system, which can be, e.g., printed
  | *Example*:

  .. code-block:: python

     print(mbs)

* | **systemIsConsistent**:
  | this flag is used by solvers to decide, whether the system is in a solvable state; this flag is set to False as long as Assemble() has not been called; any modification to the system, such as Add...(), Modify...(), etc. will set the flag to False again; this flag can be modified (set to True), if a change of e.g.~an object (change of stiffness) or load (change of force) keeps the system consistent, but would normally lead to systemIsConsistent=False
* | **interactiveMode**:
  | set this flag to True in order to invoke a Assemble() command in every system modification, e.g. AddNode, AddObject, ModifyNode, ...; this helps that the system can be visualized in interactive mode.
* | **variables**:
  | this dictionary may be used by the user to store model-specific data, in order to avoid global Python variables in complex models; mbs.variables["myvar"] = 42 
* | **sys**:
  | this dictionary is used by exudyn Python libraries, e.g., solvers, to avoid global Python variables 
* | **solverSignalJacobianUpdate**:
  | this flag is used by solvers to decide, whether the jacobian should be updated; at beginning of simulation and after jacobian computation, this flag is set automatically to False; use this flag to indicate system changes, e.g. during time integration  
* | **systemData**:
  | Access to SystemData structure; enables access to number of nodes, objects, ... and to (current, initial, reference, ...) state variables (ODE2, AE, Data,...)



.. _sec-mainsystem-pythonextensionscreate:


MainSystem extensions (create)
==============================

This section represents extensions to MainSystem, which are direct calls to Python functions; the 'create' extensions to simplify the creation of multibody systems, such as CreateMassPoint(...); these extensions allow a more intuitive interaction with the MainSystem class, see the following example. For activation, import \ ``exudyn.mainSystemExtensions``\  or \ ``exudyn.utilities``\ 

.. code-block:: python
   :linenos:
   
   import exudyn as exu           
   from exudyn.utilities import * 
   #alternative: import exudyn.mainSystemExtensions
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   #
   #create rigid body
   b1=mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, sideLengths=[0.1,0.1,1]),
                          referencePosition = [1,0,0], 
                          gravity = [0,0,-9.81])


.. _sec-mainsystemextensions-createground:

Function: CreateGround
^^^^^^^^^^^^^^^^^^^^^^
`CreateGround <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L134>`__\ (\ ``name = ''``\ , \ ``referencePosition = [0.,0.,0.]``\ , \ ``referenceRotationMatrix = np.eye(3)``\ , \ ``graphicsDataList = []``\ , \ ``graphicsDataUserFunction = 0``\ , \ ``show = True``\ )

- | \ *function description*\ :
  | helper function to create a ground object, using arguments of ObjectGround; this function is mainly added for consistency with other mainSystemExtensions
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateGround.
- | \ *input*\ :
  | \ ``name``\ : name string for object
  | \ ``referencePosition``\ : reference coordinates for point node (always a 3D vector, no matter if 2D or 3D mass)
  | \ ``referenceRotationMatrix``\ : reference rotation matrix for rigid body node (always 3D matrix, no matter if 2D or 3D body)
  | \ ``graphicsDataList``\ : list of GraphicsData for optional ground visualization
  | \ ``graphicsDataUserFunction``\ : a user function graphicsDataUserFunction(mbs, itemNumber)->BodyGraphicsData (list of GraphicsData), which can be used to draw user-defined graphics; this is much slower than regular GraphicsData
  | \ ``color``\ : color of node
  | \ ``show``\ : True: show ground object;
- | \ *output*\ :
  | ObjectIndex; returns ground object index
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  ground=mbs.CreateGround(referencePosition = [2,0,0],
                          graphicsDataList = [GraphicsDataCheckerBoard(point=[0,0,0], normal=[0,1,0],size=4)])


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `basicTutorial2024.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/basicTutorial2024.py>`_\  (Ex), \ `beamTutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beamTutorial.py>`_\  (Ex), \ `cartesianSpringDamper.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamper.py>`_\  (Ex), \ `cartesianSpringDamperUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamperUserFunction.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `ConvexContactTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ConvexContactTest.py>`_\  (TM), \ `rigidBodySpringDamperIntrinsic.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodySpringDamperIntrinsic.py>`_\  (TM), \ `sliderCrank3Dbenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dbenchmark.py>`_\  (TM)



.. _sec-mainsystemextensions-createmasspoint:

Function: CreateMassPoint
^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateMassPoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L203>`__\ (\ ``name = ''``\ , \ ``referencePosition = [0.,0.,0.]``\ , \ ``initialDisplacement = [0.,0.,0.]``\ , \ ``initialVelocity = [0.,0.,0.]``\ , \ ``physicsMass = 0``\ , \ ``gravity = [0.,0.,0.]``\ , \ ``graphicsDataList = []``\ , \ ``drawSize = -1``\ , \ ``color = [-1.,-1.,-1.,-1.]``\ , \ ``show = True``\ , \ ``create2D = False``\ , \ ``returnDict = False``\ )

- | \ *function description*\ :
  | helper function to create 2D or 3D mass point object and node, using arguments as in NodePoint and MassPoint
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateMassPoint.
- | \ *input*\ :
  | \ ``name``\ : name string for object, node is 'Node:'+name
  | \ ``referencePosition``\ : reference coordinates for point node (always a 3D vector, no matter if 2D or 3D mass)
  | \ ``initialDisplacement``\ : initial displacements for point node (always a 3D vector, no matter if 2D or 3D mass)
  | \ ``initialVelocity``\ : initial velocities for point node (always a 3D vector, no matter if 2D or 3D mass)
  | \ ``physicsMass``\ : mass of mass point
  | \ ``gravity``\ : gravity vevtor applied (always a 3D vector, no matter if 2D or 3D mass)
  | \ ``graphicsDataList``\ : list of GraphicsData for optional mass visualization
  | \ ``drawSize``\ : general drawing size of node
  | \ ``color``\ : color of node
  | \ ``show``\ : True: if graphicsData list is empty, node is shown, otherwise body is shown; otherwise, nothing is shown
  | \ ``create2D``\ : if True, create NodePoint2D and MassPoint2D
  | \ ``returnDict``\ : if False, returns object index; if True, returns dict of all information on created object and node
- | \ *output*\ :
  | Union[dict, ObjectIndex]; returns mass point object index or dict with all data on request (if returnDict=True)
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0=mbs.CreateMassPoint(referencePosition = [0,0,0],
                         initialVelocity = [2,5,0],
                         physicsMass = 1, gravity = [0,-9.81,0],
                         drawSize = 0.5, color=color4blue)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `basicTutorial2024.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/basicTutorial2024.py>`_\  (Ex), \ `cartesianSpringDamper.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamper.py>`_\  (Ex), \ `cartesianSpringDamperUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamperUserFunction.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `springDamperTutorialNew.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/springDamperTutorialNew.py>`_\  (Ex), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM), \ `symbolicUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/symbolicUserFunctionTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createrigidbody:

Function: CreateRigidBody
^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateRigidBody <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L334>`__\ (\ ``name = ''``\ , \ ``referencePosition = [0.,0.,0.]``\ , \ ``referenceRotationMatrix = np.eye(3)``\ , \ ``initialVelocity = [0.,0.,0.]``\ , \ ``initialAngularVelocity = [0.,0.,0.]``\ , \ ``initialDisplacement = None``\ , \ ``initialRotationMatrix = None``\ , \ ``inertia = None``\ , \ ``gravity = [0.,0.,0.]``\ , \ ``nodeType = exudyn.NodeType.RotationEulerParameters``\ , \ ``graphicsDataList = []``\ , \ ``graphicsDataUserFunction = 0``\ , \ ``drawSize = -1``\ , \ ``color = [-1.,-1.,-1.,-1.]``\ , \ ``show = True``\ , \ ``create2D = False``\ , \ ``returnDict = False``\ )

- | \ *function description*\ :
  | helper function to create 3D (or 2D) rigid body object and node; all quantities are global (angular velocity, etc.)
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateRigidBody.
- | \ *input*\ :
  | \ ``name``\ : name string for object, node is 'Node:'+name
  | \ ``referencePosition``\ : reference position vector for rigid body node (always a 3D vector, no matter if 2D or 3D body)
  | \ ``referenceRotationMatrix``\ : reference rotation matrix for rigid body node (always 3D matrix, no matter if 2D or 3D body)
  | \ ``initialVelocity``\ : initial translational velocity vector for node (always a 3D vector, no matter if 2D or 3D body)
  | \ ``initialAngularVelocity``\ : initial angular velocity vector for node (always a 3D vector, no matter if 2D or 3D body)
  | \ ``initialDisplacement``\ : initial translational displacement vector for node (always a 3D vector, no matter if 2D or 3D body); these displacements are deviations from reference position, e.g. for a finite element node [None: unused]
  | \ ``initialRotationMatrix``\ : initial rotation provided as matrix (always a 3D matrix, no matter if 2D or 3D body); this rotation is superimposed to reference rotation [None: unused]
  | \ ``inertia``\ : an instance of class RigidBodyInertia, see rigidBodyUtilities; may also be from derived class (InertiaCuboid, InertiaMassPoint, InertiaCylinder, ...)
  | \ ``gravity``\ : gravity vevtor applied (always a 3D vector, no matter if 2D or 3D mass)
  | \ ``graphicsDataList``\ : list of GraphicsData for rigid body visualization; use graphicsDataUtilities function GraphicsData...(...)
  | \ ``graphicsDataUserFunction``\ : a user function graphicsDataUserFunction(mbs, itemNumber)->BodyGraphicsData (list of GraphicsData), which can be used to draw user-defined graphics; this is much slower than regular GraphicsData
  | \ ``drawSize``\ : general drawing size of node
  | \ ``color``\ : color of node
  | \ ``show``\ : True: if graphicsData list is empty, node is shown, otherwise body is shown; False: nothing is shown
  | \ ``create2D``\ : if True, create NodeRigidBody2D and ObjectRigidBody2D
  | \ ``returnDict``\ : if False, returns object index; if True, returns dict of all information on created object and node
- | \ *output*\ :
  | Union[dict, ObjectIndex]; returns rigid body object index (or dict with 'nodeNumber', 'objectNumber' and possibly 'loadNumber' and 'markerBodyMass' if returnDict=True)
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [1,0,0],
                           initialVelocity = [2,5,0],
                           initialAngularVelocity = [5,0.5,0.7],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1],
                                                                        color=color4red)])
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Ex), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM), \ `computeODE2AEeigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2AEeigenvaluesTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createspringdamper:

Function: CreateSpringDamper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateSpringDamper <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L562>`__\ (\ ``name = ''``\ , \ ``bodyList = [None, None]``\ , \ ``localPosition0 = [0.,0.,0.]``\ , \ ``localPosition1 = [0.,0.,0.]``\ , \ ``referenceLength = None``\ , \ ``stiffness = 0.``\ , \ ``damping = 0.``\ , \ ``force = 0.``\ , \ ``velocityOffset = 0.``\ , \ ``springForceUserFunction = 0``\ , \ ``bodyOrNodeList = [None, None]``\ , \ ``show = True``\ , \ ``drawSize = -1``\ , \ ``color = color4default``\ )

- | \ *function description*\ :
  | helper function to create SpringDamper connector, using arguments from ObjectConnectorSpringDamper; similar interface as CreateDistanceConstraint(...), see there for for further information
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateSpringDamper.
- | \ *input*\ :
  | \ ``name``\ : name string for connector; markers get Marker0:name and Marker1:name
  | \ ``bodyList``\ : a list of two body numbers (ObjectIndex) to be connected
  | \ ``localPosition0``\ : local position (as 3D list or numpy array) on body0, if not a node number
  | \ ``localPosition1``\ : local position (as 3D list or numpy array) on body1, if not a node number
  | \ ``referenceLength``\ : if None, length is computed from reference position of bodies or nodes; if not None, this scalar reference length is used for spring
  | \ ``stiffness``\ : scalar stiffness coefficient
  | \ ``damping``\ : scalar damping coefficient
  | \ ``force``\ : scalar additional force applied
  | \ ``velocityOffset``\ : scalar offset: if referenceLength is changed over time, the velocityOffset may be changed accordingly to emulate a reference motion
  | \ ``springForceUserFunction``\ : a user function springForceUserFunction(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force)->float ; this function replaces the internal connector force compuation
  | \ ``bodyOrNodeList``\ : alternative to bodyList; a list of object numbers (with specific localPosition0/1) or node numbers; may also be of mixed types; to use this case, set bodyList = [None,None]
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``drawSize``\ : general drawing size of connector
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | ObjectIndex; returns index of newly created object
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateMassPoint(referencePosition = [2,0,0],
                           initialVelocity = [2,5,0],
                           physicsMass = 1, gravity = [0,-9.81,0],
                           drawSize = 0.5, color=color4blue)
  oGround = mbs.AddObject(ObjectGround())
  #add vertical spring
  oSD = mbs.CreateSpringDamper(bodyList=[oGround, b0],
                               localPosition0=[2,1,0],
                               localPosition1=[0,0,0],
                               stiffness=1e4, damping=1e2,
                               drawSize=0.2)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  SC.visualizationSettings.nodes.drawNodesAsPoint=False
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `basicTutorial2024.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/basicTutorial2024.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `springDamperTutorialNew.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/springDamperTutorialNew.py>`_\  (Ex), \ `symbolicUserFunctionMasses.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/symbolicUserFunctionMasses.py>`_\  (Ex), \ `tutorialNeuralNetwork.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/tutorialNeuralNetwork.py>`_\  (Ex), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM), \ `symbolicUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/symbolicUserFunctionTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createcartesianspringdamper:

Function: CreateCartesianSpringDamper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateCartesianSpringDamper <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L695>`__\ (\ ``name = ''``\ , \ ``bodyList = [None, None]``\ , \ ``localPosition0 = [0.,0.,0.]``\ , \ ``localPosition1 = [0.,0.,0.]``\ , \ ``stiffness = [0.,0.,0.]``\ , \ ``damping = [0.,0.,0.]``\ , \ ``offset = [0.,0.,0.]``\ , \ ``springForceUserFunction = 0``\ , \ ``bodyOrNodeList = [None, None]``\ , \ ``show = True``\ , \ ``drawSize = -1``\ , \ ``color = color4default``\ )

- | \ *function description*\ :
  | helper function to create CartesianSpringDamper connector, using arguments from ObjectConnectorCartesianSpringDamper
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateCartesianSpringDamper.
- | \ *input*\ :
  | \ ``name``\ : name string for connector; markers get Marker0:name and Marker1:name
  | \ ``bodyList``\ : a list of two body numbers (ObjectIndex) to be connected
  | \ ``localPosition0``\ : local position (as 3D list or numpy array) on body0, if not a node number
  | \ ``localPosition1``\ : local position (as 3D list or numpy array) on body1, if not a node number
  | \ ``stiffness``\ : stiffness coefficients (as 3D list or numpy array)
  | \ ``damping``\ : damping coefficients (as 3D list or numpy array)
  | \ ``offset``\ : offset vector (as 3D list or numpy array)
  | \ ``springForceUserFunction``\ : a user function springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset)->[float,float,float] ; this function replaces the internal connector force compuation
  | \ ``bodyOrNodeList``\ : alternative to bodyList; a list of object numbers (with specific localPosition0/1) or node numbers; may also be of mixed types; to use this case, set bodyList = [None,None]
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``drawSize``\ : general drawing size of connector
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | ObjectIndex; returns index of newly created object
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateMassPoint(referencePosition = [7,0,0],
                            physicsMass = 1, gravity = [0,-9.81,0],
                            drawSize = 0.5, color=color4blue)
  oGround = mbs.AddObject(ObjectGround())
  oSD = mbs.CreateCartesianSpringDamper(bodyList=[oGround, b0],
                                localPosition0=[7.5,1,0],
                                localPosition1=[0,0,0],
                                stiffness=[200,2000,0], damping=[2,20,0],
                                drawSize=0.2)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  SC.visualizationSettings.nodes.drawNodesAsPoint=False
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `cartesianSpringDamper.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamper.py>`_\  (Ex), \ `cartesianSpringDamperUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamperUserFunction.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `computeODE2AEeigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2AEeigenvaluesTest.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-createrigidbodyspringdamper:

Function: CreateRigidBodySpringDamper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateRigidBodySpringDamper <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L783>`__\ (\ ``name = ''``\ , \ ``bodyList = [None, None]``\ , \ ``localPosition0 = [0.,0.,0.]``\ , \ ``localPosition1 = [0.,0.,0.]``\ , \ ``stiffness = np.zeros((6,6))``\ , \ ``damping = np.zeros((6,6))``\ , \ ``offset = [0.,0.,0.,0.,0.,0.]``\ , \ ``rotationMatrixJoint = np.eye(3)``\ , \ ``useGlobalFrame = True``\ , \ ``intrinsicFormulation = True``\ , \ ``springForceTorqueUserFunction = 0``\ , \ ``postNewtonStepUserFunction = 0``\ , \ ``bodyOrNodeList = [None, None]``\ , \ ``show = True``\ , \ ``drawSize = -1``\ , \ ``color = color4default``\ )

- | \ *function description*\ :
  | helper function to create RigidBodySpringDamper connector, using arguments from ObjectConnectorRigidBodySpringDamper, see there for the full documentation
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateRigidBodySpringDamper.
- | \ *input*\ :
  | \ ``name``\ : name string for connector; markers get Marker0:name and Marker1:name
  | \ ``bodyList``\ : a list of two body numbers (ObjectIndex) to be connected
  | \ ``localPosition0``\ : local position (as 3D list or numpy array) on body0, if not a node number
  | \ ``localPosition1``\ : local position (as 3D list or numpy array) on body1, if not a node number
  | \ ``stiffness``\ : stiffness coefficients (as 6D matrix or numpy array)
  | \ ``damping``\ : damping coefficients (as 6D matrix or numpy array)
  | \ ``offset``\ : offset vector (as 6D list or numpy array)
  | \ ``rotationMatrixJoint``\ : additional rotation matrix; in case  useGlobalFrame=False, it transforms body0/node0 local frame to joint frame; if useGlobalFrame=True, it transforms global frame to joint frame
  | \ ``useGlobalFrame``\ : if False, the rotationMatrixJoint is defined in the local coordinate system of body0
  | \ ``intrinsicFormulation``\ : if True, uses intrinsic formulation of Maserati and Morandini, which uses matrix logarithm and is independent of order of markers (preferred formulation); otherwise, Tait-Bryan angles are used for computation of torque, see documentation
  | \ ``springForceTorqueUserFunction``\ : a user function springForceTorqueUserFunction(mbs, t, itemNumber, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1, offset)->[float,float,float, float,float,float] ; this function replaces the internal connector force / torque compuation
  | \ ``postNewtonStepUserFunction``\ : a special user function postNewtonStepUserFunction(mbs, t, Index itemIndex, dataCoordinates, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1, offset)->[PNerror, recommendedStepSize, data[0], data[1], ...] ; for details, see RigidBodySpringDamper for full docu
  | \ ``bodyOrNodeList``\ : alternative to bodyList; a list of object numbers (with specific localPosition0/1) or node numbers; may also be of mixed types; to use this case, set bodyList = [None,None]
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``drawSize``\ : general drawing size of connector
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | ObjectIndex; returns index of newly created object
- | \ *example*\ :

.. code-block:: python

  #TODO


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `rigidBodySpringDamperIntrinsic.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodySpringDamperIntrinsic.py>`_\  (TM)



.. _sec-mainsystemextensions-createrevolutejoint:

Function: CreateRevoluteJoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateRevoluteJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L928>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``position = []``\ , \ ``axis = []``\ , \ ``useGlobalFrame = True``\ , \ ``show = True``\ , \ ``axisRadius = 0.1``\ , \ ``axisLength = 0.4``\ , \ ``color = color4default``\ )

- | \ *function description*\ :
  | Create revolute joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateRevoluteJoint.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of object numbers for body0 and body1; must be rigid body or ground object
  | \ ``position``\ : a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  | \ ``axis``\ : a 3D vector as list or np.array containing the joint axis either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True)
  | \ ``useGlobalFrame``\ : if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``axisRadius``\ : radius of axis for connector graphical representation
  | \ ``axisLength``\ : length of axis for connector graphical representation
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | [ObjectIndex, MarkerIndex, MarkerIndex]; returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [3,0,0],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1],
                                                                        color=color4steelblue)])
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b0], position=[2.5,0,0], axis=[0,0,1],
                          useGlobalFrame=True, axisRadius=0.02, axisLength=0.14)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Ex), \ `pistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pistonEngine.py>`_\  (Ex), \ `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_\  (Ex), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM), \ `perf3DRigidBodies.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/perf3DRigidBodies.py>`_\  (TM)



.. _sec-mainsystemextensions-createprismaticjoint:

Function: CreatePrismaticJoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreatePrismaticJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1030>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``position = []``\ , \ ``axis = []``\ , \ ``useGlobalFrame = True``\ , \ ``show = True``\ , \ ``axisRadius = 0.1``\ , \ ``axisLength = 0.4``\ , \ ``color = color4default``\ )

- | \ *function description*\ :
  | Create prismatic joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreatePrismaticJoint.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of object numbers for body0 and body1; must be rigid body or ground object
  | \ ``position``\ : a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  | \ ``axis``\ : a 3D vector as list or np.array containing the joint axis either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True)
  | \ ``useGlobalFrame``\ : if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``axisRadius``\ : radius of axis for connector graphical representation
  | \ ``axisLength``\ : length of axis for connector graphical representation
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | [ObjectIndex, MarkerIndex, MarkerIndex]; returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [4,0,0],
                           initialVelocity = [0,4,0],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1],
                                                                        color=color4steelblue)])
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreatePrismaticJoint(bodyNumbers=[oGround, b0], position=[3.5,0,0], axis=[0,1,0],
                           useGlobalFrame=True, axisRadius=0.02, axisLength=1)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Ex), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-createsphericaljoint:

Function: CreateSphericalJoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateSphericalJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1124>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``position = []``\ , \ ``constrainedAxes = [1,1,1]``\ , \ ``useGlobalFrame = True``\ , \ ``show = True``\ , \ ``jointRadius = 0.1``\ , \ ``color = color4default``\ )

- | \ *function description*\ :
  | Create spherical joint between two bodies; definition of joint position in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers are automatically computed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateSphericalJoint.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of object numbers for body0 and body1; must be mass point, rigid body or ground object
  | \ ``position``\ : a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  | \ ``constrainedAxes``\ : flags, which determines which (global) translation axes are constrained; each entry may only be 0 (=free) axis or 1 (=constrained axis)
  | \ ``useGlobalFrame``\ : if False, the point and axis vectors are defined in the local coordinate system of body0
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``jointRadius``\ : radius of sphere for connector graphical representation
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | [ObjectIndex, MarkerIndex, MarkerIndex]; returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [5,0,0],
                           initialAngularVelocity = [5,0,0],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1],
                                                                        color=color4orange)])
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreateSphericalJoint(bodyNumbers=[oGround, b0], position=[5.5,0,0],
                           useGlobalFrame=True, jointRadius=0.06)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-creategenericjoint:

Function: CreateGenericJoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateGenericJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1214>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``position = []``\ , \ ``rotationMatrixAxes = np.eye(3)``\ , \ ``constrainedAxes = [1,1,1, 1,1,1]``\ , \ ``useGlobalFrame = True``\ , \ ``offsetUserFunction = 0``\ , \ ``offsetUserFunction_t = 0``\ , \ ``show = True``\ , \ ``axesRadius = 0.1``\ , \ ``axesLength = 0.4``\ , \ ``color = color4default``\ )

- | \ *function description*\ :
  | Create generic joint between two bodies; definition of joint position (position) and axes (rotationMatrixAxes) in global coordinates (useGlobalFrame=True) or in local coordinates of body0 (useGlobalFrame=False), where rotationMatrixAxes is an additional rotation to body0; all markers, markerRotation and other quantities are automatically computed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateGenericJoint.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumber0``\ : a object number for body0, must be rigid body or ground object
  | \ ``bodyNumber1``\ : a object number for body1, must be rigid body or ground object
  | \ ``position``\ : a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  | \ ``rotationMatrixAxes``\ : rotation matrix which defines orientation of constrainedAxes; if useGlobalFrame, this rotation matrix is global, else the rotation matrix is post-multiplied with the rotation of body0, identical with rotationMarker0 in the joint
  | \ ``constrainedAxes``\ : flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; each entry may only be 0 (=free) axis or 1 (=constrained axis); ALL constrained Axes are defined relative to reference rotation of body0 times rotation0
  | \ ``useGlobalFrame``\ : if False, the position is defined in the local coordinate system of body0, otherwise it is defined in global coordinates
  | \ ``offsetUserFunction``\ : a user function offsetUserFunction(mbs, t, itemNumber, offsetUserFunctionParameters)->float ; this function replaces the internal (constant) by a user-defined offset. This allows to realize rheonomic joints and allows kinematic simulation
  | \ ``offsetUserFunction_t``\ : a user function offsetUserFunction_t(mbs, t, itemNumber, offsetUserFunctionParameters)->float ; this function replaces the internal (constant) by a user-defined offset velocity; this function is used instead of offsetUserFunction, if velocityLevel (index2) time integration
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``axesRadius``\ : radius of axes for connector graphical representation
  | \ ``axesLength``\ : length of axes for connector graphical representation
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | [ObjectIndex, MarkerIndex, MarkerIndex]; returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [6,0,0],
                           initialAngularVelocity = [0,8,0],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1],
                                                                        color=color4orange)])
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreateGenericJoint(bodyNumbers=[oGround, b0], position=[5.5,0,0],
                         constrainedAxes=[1,1,1, 1,0,0],
                         rotationMatrixAxes=RotationMatrixX(0.125*pi), #tilt axes
                         useGlobalFrame=True, axesRadius=0.02, axesLength=0.2)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `pistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pistonEngine.py>`_\  (Ex), \ `universalJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/universalJoint.py>`_\  (Ex), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `computeODE2AEeigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2AEeigenvaluesTest.py>`_\  (TM), \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM), \ `rigidBodyCOMtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyCOMtest.py>`_\  (TM), \ `sliderCrank3Dbenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dbenchmark.py>`_\  (TM)



.. _sec-mainsystemextensions-createdistanceconstraint:

Function: CreateDistanceConstraint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateDistanceConstraint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1327>`__\ (\ ``name = ''``\ , \ ``bodyList = [None, None]``\ , \ ``localPosition0 = [0.,0.,0.]``\ , \ ``localPosition1 = [0.,0.,0.]``\ , \ ``distance = None``\ , \ ``bodyOrNodeList = [None, None]``\ , \ ``show = True``\ , \ ``drawSize = -1.``\ , \ ``color = color4default``\ )

- | \ *function description*\ :
  | Create distance joint between two bodies; definition of joint positions in local coordinates of bodies or nodes; if distance=None, it is computed automatically from reference length; all markers are automatically computed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateDistanceConstraint.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyList``\ : a list of two body numbers (ObjectIndex) to be constrained
  | \ ``localPosition0``\ : local position (as 3D list or numpy array) on body0, if not a node number
  | \ ``localPosition1``\ : local position (as 3D list or numpy array) on body1, if not a node number
  | \ ``distance``\ : if None, distance is computed from reference position of bodies or nodes; if not None, this distance is prescribed between the two positions; if distance = 0, it will create a SphericalJoint as this case is not possible with a DistanceConstraint
  | \ ``bodyOrNodeList``\ : alternative to bodyList; a list of object numbers (with specific localPosition0/1) or node numbers; may also be of mixed types; to use this case, set bodyList = [None,None]
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``drawSize``\ : general drawing size of node
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | [ObjectIndex, MarkerIndex, MarkerIndex]; returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                    sideLengths=[1,0.1,0.1]),
                            referencePosition = [6,0,0],
                            gravity = [0,-9.81,0],
                            graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1],
                                                                        color=color4orange)])
  m1 = mbs.CreateMassPoint(referencePosition=[5.5,-1,0],
                           physicsMass=1, drawSize = 0.2)
  n1 = mbs.GetObject(m1)['nodeNumber']
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreateDistanceConstraint(bodyList=[oGround, b0],
                               localPosition0 = [6.5,1,0],
                               localPosition1 = [0.5,0,0],
                               distance=None, #automatically computed
                               drawSize=0.06)
  mbs.CreateDistanceConstraint(bodyOrNodeList=[b0, n1],
                               localPosition0 = [-0.5,0,0],
                               localPosition1 = [0.,0.,0.], #must be [0,0,0] for Node
                               distance=None, #automatically computed
                               drawSize=0.06)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Ex), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-createforce:

Function: CreateForce
^^^^^^^^^^^^^^^^^^^^^
`CreateForce <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1447>`__\ (\ ``name = ''``\ , \ ``bodyNumber = None``\ , \ ``loadVector = [0.,0.,0.]``\ , \ ``localPosition = [0.,0.,0.]``\ , \ ``bodyFixed = False``\ , \ ``loadVectorUserFunction = 0``\ , \ ``show = True``\ )

- | \ *function description*\ :
  | helper function to create force applied to given body
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateForce.
- | \ *input*\ :
  | \ ``name``\ : name string for object
  | \ ``bodyNumber``\ : body number (ObjectIndex) at which the force is applied to
  | \ ``loadVector``\ : force vector (as 3D list or numpy array)
  | \ ``localPosition``\ : local position (as 3D list or numpy array) where force is applied
  | \ ``bodyFixed``\ : if True, the force is corotated with the body; else, the force is global
  | \ ``loadVectorUserFunction``\ : A Python function f(mbs, t, load)->loadVector which defines the time-dependent load and replaces loadVector in every time step; the arg load is the static loadVector
  | \ ``show``\ : if True, load is drawn
- | \ *output*\ :
  | LoadIndex; returns load index
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0=mbs.CreateMassPoint(referencePosition = [0,0,0],
                         initialVelocity = [2,5,0],
                         physicsMass = 1, gravity = [0,-9.81,0],
                         drawSize = 0.5, color=color4blue)
  f0=mbs.CreateForce(bodyNumber=b0, loadVector=[100,0,0],
                     localPosition=[0,0,0])
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `cartesianSpringDamper.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamper.py>`_\  (Ex), \ `cartesianSpringDamperUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamperUserFunction.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Ex), \ `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_\  (Ex), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-createtorque:

Function: CreateTorque
^^^^^^^^^^^^^^^^^^^^^^
`CreateTorque <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1525>`__\ (\ ``name = ''``\ , \ ``bodyNumber = None``\ , \ ``loadVector = [0.,0.,0.]``\ , \ ``localPosition = [0.,0.,0.]``\ , \ ``bodyFixed = False``\ , \ ``loadVectorUserFunction = 0``\ , \ ``show = True``\ )

- | \ *function description*\ :
  | helper function to create torque applied to given body
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateTorque.
- | \ *input*\ :
  | \ ``name``\ : name string for object
  | \ ``bodyNumber``\ : body number (ObjectIndex) at which the torque is applied to
  | \ ``loadVector``\ : torque vector (as 3D list or numpy array)
  | \ ``localPosition``\ : local position (as 3D list or numpy array) where torque is applied
  | \ ``bodyFixed``\ : if True, the torque is corotated with the body; else, the torque is global
  | \ ``loadVectorUserFunction``\ : A Python function f(mbs, t, load)->loadVector which defines the time-dependent load and replaces loadVector in every time step; the arg load is the static loadVector
  | \ ``show``\ : if True, load is drawn
- | \ *output*\ :
  | LoadIndex; returns load index
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [1,3,0],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1],
                                                                        color=color4red)])
  f0=mbs.CreateTorque(bodyNumber=b0, loadVector=[0,100,0])
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Ex), \ `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_\  (Ex), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)


.. _sec-mainsystem-pythonextensions:


MainSystem extensions (general)
===============================

This section represents general extensions to MainSystem, which are direct calls to Python functions, such as PlotSensor or SolveDynamic; these extensions allow a more intuitive interaction with the MainSystem class, see the following example. For activation, import \ ``exudyn.mainSystemExtensions``\  or \ ``exudyn.utilities``\ 

.. code-block:: python
   :linenos:
   
   #this example sketches the usage 
   #for complete examples see Examples/ or TestModels/ folders
   #create some multibody system (mbs) first:
   # ... 
   #
   #compute system degree of freedom: 
   mbs.ComputeSystemDegreeOfFreedom(verbose=True)
   #
   #call solver function directly from mbs:
   mbs.SolveDynamic(exu.SimulationSettings())
   #
   #plot sensor directly from mbs:
   mbs.PlotSensor(...)


.. _sec-mainsystemextensions-solutionviewer:

Function: SolutionViewer
^^^^^^^^^^^^^^^^^^^^^^^^
`SolutionViewer <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/interactive.py\#L786>`__\ (\ ``solution = None``\ , \ ``rowIncrement = 1``\ , \ ``timeout = 0.04``\ , \ ``runOnStart = True``\ , \ ``runMode = 2``\ , \ ``fontSize = 12``\ , \ ``title = ''``\ , \ ``checkRenderEngineStopFlag = True``\ )

- | \ *function description*\ :
  | open interactive dialog and visulation (animate) solution loaded with LoadSolutionFile(...); Change slider 'Increment' to change the automatic increment of time frames; Change mode between continuous run, one cycle (fits perfect for animation recording) or 'Static' (to change Solution steps manually with the mouse); update period also lets you change the speed of animation; Press Run / Stop button to start/stop interactive mode (updating of grpahics)
  | - NOTE that this function is added to MainSystem via Python function SolutionViewer.
- | \ *input*\ :
  | \ ``solution``\ : solution dictionary previously loaded with exudyn.utilities.LoadSolutionFile(...); will be played from first to last row; if solution==None, it tries to load the file coordinatesSolutionFileName as stored in mbs.sys['simulationSettings'], which are the simulationSettings of the previous simulation
  | \ ``rowIncrement``\ : can be set larger than 1 in order to skip solution frames: e.g. rowIncrement=10 visualizes every 10th row (frame)
  | \ ``timeout``\ : in seconds is used between frames in order to limit the speed of animation; e.g. use timeout=0.04 to achieve approximately 25 frames per second
  | \ ``runOnStart``\ : immediately go into 'Run' mode
  | \ ``runMode``\ : 0=continuous run, 1=one cycle, 2=static (use slider/mouse to vary time steps)
  | \ ``fontSize``\ : define font size for labels in InteractiveDialog
  | \ ``title``\ : if empty, it uses default; otherwise define specific title
  | \ ``checkRenderEngineStopFlag``\ : if True, stopping renderer (pressing Q or Escape) also causes stopping the interactive dialog
- | \ *output*\ :
  | None; updates current visualization state, renders the scene continuously (after pressing button 'Run')
- | \ *example*\ :

.. code-block:: python

  #HERE, mbs must contain same model as solution stored in coordinatesSolution.txt
  #adjust autoFitScence, otherwise it may lead to unwanted fit to scene
  SC.visualizationSettings.general.autoFitScene = False
  from exudyn.interactive import SolutionViewer #import function
  sol = LoadSolutionFile('coordinatesSolution.txt') #load solution: adjust to your file name
  mbs.SolutionViewer(sol) #call via MainSystem


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `ANCFcableCantilevered.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcableCantilevered.py>`_\  (Ex), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Ex), \ `basicTutorial2024.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/basicTutorial2024.py>`_\  (Ex), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TM), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TM)



.. _sec-mainsystemextensions-plotsensor:

Function: PlotSensor
^^^^^^^^^^^^^^^^^^^^
`PlotSensor <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L230>`__\ (\ ``sensorNumbers = []``\ , \ ``components = 0``\ , \ ``xLabel = 'time (s)'``\ , \ ``yLabel = None``\ , \ ``labels = []``\ , \ ``colorCodeOffset = 0``\ , \ ``newFigure = True``\ , \ ``closeAll = False``\ , \ ``componentsX = []``\ , \ ``title = ''``\ , \ ``figureName = ''``\ , \ ``fontSize = 16``\ , \ ``colors = []``\ , \ ``lineStyles = []``\ , \ ``lineWidths = []``\ , \ ``markerStyles = []``\ , \ ``markerSizes = []``\ , \ ``markerDensity = 0.08``\ , \ ``rangeX = []``\ , \ ``rangeY = []``\ , \ ``majorTicksX = 10``\ , \ ``majorTicksY = 10``\ , \ ``offsets = []``\ , \ ``factors = []``\ , \ ``subPlot = []``\ , \ ``sizeInches = [6.4,4.8]``\ , \ ``fileName = ''``\ , \ ``useXYZcomponents = True``\ , \ ``**kwargs``\ )

- | \ *function description*\ :
  | Helper function for direct and easy visualization of sensor outputs, without need for loading text files, etc.; PlotSensor can be used to simply plot, e.g., the measured x-Position over time in a figure. PlotSensor provides an interface to matplotlib (which needs to be installed). Default values of many function arguments can be changed using the exudyn.plot function PlotSensorDefaults(), see there for usage.
  | - NOTE that this function is added to MainSystem via Python function PlotSensor.
- | \ *input*\ :
  | \ ``sensorNumbers``\ : consists of one or a list of sensor numbers (type SensorIndex or int) as returned by the mbs function AddSensor(...); sensors need to set writeToFile=True and/or storeInternal=True for PlotSensor to work; alternatively, it may contain FILENAMES (incl. path) to stored sensor or solution files OR a numpy array instead of sensor numbers; the format of data (file or numpy array) must contain per row the time and according solution values in columns; if components is a list and sensorNumbers is a scalar, sensorNumbers is adjusted automatically to the components
  | \ ``components``\ : consists of one or a list of components according to the component of the sensor to be plotted at y-axis; if components is a list and sensorNumbers is a scalar, sensorNumbers is adjusted automatically to the components; as always, components are zero-based, meaning 0=X, 1=Y, etc.; for regular sensor files, time will be component=-1; to show the norm (e.g., of a force vector), use component=[plot.componentNorm] for according sensors; norm will consider all values of sensor except time (for 3D force, it will be \ :math:`\sqrt{f_0^2+f_1^2+f_2^2}`\ ); offsets and factors are mapped on norm (plot value=factor\*(norm(values) + offset) ), not on component values
  | \ ``componentsX``\ : default componentsX=[] uses time in files; otherwise provide componentsX as list of components (or scalar) representing x components of sensors in plotted curves; DON'T forget to change xLabel accordingly!
  | Using componentsX=[...] with a list of column indices specifies the respective columns used for the x-coordinates in all sensors; by default, values are plotted against the first column in the files, which is time; according to counting in PlotSensor, this represents componentX=-1;
  | plotting y over x in a position sensor thus reads: components=[1], componentsX=[0];
  | plotting time over x reads: components=[-1], componentsX=[0];
  | the default value reads componentsX=[-1,-1,...]
  | \ ``xLabel``\ : string for text at x-axis
  | \ ``yLabel``\ : string for text at y-axis (default: None==> label is automatically computed from sensor value types)
  | \ ``labels``\ : string (for one sensor) or list of strings (according to number of sensors resp. components) representing the labels used in legend; if labels=[], automatically generated legend is used
  | \ ``rangeX``\ : default rangeX=[]: computes range automatically; otherwise use rangeX to set range (limits) for x-axis provided as sorted list of two floats, e.g., rangeX=[0,4]
  | \ ``rangeY``\ : default rangeY=[]: computes range automatically; otherwise use rangeY to set range (limits) for y-axis provided as sorted list of two floats, e.g., rangeY=[-1,1]
  | \ ``figureName``\ : optional name for figure, if newFigure=True
  | \ ``fontSize``\ : change general fontsize of axis, labels, etc. (matplotlib default is 12, default in PlotSensor: 16)
  | \ ``title``\ : optional string representing plot title
  | \ ``offsets``\ : provide as scalar, list of scalars (per sensor) or list of 2D numpy.arrays (per sensor, having same rows/columns as sensor data; in this case it will also influence x-axis if componentsX is different from -1) to add offset to each sensor output; for an original value fOrig, the new value reads fNew = factor\*(fOrig+offset); for offset provided as numpy array (with same time values), the 'time' column is ignored in the offset computation; can be used to compute difference of sensors; if offsets=[], no offset is used
  | \ ``factors``\ : provide as scalar or list (per sensor) to add factor to each sensor output; for an original value fOrig, the new value reads fNew = factor\*(fOrig+offset); if factor=[], no factor is used
  | \ ``majorTicksX``\ : number of major ticks on x-axis; default: 10
  | \ ``majorTicksY``\ : number of major ticks on y-axis; default: 10
  | \ ``colorCodeOffset``\ : int offset for color code, color codes going from 0 to 27 (see PlotLineCode(...)); automatic line/color codes are used if no colors and lineStyles are used
  | \ ``colors``\ : color is automatically selected from colorCodeOffset if colors=[]; otherwise chose from 'b', 'g', 'r', 'c', 'm', 'y', 'k' and many other colors see https://matplotlib.org/stable/gallery/color/named_colors.html
  | \ ``lineStyles``\ : line style is automatically selected from colorCodeOffset if lineStyles=[]; otherwise define for all lines with string or with list of strings, chosing from '-', '--', '-.', ':', or ''
  | \ ``lineWidths``\ : float to define line width by float (default=1); either use single float for all sensors or list of floats with length >= number of sensors
  | \ ``markerStyles``\ : if different from [], marker styles are defined as list of marker style strings or single string for one sensor; chose from '.', 'o', 'x', '+' ... check listMarkerStylesFilled and listMarkerStyles in exudyn.plot and see https://matplotlib.org/stable/api/markers_api.html ; ADD a space to markers to make them empty (transparent), e.g. 'o ' will create an empty circle
  | \ ``markerSizes``\ : float to define marker size by float (default=6); either use single float for all sensors or list of floats with length >= number of sensors
  | \ ``markerDensity``\ : if int, it defines approx. the total number of markers used along each graph; if float, this defines the distance of markers relative to the diagonal of the plot (default=0.08); if None, it adds a marker to every data point if marker style is specified for sensor
  | \ ``newFigure``\ : if True, a new matplotlib.pyplot figure is created; otherwise, existing figures are overwritten
  | \ ``subPlot``\ : given as list [nx, ny, position] with nx, ny being the number of subplots in x and y direction (nx=cols, ny=rows), and position in [1,..., nx\*ny] gives the position in the subplots; use the same structure for first PlotSensor (with newFigure=True) and all subsequent PlotSensor calls with newFigure=False, which creates the according subplots; default=[](no subplots)
  | \ ``sizeInches``\ : given as list [sizeX, sizeY] with the sizes per (sub)plot given in inches; default: [6.4, 4.8]; in case of sub plots, the total size of the figure is computed from nx\*sizeInches[0] and ny\*sizeInches[1]
  | \ ``fileName``\ : if this string is non-empty, figure will be saved to given path and filename (use figName.pdf to safe as PDF or figName.png to save as PNG image); use matplotlib.use('Agg') in order not to open figures if you just want to save them
  | \ ``useXYZcomponents``\ : of True, it will use X, Y and Z for sensor components, e.g., measuring Position, Velocity, etc. wherever possible
  | \ ``closeAll``\ : if True, close all figures before opening new one (do this only in first PlotSensor command!)
  | \ ``[*kwargs]``\ :
  | \ ``minorTicksXon``\ : if True, turn minor ticks for x-axis on
  | \ ``minorTicksYon``\ : if True, turn minor ticks for y-axis on
  | \ ``logScaleX``\ : use log scale for x-axis
  | \ ``logScaleY``\ : use log scale for y-axis
  | \ ``fileCommentChar``\ : if exists, defines the comment character in files (\#, 
  | \ ``fileDelimiterChar``\ : if exists, defines the character indicating the columns for data (',', ' ', ';', ...)
- | \ *output*\ :
  | [Any, Any, Any, Any]; plots the sensor data; returns [plt, fig, ax, line] in which plt is matplotlib.pyplot, fig is the figure (or None), ax is the axis (or None) and line is the return value of plt.plot (or None) which could be changed hereafter
- | \ *notes*\ :
  | adjust default values by modifying the variables exudyn.plot.plotSensorDefault..., e.g., exudyn.plot.plotSensorDefaultFontSize
- | \ *example*\ :

.. code-block:: python

  #assume to have some position-based nodes 0 and 1:
  s0=mbs.AddSensor(SensorNode(nodeNumber=0, fileName='s0.txt',
                              outputVariableType=exu.OutputVariableType.Position))
  s1=mbs.AddSensor(SensorNode(nodeNumber=1, fileName='s1.txt',
                              outputVariableType=exu.OutputVariableType.Position))
  mbs.PlotSensor(s0, 0) #plot x-coordinate
  #plot x for s0 and z for s1:
  mbs.PlotSensor(sensorNumbers=[s0,s1], components=[0,2], yLabel='this is the position in meter')
  mbs.PlotSensor(sensorNumbers=s0, components=plot.componentNorm) #norm of position
  mbs.PlotSensor(sensorNumbers=s0, components=[0,1,2], factors=1000., title='Answers to the big questions')
  mbs.PlotSensor(sensorNumbers=s0, components=[0,1,2,3],
             yLabel='Coordantes with offset 1\nand scaled with $\\frac{1}{1000}$',
             factors=1e-3, offsets=1,fontSize=12, closeAll=True)
  #assume to have body sensor sBody, marker sensor sMarker:
  mbs.PlotSensor(sensorNumbers=[sBody]*3+[sMarker]*3, components=[0,1,2,0,1,2],
             colorCodeOffset=3, newFigure=False, fontSize=10,
             yLabel='Rotation $\\alpha, \\beta, \\gamma$ and\n Position $x,y,z$',
             title='compare marker and body sensor')
  #assume having file plotSensorNode.txt:
  mbs.PlotSensor(sensorNumbers=[s0]*3+ [filedir+'plotSensorNode.txt']*3,
             components=[0,1,2]*2)
  #plot y over x:
  mbs.PlotSensor(sensorNumbers=s0, componentsX=[0], components=[1], xLabel='x-Position', yLabel='y-Position')
  #for further examples, see also Examples/plotSensorExamples.py


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFALEtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFALEtest.py>`_\  (Ex), \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Ex), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Ex), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Ex), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TM), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TM)



.. _sec-mainsystemextensions-solvestatic:

Function: SolveStatic
^^^^^^^^^^^^^^^^^^^^^
`SolveStatic <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L154>`__\ (\ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``updateInitialValues = False``\ , \ ``storeSolver = True``\ , \ ``showHints = False``\ , \ ``showCausingItems = True``\ )

- | \ *function description*\ :
  | solves the static mbs problem using simulationSettings; check theDoc.pdf for MainSolverStatic for further details of the static solver; this function is also available in exudyn (using exudyn.SolveStatic(...))
  | - NOTE that this function is added to MainSystem via Python function SolveStatic.
- | \ *input*\ :
  | \ ``simulationSettings``\ : specific simulation settings out of exu.SimulationSettings(), as described in Section :ref:`sec-solutionsettings`\ ; use options for newton, discontinuous settings, etc., from staticSolver sub-items
  | \ ``updateInitialValues``\ : if True, the results are written to initial values, such at a consecutive simulation uses the results of this simulation as the initial values of the next simulation
  | \ ``storeSolver``\ : if True, the staticSolver object is stored in the mbs.sys dictionary as mbs.sys['staticSolver'], and simulationSettings are stored as mbs.sys['simulationSettings']
- | \ *output*\ :
  | bool; returns True, if successful, False if fails; if storeSolver = True, mbs.sys contains staticSolver, which allows to investigate solver problems (check theDoc.pdf Section :ref:`sec-solversubstructures`\  and the items described in Section :ref:`sec-mainsolverstatic`\ )
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.itemInterface import *
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  #create simple system:
  ground = mbs.AddObject(ObjectGround())
  mbs.AddNode(NodePoint())
  body = mbs.AddObject(MassPoint(physicsMass=1, nodeNumber=0))
  m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=ground))
  m1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=body))
  mbs.AddObject(CartesianSpringDamper(markerNumbers=[m0,m1], stiffness=[100,100,100]))
  mbs.AddLoad(LoadForceVector(markerNumber=m1, loadVector=[10,10,10]))
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings()
  simulationSettings.timeIntegration.endTime = 10
  success = mbs.SolveStatic(simulationSettings, storeSolver = True)
  print("success =", success)
  print("iterations = ", mbs.sys['staticSolver'].it)
  print("pos=", mbs.GetObjectOutputBody(body,localPosition=[0,0,0],
        variableType=exu.OutputVariableType.Position))


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `3SpringsDistance.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/3SpringsDistance.py>`_\  (Ex), \ `ALEANCFpipe.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ALEANCFpipe.py>`_\  (Ex), \ `ANCFALEtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFALEtest.py>`_\  (Ex), \ `ANCFcantileverTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcantileverTest.py>`_\  (Ex), \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Ex), \ `ANCFBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamTest.py>`_\  (TM), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TM), \ `ANCFcontactCircleTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactCircleTest.py>`_\  (TM)



.. _sec-mainsystemextensions-solvedynamic:

Function: SolveDynamic
^^^^^^^^^^^^^^^^^^^^^^
`SolveDynamic <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L219>`__\ (\ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``solverType = exudyn.DynamicSolverType.GeneralizedAlpha``\ , \ ``updateInitialValues = False``\ , \ ``storeSolver = True``\ , \ ``showHints = False``\ , \ ``showCausingItems = True``\ )

- | \ *function description*\ :
  | solves the dynamic mbs problem using simulationSettings and solver type; check theDoc.pdf for MainSolverImplicitSecondOrder for further details of the dynamic solver; this function is also available in exudyn (using exudyn.SolveDynamic(...))
  | - NOTE that this function is added to MainSystem via Python function SolveDynamic.
- | \ *input*\ :
  | \ ``simulationSettings``\ : specific simulation settings out of exu.SimulationSettings(), as described in Section :ref:`sec-solutionsettings`\ ; use options for newton, discontinuous settings, etc., from timeIntegration; therein, implicit second order solvers use settings from generalizedAlpha and explict solvers from explicitIntegration; be careful with settings, as the influence accuracy (step size!), convergence and performance (see special Section :ref:`sec-overview-basics-speedup`\ )
  | \ ``solverType``\ : use exudyn.DynamicSolverType to set specific solver (default=generalized alpha)
  | \ ``updateInitialValues``\ : if True, the results are written to initial values, such at a consecutive simulation uses the results of this simulation as the initial values of the next simulation
  | \ ``storeSolver``\ : if True, the staticSolver object is stored in the mbs.sys dictionary as mbs.sys['staticSolver'], and simulationSettings are stored as mbs.sys['simulationSettings']
  | \ ``showHints``\ : show additional hints, if solver fails
  | \ ``showCausingItems``\ : if linear solver fails, this option helps to identify objects, etc. which are related to a singularity in the linearized system matrix
- | \ *output*\ :
  | bool; returns True, if successful, False if fails; if storeSolver = True, mbs.sys contains staticSolver, which allows to investigate solver problems (check theDoc.pdf Section :ref:`sec-solversubstructures`\  and the items described in Section :ref:`sec-mainsolverstatic`\ )
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.itemInterface import *
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  #create simple system:
  ground = mbs.AddObject(ObjectGround())
  mbs.AddNode(NodePoint())
  body = mbs.AddObject(MassPoint(physicsMass=1, nodeNumber=0))
  m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=ground))
  m1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=body))
  mbs.AddObject(CartesianSpringDamper(markerNumbers=[m0,m1], stiffness=[100,100,100]))
  mbs.AddLoad(LoadForceVector(markerNumber=m1, loadVector=[10,10,10]))
  #
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings()
  simulationSettings.timeIntegration.endTime = 10
  success = mbs.SolveDynamic(simulationSettings, storeSolver = True)
  print("success =", success)
  print("iterations = ", mbs.sys['dynamicSolver'].it)
  print("pos=", mbs.GetObjectOutputBody(body,localPosition=[0,0,0],
        variableType=exu.OutputVariableType.Position))


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `3SpringsDistance.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/3SpringsDistance.py>`_\  (Ex), \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `ALEANCFpipe.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ALEANCFpipe.py>`_\  (Ex), \ `ANCFALEtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFALEtest.py>`_\  (Ex), \ `abaqusImportTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/abaqusImportTest.py>`_\  (TM), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM), \ `ANCFBeamEigTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamEigTest.py>`_\  (TM)



.. _sec-mainsystemextensions-computelinearizedsystem:

Function: ComputeLinearizedSystem
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeLinearizedSystem <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L366>`__\ (\ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``useSparseSolver = False``\ )

- | \ *function description*\ :
  | compute linearized system of equations for ODE2 part of mbs, not considering the effects of algebraic constraints
  | - NOTE that this function is added to MainSystem via Python function ComputeLinearizedSystem.
- | \ *input*\ :
  | \ ``simulationSettings``\ : specific simulation settings used for computation of jacobian (e.g., sparse mode in static solver enables sparse computation)
  | \ ``useSparseSolver``\ : if False (only for small systems), all eigenvalues are computed in dense mode (slow for large systems!); if True, only the numberOfEigenvalues are computed (numberOfEigenvalues must be set!); Currently, the matrices are exported only in DENSE MODE from mbs! NOTE that the sparsesolver accuracy is much less than the dense solver
- | \ *output*\ :
  | [ArrayLike, ArrayLike, ArrayLike]; [M, K, D]; list containing numpy mass matrix M, stiffness matrix K and damping matrix D
- | \ *notes*\ :
  | consider paper of Agundez, Vallejo, Freire, Mikkola, "The dependent coordinates in the linearization of constrained multibody systems: Handling and elimination", https://www.sciencedirect.com/science/article/pii/S0020740324000791
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  #
  b0 = mbs.CreateMassPoint(referencePosition = [2,0,0],
                           initialVelocity = [2*0,5,0],
                           physicsMass = 1, gravity = [0,-9.81,0],
                           drawSize = 0.5, color=color4blue)
  #
  oGround = mbs.AddObject(ObjectGround())
  #add vertical spring
  oSD = mbs.CreateSpringDamper(bodyOrNodeList=[oGround, b0],
                               localPosition0=[2,1,0],
                               localPosition1=[0,0,0],
                               stiffness=1e4, damping=1e2,
                               drawSize=0.2)
  #
  mbs.Assemble()
  [M,K,D] = mbs.ComputeLinearizedSystem()
  exu.Print('M=\n',M,'\nK=\n',K,'\nD=\n',D)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFBeamEigTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamEigTest.py>`_\  (TM), \ `ANCFBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamTest.py>`_\  (TM), \ `geometricallyExactBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/geometricallyExactBeamTest.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-computeode2eigenvalues:

Function: ComputeODE2Eigenvalues
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeODE2Eigenvalues <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L442>`__\ (\ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``useSparseSolver = False``\ , \ ``numberOfEigenvalues = 0``\ , \ ``constrainedCoordinates = []``\ , \ ``convert2Frequencies = False``\ , \ ``useAbsoluteValues = True``\ , \ ``ignoreAlgebraicEquations = False``\ , \ ``singularValuesTolerance = 1e-12``\ )

- | \ *function description*\ :
  | compute eigenvalues for unconstrained ODE2 part of mbs; the computation may include constraints in case that ignoreAlgebraicEquations=False; for algebraic constraints, however, a dense singular value decomposition of the constraint jacobian is used for the nullspace projection; the computation is done for the initial values of the mbs, independently of previous computations. If you would like to use the current state for the eigenvalue computation, you need to copy the current state to the initial state (using GetSystemState, SetSystemState, see Section :ref:`sec-mbs-systemdata`\ ); note that mass and stiffness matrices are computed in dense mode so far, while eigenvalues are computed according to useSparseSolver.
  | - NOTE that this function is added to MainSystem via Python function ComputeODE2Eigenvalues.
- | \ *input*\ :
  | \ ``simulationSettings``\ : specific simulation settings used for computation of jacobian (e.g., sparse mode in static solver enables sparse computation)
  | \ ``useSparseSolver``\ : if False (only for small systems), all eigenvalues are computed in dense mode (slow for large systems!); if True, only the numberOfEigenvalues are computed (numberOfEigenvalues must be set!); Currently, the matrices are exported only in DENSE MODE from mbs! NOTE that the sparsesolver accuracy is much less than the dense solver
  | \ ``numberOfEigenvalues``\ : number of eigenvalues and eivenvectors to be computed; if numberOfEigenvalues==0, all eigenvalues will be computed (may be impossible for larger or sparse problems!)
  | \ ``constrainedCoordinates``\ : if this list is non-empty (and there are no algebraic equations of ignoreAlgebraicEquations=True), the integer indices represent constrained coordinates of the system, which are fixed during eigenvalue/vector computation; according rows/columns of mass and stiffness matrices are erased; in this case, algebraic equations of the system are ignored
  | \ ``convert2Frequencies``\ : if True, the eigen values are converted into frequencies (Hz) and the output is [eigenFrequencies, eigenVectors]
  | \ ``useAbsoluteValues``\ : if True, abs(eigenvalues) is used, which avoids problems for small (close to zero) eigenvalues; needed, when converting to frequencies
  | \ ``ignoreAlgebraicEquations``\ : if True, algebraic equations (and constraint jacobian) are not considered for eigenvalue computation; otherwise, the solver tries to automatically project the system into the nullspace kernel of the constraint jacobian using a SVD; this gives eigenvalues of the constrained system; eigenvectors are not computed
  | \ ``singularValuesTolerance``\ : tolerance used to distinguish between zero and nonzero singular values for algebraic constraints projection
- | \ *output*\ :
  | [ArrayLike, ArrayLike]; [eigenValues, eigenVectors]; eigenValues being a numpy array of eigen values (\ :math:`\omega_i^2`\ , being the squared eigen frequencies in (\ :math:`\omega_i`\  in rad/s)!), eigenVectors a numpy array containing the eigenvectors in every column
- | \ *author*\ :
  | Johannes Gerstmayr, Michael Pieber
- | \ *example*\ :

.. code-block:: python

   #take any example from the Examples or TestModels folder, e.g., 'cartesianSpringDamper.py' and run it
   #specific example:
  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  #
  b0 = mbs.CreateMassPoint(referencePosition = [2,0,0],
                           initialVelocity = [2*0,5,0],
                           physicsMass = 1, gravity = [0,-9.81,0],
                           drawSize = 0.5, color=color4blue)
  #
  oGround = mbs.AddObject(ObjectGround())
  #add vertical spring
  oSD = mbs.CreateSpringDamper(bodyOrNodeList=[oGround, b0],
                               localPosition0=[2,1,0],
                               localPosition1=[0,0,0],
                               stiffness=1e4, damping=1e2,
                               drawSize=0.2)
  #
  mbs.Assemble()
  #
  [eigenvalues, eigenvectors] = mbs.ComputeODE2Eigenvalues()
   #==>eigenvalues contain the eigenvalues of the ODE2 part of the system in the current configuration


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `nMassOscillator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillator.py>`_\  (Ex), \ `nMassOscillatorEigenmodes.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorEigenmodes.py>`_\  (Ex), \ `nMassOscillatorInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorInteractive.py>`_\  (Ex), \ `ANCFBeamEigTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamEigTest.py>`_\  (TM), \ `ANCFBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamTest.py>`_\  (TM), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `computeODE2AEeigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2AEeigenvaluesTest.py>`_\  (TM), \ `computeODE2EigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2EigenvaluesTest.py>`_\  (TM)



.. _sec-mainsystemextensions-computesystemdegreeoffreedom:

Function: ComputeSystemDegreeOfFreedom
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeSystemDegreeOfFreedom <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L621>`__\ (\ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``threshold = 1e-12``\ , \ ``verbose = False``\ , \ ``useSVD = False``\ )

- | \ *function description*\ :
  | compute system DOF numerically, considering GrÃ¼bler-Kutzbach formula as well as redundant constraints; uses numpy matrix rank or singular value decomposition of scipy (useSVD=True)
  | - NOTE that this function is added to MainSystem via Python function ComputeSystemDegreeOfFreedom.
- | \ *input*\ :
  | \ ``simulationSettings``\ : used e.g. for settings regarding numerical differentiation; default settings may be used in most cases
  | \ ``threshold``\ : threshold factor for singular values which estimate the redundant constraints
  | \ ``useSVD``\ : use singular value decomposition directly, also showing SVD values if verbose=True
  | \ ``verbose``\ : if True, it will show the singular values and one may decide if the threshold shall be adapted
- | \ *output*\ :
  | dict; returns dictionary with key words 'degreeOfFreedom', 'redundantConstraints', 'nODE2', 'nODE1', 'nAE', 'nPureAE', where: degreeOfFreedom = the system degree of freedom computed numerically, redundantConstraints=the number of redundant constraints, nODE2=number of ODE2 coordinates, nODE1=number of ODE1 coordinates, nAE=total number of constraints, nPureAE=number of constraints on algebraic variables (e.g., lambda=0) that are not coupled to ODE2 coordinates
- | \ *notes*\ :
  | this approach could possibly fail with special constraints! Currently only works with dense matrices, thus it will be slow for larger systems
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  #
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [6,0,0],
                           initialAngularVelocity = [0,8,0],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1],
                                                                        color=color4orange)])
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreateGenericJoint(bodyNumbers=[oGround, b0], position=[5.5,0,0],
                         constrainedAxes=[1,1,1, 1,0,0],
                         rotationMatrixAxes=RotationMatrixX(0.125*pi), #tilt axes
                         useGlobalFrame=True, axesRadius=0.02, axesLength=0.2)
  #
  mbs.Assemble()
  dof = mbs.ComputeSystemDegreeOfFreedom(verbose=1)['degreeOfFreedom'] #print out details


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Ex), \ `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_\  (Ex), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-createdistancesensorgeometry:

Function: CreateDistanceSensorGeometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateDistanceSensorGeometry <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L161>`__\ (\ ``meshPoints``\ , \ ``meshTrigs``\ , \ ``rigidBodyMarkerIndex``\ , \ ``searchTreeCellSize = [8,8,8]``\ )

- | \ *function description*\ :
  | Add geometry for distance sensor given by points and triangles (point indices) to mbs; use a rigid body marker where the geometry is put on;
  | Creates a GeneralContact for efficient search on background. If you have several sets of points and trigs, first merge them or add them manually to the contact
  | - NOTE that this function is added to MainSystem via Python function CreateDistanceSensorGeometry.
- | \ *input*\ :
  | \ ``meshPoints``\ : list of points (3D), as returned by GraphicsData2PointsAndTrigs()
  | \ ``meshTrigs``\ : list of trigs (3 node indices each), as returned by GraphicsData2PointsAndTrigs()
  | \ ``rigidBodyMarkerIndex``\ : rigid body marker to which the triangles are fixed on (ground or moving object)
  | \ ``searchTreeCellSize``\ : size of search tree (X,Y,Z); use larger values in directions where more triangles are located
- | \ *output*\ :
  | int; returns ngc, which is the number of GeneralContact in mbs, to be used in CreateDistanceSensor(...); keep the gContact as deletion may corrupt data
- | \ *notes*\ :
  | should be used by CreateDistanceSensor(...) and AddLidar(...) for simple initialization of GeneralContact; old name: DistanceSensorSetupGeometry(...)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `laserScannerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/laserScannerTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createdistancesensor:

Function: CreateDistanceSensor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateDistanceSensor <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L194>`__\ (\ ``generalContactIndex``\ , \ ``positionOrMarker``\ , \ ``dirSensor``\ , \ ``minDistance = -1e7``\ , \ ``maxDistance = 1e7``\ , \ ``cylinderRadius = 0``\ , \ ``selectedTypeIndex = exudyn.ContactTypeIndex.IndexEndOfEnumList``\ , \ ``storeInternal = False``\ , \ ``fileName = ''``\ , \ ``measureVelocity = False``\ , \ ``addGraphicsObject = False``\ , \ ``drawDisplaced = True``\ , \ ``color = color4red``\ )

- | \ *function description*\ :
  | Function to create distance sensor based on GeneralContact in mbs; sensor can be either placed on absolute position or attached to rigid body marker; in case of marker, dirSensor is relative to the marker
  | - NOTE that this function is added to MainSystem via Python function CreateDistanceSensor.
- | \ *input*\ :
  | \ ``generalContactIndex``\ : the number of the GeneralContact object in mbs; the index of the GeneralContact object which has been added with last AddGeneralContact(...) command is generalContactIndex=mbs.NumberOfGeneralContacts()-1
  | \ ``positionOrMarker``\ : either a 3D position as list or np.array, or a MarkerIndex with according rigid body marker
  | \ ``dirSensor``\ : the direction (no need to normalize) along which the distance is measured (must not be normalized); in case of marker, the direction is relative to marker orientation if marker contains orientation (BodyRigid, NodeRigid)
  | \ ``minDistance``\ : the minimum distance which is accepted; smaller distance will be ignored
  | \ ``maxDistance``\ : the maximum distance which is accepted; items being at maxDistance or futher are ignored; if no items are found, the function returns maxDistance
  | \ ``cylinderRadius``\ : in case of spheres (selectedTypeIndex=ContactTypeIndex.IndexSpheresMarkerBased), a cylinder can be used which measures the shortest distance at a certain radius (geometrically interpreted as cylinder)
  | \ ``selectedTypeIndex``\ : either this type has default value, meaning that all items in GeneralContact are measured, or there is a specific type index, which is the only type that is considered during measurement
  | \ ``storeInternal``\ : like with any SensorUserFunction, setting to True stores sensor data internally
  | \ ``fileName``\ : if defined, recorded data of SensorUserFunction is written to specified file
  | \ ``measureVelocity``\ : if True, the sensor measures additionally the velocity (component 0=distance, component 1=velocity); velocity is the velocity in direction 'dirSensor' and does not account for changes in geometry, thus it may be different from the time derivative of the distance!
  | \ ``addGraphicsObject``\ : if True, the distance sensor is also visualized graphically in a simplified manner with a red line having the length of dirSensor; NOTE that updates are ONLY performed during computation, not in visualization; for this reason, solutionSettings.sensorsWritePeriod should be accordingly small
  | \ ``drawDisplaced``\ : if True, the red line is drawn backwards such that it moves along the measured surface; if False, the beam is fixed to marker or position
  | \ ``color``\ : optional color for 'laser beam' to be drawn
- | \ *output*\ :
  | SensorIndex; creates sensor and returns according sensor number of SensorUserFunction
- | \ *notes*\ :
  | use generalContactIndex = CreateDistanceSensorGeometry(...) before to create GeneralContact module containing geometry; old name: AddDistanceSensor(...)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM), \ `laserScannerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/laserScannerTest.py>`_\  (TM)



.. _sec-mainsystemextensions-drawsystemgraph:

Function: DrawSystemGraph
^^^^^^^^^^^^^^^^^^^^^^^^^
`DrawSystemGraph <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L849>`__\ (\ ``showLoads = True``\ , \ ``showSensors = True``\ , \ ``useItemNames = False``\ , \ ``useItemTypes = False``\ , \ ``addItemTypeNames = True``\ , \ ``multiLine = True``\ , \ ``fontSizeFactor = 1.``\ , \ ``layoutDistanceFactor = 3.``\ , \ ``layoutIterations = 100``\ , \ ``showLegend = True``\ , \ ``tightLayout = True``\ )

- | \ *function description*\ :
  | helper function which draws system graph of a MainSystem (mbs); several options let adjust the appearance of the graph; the graph visualization uses randomizer, which results in different graphs after every run!
  | - NOTE that this function is added to MainSystem via Python function DrawSystemGraph.
- | \ *input*\ :
  | \ ``showLoads``\ : toggle appearance of loads in mbs
  | \ ``showSensors``\ : toggle appearance of sensors in mbs
  | \ ``useItemNames``\ : if True, object names are shown instead of basic object types (Node, Load, ...)
  | \ ``useItemTypes``\ : if True, object type names (MassPoint, JointRevolute, ...) are shown instead of basic object types (Node, Load, ...); Note that Node, Object, is omitted at the beginning of itemName (as compared to theDoc.pdf); item classes become clear from the legend
  | \ ``addItemTypeNames``\ : if True, type nymes (Node, Load, etc.) are added
  | \ ``multiLine``\ : if True, labels are multiline, improving readability
  | \ ``fontSizeFactor``\ : use this factor to scale fonts, allowing to fit larger graphs on the screen with values < 1
  | \ ``showLegend``\ : shows legend for different item types
  | \ ``layoutDistanceFactor``\ : this factor influences the arrangement of labels; larger distance values lead to circle-like results
  | \ ``layoutIterations``\ : more iterations lead to better arrangement of the layout, but need more time for larger systems (use 1000-10000 to get good results)
  | \ ``tightLayout``\ : if True, uses matplotlib plt.tight_layout() which may raise warning
- | \ *output*\ :
  | [Any, Any, Any]; returns [networkx, G, items] with nx being networkx, G the graph and item what is returned by nx.draw_networkx_labels(...)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Ex), \ `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_\  (Ex), \ `rigidBodyTutorial3withMarkers.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3withMarkers.py>`_\  (Ex), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)


.. _sec-mainsystem-node:


MainSystem: Node
================




This section provides functions for adding, reading and modifying nodes. Nodes are used to define coordinates (unknowns to the static system and degrees of freedom if constraints are not present). Nodes can provide various types of coordinates for second/first order differential equations (ODE2/ODE1), algebraic equations (AE) and for data (history) variables -- which are not providing unknowns in the nonlinear solver but will be solved in an additional nonlinear iteration for e.g. contact, friction or plasticity.

.. code-block:: python
   :linenos:
   
   import exudyn as exu               #EXUDYN package including C++ core part
   from exudyn.itemInterface import * #conversion of data to exudyn dictionaries
   SC = exu.SystemContainer()         #container of systems
   mbs = SC.AddSystem()               #add a new system to work with
   nMP = mbs.AddNode(NodePoint2D(referenceCoordinates=[0,0]))

\ The class **MainSystem** has the following **functions and structures** regarding **nodes**:

* | **AddNode**\ (\ *pyObject*\ ): 
  | add a node with nodeDefinition from Python node class; returns (global) node index (type NodeIndex) of newly added node; use int(nodeIndex) to convert to int, if needed (but not recommended in order not to mix up index types of nodes, objects, markers, ...)
  | *Example*:

  .. code-block:: python

     item = Rigid2D( referenceCoordinates= [1,0.5,0], initialVelocities= [10,0,0]) 
     mbs.AddNode(item) 
     nodeDict = {'nodeType': 'Point', 
     'referenceCoordinates': [1.0, 0.0, 0.0], 
     'initialCoordinates': [0.0, 2.0, 0.0], 
     'name': 'example node'} 
     mbs.AddNode(nodeDict)

* | **GetNodeNumber**\ (\ *nodeName*\ ): 
  | get node's number by name (string)
  | *Example*:

  .. code-block:: python

     n = mbs.GetNodeNumber('example node')

* | **GetNode**\ (\ *nodeNumber*\ ): 
  | get node's dictionary by node number (type NodeIndex)
  | *Example*:

  .. code-block:: python

     nodeDict = mbs.GetNode(0)

* | **ModifyNode**\ (\ *nodeNumber*\ , \ *nodeDict*\ ): 
  | modify node's dictionary by node number (type NodeIndex)
  | *Example*:

  .. code-block:: python

     mbs.ModifyNode(nodeNumber, nodeDict)

* | **GetNodeDefaults**\ (\ *typeName*\ ): 
  | get node's default values for a certain nodeType as (dictionary)
  | *Example*:

  .. code-block:: python

     nodeType = 'Point'
     nodeDict = mbs.GetNodeDefaults(nodeType)

* | **GetNodeOutput**\ (\ *nodeNumber*\ , \ *variableType*\ , \ *configuration*\  = exu.ConfigurationType.Current): 
  | get the ouput of the node specified with the OutputVariableType; output may be scalar or array (e.g. displacement vector)
  | *Example*:

  .. code-block:: python

     mbs.GetNodeOutput(nodeNumber=0, variableType=exu.OutputVariableType.Displacement)

* | **GetNodeODE2Index**\ (\ *nodeNumber*\ ): 
  | get index in the global ODE2 coordinate vector for the first node coordinate of the specified node
  | *Example*:

  .. code-block:: python

     mbs.GetNodeODE2Index(nodeNumber=0)

* | **GetNodeODE1Index**\ (\ *nodeNumber*\ ): 
  | get index in the global ODE1 coordinate vector for the first node coordinate of the specified node
  | *Example*:

  .. code-block:: python

     mbs.GetNodeODE1Index(nodeNumber=0)

* | **GetNodeAEIndex**\ (\ *nodeNumber*\ ): 
  | get index in the global AE coordinate vector for the first node coordinate of the specified node
  | *Example*:

  .. code-block:: python

     mbs.GetNodeAEIndex(nodeNumber=0)

* | **GetNodeParameter**\ (\ *nodeNumber*\ , \ *parameterName*\ ): 
  | get nodes's parameter from node number (type NodeIndex) and parameterName; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix
  | *Example*:

  .. code-block:: python

     mbs.GetNodeParameter(0, 'referenceCoordinates')

* | **SetNodeParameter**\ (\ *nodeNumber*\ , \ *parameterName*\ , \ *value*\ ): 
  | set parameter 'parameterName' of node with node number (type NodeIndex) to value; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix
  | *Example*:

  .. code-block:: python

     mbs.SetNodeParameter(0, 'Vshow', True)




.. _sec-mainsystem-object:


MainSystem: Object
==================




This section provides functions for adding, reading and modifying objects, which can be bodies (mass point, rigid body, finite element, ...), connectors (spring-damper or joint) or general objects. Objects provided terms to the residual of equations resulting from every coordinate given by the nodes. Single-noded objects (e.g.~mass point) provides exactly residual terms for its nodal coordinates. Connectors constrain or penalize two markers, which can be, e.g., position, rigid or coordinate markers. Thus, the dependence of objects is either on the coordinates of the marker-objects/nodes or on nodes which the objects possess themselves.

.. code-block:: python
   :linenos:
   
   import exudyn as exu               #EXUDYN package including C++ core part
   from exudyn.itemInterface import * #conversion of data to exudyn dictionaries
   SC = exu.SystemContainer()         #container of systems
   mbs = SC.AddSystem()               #add a new system to work with
   nMP = mbs.AddNode(NodePoint2D(referenceCoordinates=[0,0]))
   mbs.AddObject(ObjectMassPoint2D(physicsMass=10, nodeNumber=nMP ))

\ The class **MainSystem** has the following **functions and structures** regarding **objects**:

* | **AddObject**\ (\ *pyObject*\ ): 
  | add an object with objectDefinition from Python object class; returns (global) object number (type ObjectIndex) of newly added object
  | *Example*:

  .. code-block:: python

     item = MassPoint(name='heavy object', nodeNumber=0, physicsMass=100) 
     mbs.AddObject(item) 
     objectDict = {'objectType': 'MassPoint', 
     'physicsMass': 10, 
     'nodeNumber': 0, 
     'name': 'example object'} 
     mbs.AddObject(objectDict)

* | **GetObjectNumber**\ (\ *objectName*\ ): 
  | get object's number by name (string)
  | *Example*:

  .. code-block:: python

     n = mbs.GetObjectNumber('heavy object')

* | **GetObject**\ (\ *objectNumber*\ , \ *addGraphicsData*\  = False): 
  | get object's dictionary by object number (type ObjectIndex); NOTE: visualization parameters have a prefix 'V'; in order to also get graphicsData written, use addGraphicsData=True (which is by default False, as it would spoil the information)
  | *Example*:

  .. code-block:: python

     objectDict = mbs.GetObject(0)

* | **ModifyObject**\ (\ *objectNumber*\ , \ *objectDict*\ ): 
  | modify object's dictionary by object number (type ObjectIndex); NOTE: visualization parameters have a prefix 'V'
  | *Example*:

  .. code-block:: python

     mbs.ModifyObject(objectNumber, objectDict)

* | **GetObjectDefaults**\ (\ *typeName*\ ): 
  | get object's default values for a certain objectType as (dictionary)
  | *Example*:

  .. code-block:: python

     objectType = 'MassPoint'
     objectDict = mbs.GetObjectDefaults(objectType)

* | **GetObjectOutput**\ (\ *objectNumber*\ , \ *variableType*\ , \ *configuration*\  = exu.ConfigurationType.Current): 
  | get object's current output variable from object number (type ObjectIndex) and OutputVariableType; for connectors, it can only be computed for exu.ConfigurationType.Current configuration!
* | **GetObjectOutputBody**\ (\ *objectNumber*\ , \ *variableType*\ , \ *localPosition*\  = [0,0,0], \ *configuration*\  = exu.ConfigurationType.Current): 
  | get body's output variable from object number (type ObjectIndex) and OutputVariableType, using the localPosition as defined in the body, and as used in MarkerBody and SensorBody
  | *Example*:

  .. code-block:: python

     u = mbs.GetObjectOutputBody(objectNumber = 1, variableType = exu.OutputVariableType.Position, localPosition=[1,0,0], configuration = exu.ConfigurationType.Initial)

* | **GetObjectOutputSuperElement**\ (\ *objectNumber*\ , \ *variableType*\ , \ *meshNodeNumber*\ , \ *configuration*\  = exu.ConfigurationType.Current): 
  | get output variable from mesh node number of object with type SuperElement (GenericODE2, FFRF, FFRFreduced - CMS) with specific OutputVariableType; the meshNodeNumber is the object's local node number, not the global node number!
  | *Example*:

  .. code-block:: python

     u = mbs.GetObjectOutputSuperElement(objectNumber = 1, variableType = exu.OutputVariableType.Position, meshNodeNumber = 12, configuration = exu.ConfigurationType.Initial)

* | **GetObjectParameter**\ (\ *objectNumber*\ , \ *parameterName*\ ): 
  | get objects's parameter from object number (type ObjectIndex) and parameterName; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix; NOTE that BodyGraphicsData cannot be get or set, use dictionary access instead
  | *Example*:

  .. code-block:: python

     mbs.GetObjectParameter(objectNumber = 0, parameterName = 'nodeNumber')

* | **SetObjectParameter**\ (\ *objectNumber*\ , \ *parameterName*\ , \ *value*\ ): 
  | set parameter 'parameterName' of object with object number (type ObjectIndex) to value;; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix; NOTE that BodyGraphicsData cannot be get or set, use dictionary access instead
  | *Example*:

  .. code-block:: python

     mbs.SetObjectParameter(objectNumber = 0, parameterName = 'Vshow', value=True)




.. _sec-mainsystem-marker:


MainSystem: Marker
==================




This section provides functions for adding, reading and modifying markers. Markers define how to measure primal kinematical quantities on objects or nodes (e.g., position, orientation or coordinates themselves), and how to act on the quantities which are dual to the kinematical quantities (e.g., force, torque and generalized forces). Markers provide unique interfaces for loads, sensors and constraints in order to address these quantities independently of the structure of the object or node (e.g., rigid or flexible body).

.. code-block:: python
   :linenos:
   
   import exudyn as exu               #EXUDYN package including C++ core part
   from exudyn.itemInterface import * #conversion of data to exudyn dictionaries
   SC = exu.SystemContainer()         #container of systems
   mbs = SC.AddSystem()               #add a new system to work with
   nMP = mbs.AddNode(NodePoint2D(referenceCoordinates=[0,0]))
   mbs.AddObject(ObjectMassPoint2D(physicsMass=10, nodeNumber=nMP ))
   mMP = mbs.AddMarker(MarkerNodePosition(nodeNumber = nMP))

\ The class **MainSystem** has the following **functions and structures** regarding **markers**:

* | **AddMarker**\ (\ *pyObject*\ ): 
  | add a marker with markerDefinition from Python marker class; returns (global) marker number (type MarkerIndex) of newly added marker
  | *Example*:

  .. code-block:: python

     item = MarkerNodePosition(name='my marker',nodeNumber=1) 
     mbs.AddMarker(item)
     markerDict = {'markerType': 'NodePosition', 
       'nodeNumber': 0, 
       'name': 'position0'}
     mbs.AddMarker(markerDict)

* | **GetMarkerNumber**\ (\ *markerName*\ ): 
  | get marker's number by name (string)
  | *Example*:

  .. code-block:: python

     n = mbs.GetMarkerNumber('my marker')

* | **GetMarker**\ (\ *markerNumber*\ ): 
  | get marker's dictionary by index
  | *Example*:

  .. code-block:: python

     markerDict = mbs.GetMarker(0)

* | **ModifyMarker**\ (\ *markerNumber*\ , \ *markerDict*\ ): 
  | modify marker's dictionary by index
  | *Example*:

  .. code-block:: python

     mbs.ModifyMarker(markerNumber, markerDict)

* | **GetMarkerDefaults**\ (\ *typeName*\ ): 
  | get marker's default values for a certain markerType as (dictionary)
  | *Example*:

  .. code-block:: python

     markerType = 'NodePosition'
     markerDict = mbs.GetMarkerDefaults(markerType)

* | **GetMarkerParameter**\ (\ *markerNumber*\ , \ *parameterName*\ ): 
  | get markers's parameter from markerNumber and parameterName; parameter names can be found for the specific items in the reference manual
* | **SetMarkerParameter**\ (\ *markerNumber*\ , \ *parameterName*\ , \ *value*\ ): 
  | set parameter 'parameterName' of marker with markerNumber to value; parameter names can be found for the specific items in the reference manual
* | **GetMarkerOutput**\ (\ *markerNumber*\ , \ *variableType*\ , \ *configuration*\  = exu.ConfigurationType.Current): 
  | get the ouput of the marker specified with the OutputVariableType; currently only provides Displacement, Position and Velocity for position based markers, and RotationMatrix, Rotation and AngularVelocity(Local) for markers providing orientation; Coordinates and Coordinates_t available for coordinate markers
  | *Example*:

  .. code-block:: python

     mbs.GetMarkerOutput(markerNumber=0, variableType=exu.OutputVariableType.Position)




.. _sec-mainsystem-load:


MainSystem: Load
================




This section provides functions for adding, reading and modifying operating loads. Loads are used to act on the quantities which are dual to the primal kinematic quantities, such as displacement and rotation. Loads represent, e.g., forces, torques or generalized forces.

.. code-block:: python
   :linenos:
   
   import exudyn as exu               #EXUDYN package including C++ core part
   from exudyn.itemInterface import * #conversion of data to exudyn dictionaries
   SC = exu.SystemContainer()         #container of systems
   mbs = SC.AddSystem()               #add a new system to work with
   nMP = mbs.AddNode(NodePoint2D(referenceCoordinates=[0,0]))
   mbs.AddObject(ObjectMassPoint2D(physicsMass=10, nodeNumber=nMP ))
   mMP = mbs.AddMarker(MarkerNodePosition(nodeNumber = nMP))
   mbs.AddLoad(Force(markerNumber = mMP, loadVector=[0.001,0,0]))

\ The class **MainSystem** has the following **functions and structures** regarding **loads**:

* | **AddLoad**\ (\ *pyObject*\ ): 
  | add a load with loadDefinition from Python load class; returns (global) load number (type LoadIndex) of newly added load
  | *Example*:

  .. code-block:: python

     item = mbs.AddLoad(LoadForceVector(loadVector=[1,0,0], markerNumber=0, name='heavy load')) 
     mbs.AddLoad(item)
     loadDict = {'loadType': 'ForceVector',
       'markerNumber': 0,
       'loadVector': [1.0, 0.0, 0.0],
       'name': 'heavy load'} 
     mbs.AddLoad(loadDict)

* | **GetLoadNumber**\ (\ *loadName*\ ): 
  | get load's number by name (string)
  | *Example*:

  .. code-block:: python

     n = mbs.GetLoadNumber('heavy load')

* | **GetLoad**\ (\ *loadNumber*\ ): 
  | get load's dictionary by index
  | *Example*:

  .. code-block:: python

     loadDict = mbs.GetLoad(0)

* | **ModifyLoad**\ (\ *loadNumber*\ , \ *loadDict*\ ): 
  | modify load's dictionary by index
  | *Example*:

  .. code-block:: python

     mbs.ModifyLoad(loadNumber, loadDict)

* | **GetLoadDefaults**\ (\ *typeName*\ ): 
  | get load's default values for a certain loadType as (dictionary)
  | *Example*:

  .. code-block:: python

     loadType = 'ForceVector'
     loadDict = mbs.GetLoadDefaults(loadType)

* | **GetLoadValues**\ (\ *loadNumber*\ ): 
  | Get current load values, specifically if user-defined loads are used; can be scalar or vector-valued return value
* | **GetLoadParameter**\ (\ *loadNumber*\ , \ *parameterName*\ ): 
  | get loads's parameter from loadNumber and parameterName; parameter names can be found for the specific items in the reference manual
* | **SetLoadParameter**\ (\ *loadNumber*\ , \ *parameterName*\ , \ *value*\ ): 
  | set parameter 'parameterName' of load with loadNumber to value; parameter names can be found for the specific items in the reference manual



.. _sec-mainsystem-sensor:


MainSystem: Sensor
==================




This section provides functions for adding, reading and modifying operating sensors. Sensors are used to measure information in nodes, objects, markers, and loads for output in a file.

.. code-block:: python
   :linenos:
   
   import exudyn as exu               #EXUDYN package including C++ core part
   from exudyn.itemInterface import * #conversion of data to exudyn dictionaries
   SC = exu.SystemContainer()         #container of systems
   mbs = SC.AddSystem()               #add a new system to work with
   nMP = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,0]))
   mbs.AddObject(ObjectMassPoint(physicsMass=10, nodeNumber=nMP ))
   mMP = mbs.AddMarker(MarkerNodePosition(nodeNumber = nMP))
   mbs.AddLoad(Force(markerNumber = mMP, loadVector=[2,0,5]))
   sMP = mbs.AddSensor(SensorNode(nodeNumber=nMP, storeInternal=True,
                                  outputVariableType=exu.OutputVariableType.Position))
   mbs.Assemble()
   exu.SolveDynamic(mbs, exu.SimulationSettings())
   from exudyn.plot import PlotSensor
   PlotSensor(mbs, sMP, components=[0,1,2])

\ The class **MainSystem** has the following **functions and structures** regarding **sensors**:

* | **AddSensor**\ (\ *pyObject*\ ): 
  | add a sensor with sensor definition from Python sensor class; returns (global) sensor number (type SensorIndex) of newly added sensor
  | *Example*:

  .. code-block:: python

     item = mbs.AddSensor(SensorNode(sensorType= exu.SensorType.Node, nodeNumber=0, name='test sensor')) 
     mbs.AddSensor(item)
     sensorDict = {'sensorType': 'Node',
       'nodeNumber': 0,
       'fileName': 'sensor.txt',
       'name': 'test sensor'} 
     mbs.AddSensor(sensorDict)

* | **GetSensorNumber**\ (\ *sensorName*\ ): 
  | get sensor's number by name (string)
  | *Example*:

  .. code-block:: python

     n = mbs.GetSensorNumber('test sensor')

* | **GetSensor**\ (\ *sensorNumber*\ ): 
  | get sensor's dictionary by index
  | *Example*:

  .. code-block:: python

     sensorDict = mbs.GetSensor(0)

* | **ModifySensor**\ (\ *sensorNumber*\ , \ *sensorDict*\ ): 
  | modify sensor's dictionary by index
  | *Example*:

  .. code-block:: python

     mbs.ModifySensor(sensorNumber, sensorDict)

* | **GetSensorDefaults**\ (\ *typeName*\ ): 
  | get sensor's default values for a certain sensorType as (dictionary)
  | *Example*:

  .. code-block:: python

     sensorType = 'Node'
     sensorDict = mbs.GetSensorDefaults(sensorType)

* | **GetSensorValues**\ (\ *sensorNumber*\ , \ *configuration*\  = exu.ConfigurationType.Current): 
  | get sensors's values for configuration; can be a scalar or vector-valued return value!
* | **GetSensorStoredData**\ (\ *sensorNumber*\ ): 
  | get sensors's internally stored data as matrix (all time points stored); rows are containing time and sensor values as obtained by sensor (e.g., time, and x, y, and z value of position)
* | **GetSensorParameter**\ (\ *sensorNumber*\ , \ *parameterName*\ ): 
  | get sensors's parameter from sensorNumber and parameterName; parameter names can be found for the specific items in the reference manual
* | **SetSensorParameter**\ (\ *sensorNumber*\ , \ *parameterName*\ , \ *value*\ ): 
  | set parameter 'parameterName' of sensor with sensorNumber to value; parameter names can be found for the specific items in the reference manual


