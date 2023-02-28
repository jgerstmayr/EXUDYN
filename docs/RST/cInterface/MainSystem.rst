
**********
MainSystem
**********




This is the structure which defines a (multibody) system. In C++, there is a MainSystem (links to Python) and a System (computational part). For that reason, the name is MainSystem on the Python side, but it is often just called 'system'. For compatibility, it is recommended to denote the variable holding this system as mbs, the multibody dynamics system. It can be created, visualized and computed. Use the following functions for system manipulation.

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
* | **SetRenderEngineStopFlag**\ (): 
  | set the current stop simulation flag; set to False, in order to continue a previously user-interrupted simulation
* | **ActivateRendering**\ (\ *flag*\  = True): 
  | activate (flag=True) or deactivate (flag=False) rendering for this system
* | **SetPreStepUserFunction**\ (): 
  | Sets a user function PreStepUserFunction(mbs, t) executed at beginning of every computation step; in normal case return True; return False to stop simulation after current step
  | *Example*:

  .. code-block:: python

     def PreStepUserFunction(mbs, t):
         print(mbs.systemData.NumberOfNodes())
         if(t>1): 
             return False 
         return True 
      mbs.SetPreStepUserFunction(PreStepUserFunction)

* | **SetPostNewtonUserFunction**\ (): 
  | Sets a user function PostNewtonUserFunction(mbs, t) executed after successful Newton iteration in implicit or static solvers and after step update of explicit solvers, but BEFORE PostNewton functions are called by the solver; function returns list [discontinuousError, recommendedStepSize], containing a error of the PostNewtonStep, which is compared to [solver].discontinuous.iterationTolerance. The recommendedStepSize shall be negative, if no recommendation is given, 0 in order to enforce minimum step size or a specific value to which the current step size will be reduced and the step will be repeated; use this function, e.g., to reduce step size after impact or change of data variables
  | *Example*:

  .. code-block:: python

     def PostNewtonUserFunction(mbs, t):
         if(t>1): 
             return [0, 1e-6] 
         return [0,0] 
      mbs.SetPostNewtonUserFunction(PostNewtonUserFunction)

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



.. _sec-mainsystem-node:


MainSystem: Node
================




This section provides functions for adding, reading and modifying nodes. Nodes are used to define coordinates (unknowns to the static system and degrees of freedom if constraints are not present). Nodes can provide various types of coordinates for second/first order differential equations (ODE2/ODE1), algebraic equations (AE) and for data (history) variables -- which are not providing unknowns in the nonlinear solver but will be solved in an additional nonlinear iteration for e.g. contact, friction or plasticity.

.. code-block:: python
   
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

* | **GetNodeOutput**\ (\ *nodeNumber*\ , \ *variableType*\ , \ *configuration*\  = ConfigurationType.Current): 
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

* | **GetObjectOutput**\ (\ *objectNumber*\ , \ *variableType*\ , \ *configuration*\  = ConfigurationType.Current): 
  | get object's current output variable from object number (type ObjectIndex) and OutputVariableType; for connectors, it can only be computed for exu.ConfigurationType.Current configuration!
* | **GetObjectOutputBody**\ (\ *objectNumber*\ , \ *variableType*\ , \ *localPosition*\  = [0,0,0], \ *configuration*\  = ConfigurationType.Current): 
  | get body's output variable from object number (type ObjectIndex) and OutputVariableType, using the localPosition as defined in the body, and as used in MarkerBody and SensorBody
  | *Example*:

  .. code-block:: python

     u = mbs.GetObjectOutputBody(objectNumber = 1, variableType = exu.OutputVariableType.Position, localPosition=[1,0,0], configuration = exu.ConfigurationType.Initial)

* | **GetObjectOutputSuperElement**\ (\ *objectNumber*\ , \ *variableType*\ , \ *meshNodeNumber*\ , \ *configuration*\  = ConfigurationType.Current): 
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
* | **GetMarkerOutput**\ (\ *markerNumber*\ , \ *variableType*\ , \ *configuration*\  = ConfigurationType.Current): 
  | get the ouput of the marker specified with the OutputVariableType; currently only provides Displacement, Position and Velocity for position based markers, and RotationMatrix, Rotation and AngularVelocity(Local) for markers providing orientation; Coordinates and Coordinates_t available for coordinate markers
  | *Example*:

  .. code-block:: python

     mbs.GetMarkerOutput(markerNumber=0, variableType=exu.OutputVariableType.Position)




.. _sec-mainsystem-load:


MainSystem: Load
================




This section provides functions for adding, reading and modifying operating loads. Loads are used to act on the quantities which are dual to the primal kinematic quantities, such as displacement and rotation. Loads represent, e.g., forces, torques or generalized forces.

.. code-block:: python
   
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

* | **GetSensorValues**\ (\ *sensorNumber*\ , \ *configuration*\  = ConfigurationType.Current): 
  | get sensors's values for configuration; can be a scalar or vector-valued return value!
* | **GetSensorStoredData**\ (\ *sensorNumber*\ ): 
  | get sensors's internally stored data as matrix (all time points stored); rows are containing time and sensor values as obtained by sensor (e.g., time, and x, y, and z value of position)
* | **GetSensorParameter**\ (\ *sensorNumber*\ , \ *parameterName*\ ): 
  | get sensors's parameter from sensorNumber and parameterName; parameter names can be found for the specific items in the reference manual
* | **SetSensorParameter**\ (\ *sensorNumber*\ , \ *parameterName*\ , \ *value*\ ): 
  | set parameter 'parameterName' of sensor with sensorNumber to value; parameter names can be found for the specific items in the reference manual


