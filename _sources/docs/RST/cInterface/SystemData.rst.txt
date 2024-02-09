
.. _sec-mbs-systemdata:


**********
SystemData
**********




This is the data structure of a system which contains Objects (bodies/constraints/...), Nodes, Markers and Loads. The SystemData structure allows advanced access to this data, which HAS TO BE USED WITH CARE, as unexpected results and system crash might happen.

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
   mbs.Assemble()
   exu.SolveDynamic(mbs, exu.SimulationSettings())
   
   #obtain current ODE2 system vector (e.g. after static simulation finished):
   u = mbs.systemData.GetODE2Coordinates()
   #set initial ODE2 vector for next simulation:
   mbs.systemData.SetODE2Coordinates(coordinates=u,
                  configuration=exu.ConfigurationType.Initial)
   #print detailed information on items:
   mbs.systemData.Info()
   #print LTG lists for objects and loads:
   mbs.systemData.InfoLTG()

\ The class **MainSystemData** has the following **functions and structures**:

* | **NumberOfLoads**\ (): 
  | return number of loads in system
  | *Example*:

  .. code-block:: python

     print(mbs.systemData.NumberOfLoads())

* | **NumberOfMarkers**\ (): 
  | return number of markers in system
  | *Example*:

  .. code-block:: python

     print(mbs.systemData.NumberOfMarkers())

* | **NumberOfNodes**\ (): 
  | return number of nodes in system
  | *Example*:

  .. code-block:: python

     print(mbs.systemData.NumberOfNodes())

* | **NumberOfObjects**\ (): 
  | return number of objects in system
  | *Example*:

  .. code-block:: python

     print(mbs.systemData.NumberOfObjects())

* | **NumberOfSensors**\ (): 
  | return number of sensors in system
  | *Example*:

  .. code-block:: python

     print(mbs.systemData.NumberOfSensors())

* | **ODE2Size**\ (\ *configurationType*\  = exu.ConfigurationType.Current): 
  | get size of ODE2 coordinate vector for given configuration (only works correctly after mbs.Assemble() )
  | *Example*:

  .. code-block:: python

     print('ODE2 size=',mbs.systemData.ODE2Size())

* | **ODE1Size**\ (\ *configurationType*\  = exu.ConfigurationType.Current): 
  | get size of ODE1 coordinate vector for given configuration (only works correctly after mbs.Assemble() )
  | *Example*:

  .. code-block:: python

     print('ODE1 size=',mbs.systemData.ODE1Size())

* | **AEsize**\ (\ *configurationType*\  = exu.ConfigurationType.Current): 
  | get size of AE coordinate vector for given configuration (only works correctly after mbs.Assemble() )
  | *Example*:

  .. code-block:: python

     print('AE size=',mbs.systemData.AEsize())

* | **DataSize**\ (\ *configurationType*\  = exu.ConfigurationType.Current): 
  | get size of Data coordinate vector for given configuration (only works correctly after mbs.Assemble() )
  | *Example*:

  .. code-block:: python

     print('Data size=',mbs.systemData.DataSize())

* | **SystemSize**\ (\ *configurationType*\  = exu.ConfigurationType.Current): 
  | get size of System coordinate vector for given configuration (only works correctly after mbs.Assemble() )
  | *Example*:

  .. code-block:: python

     print('System size=',mbs.systemData.SystemSize())

* | **GetTime**\ (\ *configurationType*\  = exu.ConfigurationType.Current): 
  | get configuration dependent time.
  | *Example*:

  .. code-block:: python

     mbs.systemData.GetTime(exu.ConfigurationType.Initial)

* | **SetTime**\ (\ *newTime*\ , \ *configurationType*\  = exu.ConfigurationType.Current): 
  | set configuration dependent time; use this access with care, e.g. in user-defined solvers.
  | *Example*:

  .. code-block:: python

     mbs.systemData.SetTime(10., exu.ConfigurationType.Initial)

* | **AddODE2LoadDependencies**\ (\ *loadNumber*\ , \ *globalODE2coordinates*\ ): 
  | advanced function for adding special dependencies of loads onto ODE2 coordinates, taking a list / numpy array of global ODE2 coordinates; this function needs to be called after Assemble() and needs to contain ODE2 coordinate indices; this list only affects implicit or static solvers if numericalDifferentiation.loadsJacobian=True; in this case, it may greatly improve convergence if loads with user functions depend on some system states, such as in a load with feedback control loop; the additional dependencies are not required, if doSystemWideDifferentiation=True, however the latter option being much less efficient!
  | *Example*:

  .. code-block:: python

     mbs.systemData.AddODE2LoadDependencies(0,[0,1,2])
     #add dependency of load 5 onto node 2 coordinates:
     nodeLTG2 = mbs.systemData.GetNodeLTGODE2(2)
     mbs.systemData.AddODE2LoadDependencies(5,nodeLTG2)

* | **Info**\ (): 
  | print detailed information on every item; for short information use print(mbs)
  | *Example*:

  .. code-block:: python

     mbs.systemData.Info()

* | **InfoLTG**\ (): 
  | print LTG information of objects and load dependencies
  | *Example*:

  .. code-block:: python

     mbs.systemData.InfoLTG()




.. _sec-mbs-systemdata-coordinates:


SystemData: Access coordinates
==============================




This section provides access functions to global coordinate vectors. Assigning invalid values or using wrong vector size might lead to system crash and unexpected results.

\ The class **MainSystemData** has the following **functions and structures** regarding **coordinate access**:

* | **GetODE2Coordinates**\ (\ *configuration*\  = exu.ConfigurationType.Current): 
  | get ODE2 system coordinates (displacements) for given configuration (default: exu.Configuration.Current)
  | *Example*:

  .. code-block:: python

     uCurrent = mbs.systemData.GetODE2Coordinates()

* | **SetODE2Coordinates**\ (\ *coordinates*\ , \ *configuration*\  = exu.ConfigurationType.Current): 
  | set ODE2 system coordinates (displacements) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!
  | *Example*:

  .. code-block:: python

     mbs.systemData.SetODE2Coordinates(uCurrent)

* | **GetODE2Coordinates\_t**\ (\ *configuration*\  = exu.ConfigurationType.Current): 
  | get ODE2 system coordinates (velocities) for given configuration (default: exu.Configuration.Current)
  | *Example*:

  .. code-block:: python

     vCurrent = mbs.systemData.GetODE2Coordinates_t()

* | **SetODE2Coordinates\_t**\ (\ *coordinates*\ , \ *configuration*\  = exu.ConfigurationType.Current): 
  | set ODE2 system coordinates (velocities) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!
  | *Example*:

  .. code-block:: python

     mbs.systemData.SetODE2Coordinates_t(vCurrent)

* | **GetODE2Coordinates\_tt**\ (\ *configuration*\  = exu.ConfigurationType.Current): 
  | get ODE2 system coordinates (accelerations) for given configuration (default: exu.Configuration.Current)
  | *Example*:

  .. code-block:: python

     vCurrent = mbs.systemData.GetODE2Coordinates_tt()

* | **SetODE2Coordinates\_tt**\ (\ *coordinates*\ , \ *configuration*\  = exu.ConfigurationType.Current): 
  | set ODE2 system coordinates (accelerations) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!
  | *Example*:

  .. code-block:: python

     mbs.systemData.SetODE2Coordinates_tt(aCurrent)

* | **GetODE1Coordinates**\ (\ *configuration*\  = exu.ConfigurationType.Current): 
  | get ODE1 system coordinates (displacements) for given configuration (default: exu.Configuration.Current)
  | *Example*:

  .. code-block:: python

     qCurrent = mbs.systemData.GetODE1Coordinates()

* | **SetODE1Coordinates**\ (\ *coordinates*\ , \ *configuration*\  = exu.ConfigurationType.Current): 
  | set ODE1 system coordinates (velocities) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!
  | *Example*:

  .. code-block:: python

     mbs.systemData.SetODE1Coordinates_t(qCurrent)

* | **GetODE1Coordinates\_t**\ (\ *configuration*\  = exu.ConfigurationType.Current): 
  | get ODE1 system coordinates (velocities) for given configuration (default: exu.Configuration.Current)
  | *Example*:

  .. code-block:: python

     qCurrent = mbs.systemData.GetODE1Coordinates_t()

* | **SetODE1Coordinates\_t**\ (\ *coordinates*\ , \ *configuration*\  = exu.ConfigurationType.Current): 
  | set ODE1 system coordinates (displacements) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!
  | *Example*:

  .. code-block:: python

     mbs.systemData.SetODE1Coordinates(qCurrent)

* | **GetAECoordinates**\ (\ *configuration*\  = exu.ConfigurationType.Current): 
  | get algebraic equations (AE) system coordinates for given configuration (default: exu.Configuration.Current)
  | *Example*:

  .. code-block:: python

     lambdaCurrent = mbs.systemData.GetAECoordinates()

* | **SetAECoordinates**\ (\ *coordinates*\ , \ *configuration*\  = exu.ConfigurationType.Current): 
  | set algebraic equations (AE) system coordinates for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!
  | *Example*:

  .. code-block:: python

     mbs.systemData.SetAECoordinates(lambdaCurrent)

* | **GetDataCoordinates**\ (\ *configuration*\  = exu.ConfigurationType.Current): 
  | get system data coordinates for given configuration (default: exu.Configuration.Current)
  | *Example*:

  .. code-block:: python

     dataCurrent = mbs.systemData.GetDataCoordinates()

* | **SetDataCoordinates**\ (\ *coordinates*\ , \ *configuration*\  = exu.ConfigurationType.Current): 
  | set system data coordinates for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!
  | *Example*:

  .. code-block:: python

     mbs.systemData.SetDataCoordinates(dataCurrent)

* | **GetSystemState**\ (\ *configuration*\  = exu.ConfigurationType.Current): 
  | get system state for given configuration (default: exu.Configuration.Current); state vectors do not include the non-state derivatives ODE1_t and ODE2_tt and the time; function is copying data - not highly efficient; format of pyList: [ODE2Coords, ODE2Coords_t, ODE1Coords, AEcoords, dataCoords]
  | *Example*:

  .. code-block:: python

     sysStateList = mbs.systemData.GetSystemState()

* | **SetSystemState**\ (\ *systemStateList*\ , \ *configuration*\  = exu.ConfigurationType.Current): 
  | set system data coordinates for given configuration (default: exu.Configuration.Current); invalid list of vectors / vector size may lead to system crash; write access to state vectors (but not the non-state derivatives ODE1_t and ODE2_tt and the time); function is copying data - not highly efficient; format of pyList: [ODE2Coords, ODE2Coords_t, ODE1Coords, AEcoords, dataCoords]
  | *Example*:

  .. code-block:: python

     mbs.systemData.SetSystemState(sysStateList, configuration = exu.ConfigurationType.Initial)




.. _sec-systemdata-objectltg:


SystemData: Get object LTG coordinate mappings
==============================================




This section provides access functions the \ :ref:`LTG <LTG>`\ -lists for every object (body, constraint, ...) in the system. For details on the \ :ref:`LTG <LTG>`\  mapping, see Section :ref:`sec-overview-ltgmapping`\ .

\ The class **MainSystemData** has the following **functions and structures** regarding **object LTG coordinate mappings**:

* | **GetObjectLTGODE2**\ (\ *objectNumber*\ ): 
  | get object local-to-global coordinate mapping (list of global coordinate indices) for ODE2 coordinates; only available after Assemble()
  | *Example*:

  .. code-block:: python

     ltgObject4 = mbs.systemData.GetObjectLTGODE2(4)

* | **GetObjectLTGODE1**\ (\ *objectNumber*\ ): 
  | get object local-to-global coordinate mapping (list of global coordinate indices) for ODE1 coordinates; only available after Assemble()
  | *Example*:

  .. code-block:: python

     ltgObject4 = mbs.systemData.GetObjectLTGODE1(4)

* | **GetObjectLTGAE**\ (\ *objectNumber*\ ): 
  | get object local-to-global coordinate mapping (list of global coordinate indices) for algebraic equations (AE) coordinates; only available after Assemble()
  | *Example*:

  .. code-block:: python

     ltgObject4 = mbs.systemData.GetObjectLTGODE2(4)

* | **GetObjectLTGData**\ (\ *objectNumber*\ ): 
  | get object local-to-global coordinate mapping (list of global coordinate indices) for data coordinates; only available after Assemble()
  | *Example*:

  .. code-block:: python

     ltgObject4 = mbs.systemData.GetObjectLTGData(4)

* | **GetNodeLTGODE2**\ (\ *nodeNumber*\ ): 
  | get node local-to-global coordinate mapping (list of global coordinate indices) for ODE2 coordinates; only available after Assemble()
  | *Example*:

  .. code-block:: python

     ltgNode4 = mbs.systemData.GetNodeLTGODE2(4)

* | **GetNodeLTGODE1**\ (\ *nodeNumber*\ ): 
  | get node local-to-global coordinate mapping (list of global coordinate indices) for ODE1 coordinates; only available after Assemble()
  | *Example*:

  .. code-block:: python

     ltgNode4 = mbs.systemData.GetNodeLTGODE1(4)

* | **GetNodeLTGAE**\ (\ *nodeNumber*\ ): 
  | get node local-to-global coordinate mapping (list of global coordinate indices) for AE coordinates; only available after Assemble()
  | *Example*:

  .. code-block:: python

     ltgNode4 = mbs.systemData.GetNodeLTGAE(4)

* | **GetNodeLTGData**\ (\ *nodeNumber*\ ): 
  | get node local-to-global coordinate mapping (list of global coordinate indices) for Data coordinates; only available after Assemble()
  | *Example*:

  .. code-block:: python

     ltgNode4 = mbs.systemData.GetNodeLTGData(4)



