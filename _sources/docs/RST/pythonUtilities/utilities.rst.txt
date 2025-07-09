
.. _sec-module-utilities:

Module: utilities
=================

Basic support functions for simpler creation of Exudyn models.
Advanced functions for loading and animating solutions and for drawing a graph of the mbs system.
This library requires numpy (as well as time and copy)

- Author:    Johannes Gerstmayr 
- Date:      2019-07-26 (created) 


.. _sec-utilities-getothermarker:

Function: GetOtherMarker
^^^^^^^^^^^^^^^^^^^^^^^^
`GetOtherMarker <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L85>`__\ (\ ``mbs``\ , \ ``bodyNumber``\ , \ ``existingMarker``\ , \ ``show = True``\ )

- | \ *function description*\ :
  | creates a new marker for body with bodyNumber using another marker existingMarker, such that the new marker has the same reference position as the existing marker, working for MarkerBodyPosition (no rotations included); this alleviates creation of markers and calculation of localPosition
- | \ *input*\ :
  | \ ``mbs``\ : multibody system where new marker is added to
  | \ ``bodyNumber``\ : body where new marker shall be attached to
  | \ ``existingMarker``\ : marker number which serves as a reference
  | \ ``show``\ : if True, marker is shown
- | \ *output*\ :
  | returns marker number of new marker
- | \ *example*\ :

.. code-block:: python

  #oBody0 = mbs.CreateRigidBody(...)
  #oBody1 = mbs.CreateRigidBody(...)
  marker0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oBody0,localPosition=[1,0,0]))
  #create joint from one marker (with rotation) and other body
  mbs.AddObject(SphericalJoint(markerNumbers=[marker0, GetOtherMarker(mbs, oBody1, marker0)]))


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `NGsolveFFRF.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveFFRF.py>`_\  (Ex)



----


.. _sec-utilities-getjointargs:

Function: GetJointArgs
^^^^^^^^^^^^^^^^^^^^^^
`GetJointArgs <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L127>`__\ (\ ``mbs``\ , \ ``markerNumber0 = None``\ , \ ``markerNumber1 = None``\ , \ ``rotationMarker0 = None``\ , \ ``rotationMarker1 = None``\ , \ ``bodyNumber0 = None``\ , \ ``bodyNumber1 = None``\ )

- | \ *function description*\ :
  | creates input args for joints, based on an exiting marker (markerNumber, may be rigid or flex body), with optional existing rotationMarker and uses another rigid body (given as bodyNumber) to create a new MarkerBodyRigid and rotationMarker; this alleviates creation of joint args, see the example; inputs are either markerNumber0 [, rotationMarker0], bodyNumber1 OR markerNumber1 [, rotationMarker1], bodyNumber0
- | \ *input*\ :
  | \ ``mbs``\ : multibody system where new marker is added to
  | \ ``markerNumber0``\ : markerNumber of existing rigid body marker
  | \ ``markerNumber1``\ : markerNumber of existing rigid body marker
  | \ ``rotationMarker0``\ : joint marker rotation matrix for markerNumber0 (must be MarkerBodyRigid)
  | \ ``rotationMarker1``\ : joint marker rotation matrix for markerNumber1 (must be MarkerBodyRigid)
  | \ ``bodyNumber0``\ : existing body used to create new marker
  | \ ``bodyNumber1``\ : existing body used to create new marker
- | \ *output*\ :
  | returns dict with 'markerNumbers' list, 'rotationMarker0' and 'rotationMarker1', ready to be used as args
- | \ *example*\ :

.. code-block:: python

  #oBody0 = mbs.CreateRigidBody(...)
  #oBody1 = mbs.CreateRigidBody(...)
  marker0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oBody0,localPosition=[1,0,0]))
  rotM0 = RotationMatrixX(0.5*pi)
  #create joint from one marker (with rotation) and other body
  mbs.AddObject(RevoluteJointZ(**GetJointArgs(mbs, markerNumber0=marker0,
                                              rotationMarker0=rotM0,
                                              bodyNumber1=oBody1)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `jointArgsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/jointArgsTest.py>`_\  (TM)



----


.. _sec-utilities-showonlyobjects:

Function: ShowOnlyObjects
^^^^^^^^^^^^^^^^^^^^^^^^^
`ShowOnlyObjects <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L186>`__\ (\ ``mbs``\ , \ ``objectNumbers = []``\ , \ ``showOthers = False``\ )

- | \ *function description*\ :
  | function to hide all objects in mbs except for those listed in objectNumbers
- | \ *input*\ :
  | \ ``mbs``\ : mbs containing object
  | \ ``objectNumbers``\ : integer object number or list of object numbers to be shown; if empty list [], then all objects are shown
  | \ ``showOthers``\ : if True, then all other objects are shown again
- | \ *output*\ :
  | changes all colors in mbs, which is NOT reversible



----


.. _sec-utilities-highlightitem:

Function: HighlightItem
^^^^^^^^^^^^^^^^^^^^^^^
`HighlightItem <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L209>`__\ (\ ``SC``\ , \ ``mbs``\ , \ ``itemNumber``\ , \ ``itemType = exudyn.ItemType.Object``\ , \ ``showNumbers = True``\ )

- | \ *function description*\ :
  | highlight a certain item with number itemNumber; set itemNumber to -1 to show again all objects
- | \ *input*\ :
  | \ ``mbs``\ : mbs containing object
  | \ ``itemNumbers``\ : integer object/node/etc number to be highlighted
  | \ ``itemType``\ : type of items to be highlighted
  | \ ``showNumbers``\ : if True, then the numbers of these items are shown



----


.. _sec-utilities---ufsensordistance:

Function: __UFsensorDistance
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`__UFsensorDistance <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L248>`__\ (\ ``mbs``\ , \ ``t``\ , \ ``sensorNumbers``\ , \ ``factors``\ , \ ``configuration``\ )

- | \ *function description*\ :
  | internal function used for CreateDistanceSensor



----

Function: CreateDistanceSensorGeometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateDistanceSensorGeometry <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L306>`__\ (\ ``mbs``\ , \ ``meshPoints``\ , \ ``meshTrigs``\ , \ ``rigidBodyMarkerIndex``\ , \ ``searchTreeCellSize = [8,8,8]``\ )


- | **NOTE**\ : this function is directly available in MainSystem (mbs); it should be directly called as mbs.CreateDistanceSensorGeometry(...). For description of the interface, see the MainSystem Python extensions,  :ref:`sec-mainsystemextensions-createdistancesensorgeometry`\ 



----

Function: CreateDistanceSensor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateDistanceSensor <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L339>`__\ (\ ``mbs``\ , \ ``generalContactIndex``\ , \ ``positionOrMarker``\ , \ ``dirSensor``\ , \ ``minDistance = -1e7``\ , \ ``maxDistance = 1e7``\ , \ ``cylinderRadius = 0``\ , \ ``selectedTypeIndex = exudyn.ContactTypeIndex.IndexEndOfEnumList``\ , \ ``storeInternal = False``\ , \ ``fileName = ''``\ , \ ``measureVelocity = False``\ , \ ``addGraphicsObject = False``\ , \ ``drawDisplaced = True``\ , \ ``color = exudyn.graphics.color.red``\ )


- | **NOTE**\ : this function is directly available in MainSystem (mbs); it should be directly called as mbs.CreateDistanceSensor(...). For description of the interface, see the MainSystem Python extensions,  :ref:`sec-mainsystemextensions-createdistancesensor`\ 



----


.. _sec-utilities-ufsensorrecord:

Function: UFsensorRecord
^^^^^^^^^^^^^^^^^^^^^^^^
`UFsensorRecord <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L388>`__\ (\ ``mbs``\ , \ ``t``\ , \ ``sensorNumbers``\ , \ ``factors``\ , \ ``configuration``\ )

- | \ *function description*\ :
  | DEPRECATED: Internal SensorUserFunction, used in function AddSensorRecorder
- | \ *notes*\ :
  | Warning: this method is DEPRECATED, use storeInternal in Sensors, which is much more performant; Note, that a sensor usually just passes through values of an existing sensor, while recording the values to a numpy array row-wise (time in first column, data in remaining columns)



----


.. _sec-utilities-addsensorrecorder:

Function: AddSensorRecorder
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`AddSensorRecorder <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L409>`__\ (\ ``mbs``\ , \ ``sensorNumber``\ , \ ``endTime``\ , \ ``sensorsWritePeriod``\ , \ ``sensorOutputSize = 3``\ )

- | \ *function description*\ :
  | DEPRECATED: Add a SensorUserFunction object in order to record sensor output internally; this avoids creation of files for sensors, which can speedup and simplify evaluation in ParameterVariation and GeneticOptimization; values are stored internally in mbs.variables['sensorRecord'+str(sensorNumber)] where sensorNumber is the mbs sensor number
- | \ *input*\ :
  | \ ``mbs``\ : mbs containing object
  | \ ``sensorNumber``\ : integer sensor number to be recorded
  | \ ``endTime``\ : end time of simulation, as given in simulationSettings.timeIntegration.endTime
  | \ ``sensorsWritePeriod``\ : as given in simulationSettings.solutionSettings.sensorsWritePeriod
  | \ ``sensorOutputSize``\ : size of sensor data: 3 for Displacement, Position, etc. sensors; may be larger for RotationMatrix or Coordinates sensors; check this size by calling mbs.GetSensorValues(sensorNumber)
- | \ *output*\ :
  | adds an according SensorUserFunction sensor to mbs; returns new sensor number; during initialization a new numpy array is allocated in  mbs.variables['sensorRecord'+str(sensorNumber)] and the information is written row-wise: [time, sensorValue1, sensorValue2, ...]
- | \ *notes*\ :
  | Warning: this method is DEPRECATED, use storeInternal in Sensors, which is much more performant; Note, that a sensor usually just passes through values of an existing sensor, while recording the values to a numpy array row-wise (time in first column, data in remaining columns)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ComputeSensitivitiesExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ComputeSensitivitiesExample.py>`_\  (Ex)



----


.. _sec-utilities-loadsolutionfile:

Function: LoadSolutionFile
^^^^^^^^^^^^^^^^^^^^^^^^^^
`LoadSolutionFile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L436>`__\ (\ ``fileName``\ , \ ``safeMode = False``\ , \ ``maxRows = -1``\ , \ ``verbose = True``\ , \ ``hasHeader = True``\ )

- | \ *function description*\ :
  | read coordinates solution file (exported during static or dynamic simulation with option exu.SimulationSettings().solutionSettings.coordinatesSolutionFileName='...') into dictionary:
- | \ *input*\ :
  | \ ``fileName``\ : string containing directory and filename of stored coordinatesSolutionFile
  | \ ``saveMode``\ : if True, it loads lines directly to load inconsistent lines as well; use this for huge files (>2GB); is slower but needs less memory!
  | \ ``verbose``\ : if True, some information is written when importing file (use for huge files to track progress)
  | \ ``maxRows``\ : maximum number of data rows loaded, if saveMode=True; use this for huge files to reduce loading time; set -1 to load all rows
  | \ ``hasHeader``\ : set to False, if file is expected to have no header; if False, then some error checks related to file header are not performed
- | \ *output*\ :
  | dictionary with 'data': the matrix of stored solution vectors, 'columnsExported': a list with integer values showing the exported sizes [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData], 'nColumns': the number of data columns and 'nRows': the number of data rows

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Ex), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Ex), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Ex), \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TM), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TM)



----


.. _sec-utilities-numpyint8arraytostring:

Function: NumpyInt8ArrayToString
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`NumpyInt8ArrayToString <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L567>`__\ (\ ``npArray``\ )

- | \ *function description*\ :
  | simple conversion of int8 arrays into strings (not highly efficient, so use only for short strings)



----


.. _sec-utilities-binaryreadindex:

Function: BinaryReadIndex
^^^^^^^^^^^^^^^^^^^^^^^^^
`BinaryReadIndex <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L574>`__\ (\ ``file``\ , \ ``intType``\ )

- | \ *function description*\ :
  | read single Index from current file position in binary solution file



----


.. _sec-utilities-binaryreadreal:

Function: BinaryReadReal
^^^^^^^^^^^^^^^^^^^^^^^^
`BinaryReadReal <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L580>`__\ (\ ``file``\ , \ ``realType``\ )

- | \ *function description*\ :
  | read single Real from current file position in binary solution file



----


.. _sec-utilities-binaryreadstring:

Function: BinaryReadString
^^^^^^^^^^^^^^^^^^^^^^^^^^
`BinaryReadString <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L586>`__\ (\ ``file``\ , \ ``intType``\ )

- | \ *function description*\ :
  | read string from current file position in binary solution file



----


.. _sec-utilities-binaryreadarrayindex:

Function: BinaryReadArrayIndex
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`BinaryReadArrayIndex <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L592>`__\ (\ ``file``\ , \ ``intType``\ )

- | \ *function description*\ :
  | read Index array from current file position in binary solution file



----


.. _sec-utilities-binaryreadrealvector:

Function: BinaryReadRealVector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`BinaryReadRealVector <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L599>`__\ (\ ``file``\ , \ ``intType``\ , \ ``realType``\ )

- | \ *function description*\ :
  | read Real vector from current file position in binary solution file
- | \ *output*\ :
  | return data as numpy array, or False if no data read



----


.. _sec-utilities-loadbinarysolutionfile:

Function: LoadBinarySolutionFile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`LoadBinarySolutionFile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L615>`__\ (\ ``fileName``\ , \ ``maxRows = -1``\ , \ ``verbose = True``\ )

- | \ *function description*\ :
  | read BINARY coordinates solution file (exported during static or dynamic simulation with option exu.SimulationSettings().solutionSettings.coordinatesSolutionFileName='...') into dictionary
- | \ *input*\ :
  | \ ``fileName``\ : string containing directory and filename of stored coordinatesSolutionFile
  | \ ``verbose``\ : if True, some information is written when importing file (use for huge files to track progress)
  | \ ``maxRows``\ : maximum number of data rows loaded, if saveMode=True; use this for huge files to reduce loading time; set -1 to load all rows
- | \ *output*\ :
  | dictionary with 'data': the matrix of stored solution vectors, 'columnsExported': a list with integer values showing the exported sizes [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData], 'nColumns': the number of data columns and 'nRows': the number of data rows



----


.. _sec-utilities-recoversolutionfile:

Function: RecoverSolutionFile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`RecoverSolutionFile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L800>`__\ (\ ``fileName``\ , \ ``newFileName``\ , \ ``verbose = 0``\ )

- | \ *function description*\ :
  | recover solution file with last row not completely written (e.g., if crashed, interrupted or no flush file option set)
- | \ *input*\ :
  | \ ``fileName``\ : string containing directory and filename of stored coordinatesSolutionFile
  | \ ``newFileName``\ : string containing directory and filename of new coordinatesSolutionFile
  | \ ``verbose``\ : 0=no information, 1=basic information, 2=information per row
- | \ *output*\ :
  | writes only consistent rows of file to file with name newFileName



----


.. _sec-utilities-initializefromrestartfile:

Function: InitializeFromRestartFile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`InitializeFromRestartFile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L856>`__\ (\ ``mbs``\ , \ ``simulationSettings``\ , \ ``restartFileName``\ , \ ``verbose = True``\ )

- | \ *function description*\ :
  | recover initial coordinates, time, etc. from given restart file
- | \ *input*\ :
  | \ ``mbs``\ : MainSystem to be operated with
  | \ ``simulationSettings``\ : simulationSettings which is updated and shall be used afterwards for SolveDynamic(...) or SolveStatic(...)
  | \ ``restartFileName``\ : string containing directory and filename of stored restart file, as given in solutionSettings.restartFileName
  | \ ``verbose``\ : False=no information, True=basic information
- | \ *output*\ :
  | modifies simulationSettings and sets according initial conditions in mbs



----


.. _sec-utilities-setsolutionstate:

Function: SetSolutionState
^^^^^^^^^^^^^^^^^^^^^^^^^^
`SetSolutionState <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L918>`__\ (\ ``mbs``\ , \ ``solution``\ , \ ``row``\ , \ ``configuration = exudyn.ConfigurationType.Current``\ , \ ``sendRedrawSignal = True``\ )

- | \ *function description*\ :
  | load selected row of solution dictionary (previously loaded with LoadSolutionFile) into specific state; flag sendRedrawSignal is only used if configuration = exudyn.ConfigurationType.Visualization



----


.. _sec-utilities-animatesolution:

Function: AnimateSolution
^^^^^^^^^^^^^^^^^^^^^^^^^
`AnimateSolution <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L951>`__\ (\ ``mbs``\ , \ ``solution``\ , \ ``rowIncrement = 1``\ , \ ``timeout = 0.04``\ , \ ``createImages = False``\ , \ ``runLoop = False``\ )

- | \ *function description*\ :
  | This function is not further maintaned and should only be used if you do not have tkinter (like on some MacOS versions); use exudyn.interactive.SolutionViewer() instead! AnimateSolution consecutively load the rows of a solution file and visualize the result
- | \ *input*\ :
  | \ ``mbs``\ : the system used for animation
  | \ ``solution``\ : solution dictionary previously loaded with LoadSolutionFile; will be played from first to last row
  | \ ``rowIncrement``\ : can be set larger than 1 in order to skip solution frames: e.g. rowIncrement=10 visualizes every 10th row (frame)
  | \ ``timeout``\ : in seconds is used between frames in order to limit the speed of animation; e.g. use timeout=0.04 to achieve approximately 25 frames per second
  | \ ``createImages``\ : creates consecutively images from the animation, which can be converted into an animation
  | \ ``runLoop``\ : if True, the animation is played in a loop until 'q' is pressed in render window
- | \ *output*\ :
  | renders the scene in mbs and changes the visualization state in mbs continuously

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `SliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/SliderCrank.py>`_\  (Ex), \ `slidercrankWithMassSpring.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/slidercrankWithMassSpring.py>`_\  (Ex), \ `sliderCrankFloatingTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrankFloatingTest.py>`_\  (TM)



----

Function: DrawSystemGraph
^^^^^^^^^^^^^^^^^^^^^^^^^
`DrawSystemGraph <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L995>`__\ (\ ``mbs``\ , \ ``showLoads = True``\ , \ ``showSensors = True``\ , \ ``useItemNames = False``\ , \ ``useItemTypes = False``\ , \ ``addItemTypeNames = True``\ , \ ``multiLine = True``\ , \ ``fontSizeFactor = 1.``\ , \ ``layoutDistanceFactor = 3.``\ , \ ``layoutIterations = 100``\ , \ ``showLegend = True``\ , \ ``tightLayout = True``\ )


- | **NOTE**\ : this function is directly available in MainSystem (mbs); it should be directly called as mbs.DrawSystemGraph(...). For description of the interface, see the MainSystem Python extensions,  :ref:`sec-mainsystemextensions-drawsystemgraph`\ 



----


.. _sec-utilities-createtcpipconnection:

Function: CreateTCPIPconnection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateTCPIPconnection <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L1394>`__\ (\ ``sendSize``\ , \ ``receiveSize``\ , \ ``IPaddress = '127.0.0.1'``\ , \ ``port = 52421``\ , \ ``bigEndian = False``\ , \ ``verbose = False``\ )

- | \ *function description*\ :
  | function which has to be called before simulation to setup TCP/IP socket (server) for
  | sending and receiving data; can be used to communicate with other Python interpreters
  | or for communication with MATLAB/Simulink
- | \ *input*\ :
  | \ ``sendSize``\ : number of double values to be sent to TCPIP client
  | \ ``receiveSize``\ : number of double values to be received from TCPIP client
  | \ ``IPaddress``\ : string containing IP address of client (e.g., '127.0.0.1')
  | \ ``port``\ : port for communication with client
  | \ ``bigEndian``\ : if True, it uses bigEndian, otherwise littleEndian is used for byte order
- | \ *output*\ :
  | returns information (TCPIPdata class) on socket; recommended to store this in mbs.sys['TCPIPobject']
- | \ *example*\ :

.. code-block:: python

  mbs.sys['TCPIPobject'] = CreateTCPIPconnection(sendSize=3, receiveSize=2,
                                                 bigEndian=True, verbose=True)
  sampleTime = 0.01 #sample time in MATLAB! must be same!
  mbs.variables['tLast'] = 0 #in case that exudyn makes finer steps than sample time
  def PreStepUserFunction(mbs, t):
      if t >= mbs.variables['tLast'] + sampleTime:
          mbs.variables['tLast'] += sampleTime
          tcp = mbs.sys['TCPIPobject']
          y = TCPIPsendReceive(tcp, np.array([t, np.sin(t), np.cos(t)])) #time, torque
          tau = y[1]
          exudyn.Print('tau=',tau)
      return True
  try:
      mbs.SetPreStepUserFunction(PreStepUserFunction)
      #%%++++++++++++++++++++++++++++++++++++++++++++++++++
      mbs.Assemble()
      [...] #start renderer; simulate model
  finally: #use this to always close connection, even in case of errors
      CloseTCPIPconnection(mbs.sys['TCPIPobject'])
  #*****************************************
  #the following settings work between Python and MATLAB-Simulink (client), and gives stable results(with only delay of one step):
  # TCP/IP Client Send:
  #   priority = 2 (in properties)
  #   blocking = false
  #   Transfer Delay on (but off also works)
  # TCP/IP Client Receive:
  #   priority = 1 (in properties)
  #   blocking = true
  #   Sourec Data type = double
  #   data size = number of double in packer
  #   Byte order = BigEndian
  #   timeout = 10


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `TCPIPexudynMatlab.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/TCPIPexudynMatlab.py>`_\  (Ex)



----


.. _sec-utilities-tcpipsendreceive:

Function: TCPIPsendReceive
^^^^^^^^^^^^^^^^^^^^^^^^^^
`TCPIPsendReceive <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L1427>`__\ (\ ``TCPIPobject``\ , \ ``sendData``\ )

- | \ *function description*\ :
  | call this function at every simulation step at which you intend to communicate with
  | other programs via TCPIP; e.g., call this function in preStepUserFunction of a mbs model
- | \ *input*\ :
  | \ ``TCPIPobject``\ : the object returned by CreateTCPIPconnection(...)
  | \ ``sendData``\ : numpy array containing data (double array) to be sent; must agree with sendSize
- | \ *output*\ :
  | returns array as received from TCPIP
- | \ *example*\ :

.. code-block:: python

  mbs.sys['TCPIPobject']=CreateTCPIPconnection(sendSize=2, receiveSize=1, IPaddress='127.0.0.1')
  y = TCPIPsendReceive(mbs.sys['TCPIPobject'], np.array([1.,2.]))
  exudyn.Print(y)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `TCPIPexudynMatlab.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/TCPIPexudynMatlab.py>`_\  (Ex)



----


.. _sec-utilities-closetcpipconnection:

Function: CloseTCPIPconnection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CloseTCPIPconnection <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L1440>`__\ (\ ``TCPIPobject``\ )

- | \ *function description*\ :
  | close a previously created TCPIP connection

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `TCPIPexudynMatlab.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/TCPIPexudynMatlab.py>`_\  (Ex)


.. _sec-module-utilities-class-tcpipdata:

CLASS TCPIPdata (in module utilities)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    helper class for CreateTCPIPconnection and for TCPIPsendReceive


