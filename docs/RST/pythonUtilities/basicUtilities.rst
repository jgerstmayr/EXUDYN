
.. _sec-module-basicutilities:

Module: basicUtilities
======================

Basic utility functions and constants; they depend on numpy only, not on exudyn.

- Author:    Johannes Gerstmayr 
- Date:      2020-03-10 (created) 
- | Notes:
  | Additional constants are defined:
  | pi = 3.1415926535897932
  | sqrt2 = 2\*\*0.5
  | g=9.81
  | Two variables 'gaussIntegrationPoints' and 'gaussIntegrationWeights' define integration points and weights for function GaussIntegrate(...)


.. _sec-basicutilities-clearworkspace:

Function: ClearWorkspace
^^^^^^^^^^^^^^^^^^^^^^^^
`ClearWorkspace <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L30>`__\ ()

- | \ *function description*\ :
  | clear all workspace variables except for system variables with '_' at beginning,
  | 'func' or 'module' in name; it also deletes all items in exudyn.sys and exudyn.variables,
  | EXCEPT from exudyn.sys['renderState'] for pertaining the previous view of the renderer
- | \ *notes*\ :
  | Use this function with CARE! In Spyder, it is certainly safer to add the preference Run\ :math:`\ra`\ 'remove all variables before execution'. It is recommended to call ClearWorkspace() at the very beginning of your models, to avoid that variables still exist from previous computations which may destroy repeatability of results
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  import exudyn.utilities
  #clear workspace at the very beginning, before loading other modules and potentially destroying unwanted things ...
  ClearWorkspace()       #cleanup
  #now continue with other code
  from exudyn.itemInterface import *
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  ...


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `springDamperUserFunctionNumbaJIT.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/springDamperUserFunctionNumbaJIT.py>`_\  (Ex), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM), \ `runTestExamples.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/runTestExamples.py>`_\  (TM)



----


.. _sec-basicutilities-smartround2string:

Function: SmartRound2String
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`SmartRound2String <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L86>`__\ (\ ``x``\ , \ ``prec = 3``\ )

- | \ *function description*\ :
  | round to max number of digits; may give more digits if this is shorter; using in general the format() with '.g' option, but keeping decimal point and using exponent where necessary



----


.. _sec-basicutilities-normalize:

Function: Normalize
^^^^^^^^^^^^^^^^^^^
`Normalize <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L98>`__\ (\ ``v``\ )

- | \ *function description*\ :
  | take a vector and return it normalized to L2-norm 1; a zero vector is returned as zero vector
- | \ *input*\ :
  | vector v as list or in numpy format
- | \ *output*\ :
  | \ ``list``\ : v multiplied with a scalar such that its L2-norm is 1, or the zero vector; a list, as
  | callers append the result to lists of normals

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `contactCurveWithLongCurve.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/contactCurveWithLongCurve.py>`_\  (Ex), \ `NGsolveCMStutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCMStutorial.py>`_\  (Ex), \ `NGsolveGeometry.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveGeometry.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `ObjectFFRFconvergenceTestHinge.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ObjectFFRFconvergenceTestHinge.py>`_\  (Ex)



----


.. _sec-basicutilities-gaussintegrate:

Function: GaussIntegrate
^^^^^^^^^^^^^^^^^^^^^^^^
`GaussIntegrate <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L130>`__\ (\ ``functionOfX``\ , \ ``integrationOrder``\ , \ ``a``\ , \ ``b``\ )

- | \ *function description*\ :
  | compute numerical integration of functionOfX in interval [a,b] using Gaussian integration
- | \ *input*\ :
  | \ ``functionOfX``\ : scalar, vector or matrix-valued function with scalar argument (X or other variable)
  | \ ``integrationOrder``\ : odd number in {1,3,5,7,9}; currently maximum order is 9
  | \ ``a``\ : integration range start
  | \ ``b``\ : integration range end
- | \ *output*\ :
  | (scalar or vectorized) integral value



----


.. _sec-basicutilities-lobattointegrate:

Function: LobattoIntegrate
^^^^^^^^^^^^^^^^^^^^^^^^^^
`LobattoIntegrate <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L170>`__\ (\ ``functionOfX``\ , \ ``integrationOrder``\ , \ ``a``\ , \ ``b``\ )

- | \ *function description*\ :
  | compute numerical integration of functionOfX in interval [a,b] using Lobatto integration
- | \ *input*\ :
  | \ ``functionOfX``\ : scalar, vector or matrix-valued function with scalar argument (X or other variable)
  | \ ``integrationOrder``\ : odd number in {1,3,5}; currently maximum order is 5
  | \ ``a``\ : integration range start
  | \ ``b``\ : integration range end
- | \ *output*\ :
  | (scalar or vectorized) integral value



----


.. _sec-basicutilities-getothermarker:

Function: GetOtherMarker
^^^^^^^^^^^^^^^^^^^^^^^^
`GetOtherMarker <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L203>`__\ (\ ``mbs``\ , \ ``bodyNumber``\ , \ ``existingMarker``\ , \ ``show = True``\ )

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


.. _sec-basicutilities-getjointargs:

Function: GetJointArgs
^^^^^^^^^^^^^^^^^^^^^^
`GetJointArgs <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L240>`__\ (\ ``mbs``\ , \ ``markerNumber0 = None``\ , \ ``markerNumber1 = None``\ , \ ``rotationMarker0 = None``\ , \ ``rotationMarker1 = None``\ , \ ``bodyNumber0 = None``\ , \ ``bodyNumber1 = None``\ )

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


.. _sec-basicutilities-showonlyobjects:

Function: ShowOnlyObjects
^^^^^^^^^^^^^^^^^^^^^^^^^
`ShowOnlyObjects <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L316>`__\ (\ ``mbs``\ , \ ``objectNumbers = []``\ , \ ``showOthers = False``\ )

- | \ *function description*\ :
  | function to hide all objects in mbs except for those listed in objectNumbers
- | \ *input*\ :
  | \ ``mbs``\ : mbs containing object
  | \ ``objectNumbers``\ : integer object number or list of object numbers to be shown; if empty list [], then all objects are shown
  | \ ``showOthers``\ : if True, then all other objects are shown again
- | \ *output*\ :
  | changes all colors in mbs, which is NOT reversible



----


.. _sec-basicutilities-highlightitem:

Function: HighlightItem
^^^^^^^^^^^^^^^^^^^^^^^
`HighlightItem <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L344>`__\ (\ ``SC``\ , \ ``mbs``\ , \ ``itemNumber``\ , \ ``itemType = exudyn.ItemType.Object``\ , \ ``showNumbers = True``\ )

- | \ *function description*\ :
  | highlight a certain item with number itemNumber; set itemNumber to -1 to show again all objects
- | \ *input*\ :
  | \ ``mbs``\ : mbs containing object
  | \ ``itemNumbers``\ : integer object/node/etc number to be highlighted
  | \ ``itemType``\ : type of items to be highlighted
  | \ ``showNumbers``\ : if True, then the numbers of these items are shown



----


.. _sec-basicutilities-ufsensorrecord:

Function: UFsensorRecord
^^^^^^^^^^^^^^^^^^^^^^^^
`UFsensorRecord <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L387>`__\ (\ ``mbs``\ , \ ``t``\ , \ ``sensorNumbers``\ , \ ``factors``\ , \ ``configuration``\ )

- | \ *function description*\ :
  | DEPRECATED: Internal SensorUserFunction, used in function AddSensorRecorder
- | \ *notes*\ :
  | Warning: this method is DEPRECATED, use storeInternal in Sensors, which is much more performant; Note, that a sensor usually just passes through values of an existing sensor, while recording the values to a numpy array row-wise (time in first column, data in remaining columns)



----


.. _sec-basicutilities-addsensorrecorder:

Function: AddSensorRecorder
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`AddSensorRecorder <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L405>`__\ (\ ``mbs``\ , \ ``sensorNumber``\ , \ ``endTime``\ , \ ``sensorsWritePeriod``\ , \ ``sensorOutputSize = 3``\ )

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


.. _sec-basicutilities-loadsolutionfile:

Function: LoadSolutionFile
^^^^^^^^^^^^^^^^^^^^^^^^^^
`LoadSolutionFile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L433>`__\ (\ ``fileName``\ , \ ``safeMode = False``\ , \ ``maxRows = -1``\ , \ ``verbose = True``\ , \ ``hasHeader = True``\ )

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


.. _sec-basicutilities-numpyint8arraytostring:

Function: NumpyInt8ArrayToString
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`NumpyInt8ArrayToString <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L571>`__\ (\ ``npArray``\ )

- | \ *function description*\ :
  | simple conversion of int8 arrays into strings (not highly efficient, so use only for short strings)



----


.. _sec-basicutilities-binaryreadindex:

Function: BinaryReadIndex
^^^^^^^^^^^^^^^^^^^^^^^^^
`BinaryReadIndex <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L580>`__\ (\ ``file``\ , \ ``intType``\ )

- | \ *function description*\ :
  | read single Index from current file position in binary solution file



----


.. _sec-basicutilities-binaryreadreal:

Function: BinaryReadReal
^^^^^^^^^^^^^^^^^^^^^^^^
`BinaryReadReal <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L588>`__\ (\ ``file``\ , \ ``realType``\ )

- | \ *function description*\ :
  | read single Real from current file position in binary solution file



----


.. _sec-basicutilities-binaryreadstring:

Function: BinaryReadString
^^^^^^^^^^^^^^^^^^^^^^^^^^
`BinaryReadString <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L596>`__\ (\ ``file``\ , \ ``intType``\ )

- | \ *function description*\ :
  | read string from current file position in binary solution file



----


.. _sec-basicutilities-binaryreadarrayindex:

Function: BinaryReadArrayIndex
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`BinaryReadArrayIndex <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L604>`__\ (\ ``file``\ , \ ``intType``\ )

- | \ *function description*\ :
  | read Index array from current file position in binary solution file



----


.. _sec-basicutilities-binaryreadrealvector:

Function: BinaryReadRealVector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`BinaryReadRealVector <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L612>`__\ (\ ``file``\ , \ ``intType``\ , \ ``realType``\ )

- | \ *function description*\ :
  | read Real vector from current file position in binary solution file
- | \ *output*\ :
  | return data as numpy array, or False if no data read



----


.. _sec-basicutilities-loadbinarysolutionfile:

Function: LoadBinarySolutionFile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`LoadBinarySolutionFile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L626>`__\ (\ ``fileName``\ , \ ``maxRows = -1``\ , \ ``verbose = True``\ )

- | \ *function description*\ :
  | read BINARY coordinates solution file (exported during static or dynamic simulation with option exu.SimulationSettings().solutionSettings.coordinatesSolutionFileName='...') into dictionary
- | \ *input*\ :
  | \ ``fileName``\ : string containing directory and filename of stored coordinatesSolutionFile
  | \ ``verbose``\ : if True, some information is written when importing file (use for huge files to track progress)
  | \ ``maxRows``\ : maximum number of data rows loaded, if saveMode=True; use this for huge files to reduce loading time; set -1 to load all rows
- | \ *output*\ :
  | dictionary with 'data': the matrix of stored solution vectors, 'columnsExported': a list with integer values showing the exported sizes [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData], 'nColumns': the number of data columns and 'nRows': the number of data rows



----


.. _sec-basicutilities-recoversolutionfile:

Function: RecoverSolutionFile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`RecoverSolutionFile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L812>`__\ (\ ``fileName``\ , \ ``newFileName``\ , \ ``verbose = 0``\ )

- | \ *function description*\ :
  | recover solution file with last row not completely written (e.g., if crashed, interrupted or no flush file option set)
- | \ *input*\ :
  | \ ``fileName``\ : string containing directory and filename of stored coordinatesSolutionFile
  | \ ``newFileName``\ : string containing directory and filename of new coordinatesSolutionFile
  | \ ``verbose``\ : 0=no information, 1=basic information, 2=information per row
- | \ *output*\ :
  | writes only consistent rows of file to file with name newFileName



----


.. _sec-basicutilities-initializefromrestartfile:

Function: InitializeFromRestartFile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`InitializeFromRestartFile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L871>`__\ (\ ``mbs``\ , \ ``simulationSettings``\ , \ ``restartFileName``\ , \ ``verbose = True``\ )

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


.. _sec-basicutilities-setsolutionstate:

Function: SetSolutionState
^^^^^^^^^^^^^^^^^^^^^^^^^^
`SetSolutionState <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L947>`__\ (\ ``mbs``\ , \ ``solution``\ , \ ``row``\ , \ ``configuration = exudyn.ConfigurationType.Current``\ , \ ``sendRedrawSignal = True``\ )

- | \ *function description*\ :
  | load selected row of solution dictionary (previously loaded with LoadSolutionFile) into specific state; flag sendRedrawSignal is only used if configuration = exudyn.ConfigurationType.Visualization



----


.. _sec-basicutilities-animatesolution:

Function: AnimateSolution
^^^^^^^^^^^^^^^^^^^^^^^^^
`AnimateSolution <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L973>`__\ (\ ``mbs``\ , \ ``solution``\ , \ ``rowIncrement = 1``\ , \ ``timeout = 0.04``\ , \ ``createImages = False``\ , \ ``runLoop = False``\ )

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

