
.. _sec-module-robotics-utilities:

Module: robotics.utilities
--------------------------

The utilities contains general helper functions for the robotics module

- Date:      2023-04-15 


.. _sec-utilities-addlidar:

Function: AddLidar
^^^^^^^^^^^^^^^^^^
`AddLidar <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/utilities.py\#L42>`__\ (\ ``mbs``\ , \ ``generalContactIndex``\ , \ ``positionOrMarker``\ , \ ``minDistance = 0``\ , \ ``maxDistance = 1e7``\ , \ ``cylinderRadius = 0``\ , \ ``lineLength = 1``\ , \ ``numberOfSensors = 100``\ , \ ``angleStart = 0``\ , \ ``angleEnd = 2*np.pi``\ , \ ``inclination = 0``\ , \ ``rotation = np.eye(3)``\ , \ ``selectedTypeIndex = exudyn.ContactTypeIndex.IndexEndOfEnumList``\ , \ ``storeInternal = False``\ , \ ``fileName = ''``\ , \ ``measureVelocity = False``\ , \ ``addGraphicsObject = True``\ , \ ``drawDisplaced = True``\ , \ ``color = [1.0, 0.0, 0.0, 1.0]``\ )

- | \ *function description*\ :
  | Function to add many distance sensors to represent Lidar; sensors can be either placed on absolute position or attached to rigid body marker
- | \ *input*\ :
  | \ ``generalContactIndex``\ : the number of the GeneralContact object in mbs; the index of the GeneralContact object which has been added with last AddGeneralContact(...) command is generalContactIndex=mbs.NumberOfGeneralContacts()-1
  | \ ``positionOrMarker``\ : either a 3D position as list or np.array, or a MarkerIndex with according rigid body marker
  | \ ``minDistance``\ : the minimum distance which is accepted; smaller distance will be ignored
  | \ ``maxDistance``\ : the maximum distance which is accepted; items being at maxDistance or futher are ignored; if no items are found, the function returns maxDistance
  | \ ``cylinderRadius``\ : in case of spheres (selectedTypeIndex=ContactTypeIndex.IndexSpheresMarkerBased), a cylinder can be used which measures the shortest distance at a certain radius (geometrically interpreted as cylinder)
  | \ ``lineLength``\ : length of line to be drawn; note that this length is drawn from obstacle towards sensor if drawDisplaced=True, but the length is always constant
  | \ ``numberOfSensors``\ : number of sensors arranged between angleStart and angleEnd; higher numbers give finer resolution (but requires more CPU time); must be larger than 1
  | \ ``angleStart``\ : starting rangle of angles to be used (in radiant); angle of lidar beam is relative to X-axis, using positive rotation sense about Z-axis
  | \ ``angleEnd``\ : end of range for angle to be used (in radiant); angle of lidar beam is relative to X-axis, using positive rotation sense about Z-axis
  | \ ``inclination``\ : angle of inclination (radiant), positive values showing upwards (Z-direction) if rotation is the identity matrix
  | \ ``rotation``\ : a 3x3 rotation matrix (numpy); the sensor is placed in the X-Y plane of the marker where it is added to; however, you can use this rotation matrix to change the orientation
  | \ ``selectedTypeIndex``\ : either this type has default value, meaning that all items in GeneralContact are measured, or there is a specific type index, which is the only type that is considered during measurement
  | \ ``storeInternal``\ : like with any SensorUserFunction, setting to True stores sensor data internally
  | \ ``fileName``\ : if defined, recorded data of SensorUserFunction is written to specified file
  | \ ``measureVelocity``\ : if True, the sensor measures additionally the velocity (component 0=distance, component 1=velocity); velocity is the velocity in direction 'dirSensor' and does not account for changes in geometry, thus it may be different from the time derivative of the distance!
  | \ ``addGraphicsObject``\ : if True, the distance sensor is also visualized graphically in a simplified manner with a red line having the length of dirSensor; NOTE that updates are ONLY performed during computation, not in visualization; for this reason, solutionSettings.sensorsWritePeriod should be accordingly small
  | \ ``drawDisplaced``\ : if True, the red line is drawn backwards such that it moves along the measured surface; if False, the beam is fixed to marker or position
  | \ ``color``\ : optional color for 'laser beam' to be drawn
- | \ *output*\ :
  | creates sensor and returns list of sensor numbers for all laser sensors
- | \ *notes*\ :
  | use generalContactIndex = CreateDistanceSensorGeometry(...) before to create GeneralContact module containing geometry

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `mobileMecanumWheelRobotWithLidar.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mobileMecanumWheelRobotWithLidar.py>`_\  (Ex), \ `laserScannerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/laserScannerTest.py>`_\  (TM)



----


.. _sec-utilities-getroboticstoolboxinternalmodel:

Function: GetRoboticsToolboxInternalModel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetRoboticsToolboxInternalModel <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/utilities.py\#L85>`__\ (\ ``modelName = ''``\ , \ ``ignoreURDFerrors = True``\ )

- | \ *function description*\ :
  | Interface to roboticstoolbox (RTB) for loading internal robot models. Function retrieves internal model available from roboticstoolbox.models.URDF, usually stored in
  | site-packages/rtbdata/xacro/. See the github project of roboticstoolbox-python of P. Corke and J. Haviland for more details.
  | The model name is the short name used internally in the RTB. For available names, see the list roboticstoolbox.models.URDF.__all__ !
- | \ *input*\ :
  | \ ``modelName``\ : string for model, such as UR5, Puma560, Panda or LBR
  | \ ``ignoreURDFerrors``\ : if set True, urdf errors are ignored and only the model is loaded
- | \ *output*\ :
  | returns dictionary with 'robot' (RTB Robot class), 'urdf' which is the RTB representation of the URDF file for loading mesh files
- | \ *notes*\ :
  | requires installation (pip install) of roboticstoolbox-python; in our tests we had problems with installers on newer Python and therefore tested with Python 3.9! Note that some models doe not include mass and inertia and therefore will not run as dynamic models!

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `serialRobotURDF.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotURDF.py>`_\  (Ex)



----


.. _sec-utilities-loadurdfrobot:

Function: LoadURDFrobot
^^^^^^^^^^^^^^^^^^^^^^^
`LoadURDFrobot <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/utilities.py\#L155>`__\ (\ ``urdfFilePath``\ , \ ``urdfBasePath``\ , \ ``gripperLinks = None``\ , \ ``manufacturer = ''``\ )

- | \ *function description*\ :
  | Interface to roboticstoolbox (RTB) of P. Corke and J. Haviland. Use this function for loading urdf/xacro files.
- | \ *input*\ :
  | \ ``urdfFilePath``\ : string relative to urdfBasePath, representing xacro or urdf file
  | \ ``urdfBasePath``\ : string representing the base path of the urdf or xacro directory for the respective robot; the urdfBasePath is used to load further files which are given internally in urdf files, such as collision or mesh files
  | \ ``gripperLinks``\ : list of link numbers representing gripper (as used internally in RTB Robot class)
- | \ *output*\ :
  | returns dictionary with 'robot' (RTB Robot class), 'urdf' which is the RTB representation of the URDF file for loading mesh files
- | \ *notes*\ :
  | requires installation (pip install) of roboticstoolbox-python; in our tests we had problems with installers on newer Python and therefore tested with Python 3.9! Note that some models doe not include mass and inertia and therefore will not run as dynamic models!



----


.. _sec-utilities-geturdfrobotdata:

Function: GetURDFrobotData
^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetURDFrobotData <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/utilities.py\#L199>`__\ (\ ``robot``\ , \ ``urdf = None``\ , \ ``linkColorList = None``\ , \ ``staticJointValues = None``\ , \ ``returnStaticGraphicsList = False``\ , \ ``exportMesh = False``\ , \ ``verbose = 1``\ )

- | \ *function description*\ :
  | Interface to roboticstoolbox (RTB) of P. Corke and J. Haviland and Pymeshlab to import robot model and visualization into a struture readable by Exudyn. NOTE that this function is to be seen as a starting point for import, while some models have to be imported differently, in particular for joints that are not revolute or prismatic (in this case, copy function into local file and modify)!
- | \ *input*\ :
  | \ ``robot``\ : a RTB Robot (class) model, as returned e.g. by LoadURDFrobot
  | \ ``urdf``\ : a RTB URDF (class) representing the URDF data, as returned e.g. by LoadURDFrobot
  | \ ``linkColorList``\ : if not None, this can contain a list of RGBA color lists for each link to prescribe colors instead of using internally stored colors or general color information (.obj files); set linkColorList=[graphics.color.red]\*8 to set 8 link colors red; links are counted as in the urdf file and may be different from the number of joints
  | \ ``staticJointValues``\ : if not None, has to be a list of joint angles (or displacements) for computing the static graphics list
  | \ ``returnStaticGraphicsList``\ : return a list of GraphicsData which can be put into a ground to check visualization for zero joints, using: mbs.CreateGround(graphicsDataList=staticGraphicsList)
  | \ ``exportMesh``\ : if True, the returned dict also contains a meshSetList which refers to the MeshSet in pymeshlab, which can be used for debugging purposes
  | \ ``verbose``\ : 0 .. no output printed (only exceptions), 1 .. warnings, 2 .. further information
- | \ *output*\ :
  | returns dictionary with items linkList, graphicsBaseList, graphicsToolList
- | \ *notes*\ :
  | requires installation (pip install) of roboticstoolbox-python and pymeshlab; if pymeshlab is not installed, a warning is raised and graphics is ignored

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `serialRobotURDF.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotURDF.py>`_\  (Ex)

