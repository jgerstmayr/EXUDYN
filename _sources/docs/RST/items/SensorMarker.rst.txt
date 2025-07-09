

.. _sec-item-sensormarker:

SensorMarker
============

A sensor attached to a marker. The sensor measures the selected marker values and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Depending on markers, it can measure Coordinates (MarkerNodeCoordinate), Position and Velocity (MarkerXXXPosition), Position, Velocity, Rotation and AngularVelocityLocal (MarkerXXXRigid). Note that marker values are only available for the current configuration. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file

The item \ **SensorMarker**\  with type = 'Marker' has the following parameters:

* | **name** [type = String, default = '']:
  | sensor's unique name
* | **markerNumber** [type = MarkerIndex, default = invalid (-1)]:
  | marker number to which sensor is attached to
* | **writeToFile** [type = Bool, default = True]:
  | True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''
* | **fileName** [type = String, default = '']:
  | directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist
* | **outputVariableType** [type = OutputVariableType, default = OutputVariableType::_None]:
  | OutputVariableType for sensor; output variables are only possible according to markertype, see general description of SensorMarker
* | **storeInternal** [type = Bool, default = False]:
  | true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available
* | **visualization** [type = VSensorMarker]:
  | parameters for visualization of item



The item VSensorMarker has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-sensormarker:

DESCRIPTION of SensorMarker
---------------------------

Relevant Examples and TestModels with weblink:

    \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Examples/), \ `NGsolveCMStutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCMStutorial.py>`_\  (Examples/), \ `NGsolveModalAnalysis.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveModalAnalysis.py>`_\  (Examples/), \ `ObjectFFRFconvergenceTestHinge.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ObjectFFRFconvergenceTestHinge.py>`_\  (Examples/), \ `pendulumVerify.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulumVerify.py>`_\  (Examples/), \ `ROSMobileManipulator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSMobileManipulator.py>`_\  (Examples/), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TestModels/), \ `pendulumFriction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/pendulumFriction.py>`_\  (TestModels/), \ `plotSensorTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/plotSensorTest.py>`_\  (TestModels/), \ `reevingSystemSpringsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/reevingSystemSpringsTest.py>`_\  (TestModels/), \ `relativeRotationTranslationMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/relativeRotationTranslationMechanism.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


