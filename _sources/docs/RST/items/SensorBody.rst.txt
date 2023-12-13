

.. _sec-item-sensorbody:

SensorBody
==========

A sensor attached to a body-object with local position \ :math:`\pLocB`\ . As a difference to SensorObject, the body sensor needs a local position at which the sensor is attached to. The sensor measures OutputVariableBody and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.

The item \ **SensorBody**\  with type = 'Body' has the following parameters:

* | **name** [type = String, default = '']:
  | sensor's unique name
* | **bodyNumber** [type = ObjectIndex, default = invalid (-1)]:
  | body (=object) number to which sensor is attached to
* | **localPosition** [\ :math:`\pLocB`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | local (body-fixed) body position of sensor
* | **writeToFile** [type = Bool, default = True]:
  | True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''
* | **fileName** [type = String, default = '']:
  | directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist
* | **outputVariableType** [type = OutputVariableType, default = OutputVariableType::_None]:
  | OutputVariableType for sensor
* | **storeInternal** [type = Bool, default = False]:
  | true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available
* | **visualization** [type = VSensorBody]:
  | parameters for visualization of item



The item VSensorBody has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-sensorbody:

DESCRIPTION of SensorBody
-------------------------

Relevant Examples and TestModels with weblink:

    \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Examples/), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Examples/), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Examples/), \ `cartesianSpringDamperUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamperUserFunction.py>`_\  (Examples/), \ `finiteSegmentMethod.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/finiteSegmentMethod.py>`_\  (Examples/), \ `flexiblePendulumANCF.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/flexiblePendulumANCF.py>`_\  (Examples/), \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Examples/), \ `rigidBodyIMUtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyIMUtest.py>`_\  (Examples/), \ `rigidBodyTutorial2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial2.py>`_\  (Examples/), \ `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_\  (Examples/), \ `rigidBodyTutorial3withMarkers.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3withMarkers.py>`_\  (Examples/), \ `rigidRotor3DbasicBehaviour.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidRotor3DbasicBehaviour.py>`_\  (Examples/), \ `ANCFoutputTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFoutputTest.py>`_\  (TestModels/), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TestModels/), \ `computeODE2AEeigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2AEeigenvaluesTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


