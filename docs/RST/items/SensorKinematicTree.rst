

.. _sec-item-sensorkinematictree:

SensorKinematicTree
===================

A sensor attached to a KinematicTree with local position \ :math:`\pLocB`\  and link number \ :math:`n_l`\ . As a difference to SensorBody, the KinematicTree sensor needs a local position and a link number, which defines the sub-body at which the sensor values are evaluated. The local position is given in sub-body (link) local coordinates. The sensor measures OutputVariableKinematicTree and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.

The item \ **SensorKinematicTree**\  with type = 'KinematicTree' has the following parameters:

* | **name** [type = String, default = '']:
  | sensor's unique name
* | **objectNumber** [type = ObjectIndex, default = invalid (-1)]:
  | object number of KinematicTree to which sensor is attached to
* | **linkNumber** [\ :math:`n_l`\ , type = UInt, default = invalid (-1)]:
  | number of link in KinematicTree to measure quantities
* | **localPosition** [\ :math:`\LU{l}{{\mathbf{b}}}`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | local (link-fixed) position of sensor, defined in link (\ :math:`n_l`\ ) coordinate system
* | **writeToFile** [type = Bool, default = True]:
  | True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''
* | **fileName** [type = String, default = '']:
  | directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist
* | **outputVariableType** [type = OutputVariableType, default = OutputVariableType::_None]:
  | OutputVariableType for sensor
* | **storeInternal** [type = Bool, default = False]:
  | true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available
* | **visualization** [type = VSensorKinematicTree]:
  | parameters for visualization of item



The item VSensorKinematicTree has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-sensorkinematictree:

DESCRIPTION of SensorKinematicTree
----------------------------------

Relevant Examples and TestModels with weblink:

    \ `openAIgymNLinkContinuous.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymNLinkContinuous.py>`_\  (Examples/), \ `serialRobotInverseKinematics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInverseKinematics.py>`_\  (Examples/), \ `serialRobotKinematicTreeDigging.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTreeDigging.py>`_\  (Examples/), \ `stiffFlyballGovernorKT.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stiffFlyballGovernorKT.py>`_\  (Examples/), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TestModels/), \ `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


