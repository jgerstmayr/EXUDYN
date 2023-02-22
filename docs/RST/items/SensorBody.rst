

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




\ **The web version may not be complete. For details, always consider the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


