

.. _sec-item-sensoruserfunction:

SensorUserFunction
==================

A sensor defined by a user function. The sensor is intended to collect sensor values of a list of given sensors and recombine the output into a new value for output or control purposes. It is also possible to use this sensor without any dependence on other sensors in order to generate output for, e.g., any quantities in mbs or solvers.

The item \ **SensorUserFunction**\  with type = 'UserFunction' has the following parameters:

* | **name** [type = String, default = '']:
  | sensor's unique name
* | **sensorNumbers** [\ :math:`\mathbf{n}_s = [s_0,\,\ldots,\,s_n]\tp`\ , type = ArraySensorIndex, default = []]:
  | optional list of \ :math:`n`\  sensor numbers for use in user function
* | **factors** [\ :math:`\mathbf{f}_s = [f_0,\,\ldots,\,f_m]\tp`\ , type = Vector, default = []]:
  | optional list of \ :math:`m`\  factors which can be used, e.g., for weighting sensor values
* | **writeToFile** [type = Bool, default = True]:
  | True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''
* | **fileName** [type = String, default = '']:
  | directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist
* | **sensorUserFunction** [type = PyFunctionVectorMbsScalarArrayIndexVectorConfiguration, default =  0]:
  | A Python function which defines the time-dependent user function, which usually evaluates one or several sensors and computes a new sensor value, see example
* | **storeInternal** [type = Bool, default = False]:
  | true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available
* | **visualization** [type = VSensorUserFunction]:
  | parameters for visualization of item



The item VSensorUserFunction has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown; sensor visualization CURRENTLY NOT IMPLEMENTED




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


