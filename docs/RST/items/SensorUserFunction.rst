

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


----------

.. _description-sensoruserfunction:

DESCRIPTION of SensorUserFunction
---------------------------------
The sensor collects data via a user function, which completely describes the output itself.
Note that the sensorNumbers and factors need to be consistent. 
The return value of the user function is a list of \ ``float``\  numbers which cast to a \ ``std::vector``\  in pybind.
This list can have arbitrary dimension, but should be kept constant during simulation.

--------

\ **Userfunction**\ : ``sensorUserFunction(mbs, t, sensorNumbers, factors, configuration)`` 


A user function, which computes a sensor output from other sensor outputs (or from generic time dependent functions).
The configuration in general will be the exudyn.ConfigurationType.Current, but others could be used as well except for SensorMarker.
The user function arguments are as follows:

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments /  return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs to which object belongs
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs
   * - | \ ``sensorNumbers``\ 
     - | Array \ :math:`\in \Ncal^n`\ 
     - | list of sensor numbers
   * - | \ ``factors``\ 
     - | Vector \ :math:`\in \Rcal^n`\ 
     - | list of factors that can be freely used for the user function
   * - | \ ``configuration``\ 
     - | exudyn.ConfigurationType
     - | usually the exudyn.ConfigurationType.Current, but could also be different in user defined functions.
   * - | \returnValue
     - | Vector \ :math:`\in \Rcal^{n_r}`\ 
     - | returns list or numpy array of sensor output values; size \ :math:`n_r`\  is implicitly defined by the returned list and may not be changed during simulation.


--------

\ **User function example**\ :



.. code-block:: python

    import exudyn as exu
    from exudyn.itemInterface import *
    from math import pi, atan2
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    node = mbs.AddNode(NodePoint(referenceCoordinates = [1,1,0], 
                                 initialCoordinates=[0,0,0],
                                 initialVelocities=[0,-1,0]))
    mbs.AddObject(MassPoint(nodeNumber = node, physicsMass=1))
    
    sNode = mbs.AddSensor(SensorNode(nodeNumber=node, fileName='solution/sensorTest.txt',
                          outputVariableType=exu.OutputVariableType.Position))

    #user function for sensor, convert position into angle:
    def UFsensor(mbs, t, sensorNumbers, factors, configuration):
        val = mbs.GetSensorValues(sensorNumbers[0]) #x,y,z
        phi = atan2(val[1],val[0]) #compute angle from x,y: atan2(y,x)
        return [factors[0]*phi] #return angle in degree
    
    sUser = mbs.AddSensor(SensorUserFunction(sensorNumbers=[sNode], factors=[180/pi], 
                                     fileName='solution/sensorTest2.txt',
                                     sensorUserFunction=UFsensor))

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic()

    if False:
        from exudyn.plot import PlotSensor
        PlotSensor(mbs, [sNode, sNode, sUser], [0, 1, 0])
    




Relevant Examples and TestModels with weblink:

    \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Examples/), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Examples/), \ `lugreFrictionODE1.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/lugreFrictionODE1.py>`_\  (Examples/), \ `lugreFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/lugreFrictionTest.py>`_\  (Examples/), \ `pendulumIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulumIftommBenchmark.py>`_\  (Examples/), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TestModels/), \ `fourBarMechanismIftomm.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/fourBarMechanismIftomm.py>`_\  (TestModels/), \ `sensorUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sensorUserFunctionTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


