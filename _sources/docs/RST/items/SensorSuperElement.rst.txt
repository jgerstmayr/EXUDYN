

.. _sec-item-sensorsuperelement:

SensorSuperElement
==================

A sensor attached to a SuperElement-object with mesh node number. As a difference to other ObjectSensors, the SuperElement sensor has a mesh node number at which the sensor is attached to. The sensor measures OutputVariableSuperElement and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.

The item \ **SensorSuperElement**\  with type = 'SuperElement' has the following parameters:

* | **name** [type = String, default = '']:
  | sensor's unique name
* | **bodyNumber** [type = ObjectIndex, default = invalid (-1)]:
  | body (=object) number to which sensor is attached to
* | **meshNodeNumber** [type = UInt, default = invalid (-1)]:
  | mesh node number, which is a local node number with in the object (starting with 0); the node number may represent a real Node in mbs, or may be virtual and reconstructed from the object coordinates such as in ObjectFFRFreducedOrder
* | **writeToFile** [type = Bool, default = True]:
  | True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''
* | **fileName** [type = String, default = '']:
  | directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist
* | **outputVariableType** [type = OutputVariableType, default = OutputVariableType::_None]:
  | OutputVariableType for sensor, based on the output variables available for the mesh nodes (see special section for super element output variables, e.g, in ObjectFFRFreducedOrder, Section :ref:`sec-objectffrfreducedorder-superelementoutput`\ )
* | **storeInternal** [type = Bool, default = False]:
  | true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available
* | **visualization** [type = VSensorSuperElement]:
  | parameters for visualization of item



The item VSensorSuperElement has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-sensorsuperelement:

DESCRIPTION of SensorSuperElement
---------------------------------

Relevant Examples and TestModels with weblink:

    \ `CMSexampleCourse.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/CMSexampleCourse.py>`_\  (Examples/), \ `NGsolveCraigBampton.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCraigBampton.py>`_\  (Examples/), \ `NGsolvePostProcessingStresses.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePostProcessingStresses.py>`_\  (Examples/), \ `ObjectFFRFconvergenceTestBeam.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ObjectFFRFconvergenceTestBeam.py>`_\  (Examples/), \ `objectFFRFreducedOrderNetgen.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/objectFFRFreducedOrderNetgen.py>`_\  (Examples/), \ `pendulumVerify.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulumVerify.py>`_\  (Examples/), \ `abaqusImportTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/abaqusImportTest.py>`_\  (TestModels/), \ `objectFFRFreducedOrderAccelerations.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderAccelerations.py>`_\  (TestModels/), \ `objectFFRFreducedOrderStressModesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderStressModesTest.py>`_\  (TestModels/), \ `objectFFRFreducedOrderTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderTest.py>`_\  (TestModels/), \ `objectFFRFTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFTest.py>`_\  (TestModels/), \ `objectFFRFTest2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFTest2.py>`_\  (TestModels/), \ `objectGenericODE2Test.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectGenericODE2Test.py>`_\  (TestModels/), \ `perfObjectFFRFreducedOrder.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/perfObjectFFRFreducedOrder.py>`_\  (TestModels/), \ `superElementRigidJointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/superElementRigidJointTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


