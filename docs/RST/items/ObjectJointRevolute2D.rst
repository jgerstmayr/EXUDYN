

.. _sec-item-objectjointrevolute2d:

ObjectJointRevolute2D
=====================

A revolute joint in 2D; constrains the absolute 2D position of two points given by PointMarkers or RigidMarkers

\ **Additional information for ObjectJointRevolute2D**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``RevoluteJoint2D``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRevoluteJoint2D``\ 


The item \ **ObjectJointRevolute2D**\  with type = 'JointRevolute2D' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectJointRevolute2D]:
  | parameters for visualization of item



The item VObjectJointRevolute2D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = radius of revolute joint; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectjointrevolute2d:

DESCRIPTION of ObjectJointRevolute2D
------------------------------------

Relevant Examples and TestModels with weblink:

    \ `pendulumGeomExactBeam2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulumGeomExactBeam2D.py>`_\  (Examples/), \ `sliderCrank3DwithANCFbeltDrive2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive2.py>`_\  (Examples/), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFslidingJoint2Drigid.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2Drigid.py>`_\  (Examples/), \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Examples/), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Examples/), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Examples/), \ `doublePendulum2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/doublePendulum2D.py>`_\  (Examples/), \ `finiteSegmentMethod.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/finiteSegmentMethod.py>`_\  (Examples/), \ `geneticOptimizationSliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/geneticOptimizationSliderCrank.py>`_\  (Examples/), \ `HydraulicActuator2Arms.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/HydraulicActuator2Arms.py>`_\  (Examples/), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TestModels/), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TestModels/), \ `ANCFoutputTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFoutputTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


