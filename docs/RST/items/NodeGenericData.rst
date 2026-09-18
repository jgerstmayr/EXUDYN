

.. _sec-item-nodegenericdata:

NodeGenericData
===============

A node containing a number of data (history) variables. Use this node e.g. for contact (active set), friction or plasticity (history variables).

\ **Additional information for NodeGenericData**\ :

* | This \ ``Node``\  has/provides the following types = \ ``GenericData``\ 


The item \ **NodeGenericData**\  with type = 'GenericData' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **initialCoordinates** [\ :math:`{\mathbf{x}}\cIni = [x_0,\,\ldots,\,x_{n_c}]\tp\cIni`\ , type = Vector, default = []]:
  | initial data coordinates
* | **numberOfDataCoordinates** [\ :math:`n_c`\ , type = UInt, default = 0]:
  | number of generic data coordinates (history variables)
* | **visualization** [type = VNodeGenericData]:
  | parameters for visualization of item



The item VNodeGenericData has the following parameters:

* | **show** [type = Bool, default = False]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-nodegenericdata:

DESCRIPTION of NodeGenericData
------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Coordinates``\ : \ :math:`{\mathbf{x}}\cConfig = [x_0,\,\ldots,\,x_{nc}]\tp\cConfig`\ 
  | data coordinates (history variables) vector of node



Relevant Examples and TestModels with weblink:

    \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/ANCFcontactCircle.py>`_\  (Examples/), \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/ANCFcontactCircle2.py>`_\  (Examples/), \ `ANCFmovingRigidbody.py <https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/ANCFmovingRigidbody.py>`_\  (Examples/), \ `ANCFslidingJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/ANCFslidingJoint.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFslidingJoint2Drigid.py <https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/ANCFslidingJoint2Drigid.py>`_\  (Examples/), \ `ANCFswitchingSlidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/ANCFswitchingSlidingJoint2D.py>`_\  (Examples/), \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/beltDriveALE.py>`_\  (Examples/), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/beltDriveReevingSystem.py>`_\  (Examples/), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/beltDrivesComparison.py>`_\  (Examples/), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/bicycleIftommBenchmark.py>`_\  (Examples/), \ `camFollowerExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/camFollowerExample.py>`_\  (Examples/), \ `ANCFcontactCircleTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/ANCFcontactCircleTest.py>`_\  (TestModels/), \ `ANCFcontactFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/ANCFcontactFrictionTest.py>`_\  (TestModels/), \ `ANCFmovingRigidBodyTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/ANCFmovingRigidBodyTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


