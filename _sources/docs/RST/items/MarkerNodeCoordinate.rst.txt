

.. _sec-item-markernodecoordinate:

MarkerNodeCoordinate
====================

A node-Marker attached to a \ :ref:`ODE2 <ODE2>`\  coordinate of a node; this marker allows to connect a coordinate-based constraint or connector to a nodal coordinate (also NodeGround); for \ :ref:`ODE1 <ODE1>`\  coordinates use MarkerNodeODE1Coordinate.

\ **Additional information for MarkerNodeCoordinate**\ :

* | This \ ``Marker``\  has/provides the following types = \ ``Node``\ , \ ``Coordinate``\ 


The item \ **MarkerNodeCoordinate**\  with type = 'NodeCoordinate' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number to which marker is attached to
* | **coordinate** [type = UInt, default = invalid (-1)]:
  | coordinate of node to which marker is attached to
* | **visualization** [type = VMarkerNodeCoordinate]:
  | parameters for visualization of item



The item VMarkerNodeCoordinate has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-markernodecoordinate:

DESCRIPTION of MarkerNodeCoordinate
-----------------------------------

Relevant Examples and TestModels with weblink:

    \ `ALEANCFpipe.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ALEANCFpipe.py>`_\  (Examples/), \ `ANCFALEtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFALEtest.py>`_\  (Examples/), \ `ANCFcantileverTestDyn.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcantileverTestDyn.py>`_\  (Examples/), \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Examples/), \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Examples/), \ `ANCFmovingRigidbody.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFmovingRigidbody.py>`_\  (Examples/), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFslidingJoint2Drigid.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2Drigid.py>`_\  (Examples/), \ `ANCFswitchingSlidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFswitchingSlidingJoint2D.py>`_\  (Examples/), \ `ANCFtestHalfcircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFtestHalfcircle.py>`_\  (Examples/), \ `ANCFtests2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFtests2.py>`_\  (Examples/), \ `ANCFBeamEigTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamEigTest.py>`_\  (TestModels/), \ `ANCFBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamTest.py>`_\  (TestModels/), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


