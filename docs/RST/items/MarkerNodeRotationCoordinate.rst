

.. _sec-item-markernoderotationcoordinate:

MarkerNodeRotationCoordinate
============================

A node-Marker attached to a a node containing rotation; the Marker measures a rotation coordinate (Tait-Bryan angles) or angular velocities on the velocity level.

\ **Additional information for MarkerNodeRotationCoordinate**\ :

* | This \ ``Marker``\  has/provides the following types = \ ``Node``\ , \ ``Coordinate``\ 


The item \ **MarkerNodeRotationCoordinate**\  with type = 'NodeRotationCoordinate' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number to which marker is attached to
* | **rotationCoordinate** [type = UInt, default = invalid (-1)]:
  | rotation coordinate: 0=x, 1=y, 2=z
* | **visualization** [type = VMarkerNodeRotationCoordinate]:
  | parameters for visualization of item



The item VMarkerNodeRotationCoordinate has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-markernoderotationcoordinate:

DESCRIPTION of MarkerNodeRotationCoordinate
-------------------------------------------

Relevant Examples and TestModels with weblink:

    \ `openVRengine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openVRengine.py>`_\  (Examples/), \ `pistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pistonEngine.py>`_\  (Examples/), \ `rigidRotor3DbasicBehaviour.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidRotor3DbasicBehaviour.py>`_\  (Examples/), \ `sliderCrank3DwithANCFbeltDrive2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive2.py>`_\  (Examples/), \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TestModels/), \ `sliderCrank3Dbenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dbenchmark.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


