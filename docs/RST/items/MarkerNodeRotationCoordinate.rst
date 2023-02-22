

.. _sec-item-markernoderotationcoordinate:

MarkerNodeRotationCoordinate
============================

A node-Marker attached to a a node containing rotation; the Marker measures a rotation coordinate (Tait-Bryan angles) or angular velocities on the velocity level.

\ **Additional information for MarkerNodeRotationCoordinate**\ :

* | The Marker has the following types = \ ``Node``\ , \ ``Orientation``\ , \ ``Coordinate``\ 


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




\ **The web version may not be complete. For details, always consider the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


