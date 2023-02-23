

.. _sec-item-markernodecoordinate:

MarkerNodeCoordinate
====================

A node-Marker attached to a \ :ref:`ODE2 <ODE2>`\  coordinate of a node; this marker allows to connect a coordinate-based constraint or connector to a nodal coordinate (also NodeGround); for \ :ref:`ODE1 <ODE1>`\  coordinates use MarkerNodeODE1Coordinate.

\ **Additional information for MarkerNodeCoordinate**\ :

* | The Marker has the following types = \ ``Node``\ , \ ``Coordinate``\ 


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




\ **The web version may not be complete. For details, always consider the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


