

.. _sec-item-markernodecoordinate:

MarkerNodeCoordinate
====================

A node-Marker attached to a ODE2 coordinate of a node; this marker allows to connect a coordinate-based constraint or connector to a nodal coordinate (also NodeGround); for ODE1 coordinates use MarkerNodeODE1Coordinate.
 



The item \ **MarkerNodeCoordinate**\  with type = 'NodeCoordinate' has the following parameters:

 

* | **name** [type = String, default = '']:
  | marker's unique name
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number to which marker is attached to
* | **coordinate** [type = UInt, default = invalid (-1)]:
  | coordinate of node to which marker is attached to



The item VMarkerNodeCoordinate has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


