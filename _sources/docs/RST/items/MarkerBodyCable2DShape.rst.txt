

.. _sec-item-markerbodycable2dshape:

MarkerBodyCable2DShape
======================

A special Marker attached to a 2D ANCF beam finite element with cubic interpolation and 8 coordinates.

\ **Additional information for MarkerBodyCable2DShape**\ :

* | The Marker has the following types = \ ``Object``\ , \ ``Body``\ , \ ``Coordinate``\ 


The item \ **MarkerBodyCable2DShape**\  with type = 'BodyCable2DShape' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **bodyNumber** [type = ObjectIndex, default = invalid (-1)]:
  | body number to which marker is attached to
* | **numberOfSegments** [type = PInt, default = 3]:
  | number of number of segments; each segment is a line and is associated to a data (history) variable; must be same as in according contact element
* | **verticalOffset** [type = Real, default = 0.]:
  | vertical offset from beam axis in positive (local) Y-direction; this offset accounts for consistent computation of positions and velocities at the surface of the beam
* | **visualization** [type = VMarkerBodyCable2DShape]:
  | parameters for visualization of item



The item VMarkerBodyCable2DShape has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


