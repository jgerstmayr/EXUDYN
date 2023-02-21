

.. _sec-item-markerobjectode2coordinates:

MarkerObjectODE2Coordinates
===========================

A Marker attached to all coordinates of an object (currently only body is possible), e.g. to apply special constraints or loads on all coordinates. The measured coordinates INCLUDE reference + current coordinates.

\ **Additional information for MarkerObjectODE2Coordinates**\ :

* | The Marker has the following types = \ ``Object``\ , \ ``Body``\ , \ ``Coordinate``\ 


The item \ **MarkerObjectODE2Coordinates**\  with type = 'ObjectODE2Coordinates' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **objectNumber** [type = ObjectIndex, default = invalid (-1)]:
  | body number to which marker is attached to
* | **visualization** [type = VMarkerObjectODE2Coordinates]:
  | parameters for visualization of item



The item VMarkerObjectODE2Coordinates has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


