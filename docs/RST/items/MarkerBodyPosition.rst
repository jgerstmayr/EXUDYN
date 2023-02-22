

.. _sec-item-markerbodyposition:

MarkerBodyPosition
==================

A position body-marker attached to a local (body-fixed) position \ :math:`\pLocB = [b_0,\; b_1,\; b_2]`\  (\ :math:`x`\ , \ :math:`y`\ , and \ :math:`z`\  coordinates) of the body.

\ **Additional information for MarkerBodyPosition**\ :

* | The Marker has the following types = \ ``Object``\ , \ ``Body``\ , \ ``Position``\ 


The item \ **MarkerBodyPosition**\  with type = 'BodyPosition' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **bodyNumber** [type = ObjectIndex, default = invalid (-1)]:
  | body number to which marker is attached to
* | **localPosition** [\ :math:`\pLocB`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | local body position of marker; e.g. local (body-fixed) position where force is applied to
* | **visualization** [type = VMarkerBodyPosition]:
  | parameters for visualization of item



The item VMarkerBodyPosition has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown




\ **The web version may not be complete. For details, always consider the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


