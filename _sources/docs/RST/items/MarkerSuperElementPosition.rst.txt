

.. _sec-item-markersuperelementposition:

MarkerSuperElementPosition
==========================

A position marker attached to a SuperElement, such as ObjectFFRF, ObjectGenericODE2 and ObjectFFRFreducedOrder (for which it is in its current implementation inefficient for large number of meshNodeNumbers). The marker acts on the mesh (interface) nodes, not on the underlying nodes of the object.
 



The item \ **MarkerSuperElementPosition**\  with type = 'SuperElementPosition' has the following parameters:

 

* | **name** [type = String, default = '']:
  | marker's unique name
* | **bodyNumber** [type = ObjectIndex, default = invalid (-1)]:
  | body number to which marker is attached to
* | **meshNodeNumbers** [type = ArrayIndex, default = []]:
  | a list of \ :math:`n_m`\  mesh node numbers of superelement (=interface nodes) which are used to compute the body-fixed marker position; the related nodes must provide 3D position information, such as NodePoint, NodePoint2D, NodeRigidBody[..]; in order to retrieve the global node number, the generic body needs to convert local into global node numbers
* | **weightingFactors** [type = Vector, default = []]:
  | a list of \ :math:`n_m`\  weighting factors per node to compute the final local position; the sum of these weights shall be 1, such that a summation of all nodal positions times weights gives the average position of the marker



The item VMarkerSuperElementPosition has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **showMarkerNodes** [type = Bool, default = True]:
  | set true, if all nodes are shown (similar to marker, but with less intensity)




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


