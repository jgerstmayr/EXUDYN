

.. _sec-item-markersuperelementrigid:

MarkerSuperElementRigid
=======================

A position and orientation (rigid-body) marker attached to a SuperElement, such as ObjectFFRF, ObjectGenericODE2 and ObjectFFRFreducedOrder (for which it may be inefficient). The marker acts on the mesh nodes, not on the underlying nodes of the object. Note that in contrast to the MarkerSuperElementPosition, this marker needs a set of interface nodes which are not aligned at one line, such that these node points can represent a rigid body motion. Note that definitions of marker positions are slightly different from MarkerSuperElementPosition.

\ **Additional information for MarkerSuperElementRigid**\ :

* | The Marker has the following types = \ ``Object``\ , \ ``Body``\ , \ ``Position``\ , \ ``Orientation``\ 


The item \ **MarkerSuperElementRigid**\  with type = 'SuperElementRigid' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **bodyNumber** [\ :math:`n_b`\ , type = ObjectIndex, default = invalid (-1)]:
  | body number to which marker is attached to
* | **offset** [\ :math:`\LU{r}{{\mathbf{o}}_{ref}}`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | local marker SuperElement reference position offset used to correct the center point of the marker, which is computed from the weighted average of reference node positions (which may have some offset to the desired joint position). Note that this offset shall be small and larger offsets can cause instability in simulation models (better to have symmetric meshes at joints).
* | **meshNodeNumbers** [\ :math:`[k_0,\,\ldots,\,k_{n_m-1}]\tp`\ , type = ArrayIndex, default = []]:
  | a list of \ :math:`n_m`\  mesh node numbers of superelement (=interface nodes) which are used to compute the body-fixed marker position and orientation; the related nodes must provide 3D position information, such as NodePoint, NodePoint2D, NodeRigidBody[..]; in order to retrieve the global node number, the generic body needs to convert local into global node numbers
* | **weightingFactors** [\ :math:`[w_{0},\,\ldots,\,w_{n_m-1}]\tp`\ , type = Vector, default = []]:
  | a list of \ :math:`n_m`\  weighting factors per node to compute the final local position and orientation; these factors could be based on surface integrals of the constrained mesh faces
* | **useAlternativeApproach** [type = Bool, default = True]:
  | this flag switches between two versions for the computation of the rotation and angular velocity of the marker; alternative approach uses skew symmetric matrix of reference position; follows the inertia concept
* | **visualization** [type = VMarkerSuperElementRigid]:
  | parameters for visualization of item



The item VMarkerSuperElementRigid has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **showMarkerNodes** [type = Bool, default = True]:
  | set true, if all nodes are shown (similar to marker, but with less intensity)




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


