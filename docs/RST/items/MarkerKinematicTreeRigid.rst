

.. _sec-item-markerkinematictreerigid:

MarkerKinematicTreeRigid
========================

A position and orientation (rigid-body) marker attached to a kinematic tree. The marker is attached to the ObjectKinematicTree object and additionally needs a link number as well as a local position, similar to the SensorKinematicTree. The marker allows to attach loads (LoadForceVector and LoadTorqueVector) at arbitrary links or position. It also allows to attach connectors (e.g., spring dampers or actuators) to the kinematic tree. Finally, joint constraints can be attached, which allows for realization of closed loop structures. NOTE, however, that it is less efficient to attach many markers to a kinematic tree, therefor for forces or joint control use the structures available in kinematic tree whenever possible.

\ **Additional information for MarkerKinematicTreeRigid**\ :

* | The Marker has the following types = \ ``Object``\ , \ ``Body``\ , \ ``Position``\ , \ ``Orientation``\ 


The item \ **MarkerKinematicTreeRigid**\  with type = 'KinematicTreeRigid' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **objectNumber** [\ :math:`n_b`\ , type = ObjectIndex, default = invalid (-1)]:
  | body number to which marker is attached to
* | **linkNumber** [\ :math:`n_l`\ , type = UInt, default = invalid (-1)]:
  | number of link in KinematicTree to which marker is attached to
* | **localPosition** [\ :math:`\LU{l}{\bv}`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | local (link-fixed) position of marker at link \ :math:`n_l`\ , using the link (\ :math:`n_l`\ ) coordinate system
* | **visualization** [type = VMarkerKinematicTreeRigid]:
  | parameters for visualization of item



The item VMarkerKinematicTreeRigid has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 

