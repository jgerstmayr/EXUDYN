

.. _sec-item-loadcoordinate:

LoadCoordinate
==============

Load with scalar value, which is attached to a coordinate-based marker; the load can be used e.g. to apply a force to a single axis of a body, a nodal coordinate of a finite element  or a torque to the rotatory DOF of a rigid body.

\ **Additional information for LoadCoordinate**\ :

* | Requested marker type = \ ``Coordinate``\ 


The item \ **LoadCoordinate**\  with type = 'Coordinate' has the following parameters:

* | **name** [type = String, default = '']:
  | load's unique name
* | **markerNumber** [type = MarkerIndex, default = invalid (-1)]:
  | marker's number to which load is applied
* | **load** [\ :math:`f`\ , type = Real, default = 0.]:
  | scalar load [SI:N]; in case of a user function, this value is ignored
* | **loadUserFunction** [\ :math:`\mathrm{UF} \in \Rcal`\ , type = PyFunctionMbsScalar2, default =  0]:
  | A Python function which defines the time-dependent load and replaces the load; see description below; see also notes on loadFactor and drawing in LoadForceVector!
* | **visualization** [type = VLoadCoordinate]:
  | parameters for visualization of item



The item VLoadCoordinate has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


