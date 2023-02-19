

.. _sec-item-nodegenericode2:

NodeGenericODE2
===============

A node containing a number of ODE2 variables; use e.g. for scalar dynamic equations (Mass1D) or for the ALECable element. Note that referenceCoordinates and all initialCoordinates(_t) must be initialized, because no default values exist.
 



The item \ **NodeGenericODE2**\  with type = 'GenericODE2' has the following parameters:

 

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [type = Vector, default = []]:
  | generic reference coordinates of node; must be consistent with numberOfODE2Coordinates
* | **initialCoordinates** [type = Vector, default = []]:
  | initial displacement coordinates; must be consistent with numberOfODE2Coordinates
* | **initialCoordinates_t** [type = Vector, default = []]:
  | initial velocity coordinates; must be consistent with numberOfODE2Coordinates
* | **numberOfODE2Coordinates** [type = PInt, default = 0]:
  | number of generic ODE2 coordinates



The item VNodeGenericODE2 has the following parameters:

 

* | **show** [type = Bool, default = False]:
  | set true, if item is shown in visualization and false if it is not shown




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


