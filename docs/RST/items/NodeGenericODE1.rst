

.. _sec-item-nodegenericode1:

NodeGenericODE1
===============

A node containing a number of ODE1 variables; use e.g. linear state space systems. Note that referenceCoordinates and initialCoordinates must be initialized, because no default values exist.
 



The item \ **NodeGenericODE1**\  with type = 'GenericODE1' has the following parameters:

 

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [type = Vector, default = []]:
  | generic reference coordinates of node; must be consistent with numberOfODE1Coordinates
* | **initialCoordinates** [type = Vector, default = []]:
  | initial displacement coordinates; must be consistent with numberOfODE1Coordinates
* | **numberOfODE1Coordinates** [type = PInt, default = 0]:
  | number of generic ODE1 coordinates



The item VNodeGenericODE1 has the following parameters:

 

* | **show** [type = Bool, default = False]:
  | set true, if item is shown in visualization and false if it is not shown




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


