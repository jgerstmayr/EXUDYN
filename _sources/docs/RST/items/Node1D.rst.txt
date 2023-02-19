

.. _sec-item-node1d:

Node1D
======

A node with one ODE2 coordinate for one dimensional (1D) problems; use e.g. for scalar dynamic equations (Mass1D) and mass-spring-damper mechanisms, representing either translational or rotational degrees of freedom: in most cases, Node1D is equivalent to NodeGenericODE2 using one coordinate, however, it offers a transformation to 3D translational or rotational motion and allows to couple this node to 2D or 3D bodies.
 



The item \ **Node1D**\  with type = '1D' has the following parameters:

 

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [type = Vector, default = [0.]]:
  | reference coordinate of node (in vector form)
* | **initialCoordinates** [type = Vector, default = [0.]]:
  | initial displacement coordinate (in vector form)
* | **initialVelocities** [type = Vector, default = [0.]]:
  | initial velocity coordinate (in vector form)



The item VNode1D has the following parameters:

 

* | **show** [type = Bool, default = False]:
  | set true, if item is shown in visualization and false if it is not shown; The node1D is represented as reference position and displacement along the global x-axis, which must not agree with the representation in the object using the Node1D




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


