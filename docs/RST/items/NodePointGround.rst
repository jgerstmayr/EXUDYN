

.. _sec-item-nodepointground:

NodePointGround
===============

A 3D point node fixed to ground. The node can be used as NodePoint, but it does not generate coordinates. Applied or reaction forces do not have any effect. This node can be used for 'blind' or 'dummy' ODE2 and ODE1 coordinates to which CoordinateSpringDamper or CoordinateConstraint objects are attached to.
 



The item \ **NodePointGround**\  with type = 'PointGround' has the following parameters:

 

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [type = Vector3D, default = [0.,0.,0.], size = 3]:
  | reference coordinates of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement



The item VNodePointGround has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.], size = 4]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


