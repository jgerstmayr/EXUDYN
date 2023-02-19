

.. _sec-item-nodepoint2d:

NodePoint2D
===========

A 2D point node for point masses or solid finite elements which has 2 displacement degrees of freedom for second order differential equations.
 



The item \ **NodePoint2D**\  with type = 'Point2D' has the following parameters:

 

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [type = Vector2D, default = [0.,0.], size = 2]:
  | reference coordinates of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement
* | **initialCoordinates** [type = Vector2D, default = [0.,0.], size = 2]:
  | initial displacement coordinate
* | **initialVelocities** [type = Vector2D, default = [0.,0.], size = 2]:
  | initial velocity coordinate



The item VNodePoint2D has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.], size = 4]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


