

.. _sec-item-nodepoint3dslope1:

NodePoint3DSlope1
=================

A 3D point/slope vector node for spatial Bernoulli-Euler ANCF (absolute nodal coordinate formulation) beam elements; the node has 6 displacement degrees of freedom (3 for displacement of point node and 3 for the slope vector 'slopex'); all coordinates lead to second order differential equations; the slope vector defines the directional derivative w.r.t the local axial (x) coordinate, denoted as \ :math:`()^\prime`\ ; in straight configuration aligned at the global x-axis, the slope vector reads \ :math:`{\mathbf{r}}^\prime=[r_x^\prime\;\;r_y^\prime\;\;r_z^\prime]^T=[1\;\;0]^T`\ .
 



The item \ **NodePoint3DSlope1**\  with type = 'Point3DSlope1' has the following parameters:

 

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [type = Vector6D, default = [0.,0.,0.,1.,0.,0.], size = 6]:
  | reference coordinates (x-pos,y-pos,z-pos; x-slopex, y-slopex, z-slopex) of node; global position of node without displacement
* | **initialCoordinates** [type = Vector6D, default = [0.,0.,0.,0.,0.,0.], size = 6]:
  | initial displacement coordinates: ux, uy, uz and x/y/z 'displacements' of slopex
* | **initialVelocities** [type = Vector6D, default = [0.,0.,0.,0.,0.,0.], size = 6]:
  | initial velocity coordinates



The item VNodePoint3DSlope1 has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.], size = 4]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


