

.. _sec-item-nodepoint3dslope23:

NodePoint3DSlope23
==================

A 3D point/slope vector node for spatial, shear and cross-section deformable ANCF (absolute nodal coordinate formulation) beam elements; the node has 9 ODE2 degrees of freedom (3 for displacement of point node and 2 \ :math:`\times`\  3 for the slope vectors 'slopey' and 'slopez'); all coordinates lead to second order differential equations; the slope vector defines the directional derivative w.r.t the local axial (x) coordinate, denoted as \ :math:`()^\prime`\ ; in straight configuration aligned at the global x-axis, the slopey vector reads \ :math:`{\mathbf{r}}_y^\prime=[0\;\;1\;\;0]^T`\  and slopez gets \ :math:`{\mathbf{r}}_z^\prime=[0\;\;0\;\;1]^T`\ .
 



The item \ **NodePoint3DSlope23**\  with type = 'Point3DSlope23' has the following parameters:

 

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [type = Vector9D, default = [0.,0.,0.,1.,0.,0.,1.,0.,0.], size = 9]:
  | reference coordinates (x-pos,y-pos,z-pos; x-slopey, y-slopey, z-slopey; x-slopez, y-slopez, z-slopez) of node; global position of node without displacement
* | **initialCoordinates** [type = Vector9D, default = [0.,0.,0.,0.,0.,0.,0.,0.,0.], size = 9]:
  | initial displacement coordinates relative to reference coordinates
* | **initialVelocities** [type = Vector9D, default = [0.,0.,0.,0.,0.,0.,0.,0.,0.], size = 9]:
  | initial velocity coordinates



The item VNodePoint3DSlope23 has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.], size = 4]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


