

.. _sec-item-nodepointslope1:

NodePointSlope1
===============

A 3D point/slope vector node for spatial Bernoulli-Euler ANCF (absolute nodal coordinate formulation) beam elements; the node has 6 displacement degrees of freedom (3 for displacement of point node and 3 for the slope vector 'slopex'); all coordinates lead to second order differential equations; the slope vector defines the directional derivative w.r.t the local axial (x) coordinate, denoted as \ :math:`()^\prime`\ ; in straight configuration aligned at the global x-axis, the slope vector reads \ :math:`{\mathbf{r}}^\prime=[r_x^\prime\;\;r_y^\prime\;\;r_z^\prime]^T=[1\;\;0]^T`\ .

\ **Additional information for NodePointSlope1**\ :

* | This \ ``Node``\  has/provides the following types = \ ``Position``\ 


The item \ **NodePointSlope1**\  with type = 'PointSlope1' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [type = Vector6D, size = 6, default = [0.,0.,0.,1.,0.,0.]]:
  | reference coordinates (x-pos,y-pos,z-pos; x-slopex, y-slopex, z-slopex) of node; global position of node without displacement
* | **initialCoordinates** [type = Vector6D, size = 6, default = [0.,0.,0.,0.,0.,0.]]:
  | initial displacement coordinates: ux, uy, uz and x/y/z 'displacements' of slopex
* | **initialVelocities** [type = Vector6D, size = 6, default = [0.,0.,0.,0.,0.,0.]]:
  | initial velocity coordinates
* | **visualization** [type = VNodePointSlope1]:
  | parameters for visualization of item



The item VNodePointSlope1 has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, size = 4, default = [-1.,-1.,-1.,-1.]]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used


----------

.. _description-nodepointslope1:

DESCRIPTION of NodePointSlope1
------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}\cConfig = [p_0,\, p_1,\, p_2]\cConfig\tp`\ 
  | global 3D position vector of node (=displacement+reference position)
* | ``Displacement``\ : \ :math:`\LU{0}{{\mathbf{u}}}\cConfig = [q_0,\, q_1,\, q_2]\cConfig\tp`\ 
  | global 3D displacement vector of node
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{a}}}\cConfig = [\dot q_0,\,\dot q_1,\,\dot q_2]\cConfig\tp`\ 
  | global 3D velocity vector of node
* | ``Acceleration``\ : \ :math:`\LU{0}{{\mathbf{a}}}\cConfig = [\ddot q_0,\,\ddot q_1,\,\ddot q_2]\cConfig\tp`\ 
  | global 3D acceleration vector of node
* | ``Coordinates``\ : 
  | coordinates vector of node (3 displacement coordinates + 3 slope vector coordinates)
* | ``Coordinates\_t``\ : 
  | velocity coordinates vector of node (derivative of the 3 displacement coordinates + 3 slope vector coordinates)
* | ``Coordinates\_tt``\ : 
  | acceleration coordinates vector of node (derivative of the 3 displacement coordinates + 3 slope vector coordinates)




\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


