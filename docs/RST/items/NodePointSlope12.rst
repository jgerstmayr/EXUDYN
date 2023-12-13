

.. _sec-item-nodepointslope12:

NodePointSlope12
================

A 3D point/slope vector node for thin ANCF (absolute nodal coordinate formulation) plate elements; the node has 9 ODE2 degrees of freedom (3 for displacement of point node and 2 \ :math:`\times`\  3 for the slope vectors 'slopeX' and 'slopeY'); all coordinates lead to second order differential equations; the slopeX vector defines the directional derivative w.r.t the local axial (x) coordinate, etc.; in straight configuration aligned at the global x-axis, the slopeY vector reads \ :math:`{\mathbf{r}}_y^\prime=[0\;\;1\;\;0]^T`\ .

\ **Additional information for NodePointSlope12**\ :

* | This \ ``Node``\  has/provides the following types = \ ``Position``\ , \ ``Orientation``\ 


The item \ **NodePointSlope12**\  with type = 'PointSlope12' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [type = Vector9D, size = 9, default = [0.,0.,0.,1.,0.,0.,1.,0.,0.]]:
  | reference coordinates (x-pos,y-pos,z-pos; x-slopeX, y-slopeX, z-slopeX; x-slopeY, y-slopeY, z-slopeY) of node; global position of node without displacement
* | **initialCoordinates** [type = Vector9D, size = 9, default = [0.,0.,0.,0.,0.,0.,0.,0.,0.]]:
  | initial displacement coordinates relative to reference coordinates
* | **initialVelocities** [type = Vector9D, size = 9, default = [0.,0.,0.,0.,0.,0.,0.,0.,0.]]:
  | initial velocity coordinates
* | **visualization** [type = VNodePointSlope12]:
  | parameters for visualization of item



The item VNodePointSlope12 has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, size = 4, default = [-1.,-1.,-1.,-1.]]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used


----------

.. _description-nodepointslope12:

DESCRIPTION of NodePointSlope12
-------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}\cConfig = \LU{0}{[p_0,\, p_1,\, p_2]}\cConfig\tp`\ 
  | global 3D position vector of node (=displacement+reference position)
* | ``Displacement``\ : \ :math:`\LU{0}{{\mathbf{u}}}\cConfig = \LU{0}{[q_0,\, q_1,\, q_2]}\cConfig\tp`\ 
  | global 3D displacement vector of node
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{a}}}\cConfig = \LU{0}{[\dot q_0,\,\dot q_1,\,\dot q_2]}\cConfig\tp`\ 
  | global 3D velocity vector of node
* | ``Acceleration``\ : \ :math:`\LU{0}{{\mathbf{a}}}\cConfig = \LU{0}{[\ddot q_0,\,\ddot q_1,\,\ddot q_2]}\cConfig\tp`\ 
  | global 3D acceleration vector of node
* | ``Coordinates``\ : 
  | coordinate vector of node (relative to reference configuration)
* | ``Coordinates\_t``\ : 
  | velocity coordinates vector of node
* | ``Coordinates\_tt``\ : 
  | acceleration coordinates vector of node
* | ``RotationMatrix``\ : \ :math:`[A_{00},\,A_{01},\,A_{02},\,A_{10},\,\ldots,\,A_{21},\,A_{22}]\cConfig\tp`\ 
  | vector with 9 components of the rotation matrix \ :math:`\LU{0b}{\Rot}\cConfig`\  in row-major format, in any configuration; the rotation matrix transforms local (\ :math:`b`\ ) to global (0) coordinates
* | ``Rotation``\ : \ :math:`[\varphi_0,\,\varphi_1,\,\varphi_2]\tp\cConfig`\ 
  | vector with 3 components of the Euler / Tait-Bryan angles in xyz-sequence
* | ``AngularVelocity``\ : \ :math:`\LU{0}{\tomega}\cConfig = \LU{0}{[\omega_0,\,\omega_1,\,\omega_2]}\cConfig\tp`\ 
  | global 3D angular velocity vector of node
* | ``AngularVelocityLocal``\ : \ :math:`\LU{b}{\tomega}\cConfig = \LU{b}{[\omega_0,\,\omega_1,\,\omega_2]}\cConfig\tp`\ 
  | local (body-fixed)  3D angular velocity vector of node




\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


