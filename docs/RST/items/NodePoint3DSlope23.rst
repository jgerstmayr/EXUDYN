

.. _sec-item-nodepoint3dslope23:

NodePoint3DSlope23
==================

A 3D point/slope vector node for spatial, shear and cross-section deformable ANCF (absolute nodal coordinate formulation) beam elements; the node has 9 ODE2 degrees of freedom (3 for displacement of point node and 2 \ :math:`\times`\  3 for the slope vectors 'slopey' and 'slopez'); all coordinates lead to second order differential equations; the slope vector defines the directional derivative w.r.t the local axial (x) coordinate, denoted as \ :math:`()^\prime`\ ; in straight configuration aligned at the global x-axis, the slopey vector reads \ :math:`{\mathbf{r}}_y^\prime=[0\;\;1\;\;0]^T`\  and slopez gets \ :math:`{\mathbf{r}}_z^\prime=[0\;\;0\;\;1]^T`\ .

\ **Additional information for NodePoint3DSlope23**\ :

* | This \ ``Node``\  has/provides the following types = \ ``Position``\ , \ ``Orientation``\ 
* | \ **Short name**\  for Python = \ ``Point3DS23``\ 
* | \ **Short name**\  for Python visualization object = \ ``VPoint3DS23``\ 


The item \ **NodePoint3DSlope23**\  with type = 'Point3DSlope23' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [type = Vector9D, size = 9, default = [0.,0.,0.,1.,0.,0.,1.,0.,0.]]:
  | reference coordinates (x-pos,y-pos,z-pos; x-slopey, y-slopey, z-slopey; x-slopez, y-slopez, z-slopez) of node; global position of node without displacement
* | **initialCoordinates** [type = Vector9D, size = 9, default = [0.,0.,0.,0.,0.,0.,0.,0.,0.]]:
  | initial displacement coordinates relative to reference coordinates
* | **initialVelocities** [type = Vector9D, size = 9, default = [0.,0.,0.,0.,0.,0.,0.,0.,0.]]:
  | initial velocity coordinates
* | **visualization** [type = VNodePoint3DSlope23]:
  | parameters for visualization of item



The item VNodePoint3DSlope23 has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, size = 4, default = [-1.,-1.,-1.,-1.]]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used


----------

.. _description-nodepoint3dslope23:

DESCRIPTION of NodePoint3DSlope23
---------------------------------

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



Relevant Examples and TestModels with weblink:

    \ `ANCFBeamEigTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamEigTest.py>`_\  (TestModels/), \ `ANCFBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamTest.py>`_\  (TestModels/), \ `geometricallyExactBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/geometricallyExactBeamTest.py>`_\  (TestModels/), \ `rightAngleFrame.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rightAngleFrame.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


