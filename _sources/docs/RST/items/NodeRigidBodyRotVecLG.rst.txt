

.. _sec-item-noderigidbodyrotveclg:

NodeRigidBodyRotVecLG
=====================

A 3D rigid body node based on rotation vector and Lie group methods for rigid bodies; the node has 3 displacement coordinates and three rotation coordinates and can be used in combination with explicit Lie Group time integration methods.

Authors: Gerstmayr Johannes, Holzinger Stefan

\ **Additional information for NodeRigidBodyRotVecLG**\ :

* | The Node has the following types = \ ``Position``\ , \ ``Orientation``\ , \ ``RigidBody``\ , \ ``RotationRotationVector``\ 
* | \ **Short name**\  for Python = \ ``RigidRotVecLG``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRigidRotVecLG``\ 


The item \ **NodeRigidBodyRotVecLG**\  with type = 'RigidBodyRotVecLG' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [\ :math:`{\mathbf{q}}\cRef = [q_0,\,q_1,\,q_2,\,\nu_0,\,\nu_1,\,\nu_2]\tp\cRef = [{\mathbf{p}}\tp\cRef,\,\tnu\tp\cRef]\tp`\ , type = Vector6D, size = 3, default = [0.,0.,0., 0.,0.,0.]]:
  | reference coordinates (position and rotation vector \ :math:`\tnu`\ ) of node ==> e.g. ref. coordinates for finite elements or reference position of rigid body (e.g. for definition of joints)
* | **initialCoordinates** [\ :math:`{\mathbf{q}}\cIni = [q_0,\,q_1,\,q_2,\,\nu_0,\,\nu_1,\,\nu_2]\tp\cIni = [{\mathbf{u}}\tp\cIni,\,\tnu\tp\cIni]\tp`\ , type = Vector6D, size = 3, default = [0.,0.,0., 0.,0.,0.]]:
  | initial displacement coordinates \ :math:`{\mathbf{u}}`\  and rotation vector \ :math:`\tnu`\  relative to reference coordinates
* | **initialVelocities** [\ :math:`\dot {\mathbf{q}}\cIni = [\dot q_0,\,\dot q_1,\,\dot q_2,\,\dot \nu_0,\,\dot \nu_1,\,\dot \nu_2]\tp\cIni = [\dot {\mathbf{u}}\tp\cIni,\,\dot \tnu\tp\cIni]\tp`\ , type = Vector6D, size = 3, default = [0.,0.,0., 0.,0.,0.]]:
  | initial velocity coordinate: time derivatives of displacement and angular velocity vector
* | **visualization** [type = VNodeRigidBodyRotVecLG]:
  | parameters for visualization of item



The item VNodeRigidBodyRotVecLG has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, size = 4, default = [-1.,-1.,-1.,-1.]]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}\cConfig = \LU{0}{[p_0,\,p_1,\,p_2]}\cConfig\tp= \LU{0}{{\mathbf{u}}}\cConfig + \LU{0}{{\mathbf{p}}}\cRef`\ 
  | global 3D position vector of node; \ :math:`{\mathbf{u}}\cRef=0`\ 
* | ``Displacement``\ : \ :math:`\LU{0}{{\mathbf{u}}}\cConfig = [q_0,\,q_1,\,q_2]\cConfig\tp`\ 
  | global 3D displacement vector of node
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}\cConfig = [\dot q_0,\,\dot q_1,\,\dot q_2]\cConfig\tp`\ 
  | global 3D velocity vector of node
* | ``Coordinates``\ : \ :math:`{\mathbf{c}}\cConfig = [q_0,\,q_1,\,q_2, \,\nu_0,\,\nu_1,\,\nu_2]\tp\cConfig`\ 
  | coordinate vector of node, having 3 displacement coordinates and 3 Euler angles
* | ``Coordinates\_t``\ : \ :math:`\dot{\mathbf{c}}\cConfig = [\dot q_0,\,\dot q_1,\,\dot q_2, \,\dot \nu_0,\,\dot \nu_1,\,\dot \nu_2]\tp\cConfig`\ 
  | velocity coordinates vector of node
* | ``RotationMatrix``\ : \ :math:`[A_{00},\,A_{01},\,A_{02},\,A_{10},\,\ldots,\,A_{21},\,A_{22}]\cConfig\tp`\ 
  | vector with 9 components of the rotation matrix \ :math:`\LU{0b}{\Rot}\cConfig`\  in row-major format, in any configuration; the rotation matrix transforms local (\ :math:`b`\ ) to global (0) coordinates
* | ``Rotation``\ : \ :math:`[\varphi_0,\,\varphi_1,\,\varphi_2]\tp\cConfig`\ 
  | vector with 3 components of the Euler/Tait-Bryan angles in xyz-sequence (\ :math:`\LU{0b}{\Rot}\cConfig=:\Rot_0(\varphi_0) \cdot \Rot_1(\varphi_1) \cdot \Rot_2(\varphi_2)`\ ), recomputed from rotation matrix
* | ``AngularVelocity``\ : \ :math:`\LU{0}{\tomega}\cConfig = \LU{0}{[\omega_0,\,\omega_1,\,\omega_2]}\cConfig\tp`\ 
  | global 3D angular velocity vector of node
* | ``AngularVelocityLocal``\ : \ :math:`\LU{b}{\tomega}\cConfig = \LU{b}{[\omega_0,\,\omega_1,\,\omega_2]}\cConfig\tp`\ 
  | local (body-fixed)  3D angular velocity vector of node




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


