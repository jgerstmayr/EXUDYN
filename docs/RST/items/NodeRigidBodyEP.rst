

.. _sec-item-noderigidbodyep:

NodeRigidBodyEP
===============

A 3D rigid body node based on Euler parameters for rigid bodies or beams; the node has 3 displacement coordinates (representing displacement of reference point \ :math:`\LU{0}{{\mathbf{r}}}`\ ) and four rotation coordinates (Euler parameters = unit quaternions).

\ **Additional information for NodeRigidBodyEP**\ :

* | The Node has the following types = \ ``Position``\ , \ ``Orientation``\ , \ ``RigidBody``\ , \ ``RotationEulerParameters``\ 
* | \ **Short name**\  for Python = \ ``RigidEP``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRigidEP``\ 


The item \ **NodeRigidBodyEP**\  with type = 'RigidBodyEP' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [\ :math:`{\mathbf{q}}\cRef = [q_0,\,q_1,\,q_2,\,\psi_0,\,\psi_1,\,\psi_2,\,\psi_3]\tp\cRef = [{\mathbf{p}}\tp\cRef,\,\tpsi\tp\cRef]\tp`\ , type = Vector7D, size = 7, default = [0.,0.,0., 0.,0.,0.,0.]]:
  | reference coordinates (3 position coordinates and 4 Euler parameters) of node ==> e.g. ref. coordinates for finite elements or reference position of rigid body (e.g. for definition of joints)
* | **initialCoordinates** [\ :math:`{\mathbf{q}}\cIni = [q_0,\,q_1,\,q_2,\,\psi_0,\,\psi_1,\,\psi_2,\,\psi_3]\tp\cIni = [{\mathbf{u}}\tp\cIni,\,\tpsi\tp\cIni]\tp`\ , type = Vector7D, size = 7, default = [0.,0.,0., 0.,0.,0.,0.]]:
  | initial displacement coordinates and 4 Euler parameters relative to reference coordinates
* | **initialVelocities** [\ :math:`\dot {\mathbf{q}}\cIni = [\dot q_0,\,\dot q_1,\,\dot q_2,\,\dot \psi_0,\,\dot \psi_1,\,\dot \psi_2,\,\dot \psi_3]\tp\cIni = [\dot {\mathbf{u}}\tp\cIni,\,\dot \tpsi\tp\cIni]\tp`\ , type = Vector7D, size = 7, default = [0.,0.,0., 0.,0.,0.,0.]]:
  | initial velocity coordinates: time derivatives of initial displacements and Euler parameters
* | **addConstraintEquation** [type = Bool, default = True]:
  | True: automatically add Euler parameter constraint for node; False: Euler parameter constraint is not added, must be done manually (e.g., with CoordinateVectorConstraint)
* | **visualization** [type = VNodeRigidBodyEP]:
  | parameters for visualization of item



The item VNodeRigidBodyEP has the following parameters:

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
* | ``Acceleration``\ : \ :math:`\LU{0}{{\mathbf{a}}}\cConfig = [\ddot q_0,\,\ddot q_1,\,\ddot q_2]\cConfig\tp`\ 
  | global 3D acceleration vector of node
* | ``Coordinates``\ : \ :math:`{\mathbf{c}}\cConfig = [q_0,\,q_1,\,q_2, \,\psi_0,\,\psi_1,\,\psi_2,\,\psi_3]\tp\cConfig`\ 
  | coordinate vector of node, having 3 displacement coordinates and 4 Euler parameters
* | ``Coordinates\_t``\ : \ :math:`\dot{\mathbf{c}}\cConfig = [\dot q_0,\,\dot q_1,\,\dot q_2, \,\dot \psi_0,\,\dot \psi_1,\,\dot \psi_2,\,\dot \psi_3]\tp\cConfig`\ 
  | velocity coordinates vector of node
* | ``Coordinates\_tt``\ : \ :math:`\ddot{\mathbf{c}}\cConfig = [\ddot q_0,\,\ddot q_1,\,\ddot q_2, \,\ddot \psi_0,\,\ddot \psi_1,\,\ddot \psi_2,\,\ddot \psi_3]\tp\cConfig`\ 
  | acceleration coordinates vector of node
* | ``RotationMatrix``\ : \ :math:`[A_{00},\,A_{01},\,A_{02},\,A_{10},\,\ldots,\,A_{21},\,A_{22}]\cConfig\tp`\ 
  | vector with 9 components of the rotation matrix \ :math:`\LU{0b}{\Rot}\cConfig`\  in row-major format, in any configuration; the rotation matrix transforms local (\ :math:`b`\ ) to global (0) coordinates
* | ``Rotation``\ : \ :math:`[\varphi_0,\,\varphi_1,\,\varphi_2]\tp\cConfig`\ 
  | vector with 3 components of the Euler/Tait-Bryan angles in xyz-sequence (\ :math:`\LU{0b}{\Rot}\cConfig=:\Rot_0(\varphi_0) \cdot \Rot_1(\varphi_1) \cdot \Rot_2(\varphi_2)`\ ), recomputed from rotation matrix
* | ``AngularVelocity``\ : \ :math:`\LU{0}{\tomega}\cConfig = \LU{0}{[\omega_0,\,\omega_1,\,\omega_2]}\cConfig\tp`\ 
  | global 3D angular velocity vector of node
* | ``AngularVelocityLocal``\ : \ :math:`\LU{b}{\tomega}\cConfig = \LU{b}{[\omega_0,\,\omega_1,\,\omega_2]}\cConfig\tp`\ 
  | local (body-fixed)  3D angular velocity vector of node
* | ``AngularAcceleration``\ : \ :math:`\LU{0}{\talpha}\cConfig = \LU{0}{[\alpha_0,\,\alpha_1,\,\alpha_2]}\cConfig\tp`\ 
  | global 3D angular acceleration vector of node




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


