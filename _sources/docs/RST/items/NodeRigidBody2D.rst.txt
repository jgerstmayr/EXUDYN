

.. _sec-item-noderigidbody2d:

NodeRigidBody2D
===============

A 2D rigid body node for rigid bodies or beams; the node has 2 displacement degrees of freedom and one rotation coordinate (rotation around z-axis: uphi). All coordinates are \ :ref:`ODE2 <ODE2>`\ , used for second order differetial equations.

\ **Additional information for NodeRigidBody2D**\ :

* | This \ ``Node``\  has/provides the following types = \ ``Position2D``\ , \ ``Orientation2D``\ , \ ``Position``\ , \ ``Orientation``\ , \ ``RigidBody``\ 
* | \ **Short name**\  for Python = \ ``Rigid2D``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRigid2D``\ 


The item \ **NodeRigidBody2D**\  with type = 'RigidBody2D' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [\ :math:`{\mathbf{q}}\cRef = [q_0,\,q_1,\,\psi_0]\tp\cRef`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | reference coordinates (x-pos,y-pos and rotation) of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement
* | **initialCoordinates** [\ :math:`{\mathbf{q}}\cIni = [q_0,\,q_1,\,\psi_0]\tp\cIni`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | initial displacement coordinates and angle (relative to reference coordinates)
* | **initialVelocities** [\ :math:`\dot {\mathbf{q}}\cIni = [\dot q_0,\,\dot q_1,\,\dot \psi_0]\tp\cIni =  [v_0,\,v_1,\,\omega_2]\tp\cIni`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | initial velocity coordinates
* | **visualization** [type = VNodeRigidBody2D]:
  | parameters for visualization of item



The item VNodeRigidBody2D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, size = 4, default = [-1.,-1.,-1.,-1.]]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used


----------

.. _description-noderigidbody2d:

DESCRIPTION of NodeRigidBody2D
------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}\cConfig = \LU{0}{[p_0,\,p_1,\,0]}\cConfig\tp= \LU{0}{{\mathbf{u}}}\cConfig + \LU{0}{{\mathbf{p}}}\cRef`\ 
  | global 3D position vector of node; \ :math:`{\mathbf{u}}\cRef=0`\ 
* | ``Displacement``\ : \ :math:`\LU{0}{{\mathbf{u}}}\cConfig = [q_0,\,q_1,\,0]\cConfig\tp`\ 
  | global 3D displacement vector of node
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}\cConfig = [\dot q_0,\,\dot q_1,\,0]\cConfig\tp`\ 
  | global 3D velocity vector of node
* | ``Acceleration``\ : \ :math:`\LU{0}{{\mathbf{a}}}\cConfig = [\ddot q_0,\,\ddot q_1,\,0]\cConfig\tp`\ 
  | global 3D acceleration vector of node
* | ``AngularVelocity``\ : \ :math:`\LU{0}{\tomega}\cConfig = \LU{0}{[0,\,0,\,\dot \psi_0]}\cConfig\tp`\ 
  | global 3D angular velocity vector of node
* | ``Coordinates``\ : \ :math:`{\mathbf{c}}\cConfig = [q_0,\,q_1,\,\psi_0]\tp\cConfig`\ 
  | coordinate vector of node, having 2 displacement coordinates and 1 angle
* | ``Coordinates\_t``\ : \ :math:`\dot{\mathbf{c}}\cConfig = [\dot q_0,\,\dot q_1,\,\dot \psi_0]\tp\cConfig`\ 
  | velocity coordinates vector of node
* | ``Coordinates\_tt``\ : \ :math:`\ddot{\mathbf{c}}\cConfig = [\ddot q_0,\,\ddot q_1,\,\ddot \psi_0]\tp\cConfig`\ 
  | acceleration coordinates vector of node
* | ``RotationMatrix``\ : \ :math:`[A_{00},\,A_{01},\,A_{02},\,A_{10},\,\ldots,\,A_{21},\,A_{22}]\cConfig\tp`\ 
  | vector with 9 components of the rotation matrix \ :math:`\LU{0b}{\Rot}\cConfig`\  in row-major format, in any configuration; the rotation matrix transforms local (\ :math:`b`\ ) to global (0) coordinates
* | ``Rotation``\ : \ :math:`[0,\,0,\,\theta_0]\tp\cConfig = [0,\,0,\,\psi_0]\tp\cRef + [0,\,0,\,\psi_0]\tp\cConfig`\ 
  | vector with 3rd angle around out of plane axis
* | ``AngularVelocityLocal``\ : \ :math:`\LU{b}{\tomega}\cConfig = \LU{b}{[0,\,0,\,\dot \psi_0]}\cConfig\tp`\ 
  | local (body-fixed)  3D angular velocity vector of node
* | ``AngularAcceleration``\ : \ :math:`\LU{0}{\talpha}\cConfig = \LU{0}{[0,\,0,\,\ddot \psi_0]}\cConfig\tp`\ 
  | global 3D angular acceleration vector of node



\ **Detailed information:** 
The node provides 2 displacement coordinates (displacement of \ :ref:`COM <COM>`\ , (\ :math:`q_0,q_1`\ ) ) and 1 rotation parameter (\ :math:`\theta_0`\ ). According equations need to be provided by an according object (e.g., RigidBody2D).
The node leads to 3 ODE2 equations of motions, where the first 2 equations are
residuals of global translational forces, and the third equation is the residual of the
torque around the Z-axis (due to planar motion, local=global).

Using the rotation parameter \ :math:`\theta_{0\mathrm{config}} = \psi_{0ref} + \psi_{0\mathrm{config}}`\ , the rotation matrix is defined as

.. math::

   \LU{0b}{\Rot}\cConfig = \mr{\cos(\theta_0)}{-\sin(\theta_0)}{0}{\sin(\theta_0)}{\cos(\theta_0)}{0}{0}{0}{1}\cConfig


\ **Example**\  for NodeRigidBody2D: see ObjectRigidBody2D


Relevant Examples and TestModels with weblink:

    \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Examples/), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Examples/), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Examples/), \ `doublePendulum2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/doublePendulum2D.py>`_\  (Examples/), \ `pendulumGeomExactBeam2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulumGeomExactBeam2D.py>`_\  (Examples/), \ `reevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/reevingSystem.py>`_\  (Examples/), \ `reevingSystemOpen.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/reevingSystemOpen.py>`_\  (Examples/), \ `simple4linkPendulumBing.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/simple4linkPendulumBing.py>`_\  (Examples/), \ `sliderCrank3DwithANCFbeltDrive2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive2.py>`_\  (Examples/), \ `ANCFmovingRigidbody.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFmovingRigidbody.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFslidingJoint2Drigid.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2Drigid.py>`_\  (Examples/), \ `ANCFBeamEigTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamEigTest.py>`_\  (TestModels/), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TestModels/), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


