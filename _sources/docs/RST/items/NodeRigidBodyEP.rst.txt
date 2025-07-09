

.. _sec-item-noderigidbodyep:

NodeRigidBodyEP
===============

A 3D rigid body node based on Euler parameters for rigid bodies or beams; the node has 3 displacement coordinates (representing displacement of reference point \ :math:`\LU{0}{{\mathbf{r}}}`\ ) and four rotation coordinates (Euler parameters = unit quaternions).

\ **Additional information for NodeRigidBodyEP**\ :

* | This \ ``Node``\  has/provides the following types = \ ``Position``\ , \ ``Orientation``\ , \ ``RigidBody``\ , \ ``RotationEulerParameters``\ 
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


----------

.. _description-noderigidbodyep:

DESCRIPTION of NodeRigidBodyEP
------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}\cConfig = \LU{0}{[p_0,\,p_1,\,p_2]}\cConfig\tp= \LU{0}{{\mathbf{u}}}\cConfig + \LU{0}{{\mathbf{p}}}\cRef`\ 
  | global 3D position vector of node; \ :math:`{\mathbf{u}}\cRef=0`\ 
* | ``Displacement``\ : \ :math:`\LU{0}{{\mathbf{u}}}\cConfig = [q_0,\,q_1,\,q_2]\cConfig\tp`\ 
  | global 3D displacement vector of node
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}\cConfig = [\dot q_0,\,\dot q_1,\,\dot q_2]\cConfig\tp`\ 
  | global 3D velocity vector of node
* | ``Acceleration``\ : \ :math:`\LU{0}{{\mathbf{a}}}\cConfig = [\ddot q_0,\,\ddot q_1,\,\ddot q_2]\cConfig\tp`\ 
  | global 3D acceleration vector of node
* | ``CoordinatesTotal``\ : 
  | displacement/rotation coordinates of node including reference configuration
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



\ **Detailed information:** 
All coordinates \ :math:`{\mathbf{c}}\cConfig`\  lead to second order differential equations.
The first 3 equations are residuals of translational forces in global coordinates,
while the last 4 equations are residual of local torques left-multiplied with \ :math:`\LU{b}{{\mathbf{G}}\tp}`\  or
global torques left-multiplied with \ :math:`\LU{0}{{\mathbf{G}}\tp}`\ , see Eq. :eq:`eq-noderigidbodyep-gm`\ , compare the equations of motion of
the rigid body.

There is one additional (algebraic) constraint equation for the quaternions.
The additional constraint equation, which needs to be provided by the object, reads

.. math::

   1 - \sum_{i=0}^{3} \theta_i^2 = 0.


The rotation matrix \ :math:`\LU{0b}{\Rot}\cConfig`\  transforms a local (body-fixed) 3D position 
\ :math:`\pLocB = \LU{b}{[b_0,\,b_1,\,b_2]}\tp`\  to global 3D positions,

.. math::

   \LU{0}{\pLoc}\cConfig = \LU{0b}{\Rot}\cConfig \LU{b}{\pLoc}


Note that the Euler parameters \ :math:`\ttheta\cCur`\  are computed as sum of current coordinates plus reference coordinates,

.. math::

   \ttheta\cCur = \tpsi\cCur + \tpsi\cRef.


The rotation matrix is defined as function of the rotation parameters \ :math:`\ttheta=[\theta_0,\,\theta_1,\,\theta_2,\,\theta_3]\tp`\ 

.. math::

   \LU{0b}{\Rot} = \mr{-2\theta_3^2 - 2\theta_2^2+1}{-2\theta_3\theta_0+2\theta_2\theta_1}{2*\theta_3\theta_1+2*\theta_2\theta_0} {2\theta_3\theta_0+2\theta_2\theta_1}{-2\theta_3^2-2\theta_1^2+1}{2\theta_3\theta_2-2\theta_1\theta_0} {-2\theta_2\theta_0+2\theta_3\theta_1}{2\theta_3\theta_2+2\theta_1\theta_0}{-2\theta_2^2-2\theta_1^2+1}


The derivatives of the angular velocity vectors w.r.t.\ the rotation velocity coordinates \ :math:`\dot \ttheta=[\dot \theta_0,\,\dot \theta_1,\,\dot \theta_2,\,\dot \theta_3]\tp`\  lead to the \ :math:`{\mathbf{G}}`\  matrices, as used in the equations of motion for rigid bodies,

.. math::
   :label: eq-noderigidbodyep-gm

   \LU{0}{\tomega} &=& \LU{0}{{\mathbf{G}}} \dot \ttheta, \\
   \LU{b}{\tomega} &=& \LU{b}{{\mathbf{G}}} \dot \ttheta.


For creating a \ ``NodeRigidBodyEP``\  together with a rigid body, there is a \ ``rigidBodyUtilities``\  function \ ``CreateRigidBody``\ , 
see Section :ref:`sec-mainsystemextensions-createrigidbody`\ , which simplifies the setup of a rigid body significantely!


Relevant Examples and TestModels with weblink:

    \ `rigid3Dexample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigid3Dexample.py>`_\  (Examples/), \ `rigidBodyIMUtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyIMUtest.py>`_\  (Examples/), \ `rigidRotor3DbasicBehaviour.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidRotor3DbasicBehaviour.py>`_\  (Examples/), \ `rigidRotor3DFWBW.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidRotor3DFWBW.py>`_\  (Examples/), \ `rigidRotor3Dnutation.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidRotor3Dnutation.py>`_\  (Examples/), \ `rigidRotor3Drunup.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidRotor3Drunup.py>`_\  (Examples/), \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Examples/), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Examples/), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Examples/), \ `ballBearningModel.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ballBearningModel.py>`_\  (Examples/), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Examples/), \ `bungeeJump.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bungeeJump.py>`_\  (Examples/), \ `explicitLieGroupIntegratorPythonTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupIntegratorPythonTest.py>`_\  (TestModels/), \ `explicitLieGroupIntegratorTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupIntegratorTest.py>`_\  (TestModels/), \ `explicitLieGroupMBSTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupMBSTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


