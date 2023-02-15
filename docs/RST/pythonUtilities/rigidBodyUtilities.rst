
.. _sec-module-rigidbodyutilities:

Module: rigidBodyUtilities
==========================

Advanced utility/mathematical functions for reference frames, rigid body kinematics
and dynamics. Useful Euler parameter and Tait-Bryan angle conversion functions
are included. A class for rigid body inertia creating and transformation is available.

- Author:    Johannes Gerstmayr, Stefan Holzinger (rotation vector and Tait-Bryan angles) 
- Date:      2020-03-10 (created) 


.. _sec-rigidbodyutilities-computeorthonormalbasisvectors:

`ComputeOrthonormalBasisVectors <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L30>`__\ (\ ``vector0``\ )

- | \ *function description*\ :
  | compute orthogonal basis vectors (normal1, normal2) for given vector0 (non-unique solution!); the length of vector0 must not be 1; if vector0 == [0,0,0], then any normal basis is returned
- | \ *output*\ :
  | returns [vector0normalized, normal1, normal2], in which vector0normalized is the normalized vector0 (has unit length); all vectors in numpy array format


----

.. _sec-rigidbodyutilities-computeorthonormalbasis:

`ComputeOrthonormalBasis <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L55>`__\ (\ ``vector0``\ )

- | \ *function description*\ :
  | compute orthogonal basis, in which the normalized vector0 is the first column and the other columns are normals to vector0 (non-unique solution!); the length of vector0 must not be 1; if vector0 == [0,0,0], then any normal basis is returned
- | \ *output*\ :
  | returns A, a rotation matrix, in which the first column is parallel to vector0; A is a 2D numpy array


----

.. _sec-rigidbodyutilities-gramschmidt:

`GramSchmidt <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L61>`__\ (\ ``vector0``\ , \ ``vector1``\ )

- | \ *function description*\ :
  | compute Gram-Schmidt projection of given 3D vector 1 on vector 0 and return normalized triad (vector0, vector1, vector0 x vector1)

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM), \ `sliderCrank3Dbenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dbenchmark.py>`_\  (TM), \ `sliderCrank3Dtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dtest.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-skew:

`Skew <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L81>`__\ (\ ``vector``\ )

- | \ *function description*\ :
  | compute skew symmetric 3x3-matrix from 3x1- or 1x3-vector

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `leggedRobot.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/leggedRobot.py>`_\  (Ex), \ `stiffFlyballGovernor2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stiffFlyballGovernor2.py>`_\  (Ex), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM), \ `explicitLieGroupIntegratorPythonTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupIntegratorPythonTest.py>`_\  (TM), \ `explicitLieGroupIntegratorTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupIntegratorTest.py>`_\  (TM), \ `heavyTop.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/heavyTop.py>`_\  (TM), \ `LieGroupIntegrationUnitTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/LieGroupIntegrationUnitTests.py>`_\  (TM), \ `mecanumWheelRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mecanumWheelRollingDiscTest.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-skew2vec:

`Skew2Vec <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L89>`__\ (\ ``skew``\ )

- | \ *function description*\ :
  | convert skew symmetric matrix m to vector


----

.. _sec-rigidbodyutilities-computeskewmatrix:

`ComputeSkewMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L111>`__\ (\ ``v``\ )

- | \ *function description*\ :
  | compute skew matrix from vector or matrix; used for ObjectFFRF and CMS implementation
- | \ *input*\ :
  | a vector v in np.array format, containing 3\*n components or a matrix with m columns of same shape
- | \ *output*\ :
  | if v is a vector, output is (3*n x 3) skew matrix in np.array format; if v is a (n x m) matrix, the output is a (3*n x m) skew matrix in np.array format

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `objectFFRFTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFTest.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-eulerparameters2g:

`EulerParameters2G <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L176>`__\ (\ ``eulerParameters``\ )

- | \ *function description*\ :
  | convert Euler parameters (ep) to G-matrix (=\ :math:`\partial \tomega  / \partial \pv_t`\ )
- | \ *input*\ :
  | vector of 4 eulerParameters as list or np.array
- | \ *output*\ :
  | 3x4 matrix G as np.array


----

.. _sec-rigidbodyutilities-eulerparameters2glocal:

`EulerParameters2GLocal <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L185>`__\ (\ ``eulerParameters``\ )

- | \ *function description*\ :
  | convert Euler parameters (ep) to local G-matrix (=\ :math:`\partial ^{b}{\tomega} / \partial \pv_t`\ )
- | \ *input*\ :
  | vector of 4 eulerParameters as list or np.array
- | \ *output*\ :
  | 3x4 matrix G as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `objectFFRFTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFTest.py>`_\  (TM), \ `rigidBodyAsUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyAsUserFunctionTest.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-eulerparameters2rotationmatrix:

`EulerParameters2RotationMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L194>`__\ (\ ``eulerParameters``\ )

- | \ *function description*\ :
  | compute rotation matrix from eulerParameters
- | \ *input*\ :
  | vector of 4 eulerParameters as list or np.array
- | \ *output*\ :
  | 3x3 rotation matrix as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `stiffFlyballGovernor2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stiffFlyballGovernor2.py>`_\  (Ex), \ `stiffFlyballGovernor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/stiffFlyballGovernor.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-rotationmatrix2eulerparameters:

`RotationMatrix2EulerParameters <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L204>`__\ (\ ``rotationMatrix``\ )

- | \ *function description*\ :
  | compute Euler parameters from given rotation matrix
- | \ *input*\ :
  | 3x3 rotation matrix as list of lists or as np.array
- | \ *output*\ :
  | vector of 4 eulerParameters as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `mouseInteractionExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mouseInteractionExample.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `stiffFlyballGovernor2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stiffFlyballGovernor2.py>`_\  (Ex), \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TM), \ `perf3DRigidBodies.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/perf3DRigidBodies.py>`_\  (TM), \ `stiffFlyballGovernor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/stiffFlyballGovernor.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-angularvelocity2eulerparameterst:

`AngularVelocity2EulerParameters\_t <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L249>`__\ (\ ``angularVelocity``\ , \ ``eulerParameters``\ )

- | \ *function description*\ :

  | compute time derivative of Euler parameters from (global) angular velocity vector
  | note that for Euler parameters \ :math:`\pv`\ , we have \ :math:`\tomega=\Gm \dot \pv`\  ==> \ :math:`\Gm^T \tomega = \Gm^T\cdot \Gm\cdot \dot \pv`\  ==> \ :math:`\Gm^T \Gm=4(\Im_{4x4} - \pv\cdot \pv^T)\dot\pv = 4 (\Im_{4x4}) \dot \pv`\ 
- | \ *input*\ :

  | \ ``angularVelocity``\ : 3D vector of angular velocity in global frame, as lists or as np.array
  | \ ``eulerParameters``\ : vector of 4 eulerParameters as np.array or list
- | \ *output*\ :
  | vector of time derivatives of 4 eulerParameters as np.array


----

.. _sec-rigidbodyutilities-rotationvector2rotationmatrix:

`RotationVector2RotationMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L261>`__\ (\ ``rotationVector``\ )

- | \ *function description*\ :
  | rotaton matrix from rotation vector, see appendix B in
- | \ *input*\ :
  | 3D rotation vector as list or np.array
- | \ *output*\ :
  | 3x3 rotation matrix as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `stiffFlyballGovernor2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stiffFlyballGovernor2.py>`_\  (Ex), \ `explicitLieGroupMBSTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupMBSTest.py>`_\  (TM), \ `stiffFlyballGovernor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/stiffFlyballGovernor.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-rotationmatrix2rotationvector:

`RotationMatrix2RotationVector <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L278>`__\ (\ ``rotationMatrix``\ )

- | \ *function description*\ :
  | compute rotation vector from rotation matrix
- | \ *input*\ :
  | 3x3 rotation matrix as list of lists or as np.array
- | \ *output*\ :
  | vector of 3 components of rotation vector as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `explicitLieGroupMBSTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupMBSTest.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-computerotationaxisfromrotationvector:

`ComputeRotationAxisFromRotationVector <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L297>`__\ (\ ``rotationVector``\ )

- | \ *function description*\ :
  | compute rotation axis from given rotation vector
- | \ *input*\ :
  | 3D rotation vector as np.array
- | \ *output*\ :
  | 3D vector as np.array representing the rotation axis

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `LieGroupIntegrationUnitTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/LieGroupIntegrationUnitTests.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-rotationvector2g:

`RotationVector2G <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L315>`__\ (\ ``rotationVector``\ )

- | \ *function description*\ :
  | convert rotation vector (parameters) (v) to G-matrix (=\ :math:`\partial \tomega  / \partial \dot \vv`\ )
- | \ *input*\ :
  | vector of rotation vector (len=3) as list or np.array
- | \ *output*\ :
  | 3x3 matrix G as np.array


----

.. _sec-rigidbodyutilities-rotationvector2glocal:

`RotationVector2GLocal <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L321>`__\ (\ ``eulerParameters``\ )

- | \ *function description*\ :
  | convert rotation vector (parameters) (v) to local G-matrix (=\ :math:`\partial ^{b}{\tomega}   / \partial \vv_t`\ )
- | \ *input*\ :
  | vector of rotation vector (len=3) as list or np.array
- | \ *output*\ :
  | 3x3 matrix G as np.array


----

.. _sec-rigidbodyutilities-rotxyz2rotationmatrix:

`RotXYZ2RotationMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L334>`__\ (\ ``rot``\ )

- | \ *function description*\ :
  | compute rotation matrix from consecutive xyz pRot (Tait-Bryan angles); A=Ax\*Ay\*Az; rot=[rotX, rotY, rotZ]
- | \ *input*\ :
  | 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
- | \ *output*\ :
  | 3x3 rotation matrix as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `serialRobotTestDH2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotTestDH2.py>`_\  (Ex), \ `stiffFlyballGovernor2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stiffFlyballGovernor2.py>`_\  (Ex), \ `explicitLieGroupMBSTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupMBSTest.py>`_\  (TM), \ `kinematicTreeTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeTest.py>`_\  (TM), \ `stiffFlyballGovernor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/stiffFlyballGovernor.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-rotationmatrix2rotxyz:

`RotationMatrix2RotXYZ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L349>`__\ (\ ``rotationMatrix``\ )

- | \ *function description*\ :
  | convert rotation matrix to xyz Euler angles (Tait-Bryan angles);  A=Ax\*Ay\*Az;
- | \ *input*\ :
  | 3x3 rotation matrix as list of lists or np.array
- | \ *output*\ :
  | vector of Tait-Bryan rotation parameters [X,Y,Z] (in radiant) as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `serialRobotTestDH2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotTestDH2.py>`_\  (Ex)


----

.. _sec-rigidbodyutilities-rotxyz2g:

`RotXYZ2G <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L362>`__\ (\ ``rot``\ )

- | \ *function description*\ :
  | compute (global-frame) G-matrix for xyz Euler angles (Tait-Bryan angles) (\ :math:`^{0}{\Gm} = \partial ^{0}{\tomega}  / \partial \dot \ttheta`\ )
- | \ *input*\ :
  | 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
- | \ *output*\ :
  | 3x3 matrix G as np.array


----

.. _sec-rigidbodyutilities-rotxyz2gt:

`RotXYZ2G\_t <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L377>`__\ (\ ``rot``\ , \ ``rot\_t``\ )

- | \ *function description*\ :
  | compute time derivative of (global-frame) G-matrix for xyz Euler angles (Tait-Bryan angles) (\ :math:`^{0}{\Gm} = \partial ^{0}{\tomega}  / \partial \dot \ttheta`\ )
- | \ *input*\ :

  | \ ``rot``\ : 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
  | \ ``rot\_t``\ : 3D vector of time derivative of Tait-Bryan rotation parameters [X,Y,Z] in radiant/s
- | \ *output*\ :
  | 3x3 matrix G_t as np.array


----

.. _sec-rigidbodyutilities-rotxyz2glocal:

`RotXYZ2GLocal <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L391>`__\ (\ ``rot``\ )

- | \ *function description*\ :
  | compute local (body-fixed) G-matrix for xyz Euler angles (Tait-Bryan angles) (\ :math:`^{b}{\Gm} = \partial ^{b}{\tomega}  / \partial \ttheta_t`\ )
- | \ *input*\ :
  | 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
- | \ *output*\ :
  | 3x3 matrix GLocal as np.array


----

.. _sec-rigidbodyutilities-rotxyz2glocalt:

`RotXYZ2GLocal\_t <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L406>`__\ (\ ``rot``\ , \ ``rot\_t``\ )

- | \ *function description*\ :
  | compute time derivative of (body-fixed) G-matrix for xyz Euler angles (Tait-Bryan angles) (\ :math:`^{b}{\Gm} = \partial ^{b}{\tomega}  / \partial \ttheta_t`\ )
- | \ *input*\ :

  | \ ``rot``\ : 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
  | \ ``rot\_t``\ : 3D vector of time derivative of Tait-Bryan rotation parameters [X,Y,Z] in radiant/s
- | \ *output*\ :
  | 3x3 matrix GLocal_t as np.array


----

.. _sec-rigidbodyutilities-angularvelocity2rotxyzt:

`AngularVelocity2RotXYZ\_t <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L426>`__\ (\ ``angularVelocity``\ , \ ``rotation``\ )

- | \ *function description*\ :
  | compute time derivatives of angles RotXYZ from (global) angular velocity vector and given rotation
- | \ *input*\ :

  | \ ``angularVelocity``\ : global angular velocity vector as list or np.array
  | \ ``rotation``\ : 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
- | \ *output*\ :
  | time derivative of vector of Tait-Bryan rotation parameters [X,Y,Z] (in radiant) as np.array


----

.. _sec-rigidbodyutilities-rotxyz2eulerparameters:

`RotXYZ2EulerParameters <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L445>`__\ (\ ``alpha``\ )

- | \ *function description*\ :
  | compute four Euler parameters from given RotXYZ angles, see
- | \ *input*\ :
  | alpha: 3D vector as np.array containing RotXYZ angles
- | \ *output*\ :

  | 4D vector as np.array containing four Euler parameters
  | entry zero of output represent the scalar part of Euler parameters


----

.. _sec-rigidbodyutilities-rotationmatrixx:

`RotationMatrixX <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L514>`__\ (\ ``angleRad``\ )

- | \ *function description*\ :
  | compute rotation matrix w.r.t. X-axis (first axis)
- | \ *input*\ :
  | angle around X-axis in radiant
- | \ *output*\ :
  | 3x3 rotation matrix as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `NGsolveCraigBampton.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCraigBampton.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `particleClusters.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particleClusters.py>`_\  (Ex), \ `generalContactFrictionTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactFrictionTests.py>`_\  (TM), \ `mecanumWheelRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mecanumWheelRollingDiscTest.py>`_\  (TM), \ `perf3DRigidBodies.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/perf3DRigidBodies.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-rotationmatrixy:

`RotationMatrixY <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L522>`__\ (\ ``angleRad``\ )

- | \ *function description*\ :
  | compute rotation matrix w.r.t. Y-axis (second axis)
- | \ *input*\ :
  | angle around Y-axis in radiant
- | \ *output*\ :
  | 3x3 rotation matrix as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `leggedRobot.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/leggedRobot.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `ConvexContactTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ConvexContactTest.py>`_\  (TM), \ `revoluteJointPrismaticJointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/revoluteJointPrismaticJointTest.py>`_\  (TM), \ `rollingCoinPenaltyTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rollingCoinPenaltyTest.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-rotationmatrixz:

`RotationMatrixZ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L530>`__\ (\ ``angleRad``\ )

- | \ *function description*\ :
  | compute rotation matrix w.r.t. Z-axis (third axis)
- | \ *input*\ :
  | angle around Z-axis in radiant
- | \ *output*\ :
  | 3x3 rotation matrix as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Ex), \ `mouseInteractionExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mouseInteractionExample.py>`_\  (Ex), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM), \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TM), \ `generalContactFrictionTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactFrictionTests.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-homogeneoustransformation:

`HomogeneousTransformation <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L539>`__\ (\ ``A``\ , \ ``r``\ )

- | \ *function description*\ :
  | compute HT matrix from rotation matrix A and translation vector r

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `NGsolveCraigBampton.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCraigBampton.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex)


----

.. _sec-rigidbodyutilities-httranslate:

`HTtranslate <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L549>`__\ (\ ``r``\ )

- | \ *function description*\ :
  | HT for translation with vector r

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `kinematicTreePendulum.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreePendulum.py>`_\  (Ex), \ `serialRobotFlexible.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotFlexible.py>`_\  (Ex), \ `serialRobotKinematicTree.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTree.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM), \ `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_\  (TM), \ `movingGroundRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/movingGroundRobotTest.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-httranslatex:

`HTtranslateX <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L555>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | HT for translation along x axis with value x

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-httranslatey:

`HTtranslateY <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L561>`__\ (\ ``y``\ )

- | \ *function description*\ :
  | HT for translation along y axis with value y

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `kinematicTreePendulum.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreePendulum.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM), \ `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-httranslatez:

`HTtranslateZ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L567>`__\ (\ ``z``\ )

- | \ *function description*\ :
  | HT for translation along z axis with value z


----

.. _sec-rigidbodyutilities-ht0:

`HT0 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L573>`__\ ()

- | \ *function description*\ :
  | identity HT:

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `kinematicTreePendulum.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreePendulum.py>`_\  (Ex), \ `serialRobotTestDH2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotTestDH2.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM), \ `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-htrotatex:

`HTrotateX <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L577>`__\ (\ ``angle``\ )

- | \ *function description*\ :
  | HT for rotation around axis X (first axis)


----

.. _sec-rigidbodyutilities-htrotatey:

`HTrotateY <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L583>`__\ (\ ``angle``\ )

- | \ *function description*\ :
  | HT for rotation around axis X (first axis)

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-htrotatez:

`HTrotateZ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L589>`__\ (\ ``angle``\ )

- | \ *function description*\ :
  | HT for rotation around axis X (first axis)

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-ht2translation:

`HT2translation <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L595>`__\ (\ ``T``\ )

- | \ *function description*\ :
  | return translation part of HT

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `serialRobotFlexible.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotFlexible.py>`_\  (Ex), \ `serialRobotKinematicTree.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTree.py>`_\  (Ex), \ `serialRobotKinematicTreeDigging.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTreeDigging.py>`_\  (Ex), \ `serialRobotTestDH2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotTestDH2.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM), \ `movingGroundRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/movingGroundRobotTest.py>`_\  (TM), \ `serialRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/serialRobotTest.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-ht2rotationmatrix:

`HT2rotationMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L599>`__\ (\ ``T``\ )

- | \ *function description*\ :
  | return rotation matrix of HT

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-inverseht:

`InverseHT <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L604>`__\ (\ ``T``\ )

- | \ *function description*\ :
  | return inverse HT such that inv(T)\*T = np.eye(4)

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `serialRobotKinematicTree.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTree.py>`_\  (Ex), \ `serialRobotTestDH2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotTestDH2.py>`_\  (Ex)


----

.. _sec-rigidbodyutilities-rotationx2t66:

`RotationX2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L625>`__\ (\ ``angle``\ )

- | \ *function description*\ :
  | compute 6x6 coordinate transformation matrix for rotation around X axis; output: first 3 components for rotation, second 3 components for translation! See Featherstone / Handbook of robotics


----

.. _sec-rigidbodyutilities-rotationy2t66:

`RotationY2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L637>`__\ (\ ``angle``\ )

- | \ *function description*\ :
  | compute 6x6 transformation matrix for rotation around Y axis; output: first 3 components for rotation, second 3 components for translation


----

.. _sec-rigidbodyutilities-rotationz2t66:

`RotationZ2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L649>`__\ (\ ``angle``\ )

- | \ *function description*\ :
  | compute 6x6 transformation matrix for rotation around Z axis; output: first 3 components for rotation, second 3 components for translation


----

.. _sec-rigidbodyutilities-translation2t66:

`Translation2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L661>`__\ (\ ``translation3D``\ )

- | \ *function description*\ :
  | compute 6x6 transformation matrix for translation according to 3D vector translation3D; output: first 3 components for rotation, second 3 components for translation!


----

.. _sec-rigidbodyutilities-translationx2t66:

`TranslationX2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L672>`__\ (\ ``translation``\ )

- | \ *function description*\ :
  | compute 6x6 transformation matrix for translation along X axis; output: first 3 components for rotation, second 3 components for translation!


----

.. _sec-rigidbodyutilities-translationy2t66:

`TranslationY2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L676>`__\ (\ ``translation``\ )

- | \ *function description*\ :
  | compute 6x6 transformation matrix for translation along Y axis; output: first 3 components for rotation, second 3 components for translation!


----

.. _sec-rigidbodyutilities-translationz2t66:

`TranslationZ2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L680>`__\ (\ ``translation``\ )

- | \ *function description*\ :
  | compute 6x6 transformation matrix for translation along Z axis; output: first 3 components for rotation, second 3 components for translation!


----

.. _sec-rigidbodyutilities-t66torotationtranslation:

`T66toRotationTranslation <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L686>`__\ (\ ``T66``\ )

- | \ *function description*\ :
  | convert 6x6 coordinate transformation (Plücker transform) into rotation and translation
- | \ *input*\ :
  | T66 given as  6x6 numpy array
- | \ *output*\ :
  | [A, v] with 3x3 rotation matrix A and 3D translation vector v

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex)


----

.. _sec-rigidbodyutilities-inverset66torotationtranslation:

`InverseT66toRotationTranslation <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L694>`__\ (\ ``T66``\ )

- | \ *function description*\ :
  | convert inverse 6x6 coordinate transformation (Plücker transform) into rotation and translation
- | \ *input*\ :
  | inverse T66 given as  6x6 numpy array
- | \ *output*\ :
  | [A, v] with 3x3 rotation matrix A and 3D translation vector v


----

.. _sec-rigidbodyutilities-rotationtranslation2t66:

`RotationTranslation2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L704>`__\ (\ ``A``\ , \ ``v``\ )

- | \ *function description*\ :
  | convert rotation and translation into 6x6 coordinate transformation (Plücker transform)
- | \ *input*\ :

  | \ ``A``\ : 3x3 rotation matrix A
  | \ ``v``\ : 3D translation vector v
- | \ *output*\ :
  | return 6x6 transformation matrix 'T66'


----

.. _sec-rigidbodyutilities-rotationtranslation2t66inverse:

`RotationTranslation2T66Inverse <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L714>`__\ (\ ``A``\ , \ ``v``\ )

- | \ *function description*\ :
  | convert rotation and translation into INVERSE 6x6 coordinate transformation (Plücker transform)
- | \ *input*\ :

  | \ ``A``\ : 3x3 rotation matrix A
  | \ ``v``\ : 3D translation vector v
- | \ *output*\ :
  | return 6x6 transformation matrix 'T66'

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex)


----

.. _sec-rigidbodyutilities-t66toht:

`T66toHT <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L742>`__\ (\ ``T66``\ )

- | \ *function description*\ :
  | convert 6x6 coordinate transformation (Plücker transform) into 4x4 homogeneous transformation; NOTE that the homogeneous transformation is the inverse of what is computed in function pluho() of Featherstone
- | \ *input*\ :
  | T66 given as 6x6 numpy array
- | \ *output*\ :
  | homogeneous transformation (4x4 numpy array)

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex)


----

.. _sec-rigidbodyutilities-ht2t66inverse:

`HT2T66Inverse <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L753>`__\ (\ ``T``\ )

- | \ *function description*\ :
  | convert 4x4 homogeneous transformation into 6x6 coordinate transformation (Plücker transform); NOTE that the homogeneous transformation is the inverse of what is computed in function pluho() of Featherstone
- | \ *output*\ :
  | input: T66 (6x6 numpy array)


----

.. _sec-rigidbodyutilities-inertiatensor2inertia6d:

`InertiaTensor2Inertia6D <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L764>`__\ (\ ``inertiaTensor``\ )

- | \ *function description*\ :
  | convert a 3x3 matrix (list or numpy array) into a list with 6 inertia components, sorted as J00, J11, J22, J12, J02, J01

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `serialRobotTestDH2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotTestDH2.py>`_\  (Ex)


----

.. _sec-rigidbodyutilities-inertia6d2inertiatensor:

`Inertia6D2InertiaTensor <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L769>`__\ (\ ``inertia6D``\ )

- | \ *function description*\ :
  | convert a list or numpy array with 6 inertia components (sorted as [J00, J11, J22, J12, J02, J01]) (list or numpy array) into a 3x3 matrix (np.array)

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `serialRobotTestDH2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotTestDH2.py>`_\  (Ex), \ `rigidBodyAsUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyAsUserFunctionTest.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-getrigidbodynode:

`GetRigidBodyNode <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L995>`__\ (\ ``nodeType``\ , \ ``position = [0,0,0]``\ , \ ``velocity = [0,0,0]``\ , \ ``rotationMatrix = []``\ , \ ``rotationParameters = []``\ , \ ``angularVelocity = [0,0,0]``\ )

- | \ *function description*\ :
  | get node item interface according to nodeType, using initialization with position, velocity, angularVelocity and rotationMatrix
- | \ *input*\ :

  | \ ``nodeType``\ : a node type according to exudyn.NodeType, or a string of it, e.g., 'NodeType.RotationEulerParameters' (fastest, but additional algebraic constraint equation), 'NodeType.RotationRxyz' (Tait-Bryan angles, singularity for second angle at +/- 90 degrees), 'NodeType.RotationRotationVector' (used for Lie group integration)
  | \ ``position``\ : reference position as list or numpy array with 3 components (in global/world frame)
  | \ ``velocity``\ : initial translational velocity as list or numpy array with 3 components (in global/world frame)
  | \ ``rotationMatrix``\ : 3x3 list or numpy matrix to define reference rotation; use EITHER rotationMatrix=[[...],[...],[...]] (while rotationParameters=[]) or rotationParameters=[...] (while rotationMatrix=[])
  | \ ``rotationParameters``\ : reference rotation parameters; use EITHER rotationMatrix=[[...],[...],[...]] (while rotationParameters=[]) or rotationParameters=[...] (while rotationMatrix=[])
  | \ ``angularVelocity``\ : initial angular velocity as list or numpy array with 3 components (in global/world frame)
- | \ *output*\ :
  | returns list containing node number and body number: [nodeNumber, bodyNumber]


----

.. _sec-rigidbodyutilities-addrigidbody:

`AddRigidBody <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L1086>`__\ (\ ``mainSys``\ , \ ``inertia``\ , \ ``nodeType = exu.NodeType.RotationEulerParameters``\ , \ ``position = [0,0,0]``\ , \ ``velocity = [0,0,0]``\ , \ ``rotationMatrix = []``\ , \ ``rotationParameters = []``\ , \ ``angularVelocity = [0,0,0]``\ , \ ``gravity = [0,0,0]``\ , \ ``graphicsDataList = []``\ )

- | \ *function description*\ :
  | adds a node (with str(exu.NodeType. ...)) and body for a given rigid body; all quantities (esp. velocity and angular velocity) are given in global coordinates!
- | \ *input*\ :

  | \ ``inertia``\ : an inertia object as created by class RigidBodyInertia; containing mass, COM and inertia
  | \ ``nodeType``\ : a node type according to exudyn.NodeType, or a string of it, e.g., 'NodeType.RotationEulerParameters' (fastest, but additional algebraic constraint equation), 'NodeType.RotationRxyz' (Tait-Bryan angles, singularity for second angle at +/- 90 degrees), 'NodeType.RotationRotationVector' (used for Lie group integration)
  | \ ``position``\ : reference position as list or numpy array with 3 components (in global/world frame)
  | \ ``velocity``\ : initial translational velocity as list or numpy array with 3 components (in global/world frame)
  | \ ``rotationMatrix``\ : 3x3 list or numpy matrix to define reference rotation; use EITHER rotationMatrix=[[...],[...],[...]] (while rotationParameters=[]) or rotationParameters=[...] (while rotationMatrix=[])
  | \ ``rotationParameters``\ : reference rotation parameters; use EITHER rotationMatrix=[[...],[...],[...]] (while rotationParameters=[]) or rotationParameters=[...] (while rotationMatrix=[])
  | \ ``angularVelocity``\ : initial angular velocity as list or numpy array with 3 components (in global/world frame)
  | \ ``gravity``\ : if provided as list or numpy array with 3 components, it adds gravity force to the body at the COM, i.e., fAdd = m\*gravity
  | \ ``graphicsDataList``\ : list of graphicsData objects to define appearance of body
- | \ *output*\ :
  | returns list containing node number and body number: [nodeNumber, bodyNumber]

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `craneReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/craneReevingSystem.py>`_\  (Ex), \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Ex), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM), \ `ConvexContactTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ConvexContactTest.py>`_\  (TM), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-addrevolutejoint:

`AddRevoluteJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L1156>`__\ (\ ``mbs``\ , \ ``body0``\ , \ ``body1``\ , \ ``point``\ , \ ``axis``\ , \ ``useGlobalFrame = True``\ , \ ``showJoint = True``\ , \ ``axisRadius = 0.1``\ , \ ``axisLength = 0.4``\ )

- | \ *function description*\ :
  | add revolute joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed
- | \ *input*\ :

  | \ ``mbs``\ : the MainSystem to which the joint and markers shall be added
  | \ ``body0``\ : a object number for body0, must be rigid body or ground object
  | \ ``body1``\ : a object number for body1, must be rigid body or ground object
  | \ ``point``\ : a 3D vector as list or np.array containing the global center point of the joint in reference configuration
  | \ ``axis``\ : a 3D vector as list or np.array containing the global rotation axis of the joint in reference configuration
  | \ ``useGlobalFrame``\ : if False, the point and axis vectors are defined in the local coordinate system of body0
- | \ *output*\ :
  | returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Ex), \ `openVRengine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openVRengine.py>`_\  (Ex), \ `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_\  (Ex), \ `solutionViewerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/solutionViewerTest.py>`_\  (Ex), \ `perf3DRigidBodies.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/perf3DRigidBodies.py>`_\  (TM)


----

.. _sec-rigidbodyutilities-addprismaticjoint:

`AddPrismaticJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L1237>`__\ (\ ``mbs``\ , \ ``body0``\ , \ ``body1``\ , \ ``point``\ , \ ``axis``\ , \ ``useGlobalFrame = True``\ , \ ``showJoint = True``\ , \ ``axisRadius = 0.1``\ , \ ``axisLength = 0.4``\ )

- | \ *function description*\ :
  | add prismatic joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed
- | \ *input*\ :

  | \ ``mbs``\ : the MainSystem to which the joint and markers shall be added
  | \ ``body0``\ : a object number for body0, must be rigid body or ground object
  | \ ``body1``\ : a object number for body1, must be rigid body or ground object
  | \ ``point``\ : a 3D vector as list or np.array containing the global center point of the joint in reference configuration
  | \ ``axis``\ : a 3D vector as list or np.array containing the global translation axis of the joint in reference configuration
  | \ ``useGlobalFrame``\ : if False, the point and axis vectors are defined in the local coordinate system of body0
- | \ *output*\ :
  | returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `openVRengine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openVRengine.py>`_\  (Ex)


CLASS RigidBodyInertia (in module rigidBodyUtilities)
-----------------------------------------------------
**class description**: 

    helper class for rigid body inertia (see also derived classes Inertia...).
    Provides a structure to define mass, inertia and center of mass (COM) of a rigid body.
    The inertia tensor and center of mass must correspond when initializing the body!

- | \ *notes*\ :
  | in the default mode, inertiaTensorAtCOM = False, the inertia tensor must be provided with respect to the reference point; otherwise, it is given at COM; internally, the inertia tensor is always with respect to the reference point, not w.r.t. to COM!
- | \ *example*\ :

.. code-block:: python

  i0 = RigidBodyInertia(10,np.diag([1,2,3]))
  i1 = i0.Rotated(RotationMatrixX(np.pi/2))
  i2 = i1.Translated([1,0,0])



.. _sec-rigidbodyutilities-rigidbodyinertia-init:

`\_\_init\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L791>`__\ (\ ``self``\ , \ ``mass = 0``\ , \ ``inertiaTensor = np.zeros([3,3])``\ , \ ``com = np.zeros(3)``\ , \ ``inertiaTensorAtCOM = False``\ )

- | \ *classFunction*\ :
  | initialize RigidBodyInertia with scalar mass, 3x3 inertiaTensor (w.r.t. reference point!!!) and center of mass com
- | \ *input*\ :

  | \ ``mass``\ : mass of rigid body (dimensions need to be consistent, should be in SI-units)
  | \ ``inertiaTensor``\ : tensor given w.r.t.\ reference point, NOT w.r.t.\ center of mass!
  | \ ``com``\ : center of mass relative to reference point, in same coordinate system as inertiaTensor

----

.. _sec-rigidbodyutilities-rigidbodyinertia-add:

`\_\_add\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L807>`__\ (\ ``self``\ , \ ``otherBodyInertia``\ )

- | \ *classFunction*\ :

  | add (+) operator allows adding another inertia information with SAME local coordinate system and reference point!
  | only inertias with same center of rotation can be added!
- | \ *example*\ :

.. code-block:: python

  J = InertiaSphere(2,0.1) + InertiaRodX(1,2)


----

.. _sec-rigidbodyutilities-rigidbodyinertia-iadd:

`\_\_iadd\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L818>`__\ (\ ``self``\ , \ ``otherBodyInertia``\ )

- | \ *classFunction*\ :

  | += operator allows adding another inertia information with SAME local coordinate system and reference point!
  | only inertias with same center of rotation can be added!
- | \ *example*\ :

.. code-block:: python

  J = InertiaSphere(2,0.1)
  J += InertiaRodX(1,2)


----

.. _sec-rigidbodyutilities-rigidbodyinertia-setwithcominertia:

`SetWithCOMinertia <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L827>`__\ (\ ``self``\ , \ ``mass``\ , \ ``inertiaTensorCOM``\ , \ ``com``\ )

- | \ *classFunction*\ :
  | set RigidBodyInertia with scalar mass, 3x3 inertiaTensor (w.r.t.\ com) and center of mass com
- | \ *input*\ :

  | \ ``mass``\ : mass of rigid body (dimensions need to be consistent, should be in SI-units)
  | \ ``inertiaTensorCOM``\ : tensor given w.r.t.\ reference point, NOT w.r.t.\ center of mass!
  | \ ``com``\ : center of mass relative to reference point, in same coordinate system as inertiaTensor

----

.. _sec-rigidbodyutilities-rigidbodyinertia-inertia:

`Inertia <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L838>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | returns 3x3 inertia tensor with respect to chosen reference point (not necessarily COM)

----

.. _sec-rigidbodyutilities-rigidbodyinertia-inertiacom:

`InertiaCOM <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L842>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | returns 3x3 inertia tensor with respect to COM

----

.. _sec-rigidbodyutilities-rigidbodyinertia-com:

`COM <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L846>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | returns center of mass (COM) w.r.t. chosen reference point

----

.. _sec-rigidbodyutilities-rigidbodyinertia-mass:

`Mass <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L850>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | returns mass

----

.. _sec-rigidbodyutilities-rigidbodyinertia-translated:

`Translated <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L854>`__\ (\ ``self``\ , \ ``vec``\ )

- | \ *classFunction*\ :
  | returns a RigidBodyInertia with center of mass com shifted by vec; \ :math:`\rightarrow`\  transforms the returned inertiaTensor to the new center of rotation

----

.. _sec-rigidbodyutilities-rigidbodyinertia-rotated:

`Rotated <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L868>`__\ (\ ``self``\ , \ ``rot``\ )

- | \ *classFunction*\ :
  | returns a RigidBodyInertia rotated by 3x3 rotation matrix rot, such that for a given J, the new inertia tensor reads Jnew = rot\*J\*rot.T
- | \ *notes*\ :
  | only allowed if COM=0 !

----

.. _sec-rigidbodyutilities-rigidbodyinertia-transformed:

`Transformed <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L881>`__\ (\ ``self``\ , \ ``HT``\ )

- | \ *classFunction*\ :
  | return rigid body inertia transformed by homogeneous transformation HT

----

.. _sec-rigidbodyutilities-rigidbodyinertia-getinertia6d:

`GetInertia6D <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L896>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | get vector with 6 inertia components (Jxx, Jyy, Jzz, Jyz, Jxz, Jxy) as needed in ObjectRigidBody

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `serialRobotKinematicTree.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTree.py>`_\  (Ex), \ `serialRobotTestDH2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotTestDH2.py>`_\  (Ex), \ `sliderCrank3DwithANCFbeltDrive2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive2.py>`_\  (Ex), \ `rigidBodyCOMtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyCOMtest.py>`_\  (TM), \ `rollingCoinPenaltyTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rollingCoinPenaltyTest.py>`_\  (TM), \ `rollingCoinTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rollingCoinTest.py>`_\  (TM)


CLASS InertiaCuboid(RigidBodyInertia) (in module rigidBodyUtilities)
--------------------------------------------------------------------
**class description**: 

    create RigidBodyInertia with moment of inertia and mass of a cuboid with density and side lengths sideLengths along local axes 1, 2, 3; inertia w.r.t. center of mass, com=[0,0,0]

- | \ *example*\ :

.. code-block:: python

  InertiaCuboid(density=1000,sideLengths=[1,0.1,0.1])



.. _sec-rigidbodyutilities-inertiacuboid(rigidbodyinertia)-init:

`\_\_init\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L917>`__\ (\ ``self``\ , \ ``density``\ , \ ``sideLengths``\ )

- | \ *classFunction*\ :
  | initialize inertia

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `craneReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/craneReevingSystem.py>`_\  (Ex), \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM), \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TM), \ `generalContactFrictionTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactFrictionTests.py>`_\  (TM)


CLASS InertiaRodX(RigidBodyInertia) (in module rigidBodyUtilities)
------------------------------------------------------------------
**class description**: 

    create RigidBodyInertia with moment of inertia and mass of a rod with mass m and length L in local 1-direction (x-direction); inertia w.r.t. center of mass, com=[0,0,0]


.. _sec-rigidbodyutilities-inertiarodx(rigidbodyinertia)-init:

`\_\_init\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L929>`__\ (\ ``self``\ , \ ``mass``\ , \ ``length``\ )

- | \ *classFunction*\ :
  | initialize inertia with mass and length of rod

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `fourBarMechanismRedundant.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/fourBarMechanismRedundant.py>`_\  (TM)


CLASS InertiaMassPoint(RigidBodyInertia) (in module rigidBodyUtilities)
-----------------------------------------------------------------------
**class description**: 

    create RigidBodyInertia with moment of inertia and mass of mass point with 'mass'; inertia w.r.t. center of mass, com=[0,0,0]


.. _sec-rigidbodyutilities-inertiamasspoint(rigidbodyinertia)-init:

`\_\_init\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L937>`__\ (\ ``self``\ , \ ``mass``\ )

- | \ *classFunction*\ :
  | initialize inertia with mass of point

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `stiffFlyballGovernor2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stiffFlyballGovernor2.py>`_\  (Ex), \ `stiffFlyballGovernorKT.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stiffFlyballGovernorKT.py>`_\  (Ex), \ `stiffFlyballGovernor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/stiffFlyballGovernor.py>`_\  (TM)


CLASS InertiaSphere(RigidBodyInertia) (in module rigidBodyUtilities)
--------------------------------------------------------------------
**class description**: 

    create RigidBodyInertia with moment of inertia and mass of sphere with mass and radius; inertia w.r.t. center of mass, com=[0,0,0]


.. _sec-rigidbodyutilities-inertiasphere(rigidbodyinertia)-init:

`\_\_init\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L945>`__\ (\ ``self``\ , \ ``mass``\ , \ ``radius``\ )

- | \ *classFunction*\ :
  | initialize inertia with mass and radius of sphere

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `particleClusters.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particleClusters.py>`_\  (Ex), \ `particlesSilo.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesSilo.py>`_\  (Ex), \ `tippeTop.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/tippeTop.py>`_\  (Ex), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM), \ `generalContactFrictionTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactFrictionTests.py>`_\  (TM)


CLASS InertiaHollowSphere(RigidBodyInertia) (in module rigidBodyUtilities)
--------------------------------------------------------------------------
**class description**: 

    create RigidBodyInertia with moment of inertia and mass of hollow sphere with mass (concentrated at circumference) and radius; inertia w.r.t. center of mass, com=0


.. _sec-rigidbodyutilities-inertiahollowsphere(rigidbodyinertia)-init:

`\_\_init\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L954>`__\ (\ ``self``\ , \ ``mass``\ , \ ``radius``\ )

- | \ *classFunction*\ :
  | initialize inertia with mass and (inner==outer) radius of hollow sphere


CLASS InertiaCylinder(RigidBodyInertia) (in module rigidBodyUtilities)
----------------------------------------------------------------------
**class description**: 

    create RigidBodyInertia with moment of inertia and mass of cylinder with density, length and outerRadius; axis defines the orientation of the cylinder axis (0=x-axis, 1=y-axis, 2=z-axis); for hollow cylinder use innerRadius != 0; inertia w.r.t. center of mass, com=[0,0,0]


.. _sec-rigidbodyutilities-inertiacylinder(rigidbodyinertia)-init:

`\_\_init\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L963>`__\ (\ ``self``\ , \ ``density``\ , \ ``length``\ , \ ``outerRadius``\ , \ ``axis``\ , \ ``innerRadius = 0``\ )

- | \ *classFunction*\ :
  | initialize inertia with density, length, outer radius, axis (0=x-axis, 1=y-axis, 2=z-axis) and optional inner radius (for hollow cylinder)

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `gyroStability.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/gyroStability.py>`_\  (Ex), \ `leggedRobot.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/leggedRobot.py>`_\  (Ex), \ `openVRengine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openVRengine.py>`_\  (Ex), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TM), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TM), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM), \ `ConvexContactTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ConvexContactTest.py>`_\  (TM), \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TM)

