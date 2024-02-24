
.. _sec-module-rigidbodyutilities:

Module: rigidBodyUtilities
==========================

Advanced utility/mathematical functions for reference frames, rigid body kinematics
and dynamics. Useful Euler parameter and Tait-Bryan angle conversion functions
are included. A class for rigid body inertia creating and transformation is available.

- Author:    Johannes Gerstmayr, Stefan Holzinger (rotation vector and Tait-Bryan angles) 
- Date:      2020-03-10 (created) 


.. _sec-rigidbodyutilities-computeorthonormalbasisvectors:

Function: ComputeOrthonormalBasisVectors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeOrthonormalBasisVectors <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L30>`__\ (\ ``vector0``\ )

- | \ *function description*\ :
  | compute orthogonal basis vectors (normal1, normal2) for given vector0 (non-unique solution!); the length of vector0 must not be 1; if vector0 == [0,0,0], then any normal basis is returned
- | \ *output*\ :
  | returns [vector0normalized, normal1, normal2], in which vector0normalized is the normalized vector0 (has unit length); all vectors in numpy array format



----


.. _sec-rigidbodyutilities-computeorthonormalbasis:

Function: ComputeOrthonormalBasis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeOrthonormalBasis <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L55>`__\ (\ ``vector0``\ )

- | \ *function description*\ :
  | compute orthogonal basis, in which the normalized vector0 is the first column and the other columns are normals to vector0 (non-unique solution!); the length of vector0 must not be 1; if vector0 == [0,0,0], then any normal basis is returned
- | \ *output*\ :
  | returns A, a rotation matrix, in which the first column is parallel to vector0; A is a 2D numpy array



----


.. _sec-rigidbodyutilities-gramschmidt:

Function: GramSchmidt
^^^^^^^^^^^^^^^^^^^^^
`GramSchmidt <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L61>`__\ (\ ``vector0``\ , \ ``vector1``\ )

- | \ *function description*\ :
  | compute Gram-Schmidt projection of given 3D vector 1 on vector 0 and return normalized triad (vector0, vector1, vector0 x vector1)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM), \ `sliderCrank3Dbenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dbenchmark.py>`_\  (TM), \ `sliderCrank3Dtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dtest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-skew:

Function: Skew
^^^^^^^^^^^^^^
`Skew <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L81>`__\ (\ ``vector``\ )

- | \ *function description*\ :
  | compute skew symmetric 3x3-matrix from 3x1- or 1x3-vector

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `leggedRobot.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/leggedRobot.py>`_\  (Ex), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM), \ `explicitLieGroupIntegratorPythonTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupIntegratorPythonTest.py>`_\  (TM), \ `explicitLieGroupIntegratorTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupIntegratorTest.py>`_\  (TM), \ `heavyTop.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/heavyTop.py>`_\  (TM), \ `laserScannerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/laserScannerTest.py>`_\  (TM), \ `LieGroupIntegrationUnitTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/LieGroupIntegrationUnitTests.py>`_\  (TM), \ `mecanumWheelRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mecanumWheelRollingDiscTest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-skew2vec:

Function: Skew2Vec
^^^^^^^^^^^^^^^^^^
`Skew2Vec <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L89>`__\ (\ ``skew``\ )

- | \ *function description*\ :
  | convert skew symmetric matrix m to vector

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `serialRobotInverseKinematics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInverseKinematics.py>`_\  (Ex)



----


.. _sec-rigidbodyutilities-computeskewmatrix:

Function: ComputeSkewMatrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeSkewMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L111>`__\ (\ ``v``\ )

- | \ *function description*\ :
  | compute skew matrix from vector or matrix; used for ObjectFFRF and CMS implementation
- | \ *input*\ :
  | a vector v in np.array format, containing 3\*n components or a matrix with m columns of same shape
- | \ *output*\ :
  | if v is a vector, output is (3*n x 3) skew matrix in np.array format; if v is a (n x m) matrix, the output is a (3*n x m) skew matrix in np.array format

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `objectFFRFTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFTest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-eulerparameters2g:

Function: EulerParameters2G
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`EulerParameters2G <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L176>`__\ (\ ``eulerParameters``\ )

- | \ *function description*\ :
  | convert Euler parameters (ep) to G-matrix (=\ :math:`\partial \tomega  / \partial {\mathbf{p}}_t`\ )
- | \ *input*\ :
  | vector of 4 eulerParameters as list or np.array
- | \ *output*\ :
  | 3x4 matrix G as np.array



----


.. _sec-rigidbodyutilities-eulerparameters2glocal:

Function: EulerParameters2GLocal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`EulerParameters2GLocal <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L185>`__\ (\ ``eulerParameters``\ )

- | \ *function description*\ :
  | convert Euler parameters (ep) to local G-matrix (=\ :math:`\partial \LU{b}{\tomega} / \partial {\mathbf{p}}_t`\ )
- | \ *input*\ :
  | vector of 4 eulerParameters as list or np.array
- | \ *output*\ :
  | 3x4 matrix G as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `objectFFRFTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFTest.py>`_\  (TM), \ `rigidBodyAsUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyAsUserFunctionTest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-eulerparameters2rotationmatrix:

Function: EulerParameters2RotationMatrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`EulerParameters2RotationMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L194>`__\ (\ ``eulerParameters``\ )

- | \ *function description*\ :
  | compute rotation matrix from eulerParameters
- | \ *input*\ :
  | vector of 4 eulerParameters as list or np.array
- | \ *output*\ :
  | 3x3 rotation matrix as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ROSMobileManipulator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSMobileManipulator.py>`_\  (Ex), \ `stiffFlyballGovernor2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stiffFlyballGovernor2.py>`_\  (Ex), \ `stiffFlyballGovernor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/stiffFlyballGovernor.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-rotationmatrix2eulerparameters:

Function: RotationMatrix2EulerParameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`RotationMatrix2EulerParameters <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L204>`__\ (\ ``rotationMatrix``\ )

- | \ *function description*\ :
  | compute Euler parameters from given rotation matrix
- | \ *input*\ :
  | 3x3 rotation matrix as list of lists or as np.array
- | \ *output*\ :
  | vector of 4 eulerParameters as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `mouseInteractionExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mouseInteractionExample.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `stiffFlyballGovernor2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stiffFlyballGovernor2.py>`_\  (Ex), \ `perf3DRigidBodies.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/perf3DRigidBodies.py>`_\  (TM), \ `rightAngleFrame.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rightAngleFrame.py>`_\  (TM), \ `stiffFlyballGovernor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/stiffFlyballGovernor.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-angularvelocity2eulerparameters-t:

Function: AngularVelocity2EulerParameters\_t
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`AngularVelocity2EulerParameters\_t <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L249>`__\ (\ ``angularVelocity``\ , \ ``eulerParameters``\ )

- | \ *function description*\ :
  | compute time derivative of Euler parameters from (global) angular velocity vector
  | note that for Euler parameters \ :math:`{\mathbf{p}}`\ , we have \ :math:`\tomega={\mathbf{G}} \dot {\mathbf{p}}`\  ==> \ :math:`{\mathbf{G}}^T \tomega = {\mathbf{G}}^T\cdot {\mathbf{G}}\cdot \dot {\mathbf{p}}`\  ==> \ :math:`{\mathbf{G}}^T {\mathbf{G}}=4({\mathbf{I}}_{4 \times 4} - {\mathbf{p}}\cdot {\mathbf{p}}^T)\dot{\mathbf{p}} = 4 ({\mathbf{I}}_{4x4}) \dot {\mathbf{p}}`\ 
- | \ *input*\ :
  | \ ``angularVelocity``\ : 3D vector of angular velocity in global frame, as lists or as np.array
  | \ ``eulerParameters``\ : vector of 4 eulerParameters as np.array or list
- | \ *output*\ :
  | vector of time derivatives of 4 eulerParameters as np.array



----


.. _sec-rigidbodyutilities-rotationvector2rotationmatrix:

Function: RotationVector2RotationMatrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`RotationVector2RotationMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L262>`__\ (\ ``rotationVector``\ )

- | \ *function description*\ :
  | rotaton matrix from rotation vector, see appendix B in
- | \ *input*\ :
  | 3D rotation vector as list or np.array
- | \ *output*\ :
  | 3x3 rotation matrix as np.array
- | \ *notes*\ :
  | gets inaccurate for very large rotations, \ :math:`\phi \\gg 2*\pi`\

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Ex), \ `stiffFlyballGovernor2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stiffFlyballGovernor2.py>`_\  (Ex), \ `universalJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/universalJoint.py>`_\  (Ex), \ `explicitLieGroupMBSTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupMBSTest.py>`_\  (TM), \ `stiffFlyballGovernor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/stiffFlyballGovernor.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-rotationmatrix2rotationvector:

Function: RotationMatrix2RotationVector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`RotationMatrix2RotationVector <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L279>`__\ (\ ``rotationMatrix``\ )

- | \ *function description*\ :
  | compute rotation vector from rotation matrix
- | \ *input*\ :
  | 3x3 rotation matrix as list of lists or as np.array
- | \ *output*\ :
  | vector of 3 components of rotation vector as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `explicitLieGroupMBSTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupMBSTest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-computerotationaxisfromrotationvector:

Function: ComputeRotationAxisFromRotationVector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeRotationAxisFromRotationVector <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L311>`__\ (\ ``rotationVector``\ )

- | \ *function description*\ :
  | compute rotation axis from given rotation vector
- | \ *input*\ :
  | 3D rotation vector as np.array
- | \ *output*\ :
  | 3D vector as np.array representing the rotation axis

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `LieGroupIntegrationUnitTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/LieGroupIntegrationUnitTests.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-rotationvector2g:

Function: RotationVector2G
^^^^^^^^^^^^^^^^^^^^^^^^^^
`RotationVector2G <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L329>`__\ (\ ``rotationVector``\ )

- | \ *function description*\ :
  | convert rotation vector (parameters) (v) to G-matrix (=\ :math:`\partial \tomega  / \partial \dot {\mathbf{v}}`\ )
- | \ *input*\ :
  | vector of rotation vector (len=3) as list or np.array
- | \ *output*\ :
  | 3x3 matrix G as np.array



----


.. _sec-rigidbodyutilities-rotationvector2glocal:

Function: RotationVector2GLocal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`RotationVector2GLocal <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L335>`__\ (\ ``eulerParameters``\ )

- | \ *function description*\ :
  | convert rotation vector (parameters) (v) to local G-matrix (=\ :math:`\partial \LU{b}{\tomega}   / \partial {\mathbf{v}}_t`\ )
- | \ *input*\ :
  | vector of rotation vector (len=3) as list or np.array
- | \ *output*\ :
  | 3x3 matrix G as np.array



----


.. _sec-rigidbodyutilities-rotxyz2rotationmatrix:

Function: RotXYZ2RotationMatrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`RotXYZ2RotationMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L348>`__\ (\ ``rot``\ )

- | \ *function description*\ :
  | compute rotation matrix from consecutive xyz \ :ref:`Rot <Rot>`\  (Tait-Bryan angles); A=Ax\*Ay\*Az; rot=[rotX, rotY, rotZ]
- | \ *input*\ :
  | 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
- | \ *output*\ :
  | 3x3 rotation matrix as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `stiffFlyballGovernor2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stiffFlyballGovernor2.py>`_\  (Ex), \ `explicitLieGroupMBSTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupMBSTest.py>`_\  (TM), \ `kinematicTreeTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeTest.py>`_\  (TM), \ `stiffFlyballGovernor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/stiffFlyballGovernor.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-rotationmatrix2rotxyz:

Function: RotationMatrix2RotXYZ
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`RotationMatrix2RotXYZ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L366>`__\ (\ ``rotationMatrix``\ )

- | \ *function description*\ :
  | convert rotation matrix to xyz Euler angles (Tait-Bryan angles);  A=Ax\*Ay\*Az;
- | \ *input*\ :
  | 3x3 rotation matrix as list of lists or np.array
- | \ *output*\ :
  | vector of Tait-Bryan rotation parameters [X,Y,Z] (in radiant) as np.array
- | \ *notes*\ :
  | due to gimbal lock / singularity at rot[1] = pi/2, -pi/2, ... the reconstruction of
  | \ ``RotationMatrix2RotXYZ( RotXYZ2RotationMatrix(rot) )``\  may fail, but
  | \ ``RotXYZ2RotationMatrix( RotationMatrix2RotXYZ( RotXYZ2RotationMatrix(rot) ) )``\  works always

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `serialRobotInteractiveLimits.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInteractiveLimits.py>`_\  (Ex)



----


.. _sec-rigidbodyutilities-rotxyz2g:

Function: RotXYZ2G
^^^^^^^^^^^^^^^^^^
`RotXYZ2G <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L397>`__\ (\ ``rot``\ )

- | \ *function description*\ :
  | compute (global-frame) G-matrix for xyz Euler angles (Tait-Bryan angles) (\ :math:`\LU{0}{{\mathbf{G}}} = \partial \LU{0}{\tomega}  / \partial \dot \ttheta`\ )
- | \ *input*\ :
  | 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
- | \ *output*\ :
  | 3x3 matrix G as np.array



----


.. _sec-rigidbodyutilities-rotxyz2g-t:

Function: RotXYZ2G\_t
^^^^^^^^^^^^^^^^^^^^^
`RotXYZ2G\_t <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L412>`__\ (\ ``rot``\ , \ ``rot_t``\ )

- | \ *function description*\ :
  | compute time derivative of (global-frame) G-matrix for xyz Euler angles (Tait-Bryan angles) (\ :math:`\LU{0}{{\mathbf{G}}} = \partial \LU{0}{\tomega}  / \partial \dot \ttheta`\ )
- | \ *input*\ :
  | \ ``rot``\ : 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
  | \ ``rot_t``\ : 3D vector of time derivative of Tait-Bryan rotation parameters [X,Y,Z] in radiant/s
- | \ *output*\ :
  | 3x3 matrix G_t as np.array



----


.. _sec-rigidbodyutilities-rotxyz2glocal:

Function: RotXYZ2GLocal
^^^^^^^^^^^^^^^^^^^^^^^
`RotXYZ2GLocal <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L426>`__\ (\ ``rot``\ )

- | \ *function description*\ :
  | compute local (body-fixed) G-matrix for xyz Euler angles (Tait-Bryan angles) (\ :math:`\LU{b}{{\mathbf{G}}} = \partial \LU{b}{\tomega}  / \partial \ttheta_t`\ )
- | \ *input*\ :
  | 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
- | \ *output*\ :
  | 3x3 matrix GLocal as np.array



----


.. _sec-rigidbodyutilities-rotxyz2glocal-t:

Function: RotXYZ2GLocal\_t
^^^^^^^^^^^^^^^^^^^^^^^^^^
`RotXYZ2GLocal\_t <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L441>`__\ (\ ``rot``\ , \ ``rot_t``\ )

- | \ *function description*\ :
  | compute time derivative of (body-fixed) G-matrix for xyz Euler angles (Tait-Bryan angles) (\ :math:`\LU{b}{{\mathbf{G}}} = \partial \LU{b}{\tomega}  / \partial \ttheta_t`\ )
- | \ *input*\ :
  | \ ``rot``\ : 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
  | \ ``rot_t``\ : 3D vector of time derivative of Tait-Bryan rotation parameters [X,Y,Z] in radiant/s
- | \ *output*\ :
  | 3x3 matrix GLocal_t as np.array



----


.. _sec-rigidbodyutilities-angularvelocity2rotxyz-t:

Function: AngularVelocity2RotXYZ\_t
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`AngularVelocity2RotXYZ\_t <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L461>`__\ (\ ``angularVelocity``\ , \ ``rotation``\ )

- | \ *function description*\ :
  | compute time derivatives of angles RotXYZ from (global) angular velocity vector and given rotation
- | \ *input*\ :
  | \ ``angularVelocity``\ : global angular velocity vector as list or np.array
  | \ ``rotation``\ : 3D vector of Tait-Bryan rotation parameters [X,Y,Z] in radiant
- | \ *output*\ :
  | time derivative of vector of Tait-Bryan rotation parameters [X,Y,Z] (in radiant) as np.array



----


.. _sec-rigidbodyutilities-rotxyz2eulerparameters:

Function: RotXYZ2EulerParameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`RotXYZ2EulerParameters <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L480>`__\ (\ ``alpha``\ )

- | \ *function description*\ :
  | compute four Euler parameters from given RotXYZ angles, see
- | \ *input*\ :
  | alpha: 3D vector as np.array containing RotXYZ angles
- | \ *output*\ :
  | 4D vector as np.array containing four Euler parameters
  | entry zero of output represent the scalar part of Euler parameters



----


.. _sec-rigidbodyutilities-rotationmatrix2rotzyz:

Function: RotationMatrix2RotZYZ
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`RotationMatrix2RotZYZ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L511>`__\ (\ ``rotationMatrix``\ , \ ``flip``\ )

- | \ *function description*\ :
  | convert rotation matrix to zyz Euler angles;  A=Az\*Ay\*Az;
- | \ *input*\ :
  | \ ``rotationMatrix``\ : 3x3 rotation matrix as list of lists or np.array
  | \ ``flip``\ :           argument to choose first Euler angle to be in quadrant 2 or 3.
- | \ *output*\ :
  | vector of Euler rotation parameters [Z,Y,Z] (in radiant) as np.array
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | tested (compared with Robotics, Vision and Control book of P. Corke)



----


.. _sec-rigidbodyutilities-rotationmatrixx:

Function: RotationMatrixX
^^^^^^^^^^^^^^^^^^^^^^^^^
`RotationMatrixX <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L550>`__\ (\ ``angleRad``\ )

- | \ *function description*\ :
  | compute rotation matrix w.r.t. X-axis (first axis)
- | \ *input*\ :
  | angle around X-axis in radiant
- | \ *output*\ :
  | 3x3 rotation matrix as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/graphicsDataExample.py>`_\  (Ex), \ `NGsolveCraigBampton.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCraigBampton.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `generalContactFrictionTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactFrictionTests.py>`_\  (TM), \ `laserScannerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/laserScannerTest.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-rotationmatrixy:

Function: RotationMatrixY
^^^^^^^^^^^^^^^^^^^^^^^^^
`RotationMatrixY <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L558>`__\ (\ ``angleRad``\ )

- | \ *function description*\ :
  | compute rotation matrix w.r.t. Y-axis (second axis)
- | \ *input*\ :
  | angle around Y-axis in radiant
- | \ *output*\ :
  | 3x3 rotation matrix as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `leggedRobot.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/leggedRobot.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `computeODE2AEeigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2AEeigenvaluesTest.py>`_\  (TM), \ `ConvexContactTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ConvexContactTest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-rotationmatrixz:

Function: RotationMatrixZ
^^^^^^^^^^^^^^^^^^^^^^^^^
`RotationMatrixZ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L566>`__\ (\ ``angleRad``\ )

- | \ *function description*\ :
  | compute rotation matrix w.r.t. Z-axis (third axis)
- | \ *input*\ :
  | angle around Z-axis in radiant
- | \ *output*\ :
  | 3x3 rotation matrix as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Ex), \ `mouseInteractionExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mouseInteractionExample.py>`_\  (Ex), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM), \ `computeODE2AEeigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2AEeigenvaluesTest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-homogeneoustransformation:

Function: HomogeneousTransformation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`HomogeneousTransformation <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L575>`__\ (\ ``A``\ , \ ``r``\ )

- | \ *function description*\ :
  | compute \ :ref:`HT <HT>`\  matrix from rotation matrix A and translation vector r

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `NGsolveCraigBampton.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCraigBampton.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex)



----


.. _sec-rigidbodyutilities-httranslate:

Function: HTtranslate
^^^^^^^^^^^^^^^^^^^^^
`HTtranslate <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L585>`__\ (\ ``r``\ )

- | \ *function description*\ :
  | \ :ref:`HT <HT>`\  for translation with vector r

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `kinematicTreePendulum.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreePendulum.py>`_\  (Ex), \ `openAIgymNLinkContinuous.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymNLinkContinuous.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM), \ `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_\  (TM), \ `movingGroundRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/movingGroundRobotTest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-httranslatex:

Function: HTtranslateX
^^^^^^^^^^^^^^^^^^^^^^
`HTtranslateX <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L591>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | \ :ref:`HT <HT>`\  for translation along x axis with value x

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-httranslatey:

Function: HTtranslateY
^^^^^^^^^^^^^^^^^^^^^^
`HTtranslateY <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L597>`__\ (\ ``y``\ )

- | \ *function description*\ :
  | \ :ref:`HT <HT>`\  for translation along y axis with value y

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `kinematicTreePendulum.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreePendulum.py>`_\  (Ex), \ `openAIgymNLinkContinuous.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymNLinkContinuous.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM), \ `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-httranslatez:

Function: HTtranslateZ
^^^^^^^^^^^^^^^^^^^^^^
`HTtranslateZ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L603>`__\ (\ ``z``\ )

- | \ *function description*\ :
  | \ :ref:`HT <HT>`\  for translation along z axis with value z



----


.. _sec-rigidbodyutilities-ht0:

Function: HT0
^^^^^^^^^^^^^
`HT0 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L609>`__\ ()

- | \ *function description*\ :
  | identity \ :ref:`HT <HT>`\ :

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `kinematicTreePendulum.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreePendulum.py>`_\  (Ex), \ `openAIgymNLinkContinuous.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymNLinkContinuous.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM), \ `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-htrotatex:

Function: HTrotateX
^^^^^^^^^^^^^^^^^^^
`HTrotateX <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L613>`__\ (\ ``angle``\ )

- | \ *function description*\ :
  | \ :ref:`HT <HT>`\  for rotation around axis X (first axis)



----


.. _sec-rigidbodyutilities-htrotatey:

Function: HTrotateY
^^^^^^^^^^^^^^^^^^^
`HTrotateY <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L619>`__\ (\ ``angle``\ )

- | \ *function description*\ :
  | \ :ref:`HT <HT>`\  for rotation around axis X (first axis)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-htrotatez:

Function: HTrotateZ
^^^^^^^^^^^^^^^^^^^
`HTrotateZ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L625>`__\ (\ ``angle``\ )

- | \ *function description*\ :
  | \ :ref:`HT <HT>`\  for rotation around axis X (first axis)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ROSMobileManipulator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSMobileManipulator.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-ht2translation:

Function: HT2translation
^^^^^^^^^^^^^^^^^^^^^^^^
`HT2translation <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L631>`__\ (\ ``T``\ )

- | \ *function description*\ :
  | return translation part of \ :ref:`HT <HT>`\

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `serialRobotFlexible.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotFlexible.py>`_\  (Ex), \ `serialRobotInteractiveLimits.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInteractiveLimits.py>`_\  (Ex), \ `serialRobotInverseKinematics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInverseKinematics.py>`_\  (Ex), \ `serialRobotKinematicTree.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTree.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM), \ `movingGroundRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/movingGroundRobotTest.py>`_\  (TM), \ `serialRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/serialRobotTest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-ht2rotationmatrix:

Function: HT2rotationMatrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`HT2rotationMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L635>`__\ (\ ``T``\ )

- | \ *function description*\ :
  | return rotation matrix of \ :ref:`HT <HT>`\

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-inverseht:

Function: InverseHT
^^^^^^^^^^^^^^^^^^^
`InverseHT <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L640>`__\ (\ ``T``\ )

- | \ *function description*\ :
  | return inverse \ :ref:`HT <HT>`\  such that inv(T)\*T = np.eye(4)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `serialRobotKinematicTree.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTree.py>`_\  (Ex)



----


.. _sec-rigidbodyutilities-rotationx2t66:

Function: RotationX2T66
^^^^^^^^^^^^^^^^^^^^^^^
`RotationX2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L661>`__\ (\ ``angle``\ )

- | \ *function description*\ :
  | compute 6x6 coordinate transformation matrix for rotation around X axis; output: first 3 components for rotation, second 3 components for translation! See Featherstone / Handbook of robotics



----


.. _sec-rigidbodyutilities-rotationy2t66:

Function: RotationY2T66
^^^^^^^^^^^^^^^^^^^^^^^
`RotationY2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L673>`__\ (\ ``angle``\ )

- | \ *function description*\ :
  | compute 6x6 transformation matrix for rotation around Y axis; output: first 3 components for rotation, second 3 components for translation



----


.. _sec-rigidbodyutilities-rotationz2t66:

Function: RotationZ2T66
^^^^^^^^^^^^^^^^^^^^^^^
`RotationZ2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L685>`__\ (\ ``angle``\ )

- | \ *function description*\ :
  | compute 6x6 transformation matrix for rotation around Z axis; output: first 3 components for rotation, second 3 components for translation



----


.. _sec-rigidbodyutilities-translation2t66:

Function: Translation2T66
^^^^^^^^^^^^^^^^^^^^^^^^^
`Translation2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L697>`__\ (\ ``translation3D``\ )

- | \ *function description*\ :
  | compute 6x6 transformation matrix for translation according to 3D vector translation3D; output: first 3 components for rotation, second 3 components for translation!



----


.. _sec-rigidbodyutilities-translationx2t66:

Function: TranslationX2T66
^^^^^^^^^^^^^^^^^^^^^^^^^^
`TranslationX2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L708>`__\ (\ ``translation``\ )

- | \ *function description*\ :
  | compute 6x6 transformation matrix for translation along X axis; output: first 3 components for rotation, second 3 components for translation!



----


.. _sec-rigidbodyutilities-translationy2t66:

Function: TranslationY2T66
^^^^^^^^^^^^^^^^^^^^^^^^^^
`TranslationY2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L712>`__\ (\ ``translation``\ )

- | \ *function description*\ :
  | compute 6x6 transformation matrix for translation along Y axis; output: first 3 components for rotation, second 3 components for translation!



----


.. _sec-rigidbodyutilities-translationz2t66:

Function: TranslationZ2T66
^^^^^^^^^^^^^^^^^^^^^^^^^^
`TranslationZ2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L716>`__\ (\ ``translation``\ )

- | \ *function description*\ :
  | compute 6x6 transformation matrix for translation along Z axis; output: first 3 components for rotation, second 3 components for translation!



----


.. _sec-rigidbodyutilities-t66torotationtranslation:

Function: T66toRotationTranslation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`T66toRotationTranslation <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L722>`__\ (\ ``T66``\ )

- | \ *function description*\ :
  | convert 6x6 coordinate transformation (Plücker transform) into rotation and translation
- | \ *input*\ :
  | T66 given as  6x6 numpy array
- | \ *output*\ :
  | [A, v] with 3x3 rotation matrix A and 3D translation vector v

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex)



----


.. _sec-rigidbodyutilities-inverset66torotationtranslation:

Function: InverseT66toRotationTranslation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`InverseT66toRotationTranslation <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L730>`__\ (\ ``T66``\ )

- | \ *function description*\ :
  | convert inverse 6x6 coordinate transformation (Plücker transform) into rotation and translation
- | \ *input*\ :
  | inverse T66 given as  6x6 numpy array
- | \ *output*\ :
  | [A, v] with 3x3 rotation matrix A and 3D translation vector v



----


.. _sec-rigidbodyutilities-rotationtranslation2t66:

Function: RotationTranslation2T66
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`RotationTranslation2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L740>`__\ (\ ``A``\ , \ ``v``\ )

- | \ *function description*\ :
  | convert rotation and translation into 6x6 coordinate transformation (Plücker transform)
- | \ *input*\ :
  | \ ``A``\ : 3x3 rotation matrix A
  | \ ``v``\ : 3D translation vector v
- | \ *output*\ :
  | return 6x6 transformation matrix 'T66'



----


.. _sec-rigidbodyutilities-rotationtranslation2t66inverse:

Function: RotationTranslation2T66Inverse
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`RotationTranslation2T66Inverse <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L750>`__\ (\ ``A``\ , \ ``v``\ )

- | \ *function description*\ :
  | convert rotation and translation into INVERSE 6x6 coordinate transformation (Plücker transform)
- | \ *input*\ :
  | \ ``A``\ : 3x3 rotation matrix A
  | \ ``v``\ : 3D translation vector v
- | \ *output*\ :
  | return 6x6 transformation matrix 'T66'

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex)



----


.. _sec-rigidbodyutilities-t66toht:

Function: T66toHT
^^^^^^^^^^^^^^^^^
`T66toHT <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L778>`__\ (\ ``T66``\ )

- | \ *function description*\ :
  | convert 6x6 coordinate transformation (Plücker transform) into 4x4 homogeneous transformation; NOTE that the homogeneous transformation is the inverse of what is computed in function pluho() of Featherstone
- | \ *input*\ :
  | T66 given as 6x6 numpy array
- | \ *output*\ :
  | homogeneous transformation (4x4 numpy array)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex)



----


.. _sec-rigidbodyutilities-ht2t66inverse:

Function: HT2T66Inverse
^^^^^^^^^^^^^^^^^^^^^^^
`HT2T66Inverse <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L789>`__\ (\ ``T``\ )

- | \ *function description*\ :
  | convert 4x4 homogeneous transformation into 6x6 coordinate transformation (Plücker transform); NOTE that the homogeneous transformation is the inverse of what is computed in function pluho() of Featherstone
- | \ *output*\ :
  | input: T66 (6x6 numpy array)



----


.. _sec-rigidbodyutilities-inertiatensor2inertia6d:

Function: InertiaTensor2Inertia6D
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`InertiaTensor2Inertia6D <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L800>`__\ (\ ``inertiaTensor``\ )

- | \ *function description*\ :
  | convert a 3x3 matrix (list or numpy array) into a list with 6 inertia components, sorted as J00, J11, J22, J12, J02, J01



----


.. _sec-rigidbodyutilities-inertia6d2inertiatensor:

Function: Inertia6D2InertiaTensor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Inertia6D2InertiaTensor <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L805>`__\ (\ ``inertia6D``\ )

- | \ *function description*\ :
  | convert a list or numpy array with 6 inertia components (sorted as [J00, J11, J22, J12, J02, J01]) (list or numpy array) into a 3x3 matrix (np.array)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `rigidBodyAsUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyAsUserFunctionTest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-strnodetype2nodetype:

Function: StrNodeType2NodeType
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`StrNodeType2NodeType <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L1022>`__\ (\ ``sNodeType``\ )

- | \ *function description*\ :
  | convert string into exudyn.NodeType; call e.g. with 'NodeType.RotationEulerParameters' or 'RotationEulerParameters'
- | \ *notes*\ :
  | function is not very fast, so should be avoided in time-critical situations



----


.. _sec-rigidbodyutilities-getrigidbodynode:

Function: GetRigidBodyNode
^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetRigidBodyNode <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L1044>`__\ (\ ``nodeType``\ , \ ``position = [0,0,0]``\ , \ ``velocity = [0,0,0]``\ , \ ``rotationMatrix = []``\ , \ ``rotationParameters = []``\ , \ ``angularVelocity = [0,0,0]``\ )

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

Function: AddRigidBody
^^^^^^^^^^^^^^^^^^^^^^
`AddRigidBody <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L1137>`__\ (\ ``mainSys``\ , \ ``inertia``\ , \ ``nodeType = exu.NodeType.RotationEulerParameters``\ , \ ``position = [0,0,0]``\ , \ ``velocity = [0,0,0]``\ , \ ``rotationMatrix = []``\ , \ ``rotationParameters = []``\ , \ ``angularVelocity = [0,0,0]``\ , \ ``gravity = [0,0,0]``\ , \ ``graphicsDataList = []``\ )

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

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `craneReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/craneReevingSystem.py>`_\  (Ex), \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Ex), \ `gyroStability.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/gyroStability.py>`_\  (Ex), \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `generalContactFrictionTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactFrictionTests.py>`_\  (TM), \ `laserScannerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/laserScannerTest.py>`_\  (TM), \ `mecanumWheelRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mecanumWheelRollingDiscTest.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-addrevolutejoint:

Function: AddRevoluteJoint
^^^^^^^^^^^^^^^^^^^^^^^^^^
`AddRevoluteJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L1209>`__\ (\ ``mbs``\ , \ ``body0``\ , \ ``body1``\ , \ ``point``\ , \ ``axis``\ , \ ``useGlobalFrame = True``\ , \ ``showJoint = True``\ , \ ``axisRadius = 0.1``\ , \ ``axisLength = 0.4``\ )

- | \ *function description*\ :
  | DEPRECATED (use MainSystem function instead): add revolute joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed
- | \ *input*\ :
  | \ ``mbs``\ : the MainSystem to which the joint and markers shall be added
  | \ ``body0``\ : a object number for body0, must be rigid body or ground object
  | \ ``body1``\ : a object number for body1, must be rigid body or ground object
  | \ ``point``\ : a 3D vector as list or np.array containing the global center point of the joint in reference configuration
  | \ ``axis``\ : a 3D vector as list or np.array containing the global rotation axis of the joint in reference configuration
  | \ ``useGlobalFrame``\ : if False, the point and axis vectors are defined in the local coordinate system of body0
- | \ *output*\ :
  | returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `openVRengine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openVRengine.py>`_\  (Ex), \ `stlFileImport.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stlFileImport.py>`_\  (Ex), \ `perf3DRigidBodies.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/perf3DRigidBodies.py>`_\  (TM)



----


.. _sec-rigidbodyutilities-addprismaticjoint:

Function: AddPrismaticJoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`AddPrismaticJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L1290>`__\ (\ ``mbs``\ , \ ``body0``\ , \ ``body1``\ , \ ``point``\ , \ ``axis``\ , \ ``useGlobalFrame = True``\ , \ ``showJoint = True``\ , \ ``axisRadius = 0.1``\ , \ ``axisLength = 0.4``\ )

- | \ *function description*\ :
  | DEPRECATED (use MainSystem function instead): add prismatic joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed
- | \ *input*\ :
  | \ ``mbs``\ : the MainSystem to which the joint and markers shall be added
  | \ ``body0``\ : a object number for body0, must be rigid body or ground object
  | \ ``body1``\ : a object number for body1, must be rigid body or ground object
  | \ ``point``\ : a 3D vector as list or np.array containing the global center point of the joint in reference configuration
  | \ ``axis``\ : a 3D vector as list or np.array containing the global translation axis of the joint in reference configuration
  | \ ``useGlobalFrame``\ : if False, the point and axis vectors are defined in the local coordinate system of body0
- | \ *output*\ :
  | returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `openVRengine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openVRengine.py>`_\  (Ex)


.. _sec-module-rigidbodyutilities-class-rigidbodyinertia:

CLASS RigidBodyInertia (in module rigidBodyUtilities)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
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



.. _sec-rigidbodyutilities-rigidbodyinertia---init--:

Class function: \_\_init\_\_
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`\_\_init\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L827>`__\ (\ ``self``\ , \ ``mass = 0``\ , \ ``inertiaTensor = np.zeros([3,3])``\ , \ ``com = np.zeros(3)``\ , \ ``inertiaTensorAtCOM = False``\ )

- | \ *classFunction*\ :
  | initialize RigidBodyInertia with scalar mass, 3x3 inertiaTensor (w.r.t. reference point!!!) and center of mass com
- | \ *input*\ :
  | \ ``mass``\ : mass of rigid body (dimensions need to be consistent, should be in SI-units)
  | \ ``inertiaTensor``\ : tensor given w.r.t.\ reference point, NOT w.r.t.\ center of mass!
  | \ ``com``\ : center of mass relative to reference point, in same coordinate system as inertiaTensor

----

.. _sec-rigidbodyutilities-rigidbodyinertia---add--:

Class function: \_\_add\_\_
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`\_\_add\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L843>`__\ (\ ``self``\ , \ ``otherBodyInertia``\ )

- | \ *classFunction*\ :
  | add (+) operator allows adding another inertia information with SAME local coordinate system and reference point!
  | only inertias with same center of rotation can be added!
- | \ *example*\ :

.. code-block:: python

  J = InertiaSphere(2,0.1) + InertiaRodX(1,2)


----

.. _sec-rigidbodyutilities-rigidbodyinertia---iadd--:

Class function: \_\_iadd\_\_
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`\_\_iadd\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L854>`__\ (\ ``self``\ , \ ``otherBodyInertia``\ )

- | \ *classFunction*\ :
  | += operator allows adding another inertia information with SAME local coordinate system and reference point!
  | only inertias with same center of rotation can be added!
- | \ *example*\ :

.. code-block:: python

  J = InertiaSphere(2,0.1)
  J += InertiaRodX(1,2)


----

.. _sec-rigidbodyutilities-rigidbodyinertia-setwithcominertia:

Class function: SetWithCOMinertia
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`SetWithCOMinertia <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L863>`__\ (\ ``self``\ , \ ``mass``\ , \ ``inertiaTensorCOM``\ , \ ``com``\ )

- | \ *classFunction*\ :
  | set RigidBodyInertia with scalar mass, 3x3 inertiaTensor (w.r.t.\ com) and center of mass com
- | \ *input*\ :
  | \ ``mass``\ : mass of rigid body (dimensions need to be consistent, should be in SI-units)
  | \ ``inertiaTensorCOM``\ : tensor given w.r.t.\ reference point, NOT w.r.t.\ center of mass!
  | \ ``com``\ : center of mass relative to reference point, in same coordinate system as inertiaTensor

----

.. _sec-rigidbodyutilities-rigidbodyinertia-inertia:

Class function: Inertia
^^^^^^^^^^^^^^^^^^^^^^^
`Inertia <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L874>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | returns 3x3 inertia tensor with respect to chosen reference point (not necessarily COM)

----

.. _sec-rigidbodyutilities-rigidbodyinertia-inertiacom:

Class function: InertiaCOM
^^^^^^^^^^^^^^^^^^^^^^^^^^
`InertiaCOM <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L878>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | returns 3x3 inertia tensor with respect to COM

----

.. _sec-rigidbodyutilities-rigidbodyinertia-com:

Class function: COM
^^^^^^^^^^^^^^^^^^^
`COM <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L882>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | returns center of mass (COM) w.r.t. chosen reference point

----

.. _sec-rigidbodyutilities-rigidbodyinertia-mass:

Class function: Mass
^^^^^^^^^^^^^^^^^^^^
`Mass <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L886>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | returns mass

----

.. _sec-rigidbodyutilities-rigidbodyinertia-translated:

Class function: Translated
^^^^^^^^^^^^^^^^^^^^^^^^^^
`Translated <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L890>`__\ (\ ``self``\ , \ ``vec``\ )

- | \ *classFunction*\ :
  | returns a RigidBodyInertia with center of mass com shifted by vec; \ :math:`\ra`\  transforms the returned inertiaTensor to the new center of rotation

----

.. _sec-rigidbodyutilities-rigidbodyinertia-rotated:

Class function: Rotated
^^^^^^^^^^^^^^^^^^^^^^^
`Rotated <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L904>`__\ (\ ``self``\ , \ ``rot``\ )

- | \ *classFunction*\ :
  | returns a RigidBodyInertia rotated by 3x3 rotation matrix rot, such that for a given J, the new inertia tensor reads Jnew = rot\*J\*rot.T
- | \ *notes*\ :
  | only allowed if COM=0 !

----

.. _sec-rigidbodyutilities-rigidbodyinertia-transformed:

Class function: Transformed
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Transformed <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L917>`__\ (\ ``self``\ , \ ``HT``\ )

- | \ *classFunction*\ :
  | return rigid body inertia transformed by homogeneous transformation HT

----

.. _sec-rigidbodyutilities-rigidbodyinertia-getinertia6d:

Class function: GetInertia6D
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetInertia6D <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L932>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | get vector with 6 inertia components (Jxx, Jyy, Jzz, Jyz, Jxz, Jxy) as needed in ObjectRigidBody

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `openAIgymNLinkContinuous.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymNLinkContinuous.py>`_\  (Ex), \ `serialRobotKinematicTree.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTree.py>`_\  (Ex), \ `sliderCrank3DwithANCFbeltDrive2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive2.py>`_\  (Ex), \ `rigidBodyCOMtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyCOMtest.py>`_\  (TM), \ `rollingCoinPenaltyTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rollingCoinPenaltyTest.py>`_\  (TM), \ `rollingCoinTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rollingCoinTest.py>`_\  (TM)


.. _sec-module-rigidbodyutilities-class-inertiacuboid(rigidbodyinertia):

CLASS InertiaCuboid(RigidBodyInertia) (in module rigidBodyUtilities)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    create RigidBodyInertia with moment of inertia and mass of a cuboid with density and side lengths sideLengths along local axes 1, 2, 3; inertia w.r.t. center of mass, com=[0,0,0]

- | \ *example*\ :

.. code-block:: python

  InertiaCuboid(density=1000,sideLengths=[1,0.1,0.1])



.. _sec-rigidbodyutilities-inertiacuboid(rigidbodyinertia)---init--:

Class function: \_\_init\_\_
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`\_\_init\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L953>`__\ (\ ``self``\ , \ ``density``\ , \ ``sideLengths``\ )

- | \ *classFunction*\ :
  | initialize inertia

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Ex), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM), \ `computeODE2AEeigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2AEeigenvaluesTest.py>`_\  (TM)


.. _sec-module-rigidbodyutilities-class-inertiarodx(rigidbodyinertia):

CLASS InertiaRodX(RigidBodyInertia) (in module rigidBodyUtilities)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    create RigidBodyInertia with moment of inertia and mass of a rod with mass m and length L in local 1-direction (x-direction); inertia w.r.t. center of mass, com=[0,0,0]


.. _sec-rigidbodyutilities-inertiarodx(rigidbodyinertia)---init--:

Class function: \_\_init\_\_
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`\_\_init\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L965>`__\ (\ ``self``\ , \ ``mass``\ , \ ``length``\ )

- | \ *classFunction*\ :
  | initialize inertia with mass and length of rod

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `fourBarMechanismIftomm.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/fourBarMechanismIftomm.py>`_\  (TM)


.. _sec-module-rigidbodyutilities-class-inertiamasspoint(rigidbodyinertia):

CLASS InertiaMassPoint(RigidBodyInertia) (in module rigidBodyUtilities)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    create RigidBodyInertia with moment of inertia and mass of mass point with 'mass'; inertia w.r.t. center of mass, com=[0,0,0]


.. _sec-rigidbodyutilities-inertiamasspoint(rigidbodyinertia)---init--:

Class function: \_\_init\_\_
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`\_\_init\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L973>`__\ (\ ``self``\ , \ ``mass``\ )

- | \ *classFunction*\ :
  | initialize inertia with mass of point

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `stiffFlyballGovernor2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stiffFlyballGovernor2.py>`_\  (Ex), \ `stiffFlyballGovernorKT.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/stiffFlyballGovernorKT.py>`_\  (Ex), \ `stiffFlyballGovernor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/stiffFlyballGovernor.py>`_\  (TM)


.. _sec-module-rigidbodyutilities-class-inertiasphere(rigidbodyinertia):

CLASS InertiaSphere(RigidBodyInertia) (in module rigidBodyUtilities)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    create RigidBodyInertia with moment of inertia and mass of sphere with mass and radius; inertia w.r.t. center of mass, com=[0,0,0]


.. _sec-rigidbodyutilities-inertiasphere(rigidbodyinertia)---init--:

Class function: \_\_init\_\_
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`\_\_init\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L981>`__\ (\ ``self``\ , \ ``mass``\ , \ ``radius``\ )

- | \ *classFunction*\ :
  | initialize inertia with mass and radius of sphere

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/graphicsDataExample.py>`_\  (Ex), \ `particleClusters.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particleClusters.py>`_\  (Ex), \ `particlesSilo.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesSilo.py>`_\  (Ex), \ `ROSMassPoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSMassPoint.py>`_\  (Ex), \ `tippeTop.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/tippeTop.py>`_\  (Ex), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM), \ `generalContactFrictionTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/generalContactFrictionTests.py>`_\  (TM)


.. _sec-module-rigidbodyutilities-class-inertiahollowsphere(rigidbodyinertia):

CLASS InertiaHollowSphere(RigidBodyInertia) (in module rigidBodyUtilities)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    create RigidBodyInertia with moment of inertia and mass of hollow sphere with mass (concentrated at circumference) and radius; inertia w.r.t. center of mass, com=0


.. _sec-rigidbodyutilities-inertiahollowsphere(rigidbodyinertia)---init--:

Class function: \_\_init\_\_
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`\_\_init\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L990>`__\ (\ ``self``\ , \ ``mass``\ , \ ``radius``\ )

- | \ *classFunction*\ :
  | initialize inertia with mass and (inner==outer) radius of hollow sphere


.. _sec-module-rigidbodyutilities-class-inertiacylinder(rigidbodyinertia):

CLASS InertiaCylinder(RigidBodyInertia) (in module rigidBodyUtilities)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    create RigidBodyInertia with moment of inertia and mass of cylinder with density, length and outerRadius; axis defines the orientation of the cylinder axis (0=x-axis, 1=y-axis, 2=z-axis); for hollow cylinder use innerRadius != 0; inertia w.r.t. center of mass, com=[0,0,0]


.. _sec-rigidbodyutilities-inertiacylinder(rigidbodyinertia)---init--:

Class function: \_\_init\_\_
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`\_\_init\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/rigidBodyUtilities.py\#L999>`__\ (\ ``self``\ , \ ``density``\ , \ ``length``\ , \ ``outerRadius``\ , \ ``axis``\ , \ ``innerRadius = 0``\ )

- | \ *classFunction*\ :
  | initialize inertia with density, length, outer radius, axis (0=x-axis, 1=y-axis, 2=z-axis) and optional inner radius (for hollow cylinder)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `gyroStability.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/gyroStability.py>`_\  (Ex), \ `leggedRobot.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/leggedRobot.py>`_\  (Ex), \ `openVRengine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openVRengine.py>`_\  (Ex), \ `pistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pistonEngine.py>`_\  (Ex), \ `ROSMobileManipulator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSMobileManipulator.py>`_\  (Ex), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TM), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TM), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM)

