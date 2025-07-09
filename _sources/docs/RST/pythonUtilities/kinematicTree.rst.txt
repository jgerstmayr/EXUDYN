
.. _sec-module-kinematictree:

Module: kinematicTree
=====================

A library for preparation of minimal coordinates (kinematic tree) formulation.
This library follows mostly the algorithms of Roy Featherstone, see http://royfeatherstone.org/
His code is availble in MATLAB as well as described in the Springer Handbook of Robotics .
The main formalisms are based on 6x6 matrices, so-called Plücker transformations, denoted as \ :ref:`T66 <T66>`\ , as defined by Featherstone.

- Author:    Johannes Gerstmayr 
- Date:      2021-06-22 


.. _sec-kinematictree-masscominertia2t66:

Function: MassCOMinertia2T66
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`MassCOMinertia2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L65>`__\ (\ ``mass``\ , \ ``centerOfMass``\ , \ ``inertia``\ )

- | \ *function description*\ :
  | convert mass, COM and inertia into 6x6 inertia matrix
- | \ *input*\ :
  | \ ``mass``\ : scalar mass
  | \ ``centerOfMass``\ : 3D vector (list/array)
  | \ ``inertia``\ : 3x3 matrix (list of lists / 2D array) w.r.t. center of mass
- | \ *output*\ :
  | 6x6 numpy array for further use in minimal coordinates formulation



----


.. _sec-kinematictree-inertia2t66:

Function: Inertia2T66
^^^^^^^^^^^^^^^^^^^^^
`Inertia2T66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L74>`__\ (\ ``inertia``\ )

- | \ *function description*\ :
  | convert inertia as produced with RigidBodyInertia class into 6x6 inertia matrix (as used in KinematicTree66, Featherstone / Handbook of robotics )
- | \ *output*\ :
  | 6x6 numpy array for further use in minimal coordinates formulation
- | \ *notes*\ :
  | within the 6x6 matrix, the inertia tensor is defined w.r.t.\ the center of mass, while RigidBodyInertia defines the inertia tensor w.r.t.\ the reference point; however, this function correctly transforms all quantities of inertia.

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex)



----


.. _sec-kinematictree-inertia66tomasscominertia:

Function: Inertia66toMassCOMinertia
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Inertia66toMassCOMinertia <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L90>`__\ (\ ``inertia66``\ )

- | \ *function description*\ :
  | convert 6x6 inertia matrix into mass, COM and inertia
- | \ *input*\ :
  | 6x6 numpy array containing rigid body inertia according to Featherstone / Handbook of robotics
- | \ *output*\ :
  | [mass, centerOfMass, inertia]
  | \ ``mass``\ : scalar mass
  | \ ``centerOfMass``\ : 3D vector (list/array)
  | \ ``inertia``\ : 3x3 matrix (list of lists / 2D array) w.r.t. center of mass



----


.. _sec-kinematictree-jointtransformmotionsubspace66:

Function: JointTransformMotionSubspace66
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`JointTransformMotionSubspace66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L111>`__\ (\ ``jointType``\ , \ ``q``\ )

- | \ *function description*\ :
  | return 6x6 Plücker joint transformation matrix evaluated for scalar joint coordinate q and motion subspace ('free modes' in Table 2.6 in Handbook of robotics )

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex)



----


.. _sec-kinematictree-jointtransformmotionsubspace:

Function: JointTransformMotionSubspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`JointTransformMotionSubspace <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L132>`__\ (\ ``jointType``\ , \ ``q``\ )

- | \ *function description*\ :
  | return list containing rotation matrix, translation vector, rotation axis and translation axis for joint transformation

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex)



----


.. _sec-kinematictree-crm:

Function: CRM
^^^^^^^^^^^^^
`CRM <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L401>`__\ (\ ``v``\ )

- | \ *function description*\ :
  | computes cross product operator for motion from 6D vector v; CRM(v) @ m computes the cross product of v and motion m



----


.. _sec-kinematictree-crf:

Function: CRF
^^^^^^^^^^^^^
`CRF <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L410>`__\ (\ ``v``\ )

- | \ *function description*\ :
  | computes cross product operator for force from 6D vector v; CRF(v) @ f computes the cross product of v and force f


.. _sec-module-kinematictree-class-kinematictree33:

CLASS KinematicTree33 (in module kinematicTree)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    class to define a kinematic tree in Python, which can be used for building serial or tree-structured multibody systems
    (or robots) with a minimal coordinates formulation, using rotation matrices and 3D offsets; for efficient computation, use the C++ ObjectKinematicTree

- | \ *notes*\ :
  | The formulation and structures widely follows the more efficient formulas (but still implemented in Python!) with 3D vectors and rotation matrices as proposed in Handbook of robotics , Chapter 3, but with the rotation matrices (\ ``listOfRotations``\ ) being transposed in the Python implementation as compared to the description in the book, being thus compliant with other Exudyn functions; the 3D vector/matrix Python implementation does not offer advantages as compared to the formulation with Plücker coordinates, BUT it reflects the formulas of the C++ implementation and is used for testing


.. _sec-kinematictree-kinematictree33---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L158>`__\ (\ ``self``\ , \ ``listOfJointTypes``\ , \ ``listOfRotations``\ , \ ``listOfOffsets``\ , \ ``listOfInertia3D``\ , \ ``listOfCOM``\ , \ ``listOfMass``\ , \ ``listOfParents = []``\ , \ ``gravity = [0,0,-9.81]``\ )

- | \ *classFunction*\ :
  | initialize kinematic tree
- | \ *input*\ :
  | \ ``listOfJointTypes``\ : mandatory list of joint types 'Rx', 'Ry', 'Rz' denoting revolute joints; 'Px', 'Py', 'Pz', denoting prismatic joints
  | \ ``listOfRotations``\ : per link rotation matrix, transforming coordinates of the joint coordinate system w.r.t. the previous coordinate system (this is the inverse of Plücker coordinate transforms (6x6))
  | \ ``listOfOffsets``\ : per link offset vector from pervious coordinate system to the joint coordinate system
  | \ ``listOfInertia3D``\ : per link 3D inertia matrix, w.r.t.\ reference point (not COM!)
  | \ ``listOfCOM``\ : per link vector from reference point to center of mass (COM), in link coordinates
  | \ ``listOfMass``\ : mass per link
  | \ ``listOfParents``\ : list of parent object indices (int), according to the index in jointTypes and transformations; use empty list for kinematic chain and use -1 if no parent exists (parent=base or world frame)
  | \ ``gravity``\ : a 3D list/array containing the gravity applied to the kinematic tree (in world frame)

----

.. _sec-kinematictree-kinematictree33-size:

Class function: Size
^^^^^^^^^^^^^^^^^^^^
`Size <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L199>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | return number of joints, defined by size of jointTypes

----

.. _sec-kinematictree-kinematictree33-xl:

Class function: XL
^^^^^^^^^^^^^^^^^^
`XL <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L205>`__\ (\ ``self``\ , \ ``i``\ )

- | \ *classFunction*\ :
  | return [A, p] containing rotation matrix and offset for joint j

----

.. _sec-kinematictree-kinematictree33-forwarddynamicscrb:

Class function: ForwardDynamicsCRB
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ForwardDynamicsCRB <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L216>`__\ (\ ``self``\ , \ ``q = []``\ , \ ``q_t = []``\ , \ ``torques = []``\ , \ ``forces = []``\ )

- | \ *classFunction*\ :
  | compute forward dynamics using composite rigid body algorithm
- | \ *input*\ :
  | \ ``q``\ : joint space coordinates for the model at which the forward dynamics is evaluated
  | \ ``q_t``\ : joint space velocity coordinates for the model at which the forward dynamics is evaluated
  | \ ``torques``\ : a vector of torques applied at joint coordinates or list/array with zero length
  | \ ``forces``\ : forces acting on the bodies using special format
- | \ *output*\ :
  | returns acceleration vector q_tt of joint coordinates

----

.. _sec-kinematictree-kinematictree33-computemassmatrixandforceterms:

Class function: ComputeMassMatrixAndForceTerms
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeMassMatrixAndForceTerms <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L237>`__\ (\ ``self``\ , \ ``q``\ , \ ``q_t``\ , \ ``externalForces = []``\ )

- | \ *classFunction*\ :
  | compute generalized mass matrix M and generalized force terms for
  | kinematic tree, using current state (joint) variables q and
  | joint velocities q_t. The generalized force terms f = fGeneralized
  | contain Coriolis and gravity if given in the kinematicTree.
- | \ *input*\ :
  | \ ``q``\ : current joint coordinates
  | \ ``q_t``\ : current joint velocities
  | \ ``externalForces``\ : list of torque/forces in global (world) frame per joint; may be empty list, containing 6D vectors or matrices with 6D vectors in columns that are summed up for each link
- | \ *output*\ :
  | mass matrix \ :math:`{\mathbf{M}}`\  and RHS vector \ :math:`{\mathbf{f}}_{RHS}`\  for equations of motion \ :math:`M(q) \cdot q_{tt} + f(q,q_t,externalForces) = \tau`\ ; RHS is \ :math:`{\mathbf{f}}_{RHS}=\tau - f(q,q_t,externalForces)`\ ; \ :math:`\tau`\  can be added outside of \ ``ComputeMassMatrixAndForceTerms``\

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex)


.. _sec-module-kinematictree-class-kinematictree66:

CLASS KinematicTree66 (in module kinematicTree)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    class to define a kinematic tree, which can be used for building serial or tree-structured multibody systems
    (or robots) with a minimal coordinates formulation, using Plücker coordinate transforms (6x6); for efficient computation, use the C++ ObjectKinematicTree

- | \ *notes*\ :
  | The formulation and structures widely follow Roy Featherstone (http://royfeatherstone.org/) / Handbook of robotics


.. _sec-kinematictree-kinematictree66---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L429>`__\ (\ ``self``\ , \ ``listOfJointTypes``\ , \ ``listOfTransformations``\ , \ ``listOfInertias``\ , \ ``listOfParents = []``\ , \ ``gravity = [0,0,-9.81]``\ )

- | \ *classFunction*\ :
  | initialize kinematic tree
- | \ *input*\ :
  | \ ``listOfJointTypes``\ : mandatory list of joint types 'Rx', 'Ry', 'Rz' denoting revolute joints; 'Px', 'Py', 'Pz', denoting prismatic joints
  | \ ``listOfTransformations``\ : provide a list of Plücker coordinate transforms (6x6 numpy matrices), describing the (constant) link transformation from the link coordinate system (previous/parent joint) to this joint coordinate system
  | \ ``listOfInertias``\ : provide a list of inertias as (6x6 numpy matrices), as produced by the function MassCOMinertia2T66
  | \ ``listOfParents``\ : list of parent object indices (int), according to the index in jointTypes and transformations; use empty list for kinematic chain and use -1 if no parent exists (parent=base or world frame)
  | \ ``gravity``\ : a 3D list/array containing the gravity applied to the kinematic tree (in world frame)

----

.. _sec-kinematictree-kinematictree66-size:

Class function: Size
^^^^^^^^^^^^^^^^^^^^
`Size <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L458>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | return number of joints, defined by size of jointTypes

----

.. _sec-kinematictree-kinematictree66-xl:

Class function: XL
^^^^^^^^^^^^^^^^^^
`XL <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L462>`__\ (\ ``self``\ , \ ``i``\ )

- | \ *classFunction*\ :
  | return 6D transformation of joint i, given by transformation

----

.. _sec-kinematictree-kinematictree66-forwarddynamicscrb:

Class function: ForwardDynamicsCRB
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ForwardDynamicsCRB <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L473>`__\ (\ ``self``\ , \ ``q = []``\ , \ ``q_t = []``\ , \ ``torques = []``\ , \ ``forces = []``\ )

- | \ *classFunction*\ :
  | compute forward dynamics using composite rigid body algorithm
- | \ *input*\ :
  | \ ``q``\ : joint space coordinates for the model at which the forward dynamics is evaluated
  | \ ``q_t``\ : joint space velocity coordinates for the model at which the forward dynamics is evaluated
  | \ ``torques``\ : a vector of torques applied at joint coordinates or list/array with zero length
  | \ ``forces``\ : forces acting on the bodies using special format
- | \ *output*\ :
  | returns acceleration vector q_tt of joint coordinates

----

.. _sec-kinematictree-kinematictree66-computemassmatrixandforceterms:

Class function: ComputeMassMatrixAndForceTerms
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeMassMatrixAndForceTerms <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L494>`__\ (\ ``self``\ , \ ``q``\ , \ ``q_t``\ , \ ``externalForces = []``\ )

- | \ *classFunction*\ :
  | compute generalized mass matrix M and generalized force terms for
  | kinematic tree, using current state (joint) variables q and
  | joint velocities q_t. The generalized force terms f = fGeneralized
  | contain Coriolis and gravity if given in the kinematicTree.
- | \ *input*\ :
  | \ ``q``\ : current joint coordinates
  | \ ``q_t``\ : current joint velocities
  | \ ``externalForces``\ : list of torque/forces in global (world) frame per joint; may be empty list, containing 6D vectors or matrices with 6D vectors in columns that are summed up for each link
- | \ *output*\ :
  | mass matrix \ :math:`{\mathbf{M}}`\  and RHS vector \ :math:`{\mathbf{f}}_{RHS}`\  for equations of motion \ :math:`M(q) \cdot q_{tt} + f(q,q_t,externalForces) = \tau`\ ; RHS is \ :math:`{\mathbf{f}}_{RHS}=\tau - f(q,q_t,externalForces)`\ ; \ :math:`\tau`\  can be added outside of \ ``ComputeMassMatrixAndForceTerms``\

----

.. _sec-kinematictree-kinematictree66-addexternalforces:

Class function: AddExternalForces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`AddExternalForces <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/kinematicTree.py\#L560>`__\ (\ ``self``\ , \ ``Xup``\ , \ ``fvp``\ , \ ``externalForces = []``\ )

- | \ *classFunction*\ :
  | add action of external forces to forces fvp and return new composed vector of forces fvp
- | \ *input*\ :
  | \ ``Xup``\ : 6x6 transformation matrices per joint; as computed in ComputeMassMatrixAndForceTerms
  | \ ``fvp``\ : force (torque) per joint, as computed in ComputeMassMatrixAndForceTerms
  | \ ``externalForces``\ : list of torque/forces in global (world) frame per joint; may be empty list, containing 6D vectors or matrices with 6D vectors in columns that are summed up for each link

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex)

