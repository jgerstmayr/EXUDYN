
.. _sec-module-robotics-special:

Module: robotics.special
------------------------

additional support functions for robotics;
The library is built on Denavit-Hartenberg Parameters and
Homogeneous Transformations (HT) to describe transformations and coordinate systems

- Author:    Martin Sereinig 
- Date:      2021-22-09 


.. _sec-special-velocitymanipulability:

Function: `VelocityManipulability <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/special.py\#L31>`__\ (\ ``robot``\ , \ ``HT``\ , \ ``mode``\ )

- | \ *function description*\ :
  | compute velocity manipulability measure for given pose (homogenious transformation)
- | \ *input*\ :
  | \ ``robot``\ : robot class
  | \ ``HT``\ : actual pose as hoogenious transformaton matrix
  | \ ``mode``\ : rotational or translational part of the movement
  | \ ``singularWeight``\ : Weighting of singular configurations where the value would be infinity,default value=100
- | \ *output*\ :
  | velocity manipulability measure as scalar value, defined as \ :math:`\sqrt(det(JJ^T))`\
- | \ *notes*\ :
  | compute velocity dependent manipulability definded by Yoshikawa, see


----

.. _sec-special-forcemanipulability:

Function: `ForceManipulability <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/special.py\#L63>`__\ (\ ``robot``\ , \ ``HT``\ , \ ``mode``\ , \ ``singular_weight = 100``\ )

- | \ *function description*\ :
  | compute force manipulability measure for given pose (homogenious transformation)
- | \ *input*\ :
  | \ ``robot``\ : robot class
  | \ ``HT``\ : actual pose as hoogenious transformaton matrix
  | \ ``singularWeight``\ : Weighting of singular configurations where the value would be infinity,default value=100
  | \ ``mode``\ : rotational or translational part of the movement
- | \ *output*\ :
  | force manipulability measure as scalar value, defined as \ :math:`\sqrt((det(JJ^T))^{-1})`\
- | \ *notes*\ :
  | compute force dependent manipulability definded by Yoshikawa, see


----

.. _sec-special-stiffnessmanipulability:

Function: `StiffnessManipulability <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/special.py\#L106>`__\ (\ ``robot``\ , \ ``JointStiffness``\ , \ ``HT``\ , \ ``mode``\ , \ ``singularWeight = 1000``\ )

- | \ *function description*\ :
  | compute cartesian stiffness measure for given pose (homogenious transformation)
- | \ *input*\ :
  | \ ``robot``\ : robot class
  | \ ``JointStiffness``\ : joint stiffness matrix
  | \ ``HT``\ : actual pose as hoogenious transformaton matrix
  | \ ``mode``\ : rotational or translational part of the movement
  | \ ``singularWeight``\ : Weighting of singular configurations where the value would be infinity,default value=1000
- | \ *output*\ :
  | stiffness manipulability measure as scalar value, defined as minimum Eigenvalaue of the Cartesian stiffness matrix
  | Cartesian stiffness matrix
- | \ *notes*\ :
  | 
- | \ *status*\ :
  | this function is \ **currently under development**\  and under testing!


----

.. _sec-special-jointjacobian:

Function: `JointJacobian <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/special.py\#L141>`__\ (\ ``robot``\ , \ ``HTJoint``\ , \ ``HTLink``\ )

- | \ *function description*\ :
  | compute joint jacobian for each frame for given pose (homogenious transformation)
- | \ *input*\ :
  | \ ``robot``\ : robot class
  | \ ``HT``\ : actual pose as hoogenious transformaton matrix
- | \ *output*\ :
  | Link(body)-Jacobi matrix JJ: \ :math:`\LU{i}{JJ_i}=[\LU{i}{J_{Ri}},\; \LU{i}{J_{Ti}}]`\  for each link i, seperated in rotational (\ :math:`J_R`\ ) and translational (\ :math:`J_T`\ ) part of Jacobian matrix located in the \ :math:`i^{th}`\  coordiante system, see
- | \ *notes*\ :
  | runs over number of HTs given in HT (may be less than number of links), caclulations in link coordinate system located at the end of each link regarding Standard  Denavid-Hartenberg parameters, see


----

.. _sec-special-massmatrix:

Function: `MassMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/special.py\#L203>`__\ (\ ``robot``\ , \ ``HT``\ , \ ``jointJacobian``\ )

- | \ *function description*\ :
  | compute mass matrix from jointJacobian
- | \ *input*\ :
  | \ ``robot``\ : robot structure
  | \ ``HT``\ : actual pose as hoogenious transformaton matrix
  | \ ``jointJacobian``\ : provide list of jacobians as provided by function JointJacobian(...)
- | \ *output*\ :
  | MM: Mass matrix
- | \ *notes*\ :
  | Mass Matrix calculation calculated in joint coordinates regarding (std) DH parameter:
  | \*\*       Dynamic equations in minimal coordinates as described in MehrkÃ¶rpersysteme by Woernle, , p206, eq6.90.
  | \*\*       Caclulations in link coordinate system at the end of each link

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `solverFunctionsTestEigenvalues.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/solverFunctionsTestEigenvalues.py>`_\  (Ex), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM), \ `manualExplicitIntegrator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/manualExplicitIntegrator.py>`_\  (TM), \ `objectFFRFreducedOrderAccelerations.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderAccelerations.py>`_\  (TM), \ `objectFFRFreducedOrderShowModes.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderShowModes.py>`_\  (TM), \ `objectFFRFreducedOrderStressModesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderStressModesTest.py>`_\  (TM), \ `objectFFRFreducedOrderTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderTest.py>`_\  (TM), \ `objectFFRFTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFTest.py>`_\  (TM)


----

.. _sec-special-dynamicmanipulability:

Function: `DynamicManipulability <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/special.py\#L236>`__\ (\ ``robot``\ , \ ``HT``\ , \ ``MassMatrix``\ , \ ``Tmax``\ , \ ``mode``\ , \ ``singularWeight = 1000``\ )

- | \ *function description*\ :
  | compute dynamic manipulability measure for given pose (homogenious transformation)
- | \ *input*\ :
  | \ ``robot``\ : robot structure
  | \ ``HT``\ : actual pose as hoogenious transformaton matrix
  | \ ``Tmax``\ : maximum joint torques
  | \ ``mode``\ : rotational or translational part of the movement
  | \ ``MassMatrix``\ : Mass (inertia) Maxtrix provided by the function MassMatrix
  | \ ``singularWeight``\ : Weighting of singular configurations where the value would be infinity,default value=1000
- | \ *output*\ :
  | dynamic manipulability measure as scalar value, defined as minimum Eigenvalaue of the dynamic manipulability matrix N
  | dynamic manipulability matrix
- | \ *notes*\ :
  | acceleration dependent manipulability definded by Chiacchio, see , eq.32. The eigenvectors and eigenvalues of N ([eigenvec eigenval]=eig(N))gives the direction and value of minimal and maximal accaleration )
- | \ *status*\ :
  | this function is \ **currently under development**\  and under testing!


----

.. _sec-special-computeik3r:

Function: `ComputeIK3R <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/special.py\#L284>`__\ (\ ``robot``\ , \ ``HT``\ )

- | \ *function description*\ :
  | calculates the analytical inverse kinematics for 3R elbow type serial robot manipulator
- | \ *input*\ :
  | \ ``robot``\ : robot structure
  | \ ``HT``\ : desired position and orientation for the end effector as 4x4 homogeneous transformation matrix as list of lists or np.array
- | \ *output*\ :
  | solutions, list of lists with posible joint angles [q1,q2,q3] (in radiant)
  | to achive the desired position and orientation (4 posible solutions,schoulder left/right, ellbow up/down )
  | left/down, left/up, right/up, right/down
- | \ *notes*\ :
  | only applicable for standard Denavit-Hartenberg parameters
- | \ *status*\ :
  | testet with various configurations and joint angels


----

.. _sec-special-computeikpuma560:

Function: `ComputeIKPuma560 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/special.py\#L355>`__\ (\ ``robot``\ , \ ``HT``\ )

- | \ *function description*\ :
  | calculates the analytical inverse kinematics for Puma560 serial 6R robot manipulator
- | \ *input*\ :
  | \ ``robot``\ : robot structure
  | \ ``HT``\ : desired position and orientation for the end effector as 4x4 homogeneous transformation matrix as list of lists or np.array
- | \ *output*\ :
  | solutions, list of lists with posible joint angles [q1,q2,q3,q4,q5,q6] (in radiant)
  | to achive the desired position and orientation (8 posible solutions,schoulder left/right, ellbow up/down, wrist flipped/notflipped (rotated by pi) )
  | left/down/notflipped, left/down/flipped, left/up/notflipped, left/up/flipped, right/up/notflipped, right/up/flipped, right/down/notflipped, right/down/flipped
- | \ *notes*\ :
  | Usage for different manipulators with sperical wrist posible, only applicable for standard Denavit-Hartenberg parameters
- | \ *status*\ :
  | tested (compared with Robotcs, Vision and Control book of P. Corke


----

.. _sec-special-computeikur:

Function: `ComputeIKUR <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/special.py\#L481>`__\ (\ ``robot``\ , \ ``HTdes``\ )

- | \ *function description*\ :
  | calculates the analytical inverse kinematics for UR type serial 6R robot manipulator without sperical wrist
- | \ *input*\ :
  | \ ``robot``\ : robot structure
  | \ ``HT``\ : desired position and orientation for the end effector as 4x4 homogeneous transformation matrix as list of lists or np.array
- | \ *output*\ :
  | solutions, list of lists with posible joint angles [q1,q2,q3,q4,q5,q6] (in radiant)
  | to achive the desired position and orientation (8 posible solutions,schoulder left/right, ellbow up/down, wrist flipped/notflipped (rotated by pi) )
  | [left/down/notflipped, left/down/flipped, left/up/notflipped, left/up/flipped, right/up/notflipped, right/up/flipped, right/down/notflipped, right/down/flipped]
- | \ *notes*\ :
  | Usage for different manipulators without sperical wrist posible UR3,UR5,UR10, only applicable for standard Denavit-Hartenberg parameters
- | \ *status*\ :
  | under development, works for most configurations, singularities not checked -> ZeroConfiguration not working

