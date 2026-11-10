# The system Jacobians inside

How CSystem assembles the Jacobian of the implicit and static solvers from the local Jacobians of the items
(`src/Main/CSystemJacobians.cpp`), and why it is built so. What a user sets is {ref}`sec-systemjacobian`.

## 1. The functions

| function | rows | what |
|---|---|---|
| `JacobianODE2RHS` | ODE2 | per object `ComputeLocalJacobianODE2` and `AddLocalJacobianODE2`; then the loads (`JacobianODE2Loads`, with `computeLoadsJacobian`) and GeneralContact |
| `JacobianODE1RHS` | ODE1 | per object `ComputeLocalJacobianODE1` and `AddLocalJacobianODE1`; then `JacobianODE1Loads` |
| `JacobianAE` | AE, and the transposed block in the ODE2 and ODE1 rows | per constraint, body with algebraic equations and node with algebraic equations (Euler parameters) `ComputeObjectJacobianAE` / `ComputeNodeJacobianAE` and `AddLocalJacobianAE` |
| `NumericalJacobianAE` and the branches of `doSystemWideDifferentiation` | all | the system residual differentiated by every coordinate; the AE block of the reaction forces from the unit Lagrange multipliers (`ComputeODE2ProjectedReactionForces`), with the factors of the solver |

- `LocalJacobian` says where each block of an object is: its own (`temp.jacobianODE2Container`), the numerical one of
  CSystem (`temp.localJacobian`), the coupling block by the ODE1 coordinates (`temp.localJacobianODE1`).
- The object decides how its block is computed, through virtual functions of CObject: `ComputeLocalJacobianODE2`
  (default: `ComputeJacobianODE2_ODE2` if the object declares the function flags in `GetAvailableJacobians`),
  `ComputeLocalJacobianAE` (default: `ComputeJacobianAE` of a body with algebraic equations); CObjectConnector and
  CObjectConstraint override them with the connector interface or the path of the marker data.
- An object that returns false is differentiated numerically by CSystem by its coordinates without duplicates
  (`GetLocalToGlobalODE2NumDiff`, #1424), with the Lie group nodes composed (`AddNumDiffLieGroup`). This perturbs the
  global state, so it is serial; it is not in CObject because it needs the Lie group composition of CSystem.

## 2. The connector interface

A connector's ODE2 rows are $J\tp f$, with the force $f(p, v)$ of the marker kinematics. Its Jacobian

$$
\frac{\partial (J\tp f)}{\partial \dot q} = J\tp \frac{\partial f}{\partial v} J, \qquad
\frac{\partial (J\tp f)}{\partial q} = J\tp \frac{\partial f}{\partial p} J
  + \left.\frac{\partial (J\tp f)}{\partial q}\right|_{f}
  + J\tp \frac{\partial f}{\partial v} \frac{\partial v}{\partial q}
$$

is computed by `ComputeJacobianODE2Connector` in parallel, as it does not touch the system state:

- $\partial f/\partial p$, $\partial f/\partial v$ by automatic differentiation of the force (`ComputeConnectorForce...Diff`,
  `ItemConnectorInterface(jacobian='automatic')`, #2745), or by a local numerical differentiation of the force by the
  marker kinematics (`jacobian='numerical'`): the contacts, the rolling disc, ConnectorCoordinateSpringDamperExt and
  the intrinsic RigidBodySpringDamper, whose logarithm of SE(3) has no automatic derivative at zero relative motion.
- The second term is the Jacobian derivative of the markers, with `jacobianConnectorDerivative` (on by default:
  without it objectFFRFTest needed 69 instead of 54 Newton iterations, objectFFRFreducedOrderTest 840 instead of 799).
- The third, dv/dq, only where the connector requests it (`velocityDerivative=True`: friction, rolling): its size is
  $\partial f/\partial v$ times a velocity of the moving frame, which is large for regularized friction. Without it
  rotatingTableTest needed 1083889 instead of 26740 Newton iterations; for the spring-dampers it changed no Newton
  count of the test suite. `CMarker::GetVelocity_q` gives it - zero where the Jacobian is constant, closed forms for
  the rigid nodes (`RigidBodyNodeVelocity_q`); a marker returning false leaves the connector to CSystem.
- **The direction along the Euler parameters.** The rotation matrix of the Euler parameters is not normalized,
  `dA/de e = 2 (A - I)`, while the marker Jacobians have no such direction (`G e = 0`). The predictor of every time
  step leaves the unit sphere, so the first Newton increment has a part along e: without it rollingCoinPenaltyTest
  needed 12036 instead of 9654 iterations. `CMarker::GetRadialKinematics` gives the derivatives of position and rotation
  along e and the row `e^T/(e^T e)`; the chains add one force evaluation per Euler parameter marker. For constraints the
  part goes into the rows of the equations only (`temp.localJacobianAE_ODE2radial`): the reaction forces have none,
  and with it restartFileTest and userJointTest did not converge.
- **Constraints on rigid markers** (#2884): `ConstraintEquationsDerivativesRigid` (CObjectConnector.cpp) gives the
  equations and their rows in the 12 variations [dp0, dtheta0, dp1, dtheta1] (`RigidMarkersDerivatives`), for both
  `ConstraintJacobianRigidMarkers` and `ConstraintReactionForcesRigidMarkers`: analytic from
  `CObjectConstraint::ComputeConstraintEquationsRigidDerivatives` where the constraint has it - the 3D joints, on the
  relative frame of their joint frames, `src/System/JointRelativeFrame.h` -, else by automatic differentiation of
  `ComputeConstraintEquationsRigidDiff` (`AutoDiff<12>`). The velocity level of the joints is the same rows times the
  twists. A chain of 400 Euler parameter bodies, 200 steps, generalized-alpha with modified Newton, automatic /
  analytic: JointRevoluteZ 1.06 / 0.85 s (reaction forces 0.39 / 0.19 s), JointGeneric revolute 1.00 / 0.87 s,
  universal 1.08 / 1.00 s, JointPrismaticX 0.18 / 0.15 s; with full Newton the factorization dominates, 6.0 / 5.8 s.
  The rest is the kinematics and Jacobians of the markers and their projection.
- The Lie group node NodeRigidBodyRotVecLG (#3217): its rotation velocity coordinates are the local angular velocity
  omega, and the system differentiates its rotation coordinates by the local composition A Exp(delta). So
  `RigidBodyNodeVelocity_q` gives d(A v)/d delta = -A v~ for the angular velocity and the velocity of a point, and
  `CObjectRigidBody::ComputeJacobianODE2_ODE2LieGroup` the gyroscopic forces by omega (omega~ J - (J omega)~) and,
  for a center of mass, the force A w by omega and by delta; JacobianODE2Loads differentiates the loads by the
  composition too (`SetNumDiffObjectLieGroup`), as the mass proportional load of a body with a center of mass turns
  with it. A chain of 200 bodies with spherical joints, 200 steps of full Newton: 1.96 / 1.24 s, 0.83 s with 4 threads.
- ObjectFFRFreducedOrder (#3212): `ComputeJacobianODE2_ODE2` with the reduced stiffness and damping matrices, the
  derivatives of the quadratic velocity vector by the modal coordinates and their velocities analytic (the formulas in
  the comment of the function; W = S (I x omega) once, O(9 m^2) for m modes), by the rotation parameters and their
  velocities by forward differences of `ComputeQuadraticVelocityVector`, which `ComputeODE2LHS` uses too, at the
  rotation and G of the node's parametrization; the Euler angle terms with G_t, which the left-hand side does not
  have, are not in it either. Within 1e-8 of the system-wide Jacobian on Euler parameters and Tait-Bryan angles.
  perfObjectFFRFreducedOrder (2000 steps), numerical / own: 8 modes 0.35 / 0.32 s (modified Newton), 1.13 / 0.92 s
  (full); 24 modes 2.40 / 1.63 s, 13.4 / 6.85 s; the same Newton iterations. The rest of the time is the mass matrix
  and the markers, so the quadratic velocity terms are not simplified; `numericalJacobian` is the switch back.
- ObjectKinematicTree (#3211): `ComputeJacobianODE2_ODE2` by forward differences of the inverse dynamics
  `ComputeMassMatrixAndODE2LHS(..., qTotal, q_t, q_tt)` - the recursive Newton-Euler forces with the accelerations of
  the joints - in temporaries of the thread, so in the parallel assembly; with `jacobianMassMatrixDerivative` (the
  default) at the current accelerations, so that it contains d(M q_tt)/dq, which the other Jacobians leave out. An
  analytic derivative of the recursion (by the derivatives of the link motions) is not done: the differences cost
  2n evaluations of O(n), as the mass matrix, and the measured gain is the term, not the arithmetic. Measured
  (generalized-alpha, full Newton, a CartesianSpringDamper from every link to the ground), the system's numerical
  Jacobian / the tree's: 20 links, 200 steps, 664 / 571 iterations, 0.170 / 0.149 s, 0.100 / 0.088 s with four
  threads; 60 links, 50 steps, 1591 / 317 iterations, 6.07 / 1.18 s, 4.27 / 0.46 s with four threads - the mass
  matrix of a long chain depends strongly on its coordinates, and without the term the Newton method needs some 30
  iterations per step. With a floating base or a forceUserFunction the Jacobian stays that of the system.
- The floating base of ObjectKinematicTree (#1549): `baseNodeNumber`, a 3D rigid body node; the coordinates of the
  tree are the joint coordinates and those of the base node. `BaseMotion` gives the placement and the motion of the
  base in its frame - $(\omega, A^T \dot p)$ and its time derivative -, the parent motion of the root links in the
  kinematics; in the forces the bias acceleration of the base, $\dot G_{local}\dot\theta$ and $-\omega \times v$, with the
  gravity, and the forces of the root links on the base by the transposed motion subspace, $A f$ and
  $G_{local}^T n$; in the mass matrix the composite inertia of the whole tree in the base, its block $S^T I S$ and the
  couplings with the joints, the force of the composite inertia of a joint carried into the base. The marker
  Jacobians get the columns of the base; their derivatives by the base coordinates are not given
  (`CMarker::HasJacobianDerivative`, false for MarkerKinematicTreeRigid then), so the connectors on them are
  differentiated by the system. Checked (kinematicTreeFloatingBaseTest.py) against a redundant model of rigid bodies
  and revolute joints on Lie group nodes, Euler parameters and Tait-Bryan angles, to the error of the time
  integration, which falls by 16 for 4 times the steps; RK44 with the explicit Lie group integration on the Lie group
  node.
- MarkerKinematicTreeRigid (#3216): `CObjectKinematicTree::JacobianColumnDerivative` gives the derivative of a
  Jacobian column of a link's chain by a joint coordinate - a revolute joint above turns the column about its axis, a
  prismatic one leaves it, a joint at or below moves the point -, from which the marker has the Jacobian derivative
  (`JacobianDerivativeAvailable`) and `GetVelocity_q`, so that connectors on it are on the connector interface and in
  the parallel assembly; within 2e-8 of the system-wide Jacobian, also at velocities with a contact (dv/dq). The marker
  functions read the frames of the links from the cache of the tree, which `CSystem::PreComputeItems` validates
  serially before the parallel loops (#3229, userFunctions.md).
- ConnectorHydraulicActuatorSimple: the force by automatic differentiation, the block by the pressures analytic
  (`CObject::ComputeJacobianODE2_ODE1` with the system and temporary data, which finds the marker Jacobians in
  temp.markerTemp); the pressure equations analytic by `CObjectConnector::ComputeJacobianODE1RHSConnector` (#3214),
  which `CSystem::ComputeLocalJacobianODE1` calls for a connector with ODE1_ODE1_function: by the pressures, by the
  stroke and its velocity, chained with the marker Jacobians, the direction along the Euler parameters and dv/dq of the
  markers - false, numerical, for a marker without dv/dq. The derivative of the valve flow, sqrt of the pressure
  difference, is limited at a difference of 1e-8 times the system pressure. The Newton iterations are those of the
  numerical Jacobian; HydraulicActuator2Arms with full Newton 1.00 instead of 1.23 s.

## 3. The parallel assembly

- ODE2 rows (#1203): `listComputeObjectODE2LhsNoUF` in `ParallelFor`, `ComputeLocalJacobianODE2` with `analyticOnly` and
  `AppendLocalJacobianODE2` into triplets per thread; the objects that need a numerical block and those with Python
  functions afterwards, serially.
- AE rows (#1202): `objectsConstraintWithAENoUF` and `nodesODE2WithAE` in one `ParallelFor`, `AppendLocalJacobianAE`
  per thread (`LocalJacobianAEBlocks`, shared with `AddLocalJacobianAE`); the bodies with algebraic equations and the
  constraints with Python functions serially.
- Above `parallel.multithreadedLowerLimitJacobians` items (default 20), split by `GetTaskSplit` into
  `taskSplitTasksPerThread` (16) tasks per thread of equal numbers of items, which the threads take as they become
  free. No cost per item (#858): at 16 tasks per thread items of different cost even out.
- Measured: 2000 joints on Euler parameter bodies 2.44 / 0.34 ms for 1 / 8 threads; 2000 free rigid bodies (their own
  Jacobian by automatic differentiation, #3209) 1.57 ms numerical and serial, 0.39 / 0.06 ms for 1 / 8 threads; an
  ANCF cable of 2000 elements with spring-dampers 20.9 / 4.6 ms.

## 4. The checks

- `doSystemWideDifferentiation` is the reference: python/testing/test_systemJacobians.py compares every block of the
  local Jacobians with it for small models of the item types, at zero and nonzero velocities, and the parallel
  assembly with the serial one.
- d(C_q^T lambda)/dq and /dq_t with `constraintReactionDerivative` (#3230): `CSystem::JacobianODE2ReactionForces`,
  called at the end of `JacobianODE2RHS` (also with doSystemWideDifferentiation), differentiates
  `ComputeObjectReactionForces` of each object of `listObjectProjectedReactionForcesODE2` and the reaction forces of
  the nodes with algebraic equations by their own coordinates (`SetNumDiffObjectLieGroup`, the Lie group nodes by the
  composition) and velocities, serially. Measured with full / modified Newton, off / on: fourBarMechanismIftomm 1058 /
  802 and 2739 / 2719 iterations, 0.023 / 0.031 s; bricardMechanism, sliderCrank3Dbenchmark, serialRobotTest,
  rigidBodyCOMtest, sphericalJointTest, kinematicTreeAndMBStest, superElementRigidJointTest and
  revoluteJointPrismaticJointTest within 3 iterations, with full Newton 1.4 to 2.6 times the time; so off by default.
- What none of the Jacobians contains: d(M q_tt)/dq - symbolicMassMatrixDerivativeTest.py: three rigid bodies on
  Tait-Bryan angles spinning at some 30 rad/s, generalized-alpha, full Newton, 200 steps, 417 iterations with the exact
  Jacobian of SymbolicSystem, 614 without the term, as SolveDynamic (#3228) -; dependencies outside the coordinates of an
  item (a user function reading a sensor) only the system-wide one.
