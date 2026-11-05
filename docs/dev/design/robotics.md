# The revision of the robotics submodule: evaluation (revision2026c group RG19)

A design note of open steps: the maintainer's request as given, then what exists, the evaluation, the open questions
and the steps. It leaves the repository when the steps close (WORKFLOW.md §8.4).

## 1. The request (maintainer 2026-11-04)

A new group (requested as RG15, which revision2026b has taken; it is RG19) for the revision of `exudyn.robotics`, a larger amount of work, with this design note and no issues
before the evaluation. RG11.8.11 (#2267, the description of the dict of GetURDFrobotData) is closed: the URDF import
and the connection to the Robotics Toolbox for Python (RTB) of P. Corke are revised as a whole.

**Basics**
- analyze the current robotics library (an environment with the RTB can be provided);
- move the URDF import from utilities to URDFimport.py;
- clean up the robotics files (commented code nobody needs);
- some functions may be replaced by exudyn.HT; additional C++ functions (exudyn.cUtilities is the access point now)
  to improve and speed up.

**URDF import**
- revise and check the import: some relevant things are probably not imported, some offsets or kinematics may be
  lost; it works well for many models. Important: the check of the inertia parameters, which otherwise give wrong or
  unphysical models, while an adjustment of the inertia by suitable parameters stays possible;
- textures, which Exudyn supports now; the storing and loading of Exudyn robot models revised;
- the RTB (release 1.5) changed a lot since the import was written;
- the common robots of the RTB (those of robotics/models.py and 3 very common ones) tested, as an example (the tests
  do not include the toolbox);
- Examples/FurtherExamples/spotModel.py imports spot from .h5; the original URDF files are in
  C:\DATA\cpp\DocumentationAndInformation\URDF - spot, the mars rover, anymal, a KUKA robot and the Stäubli TX2-90 -
  to be tested as well; a local (non-git) data set for experiments and lectures; small hdf5 files (< 1 MB) may be kept
  in the repository; the examples say which files they need.

**The core idea of a revised robotics submodule**
- the class Robot creates a robot similar to ObjectKinematicTree (which came later, hence differences), and both a
  minimal and a redundant coordinate model (with an equivalent control); much could be simpler, some things differ
  between the two; to be checked, changes proposed;
- ObjectKinematicTree, possibly with a Jacobian function (mbs.ComputeItem with a link number, as SensorKinematicTree),
  gives the Jacobian and the static torques in a separate mbs;
- the control and the static compensation could be simpler, also a dynamic compensation (not integrated yet);
- missing: friction and end positions - with MarkerBodiesRelativeRotationCoordinate for the redundant model and the
  single coordinates otherwise, and a CoordinateSpringDamperExt for friction and end stops;
- Robot gets `self.q0`, the zero initial position; models.py gets a common initial configuration (which may differ
  from zero), as in the RTB.

**Trajectory planning**
- robotics.motion has basic PTP profiles on coordinate level, applied in mbs.PreStepUserFunction;
- BasicProfileLinearAcceleration exists, but linear profiles are probably not fully included - check, extend;
- no 5th-order polynomials (s0, s0_t, s0_tt, s1, s1_t, s1_tt as boundary values), used in teaching;
- no continuous path trajectories - linear, screw (including circular), spline - to be added in a common way;
- ideally all trajectory types mixed in class Trajectory; jumps on position level detected (above a threshold) and
  raised;
- suggest what is missing or could be improved.

**Robot and Trajectory together, models.py**
- check the typical robot examples and test models; much code could be unified, the trajectory planning, the control;
  recommend what and how;
- at least: a function of Robot used inside or as PreStepUserFunction for the joint control, one for the minimal and
  one for the redundant formulation, with at least (mbs, t, trajectory, compensateStaticTorques); if trajectory and
  compensateStaticTorques become members of Robot, the interface could match PreStepUserFunction directly for one robot
  (whether the data of the robot is kept, or a separate call is needed);
- Robot.AddSensors(...) with a list of exudyn.OutputVariableType per group of sensors, per joint (angle, angular
  velocity, torque, ...).

**Inverse kinematics of serial robots**
- the analytic inverse kinematics of a standard 6R manipulator with the three hand axes crossing in one point, in Robot;
- a more robust numerical inverse kinematics than the present one (works, but not always, with many tunings),
  possibly with the Jacobian of ObjectKinematicTree, following a standard approach as in the RTB; two tests: a 6R robot
  with quasi-random end-effector HT in the work space (one run with many positions, in the test suite as many as run in
  < 0.25 s), and a similar test with the Franka Emika Panda.

**C++ functions** (maintainer 2026-11-05)
- the analytic inverse kinematics of 6R robots, and possibly other functions, also as C++ functions in
  exudyn.cUtilities, for efficiency.

**Misc**
- models.py still working at the end (it could be revised; some models may be outdated or lack files); wanted: PUMA560,
  UR5, Franka Emika Panda, possibly the Stäubli TX2-90L; models.py has only the kinematics (runs without the RTB);
- the submodule is mostly used internally: interfaces may change (remove what is not needed, Exudyn style, simpler
  code); what examples and test models use from outside stays compatible - at least Robot and RobotLink, the creation
  of the mbs models from Robot, functions like the static torques;
- the robotics examples and test models are partly very old (sensors written to files, matplotlib) and redefine data
  that robotics.models has; some duplicate functions available in simpler form - clean them up at the beginning (to see
  the needed adjustments) and again at the end, when they can be simplified (e.g. the PD control by the creator
  function); slight changes of P/D values in favour of simple scripts are fine.

## 2. What exists (analyzed 2026-11-04, without the RTB)

- **Files** (5170 lines): roboticsCore.py 1476 (RobotLink, RobotTool, RobotBase and their V-classes, Robot,
  StdDH2HT, ModDHKK2HT, InverseKinematicsNumerical), models.py 732 (Manipulator4Rsimple, Manipulator3RSimple,
  ManipulatorPANDA, ManipulatorUR5, ManipulatorPuma560, LinkDict2Robot, LinkDictModDHKK2Robot), motion.py 609,
  mobile.py 599, future.py 583 (MakeCorkeRobot, ComputeIK3R, ComputeIKPuma560, ComputeIKUR - analytic IK, not in the
  public flow), utilities.py 413 (AddLidar and the URDF import: GetRoboticsToolboxInternalModel, LoadURDFrobot,
  GetURDFrobotData), special.py 402 (manipulability measures, JointJacobian, MassMatrix), rosInterface.py 332. About
  140 lines of commented-out code (roboticsCore 36, future 27, motion 26, models 16, special 15).
- **Used from outside** (examples and test models): ProfileConstantAcceleration 90 times, VRobotLink 65, RobotLink 60,
  Robot 28, RobotBase 26, RobotTool 24, StdDH2HT 23, CreateKinematicTree 19, Trajectory 15, Jacobian 15,
  CreateRedundantCoordinateMBS 14, JointHT 13, StaticTorques 8, AddLidar 8, ProfilePTP 5, the Manipulator functions
  10, InverseKinematicsNumerical 3, GetURDFrobotData and GetRoboticsToolboxInternalModel once each.
- **Robot**: links with localHT (or preHT) and a joint type, PD control per link (`PDcontrol=(P, D)`), base, tool,
  gravity, `referenceConfiguration`. `CreateKinematicTree` puts the PD control into ObjectKinematicTree
  (jointPControlVector, jointPositionOffsetVector, ...), compensation into jointForceVector is left to the script;
  `CreateRedundantCoordinateMBS` builds rigid bodies with generic joints and TorsionalSpringDamper or
  LinearSpringDamper for the PD control (the user function list "NOT IMPLEMENTED", the torque loads "DEPRECATED").
  The two models differ: offsets set by `jointPositionOffsetVector` in the tree and by the spring-damper `offset` in
  the redundant model; the scripts set them in their own PreStepUserFunction.
- **Kinematics**: `LinkHT`, `JointHT`, `COMHT`, `Jacobian(HT, toolPosition, mode, linkIndex)` and `StaticTorques(HT)`
  computed in Python from the HT lists (J^T m g per link) - the RTB-like path; the kinematic tree computes the same in
  C++ (MarkerKinematicTreeRigid with its Jacobian, the tree's own Jacobian #3211, the floating base #1549).
- **Scripts**: the serial robot examples (serialRobotTSD, serialRobotKinematicTree, serialRobotFlexible,
  serialRobotInteractiveLimits, humanRobotInteraction, ROSMobileManipulator, serialRobotTest, movingGroundRobotTest)
  each have a PreStepUserFunction with the trajectory, the PD offsets and StaticTorques (3-4 calls each), sensors
  written to files (3-9 per script) and matplotlib (up to 21 lines) - the duplication the request names.
- **Trajectory** (motion.py): `Trajectory` of profiles - `ProfileConstantAcceleration` (a PTP with constant
  acceleration phases in a given time), `ProfilePTP` (synchronized, with max. velocities and accelerations),
  `ProfileLinearAccelerationsList` (accelerations interpolated linearly in time, evaluated by
  `BasicProfileLinearAcceleration`) - all on coordinate (joint) level; no polynomial profiles, no Cartesian paths, no
  check of continuity between profiles.
- **URDF import** (utilities.py): through the RTB (`robot.links`, `link.Ts`, `link.I`, `link.r`, `link.m`, axis) and
  pymeshlab for meshes, colors per face of .obj; special cases: an offset of the UR robots ("no offset for UR5 ?WHY?"),
  flipped joints not implemented (a warning), the links without joints before the first joint into the base graphics;
  no check of the inertia (positive definiteness, triangle inequality), no textures, no fixed joints after the first
  joint (they are merged by the RTB?), no joint limits used in the model (only stored). Written against an older RTB.
- **Inverse kinematics**: `InverseKinematicsNumerical` solves a static problem of the kinematic tree pulled to the goal
  by a spring (jointStiffness, interpolation of the HT in rotation steps, `SolveSafe` with retries) - robust only with
  tuning; the analytic IK of future.py (3R, Puma560, UR) is not used.
- **models.py**: the kinematics (DH or modified DH, masses, inertias, COM) of 4R, 3R, Panda, UR5, Puma560 as dicts;
  no initial configuration, no Stäubli.

### 2.1 Measured (2026-11-05)

Puma560 and UR5 of models.py (6 links), Python 3.13, one core:

| function | time per call |
|---|---|
| `Robot.LinkHT(q)`, `JointHT(q)` | 17 us |
| `Robot.Jacobian(HT)` | 23 us |
| `Robot.StaticTorques(HT)` (a Jacobian per link) | 114 us |
| an exudyn.HT product, `HT.Inverse()`, `cMath.atan2` | 0.5, 0.4, 0.2 us (a numpy 4x4 product 2.3 us) |
| a step of the Puma560 as ObjectKinematicTree with PD control (generalized-alpha) | 9.8 us |
| the same with `StaticTorques(JointHT(q))` in the PreStepUserFunction | 145 us |
| `InverseKinematicsNumerical.Solve` from 0.1 / 0.5 rad off the goal | 0.1 / 0.3 to 0.5 ms, 39 / 31 to 35 of 40 found |
| `InverseKinematicsNumerical.SolveSafe` from the last solution | 3.5 to 5 ms, 8 to 10 of 40 found |

- **The Python kinematics in the control loop costs 15 times the step**: the static compensation in a PreStep
  makes a kinematic tree simulation 15 times slower - the reason for C++ (or the tree's own functions) for
  StaticTorques, the Jacobian and the forward kinematics. The numerical IK is fast enough per solve; it lacks
  robustness (a quarter of the random poses found from the last solution), not speed. An analytic 6R IK in Python
  would take some 30 to 60 us (about 30 cMath functions, 10 HT operations, two ZYZ angle sets per wrist solution) -
  fine for single poses, but a Cartesian path evaluated in each step adds it to every step.
- **Found in the analysis** (the tool frame, InterpolateHTs, the Panda and the Puma560 inertias fixed in #3255, RG19.2.2;
  future.py goes with the C++ IK, RG19.6):
  - `InverseKinematicsNumerical` places the tool on the joint frame of the last link, without that link's `localHT`
    (`MarkerKinematicTreeRigid(localHT=tool.HT)`), while `Robot.Jacobian` and `LinkHT` take the link frame: for the
    UR5 (d6 = 0.0823) the tool frames differ by 0.079 m, and the IK of a pose from `LinkHT(q)[-1] * tool.HT` fails
    for all 40 poses started 0.5 rad off; with the marker on `localHT * tool.HT` 35 of 40 are found (39 from 0.1 rad). The examples and test models write
    the goal as `JointHT(q)[-1] * tool.HT`, which matches the IK; the tool graphics of CreateKinematicTree are on the
    joint frame as well. Which frame the tool HT is relative to - the last joint frame (Exudyn now) or the last link
    frame (DH, the RTB's fkine) - is question 9 (RG19.4, RG19.6).
  - `InverseKinematicsNumerical.InterpolateHTs` takes `n = min(minSteps, 1 + int(rotAng/rotStep))` steps, with
    `minSteps=1` always one: SolveSafe does not interpolate between the poses as intended (`max` meant) - one reason
    for its low rate (RG19.6).
  - future.py: `ComputeIK3R`, `ComputeIKPuma560`, `ComputeIKUR` call `rob.ComputeJointHT`, `rob.HTtranslate` and
    `rob.HTrotateX`, which exudyn.robotics no longer has, on a robot dict with 'stdDH' per link - they cannot run.
    Nothing calls them (RG19.2, RG19.6).
  - models.py: `LinkDict2Robot` refuses `dhMode='modDHKK'` ('untested'), the dhMode of `ManipulatorPANDA`, and the
    deprecated `LinkDictModDHKK2Robot` fails on the base HT (a dict passed as HT) - the Panda of models.py cannot be
    made a Robot (RG19.7). The Puma560 inertias of links 0 and 2 violate the triangle inequality (link 0 has only
    I_zz = 0.35, as in the RTB's data) - the warning of RigidBodyInertia at each use (RG19.3, RG19.7).
  - The RTB environment `venvP310rtb` had roboticstoolbox 1.1.1; `venvP313rtb` has release 1.5.0 (decision 8).
- **exudyn.cUtilities.KinematicChain** (#3259, RG19.6.1): the forward kinematics, the Jacobian and the static torques
  in C++, created by `Robot.GetKinematicChain()`; they agree with the Python functions of Robot to 1.4e-14 (Puma560,
  UR5, Panda, a tree with prismatic joints). Per call for 6 links: LinkHT 1.7 us (17 in Python; most of it the list
  of exu.HT returned), Jacobian 1.0 us (23), StaticTorques 1.0 us (130; the Python version is now the same O(n)
  backward pass, as fast as before because JointHT and COMHT dominate it). A kinematic tree step of the Puma560 with
  the static compensation in the PreStepUserFunction: 14.9 us with `KinematicChain.StaticTorques`, 145 us with the
  Python function, 9.8 us without the compensation.
- **The analytic IK** (#3260, RG19.6.2): `KinematicChain.InverseKinematicsAnalytic` and `Robot.InverseKinematicsAnalytic`
  solve the product of exponentials T(q) = E_1(q_1) ... E_n(q_n) T(0) of the chain by the subproblems of Paden and Kahan
  - independent of the DH convention, of the base and of the tool - for 3R with the axes 2 and 3 parallel, 6R with a
  spherical wrist and 6R of the UR geometry; the geometry is detected from the axes at q = 0. All solutions are checked
  by the forward kinematics: 8 for the Puma560 (standard and modified DH), up to 8 for the UR5 (2 to 8 at random poses,
  the same number as 300 numerical solutions from random starts find), 4 for the 3R. C++ 2.9 / 8.2 / 7.3 us per call
  (3R / Puma560 / UR5), Python 0.36 / 2.5 / 1.6 ms; the two agree to 5.5e-13. In a singular configuration the free angle
  takes the value of qReference (wrist q_5 = 0: q_4 from qReference, q_4 + q_6 from the pose).
- **After #3255**: the IK of a pose `LinkHT(q)[-1] * tool.HT` from 0.1 / 0.5 rad off: UR5 39 / 35 of 40, Puma560 39 /
  31 of 40; SolveSafe, which now interpolates, 12 of 40 for both (from 8 to 10) at 6 to 8 ms - the random poses are far
  apart, a robust numerical IK is the LM of RG19.6. The Panda of models.py: the modified DH parameters, the tool (the
  Franka hand, as the RTB) and the gravity torques agree with the RTB 1.5 Panda to 1e-15; its standard DH parameters
  were wrong (alpha of joints 1 and 2, a = 0.088 instead of 0.0825 for joints 3 and 4) and agree now as well; the
  Puma560 gravity torques agree with the RTB's.

## 3. Evaluation and recommendations

- **Structure** (decided, decision 1): robotics/urdfImport.py (the URDF and RTB functions, moved from utilities.py;
  utilities.py keeps AddLidar), roboticsCore.py (Robot and links), kinematics in roboticsCore or a new
  robotics/kinematics.py (HT of the links, Jacobian, static torques, IK, the Python versions of the C++ functions),
  motion.py (trajectories), models.py, special.py; future.py removed - its analytic IK (3R, Puma560, UR) replaced by
  the revised analytic IK in C++ and Python, MakeCorkeRobot into urdfImport.py.
- **HT**: the lists of 4x4 numpy matrices of LinkHT/JointHT/COMHT become lists of exudyn.HT (composition in C++,
  `Relative`, `Inverse`); the Jacobian and the static torques from the kinematic tree through mbs.ComputeItem of
  MarkerKinematicTreeRigid (the Jacobian of a marker on a link exists in C++) - a separate mbs of the robot without
  dynamics, as the request says; the Python path stays for robots without an mbs (teaching).
- **Robot**:
  - `q0` (the initial joint coordinates), used by both model creators and by the trajectory start;
  - one control description per link - P, D, offset, and optionally the friction (viscous, Coulomb with a regularization)
    and end stops (lower, upper, stiffness, damping) - realized in the tree by its PD vectors and, for friction and
    stops, CoordinateSpringDamperExt on the joint coordinate (node coordinates of the tree's NodeGenericODE2), in the
    redundant model by MarkerBodiesRelativeRotationCoordinate (relative translation for prismatic joints) with the same
    CoordinateSpringDamperExt - one parameter set, two realizations;
  - the control function: `robot.PreStepControl(mbs, t)` for one robot as PreStepUserFunction, with the trajectory and
    the compensation (`None`, `'static'`, later `'dynamic'`) as members set by `robot.SetControl(trajectory,
    compensation='static')`; the model creators store the item numbers they need (the tree, or the spring-dampers) in
    the robot, so the same function serves both formulations; several robots: a mbs.PreStepUserFunction calling each
    robot's function (Python keeps the data in the robot object, nothing is lost);
  - `AddSensors(mbs, outputVariables=[exu.OutputVariableType.Rotation, ...], storeInternal=True)` per joint, returning
    the sensor numbers by group, on SensorKinematicTree or the joints/spring-dampers of the redundant model;
  - static compensation from the tree: the joint torques of gravity at the present coordinates by a static evaluation
    of the tree (ObjectKinematicTree has the inverse dynamics, `ComputeMassMatrixAndODE2LHS` with zero velocities and
    accelerations gives exactly the gravity torques) - a ComputeItem of the tree for it is cheaper than the Python
    loop; dynamic compensation (feed-forward of the inverse dynamics with the trajectory accelerations) by the same
    function with the planned accelerations - the natural next step.
- **Trajectories**: one interface for all segments - `Evaluate(t) -> (q, q_t, q_tt)` - with joint-space segments (the
  existing profiles, a quintic polynomial with s0, s0_t, s0_tt, s1, s1_t, s1_tt, a cubic one) and Cartesian segments
  (linear, circular and screw motion of the tool HT with an orientation interpolation by the rotation vector or slerp,
  splines through via points), the Cartesian ones needing the IK per evaluation; the time law of a Cartesian path
  by the same scalar profiles s(t) (constant acceleration, quintic); Trajectory checks at each junction the jump of q
  (and optionally of q_t) against a threshold and raises.
- **IK**:
  - analytic for 6R with a spherical wrist (Pieper): position of the wrist center from the first three joints (closed
    form for the common geometries, e.g. Puma and Stäubli; UR has no spherical wrist and its own closed form, which
    future.py has), the wrist angles from the remaining rotation; all 8 solutions, the one nearest to a given q chosen;
  - numerical: Levenberg-Marquardt / damped least squares on the 6D pose error (the logarithm of the relative HT, as
    the RTB's ik_LM), with the Jacobian of the tree or the Python Jacobian, joint limits by clamping, restarts from
    random q for robustness; no mbs needed per solve.
- **C++ in exudyn.cUtilities** (decided, decision 10): a small kinematic chain in C++ - the link data (preHT,
  localHT, joint type, parent, mass, COM, inertia) given once from Robot, as a class of cUtilities
  (`cUtilities.KinematicChain`) - with:
  - the forward kinematics (the HT of all joints and links), the Jacobian of a point or frame of a link, and the
    static torques (J^T m g summed in one backward pass, O(n) instead of a Jacobian per link, O(n^2));
  - the analytic IK of 6R robots with a spherical wrist (Pieper: the wrist center from the first three joints, closed
    form for the Puma/Staeubli/KUKA geometry; the wrist angles from the remaining rotation; up to 8 solutions, the one
    nearest to a given q) and of the UR geometry (no spherical wrist, its own closed form);
  - the numerical IK by Levenberg-Marquardt on the 6D pose error (`LogSE3` of the relative HT, as the RTB's ik_LM)
    with the Jacobian above - the iteration loop in C++, restarts from random q in Python or C++;
  - shared with ObjectKinematicTree, which computes the same transformations and Jacobians (CObjectKinematicTree), so
    that the two cannot differ; every C++ function has a Python version of the same algorithm in exudyn.robotics,
    the two docstrings citing each other - for checking, for teaching, and as the reference of the tests.
  Expected: about 1 us for the forward kinematics and the static torques of 6 links (100 times the Python path), a few
  us for the analytic IK; to be measured in RG19.6 against the table of section 2.1.
- **URDF import**: inertia check (mass > 0, inertia positive definite, triangle inequality of the principal moments)
  with a report per link and an option to correct (e.g. a minimum ratio, or replace by the inertia of the mesh's
  bounding box with the given mass); textures from the meshes (pymeshlab gives texture coordinates and file names for
  .dae/.obj); flipped joints implemented; the UR offset understood rather than special-cased; the store and load of
  Exudyn robot models as one dict (hdf5) with the graphics; checked against RTB 1.5 with the environment the
  maintainer provides.
- **Data**: a local data set outside the repository, with a variable or argument for its path (e.g.
  `EXUDYN_ROBOT_DATA`), examples that say what they need and skip with a message when it is missing; small hdf5 files
  of models (< 1 MB) in the repository.
- **Scripts**: the serial robot examples and test models rewritten on `PreStepControl` and `AddSensors`, storeInternal
  and PlotSensor instead of files and matplotlib; the redefined robot data replaced by models.py.

## 4. Decisions and open questions

### 4.1 Decided (maintainer 2026-11-05)

1. **File layout**: the URDF import moves to `robotics/urdfImport.py` (the lowerCamelCase of the module files);
   future.py is removed once its content has a place - the analytic IK as the revised C++ and Python IK (RG19.6),
   MakeCorkeRobot into urdfImport.py.
8. **RTB environment**: `venvP313rtb` with roboticstoolbox 1.5.0 (`pip install exudyn[all]`; further packages may be
   installed there).
9. **Tool frame**: the tool HT is relative to the frame of the last link (after its localHT), the DH convention and
   the RTB's - in LinkHT, the Jacobian, the IK and the graphics of the kinematic tree (#3255).
10. **C++**: a kinematic chain class in exudyn.cUtilities (forward kinematics, Jacobian, static torques, the analytic
    IK of 3R, 6R with a spherical wrist and the UR geometry, the numerical LM IK); each function has a Python version
    of the same algorithm in exudyn.robotics, cross-referenced, for checking and teaching.
- Fixed now: the problems of section 2.1 (#3255); the Panda of models.py is `ManipulatorFrankaEmikaPanda`
  (`ManipulatorPANDA` deprecated), the Puma560 inertias are physical (the change is said in its docstring).

### 4.2 Open (numbered as before)

2. **The joint control in the scripts.** Today every robot example writes its own PreStepUserFunction: it evaluates
   the trajectory, writes the joint offsets into the model (`jointPositionOffsetVector` of the kinematic tree, or the
   `offset` of each spring-damper of the redundant model) and adds `StaticTorques`. Proposed: the robot keeps the
   trajectory and the kind of compensation, `robot.SetControl(trajectory, compensation='static')`, and offers
   `robot.PreStepControl(mbs, t)`, which a script passes directly, `mbs.SetPreStepUserFunction(robot.PreStepControl)`;
   it works for both models because the creators store the item numbers in the robot. With several robots, the
   script's own PreStep calls `PreStepControl` of each. Agreed?
3. **Friction and end stops of the joints.** Neither model has them now. Proposed: one set of parameters per link in
   RobotLink (viscous and Coulomb friction, lower and upper stop with stiffness and damping), realized by a
   CoordinateSpringDamperExt on the joint coordinate - in the kinematic tree on its node's coordinate, in the
   redundant model between the two bodies by MarkerBodiesRelativeRotationCoordinate. Agreed?
4. **Where the static torques come from.** Decision 10 puts them into the C++ chain (O(n), shared with
   ObjectKinematicTree). Proposed consequence: `Robot.StaticTorques` calls the chain, the present Python loop becomes its
   Python version; no separate ComputeItem of ObjectKinematicTree for them. Agreed, or a ComputeItem of the tree as
   well (for a tree built without Robot)?
5. **Cartesian paths need the IK.** A linear or circular path of the tool is given in Cartesian space; the joint
   controller needs joint coordinates. Either the IK runs at each evaluation in the PreStep (exact; a few us with the
   analytic C++ IK, so affordable now), or the path is converted once before the simulation into a joint trajectory
   sampled in time and splined (fast, approximate between the samples). Recommended now: the IK at each evaluation,
   the sampled form as an option for the numerical IK. Agreed?
6. **The fate of InverseKinematicsNumerical.** It solves the IK as a static problem of a kinematic tree in its own mbs
   (robust only with tuning, section 2.1). The LM IK of decision 10 does not need an mbs. Replace it (the class kept
   for a deprecation period, calling the LM IK) or keep both?
7. **Where the local robot data set is found.** The URDF files (spot, the mars rover, anymal, KUKA, Staeubli) are
   not in the repository; examples and lectures that use them need their path. By an environment variable (e.g.
   `EXUDYN_ROBOT_DATA`), a key of `exudyn.config` (the ~/.exudyn/config.json of the settings), or only an argument of
   the loading function? Without the data, the examples say what is missing and stop.

## 5. Steps proposed (group RG19)

- RG19.1 Analyze the robotics submodule and propose the revision (this note): .1 the code analysis (done), .2 the URDF
  import against RTB 1.5 and the local data set (with the maintainer's environment).
- RG19.2 Clean up (.1 done: the commented-out code and the dead branches of all files except future.py, 220 lines,
  #3254; .2 done: the problems of section 2.1, #3255; .3 done: urdfImport.py, #3256): the commented code, future.py removed after RG19.6, exudyn.HT in the kinematics; the examples
  and test models cleaned up a first time (sensors stored internally, PlotSensor, models.py data).
- RG19.3 Revise the URDF import: the inertia check and correction, textures, flipped joints, the UR offset, the store
  and load of Exudyn robot models; tested with the RTB robots and the local data set in examples.
- RG19.4 Revise Robot: q0, the control description with friction and end stops for both models, SetControl and
  PreStepControl, AddSensors, the static (and dynamic) compensation from the kinematic tree.
- RG19.5 Extend the trajectories: quintic and cubic polynomials, linear profiles completed, Cartesian linear, circular,
  screw and spline paths, mixed in Trajectory with the check of jumps.
- RG19.6 Inverse kinematics: analytic for 6R with a spherical wrist, numerical by Levenberg-Marquardt; the two tests
  (6R, Panda); the kinematic chain, the IK and the static torques in C++ (exudyn.cUtilities, section 3); .1 done: the
  KinematicChain with the forward kinematics, the Jacobian and the static torques (#3259); .2 done: the analytic IK
  (3R, 6R with a spherical wrist, UR) in the chain and in Robot, future.py removed (#3260); open: the LM IK in the
  chain with its Python version.
- RG19.7 Revise models.py: PUMA560, UR5, Panda, Stäubli TX2-90L, their initial configurations; all models checked.
- RG19.8 Clean up the robotics examples and test models on the revised interfaces.
