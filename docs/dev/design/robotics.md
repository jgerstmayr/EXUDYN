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

## 3. Evaluation and recommendations

- **Structure**: robotics/urdfImport.py (the URDF and RTB functions, moved from utilities.py; utilities.py keeps
  AddLidar), roboticsCore.py (Robot and links), kinematics in roboticsCore or a new robotics/kinematics.py (HT of the
  links, Jacobian, static torques, IK), motion.py (trajectories), models.py, special.py; future.py dissolved - its
  analytic IK into the IK of Robot, MakeCorkeRobot into urdfImport.py.
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

## 4. Open questions (to be decided by the maintainer)

1. The file layout: urdfImport.py (the name style of the module files is lowerCamelCase - `URDFimport.py` as
   requested, or `urdfImport.py`)? future.py dissolved?
2. The control interface: `robot.SetControl(trajectory, compensation)` and `robot.PreStepControl(mbs, t)` - agreed, and
   with several robots one PreStepUserFunction calling each?
3. Friction and end stops: one parameter set per link (RobotLink fields), realized by CoordinateSpringDamperExt in
   both models - agreed?
4. The static torques: from the kinematic tree (a ComputeItem of ObjectKinematicTree giving the inverse dynamics, to be
   added), keeping the Python StaticTorques for robots without mbs?
5. Cartesian trajectories need the IK at each evaluation (or a precomputed joint trajectory sampled in time): which -
   IK per PreStep evaluation (exact, slower) or precomputed and splined (fast, approximate)?
6. The numerical IK: Levenberg-Marquardt as the RTB's ik_LM (recommended), replacing InverseKinematicsNumerical, or
   kept beside it?
7. The data set path: an environment variable, `exudyn.config`, or an argument only?
8. The RTB environment for the analysis of the import against release 1.5: when it is available, the URDF part of the
   analysis is repeated with it.

## 5. Steps proposed (group RG19)

- RG19.1 Analyze the robotics submodule and propose the revision (this note): .1 the code analysis (done), .2 the URDF
  import against RTB 1.5 and the local data set (with the maintainer's environment).
- RG19.2 Clean up: urdfImport.py, the commented code, future.py dissolved, exudyn.HT in the kinematics; the examples
  and test models cleaned up a first time (sensors stored internally, PlotSensor, models.py data).
- RG19.3 Revise the URDF import: the inertia check and correction, textures, flipped joints, the UR offset, the store
  and load of Exudyn robot models; tested with the RTB robots and the local data set in examples.
- RG19.4 Revise Robot: q0, the control description with friction and end stops for both models, SetControl and
  PreStepControl, AddSensors, the static (and dynamic) compensation from the kinematic tree.
- RG19.5 Extend the trajectories: quintic and cubic polynomials, linear profiles completed, Cartesian linear, circular,
  screw and spline paths, mixed in Trajectory with the check of jumps.
- RG19.6 Inverse kinematics: analytic for 6R with a spherical wrist, numerical by Levenberg-Marquardt; the two tests
  (6R, Panda).
- RG19.7 Revise models.py: PUMA560, UR5, Panda, Stäubli TX2-90L, their initial configurations; all models checked.
- RG19.8 Clean up the robotics examples and test models on the revised interfaces.
