# The revision of the robotics submodule (revision2026c group RG19)

A design note of open steps: the open tasks, the maintainer's request as given, the decisions taken and a list of what
is done. The analysis, the measurements and the details of each step are in the log (exudynRevisionLog2026c.md, the
entries RG19.*). The note leaves the repository when the steps close (WORKFLOW.md §8.4).

## 1. State (2026-11-08)

The steps of the first request are done (section 4). What remains are the steps below; each has its issue.

### 1.1 Decided, to be done

| step | issue | what | depends on |
|---|---|---|---|
| RG19.10.2 | #3353 | the articulated dummy of humanRobotInteraction.py as a Robot in HDF5, in `RobotHDF5` of CommonData_git | - |
| RG19.11.2 | #3355 | all other robot examples moved to `python/Examples/robotics` | - |
| RG19.13.1 | #3357 | the inertia workflow of section 3.3: `densityForInertiaEstimation`, and the completion of partly given inertias as a function of its own | RG19.12.1 |
| RG19.14.1 | #3359 | a floating base for `Robot` by the one of `ObjectKinematicTree` (#1549) | - |
| RG19.14.2 | #3360 | spotModel.py on the floating base and `RobotFromURDFdata`; ANYmal and the Mars rover imported the same way | RG19.14.1 |
| RG19.15.2 | #3361 | when the deprecation period of `jointLoadUserFunctionList` and `createJointTorqueLoads` of `CreateRedundantCoordinateMBS` ends: remove them | the deprecation period |
| RG19.16 | #3362 | serialRobotInteractiveLimits.py adapted to new features of the renderer and checked by hand (sliders, mouse drag, end stops); it has run only without windows so far | the renderer features |

### 1.2 Open decisions of the maintainer

1. **The proposals of section 3.5**: leaving out links such as the fingers of the Panda, and one definition of models.py and the URDF files, with the robots to add.

### 1.3 Not revised, no step

exudyn.robotics.mobile (mecanum wheels), special.py (manipulability) and rosInterface.py, which is untested (no ROS in
the tests); ROSMobileManipulator.py keeps its state machine.

## 2. The request (maintainer 2026-11-04)

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

## 3. Decisions (maintainer 2026-11-05)

### 3.1 First decisions

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
- Fixed: the problems found in the analysis (#3255); the Panda of models.py is `ManipulatorFrankaEmikaPanda`
  (`ManipulatorPANDA` deprecated), the Puma560 inertias are physical (the change is said in its docstring).

### 3.2 Further decisions (numbered as before)

2. **Joint control**: the robot keeps the trajectory and the compensation, `robot.SetControl(trajectory,
   compensation=None | 'static')`, and `robot.PreStepControl(mbs, t)` is passed directly,
   `mbs.SetPreStepUserFunction(robot.PreStepControl)` (done for the kinematic tree, #3262, and the redundant model,
   #3270).
3. **Friction and end stops**: per link in RobotLink (viscous and Coulomb friction, lower and upper stop with
   stiffness and damping), by a CoordinateSpringDamperExt on the joint coordinate (the node coordinate of the kinematic
   tree; MarkerBodiesRelativeRotationCoordinate between the bodies of the redundant model). `Robot(...,
   enableEndStops=False)`; global viscous and Coulomb friction as arguments of a function of Robot that sets them in
   every link; the models get friction values where known; end stops from the models or the URDF (joint limits),
   friction from the URDF (`<dynamics damping friction>`) into the links where given.
4. **Static torques** (the question of the maintainer, answered here): ObjectKinematicTree evaluated with zero
   velocities and accelerations gives the static generalized forces as well - the gravity part of its ODE2 right-hand
   side - from the same data (joint frames, masses, COMs, gravity), so the two are consistent by construction and by
   test (kinematicChainTest.py compares the Python algorithm, the C++ chain and J^T m g). The difference is where they
   are used: the tree needs an mbs, its state set and an evaluation of the item (a ComputeItem or ComputeODE2RHS of the
   whole system, which also contains the PD forces and other loads); the chain is standalone - a controller, the
   planning of a trajectory, the IK and a PreStepUserFunction call it without an mbs, at 1 us. They take the same input;
   the IK needs only the kinematic part (joint types, preHT, localHT, base, tool), the static torques also masses, COMs
   and gravity. Recommended: no ComputeItem of the tree for the static torques; if a tree built without Robot needs
   them, a KinematicChain created from the parameters of the ObjectKinematicTree (a function GetKinematicChain of the
   tree's parameters), so both still share one algorithm and one data set. The dynamic compensation (M a + h) is the
   tree's domain.
5. **Cartesian paths**: the IK at each evaluation, also for the numerical IK (ProfileCartesianLinear, #3262).
6. **InverseKinematicsNumerical**: replaced by the LM IK, the class kept for a deprecation period, calling it.
7. **The robot data set** (for the examples replaced by decision 11): by a path given to the loading function; the folders of spot, the mars rover, anymal,
   KUKA and Staeubli copied to `EXUDYN_git/tmp/URDF` (inside the repository, not committed, as tmp/ is ignored);
   without the data an example says what is missing and where to download it (the ros-industrial repositories on
   GitHub, e.g. https://github.com/ros-industrial/staubli_experimental), and stops.

### 3.3 Decisions of 2026-11-08 (maintainer)

11. **The data of the robots**: the second repository `CommonData_git`, for the internal developers and users, lies
    beside the Exudyn repository and is reached by a relative path. `CommonData_git/data/Exudyn/robotics/URDF` has the
    URDF folders the examples use (Stäubli TX2-90, UR5, Franka Emika Panda), `.../RobotHDF5` the robots stored as HDF5
    that are too large for the Exudyn repository (the data there must not grow without limit). An example runs when
    CommonData_git is there, and says where to download its URDF otherwise. `EXUDYN_git/tmp` is a work folder only. The
    test models use `python/TestModels/testData/urdf`. This replaces decision 7 for the examples.
12. **Examples**: `python/Examples/robotics` holds all robot examples in the future, its data in `robotics/data`.
13. **A Robot from the URDF data**: one function, `urdfImport.RobotFromURDFdata`; everything is then saved from the
    class Robot, by `GetDictionary` and also by `SaveToHDF5` / `LoadFromHDF5`.
14. **Legged and mobile robots** use a floating base; the Spot model is adapted to it.
15. **The workflow of the inertias** of imported robots (proposed here on the maintainer's request):
    1. *The inertials of the URDF come first*, and a URDF with correct inertials is what to look for. They are checked
       by `_InertiaProblems` (positive mass, positive definite, triangle inequality) and reported per link.
    2. *A mesh gives the surface, not the interior*: hollow arms, drive trains and cables are unknown, so a density of
       a material gives nonsense. Only one overall density of the enclosed volume of a link makes sense, around
       1 kg/dm^3; for robots with known masses a good value can be guessed from them. `GetURDFrobotData` gets
       `densityForInertiaEstimation=None` (in place of `meshDensity`), used only when it is not None, for links without
       any inertial data.
    3. *Partly given data* (the mass but no inertia, or an inertia with missing components) is completed by a function
       of its own, documented as an estimate: the shape's inertia from the meshes with that density, scaled to the given
       mass where there is one, overrides the given values, and the principal moments are then adapted as little as
       possible so that the triangle inequality holds (`_CorrectedInertia`). Every link reports what was changed in
       `inertiaProblems`.
    4. *The result is stored* with the Robot (HDF5), so the estimate is made once and can be read and corrected.

### 3.4 Decisions of 2026-11-08, second round (maintainer)

16. **ManipulatorStaubliTX2_90L**: the manufacturer gives no inertias, only the total mass of 117 kg, taken as including
    the base. The inertias are those of the solids of the visual meshes with one density, scaled to this mass:
    2032 kg/m^3 (done, #3358).
17. **The deprecated joint torque loads**: serialRobotTest.py and movingGroundRobotTest.py are on `SetControl` now
    (RG19.15.1); the parameters are removed when the deprecation period ends (RG19.15.2, #3361).
18. **The friction of a URDF** is read by `RobotFromURDFdata` because the Robotics Toolbox does not read it; the docstring
    says so, as the toolbox may change.

### 3.5 Proposals for the maintainer (2026-11-08)

**(a) Leaving out links: the fingers of the Panda.** The import has no option for it; the URDF of the Panda gives 9
joints with its two fingers, models.py gives 7 and no fingers. Recommended: one function on the data of
GetURDFrobotData, without the toolbox and so testable in the suite,

```python
robotData = ReduceURDFrobotData(robotData, lockedJoints={'panda_finger_joint1':0.04, 'panda_finger_joint2':0.04},
                                removedLinks=None)
```

- `lockedJoints` (the common case): the joint is fixed at the given value, the link is merged into its parent: the mass,
  the center of mass and the inertia added by the parallel axis theorem, the graphics moved, its children reparented. The
  gripper keeps its mass in the arm.
- `removedLinks`: a link and all that hangs on it are left out, mass and graphics as well. "Only the first n links" is
  `removedLinks=[name of link n+1]`, a list of link names rather than a number, since a tree has no first n.
- The tool: unchanged, the frame of `toolLinkName` relative to the last joint link that remains; without a name
  it is the frame of the last link (`toolHT` of `RobotFromURDFdata` can define it separately).
- In models.py the same names: `ManipulatorFrankaEmikaPanda(gripper=None)` as now; see (b).

**(b) The models and the URDF files as one definition.** Today the two differ: models.py has DH parameters for the
Panda, UR5, Puma560, and the frames of the URDF for the TX2-90L, and other inertias. Proposed:

1. *The role of models.py*: for each robot the definition that **runs without the toolbox and without files**: the
   parameters of the URDF (kinematics, masses, inertias, limits, in the joint frames of the URDF: `dhMode='jointFrames'`)
   and what a URDF does not give: friction, PD values, named configurations, maximum velocities where they are not in the
   URDF, the source of estimated values, and simple collision shapes (spheres, capsules, boxes in the frames of the links)
   instead of STL files. A DH table stays as an alternative where teaching needs it.
2. *Generated once, kept by a test*: a function `urdfImport.RobotModelDictFromData(robotData)` writes the part from the
   URDF as Python text to paste into the model; the additional parameters are added by hand in the same dict.
3. *The workflow per robot*, the same each time:
   URDF into CommonData_git, with its source in the README → `GetURDFrobotData` and `_InertiaProblems` → the inertias
   by the workflow of decision 15 if they are missing, adjusted to the total mass → the dict for models.py by
   `RobotModelDictFromData` → simple collision shapes fitted to the collision meshes (a capsule per link, by the
   convex hull) → the additional parameters by hand.
4. *The check*: a test `test_robotModelsAgainstURDF.py` for each model with `'urdfGraphics'`: forward kinematics at
   random joints, masses, centers of mass, inertias, limits equal to the URDF read directly; skipped if CommonData_git is
   not there. So the identity of model and URDF is measured instead of promised. The reading of the URDF for this check
   needs no toolbox for a serial robot (`LoadURDFgraphics` reads the file already; kinematics from the `<joint>` elements).
5. *The robots in models.py*, all running without shapes: the Puma560, UR5, Panda, TX2-90L now; proposed in addition:
   **Spot** and **ANYmal** (floating base, RG19.14), the **Mars rover** (mobile base), **KUKA LBR iiwa** (a 7-axis arm in
   the local data set), **Kinova Gen3**, **UR10**. A tree or a floating base needs `parent` in the link dict, which
   `LinkDict2Robot` has to pass on.

Decided 2026-11-08 (maintainer): (a) and (b) as above, with Spot, ANYmal, the Mars rover, LBR iiwa and UR10; collision shapes for Spot, UR5, Panda and TX2-90L; steps RG19.17.1 to RG19.17.4 (#3372 to #3375).

## 4. Done (2026-11-04 to 2026-11-06)

| step | issue | what |
|---|---|---|
| RG19.1.1 | - | the analysis of the module and its users, the recommendations and questions |
| RG19.1.2 | #3280 | the URDF import ported to the RTB 1.5; fixed links merged into the joint link above; equal to the RTB (1e-16) for its models and the local data set; UR, Panda, Kinova checked with #3285 |
| RG19.1.3, RG19.2.1 | #3254 | the Python kinematics measured (the static compensation in a PreStep 15 times the step); the commented-out code removed |
| RG19.2.2 | #3255 | the tool frame of the IK and the tree, InterpolateHTs, the Panda of models.py, the Puma560 inertias |
| RG19.2.3, RG19.2.4 | #3256, #3268 | urdfImport.py; Robot.BuildFromDictionary removed |
| RG19.3.1 to RG19.3.7 | #3263, #3264, #3267, #3273, #3275, #3282, #3285 | URDF meshes without the RTB; one STL reader with mass properties; Robot as dict and HDF5 with textures and a format version; STL orientation; flipped joints, inertia check and correction, masses from meshes, textures; the test robot in the repository |
| RG19.4.1, RG19.4.2 | #3270, #3276 | q0, friction and end stops in both models, PreStepControl, AddSensors; the inverse dynamics and the dynamic compensation |
| RG19.5.1 to RG19.5.4 | #3262, #3272, #3274, #3283 | ProfilePolynomial5, Cartesian linear, circular, screw and spline paths, keepConfiguration, the jumps in Trajectory; blending, 'auto', the velocity check, Cartesian blending |
| RG19.6.1 to RG19.6.4 | #3259, #3260, #3265, #3269 | cUtilities.KinematicChain (1 to 2 us); the analytic IK (3R, spherical wrist, UR) and the arm configuration; the LM IK; future.py removed |
| RG19.7.1 | #3277, #3279 | the configurations, initial configuration and friction of models.py; the Puma560 limits of the RTB; Manipulator4Rsimple removed |
| RG19.8.1, RG19.8.2 | #3279, #3284 | the serial robot examples on the revised interfaces; the examples runner gives `__file__` |
| RG19.9 | #3281 | the chapter Robotics of the user manual and tutorialRobotics.ipynb |
| RG11.8.20 | #2993 | closed as superseded by #3269 (see task 9) |
| RG19.10.1 | #3352 | the URDF of the TX2-90, UR5 and Panda in CommonData_git, found by the examples |
| RG19.11.1 | #3354 | the Spot examples and their data in Examples/robotics |
| RG19.12.1 | #3356 | RobotFromURDFdata, Robot.SaveToHDF5 and LoadFromHDF5; the friction read from the URDF file, as the toolbox does not |
