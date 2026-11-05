(sec-robotics)=
# Robotics

`exudyn.robotics` builds robots as multibody systems: serial and tree-like manipulators with their kinematics,
inverse kinematics and dynamics, trajectories, joint control and sensors, robots read from URDF files, mobile robots
with mecanum wheels, and an interface to ROS. A robot is described once, as a `Robot`, and then computed in two ways:
without a model, by the kinematic chain in C++, for planning and control; and as a model of Exudyn, which simulates its
dynamics together with everything else in the system - flexible links, contact, a moving base, a workpiece.

```{figure} /docs/figures/robotStaubliTX2.png
:width: 600px

The Stäubli TX2-90L of `exudyn.robotics.models` with the meshes of its URDF, a gripper and the paths of its tool,
without and with blending at the corners; raytraced, from the example staubliTX2blending.py.
```

## The modules

| module | what it has |
|---|---|
| `exudyn.robotics` (roboticsCore) | `Robot` with `RobotLink`, `RobotBase`, `RobotTool`: the description of a robot, its models, its control, its sensors, saving and loading; `StdDH2HT`, `ModDHKK2HT` |
| `exudyn.robotics.models` | the Puma560, UR5, Franka Emika Panda, Stäubli TX2-90L and a 3R manipulator as dictionaries, with masses, inertias, joint limits, named configurations and, where known, friction; `LinkDict2Robot` |
| `exudyn.robotics.motion` | `Trajectory` and its profiles: point-to-point motions in joint space (constant acceleration, synchronized with limits, polynomials), and paths of the tool (linear, circular, screw, splines), blended |
| `exudyn.robotics.urdfImport` | robots and their meshes from URDF files, through the Robotics Toolbox for Python, and the meshes alone without it |
| `exudyn.cUtilities.KinematicChain` | the kinematics of a robot in C++: forward kinematics, Jacobian, static torques, inverse dynamics, analytic and numerical inverse kinematics |
| `exudyn.robotics.special` | manipulability measures: velocity, force, stiffness and dynamic manipulability |
| `exudyn.robotics.mobile` | mobile robots on mecanum wheels and their kinematics |
| `exudyn.robotics.utilities` | `AddLidar`: a lidar of rays measured by general contact |
| `exudyn.robotics.rosInterface` | the communication with ROS 1, see {ref}`sec-robotics-ros` |

The functions are listed in the reference manual under the Python utility functions.

## A robot

A `Robot` is a list of links. Each link has a joint - a rotation or translation about the x, y or z axis of its frame
(`'Rz'`, `'Px'`, ...) - and the frame of the joint relative to the link before (`preHT`, or `localHT` as the
Denavit-Hartenberg parameters give it), its mass, center of mass and inertia, the P and D values of the joint control,
optionally friction, joint limits and end stops, and graphics. The base and the tool are frames with graphics, and
gravity acts on all links. A robot is built in one of three ways:

- from a model: `LinkDict2Robot(ManipulatorPuma560())`, with `initialConfiguration` (`'qr'`, `'qz'`, ... or the
  model's own) and `friction=True`;
- link by link, `robot.AddLink(RobotLink(mass, COM, inertia, localHT=StdDH2HT([theta, d, a, alpha]), ...))`, from
  standard or modified Denavit-Hartenberg parameters or from any joint frames; a link may have another parent than the
  link before, for robots with branches;
- from a URDF file, see {ref}`sec-robotics-urdf`.

`robot.GetDictionary()` and `SetDictionary` store a robot with its graphics and textures as a dict, which
`exudyn.advancedUtilities.SaveDictToHDF5` writes to a file; a robot read from a URDF is then loaded without the
packages that read it.

## Kinematics and dynamics without a model

`robot.GetKinematicChain()` gives a `KinematicChain` of `exudyn.cUtilities`, the same data in C++. It computes in a
few microseconds the tool frame (`ToolHT(q)`), the frames of all links, the Jacobian of the tool or of a point of a
link, the joint torques that hold the robot against gravity (`StaticTorques`) and those of the inverse dynamics
(`InverseDynamics(q, q_t, q_tt)`, recursive Newton-Euler). The inverse kinematics is analytic for three geometries -
3 links with two parallel axes, 6 links with a spherical wrist (Puma560, KUKA, Stäubli), 6 links of the UR robots -
with all solutions, sorted by the distance from a reference or selected by the arm configuration (shoulder, elbow,
wrist), and numerical by Levenberg-Marquardt for any serial robot. `Robot` has the same functions in Python, with the
same algorithm, for checking and teaching.

## Models in Exudyn

- `robot.CreateKinematicTree(mbs)` builds the robot in minimal coordinates, one `ObjectKinematicTree` with the joint
  coordinates as its coordinates: the fast model, for the robot alone or on a moving base.
- `robot.CreateRedundantCoordinateMBS(mbs, baseMarker)` builds rigid bodies connected by joints, with a spring-damper
  per joint for the control: the model to which other parts are connected anywhere, such as flexible links, or
  bodies in contact.

Both have the PD control of the joints, the friction and the end stops of the links (`Robot(enableEndStops=True)`,
`SetJointFriction`, `SetEndStops`), and they compute the same motion. The robot starts in `robot.q0`.

## Trajectories, control and sensors

A `Trajectory` starts at joint coordinates and consists of profiles, each ending at rest:
`ProfileConstantAcceleration` and `ProfilePTP` (point-to-point in joint space, synchronized or with limits of
velocity and acceleration), `ProfilePolynomial5`, and the paths of the tool `ProfileCartesianLinear`,
`ProfileCartesianCircular`, `ProfileCartesianScrew` and `ProfileCartesianSpline`, which keep the arm configuration of
their start and raise where the joint coordinates would jump. `Trajectory.Add(profile, blendTimeWithNext)` lets the
next profile start earlier; the trajectory is then the sum of both motions and passes the via point at a distance that
`GetBlendings` reports.

`robot.SetControl(trajectory, compensation)` and `mbs.SetPreStepUserFunction(robot.PreStepControl)` let the joint
control follow the trajectory, in either model; `compensation='static'` adds the torques against gravity and
`'dynamic'` those of the inverse dynamics. Several robots, or a trajectory repeated in cycles, call
`robot.PreStepControl(mbs, t)` from one PreStepUserFunction of the script. `robot.AddSensors(mbs, [...])` measures the
joint coordinates, velocities and torques, stored for `mbs.PlotSensor`.

A typical script, as the notebook below:

```python
robot = LinkDict2Robot(ManipulatorPuma560(), initialConfiguration='qn')
trajectory = Trajectory(initialCoordinates=robot.q0)
trajectory.Add(ProfileConstantAcceleration(qGoal, 0.8))
robot.CreateKinematicTree(mbs)
robot.SetControl(trajectory, compensation='dynamic')
mbs.SetPreStepUserFunction(robot.PreStepControl)
sensors = robot.AddSensors(mbs, [exu.OutputVariableType.Coordinates])
mbs.Assemble()
mbs.SolveDynamic(simulationSettings)
```

```{toctree}
:maxdepth: 1

/docs/generated/notebooks/tutorialRobotics
```

(sec-robotics-urdf)=
## Robots from URDF files

`urdfImport.LoadURDFgraphics` and `AddURDFgraphics` read the meshes of a URDF file (STL) as the graphics of a
`Robot` without further packages, as the TX2-90L of `models.py` does. A whole robot - kinematics, masses and meshes -
is read through the Robotics Toolbox for Python 1.5 (`pip install roboticstoolbox-python`, and pymeshlab for the
meshes): `GetRoboticsToolboxInternalModel('Puma560')` for the URDF models of the toolbox, `LoadURDFrobot(file,
packagePath)` for a URDF or xacro file of a ROS package, and `GetURDFrobotData` for the links as `Robot` needs them; the
links of fixed joints are added to the joint link above them. The import gives the kinematics of the toolbox for the
robots of its data and of the local robot data sets tried, except for flipped joints; many URDF files have no masses
or inertias, which a dynamic model needs. The example serialRobotURDF.py builds robots this way.

(sec-robotics-ros)=
## ROS

`exudyn.robotics.rosInterface` exchanges data with ROS 1 (noetic) by rospy: it publishes the state of a simulation
and receives commands, e.g. velocities for a mobile robot. The examples ROSMassPoint.py, ROSTurtle.py and
ROSMobileManipulator.py (with the ROS node in `python/Examples/testData/ROS`) show it. These features and examples are
currently untested: they need a ROS installation, which the tests of Exudyn do not have. Make sure to use Python 3, as
there is no Python 2 support for Exudyn.

## Examples

| example | what it shows |
|---|---|
| serialRobotKinematicTree.py, serialRobotTSD.py | the Puma560 in both models, with control, sensors and plots |
| staubliTX2trajectories.py, staubliTX2blending.py | the TX2-90L with its meshes: profiles of the joints and of the tool, the dynamic compensation, blending, saving the robot as HDF5 |
| serialRobotInverseKinematics.py, InverseKinematicsNumericalExample.py | the tool on a path by the inverse kinematics in every step; the numerical and analytic inverse kinematics |
| serialRobotInteractiveLimits.py | moving a robot by sliders and the mouse, with end stops |
| serialRobotFlexible.py | a robot on a flexible base of finite elements |
| serialRobotKinematicTreeDigging.py | a robot digging in particles |
| humanRobotInteraction.py | a robot touching an articulated dummy |
| serialRobotURDF.py | robots read from URDF files |
| mobileMecanumWheelRobotWithLidar.py | a mobile robot on mecanum wheels with a lidar |
