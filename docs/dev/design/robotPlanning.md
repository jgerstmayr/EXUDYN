# Path planning and navigation of robots (revision2026c group RG21)

A design note of open steps: the request, the evaluation and the design, decided by the maintainer on 2026-11-07
(section 5). The note leaves the repository when the steps close (WORKFLOW.md §8.4).

## 1. The request (maintainer 2026-11-06)

- Path and trajectory planning and navigation, for teaching and in Python functions, using the efficient C++
  kinematics (exudyn.cUtilities.KinematicChain); in a new file of exudyn/robotics with a name that can also hold SLAM
  later.
- No MoveIt: too complicated and hard to install for students.
- Standard methods for mobile robots (mecanum-wheeled or differential drive) and for serial robots. No SLAM now: the
  pose of a mobile robot and the obstacles are known.
- Collision shapes: only the basic contact between two primitives - sphere, cylinder, triangle (and convex shapes of
  triangles) - from the contact primitives of RG9.12 (design note contactPrimitives.md), until the revised contact
  module is available. The contact items get an output of the minimum distance; for planning they are deactivated or
  not in the simulation at all, so they cost nothing there.
- Collision shapes may come from the URDF (an extension of the import); the joint limits are respected.
- Input: a robot of class Robot; the C++ kinematics may be used and extended.
- 1 to 3 robust but not overly difficult algorithms for each kind of robot, including obstacles of the environment;
  a grid-based method and an RRT variant (in particular for the serial robot) could do.
- The result applied to the robot as a trajectory, with the present Trajectory functionality, extended where needed.

## 2. What exists

| piece | where | use for planning |
|---|---|---|
| Robot, RobotLink (links, joint limits as end stops, tool, base) | robotics/roboticsCore.py | the input; joint limits |
| KinematicChain: LinkHT, JointHT, ToolHT, Jacobian, IK analytic and LM, static torques, inverse dynamics (1 to 2 us) | exudyn.cUtilities (C++) | the forward kinematics of every collision check |
| Trajectory: PTP, polynomial and Cartesian profiles, blending with 'auto' blend times and the velocity check | robotics/motion.py | the planned path as a trajectory |
| Robot.SetControl(trajectory), PreStepControl | roboticsCore.py | running the result in a simulation |
| MobileKinematics, MecanumXYphi2WheelVelocities, MobileRobot2MBS | robotics/mobile.py | the wheel velocities of a planned base motion |
| ObjectContactSphereSphere, ObjectContactSphereTriangle, GeneralContact | C++ items | not suited for planning: in an mbs, forces only |
| URDF import: links, joints, inertia, visual meshes | robotics/urdfImport.py | `<collision>` elements are not read yet |

## 3. Evaluation

### 3.1 The name of the module

`exudyn/robotics/navigation.py` (proposed). In the robotics literature (e.g. Corke, *Robotics, Vision and Control*)
navigation covers maps, path planning in the plane and in configuration space, and localization and mapping (SLAM);
the planning of a serial robot is planning in its configuration space, which the same module holds. Alternatives:
`planning.py` (clear for the planners, odd for SLAM later), `motionPlanning.py` (the term of manipulators, odd for
SLAM). The collision shapes belong to the links and stay next to the classes RobotLink and Robot in roboticsCore.py
(section 4.1, decision 1), so that a later SLAM in navigation.py does not mix with the geometry.

### 3.2 The algorithms

| algorithm | robot | for | against | proposed |
|---|---|---|---|---|
| occupancy grid + A* (8-connected), obstacles inflated by the robot radius | mobile | simple, complete on the grid, optimal on the grid, ideal for teaching; 2D grids of 100 x 100 to 500 x 500 plan in milliseconds with numpy | resolution-dependent paths with grid corners; non-holonomic constraints ignored | **yes**, with a smoothing by line of sight |
| distance transform (wavefront) | mobile | one computation for all start points, gives the distance to obstacles | same grid limits | as a helper of A* (the inflation and a clearance cost), not as a planner of its own |
| RRT-Connect (bidirectional RRT) in joint space | serial (and mobile in SE(2)) | robust in 6 to 7 dimensions, few parameters (step size, goal bias, iterations), no grid; standard in OMPL and MoveIt | random, not optimal: needs a shortcut smoothing; probabilistically complete only | **yes**, with shortcut smoothing |
| PRM (probabilistic roadmap) | serial | many queries in one static environment | the roadmap costs more than one RRT query | optional, later |
| RRT* | both | asymptotically optimal | slow convergence, more parameters | no; shortcut smoothing gives most of its benefit |
| Hybrid A* / Reeds-Shepp | differential drive (car-like) | respects non-holonomic constraints | complex for teaching | no; a differential drive turns on the spot, so a grid path followed by a pure-pursuit controller suffices |
| potential fields | both | trivial | local minima | no (a teaching example at most) |

Proposed: **grid A\*** for mobile robots, **RRT-Connect** for serial robots, both with a smoothing; the same RRT
code also plans in SE(2) for a mobile robot in narrow spaces where the grid is too coarse (a second algorithm for
mobile robots without extra code).

### 3.3 Collision checking

A planner calls the check 1e4 to 1e5 times per query (RRT) or once per grid cell (A\*):

- a serial robot: per sample the link HTs by KinematicChain (1 us), per pair of a link shape and an obstacle shape a
  distance (primitive kernels, 0.1 to 1 us in C++), pairs filtered by bounding spheres; with 7 links of 1 to 3 shapes
  and 20 obstacle shapes some 100 pairs, 10 to 50 us in C++, about 1 ms in Python per sample - **the check belongs in
  C++**, as one call `CollisionModel.MinimumDistance(q)` or `IsFree(q)`; no Python version of the check
  (decision 3);
- an edge between two samples by checking points at a maximum joint step (e.g. 0.02 rad) or, better, by the
  distance: a step is free if the distance exceeds the largest motion of any link point (a conservative bound from the
  Jacobian norms), which needs fewer checks;
- a mobile robot: the grid is computed once from the obstacles projected to the ground plane (cells whose distance to
  an obstacle is below the robot radius plus a margin); A\* itself does no geometry.

The distance functions are those of the contact primitives (contactPrimitives.md): sphere, cylinder (capsule as a
variant), triangle and convex shape of points, with GJK for the distance and EPA for a penetration. They live as free
functions in C++ (`src/Linalg/Geometry*.h`), used by the contact items and exported in exudyn.cUtilities for the
planner, so the items and the planner share one implementation; the item output `Distance` serves the simulation
(checking a planned motion, monitoring).

## 4. Proposed design

### 4.1 The collision shapes in robotics/roboticsCore.py

- `CollisionShape`: a primitive in a local frame - `Sphere(radius)`, `Capsule(radius, length)`, `Cylinder(radius,
  length)` (axis along local z, centered), `Box(size)`, `Convex(points)` (e.g. a convex link mesh), `Triangles(points,
  triangles)` (an environment surface, as triangles); with the local HT. The shapes of a link are a list
  `collisionShapes` of RobotLink, the shapes of the base one of Robot, next to the graphics data of both.
- `CollisionModel(robot=None)`: the shapes of each link of the robot (`AddLinkShape(linkIndex, shape, HT)`), of the
  base, and of the environment (`AddObstacle(shape, HT)`); pairs of adjacent links and pairs given by the user are
  excluded; `MinimumDistance(q)`, `IsFree(q, margin=0)`, `IsEdgeFree(q0, q1, margin)`; built in C++ from these lists.
- the `<collision>` elements of a URDF - all its geometries: box, cylinder, sphere, mesh (as its convex hull, or as
  triangles), and the capsule of the URDF extensions where it is used - per link, read by urdfImport.GetURDFrobotData
  under a new key `'collision'` and passed into the `collisionShapes` of the links.
- `ShapesFromLinks(robot, radius)`: capsules along the links of a Robot for a robot without collision data.
- `AddToMbs(mbs, ...)`: the shapes as contact items with `activeConnector=False` and their Distance output, and as
  graphics, to see and check the model.

### 4.2 robotics/navigation.py

Serial robots:
- `PlanJointPath(collisionModel, qStart, qGoal, jointLimits=None, maxStep=0.05, maxIterations=20000, goalBias=0.1,
  seed=None)` - RRT-Connect, returns the joint waypoints or raises with the number of samples tried;
- `ShortcutPath(collisionModel, path, iterations=200)` - random shortcuts and pruning of collinear points;
- `PlanToPose(collisionModel, qStart, HTgoal, ...)` - the goal configuration by the IK (analytic with the configuration
  of the start, else LM), then PlanJointPath.

Mobile robots:
- `OccupancyGrid(xRange, yRange, cellSize)` with `AddObstacles(shapes, height range)`, `Inflate(radius)`,
  `DistanceTransform()`; plotted with matplotlib for teaching;
- `PlanGridPath(grid, start, goal, connectivity=8, clearanceWeight=0)` - A\* on the grid, a smoothing by line of sight;
- `PlanSE2Path(...)` - RRT-Connect in (x, y, phi) with the robot shapes, for narrow spaces.

### 4.3 The result as a trajectory

- serial: `Trajectory.FromWaypoints(qList, maxVelocities, maxAccelerations, blend='auto')` (new): PTP segments of the
  present ProfilePTP through the waypoints, blended with the existing 'auto' blend times and the velocity check, so
  the robot does not stop at each waypoint; then `robot.SetControl(trajectory)` as now. A planned path re-checked
  along the blended trajectory (blending cuts corners), with the step refined where the check fails.
- mobile: the same profiles on the coordinates (x, y, phi) of the base: a Trajectory of three coordinates;
  - mecanum (holonomic): the base velocities of the trajectory to wheel velocities by
    MecanumXYphi2WheelVelocities, as a PreStepUserFunction;
  - differential drive: phi along the path tangent, and a pure-pursuit tracking controller
    (`PurePursuit(path, lookAhead)`) giving v and omega, then the wheel velocities; the tracking error is a teaching
    subject in itself.

### 4.4 C++

- the distance kernels (shared with RG9.12) and a class `CollisionModel` in exudyn.cUtilities (a KinematicChain plus
  shapes per link and obstacles; MinimumDistance, IsFree, IsEdgeFree), the only implementation of the check;
- optionally the RRT loop in C++ later, if Python is the bottleneck (it should not be: the check dominates).

## 5. Decisions (maintainer 2026-11-07)

The design as proposed, kept open for extensions (further planners, SLAM, the contact module replacing the
primitives):

1. The module `navigation.py`; the collision shapes next to RobotLink and Robot in roboticsCore.py, no `collision.py`.
2. Grid A\* and RRT-Connect with smoothing, RRT in SE(2) for mobile robots; other planners later.
3. The collision check in C++ (`cUtilities.CollisionModel`), without a Python version.
4. The shapes sphere, capsule, cylinder, box, convex points and triangles; the common collision geometries of the
   URDF supported.
5. `<collision>` from the URDF under the new key `'collision'` of GetURDFrobotData.
6. Trajectory.FromWaypoints with blending; the pure-pursuit controller for differential drives.
7. RG9.12 first (the distance kernels), then RG21.
8. Examples: the UR5 among obstacles, a mecanum-wheeled robot and a differential-drive robot (a vacuum cleaner), the
   two mobile robots built in the example with a cuboid and a flat cylindrical body and graphics shapes of their
   bodies.

## 6. Proposed steps (RG21)

| step | what |
|---|---|
| RG21.1 | this evaluation and design, decided by the maintainer (2026-11-07) |
| RG21.2 | the collision shapes of RobotLink and Robot and cUtilities.CollisionModel on the distance kernels of RG9.12; ShapesFromLinks; tests of distances against analytic cases |
| RG21.3 | `<collision>` from the URDF; the test robot of #3285 with collision shapes |
| RG21.4 | RRT-Connect and shortcut smoothing for serial robots, PlanToPose; Trajectory.FromWaypoints; a test with the UR5 or Panda among boxes and cylinders, run within 0.25 s |
| RG21.5 | occupancy grid, A\*, smoothing, RRT in SE(2); the mecanum and differential-drive execution with pure pursuit; tests on a fixed map |
| RG21.6 | examples: the UR5 among obstacles, a mecanum-wheeled robot with a cuboid body and a differential-drive robot like a vacuum cleaner through a room, with graphics of their bodies; a section of the robotics chapter, a notebook for teaching |
