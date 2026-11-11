# Path planning and navigation of robots

How `exudyn.robotics.navigation` plans motions among known obstacles: the collision shapes of a `Robot`, the
collision check in C++, RRT-Connect in joint space, the trajectory along a path, and the occupancy grid, A\* and the
controllers of mobile robots (#3380, #3382, #3383, #3384, #3397). What a user does with them is the section
{ref}`sec-robotics-planning` of the manual and the notebook `tutorialPathPlanning`; this page is how they work inside.

## 1. The pieces

| piece | where | what |
|---|---|---|
| `ConvexShape`, `Distance` | `src/Pymodules/PyConvexShape.h`, on `src/Linalg/GeometryConvex.h` | the convex primitives and their signed distance, in `exudyn.cUtilities` |
| `CollisionModel` | `src/Pymodules/PyCollisionModel.h` | the check: a copied `KinematicChain`, shapes on links and base, obstacles; bound in `definitions/pybindCUtilities.py` |
| `RobotLink.collisionShapes`, `RobotBase.collisionShapes`, `Robot.GetCollisionModel` | `robotics/roboticsCore.py` | the shapes of a robot, and the model built from them |
| `GetURDFrobotData(collision=True)`, `CollisionShapesFromGeometry`, `FitCapsulesToMesh`, `FitCapsuleToPoints`, `ConvexHullOfMesh` | `robotics/urdfImport.py` | the shapes from the `<collision>` elements of a URDF |
| `PlanJointPath`, `ShortcutPath`, `PlanToPose`, `TrajectoryFromPath` | `robotics/navigation.py` | serial robots, in joint space |
| `OccupancyGrid`, `PlanGridPath`, `PlanSE2Path`, `PathWithHeadings`, `PurePursuit`, `DifferentialDriveWheelVelocities` | `robotics/navigation.py` | mobile robots, in the plane |
| `ProfileJointPath`, `Trajectory.FromWaypoints` | `robotics/motion.py` | a path as a `Trajectory` |

The planners are Python; the only geometry they call is the C++ check (section 3). `navigation.py` is named for
maps and planning in the plane and in configuration space, which leaves room for localization and mapping; the shapes
belong to the links and stay in roboticsCore.py. Its arithmetic uses `exudyn.cMath`, so a path is the same on every
platform for the same `seed` (#2962).

## 2. The collision shapes

A shape of a link is a tuple `(ConvexShape, HT)` in the link frame, or a dict that `_ConvexShapeAndHT`
(roboticsCore.py) turns into one: `'capsule'` with `pAxis`, `vAxis`, `radius` (the form of `graphics.Capsule` and of
`robotics.models`; the frame is built with its z axis along `vAxis`, centered at `pAxis + vAxis/2`), `'sphere'`,
`'box'`, `'cylinder'`, `'points'`, each with an optional `'HT'`. `ConvexShape` has the types Sphere, Capsule,
Cylinder, Box, Triangle, Points (their convex hull) and Cone (truncated); all of them work as link shapes and as
obstacles. A surface of many triangles is not a shape: a mesh enters as capsules or as its convex hull.

- **robotics.models**: the UR5 and the Panda with `useDH=False` and the TX2-90L carry one or two capsules per link
  (`'collisionShapes'`) and of the base (`'baseCollisionShapes'`), fitted to their collision meshes.
- **URDF**: `GetURDFrobotData(collision=True)` reads box, cylinder, sphere and mesh (a mesh needs pymeshlab) into the
  key `'collision'` of each joint link and `'collisionBase'`, in the frame of the joint link, including the links
  fixed to it; other geometries are skipped with a warning (the capsule of the URDF extensions is not read).
  `CollisionShapesFromGeometry(meshesAs='capsules')` puts all meshes of a link together into up to two capsules by
  `FitCapsulesToMesh`; `'convex'` keeps each mesh as the points of its convex hull (scipy).
- **Capsule fitting**: `FitCapsuleToPoints` tries the three principal axes, the smallest enclosing circle across each
  and the sphere centers that reach the extreme points, and keeps the smallest volume. `FitCapsulesToMesh` samples
  points on long triangles (a hull alone misses the middle of a straight link), assigns the points to the nearer
  capsule axis and fits again, and keeps two capsules only if the volume of their union is smaller than one capsule:
  relative volumes UR5 forearm 0.86, upper arm 0.89, Panda link 7 0.67, TX2 link 6 0.79, Spot lower leg 0.63; an
  L-shaped mesh in robotCollisionShapesTest.py gets two capsules of radius 0.076 instead of one of 0.43.

## 3. The collision check in C++

`CollisionModel(chain)` copies the `KinematicChain` and holds a list of items: shape, local HT, bounding radius and a
link index, `>= 0` a link, `-1` the base, `-2` an obstacle in the global frame. `Robot.GetCollisionModel(obstacles,
baseShapes, excludeCollidingAt, excludeMargin)` fills it from the shapes of section 2; it must be created again after
the robot changes.

- **Which pairs**: all pairs of items except those on the same link (or both fixed: base and obstacles), a link and
  its parent (so the first links and the base), and the link pairs in `excluded`. `ExcludeLinkPair(i, j)` adds one;
  `ExcludeCollidingPairs(q, margin)` adds every link or base pair whose shapes are nearer than `margin` at q - the
  capsules of neighboring links overlap by construction, at the UR5 the base and link 2 at any configuration. q must
  be a free configuration in which nothing else touches; the starting configuration of a task is not always one (the
  shelf scene of pathPlanningUR5.py is not solvable when excluded at its start).
- **One evaluation** (`Compute(q, stopBelow)`): the link frames by `ComputeFrames` of the chain, the frame of each
  item, then the pairs in order; a pair whose bound `|p_i - p_j| - r_i - r_j` (frame origins, bounding radii) is not
  below the current minimum is skipped, the others go to `EGeometry::ConvexDistance`; the loop stops as soon as the
  minimum is at or below `stopBelow`. Without a checked pair the result is the largest `Real`.
- `MinimumDistance(q)` runs it to the end; `IsFree(q, margin)` is `Compute(q, margin) > margin` and stops at the first
  pair within the margin; `IsEdgeFree(q0, q1, margin, maxStep)` checks the straight joint motion at
  `ceil(max|q1 - q0|/maxStep)` equal steps, both ends included. The edge is sampled, not bounded: between two samples a
  link may pass closer than the margin by the motion of one step, which the planners cover by a small extra margin
  (sections 4 and 5).
- **The distance** (`GeometryConvex.h`, #3317): GJK on the cores of the shapes - the sphere a point, the capsule a
  segment, the radius a margin subtracted afterwards - and EPA where the cores overlap, so the distance is signed;
  a cylinder, whose curved side GJK and EPA approach slowly, is refined by alternating projections, and a sphere
  against a cylinder has a closed formula. The contact items (`CObjectContactConvexConvex`,
  `...CylinderCylinder`, `...SphereCylinder`) call the same functions, so items and planner share one
  implementation.

The check exists in C++ only: a planner calls it 1e4 to 1e5 times per query, which would take about a millisecond
each in Python. Measured: one `MinimumDistance` of the UR5 with two obstacles 2.1 to 2.3 us (robotCollisionModelTest.py).

## 4. Serial robots: RRT-Connect and shortcuts

- **`PlanJointPath`** (RRT-Connect, Kuffner and LaValle 2000): the start and the goal must be within the limits and
  free, else a `ValueError`; a free straight edge between them is returned at once. Two trees (`_Tree`: a growing
  numpy array of points and a parent list) alternate: one grows towards a random point within the joint limits, or
  with probability `goalBias` towards the root of the other tree, by one step of at most `maxStep` in the largest
  joint (`_Steer`, the maximum norm); the nearest node is found by a linear scan in the Euclidean norm. If the edge is
  free, the other tree grows towards the new point step by step until an edge is blocked or it reaches the point,
  and then the path is the two branches joined. After `maxIterations` random points a `ValueError` gives the sizes of
  both trees.
- **`ShortcutPath`**: `iterations` pairs of random points anywhere on the path (by arc length, not only at
  waypoints); where the straight edge between them is free, the part between is replaced by it. Then each waypoint
  whose neighbors see each other is removed. Both use `margin + 1e-3`, as a shortcut otherwise grazes the obstacles
  between the samples of its check. Points inside the edges matter: with waypoints only, the base joint of the UR5
  went past its goal (1.56 rad for 0.8) and came back.
- **`PlanToPose`**: the candidates are all solutions of `InverseKinematicsAnalytic` or, for a robot without one
  (`ValueError`), the converged results of `InverseKinematicsNumerical` from qStart and `attempts - 1` random starts;
  those within the limits and free are sorted by their distance to qStart and the nearest is planned to.
- The loop stays in Python: the check dominates the time. Measured on the UR5 around a wall
  (robotPathPlanningTest.py, seed 2): 47 waypoints in 30 to 40 ms, 10 after `ShortcutPath`. A narrow passage is hard
  for a random planner: the window scene of pathPlanningUR5.py (0.36 x 0.35 m) fails for seed 2 within 20000 points.

## 5. The path as a trajectory

`TrajectoryFromPath` adds one `ProfileJointPath` (#3397) to a `Trajectory`:

- **Geometry**: straight edges between the waypoints, each corner rounded by a quadratic Bezier curve from
  `blendRadius/2` times the shorter adjacent edge before the waypoint to the same distance after it; the path
  parameter is scaled per piece so that `|dq/dp| = 1` at its ends: continuous joint velocities where line meets curve.
- **Timing**: a time-optimal parametrization on `numberOfPoints` points per piece: the largest `(dp/dt)^2` from the
  velocity limits and, by bisection, from `|q_pp x + q_p pDDot| <= aMax`; then a forward pass with the largest and a
  backward pass with the smallest constant acceleration per interval that keeps the limits at both of its ends, each a
  set of linear inequalities in the unknown end. A corner without rounding has velocity 0. The accelerations exceed
  the limits by about `1/numberOfPoints`: 10 % at 20 points, 4 % at 40, 1.5 % at 100 (the default, 0.3 s to compute),
  0.5 % at 200.
- **The check**: the rounded motion leaves the checked edges, so it is sampled at four times the density of
  `edgeStep` with `margin + 1e-3`; if it is not free, the rounding is halved (1, 0.5, 0.25, 0.125 of `blendRadius`)
  and finally 0, which is the path itself and free.

Measured on the three scenes of pathPlanningUR5.py (1.5 rad/s, 4 rad/s^2), `blendRadius=1`: wall 2.77 s, shelf
1.78 s, window 1.98 s, against 4.73, 2.19 and 2.54 s for blended point-to-point profiles, which stopped seven times at
the wall and exceeded the acceleration limit at the shelf; a uniform cubic B-spline with the waypoints as control
points was slower still at the same limits. `Trajectory.FromWaypoints` remains for coordinates without obstacles, as
the base of the mecanum robot: `ProfilePTP(syncAccTimes=True)` per edge, so that each edge is the straight line in
the coordinates, blended with `'auto'` times.

## 6. Mobile robots

- **`OccupancyGrid`**: a boolean array `occupied[i, j]` of square cells from `origin`. `AddCircle` and `AddRectangle`
  occupy the cells whose centers are inside. `AddObstacles(obstacles, zRange, numberOfLayers)` takes the obstacles of
  a collision model: cells within the bounding radius of an obstacle are probed by `cUtilities.Distance` to a sphere of
  radius 1e-6 at the cell center, at `numberOfLayers` heights.
- **`DistanceTransform`**: the exact Euclidean distance from each free cell center to the nearest occupied cell
  center, against the boundary cells only (occupied with a free neighbor, or at the edge of the grid), in numpy blocks
  of 512; the cost grows with free times boundary cells. `Inflate(radius)` is `DistanceTransform() <= radius`, after
  which the robot is a point. `IsSegmentFree` samples a line at a quarter cell; a line leaving the grid is not free.
- **`PlanGridPath`**: A\* on the cell centers, 4 or 8 neighbors, a diagonal step only if both cells beside it are
  free; costs 1 and sqrt(2) per step, times `1 + clearanceWeight/d` with d the distance of the cell to the obstacles
  (`clearanceWeight` is a length); the heuristic is the octile (8) or Manhattan (4) distance; a heap with a counter
  for ties and a closed set. Start and goal replace the first and last cell centers; `smooth=True` connects from each
  waypoint the farthest one in line of sight. Measured (robotNavigationTest.py, cells 0.05 m, robot radius 0.3 m,
  around a wall): 4 waypoints, length 13.47 against 13.28 of the geometric shortest path.
- **`PlanSE2Path`**: a `Robot` of three links `Px`, `Py`, `Rz` with the shapes of the robot on the last, and
  `PlanJointPath` on its collision model with the limits x, y, phi: the same RRT and the same C++ check, no code of
  its own.
- **Execution**: a mecanum robot follows a `Trajectory` of x, y, phi, whose velocities give the wheel velocities by
  `mobile.MobileKinematics`; a differential drive turns on the spot at each waypoint (`PathWithHeadings`, the
  shorter turn, phi continuous) or follows `PurePursuit` (Coulter 1992): the projection onto the current segment
  (segments advance, never go back), the point `lookAhead` further along the path, the arc of curvature
  `2 y/d^2` in the robot frame, `omega` limited by `maxTurnRate`, a turn on the spot if the point is behind, and
  `(0, 0)` within `goalTolerance` of the end. `DifferentialDriveWheelVelocities` gives the wheel velocities. Measured,
  integrated kinematically: largest deviation from the path 0.053 m at a corner (robotNavigationTest.py).

## 7. Tests and examples

| file | what |
|---|---|
| `python/TestModels/robotCollisionModelTest.py` | distances of a planar arm against a sphere and a box from the geometry, parent and excluded pairs, an edge, the time of one UR5 check |
| `python/TestModels/robotCollisionShapesTest.py` | `FitCapsulesToMesh` on an L-shaped mesh; sphere, box, cylinder and points on a link and on the base against known distances |
| `python/TestModels/robotPathPlanningTest.py` | the UR5 around a wall: RRT-Connect, shortcuts, the limits and continuity of the trajectory, `PlanToPose`, refused inputs; run directly, three robots show the paths |
| `python/TestModels/robotNavigationTest.py` | grid, distance transform, A\* length, clearance, `PlanSE2Path` through a door, pure pursuit |
| `python/Examples/robotics/pathPlanningUR5.py` | three scenes (wall, shelf, window) and a second UR5 picking from a pocket and placing on a column, obstacles of all shape types (#3408, #3416) |
| `python/Examples/robotics/mobileRobotsNavigation.py` | a mecanum robot and a differential drive through a room, as kinematic trees under PD without wheel contact; the distance map on the floor |
