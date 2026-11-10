#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library for robotics
#
# Details:  path planning and navigation of robots with known obstacles: RRT-Connect in joint space with shortcut
#           smoothing for serial robots, on the collision check in C++ exudyn.cUtilities.CollisionModel (#3383);
#           an occupancy grid with A* and RRT-Connect in the plane for mobile robots, pure pursuit for a differential drive (#3384)
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-09
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import heapq

import numpy as np
import exudyn
import exudyn.cMath as cMath #the math functions of the C++ code, the same bits on every platform (#2962)
from exudyn.robotics.motion import Trajectory, ProfileJointPath

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'JointLimits', 'PlanJointPath', 'ShortcutPath', 'PlanToPose', 'TrajectoryFromPath', 'OccupancyGrid', 'PlanGridPath', 'PlanSE2Path',
    'PathWithHeadings', 'PurePursuit', 'DifferentialDriveWheelVelocities',
    ]

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def JointLimits(robot, defaultLimits=(-np.pi, np.pi)):
    """The limits of the joints of a Robot as the planners need them.

    Args:
        robot: the Robot
        defaultLimits: the (lower, upper) limit of a joint which has none

    Returns:
        array of shape (n, 2), lower and upper limit per joint
    """
    return np.array([defaultLimits if link.jointLimits is None else link.jointLimits for link in robot.links], dtype=float)


def _Steer(qFrom, qTo, maxStep):
    """the point at the step maxStep (the largest change of a coordinate) from qFrom towards qTo; qTo if it is nearer"""
    delta = qTo - qFrom
    largest = np.max(np.abs(delta))
    return qTo if largest <= maxStep else qFrom + delta*maxStep/largest


class _Tree:
    """a tree of an RRT: the points in a growing array and the index of the parent of each"""
    def __init__(self, root):
        self.points = np.zeros((1024, len(root)))
        self.points[0] = root
        self.parents = [-1]

    def Nearest(self, q):
        delta = self.points[:len(self.parents)] - q
        return int(np.argmin(np.sum(delta*delta, axis=1)))

    def Add(self, q, parent):
        if len(self.parents) == len(self.points):
            self.points = np.vstack([self.points, np.zeros(self.points.shape)])
        self.points[len(self.parents)] = q
        self.parents += [parent]
        return len(self.parents) - 1

    def PathToRoot(self, index):
        path = []
        while index != -1:
            path += [self.points[index].copy()]
            index = self.parents[index]
        return path


def PlanJointPath(collisionModel, qStart, qGoal, jointLimits=None, margin=0., maxStep=0.1, edgeStep=0.02, maxIterations=20000, goalBias=0.1, seed=None):
    """A collision-free path in joint space by RRT-Connect (J. Kuffner, S. LaValle, 2000): two trees grow from the start and the goal, one to a random point and the other as far as it can to the new point of the first. The result is not optimal, see ShortcutPath.

    Args:
        collisionModel: the exudyn.cUtilities.CollisionModel of the robot and its environment
        qStart: the joint coordinates of the start
        qGoal: the joint coordinates of the goal
        jointLimits: None for -pi to pi, or an array of shape (n, 2), see JointLimits
        margin: the distance which the shapes keep
        maxStep: the largest change of a joint coordinate in an edge of the tree
        edgeStep: the step of the collision check along an edge
        maxIterations: the largest number of random points
        goalBias: the probability of growing towards the root of the other tree instead of a random point
        seed: the seed of the random numbers, for a repeatable result

    Returns:
        array of the waypoints, one per row, from qStart to qGoal
    """
    start = np.array(qStart, dtype=float)
    goal = np.array(qGoal, dtype=float)
    limits = np.array([(-np.pi, np.pi)]*len(start), dtype=float) if jointLimits is None else np.array(jointLimits, dtype=float)
    if limits.shape != (len(start), 2):
        raise ValueError('PlanJointPath: jointLimits must have the shape ' + str((len(start), 2)) + ', not ' + str(limits.shape))
    for (name, q) in [('qStart', start), ('qGoal', goal)]:
        if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
            raise ValueError('PlanJointPath: ' + name + ' is outside the joint limits')
        if not collisionModel.IsFree(q, margin):
            raise ValueError('PlanJointPath: ' + name + ' is in collision, the minimum distance is ' + str(collisionModel.MinimumDistance(q)))
    if collisionModel.IsEdgeFree(start, goal, margin, edgeStep):
        return np.array([start, goal])

    rng = np.random.default_rng(seed)
    trees = [_Tree(start), _Tree(goal)]
    for iteration in range(maxIterations):
        (tree, other) = (trees[iteration % 2], trees[(iteration + 1) % 2])
        target = other.points[0] if rng.random() < goalBias else rng.uniform(limits[:, 0], limits[:, 1])
        nearest = tree.Nearest(target)
        q = _Steer(tree.points[nearest], target, maxStep)
        if not collisionModel.IsEdgeFree(tree.points[nearest], q, margin, edgeStep):
            continue
        index = tree.Add(q, nearest)
        #the other tree grows to the new point as far as it can
        indexOther = other.Nearest(q)
        while True:
            qOther = _Steer(other.points[indexOther], q, maxStep)
            if not collisionModel.IsEdgeFree(other.points[indexOther], qOther, margin, edgeStep):
                break
            indexOther = other.Add(qOther, indexOther)
            if np.array_equal(qOther, q):
                (fromStart, fromGoal) = (trees[0], trees[1])
                (i0, i1) = (index, indexOther) if tree is fromStart else (indexOther, index)
                return np.array(fromStart.PathToRoot(i0)[::-1] + fromGoal.PathToRoot(i1)[1:])
    raise ValueError('PlanJointPath: no path found with ' + str(maxIterations) + ' random points, the trees have ' + str(len(trees[0].parents)) + ' and ' + str(len(trees[1].parents)) + ' points')


def ShortcutPath(collisionModel, path, margin=0., edgeStep=0.02, iterations=200, seed=None):
    """A shorter path: random points anywhere on the path are connected directly wherever the straight motion in joint space is free, then waypoints which are not needed are left out.

    Args:
        collisionModel: the exudyn.cUtilities.CollisionModel of the robot and its environment
        path: the waypoints, one per row, as returned by PlanJointPath
        margin: the distance which the shapes keep
        edgeStep: the step of the collision check along an edge
        iterations: the number of random pairs of points
        seed: the seed of the random numbers, for a repeatable result

    Returns:
        array of the waypoints, one per row, with the first and the last one of path
    """
    waypoints = [np.array(q, dtype=float) for q in path]
    rng = np.random.default_rng(seed)
    clearance = 1e-3 #a shortcut touches the obstacles between the points of its check otherwise

    def PointAt(length, positions):
        """the edge containing the arc length, and the point on it"""
        i = min(int(np.searchsorted(positions, length, side='right')) - 1, len(waypoints) - 2)
        fraction = (length - positions[i])/max(positions[i + 1] - positions[i], 1e-14)
        return (i, waypoints[i] + fraction*(waypoints[i + 1] - waypoints[i]))

    for iteration in range(iterations): #points inside the edges remove the corners that the waypoints of a tree have
        if len(waypoints) < 3:
            break
        positions = np.concatenate([[0.], np.cumsum([float(cMath.norm(b - a)) for (a, b) in zip(waypoints[:-1], waypoints[1:])])])
        (i, qA) = PointAt(rng.uniform(0., positions[-1]), positions)
        (j, qB) = PointAt(rng.uniform(0., positions[-1]), positions)
        if i > j:
            (i, j, qA, qB) = (j, i, qB, qA)
        if i != j and collisionModel.IsEdgeFree(qA, qB, margin + clearance, edgeStep):
            waypoints = waypoints[:i + 1] + [qA, qB] + waypoints[j + 1:]
    k = 1
    while k < len(waypoints) - 1: #each waypoint less is a segment less of the trajectory
        if collisionModel.IsEdgeFree(waypoints[k - 1], waypoints[k + 1], margin + clearance, edgeStep):
            del waypoints[k]
        else:
            k += 1
    return np.array(waypoints)


def PlanToPose(collisionModel, robot, qStart, HTgoal, jointLimits=None, margin=0., attempts=30, seed=None, **options):
    """A collision-free path in joint space to a pose of the tool: the free solutions of the inverse kinematics are tried, the nearest to qStart first; further keyword arguments go to PlanJointPath, e.g. maxStep.

    Args:
        collisionModel: the exudyn.cUtilities.CollisionModel of the robot and its environment
        robot: the Robot
        qStart: the joint coordinates of the start
        HTgoal: the frame of the tool at the goal, exu.HT or 4x4 matrix
        jointLimits: None for those of the robot, or an array of shape (n, 2), see JointLimits
        margin: the distance which the shapes keep
        attempts: the number of random starts of the numerical inverse kinematics, for robots without analytic solution
        seed: the seed of the random numbers, for a repeatable result

    Returns:
        array of the waypoints, one per row, from qStart to the goal
    """
    start = np.array(qStart, dtype=float)
    limits = JointLimits(robot) if jointLimits is None else np.array(jointLimits, dtype=float)
    def Usable(q):
        return np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1]) and collisionModel.IsFree(q, margin)

    chain = robot.GetKinematicChain()
    try:
        candidates = list(chain.InverseKinematicsAnalytic(HTgoal, start))
    except ValueError: #not one of the robots with an analytic solution
        candidates = []
    if len(candidates) == 0:
        rng = np.random.default_rng(seed)
        for attempt in range(attempts):
            qInitial = start if attempt == 0 else rng.uniform(limits[:, 0], limits[:, 1])
            (q, success, unused) = chain.InverseKinematicsNumerical(HTgoal, qInitial)
            if success:
                candidates += [np.array(q)]
    candidates = sorted([q for q in candidates if Usable(q)], key=lambda q: float(cMath.norm(q - start)))
    if len(candidates) == 0:
        raise ValueError('PlanToPose: the inverse kinematics has no solution which is free and within the joint limits')
    return PlanJointPath(collisionModel, start, candidates[0], limits, margin, seed=seed, **options)


def TrajectoryFromPath(collisionModel, path, maxVelocities, maxAccelerations, margin=0., edgeStep=0.02, blendRadius=1.):
    """A Trajectory along the waypoints of a path which does not stop at them, as fast as the limits of the joints allow: ProfileJointPath rounds the corners by curves, which are checked for collisions and made smaller where needed; without rounding the motion is the path, which is free.

    Args:
        collisionModel: the exudyn.cUtilities.CollisionModel of the robot and its environment
        path: the waypoints, one per row
        maxVelocities: the largest velocity per joint
        maxAccelerations: the largest acceleration per joint
        margin: the distance which the shapes keep
        edgeStep: the largest change of a joint coordinate between two checks
        blendRadius: the size of the rounding of the corners, relative to the shorter edge at a corner, see ProfileJointPath; it is halved until the trajectory is free, then 0

    Returns:
        the Trajectory, for Robot.SetControl or Trajectory.Evaluate
    """
    for factor in [1., 0.5, 0.25, 0.125, 0.]:
        trajectory = Trajectory(path[0], maxVelocities=maxVelocities, maxAccelerations=maxAccelerations)
        trajectory.Add(ProfileJointPath(path, blendRadius=blendRadius*factor))
        if factor == 0. or blendRadius == 0.:
            break
        times = trajectory.GetTimes()
        length = sum(float(cMath.norm(np.array(b, dtype=float) - np.array(a, dtype=float))) for (a, b) in zip(path[:-1], path[1:]))
        points = 4*int(np.ceil(length/edgeStep)) + 1 #finer than a path, as a curve is close to the obstacles at its corner
        if all(collisionModel.IsFree(trajectory.Evaluate(t)[0], margin + 1e-3) for t in np.linspace(times[0], times[-1], points)):
            break
    return trajectory


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class OccupancyGrid:
    """A map of the plane in square cells which are free or occupied; a mobile robot is planned as a point on the grid, after the obstacles are inflated by its radius.

    Attributes:
        occupied: boolean array, indices x and y

    Example:
        grid = OccupancyGrid([0, 10], [0, 8], 0.05)
        grid.AddRectangle([5, 4], [0.2, 6])
        grid.AddCircle([2, 2], 0.5)
        grid.Inflate(0.3)
        path = PlanGridPath(grid, [1, 1], [9, 7])
    """
    def __init__(self, xRange, yRange, cellSize):
        """create an empty grid

        Args:
            xRange: [xMin, xMax] of the map
            yRange: [yMin, yMax] of the map
            cellSize: the edge length of a cell
        """
        if not cellSize > 0:
            raise ValueError('OccupancyGrid: cellSize must be positive')
        self.origin = np.array([xRange[0], yRange[0]], dtype=float)
        self.cellSize = float(cellSize)
        self.occupied = np.zeros((int(np.ceil((xRange[1] - xRange[0])/cellSize)), int(np.ceil((yRange[1] - yRange[0])/cellSize))), dtype=bool)

    def _Centers(self):
        (nx, ny) = self.occupied.shape
        return np.meshgrid(self.origin[0] + (np.arange(nx) + 0.5)*self.cellSize, self.origin[1] + (np.arange(ny) + 0.5)*self.cellSize, indexing='ij')

    def CellOfPoint(self, point):
        """the indices (i, j) of the cell with the point; a ValueError if it is outside the grid"""
        (i, j) = np.floor((np.array(point[:2], dtype=float) - self.origin)/self.cellSize).astype(int)
        if not (0 <= i < self.occupied.shape[0] and 0 <= j < self.occupied.shape[1]):
            raise ValueError('OccupancyGrid: the point ' + str(list(point)) + ' is outside the grid')
        return (int(i), int(j))

    def CenterOfCell(self, cell):
        """the position of the center of the cell (i, j)"""
        return self.origin + (np.array(cell, dtype=float) + 0.5)*self.cellSize

    def AddCircle(self, center, radius):
        """mark the cells whose centers are in a circle as occupied"""
        (x, y) = self._Centers()
        self.occupied |= (x - center[0])**2 + (y - center[1])**2 <= radius*radius

    def AddRectangle(self, center, size, angle=0.):
        """mark the cells whose centers are in a rectangle as occupied

        Args:
            center: the center of the rectangle
            size: its two edge lengths
            angle: its rotation about the center
        """
        (x, y) = self._Centers()
        (dx, dy) = (x - center[0], y - center[1])
        (c, s) = (cMath.cos(angle), cMath.sin(angle))
        self.occupied |= (np.abs(c*dx + s*dy) <= 0.5*size[0]) & (np.abs(-s*dx + c*dy) <= 0.5*size[1])

    def AddObstacles(self, obstacles, zRange=(0., 1.), numberOfLayers=5):
        """mark the cells in which a shape is within a range of heights as occupied

        Args:
            obstacles: a list of tuples (exudyn.cUtilities.ConvexShape, HT), as the obstacles of a CollisionModel
            zRange: the range of the heights of the robot
            numberOfLayers: the number of heights which are checked in zRange
        """
        point = exudyn.cUtilities.ConvexShape('Sphere', radius=1e-6)
        heights = np.linspace(zRange[0], zRange[1], numberOfLayers)
        (x, y) = self._Centers()
        for (shape, HT) in obstacles:
            HTobstacle = exudyn.HT(HT)
            center = np.array(HTobstacle.translation)
            near = (x - center[0])**2 + (y - center[1])**2 <= shape.BoundingRadius()**2 + 2*self.cellSize**2
            for (i, j) in zip(*np.nonzero(near & ~self.occupied)):
                for z in heights:
                    if exudyn.cUtilities.Distance(shape, HTobstacle, point, exudyn.HT(translation=[x[i, j], y[i, j], z]))[0] <= 0.:
                        self.occupied[i, j] = True
                        break

    def DistanceTransform(self):
        """the distance of the center of each cell to the center of the nearest occupied cell

        Returns:
            array of the shape of occupied; zero for occupied cells, inf if the grid has no occupied cell
        """
        if not np.any(self.occupied):
            return np.full(self.occupied.shape, np.inf)
        o = self.occupied
        interior = o.copy() #only the occupied cells with a free neighbor can be the nearest
        interior[1:-1, 1:-1] = o[:-2, 1:-1] & o[2:, 1:-1] & o[1:-1, :-2] & o[1:-1, 2:] & o[1:-1, 1:-1]
        interior[0, :] = interior[-1, :] = False
        interior[:, 0] = interior[:, -1] = False
        (bi, bj) = np.nonzero(o & ~interior)
        (i, j) = np.nonzero(~o)
        result = np.zeros(o.shape)
        best = np.full(len(i), np.inf)
        for k in range(0, len(bi), 512):
            di = i[:, None] - bi[k:k + 512][None, :]
            dj = j[:, None] - bj[k:k + 512][None, :]
            best = np.minimum(best, np.min(di*di + dj*dj, axis=1))
        result[i, j] = np.sqrt(best)*self.cellSize
        return result

    def Inflate(self, radius):
        """occupy all cells within the distance radius of an occupied cell: a robot of this radius is then planned as a point"""
        self.occupied = self.DistanceTransform() <= radius

    def IsSegmentFree(self, p0, p1):
        """True if the straight line between two points only passes free cells inside the grid"""
        (p0, p1) = (np.array(p0[:2], dtype=float), np.array(p1[:2], dtype=float))
        steps = int(np.ceil(cMath.norm(p1 - p0)/(0.25*self.cellSize))) + 1
        cells = np.floor((p0 + np.outer(np.linspace(0., 1., steps), p1 - p0) - self.origin)/self.cellSize).astype(int)
        if np.any(cells < 0) or np.any(cells >= np.array(self.occupied.shape)):
            return False
        return not np.any(self.occupied[cells[:, 0], cells[:, 1]])


def _SmoothByLineOfSight(grid, path):
    """drop the waypoints which can be left out as the straight line is free"""
    result = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not grid.IsSegmentFree(path[i], path[j]):
            j -= 1
        result += [path[j]]
        i = j
    return np.array(result)


def PlanGridPath(grid, start, goal, connectivity=8, clearanceWeight=0., smooth=True):
    """The shortest path on the cells of an occupancy grid by A*. Inflate the grid by the radius of the robot before.

    Args:
        grid: the OccupancyGrid
        start: the position [x, y] of the start
        goal: the position [x, y] of the goal
        connectivity: 4 or 8, the neighbors of a cell; with 8, a diagonal step needs both neighbor cells beside it free
        clearanceWeight: if > 0, a cell at distance d of an obstacle costs more by clearanceWeight/d per cell, which keeps the path away from the obstacles
        smooth: if True, waypoints are left out where the straight line between the others is free

    Returns:
        array of the waypoints [x, y], one per row, from start to goal
    """
    if connectivity not in [4, 8]:
        raise ValueError('PlanGridPath: connectivity must be 4 or 8, not ' + str(connectivity))
    (cellStart, cellGoal) = (grid.CellOfPoint(start), grid.CellOfPoint(goal))
    for (name, cell) in [('start', cellStart), ('goal', cellGoal)]:
        if grid.occupied[cell]:
            raise ValueError('PlanGridPath: the ' + name + ' is in an occupied cell')
    clearance = None if clearanceWeight <= 0. else grid.DistanceTransform()
    neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)] + ([] if connectivity == 4 else [(1, 1), (1, -1), (-1, 1), (-1, -1)])
    (nx, ny) = grid.occupied.shape
    occupied = grid.occupied
    def Heuristic(cell): #the length of a path of straight and diagonal steps, without obstacles
        (dx, dy) = (abs(cell[0] - cellGoal[0]), abs(cell[1] - cellGoal[1]))
        return abs(dx - dy) + min(dx, dy)*cMath.sqrt(2.) if connectivity == 8 else dx + dy

    costs = {cellStart: 0.}
    parents = {cellStart: None}
    heap = [(Heuristic(cellStart), 0, cellStart)]
    counter = 0
    closed = set()
    while len(heap) != 0:
        (unused, unused2, cell) = heapq.heappop(heap)
        if cell == cellGoal:
            break
        if cell in closed:
            continue
        closed.add(cell)
        for (dx, dy) in neighbors:
            (i, j) = (cell[0] + dx, cell[1] + dy)
            if not (0 <= i < nx and 0 <= j < ny) or occupied[i, j]:
                continue
            if dx != 0 and dy != 0 and (occupied[cell[0] + dx, cell[1]] or occupied[cell[0], cell[1] + dy]):
                continue
            step = (cMath.sqrt(2.) if dx != 0 and dy != 0 else 1.)
            if clearance is not None:
                step *= 1. + clearanceWeight/max(clearance[i, j], 1e-12)
            cost = costs[cell] + step
            if (i, j) not in costs or cost < costs[(i, j)]:
                costs[(i, j)] = cost
                parents[(i, j)] = cell
                counter += 1
                heapq.heappush(heap, (cost + Heuristic((i, j)), counter, (i, j)))
    if cellGoal not in parents:
        raise ValueError('PlanGridPath: the goal cannot be reached from the start')
    cells = []
    cell = cellGoal
    while cell is not None:
        cells += [cell]
        cell = parents[cell]
    path = [np.array(start[:2], dtype=float)] + [grid.CenterOfCell(c) for c in cells[::-1][1:-1]] + [np.array(goal[:2], dtype=float)]
    return _SmoothByLineOfSight(grid, path) if smooth else np.array(path)


def PlanSE2Path(shapes, obstacles, start, goal, xRange, yRange, phiRange=(-2*np.pi, 2*np.pi), margin=0., **options):
    """A path of a robot in the plane in the coordinates x, y and the rotation phi by RRT-Connect, for narrow spaces in which the shape of the robot matters and a grid with the radius of the robot would be closed; further keyword arguments go to PlanJointPath, e.g. maxStep and seed.

    Args:
        shapes: the collision shapes of the robot, in its frame, in a list as for Robot.GetCollisionModel
        obstacles: the obstacles as for Robot.GetCollisionModel
        start: the pose [x, y, phi] of the start
        goal: the pose [x, y, phi] of the goal
        xRange: [xMin, xMax], the range of x
        yRange: [yMin, yMax], the range of y
        phiRange: the range of phi
        margin: the distance which the shapes keep

    Returns:
        array of the waypoints [x, y, phi], one per row, from start to goal; straight motions in these coordinates are free
    """
    from exudyn.robotics.roboticsCore import Robot, RobotLink
    robot = Robot(gravity=[0., 0., 0.])
    inertia = np.eye(3)
    for (i, jointType) in enumerate(['Px', 'Py', 'Rz']):
        robot.AddLink(RobotLink(mass=1., COM=[0, 0, 0], inertia=inertia, jointType=jointType, preHT=exudyn.HT(), collisionShapes=list(shapes) if i == 2 else []))
    limits = np.array([list(xRange), list(yRange), list(phiRange)], dtype=float)
    return PlanJointPath(robot.GetCollisionModel(obstacles=obstacles), start, goal, limits, margin, **options)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def PathWithHeadings(path, initialHeading=None):
    """The poses of a differential drive along a path in the plane, which turns on the spot at each waypoint: the waypoints twice, with the heading of the segment before and after; a coordinate is a Trajectory waypoint.

    Args:
        path: the waypoints [x, y], one per row
        initialHeading: None to start in the direction of the first segment, or the heading at the start

    Returns:
        array of the waypoints [x, y, phi], one per row; phi is continuous, the shorter turn is taken
    """
    points = np.array(path, dtype=float)[:, :2]
    headings = [float(cMath.atan2(b[1] - a[1], b[0] - a[0])) for (a, b) in zip(points[:-1], points[1:])]
    phi = headings[0] if initialHeading is None else float(initialHeading)
    poses = [[points[0, 0], points[0, 1], phi]]
    for (k, heading) in enumerate(headings):
        phi = phi + (heading - phi + np.pi) % (2*np.pi) - np.pi
        if abs(phi - poses[-1][2]) > 1e-12:
            poses += [[points[k, 0], points[k, 1], phi]]
        poses += [[points[k + 1, 0], points[k + 1, 1], phi]]
    return np.array(poses)


class PurePursuit:
    """The pure-pursuit controller of a differential drive: the robot drives on the arc through a point of the path at the distance lookAhead ahead of it (R. C. Coulter, 1992).

    Example:
        controller = PurePursuit(path, lookAhead=0.3, speed=0.4)
        (v, omega) = controller.Control([x, y, phi])
    """
    def __init__(self, path, lookAhead, speed, goalTolerance=0.02, maxTurnRate=np.inf):
        """create the controller

        Args:
            path: the waypoints [x, y], one per row
            lookAhead: the distance of the point of the path which is followed
            speed: the forward velocity
            goalTolerance: the distance to the end of the path at which the robot stops
            maxTurnRate: the largest absolute value of the rotation velocity
        """
        if not (lookAhead > 0. and speed > 0.):
            raise ValueError('PurePursuit: lookAhead and speed must be positive')
        self.path = np.array(path, dtype=float)[:, :2]
        (self.lookAhead, self.speed, self.goalTolerance, self.maxTurnRate) = (float(lookAhead), float(speed), float(goalTolerance), float(maxTurnRate))
        self.segment = 0
        self.finished = False

    def _TargetPoint(self, position):
        """the point of the path at lookAhead ahead of the projection of position on it, going through the segments in their order"""
        segments = len(self.path) - 1
        k = self.segment
        def Projection(k):
            (a, b) = (self.path[k], self.path[k + 1])
            return float(np.clip((position - a) @ (b - a)/max((b - a) @ (b - a), 1e-30), 0., 1.))
        while k < segments - 1 and Projection(k) >= 1.: #the segment was passed
            k += 1
        self.segment = k
        s = Projection(k)
        remaining = self.lookAhead
        point = self.path[k] + s*(self.path[k + 1] - self.path[k])
        for m in range(k, segments):
            length = float(cMath.norm(self.path[m + 1] - point))
            if remaining <= length:
                return point + (self.path[m + 1] - point)*remaining/max(length, 1e-30)
            remaining -= length
            point = self.path[m + 1]
        return self.path[-1]

    def Control(self, pose):
        """the velocities of the robot at a pose

        Args:
            pose: [x, y, phi] of the robot

        Returns:
            (v, omega), the forward velocity and the rotation velocity; (0, 0) after the end of the path is reached
        """
        position = np.array(pose[:2], dtype=float)
        if float(cMath.norm(self.path[-1] - position)) <= self.goalTolerance:
            self.finished = True
        if self.finished:
            return (0., 0.)
        target = self._TargetPoint(position)
        delta = target - position
        (c, s) = (cMath.cos(pose[2]), cMath.sin(pose[2]))
        (xLocal, yLocal) = (c*delta[0] + s*delta[1], -s*delta[0] + c*delta[1])
        distance2 = xLocal*xLocal + yLocal*yLocal
        if xLocal < 0.: #the target is behind: turn on the spot
            return (0., float(np.sign(yLocal if yLocal != 0. else 1.))*(self.speed/self.lookAhead if np.isinf(self.maxTurnRate) else self.maxTurnRate))
        curvature = 2.*yLocal/max(distance2, 1e-30)
        omega = float(np.clip(self.speed*curvature, -self.maxTurnRate, self.maxTurnRate))
        return (self.speed, omega)


def DifferentialDriveWheelVelocities(v, omega, wheelRadius, trackWidth):
    """The velocities of the two wheels of a differential drive.

    Args:
        v: the forward velocity of the center between the wheels
        omega: the rotation velocity about the vertical axis
        wheelRadius: the radius of the wheels
        trackWidth: the distance of the wheels

    Returns:
        (wLeft, wRight), the angular velocities of the left and the right wheel
    """
    return ((v - 0.5*trackWidth*omega)/wheelRadius, (v + 0.5*trackWidth*omega)/wheelRadius)
