def ComputeOrthonormalBasisVectors(vector0: ArrayLike) -> NDArray: 
    """The normalized vector0 and two normals to it, as the rows of a 3x3 array (unpacked as [v0, n1, n2] = ...); the normals are not unique; for vector0 = [0,0,0] the rows are vector0 and the x and y axes; computed by EXUmath::ComputeOrthogonalBasisVectors of the C++ code."""
    ...
def ComputeOrthonormalBasis(vector0: ArrayLike) -> NDArray: 
    """The rotation matrix whose columns are the rows of ComputeOrthonormalBasisVectors(vector0): the first column is parallel to vector0; computed by EXUmath::ComputeOrthogonalBasisVectors of the C++ code."""
    ...
def GramSchmidt(vector0: ArrayLike, vector1: ArrayLike) -> NDArray: 
    """The normalized vector0, the normalized part of vector1 normal to vector0, and their cross product, as the rows of a 3x3 array; zero or parallel vectors raise a ValueError; computed by GramSchmidt of PyCUtilities.h of the C++ code."""
    ...
def SymmetricBasis(vector0: ArrayLike, vector1: ArrayLike) -> NDArray: 
    """The rotation matrix of the symmetric basis of two non-parallel vectors (#2228): its first two columns lie in the plane of the vectors at 45 degrees to their bisector, the first as close to vector0 as the second to vector1, the third is their cross product; symmetric in the two vectors, independent of their lengths, the rotation of the polar decomposition of the normalized vectors; the frame of the slope nodes and of ObjectANCFBeam and ObjectANCFThinPlate; zero or parallel vectors raise a ValueError; computed by RigidBodyMath::SymmetricBasis of the C++ code."""
    ...
def SymmetricBasisRotationJacobian(vector0: ArrayLike, vector1: ArrayLike) -> NDArray: 
    """The 3x6 rotation Jacobian G of SymmetricBasis(vector0, vector1): the global angular velocity of the basis is G @ [vector0_t, vector1_t], for the time derivatives of the two vectors; computed by RigidBodyMath::SymmetricBasisRotationJacobian of the C++ code."""
    ...
def Skew(vector: ArrayLike) -> NDArray: 
    """The skew symmetric 3x3 matrix of a 3D vector, which gives the cross product: Skew(a) @ b = a x b; for a vector of 3n components the n skew matrices one below the other (3n x 3); for a matrix of 3n rows and m columns the skew matrices of its columns side by side (3n x 3m), as needed for the floating frame of reference formulation; computed by Skew of PyCUtilities.h of the C++ code."""
    ...
def Skew2Vec(skew: ArrayLike) -> NDArray: 
    """The vector of a skew symmetric 3x3 matrix; for a 4x4 matrix of the Lie algebra se(3) the 6 values [u, w], u the translational part (last column) and w the vector of the skew part; computed by Skew2Vec of PyCUtilities.h of the C++ code."""
    ...
def EulerParameters2G(eulerParameters: ArrayLike) -> NDArray: 
    r"""The 3x4 matrix G of Euler parameters, $\tomega = \Gm \dot \pv$ with the global angular velocity; computed by RigidBodyMath::EP2G of the C++ code."""
    ...
def EulerParameters2GLocal(eulerParameters: ArrayLike) -> NDArray: 
    r"""The 3x4 matrix G of Euler parameters for the local (body-fixed) angular velocity, $\LU{b}{\tomega} = \LU{b}{\Gm} \dot \pv$; computed by RigidBodyMath::EP2Glocal of the C++ code."""
    ...
def EulerParameters2RotationMatrix(eulerParameters: ArrayLike) -> NDArray: 
    """The rotation matrix of 4 Euler parameters; computed by RigidBodyMath::EP2RotationMatrix of the C++ code."""
    ...
def RotationMatrix2EulerParameters(rotationMatrix: ArrayLike) -> NDArray: 
    """The 4 Euler parameters of a rotation matrix, normalized, so that an inaccurate rotation matrix gives unit Euler parameters; computed by RigidBodyMath::RotationMatrix2EP of the C++ code."""
    ...
def AngularVelocity2EulerParameters_t(angularVelocity: ArrayLike, eulerParameters: ArrayLike) -> NDArray: 
    r"""The time derivative of the Euler parameters from the global angular velocity, $\dot \pv = \frac{1}{4}\Gm\tp \tomega$, for unit Euler parameters; computed by RigidBodyMath::EP2G of the C++ code."""
    ...
def RotationVector2RotationMatrix(rotationVector: ArrayLike) -> NDArray: 
    """The rotation matrix of a rotation vector, the same as NodeRigidBodyRotVecLG computes; computed by RigidBodyMath::RotationVector2RotationMatrix of the C++ code."""
    ...
def RotationMatrix2RotationVector(rotationMatrix: ArrayLike) -> NDArray: 
    """The rotation vector of a rotation matrix, with an angle in [0, pi]; computed by EXUlie::LogSO3Vector of the C++ code."""
    ...
def ComputeRotationAxisFromRotationVector(rotationVector: ArrayLike) -> NDArray: 
    """The unit rotation axis of a rotation vector; zero for the zero vector; computed by EXUlie::RotationVector2RotationAxis of the C++ code."""
    ...
def ExpSO3(rotationVector: ArrayLike) -> NDArray: 
    """The exponential map of the Lie group SO(3): the rotation matrix of a rotation vector, computed with the sinc function; computed by EXUlie::ExpSO3 of the C++ code."""
    ...
def LogSO3(rotationMatrix: ArrayLike) -> NDArray: 
    """The logarithmic map of the Lie group SO(3): the skew symmetric matrix of the rotation vector of a rotation matrix; computed by EXUlie::LogSO3 of the C++ code."""
    ...
def RotXYZ2RotationMatrix(rot: ArrayLike) -> NDArray: 
    """The rotation matrix of the Tait-Bryan angles rot = [rotX, rotY, rotZ] (in radians), A = Ax*Ay*Az; computed by RigidBodyMath::RotXYZ2RotationMatrix of the C++ code."""
    ...
def RotationMatrix2RotXYZ(rotationMatrix: ArrayLike) -> NDArray: 
    """The Tait-Bryan angles [rotX, rotY, rotZ] of a rotation matrix; at the singularity rotY = pi/2, -pi/2, ...
    
    rotX is set 0; computed by RigidBodyMath::RotationMatrix2RotXYZ of the C++ code
    """
    ...
def RotXYZ2G(rot: ArrayLike) -> NDArray: 
    r"""The 3x3 matrix G of Tait-Bryan angles, $\LU{0}{\tomega} = \Gm \dot \ttheta$ with the global angular velocity; computed by RigidBodyMath::RotXYZ2GTemplate of the C++ code."""
    ...
def RotXYZ2G_t(rot: ArrayLike, rot_t: ArrayLike) -> NDArray: 
    """The time derivative of RotXYZ2G, for the angles rot and their time derivatives rot_t; computed by RigidBodyMath::RotXYZ2G_tTemplate of the C++ code."""
    ...
def RotXYZ2GLocal(rot: ArrayLike) -> NDArray: 
    r"""The 3x3 matrix G of Tait-Bryan angles for the local (body-fixed) angular velocity, $\LU{b}{\tomega} = \LU{b}{\Gm} \dot \ttheta$; computed by RigidBodyMath::RotXYZ2GlocalTemplate of the C++ code."""
    ...
def RotXYZ2GLocal_t(rot: ArrayLike, rot_t: ArrayLike) -> NDArray: 
    """The time derivative of RotXYZ2GLocal, for the angles rot and their time derivatives rot_t; computed by RigidBodyMath::RotXYZ2Glocal_tTemplate of the C++ code."""
    ...
def AngularVelocity2RotXYZ_t(angularVelocity: ArrayLike, rotation: ArrayLike) -> NDArray: 
    """The time derivative of the Tait-Bryan angles rotation from the global angular velocity; a ValueError at the singularity, |cos(rotation[1])| < 1e-14; computed by RigidBodyMath::AngularVelocity2RotXYZ_t of the C++ code."""
    ...
def RotXYZ2EulerParameters(alpha: ArrayLike) -> NDArray: 
    """The 4 Euler parameters of the Tait-Bryan angles alpha, see [Henderson1977]; entry 0 is the scalar part; computed by RigidBodyMath::RotXYZ2EP of the C++ code."""
    ...
def RotationMatrix2RotZYZ(rotationMatrix: ArrayLike, flip: bool=False) -> NDArray: 
    """The Euler angles [Z, Y, Z] (in radians) of a rotation matrix, A = Az*Ay*Az; flip chooses the first angle in quadrant 2 or 3; computed by RigidBodyMath::RotationMatrix2RotZYZ of the C++ code."""
    ...
def RotationMatrixX(angleRad: float) -> NDArray: 
    """The rotation matrix of a rotation by angleRad (in radians) about the x-axis; computed by RigidBodyMath::RotationMatrixX of the C++ code."""
    ...
def RotationMatrixY(angleRad: float) -> NDArray: 
    """The rotation matrix of a rotation by angleRad (in radians) about the y-axis; computed by RigidBodyMath::RotationMatrixY of the C++ code."""
    ...
def RotationMatrixZ(angleRad: float) -> NDArray: 
    """The rotation matrix of a rotation by angleRad (in radians) about the z-axis; computed by RigidBodyMath::RotationMatrixZ of the C++ code."""
    ...
def RotationMatrix2D(angleRad: float) -> NDArray: 
    """The 2x2 rotation matrix of a rotation by angleRad (in radians) in the plane; computed by RotationMatrix2D of PyCUtilities.h of the C++ code."""
    ...
def PointVelocity(HT: HT, twist: Twist, localPoint: ArrayLike) -> NDArray: 
    r"""The velocity of a point fixed in the frame HT, given by its position localPoint in the frame, for the twist of the frame: $\vv + \tomega \times (\Rot\, \pv_{loc})$; computed by RigidBodyMath::PointVelocity of the C++ code."""
    ...
def LocalTwist(HT: HT, twist: Twist) -> Twist: 
    r"""The twist in the components of the frame HT: $[\Rot\tp \vv,\; \Rot\tp \tomega]$; computed by RigidBodyMath::LocalTwist of the C++ code."""
    ...
def RelativeTwist(HT0: HT, twist0: Twist, HT1: HT, twist1: Twist) -> Twist: 
    r"""The velocity of frame 1 relative to frame 0 in the components of frame 0, the time derivative of HT0.Relative(HT1): $[\Rot_0\tp (\vv_1 - \vv_0 - \tomega_0 \times (\pv_1 - \pv_0)),\; \Rot_0\tp (\tomega_1 - \tomega_0)]$; computed by RigidBodyMath::RelativeTwist of the C++ code."""
    ...

#stub information for class KinematicChain functions
class KinematicChain:
    """The KinematicChain is the kinematics of a serial or tree-like robot in C++: per link a joint type, the parent link, preHT (from the link frame of the parent to the joint frame, before the joint motion), localHT (from the joint frame after the joint motion to the link frame), mass, center of mass and inertia tensor in the link frame; the base HT, the tool HT relative to the frame of the last link, and gravity - the data of exudyn.robotics.Robot, whose GetKinematicChain() creates it.

    The joint frame of link i is JointHT_i = LinkHT_parent * preHT_i * HTjoint(q_i), its link frame LinkHT_i = JointHT_i * localHT_i
    Robot.JointHT, Robot.LinkHT, Robot.Jacobian, Robot.StaticTorques, Robot.InverseDynamics and Robot.InverseKinematicsAnalytic are the Python versions of the same algorithms.
    """
    def NumberOfLinks(self) -> int: 
        """The number of links."""
        ...
    def JointHT(self, q: ArrayLike) -> list[HT]: 
        """The global joint frames of all links, after the joint motion, for the joint coordinates q, a list of exu.HT; the Python version is Robot.JointHT."""
        ...
    def LinkHT(self, q: ArrayLike) -> list[HT]: 
        """The global link frames of all links for the joint coordinates q, a list of exu.HT; the Python version is Robot.LinkHT."""
        ...
    def ToolHT(self, q: ArrayLike) -> HT: 
        """The global frame of the tool, the link frame of the last link times the tool HT, for the joint coordinates q."""
        ...
    def Jacobian(self, q: ArrayLike, linkIndex: int=-1, localPosition: ArrayLike=None) -> NDArray: 
        """The 6 x n Jacobian of the point localPosition (None the origin), given in the link frame of link linkIndex (-1 the last link), for the joint coordinates q: rows 0 to 2 the velocity of the point, rows 3 to 5 the angular velocity of the link, per joint velocity; the columns of the joints not between the link and the base are zero; the Python version is Robot.Jacobian, which takes the joint frames and the global position."""
        ...
    def StaticTorques(self, q: ArrayLike) -> NDArray: 
        """The generalized forces of gravity on the joints for the joint coordinates q, the sum over the links j of J_j^T m_j g with the Jacobian J_j of the center of mass of link j, computed in one backward pass over the links (the mass and its first moment summed over the links after each joint); the joint torques that hold the robot are their negative; the Python version is Robot.StaticTorques."""
        ...
    def InverseDynamics(self, q: ArrayLike, q_t: ArrayLike, q_tt: ArrayLike) -> NDArray: 
        """The joint torques and forces that move the chain with the joint accelerations q_tt at the joint coordinates q and velocities q_t against gravity (#3276), M(q) q_tt + h(q, q_t) minus the generalized forces of gravity - at q_t = q_tt = 0 the negative of StaticTorques; by the recursive Newton-Euler algorithm, O(n), with the masses, COMs and inertias; the Python version is Robot.InverseDynamics."""
        ...
    def InverseKinematicsNumerical(self, HT: HT, qInitial: ArrayLike, weights: ArrayLike=None, tolerance: float=1e-10, maxIterations: int=100, restarts: int=0) -> tuple[NDArray, bool, int]: 
        """The numerical inverse kinematics by Levenberg-Marquardt: joint coordinates q with ToolHT(q) equal to HT (exu.HT or 4x4 matrix), starting from qInitial, for any serial chain (also with fewer or more than 6 joints); the error e is the position difference and the rotation vector of R_HT R(q)^T, multiplied by the 6 weights (None: ones; zeros for components that are free), the step (J^T J + lambda I) dq = J^T e with lambda adapted (divided by 10 after a step that lowers |e|, multiplied by 10 until one does); converged for |e| <= tolerance; restarts further starts from qInitial shifted by the fixed sequence pi (2 frac(k sqrt(p_i)) - 1) with the primes p_i for the revolute joints; returns (q, success, iterations), q of the last start if none converged; the Python version is Robot.InverseKinematicsNumerical."""
        ...
    def ArmConfiguration(self, q: ArrayLike) -> list[int]: 
        """The configuration [shoulder, elbow, wrist] of the arm at the joint coordinates q, which tells the solutions of InverseKinematicsAnalytic apart, each +1 or -1 and 0 in the singular configuration between them; from the joint axes omega_i at q, independent of the DH convention: with the wrist center W (the point of the axes 5 and 6; the tool position for 3 links) and the vectors a, b from axis 2 to axis 3 and from axis 3 to W (to axis 4 for the UR) normal to omega_2, shoulder the sign of (W - p_1).
        
        (omega_1 x omega_2), elbow of omega_3 . (a x b), wrist of omega_5 . (omega_4 x omega_6) (0 for 3 links); the Python version is Robot.ArmConfiguration
        """
        ...
    def InverseKinematicsAnalytic(self, HT: HT, qReference: ArrayLike=None, configuration: list[int]=None) -> NDArray: 
        """The analytic inverse kinematics: all joint coordinates q with ToolHT(q) equal to HT (exu.HT or 4x4 matrix), one solution per row, no rows if the pose cannot be reached; for revolute joints in series: 3 links with the axes 2 and 3 parallel (the position of the tool only, up to 4 solutions), 6 links with the axes 2 and 3 parallel and the axes 4, 5, 6 through one point (a spherical wrist: Puma560, KUKA, Staeubli; up to 8), 6 links with the axes 2, 3, 4 parallel and the axes 5 and 6 crossing (UR; up to 8), a ValueError for other robots; by the subproblems of Paden and Kahan on the product of exponentials, every solution checked by the forward kinematics; with qReference the angles nearest to it and the solutions sorted by their distance from it, the nearest first, else in [-pi, pi); an angle that is free in a singular configuration takes its value from qReference (or 0); with configuration [shoulder, elbow, wrist] of ArmConfiguration (each 1, -1, or 0 or None for any; e.g. ArmConfiguration(qReference)) only the solutions of this configuration and of its boundary; the Python version is Robot.InverseKinematicsAnalytic."""
        ...
    def __repr__(self) -> str: 
        """The string representation of the KinematicChain."""
        ...

#stub information for class ConvexShape functions
class ConvexShape:
    """A ConvexShape is a convex primitive in its local frame, for the distance of two shapes by Distance: a sphere about the origin, a capsule (a segment of the given length along the local z axis, centered, with the radius), a cylinder (along the local z axis, centered, with radius and length), a box (centered, with the edge lengths size along x, y, z), a triangle (3 points), the convex hull of points, or a truncated cone (centered along the local z axis, with radius and radius1 at its ends).

    Distance computes it by GJK and EPA, the same functions as the contact items in C++.
    """
    def ShapeType(self) -> str: 
        """The type of the shape, 'Sphere', 'Capsule', 'Cylinder', 'Box', 'Triangle' or 'Points'."""
        ...
    def BoundingRadius(self) -> float: 
        """The radius of a sphere about the origin of the local frame containing the shape."""
        ...
    def __repr__(self) -> str: 
        """The string representation of the ConvexShape."""
        ...

#stub information for class CollisionModel functions
class CollisionModel:
    """A CollisionModel is the collision check of a robot and its environment in C++: a KinematicChain with ConvexShape on its links and base, and obstacles; a link is not checked against its parent; Robot.GetCollisionModel builds it from the collision shapes of the links"""
    def AddLinkShape(self, linkIndex: int, shape: ConvexShape, HT: HT=None) -> None: 
        """Add a shape on link linkIndex (-1 the base) in the frame HT relative to the link frame; None for the link frame."""
        ...
    def AddObstacle(self, shape: ConvexShape, HT: HT=None) -> None: 
        """Add a shape fixed in the world, in the global frame HT."""
        ...
    def ExcludeLinkPair(self, link0: int, link1: int) -> None: 
        """The pair of links is not checked, in addition to each link and its parent; -1 the base."""
        ...
    def ExcludeCollidingPairs(self, q: ArrayLike, margin: float=0.) -> int: 
        """Exclude all pairs of links and base whose shapes are not farther than margin at the joint coordinates q, as the shapes of neighboring links usually overlap at any configuration; returns the number of shape pairs found."""
        ...
    def NumberOfShapes(self) -> int: 
        """The number of shapes on the links and the base and of obstacles."""
        ...
    def MinimumDistance(self, q: ArrayLike) -> float: 
        """The smallest signed distance over all checked pairs at the joint coordinates q, negative for a penetration."""
        ...
    def IsFree(self, q: ArrayLike, margin: float=0.) -> bool: 
        """True if all checked distances at the joint coordinates q are larger than margin."""
        ...
    def IsEdgeFree(self, q0: ArrayLike, q1: ArrayLike, margin: float=0., maxStep: float=0.02) -> bool: 
        """True if the straight motion in joint space from q0 to q1 is free, checked at steps of at most maxStep in any coordinate."""
        ...
    def __repr__(self) -> str: 
        """The string representation of the CollisionModel."""
        ...
def Distance(shape0: ConvexShape, HT0: HT, shape1: ConvexShape, HT1: HT, analytic: bool=True) -> tuple[float, NDArray, NDArray, NDArray]: 
    """The signed distance of two ConvexShape in the frames HT0 and HT1 (exu.HT or 4x4 matrix) as a tuple (distance, point0, point1, normal): distance negative for a penetration (minus the penetration depth), point0 and point1 the closest points of the shapes (the deepest points of a penetration), normal the unit vector from shape 0 to shape 1, all global; by GJK on the cores (the sphere a point, the capsule a segment, their radii subtracted) and EPA where the cores overlap, to the rounding for spheres, capsules, boxes, triangles and points; a cylinder, whose curved side GJK and EPA approach slowly, is refined by alternating projections of the closest points (separated) and by moving shape 1 out along the normal (penetrating), to about 1e-12 of the size of the shapes when they are apart and 1e-7 within about 1e-6 of contact; with analytic=True a sphere and a cylinder by the closed formula."""
    ...
