#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN definition file
#
# Details:  the submodule exudyn.cUtilities: the kinematics of rigid bodies in C++ for Python (#2960); pb holds the
#           page, module the functions, whose C++ bindings go into pybind_manual_classes.h (the compiled submodule
#           exudyn._cUtilities) and whose stubs go into python/exudyn/cUtilities.pyi. The functions are in
#           src/Pymodules/PyCUtilities.h; python/exudyn/cUtilities.py takes them, and exudyn.rigidBodyUtilities
#           calls them under the same names.
#           The calls are recorded by PybindInterface (pybindTypes.py) and replayed by
#           tools/generators/pybindEmitter.py.
#
#           DESCRIPTIONS: read definitions/README.md, section "Writing a
#           description", before writing or changing one - what the text may
#           contain, and how it is checked.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-11
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from pybindTypes import *

pb = PybindInterface()
module = PybindInterface()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
pb.AddDocu(r"""The submodule `exudyn.cUtilities` gives Python the kinematics functions of the C++ code of Exudyn: rotation matrices, Euler parameters, Tait-Bryan angles, rotation vectors, skew matrices and basis vectors. They compute with the math functions of `exudyn.cMath` and give the same bits on every platform. `exudyn.rigidBodyUtilities` calls them under the same names; the documentation there says which. A vector is given as list or numpy array, also of shape (3,1) or (1,3); a matrix as list of lists or numpy array; the result is a numpy array. A wrong size raises a `ValueError`.""",
            section='exudyn.cUtilities', sectionLevel=1, sectionLabel='sec-exudyn-cutilities')

module.AddDocu(r"""Usage: `import exudyn.cUtilities as cu` and `cu.RotationMatrixZ(0.1)`. The functions are:""")

#the compiled submodule; 'm' is the submodule in this block, so that the declarations below bind into it
module.CppCode('        {\n        py::module cUtilitiesSubmodule = m.def_submodule("_cUtilities", "the compiled functions of exudyn.cUtilities");\n'
               '        {\n        py::module& m = cUtilitiesSubmodule;\n')

vector = 'ArrayLike'
returnType = 'NDArray'

#name, arguments, defaults, argument types, C++ function computing it, description
functions = [
    ('ComputeOrthonormalBasisVectors', ['vector0'], [], [vector], 'EXUmath::ComputeOrthogonalBasisVectors',
     'the normalized vector0 and two normals to it, as the rows of a 3x3 array (unpacked as [v0, n1, n2] = ...); the normals are not unique; for vector0 = [0,0,0] the rows are vector0 and the x and y axes'),
    ('ComputeOrthonormalBasis', ['vector0'], [], [vector], 'EXUmath::ComputeOrthogonalBasisVectors',
     'the rotation matrix whose columns are the rows of ComputeOrthonormalBasisVectors(vector0): the first column is parallel to vector0'),
    ('GramSchmidt', ['vector0', 'vector1'], [], [vector, vector], 'GramSchmidt of PyCUtilities.h',
     'the normalized vector0, the normalized part of vector1 normal to vector0, and their cross product, as the rows of a 3x3 array; zero or parallel vectors raise a ValueError'),
    ('SymmetricBasis', ['vector0', 'vector1'], [], [vector, vector], 'RigidBodyMath::SymmetricBasis',
     'the rotation matrix of the symmetric basis of two non-parallel vectors (#2228): its first two columns lie in the plane of the vectors at 45 degrees to their bisector, the first as close to vector0 as the second to vector1, the third is their cross product; symmetric in the two vectors, independent of their lengths, the rotation of the polar decomposition of the normalized vectors; the frame of the slope nodes and of ObjectANCFBeam and ObjectANCFThinPlate; zero or parallel vectors raise a ValueError'),
    ('SymmetricBasisRotationJacobian', ['vector0', 'vector1'], [], [vector, vector], 'RigidBodyMath::SymmetricBasisRotationJacobian',
     'the 3x6 rotation Jacobian G of SymmetricBasis(vector0, vector1): the global angular velocity of the basis is G @ [vector0_t, vector1_t], for the time derivatives of the two vectors'),
    ('Skew', ['vector'], [], [vector], 'Skew of PyCUtilities.h',
     'the skew symmetric 3x3 matrix of a 3D vector, which gives the cross product: Skew(a) @ b = a x b; for a vector of 3n components the n skew matrices one below the other (3n x 3); for a matrix of 3n rows and m columns the skew matrices of its columns side by side (3n x 3m), as needed for the floating frame of reference formulation'),
    ('Skew2Vec', ['skew'], [], [vector], 'Skew2Vec of PyCUtilities.h',
     'the vector of a skew symmetric 3x3 matrix; for a 4x4 matrix of the Lie algebra se(3) the 6 values [u, w], u the translational part (last column) and w the vector of the skew part'),
    ('EulerParameters2G', ['eulerParameters'], [], [vector], 'RigidBodyMath::EP2G',
     r'the 3x4 matrix G of Euler parameters, $\tomega = \Gm \dot \pv$ with the global angular velocity'),
    ('EulerParameters2GLocal', ['eulerParameters'], [], [vector], 'RigidBodyMath::EP2Glocal',
     r'the 3x4 matrix G of Euler parameters for the local (body-fixed) angular velocity, $\LU{b}{\tomega} = \LU{b}{\Gm} \dot \pv$'),
    ('EulerParameters2RotationMatrix', ['eulerParameters'], [], [vector], 'RigidBodyMath::EP2RotationMatrix',
     'the rotation matrix of 4 Euler parameters'),
    ('RotationMatrix2EulerParameters', ['rotationMatrix'], [], [vector], 'RigidBodyMath::RotationMatrix2EP',
     'the 4 Euler parameters of a rotation matrix, normalized, so that an inaccurate rotation matrix gives unit Euler parameters'),
    ('AngularVelocity2EulerParameters_t', ['angularVelocity', 'eulerParameters'], [], [vector, vector], 'RigidBodyMath::EP2G',
     r'the time derivative of the Euler parameters from the global angular velocity, $\dot \pv = \frac{1}{4}\Gm\tp \tomega$, for unit Euler parameters'),
    ('RotationVector2RotationMatrix', ['rotationVector'], [], [vector], 'RigidBodyMath::RotationVector2RotationMatrix',
     'the rotation matrix of a rotation vector, the same as NodeRigidBodyRotVecLG computes'),
    ('RotationMatrix2RotationVector', ['rotationMatrix'], [], [vector], 'EXUlie::LogSO3Vector',
     'the rotation vector of a rotation matrix, with an angle in [0, pi]'),
    ('ComputeRotationAxisFromRotationVector', ['rotationVector'], [], [vector], 'EXUlie::RotationVector2RotationAxis',
     'the unit rotation axis of a rotation vector; zero for the zero vector'),
    ('ExpSO3', ['rotationVector'], [], [vector], 'EXUlie::ExpSO3',
     'the exponential map of the Lie group SO(3): the rotation matrix of a rotation vector, computed with the sinc function'),
    ('LogSO3', ['rotationMatrix'], [], [vector], 'EXUlie::LogSO3',
     'the logarithmic map of the Lie group SO(3): the skew symmetric matrix of the rotation vector of a rotation matrix'),
    ('RotXYZ2RotationMatrix', ['rot'], [], [vector], 'RigidBodyMath::RotXYZ2RotationMatrix',
     'the rotation matrix of the Tait-Bryan angles rot = [rotX, rotY, rotZ] (in radians), A = Ax*Ay*Az'),
    ('RotationMatrix2RotXYZ', ['rotationMatrix'], [], [vector], 'RigidBodyMath::RotationMatrix2RotXYZ',
     'the Tait-Bryan angles [rotX, rotY, rotZ] of a rotation matrix; at the singularity rotY = pi/2, -pi/2, ... rotX is set 0'),
    ('RotXYZ2G', ['rot'], [], [vector], 'RigidBodyMath::RotXYZ2GTemplate',
     r'the 3x3 matrix G of Tait-Bryan angles, $\LU{0}{\tomega} = \Gm \dot \ttheta$ with the global angular velocity'),
    ('RotXYZ2G_t', ['rot', 'rot_t'], [], [vector, vector], 'RigidBodyMath::RotXYZ2G_tTemplate',
     'the time derivative of RotXYZ2G, for the angles rot and their time derivatives rot_t'),
    ('RotXYZ2GLocal', ['rot'], [], [vector], 'RigidBodyMath::RotXYZ2GlocalTemplate',
     r'the 3x3 matrix G of Tait-Bryan angles for the local (body-fixed) angular velocity, $\LU{b}{\tomega} = \LU{b}{\Gm} \dot \ttheta$'),
    ('RotXYZ2GLocal_t', ['rot', 'rot_t'], [], [vector, vector], 'RigidBodyMath::RotXYZ2Glocal_tTemplate',
     'the time derivative of RotXYZ2GLocal, for the angles rot and their time derivatives rot_t'),
    ('AngularVelocity2RotXYZ_t', ['angularVelocity', 'rotation'], [], [vector, vector], 'RigidBodyMath::AngularVelocity2RotXYZ_t',
     'the time derivative of the Tait-Bryan angles rotation from the global angular velocity; a ValueError at the singularity, |cos(rotation[1])| < 1e-14'),
    ('RotXYZ2EulerParameters', ['alpha'], [], [vector], 'RigidBodyMath::RotXYZ2EP',
     'the 4 Euler parameters of the Tait-Bryan angles alpha, see [CITE:Henderson1977]; entry 0 is the scalar part'),
    ('RotationMatrix2RotZYZ', ['rotationMatrix', 'flip'], ['', 'False'], [vector, 'bool'], 'RigidBodyMath::RotationMatrix2RotZYZ',
     'the Euler angles [Z, Y, Z] (in radians) of a rotation matrix, A = Az*Ay*Az; flip chooses the first angle in quadrant 2 or 3'),
    ('RotationMatrixX', ['angleRad'], [], ['float'], 'RigidBodyMath::RotationMatrixX',
     'the rotation matrix of a rotation by angleRad (in radians) about the x-axis'),
    ('RotationMatrixY', ['angleRad'], [], ['float'], 'RigidBodyMath::RotationMatrixY',
     'the rotation matrix of a rotation by angleRad (in radians) about the y-axis'),
    ('RotationMatrixZ', ['angleRad'], [], ['float'], 'RigidBodyMath::RotationMatrixZ',
     'the rotation matrix of a rotation by angleRad (in radians) about the z-axis'),
    ('RotationMatrix2D', ['angleRad'], [], ['float'], 'RotationMatrix2D of PyCUtilities.h',
     'the 2x2 rotation matrix of a rotation by angleRad (in radians) in the plane'),
    ]

for (name, argList, defaultArgs, argTypes, cFunction, description) in functions:
    module.DefPyFunctionAccess(cClass='', pyName=name, cName='PyCUtilities::' + name,
                               description=description + '; computed by ' + cFunction + ' of the C++ code',
                               argList=argList,
                               defaultArgs=defaultArgs,
                               argTypes=argTypes,
                               returnType=returnType,
                               )

#the twist functions on exudyn.HT and exudyn.Twist (#3174); exudyn.symbolics.kinematics calls them for numbers
twistFunctions = [
    ('PointVelocity', ['HT', 'twist', 'localPoint'], ['HT', 'Twist', vector], 'NDArray', 'RigidBodyMath::PointVelocity',
     r'the velocity of a point fixed in the frame HT, given by its position localPoint in the frame, for the twist of the frame: $\vv + \tomega \times (\Rot\, \pv_{loc})$'),
    ('LocalTwist', ['HT', 'twist'], ['HT', 'Twist'], 'Twist', 'RigidBodyMath::LocalTwist',
     r'the twist in the components of the frame HT: $[\Rot\tp \vv,\; \Rot\tp \tomega]$'),
    ('RelativeTwist', ['HT0', 'twist0', 'HT1', 'twist1'], ['HT', 'Twist', 'HT', 'Twist'], 'Twist', 'RigidBodyMath::RelativeTwist',
     r'the velocity of frame 1 relative to frame 0 in the components of frame 0, the time derivative of HT0.Relative(HT1): $[\Rot_0\tp (\vv_1 - \vv_0 - \tomega_0 \times (\pv_1 - \pv_0)),\; \Rot_0\tp (\tomega_1 - \tomega_0)]$'),
    ]

for (name, argList, argTypes, twistReturnType, cFunction, description) in twistFunctions:
    module.DefPyFunctionAccess(cClass='', pyName=name, cName='PyCUtilities::' + name,
                               description=description + '; computed by ' + cFunction + ' of the C++ code',
                               argList=argList,
                               argTypes=argTypes,
                               returnType=twistReturnType,
                               )

#the kinematic chain of a robot (#3259); exudyn.robotics.Robot.GetKinematicChain() creates it
classStr = 'PyKinematicChain'
module.DefPyStartClass(classStr, 'KinematicChain', 'The KinematicChain is the kinematics of a serial or tree-like robot in C++: per link a joint type, the parent link, preHT (from the link frame of the parent to the joint frame, before the joint motion), localHT (from the joint frame after the joint motion to the link frame), mass, center of mass and inertia tensor in the link frame; the base HT, the tool HT relative to the frame of the last link, and gravity - the data of exudyn.robotics.Robot, whose GetKinematicChain() creates it. The joint frame of link i is JointHT_i = LinkHT_parent * preHT_i * HTjoint(q_i), its link frame LinkHT_i = JointHT_i * localHT_i. Robot.JointHT, Robot.LinkHT, Robot.Jacobian, Robot.StaticTorques, Robot.InverseDynamics and Robot.InverseKinematicsAnalytic are the Python versions of the same algorithms.',
                       subSection=True, labelName='sec:cutilities-kinematicchain', forbidPythonConstructor=True)
module.DefStartTable('KinematicChain')
module.CppCode('        .def(py::init<const py::object&, const py::object&, const py::object&, const py::object&, const py::object&, const py::object&, const py::object&, const py::object&, const py::object&, const py::object&>(), py::arg("jointTypes"), py::arg("preHTs") = py::none(), py::arg("localHTs") = py::none(), py::arg("parents") = py::none(), py::arg("masses") = py::none(), py::arg("COMs") = py::none(), py::arg("baseHT") = py::none(), py::arg("toolHT") = py::none(), py::arg("gravity") = py::none(), py::arg("inertias") = py::none())\n')
module.AddDocuList(['KinematicChain(jointTypes, preHTs=None, localHTs=None, parents=None, masses=None, COMs=None, baseHT=None, toolHT=None, gravity=None, inertias=None): jointTypes a list of Rx, Ry, Rz, Px, Py, Pz (revolute about or prismatic along an axis of the joint frame), preHTs and localHTs a list of one exu.HT (or 4x4 matrix) per link, None the identities, parents the parent link per link (-1 the base, a link before it), None a serial chain, masses and COMs (the centers of mass in the link frames) None for zero, baseHT and toolHT None for the identity, gravity a 3D vector, inertias the 3x3 inertia tensors at the centers of mass in the link frames, None for zero (#3276)'])
for (name, argList, defaultArgs, argTypes, kcReturnType, description) in [
    ('NumberOfLinks', [], [], [], 'int', 'the number of links'),
    ('JointHT', ['q'], [], [vector], 'list[HT]', 'the global joint frames of all links, after the joint motion, for the joint coordinates q, a list of exu.HT; the Python version is Robot.JointHT'),
    ('LinkHT', ['q'], [], [vector], 'list[HT]', 'the global link frames of all links for the joint coordinates q, a list of exu.HT; the Python version is Robot.LinkHT'),
    ('ToolHT', ['q'], [], [vector], 'HT', 'the global frame of the tool, the link frame of the last link times the tool HT, for the joint coordinates q'),
    ('Jacobian', ['q', 'linkIndex', 'localPosition'], ['', '-1', 'py::none()'], [vector, 'int', vector], 'NDArray',
     'the 6 x n Jacobian of the point localPosition (None the origin), given in the link frame of link linkIndex (-1 the last link), for the joint coordinates q: rows 0 to 2 the velocity of the point, rows 3 to 5 the angular velocity of the link, per joint velocity; the columns of the joints not between the link and the base are zero; the Python version is Robot.Jacobian, which takes the joint frames and the global position'),
    ('StaticTorques', ['q'], [], [vector], 'NDArray', 'the generalized forces of gravity on the joints for the joint coordinates q, the sum over the links j of J_j^T m_j g with the Jacobian J_j of the center of mass of link j, computed in one backward pass over the links (the mass and its first moment summed over the links after each joint); the joint torques that hold the robot are their negative; the Python version is Robot.StaticTorques'),
    ('InverseDynamics', ['q', 'q_t', 'q_tt'], [], [vector, vector, vector], 'NDArray', 'the joint torques and forces that move the chain with the joint accelerations q_tt at the joint coordinates q and velocities q_t against gravity (#3276), M(q) q_tt + h(q, q_t) minus the generalized forces of gravity - at q_t = q_tt = 0 the negative of StaticTorques; by the recursive Newton-Euler algorithm, O(n), with the masses, COMs and inertias; the Python version is Robot.InverseDynamics'),
    ('InverseKinematicsNumerical', ['HT', 'qInitial', 'weights', 'tolerance', 'maxIterations', 'restarts'], ['', '', 'py::none()', '1e-10', '100', '0'], ['HT', vector, vector, 'float', 'int', 'int'], 'tuple[NDArray, bool, int]',
     'the numerical inverse kinematics by Levenberg-Marquardt: joint coordinates q with ToolHT(q) equal to HT (exu.HT or 4x4 matrix), starting from qInitial, for any serial chain (also with fewer or more than 6 joints); the error e is the position difference and the rotation vector of R_HT R(q)^T, multiplied by the 6 weights (None: ones; zeros for components that are free), the step (J^T J + lambda I) dq = J^T e with lambda adapted (divided by 10 after a step that lowers |e|, multiplied by 10 until one does); converged for |e| <= tolerance; restarts further starts from qInitial shifted by the fixed sequence pi (2 frac(k sqrt(p_i)) - 1) with the primes p_i for the revolute joints; returns (q, success, iterations), q of the last start if none converged; the Python version is Robot.InverseKinematicsNumerical'),
    ('ArmConfiguration', ['q'], [], [vector], 'list[int]', 'the configuration [shoulder, elbow, wrist] of the arm at the joint coordinates q, which tells the solutions of InverseKinematicsAnalytic apart, each +1 or -1 and 0 in the singular configuration between them; from the joint axes omega_i at q, independent of the DH convention: with the wrist center W (the point of the axes 5 and 6; the tool position for 3 links) and the vectors a, b from axis 2 to axis 3 and from axis 3 to W (to axis 4 for the UR) normal to omega_2, shoulder the sign of (W - p_1) . (omega_1 x omega_2), elbow of omega_3 . (a x b), wrist of omega_5 . (omega_4 x omega_6) (0 for 3 links); the Python version is Robot.ArmConfiguration'),
    ('InverseKinematicsAnalytic', ['HT', 'qReference', 'configuration'], ['', 'py::none()', 'py::none()'], ['HT', vector, 'list[int]'], 'NDArray',
     'the analytic inverse kinematics: all joint coordinates q with ToolHT(q) equal to HT (exu.HT or 4x4 matrix), one solution per row, no rows if the pose cannot be reached; for revolute joints in series: 3 links with the axes 2 and 3 parallel (the position of the tool only, up to 4 solutions), 6 links with the axes 2 and 3 parallel and the axes 4, 5, 6 through one point (a spherical wrist: Puma560, KUKA, Staeubli; up to 8), 6 links with the axes 2, 3, 4 parallel and the axes 5 and 6 crossing (UR; up to 8), a ValueError for other robots; by the subproblems of Paden and Kahan on the product of exponentials, every solution checked by the forward kinematics; with qReference the angles nearest to it and the solutions sorted by their distance from it, the nearest first, else in [-pi, pi); an angle that is free in a singular configuration takes its value from qReference (or 0); with configuration [shoulder, elbow, wrist] of ArmConfiguration (each 1, -1, or 0 or None for any; e.g. ArmConfiguration(qReference)) only the solutions of this configuration and of its boundary; the Python version is Robot.InverseKinematicsAnalytic'),
    ]:
    module.DefPyFunctionAccess(cClass=classStr, pyName=name, cName=name, description=description,
                               argList=argList, defaultArgs=defaultArgs, argTypes=argTypes, returnType=kcReturnType)
module.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', cName='[](const PyKinematicChain &item) {\n            return item.ToString(); }',
                           description='the string representation of the KinematicChain', returnType='str', isLambdaFunction=True)
module.DefPyFinishClass('KinematicChain')

#the convex shapes and their distance (#3317), for the contact primitives and the collision check of path planning
classStr = 'PyConvexShape'
module.DefPyStartClass(classStr, 'ConvexShape', 'A ConvexShape is a convex primitive in its local frame, for the distance of two shapes by Distance: a sphere about the origin, a capsule (a segment of the given length along the local z axis, centered, with the radius), a cylinder (along the local z axis, centered, with radius and length), a box (centered, with the edge lengths size along x, y, z), a triangle (3 points), or the convex hull of points. Distance computes it by GJK and EPA, the same functions as the contact items in C++.',
                       subSection=True, labelName='sec:cutilities-convexshape', forbidPythonConstructor=True)
module.DefStartTable('ConvexShape')
module.CppCode('        .def(py::init<const STDstring&, Real, Real, const py::object&, const py::object&>(), py::arg("shapeType"), py::arg("radius") = 0., py::arg("length") = 0., py::arg("size") = py::none(), py::arg("points") = py::none())\n')
module.AddDocuList(["ConvexShape(shapeType, radius=0., length=0., size=None, points=None): shapeType 'Sphere' (radius), 'Capsule' (radius, length >= 0), 'Cylinder' (radius, length), 'Box' (size, the 3 edge lengths), 'Triangle' (points, 3 rows of 3 coordinates) or 'Points' (points, n rows of 3 coordinates, their convex hull); a parameter that the type does not use, or a radius, length or size not positive, raises a ValueError"])
module.DefPyFunctionAccess(cClass=classStr, pyName='ShapeType', cName='ShapeType', description="the type of the shape, 'Sphere', 'Capsule', 'Cylinder', 'Box', 'Triangle' or 'Points'",
                           returnType='str')
module.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', cName='[](const PyConvexShape &item) {\n            return item.ToString(); }',
                           description='the string representation of the ConvexShape', returnType='str', isLambdaFunction=True)
module.DefPyFinishClass('ConvexShape')

module.DefPyFunctionAccess(cClass='', pyName='Distance', cName='PyCUtilities::Distance',
                           description='the signed distance of two ConvexShape in the frames HT0 and HT1 (exu.HT or 4x4 matrix) as a tuple (distance, point0, point1, normal): distance negative for a penetration (minus the penetration depth), point0 and point1 the closest points of the shapes (the deepest points of a penetration), normal the unit vector from shape 0 to shape 1, all global; by GJK on the cores (the sphere a point, the capsule a segment, their radii subtracted) and EPA where the cores overlap, to the rounding for spheres, capsules, boxes, triangles and points; a cylinder, whose curved side GJK and EPA approach slowly, is refined by alternating projections of the closest points (separated) and by moving shape 1 out along the normal (penetrating), to about 1e-12 of the size of the shapes when they are apart and 1e-7 within about 1e-6 of contact; with analytic=True a sphere and a cylinder by the closed formula',
                           argList=['shape0', 'HT0', 'shape1', 'HT1', 'analytic'], defaultArgs=['', '', '', '', 'true'],
                           argTypes=['ConvexShape', 'HT', 'ConvexShape', 'HT', 'bool'], returnType='tuple[float, NDArray, NDArray, NDArray]')

module.CppCode('        }\n        }\n')
module.DefFinishTable()
