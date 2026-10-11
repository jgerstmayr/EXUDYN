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

module.CppCode('        }\n        }\n')
module.DefFinishTable()
