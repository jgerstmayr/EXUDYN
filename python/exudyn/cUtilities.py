#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  exudyn.cUtilities: the kinematics functions of the C++ code of Exudyn (rotation matrices, Euler parameters,
#           Tait-Bryan angles, rotation vectors, skew matrices, basis vectors, the twist of a frame), the same bits on every
#           platform. The functions are compiled (the submodule exudyn._cUtilities, src/Pymodules/PyCUtilities.h,
#           declared in definitions/pybindCUtilities.py, which also writes their documentation and cUtilities.pyi); this
#           module takes
#           them. exudyn.rigidBodyUtilities calls them under the same names. Documented in the Python-C++
#           interface, page Exudyn module functions.
#
# Authors:  Johannes Gerstmayr
# Date:     2026-10-11
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from exudyn import _compiledModule

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'ComputeOrthonormalBasisVectors', 'ComputeOrthonormalBasis', 'GramSchmidt', 'SymmetricBasis',
    'SymmetricBasisRotationJacobian', 'Skew', 'Skew2Vec',
    'EulerParameters2G', 'EulerParameters2GLocal', 'EulerParameters2RotationMatrix',
    'RotationMatrix2EulerParameters', 'AngularVelocity2EulerParameters_t', 'RotationVector2RotationMatrix',
    'RotationMatrix2RotationVector', 'ComputeRotationAxisFromRotationVector', 'ExpSO3', 'LogSO3',
    'RotXYZ2RotationMatrix', 'RotationMatrix2RotXYZ', 'RotXYZ2G', 'RotXYZ2G_t', 'RotXYZ2GLocal', 'RotXYZ2GLocal_t',
    'AngularVelocity2RotXYZ_t', 'RotXYZ2EulerParameters', 'RotationMatrix2RotZYZ', 'RotationMatrixX',
    'RotationMatrixY', 'RotationMatrixZ', 'RotationMatrix2D', 'PointVelocity', 'LocalTwist', 'RelativeTwist',
    'KinematicChain', 'ConvexShape', 'Distance',
    ]

_cUtilities = _compiledModule._cUtilities

ComputeOrthonormalBasisVectors = _cUtilities.ComputeOrthonormalBasisVectors
ComputeOrthonormalBasis = _cUtilities.ComputeOrthonormalBasis
GramSchmidt = _cUtilities.GramSchmidt
SymmetricBasis = _cUtilities.SymmetricBasis
SymmetricBasisRotationJacobian = _cUtilities.SymmetricBasisRotationJacobian
Skew = _cUtilities.Skew
Skew2Vec = _cUtilities.Skew2Vec
EulerParameters2G = _cUtilities.EulerParameters2G
EulerParameters2GLocal = _cUtilities.EulerParameters2GLocal
EulerParameters2RotationMatrix = _cUtilities.EulerParameters2RotationMatrix
RotationMatrix2EulerParameters = _cUtilities.RotationMatrix2EulerParameters
AngularVelocity2EulerParameters_t = _cUtilities.AngularVelocity2EulerParameters_t
RotationVector2RotationMatrix = _cUtilities.RotationVector2RotationMatrix
RotationMatrix2RotationVector = _cUtilities.RotationMatrix2RotationVector
ComputeRotationAxisFromRotationVector = _cUtilities.ComputeRotationAxisFromRotationVector
ExpSO3 = _cUtilities.ExpSO3
LogSO3 = _cUtilities.LogSO3
RotXYZ2RotationMatrix = _cUtilities.RotXYZ2RotationMatrix
RotationMatrix2RotXYZ = _cUtilities.RotationMatrix2RotXYZ
RotXYZ2G = _cUtilities.RotXYZ2G
RotXYZ2G_t = _cUtilities.RotXYZ2G_t
RotXYZ2GLocal = _cUtilities.RotXYZ2GLocal
RotXYZ2GLocal_t = _cUtilities.RotXYZ2GLocal_t
AngularVelocity2RotXYZ_t = _cUtilities.AngularVelocity2RotXYZ_t
RotXYZ2EulerParameters = _cUtilities.RotXYZ2EulerParameters
RotationMatrix2RotZYZ = _cUtilities.RotationMatrix2RotZYZ
RotationMatrixX = _cUtilities.RotationMatrixX
RotationMatrixY = _cUtilities.RotationMatrixY
RotationMatrixZ = _cUtilities.RotationMatrixZ
RotationMatrix2D = _cUtilities.RotationMatrix2D
PointVelocity = _cUtilities.PointVelocity
LocalTwist = _cUtilities.LocalTwist
RelativeTwist = _cUtilities.RelativeTwist
KinematicChain = _cUtilities.KinematicChain #the kinematics of a robot (#3259)
ConvexShape = _cUtilities.ConvexShape #convex shapes and their distance (#3317)
Distance = _cUtilities.Distance
