#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  exudyn.HT, the homogeneous transformation of Exudyn's C++ core (#2780), against the 4x4 numpy matrices
#           of exudyn.rigidBodyUtilities: for random rotations and translations, the composition H1*H2, the
#           transformed point H*v, the inverse, the 4x4 matrix in both directions, and the transformations set
#           without rotation (identity, SetTranslation), whose products skip the rotation.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-02
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

rng = np.random.default_rng(1)
errors = []
total = 0.
for i in range(20):
    A1 = RotXYZ2RotationMatrix(rng.standard_normal(3))
    A2 = RotXYZ2RotationMatrix(rng.standard_normal(3))
    p1 = rng.standard_normal(3)
    p2 = rng.standard_normal(3)
    v = rng.standard_normal(3)
    H1 = exu.HT(rotation=A1, translation=p1)
    H2 = exu.HT(rotation=A2, translation=p2)
    T1 = HomogeneousTransformation(A1, p1) #4x4 numpy matrices
    T2 = HomogeneousTransformation(A2, p2)
    errors += [np.abs((H1*H2).HT44() - T1 @ T2).max(),                       #composition
               np.abs(H1*v - (T1 @ np.append(v, 1))[0:3]).max(),            #transformed point
               np.abs(H1.Inverse().HT44() - InverseHT(T1)).max(),             #inverse
               np.abs(exu.HT(T1).HT44() - T1).max(),                          #from and to 4x4
               np.abs(H1.RotateVectorTransposed(H1.RotateVector(v)) - v).max()]
    total += np.abs((H1*H2).HT44()).sum()

#transformations without rotation: the flag, and the same results as with a unit matrix
HT0 = exu.HT()
Htranslation = exu.HT(translation=[1, 2, 3])
Hrotation = exu.HT(rotation=RotationMatrixZ(0.3))
exu.Print('without rotation:', HT0.HasNoRotation(), Htranslation.HasNoRotation(), Hrotation.HasNoRotation(),
          (Htranslation*HT0).HasNoRotation(), (Htranslation*Hrotation).HasNoRotation())
errors += [np.abs((Htranslation*Hrotation).HT44() - HTtranslate([1, 2, 3]) @ HTrotateZ(0.3)).max(),
           np.abs((Hrotation*Htranslation).HT44() - HTrotateZ(0.3) @ HTtranslate([1, 2, 3])).max(),
           np.abs(Htranslation.Inverse().HT44() - HTtranslate([-1, -2, -3])).max()]
flags = [HT0.HasNoRotation(), Htranslation.HasNoRotation(), not Hrotation.HasNoRotation(),
         (Htranslation*HT0).HasNoRotation(), not (Htranslation*Hrotation).HasNoRotation()]

exu.Print('largest difference to rigidBodyUtilities:', max(errors), ', flags as expected:', all(flags))
testResult = total + (max(errors) < 1e-12) + sum(flags)
exu.Print('solution of homogeneousTransformationTest=', testResult)
exu.sys['testResult'] = testResult
