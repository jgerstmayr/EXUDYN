#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A deprecated function or argument of the Python library warns like the deprecations of the C++
#           side (#2807): a DeprecationWarning at the line of the script that uses it, once per session
#           and name - every time with exu.special.deprecations.warnOnce = False -, and each use counted
#           in exu.sys['deprecationUse']['library']. graphics.BrickXYZ is a deprecated function,
#           bodyList of the Create functions a deprecated argument; graphics.Brick, which uses BrickXYZ
#           inside, is not deprecated and records nothing.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-03
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
import numpy as np
import warnings
import os

testIsActive = exu.sys.get('testIsActive', False)

def Uses(name):
    return exu.sys.get('deprecationUse', {}).get('library', {}).get(name, 0)

def FromThisScript(caught):
    """the warnings point at this script, not into the package"""
    return all(os.sep + 'exudyn' + os.sep not in entry.filename for entry in caught)

errors = 0
deprecations = exu.special.deprecations
warnOnceStored = deprecations.warnOnce

#a deprecated function
used = Uses('graphics.BrickXYZ')
graphics.Brick(centerPoint=[0,0,0], size=[1,1,1]) #uses BrickXYZ inside: no use of a deprecated name
if Uses('graphics.BrickXYZ') != used:
    errors += 1
nWarnings = []
for warnOnce in [True, False]:
    deprecations.warnOnce = warnOnce
    deprecations.Reset()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter('always')
        for i in range(3):
            graphics.BrickXYZ(0, 0, 0, 1, 1, 1)
    caught = [entry for entry in caught if issubclass(entry.category, DeprecationWarning)]
    nWarnings.append(len(caught))
    if not FromThisScript(caught) or any('graphics.BrickXYZ is deprecated' not in str(entry.message) for entry in caught):
        errors += 1
if nWarnings != [1, 3] or Uses('graphics.BrickXYZ') != used + 6:
    errors += 1

#a deprecated argument, reported for the Create function that was called
SC = exu.SystemContainer()
mbs = SC.AddSystem()
oGround = mbs.CreateGround()
b0 = mbs.CreateMassPoint(referencePosition=[1,0,0], mass=1)
used = Uses('MainSystem.CreateSpringDamper.bodyList')
deprecations.warnOnce = False
with warnings.catch_warnings(record=True) as caught:
    warnings.simplefilter('always')
    mbs.CreateSpringDamper(bodyList=[oGround, b0], stiffness=100)
caught = [entry for entry in caught if issubclass(entry.category, DeprecationWarning)]
deprecations.warnOnce = warnOnceStored
exu.Print('bodyList:', [str(entry.message) for entry in caught])
if len(caught) != 1 or not FromThisScript(caught) or Uses('MainSystem.CreateSpringDamper.bodyList') != used + 1:
    errors += 1

exu.Print('libraryDeprecationTest: warnings', nWarnings, ', errors', errors)
u = sum(nWarnings) + errors
exu.Print('solution of libraryDeprecationTest=', u)

exu.sys['testResult'] = u
