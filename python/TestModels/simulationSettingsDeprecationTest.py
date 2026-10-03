#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A renamed simulation setting keeps working under its old name, with a DeprecationWarning that names
#           the new one (#2588): parallel.multithreadedLLimitLoads (and ...Residuals, ...Jacobians,
#           ...MassMatrices) is parallel.multithreadedLowerLimitLoads now. Writing the old name writes the new one,
#           reading it reads the new one - also in a copy of the settings and in settings constructed on their own.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-03
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import copy
import warnings

testIsActive = exu.sys.get('testIsActive', False)

errors = 0
total = 0
for (k, name) in enumerate(['Loads', 'Residuals', 'Jacobians', 'MassMatrices']):
    simulationSettings = exu.SimulationSettings()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter('always')
        setattr(simulationSettings.parallel, 'multithreadedLLimit' + name, 30 + k) #the old name
        oldValue = getattr(simulationSettings.parallel, 'multithreadedLLimit' + name)
    newValue = getattr(simulationSettings.parallel, 'multithreadedLowerLimit' + name)
    warned = [str(entry.message) for entry in caught if issubclass(entry.category, DeprecationWarning)]
    if newValue != 30 + k or oldValue != 30 + k:
        errors += 1
    if len(warned) != 2 or ('parallel.multithreadedLowerLimit' + name) not in warned[0]:
        errors += 1
    total += newValue

#a copy has its own values, and the old name forwards in it as well
simulationSettings = exu.SimulationSettings()
settingsCopy = copy.copy(simulationSettings)
with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    settingsCopy.parallel.multithreadedLLimitLoads = 7
if settingsCopy.parallel.multithreadedLowerLimitLoads != 7 or simulationSettings.parallel.multithreadedLowerLimitLoads == 7:
    errors += 1

exu.Print('simulationSettingsDeprecationTest: errors', errors)
u = total + errors
exu.Print('solution of simulationSettingsDeprecationTest=', u)

exu.sys['testResult'] = u
