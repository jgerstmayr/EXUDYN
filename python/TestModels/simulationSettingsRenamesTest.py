#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The simulation settings renamed and restructured in Exudyn 1.13 (#2813): every old name still works,
#           with a DeprecationWarning, and reaches its new place - written under the old name, read under the
#           new one, and the other way round; every use is counted in exu.sys['deprecationUse']; the old keys of
#           a dictionary given to SetDictionary reach their new place too. The list of
#           names is checked against the declarations by python/testing/test_checkDeprecations.py, so that it
#           cannot miss one.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-04
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import warnings

testIsActive = exu.sys.get('testIsActive', False)

#old name -> new name, both from simulationSettings
renames = {
    'solutionSettings.coordinatesSolutionFileName': 'solution.file.name',
    'solutionSettings.writeSolutionToFile': 'solution.file.write',
    'solutionSettings.writeFileHeader': 'solution.file.writeHeader',
    'solutionSettings.writeFileFooter': 'solution.file.writeFooter',
    'solutionSettings.writeInitialValues': 'solution.file.writeInitialValues',
    'solutionSettings.solutionWritePeriod': 'solution.file.writePeriod',
    'solutionSettings.binarySolutionFile': 'solution.file.binary',
    'solutionSettings.appendToFile': 'solution.file.append',
    'solutionSettings.flushFilesImmediately': 'solution.flushFilesImmediately',
    'solutionSettings.flushFilesDOF': 'solution.file.flushAboveCoordinates',
    'solutionSettings.exportAccelerations': 'solution.file.export.accelerations',
    'solutionSettings.exportAlgebraicCoordinates': 'solution.file.export.algebraicCoordinates',
    'solutionSettings.exportDataCoordinates': 'solution.file.export.dataCoordinates',
    'solutionSettings.exportODE1Velocities': 'solution.file.export.ODE1Velocities',
    'solutionSettings.exportVelocities': 'solution.file.export.velocities',
    'solutionSettings.outputPrecision': 'solution.precision',
    'solutionSettings.writeRestartFile': 'solution.restart.write',
    'solutionSettings.restartFileName': 'solution.restart.name',
    'solutionSettings.restartWritePeriod': 'solution.restart.writePeriod',
    'solutionSettings.sensorsAppendToFile': 'solution.sensors.append',
    'solutionSettings.sensorsWriteFileHeader': 'solution.sensors.writeHeader',
    'solutionSettings.sensorsWriteFileFooter': 'solution.sensors.writeFooter',
    'solutionSettings.sensorsWritePeriod': 'solution.sensors.writePeriod',
    'solutionSettings.sensorsStoreAndWriteFiles': 'solution.sensors.active',
    'solutionSettings.solutionInformation': 'solution.file.information',
    'solutionSettings.solverInformationFileName': 'solution.solverInformationFileName',
    'solutionSettings.recordImagesInterval': 'solution.recordImagesInterval',
    'linearSolverSettings.pivotThreshold': 'linearSolver.pivotThreshold',
    'linearSolverSettings.ignoreSingularJacobian': 'linearSolver.ignoreSingularJacobian',
    'linearSolverSettings.reuseAnalyzedPattern': 'linearSolver.reuseAnalyzedPattern',
    'linearSolverSettings.showCausingItems': 'linearSolver.showCausingItems',
    'linearSolverType': 'linearSolver.solverType',
    'displayComputationTime': 'show.computationTime',
    'displayGlobalTimers': 'show.globalTimers',
    'displayStatistics': 'show.statistics',
    'outputPrecision': 'consolePrecision',
    'timeIntegration.explicitIntegration.dynamicSolverType': 'timeIntegration.solverType',
    'timeIntegration.explicitIntegration.eliminateConstraints': 'timeIntegration.explicit.eliminateConstraints',
    'timeIntegration.explicitIntegration.useLieGroupIntegration': 'timeIntegration.explicit.useLieGroupIntegration',
    'timeIntegration.explicitIntegration.computeEndOfStepAccelerations': 'timeIntegration.explicit.computeEndOfStepAccelerations',
    'timeIntegration.explicitIntegration.computeMassMatrixInversePerBody': 'timeIntegration.explicit.computeMassMatrixInversePerBody',
    'timeIntegration.simulateInRealtime': 'timeIntegration.realtime.active',
    'timeIntegration.realtimeFactor': 'timeIntegration.realtime.factor',
    'timeIntegration.realtimeWaitMicroseconds': 'timeIntegration.realtime.waitMicroseconds',
    'timeIntegration.newton.newtonResidualMode': 'timeIntegration.newton.residualMode',
    'timeIntegration.newton.useNewtonSolver': 'timeIntegration.newton.active',
    'staticSolver.newton.newtonResidualMode': 'staticSolver.newton.residualMode',
    'staticSolver.newton.useNewtonSolver': 'staticSolver.newton.active',
    'timeIntegration.newton.numericalDifferentiation.forODE2connectors': 'timeIntegration.newton.numericalDifferentiation.forODE2Connectors',
    'staticSolver.constrainODE1coordinates': 'staticSolver.constrainODE1Coordinates',
    }

def Get(settings, path):
    for name in path.split('.'):
        settings = getattr(settings, name)
    return settings

def Set(settings, path, value):
    names = path.split('.')
    for name in names[:-1]:
        settings = getattr(settings, name)
    setattr(settings, names[-1], value)

def Other(value):
    """a value different from the given one, of the same type"""
    if isinstance(value, bool):
        return not value
    if isinstance(value, int):
        return value + 1
    if isinstance(value, float):
        return value * 0.5 + 0.125
    if isinstance(value, str):
        return value + 'X'
    if isinstance(value, exu.DynamicSolverType):
        return exu.DynamicSolverType.RK44 if value != exu.DynamicSolverType.RK44 else exu.DynamicSolverType.DOPRI5
    if isinstance(value, exu.LinearSolverType):
        return exu.LinearSolverType.EigenSparse if value != exu.LinearSolverType.EigenSparse else exu.LinearSolverType.EXUdense
    raise ValueError('no other value for ' + str(value))

def Uses():
    return sum(exu.sys.get('deprecationUse', {}).get('simulationSettings', {}).values())

errors = 0
used = Uses()
for (old, new) in renames.items():
    settings = exu.SimulationSettings()
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', DeprecationWarning)
        value = Other(Get(settings, new))
        Set(settings, old, value)              #written under the old name, read under the new one
        if Get(settings, new) != value:
            errors += 1
            exu.Print('simulationSettingsRenamesTest: writing', old, 'does not reach', new)
        value = Other(value)
        Set(settings, new, value)              #and the other way round
        if Get(settings, old) != value:
            errors += 1
            exu.Print('simulationSettingsRenamesTest: reading', old, 'does not give', new)
nUses = Uses() - used
exu.Print('simulationSettingsRenamesTest:', len(renames), 'old names,', nUses, 'uses recorded, errors', errors)
if nUses != 2 * len(renames):
    errors += 1

#a dictionary of an earlier Exudyn, given to SetDictionary: each old key reaches its new place, and the keys that are
#not given keep their values (RG12.34.7)
def NestedDictionary(path, value):
    d = value
    for name in reversed(path.split('.')):
        d = {name: d}
    return d

for (old, new) in renames.items():
    settings = exu.SimulationSettings()
    settings.timeIntegration.endTime = 7.5     #not in the dictionary: stays
    value = Other(Get(settings, new))
    settings.SetDictionary(NestedDictionary(old, value))
    if Get(settings, new) != value or settings.timeIntegration.endTime != 7.5:
        errors += 1
        exu.Print('simulationSettingsRenamesTest: the dictionary key', old, 'does not reach', new)

#the same for the visualization settings, with three of their renamed keys
SC = exu.SystemContainer()
SC.visualizationSettings.SetDictionary({'window': {'renderWindowSize': [333, 222]}, 'general': {'drawWorldBasis': True},
                                        'openGL': {'light0position': [1., 2., 3., 0.]}})
visualizationSettings = SC.visualizationSettings
if (list(visualizationSettings.view0.window.renderWindowSize) != [333, 222] or not visualizationSettings.view0.scene.drawWorldBasis
        or list(visualizationSettings.openGL.light0.position) != [1., 2., 3., 0.]):
    errors += 1
    exu.Print('simulationSettingsRenamesTest: a renamed key of the visualization settings does not reach its new place')

u = len(renames) + errors
exu.Print('solution of simulationSettingsRenamesTest=', u)

exu.sys['testResult'] = u
