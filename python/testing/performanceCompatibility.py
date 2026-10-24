#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  The performance models of today with an Exudyn older than the renames of 1.12.256 (#3080), for
#           runPerformanceTests.py --tolerant: the new names of the models are given the old names of the
#           installed version.
#           - item parameters (mass -> physicsMass, ...) and the arguments of the Create functions
#             (itemNumbers -> bodyNumbers, ...): read from the docstrings of the repository's itemInterface.py
#             ('physicsMass: deprecated since ...: use mass') and mainSystemExtensions.py ('bodyNumbers:
#             deprecated name of itemNumbers'); the classes and functions of the installed version are wrapped;
#           - the simulation settings (simulationSettings.solution.file.write -> ...solutionSettings.writeSolutionToFile,
#             ...): the paths the performance models use, replaced in their source before it runs.
#           A model that uses something the older version does not have still raises, and is skipped.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-24 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or
#           modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import re
import inspect
import functools

packageDir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'exudyn')
renameVersion = [1, 12, 256]    #the simulation settings and item parameters were renamed in 1.12.256 and 1.12.258

#the new paths of the simulation settings the performance models use, and their names before 1.12.256 (from the
#deprecated entries of definitions/structureDefsSimulationSettings.py); longer paths first
settingsRenames = [
    ('.solution.file.export.velocities', '.solutionSettings.exportVelocities'),
    ('.solution.file.export.accelerations', '.solutionSettings.exportAccelerations'),
    ('.solution.file.writePeriod', '.solutionSettings.solutionWritePeriod'),
    ('.solution.file.write', '.solutionSettings.writeSolutionToFile'),
    ('.solution.file.name', '.solutionSettings.coordinatesSolutionFileName'),
    ('.solution.file.information', '.solutionSettings.solutionInformation'),
    ('.solution.sensors.writePeriod', '.solutionSettings.sensorsWritePeriod'),
    ('.solution.sensors.active', '.solutionSettings.sensorsStoreAndWriteFiles'),
    ('.solution.recordImagesInterval', '.solutionSettings.recordImagesInterval'),
    ('.solution.precision', '.solutionSettings.outputPrecision'),
    ('.show.computationTime', '.displayComputationTime'),
    ('.show.statistics', '.displayStatistics'),
    ('.show.globalTimers', '.displayGlobalTimers'),
    ('.linearSolver.solverType', '.linearSolverType'),
    ('.timeIntegration.explicit.', '.timeIntegration.explicitIntegration.'),
]


def VersionKey(version):
    return [int(part) if part.isdigit() else 0 for part in version.split('.')[0:3]]


def NeedsCompatibility(version):
    """whether an installed version has the names before the renames of 1.12.256"""
    return VersionKey(version) < renameVersion


def RenamesFromDocstrings(fileName, definitionPattern, renamePattern):
    """{function or class name: {new name: old name}} from the docstrings of a file of the repository"""
    with open(os.path.join(packageDir, fileName), encoding='utf-8') as file:
        text = file.read()
    renames = {}
    parts = re.split(definitionPattern, text)
    for i in range(1, len(parts) - 1, 2):
        pairs = {new: old for (old, new) in re.findall(renamePattern, parts[i + 1])}
        if len(pairs):
            renames[parts[i]] = pairs
    return renames


def Renamed(function, renames, acceptedNames):
    """function called with the new names of its arguments, which it receives with the old ones"""
    @functools.wraps(function)
    def Wrapper(*args, **kwargs):
        for (new, old) in renames.items():
            if new in kwargs and new not in acceptedNames:
                kwargs[old] = kwargs.pop(new)
        return function(*args, **kwargs)
    return Wrapper


def ApplyRenames(exu):
    """wrap the item classes and the Create functions of an installed older version; returns the number wrapped"""
    import exudyn.itemInterface as itemInterface
    count = 0
    itemRenames = RenamesFromDocstrings('itemInterface.py', r'\nclass (\w+)\b',
                                        r'(\w+): deprecated since [\d.]+, removed in \d+: use (\w+)')
    for (className, renames) in itemRenames.items():
        itemClass = getattr(itemInterface, className, None)
        if itemClass is not None:
            accepted = inspect.signature(itemClass.__init__).parameters
            itemClass.__init__ = Renamed(itemClass.__init__, renames, accepted)
            count += 1
    createRenames = RenamesFromDocstrings('misc/mainSystemExtensions.py', r'\ndef MainSystem(\w+)\(',
                                          r'(\w+): deprecated name of (\w+)')
    for (functionName, renames) in createRenames.items():
        function = getattr(exu.MainSystem, functionName, None)
        if function is not None:
            accepted = inspect.signature(function).parameters
            setattr(exu.MainSystem, functionName, Renamed(function, renames, accepted))
            count += 1
    return count


def TranslateSource(source):
    """the paths of the simulation settings in the source of a model, with the names before 1.12.256"""
    for (new, old) in settingsRenames:
        source = re.sub(re.escape(new) + (r'\b' if not new.endswith('.') else ''), old, source)
    return source
