#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A model without coordinates - a ground object moved by a PreStepUserFunction - writes a
#           solution file with the time column only; LoadSolutionFile reads it as a matrix with one column,
#           as text and as binary file, and SetSolutionState, which the SolutionViewer calls for every
#           frame, works with it (#2950).
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-10
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.basicUtilities import LoadSolutionFile, OutputFilePath, SetSolutionState
import exudyn.graphics as graphics
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround = mbs.CreateGround(graphicsDataList=[graphics.Brick(size=[0.3, 0.2, 0.1])])

def PreStepUserFunction(mbs, t):
    mbs.SetObjectParameter(oGround, 'referenceHT', exu.HT(translation=[t, 0, 0]))
    return True
mbs.SetPreStepUserFunction(PreStepUserFunction)
mbs.Assemble()

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 10
simulationSettings.timeIntegration.endTime = 1
simulationSettings.solution.file.write = True
simulationSettings.solution.file.writePeriod = 0.1

u = 0
for binary in [False, True]:
    simulationSettings.solution.file.binary = binary
    simulationSettings.solution.file.name = 'solution/withoutCoordinates' + ('Binary' if binary else '')
    mbs.SolveDynamic(simulationSettings)

    sol = LoadSolutionFile(OutputFilePath(simulationSettings.solution.file.name + ('.sol' if binary else '.txt')),
                           verbose=False)
    exu.Print('binary' if binary else 'text', 'solution:', sol['data'].shape, 'columns', sol['nColumns'])
    for row in range(sol['nRows']):
        SetSolutionState(mbs, sol, row, exu.ConfigurationType.Visualization)
    u += sol['data'].shape[0] + 10*sol['data'].shape[1] + np.sum(sol['data'][:, 0]) #11 rows, 1 column, times 0..1

exu.Print('solution of solutionFileWithoutCoordinatesTest=', u)

exu.sys['testResult'] = u
