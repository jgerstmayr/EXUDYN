#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The binary solution file stores its numbers as float or as double, as simulationSettings.solution.precision
#           says (< 8: float, 4 bytes; >= 8: double, 8 bytes) - the precision of the files, not of the console. A mass
#           on a spring is solved twice, and the solution files read back: the double file gives the final state
#           of the solver exactly, the float file to about 7 digits, and it is half the size.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-04
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import numpy as np
import os

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()
oGround = mbs.CreateGround()
oMass = mbs.CreateMassPoint(referencePosition=[1, 0, 0], initialVelocity=[0, 2, 0], mass=1.3)
mbs.CreateSpringDamper(itemNumbers=[oGround, oMass], stiffness=1000, damping=0.7)
mbs.Assemble()
nNode = mbs.GetObject(oMass)['nodeNumber']

errors = 0
results = {}
for precision in [6, 12]:
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 100
    simulationSettings.timeIntegration.endTime = 0.1
    simulationSettings.timeIntegration.newton.useModifiedNewton = False #Just for the test; modified Newton is usually faster
    simulationSettings.solution.file.binary = True
    simulationSettings.solution.precision = precision
    simulationSettings.solution.file.name = 'solution/binarySolutionFileTest' + str(precision) + '.sol'
    mbs.SolveDynamic(simulationSettings)

    fileName = OutputFilePath(simulationSettings.solution.file.name) #where the solver wrote it
    solution = LoadSolutionFile(fileName, verbose=False)
    lastRow = solution['data'][-1, 1:4]                     #time, then the three coordinates of the node
    exact = mbs.GetNodeOutput(nNode, exu.OutputVariableType.Coordinates)
    results[precision] = (np.linalg.norm(lastRow - exact), os.path.getsize(fileName), lastRow)

(errorFloat, sizeFloat, rowFloat) = results[6]
(errorDouble, sizeDouble, rowDouble) = results[12]
exu.Print('binarySolutionFileTest: error float =', errorFloat, ', double =', errorDouble, '; sizes', sizeFloat, sizeDouble)
if not (errorDouble == 0 and 0 < errorFloat < 1e-5):
    errors += 1
if not sizeFloat < 0.75 * sizeDouble:
    errors += 1

u = np.sum(rowDouble) + errors
exu.Print('solution of binarySolutionFileTest=', u)

exu.sys['testResult'] = u
