#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  A multithreaded solve that fails - a user function raises - stops its worker threads (#2776). They
#           were left running: a following multithreaded solve or the raytracer used them, and the process did not
#           end - the hang of a pytest worker. Each case runs in a process of its own, which must end in time.
#
# Usage:    pytest python/testing/test_solverThreads.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-04
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import subprocess
import sys

import pytest

script = r'''
import sys
import exudyn as exu
from exudyn.utilities import *
after = sys.argv[1]
SC = exu.SystemContainer()
mbs = SC.AddSystem()
oGround = mbs.CreateGround()
oMass = mbs.CreateMassPoint(referencePosition=[1,0,0], mass=1)
def UFspring(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force):
    if t > 0.1: raise ValueError('the user function fails')
    return stiffness*deltaL
mbs.CreateSpringDamper(bodyNumbers=[oGround, oMass], stiffness=10, springForceUserFunction=UFspring)
mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.parallel.numberOfThreads = 4
simulationSettings.timeIntegration.numberOfSteps = 100
simulationSettings.timeIntegration.verboseMode = 0
try:
    mbs.SolveDynamic(simulationSettings)
except exu.ModelError:
    print('the solve failed')
if after == 'raytracer':
    SC.visualizationSettings.raytracer.numberOfThreads = 4
    SC.visualizationSettings.view0.window.renderWindowSize = [40, 40]
    SC.renderer.RedrawAndGetImage(useRaytracer=True)
elif after == 'solve':
    mbs.SetObjectParameter(2, 'springForceUserFunction', 0) #the spring-damper, now without the failing function
    mbs.SolveDynamic(simulationSettings)
print('done')
'''


@pytest.mark.parametrize('after', ['nothing', 'raytracer', 'solve'])
def testAFailedMultithreadedSolveStopsItsThreads(after, tmp_path):
    environment = dict(os.environ, EXUDYN_SUPPRESS_UI_WINDOW_OPEN='1', EXUDYN_OUTPUTDIRECTORY=str(tmp_path))
    environment.pop('PYTHONPATH', None)
    result = subprocess.run([sys.executable, '-c', script, after], env=environment, capture_output=True, text=True,
                            timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'the solve failed' in result.stdout and 'done' in result.stdout
