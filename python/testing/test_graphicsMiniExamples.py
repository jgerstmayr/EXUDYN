#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The graphics of every item, through its MiniExample (#2751): each MiniExample of
#           python/MiniExamples/ runs as it is; then every body that has graphicsData gets a small
#           one injected - lines, a few triangles and a text - so that each body draws something
#           with little data, and the model is assembled again. The graphics data
#           (SC.renderer.GetGraphicsData(), no window) is taken at the initial state and after five
#           steps of its own step size, reduced to a fingerprint (graphicsRegression.py) and compared
#           with the reference in graphicsReferences/miniExamples/. What the ground draws must not
#           move. Nodes, markers, loads and sensors are drawn with their MiniExamples as well.
#
#           A new or changed reference: EXUDYN_RECORD_GRAPHICS_REFERENCES=1 pytest <this file>,
#           then look at the diff of python/testing/graphicsReferences/miniExamples/.
#
# Usage:    pytest python/testing/test_graphicsMiniExamples.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-30
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import sys

import pytest

import exudyn as exu
import exudyn.graphics as graphics

testingDir = os.path.dirname(os.path.abspath(__file__))
pythonDir = os.path.dirname(testingDir)
sys.path.insert(0, testingDir)
sys.path.insert(0, pythonDir)
import graphicsRegression                                                   # noqa: E402
from MiniExamples.miniExamplesFileList import miniExamplesFileList         # noqa: E402

exu.special.userInterface.SuppressAll(True)   #no window, whatever a MiniExample asks for
numberOfSteps = 5


def InjectedGraphics():
    """a little of each kind the graphics data has: lines, triangles (a tetrahedron) and a text"""
    s = 0.05
    return [graphics.Lines([[0, 0, 0], [s, 0, 0], [s, s, 0]], color=graphics.color.blue),
            graphics.FromPointsAndTrigs([[0, 0, 0], [s, 0, 0], [0, s, 0], [0, 0, s]],
                                        [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]], color=graphics.color.red),
            graphics.Text(point=[0, 0, s], text='G', color=graphics.color.black)]


def RunMiniExample(fileName):
    """(SC, mbs, simulationSettings) after the MiniExample of fileName ran as it is"""
    exu.config.printToConsole = False
    exu.sys['testIsActive'] = True
    namespace = {'__name__': '__mini__'}
    source = open(os.path.join(pythonDir, 'MiniExamples', fileName), encoding='utf-8').read()
    try:
        exec(compile(source, fileName, 'exec'), namespace)
    finally:
        exu.config.printToConsole = True
        exu.sys['testIsActive'] = False
    simulationSettings = namespace.get('simulationSettings', None)
    if not isinstance(simulationSettings, exu.SimulationSettings):
        simulationSettings = exu.SimulationSettings()
    return (namespace['SC'], namespace['mbs'], simulationSettings)


def GroundKeys(mbs):
    """the fingerprint keys of the ground objects, which must not move"""
    return {'Object ' + str(i) for i in range(mbs.systemData.NumberOfObjects())
            if mbs.GetObject(i)['objectType'] == 'Ground'}


@pytest.fixture
def outputDirectory(tmp_path):
    """what a MiniExample writes goes to tmp_path; exu.config is global, and the next test in this process
    must find it as it was"""
    previous = exu.config.outputDirectory
    exu.config.outputDirectory = str(tmp_path)
    yield
    exu.config.outputDirectory = previous


@pytest.mark.parametrize('fileName', miniExamplesFileList)
def test_miniExampleGraphics(fileName, outputDirectory):
    (SC, mbs, simulationSettings) = RunMiniExample(fileName)

    for i in range(mbs.systemData.NumberOfObjects()):
        if 'VgraphicsData' in mbs.GetObject(i):
            mbs.SetObjectParameter(i, 'VgraphicsData', InjectedGraphics())
    mbs.Assemble()
    for items in [SC.visualizationSettings.nodes, SC.visualizationSettings.markers,
                  SC.visualizationSettings.loads, SC.visualizationSettings.sensors]:
        items.show = True
    initial = graphicsRegression.Fingerprint(SC.renderer.GetGraphicsData())

    #five steps of the MiniExample's own step size
    ti = simulationSettings.timeIntegration
    stepSize = ti.endTime / max(1, ti.numberOfSteps)
    ti.numberOfSteps = numberOfSteps
    ti.endTime = numberOfSteps * stepSize
    simulationSettings.solutionSettings.writeSolutionToFile = False
    simulationSettings.solutionSettings.sensorsWritePeriod = ti.endTime
    simulationSettings.displayComputationTime = False
    simulationSettings.displayStatistics = False
    ti.verboseMode = 0
    exu.config.printToConsole = False
    try:
        mbs.SolveDynamic(simulationSettings)
    finally:
        exu.config.printToConsole = True
    later = graphicsRegression.Fingerprint(SC.renderer.GetGraphicsData(), perItem=initial['perItem'])

    if initial['perItem']:
        moved = graphicsRegression.VariantDelta(initial, later)
        assert not (GroundKeys(mbs) & set(moved)), 'a ground object moved: ' + str(GroundKeys(mbs) & set(moved))

    case = 'miniExamples/' + fileName[:-3]
    differences = graphicsRegression.CheckVariantsAgainstReference(case, initial,
                                                                   {'after %d steps' % numberOfSteps: later})
    assert differences == [], '\n'.join(differences)
