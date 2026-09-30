#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The graphics of every item, through its MiniExample (#2751): each MiniExample of
#           python/MiniExamples/ runs as it is; then every body that has graphicsData gets a small
#           one injected - a brick away from its reference point, lines and a text - so that each
#           body draws something with little data, and the model is assembled again. The graphics
#           data (SC.renderer.GetGraphicsData(), no window) is taken at the initial state and after
#           five steps of its own step size, reduced to a fingerprint (graphicsRegression.py) and
#           compared with the reference in graphicsReferences/miniExamples/. What the ground draws must
#           not move, and the corners of each drawn brick must be where the body's kinematics put its
#           local corners - the transformation of the drawing, checked directly. Nodes, markers, loads,
#           sensors and the world basis are drawn as well.
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

import numpy as np
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


#the injected brick: away from the reference point of the body and not symmetric to it, so that a missing
#translation or rotation of the drawing moves its corners
brickMin = np.array([0.04, 0.02, 0.01])
brickMax = np.array([0.16, 0.08, 0.04])
brickCorners = np.array([[x, y, z] for x in [brickMin[0], brickMax[0]] for y in [brickMin[1], brickMax[1]]
                         for z in [brickMin[2], brickMax[2]]])


def InjectedGraphics():
    """a little of each kind the graphics data has: triangles (a brick), lines and a text"""
    return [graphics.BrickXYZ(*brickMin, *brickMax, color=graphics.color.steelblue),
            graphics.Lines([[0, 0, 0], brickMin, [brickMax[0], brickMin[1], brickMin[2]]], color=graphics.color.red),
            graphics.Text(point=list(brickMax), text='G', color=graphics.color.black)]


def CheckBrickTransformations(SC, mbs, bodies):
    """the corners each body's brick is drawn at against its local corners mapped by the body's kinematics
    (Visualization configuration); returns lines that name what differs. A body whose kinematics cannot say
    where a local point is (no Position output) is left out"""
    data = SC.renderer.GetGraphicsData()
    triangles = data['triangles']
    lines = []
    for i in bodies:
        try:
            expected = np.array([mbs.GetObjectOutputBody(i, exu.OutputVariableType.Position, list(c),
                                                         exu.ConfigurationType.Visualization) for c in brickCorners])
        except Exception:
            continue
        selection = (triangles['items'][:, 1] == int(exu.ItemType.Object)) & (triangles['items'][:, 2] == i)
        drawn = np.asarray(triangles['points'])[selection].reshape(-1, 3)
        if len(drawn) == 0:
            lines.append('Object %d: no triangles drawn' % i)
            continue
        #every drawn vertex of the brick is one of the expected corners, and every corner is drawn
        distanceToCorner = np.min(np.linalg.norm(drawn[:, None, :] - expected[None, :, :], axis=2), axis=1)
        distanceToDrawn = np.min(np.linalg.norm(expected[:, None, :] - drawn[None, :, :], axis=2), axis=1)
        tolerance = 1e-5 * (1 + np.abs(expected).max())      #the graphics data are floats
        if distanceToCorner.max() > tolerance or distanceToDrawn.max() > tolerance:
            lines.append('Object %d (%s): brick drawn %.3g away from its corners' %
                         (i, mbs.GetObject(i)['objectType'], max(distanceToCorner.max(), distanceToDrawn.max())))
    return lines


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

    bodies = [i for i in range(mbs.systemData.NumberOfObjects()) if 'VgraphicsData' in mbs.GetObject(i)]
    for i in bodies:
        mbs.SetObjectParameter(i, 'VgraphicsData', InjectedGraphics())
    mbs.Assemble()
    for items in [SC.visualizationSettings.nodes, SC.visualizationSettings.markers,
                  SC.visualizationSettings.loads, SC.visualizationSettings.sensors]:
        items.show = True
    SC.visualizationSettings.view0.scene.drawWorldBasis = True
    initial = graphicsRegression.Fingerprint(SC.renderer.GetGraphicsData())
    wrong = CheckBrickTransformations(SC, mbs, bodies)

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
    wrong += CheckBrickTransformations(SC, mbs, bodies)
    assert wrong == [], '\n'.join(wrong)

    if initial['perItem']:
        moved = graphicsRegression.VariantDelta(initial, later)
        assert not (GroundKeys(mbs) & set(moved)), 'a ground object moved: ' + str(GroundKeys(mbs) & set(moved))

    case = 'miniExamples/' + fileName[:-3]
    differences = graphicsRegression.CheckVariantsAgainstReference(case, initial,
                                                                   {'after %d steps' % numberOfSteps: later})
    assert differences == [], '\n'.join(differences)
