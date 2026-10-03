#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  A user who stops a simulation - by closing the render window, pressing Escape, or with
#           SC.renderer.StopSimulation() - is not a solver failure (#2616), before the simulation
#           starts or while it runs. The renderer thread sets the same flags from a key press or a
#           closed window; SC.renderer.StopSimulation() sets them from Python, which is what makes
#           this testable without a window (#2674).
#
# Usage:    pytest python/testing/test_stopSimulation.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-28
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu


def MassPointModel():
    """one mass point under gravity: its x coordinate stays 0, its y coordinate falls"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oMass = mbs.CreateMassPoint(referencePosition=[0, 0, 0], mass=1, gravity=[0, -9.81, 0])
    mbs.Assemble()
    return (SC, mbs, mbs.GetObject(oMass)['nodeNumber'])


def Settings(endTime=1.):
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.endTime = endTime
    simulationSettings.timeIntegration.numberOfSteps = 100
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.solution.file.write = False
    simulationSettings.show.computationTime = False
    simulationSettings.show.statistics = False
    return simulationSettings


def testQuittingBeforeTheStartComputesNothingAndRaisesNothing():
    """what #2616 fixed: SolveDynamic returns quietly, and the system stays where it was"""
    (SC, mbs, node) = MassPointModel()
    SC.renderer.StopSimulation()                            #as closing the render window does
    assert mbs.SolveDynamic(Settings()) is True             #no SolverError, no failure block
    assert mbs.systemData.GetTime() == 0.
    assert mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[1] == 0.
    assert mbs.GetRenderEngineStopFlag()


def testTheFlagIsResetBySetRenderEngineStopFlag():
    """the way to continue after a quit, which the solver's note names"""
    (SC, mbs, node) = MassPointModel()
    SC.renderer.StopSimulation()
    mbs.SetRenderEngineStopFlag(False)
    mbs.SolveDynamic(Settings())
    assert abs(mbs.systemData.GetTime() - 1.) < 1e-12
    assert mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[1] < -4.


def testStoppingWhileRunningEndsAfterTheStep():
    """from a user function, as a key press does from the renderer thread: the simulation ends
    early and without an error"""
    (SC, mbs, node) = MassPointModel()

    def PreStep(mbs, t):
        if t >= 0.3:
            SC.renderer.StopSimulation(forceQuit=False)
        return True

    mbs.SetPreStepUserFunction(PreStep)
    assert mbs.SolveDynamic(Settings()) is True
    assert 0.3 <= mbs.systemData.GetTime() < 0.5


def testAStopWithoutForceQuitDoesNotStopTheNextSimulation():
    """the running simulation only: the next one starts with the flag reset by the solver"""
    (SC, mbs, node) = MassPointModel()
    SC.renderer.StopSimulation(forceQuit=False)
    mbs.SolveDynamic(Settings())
    assert abs(mbs.systemData.GetTime() - 1.) < 1e-12
