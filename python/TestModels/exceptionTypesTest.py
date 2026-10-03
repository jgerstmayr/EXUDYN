#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  What an Exudyn error looks like from Python.
#           Ten things a user can get wrong - a bad item number, a parameter of the wrong type, a
#           size that does not fit, a feature an item does not have, a model the solver cannot
#           solve, and a Python user function that raises - each provoked on purpose, caught, and
#           reported with the exception CLASS it arrived as and the message str(exception) carries.
#
#           It is a test model and a record at the same time. As a test it checks one thing only:
#           every case must raise SOMETHING. The classes and the messages are printed rather than
#           asserted: pinning a class or a wording here would mean editing this file whenever
#           one of them is improved, and the log of a test run already shows what they were
#           on that day.
#
# Usage:    Run it as it is and read the table.
#           To see an error the way an IDE shows it - the traceback in Spyder or VS Code - set
#           catchErrors = False below, and singleCase to the number of the case to let fly.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-18
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the two switches this file exists for: with catchErrors=False the selected case is NOT caught, so
#the exception reaches the IDE and is shown the way a user sees it
catchErrors = True
singleCase = 0      #which case to run uncaught (1-based); 0 means "all of them", caught

cases = []          #(name, what the user did wrong, callable)


def Case(name, whatTheUserDidWrong):
    """collect a case; the decorated function provokes exactly one error"""
    def Decorate(function):
        cases.append((name, whatTheUserDidWrong, function))
        return function
    return Decorate


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#a small system every case can use; nothing here is meant to fail
SC = exu.SystemContainer()
mbs = SC.AddSystem()
nodeNumber = mbs.AddNode(NodePoint(referenceCoordinates=[0, 0, 0]))
objectNumber = mbs.AddObject(MassPoint(physicsMass=1, nodeNumber=nodeNumber))
markerNumber = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodeNumber, coordinate=0))
mbs.Assemble()


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the cases: bad input first, the solver and the user function last, because an error inside the
#solver sets the global error flag and what happens after it is no longer a clean measurement

@Case('item number out of range', 'asks for object 999 in a system that has one object')
def ObjectNumberOutOfRange():
    mbs.GetObject(999)


@Case('output variable index', 'asks for the output of a node that does not exist')
def NodeNumberOutOfRange():
    mbs.GetNodeOutput(999, exu.OutputVariableType.Position)


#the two cases that ADD an item build their own system: an item added to the assembled mbs above
#would leave it inconsistent, and the later cases would then measure that instead of themselves
@Case('parameter of the wrong type', 'writes a string into a parameter that is a number')
def ParameterOfWrongType():
    scratch = exu.SystemContainer().AddSystem()
    scratchNode = scratch.AddNode(NodePoint(referenceCoordinates=[0, 0, 0]))
    scratch.AddObject(MassPoint(physicsMass='heavy', nodeNumber=scratchNode))


@Case('parameter of the wrong value', 'gives a 3D reference position two components')
def ParameterOfWrongValue():
    scratch = exu.SystemContainer().AddSystem()
    scratch.AddNode(NodePoint(referenceCoordinates=[1, 2]))


@Case('object that is not a matrix', 'hands a list to something that wants a scipy sparse matrix')
def NotAMatrix():
    exu.MatrixContainer().SetWithSparseMatrix([1, 2, 3])


@Case('vector of the wrong size', 'writes five coordinates into a system that has three')
def VectorOfWrongSize():
    mbs.systemData.SetODE2Coordinates([1, 2, 3, 4, 5])


@Case('output an item does not have', 'asks a mass point for a strain')
def OutputVariableNotAvailable():
    mbs.GetObjectOutput(objectNumber, exu.OutputVariableType.StrainLocal)


@Case('marker that does not exist', 'loads marker 999 and assembles')
def MarkerDoesNotExist():
    localSC = exu.SystemContainer()
    localMbs = localSC.AddSystem()
    localNode = localMbs.AddNode(NodePoint(referenceCoordinates=[0, 0, 0]))
    localMbs.AddObject(MassPoint(physicsMass=1, nodeNumber=localNode))
    localMbs.AddLoad(LoadCoordinate(markerNumber=999, load=1))
    localMbs.Assemble()


@Case('a system the solver cannot solve', 'runs a static solution of a body that is not held')
def SolverCannotSolve():
    localSC = exu.SystemContainer()
    localMbs = localSC.AddSystem()
    localNode = localMbs.AddNode(NodePoint(referenceCoordinates=[0, 0, 0]))
    localMbs.AddObject(MassPoint(physicsMass=1, nodeNumber=localNode))
    localMarker = localMbs.AddMarker(MarkerNodeCoordinate(nodeNumber=localNode, coordinate=0))
    localMbs.AddLoad(LoadCoordinate(markerNumber=localMarker, load=10))
    localMbs.Assemble()

    simulationSettings = exu.SimulationSettings()
    simulationSettings.solution.file.write = False
    simulationSettings.staticSolver.verboseMode = 0
    localMbs.SolveStatic(simulationSettings)


@Case('a user function that raises', 'divides by zero inside a load user function')
def UserFunctionRaises():
    localSC = exu.SystemContainer()
    localMbs = localSC.AddSystem()
    localNode = localMbs.AddNode(NodePoint(referenceCoordinates=[0, 0, 0]))
    localMbs.AddObject(MassPoint(physicsMass=1, nodeNumber=localNode))
    localMarker = localMbs.AddMarker(MarkerNodeCoordinate(nodeNumber=localNode, coordinate=0))

    def LoadUserFunction(mbs, t, load):
        if t > 0.02:
            return load / 0.0          #the user's own mistake, inside the user's own function
        return load

    localMbs.AddLoad(LoadCoordinate(markerNumber=localMarker, load=1,
                                    loadUserFunction=LoadUserFunction))
    localMbs.Assemble()

    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 10
    simulationSettings.timeIntegration.endTime = 0.05
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.solution.file.write = False
    localMbs.SolveDynamic(simulationSettings)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#run them
if not catchErrors:
    #the point of this branch: no try/except anywhere, so the exception travels all the way out and
    #the IDE shows it with its traceback
    number = singleCase if singleCase > 0 else 1
    (name, whatTheUserDidWrong, function) = cases[number - 1]
    exu.Print('case ' + str(number) + ': ' + name + ' (' + whatTheUserDidWrong + ')')
    function()
    exu.Print('NOTHING WAS RAISED')
    exu.sys['testResult'] = -1
else:
    results = []
    silent = 0
    for (number, (name, whatTheUserDidWrong, function)) in enumerate(cases, start=1):
        try:
            function()
            results.append((number, name, 'NOTHING RAISED', ''))
            silent += 1
        except BaseException as exception:
            message = str(exception).replace('\n', ' | ')
            #an Exudyn exception carries the error that caused it as __cause__ - the object,
            #with its traceback - and not only its words (#2537)
            cause = exception.__cause__
            if cause is not None:
                message = '[__cause__ ' + type(cause).__name__ + ': ' + str(cause) + '] ' + message
            results.append((number, name, type(exception).__name__, message))

    exu.Print('')
    exu.Print('exceptionTypesTest: what a user gets, ' + str(len(cases)) + ' cases')
    exu.Print('-' * 110)
    for (number, name, className, message) in results:
        exu.Print('%2d  %-32s %-22s %s' % (number, name, className, message[:62]))
    exu.Print('-' * 110)

    #the ONLY thing this model asserts: every case must raise something. Which class and which
    #message are printed above (#2527, #2528)
    exu.sys['testResult'] = silent
    exu.Print('exceptionTypesTest: cases that raised nothing = ' + str(silent))
