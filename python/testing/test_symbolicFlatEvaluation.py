#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  The flat evaluation of symbolic user functions (#3053): a result made of scalar expressions is evaluated by
#           the list of operations built after the recording, each shared node once. It must give the same bits as the
#           evaluation of the tree (flatEvaluation=False), for single values, vectors and matrices of components, in a
#           simulation too; GetFlatProgram() gives the list as text.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-20 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or
#           modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import re

import numpy as np

import exudyn as exu
from exudyn.advancedUtilities import CreateSymbolicUserFunction
from exudyn.utilities import NodeGenericODE2, ObjectGenericODE2

esym = exu.symbolic


def Force(mbs, t, itemNumber, q, q_t):
    """a force with shared values on several levels, as the quadratic velocity vector of a rigid body"""
    c1, s1, c2, s2 = esym.cos(q[1]), esym.sin(q[1]), esym.cos(q[2]), esym.sin(q[2])
    omega = [c1*c2*q_t[0] + s2*q_t[1], -c1*s2*q_t[0] + c2*q_t[1], s1*q_t[0] + q_t[2]]
    jOmega = [1*omega[0], 2*omega[1], 3*omega[2]]
    cross = [omega[1]*jOmega[2] - omega[2]*jOmega[1], omega[2]*jOmega[0] - omega[0]*jOmega[2],
             omega[0]*jOmega[1] - omega[1]*jOmega[0]]
    return [-100*q[0] - (c1*c2*cross[0] - c1*s2*cross[1] + s1*cross[2]),
            -100*q[1] - (s2*cross[0] + c2*cross[1]),
            -100*q[2] - cross[2]]


def MassMatrix(mbs, t, itemNumber, q, q_t):
    """a matrix of scalar components, with constant zeros (a sparse result)"""
    c = esym.cos(q[1])
    return esym.Matrix([[2 + c*c, c, 0], [c, 2 + c*c, 0], [0, 0, 1 + esym.sin(q[2])**2]])


def SpringForce(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force):
    """a scalar with the functions, comparisons and IfThenElse"""
    x = deltaL + 0.25
    return (esym.IfThenElse(x > 0.2, stiffness*esym.sqrt(x*x + 0.01), -stiffness*x)
            + damping*deltaL_t + esym.atan2(x, 1.5)*esym.exp(-x*x) + esym.min(x, 0.3)*esym.max(deltaL_t, -0.1)
            + esym.abs(x - 0.4)*esym.sign(deltaL_t) + esym.mod(5*x, 0.7) + x**3 - esym.log(1 + x*x) + force)


def Model():
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    node = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0]*3, initialCoordinates=[0.01, 0.2, -0.1],
                                       initialCoordinates_t=[0.5, -0.3, 1.], numberOfODE2Coordinates=3))
    item = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node]))
    keep = [CreateSymbolicUserFunction(mbs, MassMatrix, 'massMatrixUserFunction', item),
            CreateSymbolicUserFunction(mbs, Force, 'forceUserFunction', item)]
    mbs.SetObjectParameter(item, 'massMatrixUserFunction', keep[0])
    mbs.SetObjectParameter(item, 'forceUserFunction', keep[1])
    spring = CreateSymbolicUserFunction(mbs, SpringForce, 'springForceUserFunction', itemTypeName='ObjectConnectorSpringDamper')
    return (SC, mbs, keep + [spring])


def Evaluations(function, arguments):
    """the result of a function for each set of arguments, as the flat program and as the tree computes it"""
    function.flatEvaluation = True
    flat = [np.array(function.Evaluate(*args)) for args in arguments]
    function.flatEvaluation = False
    tree = [np.array(function.Evaluate(*args)) for args in arguments]
    function.flatEvaluation = True
    return (flat, tree)


def testTheFlatEvaluationGivesTheBitsOfTheTree():
    (SC, mbs, (mass, force, spring)) = Model()
    rng = np.random.default_rng(1)
    vectors = [(mbs, 0., 0, list(rng.uniform(-2, 2, 3)), list(rng.uniform(-2, 2, 3))) for i in range(25)]
    scalars = [(mbs, 0., 0, rng.uniform(-1, 1), rng.uniform(-1, 1), 100., 2., 0.5) for i in range(50)]
    for (function, arguments) in [(force, vectors), (mass, vectors), (spring, scalars)]:
        (flat, tree) = Evaluations(function, arguments)
        for (a, b) in zip(flat, tree):
            assert a.tobytes() == b.tobytes(), str(function)


def testTheFlatProgramComputesEachSharedNodeOnce():
    (SC, mbs, (mass, force, spring)) = Model()
    text = force.GetFlatProgram()
    operations = int(re.match(r'#\w+: (\d+) operations, 3 results', text).group(1))
    lines = text.strip().split('\n')
    assert len([line for line in lines if re.match(r't\d+ = ', line)]) == operations
    assert [line.split(' =')[0] for line in lines[-3:]] == ['result[0]', 'result[1]', 'result[2]']
    assert len([line for line in lines if line.endswith('= cos(q[1])')]) == 1     #recorded once, computed once
    #the tree evaluates the shared values at every use, as its text shows them
    assert str(force).count('cos(q[1])') > 5
    assert 'result[2,2] = ' in mass.GetFlatProgram()


def testASimulationGivesTheBitsOfTheTree():
    """the solver through the item: the force as a vector, the sparse mass matrix; the same solution with both"""
    solutions = []
    for flatEvaluation in [True, False]:
        (SC, mbs, functions) = Model()
        for function in functions:
            function.flatEvaluation = flatEvaluation
        mbs.Assemble()
        settings = exu.SimulationSettings()
        settings.timeIntegration.numberOfSteps = 200
        settings.timeIntegration.endTime = 1
        settings.timeIntegration.verboseMode = 0
        settings.solution.file.write = False
        mbs.SolveDynamic(settings)
        solutions.append(np.array(mbs.systemData.GetODE2Coordinates()))
    assert solutions[0].tobytes() == solutions[1].tobytes()
