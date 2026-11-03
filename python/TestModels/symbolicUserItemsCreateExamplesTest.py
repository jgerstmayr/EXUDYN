#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The examples of the Create functions of MainSystem with the user items of UserItemCreator: the Example of
#           the docstring of each Create function that UserItemCreator has (exudyn.misc.mainSystemExtensions) is run as
#           it is, with the built-in items, and once more with creator = UserItemCreator(mbs) in place of mbs for the
#           Create functions and the user functions recorded before mbs.Assemble(); the rigid bodies of both on
#           NodeRigidBodyRxyz, the node of the user rigid body. The positions and velocities of the nodes at the end
#           of the simulation are compared, and the CPU times of both solvers printed.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-03
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import exudyn.misc.mainSystemExtensions as mainSystemExtensions
from exudyn.symbolics.userItems import UserItemCreator
import inspect, re, time
import numpy as np

def ExampleCode(createFunction):
    """the code of the Example of the docstring of MainSystem.<createFunction>, None if it has none that runs"""
    doc = inspect.getdoc(getattr(mainSystemExtensions, 'MainSystem' + createFunction))
    if 'Example:' not in doc:
        return None
    lines = doc[doc.index('Example:') + len('Example:'):].split('\n')[1:]
    code = []
    for line in lines:
        if line.strip() and not line.startswith('    '):
            break #the next section of the docstring
        code.append(line[4:])
    code = '\n'.join(code)
    if 'mbs.Assemble()' not in code:
        return None
    #the user rigid body is on NodeRigidBodyRxyz only, so both runs take it
    return code.replace('mbs.CreateRigidBody(', 'mbs.CreateRigidBody(nodeType=exu.NodeType.RotationRxyz, ')

def UserItemsCode(code):
    """the example with the Create functions of UserItemCreator, recorded before mbs.Assemble()"""
    code = code.replace('mbs = SC.AddSystem()', 'mbs = SC.AddSystem()\ncreator = UserItemCreator(mbs)\nmbs.variables["creator"] = creator', 1)
    code = re.sub(r'\bmbs\.Create', 'creator.Create', code)
    code = code.replace("['nodeNumber']", "['nodeNumbers'][0]") #the node of a user mass point, an ObjectUserBody
    return code.replace('mbs.Assemble()', 'creator.Record()\nmbs.Assemble()', 1)

def Run(code):
    """run an example; its nodes' positions and velocities at the end, and the CPU time of the solver"""
    space = {'UserItemCreator': UserItemCreator}
    source = code.replace('mbs.SolveDynamic(', 'startTime = time.perf_counter()\nmbs.SolveDynamic(')
    exec(source + '\nsolverTime = time.perf_counter() - startTime\n', {'time': time, **space}, space)
    mbs = space['mbs']
    state = []
    for node in range(mbs.systemData.NumberOfNodes()):
        if mbs.GetNode(node)['nodeType'] in ['Point', 'RigidBodyEP', 'RigidBodyRxyz', 'RigidBodyRotVecLG']:
            for variable in [exu.OutputVariableType.Position, exu.OutputVariableType.Velocity]:
                state += list(mbs.GetNodeOutput(node, variable))
    return (np.array(state), space['solverTime'])

printToConsole = exu.config.printToConsole
exu.config.printToConsole = False #the solvers of the examples
failed = 0
(results, refused, withoutExample) = ([], [], [])
for createFunction in sorted(name for name in dir(UserItemCreator) if name.startswith('Create')):
    code = ExampleCode(createFunction)
    if code is None:
        withoutExample.append(createFunction)
        continue
    (stateBuiltIn, timeBuiltIn) = Run(code)
    try:
        (stateUserItems, timeUserItems) = Run(UserItemsCode(code))
    except ValueError as e: #an argument the user item does not support, which it refuses
        if 'not supported by the user item' not in str(e):
            raise
        refused.append((createFunction, str(e)))
        continue
    difference = np.max(np.abs(stateUserItems - stateBuiltIn)) if len(stateUserItems) == len(stateBuiltIn) else np.inf
    results.append((createFunction, difference, timeBuiltIn, timeUserItems))
    failed += int(not difference < 1e-8)
exu.config.printToConsole = printToConsole

exu.Print('example of                    difference   built-in   user items')
for (createFunction, difference, timeBuiltIn, timeUserItems) in results:
    exu.Print('{:29s}{:10.2e}{:9.3f} s{:9.3f} s'.format(createFunction, difference, timeBuiltIn, timeUserItems))
for (createFunction, reason) in refused:
    exu.Print('{:29s}refused: {}'.format(createFunction, reason))
exu.Print('without an example that runs a simulation:', ', '.join(withoutExample))
#the examples the user items refuse, so that a new refusal is seen
failed += int([createFunction for (createFunction, reason) in refused] != ['CreateDistanceConstraint'])

u = failed
exu.Print('solution of symbolicUserItemsCreateExamplesTest=', u)
exu.sys['testResult'] = u
