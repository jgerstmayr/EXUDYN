#measurement for docs/dev/internals/userFunctions.md (revision2026c step RG12.3.1, #2996): the cost of one call of a
#user function at the C++/Python boundary, read from the solver timer userFunctions (the call with its conversions)
#(1) ObjectGenericODE2.forceUserFunction with n coordinates: q and q_t in, n values out - symbolic,
#    Python computing a list, Python returning a prepared list, Python returning a prepared numpy array
#(2) LoadCoordinate.loadUserFunction with scalar arguments only: symbolic and Python, the reference without vectors
import exudyn as exu
from exudyn.utilities import *
from exudyn.advancedUtilities import CreateSymbolicUserFunction
import numpy as np

k = 100.
prepared = {}
argumentType = {}

def ForcePython(mbs, t, itemNumber, q, q_t):
    argumentType['q'] = type(q).__name__
    return [-k*x for x in q]

def ForcePreparedList(mbs, t, itemNumber, q, q_t):
    return prepared['list']

def ForcePreparedNumpy(mbs, t, itemNumber, q, q_t):
    return prepared['numpy']

def ForceSymbolic(mbs, t, itemNumber, q, q_t):
    return -k*q

def LoadUF(mbs, t, load):
    return 0*t + load

def Settings(steps):
    s = exu.SimulationSettings()
    s.timeIntegration.numberOfSteps = steps
    s.timeIntegration.endTime = 0.001*steps
    s.timeIntegration.verboseMode = 0
    s.solution.file.write = False
    s.solution.sensors.active = False
    s.show.computationTime = True  #the solver timers run with it
    return s

def MicrosecondsPerCall(mbs, steps):
    mbs.SolveDynamic(Settings(steps), solverType=exu.DynamicSolverType.RK44, storeSolver=True)
    return mbs.sys['dynamicSolver'].timer.userFunctions/(4*steps)*1e6 #RK44: 4 evaluations per step

def ForceCall(n, variant, steps):
    SC = exu.SystemContainer(attachToRenderer=False)
    mbs = SC.AddSystem()
    node = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0]*n, initialCoordinates=[1.]*n, initialCoordinates_t=[0]*n,
                                       numberOfODE2Coordinates=n))
    prepared['list'] = [0.]*n
    prepared['numpy'] = np.zeros(n)
    o = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node], massMatrix=np.eye(n)))
    keep = None
    if variant == 'symbolic':
        keep = CreateSymbolicUserFunction(mbs, ForceSymbolic, 'forceUserFunction', o)
        mbs.SetObjectParameter(o, 'forceUserFunction', keep)
    else:
        mbs.SetObjectParameter(o, 'forceUserFunction', {'python': ForcePython, 'list': ForcePreparedList,
                                                        'numpy': ForcePreparedNumpy}[variant])
    mbs.Assemble()
    return MicrosecondsPerCall(mbs, steps)

def ScalarCall(symbolic, steps=5000):
    SC = exu.SystemContainer(attachToRenderer=False)
    mbs = SC.AddSystem()
    n = mbs.AddNode(Node1D(referenceCoordinates=[0], initialCoordinates=[1], initialVelocities=[0]))
    mbs.AddObject(Mass1D(nodeNumber=n, physicsMass=1))
    m = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n, coordinate=0))
    load = mbs.AddLoad(LoadCoordinate(markerNumber=m, load=1))
    keep = None
    if symbolic:
        keep = CreateSymbolicUserFunction(mbs, LoadUF, 'loadUserFunction', load)
        mbs.SetLoadParameter(load, 'loadUserFunction', keep)
    else:
        mbs.SetLoadParameter(load, 'loadUserFunction', LoadUF)
    mbs.Assemble()
    return MicrosecondsPerCall(mbs, steps)

if __name__ == '__main__':
    print('module', exu.config.Version(True))
    print('(1) forceUserFunction, microseconds per call, best of 3')
    print('      n  symbolic  py:compute  py:list  py:numpy')
    for n in [1, 5, 7, 10, 100, 1000]:
        steps = 5000 if n < 1000 else 500
        row = []
        for variant in ['symbolic', 'python', 'list', 'numpy']:
            row.append(min(ForceCall(n, variant, steps) for i in range(3)))
        print('%7d %9.3f %11.3f %8.3f %9.3f' % tuple([n] + row), flush=True)
    print('the Python function gets q as', argumentType.get('q'))
    print('(2) scalar loadUserFunction, microseconds per call: symbolic %.3f, Python %.3f'
          % (min(ScalarCall(True) for i in range(3)), min(ScalarCall(False) for i in range(3))))
