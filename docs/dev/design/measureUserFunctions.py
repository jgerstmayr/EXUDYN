#measurement for docs/dev/design/pythonUserItems.md (revision2026c group RG7, #2971): the cost of a call into Python from the solver
#a chain of n mass points on coordinate springs, explicit RK4 (5 evaluations per step, no Jacobian), 3 variants:
#C++ spring, Python springForceUserFunction, symbolic springForceUserFunction; and ObjectGenericODE2 with a Python
#forceUserFunction for the whole chain (one call per evaluation), and one ObjectGenericODE2 per mass point (unconnected)
import time
import numpy as np
import exudyn as exu
from exudyn.utilities import *

n = 100
steps = 2000

def Model(variant):
    SC = exu.SystemContainer(); mbs = SC.AddSystem()
    mbs.variables['SC'] = SC
    ground = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=mbs.AddNode(NodePointGround()), coordinate=0))
    if variant == 'genericODE2':
        node = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=n, referenceCoordinates=[0]*n,
                                           initialCoordinates=[0.01]*n, initialCoordinates_t=[0]*n))
        K = np.diag([2.]*n) - np.diag([1.]*(n-1), 1) - np.diag([1.]*(n-1), -1)
        K *= 100
        def Force(mbs, t, itemNumber, q, q_t):
            return -K @ q
        mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node], massMatrix=np.eye(n), forceUserFunction=Force))
        mbs.Assemble()
        return (SC, mbs)
    if variant == 'genericODE2perMass':
        #every mass point an ObjectGenericODE2 of 1 coordinate with a Python force: the cost of a small Python body
        for i in range(n):
            nd = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=1, referenceCoordinates=[0],
                                             initialCoordinates=[0.01], initialCoordinates_t=[0]))
            def Force(mbs, t, itemNumber, q, q_t):
                return np.array([-100*q[0]])
            mbs.AddObject(ObjectGenericODE2(nodeNumbers=[nd], massMatrix=np.eye(1), forceUserFunction=Force))
        mbs.Assemble()
        return (SC, mbs)
    if variant in ['userBodyPerMass', 'userBodyPerMassSymbolic']:
        #every mass point an ObjectUserBody of 1 coordinate with a Python or symbolic force, the mass matrix stored
        #(constantMassMatrix): the cost of the arguments qRef, qTotal, q_t, parameters (#3056) against ObjectGenericODE2
        def UFmass(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
            return np.eye(1)
        def UFforce(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
            return -100*qTotal
        symbolicForce = None
        for i in range(n):
            nd = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=1, referenceCoordinates=[0],
                                             initialCoordinates=[0.01], initialCoordinates_t=[0]))
            body = mbs.AddObject(ObjectUserBody(nodeNumbers=[nd], massMatrixUserFunction=UFmass, forceUserFunction=UFforce,
                                                constantMassMatrix=True))
            if variant == 'userBodyPerMassSymbolic':
                if symbolicForce is None: #one recording for all bodies
                    symbolicForce = CreateSymbolicUserFunction(mbs, UFforce, 'forceUserFunction', body)
                    mbs.variables['keep'] = symbolicForce
                mbs.SetObjectParameter(body, 'forceUserFunction', symbolicForce)
        mbs.Assemble()
        return (SC, mbs)
    previous = ground
    esym = exu.symbolic
    for i in range(n):
        nd = mbs.AddNode(Node1D(referenceCoordinates=[0], initialCoordinates=[0.01]))
        mbs.AddObject(Mass1D(nodeNumber=nd, mass=1))
        m = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nd, coordinate=0))
        if variant == 'cpp':
            mbs.AddObject(CoordinateSpringDamper(markerNumbers=[previous, m], stiffness=100))
        elif variant == 'python':
            def UF(mbs, t, itemNumber, u, v, k, d, offset):
                return k*u
            mbs.AddObject(CoordinateSpringDamper(markerNumbers=[previous, m], stiffness=100, springForceUserFunction=UF))
        elif variant == 'symbolic':
            def UFs(mbs, t, itemNumber, u, v, k, d, offset):
                return k*u
            o = mbs.AddObject(CoordinateSpringDamper(markerNumbers=[previous, m], stiffness=100))
            symbolicUF = CreateSymbolicUserFunction(mbs, UFs, 'springForceUserFunction', o)
            mbs.SetObjectParameter(o, 'springForceUserFunction', symbolicUF)
        previous = m
    mbs.Assemble()
    return (SC, mbs)

results = {}
for variant in ['cpp', 'python', 'symbolic', 'genericODE2', 'genericODE2perMass', 'userBodyPerMass', 'userBodyPerMassSymbolic']:
    (SC, mbs) = Model(variant)
    s = exu.SimulationSettings()
    s.timeIntegration.numberOfSteps = steps
    s.timeIntegration.endTime = steps*1e-3
    s.timeIntegration.verboseMode = 0
    s.solution.file.write = False
    s.timeIntegration.solverType = exu.DynamicSolverType.RK44
    start = time.perf_counter()
    mbs.SolveDynamic(s)
    results[variant] = time.perf_counter() - start
calls = steps*5*n #RK44 evaluates 5 times per step here (counted)
print('n=%d, %d steps RK44: %d user function calls per spring variant' % (n, steps, calls))
for v, t in results.items():
    print('%-12s %8.3f s   %6.2f us per step and spring' % (v, t, t/(steps*n)*1e6))
print('Python user function: %.2f us per call more than C++' % ((results['python']-results['cpp'])/calls*1e6))
print('symbolic user function: %.2f us per call more than C++' % ((results['symbolic']-results['cpp'])/calls*1e6))
print('ObjectGenericODE2, one Python call per evaluation for all %d coordinates: %.1f us per call' % (n, results['genericODE2']/(steps*5)*1e6))
print('ObjectGenericODE2 per mass point (a small Python body): %.2f us per call more than C++' % ((results['genericODE2perMass']-results['cpp'])/calls*1e6))
print('ObjectUserBody per mass point, Python: %.2f us per call more than C++, %.2f us more than ObjectGenericODE2' %
      ((results['userBodyPerMass']-results['cpp'])/calls*1e6, (results['userBodyPerMass']-results['genericODE2perMass'])/calls*1e6))
print('ObjectUserBody per mass point, symbolic: %.2f us per call more than C++' % ((results['userBodyPerMassSymbolic']-results['cpp'])/calls*1e6))
