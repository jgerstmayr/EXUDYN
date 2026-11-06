#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  The solvers of the systems of exudyn.symbolics.systemModel (#3112, #3126): SolveExplicit integrates a
#           SymbolicSystemBase - a SymbolicSystem or a NumpySystem - by explicit Euler, RK44 or velocity Verlet, as
#           Exudyn's explicit solver does, SolveGeneralizedAlpha by the generalized-alpha method, as Exudyn's implicit
#           solver does, with ODE1 coordinates and index-3 constraints (#3134); both take the SimulationSettings of
#           Exudyn, write the solution file and the sensors as SolveDynamic does, and write the state into mbs for a
#           SymbolicSystem. With data coordinates, both compute a step again from its start while the post-Newton step
#           of the system gives an error above discontinuous.iterationTolerance, as CSolverBase::DiscontinuousIteration
#           (#3196); with timeIntegration.adaptiveStep, a step whose post-Newton step recommends a smaller step size, or
#           whose discontinuous iteration does not converge, is repeated with a smaller step, which grows again after a
#           few easy steps, as CSolverBase::SolveSteps (#3220); the statistics count the discontinuousIterations and the
#           rejectedSteps. SolveExplicit eliminates coordinates fixed by algebraic equations, as Exudyn's explicit solver
#           eliminates CoordinateConstraints to the ground (#3238). The output is written at the exact times of the write
#           periods, interpolated within the step (#3241).
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-26 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import warnings
import exudyn
import exudyn.cMath as cMath
from exudyn.symbolics.systemModel import _denseMassMatrixSize
try:
    import scipy.linalg
    import scipy.sparse
    import scipy.sparse.linalg
except ImportError:
    raise ImportError('exudyn.symbolics.solvers needs scipy for its sparse matrices: pip install scipy')

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = ['explicitSolverTypes', 'SolveExplicit', 'SolveGeneralizedAlpha']


#the explicit solvers of SolveExplicit
explicitSolverTypes = [exudyn.DynamicSolverType.ExplicitEuler, exudyn.DynamicSolverType.RK44,
                       exudyn.DynamicSolverType.VelocityVerlet]

#the Butcher tableaus (A, weights, times) of src/Solver/CSolverExplicit.cpp
_butcherTableaus = {exudyn.DynamicSolverType.ExplicitEuler: ([[0.]], [1.], [0.]),
                    exudyn.DynamicSolverType.RK44: ([[0., 0., 0., 0.], [0.5, 0., 0., 0.], [0., 0.5, 0., 0.], [0., 0., 1., 0.]],
                                                    [1./6., 1./3., 1./3., 1./6.], [0., 0.5, 0.5, 1.])}




class _MassMatrixSolver:
    """the accelerations a = M^-1 f of a system: by the inverse per body, or by the LU decomposition of M, dense up to
    _denseMassMatrixSize coordinates, else sparse; the inverse or the decomposition computed once for a constant mass
    matrix"""
    def __init__(self, system, perBody, reuseConstant):
        self.system = system
        self.perBody = perBody
        self.dense = system.nODE2 <= _denseMassMatrixSize
        self.constant = reuseConstant and system.IsMassMatrixConstant()
        self.stored = None
        self.evaluations = 0
        if perBody:
            ltg = np.concatenate([block[0].ravel() for block in system.MassMatrixBlocks()])
            if len(ltg) != len(np.unique(ltg)):
                raise ValueError('SolveExplicit: explicit.computeMassMatrixInversePerBody needs bodies that share no '
                                 'coordinates, but two bodies share a coordinate')

    def _Inverse(self, q, q_t, t):
        self.evaluations += 1
        if self.perBody:
            return [(ltg, np.linalg.inv(blocks)) for (ltg, blocks) in self.system.MassMatrixBlocks(q, q_t, t)]
        if self.dense:
            return scipy.linalg.lu_factor(self.system.MassMatrix(q, q_t, t))
        return scipy.sparse.linalg.splu(self.system.MassMatrix(q, q_t, t, sparse=True).tocsc())

    def Accelerations(self, q, q_t, t, f):
        inverse = self.stored if self.stored is not None else self._Inverse(q, q_t, t)
        if self.constant:
            self.stored = inverse
        if not self.perBody:
            return scipy.linalg.lu_solve(inverse, f) if self.dense else inverse.solve(f)
        a = np.zeros(len(f))
        for (ltg, inverses) in inverse:
            a[ltg] = np.einsum('kij,kj->ki', inverses, f[ltg])
        return a


def _Interpolate(start, end, t):
    """the state at t within a step from start to end, dicts of 'time', 'q', 'q_t', 'q_tt' and optionally 'y', 'y_t',
    'z' (#3241): q by the cubic Hermite interpolation of q and q_t at both ends, q_t its derivative, the others linear"""
    (t0, t1) = (start['time'], end['time'])
    h = t1 - t0
    s = (t - t0)/h
    (h00, h10, h01, h11) = (2*s**3 - 3*s**2 + 1, s**3 - 2*s**2 + s, -2*s**3 + 3*s**2, s**3 - s**2)
    (d00, d10, d01, d11) = (6*s**2 - 6*s, 3*s**2 - 4*s + 1, -6*s**2 + 6*s, 3*s**2 - 2*s)
    state = {'time': t,
             'q': h00*start['q'] + (h10*h)*start['q_t'] + h01*end['q'] + (h11*h)*end['q_t'],
             'q_t': (d00/h)*start['q'] + d10*start['q_t'] + (d01/h)*end['q'] + d11*end['q_t']}
    for name in ['q_tt', 'y', 'y_t', 'z']:
        state[name] = None if end.get(name) is None else (1 - s)*start[name] + s*end[name]
    return state


class _Output:
    """the output of a solver of this module: the states at the times of solution.file.writePeriod - the start, the
    multiples of the write period and the end -, within a step by _Interpolate (#3241); for a SymbolicSystem the output
    through its mbs: an Exudyn solver object writes the solution file and the sensors at the times of their write
    periods, interpolated within the step as well, and updates the renderer at the end of each step in FinishStep, by
    simulationSettings.solution; it runs as explicit Euler without inverse per body, which only decides what its
    initialization computes. The data coordinates within a step are those of its end"""
    def __init__(self, system, simulationSettings, tStart, tEnd, h):
        (self.tStart, self.tEnd, self.h) = (tStart, tEnd, h)
        self.period = simulationSettings.solution.file.writePeriod
        self.written = 0 #the write times passed, tStart + written*period
        self.previous = None
        self.result = {'time': [], 'q': [], 'q_t': [], 'q_tt': []}
        if system.nODE1:
            self.result.update({'y': [], 'y_t': []})
        if system.nAE:
            self.result['z'] = []
        self.system = system
        self.solver = None
        if getattr(system, 'mbs', None) is not None:
            self.simulationSettings = exudyn.SimulationSettings()
            self.simulationSettings.SetDictionary(simulationSettings.GetDictionary())
            if system.nAE: #Exudyn's explicit solver refuses the joints; the implicit one writes the same output
                self.simulationSettings.timeIntegration.solverType = exudyn.DynamicSolverType.GeneralizedAlpha
                self.solver = exudyn.MainSolverImplicitSecondOrder()
            else:
                self.simulationSettings.timeIntegration.solverType = exudyn.DynamicSolverType.ExplicitEuler
                self.simulationSettings.timeIntegration.explicit.computeMassMatrixInversePerBody = False
                self.solver = exudyn.MainSolverExplicit()
            self.solver.InitializeSolver(system.mbs, self.simulationSettings)
            #the state vectors of mbs, written in place, without the conversion of SetODE2Coordinates (#3173); valid
            #as long as mbs is not assembled again
            self.state = system.mbs.systemData.GetSystemStateDict(reference=True)
            self.writeFile = self.solver.output.writeToSolutionFile
            self.writeSensors = system.mbs.systemData.NumberOfSensors() > 0 and simulationSettings.solution.sensors.active

    def _Record(self, state):
        for name in self.result:
            self.result[name].append(np.copy(state[name]))

    def _SetState(self, state):
        """the state into mbs"""
        for (name, value) in [('ODE2Coords', state['q']), ('ODE2Coords_t', state['q_t']), ('ODE2Coords_tt', state['q_tt']),
                              ('ODE1Coords', state['y']), ('ODE1Coords_t', state['y_t']), ('AECoords', state['z']),
                              ('dataCoords', self.system.data)]:
            if value is not None and len(value):
                self.state[name][:] = value
        self.system.mbs.systemData.SetTime(state['time'])
        self.solver.it.currentTime = state['time']

    def Step(self, stepIndex, t, q, q_t, q_tt, last=False, y=None, y_t=None, z=None, h=None):
        """the state after step stepIndex at t, of the step size h (None: the constant one); stepIndex 0 is the initial
        state, as SolveDynamic writes it; the write times within the step interpolated from the previous state"""
        if h is not None:
            self.h = h
        state = {'time': t, 'q': q, 'q_t': q_t, 'q_tt': q_tt, 'y': y, 'y_t': y_t, 'z': z}
        if stepIndex == 0:
            self._Record(state)
            self.written = 1
        else:
            while self.tStart + self.written*self.period < t - 1e-10:
                self._Record(_Interpolate(self.previous, state, self.tStart + self.written*self.period))
                self.written += 1
            if last or self.tStart + self.written*self.period <= t + 1e-10:
                self._Record(state)
                while self.tStart + self.written*self.period <= t + 1e-10:
                    self.written += 1
        if self.solver is not None:
            output = self.solver.output
            while stepIndex != 0: #the write times of the solution file and the sensors within the step; each writer
                #writes if its write time is passed and advances it by its period
                times = ([output.lastSolutionWritten] if self.writeFile else []) + ([output.lastSensorsWritten] if self.writeSensors else [])
                if not times or min(times) >= t - 1e-10:
                    break
                self._SetState(_Interpolate(self.previous, state, min(times)))
                if self.writeFile:
                    self.solver.WriteCoordinatesToFile(self.system.mbs, self.simulationSettings)
                if self.writeSensors:
                    self.solver.WriteSensorsToFile(self.system.mbs, self.simulationSettings)
            self._SetState(state)
            self.solver.it.currentStepIndex = stepIndex
            self.solver.it.currentStepSize = self.h
            self.solver.FinishStep(self.system.mbs, self.simulationSettings)
        self.previous = {name: (None if value is None else np.copy(value)) for (name, value) in state.items()}

    def Finalize(self, statistics):
        """the result dict, with the statistics of the solver"""
        if self.solver is not None:
            self.solver.FinalizeSolver(self.system.mbs, self.simulationSettings)
        result = {name: np.array(value) for (name, value) in self.result.items()}
        result.update(statistics)
        return result


def _Steps(simulationSettings):
    """the number of steps, the start and end time and the step size of timeIntegration"""
    timeIntegration = simulationSettings.timeIntegration
    n = int(timeIntegration.numberOfSteps)
    (tStart, tEnd) = (timeIntegration.startTime, timeIntegration.endTime)
    return (n, tStart, tEnd, (tEnd - tStart)/n)


class _StepSize:
    """the step size of a solver of this module, as CSolverBase::SolveSteps and the step size functions of
    CSolverImplicitSecondOrderTimeInt and CSolverExplicitTimeInt (#3220): with timeIntegration.adaptiveStep the time
    advances by the step size, a rejected step is repeated from its start with min(0.75 h, h_recommended), or with
    h adaptiveStepDecrease (explicit: h/2) without a recommendation, down to minimumStepSize, and after
    adaptiveStepRecoverySteps steps of at most adaptiveStepRecoveryIterations Newton and discontinuous iterations the
    step grows by adaptiveStepIncrease (explicit: 2) up to the nominal (endTime - startTime)/numberOfSteps; without
    adaptiveStep the steps are of the nominal size"""
    def __init__(self, simulationSettings, explicit):
        timeIntegration = simulationSettings.timeIntegration
        (self.n, self.tStart, self.tEnd, self.hMax) = _Steps(simulationSettings)
        self.adaptive = timeIntegration.adaptiveStep
        self.useRecommended = self.adaptive and timeIntegration.discontinuous.useRecommendedStepSize
        self.explicit = explicit
        self.hMin = timeIntegration.minimumStepSize
        (self.decrease, self.increase) = (timeIntegration.adaptiveStepDecrease, timeIntegration.adaptiveStepIncrease)
        (self.recoverySteps, self.recoveryIterations) = (timeIntegration.adaptiveStepRecoverySteps,
                                                         timeIntegration.adaptiveStepRecoveryIterations)
        (self.h, self.t, self.stepIndex, self.sinceReduction, self.rejected) = (self.hMax, self.tStart, 0, 0, 0)

    def Done(self):
        return self.t >= self.tEnd - 1e-10 if self.adaptive else self.stepIndex >= self.n

    def Next(self):
        """the start time, end time and size of the next step, as UpdateCurrentTime"""
        if not self.adaptive:
            return (self.tStart + self.stepIndex*self.h, self.tStart + (self.stepIndex + 1)*self.h, self.h)
        if self.t + self.h > self.tEnd:
            self.h = self.tEnd - self.t
        return (self.t, self.t + self.h, self.h)

    def Reduce(self, recommendedStepSize):
        """a smaller step after a rejected one, as ReduceStepSize; False at the minimum step size"""
        if not self.adaptive or self.h <= self.hMin:
            return False
        if recommendedStepSize != -1:
            self.h = min(0.75*self.h, recommendedStepSize) #0.75: some reduction, which keeps the iteration from sticking
        else:
            self.h = 0.5*self.h if self.explicit else self.decrease*self.h
        self.h = max(self.hMin, self.h)
        (self.sinceReduction, self.rejected) = (0, self.rejected + 1)
        return True

    def Accept(self, t, iterations):
        """the step to t done after iterations Newton and discontinuous iterations; the step grows again, as
        IncreaseStepSize"""
        (self.t, self.stepIndex) = (t, self.stepIndex + 1)
        if not self.adaptive:
            return
        self.sinceReduction += 1
        if self.sinceReduction >= self.recoverySteps and iterations <= self.recoveryIterations:
            self.h = min(self.hMax, (2. if self.explicit else self.increase)*self.h)
            self.sinceReduction = 0


def _FixedCoordinates(system, q, q_t, y, t, rows, epsilon=1e-6, tolerance=1e-8):
    """the ODE2 coordinates fixed by the algebraic equations, which an explicit solver eliminates as Exudyn's explicit
    solver eliminates CoordinateConstraints to the ground (#3238): every equation linear in one ODE2 coordinate and
    independent of everything else - its velocity, the ODE1 coordinates, the multipliers, the time

    Args:
        system: the SymbolicSystemBase
        q: the initial ODE2 coordinates
        q_t: the initial ODE2 velocities
        y: the initial ODE1 coordinates
        t: the start time
        rows: the algebraic equations to eliminate
        epsilon: the step of the central differences of the constraint Jacobian
        tolerance: an entry of the Jacobian below tolerance times its largest entry is zero

    Returns:
        (indices, values): the fixed coordinates and the values that fulfil their equations
    """
    nAE = system.nAE
    z = np.zeros(nAE)
    def C(q, q_t=q_t, y=y, z=z, t=t):
        return np.asarray(system.AlgebraicEquations(q, q_t, y, z, t), dtype=float)
    c0 = C(q)
    C_q = np.zeros((nAE, system.nODE2))
    for j in range(system.nODE2):
        dq = np.zeros(system.nODE2)
        dq[j] = epsilon
        C_q[:, j] = (C(q + dq) - C(q - dq))/(2*epsilon)
    scale = max(1., np.max(np.abs(c0)), np.max(np.abs(C_q)))
    random = np.random.default_rng(1)
    dq = random.uniform(-1., 1., system.nODE2)
    variations = [C(q + dq) - (c0 + C_q @ dq), #linear in q
                  C(q, q_t=q_t + random.uniform(-1., 1., len(q_t))) - c0,
                  C(q, y=y + random.uniform(-1., 1., len(y))) - c0,
                  C(q, z=random.uniform(-1., 1., nAE)) - c0,
                  C(q, t=t + 1.) - c0]
    indices = []
    for i in rows:
        columns = np.flatnonzero(np.abs(C_q[i]) > tolerance*scale)
        if len(columns) != 1 or max(abs(variation[i]) for variation in variations) > 1e3*tolerance*scale:
            raise ValueError('SolveExplicit: an explicit solver eliminates only algebraic equations that fix one ODE2 '
                             'coordinate each (linear in it and in nothing else, as a coordinate constraint to the ground; '
                             'timeIntegration.explicit.eliminateConstraints), and equation ' + str(i) + ' does not; use '
                             'SolveGeneralizedAlpha')
        indices.append(int(columns[0]))
    if len(set(indices)) != len(indices):
        raise ValueError('SolveExplicit: a coordinate is fixed by two algebraic equations')
    (indices, rows) = (np.array(indices, dtype=int), np.array(rows, dtype=int))
    values = q[indices] - c0[rows]/C_q[rows, indices]
    return (indices, values)


def _ProjectEulerParameters(nodes, qRef, q, q_t):
    """the Euler parameters of nodes onto |p| = 1 and their velocities onto p^T p_t = 0, in place (#3238)"""
    for node in nodes:
        ltg = node['ltg']
        p = qRef[ltg] + q[ltg]
        p = p/cMath.norm(p)
        q[ltg] = p - qRef[ltg]
        q_t[ltg] -= (p @ q_t[ltg])*p


def _RefuseLieGroupNodes(system, useLieGroupIntegration, where):
    """Lie group nodes are integrated by Exudyn with the Lie group formulas, which are not implemented here"""
    mbs = getattr(system, 'mbs', None)
    if mbs is not None and useLieGroupIntegration:
        for nodeNumber in range(mbs.systemData.NumberOfNodes()):
            if mbs.GetNode(nodeNumber)['nodeType'].endswith('LG'):
                raise ValueError(where + ': node ' + str(nodeNumber) + ' is a Lie group node, which Exudyn integrates '
                                 'by the Lie group formulas (useLieGroupIntegration); they are not implemented here')


def SolveExplicit(system, simulationSettings=None, solverType=None):
    """the explicit time integration of a SymbolicSystemBase by explicit Euler, RK44 or velocity Verlet, as Exudyn's
    explicit solver computes them (#3112): the steps from
    timeIntegration.startTime, endTime and numberOfSteps, smaller with adaptiveStep where the post-Newton step of data
    coordinates rejects a step (#3220); the accelerations by the inverse per body with
    timeIntegration.explicit.computeMassMatrixInversePerBody, else by the sparse LU decomposition of the mass
    matrix, both once for a constant mass matrix (timeIntegration.reuseConstantMassMatrix); ODE1 coordinates in the
    stages of explicit Euler and RK44, with zero initial velocities, as Exudyn (#3134); nodes with Euler parameters: the
    accelerations with their constraint on acceleration level, 2 p^T a + 2 p_t^T p_t = 0, in [M C^T; C 0], and the
    parameters projected onto |p| = 1 and their velocities onto p^T p_t = 0 after each step, so that the norm holds and
    energy and momentum drift in the order of the error of the method (#3238); algebraic equations that fix
    one ODE2 coordinate each - linear in it and in nothing else, as a coordinate constraint to the ground - are
    eliminated with timeIntegration.explicit.eliminateConstraints, as Exudyn's explicit solver eliminates
    CoordinateConstraints to the ground: the coordinate keeps the value of its equation, its velocity and acceleration
    are zero, and the other accelerations come from the mass matrix without its rows and columns; other algebraic
    equations are refused, and all of them are ignored with a warning with eliminateConstraints=False, as Exudyn does
    (#3238); for a SymbolicSystem the solution file, the sensors and the renderer of its mbs by
    simulationSettings.solution, as after SolveDynamic

    Args:
        system: the system, a SymbolicSystemBase
        simulationSettings: exudyn.SimulationSettings; None: the defaults
        solverType: an exudyn.DynamicSolverType of explicitSolverTypes; None: timeIntegration.solverType

    Returns:
        dict with 'time' (n_out), 'q', 'q_t', 'q_tt' (n_out x nODE2) - with ODE1 coordinates 'y', 'y_t' (n_out x
        nODE1) - at the steps of solution.file.writePeriod and the last step, and the statistics 'steps',
        'forceEvaluations', 'massMatrixEvaluations', 'discontinuousIterations', 'rejectedSteps'
    """
    if simulationSettings is None:
        simulationSettings = exudyn.SimulationSettings()
    timeIntegration = simulationSettings.timeIntegration
    if solverType is None:
        solverType = timeIntegration.solverType
    if solverType not in explicitSolverTypes:
        raise ValueError('SolveExplicit: solverType must be one of ' + str(explicitSolverTypes) + ', not ' + str(solverType))
    _RefuseLieGroupNodes(system, timeIntegration.explicit.useLieGroupIntegration, 'SolveExplicit')
    if system.nODE1 and solverType == exudyn.DynamicSolverType.VelocityVerlet:
        raise ValueError('SolveExplicit: VelocityVerlet integrates ODE2 coordinates only; use ExplicitEuler or RK44')

    (n, tStart, tEnd, h) = _Steps(simulationSettings)
    massSolver = _MassMatrixSolver(system, timeIntegration.explicit.computeMassMatrixInversePerBody,
                                   timeIntegration.reuseConstantMassMatrix)
    forceEvaluations = [0]
    discontinuous = timeIntegration.discontinuous
    statistics = {'discontinuousIterations': 0}
    system.data = system.InitialStateData()
    (q, q_t) = system.InitialState()
    q_tt = np.zeros(system.nODE2)
    (y, y_t) = (system.InitialStateODE1(), np.zeros(system.nODE1))

    #the nodes with Euler parameters and the coordinates fixed by the other algebraic equations (#3238)
    eulerParameterNodes = getattr(system, 'eulerParameterNodes', [])
    rowsEP = [int(row) for node in eulerParameterNodes for row in node['ltgAE']]
    rows = [i for i in range(system.nAE) if i not in rowsEP]
    (fixed, fixedValues, free, coupled) = (np.zeros(0, dtype=int), np.zeros(0), np.arange(system.nODE2), bool(rowsEP))
    if rows and not timeIntegration.explicit.eliminateConstraints:
        warnings.warn('SolveExplicit: eliminateConstraints=False: the algebraic equations are ignored')
    elif rows:
        (fixed, fixedValues) = _FixedCoordinates(system, q, q_t, y, tStart, rows)
        free = np.setdiff1d(np.arange(system.nODE2), fixed)
        massMatrix = system.MassMatrix(q, q_t, tStart)
        coupled = coupled or not system.IsMassMatrixConstant() or np.any(massMatrix[np.ix_(fixed, free)] != 0.)
        (q[fixed], q_t[fixed]) = (fixedValues, 0.)
    if eulerParameterNodes:
        qRef = system.qRef
        _ProjectEulerParameters(eulerParameterNodes, qRef, q, q_t)
        position = {int(i): k for (k, i) in enumerate(free)} #the free coordinate of each coordinate

    def Accelerations(q, q_t, t, y=None):
        forceEvaluations[0] += 1
        forces = system.ODE2RHS(q, q_t, t, y=y) if system.nODE1 else system.ODE2RHS(q, q_t, t)
        if coupled: #the mass matrix without the rows and columns of the fixed coordinates, with the Euler parameters
            #[M C^T; C 0] [a; z] = [f; -2 p_t^T p_t], C = 2 p^T
            massMatrix = system.MassMatrix(q, q_t, t)
            (nFree, nEP) = (len(free), len(eulerParameterNodes))
            A = np.zeros((nFree + nEP, nFree + nEP))
            rhs = np.zeros(nFree + nEP)
            A[:nFree, :nFree] = massMatrix[np.ix_(free, free)]
            rhs[:nFree] = forces[free]
            for (k, node) in enumerate(eulerParameterNodes):
                columns = [position[int(i)] for i in node['ltg']]
                p = qRef[node['ltg']] + q[node['ltg']]
                A[nFree + k, columns] = 2.*p
                A[columns, nFree + k] = 2.*p
                rhs[nFree + k] = -2.*(q_t[node['ltg']] @ q_t[node['ltg']])
            accelerations = np.zeros(system.nODE2)
            accelerations[free] = np.linalg.solve(A, rhs)[:nFree]
            return accelerations
        accelerations = massSolver.Accelerations(q, q_t, t, forces)
        accelerations[fixed] = 0.
        return accelerations
    if solverType == exudyn.DynamicSolverType.VelocityVerlet:
        q_tt = Accelerations(q, q_t, tStart)
    output = _Output(system, simulationSettings, tStart, tEnd, h)
    output.Step(0, tStart, q, q_t, q_tt, y=y, y_t=y_t)

    stepSize = _StepSize(simulationSettings, explicit=True)
    while not stepSize.Done():
        start = (q, q_t, q_tt, y, y_t)
        dataStart = system.data.copy()
        (t0, t, h) = stepSize.Next()
        iteration = 0
        while True:
            (q, q_t, q_tt, y, y_t) = (x.copy() for x in start) #the step again from its start, with the new data
            if solverType == exudyn.DynamicSolverType.VelocityVerlet:
                q_t = q_t + (0.5*h)*q_tt
                q = q + h*q_t
                q_tt = Accelerations(q, q_t, t)
                q_t = q_t + (0.5*h)*q_tt
            else:
                (A, weights, times) = _butcherTableaus[solverType]
                K, K_t, K_y = [], [], []
                for i in range(len(weights)):
                    (g, g_t, g_y) = (q.copy(), q_t.copy(), y.copy())
                    for j in range(i):
                        if A[i][j] != 0.:
                            g += (A[i][j]*h)*K[j]
                            g_t += (A[i][j]*h)*K_t[j]
                            g_y += (A[i][j]*h)*K_y[j]
                    K.append(g_t)
                    K_t.append(Accelerations(g, g_t, t0 + h*times[i], g_y))
                    K_y.append(system.ODE1RHS(g, g_t, g_y, t0 + h*times[i]) if system.nODE1 else g_y)
                for i in range(len(weights)):
                    if weights[i] != 0.:
                        q = q + (weights[i]*h)*K[i]
                        q_t = q_t + (weights[i]*h)*K_t[i]
                        y = y + (weights[i]*h)*K_y[i]
                if system.nODE1:
                    y_t = system.ODE1RHS(q, q_t, y, t)
                if timeIntegration.explicit.computeEndOfStepAccelerations:
                    q_tt = Accelerations(q, q_t, t, y)
                else:
                    q_tt = K_t[-1]
            if eulerParameterNodes:
                _ProjectEulerParameters(eulerParameterNodes, qRef, q, q_t)
            done = _PostNewton(system, discontinuous, q, q_t, y, None, dataStart, t, iteration, statistics, stepSize, h)
            if done:
                break
            iteration += 1
            if done is None: #rejected: the step with the smaller step size, from the start of the step and its data
                (t0, t, h) = stepSize.Next()
                iteration = 0
        stepSize.Accept(t, iteration + 1)
        output.Step(stepSize.stepIndex, t, q, q_t, q_tt, last=stepSize.Done(), y=y, y_t=y_t, h=h)

    return output.Finalize({'steps': stepSize.stepIndex, 'forceEvaluations': forceEvaluations[0],
                            'massMatrixEvaluations': massSolver.evaluations, 'rejectedSteps': stepSize.rejected, **statistics})


def SolveGeneralizedAlpha(system, simulationSettings=None):
    """the implicit time integration of a SymbolicSystemBase by the generalized-alpha method in the form of Arnold
    and Bruls, as Exudyn's MainSolverImplicitSecondOrder computes it (#3126, #3134):
    per step the predictor of the displacements, velocities and algorithmic
    accelerations, the ODE1 coordinates by the trapezoidal rule, then Newton's method for the accelerations, the ODE1
    coordinates and the algebraic coordinates with the residual of SystemResidual - r = M q_tt - f + C_q^T z,
    g - y_t, c - and its Jacobian, the ODE2 rows scaled by beta h^2 and the algebraic columns by 1/(beta h^2), as
    Exudyn; by timeIntegration: numberOfSteps, startTime, endTime, generalizedAlpha.spectralRadius, useNewmark with
    newmarkBeta and newmarkGamma, computeInitialAccelerations - with constraints of [M C_q^T; C_q 0] [q_tt; z] =
    [f; -(C_q q_t)_q q_t] -, storeInitialAlgebraicCoordinates, and newton.relativeTolerance, absoluteTolerance,
    maxIterations, useModifiedNewton (the Jacobian once per step) and weightTolerancePerCoordinate; for a
    SymbolicSystem the solution file, the sensors and the renderer of its mbs, as SolveExplicit. With adaptiveStep, a
    step rejected by the post-Newton step of data coordinates is repeated with a smaller step size, as in SolveExplicit
    (#3220); a step whose Newton iteration does not converge raises. The ODE1 velocities start from the right-hand
    side of the ODE1 equations at the initial state, as in Exudyn (#3258)

    Args:
        system: the system, a SymbolicSystemBase
        simulationSettings: exudyn.SimulationSettings; None: the defaults

    Returns:
        dict with 'time' (n_out), 'q', 'q_t', 'q_tt' (n_out x nODE2), with ODE1 coordinates 'y', 'y_t', with algebraic
        equations 'z', at the steps of solution.file.writePeriod and the last step, and the statistics 'steps',
        'newtonIterations', 'jacobianEvaluations', 'discontinuousIterations', 'rejectedSteps'
    """
    if simulationSettings is None:
        simulationSettings = exudyn.SimulationSettings()
    timeIntegration = simulationSettings.timeIntegration
    generalizedAlpha = timeIntegration.generalizedAlpha
    newton = timeIntegration.newton
    _RefuseLieGroupNodes(system, True, 'SolveGeneralizedAlpha') #the implicit solver of Exudyn integrates them always so
    if generalizedAlpha.useIndex2Constraints:
        raise ValueError('SolveGeneralizedAlpha: useIndex2Constraints is not implemented')

    #the parameters of src/Solver/CSolverImplicitSecondOrder.cpp, PreInitializeSolverSpecific
    if generalizedAlpha.useNewmark:
        (alphaM, alphaF) = (0.5, 0.5) #a spectral radius of 1; the predictor takes the accelerations directly
        (beta, gamma) = (generalizedAlpha.newmarkBeta, generalizedAlpha.newmarkGamma)
    else:
        rho = generalizedAlpha.spectralRadius
        (alphaM, alphaF) = ((2*rho - 1)/(rho + 1), rho/(rho + 1))
        gamma = 0.5 + alphaF - alphaM
        beta = 0.25*(gamma + 0.5)**2
    (n, tStart, tEnd, h) = _Steps(simulationSettings)
    (nODE2, nODE1, nAE) = (system.nODE2, system.nODE1, system.nAE)
    nSystem = nODE2 + nODE1 + nAE
    general = (nODE1 + nAE) > 0 #with ODE1 or algebraic coordinates: the residual and Jacobian of all equations, dense
    errorFactor = np.sqrt(nSystem) if (newton.weightTolerancePerCoordinate and nSystem) else 1.

    (q, q_t) = system.InitialState()
    q_tt = np.zeros(nODE2)
    (y, y_t, z) = (system.InitialStateODE1(), np.zeros(nODE1), np.zeros(nAE))
    system.data = system.InitialStateData()
    zInitial = np.zeros(nAE)
    if generalizedAlpha.computeInitialAccelerations:
        if not general:
            q_tt = scipy.sparse.linalg.spsolve(system.MassMatrix(q, q_t, tStart, sparse=True).tocsc(), system.ODE2RHS(q, q_t, tStart))
        else:
            (q_tt, zInitial) = _InitialAccelerations(system, q, q_t, y, tStart)
            if generalizedAlpha.storeInitialAlgebraicCoordinates:
                z = zInitial
    if nODE1: #the trapezoidal rule starts from the derivatives of the initial state, as Exudyn (#3258)
        y_t = system.ODE1RHS(q, q_t, y, tStart) - (system.ODE1ReactionTermsLHS(q, q_t, y, zInitial, tStart) if nAE else 0.)
    aAlgorithmic = q_tt.copy()
    output = _Output(system, simulationSettings, tStart, tEnd, h)
    output.Step(0, tStart, q, q_t, q_tt, y=y, y_t=y_t, z=z)

    def NewtonResidual():
        if general:
            return system.SystemResidual(q, q_t, q_tt, y, y_t, z, t)
        return system.ODE2ResidualLHS(q, q_t, q_tt, t)

    (newtonIterations, jacobianEvaluations) = (0, 0)
    discontinuous = timeIntegration.discontinuous
    statistics = {'discontinuousIterations': 0}
    stepSize = _StepSize(simulationSettings, explicit=False)
    while not stepSize.Done():
        start = (q, q_t, q_tt, aAlgorithmic, y, y_t, z)
        dataStart = system.data.copy()
        (t0, t, h) = stepSize.Next()
        discontinuousIteration = 0
        while True:
            (q, q_t, q_tt, aAlgorithmic, y, y_t, z) = (x.copy() for x in start) #the step again from its start, with the new data
            betaPrime = (1 - alphaM)/(h**2*beta*(1 - alphaF))
            gammaPrime = gamma/(h*beta)
            scaling = beta*h**2 #of the ODE2 rows and, inverse, the algebraic columns, as Exudyn's useScaling
            #the predictor, ComputeNewtonUpdate(initial=True)
            if generalizedAlpha.useNewmark:
                q = q + h*q_t + (h**2*(0.5 - beta))*q_tt
                q_t = q_t + (h*(1 - gamma))*q_tt
            else:
                q = q + h*q_t + (h**2*(0.5 - beta))*aAlgorithmic
                q_t = q_t + (h*(1 - gamma))*aAlgorithmic
                aAlgorithmic = (alphaF*q_tt - alphaM*aAlgorithmic)/(1 - alphaM)
                q = q + (h**2*beta)*aAlgorithmic
                q_t = q_t + (h*gamma)*aAlgorithmic
            q_tt = np.zeros(nODE2)
            y = y + (0.5*h)*y_t #the trapezoidal rule y = y0 + h/2 (y_t0 + y_t)
            y_t = np.zeros(nODE1)

            #Newton's method, as CSolverBase::Newton: converged by the residual relative to the initial one or absolute
            initialResidual = cMath.norm(NewtonResidual())/errorFactor
            if initialResidual <= newton.absoluteTolerance:
                initialResidual = 2*newton.absoluteTolerance #at least one Newton step
            factorization = None
            converged = False
            newtonSteps = 0
            for iteration in range(int(newton.maxIterations)):
                if factorization is None or not newton.useModifiedNewton:
                    if general:
                        J = system.SystemJacobian(q, q_t, q_tt, y, y_t, z, t, factor_q=1., factor_q_t=gammaPrime, factor_q_tt=betaPrime,
                                                  factor_y=1., factor_y_t=2./h)
                        J[:nODE2, :] *= scaling
                        J[:, nODE2 + nODE1:] /= scaling
                        factorization = scipy.linalg.lu_factor(J)
                    else:
                        factorization = scipy.sparse.linalg.splu(system.ODE2ResidualLHSJacobian(q, q_t, q_tt, t, factor_q=1., factor_q_t=gammaPrime,
                                                                                         factor_q_tt=betaPrime, sparse=True).tocsc())
                    jacobianEvaluations += 1
                if general:
                    r = NewtonResidual()
                    r[:nODE2] *= scaling
                    increment = scipy.linalg.lu_solve(factorization, r)
                    y = y - increment[nODE2:nODE2 + nODE1]
                    y_t = y_t - (2./h)*increment[nODE2:nODE2 + nODE1]
                    z = z - increment[nODE2 + nODE1:]/scaling
                    increment = increment[:nODE2]
                else:
                    increment = factorization.solve(NewtonResidual())
                q = q - increment
                q_t = q_t - gammaPrime*increment
                q_tt = q_tt - betaPrime*increment
                newtonIterations += 1
                newtonSteps += 1
                residual = cMath.norm(NewtonResidual())/errorFactor
                if residual/initialResidual <= newton.relativeTolerance or residual <= newton.absoluteTolerance:
                    converged = True
                    break
            if not converged:
                raise ValueError('SolveGeneralizedAlpha: Newton did not converge in step ' + str(stepSize.stepIndex + 1) + ' at t=' + str(t)
                                 + ' after ' + str(int(newton.maxIterations)) + ' iterations; residual ' + str(residual))
            if not generalizedAlpha.useNewmark:
                aAlgorithmic = aAlgorithmic + ((1 - alphaF)/(1 - alphaM))*q_tt #FinalizeNewton
            done = _PostNewton(system, discontinuous, q, q_t, y, z, dataStart, t, discontinuousIteration, statistics, stepSize, h)
            if done:
                break
            discontinuousIteration += 1
            if done is None: #rejected: the step with the smaller step size, from the start of the step and its data
                (t0, t, h) = stepSize.Next()
                discontinuousIteration = 0
        stepSize.Accept(t, newtonSteps + discontinuousIteration + 1)
        output.Step(stepSize.stepIndex, t, q, q_t, q_tt, last=stepSize.Done(), y=y, y_t=y_t, z=z, h=h)

    return output.Finalize({'steps': stepSize.stepIndex, 'newtonIterations': newtonIterations,
                            'jacobianEvaluations': jacobianEvaluations, 'rejectedSteps': stepSize.rejected, **statistics})


def _PostNewton(system, discontinuous, q, q_t, y, z, dataStart, t, iteration, statistics, stepSize, h):
    """the post-Newton step after a step of a solver, as CSolverBase::DiscontinuousIteration (#3196): the new data
    coordinates into system.data; True if the step is done - the error is at most discontinuous.iterationTolerance,
    or the system has no data coordinates -, False if it is computed again from its start with the new data, None if
    it is rejected and stepSize reduced, the data back at the start of the step (#3220): where the recommended step
    size is below h, or after discontinuous.maxIterations. Without a smaller step it raises after maxIterations, unless
    discontinuous.ignoreMaxIterations"""
    if not system.nData:
        return True
    (system.data, error, recommendedStepSize) = system.PostNewton(q, q_t, y, z, system.data, dataStart, t)
    statistics['discontinuousIterations'] += 1
    if error <= discontinuous.iterationTolerance:
        return True
    if not stepSize.useRecommended:
        recommendedStepSize = -1
    if iteration + 1 < discontinuous.maxIterations:
        if recommendedStepSize == -1 or recommendedStepSize >= (1 + 1e-8)*h: #a tolerance against switching for linear problems
            return False
    elif discontinuous.ignoreMaxIterations:
        return True
    if stepSize.Reduce(recommendedStepSize):
        system.data = dataStart.copy()
        return None
    raise ValueError('the discontinuous iteration did not converge in ' + str(int(discontinuous.maxIterations))
                     + ' iterations at t=' + str(t) + ', error ' + str(error) + (', at the minimum step size' if stepSize.adaptive
                     else '') + '; discontinuous.ignoreMaxIterations continues anyway')


def _InitialAccelerations(system, q, q_t, y, t, epsilon=1e-6):
    """the initial accelerations and algebraic coordinates of CSolverImplicitSecondOrder: [M C_q^T; C_q C_z] [q_tt; z]
    = [f; -(C_q q_t)_q q_t], the ODE1 rows a unit matrix with zero right-hand side; C_q and C_z - the derivative by z
    of an equation that depends on it, as of ObjectGenericDAE (#3191) - by central differences of the algebraic
    equations, (C_q q_t)_q q_t as their second derivative along q_t"""
    (nODE2, nODE1, nAE) = (system.nODE2, system.nODE1, system.nAE)
    nSystem = nODE2 + nODE1 + nAE
    z = np.zeros(nAE)
    A = np.zeros((nSystem, nSystem))
    rhs = np.zeros(nSystem)
    A[:nODE2, :nODE2] = system.MassMatrix(q, q_t, t)
    rhs[:nODE2] = system.ODE2RHS(q, q_t, t, y=y) if nODE1 else system.ODE2RHS(q, q_t, t)
    A[nODE2:nODE2 + nODE1, nODE2:nODE2 + nODE1] = np.eye(nODE1)
    if nAE:
        def Equations(qq):
            return system.AlgebraicEquations(qq, q_t, y, z, t)
        Cq = np.array([(Equations(q + epsilon*e) - Equations(q - epsilon*e))/(2*epsilon) for e in np.eye(nODE2)]).T
        A[nODE2 + nODE1:, :nODE2] = Cq
        A[:nODE2, nODE2 + nODE1:] = Cq.T
        A[nODE2 + nODE1:, nODE2 + nODE1:] = np.array([(system.AlgebraicEquations(q, q_t, y, z + epsilon*e, t)
                                                      - system.AlgebraicEquations(q, q_t, y, z - epsilon*e, t))/(2*epsilon)
                                                     for e in np.eye(nAE)]).T
        if cMath.norm(q_t) > 1e-10:
            delta = 1e-4
            rhs[nODE2 + nODE1:] = -(Equations(q + delta*q_t) - 2*Equations(q) + Equations(q - delta*q_t))/delta**2
    solution = np.linalg.solve(A, rhs)
    return solution[:nODE2], solution[nODE2 + nODE1:]
