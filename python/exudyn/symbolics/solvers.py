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
#           (#3196); the statistics count the discontinuousIterations.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-26 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
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


class _Output:
    """the output of a solver of this module: the states at the steps of solution.file.writePeriod, as CSolverBase writes
    them - the start, then each time a write period is passed, and the last step -, and for a SymbolicSystem the output
    through its mbs: an Exudyn solver object, which writes the solution file and the sensors and updates the renderer in
    FinishStep, by simulationSettings.solution; it runs as explicit Euler without inverse per body, which only decides
    what its initialization computes"""
    def __init__(self, system, simulationSettings, tStart, tEnd, h):
        (self.tStart, self.tEnd, self.h) = (tStart, tEnd, h)
        self.period = simulationSettings.solution.file.writePeriod
        self.lastWritten = tStart
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

    def Step(self, stepIndex, t, q, q_t, q_tt, last=False, y=None, y_t=None, z=None):
        """the state after step stepIndex at t; stepIndex 0 is the initial state, as SolveDynamic writes it"""
        if last:
            self.lastWritten = self.tEnd - self.h
        if stepIndex == 0 or t - self.lastWritten >= -1e-10:
            self.lastWritten = max(self.lastWritten + self.period, t)
            for (name, value) in [('time', t), ('q', q), ('q_t', q_t), ('q_tt', q_tt), ('y', y), ('y_t', y_t), ('z', z)]:
                if name in self.result:
                    self.result[name].append(np.copy(value))
        if self.solver is not None:
            for (name, value) in [('ODE2Coords', q), ('ODE2Coords_t', q_t), ('ODE2Coords_tt', q_tt), ('ODE1Coords', y),
                                  ('ODE1Coords_t', y_t), ('AECoords', z), ('dataCoords', self.system.data)]:
                if value is not None and len(value):
                    self.state[name][:] = value
            self.system.mbs.systemData.SetTime(t)
            self.solver.it.currentStepIndex = stepIndex
            self.solver.it.currentTime = t
            self.solver.it.currentStepSize = self.h
            self.solver.FinishStep(self.system.mbs, self.simulationSettings)

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


def _RefuseLieGroupNodes(system, useLieGroupIntegration, where):
    """Lie group nodes are integrated by Exudyn with the Lie group formulas, which are not implemented here"""
    mbs = getattr(system, 'mbs', None)
    if mbs is not None and useLieGroupIntegration:
        for nodeNumber in range(mbs.systemData.NumberOfNodes()):
            if mbs.GetNode(nodeNumber)['nodeType'].endswith('LG'):
                raise ValueError(where + ': node ' + str(nodeNumber) + ' is a Lie group node, which Exudyn integrates '
                                 'by the Lie group formulas (useLieGroupIntegration); they are not implemented here')


def SolveExplicit(system, simulationSettings=None, solverType=None):
    """the explicit time integration of a SymbolicSystemBase with constant step size, by explicit Euler, RK44 or
    velocity Verlet, as Exudyn's explicit solver computes them (#3112, design docs/dev/design/symbolicSystem.md §10):
    the steps from timeIntegration.startTime, endTime and numberOfSteps; the accelerations by the inverse per body with
    timeIntegration.explicit.computeMassMatrixInversePerBody, else by the sparse LU decomposition of the mass
    matrix, both once for a constant mass matrix (timeIntegration.reuseConstantMassMatrix); ODE1 coordinates in the
    stages of explicit Euler and RK44, with zero initial velocities, as Exudyn (#3134); algebraic equations are refused
    (Exudyn's explicit solver eliminates only constraints of coordinates on the ground); for a SymbolicSystem the
    solution file, the sensors and the renderer of its mbs by simulationSettings.solution, as after SolveDynamic

    Args:
        system: the system, a SymbolicSystemBase
        simulationSettings: exudyn.SimulationSettings; None: the defaults
        solverType: an exudyn.DynamicSolverType of explicitSolverTypes; None: timeIntegration.solverType

    Returns:
        dict with 'time' (n_out), 'q', 'q_t', 'q_tt' (n_out x nODE2) - with ODE1 coordinates 'y', 'y_t' (n_out x
        nODE1) - at the steps of solution.file.writePeriod and the last step, and the statistics 'steps',
        'forceEvaluations', 'massMatrixEvaluations'
    """
    if simulationSettings is None:
        simulationSettings = exudyn.SimulationSettings()
    timeIntegration = simulationSettings.timeIntegration
    if solverType is None:
        solverType = timeIntegration.solverType
    if solverType not in explicitSolverTypes:
        raise ValueError('SolveExplicit: solverType must be one of ' + str(explicitSolverTypes) + ', not ' + str(solverType))
    _RefuseLieGroupNodes(system, timeIntegration.explicit.useLieGroupIntegration, 'SolveExplicit')
    if system.nAE:
        raise ValueError('SolveExplicit: the system has algebraic equations, which an explicit solver does not integrate; '
                         'use SolveGeneralizedAlpha')
    if system.nODE1 and solverType == exudyn.DynamicSolverType.VelocityVerlet:
        raise ValueError('SolveExplicit: VelocityVerlet integrates ODE2 coordinates only; use ExplicitEuler or RK44')

    (n, tStart, tEnd, h) = _Steps(simulationSettings)
    massSolver = _MassMatrixSolver(system, timeIntegration.explicit.computeMassMatrixInversePerBody,
                                   timeIntegration.reuseConstantMassMatrix)
    forceEvaluations = [0]
    discontinuous = timeIntegration.discontinuous
    statistics = {'discontinuousIterations': 0}
    system.data = system.InitialStateData()
    def Accelerations(q, q_t, t, y=None):
        forceEvaluations[0] += 1
        forces = system.ODE2RHS(q, q_t, t, y=y) if system.nODE1 else system.ODE2RHS(q, q_t, t)
        return massSolver.Accelerations(q, q_t, t, forces)

    (q, q_t) = system.InitialState()
    q_tt = np.zeros(system.nODE2)
    (y, y_t) = (system.InitialStateODE1(), np.zeros(system.nODE1))
    if solverType == exudyn.DynamicSolverType.VelocityVerlet:
        q_tt = Accelerations(q, q_t, tStart)
    output = _Output(system, simulationSettings, tStart, tEnd, h)
    output.Step(0, tStart, q, q_t, q_tt, y=y, y_t=y_t)

    for step in range(1, n + 1):
        t0 = tStart + (step - 1)*h
        t = tStart + step*h
        start = (q, q_t, q_tt, y, y_t)
        dataStart = system.data.copy()
        for iteration in range(discontinuous.maxIterations):
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
            if _PostNewton(system, discontinuous, q, q_t, y, None, dataStart, t, iteration, statistics):
                break
        output.Step(step, t, q, q_t, q_tt, last=(step == n), y=y, y_t=y_t)

    return output.Finalize({'steps': n, 'forceEvaluations': forceEvaluations[0],
                            'massMatrixEvaluations': massSolver.evaluations, **statistics})


def SolveGeneralizedAlpha(system, simulationSettings=None):
    """the implicit time integration of a SymbolicSystemBase with constant step size by the generalized-alpha method
    in the form of Arnold and Bruls, as Exudyn's MainSolverImplicitSecondOrder computes it (#3126, #3134, design
    docs/dev/design/symbolicSystem.md §6): per step the predictor of the displacements, velocities and algorithmic
    accelerations, the ODE1 coordinates by the trapezoidal rule, then Newton's method for the accelerations, the ODE1
    coordinates and the algebraic coordinates with the residual of SystemResidual - r = M q_tt - f + C_q^T z,
    g - y_t, c - and its Jacobian, the ODE2 rows scaled by beta h^2 and the algebraic columns by 1/(beta h^2), as
    Exudyn; by timeIntegration: numberOfSteps, startTime, endTime, generalizedAlpha.spectralRadius, useNewmark with
    newmarkBeta and newmarkGamma, computeInitialAccelerations - with constraints of [M C_q^T; C_q 0] [q_tt; z] =
    [f; -(C_q q_t)_q q_t] -, storeInitialAlgebraicCoordinates, and newton.relativeTolerance, absoluteTolerance,
    maxIterations, useModifiedNewton (the Jacobian once per step) and weightTolerancePerCoordinate; for a
    SymbolicSystem the solution file, the sensors and the renderer of its mbs, as SolveExplicit. A step whose Newton
    iteration does not converge raises: the step size is constant. The ODE1 velocities start at zero, as in Exudyn

    Args:
        system: the system, a SymbolicSystemBase
        simulationSettings: exudyn.SimulationSettings; None: the defaults

    Returns:
        dict with 'time' (n_out), 'q', 'q_t', 'q_tt' (n_out x nODE2), with ODE1 coordinates 'y', 'y_t', with algebraic
        equations 'z', at the steps of solution.file.writePeriod and the last step, and the statistics 'steps',
        'newtonIterations', 'jacobianEvaluations'
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
    betaPrime = (1 - alphaM)/(h**2*beta*(1 - alphaF))
    gammaPrime = gamma/(h*beta)
    (nODE2, nODE1, nAE) = (system.nODE2, system.nODE1, system.nAE)
    nSystem = nODE2 + nODE1 + nAE
    general = (nODE1 + nAE) > 0 #with ODE1 or algebraic coordinates: the residual and Jacobian of all equations, dense
    scaling = beta*h**2 #of the ODE2 rows and, inverse, the algebraic columns, as Exudyn's useScaling
    errorFactor = np.sqrt(nSystem) if (newton.weightTolerancePerCoordinate and nSystem) else 1.

    (q, q_t) = system.InitialState()
    q_tt = np.zeros(nODE2)
    (y, y_t, z) = (system.InitialStateODE1(), np.zeros(nODE1), np.zeros(nAE))
    system.data = system.InitialStateData()
    if generalizedAlpha.computeInitialAccelerations:
        if not general:
            q_tt = scipy.sparse.linalg.spsolve(system.MassMatrix(q, q_t, tStart, sparse=True).tocsc(), system.ODE2RHS(q, q_t, tStart))
        else:
            (q_tt, zInitial) = _InitialAccelerations(system, q, q_t, y, tStart)
            if generalizedAlpha.storeInitialAlgebraicCoordinates:
                z = zInitial
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
    for step in range(1, n + 1):
        t = tStart + step*h
        start = (q, q_t, q_tt, aAlgorithmic, y, y_t, z)
        dataStart = system.data.copy()
        for iteration in range(discontinuous.maxIterations):
            (q, q_t, q_tt, aAlgorithmic, y, y_t, z) = (x.copy() for x in start) #the step again from its start, with the new data
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
                residual = cMath.norm(NewtonResidual())/errorFactor
                if residual/initialResidual <= newton.relativeTolerance or residual <= newton.absoluteTolerance:
                    converged = True
                    break
            if not converged:
                raise ValueError('SolveGeneralizedAlpha: Newton did not converge in step ' + str(step) + ' at t=' + str(t)
                                 + ' after ' + str(int(newton.maxIterations)) + ' iterations; residual ' + str(residual))
            if not generalizedAlpha.useNewmark:
                aAlgorithmic = aAlgorithmic + ((1 - alphaF)/(1 - alphaM))*q_tt #FinalizeNewton
            if _PostNewton(system, discontinuous, q, q_t, y, z, dataStart, t, iteration, statistics):
                break
        output.Step(step, t, q, q_t, q_tt, last=(step == n), y=y, y_t=y_t, z=z)

    return output.Finalize({'steps': n, 'newtonIterations': newtonIterations, 'jacobianEvaluations': jacobianEvaluations,
                            **statistics})


def _PostNewton(system, discontinuous, q, q_t, y, z, dataStart, t, iteration, statistics):
    """the post-Newton step after a step of a solver, as CSolverBase::DiscontinuousIteration (#3196): the new data
    coordinates into system.data; True if the step is done - the error is at most discontinuous.iterationTolerance,
    or the system has no data coordinates -, False if it is computed again from its start with the new data. After
    discontinuous.maxIterations it raises, unless discontinuous.ignoreMaxIterations; the recommended step size is not
    used by the constant step size"""
    if not system.nData:
        return True
    (system.data, error, recommendedStepSize) = system.PostNewton(q, q_t, y, z, system.data, dataStart, t)
    statistics['discontinuousIterations'] += 1
    if error <= discontinuous.iterationTolerance:
        return True
    if iteration + 1 >= discontinuous.maxIterations:
        if not discontinuous.ignoreMaxIterations:
            raise ValueError('the discontinuous iteration did not converge in ' + str(int(discontinuous.maxIterations))
                             + ' iterations at t=' + str(t) + ', error ' + str(error) + '; discontinuous.ignoreMaxIterations '
                             'continues anyway')
        return True
    return False


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
