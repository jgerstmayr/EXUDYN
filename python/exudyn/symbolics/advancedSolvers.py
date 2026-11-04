#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Implicit solvers beyond those of Exudyn's C++ core, for the systems of exudyn.symbolics.systemModel:
#           SolveTrapezoidalGGL integrates a SymbolicSystemBase by the trapezoidal rule with the position and the
#           velocity constraints enforced by the stabilization of Gear, Gupta and Leimkuhler (GGL), with constant steps
#           or with a step size controlled by the local error estimated from the accelerations (#3242). They take the
#           SimulationSettings of Exudyn and write their output as the solvers of exudyn.symbolics.solvers.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-04 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import warnings
import exudyn
import exudyn.cMath as cMath
from exudyn.symbolics.solvers import (_Output, _Steps, _StepSize, _PostNewton, _InitialAccelerations,
                                      _RefuseLieGroupNodes)
try:
    import scipy.linalg
except ImportError:
    raise ImportError('exudyn.symbolics.advancedSolvers needs scipy: pip install scipy')

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'SolveTrapezoidalGGL', 'implicitRungeKuttaMethods', 'SolveImplicitRungeKutta',
    ]


class _ErrorStepSize(_StepSize):
    """the step size controller of the solvers of this module: with timeIntegration.automaticStepSize controlled by
    an estimate of the local error (#3242), in the norm of the automatic step size of Exudyn's explicit solvers - the
    root mean square of the errors over absoluteTolerance + relativeTolerance times the values - by the PI controller
    of Gustafsson (#3243): after an accepted step of error e and the previous one e_p, the next step h stepSizeSafety
    (1/e)^(0.7/k) e_p^(0.4/k), k the order of the local error; a step of e > 1 is repeated with h stepSizeSafety
    (1/e)^(1/k), and the step after it does not grow; at most stepSizeMaxIncrease times h and the maximum (endTime -
    startTime)/numberOfSteps, at least minimumStepSize, where the step is accepted with a warning; the first step of
    initialStepSize, 0: the maximum. A post-Newton step or a Newton iteration that fails reduces the step as
    _StepSize. Without automaticStepSize the steps of _StepSize"""
    def __init__(self, simulationSettings, order):
        _StepSize.__init__(self, simulationSettings, explicit=False)
        timeIntegration = simulationSettings.timeIntegration
        self.automatic = timeIntegration.automaticStepSize
        self.order = order
        (self.absoluteTolerance, self.relativeTolerance) = (timeIntegration.absoluteTolerance, timeIntegration.relativeTolerance)
        (self.safety, self.maxIncrease) = (timeIntegration.stepSizeSafety, timeIntegration.stepSizeMaxIncrease)
        self.rejectedByError = 0
        (self.errorPrevious, self.rejectedLast, self.warned) = (None, False, False)
        if self.automatic:
            if self.absoluteTolerance <= 0:
                raise ValueError('SolveTrapezoidalGGL: timeIntegration.absoluteTolerance must be > 0')
            self.adaptive = True
            self.useRecommended = timeIntegration.discontinuous.useRecommendedStepSize
            if timeIntegration.initialStepSize > 0:
                self.h = min(self.hMax, timeIntegration.initialStepSize)
            self.hNext = self.h

    def Error(self, errors, values0, values1):
        """the norm of the estimated local errors: the root mean square of errors/(absoluteTolerance + relativeTolerance
        max(|values0|, |values1|))"""
        if not len(errors):
            return 0.
        scale = self.absoluteTolerance + self.relativeTolerance*np.maximum(np.abs(values0), np.abs(values1))
        return float(np.sqrt(np.mean((errors/scale)**2)))

    def Control(self, error):
        """the step size after a step of the given error; False if the step is rejected and repeated with the new step
        size"""
        error = max(error, 1e-12)
        k = self.order
        if error > 1.:
            if self.h > self.hMin:
                self.h = max(self.hMin, self.safety*self.h*cMath.pow(1./error, 1./k))
                (self.rejectedByError, self.rejectedLast) = (self.rejectedByError + 1, True)
                return False
            if not self.warned:
                warnings.warn('the automatic step size reached timeIntegration.minimumStepSize at t=' + str(self.t)
                              + '; the error is above the tolerances')
                self.warned = True
        if self.errorPrevious is None:
            factor = self.safety*cMath.pow(1./error, 1./k)
        else:
            factor = self.safety*cMath.pow(1./error, 0.7/k)*cMath.pow(self.errorPrevious, 0.4/k)
        if self.rejectedLast:
            factor = min(factor, 1.)
        (self.errorPrevious, self.rejectedLast) = (error, False)
        self.hNext = min(self.hMax, self.maxIncrease*self.h, max(self.hMin, factor*self.h))
        return True

    def Reduce(self, recommendedStepSize):
        """a smaller step after a failed post-Newton step or Newton iteration"""
        reduced = _StepSize.Reduce(self, recommendedStepSize)
        self.hNext = self.h
        return reduced

    def Accept(self, t, iterations):
        """the step to t done; with automaticStepSize the next one of the size of Control"""
        if not self.automatic:
            _StepSize.Accept(self, t, iterations)
            return
        (self.t, self.stepIndex, self.h) = (t, self.stepIndex + 1, self.hNext)


def _ConstraintJacobian(system, q, q_t, y, t):
    """C_q, the Jacobian of the algebraic equations by q, exact as the reaction forces C_q^T z of the system, row by
    row"""
    C_q = np.zeros((system.nAE, system.nODE2))
    for i in range(system.nAE):
        unit = np.zeros(system.nAE)
        unit[i] = 1.
        C_q[i] = system.ODE2ReactionForcesLHS(q, q_t, y, unit, t)
    return C_q


def _RefuseConstraintsBeyondPositions(system, q, q_t, y, t):
    """TrapezoidalGGL differentiates the algebraic equations by time: each must depend on q and t only - not on the
    velocities, the ODE1 coordinates or the algebraic coordinates -, which random variations of them show"""
    z = np.zeros(system.nAE)
    c0 = np.asarray(system.AlgebraicEquations(q, q_t, y, z, t), dtype=float)
    random = np.random.default_rng(1)
    variations = [system.AlgebraicEquations(q, q_t + random.uniform(-1., 1., len(q_t)), y, z, t),
                  system.AlgebraicEquations(q, q_t, y + random.uniform(-1., 1., len(y)), z, t),
                  system.AlgebraicEquations(q, q_t, y, random.uniform(-1., 1., system.nAE), t)]
    for (variation, name) in zip(variations, ['the velocities', 'the ODE1 coordinates', 'the algebraic coordinates']):
        rows = np.flatnonzero(np.abs(np.asarray(variation, dtype=float) - c0) > 1e-10*max(1., np.max(np.abs(c0))))
        if len(rows):
            raise ValueError('SolveTrapezoidalGGL: algebraic equation ' + str(int(rows[0])) + ' depends on ' + name
                             + '; the GGL stabilization needs equations of the coordinates and the time; use '
                             'SolveGeneralizedAlpha')


def SolveTrapezoidalGGL(system, simulationSettings=None):
    """the implicit time integration of a SymbolicSystemBase by the trapezoidal rule with the stabilization of Gear,
    Gupta and Leimkuhler (GGL) for index-3 constraints (#3242): per step v1 = v0 + h/2 (a0 + a1), q1 = q0 + h/2 (v0 +
    v1) - C_q^T mu, M a1 = f(q1, v1, t1) - C_q^T z1, c(q1, t1) = 0 and C_q v1 + c_t = 0, with C_q = C_q(q1) - in the term of mu that of the previous Newton iterate -, the multiplier mu of the velocity
    constraints, which moves the positions back onto the constraints, and the ODE1 coordinates by the trapezoidal rule
    as well; Newton's method for a1, y1, z1 and mu with the Jacobian of the system, the ODE2 rows scaled by h^2/4 and
    the algebraic columns by its inverse, as Exudyn's implicit solver; of the second derivatives of the constraints
    only (C_q v)_q in the Jacobian. The method is of order 2 in q, v, the accelerations and z, and does not damp; the
    constraints hold on position and velocity level in every step. The algebraic equations must depend on q and t
    only; C_q comes from the reaction forces of the system.

    With timeIntegration.automaticStepSize (the default, as for the explicit solvers DOPRI5 and ODE23) the step size
    is controlled by the local errors of the positions, h^3/12 q_ttt = h^2/12 (a1 - a0), and of the velocities, h^3/12
    a_tt by the second difference of the mean accelerations (v1 - v0)/h of the last three steps - the accelerations a
    of the trapezoidal rule alternate with constraints, their means do not -, with the PI controller of
    _ErrorStepSize and timeIntegration.absoluteTolerance, relativeTolerance, stepSizeSafety, stepSizeMaxIncrease,
    initialStepSize and minimumStepSize, the maximum step (endTime - startTime)/numberOfSteps; the tolerances are
    those of a step, the global error grows with the number of steps and with the sensitivity of the motion. The
    ODE1 coordinates are not in the estimate. Without automaticStepSize the steps are constant, as
    SolveGeneralizedAlpha. The initial accelerations and
    algebraic coordinates are consistent ([M C_q^T; C_q 0] [a0; z0] = [f; -(C_q v)_q v]), the initial ODE1 velocities
    those of the ODE1 equations. Newton's method by timeIntegration.newton (relativeTolerance, absoluteTolerance,
    maxIterations, useModifiedNewton,
    weightTolerancePerCoordinate); a step whose Newton iteration does not converge or diverges is repeated with a
    smaller step with automaticStepSize or adaptiveStep, else raises. Data coordinates and adaptiveStep as SolveGeneralizedAlpha;
    the output - the result, and for a SymbolicSystem the solution file, the sensors and the renderer of its mbs - as
    the solvers of exudyn.symbolics.solvers, at the exact write times

    Args:
        system: the system, a SymbolicSystemBase
        simulationSettings: exudyn.SimulationSettings; None: the defaults

    Returns:
        dict with 'time' (n_out), 'q', 'q_t', 'q_tt' (n_out x nODE2), with ODE1 coordinates 'y', 'y_t', with algebraic
        equations 'z', at the times of solution.file.writePeriod and the end, and the statistics 'steps',
        'newtonIterations', 'jacobianEvaluations', 'discontinuousIterations', 'rejectedSteps' (by Newton or the
        post-Newton step) and 'rejectedStepsError' (by the error estimate)
    """
    if simulationSettings is None:
        simulationSettings = exudyn.SimulationSettings()
    timeIntegration = simulationSettings.timeIntegration
    newton = timeIntegration.newton
    _RefuseLieGroupNodes(system, True, 'SolveTrapezoidalGGL')
    (n, tStart, tEnd, h) = _Steps(simulationSettings)
    (nODE2, nODE1, nAE) = (system.nODE2, system.nODE1, system.nAE)
    nEquations = nODE2 + nODE1 + 2*nAE
    errorFactor = np.sqrt(nEquations) if (newton.weightTolerancePerCoordinate and nEquations) else 1.
    (rowsODE2, rowsODE1, rowsAE) = (slice(0, nODE2), slice(nODE2, nODE2 + nODE1), slice(nODE2 + nODE1, nODE2 + nODE1 + nAE))
    rowsV = slice(nODE2 + nODE1 + nAE, nEquations)

    (q, q_t) = system.InitialState()
    y = system.InitialStateODE1()
    system.data = system.InitialStateData()
    if nAE:
        _RefuseConstraintsBeyondPositions(system, q, q_t, y, tStart)
    (q_tt, z) = _InitialAccelerations(system, q, q_t, y, tStart)
    y_t = system.ODE1RHS(q, q_t, y, tStart) if nODE1 else np.zeros(0)
    output = _Output(system, simulationSettings, tStart, tEnd, h)
    output.Step(0, tStart, q, q_t, q_tt, y=y, y_t=y_t, z=z)

    def TimeDerivative(qq, t, epsilon=1e-6):
        """c_t, the partial derivative of the constraints by time"""
        return (np.asarray(system.AlgebraicEquations(qq, q_t, y, z, t + epsilon), dtype=float)
                - np.asarray(system.AlgebraicEquations(qq, q_t, y, z, t - epsilon), dtype=float))/(2*epsilon)

    (newtonIterations, jacobianEvaluations) = (0, 0)
    discontinuous = timeIntegration.discontinuous
    statistics = {'discontinuousIterations': 0}
    stepSize = _ErrorStepSize(simulationSettings, order=3) #the local errors are of order h^3
    history = []
    while not stepSize.Done():
        start = (q, q_t, q_tt, y, y_t, z)
        dataStart = system.data.copy()
        (t0, t, h) = stepSize.Next()
        discontinuousIteration = 0
        while True:
            (q0, v0, a0, y0, y_t0, z0) = (x.copy() for x in start) #the step again from its start, with the new data
            scaling = 0.25*h**2 #of the ODE2 rows and, inverse, the algebraic columns, as Exudyn's useScaling
            (a, y, z, mu) = (a0.copy(), y0 + h*y_t0, z0.copy(), np.zeros(nAE))
            qIterate = q0 + h*v0 + (0.5*h**2)*a0 #the positions of the previous iterate, for C_q^T mu

            def State():
                v = v0 + (0.5*h)*(a0 + a)
                qq = q0 + (0.5*h)*(v0 + v)
                if nAE:
                    qq -= system.ODE2ReactionForcesLHS(qIterate, v, y, mu, t)
                return (qq, v, (2./h)*(y - y0) - y_t0)

            def Residual():
                (qq, v, yy_t) = State()
                r = np.zeros(nEquations)
                r[:nODE2 + nODE1 + nAE] = system.SystemResidual(qq, v, a, y, yy_t, z, t)
                r[rowsODE2] *= scaling
                if nAE:
                    r[rowsV] = (0.5*h)*(_ConstraintJacobian(system, qq, v, y, t) @ v + TimeDerivative(qq, t))
                return r

            #Newton's method, as SolveGeneralizedAlpha
            factorization = None
            converged = False
            newtonSteps = 0
            residual = initialResidual = None
            for iteration in range(int(newton.maxIterations)):
                if factorization is None or not newton.useModifiedNewton:
                    (qq, v, yy_t) = State()
                    J = np.zeros((nEquations, nEquations))
                    J[:nODE2 + nODE1 + nAE, :nODE2 + nODE1 + nAE] = system.SystemJacobian(
                        qq, v, a, y, yy_t, z, t, factor_q=scaling, factor_q_t=0.5*h, factor_q_tt=1., factor_y=1., factor_y_t=2./h)
                    if nAE:
                        J_q = system.SystemJacobian(qq, v, a, y, yy_t, z, t, factor_q=1., factor_q_t=0., factor_q_tt=0.,
                                                    factor_y=0., factor_y_t=0.)[:, :nODE2]
                        C_q = J_q[rowsAE]
                        epsilon = 1e-6 #H = d(C_q v)/dq = (C_q)_q v, the derivative of C_q along v
                        H = (_ConstraintJacobian(system, qq + epsilon*v, v, y, t) - _ConstraintJacobian(system, qq - epsilon*v, v, y, t))/(2*epsilon)
                        J[:nODE2 + nODE1 + nAE, rowsV] = -J_q @ C_q.T #dq/dmu = -C_q^T
                        J[rowsV, :nODE2] = scaling*(C_q + (0.5*h)*H) #dv/da = h/2, dq/da = h^2/4
                        J[rowsV, rowsV] = -(0.5*h)*H @ C_q.T
                    J[rowsODE2, :] *= scaling
                    J[:, rowsAE] /= scaling
                    factorization = scipy.linalg.lu_factor(J)
                    jacobianEvaluations += 1
                r = Residual()
                if not np.all(np.isfinite(r)): #diverged: as not converged
                    break
                if initialResidual is None:
                    initialResidual = cMath.norm(r)/errorFactor
                    if initialResidual <= newton.absoluteTolerance:
                        initialResidual = 2*newton.absoluteTolerance #at least one Newton step
                increment = scipy.linalg.lu_solve(factorization, r)
                qIterate = State()[0]
                a = a - increment[:nODE2]
                y = y - increment[rowsODE1]
                z = z - increment[rowsAE]/scaling
                mu = mu - increment[rowsV]
                newtonIterations += 1
                newtonSteps += 1
                residual = cMath.norm(Residual())/errorFactor
                if residual/initialResidual <= newton.relativeTolerance or residual <= newton.absoluteTolerance:
                    converged = True
                    break
            if not converged:
                if stepSize.Reduce(-1):
                    (t0, t, h) = stepSize.Next()
                    discontinuousIteration = 0
                    continue
                raise ValueError('SolveTrapezoidalGGL: Newton did not converge in step ' + str(stepSize.stepIndex + 1) + ' at t='
                                 + str(t) + ' after ' + str(int(newton.maxIterations)) + ' iterations; residual ' + str(residual))
            (q, q_t, y_t) = State()
            q_tt = a
            if stepSize.automatic: #the local errors h^3/12 q''' and h^3/12 v''', from the accelerations
                errors = [((h**2/12.)*(a - a0), q0, q)]
                if len(history) == 2: #v''' by the mean accelerations of the steps, which do not alternate as a
                    mean = (q_t - v0)/h
                    ((mean1, h1), (mean2, h2)) = (history[1], history[0])
                    (tau1, tau2) = (0.5*(h + h1), 0.5*(h1 + h2))
                    a_tt = (2./(tau1 + tau2))*((mean - mean1)/tau1 - (mean1 - mean2)/tau2)
                    errors.append(((h**3/12.)*a_tt, v0, q_t))
                error = stepSize.Error(*(np.concatenate(x) for x in zip(*errors)))
                if not stepSize.Control(error): #rejected: the step again with the smaller step size
                    system.data = dataStart.copy()
                    (t0, t, h) = stepSize.Next()
                    discontinuousIteration = 0
                    continue
            done = _PostNewton(system, discontinuous, q, q_t, y, z, dataStart, t, discontinuousIteration, statistics, stepSize, h)
            if done:
                break
            discontinuousIteration += 1
            if done is None: #rejected: the step with the smaller step size, from the start of the step and its data
                (t0, t, h) = stepSize.Next()
                discontinuousIteration = 0
        stepSize.Accept(t, newtonSteps + discontinuousIteration + 1)
        history = (history + [((q_t - start[1])/h, h)])[-2:] #the mean accelerations (v1 - v0)/h of the last steps
        output.Step(stepSize.stepIndex, t, q, q_t, q_tt, last=stepSize.Done(), y=y, y_t=y_t, z=z, h=h)

    return output.Finalize({'steps': stepSize.stepIndex, 'newtonIterations': newtonIterations,
                            'jacobianEvaluations': jacobianEvaluations, 'rejectedSteps': stepSize.rejected,
                            'rejectedStepsError': stepSize.rejectedByError, **statistics})



#the tableaus (A, c): the Lobatto methods of the stages 0, 1/2, 1, the trapezoidal rule (the 2-stage Lobatto IIIA) and
#Radau IIA with 2 stages at 1/3, 1
_rungeKuttaTableaus = {'LobattoIIIA': (np.array([[0., 0., 0.], [5./24., 1./3., -1./24.], [1./6., 2./3., 1./6.]]), np.array([0., 0.5, 1.])),
                       'LobattoIIIC': (np.array([[1./6., -1./3., 1./6.], [1./6., 5./12., -1./12.], [1./6., 2./3., 1./6.]]), np.array([0., 0.5, 1.])),
                       'Trapezoidal': (np.array([[0., 0.], [0.5, 0.5]]), np.array([0., 1.])),
                       'RadauIIA': (np.array([[5./12., -1./12.], [3./4., 1./4.]]), np.array([1./3., 1.]))}

def _QuadratureError(points, b, stageValues):
    """the error of the quadrature of the method, sum_j b_j stageValues_j, against the integral over the step of the
    polynomial through points, a list of (tau, value) with tau the time from the start of the step in steps (#3250)"""
    tau = np.array([point[0] for point in points])
    weights = scipy.linalg.solve(np.vander(tau, len(tau), increasing=True).T, 1./np.arange(1., len(tau) + 1.))
    return sum(bj*value for (bj, value) in zip(b, stageValues)) - sum(w*point[1] for (w, point) in zip(weights, points))


#the methods of SolveImplicitRungeKutta: (tableau, where the constraints are)
implicitRungeKuttaMethods = {'LobattoIIIA4': ('LobattoIIIA', 'end'), 'LobattoIIIC4-GGL': ('LobattoIIIC', 'GGL'),
                             'Radau3': ('RadauIIA', 'end')}


def _SolveRungeKutta(system, simulationSettings, tableau, constraints, name):
    """the stage equations of a stiffly accurate implicit Runge-Kutta method of the tableau (A, c) for q_t = v, v_t = a,
    M a = f - C_q^T z (#3244, #3246); the stages of a first row of zeros are the start of the step (Lobatto IIIA). The
    constraints: 'GGL', c = 0 and C_q V_i + c_t = 0 at the implicit stages, with a multiplier z_i per implicit stage and
    the positions Q_i - C_q^T mu_i; 'end', c = 0 and C_q v + c_t = 0 at the end of the step only, with a multiplier z_i
    per stage after the start, the one of a stage at the start the multiplier of the end of the previous step - an
    equation of each kind for the two unknown multipliers. With timeIntegration.automaticStepSize the step by
    _ErrorStepSize with the error of the quadrature of the method against the polynomial through the stage values of
    the previous and this step (#3250); the first step by the difference to the embedded trapezoidal solution of the
    first and the last stage (of the start and the last stage if no stage is at the start)"""
    if simulationSettings is None:
        simulationSettings = exudyn.SimulationSettings()
    timeIntegration = simulationSettings.timeIntegration
    newton = timeIntegration.newton
    _RefuseLieGroupNodes(system, True, name)
    (nODE2, nAE) = (system.nODE2, system.nAE)
    if system.nODE1 or system.nData:
        raise ValueError(name + ': ODE1 and data coordinates are not implemented')
    (n, tStart, tEnd, h) = _Steps(simulationSettings)
    (A, c) = tableau
    s = len(c)
    ggl = constraints == 'GGL'
    implicit = [i for i in range(s) if np.any(A[i] != 0.)]
    unknownZ = implicit if ggl else [i for i in range(s) if c[i] > 0.] #the stages of unknown multipliers
    if not ggl and len(unknownZ) != 2:
        raise ValueError(name + ': the constraints at the end need two stages after the start')
    A2 = A @ A
    #the unknowns: per implicit stage A_i, (z_i, mu_i with GGL); then the multipliers of the end constraints
    index = 0
    (sliceA, sliceZ, sliceMu, rowsAE, rowsV) = ({}, {}, {}, {}, {})
    for i in implicit:
        sliceA[i] = slice(index, index + nODE2)
        index += nODE2
        if ggl:
            (sliceZ[i], sliceMu[i]) = (slice(index, index + nAE), slice(index + nAE, index + 2*nAE))
            (rowsAE[i], rowsV[i]) = (sliceZ[i], sliceMu[i]) #the constraint rows of a stage at the columns of its multipliers
            index += 2*nAE
    if not ggl:
        for i in unknownZ:
            sliceZ[i] = slice(index, index + nAE)
            index += nAE
        (rowsEndAE, rowsEndV) = (sliceZ[unknownZ[0]], sliceZ[unknownZ[1]])
    nUnknowns = index
    errorFactor = np.sqrt(nUnknowns) if (newton.weightTolerancePerCoordinate and nUnknowns) else 1.
    empty = np.zeros(0)

    (q, q_t) = system.InitialState()
    system.data = system.InitialStateData()
    if nAE:
        _RefuseConstraintsBeyondPositions(system, q, q_t, empty, tStart)
    (q_tt, z) = _InitialAccelerations(system, q, q_t, empty, tStart)
    output = _Output(system, simulationSettings, tStart, tEnd, h)
    output.Step(0, tStart, q, q_t, q_tt, z=z)

    def TimeDerivative(qq, t, epsilon=1e-6):
        """c_t, the partial derivative of the constraints by time"""
        return (np.asarray(system.AlgebraicEquations(qq, q_t, empty, z, t + epsilon), dtype=float)
                - np.asarray(system.AlgebraicEquations(qq, q_t, empty, z, t - epsilon), dtype=float))/(2*epsilon)

    def VelocityDerivative(qq, v, t, epsilon=1e-6):
        """C_q and H = (C_q)_q v, the derivative of C_q along v"""
        return (_ConstraintJacobian(system, qq, v, empty, t), (_ConstraintJacobian(system, qq + epsilon*v, v, empty, t)
                - _ConstraintJacobian(system, qq - epsilon*v, v, empty, t))/(2*epsilon))

    (newtonIterations, jacobianEvaluations) = (0, 0)
    stepSize = _ErrorStepSize(simulationSettings, order=3) #the first step: the embedded trapezoidal solution, h^3
    previous = None #the start, the step and the stage velocities and accelerations of the last accepted step
    while not stepSize.Done():
        (t0, t, h) = stepSize.Next()
        (q0, v0, a0, z0) = (q.copy(), q_t.copy(), q_tt.copy(), z.copy())
        times = t0 + h*c
        stageA = [a0.copy() for i in range(s)] #the stage accelerations, the first of IIIA the start of the step
        stageZ = [z0.copy() for i in range(s)]
        stageMu = [np.zeros(nAE) for i in range(s)]
        qIterate = [q0 + (h*c[i])*v0 + (0.5*(h*c[i])**2)*a0 for i in range(s)] #for C_q^T mu

        def Stages():
            """the stage positions and velocities"""
            V = [v0 + h*sum(A[i, j]*stageA[j] for j in range(s)) for i in range(s)]
            Q = [q0 + (h*c[i])*v0 + h**2*sum(A2[i, j]*stageA[j] for j in range(s)) for i in range(s)]
            if ggl and nAE:
                for i in implicit:
                    Q[i] = Q[i] - system.ODE2ReactionForcesLHS(qIterate[i], V[i], empty, stageMu[i], times[i])
            return (Q, V)

        def Residual():
            (Q, V) = Stages()
            r = np.zeros(nUnknowns)
            for i in implicit:
                r[sliceA[i]] = h**2*system.SystemResidual(Q[i], V[i], stageA[i], empty, empty, stageZ[i], times[i])[:nODE2]
            for i in (implicit if ggl else [s - 1]) if nAE else []:
                (ae, vRows) = (rowsAE[i], rowsV[i]) if ggl else (rowsEndAE, rowsEndV)
                r[ae] = system.AlgebraicEquations(Q[i], V[i], empty, stageZ[i], times[i])
                r[vRows] = h*(_ConstraintJacobian(system, Q[i], V[i], empty, times[i]) @ V[i] + TimeDerivative(Q[i], times[i]))
            return r

        def Jacobian():
            (Q, V) = Stages()
            J = np.zeros((nUnknowns, nUnknowns))
            for i in implicit:
                J_q = system.SystemJacobian(Q[i], V[i], stageA[i], empty, empty, stageZ[i], times[i], factor_q=1., factor_q_t=0.,
                                            factor_q_tt=0.)
                J_v = system.SystemJacobian(Q[i], V[i], stageA[i], empty, empty, stageZ[i], times[i], factor_q=0., factor_q_t=1.,
                                            factor_q_tt=0.)[:nODE2, :nODE2]
                M = system.MassMatrix(Q[i], V[i], times[i])
                K = J_q[:nODE2, :nODE2]
                for j in implicit:
                    J[sliceA[i], sliceA[j]] = h**2*((h**2*A2[i, j])*K + (h*A[i, j])*J_v + (M if i == j else 0.))
                if i in sliceZ:
                    J[sliceA[i], sliceZ[i]] = h**2*J_q[:nODE2, nODE2:]
                if ggl and nAE:
                    (C_q, H) = VelocityDerivative(Q[i], V[i], times[i])
                    for j in implicit:
                        J[rowsAE[i], sliceA[j]] = (h**2*A2[i, j])*C_q
                        J[rowsV[i], sliceA[j]] = h*((h*A[i, j])*C_q + (h**2*A2[i, j])*H)
                    J[rowsAE[i], sliceZ[i]] = J_q[nODE2:, nODE2:]
                    J[sliceA[i], sliceMu[i]] = -h**2*K @ C_q.T #dQ_i/dmu_i = -C_q^T
                    J[rowsAE[i], sliceMu[i]] = -C_q @ C_q.T
                    J[rowsV[i], sliceMu[i]] = -h*H @ C_q.T
            if not ggl and nAE: #the end of the step, the last stage
                (C_q, H) = VelocityDerivative(Q[s - 1], V[s - 1], times[s - 1])
                for j in implicit:
                    J[rowsEndAE, sliceA[j]] = (h**2*A2[s - 1, j])*C_q
                    J[rowsEndV, sliceA[j]] = h*((h*A[s - 1, j])*C_q + (h**2*A2[s - 1, j])*H)
            for i in sliceZ: #the multiplier columns scaled by 1/h^2, as the ODE2 rows by h^2
                J[:, sliceZ[i]] /= h**2
            return J

        factorization = None
        converged = False
        initialResidual = residual = None
        for iteration in range(int(newton.maxIterations)):
            if factorization is None or not newton.useModifiedNewton:
                factorization = scipy.linalg.lu_factor(Jacobian())
                jacobianEvaluations += 1
            r = Residual()
            if not np.all(np.isfinite(r)): #diverged: as not converged
                break
            if initialResidual is None:
                initialResidual = max(cMath.norm(r)/errorFactor, 2*newton.absoluteTolerance)
            increment = scipy.linalg.lu_solve(factorization, r)
            if ggl:
                qIterate = Stages()[0]
            for i in implicit:
                stageA[i] = stageA[i] - increment[sliceA[i]]
                if ggl:
                    stageMu[i] = stageMu[i] - increment[sliceMu[i]]
            for i in sliceZ:
                stageZ[i] = stageZ[i] - increment[sliceZ[i]]/h**2
            newtonIterations += 1
            residual = cMath.norm(Residual())/errorFactor
            if residual/initialResidual <= newton.relativeTolerance or residual <= newton.absoluteTolerance:
                converged = True
                break
        if not converged:
            if stepSize.Reduce(-1):
                continue
            raise ValueError(name + ': Newton did not converge in step ' + str(stepSize.stepIndex + 1) + ' at t=' + str(t)
                             + ' after ' + str(int(newton.maxIterations)) + ' iterations; residual ' + str(residual))
        (Q, V) = Stages()
        if stepSize.automatic and previous is None: #the first step: against the trapezoidal rule of the first (or the
            #start) and the last stage, an error of order h^3
            (vFirst, aFirst) = (V[0], stageA[0]) if c[0] == 0. else (v0, a0)
            errors = np.concatenate([Q[-1] - (q0 + (0.5*h)*(vFirst + V[-1])), V[-1] - (v0 + (0.5*h)*(aFirst + stageA[-1]))])
            if not stepSize.Control(stepSize.Error(errors, np.concatenate([q0, v0]), np.concatenate([Q[-1], V[-1]]))):
                continue
        elif stepSize.automatic: #the quadrature of the method against the polynomial through the stage velocities of
            #the previous and this step, of the order of the method (#3250); the accelerations only without constraints,
            #as their stage values are of order 2 with them
            (tPrevious, hPrevious, VPrevious, APrevious) = previous
            taus = [(tPrevious + hPrevious*cj - t0)/h for cj in c]
            older = [j for j in range(s) if min(abs(taus[j] - ck) for ck in c) > 1e-9]
            pointsV = [(taus[j], VPrevious[j]) for j in older] + [(c[j], V[j]) for j in range(s)]
            errors = h*_QuadratureError(pointsV, A[-1], V)
            (values0, values1) = (q0, Q[-1])
            if nAE == 0:
                pointsA = [(taus[j], APrevious[j]) for j in older] + [(c[j], stageA[j]) for j in range(s)]
                errors = np.concatenate([errors, h*_QuadratureError(pointsA, A[-1], stageA)])
                (values0, values1) = (np.concatenate([q0, v0]), np.concatenate([Q[-1], V[-1]]))
            stepSize.order = len(pointsV) #the local error of a method of order len(points) - 1
            if not stepSize.Control(stepSize.Error(errors, values0, values1)):
                continue
        previous = (t0, h, V, [a.copy() for a in stageA])
        (q, q_t, q_tt, z) = (Q[-1], V[-1], stageA[-1], stageZ[-1]) #stiffly accurate: the end is the last stage
        stepSize.Accept(t, 1)
        output.Step(stepSize.stepIndex, t, q, q_t, q_tt, last=stepSize.Done(), z=z, h=h)

    return output.Finalize({'steps': stepSize.stepIndex, 'newtonIterations': newtonIterations,
                            'jacobianEvaluations': jacobianEvaluations, 'rejectedSteps': stepSize.rejected,
                            'rejectedStepsError': stepSize.rejectedByError})


def SolveImplicitRungeKutta(system, simulationSettings=None, method='LobattoIIIA4'):
    """the implicit time integration of a SymbolicSystemBase by an implicit Runge-Kutta method for index-3 constraints
    (#3244, #3246, #3247), of implicitRungeKuttaMethods, each with two implicit stages:

    - 'LobattoIIIA4': Lobatto IIIA with the stages at the start, the middle and the end of the step, the constraints and
      the velocity constraints at the end of the step only and a multiplier per stage after the start, the one of the
      start that of the end of the previous step; of order 4 in q and v, without numerical damping, it keeps the
      energy - the simplest form that solves index 3 directly;
    - 'LobattoIIIC4-GGL': Lobatto IIIC, the stages at the same times, the first implicit as well, with the constraints
      and the velocity constraints at each stage and its positions moved onto the constraints by a multiplier mu (the
      GGL stabilization, as SolveTrapezoidalGGL); of order 4 in q and v, L-stable: it damps fast modes, also with steps
      of 70/omega of the mode;
    - 'Radau3': Radau IIA with the stages at 1/3 and the end of the step, the constraints as LobattoIIIA4; of order 3,
      L-stable.

    All of order 2 in the accelerations and multipliers (measured on a pendulum and a slider crank) and stiffly
    accurate: the end of the step is the last stage. With timeIntegration.automaticStepSize (the default) the step
    size is controlled by an estimate of the local error of the order of the method (#3250): the quadrature of the
    method, h sum_j b_j V_j, against the integral of the polynomial through the stage velocities of the previous and
    this step - for the Lobatto methods of degree 4, Simpson's error h^5/2880 v'''' -, and the same for the stage
    accelerations without constraints (with them, their stage values are of order 2, and the error of q decides); no
    extra solve. It is 0.3 to 0.6 times the true local error, for all step sizes; the global error is then of the size
    of the tolerances. The first step, without a previous one, by the embedded trapezoidal solution of the first (or
    the start) and the last stage, an error of order h^3. The PI controller and the settings as SolveTrapezoidalGGL;
    without it the steps of timeIntegration.numberOfSteps. Newton's method for the stage accelerations and multipliers (and mu)
    with the Jacobian of the system, the ODE2 rows scaled by h^2, by timeIntegration.newton as SolveTrapezoidalGGL; a
    step whose Newton iteration does not converge is repeated with a smaller step with automaticStepSize or
    adaptiveStep, else raises. Consistent initial accelerations and algebraic coordinates; the output as the solvers
    of exudyn.symbolics.solvers. The algebraic equations must depend on q and t only; ODE1 and data coordinates are not
    implemented

    Args:
        system: the system, a SymbolicSystemBase
        simulationSettings: exudyn.SimulationSettings; None: the defaults
        method: 'LobattoIIIA4', 'LobattoIIIC4-GGL' or 'Radau3'

    Returns:
        dict with 'time' (n_out), 'q', 'q_t', 'q_tt' (n_out x nODE2), with algebraic equations 'z', at the times of
        solution.file.writePeriod and the end, and the statistics 'steps', 'newtonIterations', 'jacobianEvaluations',
        'rejectedSteps' (by Newton) and 'rejectedStepsError'
    """
    if method not in implicitRungeKuttaMethods:
        raise ValueError('SolveImplicitRungeKutta: method must be one of ' + str(list(implicitRungeKuttaMethods))
                         + ', not ' + str(method))
    (tableau, constraints) = implicitRungeKuttaMethods[method]
    return _SolveRungeKutta(system, simulationSettings, _rungeKuttaTableaus[tableau], constraints, 'SolveImplicitRungeKutta')
