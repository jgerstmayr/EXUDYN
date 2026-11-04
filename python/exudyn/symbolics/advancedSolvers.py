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
    'SolveTrapezoidalGGL', 'lobatto4Variants', 'SolveLobatto4',
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



#the tableaus (A, c) of the Lobatto methods of the stages 0, 1/2, 1 (and of the trapezoidal rule, the 2-stage IIIA)
_lobattoTableaus = {'IIIA': (np.array([[0., 0., 0.], [5./24., 1./3., -1./24.], [1./6., 2./3., 1./6.]]), np.array([0., 0.5, 1.])),
                    'IIIC': (np.array([[1./6., -1./3., 1./6.], [1./6., 5./12., -1./12.], [1./6., 2./3., 1./6.]]), np.array([0., 0.5, 1.])),
                    'IIIA2': (np.array([[0., 0.], [0.5, 0.5]]), np.array([0., 1.]))}

#the variants of SolveLobatto4: (tableau, GGL stabilization)
lobatto4Variants = {'IIIC-GGL': ('IIIC', True), 'IIIA-GGL': ('IIIA', True), 'IIIC': ('IIIC', False)}


def _SolveLobatto(system, simulationSettings, tableau, ggl, name):
    """the stage equations of a stiffly accurate implicit Runge-Kutta method of the tableau (A, c) for q_t = v, v_t = a,
    M a = f - C_q^T z and the constraints at the implicit stages, with constant steps; the stages of a first row of
    zeros are the start of the step (Lobatto IIIA); with ggl also C_q V_i + c_t = 0 and the positions Q_i - C_q^T mu_i
    at the implicit stages (#3244)"""
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
    implicit = [i for i in range(s) if np.any(A[i] != 0.)]
    A2 = A @ A
    nMu = nAE if ggl else 0
    nStage = nODE2 + nAE + nMu #the rows of a stage: ODE2, constraints, (velocity constraints)
    (rowsODE2, rowsAE, rowsV) = (slice(0, nODE2), slice(nODE2, nODE2 + nAE), slice(nODE2 + nAE, nODE2 + nAE + nMu))
    nUnknowns = len(implicit)*nStage
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

    (newtonIterations, jacobianEvaluations) = (0, 0)
    for step in range(n):
        (t0, h) = (tStart + step*h, h)
        times = t0 + h*c
        (q0, v0, a0) = (q.copy(), q_t.copy(), q_tt.copy())
        stageA = [a0.copy() for i in range(s)] #the stage accelerations, the first of IIIA the start of the step
        stageZ = [z.copy() for i in range(s)]
        stageMu = [np.zeros(nMu) for i in range(s)]
        qIterate = [q0 + (h*c[i])*v0 + (0.5*(h*c[i])**2)*a0 for i in range(s)] #for C_q^T mu

        def Stages():
            """the stage positions and velocities"""
            V = [v0 + h*sum(A[i, j]*stageA[j] for j in range(s)) for i in range(s)]
            Q = [q0 + (h*c[i])*v0 + h**2*sum(A2[i, j]*stageA[j] for j in range(s)) for i in range(s)]
            if ggl:
                for i in implicit:
                    Q[i] = Q[i] - system.ODE2ReactionForcesLHS(qIterate[i], V[i], empty, stageMu[i], times[i])
            return (Q, V)

        def Residual():
            (Q, V) = Stages()
            r = np.zeros(nUnknowns)
            for (k, i) in enumerate(implicit):
                rows = r[k*nStage:(k + 1)*nStage]
                rows[:nODE2 + nAE] = system.SystemResidual(Q[i], V[i], stageA[i], empty, empty, stageZ[i], times[i])
                rows[rowsODE2] *= h**2
                if ggl and nAE:
                    rows[rowsV] = h*(_ConstraintJacobian(system, Q[i], V[i], empty, times[i]) @ V[i] + TimeDerivative(Q[i], times[i]))
            return r

        def Jacobian():
            (Q, V) = Stages()
            J = np.zeros((nUnknowns, nUnknowns))
            for (k, i) in enumerate(implicit):
                J_q = system.SystemJacobian(Q[i], V[i], stageA[i], empty, empty, stageZ[i], times[i], factor_q=1., factor_q_t=0.,
                                            factor_q_tt=0.)
                J_v = system.SystemJacobian(Q[i], V[i], stageA[i], empty, empty, stageZ[i], times[i], factor_q=0., factor_q_t=1.,
                                            factor_q_tt=0.)[:, :nODE2]
                M = system.MassMatrix(Q[i], V[i], times[i])
                C_q = J_q[rowsAE, :nODE2]
                rows = slice(k*nStage, k*nStage + nODE2 + nAE)
                if ggl and nAE:
                    epsilon = 1e-6 #H = (C_q)_q v, the derivative of C_q along v
                    H = (_ConstraintJacobian(system, Q[i] + epsilon*V[i], V[i], empty, times[i])
                         - _ConstraintJacobian(system, Q[i] - epsilon*V[i], V[i], empty, times[i]))/(2*epsilon)
                for (m, j) in enumerate(implicit):
                    columns = slice(m*nStage, m*nStage + nODE2)
                    J[rows, columns] = (h**2*A2[i, j])*J_q[:, :nODE2] + (h*A[i, j])*J_v
                    if i == j:
                        J[k*nStage:k*nStage + nODE2, columns] += M
                    if ggl and nAE:
                        J[k*nStage + nODE2 + nAE:(k + 1)*nStage, columns] = h*((h*A[i, j])*C_q + (h**2*A2[i, j])*H)
                J[rows, k*nStage + nODE2:k*nStage + nODE2 + nAE] = J_q[:, nODE2:]
                if ggl and nAE:
                    J[rows, k*nStage + nODE2 + nAE:(k + 1)*nStage] = -J_q[:, :nODE2] @ C_q.T #dQ_i/dmu_i = -C_q^T
                    J[k*nStage + nODE2 + nAE:(k + 1)*nStage, k*nStage + nODE2 + nAE:(k + 1)*nStage] = -h*H @ C_q.T
            for k in range(len(implicit)): #the ODE2 rows scaled by h^2, the multiplier columns by 1/h^2
                J[k*nStage:k*nStage + nODE2, :] *= h**2
                J[:, k*nStage + nODE2:k*nStage + nODE2 + nAE] /= h**2
            return J

        factorization = None
        converged = False
        initialResidual = residual = None
        for iteration in range(int(newton.maxIterations)):
            if factorization is None or not newton.useModifiedNewton:
                factorization = scipy.linalg.lu_factor(Jacobian())
                jacobianEvaluations += 1
            r = Residual()
            if not np.all(np.isfinite(r)):
                break
            if initialResidual is None:
                initialResidual = max(cMath.norm(r)/errorFactor, 2*newton.absoluteTolerance)
            increment = scipy.linalg.lu_solve(factorization, r)
            if ggl:
                qIterate = Stages()[0]
            for (k, i) in enumerate(implicit):
                stageA[i] = stageA[i] - increment[k*nStage:k*nStage + nODE2]
                stageZ[i] = stageZ[i] - increment[k*nStage + nODE2:k*nStage + nODE2 + nAE]/h**2
                stageMu[i] = stageMu[i] - increment[k*nStage + nODE2 + nAE:(k + 1)*nStage]
            newtonIterations += 1
            residual = cMath.norm(Residual())/errorFactor
            if residual/initialResidual <= newton.relativeTolerance or residual <= newton.absoluteTolerance:
                converged = True
                break
        if not converged:
            raise ValueError(name + ': Newton did not converge in step ' + str(step + 1) + ' at t=' + str(t0 + h)
                             + ' after ' + str(int(newton.maxIterations)) + ' iterations; residual ' + str(residual))
        (Q, V) = Stages()
        (q, q_t, q_tt, z) = (Q[-1], V[-1], stageA[-1], stageZ[-1]) #stiffly accurate: the end is the last stage
        output.Step(step + 1, t0 + h, q, q_t, q_tt, last=step + 1 == n, z=z, h=h)

    return output.Finalize({'steps': n, 'newtonIterations': newtonIterations, 'jacobianEvaluations': jacobianEvaluations})


def SolveLobatto4(system, simulationSettings=None, variant='IIIC-GGL'):
    """the implicit time integration of a SymbolicSystemBase by a 3-stage Lobatto method of order 4, with the stages
    at the start, the middle and the end of the step and constant steps (#3244), in three variants for index-3
    constraints (lobatto4Variants): 'IIIC-GGL', Lobatto IIIC - three implicit stages, L-stable, damping fast modes -,
    and 'IIIA-GGL', Lobatto IIIA - its first stage the start of the step, two implicit stages, without damping -, both
    with the constraints and the velocity constraints at the implicit stages and the positions of each stage moved
    onto the constraints by a multiplier mu, as SolveTrapezoidalGGL does for the trapezoidal rule, the 2-stage Lobatto
    IIIA; 'IIIC', Lobatto IIIC with the constraints at all stages only. All are stiffly accurate: the end of the step
    is the last stage. Measured on a pendulum and a slider crank: the GGL variants of order 4 in q and v and of order
    2 in the accelerations and z, IIIA-GGL with errors of the accelerations 25 times smaller; 'IIIC' of order 4 in q,
    2 in v and 1 in the accelerations, with a drift of the velocity constraints; with a fast mode of h omega = 70,
    IIIC-GGL damps it and IIIA-GGL does not converge. Newton's method for the stage accelerations, multipliers (and
    mu) with the Jacobian of the system, the ODE2 rows scaled by h^2, by timeIntegration.newton as
    SolveTrapezoidalGGL; the steps of timeIntegration.numberOfSteps, startTime and endTime; consistent initial
    accelerations and algebraic coordinates; the output as the solvers of exudyn.symbolics.solvers. The algebraic
    equations must depend on q and t only; ODE1 and data coordinates are not implemented

    Args:
        system: the system, a SymbolicSystemBase
        simulationSettings: exudyn.SimulationSettings; None: the defaults
        variant: 'IIIC-GGL', 'IIIA-GGL' or 'IIIC'

    Returns:
        dict with 'time' (n_out), 'q', 'q_t', 'q_tt' (n_out x nODE2), with algebraic equations 'z', at the times of
        solution.file.writePeriod and the end, and the statistics 'steps', 'newtonIterations', 'jacobianEvaluations'
    """
    if variant not in lobatto4Variants:
        raise ValueError('SolveLobatto4: variant must be one of ' + str(list(lobatto4Variants)) + ', not ' + str(variant))
    (tableau, ggl) = lobatto4Variants[variant]
    return _SolveLobatto(system, simulationSettings, _lobattoTableaus[tableau], ggl, 'SolveLobatto4')
