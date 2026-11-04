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
import exudyn
import exudyn.cMath as cMath
from exudyn.symbolics.solvers import (_Output, _Steps, _StepSize, _PostNewton, _InitialAccelerations,
                                      _RefuseLieGroupNodes)
try:
    import scipy.linalg
except ImportError:
    raise ImportError('exudyn.symbolics.advancedSolvers needs scipy: pip install scipy')

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = ['SolveTrapezoidalGGL']


class _ErrorStepSize(_StepSize):
    """the step size of a solver of this module: with timeIntegration.automaticStepSize controlled by an estimate of
    the local error, as the automatic step size of Exudyn's explicit solvers (#3242) - the error in the norm of
    absoluteTolerance and relativeTolerance, h (1/error)^(1/order) times stepSizeSafety as the next step, at most
    stepSizeMaxIncrease times h and the maximum (endTime - startTime)/numberOfSteps, at least minimumStepSize, a step
    of error > 1 repeated with it; the first step of initialStepSize, 0: the maximum. A post-Newton step or a Newton
    iteration that fails reduces the step as _StepSize. Without automaticStepSize the steps of _StepSize"""
    def __init__(self, simulationSettings, order):
        _StepSize.__init__(self, simulationSettings, explicit=False)
        timeIntegration = simulationSettings.timeIntegration
        self.automatic = timeIntegration.automaticStepSize
        self.order = order
        (self.absoluteTolerance, self.relativeTolerance) = (timeIntegration.absoluteTolerance, timeIntegration.relativeTolerance)
        (self.safety, self.maxIncrease) = (timeIntegration.stepSizeSafety, timeIntegration.stepSizeMaxIncrease)
        self.rejectedByError = 0
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
        hOpt = self.h*cMath.pow(1./max(error, 1e-12), 1./self.order)
        hNew = min(self.hMax, self.maxIncrease*self.h, max(self.hMin, self.safety*hOpt))
        if hOpt < self.h and self.h > self.hMin:
            (self.h, self.rejectedByError) = (hNew, self.rejectedByError + 1)
            return False
        self.hNext = hNew
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
    only; C_q comes from the reaction forces of the system. With timeIntegration.automaticStepSize (the default, as for the explicit solvers DOPRI5 and ODE23) the step size
    is controlled by the local error of the positions, h^3/12 q_ttt = h^2/12 (a1 - a0), with
    timeIntegration.absoluteTolerance, relativeTolerance, stepSizeSafety, stepSizeMaxIncrease, initialStepSize and
    minimumStepSize, the maximum step (endTime - startTime)/numberOfSteps; the tolerances are those of a step, the
    global error grows with the number of steps. The velocities and the ODE1 coordinates are not in the estimate.
    Without automaticStepSize the steps are constant, as SolveGeneralizedAlpha. The initial accelerations and
    algebraic coordinates are consistent ([M C_q^T; C_q 0] [a0; z0] = [f; -(C_q v)_q v]), the initial ODE1 velocities
    those of the ODE1 equations. Newton's method by timeIntegration.newton (relativeTolerance, absoluteTolerance,
    maxIterations, useModifiedNewton,
    weightTolerancePerCoordinate); a step whose Newton iteration does not converge is repeated with a smaller step
    with automaticStepSize or adaptiveStep, else raises. Data coordinates and adaptiveStep as SolveGeneralizedAlpha;
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
    stepSize = _ErrorStepSize(simulationSettings, order=3) #the local error is of order h^3
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
            if stepSize.automatic: #the local error h^3/12 q''' of the positions
                error = stepSize.Error((h**2/12.)*(a - a0), q0, q)
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
        output.Step(stepSize.stepIndex, t, q, q_t, q_tt, last=stepSize.Done(), y=y, y_t=y_t, z=z, h=h)

    return output.Finalize({'steps': stepSize.stepIndex, 'newtonIterations': newtonIterations,
                            'jacobianEvaluations': jacobianEvaluations, 'rejectedSteps': stepSize.rejected,
                            'rejectedStepsError': stepSize.rejectedByError, **statistics})
