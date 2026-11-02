#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  The equations of motion of a system of user items as symbolic programs; see the chapter "Symbolic system
#           and user functions" of the user manual. SymbolicSystem(mbs) collects them, after mbs.Assemble(), from the
#           user items with symbolic user functions, as UserItemCreator of exudyn.symbolics.userItems creates and
#           records them: M(q, t) q_tt = f(q, q_t, y, t) - C_q^T z, y_t = g(q, q_t, y, t), c(q, q_t, y, z, t) = 0, with
#           the data coordinates of the post-Newton steps. It evaluates them - ODE2RHS, MassMatrix, SystemResidual, the
#           exact SystemJacobian -, gives them as expressions (Equations, also as sympy) or as a flat program
#           (GetFlatProgram), and writes a state into mbs (SetSystemState). SymbolicSystemBase is the interface on
#           which the solvers of exudyn.symbolics.solvers work; NumpySystem takes equations written by hand;
#           SystemVariables gives the variables of the system and the states of the markers as expressions. What
#           each class and function takes and refuses is in its docstring.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-26 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import exudyn
from exudyn.symbolics.codeExport import ToFunction
try:
    import scipy.linalg
    import scipy.sparse
    import scipy.sparse.linalg
except ImportError:
    raise ImportError('exudyn.symbolics.systemModel needs scipy for its sparse matrices: pip install scipy')

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = ['SymbolicSystemBase', 'NumpySystem', 'SymbolicKind', 'SymbolicLoad', 'SymbolicSystem', 'SystemVariables']

#the user functions of a kind, by object type
_userFunctionNames = {'UserBody': ['massMatrixUserFunction', 'forceUserFunction'],
                      'GenericODE2': ['massMatrixUserFunction', 'forceUserFunction'],
                      'ConnectorUser': ['forceUserFunction', 'postNewtonStepUserFunction']}
#the user functions of a joint, ObjectJointUser on two rigid markers (#3141)
_jointFunctionNames = ['constraintUserFunction', 'jacobianUserFunction']
#the user functions of ObjectGenericDAE, (mbs, t, itemNumber, q, q_t, y, parameters) - the constraintUserFunction with
#z and velocityLevel before parameters -, and the positions of q, q_t and y in them, mbs at 0 (#3190, #3191)
_daeFunctionNames = ['forceUserFunction', 'rhsUserFunction', 'massMatrixUserFunction', 'constraintUserFunction',
                     'postNewtonStepUserFunction']
(_daeQ, _daeQ_t, _daeY) = (3, 4, 5)
#the matrices and vectors of ObjectGenericDAE by their symbols in the equations of its description
_daeMatrices = {'M': 'massMatrix', 'D': 'dampingMatrix', 'K': 'stiffnessMatrix', 'E': 'couplingMatrixODE1',
                'A': 'systemMatrix', 'B': 'couplingMatrixODE2', 'B_t': 'couplingMatrixODE2_t',
                'C_q': 'constraintMatrixODE2', 'C_y': 'constraintMatrixODE1'}
_daeVectors = {'f': 'forceVector', 'g': 'rhsVector', 'c0': 'constraintVector', 'parameters': 'parameters'}
#the positions of the arguments q (the total coordinates for UserBody) and q_t in the user functions, mbs at 0
_argumentPositions = {'UserBody': (4, 5), 'GenericODE2': (3, 4)}
#the user function of ObjectUserBody that projects a load on a body marker into its coordinates (#3118), and the
#positions of qTotal and q_t in it; the functions of a local position get it after itemNumber
_loadProjections = {'ForceVector': 'positionJacobianUserFunction', 'TorqueVector': 'rotationJacobianUserFunction',
                    'MassProportional': 'massIntegralUserFunction'}
_bodyArgumentPositions = {'positionJacobianUserFunction': (5, 6), 'rotationJacobianUserFunction': (5, 6),
                          'rotationMatrixUserFunction': (5, 6), 'massIntegralUserFunction': (4, 5)}
#the states of the markers of a connector, [position, velocity, rotation, angular velocity] each, as the leaves of the
#frame and the twist of each marker in a recorded forceUserFunction: rotation, translation, velocity, angular velocity
#(#3183)
def _ConnectorLeaves(states):
    leaves = []
    for k in range(0, len(states), 4):
        (position, velocity, rotation, angularVelocity) = states[k:k + 4]
        leaves += [rotation, position, velocity, angularVelocity]
    return leaves

#the states of the markers of a joint as the leaves of the frame of each marker in its recorded user functions: the
#rotation and the translation
def _JointLeaves(states):
    return [states[2], states[0], states[6], states[4]]

#the user functions of ObjectUserBody a connector marker on it needs, without and with the rotations (#3129)
_markerFunctions = {False: ['positionUserFunction', 'positionJacobianUserFunction'],
                    True: ['positionUserFunction', 'positionJacobianUserFunction', 'rotationMatrixUserFunction',
                           'rotationJacobianUserFunction']}


class SymbolicSystemBase:
    """the equations of a system, the interface on which the solvers work (#3113, #3134), in Exudyn's form: nODE2
    second order coordinates q, nODE1 first order coordinates y and nAE algebraic coordinates z - the Lagrange
    multipliers lambda of the documentation, named z in the code, as lambda is a keyword of Python - with

        M(q, t) q_tt = f(q, q_t, t, y) - C_q^T z      (ODE2, the reaction forces C_q^T z by ODE2ReactionForcesLHS)
        y_t = g(q, q_t, y, t)                               (ODE1, ODE1RHS)
        c(q, q_t, y, z, t) = 0                              (AE, AlgebraicEquations)

    A derived class sets nODE2 (nODE1 and nAE, zero by default) and computes ODE2RHS and MassMatrixTriplets (or
    MassMatrix), and ODE1RHS, AlgebraicEquations and ODE2ReactionForcesLHS if it has such coordinates; the residuals
    and their Jacobians follow from them, unless the derived class computes them itself. Any equation may depend on
    the coordinates of every kind, so that an item coupling all of them fits

    Note:
        Its members: nODE2, nODE1, nAE, nData, the numbers of coordinates; data, the data coordinates x, which every
        equation reads and which are constant during the Newton iterations of a step: the solvers set them, after
        the post-Newton step of PostNewton (#3196)
    """
    nODE2 = 0
    nODE1 = 0
    nAE = 0
    nData = 0
    data = np.zeros(0)

    def InitialStateData(self):
        """the initial data coordinates x0; empty unless the derived class has data coordinates"""
        return np.zeros(self.nData)

    def PostNewton(self, q, q_t, y, z, x, x0, t):
        """the post-Newton step after the Newton iterations of a step (#3196): the new data coordinates, the
        discontinuous error and a recommended step size (< 0: none); the data coordinates x are constant during the
        Newton iterations, x0 those at the start of the step. The base class has no data coordinates

        Args:
            q: the ODE2 coordinates
            q_t: the ODE2 velocities
            y: the ODE1 coordinates
            z: the algebraic coordinates
            x: the data coordinates of the last iteration
            x0: the data coordinates at the start of the step
            t: the time

        Returns:
            (x, error, recommendedStepSize)
        """
        return (np.array(x, dtype=float), 0., -1.)

    def InitialState(self):
        """the initial coordinates and velocities (q0, q_t0); zero unless the derived class gives them"""
        return np.zeros(self.nODE2), np.zeros(self.nODE2)

    def InitialStateODE1(self):
        """the initial ODE1 coordinates y0; zero unless the derived class gives them"""
        return np.zeros(self.nODE1)

    def IsMassMatrixConstant(self):
        """True if the mass matrix depends neither on q nor on t, so that a solver factorizes it once"""
        return False

    def ODE2RHS(self, q=None, q_t=None, t=0., y=None):
        """the forces f(q, q_t, t, y) of the right-hand side, of size nODE2, without the reaction forces; y, the ODE1
        coordinates, is given only if nODE1 > 0"""
        raise NotImplementedError('SymbolicSystemBase.ODE2RHS: computed by the derived class')

    def ODE1RHS(self, q, q_t, y, t):
        """the right-hand side g(q, q_t, y, t) = y_t of the ODE1 coordinates, of size nODE1"""
        if self.nODE1:
            raise NotImplementedError('SymbolicSystemBase.ODE1RHS: computed by the derived class')
        return np.zeros(0)

    def AlgebraicEquations(self, q, q_t, y, z, t, velocityLevel=False):
        """the algebraic equations c(q, q_t, y, z, t), of size nAE; velocityLevel: their time derivative (index 2)"""
        if self.nAE:
            raise NotImplementedError('SymbolicSystemBase.AlgebraicEquations: computed by the derived class')
        return np.zeros(0)

    def ODE2ReactionForcesLHS(self, q, q_t, y, z, t):
        """the reaction forces C_q^T z of the algebraic coordinates, of size nODE2, which the ODE2 residual adds"""
        if self.nAE:
            raise NotImplementedError('SymbolicSystemBase.ODE2ReactionForcesLHS: computed by the derived class')
        return np.zeros(self.nODE2)

    def ODE1ReactionTermsLHS(self, q, q_t, y, z, t):
        """the reaction terms C_y^T z of the algebraic coordinates in the ODE1 equations, on their left-hand side, of
        size nODE1 (#3155); zero, unless a derived class has algebraic equations of ODE1 coordinates"""
        return np.zeros(self.nODE1)

    def MassMatrixTriplets(self, q=None, q_t=None, t=0.):
        """the mass matrix as triplets (rows, columns, values); equal rows and columns are added"""
        raise NotImplementedError('SymbolicSystemBase.MassMatrixTriplets: computed by the derived class')

    def MassMatrix(self, q=None, q_t=None, t=0., sparse=False):
        """the mass matrix M(q, t); sparse=True: a scipy.sparse.csr_matrix, else a dense array"""
        return self._Matrix(*self.MassMatrixTriplets(q, q_t, t), sparse)

    def MassMatrixBlocks(self, q=None, q_t=None, t=0.):
        """the mass matrix in blocks of bodies: a list of (ltg, blocks), ltg the global indices (n_b x n) of n_b bodies
        of n coordinates and blocks their mass matrices (n_b x n x n); for the inverse per body of the explicit solvers
        (#3112). The base class gives one block of all coordinates"""
        return [(np.arange(self.nODE2)[np.newaxis, :], self.MassMatrix(q, q_t, t)[np.newaxis, :, :])]

    def ODE2ResidualLHS(self, q=None, q_t=None, q_tt=None, t=0., y=None):
        """the residual r = M(q, t) q_tt - f(q, q_t, t, y) of the ODE2 equations, all terms on the left-hand side (LHS
        minus RHS), of size nODE2, without reaction forces"""
        (q, q_t, q_tt) = self._State(q, q_t, q_tt)
        forces = self.ODE2RHS(q, q_t, t) if y is None else self.ODE2RHS(q, q_t, t, y=y)
        return self.MassMatrix(q, q_t, t, sparse=True) @ q_tt - forces

    def SystemResidual(self, q, q_t, q_tt, y, y_t, z, t, velocityLevel=False):
        """the residual of all equations as the implicit solver of Exudyn has it (#3134): [M q_tt - f + C_q^T z,
        g - y_t - C_y^T z, c], of size nODE2 + nODE1 + nAE"""
        yArgument = y if self.nODE1 else None
        parts = [self.ODE2ResidualLHS(q, q_t, q_tt, t, y=yArgument)]
        if self.nAE:
            parts[0] = parts[0] + self.ODE2ReactionForcesLHS(q, q_t, y, z, t)
        if self.nODE1: #the ODE1 rows RHS minus LHS, as CSolverImplicitSecondOrder
            parts.append(self.ODE1RHS(q, q_t, y, t) - y_t - (self.ODE1ReactionTermsLHS(q, q_t, y, z, t) if self.nAE else 0.))
        if self.nAE:
            parts.append(self.AlgebraicEquations(q, q_t, y, z, t, velocityLevel))
        return np.concatenate(parts)

    def SystemJacobian(self, q, q_t, q_tt, y, y_t, z, t, factor_q=1., factor_q_t=0., factor_q_tt=0., factor_y=1.,
                       factor_y_t=0., velocityLevel=False, epsilon=1e-6):
        """the Jacobian of SystemResidual (#3134), column by column: ODE2 columns factor_q d/dq + factor_q_t d/dq_t +
        factor_q_tt d/dq_tt, ODE1 columns factor_y d/dy + factor_y_t d/dy_t, AE columns d/dz; the ODE2 block by
        ODE2ResidualLHSJacobian (exact for a SymbolicSystem), the others by central differences of SystemResidual

        Args:
            q: the ODE2 coordinates
            q_t: the ODE2 velocities
            q_tt: the ODE2 accelerations
            y: the ODE1 coordinates
            y_t: their velocities
            z: the algebraic coordinates, the Lagrange multipliers lambda of the documentation
            t: the time
            factor_q: the factor of d/dq
            factor_q_t: the factor of d/dq_t
            factor_q_tt: the factor of d/dq_tt
            factor_y: the factor of d/dy
            factor_y_t: the factor of d/dy_t
            velocityLevel: the algebraic equations on velocity level
            epsilon: the step of the differences

        Returns:
            the dense matrix (nSystem x nSystem), nSystem = nODE2 + nODE1 + nAE
        """
        (nODE2, nODE1, nAE) = (self.nODE2, self.nODE1, self.nAE)
        nSystem = nODE2 + nODE1 + nAE
        state = [np.array(x, dtype=float) for x in [q, q_t, q_tt, y, y_t, z]]
        J = np.zeros((nSystem, nSystem))
        if nODE1:
            J[:nODE2, :nODE2] = self.ODE2ResidualLHSJacobian(q, q_t, q_tt, t, factor_q, factor_q_t, factor_q_tt, y=y)
        else:
            J[:nODE2, :nODE2] = self.ODE2ResidualLHSJacobian(q, q_t, q_tt, t, factor_q, factor_q_t, factor_q_tt)

        def Difference(direction, rows): #the derivative of the residual rows along a direction of the state
            residuals = []
            for sign in [1., -1.]:
                perturbed = [x + sign*epsilon*d for (x, d) in zip(state, direction)]
                residuals.append(self.SystemResidual(*perturbed, t, velocityLevel)[rows] if rows != 'reaction'
                                 else self.ODE2ReactionForcesLHS(perturbed[0], perturbed[1], perturbed[3], perturbed[5], t))
            return (residuals[0] - residuals[1])/(2*epsilon)

        zeros = [np.zeros(len(x)) for x in state]
        for j in range(nSystem):
            direction = [x.copy() for x in zeros]
            if j < nODE2: #the ODE2 block is there; the reaction forces and the rows of the other equations
                for (k, factor) in enumerate([factor_q, factor_q_t, factor_q_tt]):
                    direction[k][j] = factor
                if nAE:
                    J[:nODE2, j] += Difference(direction, 'reaction')
                if nODE1 + nAE:
                    J[nODE2:, j] = Difference(direction, slice(nODE2, nSystem))
            elif j < nODE2 + nODE1:
                direction[3][j - nODE2] = factor_y
                direction[4][j - nODE2] = factor_y_t
                J[:, j] = Difference(direction, slice(0, nSystem))
            else:
                direction[5][j - nODE2 - nODE1] = 1.
                J[:, j] = Difference(direction, slice(0, nSystem))
        return J

    def ODE2ResidualLHSJacobian(self, q=None, q_t=None, q_tt=None, t=0., factor_q=1., factor_q_t=0., factor_q_tt=0.,
                         sparse=False, epsilon=1e-6, y=None):
        """the Jacobian of the residual, factor_q dr/dq + factor_q_t dr/dq_t + factor_q_tt dr/dq_tt, by central
        differences of ODE2ResidualLHS - for a system without derivatives of its own; factor_q_tt=1 alone gives the mass
        matrix, a Newton step of an implicit integrator combines the three

        Args:
            q: the ODE2 coordinates, of size nODE2; None: zero
            q_t: the ODE2 velocities; None: zero
            q_tt: the ODE2 accelerations; None: zero
            t: the time
            factor_q: the factor of dr/dq
            factor_q_t: the factor of dr/dq_t
            factor_q_tt: the factor of dr/dq_tt
            sparse: True: a scipy.sparse.csr_matrix, else a dense array
            epsilon: the step of the differences
            y: the ODE1 coordinates, if the forces depend on them

        Returns:
            the matrix (nODE2 x nODE2)
        """
        state = list(self._State(q, q_t, q_tt))
        J = np.zeros((self.nODE2, self.nODE2))
        for (k, factor) in enumerate([factor_q, factor_q_t, factor_q_tt]):
            if factor == 0.:
                continue
            for j in range(self.nODE2):
                residuals = []
                for sign in [1., -1.]:
                    perturbed = [x.copy() for x in state]
                    perturbed[k][j] += sign*epsilon
                    residuals.append(self.ODE2ResidualLHS(*perturbed, t, y=y))
                J[:, j] += factor*(residuals[0] - residuals[1])/(2*epsilon)
        (rows, columns) = np.nonzero(J)
        return self._Matrix(rows, columns, J[rows, columns], sparse)

    def _State(self, q, q_t, q_tt=None):
        """the state as float arrays; None: zero"""
        return tuple(np.zeros(self.nODE2) if x is None else np.asarray(x, dtype=float) for x in [q, q_t, q_tt])

    def _Matrix(self, rows, columns, values, sparse):
        """the matrix (nODE2 x nODE2) of triplets, dense or as scipy.sparse.csr_matrix"""
        if sparse:
            return scipy.sparse.csr_matrix((values, (rows, columns)), shape=(self.nODE2, self.nODE2))
        matrix = np.zeros((self.nODE2, self.nODE2))
        np.add.at(matrix, (rows, columns), values)
        return matrix


class NumpySystem(SymbolicSystemBase):
    """equations written by hand, e.g. with numpy: the simplest system for a solver of this module, and a check of it
    independent of Exudyn (#3112); ODE2 equations M q_tt = f, optionally ODE1 coordinates and constraints (#3134)

    Args:
        massMatrix: the mass matrix, constant (numpy array or scipy sparse matrix) or a function (q, q_t, t) -> matrix
        ode2RHS: the function (q, q_t, t) -> f of the right-hand side, of size nODE2; with ODE1 coordinates (q, q_t, t,
            y) -> f
        initialState: (q0, q_t0); None: zero
        massMatrixBlocks: the blocks of bodies as SymbolicSystemBase.MassMatrixBlocks returns them, for a constant mass
            matrix; None: one block of all coordinates
        ode1RHS: None, or the function (q, q_t, y, t) -> y_t of the ODE1 coordinates
        initialStateODE1: the initial ODE1 coordinates y0, which also give their number
        constraints: None, or the function (q, q_t, y, z, t) -> c of the algebraic equations on position level
        constraintJacobian: the function (q, q_t, y, t) -> C_q (nAE x nODE2) of the reaction forces C_q^T z
        numberOfConstraints: nAE
    """
    def __init__(self, massMatrix, ode2RHS, initialState=None, massMatrixBlocks=None, ode1RHS=None,
                 initialStateODE1=None, constraints=None, constraintJacobian=None, numberOfConstraints=0):
        self._massMatrix = massMatrix
        self._forces = ode2RHS
        self.nODE2 = (len(initialState[0]) if initialState is not None else massMatrix.shape[0])
        self._initialState = initialState
        self._massMatrixBlocks = massMatrixBlocks
        self._ode1 = ode1RHS
        self._y0 = np.array(initialStateODE1 if initialStateODE1 is not None else [], dtype=float)
        self.nODE1 = len(self._y0) if ode1RHS is not None else 0
        self._constraints = constraints
        self._constraintJacobian = constraintJacobian
        self.nAE = int(numberOfConstraints) if constraints is not None else 0

    def InitialStateODE1(self):
        """y0 as given"""
        return self._y0.copy()

    def ODE1RHS(self, q, q_t, y, t):
        """g(q, q_t, y, t) of the function given"""
        if self._ode1 is None:
            return np.zeros(0)
        return np.asarray(self._ode1(q, q_t, y, t), dtype=float)

    def AlgebraicEquations(self, q, q_t, y, z, t, velocityLevel=False):
        """c of the function given; velocity level not supported"""
        if velocityLevel:
            raise ValueError('NumpySystem.AlgebraicEquations: the constraints are given on position level only')
        if self._constraints is None:
            return np.zeros(0)
        return np.asarray(self._constraints(q, q_t, y, z, t), dtype=float)

    def ODE2ReactionForcesLHS(self, q, q_t, y, z, t):
        """C_q^T z of the constraint Jacobian given"""
        if self._constraints is None:
            return np.zeros(self.nODE2)
        return np.asarray(self._constraintJacobian(q, q_t, y, t), dtype=float).T @ z

    def InitialState(self):
        """(q0, q_t0) as given, zero otherwise"""
        if self._initialState is None:
            return SymbolicSystemBase.InitialState(self)
        return tuple(np.array(x, dtype=float) for x in self._initialState)

    def IsMassMatrixConstant(self):
        """True for a mass matrix given as matrix"""
        return not callable(self._massMatrix)

    def ODE2RHS(self, q=None, q_t=None, t=0., y=None):
        """f(q, q_t, t) of the function given, or f(q, q_t, t, y) with ODE1 coordinates"""
        (q, q_t, _) = self._State(q, q_t)
        if self.nODE1:
            return np.asarray(self._forces(q, q_t, t, self._y0 if y is None else y), dtype=float)
        return np.asarray(self._forces(q, q_t, t), dtype=float)

    def MassMatrix(self, q=None, q_t=None, t=0., sparse=False):
        """the mass matrix given, or of the function given; sparse=True: a scipy.sparse.csr_matrix"""
        if callable(self._massMatrix):
            (q, q_t, _) = self._State(q, q_t)
            matrix = self._massMatrix(q, q_t, t)
        else:
            matrix = self._massMatrix
        if sparse:
            return scipy.sparse.csr_matrix(matrix)
        return matrix.toarray() if scipy.sparse.issparse(matrix) else np.array(matrix, dtype=float)

    def MassMatrixTriplets(self, q=None, q_t=None, t=0.):
        """the triplets of MassMatrix"""
        matrix = self.MassMatrix(q, q_t, t, sparse=True).tocoo()
        return matrix.row, matrix.col, matrix.data

    def MassMatrixBlocks(self, q=None, q_t=None, t=0.):
        """the blocks given, else one block of all coordinates"""
        if self._massMatrixBlocks is not None:
            return self._massMatrixBlocks
        return SymbolicSystemBase.MassMatrixBlocks(self, q, q_t, t)


def _Recording(value, itemNumber, name, itemKind='object'):
    """the recorded symbolic user function of a parameter, None if there is none; a Python function, or a recording
    that reads mbs, raises"""
    if isinstance(value, dict) and value.get('type') == 'Symbolic':
        recording = value['function']
        if 'mbs.' in recording.GetFlatProgram():
            raise ValueError('SymbolicSystem: the ' + name + ' of ' + itemKind + ' ' + str(itemNumber) + ' reads the MainSystem '
                             '(an item output), which is not part of the equations of the symbolic system')
        return recording
    if isinstance(value, dict) or (not isinstance(value, int) and value is not None):
        raise ValueError('SymbolicSystem: ' + itemKind + ' ' + str(itemNumber) + ' has a Python ' + name
                         + '; only symbolic user functions can be assembled, see CreateSymbolicUserFunction')
    return None


def _Dense(matrix):
    """a MatrixContainer or array as dense numpy array; empty if not given"""
    if hasattr(matrix, 'Convert2DenseMatrix'):
        matrix = matrix.Convert2DenseMatrix()
    return np.array(matrix, dtype=float)


class SymbolicKind:
    """the objects of one object type whose user functions are the same recordings: one recording per user function,
    the table of the objects, their parameters and the global indices of their ODE2 coordinates

    Note:
        Its members: objectType, the type name 'UserBody', 'GenericODE2', 'ConnectorUser', 'JointUser', 'GenericODE1'
        or 'GenericDAE'; recordings, the dict of the recorded user functions by name (a name without recording is
        missing); objects, the list of the object numbers; ltg, the array (number of objects x local coordinates) of the
        global ODE2 indices - of the global ODE1 indices for GenericODE1 -; parameters, the list of the
        parameter vectors of the objects (UserBody), of the dicts of the constant matrices and the force vector
        (GenericODE2), or of the dicts of the parameter vector, 'orientation' (the connector gets the rotations and
        returns the torques) and the markers (ConnectorUser: per marker a dict of its 'type' - 'fixed', 'point' or
        'body' -, its local coordinate indices 'indices', its reference 'position' and 'rotation' - for a marker on a
        user body its local position and rotation, 'objectNumber', 'qRef', 'parameters' and the 'recordings' of the
        body that give its kinematics), of the dicts of 'parameters', 'markers', 'orientation' and the global indices
        'ltgAE' of its algebraic coordinates (JointUser), of the dicts of the system matrix 'A' and the vector 'f'
        (GenericODE1), or of the dicts of the matrices and vectors by their symbols - 'M', 'D', 'K', 'E', 'A', 'B',
        'B_t', 'C_q', 'C_y', 'f', 'g', 'c0', empty if not given -, the 'parameters' and the global indices 'ltgODE1' and
        'ltgAE' of its ODE1 and algebraic coordinates (GenericDAE, ltg those of its ODE2 coordinates)
    """
    def __init__(self, objectType, recordings):
        self.objectType = objectType
        self.recordings = recordings
        self.objects = []
        self.ltg = []
        self.parameters = []


class SymbolicLoad:
    """a load of the system (#3118): its value, constant or by the recording of its user function of the time, and how
    it acts on the coordinates - directly on the coordinates of a point node or on one node coordinate, or through a
    user function of the user body of its marker

    Note:
        Its members: loadNumber; loadType, 'ForceVector', 'TorqueVector', 'MassProportional' or 'Coordinate'; ltg, the
        global ODE2 indices it acts on; value, the loadVector or load; recording, the recorded loadVectorUserFunction or
        loadUserFunction, None if constant; body, None for a node marker, else the dict of the user body: 'objectNumber',
        'localPosition', 'qRef', 'parameters', 'projection' (the recording of the function whose transposed maps the
        load into the coordinates) with its 'projectionName', and 'rotation' (the recorded rotationMatrixUserFunction for
        bodyFixed, else None); ode1, True for a LoadCoordinate on an ODE1 coordinate, whose ltg are global ODE1 indices
        and which adds to the right-hand side of the ODE1 equations (#3144)
    """
    def __init__(self, loadNumber, loadType, ltg, value, recording, body=None, ode1=False):
        self.loadNumber = loadNumber
        self.loadType = loadType
        self.ltg = np.array(ltg, dtype=int)
        self.value = value
        self.recording = recording
        self.body = body
        self.ode1 = ode1


class SymbolicSystem(SymbolicSystemBase):
    """the ODE2 equations of motion of mbs, by kinds of objects with symbolic user functions, evaluated by
    exudyn.symbolic (#3110, #3113); mbs must be assembled; see the Details of this module for the items it takes

    Args:
        mbs: the assembled MainSystem
        composeSystem: True: ODE2RHS, MassMatrix and ODE2ResidualLHS evaluate two recordings of the composed system - the
            forces and the nonzero entries of the mass matrix of Equations(), each one flat program (#3131) -; False:
            each object and load by its own recordings, one call each
        parameters: None, or the ParameterBindings of the parameters of user items - ObjectUserBody,
            ObjectConnectorUser, ObjectJointUser -, each a number or an expression of named variables of
            mbs.symbolicVariables, the values those of the item: UserItemCreator.parameterBindings; Equations() keeps
            the expressions, the parameters are the named variables in them (#3152, #3159, #3208)

    Note:
        Its members: mbs; kinds, the list of SymbolicKind; jointKinds, the list of SymbolicKind of the ObjectJointUser
        (#3141), whose constraint equations are AlgebraicEquations and whose reaction forces are ODE2ReactionForcesLHS - in Equations() on request
        (#3146) -; ode1Kinds, the list of SymbolicKind of the ObjectGenericODE1 (#3144), whose right-hand
        side is ODE1RHS; daeKinds, the list of SymbolicKind of the ObjectGenericDAE (#3190), which add to the ODE2,
        the ODE1 and the algebraic equations; loads, the list of SymbolicLoad on ODE2 coordinates, ode1Loads those on ODE1
        coordinates; nODE2, nODE1, the numbers of ODE2 and ODE1 coordinates of the system; nAE, the number of the
        algebraic equations; qRef, the reference values of the ODE2 coordinates; composed, None or the dict of
        the recordings 'ODE2RHS', 'massMatrix' and 'massMatrixBlocks' - esym.Function of the inputs q, q_t and t, the
        VariableSet 'inputs' (#3149) - and the 'rows' and 'columns' of the entries of the mass matrix; parameterBindings,
        the ParameterBindings, checked
    """
    def __init__(self, mbs, composeSystem=True, parameters=None):
        if not mbs.systemIsConsistent:
            raise ValueError('SymbolicSystem: mbs must be assembled first')
        self.mbs = mbs
        self._itemFunctions = {} #the functions of the connectors and joints for their Jacobians, built at their first use
        self.nODE2 = mbs.systemData.ODE2Size()
        self.qRef = np.array(mbs.systemData.GetODE2Coordinates(configuration=exudyn.ConfigurationType.Reference))
        refused = []
        self.loads = []
        self.ode1Loads = []
        for loadNumber in range(mbs.systemData.NumberOfLoads()):
            (load, reason) = self._Load(loadNumber)
            if reason:
                refused += ['load ' + str(loadNumber) + ' (' + mbs.GetLoad(loadNumber)['loadType'] + ': ' + reason + ')']
            elif load is not None:
                (self.ode1Loads if load.ode1 else self.loads).append(load)
        self.nData = mbs.systemData.DataSize()
        self.nODE1 = mbs.systemData.ODE1Size()
        self.nAE = mbs.systemData.AEsize()
        kinds = {}
        ode1Kinds = {}
        jointKinds = {}
        daeKinds = {}
        self._constantMassMatrix = True
        for objectNumber in range(mbs.systemData.NumberOfObjects()):
            item = mbs.GetObject(objectNumber)
            objectType = item['objectType']
            ltg = list(mbs.systemData.GetObjectLTGODE2(objectNumber))
            if objectType == 'JointUser':
                reason = self._AddJoint(jointKinds, objectNumber, item)
                if reason:
                    refused += ['object ' + str(objectNumber) + ' (JointUser: ' + reason + ')']
                continue
            if objectType == 'GenericODE1':
                self._AddODE1Object(ode1Kinds, objectNumber, item)
                continue
            if objectType == 'GenericDAE':
                self._AddDAEObject(daeKinds, objectNumber, item)
                continue
            if objectType not in _userFunctionNames:
                if len(ltg):
                    refused += ['object ' + str(objectNumber) + ' (' + objectType + ')']
                continue
            markers = None
            if objectType == 'ConnectorUser':
                (markers, ltg, reason) = self._ConnectorMarkers(objectNumber, item)
                if reason:
                    refused += ['object ' + str(objectNumber) + ' (ConnectorUser: ' + reason + ')']
                    continue
                if not item['activeConnector']:
                    continue
            recordings = {}
            for name in _userFunctionNames[objectType]:
                recording = _Recording(item[name], objectNumber, name)
                if recording is not None:
                    recordings[name] = recording
            if objectType == 'UserBody':
                if 'massMatrixUserFunction' not in recordings:
                    raise ValueError('SymbolicSystem: object ' + str(objectNumber) + ' (UserBody) has no massMatrixUserFunction')
                self._constantMassMatrix &= bool(item['constantMassMatrix'])
            elif objectType == 'GenericODE2':
                self._constantMassMatrix &= 'massMatrixUserFunction' not in recordings
            elif 'forceUserFunction' not in recordings:
                continue #a connector without forces
            key = (objectType,) + tuple((name, recordings[name].GetFlatProgram()) for name in sorted(recordings))
            if markers is not None: #the connectors of a kind have the same pattern of markers and of their bodies
                key += (len(mbs.systemData.GetObjectLTGData(objectNumber)),)
                key += tuple((marker['type'], len(marker['indices']))
                             + tuple(recording.GetFlatProgram() for recording in marker.get('recordings', {}).values())
                             for marker in markers)
            if key not in kinds:
                kinds[key] = SymbolicKind(objectType, recordings)
            kind = kinds[key]
            kind.objects.append(objectNumber)
            kind.ltg.append(ltg)
            if objectType == 'UserBody':
                kind.parameters.append(np.array(item['parameters'], dtype=float))
            elif objectType == 'ConnectorUser':
                kind.parameters.append({'parameters': np.array(item['parameters'], dtype=float), 'markers': markers,
                                        'orientation': bool(int(item['requestedMarkerType']) & int(exudyn.MarkerType.Orientation)),
                                        'ltgData': np.array(mbs.systemData.GetObjectLTGData(objectNumber), dtype=int)})
            else:
                kind.parameters.append({'M': _Dense(item['massMatrix']), 'K': _Dense(item['stiffnessMatrix']),
                                        'D': _Dense(item['dampingMatrix']), 'f': np.array(item['forceVector'], dtype=float)})
        self.jointKinds = list(jointKinds.values())
        self.ode1Kinds = list(ode1Kinds.values())
        self.daeKinds = list(daeKinds.values())
        daeObjects = [dae for kind in self.daeKinds for dae in kind.parameters]
        if not refused and (sum(len(joint['ltgAE']) for kind in self.jointKinds for joint in kind.parameters)
                            + sum(len(dae['ltgAE']) for dae in daeObjects)) != self.nAE:
            refused += ['algebraic equations of other items than ObjectJointUser and ObjectGenericDAE']
        connectorData = [connector['ltgData'] for kind in kinds.values() if kind.objectType == 'ConnectorUser' for connector in kind.parameters]
        if not refused and (sum(len(ltgData) for ltgData in connectorData) + sum(len(dae['ltgData']) for dae in daeObjects)) != self.nData:
            refused += ['data coordinates of other items than ObjectConnectorUser and ObjectGenericDAE']
        if not refused and (sum(len(ltg) for kind in self.ode1Kinds for ltg in kind.ltg)
                            + sum(len(dae['ltgODE1']) for dae in daeObjects)) != self.nODE1:
            refused += ['ODE1 coordinates of other items than ObjectGenericODE1 and ObjectGenericDAE']
        if refused:
            raise ValueError('SymbolicSystem: these items cannot be assembled (yet): ' + ', '.join(refused))
        self.kinds = list(kinds.values())
        for kind in self.kinds + self.jointKinds + self.ode1Kinds + self.daeKinds:
            kind.ltg = np.array(kind.ltg, dtype=int)
        #the ODE2 equations depend on the ODE1 coordinates through an ObjectGenericDAE with both: the forces, the
        #composed system and Equations() then take y (#3190)
        self._ode2UsesY = any(kind.ltg.shape[1] > 0 and len(dae['ltgODE1']) > 0 for kind in self.daeKinds for dae in kind.parameters)
        self.parameterBindings = self._ParameterBindings(parameters)
        self.data = self.InitialStateData()
        self.composed = self._Compose() if composeSystem else None

    def _ParameterBindings(self, parameters):
        """the ParameterBindings of the items, checked - the number of the parameters of each item, the values those of
        the item, the variables of mbs.symbolicVariables -, and the named variables in them by name (#3152, #3159)"""
        self._parameterNames = {}
        if parameters is None:
            return None
        if not (hasattr(parameters, 'Parameters') and hasattr(parameters, 'Check')):
            raise TypeError('SymbolicSystem: parameters must be a ParameterBindings, UserItemCreator.parameterBindings, '
                            'not ' + type(parameters).__name__)
        parameters.Check()
        self._parameterNames = {str(variable): variable for variable in parameters.Parameters()}
        return parameters

    def SetParameterValues(self, values):
        """set named parameters (#3152): the values of the variables of mbs.symbolicVariables, the numbers of the
        items in mbs, then the system again from mbs

        Args:
            values: dict of the names of the parameters of parameterBindings and their new values
        """
        unknown = [name for name in values if name not in self._parameterNames]
        if unknown:
            raise ValueError('SymbolicSystem.SetParameterValues: no parameter named ' + str(unknown) + '; the parameters: '
                             + str(list(self._parameterNames)))
        for (name, value) in values.items():
            self._parameterNames[name].SetValue(float(value))
        self.parameterBindings.Apply()
        self.__init__(self.mbs, self.composed is not None, self.parameterBindings)

    def _ParameterVector(self, backend, objectNumber, numbers):
        """the parameters of an item for Equations: its numbers, or with symbolic parameters its list of expressions and
        numbers, if the backend takes them (#3152, #3159)"""
        bindings = self.parameterBindings
        variables = bindings[objectNumber] if (backend.parameterVariables and bindings is not None and objectNumber in bindings) else None
        if variables is None:
            return backend.Vector(numbers)
        return backend.Vector([backend.Parameter(value) for value in variables])

    def _Compose(self):
        """the recordings of the composed system (#3131): the forces f(q, q_t, t) and the entries of the mass matrix at
        the rows and columns of the local mass matrices, as esym.Function of the inputs q, q_t and t (#3149) of the
        expressions of Equations, optimized as CreateSymbolicUserFunction does; one call of each evaluates the system"""
        entries = set()
        for kind in self._BodyKinds():
            for ltg in kind.ltg:
                entries.update((int(i), int(j)) for i in ltg for j in ltg)
        entries = sorted(entries)
        rows = np.array([i for (i, j) in entries], dtype=int)
        columns = np.array([j for (i, j) in entries], dtype=int)
        backend = _Backend('symbolic', parameterVariables=False) #the numbers of the items; SetParameterValues composes again
        (q0, q_t0) = self.InitialState()
        inputs = exudyn.symbolic.VariableSet() #the scope of the inputs, in the order of the arguments of Evaluate
        (q, q_t, t) = (inputs.Add('q', q0), inputs.Add('q_t', q_t0), inputs.Add('t', 0.))
        y = inputs.Add('y', self.InitialStateODE1()) if self._ode2UsesY else None #(#3190)
        x = inputs.Add('x', self.data) if self.nData else None #the data coordinates, constant in a step (#3196)
        (M, f, blocks) = self._EquationLists(backend, q, q_t, t, y=y, x=x)[:3]
        massMatrix = [M[i][j] for (i, j) in entries] if entries else [0.]
        blockEntries = [x for (ltg, kindBlocks) in blocks for block in kindBlocks for row in block for x in row]

        def Function(values, name):
            return exudyn.symbolic.Function(exudyn.symbolic.Optimize(backend.Vector(values)), inputs, name)

        return {'ODE2RHS': Function(f, 'ODE2RHS'), 'massMatrix': Function(massMatrix, 'massMatrix'),
                'massMatrixBlocks': Function(blockEntries if blockEntries else [0.], 'massMatrixBlocks'),
                'inputs': inputs, 'rows': rows, 'columns': columns}

    def _AddODE1Object(self, ode1Kinds, objectNumber, item):
        """an ObjectGenericODE1 into its kind (#3144): the objects with the same recorded rhsUserFunction, or without
        one; a Python rhsUserFunction raises"""
        recording = _Recording(item['rhsUserFunction'], objectNumber, 'rhsUserFunction')
        recordings = {} if recording is None else {'rhsUserFunction': recording}
        key = None if recording is None else recording.GetFlatProgram()
        if key not in ode1Kinds:
            ode1Kinds[key] = SymbolicKind('GenericODE1', recordings)
        kind = ode1Kinds[key]
        kind.objects.append(objectNumber)
        kind.ltg.append(list(self.mbs.systemData.GetObjectLTGODE1(objectNumber)))
        kind.parameters.append({'A': _Dense(item['systemMatrix']), 'f': np.array(item['rhsVector'], dtype=float)})

    def _AddDAEObject(self, daeKinds, objectNumber, item):
        """an ObjectGenericDAE into its kind (#3190): the objects with the same recorded user functions and the same
        numbers of coordinates; a Python user function raises"""
        systemData = self.mbs.systemData
        recordings = {}
        for name in _daeFunctionNames:
            recording = _Recording(item[name], objectNumber, name)
            if recording is not None:
                recordings[name] = recording
        dae = {symbol: _Dense(item[name]) for (symbol, name) in _daeMatrices.items()}
        dae.update({symbol: np.array(item[name], dtype=float) for (symbol, name) in _daeVectors.items()})
        dae['ltgODE1'] = np.array(systemData.GetObjectLTGODE1(objectNumber), dtype=int)
        dae['ltgAE'] = np.array(systemData.GetObjectLTGAE(objectNumber), dtype=int)
        dae['ltgData'] = np.array(systemData.GetObjectLTGData(objectNumber), dtype=int)
        if 'massMatrixUserFunction' in recordings:
            if len(dae['ltgODE1']):
                raise ValueError('SymbolicSystem: object ' + str(objectNumber) + ' (GenericDAE) has a massMatrixUserFunction and '
                                 'ODE1 coordinates; the mass matrix of SymbolicSystem does not depend on y')
            self._constantMassMatrix = False
        ltg = list(systemData.GetObjectLTGODE2(objectNumber))
        key = ((len(ltg), len(dae['ltgODE1']), len(dae['ltgAE']), len(dae['ltgData']), dae['M'].size == 0) #without M: no mass, only constraints
               + tuple((name, recordings[name].GetFlatProgram()) for name in sorted(recordings)))
        if key not in daeKinds:
            daeKinds[key] = SymbolicKind('GenericDAE', recordings)
        kind = daeKinds[key]
        kind.objects.append(objectNumber)
        kind.ltg.append(ltg)
        kind.parameters.append(dae)

    def _AddJoint(self, jointKinds, objectNumber, item):
        """an ObjectJointUser into its kind, grouped as the connectors; the reason it is refused, empty if it is taken"""
        if not item['activeConnector']:
            return 'not active (its equations z = 0 are not given)'
        (markers, ltg, reason) = self._ConnectorMarkers(objectNumber, item, orientation=True)
        if reason:
            return reason
        recordings = {name: _Recording(item[name], objectNumber, name) for name in _jointFunctionNames}
        if any(recording is None for recording in recordings.values()):
            return 'the constraintUserFunction and the jacobianUserFunction must be symbolic'
        key = (tuple((name, recordings[name].GetFlatProgram()) for name in _jointFunctionNames)
               + tuple((marker['type'], len(marker['indices']))
                       + tuple(recording.GetFlatProgram() for recording in marker.get('recordings', {}).values())
                       for marker in markers))
        if key not in jointKinds:
            jointKinds[key] = SymbolicKind('JointUser', recordings)
        kind = jointKinds[key]
        kind.objects.append(objectNumber)
        kind.ltg.append(ltg)
        kind.parameters.append({'parameters': np.array(item['parameters'], dtype=float), 'markers': markers, 'orientation': True,
                                'ltgAE': np.array(self.mbs.systemData.GetObjectLTGAE(objectNumber), dtype=int),
                                'ltgData': np.zeros(0, dtype=int)})
        return ''

    def _ConnectorMarkers(self, objectNumber, item, orientation=None):
        """the markers of an ObjectConnectorUser - or of an ObjectJointUser, orientation=True - as the dicts of
        SymbolicKind, the global indices of the coordinates of the connector, and the reason it is refused, empty if it
        is taken"""
        if orientation is None:
            if item['markerAccessUserFunction'] != 0:
                return (None, None, 'a markerAccessUserFunction')
            orientation = bool(int(item['requestedMarkerType']) & int(exudyn.MarkerType.Orientation))
        (markers, ltg) = ([], [])
        for markerNumber in item['markerNumbers']:
            (data, coordinates, reason) = self._MarkerData(markerNumber, orientation)
            if reason:
                return (None, None, reason)
            data['indices'] = np.arange(len(ltg), len(ltg) + len(coordinates), dtype=int)
            markers.append(data)
            ltg += coordinates
        return (markers, ltg, '')

    def _MarkerData(self, markerNumber, orientation):
        """the dict of a marker as in SymbolicKind, the global indices of its coordinates and the reason it is refused,
        empty if it is taken; orientation: its rotation is needed"""
        mbs = self.mbs
        marker = mbs.GetMarker(markerNumber)
        markerType = marker['markerType']
        if markerType not in ['NodePosition', 'BodyPosition', 'BodyRigid']:
            return (None, None, 'a marker of type ' + markerType)
        reference = exudyn.ConfigurationType.Reference
        position = np.array(mbs.GetMarkerOutput(markerNumber, exudyn.OutputVariableType.Position, configuration=reference))
        data = {'type': 'fixed', 'position': position, 'rotation': np.eye(3)}
        coordinates = []
        if markerType == 'NodePosition':
            nodeType = mbs.GetNode(marker['nodeNumber'])['nodeType']
            if nodeType == 'Point':
                data['type'] = 'point'
                coordinates = list(mbs.systemData.GetNodeLTGODE2(marker['nodeNumber']))
            elif nodeType != 'PointGround':
                return (None, None, 'a marker on a node of type ' + nodeType)
        else:
            bodyNumber = marker['bodyNumber']
            body = mbs.GetObject(bodyNumber)
            localRotation = np.array(marker['localHT'], dtype=float)[0:3, 0:3] if markerType == 'BodyRigid' else np.eye(3)
            if body['objectType'] == 'Ground':
                if orientation:
                    data['rotation'] = np.array(mbs.GetMarkerOutput(markerNumber, exudyn.OutputVariableType.RotationMatrix,
                                                                    configuration=reference)).reshape(3, 3)
            elif body['objectType'] == 'UserBody':
                recordings = {}
                for name in _markerFunctions[orientation]:
                    recordings[name] = _Recording(body[name], bodyNumber, name)
                    if recordings[name] is None:
                        return (None, None, 'a marker on the user body ' + str(bodyNumber) + ', which has no ' + name)
                coordinates = list(mbs.systemData.GetObjectLTGODE2(bodyNumber))
                data = {'type': 'body', 'position': np.array(marker['localPosition'], dtype=float), 'rotation': localRotation,
                        'objectNumber': int(bodyNumber), 'qRef': self.qRef[coordinates],
                        'parameters': np.array(body['parameters'], dtype=float), 'recordings': recordings}
            else:
                return (None, None, 'a marker on an object of type ' + body['objectType'])
        return (data, coordinates, '')

    def _Load(self, loadNumber):
        """the SymbolicLoad of a load and the reason it is refused, empty if it is taken; None for a load on the
        ground, which acts on no coordinate"""
        mbs = self.mbs
        item = mbs.GetLoad(loadNumber)
        loadType = item['loadType']
        if loadType not in ['ForceVector', 'TorqueVector', 'MassProportional', 'Coordinate']:
            return (None, 'not taken (yet)')
        if loadType == 'Coordinate':
            (value, userFunctionName) = (float(item['load']), 'loadUserFunction')
        else:
            (value, userFunctionName) = (np.array(item['loadVector'], dtype=float), 'loadVectorUserFunction')
        recording = _Recording(item[userFunctionName], loadNumber, userFunctionName, 'load')
        bodyFixed = item.get('bodyFixed', False)
        marker = mbs.GetMarker(item['markerNumber'])
        markerType = marker['markerType']
        if markerType == 'NodeODE1Coordinate': #on the right-hand side of the ODE1 equation of the coordinate (#3144)
            if loadType != 'Coordinate':
                return (None, 'a marker of type ' + markerType)
            ltg = list(mbs.systemData.GetNodeLTGODE1(marker['nodeNumber']))
            return (SymbolicLoad(loadNumber, loadType, ltg[marker['coordinate']:marker['coordinate'] + 1], value, recording,
                                 ode1=True), '')
        if markerType in ['NodeCoordinate', 'NodePosition']:
            if (markerType == 'NodeCoordinate') != (loadType == 'Coordinate') or loadType not in ['ForceVector', 'Coordinate']:
                return (None, 'a marker of type ' + markerType)
            node = mbs.GetNode(marker['nodeNumber'])
            ltg = list(mbs.systemData.GetNodeLTGODE2(marker['nodeNumber']))
            if markerType == 'NodeCoordinate':
                return (SymbolicLoad(loadNumber, loadType, ltg[marker['coordinate']:marker['coordinate'] + 1], value, recording)
                        if len(ltg) else None, '')
            if node['nodeType'] == 'PointGround':
                return (None, '')
            if node['nodeType'] != 'Point' or bodyFixed:
                return (None, 'a force on a node of type ' + node['nodeType'] + (', bodyFixed' if bodyFixed else ''))
            return (SymbolicLoad(loadNumber, loadType, ltg, value, recording), '')
        if markerType not in ['BodyPosition', 'BodyRigid', 'BodyMass'] or loadType == 'Coordinate':
            return (None, 'a marker of type ' + markerType)
        bodyNumber = marker['bodyNumber']
        body = mbs.GetObject(bodyNumber)
        if body['objectType'] == 'Ground':
            return (None, '')
        if body['objectType'] != 'UserBody':
            return (None, 'a marker on an object of type ' + body['objectType'])
        projectionName = _loadProjections[loadType]
        projection = _Recording(body[projectionName], bodyNumber, projectionName)
        rotation = (_Recording(body['rotationMatrixUserFunction'], bodyNumber, 'rotationMatrixUserFunction')
                    if bodyFixed else None)
        if projection is None or (bodyFixed and rotation is None):
            return (None, 'the user body ' + str(bodyNumber) + ' has no '
                    + (projectionName if projection is None else 'rotationMatrixUserFunction for bodyFixed'))
        ltg = list(mbs.systemData.GetObjectLTGODE2(bodyNumber))
        return (SymbolicLoad(loadNumber, loadType, ltg, value, recording,
                             {'objectNumber': int(bodyNumber), 'localPosition': np.array(marker.get('localPosition', np.zeros(3)), dtype=float),
                              'qRef': self.qRef[ltg], 'parameters': np.array(body['parameters'], dtype=float),
                              'projectionName': projectionName, 'projection': projection, 'rotation': rotation}), '')

    def InitialState(self):
        """the initial coordinates and velocities (q0, q_t0) of the ODE2 coordinates of mbs"""
        systemData = self.mbs.systemData
        return (np.array(systemData.GetODE2Coordinates(configuration=exudyn.ConfigurationType.Initial)),
                np.array(systemData.GetODE2Coordinates_t(configuration=exudyn.ConfigurationType.Initial)))

    def InitialStateData(self):
        """the initial data coordinates of mbs (#3196)"""
        return np.array(self.mbs.systemData.GetDataCoordinates(configuration=exudyn.ConfigurationType.Initial), dtype=float)

    def PostNewton(self, q, q_t, y, z, x, x0, t):
        """the post-Newton step (#3196): the recorded postNewtonStepUserFunction of each ObjectConnectorUser and
        ObjectGenericDAE that has one, as CSystem::PostNewtonStep calls them - its new data coordinates, the largest of
        the errors and the smallest recommended step size of 0 or more

        Args:
            q: the ODE2 coordinates
            q_t: the ODE2 velocities
            y: the ODE1 coordinates
            z: the algebraic coordinates
            x: the data coordinates of the last iteration, which the equations read
            x0: the data coordinates at the start of the step
            t: the time

        Returns:
            (x, error, recommendedStepSize), recommendedStepSize < 0 for none
        """
        (q, q_t, _) = self._State(q, q_t)
        (y, z) = (np.zeros(self.nODE1) if y is None else np.asarray(y, dtype=float), np.zeros(self.nAE) if z is None else np.asarray(z, dtype=float))
        (x, x0) = (np.asarray(x, dtype=float), np.asarray(x0, dtype=float))
        (xNew, error, stepSize) = (x.copy(), 0., -1.)
        data = self.data
        self.data = x #the data of the iteration for the states of the markers
        try:
            for (kind, i, ltgData, values) in self._PostNewtonResults(q, q_t, y, z, x, x0, t):
                xNew[ltgData] = values[2:]
                error = max(error, values[0])
                if values[1] >= 0 and (stepSize < 0 or values[1] < stepSize):
                    stepSize = values[1]
        finally:
            self.data = data
        return (xNew, error, stepSize)

    def _PostNewtonResults(self, q, q_t, y, z, x, x0, t):
        """the results [error, recommendedStepSize, data] of the recorded post-Newton functions, per item with its
        kind, index and global data indices"""
        for kind in self.kinds:
            if kind.objectType != 'ConnectorUser' or 'postNewtonStepUserFunction' not in kind.recordings:
                continue
            for i in range(len(kind.objects)):
                connector = kind.parameters[i]
                arguments = self._ConnectorStates(kind, i, q, q_t, t)[0]
                (leaves, parameters) = (arguments[:-2], arguments[-1])
                values = np.array(kind.recordings['postNewtonStepUserFunction'].Evaluate(
                    self.mbs, *leaves, x[connector['ltgData']], x0[connector['ltgData']], parameters), dtype=float)
                yield (kind, i, connector['ltgData'], values)
        for kind in self.daeKinds:
            if 'postNewtonStepUserFunction' not in kind.recordings:
                continue
            for i in range(len(kind.objects)):
                (ltg, dae) = (kind.ltg[i], kind.parameters[i])
                values = np.array(kind.recordings['postNewtonStepUserFunction'].Evaluate(
                    self.mbs, t, kind.objects[i], q[ltg], q_t[ltg], y[dae['ltgODE1']], z[dae['ltgAE']], x[dae['ltgData']],
                    x0[dae['ltgData']], dae['parameters']), dtype=float)
                yield (kind, i, dae['ltgData'], values)

    def InitialStateODE1(self):
        """the initial ODE1 coordinates y0 of mbs"""
        return np.array(self.mbs.systemData.GetODE1Coordinates(configuration=exudyn.ConfigurationType.Initial))

    def IsMassMatrixConstant(self):
        """True if every user body has constantMassMatrix and no ObjectGenericODE2 has a massMatrixUserFunction"""
        return self._constantMassMatrix

    def ODE1RHS(self, q, q_t, y, t):
        """the right-hand side g(q, q_t, y, t) of the ODE1 equations y_t = g (#3144): per ObjectGenericODE1 A y + f and
        its recorded rhsUserFunction of its coordinates, as CObjectGenericODE1 computes it, per ObjectGenericDAE A y + B q
        + B_t q_t + g + g_u (#3190), and the LoadCoordinate on ODE1 coordinates

        Args:
            q: the ODE2 coordinates, used by ObjectGenericDAE only
            q_t: the ODE2 velocities, used by ObjectGenericDAE only
            y: the ODE1 coordinates, of size nODE1
            t: the time

        Returns:
            the vector g of size nODE1
        """
        y = np.asarray(y, dtype=float)
        g = np.zeros(self.nODE1)
        for kind in self.ode1Kinds:
            for i in range(len(kind.objects)):
                np.add.at(g, kind.ltg[i], self._LocalODE1RHS(kind, i, y, t))
        if self.daeKinds:
            (q, q_t, _) = self._State(q, q_t)
            for kind in self.daeKinds:
                for i in range(len(kind.objects)):
                    np.add.at(g, kind.parameters[i]['ltgODE1'], self._DAERHS(kind, i, q, q_t, y, t))
        for load in self.ode1Loads:
            np.add.at(g, load.ltg, np.atleast_1d(self._LoadValue(load, t)))
        return g

    def _ODE1State(self, y, where):
        """the ODE1 coordinates y as float array; None: zero, unless the ODE2 equations depend on them"""
        if y is None:
            if self._ode2UsesY:
                raise ValueError('SymbolicSystem.' + where + ': the ODE2 equations depend on the ODE1 coordinates '
                                 '(ObjectGenericDAE), so y must be given')
            return np.zeros(self.nODE1)
        return np.asarray(y, dtype=float)

    def _DAEArguments(self, kind, i, q, q_t, y, t):
        """the arguments after mbs of the user functions of ObjectGenericDAE i of kind: t, itemNumber, its q, q_t, y and
        its parameters"""
        (ltg, dae) = (kind.ltg[i], kind.parameters[i])
        return (t, kind.objects[i], q[ltg], q_t[ltg], y[dae['ltgODE1']], self.data[dae['ltgData']], dae['parameters'])

    def _DAECall(self, kind, name, i, q, q_t, y, t):
        """the result of the recorded user function name of ObjectGenericDAE i of kind, as numpy array"""
        return np.array(kind.recordings[name].Evaluate(self.mbs, *self._DAEArguments(kind, i, q, q_t, y, t)), dtype=float)

    def _DAEJacobian(self, kind, name, i, position, q, q_t, y, t):
        """the derivative of the recorded user function name of ObjectGenericDAE i of kind by its argument at position -
        _daeQ, _daeQ_t or _daeY -, by Diff of the recording"""
        return self._RecordingJacobian(kind.recordings[name], position, self._DAEArguments(kind, i, q, q_t, y, t))

    def _DAEForces(self, kind, i, q, q_t, y, t):
        """the ODE2 forces of ObjectGenericDAE i of kind on the right-hand side, f + f_u - K q - D q_t - E y, the negative
        of what CObjectGenericDAE::ComputeODE2LHS computes"""
        (ltg, dae) = (kind.ltg[i], kind.parameters[i])
        f = np.zeros(len(ltg))
        if len(dae['f']):
            f += dae['f']
        f -= _MatVec(dae['K'], q[ltg]) + _MatVec(dae['D'], q_t[ltg]) + _MatVec(dae['E'], y[dae['ltgODE1']])
        if 'forceUserFunction' in kind.recordings:
            f += self._DAECall(kind, 'forceUserFunction', i, q, q_t, y, t)
        return f

    def _DAERHS(self, kind, i, q, q_t, y, t):
        """the ODE1 right-hand side of ObjectGenericDAE i of kind, A y + B q + B_t q_t + g + g_u"""
        (ltg, dae) = (kind.ltg[i], kind.parameters[i])
        g = np.zeros(len(dae['ltgODE1']))
        if len(dae['g']):
            g += dae['g']
        g += _MatVec(dae['A'], y[dae['ltgODE1']]) + _MatVec(dae['B'], q[ltg]) + _MatVec(dae['B_t'], q_t[ltg])
        if 'rhsUserFunction' in kind.recordings:
            g += self._DAECall(kind, 'rhsUserFunction', i, q, q_t, y, t)
        return g

    def _DAEConstraints(self, kind, i, q, y):
        """the algebraic equations of ObjectGenericDAE i of kind at position level, C_q q + C_y y - c_0"""
        (ltg, dae) = (kind.ltg[i], kind.parameters[i])
        c = np.zeros(len(dae['ltgAE']))
        c += _MatVec(dae['C_q'], q[ltg]) + _MatVec(dae['C_y'], y[dae['ltgODE1']])
        if len(dae['c0']):
            c -= dae['c0']
        return c

    def _DAEConstraintFunction(self, kind, i):
        """the recorded constraintUserFunction of ObjectGenericDAE i of kind as one exudyn.symbolic.Function (#3191): its
        term c_u of the algebraic equations, then the reaction terms (dc_u/dq)^T z and (dc_u/dy)^T z, of the variables q,
        q_t, y, z, x and t of the object, at position level; built once, as _ItemFunction"""
        key = ('GenericDAE', id(kind), i)
        if key not in self._itemFunctions:
            (ltg, dae, objectNumber) = (kind.ltg[i], kind.parameters[i], kind.objects[i])
            variables = exudyn.symbolic.VariableSet()
            (q, q_t, y, z) = (variables.Add(name, np.zeros(n)) for (name, n) in [('q', len(ltg)), ('q_t', len(ltg)),
                                                                              ('y', len(dae['ltgODE1'])), ('z', len(dae['ltgAE']))])
            x = variables.Add('x', np.zeros(len(dae['ltgData']))) #the data coordinates, constant in a step (#3196)
            t = variables.Add('t', 0.)
            backend = _Backend('symbolic', parameterVariables=False)
            c = _Caller('symbolic')(kind.recordings['constraintUserFunction'], t, objectNumber, q, q_t, y, z, x, 0.,
                                    self._ParameterVector(backend, objectNumber, dae['parameters']))
            c = backend.Vector([c[k] for k in range(len(dae['ltgAE']))])
            result = [c[k] for k in range(len(dae['ltgAE']))]
            for (x, n) in [(q, len(ltg)), (y, len(dae['ltgODE1']))]:
                if n:
                    reaction = exudyn.symbolic.Jacobian(c, x).T @ z
                    result += [reaction[k] for k in range(n)]
            self._itemFunctions[key] = exudyn.symbolic.Function(backend.Vector(result), variables, 'GenericDAE' + str(int(objectNumber)))
        return self._itemFunctions[key]

    def _DAEConstraintTerms(self, kind, i, q, q_t, y, z, t, derivatives=False):
        """the values of _DAEConstraintFunction of ObjectGenericDAE i of kind - c_u, (dc_u/dq)^T z, (dc_u/dy)^T z -, and
        with derivatives the dict of their derivatives by 'q', 'q_t', 'y' and 'z', each (rows x local columns)"""
        (ltg, dae) = (kind.ltg[i], kind.parameters[i])
        function = self._DAEConstraintFunction(kind, i)
        z = np.zeros(self.nAE) if z is None else np.asarray(z, dtype=float)
        arguments = (q[ltg], q_t[ltg], y[dae['ltgODE1']], z[dae['ltgAE']], self.data[dae['ltgData']], float(t))
        values = np.array(function.Evaluate(*arguments), dtype=float)
        if not derivatives:
            return values
        sizes = {'q': len(ltg), 'q_t': len(ltg), 'y': len(dae['ltgODE1']), 'z': len(dae['ltgAE'])}
        return (values, {name: np.array(function.EvaluateJacobian(name, *arguments), dtype=float).reshape(len(values), n)
                         for (name, n) in sizes.items()})

    def ODE1ReactionTermsLHS(self, q, q_t, y, z, t):
        """the reaction terms C_y^T z of the algebraic coordinates of the ObjectGenericDAE in the ODE1 equations, on
        their left-hand side (#3155, #3190), of size nODE1"""
        z = np.asarray(z, dtype=float)
        r = np.zeros(self.nODE1)
        for kind in self.daeKinds:
            for (i, dae) in enumerate(kind.parameters):
                if dae['C_y'].size:
                    np.add.at(r, dae['ltgODE1'], dae['C_y'].T @ z[dae['ltgAE']])
                if 'constraintUserFunction' in kind.recordings and len(dae['ltgODE1']): #(dc_u/dy)^T z (#3191)
                    (q, q_t, _) = self._State(q, q_t)
                    np.add.at(r, dae['ltgODE1'], self._DAEConstraintTerms(kind, i, q, q_t, np.asarray(y, dtype=float), z, t)[
                        len(dae['ltgAE']) + kind.ltg.shape[1]:])
        return r

    def _LocalODE1RHS(self, kind, i, y, t):
        """the right-hand side of ObjectGenericODE1 i of kind, A y + f + f_user(mbs, t, itemNumber, y)"""
        (ltg, matrices) = (kind.ltg[i], kind.parameters[i])
        g = np.zeros(len(ltg))
        if matrices['A'].size:
            g += matrices['A'] @ y[ltg]
        if len(matrices['f']):
            g += matrices['f']
        if 'rhsUserFunction' in kind.recordings:
            g += np.array(kind.recordings['rhsUserFunction'].Evaluate(self.mbs, t, kind.objects[i], y[ltg]), dtype=float)
        return g

    def ODE1Jacobian(self, y, t, q=None, q_t=None):
        """the derivative of ODE1RHS by y (nODE1 x nODE1), exact: A and the derivative of the recorded
        rhsUserFunction (#3144, #3116); q and q_t, zero if None, for an ObjectGenericDAE (#3190)"""
        y = np.asarray(y, dtype=float)
        J = np.zeros((self.nODE1, self.nODE1))
        for kind in self.ode1Kinds:
            for i in range(len(kind.objects)):
                (ltg, matrices) = (kind.ltg[i], kind.parameters[i])
                local = np.zeros((len(ltg), len(ltg)))
                if matrices['A'].size:
                    local += matrices['A']
                if 'rhsUserFunction' in kind.recordings:
                    local += self._RecordingJacobian(kind.recordings['rhsUserFunction'], 3, (t, kind.objects[i], y[ltg]))
                J[np.ix_(ltg, ltg)] += local
        (q, q_t, _) = self._State(q, q_t)
        for kind in self.daeKinds:
            for i in range(len(kind.objects)):
                (ltg, dae) = (kind.ltg[i], kind.parameters[i])
                if not len(dae['ltgODE1']):
                    continue
                local = np.zeros((len(dae['ltgODE1']), len(dae['ltgODE1'])))
                if dae['A'].size:
                    local += dae['A']
                if 'rhsUserFunction' in kind.recordings:
                    local += self._DAEJacobian(kind, 'rhsUserFunction', i, _daeY, q, q_t, y, t)
                J[np.ix_(dae['ltgODE1'], dae['ltgODE1'])] += local
        return J

    def SetSystemState(self, q, q_t=None, q_tt=None, t=None, configuration=exudyn.ConfigurationType.Current):
        """write a state into mbs: the ODE2 coordinates, velocities and accelerations and the time - None leaves one
        unchanged -, so that the sensors, the output functions (Current) and the renderer (Visualization) show it

        Args:
            q: the ODE2 coordinates, of size nODE2
            q_t: the ODE2 velocities; None: unchanged
            q_tt: the ODE2 accelerations; None: unchanged
            t: the time; None: unchanged
            configuration: the configuration written, exudyn.ConfigurationType.Current or Visualization
        """
        systemData = self.mbs.systemData
        for (x, Set) in [(q, systemData.SetODE2Coordinates), (q_t, systemData.SetODE2Coordinates_t),
                         (q_tt, systemData.SetODE2Coordinates_tt)]:
            if x is not None:
                Set(np.asarray(x, dtype=float), configuration=configuration)
        if t is not None:
            systemData.SetTime(t, configuration)

    def _Arguments(self, kind, i, q, q_t, t):
        """the arguments of the user functions of object i of kind, after mbs"""
        ltg = kind.ltg[i]
        if kind.objectType == 'UserBody':
            qRef = self.qRef[ltg]
            return (t, kind.objects[i], qRef, qRef + q[ltg], q_t[ltg], kind.parameters[i])
        if kind.objectType == 'ConnectorUser':
            return self._ConnectorStates(kind, i, q, q_t, t)[0]
        return (t, kind.objects[i], q[ltg], q_t[ltg])

    def _ConnectorStates(self, kind, i, q, q_t, t):
        """the arguments after mbs of the recorded forceUserFunction of connector i of kind - the leaves of the frame
        and the twist of each marker, unit rotations and no angular velocities if it requests no orientation -, and per
        marker its position and rotation Jacobians (None for a fixed marker) and the arguments of the user functions of
        its body (None for a point node or a fixed marker)"""
        (ltg, connector) = (kind.ltg[i], kind.parameters[i])
        (states, markerData) = ([], [])
        for marker in connector['markers']:
            (Jpos, Jrot, bodyArguments) = (None, None, None)
            if marker['type'] == 'fixed':
                states += [marker['position'], np.zeros(3), marker['rotation'], np.zeros(3)]
            elif marker['type'] == 'point':
                indices = ltg[marker['indices']]
                Jpos = np.eye(3)
                states += [marker['position'] + q[indices], q_t[indices], np.eye(3), np.zeros(3)]
            else: #on a user body: v = J_pos q_t, omega = J_rot q_t, the rotation of the marker after the one of the body
                indices = ltg[marker['indices']]
                recordings = marker['recordings']
                bodyArguments = (t, marker['objectNumber'], marker['position'], marker['qRef'], marker['qRef'] + q[indices],
                                 q_t[indices], marker['parameters'])
                values = {name: np.array(recording.Evaluate(self.mbs, *bodyArguments), dtype=float)
                          for (name, recording) in recordings.items()} #with Orientation the rotation functions too
                Jpos = values['positionJacobianUserFunction']
                (rotation, angularVelocity) = (np.eye(3), np.zeros(3))
                if connector['orientation']:
                    Jrot = values['rotationJacobianUserFunction']
                    rotation = values['rotationMatrixUserFunction'] @ marker['rotation']
                    angularVelocity = Jrot @ q_t[indices]
                states += [values['positionUserFunction'], Jpos @ q_t[indices], rotation, angularVelocity]
            markerData.append((Jpos, Jrot, bodyArguments))
        return ((t, kind.objects[i], *_ConnectorLeaves(states), self.data[connector['ltgData']], connector['parameters']), markerData)

    def _JointArguments(self, kind, i, q, t):
        """the arguments after mbs of the recorded user functions of joint i of kind - the leaves of the frame of each
        marker -, and per marker its Jacobians and the arguments of its body, as _ConnectorStates"""
        (arguments, markerData) = self._ConnectorStates(kind, i, q, np.zeros(self.nODE2), t)
        (t, objectNumber, R0, p0, v0, w0, R1, p1, v1, w1, data, parameters) = arguments
        return ((t, objectNumber, R0, p0, R1, p1, parameters), markerData)

    def AlgebraicEquations(self, q, q_t, y, z, t, velocityLevel=False):
        """the constraint equations c(q, y, t) at position level: of the joints by their recorded
        constraintUserFunction (#3141), of the ObjectGenericDAE C_q q + C_y y - c_0 (#3190); the velocity level is not
        given

        Args:
            q: the ODE2 coordinates
            q_t: the ODE2 velocities, not used
            y: the ODE1 coordinates, used by ObjectGenericDAE only
            z: the algebraic coordinates, the Lagrange multipliers lambda of the documentation, not used
            t: the time
            velocityLevel: must be False

        Returns:
            the vector c of size nAE
        """
        if velocityLevel:
            raise ValueError('SymbolicSystem.AlgebraicEquations: the joints and ObjectGenericDAE give their equations at position level only')
        q = np.asarray(q, dtype=float)
        c = np.zeros(self.nAE)
        for kind in self.jointKinds:
            for i in range(len(kind.objects)):
                arguments = self._JointArguments(kind, i, q, t)[0]
                c[kind.parameters[i]['ltgAE']] = np.array(kind.recordings['constraintUserFunction'].Evaluate(self.mbs, *arguments),
                                                          dtype=float)
        if self.daeKinds:
            (q, q_t, _) = self._State(q, q_t)
            y = np.zeros(self.nODE1) if y is None else np.asarray(y, dtype=float)
            for kind in self.daeKinds:
                for i in range(len(kind.objects)):
                    dae = kind.parameters[i]
                    c[dae['ltgAE']] = self._DAEConstraints(kind, i, q, y)
                    if 'constraintUserFunction' in kind.recordings: #+ c_u (#3191)
                        c[dae['ltgAE']] += self._DAEConstraintTerms(kind, i, q, q_t, y, z, t)[:len(dae['ltgAE'])]
        return c

    def ODE2ReactionForcesLHS(self, q, q_t, y, z, t):
        """the reaction forces C_q^T z of the joints (#3141): the recorded jacobianUserFunction C by the marker
        translations and global rotation increments, C^T z the forces and torques on the markers, projected into the
        coordinates by the position and rotation Jacobians of the markers, as the forces of a connector; of the
        ObjectGenericDAE C_q^T z by its matrix (#3190)

        Args:
            q: the ODE2 coordinates
            q_t: the ODE2 velocities, not used
            y: the ODE1 coordinates, not used
            z: the algebraic coordinates, the Lagrange multipliers lambda of the documentation
            t: the time

        Returns:
            the vector of size nODE2
        """
        (q, z) = (np.asarray(q, dtype=float), np.asarray(z, dtype=float))
        f = np.zeros(self.nODE2)
        for kind in self.jointKinds:
            for i in range(len(kind.objects)):
                joint = kind.parameters[i]
                (arguments, markerData) = self._JointArguments(kind, i, q, t)
                C = np.array(kind.recordings['jacobianUserFunction'].Evaluate(self.mbs, *arguments), dtype=float).reshape(-1, 12)
                forces = C.T @ z[joint['ltgAE']]
                local = np.zeros(kind.ltg.shape[1])
                for (k, (marker, (Jpos, Jrot, bodyArguments))) in enumerate(zip(joint['markers'], markerData)):
                    if Jpos is not None:
                        local[marker['indices']] += Jpos.T @ forces[6*k:6*k + 3]
                    if Jrot is not None:
                        local[marker['indices']] += Jrot.T @ forces[6*k + 3:6*k + 6]
                np.add.at(f, kind.ltg[i], local)
        for kind in self.daeKinds:
            for (i, (ltg, dae)) in enumerate(zip(kind.ltg, kind.parameters)):
                if dae['C_q'].size:
                    np.add.at(f, ltg, dae['C_q'].T @ z[dae['ltgAE']])
                if 'constraintUserFunction' in kind.recordings and len(ltg): #(dc_u/dq)^T z (#3191)
                    (q_t, y) = (self._State(q, q_t)[1], np.zeros(self.nODE1) if y is None else np.asarray(y, dtype=float))
                    nAE = len(dae['ltgAE'])
                    np.add.at(f, ltg, self._DAEConstraintTerms(kind, i, q, q_t, y, z, t)[nAE:nAE + len(ltg)])
        return f

    def SystemJacobian(self, q, q_t, q_tt, y, y_t, z, t, factor_q=1., factor_q_t=0., factor_q_tt=0., factor_y=1.,
                       factor_y_t=0., velocityLevel=False, epsilon=1e-6):
        """the Jacobian of SystemResidual, exact (#3145): the ODE2 block by ODE2ResidualLHSJacobian plus the derivative of the
        reaction forces, the blocks C_q^T and C_q of the joints by _JointJacobian, the ODE1 block by ODE1Jacobian; the
        arguments as SymbolicSystemBase.SystemJacobian, epsilon not used

        Args:
            q: the ODE2 coordinates
            q_t: the ODE2 velocities
            q_tt: the ODE2 accelerations
            y: the ODE1 coordinates
            y_t: their velocities
            z: the algebraic coordinates, the Lagrange multipliers lambda of the documentation
            t: the time
            factor_q: the factor of d/dq
            factor_q_t: the factor of d/dq_t
            factor_q_tt: the factor of d/dq_tt
            factor_y: the factor of d/dy
            factor_y_t: the factor of d/dy_t
            velocityLevel: must be False, as for AlgebraicEquations
            epsilon: not used

        Returns:
            the dense matrix (nSystem x nSystem), nSystem = nODE2 + nODE1 + nAE
        """
        if velocityLevel and self.nAE:
            raise ValueError('SymbolicSystem.SystemJacobian: the joints give their equations at position level only')
        (nODE2, nODE1, nAE) = (self.nODE2, self.nODE1, self.nAE)
        nSystem = nODE2 + nODE1 + nAE
        (q, q_t, q_tt) = self._State(q, q_t, q_tt)
        y = np.zeros(nODE1) if y is None else np.asarray(y, dtype=float)
        J = np.zeros((nSystem, nSystem))
        J[:nODE2, :nODE2] = self.ODE2ResidualLHSJacobian(q, q_t, q_tt, t, factor_q, factor_q_t, factor_q_tt, y=y)
        offset = nODE2 + nODE1 #the first algebraic coordinate
        if nAE:
            z = np.asarray(z, dtype=float)
            for kind in self.jointKinds:
                for i in range(len(kind.objects)):
                    (ltg, ltgAE) = (kind.ltg[i], kind.parameters[i]['ltgAE'] + offset)
                    (C_q, dReaction) = self._JointJacobian(kind, i, q, z, t)
                    J[np.ix_(ltg, ltg)] += factor_q*dReaction
                    J[np.ix_(ltg, ltgAE)] += C_q.T
                    J[np.ix_(ltgAE, ltg)] += factor_q*C_q
        if nODE1: #the rows g - y_t
            J[nODE2:offset, nODE2:offset] = factor_y*self.ODE1Jacobian(y, t, q, q_t) - factor_y_t*np.eye(nODE1)
        for kind in self.daeKinds: #the coupling blocks of ObjectGenericDAE (#3190), the ODE2 block by ODE2ResidualLHSJacobian
            for i in range(len(kind.objects)):
                for (rows, columns, block) in self._DAECouplings(kind, i, q, q_t, y, z, t, factor_q, factor_q_t, factor_y):
                    J[np.ix_(rows, columns)] += block
        return J

    def _DAECouplings(self, kind, i, q, q_t, y, z, t, factor_q, factor_q_t, factor_y):
        """the blocks of SystemJacobian of ObjectGenericDAE i of kind outside the ODE2 block and the ODE1 block, as
        (global rows, global columns, block) of the residual [M q_tt - f + C_q^T z, g - y_t - C_y^T z, c]: the ODE2 rows
        by y, factor_y (E - df_u/dy), and by z, C_q^T; the ODE1 rows by q and q_t, factor_q (B + dg_u/dq) + factor_q_t
        (B_t + dg_u/dq_t), and by z, -C_y^T; the algebraic rows factor_q C_q and factor_y C_y; a recorded
        constraintUserFunction adds the derivatives of c_u and of its reaction terms (#3191), also to the ODE2 and the
        ODE1 block"""
        (ltg, dae) = (kind.ltg[i], kind.parameters[i])
        (nODE2, nODE1) = (self.nODE2, self.nODE1)
        (ltgODE1, ltgAE) = (dae['ltgODE1'] + nODE2, dae['ltgAE'] + nODE2 + nODE1)
        (n2, n1, nAE) = (len(ltg), len(ltgODE1), len(ltgAE))
        blocks = []
        if n2 and n1:
            block = factor_y*(dae['E'] if dae['E'].size else np.zeros((n2, n1)))
            if 'forceUserFunction' in kind.recordings:
                block = block - factor_y*self._DAEJacobian(kind, 'forceUserFunction', i, _daeY, q, q_t, y, t)
            blocks.append((ltg, ltgODE1, block))
            block = np.zeros((n1, n2))
            for (name, position, factor) in [('B', _daeQ, factor_q), ('B_t', _daeQ_t, factor_q_t)]:
                if factor == 0.:
                    continue
                if dae[name].size:
                    block += factor*dae[name]
                if 'rhsUserFunction' in kind.recordings:
                    block += factor*self._DAEJacobian(kind, 'rhsUserFunction', i, position, q, q_t, y, t)
            blocks.append((ltgODE1, ltg, block))
        if nAE and n2 and dae['C_q'].size:
            blocks += [(ltg, ltgAE, dae['C_q'].T), (ltgAE, ltg, factor_q*dae['C_q'])]
        if nAE and n1 and dae['C_y'].size:
            blocks += [(ltgODE1, ltgAE, -dae['C_y'].T), (ltgAE, ltgODE1, factor_y*dae['C_y'])]
        if 'constraintUserFunction' in kind.recordings: #c_u and its reaction terms, also in the ODE2 and the ODE1 block (#3191)
            d = self._DAEConstraintTerms(kind, i, q, q_t, y, z, t, derivatives=True)[1]
            parts = [(ltgAE, slice(0, nAE), 1.)]
            if n2:
                parts.append((ltg, slice(nAE, nAE + n2), 1.))
            if n1:
                parts.append((ltgODE1, slice(nAE + n2, nAE + n2 + n1), -1.)) #the ODE1 rows g - y_t - C_y^T z
            for (rows, part, sign) in parts:
                if n2:
                    blocks.append((rows, ltg, sign*(factor_q*d['q'][part] + factor_q_t*d['q_t'][part])))
                if n1:
                    blocks.append((rows, ltgODE1, sign*factor_y*d['y'][part]))
                blocks.append((rows, ltgAE, sign*d['z'][part]))
        return blocks

    def _Call(self, kind, name, i, q, q_t, t):
        """the result of the user function name of object i of kind, by its recording, as numpy array"""
        return np.array(kind.recordings[name].Evaluate(self.mbs, *self._Arguments(kind, i, q, q_t, t)), dtype=float)

    def _Jacobian(self, kind, name, i, position, q, q_t, t):
        """the Jacobian of the user function name of object i of kind by its argument at position, by Diff of the
        recording (#3115)"""
        return self._RecordingJacobian(kind.recordings[name], position, self._Arguments(kind, i, q, q_t, t))

    def _RecordingJacobian(self, recording, position, arguments):
        """the Jacobian of a recording by its argument at position (mbs at 0), for the arguments after mbs: the
        dimensions of the result, then the one of the argument"""
        argumentName = recording.GetFlatProgramData()['arguments'][position]['name']
        return np.array(recording.EvaluateJacobian(argumentName, self.mbs, *arguments), dtype=float)

    def _BodyJacobian(self, marker, name, bodyArguments):
        """the derivative by qTotal of the user function name of the user body of a connector marker"""
        return self._RecordingJacobian(marker['recordings'][name], _bodyArgumentPositions[name][0], bodyArguments)

    def _BodyArguments(self, load, name, q, q_t, t):
        """the arguments after mbs of the user function name of the user body of a load"""
        body = load.body
        arguments = (t, body['objectNumber'], body['qRef'], body['qRef'] + q[load.ltg], q_t[load.ltg], body['parameters'])
        if name == 'massIntegralUserFunction':
            return arguments
        return arguments[:2] + (body['localPosition'],) + arguments[2:]

    def _LoadValue(self, load, t):
        """the value of a load at t: its loadVector or load, or what its recorded user function returns"""
        if load.recording is None:
            return load.value
        return np.array(load.recording.Evaluate(self.mbs, t, load.value), dtype=float)

    def _LoadForces(self, load, q, q_t, t):
        """the generalized forces of a load on its coordinates: the value itself on a point node or a coordinate, else
        J^T v by the projection J of the user body - J_pos, J_rot or the mass integral -, v rotated by the rotation
        matrix of the marker for bodyFixed, as CSystem applies loads"""
        value = self._LoadValue(load, t)
        if load.body is None:
            return np.atleast_1d(value)
        body = load.body
        J = np.array(body['projection'].Evaluate(self.mbs, *self._BodyArguments(load, body['projectionName'], q, q_t, t)), dtype=float)
        if body['rotation'] is not None:
            value = np.array(body['rotation'].Evaluate(self.mbs, *self._BodyArguments(load, 'rotationMatrixUserFunction', q, q_t, t)),
                             dtype=float) @ value
        return J.T @ value

    def _LoadJacobian(self, load, q, q_t, t, factor_q, factor_q_t):
        """the local Jacobian of the residual -f of a load on a user body, by Diff of the recordings of the body; a load on
        a node or a coordinate depends on the time only"""
        n = len(load.ltg)
        local = np.zeros((n, n))
        body = load.body
        if body is None:
            return local
        value = self._LoadValue(load, t)
        name = body['projectionName']
        arguments = self._BodyArguments(load, name, q, q_t, t)
        if body['rotation'] is not None:
            rotationArguments = self._BodyArguments(load, 'rotationMatrixUserFunction', q, q_t, t)
            R = np.array(body['rotation'].Evaluate(self.mbs, *rotationArguments), dtype=float)
            J = np.array(body['projection'].Evaluate(self.mbs, *arguments), dtype=float)
        for (k, factor) in enumerate([factor_q, factor_q_t]):
            if factor == 0.:
                continue
            dJ = self._RecordingJacobian(body['projection'], _bodyArgumentPositions[name][k], arguments)
            if body['rotation'] is None:
                local -= factor*np.einsum('ijk,i->jk', dJ, value)
            else: #d(J^T R v) = dJ^T R v + J^T dR v
                dR = self._RecordingJacobian(body['rotation'], _bodyArgumentPositions['rotationMatrixUserFunction'][k],
                                             rotationArguments)
                local -= factor*(np.einsum('ijk,i->jk', dJ, R @ value) + J.T @ np.einsum('ijk,j->ik', dR, value))
        return local

    def _LocalMassMatrix(self, kind, i, q, q_t, t):
        if kind.objectType == 'ConnectorUser':
            return np.zeros((kind.ltg.shape[1], kind.ltg.shape[1]))
        if kind.objectType == 'GenericDAE': #a massMatrixUserFunction only without ODE1 coordinates, so y is empty
            if 'massMatrixUserFunction' in kind.recordings:
                return self._DAECall(kind, 'massMatrixUserFunction', i, q, q_t, np.zeros(self.nODE1), t)
            return kind.parameters[i]['M'] if kind.parameters[i]['M'].size else np.zeros((kind.ltg.shape[1], kind.ltg.shape[1]))
        if 'massMatrixUserFunction' in kind.recordings:
            return self._Call(kind, 'massMatrixUserFunction', i, q, q_t, t)
        return kind.parameters[i]['M']

    def _LocalForces(self, kind, i, q, q_t, t):
        """the forces on the right-hand side; for ObjectGenericODE2 f + f_user - K q - D q_t"""
        f = np.zeros(kind.ltg.shape[1])
        if kind.objectType == 'ConnectorUser': #J_pos^T f + J_rot^T m of each marker, into the coordinates of its node or body
            (arguments, markerData) = self._ConnectorStates(kind, i, q, q_t, t)
            forces = np.array(kind.recordings['forceUserFunction'].Evaluate(self.mbs, *arguments), dtype=float)
            stride = 6 if kind.parameters[i]['orientation'] else 3
            for (k, (marker, (Jpos, Jrot, bodyArguments))) in enumerate(zip(kind.parameters[i]['markers'], markerData)):
                if Jpos is not None:
                    f[marker['indices']] += Jpos.T @ forces[stride*k:stride*k + 3]
                if Jrot is not None:
                    f[marker['indices']] += Jrot.T @ forces[stride*k + 3:stride*k + 6]
            return f
        if 'forceUserFunction' in kind.recordings:
            f += self._Call(kind, 'forceUserFunction', i, q, q_t, t)
        if kind.objectType == 'GenericODE2':
            matrices = kind.parameters[i]
            ltg = kind.ltg[i]
            if len(matrices['f']):
                f += matrices['f']
            if matrices['K'].size:
                f -= matrices['K'] @ q[ltg]
            if matrices['D'].size:
                f -= matrices['D'] @ q_t[ltg]
        return f

    def ODE2RHS(self, q=None, q_t=None, t=0., y=None):
        """the global forces f(q, q_t, t, y) of the right-hand side, as the ODE2 part of the solver's right-hand side

        Args:
            q: the ODE2 coordinates (displacements), of size nODE2; None: zero
            q_t: the ODE2 velocities; None: zero
            t: the time
            y: the ODE1 coordinates, of size nODE1, used by an ObjectGenericDAE with ODE2 and ODE1 coordinates only -
                which needs them (#3190) -; None: zero

        Returns:
            the vector f of size nODE2
        """
        (q, q_t, _) = self._State(q, q_t)
        y = self._ODE1State(y, 'ODE2RHS')
        if self.composed is not None:
            return np.array(self.composed['ODE2RHS'].Evaluate(*self._ComposedInputs(q, q_t, t, y)), dtype=float)
        f = np.zeros(self.nODE2)
        for kind in self.kinds:
            for i in range(len(kind.objects)):
                np.add.at(f, kind.ltg[i], self._LocalForces(kind, i, q, q_t, t))
        for kind in self._DAEKindsODE2():
            for i in range(len(kind.objects)):
                np.add.at(f, kind.ltg[i], self._DAEForces(kind, i, q, q_t, y, t))
        for load in self.loads:
            np.add.at(f, load.ltg, self._LoadForces(load, q, q_t, t))
        return f

    def _DAEKindsODE2(self):
        """the kinds of the ObjectGenericDAE with ODE2 coordinates"""
        return [kind for kind in self.daeKinds if kind.ltg.shape[1]]

    def _BodyKinds(self):
        """the kinds with a mass matrix, in the order of MassMatrixBlocks: the kinds of the bodies, then those of the
        ObjectGenericDAE with ODE2 coordinates and a mass matrix - one without acts on the coordinates of other items, by
        its constraints, as of CreateCoordinateConstraint of UserItemCreator (#3193)"""
        return ([kind for kind in self.kinds if kind.objectType != 'ConnectorUser']
                + [kind for kind in self._DAEKindsODE2() if 'massMatrixUserFunction' in kind.recordings or kind.parameters[0]['M'].size])

    def _ComposedInputs(self, q, q_t, t, y=None):
        """the arguments of the recordings of the composed system: q, q_t, t, and y if the ODE2 equations depend on it;
        None: zero, for the constant mass matrix of ObjectGenericDAE"""
        inputs = (q, q_t, t) if not self._ode2UsesY else (q, q_t, t, np.zeros(self.nODE1) if y is None else y)
        return inputs + ((self.data,) if self.nData else ())

    def MassMatrixTriplets(self, q=None, q_t=None, t=0.):
        """the global mass matrix as triplets (rows, columns, values), a value per nonzero local entry; equal rows
        and columns are added when the matrix is formed"""
        (q, q_t, _) = self._State(q, q_t)
        if self.composed is not None:
            values = np.array(self.composed['massMatrix'].Evaluate(*self._ComposedInputs(q, q_t, t)), dtype=float)
            return self.composed['rows'], self.composed['columns'], values[:len(self.composed['rows'])]
        rows, columns, values = [], [], []
        for kind in self.kinds + self._DAEKindsODE2():
            for i in range(len(kind.objects)):
                local = self._LocalMassMatrix(kind, i, q, q_t, t)
                (r, c) = np.nonzero(local)
                rows.append(kind.ltg[i][r])
                columns.append(kind.ltg[i][c])
                values.append(local[r, c])
        if not rows:
            return np.zeros(0, dtype=int), np.zeros(0, dtype=int), np.zeros(0)
        return np.concatenate(rows), np.concatenate(columns), np.concatenate(values)

    def MassMatrixBlocks(self, q=None, q_t=None, t=0.):
        """the local mass matrices of the objects, per kind: a list of (ltg, blocks), the LTG table of the kind and its
        mass matrices (objects x n x n)"""
        (q, q_t, _) = self._State(q, q_t)
        kinds = self._BodyKinds()
        if self.composed is not None: #one call: the entries of all blocks, kind after kind
            values = np.array(self.composed['massMatrixBlocks'].Evaluate(*self._ComposedInputs(q, q_t, t)), dtype=float)
            (result, start) = ([], 0)
            for kind in kinds:
                (m, nLocal) = kind.ltg.shape
                result.append((kind.ltg, values[start:start + m*nLocal*nLocal].reshape(m, nLocal, nLocal)))
                start += m*nLocal*nLocal
            return result
        return [(kind.ltg, np.array([self._LocalMassMatrix(kind, i, q, q_t, t) for i in range(len(kind.objects))]))
                for kind in kinds]

    def ODE2ResidualLHS(self, q=None, q_t=None, q_tt=None, t=0., y=None):
        """the residual r = M(q) q_tt - f(q, q_t, t, y) of the ODE2 equations, LHS minus RHS, without forming M: by the
        recordings of the composed system, else object by object; y as in ODE2RHS"""
        (q, q_t, q_tt) = self._State(q, q_t, q_tt)
        y = self._ODE1State(y, 'ODE2ResidualLHS')
        if self.composed is not None:
            (rows, columns, values) = self.MassMatrixTriplets(q, q_t, t)
            return np.bincount(rows, values*q_tt[columns], minlength=self.nODE2) - self.ODE2RHS(q, q_t, t, y=y)
        r = np.zeros(self.nODE2)
        for kind in self.kinds:
            for i in range(len(kind.objects)):
                ltg = kind.ltg[i]
                local = self._LocalMassMatrix(kind, i, q, q_t, t) @ q_tt[ltg] - self._LocalForces(kind, i, q, q_t, t)
                np.add.at(r, ltg, local)
        for kind in self._DAEKindsODE2():
            for i in range(len(kind.objects)):
                ltg = kind.ltg[i]
                np.add.at(r, ltg, self._LocalMassMatrix(kind, i, q, q_t, t) @ q_tt[ltg] - self._DAEForces(kind, i, q, q_t, y, t))
        for load in self.loads:
            np.add.at(r, load.ltg, -self._LoadForces(load, q, q_t, t))
        return r

    def ODE2ResidualLHSJacobian(self, q=None, q_t=None, q_tt=None, t=0., factor_q=1., factor_q_t=0., factor_q_tt=0.,
                         sparse=False, y=None):
        """the Jacobian of the residual, factor_q dr/dq + factor_q_t dr/dq_t + factor_q_tt dr/dq_tt, exact by the
        derivatives of the recordings (UserFunction.EvaluateJacobian, #3115, in one pass over the flat program #3116):
        of the composed system up to _denseMassMatrixSize coordinates - two calls, a dense Jacobian -, else object by
        object, sparse; factor_q_tt=1 alone gives the mass matrix, a Newton step of an implicit integrator combines
        the three

        Args:
            q: the ODE2 coordinates, of size nODE2; None: zero
            q_t: the ODE2 velocities; None: zero
            q_tt: the ODE2 accelerations; None: zero
            t: the time
            factor_q: the factor of dr/dq
            factor_q_t: the factor of dr/dq_t
            factor_q_tt: the factor of dr/dq_tt
            sparse: True: a scipy.sparse.csr_matrix, else a dense array
            y: the ODE1 coordinates, as in ODE2RHS

        Returns:
            the matrix (nODE2 x nODE2)
        """
        (q, q_t, q_tt) = self._State(q, q_t, q_tt)
        y = self._ODE1State(y, 'ODE2ResidualLHSJacobian')
        if self.composed is not None and self.nODE2 <= _denseMassMatrixSize: #r = M q_tt - f of the two recordings
            composed = self.composed
            inputs = self._ComposedInputs(q, q_t, t, y)
            J = np.zeros((self.nODE2, self.nODE2))
            for (name, factor) in [('q', factor_q), ('q_t', factor_q_t)]:
                if factor != 0.:
                    J -= factor*np.array(composed['ODE2RHS'].EvaluateJacobian(name, *inputs), dtype=float)
            (rows, columns) = (composed['rows'], composed['columns'])
            if factor_q != 0. and len(rows) and not self._constantMassMatrix: #d(M q_tt)/dq
                dM = np.array(composed['massMatrix'].EvaluateJacobian('q', *inputs), dtype=float)[:len(rows)]
                np.add.at(J, rows, factor_q*dM*q_tt[columns][:, np.newaxis])
            if factor_q_tt != 0. and len(rows):
                np.add.at(J, (rows, columns), factor_q_tt*self.MassMatrixTriplets(q, q_t, t)[2])
            (r, c) = np.nonzero(J)
            return self._Matrix(r, c, J[r, c], sparse)
        rows, columns, values = [], [], []
        for kind in self.kinds:
            for i in range(len(kind.objects)):
                ltg = kind.ltg[i]
                local = np.zeros((len(ltg), len(ltg)))
                if kind.objectType == 'ConnectorUser':
                    local = self._ConnectorJacobian(kind, i, q, q_t, t, factor_q, factor_q_t)
                    (r, c) = np.nonzero(local)
                    rows.append(ltg[r])
                    columns.append(ltg[c])
                    values.append(local[r, c])
                    continue
                (position_q, position_q_t) = _argumentPositions[kind.objectType]
                if 'forceUserFunction' in kind.recordings: #r = M q_tt - f
                    if factor_q != 0.:
                        local -= factor_q*self._Jacobian(kind, 'forceUserFunction', i, position_q, q, q_t, t)
                    if factor_q_t != 0.:
                        local -= factor_q_t*self._Jacobian(kind, 'forceUserFunction', i, position_q_t, q, q_t, t)
                if 'massMatrixUserFunction' in kind.recordings:
                    if factor_q != 0.: #d(M q_tt)/dq
                        local += factor_q*np.einsum('ijk,j->ik', self._Jacobian(kind, 'massMatrixUserFunction', i, position_q,
                                                                               q, q_t, t), q_tt[ltg])
                    if factor_q_tt != 0.:
                        local += factor_q_tt*self._Call(kind, 'massMatrixUserFunction', i, q, q_t, t)
                if kind.objectType == 'GenericODE2': #the constant matrices of r = M q_tt + D q_t + K q - f
                    matrices = kind.parameters[i]
                    for (name, factor) in [('K', factor_q), ('D', factor_q_t), ('M', factor_q_tt)]:
                        if name == 'M' and 'massMatrixUserFunction' in kind.recordings:
                            continue #the user function computes the mass matrix instead
                        if matrices[name].size and factor != 0.:
                            local += factor*matrices[name]
                (r, c) = np.nonzero(local)
                rows.append(ltg[r])
                columns.append(ltg[c])
                values.append(local[r, c])
        for kind in self._DAEKindsODE2(): #factor_q (K - df_u/dq) + factor_q_t (D - df_u/dq_t) + factor_q_tt M (#3190)
            for i in range(len(kind.objects)):
                (ltg, dae) = (kind.ltg[i], kind.parameters[i])
                local = factor_q_tt*self._LocalMassMatrix(kind, i, q, q_t, t)
                if 'massMatrixUserFunction' in kind.recordings and factor_q != 0.: #d(M q_tt)/dq (#3191)
                    local = local + factor_q*np.einsum('ijk,j->ik', self._DAEJacobian(kind, 'massMatrixUserFunction', i, _daeQ, q, q_t,
                                                                                      np.zeros(self.nODE1), t), q_tt[ltg])
                for (name, position, factor) in [('K', _daeQ, factor_q), ('D', _daeQ_t, factor_q_t)]:
                    if factor == 0.:
                        continue
                    if dae[name].size:
                        local = local + factor*dae[name]
                    if 'forceUserFunction' in kind.recordings:
                        local = local - factor*self._DAEJacobian(kind, 'forceUserFunction', i, position, q, q_t, y, t)
                (r, c) = np.nonzero(local)
                rows.append(ltg[r])
                columns.append(ltg[c])
                values.append(local[r, c])
        for load in self.loads:
            local = self._LoadJacobian(load, q, q_t, t, factor_q, factor_q_t)
            (r, c) = np.nonzero(local)
            rows.append(load.ltg[r])
            columns.append(load.ltg[c])
            values.append(local[r, c])
        if not rows:
            return self._Matrix(np.zeros(0, dtype=int), np.zeros(0, dtype=int), np.zeros(0), sparse)
        return self._Matrix(np.concatenate(rows), np.concatenate(columns), np.concatenate(values), sparse)

    def _ItemFunction(self, kind, i, joint):
        """connector i of kind as one exudyn.symbolic.Function (#3166): its generalized forces, J_pos^T f + J_rot^T m of
        its markers - for a joint its constraint equations c and its reaction forces C_q^T z, one vector -, of the
        variables q<g> and q_t<g>, the local coordinates of each body or node g of its markers, then z (of a joint) and t;
        the same expressions as Equations(); returned with the local indices of each g, built once"""
        key = (id(kind), i)
        if key in self._itemFunctions:
            return self._itemFunctions[key]
        (item, objectNumber) = (kind.parameters[i], kind.objects[i])
        n = kind.ltg.shape[1]
        groups = [] #the local indices of the coordinates of each body or node
        for marker in item['markers']:
            group = tuple(int(j) for j in marker['indices']) if marker['type'] != 'fixed' else None
            if group is not None and group not in groups:
                groups.append(group)
        if len(set(j for group in groups for j in group)) != sum(len(group) for group in groups):
            groups = [tuple(range(n))] #a node marker on a node of a body marker: one variable of all coordinates
        variables = exudyn.symbolic.VariableSet()
        (qs, q_ts) = ([0.]*n, [0.]*n)
        for (g, group) in enumerate(groups):
            (qg, q_tg) = (variables.Add('q' + str(g), np.zeros(len(group))), variables.Add('q_t' + str(g), np.zeros(len(group))))
            for (a, j) in enumerate(group):
                (qs[j], q_ts[j]) = (qg[a], q_tg[a])
        nConstraints = len(item['ltgAE']) if joint else 0
        z = variables.Add('z', np.zeros(nConstraints)) if joint else None
        x = variables.Add('x', np.zeros(len(item['ltgData']))) if not joint else None #the data coordinates (#3196)
        t = variables.Add('t', 0.)
        (backend, Call, local) = (_Backend('symbolic', parameterVariables=False), _Caller('symbolic'), np.arange(n))
        (states, jacobians) = self._MarkerStateExpressions(item, local, backend, Call, t, qs, q_ts)
        parameters = self._ParameterVector(backend, objectNumber, item['parameters'])
        f = [0]*n
        if joint:
            arguments = (t, objectNumber, *_JointLeaves(states), parameters)
            c = Call(kind.recordings['constraintUserFunction'], *arguments)
            C = Call(kind.recordings['jacobianUserFunction'], *arguments)
            self._AddMarkerForces(item, local, backend, C.T @ backend.Vector([z[k] for k in range(nConstraints)]), 6, jacobians, f)
            result = backend.Vector([c[k] for k in range(nConstraints)] + f)
        else:
            forces = Call(kind.recordings['forceUserFunction'], t, objectNumber, *_ConnectorLeaves(states), x, parameters)
            self._AddMarkerForces(item, local, backend, forces, 6 if item['orientation'] else 3, jacobians, f)
            result = backend.Vector(f)
        self._itemFunctions[key] = (exudyn.symbolic.Function(result, variables, 'Item' + str(int(objectNumber))), groups)
        return self._itemFunctions[key]

    def _ItemJacobians(self, kind, i, q, q_t, z, t, joint, velocities):
        """the derivatives of the Function of connector i of kind (_ItemFunction) by the coordinates of each of its bodies
        and nodes - and with velocities by their velocities -, each with the local indices of its columns"""
        (function, groups) = self._ItemFunction(kind, i, joint)
        ltg = kind.ltg[i]
        values = []
        for group in groups:
            values += [q[ltg[list(group)]], q_t[ltg[list(group)]]]
        if joint:
            values.append(z[kind.parameters[i]['ltgAE']])
        else:
            values.append(self.data[kind.parameters[i]['ltgData']])
        values.append(float(t))
        derivatives = []
        for (g, group) in enumerate(groups):
            d_q = np.array(function.EvaluateJacobian('q' + str(g), *values), dtype=float).reshape(-1, len(group))
            d_q_t = None
            if velocities:
                d_q_t = np.array(function.EvaluateJacobian('q_t' + str(g), *values), dtype=float).reshape(-1, len(group))
            derivatives.append((list(group), d_q, d_q_t))
        return derivatives

    def _ConnectorJacobian(self, kind, i, q, q_t, t, factor_q, factor_q_t):
        """the local Jacobian of the residual -f of a connector, -(factor_q df/dq + factor_q_t df/dq_t) of its generalized
        forces f, by the one-pass derivative of their expression (_ItemFunction, #3166)"""
        n = kind.ltg.shape[1]
        local = np.zeros((n, n))
        for (columns, d_q, d_q_t) in self._ItemJacobians(kind, i, q, q_t, None, t, False, factor_q_t != 0.):
            if factor_q != 0.:
                local[:, columns] -= factor_q*d_q
            if factor_q_t != 0.:
                local[:, columns] -= factor_q_t*d_q_t
        return local

    def _JointJacobian(self, kind, i, q, z, t):
        """the derivatives of joint i of kind in its local coordinates: C_q, the derivative of its constraint equations,
        and the derivative of its reaction forces C_q^T z by the coordinates, by the one-pass derivative of their
        expression (_ItemFunction, #3166)"""
        n = kind.ltg.shape[1]
        nConstraints = len(kind.parameters[i]['ltgAE'])
        (C_q, dReaction) = (np.zeros((nConstraints, n)), np.zeros((n, n)))
        for (columns, d_q, d_q_t) in self._ItemJacobians(kind, i, q, np.zeros(self.nODE2), z, t, True, False):
            C_q[:, columns] += d_q[:nConstraints]
            dReaction[:, columns] += d_q[nConstraints:]
        return (C_q, dReaction)

    def Equations(self, format='symbolic', optimize=True, variables=None, massMatrix=True, ODE2RHS=True,
                  massMatrixBlocks=False, algebraicEquations=False, ODE2ReactionForcesLHS=False, ODE1RHS=False,
                  ODE1ReactionTermsLHS=False, postNewton=False):
        """the equations of motion M(q, t) q_tt = f(q, q_t, t) of the system as expressions in its variables (#3130,
        #3132), composed of the recordings of the kinds, the marker kinematics of the connectors and the loads; each
        recording is called with symbolic arguments through its code of exudyn.symbolics.codeExport, the parameters of
        the items and the reference coordinates are numbers. The forces f are the whole right-hand side - the forces of
        the bodies (their quadratic velocity terms), of the connectors and of the loads -, the same whether the
        accelerations are solved with the global mass matrix or with the inverse of the mass matrix of each body
        (explicit.computeMassMatrixInversePerBody), for which massMatrixBlocks gives the matrices of the bodies

        Args:
            format: 'symbolic': exudyn.symbolic expressions, the variables named vectors q, q_t (with the initial state
                as values) and the real t; 'sympy': sympy expressions of the symbols q0, q1, ..., q_t0, ..., t (sympy
                must be installed)
            optimize: for 'symbolic', simplify the expressions by exudyn.symbolic.Optimize, level 2 (1*x, x+0, 0*x,
                the combination of numbers, ...)
            variables: None, or (q, q_t, t) of the format given by the caller - e.g. the arguments of a function being
                recorded -, used instead of new variables; (q, q_t, t, z) with the algebraic coordinates, (q, q_t, t, z,
                y) with the ODE1 coordinates
            massMatrix: True: the global mass matrix, 'massMatrix' (nODE2 x nODE2)
            ODE2RHS: True: the forces of the right-hand side, 'ODE2RHS' (nODE2)
            massMatrixBlocks: True: the mass matrices of the bodies, 'massMatrixBlocks', as MassMatrixBlocks() gives
                them: a list per kind of bodies of (ltg, matrices), the LTG table of the kind (bodies x n) and the list
                of their n x n matrices
            algebraicEquations: True: the constraint equations c(q, t) of the joints, 'algebraicEquations' (nAE), as
                AlgebraicEquations (#3146)
            ODE2ReactionForcesLHS: True: the reaction forces C_q^T z of the joints, 'ODE2ReactionForcesLHS' (nODE2), as
                ODE2ReactionForcesLHS, of the algebraic coordinates, the variable 'z' (#3146)
            ODE1RHS: True: the right-hand side g(q, q_t, y, t) of the ODE1 equations y_t = g, 'ODE1RHS' (nODE1), as
                ODE1RHS, of the ODE1 coordinates, the variable 'y' with the initial ODE1 state (#3162)
            ODE1ReactionTermsLHS: True: the reaction terms C_y^T z of the ObjectGenericDAE in the ODE1 equations,
                'ODE1ReactionTermsLHS' (nODE1), as ODE1ReactionTermsLHS, of the variable 'z' (#3190)
            postNewton: True: 'postNewton', the results [error, recommendedStepSize, data] of the recorded post-Newton
                functions, item after item as PostNewton takes them, of the variables and 'x0', the data coordinates at
                the start of the step (#3196)

        Returns:
            dict with 'q', 'q_t', 't' (the variables), 'z' if ODE2ReactionForcesLHS, ODE1ReactionTermsLHS or a
            constraintUserFunction in algebraicEquations (#3191), 'y' if ODE1RHS or the system has an ObjectGenericDAE
            with ODE1 coordinates (#3190), 'x' - the data coordinates, which the equations read as constant in a step -
            if the system has data coordinates, 'x0' if postNewton (#3196), 'parameters' - the named parameters
            of parameterBindings, as exudyn.symbolic.Real or sympy symbols - if there are any (#3152, #3159), and the results
            asked for, exudyn.symbolic.Matrix and Vector or sympy.Matrix
        """
        backend = _Backend(format)
        n = self.nODE2
        (q0, q_t0) = self.InitialState()
        z = None
        if variables is None:
            (q, q_t) = (backend.Variable(name, n, value) for (name, value) in [('q', q0), ('q_t', q_t0)])
            t = backend.Real('t')
        else:
            (q, q_t, t) = variables[:3]
            if len(variables) > 3:
                z = variables[3]
        withZ = (ODE2ReactionForcesLHS or ODE1ReactionTermsLHS or postNewton
                 or (algebraicEquations and any('constraintUserFunction' in kind.recordings for kind in self.daeKinds)))
        if withZ and z is None:
            z = backend.Variable('z', self.nAE, np.zeros(self.nAE))
        y = variables[4] if variables is not None and len(variables) > 4 else None
        withY = ODE1RHS or postNewton or (bool(self.daeKinds) and self.nODE1 > 0)
        if withY and y is None:
            y = backend.Variable('y', self.nODE1, self.InitialStateODE1())
        x = variables[5] if variables is not None and len(variables) > 5 else None
        if self.nData and x is None:
            x = backend.Variable('x', self.nData, self.data)
        x0 = backend.Variable('x0', self.nData, self.data) if postNewton else None
        (M, f, blocks, c, r) = self._EquationLists(backend, q, q_t, t, z if withZ else None,
                                                   algebraicEquations or ODE2ReactionForcesLHS, y=y, reactions=ODE2ReactionForcesLHS, x=x)
        (names, results) = ([], [])
        for (name, wanted, Result) in [('massMatrix', massMatrix, lambda: backend.Matrix(M)), ('ODE2RHS', ODE2RHS, lambda: backend.Vector(f)),
                                       ('algebraicEquations', algebraicEquations, lambda: backend.Vector(c)),
                                       ('ODE2ReactionForcesLHS', ODE2ReactionForcesLHS, lambda: backend.Vector(r)),
                                       ('ODE1RHS', ODE1RHS, lambda: backend.Vector(self._ODE1Equations(backend, y, t, q, q_t, x))),
                                       ('ODE1ReactionTermsLHS', ODE1ReactionTermsLHS, lambda: backend.Vector(self._ODE1ReactionTerms(backend, z, q, q_t, y, t, x))),
                                       ('postNewton', postNewton, lambda: backend.Vector(self._PostNewtonEquations(backend, q, q_t, y, z, x, x0, t)))]:
            if wanted:
                names.append(name)
                results.append(Result())
        if massMatrixBlocks:
            names += ['massMatrixBlocks']
            results += [[backend.Matrix(block) for (ltg, kindBlocks) in blocks for block in kindBlocks]]
        if optimize and format == 'symbolic':
            results = [exudyn.symbolic.Optimize(result, level=2) if name != 'massMatrixBlocks' or result else result
                       for (name, result) in zip(names, results)]
        equations = {'q': q, 'q_t': q_t, 't': t}
        if withZ:
            equations['z'] = z
        if withY:
            equations['y'] = y
        if self.nData:
            equations['x'] = x
        if postNewton:
            equations['x0'] = x0
        if self._parameterNames:
            equations['parameters'] = [backend.Parameter(value) for value in self._parameterNames.values()]
        for (name, result) in zip(names, results):
            equations[name] = result
        if massMatrixBlocks: #back into the kinds
            matrices = list(equations['massMatrixBlocks'])
            equations['massMatrixBlocks'] = []
            for (ltg, kindBlocks) in blocks:
                equations['massMatrixBlocks'].append((ltg, matrices[:len(kindBlocks)]))
                matrices = matrices[len(kindBlocks):]
        return equations

    def _EquationLists(self, backend, q, q_t, t, z=None, constraints=False, y=None, reactions=True, x=None):
        """the mass matrix as list of rows, the forces as list of the expressions of Equations, of the variables q, q_t
        and t of backend - and y, the ODE1 coordinates, for an ObjectGenericDAE (#3190) -, the local mass matrices of the
        bodies, per kind (ltg, list of their lists of rows), and with constraints the constraint equations of the joints
        and the ObjectGenericDAE and - for the algebraic coordinates z and reactions - their reaction forces as lists
        (#3146); x the data coordinates, constant in a step, numbers of data if None (#3196)"""
        xs = _DataComponents(x, self)
        n = self.nODE2
        format = backend.format
        (qs, q_ts) = ([x[i] for i in range(n)] for x in [q, q_t]) #the components, to be gathered per item
        M = [[0]*n for i in range(n)]
        f = [0]*n
        bodyKinds = self._BodyKinds()
        blocks = [(kind.ltg, []) for kind in bodyKinds]
        blocksOfKind = {id(kind): blocks[k][1] for (k, kind) in enumerate(bodyKinds)}
        Call = _Caller(format)

        def Add(target, ltg, values):
            for (i, value) in zip(ltg, values):
                target[i] = _Sum(target[i], value)

        def Local(values, ltg, offset=None):
            return backend.Vector([(float(offset[k]) + values[i] if offset is not None else values[i]) for (k, i) in enumerate(ltg)])

        def Components(vector, size):
            return [vector[i] for i in range(size)]

        for kind in self.kinds:
            for (index, objectNumber) in enumerate(kind.objects):
                ltg = kind.ltg[index]
                nLocal = len(ltg)
                if kind.objectType == 'UserBody':
                    qRef = self.qRef[ltg]
                    arguments = (t, objectNumber, backend.Vector(qRef), Local(qs, ltg, qRef), Local(q_ts, ltg),
                                 self._ParameterVector(backend, objectNumber, kind.parameters[index]))
                    Mlocal = Call(kind.recordings['massMatrixUserFunction'], *arguments)
                    blocksOfKind[id(kind)].append([[Mlocal[i, j] for j in range(nLocal)] for i in range(nLocal)])
                    for i in range(nLocal):
                        for j in range(nLocal):
                            M[ltg[i]][ltg[j]] = _Sum(M[ltg[i]][ltg[j]], Mlocal[i, j])
                    if 'forceUserFunction' in kind.recordings:
                        Add(f, ltg, Components(Call(kind.recordings['forceUserFunction'], *arguments), nLocal))
                elif kind.objectType == 'GenericODE2':
                    matrices = kind.parameters[index]
                    arguments = (t, objectNumber, Local(qs, ltg), Local(q_ts, ltg))
                    if 'massMatrixUserFunction' in kind.recordings:
                        Mlocal = Call(kind.recordings['massMatrixUserFunction'], *arguments)
                    else:
                        Mlocal = matrices['M']
                    Mlocal = [[float(Mlocal[i, j]) if isinstance(Mlocal, np.ndarray) else Mlocal[i, j] for j in range(nLocal)]
                              for i in range(nLocal)]
                    blocksOfKind[id(kind)].append(Mlocal)
                    for i in range(nLocal):
                        for j in range(nLocal):
                            M[ltg[i]][ltg[j]] = _Sum(M[ltg[i]][ltg[j]], Mlocal[i][j])
                    if 'forceUserFunction' in kind.recordings:
                        Add(f, ltg, Components(Call(kind.recordings['forceUserFunction'], *arguments), nLocal))
                    if len(matrices['f']):
                        Add(f, ltg, [float(x) for x in matrices['f']])
                    for (name, x) in [('K', qs), ('D', q_ts)]: #f - K q - D q_t
                        if matrices[name].size:
                            Add(f, ltg, [-sum(float(matrices[name][i, j])*x[ltg[j]] for j in range(nLocal) if matrices[name][i, j] != 0)
                                         for i in range(nLocal)])
                else:
                    self._ConnectorEquations(kind, index, backend, Call, t, qs, q_ts, f, xs)
        for load in self.loads:
            if load.recording is None:
                value = backend.Vector(np.atleast_1d(load.value))
            elif load.loadType == 'Coordinate':
                value = backend.Vector([Call(load.recording, t, float(load.value))])
            else:
                value = Call(load.recording, t, backend.Vector(load.value))
            if load.body is None:
                Add(f, load.ltg, Components(value, len(load.ltg)))
                continue
            body = load.body
            arguments = (t, body['objectNumber'], backend.Vector(body['qRef']), Local(qs, load.ltg, body['qRef']), Local(q_ts, load.ltg),
                         self._ParameterVector(backend, body['objectNumber'], body['parameters']))
            withPosition = arguments[:2] + (backend.Vector(body['localPosition']),) + arguments[2:]
            J = Call(body['projection'], *(arguments if body['projectionName'] == 'massIntegralUserFunction' else withPosition))
            if body['rotation'] is not None:
                value = Call(body['rotation'], *withPosition) @ value
            Add(f, load.ltg, Components(J.T @ value, len(load.ltg)))
        (c, r) = ([0]*self.nAE, [0]*n)
        zComponents = None if (z is None or not constraints or not reactions) else [z[i] for i in range(self.nAE)]
        if constraints:
            for kind in self.jointKinds:
                for index in range(len(kind.objects)):
                    self._JointEquations(kind, index, backend, Call, t, qs, q_ts, zComponents, c, r)
        ys = None if y is None else [y[i] for i in range(self.nODE1)]
        zAll = None if z is None else [z[i] for i in range(self.nAE)] #c_u may depend on z without the reaction forces
        for kind in self.daeKinds:
            for index in range(len(kind.objects)):
                self._DAEEquations(kind, index, backend, Call, t, qs, q_ts, ys, zComponents, constraints, M, f,
                                   blocksOfKind.get(id(kind)), c, r, q, zAll, xs)
        return (M, f, blocks, c, r)

    def _DAEEquations(self, kind, index, backend, Call, t, qs, q_ts, ys, zComponents, constraints, M, f, blocks, c, r, q, zAll, xs):
        """add the equations of ObjectGenericDAE index of kind to the lists of _EquationLists (#3190): its mass matrix to
        M and blocks, f + f_u - K q - D q_t - E y to f, with constraints C_q q + C_y y - c_0 + c_u into c, and for
        zComponents (C_q + dc_u/dq)^T z to r, dc_u/dq by backend.Jacobian of the variable q (#3191); ys the components of
        the ODE1 coordinates, needed if it has some; zAll those of z, needed for a constraintUserFunction"""
        (ltg, dae, objectNumber) = (kind.ltg[index], kind.parameters[index], kind.objects[index])
        (qLocal, q_tLocal) = ([qs[i] for i in ltg], [q_ts[i] for i in ltg])
        xLocal = [xs[i] for i in dae['ltgData']]
        ltgAE = dae['ltgAE']
        if not len(dae['ltgODE1']):
            yLocal = []
        elif ys is not None:
            yLocal = [ys[i] for i in dae['ltgODE1']]
        elif len(ltg) or (constraints and len(ltgAE)):
            raise ValueError('SymbolicSystem: object ' + str(objectNumber) + ' (GenericDAE) has ODE1 coordinates, so its '
                             'equations need the variable y')

        def Add(target, indices, values):
            if values is not None:
                for (i, value) in zip(indices, values):
                    target[i] = _Sum(target[i], value)

        if len(ltg):
            if 'massMatrixUserFunction' in kind.recordings: #(#3191)
                value = Call(kind.recordings['massMatrixUserFunction'], *self._DAESymbolicArguments(backend, kind, index, t, qLocal,
                                                                                                    q_tLocal, yLocal, xLocal))
                Mlocal = [[value[i, j] for j in range(len(ltg))] for i in range(len(ltg))]
            elif dae['M'].size:
                Mlocal = [[float(dae['M'][i, j]) for j in range(len(ltg))] for i in range(len(ltg))]
            else: #no mass, not a body of MassMatrixBlocks
                Mlocal = None
            if blocks is not None:
                blocks.append(Mlocal)
            for (i, row) in zip(ltg, Mlocal if Mlocal is not None else []):
                for (j, value) in zip(ltg, row):
                    M[i][j] = _Sum(M[i][j], value)
            Add(f, ltg, [float(x) for x in dae['f']] if len(dae['f']) else None)
            for (name, x) in [('K', qLocal), ('D', q_tLocal), ('E', yLocal)]:
                Add(f, ltg, _MatrixTimes(dae[name], x, -1.))
            if 'forceUserFunction' in kind.recordings:
                value = Call(kind.recordings['forceUserFunction'], *self._DAESymbolicArguments(backend, kind, index, t, qLocal,
                                                                                               q_tLocal, yLocal, xLocal))
                Add(f, ltg, [value[i] for i in range(len(ltg))])
        if constraints and len(ltgAE):
            for (name, x) in [('C_q', qLocal), ('C_y', yLocal)]:
                Add(c, ltgAE, _MatrixTimes(dae[name], x))
            Add(c, ltgAE, [-float(x) for x in dae['c0']] if len(dae['c0']) else None)
            if zComponents is not None and dae['C_q'].size:
                Add(r, ltg, _MatrixTimes(dae['C_q'].T, [zComponents[i] for i in ltgAE]))
            if 'constraintUserFunction' in kind.recordings:
                cUser = self._DAEConstraintExpression(backend, Call, kind, index, t, qLocal, q_tLocal, yLocal, zAll, xLocal)
                Add(c, ltgAE, cUser)
                if zComponents is not None and len(ltg): #(dc_u/dq)^T z, dense in the system coordinates
                    Add(r, range(self.nODE2), backend.TransposedJacobianTimes(cUser, q, [zComponents[i] for i in ltgAE], self.nODE2))

    def _DAEConstraintExpression(self, backend, Call, kind, index, t, qLocal, q_tLocal, yLocal, zAll, xLocal):
        """the term c_u of the recorded constraintUserFunction of ObjectGenericDAE index of kind at position level, as list
        of expressions (#3191)"""
        (dae, objectNumber) = (kind.parameters[index], kind.objects[index])
        if zAll is None:
            raise ValueError('SymbolicSystem: object ' + str(objectNumber) + ' (GenericDAE) has a constraintUserFunction, so its '
                             'equations need the variable z')
        value = Call(kind.recordings['constraintUserFunction'], t, objectNumber, backend.Vector(qLocal), backend.Vector(q_tLocal),
                     backend.Vector(yLocal), backend.Vector([zAll[i] for i in dae['ltgAE']]), backend.Vector(xLocal), 0.,
                     self._ParameterVector(backend, objectNumber, dae['parameters']))
        return [value[k] for k in range(len(dae['ltgAE']))]

    def _DAESymbolicArguments(self, backend, kind, index, t, qLocal, q_tLocal, yLocal, xLocal):
        """the arguments after mbs of the recorded user functions of ObjectGenericDAE index of kind as expressions"""
        objectNumber = kind.objects[index]
        return (t, objectNumber, backend.Vector(qLocal), backend.Vector(q_tLocal), backend.Vector(yLocal), backend.Vector(xLocal),
                self._ParameterVector(backend, objectNumber, kind.parameters[index]['parameters']))

    def _ODE1ReactionTerms(self, backend, z, q, q_t, y, t, x=None):
        """the reaction terms (C_y + dc_u/dy)^T z of the ObjectGenericDAE in the ODE1 equations as list of expressions of
        z - and of q, q_t, y for a constraintUserFunction, dc_u/dy by backend of the variable y (#3190, #3191)"""
        g = [0]*self.nODE1
        (qs, q_ts, zs) = ([q[i] for i in range(self.nODE2)], [q_t[i] for i in range(self.nODE2)], [z[i] for i in range(self.nAE)])
        ys = None if y is None else [y[i] for i in range(self.nODE1)]
        xs = _DataComponents(x, self)
        Call = _Caller(backend.format)
        for kind in self.daeKinds:
            for (index, dae) in enumerate(kind.parameters):
                zLocal = [zs[k] for k in dae['ltgAE']]
                terms = []
                if dae['C_y'].size:
                    terms.append((dae['ltgODE1'], _MatrixTimes(dae['C_y'].T, zLocal)))
                if 'constraintUserFunction' in kind.recordings and len(dae['ltgODE1']):
                    ltg = kind.ltg[index]
                    cUser = self._DAEConstraintExpression(backend, Call, kind, index, t, [qs[i] for i in ltg], [q_ts[i] for i in ltg],
                                                          [ys[i] for i in dae['ltgODE1']], zs, [xs[i] for i in dae['ltgData']])
                    terms.append((range(self.nODE1), backend.TransposedJacobianTimes(cUser, y, zLocal, self.nODE1)))
                for (indices, values) in terms:
                    for (i, value) in zip(indices, values):
                        g[i] = _Sum(g[i], value)
        return g

    def _ODE1Equations(self, backend, y, t, q, q_t, x=None):
        """the right-hand side of the ODE1 equations as list of expressions of the ODE1 coordinates y - and q, q_t for an
        ObjectGenericDAE -, as ODE1RHS: per ObjectGenericODE1 A y + f and its recorded rhsUserFunction, per
        ObjectGenericDAE A y + B q + B_t q_t + g + g_u (#3190), and the LoadCoordinate on ODE1 coordinates (#3162)"""
        Call = _Caller(backend.format)
        ys = [y[i] for i in range(self.nODE1)]
        xs = _DataComponents(x, self)
        g = [0]*self.nODE1
        for kind in self.daeKinds:
            for (index, dae) in enumerate(kind.parameters):
                ltgODE1 = dae['ltgODE1']
                if not len(ltgODE1):
                    continue
                ltg = kind.ltg[index]
                (qLocal, q_tLocal, yLocal) = ([q[i] for i in ltg], [q_t[i] for i in ltg], [ys[i] for i in ltgODE1])
                terms = [[float(x) for x in dae['g']] if len(dae['g']) else None]
                terms += [_MatrixTimes(dae[name], x) for (name, x) in [('A', yLocal), ('B', qLocal), ('B_t', q_tLocal)]]
                if 'rhsUserFunction' in kind.recordings:
                    value = Call(kind.recordings['rhsUserFunction'], *self._DAESymbolicArguments(backend, kind, index, t, qLocal,
                                                                                                 q_tLocal, yLocal, [xs[i] for i in dae['ltgData']]))
                    terms.append([value[i] for i in range(len(ltgODE1))])
                for term in terms:
                    if term is not None:
                        for (i, value) in zip(ltgODE1, term):
                            g[i] = _Sum(g[i], value)
        for kind in self.ode1Kinds:
            for (index, objectNumber) in enumerate(kind.objects):
                (ltg, matrices) = (kind.ltg[index], kind.parameters[index])
                local = [0]*len(ltg)
                if matrices['A'].size:
                    local = [_Sum(local[i], sum(float(matrices['A'][i, j])*ys[ltg[j]] for j in range(len(ltg)) if matrices['A'][i, j] != 0))
                             for i in range(len(ltg))]
                if len(matrices['f']):
                    local = [_Sum(local[i], float(matrices['f'][i])) for i in range(len(ltg))]
                if 'rhsUserFunction' in kind.recordings:
                    value = Call(kind.recordings['rhsUserFunction'], t, objectNumber, backend.Vector([ys[i] for i in ltg]))
                    local = [_Sum(local[i], value[i]) for i in range(len(ltg))]
                for (i, value) in zip(ltg, local):
                    g[i] = _Sum(g[i], value)
        for load in self.ode1Loads:
            if load.recording is None:
                value = float(np.atleast_1d(load.value)[0])
            else:
                value = Call(load.recording, t, float(load.value))
            g[load.ltg[0]] = _Sum(g[load.ltg[0]], value)
        return g

    def _MarkerStateExpressions(self, connector, ltg, backend, Call, t, qs, q_ts):
        """the states of the markers of a connector or joint as expressions, as _ConnectorStates: per marker its
        position, velocity, rotation matrix and angular velocity in the list of states, and (kind, J_pos, J_rot) or
        None for a fixed marker in the list of Jacobians"""
        (states, jacobians) = ([], [])
        for marker in connector['markers']:
            (state, jacobian) = self._MarkerState(marker, ltg[marker['indices']], backend, Call, t, qs, q_ts, connector['orientation'])
            states += state
            jacobians.append(jacobian)
        return (states, jacobians)

    def _MarkerState(self, marker, indices, backend, Call, t, qs, q_ts, orientation):
        """the state of one marker as expressions of the components qs, q_ts of the system coordinates: the list of its
        position, velocity, rotation matrix and angular velocity, and (kind, J_pos, J_rot), None for a fixed marker;
        indices: the global indices of its coordinates; orientation: with the rotation of a body marker (#3151)"""
        (zero, unit) = (backend.Vector([0., 0., 0.]), backend.Matrix(np.eye(3)))
        if marker['type'] == 'fixed':
            return ([backend.Vector(marker['position']), zero, backend.Matrix(marker['rotation']), zero], None)
        if marker['type'] == 'point':
            return ([backend.Vector([float(marker['position'][a]) + qs[i] for (a, i) in enumerate(indices)]),
                     backend.Vector([q_ts[i] for i in indices]), unit, zero], ('point', None, None))
        recordings = marker['recordings']
        q_tLocal = backend.Vector([q_ts[i] for i in indices])
        arguments = (t, marker['objectNumber'], backend.Vector(marker['position']), backend.Vector(marker['qRef']),
                     backend.Vector([float(marker['qRef'][a]) + qs[i] for (a, i) in enumerate(indices)]), q_tLocal,
                     self._ParameterVector(backend, marker['objectNumber'], marker['parameters']))
        Jpos = Call(recordings['positionJacobianUserFunction'], *arguments)
        (rotation, angularVelocity, Jrot) = (unit, zero, None)
        if orientation:
            Jrot = Call(recordings['rotationJacobianUserFunction'], *arguments)
            rotation = Call(recordings['rotationMatrixUserFunction'], *arguments) @ backend.Matrix(marker['rotation'])
            angularVelocity = Jrot @ q_tLocal
        return ([Call(recordings['positionUserFunction'], *arguments), Jpos @ q_tLocal, rotation, angularVelocity],
                ('body', Jpos, Jrot))

    def _AddMarkerForces(self, connector, ltg, backend, forces, stride, jacobians, f):
        """add J_pos^T f + J_rot^T m of each marker of a connector or joint to the list f, the forces and torques of
        marker k at stride*k of forces"""
        for (k, (marker, jacobian)) in enumerate(zip(connector['markers'], jacobians)):
            if jacobian is None:
                continue
            indices = ltg[marker['indices']]
            force = backend.Vector([forces[stride*k + a] for a in range(3)])
            if jacobian[0] == 'point':
                generalized = [force[a] for a in range(len(indices))]
            else:
                vector = jacobian[1].T @ force
                if jacobian[2] is not None:
                    vector = vector + jacobian[2].T @ backend.Vector([forces[stride*k + 3 + a] for a in range(3)])
                generalized = [vector[a] for a in range(len(indices))]
            for (i, value) in zip(indices, generalized):
                f[i] = _Sum(f[i], value)

    def _JointEquations(self, kind, index, backend, Call, t, qs, q_ts, zComponents, c, r):
        """put the constraint equations of joint index of kind into the list c and, for zComponents, add its reaction forces
        C_q^T z to the list r, as expressions: the marker states as in _JointArguments, the recorded
        constraintUserFunction and the forces and torques C^T z of the recorded jacobianUserFunction, as
        AlgebraicEquations and ODE2ReactionForcesLHS (#3146)"""
        (ltg, joint) = (kind.ltg[index], kind.parameters[index])
        (states, jacobians) = self._MarkerStateExpressions(joint, ltg, backend, Call, t, qs, q_ts)
        arguments = (t, kind.objects[index], *_JointLeaves(states),
                     self._ParameterVector(backend, kind.objects[index], joint['parameters']))
        ltgAE = joint['ltgAE']
        equations = Call(kind.recordings['constraintUserFunction'], *arguments)
        for (k, i) in enumerate(ltgAE):
            c[i] = equations[k]
        if zComponents is None:
            return
        C = Call(kind.recordings['jacobianUserFunction'], *arguments)
        forces = C.T @ backend.Vector([zComponents[i] for i in ltgAE])
        self._AddMarkerForces(joint, ltg, backend, forces, 6, jacobians, r)

    def _ConnectorEquations(self, kind, index, backend, Call, t, qs, q_ts, f, xs):
        """add the generalized forces of connector index of kind to the list f, as expressions: the marker states as in
        _ConnectorStates, the forces of the recording, J_pos^T f + J_rot^T m of each marker"""
        (ltg, connector) = (kind.ltg[index], kind.parameters[index])
        stride = 6 if connector['orientation'] else 3
        (states, jacobians) = self._MarkerStateExpressions(connector, ltg, backend, Call, t, qs, q_ts)
        forces = Call(kind.recordings['forceUserFunction'], t, kind.objects[index], *_ConnectorLeaves(states),
                      backend.Vector([xs[i] for i in connector['ltgData']]),
                      self._ParameterVector(backend, kind.objects[index], connector['parameters']))
        self._AddMarkerForces(connector, ltg, backend, forces, stride, jacobians, f)

    def _PostNewtonEquations(self, backend, q, q_t, y, z, x, x0, t):
        """the results [error, recommendedStepSize, data] of the recorded post-Newton functions as expressions, item after
        item in the order of PostNewton (#3196)"""
        Call = _Caller(backend.format)
        n = self.nODE2
        (qs, q_ts) = ([q[i] for i in range(n)], [q_t[i] for i in range(n)])
        (ys, zs, xs, x0s) = ([y[i] for i in range(self.nODE1)], [z[i] for i in range(self.nAE)], _DataComponents(x, self),
                             _DataComponents(x0, self))
        results = []
        for kind in self.kinds:
            if kind.objectType != 'ConnectorUser' or 'postNewtonStepUserFunction' not in kind.recordings:
                continue
            for index in range(len(kind.objects)):
                (ltg, connector) = (kind.ltg[index], kind.parameters[index])
                states = self._MarkerStateExpressions(connector, ltg, backend, Call, t, qs, q_ts)[0]
                ltgData = connector['ltgData']
                value = Call(kind.recordings['postNewtonStepUserFunction'], t, kind.objects[index], *_ConnectorLeaves(states),
                             backend.Vector([xs[i] for i in ltgData]), backend.Vector([x0s[i] for i in ltgData]),
                             self._ParameterVector(backend, kind.objects[index], connector['parameters']))
                results += [value[k] for k in range(2 + len(ltgData))]
        for kind in self.daeKinds:
            if 'postNewtonStepUserFunction' not in kind.recordings:
                continue
            for (index, dae) in enumerate(kind.parameters):
                ltg = kind.ltg[index]
                value = Call(kind.recordings['postNewtonStepUserFunction'], t, kind.objects[index],
                             backend.Vector([qs[i] for i in ltg]), backend.Vector([q_ts[i] for i in ltg]),
                             backend.Vector([ys[i] for i in dae['ltgODE1']]), backend.Vector([zs[i] for i in dae['ltgAE']]),
                             backend.Vector([xs[i] for i in dae['ltgData']]), backend.Vector([x0s[i] for i in dae['ltgData']]),
                             self._ParameterVector(backend, kind.objects[index], dae['parameters']))
                results += [value[k] for k in range(2 + len(dae['ltgData']))]
        return results

    def GetFlatProgram(self, format='Python', flatten=True, results=['ODE2RHS', 'massMatrix']):
        """the flat program of results of Equations(), a function of q, q_t and t - and z with 'ODE2ReactionForcesLHS'
        or 'ODE1ReactionTermsLHS', y with 'ODE1RHS' or an ObjectGenericDAE with ODE1 coordinates (#3130, #3132, #3146,
        #3190)

        Args:
            format: 'Python' (the text), or the executable 'symbolic', 'numpy' or 'sympy' of exudyn.symbolics.codeExport
            flatten: True: one line per operation; False: each result as one expression
            results: the names of the results: 'ODE2RHS', 'massMatrix', 'algebraicEquations', 'ODE2ReactionForcesLHS',
                'ODE1RHS', 'ODE1ReactionTermsLHS', 'postNewton'

        Returns:
            the program as str, for the executable formats a function Equations(q, q_t, t) - Equations(q, q_t, t,
            z) with z, then y, x and x0 if the equations have them (#3196), followed by the named parameters (#3152) -
            returning the results
        """
        names = ['ODE2RHS', 'massMatrix', 'algebraicEquations', 'ODE2ReactionForcesLHS', 'ODE1RHS', 'ODE1ReactionTermsLHS', 'postNewton']
        unknown = [name for name in results if name not in names]
        if unknown:
            raise ValueError('SymbolicSystem.GetFlatProgram: unknown results ' + str(unknown) + '; possible: ' + str(names))
        equations = self.Equations('symbolic', **{name: name in results for name in names})
        variables = ['q', 'q_t', 't'] + [name for name in ['z', 'y', 'x', 'x0'] if name in equations]
        return exudyn.symbolic.GetFlatProgram([equations[name] for name in results],
                                              arguments=[equations[name] for name in variables] + equations.get('parameters', []),
                                              resultNames=list(results), format=format, flatten=flatten, name='Equations')

    def __repr__(self):
        lines = ['SymbolicSystem: ' + str(self.nODE2) + ' ODE2 coordinates, ' + str(len(self.kinds)) + ' kinds']
        for kind in self.kinds:
            lines.append('  ' + kind.objectType + ': ' + str(len(kind.objects)) + ' objects of ' + str(kind.ltg.shape[1])
                         + ' coordinates, user functions ' + str(sorted(kind.recordings)))
        for kind in self.jointKinds:
            lines.append('  JointUser: ' + str(len(kind.objects)) + ' joints of ' + str(len(kind.parameters[0]['ltgAE']))
                         + ' equations on ' + str(kind.ltg.shape[1]) + ' coordinates')
        for kind in self.ode1Kinds:
            lines.append('  GenericODE1: ' + str(len(kind.objects)) + ' objects of ' + str(kind.ltg.shape[1])
                         + ' ODE1 coordinates, user functions ' + str(sorted(kind.recordings)))
        for kind in self.daeKinds:
            dae = kind.parameters[0]
            lines.append('  GenericDAE: ' + str(len(kind.objects)) + ' objects of ' + str(kind.ltg.shape[1]) + ' ODE2, '
                         + str(len(dae['ltgODE1'])) + ' ODE1 and ' + str(len(dae['ltgAE'])) + ' algebraic coordinates, '
                         + 'user functions ' + str(sorted(kind.recordings)))
        if self.loads or self.ode1Loads:
            lines.append('  loads: ' + ', '.join(load.loadType + (' (ODE1)' if load.ode1 else '')
                                                + (' (user function)' if load.recording is not None else '')
                                                for load in self.loads + self.ode1Loads))
        return '\n'.join(lines)


class SystemVariables:
    """the system variables of a SymbolicSystem in one exudyn.symbolic.VariableSet (#3151): the named vectors q and q_t
    (the ODE2 coordinates and velocities, relative to the reference coordinates, as in systemData), y and y_t (the
    ODE1 coordinates and their derivatives), z (the algebraic coordinates, the Lagrange multipliers lambda of the
    documentation) and the real t, with the initial state as values; the states of markers as expressions of them - of
    the coordinates of the node for a marker on a point node, by the recordings of the user body for a marker on it,
    constants for the ground -, by the same composition as Equations(); an expression of marker states is a function
    of the system coordinates, whose Jacobian EvaluateJacobian of Function gives in one pass

    Args:
        system: the SymbolicSystem

    Note:
        Its members: system; variables, the VariableSet of q, q_t, y, y_t, z and t in this order; q, q_t, y, y_t, z, t,
        its variables. Example:

        variables = SystemVariables(system)
        distance = variables.MarkerPosition(m1) - variables.MarkerPosition(m0)
        F = variables.Function(distance)
        F.EvaluateJacobian('q', q, q_t, y, y_t, z, t) #the Jacobian of the distance by the ODE2 coordinates
    """
    def __init__(self, system):
        self.system = system
        (q0, q_t0) = system.InitialState()
        self.variables = exudyn.symbolic.VariableSet()
        self.q = self.variables.Add('q', q0)
        self.q_t = self.variables.Add('q_t', q_t0)
        self.y = self.variables.Add('y', system.InitialStateODE1())
        self.y_t = self.variables.Add('y_t', np.zeros(system.nODE1))
        self.z = self.variables.Add('z', np.zeros(system.nAE))
        self.t = self.variables.Add('t', 0.)
        self._backend = _Backend('symbolic')
        self._Call = _Caller('symbolic')
        (self._qs, self._q_ts) = ([x[i] for i in range(system.nODE2)] for x in [self.q, self.q_t])

    def _MarkerState(self, markerNumber, orientation, where):
        (marker, coordinates, reason) = self.system._MarkerData(markerNumber, orientation)
        if reason:
            raise ValueError('SystemVariables.' + where + ': marker ' + str(int(markerNumber)) + ' is not taken: ' + reason)
        return self.system._MarkerState(marker, np.array(coordinates, dtype=int), self._backend, self._Call, self.t,
                                        self._qs, self._q_ts, orientation)[0]

    def MarkerPosition(self, markerNumber):
        """the global position of a marker, an exudyn.symbolic.Vector of the system variables"""
        return self._MarkerState(markerNumber, False, 'MarkerPosition')[0]

    def MarkerVelocity(self, markerNumber):
        """the global velocity of a marker, an exudyn.symbolic.Vector of the system variables"""
        return self._MarkerState(markerNumber, False, 'MarkerVelocity')[1]

    def MarkerRotationMatrix(self, markerNumber):
        """the rotation matrix of a marker - of a rigid marker, its local frame included -, an exudyn.symbolic.Matrix of
        the system variables; the unit matrix for a marker on a point node"""
        return self._MarkerState(markerNumber, True, 'MarkerRotationMatrix')[2]

    def MarkerAngularVelocity(self, markerNumber):
        """the global angular velocity of a marker, an exudyn.symbolic.Vector of the system variables"""
        return self._MarkerState(markerNumber, True, 'MarkerAngularVelocity')[3]

    def Function(self, result, name='Function'):
        """an expression of the system variables as exudyn.symbolic.Function of variables (#3149): Evaluate(q, q_t, y,
        y_t, z, t) and EvaluateJacobian(name, q, q_t, y, y_t, z, t) of name 'q', 'q_t', 'y', 'y_t', 'z' or 't'"""
        return exudyn.symbolic.Function(result, self.variables, name)

    def Equations(self, **options):
        """Equations() of the system in these variables: q, q_t, t, z and y; options as in SymbolicSystem.Equations"""
        return self.system.Equations(variables=(self.q, self.q_t, self.t, self.z, self.y), **options)


def _Sum(a, b):
    """a + b, without the sum when a is the number 0 that a sum starts with"""
    return b if isinstance(a, int) and a == 0 else a + b


def _DataComponents(x, system):
    """the components of the data coordinates x, a variable, or the numbers of system.data if None (#3196)"""
    if x is None:
        return [float(value) for value in system.data]
    return [x[i] for i in range(system.nData)]


def _MatVec(matrix, x):
    """matrix @ x of numpy arrays, 0 for a matrix not given"""
    return matrix @ x if matrix.size else 0.


def _MatrixTimes(matrix, components, factor=1.):
    """factor*matrix times the list of components, as list of expressions without the zero entries of the matrix; None
    for a matrix not given"""
    if not matrix.size:
        return None
    return [sum(factor*float(matrix[i, j])*components[j] for j in range(matrix.shape[1]) if matrix[i, j] != 0)
            for i in range(matrix.shape[0])]


def _Caller(format):
    """the function that calls a recording with arguments of the format, by its code of exudyn.symbolics.codeExport,
    written once per recording"""
    functions = {}

    def Call(recording, *arguments):
        if id(recording) not in functions:
            functions[id(recording)] = ToFunction(recording.GetFlatProgramData(), format)
        return functions[id(recording)](None, *arguments)
    return Call


class _Backend:
    """the types of Equations: exudyn.symbolic or sympy; parameterVariables: the named parameters of the items as
    variables, else their numbers (#3152)"""
    def __init__(self, format, parameterVariables=True):
        if format not in ['symbolic', 'sympy']:
            raise ValueError("SymbolicSystem.Equations: format must be 'symbolic' or 'sympy', not " + str(format))
        self.format = format
        self.parameterVariables = parameterVariables
        if format == 'sympy':
            try:
                import sympy
            except ImportError:
                raise ImportError("SymbolicSystem.Equations(format='sympy') needs sympy: pip install sympy")
            self.sympy = sympy

    def Variable(self, name, size, value):
        if self.format == 'symbolic':
            return exudyn.symbolic.Vector(name, [float(x) for x in value], scope=exudyn.symbolic.VariableSet()) #of no model (#3208)
        return self.sympy.Matrix([self.sympy.Symbol(name + str(i)) for i in range(size)])

    def Real(self, name):
        return exudyn.symbolic.Real(name, 0., scope=exudyn.symbolic.VariableSet()) if self.format == 'symbolic' else self.sympy.Symbol(name)

    def Parameter(self, value):
        """a parameter of an item: a number, or an exudyn.symbolic.Real expression of named variables - for sympy the
        expression of the symbols of their names (#3159)"""
        if isinstance(value, exudyn.symbolic.Real) and self.format == 'sympy':
            from exudyn.symbolics.codeExport import ToSympy
            return ToSympy(exudyn.symbolic.GetFlatProgramData([value]))[1][0]
        return value

    def TransposedJacobianTimes(self, expressions, variable, z, size):
        """(d expressions/d variable)^T z as list of size expressions, the derivative by the variable vector (#3191)"""
        if self.format == 'symbolic':
            reaction = exudyn.symbolic.Jacobian(self.Vector(expressions), variable).T @ self.Vector(z)
        else:
            reaction = self.sympy.Matrix(expressions).jacobian(variable).T*self.sympy.Matrix(z)
        return [reaction[k] for k in range(size)]

    def Vector(self, components):
        components = [float(x) if isinstance(x, (np.floating, np.integer)) else x for x in components]
        return exudyn.symbolic.Vector(components) if self.format == 'symbolic' else self.sympy.Matrix(components)

    def Matrix(self, rows):
        rows = [[float(x) if isinstance(x, (np.floating, np.integer)) else x for x in row] for row in rows]
        return exudyn.symbolic.Matrix(rows) if self.format == 'symbolic' else self.sympy.Matrix(rows)

#up to this number of coordinates, the mass matrix is decomposed as dense matrix: forming a sparse one costs more (#3131)
_denseMassMatrixSize = 200
