#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  The equations of motion of a system as symbolic programs (#3110, #3113, design:
#           docs/dev/design/symbolicSystem.md): SymbolicSystemBase is the interface of the equations on which the
#           solvers work, in Exudyn's form M(q, t) q_tt = f(q, q_t, t, y) - C_q^T z, y_t = g(q, q_t, y, t),
#           c(q, q_t, y, z, t) = 0 with ODE2, ODE1 and algebraic coordinates (#3134). SymbolicSystem(mbs) extracts them, after
#           mbs.Assemble(), from the objects with symbolic user functions, collected in kinds - the objects whose user
#           functions are the same recordings -, each with the table of its objects, their parameters and the global
#           indices of their ODE2 coordinates (the local-to-global maps of systemData). It evaluates the residual
#           r = M q_tt - f, the sparse mass matrix, the forces and the Jacobian of r by exudyn.symbolic, and writes a
#           computed state back into mbs, where the renderer, the sensors and the output functions read it.
#           NumpySystem takes equations written by hand. The solvers of exudyn.symbolics.solvers integrate any of them.
#           Equations() gives the equations of the system as expressions in the system variables q, q_t and t -
#           exudyn.symbolic, whose flat program GetFlatProgram() writes, or sympy - composed of the recordings (#3130);
#           the forces and the mass matrix of these expressions, recorded once, are what ODE2RHS, MassMatrix and ODE2ResidualLHS
#           evaluate: one call each for the whole system (#3131).
#           Special: not for the broad use, a basis for the export of the equations of motion and their evaluation
#           outside Exudyn. The items it takes: ObjectUserBody with symbolic massMatrixUserFunction and
#           forceUserFunction, ObjectGenericODE2 with constant matrices and symbolic user functions,
#           ObjectConnectorUser with a symbolic forceUserFunction on position markers of point nodes, on position and
#           rigid markers of the ground and of user bodies, with the rotations if it requests them (#3121, #3129), items
#           without coordinates (ground), and the loads LoadForceVector, LoadTorqueVector,
#           LoadMassProportional and LoadCoordinate, constant or with a symbolic user function of the time, on markers
#           of point nodes, node coordinates and user bodies (#3118); ObjectJointUser with symbolic constraint and
#           Jacobian functions (#3141); ObjectGenericODE1 with its matrix, vector and a symbolic rhsUserFunction, and
#           LoadCoordinate on its coordinates (#3144); every other object and load, data coordinates and recordings
#           that read mbs are refused.
#           SystemVariables(system) gives the system variables as one exudyn.symbolic.VariableSet and the states of the
#           markers as expressions of them (#3151). The parameters of the user items may be named exudyn.symbolic.Real,
#           SymbolicSystem(mbs, parameters=...), which the expressions of Equations() keep (#3152).
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
                      'ConnectorUser': ['forceUserFunction']}
#the user functions of a joint, ObjectJointUser on two rigid markers (#3141)
_jointFunctionNames = ['constraintUserFunction', 'jacobianUserFunction']
#the positions of the arguments q (the total coordinates for UserBody) and q_t in the user functions, mbs at 0
_argumentPositions = {'UserBody': (4, 5), 'GenericODE2': (3, 4)}
#the user function of ObjectUserBody that projects a load on a body marker into its coordinates (#3118), and the
#positions of qTotal and q_t in it; the functions of a local position get it after itemNumber
_loadProjections = {'ForceVector': 'positionJacobianUserFunction', 'TorqueVector': 'rotationJacobianUserFunction',
                    'MassProportional': 'massIntegralUserFunction'}
_bodyArgumentPositions = {'positionJacobianUserFunction': (5, 6), 'rotationJacobianUserFunction': (5, 6),
                          'rotationMatrixUserFunction': (5, 6), 'massIntegralUserFunction': (4, 5)}
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
        Its members: nODE2, nODE1, nAE, the numbers of coordinates
    """
    nODE2 = 0
    nODE1 = 0
    nAE = 0

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
        Its members: objectType, the type name 'UserBody', 'GenericODE2', 'ConnectorUser', 'JointUser' or
        'GenericODE1'; recordings, the dict of the recorded user functions by name (a name without recording is
        missing); objects, the list of the object numbers; ltg, the array (number of objects x local coordinates) of the
        global ODE2 indices - of the global ODE1 indices for GenericODE1 -; parameters, the list of the
        parameter vectors of the objects (UserBody), of the dicts of the constant matrices and the force vector
        (GenericODE2), or of the dicts of the parameter vector, 'orientation' (the connector gets the rotations and
        returns the torques) and the markers (ConnectorUser: per marker a dict of its 'type' - 'fixed', 'point' or
        'body' -, its local coordinate indices 'indices', its reference 'position' and 'rotation' - for a marker on a
        user body its local position and rotation, 'objectNumber', 'qRef', 'parameters' and the 'recordings' of the
        body that give its kinematics), of the dicts of 'parameters', 'markers', 'orientation' and the global indices
        'ltgAE' of its algebraic coordinates (JointUser), or of the dicts of the system matrix 'A' and the vector 'f'
        (GenericODE1)
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
        parameters: None, or the dict of the parameters of user items - ObjectUserBody, ObjectConnectorUser,
            ObjectJointUser - by item number, each a list of numbers and named exudyn.symbolic.Real, the values those of
            the item: UserItemCreator.parameterVariables; Equations() keeps the variables, a variable of several items is
            one variable (#3152)
        itemJacobians: how the Jacobians of the connectors and joints are computed (#3166): 'chainRule', the derivatives
            of their recordings by the marker states composed with the marker Jacobians; 'onePass', the generalized
            forces of a connector - the constraint equations and reaction forces of a joint - as one expression of the
            coordinates of its bodies and nodes, differentiated in one pass per body or node

    Note:
        Its members: mbs; kinds, the list of SymbolicKind; jointKinds, the list of SymbolicKind of the ObjectJointUser
        (#3141), whose constraint equations are AlgebraicEquations and whose reaction forces are ODE2ReactionForcesLHS - in Equations() on request
        (#3146) -; ode1Kinds, the list of SymbolicKind of the ObjectGenericODE1 (#3144), whose right-hand
        side is ODE1RHS; loads, the list of SymbolicLoad on ODE2 coordinates, ode1Loads those on ODE1
        coordinates; nODE2, nODE1, the numbers of ODE2 and ODE1 coordinates of the system; nAE, the number of the
        algebraic equations; qRef, the reference values of the ODE2 coordinates; composed, None or the dict of
        the recordings 'ODE2RHS', 'massMatrix' and 'massMatrixBlocks' - esym.Function of the inputs q, q_t and t, the
        VariableSet 'inputs' (#3149) - and the 'rows' and 'columns' of the entries of the mass matrix; parameterVariables,
        the dict of parameters, checked
    """
    def __init__(self, mbs, composeSystem=True, parameters=None, itemJacobians='chainRule'):
        if not mbs.systemIsConsistent:
            raise ValueError('SymbolicSystem: mbs must be assembled first')
        if itemJacobians not in ['chainRule', 'onePass']:
            raise ValueError("SymbolicSystem: itemJacobians must be 'chainRule' or 'onePass', not " + str(itemJacobians))
        self.mbs = mbs
        self.itemJacobians = itemJacobians
        self._itemFunctions = {} #the functions of 'onePass' by (kind, index), built at their first use
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
        if mbs.systemData.DataSize() > 0:
            refused += ['data coordinates']
        self.nODE1 = mbs.systemData.ODE1Size()
        self.nAE = mbs.systemData.AEsize()
        kinds = {}
        ode1Kinds = {}
        jointKinds = {}
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
                                        'orientation': bool(int(item['requestedMarkerType']) & int(exudyn.MarkerType.Orientation))})
            else:
                kind.parameters.append({'M': _Dense(item['massMatrix']), 'K': _Dense(item['stiffnessMatrix']),
                                        'D': _Dense(item['dampingMatrix']), 'f': np.array(item['forceVector'], dtype=float)})
        self.jointKinds = list(jointKinds.values())
        self.ode1Kinds = list(ode1Kinds.values())
        if not refused and sum(len(joint['ltgAE']) for kind in self.jointKinds for joint in kind.parameters) != self.nAE:
            refused += ['algebraic equations of other items than ObjectJointUser']
        if not refused and sum(len(ltg) for kind in self.ode1Kinds for ltg in kind.ltg) != self.nODE1:
            refused += ['ODE1 coordinates of other items than ObjectGenericODE1']
        if refused:
            raise ValueError('SymbolicSystem: these items cannot be assembled (yet): ' + ', '.join(refused))
        self.kinds = list(kinds.values())
        for kind in self.kinds + self.jointKinds + self.ode1Kinds:
            kind.ltg = np.array(kind.ltg, dtype=int)
        self.parameterVariables = self._ParameterVariables(parameters)
        self.composed = self._Compose() if composeSystem else None

    def _ParameterVariables(self, parameters):
        """the parameters of the items given as numbers and named exudyn.symbolic.Real, checked: the number of the
        parameters of the item, the values those of the item, one variable per name (#3152)"""
        (variables, self._parameterNames) = ({}, {})
        for (objectNumber, values) in (parameters or {}).items():
            numbers = list(self.mbs.GetObject(int(objectNumber))['parameters'])
            where = 'SymbolicSystem: the parameters of object ' + str(int(objectNumber))
            if len(values) != len(numbers):
                raise ValueError(where + ' are ' + str(len(numbers)) + ' numbers, not ' + str(len(values)))
            for (value, number) in zip(values, numbers):
                if isinstance(value, exudyn.symbolic.Real):
                    name = str(value)
                    if not name.isidentifier():
                        raise ValueError(where + ': ' + name + ' is no named exudyn.symbolic.Real')
                    if self._parameterNames.setdefault(name, value) is not value:
                        raise ValueError(where + ': two different variables are named ' + name)
                    value = value.Evaluate()
                if float(value) != float(number):
                    raise ValueError(where + ': the value ' + str(value) + ' differs from the number ' + str(number)
                                     + ' of the item; SetParameterValues sets both')
            variables[int(objectNumber)] = list(values)
        return variables

    def SetParameterValues(self, values):
        """set named parameters (#3152): the values of the variables, the numbers of the items in mbs, then the system
        again from mbs

        Args:
            values: dict of the names of variables of parameterVariables and their new values
        """
        unknown = [name for name in values if name not in self._parameterNames]
        if unknown:
            raise ValueError('SymbolicSystem.SetParameterValues: no parameter named ' + str(unknown) + '; the parameters: '
                             + str(list(self._parameterNames)))
        for (name, value) in values.items():
            self._parameterNames[name].SetValue(float(value))
        for (objectNumber, parameters) in self.parameterVariables.items():
            self.mbs.SetObjectParameter(objectNumber, 'parameters', [float(value.Evaluate()) if isinstance(value, exudyn.symbolic.Real)
                                                                     else value for value in parameters])
        self.__init__(self.mbs, self.composed is not None, self.parameterVariables, self.itemJacobians)

    def _ParameterVector(self, backend, objectNumber, numbers):
        """the parameters of an item for Equations: its numbers, or with named parameters its list of variables and
        numbers, if the backend takes them (#3152)"""
        variables = self.parameterVariables.get(int(objectNumber)) if backend.parameterVariables else None
        if variables is None:
            return backend.Vector(numbers)
        return backend.Vector([backend.Parameter(value) for value in variables])

    def _Compose(self):
        """the recordings of the composed system (#3131): the forces f(q, q_t, t) and the entries of the mass matrix at
        the rows and columns of the local mass matrices, as esym.Function of the inputs q, q_t and t (#3149) of the
        expressions of Equations, optimized as CreateSymbolicUserFunction does; one call of each evaluates the system"""
        entries = set()
        for kind in self.kinds:
            if kind.objectType != 'ConnectorUser':
                for ltg in kind.ltg:
                    entries.update((int(i), int(j)) for i in ltg for j in ltg)
        entries = sorted(entries)
        rows = np.array([i for (i, j) in entries], dtype=int)
        columns = np.array([j for (i, j) in entries], dtype=int)
        backend = _Backend('symbolic', parameterVariables=False) #the numbers of the items; SetParameterValues composes again
        (q0, q_t0) = self.InitialState()
        inputs = exudyn.symbolic.VariableSet() #the scope of the inputs, in the order of the arguments of Evaluate
        (q, q_t, t) = (inputs.Add('q', q0), inputs.Add('q_t', q_t0), inputs.Add('t', 0.))
        (M, f, blocks) = self._EquationLists(backend, q, q_t, t)[:3]
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
                                'ltgAE': np.array(self.mbs.systemData.GetObjectLTGAE(objectNumber), dtype=int)})
        return ''

    def _ConnectorMarkers(self, objectNumber, item, orientation=None):
        """the markers of an ObjectConnectorUser - or of an ObjectJointUser, orientation=True - as the dicts of
        SymbolicKind, the global indices of the coordinates of the connector, and the reason it is refused, empty if it
        is taken"""
        if orientation is None:
            if item['markerAccessUserFunction'] != 0 or item['postNewtonStepUserFunction'] != 0:
                return (None, None, 'a markerAccessUserFunction or postNewtonStepUserFunction')
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

    def InitialStateODE1(self):
        """the initial ODE1 coordinates y0 of mbs"""
        return np.array(self.mbs.systemData.GetODE1Coordinates(configuration=exudyn.ConfigurationType.Initial))

    def IsMassMatrixConstant(self):
        """True if every user body has constantMassMatrix and no ObjectGenericODE2 has a massMatrixUserFunction"""
        return self._constantMassMatrix

    def ODE1RHS(self, q, q_t, y, t):
        """the right-hand side g(y, t) of the ODE1 equations y_t = g (#3144): per ObjectGenericODE1 A y + f and its
        recorded rhsUserFunction of its coordinates, as CObjectGenericODE1 computes it, and the LoadCoordinate on ODE1
        coordinates

        Args:
            q: the ODE2 coordinates, not used
            q_t: the ODE2 velocities, not used
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
        for load in self.ode1Loads:
            np.add.at(g, load.ltg, np.atleast_1d(self._LoadValue(load, t)))
        return g

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

    def ODE1Jacobian(self, y, t):
        """the derivative of ODE1RHS by y (nODE1 x nODE1), exact: A and the derivative of the recorded
        rhsUserFunction (#3144, #3116)"""
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
        """the arguments after mbs of the forceUserFunction of connector i of kind - the position, velocity, rotation
        matrix and angular velocity of each marker, unit rotations and no angular velocities if it requests no
        orientation -, and per marker its position and rotation Jacobians (None for a fixed marker) and the arguments
        of the user functions of its body (None for a point node or a fixed marker)"""
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
        return ((t, kind.objects[i], *states, [], connector['parameters']), markerData)

    def _JointArguments(self, kind, i, q, t):
        """the arguments after mbs of the user functions of joint i of kind - the position and rotation matrix of each
        marker -, and per marker its Jacobians and the arguments of its body, as _ConnectorStates"""
        (arguments, markerData) = self._ConnectorStates(kind, i, q, np.zeros(self.nODE2), t)
        (t, objectNumber, p0, v0, R0, w0, p1, v1, R1, w1, data, parameters) = arguments
        return ((t, objectNumber, p0, R0, p1, R1, parameters), markerData)

    def AlgebraicEquations(self, q, q_t, y, z, t, velocityLevel=False):
        """the constraint equations c(q, t) of the joints at position level (#3141), by their recorded
        constraintUserFunction; the velocity level is not given

        Args:
            q: the ODE2 coordinates
            q_t: the ODE2 velocities, not used
            y: the ODE1 coordinates, not used
            z: the algebraic coordinates, the Lagrange multipliers lambda of the documentation, not used
            t: the time
            velocityLevel: must be False

        Returns:
            the vector c of size nAE
        """
        if velocityLevel:
            raise ValueError('SymbolicSystem.AlgebraicEquations: the joints give their equations at position level only')
        c = np.zeros(self.nAE)
        for kind in self.jointKinds:
            for i in range(len(kind.objects)):
                arguments = self._JointArguments(kind, i, np.asarray(q, dtype=float), t)[0]
                c[kind.parameters[i]['ltgAE']] = np.array(kind.recordings['constraintUserFunction'].Evaluate(self.mbs, *arguments),
                                                          dtype=float)
        return c

    def ODE2ReactionForcesLHS(self, q, q_t, y, z, t):
        """the reaction forces C_q^T z of the joints (#3141): the recorded jacobianUserFunction C by the marker
        translations and global rotation increments, C^T z the forces and torques on the markers, projected into the
        coordinates by the position and rotation Jacobians of the markers, as the forces of a connector

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
        return f

    def _JointJacobian(self, kind, i, q, z, t):
        """the derivatives of joint i of kind in its local coordinates (#3145): C_q, the derivative of its constraint
        equations by the coordinates - the recorded jacobianUserFunction C by the marker translations and global
        rotation increments times the position and rotation Jacobians of the markers -, and the derivative of its
        reaction forces C_q^T z by the coordinates: the derivatives of C by the positions and rotation matrices of
        the markers (arguments 3 + 2k and 4 + 2k of the recording), composed with J_pos and dA/dq of the bodies, and the
        derivatives of J_pos and J_rot of the bodies times the forces and torques C^T z"""
        n = kind.ltg.shape[1]
        joint = kind.parameters[i]
        (arguments, markerData) = self._JointArguments(kind, i, q, t)
        recording = kind.recordings['jacobianUserFunction']
        C = np.array(recording.Evaluate(self.mbs, *arguments), dtype=float).reshape(-1, 12)
        zJoint = z[joint['ltgAE']]
        forces = C.T @ zJoint #the forces and torques on the markers
        C_q = np.zeros((C.shape[0], n))
        dForces = np.zeros((12, n)) #the derivative of C^T z by the local coordinates
        for (k, (marker, (Jpos, Jrot, bodyArguments))) in enumerate(zip(joint['markers'], markerData)):
            if Jpos is None:
                continue
            indices = marker['indices']
            C_q[:, indices] += C[:, 6*k:6*k + 3] @ Jpos
            dForces[:, indices] += np.einsum('ijk,i->jk', self._RecordingJacobian(recording, 3 + 2*k, arguments).reshape(-1, 12, 3),
                                             zJoint) @ Jpos
            if Jrot is not None:
                C_q[:, indices] += C[:, 6*k + 3:6*k + 6] @ Jrot
            if bodyArguments is not None and Jrot is not None: #the rotation of the marker after the one of the body
                dA = np.einsum('ijk,jl->ilk', self._BodyJacobian(marker, 'rotationMatrixUserFunction', bodyArguments),
                               marker['rotation'])
                dForces[:, indices] += np.einsum('ijk,i->jk', self._RecordingJacobian(recording, 4 + 2*k, arguments).reshape(-1, 12, 9),
                                                 zJoint) @ dA.reshape(9, len(indices))
        dReaction = np.zeros((n, n))
        for (k, (marker, (Jpos, Jrot, bodyArguments))) in enumerate(zip(joint['markers'], markerData)):
            if Jpos is None:
                continue
            indices = marker['indices']
            (force, torque) = (slice(6*k, 6*k + 3), slice(6*k + 3, 6*k + 6))
            dReaction[indices, :] += Jpos.T @ dForces[force]
            if Jrot is not None:
                dReaction[indices, :] += Jrot.T @ dForces[torque]
            if bodyArguments is not None:
                dJpos = self._BodyJacobian(marker, 'positionJacobianUserFunction', bodyArguments)
                dReaction[np.ix_(indices, indices)] += np.einsum('ijk,i->jk', dJpos, forces[force])
                if Jrot is not None:
                    dJrot = self._BodyJacobian(marker, 'rotationJacobianUserFunction', bodyArguments)
                    dReaction[np.ix_(indices, indices)] += np.einsum('ijk,i->jk', dJrot, forces[torque])
        return (C_q, dReaction)

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
        J = np.zeros((nSystem, nSystem))
        J[:nODE2, :nODE2] = self.ODE2ResidualLHSJacobian(q, q_t, q_tt, t, factor_q, factor_q_t, factor_q_tt)
        if nAE:
            z = np.asarray(z, dtype=float)
            offset = nODE2 + nODE1 #the first algebraic coordinate
            for kind in self.jointKinds:
                for i in range(len(kind.objects)):
                    (ltg, ltgAE) = (kind.ltg[i], kind.parameters[i]['ltgAE'] + offset)
                    if self.itemJacobians == 'onePass':
                        (C_q, dReaction) = self._JointJacobianOnePass(kind, i, q, z, t)
                    else:
                        (C_q, dReaction) = self._JointJacobian(kind, i, q, z, t)
                    J[np.ix_(ltg, ltg)] += factor_q*dReaction
                    J[np.ix_(ltg, ltgAE)] += C_q.T
                    J[np.ix_(ltgAE, ltg)] += factor_q*C_q
        if nODE1: #the rows g - y_t
            J[nODE2:nODE2 + nODE1, nODE2:nODE2 + nODE1] = factor_y*self.ODE1Jacobian(y, t) - factor_y_t*np.eye(nODE1)
        return J

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
        """the global forces f(q, q_t, t) of the right-hand side, as the ODE2 part of the solver's right-hand side; y is
        not used, as no item taken couples the ODE1 coordinates into the ODE2 equations

        Args:
            q: the ODE2 coordinates (displacements), of size nODE2; None: zero
            q_t: the ODE2 velocities; None: zero
            t: the time
            y: the ODE1 coordinates, not used

        Returns:
            the vector f of size nODE2
        """
        (q, q_t, _) = self._State(q, q_t)
        if self.composed is not None:
            return np.array(self.composed['ODE2RHS'].Evaluate(q, q_t, t), dtype=float)
        f = np.zeros(self.nODE2)
        for kind in self.kinds:
            for i in range(len(kind.objects)):
                np.add.at(f, kind.ltg[i], self._LocalForces(kind, i, q, q_t, t))
        for load in self.loads:
            np.add.at(f, load.ltg, self._LoadForces(load, q, q_t, t))
        return f

    def MassMatrixTriplets(self, q=None, q_t=None, t=0.):
        """the global mass matrix as triplets (rows, columns, values), a value per nonzero local entry; equal rows
        and columns are added when the matrix is formed"""
        (q, q_t, _) = self._State(q, q_t)
        if self.composed is not None:
            values = np.array(self.composed['massMatrix'].Evaluate(q, q_t, t), dtype=float)
            return self.composed['rows'], self.composed['columns'], values[:len(self.composed['rows'])]
        rows, columns, values = [], [], []
        for kind in self.kinds:
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
        kinds = [kind for kind in self.kinds if kind.objectType != 'ConnectorUser']
        if self.composed is not None: #one call: the entries of all blocks, kind after kind
            values = np.array(self.composed['massMatrixBlocks'].Evaluate(q, q_t, t), dtype=float)
            (result, start) = ([], 0)
            for kind in kinds:
                (m, nLocal) = kind.ltg.shape
                result.append((kind.ltg, values[start:start + m*nLocal*nLocal].reshape(m, nLocal, nLocal)))
                start += m*nLocal*nLocal
            return result
        return [(kind.ltg, np.array([self._LocalMassMatrix(kind, i, q, q_t, t) for i in range(len(kind.objects))]))
                for kind in kinds]

    def ODE2ResidualLHS(self, q=None, q_t=None, q_tt=None, t=0., y=None):
        """the residual r = M(q) q_tt - f(q, q_t, t) of the ODE2 equations, LHS minus RHS, without forming M: by the recordings of the
        composed system, else object by object; y is not used, as no item taken couples the ODE1 coordinates into the
        ODE2 equations"""
        (q, q_t, q_tt) = self._State(q, q_t, q_tt)
        if self.composed is not None:
            (rows, columns, values) = self.MassMatrixTriplets(q, q_t, t)
            return np.bincount(rows, values*q_tt[columns], minlength=self.nODE2) - self.ODE2RHS(q, q_t, t)
        r = np.zeros(self.nODE2)
        for kind in self.kinds:
            for i in range(len(kind.objects)):
                ltg = kind.ltg[i]
                local = self._LocalMassMatrix(kind, i, q, q_t, t) @ q_tt[ltg] - self._LocalForces(kind, i, q, q_t, t)
                np.add.at(r, ltg, local)
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
            y: the ODE1 coordinates, not used

        Returns:
            the matrix (nODE2 x nODE2)
        """
        (q, q_t, q_tt) = self._State(q, q_t, q_tt)
        if self.composed is not None and self.nODE2 <= _denseMassMatrixSize: #r = M q_tt - f of the two recordings
            composed = self.composed
            J = np.zeros((self.nODE2, self.nODE2))
            for (name, factor) in [('q', factor_q), ('q_t', factor_q_t)]:
                if factor != 0.:
                    J -= factor*np.array(composed['ODE2RHS'].EvaluateJacobian(name, q, q_t, t), dtype=float)
            (rows, columns) = (composed['rows'], composed['columns'])
            if factor_q != 0. and len(rows) and not self._constantMassMatrix: #d(M q_tt)/dq
                dM = np.array(composed['massMatrix'].EvaluateJacobian('q', q, q_t, t), dtype=float)[:len(rows)]
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
                    if self.itemJacobians == 'onePass':
                        local = self._ConnectorJacobianOnePass(kind, i, q, q_t, t, factor_q, factor_q_t)
                    else:
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
        for load in self.loads:
            local = self._LoadJacobian(load, q, q_t, t, factor_q, factor_q_t)
            (r, c) = np.nonzero(local)
            rows.append(load.ltg[r])
            columns.append(load.ltg[c])
            values.append(local[r, c])
        if not rows:
            return self._Matrix(np.zeros(0, dtype=int), np.zeros(0, dtype=int), np.zeros(0), sparse)
        return self._Matrix(np.concatenate(rows), np.concatenate(columns), np.concatenate(values), sparse)

    def _ConnectorJacobian(self, kind, i, q, q_t, t, factor_q, factor_q_t):
        """the local Jacobian of the residual -f of a connector, -d(sum_k J_pos,k^T f_k + J_rot,k^T m_k): the derivatives
        of its forces by the states of its markers (arguments 3 + 4k to 6 + 4k) composed with the derivatives of the
        states by the coordinates - J_pos, dJ_pos/dq q_t, dA/dq, J_rot, dJ_rot/dq q_t, by Diff of the recordings of the
        bodies -, and the derivatives of the Jacobians of the bodies times the forces (#3129)"""
        n = kind.ltg.shape[1]
        connector = kind.parameters[i]
        orientation = connector['orientation']
        stride = 6 if orientation else 3
        (arguments, markerData) = self._ConnectorStates(kind, i, q, q_t, t)
        recording = kind.recordings['forceUserFunction']
        forces = np.array(recording.Evaluate(self.mbs, *arguments), dtype=float)
        nForces = len(forces)
        (dF_q, dF_q_t) = (np.zeros((nForces, n)), np.zeros((nForces, n)))
        bodyDerivatives = [] #per marker on a user body: dJ_pos/dq, dJ_rot/dq
        for (k, (marker, (Jpos, Jrot, bodyArguments))) in enumerate(zip(connector['markers'], markerData)):
            indices = marker['indices']
            if Jpos is None:
                bodyDerivatives.append(None)
                continue
            (dJpos, dJrot) = (None, None)
            dF_dv = self._RecordingJacobian(recording, 4 + 4*k, arguments)
            if factor_q != 0.: #by the position, and the velocity by q through J_pos
                dF_q[:, indices] += self._RecordingJacobian(recording, 3 + 4*k, arguments) @ Jpos
            if factor_q_t != 0.:
                dF_q_t[:, indices] += dF_dv @ Jpos
            if bodyArguments is not None and factor_q != 0.:
                dJpos = self._BodyJacobian(marker, 'positionJacobianUserFunction', bodyArguments)
                dF_q[:, indices] += dF_dv @ np.einsum('ijk,j->ik', dJpos, bodyArguments[5])
            if orientation and Jrot is not None:
                dF_domega = self._RecordingJacobian(recording, 6 + 4*k, arguments)
                if factor_q_t != 0.:
                    dF_q_t[:, indices] += dF_domega @ Jrot
                if factor_q != 0.:
                    dJrot = self._BodyJacobian(marker, 'rotationJacobianUserFunction', bodyArguments)
                    dA = np.einsum('ijk,jl->ilk', self._BodyJacobian(marker, 'rotationMatrixUserFunction', bodyArguments),
                                   marker['rotation'])
                    dF_q[:, indices] += (self._RecordingJacobian(recording, 5 + 4*k, arguments) @ dA.reshape(9, len(indices))
                                         + dF_domega @ np.einsum('ijk,j->ik', dJrot, bodyArguments[5]))
            bodyDerivatives.append((dJpos, dJrot))
        local = np.zeros((n, n))
        for (k, (marker, (Jpos, Jrot, bodyArguments))) in enumerate(zip(connector['markers'], markerData)):
            if Jpos is None:
                continue
            indices = marker['indices']
            (force, torque) = (slice(stride*k, stride*k + 3), slice(stride*k + 3, stride*k + 6))
            for (dF, factor) in [(dF_q, factor_q), (dF_q_t, factor_q_t)]:
                if factor != 0.:
                    local[indices, :] -= factor*(Jpos.T @ dF[force] + (Jrot.T @ dF[torque] if Jrot is not None else 0.))
            if factor_q != 0. and bodyDerivatives[k] is not None:
                (dJpos, dJrot) = bodyDerivatives[k]
                if dJpos is not None:
                    local[np.ix_(indices, indices)] -= factor_q*np.einsum('ijk,i->jk', dJpos, forces[force])
                if dJrot is not None:
                    local[np.ix_(indices, indices)] -= factor_q*np.einsum('ijk,i->jk', dJrot, forces[torque])
        return local

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
        t = variables.Add('t', 0.)
        (backend, Call, local) = (_Backend('symbolic', parameterVariables=False), _Caller('symbolic'), np.arange(n))
        (states, jacobians) = self._MarkerStateExpressions(item, local, backend, Call, t, qs, q_ts)
        parameters = self._ParameterVector(backend, objectNumber, item['parameters'])
        f = [0]*n
        if joint:
            arguments = (t, objectNumber, states[0], states[2], states[4], states[6], parameters)
            c = Call(kind.recordings['constraintUserFunction'], *arguments)
            C = Call(kind.recordings['jacobianUserFunction'], *arguments)
            self._AddMarkerForces(item, local, backend, C.T @ backend.Vector([z[k] for k in range(nConstraints)]), 6, jacobians, f)
            result = backend.Vector([c[k] for k in range(nConstraints)] + f)
        else:
            forces = Call(kind.recordings['forceUserFunction'], t, objectNumber, *states, backend.Vector([]), parameters)
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
        values.append(float(t))
        derivatives = []
        for (g, group) in enumerate(groups):
            d_q = np.array(function.EvaluateJacobian('q' + str(g), *values), dtype=float).reshape(-1, len(group))
            d_q_t = None
            if velocities:
                d_q_t = np.array(function.EvaluateJacobian('q_t' + str(g), *values), dtype=float).reshape(-1, len(group))
            derivatives.append((list(group), d_q, d_q_t))
        return derivatives

    def _ConnectorJacobianOnePass(self, kind, i, q, q_t, t, factor_q, factor_q_t):
        """the local Jacobian of the residual -f of a connector, as _ConnectorJacobian, by the one-pass derivative of its
        generalized forces (#3166)"""
        n = kind.ltg.shape[1]
        local = np.zeros((n, n))
        for (columns, d_q, d_q_t) in self._ItemJacobians(kind, i, q, q_t, None, t, False, factor_q_t != 0.):
            if factor_q != 0.:
                local[:, columns] -= factor_q*d_q
            if factor_q_t != 0.:
                local[:, columns] -= factor_q_t*d_q_t
        return local

    def _JointJacobianOnePass(self, kind, i, q, z, t):
        """C_q and the derivative of the reaction forces C_q^T z by the coordinates of joint i of kind, as _JointJacobian,
        by the one-pass derivative of its constraint equations and reaction forces (#3166)"""
        n = kind.ltg.shape[1]
        nConstraints = len(kind.parameters[i]['ltgAE'])
        (C_q, dReaction) = (np.zeros((nConstraints, n)), np.zeros((n, n)))
        for (columns, d_q, d_q_t) in self._ItemJacobians(kind, i, q, np.zeros(self.nODE2), z, t, True, False):
            C_q[:, columns] += d_q[:nConstraints]
            dReaction[:, columns] += d_q[nConstraints:]
        return (C_q, dReaction)

    def Equations(self, format='symbolic', optimize=True, variables=None, massMatrix=True, ODE2RHS=True,
                  massMatrixBlocks=False, algebraicEquations=False, ODE2ReactionForcesLHS=False, ODE1RHS=False):
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
            ODE1RHS: True: the right-hand side g(y, t) of the ODE1 equations y_t = g, 'ODE1RHS' (nODE1), as ODE1RHS, of
                the ODE1 coordinates, the variable 'y' with the initial ODE1 state (#3162)

        Returns:
            dict with 'q', 'q_t', 't' (the variables), 'z' if ODE2ReactionForcesLHS, 'y' if ODE1RHS, 'parameters' - the named parameters
            of parameterVariables, as exudyn.symbolic.Real or sympy symbols - if there are any (#3152), and the results
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
        if ODE2ReactionForcesLHS and z is None:
            z = backend.Variable('z', self.nAE, np.zeros(self.nAE))
        y = variables[4] if variables is not None and len(variables) > 4 else None
        if ODE1RHS and y is None:
            y = backend.Variable('y', self.nODE1, self.InitialStateODE1())
        (M, f, blocks, c, r) = self._EquationLists(backend, q, q_t, t, z if ODE2ReactionForcesLHS else None,
                                                   algebraicEquations or ODE2ReactionForcesLHS)
        (names, results) = ([], [])
        for (name, wanted, Result) in [('massMatrix', massMatrix, lambda: backend.Matrix(M)), ('ODE2RHS', ODE2RHS, lambda: backend.Vector(f)),
                                       ('algebraicEquations', algebraicEquations, lambda: backend.Vector(c)),
                                       ('ODE2ReactionForcesLHS', ODE2ReactionForcesLHS, lambda: backend.Vector(r)),
                                       ('ODE1RHS', ODE1RHS, lambda: backend.Vector(self._ODE1Equations(backend, y, t)))]:
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
        if ODE2ReactionForcesLHS:
            equations['z'] = z
        if ODE1RHS:
            equations['y'] = y
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

    def _EquationLists(self, backend, q, q_t, t, z=None, constraints=False):
        """the mass matrix as list of rows, the forces as list of the expressions of Equations, of the variables q, q_t
        and t of backend, the local mass matrices of the bodies, per kind (ltg, list of their lists of rows), and with
        constraints the constraint equations of the joints and - for the algebraic coordinates z - their reaction
        forces as lists (#3146)"""
        n = self.nODE2
        format = backend.format
        (qs, q_ts) = ([x[i] for i in range(n)] for x in [q, q_t]) #the components, to be gathered per item
        M = [[0]*n for i in range(n)]
        f = [0]*n
        blocks = [(kind.ltg, []) for kind in self.kinds if kind.objectType != 'ConnectorUser']
        blocksOfKind = {id(kind): blocks[k][1] for (k, kind) in enumerate(kind for kind in self.kinds if kind.objectType != 'ConnectorUser')}
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
                    self._ConnectorEquations(kind, index, backend, Call, t, qs, q_ts, f)
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
        if constraints:
            zComponents = None if z is None else [z[i] for i in range(self.nAE)]
            for kind in self.jointKinds:
                for index in range(len(kind.objects)):
                    self._JointEquations(kind, index, backend, Call, t, qs, q_ts, zComponents, c, r)
        return (M, f, blocks, c, r)

    def _ODE1Equations(self, backend, y, t):
        """the right-hand side of the ODE1 equations as list of expressions of the ODE1 coordinates y, as ODE1RHS: per
        ObjectGenericODE1 A y + f and its recorded rhsUserFunction, and the LoadCoordinate on ODE1 coordinates (#3162)"""
        Call = _Caller(backend.format)
        ys = [y[i] for i in range(self.nODE1)]
        g = [0]*self.nODE1
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
        arguments = (t, kind.objects[index], states[0], states[2], states[4], states[6],
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

    def _ConnectorEquations(self, kind, index, backend, Call, t, qs, q_ts, f):
        """add the generalized forces of connector index of kind to the list f, as expressions: the marker states as in
        _ConnectorStates, the forces of the recording, J_pos^T f + J_rot^T m of each marker"""
        (ltg, connector) = (kind.ltg[index], kind.parameters[index])
        stride = 6 if connector['orientation'] else 3
        (states, jacobians) = self._MarkerStateExpressions(connector, ltg, backend, Call, t, qs, q_ts)
        forces = Call(kind.recordings['forceUserFunction'], t, kind.objects[index], *states, backend.Vector([]),
                      self._ParameterVector(backend, kind.objects[index], connector['parameters']))
        self._AddMarkerForces(connector, ltg, backend, forces, stride, jacobians, f)

    def GetFlatProgram(self, format='Python', flatten=True, results=['ODE2RHS', 'massMatrix']):
        """the flat program of results of Equations(), a function of q, q_t and t - and z with 'ODE2ReactionForcesLHS'
        (#3130, #3132, #3146)

        Args:
            format: 'Python' (the text), or the executable 'symbolic', 'numpy' or 'sympy' of exudyn.symbolics.codeExport
            flatten: True: one line per operation; False: each result as one expression
            results: the names of the results: 'ODE2RHS', 'massMatrix', 'algebraicEquations', 'ODE2ReactionForcesLHS',
                'ODE1RHS'

        Returns:
            the program as str, for the executable formats a function Equations(q, q_t, t) - Equations(q, q_t, t,
            z) with 'ODE2ReactionForcesLHS', y after them with 'ODE1RHS', followed by the named parameters (#3152) -
            returning the results
        """
        names = ['ODE2RHS', 'massMatrix', 'algebraicEquations', 'ODE2ReactionForcesLHS', 'ODE1RHS']
        unknown = [name for name in results if name not in names]
        if unknown:
            raise ValueError('SymbolicSystem.GetFlatProgram: unknown results ' + str(unknown) + '; possible: ' + str(names))
        equations = self.Equations('symbolic', **{name: name in results for name in names})
        variables = (['q', 'q_t', 't'] + (['z'] if 'ODE2ReactionForcesLHS' in results else [])
                     + (['y'] if 'ODE1RHS' in results else []))
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
            return exudyn.symbolic.Vector(name, [float(x) for x in value])
        return self.sympy.Matrix([self.sympy.Symbol(name + str(i)) for i in range(size)])

    def Real(self, name):
        return exudyn.symbolic.Real(name, 0.) if self.format == 'symbolic' else self.sympy.Symbol(name)

    def Parameter(self, value):
        """a parameter of an item: a number, or a named exudyn.symbolic.Real - for sympy the symbol of its name"""
        if isinstance(value, exudyn.symbolic.Real) and self.format == 'sympy':
            return self.sympy.Symbol(str(value))
        return value

    def Vector(self, components):
        components = [float(x) if isinstance(x, (np.floating, np.integer)) else x for x in components]
        return exudyn.symbolic.Vector(components) if self.format == 'symbolic' else self.sympy.Matrix(components)

    def Matrix(self, rows):
        rows = [[float(x) if isinstance(x, (np.floating, np.integer)) else x for x in row] for row in rows]
        return exudyn.symbolic.Matrix(rows) if self.format == 'symbolic' else self.sympy.Matrix(rows)

#up to this number of coordinates, the mass matrix is decomposed as dense matrix: forming a sparse one costs more (#3131)
_denseMassMatrixSize = 200
