#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  The equations of motion of a system as symbolic programs (#3110, #3113, design:
#           docs/dev/design/symbolicSystem.md): SymbolicSystemBase is the interface of the equations on which the
#           solvers work, in Exudyn's form M(q, t) q_tt = f(q, q_t, t, y) - C_q^T lambda, y_t = g(q, q_t, y, t),
#           c(q, q_t, y, lambda, t) = 0 with ODE2, ODE1 and algebraic coordinates (#3134). SymbolicSystem(mbs) extracts them, after
#           mbs.Assemble(), from the objects with symbolic user functions, collected in kinds - the objects whose user
#           functions are the same recordings -, each with the table of its objects, their parameters and the global
#           indices of their ODE2 coordinates (the local-to-global maps of systemData). It evaluates the residual
#           r = M q_tt - f, the sparse mass matrix, the forces and the Jacobian of r by exudyn.symbolic, and writes a
#           computed state back into mbs, where the renderer, the sensors and the output functions read it.
#           NumpySystem takes equations written by hand. SolveExplicit integrates any of them by explicit Euler, RK44
#           or velocity Verlet, as Exudyn's explicit solver does (#3112), SolveGeneralizedAlpha by the generalized-alpha
#           method, as Exudyn's implicit solver does (#3126).
#           Equations() gives the equations of the system as expressions in the system variables q, q_t and t -
#           exudyn.symbolic, whose flat program GetFlatProgram() writes, or sympy - composed of the recordings (#3130);
#           the forces and the mass matrix of these expressions, recorded once, are what Forces, MassMatrix and Residual
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
from exudyn.misc.symbolicExport import ToFunction
try:
    import scipy.linalg
    import scipy.sparse
    import scipy.sparse.linalg
except ImportError:
    raise ImportError('exudyn.misc.symbolicSystem needs scipy for its sparse matrices: pip install scipy')

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = ['SymbolicSystemBase', 'NumpySystem', 'SymbolicKind', 'SymbolicLoad', 'SymbolicSystem', 'explicitSolverTypes',
           'SolveExplicit', 'SolveGeneralizedAlpha']

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
    second order coordinates q, nODE1 first order coordinates y and nAE algebraic coordinates lambda with

        M(q, t) q_tt = f(q, q_t, t, y) - C_q^T lambda      (ODE2, the reaction forces C_q^T lambda by ReactionForces)
        y_t = g(q, q_t, y, t)                               (ODE1, ODE1RightHandSide)
        c(q, q_t, y, lambda, t) = 0                         (AE, AlgebraicEquations)

    A derived class sets nODE2 (nODE1 and nAE, zero by default) and computes Forces and MassMatrixTriplets (or
    MassMatrix), and ODE1RightHandSide, AlgebraicEquations and ReactionForces if it has such coordinates; the residuals
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

    def Forces(self, q=None, q_t=None, t=0., y=None):
        """the forces f(q, q_t, t, y) of the right-hand side, of size nODE2, without the reaction forces; y, the ODE1
        coordinates, is given only if nODE1 > 0"""
        raise NotImplementedError('SymbolicSystemBase.Forces: computed by the derived class')

    def ODE1RightHandSide(self, q, q_t, y, t):
        """the right-hand side g(q, q_t, y, t) = y_t of the ODE1 coordinates, of size nODE1"""
        if self.nODE1:
            raise NotImplementedError('SymbolicSystemBase.ODE1RightHandSide: computed by the derived class')
        return np.zeros(0)

    def AlgebraicEquations(self, q, q_t, y, lambda_, t, velocityLevel=False):
        """the algebraic equations c(q, q_t, y, lambda, t), of size nAE; velocityLevel: their time derivative (index 2)"""
        if self.nAE:
            raise NotImplementedError('SymbolicSystemBase.AlgebraicEquations: computed by the derived class')
        return np.zeros(0)

    def ReactionForces(self, q, q_t, y, lambda_, t):
        """the reaction forces C_q^T lambda of the algebraic coordinates, of size nODE2, which the ODE2 residual adds"""
        if self.nAE:
            raise NotImplementedError('SymbolicSystemBase.ReactionForces: computed by the derived class')
        return np.zeros(self.nODE2)

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

    def Residual(self, q=None, q_t=None, q_tt=None, t=0., y=None):
        """the residual r = M(q, t) q_tt - f(q, q_t, t, y) of the ODE2 equations, of size nODE2, without reaction forces"""
        (q, q_t, q_tt) = self._State(q, q_t, q_tt)
        forces = self.Forces(q, q_t, t) if y is None else self.Forces(q, q_t, t, y=y)
        return self.MassMatrix(q, q_t, t, sparse=True) @ q_tt - forces

    def SystemResidual(self, q, q_t, q_tt, y, y_t, lambda_, t, velocityLevel=False):
        """the residual of all equations as the implicit solver of Exudyn has it (#3134): [M q_tt - f + C_q^T lambda,
        g - y_t, c], of size nODE2 + nODE1 + nAE"""
        yArgument = y if self.nODE1 else None
        parts = [self.Residual(q, q_t, q_tt, t, y=yArgument)]
        if self.nAE:
            parts[0] = parts[0] + self.ReactionForces(q, q_t, y, lambda_, t)
        if self.nODE1:
            parts.append(self.ODE1RightHandSide(q, q_t, y, t) - y_t)
        if self.nAE:
            parts.append(self.AlgebraicEquations(q, q_t, y, lambda_, t, velocityLevel))
        return np.concatenate(parts)

    def SystemJacobian(self, q, q_t, q_tt, y, y_t, lambda_, t, factor_q=1., factor_q_t=0., factor_q_tt=0., factor_y=1.,
                       factor_y_t=0., velocityLevel=False, epsilon=1e-6):
        """the Jacobian of SystemResidual (#3134), column by column: ODE2 columns factor_q d/dq + factor_q_t d/dq_t +
        factor_q_tt d/dq_tt, ODE1 columns factor_y d/dy + factor_y_t d/dy_t, AE columns d/dlambda; the ODE2 block by
        ResidualJacobian (exact for a SymbolicSystem), the others by central differences of SystemResidual

        Args:
            q: the ODE2 coordinates
            q_t: the ODE2 velocities
            q_tt: the ODE2 accelerations
            y: the ODE1 coordinates
            y_t: their velocities
            lambda_: the algebraic coordinates lambda
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
        state = [np.array(x, dtype=float) for x in [q, q_t, q_tt, y, y_t, lambda_]]
        J = np.zeros((nSystem, nSystem))
        if nODE1:
            J[:nODE2, :nODE2] = self.ResidualJacobian(q, q_t, q_tt, t, factor_q, factor_q_t, factor_q_tt, y=y)
        else:
            J[:nODE2, :nODE2] = self.ResidualJacobian(q, q_t, q_tt, t, factor_q, factor_q_t, factor_q_tt)

        def Difference(direction, rows): #the derivative of the residual rows along a direction of the state
            residuals = []
            for sign in [1., -1.]:
                perturbed = [x + sign*epsilon*d for (x, d) in zip(state, direction)]
                residuals.append(self.SystemResidual(*perturbed, t, velocityLevel)[rows] if rows != 'reaction'
                                 else self.ReactionForces(perturbed[0], perturbed[1], perturbed[3], perturbed[5], t))
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

    def ResidualJacobian(self, q=None, q_t=None, q_tt=None, t=0., factor_q=1., factor_q_t=0., factor_q_tt=0.,
                         sparse=False, epsilon=1e-6, y=None):
        """the Jacobian of the residual, factor_q dr/dq + factor_q_t dr/dq_t + factor_q_tt dr/dq_tt, by central
        differences of Residual - for a system without derivatives of its own; factor_q_tt=1 alone gives the mass
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
                    residuals.append(self.Residual(*perturbed, t, y=y))
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
        forces: the function (q, q_t, t) -> f, of size nODE2; with ODE1 coordinates (q, q_t, t, y) -> f
        initialState: (q0, q_t0); None: zero
        massMatrixBlocks: the blocks of bodies as SymbolicSystemBase.MassMatrixBlocks returns them, for a constant mass
            matrix; None: one block of all coordinates
        ode1RightHandSide: None, or the function (q, q_t, y, t) -> y_t of the ODE1 coordinates
        initialStateODE1: the initial ODE1 coordinates y0, which also give their number
        constraints: None, or the function (q, q_t, y, lambda_, t) -> c of the algebraic equations on position level
        constraintJacobian: the function (q, q_t, y, t) -> C_q (nAE x nODE2) of the reaction forces C_q^T lambda
        numberOfConstraints: nAE
    """
    def __init__(self, massMatrix, forces, initialState=None, massMatrixBlocks=None, ode1RightHandSide=None,
                 initialStateODE1=None, constraints=None, constraintJacobian=None, numberOfConstraints=0):
        self._massMatrix = massMatrix
        self._forces = forces
        self.nODE2 = (len(initialState[0]) if initialState is not None else massMatrix.shape[0])
        self._initialState = initialState
        self._massMatrixBlocks = massMatrixBlocks
        self._ode1 = ode1RightHandSide
        self._y0 = np.array(initialStateODE1 if initialStateODE1 is not None else [], dtype=float)
        self.nODE1 = len(self._y0) if ode1RightHandSide is not None else 0
        self._constraints = constraints
        self._constraintJacobian = constraintJacobian
        self.nAE = int(numberOfConstraints) if constraints is not None else 0

    def InitialStateODE1(self):
        """y0 as given"""
        return self._y0.copy()

    def ODE1RightHandSide(self, q, q_t, y, t):
        """g(q, q_t, y, t) of the function given"""
        if self._ode1 is None:
            return np.zeros(0)
        return np.asarray(self._ode1(q, q_t, y, t), dtype=float)

    def AlgebraicEquations(self, q, q_t, y, lambda_, t, velocityLevel=False):
        """c of the function given; velocity level not supported"""
        if velocityLevel:
            raise ValueError('NumpySystem.AlgebraicEquations: the constraints are given on position level only')
        if self._constraints is None:
            return np.zeros(0)
        return np.asarray(self._constraints(q, q_t, y, lambda_, t), dtype=float)

    def ReactionForces(self, q, q_t, y, lambda_, t):
        """C_q^T lambda of the constraint Jacobian given"""
        if self._constraints is None:
            return np.zeros(self.nODE2)
        return np.asarray(self._constraintJacobian(q, q_t, y, t), dtype=float).T @ lambda_

    def InitialState(self):
        """(q0, q_t0) as given, zero otherwise"""
        if self._initialState is None:
            return SymbolicSystemBase.InitialState(self)
        return tuple(np.array(x, dtype=float) for x in self._initialState)

    def IsMassMatrixConstant(self):
        """True for a mass matrix given as matrix"""
        return not callable(self._massMatrix)

    def Forces(self, q=None, q_t=None, t=0., y=None):
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
        composeSystem: True: Forces, MassMatrix and Residual evaluate two recordings of the composed system - the
            forces and the nonzero entries of the mass matrix of Equations(), each one flat program (#3131) -; False:
            each object and load by its own recordings, one call each

    Note:
        Its members: mbs; kinds, the list of SymbolicKind; jointKinds, the list of SymbolicKind of the ObjectJointUser
        (#3141), whose constraint equations are AlgebraicEquations and whose reaction forces are ReactionForces - they are
        not in Equations() -; ode1Kinds, the list of SymbolicKind of the ObjectGenericODE1 (#3144), whose right-hand
        side is ODE1RightHandSide; loads, the list of SymbolicLoad on ODE2 coordinates, ode1Loads those on ODE1
        coordinates; nODE2, nODE1, the numbers of ODE2 and ODE1 coordinates of the system; nAE, the number of the
        algebraic equations; qRef, the reference values of the ODE2 coordinates; composed, None or the dict of
        the recordings 'forces' and 'massMatrix' and the 'rows' and 'columns' of the entries of the mass matrix
    """
    def __init__(self, mbs, composeSystem=True):
        if not mbs.systemIsConsistent:
            raise ValueError('SymbolicSystem: mbs must be assembled first')
        self.mbs = mbs
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
        self.composed = self._Compose() if composeSystem else None

    def _Compose(self):
        """the recordings of the composed system (#3131): the forces f(q, q_t, t) and the entries of the mass matrix at
        the rows and columns of the local mass matrices, recorded as functions of the arguments (mbs, t, itemNumber, q,
        q_t) of ObjectGenericODE2 from the expressions of Equations; one call of each evaluates the system"""
        from exudyn.advancedUtilities import CreateSymbolicUserFunction
        entries = set()
        for kind in self.kinds:
            if kind.objectType != 'ConnectorUser':
                for ltg in kind.ltg:
                    entries.update((int(i), int(j)) for i in ltg for j in ltg)
        entries = sorted(entries)
        rows = np.array([i for (i, j) in entries], dtype=int)
        columns = np.array([j for (i, j) in entries], dtype=int)
        backend = _Backend('symbolic')

        def UFforces(mbs, t, itemNumber, q, q_t):
            return backend.Vector(self._EquationLists(backend, q, q_t, t)[1])

        def UFmassMatrix(mbs, t, itemNumber, q, q_t):
            M = self._EquationLists(backend, q, q_t, t)[0]
            return backend.Vector([M[i][j] for (i, j) in entries] if entries else [0.])

        def UFmassMatrixBlocks(mbs, t, itemNumber, q, q_t): #the entries of all local mass matrices, row by row
            blocks = self._EquationLists(backend, q, q_t, t)[2]
            values = [x for (ltg, kindBlocks) in blocks for block in kindBlocks for row in block for x in row]
            return backend.Vector(values if values else [0.])

        def Record(function):
            return CreateSymbolicUserFunction(self.mbs, function, 'forceUserFunction', itemTypeName='ObjectGenericODE2')

        return {'forces': Record(UFforces), 'massMatrix': Record(UFmassMatrix), 'massMatrixBlocks': Record(UFmassMatrixBlocks),
                'rows': rows, 'columns': columns}

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
            return 'not active (its equations lambda = 0 are not given)'
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
        mbs = self.mbs
        if orientation is None:
            if item['markerAccessUserFunction'] != 0 or item['postNewtonStepUserFunction'] != 0:
                return (None, None, 'a markerAccessUserFunction or postNewtonStepUserFunction')
            orientation = bool(int(item['requestedMarkerType']) & int(exudyn.MarkerType.Orientation))
        (markers, ltg) = ([], [])
        for markerNumber in item['markerNumbers']:
            marker = mbs.GetMarker(markerNumber)
            markerType = marker['markerType']
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
            elif markerType in ['BodyPosition', 'BodyRigid']:
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
            else:
                return (None, None, 'a marker of type ' + markerType)
            data['indices'] = np.arange(len(ltg), len(ltg) + len(coordinates), dtype=int)
            markers.append(data)
            ltg += coordinates
        return (markers, ltg, '')

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

    def ODE1RightHandSide(self, q, q_t, y, t):
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
                np.add.at(g, kind.ltg[i], self._LocalODE1RightHandSide(kind, i, y, t))
        for load in self.ode1Loads:
            np.add.at(g, load.ltg, np.atleast_1d(self._LoadValue(load, t)))
        return g

    def _LocalODE1RightHandSide(self, kind, i, y, t):
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
        """the derivative of ODE1RightHandSide by y (nODE1 x nODE1), exact: A and the derivative of the recorded
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

    def AlgebraicEquations(self, q, q_t, y, lambda_, t, velocityLevel=False):
        """the constraint equations c(q, t) of the joints at position level (#3141), by their recorded
        constraintUserFunction; the velocity level is not given

        Args:
            q: the ODE2 coordinates
            q_t: the ODE2 velocities, not used
            y: the ODE1 coordinates, not used
            lambda_: the algebraic coordinates lambda, not used
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

    def ReactionForces(self, q, q_t, y, lambda_, t):
        """the reaction forces C_q^T lambda of the joints (#3141): the recorded jacobianUserFunction C by the marker
        translations and global rotation increments, C^T lambda the forces and torques on the markers, projected into the
        coordinates by the position and rotation Jacobians of the markers, as the forces of a connector

        Args:
            q: the ODE2 coordinates
            q_t: the ODE2 velocities, not used
            y: the ODE1 coordinates, not used
            lambda_: the algebraic coordinates lambda
            t: the time

        Returns:
            the vector of size nODE2
        """
        (q, lambda_) = (np.asarray(q, dtype=float), np.asarray(lambda_, dtype=float))
        f = np.zeros(self.nODE2)
        for kind in self.jointKinds:
            for i in range(len(kind.objects)):
                joint = kind.parameters[i]
                (arguments, markerData) = self._JointArguments(kind, i, q, t)
                C = np.array(kind.recordings['jacobianUserFunction'].Evaluate(self.mbs, *arguments), dtype=float).reshape(-1, 12)
                forces = C.T @ lambda_[joint['ltgAE']]
                local = np.zeros(kind.ltg.shape[1])
                for (k, (marker, (Jpos, Jrot, bodyArguments))) in enumerate(zip(joint['markers'], markerData)):
                    if Jpos is not None:
                        local[marker['indices']] += Jpos.T @ forces[6*k:6*k + 3]
                    if Jrot is not None:
                        local[marker['indices']] += Jrot.T @ forces[6*k + 3:6*k + 6]
                np.add.at(f, kind.ltg[i], local)
        return f

    def _JointJacobian(self, kind, i, q, lambda_, t):
        """the derivatives of joint i of kind in its local coordinates (#3145): C_q, the derivative of its constraint
        equations by the coordinates - the recorded jacobianUserFunction C by the marker translations and global
        rotation increments times the position and rotation Jacobians of the markers -, and the derivative of its
        reaction forces C_q^T lambda by the coordinates: the derivatives of C by the positions and rotation matrices of
        the markers (arguments 3 + 2k and 4 + 2k of the recording), composed with J_pos and dA/dq of the bodies, and the
        derivatives of J_pos and J_rot of the bodies times the forces and torques C^T lambda"""
        n = kind.ltg.shape[1]
        joint = kind.parameters[i]
        (arguments, markerData) = self._JointArguments(kind, i, q, t)
        recording = kind.recordings['jacobianUserFunction']
        C = np.array(recording.Evaluate(self.mbs, *arguments), dtype=float).reshape(-1, 12)
        lambdaJoint = lambda_[joint['ltgAE']]
        forces = C.T @ lambdaJoint #the forces and torques on the markers
        C_q = np.zeros((C.shape[0], n))
        dForces = np.zeros((12, n)) #the derivative of C^T lambda by the local coordinates
        for (k, (marker, (Jpos, Jrot, bodyArguments))) in enumerate(zip(joint['markers'], markerData)):
            if Jpos is None:
                continue
            indices = marker['indices']
            C_q[:, indices] += C[:, 6*k:6*k + 3] @ Jpos
            dForces[:, indices] += np.einsum('ijk,i->jk', self._RecordingJacobian(recording, 3 + 2*k, arguments).reshape(-1, 12, 3),
                                             lambdaJoint) @ Jpos
            if Jrot is not None:
                C_q[:, indices] += C[:, 6*k + 3:6*k + 6] @ Jrot
            if bodyArguments is not None and Jrot is not None: #the rotation of the marker after the one of the body
                dA = np.einsum('ijk,jl->ilk', self._BodyJacobian(marker, 'rotationMatrixUserFunction', bodyArguments),
                               marker['rotation'])
                dForces[:, indices] += np.einsum('ijk,i->jk', self._RecordingJacobian(recording, 4 + 2*k, arguments).reshape(-1, 12, 9),
                                                 lambdaJoint) @ dA.reshape(9, len(indices))
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

    def SystemJacobian(self, q, q_t, q_tt, y, y_t, lambda_, t, factor_q=1., factor_q_t=0., factor_q_tt=0., factor_y=1.,
                       factor_y_t=0., velocityLevel=False, epsilon=1e-6):
        """the Jacobian of SystemResidual, exact (#3145): the ODE2 block by ResidualJacobian plus the derivative of the
        reaction forces, the blocks C_q^T and C_q of the joints by _JointJacobian, the ODE1 block by ODE1Jacobian; the
        arguments as SymbolicSystemBase.SystemJacobian, epsilon not used

        Args:
            q: the ODE2 coordinates
            q_t: the ODE2 velocities
            q_tt: the ODE2 accelerations
            y: the ODE1 coordinates
            y_t: their velocities
            lambda_: the algebraic coordinates lambda
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
        J[:nODE2, :nODE2] = self.ResidualJacobian(q, q_t, q_tt, t, factor_q, factor_q_t, factor_q_tt)
        if nAE:
            lambda_ = np.asarray(lambda_, dtype=float)
            offset = nODE2 + nODE1 #the first algebraic coordinate
            for kind in self.jointKinds:
                for i in range(len(kind.objects)):
                    (ltg, ltgAE) = (kind.ltg[i], kind.parameters[i]['ltgAE'] + offset)
                    (C_q, dReaction) = self._JointJacobian(kind, i, q, lambda_, t)
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

    def Forces(self, q=None, q_t=None, t=0., y=None):
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
            return np.array(self.composed['forces'].Evaluate(self.mbs, t, 0, q, q_t), dtype=float)
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
            values = np.array(self.composed['massMatrix'].Evaluate(self.mbs, t, 0, q, q_t), dtype=float)
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
            values = np.array(self.composed['massMatrixBlocks'].Evaluate(self.mbs, t, 0, q, q_t), dtype=float)
            (result, start) = ([], 0)
            for kind in kinds:
                (m, nLocal) = kind.ltg.shape
                result.append((kind.ltg, values[start:start + m*nLocal*nLocal].reshape(m, nLocal, nLocal)))
                start += m*nLocal*nLocal
            return result
        return [(kind.ltg, np.array([self._LocalMassMatrix(kind, i, q, q_t, t) for i in range(len(kind.objects))]))
                for kind in kinds]

    def Residual(self, q=None, q_t=None, q_tt=None, t=0., y=None):
        """the residual r = M(q) q_tt - f(q, q_t, t) of the ODE2 equations without forming M: by the recordings of the
        composed system, else object by object; y is not used, as no item taken couples the ODE1 coordinates into the
        ODE2 equations"""
        (q, q_t, q_tt) = self._State(q, q_t, q_tt)
        if self.composed is not None:
            (rows, columns, values) = self.MassMatrixTriplets(q, q_t, t)
            return np.bincount(rows, values*q_tt[columns], minlength=self.nODE2) - self.Forces(q, q_t, t)
        r = np.zeros(self.nODE2)
        for kind in self.kinds:
            for i in range(len(kind.objects)):
                ltg = kind.ltg[i]
                local = self._LocalMassMatrix(kind, i, q, q_t, t) @ q_tt[ltg] - self._LocalForces(kind, i, q, q_t, t)
                np.add.at(r, ltg, local)
        for load in self.loads:
            np.add.at(r, load.ltg, -self._LoadForces(load, q, q_t, t))
        return r

    def ResidualJacobian(self, q=None, q_t=None, q_tt=None, t=0., factor_q=1., factor_q_t=0., factor_q_tt=0.,
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
                    J -= factor*np.array(composed['forces'].EvaluateJacobian(name, self.mbs, t, 0, q, q_t), dtype=float)
            (rows, columns) = (composed['rows'], composed['columns'])
            if factor_q != 0. and len(rows) and not self._constantMassMatrix: #d(M q_tt)/dq
                dM = np.array(composed['massMatrix'].EvaluateJacobian('q', self.mbs, t, 0, q, q_t), dtype=float)[:len(rows)]
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

    def Equations(self, format='symbolic', optimize=True, variables=None, massMatrix=True, forces=True,
                  massMatrixBlocks=False):
        """the equations of motion M(q, t) q_tt = f(q, q_t, t) of the system as expressions in its variables (#3130,
        #3132), composed of the recordings of the kinds, the marker kinematics of the connectors and the loads; each
        recording is called with symbolic arguments through its code of exudyn.misc.symbolicExport, the parameters of
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
                recorded -, used instead of new variables
            massMatrix: True: the global mass matrix, 'massMatrix' (nODE2 x nODE2)
            forces: True: the forces, 'forces' (nODE2)
            massMatrixBlocks: True: the mass matrices of the bodies, 'massMatrixBlocks', as MassMatrixBlocks() gives
                them: a list per kind of bodies of (ltg, matrices), the LTG table of the kind (bodies x n) and the list
                of their n x n matrices

        Returns:
            dict with 'q', 'q_t', 't' (the variables) and the results asked for, exudyn.symbolic.Matrix and Vector or
            sympy.Matrix
        """
        backend = _Backend(format)
        n = self.nODE2
        (q0, q_t0) = self.InitialState()
        if variables is None:
            (q, q_t) = (backend.Variable(name, n, value) for (name, value) in [('q', q0), ('q_t', q_t0)])
            t = backend.Real('t')
        else:
            (q, q_t, t) = variables
        (M, f, blocks) = self._EquationLists(backend, q, q_t, t)
        (names, results) = ([], [])
        for (name, wanted, Result) in [('massMatrix', massMatrix, lambda: backend.Matrix(M)), ('forces', forces, lambda: backend.Vector(f))]:
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
        for (name, result) in zip(names, results):
            equations[name] = result
        if massMatrixBlocks: #back into the kinds
            matrices = list(equations['massMatrixBlocks'])
            equations['massMatrixBlocks'] = []
            for (ltg, kindBlocks) in blocks:
                equations['massMatrixBlocks'].append((ltg, matrices[:len(kindBlocks)]))
                matrices = matrices[len(kindBlocks):]
        return equations

    def _EquationLists(self, backend, q, q_t, t):
        """the mass matrix as list of rows, the forces as list of the expressions of Equations, of the variables q, q_t
        and t of backend, and the local mass matrices of the bodies, per kind (ltg, list of their lists of rows)"""
        n = self.nODE2
        format = backend.format
        (qs, q_ts) = ([x[i] for i in range(n)] for x in [q, q_t]) #the components, to be gathered per item
        M = [[0]*n for i in range(n)]
        f = [0]*n
        blocks = [(kind.ltg, []) for kind in self.kinds if kind.objectType != 'ConnectorUser']
        blocksOfKind = {id(kind): blocks[k][1] for (k, kind) in enumerate(kind for kind in self.kinds if kind.objectType != 'ConnectorUser')}
        functions = {} #the function of each recording, written once

        def Call(recording, *arguments):
            if id(recording) not in functions:
                functions[id(recording)] = ToFunction(recording.GetFlatProgramData(), format)
            return functions[id(recording)](None, *arguments)

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
                                 backend.Vector(kind.parameters[index]))
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
                         backend.Vector(body['parameters']))
            withPosition = arguments[:2] + (backend.Vector(body['localPosition']),) + arguments[2:]
            J = Call(body['projection'], *(arguments if body['projectionName'] == 'massIntegralUserFunction' else withPosition))
            if body['rotation'] is not None:
                value = Call(body['rotation'], *withPosition) @ value
            Add(f, load.ltg, Components(J.T @ value, len(load.ltg)))
        return (M, f, blocks)

    def _ConnectorEquations(self, kind, index, backend, Call, t, qs, q_ts, f):
        """add the generalized forces of connector index of kind to the list f, as expressions: the marker states as in
        _ConnectorStates, the forces of the recording, J_pos^T f + J_rot^T m of each marker"""
        (ltg, connector) = (kind.ltg[index], kind.parameters[index])
        stride = 6 if connector['orientation'] else 3
        (states, jacobians) = ([], [])
        (zero, unit) = (backend.Vector([0., 0., 0.]), backend.Matrix(np.eye(3)))
        for marker in connector['markers']:
            indices = ltg[marker['indices']]
            if marker['type'] == 'fixed':
                states += [backend.Vector(marker['position']), zero, backend.Matrix(marker['rotation']), zero]
                jacobians.append(None)
            elif marker['type'] == 'point':
                states += [backend.Vector([float(marker['position'][a]) + qs[i] for (a, i) in enumerate(indices)]),
                           backend.Vector([q_ts[i] for i in indices]), unit, zero]
                jacobians.append(('point', None, None))
            else:
                recordings = marker['recordings']
                q_tLocal = backend.Vector([q_ts[i] for i in indices])
                arguments = (t, marker['objectNumber'], backend.Vector(marker['position']), backend.Vector(marker['qRef']),
                             backend.Vector([float(marker['qRef'][a]) + qs[i] for (a, i) in enumerate(indices)]), q_tLocal,
                             backend.Vector(marker['parameters']))
                Jpos = Call(recordings['positionJacobianUserFunction'], *arguments)
                (rotation, angularVelocity, Jrot) = (unit, zero, None)
                if connector['orientation']:
                    Jrot = Call(recordings['rotationJacobianUserFunction'], *arguments)
                    rotation = Call(recordings['rotationMatrixUserFunction'], *arguments) @ backend.Matrix(marker['rotation'])
                    angularVelocity = Jrot @ q_tLocal
                states += [Call(recordings['positionUserFunction'], *arguments), Jpos @ q_tLocal, rotation, angularVelocity]
                jacobians.append(('body', Jpos, Jrot))
        forces = Call(kind.recordings['forceUserFunction'], t, kind.objects[index], *states, backend.Vector([]),
                      backend.Vector(connector['parameters']))
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

    def GetFlatProgram(self, format='Python', flatten=True, results=['forces', 'massMatrix']):
        """the flat program of results of Equations(), a function of q, q_t and t (#3130, #3132)

        Args:
            format: 'Python' (the text), or the executable 'symbolic', 'numpy' or 'sympy' of exudyn.misc.symbolicExport
            flatten: True: one line per operation; False: each result as one expression
            results: the names of the results, 'forces' and 'massMatrix'

        Returns:
            the program as str, for the executable formats a function Equations(q, q_t, t) returning the results
        """
        equations = self.Equations('symbolic', massMatrix='massMatrix' in results, forces='forces' in results)
        return exudyn.symbolic.GetFlatProgram([equations[name] for name in results], arguments=[equations[name] for name in ['q', 'q_t', 't']],
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


def _Sum(a, b):
    """a + b, without the sum when a is the number 0 that a sum starts with"""
    return b if isinstance(a, int) and a == 0 else a + b


class _Backend:
    """the types of Equations: exudyn.symbolic or sympy"""
    def __init__(self, format):
        if format not in ['symbolic', 'sympy']:
            raise ValueError("SymbolicSystem.Equations: format must be 'symbolic' or 'sympy', not " + str(format))
        self.format = format
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

    def Vector(self, components):
        components = [float(x) if isinstance(x, (np.floating, np.integer)) else x for x in components]
        return exudyn.symbolic.Vector(components) if self.format == 'symbolic' else self.sympy.Matrix(components)

    def Matrix(self, rows):
        rows = [[float(x) if isinstance(x, (np.floating, np.integer)) else x for x in row] for row in rows]
        return exudyn.symbolic.Matrix(rows) if self.format == 'symbolic' else self.sympy.Matrix(rows)


#the explicit solvers of SolveExplicit
explicitSolverTypes = [exudyn.DynamicSolverType.ExplicitEuler, exudyn.DynamicSolverType.RK44,
                       exudyn.DynamicSolverType.VelocityVerlet]

#the Butcher tableaus (A, weights, times) of src/Solver/CSolverExplicit.cpp
_butcherTableaus = {exudyn.DynamicSolverType.ExplicitEuler: ([[0.]], [1.], [0.]),
                    exudyn.DynamicSolverType.RK44: ([[0., 0., 0., 0.], [0.5, 0., 0., 0.], [0., 0.5, 0., 0.], [0., 0., 1., 0.]],
                                                    [1./6., 1./3., 1./3., 1./6.], [0., 0.5, 0.5, 1.])}


#up to this number of coordinates, the mass matrix is decomposed as dense matrix: forming a sparse one costs more (#3131)
_denseMassMatrixSize = 200


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
            self.result['lambda'] = []
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

    def Step(self, stepIndex, t, q, q_t, q_tt, last=False, y=None, y_t=None, lambda_=None):
        """the state after step stepIndex at t; stepIndex 0 is the initial state, as SolveDynamic writes it"""
        if last:
            self.lastWritten = self.tEnd - self.h
        if stepIndex == 0 or t - self.lastWritten >= -1e-10:
            self.lastWritten = max(self.lastWritten + self.period, t)
            for (name, value) in [('time', t), ('q', q), ('q_t', q_t), ('q_tt', q_tt), ('y', y), ('y_t', y_t), ('lambda', lambda_)]:
                if name in self.result:
                    self.result[name].append(np.copy(value))
        if self.solver is not None:
            self.system.SetSystemState(q, q_t, q_tt, t)
            if y is not None and len(y):
                self.system.mbs.systemData.SetODE1Coordinates(np.asarray(y, dtype=float))
                if y_t is not None:
                    self.system.mbs.systemData.SetODE1Coordinates_t(np.asarray(y_t, dtype=float))
            if lambda_ is not None and len(lambda_):
                self.system.mbs.systemData.SetAECoordinates(np.asarray(lambda_, dtype=float))
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
    def Accelerations(q, q_t, t, y=None):
        forceEvaluations[0] += 1
        forces = system.Forces(q, q_t, t, y=y) if system.nODE1 else system.Forces(q, q_t, t)
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
                K_y.append(system.ODE1RightHandSide(g, g_t, g_y, t0 + h*times[i]) if system.nODE1 else g_y)
            for i in range(len(weights)):
                if weights[i] != 0.:
                    q = q + (weights[i]*h)*K[i]
                    q_t = q_t + (weights[i]*h)*K_t[i]
                    y = y + (weights[i]*h)*K_y[i]
            if system.nODE1:
                y_t = system.ODE1RightHandSide(q, q_t, y, t)
            if timeIntegration.explicit.computeEndOfStepAccelerations:
                q_tt = Accelerations(q, q_t, t, y)
            else:
                q_tt = K_t[-1]
        output.Step(step, t, q, q_t, q_tt, last=(step == n), y=y, y_t=y_t)

    return output.Finalize({'steps': n, 'forceEvaluations': forceEvaluations[0],
                            'massMatrixEvaluations': massSolver.evaluations})


def SolveGeneralizedAlpha(system, simulationSettings=None):
    """the implicit time integration of a SymbolicSystemBase with constant step size by the generalized-alpha method
    in the form of Arnold and Bruls, as Exudyn's MainSolverImplicitSecondOrder computes it (#3126, #3134, design
    docs/dev/design/symbolicSystem.md §6): per step the predictor of the displacements, velocities and algorithmic
    accelerations, the ODE1 coordinates by the trapezoidal rule, then Newton's method for the accelerations, the ODE1
    coordinates and the algebraic coordinates with the residual of SystemResidual - r = M q_tt - f + C_q^T lambda,
    g - y_t, c - and its Jacobian, the ODE2 rows scaled by beta h^2 and the algebraic columns by 1/(beta h^2), as
    Exudyn; by timeIntegration: numberOfSteps, startTime, endTime, generalizedAlpha.spectralRadius, useNewmark with
    newmarkBeta and newmarkGamma, computeInitialAccelerations - with constraints of [M C_q^T; C_q 0] [q_tt; lambda] =
    [f; -(C_q q_t)_q q_t] -, storeInitialAlgebraicCoordinates, and newton.relativeTolerance, absoluteTolerance,
    maxIterations, useModifiedNewton (the Jacobian once per step) and weightTolerancePerCoordinate; for a
    SymbolicSystem the solution file, the sensors and the renderer of its mbs, as SolveExplicit. A step whose Newton
    iteration does not converge raises: the step size is constant. The ODE1 velocities start at zero, as in Exudyn

    Args:
        system: the system, a SymbolicSystemBase
        simulationSettings: exudyn.SimulationSettings; None: the defaults

    Returns:
        dict with 'time' (n_out), 'q', 'q_t', 'q_tt' (n_out x nODE2), with ODE1 coordinates 'y', 'y_t', with algebraic
        equations 'lambda', at the steps of solution.file.writePeriod and the last step, and the statistics 'steps',
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
    (y, y_t, lambda_) = (system.InitialStateODE1(), np.zeros(nODE1), np.zeros(nAE))
    if generalizedAlpha.computeInitialAccelerations:
        if not general:
            q_tt = scipy.sparse.linalg.spsolve(system.MassMatrix(q, q_t, tStart, sparse=True).tocsc(), system.Forces(q, q_t, tStart))
        else:
            (q_tt, lambdaInitial) = _InitialAccelerations(system, q, q_t, y, tStart)
            if generalizedAlpha.storeInitialAlgebraicCoordinates:
                lambda_ = lambdaInitial
    aAlgorithmic = q_tt.copy()
    output = _Output(system, simulationSettings, tStart, tEnd, h)
    output.Step(0, tStart, q, q_t, q_tt, y=y, y_t=y_t, lambda_=lambda_)

    def Residual():
        if general:
            return system.SystemResidual(q, q_t, q_tt, y, y_t, lambda_, t)
        return system.Residual(q, q_t, q_tt, t)

    (newtonIterations, jacobianEvaluations) = (0, 0)
    for step in range(1, n + 1):
        t = tStart + step*h
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
        initialResidual = cMath.norm(Residual())/errorFactor
        if initialResidual <= newton.absoluteTolerance:
            initialResidual = 2*newton.absoluteTolerance #at least one Newton step
        factorization = None
        converged = False
        for iteration in range(int(newton.maxIterations)):
            if factorization is None or not newton.useModifiedNewton:
                if general:
                    J = system.SystemJacobian(q, q_t, q_tt, y, y_t, lambda_, t, factor_q=1., factor_q_t=gammaPrime, factor_q_tt=betaPrime,
                                              factor_y=1., factor_y_t=2./h)
                    J[:nODE2, :] *= scaling
                    J[:, nODE2 + nODE1:] /= scaling
                    factorization = scipy.linalg.lu_factor(J)
                else:
                    factorization = scipy.sparse.linalg.splu(system.ResidualJacobian(q, q_t, q_tt, t, factor_q=1., factor_q_t=gammaPrime,
                                                                                     factor_q_tt=betaPrime, sparse=True).tocsc())
                jacobianEvaluations += 1
            if general:
                r = Residual()
                r[:nODE2] *= scaling
                increment = scipy.linalg.lu_solve(factorization, r)
                y = y - increment[nODE2:nODE2 + nODE1]
                y_t = y_t - (2./h)*increment[nODE2:nODE2 + nODE1]
                lambda_ = lambda_ - increment[nODE2 + nODE1:]/scaling
                increment = increment[:nODE2]
            else:
                increment = factorization.solve(Residual())
            q = q - increment
            q_t = q_t - gammaPrime*increment
            q_tt = q_tt - betaPrime*increment
            newtonIterations += 1
            residual = cMath.norm(Residual())/errorFactor
            if residual/initialResidual <= newton.relativeTolerance or residual <= newton.absoluteTolerance:
                converged = True
                break
        if not converged:
            raise ValueError('SolveGeneralizedAlpha: Newton did not converge in step ' + str(step) + ' at t=' + str(t)
                             + ' after ' + str(int(newton.maxIterations)) + ' iterations; residual ' + str(residual))
        if not generalizedAlpha.useNewmark:
            aAlgorithmic = aAlgorithmic + ((1 - alphaF)/(1 - alphaM))*q_tt #FinalizeNewton
        output.Step(step, t, q, q_t, q_tt, last=(step == n), y=y, y_t=y_t, lambda_=lambda_)

    return output.Finalize({'steps': n, 'newtonIterations': newtonIterations, 'jacobianEvaluations': jacobianEvaluations})


def _InitialAccelerations(system, q, q_t, y, t, epsilon=1e-6):
    """the initial accelerations and algebraic coordinates of CSolverImplicitSecondOrder: [M C_q^T; C_q 0] [q_tt; lambda]
    = [f; -(C_q q_t)_q q_t], the ODE1 rows a unit matrix with zero right-hand side; C_q by central differences of the
    algebraic equations, (C_q q_t)_q q_t as their second derivative along q_t"""
    (nODE2, nODE1, nAE) = (system.nODE2, system.nODE1, system.nAE)
    nSystem = nODE2 + nODE1 + nAE
    lambda_ = np.zeros(nAE)
    A = np.zeros((nSystem, nSystem))
    rhs = np.zeros(nSystem)
    A[:nODE2, :nODE2] = system.MassMatrix(q, q_t, t)
    rhs[:nODE2] = system.Forces(q, q_t, t, y=y) if nODE1 else system.Forces(q, q_t, t)
    A[nODE2:nODE2 + nODE1, nODE2:nODE2 + nODE1] = np.eye(nODE1)
    if nAE:
        def Equations(qq):
            return system.AlgebraicEquations(qq, q_t, y, lambda_, t)
        Cq = np.array([(Equations(q + epsilon*e) - Equations(q - epsilon*e))/(2*epsilon) for e in np.eye(nODE2)]).T
        A[nODE2 + nODE1:, :nODE2] = Cq
        A[:nODE2, nODE2 + nODE1:] = Cq.T
        if cMath.norm(q_t) > 1e-10:
            delta = 1e-4
            rhs[nODE2 + nODE1:] = -(Equations(q + delta*q_t) - 2*Equations(q) + Equations(q - delta*q_t))/delta**2
    solution = np.linalg.solve(A, rhs)
    return solution[:nODE2], solution[nODE2 + nODE1:]
