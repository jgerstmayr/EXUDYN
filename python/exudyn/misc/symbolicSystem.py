#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  The equations of motion of a system as symbolic programs (#3110, #3113, design:
#           docs/dev/design/symbolicSystem.md): SymbolicSystemBase is the interface of the ODE2 equations
#           M(q, t) q_tt = f(q, q_t, t) on which the solvers work. SymbolicSystem(mbs) extracts them, after
#           mbs.Assemble(), from the objects with symbolic user functions, collected in kinds - the objects whose user
#           functions are the same recordings -, each with the table of its objects, their parameters and the global
#           indices of their ODE2 coordinates (the local-to-global maps of systemData). It evaluates the residual
#           r = M q_tt - f, the sparse mass matrix, the forces and the Jacobian of r by exudyn.symbolic, and writes a
#           computed state back into mbs, where the renderer, the sensors and the output functions read it.
#           NumpySystem takes equations written by hand. SolveExplicit integrates any of them by explicit Euler, RK44
#           or velocity Verlet, as Exudyn's explicit solver does (#3112).
#           Special: not for the broad use, a basis for the export of the equations of motion and their evaluation
#           outside Exudyn. The items it takes: ObjectUserBody with symbolic massMatrixUserFunction and
#           forceUserFunction, ObjectGenericODE2 with constant matrices and symbolic user functions, and items without
#           coordinates (ground); every other object, loads, constraints and recordings that read mbs are refused.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-26 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import exudyn
try:
    import scipy.sparse
    import scipy.sparse.linalg
except ImportError:
    raise ImportError('exudyn.misc.symbolicSystem needs scipy for its sparse matrices: pip install scipy')

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = ['SymbolicSystemBase', 'NumpySystem', 'SymbolicKind', 'SymbolicSystem', 'explicitSolverTypes', 'SolveExplicit']

#the user functions of a kind, by object type
_userFunctionNames = {'UserBody': ['massMatrixUserFunction', 'forceUserFunction'],
                      'GenericODE2': ['massMatrixUserFunction', 'forceUserFunction']}
#the positions of the arguments q (the total coordinates for UserBody) and q_t in the user functions, mbs at 0
_argumentPositions = {'UserBody': (4, 5), 'GenericODE2': (3, 4)}


class SymbolicSystemBase:
    """the ODE2 equations of motion M(q, t) q_tt = f(q, q_t, t) of nODE2 coordinates, the interface on which the solvers
    work (#3113): a derived class sets nODE2 and computes Forces and MassMatrixTriplets (or MassMatrix); the residual and
    its Jacobian follow from them, unless the derived class computes them itself

    Note:
        Its member: nODE2, the number of ODE2 coordinates
    """
    nODE2 = 0

    def InitialState(self):
        """the initial coordinates and velocities (q0, q_t0); zero unless the derived class gives them"""
        return np.zeros(self.nODE2), np.zeros(self.nODE2)

    def IsMassMatrixConstant(self):
        """True if the mass matrix depends neither on q nor on t, so that a solver factorizes it once"""
        return False

    def Forces(self, q=None, q_t=None, t=0.):
        """the forces f(q, q_t, t) of the right-hand side, of size nODE2"""
        raise NotImplementedError('SymbolicSystemBase.Forces: computed by the derived class')

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

    def Residual(self, q=None, q_t=None, q_tt=None, t=0.):
        """the residual r = M(q, t) q_tt - f(q, q_t, t), of size nODE2"""
        (q, q_t, q_tt) = self._State(q, q_t, q_tt)
        return self.MassMatrix(q, q_t, t, sparse=True) @ q_tt - self.Forces(q, q_t, t)

    def ResidualJacobian(self, q=None, q_t=None, q_tt=None, t=0., factor_q=1., factor_q_t=0., factor_q_tt=0.,
                         sparse=False, epsilon=1e-6):
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
                    residuals.append(self.Residual(*perturbed, t))
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
    """ODE2 equations M q_tt = f(q, q_t, t) written by hand, e.g. with numpy: the simplest system for a solver of this
    module, and a check of it independent of Exudyn (#3112)

    Args:
        massMatrix: the mass matrix, constant (numpy array or scipy sparse matrix) or a function (q, q_t, t) -> matrix
        forces: the function (q, q_t, t) -> f, of size nODE2
        initialState: (q0, q_t0); None: zero
        massMatrixBlocks: the blocks of bodies as SymbolicSystemBase.MassMatrixBlocks returns them, for a constant mass
            matrix; None: one block of all coordinates
    """
    def __init__(self, massMatrix, forces, initialState=None, massMatrixBlocks=None):
        self._massMatrix = massMatrix
        self._forces = forces
        self.nODE2 = (len(initialState[0]) if initialState is not None else massMatrix.shape[0])
        self._initialState = initialState
        self._massMatrixBlocks = massMatrixBlocks

    def InitialState(self):
        """(q0, q_t0) as given, zero otherwise"""
        if self._initialState is None:
            return SymbolicSystemBase.InitialState(self)
        return tuple(np.array(x, dtype=float) for x in self._initialState)

    def IsMassMatrixConstant(self):
        """True for a mass matrix given as matrix"""
        return not callable(self._massMatrix)

    def Forces(self, q=None, q_t=None, t=0.):
        """f(q, q_t, t) of the function given"""
        (q, q_t, _) = self._State(q, q_t)
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


def _Recording(value, objectNumber, name):
    """the recorded symbolic user function of a parameter, None if there is none; a Python function, or a recording
    that reads mbs, raises"""
    if isinstance(value, dict) and value.get('type') == 'Symbolic':
        recording = value['function']
        if 'mbs.' in recording.GetFlatProgram():
            raise ValueError('SymbolicSystem: the ' + name + ' of object ' + str(objectNumber) + ' reads the MainSystem '
                             '(an item output), which is not part of the equations of the symbolic system')
        return recording
    if isinstance(value, dict) or (not isinstance(value, int) and value is not None):
        raise ValueError('SymbolicSystem: object ' + str(objectNumber) + ' has a Python ' + name
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
        Its members: objectType, the type name 'UserBody' or 'GenericODE2'; recordings, the dict of the recorded user
        functions by name (a name without recording is missing); objects, the list of the object numbers; ltg, the
        array (number of objects x local coordinates) of the global ODE2 indices; parameters, the list of the parameter
        vectors of the objects (UserBody) or of the dicts of the constant matrices and the force vector (GenericODE2)
    """
    def __init__(self, objectType, recordings):
        self.objectType = objectType
        self.recordings = recordings
        self.objects = []
        self.ltg = []
        self.parameters = []


class SymbolicSystem(SymbolicSystemBase):
    """the ODE2 equations of motion of mbs, by kinds of objects with symbolic user functions, evaluated by
    exudyn.symbolic (#3110, #3113); mbs must be assembled; see the Details of this module for the items it takes

    Args:
        mbs: the assembled MainSystem

    Note:
        Its members: mbs; kinds, the list of SymbolicKind; nODE2, the number of ODE2 coordinates of the system; qRef,
        the reference values of the ODE2 coordinates
    """
    def __init__(self, mbs):
        if not mbs.systemIsConsistent:
            raise ValueError('SymbolicSystem: mbs must be assembled first')
        self.mbs = mbs
        self.nODE2 = mbs.systemData.ODE2Size()
        self.qRef = np.array(mbs.systemData.GetODE2Coordinates(configuration=exudyn.ConfigurationType.Reference))
        refused = []
        if mbs.systemData.NumberOfLoads() > 0:
            refused += ['loads']
        for (name, size) in [('ODE1 coordinates', mbs.systemData.ODE1Size()), ('algebraic equations', mbs.systemData.AEsize()),
                             ('data coordinates', mbs.systemData.DataSize())]:
            if size > 0:
                refused += [name]
        kinds = {}
        self._constantMassMatrix = True
        for objectNumber in range(mbs.systemData.NumberOfObjects()):
            item = mbs.GetObject(objectNumber)
            objectType = item['objectType']
            ltg = list(mbs.systemData.GetObjectLTGODE2(objectNumber))
            if objectType not in _userFunctionNames:
                if len(ltg):
                    refused += ['object ' + str(objectNumber) + ' (' + objectType + ')']
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
            else:
                self._constantMassMatrix &= 'massMatrixUserFunction' not in recordings
            key = (objectType,) + tuple((name, recordings[name].GetFlatProgram()) for name in sorted(recordings))
            if key not in kinds:
                kinds[key] = SymbolicKind(objectType, recordings)
            kind = kinds[key]
            kind.objects.append(objectNumber)
            kind.ltg.append(ltg)
            if objectType == 'UserBody':
                kind.parameters.append(np.array(item['parameters'], dtype=float))
            else:
                kind.parameters.append({'M': _Dense(item['massMatrix']), 'K': _Dense(item['stiffnessMatrix']),
                                        'D': _Dense(item['dampingMatrix']), 'f': np.array(item['forceVector'], dtype=float)})
        if refused:
            raise ValueError('SymbolicSystem: these items cannot be assembled (yet): ' + ', '.join(refused))
        self.kinds = list(kinds.values())
        for kind in self.kinds:
            kind.ltg = np.array(kind.ltg, dtype=int)

    def InitialState(self):
        """the initial coordinates and velocities (q0, q_t0) of the ODE2 coordinates of mbs"""
        systemData = self.mbs.systemData
        return (np.array(systemData.GetODE2Coordinates(configuration=exudyn.ConfigurationType.Initial)),
                np.array(systemData.GetODE2Coordinates_t(configuration=exudyn.ConfigurationType.Initial)))

    def IsMassMatrixConstant(self):
        """True if every user body has constantMassMatrix and no ObjectGenericODE2 has a massMatrixUserFunction"""
        return self._constantMassMatrix

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
        return (t, kind.objects[i], q[ltg], q_t[ltg])

    def _Call(self, kind, name, i, q, q_t, t):
        """the result of the user function name of object i of kind, by its recording, as numpy array"""
        return np.array(kind.recordings[name].Evaluate(self.mbs, *self._Arguments(kind, i, q, q_t, t)), dtype=float)

    def _Jacobian(self, kind, name, i, position, q, q_t, t):
        """the Jacobian of the user function name of object i of kind by its argument at position, by Diff of the
        recording (#3115)"""
        recording = kind.recordings[name]
        argumentName = recording.GetFlatProgramData()['arguments'][position]['name']
        return np.array(recording.EvaluateJacobian(argumentName, self.mbs, *self._Arguments(kind, i, q, q_t, t)), dtype=float)

    def _LocalMassMatrix(self, kind, i, q, q_t, t):
        if 'massMatrixUserFunction' in kind.recordings:
            return self._Call(kind, 'massMatrixUserFunction', i, q, q_t, t)
        return kind.parameters[i]['M']

    def _LocalForces(self, kind, i, q, q_t, t):
        """the forces on the right-hand side; for ObjectGenericODE2 f + f_user - K q - D q_t"""
        f = np.zeros(kind.ltg.shape[1])
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

    def Forces(self, q=None, q_t=None, t=0.):
        """the global forces f(q, q_t, t) of the right-hand side, as the ODE2 part of the solver's right-hand side

        Args:
            q: the ODE2 coordinates (displacements), of size nODE2; None: zero
            q_t: the ODE2 velocities; None: zero
            t: the time

        Returns:
            the vector f of size nODE2
        """
        (q, q_t, _) = self._State(q, q_t)
        f = np.zeros(self.nODE2)
        for kind in self.kinds:
            for i in range(len(kind.objects)):
                np.add.at(f, kind.ltg[i], self._LocalForces(kind, i, q, q_t, t))
        return f

    def MassMatrixTriplets(self, q=None, q_t=None, t=0.):
        """the global mass matrix as triplets (rows, columns, values), a value per nonzero local entry; equal rows
        and columns are added when the matrix is formed"""
        (q, q_t, _) = self._State(q, q_t)
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
        return [(kind.ltg, np.array([self._LocalMassMatrix(kind, i, q, q_t, t) for i in range(len(kind.objects))]))
                for kind in self.kinds]

    def Residual(self, q=None, q_t=None, q_tt=None, t=0.):
        """the residual r = M(q) q_tt - f(q, q_t, t) of the ODE2 equations, object by object without forming M"""
        (q, q_t, q_tt) = self._State(q, q_t, q_tt)
        r = np.zeros(self.nODE2)
        for kind in self.kinds:
            for i in range(len(kind.objects)):
                ltg = kind.ltg[i]
                local = self._LocalMassMatrix(kind, i, q, q_t, t) @ q_tt[ltg] - self._LocalForces(kind, i, q, q_t, t)
                np.add.at(r, ltg, local)
        return r

    def ResidualJacobian(self, q=None, q_t=None, q_tt=None, t=0., factor_q=1., factor_q_t=0., factor_q_tt=0.,
                         sparse=False):
        """the Jacobian of the residual, factor_q dr/dq + factor_q_t dr/dq_t + factor_q_tt dr/dq_tt, object by object
        by Diff of the recordings (UserFunction.EvaluateJacobian, #3115): exact, one derivative of the tree per column;
        factor_q_tt=1 alone gives the mass matrix, a Newton step of an implicit integrator combines the three

        Args:
            q: the ODE2 coordinates, of size nODE2; None: zero
            q_t: the ODE2 velocities; None: zero
            q_tt: the ODE2 accelerations; None: zero
            t: the time
            factor_q: the factor of dr/dq
            factor_q_t: the factor of dr/dq_t
            factor_q_tt: the factor of dr/dq_tt
            sparse: True: a scipy.sparse.csr_matrix, else a dense array

        Returns:
            the matrix (nODE2 x nODE2)
        """
        (q, q_t, q_tt) = self._State(q, q_t, q_tt)
        rows, columns, values = [], [], []
        for kind in self.kinds:
            (position_q, position_q_t) = _argumentPositions[kind.objectType]
            for i in range(len(kind.objects)):
                ltg = kind.ltg[i]
                local = np.zeros((len(ltg), len(ltg)))
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
        if not rows:
            return self._Matrix(np.zeros(0, dtype=int), np.zeros(0, dtype=int), np.zeros(0), sparse)
        return self._Matrix(np.concatenate(rows), np.concatenate(columns), np.concatenate(values), sparse)

    def __repr__(self):
        lines = ['SymbolicSystem: ' + str(self.nODE2) + ' ODE2 coordinates, ' + str(len(self.kinds)) + ' kinds']
        for kind in self.kinds:
            lines.append('  ' + kind.objectType + ': ' + str(len(kind.objects)) + ' objects of ' + str(kind.ltg.shape[1])
                         + ' coordinates, user functions ' + str(sorted(kind.recordings)))
        return '\n'.join(lines)


#the explicit solvers of SolveExplicit
explicitSolverTypes = [exudyn.DynamicSolverType.ExplicitEuler, exudyn.DynamicSolverType.RK44,
                       exudyn.DynamicSolverType.VelocityVerlet]

#the Butcher tableaus (A, weights, times) of src/Solver/CSolverExplicit.cpp
_butcherTableaus = {exudyn.DynamicSolverType.ExplicitEuler: ([[0.]], [1.], [0.]),
                    exudyn.DynamicSolverType.RK44: ([[0., 0., 0., 0.], [0.5, 0., 0., 0.], [0., 0.5, 0., 0.], [0., 0., 1., 0.]],
                                                    [1./6., 1./3., 1./3., 1./6.], [0., 0.5, 0.5, 1.])}


class _MassMatrixSolver:
    """the accelerations a = M^-1 f of a system: by the inverse per body, or by the sparse LU decomposition of M; the
    inverse or the decomposition computed once for a constant mass matrix"""
    def __init__(self, system, perBody, reuseConstant):
        self.system = system
        self.perBody = perBody
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
        return scipy.sparse.linalg.splu(self.system.MassMatrix(q, q_t, t, sparse=True).tocsc())

    def Accelerations(self, q, q_t, t, f):
        inverse = self.stored if self.stored is not None else self._Inverse(q, q_t, t)
        if self.constant:
            self.stored = inverse
        if not self.perBody:
            return inverse.solve(f)
        a = np.zeros(len(f))
        for (ltg, inverses) in inverse:
            a[ltg] = np.einsum('kij,kj->ki', inverses, f[ltg])
        return a


class _MbsOutput:
    """the output of a solver of this module through the MainSystem of a SymbolicSystem: an Exudyn solver object, which
    writes the solution file and the sensors and updates the renderer in FinishStep, by simulationSettings.solution; it
    runs as explicit Euler without inverse per body, which only decides what its initialization computes"""
    def __init__(self, system, simulationSettings):
        self.system = system
        self.simulationSettings = exudyn.SimulationSettings()
        self.simulationSettings.SetDictionary(simulationSettings.GetDictionary())
        self.simulationSettings.timeIntegration.solverType = exudyn.DynamicSolverType.ExplicitEuler
        self.simulationSettings.timeIntegration.explicit.computeMassMatrixInversePerBody = False
        simulationSettings = self.simulationSettings
        self.solver = exudyn.MainSolverExplicit()
        self.solver.InitializeSolver(system.mbs, simulationSettings)

    def Step(self, stepIndex, t, h, q, q_t, q_tt):
        self.system.SetSystemState(q, q_t, q_tt, t)
        self.solver.it.currentStepIndex = stepIndex
        self.solver.it.currentTime = t
        self.solver.it.currentStepSize = h
        self.solver.FinishStep(self.system.mbs, self.simulationSettings)

    def Finalize(self):
        self.solver.FinalizeSolver(self.system.mbs, self.simulationSettings)


def SolveExplicit(system, simulationSettings=None, solverType=None):
    """the explicit time integration of a SymbolicSystemBase with constant step size, by explicit Euler, RK44 or
    velocity Verlet, as Exudyn's explicit solver computes them (#3112, design docs/dev/design/symbolicSystem.md §10):
    the steps from timeIntegration.startTime, endTime and numberOfSteps; the accelerations by the inverse per body with
    timeIntegration.explicit.computeMassMatrixInversePerBody, else by the sparse LU decomposition of the mass
    matrix, both once for a constant mass matrix (timeIntegration.reuseConstantMassMatrix); for a SymbolicSystem the
    solution file, the sensors and the renderer of its mbs by simulationSettings.solution, as after SolveDynamic

    Args:
        system: the system, a SymbolicSystemBase
        simulationSettings: exudyn.SimulationSettings; None: the defaults
        solverType: an exudyn.DynamicSolverType of explicitSolverTypes; None: timeIntegration.solverType

    Returns:
        dict with 'time' (n_out), 'q', 'q_t', 'q_tt' (n_out x nODE2) at the steps of solution.file.writePeriod and
        the last step, and the statistics 'steps', 'forceEvaluations', 'massMatrixEvaluations'
    """
    if simulationSettings is None:
        simulationSettings = exudyn.SimulationSettings()
    timeIntegration = simulationSettings.timeIntegration
    if solverType is None:
        solverType = timeIntegration.solverType
    if solverType not in explicitSolverTypes:
        raise ValueError('SolveExplicit: solverType must be one of ' + str(explicitSolverTypes) + ', not ' + str(solverType))
    mbs = getattr(system, 'mbs', None)
    if mbs is not None and timeIntegration.explicit.useLieGroupIntegration:
        for nodeNumber in range(mbs.systemData.NumberOfNodes()):
            if mbs.GetNode(nodeNumber)['nodeType'].endswith('LG'):
                raise ValueError('SolveExplicit: node ' + str(nodeNumber) + ' is a Lie group node, which Exudyn integrates '
                                 'by the Lie group formulas (explicit.useLieGroupIntegration); they are not implemented here')

    n = int(timeIntegration.numberOfSteps)
    (tStart, tEnd) = (timeIntegration.startTime, timeIntegration.endTime)
    h = (tEnd - tStart)/n
    massSolver = _MassMatrixSolver(system, timeIntegration.explicit.computeMassMatrixInversePerBody,
                                   timeIntegration.reuseConstantMassMatrix)
    forceEvaluations = [0]
    def Accelerations(q, q_t, t):
        forceEvaluations[0] += 1
        return massSolver.Accelerations(q, q_t, t, system.Forces(q, q_t, t))

    (q, q_t) = system.InitialState()
    q_tt = np.zeros(system.nODE2)
    if solverType == exudyn.DynamicSolverType.VelocityVerlet:
        q_tt = Accelerations(q, q_t, tStart)

    #the output as CSolverBase writes it: the start, then each time a write period is passed, and the last step
    period = simulationSettings.solution.file.writePeriod
    result = {'time': [], 'q': [], 'q_t': [], 'q_tt': []}
    lastWritten = [tStart]
    def Write(t, last):
        if last:
            lastWritten[0] = tEnd - h
        if t == tStart or t - lastWritten[0] >= -1e-10:
            lastWritten[0] = max(lastWritten[0] + period, t)
            for (name, value) in [('time', t), ('q', q), ('q_t', q_t), ('q_tt', q_tt)]:
                result[name].append(np.copy(value))
    Write(tStart, False)
    output = _MbsOutput(system, simulationSettings) if mbs is not None else None
    if output is not None:
        output.Step(0, tStart, h, q, q_t, q_tt) #the initial state, as SolveDynamic writes it

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
            K, K_t = [], []
            for i in range(len(weights)):
                (g, g_t) = (q.copy(), q_t.copy())
                for j in range(i):
                    if A[i][j] != 0.:
                        g += (A[i][j]*h)*K[j]
                        g_t += (A[i][j]*h)*K_t[j]
                K.append(g_t)
                K_t.append(Accelerations(g, g_t, t0 + h*times[i]))
            for i in range(len(weights)):
                if weights[i] != 0.:
                    q = q + (weights[i]*h)*K[i]
                    q_t = q_t + (weights[i]*h)*K_t[i]
            if timeIntegration.explicit.computeEndOfStepAccelerations:
                q_tt = Accelerations(q, q_t, t)
            else:
                q_tt = K_t[-1]
        Write(t, step == n)
        if output is not None:
            output.Step(step, t, h, q, q_t, q_tt)
    if output is not None:
        output.Finalize()

    result = {name: np.array(value) for (name, value) in result.items()}
    result.update({'steps': n, 'forceEvaluations': forceEvaluations[0], 'massMatrixEvaluations': massSolver.evaluations})
    return result
