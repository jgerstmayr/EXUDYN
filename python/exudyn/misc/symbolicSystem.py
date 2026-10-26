#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  The equations of motion of a system as symbolic programs (#3110, design: docs/dev/design/symbolicSystem.md):
#           SymbolicSystem(mbs) collects, after mbs.Assemble(), the objects with symbolic user functions in kinds -
#           the objects whose user functions are the same recordings -, each with the table of its objects, their
#           parameters and the global indices of their ODE2 coordinates (the local-to-global maps of systemData).
#           From them it evaluates the residual of the ODE2 equations r = M(q) q_tt - f(q, q_t, t), the mass matrix
#           M (sparse, as triplets) and the forces f, by exudyn.symbolic or by numpy code of the recordings.
#           Special: not for the broad use, a basis for the export of the equations of motion and their evaluation
#           outside Exudyn. The items it takes: ObjectUserBody with symbolic massMatrixUserFunction and
#           forceUserFunction, ObjectGenericODE2 with constant matrices and symbolic user functions, and items without
#           coordinates (ground); every other object, loads and constraints are refused.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-26 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import exudyn
from exudyn.misc import symbolicExport

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = ['evaluators', 'SymbolicKind', 'SymbolicSystem']

#the evaluators of the recordings: exudyn.symbolic (the recording itself) or the numpy code written from it
evaluators = ['symbolic', 'numpy']

#the user functions of a kind, by object type; the first ones give the mass matrix and the forces
_userFunctionNames = {'UserBody': ['massMatrixUserFunction', 'forceUserFunction'],
                      'GenericODE2': ['massMatrixUserFunction', 'forceUserFunction']}


def _Recording(value, objectNumber, name):
    """the recorded symbolic user function of a parameter, None if there is none; a Python function raises"""
    if isinstance(value, dict) and value.get('type') == 'Symbolic':
        return value['function']
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
    """the objects of one object type whose user functions are the same recordings: one program per user function,
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
        self._functions = {} #the numpy functions of the recordings, written when first used

    def Function(self, name, evaluator):
        """the function that evaluates the recording name, with the arguments of the user function"""
        recording = self.recordings[name]
        if evaluator == 'symbolic':
            return recording.Evaluate
        if name not in self._functions:
            self._functions[name] = symbolicExport.ToFunction(recording.GetFlatProgramData(), format='numpy')
        return self._functions[name]


class SymbolicSystem:
    """the ODE2 equations of motion of mbs, by kinds of objects with symbolic user functions (#3110); mbs must be
    assembled; see the Details of this module for the items it takes

    Args:
        mbs: the assembled MainSystem
        evaluator: 'symbolic' (the recordings) or 'numpy' (numpy code written from them)

    Note:
        Its members: kinds, the list of SymbolicKind; nODE2, the number of ODE2 coordinates of the system; qRef, the
        reference values of the ODE2 coordinates
    """
    def __init__(self, mbs, evaluator='symbolic'):
        if evaluator not in evaluators:
            raise ValueError('SymbolicSystem: evaluator must be one of ' + str(evaluators) + ', not ' + repr(evaluator))
        if not mbs.systemIsConsistent:
            raise ValueError('SymbolicSystem: mbs must be assembled first')
        self.mbs = mbs
        self.evaluator = evaluator
        self.nODE2 = mbs.systemData.ODE2Size()
        self.qRef = np.array(mbs.systemData.GetODE2Coordinates(configuration=exudyn.ConfigurationType.Reference))
        refused = []
        if mbs.systemData.NumberOfLoads() > 0:
            refused += ['loads']
        kinds = {}
        for objectNumber in range(mbs.systemData.NumberOfObjects()):
            item = mbs.GetObject(objectNumber)
            objectType = item['objectType']
            ltg = list(mbs.systemData.GetObjectLTGODE2(objectNumber))
            if objectType not in _userFunctionNames:
                if len(ltg) or mbs.systemData.GetObjectLTGAE(objectNumber):
                    refused += ['object ' + str(objectNumber) + ' (' + objectType + ')']
                continue
            recordings = {}
            for name in _userFunctionNames[objectType]:
                recording = _Recording(item[name], objectNumber, name)
                if recording is not None:
                    recordings[name] = recording
            if objectType == 'UserBody' and 'massMatrixUserFunction' not in recordings:
                raise ValueError('SymbolicSystem: object ' + str(objectNumber) + ' (UserBody) has no massMatrixUserFunction')
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

    def _Arguments(self, kind, i, q, q_t, t):
        """the arguments of the user functions of object i of kind, without mbs"""
        ltg = kind.ltg[i]
        if kind.objectType == 'UserBody':
            qRef = self.qRef[ltg]
            return (t, kind.objects[i], qRef, qRef + q[ltg], q_t[ltg], kind.parameters[i])
        return (t, kind.objects[i], q[ltg], q_t[ltg])

    def _Call(self, kind, name, i, q, q_t, t):
        """the result of the user function name of object i of kind, as numpy array"""
        return np.array(kind.Function(name, self.evaluator)(self.mbs, *self._Arguments(kind, i, q, q_t, t)), dtype=float)

    def _LocalMassMatrix(self, kind, i, q, q_t, t):
        if 'massMatrixUserFunction' in kind.recordings:
            return self._Call(kind, 'massMatrixUserFunction', i, q, q_t, t)
        return kind.parameters[i]['M']

    def _LocalForces(self, kind, i, q, q_t, t):
        """the forces on the right-hand side; for ObjectGenericODE2 f + f_user - K q - D q_t"""
        n = kind.ltg.shape[1]
        f = np.zeros(n)
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

    def _State(self, q, q_t):
        q = np.zeros(self.nODE2) if q is None else np.asarray(q, dtype=float)
        q_t = np.zeros(self.nODE2) if q_t is None else np.asarray(q_t, dtype=float)
        return q, q_t

    def Forces(self, q=None, q_t=None, t=0.):
        """the global forces f(q, q_t, t) of the right-hand side, as the ODE2 part of the solver's right-hand side

        Args:
            q, q_t: the ODE2 coordinates (displacements) and their velocities, of size nODE2; None: zero
            t: the time
        """
        q, q_t = self._State(q, q_t)
        f = np.zeros(self.nODE2)
        for kind in self.kinds:
            for i in range(len(kind.objects)):
                np.add.at(f, kind.ltg[i], self._LocalForces(kind, i, q, q_t, t))
        return f

    def MassMatrixTriplets(self, q=None, q_t=None, t=0.):
        """the global mass matrix as triplets (rows, columns, values), a value per nonzero local entry; equal rows
        and columns are added when the matrix is formed"""
        q, q_t = self._State(q, q_t)
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

    def MassMatrix(self, q=None, q_t=None, t=0., sparse=False):
        """the global mass matrix M(q, t); sparse=True: a scipy.sparse.csr_matrix (scipy needed), else a dense array"""
        (rows, columns, values) = self.MassMatrixTriplets(q, q_t, t)
        if sparse:
            try:
                import scipy.sparse
            except ImportError:
                raise ImportError('SymbolicSystem.MassMatrix(sparse=True) needs scipy; use sparse=False or MassMatrixTriplets()')
            return scipy.sparse.csr_matrix((values, (rows, columns)), shape=(self.nODE2, self.nODE2))
        M = np.zeros((self.nODE2, self.nODE2))
        np.add.at(M, (rows, columns), values)
        return M

    def Residual(self, q=None, q_t=None, q_tt=None, t=0.):
        """the residual r = M(q) q_tt - f(q, q_t, t) of the ODE2 equations, object by object without forming M"""
        q, q_t = self._State(q, q_t)
        q_tt = np.zeros(self.nODE2) if q_tt is None else np.asarray(q_tt, dtype=float)
        r = np.zeros(self.nODE2)
        for kind in self.kinds:
            for i in range(len(kind.objects)):
                ltg = kind.ltg[i]
                local = self._LocalMassMatrix(kind, i, q, q_t, t) @ q_tt[ltg] - self._LocalForces(kind, i, q, q_t, t)
                np.add.at(r, ltg, local)
        return r

    def __repr__(self):
        lines = ['SymbolicSystem: ' + str(self.nODE2) + ' ODE2 coordinates, ' + str(len(self.kinds)) + ' kinds']
        for kind in self.kinds:
            lines.append('  ' + kind.objectType + ': ' + str(len(kind.objects)) + ' objects of ' + str(kind.ltg.shape[1])
                         + ' coordinates, user functions ' + str(sorted(kind.recordings)))
        return '\n'.join(lines)
