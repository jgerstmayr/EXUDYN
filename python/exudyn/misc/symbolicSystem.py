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
#           or velocity Verlet, as Exudyn's explicit solver does (#3112), SolveGeneralizedAlpha by the generalized-alpha
#           method, as Exudyn's implicit solver does (#3126).
#           Special: not for the broad use, a basis for the export of the equations of motion and their evaluation
#           outside Exudyn. The items it takes: ObjectUserBody with symbolic massMatrixUserFunction and
#           forceUserFunction, ObjectGenericODE2 with constant matrices and symbolic user functions,
#           ObjectConnectorUser with a symbolic forceUserFunction on position markers of point nodes, on position and
#           rigid markers of the ground and of user bodies, with the rotations if it requests them (#3121, #3129), items
#           without coordinates (ground), and the loads LoadForceVector, LoadTorqueVector,
#           LoadMassProportional and LoadCoordinate, constant or with a symbolic user function of the time, on markers
#           of point nodes, node coordinates and user bodies (#3118); every other object and load, constraints and
#           recordings that read mbs are refused.
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
try:
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
        Its members: objectType, the type name 'UserBody', 'GenericODE2' or 'ConnectorUser'; recordings, the dict of the
        recorded user functions by name (a name without recording is missing); objects, the list of the object numbers;
        ltg, the array (number of objects x local coordinates) of the global ODE2 indices; parameters, the list of the
        parameter vectors of the objects (UserBody), of the dicts of the constant matrices and the force vector
        (GenericODE2), or of the dicts of the parameter vector, 'orientation' (the connector gets the rotations and
        returns the torques) and the markers (ConnectorUser: per marker a dict of its 'type' - 'fixed', 'point' or
        'body' -, its local coordinate indices 'indices', its reference 'position' and 'rotation' - for a marker on a
        user body its local position and rotation, 'objectNumber', 'qRef', 'parameters' and the 'recordings' of the
        body that give its kinematics)
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
        bodyFixed, else None)
    """
    def __init__(self, loadNumber, loadType, ltg, value, recording, body=None):
        self.loadNumber = loadNumber
        self.loadType = loadType
        self.ltg = np.array(ltg, dtype=int)
        self.value = value
        self.recording = recording
        self.body = body


class SymbolicSystem(SymbolicSystemBase):
    """the ODE2 equations of motion of mbs, by kinds of objects with symbolic user functions, evaluated by
    exudyn.symbolic (#3110, #3113); mbs must be assembled; see the Details of this module for the items it takes

    Args:
        mbs: the assembled MainSystem

    Note:
        Its members: mbs; kinds, the list of SymbolicKind; loads, the list of SymbolicLoad; nODE2, the number of ODE2
        coordinates of the system; qRef, the reference values of the ODE2 coordinates
    """
    def __init__(self, mbs):
        if not mbs.systemIsConsistent:
            raise ValueError('SymbolicSystem: mbs must be assembled first')
        self.mbs = mbs
        self.nODE2 = mbs.systemData.ODE2Size()
        self.qRef = np.array(mbs.systemData.GetODE2Coordinates(configuration=exudyn.ConfigurationType.Reference))
        refused = []
        self.loads = []
        for loadNumber in range(mbs.systemData.NumberOfLoads()):
            (load, reason) = self._Load(loadNumber)
            if reason:
                refused += ['load ' + str(loadNumber) + ' (' + mbs.GetLoad(loadNumber)['loadType'] + ': ' + reason + ')']
            elif load is not None:
                self.loads.append(load)
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
        if refused:
            raise ValueError('SymbolicSystem: these items cannot be assembled (yet): ' + ', '.join(refused))
        self.kinds = list(kinds.values())
        for kind in self.kinds:
            kind.ltg = np.array(kind.ltg, dtype=int)

    def _ConnectorMarkers(self, objectNumber, item):
        """the markers of an ObjectConnectorUser as the dicts of SymbolicKind, the global indices of the coordinates of
        the connector, and the reason it is refused, empty if it is taken"""
        mbs = self.mbs
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
        for load in self.loads:
            np.add.at(f, load.ltg, self._LoadForces(load, q, q_t, t))
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
                for kind in self.kinds if kind.objectType != 'ConnectorUser']

    def Residual(self, q=None, q_t=None, q_tt=None, t=0.):
        """the residual r = M(q) q_tt - f(q, q_t, t) of the ODE2 equations, object by object without forming M"""
        (q, q_t, q_tt) = self._State(q, q_t, q_tt)
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

    def __repr__(self):
        lines = ['SymbolicSystem: ' + str(self.nODE2) + ' ODE2 coordinates, ' + str(len(self.kinds)) + ' kinds']
        for kind in self.kinds:
            lines.append('  ' + kind.objectType + ': ' + str(len(kind.objects)) + ' objects of ' + str(kind.ltg.shape[1])
                         + ' coordinates, user functions ' + str(sorted(kind.recordings)))
        if self.loads:
            lines.append('  loads: ' + ', '.join(load.loadType + (' (user function)' if load.recording is not None else '')
                                                for load in self.loads))
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
        self.system = system
        self.solver = None
        if getattr(system, 'mbs', None) is not None:
            self.simulationSettings = exudyn.SimulationSettings()
            self.simulationSettings.SetDictionary(simulationSettings.GetDictionary())
            self.simulationSettings.timeIntegration.solverType = exudyn.DynamicSolverType.ExplicitEuler
            self.simulationSettings.timeIntegration.explicit.computeMassMatrixInversePerBody = False
            self.solver = exudyn.MainSolverExplicit()
            self.solver.InitializeSolver(system.mbs, self.simulationSettings)

    def Step(self, stepIndex, t, q, q_t, q_tt, last=False):
        """the state after step stepIndex at t; stepIndex 0 is the initial state, as SolveDynamic writes it"""
        if last:
            self.lastWritten = self.tEnd - self.h
        if stepIndex == 0 or t - self.lastWritten >= -1e-10:
            self.lastWritten = max(self.lastWritten + self.period, t)
            for (name, value) in [('time', t), ('q', q), ('q_t', q_t), ('q_tt', q_tt)]:
                self.result[name].append(np.copy(value))
        if self.solver is not None:
            self.system.SetSystemState(q, q_t, q_tt, t)
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
    _RefuseLieGroupNodes(system, timeIntegration.explicit.useLieGroupIntegration, 'SolveExplicit')

    (n, tStart, tEnd, h) = _Steps(simulationSettings)
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
    output = _Output(system, simulationSettings, tStart, tEnd, h)
    output.Step(0, tStart, q, q_t, q_tt)

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
        output.Step(step, t, q, q_t, q_tt, last=(step == n))

    return output.Finalize({'steps': n, 'forceEvaluations': forceEvaluations[0],
                            'massMatrixEvaluations': massSolver.evaluations})


def SolveGeneralizedAlpha(system, simulationSettings=None):
    """the implicit time integration of a SymbolicSystemBase with constant step size by the generalized-alpha method
    in the form of Arnold and Bruls, as Exudyn's MainSolverImplicitSecondOrder computes it (#3126, design
    docs/dev/design/symbolicSystem.md §6): per step the predictor of the displacements, velocities and algorithmic
    accelerations, then Newton's method for the accelerations with the residual r = M q_tt - f and the Jacobian
    beta' M + gamma' dr/dq_t + dr/dq of ResidualJacobian; by timeIntegration: numberOfSteps, startTime, endTime,
    generalizedAlpha.spectralRadius, useNewmark with newmarkBeta and newmarkGamma, computeInitialAccelerations, and
    newton.relativeTolerance, absoluteTolerance, maxIterations, useModifiedNewton (the Jacobian once per step) and
    weightTolerancePerCoordinate; for a SymbolicSystem the solution file, the sensors and the renderer of its mbs, as
    SolveExplicit. A step whose Newton iteration does not converge raises: the step size is constant

    Args:
        system: the system, a SymbolicSystemBase
        simulationSettings: exudyn.SimulationSettings; None: the defaults

    Returns:
        dict with 'time' (n_out), 'q', 'q_t', 'q_tt' (n_out x nODE2) at the steps of solution.file.writePeriod and
        the last step, and the statistics 'steps', 'newtonIterations', 'jacobianEvaluations'
    """
    if simulationSettings is None:
        simulationSettings = exudyn.SimulationSettings()
    timeIntegration = simulationSettings.timeIntegration
    generalizedAlpha = timeIntegration.generalizedAlpha
    newton = timeIntegration.newton
    _RefuseLieGroupNodes(system, True, 'SolveGeneralizedAlpha') #the implicit solver of Exudyn integrates them always so
    if generalizedAlpha.useIndex2Constraints:
        raise ValueError('SolveGeneralizedAlpha: useIndex2Constraints is not implemented, as the system has no constraints yet')

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
    errorFactor = np.sqrt(system.nODE2) if (newton.weightTolerancePerCoordinate and system.nODE2) else 1.

    (q, q_t) = system.InitialState()
    q_tt = np.zeros(system.nODE2)
    if generalizedAlpha.computeInitialAccelerations:
        q_tt = scipy.sparse.linalg.spsolve(system.MassMatrix(q, q_t, tStart, sparse=True).tocsc(), system.Forces(q, q_t, tStart))
    aAlgorithmic = q_tt.copy()
    output = _Output(system, simulationSettings, tStart, tEnd, h)
    output.Step(0, tStart, q, q_t, q_tt)

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
        q_tt = np.zeros(system.nODE2)

        #Newton's method, as CSolverBase::Newton: converged by the residual relative to the initial one or absolute
        initialResidual = cMath.norm(system.Residual(q, q_t, q_tt, t))/errorFactor
        if initialResidual <= newton.absoluteTolerance:
            initialResidual = 2*newton.absoluteTolerance #at least one Newton step
        factorization = None
        converged = False
        for iteration in range(int(newton.maxIterations)):
            if factorization is None or not newton.useModifiedNewton:
                factorization = scipy.sparse.linalg.splu(system.ResidualJacobian(q, q_t, q_tt, t, factor_q=1., factor_q_t=gammaPrime,
                                                                                 factor_q_tt=betaPrime, sparse=True).tocsc())
                jacobianEvaluations += 1
            increment = factorization.solve(system.Residual(q, q_t, q_tt, t))
            q = q - increment
            q_t = q_t - gammaPrime*increment
            q_tt = q_tt - betaPrime*increment
            newtonIterations += 1
            residual = cMath.norm(system.Residual(q, q_t, q_tt, t))/errorFactor
            if residual/initialResidual <= newton.relativeTolerance or residual <= newton.absoluteTolerance:
                converged = True
                break
        if not converged:
            raise ValueError('SolveGeneralizedAlpha: Newton did not converge in step ' + str(step) + ' at t=' + str(t)
                             + ' after ' + str(int(newton.maxIterations)) + ' iterations; residual ' + str(residual))
        if not generalizedAlpha.useNewmark:
            aAlgorithmic = aAlgorithmic + ((1 - alphaF)/(1 - alphaM))*q_tt #FinalizeNewton
        output.Step(step, t, q, q_t, q_tt, last=(step == n))

    return output.Finalize({'steps': n, 'newtonIterations': newtonIterations, 'jacobianEvaluations': jacobianEvaluations})
