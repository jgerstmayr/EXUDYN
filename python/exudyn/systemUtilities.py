#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  The functions of a whole system (#2997): quantities of the system from the contributions of its items -
#           SystemQuantity with SystemEnergy and SystemConstraintDrift, their sensors -, the residuals of the
#           constraints, the numerical Jacobian of a computation of the current state, and the system graph.
#           'from exudyn.utilities import *' includes this module.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-16 (created; the functions came from advancedUtilities, solver and mainSystemExtensions)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np

import exudyn
import exudyn.itemInterface
from exudyn.misc.extensionRegistry import extends

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'SystemQuantity', 'LoadPotentialEnergy', 'SystemEnergy', 'SystemConstraintDrift',
    'ComputeConstraintResiduals', 'NumericalJacobian', 'ComputeSystemGraph', 'DrawSystemGraph',
    ]


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#QUANTITIES OF THE WHOLE SYSTEM
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def _ItemKey(item):
    """the kind and number of an item index, to compare item indices of different kinds"""
    return (type(item).__name__, int(item))


class SystemQuantity:
    """a quantity of the whole system with named components - an energy, a drift - from the contributions of its items;
    the base class of SystemEnergy and SystemConstraintDrift, which define the components and how an item contributes

    Args:
        mbs: the MainSystem, assembled
        items: None for all items that contribute, or a list of the items to take into account (ObjectIndex,
            NodeIndex, LoadIndex)
        skipUnavailable: if True, an item that has the quantity but cannot compute it with its parameters (a user
            function) is left out and listed in .unavailable; if False, it raises

    Note:
        The contributing items are collected once, when the object is created: .items, and .unavailable, a list of
        (item, type name, component, reason); call Refresh() after the system changed. Compute() gives one value per
        component, in the order of .names; ComputeItems() the contribution of each item; AddSensor() adds a
        SensorUserFunction that records the components - call mbs.Assemble() after it.
    """
    names = []  #the components, defined by the derived class

    def __init__(self, mbs, items=None, skipUnavailable=True):
        self.mbs = mbs
        self.selectedItems = None if items is None else set(_ItemKey(item) for item in items)
        self.skipUnavailable = skipUnavailable
        self.Refresh()

    def Refresh(self):
        """collect the contributing items again, after the system changed"""
        self.items = []
        self.unavailable = []
        self._CollectItems()

    def _Selected(self, item):
        """True if the item is to be taken into account"""
        return self.selectedItems is None or _ItemKey(item) in self.selectedItems

    def _Unavailable(self, item, typeName, component, reason):
        """record an item that has the component but cannot compute it, or raise if skipUnavailable=False"""
        if not self.skipUnavailable:
            raise NotImplementedError(type(self).__name__ + ': ' + component + ' of ' + typeName + ' ' + str(item)
                                      + ' is not available: ' + reason)
        self.unavailable.append((item, typeName, component, reason))

    def _CollectItems(self):
        """fill .items and .unavailable; implemented by the derived class"""
        raise NotImplementedError('SystemQuantity: _CollectItems is implemented by the derived class')

    def _ItemContribution(self, item, configuration):
        """the contribution of one item, one value per component; implemented by the derived class"""
        raise NotImplementedError('SystemQuantity: _ItemContribution is implemented by the derived class')

    def _Reduce(self, contributions):
        """the values of the system from the contributions of the items: their sum"""
        if len(contributions) == 0:
            return np.zeros(len(self.names))
        return np.sum(contributions, axis=0)

    def ComputeItems(self, configuration=exudyn.ConfigurationType.Current):
        """the contribution of each item

        Args:
            configuration: the configuration in which the items are evaluated

        Returns:
            a list of (item, numpy array with one value per component)
        """
        return [(item, self._ItemContribution(item, configuration)) for item in self.items]

    def Compute(self, configuration=exudyn.ConfigurationType.Current):
        """the values of the system

        Args:
            configuration: the configuration in which the items are evaluated

        Returns:
            a numpy array with one value per component, in the order of .names
        """
        return self._Reduce([values for (item, values) in self.ComputeItems(configuration)])

    def SensorFunction(self, mbs, t, sensorNumbers, factors, configuration):
        """a sensorUserFunction for SensorUserFunction: Compute(configuration) as a list

        Args:
            mbs: the MainSystem, the one of this object
            t: the time, unused
            sensorNumbers: the sensorNumbers of the SensorUserFunction, unused
            factors: the factors of the SensorUserFunction, unused
            configuration: the configuration in which the items are evaluated

        Returns:
            the list of the values, one per component
        """
        return list(self.Compute(configuration))

    def AddSensor(self, components=None, storeInternal=True, writeToFile=False, fileName='', name=''):
        """add a SensorUserFunction that records the components; call mbs.Assemble() after it

        Args:
            components: None for all components, or a list of names of .names
            storeInternal: store the values in the sensor
            writeToFile: write the values to a file
            fileName: the file, if written
            name: the name of the sensor

        Returns:
            the SensorIndex of the new sensor

        Example:
            sEnergy = SystemEnergy(mbs).AddSensor(components=['total'])
            mbs.Assemble()
        """
        if components is None:
            function = self.SensorFunction
        else:
            for component in components:
                if component not in self.names:
                    raise ValueError(type(self).__name__ + '.AddSensor: unknown component ' + repr(component)
                                     + '; the components are ' + str(self.names))
            indices = [self.names.index(component) for component in components]
            def function(mbs, t, sensorNumbers, factors, configuration):
                values = self.Compute(configuration)
                return [values[i] for i in indices]
        return self.mbs.AddSensor(exudyn.itemInterface.SensorUserFunction(sensorUserFunction=function, name=name,
                                  storeInternal=storeInternal, writeToFile=writeToFile, fileName=fileName))


#the bodies whose mass and center of mass a mass-proportional load can use: the type, and a function that
#returns (mass, local position of the center of mass)
_massOfBody = {
    'MassPoint': lambda d: (d['mass'], [0, 0, 0]),
    'MassPoint2D': lambda d: (d['mass'], [0, 0, 0]),
    'Mass1D': lambda d: (d['mass'], [0, 0, 0]),
    'RigidBody': lambda d: (d['mass'], list(d['centerOfMass'])),
    'RigidBody2D': lambda d: (d['mass'], list(d['centerOfMass']) + [0]),
    }


def LoadPotentialEnergy(mbs, loadNumber, configuration=exudyn.ConfigurationType.Current):
    """the potential energy of a constant load, zero in the reference configuration: $-\\fv\\tp \\uv$ of the
    displacement $\\uv$ of its marker point; None if the load has none that can be computed

    Args:
        mbs: the MainSystem of the load
        loadNumber: the load, a LoadIndex
        configuration: the configuration its marker is evaluated in

    Returns:
        the potential energy as a float, or None for a load whose potential is not available: a load with a
        user function (it may depend on time), a body-fixed load, a torque (not conservative in 3D), a
        mass-proportional load on a body other than a mass point, a 1D mass or a rigid body

    Note:
        Covered are ForceVector and Coordinate loads on any marker that gives a position or a coordinate,
        and MassProportional loads on the bodies named above.
    """
    R = exudyn.ConfigurationType.Reference
    d = mbs.GetLoad(loadNumber)
    loadType = d['loadType']
    if d.get('loadVectorUserFunction', 0) != 0 or d.get('loadUserFunction', 0) != 0 or d.get('bodyFixed', False):
        return None
    marker = d['markerNumber']
    if loadType == 'ForceVector':
        f = np.array(d['loadVector'])
        u = mbs.GetMarkerOutput(marker, exudyn.OutputVariableType.Position, configuration) - \
            mbs.GetMarkerOutput(marker, exudyn.OutputVariableType.Position, R)
        return float(-f @ u)
    if loadType == 'Coordinate':
        q = mbs.GetMarkerOutput(marker, exudyn.OutputVariableType.Coordinates, configuration)
        return float(-d['load'] * q[0])
    if loadType == 'MassProportional':
        markerData = mbs.GetMarker(marker)
        if markerData['markerType'] != 'BodyMass':
            return None
        body = markerData['bodyNumber']
        bodyData = mbs.GetObject(body)
        if bodyData['objectType'] not in _massOfBody:
            return None
        (mass, com) = _massOfBody[bodyData['objectType']](bodyData)
        u = mbs.GetObjectOutputBody(body, exudyn.OutputVariableType.Position, com, configuration) - \
            mbs.GetObjectOutputBody(body, exudyn.OutputVariableType.Position, com, R)
        return float(-mass * np.array(d['loadVector']) @ u)
    return None


class SystemEnergy(SystemQuantity):
    """the energies of the whole system (#2202): the output variables KineticEnergy and PotentialEnergy of its objects
    and the potential of its constant loads, LoadPotentialEnergy; components 'kinetic', 'potential' (of the objects),
    'loads' and 'total', their sum

    Args:
        mbs: the MainSystem, assembled
        items: None for all objects and loads, or a list of the ObjectIndex and LoadIndex to take into account
        skipUnavailable: if True, an object that has an energy but cannot compute it with its parameters (a user
            function) and a load without a potential are left out and listed in .unavailable; if False, they raise

    Note:
        The kinetic energy of a body computed from its mass matrix exists in the current configuration only.

    Example:
        energy = SystemEnergy(mbs)
        [kinetic, potential, loads, total] = energy.Compute()
        sEnergy = energy.AddSensor()
        mbs.Assemble()
    """
    names = ['kinetic', 'potential', 'loads', 'total']

    def _CollectItems(self):
        mbs = self.mbs
        self.energies = {}  #(kind, number) -> [is a body, the energies the object has, of KineticEnergy, PotentialEnergy]
        KE = exudyn.OutputVariableType.KineticEnergy
        PE = exudyn.OutputVariableType.PotentialEnergy
        for i in range(mbs.systemData.NumberOfObjects()):
            o = exudyn.ObjectIndex(i)
            if not self._Selected(o):
                continue
            isBody = exudyn.ObjectType.Body in mbs.Inspect(o, exudyn.InspectType.ObjectType)
            GetOutput = mbs.GetObjectOutputBody if isBody else mbs.GetObjectOutput
            #Inspect lists an energy only where it can be computed with the current parameters
            available = mbs.Inspect(o, exudyn.InspectType.OutputVariables)
            variables = []
            for (variable, component) in [(KE, 'kinetic'), (PE, 'potential')]:
                if variable in available:
                    variables.append(variable)
                    continue
                #not listed: the type has no such energy (ValueError), or has it but not with these parameters - then
                #reading it raises NotImplementedError with the reason, which is what .unavailable reports
                try:
                    GetOutput(o, variable)
                except NotImplementedError as error:
                    self._Unavailable(o, mbs.GetObject(o)['objectType'], component, str(error).splitlines()[0])
                except ValueError:
                    pass
            if len(variables):
                self.items.append(o)
                self.energies[_ItemKey(o)] = (isBody, variables)
        for i in range(mbs.systemData.NumberOfLoads()):
            load = exudyn.LoadIndex(i)
            if not self._Selected(load):
                continue
            if LoadPotentialEnergy(mbs, load) is None:
                self._Unavailable(load, mbs.GetLoad(load)['loadType'], 'loads', 'no potential energy for this load')
            else:
                self.items.append(load)

    def _ItemContribution(self, item, configuration):
        if isinstance(item, exudyn.LoadIndex):
            V = LoadPotentialEnergy(self.mbs, item, configuration)
            return np.array([0., 0., V, V])
        (isBody, variables) = self.energies[_ItemKey(item)]
        GetOutput = self.mbs.GetObjectOutputBody if isBody else self.mbs.GetObjectOutput
        values = np.zeros(4)
        for variable in variables:
            k = 0 if variable == exudyn.OutputVariableType.KineticEnergy else 1
            values[k] = float(GetOutput(item, variable, configuration=configuration))
        values[3] = values[0] + values[1]
        return values


class SystemConstraintDrift(SystemQuantity):
    """the drift of the constraints of the whole system (#565): components 'positionDrift', the largest absolute
    residual of the position-level constraints (ComputeItemType.ConstraintDrift), and 'velocityResidual', the largest
    absolute residual of the equations at velocity level (ComputeItemType.AlgebraicEquations_t); the items are the
    objects and nodes with algebraic equations, e.g. joints and Euler parameter nodes

    Args:
        mbs: the MainSystem, assembled
        items: None for all constraints and nodes with algebraic equations, or a list of the ObjectIndex and NodeIndex
            to take into account
        skipUnavailable: if True, an item without the velocity level (ObjectJointSliding, ObjectJointSliding2D) gives
            NaN for velocityResidual and is listed in .unavailable; if False, it raises

    Note:
        The residuals are those of the current state; the configuration of Compute and of the sensor must be the
        current one. The residual vectors themselves: mbs.ComputeConstraintResiduals() for the system,
        mbs.ComputeItem(item, ComputeItemType.ConstraintDrift) for one item. The drift check of the solver,
        timeIntegration.generalizedAlpha.checkConstraintDrift, uses the same definition of the drift.

    Example:
        drift = SystemConstraintDrift(mbs)
        sDrift = drift.AddSensor()
        mbs.Assemble()
        mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)
        print('largest drift', max(mbs.GetSensorStoredData(sDrift)[:,1]))
    """
    names = ['positionDrift', 'velocityResidual']

    def _CollectItems(self):
        mbs = self.mbs
        self.hasVelocityLevel = {}
        D = exudyn.ComputeItemType.ConstraintDrift
        V = exudyn.ComputeItemType.AlgebraicEquations_t
        candidates = [exudyn.ObjectIndex(i) for i in range(mbs.systemData.NumberOfObjects())] + \
                     [exudyn.NodeIndex(i) for i in range(mbs.systemData.NumberOfNodes())]
        for item in candidates:
            if not self._Selected(item):
                continue
            what = mbs.ComputeItem(item)
            if D not in what:
                continue
            self.items.append(item)
            self.hasVelocityLevel[_ItemKey(item)] = V in what
            if V not in what:
                typeName = (mbs.GetObject(item)['objectType'] if isinstance(item, exudyn.ObjectIndex)
                            else mbs.GetNode(item)['nodeType'])
                self._Unavailable(item, typeName, 'velocityResidual', 'no algebraic equations at velocity level')

    def _ItemContribution(self, item, configuration):
        if configuration != exudyn.ConfigurationType.Current:
            raise ValueError('SystemConstraintDrift: the residuals exist in the current configuration only')
        drift = self.mbs.ComputeItem(item, exudyn.ComputeItemType.ConstraintDrift)
        values = [float(np.max(np.abs(drift))) if len(drift) else 0., float('nan')]
        if self.hasVelocityLevel[_ItemKey(item)]:
            residual = self.mbs.ComputeItem(item, exudyn.ComputeItemType.AlgebraicEquations_t)
            values[1] = float(np.max(np.abs(residual))) if len(residual) else 0.
        return np.array(values)

    def _Reduce(self, contributions):
        """the largest value of the items; NaN for velocityResidual if an item does not provide it"""
        if len(contributions) == 0:
            return np.zeros(len(self.names))
        return np.max(contributions, axis=0)


@extends(exudyn.MainSystem)
def ComputeConstraintResiduals(mbs, velocityLevel = False, autoAssemble = True) -> list:
    """the residuals of all algebraic equations of the system in the current state - the drift of the constraints (#565);
    computed per item with mbs.ComputeItem, without a solver, so that it can be called at any time, e.g., in a
    PreStepUserFunction or a SensorUserFunction during a simulation

    Args:
        mbs: the MainSystem
        velocityLevel: False: the position-level constraints, ComputeItemType.ConstraintDrift - zero for the equations of
            velocity-level constraints (rolling disc, CoordinateConstraint with velocityLevel=True), which do not drift;
            True: the equations at velocity level, ComputeItemType.AlgebraicEquations_t
        autoAssemble: if True and mbs.systemIsConsistent=False, mbs.Assemble() is called first

    Returns:
        :list: [residuals, items], residuals is a numpy array with one entry per algebraic equation of the system (the
        numbering of the Lagrange multipliers, mbs.systemData.GetAECoordinates()), items the ObjectIndex or NodeIndex
        of each entry

    Example:
        [residuals, items] = mbs.ComputeConstraintResiduals()
        k = np.argmax(abs(residuals))
        print('largest drift', residuals[k], 'in', items[k])
    """
    if not mbs.systemIsConsistent:
        if not autoAssemble:
            raise ValueError('ComputeConstraintResiduals: the system is not assembled; call mbs.Assemble() first')
        mbs.Assemble()
    what = exudyn.ComputeItemType.AlgebraicEquations_t if velocityLevel else exudyn.ComputeItemType.ConstraintDrift
    residuals = np.zeros(len(mbs.systemData.GetAECoordinates()))
    items = [None]*len(residuals)
    for i in range(mbs.systemData.NumberOfObjects()):
        item = exudyn.ObjectIndex(i)
        #the objects with algebraic equations of their own: a body lists those of its nodes here too
        if what not in mbs.ComputeItem(item):
            continue
        values = mbs.ComputeItem(item, what)
        for (k, value) in zip(list(mbs.systemData.GetObjectLTGAE(i)), values):
            residuals[k] += value
            items[k] = item
    for i in range(mbs.systemData.NumberOfNodes()):
        node = exudyn.NodeIndex(i)
        if what not in mbs.ComputeItem(node): #the nodes with algebraic equations: Euler parameters
            continue
        values = mbs.ComputeItem(node, what)
        start = mbs.GetNodeAEIndex(i)
        for (k, value) in enumerate(values):
            residuals[start + k] += value
            items[start + k] = node
    return [residuals, items]


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#NUMERICAL DERIVATIVES OF COMPUTATIONS OF THE STATE (#2779)
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def NumericalJacobian(mbs, function, systemCoordinates, velocityLevel=False, epsilon=1e-8):
    """the derivative of a computation of the current state of the system - an output variable, mbs.ComputeItem, a
    user function - by some of the system ODE2 coordinates or their velocities, by central differences; for tests of
    the Jacobians of an item, in particular of a user item, against mbs.ComputeItem

    Args:
        mbs: the MainSystem, assembled
        function: a function without arguments that computes the values from the current state, e.g.
            lambda: mbs.ComputeItem(oBody, exu.ComputeItemType.ODE2LHS)
        systemCoordinates: the indices of the system ODE2 coordinates to differentiate by, e.g. the coordinates of an
            object, mbs.systemData.GetObjectLTGODE2(objectNumber), or of a node, mbs.systemData.GetNodeLTGODE2(nodeNumber)
        velocityLevel: False: by the coordinates; True: by their time derivatives
        epsilon: the step of the central differences, absolute; 1e-8 as in the numerical differentiation of the solvers;
            the round-off error is about 1e-16/epsilon, the truncation error about epsilon**2 - 1e-6 is more accurate for
            smooth functions of coordinates of order 1

    Returns:
        the derivative as numpy array, (number of values of function) x (number of systemCoordinates); the state of
        the system is restored afterwards

    Example:
        coordinates = mbs.systemData.GetObjectLTGODE2(oBody)
        numerical = NumericalJacobian(mbs, lambda: mbs.GetObjectOutputBody(oBody, exu.OutputVariableType.Velocity,
                                      localPosition=[0.1,0,0]), coordinates, velocityLevel=True)
        analytic = mbs.ComputeItem(oBody, exu.ComputeItemType.PositionJacobian, localPosition=[0.1,0,0])
    """
    Get = mbs.systemData.GetODE2Coordinates_t if velocityLevel else mbs.systemData.GetODE2Coordinates
    Set = mbs.systemData.SetODE2Coordinates_t if velocityLevel else mbs.systemData.SetODE2Coordinates
    q0 = np.array(Get())
    columns = []
    try:
        for i in systemCoordinates:
            q = q0.copy()
            q[i] += epsilon
            Set(q)
            fPlus = np.array(function(), dtype=float).flatten()
            q[i] -= 2*epsilon
            Set(q)
            fMinus = np.array(function(), dtype=float).flatten()
            columns.append((fPlus - fMinus)/(2*epsilon))
    finally:
        Set(q0)
    return np.array(columns).T


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#THE SYSTEM GRAPH
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#the colors of the kinds of items and of the edges; https://matplotlib.org/examples/color/named_colors.html
_graphColors = {'Node':'red', 'Object':'skyblue', 'Oconnector':'dodgerblue', 'Ojoint':'dodgerblue',
                'Ocontact':'dodgerblue', 'Marker': 'orange', 'Load': 'mediumorchid', 'Sensor': 'forestgreen'}
_graphObjectNodeColor = 'navy' #the edges between objects and nodes, drawn wider


def ComputeSystemGraph(mbs, showLoads=True, showSensors=True, useItemNames=False, useItemTypes=False,
                       addItemTypeNames=True, multiLine=False, addItemData=False, addAnnotations=False):
    """the graph of a MainSystem as networkx graph, without drawing: one graph node per item and the edges object - node,
    marker - object or node, connector - marker, load - marker, sensor - object or node; for an analysis of the
    structure, e.g. its connected parts or the items without connection; needs networkx

    Args:
        mbs: the MainSystem
        showLoads: if True, the loads are included
        showSensors: if True, the sensors are included
        useItemNames: if True, the graph nodes are named by the names of the items instead of their kind and number
        useItemTypes: if True, the graph nodes are named by the item types (MassPoint, JointRevolute, ...) and number
        addItemTypeNames: with useItemTypes, the kind (Node, Object, ...) is put in front
        multiLine: if True, the names are split into lines, for drawing
        addItemData: if True, each graph node gets the attributes 'type' (e.g. 'ObjectMassPoint'), 'ID' (the item
            number) and 'name'
        addAnnotations: if True, each graph node gets the attribute 'annotations': the data of the item except graphics
            data, item numbers, names and types

    Returns:
        the networkx Graph; each graph node has the attributes 'basicType' (Node, Marker, Object, Load, Sensor) and
        'color', each edge 'color'

    Example:
        import networkx as nx
        G = ComputeSystemGraph(mbs)
        print('connected parts:', nx.number_connected_components(G))
    """
    try:
        import networkx as nx
    except ImportError as e:
        raise ImportError("networkx is required for ComputeSystemGraph(...) and DrawSystemGraph(...): pip install networkx") from e

    excludeAnnotations = ['V', 'nodeNumber', 'objectNumber', 'bodyNumber', 'markerNumber', 'loadNumber', 'sensorNumber',
                          'nodeType', 'objectType', 'markerType', 'loadType', 'sensorType', 'name']
    def GetAnnotations(itemDict):
        return {key: value for (key, value) in itemDict.items()
                if not any(key.startswith(startWith) for startWith in excludeAnnotations)}

    G = nx.Graph()
    sLineBreak = '-\n' if multiLine else ''

    def AddItem(itemName, basicType, typeName, i, item, color):
        G.add_node(itemName, basicType=basicType, color=color)
        if addItemData:
            G.nodes[itemName].update({'type': basicType + typeName, 'ID': i, 'name': item['name']})
        if addAnnotations:
            G.nodes[itemName].update({'annotations': GetAnnotations(item)})
        return itemName

    def Label(basicType, typeName, i, item, prefix, joined):
        """the name of a graph node; prefix is what stands in front with addItemTypeNames, joined the type prefixes
        kept on the first line with multiLine"""
        if useItemNames:
            return item['name']
        if not useItemTypes:
            return basicType + str(i)
        itemName = typeName + str(i)
        if addItemTypeNames:
            itemName = prefix + sLineBreak + itemName
        if sLineBreak != '':
            for start in joined:
                itemName = itemName.replace(basicType + sLineBreak + start, basicType + start + sLineBreak)
        return itemName

    nodeNames = []
    for i in range(mbs.systemData.NumberOfNodes()):
        item = mbs.GetNode(i)
        prefix = 'NodeGround' if item['nodeType'].find('Ground') != -1 else 'Node'
        nodeNames.append(AddItem(Label('Node', item['nodeType'], i, item, prefix, ['Rigid', 'Generic']),
                                 'Node', item['nodeType'], i, item, _graphColors['Node']))

    markerNames = []
    for i in range(mbs.systemData.NumberOfMarkers()):
        item = mbs.GetMarker(i)
        markerNames.append(AddItem(Label('Marker', item['markerType'], i, item, 'Marker',
                                         ['Body', 'Object', 'Node', 'SuperElement', 'Kinematic']),
                                   'Marker', item['markerType'], i, item, _graphColors['Marker']))

    objectNames = []
    for i in range(mbs.systemData.NumberOfObjects()):
        item = mbs.GetObject(i)
        objectType = 'Object'
        for (part, kind) in [('Connector', 'Oconnector'), ('Contact', 'Ocontact'), ('Joint', 'Ojoint')]:
            if item['objectType'].find(part) != -1:
                objectType = kind
                break
        if useItemNames or useItemTypes:
            itemName = Label('Object', item['objectType'], i, item, 'Object',
                             ['Joint', 'Mass', 'Beam', 'ANCF', 'Contact', 'Connector', 'Rigid'])
            if sLineBreak != '' and useItemTypes and not useItemNames:
                itemName = itemName.replace('Object' + sLineBreak + 'FFRFr', 'ObjectFFRF' + sLineBreak + 'r')
        else:
            itemName = objectType + str(i)
        objectNames.append(AddItem(itemName, 'Object', item['objectType'], i, item, _graphColors[objectType]))

        nodeNumbers = []
        if 'nodeNumber' in item:
            nodeNumbers += [item['nodeNumber']]
        if 'nodeNumbers' in item:
            nodeNumbers += item['nodeNumbers']
        for j in nodeNumbers:
            if int(j) != exudyn.InvalidIndex(): #for RigidBodySpringDamper
                G.add_edge(objectNames[i], nodeNames[int(j)], color=_graphObjectNodeColor)
        for j in item.get('markerNumbers', []): #connectors, contact, joints
            G.add_edge(objectNames[i], markerNames[int(j)], color=_graphColors['Oconnector'])

    for i in range(mbs.systemData.NumberOfMarkers()):
        item = mbs.GetMarker(i)
        if 'nodeNumber' in item:
            G.add_edge(markerNames[i], nodeNames[int(item['nodeNumber'])], color=_graphColors['Marker'])
        for key in ['objectNumber', 'bodyNumber']:
            if key in item:
                G.add_edge(markerNames[i], objectNames[int(item[key])], color=_graphColors['Marker'])

    if showLoads:
        for i in range(mbs.systemData.NumberOfLoads()):
            item = mbs.GetLoad(i)
            itemName = AddItem(Label('Load', item['loadType'], i, item, 'Load', ['Mass']),
                               'Load', item['loadType'], i, item, _graphColors['Load'])
            G.add_edge(itemName, markerNames[int(item['markerNumber'])], color=_graphColors['Load'])

    if showSensors:
        for i in range(mbs.systemData.NumberOfSensors()):
            item = mbs.GetSensor(i)
            itemName = AddItem(Label('Sensor', item['sensorType'], i, item, 'Sensor', []),
                               'Sensor', item['sensorType'], i, item, _graphColors['Sensor'])
            for key in ['objectNumber', 'bodyNumber']:
                if key in item:
                    G.add_edge(itemName, objectNames[int(item[key])], color=_graphColors['Sensor'])
            if 'nodeNumber' in item:
                G.add_edge(itemName, nodeNames[int(item['nodeNumber'])], color=_graphColors['Sensor'])
    return G


@extends(exudyn.MainSystem)
def DrawSystemGraph(mbs, showLoads=True, showSensors=True, useItemNames = False,
                    useItemTypes = False, addItemTypeNames=True, multiLine=True, fontSizeFactor=1.,
                    layoutDistanceFactor=3., layoutIterations=100, showLegend = True, tightLayout = True,
                    showGraph = True, addItemData = False, addAnnotations = False) -> list:
    """draw the graph of a MainSystem with matplotlib, from ComputeSystemGraph; several options adjust the appearance;
    the layout is random, so it differs from run to run; needs networkx and matplotlib

    Args:
        mbs: MainSystem to be operated with
        showLoads: toggle appearance of loads in mbs
        showSensors: toggle appearance of sensors in mbs
        useItemNames: if True, object names are shown instead of basic object types (Node, Load, ...)
        useItemTypes: if True, object type names (MassPoint, JointRevolute, ...) are shown instead of basic object types (Node, Load, ...); Note that Node, Object, is omitted at the beginning of itemName (as compared to the reference manual); item classes become clear from the legend
        addItemTypeNames: if True, type nymes (Node, Load, etc.) are added
        multiLine: if True, labels are multiline, improving readability; ignored if showGraph = False
        fontSizeFactor: use this factor to scale fonts, allowing to fit larger graphs on the screen with values < 1
        layoutDistanceFactor: this factor influences the arrangement of labels; larger distance values lead to circle-like results
        layoutIterations: more iterations lead to better arrangement of the layout, but need more time for larger systems (use 1000-10000 to get good results)
        showLegend: shows legend for different item types
        tightLayout: if True, uses matplotlib plt.tight_layout() which may raise warning
        showGraph: if True, graph is plotted with matplotlib
        addItemData: if True, specific data is added to the graph nodes, to be used for deeper analysis of system graphs
        addAnnotations: add data node graphs (not shown), except for graphics data, item numbers, names and types (which are already available in graph data or edges)

    Returns:
        :[Any, Any, Any]: returns [networkx, G, items] with nx being networkx, G the graph and item what is returned by nx.draw_networkx_labels(...)
    """
    G = ComputeSystemGraph(mbs, showLoads=showLoads, showSensors=showSensors, useItemNames=useItemNames,
                           useItemTypes=useItemTypes, addItemTypeNames=addItemTypeNames,
                           multiLine=multiLine and showGraph, addItemData=addItemData, addAnnotations=addAnnotations)
    import networkx as nx
    items = None #only assigned if shown
    if showGraph:
        try:
            import matplotlib.pyplot as plt
        except ImportError as e:
            raise ImportError("matplotlib is required for DrawSystemGraph(...)") from e
        plt.clf()

        if showLegend:
            legendColors = {'Node':'red', 'Object':'skyblue', 'Object(Connector)':'dodgerblue',
                            'Marker': 'orange', 'Load': 'mediumorchid', 'Sensor': 'forestgreen'}
            for label in legendColors:
                plt.plot([0],[0],linewidth=8,color=legendColors[label],label=label)
            fontSizeLegend = 10
            if fontSizeFactor > 1: #do not make font size smaller!
                fontSizeLegend *= fontSizeFactor
            plt.legend(fontsize=fontSizeLegend)

        edgeColorMap = []
        edgeWidths = []
        for (a, b, data) in G.edges(data=True):
            edgeColorMap += [data['color']]
            edgeWidths += [4 if data['color'] == _graphObjectNodeColor else 2] #object-node is emphasized

        pos = nx.drawing.spring_layout(G, scale=0.5, k=layoutDistanceFactor*1/np.sqrt(G.size()),
                                       threshold = 1e-5, iterations = layoutIterations)
        nx.draw_networkx_nodes(G, pos, node_size=1)
        nx.draw_networkx_edges(G, pos, edge_color=edgeColorMap, width=edgeWidths)

        #reproduce what draw_networkx_labels does, allowing different colors for nodes
        items = nx.draw_networkx_labels(G, pos, font_size=10*fontSizeFactor, clip_on=False, #clip at plot boundary
                                        bbox=dict(facecolor='skyblue', edgecolor='black',
                                                  boxstyle='round,pad=0.1', lw=10*fontSizeFactor))
        #the box and font per kind of item
        style = {'Object': ('round,pad=0.2', 12), 'Node': ('square,pad=0.1', 10), 'Marker': ('square,pad=0.1', 8)}
        for (itemName, data) in G.nodes(data=True):
            (boxStyle, fontSize) = style.get(data['basicType'], ('round,pad=0.2', 10))
            items[itemName].set_bbox(dict(facecolor=data['color'], edgecolor=data['color'], boxstyle=boxStyle))
            items[itemName].set_fontsize(fontSize*fontSizeFactor)

        plt.axis('off') #do not show frame, because usually some nodes are very close to frame ...
        if tightLayout:
            plt.tight_layout()
        plt.margins(x=0.1*fontSizeFactor, y=0.1*fontSizeFactor) #larger margin, to avoid clipping of long texts
        plt.draw() #force redraw after colors have changed

    return [nx, G, items]
