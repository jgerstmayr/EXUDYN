#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  User items with the interfaces of the built-in items (#3111, #3125, design:
#           docs/dev/design/symbolicUserItems.md): UserItemCreator(mbs) has the Create functions of mbs with the same
#           arguments, and creates ObjectUserBody and ObjectConnectorUser instead of the built-in objects, with the same
#           nodes, markers and loads. The user functions are Python functions, so that the system runs in Exudyn as
#           built; Record() replaces them by one symbolic recording per kind of item, which SymbolicSystem and the
#           export take. A script that builds a system with creator = mbs builds the same system of user items with
#           creator = UserItemCreator(mbs).
#           The items: CreateGround, CreateMassPoint, CreateSpringDamper, CreateForce. An argument a user item does
#           not support raises, naming it.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-27 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import exudyn
import exudyn.itemInterface as eii
import exudyn.cMath as cMath
import exudyn.graphics as graphics
from exudyn.advancedUtilities import CreateSymbolicUserFunction
from exudyn.misc.mainSystemExtensions import GetMarkersPosRot

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = ['UserItemCreator']


#the mass point, ObjectUserBody on a NodePoint, parameters [mass]: as ObjectMassPoint, M = m I, the position of a local
#point is the node position plus the local position; it does not rotate
def _UFmassPointMass(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return parameters[0]*np.eye(3)

def _UFmassPointMassIntegral(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return parameters[0]*np.eye(3)

def _UFmassPointPosition(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return qTotal + localPosition

def _UFmassPointPositionJacobian(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return np.eye(3)

def _UFmassPointRotationMatrix(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return np.eye(3)

def _UFmassPointRotationJacobian(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return np.zeros((3, 3))

#the spring-damper, ObjectConnectorUser on two position markers, parameters [stiffness, damping, referenceLength, force,
#velocityOffset]: as ObjectConnectorSpringDamper, the force k (L - L0) + d (dL/dt - v0) + f along the axis, pulling
#marker 0 towards marker 1 if positive; with cMath, it computes numbers and is recorded
def _UFspringDamperForce(mbs, t, itemNumber, position0, velocity0, rotationMatrix0, angularVelocity0,
                        position1, velocity1, rotationMatrix1, angularVelocity1, dataCoordinates, parameters):
    delta = position1 - position0
    length = cMath.sqrt(delta @ delta)
    axis = (1/length)*delta
    force = (parameters[0]*(length - parameters[2]) + parameters[1]*(axis @ (velocity1 - velocity0) - parameters[4])
             + parameters[3])
    return np.hstack([force*axis, -force*axis])

#the user functions of each kind of item, which Record records once per kind
_kindFunctions = {'massPoint': [('massMatrixUserFunction', _UFmassPointMass),
                                ('massIntegralUserFunction', _UFmassPointMassIntegral),
                                ('positionUserFunction', _UFmassPointPosition),
                                ('positionJacobianUserFunction', _UFmassPointPositionJacobian),
                                ('rotationMatrixUserFunction', _UFmassPointRotationMatrix),
                                ('rotationJacobianUserFunction', _UFmassPointRotationJacobian)],
                  'springDamper': [('forceUserFunction', _UFspringDamperForce)]}


def _Refuse(where, name, value, default, reason=''):
    """raise naming an argument the user item does not support, if it differs from its default"""
    if isinstance(value, (list, tuple, np.ndarray)) or isinstance(default, (list, tuple, np.ndarray)):
        differs = np.shape(value) != np.shape(default) or not np.array_equal(value, default)
    else:
        differs = value is not default and value != default
    if differs:
        raise ValueError('UserItemCreator.' + where + ': the argument ' + name + ' is not supported by the user item'
                         + (' (' + reason + ')' if reason else ''))


class UserItemCreator:
    """creates user items with the Create functions of mbs and their arguments (#3125): ObjectUserBody and
    ObjectConnectorUser with Python user functions instead of the built-in objects, recorded as symbolic user functions
    by Record, one recording per kind of item; see the Details of this module

    Args:
        mbs: the MainSystem where the items are created

    Note:
        Its members: mbs; items, the dict of the item numbers of each kind ('massPoint', 'springDamper'); loads, the
        list of the loads with a user function created by CreateForce; recordings, the symbolic user functions of
        Record, which must live as long as the system. Example:

        creator = UserItemCreator(mbs) #or creator = mbs: the same script with the built-in items
        b0 = creator.CreateMassPoint(referencePosition=[1,0,0], mass=2, gravity=[0,-9.81,0])
        creator.CreateSpringDamper(itemNumbers=[creator.CreateGround(), b0], stiffness=100, damping=1)
        creator.Record()
        mbs.Assemble()
    """
    def __init__(self, mbs):
        self.mbs = mbs
        self.items = {kind: [] for kind in _kindFunctions}
        self.loads = []
        self.recordings = []
        self._massPointNodes = {} #the node of each user mass point, by the int of its body number

    def CreateGround(self, name='', referencePosition=None, referenceRotationMatrix=None, graphicsDataList=[],
                     graphicsDataUserFunction=0, show=True, referenceHT=None):
        """the built-in ObjectGround, as mbs.CreateGround: it has no coordinates

        Args:
            name: as in mbs.CreateGround
            referencePosition: as in mbs.CreateGround
            referenceRotationMatrix: as in mbs.CreateGround
            graphicsDataList: as in mbs.CreateGround
            graphicsDataUserFunction: as in mbs.CreateGround
            show: as in mbs.CreateGround
            referenceHT: as in mbs.CreateGround

        Returns:
            the ObjectIndex of the ground
        """
        return self.mbs.CreateGround(name=name, referencePosition=referencePosition,
                                     referenceRotationMatrix=referenceRotationMatrix, graphicsDataList=graphicsDataList,
                                     graphicsDataUserFunction=graphicsDataUserFunction, show=show, referenceHT=referenceHT)

    def CreateMassPoint(self, name='', referencePosition=[0., 0., 0.], initialDisplacement=[0., 0., 0.],
                        initialVelocity=[0., 0., 0.], mass=0, gravity=[0., 0., 0.], graphicsDataList=[], drawSize=-1,
                        color=[-1., -1., -1., -1.], show=True, create2D=False, returnDict=False, physicsMass=None):
        """a mass point as ObjectUserBody on a NodePoint, with the nodes, markers and loads of mbs.CreateMassPoint and
        its arguments

        Args:
            name: as in mbs.CreateMassPoint
            referencePosition: as in mbs.CreateMassPoint
            initialDisplacement: as in mbs.CreateMassPoint
            initialVelocity: as in mbs.CreateMassPoint
            mass: as in mbs.CreateMassPoint; the parameter of the user body
            gravity: as in mbs.CreateMassPoint
            graphicsDataList: as in mbs.CreateMassPoint, the graphicsData of the user body
            drawSize: as in mbs.CreateMassPoint
            color: as in mbs.CreateMassPoint
            show: as in mbs.CreateMassPoint
            create2D: refused if True: a planar user mass point does not exist yet
            returnDict: as in mbs.CreateMassPoint
            physicsMass: refused if given: use mass

        Returns:
            the ObjectIndex of the body, or with returnDict the dict of mbs.CreateMassPoint
        """
        _Refuse('CreateMassPoint', 'create2D', create2D, False, 'a planar user mass point does not exist yet')
        _Refuse('CreateMassPoint', 'physicsMass', physicsMass, None, 'use mass')
        mbs = self.mbs
        if len(graphicsDataList) != 0:
            drawSize = 0 #the node is shown (number, basis), but not drawn, as mbs.CreateMassPoint does
        nodeNumber = mbs.AddNode(eii.NodePoint(name='Node:' + name if name != '' else '', referenceCoordinates=referencePosition,
                                               initialCoordinates=initialDisplacement, initialVelocities=initialVelocity,
                                               visualization=eii.VNodePoint(show=show, drawSize=drawSize, color=color)))
        bodyNumber = mbs.AddObject(eii.ObjectUserBody(name=name, nodeNumbers=[nodeNumber], parameters=[mass],
                                                      constantMassMatrix=True,
                                                      **{functionName: function for (functionName, function) in _kindFunctions['massPoint']},
                                                      visualization=eii.VObjectUserBody(show=graphicsDataList != [],
                                                                                        graphicsData=graphicsDataList)))
        self.items['massPoint'].append(bodyNumber)
        self._massPointNodes[int(bodyNumber)] = nodeNumber
        result = {'nodeNumber': nodeNumber, 'bodyNumber': bodyNumber}
        if list(gravity) != [0., 0., 0.]:
            result['markerBodyMass'] = mbs.AddMarker(eii.MarkerBodyMass(bodyNumber=bodyNumber))
            result['loadNumber'] = mbs.AddLoad(eii.LoadMassProportional(markerNumber=result['markerBodyMass'], loadVector=gravity))
        return result if returnDict else bodyNumber

    def CreateSpringDamper(self, name='', itemNumbers=[None, None], localPosition0=[0., 0., 0.], localPosition1=[0., 0., 0.],
                           referenceLength=None, stiffness=0., damping=0., force=0., velocityOffset=0.,
                           springForceUserFunction=0, show=True, drawSize=-1, color=exudyn.graphics.color.default,
                           bodyNumbers=None, bodyOrNodeList=None):
        """a spring-damper as ObjectConnectorUser on the markers of mbs.CreateSpringDamper, with its arguments and its
        force; at the center of a user mass point the marker is on its node, which has the same position; drawn by a
        graphicsDataUserFunction

        Args:
            name: as in mbs.CreateSpringDamper
            itemNumbers: as in mbs.CreateSpringDamper
            localPosition0: as in mbs.CreateSpringDamper
            localPosition1: as in mbs.CreateSpringDamper
            referenceLength: as in mbs.CreateSpringDamper
            stiffness: as in mbs.CreateSpringDamper
            damping: as in mbs.CreateSpringDamper
            force: as in mbs.CreateSpringDamper
            velocityOffset: as in mbs.CreateSpringDamper
            springForceUserFunction: refused if given: the force is the forceUserFunction of the connector
            show: as in mbs.CreateSpringDamper
            drawSize: a line if not positive, else a cylinder of diameter drawSize
            color: the color of the drawing; the default: grey
            bodyNumbers: refused if given: use itemNumbers
            bodyOrNodeList: refused if given: use itemNumbers

        Returns:
            the ObjectIndex of the connector
        """
        _Refuse('CreateSpringDamper', 'springForceUserFunction', springForceUserFunction, 0,
                'the force is the forceUserFunction of the connector')
        _Refuse('CreateSpringDamper', 'bodyNumbers', bodyNumbers, None, 'use itemNumbers')
        _Refuse('CreateSpringDamper', 'bodyOrNodeList', bodyOrNodeList, None, 'use itemNumbers')
        mbs = self.mbs
        #at the center of a user mass point, the marker is on its node, which SymbolicSystem takes (#3121); the position
        #is the same as on the body
        itemNumbers = [self._massPointNodes[int(item)] if (isinstance(item, exudyn.ObjectIndex) and int(item) in self._massPointNodes
                                                           and not np.any(localPosition)) else item
                       for (item, localPosition) in zip(itemNumbers, [localPosition0, localPosition1])]
        [marker0, marker1, p0, p1, A0, A1] = GetMarkersPosRot(mbs, name, itemNumbers, localPosition0, localPosition1,
                                                              getPosition=True, getRotationMatrix=False, useRigidMarker=False)
        if referenceLength is None:
            referenceLength = cMath.norm(np.array(p1) - p0)
        drawColor = list(color) if color[0] >= 0 else graphics.color.grey

        def UFgraphics(mbs, itemNumber):
            positions = [np.array(mbs.GetMarkerOutput(marker, exudyn.OutputVariableType.Position,
                                                      configuration=exudyn.ConfigurationType.Visualization))
                         for marker in [marker0, marker1]]
            if drawSize > 0:
                return [graphics.Cylinder(pAxis=positions[0], vAxis=positions[1] - positions[0], radius=0.5*drawSize,
                                          color=drawColor, nTiles=8)]
            return [graphics.Lines(positions, color=drawColor)]

        connector = mbs.AddObject(eii.ObjectConnectorUser(name=name, markerNumbers=[marker0, marker1],
                                                          parameters=[stiffness, damping, referenceLength, force, velocityOffset],
                                                          forceUserFunction=_UFspringDamperForce,
                                                          visualization=eii.VObjectConnectorUser(show=show, graphicsDataUserFunction=UFgraphics)))
        self.items['springDamper'].append(connector)
        return connector

    def CreateForce(self, name='', itemNumber=None, loadVector=[0., 0., 0.], localPosition=[0., 0., 0.], bodyFixed=False,
                    loadVectorUserFunction=0, show=True, bodyNumber=None):
        """the built-in LoadForceVector of mbs.CreateForce, with its arguments: on a user mass point, its marker acts by
        the Jacobian of the user body

        Args:
            name: as in mbs.CreateForce
            itemNumber: as in mbs.CreateForce
            loadVector: as in mbs.CreateForce
            localPosition: as in mbs.CreateForce
            bodyFixed: as in mbs.CreateForce
            loadVectorUserFunction: as in mbs.CreateForce; a Python function is recorded by Record
            show: as in mbs.CreateForce
            bodyNumber: refused if given: use itemNumber

        Returns:
            the LoadIndex of the force
        """
        _Refuse('CreateForce', 'bodyNumber', bodyNumber, None, 'use itemNumber')
        load = self.mbs.CreateForce(name=name, itemNumber=itemNumber, loadVector=loadVector, localPosition=localPosition,
                                    bodyFixed=bodyFixed, loadVectorUserFunction=loadVectorUserFunction, show=show)
        if callable(loadVectorUserFunction):
            self.loads.append((load, loadVectorUserFunction))
        return load

    def Record(self):
        """replace the Python user functions of the items by symbolic ones: each user function of a kind recorded once,
        with the first item of the kind, and set in every item of the kind - one recording per kind, as SymbolicSystem
        groups them -; the loadVectorUserFunction of each force of CreateForce recorded on its own. The recordings are
        kept in recordings, as long as this object lives"""
        mbs = self.mbs
        for (kind, functions) in _kindFunctions.items():
            if not self.items[kind]:
                continue
            for (functionName, function) in functions:
                recording = CreateSymbolicUserFunction(mbs, function, functionName, self.items[kind][0])
                self.recordings.append(recording)
                for item in self.items[kind]:
                    mbs.SetObjectParameter(item, functionName, recording)
        for (load, function) in self.loads:
            recording = CreateSymbolicUserFunction(mbs, function, 'loadVectorUserFunction', itemTypeName='LoadForceVector')
            self.recordings.append(recording)
            mbs.SetLoadParameter(load, 'loadVectorUserFunction', recording)
        self.loads = []
