#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The energies as output variables (#2202) refuse what they cannot mean: a local position
#           other than [0,0,0] (the energy is one value for the whole item), and a spring-damper whose
#           force law is a user function (its potential is unknown). The values themselves are checked
#           by the test model energiesTest.py.
#
# Usage:    pytest python/testing/test_energies.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-01
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import pytest

import exudyn as exu
from exudyn.utilities import (ObjectGround, NodePoint, MassPoint, MarkerBodyPosition, MarkerNodePosition,
                              SpringDamper)

exu.special.userInterface.SuppressAll(True)


def Model(userFunction=None):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.AddObject(ObjectGround())
    n = mbs.AddNode(NodePoint(referenceCoordinates=[1, 0, 0], initialVelocities=[0, 2, 0]))
    oMass = mbs.AddObject(MassPoint(nodeNumber=n, physicsMass=3))
    kwargs = {} if userFunction is None else {'springForceUserFunction': userFunction}
    oSpring = mbs.AddObject(SpringDamper(markerNumbers=[mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround)),
                                                        mbs.AddMarker(MarkerNodePosition(nodeNumber=n))],
                                         referenceLength=0.5, stiffness=10, **kwargs))
    mbs.Assemble()
    return mbs, oMass, oSpring


def test_theEnergiesOfAMassPointAndASpring():
    mbs, oMass, oSpring = Model()
    assert mbs.GetObjectOutputBody(oMass, exu.OutputVariableType.KineticEnergy) == pytest.approx(0.5*3*2**2)
    assert mbs.GetObjectOutput(oSpring, exu.OutputVariableType.PotentialEnergy) == pytest.approx(0.5*10*0.5**2)


def test_aLocalPositionIsRefused():
    mbs, oMass, oSpring = Model()
    with pytest.raises(ValueError, match='localPosition must be'):
        mbs.GetObjectOutputBody(oMass, exu.OutputVariableType.KineticEnergy, localPosition=[0.1, 0, 0])


def test_aUserFunctionHasNoPotentialEnergy():
    mbs, oMass, oSpring = Model(userFunction=lambda mbs, t, itemNumber, u, v, k, d, f: k*u)
    with pytest.raises(NotImplementedError, match='springForceUserFunction'):
        mbs.GetObjectOutput(oSpring, exu.OutputVariableType.PotentialEnergy)
