#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  A parameter that the core COMPUTES can be read from Python and cannot be written
#           (#2413). `ObjectContactConvexRoll` has two of them:
#
#               pContact          the current potential contact point, computed in every
#                                 contact evaluation
#               rBoundingSphere   computed from coefficientsHull in InitializeObject
#
#           Reading them is the point - a user who wants to know where the roll touches the
#           ground has no other way - and writing them was a no-op that looked like it worked:
#           SetObjectParameter ends in ParametersHaveChanged(), which recomputes what was just
#           written. rBoundingSphere is read-only since RG4.2, so it raises instead.
#
#           These are the FFRF-computed-member pattern, which the maintainer asked pContact to
#           follow: readable, documented as computed, and not settable.
#
# Usage:    pytest python/testing/test_computedParameters.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-26
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import pytest

import exudyn as exu
from exudyn.utilities import (ObjectContactConvexRoll, ObjectGround, MarkerBodyRigid,
                              MarkerNodeRigid, NodeGenericData, NodePoint, NodeRigidBodyEP,
                              ObjectRigidBody)

#the hull of the roll, from ConvexContactTest.py: a polynomial in the local axis coordinate, in
#numpy order (highest power first), so the radius at the middle of the roll is its last coefficient
hullCoefficients = [-3.6, 0., 1.65e-02]


@pytest.fixture
def rollSystem():
    """a ground, a rigid body and one ObjectContactConvexRoll between them"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    ground = mbs.AddObject(ObjectGround())
    markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=ground, localPosition=[0, 0, 0]))

    node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0, 0, 5e-3, 1, 0, 0, 0]))
    body = mbs.AddObject(ObjectRigidBody(physicsMass=1,
                                         physicsInertia=[1e-3, 1e-3, 1e-3, 0, 0, 0],
                                         nodeNumber=node))
    markerRoll = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node))

    data = mbs.AddNode(NodeGenericData(initialCoordinates=[0, 0, 0], numberOfDataCoordinates=3))
    roll = mbs.AddObject(ObjectContactConvexRoll(
        markerNumbers=[markerGround, markerRoll], nodeNumber=data,
        contactStiffness=1e3, contactDamping=1, dynamicFriction=0.9,
        staticFrictionOffset=0, viscousFriction=0, exponentialDecayStatic=1e-3,
        frictionProportionalZone=1e-4, rollLength=0.1, coefficientsHull=hullCoefficients))
    mbs.Assemble()
    return (mbs, roll)


def test_pContactIsReadable(rollSystem):
    """the computed contact point reaches Python; nothing else exposes it"""
    (mbs, roll) = rollSystem
    value = mbs.GetObjectParameter(roll, 'pContact')
    assert len(value) == 3, 'pContact is a 3D point, got ' + str(value)
    assert all(np.isfinite(value)), 'pContact holds ' + str(value)


def test_rBoundingSphereIsComputedFromTheHull(rollSystem):
    """the bounding sphere is the hull polynomial at 0, and Python can read it"""
    (mbs, roll) = rollSystem
    value = mbs.GetObjectParameter(roll, 'rBoundingSphere')
    assert value == pytest.approx(np.polyval(hullCoefficients, 0.)), (
        'rBoundingSphere is ' + str(value) + ', the hull polynomial at 0 is '
        + str(np.polyval(hullCoefficients, 0.)))


def test_bothAreInTheDictionary(rollSystem):
    """mbs.GetObject returns them too, which is how a user finds them"""
    (mbs, roll) = rollSystem
    item = mbs.GetObject(roll)
    assert 'pContact' in item and 'rBoundingSphere' in item, sorted(item.keys())


@pytest.mark.parametrize('name', ['pContact', 'rBoundingSphere'])
def test_aComputedParameterCannotBeWritten(rollSystem, name):
    """writing it used to be accepted and undone at once; it raises now

    That is the whole of RG4.2: a value the core recomputes must not look settable."""
    (mbs, roll) = rollSystem
    with pytest.raises(Exception):
        mbs.SetObjectParameter(roll, name, mbs.GetObjectParameter(roll, name))
