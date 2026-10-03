#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  A body gets its access function types from the functions it provides - GetPositionJacobian,
#           GetRotationJacobian, GetMassWeightedPositionJacobian,
#           GetJacobianTransposedTimesVectorDerivative -, derived by ItemAccessFunctionTypes() in
#           definitionTypes, with the table of the hand-written parent classes (#2744). Rule 7 of the
#           definition validator checks that table against the C++ headers; this test checks the
#           derivation, the own markers of the super elements and the kinematic tree, and that rule 7
#           finds a derived type the C++ does not back and a provided function that is not derived.
#
# Usage:    pytest python/testing/test_accessFunctionDeclarations.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-02
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import collections
import copy
import os
import sys

repositoryRoot = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(repositoryRoot, 'tools', 'generators'))
sys.path.insert(0, os.path.join(repositoryRoot, 'definitions'))
import definitionValidator                                                  # noqa: E402
import itemDefsObjects                                                      # noqa: E402

classes = definitionValidator.ParseParentClasses()


def _Definition(className):
    return copy.deepcopy([d for d in itemDefsObjects.definitions if d['className'] == className][0])


def _Check(definition):
    return definitionValidator._CheckAccessFunctions('itemDefsObjects', definition, classes, collections.Counter())


def test_everyObjectProvidesTheAccessFunctionsItDeclares():
    violations = []
    for definition in itemDefsObjects.definitions:
        violations += _Check(definition)
    assert violations == []


def _AccessMember(definition):
    return [m for m in definition['members'] if m.get('deriveAccess', False)][0]


def test_theTypesAreDerivedFromTheFunctions():
    assert _AccessMember(_Definition('ObjectMassPoint'))['accessFunctionTypes'] == [
        'TranslationalVelocity_qt', 'JacobianTtimesVector_q', 'DisplacementMassIntegral_q']
    assert _AccessMember(_Definition('ObjectRigidBody'))['accessFunctionTypes'] == [
        'TranslationalVelocity_qt', 'AngularVelocity_qt', 'JacobianTtimesVector_q', 'DisplacementMassIntegral_q']


def test_ownMarkersOnlyIsDerived():
    #an object with markers of its own and none of the body functions refuses the general body markers
    for (className, ownMarkersOnly) in [('ObjectGenericODE2', True), ('ObjectKinematicTree', True), ('ObjectFFRF', False)]:
        member = _AccessMember(_Definition(className))
        assert (not member['bodyMarkers']) == ownMarkersOnly
        assert ('OwnMarkersOnly' in member['implementation']) == ownMarkersOnly


def test_aDerivedTypeTheCppDoesNotBackIsFound():
    definition = _Definition('ObjectMassPoint')
    definition['members'] = [m for m in definition['members'] if m['pythonName'] != 'GetMassWeightedPositionJacobian']
    assert _Check(definition) == ['itemDefsObjects.py: ObjectMassPoint: DisplacementMassIntegral_q is derived, but the C++ '
                                  'class does not provide GetMassWeightedPositionJacobian - correct definitionTypes.parentClassAccessFunctions']


def test_aProvidedFunctionThatIsNotDerivedIsFound():
    definition = _Definition('ObjectANCFCable2D') #as if the table missed GetRotationJacobian of CObjectANCFCable2DBase
    _AccessMember(definition)['bodyAccessFunctionTypes'].remove('AngularVelocity_qt')
    assert _Check(definition) == ['itemDefsObjects.py: ObjectANCFCable2D: AngularVelocity_qt is not derived, but the C++ '
                                  'class provides GetRotationJacobian - correct definitionTypes.parentClassAccessFunctions']


def test_theFunctionsOfAHandWrittenParentCount():
    #ObjectANCFCable2D provides its access functions through CObjectANCFCable2DBase
    assert _Check(_Definition('ObjectANCFCable2D')) == []
