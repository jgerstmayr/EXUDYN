#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  A body declares its access function types (ItemAccessFunctionTypes) and provides one
#           function per type - GetPositionJacobian, GetRotationJacobian,
#           GetMassWeightedPositionJacobian, GetJacobianTransposedTimesVectorDerivative (#2744).
#           Rule 7 of the definition validator keeps the two in agreement; this test checks that it
#           finds a type without its function, a function without its type, and nothing in the
#           definitions as they are.
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


def test_aDeclaredTypeWithoutItsFunctionIsFound():
    definition = _Definition('ObjectMassPoint')
    definition['members'] = [m for m in definition['members'] if m['pythonName'] != 'GetMassWeightedPositionJacobian']
    assert _Check(definition) == ['itemDefsObjects.py: ObjectMassPoint: declares DisplacementMassIntegral_q, '
                                  'but provides no GetMassWeightedPositionJacobian']


def test_aFunctionWithoutItsDeclaredTypeIsFound():
    definition = _Definition('ObjectRigidBody')
    for member in definition['members']:
        if 'accessFunctionTypes' in member:
            member['accessFunctionTypes'].remove('JacobianTtimesVector_q')
    assert _Check(definition) == ['itemDefsObjects.py: ObjectRigidBody: provides GetJacobianTransposedTimesVectorDerivative, '
                                  'but does not declare JacobianTtimesVector_q']


def test_theFunctionsOfAHandWrittenParentCount():
    #ObjectANCFCable2D provides its access functions through CObjectANCFCable2DBase
    assert _Check(_Definition('ObjectANCFCable2D')) == []
