#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  An item parameter that is renamed keeps its old name in the definition, with
#           deprecated=Deprecated(since, expires) and the new name as its description (#2589). The
#           generators then forward the old name to the new one everywhere a script can write it - the
#           item's dictionary, GetObjectParameter / SetObjectParameter, the keyword of the Python item
#           class - with a DeprecationWarning, and they test the old names LAST, so that a model that uses
#           the current names pays nothing. No item has a renamed parameter today, so this test renames
#           physicsMass of ObjectMassPoint in a copy of its definition and reads what the generators
#           emit (it was also built and run once, 2026-10-03: see the revision log, RG12.2).
#
# Usage:    pytest python/testing/test_itemParameterDeprecation.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-03
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import sys

import pytest

repositoryRoot = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(repositoryRoot, 'tools', 'generators'))
sys.path.insert(0, os.path.join(repositoryRoot, 'definitions'))
import definitionTypes as dt                                                # noqa: E402
import itemModel                                                            # noqa: E402
import itemHeaderEmitter                                                    # noqa: E402
import itemInterfaceEmitter                                                 # noqa: E402


def _MassPointWithOldName(forwardsTo='physicsMass'):
    definition = dict([d for d in itemModel.ItemDefinitions() if d['className'] == 'ObjectMassPoint'][0])
    definition['members'] = list(definition['members']) #a copy of the list; the members are not changed
    old = dt.ItemParameter(type=dt.TReal(minimum=0), destination=dt.DestComp + dt.DestParam, pythonName='mass',
                           deprecated=dt.Deprecated('1.12.245', 2028), defaultValue=dt.NoDefaultValue,
                           description=forwardsTo)
    index = [m['pythonName'] for m in definition['members']].index('physicsMass')
    definition['members'].insert(index + 1, old)
    return definition


def test_theOldNameForwardsLastInEveryPath():
    mainHeader = itemHeaderEmitter.ItemCppHeaders(_MassPointWithOldName())[1]
    dictionaryWrite = mainHeader[mainHeader.index('SetWithDictionary'):mainHeader.index('GetDictionary(')]
    #the old name, after the new one, written into the new one's storage, with the warning
    assert dictionaryWrite.index('"physicsMass"') < dictionaryWrite.index('"mass"')
    assert 'PyDeprecated("items", "ObjectMassPoint.mass", "ObjectMassPoint: the parameter mass is deprecated' in dictionaryWrite
    assert 'EPyUtils::FromPython(d["mass"], cObjectMassPoint->GetParameters().physicsMass' in dictionaryWrite
    #not stored, not in the dictionary that is read
    assert 'd["mass"] =' not in mainHeader
    for function in ['GetParameter(', 'SetParameter(']:
        body = mainHeader[mainHeader.index('virtual ' + ('py::object ' if function == 'GetParameter(' else 'void ') + function):]
        body = body[:body.index('illegal parameter name')]
        assert body.index('"mass"') > body.index('"physicsMass"') #searched last
        assert body.index('"mass"') > body.index('"nodeNumber"')
    assert 'Real mass;' not in itemHeaderEmitter.ItemCppHeaders(_MassPointWithOldName())[0]


def test_thePythonClassTakesTheOldNameLastAndOnlyGivesItIfUsed():
    text = itemInterfaceEmitter.ItemClasses(_MassPointWithOldName())
    signature = [line for line in text.split('\n') if 'def __init__(self, name' in line and 'physicsMass' in line][0]
    assert signature.index('nodeNumber') < signature.index('mass = None') #positions of the current parameters unchanged
    assert "if self.mass is not None:" in text


def test_aDeprecatedNameNeedsAParameterToForwardTo():
    with pytest.raises(ValueError, match='is not an interface parameter'):
        itemHeaderEmitter.ItemCppHeaders(_MassPointWithOldName(forwardsTo='physicsMas'))
