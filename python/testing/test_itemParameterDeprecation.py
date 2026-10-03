#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  An item parameter that is renamed keeps its old name in the definition, with
#           deprecated=Deprecated(since, expires) and the new name as its description (#2589). The
#           generators then forward the old name to the new one everywhere a script can write it - the
#           item's dictionary, GetObjectParameter / SetObjectParameter, the keyword of the Python item
#           class - with a DeprecationWarning, and they test the old names LAST, so that a model that uses
#           the current names pays nothing. The test reads what the generators emit for physicsMass of
#           ObjectMassPoint, renamed mass by #2814.
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


def _MassPointWithOldName(forwardsTo='mass'):
    """ObjectMassPoint, whose physicsMass is the old name of mass (#2814); forwardsTo changes where it forwards, in a copy"""
    definition = dict([d for d in itemModel.ItemDefinitions() if d['className'] == 'ObjectMassPoint'][0])
    definition['members'] = [dict(m, description=forwardsTo) if m['pythonName'] == 'physicsMass' else m
                             for m in definition['members']]
    return definition


def test_theOldNameForwardsLastInEveryPath():
    mainHeader = itemHeaderEmitter.ItemCppHeaders(_MassPointWithOldName())[1]
    dictionaryWrite = mainHeader[mainHeader.index('SetWithDictionary'):mainHeader.index('GetDictionary(')]
    #the old name, after the new one, written into the new one's storage, with the warning
    assert dictionaryWrite.index('"mass"') < dictionaryWrite.index('"physicsMass"')
    assert 'PyDeprecated("items", "ObjectMassPoint.physicsMass", "ObjectMassPoint: the parameter physicsMass is deprecated' in dictionaryWrite
    assert 'EPyUtils::FromPython(d["physicsMass"], cObjectMassPoint->GetParameters().mass' in dictionaryWrite
    #not stored, not in the dictionary that is read
    assert 'd["physicsMass"] =' not in mainHeader
    for function in ['GetParameter(', 'SetParameter(']:
        body = mainHeader[mainHeader.index('virtual ' + ('py::object ' if function == 'GetParameter(' else 'void ') + function):]
        body = body[:body.index('illegal parameter name')]
        assert body.index('"physicsMass"') > body.index('"mass"') #searched last
        assert body.index('"physicsMass"') > body.index('"nodeNumber"')
    assert 'Real physicsMass;' not in itemHeaderEmitter.ItemCppHeaders(_MassPointWithOldName())[0]


def test_thePythonClassTakesTheOldNameLastAndOnlyGivesItIfUsed():
    text = itemInterfaceEmitter.ItemClasses(_MassPointWithOldName())
    signature = [line for line in text.split('\n') if 'def __init__(self, name' in line and 'mass' in line][0]
    assert signature.index('nodeNumber') < signature.index('physicsMass = None') #positions of the current parameters unchanged
    assert "if self.physicsMass is not None:" in text


def test_aDeprecatedNameNeedsAParameterToForwardTo():
    with pytest.raises(ValueError, match='is not an interface parameter'):
        itemHeaderEmitter.ItemCppHeaders(_MassPointWithOldName(forwardsTo='mas'))
