#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The layer under the settings dialog: the four functions of exudyn.misc.GUI that decide
#           what a typed value becomes. They need no window, which is why they can be tested at
#           all - everything above them opens one (revision2026b step RG6.2.2, #2596).
#
#           The strongest test here is not invented data: it walks the REAL settings structures,
#           622 values between simulationSettings and visualizationSettings, and requires that
#           every one of them survives the round trip the dialog puts it through -
#           ConvertValue2String on the way in, CheckType and ConvertString2Value on the way out.
#           A value that does not survive is one a user cannot open the dialog on without
#           changing it.
#
# Usage:    pytest python/testing/test_guiValues.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-23
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import pytest

import exudyn
import exudyn.misc.GUI as gui


def Leaves(dictionary, path=''):
    """every editable value of a settings structure, as (path, value, type, size)"""
    leaves = []
    for (key, value) in dictionary.items():
        if isinstance(value, dict) and 'itemIdentifier' in value:
            leaves.append((path + key, value['value'], value['type'], value['size']))
        elif isinstance(value, dict):
            leaves += Leaves(value, path + key + '.')
    return leaves


def SettingsLeaves():
    return (Leaves(exudyn.SimulationSettings().GetDictionaryWithTypeInfo())
            + Leaves(exudyn.VisualizationSettings().GetDictionaryWithTypeInfo()))


@pytest.fixture(scope='module')
def leaves():
    return SettingsLeaves()


@pytest.fixture(scope='module')
def comboLists():
    return gui.GetComboBoxListsDict(exudyn)


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the real settings, which is what the dialog is opened on

def testThereIsSomethingToTest(leaves):
    """if this drops to nothing, the test below passes for the wrong reason"""
    assert len(leaves) > 500
    assert len({leafType for (_, _, leafType, _) in leaves}) > 15


#What does NOT survive the round trip today, by path (#2597, revision2026b step RG6.2.3). This
#list is meant to SHRINK: a path that starts working has to be taken out here, and a path that
#stops working is a new failure. CheckType has no branch for an enum type, and ':' is not one of
#its valid file name characters - so an absolute Windows path is refused, including this one,
#which is a shipped default.
knownRoundTripGaps = [
    'timeIntegration.explicitIntegration.dynamicSolverType',
    'linearSolverType',
    'contour.outputVariable',
    'interactive.highlightItemType',
    'interactive.openVR.actionManifestFileName',
    ]


def testEveryCurrentValueSurvivesTheRoundTrip(leaves, comboLists):
    """value -> string -> value, for every setting there is: what the dialog does when it is
    opened and closed without touching anything"""
    failures = []
    survived = []
    for (path, value, leafType, size) in leaves:
        asString = gui.ConvertValue2String(value, leafType, size)
        [isValid, message] = gui.CheckType(asString, leafType, size)
        if not isValid:
            failures.append(path + ' (' + leafType + '): CheckType says "' + message + '"')
            continue
        [back, errorMessage] = gui.ConvertString2Value(asString, leafType, size, comboLists)
        if errorMessage != '':
            failures.append(path + ' (' + leafType + '): ' + errorMessage)
        elif isinstance(value, float) and isinstance(back, float):
            #floats are written through float32 on purpose: the C++ side is single precision
            if not (abs(back - value) <= 1e-6*max(1.0, abs(value))):
                failures.append(path + ': ' + str(value) + ' came back as ' + str(back))
        elif isinstance(value, (bool, int, str)) and back != value:
            failures.append(path + ': ' + repr(value) + ' came back as ' + repr(back))
        else:
            survived.append(path)

    unexpected = [failure for failure in failures
                  if failure.split(' ')[0] not in knownRoundTripGaps]
    assert unexpected == [], '\n'.join(unexpected)

    fixed = [path for path in knownRoundTripGaps if path in survived]
    assert fixed == [], ('these no longer fail, so take them out of knownRoundTripGaps: '
                         + ', '.join(fixed))


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the conversions, one type at a time

@pytest.mark.parametrize(('text', 'valueType', 'expected'), [
    ('True', 'bool', True),
    ('False', 'bool', False),
    ('anything else', 'bool', False),          #bool is not parsed, it is compared with 'True'
    ('1.5', 'float', 1.5),
    ('-2', 'Real', -2.0),
    ('3', 'Index', 3),
    ('0', 'UInt', 0),
    ('some text', 'String', 'some text'),
    ('C:/a path/file.txt', 'FileName', 'C:/a path/file.txt'),
    ])
def testConvertString2ValueTakesWhatItSays(text, valueType, expected, comboLists):
    [value, errorMessage] = gui.ConvertString2Value(text, valueType, [1], comboLists)
    assert errorMessage == ''
    assert value == expected


@pytest.mark.parametrize(('text', 'valueType'), [
    ('-1', 'PReal'),                           #must be > 0
    ('0', 'PReal'),
    ('-0.5', 'UReal'),                         #must be >= 0
    ('-1', 'PFloat'),
    ('-0.5', 'UFloat'),
    ('-3', 'UInt'),                            #must be >= 0
    ('0', 'PInt'),                             #must be > 0
    ])
def testConvertString2ValueReportsAValueOutOfRange(text, valueType, comboLists):
    """the range is in the type name, and the message has to name it - this is what the dialog
    prints when a value is rejected"""
    [_, errorMessage] = gui.ConvertString2Value(text, valueType, [1], comboLists)
    assert errorMessage != ''
    assert valueType in errorMessage


def testConvertString2ValueReadsAnEnumFromItsName(comboLists):
    [value, errorMessage] = gui.ConvertString2Value('OutputVariableType.Displacement',
                                                    'OutputVariableType', [1], comboLists)
    assert errorMessage == ''
    assert value == exudyn.OutputVariableType.Displacement


def testConvertString2ValueReportsATypeItDoesNotKnow(comboLists):
    [_, errorMessage] = gui.ConvertString2Value('7', 'NoSuchType', [1], comboLists)
    assert 'unknown type' in errorMessage


@pytest.mark.parametrize(('value', 'valueType', 'size', 'expected'), [
    (True, 'bool', [1], 'True'),
    (3, 'Index', [1], '3'),
    ('text', 'String', [1], 'text'),
    ([1, 2, 3], 'IndexArray', [3], '[1, 2, 3]'),
    ])
def testConvertValue2StringWritesWhatCanBeReadBack(value, valueType, size, expected):
    assert gui.ConvertValue2String(value, valueType, size) == expected


def testConvertValue2StringWritesFloatsAsSinglePrecision():
    """the C++ side stores these as float, and a dialog that shows 17 digits of a number that
    only has 7 invites a change that is not one"""
    assert gui.ConvertValue2String(1.0/3.0, 'float', [1]) == '0.33333334'
    assert gui.ConvertValue2String([1.0/3.0], 'VectorFloat', [1]) == '[0.33333334]'


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#what CheckType lets through, which is what reaches the settings structure

@pytest.mark.parametrize(('text', 'valueType', 'size'), [
    ('1.5', 'float', [1]),
    ('anyName.txt', 'FileName', [1]),
    ('7', 'Index', [1]),
    ('[1, 2, 3]', 'IndexArray', [3]),
    ('[[1, 2], [3, 4]]', 'MatrixFloat', [2, 2]),
    ])
def testCheckTypeAcceptsWhatItShould(text, valueType, size):
    [isValid, message] = gui.CheckType(text, valueType, size)
    assert isValid, message


@pytest.mark.parametrize(('text', 'valueType', 'size', 'inMessage'), [
    ('not a number', 'float', [1], 'float'),
    ('', 'FileName', [1], 'empty'),
    (' leadingSpace.txt', 'FileName', [1], 'SPACE'),
    ('file*name?.txt', 'FileName', [1], 'invalid character'),
    ('-3', 'Index', [1], 'positive'),
    ('[1, 2]', 'IndexArray', [3], 'length 3'),
    ('[-1, 2, 3]', 'IndexArray', [3], 'positive integer'),
    ('[[1, 2, 3], [4, 5, 6]]', 'MatrixFloat', [2, 2], 'columns'),
    ('[[1, 2]]', 'MatrixFloat', [2, 2], 'rows'),
    ('[1, 2', 'IndexArray', [3], 'brackets'),
    ])
def testCheckTypeRejectsWithAMessageThatSaysWhy(text, valueType, size, inMessage):
    [isValid, message] = gui.CheckType(text, valueType, size)
    assert not isValid
    assert inMessage in message


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the combo box lists, which decide whether a value is picked or typed

def testEveryEnumSettingCouldBePickedFromAList(leaves, comboLists):
    """an enum that has no list is edited as free text, where a typo is a silent wrong value.
    KNOWN GAP: GetComboBoxListsDict names three enum types by hand, and
    timeIntegration.explicitIntegration.dynamicSolverType is not one of them (revision2026b step
    RG6.2.3)"""
    missing = sorted({leafType + ' (' + path + ')' for (path, _, leafType, _) in leaves
                      if leafType.endswith('Type') and leafType not in comboLists})
    if missing != []:
        pytest.xfail('enum types without a list: ' + ', '.join(missing))


def testTheListsHoldTheValuesTheyOfferAsStrings(comboLists):
    """the dialog compares str(value) with what the combo box shows, so the entries must be the
    exudyn values and not their names"""
    assert comboLists['bool'] == [True, False]
    for name in ['OutputVariableType', 'LinearSolverType', 'ItemType']:
        values = comboLists[name]
        assert len(values) > 1
        assert all(str(value).startswith(name + '.') for value in values)
