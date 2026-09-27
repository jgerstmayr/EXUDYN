#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The layer under the settings dialog: the four functions of exudyn.misc.GUI that decide
#           what a typed value becomes. They need no window, which is why they can be tested at
#           all - everything above them opens one (#2596).
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
    """every editable value of a settings structure, as (path, value, type, size)

    The walk itself is gui.SettingsLeafList (#2605), where the
    dialog began to need it as well; this keeps the four fields the tests below read."""
    return [(leafPath, value, leafType, size)
            for (leafPath, value, _, leafType, size, _)
            in gui.SettingsLeafList(dictionary, path)]


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


#What does NOT survive the round trip, by path. The list is meant to SHRINK: a path that starts
#working has to be taken out here, and a path that stops working is a new failure. It held five
#entries when this test was written (#2597) - every enum value, because CheckType had no branch
#for one, and every absolute path, because ':' was not a valid file name character - and
#all five work now.
#THE FOUR SETTINGS THE DIALOG CANNOT UNSET (#2689). A render window
#position of (-1,-1) means "wherever the window manager puts it", and the dialog's rule for an
#IndexArray - the type it shares with renderWindowSize, so nothing can tell them apart - is that its
#values are not negative. That rule is the only one there is, because the C++ side accepts a negative
#sensor number and a negative window size, so it stays: a user SETS a position in the dialog, which is
#positive and passes, and unsets it in the settings file or from a script. This list is what keeps the
#round trip honest about the gap rather than silent about it.
knownRoundTripGaps = ['view0.window.renderWindowPosition', 'view1.window.renderWindowPosition',
                      'view2.window.renderWindowPosition', 'view3.window.renderWindowPosition']


def testEveryCurrentValueSurvivesTheRoundTrip(leaves, comboLists):
    """value -> string -> value, for every setting there is: what the dialog does when it is
    opened and closed without touching anything"""
    failures = []
    survived = []
    for (path, value, leafType, size) in leaves:
        asString = gui.ConvertValue2String(value, leafType, size)
        #with the combo lists, which is what the dialog has: an enum is a value of a list
        [isValid, message] = gui.CheckType(asString, leafType, size, comboLists)
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
#the line that sets a setting: what the dialog offers to
#copy into a script

def testValueLiteralWritesWhatPythonReadsBack(comboLists):
    """a string is quoted, an enum carries the module it lives in, a number stands as it is"""
    assert gui.ValueLiteral('some text', 'String', comboLists) == "'some text'"
    assert gui.ValueLiteral('C:/a path/file.txt', 'FileName', comboLists) == "'C:/a path/file.txt'"
    assert gui.ValueLiteral('True', 'bool', comboLists) == 'True'
    assert gui.ValueLiteral('16.0', 'float', comboLists) == '16.0'
    assert gui.ValueLiteral('[1.0, 2.0]', 'VectorFloat', comboLists) == '[1.0, 2.0]'
    assert (gui.ValueLiteral('OutputVariableType.Displacement', 'OutputVariableType', comboLists)
            == 'exu.OutputVariableType.Displacement')


def testEveryCurrentValueCanBeWrittenAsALineThatSetsIt(leaves, comboLists):
    """the copy of the dialog is only worth having if what it copies can be run: every literal it
    writes has to be one Python evaluates back to the value it came from"""
    import ast
    failures = []
    for (path, value, leafType, size) in leaves:
        literal = gui.ValueLiteral(gui.ConvertValue2String(value, leafType, size),
                                   leafType, comboLists)
        if literal.startswith('exu.'):
            continue                    #an enum needs the module, which ast does not have
        try:
            ast.literal_eval(literal)
        except (ValueError, SyntaxError) as exception:
            failures.append(path + ' (' + leafType + '): ' + literal + ' - ' + str(exception))
    assert failures == [], '\n'.join(failures)


def testTheSettingsPrefixIsTheNameAScriptUses():
    assert gui.SettingsPrefix(exudyn.VisualizationSettings()) == 'SC.visualizationSettings'
    assert gui.SettingsPrefix(exudyn.SimulationSettings()) == 'simulationSettings'


def testTheLeafListCoversTheWholeStructure():
    """if the walk ever stops at a sub-structure, everything built on it goes quietly wrong"""
    leaves = gui.SettingsLeafList(exudyn.VisualizationSettings().GetDictionaryWithTypeInfo())
    paths = [path for (path, _, _, _, _, _) in leaves]
    assert len(paths) > 400
    assert len(set(paths)) == len(paths)
    assert 'openGL.lineWidth' in paths              #a leaf two levels down
    assert 'openGL.advanced.textLineWidth' in paths #and one three levels down
    assert all('.' in path for path in paths)   #every setting sits in a folder
    #the string is the one the dialog shows in the cell
    (_, value, valueString, leafType, size, _) = leaves[0]
    assert valueString == gui.ConvertValue2String(value, leafType, size)


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#what differs from a reference: the coloured rows of the dialog and
#the two windows it copies from are this one comparison

@pytest.fixture(scope='module')
def visualizationLeaves():
    return gui.SettingsLeafList(exudyn.VisualizationSettings().GetDictionaryWithTypeInfo())


def testNothingDiffersFromItself(visualizationLeaves, comboLists):
    reference = gui.SettingsValueStrings(
        exudyn.VisualizationSettings().GetDictionaryWithTypeInfo())
    assert gui.SettingsCodeLines(visualizationLeaves, reference,
                                 'SC.visualizationSettings', comboLists) == []


def testOneChangedValueGivesOneLineThatSetsIt(visualizationLeaves, comboLists):
    """the line has to name the path and the new value, and it has to be Python"""
    import ast
    reference = gui.SettingsValueStrings(
        exudyn.VisualizationSettings().GetDictionaryWithTypeInfo())
    reference['openGL.lineWidth'] = 'a value it never had'

    lines = gui.SettingsCodeLines(visualizationLeaves, reference, 'SC.visualizationSettings',
                                 comboLists)
    assert len(lines) == 1
    (path, line) = lines[0]
    assert path == 'openGL.lineWidth'
    assert line.startswith('SC.visualizationSettings.openGL.lineWidth = ')
    parsed = ast.parse(line).body[0]
    assert isinstance(parsed, ast.Assign)


def testAPathTheReferenceDoesNotKnowCountsAsUnchanged(visualizationLeaves, comboLists):
    """a settings structure that gained a value must not report all of it as changed"""
    reference = gui.SettingsValueStrings(
        exudyn.VisualizationSettings().GetDictionaryWithTypeInfo())
    del reference['openGL.lineWidth']
    assert gui.SettingsCodeLines(visualizationLeaves, reference,
                                 'SC.visualizationSettings', comboLists) == []


def testEveryDifferenceIsALineThatRuns(comboLists):
    """the worst case of the copy: a structure where EVERY value differs from the reference, so
    that every type is written as a line - each one has to parse"""
    import ast
    leaves = gui.SettingsLeafList(exudyn.SimulationSettings().GetDictionaryWithTypeInfo())
    lines = gui.SettingsCodeLines(leaves, {}, 'simulationSettings', comboLists)
    assert lines == []                      #an empty reference means nothing is known to differ

    reference = {path: 'a value it never had' for (path, _, _, _, _, _) in leaves}
    lines = gui.SettingsCodeLines(leaves, reference, 'simulationSettings', comboLists)
    assert len(lines) == len(leaves)
    for (_, line) in lines:
        parsed = ast.parse(line).body[0]    #raises if the dialog writes something Python rejects
        assert isinstance(parsed, ast.Assign)


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#WHAT a difference is measured against (#2612)

def testTheDialogNeverCreatesASystemContainer():
    """the defect this test exists for (#2625): MainSystemContainer() ATTACHES to the running
    render engine in its constructor and DETACHES in its destructor, so a temporary container -
    which is what the dialog used to create to read the defaults - takes the render window away
    from the container that owns it, and the window closes. The reference is the structure's own
    constructor, and nothing here may allocate a container."""
    import io                                                                  # noqa: PLC0415
    with io.open(gui.__file__, encoding='utf-8') as file:
        source = file.read()
    assert 'exudyn.SystemContainer()' not in source, (
        'the settings dialog must not create a SystemContainer: it attaches to and detaches from'
        ' the render engine (#2625)')


def testASystemContainerInitialisesNothingBeyondTheDefaults(comboLists):
    """the point: a fresh SystemContainer must show NO difference
    to a plain exu.VisualizationSettings(). Until then a container dimmed three lights and filled
    ten raytracer materials after construction, so the dialog reported 59 settings as changed that
    nobody had touched, and the documentation printed defaults the renderer did not use. The
    values are defaults of the structure now; containerInitialisedSettings is where an exception
    would be named, and it has to stay empty."""
    container = exudyn.SystemContainer()
    leaves = gui.SettingsLeafList(container.visualizationSettings.GetDictionaryWithTypeInfo())
    plain = gui.SettingsValueStrings(
        gui.DefaultSettingsDictionary(container.visualizationSettings))

    differences = gui.SettingsCodeLines(leaves, plain, 'SC.visualizationSettings', comboLists)
    unexpected = [path for (path, _) in differences
                  if not any(path.startswith(known)
                             for known in gui.containerInitialisedSettings)]
    assert unexpected == [], ('a SystemContainer initialises these beyond the defaults: '
                              + ', '.join(unexpected) + ' - they belong in'
                              ' definitions/structureDefsVisualizationSettings.py')
    assert gui.containerInitialisedSettings == [], (
        'nothing should need this list any more')


def testTheRendererLinkIsAMemberOfTheCppSide():
    """exudyn.special.currentRendererSystemContainer, not an entry of exudyn.sys (#2692)

    A dictionary entry can hold anything, and #2691 was exactly that: a Python subclass under the
    module's own name made the isinstance() that guarded the entry False, and every dialog that needs
    the container stopped working without a word. A typed member cannot be wrong about its own type.
    """
    assert 'currentRendererSystemContainer' not in exudyn.sys

    container = exudyn.SystemContainer()
    assert exudyn.special.currentRendererSystemContainer is container
    assert gui.GetRendererSystemContainer() is container

    #it is the C++ side's, and Python does not get to set it
    with pytest.raises(AttributeError):
        exudyn.special.currentRendererSystemContainer = None


def testReadingTheDefaultsLeavesTheRendererItsContainer():
    """creating a SystemContainer REPLACES the renderer's link, and the defaults used to be read
    from a throw-away one - so the settings dialog handed the renderer a different container: the
    redraw signal went to it, the dialog read its window settings from it, and once it was
    collected, touching it was an access violation (#2623). The link is
    exudyn.special.currentRendererSystemContainer (#2692)."""
    container = exudyn.SystemContainer()
    assert exudyn.special.currentRendererSystemContainer is container

    gui.DefaultSettingsDictionary(container.visualizationSettings)

    assert exudyn.special.currentRendererSystemContainer is container
    #and it must still be usable, which is what the access violation took away
    assert container.visualizationSettings.dialogs.alphaTransparency >= 0.


def testASettingsStructureThatIsNotOnTheContainerStillHasDefaults():
    """simulationSettings has no SystemContainer to come from, and must not lose its reference"""
    reference = gui.SettingsValueStrings(
        gui.DefaultSettingsDictionary(exudyn.SimulationSettings()))
    assert reference != {}
    assert 'timeIntegration.endTime' in reference


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#what a FOLDER of the settings tree is (#2615)

def testEverySettingsFolderSaysWhatItIs():
    """the dialog shows this when the mouse rests on a folder; before #2615 the dictionary
    carried a description for the leaves only"""
    missing = []

    def Walk(dictionary, path):
        if 'structureDescription' not in dictionary:
            missing.append(path + ' (no description at all)')
        elif str(dictionary['structureDescription']).strip() == '':
            missing.append(path + ' (empty description)')
        for (key, value) in dictionary.items():
            if isinstance(value, dict) and 'itemIdentifier' not in value:
                Walk(value, path + '.' + key)

    for (name, structure) in [('visualizationSettings', exudyn.VisualizationSettings()),
                              ('simulationSettings', exudyn.SimulationSettings())]:
        Walk(structure.GetDictionaryWithTypeInfo(), name)
    assert missing == [], 'folders without a description: ' + ', '.join(missing)


def testTheDescriptionOfAFolderIsNoSetting(leaves):
    """it is a string beside the values, so everything that walks the tree must step over it"""
    assert all(not path.endswith('structureDescription') for (path, _, _, _) in leaves)


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#find a setting: what the dialog offers when a user does not know
#which folder a setting sits in

def testFindPutsTheNameHitsFirst(visualizationLeaves):
    """the maintainer's requirement in one assertion: names before descriptions. 'shadow' is both
    a setting and a word many descriptions use"""
    hits = gui.FindMatches(visualizationLeaves, 'shadow')
    assert hits != []
    nameHits = [path for (path, _) in hits if 'shadow' in path.split('.')[-1].lower()]
    assert nameHits != []
    assert [path for (path, _) in hits][:len(nameHits)] == nameHits


def testFindReadsTheDescriptionsToo(visualizationLeaves):
    """a word that is in no name at all still has to lead somewhere, and the hit says why"""
    hits = gui.FindMatches(visualizationLeaves, 'transparency')
    fromDescription = [(path, label) for (path, label) in hits
                       if 'transparen' not in path.lower()]
    assert fromDescription != []
    assert all('...' in label for (_, label) in fromDescription)


def testFindIgnoresCaseAndFindsNothingForNothing(visualizationLeaves):
    assert (gui.FindMatches(visualizationLeaves, 'LINEWIDTH')
            == gui.FindMatches(visualizationLeaves, 'linewidth'))
    assert gui.FindMatches(visualizationLeaves, '   ') == []
    assert gui.FindMatches(visualizationLeaves, 'zzz no such setting zzz') == []


def testEveryFindHitNamesASettingThatExists(visualizationLeaves):
    """the hit is what the dialog jumps to, so a path that is in no tree is a dead end"""
    paths = {path for (path, _, _, _, _, _) in visualizationLeaves}
    for searchText in ['color', 'size', 'draw', 'openGL']:
        for (path, _) in gui.FindMatches(visualizationLeaves, searchText):
            assert path in paths


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the combo box lists, which decide whether a value is picked or typed

def testEveryEnumSettingCouldBePickedFromAList(leaves, comboLists):
    """an enum that has no list is edited as free text, where a typo is a silent wrong value.
    GetComboBoxListsDict named three enum types by hand, and
    timeIntegration.explicitIntegration.dynamicSolverType was not one of them; it builds the
    lists from the module now, so a new enum arrives here by itself"""
    missing = sorted({leafType + ' (' + path + ')' for (path, _, leafType, _) in leaves
                      if leafType.endswith('Type') and leafType not in comboLists})
    assert missing == [], 'enum types without a list: ' + ', '.join(missing)


def testTheListsHoldTheValuesTheyOfferAsStrings(comboLists):
    """the dialog compares str(value) with what the combo box shows, so the entries must be the
    exudyn values and not their names"""
    assert comboLists['bool'] == [True, False]
    for name in ['OutputVariableType', 'LinearSolverType', 'ItemType']:
        values = comboLists[name]
        assert len(values) > 1
        assert all(str(value).startswith(name + '.') for value in values)


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#how large a row is (#2631): dialogs.fontScaling only worked at 0,
#because the row height and the column width were computed from systemScaling instead of from the
#font that is really drawn - 13 pixels for a font with a linespace of 16 to 18, and an INTEGER
#column factor that stayed at 1 for every value below 1.5

def testTheFontSizeFollowsTheScalingAndNeverCollapses():
    assert gui.DialogFontSize(1.) == gui.treeviewDefaultFontSize
    assert gui.DialogFontSize(2.) == 2*gui.treeviewDefaultFontSize
    assert gui.DialogFontSize(0.) >= 6, 'a font of zero points is not a font'
    assert gui.DialogFontSize(-1.) >= 6


#ONE root for the whole process. A second tk.Tk() after the first was destroyed fails on this
#Windows build - the tests that ran later reported "no display" although the display was there -
#and under "pytest -n 8" a worker that built widgets in a second root crashed outright. So the
#root is created once, kept, and never destroyed: the process ends and takes it with it.
_tkRoot = []


def TkRootOrSkip():
    """the withdrawn root of this process; no window is ever mapped, and no display is a skip"""
    import tkinter as tk                                                        # noqa: PLC0415
    if len(_tkRoot) == 0:
        try:
            root = tk.Tk()
        except Exception:                    # noqa: BLE001 - any display problem is a skip
            _tkRoot.append(None)
        else:
            root.withdraw()
            _tkRoot.append(root)
    if _tkRoot[0] is None:
        pytest.skip('no tkinter display available')
    return _tkRoot[0]


def testTheRowHeightAndTheColumnsFollowTheFont():
    """the point of the step: both are MEASURED, so any fontScaling is usable"""
    root = TkRootOrSkip()
    try:
        [smallRow, smallColumns] = gui.DialogRowMetrics(root, 1.)
        [largeRow, largeColumns] = gui.DialogRowMetrics(root, 2.)

        #a row must hold the line it draws
        for fontFactor in [1., 1.25, 1.5, 2.]:
            [rowHeight, _] = gui.DialogRowMetrics(root, fontFactor)
            import tkinter.font as tkFont                                       # noqa: PLC0415
            font = tkFont.Font(root=root, size=gui.DialogFontSize(fontFactor))
            assert rowHeight >= font.metrics('linespace'), (
                'the text is clipped at fontScaling=' + str(fontFactor))

        assert largeRow > smallRow, 'a larger font must get taller rows'
        assert largeColumns > smallColumns, 'a larger font must get wider columns'
        assert smallColumns == 1., 'the unscaled font must leave the columns as they were'
    finally:
        pass                 #the root is shared and outlives the test


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#how large the dialogs come out when no renderer is running (#2634):
#'python -m exudyn dialogs' opens the same windows as the render window does, and they were
#bigger and blurred, because the process was not DPI aware and the display scaling was read as 1

def testTheProcessCanBeMadeDpiAware():
    """on Windows this is what keeps the dialog sharp; elsewhere it is a no-op that says True"""
    assert gui.MakeProcessDpiAware() in [True, False]   #False only on an old Windows


def testTheDisplayScalingIsAskedOfTkinterWhenNoRendererCanBeAsked(monkeypatch):
    root = TkRootOrSkip()
    #another test in this file creates a container, and the renderer branch would win. The link
    #belongs to the C++ side (#2692) and cannot be taken away from Python,
    #so what is replaced is the one function that reads it
    monkeypatch.setattr(gui, 'GetRendererSystemContainer', lambda: None)
    try:
        withoutRoot = gui.GetExudynDisplayScaling()
        withRoot = gui.GetExudynDisplayScaling(root)
        assert withoutRoot == 1, 'nothing to ask, so the old answer stands'
        assert withRoot >= 1.
        #it is the display's, not a guess: tkinter measures an inch against 96 dpi
        assert abs(withRoot - max(1., root.winfo_fpixels('1i') / 96.)) < 1e-9
    finally:
        pass                 #the root is shared and outlives the test


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#THE COMMAND WINDOW (#2654). It opens a window, so the suite cannot run it - but what it does
#with a command is one function, and that is where the fault was.
def testTheCommandWindowRunsInTheScopeOfTheModel():
    """it says "operates in global scope of you Python model", and for a while it did not"""
    import __main__
    assert gui.ModelScope() is vars(__main__)


def testACommandOfTheWindowSeesTheModelAndItsAssignmentSurvives():
    """exec(code, globals(), locals()) put an assignment into the handler and lost it"""
    import __main__
    scope = gui.ModelScope()
    scope['exudynTestModelVariable'] = 17
    try:
        exec('exudynTestModelResult = exudynTestModelVariable * 2', gui.ModelScope())
        assert getattr(__main__, 'exudynTestModelResult', None) == 34, \
            'the command sees the model AND writes back into it'
    finally:
        for name in ['exudynTestModelVariable', 'exudynTestModelResult']:
            scope.pop(name, None)


def testTheModuleNamespaceIsNotTheModelNamespace():
    """the fault itself: globals() inside exudyn.misc.GUI is not where a model lives"""
    assert gui.ModelScope() is not vars(gui)

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the columns are fractions of the dialog width (#2667) and Ctrl with the
#wheel changes the font size (#2668). Both are tested in a WITHDRAWN root: the widgets are
#real, no window is ever mapped
def testTheColumnFractionsAreWhatWasConfigured():
    assert gui.ColumnWidthFractions([0.31, 0.18, 0.11]) == (0.31, 0.18, 0.11)


def testTheColumnFractionsLeaveRoomForTheDescription():
    """three independent settings can ask for more than the whole dialog; the description column
    must not disappear"""
    fractions = gui.ColumnWidthFractions([0.6, 0.5, 0.4])
    assert sum(fractions) == pytest.approx(0.9)
    assert fractions[0] > fractions[1] > fractions[2], 'the proportions are kept'


def testAColumnCannotBeGivenZeroWidth():
    assert gui.ColumnWidthFractions([0., -1., 0.]) == (0.05, 0.05, 0.05)


@pytest.fixture(scope='module')
def tkRoot():
    """ONE withdrawn root for the tests below that build a whole settings tree

    SKIPPED IN A PARALLEL RUN. Building the tree inside an xdist worker crashes the worker at the
    Tk level - "node down: Not properly terminated", no Python traceback - about one run in three,
    and it is the construction and not the number of calls: the clamp test was rewritten from 180
    font changes to two and it made no difference. The same tests are stable in a plain `pytest`,
    which is how they are meant to be run; what they cover is the dialog, and a dialog is not what
    eight workers are for. A crash that is reported as a failing test costs an hour of somebody's
    day, which is why this is a skip and not a retry.
    """
    #SKIPPED IN A PARALLEL RUN, and only here: building a settings tree inside an xdist worker
    #crashes the worker at the Tk level - "node down: Not properly terminated", no Python
    #traceback - about one run in three. Measured, not assumed: with one root per process the
    #serial run went from three skips to 73 passed, and the parallel run still crashes. A crash
    #reported as a failing test costs an hour of somebody's day, so these three run serially,
    #which is how a dialog is used anyway.
    import os                                                                   # noqa: PLC0415
    if os.environ.get('PYTEST_XDIST_WORKER', '') != '':
        pytest.skip('a settings tree crashes an xdist worker; run pytest without -n for these')
    root = TkRootOrSkip()
    yield root
    pass                 #the root is shared and outlives the test


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#A STORED GEOMETRY IS USED (#2686). RestoreWindowGeometry asked
#dialogs.storeDialogPositions first, so what the store button (#2685) wrote was never read back.
#A withdrawn window reports 1x1+0+0 whatever it was given, so what is tested is what the function
#ASKS the window manager for
def RequestedGeometry(name, width, height):
    """what RestoreWindowGeometry asks for, in a withdrawn window of its own"""
    import tkinter as tk                                                         # noqa: PLC0415

    root = TkRootOrSkip()
    requested = []
    original = tk.Toplevel.geometry

    def Recording(self, newGeometry=None):
        if newGeometry is not None:
            requested.append(newGeometry)
        return original(self, newGeometry)

    window = tk.Toplevel(root)
    window.withdraw()
    tk.Toplevel.geometry = Recording
    try:
        gui.RestoreWindowGeometry(window, name, width, height)
    finally:
        tk.Toplevel.geometry = original
        screen = [window.winfo_vrootx(), window.winfo_vrooty(),
                  max(window.winfo_vrootwidth(), window.winfo_screenwidth()),
                  max(window.winfo_vrootheight(), window.winfo_screenheight())]
        window.destroy()
    return (requested[-1] if requested else '', screen)


@pytest.fixture
def storedGeometry(tmp_path, monkeypatch):
    """a settings file of this test's own, holding one dialog geometry"""
    from exudyn.misc import overrideSettings

    fileName = str(tmp_path / 'config.json')
    monkeypatch.setenv('EXUDYN_CONFIG_FILE', fileName)
    monkeypatch.delenv('EXUDYN_NO_USER_SETTINGS', raising=False)

    def Store(size, position):
        overrideSettings.Settings().clear()
        overrideSettings.StoreSection('dialogs', {overrideSettings.DialogKey('a dialog'):
                                                  {'size': size, 'position': position}})
        return 'a dialog'
    yield Store
    overrideSettings.Settings().clear()


def testAStoredGeometryIsUsedWhateverTheFlagSays(storedGeometry):
    """the store button writes it without the flag, so the flag must not decide whether it is read

    Measured before the step: the window was asked for the default 900x700 while 1122x1751+7+14 was
    stored."""
    name = storedGeometry([600, 500], [40, 50])
    (requested, _) = RequestedGeometry(name, 900, 700)
    assert requested == '600x500+40+50'


def testWithoutAStoredGeometryTheDialogGetsTheSizeItAsksedFor(storedGeometry):
    storedGeometry([600, 500], [40, 50])
    (requested, _) = RequestedGeometry('a dialog nobody stored', 900, 700)
    assert requested == '900x700'


def testAStoredSizeIsCutDownToThisScreen(storedGeometry):
    """a dialog taller than the screen has its button row - and its close button - off the bottom"""
    name = storedGeometry([30000, 30000], [0, 0])
    (requested, screen) = RequestedGeometry(name, 900, 700)
    (size, _) = requested.split('+', 1)
    (width, height) = [int(part) for part in size.split('x')]
    assert width <= screen[2] and height <= screen[3]
    assert width == screen[2] - 2 * gui.dialogScreenMargin
    assert height == screen[3] - 2 * gui.dialogScreenMargin


def testAnUnreachablePositionIsStillRefused(storedGeometry):
    """the rule of #2608 stands: the size comes back, the position only if it is reachable"""
    name = storedGeometry([600, 500], [-30000, -30000])
    (requested, _) = RequestedGeometry(name, 900, 700)
    assert requested == '600x500'                       #the size, and no position


def testADialogWhoseSizeComesFromItsLayoutIsLeftAlone(storedGeometry):
    """RestoreWindowGeometry(window, name) without a size: the InteractiveDialog of the
    SolutionViewer computes its size from its widgets, so nothing may be imposed on it unless
    something IS stored (#2689)"""
    storedGeometry([600, 500], [40, 50])

    (requested, _) = RequestedGeometry('a dialog nobody stored', None, None)
    assert requested == '', 'a window with nothing stored must be left to its layout'

    #and with something stored, that is used - size and position, as for any other dialog
    (requested, _) = RequestedGeometry('a dialog', None, None)
    assert requested == '600x500+40+50'


def testTheFlagIsAskedOfTheStructureBeingEdited():
    """python -m exudyn dialogs has no SystemContainer, so the flag was False however it was set

    That is why such a dialog could never store itself when it closed."""
    structure = exudyn.VisualizationSettings()
    structure.dialogs.storeDialogPositions = True
    assert gui.StoreDialogPositions(structure)
    structure.dialogs.storeDialogPositions = False
    assert not gui.StoreDialogPositions(structure)
    assert not gui.StoreDialogPositions()               #nothing to ask: no renderer, no structure


def TreeDialogOrSkip(root, columnWidths=None):
    """the settings tree of a visualizationSettings dialog, inside a withdrawn root"""
    settings = exudyn.VisualizationSettings()
    [systemScaling, fontFactor] = gui.DialogScaling(root)
    [textHeight, columnScale] = gui.DialogRowMetrics(root, fontFactor)
    return gui.TkinterEditDictionaryWithTypeInfo(
        parent=root, settingsStructure=settings, dictionaryTypesT=gui.GetComboBoxListsDict(exudyn),
        updateOnChange=False, treeOpen=False, textHeight=textHeight,
        systemScaling=systemScaling, fontFactor=fontFactor, columnScale=columnScale,
        columnWidths=columnWidths)


def testTheColumnsGetTheirShareOfTheDialog(tkRoot):
    tree = TreeDialogOrSkip(tkRoot, [0.4, 0.2, 0.1]).tree
    widths = [tree.column(name, 'width') for name in ['#0', 'value', 'type', 'description']]
    total = float(sum(widths))
    assert widths[0] / total == pytest.approx(0.4, abs=0.02)
    assert widths[1] / total == pytest.approx(0.2, abs=0.02)
    assert widths[2] / total == pytest.approx(0.1, abs=0.02)
    assert widths[3] / total == pytest.approx(0.3, abs=0.02), 'the description takes the rest'


def testCtrlAndTheWheelChangeTheFontSize(tkRoot):
    dialog = TreeDialogOrSkip(tkRoot)
    (before, beforeRow) = (dialog.fontFactor, dialog.textHeight)
    dialog.ChangeFontSize(1.1)
    assert dialog.fontFactor > before
    assert dialog.textHeight >= beforeRow, 'the row height follows the font'
    dialog.ChangeFontSize(1 / 1.1)
    assert dialog.fontFactor == pytest.approx(before, rel=1e-6)


def testTheFontSizeCannotBeScrolledAwayInEitherDirection(tkRoot):
    """a dialog whose font is two pixels tall cannot be read back to a usable size

    The clamp is checked AT its boundary rather than by scrolling 180 times: every call measures a
    font and reconfigures the ttk style, and that loop crashed a parallel pytest worker at the Tk
    level about one run in ten. A user changes the font a few times; the test does what the code
    has to get right.
    """
    dialog = TreeDialogOrSkip(tkRoot)
    dialog.fontFactor = 0.41
    dialog.ChangeFontSize(1 / 1.1)
    assert dialog.fontFactor == pytest.approx(0.4), 'the floor'
    dialog.fontFactor = 3.9
    dialog.ChangeFontSize(1.1)
    assert dialog.fontFactor == pytest.approx(4.), 'the ceiling'


def testStorePositionsStoresEveryOpenWindow(tkRoot, tmp_path, monkeypatch):
    """the store positions button lists and stores every open window, not only its own dialog: the
    other interactive dialogs and the PlotSensor windows (#2719); the render window is added when a
    renderer is running, which a test does not have"""
    import json
    import types
    from exudyn.misc import overrideSettings
    import exudyn.interactive as interactive
    import exudyn.plot as plot

    fileName = str(tmp_path / 'config.json')
    monkeypatch.setenv('EXUDYN_CONFIG_FILE', fileName)
    monkeypatch.delenv('EXUDYN_NO_USER_SETTINGS', raising=False)
    with open(fileName, 'w', encoding='utf-8') as file:
        json.dump({'version': overrideSettings.fileFormatVersion}, file)
    overrideSettings.Settings().clear()

    viewer = types.SimpleNamespace(dialogName='Solution Viewer',
                                   tkWindow=types.SimpleNamespace(geometry=lambda: '500x300+20+30'))
    monkeypatch.setattr(interactive, 'openDialogs', [viewer])
    monkeypatch.setattr(plot, 'PlotWindowGeometries', lambda: [('PlotSensor 1', '640x480+700+30')])

    dialog = TreeDialogOrSkip(tkRoot)
    shown = {}

    def Capture(title, lines, description, confirm=None):
        shown['lines'] = [line for (_, line) in lines]
        confirm[1]()                                   #press "store"
    monkeypatch.setattr(dialog, 'ShowCodeLines', Capture)
    dialog.OnStorePositions()

    assert any('Solution Viewer: 500x300+20+30' in line for line in shown['lines'])
    assert any('PlotSensor 1: 640x480+700+30' in line for line in shown['lines'])
    dialogs = overrideSettings.Load()['dialogs']
    assert dialogs[overrideSettings.DialogKey('Solution Viewer')] == {'size': [500, 300], 'position': [20, 30]}
    assert dialogs[overrideSettings.DialogKey('PlotSensor 1')] == {'size': [640, 480], 'position': [700, 30]}
    overrideSettings.Settings().clear()
