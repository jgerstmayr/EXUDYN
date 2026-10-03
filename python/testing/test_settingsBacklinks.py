#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  A settings structure that Python constructs itself must work. Until #2603 two
#           lines segfaulted the process:
#
#               import exudyn as exu
#               exu.VisualizationSettings().general.drawWorldBasis     #exit code 139
#
#           Every one of the 93 deprecated members forwards to its replacement through
#           backlink->..., and Init(), which sets those backlinks, was called in exactly one
#           place - for the settings that belong to a SystemContainer. A standalone structure had
#           every backlink at nullptr. The top class links itself now, and it keeps its own links
#           when it is copied.
#
#           These tests are deliberately crude: they read EVERY member of a standalone structure,
#           deprecated or not, because a segfault cannot be caught and the only way to find one is
#           to touch everything. A crash takes the test process down, which is the report.
#
# Usage:    pytest python/testing/test_settingsBacklinks.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-23
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import warnings

import pytest

import exudyn

#measured on 2026-09-23: 314 readable members of visualizationSettings, 93 of them deprecated.
#The count may FALL as deprecated members expire; it is here so that a drop to zero - which would
#make the walk below prove nothing - is noticed
minimumDeprecatedMembers = 80


def SettingsGroups(settings):
    """the sub-structures of a settings structure, by name"""
    groups = []
    for name in dir(settings):
        if name.startswith('_'):
            continue
        group = getattr(settings, name)
        if type(group).__name__ in ['builtin_function_or_method', 'method']:
            continue
        groups.append((name, group))
    return groups


def ReadEveryMember(settings):
    """read every member of every sub-structure; returns (read, deprecated, problems); every deprecated member
    warns here, not only the first of the session (#2804)"""
    warnOnceStored = exudyn.special.deprecations.warnOnce
    exudyn.special.deprecations.warnOnce = False
    try:
        return ReadGroups(settings)
    finally:
        exudyn.special.deprecations.warnOnce = warnOnceStored


def ReadGroups(settings):
    (read, deprecated, problems) = (0, 0, [])
    for (groupName, group) in SettingsGroups(settings):
        for name in dir(group):
            if name.startswith('_'):
                continue
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter('always')
                try:
                    getattr(group, name)
                except Exception as exception:          # noqa: BLE001 - that is the finding
                    problems.append(groupName + '.' + name + ': ' + type(exception).__name__
                                    + ': ' + str(exception))
                    continue
            read += 1
            if any(issubclass(entry.category, DeprecationWarning) for entry in caught):
                deprecated += 1
    return (read, deprecated, problems)


def testAStandaloneVisualizationSettingsCanBeRead():
    """the two lines of #2603, and the other 92 deprecated members with them"""
    (read, deprecated, problems) = ReadEveryMember(exudyn.VisualizationSettings())
    assert problems == [], 'a standalone VisualizationSettings cannot be read: ' + str(problems[:5])
    assert read > 200, 'the walk found almost nothing - has the structure changed?'
    assert deprecated >= minimumDeprecatedMembers, (
        'only ' + str(deprecated) + ' deprecated members were reached, so this test no longer'
        ' proves what it was written for')


def testTheReportedTwoLines():
    """the report itself, spelled out, because that is what a user typed"""
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', DeprecationWarning)
        assert exudyn.VisualizationSettings().general.drawWorldBasis in [True, False]


def testWritingADeprecatedMemberReachesItsReplacement():
    """a deprecated member is not merely readable: it forwards both ways"""
    settings = exudyn.VisualizationSettings()
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', DeprecationWarning)
        settings.general.drawWorldBasis = True
        assert settings.view0.scene.drawWorldBasis
        settings.general.drawWorldBasis = False
        assert not settings.view0.scene.drawWorldBasis


def testTwoStandaloneStructuresAreIndependent():
    """they must not share anything through their backlinks

    The copy constructor and the copy assignment that the top class defines cannot be reached
    from Python - pybind exposes neither - so this is what is testable here: two structures, each
    linked to itself. The C++ copy is what the generated code guarantees, and it matters for the
    C++ side, where a settings structure IS assigned.
    """
    first = exudyn.VisualizationSettings()
    second = exudyn.VisualizationSettings()
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', DeprecationWarning)
        first.general.drawWorldBasis = True
        second.general.drawWorldBasis = False

        assert first.view0.scene.drawWorldBasis
        assert not second.view0.scene.drawWorldBasis


def testTheContainerSettingsAreUnaffected():
    """the path that always worked must keep working"""
    SC = exudyn.SystemContainer()
    (read, deprecated, problems) = ReadEveryMember(SC.visualizationSettings)
    assert problems == [], str(problems[:5])
    assert deprecated >= minimumDeprecatedMembers


def testASubStructureWithoutALinkRaisesInsteadOfCrashing():
    """a sub-structure constructed on its own has no link, and must say so - the guard is what
    stands between a future missing Init and another segfault"""
    if not hasattr(exudyn, 'VSettingsGeneral'):
        pytest.skip('VSettingsGeneral is not exposed')
    standalone = exudyn.VSettingsGeneral()
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', DeprecationWarning)
        with pytest.raises(Exception) as raised:
            standalone.drawWorldBasis
    assert 'not linked' in str(raised.value)


def testAStandaloneSimulationSettingsCanBeRead():
    """SimulationSettings link their sub-structures as VisualizationSettings do (#2588): every member of a
    standalone structure and of a copy can be read, the deprecated ones included"""
    import copy
    for settings in [exudyn.SimulationSettings(), copy.copy(exudyn.SimulationSettings())]:
        (read, deprecated, problems) = ReadEveryMember(settings)
        assert problems == [], 'a standalone SimulationSettings cannot be read: ' + str(problems[:5])
        assert read > 100 and deprecated >= 4
