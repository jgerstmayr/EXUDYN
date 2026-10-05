#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  tools/checkDescriptions.py - the record that an item description was checked against its
#           implementation, with a fingerprint of the implementation (#2717). The fingerprint ignores
#           comments and white space - the generated headers carry the descriptions as comments - and
#           changes with the code; a recorded item whose implementation changed is reported by --check
#           until --mark records it again.
#
# Usage:    pytest python/testing/test_checkDescriptions.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-05
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import importlib.util
import os

root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
spec = importlib.util.spec_from_file_location('checkDescriptions', os.path.join(root, 'tools', 'checkDescriptions.py'))
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


def testCommentsAndWhiteSpaceDoNotCount():
    code = 'int f(int a) { return a+1; } //one\n/* two */ const char* s = "a // not a comment";'
    assert checker.WithoutComments(code) == checker.WithoutComments(code.replace('//one', '//changed').replace('  ', ' '))
    assert '"a // not a comment"' in checker.WithoutComments(code)
    assert checker.WithoutComments(code) != checker.WithoutComments(code.replace('a+1', 'a+2'))


def testTheMembersOfAClassInASharedSource():
    text = checker.WithoutComments('#include "x.h"\nvoid CLoadA::F(int a) const { if (a) { g(); } }\n'
                                   'Real CLoadAB::G() { return 1; }\nvoid Other() { CLoadA::F(1); }\n'
                                   'void VisualizationCLoadA::UpdateGraphics(int v) { h(); }')
    found = checker.MemberDefinitions(text, 'CLoadA')
    assert found == ['void CLoadA::F(int a) const { if (a) { g(); } }', 'void VisualizationCLoadA::UpdateGraphics(int v) { h(); }']


def testEveryItemHasAFingerprint():
    items = checker.Items()
    assert len(items) > 90 and ('NodePoint', 'nodes') in items and ('SensorBody', 'sensors') in items
    shared = checker.SharedSources()
    assert all(len(checker.Fingerprint(name, kind, shared)) == 12 for (name, kind) in items)


def testAChangedItemIsReportedUntilItIsMarkedAgain(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(checker, 'recordFile', str(tmp_path / 'checks.json'))
    assert checker.Main(['--mark', 'NodePoint', '--by', 'test']) == 0
    assert checker.Main(['--check']) == 0
    records = checker.ReadRecords()
    records['NodePoint']['fingerprint'] = '000000000000' #as if the implementation had changed since
    checker.WriteRecords(records)
    capsys.readouterr()
    assert checker.Main(['--check']) == 1
    assert 'the implementation of NodePoint changed since its description was checked' in capsys.readouterr().out
    assert checker.Main(['--mark', 'NodePoint']) == 0
    assert checker.Main(['--check']) == 0
