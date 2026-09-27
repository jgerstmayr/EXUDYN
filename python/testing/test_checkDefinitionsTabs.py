#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  tools/checkDefinitions.py finds a TAB in a description (#2683). A backslash-t in a
#           literal that was not raw became a TAB, took the backslash with it, and left
#           'exttt{...}' on three pages of the Symbolic manual, where the LaTeX rule could not see it.
#           C++ and Python may indent with a TAB, so those are not findings.
#
# Usage:    pytest python/testing/test_checkDefinitionsTabs.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-27
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import importlib.util
import os

root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
spec = importlib.util.spec_from_file_location('checkDefinitions',
                                              os.path.join(root, 'tools', 'checkDefinitions.py'))
checkDefinitions = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checkDefinitions)

tab = chr(9)


def Findings(tmp_path, source):
    path = tmp_path / 'definitionsExample.py'
    path.write_text(source, encoding='utf-8')
    return checkDefinitions.CheckTabs([str(path)])


def testATabInADescriptionIsFoundOnItsLine(tmp_path):
    source = ('pb.AddDocu(r"""first line\n'
              'turning on recording by using ' + tab + 'exttt{exudyn.symbolic.SetRecording(True)}""")\n')
    findings = Findings(tmp_path, source)
    assert len(findings) == 1
    assert findings[0][1] == 2                  #the line of the TAB, not of the literal


def testATabInCppCodeIsNotAFinding(tmp_path):
    assert Findings(tmp_path, "pb.CppCode('" + tab + tab + "'+values+';')\n") == []


def testATabInCodeIsNotAFinding(tmp_path):
    assert Findings(tmp_path, "pb.DefPyFunctionAccess(cName='" + tab + "return 0;')\n") == []


def testTheDefinitionsHaveNone():
    paths = [os.path.join(root, 'definitions', name)
             for name in sorted(os.listdir(os.path.join(root, 'definitions'))) if name.endswith('.py')]
    assert checkDefinitions.CheckTabs(paths) == []
