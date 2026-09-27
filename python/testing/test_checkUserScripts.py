#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  tools/checkUserScripts.py - "exudev scripts" - reads an Exudyn script written for an
#           earlier version and says what it has to change (#2712). It parses and never runs, so a
#           test is a script as a string and the findings it must produce - and, as important, the
#           ones it must NOT produce: a name the script binds itself is not a missing import, and
#           view0.window is not the deprecated window of the visualization settings.
#
# Usage:    pytest python/testing/test_checkUserScripts.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-27
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import ast
import importlib.util
import os

import pytest

root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
spec = importlib.util.spec_from_file_location('checkUserScripts',
                                              os.path.join(root, 'tools', 'checkUserScripts.py'))
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


@pytest.fixture(scope='module')
def tables():
    return {'functions': checker.DeprecatedFunctions(), 'settings': checker.DeprecatedSettings(),
            'submodules': checker.SubmodulesNotLoaded()}


def Findings(source, tables):
    return [text for (line, text) in checker.CheckTree(ast.parse(source), tables)]


def testTheTablesAreReadFromTheDefinitions(tables):
    assert tables['functions'][('module', 'SolveDynamic')] == 'use mbs.SolveDynamic(...)'
    assert ('general', 'drawWorldBasis') in tables['settings']
    assert 'robotics' in tables['submodules']


def testANameTheStarImportNoLongerProvides(tables):
    found = Findings('from exudyn.utilities import *\nx = np.zeros(3)\n', tables)
    assert found == ["'np' no longer comes with 'from exudyn.utilities import *'; add: import numpy as np"]


def testANameTheScriptBindsIsNotMissing(tables):
    #imported, assigned, or a parameter of the function that uses it
    assert Findings('from exudyn.utilities import *\nimport numpy as np\nx = np.zeros(3)\n', tables) == []
    assert Findings('from exudyn.utilities import *\ndef F(sin): return sin\n', tables) == []


def testAParameterElsewhereDoesNotHideAMissingImport(tables):
    """a function whose argument is called np does not bind np for the module"""
    found = Findings('from exudyn.utilities import *\ndef F(np): return np\nx = np.zeros(3)\n', tables)
    assert len(found) == 1 and found[0].startswith("'np' no longer comes with")


def testRemovedNamesAndTheirReplacement(tables):
    found = Findings('from exudyn.utilities import *\nn = NormL2([1,2])\n'
                     'g = GraphicsDataOrthoCubePoint([0,0,0],[1,1,1])\n', tables)
    assert "'NormL2' is removed; use np.linalg.norm(v)" in found
    assert any(text.startswith("'GraphicsDataOrthoCubePoint' is removed; use graphics.Brick") for text in found)


def testDeprecatedFunctionsAndSettings(tables):
    found = Findings('import exudyn as exu\nSC = exu.SystemContainer()\nmbs = SC.AddSystem()\n'
                     'exu.SolveDynamic(mbs)\nSC.GetRenderState()\n'
                     'SC.visualizationSettings.general.drawWorldBasis = True\n', tables)
    assert "'exu.SolveDynamic' is deprecated: use mbs.SolveDynamic(...)" in found
    assert "'SC.GetRenderState' is deprecated: use SC.renderer.GetState()" in found
    assert "'general.drawWorldBasis' is deprecated; use view0.scene.drawWorldBasis" in found


def testTheWindowOfAViewIsNotTheDeprecatedWindow(tables):
    """window is a deprecated member at the top of the visualization settings AND the current one of
    every view: only the first is a finding"""
    assert Findings('import exudyn as exu\nSC = exu.SystemContainer()\n'
                    'SC.visualizationSettings.view0.window.renderWindowSize = [800, 600]\n', tables) == []
    assert len(Findings('import exudyn as exu\nSC = exu.SystemContainer()\n'
                        'SC.visualizationSettings.window.renderWindowSize = [800, 600]\n', tables)) == 1


def testRemovedSettingsArgumentsAndUnloadedSubmodules(tables):
    found = Findings('import exudyn as exu\nSC = exu.SystemContainer()\n'
                     'SC.visualizationSettings.exportImages.saveImageAsTextLines = True\n'
                     'o = ObjectContactConvexRoll(rBoundingSphere=1)\nr = exu.robotics.Robot()\n', tables)
    assert any(text.startswith("'exportImages.saveImageAsTextLines' is removed") for text in found)
    assert any(text.startswith("argument 'rBoundingSphere' is removed") for text in found)
    assert any(text.startswith("'exu.robotics' is used, but 'import exudyn' does not load it") for text in found)
    assert Findings('import exudyn as exu\nimport exudyn.robotics\nr = exu.robotics.Robot()\n', tables) == []


def testTheCommandLine(tmp_path):
    """a folder with an old script, a Python 2 script and a file without exudyn"""
    (tmp_path / 'old.py').write_text('from exudyn.utilities import *\nx = np.zeros(3)\n')
    (tmp_path / 'py2.py').write_text('import exudyn\nprint "hello"\n')
    (tmp_path / 'other.py').write_text('import numpy\n')
    import subprocess
    import sys
    result = subprocess.run([sys.executable, os.path.join(root, 'tools', 'checkUserScripts.py'),
                             str(tmp_path), '--check', '--base', str(tmp_path)],
                            capture_output=True, text=True, timeout=120)
    assert result.returncode == 1
    assert "old.py:2: 'np' no longer comes with" in result.stdout
    assert 'py2.py:2: does not parse as Python 3' in result.stdout
    assert '1 other .py files skipped' in result.stdout
