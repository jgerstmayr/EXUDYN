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
            'submodules': checker.SubmodulesNotLoaded(), 'itemParameters': checker.DeprecatedItemParameters(),
            'library': checker.DeprecatedLibrary()}


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
    assert "'general.drawWorldBasis' is deprecated since 1.10.80; use view0.scene.drawWorldBasis" in found


def testDeprecatedAndRemovedItemParameters(tables):
    """as keyword of the item class, its short name and as key of a dictionary (#2805); the settings renamed by #2800
    are found as all deprecated settings"""
    found = Findings('import exudyn as exu\nfrom exudyn.utilities import *\n'
                     'mbs.AddObject(ObjectJointGeneric(markerNumbers=[0,1], rotationMarker0=A))\n'
                     'mbs.AddObject(RigidBodySpringDamper(markerNumbers=[0,1], rotationMarker1=A))\n'
                     "mbs.AddObject({'objectType': 'JointRevoluteZ', 'markerNumbers': [0,1], 'rotationMarker0': A})\n"
                     'mbs.AddObject(ObjectContactCurveCircles(markerNumbers=[0,1], rotationMarker0=A))\n'
                     'mbs.AddObject(ObjectJointSpherical(markerNumbers=[0,1]))\n'
                     's = exu.SimulationSettings()\ns.parallel.multithreadedLLimitLoads = 10\n', tables)
    assert any(text.startswith('ObjectJointGeneric(rotationMarker0=...): the parameter is deprecated since') and
               'give the rotation to marker 0 as its localHT' in text for text in found)
    assert any(text.startswith('RigidBodySpringDamper(rotationMarker1=...): the parameter is deprecated') for text in found)
    assert any(text.startswith("'rotationMarker0' of JointRevoluteZ: the parameter is deprecated") for text in found)
    assert any(text.startswith('ObjectContactCurveCircles(rotationMarker0=...): the parameter is removed') for text in found)
    assert "'parallel.multithreadedLLimitLoads' is deprecated since 1.12.244; use multithreadedLowerLimitLoads" in found
    assert len(found) == 5


def testDeprecatedFunctionsAndArgumentsOfTheLibrary(tables):
    """the declarations of exudyn.misc.deprecation (#2807): a function by its name, an argument as keyword of its
    function, and bodyList, whose function is named at runtime, in any call"""
    found = Findings('import exudyn as exu\nfrom exudyn.utilities import *\n'
                     'import exudyn.graphics as graphics\n'
                     'b = AddRigidBody(mbs, inertia, nodeType=exu.NodeType.RotationEulerParameters)\n'
                     'g = graphics.BrickXYZ(0,0,0,1,1,1)\n'
                     'GeneticOptimization(F, p, numberOfChildren=8)\n'
                     'mbs.CreateSpringDamper(bodyList=[b0, b1])\n'
                     'mbs.CreateSpringDamper(bodyNumbers=[b0, b1])\n', tables)
    assert any(text.startswith("'AddRigidBody' is deprecated since 1.11.0 and removed in 2029; use mbs.CreateRigidBody")
               for text in found)
    assert any(text.startswith("'BrickXYZ' is deprecated") for text in found)
    assert any(text.startswith('GeneticOptimization(numberOfChildren=...): the argument is deprecated') for text in found)
    assert any(text.startswith('CreateSpringDamper(bodyList=...): the argument is deprecated') for text in found)
    assert len(found) == 4


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


def testDeprecationsTheCppWarnsAbout(tables):
    """WaitForRenderEngineStopFlag warns in C++ although its description does not say DEPRECATED"""
    found = Findings('import exudyn as exu\nSC = exu.SystemContainer()\nmbs = SC.AddSystem()\n'
                     'mbs.WaitForUserToContinue()\nSC.WaitForRenderEngineStopFlag()\n', tables)
    assert "'mbs.WaitForUserToContinue' is deprecated: use SC.renderer.DoIdleTasks()" in found
    assert "'SC.WaitForRenderEngineStopFlag' is deprecated: use SC.renderer.DoIdleTasks()" in found


def testAFileNamedWithoutADirectoryIsWrittenBesideTheScript(tables):
    """the solution file, a sensor, a computed name - each named without a directory (#2718)"""
    found = Findings('import exudyn as exu\nss = exu.SimulationSettings()\n'
                     "ss.solution.file.name = 'static.txt'\n"
                     "s = SensorNode(nodeNumber=0, fileName='node.txt')\n"
                     "ss.solution.solverInformationFileName = 'info' + str(3) + '.txt'\n", tables)
    assert len(found) == 3
    assert all("where the script runs; name a directory: 'solution/" in text for text in found)
    assert Findings('import exudyn as exu\nss = exu.SimulationSettings()\n'
                    "ss.solution.file.name = 'solution/static.txt'\n", tables) == []


def testTheOldDefaultSolutionFileIsReadFromItsNewPlace(tables):
    found = Findings("import numpy as np\nimport exudyn\ndata = np.loadtxt('coordinatesSolution.txt')\n", tables)
    assert found == ["'coordinatesSolution.txt' is not where Exudyn writes it any more; the default is "
                     "'solution/coordinatesSolution.txt'"]
    #unless the script writes that name itself - then it is a file beside the script, reported as such
    found = Findings('import numpy as np\nimport exudyn as exu\nss = exu.SimulationSettings()\n'
                     "ss.solution.file.name = 'coordinatesSolution.txt'\n"
                     "data = np.loadtxt('coordinatesSolution.txt')\n", tables)
    assert len(found) == 1 and 'where the script runs' in found[0]


def testASettingThatBecameADummyIsToBeRemoved(tables):
    found = Findings('import exudyn as exu\nSC = exu.SystemContainer()\n'
                     'SC.visualizationSettings.openGL.light0ambient = 0.5\n', tables)
    assert len(found) == 1 and found[0].endswith('it has no effect; remove it')



def testRemovedVisualizationParameters(tables):
    """show of an item that draws nothing, and drawSize of the gravity connector, are gone (#2843): found as the
    keyword of the visualization class and as the key of an item dictionary; show of an item that draws is not"""
    findings = Findings('import exudyn as exu\n'
                        'mbs.AddNode(Node1D(visualization=VNode1D(show=False)))\n'
                        'v = VCoordinateVectorConstraint(show=False, color=[1,0,0,1])\n'
                        "mbs.AddObject({'objectType': 'ConnectorGravity', 'markerNumbers': [0,1], 'VdrawSize': 0.1})\n"
                        'v2 = VMassPoint(show=False)\n', tables)
    assert len(findings) == 4
    assert all('removed' in text for text in findings)
    assert any('VNode1D(show' in text for text in findings)
    assert any("'VdrawSize' of ConnectorGravity" in text for text in findings)
