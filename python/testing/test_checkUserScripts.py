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
    """the declarations of exudyn.misc.deprecation (#2807): a function by its name, and an argument as keyword of its
    function - bodyNumbers of CreateSpringDamper, not of CreateRollingDisc"""
    found = Findings('import exudyn as exu\nfrom exudyn.utilities import *\n'
                     'import exudyn.graphics as graphics\n'
                     'b = AddRigidBody(mbs, inertia, nodeType=exu.NodeType.RotationEulerParameters)\n'
                     'g = graphics.BrickXYZ(0,0,0,1,1,1)\n'
                     'GeneticOptimization(F, p, numberOfChildren=8)\n'
                     'mbs.CreateSpringDamper(bodyNumbers=[b0, b1])\n'
                     'mbs.CreateRollingDisc(bodyNumbers=[b0, b1])\n'
                     'mbs.CreateSpringDamper(itemNumbers=[b0, b1])\n', tables)
    assert any(text.startswith("'AddRigidBody' is deprecated since 1.11.0 and removed in 2029; use mbs.CreateRigidBody")
               for text in found)
    assert any(text.startswith("'BrickXYZ' is deprecated") for text in found)
    assert any(text.startswith('GeneticOptimization(numberOfChildren=...): the argument is deprecated') for text in found)
    assert any(text.startswith('CreateSpringDamper(bodyNumbers=...): the argument is deprecated') for text in found)
    assert len(found) == 4


def testTheRenamedArgumentsOfTheCreateFunctionsAreFixed(tables):
    """--fix writes itemNumbers for bodyNumbers, bodyOrNodeList and bodyList of the Create functions, and itemNumber
    for bodyNumber of CreateForce (#2863); bodyNumbers of CreateRollingDisc and of an item stays"""
    source = ('import exudyn as exu\n'
              'mbs.CreateSpringDamper(name="s", bodyNumbers=[b0, b1], stiffness=1)\n'
              'mbs.CreateDistanceConstraint(bodyOrNodeList=[b0, n1])\n'
              'mbs.CreateCartesianSpringDamper(bodyList=[b0, b1])\n'
              'mbs.CreateForce(bodyNumber=b0, loadVector=[1,0,0])\n'
              'mbs.CreateRollingDisc(bodyNumbers=[b0, b1])\n'
              'm = MarkerBodiesRelativeRotationCoordinate(bodyNumbers=[b0, b1])\n')
    fixes = []
    checker.CheckTree(ast.parse(source), tables, fixes)
    fixed = checker.ApplyFixes(source, fixes)
    assert fixed == source.replace('bodyNumbers=[b0, b1], stiffness', 'itemNumbers=[b0, b1], stiffness').replace(
        'bodyOrNodeList=', 'itemNumbers=').replace('bodyList=', 'itemNumbers=').replace('bodyNumber=b0', 'itemNumber=b0')
    assert any(text.startswith("argument 'bodyOrNodeList' is removed") for text in Findings(source, tables))


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


def testRunningScriptsInACopy(tmp_path):
    """--run (#2713): a script runs in a copy of its folder - the files it writes stay out of the original -, a failing
    one reports its error, a missing package is named, and a path outside the folder keeps a script from running"""
    folder = tmp_path / 'scripts'
    folder.mkdir()
    (folder / 'ok.py').write_text("import exudyn as exu\nopen('written.txt', 'w').write('x')\n")
    (folder / 'broken.py').write_text("import exudyn as exu\nraise ValueError('broken script')\n")
    (folder / 'needs.py').write_text("import exudyn as exu\nimport aPackageThatIsNotInstalled\n")
    outside = ast.parse("import exudyn as exu\ndata = open('../secret.txt')\n")
    assert checker.PathsOutsideTheFolder(outside) == [(2, '../secret.txt')]
    assert checker.PathsOutsideTheFolder(ast.parse("x = 'C:/data/model.stl'\ny = 'solution/a.txt'\n")) == [(1, 'C:/data/model.stl')]

    results = checker.RunScripts([(str(folder / name), name) for name in ['ok.py', 'broken.py', 'needs.py']], timeout=60)
    results = dict((shown, (result, text)) for (shown, result, text) in results)
    assert results['ok.py'] == ('ran', '')
    assert results['broken.py'][0] == 'failed' and 'broken script' in results['broken.py'][1]
    assert results['needs.py'] == ('needs', 'aPackageThatIsNotInstalled')
    assert not (folder / 'written.txt').exists()


def testWhatFixRewrites(tables):
    """--fix (#2938): the missing imports of a star import, the removed vector helpers and GraphicsData aliases, the
    colors, the deprecated module functions and SystemContainer methods, the old default solution file, and a function
    that became one of the MainSystem; afterwards nothing is left to report"""
    source = ('import exudyn as exu\n'
              'from exudyn.utilities import *\n'
              '\n'
              'SC = exu.SystemContainer()\n'
              'mbs = SC.AddSystem()\n'
              'exu.SetOutputPrecision(8)\n'
              'v = VAdd([1,2,3], [4,5,6])\n'
              'w = ScalarMult(a+b, v)\n'
              'n = NormL2(VSub(v, w))\n'
              'A = DiagonalMatrix(3, value=2.)\n'
              'B = eye3D\n'
              's = np.zeros(3) + sin(0.1)\n'
              'g = GraphicsDataOrthoCubePoint(size=[1,1,1], color=color4red)\n'
              'exu.StartRenderer()\n'
              'state = SC.GetRenderState()\n'
              'exu.SolveDynamic(mbs, sims)\n'
              'mbs.WaitForUserToContinue()\n'
              "data = np.loadtxt('coordinatesSolution.txt')\n"
              'CreateDistanceSensor(mbs, generalContact, positionOrMarker=[0,0,0])\n')
    (fixed, tree, nFixed) = checker.FixSource(source, ast.parse(source), tables)
    assert fixed == ('import exudyn as exu\n'
                     'from exudyn.utilities import *\n'
                     'import numpy as np  #added by exudev scripts --fix\n'
                     'import exudyn.graphics as graphics  #added by exudev scripts --fix\n'
                     'from math import sin  #added by exudev scripts --fix\n'
                     '\n'
                     'SC = exu.SystemContainer()\n'
                     'mbs = SC.AddSystem()\n'
                     'exu.config.outputPrecision = 8\n'
                     'v = (np.array([1,2,3]) + np.array([4,5,6]))\n'
                     'w = (a+b)*np.array(v)\n'
                     'n = np.linalg.norm((np.array(v) - np.array(w)))\n'
                     'A = 2.*np.eye(3)\n'
                     'B = np.eye(3)\n'
                     's = np.zeros(3) + sin(0.1)\n'
                     'g = graphics.Brick(size=[1,1,1], color=graphics.color.red)\n'
                     'SC.renderer.Start()\n'
                     'state = SC.renderer.GetState()\n'
                     'mbs.SolveDynamic(sims)\n'
                     'SC.renderer.DoIdleTasks()\n'
                     "data = np.loadtxt('solution/coordinatesSolution.txt')\n"
                     'mbs.CreateDistanceSensor(generalContact, positionOrMarker=[0,0,0])\n')
    assert nFixed > 0 and Findings(fixed, tables) == []


def testFixLeavesWhatItCannotDecide(tables):
    """without one SystemContainer the renderer functions stay, a keyword call of VAdd stays, and an import the script
    has already is not added twice"""
    source = ('import exudyn as exu\n'
              'import numpy as np\n'
              'from exudyn.utilities import *\n'
              'exu.StartRenderer()\n'
              'v = VAdd(v0=[1], v1=[2])\n'
              'x = np.zeros(3)\n')
    (fixed, tree, nFixed) = checker.FixSource(source, ast.parse(source), tables)
    assert fixed == source
    assert len(Findings(fixed, tables)) == 2


def testADeprecatedArgumentWithAnotherValueIsConverted(tables):
    """eulerParametersRef of AddObjectFFRFreducedOrderWithUserFunctions becomes rotationMatrixRef with the matrix of
    the Euler parameters, not the Euler parameters themselves; eulerParametersRef of AddObjectFFRF is not deprecated"""
    source = ('import exudyn as exu\n'
              'o1 = cms.AddObjectFFRFreducedOrderWithUserFunctions(exu, mbs, eulerParametersRef=ep0)\n'
              'o2 = ffrf.AddObjectFFRF(exu, mbs, eulerParametersRef=ep0)\n')
    (fixed, tree, nFixed) = checker.FixSource(source, ast.parse(source), tables)
    assert fixed == ('import exudyn as exu\n'
                     'from exudyn.rigidBodyUtilities import EulerParameters2RotationMatrix  #added by exudev scripts --fix\n'
                     'o1 = cms.AddObjectFFRFreducedOrderWithUserFunctions(exu, mbs, '
                     'rotationMatrixRef=EulerParameters2RotationMatrix(ep0))\n'
                     'o2 = ffrf.AddObjectFFRF(exu, mbs, eulerParametersRef=ep0)\n')


def testFixDeletesWhatHasNoEffect(tables):
    """--fix (#2938, the maintainer's decision): a dummy or removed setting, a removed argument and a visualization
    parameter of an item that draws nothing are deleted - the statement with its line, the argument with its comma -,
    'pass' where the statement was the only one of its block; AttachToRenderEngine stays; BrickXYZ becomes Brick"""
    source = ('import exudyn as exu\n'
              'import exudyn.graphics as graphics\n'
              'SC = exu.SystemContainer()\n'
              'SC.visualizationSettings.openGL.light0ambient = 0.5  #old\n'
              'if True:\n'
              '    SC.visualizationSettings.interactive.openVR = True\n'
              'n = mbs.AddNode(Node1D(visualization=VNode1D(show=False)))\n'
              'v = VCoordinateVectorConstraint(show=False, color=[1,0,0,1])\n'
              "mbs.AddObject({'objectType': 'ConnectorGravity', 'markerNumbers': [0,1], 'VdrawSize': 0.1})\n"
              'c = ObjectContactConvexRoll(markerNumbers=[0,1], rBoundingSphere=0.1, contactStiffness=1e4)\n'
              'SC.AttachToRenderEngine()\n'
              'b1 = graphics.BrickXYZ(0, 0, 0, 1, 2, 0.5, color=graphics.color.red)\n'
              'b2 = graphics.BrickXYZ(x0, y0, 0, x1, y1, 1)\n')
    (fixed, tree, nFixed) = checker.FixSource(source, ast.parse(source), tables)
    assert fixed == ('import exudyn as exu\n'
                     'import exudyn.graphics as graphics\n'
                     'SC = exu.SystemContainer()\n'
                     'if True:\n'
                     '    pass\n'
                     'n = mbs.AddNode(Node1D(visualization=VNode1D()))\n'
                     'v = VCoordinateVectorConstraint()\n'
                     "mbs.AddObject({'objectType': 'ConnectorGravity', 'markerNumbers': [0,1]})\n"
                     'c = ObjectContactConvexRoll(markerNumbers=[0,1], contactStiffness=1e4)\n'
                     'SC.AttachToRenderEngine()\n'
                     'b1 = graphics.Brick(centerPoint=[0.5, 1.0, 0.25], size=[1, 2, 0.5], color=graphics.color.red)\n'
                     'b2 = graphics.Brick(centerPoint=[0.5*(x0+x1), 0.5*(y0+y1), 0.5], size=[x1-x0, y1-y0, 1])\n')
    assert [text for text in Findings(fixed, tables)] == ["'SC.AttachToRenderEngine' is deprecated: "
                                                         + tables['functions'][('SystemContainer', 'AttachToRenderEngine')]]


def testFixKeepsTheOriginalAsBackup(tmp_path, tables):
    """the command line writes the fixed script in place and keeps the original as .bak, once"""
    import subprocess
    import sys
    script = tmp_path / 'old.py'
    script.write_text('import exudyn as exu\nfrom exudyn.utilities import *\nx = np.zeros(3)\n')
    subprocess.run([sys.executable, os.path.join(root, 'tools', 'checkUserScripts.py'), str(tmp_path), '--fix'],
                   capture_output=True, text=True, check=True)
    assert 'import numpy as np' in script.read_text()
    assert (tmp_path / 'old.py.bak').read_text() == 'import exudyn as exu\nfrom exudyn.utilities import *\nx = np.zeros(3)\n'


def testFixDrawsRectanglesAndCubeLinesAsBricks(tables):
    """--fix (#2941): GraphicsDataRectangle and GraphicsDataOrthoCubeLines become graphics.Brick with edges only,
    the rectangle with size 0 in z, its color as the color of the edges"""
    source = ('import exudyn as exu\n'
              'import exudyn.graphicsDataUtilities as gdu\n'
              'g = gdu.GraphicsDataRectangle(-1, -2, 1, 2, color=[1,0,0,1])\n'
              'h = gdu.GraphicsDataOrthoCubeLines(0, 0, 0, a, 1, 1)\n')
    (fixed, tree, nFixed) = checker.FixSource(source, ast.parse(source), tables)
    assert fixed == ('import exudyn as exu\n'
                     'import exudyn.graphicsDataUtilities as gdu\n'
                     'import exudyn.graphics as graphics  #added by exudev scripts --fix\n'
                     'g = graphics.Brick(centerPoint=[0.0, 0.0, 0], size=[2, 4, 0], addFaces=False, addEdges=True, '
                     'edgeColor=[1,0,0,1])\n'
                     'h = graphics.Brick(centerPoint=[0.5*(0+a), 0.5, 0.5], size=[a-0, 1, 1], addFaces=False, addEdges=True)\n')


def testFixReplacesAddRigidBodyByCreateRigidBody(tables):
    """--fix (#2944): AddRigidBody becomes mbs.CreateRigidBody with its arguments renamed; the node of the returned
    [node, body] is read with mbs.GetObject(body)['nodeNumber'], also where the list is used as rb[0] and rb[1];
    rotationParameters become a rotation matrix of the node type; a result used otherwise stays reported"""
    source = ('import exudyn as exu\n'
              'from exudyn.utilities import *\n'
              '[n0, b0] = AddRigidBody(mainSys=mbs, inertia=i0, position=[1,0,0], gravity=g)\n'
              'if True:\n'
              '    n1, b1 = AddRigidBody(mbs, i0, exu.NodeType.RotationRxyz, [2,0,0], [0,0,0], [], [0,0,0.3])  #c\n'
              'rb = AddRigidBody(mbs, i0)\n'
              'm = MarkerNodeRigid(nodeNumber=rb[0])\n'
              'b = MarkerBodyRigid(bodyNumber=rb[1])\n'
              'AddRigidBody(mbs, i0, position=[4,0,0])\n'
              'items = AddRigidBody(mbs, i0)\n'
              'print(items)\n')
    (fixed, tree, nFixed) = checker.FixSource(source, ast.parse(source), tables)
    assert fixed == ('import exudyn as exu\n'
                     'from exudyn.utilities import *\n'
                     'b0 = mbs.CreateRigidBody(inertia=i0, referencePosition=[1,0,0], gravity=g)\n'
                     "n0 = mbs.GetObject(b0)['nodeNumber']\n"
                     'if True:\n'
                     '    b1 = mbs.CreateRigidBody(inertia=i0, nodeType=exu.NodeType.RotationRxyz, referencePosition=[2,0,0], '
                     'initialVelocity=[0,0,0], referenceRotationMatrix=RotXYZ2RotationMatrix([0,0,0.3]))  #c\n'
                     "    n1 = mbs.GetObject(b1)['nodeNumber']\n"
                     'rb = mbs.CreateRigidBody(inertia=i0)\n'
                     "m = MarkerNodeRigid(nodeNumber=mbs.GetObject(rb)['nodeNumber'])\n"
                     'b = MarkerBodyRigid(bodyNumber=rb)\n'
                     'mbs.CreateRigidBody(inertia=i0, referencePosition=[4,0,0])\n'
                     'items = AddRigidBody(mbs, i0)\n'
                     'print(items)\n')
    assert len(Findings(fixed, tables)) == 1


def testFixReplacesTheAddJointsAndTheOldSparseConversion(tables):
    """--fix (#2945): AddRevoluteJoint/AddPrismaticJoint become the Create functions, the markers of the returned
    [joint, marker0, marker1] are read from the joint; CSRtoScipySparseCSR of a matrix of the FEM interface is the
    matrix; a CSRtoScipySparseCSR the script defines itself is neither reported nor changed"""
    source = ('import exudyn as exu\n'
              'from exudyn.utilities import *\n'
              'j, m0, m1 = AddRevoluteJoint(mbs=mbs, body0=g, body1=b, point=[0,0,0], axis=[0,0,1], showJoint=False)\n'
              'p = AddPrismaticJoint(mbs, b, g, [1,0,0], [1,0,0])\n'
              'q = mbs.GetObject(p[0])\n'
              'r = p[2]\n'
              'm = CSRtoScipySparseCSR(fem.GetMassMatrix(sparse=True))\n')
    (fixed, tree, nFixed) = checker.FixSource(source, ast.parse(source), tables)
    assert fixed == ('import exudyn as exu\n'
                     'from exudyn.utilities import *\n'
                     'j = mbs.CreateRevoluteJoint(itemNumbers=[g, b], position=[0,0,0], axis=[0,0,1], show=False)\n'
                     "m0 = mbs.GetObject(j)['markerNumbers'][0]\n"
                     "m1 = mbs.GetObject(j)['markerNumbers'][1]\n"
                     'p = mbs.CreatePrismaticJoint(itemNumbers=[b, g], position=[1,0,0], axis=[1,0,0])\n'
                     'q = mbs.GetObject(p)\n'
                     "r = mbs.GetObject(p)['markerNumbers'][1]\n"
                     'm = fem.GetMassMatrix(sparse=True)\n')
    own = 'import exudyn as exu\ndef CSRtoScipySparseCSR(x): return x\nm = CSRtoScipySparseCSR(a)\n'
    assert Findings(own, tables) == [] and checker.FixSource(own, ast.parse(own), tables)[0] == own


def testFixMovesRotationMarkerIntoTheLocalHTOfItsMarker(tables):
    """--fix (#2946): rotationMarker<k> goes into the localHT of marker k where the script creates that marker -
    inline, or once and used by this item only, with the rotation known before it -; a unit matrix just goes; a marker
    used twice stays reported"""
    source = ('import exudyn as exu\n'
              'A = RotationMatrixZ(0.3)\n'
              'mG = mbs.AddMarker(MarkerBodyRigid(bodyNumber=g, localPosition=[0,0,0]))\n'
              'mB = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b, localPosition=[-0.5,0,0]))\n'
              'mbs.AddObject(GenericJoint(markerNumbers=[mG, mB], rotationMarker0=np.eye(3), rotationMarker1=A))\n'
              'mbs.AddObject(TorsionalSpringDamper(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=g)), mC], '
              'rotationMarker0=A))\n'
              'mD = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b))\n'
              'mbs.AddObject(GenericJoint(markerNumbers=[mD, mC], rotationMarker0=A))\n'
              'mbs.AddObject(GenericJoint(markerNumbers=[mD, mC]))\n')
    (fixed, tree, nFixed) = checker.FixSource(source, ast.parse(source), tables)
    assert fixed == ('import exudyn as exu\n'
                     'A = RotationMatrixZ(0.3)\n'
                     'mG = mbs.AddMarker(MarkerBodyRigid(bodyNumber=g, localPosition=[0,0,0]))\n'
                     'mB = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b, localHT=exu.HT(rotation=A, translation=[-0.5,0,0])))\n'
                     'mbs.AddObject(GenericJoint(markerNumbers=[mG, mB]))\n'
                     'mbs.AddObject(TorsionalSpringDamper(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=g, '
                     'localHT=exu.HT(rotation=A))), mC]))\n'
                     'mD = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b))\n'
                     'mbs.AddObject(GenericJoint(markerNumbers=[mD, mC], rotationMarker0=A))\n'
                     'mbs.AddObject(GenericJoint(markerNumbers=[mD, mC]))\n')


def testFixGivesCreateRigidBodyTheNodeTypeItself(tables):
    """AddRigidBody also took str(exu.NodeType.X) and 'NodeType.X', which CreateRigidBody refuses: --fix writes the
    NodeType itself, also in a CreateRigidBody an earlier --fix wrote (#2946); a marker variable used again for the next
    marker is fixed where the item is its only use"""
    source = ('import exudyn as exu\n'
              'b = mbs.CreateRigidBody(inertia=i, nodeType=str(exu.NodeType.RotationEulerParameters))\n'
              "c = mbs.CreateRigidBody(inertia=i, nodeType='NodeType.RotationRxyz')\n"
              'm = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b))\n'
              'mbs.AddObject(GenericJoint(markerNumbers=[m, g]))\n'
              'm = mbs.AddMarker(MarkerBodyRigid(bodyNumber=c, localPosition=[1,0,0]))\n'
              'mbs.AddObject(GenericJoint(markerNumbers=[g, m], rotationMarker1=A))\n')
    (fixed, tree, nFixed) = checker.FixSource(source, ast.parse(source), tables)
    assert fixed == ('import exudyn as exu\n'
                     'b = mbs.CreateRigidBody(inertia=i, nodeType=exu.NodeType.RotationEulerParameters)\n'
                     'c = mbs.CreateRigidBody(inertia=i, nodeType=exu.NodeType.RotationRxyz)\n'
                     'm = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b))\n'
                     'mbs.AddObject(GenericJoint(markerNumbers=[m, g]))\n'
                     'm = mbs.AddMarker(MarkerBodyRigid(bodyNumber=c, localHT=exu.HT(rotation=A, translation=[1,0,0])))\n'
                     'mbs.AddObject(GenericJoint(markerNumbers=[g, m]))\n')
