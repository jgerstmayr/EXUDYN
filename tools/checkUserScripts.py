#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# checkUserScripts - what an Exudyn script written for an earlier version has to change (#2712)
#
# Teaching folders and user projects hold scripts written against Exudyn 1.x. This checker PARSES
# them and never runs them, and reports per file and line:
#   - a name the script uses that 'from exudyn.utilities import *' no longer provides - np, sin,
#     graphics, ... - with the import line that provides it;
#   - a name that is gone, with its replacement: the eleven vector helpers of basicUtilities and the
#     GraphicsData... aliases of exudyn.utilities;
#   - a function, method, setting or item parameter that is DEPRECATED, with what to use instead -
#     also a function or an argument of the Python library (exudyn.misc.deprecation, #2807) -
#     read from definitions/, so that the list is the one the documentation is generated from; an item
#     parameter is found as the keyword of its item class and as the key of an item dictionary;
#   - a setting or a function that is REMOVED;
#   - a submodule used through 'exu.<submodule>' that 'import exudyn' does not load;
#   - a file the script writes without naming a directory - a solution file, a sensor file, the
#     results of a parameter variation - which lands beside the script, and the default solution
#     file read back by its old name: the files a run writes by default are in solution/ (#2718).
# With --run it also RUNS them (#2713): each folder of scripts is copied into a temporary folder, and each script runs
# there in a process of its own, without windows, with a timeout and a solver timeout, as the examples are run; a
# script that names a path outside its folder - 'C:/...', '/home/...', '../data' - is not run, because its copy would
# read or write somewhere else. It reports per script: ran, failed (with the last line of the error), timed out, or
# needs a package that is not installed.
# It checks every .py file under the folders it is given that imports exudyn; the others are counted
# and skipped. A file that does not parse - Python 2, say - is reported as such.
#
# The tables of what was removed are history and do not change; what is deprecated is read from
# definitions/ each time. Where they come from is said at each table.
#
# Usage:
#   python tools/checkUserScripts.py <folder or file> [...]          #report
#   python tools/checkUserScripts.py <folder> --check                 #exit 1 if anything was found
#   python tools/checkUserScripts.py <folder> --fix                   #rewrite the renamed settings in place (#2813)
#   python tools/checkUserScripts.py <folder> --run [--timeout 120]   #check, then run each script in a copy (#2713)
#   exudev scripts <folder> [...]                                     #the same, through the driver
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import argparse
import ast
import builtins
import glob
import io
import os
import re
import shutil
import subprocess
import sys
import tempfile
import warnings

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#WHAT A STAR IMPORT FROM EXUDYN PROVIDED AND NO LONGER DOES, with the line that provides it now.
#Measured on 'from exudyn.utilities import *' in the sources before __all__ was introduced (#2444):
#the names every module of its star-import chain had imported for itself. A star import of one of
#those modules directly - exudyn.graphicsDataUtilities, say - provided the same kind of names.
formerStarNames = {
    'np':       'import numpy as np',
    'sin':      'from math import sin',
    'cos':      'from math import cos',
    'sqrt':     'from math import sqrt',
    'math':     'import math',
    'copy':     'import copy',
    'Enum':     'from enum import Enum',
    'exudyn':   'import exudyn',
    'exu':      'import exudyn as exu',
    'eii':      'import exudyn.itemInterface as eii',
    'graphics': 'import exudyn.graphics as graphics',
    'docmeta':  'from exudyn.misc.docmeta import docmeta',
    }

#WHAT 'from exudyn.utilities import *' NO LONGER PROVIDES (#2756): name -> (the module that provides it,
#what to write). Reported only when the script does not star-import that module itself
utilitiesNames = {}
for name in ['GenerateStraightLineANCFCable2D', 'GenerateSlidingJoint', 'GenerateAleSlidingJoint', 'GenerateStraightBeam']:
    utilitiesNames[name] = ('exudyn.beams', 'from exudyn.beams import ' + name)
for name in ['CreateDistanceSensorGeometry', 'CreateDistanceSensor', 'DrawSystemGraph']:
    utilitiesNames[name] = ('exudyn.misc.mainSystemExtensions', 'mbs.' + name + '(...), a function of the MainSystem')
for name in ['red', 'green', 'blue', 'cyan', 'magenta', 'yellow', 'orange', 'pink', 'lawngreen', 'springgreen',
             'violet', 'dodgerblue', 'lightred', 'lightgreen', 'steelblue', 'brown', 'black', 'darkgrey',
             'darkgrey2', 'grey', 'lightgrey', 'lightgrey2', 'white', 'default']:
    utilitiesNames['color4' + name] = ('exudyn.graphicsDataUtilities', 'graphics.color.' + name
                                       + ' with import exudyn.graphics as graphics')
for name in ['color4list', 'color4listSize', 'SwitchTripletOrder', 'ComputeTriangleNormal', 'ComputeTriangleArea',
             'Compute6NodeTrigsNormals', 'RefineMesh', 'ShrinkMeshNormalToSurface', 'ComputeTriangularMesh',
             'SegmentsFromPoints', 'CirclePointsAndSegments']:
    utilitiesNames[name] = ('exudyn.graphicsDataUtilities', 'from exudyn.graphicsDataUtilities import ' + name)
utilitiesNames['GraphicsDataRectangle'] = ('exudyn.graphicsDataUtilities', 'graphics.Lines([[x0,y0,0], [x1,y0,0], '
                                           '[x1,y1,0], [x0,y1,0], [x0,y0,0]], color=color)')
utilitiesNames['GraphicsDataOrthoCubeLines'] = ('exudyn.graphicsDataUtilities', 'graphics.BrickXYZ(x0, y0, z0, x1, y1, '
                                                'z1, addFaces=False, addEdges=True, edgeColor=color)')

#NAMES THAT ARE GONE, with what to write instead: the vector helpers removed from basicUtilities
#(#2442) and the GraphicsData... aliases removed from exudyn.utilities (#2443)
removedNames = {
    'NormL2':         'np.linalg.norm(v)',
    'VSum':           'np.sum(v)',
    'VAdd':           'np.array(v0) + np.array(v1)',
    'VSub':           'np.array(v0) - np.array(v1)',
    'VMult':          'np.dot(v0, v1)',
    'ScalarMult':     'scalar * np.array(v)',
    'Vec2Tilde':      'Skew(v)',
    'Tilde2Vec':      'Skew2Vec(m)',
    'DiagonalMatrix': 'value * np.eye(n)',
    'eye2D':          'np.eye(2)',
    'eye3D':          'np.eye(3)',
    'LoadImage':      'SC.renderer.GetGraphicsData(), which exudyn.plot.PlotImage draws directly (#2700)',
    'InitializeFromRestartFile': 'nothing yet: it never worked (it raised "not fully implemented"); a restart is #2850',
    }
for (alias, name) in [('GraphicsDataOrthoCubePoint', 'Brick'), ('GraphicsDataCube', 'Cuboid'),
                      ('GraphicsDataOrthoCube', 'BrickXYZ'), ('GraphicsDataSphere', 'Sphere'),
                      ('GraphicsDataCylinder', 'Cylinder'), ('GraphicsDataLine', 'Lines'),
                      ('GraphicsDataQuad', 'Quad'), ('GraphicsDataCircle', 'Circle'),
                      ('GraphicsDataText', 'Text'), ('GraphicsDataRigidLink', 'RigidLink'),
                      ('GraphicsDataSolidOfRevolution', 'SolidOfRevolution'),
                      ('GraphicsDataSolidExtrusion', 'SolidExtrusion'), ('GraphicsDataArrow', 'Arrow'),
                      ('GraphicsDataBasis', 'Basis'), ('GraphicsDataFrame', 'Frame'),
                      ('GraphicsDataCheckerBoard', 'CheckerBoard'),
                      ('GraphicsDataFromSTLfile', 'FromSTLfile'),
                      ('GraphicsDataFromSTLfileTxt', 'FromSTLfileASCII'),
                      ('GraphicsDataFromPointsAndTrigs', 'FromPointsAndTrigs'),
                      ('GraphicsData2PointsAndTrigs', 'ToPointsAndTrigs'),
                      ('ExportGraphicsData2STL', 'ExportSTL'), ('MoveGraphicsData', 'Move'),
                      ('MergeGraphicsDataTriangleList', 'MergeTriangleLists'),
                      ('AddEdgesAndSmoothenNormals', 'AddEdgesAndSmoothenNormals')]:
    removedNames[alias] = 'graphics.' + name + '(...) with import exudyn.graphics as graphics'

#SETTINGS THAT ARE GONE: (the structure member, the setting) -> what to do instead
removedSettings = {
    ('exportImages', 'saveImageAsTextCircles'):   'SC.renderer.GetGraphicsData() (#2700)',
    ('exportImages', 'saveImageAsTextLines'):     'SC.renderer.GetGraphicsData() (#2700)',
    ('exportImages', 'saveImageAsTextTriangles'): 'SC.renderer.GetGraphicsData() (#2700)',
    ('exportImages', 'saveImageAsTextTexts'):     'SC.renderer.GetGraphicsData() (#2700)',
    ('interactive', 'openVR'):                    'nothing: OpenVR is removed',
    ('dialogs', 'fontScalingMacOS'):              'dialogs.fontScaling, which works on every platform',
    }

#WHAT TO WRITE INSTEAD of a deprecated function; the description in definitions/ says what the
#function does, and this says what replaces it. A deprecated function missing here is reported with
#its description
replacements = {
    ('module', 'SolveStatic'):               'mbs.SolveStatic(...)',
    ('module', 'SolveDynamic'):              'mbs.SolveDynamic(...)',
    ('module', 'ComputeODE2Eigenvalues'):    'mbs.ComputeODE2Eigenvalues(...)',
    ('module', 'StartRenderer'):             'SC.renderer.Start()',
    ('module', 'StopRenderer'):              'SC.renderer.Stop()',
    ('module', 'DoRendererIdleTasks'):       'SC.renderer.DoIdleTasks()',
    ('module', 'IsRendererActive'):          'SC.renderer.IsActive()',
    ('module', 'SetOutputPrecision'):        'exudyn.config.outputPrecision = ...',
    ('module', 'SetLinalgOutputFormatPython'): 'exudyn.config.linalgOutputFormatPython = ...',
    ('module', 'SetPrintDelayMilliSeconds'): 'exudyn.config.printDelayMilliSeconds = ...',
    ('module', 'SuppressWarnings'):          'exudyn.config.suppressWarnings = ...',
    ('module', 'SetWriteToConsole'):         'exudyn.config.printToConsole = ...',
    ('module', 'InfoStat'):                  'exudyn.special.InfoStat()',
    ('SystemContainer', 'GetRenderState'):   'SC.renderer.GetState()',
    ('SystemContainer', 'SetRenderState'):   'SC.renderer.SetState(...)',
    ('SystemContainer', 'RedrawAndSaveImage'): 'SC.renderer.RedrawAndSaveImage()',
    ('SystemContainer', 'RenderEngineZoomAll'): 'SC.renderer.ZoomAll()',
    ('SystemContainer', 'AttachToRenderEngine'): 'nothing: SC.renderer.Start() attaches the container',
    ('SystemContainer', 'DetachFromRenderEngine'): 'nothing: SC.renderer.Stop() detaches it',
    ('SystemContainer', 'SendRedrawSignal'): 'SC.renderer.SendRedrawSignal()',
    ('SystemContainer', 'GetCurrentMouseCoordinates'): 'SC.renderer.GetMouseCoordinates()',
    ('renderer', 'Detach'):                  'nothing: SC.renderer.Stop() detaches the container, and '
                                             'a new SystemContainer attaches itself',
    ('MainSystem', 'WaitForUserToContinue'): 'SC.renderer.DoIdleTasks()',
    ('SystemContainer', 'WaitForRenderEngineStopFlag'): 'SC.renderer.DoIdleTasks()',
    }

#THE FILES A RUN WRITES BY DEFAULT are in solution/ (#2718): these were their names before
formerDefaultFiles = {
    'coordinatesSolution.txt': 'solution/coordinatesSolution.txt',
    'coordinatesSolution.sol': 'solution/coordinatesSolution.sol',
    'coordinatesSolution':     'solution/coordinatesSolution',
    'solverInformation.txt':   'solution/solverInformation.txt',
    'restartFile.txt':         'solution/restartFile.txt',
    }

#WHERE A SCRIPT NAMES A FILE IT WRITES: a setting, or a keyword argument of a function
outputSettings = ['coordinatesSolutionFileName', 'solverInformationFileName', 'restartFileName',
                  'saveImageFileName']
outputKeywords = {'fileName': 'Sensor', 'resultsFile': ''}   #keyword -> the call must contain this

#ARGUMENTS THAT ARE GONE
removedKeywords = {
    'rBoundingSphere': 'ObjectContactConvexRoll computes it from coefficientsHull; leave it out',
    'bodyOrNodeList': 'the Create functions take the items in itemNumbers (#2863)',
    'bodyList': 'the Create functions take the items in itemNumbers (#2863)',
    }
#the removed keywords that --fix renames: keyword -> its name now
removedKeywordRenames = {'bodyOrNodeList': 'itemNumbers', 'bodyList': 'itemNumbers'}

#ITEM PARAMETERS THAT ARE GONE, by item: (item class, parameter) -> what to do instead
removedItemParameters = {
    ('ObjectContactCurveCircles', 'rotationMarker0'): 'the curve lies in the x-y plane of marker 0; give a rotation '
                                                      'to marker 0 as its localHT (#2803)',
    }

#VISUALIZATION PARAMETERS THAT ARE GONE, by item: (item class, parameter) -> what to do instead; found as the keyword of
#the visualization class V<item> and as the key 'V<parameter>' of an item dictionary (#2843)
_drawsNothing = 'the item draws nothing; leave the visualization out (#2843)'
removedVisualizationParameters = {(className, 'show'): _drawsNothing for className in [
    'Node1D', 'NodeGenericODE2', 'NodeGenericODE1', 'NodeGenericAE', 'NodeGenericData', 'ObjectGenericODE1',
    'ObjectConnectorCoordinateVector', 'MarkerNodeCoordinate', 'MarkerNodeCoordinates', 'MarkerNodeODE1Coordinate',
    'MarkerNodeRotationCoordinate', 'MarkerObjectODE2Coordinates', 'LoadCoordinate', 'SensorUserFunction']}
removedVisualizationParameters[('ObjectConnectorCoordinateVector', 'color')] = _drawsNothing
removedVisualizationParameters[('ObjectConnectorGravity', 'drawSize')] = 'the connector is drawn as a line; leave drawSize out (#2843)'


#WHAT --fix WRITES INSTEAD OF A REMOVED NAME (#2938): the name replaced, the call left as it is
removedNameRenames = {'NormL2': 'np.linalg.norm', 'VSum': 'np.sum', 'VMult': 'np.dot', 'Vec2Tilde': 'Skew',
                      'Tilde2Vec': 'Skew2Vec', 'eye2D': 'np.eye(2)', 'eye3D': 'np.eye(3)'}
#AddRigidBody(mainSys, inertia, ...) -> mainSys.CreateRigidBody(...) (#2944): its arguments in order, the names
#CreateRigidBody gives them, and the conversion of rotationParameters to a rotation matrix per node type
addRigidBodyArguments = ['mainSys', 'inertia', 'nodeType', 'position', 'velocity', 'rotationMatrix', 'rotationParameters',
                         'angularVelocity', 'gravity', 'graphicsDataList']
createRigidBodyArguments = {'inertia': 'inertia', 'nodeType': 'nodeType', 'position': 'referencePosition',
                            'velocity': 'initialVelocity', 'rotationMatrix': 'referenceRotationMatrix',
                            'angularVelocity': 'initialAngularVelocity', 'gravity': 'gravity',
                            'graphicsDataList': 'graphicsDataList'}
#AddRevoluteJoint and AddPrismaticJoint (#2945): their arguments in order, and the names of the Create function
addJointArguments = ['mbs', 'body0', 'body1', 'point', 'axis', 'useGlobalFrame', 'showJoint', 'axisRadius', 'axisLength']
createJointArguments = {'point': 'position', 'axis': 'axis', 'useGlobalFrame': 'useGlobalFrame', 'showJoint': 'show',
                        'axisRadius': 'axisRadius', 'axisLength': 'axisLength'}
#functions of the FEM interface whose sparse matrix is a scipy matrix since 1.9: CSRtoScipySparseCSR of it is the matrix
femSparseGetters = ['GetMassMatrix', 'GetStiffnessMatrix']
rotationParametersToMatrix = {'RotationEulerParameters': 'EulerParameters2RotationMatrix',
                              'RotationRxyz': 'RotXYZ2RotationMatrix',
                              'RotationRotationVector': 'RotationVector2RotationMatrix'}
#a deprecated argument whose replacement takes another value: --fix converts the value as well (#2938)
argumentValueFixes = {'eulerParametersRef': ('EulerParameters2RotationMatrix({})', 'EulerParameters2RotationMatrix')}
#the deprecated functions of the module that --fix rewrites: the receiver form takes the first argument as the object
#(exu.SolveDynamic(mbs, s) -> mbs.SolveDynamic(s)), the renderer form needs the one SystemContainer of the script, the
#config form is a statement 'exu.SetOutputPrecision(8)' -> 'exu.config.outputPrecision = 8'
moduleFunctionFixes = {'SolveStatic': ('receiver', 'SolveStatic'), 'SolveDynamic': ('receiver', 'SolveDynamic'),
                       'ComputeODE2Eigenvalues': ('receiver', 'ComputeODE2Eigenvalues'),
                       'StartRenderer': ('renderer', 'Start'), 'StopRenderer': ('renderer', 'Stop'),
                       'DoRendererIdleTasks': ('renderer', 'DoIdleTasks'), 'IsRendererActive': ('renderer', 'IsActive'),
                       'SetOutputPrecision': ('config', 'outputPrecision'),
                       'SetLinalgOutputFormatPython': ('config', 'linalgOutputFormatPython'),
                       'SetPrintDelayMilliSeconds': ('config', 'printDelayMilliSeconds'),
                       'SuppressWarnings': ('config', 'suppressWarnings'),
                       'SetWriteToConsole': ('config', 'printToConsole'),
                       'InfoStat': ('special', 'InfoStat')}


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#what definitions/ says is deprecated
def DeprecatedFunctions():
    """{('module'|'SystemContainer'|'renderer'|'MainSystem', name): what to use instead} from the
    pybind definitions, and from the SystemContainer functions that warn in C++ although their
    description does not say DEPRECATED - WaitForRenderEngineStopFlag is one"""
    receivers = {'pybindModule.py': 'module', 'pybindSystemContainer.py': 'SystemContainer',
                 'pybindRenderer.py': 'renderer'}
    found = {}
    for (fileName, receiver) in receivers.items():
        tree = ast.parse(io.open(os.path.join(root, 'definitions', fileName), encoding='utf-8').read())
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Call) and getattr(node.func, 'attr', '') == 'DefPyFunctionAccess'):
                continue
            keywords = {keyword.arg: keyword.value for keyword in node.keywords}
            (name, description) = (keywords.get('pyName'), keywords.get('description'))
            if (isinstance(name, ast.Constant) and isinstance(description, ast.Constant)
                    and str(description.value).startswith('DEPRECATED')):
                key = (receiver, name.value)
                if key in replacements:
                    found[key] = 'use ' + replacements[key]
                else:
                    text = str(description.value)[len('DEPRECATED'):].lstrip(';: ')
                    found[key] = text.split(';')[0]

    #renderer.DeprecationWarning("OldName", "NewName") in the C++ of the SystemContainer
    cpp = io.open(os.path.join(root, 'src', 'Main', 'MainSystemContainer.cpp'), encoding='utf-8').read()
    marker = 'renderer.DeprecationWarning("'
    position = cpp.find(marker)
    while position != -1:
        arguments = cpp[position + len(marker):cpp.index(')', position)]
        (old, new) = [part.strip().strip('"') for part in arguments.split(',')]
        key = ('SystemContainer', old)
        found.setdefault(key, 'use ' + replacements.get(key, 'SC.renderer.' + new + '(...)'))
        position = cpp.find(marker, position + 1)
    found[('MainSystem', 'WaitForUserToContinue')] = 'use ' + replacements[('MainSystem', 'WaitForUserToContinue')]
    return found


def DeprecatedSettings():
    """{(structure member, setting): (the path to use, the version it was deprecated in, the member
    the structure sits in, or None at the top of a settings tree)}"""
    sys.path.insert(0, os.path.join(root, 'tools', 'generators'))
    import structureModel                                                   # noqa: PLC0415
    definitions = structureModel.StructureDefinitions()
    #the member name under which each settings class appears in its parent
    classNames = set(definition['className'] for definition in definitions)
    (memberOf, parentOf) = ({}, {})
    for definition in definitions:
        for member in definition['members']:
            typeName = str(member.get('type', ''))
            if typeName in classNames:
                memberOf.setdefault(typeName, member['pythonName'])
                parentOf.setdefault(typeName, definition['className'])
    #'window' is a deprecated member at the top of the visualization settings AND the current one of
    #every view, so a pair alone is ambiguous: the member above it decides (view0.window is not)
    found = {}
    DeprecatedSettings.topLevel = {}     #{member: (the path to use, the version)} of the top structures (#2813)
    for definition in definitions:
        owner = memberOf.get(definition['className'])
        if owner is None:
            if definition['className'] in ['SimulationSettings', 'VisualizationSettings']:
                for member in definition['members']:
                    deprecated = member.get('deprecated')
                    if deprecated and str(member.get('type', '')) not in classNames:
                        DeprecatedSettings.topLevel[member['pythonName']] = (member['description'].strip(),
                                                                             str(getattr(deprecated, 'since', '')))
            continue
        parentMember = memberOf.get(parentOf.get(definition['className']))
        for member in definition['members']:
            deprecated = member.get('deprecated')
            if deprecated and str(member.get('type', '')) not in classNames: #a deprecated structure: its members are listed
                found[(owner, member['pythonName'])] = (member['description'].strip(),
                                                        str(getattr(deprecated, 'since', '')),
                                                        parentMember)
    DeprecatedSettings.structureMembers = set(memberOf.values())
    return found


def DeprecatedItemParameters():
    """{parameter: {name of the item class, its short name or its dictionary type: (what to do instead, the version
    it was deprecated in, the year it is removed)}}: the renamed item parameters (#2589) and those that stay but are
    deprecated (#2804), from definitions/; and the removed ones of removedItemParameters, with version ''"""
    sys.path.insert(0, os.path.join(root, 'tools', 'generators'))
    import itemModel                                                        # noqa: PLC0415

    def Names(className, shortName):
        names = [className] + ([shortName] if shortName else [])
        for prefix in ['Object', 'Node', 'Marker', 'Load', 'Sensor']:   #the type string of a dictionary
            if className.startswith(prefix):
                names.append(prefix.lower() + 'Type:' + className[len(prefix):])
        return names

    found = {}
    shortNames = {}
    for definition in itemModel.ItemDefinitions():
        shortNames[definition['className']] = definition.get('pythonShortName', '')
        for member in definition['members']:
            deprecated = member.get('deprecated')
            if deprecated is None or 'Function' in member['kind']:
                continue
            advice = deprecated.advice or ('use ' + str(member['description']).strip())
            for name in Names(definition['className'], definition.get('pythonShortName', '')):
                found.setdefault(member['pythonName'], {})[name] = (advice, str(deprecated.since), str(deprecated.expires))
    for ((className, parameter), advice) in removedItemParameters.items():
        for name in Names(className, shortNames.get(className, '')):
            found.setdefault(parameter, {})[name] = (advice, '', '')
    for ((className, parameter), advice) in removedVisualizationParameters.items():
        shortName = shortNames.get(className, '')
        for name in ['V' + className] + (['V' + shortName] if shortName else []):  #VNode1D(show=False)
            found.setdefault(parameter, {})[name] = (advice, '', '')
        for name in Names(className, shortName)[len([className] + ([shortName] if shortName else [])):]:
            found.setdefault('V' + parameter, {})[name] = (advice, '', '')            #{'nodeType':'1D', 'Vshow':False}
    return found


def DeprecatedLibrary():
    """the deprecations of the Python library (#2807): {'functions': {name: (use, since, expires)},
    'arguments': [(function name, or None for any call, argument, use, since, expires)]}; an argument whose
    function is named at runtime (the Create functions name themselves) applies to any call"""
    sys.path.insert(0, os.path.join(root, 'tools', 'generators'))
    import deprecationModel                                                 # noqa: PLC0415
    found = {'functions': {}, 'arguments': []}
    for (entry, kind, functionName, argument, function) in deprecationModel.LibraryDeclarations():
        data = (entry['use'], entry['since'], str(entry['expires']))
        if kind == 'function':
            found['functions'][functionName] = data
        else:
            owner = functionName if function is None else (None if function == '*' else function.split('.')[-1])
            found['arguments'].append((owner, argument) + data)
    return found


def SubmodulesNotLoaded():
    """the submodules of exudyn that 'import exudyn' does not load"""
    os.environ.setdefault('EXUDYN_NO_USER_SETTINGS', '1')
    try:
        import pkgutil                                                      # noqa: PLC0415
        import exudyn                                                       # noqa: PLC0415
        return set(module.name for module in pkgutil.iter_modules(exudyn.__path__)
                   if not hasattr(exudyn, module.name) and not module.name.startswith('_'))
    except Exception:                                                        # noqa: BLE001
        return set()


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def ImportsExudyn(tree):
    for node in ast.walk(tree):
        if isinstance(node, ast.Import) and any(a.name.split('.')[0] == 'exudyn' for a in node.names):
            return True
        if isinstance(node, ast.ImportFrom) and (node.module or '').split('.')[0] == 'exudyn':
            return True
    return False


scopeNodes = (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Lambda)


def ScopeBindings(scope):
    """the names a module, function, class or lambda binds itself - not those of the scopes inside it"""
    bound = set()
    if isinstance(scope, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
        arguments = scope.args
        for argument in arguments.posonlyargs + arguments.args + arguments.kwonlyargs:
            bound.add(argument.arg)
        for argument in [arguments.vararg, arguments.kwarg]:
            if argument is not None:
                bound.add(argument.arg)
    stack = list(ast.iter_child_nodes(scope))
    while stack:
        node = stack.pop()
        if isinstance(node, scopeNodes):
            if not isinstance(node, ast.Lambda):
                bound.add(node.name)
            continue                                     #its own body is its own scope
        if isinstance(node, ast.Name) and isinstance(node.ctx, (ast.Store, ast.Del)):
            bound.add(node.id)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:
                if alias.name != '*':
                    bound.add(alias.asname or alias.name.split('.')[0])
        elif isinstance(node, ast.ExceptHandler) and node.name:
            bound.add(node.name)
        elif isinstance(node, (ast.Global, ast.Nonlocal)):
            bound.update(node.names)
        stack += list(ast.iter_child_nodes(node))
    return bound


def UnboundLoads(tree):
    """[(node, name)] of every name that is read where neither its scope, an enclosing one, the module
    nor the builtins bind it - what a star import used to supply"""
    result = []
    moduleBound = ScopeBindings(tree) | set(dir(builtins))

    def Walk(node, visible):
        for child in ast.iter_child_nodes(node):
            if isinstance(child, scopeNodes):
                inner = visible if isinstance(child, ast.ClassDef) else visible | ScopeBindings(child)
                Walk(child, inner)
            else:
                if isinstance(child, ast.Name) and isinstance(child.ctx, ast.Load)                         and child.id not in visible:
                    result.append(child)
                Walk(child, visible)
    Walk(tree, moduleBound)
    return result


def AttributeChain(node):
    """['SC', 'visualizationSettings', 'general', 'drawWorldBasis'] for SC.visualizationSettings...,
    or None if the chain does not start at a name"""
    chain = []
    while isinstance(node, ast.Attribute):
        chain.insert(0, node.attr)
        node = node.value
    if isinstance(node, ast.Name):
        return [node.id] + chain
    return None


def LeadingString(node):
    """the first literal piece of a file name: 'a.txt', 'dir/' + name, f'dir/{x}.txt' - or None"""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        return LeadingString(node.left)
    if isinstance(node, ast.JoinedStr) and node.values and isinstance(node.values[0], ast.Constant):
        return str(node.values[0].value)
    return None


def IsLocal(name):
    """True if a file name has no directory: the file lands where the script runs"""
    return name != '' and '/' not in name and chr(92) not in name


def CheckTree(tree, tables, fixes=None, source=None):
    """the findings of one parsed script: [(line, text)]; fixes, if a list, receives what --fix rewrites: (attribute
    node, number of trailing names to replace, the names to write instead) for a renamed setting (#2813), ('span', line,
    start, end, text) and ('range', line, start, endLine, end, text) for a replacement, ('import', line of code) for an
    import to add; the fixes that rewrite a call need the source (#2938)"""
    findings = []
    parents = {}
    for parent in ast.walk(tree):
        for child in ast.iter_child_nodes(parent):
            parents[id(child)] = parent
    moduleBound = ScopeBindings(tree)
    definedFunctions = set(n.name for n in ast.walk(tree) if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)))
    #the one SystemContainer of the script, which a module function of the renderer becomes a method of
    containers = set(node.targets[0].id for node in tree.body if isinstance(node, ast.Assign) and len(node.targets) == 1
                     and isinstance(node.targets[0], ast.Name) and isinstance(node.value, ast.Call)
                     and getattr(node.value.func, 'attr', getattr(node.value.func, 'id', '')) == 'SystemContainer')
    container = next(iter(containers)) if len(containers) == 1 else None

    def Fix(*fix):
        if fixes is not None:
            fixes.append(fix)

    def Replace(node, text):
        Fix('range', node.lineno, node.col_offset, node.end_lineno, node.end_col_offset, text)

    def Import(name, line):
        """an import of the line, unless the script binds the name already"""
        if name not in moduleBound:
            Fix('import', line)

    def Segment(node):
        return ast.get_source_segment(source, node) if source is not None else None

    def Arguments(call):
        """the source of each argument of a call, keywords as 'name=value'; None if there are * or ** arguments"""
        if source is None or any(isinstance(a, ast.Starred) for a in call.args) or any(k.arg is None for k in call.keywords):
            return None
        return [Segment(a) for a in call.args] + [k.arg + '=' + Segment(k.value) for k in call.keywords]

    def CallOf(node):
        parent = parents.get(id(node))
        return parent if isinstance(parent, ast.Call) and parent.func is node else None

    sourceLines = source.split('\n') if source is not None else []

    def StandsAlone(statement):
        """True if nothing but the statement and a comment is on its lines"""
        before = sourceLines[statement.lineno - 1].encode('utf-8')[:statement.col_offset].decode('utf-8', 'replace')
        after = sourceLines[statement.end_lineno - 1].encode('utf-8')[statement.end_col_offset:].decode('utf-8', 'replace')
        return before.strip() == '' and (after.strip() == '' or after.strip().startswith('#'))

    def DeleteStatement(node):
        """the statement node belongs to deleted with its lines - or replaced by 'pass' if it is the only one of its
        block; nothing if it shares a line with another statement (#2938)"""
        while node is not None and not isinstance(node, ast.stmt):
            node = parents.get(id(node))
        if node is None or source is None or not StandsAlone(node):
            return
        parent = parents.get(id(node))
        block = next((getattr(parent, field) for field in ['body', 'orelse', 'finalbody']
                      if node in getattr(parent, field, [])), [node, None])
        if len(block) == 1:
            Replace(node, 'pass')
        else:
            Fix('range', node.lineno, 0, node.end_lineno + 1, 0, '')

    def DeleteItem(items, k):
        """item k of a list of argument nodes (or of (key, value) pairs) deleted with its comma"""
        (first, last) = items[k] if isinstance(items[k], tuple) else (items[k], items[k])
        if k > 0:
            previous = items[k - 1][1] if isinstance(items[k - 1], tuple) else items[k - 1]
            Fix('range', previous.end_lineno, previous.end_col_offset, last.end_lineno, last.end_col_offset, '')
        elif k + 1 < len(items):
            following = items[k + 1][0] if isinstance(items[k + 1], tuple) else items[k + 1]
            Fix('range', first.lineno, first.col_offset, following.lineno, following.col_offset, '')
        else:
            Fix('range', first.lineno, first.col_offset, last.end_lineno, last.end_col_offset, '')

    def DeleteArgument(call, keyword):
        items = sorted(list(call.args) + list(call.keywords), key=lambda n: (n.lineno, n.col_offset))
        DeleteItem(items, items.index(keyword))

    def IsEmptyList(node):
        return isinstance(node, (ast.List, ast.Tuple)) and len(node.elts) == 0

    def FixAddRigidBody(call):
        """AddRigidBody(mbs, inertia, ...) -> mbs.CreateRigidBody(inertia=..., referencePosition=..., ...) (#2944); the
        returned [nodeNumber, bodyNumber] becomes the body, and the node is read from it with mbs.GetObject(body)
        ['nodeNumber']: for '[n, b] = AddRigidBody(...)', for 'rb = AddRigidBody(...)' used only as rb[0] and rb[1], and
        for a call whose result is not used; any other use stays reported"""
        names = addRigidBodyArguments
        if Arguments(call) is None or len(call.args) > len(names):
            return
        values = dict(zip(names, call.args))
        values.update((k.arg, k.value) for k in call.keywords)
        if 'mainSys' not in values or 'inertia' not in values or not set(values) <= set(names):
            return
        parts = []
        for name in names[1:]:
            if name not in values:
                continue
            if name in ['rotationMatrix', 'rotationParameters'] and IsEmptyList(values[name]):
                continue
            if name == 'rotationParameters':
                if 'rotationMatrix' in values and not IsEmptyList(values['rotationMatrix']):
                    return
                nodeType = Segment(values['nodeType']) if 'nodeType' in values else 'RotationEulerParameters'
                function = next((f for (key, f) in rotationParametersToMatrix.items() if nodeType.endswith(key)), None)
                if function is None:
                    return                           #a node type that is no literal: the user decides
                if not set(starModules) & {'exudyn.utilities', 'exudyn.rigidBodyUtilities'}:
                    Import(function, 'from exudyn.rigidBodyUtilities import ' + function)
                parts.append('referenceRotationMatrix=' + function + '(' + Segment(values[name]) + ')')
                continue
            parts.append(createRigidBodyArguments[name] + '=' + Segment(values[name]))
        system = Parenthesized(Segment(values['mainSys']))
        FixResultList(call, system + '.CreateRigidBody(' + ', '.join(parts) + ')', system, 1,
                      {0: "{system}.GetObject({name})['nodeNumber']"})

    def FixResultList(call, created, system, keep, others):
        """a call that returned a list and is replaced by one that returns its entry 'keep': '[a, b] = F(...)' (or 'a, b =')
        gets 'b = created' and a line per other entry, written from others[k] with {system} and {name}; 'r = F(...)' used
        only as r[k] gets r[keep] -> r and r[k] -> others[k]; a result that is not used is replaced as it is; any other
        use is left (#2944, #2945)"""
        parent = parents.get(id(call))
        if isinstance(parent, ast.Expr):
            Replace(call, created)
            return
        if not (isinstance(parent, ast.Assign) and len(parent.targets) == 1 and parent.value is call):
            return
        target = parent.targets[0]
        size = 1 + len(others)
        if isinstance(target, (ast.Tuple, ast.List)) and len(target.elts) == size \
                and all(isinstance(e, ast.Name) for e in target.elts):
            names = [e.id for e in target.elts]
            if not StandsAlone(parent):
                return
            kept = names[keep]
            if kept == '_':                  #the entry the call returns now is not used: one other, assigned directly
                used = [k for k in range(size) if k != keep and names[k] != '_']
                if len(used) != 1:
                    return
                Replace(parent, names[used[0]] + ' = ' + others[used[0]].format(system=system, name=created))
                return
            Replace(parent, kept + ' = ' + created)
            line = sourceLines[parent.lineno - 1]
            indent = line[:len(line) - len(line.lstrip())]
            added = ''.join(indent + names[k] + ' = ' + others[k].format(system=system, name=kept) + '\n'
                            for k in range(size) if k != keep and names[k] != '_')
            if added:
                Fix('range', parent.end_lineno + 1, 0, parent.end_lineno + 1, 0, added)
        elif isinstance(target, ast.Name):
            name = target.id
            stores = [n for n in ast.walk(tree) if isinstance(n, ast.Name) and n.id == name and isinstance(n.ctx, ast.Store)]
            loads = [n for n in ast.walk(tree) if isinstance(n, ast.Name) and n.id == name and isinstance(n.ctx, ast.Load)]
            subscripts = [parents.get(id(n)) for n in loads]
            if len(stores) != 1 or not all(isinstance(s, ast.Subscript) and isinstance(s.slice, ast.Constant)
                                           and s.slice.value in range(size) and isinstance(s.ctx, ast.Load)
                                           for s in subscripts):
                return
            Replace(call, created)
            for s in subscripts:
                Replace(s, name if s.slice.value == keep else others[s.slice.value].format(system=system, name=name))

    def FixAddJoint(call, functionName):
        """AddRevoluteJoint/AddPrismaticJoint(mbs, body0, body1, point, axis, ...) -> mbs.CreateRevoluteJoint(itemNumbers=
        [body0, body1], position=point, axis=axis, ...); of the returned [joint, marker0, marker1] the joint stays, the
        markers are read with mbs.GetObject(joint)['markerNumbers'] (#2945)"""
        names = addJointArguments
        if Arguments(call) is None or len(call.args) > len(names):
            return
        values = dict(zip(names, call.args))
        values.update((k.arg, k.value) for k in call.keywords)
        if not all(n in values for n in names[:5]) or not set(values) <= set(names):
            return
        parts = ['itemNumbers=[' + Segment(values['body0']) + ', ' + Segment(values['body1']) + ']']
        parts += [createJointArguments[n] + '=' + Segment(values[n]) for n in names[3:] if n in values]
        system = Parenthesized(Segment(values['mbs']))
        created = system + '.' + functionName.replace('Add', 'Create', 1) + '(' + ', '.join(parts) + ')'
        FixResultList(call, created, system, 0, {1: "{system}.GetObject({name})['markerNumbers'][0]",
                                                 2: "{system}.GetObject({name})['markerNumbers'][1]"})

    def FixCSRtoScipySparseCSR(call):
        """CSRtoScipySparseCSR(fem.GetMassMatrix(sparse=True)) -> fem.GetMassMatrix(sparse=True), which is a scipy matrix
        since 1.9; for any other argument the matrix may be one or sparse triplets, and the call stays reported (#2945)"""
        if len(call.args) == 1 and not call.keywords and isinstance(call.args[0], ast.Call) \
                and getattr(call.args[0].func, 'attr', '') in femSparseGetters:
            Replace(call, Segment(call.args[0]))

    def FixCornersToBrick(call, functionName):
        """a box or rectangle given by its corners as graphics.Brick(centerPoint=..., size=...) (#2938, #2941):
        BrickXYZ(x0, y0, z0, x1, y1, z1, ...) keeps its other arguments; GraphicsDataOrthoCubeLines(x0, y0, z0, x1, y1,
        z1, color) and GraphicsDataRectangle(x0, y0, x1, y1, color) draw edges only, the rectangle with size 0 in z"""
        if functionName == 'BrickXYZ':
            names = ['xMin', 'yMin', 'zMin', 'xMax', 'yMax', 'zMax', 'color', 'addNormals', 'addEdges', 'edgeColor',
                     'addFaces']
        elif functionName == 'GraphicsDataOrthoCubeLines':
            names = ['xMin', 'yMin', 'zMin', 'xMax', 'yMax', 'zMax', 'color']
        else:
            names = ['xMin', 'yMin', 'xMax', 'yMax', 'color']
        corners = [n for n in names if n[1:] in ['Min', 'Max']]
        if Arguments(call) is None or len(call.args) > len(names):
            return
        values = dict(zip(names, call.args))
        values.update((k.arg, k.value) for k in call.keywords)
        if not all(name in values for name in corners) or not set(values) <= set(names):
            return

        def Combined(a, b, minus):
            if a not in values:              #the rectangle has no z: center 0, size 0
                return '0'
            try:
                (x, y) = (ast.literal_eval(values[a]), ast.literal_eval(values[b]))
                return repr(y - x if minus else 0.5 * (x + y))
            except (ValueError, TypeError, SyntaxError):
                (x, y) = (Parenthesized(Segment(values[a])), Parenthesized(Segment(values[b])))
                return y + '-' + x if minus else '0.5*(' + x + '+' + y + ')'
        axes = [('xMin', 'xMax'), ('yMin', 'yMax'), ('zMin', 'zMax')]
        center = '[' + ', '.join(Combined(a, b, False) for (a, b) in axes) + ']'
        size = '[' + ', '.join(Combined(a, b, True) for (a, b) in axes) + ']'
        if functionName == 'BrickXYZ':
            rest = [name + '=' + Segment(values[name]) for name in names[6:] if name in values]
            function = Segment(call.func)[:-len('BrickXYZ')] + 'Brick'
        else:                                #edges only, in the color of the lines
            rest = ['addFaces=False', 'addEdges=True'] + (['edgeColor=' + Segment(values['color'])]
                                                          if 'color' in values else [])
            function = 'graphics.Brick'
            Import('graphics', 'import exudyn.graphics as graphics')
        Replace(call, function + '(centerPoint=' + center + ', size=' + size + ''.join(', ' + r for r in rest) + ')')

    def Parenthesized(text):
        return text if re.fullmatch(r'[\w.]+', text) else '(' + text + ')'

    def FixRemovedName(node, name):
        """a removed name: a GraphicsData alias, a vector helper; the call rewritten where its form changes"""
        use = removedNames[name]
        call = CallOf(node)
        if use.startswith('graphics.'):
            Replace(node, use.split('(')[0])
            Import('graphics', 'import exudyn.graphics as graphics')
        elif name in removedNameRenames:
            Replace(node, removedNameRenames[name])
            if removedNameRenames[name].startswith('np.'):
                Import('np', 'import numpy as np')
            elif not set(starModules) & {'exudyn.utilities', 'exudyn.rigidBodyUtilities'}:
                Import(removedNameRenames[name], 'from exudyn.rigidBodyUtilities import ' + removedNameRenames[name])
        elif call is not None and name in ['VAdd', 'VSub', 'ScalarMult', 'DiagonalMatrix'] \
                and Arguments(call) is not None:
            a = Arguments(call)
            text = None
            if name in ['VAdd', 'VSub'] and len(a) == 2 and not call.keywords:
                text = '(np.array(' + a[0] + ') ' + ('+' if name == 'VAdd' else '-') + ' np.array(' + a[1] + '))'
            elif name == 'ScalarMult' and len(a) == 2 and not call.keywords:
                text = Parenthesized(a[0]) + '*np.array(' + a[1] + ')'
            elif name == 'DiagonalMatrix':       #DiagonalMatrix(rowsColumns, value=1)
                values = dict(zip(['rowsColumns', 'value'], [Segment(x) for x in call.args]))
                values.update((k.arg, Segment(k.value)) for k in call.keywords)
                if set(values) <= {'rowsColumns', 'value'} and 'rowsColumns' in values:
                    text = 'np.eye(' + values['rowsColumns'] + ')'
                    if 'value' in values:
                        text = Parenthesized(values['value']) + '*' + text
            if text is not None:
                Replace(call, text)
                Import('np', 'import numpy as np')
    unbound = set(id(node) for node in UnboundLoads(tree))
    starModules = [node.module for node in ast.walk(tree)
                   if isinstance(node, ast.ImportFrom) and (node.module or '').startswith('exudyn')
                   and any(alias.name == '*' for alias in node.names)]
    starImport = len(starModules) != 0
    exudynAliases = set(['exudyn'])
    importedSubmodules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                parts = alias.name.split('.')
                if parts[0] == 'exudyn':
                    if len(parts) == 1:
                        exudynAliases.add(alias.asname or 'exudyn')
                    else:
                        importedSubmodules.add(parts[1])
        elif isinstance(node, ast.ImportFrom) and (node.module or '').startswith('exudyn.'):
            importedSubmodules.add(node.module.split('.')[1])

    reported = set()

    def Report(line, key, text):
        if key not in reported:
            reported.add(key)
            findings.append((line, text))

    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and id(node) in unbound:
            if starImport and node.id in formerStarNames:
                Report(node.lineno, ('star', node.id), "'" + node.id + "' no longer comes with "
                       "'from " + starModules[0] + " import *'; add: " + formerStarNames[node.id])
                Fix('import', formerStarNames[node.id])
            elif ('exudyn.utilities' in starModules and node.id in utilitiesNames
                  and utilitiesNames[node.id][0] not in starModules):
                Report(node.lineno, ('utilities', node.id), "'" + node.id + "' no longer comes with "
                       "'from exudyn.utilities import *'; use: " + utilitiesNames[node.id][1])
                use = utilitiesNames[node.id][1]
                call = CallOf(node)
                if use.startswith('from '):
                    Fix('import', use)
                elif node.id.startswith('color4') and use.startswith('graphics.color.'):
                    Replace(node, use.split(' ')[0])
                    Import('graphics', 'import exudyn.graphics as graphics')
                elif use.startswith('mbs.') and call is not None and call.args and Arguments(call) is not None:
                    a = Arguments(call)              #DrawSystemGraph(mbs, ...) -> mbs.DrawSystemGraph(...)
                    Replace(call, Parenthesized(a[0]) + '.' + node.id + '(' + ', '.join(a[1:]) + ')')
            elif node.id in removedNames:
                Report(node.lineno, ('removed', node.id), "'" + node.id + "' is removed; use "
                       + removedNames[node.id])
                FixRemovedName(node, node.id)

        elif isinstance(node, ast.Attribute):
            chain = AttributeChain(node)
            if chain is None:
                continue
            if chain[-1] in removedNames and len(chain) == 2 and chain[0] in exudynAliases | {'eii'}:
                Report(node.lineno, ('removed', chain[-1]), "'" + '.'.join(chain) + "' is removed; use "
                       + removedNames[chain[-1]])
                FixRemovedName(node, chain[-1])
            if len(chain) >= 3:
                pair = (chain[-2], chain[-1])
                if pair in tables['settings']:
                    (use, version, parentMember) = tables['settings'][pair]
                    above = chain[-3]
                    if '.' in use and (parentMember is not None and above != parentMember) or                             (parentMember is None and (above in DeprecatedSettings.structureMembers
                                                       or above.startswith('view'))):
                        continue                          #the same pair in another structure
                    advice = ('it has no effect; remove it' if use.endswith('.dummy') else 'use ' + use)
                    Report(node.lineno, ('setting',) + pair, "'" + '.'.join(pair) + "' is deprecated"
                           + (' since ' + version if version else '') + '; ' + advice)
                    if use.endswith('.dummy') and isinstance(parents.get(id(node)), ast.Assign):
                        DeleteStatement(node)
                    if fixes is not None and not use.endswith('.dummy'):
                        if '.' not in use:                  #a rename in its own structure
                            fixes.append((node, 1, [use]))
                        else:                               #a path from the top structure
                            fixes.append((node, 2 if parentMember is None else 3, use.split('.')))
                if pair in removedSettings:
                    Report(node.lineno, ('removedSetting',) + pair, "'" + '.'.join(pair)
                           + "' is removed; use " + removedSettings[pair])
                    if isinstance(parents.get(id(node)), ast.Assign):
                        DeleteStatement(node)
            topLevel = getattr(DeprecatedSettings, 'topLevel', {})
            if (len(chain) >= 2 and chain[-1] in topLevel and chain[-2] != 'config'
                    and chain[-2] not in getattr(DeprecatedSettings, 'structureMembers', set())):
                (use, version) = topLevel[chain[-1]]  #a deprecated member of the top of a settings tree (#2813)
                Report(node.lineno, ('settingTop', chain[-1]), "'" + chain[-1] + "' is deprecated"
                       + (' since ' + version if version else '') + '; use ' + use)
                if fixes is not None:
                    fixes.append((node, 1, use.split('.')))
            if len(chain) == 2 and chain[0] in exudynAliases:
                if ('module', chain[1]) in tables['functions']:
                    Report(node.lineno, ('function', chain[1]), "'" + '.'.join(chain)
                           + "' is deprecated: " + tables['functions'][('module', chain[1])])
                    (form, newName) = moduleFunctionFixes.get(chain[1], (None, None))
                    call = CallOf(node)
                    arguments = Arguments(call) if call is not None else None
                    if form == 'receiver' and call is not None and call.args and arguments is not None:
                        Replace(call, Parenthesized(arguments[0]) + '.' + newName + '(' + ', '.join(arguments[1:]) + ')')
                    elif form == 'renderer' and container is not None:
                        Replace(node, container + '.renderer.' + newName)
                    elif form == 'special':
                        Replace(node, chain[0] + '.special.' + newName)
                    elif form == 'config' and call is not None and arguments is not None and len(arguments) == 1 \
                            and not call.keywords and isinstance(parents.get(id(call)), ast.Expr):
                        Replace(parents[id(call)], chain[0] + '.config.' + newName + ' = ' + arguments[0])
            if len(chain) >= 2 and chain[0] in exudynAliases and chain[1] in tables['submodules'] \
                    and chain[1] not in importedSubmodules:
                Report(node.lineno, ('submodule', chain[1]), "'" + chain[0] + '.' + chain[1]
                       + "' is used, but 'import exudyn' does not load it; add: import exudyn."
                       + chain[1])
                Fix('import', 'import exudyn.' + chain[1])
            if len(chain) >= 2 and chain[0] not in exudynAliases:
                receivers = ['renderer'] if chain[-2] == 'renderer' else ['SystemContainer', 'MainSystem']
                for receiver in receivers:
                    if (receiver, chain[-1]) in tables['functions']:
                        Report(node.lineno, ('method', receiver, chain[-1]), "'" + '.'.join(chain[-2:])
                               + "' is deprecated: " + tables['functions'][(receiver, chain[-1])])
                        advice = tables['functions'][(receiver, chain[-1])]
                        method = re.match(r'use SC\.renderer\.(\w+)\(', advice)
                        call = CallOf(node)
                        if receiver == 'SystemContainer' and method is not None and chain[-2] != 'renderer':
                            #SC.GetRenderState() -> SC.renderer.GetState(): the name after the object replaced
                            Fix('range', node.value.end_lineno, node.value.end_col_offset, node.end_lineno,
                                node.end_col_offset, '.renderer.' + method.group(1))
                        elif (receiver == 'MainSystem' and method is not None and container is not None
                              and call is not None and not call.args and not call.keywords):
                            Replace(call, container + '.renderer.' + method.group(1) + '()')
                        break

        elif isinstance(node, ast.Call) and (tables.get('itemParameters') or tables.get('library')):
            className = getattr(node.func, 'attr', getattr(node.func, 'id', ''))
            #a deprecated function or argument of the library (#2807)
            library = tables.get('library') or {'functions': {}, 'arguments': []}
            #a function the script defines itself, called by its name, is not the one of the library (#2945)
            ownFunction = isinstance(node.func, ast.Name) and className in definedFunctions
            if className in library['functions'] and not ownFunction:
                (use, since, expires) = library['functions'][className]
                Report(node.lineno, ('library', className), "'" + className + "' is deprecated since " + since
                       + ' and removed in ' + expires + ('; use ' + use if use else ''))
                if className in ['BrickXYZ', 'GraphicsDataOrthoCubeLines', 'GraphicsDataRectangle'] and fixes is not None:
                    FixCornersToBrick(node, className)
                elif className == 'AddRigidBody' and fixes is not None:
                    FixAddRigidBody(node)
                elif className in ['AddRevoluteJoint', 'AddPrismaticJoint'] and fixes is not None:
                    FixAddJoint(node, className)
                elif className == 'CSRtoScipySparseCSR' and fixes is not None:
                    FixCSRtoScipySparseCSR(node)
            for (owner, argument, use, since, expires) in library['arguments']:
                if owner in (None, className) and any(keyword.arg == argument for keyword in node.keywords):
                    Report(node.lineno, ('libraryArgument', className, argument), className + '(' + argument
                           + '=...): the argument is deprecated since ' + since + ' and removed in ' + expires
                           + ('; use ' + use if use else ''))
                    if fixes is not None and use.isidentifier():  #a renamed argument (#2863)
                        for keyword in node.keywords:
                            if keyword.arg != argument:
                                continue
                            if argument in argumentValueFixes:   #the value converted too (#2938)
                                (template, function) = argumentValueFixes[argument]
                                if source is not None:
                                    Replace(keyword, use + '=' + template.format(Segment(keyword.value)))
                                    if not set(starModules) & {'exudyn.utilities', 'exudyn.rigidBodyUtilities'}:
                                        Import(function, 'from exudyn.rigidBodyUtilities import ' + function)
                            else:
                                fixes.append(('span', keyword.lineno, keyword.col_offset,
                                              keyword.col_offset + len(argument), use))
            #a deprecated item parameter as keyword of its item class (#2805)
            for keyword in node.keywords:
                entry = (tables.get('itemParameters') or {}).get(keyword.arg, {}).get(className)
                if entry is not None:
                    Report(keyword.value.lineno, ('itemParameter', className, keyword.arg),
                           ItemParameterFinding(className + '(' + keyword.arg + '=...)', entry))
                    if entry[1] == '' and className.startswith('V') and fixes is not None:
                        DeleteArgument(node, keyword)          #a visualization parameter that does nothing (#2938)
                    if fixes is not None and entry[0].startswith('use ') and entry[1] != '': #a renamed parameter (#2814)
                        fixes.append(('span', keyword.lineno, keyword.col_offset, keyword.col_offset + len(keyword.arg),
                                      entry[0][len('use '):]))

        elif isinstance(node, ast.Dict) and tables.get('itemParameters'):
            #and as key of an item dictionary: {'objectType': 'JointGeneric', 'rotationMarker0': ...}
            keys = [key.value if isinstance(key, ast.Constant) else None for key in node.keys]
            typeName = None
            for (key, value) in zip(keys, node.values):
                if isinstance(key, str) and key.endswith('Type') and isinstance(value, ast.Constant):
                    typeName = key + ':' + str(value.value)
            if typeName is not None:
                for (key, value) in zip(keys, node.values):
                    entry = tables['itemParameters'].get(key, {}).get(typeName) if isinstance(key, str) else None
                    if entry is not None:
                        Report(value.lineno, ('itemParameter', typeName, key),
                               ItemParameterFinding("'" + key + "' of " + typeName.split(':')[1], entry))
                        if entry[1] == '' and key.startswith('V') and fixes is not None:
                            pairs = list(zip(node.keys, node.values))
                            DeleteItem(pairs, keys.index(key))  #a visualization parameter that does nothing (#2938)
                        keyNode = node.keys[keys.index(key)]
                        if fixes is not None and entry[0].startswith('use ') and entry[1] != '' and keyNode.lineno == keyNode.end_lineno:
                            fixes.append(('span', keyNode.lineno, keyNode.col_offset + 1, keyNode.end_col_offset - 1,
                                          entry[0][len('use '):]))   #inside the quotes

        elif isinstance(node, ast.keyword) and node.arg in removedKeywords:
            Report(node.value.lineno, ('keyword', node.arg), "argument '" + node.arg
                   + "' is removed: " + removedKeywords[node.arg])
            if fixes is not None and node.arg in removedKeywordRenames:
                fixes.append(('span', node.lineno, node.col_offset, node.col_offset + len(node.arg),
                              removedKeywordRenames[node.arg]))
            elif isinstance(parents.get(id(node)), ast.Call):
                DeleteArgument(parents[id(node)], node)        #an argument that does nothing any more (#2938)

    #WHERE THE FILES GO (#2718): a file named without a directory is written where the script runs,
    #and the default solution files are in solution/ now, so a script that reads one of them by its
    #old name reads nothing - unless it writes that name itself
    (written, insideTargets) = (set(), set())
    for node in ast.walk(tree):
        target = None
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and                 isinstance(node.targets[0], ast.Attribute) and (node.targets[0].attr in outputSettings or (node.targets[0].attr == 'name' and getattr(node.targets[0].value, 'attr', '') in ['file', 'restart'])):
            (target, what) = (node.value, node.targets[0].attr)
        elif isinstance(node, ast.Call):
            name = getattr(node.func, 'attr', getattr(node.func, 'id', ''))
            for keyword in node.keywords:
                if keyword.arg in outputKeywords and outputKeywords[keyword.arg] in name:
                    (target, what) = (keyword.value, name + '(' + keyword.arg + '=...)')
        if target is None:
            continue
        leading = LeadingString(target)
        insideTargets.update(id(part) for part in ast.walk(target))
        if isinstance(target, ast.Constant):
            written.add(target.value)
        if leading is not None and IsLocal(leading):
            Report(target.lineno, ('local', target.lineno), what + " writes '" + leading
                   + ("..." if not isinstance(target, ast.Constant) else '') + "' where the script "
                   "runs; name a directory: 'solution/" + leading + ("...'" if not isinstance(target, ast.Constant) else "'"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str)                 and node.value in formerDefaultFiles and node.value not in written                 and id(node) not in insideTargets:
            Report(node.lineno, ('default', node.value), "'" + node.value + "' is not where Exudyn "
                   "writes it any more; the default is '" + formerDefaultFiles[node.value] + "'")
            text = Segment(node)
            if text is not None and len(text) >= 2 and text[0] in '\'"' and text[-1] == text[0]:
                Replace(node, text[0] + formerDefaultFiles[node.value] + text[0])

    return sorted(findings)


def ItemParameterFinding(where, entry):
    """the text of a deprecated or removed item parameter"""
    (advice, since, expires) = entry
    if since == '':
        return where + ': the parameter is removed; ' + advice
    return where + ': the parameter is deprecated since ' + since + ' and removed in ' + expires + '; ' + advice


def ApplyFixes(source, fixes):
    """the source with the fixes of CheckTree applied: a renamed setting replaces the last names of an attribute chain
    (on one line); a 'span' or 'range' replaces the text between two positions; an 'import' adds a line after the
    imports at the top - after the star import from exudyn, if there is one, else after the last import - unless the
    script has that line already. The columns of the ast are UTF-8 bytes. Overlapping replacements: the first is
    kept, the other is left for the next pass"""
    data = source.encode('utf-8')
    lineStarts = [0]
    for (k, byte) in enumerate(data):
        if byte == 10:
            lineStarts.append(k + 1)

    def Offset(line, column):
        return lineStarts[line - 1] + column if line - 1 < len(lineStarts) else len(data)

    edits = []                                 #(start, end, text)
    imports = []
    existing = [line.strip() for line in source.split('\n')]
    for fix in fixes:
        if fix[0] == 'span':                   #(span, line, start, end, text)
            edits.append((Offset(fix[1], fix[2]), Offset(fix[1], fix[3]), fix[4]))
        elif fix[0] == 'range':                #(range, line, start, endLine, end, text)
            edits.append((Offset(fix[1], fix[2]), Offset(fix[3], fix[4]), fix[5]))
        elif fix[0] == 'import':
            if fix[1] not in imports and fix[1] not in existing:
                imports.append(fix[1])
        else:
            (node, count, names) = fix
            base = node
            for i in range(count):
                base = base.value
            if base.end_lineno != node.end_lineno:
                continue                        #a chain over two lines is left to the user
            edits.append((Offset(node.end_lineno, base.end_col_offset), Offset(node.end_lineno, node.end_col_offset),
                          '.' + '.'.join(names)))
    if imports:
        tree = ast.parse(source)
        top = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom))]
        star = [node for node in top if isinstance(node, ast.ImportFrom) and (node.module or '').startswith('exudyn')
                and any(alias.name == '*' for alias in node.names)]
        if star:
            at = Offset(star[0].end_lineno + 1, 0)
        elif top:
            at = Offset(top[-1].end_lineno + 1, 0)
        elif tree.body:                         #no import at the top: before the first statement after a docstring
            first = tree.body[1] if (len(tree.body) > 1 and isinstance(tree.body[0], ast.Expr)
                                     and isinstance(tree.body[0].value, ast.Constant)) else tree.body[0]
            at = Offset(first.lineno, 0)
        else:
            at = len(data)
        prefix = '' if at == 0 or data[at - 1:at] == b'\n' else '\n'
        edits.append((at, at, prefix + ''.join(line + '  #added by exudev scripts --fix\n' for line in imports)))
    applied = []
    for (start, end, text) in sorted(edits, key=lambda e: (e[0], e[1])):
        if applied and start < applied[-1][1]:
            continue                            #overlaps the one before: left for the next pass
        applied.append((start, end, text))
    for (start, end, text) in reversed(applied):
        data = data[:start] + text.encode('utf-8') + data[end:]
    return data.decode('utf-8')


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#running the scripts in a copy (#2713)
def PathsOutsideTheFolder(tree):
    """[(line, path)]: the strings of a script that name an absolute path - 'C:/...', '\\\\server', '/home/...' - or
    leave its folder - '../data' -: a copy of the script would read or write somewhere else"""
    found = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            text = node.value.strip()
            if (re.match(r'^[A-Za-z]:[\\/]', text) or text.startswith('\\\\') or re.match(r'^/[\w.-]+/', text)
                    or '../' in text or '..\\' in text):
                found.append((node.lineno, text))
    return found


#the process a script runs in: no windows (the environment), a solver timeout, the script as __main__
_runner = """
import sys, runpy
import exudyn as exu
exu.special.solver.timeout = float(sys.argv[2])
script = sys.argv[1]
sys.argv = [script]
runpy.run_path(script, run_name='__main__')
"""


def RunScript(path, workFolder, timeout=120, solverTimeout=2):
    """run a copied script in its folder; returns (result, text): 'ran', 'failed' with the last line of the error,
    'timeout', or 'needs' with the package that is missing"""
    environment = dict(os.environ, EXUDYN_SUPPRESS_UI_WINDOW_OPEN='1')
    environment.pop('EXUDYN_OUTPUTDIRECTORY', None)      #the copy is the place for its files
    try:
        result = subprocess.run([sys.executable, '-c', _runner, path, str(solverTimeout)], cwd=workFolder,
                                env=environment, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return ('timeout', 'did not end within ' + str(timeout) + ' s')
    if result.returncode == 0:
        return ('ran', '')
    lines = [line for line in result.stderr.strip().split('\n') if line.strip()]
    last = lines[-1] if lines else 'exit code ' + str(result.returncode)
    missing = re.match(r"ModuleNotFoundError: No module named '([^'.]+)", last)
    if missing and missing.group(1) != 'exudyn':
        return ('needs', missing.group(1))
    return ('failed', last)


def RunScripts(scripts, timeout=120, solverTimeout=2, keep=False):
    """scripts: [(path, shown name)]; each folder copied once into a temporary folder, each script run in its copy;
    returns [(shown, result, text)]"""
    workRoot = tempfile.mkdtemp(prefix='exudevScripts')
    copies = {}
    results = []
    try:
        for (path, shown) in scripts:
            folder = os.path.dirname(os.path.abspath(path))
            if folder not in copies:
                copies[folder] = os.path.join(workRoot, str(len(copies)))
                shutil.copytree(folder, copies[folder], ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '.git'))
            copy = os.path.join(copies[folder], os.path.basename(path))
            (result, text) = RunScript(copy, copies[folder], timeout, solverTimeout)
            results.append((shown, result, text))
    finally:
        if not keep:
            shutil.rmtree(workRoot, ignore_errors=True)
    return results


def FixSource(source, tree, tables, path='<script>'):
    """(the fixed source, its tree, the number of fixes): the fixes applied in up to three passes, since a fix inside
    another one (a removed helper in the argument of a deprecated function) waits for the next pass; a fixed source
    that does not parse raises before anything is written"""
    nFixed = 0
    for attempt in range(3):
        fixes = []
        CheckTree(tree, tables, fixes, source)
        fixed = ApplyFixes(source, fixes) if fixes else source
        if fixed == source:
            break
        newTree = ast.parse(fixed, filename=path)
        nFixed += len(fixes)
        (source, tree) = (fixed, newTree)
    return (source, tree, nFixed)


def PythonFiles(paths):
    files = []
    for path in paths:
        if os.path.isdir(path):
            files += sorted(glob.glob(os.path.join(path, '**', '*.py'), recursive=True))
        elif path.endswith('.py'):
            files.append(path)
    return files


def main():
    parser = argparse.ArgumentParser(description='what an Exudyn script written for an earlier '
                                     'version has to change; it parses the scripts and never runs them')
    parser.add_argument('paths', nargs='+', help='folders (searched recursively) or .py files')
    parser.add_argument('--check', action='store_true', help='exit 1 if anything was found')
    parser.add_argument('--fix', action='store_true', help='rewrite in place what can be rewritten - renamed settings, item parameters and arguments, missing '
                        'imports, removed and deprecated names and functions (#2938); what cannot is still reported')
    parser.add_argument('--run', action='store_true', help='also run each script, in a copy of its folder, without '
                        'windows, with a timeout (#2713); a script that names a path outside its folder is not run')
    parser.add_argument('--timeout', type=float, default=120, help='seconds a script may run with --run (default 120)')
    parser.add_argument('--solver-timeout', type=float, default=2, dest='solverTimeout',
                        help='seconds each solve may take with --run, exudyn.special.solver.timeout (default 2)')
    parser.add_argument('--base', default=os.getcwd(),
                        help='print the paths relative to this folder (default: the current one)')
    args = parser.parse_args()

    tables = {'functions': DeprecatedFunctions(), 'settings': DeprecatedSettings(),
              'submodules': SubmodulesNotLoaded(), 'itemParameters': DeprecatedItemParameters(),
              'library': DeprecatedLibrary()}

    (checked, skipped, withFindings, total, unreadable) = (0, 0, 0, 0, [])
    toRun = []           #(path, shown) of the scripts --run runs
    notRun = []          #(shown, line, path) of those it does not run
    for path in PythonFiles(args.paths):
        shown = os.path.relpath(path, args.base) if os.path.abspath(path).startswith(
            os.path.abspath(args.base)) else path
        shown = shown.replace(chr(92), '/')              #one spelling of a path in the report
        try:
            source = io.open(path, encoding='utf-8', errors='replace').read()
            with warnings.catch_warnings():
                warnings.simplefilter('ignore')          #an invalid escape is the script's business
                tree = ast.parse(source, filename=path)
        except SyntaxError as error:
            if 'exudyn' in source:
                unreadable.append((shown, error))
            else:
                skipped += 1
            continue
        if not ImportsExudyn(tree):
            skipped += 1
            continue
        checked += 1
        if args.fix:
            (source, tree, nFixed) = FixSource(source, tree, tables, path)
            if nFixed:
                original = io.open(path, encoding='utf-8', errors='replace', newline='').read()
                if not os.path.exists(path + '.bak'):          #the original kept once, beside the script (#2938)
                    shutil.copy2(path, path + '.bak')
                newline = '\r\n' if '\r\n' in original else '\n'
                io.open(path, 'w', encoding='utf-8', newline='').write(source.replace('\n', newline))
                print(shown + ': ' + str(nFixed) + ' fix(es) written; the original is ' + os.path.basename(path) + '.bak')
        if args.run:
            outside = PathsOutsideTheFolder(tree)
            if outside:
                notRun.append((shown,) + outside[0])
            else:
                toRun.append((path, shown))
        findings = CheckTree(tree, tables)
        if findings:
            withFindings += 1
            total += len(findings)
            for (line, text) in findings:
                print(shown + ':' + str(line) + ': ' + text)

    for (path, error) in unreadable:
        print(path + ':' + str(error.lineno) + ': does not parse as Python 3 (' + str(error.msg)
              + ') - not checked')
    print('checked ' + str(checked) + ' scripts that import exudyn: ' + str(total) + ' finding(s) in '
          + str(withFindings) + ' of them' + (', ' + str(len(unreadable)) + ' that do not parse'
                                              if unreadable else '')
          + '; ' + str(skipped) + ' other .py files skipped')
    failedRuns = 0
    if args.run:
        for (shown, line, path) in notRun:
            print(shown + ':' + str(line) + ': not run - the path ' + repr(path) + ' is outside the folder of the script')
        results = RunScripts(toRun, args.timeout, args.solverTimeout)
        for (shown, result, text) in results:
            if result != 'ran':
                print(shown + ': ' + {'failed': 'failed: ', 'timeout': 'timeout: ', 'needs': 'not run to its end - needs the package '}[result]
                      + text)
        counts = dict((kind, sum(1 for r in results if r[1] == kind)) for kind in ['ran', 'failed', 'timeout', 'needs'])
        failedRuns = counts['failed'] + counts['timeout']
        print('ran ' + str(len(results)) + ' scripts in a copy: ' + str(counts['ran']) + ' ran, ' + str(counts['failed'])
              + ' failed, ' + str(counts['timeout']) + ' timed out, ' + str(counts['needs']) + ' need a package; '
              + str(len(notRun)) + ' not run for a path outside their folder')
    failing = args.check and (total != 0 or unreadable or failedRuns)
    if failing:     #--check: say that the exit code is the verdict on the scripts, not an error of the tool (#2942)
        print('--check: the scripts need changes (' + str(total) + ' finding(s)' + (', ' + str(len(unreadable))
              + " that do not parse" if unreadable else '') + (', ' + str(failedRuns) + ' failed run(s)' if failedRuns
                                                               else '') + ') - exit code 1'
              + ('; --fix rewrites what can be rewritten' if total and not args.fix else ''))
    return 1 if failing else 0


if __name__ == '__main__':
    sys.exit(main())
