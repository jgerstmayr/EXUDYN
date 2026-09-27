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
#   - a function, method or setting that is DEPRECATED, with what to use instead - read from
#     definitions/, so that the list is the one the documentation is generated from;
#   - a setting or a function that is REMOVED;
#   - a submodule used through 'exu.<submodule>' that 'import exudyn' does not load.
# It checks every .py file under the folders it is given that imports exudyn; the others are counted
# and skipped. A file that does not parse - Python 2, say - is reported as such.
#
# The tables of what was removed are history and do not change; what is deprecated is read from
# definitions/ each time. Where they come from is said at each table.
#
# Usage:
#   python tools/checkUserScripts.py <folder or file> [...]          #report
#   python tools/checkUserScripts.py <folder> --check                 #exit 1 if anything was found
#   exudev scripts <folder> [...]                                     #the same, through the driver
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import argparse
import ast
import builtins
import glob
import io
import os
import sys
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
    ('renderer', 'Detach'):                  'nothing: SC.renderer.Stop() detaches the container',
    }

#ARGUMENTS THAT ARE GONE
removedKeywords = {
    'rBoundingSphere': 'ObjectContactConvexRoll computes it from coefficientsHull; leave it out',
    }


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#what definitions/ says is deprecated
def DeprecatedFunctions():
    """{('module'|'SystemContainer'|'renderer', name): description} from the pybind definitions"""
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
    for definition in definitions:
        owner = memberOf.get(definition['className'])
        if owner is None:
            continue
        parentMember = memberOf.get(parentOf.get(definition['className']))
        for member in definition['members']:
            deprecated = member.get('deprecated')
            if deprecated:
                found[(owner, member['pythonName'])] = (member['description'].strip(),
                                                        getattr(deprecated, 'version', ''),
                                                        parentMember)
    DeprecatedSettings.structureMembers = set(memberOf.values())
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


def CheckTree(tree, tables):
    """the findings of one parsed script: [(line, text)]"""
    findings = []
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
            elif node.id in removedNames:
                Report(node.lineno, ('removed', node.id), "'" + node.id + "' is removed; use "
                       + removedNames[node.id])

        elif isinstance(node, ast.Attribute):
            chain = AttributeChain(node)
            if chain is None:
                continue
            if chain[-1] in removedNames and len(chain) == 2 and chain[0] in exudynAliases | {'eii'}:
                Report(node.lineno, ('removed', chain[-1]), "'" + '.'.join(chain) + "' is removed; use "
                       + removedNames[chain[-1]])
            if len(chain) >= 3:
                pair = (chain[-2], chain[-1])
                if pair in tables['settings']:
                    (use, version, parentMember) = tables['settings'][pair]
                    above = chain[-3]
                    if (parentMember is not None and above != parentMember) or                             (parentMember is None and (above in DeprecatedSettings.structureMembers
                                                       or above.startswith('view'))):
                        continue                          #the same pair in another structure
                    Report(node.lineno, ('setting',) + pair, "'" + '.'.join(pair) + "' is deprecated"
                           + (' since ' + version if version else '') + '; use ' + use)
                if pair in removedSettings:
                    Report(node.lineno, ('removedSetting',) + pair, "'" + '.'.join(pair)
                           + "' is removed; use " + removedSettings[pair])
            if len(chain) == 2 and chain[0] in exudynAliases:
                if ('module', chain[1]) in tables['functions']:
                    Report(node.lineno, ('function', chain[1]), "'" + '.'.join(chain)
                           + "' is deprecated: " + tables['functions'][('module', chain[1])])
            if len(chain) >= 2 and chain[0] in exudynAliases and chain[1] in tables['submodules'] \
                    and chain[1] not in importedSubmodules:
                Report(node.lineno, ('submodule', chain[1]), "'" + chain[0] + '.' + chain[1]
                       + "' is used, but 'import exudyn' does not load it; add: import exudyn."
                       + chain[1])
            if len(chain) >= 2 and chain[0] not in exudynAliases:
                receiver = 'renderer' if chain[-2] == 'renderer' else 'SystemContainer'
                if (receiver, chain[-1]) in tables['functions']:
                    Report(node.lineno, ('method', receiver, chain[-1]), "'" + '.'.join(chain[-2:])
                           + "' is deprecated: " + tables['functions'][(receiver, chain[-1])])

        elif isinstance(node, ast.keyword) and node.arg in removedKeywords:
            Report(node.value.lineno, ('keyword', node.arg), "argument '" + node.arg
                   + "' is removed: " + removedKeywords[node.arg])

    return sorted(findings)


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
    parser.add_argument('--base', default=os.getcwd(),
                        help='print the paths relative to this folder (default: the current one)')
    args = parser.parse_args()

    tables = {'functions': DeprecatedFunctions(), 'settings': DeprecatedSettings(),
              'submodules': SubmodulesNotLoaded()}

    (checked, skipped, withFindings, total, unreadable) = (0, 0, 0, 0, [])
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
    return 1 if args.check and (total != 0 or unreadable) else 0


if __name__ == '__main__':
    sys.exit(main())
