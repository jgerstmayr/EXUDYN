#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN generator module
#
# Details:  Every deprecation of Exudyn in one list (#2807), read where it is declared:
#             settings          definitions/structureDefs*.py, deprecated=Deprecated(since, expires)
#             item parameters   definitions/itemDefs*.py, the same
#             functions of C++  definitions/pybind*.py, DefPyFunctionAccess(..., deprecated=Deprecated(...))
#             the library       python/exudyn, @Deprecated(since, expires, use) and
#                               DeprecatedArgument(name, since, expires, use) (exudyn.misc.deprecation)
#           Nothing is imported from python/exudyn: the library is parsed. Collect() gives the list,
#           Problems() what is inconsistent - a deprecation said in a text but not declared, a warning
#           in C++ for a function that is not declared deprecated. tools/checkDeprecations.py checks
#           the years, deprecationsEmitter.py writes the list into the developer documentation and
#           tools/checkUserScripts.py reports the library's deprecations in a user's script.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-03
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import ast
import glob
import importlib
import io
import os
import re
import sys

generatorDirectory = os.path.dirname(os.path.abspath(__file__))
repositoryRoot = os.path.normpath(os.path.join(generatorDirectory, '..', '..'))
for path in [generatorDirectory, os.path.join(repositoryRoot, 'definitions')]:
    if path not in sys.path:
        sys.path.insert(0, path)

#the name under which the functions of a pybind definition file are reported
pybindPrefixes = {'pybindModule': 'exudyn', 'pybindSystemContainer': 'SystemContainer', 'pybindRenderer': 'renderer',
                  'pybindMainSystem': 'MainSystem', 'pybindSystemData': 'systemData',
                  'pybindDataStructures': '', 'pybindGeneralContact': 'GeneralContact', 'pybindSymbolic': 'symbolic'}


def Entry(source, name, where, deprecated=None, since='', expires='', use=''):
    """one deprecation: source (settings, items, functions, library), its name, where it is declared, since,
    the year of removal and what to use instead"""
    if deprecated is not None:
        (since, expires) = (deprecated.since, deprecated.expires)
    return {'source': source, 'name': name, 'where': where, 'since': str(since), 'expires': expires, 'use': use}


def _Settings():
    import structureModel                                                   # noqa: PLC0415
    entries = []
    for definition in structureModel.StructureDefinitions():
        for member in definition['members']:
            if member.get('deprecated') is not None:
                entries.append(Entry('settings', definition['className'] + '.' + member['pythonName'],
                                     'definitions (' + definition['className'] + ')', member['deprecated'],
                                     use=str(member.get('description', '')).strip()))
    return entries


def _Items():
    import itemModel                                                        # noqa: PLC0415
    entries = []
    for definition in itemModel.ItemDefinitions():
        for member in definition['members']:
            deprecated = member.get('deprecated')
            if deprecated is not None and 'Function' not in member['kind']:
                use = deprecated.advice or str(member.get('description', '')).strip()
                entries.append(Entry('items', definition['className'] + '.' + member['pythonName'],
                                     'definitions (' + definition['className'] + ')', deprecated, use=use))
    return entries


def _PybindCalls():
    """[(module name, prefix, kwargs, args)] of every DefPyFunctionAccess of the pybind definitions"""
    calls = []
    for fileName in sorted(glob.glob(os.path.join(repositoryRoot, 'definitions', 'pybind*.py'))):
        moduleName = os.path.splitext(os.path.basename(fileName))[0]
        if moduleName == 'pybindTypes':
            continue
        module = importlib.import_module(moduleName)
        recorded = []
        for name in dir(module):
            value = getattr(module, name)
            if hasattr(value, 'calls') and isinstance(value.calls, list):
                recorded += value.calls
        prefix = pybindPrefixes.get(moduleName, '')
        className = ''
        for (call, args, kwargs) in recorded:
            if call == 'DefPyStartClass':
                className = args[1] if len(args) > 1 else kwargs.get('pyClass', '')
            if call == 'DefPyFunctionAccess':
                calls.append((moduleName, prefix or className, kwargs, args))
    return calls


def _PyName(kwargs, args):
    return kwargs.get('pyName', args[1] if len(args) > 1 else '')


def _Functions():
    entries = []
    for (moduleName, prefix, kwargs, args) in _PybindCalls():
        if kwargs.get('deprecated') is not None:
            entries.append(Entry('functions', (prefix + '.' if prefix else '') + _PyName(kwargs, args),
                                 'definitions/' + moduleName + '.py', kwargs['deprecated']))
    return entries


def _Literal(node):
    try:
        return ast.literal_eval(node)
    except ValueError:
        return None


def _LibraryFiles():
    return sorted(glob.glob(os.path.join(repositoryRoot, 'python', 'exudyn', '**', '*.py'), recursive=True))


def _ModuleName(fileName):
    relative = os.path.relpath(fileName, os.path.join(repositoryRoot, 'python', 'exudyn'))
    return os.path.splitext(relative)[0].replace(os.sep, '.')


def _Arguments(call):
    """(since, expires, use, function) of a call Deprecated(...) or DeprecatedArgument(name, ...)"""
    offset = 1 if getattr(call.func, 'id', getattr(call.func, 'attr', '')) == 'DeprecatedArgument' else 0
    values = [_Literal(argument) for argument in call.args]
    keywords = {keyword.arg: keyword.value for keyword in call.keywords}
    since = values[offset] if len(values) > offset else _Literal(keywords['since']) if 'since' in keywords else None
    expires = values[offset + 1] if len(values) > offset + 1 else _Literal(keywords['expires']) if 'expires' in keywords else None
    use = _Literal(keywords['use']) if 'use' in keywords else (values[offset + 2] if len(values) > offset + 2 else '')
    function = None
    if 'function' in keywords: #'*': named at runtime
        function = _Literal(keywords['function']) or '*'
    return (since, expires, use or '', function)


def LibraryDeclarations():
    """the deprecations of the library: [(entry, kind, function name, argument name or '', the name given as function=:
    a string, '*' if it is computed at runtime, or None)]"""
    found = []
    for fileName in _LibraryFiles():
        if fileName.endswith(os.path.join('misc', 'deprecation.py')):
            continue
        tree = ast.parse(io.open(fileName, encoding='utf-8').read(), filename=fileName)
        moduleName = _ModuleName(fileName)
        where = os.path.relpath(fileName, repositoryRoot).replace(os.sep, '/')
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            for decorator in node.decorator_list:
                if isinstance(decorator, ast.Call) and getattr(decorator.func, 'id', '') == 'Deprecated':
                    (since, expires, use, _) = _Arguments(decorator)
                    found.append((Entry('library', moduleName + '.' + node.name, where + ':' + str(node.lineno),
                                        since=since, expires=expires, use=use), 'function', node.name, '', None))
            for inner in ast.walk(node):
                if isinstance(inner, ast.Call) and getattr(inner.func, 'id', '') == 'DeprecatedArgument':
                    argument = _Literal(inner.args[0]) if inner.args else None
                    (since, expires, use, function) = _Arguments(inner)
                    owner = function if function not in (None, '*') else moduleName + '.' + node.name
                    found.append((Entry('library', owner + '.' + str(argument), where + ':' + str(inner.lineno),
                                        since=since, expires=expires, use=use), 'argument', node.name, argument, function))
    return found


def Collect():
    """all deprecations, as a list of Entry dictionaries, in the order settings, items, functions, library"""
    return _Settings() + _Items() + _Functions() + [entry for (entry, *rest) in LibraryDeclarations()]


def _SaysDeprecated(docstring):
    """a docstring that says the function is deprecated: it starts with DEPRECATED, or a line of it does"""
    if not docstring:
        return False
    return docstring.lstrip().startswith('DEPRECATED') or any(line.strip().startswith('DEPRECATED')
                                                              for line in docstring.split('\n'))


def Problems():
    """what is inconsistent, as a list of strings: each must be fixed, a build check fails for it"""
    problems = []
    #every entry has a version and a year
    for entry in Collect():
        if not entry['since'] or not isinstance(entry['expires'], int):
            problems.append(entry['where'] + ': ' + entry['name'] + ' has no version or no year of removal')
    #a function of the library whose docstring says DEPRECATED is declared, and a declared one says it
    declared = {}
    for (entry, kind, functionName, argument, function) in LibraryDeclarations():
        if kind == 'function':
            declared[(entry['where'].split(':')[0], functionName)] = True
    for fileName in _LibraryFiles():
        if fileName.endswith(os.path.join('misc', 'deprecation.py')):
            continue
        where = os.path.relpath(fileName, repositoryRoot).replace(os.sep, '/')
        tree = ast.parse(io.open(fileName, encoding='utf-8').read(), filename=fileName)
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and not node.name.startswith('_'):
                says = _SaysDeprecated(ast.get_docstring(node))
                isDeclared = (where, node.name) in declared
                if says and not isDeclared:
                    problems.append(where + ':' + str(node.lineno) + ': ' + node.name + ' says DEPRECATED in its '
                                    'docstring, but is not declared with @Deprecated(since, expires, use)')
                if isDeclared and not says:
                    problems.append(where + ':' + str(node.lineno) + ': ' + node.name + ' is declared deprecated, '
                                    'but its docstring does not say DEPRECATED')
    #a function of C++ whose description says DEPRECATED is declared
    pybindDeprecated = set()
    for (moduleName, prefix, kwargs, args) in _PybindCalls():
        description = str(kwargs.get('description', args[3] if len(args) > 3 else ''))
        if kwargs.get('deprecated') is not None:
            pybindDeprecated.add(_PyName(kwargs, args))
        elif description.lstrip().startswith(('DEPRECATED', 'DEPRECTED')):
            problems.append('definitions/' + moduleName + '.py: ' + _PyName(kwargs, args) + ' says DEPRECATED, '
                            'but has no deprecated=Deprecated(since, expires)')
    #a warning in C++ is for a declared function
    for fileName in sorted(glob.glob(os.path.join(repositoryRoot, 'src', '**', '*.*'), recursive=True)):
        if 'Autogenerated' in fileName or not fileName.endswith(('.h', '.cpp')):
            continue
        text = io.open(fileName, encoding='utf-8', errors='replace').read()
        names = re.findall(r'PyDeprecated\("functions", "([^"]+)"', text)
        names += re.findall(r'renderer\.DeprecationWarning\("([^"]+)"', text)
        for name in names:
            if name.endswith('.'): #the function that composes the name
                continue
            if name.split('.')[-1] not in pybindDeprecated:
                problems.append(os.path.relpath(fileName, repositoryRoot).replace(os.sep, '/') + ': warns that '
                                + name + ' is deprecated, but definitions/pybind*.py does not declare it')
    return problems
