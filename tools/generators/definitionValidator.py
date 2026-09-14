#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Validates the definitions in definitions/ across files - the checks no single
#           constructor can make. Much of the schema already checks itself at import: an unknown
#           flag, type, parent class or OutputVariableType is a NameError, an impossible shape
#           raises in definitionTypes.py. What remains is cross-cutting, and is checked here:
#
#           1. every virtual item function overrides a declaration that really exists in the
#              hand-written C++ parent chain (src/System/C*.h, Main*.h, Visualization*.h), with
#              the same return type, argument types and constness - this is what keeps
#              definitions/itemFunctions.py a checked statement instead of a silent second copy
#              of the C++ headers; parameter NAMES may differ, as in C++ itself;
#           2. a fromParent parameter names a data member the parent chain declares;
#           3. no member is declared twice with the same signature (overloads are fine);
#           4. a fixed-size vector or matrix default has as many entries as its type;
#           5. a type written as a bare name is either a type constant of definitionTypes.py or
#              the name of a structure defined in definitions/ - a typo in a substructure type
#              would otherwise reach the generated C++ unnoticed.
#
#           ALL violations are reported, not just the first. Revision plan step 32.
#
# Usage:    python tools/generators/definitionValidator.py        (exit code 1 on violations)
#           called by tools/regenerate.py before the generators run
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-14 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import collections
import io
import os
import re
import sys

repositoryRoot = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                               '..', '..'))
definitionsDirectory = os.path.join(repositoryRoot, 'definitions')
toolsDirectory = os.path.dirname(os.path.abspath(__file__))
if toolsDirectory not in sys.path:
    sys.path.insert(0, toolsDirectory)
import typeModel as tm          # noqa: E402 - the C++ spelling of a definition type, as the header emitter renders it

itemModules = ['itemDefsNodes', 'itemDefsObjects', 'itemDefsMarkers', 'itemDefsLoads',
               'itemDefsSensors']
structureModules = ['structureDefsSimulationSettings', 'structureDefsVisualizationSettings',
                    'structureDefsSolverData', 'structureDefsSolvers', 'structureDefsOther']

#the hand-written parent classes of the generated item classes
parentHeaders = ['src/System/CNode.h', 'src/System/CObject.h', 'src/System/CObjectBody.h',
                 'src/System/CObjectConnector.h', 'src/System/CMarker.h', 'src/System/CLoad.h',
                 'src/System/CSensor.h', 'src/Objects/CObjectANCFCable2DBase.h',
                 'src/System/MainNode.h', 'src/System/MainObject.h', 'src/System/MainMarker.h',
                 'src/System/MainLoad.h', 'src/System/MainSensor.h',
                 'src/System/VisualizationNode.h', 'src/System/VisualizationObject.h',
                 'src/System/VisualizationMarker.h', 'src/System/VisualizationLoad.h',
                 'src/System/VisualizationSensor.h']



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#C++ parsing - deliberately small: declarations in a handful of hand-written base headers
def _StripComments(text):
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
    return re.sub(r'//[^\n]*', ' ', text)


def _SplitArguments(arguments):
    """split at top-level commas; template arguments and default-value calls stay whole"""
    parts, depth, current = [], 0, ''
    for character in arguments:
        if character in '<([':
            depth += 1
        elif character in '>)]':
            depth -= 1
        if character == ',' and depth == 0:
            parts.append(current)
            current = ''
        else:
            current += character
    if current.strip():
        parts.append(current)
    return parts


def _ArgumentType(argument):
    """the type of one argument: default value and parameter name removed, spaces dropped"""
    argument = argument.split('=')[0].strip()
    match = re.match(r'^(.*?[\s&*>])([A-Za-z_]\w*)$', argument)
    if match and match.group(1).strip() not in ('', 'const', 'unsigned'):
        argument = match.group(1)
    return re.sub(r'\s+', '', argument)


def Signature(returnType, arguments, isConst):
    return (re.sub(r'\s+', '', returnType),
            tuple(_ArgumentType(a) for a in _SplitArguments(arguments)), bool(isConst))


def _TopLevel(body):
    """the class body with nested braces (inline function bodies) removed"""
    result, depth = [], 0
    for character in body:
        if character == '{':
            depth += 1
        elif character == '}':
            depth -= 1
            result.append(';')
        elif depth == 0:
            result.append(character)
    return ''.join(result)


def ParseParentClasses():
    """class name -> (base class, {function name: [signatures]}, set of data member names)"""
    classes = {}
    for fileName in parentHeaders:
        text = _StripComments(io.open(os.path.join(repositoryRoot, fileName), encoding='utf8',
                                      errors='replace').read())
        for match in re.finditer(r'\bclass\s+(\w+)\s*(?::\s*public\s+(\w+))?\s*\{', text):
            index, depth = match.end(), 1
            while depth and index < len(text):
                depth += {'{': 1, '}': -1}.get(text[index], 0)
                index += 1
            body = _TopLevel(text[match.end():index - 1])
            functions = collections.defaultdict(list)
            for d in re.finditer(r'virtual\s+([\w:<>,\s\*&]+?)\s+(\w+)\s*'
                                 r'\(([^()]*(?:\([^()]*\)[^()]*)*)\)\s*(const)?', body):
                functions[d.group(2)].append(Signature(d.group(1), d.group(3), d.group(4)))
            members = set(re.findall(r'(?:^|[;:])\s*(?:mutable\s+)?[\w:<>,\*&\s]+?[\s\*&](\w+)\s*;',
                                     body, flags=re.M))
            classes[match.group(1)] = (match.group(2), functions, members)
    return classes


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def _Chain(classes, parent):
    seen = []
    while parent in classes and parent not in seen:
        seen.append(parent)
        parent = classes[parent][0]
    return seen


def _ParentFor(definition, destinationLetter):
    return definition.get({'C': 'cParentClass', 'M': 'mainParentClass',
                           'V': 'visuParentClass'}[destinationLetter], '')


def _DefaultEntries(value):
    """number of entries of a list-like default; None if it is not list-like"""
    if isinstance(value, (list, tuple)):
        return len(value)
    python = getattr(value, 'python', None)
    if isinstance(python, str) and python.strip().startswith('['):
        return len([p for p in python.strip()[1:-1].split(',') if p.strip()])
    return None


#the parameters of PyLatexRST.DefPyFunctionAccess, in order (tools/generators/autoGenerateHelper.py)
pybindFunctionParameters = ['cClass', 'pyName', 'cName', 'description', 'argList', 'defaultArgs', 'example',
                            'options', 'isLambdaFunction', 'argTypes', 'returnType', 'addDocu']
pybindBeginEnd = {'BeginCppWrittenByHand': 'EndCppWrittenByHand', 'BeginNoStub': 'EndNoStub'}


def ValidatePybindDeclarations(counts):
    """checks on definitions/pybind*.py: argument lists agree with defaults and types, no function
    is declared twice with the same arguments, Begin/End steering calls are balanced"""
    violations = []
    for fileName in sorted(os.listdir(definitionsDirectory)):
        if not (fileName.startswith('pybind') and fileName.endswith('.py')) or fileName == 'pybindTypes.py':
            continue
        module = __import__(fileName[:-3])
        for recorderName, recorder in vars(module).items():
            if type(recorder).__name__ != 'PybindInterface':
                continue
            seen = set()
            open_ = []
            for name, args, kwargs in recorder.calls:
                if name in pybindBeginEnd:
                    open_.append(name)
                elif name in pybindBeginEnd.values():
                    if not open_ or pybindBeginEnd[open_.pop()] != name:
                        violations.append(fileName + ': ' + name + ' without matching Begin')
                if name != 'DefPyFunctionAccess':
                    continue
                counts['pybind'] += 1
                call = dict(zip(pybindFunctionParameters, args))
                call.update(kwargs)
                where = fileName + ': ' + (call['cClass'] + '.' if call.get('cClass') else '') + call['pyName']
                argList = call.get('argList', [])
                for listName in ('defaultArgs', 'argTypes'):
                    other = call.get(listName, [])
                    if len(other) != 0 and len(other) != len(argList):
                        violations.append(where + ': ' + listName + ' has ' + str(len(other))
                                          + ' entries, argList has ' + str(len(argList)))
                key = (recorderName, call.get('cClass'), call['pyName'], tuple(argList), tuple(call.get('argTypes', [])))
                if key in seen:
                    violations.append(where + ': declared twice with the same arguments')
                seen.add(key)
            for name in open_:
                violations.append(fileName + ': ' + name + ' is not closed')
    return violations


def ValidateDefinitions(verbose=True):
    """Run every check over the loaded definitions; return the list of violations (strings)."""
    if definitionsDirectory not in sys.path:
        sys.path.insert(0, definitionsDirectory)

    classes = ParseParentClasses()
    import definitionTypes
    knownTypeNames = set(str(v) for k, v in vars(definitionTypes).items()
                         if k.startswith('T') and isinstance(v, definitionTypes.TypeSpec))
    for table in ('vectorSizes', 'matrixSizes', 'indexTupleSizes', 'nodeIndexTupleSizes'):
        knownTypeNames.update(getattr(definitionTypes, table).values())
    for base in ('TReal', 'Tfloat', 'TIndex', 'TArrayIndex'):
        knownTypeNames.update(getattr(definitionTypes, base).constrainedForms.values())
    for moduleName in structureModules:
        knownTypeNames.update(d['className'] for d in __import__(moduleName).definitions)
    violations = []
    counts = collections.Counter()

    for moduleName in itemModules + structureModules:
        isItem = moduleName in itemModules
        for definition in __import__(moduleName).definitions:
            className = definition['className']
            seen = {}
            for member in definition['members']:
                isFunction = 'Function' in member['kind']
                name = member['pythonName']
                where = moduleName + '.py: ' + className + '.' + name

                #---- 3. declared twice with the same signature
                key = (name, isFunction, str(member.get('destination', '')),
                       str(member.get('type', '')), member.get('args', '') or '',
                       member.get('cFlags', '') or '')
                if key in seen:
                    violations.append(where + ': declared twice with the same signature')
                seen[key] = True

                #---- 5. a bare type name must be known
                typeName = str(member.get('type', ''))
                if re.match(r'^[A-Za-z_]\w*$', typeName) and typeName not in knownTypeNames:
                    violations.append(where + ': type ' + repr(typeName) + ' is neither a type'
                                      + ' constant nor a structure defined in definitions/')

                #---- 4. shape versus default value
                size = getattr(member.get('type'), 'size', None)
                entries = _DefaultEntries(member.get('defaultValue'))
                if size and entries is not None:
                    counts['shape'] += 1
                    wanted = size if isinstance(size, int) else size[0] * size[1]
                    if entries != wanted:
                        violations.append(where + ': default value has ' + str(entries)
                                          + ' entries, type ' + str(member['type'])
                                          + ' has ' + str(wanted))

                if not isItem:
                    continue

                for letter in str(member.get('destination', '')):
                    if letter not in 'CMV':
                        continue
                    chain = _Chain(classes, _ParentFor(definition, letter))
                    parentName = _ParentFor(definition, letter)

                    #---- 1. virtual functions override a real parent declaration
                    if (isFunction and member.get('isVirtual', True)
                            and not member.get('isStatic', False)):
                        counts['override'] += 1
                        if not chain:
                            violations.append(where + ': parent class ' + repr(parentName)
                                              + ' is not in the parsed parent headers')
                            continue
                        own = Signature(tm.CppMemberType(member['type'], 'items'),
                                        member.get('args', '') or '',
                                        'C' in (member.get('cFlags', '') or ''))
                        declared = [s for c in chain for s in classes[c][1].get(name, [])]
                        if own not in declared:
                            violations.append(
                                where + ': isVirtual, but ' + ' -> '.join(chain)
                                + (' declares no virtual ' + name if not declared else
                                   ' declares it as ' + '; '.join(repr(s) for s in declared)
                                   + ', not ' + repr(own))
                                + ' - fix the declaration, or set isVirtual=False')

                    #---- 2. fromParent parameters exist in the parent
                    if not isFunction and member.get('fromParent', False):
                        counts['fromParent'] += 1
                        if not any(name in classes[c][2] for c in chain):
                            violations.append(where + ': fromParent, but ' + (' -> '.join(chain)
                                              or repr(parentName)) + ' has no member ' + name)

    violations += ValidatePybindDeclarations(counts)

    if verbose:
        print('definitionValidator: %d overrides, %d fromParent members, %d shaped defaults, '
              '%d pybind functions checked; %d violation(s)' % (counts['override'], counts['fromParent'],
                                            counts['shape'], counts['pybind'], len(violations)))
        for violation in violations:
            print('  ' + violation)

    return violations


if __name__ == '__main__':
    sys.exit(1 if ValidateDefinitions() else 0)
