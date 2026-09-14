#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Loads the definitions in definitions/ and hands them to the generators in the form
#           their old line parser produced: one (parseInfo, parameterList) pair per class, every
#           value a string, exactly as SplitString used to deliver it. The generators' per-class
#           code is therefore unchanged - only its input moved (revision plan step 33, part 1).
#
#           The conversion is the inverse of what step 31 folded into the new format: flags that
#           became derived (declaration-only, interface, substructure) are put back as letters,
#           values that became Python values are rendered back to their C++ spelling, the shape
#           a type now carries becomes 'size' again, and the output variables become the dict
#           literal the generators eval(). It was proven lossless against the old parser field
#           by field before the old definition files were removed; tools/regenerate.py --check
#           is the gate that keeps it so.
#
#           This adapter exists for as long as the generators consume the old representation.
#           Splitting them into emitters that read definitions/ directly (step 33, part 2)
#           removes it.
#
# Usage:    import definitionLoader
#           for parseInfo, parameterList in definitionLoader.LoadItemDefinitions(template): ...
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-14 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import copy
import os
import sys

repositoryRoot = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                               '..', '..'))
definitionsDirectory = os.path.join(repositoryRoot, 'definitions')
if definitionsDirectory not in sys.path:
    sys.path.insert(0, definitionsDirectory)

import definitionTypes          # noqa: E402

#ORDER MATTERS: classes are generated in this order, and structures appended to one file
#(appendToFile) depend on it. It is the order the old definition files had.
itemModules = ['itemDefsNodes', 'itemDefsObjects', 'itemDefsMarkers', 'itemDefsLoads',
               'itemDefsSensors']
structureModules = ['structureDefsOther', 'structureDefsSimulationSettings',
                    'structureDefsVisualizationSettings', 'structureDefsSolverData',
                    'structureDefsSolvers']

#header keys holding a real bool in definitions/, a 'True'/'False' string for the generators
booleanHeaderKeys = set(['writePybindIncludes', 'appendToFile', 'addDictionaryAccess',
                         'excludeFromTheDoc'])

#the old item parser turned every literal backslash-n into a newline - except inside the
#multi-line blocks; the structure parser did so only for three header keys. The definitions store
#the readable form, so the generators get the parser's form back.
_mangleRules = {'items':      {'all': True, 'keys': set(), 'verbatim': set(['equations',
                                                                           'miniExample'])},
                'structures': {'all': False, 'verbatim': set(),
                               'keys': set(['classDescription', 'latexText', 'cppText'])}}

BACKSLASH, NEWLINE = chr(92), chr(10)


def _Mangle(text, key, source):
    rule = _mangleRules[source]
    if key in rule['verbatim'] or (not rule['all'] and key not in rule['keys']):
        return text
    return text.replace(BACKSLASH + 'n', NEWLINE)


def _LineType(member):
    text = 'F' if 'Function' in member['kind'] else 'V'
    for field, letter in (('isVirtual', 'v'), ('isStatic', 's'), ('fromParent', 'p'),
                          ('isLinked', 'L')):
        #isVirtual defaults to True in the constructors but only means something for functions
        if member.get(field, False) and (field != 'isVirtual' or text == 'F'):
            text += letter
    return text


def _Size(member):
    """the old 'size' column, derived from the shape the type carries"""
    if 'Function' in member['kind']:
        return ''
    typeSpec = member.get('type', '')
    size = getattr(typeSpec, 'size', None)
    if size is not None:
        if isinstance(size, tuple):          #the old format spelled a matrix as its flat count
            product = 1
            for value in size:
                product *= value
            return str(product)
        return str(size)
    fixed = {'Float3': '3', 'Float4': '4', 'StdArray33F': '3x3', 'Matrix6D': '36'}
    fixed.update({name: str(n) for n, name in definitionTypes.vectorSizes.items()})
    fixed.update({name: str(r * c) for (r, c), name in definitionTypes.matrixSizes.items()})
    fixed.update({name: str(n) for n, name in definitionTypes.indexTupleSizes.items()})
    fixed.update({name: str(n) for n, name in definitionTypes.nodeIndexTupleSizes.items()})
    if str(typeSpec) in fixed:
        return fixed[str(typeSpec)]
    return member.get('size', '') or ''


def _DefaultValue(member):
    if 'Function' in member['kind']:
        implementation = member.get('implementation', None)
        return '' if implementation is None else implementation
    deprecated = member.get('deprecated', None)
    if deprecated is not None:
        return deprecated.ToCpp()
    value = member.get('defaultValue', '')
    if value is definitionTypes.NoDefaultValue or value is None or value == '':
        return ''
    return definitionTypes.CppLiteral(value, str(member.get('type', '')))


def _Flags(member, source, structureClassNames):
    flags = member.get('cFlags', '') or ''
    isFunction = 'Function' in member['kind']
    if isFunction and member.get('implementation', None) is None:
        flags += 'D'
    if source == 'items':
        if isFunction:
            flags += 'I'        #inert on functions; restored for byte identity of the old input
        elif 'n' in flags:
            flags = flags.replace('n', '')
        else:
            flags += 'I'
    elif str(member.get('type', '')) in structureClassNames:
        flags = flags.replace('P', 'PS', 1) if 'P' in flags else 'S' + flags
    return flags


def _OutputVariables(entries):
    """the dict literal both generators eval() after doubling the backslashes"""
    if not entries:
        return ''
    parts = []
    for entry in entries:
        description = entry['description']
        quote = chr(34) if "'" in description else "'"
        parts.append("'" + entry['outputVariable'].name + "':" + quote + description + quote)
    return '{' + ', '.join(parts) + '}'


def _Member(member, lineDefinition, source, structureClassNames):
    values = {'lineType': _LineType(member),
              'destination': str(member.get('destination', '')),
              'pythonName': member['pythonName'],
              'cplusplusName': member.get('cplusplusName', '') or member['pythonName'],
              'size': _Size(member),
              'type': str(member.get('type', '')),
              'defaultValue': _DefaultValue(member),
              'args': member.get('args', '') or '',
              'cFlags': _Flags(member, source, structureClassNames),
              'parameterDescription': member.get('description', '') or ''}
    return {key: _Mangle(values[key], key, source) for key in lineDefinition}


def _Load(modules, parseInfoTemplate, lineDefinition, source):
    definitions = []
    for moduleName in modules:
        definitions += __import__(moduleName).definitions
    structureClassNames = set(d['className'] for d in definitions) if source == 'structures' \
        else set()

    for definition in definitions:
        parseInfo = copy.deepcopy(parseInfoTemplate)
        parseInfo['class'] = definition['className']
        for key, value in definition.items():
            if key in ('className', 'members'):
                continue
            if key not in parseInfo:
                raise ValueError(definition['className'] + ': header key ' + repr(key)
                                 + ' is unknown to the ' + source + ' generator')
            if key in booleanHeaderKeys:
                value = 'True' if value else 'False'
            elif key == 'outputVariables':
                value = _OutputVariables(value)
            else:
                value = _Mangle(str(value), key, source)
            parseInfo[key] = value
        parameterList = [_Member(m, lineDefinition, source, structureClassNames)
                         for m in definition['members']]
        yield parseInfo, parameterList


def LoadItemDefinitions(parseInfoTemplate, lineDefinition):
    """(parseInfo, parameterList) for every item, in generation order"""
    return _Load(itemModules, parseInfoTemplate, lineDefinition, 'items')


def LoadStructureDefinitions(parseInfoTemplate, lineDefinition):
    """(parseInfo, parameterList) for every structure, in generation order"""
    return _Load(structureModules, parseInfoTemplate, lineDefinition, 'structures')
