#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Reads the classes of definitions/ and hands each one to a generator as the pair
#           (parseInfo, parameterList) of plain strings that the line-based parser of the old
#           definition files produced. Three generators still want that form:
#
#             itemDocsEmitter.py     the item pages of the reference manual
#             structureDocsEmitter.py  the settings pages, through structureModel.py
#             structureModel.py      takes the LIST of definition modules from here, so every
#                                    structure emitter depends on this file for that alone
#
#           The conversion puts back what the current format derives: the flags that became
#           predicates are letters again, a Python value is rendered in its C++ spelling, the
#           shape of a type becomes 'size', and the output variables become the dict literal
#           the emitters eval(). It was proven lossless against the old parser field by field,
#           and `tools/regenerate.py --check` is what keeps it so.
#
#           It is NOT a leftover kept as a backup: without it three generators do not run
#           (#2653). It disappears when those two documentation emitters read the definitions
#           directly, which is work of its own and not a promise this file can make.
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

from itemModel import Mangle as _Mangle, LineType as _LineType, Size as _Size, \
    DefaultValueString as _DefaultValue, DefaultValueDocument as _DefaultValueDocument, \
    Flags as _Flags, OutputVariablesString as _OutputVariables, booleanHeaderKeys


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
    line = {key: _Mangle(values[key], key, source) for key in lineDefinition}
    #a user function is a real Python def in the definition file, and what is carried through is the
    #FUNCTION OBJECT, not a string: its arguments, their types and the docstring are read from its
    #source, and the documentation block is generated from them (#2664)
    #what a documentation table shows as the default value; the C++ literal in 'defaultValue' is
    #what the headers need, and the two used to be the same string put through a converter that
    #guessed (#2682)
    line['defaultValueDocument'] = _DefaultValueDocument(member)
    if member.get('userFunction') is not None:
        line['userFunction'] = member['userFunction']
        line['userFunctionExample'] = member.get('userFunctionExample') or ''
    return line


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
