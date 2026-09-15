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
#           Since step 34a only the two documentation emitters (itemDocsEmitter.py,
#           structureDocsEmitter.py) consume the old representation; this adapter is deleted with
#           them in step 50.
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
    DefaultValueString as _DefaultValue, Flags as _Flags, OutputVariablesString as _OutputVariables, booleanHeaderKeys


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
