#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Emits python/exudyn/types/items.py - the type information of every item as Python data
#: its kind, type bits, requested node and marker types, access
#           function types and output variables, and per parameter the type, size, range, default,
#           must-be-given flag and description. Read from definitions/, so there is no second copy;
#           the query functions in python/exudyn/types/__init__.py work on this data.
#
# Usage:    python tools/generators/typesEmitter.py [--output FILE]
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-15 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import io
import os
import re
import sys

toolsDirectory = os.path.dirname(os.path.abspath(__file__))
if toolsDirectory not in sys.path:
    sys.path.insert(0, toolsDirectory)

import itemModel as im                                                              # noqa: E402
import typeModel as tm                                                              # noqa: E402
from itemModel import ExtractMathSymbol                                            # noqa: E402
from autoGenerateHelper import CleanStringForPyiDescription                        # noqa: E402

#the C++ enum whose values GetType returns, per item kind
typeEnums = {'Node': 'Node', 'Marker': 'Marker', 'Object': 'CObjectType', 'Load': 'LoadType',
             'Sensor': 'SensorType'}


def Member(definition, pythonName):
    for member in definition['members']:
        if member.get('pythonName') == pythonName and im.IsFunction(member):
            return member
    return None


def TypeNames(definition):
    """the type bits of an item: declared for nodes and markers; parsed from the C++ body
    for objects, loads and sensors, whose enums are hand-written"""
    member = Member(definition, 'GetType')
    if member is not None and 'itemTypes' in member:
        return list(member['itemTypes'])
    if member is not None:
        body = member.get('implementation') or ''
    else: #inherited from a hand-written parent class, e.g. CObjectANCFCable2DBase
        parentHeader = os.path.join(im.repositoryRoot, 'src', 'ImplObjects',
                                    str(definition.get('cParentClass', '')) + '.h')
        if not os.path.isfile(parentHeader):
            #a definition WITHOUT a parent class has no header to read, which is normal; a
            #definition WITH one whose header is not found is a mistake that used to pass as an
            #empty type list (revision2026 step R11.4.5 renamed the folder and this returned [])
            if definition.get('cParentClass', ''):
                print('   WARNING: ' + definition['className'] + ' names the parent class '
                      + str(definition['cParentClass']) + ', whose header is not at '
                      + os.path.relpath(parentHeader, im.repositoryRoot)
                      + ' - its item types cannot be read')
            return []
        text = io.open(parentHeader, encoding='utf8', errors='replace').read()
        match = re.search(r'GetType\(\) const override\s*\{(.*?)\}', text, re.S)
        body = match.group(1) if match else ''
    body = '\n'.join(line.split('//')[0] for line in body.split('\n'))
    return [name for name in re.findall(typeEnums[definition['classType']] + r'::(\w+)', body) if name != '_None']


def Parameter(member):
    [description, mathSymbol] = ExtractMathSymbol(im.Description(member))
    typeName = im.TypeName(member)
    return {'type': typeName,
            'size': im.Size(member),
            'range': tm.ConstraintNote(typeName).replace('must be ', '').replace('; ', ''),
            'default': im.DefaultValuePython(member),
            'mustBeGiven': 'Q' in (member.get('cFlags', '') or ''),
            'description': CleanStringForPyiDescription(description).strip()}


def ItemData(definition):
    data = {'kind': definition['classType'], 'types': TypeNames(definition)}
    for functionName, key in [('GetRequestedNodeType', 'requestedNodeTypes'),
                              ('GetRequestedMarkerType', 'requestedMarkerTypes')]:
        member = Member(definition, functionName)
        if member is not None and 'requestedTypes' in member:
            data[key] = list(member['requestedTypes'])
            if member['conditionalTypes']:
                data['conditional' + key[0].upper() + key[1:]] = [list(c) for c in member['conditionalTypes']]
    member = Member(definition, 'GetAccessFunctionTypes')
    if member is not None and 'accessFunctionTypes' in member:
        data['accessFunctionTypes'] = list(member['accessFunctionTypes'])
    data['outputVariables'] = [entry['outputVariable'].name for entry in (definition.get('outputVariables') or [])]
    parameters, visualization = {}, {}
    for member in definition['members']:
        if im.IsInterfaceParameter(member) and not im.IsReadOnly(member):
            target = visualization if 'V' in im.Destination(member) else parameters
            target[member['pythonName']] = Parameter(member)
    data['parameters'] = parameters
    data['visualization'] = visualization
    return data


def ModuleText():
    items = {}
    for definition in im.ItemDefinitions():
        if im.HasPythonInterface(definition):
            items[definition['className']] = ItemData(definition)
    s = ('#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n'
         '# AUTO GENERATED FILE - DO NOT EDIT\n'
         '#\n'
         '# Details:  Type information of the Exudyn items, generated from definitions/ by\n'
         '#           tools/generators/typesEmitter.py. Type names are the\n'
         '#           values of exudyn.NodeType, exudyn.MarkerType, exudyn.AccessFunctionType and of the\n'
         '#           C++ enums CObjectType, LoadType, SensorType; defaults are Python source text.\n'
         '#           Use the query functions of exudyn.types instead of reading this dict directly.\n'
         '#\n'
         '# Copyright:This file is part of Exudyn. Exudyn is free software: see \'LICENSE.txt\'\n'
         '#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n'
         '\n'
         "__all__ = ['items']\n"
         '\n'
         '#item class name -> type information\n')
    #one line per item fact and per parameter: pprint wraps the long descriptions into many lines
    s += 'items = {\n'
    for className, data in items.items():
        s += '  ' + repr(className) + ': {\n'
        for key, value in data.items():
            if key in ('parameters', 'visualization'):
                s += '    ' + repr(key) + ': {\n'
                for name, parameter in value.items():
                    s += '      ' + repr(name) + ': ' + repr(parameter) + ',\n'
                s += '    },\n'
            else:
                s += '    ' + repr(key) + ': ' + repr(value) + ',\n'
        s += '  },\n'
    s += '}\n'
    return s


def main(argv=None):
    parser = argparse.ArgumentParser(description='Emit python/exudyn/types/items.py from definitions/.')
    parser.add_argument('--output', default=os.path.join(im.repositoryRoot, 'python', 'exudyn', 'types', 'items.py'))
    args = parser.parse_args(argv)
    text = ModuleText()
    existing = io.open(args.output, encoding='utf8').read() if os.path.isfile(args.output) else ''
    if existing != text:
        os.makedirs(os.path.dirname(args.output), exist_ok=True)
        io.open(args.output, 'w', encoding='utf8', newline='\n').write(text)
        print('types/items.py written')
    else:
        print('types/items.py unchanged')
    return 0


if __name__ == '__main__':
    sys.exit(main())
