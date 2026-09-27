#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# itemDocumentationReport - the state of the documentation of every item, measured (#2715)
#
# Every item - node, object, marker, load, sensor - is documented by what its definition in
# definitions/itemDefs*.py carries, because its reference manual page is generated from it. This
# reads the definitions and the scripts of the repository and says, per item:
#   - the words of the overall description ('overallDescription') and of the detailed description
#     ('detailedDescription'), and how many sections that one has;
#   - whether the page shows a figure of the item;
#   - how many of its parameters have no description, or one of fewer than three words;
#   - how many output variables it declares, and how many of those have no description;
#   - whether it has a MiniExample (the test suite runs every MiniExample);
#   - in how many examples and test models it is used - the same search as the links of its page -
#     and in how many modules of the package itself.
# It changes nothing. It is the table of revision2026b step RG13.1 and the way to ask again later.
#
# Usage:
#   python tools/itemDocumentationReport.py                 #the table, as Markdown, to stdout
#   python tools/itemDocumentationReport.py --summary       #the counts per kind of item only
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import argparse
import os
import re
import sys

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(root, 'tools', 'generators'))
sys.path.insert(0, os.path.join(root, 'definitions'))

import definitionLoader                                                     # noqa: E402
from autoGenerateHelper import ExampleKeywords                              # noqa: E402


def Words(text):
    """the words of a description, without its mathematics and markup"""
    text = re.sub(r'\$\$.*?\$\$', ' ', text or '', flags=re.S)
    text = re.sub(r'\$[^$]*\$', ' ', text)
    return len(re.findall(r'[A-Za-z]{2,}', text))


def ScriptTexts():
    """the text of every example and test model, read once"""
    texts = []
    for folder in ['Examples', 'TestModels']:
        directory = os.path.join(root, 'python', folder)
        for name in sorted(os.listdir(directory)):
            if name.endswith('.py'):
                with open(os.path.join(directory, name), encoding='utf-8', errors='replace') as file:
                    texts.append(file.read())
    return texts


def LibraryTexts():
    """the text of every module of the package that could create an item - not itemInterface.py,
    which defines every item class, and not the stubs"""
    texts = []
    package = os.path.join(root, 'python', 'exudyn')
    for (directory, _, names) in os.walk(package):
        for name in sorted(names):
            if name.endswith('.py') and name != 'itemInterface.py':
                with open(os.path.join(directory, name), encoding='utf-8', errors='replace') as file:
                    texts.append(file.read())
    return texts


def Kind(definition):
    classType = definition.get('classType', '')
    if classType == 'Object':
        return 'Object (' + (definition.get('objectType') or 'Object') + ')'
    return classType


def Measure(definition, scripts, library):
    members = definition['members']
    parameters = [m for m in members if m.get('kind') == 'ItemParameter'
                  and 'V' not in str(m.get('destination', ''))]      #the item's own, not its visualization
    visualization = [m for m in members if m.get('kind') == 'ItemParameter'
                     and 'V' in str(m.get('destination', ''))]
    #'name' has the same short description in every item, and it says all there is to say
    undescribed = [m['pythonName'] for m in parameters + visualization
                   if m['pythonName'] != 'name' and Words(m.get('description', '')) < 3]
    outputs = [entry.get('description', '') for entry in (definition.get('outputVariables') or [])
               if isinstance(entry, dict)]
    equations = definition.get('detailedDescription') or ''
    allText = (definition.get('overallDescription') or '') + equations
    className = definition['className']
    itemType = definition.get('classType', '')
    shortName = definition.get('pythonShortName', '')
    keywords = ExampleKeywords(itemType, itemType + className if not className.startswith(itemType)
                               else className, shortName)
    uses = sum(1 for text in scripts if any(keyword in text for keyword in keywords))
    #the package uses items too - the bearings are made of ObjectContactSphereTorus - and there an
    #item is created in every spelling, so the class name and a bracket is what is searched
    fullName = className if className.startswith(itemType) else itemType + className
    libraryUses = sum(1 for text in library if fullName + '(' in text)
    return {
        'item': className if className.startswith(itemType) else itemType + className,
        'kind': Kind(definition),
        'classWords': Words(definition.get('overallDescription')),
        'equationWords': Words(equations),
        'sections': len(re.findall(r'(?m)^\s*#{2,}\s', equations)),
        'figure': ('addExampleImage' in allText or '{figure}' in allText or '{image}' in allText
                   or '.png' in allText or '.jpg' in allText),
        'parameters': len(parameters) + len(visualization),
        'undescribed': undescribed,
        'outputs': len(outputs),
        'outputsUndescribed': sum(1 for text in outputs if Words(text) < 2),
        'miniExample': bool(definition.get('miniExample')),
        'uses': uses,
        'libraryUses': libraryUses,
        }


def main():
    parser = argparse.ArgumentParser(description='the state of the documentation of every item')
    parser.add_argument('--summary', action='store_true', help='the counts per kind of item only')
    args = parser.parse_args()

    scripts = ScriptTexts()
    library = LibraryTexts()
    rows = []
    for moduleName in definitionLoader.itemModules:
        for definition in __import__(moduleName).definitions:
            rows.append(Measure(definition, scripts, library))

    kinds = []
    for row in rows:
        if row['kind'] not in kinds:
            kinds.append(row['kind'])

    print('<!-- written by tools/itemDocumentationReport.py - run it again rather than editing this -->')
    print('')
    print('| kind | items | no detailed description | no figure | no MiniExample | parameters without a real '
          'description | output variables without one | used in no script | ... nor in the package |')
    print('|---|---|---|---|---|---|---|---|---|')
    for kind in kinds + ['**all**']:
        group = rows if kind == '**all**' else [row for row in rows if row['kind'] == kind]
        print('| ' + kind + ' | ' + str(len(group))
              + ' | ' + str(sum(1 for row in group if row['equationWords'] == 0))
              + ' | ' + str(sum(1 for row in group if not row['figure']))
              + ' | ' + str(sum(1 for row in group if not row['miniExample']))
              + ' | ' + str(sum(len(row['undescribed']) for row in group)) + ' of '
              + str(sum(row['parameters'] for row in group))
              + ' | ' + str(sum(row['outputsUndescribed'] for row in group)) + ' of '
              + str(sum(row['outputs'] for row in group))
              + ' | ' + str(sum(1 for row in group if row['uses'] == 0))
              + ' | ' + str(sum(1 for row in group if row['uses'] == 0 and row['libraryUses'] == 0)) + ' |')
    if args.summary:
        return 0

    print('')
    print('| item | kind | overall description (words) | detailed description (words, sections) | figure | '
          'parameters: without a real description | output variables (undescribed) | MiniExample | '
          'used in scripts | used in the package |')
    print('|---|---|---|---|---|---|---|---|---|---|')
    for row in rows:
        missing = row['undescribed']
        print('| ' + row['item'] + ' | ' + row['kind'] + ' | ' + str(row['classWords'])
              + ' | ' + (str(row['equationWords']) + ', ' + str(row['sections']) if row['equationWords'] else '-')
              + ' | ' + ('yes' if row['figure'] else '-')
              + ' | ' + str(len(missing)) + ' of ' + str(row['parameters'])
              + (': `' + '`, `'.join(missing) + '`' if missing and len(missing) <= 4 else '')
              + ' | ' + str(row['outputs']) + (' (' + str(row['outputsUndescribed']) + ')' if row['outputsUndescribed'] else '')
              + ' | ' + ('yes' if row['miniExample'] else '-')
              + ' | ' + str(row['uses']) + ' | ' + str(row['libraryUses']) + ' |')
    return 0


if __name__ == '__main__':
    sys.exit(main())
