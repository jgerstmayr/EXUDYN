#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# itemDrawingReport - how every item is drawn, read from the C++ (#2840)
#
# The drawing of an item is its UpdateGraphics function in src/ (Visualization<Item>::UpdateGraphics) and
# the helpers it calls (src/Graphics/VisualizationItemHelpers.h, the EXUvis functions of
# VisualizationPrimitives.cpp). This reads them and says, per item:
#   - the visualization settings they read (visualizationSettings.<structure>.<member>, also through a
#     local reference to a structure), with those of the helpers;
#   - the visualization parameters of its definition (V...) they read - and those they do not read;
#   - the primitives they draw (EXUvis::Draw..., graphicsData.Add...);
# and lists the items that have no UpdateGraphics. It changes nothing; it is the inventory of
# revision2026b step RG13.8.1, and what a section "Drawing" of the item pages (RG13.8.2) can be checked
# against. What a setting MEANS is not here - that needs a reader, see itemDrawingInventory.md.
#
# Usage:
#   python tools/itemDrawingReport.py           #the table, as Markdown, to stdout
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import glob
import os
import re
import sys

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(root, 'tools', 'generators'))
sys.path.insert(0, os.path.join(root, 'definitions'))

import definitionLoader                                                     # noqa: E402


def Read(path):
    with open(path, encoding='utf-8', errors='replace') as file:
        return file.read()


def Body(text, start):
    """the text between the first '{' after start and the brace that closes it"""
    begin = text.index('{', start)
    depth = 0
    for position in range(begin, len(text)):
        if text[position] == '{':
            depth += 1
        elif text[position] == '}':
            depth -= 1
            if depth == 0:
                return text[begin:position + 1]
    return text[begin:]


def WithoutComments(text):
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
    return re.sub(r'//[^\n]*', ' ', text)


def Settings(body):
    """visualizationSettings.a.b(.c) read in a body, also through 'X& name = visualizationSettings.a;'"""
    found = set(re.findall(r'visualizationSettings\.(\w+\.\w+(?:\.\w+)?)', body))
    for (alias, structure) in re.findall(r'(\w+)\s*=\s*visualizationSettings\.(\w+)\s*;', body):
        found.update(structure + '.' + member for member in re.findall(r'\b' + alias + r'\.(\w+)', body))
    #a structure taken as a whole and passed on is not a setting read
    return {name for name in found if not name.endswith(('.size', '.NumberOfItems'))}


def Primitives(body):
    names = set(re.findall(r'EXUvis::(Draw\w+|AddBodyGraphicsData\w*)', body))
    names.update('Add' + name for name in re.findall(r'graphicsData\.Add(\w+)', body))
    return names


def Helpers():
    """the functions of the drawing helpers, by name: their body (all overloads together)"""
    helpers = {}
    for path in [os.path.join(root, 'src', 'Graphics', 'VisualizationItemHelpers.h'),
                 os.path.join(root, 'src', 'Graphics', 'VisualizationPrimitives.cpp')]:
        text = WithoutComments(Read(path))
        for match in re.finditer(r'\n[ \t]*(?:inline\s+|static\s+)?(?:void|bool|Index|Real|float)\s+(\w+)\s*\([^;{]*\)\s*(?:const\s*)?\{', text):
            helpers[match.group(1)] = helpers.get(match.group(1), '') + Body(text, match.end() - 1)
    return helpers


def Merge(functions, item, function):
    """UpdateGraphics and CallUserFunction of an item, or its own and the one of its base class, together"""
    if item in functions:
        (path, body, full, called) = functions[item]
        functions[item] = (path + ', ' + function[0] if function[0] not in path else path, body + function[1],
                           full + function[2], sorted(set(called) | set(function[3])))
    else:
        functions[item] = function


def DrawingFunctions():
    """Visualization<Item>::UpdateGraphics by item name: (file, body with the bodies of the helpers it calls)"""
    helpers = Helpers()
    functions = {}
    ownUpdate = set()                                                   #the items with an UpdateGraphics of their own
    for path in sorted(glob.glob(os.path.join(root, 'src', '**', '*.cpp'), recursive=True)):
        text = WithoutComments(Read(path))
        for match in re.finditer(r'void\s+Visualization(\w+)::(UpdateGraphics|CallUserFunction)\s*\(', text):
            body = Body(text, match.end())
            called = set()
            pending = [body]
            while pending:                                              #helpers that call helpers
                for name in re.findall(r'\b(\w+)\s*(?:<[^<>()]*>)?\s*\(', pending.pop()):
                    if name in helpers and name not in called:
                        called.add(name)
                        pending.append(helpers[name])
            full = body + ''.join(helpers[name] for name in sorted(called))
            item = match.group(1)
            if match.group(2) == 'UpdateGraphics':
                ownUpdate.add(item)
            Merge(functions, item, (os.path.relpath(path, root).replace(os.sep, '/'), body, full, sorted(called)))
    #an item without an UpdateGraphics of its own draws with the one of the class its visualization derives from
    for path in glob.glob(os.path.join(root, 'src', 'Autogenerated', '**', 'Visu*.h'), recursive=True):
        for (item, base) in re.findall(r'class\s+Visualization(\w+)\s*:\s*public\s+Visualization(\w+)', Read(path)):
            if item not in ownUpdate and base in functions:
                (basePath, baseBody, baseFull, baseCalled) = functions[base]
                Merge(functions, item, (basePath, baseBody, baseFull, baseCalled + ['as ' + base]))
    return functions


def main():
    functions = DrawingFunctions()
    rows = []
    for moduleName in definitionLoader.itemModules:
        for definition in __import__(moduleName).definitions:
            className = definition['className']
            itemType = definition.get('classType', '')
            item = className if className.startswith(itemType) else itemType + className
            visualization = [m['pythonName'] for m in definition['members'] if m.get('kind') == 'ItemParameter'
                             and 'V' in str(m.get('destination', '')) and 'n' not in str(m.get('cFlags', ''))]
            rows.append((item, visualization, functions.pop(item, None)))

    print('<!-- written by tools/itemDrawingReport.py - run it again rather than editing this -->')
    print('')
    print('| item | settings read | visualization parameters read | not read | draws with | file, helpers |')
    print('|---|---|---|---|---|---|')
    for (item, visualization, function) in rows:
        if function is None:
            print('| ' + item + ' | *no UpdateGraphics* | | ' + ', '.join(visualization) + ' | | |')
            continue
        (path, body, full, helpers) = function
        read = [name for name in visualization if re.search(r'\b' + name + r'\b', full)
                or re.search(r'\bGet' + name[0].upper() + name[1:] + r'\s*\(', full)]
        notRead = [name for name in visualization if name not in read and name != 'show']
        print('| ' + item + ' | ' + ', '.join('`' + s + '`' for s in sorted(Settings(full)))
              + ' | ' + ', '.join(read) + ' | ' + ', '.join(notRead)
              + ' | ' + ', '.join(sorted(Primitives(full)))
              + ' | ' + path.replace('src/', '') + (' (' + ', '.join(helpers) + ')' if helpers else '') + ' |')
    if functions:
        print('')
        print('Drawing functions without an item of their own (base classes): ' + ', '.join(sorted(functions)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
