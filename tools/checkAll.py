#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Keeps the __all__ lists of the Python modules in python/exudyn complete (revision plan
#           step 107c). For each module, the names declared in __all__ must equal its public names
#           (rule in tools/generators/publicApi.py: top-level functions, classes and assigned
#           names not starting with '_', except @docmeta(public=False)). A pure AST scan; nothing
#           is imported.
#
#           Not checked: __init__.py (the package), utilities.py (composes the lists of the modules
#           it imports) and robotics/__init__.py. itemInterface.py is generated with its __all__ by
#           itemInterfaceEmitter.py and is checked, never written.
#
# Usage:    python tools/checkAll.py            report
#           python tools/checkAll.py --check    the same, exit non-zero on a difference (CI)
#           python tools/checkAll.py --write    insert or update __all__ in the hand-written modules
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-15 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import ast
import glob
import io
import os
import sys

repositoryRoot = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(repositoryRoot, 'tools', 'generators'))
import publicApi                                                                 # noqa: E402

packageDirectory = os.path.join(repositoryRoot, 'python', 'exudyn')
notChecked = ['__init__.py', 'utilities.py', 'robotics/__init__.py']
generated = ['itemInterface.py']


def Modules():
    files = []
    for path in sorted(glob.glob(os.path.join(packageDirectory, '**', '*.py'), recursive=True)):
        relative = os.path.relpath(path, packageDirectory).replace('\\', '/')
        if relative not in notChecked:
            files.append(relative)
    return files


def InsertionLine(source):
    """0-based line before which a new __all__ goes: before the first top-level definition or
    assignment, above the comment lines directly preceding it"""
    lines = source.split('\n')
    tree = ast.parse(source)
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            continue
        if isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant):
            continue    #module docstring
        if isinstance(node, (ast.Try, ast.If)) and all(
                isinstance(n, (ast.Import, ast.ImportFrom, ast.Pass))
                for n in ast.walk(node) if isinstance(n, ast.stmt) and n is not node):
            continue    #an import block such as try: import x / except: pass
        line = min([d.lineno for d in getattr(node, 'decorator_list', [])] + [node.lineno]) - 1
        while line > 0 and lines[line - 1].lstrip().startswith('#'):
            line -= 1
        return line
    return len(lines)


def main():
    parser = argparse.ArgumentParser(description='keep __all__ of the exudyn modules complete')
    parser.add_argument('--check', action='store_true', help='exit non-zero on a difference')
    parser.add_argument('--write', action='store_true', help='insert or update __all__')
    args = parser.parse_args()

    failures = 0
    for relative in Modules():
        path = os.path.join(packageDirectory, relative)
        source = io.open(path, encoding='utf8', newline='').read()
        newline = '\r\n' if '\r\n' in source else '\n'
        text = source.replace('\r\n', '\n')
        public = publicApi.PublicNames(text)
        declared = publicApi.DeclaredAll(text)
        if declared is not None and declared[0] == public:
            continue
        if args.write and relative not in generated:
            lines = text.split('\n')
            allLines = publicApi.AllText(public).rstrip('\n').split('\n')
            if declared is None:
                at = InsertionLine(text)
                lines[at:at] = allLines + ['']
            else:
                first = declared[1] - 1
                if first > 0 and lines[first - 1] == publicApi.allComment:
                    first -= 1
                lines[first:declared[2]] = allLines
            io.open(path, 'w', encoding='utf8', newline='').write('\n'.join(lines).replace('\n', newline))
            print('written:', relative, len(public), 'names')
            continue
        failures += 1
        if declared is None:
            print(relative + ': no literal __all__ (' + str(len(public)) + ' public names)')
        else:
            missing = [n for n in public if n not in declared[0]]
            extra = [n for n in declared[0] if n not in public]
            print(relative + ': __all__ differs' + (' - missing ' + str(missing) if missing else '')
                  + (' - not defined ' + str(extra) if extra else '')
                  + ('' if missing or extra else ' - order differs'))
    if failures == 0:
        print('OK: __all__ of ' + str(len(Modules())) + ' modules matches their public names.')
    return 1 if (args.check and failures) else 0


if __name__ == '__main__':
    sys.exit(main())
