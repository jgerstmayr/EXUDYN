#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# checkDefinitions - the descriptions in definitions/ obey the rules of definitions/README.md
#
# Why this check exists (#2655): a description in definitions/ is the source of a page of the
# reference manual. It goes through one converter, tools/generators/latexToMarkdown.py, and what
# the converter does not recognise is carried through to the page as itself - no warning, no
# failure, and the page builds. This is the check that names the file and the line instead.
#
# What it checks today: every ABRV:KEY names an abbreviation that the list actually has. The
# abbreviations are the dict in tools/generators/examplesDocsEmitter.py, which also writes
# docs/generated/abbreviations.md, so the key and its target cannot drift apart.
#
# Usage:
#   python tools/checkDefinitions.py            #report
#   python tools/checkDefinitions.py --check    #exit 1 on a finding (the gate)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import argparse
import ast
import glob
import io
import os
import re
import sys


def DeclaredAbbreviations(path):
    """the keys of the abbreviations dict, read as a literal - importing the emitter would pull in
    the whole generator package for one dict"""
    tree = ast.parse(io.open(path, encoding='utf-8').read())
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and getattr(node.targets[0], 'id', '') == 'abbreviations'):
            return set(key.value for key in node.value.keys)
    raise ValueError(path + ': no "abbreviations" dict found')


def Descriptions(path):
    """every string literal of a definition file, with the line it starts on"""
    for node in ast.walk(ast.parse(io.open(path, encoding='utf-8').read())):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            yield (node.value, node.lineno)


def CheckAbbreviations(paths, declared):
    """ABRV:KEY with a key the list does not have"""
    findings = []
    for path in paths:
        for (text, lineno) in Descriptions(path):
            for match in re.finditer(r'\bABRV:([A-Za-z0-9]+)', text):
                if match.group(1) in declared:
                    continue
                line = lineno + text.count('\n', 0, match.start())
                findings.append((path, line, 'ABRV:' + match.group(1)
                                 + ' - no such abbreviation'))
    return findings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='exit 1 on a finding')
    parser.add_argument('--quiet', action='store_true', help='print only findings')
    args = parser.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    paths = sorted(glob.glob(os.path.join(root, 'definitions', '*.py')))
    declared = DeclaredAbbreviations(os.path.join(root, 'tools', 'generators',
                                                  'examplesDocsEmitter.py'))
    findings = CheckAbbreviations(paths, declared)

    if len(findings) == 0:
        if not args.quiet:
            print('OK: the descriptions of ' + str(len(paths)) + ' definition files use only the '
                  + str(len(declared)) + ' abbreviations that the list has.')
        return 0

    print('FINDINGS in definitions/ - see definitions/README.md, "Writing a description":')
    for (path, line, what) in findings:
        print('   ' + os.path.relpath(path, root).replace(os.sep, '/') + ':' + str(line)
              + '  ' + what)
    return 1 if args.check else 0


if __name__ == '__main__':
    sys.exit(main())
