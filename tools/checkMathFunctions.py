#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# checkMathFunctions - the C++ code calls the math functions of Exudyn, EM::Sin, ..., not those of the library
#
# Why this check exists (#2953): the transcendental functions of the C++ library round differently on every platform,
# which moves the results of contact and friction models between Windows, Linux and macOS (#2947). The code calls
# EM::Sin, ... (src/Utilities/MathFunctions.h), which the switch EXUDYN_PORTABLE_MATH turns into portable functions
# with the same results everywhere (#2949). One call of std::sin - or of sin, which is the same - escapes the switch,
# and nothing else would notice.
#
# What is checked, in src/ except the graphics and the tests (which may use the library) and the two files that
# implement the functions:
#   - no call of sin, cos, tan, asin, acos, atan, atan2, exp, log, pow, sinh, cosh, tanh, cbrt, with or without std::
#     or :: - in code, not in comments or strings; a declaration such as 'AutoDiff<...> sin(' and a function of a
#     class such as SReal::sin are not;
#   - no 'using namespace EM;' in a header, where it would reach every file that includes it (CODING_STYLE.md §1).
#
# Usage:
#   python tools/checkMathFunctions.py            #report
#   python tools/checkMathFunctions.py --check    #exit 1 on a finding (the gate)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import argparse
import io
import os
import re
import sys

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
libraryFunctions = ['sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'exp', 'log', 'pow', 'sinh', 'cosh', 'tanh',
                    'cbrt']
exudynFunctions = {name: name[0].upper() + name[1:] for name in libraryFunctions}
skipDirectories = ['src/Graphics', 'src/Tests']
skipFiles = ['src/Utilities/MathFunctions.h', 'src/Utilities/PortableMath.cpp']
callPattern = re.compile(r'(?<![\w.>:])(std::|::)?(' + '|'.join(sorted(libraryFunctions, key=len, reverse=True))
                         + r')\s*\(')
declarationKeywords = {'return', 'else', 'case', 'throw', 'do'}


def CodeMask(text):
    """True for each character that is code: not in a comment, a string or a character literal"""
    mask = [True] * len(text)
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if text.startswith('//', i):
            j = text.find('\n', i)
            j = n if j < 0 else j
        elif text.startswith('/*', i):
            j = text.find('*/', i + 2)
            j = n if j < 0 else j + 2
        elif c == '"' or c == "'":
            j = i + 1
            while j < n and text[j] != c and text[j] != '\n':
                j += 2 if text[j] == '\\' else 1
            j += 1
        else:
            i += 1
            continue
        for k in range(i, min(j, n)):
            mask[k] = False
        i = j
    return mask


def Findings(text, fileName):
    """the calls of library functions and the using-directives a file must not have, as messages"""
    findings = []
    mask = CodeMask(text)
    for m in callPattern.finditer(text):
        if not mask[m.start()]:
            continue
        lineStart = text.rfind('\n', 0, m.start()) + 1
        before = text[lineStart:m.start()]
        if m.group(1) is None:
            stripped = before.rstrip()
            word = re.search(r'(\w+)$', stripped)
            if before != stripped and stripped and (stripped[-1] == '>' or (word and word.group(1) not in declarationKeywords)):
                continue    #a declaration of an overload, e.g. 'static SReal sin(' or 'AutoDiff<n, T> sin('
        line = text.count('\n', 0, m.start()) + 1
        findings.append(fileName + ':' + str(line) + ': ' + (m.group(1) or '') + m.group(2) + '(...) calls the library;'
                        ' use EM::' + exudynFunctions[m.group(2)] + ' (CODING_STYLE.md §1, #2953)')
    if fileName.endswith('.h'):
        for m in re.finditer(r'using\s+namespace\s+EM\s*;', text):
            if mask[m.start()]:
                line = text.count('\n', 0, m.start()) + 1
                findings.append(fileName + ':' + str(line) + ': "using namespace EM;" in a header reaches every file that'
                                ' includes it; write EM::Sin (CODING_STYLE.md §1)')
    return findings


def SourceFiles():
    for (directory, _, fileNames) in os.walk(os.path.join(root, 'src')):
        relativeDirectory = os.path.relpath(directory, root).replace(os.sep, '/')
        if any(relativeDirectory == d or relativeDirectory.startswith(d + '/') for d in skipDirectories):
            continue
        for fileName in sorted(fileNames):
            relative = relativeDirectory + '/' + fileName
            if fileName.endswith(('.h', '.cpp')) and relative not in skipFiles:
                yield relative


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='the C++ code calls EM::Sin, ..., not the library (#2953)')
    parser.add_argument('--check', action='store_true', help='exit 1 on a finding')
    args = parser.parse_args()

    findings = []
    for relative in SourceFiles():
        raw = io.open(os.path.join(root, relative), 'rb').read()
        try:
            text = raw.decode('utf-8')
        except UnicodeDecodeError:
            text = raw.decode('latin-1')
        findings += Findings(text, relative)
    for finding in findings:
        print(finding)
    print('checkMathFunctions: ' + (str(len(findings)) + ' finding(s)' if findings else 'OK'))
    sys.exit(1 if (findings and args.check) else 0)
