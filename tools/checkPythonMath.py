#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# checkPythonMath - the Python library and the test models take their math functions from exudyn.cMath
#
# Why this check exists (#2964): the transcendental functions of math and numpy come from the library of each platform
# and round some results differently on Windows, Linux and macOS; through an initial value or a point of a geometry
# that moves the results of a model (#2956). exudyn.cMath has the functions of the C++ code, the same bits everywhere.
# Python's x**0.5 and x**y call the library's pow as well (on 400k arguments, x**0.5 differs from sqrt in 234 on
# Windows and 357 on Linux); sqrt is correctly rounded everywhere.
#
# What is checked, in python/exudyn and python/TestModels, in code (not in comments or strings):
#   - 'from math import sin', ..., and 'math.sin(', ... for the functions exudyn.cMath has;
#   - 'np.sin(', 'np.arctan2(', 'np.power(', ... (numpy as np or numpy);
#   - '**' with an exponent that is not a whole number: 0.5, 1.5, (1/3), ...
#   - 'np.linalg.norm(x)' with one argument: numpy sums in an order of its BLAS library and the CPU; exudyn.cMath.norm
# Allowed: sqrt, pi, fsum, floor, ... of math and numpy, and a whole exponent (x**2), which is counted and reported
# only: pow(x, 2) is exact for most x, but not all. A line whose comment says 'checkPythonMath: allowed' and why is
# not checked, e.g. x**1.2 of a symbolic.Real, which records the expression and computes in C++.
#
# Usage:
#   python tools/checkPythonMath.py            #report
#   python tools/checkPythonMath.py --check    #exit 1 on a finding (the gate)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import argparse
import io
import os
import sys
import tokenize

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
directories = ['python/exudyn', 'python/TestModels']
skipFiles = ['python/exudyn/cMath.py']

#the functions of exudyn.cMath, by the name math and numpy give them
mathFunctions = {name: name for name in ['sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'exp', 'log', 'pow',
                                         'sinh', 'cosh', 'tanh', 'cbrt']}
numpyFunctions = {'sin': 'sin', 'cos': 'cos', 'tan': 'tan', 'arcsin': 'asin', 'arccos': 'acos', 'arctan': 'atan',
                  'arctan2': 'atan2', 'exp': 'exp', 'log': 'log', 'power': 'pow', 'sinh': 'sinh', 'cosh': 'cosh',
                  'tanh': 'tanh', 'cbrt': 'cbrt'}


def IsWholeNumber(text):
    try:
        value = float(text)
    except ValueError:
        return False
    return value == int(value)


def Tokens(text):
    """the tokens of code: names, operators and numbers; comments, strings and layout dropped"""
    skip = {tokenize.COMMENT, tokenize.STRING, tokenize.NL, tokenize.NEWLINE, tokenize.INDENT, tokenize.DEDENT,
            tokenize.ENCODING, tokenize.ENDMARKER}
    for fString in ['FSTRING_START', 'FSTRING_MIDDLE', 'FSTRING_END']:
        if hasattr(tokenize, fString):
            skip.add(getattr(tokenize, fString))
    return [token for token in tokenize.generate_tokens(io.StringIO(text).readline) if token.type not in skip]


def Findings(text, fileName):
    """(findings, number of whole exponents): the findings as messages"""
    findings = []
    wholeExponents = 0
    tokens = Tokens(text)
    allowedLines = {token.start[0] for token in tokenize.generate_tokens(io.StringIO(text).readline)
                    if token.type == tokenize.COMMENT and 'checkPythonMath: allowed' in token.string}

    def Add(token, message):
        if token.start[0] not in allowedLines:
            findings.append(fileName + ':' + str(token.start[0]) + ': ' + message)

    n = len(tokens)
    for i, token in enumerate(tokens):
        nextString = tokens[i+1].string if i+1 < n else ''
        #from math import sin, ... ; from numpy import sin, ...
        if token.string == 'from' and i+2 < n and tokens[i+1].string in ['math', 'numpy'] and tokens[i+2].string == 'import':
            table = mathFunctions if tokens[i+1].string == 'math' else numpyFunctions
            j = i + 3
            while j < n and tokens[j].start[0] == token.start[0] or (j < n and tokens[j].string in [',', '(', ')']):
                name = tokens[j].string
                if name in table and not (j > 0 and tokens[j-1].string == 'as'):
                    Add(tokens[j], 'from ' + tokens[i+1].string + ' import ' + name + ': use exudyn.cMath.'
                        + table[name] + ' (#2964)')
                j += 1
        #math.sin(, np.sin(
        elif token.type == tokenize.NAME and nextString == '.' and i+3 < n and tokens[i+3].string == '(' \
                and (i == 0 or tokens[i-1].string != '.'):
            module = token.string
            name = tokens[i+2].string
            if module == 'math' and name in mathFunctions:
                Add(token, 'math.' + name + '(...) depends on the platform: use exudyn.cMath.' + name + ' (#2964)')
            elif module in ['np', 'numpy'] and name in numpyFunctions:
                Add(token, module + '.' + name + '(...) depends on the platform: use exudyn.cMath.' + numpyFunctions[name]
                    + ' (#2964)')
        #np.linalg.norm(x) with one argument; with axis or ord there is nothing portable to take instead
        elif token.string in ['np', 'numpy'] and i+5 < n and [t.string for t in tokens[i+1:i+6]] == ['.', 'linalg', '.', 'norm', '('] \
                and (i == 0 or tokens[i-1].string != '.'):
            depth, j, oneArgument = 0, i+5, True
            while j < n:
                depth += {'(': 1, ')': -1, '[': 1, ']': -1, '{': 1, '}': -1}.get(tokens[j].string, 0)
                if depth == 0:
                    break
                if depth == 1 and tokens[j].string == ',':
                    oneArgument = False
                j += 1
            if oneArgument:
                Add(token, token.string + '.linalg.norm(...) sums in an order of the BLAS library: use exudyn.cMath.norm (#2963)')
        #x**0.5, x**(1/3)
        elif token.type == tokenize.OP and token.string == '**' and i+1 < n:
            exponent = tokens[i+1]
            if exponent.string == '-' and i+2 < n:
                exponent = tokens[i+2]
            if exponent.type == tokenize.NUMBER:
                if IsWholeNumber(exponent.string):
                    wholeExponents += 1
                else:
                    Add(token, '**' + exponent.string + ' calls the library pow: use sqrt or exudyn.cMath.pow (#2964)')
            elif exponent.string == '(':
                #a parenthesized exponent: a division or a float in it is not a whole number
                depth, j, inner = 0, i+1, []
                while j < n:
                    depth += {'(': 1, ')': -1}.get(tokens[j].string, 0)
                    inner.append(tokens[j])
                    if depth == 0:
                        break
                    j += 1
                if any(t.string == '/' or (t.type == tokenize.NUMBER and not IsWholeNumber(t.string)) for t in inner):
                    Add(token, '**(' + ''.join(t.string for t in inner[1:-1]) + ') calls the library pow: use sqrt, '
                        'cbrt or exudyn.cMath.pow (#2964)')
    return (findings, wholeExponents)


def SourceFiles():
    for directory in directories:
        for (current, subDirectories, fileNames) in os.walk(os.path.join(root, directory)):
            subDirectories[:] = sorted(d for d in subDirectories if d != '__pycache__')
            for fileName in sorted(fileNames):
                relative = os.path.relpath(os.path.join(current, fileName), root).replace(os.sep, '/')
                if fileName.endswith('.py') and relative not in skipFiles:
                    yield relative


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='the Python library and the test models take sin, ... from exudyn.cMath (#2964)')
    parser.add_argument('--check', action='store_true', help='exit 1 on a finding')
    args = parser.parse_args()

    findings = []
    wholeExponents = 0
    for relative in SourceFiles():
        text = io.open(os.path.join(root, relative), 'r', encoding='utf-8', errors='replace').read()
        (fileFindings, fileWholeExponents) = Findings(text, relative)
        findings += fileFindings
        wholeExponents += fileWholeExponents
    for finding in findings:
        print(finding)
    print('checkPythonMath: ' + (str(len(findings)) + ' finding(s)' if findings else 'OK')
          + '; ' + str(wholeExponents) + ' whole exponent(s) such as x**2, reported only')
    sys.exit(1 if (findings and args.check) else 0)
