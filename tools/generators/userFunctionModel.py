#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool - part of the code generators
#
# Details:  Reads a user function as the definitions write it: an ordinary Python def with real
#           annotations and a real docstring, passed to the ItemParameter it belongs to
#           (revision2026b step RG12.4, #2664). Everything else about a user function - the
#           documentation block, the argument table, the entries of userFunctionArgsDict and the
#           consistency check against the std::function - is derived from what this module returns.
#
#           NOTHING IS EXECUTED. The function object is only used to find its source, which is then
#           read with ast: an annotation is reported as it is WRITTEN, so "np.ndarray" stays
#           "np.ndarray" and a name that means something only to the documentation stays that name.
#
#           The SIZE of an argument is not part of its type. It belongs in the argument's line of
#           the docstring, as a formula - maintainer, 2026-09-25 - because a formula renders there
#           and does not render inside a code block. That is why the arguments can be a signature
#           at all; see revision2026b step RG3.14.5.
#
# Usage:    from userFunctionModel import ReadUserFunction
#           python tools/generators/userFunctionModel.py     #read every user function and print it
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-25 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import ast
import inspect
import re
import textwrap


class UserFunction:
    """one user function, as the documentation and the interface need it

    name          the name the documentation prints - the parameter's pythonName, not the def's
    arguments     [(name, annotation)] in order, the annotation exactly as written
    returnType    the return annotation as written, or '' when there is none
    summary       the first paragraph of the docstring
    details       what follows it, or ''
    argumentText  {name: description} from the Args: lines
    returnText    the Returns: line, or ''
    """

    def __init__(self, name, arguments, returnType, summary, details, argumentText, returnText):
        self.name = name
        self.arguments = arguments
        self.returnType = returnType
        self.summary = summary
        self.details = details
        self.argumentText = argumentText
        self.returnText = returnText

    def Signature(self):
        """the signature as the documentation prints it: the documented name and the argument names"""
        return self.name + '(' + ', '.join(name for (name, _) in self.arguments) + ')'

    def __repr__(self):
        return ('UserFunction(' + self.Signature() + ' -> ' + repr(self.returnType) + ', '
                + str(len(self.argumentText)) + ' described)')


def _Annotation(node):
    """an annotation as it is written in the file; '' when the argument carries none"""
    return '' if node is None else ast.unparse(node)


def _SplitDocstring(text):
    """(summary, details, {argument: text}, returnText) of a Google-style docstring

    Only two sections are read, Args: and Returns:, because those are the two the documentation has
    a place for. docstringText already parses this shape for the stub files, so a writer meets one
    convention and not two."""
    text = textwrap.dedent(text or '').strip()
    sections = re.split(r'(?m)^(Args|Arguments|Returns|Return)\s*:\s*$', text)
    prose = sections[0].strip()
    arguments = {}
    returnText = ''
    for index in range(1, len(sections) - 1, 2):
        (name, body) = (sections[index], sections[index + 1])
        if name.startswith('Return'):
            returnText = ' '.join(line.strip() for line in body.strip().split('\n')).strip()
            continue
        current = None
        for line in textwrap.dedent(body).split('\n'):
            match = re.match(r'^\s*([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.*)$', line)
            if match is not None:
                current = match.group(1)
                arguments[current] = match.group(2).strip()
            elif current is not None and line.strip() != '':
                arguments[current] += ' ' + line.strip()
    paragraphs = prose.split('\n\n')
    summary = ' '.join(line.strip() for line in paragraphs[0].split('\n')).strip()
    details = '\n\n'.join(paragraph.strip() for paragraph in paragraphs[1:]).strip()
    return (summary, details, arguments, returnText)


def ReadUserFunction(function, name=None):
    """the UserFunction of a def written in a definition file

    'name' is what the documentation calls it - the parameter's pythonName. The def itself is named
    <Item>_<parameter> so that the 35 user functions of one definition file do not shadow each
    other, and that name is deliberately not the one that is printed."""
    source = textwrap.dedent(inspect.getsource(function))
    tree = ast.parse(source)
    node = tree.body[0]
    if not isinstance(node, ast.FunctionDef):
        raise ValueError('ReadUserFunction: ' + repr(getattr(function, '__name__', function))
                         + ' is not a plain function definition')

    arguments = [(argument.arg, _Annotation(argument.annotation)) for argument in node.args.args]
    (summary, details, argumentText, returnText) = _SplitDocstring(ast.get_docstring(node))

    described = set(argumentText)
    known = set(name for (name, _) in arguments)
    unknown = sorted(described - known)
    if unknown:
        raise ValueError('ReadUserFunction: ' + node.name + ' describes arguments it does not have: '
                         + ', '.join(unknown))

    return UserFunction(name or node.name, arguments, _Annotation(node.returns),
                        summary, details, argumentText, returnText)


def UserFunctionsOf(definition):
    """(parameterName, UserFunction) for every member of an item that carries one"""
    found = []
    for member in definition.get('members', []):
        function = member.get('userFunction')
        if function is not None:
            found.append((member['pythonName'], ReadUserFunction(function, member['pythonName'])))
    return found


def main():
    """read every user function the definitions carry, and print what was read"""
    import os
    import sys
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    for directory in [os.path.join(root, 'definitions'), os.path.dirname(os.path.abspath(__file__))]:
        if directory not in sys.path:
            sys.path.insert(0, directory)

    total = 0
    for module in ['itemDefsNodes', 'itemDefsObjects', 'itemDefsMarkers', 'itemDefsLoads',
                   'itemDefsSensors']:
        for definition in __import__(module).definitions:
            for (parameterName, userFunction) in UserFunctionsOf(definition):
                total += 1
                print(definition['className'] + '.' + parameterName)
                print('   ' + userFunction.Signature() + ' -> ' + userFunction.returnType)
                for (name, annotation) in userFunction.arguments:
                    print('      ' + name.ljust(12) + annotation.ljust(14)
                          + userFunction.argumentText.get(name, '(not described)'))
                print('      ' + 'return'.ljust(12) + userFunction.returnType.ljust(14)
                      + (userFunction.returnText or '(not described)'))
                print('      summary: ' + userFunction.summary)
    print('')
    print(str(total) + ' user function(s) read')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
