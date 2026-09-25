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
    argumentText  {name: description} from the Args: lines, without the leading size formula
    returnText    the Returns: line, or ''
    argumentSize  {name: size} for the arguments whose description began with one, as '$\\in ...$'
    returnSize    the size the Returns: line began with, or ''
    """

    def __init__(self, name, arguments, returnType, summary, details, argumentText, returnText,
                 argumentSize=None, returnSize=''):
        self.name = name
        self.arguments = arguments
        self.returnType = returnType
        self.summary = summary
        self.details = details
        self.argumentText = argumentText
        self.returnText = returnText
        self.argumentSize = argumentSize or {}
        self.returnSize = returnSize

    def TypeAndSize(self, annotation, size):
        """what the 'type or size' column of the table says: the annotation, and the size if the
        size is not already in it"""
        return annotation + (' ' + size if size != '' else '')

    def Signature(self):
        """the signature as the documentation prints it: the documented name and the argument names"""
        return self.name + '(' + ', '.join(name for (name, _) in self.arguments) + ')'

    def __repr__(self):
        return ('UserFunction(' + self.Signature() + ' -> ' + repr(self.returnType) + ', '
                + str(len(self.argumentText)) + ' described)')


def _Annotation(node):
    """an annotation as it is written in the file; '' when the argument carries none"""
    return '' if node is None else ast.unparse(node)


def _SplitSize(text):
    """(size, description) of one Args: or Returns: line

    A description that begins with '$\\in ...$' begins with the SIZE of the argument, which the table
    prints in the type column: 'q: $\\in \\Rcal^n$ object coordinates' is a Vector of n entries. A
    formula that does not start with \\in is part of the description - '$\\fv$ copied from object' is
    the symbol of the argument and not its size."""
    match = re.match(r'^(\$\\in\s[^$]*\$)\s*(.*)$', text, flags=re.DOTALL)
    if match is None:
        return ('', text)
    return (match.group(1), match.group(2).strip())


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
    returnSize = ''
    sizes = {}
    for index in range(1, len(sections) - 1, 2):
        (name, body) = (sections[index], sections[index + 1])
        if name.startswith('Return'):
            (returnSize, returnText) = _SplitSize(
                ' '.join(line.strip() for line in body.strip().split('\n')).strip())
            continue
        current = None
        for line in textwrap.dedent(body).split('\n'):
            match = re.match(r'^\s*([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.*)$', line)
            if match is not None:
                current = match.group(1)
                arguments[current] = match.group(2).strip()
            elif current is not None and line.strip() != '':
                arguments[current] += ' ' + line.strip()
    #Google style: the summary is the FIRST LINE, and the details are what follows it. The details
    #keep their own line breaks, because a description is Markdown and a writer laid those out
    for (name, description) in list(arguments.items()):
        (sizes[name], arguments[name]) = _SplitSize(description)

    lines = prose.split('\n')
    summary = lines[0].strip()
    details = '\n'.join(lines[1:]).strip()
    return (summary, details, arguments, returnText, sizes, returnSize)


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
    (summary, details, argumentText, returnText, sizes, returnSize) = _SplitDocstring(
        ast.get_docstring(node))

    described = set(argumentText)
    known = set(name for (name, _) in arguments)
    unknown = sorted(described - known)
    if unknown:
        raise ValueError('ReadUserFunction: ' + node.name + ' describes arguments it does not have: '
                         + ', '.join(unknown))

    return UserFunction(name or node.name, arguments, _Annotation(node.returns),
                        summary, details, argumentText, returnText, sizes, returnSize)


#the Python annotation that a C++ type of a std::function accepts, one to one (revision2026b step
#RG12.4, #2664). A FIXED size is part of the annotation - Vector3D is not Vector - because that is
#what the argument table of a user function says and what the C++ signature needs. A size that is not
#fixed is a formula in the argument's description, which is where a formula renders. py::object says
#nothing about what it carries, so it accepts the two things that are passed as one.
cppToAnnotation = {'MainSystem': ['MainSystem'],
                   'Real': ['Real'],
                   'Index': ['Index'],
                   'int': ['Index'],
                   'bool': ['Bool'],
                   'StdVector': ['Vector'],
                   'StdVector2D': ['Vector2D'],
                   'StdVector3D': ['Vector3D'],
                   'StdVector6D': ['Vector6D'],
                   'StdMatrix3D': ['Matrix3D'],
                   'StdMatrix6D': ['Matrix6D'],
                   'NumpyMatrix': ['NumpyMatrix'],
                   'StdArrayIndex': ['Array'],
                   'ConfigurationType': ['ConfigurationType'],
                   'py::object': ['BodyGraphicsData', 'MatrixContainer'],
                   }


#the runtime Python type of an annotation, for the generated Protocol in itemInterface.py: there the
#name has to exist when the module is imported, so it is exudyn's own class or a builtin. A
#definition file's MainSystem is a name for an annotation; exudyn.MainSystem is the class itself.
annotationToPython = {'MainSystem': 'exudyn.MainSystem',
                      'Real': 'float',
                      'Index': 'int',
                      'Bool': 'bool',
                      'Vector': 'np.ndarray',
                      'Vector2D': 'np.ndarray',
                      'Vector3D': 'np.ndarray',
                      'Vector6D': 'np.ndarray',
                      'Matrix3D': 'np.ndarray',
                      'Matrix6D': 'np.ndarray',
                      'NumpyMatrix': 'np.ndarray',
                      'Array': 'np.ndarray',
                      'BodyGraphicsData': 'list',
                      'MatrixContainer': 'exudyn.MatrixContainer',
                      'ConfigurationType': 'exudyn.ConfigurationType',
                      }


def PythonType(annotation):
    """the runtime type the generated Protocol uses for an annotation of a definition file"""
    if annotation not in annotationToPython:
        raise ValueError('PythonType: ' + repr(annotation) + ' has no runtime type - add it to '
                         'userFunctionModel.annotationToPython')
    return annotationToPython[annotation]


def CppSignatureTypes(stdFunction):
    """([argument types], return type) of a std::function<...> as definitionTypes.py writes it

    'const MainSystem&' is reported as 'MainSystem': a reference and a const are how C++ takes an
    argument and say nothing a Python annotation could state."""
    inner = stdFunction.split('<', 1)[1].rsplit('>', 1)[0]
    returnType = inner.split('(', 1)[0].strip()
    arguments = [argument.replace('const', '').replace('&', '').strip()
                 for argument in inner.split('(', 1)[1].rsplit(')', 1)[0].split(',')
                 if argument.strip() != '']
    return (arguments, returnType)


def CheckAgainstCpp(userFunction, stdFunction):
    """what disagrees between the Python def and the C++ user function; [] when they agree

    This is the only place where the two halves of a user function meet, and until it existed
    nothing compared them: a signature was stated in five places and checked in none, so a
    disagreement was found by a user whose function was called with the wrong number of arguments
    (revision2026b step RG12.4, #2664)."""
    (cppArguments, cppReturn) = CppSignatureTypes(stdFunction)
    findings = []

    if len(userFunction.arguments) != len(cppArguments):
        return [('takes ' + str(len(userFunction.arguments)) + ' argument(s), the C++ user function '
                 + str(len(cppArguments)) + ': ' + ', '.join(cppArguments))]

    for (index, (name, annotation)) in enumerate(userFunction.arguments):
        accepted = cppToAnnotation.get(cppArguments[index])
        if accepted is None:
            findings.append('argument ' + name + ': the C++ type ' + cppArguments[index]
                            + ' has no Python annotation - add it to userFunctionModel')
        elif annotation not in accepted:
            findings.append('argument ' + name + ': ' + repr(annotation) + ' cannot be the C++ '
                            + cppArguments[index] + ' (' + ' or '.join(accepted) + ')')
        if name not in userFunction.argumentText:
            findings.append('argument ' + name + ': the docstring does not describe it, so its '
                            'row of the table would be empty')

    accepted = cppToAnnotation.get(cppReturn)
    if accepted is None:
        findings.append('the return: the C++ type ' + cppReturn + ' has no Python annotation'
                        ' - add it to userFunctionModel')
    elif userFunction.returnType not in accepted:
        findings.append('the return: ' + repr(userFunction.returnType) + ' cannot be the C++ '
                        + cppReturn + ' (' + ' or '.join(accepted) + ')')
    return findings


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
