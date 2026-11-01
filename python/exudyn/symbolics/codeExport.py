#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Writes symbolic expressions and recorded symbolic user functions as code, from the structure of their flat
#           program (#3087, #3088): exudyn.symbolic.GetFlatProgramData(expressions) or
#           UserFunction.GetFlatProgramData(). Entry k of its 'operations' is node k of the recording, the tK of
#           GetFlatProgram(); an operation refers to its operands by their numbers, which are always smaller than k.
#           The formats:
#             'Python'   the text of GetFlatProgram() and str(), not executable (bare function names);
#             'symbolic' Python with exudyn.symbolic: called with symbolic arguments, it records the same graph;
#             'numpy'    Python with numpy, runs without Exudyn;
#             'sympy'    Python that builds sympy expressions; sympy writes C, C++, Fortran, Julia, Matlab, LaTeX, ...
#           Every format computes what Exudyn computes: mod is C fmod, round rounds halves away from zero, a comparison
#           is the number 0 or 1, IfThenElse is true for a condition other than 0; an exudyn.symbolic.LookupTable is
#           written with its values, by a function of the program in numpy and sympy (#3175).
#           exudyn.symbolic.GetFlatProgram(..., format=...), UserFunction.GetFlatProgram(format=...) and
#           ToString(format=...) of exudyn.symbolic.Real, Vector and Matrix call Write(); the format 'Python' of
#           GetFlatProgram() and ToString() is written by C++, and Write() gives the same text.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-25 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import math

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = ['formats', 'Write', 'ToFunction', 'ToSympy']

#the formats of Write()
formats = ['Python', 'symbolic', 'numpy', 'sympy']

#the imports at the top of an executable program
_imports = {'symbolic': 'import exudyn as exu\nesym = exu.symbolic\n',
            'numpy': 'import numpy as np\n',
            'sympy': 'import sympy\n'}

_comparisons = {'==': 'Eq', '!=': 'Ne', '>': 'Gt', '>=': 'Ge', '<=': 'Le', '<': 'Lt'}
_unaryFunctions = ['abs', 'sign', 'round', 'ceil', 'floor', 'sqrt', 'exp', 'log', 'sin', 'cos', 'tan', 'asin', 'acos',
                   'atan', 'sinh', 'cosh', 'tanh', 'asinh', 'acosh', 'atanh', 'isfinite', 'Not']
_binaryFunctions = ['pow', 'atan2', 'mod', 'min', 'max']

#names in numpy and sympy where they differ from exudyn.symbolic; a template where it is not one function
_numpyFunctions = {'asin': 'np.arcsin', 'acos': 'np.arccos', 'atan': 'np.arctan', 'asinh': 'np.arcsinh',
                   'acosh': 'np.arccosh', 'atanh': 'np.arctanh', 'pow': 'np.power', 'atan2': 'np.arctan2',
                   'mod': 'np.fmod', 'min': 'np.minimum', 'max': 'np.maximum',
                   'isfinite': '(1.0*np.isfinite({0}))', 'Not': '(1.0*({0} == 0))',
                   'round': '(np.trunc({0}) + np.sign({0})*(np.abs({0} - np.trunc({0})) >= 0.5))'}
_sympyFunctions = {'abs': 'sympy.Abs', 'ceil': 'sympy.ceiling', 'min': 'sympy.Min', 'max': 'sympy.Max',
                   'pow': '({0}**{1})', 'mod': '({0} - {1}*sympy.sign({0}/{1})*sympy.floor(sympy.Abs({0}/{1})))',
                   'Not': 'sympy.Piecewise((1, sympy.Eq({0}, 0)), (0, True))',
                   'round': '(sympy.sign({0})*sympy.floor(sympy.Abs({0}) + sympy.Rational(1, 2)))'}
_modulePrefix = {'symbolic': 'esym.', 'numpy': 'np.', 'sympy': 'sympy.'}

#the lookup table of exudyn.symbolic (#3175) in the formats numpy and sympy, written into a program that uses one: the
#value at x - with derivative the slope of its interval - of values at start + i*step, linearly interpolated, as
#LookupTableData::Value of the C++ code
_lookupTableHelpers = {'numpy': """def _LookupTable(values, step, start, outside, derivative, x):
    n = len(values)
    u = (x - start)/step
    if outside == 'periodic':
        i = int(np.floor(u))
        fraction = u - i
        i = i % n
        (a, b) = (values[i], values[(i + 1) % n])
        return (b - a)/step if derivative else a + (b - a)*fraction
    if u < 0 or u > n - 1:
        if derivative or outside == 'zero':
            return 0.
        return values[0] if u < 0 else values[n - 1]
    if n == 1:
        return 0. if derivative else values[0]
    i = min(int(np.floor(u)), n - 2)
    (a, b) = (values[i], values[i + 1])
    return (b - a)/step if derivative else a + (b - a)*(u - i)
""", 'sympy': """def _LookupTable(values, step, start, outside, derivative, x):
    n = len(values)
    u = (x - start)/step
    def Segment(i, b):
        return (b - values[i])/step if derivative else values[i] + (b - values[i])*(u - i)
    if outside == 'periodic':
        u = u - n*sympy.floor(u/n)
        pieces = [(Segment(i, values[(i + 1) % n]), u < i + 1) for i in range(n - 1)]
        return sympy.Piecewise(*(pieces + [(Segment(n - 1, values[0]), True)]))
    (low, high) = (0, 0) if (derivative or outside == 'zero') else (values[0], values[n - 1])
    if n == 1:
        return sympy.Piecewise((0 if derivative else values[0], sympy.Eq(u, 0)), (low, u < 0), (high, True))
    pieces = [(low, u < 0)] + [(Segment(i, values[i + 1]), u <= i + 1) for i in range(n - 1)]
    return sympy.Piecewise(*(pieces + [(high, True)]))
"""}
_lookupTableOperations = ['LookupTable', 'LookupTableDerivative']


def _Number(value, format):
    """a number, a vector or a matrix of numbers in an executable format"""
    if isinstance(value, list):
        isMatrix = len(value) != 0 and isinstance(value[0], list)
        rows = value if isMatrix else [value]
        texts = ['[' + ', '.join(_Number(item, format) for item in row) + ']' for row in rows]
        text = '[' + ', '.join(texts) + ']' if isMatrix else texts[0]
        if format == 'Python':
            return text
        constructor = {'symbolic': 'esym.Matrix(' if isMatrix else 'esym.Vector(', 'numpy': 'np.array(',
                       'sympy': 'sympy.Matrix('}[format]
        return constructor + text + ')'
    value = float(value)
    if math.isfinite(value):
        return repr(value)
    if math.isnan(value):
        return {'Python': "float('nan')", 'symbolic': "float('nan')", 'numpy': 'np.nan', 'sympy': 'sympy.nan'}[format]
    sign = '-' if value < 0 else ''
    return sign + {'Python': "float('inf')", 'symbolic': "float('inf')", 'numpy': 'np.inf', 'sympy': 'sympy.oo'}[format]


def _Index(entry, text):
    """the text of an index operand: an integer, if it is a number"""
    if entry['op'] == 'number':
        return str(int(entry['value']))
    return 'int(' + text + ')'


def _Function(function, operands, format):
    """a function of exudyn.symbolic in a format"""
    if format == 'Python':
        return function + '(' + ', '.join(operands) + ')'
    table = {'numpy': _numpyFunctions, 'sympy': _sympyFunctions}.get(format, {})
    if format == 'sympy' and function == 'isfinite':
        raise ValueError('symbolicExport: isfinite has no meaning in the format sympy, where a symbol is finite')
    template = table.get(function, _modulePrefix[format] + function)
    if '{' not in template:
        template += '(' + ', '.join('{' + str(i) + '}' for i in range(len(operands))) + ')'
    return template.format(*operands)


def _Operation(entry, operands, operandEntries, format):
    """the text of one operation, its operands given as text"""
    op = entry['op']
    kind = entry['kind']
    kinds = [operandEntry['kind'] for operandEntry in operandEntries]
    if op in ['+', '-', '*', '/']:
        return '(' + operands[0] + ' ' + op + ' ' + operands[1] + ')'
    if op in _comparisons:
        text = '(' + operands[0] + ' ' + op + ' ' + operands[1] + ')'
        if format in ['Python', 'symbolic']:
            return text
        if kinds[0] == 'Vector': #the comparison of two vectors: 1 if all components are equal
            if format == 'numpy':
                return ('(1.0*np.array_equal(' if op == '==' else '(1.0*(not np.array_equal(') + operands[0] + ', ' \
                    + operands[1] + (')' if op == '==' else ')))')
            pairs = '[sympy.' + _comparisons[op] + '(_a, _b) for _a, _b in zip(' + operands[0] + ', ' + operands[1] + ')]'
            return 'sympy.Piecewise((1, sympy.' + ('And' if op == '==' else 'Or') + '(*' + pairs + ')), (0, True))'
        if format == 'numpy':
            return '(1.0*' + text + ')'
        return 'sympy.Piecewise((1, sympy.' + _comparisons[op] + '(' + operands[0] + ', ' + operands[1] + ')), (0, True))'
    if op == 'pos':
        return operands[0]
    if op == 'neg':
        return '(-' + operands[0] + ')'
    if op in _unaryFunctions or op in _binaryFunctions:
        return _Function(op, operands, format)
    if op == 'IfThenElse':
        return {'Python': 'IfThenElse({0},{1},{2})', 'symbolic': 'esym.IfThenElse({0}, {1}, {2})',
                'numpy': '({1} if {0} else {2})',
                'sympy': 'sympy.Piecewise(({1}, sympy.Ne({0}, 0)), ({2}, True))'}[format].format(*operands)
    if op in _lookupTableOperations:
        if format == 'Python':
            return op + '(' + operands[0] + ')'
        table = entry['table']
        data = (_Number(table['values'], 'Python') + ', ' + _Number(table['step'], format) + ', '
                + _Number(table['start'], format) + ", '" + table['outside'] + "'")
        if format == 'symbolic':
            return ('esym.LookupTable(' + data + ')' + ('.Derivative(' if op == 'LookupTableDerivative' else '(')
                    + operands[0] + ')')
        return '_LookupTable(' + data + ', ' + str(op == 'LookupTableDerivative') + ', ' + operands[0] + ')'
    if op == 'vector':
        text = '[' + ', '.join(operands) + ']'
        return text if format == 'Python' else {'symbolic': 'esym.Vector(', 'numpy': 'np.array(', 'sympy': 'sympy.Matrix('}[format] + text + ')'
    if op == 'matrix':
        columns = entry['columns']
        rows = ['[' + ', '.join(operands[row*columns:(row+1)*columns]) + ']' for row in range(entry['rows'])]
        text = '[' + ', '.join(rows) + ']'
        return text if format == 'Python' else {'symbolic': 'esym.Matrix(', 'numpy': 'np.array(', 'sympy': 'sympy.Matrix('}[format] + text + ')'
    if op == '@':
        if format == 'sympy' and kind == 'Real': #the scalar product of two vectors
            return '(' + operands[0] + '.dot(' + operands[1] + '))'
        return '(' + operands[0] + ' @ ' + operands[1] + ')'
    if op == 'vector@matrix': #the operands are the matrix and the vector
        if format == 'sympy':
            return '(' + operands[0] + '.T @ ' + operands[1] + ')'
        return '(' + operands[1] + ' @ ' + operands[0] + ')'
    if op == 'multiply':
        return {'Python': '{0}.multiply({1})', 'symbolic': '{0}.multiply({1})', 'numpy': 'np.multiply({0}, {1})',
                'sympy': '{0}.multiply_elementwise({1})'}[format].format(*operands)
    if op == 'NormL2':
        return {'Python': 'NormL2({0})', 'symbolic': '{0}.NormL2()', 'numpy': 'np.linalg.norm({0})',
                'sympy': '{0}.norm()'}[format].format(*operands)
    if op == 'T':
        return operands[0] + '.T'
    if op == 'component':
        if format == 'Python':
            return operands[0] + '[' + ','.join(operands[1:]) + ']'
        return operands[0] + '[' + ', '.join(_Index(e, t) for (e, t) in zip(operandEntries[1:], operands[1:])) + ']'
    raise ValueError('symbolicExport: the operation "' + str(op) + '" is not known to the format ' + format)


def _Leaf(entry, format):
    """the text of a number or a named variable"""
    if entry['op'] == 'variable':
        return entry['name']
    if entry['op'] == 'number':
        return entry['text'] if format == 'Python' else _Number(entry['value'], format)
    if format == 'Python':
        return entry['text']
    raise ValueError('symbolicExport: the expression ' + entry['text'] + ' has no operations that the format '
                     + format + ' can write')


def Write(data, format='Python', flatten=True, name='', expressionOnly=False):
    """write the flat program of symbolic expressions or of a recorded user function as code; for exudyn.symbolic.Real,
    Vector and Matrix, ToString(format) calls it, for user functions and expressions GetFlatProgram(format=...)

    Args:
        data: the structure of exudyn.symbolic.GetFlatProgramData(...) or UserFunction.GetFlatProgramData()
        format: 'Python' (the text of GetFlatProgram()), 'symbolic', 'numpy' or 'sympy' (executable Python)
        flatten: True: one line per operation, each node computed once; False: each result as one expression
        name: the name of the function of an executable format; default: the name of the user function, or F
        expressionOnly: True: only the expression of the first result, as ToString() gives it

    Returns:
        the code as str; for an executable format a function with the arguments of the user function, or of the
        named variables of the expressions, and the variables of exudyn.symbolic.variables as keyword arguments
    """
    if format not in formats:
        raise ValueError('symbolicExport.Write: format must be one of ' + str(formats) + ', but received ' + str(format))
    operations = data['operations']
    texts = [''] * len(operations)
    lines = []
    for (k, entry) in enumerate(operations):
        if 'operands' not in entry:
            text = _Leaf(entry, format)
        else:
            operandEntries = [operations[i] for i in entry['operands']]
            text = _Operation(entry, [texts[i] for i in entry['operands']], operandEntries, format)
        if entry['inline'] or not flatten or expressionOnly:
            texts[k] = text
        else:
            texts[k] = 't' + str(k)
            lines.append(texts[k] + ' = ' + text)
    results = [(result['name'], texts[result['node']]) for result in data['results']]
    if expressionOnly:
        return results[0][1]

    numberOfComputations = sum(1 for entry in operations if not entry['inline'])
    isUserFunction = 'variables' in data
    if format == 'Python':
        lines = lines if flatten else []
        header = ''
        if isUserFunction:
            if numberOfComputations == 0: #as C++: a leaf has no flat program
                return '#' + data['name'] + ': no flat program; the result is evaluated as a tree\n'
            header = '#' + data['name'] + ': ' + str(numberOfComputations) + ' operations\n'
        return header + ''.join(line + '\n' for line in lines) + ''.join(n + ' = ' + t + '\n' for (n, t) in results)

    #the variables of exudyn.symbolic.variables: read from there by the format symbolic, which keeps the signature of the
    #user function and records them again; keyword arguments with their present value otherwise
    parameters = [argument['name'] for argument in data['arguments']]
    variableLines = []
    for variable in data.get('variables', []):
        if format == 'symbolic':
            variableLines.append(variable['name'] + " = esym.variables.Get('" + variable['name'] + "')")
        else:
            parameters.append(variable['name'] + '=' + _Number(variable['value'], format))
    functionName = name if name else data.get('name', 'F')
    body = variableLines + (lines if flatten else [])
    returns = ', '.join(t for (n, t) in results)
    helper = ''
    if format in _lookupTableHelpers and any(entry.get('op') in _lookupTableOperations for entry in operations):
        helper = _lookupTableHelpers[format] + '\n'
    return (_imports[format] + helper + 'def ' + functionName + '(' + ', '.join(parameters) + '):\n'
            + ''.join('    ' + line + '\n' for line in body) + '    return ' + returns + '\n')


def ToFunction(data, format='numpy', flatten=True):
    """the Python function of an executable format, as Write() gives it as text

    Args:
        data: the structure of exudyn.symbolic.GetFlatProgramData(...) or UserFunction.GetFlatProgramData()
        format: 'symbolic', 'numpy' or 'sympy'
        flatten: as in Write()

    Returns:
        the function, with the arguments of the user function or of the named variables
    """
    if format == 'Python':
        raise ValueError('symbolicExport.ToFunction: the format Python is not executable; use symbolic, numpy or sympy')
    space = {}
    exec(Write(data, format, flatten, name='_F'), space)
    return space['_F']


def ToSympy(data):
    """the results as sympy expressions, with a sympy symbol per argument: x for a number, the components q[0], q[1],
    ... of a vector q as a column sympy.Matrix, M[0][0], ... of a matrix M; sympy must be installed

    Args:
        data: the structure of exudyn.symbolic.GetFlatProgramData(...) or UserFunction.GetFlatProgramData()

    Returns:
        (arguments, results): a dict of the argument names to their symbols (None for the MainSystem) and the list of
        the results; a variable of exudyn.symbolic.variables is a symbol as well
    """
    import sympy #optional: needed only here and in the code of the format sympy
    arguments = {}
    for argument in data['arguments'] + data.get('variables', []):
        argumentName, kind = argument['name'], argument['kind']
        if kind == 'MainSystem':
            arguments[argumentName] = None
        elif kind == 'Real':
            arguments[argumentName] = sympy.Symbol(argumentName)
        elif kind == 'Vector':
            if not argument.get('size', 0):
                raise ValueError('symbolicExport.ToSympy: the size of the vector ' + argumentName + ' is not known')
            arguments[argumentName] = sympy.Matrix([sympy.Symbol(argumentName + '[' + str(i) + ']')
                                                    for i in range(argument['size'])])
        else:
            arguments[argumentName] = sympy.Matrix([[sympy.Symbol(argumentName + '[' + str(i) + '][' + str(j) + ']')
                                                     for j in range(argument['columns'])] for i in range(argument['rows'])])
    function = ToFunction(data, 'sympy')
    results = function(**arguments)
    return (arguments, list(results) if len(data['results']) > 1 else [results])
