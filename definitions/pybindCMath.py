#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN definition file
#
# Details:  the submodule exudyn.cMath: the math functions of the C++ code for Python (#2958, #2959);
#           pb holds the page, module the functions, whose C++ bindings go into
#           pybind_manual_classes.h (the compiled submodule exudyn._cMath) and whose stubs go into
#           python/exudyn/cMath.pyi. python/exudyn/cMath.py takes the compiled functions and adds pi.
#           The calls are recorded by PybindInterface (pybindTypes.py) and replayed by
#           tools/generators/pybindEmitter.py.
#
#           DESCRIPTIONS: read definitions/README.md, section "Writing a
#           description", before writing or changing one - what the text may
#           contain, and how it is checked.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-10
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from pybindTypes import *

pb = PybindInterface()
module = PybindInterface()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
pb.AddDocu(r"""The submodule `exudyn.cMath` gives Python the math functions the C++ code of Exudyn computes with: `sin`, `cos`, ..., `sqrt` and `norm`, which give the same bits on every platform. It is for the test models of Exudyn, which use it, and for everybody who wants a model to give the same results on every platform; other scripts may use `math` and `numpy` as before.""",
            section='exudyn.cMath', sectionLevel=1, sectionLabel='sec-exudyn-cmath')

pb.AddDocu(r"""The transcendental functions - `sin`, `exp`, `atan2`, ... - are not correctly rounded by the libraries of the platforms, and each library rounds some results differently: the libraries of Windows and Linux differ by 1 to 3 units in the last place for between 0.1 and 55 percent of the arguments, depending on the function (#2947). A difference in the last bit is usually harmless, but models with contact, friction or impacts amplify it: a contact model gave results differing by a relative 1e-5 between Windows and Linux. Exudyn therefore compiles one portable implementation of these functions (from musl libc) into its C++ code, which gives the same bits on every platform, and `exudyn.cMath` gives the same functions to Python, for the values that enter a model: initial velocities, rotations, points of a geometry. With both, the test models of Exudyn give the same results on Windows and Linux to the last bit.""",
            section='Why own math in Exudyn?', sectionLevel=2, sectionLabel='sec-exudyn-cmath-why')

pb.AddDocu(r"""Each function takes a float, which gives a float, or a list or numpy array, which gives a numpy array of the same shape; the two arguments of `atan2` and `pow` are broadcast like numpy does. A symbolic argument, an `exudyn.symbolic.Real`, gives the function of `exudyn.symbolic` (all but `cbrt`; for `atan2` and `pow` also mixed with a float), so that one function written with `exudyn.cMath` runs on numbers and is recorded as symbolic user function (#3114). A call with one float takes about 0.2 microseconds, against 0.03 of `math.sin`; on arrays the functions are as fast as numpy. `sqrt` is correctly rounded on every platform, also in `math` and `numpy`; it is here so that one import serves. Python's `x**0.5` and `x**y` call the library's `pow` and differ per platform: use `sqrt`, or `pow` of this module. A complex array is not supported: it loses its imaginary part, with the ComplexWarning of numpy.""",
            section='Functions', sectionLevel=2, sectionLabel='sec-exudyn-cmath-functions')

module.AddDocu(r"""Usage: `from exudyn.cMath import sin, cos, pi`, or `import exudyn.cMath as cm` and `cm.sin(x)`. The functions are:""")

#the compiled submodule; 'm' is the submodule in this block, so that the declarations below bind into it
module.CppCode('        {\n        py::module cMathSubmodule = m.def_submodule("_cMath", "the compiled functions of exudyn.cMath");\n'
                    '        {\n        py::module& m = cMathSubmodule;\n')

#name, C++ function, description of the argument range
functions1 = [('sin', 'EM::Sin', 'sine of x (in radians)'),
              ('cos', 'EM::Cos', 'cosine of x (in radians)'),
              ('tan', 'EM::Tan', 'tangent of x (in radians)'),
              ('asin', 'EM::Asin', 'arc sine of x in [-1,1], in [-pi/2, pi/2]'),
              ('acos', 'EM::Acos', 'arc cosine of x in [-1,1], in [0, pi]'),
              ('atan', 'EM::Atan', 'arc tangent of x, in [-pi/2, pi/2]'),
              ('exp', 'EM::Exp', 'exponential function of x'),
              ('log', 'EM::Log', 'natural logarithm of x'),
              ('sinh', 'EM::Sinh', 'hyperbolic sine of x'),
              ('cosh', 'EM::Cosh', 'hyperbolic cosine of x'),
              ('tanh', 'EM::Tanh', 'hyperbolic tangent of x'),
              ('cbrt', 'EM::Cbrt', 'cube root of x, also for a negative x'),
              ('sqrt', 'std::sqrt', 'square root of x; correctly rounded on every platform'),
              ]
functions2 = [('atan2', 'EM::Atan2', 'arc tangent of y/x in [-pi, pi], with the quadrant of the point (x, y); y is the FIRST argument, as in math.atan2'),
              ('pow', 'EM::Pow', 'x to the power y'),
              ]
argumentType = 'ArrayLike'
returnType = 'Union[float, NDArray]'
#a float goes to an overload of its own, registered first: pybind11 tries the overloads in order, first without
#conversion, so a float takes it at once and a float64 array the vectorized one; an int or a list is converted and takes
#the vectorized one. A float through py::vectorize took 530 ns per call, this overload 155 ns (math.sin: 30 ns)
scalarOverload = '        m.def("{name}", []({arguments}) {{ return {call}; }}, {pyArgs});\n'
#a symbolic argument, exudyn.symbolic.Real, goes to the function of exudyn.symbolic (#3114): one function written with
#cMath runs on numbers and is recorded as symbolic user function; cbrt has no symbolic counterpart
symbolicOverload = '        m.def("{name}", &Symbolic::SReal::{name}<{types}>, {pyArgs});\n'
symbolicNames = ['sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'exp', 'log', 'sinh', 'cosh', 'tanh', 'sqrt', 'atan2', 'pow']

for (name, cName, description) in functions1:
    module.CppCode(scalarOverload.format(name=name, arguments='Real x', call=cName + '(x)', pyArgs='py::arg("x")'))
    if name in symbolicNames:
        module.CppCode(symbolicOverload.format(name=name, types='Symbolic::SReal', pyArgs='py::arg("x")'))
    module.DefPyFunctionAccess(cClass='', pyName=name, cName='py::vectorize([](Real x) { return ' + cName + '(x); })',
                                    isLambdaFunction=True,
                                    description=description + '; calls ' + cName + ' of the C++ code',
                                    argList=['x'],
                                    argTypes=[argumentType],
                                    returnType=returnType,
                                    )

for (name, cName, description) in functions2:
    (a0, a1) = ('y', 'x') if name == 'atan2' else ('x', 'y')
    module.CppCode(scalarOverload.format(name=name, arguments='Real ' + a0 + ', Real ' + a1, call=cName + '(' + a0 + ', ' + a1 + ')',
                                              pyArgs='py::arg("' + a0 + '"), py::arg("' + a1 + '")'))
    for types in ['Symbolic::SReal, Symbolic::SReal', 'Symbolic::SReal, Real', 'Real, Symbolic::SReal']:
        module.CppCode(symbolicOverload.format(name=name, types=types, pyArgs='py::arg("' + a0 + '"), py::arg("' + a1 + '")'))
    module.DefPyFunctionAccess(cClass='', pyName=name, cName='py::vectorize([](Real ' + a0 + ', Real ' + a1 + ') { return ' + cName + '(' + a0 + ', ' + a1 + '); })',
                                    isLambdaFunction=True,
                                    description=description + '; calls ' + cName + ' of the C++ code',
                                    argList=[a0, a1],
                                    argTypes=[argumentType, argumentType],
                                    returnType=returnType,
                                    )

module.CppCode('        }\n        }\n')

module.DefDataAccess('pi', 'the float closest to pi, the same as math.pi; defined in python/exudyn/cMath.py',
                          dataType='float', isTopLevel=True)
module.BeginCppWrittenByHand() #defined in python/exudyn/cMath.py
module.DefPyFunctionAccess(cClass='', pyName='norm', cName='',
                           description='the Euclidean norm of a vector, or the Frobenius norm of a matrix: the square root of the sum of the squares, added by math.fsum without rounding before the end; numpy.linalg.norm sums in an order that depends on the BLAS library and the CPU (#2963); defined in python/exudyn/cMath.py',
                           argList=['x'],
                           argTypes=['ArrayLike'],
                           returnType='float',
                           )
module.EndCppWrittenByHand()
module.DefFinishTable()
