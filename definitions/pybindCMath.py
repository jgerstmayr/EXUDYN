#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN definition file
#
# Details:  the submodule exudyn.cMath: the math functions of the C++ code for Python (#2958, #2959);
#           pb holds the page, cMathModule the functions, whose C++ bindings go into
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
cMathModule = PybindInterface()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
pb.CreateNewRSTfile('CMath')

pb.AddDocu(r"""The submodule `exudyn.cMath` gives Python the math functions the C++ code of Exudyn computes with. They give the same bits on every platform: Exudyn compiles portable implementations of them (from musl libc), while the functions of `math` and `numpy` come from the library of each platform and differ in the last bit between Windows, Linux and macOS for some arguments. A value that enters a simulation - an initial velocity, a rotation, a point of a geometry - therefore gives the same results everywhere if it is computed with `exudyn.cMath`. Each function takes a float, which gives a float, or a list or numpy array, which gives a numpy array of the same shape; the two arguments of `atan2` and `pow` are broadcast like numpy does. `sqrt` is correctly rounded on every platform, also in `math` and `numpy`; it is here so that one import serves. Python's `x**0.5` and `x**y` call the library's `pow` and differ per platform: use `sqrt`, or `pow` of this module.""",
            section='exudyn.cMath', sectionLevel=1, sectionLabel='sec-exudyn-cmath')

cMathModule.AddDocu(r"""Usage: `from exudyn.cMath import sin, cos, pi`, or `import exudyn.cMath as cm` and `cm.sin(x)`. The functions are:""")

#the compiled submodule; 'm' is the submodule in this block, so that the declarations below bind into it
cMathModule.CppCode('        {\n        py::module cMathSubmodule = m.def_submodule("_cMath", "the compiled functions of exudyn.cMath");\n'
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

for (name, cName, description) in functions1:
    cMathModule.CppCode(scalarOverload.format(name=name, arguments='Real x', call=cName + '(x)', pyArgs='py::arg("x")'))
    cMathModule.DefPyFunctionAccess(cClass='', pyName=name, cName='py::vectorize([](Real x) { return ' + cName + '(x); })',
                                    isLambdaFunction=True,
                                    description=description + '; calls ' + cName + ' of the C++ code',
                                    argList=['x'],
                                    argTypes=[argumentType],
                                    returnType=returnType,
                                    )

for (name, cName, description) in functions2:
    (a0, a1) = ('y', 'x') if name == 'atan2' else ('x', 'y')
    cMathModule.CppCode(scalarOverload.format(name=name, arguments='Real ' + a0 + ', Real ' + a1, call=cName + '(' + a0 + ', ' + a1 + ')',
                                              pyArgs='py::arg("' + a0 + '"), py::arg("' + a1 + '")'))
    cMathModule.DefPyFunctionAccess(cClass='', pyName=name, cName='py::vectorize([](Real ' + a0 + ', Real ' + a1 + ') { return ' + cName + '(' + a0 + ', ' + a1 + '); })',
                                    isLambdaFunction=True,
                                    description=description + '; calls ' + cName + ' of the C++ code',
                                    argList=[a0, a1],
                                    argTypes=[argumentType, argumentType],
                                    returnType=returnType,
                                    )

cMathModule.CppCode('        }\n        }\n')

cMathModule.DefDataAccess('pi', 'the float closest to pi, the same as math.pi; defined in python/exudyn/cMath.py',
                          dataType='float', isTopLevel=True)
cMathModule.DefFinishTable()
