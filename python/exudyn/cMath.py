#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  exudyn.cMath: the math functions the C++ code of Exudyn computes with (sin, cos, ..., EM::Sin in
#           C++), which give the same bits on every platform, for a float or a numpy array. The functions are
#           compiled (the submodule exudyn._cMath, declared in definitions/pybindCMath.py, which also writes
#           their documentation and cMath.pyi); this module takes them and adds pi. Documented in the
#           Python-C++ command interface, page exudyn.cMath.
#
# Authors:  Johannes Gerstmayr
# Date:     2026-10-10
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import math as _math

from exudyn import _compiledModule

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'exp', 'log', 'pow', 'sinh', 'cosh', 'tanh', 'cbrt',
    'sqrt', 'pi',
    ]

_cMath = _compiledModule._cMath

sin = _cMath.sin
cos = _cMath.cos
tan = _cMath.tan
asin = _cMath.asin
acos = _cMath.acos
atan = _cMath.atan
atan2 = _cMath.atan2
exp = _cMath.exp
log = _cMath.log
pow = _cMath.pow
sinh = _cMath.sinh
cosh = _cMath.cosh
tanh = _cMath.tanh
cbrt = _cMath.cbrt
sqrt = _cMath.sqrt

pi = _math.pi
