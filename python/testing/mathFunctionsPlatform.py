#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  The functions of the math library (sin, atan2, pow, ...) as the compiled module of Exudyn calls them, on a
#           fixed set of arguments: which results are not correctly rounded on this platform, and which differ from
#           another platform. glibc, the MSVC UCRT and Apple's libm guarantee about one unit in the last place (ULP) and
#           round some results differently; sqrt is correctly rounded everywhere and is the control (#2947).
#
#           Run on each platform, then compare the files:
#             python mathFunctionsPlatform.py                       #this platform: not correctly rounded, needs mpmath
#             python mathFunctionsPlatform.py --implementation stdNoFMA3
#             python mathFunctionsPlatform.py --compare a.npz b.npz #where two platforms differ
#           The arguments come from numpy's PCG64 with a fixed seed and are the same on every platform. The values go
#           to python/logs/tmp/mathFunctions_<platform>_<module>_<implementation>.npz (or --out); EXUDYN_MODULE=fast
#           evaluates in exudynCPPfast.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-10 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or
#           modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import math
import os
import platform
import sys

import numpy as np

twoArguments = ['atan2', 'pow', 'fmod']

def Arguments(name, count):
    """the arguments of function name: count values per range, the ranges the code of Exudyn uses the function in, and
    some values where implementations are known to round differently; returns (x, y), y empty for one argument.
    The arguments must be the same bits on every platform: they are made of random integers with single operations of
    numpy (each rounded by IEEE 754) - not with numpy's power or rng.uniform, whose arithmetic may differ"""
    rng = np.random.Generator(np.random.PCG64(2947))
    def Unit(n):                                                #[0, 1), exact: an integer of 53 bits times 2^-53
        return np.ldexp(rng.integers(0, 2**53, n, dtype=np.int64).astype(np.float64), -53)
    def U(a, b, n=count):                                       #uniform in [a, b)
        return np.add(a, np.multiply(b - a, Unit(n)))
    def L(a, b, n=count):                                       #log-uniform in about [10^a, 10^b]: (1+u)*2^e
        (e0, e1) = (int(np.floor(a*3.321928)), int(np.ceil(b*3.321928)))
        return np.ldexp(np.add(1., Unit(n)), rng.integers(e0, e1, n))
    pi = np.pi
    y = np.zeros(0)
    if name in ['sin', 'cos', 'tan']:
        x = np.concatenate([U(-pi, pi), U(-2*pi, 2*pi), U(-1e-3, 1e-3), U(-100, 100), U(-1e5, 1e5)])
    elif name in ['asin', 'acos']:
        x = np.concatenate([U(-1, 1), U(0.99, 1), U(-1, -0.99), U(-1e-3, 1e-3)])
    elif name == 'atan':
        x = np.concatenate([U(-1, 1), U(-10, 10), L(-8, 8), -L(-8, 8)])
    elif name == 'atan2':
        x = np.concatenate([U(-1, 1), U(-1, 1), U(-1e-6, 1e-6), U(-100, 100)])
        y = np.concatenate([U(-1, 1), U(0.5, 1), U(0.5, 1), U(-100, 100)])
    elif name == 'exp':
        x = np.concatenate([U(-1, 1), U(-20, 20), U(-700, 700), U(-1e-6, 1e-6)])
    elif name == 'log':
        x = np.concatenate([U(0.5, 2), L(-10, 10), U(1-1e-6, 1+1e-6)])
    elif name == 'pow':
        #contact: penetration**1.5 (Hertz), exponents of regularized friction; integer and half exponents
        base = np.concatenate([L(-10, 1), U(0, 10), L(-10, 1), U(0, 10)])
        exponent = np.concatenate([U(-3, 3), U(-3, 3), rng.choice([0.5, 1.5, 2., 3., -1., -0.5, 2.5], count),
                                   rng.choice([0.5, 1.5, 2., 3., -1., -0.5, 2.5], count)])
        x, y = base, exponent
    elif name in ['sinh', 'cosh', 'tanh']:
        x = np.concatenate([U(-1, 1), U(-5, 5), U(-30, 30), U(-1e-4, 1e-4)])
    elif name == 'cbrt':
        x = np.concatenate([U(-10, 10), L(-10, 10), -L(-10, 10)])
    elif name == 'fmod':
        x = np.concatenate([U(-100, 100), U(-1e4, 1e4)])
        y = np.concatenate([np.full(count, 2*pi), U(0.1, 10)])
    elif name == 'sqrt':
        x = np.concatenate([U(0, 1), U(0, 100), L(-10, 10)])
    else:
        raise ValueError('unknown function ' + name)
    return (x, y)

#mpmath functions, evaluated with 120 bits and rounded to the nearest double
def CorrectlyRounded(name, x, y):
    import mpmath
    mpmath.mp.prec = 120
    f = {'sin': mpmath.sin, 'cos': mpmath.cos, 'tan': mpmath.tan, 'asin': mpmath.asin, 'acos': mpmath.acos,
         'atan': mpmath.atan, 'exp': mpmath.exp, 'log': mpmath.log, 'sinh': mpmath.sinh, 'cosh': mpmath.cosh,
         'tanh': mpmath.tanh, 'sqrt': mpmath.sqrt,
         'cbrt': lambda a: mpmath.cbrt(a) if a >= 0 else -mpmath.cbrt(-a)}      #mpmath.cbrt is complex for a < 0
    result = np.empty(len(x))
    for i in range(len(x)):
        a = mpmath.mpf(float(x[i]))
        if name == 'atan2':
            value = mpmath.atan2(a, mpmath.mpf(float(y[i])))
        elif name == 'pow':
            value = mpmath.power(a, mpmath.mpf(float(y[i])))
        elif name == 'fmod':
            value = math.fmod(float(x[i]), float(y[i]))      #exact by IEEE 754 on every platform; mpmath.fmod rounds differently
        else:
            value = f[name](a)
        result[i] = float(value)        #mpf -> float rounds to nearest
    return result

def UlpDistance(a, b):
    """the number of doubles between a and b, elementwise; equal values give 0, also +0 and -0"""
    def Ordered(v):
        i = np.ascontiguousarray(v, dtype=np.float64).view(np.int64)
        return np.where(i < 0, np.int64(-0x8000000000000000) - i, i)
    return np.abs(Ordered(a) - Ordered(b))

def PlatformTag():
    return sys.platform + '-' + platform.machine()

def Evaluate(names, count, implementation):
    import exudyn as exu
    values = {}
    for name in names:
        (x, y) = Arguments(name, count)
        (v, seconds) = exu.experimental.EvaluateMathFunction(name, x, y, implementation)
        values[name] = np.asarray(v)
        values['_x_' + name] = x
        values['_y_' + name] = y
    return values

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
if __name__ == '__main__':
    import exudyn as exu
    names = ['sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'exp', 'log', 'pow', 'sinh', 'cosh', 'tanh',
             'cbrt', 'fmod', 'sqrt']
    parser = argparse.ArgumentParser(description='the math library of the compiled module on this platform (#2947)')
    parser.add_argument('--implementation', default='std', help='std (default), stdLoop, portable or stdNoFMA3 (Windows x64)')
    parser.add_argument('--count', type=int, default=5000, help='arguments per range (default 5000)')
    parser.add_argument('--functions', default=','.join(names), help='comma separated, default all')
    parser.add_argument('--out', default='', help='the file the values go to (.npz)')
    parser.add_argument('--no-reference', action='store_true', help='do not compare with mpmath')
    parser.add_argument('--compare', nargs=2, metavar='FILE', help='compare two files of this script, nothing else')
    parser.add_argument('--timing', action='store_true', help='the time per evaluation of std, portable and exudyn (what the C++ code calls), nothing else')
    args = parser.parse_args()

    if args.timing:
        #the fastest of five runs of 20 repetitions each, in ns per evaluation; the arguments are those of the check
        print('ns per evaluation, ' + exu._compiledModuleName + ', ' + PlatformTag())
        print('%-7s %9s %9s %9s %7s' % ('', 'std', 'portable', 'exudyn', 'ratio'))
        for name in args.functions.split(','):
            (x, y) = Arguments(name, args.count)
            t = {}
            for implementation in ['std', 'portable', 'exudyn']:
                t[implementation] = min(exu.experimental.EvaluateMathFunction(name, x, y, implementation, 20)[1]
                                        for i in range(5)) * 1e9
            print('%-7s %9.2f %9.2f %9.2f %7.2f' % (name, t['std'], t['portable'], t['exudyn'], t['portable'] / t['std']))
        sys.exit(0)

    if args.compare:
        (a, b) = (np.load(args.compare[0]), np.load(args.compare[1]))
        print('compare', args.compare[0], '|', str(a['_info']))
        print('   with', args.compare[1], '|', str(b['_info']))
        print('%-7s %8s %10s %9s' % ('', 'values', 'differ', 'max ULP'))
        for name in [n for n in a.files if not n.startswith('_')]:
            if name not in b.files or len(a[name]) != len(b[name]):
                print('%-7s missing or of another size in the second file' % name)
                continue
            if any(k in a.files and k in b.files and UlpDistance(a[k], b[k]).max(initial=0) != 0
                   for k in ['_x_' + name, '_y_' + name]):
                print('%-7s the arguments differ: not comparable' % name)
                continue
            d = UlpDistance(a[name], b[name])
            print('%-7s %8d %10d %9d' % (name, len(d), np.sum(d != 0), d.max() if len(d) else 0))
        sys.exit(0)

    names = args.functions.split(',')
    info = ('exudyn ' + exu.__version__ + ', module ' + exu._compiledModuleName + ', ' + PlatformTag()
            + ', Python ' + platform.python_version() + ', ' + args.implementation)
    print(info)
    values = Evaluate(names, args.count, args.implementation)

    out = args.out
    if not out:
        folder = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'logs', 'tmp')
        os.makedirs(folder, exist_ok=True)
        out = os.path.join(folder, 'mathFunctions_' + PlatformTag() + '_' + exu._compiledModuleName + '_'
                           + args.implementation + '.npz')
    np.savez_compressed(out, _info=np.array(info), **values)
    print('values written to', out)

    if not args.no_reference:
        try:
            import mpmath
        except ImportError:
            print('no mpmath: the correctly rounded values are not computed (pip install mpmath)')
            sys.exit(0)
        print('not correctly rounded, against mpmath (120 bits):')
        print('%-7s %8s %10s %9s' % ('', 'values', 'not CR', 'max ULP'))
        for name in names:
            (x, y) = (values['_x_' + name], values['_y_' + name])
            d = UlpDistance(values[name], CorrectlyRounded(name, x, y))
            print('%-7s %8d %10d %9d' % (name, len(d), np.sum(d != 0), d.max()))
