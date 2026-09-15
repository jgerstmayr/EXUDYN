#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Basic utility functions and constants; they depend on numpy only, not on exudyn.
#
# Author:   Johannes Gerstmayr
# Date:     2020-03-10 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# Notes:    Additional constants are defined: \\
#           pi = 3.1415926535897932 \\
#           sqrt2 = 2**0.5\\
#           g=9.81\\
#           Two variables 'gaussIntegrationPoints' and 'gaussIntegrationWeights' define integration points and weights for function GaussIntegrate(...)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import math #always available in Python
import numpy as np

#define some constants which would require external libraries
#pi = 3.1415926535897932 #define pi in order to avoid importing large libraries; identical to from math import pi
pi = math.pi
sqrt2 = 2.**0.5
g = 9.81 #gravity constant


def ClearWorkspace():
    r"""clear all workspace variables except for system variables with '_' at beginning,
    'func' or 'module' in name; it also deletes all items in exudyn.sys and exudyn.variables,
    EXCEPT from exudyn.sys['renderState'] for pertaining the previous view of the renderer

    Note:
        Use this function with CARE! In Spyder, it is certainly safer to add the preference Run$\ra$'remove all variables before execution'. It is recommended to call ClearWorkspace() at the very beginning of your models, to avoid that variables still exist from previous computations which may destroy repeatability of results

    Example:
        import exudyn as exu
        import exudyn.utilities
        #clear workspace at the very beginning, before loading other modules and potentially destroying unwanted things ...
        ClearWorkspace()       #cleanup
        #now continue with other code
        from exudyn.itemInterface import *
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        ...
    """
    #if __name__ == "__main__":  #this won't work as the function is not running in __main__, but in exudyn.basicUtilities
    gl = globals().copy()

    for var in gl:
        if var[0] == '_': continue
        if 'func' in str(globals()[var]): continue
        if 'module' in str(globals()[var]): continue
        del globals()[var]

    import inspect
    fglobals = inspect.stack()[1][0].f_globals
    gl2 = fglobals.copy() #these are the globals of the caller
    for var in gl2:
        if var[0] == '_': continue
        if 'func' in str(fglobals[var]): continue
        if 'module' in str(fglobals[var]): continue

        del fglobals[var]


    import sys
    if 'exudyn' in sys.modules:
        import exudyn #previously, it may have been loaded under another name (e.g., exu)

        sysCopy = exudyn.sys.copy()
        for (key,value) in sysCopy.items():
            if (#key != 'currentRendererSystemContainer' and
                key != 'renderState'):
                del exudyn.sys[key]
        variablesCopy = exudyn.variables.copy()
        for (key,value) in variablesCopy.items():
            del exudyn.variables[key]

    if 'matplotlib' in sys.modules: #if already imported, we check if there are open figures (which would be lost otherwise)
        import matplotlib.pyplot as plt
        plt.close('all')

def SmartRound2String(x, prec=3):
    """round to max number of digits; may give more digits if this is shorter; using in general the format() with '.g' option, but keeping decimal point and using exponent where necessary
    """
    s = ("{:.0"+str(prec)+"g}").format(x)
    if abs(x) > 1 and x != int(x) and '.' not in s and 'e' not in s:
        s = s+'.'
    if x == int(x) and len(s) > len(str(x)):
        s = str(x)
    return s
        


def Normalize(v):
    """take a vector and return it normalized to L2-norm 1; a zero vector is returned as zero vector

    Args:
        vector v as list or in numpy format

    Returns:
        list: v multiplied with a scalar such that its L2-norm is 1, or the zero vector; a list, as
        callers append the result to lists of normals
    """
    v = np.array(v, dtype=float)
    norm = np.linalg.norm(v)
    if norm != 0:
        v /= norm
    return v.tolist()

#integration points per integration order (1, 3, ...); for interval [-1,1]
gaussIntegrationPoints=[[0],
                        [-(1. / 3.)**0.5, (1. / 3.)**0.5],
                        [-(3. / 5.)**0.5, 0., (3. / 5.)**0.5],
                        [-(3. / 7. + (120.)**0.5 / 35.)**0.5, -(3. / 7. - (120.)**0.5 / 35.)**0.5, (3. / 7. - (120.)**0.5 / 35.)**0.5, (3. / 7. + (120.)**0.5 / 35.)**0.5],
                        [-0.906179845938664, -0.5384693101056831, 0., 0.5384693101056831, 0.906179845938664],
                        ]

#integration weights per integration order (1, 3, ...); for interval [-1,1]
gaussIntegrationWeights=[[2],
                         [1., 1.],
                         [5. / 9., 8. / 9., 5. / 9.],
                         [1. / 2. - 5. / (3.*(120.)**0.5), 1. / 2. + 5. / (3.*(120.)**0.5), 1. / 2. + 5. / (3.*(120.)**0.5), 1. / 2. - 5. / (3.*(120.)**0.5)],
                         [0.23692688505618914, 0.47862867049936636, 0.5688888888888889, 0.47862867049936636, 0.23692688505618914],
                         ]

def GaussIntegrate(functionOfX, integrationOrder, a, b):
    """compute numerical integration of functionOfX in interval [a,b] using Gaussian integration

    Args:
        functionOfX: scalar, vector or matrix-valued function with scalar argument (X or other variable)
        integrationOrder: odd number in {1,3,5,7,9}; currently maximum order is 9
        a: integration range start
        b: integration range end

    Returns:
        (scalar or vectorized) integral value
    """
    cnt = 0
    value = 0*functionOfX(0) #initialize value with correct shape
    if integrationOrder > 9:
        raise ValueError("GaussIntegrate: maximum implemented integration order is 9!")
    if integrationOrder%2 != 1 or integrationOrder < 1:
        raise ValueError("GaussIntegrate: integration order must be odd (1,3,5,...) and > 0")
    
    points = gaussIntegrationPoints[int(integrationOrder/2)]
    weights = gaussIntegrationWeights[int(integrationOrder/2)]
    
    for p in points:
        x = 0.5*(b - a)*p + 0.5*(b + a)
        value += 0.5*(b - a)*weights[cnt]*functionOfX(x);
        cnt += 1

    return value


#integration points per integration order (1, 3, ...); for interval [-1,1]
lobattoIntegrationPoints=[[-1.,1.],
                          [-1., 0., 1.],
                          [-1., -(1./5.)**0.5, (1./5.)**0.5, 1.]]

#integration weights per integration order (1, 3, ...); for interval [-1,1]
lobattoIntegrationWeights=[[ 1., 1.],
                           [ 1./3., 4./3., 1./3.],
                           [ 1./6., 5./6., 5./6., 1./6.]]

def LobattoIntegrate(functionOfX, integrationOrder, a, b):
    """compute numerical integration of functionOfX in interval [a,b] using Lobatto integration

    Args:
        functionOfX: scalar, vector or matrix-valued function with scalar argument (X or other variable)
        integrationOrder: odd number in {1,3,5}; currently maximum order is 5
        a: integration range start
        b: integration range end

    Returns:
        (scalar or vectorized) integral value
    """
    cnt = 0
    value = 0*functionOfX(0) #initialize value with correct shape
    if integrationOrder > 5:
        raise ValueError("LobattoIntegrate: maximum implemented integration order is 5!")
    if integrationOrder%2 != 1 or integrationOrder < 1:
        raise ValueError("LobattoIntegrate: integration order must be odd (1,3,5,...) and >= 1")
    
    points = lobattoIntegrationPoints[int(integrationOrder/2)]
    weights = lobattoIntegrationWeights[int(integrationOrder/2)]
    
    for p in points:
        x = 0.5*(b - a)*p + 0.5*(b + a)
        value += 0.5*(b - a)*weights[cnt]*functionOfX(x);
        cnt += 1

    return value







