#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Functions for symbolic user functions that are neither kinematics, user items nor system models (#3175):
#           SampleTime, the discrete time of a sampling period, and Noise, a random signal of the time as an
#           exudyn.symbolic.LookupTable of random values. Both compute with numbers and with exudyn.symbolic
#           expressions, so that a user function that uses them computes numbers and is recorded.
#           A function of the time held between the samples is f(SampleTime(t, period)); holding a state - read at
#           the sampling instants, kept in between - needs data coordinates and is not done here.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-01 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import exudyn

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = ['SampleTime', 'NoiseTable', 'Noise']

#the fraction of the period by which a time just below a sampling instant, by rounding, still counts as the instant
_sampleTolerance = 1e-12


def SampleTime(t, period, offset=0., functions=exudyn.symbolic):
    """the discrete time of the samples at offset + k*period: the last sampling instant at or before t, offset +
    period*floor((t - offset)/period), with a tolerance of 1e-12 period, so that a time computed as k*h with a step
    size h that divides the period counts as its sampling instant

    Args:
        t: the time, a number or an exudyn.symbolic.Real
        period: the sampling period, positive
        offset: the time of the first sample
        functions: the module of floor: exudyn.symbolic (default), which takes numbers and expressions, or numpy

    Returns:
        the sampling instant, a number or an exudyn.symbolic.Real

    Example:
        def UFload(mbs, t, load):  #a load that changes at the samples of 0.01 s only
            return load*esym.sin(10*SampleTime(t, 0.01))
    """
    if not period > 0.:
        raise ValueError('SampleTime: the period must be positive, not ' + str(period))
    return offset + period*functions.floor((t - offset)/period + _sampleTolerance)


#the tables of NoiseTable by their arguments, created once
_noiseTables = {}


def _NoiseTable(step, length, seed, amplitude, distribution, outside):
    """the table of NoiseTable, created"""
    n = int(round(length/step))
    if n < 1:
        raise ValueError('Noise: length/step must give at least one value, not ' + str(length) + '/' + str(step))
    rng = np.random.default_rng(seed)
    if distribution == 'uniform':
        values = rng.uniform(-amplitude, amplitude, n)
    elif distribution == 'normal':
        values = rng.normal(0., amplitude, n)
    else:
        raise ValueError("Noise: distribution must be 'uniform' or 'normal', not " + repr(distribution))
    return exudyn.symbolic.LookupTable(list(values), step, 0., outside)


def NoiseTable(step, length, seed=0, amplitude=1., distribution='uniform', outside='periodic'):
    """the exudyn.symbolic.LookupTable of Noise: round(length/step) random values of numpy.random.default_rng(seed) at
    the times i*step, linearly interpolated; the same values for the same arguments, in every run, step size, Newton
    iteration and order of evaluation; created once for each set of arguments

    Args:
        step: the time between two random values
        length: the length of the table in time; 'periodic' repeats it after length
        seed: the seed of the random generator
        amplitude: 'uniform': the values are in [-amplitude, amplitude]; 'normal': their standard deviation
        distribution: 'uniform' or 'normal'
        outside: as exudyn.symbolic.LookupTable: 'periodic' (default), 'zero' or 'hold'

    Returns:
        the exudyn.symbolic.LookupTable; its memory, one number per value, is the cost of the noise
    """
    key = (float(step), float(length), int(seed), float(amplitude), distribution, outside)
    if key not in _noiseTables:
        _noiseTables[key] = _NoiseTable(*key)
    return _noiseTables[key]


def Noise(t, step, length, seed=0, amplitude=1., distribution='uniform', outside='periodic'):
    """a random signal of the time t, the value of NoiseTable(step, length, seed, amplitude, distribution, outside) at t:
    repeatable for each time, continuous and piecewise linear between the random values; held between samples by
    Noise(SampleTime(t, period), ...)

    Args:
        t: the time, a number or an exudyn.symbolic.Real
        step: the time between two random values
        length: the length of the table in time
        seed: the seed of the random generator
        amplitude: 'uniform': the values are in [-amplitude, amplitude]; 'normal': their standard deviation
        distribution: 'uniform' or 'normal'
        outside: 'periodic' (default), 'zero' or 'hold'

    Returns:
        the value, a number or an exudyn.symbolic.Real

    Example:
        def UFforce(mbs, t, load):  #a disturbance of 0.5 N, a new random value every 0.01 s
            return load + Noise(t, 0.01, 10., seed=1, amplitude=0.5)
    """
    return NoiseTable(step, length, seed, amplitude, distribution, outside)(t)
