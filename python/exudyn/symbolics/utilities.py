#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Functions for symbolic user functions that are neither kinematics, user items nor system models (#3175):
#           SampleTime, the discrete time of a sampling period, and Noise, a random signal of the time as an
#           exudyn.symbolic.LookupTable of random values. Both compute with numbers and with exudyn.symbolic
#           expressions, so that a user function that uses them computes numbers and is recorded.
#           A function of the time held between the samples is f(SampleTime(t, period)); holding a state - read at
#           the sampling instants, kept in between - needs data coordinates and is not done here.
#           SparseMatrix, a matrix of few nonzero entries declared by its triplets: its products are the scalar
#           operations of the entries, so that a recording does not multiply the zeros, with the results and the
#           one-pass derivatives of the dense product to the bit (#3199).
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
__all__ = ['SampleTime', 'NoiseTable', 'Noise', 'SparseMatrix']

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


def _IsSymbolic(value):
    return isinstance(value, (exudyn.symbolic.Real, exudyn.symbolic.Vector, exudyn.symbolic.Matrix))


class SparseMatrix:
    """a matrix of few nonzero entries, numbers or exudyn.symbolic.Real, declared by its triplets (#3199): its pattern
    is fixed when it is made, and its products are the scalar operations of the entries, each row summed in the order
    of the columns as the dense product does - so that a recording does not multiply the zeros, and the results and the
    one-pass derivatives are those of the dense product to the bit wherever no left-out entry would meet an inf or a
    nan; esym.Jacobian, which builds other sums, to their rounding. The
    one-pass derivative, esym.Jacobian, Optimize and the export take them as any expression. For numbers, the results
    are numpy arrays

    Args:
        rows: the number of rows
        columns: the number of columns
        triplets: list of (row, column, value); values of the same row and column are added

    Example:
        A = SparseMatrix(3, 3, [(i, i, 2*k) for i in range(3)] + [(0, 1, -k), (1, 0, -k)])
        f = A @ q  #3 sums of the products of the nonzero entries
    """
    def __init__(self, rows, columns, triplets):
        (self.rows, self.columns) = (int(rows), int(columns))
        entries = {}
        for (i, j, value) in triplets:
            (i, j) = (int(i), int(j))
            if not (0 <= i < self.rows and 0 <= j < self.columns):
                raise ValueError('SparseMatrix: the entry (' + str(i) + ', ' + str(j) + ') is outside the matrix of '
                                 + str(self.rows) + ' x ' + str(self.columns))
            entries[(i, j)] = entries[(i, j)] + value if (i, j) in entries else value
        self._entries = dict(sorted(entries.items()))

    @staticmethod
    def FromMatrix(matrix):
        """the sparse matrix of the entries of a dense one that are not the number 0: a list of rows of numbers and
        exudyn.symbolic.Real, or a numpy array

        Args:
            matrix: the dense matrix, a list of rows or a numpy array

        Returns:
            SparseMatrix
        """
        rows = [list(row) for row in matrix]
        triplets = [(i, j, value) for (i, row) in enumerate(rows) for (j, value) in enumerate(row)
                    if _IsSymbolic(value) or value != 0]
        return SparseMatrix(len(rows), len(rows[0]) if rows else 0, triplets)

    @property
    def shape(self):
        """(rows, columns)"""
        return (self.rows, self.columns)

    @property
    def nnz(self):
        """the number of entries of the pattern"""
        return len(self._entries)

    def Triplets(self):
        """the entries as list of (row, column, value), by rows and columns"""
        return [(i, j, value) for ((i, j), value) in self._entries.items()]

    def __getitem__(self, index):
        (i, j) = index
        return self._entries.get((int(i), int(j)), 0.)

    def _Result(self, components):
        if any(_IsSymbolic(x) for x in components):
            return exudyn.symbolic.Vector(components)
        return np.array(components, dtype=float)

    def __matmul__(self, vector):
        """A v, a vector: per row the sum of the products of its entries in the order of the columns; an empty row is 0"""
        if isinstance(vector, SparseMatrix):
            raise ValueError('SparseMatrix: the product of two sparse matrices is not available; convert one by ToDense()')
        if not isinstance(vector, exudyn.symbolic.Vector) and len(vector) != self.columns: #a symbolic vector may have no size while recorded
            raise ValueError('SparseMatrix: a vector of size ' + str(len(vector)) + ' with a matrix of ' + str(self.columns) + ' columns')
        sums = [None]*self.rows
        for ((i, j), value) in self._entries.items():
            product = value*vector[j]
            sums[i] = product if sums[i] is None else sums[i] + product
        return self._Result([0. if x is None else x for x in sums])

    @property
    def T(self):
        """the transposed matrix, a SparseMatrix of the same entries"""
        return SparseMatrix(self.columns, self.rows, [(j, i, value) for ((i, j), value) in self._entries.items()])

    def __mul__(self, factor):
        """s A, the factor times each entry"""
        return SparseMatrix(self.rows, self.columns, [(i, j, factor*value) for ((i, j), value) in self._entries.items()])

    __rmul__ = __mul__

    def __add__(self, other):
        """A + B, on the union of the patterns"""
        if not isinstance(other, SparseMatrix) or other.shape != self.shape:
            raise ValueError('SparseMatrix: only a SparseMatrix of the same shape is added')
        return SparseMatrix(self.rows, self.columns, self.Triplets() + other.Triplets())

    def ToDense(self):
        """the dense matrix, an exudyn.symbolic.Matrix if an entry is symbolic, else a numpy array"""
        rows = [[self._entries.get((i, j), 0.) for j in range(self.columns)] for i in range(self.rows)]
        if any(_IsSymbolic(value) for value in self._entries.values()):
            return exudyn.symbolic.Matrix(rows)
        return np.array(rows, dtype=float)

    def __repr__(self):
        return 'SparseMatrix(' + str(self.rows) + ' x ' + str(self.columns) + ', ' + str(self.nnz) + ' entries)'
