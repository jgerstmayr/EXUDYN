#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The linear algebra micro-benchmark that every build carries, exudyn.special.RunLinalgBenchmark (#3410):
#           it runs with a short minimum time, single- and multithreaded, and returns one entry per operation and
#           size with a positive time per call and a finite check sum; the multithreaded entries are those of
#           ResizableVectorParallel at and above the threading limit; the groups and the libraries (Exudyn, Eigen as
#           the reference, the prototype of compile-time size) are all present. The times themselves are not checked.
#
# Usage:    pytest python/testing/test_linalgBenchmark.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-11
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import math

import exudyn as exu


def test_singleThreaded():
    results = exu.special.RunLinalgBenchmark(minimumTime=1e-4)
    operations = set(r['operation'] for r in results)
    for name in ['Vector +=', 'Vector MultAdd', 'ResizableVectorParallel MultAdd', 'Matrix3D * Matrix3D',
                 'ConstSizeMatrix<36> * ConstSizeVector<6>', 'HT inverse, rotation', 'Matrix * Vector',
                 'J^T K J, ResizableMatrix', '(J^T K) J into a block', 'dense factorize (InvertSpecial)', 'AddSubmatrix + FinalizeMatrix (setFromTriplets)']:
        assert name in operations
    assert set(r['group'] for r in results) == {'vector', 'small', 'temporaries', 'jacobian', 'connector', 'autodiff', 'dense', 'assembly'}
    assert set(r['library'] for r in results) == {'Exudyn', 'Eigen', 'prototype'}
    for r in results:
        assert set(r.keys()) == {'group', 'library', 'operation', 'size', 'threads', 'nsPerCall', 'checkSum'}
        assert r['threads'] == 1
        assert r['size'] >= 1
        assert r['nsPerCall'] > 0
        assert math.isfinite(r['checkSum'])
    #the sweep of the vector sizes reaches across the threading limit
    sizes = sorted(set(r['size'] for r in results if r['operation'] == 'Vector +='))
    assert sizes[0] == 3 and sizes[-1] >= 100000


def test_multiThreaded():
    results = exu.special.RunLinalgBenchmark(minimumTime=1e-4, numberOfThreads=2)
    threaded = [r for r in results if r['threads'] > 1]
    assert len(threaded) > 0
    limit = min(r['size'] for r in threaded)
    for r in threaded:
        assert r['operation'].startswith('ResizableVectorParallel')
        assert r['threads'] == 2
    #the same operations single-threaded for each size with threads
    single = set((r['operation'], r['size']) for r in results if r['threads'] == 1)
    for r in threaded:
        assert (r['operation'], r['size']) in single
    assert limit >= 1000
    #the threads are released: a following run is single-threaded again
    assert all(r['threads'] == 1 for r in exu.special.RunLinalgBenchmark(minimumTime=1e-4))
