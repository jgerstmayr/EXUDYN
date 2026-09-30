#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The surface triangles of a mesh imported from NGsolve point outward (#2321), as those of
#           graphics.NGsolveMesh2PointsAndTrigs and of FEMinterface.VolumeToSurfaceElements do: the
#           renderer lights and the raytracer shades a triangle by its orientation. A box meshed by
#           netgen, linear and quadratic; skipped without ngsolve, an optional package.
#
# Usage:    pytest python/testing/test_femSurfaceOrientation.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-30
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import pytest

ngs = pytest.importorskip('ngsolve')
occ = pytest.importorskip('netgen.occ')

import exudyn.graphics as graphics                                          # noqa: E402
from exudyn.FEM import FEMinterface                                         # noqa: E402

boxCenter = np.array([0.5, 0.25, 0.15])


def BoxMesh():
    geo = occ.OCCGeometry(occ.Box(occ.Pnt(0, 0, 0), occ.Pnt(1, 0.5, 0.3)))
    return ngs.Mesh(geo.GenerateMesh(maxh=0.2))


def OutwardFraction(points, trigs):
    """the fraction of triangles whose normal points away from the center of the (convex) box"""
    p = np.array(points)
    t = np.array(trigs)
    n = np.cross(p[t[:, 1]] - p[t[:, 0]], p[t[:, 2]] - p[t[:, 0]])
    c = (p[t[:, 0]] + p[t[:, 1]] + p[t[:, 2]]) / 3
    return np.mean(np.sum(n * (c - boxCenter), axis=1) > 0)


@pytest.mark.parametrize('meshOrder', [1, 2])
def test_importedSurfacePointsOutward(meshOrder):
    fem = FEMinterface()
    fem.ImportMeshFromNGsolve(BoxMesh(), density=1000, youngsModulus=1e8, poissonsRatio=0.3, meshOrder=meshOrder)
    assert OutwardFraction(fem.GetNodePositionsAsArray(), fem.GetSurfaceTriangles()) == 1.


@pytest.mark.parametrize('meshOrder', [1, 2])
def test_graphicsFromNGsolvePointsOutward(meshOrder):
    [points, trigs] = graphics.NGsolveMesh2PointsAndTrigs(mesh=BoxMesh(), meshOrder=meshOrder, addNormals=False)[:2]
    assert OutwardFraction(points, trigs) == 1.


def test_surfaceFromVolumeElementsPointsOutward():
    fem = FEMinterface()
    fem.ImportMeshFromNGsolve(BoxMesh(), density=1000, youngsModulus=1e8, poissonsRatio=0.3, meshOrder=1)
    fem.surface = []
    fem.VolumeToSurfaceElements()
    assert OutwardFraction(fem.GetNodePositionsAsArray(), fem.GetSurfaceTriangles()) == 1.
