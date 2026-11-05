#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  graphics.FromSTLfile without numpy-stl (#3264): a brick written as a binary and as a text STL file, read
#           with scale, rotation Aoff and offset pOff and density: the points agree with FromSTLfileASCII, the normals
#           point outward after the inversion, and volume, mass, center of mass and the inertia tensor at the center of
#           mass agree with the formulas of the brick, the inertia rotated by Aoff. The result is the number of checks
#           that pass plus the mass and the trace of the inertia.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import exudyn.graphics as graphics
from exudyn.rigidBodyUtilities import RotationVector2RotationMatrix
import numpy as np
import os
import tempfile

testIsActive = exu.sys.get('testIsActive', False)

def WriteSTL(fileName, graphicsData, binary):
    """a closed mesh as STL, its triangles reversed, as FromSTLfile inverts them by default"""
    points = np.array(graphicsData['points']).reshape((-1, 3))
    triangles = np.array(graphicsData['triangles']).reshape((-1, 3))[:, ::-1]
    corners = points[triangles]
    normals = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
    if binary:
        records = np.zeros(len(corners), dtype=np.dtype([('normal', '<f4', 3), ('points', '<f4', (3, 3)), ('attribute', '<u2')]))
        (records['normal'], records['points']) = (normals, corners)
        with open(fileName, 'wb') as file:
            file.write(b'\0'*80 + np.uint32(len(corners)).tobytes() + records.tobytes())
    else:
        with open(fileName, 'w') as file:
            file.write('solid brick\n')
            for (n, c) in zip(normals, corners):
                file.write('facet normal %g %g %g\n outer loop\n' % tuple(n) + ''.join(' vertex %.9g %.9g %.9g\n' % tuple(p) for p in c)
                           + ' endloop\nendfacet\n')
            file.write('endsolid brick\n')

size = np.array([0.5, 0.25, 0.125]) #the corners exact in the single precision of binary STL
center = np.array([0.125, 0.25, 0.375])
brick = graphics.Brick(centerPoint=center, size=size)
density = 1000.
scale = 2.
A = RotationVector2RotationMatrix([0.3, -0.2, 0.5])
pOff = np.array([1., -2., 0.5])

checks = []
with tempfile.TemporaryDirectory() as directory:
    files = [os.path.join(directory, name) for name in ['brickBinary.stl', 'brickText.stl']]
    WriteSTL(files[0], brick, True)
    WriteSTL(files[1], brick, False)
    results = [graphics.FromSTLfile(fileName, density=density, scale=scale, Aoff=A, pOff=pOff) for fileName in files]
    ascii = graphics.FromSTLfileASCII(files[1])

#the same points from both files and from FromSTLfileASCII (scaled, rotated and moved)
points = [np.array(g['points']).reshape((-1, 3)) for (g, data) in results]
checks += [np.max(np.abs(points[0] - points[1])) < 1e-6]
checks += [np.max(np.abs(points[1] - ((np.array(ascii['points']).reshape((-1, 3))*scale) @ A.T + pOff))) < 1e-6]

#the normals outward: at each triangle, from the center of the brick to the triangle
(gData, data) = results[0]
normals = np.array(gData['normals']).reshape((-1, 3))
c = scale*A @ center + pOff
checks += [all(n @ (p - c) > 0 for (n, p) in zip(normals, points[0]))]
checks += [np.max(np.abs(np.linalg.norm(normals, axis=1) - 1.)) < 1e-12]

#the mass properties of the scaled brick
s = scale*size
mass = density*np.prod(s)
inertia = A @ np.diag(mass/12.*np.array([s[1]**2 + s[2]**2, s[0]**2 + s[2]**2, s[0]**2 + s[1]**2])) @ A.T
exu.Print('volume', data['volume'], 'mass', data['mass'], 'COM', data['COM'])
exu.Print('inertia', data['inertia'])
checks += [abs(data['volume'] - np.prod(s)) < 1e-12, abs(data['mass'] - mass) < 1e-9,
           np.max(np.abs(data['COM'] - c)) < 1e-12, np.max(np.abs(data['inertia'] - inertia)) < 1e-9*mass,
           np.max(np.abs(data['minPos'] - points[0].min(axis=0))) == 0]

u = sum(checks) + data['mass']*1e-3 + np.trace(data['inertia'])*1e-2
exu.Print('checks:', checks)
exu.Print('solution of stlFileImportTest=', u)
exu.sys['testResult'] = u
