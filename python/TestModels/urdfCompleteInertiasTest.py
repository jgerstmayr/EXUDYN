#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  urdfImport.CompleteInertiasFromMeshes (#3357) on links of the form of GetURDFrobotData, with the mesh of a brick, whose inertia is known: a mass without inertia, no mass at all, an
#           inertia violating the triangle inequality, a correct link which stays as it is, a link without meshes which is reported. The result is the sum of the checks and estimated values.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-09
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
from exudyn.robotics.urdfImport import CompleteInertiasFromMeshes
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)
checks = []

size = np.array([0.1, 0.2, 0.4])
center = np.array([0.3, 0., 0.])
mesh = graphics.Brick(centerPoint=center, size=size)
volume = size[0]*size[1]*size[2]
def BoxInertia(mass):
    return mass/12.*np.diag([size[1]**2 + size[2]**2, size[0]**2 + size[2]**2, size[0]**2 + size[1]**2])

def Link(name, mass, inertia, graphicsDataList):
    return {'name': name, 'mass': mass, 'com': np.zeros(3), 'inertiaCOM': np.array(inertia, dtype=float), 'graphicsDataList': graphicsDataList, 'inertiaProblems': []}

correct = np.diag([1., 1.2, 1.5])
violating = np.diag([0.1, 0.2, 1.])
data = {'linkList': [Link('massOnly', 2., np.zeros((3, 3)), [mesh]), Link('nothing', 0., np.zeros((3, 3)), [mesh]), Link('triangle', 1., violating, [mesh]),
                     Link('correct', 3., correct, [mesh]), Link('noMesh', 0., np.zeros((3, 3)), [])]}
result = CompleteInertiasFromMeshes(data, density=800., verbose=0)
links = result['linkList']

checks += [np.allclose(links[0]['inertiaCOM'], BoxInertia(2.), atol=1e-12), links[0]['mass'] == 2., np.allclose(links[0]['com'], 0.), 'corrected' in links[0]['inertiaProblems'][0]]
checks += [abs(links[1]['mass'] - 800.*volume) < 1e-10, np.allclose(links[1]['com'], center), np.allclose(links[1]['inertiaCOM'], BoxInertia(800.*volume), atol=1e-12)]
moments = np.linalg.eigvalsh(links[2]['inertiaCOM'])
checks += [abs(moments[0] + moments[1] - moments[2]) < 1e-12, abs(moments[2] - 1.) < 1e-12, 'corrected' in links[2]['inertiaProblems'][0]]
checks += [np.array_equal(links[3]['inertiaCOM'], correct), links[3]['inertiaProblems'] == []]
checks += [links[4]['mass'] == 0., len(links[4]['inertiaProblems']) == 1, 'corrected' not in links[4]['inertiaProblems'][0]]
checks += [data['linkList'][0]['mass'] == 2. and np.max(np.abs(data['linkList'][0]['inertiaCOM'])) == 0.] #the given dict is not changed

u = sum(int(c) for c in checks) + links[1]['mass'] + links[0]['inertiaCOM'][0, 0]
exu.Print('checks:', checks)
exu.Print('solution of urdfCompleteInertiasTest=', u)
exu.sys['testResult'] = u
