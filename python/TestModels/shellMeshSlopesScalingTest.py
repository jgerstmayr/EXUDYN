#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The slope scaling of ShellMesh (#2226): the slopes of a node are unit vectors, scaled in each element by
#           slopesScalingX/Y. ShellMesh gives each element edge the length for which the cubic edge passes through the
#           mapped midpoint of the edge - for a circular arc the best cubic approximation - and on straight edges the
#           length of the derivative of the geometry times the parameter length of an element. On a quarter cylinder of
#           3 x 1 elements, the points between the nodes lie on the cylinder about 1000 times more closely than with
#           the chord lengths, which the element takes when no scaling is given; a flat mesh is the same with both.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-14
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.shells import ShellMesh, SurfaceMap
from exudyn import cMath
import numpy as np

SC = exu.SystemContainer()
OVT = exu.OutputVariableType
R = 1.
Lx = 1.
Ly = 0.5*np.pi*R #the quarter circle as arc length

def RadiusError(mbs, plate, useChord):
    """the largest deviation of points of the reference mid-surface from the cylinder radius; with useChord, the
    elements take the chord lengths between their nodes as slope scaling"""
    if useChord:
        for e in plate.elementNumbers:
            mbs.ModifyObject(e, {'slopesScalingX': [-1., -1., -1., -1.], 'slopesScalingY': [-1., -1., -1., -1.]})
    error = 0.
    for e in plate.elementNumbers:
        for xi in np.linspace(-1, 1, 5):
            for eta in np.linspace(-1, 1, 5):
                p = mbs.GetObjectOutputBody(e, OVT.Position, localPosition=[xi, eta, 0], configuration=exu.ConfigurationType.Reference)
                error = max(error, abs(cMath.norm(p[1:3]) - R)) #the cylinder axis is x
    return error

errors = {}
for useChord in [False, True]:
    mbs = SC.AddSystem()
    plate = ShellMesh(vertices=[[0, 0, 0], [Lx, 0, 0], [Lx, Ly, 0], [0, Ly, 0]], numberOfElementsX=1, numberOfElementsY=3,
                      youngsModulus=2e9, poissonsRatio=0.3, density=1000, thickness=0.01,
                      surfaceMap=SurfaceMap.Cylinder(Lx, Ly, R, phiDeg=90))
    plate.CreateANCFThinPlateElements(mbs)
    mbs.Assemble()
    errors[useChord] = RadiusError(mbs, plate, useChord)
exu.Print('quarter cylinder, 3 elements around: radius error with the scaling of ShellMesh', errors[False],
          ', with the chord lengths', errors[True])

#a flat trapezoidal mesh: the scaling of ShellMesh is the chord length along its straight rows
mbs = SC.AddSystem()
plate = ShellMesh(vertices=[[0, 0, 0], [2, 0, 0], [1.5, 1, 0], [0.2, 1, 0]], numberOfElementsX=2, numberOfElementsY=2,
                  youngsModulus=2e9, poissonsRatio=0.3, density=1000, thickness=0.01)
plate.CreateANCFThinPlateElements(mbs)
mbs.Assemble()
scalings = [np.array(mbs.GetObjectParameter(e, 'slopesScalingX')) for e in plate.elementNumbers]
for e in plate.elementNumbers:
    mbs.ModifyObject(e, {'slopesScalingX': [-1., -1., -1., -1.], 'slopesScalingY': [-1., -1., -1., -1.]})
chords = [np.array(mbs.GetObjectParameter(e, 'slopesScalingX')) for e in plate.elementNumbers]
flatDifference = max(np.abs(s - c).max() for (s, c) in zip(scalings, chords))
exu.Print('flat trapezoid: largest difference of the scaling of ShellMesh to the chord lengths', flatDifference)

checks = [errors[False] < 1e-6, errors[False] < 1e-2*errors[True], flatDifference < 1e-14]
exu.Print('checks:', checks)
u = sum(checks) - len(checks) + errors[False] + errors[True]
exu.Print('solution of shellMeshSlopesScalingTest=', u)

exu.sys['testResult'] = u
