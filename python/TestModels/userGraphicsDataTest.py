#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The graphics data of a user body (#3064): VObjectUserBody(graphicsData=...) is given in local coordinates,
#           and every point is mapped by the positionUserFunction in the visualization configuration. (1) A rigid
#           body as user body draws a brick, a sphere and lines where ObjectRigidBody draws the same graphics data,
#           with the same normals. (2) A bent user ANCF cable element maps a line along its axis and a strip of
#           triangles onto the deformed axis, at the positions the body gives, the normals of the strip computed from
#           the mapped triangles; (3) a symbolic positionUserFunction
#           draws the same; (4) with show=False nothing is drawn.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-23
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.advancedUtilities import CreateSymbolicUserFunction
import exudyn.graphics as graphics
import exudyn.cUtilities as cu
import numpy as np
esym = exu.symbolic

def Rows(graphicsData, kind, itemNumber):
    """the rows of one kind of primitive drawn by an object; items are (system, item type, item number)"""
    return [i for (i, item) in enumerate(graphicsData[kind]['items'])
            if item[1] == int(exu.ItemType.Object) and item[2] == int(itemNumber)]

def ItemPoints(graphicsData, kind, itemNumber):
    """the points of one kind of primitive drawn by an object"""
    return np.array(graphicsData[kind]['points'])[Rows(graphicsData, kind, itemNumber)]

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(1) a rigid body as user body against ObjectRigidBody, with the same graphics data
bodyGraphics = [graphics.Brick(centerPoint=[0.2, 0, 0], size=[0.4, 0.1, 0.2], color=graphics.color.steelblue),
                graphics.Sphere(point=[0.4, 0, 0], radius=0.05, nTiles=8),
                graphics.Lines([[0, 0, 0], [0.4, 0, 0], [0.4, 0.2, 0]])]

def UFpositionRigid(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return qTotal[0:3] + cu.RotXYZ2RotationMatrix(qTotal[3:6]) @ localPosition

SC = exu.SystemContainer()
mbs = SC.AddSystem()
reference = [0.5, 0.2, 0.1, 0.3, -0.2, 0.7]
bodies = []
for isUser in [False, True]:
    node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=reference))
    if isUser:
        bodies += [mbs.AddObject(ObjectUserBody(nodeNumbers=[node], massMatrixUserFunction=lambda mbs, t, i, qRef, qTotal, q_t, parameters: np.eye(6),
                                                positionUserFunction=UFpositionRigid,
                                                visualization=VObjectUserBody(graphicsData=bodyGraphics)))]
    else:
        bodies += [mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=1, inertia=[1, 1, 1, 0, 0, 0],
                                                 visualization=VObjectRigidBody(graphicsData=bodyGraphics)))]
mbs.Assemble()
drawn = SC.renderer.GetGraphicsData()
differenceRigid = 0
for kind in ['triangles', 'lines', 'spheres']:
    points = [ItemPoints(drawn, kind, body).reshape(-1, 3) for body in bodies]
    differenceRigid = max(differenceRigid, np.max(np.abs(points[1] - points[0])))
normals = [np.array(drawn['triangles']['normals'])[Rows(drawn, 'triangles', body)] for body in bodies]
differenceNormals = np.max(np.abs(normals[1] - normals[0]))
exu.Print('rigid body: largest difference of the points to ObjectRigidBody', differenceRigid, ', of the normals', differenceNormals)

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(2) one user ANCF cable element, bent by its initial coordinates: a line and a strip along the axis
Le = 1
def S(x):
    xi = x/Le
    return np.kron([1 - 3*xi**2 + 2*xi**3, x - 2*x**2/Le + x*xi**2, 3*xi**2 - 2*xi**3, -x*xi + x*xi**2], np.eye(2))

def UFpositionCable(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return list(S(localPosition[0]) @ qTotal) + [localPosition[2]]

def UFpositionCableSymbolic(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    xi = localPosition[0]/Le
    N = [1 - 3*xi**2 + 2*xi**3, localPosition[0] - 2*localPosition[0]**2/Le + localPosition[0]*xi**2,
         3*xi**2 - 2*xi**3, -localPosition[0]*xi + localPosition[0]*xi**2]
    return [N[0]*qTotal[0] + N[1]*qTotal[2] + N[2]*qTotal[4] + N[3]*qTotal[6],
            N[0]*qTotal[1] + N[1]*qTotal[3] + N[2]*qTotal[5] + N[3]*qTotal[7], localPosition[2]]

nSegments = 8
xAxis = np.linspace(0, Le, nSegments + 1)
axis = graphics.Lines([[x, 0, 0] for x in xAxis])
strip = {'type': 'TriangleList', 'points': [c for x in xAxis for c in [x, 0, -0.05, x, 0, 0.05]],
         'colors': [0.2, 0.6, 0.2, 1]*(2*(nSegments + 1)),
         'triangles': [j for i in range(nSegments) for j in [2*i, 2*i + 2, 2*i + 1, 2*i + 1, 2*i + 2, 2*i + 3]]}

def CableModel(kind, show=True):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    #bent: the second node lower and turned
    nodes = [mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[0, 0, 1, 0])),
             mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[Le, 0, 1, 0], initialCoordinates=[-0.1, -0.3, -0.05, -0.6]))]
    body = mbs.AddObject(ObjectUserBody(nodeNumbers=nodes, massMatrixUserFunction=lambda mbs, t, i, qRef, qTotal, q_t, parameters: np.eye(8),
                                        positionUserFunction=UFpositionCable,
                                        visualization=VObjectUserBody(show=show, graphicsData=[axis, strip])))
    if kind == 'symbolic':
        mbs.variables['keep'] = CreateSymbolicUserFunction(mbs, UFpositionCableSymbolic, 'positionUserFunction', body)
        mbs.SetObjectParameter(body, 'positionUserFunction', mbs.variables['keep'])
    mbs.Assemble()
    return (SC, mbs, body)

(SC, mbs, body) = CableModel('python')
drawn = SC.renderer.GetGraphicsData()
linePoints = ItemPoints(drawn, 'lines', body).reshape(-1, 3)
expected = np.array([mbs.GetObjectOutputBody(body, exu.OutputVariableType.Position, localPosition=[x, 0, 0],
                                             configuration=exu.ConfigurationType.Visualization) for x in xAxis])
#each segment from point i to i+1
differenceCable = max(np.max(np.abs(linePoints[0::2] - expected[:-1])), np.max(np.abs(linePoints[1::2] - expected[1:])))
#the strip, across the axis in z: the normals of its mapped triangles lie in the plane of bending, normal to the axis
stripNormals = np.array(drawn['triangles']['normals'])[Rows(drawn, 'triangles', body)]
stripPoints = ItemPoints(drawn, 'triangles', body)
tangents = np.array([p[1] - p[0] if abs(p[1][0] - p[0][0]) > 1e-6 else p[2] - p[1] for p in stripPoints])
normalsOff = max(np.max(np.abs(stripNormals[:, :, 2])),
                 np.max(np.abs(np.einsum('tk,tik->ti', tangents, stripNormals))/np.linalg.norm(tangents, axis=1)[:, None]),
                 np.max(np.abs(np.linalg.norm(stripNormals, axis=2) - 1)))
exu.Print('bent cable: largest difference of the line to the positions of the body', differenceCable,
          ', normals of the strip off the plane, the axis or unit length', normalsOff, ', tip', linePoints[-1])

(SCs, mbsS, bodyS) = CableModel('symbolic')
symbolicPoints = ItemPoints(SCs.renderer.GetGraphicsData(), 'lines', bodyS).reshape(-1, 3)
differenceSymbolic = np.max(np.abs(symbolicPoints - linePoints))
exu.Print('symbolic positionUserFunction: largest difference of the drawing', differenceSymbolic)

(SCh, mbsH, bodyH) = CableModel('python', show=False)
hidden = SCh.renderer.GetGraphicsData()
drawnHidden = len(ItemPoints(hidden, 'lines', bodyH)) + len(ItemPoints(hidden, 'triangles', bodyH))
exu.Print('with show=False, primitives drawn:', drawnHidden)

u = round(float(np.sum(linePoints)), 5) #the points are float32: 5 digits
u += int(differenceRigid > 1e-6) + int(differenceNormals > 1e-6) + int(differenceCable > 1e-6) + int(normalsOff > 1e-5)
u += int(differenceSymbolic > 1e-6) + int(drawnHidden != 0)
exu.Print('solution of userGraphicsDataTest=', u)

exu.sys['testResult'] = u
