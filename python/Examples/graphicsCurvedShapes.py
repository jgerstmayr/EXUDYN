#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The curved shapes of GraphicsData, for looking at them: 6-node (quadratic) triangles
#           ('triangles6' of a TriangleList) with quadratic edges ('edges3'), quadratic lines
#           (Lines with shape 'quadratic') and spheres (type 'Spheres'), each beside the flat
#           shape of exudyn.graphics it compares to (#2709).
#           Row y=0: cylinder, torus, sphere and vase of 6-node triangles - few, large elements (45 degrees),
#           the renderer splits them when it draws; the cylinder and the vase carry red edges3 on their rims.
#           Row y=-3: graphics.Cylinder, Torus, Sphere, SolidOfRevolution, LinkedCylinders (with a bore) and a hollow
#           Sphere - curved as well, of 6-node
#           triangles (the sphere with edges is a TriangleList of flat triangles); row y=-6: the same shapes as
#           flat triangles (graphics.Triangles6ToTriangles), with their facets.
#           Row y=3: quadratic lines (a circle of 4 segments, a helix), many spheres, a whole graphics.Sphere.
#           At x=12: a rotating rigid body with a curved cylinder, its rims and a quadratic ring.
#           What to look at:
#           - the curved rows are smooth, the flat row shows its facets; the red rims lie on the surfaces;
#           - key V, openGL.advanced.curvedTriangleTilingAngle: 90 shows the coarse elements, 5 a fine split,
#             and the change shows at once; curvedTriangleMaxTiling limits it;
#           - key T (face edges): the edges of the 6-node triangles are curved, not those of the split;
#           - CTRL+R (raytracer): the same shapes, the spheres exact; with transparent faces as well;
#           - SPACE: the body rotates, its surface, rims and ring move together.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#a surface F(u,v) of 6-node triangles: nu x nv quadratic patches, each split into two 6-node triangles;
#normalFunction gives the normals at the nodes, None uses those of the geometry; rims=True adds the
#boundaries v=v0 and v=v1 as quadratic edges
def Surface6(F, uRange, vRange, nu, nv, color, normalFunction=None, rims=False, edgeColor=graphics.color.red):
    us = np.linspace(uRange[0], uRange[1], 2*nu+1)
    vs = np.linspace(vRange[0], vRange[1], 2*nv+1)
    Node = lambda i, j: j*(2*nu+1) + i #node (i,j) of the grid of (2nu+1) x (2nv+1) nodes
    points = [F(u, v) for v in vs for u in us]
    triangles6 = []
    for j in range(nv):
        for i in range(nu):
            (a, b, c, d) = (Node(2*i, 2*j), Node(2*i+2, 2*j), Node(2*i+2, 2*j+2), Node(2*i, 2*j+2))
            (ab, bc, cd, da, center) = (Node(2*i+1, 2*j), Node(2*i+2, 2*j+1), Node(2*i+1, 2*j+2),
                                        Node(2*i, 2*j+1), Node(2*i+1, 2*j+1))
            triangles6 += [[a, b, c, ab, bc, center], [a, c, d, center, cd, da]] #corners, then mid nodes 01, 12, 20
    g = {'type':'TriangleList', 'points':np.array(points).flatten(),
         'colors':np.tile(color, len(points)), 'triangles6':np.array(triangles6).flatten()}
    if normalFunction is not None:
        g['normals'] = np.array([normalFunction(u, v) for v in vs for u in us]).flatten()
    if rims:
        g['edges3'] = np.array([[Node(2*i, j), Node(2*i+2, j), Node(2*i+1, j)]
                                for j in [0, 2*nv] for i in range(nu)]) #end points, then the mid node
        g['edgeColor'] = edgeColor
    return g

r = 0.5
gCurved = []
#cylinder: 8 elements around, 45 degrees each
gCurved += [Surface6(lambda u, v: [r*np.cos(u), r*np.sin(u), v], [0, 2*np.pi], [0, 1.5], 8, 1,
                     graphics.color.steelblue, normalFunction=lambda u, v: [np.cos(u), np.sin(u), 0], rims=True)]
#torus: 8 x 4 elements
(R0, r0) = (0.6, 0.25)
gCurved += [graphics.Move(Surface6(lambda u, v: [(R0+r0*np.cos(v))*np.cos(u), (R0+r0*np.cos(v))*np.sin(u), r0*np.sin(v)],
                                   [0, 2*np.pi], [0, 2*np.pi], 8, 4, graphics.color.orange,
                                   normalFunction=lambda u, v: [np.cos(v)*np.cos(u), np.cos(v)*np.sin(u), np.sin(v)]),
                          [3, 0, 0.5])]
#sphere: 8 elements around, 4 from pole to pole
gCurved += [graphics.Move(Surface6(lambda u, v: [r*np.cos(v)*np.cos(u), r*np.cos(v)*np.sin(u), r*np.sin(v)],
                                   [0, 2*np.pi], [-np.pi/2, np.pi/2], 8, 4, graphics.color.lawngreen,
                                   normalFunction=lambda u, v: [np.cos(v)*np.cos(u), np.cos(v)*np.sin(u), np.sin(v)]),
                          [6, 0, 0.5])]
#vase, a solid of revolution with the radius rV(z), without normals: at each point the mean of the normals of the geometry
rV = lambda z: 0.3 + 0.2*np.sin(2.5*z)
gCurved += [graphics.Move(Surface6(lambda u, v: [rV(v)*np.cos(u), rV(v)*np.sin(u), v], [0, 2*np.pi], [0, 1.5], 8, 3,
                                   graphics.color.dodgerblue, rims=True), [9, 0, 0])]

#the shapes of exudyn.graphics, curved, and flat for comparison
contour = [[z, rV(z)] for z in np.linspace(0, 1.5, 9)]
gFlat = [graphics.Cylinder(pAxis=[0, -3, 0], vAxis=[0, 0, 1.5], radius=r, color=graphics.color.steelblue, nTiles=8, addEdges=True),
         graphics.Torus(point=[3, -3, 0.5], axis=[0, 0, 1], radiusMajor=R0, radiusMinor=r0, color=graphics.color.orange,
                        nTilesMajor=12, nTilesMinor=8),
         graphics.Sphere(point=[6, -3, 0.5], radius=r, color=graphics.color.lawngreen, nTiles=8, addEdges=True), #with edges: a TriangleList
         graphics.SolidOfRevolution(pAxis=[9, -3, 0], vAxis=[0, 0, 1], contour=contour, color=graphics.color.dodgerblue,
                                    nTiles=12, addEdges=True),
         graphics.LinkedCylinders(point0=[12, -3, 0], point1=[13, -3, 0], axisCylinder=[0, 0, 0.5], radius0=0.4,
                                  radius1=0.25, radiusInner0=0.2, nTiles=16, color=graphics.color.steelblue, addEdges=True),
         graphics.Sphere(point=[15, -3, 0.5], radius=r, innerRadius=0.7*r, majorAngleMin=-0.3*np.pi, majorAngleMax=0.2*np.pi,
                         color=graphics.color.orange, nTiles=12, addEdges=True)]
gFlat += [graphics.Move(graphics.Triangles6ToTriangles(g), [0, -3, 0]) for g in gFlat]

#lines, spheres
s = np.sqrt(0.5)
circle = [[r, 0, 0], [r*s, r*s, 0], [0, r, 0], [-r*s, r*s, 0], [-r, 0, 0], [-r*s, -r*s, 0], [0, -r, 0], [r*s, -r*s, 0], [r, 0, 0]]
helix = [[0.4*np.cos(t), 0.4*np.sin(t), 0.1*t] for t in np.linspace(0, 4*np.pi, 17)] #2 turns, 8 quadratic segments
gLines = [graphics.Move(graphics.Lines(circle, color=graphics.color.red, shape='quadratic'), [0, 3, 0.5]),
          graphics.Move(graphics.Lines(helix, color=graphics.color.blue, shape='quadratic'), [3, 3, 0]),
          {'type':'Lines', 'points':[[2.4, 3, 0], [3.6, 3, 0]], 'colors':[graphics.color.black]*2}, #given as rows
          graphics.Spheres(points=[[6+0.3*i, 3+0.3*j, 0.2*(i+j)] for i in range(-2, 3) for j in range(-2, 3)],
                           radii=[0.05+0.02*(i % 4) for i in range(25)],
                           colors=[graphics.colorList[i % 10] for i in range(25)], nTiles=16),
          graphics.Sphere(point=[9, 3, 0.5], radius=r, color=graphics.color.grey, nTiles=16)] #a whole sphere: the type Spheres

oGround = mbs.CreateGround(graphicsDataList=gCurved + gFlat + gLines +
                           [graphics.CheckerBoard(point=[4.5, 0, -0.5], size=16, nTiles=8)])

#a rotating body with a curved cylinder, its rims and a quadratic ring
gBody = [graphics.Move(Surface6(lambda u, v: [0.3*np.cos(u), 0.3*np.sin(u), v], [0, 2*np.pi], [-0.6, 0.6], 3, 1,
                                graphics.color.steelblue, normalFunction=lambda u, v: [np.cos(u), np.sin(u), 0], rims=True),
                       [0, 0, 0], RotationMatrixY(0.5*np.pi)),
         graphics.Lines([[0, 0.6*np.cos(t), 0.6*np.sin(t)] for t in np.linspace(0, 2*np.pi, 7)], color=graphics.color.red,
                        shape='quadratic')]
mbs.CreateRigidBody(inertia=InertiaCylinder(1000, 1.2, 0.3, axis=0), referencePosition=[12, 0, 0.5],
                    initialAngularVelocity=[0, 0, 1], gravity=[0, 0, 0], graphicsDataList=gBody)
mbs.Assemble()

#what is in the graphics data, without a window: the curved shapes as they are, and refined once
data = SC.renderer.GetGraphicsData()
exu.Print('native shapes: triangles6 =', len(data['triangles6']['items']), ', lines3 =', len(data['lines3']['items']),
          ', spheres =', len(data['spheres']['items']))
dataFlat = SC.renderer.GetGraphicsData(flatShapes=True)
exu.Print('refined once:  triangles =', len(dataFlat['triangles']['items']), ', lines =', len(dataFlat['lines']['items']))

SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.openGL.light0.shadow = 0.3
SC.visualizationSettings.openGL.faceEdgesColor = [0, 0, 0, 1]
SC.visualizationSettings.nodes.show = False
SC.visualizationSettings.general.graphicsUpdateInterval = 0.02

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
simulationSettings.timeIntegration.endTime = 1

SC.renderer.Start()
if SC.renderer.IsActive(): #with a window: rotate for a while, in real time
    simulationSettings.timeIntegration.endTime = 200
    simulationSettings.timeIntegration.numberOfSteps = 200000
    simulationSettings.timeIntegration.simulateInRealtime = True
    SC.renderer.DoIdleTasks()
mbs.SolveDynamic(simulationSettings)
SC.renderer.DoIdleTasks()
SC.renderer.Stop() #safely close rendering window!
