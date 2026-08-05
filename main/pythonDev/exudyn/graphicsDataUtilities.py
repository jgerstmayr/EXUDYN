#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Utility functions for visualization, which provides functions for special graphics manipulation, colors, mesh manipulation, etc.;
#           note that specific function for GraphicsData creation now moved into the graphics submodule;
#           includes functionality like mesh manipulation and some helper functions
#
# Author:   Johannes Gerstmayr
# Date:     2020-07-26 (created)
# Modified: 2024-05-10 (moved primitive functions to graphics)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# Notes:    Some useful colors are defined, using RGBA (Red, Green, Blue and Alpha = opacity) channels
#           in the range [0,1], e.g., red = [1,0,0,1].\\
#           Available colors are: color4red, color4green, color4blue, color4cyan, color4magenta, color4yellow, color4orange, color4pink, color4lawngreen, color4violet, color4springgreen, color4dodgerblue, color4grey, color4darkgrey, color4lightgrey, color4lightred, color4lightgreen, color4steelblue, color4brown, color4black, color4darkgrey2, color4lightgrey2, color4white\\
#           Additionally, a list of 16 colors 'color4list' is available, which is intended to be used, e.g., for creating n bodies with different colors
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#import exudyn.basicUtilities as ebu
# from exudyn.rigidBodyUtilities import ComputeOrthonormalBasisVectors, HomogeneousTransformation, \
#                                       HT2rotationMatrix, HT2translation

#constants and fixed structures:
import numpy as np #LoadSolutionFile
import copy as copy #to be able to copy e.g. lists
from math import pi, sin, cos

# color definitions
color4red = [1.,0.,0.,1.]
color4green = [0.,1.,0.,1.]
color4blue = [0.,0.,1.,1.]

color4cyan = [0.,1.,1.,1.]
color4magenta = [1.,0.,1.,1.]
color4yellow = [1.,1.,0.,1.]

color4orange = [1.,0.5,0.,1.]
color4pink = [1.,0.,0.5,1.]
color4lawngreen = [0.5,1.,0.,1.]

color4springgreen = [0.,1.,0.5,1.]
color4violet = [0.5,0.,1.,1.]
color4dodgerblue = [0.,0.5,1.,1.]


color4lightred = [0.9,0.4,0.4,1.]
color4lightgreen = [0.4,0.9,0.4,1.]
color4steelblue = [0.4,0.4,0.9,1.]
color4brown = [0.65,0.2,0.2,1.]

color4black =      [0.,0.,0.,1.]
color4darkgrey =   [0.2,0.2,0.2,1.]
color4darkgrey2 =  [0.35,0.35,0.35,1.]
color4grey =       [0.5,0.5,0.5,1.]
color4lightgrey =  [0.7,0.7,0.7,1.]
color4lightgrey2 = [0.85,0.85,0.85,1.]
color4white =      [1.,1.,1.,1.]

color4default =    [-1.,-1.,-1.,-1.] #indicates that default color is used

#define a list of 16 colors for numbered colors
color4list = [color4red, color4green, color4blue, 
              color4cyan, color4magenta, color4yellow,
              color4orange, color4pink, color4lawngreen,
              color4violet, color4springgreen, color4dodgerblue,
              color4grey, color4darkgrey, color4lightgrey,
              #color4lightred, color4lightgreen, color4steelblue, 
              color4brown]

color4listSize = len(color4list) #maximum number of colors in color4list

# #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# graphicsDataNormalsFactor = 1. #this is a factor being either -1. [original normals pointing inside; until 2022-06-27], while +1. gives corrected normals pointing outside
# graphicsDataSwitchTriangleOrder = False #this is the old ordering of triangles in some Sphere or Cylinder functions, causing computed normals to point inside

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#**function: helper function to switch order of three items in a list; mostly used for reverting normals in triangles
#**input: 3D vector as list or as np.array
#**output: interchanged 2nd and 3rd component of list
def SwitchTripletOrder(vector):
    v=list(vector) #copy, such that vector is not changed
    a = v[2]
    v[2] = v[1]
    v[1] = a
    return v

#**function: compute normalized normal for 3 triangle points
#**input: 3D vector as list or as np.array
#**output: normal as np.array
def ComputeTriangleNormal(p0,p1,p2):
    v0 = np.array(p1) - np.array(p0)
    v1 = np.array(p2) - np.array(p0)

    n = np.cross(v0,v1)
    ln = np.linalg.norm(n)
    if ln != 0.:
        n /= ln
    return n

#**function: compute area of triangle given by 3 points
#**input: 3D vector as list or as np.array
#**output: area as float
def ComputeTriangleArea(p0,p1,p2):
    return 0.5*np.linalg.norm(np.cross(np.array(p1) - np.array(p0), np.array(p2) - np.array(p0)))


#**function: Internal function: compute normals to 6-node triangular surface given by elementNodes
#input: elementNodes given as np.array with 6 node vectors in rows; node ordering must follow Netgen order, see the local coordinates in the function
#**output: returns np.array with 6 normals in rows
def Compute6NodeTrigsNormals(elementNodes):
    #unused; compute shape functions in local coordinates xi and eta
    def ShapeFunctions(xi, eta):
        N = np.zeros(6)
        N[0] = (1 - xi - eta) * (2 * (1 - xi - eta) - 1)         # N1
        N[1] = xi * (2 * xi - 1)                                 # N2
        N[2] = eta * (2 * eta - 1)                               # N3
        N[3] = 4 * xi * eta                                      # N4
        N[4] = 4 * (1 - xi - eta) * eta                          # N5
        N[5] = 4 * (1 - xi - eta) * xi                           # N6
        return N

    #compute derivatives of shape functions at xi and eta
    def ShapeFunctionDerivatives(xi, eta):
        """Compute the derivatives of the shape functions with respect to xi and eta."""
        dN_dxi = np.array([
            -3 + 4*xi + 4*eta,  # dN1/dxi
             4*xi - 1,          # dN2/dxi
             0,                 # dN3/dxi
             4*eta,             # dN5/dxi
            -4*eta,             # dN6/dxi
             4 - 8*xi - 4*eta,  # dN4/dxi
        ])
        dN_deta = np.array([
            -3 + 4*xi + 4*eta,  # dN1/deta
             0,                 # dN2/deta
             4*eta - 1,         # dN3/deta
             4*xi,              # dN5/deta
             4 - 4*xi - 8*eta,  # dN6/deta
            -4*xi,              # dN4/deta
        ])
        return [dN_dxi, dN_deta]

    # Local coordinates of the 6 nodes in the reference triangle
    # this also represents the node ordering in Netgen!
    localCoords = np.array([
        [0.0, 0.0],  # Corner 1
        [1.0, 0.0],  # Corner 2
        [0.0, 1.0],  # Corner 3
        [0.5, 0.5],  # Mid-edge 23
        [0.0, 0.5],  # Mid-edge 31
        [0.5, 0.0],  # Mid-edge 12
    ])
    
    normals = np.zeros_like(elementNodes)
    
    # Compute the normal at each node
    for i, (xi, eta) in enumerate(localCoords):
        dN_dxi, dN_deta = ShapeFunctionDerivatives(xi, eta)
        
        # Compute the tangents
        t_xi = np.sum(dN_dxi[:, None] * elementNodes, axis=0)
        t_eta = np.sum(dN_deta[:, None] * elementNodes, axis=0)
        
        # Compute the normal as the cross product of the tangents
        normal = np.cross(t_xi, t_eta)
        norm = np.linalg.norm(normal)
        normal = normal/norm if norm > 0 else normal

        normals[i] = normal
    
    return normals


#************************************************
#**function: refine triangle mesh; every triangle is subdivided into 4 triangles
#**input:
#  points: list of np.array with 3 floats per point 
#  triangles: list of np.array with 3 int per triangle (0-based indices to triangles)
#**output: returns [points2, triangles2] containing the refined mesh; if the original mesh is consistent, no points are duplicated; if the mesh is not consistent, some mesh points are duplicated!
#**notes: becomes slow for meshes with more than 5000 points
def RefineMesh(points, triangles):
    # 2
    # |\
    # a c
    # |  \
    # 0-b-1
    points2 = copy.deepcopy(points)
    triangles2 = []
    
    #build point2trig list for efficiency, at most, per triangle 3 new points:
    trigsPerPoint = [ [] for _ in range(len(points) + len(triangles)*3) ]
    # for (ti, trig) in enumerate(triangles):
    #     for i in trig:
    #         trigsPerPoint[i] += [ti]

    pnew = [0,0,0] #a,b,c
    for (ti, trig) in enumerate(triangles):
        for j in range(3):
            pointNew = 0.5*(np.array(points[trig[j]])+np.array(points[trig[j-1]]))
            found = -1

            #go through all triangles at one point, if new (refined) trig exists, it contains the new point:
            for (i, ti2) in enumerate(trigsPerPoint[trig[j]]):
                for pointIndex in triangles2[ti2]:
                    if np.linalg.norm(pointNew-points2[pointIndex]) <= 1e-12:
                        found = pointIndex

            if found==-1:
                pnew[j] = len(points2)
                points2 += [pointNew]
            else:
                pnew[j] = found
        toff = len(triangles2)
        triangles2 += [np.array([trig[0],pnew[1],pnew[0]],dtype=int)]
        triangles2 += [np.array([trig[1],pnew[2],pnew[1]],dtype=int)]
        triangles2 += [np.array([trig[2],pnew[0],pnew[2]],dtype=int)]
        triangles2 += [np.array([pnew[0],pnew[1],pnew[2]],dtype=int)]
        #add new triangles to trigsPerPoint:
        for (ti, trig) in enumerate(triangles2[-4:]):
            for i in trig:
                trigsPerPoint[i] += [toff+ti]

    return [points2, triangles2]

#************************************************
#**function: shrink mesh using triangle normals; every point is at least moved a distance 'distance' normal from boundary
#**input:
#  points: list of np.array with 3 floats per point 
#  triangles: list of np.array with 3 int per triangle (0-based indices to triangles)
#  distance: float value of minimum distance
#**output: returns [points2, triangles2] containing the refined mesh; currently the points of the subdivided triangles are duplicated!
#**notes: ONLY works for consistent meshes (no duplicated points!)
def ShrinkMeshNormalToSurface(points, triangles, distance):
    points2 = copy.deepcopy(points)
    triangles2 = copy.deepcopy(triangles)
    #disp = [np.zeros(3).copy()]*len(points2) #copy, otherwise linked!!!
    disp = copy.deepcopy(points)
    for i in range(len(points2)):
        disp[i] *= 0.
    
    for trig in triangles:
        n = ComputeTriangleNormal(points[trig[0]],points[trig[1]],points[trig[2]])

        for i in range(3):
            dn = -distance*n
            for j in range(3):
                if abs(dn[j]) > abs(disp[trig[i]][j]):
                    disp[trig[i]][j] = dn[j]

    for i in range(len(points2)):
        points2[i] += disp[i]

    return [points2, triangles2]


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: helper function to compute triangular mesh from list of vertices (=points) and segments;
#   computes triangular meshes for non-convex case. In order to make it efficient, it first computes
#   neighbors and then defines triangles at segments to be inside/outside. Finally neighboring
#   relations are used to define all triangles inside/outside
#   finally only returns triangles that are inside the segments
#**input:
#  vertices: list of pairs of coordinates of vertices in mesh [x,y]
#  segments: list of segments, which are pairs of node numbers [i,j], defining the boundary of the mesh;
#            the ordering of the nodes is such that left triangle = inside, right triangle = outside, compare example with segment [V1,V2]:\\
#  
#     inside
#  V1         V2
#  O----------O
#    outside
#**output:
#  triangulation structure of Delaunay(...), see scipy.spatial.Delaunaystructure, containing all simplices (=triangles)
#**notes: Delauney will not work if points are duplicated; you must first create point lists without duplicated points!
#**example:
# points = np.array([[0, 0], [0, 2], [2, 2], [2, 1], [1, 1], [0, 1], [1, 0]])
# segments = [len(points)-1,0]
# for i in range(len(points)-1):
#     segments += [i,i+1]
# tri = ComputeTriangularMesh(points, segments)
# exudyn.Print(tri.simplices)
def ComputeTriangularMesh(vertices, segments):
    from scipy.spatial import Delaunay
    from copy import deepcopy

    nVertices = len(vertices)
    tri = Delaunay(np.array(vertices))
    trigs = deepcopy(tri.simplices)
    
    #+++++++++++++++++++++++++++++++++
    #compute vertices2simplices list:
    vertices2simplices = [[] for _ in range(len(vertices))]

    for i, trig in enumerate(trigs):
        for vertex_idx in trig:
            vertices2simplices[vertex_idx].append(i)

    for s in vertices2simplices:
        if len(s) == 0:
            import exudyn as exu
            exu.Print('WARNING: ComputeTriangularMesh: vertex with no relation to triangles found - this indicates duplicated vertices, e.g., in SolidExtrusion() vertices')
            break

    trigNeighbors = tri.neighbors 

    #+++++++++++++++++++++++++++++++++
    #compute inside triangles:
    trianglesInside = np.full(len(trigs), -1, dtype=int)
    for iSeg, seg in enumerate(segments): #triangles left to segment are inside
        listTest = vertices2simplices[seg[0]] + vertices2simplices[seg[1]]
        for trigIndex in listTest:
            for k in range(3):
                t0 = trigs[trigIndex, k]
                t1 = trigs[trigIndex, (k+1)%3]
                if (seg[0] == t0) and (seg[1] == t1): #inside triangle
                    trianglesInside[trigIndex] = 1
                elif (seg[0] == t1) and (seg[1] == t0): #outside triangle
                    trianglesInside[trigIndex] = 0

    #finally find remaining triangles (usually all triangles are on boundary, so nothing remains):
    undefinedTrigs = True
    while undefinedTrigs: #iterate as long as there are undefined triangles; usually only few iterations necessary
        undefinedTrigs = False
        for i in range(len(trigs)):
            if trianglesInside[i] == -1: #still undefined
                found = False
                for j in range(3): #look at all neighbors
                    tn = trigNeighbors[i, j]
                    if trianglesInside[tn] != -1:
                        trianglesInside[i] = trianglesInside[tn]
                        found = True
                if not found:
                    undefinedTrigs = True

    #now create new list of interior triangles
    interiorTrigs = []
    for i in range(len(trigs)):
        if trianglesInside[i] == 1: 
            interiorTrigs += [list(trigs[i])]
    
    tri.simplices = np.array(interiorTrigs)
    
    return tri

#**function: convert point list into segments (indices to points); point indices start with pointIndexOffset
#**input:
#  invert: True: circle defines outter boundary; False: circle cuts out geometry inside a geometry
#  pointIndexOffset: point indices start with pointIndexOffset
#**output: return segments, containing list of lists of point indices for segments
def SegmentsFromPoints(points, pointIndexOffset = 0, invert=False, closeCurve=True):
    n = len(points)
    segments = np.zeros((n,2),dtype=int)
    if invert:
        for i in reversed(range(n-1)):
            segments[i,:] = [i+1, i]
    else:
        for i in range(n-1):
            segments[i,:] = [i, i+1]

    if closeCurve:
        if invert:
            segments[n-1,:] = [0, n-1] #close segments
        else:
            segments[n-1,:] = [n-1, 0] #close segments

    return segments

#**function: create points and segments, used in SolidExtrusion(...) for circle with given parameters
#**input:
#  center: 2D center point (list/numpy array) for circle center
#  radius: radius of circle
#  invert: True: circle defines outter boundary; False: circle cuts out geometry inside a geometry
#  pointIndexOffset: point indices start with pointIndexOffset
#  nTiles: number of tiles/segments for circle creation (higher is finer)
#**output: return [points, segments], both containing lists of lists
#**notes: geometries may not intersect!
def CirclePointsAndSegments(center=[0,0], radius=0.1, invert = False, pointIndexOffset=0, nTiles=16):
    segments = np.zeros((nTiles,2),dtype=int)
    points = np.zeros((nTiles,2))
    
    for i in range(nTiles):
        phi = i/nTiles*2*pi
        points[i,:] = [radius*sin(phi)+center[0], radius*cos(phi)+center[1]]
        segments[i,:] = [(i+int(invert))%nTiles+pointIndexOffset, (i+1-int(invert))%nTiles+pointIndexOffset]

    segments = segments.tolist()
    points = points.tolist()
        
    return [points, segments]


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#************************************************
#**function: generate graphics data for 2D rectangle
#**input: minimal and maximal cartesian coordinates in (x/y) plane; color provided as list of 4 RGBA values
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
#**notes: DEPRECATED
def GraphicsDataRectangle(xMin, yMin, xMax, yMax, color=[0.,0.,0.,1.]): 

    rect = [xMin, yMin,xMax,yMax]
    dataRect = {'type':'Line', 'color': list(color), 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]}

    return dataRect

#************************************************
#**function: generate graphics data for orthogonal block drawn with lines
#**input: minimal and maximal cartesian coordinates for orthogonal cube; color provided as list of 4 RGBA values
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
#**notes: DEPRECATED
def GraphicsDataOrthoCubeLines(xMin, yMin, zMin, xMax, yMax, zMax, color=[0.,0.,0.,1.]): 

    dataRect = {'type':'Line', 'color': list(color), 'data':[xMin,yMin,zMin, xMin,yMax,zMin, xMin,yMin,zMin, xMax,yMin,zMin, xMax,yMax,zMin, xMax,yMin,zMin, 
                                                       xMax,yMin,zMax, xMax,yMax,zMax, xMax,yMin,zMax, xMin,yMin,zMax, xMin,yMax,zMax, xMin,yMin,zMax, 
                                                       xMin,yMin,zMin, xMin,yMax,zMin, xMax,yMax,zMin, xMax,yMax,zMax, xMin,yMax,zMax, xMin,yMax,zMin]}

    return dataRect


