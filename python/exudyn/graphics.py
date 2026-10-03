#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  This module newly introduces revised graphics functions, coherent with Exudyn terminology;
#           it provides basic graphics elements like cuboid, cylinder, sphere, solid of revolution, etc.;
#           offers also some advanced functions for STL import and mesh manipulation; 
#           for some advanced functions see graphicsDataUtilties;
#           GraphicsData helper functions generate dictionaries which contain line, text or triangle primitives for drawing in Exudyn using OpenGL.
#
# Author:   Johannes Gerstmayr
# Date:     2024-05-10 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn
import exudyn.basicUtilities as ebu
from exudyn.rigidBodyUtilities import ComputeOrthonormalBasisVectors, HomogeneousTransformation, \
                                      HT2rotationMatrix, HT2translation, RotationVector2RotationMatrix, \
                                      RotationMatrix2D, RotationMatrixZ, GramSchmidt
import exudyn.graphicsDataUtilities as gdu

from exudyn.advancedUtilities import IsEmptyList
from exudyn.misc.deprecation import Deprecated #the deprecations of the library (#2807)

#constants and fixed structures:
import numpy as np #LoadSolutionFile
import copy as copy #to be able to copy e.g. lists
import functools
from math import radians, pi, sin, cos, tan, asin #, acos

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'graphicsDataNormalsFactor', 'graphicsDataSwitchTriangleOrder', 'color', 'material',
    'colorList', 'Sphere', 'Spheres', 'Triangles6ToTriangles', 'SpheresToTriangleList', 'Lines',
    'Circle', 'Text', 'Cuboid', 'BrickXYZ', 'Brick', 'Cylinder', 'Tube', 'Torus', 'RigidLink',
    'SolidOfRevolution', 'Arrow', 'Basis', 'Frame', 'Quad', 'CheckerBoard', 'SolidExtrusion',
    'LinkedCylinders', 'BallBearingRings', 'InvoluteGear', 'ToothedRack', 'BoundingBoxSingle',
    'BoundingBox', 'FromPointsAndTrigs', 'ToPointsAndTrigs', 'Transform', 'Move',
    'MergeTriangleLists', 'InvertTriangles', 'InconsistentTriangles', 'NGsolveMesh2PointsAndTrigs',
    'FromSTLfileASCII', 'FromPyMeshlabFile', 'FromSTLfile', 'AddEdgesAndSmoothenNormals',
    'ExportSTL',
    ]

graphicsDataNormalsFactor = 1. #this is a factor being either -1. [original normals pointing inside; until 2022-06-27], while +1. gives corrected normals pointing outside
graphicsDataSwitchTriangleOrder = False #this is the old ordering of triangles in some Sphere or Cylinder functions, causing computed normals to point inside

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#colors ...

class color:
    """A structure with default values representing RGBA-colors (list of 4 values ranging from 0 to 1); users will access colors via graphics.color, e.g., graphics.color.red
    """
    red = exudyn.graphicsDataUtilities.color4red
    green = exudyn.graphicsDataUtilities.color4green
    blue = exudyn.graphicsDataUtilities.color4blue
    
    cyan = exudyn.graphicsDataUtilities.color4cyan
    magenta = exudyn.graphicsDataUtilities.color4magenta
    yellow = exudyn.graphicsDataUtilities.color4yellow
    
    orange = exudyn.graphicsDataUtilities.color4orange
    pink = exudyn.graphicsDataUtilities.color4pink
    lawngreen = exudyn.graphicsDataUtilities.color4lawngreen
    
    springgreen = exudyn.graphicsDataUtilities.color4springgreen
    violet = exudyn.graphicsDataUtilities.color4violet
    dodgerblue = exudyn.graphicsDataUtilities.color4dodgerblue
    
    lightred = exudyn.graphicsDataUtilities.color4lightred
    lightgreen = exudyn.graphicsDataUtilities.color4lightgreen
    steelblue = exudyn.graphicsDataUtilities.color4steelblue
    brown = exudyn.graphicsDataUtilities.color4brown
    
    black = exudyn.graphicsDataUtilities.color4black
    darkgrey = exudyn.graphicsDataUtilities.color4darkgrey
    darkgrey2 = exudyn.graphicsDataUtilities.color4darkgrey2
    grey = exudyn.graphicsDataUtilities.color4grey
    lightgrey = exudyn.graphicsDataUtilities.color4lightgrey
    lightgrey2 = exudyn.graphicsDataUtilities.color4lightgrey2
    white = exudyn.graphicsDataUtilities.color4white
    
    default = exudyn.graphicsDataUtilities.color4default
    defaultBody = [0.4,0.4,0.9,1] #default body color for some functions; same as in VisualizationBasics.h
    defaultJoint = [0.6,0.6,0.8,1] #default body color for some functions; same as in VisualizationBasics.h
    defaultFFRF = exudyn.graphicsDataUtilities.color4green #for create FFRF function


class material:
    """A structure that defines material indices and RGBA-values for standard materials; the material index (like indexChrome) can be used for the alpha-channel of a color to represent the material index; used only in the raytracer!
    """
    indexBase = 1000
    indexDefault  = 0+indexBase #use as colorRGBA = [1.,0,0,indexDefault]
    indexMatt     = 1+indexBase
    indexSteel    = 2+indexBase
    indexPlastic  = 3+indexBase
    indexChrome   = 4+indexBase
    indexShiny    = 5+indexBase
    indexTransparent = 6+indexBase
    indexGlass    = 7+indexBase
    indexMirror   = 8+indexBase
    indexEmission = 9+indexBase

    #these are the RGBA colors to represent the materials, using default color
    default  = [-1,-1,-1, indexDefault ]
    matt     = [-1,-1,-1, indexMatt    ]
    steel    = [-1,-1,-1, indexSteel   ]
    plastic  = [-1,-1,-1, indexPlastic ]
    chrome   = [-1,-1,-1, indexChrome  ]
    shiny    = [-1,-1,-1, indexShiny   ]
    transparent = [-1,-1,-1, indexTransparent]
    glass    = [-1,-1,-1, indexGlass   ]
    mirror   = [-1,-1,-1, indexMirror  ]
    emission = [-1,-1,-1, indexEmission]

#a convenient list for creating automatic coloring of objects
colorList = exudyn.graphicsDataUtilities.color4list

#the columns of the rows of a GraphicsData, per type and key (#2709)
_rowColumns = {'TriangleList': {'points':3, 'normals':3, 'colors':4, 'triangles':3, 'triangles6':6, 'edges':2, 'edges3':3},
               'Spheres': {'points':3, 'colors':4},
               'Lines': {'points':3, 'colors':4},
               'Line': {'data':3}}

#GraphicsData with its points, normals, colors, triangles and edges as rows (#2709) - the form exudyn.graphics returns; also a list or a dict of GraphicsData; anything else unchanged
def _Rows(graphicsData):
    if isinstance(graphicsData, list):
        return [_Rows(g) for g in graphicsData]
    if not isinstance(graphicsData, dict):
        return graphicsData
    if 'type' not in graphicsData:
        return {key: _Rows(value) for (key, value) in graphicsData.items()}
    columns = _rowColumns.get(graphicsData['type'], {})
    gNew = dict(graphicsData)
    for (key, n) in columns.items():
        if key in gNew:
            gNew[key] = np.asarray(gNew[key]).reshape((-1, n))
    return gNew

#GraphicsData with flat lists, as the functions of exudyn.graphics work on them inside; reads rows and flat
def _Flat(graphicsData):
    if not isinstance(graphicsData, dict) or 'type' not in graphicsData:
        return graphicsData
    gNew = dict(graphicsData)
    for key in _rowColumns.get(graphicsData['type'], {}):
        if key in gNew:
            gNew[key] = np.asarray(gNew[key]).flatten()
    return gNew

#the GraphicsData a function of exudyn.graphics returns is given as rows (#2709)
def _ReturnsRows(function):
    @functools.wraps(function)
    def WithRows(*args, **kwargs):
        return _Rows(function(*args, **kwargs))
    return WithRows


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@_ReturnsRows
def Sphere(point=[0,0,0], radius=0.1, color=[0.,0.,0.,1.], nTiles = 8,
           addEdges = False, edgeColor=color.black, addFaces=True,
           majorAngleMin = -0.5*pi, majorAngleMax = 0.5*pi, innerRadius = None):
    """generate graphics data for a sphere with point p and radius; a whole sphere is the type 'Spheres', which the
    renderer draws as a sphere and the raytracer intersects exactly; with edges, without faces, as a part of a sphere
    or hollow, it is a 'TriangleList'

    Args:
        point: center of sphere (3D list or np.array)
        radius: positive value
        color: provided as list of 4 RGBA values
        nTiles: used to determine resolution of sphere >=2; represents resolution of a half-circle; use larger values for finer resolution
        addEdges: True or number of edges along sphere shell (under development); for optimal drawing, nTiles shall be multiple of 4 or 8
        edgeColor: optional color for edges
        addFaces: if False, no faces are added (only edges); ignored in case of hollow sphere
        majorAngleMin: starting angle for sphere to be drawn; if > -0.5*pi, it will be shortened at -Z coordinate
        majorAngleMax: final angle for sphere to be drawn; if < 0.5*pi, it will be shortened at +Z coordinate
        innerRadius: draw hollow sphere in case of majorAngleMin or majorAngleMax do not have default values; the outer and the inner sphere and the flat faces at the cuts; a sphere that is not whole consists of 6-node triangles (triangles6)

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects
    """
    if nTiles < 2:
        exudyn.Print("WARNING: graphics.Sphere: nTiles < 2: setting nTiles=2")
        nTiles = 2
    if (not addEdges and addFaces and majorAngleMin == -0.5*pi and majorAngleMax == 0.5*pi and innerRadius is None):
        return Spheres(points=[point], radii=radius, colors=color, nTiles=nTiles)
    if innerRadius is None: #6-node triangles (#2709)
        return _SphereTriangles6(point=point, radius=radius, color=color, nTiles=nTiles, addEdges=addEdges,
                                 edgeColor=edgeColor, addFaces=addFaces, majorAngleMin=majorAngleMin,
                                 majorAngleMax=majorAngleMax)
    return _SphereHollowTriangles6(point=point, radius=radius, color=color, nTiles=nTiles, addEdges=addEdges,
                                   edgeColor=edgeColor, majorAngleMin=majorAngleMin, majorAngleMax=majorAngleMax,
                                   innerRadius=innerRadius)


@_ReturnsRows
def Spheres(points, radii=0.1, colors=[0.,0.,0.,1.], nTiles=8):
    """generate graphics data for many spheres at once, as one item of the type 'Spheres' - for particles or point
    clouds; the renderer draws each as a sphere, the raytracer intersects them exactly

    Args:
        points: the centers, as a list of 3D points or a numpy array of shape (n,3)
        radii: one radius for all, or one per point
        colors: one RGBA color for all, or one per point (list of lists or numpy array of shape (n,4))
        nTiles: resolution of the drawn spheres, the number of segments of a half circle (rounded down to a power of 2)

    Returns:
        graphicsData dictionary {'type':'Spheres', 'points', 'radii', 'colors', 'resolution'}
    """
    points = np.array(points, dtype=float).reshape((-1, 3))
    n = len(points)
    radii = np.array(radii, dtype=float).flatten()
    colors = np.array(colors, dtype=float).flatten()
    if len(radii) not in [1, n]:
        raise ValueError('graphics.Spheres: radii must be one value or one per point')
    if len(colors) not in [4, 4*n]:
        raise ValueError('graphics.Spheres: colors must be one RGBA color or one per point')
    return {'type':'Spheres', 'points':points.flatten(), 'radii':radii, 'colors':colors, 'resolution':int(nTiles)}


@_ReturnsRows
def Triangles6ToTriangles(graphicsData):
    """convert the 6-node triangles (key 'triangles6') of a TriangleList into 4 flat triangles each, on the same points,
    and its quadratic edges (key 'edges3') into 2 straight edges each, for the functions that need flat triangles (STL
    export, ToPointsAndTrigs, ...); the renderer splits them finer, see visualizationSettings.openGL.advanced.curvedTriangleTilingAngle

    Args:
        graphicsData: a graphicsData dictionary

    Returns:
        a graphicsData dictionary of the type 'TriangleList' with 'triangles' only, or graphicsData itself if it has no 'triangles6'
    """
    graphicsData = _Flat(graphicsData)
    if graphicsData['type'] != 'TriangleList' or ('triangles6' not in graphicsData and 'edges3' not in graphicsData):
        return graphicsData
    gNew = {key: value for (key, value) in graphicsData.items() if key not in ['triangles6', 'edges3']}
    triangles = list(np.array(graphicsData.get('triangles', []), dtype=int).flatten())
    for (c0, c1, c2, m01, m12, m20) in np.array(graphicsData.get('triangles6', []), dtype=int).reshape((-1, 6)):
        triangles += [c0, m01, m20,  m01, c1, m12,  m20, m12, c2,  m01, m12, m20]
    gNew['triangles'] = np.array(triangles, dtype=int)
    if 'edges3' in graphicsData: #each quadratic edge into two straight ones
        edges = list(np.array(graphicsData.get('edges', []), dtype=int).flatten())
        for (p0, p1, m) in np.array(graphicsData['edges3'], dtype=int).reshape((-1, 3)):
            edges += [p0, m,  m, p1]
        gNew['edges'] = np.array(edges, dtype=int)
    return gNew


@_ReturnsRows
def SpheresToTriangleList(graphicsData):
    """convert graphics data of the type 'Spheres' into a 'TriangleList' with the triangles of graphics.Sphere, for
    the functions that need triangles (merging, STL export, ...); other types are returned unchanged

    Args:
        graphicsData: a graphicsData dictionary

    Returns:
        a graphicsData dictionary of the type 'TriangleList', or graphicsData itself if it is not of the type 'Spheres'
    """
    graphicsData = _Flat(graphicsData)
    if graphicsData['type'] != 'Spheres':
        return graphicsData
    points = np.array(graphicsData['points'], dtype=float).reshape((-1, 3))
    n = len(points)
    radii = np.array(graphicsData.get('radii', 0.1), dtype=float).flatten()
    colors = np.array(graphicsData.get('colors', [0.,0.,0.,1.]), dtype=float).flatten()
    nTiles = int(graphicsData.get('resolution', 8))
    data = None
    for i in range(n):
        g = _SphereTriangleList(point=points[i], radius=radii[0] if len(radii) == 1 else radii[i],
                                color=list(colors[0:4] if len(colors) == 4 else colors[4*i:4*i+4]), nTiles=nTiles)
        data = g if data is None else MergeTriangleLists(data, g)
    return data


def _SphereTriangles6(point=[0,0,0], radius=0.1, color=[0.,0.,0.,1.], nTiles = 8,
                      addEdges = False, edgeColor=color.black, addFaces=True,
                      majorAngleMin = -0.5*pi, majorAngleMax = 0.5*pi):
    """a sphere or a part of it between two latitudes, of 6-node triangles (#2709): nTiles quadratic elements around
    (the 2*nTiles flat segments of _SphereTriangleList) and ceil(nTiles/2) from majorAngleMin to majorAngleMax, the
    latitudes and meridians of addEdges as edges3; see Sphere"""
    if majorAngleMin < -0.5*pi or majorAngleMax > 0.5*pi or majorAngleMax <= majorAngleMin:
        raise ValueError("graphics.Sphere: majorAngleMin must be > -0.5*pi and < majorAngleMax; majorAngleMax must > majorAngleMin")
    p = np.array(point, dtype=float)
    ne = nTiles #elements around
    nv = _NumberOfQuadraticElements(nTiles) #elements from majorAngleMin to majorAngleMax
    def PointAndNormal(u, v):
        phi = 2*pi*u
        theta = majorAngleMin + v*(majorAngleMax - majorAngleMin)
        normal = np.array([cos(theta)*sin(phi), cos(theta)*cos(phi), sin(theta)])
        return (p + radius*normal, normal)
    (points, normals, triangles6, Index) = _QuadraticPatch(PointAndNormal, ne, nv, True, False)
    data = {'type':'TriangleList', 'colors':np.array(list(color)*len(points)), 'points':np.array(points).flatten(),
            'normals':np.array(normals).flatten(), 'triangles6':np.array(triangles6 if addFaces else [], dtype=int).flatten()}

    if isinstance(addEdges, bool) and addEdges:
        addEdges = 3
    if addEdges > 0 and abs(majorAngleMax-majorAngleMin-pi) <= 1e-7:
        data['edgeColor'] = np.array(edgeColor)
        edges3 = []
        nt = 2 #meridians
        if addEdges > 1:
            nt = 4
        if addEdges > 3:
            nt = 8
        mu = 2*ne
        for j in range(nt): #meridians
            column = int(j*mu/nt)
            for e in range(nv):
                edges3 += [Index(column, 2*e), Index(column, 2*e+2), Index(column, 2*e+1)]
        if addEdges > 1: #latitudes, as many as addEdges-1, starting at the south pole as _SphereTriangleList does
            sTiles = max(addEdges-1, 1)
            nStep = max(int(2*nv/sTiles), 1)
            for row in range(nStep, 2*nv, nStep):
                for e in range(ne):
                    edges3 += [Index(2*e, row), Index(2*e+2, row), Index(2*e+1, row)]
        data['edges3'] = np.array(edges3, dtype=int)
    return data


def _SphereHollowTriangles6(point=[0,0,0], radius=0.1, color=[0.,0.,0.,1.], nTiles = 8, addEdges = False,
                            edgeColor=color.black, majorAngleMin = -0.5*pi, majorAngleMax = 0.5*pi, innerRadius = 0.05):
    """a hollow sphere between two latitudes, of 6-node triangles (#2709): the outer sphere as _SphereTriangles6, the inner
    one between the cut planes - or closed where a plane does not reach it -, and the flat faces at the cuts, a ring
    between the two spheres or a disc where the inner sphere is closed; with addEdges, the circles of the cuts as edges3;
    see Sphere"""
    if majorAngleMin < -0.5*pi or majorAngleMax > 0.5*pi or majorAngleMax <= majorAngleMin:
        raise ValueError("graphics.Sphere: majorAngleMin must be > -0.5*pi and < majorAngleMax; majorAngleMax must > majorAngleMin")
    if innerRadius <= 0 or innerRadius >= radius:
        raise ValueError("graphics.Sphere: innerRadius is invalid")
    p = np.array(point, dtype=float)
    ne = nTiles #elements around
    nv = _NumberOfQuadraticElements(nTiles) #elements from the lower to the upper latitude
    def Direction(phi, theta):
        return np.array([cos(theta)*sin(phi), cos(theta)*cos(phi), sin(theta)])

    #the latitudes at which the cut planes z = radius*sin(angle) meet the inner sphere; +-pi/2 where they do not
    def InnerAngle(angle):
        s = radius*sin(angle)/innerRadius
        return np.sign(angle)*0.5*pi if abs(s) >= 1 else asin(s)
    (innerMin, innerMax) = (InnerAngle(majorAngleMin), InnerAngle(majorAngleMax))

    points, normals, triangles6 = [], [], []
    rims = [] #the cut circles, as rows of grid points
    for (r, angleMin, angleMax, sign) in [(radius, majorAngleMin, majorAngleMax, 1.), (innerRadius, innerMin, innerMax, -1.)]:
        def PointAndNormal(u, v, r=r, angleMin=angleMin, angleMax=angleMax, sign=sign):
            d = Direction(2*pi*u, angleMin + v*(angleMax - angleMin))
            return (p + r*d, sign*d)
        (pts, nrm, trigs, Grid) = _QuadraticPatch(PointAndNormal, ne, nv, True, False, offset=len(points))
        points += pts
        normals += nrm
        triangles6 += trigs

    #the faces at the cuts: a ring between the circles of the two spheres, or a disc if the inner sphere is closed there
    for (angle, innerAngle, sign) in [(majorAngleMin, innerMin, -1.), (majorAngleMax, innerMax, 1.)]:
        if abs(angle) >= 0.5*pi - 1e-14:
            continue #the pole: no cut
        z = radius*sin(angle)
        rOuter = radius*cos(angle)
        rInner = innerRadius*cos(innerAngle) if abs(innerAngle) < 0.5*pi - 1e-14 else 0.
        normal = np.array([0., 0., sign])
        def PointAndNormal(u, v, z=z, rOuter=rOuter, rInner=rInner, normal=normal):
            radial = Direction(2*pi*u, 0.)
            return (p + np.array([0., 0., z]) + ((1-v)*rOuter + v*rInner)*radial, normal)
        (pts, nrm, trigs, Grid) = _QuadraticPatch(PointAndNormal, ne, 1, True, False, offset=len(points))
        points += pts
        normals += nrm
        triangles6 += trigs
        rims += [[Grid(i, 0) for i in range(2*ne)]]

    data = {'type':'TriangleList', 'colors':np.array(list(color)*len(points)), 'points':np.array(points).flatten(),
            'normals':np.array(normals).flatten(), 'triangles6':np.array(triangles6, dtype=int).flatten()}
    if addEdges and len(rims) != 0:
        data['edgeColor'] = np.array(edgeColor)
        edges3 = []
        for rim in rims:
            for e in range(ne):
                edges3 += [rim[2*e], rim[(2*e+2) % (2*ne)], rim[2*e+1]]
        data['edges3'] = np.array(edges3, dtype=int)
    return data


def _SphereTriangleList(point=[0,0,0], radius=0.1, color=[0.,0.,0.,1.], nTiles = 8,
           addEdges = False, edgeColor=color.black, addFaces=True,
           majorAngleMin = -0.5*pi, majorAngleMax = 0.5*pi):
    """the flat triangles of a sphere, also a part of a sphere, with edges - for SpheresToTriangleList and RigidLink; see
    Sphere"""
    nTilesPhi = 2*nTiles
    if majorAngleMin < -0.5*pi or majorAngleMax > 0.5*pi or majorAngleMax <= majorAngleMin:
        raise ValueError("graphics.Sphere: majorAngleMin must be > -0.5*pi and < majorAngleMax; majorAngleMax must > majorAngleMin")
        
    p = np.array(point)
    r = radius
    #orthonormal basis:
    e0=np.array([1,0,0])
    e1=np.array([0,1,0])
    e2=np.array([0,0,1])

    points = []
    normals = []
    colors = []
    triangles = []
    
    #create points for circles around z-axis with tiling
    for i0 in range(nTiles+1):
        z = r*sin(majorAngleMin + i0/nTiles*(majorAngleMax-majorAngleMin))    #runs from -r .. r (this is the coordinate of the axis of circles)
        for iphi in range(nTilesPhi):
            phi = 2*pi*iphi/nTilesPhi #angle
            fact = sin(0.5*pi + majorAngleMin + i0/nTiles*(majorAngleMax-majorAngleMin))

            x = fact*r*sin(phi)
            y = fact*r*cos(phi)

            vv = x*e0 + y*e1 + z*e2
            points += list(p + vv)
            
            n = ebu.Normalize(vv) 
            normals += n
            
            colors += color

    if addFaces:
        for i0 in range(nTiles):
            for iphi in range(nTilesPhi):
                p0 = i0*nTilesPhi+iphi
                p1 = (i0+1)*nTilesPhi+iphi
                iphi1 = iphi + 1
                if iphi1 >= nTilesPhi: 
                    iphi1 = 0
                p2 = i0*nTilesPhi+iphi1
                p3 = (i0+1)*nTilesPhi+iphi1
    
                if True:
                    if graphicsDataSwitchTriangleOrder:
                        triangles += [p0,p3,p1, p0,p2,p3]
                    else:
                        triangles += [p0,p1,p3, p0,p3,p2]

    data = {'type':'TriangleList', 'colors':np.array(colors), 
            'points':np.array(points), 
            'normals':np.array(normals), 
            'triangles':np.array(triangles)}
    
    if type(addEdges) == bool and addEdges:
        addEdges = 3

    if addEdges > 0 and abs(majorAngleMax-majorAngleMin-pi) <= 1e-7:
        data['edgeColor'] = np.array(edgeColor)

        edges = []
        hEdges = [] #edges at half of iphi
        nt = 2
        if addEdges > 1:
            nt = 4
        if addEdges > 3:
            nt = 8
        for j in range(nt):
            hEdges += [[]]
        if nt > nTiles: #otherwise does not work!
            nt = max(2,int(nTiles/2)*2)
            
        hTiles = int(nTilesPhi/nt)
        # hLast = [None]*nt
        # hFirst = [None]*nt
        sTiles = max(addEdges-1,1) #non-negative
        nStep = max(int(nTiles/sTiles),1)
        
        for i0 in range(nTiles):
            for iphi in range(nTilesPhi):
                p0 = i0*nTilesPhi+iphi
                p1 = (i0+1)*nTilesPhi+iphi
                if i0%nStep == 0:
                    iphi1 = iphi + 1
                    if iphi1 >= nTilesPhi: 
                        iphi1 = 0
                    p2 = i0*nTilesPhi+iphi1
                    if addEdges>1:
                        edges += [p0, p2]
                if hTiles != 0:
                    if iphi%hTiles == 0:
                        j = int(iphi/hTiles)
                        if j < nt:
                            hEdges[j] += [p0,p1]

        
        for j in range(nt):
            if nt%2 == 0: #close edges only for even nt
                hEdges[j] += [hEdges[j][-1], hEdges[(j+int(nt/2))%nt][-1]]
                
            edges += hEdges[j]

        data['edges'] = np.array(edges)
    
    return data
            


#************************************************
@_ReturnsRows
def Lines(pList, color=[0.,0.,0.,1.], shape='linear'):
    """generate graphics data for a polyline, given by list of points and color; transforms to GraphicsData dictionary

    Args:
        pList: list of 3D numpy arrays or lists (to achieve closed curve, set last point equal to first point); with shape='quadratic' the points along the curve, each segment by its start point, mid point and end point, so p0, m01, p1, m12, p2, ... - an odd number of points
        color: provided as list of 4 RGBA values
        shape: 'linear' for straight segments, 'quadratic' for curved segments, each drawn as a quadratic curve through its three points and split by the renderer, see visualizationSettings.openGL.advanced.curvedTriangleTilingAngle

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects: of the type 'Line' for shape='linear', of the type 'Lines' with shape 'quadratic' else

    Example:
        #create simple 3-point lines
        gLine=graphics.Lines([[0,0,0],[1,0,0],[2,0.5,0]], color=color.red)
        #a quarter circle as one quadratic segment
        gArc=graphics.Lines([[1,0,0],[np.sqrt(0.5),np.sqrt(0.5),0],[0,1,0]], color=color.red, shape='quadratic')
    """
    if shape == 'quadratic':
        points = np.array(pList, dtype=float).reshape((-1, 3))
        if len(points) < 3 or len(points) % 2 != 1:
            raise ValueError("graphics.Lines: shape='quadratic' needs an odd number of points, at least 3: start, mid, end, mid, end, ...")
        #per segment the end points, then the mid point, as GraphicsData Lines expects it
        rows = np.concatenate([[points[k], points[k+2], points[k+1]] for k in range(0, len(points)-1, 2)])
        return {'type':'Lines', 'shape':'quadratic', 'points':rows,
                'colors':np.tile(np.array(color, dtype=float), (len(rows), 1))}
    elif shape != 'linear':
        raise ValueError("graphics.Lines: shape must be 'linear' or 'quadratic'")
    data = np.zeros(len(pList)*3)
    for i, p in enumerate(pList):
        data[i*3:i*3+3] = p
    dataRect = {'type':'Line', 'color': np.array(color), 'data':data}

    return dataRect


#************************************************
def Circle(point=[0,0,0], radius=1, color=[0.,0.,0.,1.]): 
    """generate graphics data for a single circle; currently the plane normal = [0,0,1], just allowing to draw planar circles -- this may be extended in future!

    Args:
        point: center point of circle
        radius: radius of circle
        color: provided as list of 4 RGBA values

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects

    Note:
        the tiling (number of segments to draw circle) can be adjusted by visualizationSettings.general.circleTiling
    """
    return {'type':'Circle', 'color': np.array(color), 'radius': radius, 'position':np.array(point)}


#************************************************
def Text(point=[0,0,0], text='', color=[0.,0.,0.,1.], fontSize=0., offset=[0.,0.]):
    """generate graphics data for a text drawn at a 3D position

    Args:
        point: position of text
        text: string representing text; multiline texts can be written with line breaks
        color: provided as list of 4 RGBA values
        fontSize: scalar fontSize or 0. for default; default font size in Exudyn is 12 (visualizationSettings.view0.window.globalFontSize)
        offset: offset in X/Y screen plane provided as list of 2 float values; this offset is not rotated with the model view and given relative to font size (offset [1,1] equals to offset of one character moved right and up)

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects

    Note:
        text size can be adjusted with visualizationSettings.view0.window.globalFontSize, which affects the text size (=font size) globally
    """
    return {'type':'Text', 
            'color': np.array(color), 
            'text':text, 
            'position':np.array(point),
            'fontSize':fontSize,
            'offset':offset}


@_ReturnsRows
def Cuboid(pList, color=[0.,0.,0.,1.], faces=[1,1,1,1,1,1], addNormals=False, addEdges=False, edgeColor=color.black, addFaces=True): 
    """generate graphics data for general block with endpoints, according to given vertex definition

    Args:
        pList: is a list of points [[x0,y0,z0],[x1,y1,z1],...]
        color: provided as list of 4 RGBA values
        faces: includes the list of six binary values (0/1), denoting active faces (value=1); set index to zero to hide face
        addNormals: if True, normals are added and there are separate points for every triangle
        addEdges: if True, edges are added in TriangleList of GraphicsData
        edgeColor: optional color for edges
        addFaces: if False, no faces are added (only edges)

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects
    """
    # bottom: (z goes upwards from node 0 to node 4)
    # ^y
    # |
    # 3---2
    # |   |
    # |   |
    # 0---1-->x
    #
    # top:
    # ^y
    # |
    # 7---6
    # |   |
    # |   |
    # 4---5-->x
    #
    # faces: bottom, top, sideface0, sideface1, sideface2, sideface3 (sideface0 has nodes 0,1,4,5)

    # colors=[]
    # for i in range(8):
    #     colors=colors+color
    colors = np.tile(color,8)
    if len(pList) != 8: raise ValueError('graphics.Cuboid: expects a pList with 8 points')

    points = np.zeros(24)
    for i, p in enumerate(pList):
        points[3*i:3*i+3] = p

    #1-based ... triangles = [1,3,2, 1,4,3, 5,6,7, 5,7,8, 1,2,5, 2,6,5, 2,3,6, 3,7,6, 3,4,7, 4,8,7, 4,1,8, 1,5,8 ]
    #triangles = [0,2,1, 0,3,2, 6,4,5, 6,7,4, 0,1,4, 1,5,4, 1,2,5, 2,6,5, 2,3,6, 3,7,6, 3,0,7, 0,4,7]

    trigList = [[0,2,1], [0,3,2], #
                [6,4,5], [6,7,4], #
                [0,1,4], [1,5,4], #
                [1,2,5], [2,6,5], #
                [2,3,6], [3,7,6], #
                [3,0,7], [0,4,7]] #
    triangles = []

    if not addNormals:
        for i in range(6):
            if faces[i]:
                for j in range(2):
                    if addFaces:
                        triangles += trigList[i*2+j]
        data = {'type':'TriangleList', 'colors': colors, 'points':points, 'triangles':np.array(triangles)}

        if addEdges:
            edges = [0,1, 1,2, 2,3, 3,0,
                     4,5, 5,6, 6,7, 7,4,
                     0,4, 1,5, 2,6, 3,7 ]
    else:
        normals = []
        points2 = []
        
        cnt = 0
        for i in range(6):
            if faces[i]:
                for j in range(2):
                    trig = trigList[i*2+j]
                    normal = gdu.ComputeTriangleNormal(pList[trig[0]],pList[trig[1]],pList[trig[2]])
                    normals+=list(normal)*3 #add normal for every point
                    for k in range(3):
                        if addFaces:
                            triangles += [cnt] #new point for every triangle
                        points2 += list(pList[trig[k]])
                        cnt+=1

        if addEdges:
            edges = [0,2, 2,1, 5,4, 4,3, #according to vertex occurance in trigList
                     7,8, 8,6, 9,10, 10,11,
                     12,14, 13,16, 24,26, 27,28 
                     ]
        
        data = {'type':'TriangleList', 'colors': np.tile(color,cnt), 'points':np.array(points2), 
                'normals':np.array(normals), 'triangles':np.array(triangles)}
        
    if addEdges:
        data['edges'] = np.array(edges)
        data['edgeColor'] = np.array(edgeColor)
        
    return data


@Deprecated('1.11.0', 2029, use='graphics.Brick(centerPoint, size)')
@_ReturnsRows
def BrickXYZ(xMin, yMin, zMin, xMax, yMax, zMax, color=[0.,0.,0.,1.], addNormals=False, addEdges=False, edgeColor=color.black, addFaces=True): 
    """generate graphics data for orthogonal 3D block with min and max dimensions

    Args:
        x/y/z/Min/Max: minimal and maximal cartesian coordinates for orthogonal cube
        color: list of 4 RGBA values
        addNormals: add face normals to triangle information
        addEdges: if True, edges are added in TriangleList of GraphicsData
        edgeColor: optional color for edges
        addFaces: if False, no faces are added (only edges)

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects

    Note:
        DEPRECATED
    """
    pList = [[xMin,yMin,zMin], [xMax,yMin,zMin], [xMax,yMax,zMin], [xMin,yMax,zMin],
             [xMin,yMin,zMax], [xMax,yMin,zMax], [xMax,yMax,zMax], [xMin,yMax,zMax]]
    return Cuboid(pList, color, addNormals=addNormals, addEdges=addEdges, 
                  edgeColor=edgeColor, addFaces=addFaces)


@_ReturnsRows
def Brick(centerPoint=[0,0,0], size=[0.1,0.1,0.1], color=[0.,0.,0.,1.], addNormals=False, addEdges=False, 
          edgeColor=color.black, addFaces=True, roundness=0, nTiles=12): 
    """generate graphics data for orthogonal 3D box with center point and size; using roundness=1, it draws an ellipsoid inside the box and in case 0 < roundness < 1, it draws a body blended between box and ellipsoid

    Args:
        centerPoint: center of box as 3D list or np.array
        size: size as 3D list or np.array
        color: list of 4 RGBA values
        addNormals: add face normals to triangle information
        addEdges: if True, edges are added in TriangleList of GraphicsData
        edgeColor: optional color for edges
        addFaces: if False, no faces are added (only edges)
        roundness: if > 0, it draws an ellipsoid, using nTiles for drawing; edges are not available if roundness > 0
        nTiles: only apply if roundness > 0; discretization of whole ellipsoid; should be multiple of 4 to avoid artifacts

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects; if addEdges=True, it returns a list of two dictionaries
    """
    if roundness == 0:
        xMin = centerPoint[0] - 0.5*size[0]
        yMin = centerPoint[1] - 0.5*size[1]
        zMin = centerPoint[2] - 0.5*size[2]
        xMax = centerPoint[0] + 0.5*size[0]
        yMax = centerPoint[1] + 0.5*size[1]
        zMax = centerPoint[2] + 0.5*size[2]
    
        gBox = BrickXYZ.__wrapped__(xMin, yMin, zMin, xMax, yMax, zMax, color, 
                         addNormals=addNormals, addEdges=addEdges, edgeColor=edgeColor, addFaces=addFaces)
        # if addEdges:
        #     gBox['edgeColor'] = np.array(edgeColor)
        #     gBox['edges'] = np.array([0,1, 1,2, 2,3, 3,0,  0,4, 1,5, 2,6, 3,7,  4,5, 5,6, 6,7, 7,4])
        return gBox
    else: #blending of box and ellipsoid
        if nTiles < 8: 
            exudyn.Print("WARNING: graphics.Brick: nTiles < 8: setting nTiles=8")
            nTiles = 8 #less does not work well

        point = np.array(centerPoint)
        
        nTiles2 = int(nTiles/2+1)
        sx, sy, sz = np.array(size) / 2.0  # half-sizes
        u = np.linspace(0, 2 * np.pi, nTiles, endpoint=False)
        v = np.linspace(0, np.pi, nTiles2)
    
        uu, vv = np.meshgrid(u, v)
        uu = uu.flatten()
        vv = vv.flatten()
    
        #sphere points (unit sphere)
        ux = np.cos(uu)
        uy = np.sin(uu)
        vx = np.sin(vv)
        vz = np.cos(vv)
        xSphere = np.zeros_like(uu)
        ySphere = np.zeros_like(uu)
        zSphere = np.zeros_like(uu)

        #make a smooth transition from cuboid to ellipsoid, corners rounded first
        pot = (1+max(min(1,roundness),0)) #1=rectangle, 2=sphere
        fact = 1/(abs(cos(pi/4))**pot + abs(sin(pi/4))**pot)
        addedRN = min(roundness,0.2) #added rounding at corner
        if roundness > 0.2:
            addedRN = roundness**0.5 * 0.2**0.5
        factMax = 1/(abs(cos(pi/4)) + abs(sin(pi/4)))
        for i in range(len(xSphere)):
            phiu = uu[i]
            phiv = vv[i]

            rxy = 1./( fact*(abs(cos(phiu+pi/4))**pot + abs(sin(phiu+pi/4))**pot))
            rxz = 1./( fact*(abs(cos(phiv+pi/4))**pot + abs(sin(phiv+pi/4))**pot))
            
            maxr = 1/( factMax*(abs(cos(addedRN*pi/4)) + abs(sin(addedRN*pi/4))))

            rxy = min(rxy,maxr)
            rxz = min(rxz,maxr)
            
            xSphere[i] = vx[i] * rxz * ux[i]*rxy
            ySphere[i] = vx[i] * rxz * uy[i]*rxy
            zSphere[i] = vz[i] * rxz

        spherePoints = np.stack([xSphere, ySphere, zSphere], axis=1)
    
        #stretch to ellipsoid
        ellipsoidPoints = point + spherePoints * [sx, sy, sz]
        #normals (analytical from ellipsoid)
        normalsSphere = spherePoints / np.linalg.norm(spherePoints, axis=1, keepdims=True)
        normals = normalsSphere
        vertices = ellipsoidPoints

        # Triangle indices
        triangles = []
        for i in range(nTiles2 - 1):
            for j in range(nTiles):
                i0 = i * nTiles + j
                i1 = i * nTiles + (j + 1) % nTiles
                i2 = (i + 1) * nTiles + j
                i3 = (i + 1) * nTiles + (j + 1) % nTiles
                if i!=nTiles2-2:
                    triangles.append([i0, i2, i3])
                if i!=0:
                    triangles.append([i0, i3, i1])
    
        colors = list(color) * len(vertices)
        data = {'type':'TriangleList', 'colors':np.array(colors), 
                'points':vertices.flatten(),
                'normals':np.array(normals).flatten(), 
                'triangles':np.array(triangles).flatten()}

        #to improve normals:
        # data = graphics.AddEdgesAndSmoothenNormals(data, addEdges=False, 
        #                                             edgeAngle=2*pi
        #                                             )
        return data


def _QuadraticPatch(PointAndNormal, nu, nv, closedU, closedV, offset=0):
    """6-node triangles on a parametric patch (#2709): nu x nv quadratic elements, PointAndNormal(u, v) with u, v in [0,1]
    gives a point and its (outward) normal; the grid holds the corners and the mid nodes of the elements,
    (2nu (+1)) x (2nv (+1)) points; each element is two 6-node triangles, oriented so that their corners turn
    counterclockwise about the given normals. Returns (points, normals, triangles6, Index), Index(i, j) the number of the
    grid point i (along u) and j (along v), plus offset"""
    mu = 2*nu if closedU else 2*nu+1
    mv = 2*nv if closedV else 2*nv+1
    points = []
    normals = []
    for j in range(mv):
        for i in range(mu):
            (p, n) = PointAndNormal(i/(2*nu), j/(2*nv))
            points += [np.array(p, dtype=float)]
            normals += [np.array(n, dtype=float)]

    def Index(i, j):
        return offset + (j % mv if closedV else j)*mu + (i % mu if closedU else i)

    def Coincide(i, j):
        return np.linalg.norm(points[i-offset] - points[j-offset]) <= 1e-12*(1. + np.linalg.norm(points[i-offset]))

    triangles6 = []
    for ej in range(nv):
        for ei in range(nu):
            (i0, j0) = (2*ei, 2*ej)
            (a, b, c, d) = (Index(i0, j0), Index(i0+2, j0), Index(i0+2, j0+2), Index(i0, j0+2))
            (ab, bc, cd, da, m) = (Index(i0+1, j0), Index(i0+2, j0+1), Index(i0+1, j0+2), Index(i0, j0+1), Index(i0+1, j0+1))
            if Coincide(c, d): #a pole, the apex of a cone or the center of a disc: one triangle, its sides along v
                triangles6 += [[a, b, c, ab, bc, da]]
            elif Coincide(a, b):
                triangles6 += [[b, c, d, bc, cd, da]]
            else:
                triangles6 += [[a, b, c, ab, bc, m], [a, c, d, m, cd, da]]

    #the orientation, from the first triangle that is not degenerate
    for t in triangles6:
        (pa, pb, pc) = (points[t[0]-offset], points[t[1]-offset], points[t[2]-offset])
        cross = np.cross(pb - pa, pc - pa)
        if np.linalg.norm(cross) > 1e-12*max(1., np.linalg.norm(pb - pa)**2):
            if cross @ (normals[t[0]-offset] + normals[t[1]-offset] + normals[t[2]-offset]) < 0:
                triangles6 = [[t[0], t[2], t[1], t[5], t[4], t[3]] for t in triangles6]
            break
    return (points, normals, triangles6, Index)


def _OrientTriangle6(t, points, normal):
    """the 6-node triangle t with its corners turned counterclockwise about normal"""
    (pa, pb, pc) = (np.array(points[t[0]]), np.array(points[t[1]]), np.array(points[t[2]]))
    if np.cross(pb - pa, pc - pa) @ np.array(normal) < 0:
        return [t[0], t[2], t[1], t[5], t[4], t[3]]
    return list(t)


def _OrientTriangle(t, points, normal):
    """the flat triangle t with its corners turned counterclockwise about normal"""
    (pa, pb, pc) = (np.array(points[t[0]]), np.array(points[t[1]]), np.array(points[t[2]]))
    if np.cross(pb - pa, pc - pa) @ np.array(normal) < 0:
        return [t[0], t[2], t[1]]
    return list(t)


def _NumberOfQuadraticElements(nSegments):
    """the quadratic elements that replace nSegments flat segments: each covers two of them (#2709)"""
    return max(1, int(np.ceil(nSegments/2)))

@_ReturnsRows
def Cylinder(pAxis=[0,0,0], vAxis=[0,0,1], radius=0.1, color=[0.,0.,0.,1.], nTiles = 16, 
             radiusInner = None, angleRange=[0,2*pi], lastFace = True, cutPlain = True, 
             addEdges=False, edgeColor=color.black, addFaces=True, **kwargs):  
    """generate graphics data for a cylinder with given axis, radius and color; nTiles gives the number of tiles (minimum=3);
    the cylinder consists of 6-node triangles (triangles6), ceil(nTiles/2) curved elements around, drawn with at least nTiles segments

    Args:
        pAxis: axis point of one face of cylinder (3D list or np.array)
        vAxis: vector representing the cylinder's axis (3D list or np.array)
        radius: positive value representing radius of cylinder
        color: provided as list of 4 RGBA values
        nTiles: used to determine resolution of cylinder >=3; use larger values for finer resolution
        radiusInner: if not equal 0, this represents the inner radius of a hollow cylinder; some options like angleRange, lastFace, etc. do not work in this case
        angleRange: given in rad, to draw only part of cylinder (halfcylinder, etc.); for full range use [0..2 * pi]
        lastFace: if angleRange != [0,2*pi], then the faces of the open cylinder are shown with lastFace = True
        cutPlain: only used for angleRange != [0,2*pi]; if True, a plane is cut through the part of the cylinder; if False, the cylinder becomes a cake shape ...
        addEdges: if True, edges are added in TriangleList of GraphicsData; if addEdges is integer, additional int(addEdges) lines are added on the cylinder mantle
        edgeColor: optional color for edges
        addFaces: if False, no faces are added (only edges)
        alternatingColor: if given, optionally another color in order to see rotation of solid; only works, if angleRange=[0,2*pi]

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects
    """
    if nTiles < 3: 
        exudyn.Print("WARNING: graphics.Cylinder: nTiles < 3: setting nTiles=3")
        nTiles = 3

    if radiusInner is not None: #simple alternative to draw hollow cylinder
        vLen = np.linalg.norm(vAxis)
        contour=[[ 0.  ,radius],
                 [ vLen,radius],
                 [ vLen,radiusInner],
                 [ 0.  ,radiusInner],
                 [ 0.  ,radius]]
        return SolidOfRevolution(pAxis=pAxis, vAxis=vAxis, contour=contour, color=color, nTiles=nTiles,
                                 addEdges=addEdges, addFaces=addFaces, edgeColor=edgeColor)
        
    
    #the mantle and the two faces of 6-node triangles (#2709): nTiles flat segments around become
    #ceil(nTiles/2) quadratic elements, each covering two of them; the renderer splits them when it draws
    p0 = np.array(pAxis, dtype=float)
    vAxis = np.array(vAxis, dtype=float)
    p1 = p0 + vAxis
    [axis, n1, n2] = ComputeOrthonormalBasisVectors(vAxis)
    r = radius

    alpha = angleRange[1]-angleRange[0] #angular range
    alpha0 = angleRange[0]
    fullCircle = alpha >= 2.*pi - 1e-12
    ne = _NumberOfQuadraticElements(nTiles if fullCircle else nTiles-1)
    mRing = 2*ne if fullCircle else 2*ne+1 #points on a ring

    def Radial(k):
        phi = alpha0 + k*alpha/(2*ne)
        return sin(phi)*n1 + cos(phi)*n2

    color2 = list(color) #alternating color
    if 'alternatingColor' in kwargs:
        color2 = list(kwargs['alternatingColor'])
    def RingColor(k):
        return list(color) if k < mRing/2 else color2

    #mantle: one quadratic element along the axis, the mid row at half length
    (points, normals, triangles6, Mantle) = _QuadraticPatch(lambda u, v: (p0 + v*vAxis + r*Radial(u*2*ne), Radial(u*2*ne)),
                                                           ne, 1, fullCircle, False)
    colors = []
    for j in range(3):
        for k in range(mRing):
            colors += RingColor(k)

    #the faces: a fan of 6-node triangles about the center, the mid nodes on the radii and on the rim
    faceRing = [[], []]
    triangles = [] #flat closing faces of a partial cylinder
    for (side, pCenter, normal) in [(0, p0, -axis), (1, p1, axis)]:
        iCenter = len(points)
        points += [pCenter]
        normals += [normal]
        colors += list(color)
        iRing = len(points)
        for k in range(mRing):
            points += [pCenter + r*Radial(k)]
            normals += [normal]
            colors += RingColor(k)
        iRadial = len(points) #mid nodes on the radii to the even ring points
        for k in range(0, mRing, 2):
            points += [pCenter + 0.5*r*Radial(k)]
            normals += [normal]
            colors += RingColor(k)
        faceRing[side] = list(range(iRing, iRing+mRing))
        for e in range(ne):
            k0 = 2*e
            k2 = (2*e+2) % mRing
            t = [iCenter, iRing+k0, iRing+k2, iRadial+k0//2, iRing+k0+1, iRadial+k2//2]
            triangles6 += [_OrientTriangle6(t, points, normal)]
        if not fullCircle and cutPlain: #the rest of the face, up to the chord
            triangles += [_OrientTriangle([iCenter, iRing+mRing-1, iRing], points, normal)]

    if not fullCircle and lastFace:
        if cutPlain: #the plane through the chord
            (a, b, c, d) = (Mantle(mRing-1, 0), Mantle(0, 0), Mantle(0, 2), Mantle(mRing-1, 2))
            pa, pb, pd = points[a], points[b], points[d]
            normalChord = np.cross(pb - pa, pd - pa)
            normalChord = ebu.Normalize(list(normalChord if normalChord @ (pa - p0) > 0 or normalChord @ (pb-p0) > 0 else -normalChord))
            s = len(points)
            points += [pa, pb, points[c], pd]
            normals += [normalChord]*4
            colors += list(color)*4
            triangles += [_OrientTriangle([s, s+1, s+2], points, normalChord), _OrientTriangle([s, s+2, s+3], points, normalChord)]
        else: #cake shape: two faces from the axis to the first and to the last radius
            for k in [0, mRing-1]:
                pr0 = p0 + r*Radial(k)
                pr1 = p1 + r*Radial(k)
                normalCut = np.cross(vAxis, Radial(k))
                if k == 0:
                    normalCut = -normalCut
                normalCut = ebu.Normalize(list(normalCut))
                s = len(points)
                points += [p0, p1, pr1, pr0]
                normals += [normalCut]*4
                colors += list(color)*4
                triangles += [_OrientTriangle([s, s+1, s+2], points, normalCut), _OrientTriangle([s, s+2, s+3], points, normalCut)]

    if not addFaces:
        triangles6 = []
        triangles = []

    data = {'type':'TriangleList', 'colors':np.array(colors), 'points':np.array(points).flatten(),
            'normals':np.array(normals).flatten(), 'triangles6':np.array(triangles6, dtype=int).flatten()}
    if len(triangles) != 0:
        data['triangles'] = np.array(triangles, dtype=int).flatten()

    if addEdges:
        data['edgeColor'] = np.array(edgeColor)
        edges3 = []
        edges = []
        for side in range(2): #the rims, curved
            ring = faceRing[side]
            for e in range(ne):
                edges3 += [ring[2*e], ring[(2*e+2) % mRing], ring[2*e+1]]
            if not fullCircle:
                edges += [ring[-1], ring[0]]
        if type(addEdges) != bool: #lines along the mantle
            faceEdges = int(addEdges)
            nStep = max(1, int((2*ne)/faceEdges))
            for i in range(faceEdges):
                k = (i*nStep) % mRing
                edges += [faceRing[0][k], faceRing[1][k]]
        data['edges3'] = np.array(edges3, dtype=int)
        if len(edges) != 0:
            data['edges'] = np.array(edges, dtype=int)

    return data


@_ReturnsRows
def Tube(points, axes, radius=0.1, color=[0.,0.,0.,1.], nTiles = 16):  
    """generate graphics data for a tube with given list of points and axes, radius and color; nTiles gives the number of tiles (minimum=3)

    Args:
        points: list of 3D vectors (or numpy arrays) representing the center points of the tube line
        axes: list of 3D vectors (or numpy arrays) representing the axis according to the points
        radius: positive value representing radius of tube
        color: provided as list of 4 RGBA values
        nTiles: used to determine resolution of cylinder >=3; use larger values for finer resolution; the tube consists of 6-node triangles (triangles6), ceil(nTiles/2) curved elements around, drawn with at least nTiles segments

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects
    """
    if nTiles < 3: 
        exudyn.Print("WARNING: graphics.Tube: nTiles < 3: set nTiles=3")
        nTiles = 3

    if len(points) < 2:
        raise ValueError("graphics.Tube: must have at least 2 points and 2 axes")

    if len(points) != len(axes):
        raise ValueError("graphics.Tube: points and axes lists must be the same length")

    nSegments = len(points)



    n = len(points)
    frames = []

    #create frames; frames shall not change too much, as this causes artifacts...
    # Start frame
    z0 = axes[0] / np.linalg.norm(axes[0])
    up = np.array([0, 0, 1]) if abs(z0[2]) < 0.9 else np.array([1, 0, 0])
    x0 = np.cross(up, z0)
    x0 /= np.linalg.norm(x0)
    y0 = np.cross(z0, x0)
    frames.append((x0, y0, z0))

    prev_x = x0
    prev_y = y0
    prev_z = z0

    for i in range(1, n):
        z = axes[i] / np.linalg.norm(axes[i])
        v = np.cross(prev_z, z)
        if np.linalg.norm(v) < 1e-6:#directions are nearly aligned
            x = prev_x
            y = prev_y
        else:
            v /= np.linalg.norm(v)
            angle = np.arccos(np.clip(np.dot(prev_z, z), -1.0, 1.0))
            R = RotationVector2RotationMatrix(angle*v)
            x = R @ prev_x
            y = R @ prev_y

        frames.append((x, y, z))
        prev_x, prev_y, prev_z = x, y, z


    #6-node triangles (#2709): ceil(nTiles/2) quadratic elements around, one element between two points of the tube
    #line, its mid row at the mid point, the normals there the mean of the normals at the two points
    ne = _NumberOfQuadraticElements(nTiles)
    def RingPointAndNormal(i, angle):
        [x, y, z] = frames[i]
        normal = np.cos(angle) * x + np.sin(angle) * y
        return (np.array(points[i]) + radius * normal, normal)
    def PointAndNormal(u, v):
        angle = 2 * np.pi * u
        row = int(round(v*2*(nSegments-1)))
        if row % 2 == 0:
            return RingPointAndNormal(row//2, angle)
        (p0, n0) = RingPointAndNormal(row//2, angle)
        (p1, n1) = RingPointAndNormal(row//2+1, angle)
        return (0.5*(p0 + p1), ebu.Normalize(list(n0 + n1)))
    (vertices, normals, triangles6, Index) = _QuadraticPatch(PointAndNormal, ne, nSegments-1, True, False)

    return {'type':'TriangleList',
            'colors':np.array(list(color)*len(vertices)),
            'points':np.array(vertices).flatten(),
            'normals':np.array(normals).flatten(),
            'triangles6':np.array(triangles6, dtype=int).flatten()}


@_ReturnsRows
def Torus(point, axis, radiusMajor=0.5, radiusMinor=0.1, color=[0., 0., 0., 1.], 
          nTilesMajor=24, nTilesMinor=12, minorAngleStart=0, minorAngleEnd=2*np.pi, 
          smoothNormals=True, invert=False):
    """generate graphics data for a torus with given major and minor radius, center point and axis

    Args:
        point: 3D vector (or numpy array) representing the center point of the torus
        axis: 3D vector (or numpy array) representing the axis of revolution of the torus
        radiusMajor: major radius of torus
        radiusMinor: minor radius of torus
        color: provided as list of 4 RGBA values
        nTilesMajor: used to for resolution of tube with major radius; use larger values for finer resolution
        nTilesMinor: used to for resolution of circle with minor radius; use larger values for finer resolution
        minorAngleStart: starting angle for minor radius; 0 is the angle at outmost radius of torus, pi is at inside
        minorAngleEnd: end angle for minor radius; use -0.5*pi / 0.5*pi to draw only the outer half of the torus
        smoothNormals: if True, the torus consists of 6-node triangles (triangles6), ceil(nTilesMajor/2) x ceil(nTilesMinor/2) curved elements, drawn with at least the given numbers of segments; otherwise of flat triangles
        invert: if False, the outside faces are visible; if invert=True, the inside faces are visible (influences reflections, light, etc.)

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects
    """
    if nTilesMajor < 3: 
        exudyn.Print("WARNING: graphics.Torus: nTilesMajor < 3: setting nTilesMajor=3")
        nTilesMajor = 3
    if nTilesMinor < 3: 
        exudyn.Print("WARNING: graphics.Torus: nTilesMinor < 3: setting nTilesMinor=3")
        nTilesMinor = 3

    #create orthonormal basis for torus
    [ex,ey,ez] = ComputeOrthonormalBasisVectors(axis) #ex=axis
    A = np.vstack([ey,ez,ex]).T

    if minorAngleStart >= minorAngleEnd:
        raise ValueError('Torus: ensure that minorAngleStart < minorAngleEnd !')
    isOpen = (minorAngleEnd-minorAngleStart) < 2*np.pi-1e-10 #open circle
    invertSign = (1.-2.*int(invert))

    if smoothNormals: #6-node triangles (#2709): nTiles flat segments become ceil(nTiles/2) quadratic elements
        neMajor = _NumberOfQuadraticElements(nTilesMajor)
        neMinor = _NumberOfQuadraticElements(nTilesMinor)
        def PointAndNormal(u, v):
            phi = 2*np.pi*u
            theta = minorAngleStart + (minorAngleEnd-minorAngleStart)*v
            localNormal = np.array([np.cos(phi)*np.cos(theta), np.sin(phi)*np.cos(theta), np.sin(theta)])
            center = np.array([np.cos(phi)*radiusMajor, np.sin(phi)*radiusMajor, 0.])
            return (A @ (center + radiusMinor*localNormal) + point, invertSign*(A @ localNormal))
        (vertices, normals, triangles6, Index) = _QuadraticPatch(PointAndNormal, neMajor, neMinor, True, not isOpen)
        return {'type':'TriangleList', 'colors':np.array(list(color)*len(vertices)),
                'points':np.array(vertices).flatten(), 'normals':np.array(normals).flatten(),
                'triangles6':np.array(triangles6, dtype=int).flatten()}

    #flat triangles
    vertices = []
    normals = []
    triangles = []
    nTilesMinor1 = nTilesMinor+isOpen

    for i in range(nTilesMajor):
        phi = 2 * np.pi * i / nTilesMajor  # major angle
        center = np.array([np.cos(phi) * radiusMajor,
                           np.sin(phi) * radiusMajor,
                           0.0])  # center of the tube ring

        for j in range(nTilesMinor1):
            theta = minorAngleStart+(minorAngleEnd-minorAngleStart) * j / nTilesMinor  # minor angle

            # minor circle point in local frame
            local_normal = np.array([
                np.cos(phi) * np.cos(theta),
                np.sin(phi) * np.cos(theta),
                np.sin(theta)
            ])

            local_pos = center + radiusMinor * local_normal

            world_pos = A @ local_pos + point
            world_normal = A @ local_normal

            vertices.append(world_pos)
            normals.append(invertSign*world_normal)

    # compute triangles
    for i in range(nTilesMajor):
        for j in range(nTilesMinor):
            idx0 = i * nTilesMinor1 + j
            idx1 = i * nTilesMinor1 + (j + 1) % nTilesMinor1
            idx2 = ((i + 1) % nTilesMajor) * nTilesMinor1 + j
            idx3 = ((i + 1) % nTilesMajor) * nTilesMinor1 + (j + 1) % nTilesMinor1

            if invert:
                triangles.append([idx0, idx1, idx2])
                triangles.append([idx1, idx3, idx2])
            else:
                triangles.append([idx0, idx2, idx1])
                triangles.append([idx1, idx2, idx3])

    colors = color*len(vertices)

    return {'type':'TriangleList',
            'colors':np.array(colors).flatten(),
            'points':np.array(vertices).flatten(),
            'triangles':np.array(triangles).flatten()}


@_ReturnsRows
def RigidLink(p0,p1,axis0=[0,0,0], axis1=[0,0,0], radius=[0.1,0.1], 
                          thickness=0.05, width=[0.05,0.05], color=[0.,0.,0.,1.], nTiles = 16):
    """generate graphics data for a planar Link between the two joint positions, having two axes

    Args:
        p0: joint0 center position
        p1: joint1 center position
        axis0: direction of rotation axis at p0, if drawn as a cylinder; [0,0,0] otherwise
        axis1: direction of rotation axis of p1, if drawn as a cylinder; [0,0,0] otherwise
        radius: list of two radii [radius0, radius1], being the two radii of the joints drawn by a cylinder or sphere
        width: list of two widths [width0, width1], being the two widths of the joints drawn by a cylinder; ignored for sphere
        thickness: the thickness of the link (shaft) between the two joint positions; thickness in z-direction or diameter (cylinder)
        color: provided as list of 4 RGBA values
        nTiles: used to determine resolution of cylinder >=3; use larger values for finer resolution

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects
    """
    linkAxis = (np.array(p1) - p0)
    #linkAxis0 = ebu.Normalize(linkAxis)
    a0=list(axis0)
    a1=list(axis1)
    
    data0 = Cylinder(p0, linkAxis, 0.5*thickness, color, nTiles)
    data1 = {}
    data2 = {}

    if np.linalg.norm(axis0) == 0:
        data1 = _SphereTriangleList(p0, radius[0], color, nTiles) #merged into the triangles of the link
    else:
        a0=ebu.Normalize(a0)
        data1 = Cylinder(list(np.array(p0)-0.5*width[0]*np.array(a0)), 
                                     list(width[0]*np.array(a0)), 
                                     radius[0], color, nTiles)
        
    if np.linalg.norm(axis1) == 0:
        data2 = _SphereTriangleList(p1, radius[1], color, nTiles) #merged into the triangles of the link
    else:
        a1=ebu.Normalize(a1)
        data2 = Cylinder(list(np.array(p1)-0.5*width[1]*np.array(a1)), 
                                     list(width[1]*np.array(a1)), radius[1], color, nTiles)

    #the cylinders of 6-node triangles and the flat triangles of the spheres in one list (#2709)
    return MergeTriangleLists(MergeTriangleLists(data0, data1), data2)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#   unused argument yet: contourNormals: if provided as list of 2D vectors, they prescribe the normals to the contour for smooth visualization; otherwise, contour is drawn flat
@_ReturnsRows
def SolidOfRevolution(pAxis, vAxis, contour, color=[0.,0.,0.,1.], nTiles = 16, smoothContour = False, 
                      addEdges = False, edgeColor=color.black, addFaces=True, smoothingAngle=2*np.pi, **kwargs):  
    """generate graphics data for a solid of revolution with given 3D point and axis, 2D point list for contour, (optional)2D normals and color;

    Args:
        pAxis: axis point of one face of solid of revolution (3D list or np.array)
        vAxis: vector representing the solid of revolution's axis (3D list or np.array)
        contour: a list of 2D-points, specifying the contour (x=axis, y=radius), e.g.: [[0,0],[0,0.1],[1,0.1]]
        color: provided as list of 4 RGBA values
        nTiles: used to determine resolution of solid; use larger values for finer resolution; the solid consists of 6-node triangles (triangles6), ceil(nTiles/2) curved elements around, drawn with at least nTiles segments
        smoothContour: if True, the contour is made smooth by auto-computing normals to the contour
        addEdges: True or number of edges along revolution mantle; for optimal drawing, nTiles shall be multiple addEdges
        edgeColor: optional color for edges
        addFaces: if False, no faces are added (only edges)
        smoothingAngle: if angle between two edges is smaller than smoothingAngle, smoothing is applied
        alternatingColor: add a second color, which enables to see the rotation of the solid

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects

    Example:
        #simple contour, using list of 2D points:
        contour=[[0,0.2],[0.3,0.2],[0.5,0.3],[0.7,0.4],[1,0.4],[1,0.]]
        rev1 = graphics.SolidOfRevolution(pAxis=[0,0.5,0], vAxis=[1,0,0],
                                             contour=contour, color=color.red,
                                             alternatingColor=color.grey)
        #draw torus:
        contour=[]
        r = 0.2 #small radius of torus
        R = 0.5 #big radius of torus
        nc = 16 #discretization of torus
        for i in range(nc+3): #+3 in order to remove boundary effects
            contour+=[[r*cos(i/nc*pi*2),R+r*sin(i/nc*pi*2)]]
        #use smoothContour to make torus looking smooth
        rev2 = graphics.SolidOfRevolution(pAxis=[0,0.5,0], vAxis=[1,0,0],
                                             contour=contour, color=color.red,
                                             nTiles = 64, smoothContour=True)
    """
    if len(contour) < 2: 
        raise ValueError("ERROR: SolidOfRevolution: contour must contain at least 2 points")
    if nTiles < 3: 
        exudyn.Print("WARNING: SolidOfRevolution: nTiles < 3: set nTiles=3")

    p0 = np.array(pAxis)
    #local coordinate system:
    [v,n1,n2] = ComputeOrthonormalBasisVectors(vAxis)

    color2 = list(color)
    if 'alternatingColor' in kwargs:
        color2 = kwargs['alternatingColor']

    #compute contour normals, assuming flat cones
    contourNormals = []
    for j in range(len(contour)-1):
        pc0 = np.array(contour[j])
        pc1 = np.array(contour[j+1])
        vc = pc1-pc0
        nc = ebu.Normalize([-vc[1],vc[0]])
        contourNormals += [nc]

    if np.linalg.norm(np.array(contour[0]) - np.array(contour[-1])) < 1e-10:
        contourNormals += [contourNormals[0]] #closed curve: normal for last point same as first
    else:
        contourNormals += [contourNormals[-1]] #normal for last point same as previous

    if smoothContour:
        contourNormalsAvg = [contourNormals[0]]
        contourNormalsNext = []
        for j in range(len(contour)-1):
            if np.arccos(np.array(contourNormals[j]) @ np.array(contourNormals[j+1])) < smoothingAngle:
                ns = ebu.Normalize(np.array(contourNormals[j]) + np.array(contourNormals[j+1])) #not fully correct, but sufficient
                contourNormalsAvg += [list(ns)]
                contourNormalsNext += [list(ns)]
            else:
                contourNormalsAvg += [contourNormals[j+1]]
                contourNormalsNext += [contourNormals[j]]

        contourNormalsNext += [contourNormals[-1]]
        contourNormals = contourNormalsAvg

    #per contour segment a band of 6-node triangles (#2709): nTiles flat segments around become ceil(nTiles/2)
    #quadratic elements, each covering two of them; along the segment one element, straight, its mid row at half length
    nf = graphicsDataNormalsFactor #factor for normals (inwards/outwards)
    v_ = v #the axis; v is the parameter along the contour below
    ne = _NumberOfQuadraticElements(nTiles)
    mRing = 2*ne #points on a ring

    def Radial(k):
        phi = k*2*pi/mRing
        return sin(phi)*n1 + cos(phi)*n2

    def RingColor(k):
        return list(color) if k < mRing/2 else list(color2)

    points = []
    normals = []
    colors = []
    triangles6 = []
    segmentRings = [] #the index of the first point of each segment's start ring
    for j in range(len(contour)-1):
        pc0 = np.array(contour[j])
        pc1 = np.array(contour[j+1])
        nc0 = np.array(contourNormals[j])
        nc1 = np.array(contourNormalsNext[j]) if smoothContour else nc0
        ncMid = np.array(ebu.Normalize(list(nc0 + nc1))) if np.linalg.norm(nc0 + nc1) > 1e-12 else nc0

        def PointAndNormal(u, v):
            k = u*mRing
            if v == 0:
                (pc, nc) = (pc0, nc0)
            elif v == 1:
                (pc, nc) = (pc1, nc1)
            else:
                (pc, nc) = (0.5*(pc0 + pc1), ncMid)
            radial = Radial(k)
            return (p0 + pc[1]*radial + pc[0]*v_, ebu.Normalize(list(nf*nc[1]*radial + nf*nc[0]*v_)))

        (pointsJ, normalsJ, triangles6J, Index) = _QuadraticPatch(PointAndNormal, ne, 1, True, False, offset=len(points))
        segmentRings += [len(points)]
        points += pointsJ
        normals += normalsJ
        for row in range(3):
            for k in range(mRing):
                colors += RingColor(k)
        if addFaces:
            triangles6 += triangles6J

    data = {'type':'TriangleList', 'colors':np.array(colors), 'points':np.array(points).flatten(),
            'normals':np.array(normals).flatten(), 'triangles6':np.array(triangles6, dtype=int).flatten()}

    if addEdges > 0:
        data['edgeColor'] = np.array(edgeColor)
        edges3 = [] #the rings at the start of each segment, curved
        edges = [] #lines along the segments, straight
        cntEdges = 0
        nSteps = mRing
        if type(addEdges) != bool and addEdges > 0:
            cntEdges = int(addEdges)
            nSteps = max(1, int(mRing/cntEdges))
        for s in segmentRings:
            for e in range(ne):
                edges3 += [s + 2*e, s + (2*e+2) % mRing, s + 2*e+1]
        for i in range(cntEdges):
            k = (i*nSteps) % mRing
            for s in segmentRings:
                edges += [s + k, s + 2*mRing + k]
        data['edges3'] = np.array(edges3, dtype=int)
        if len(edges) != 0:
            data['edges'] = np.array(edges, dtype=int)

    return data


@_ReturnsRows
def Arrow(pAxis, vAxis, radius, color=[0.,0.,0.,1.], headFactor = 2, headStretch = 4, nTiles = 12):  
    """generate graphics data for an arrow with given origin, axis, shaft radius, optional size factors for head and color; nTiles gives the number of tiles (minimum=3)

    Args:
        pAxis: axis point of the origin (base) of the arrow (3D list or np.array)
        vAxis: vector representing the vector pointing from the origin to the tip (head) of the error (3D list or np.array)
        radius: positive value representing radius of shaft cylinder
        headFactor: positive value representing the ratio between head's radius and the shaft radius
        headStretch: positive value representing the ratio between the head's radius and the head's length
        color: provided as list of 4 RGBA values
        nTiles: used to determine resolution of arrow (of revolution object) >=3; use larger values for finer resolution

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects
    """
    L = np.linalg.norm(vAxis)
    rHead = radius * headFactor
    xHead = L - headStretch*rHead
    contour=[[0,0],[0,radius],[xHead,radius],[xHead,rHead],[L,0]]
    return SolidOfRevolution(pAxis=pAxis, vAxis=vAxis, contour=contour, color=color, nTiles=nTiles)

@_ReturnsRows
def Basis(origin=[0,0,0], rotationMatrix = np.eye(3), length = 1, colors=[color.red, color.green, color.blue], 
                      headFactor = 2, headStretch = 4, nTiles = 12, **kwargs):  
    """generate graphics data for three arrows representing an orthogonal basis with point of origin, shaft radius, optional size factors for head and colors; nTiles gives the number of tiles (minimum=3)

    Args:
        origin: point of the origin of the base (3D list or np.array)
        rotationMatrix: optional transformation, which rotates the basis vectors
        length: positive value representing lengths of arrows for basis
        colors: provided as list of 3 colors (list of 4 RGBA values)
        headFactor: positive value representing the ratio between head's radius and the shaft radius
        headStretch: positive value representing the ratio between the head's radius and the head's length
        nTiles: used to determine resolution of arrows of basis (of revolution object) >=3; use larger values for finer resolution
        radius: positive value representing radius of arrows; default: radius = 0.01*length
        labels: a list of 3 strings written to the three axes (X, Y, Z); in this case, the result is returned as list of GraphicsData!

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects
    """
    radius = 0.01*length
    labels = None
    if 'radius' in kwargs:
        radius = kwargs['radius']
    if 'labels' in kwargs:
        labels = kwargs['labels']

    A = np.array(rotationMatrix)
    g1 = Arrow(origin,A@[length,0,0],radius, colors[0], headFactor, headStretch, nTiles)
    g2 = Arrow(origin,A@[0,length,0],radius, colors[1], headFactor, headStretch, nTiles)
    g3 = Arrow(origin,A@[0,0,length],radius, colors[2], headFactor, headStretch, nTiles)

    trigList = MergeTriangleLists(MergeTriangleLists(g1,g2),g3)
    if labels is None:
        return trigList
    else:
        p = np.array(origin)
        label1 = Text(p+A@[length,0,0], labels[0])
        label2 = Text(p+A@[0,length,0], labels[1])
        label3 = Text(p+A@[0,0,length], labels[2])
        return [trigList, label1, label2, label3]

@_ReturnsRows
def Frame(HT=np.eye(4), length = 1, colors=[color.red, color.green, color.blue], 
                      headFactor = 2, headStretch = 4, nTiles = 12, **kwargs):  
    """generate graphics data for frame (similar to Basis), showing three arrows representing an orthogonal basis for the homogeneous transformation HT; optional shaft radius, optional size factors for head and colors; nTiles gives the number of tiles (minimum=3)

    Args:
        HT: homogeneous transformation representing frame
        length: positive value representing lengths of arrows for basis
        colors: provided as list of 3 colors (list of 4 RGBA values)
        headFactor: positive value representing the ratio between head's radius and the shaft radius
        headStretch: positive value representing the ratio between the head's radius and the head's length
        nTiles: used to determine resolution of arrows of basis (of revolution object) >=3; use larger values for finer resolution
        radius: positive value representing radius of arrows; default: radius = 0.01*length

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects
    """
    radius = 0.01*length
    if 'radius' in kwargs:
        radius = kwargs['radius']

    
    A = HT2rotationMatrix(HT)
    origin = HT2translation(HT)
    
    g1 = Arrow(origin,A@[length,0,0],radius, colors[0], headFactor, headStretch, nTiles)
    g2 = Arrow(origin,A@[0,length,0],radius, colors[1], headFactor, headStretch, nTiles)
    g3 = Arrow(origin,A@[0,0,length],radius, colors[2], headFactor, headStretch, nTiles)

    return MergeTriangleLists(MergeTriangleLists(g1,g2),g3)


@_ReturnsRows
def Quad(pList, color=[0.,0.,0.,1.], **kwargs): 
    """generate graphics data for simple quad with option for checkerboard pattern;
    points are arranged counter-clock-wise, e.g.: p0=[0,0,0], p1=[1,0,0], p2=[1,1,0], p3=[0,1,0]

    Args:
        pList: list of 4 quad points [[x0,y0,z0],[x1,y1,z1],...]
        color: provided as list of 4 RGBA values
        alternatingColor: second color; if defined, a checkerboard pattern (default: 10x10) is drawn with color and alternatingColor
        nTiles: number of tiles for checkerboard pattern (default: 10)
        nTilesY: if defined, use number of tiles in y-direction different from x-direction (=nTiles)

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects

    Example:
        plane = graphics.Quad([[-8, 0, -8],[ 8, 0, -8,],[ 8, 0, 8],[-8, 0, 8]],
                                 color.darkgrey, nTiles=8,
                                 alternatingColor=color.lightgrey)
        oGround=mbs.AddObject(ObjectGround(referencePosition=[0,0,0],
                              visualization=VObjectGround(graphicsData=[plane])))
    """
    color2 = list(color)
    nTiles = 1
    if 'alternatingColor' in kwargs:
        color2 = kwargs['alternatingColor']
        nTiles = 10

    if 'nTiles' in kwargs:
        nTiles = kwargs['nTiles']
    nTilesY= nTiles
    if 'nTilesY' in kwargs:
        nTilesY = kwargs['nTilesY']

    p0 = np.array(pList[0])
    p1 = np.array(pList[1])
    p2 = np.array(pList[2])
    p3 = np.array(pList[3])

    points = []
    triangles = []
    normals = []
    #points are given always for 1 quad of checkerboard pattern
    ind = 0
    for j in range(nTilesY):
        for i in range(nTiles):
            f0 = j/(nTilesY)
            f1 = (j+1)/(nTilesY)
            pBottom0 = (nTiles-i)/nTiles  *((1-f0)*p0 + f0*p3) + (i)/nTiles  *((1-f0)*p1 + f0*p2)
            pBottom1 = (nTiles-i-1)/nTiles*((1-f0)*p0 + f0*p3) + (i+1)/nTiles*((1-f0)*p1 + f0*p2)
            pTop0 = (nTiles-i)/nTiles  *((1-f1)*p0 + f1*p3) + (i)/nTiles  *((1-f1)*p1 + f1*p2)
            pTop1 = (nTiles-i-1)/nTiles*((1-f1)*p0 + f1*p3) + (i+1)/nTiles*((1-f1)*p1 + f1*p2)
            points += list(pBottom0)+list(pBottom1)+list(pTop1)+list(pTop0)
            normal = list(gdu.ComputeTriangleNormal(pBottom0,pBottom1,pTop1))
            normals += normal*4 #per point
            #points += list(p0)+list(p1)+list(p2)+list(p3)
            triangles += [0+ind,1+ind,2+ind,  0+ind,2+ind,3+ind]
            ind+=4

    colors=[]
    for j in range(nTilesY):
        for i in range(nTiles):
            a=1
            if i%2 == 1:
                a=-1
            if j%2 == 1:
                a=-1*a
            if a==1:
                c = list(color) #if no checkerboard pattern, just this color
            else:
                c = color2
            colors=colors+c+c+c+c #4 colors for one sub-quad

    data = {'type':'TriangleList', 'colors': np.array(colors), 
            'points':np.array(points), 'normals':normals, 'triangles':np.array(triangles)}

    return data


@_ReturnsRows
def CheckerBoard(point=[0,0,0], normal=[0,0,1], size = 1,
                             color=color.lightgrey, alternatingColor=color.lightgrey2, nTiles=10, **kwargs):
    """function to generate checkerboard background;
    points are arranged counter-clock-wise, e.g.:

    Args:
        point: midpoint of pattern provided as list or np.array
        normal: normal to plane provided as list or np.array
        size: dimension of first side length of quad
        size2: dimension of second side length of quad
        color: provided as list of 4 RGBA values
        alternatingColor: second color; if defined, a checkerboard pattern (default: 10x10) is drawn with color and alternatingColor
        nTiles: number of tiles for checkerboard pattern in first direction
        nTiles2: number of tiles for checkerboard pattern in second direction; default: nTiles
        materialIndex: use special graphics material for both colors

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects

    Example:
        plane = graphics.CheckerBoard(normal=[0,0,1], size=5)
        oGround=mbs.AddObject(ObjectGround(referencePosition=[0,0,0],
                              visualization=VObjectGround(graphicsData=[plane])))
    """
    nTiles2 = nTiles
    if 'nTiles2' in kwargs:
        nTiles2 = kwargs['nTiles2']
    size2 = size
    if 'size2' in kwargs:
        size2 = kwargs['size2']
    
    color0 = color
    color1 = alternatingColor
    if 'materialIndex' in kwargs:
        color0 = color[0:3]+[kwargs['materialIndex']]
        color1 = alternatingColor[0:3]+[kwargs['materialIndex']]

    [v,n1,n2] = ComputeOrthonormalBasisVectors(normal)
    p0=np.array(point)
    points = [list(p0-0.5*size*n1-0.5*size2*n2),
              list(p0+0.5*size*n1-0.5*size2*n2),
              list(p0+0.5*size*n1+0.5*size2*n2),
              list(p0-0.5*size*n1+0.5*size2*n2)]

    return Quad(points, color=color0, alternatingColor=color1, 
                nTiles=nTiles, nTilesY=nTiles2)

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@_ReturnsRows
def SolidExtrusion(vertices, segments, height, 
                   rot = np.diag([1,1,1]), pOff = [0,0,0], 
                   relRot = np.diag([1,1,1]), relOff = [0,0,0], 
                   color = [0,0,0,1], smoothNormals = False, 
                   addEdges = False, edgeColor=color.black, addFaces=True):
    """create graphicsData for solid extrusion based on 2D points and segments; by default, the extrusion is performed in z-direction;
    additional transformations are possible to translate and rotate the extruded body;

    Args:
        vertices: list of pairs of coordinates of vertices in mesh [x,y], see ComputeTriangularMesh(...)
        segments: list of segments, which are pairs of node numbers [i,j], defining the boundary of the mesh;
                  the ordering of the nodes is such that left triangle = inside, right triangle = outside; see ComputeTriangularMesh(...)
        height:   height of extruded object
        rot:      rotation matrix, which the whole extruded object point coordinates are multiplied with before adding offset
        pOff:     3D offset vector added to all extruded coordinates (both planes); the z-coordinate of the extrusion object obtains 0 for the base plane, z=height for the top plane
        relRot: rotation matrix for transformation of top (second) plane of extrusion object
        relOff: 3D offset vector added top (second) plane of extrusion object; the z-coordinate is added to height, which is the base z-value
        color: provided as list of 4 RGBA values
        smoothNormals: if True, algorithm tries to smoothen normals at vertices and normals are added; creates more points; if False, triangle normals are used internally
        addEdges: if True or 1, edges at bottom/top are included in the GraphicsData dictionary; if 2, also mantle edges are included
        edgeColor: optional color for edges
        addFaces: if False, no faces are added (only edges)

    Returns:
        graphicsData dictionary, to be used in visualization of EXUDYN objects

    Example:
        #simple block with cutout
        g = graphics.SolidExtrusion(vertices=[[-0.4,-0.4], [0.4,-0.4], [ 0.4,0.4], [0.1,0.4],
                                              [0.1,  0.2], [-0.1,0.2], [-0.1,0.4], [-0.4,0.4]],
                                   segments=[[0,1], [1,2], [2,3], [3,4], [4,5], [5,6], [6,7], [7,0]],
                                   pOff = [0,2,-1], height=1.5,
                                   color=graphics.color.steelblue, addEdges=2)
        oGround=mbs.CreateGround(graphicsDataList=[g])
    """
    n = len(vertices)
    n2 = n*2 #total number of vertices
    ns = len(segments)
    colors=[]
    for i in range(n2):
        colors+=color

    relRotNp = np.array(relRot)
    relOffNp = np.array(relOff)

    edges = []
    mantleEdges = (addEdges == 2)

    points = [[]]*n2
    for i in range(n):
        points[i] = [vertices[i][0],vertices[i][1],0]
    for i in range(n):
        points[i+n] = relRotNp @ [vertices[i][0],vertices[i][1],0] + relOff + np.array([0,0,height])

    if addEdges: #second set of points for top/bottom faces
        edges = [[]]*(ns*2)
        for cnt, seg in enumerate(segments):
            edges[cnt] = [seg[0], seg[1]]
            edges[cnt+ns] = [seg[0]+n, seg[1]+n]

    edges = list(np.array(edges).flatten())
    if smoothNormals: #second set of points for top/bottom faces
        #pointNormals = [[]]*(2*n2)
        for i in range(n2):
            colors+=color
        pointNormals = np.zeros((2*n2,3))

        #add normals from segments:
        #normals are added twice for common points;
        #  => adding is ok, as both are normalized; later, normals are normalized
        for seg in segments:
            dirSeg = ebu.Normalize(np.array(vertices[seg[1]]) - np.array(vertices[seg[0]]))
            dirSeg3D = [dirSeg[1], -dirSeg[0], 0.] #this way points outwards ...
            pointNormals[seg[0]+2*n,:] += dirSeg3D
            pointNormals[seg[1]+2*n,:] += dirSeg3D
            pointNormals[seg[0]+3*n,:] += dirSeg3D
            pointNormals[seg[1]+3*n,:] += dirSeg3D
                
        points2 = [[]]*n2
        for i in range(n): #negative flat face
            points2[i] = [vertices[i][0],vertices[i][1],0.]
            pointNormals[i+0*n,:] = [0.,0.,-1.]
            
        for i in range(n): #positive flat face
            points2[i+n] = relRotNp @ [vertices[i][0],vertices[i][1],height] + relOffNp
            pointNormals[i+1*n,:] = (relRotNp @ [0.,0.,1.])
                        
        
    #transform points:
    pointsTransformed = []
    npRot = np.array(rot)
    npPoff = np.array(pOff)

    if smoothNormals:
        #also need to rotate normals!
        for i in range(len(pointNormals)):
            pointNormals[i] = ebu.Normalize(pointNormals[i]) #normalize as they are added twice from each segment!
            pointNormals[i] = npRot @ pointNormals[i]
        
    for i in range(n2):
        p = np.array(npRot @ points[i] + npPoff)
        pointsTransformed += list(p)
    
    if smoothNormals: #these are the points with normals from top/bottom surface
        for i in range(n2):
            p = np.array(npRot @ points2[i] + npPoff)
            pointsTransformed += list(p)

    #compute triangulation:
    tri = gdu.ComputeTriangularMesh(vertices, segments)
    trigs = tri.simplices
    nt =len(trigs)
    trigList = [[]] * (nt*2+ns*2) #top trigs, bottom trigs, circumference trigs (2 per quad)
    
    for i in range(nt):
        t = list(trigs[i])
        t.reverse()
        trigList[i] = copy.copy(t)
    for i in range(nt):
        t = list(trigs[i]+n)
        trigList[i+nt] = copy.copy(t)
        
    off = n2*int(smoothNormals)
    for i in range(ns):
        trigList[2*nt+2*i  ] = [segments[i][0]+off,segments[i][1]+off,  segments[i][1]+n+off]
        trigList[2*nt+2*i+1] = [segments[i][0]+off,segments[i][1]+n+off,segments[i][0]+n+off]

        if mantleEdges:
            edges += [segments[i][0]+off,segments[i][0]+n+off]

    triangles = []
    if addFaces:
        for t in trigList:
            triangles += t
   
    data = {'type':'TriangleList', 'colors': np.array(colors), 'points':np.array(pointsTransformed),
            'triangles':np.array(triangles)}
    if addEdges:
        data['edgeColor'] = np.array(edgeColor)
        data['edges'] = np.array(edges)

    if smoothNormals:
        data['normals'] = np.array(pointNormals.flatten())

    return data


@_ReturnsRows
def LinkedCylinders(point0, point1, axisCylinder, radius0, radius1,
                    radiusInner0=0, radiusInner1=0, nTiles=32, color=[0,0,0,1],
                    addEdges=0, edgeColor=color.black, addFaces=True, smoothNormals=True,
                    **kwargs):
    """generate graphics data for an extrusion solid linking two circles by their external tangents in a plane; the shape is extruded along axisCylinder with height equal to its norm; nTiles controls circle tessellation

    Args:
        point0: center of the first circle and base point of the extrusion (3D list or np.array)
        point1: a point whose projection into the plane through point0 with normal axisCylinder defines the direction to the second circle center (3D list or np.array)
        axisCylinder: vector normal to the circle plane and extrusion direction; extrusion height is ||axisCylinder|| (3D list or np.array)
        radius0: radius of the first circle (positive float)
        radius1: radius of the second circle (positive float)
        radiusInner0: if > 0, radius of bore of the first circle
        radiusInner1: if > 0, radius of bore of the second circle
        nTiles: tiling used for a full circle (>=3); partial arcs are sampled proportionally
        color: provided as list of 4 RGBA values
        addEdges: if True, edges are added in TriangleList of GraphicsData; if addEdges is integer, additional int(addEdges) lines are added on the extrusion
        edgeColor: optional color for edges
        addFaces: if False, no faces are added (only edges)
        smoothNormals: if True, algorithm tries to smoothen normals at vertices and normals are added; creates more points; if False, triangle normals are used internally
        kwargs: forwarded to graphics.SolidExtrusion

    Returns:
        graphicsData dictionary, to be used in visualization of Exudyn objects

    Example:
        g = graphics.LinkedCylinders(point0=[0,0,0], point1=[0.8,0.2,0.4], axisCylinder=[0,0,1.2],
                                      radius0=0.25, radius1=0.15, nTiles=48,
                                      color=graphics.color.steelblue, addEdges=2)
        oGround=mbs.CreateGround(graphicsDataList=[g])
    """

    # Convert inputs to numpy arrays
    p0 = np.asarray(point0, dtype=float).reshape(3)
    p1 = np.asarray(point1, dtype=float).reshape(3)
    axis = np.asarray(axisCylinder, dtype=float).reshape(3)
    r0 = float(radius0)
    r1 = float(radius1)

    # Orthonormal basis from provided helper (plane normal = axisCylinder, in-plane dir from point1-point0)
    e3, e1, e2 = GramSchmidt(axis, (p1 - p0))
    # rotation matrix for SolidExtrusion from (e1,e2,e3)
    rot = np.column_stack((e1, e2, e3))  # columns are basis vectors

    # project point1 onto plane and measure along e1
    L = float(np.dot(p1 - p0, e1))
    if L < 0:
        # ensure c1 lies at +x in local 2D; flip e1/e2 if needed
        e1 = -e1
        e2 = -e2
        L = -L
    if L <= max(r0,r1)-min(r1,r0):
        # degenerated -> draw larger cylinder
        pAxis = p0 if r0>r1 else p1
        return Cylinder(pAxis=pAxis, vAxis=axis, radius=max(r0,r1),
                        color=color, nTiles=nTiles, addEdges=addEdges)

    height = np.linalg.norm(axis)

    # 2D centers in the local plane
    c0 = np.array([0.0, 0.0])
    c1 = np.array([L,   0.0])

    # belt-drive angle for external tangents (open belt): beta = asin((r1 - r0)/L), clamped to [-1,1]
    m = np.clip((radius1 - radius0) / max(L, 1e-16), -1.0, 1.0)
    theta = float(np.arccos(m))  # robust, unambiguous

    # tangent point angles (upper and lower) on each circle
    theta0_u = np.pi - theta
    theta0_l = np.pi + theta
    theta1_u = np.pi - theta
    theta1_l = np.pi + theta  # equivalent to 2*np.pi - theta

    if len(kwargs) != 0 or not smoothNormals: #the flat extrusion, which takes the further arguments of SolidExtrusion
        return _LinkedCylindersFlat(c1, radius0, radius1, theta, radiusInner0, radiusInner1, nTiles, p0, rot, height,
                                    color, addEdges, edgeColor, addFaces, smoothNormals, **kwargs)

    #the outline of 6-node triangles (#2709): loops of corners, each segment with its mid node - on the arc, or halfway
    #on a tangent - and the outward normals at its three nodes (in the plane)
    def ArcSegments(center, r, a0, delta, outward):
        """the segments of an arc from angle a0 over delta (> 0 counterclockwise, < 0 clockwise), elements of two of
        today's segments each; outward: +1 if the material is inside the circle, -1 for a bore"""
        nSeg = max(2, int(np.ceil(nTiles * (abs(delta) / (2*np.pi)))))
        ne = _NumberOfQuadraticElements(nSeg)
        segs = []
        for e in range(ne):
            angles = [a0 + delta*e/ne, a0 + delta*(e+1)/ne, a0 + delta*(e+0.5)/ne]
            nodes = [np.array([np.cos(a), np.sin(a)]) for a in angles]
            segs += [[center + r*n for n in nodes] + [outward*n for n in nodes]]
        return segs

    def StraightSegment(pa, pb, outward):
        return [pa, pb, 0.5*(pa+pb), outward, outward, outward]

    loops = []
    arc1 = ArcSegments(c1, radius1, theta1_l, (theta1_u - theta1_l) % (2*np.pi), 1)
    arc0 = ArcSegments(c0, radius0, theta0_u, (theta0_l - theta0_u) % (2*np.pi), 1)
    normalUpper = np.array([np.cos(theta0_u), np.sin(theta0_u)])
    normalLower = np.array([np.cos(theta0_l), np.sin(theta0_l)])
    loops += [arc1 + [StraightSegment(arc1[-1][1], arc0[0][0], normalUpper)]
              + arc0 + [StraightSegment(arc0[-1][1], arc1[0][0], normalLower)]]
    for (center, rInner, rOuter) in [(c0, radiusInner0, radius0), (c1, radiusInner1, radius1)]:
        if rInner > 0 and rInner < rOuter: #a bore, clockwise
            loops += [ArcSegments(center, rInner, 0., -2*np.pi, -1)]

    #2D corners and their numbers, the segments and their mid nodes
    vertices2D = []
    segments = []
    segmentMid = {}
    for loop in loops:
        iFirst = len(vertices2D)
        for (k, seg) in enumerate(loop):
            vertices2D += [list(seg[0])]
            iNext = iFirst + (k+1) % len(loop)
            segments += [[iFirst+k, iNext]]
            segmentMid[(iFirst+k, iNext)] = seg[2]
            segmentMid[(iNext, iFirst+k)] = seg[2]

    def IsStraight(seg):
        return np.linalg.norm(seg[2] - 0.5*(seg[0] + seg[1])) <= 1e-14*(1. + np.linalg.norm(seg[0]))

    e3 = rot[:, 2]
    def Point3D(p2D, z):
        return p0 + rot @ np.array([p2D[0], p2D[1], z])

    points = []
    normals = []
    triangles6 = []
    rims = [[], []] #edges3 of the bottom and the top
    mantleLines = []
    #the mantle: per segment one quadratic element along the axis
    for loop in loops:
        for (k, seg) in enumerate(loop):
            (pa, pb, pm, na, nb, nm) = seg
            def PointAndNormal(u, v, pa=pa, pb=pb, pm=pm, na=na, nb=nb, nm=nm):
                N = [(1-u)*(1-2*u), u*(2*u-1), 4*u*(1-u)]
                p2D = N[0]*pa + N[1]*pb + N[2]*pm
                n2D = N[0]*na + N[1]*nb + N[2]*nm
                return (Point3D(p2D, v*height), rot @ np.array([n2D[0], n2D[1], 0.]))
            (pts, nrm, trigs, Grid) = _QuadraticPatch(PointAndNormal, 1, 1, False, False, offset=len(points))
            points += pts
            normals += nrm
            triangles6 += trigs
            rims[0] += [Grid(0, 0), Grid(2, 0), Grid(1, 0)]
            rims[1] += [Grid(0, 2), Grid(2, 2), Grid(1, 2)]
            nextSeg = loop[(k+1) % len(loop)]
            if IsStraight(seg) != IsStraight(nextSeg):
                mantleLines += [[Grid(2, 0), Grid(2, 2)]] #where an arc and a tangent meet

    #the two faces: the corner polygon triangulated, the mid nodes on the boundary taken from the segments
    tri = gdu.ComputeTriangularMesh(vertices2D, segments)
    for (z, normal) in [(0., -e3), (height, e3)]:
        iCorner = len(points)
        for v in vertices2D:
            points += [Point3D(v, z)]
            normals += [normal]
        midIndex = {}
        for trig in tri.simplices:
            t6 = [iCorner + int(trig[0]), iCorner + int(trig[1]), iCorner + int(trig[2])]
            for (a, b) in [(trig[0], trig[1]), (trig[1], trig[2]), (trig[2], trig[0])]:
                key = (min(a, b), max(a, b))
                if key not in midIndex:
                    pm = segmentMid.get((int(a), int(b)), 0.5*(np.array(vertices2D[a]) + np.array(vertices2D[b])))
                    midIndex[key] = len(points)
                    points += [Point3D(pm, z)]
                    normals += [normal]
                t6 += [midIndex[key]]
            triangles6 += [_OrientTriangle6(t6, points, normal)]

    if not addFaces:
        triangles6 = []
    data = {'type':'TriangleList', 'colors':np.array(list(color)*len(points)), 'points':np.array(points).flatten(),
            'normals':np.array(normals).flatten(), 'triangles6':np.array(triangles6, dtype=int).flatten()}
    if addEdges:
        data['edgeColor'] = np.array(edgeColor)
        data['edges3'] = np.array(rims[0] + rims[1], dtype=int)
        if not isinstance(addEdges, bool) and int(addEdges) >= 2 and len(mantleLines) != 0:
            data['edges'] = np.array(mantleLines, dtype=int).flatten()
    return data


def _LinkedCylindersFlat(c1, radius0, radius1, theta, radiusInner0, radiusInner1, nTiles, p0, rot, height,
                         color, addEdges, edgeColor, addFaces, smoothNormals, **kwargs):
    """LinkedCylinders as flat extrusion (SolidExtrusion), for the arguments that only SolidExtrusion takes"""
    c0 = np.array([0.0, 0.0])
    (theta0_u, theta0_l, theta1_u, theta1_l) = (np.pi - theta, np.pi + theta, np.pi - theta, np.pi + theta)
    # helper to wrap CCW and sample the outer (long) arc
    def _sample_arc(center, r, a0, a1, nTiles_local, out, addLastVertex=True):
        delta = a1 - a0
        if delta < 0: delta = delta + 2*np.pi
        if delta > 2*np.pi: delta = delta - 2*np.pi
        nSeg = max(2, int(np.ceil(nTiles_local * (delta / (2*np.pi)))))
        step = delta / nSeg
        for i in range(nSeg + addLastVertex):
            a = a0 + step * i
            out.append([center[0] + r*np.cos(a), center[1] + r*np.sin(a)])
        return nSeg

    # assemble 2D outline in CCW order:
    vertices2D = []
    segments = []

    # outer arc on circle 1: lower -> upper (long arc)
    _sample_arc(c1, radius1, theta1_l, theta1_u, nTiles, vertices2D)
    # outer arc on circle 0: upper -> lower (long arc)
    _sample_arc(c0, radius0, theta0_u, theta0_l, nTiles, vertices2D)

    # segments (consecutive + closing)
    for i in range(len(vertices2D) - 1):
        segments.append([i, i + 1])

    segments.append([len(vertices2D) - 1, 0]) #close curve

    # add holes for inner radii
    if radiusInner0 > 0 and radiusInner0 < radius0:
        ind0 = len(vertices2D)
        nSeg = _sample_arc(c0, radiusInner0, 0, 2*np.pi, nTiles, vertices2D, addLastVertex=False)
        for i in range(nSeg):
            segments.append([ind0 + (i + 1)%nSeg, ind0 + i])

    if radiusInner1 > 0 and radiusInner1 < radius1:
        ind0 = len(vertices2D)
        nSeg = _sample_arc(c1, radiusInner1, 0, 2*np.pi, nTiles, vertices2D, addLastVertex=False)
        for i in range(nSeg):
            segments.append([ind0 + (i + 1)%nSeg, ind0 + i])

    # Create SolidExtrusion:
    # - base at point0
    # - height = ||axisCylinder||
    # - rot for orientation of plane

    gd = SolidExtrusion(
        vertices=vertices2D,
        segments=segments,
        pOff=p0,
        rot=rot,
        height=height,
        color=color,
        addEdges=addEdges,
        edgeColor=edgeColor,
        addFaces=addFaces,
        smoothNormals=smoothNormals,
        **kwargs
    )
    return gd



#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@_ReturnsRows
def BallBearingRings(axis, outsideDiameter, boreDiameter, width, 
                     radiusCage, 
                     innerRingShoulderRadius, outerRingShoulderRadius, 
                     widthCage, heightCage,
                     innerEdgeChamfer, outerEdgeChamfer,
                     innerGrooveRadius, outerGrooveRadius,
                     innerGrooveTorusRadius, outerGrooveTorusRadius,
                     nTilesRings=32, nTilesGrooves=12, colorCage=[0.6,0.5,0.5,0.4], 
                     colorInnerRing=[0.5,0.5,0.5,0.5], colorOuterRing=[0.5,0.5,0.5,0.5],
                     **kwargs):
    """generate graphics for ball bearing rings, in particular for inner and outer rings; note that base parameters are identical as in function GetBallBearingData, assuming that the dictionary of the latter function is used as input for BallBearingRings

    Args:
        innerGrooveTorusRadius: major radius of torus for inner groove
        outerGrooveTorusRadius: major radius of torus for outer groove
        nTilesRings: circumferential tiling of rings
        nTilesGrooves: tiling of grooves
        colorCage: cage RGBA color
        colorInnerRing: inner ring RGBA color
        colorOuterRing: outer ring RGBA color

    Returns:
        dictionary of graphics data containing 'innerRingGraphics', 'outerRingGraphics' and 'cageGraphics'; Note: graphics data is in the local bearing coordinate system, which should align with inner ring, outer ring and cage bodies!

    Example:
        import exudyn.graphics as graphics
        from machines import GetBallBearingData
        data = GetBallBearingData(axis=[0,0,1], outsideDiameter=0.080,
                                  boreDiameter=0.050, width=0.010, nBalls=12)
        graphicsData = graphics.BallBearingRings(**data)
        #... graphicsData now contains graphics of rings
    """
    outsideRadius = 0.5*outsideDiameter
    boreRadius = 0.5*boreDiameter
    axis0 = np.array(axis)/np.linalg.norm(axis)

    #ring graphics:
    deltaSpaceInner = innerGrooveTorusRadius - innerRingShoulderRadius
    deltaSpaceOuter = outerRingShoulderRadius - outerGrooveTorusRadius
    phiInner = np.arcsin(deltaSpaceInner/innerGrooveRadius)
    phiOuter = np.arcsin(deltaSpaceOuter/outerGrooveRadius)

    if (np.isnan(phiInner)):
        raise ValueError('graphics.BallBearingRings: illegal bearing dimensions, thus groove cannot be calculated; '+
                         'check relations of innerGrooveTorusRadius, innerRingShoulderRadius and innerGrooveRadius')
    if (np.isnan(phiOuter)):
        raise ValueError('graphics.BallBearingRings: illegal bearing dimensions, thus groove cannot be calculated; '+
                         'check relations of outerRingShoulderRadius, outerGrooveTorusRadius and outerGrooveRadius')

    #++++++++++++++++++++++++++++++++
    #inner ring:
    contour=[[ 0.5*width, innerRingShoulderRadius],
             [ 0.5*width, boreRadius+innerEdgeChamfer],
             [ 0.5*width-innerEdgeChamfer, boreRadius],
             [-0.5*width+innerEdgeChamfer, boreRadius],
             [-0.5*width, boreRadius+innerEdgeChamfer],
             [-0.5*width, innerRingShoulderRadius],
             ]
    
    for i in range(nTilesGrooves+1):
        phi = phiInner + i/nTilesGrooves*(pi-2*phiInner)
        contour.append([-innerGrooveRadius*cos(phi),-innerGrooveRadius*sin(phi)+innerGrooveTorusRadius])

    contour.append([ 0.5*width, innerRingShoulderRadius]) #close
    
    innerRingGraphics = SolidOfRevolution(pAxis=[0,0,0], vAxis=axis0, contour=contour, 
                                          color=colorInnerRing, nTiles=nTilesRings, 
                                          smoothContour=True, smoothingAngle=0.24*pi) #smooth everything < 45°

    #++++++++++++++++++++++++++++++++
    #outer ring:
    contour=[
             [-0.5*width, outerRingShoulderRadius],
             [-0.5*width, outsideRadius-outerEdgeChamfer],
             [-0.5*width+outerEdgeChamfer, outsideRadius],
             [ 0.5*width-outerEdgeChamfer, outsideRadius],
             [ 0.5*width, outsideRadius-outerEdgeChamfer],
             [ 0.5*width, outerRingShoulderRadius],
             ]
    
    for i in range(nTilesGrooves+1):
        phi = phiOuter + i/nTilesGrooves*(pi-2*phiOuter)
        contour.append([ outerGrooveRadius*cos(phi),outerGrooveRadius*sin(phi)+outerGrooveTorusRadius])

    contour.append([-0.5*width, outerRingShoulderRadius]) #close
    
    outerRingGraphics = SolidOfRevolution(pAxis=[0,0,0], vAxis=axis0, contour=contour, 
                                          color=colorOuterRing, nTiles=nTilesRings, 
                                          smoothContour=True, smoothingAngle=0.24*pi) #smooth everything < 45°
    
    #++++++++++++++++++++++++++++++++
    #cage approximated as ring
    cageGraphics = Cylinder(pAxis=-0.5*widthCage*axis0, vAxis=widthCage*axis0, 
                            radius=radiusCage+0.5*heightCage,
                            radiusInner=radiusCage-0.5*heightCage,
                            color=colorCage, nTiles=nTilesRings)
    
    graphicsData = {'innerRingGraphics':innerRingGraphics,
                    'outerRingGraphics':outerRingGraphics,
                    'cageGraphics':cageGraphics,
                    }

    return graphicsData


@_ReturnsRows
def InvoluteGear(involuteGear, width, 
                 centerPoint=np.zeros(3), rotationMatrix = np.eye(3), 
                 helixAngleDeg=0, radius=0, relativeAngleOffset=0, 
                 color=[0,0,0,1], nTilesCylinder=32, smoothNormals = False, addEdges = False, 
                 edgeColor=color.black, addFaces=True,
                 ):
    """create graphics for involute gear, using data from machines.InvoluteGear

    Args:
        involuteGear: an instance of the class machines.InvoluteGear, containing gear data
        width: width of gear
        centerPoint: used to shift the center point of the gear; if 0, the center is in the middle of the gear
        rotationMatrix: the gear is constructed in the x-y plane, with the gear axis [0,0,1]; to get any other axis, provide the rotation matrix
        helixAngleDeg: optional angle for helix gears in degree; note that this is only an approximation to real helical gear geometry!
        radius: in case of internal gear, this is the outer radius; for regular gear, this is the bore radius
        relativeAngleOffset: angular offset (about gear wheel axis) relative to the angle of one tooth and gap; 0.5 means that the tooth goes to the position of the gap
        color: provided as list of 4 RGBA values
        smoothNormals: if True, algorithm tries to smoothen normals at vertices and normals are added; creates more points; if False, triangle normals are used internally
        addEdges: if True or 1, edges at bottom/top are included in the GraphicsData dictionary; if 2, also mantle edges are included
        edgeColor: optional color for edges
        addFaces: if False, no faces are added (only edges)

    Returns:
        single graphics data for gear
    """
    gearPoints = involuteGear.GenerateGear()
    baseCircleDiameter = involuteGear.module*involuteGear.nTeeth
    rotatedGearPoints = gearPoints @ RotationMatrix2D(relativeAngleOffset*involuteGear.angleToothAndGap)

    points = rotatedGearPoints.tolist()
    
    segments = gdu.SegmentsFromPoints(rotatedGearPoints).tolist()
    
    if (radius != 0 and not involuteGear.isInternalGear) or involuteGear.isInternalGear:
        [pointsCircle, segmentsCircle] = gdu.CirclePointsAndSegments(radius=radius, invert=False,
                                                                 nTiles=nTilesCylinder)
        nPointsOff = len(points)
        points += pointsCircle
        for seg in segmentsCircle:
            segments.append([seg[0]+nPointsOff,seg[1]+nPointsOff])

    if involuteGear.isInternalGear:
        segments.reverse()

    beta = radians(helixAngleDeg)
    rotationZ = width/(0.5*baseCircleDiameter)*tan(beta)
    
    graphicsData = SolidExtrusion(points, segments, 
                                  width, color=color,
                                  pOff=np.array(centerPoint)+[0,0,-0.5*width],
                                  rot=rotationMatrix@RotationMatrixZ(-0.5*rotationZ),
                                  relRot=RotationMatrixZ(0.5*rotationZ),
                                  smoothNormals=smoothNormals, addEdges=addEdges,
                                  edgeColor=edgeColor, addFaces=addFaces)
    
    return graphicsData
    




@_ReturnsRows
def ToothedRack(module, nTeeth, width, toothHeight, rackBaseHeight,
                pressureAngleDeg=20,
                centerPoint=np.zeros(3), rotationMatrix = np.eye(3), 
                color=[0,0,0,1], nTilesCylinder=32, addEdges = False, 
                edgeColor=color.black, addFaces=True,
                ):
    """create graphics for toothed rack

    Args:
        module: the module in m; thus, m*pi represents the mid-distance of one tooth to the next one
        width: width of gear
        nTeeth: number of teeth used; this gives the length; if this is a float number, only part of the last root or tooth are drawn accordingly
        toothHeight: height of tooth from root to head
        rackBaseHeight: height of rack below root
        pressureAngleDeg: pressure angle in degree for tooth shape
        centerPoint: used to shift the center point of the gear; if 0, the center is at the start point of the generated toothed rack (x=0,y=0), z=0 is in the middle of the rack
        rotationMatrix: the gear is constructed in the x-y plane, with width along z-axis
        color: provided as list of 4 RGBA values
        smoothNormals: if True, algorithm tries to smoothen normals at vertices and normals are added; creates more points; if False, triangle normals are used internally
        addEdges: if True or 1, edges at bottom/top are included in the GraphicsData dictionary; if 2, also mantle edges are included
        edgeColor: optional color for edges
        addFaces: if False, no faces are added (only edges)

    Returns:
        single graphics data for gear
    """
    from math import tan

    p = module*pi
    length = nTeeth*p
    h0 = rackBaseHeight
    h1 = toothHeight + rackBaseHeight
    pressureAngle = radians(pressureAngleDeg)
    xTooth = toothHeight*tan(pressureAngle)
    
    points = [[length,0],[0,0]]
    
    # xOff = -0.5*xTooth
    xOff = -0.5*(0.5*p-xTooth)
    for i in range(int(nTeeth)):
        points.append([max(0,i*p+xOff),h0])
        points.append([(i+0.5)*p-xTooth+xOff,h0])
        points.append([(i+0.5)*p+xOff,h1])
        points.append([(i+1)*p-xTooth+xOff,h1])

    points.append([nTeeth*p+xOff,h0])
    points.append([nTeeth*p,h0])
        
    segments = []        
    nPoints = len(points)
    for k, point in enumerate(points):
        segments.append([(k+1)%nPoints,k])

    graphicsData = SolidExtrusion(points, segments, 
                                  width, color=color,
                                  pOff=np.array(centerPoint)+[0,0,-0.5*width],
                                  rot=rotationMatrix,
                                  smoothNormals=False, addEdges=addEdges,
                                  edgeColor=edgeColor, addFaces=addFaces)
    
    return graphicsData




#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


def BoundingBoxSingle(graphicsData):
    """compute bounding box of single graphicsData

    Args:
        graphicsData: a single Exudyn GraphicsData object

    Returns:
        :list: [bmin, bmax]; tuple of np.array shape (3,), or (None, None) if no points.
    """

    gtype = graphicsData.get('type', None)
    if gtype is None:
        raise ValueError("BoundingBoxSingle Missing 'type' in graphicsData.")

    if gtype == 'TriangleList':
        pts = np.array(graphicsData.get('points', []), dtype=float)
        if pts.size == 0:
            return [None, None]
        pts = pts.reshape((-1, 3))
        return [pts.min(axis=0), pts.max(axis=0)]

    elif gtype == 'Lines':
        points = np.array(graphicsData.get('points', []), dtype=float)
        if points.size == 0:
            return [None, None]
        points = points.reshape((-1, 3))
        return [points.min(axis=0), points.max(axis=0)]

    elif gtype == 'Line':
        data = np.array(graphicsData.get('data', []), dtype=float)
        if data.size == 0:
            return [None, None]
        data = data.reshape((-1, 3))
        return [data.min(axis=0), data.max(axis=0)]

    elif gtype == 'Text':
        pos = np.array(graphicsData.get('position', []), dtype=float)
        if pos.size == 0:
            return [None, None]
        pos = pos.reshape((3,))
        # Text treated as point bbox
        return [pos.copy(), pos.copy()]

    elif gtype == 'Spheres':
        points = np.array(graphicsData.get('points', []), dtype=float).reshape((-1, 3))
        if points.size == 0:
            return [None, None]
        radii = np.array(graphicsData.get('radii', 0.1), dtype=float).reshape((-1, 1))
        return [(points - radii).min(axis=0), (points + radii).max(axis=0)]

    elif gtype == 'Circle':
        c = np.array(graphicsData.get('position', []), dtype=float)
        if c.size == 0:
            return [None, None]
        c = c.reshape((3,))
        r = float(graphicsData.get('radius', 0.0))
        if r < 0:
            raise ValueError("BoundingBoxSingle Circle radius must be non-negative.")

        ext = [r,r,0] #only extend in x/y directions
        return [c - ext, c + ext]

    else:
        raise ValueError(f"BoundingBoxSingle unsupported graphics data type '{gtype}'")

def BoundingBox(graphicsData):
    """compute bounding box of single GraphicsData or list of GraphicsData

    Args:
        graphicsData: a single Exudyn GraphicsData object or list

    Returns:
        :list: [bmin, bmax]; tuple of np.array shape (3,), or (None, None) if no points.
    """
    def _merge_bbox(bmin, bmax, cmin, cmax):
        """Merge two axis-aligned bounding boxes."""
        if cmin is None or cmax is None:
            return [bmin, bmax]
        if bmin is None:
            return [cmin.copy(), cmax.copy()]
        return [np.minimum(bmin, cmin), np.maximum(bmax, cmax)]

    if isinstance(graphicsData, dict):
        return BoundingBoxSingle(graphicsData)
    elif isinstance(graphicsData, list):
        bmin = None
        bmax = None
        for g in graphicsData:
            [cmin, cmax] = BoundingBoxSingle(g)
            [bmin, bmax] = _merge_bbox(bmin, bmax, cmin, cmax)
        return [bmin, bmax]
    else:
        raise ValueError("BoundingBox: graphicsData must be dict or list")


@_ReturnsRows
def FromPointsAndTrigs(points, triangles, color=[0.,0.,0.,1.], normals=None):
    """convert triangles and points as returned from graphics.ToPointsAndTrigs(...) to GraphicsData; additionally, normals and color(s) can be provided

    Args:
        points: list or np.array with np rows of 3 columns (floats) per point (with np points)
        triangles: list or np.array with 3 int per triangle (0-based indices to triangles), giving a matrix with nt rows and 3 columns (with nt triangles); a matrix with 6 columns gives 6-node (curved) triangles, corners counter-clockwise and then the mid-side nodes 01, 12, 20 (key 'triangles6')
        color: provided as list of 4 RGBA values or single list of (np)*[4 RGBA values]
        normals: if not None, they have to be provided per point (as matrix, list of lists or flattened) and will be added to returned GraphicsData

    Returns:
        returns GraphicsData with type TriangleList
    """
    pointList = np.array(points).flatten()
    triangleKey = 'triangles6' if np.array(triangles).ndim == 2 and np.array(triangles).shape[1] == 6 else 'triangles'
    triangleList = np.array(triangles).flatten()
    nPoints = int(len(pointList)/3)
    if isinstance(color,np.ndarray):
        if color.shape[0] == nPoints and color.shape[1] == 4:
            colorList = np.array(color).flatten() #without list() potential problem with mutable default value
        else:
            raise ValueError('FromPointsAndTrigs: invalid numpy array for color (check size and dimensions or provide as list)')
    elif len(color) == 4*nPoints:
        colorList = np.array(color)
    elif len(color) == 4:
        colorList = np.tile(color, nPoints)
    else:
        exudyn.Print('number of points=', nPoints)
        exudyn.Print('number of trigs=', len(triangleList)/3)
        exudyn.Print('number of colors=', len(color))
        raise ValueError('FromPointsAndTrigs: color must have either 4 RGBA values or 4*(number of points) RGBA values as a list')
    data = {'type':'TriangleList',
            'colors': colorList,
            'points':pointList,
            triangleKey:triangleList}
    if normals is not None: 
        data['normals'] = np.array(normals).flatten()
    return data



#************************************************
def ToPointsAndTrigs(g):
    """convert graphics data into list of points and list of triangle indices (triplets)

    Args:
        g contains a GraphicsData with type TriangleList

    Returns:
        returns [points, triangles], with points as list of np.array with 3 floats per point and triangles as a list of np.array with 3 int per triangle (0-based indices to points)
    """
    g = _Flat(Triangles6ToTriangles(SpheresToTriangleList(g)))
    if g['type'] == 'TriangleList':
        nPoints=int(len(g['points'])/3)
        points = [np.zeros(3)]*nPoints
        for i in range(nPoints):
            points[i] = np.array(g['points'][i*3:i*3+3])
        
        nTrigs=int(len(g['triangles'])/3)
        triangles = [np.zeros(3, dtype=int)]*nTrigs
        for i in range(nTrigs):
            triangles[i] = np.array(g['triangles'][i*3:i*3+3], dtype=int)
    else:
        raise ValueError ('ERROR: ToTrigsAndPoints(...) only takes GraphicsData of type TriangleList but found: '+
                          g['type'] )

    return [points, triangles]


#************************************************
@_ReturnsRows
def Transform(graphicsData, translation=None, rotation=None, scale=1,
              normalizeNormals=False, invertNormals=False, invertTriangles=False,
              warn=True):
    """transform a GraphicsData object in several ways: move, rotate, scale; furthermore, normals can be fixed and inverted, etc.

    Args:
        g: graphicsData to be transformed
        translation: 3D offset as list or numpy.array added to rotated points; if pOff=None, no translation is applied
        rotation: 3D rotation matrix as list of lists or numpy.array with shape (3,3); if A is scaled by factor, e.g. using 0.001*np.eye(3), you can also scale the coordinates; if Aoff=None, no rotation is performed
        scale: scaling of position coordinates
        normalizeNormals: if True, normals are scaled such that length=1 (or zero for zero-normals)
        invertTriangles: if True, it inverts the triangle orientation (changing vertex index 0 and 1)
        invertNormals: if True, the direction of normal is flipped

    Returns:
        returns new graphcsData object to be used for drawing in objects

    Note:
        the rigid body transformation corresponds to HomogeneousTransformation(rotation, translation), transforming original coordinates v into vNew = translation + rotation @ v
    """
    
    graphicsData = _Flat(graphicsData)
    if translation is None:
        translation = [0,0,0]
    if rotation is None:
        rotation = np.eye(3)
    
    p0 = np.array(translation)
    if rotation is  None:
        A0 = np.eye(3)
    else:
        A0 = np.array(rotation)
    
    if graphicsData['type'] == 'TriangleList': 
        gNew = {'type':'TriangleList'}
        gNew['colors'] = np.array(graphicsData['colors'])
        if invertTriangles and 'triangles' in graphicsData:
            nTrigs=int(len(graphicsData['triangles'])/3)
            triangles = np.array(graphicsData['triangles']).reshape((nTrigs,3))
        
            if invertTriangles:
                for i, trig in enumerate(triangles):
                    t0 = trig[0]
                    trig[0]=trig[1]
                    trig[1] = t0
                gNew['triangles'] = triangles.flatten()
        elif 'triangles' in graphicsData:
            gNew['triangles'] = np.array(graphicsData['triangles'])
        if 'triangles6' in graphicsData:
            triangles6 = np.array(graphicsData['triangles6'], dtype=int).reshape((-1, 6))
            if invertTriangles: #corners c0,c1 and the mid-side nodes m12,m20 swap (#2709)
                triangles6 = triangles6[:, [1, 0, 2, 3, 5, 4]]
            gNew['triangles6'] = triangles6.flatten()

        if 'edges' in graphicsData:
            gNew['edges'] = np.array(graphicsData['edges'])
        if 'edges3' in graphicsData:
            gNew['edges3'] = np.array(graphicsData['edges3'])
        if 'edgeColor' in graphicsData:
            gNew['edgeColor'] = np.array(graphicsData['edgeColor'])

        n=int(len(graphicsData['points'])/3)
        v0 = np.array(graphicsData['points'])
        v = np.kron(np.ones(n),p0) + scale*(A0 @ v0.reshape((n,3)).T).T.flatten()
        
        gNew['points'] = v
        if 'normals' in graphicsData:
            n0 = np.array(graphicsData['normals'])
            normals = n0.reshape((n,3))
            
            if normalizeNormals:
                if normals.ndim != 2 or normals.shape[1] != 3:
                    raise ValueError("graphics.Transform: Expected array of shape (n,3).")
            
                norms = np.linalg.norm(normals, axis=1, keepdims=True) 
                
                normals = np.divide(normals, norms,
                                out=np.zeros_like(normals),
                                where=(norms != 0) )
            
                zero_count = np.count_nonzero(norms == 0)
                if warn and zero_count:
                    exudyn.Print(f"Warning: graphics.Transform: {zero_count} zero-length normals found; left as zeros.")
                
            if invertNormals:
                normals *= -1
            
            gNew['normals'] = (A0 @ normals.T).T.flatten()
        
    elif graphicsData['type'] == 'Spheres':
        #a rotation times a uniform factor keeps a sphere a sphere; any other matrix makes an ellipsoid, which needs triangles
        uniformFactor2 = np.trace(A0 @ A0.T)/3
        if np.abs(A0 @ A0.T - uniformFactor2*np.eye(3)).max() > 1e-12*max(1., uniformFactor2):
            return Transform(SpheresToTriangleList(graphicsData), translation=translation, rotation=rotation, scale=scale,
                             normalizeNormals=normalizeNormals, invertNormals=invertNormals, invertTriangles=invertTriangles, warn=warn)
        gNew = copy.deepcopy(graphicsData)
        v0 = np.array(graphicsData['points'], dtype=float).reshape((-1, 3))
        gNew['points'] = (p0 + scale*(A0 @ v0.T).T).flatten()
        gNew['radii'] = scale*np.sqrt(uniformFactor2)*np.array(graphicsData.get('radii', 0.1), dtype=float)
    elif graphicsData['type'] == 'Lines': #any shape: only the points move
        gNew = copy.deepcopy(graphicsData)
        points = np.array(graphicsData['points'], dtype=float)
        rows = points.reshape((-1, 3))
        gNew['points'] = (p0 + scale*(A0 @ rows.T).T).reshape(points.shape)
    elif graphicsData['type'] == 'Line':
        gNew = copy.deepcopy(graphicsData)
        n=int(len(graphicsData['data'])/3)
        for i in range(n):
            v = gNew['data'][i*3:i*3+3]
            v = p0 + A0 @ v
            gNew['data'][i*3:i*3+3] = v
    elif graphicsData['type'] == 'Text':
        gNew = copy.deepcopy(graphicsData)
        v = p0 + A0 @ gNew['position']
        gNew['position'] = v
    elif graphicsData['type'] == 'Circle':
        gNew = copy.deepcopy(graphicsData)
        v = p0 + A0 @ gNew['position']
        gNew['position'] = v
        if 'normal' in gNew:
            v = A0 @ gNew['normal']
            gNew['normal'] = v
    else:
        raise ValueError('Move: unsupported graphics data type')
    return gNew



#************************************************
@_ReturnsRows
def Move(g, pOff, Aoff=None):
    """add rigid body transformation and possible scaling to GraphicsData, using position offset (global) pOff (list or np.array) and rotation Aoff (transforms local to global coordinates; list of lists or np.array)

    Args:
        g: graphicsData to be transformed
        pOff: 3D offset as list or numpy.array added to rotated points
        Aoff: 3D rotation matrix as list of lists or numpy.array with shape (3,3); if Aoff=None, no rotation is performed

    Returns:
        returns new graphcsData object to be used for drawing in objects

    Note:
        transformation corresponds to HomogeneousTransformation(Aoff, pOff), transforming original coordinates v into vNew = pOff + Aoff @ v
    """
    return Transform(graphicsData=g, translation=pOff, rotation=Aoff)

#************************************************
@_ReturnsRows
def MergeTriangleLists(g1,g2):
    """merge 2 different graphics data with triangle lists

    Args:
        graphicsData dictionaries g1 and g2 obtained from GraphicsData functions

    Returns:
        one graphicsData dictionary with single triangle lists and compatible points and normals, to be used in visualization of EXUDYN objects; edges are merged; edgeColor is taken from graphicsData g1
    """
    g1 = _Flat(SpheresToTriangleList(g1))
    g2 = _Flat(SpheresToTriangleList(g2))
    nPoints = int(len(g1['points'])/3) #number of points in g1
    useNormals = False
    if 'normals' in g1 and 'normals' in g2:
        useNormals = True

    if nPoints*4 != len(g1['colors']):
        raise ValueError('MergeTriangleLists: incompatible colors and points in lists')

    if useNormals:
        if nPoints*3 != len(g1['normals']):
            raise ValueError('MergeTriangleLists: incompatible normals and points in lists')
        data = {'type':'TriangleList', 'colors':np.array(g1['colors']), 'normals':np.array(g1['normals']),
                'points': np.array(g1['points']), 'triangles': np.array(g1.get('triangles', []), dtype=int)}

        data['normals'] = np.append(data['normals'],g2['normals'])
    else:
        data = {'type':'TriangleList', 'colors':np.array(g1['colors']),
                'points': np.array(g1['points']), 'triangles': np.array(g1.get('triangles', []), dtype=int)}
    
    data['colors'] = np.append(data['colors'], g2['colors'])
    data['points'] = np.append(data['points'], g2['points'])

    # for p in g2['triangles']:
    #     data['triangles'] += [int(p + nPoints)] 
    data['triangles'] = np.append(data['triangles'], np.array(g2.get('triangles', []), dtype=int)+nPoints ) #add nPoints offset to g2 for correct connectivity
    if 'triangles6' in g1 or 'triangles6' in g2: #6-node triangles (#2709)
        data['triangles6'] = np.append(np.array(g1.get('triangles6', []), dtype=int),
                                       np.array(g2.get('triangles6', []), dtype=int)+nPoints)

    #merge edges; edges can be available only in one triangle list; those of g2 refer to its points, which follow those of g1 (#2769)
    if 'edges' in g1 or 'edges' in g2:
        data['edges'] = np.append(np.array(g1.get('edges', []), dtype=int),
                                  np.array(g2.get('edges', []), dtype=int)+nPoints)
    if 'edges3' in g1 or 'edges3' in g2: #quadratic edges (#2709)
        data['edges3'] = np.append(np.array(g1.get('edges3', []), dtype=int),
                                   np.array(g2.get('edges3', []), dtype=int)+nPoints)
    if 'edgeColor' in g1:
        data['edgeColor'] = np.array(g1['edgeColor']) #only taken from g1, as there is only a single color
    elif 'edgeColor' in g2:
        data['edgeColor'] = np.array(g2['edgeColor']) #only taken from g2


    return data

#************************************************
@_ReturnsRows
def InvertTriangles(graphicsData, invertTriangles=True, invertNormals=True):
    """invert triangle orientation and triangle normals (or only one of these tasks); can also check consistency of normals

    Args:
        graphicsData: graphicsData as returned e.g. from graphics.Sphere
        invertTriangles: if True, it inverts the triangle orientation (changing vertex index 0 and 1)
        invertNormals: if True, the direction of normal is flipped

    Returns:
        returns new graphicsData (copy) with modified triangles and normals
    """
    graphicsData = _Flat(SpheresToTriangleList(graphicsData))
    if graphicsData['type'] != 'TriangleList':
        raise ValueError('InvertTriangles only works for graphicsData of TriangleList type')
    if 'normals' not in graphicsData and invertNormals:
        raise ValueError('InvertTriangles requires normals in TriangleList if invertNormals=True')

    gNew = {'type':'TriangleList'}
    gNew['points'] = np.array(graphicsData['points']) #copy
    gNew['colors'] = np.array(graphicsData['colors']) #copy
    gNew['triangles'] = np.array(graphicsData.get('triangles', []), dtype=int) #copy
    if 'triangles6' in graphicsData: #corners c0,c1 and the mid-side nodes m12,m20 swap (#2709)
        triangles6 = np.array(graphicsData['triangles6'], dtype=int).reshape((-1, 6))
        gNew['triangles6'] = (triangles6[:, [1, 0, 2, 3, 5, 4]] if invertTriangles else triangles6).flatten()

    nPoints=int(len(graphicsData['points'])/3)
    nTrigs=int(len(gNew['triangles'])/3)

    if 'normals' in graphicsData:
        gNew['normals'] = np.array(graphicsData['normals']).reshape((nPoints,3)) #copy

    if 'edges' in graphicsData:
        gNew['edges'] = np.array(graphicsData['edges']) #copy
    if 'edgeColor' in graphicsData:
        gNew['edgeColor'] = np.array(graphicsData['edgeColor']) #copy

    #points = np.array(graphicsData['points']).reshape((nPoints,3))
    triangles = np.array(gNew['triangles']).reshape((nTrigs,3))

    if invertTriangles:
        for i, trig in enumerate(triangles):
            t0 = trig[0]
            trig[0]=trig[1]
            trig[1] = t0
        gNew['triangles'] = triangles.flatten()

    if invertNormals:
        for i, normal in enumerate(gNew['normals']):
            gNew['normals'][i] = -normal

    if 'normals' in graphicsData:
        gNew['normals'] = gNew['normals'].flatten()

    return gNew

def InconsistentTriangles(graphicsData):
    """check consistency of orientation of triangles and vertex (point) normals

    Args:
        graphicsData: graphicsData as returned e.g. from graphics.Sphere

    Returns:
        returns number of cases in which triangle normals and vertex normals are inconsistent (scalar product is negative)
    """
    graphicsData = _Flat(Triangles6ToTriangles(SpheresToTriangleList(graphicsData)))
    if graphicsData['type'] != 'TriangleList':
        raise ValueError('InconsistentTriangles only works for graphicsData of TriangleList type')
    if 'normals' not in graphicsData:
        raise ValueError('InconsistentTriangles requires normals in TriangleList')

    nPoints=int(len(graphicsData['points'])/3)
    nTrigs=int(len(graphicsData['triangles'])/3)

    triangles = np.array(graphicsData['triangles']).reshape((nTrigs,3))
    points = np.array(graphicsData['points']).reshape((nPoints,3))
    normals = np.array(graphicsData['normals']).reshape((nPoints,3))

    cntWrong = 0
    for i, trig in enumerate(triangles):
        normalTrig = gdu.ComputeTriangleNormal(points[trig[0]],points[trig[1]],points[trig[2]])
        for j in range(3):
            if normals[trig[j]] @ normalTrig < 0:
                cntWrong+=1

    return cntWrong

def NGsolveMesh2PointsAndTrigs(mesh=None, ngMesh=None, meshOrder=2, scale=1, addNormals=True, verbose=False, triangles6=False):
    """convert NGsolve (surface) mesh into (surface) points and triangles; clearly, it requires to have ngsolve installed

    Args:
        mesh: a ngsolve mesh; having a geometry geo = OCCGeometry(...), mesh is returned from ngsolve.Mesh(geo.GenerateMesh(...))
        ngMesh: a netgen mesh; having a geometry geo = OCCGeometry(...), ngMesh is returned from geo.GenerateMesh(...)
        meshOrder: either 1 (linear, flat triangles) or 2 (quadratic, smooth triangles)
        scale: additional scaling factor for geometry, as it is recommended to define netgen geometries in mm due to tolerances
        addNormals: if True, it computes and adds normals
        verbose: print debug information
        triangles6: with meshOrder=2, return the elements as 6-node triangles (6 indices per row, for FromPointsAndTrigs), drawn curved, instead of 4 flat triangles each

    Returns:
        [points, triangles] or if addNormals=True, [points, triangles, normals] for further usage in graphics.FromPointsAndTrigs(...)

    Example:
        #assume having already a body of netgen OCCGeometry
        geo = OCCGeometry(body)
        ngMesh = geo.GenerateMesh(maxh=maxh)
        #convert mesh into points, triangles and normals (with second-order elements!)
        [points, triangles, normals] = graphics.NGsolveMesh2PointsAndTrigs(mesh=ngMesh)
        #convert into graphicsData
        gMesh = graphics.FromPointsAndTrigs( points, triangles, normals=normals,
                                            color=graphics.color.red)
        #use the mesh on a ground object
        mbs.CreateGround(graphicsDataList=[gMesh])
    """
    if mesh is not None:
        if ngMesh is not None:
            raise ValueError('NGsolveMesh2PointsAndTrigs; either mesh or ngMesh must be None!')
        ngMesh = mesh.ngmesh
    else:
        if ngMesh is None:
            raise ValueError('NGsolveMesh2PointsAndTrigs; either mesh or ngMesh must not be None!')
    
    meshPoints=[]
    if meshOrder == 2:
        ngMesh.SecondOrder()

    NP = len(ngMesh.Points())
    if verbose: exudyn.Print("number of meshPoints=", NP)

    for n in ngMesh.Points(): 
        meshPoints+=[np.array(list(n))]

    surfaceElems = ngMesh.Elements2D() #surface mesh
    if verbose: exudyn.Print('number of surface elems=',len(surfaceElems))

    points3 = []    #3 per triangle, if addNormals=True
    normals = []    #1 per point3, if addNormals=True
    triangles=[] 
    # listTexts = []
   
    #ordering of sub-triangles for visualization
    subTrigs = [[0,5,4],
                [5,1,3],
                [5,3,4],
                [4,3,2]]
    
    cntPoints = 0
    if meshOrder == 1:
        for st in surfaceElems: 
            vertices = []
            for v in st.vertices: #st.meshPoints gives all nodes (for order>1), vertices only vertex meshPoints (always 4 per tet)
                vertices += [v.nr-1] #convert to 0-based indices
            if len(vertices) != 3:
                raise ValueError('ImportMeshFromNGsolve: expected linear 3-node surface elements')

            if not addNormals:
                triangles += [vertices]
            else:
                triangles += [[cntPoints, cntPoints+1, cntPoints+2]]
                cntPoints += 3
                n = gdu.ComputeTriangleNormal(meshPoints[vertices[0]], 
                                          meshPoints[vertices[1]], 
                                          meshPoints[vertices[2]])
                normals.append(list(n))
                normals.append(list(n))
                normals.append(list(n))
                points3.append(meshPoints[vertices[0]])
                points3.append(meshPoints[vertices[1]])
                points3.append(meshPoints[vertices[2]])
    else: #order 2
        
        for el, st in enumerate(surfaceElems): 
            #for these elements, we could compute some improved normals ...
            w = []
            for v in st.points: #st.meshPoints gives all nodes (for order>1), vertices only vertex meshPoints (always 4 per tet)
                w += [v.nr-1] #convert to 0-based indices
            if len(w) != 6:
                raise ValueError('ImportMeshFromNGsolve: expected second order 6-node surface elements')
            if triangles6: #NETGEN numbers the mid-side nodes 3 (12), 4 (20), 5 (01); Exudyn 01, 12, 20 (#2709)
                order6 = [0, 1, 2, 5, 3, 4]
                if not addNormals:
                    triangles += [[w[k] for k in order6]]
                else:
                    n6 = gdu.Compute6NodeTrigsNormals([meshPoints[w[k]] for k in range(6)])
                    triangles += [list(range(cntPoints, cntPoints+6))]
                    cntPoints += 6
                    normals += [n6[k] for k in order6]
                    points3 += [meshPoints[w[k]] for k in order6]
                continue
            if not addNormals:
                #convert into 4 triangles
                for k, subTrig in enumerate(subTrigs):
                    triangles += [[w[subTrig[0]],w[subTrig[1]],w[subTrig[2]] ]]
            else:
                n6 = gdu.Compute6NodeTrigsNormals([meshPoints[w[0]],meshPoints[w[1]],meshPoints[w[2]],
                                                meshPoints[w[3]],meshPoints[w[4]],meshPoints[w[5]], ])
                # n = gdu.ComputeTriangleNormal(meshPoints[w[0]], meshPoints[w[1]], meshPoints[w[2]])
                # visualize normals and node numbers
                # mp = 1/3*(np.array(meshPoints[w[0]]) 
                #           + np.array(meshPoints[w[1]])
                #           + np.array(meshPoints[w[2]]))

                # for i in range(6):
                #     # listTexts.append(graphics.Text(0.8*meshPoints[w[i]]+0.1*n+0.2*mp, 
                #     #                                'El'+str(el)+'-N'+str(w[i])+'-'+str(i)))
                #     listTexts.append(graphics.Arrow(meshPoints[w[i]], n6[i], 0.025,graphics.color.orange))

                # # listTexts.append(graphics.Arrow(mp, 2*n, 0.025,graphics.color.red))
                    
                for k, subTrig in enumerate(subTrigs):

                    triangles += [[cntPoints, cntPoints+1, cntPoints+2]]
                    cntPoints += 3
                    normals += [n6[subTrig[0]], n6[subTrig[1]], n6[subTrig[2]], ]
                    points3 += [meshPoints[w[subTrig[0]]], 
                                meshPoints[w[subTrig[1]]], 
                                meshPoints[w[subTrig[2]]] ]

    
    if addNormals:
        return [scale*np.array(points3), np.array(triangles), np.array(normals)]
    else:
        return [scale*np.array(meshPoints), np.array(triangles)]



@_ReturnsRows
def FromSTLfileASCII(fileName, color=[0.,0.,0.,1.], verbose=False, invertNormals=True, invertTriangles=True): 
    """generate graphics data from STL file (text format!) and use color for visualization; this function is slow, use stl binary files with FromSTLfile(...)

    Args:
        fileName: string containing directory and filename of STL-file (in text / SCII format) to load
        color: provided as list of 4 RGBA values
        verbose: if True, useful information is provided during reading
        invertNormals: if True, orientation of normals (usually pointing inwards in STL mesh) are inverted for compatibility in Exudyn
        invertTriangles: if True, triangle orientation (based on local indices) is inverted for compatibility in Exudyn

    Returns:
        creates graphicsData, inverting the STL graphics regarding normals and triangle orientations (interchanged 2nd and 3rd component of triangle index)
    """
#file format, just one triangle, using GOMinspect:
#solid solidName
#facet normal -0.979434 0.000138 -0.201766
# outer loop
#    vertex 9.237351 7.700452 -9.816338
#    vertex 9.237478 10.187849 -9.815249
#    vertex 9.706021 10.170116 -12.089709
# endloop
#endfacet
#...
#endsolid solidName
    if verbose: exudyn.Print("read STL file: "+fileName)

    fileLines = []
    try: #still close file if crashes
        file=open(fileName,'r') 
        fileLines = file.readlines()
    finally:
        file.close()    

    colors=[]
    points = []
    normals = []
    triangles = []

    nf = 1.-2.*int(invertNormals) #+1 or -1 (inverted)
    indOff = int(invertTriangles) #0 or 1 (inverted)

    nLines = len(fileLines)
    lineCnt = 0
    if fileLines[lineCnt][0:5] != 'solid':
        raise ValueError("FromSTLfileTxt: expected 'solid ...' in first line, but received: " + fileLines[lineCnt])
    lineCnt+=1
    
    if nLines > 500000:
        exudyn.Print('large ascii STL file; switch to numpy-stl and binary format for faster loading!')

    while lineCnt < nLines and fileLines[lineCnt].strip().split()[0] != 'endsolid':
        if lineCnt%100000 == 0 and lineCnt !=0: 
            if verbose: exudyn.Print("  read line",lineCnt," / ", len(fileLines))

        normalLine = fileLines[lineCnt].split()
        if normalLine[0] != 'facet' or normalLine[1] != 'normal':
            raise ValueError("FromSTLfileTxt: expected 'facet normal ...' in line "+str(lineCnt)+", but received: " + fileLines[lineCnt])
        if len(normalLine) != 5:
            raise ValueError("FromSTLfileTxt: expected 'facet normal n0 n1 n2' in line "+str(lineCnt)+", but received: " + fileLines[lineCnt])
        
        normal = [nf*float(normalLine[2]),nf*float(normalLine[3]),nf*float(normalLine[4])]

        lineCnt+=1
        loopLine = fileLines[lineCnt].strip()
        if loopLine != 'outer loop':
            raise ValueError("FromSTLfileTxt: expected 'outer loop' in line "+str(lineCnt)+", but received: " + fileLines[lineCnt])

        ind = int(len(points)/3) #index for points of this triangle
        #get 3 vertices:
        lineCnt+=1
        for i in range(3):
            readLine = fileLines[lineCnt].strip().split()
            if readLine[0] != 'vertex':
                raise ValueError("FromSTLfileTxt: expected 'vertex ...' in line "+str(lineCnt)+", but received: " + fileLines[lineCnt])
            if len(readLine) != 4:
                raise ValueError("FromSTLfileTxt: expected 'vertex v0 v1 v2' in line "+str(lineCnt)+", but received: " + fileLines[lineCnt])
            
            points+=[float(readLine[1]),float(readLine[2]),float(readLine[3])]
            normals+=normal
            colors+=color
            lineCnt+=1
            
        triangles+=[ind,ind+1+indOff,ind+2-indOff] #indices of points; flip indices to match definition in EXUDYN

        loopLine = fileLines[lineCnt].strip()
        if loopLine != 'endloop':
            raise ValueError("FromSTLfileTxt: expected 'endloop' in line "+str(lineCnt)+", but received: " + fileLines[lineCnt])
        lineCnt+=1
        loopLine = fileLines[lineCnt].strip()
        if loopLine != 'endfacet':
            raise ValueError("FromSTLfileTxt: expected 'endfacet' in line "+str(lineCnt)+", but received: " + fileLines[lineCnt])
        lineCnt+=1
    
    data = {'type':'TriangleList', 'colors':colors, 'normals':normals, 'points':points, 'triangles':triangles}
    return data


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++
@_ReturnsRows
def FromPyMeshlabFile(fileName, defaultColor=color.defaultBody,
                      invertNormals=False, invertTriangles=False, normalizeNormals=True,
                      useDefaultColor=False, verbose=False):
    """generate graphics data from any file that can be loaded with PyMeshLab (in particular .obj, .dae and .stl); either use defaultColor or given color in mesh.

    Args:
        fileName: string containing directory and filename of geometry file
        defaultColor: provided as list of 4 RGBA values; used only if meshlab cannot load valid color or if file does not include color (e.g., STL)
        verbose: if True, some information is logged during file import
        invertNormals: if True, orientation of normals (usually pointing inwards in STL mesh) are inverted for compatibility in Exudyn
        invertTriangles: if True: triangle orientation (based on local indices) is inverted for compatibility in Exudyn
        normalizeNormals: if True, normals are scaled such that length=1 (or zero for zero-normals)
        useDefaultColor: if True: ignores colors of the loaded mesh and uses defaultColor

    Returns:
        :dict: graphicsData in Exudyn dictionary format

    Note:
        requires pymeshlab to be installed (pip install pymeshlab); materials and textures are currently not considered in the import functionality!
    """
    try:
        import pymeshlab #pip install pymeshlab
    except ImportError:
        raise ImportError('graphics.FromPyMeshlabFile: requires pymeshlab to be installed (not found): pip install pymeshlab')

    ms = pymeshlab.MeshSet()
    #ms.load_new_mesh(fileDir+'ur5_description/visual/base.dae')
    ms.load_new_mesh(fileName)
    mesh = ms.current_mesh()
    
    
    if np.linalg.norm(mesh.transform_matrix()-np.eye(4)) != 0:
        if verbose >= 1: 
            exudyn.Print('FromPyMeshlabFile: mesh has transformation')

    #normals are often imported with wrong scaling ...
    normals = mesh.vertex_normal_matrix()
    
    if normalizeNormals:
        norms = np.linalg.norm(normals, axis=1, keepdims=True) 
        
        normals = np.divide(normals, norms,
                        out=np.zeros_like(normals),
                        where=(norms != 0) )
    
        zero_count = np.count_nonzero(norms == 0)
        if zero_count:
            exudyn.Print(f"Warning: graphics.FromPyMeshlabFile: {zero_count} zero-length normals found; left as zeros.")
    
    triangles = mesh.face_matrix()
    if invertNormals:
        normals *= -1
    if invertTriangles:
        triangles = np.take(triangles, [0,2,1], axis=1) #swap columns

    graphicsData = FromPointsAndTrigs(points = mesh.vertex_matrix(),
                                      triangles = triangles,
                                      normals = normals,
                                      color = defaultColor,
                                      )

    #if mesh has face colors (e.g., .obj files), read them and store in graphicsData:
    if mesh.has_face_color() and not useDefaultColor:
        if verbose >= 1: 
            exudyn.Print('FromPyMeshlabFile: import available face colors')
        faceColors = mesh.face_color_matrix()
        triangles = mesh.face_matrix()
        #convert face colors to vertex colors: (only available for .obj files)
        vertexColors = np.zeros((mesh.vertex_matrix().shape[0],4))
        for it, trig in enumerate(triangles):
            color = faceColors[it, :]
            for vertex in trig:
                vertexColors[vertex,:] = color
        graphicsData['colors'] = np.array(vertexColors).flatten()

    return graphicsData



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@_ReturnsRows
def FromSTLfile(fileName, color=[0.,0.,0.,1.], verbose=False, density=0., scale=1., Aoff=[], pOff=[], invertNormals=True, invertTriangles=True):
    """generate graphics data from STL file, allowing text or binary format; requires numpy-stl to be installed; additionally can scale, rotate and translate

    Args:
        fileName: string containing directory and filename of STL-file (in text / SCII format) to load
        color: provided as list of 4 RGBA values
        verbose: if True, useful information is provided during reading
        density: if given and if verbose, mass, volume, inertia, etc. are computed
        scale: point coordinates are transformed by scaling factor
        invertNormals: if True, orientation of normals (usually pointing inwards in STL mesh) are inverted for compatibility in Exudyn
        invertTriangles: if True, triangle orientation (based on local indices) is inverted for compatibility in Exudyn

    Returns:
        creates graphicsData, inverting the STL graphics regarding normals and triangle orientations (interchanged 2nd and 3rd component of triangle index)

    Note:
        the model is first scaled, then rotated, then the offset pOff is added; finally min, max, mass, volume, inertia, com are computed!
    """
    try:
        from stl import mesh
    except ImportError:
        raise ValueError('FromSTLfile requires installation of numpy-stl; try "pip install numpy-stl"')
    
    data=mesh.Mesh.from_file(fileName)
    nPoints = 3*len(data.points) #data.points has shape (nTrigs,9), one triangle has 3 points!
    
    if scale != 1.:
        data.points *= scale
    
    p = copy.copy(pOff)
    A = copy.deepcopy(Aoff) #deepcopy for list of lists
    
    if not IsEmptyList(p) or not IsEmptyList(A):
        if IsEmptyList(p): p=[0,0,0]
        if IsEmptyList(A): A=np.eye(3)
        HT = HomogeneousTransformation(A, p)
        
        data.transform(HT)
        
    dictData = {}
    if verbose:
        exudyn.Print('FromSTLfile:')
        exudyn.Print('  max point=', list(data.max_))
        exudyn.Print('  min point=', list(data.min_))
        exudyn.Print('  STL points=', nPoints)
    if density != 0:
        [volume, mass, COM, inertia] = data.get_mass_properties_with_density(density)
        dictData = {'minPos':data.min_,
                    'maxPos':data.max_,
                    'volume':volume,
                    'mass':mass,
                    'COM':COM,
                    'inertia':inertia
                    }
    if verbose:
        exudyn.Print('  volume =', volume)
        exudyn.Print('  center of mass =', list(COM))
        exudyn.Print('  inertia =', list(inertia))
    
    colors = np.tile(color, nPoints)

    if invertTriangles:
        triangles = np.arange(nPoints-1,-1,-1)              #inverted sorting
    else:
        triangles = np.arange(0,nPoints)                    #unmodified sorting of indices
    points = data.points.flatten()
    nf = 1.-2.*int(invertNormals)                           #+1 or -1 (inverted)
    normals = np.kron([nf,nf,nf],data.normals).flatten()    #normals must be per point

    dictGraphics = {'type':'TriangleList', 'colors':colors, 'normals':normals, 
                    'points':points, 'triangles':triangles}
    if density == 0:
        return dictGraphics 
    else:
        return [dictGraphics, dictData]


@_ReturnsRows
def AddEdgesAndSmoothenNormals(graphicsData, edgeColor = color.black, edgeAngle = 0.25*pi,
                               addEdges=True, smoothNormals=True, roundDigits=5, 
                               triangleColor = []):
    """compute and return GraphicsData with edges and smoothend normals for mesh consisting of points and triangles (e.g., as returned from GraphicsData2PointsAndTrigs); ignores stored normals
    graphicsData: single GraphicsData object of type TriangleList; existing edges are ignored
    edgeColor: optional color for edges
    edgeAngle: angle above which edges are added to geometry
    addEdges: if True, edges are added in TriangleList of GraphicsData
    smoothNormals: if True, algorithm tries to smoothen normals at vertices; otherwise, uses triangle normals
    roundDigits: number of digits, relative to max dimensions of object, at which points are assumed to be equal; too small or too larger number of digits may cause artifacts
    triangleColor: if triangleColor is set to a RGBA color, this color is used for the new triangle mesh throughout; otherwise, stored colors are unchanged

    Returns:
        returns GraphicsData with added edges and smoothed normals

    Note:
        this function is suitable for STL import; it assumes that all colors in graphicsData are the same and only takes the first color!
    """
    graphicsData = _Flat(graphicsData)
    from math import acos # ,sin, cos

    oldColors = copy.copy(graphicsData['colors']) #2022-12-06: accepts now all colors; graphicsData['colors'][0:4]    
    [points, trigs]=ToPointsAndTrigs(graphicsData)
    # [points, trigs]=RefineMesh(points, trigs)

    nPoints = len(points)
    nColors = int(len(oldColors)/4)

    triangleColorNew = list(triangleColor)

    if nColors != nPoints:
        exudyn.Print('WARNING: AddEdgesAndSmoothenNormals: found inconsistent colors; they must match the point list in graphics data')
        if triangleColorNew == []:
            triangleColorNew = graphicsData['colors'][0:4]

    if len(triangleColorNew) != 4 and len(triangleColorNew) != 0:
        triangleColorNew = [1,0,0,1]
        exudyn.Print('WARNING: AddEdgesAndSmoothenNormals: colors invalid; using default')

    if len(triangleColorNew) == 4:
        oldColors = list(triangleColorNew)*nPoints

    colors = [np.zeros(4)]*nPoints
    for i in range(nPoints):
        colors[i] = np.array(oldColors[i*4:i*4+4])
    
    points = np.array(points)
    trigs = np.array(trigs)
    colors = np.array(colors)
    pMax = np.max(points, axis=0)
    pMin = np.min(points, axis=0)
    maxDim = np.linalg.norm(pMax-pMin)
    if maxDim == 0: maxDim = 1.

    points = maxDim * np.round(points*(1./maxDim),roundDigits)
    
    sortIndices = np.lexsort((points[:,2], points[:,1], points[:,0]))
    #sortedPoints = points[sortIndices]
    
    #now eliminate duplicate points:
    remap = np.zeros(nPoints,dtype=int)#np.int64)
    remap[0] = 0
    newPoints = [points[sortIndices[0],:]] #first point
    newColors = [colors[sortIndices[0],:]]
    
    cnt = 0
    for i in range(len(sortIndices)-1):
        nextIndex = sortIndices[i+1]
        if (points[nextIndex] != points[sortIndices[i]]).any():
            # newIndices.append(nextIndex)
            cnt+=1
            remap[nextIndex] = cnt#i+1
            newPoints.append(points[nextIndex,:])
            newColors.append(colors[nextIndex,:])
        else:
            remap[nextIndex] = cnt#newIndices[sortIndices[i]]
            # newIndices.append(newIndices[-1])
    newPoints = np.array(newPoints)
    newTrigs = remap[trigs]
    
    #==> now we (hopefully have connected triangle lists)
    
    nPoints = len(newPoints)
    nTrigs = len(newTrigs)
    
    #create points2trigs lists:
    points2trigs = [[] for i in range(nPoints)] #[[]]*nPoints does not work!!!!
    for cntTrig, trig in enumerate(newTrigs):
        for ind in trig:
            points2trigs[ind].append(cntTrig)
    
    #now find neighbours, compute triangle normals:
    neighbours = np.zeros((nTrigs,3),dtype=int)
    # neighbours[:,:] = -1#check if all neighbours found
    normals = np.zeros((nTrigs,3)) #per triangle
    areas = np.zeros(nTrigs)
    for cntTrig, trig in enumerate(newTrigs):
        normals[cntTrig,:] = gdu.ComputeTriangleNormal(newPoints[trig[0]], newPoints[trig[1]], newPoints[trig[2]])
        areas[cntTrig] = gdu.ComputeTriangleArea(newPoints[trig[0]], newPoints[trig[1]], newPoints[trig[2]])
        for cntNode in range(3):
            ind  = trig[cntNode]
            ind2 = trig[(cntNode+1)%3]
            for t in points2trigs[ind]:
                #if t <= cntTrig: continue #too much sorted out; check why
                trig2=newTrigs[t]
                found = False
                for cntNode2 in range(3):
                    if trig2[cntNode2] == ind2 and trig2[(cntNode2+1)%3] == ind:
                        neighbours[cntTrig, cntNode] = t
                        found = True
                        break
                if found: break
    
    #create edges:
    edges = [] #list of edge points
    pointHasEdge = [False]*nPoints
    for cntTrig, trig in enumerate(newTrigs):
        for cntNode in range(3):
            ind1  = trig[cntNode]
            ind2 = trig[(cntNode+1)%3]
            if ind1 > ind2:
                val = normals[cntTrig] @ normals[neighbours[cntTrig,cntNode]]
                if abs(val) > 1: val = np.sign(val) #because of float32 problems
                angle = acos(val)
                if angle >= edgeAngle:
                    edges+=[ind1, ind2]
                    pointHasEdge[ind1] = True
                    pointHasEdge[ind2] = True
    
    
    #smooth normals:
    #we simply do not smooth at points that have edges
    if smoothNormals:
        pointNormals = np.zeros((nPoints,3))
        for i in range(nPoints):
            if not pointHasEdge[i]:
                normal = np.zeros(3)
                for t in points2trigs[i]:
                    normal += areas[t]*normals[t]
                
                pointNormals[i] = ebu.Normalize(normal)

        
        finalTrigs = []
        newPoints = list(newPoints)
        newColors = list(newColors)
        pointNormals = list(pointNormals)
        for cnt, trig in enumerate(newTrigs):
            trigNew = [0,0,0]
            for i in range(3):
                if not pointHasEdge[trig[i]]:
                    trigNew[i] = trig[i]
                else:
                    trigNew[i] = len(newPoints)
                    newPoints.append(newPoints[trig[i]])
                    pointNormals.append(normals[cnt])
                    newColors.append(newColors[trig[i]])
            finalTrigs += [trigNew]
    else:
        finalTrigs = newTrigs
    
    graphicsData2 = FromPointsAndTrigs(newPoints, finalTrigs, list(np.array(newColors).flatten()))
    if addEdges:
        graphicsData2['edges'] = np.array(edges)
        graphicsData2['edgeColor'] = np.array(edgeColor)

    if smoothNormals:
        graphicsData2['normals'] = np.array(pointNormals).flatten()
    
    return graphicsData2

def ExportSTL(graphicsData, fileName, solidName='ExudynSolid', invertNormals=True, invertTriangles=True):
    """export given graphics data (only type TriangleList allowed!) to STL ascii file using fileName

    Args:
        graphicsData: a single GraphicsData dictionary with type='TriangleList', no list of GraphicsData
        fileName: file name including (local) path to export STL file
        solidName: optional name used in STL file
        invertNormals: if True, orientation of normals (usually pointing inwards in STL mesh) are inverted for compatibility in Exudyn
        invertTriangles: if True, triangle orientation (based on local indices) is inverted for compatibility in Exudyn
    """
    graphicsData = _Flat(Triangles6ToTriangles(SpheresToTriangleList(graphicsData)))
    if graphicsData['type'] != 'TriangleList':
        raise ValueError('ExportSTL: invalid graphics data type; only TriangleList and Spheres allowed')
        
    with open(fileName, 'w') as f:
        f.write('solid '+solidName+'\n')

        nTrig = int(len(graphicsData['triangles'])/3)
        triangles = graphicsData['triangles']
    
        for k in range(nTrig):
            p = [] #triangle points
            for i in range(3):
                ind = triangles[k*3+i]
                p += [np.array(graphicsData['points'][ind*3:ind*3+3])]
   
            n = gdu.ComputeTriangleNormal(p[0], p[1], p[2])
            if invertNormals:
                n = -n #normals inverted
            
            f.write('facet normal '+str(n[0]) + ' ' + str(n[1]) + ' ' + str(n[2]) + '\n') 
            f.write('outer loop\n')
            f.write('vertex '+str(p[0][0]) + ' ' + str(p[0][1]) + ' ' + str(p[0][2]) + '\n')
            if invertTriangles:
                f.write('vertex '+str(p[2][0]) + ' ' + str(p[2][1]) + ' ' + str(p[2][2]) + '\n') #point index reversed!
                f.write('vertex '+str(p[1][0]) + ' ' + str(p[1][1]) + ' ' + str(p[1][2]) + '\n')
            else:
                f.write('vertex '+str(p[1][0]) + ' ' + str(p[1][1]) + ' ' + str(p[1][2]) + '\n')
                f.write('vertex '+str(p[2][0]) + ' ' + str(p[2][1]) + ' ' + str(p[2][2]) + '\n') 

            f.write('endloop\n')
            f.write('endfacet\n')

        f.write('endsolid '+solidName+'\n')
