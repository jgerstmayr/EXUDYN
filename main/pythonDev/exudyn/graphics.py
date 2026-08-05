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

#constants and fixed structures:
import numpy as np #LoadSolutionFile
import copy as copy #to be able to copy e.g. lists
from math import radians, pi, sin, cos, tan, asin #, acos

graphicsDataNormalsFactor = 1. #this is a factor being either -1. [original normals pointing inside; until 2022-06-27], while +1. gives corrected normals pointing outside
graphicsDataSwitchTriangleOrder = False #this is the old ordering of triangles in some Sphere or Cylinder functions, causing computed normals to point inside

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#colors ...

#**class: A structure with default values representing RGBA-colors (list of 4 values ranging from 0 to 1); users will access colors via graphics.color, e.g., graphics.color.red
class color:
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


#**class: A structure that defines material indices and RGBA-values for standard materials; the material index (like indexChrome) can be used for the alpha-channel of a color to represent the material index; used only in the raytracer!
class material:
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


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: generate graphics data for a sphere with point p and radius
#**input:
#  point: center of sphere (3D list or np.array)
#  radius: positive value
#  color: provided as list of 4 RGBA values
#  nTiles: used to determine resolution of sphere >=2; represents resolution of a half-circle; use larger values for finer resolution
#  addEdges: True or number of edges along sphere shell (under development); for optimal drawing, nTiles shall be multiple of 4 or 8
#  edgeColor: optional color for edges
#  addFaces: if False, no faces are added (only edges); ignored in case of hollow sphere
#  majorAngleMin: starting angle for sphere to be drawn; if > -0.5*pi, it will be shortened at -Z coordinate
#  majorAngleMax: final angle for sphere to be drawn; if < 0.5*pi, it will be shortened at +Z coordinate
#  innerRadius: draw hollow sphere in case of majorAngleMin or majorAngleMax do not have default values
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
def Sphere(point=[0,0,0], radius=0.1, color=[0.,0.,0.,1.], nTiles = 8, 
           addEdges = False, edgeColor=color.black, addFaces=True,
           majorAngleMin = -0.5*pi, majorAngleMax = 0.5*pi, innerRadius = None):
    if nTiles < 2: 
        exudyn.Print("WARNING: graphics.Sphere: nTiles < 2: setting nTiles=2")
        nTiles = 2
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

    nOffOuter = len(points)//3
    innerSphereClosedMin = False
    innerSphereClosedMax = False
    if innerRadius is not None:
        if innerRadius <= 0 or innerRadius >= radius:
            raise ValueError("graphics.Sphere: innerRadius is invalid")

        r0 = r
        angleMin = majorAngleMin
        angleMax = majorAngleMax
        if r*sin(majorAngleMax) >= innerRadius:
            angleMax = 0.5*pi
            innerSphereClosedMax = True
        else:
            angleMax = asin(r*sin(majorAngleMax)/innerRadius)

        if r*sin(-majorAngleMin) >= innerRadius:
            angleMin =-0.5*pi
            innerSphereClosedMin = True
        else:
            angleMin = -asin(r*sin(-majorAngleMin)/innerRadius)

        for i0 in range(nTiles+1):
            z = innerRadius*sin(angleMax - i0/nTiles*(angleMax-angleMin))    #runs from -r .. r (this is the coordinate of the axis of circles)

            for iphi in range(nTilesPhi):
                phi = 2*pi*iphi/nTilesPhi #angle
                fact = sin(0.5*pi + angleMax - i0/nTiles*(angleMax-angleMin))

                x = fact*innerRadius*sin(phi)
                y = fact*innerRadius*cos(phi)

                vv = x*e0 + y*e1 + z*e2
                points += list(p + vv)
            
                n = ebu.Normalize(-vv) 
                normals += n
            
                colors += color
        
        nOffCircles = len(points)//3
        #draw plane circular rings:
        for iCircle in range(4):
            d = (iCircle < 2)
            angle = majorAngleMin if d else majorAngleMax
            nFact = -1 if d else 1
            
            z = r*sin(angle)
            angle2 = angle
            if r*sin(angle) < innerRadius and iCircle%2 == 1-int(d):
                angle2 = asin(r*sin(angle)/innerRadius)

            r0 = innerRadius if iCircle%2 == 1-int(d) else r
            for iphi in range(nTilesPhi):
                phi = 2*pi*iphi/nTilesPhi #angle
                fact = sin(0.5*pi + angle2)

                x = fact*r0*sin(phi)
                y = fact*r0*cos(phi)

                vv = x*e0 + y*e1 + z*e2
                points += list(p + vv)
            
                n = ebu.Normalize(nFact*e2)
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

        if innerRadius is not None:
            for i0 in range(nTiles):
                for iphi in range(nTilesPhi):
                    p0 = i0*nTilesPhi+iphi
                    p1 = (i0+1)*nTilesPhi+iphi
                    iphi1 = iphi + 1
                    if iphi1 >= nTilesPhi: 
                        iphi1 = 0
                    p2 = i0*nTilesPhi+iphi1
                    p3 = (i0+1)*nTilesPhi+iphi1
    
                    if graphicsDataSwitchTriangleOrder:
                        triangles += [nOffOuter+p0,nOffOuter+p3,nOffOuter+p1, nOffOuter+p0,nOffOuter+p2,nOffOuter+p3]
                    else:
                        triangles += [nOffOuter+p0,nOffOuter+p1,nOffOuter+p3, nOffOuter+p0,nOffOuter+p3,nOffOuter+p2]

            #draw plane circular rings:
            for iCircle in range(2):
                i0 = iCircle*2 #offset for second circle
                if iCircle == 0 and innerSphereClosedMin: continue
                if iCircle == 1 and innerSphereClosedMax: continue

                for iphi in range(nTilesPhi):
                    p0 = i0*nTilesPhi+iphi
                    p1 = (i0+1)*nTilesPhi+iphi
                    iphi1 = iphi + 1
                    if iphi1 >= nTilesPhi: 
                        iphi1 = 0
                    p2 = i0*nTilesPhi+iphi1
                    p3 = (i0+1)*nTilesPhi+iphi1
    
                    if graphicsDataSwitchTriangleOrder:
                        triangles += [nOffCircles+p0,nOffCircles+p3,nOffCircles+p1, nOffCircles+p0,nOffCircles+p2,nOffCircles+p3]
                    else:
                        triangles += [nOffCircles+p0,nOffCircles+p1,nOffCircles+p3, nOffCircles+p0,nOffCircles+p3,nOffCircles+p2]
            
    data = {'type':'TriangleList', 'colors':np.array(colors), 
            'points':np.array(points), 
            'normals':np.array(normals), 
            'triangles':np.array(triangles)}
    
    if type(addEdges) == bool and addEdges == True:
        addEdges = 3

    if addEdges > 0 and abs(majorAngleMax-majorAngleMin-pi) <= 1e-7 and innerRadius is None:
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
#**function: generate graphics data for lines, given by list of points and color; transforms to GraphicsData dictionary
#**input: 
#  pList: list of 3D numpy arrays or lists (to achieve closed curve, set last point equal to first point)
#  color: provided as list of 4 RGBA values
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
#**example:
##create simple 3-point lines
#gLine=graphics.Lines([[0,0,0],[1,0,0],[2,0.5,0]], color=color.red)
def Lines(pList, color=[0.,0.,0.,1.]): 
    data = np.zeros(len(pList)*3)
    for i, p in enumerate(pList):
        data[i*3:i*3+3] = p
    dataRect = {'type':'Line', 'color': np.array(color), 'data':data}

    return dataRect


#************************************************
#**function: generate graphics data for a single circle; currently the plane normal = [0,0,1], just allowing to draw planar circles -- this may be extended in future!
#**input: 
#  point: center point of circle
#  radius: radius of circle
#  color: provided as list of 4 RGBA values
#**notes: the tiling (number of segments to draw circle) can be adjusted by visualizationSettings.general.circleTiling
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
def Circle(point=[0,0,0], radius=1, color=[0.,0.,0.,1.]): 
    return {'type':'Circle', 'color': np.array(color), 'radius': radius, 'position':np.array(point)}


#************************************************
#**function: generate graphics data for a text drawn at a 3D position
#**input: 
#  point: position of text
#  text: string representing text; multiline texts can be written with line breaks
#  color: provided as list of 4 RGBA values
#  fontSize: scalar fontSize or 0. for default; default font size in Exudyn is 12 (visualizationSettings.view0.window.globalFontSize)
#  offset: offset in X/Y screen plane provided as list of 2 float values; this offset is not rotated with the model view and given relative to font size (offset [1,1] equals to offset of one character moved right and up)
#**nodes: text size can be adjusted with visualizationSettings.view0.window.globalFontSize, which affects the text size (=font size) globally
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
def Text(point=[0,0,0], text='', color=[0.,0.,0.,1.], fontSize=0., offset=[0.,0.]):
    return {'type':'Text', 
            'color': np.array(color), 
            'text':text, 
            'position':np.array(point),
            'fontSize':fontSize,
            'offset':offset}


#**function: generate graphics data for general block with endpoints, according to given vertex definition
#**input: 
#  pList: is a list of points [[x0,y0,z0],[x1,y1,z1],...]
#  color: provided as list of 4 RGBA values
#  faces: includes the list of six binary values (0/1), denoting active faces (value=1); set index to zero to hide face
#  addNormals: if True, normals are added and there are separate points for every triangle
#  addEdges: if True, edges are added in TriangleList of GraphicsData 
#  edgeColor: optional color for edges
#  addFaces: if False, no faces are added (only edges)
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
def Cuboid(pList, color=[0.,0.,0.,1.], faces=[1,1,1,1,1,1], addNormals=False, addEdges=False, edgeColor=color.black, addFaces=True): 
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


#**function: generate graphics data for orthogonal 3D block with min and max dimensions
#**input: 
#  x/y/z/Min/Max: minimal and maximal cartesian coordinates for orthogonal cube
#  color: list of 4 RGBA values
#  addNormals: add face normals to triangle information
#  addEdges: if True, edges are added in TriangleList of GraphicsData 
#  edgeColor: optional color for edges
#  addFaces: if False, no faces are added (only edges)
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
#**notes: DEPRECATED
def BrickXYZ(xMin, yMin, zMin, xMax, yMax, zMax, color=[0.,0.,0.,1.], addNormals=False, addEdges=False, edgeColor=color.black, addFaces=True): 
    pList = [[xMin,yMin,zMin], [xMax,yMin,zMin], [xMax,yMax,zMin], [xMin,yMax,zMin],
             [xMin,yMin,zMax], [xMax,yMin,zMax], [xMax,yMax,zMax], [xMin,yMax,zMax]]
    return Cuboid(pList, color, addNormals=addNormals, addEdges=addEdges, 
                  edgeColor=edgeColor, addFaces=addFaces)


#**function: generate graphics data for orthogonal 3D box with center point and size; using roundness=1, it draws an ellipsoid inside the box and in case 0 < roundness < 1, it draws a body blended between box and ellipsoid
#**input: 
#  centerPoint: center of box as 3D list or np.array
#  size: size as 3D list or np.array
#  color: list of 4 RGBA values
#  addNormals: add face normals to triangle information
#  addEdges: if True, edges are added in TriangleList of GraphicsData 
#  edgeColor: optional color for edges
#  addFaces: if False, no faces are added (only edges)
#  roundness: if > 0, it draws an ellipsoid, using nTiles for drawing; edges are not available if roundness > 0
#  nTiles: only apply if roundness > 0; discretization of whole ellipsoid; should be multiple of 4 to avoid artifacts
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects; if addEdges=True, it returns a list of two dictionaries
def Brick(centerPoint=[0,0,0], size=[0.1,0.1,0.1], color=[0.,0.,0.,1.], addNormals=False, addEdges=False, 
          edgeColor=color.black, addFaces=True, roundness=0, nTiles=12): 
    if roundness == 0:
        xMin = centerPoint[0] - 0.5*size[0]
        yMin = centerPoint[1] - 0.5*size[1]
        zMin = centerPoint[2] - 0.5*size[2]
        xMax = centerPoint[0] + 0.5*size[0]
        yMax = centerPoint[1] + 0.5*size[1]
        zMax = centerPoint[2] + 0.5*size[2]
    
        gBox = BrickXYZ(xMin, yMin, zMin, xMax, yMax, zMax, color, 
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


#**function: generate graphics data for a cylinder with given axis, radius and color; nTiles gives the number of tiles (minimum=3)
#**input:
#  pAxis: axis point of one face of cylinder (3D list or np.array)
#  vAxis: vector representing the cylinder's axis (3D list or np.array)
#  radius: positive value representing radius of cylinder
#  color: provided as list of 4 RGBA values
#  nTiles: used to determine resolution of cylinder >=3; use larger values for finer resolution
#  radiusInner: if not equal 0, this represents the inner radius of a hollow cylinder; some options like angleRange, lastFace, etc. do not work in this case
#  angleRange: given in rad, to draw only part of cylinder (halfcylinder, etc.); for full range use [0..2 * pi]
#  lastFace: if angleRange != [0,2*pi], then the faces of the open cylinder are shown with lastFace = True
#  cutPlain: only used for angleRange != [0,2*pi]; if True, a plane is cut through the part of the cylinder; if False, the cylinder becomes a cake shape ...
#  addEdges: if True, edges are added in TriangleList of GraphicsData; if addEdges is integer, additional int(addEdges) lines are added on the cylinder mantle
#  edgeColor: optional color for edges
#  addFaces: if False, no faces are added (only edges)
#  alternatingColor: if given, optionally another color in order to see rotation of solid; only works, if angleRange=[0,2*pi]
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
def Cylinder(pAxis=[0,0,0], vAxis=[0,0,1], radius=0.1, color=[0.,0.,0.,1.], nTiles = 16, 
             radiusInner = None, angleRange=[0,2*pi], lastFace = True, cutPlain = True, 
             addEdges=False, edgeColor=color.black, addFaces=True, **kwargs):  
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
        
    
    #create points at left and right face
    points0=list(pAxis) #[pAxis[0],pAxis[1],pAxis[2]] #avoid change of pAxis
    pAxis1=[pAxis[0]+vAxis[0],pAxis[1]+vAxis[1],pAxis[2]+vAxis[2]]
    points1=list(pAxis1) #[pAxis[0]+vAxis[0],pAxis[1]+vAxis[1],pAxis[2]+vAxis[2]] #copy in order to avoid change of pAxis1 for use lateron
    
    p0 = np.array(pAxis)
    p1 = np.array(pAxis) + np.array(vAxis)
    
    basis = ComputeOrthonormalBasisVectors(vAxis)
    #v0 = basis[0]
    n1 = basis[1]
    n2 = basis[2]
    r=radius
    
    nf = graphicsDataNormalsFactor #-1 original; -1 points inside

    #create normals at left and right face (pointing inwards)
    normals0 = ebu.Normalize([-vAxis[0],-vAxis[1],-vAxis[2]])
    normals1 = ebu.Normalize(vAxis)

    points2 = []
    points3 = []
    
    alpha = angleRange[1]-angleRange[0] #angular range
    alpha0 = angleRange[0]

    fact = nTiles #create correct part of cylinder
    if alpha < 2.*pi: 
        fact = nTiles-1

    # pointsCyl0 = []
    # pointsCyl1 = []
    
    for i in range(nTiles):
        phi = alpha0 + i*alpha/fact
        x = r*sin(phi)
        y = r*cos(phi)
        vv = x*n1 + y*n2
        pz0 = p0 + vv
        pz1 = p1 + vv
        points0 += list(pz0)
        points1 += list(pz1)
        points2 += list(pz0) #other points for side faces (different normals)
        points3 += list(pz1) #other points for side faces (different normals)
        # pointsCyl0 += list(pz0) #for edges
        # pointsCyl1 += list(pz1) #for edges
        n = ebu.Normalize(list(nf*vv))
        normals0 = normals0 + n
        normals1 = normals1 + n
        
    
    points0 += points1+points2+points3
    normals0 += normals1

    for i in range(nTiles):
        normals0 += ebu.Normalize([-nf*vAxis[0],-nf*vAxis[1],-nf*vAxis[2]])
    for i in range(nTiles):
        normals0 += ebu.Normalize([nf*vAxis[0],nf*vAxis[1],nf*vAxis[2]])

    n = nTiles+1 #number of points of one ring+midpoint
    color2 = list(color) #alternating color
    if 'alternatingColor' in kwargs:
        color2 = list(kwargs['alternatingColor'])

    colors=[]
    #for i in range(2*n+2*nTiles):
    #    colors += color
    n2 = int(nTiles/2)    
    for i in range(2):
        colors += list(color)
    for j in range(4):
        for i in range(n2):
            colors += list(color)
        for i in range(nTiles-n2):
            colors += color2

    triangles = []
    #circumference:
    for i in range(nTiles):
        if graphicsDataSwitchTriangleOrder:
            if i != nTiles-1:
                triangles += [1+i,n+1+i+1,n+1+i]
                triangles += [1+i,1+i+1,n+1+i+1]
            else:
                if lastFace and cutPlain:
                    triangles += [1+i,n+1,n+1+i]
                    triangles += [1+i,1,n+1]
        else:
            if i != nTiles-1:
                triangles += [1+i,n+1+i,n+1+i+1]
                triangles += [1+i,n+1+i+1,1+i+1]
            else:
                if lastFace and cutPlain:
                    triangles += [1+i,n+1+i,n+1]
                    triangles += [1+i,n+1,1]
            
    #sides faces left and right:
    nn=2*n #offset
    for i in range(nTiles):
        if graphicsDataSwitchTriangleOrder:
            if i != nTiles-1:
                triangles += [0,nn+i,nn+i+1]
                triangles += [n,nn+nTiles+i+1,nn+nTiles+i]
            else:
                if cutPlain:
                    triangles += [0,nn+i,nn]
                    triangles += [n,nn+nTiles,nn+nTiles+i]
        else:
            if i != nTiles-1:
                triangles += [0,nn+i,nn+i+1]
                triangles += [n,nn+nTiles+i+1,nn+nTiles+i]
            else:
                if cutPlain:
                    triangles += [0,nn+i,nn]
                    triangles += [n,nn+nTiles,nn+nTiles+i]

    #if angles are not 2*pi, add closing face
    if lastFace and not(cutPlain):
        s = int(len(points0)/3) #starting index for side triangles
        p2 = points2[0:3]
        p3 = points3[0:3]
        p4 = points2[len(points2)-3:len(points2)]
        p5 = points3[len(points3)-3:len(points3)]
        points0 += list(pAxis) + pAxis1 + p2 + p3 + list(pAxis) + pAxis1 + p4 + p5
        n1=np.cross(ebu.VSub(pAxis,pAxis1),ebu.VSub(p3,pAxis))
        n1=list(ebu.Normalize(-nf*n1))
        n2=np.cross(ebu.VSub(pAxis1,pAxis),ebu.VSub(p4,pAxis))
        n2=list(ebu.Normalize(-nf*n2))
        normals0 += n1+n1+n1+n1+n2+n2+n2+n2  #8 additional normals
        if graphicsDataSwitchTriangleOrder:
            triangles += [s+0,s+3,s+1, s+0,s+2,s+3, 
                          s+5,s+6,s+4, s+5,s+7,s+6]
        else:
            triangles += [s+0,s+1,s+3, s+0,s+3,s+2, 
                          s+5,s+4,s+6, s+5,s+6,s+7]
            
        for i in range(8): #8 additional colors
            colors += color

    if not addFaces:
        triangles = []

    #triangle normals point inwards to object ...
    data = {'type':'TriangleList', 'colors':np.array(colors), 
            'points':np.array(points0), 'normals':np.array(normals0), 'triangles':np.array(triangles)}

    if addEdges:
        data['edgeColor'] = np.array(edgeColor)
        
        faceEdges = 0
        if type(addEdges) != bool:
            faceEdges = int(addEdges)
        
        edges = []
        pLast = nTiles
        for i in range(nTiles):
            edges += [pLast, i+1]
            pLast = i+1
        
        pLast = nTiles + (nTiles+1)
        for i in range(nTiles):
            edges += [pLast, i+1+(nTiles+1)]
            pLast = i+1+(nTiles+1)
        
        if faceEdges > 0:
            nStep = int(nTiles/faceEdges)
            pLast0 = 1
            pLast1 = 1+(nTiles+1)
            for i in range(faceEdges):
                edges += [pLast0, pLast1]
                pLast0 += nStep
                pLast1 += nStep
        
        data['edges'] = np.array(edges)

    return data

#**function: generate graphics data for a tube with given list of points and axes, radius and color; nTiles gives the number of tiles (minimum=3)
#**input:
#  points: list of 3D vectors (or numpy arrays) representing the center points of the tube line
#  axes: list of 3D vectors (or numpy arrays) representing the axis according to the points
#  radius: positive value representing radius of tube
#  color: provided as list of 4 RGBA values
#  nTiles: used to determine resolution of cylinder >=3; use larger values for finer resolution
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
def Tube(points, axes, radius=0.1, color=[0.,0.,0.,1.], nTiles = 16):  
    if nTiles < 3: 
        exudyn.Print("WARNING: graphics.Tube: nTiles < 3: set nTiles=3")
        nTiles = 3

    if len(points) < 2:
        raise ValueError("graphics.Tube: must have at least 2 points and 2 axes")

    if len(points) != len(axes):
        raise ValueError("graphics.Tube: points and axes lists must be the same length")

    nSegments = len(points)

    vertices = []
    normals = []
    triangles = []


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


    # Generate circle vertices for each point
    for i in range(nSegments):
        p = points[i]
        [x, y, z] = frames[i]
        for j in range(nTiles):
            angle = 2 * np.pi * j / nTiles
            normal = np.cos(angle) * x + np.sin(angle) * y
            vertex = p + radius * normal
            vertices.append(vertex)
            normals.append(normal)

    # Generate triangle indices
    for i in range(nSegments - 1):
        for j in range(nTiles):
            i0 = i * nTiles + j
            i1 = i * nTiles + (j + 1) % nTiles
            i2 = (i + 1) * nTiles + j
            i3 = (i + 1) * nTiles + (j + 1) % nTiles

            # two triangles per quad
            triangles.append([i0, i2, i1])
            triangles.append([i1, i2, i3])

    colors = color*len(vertices)

    data = {'type':'TriangleList', 
            'colors':np.array(colors).flatten(), 
            'points':np.array(vertices).flatten(), 
            'normals':np.array(normals).flatten(), 
            'triangles':np.array(triangles).flatten()}

    return data


#**function: generate graphics data for a torus with given major and minor radius, center point and axis
#**input:
#  point: 3D vector (or numpy array) representing the center point of the torus
#  axis: 3D vector (or numpy array) representing the axis of revolution of the torus
#  radiusMajor: major radius of torus
#  radiusMinor: minor radius of torus
#  color: provided as list of 4 RGBA values
#  nTilesMajor: used to for resolution of tube with major radius; use larger values for finer resolution
#  nTilesMinor: used to for resolution of circle with minor radius; use larger values for finer resolution
#  minorAngleStart: starting angle for minor radius; 0 is the angle at outmost radius of torus, pi is at inside
#  minorAngleEnd: end angle for minor radius; use -0.5*pi / 0.5*pi to draw only the outer half of the torus
#  smoothNormals: if True, the normals are added to create a smooth contour, otherwise triangles are flat
#  invert: if False, the outside faces are visible; if invert=True, the inside faces are visible (influences reflections, light, etc.)
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
def Torus(point, axis, radiusMajor=0.5, radiusMinor=0.1, color=[0., 0., 0., 1.], 
          nTilesMajor=24, nTilesMinor=12, minorAngleStart=0, minorAngleEnd=2*np.pi, 
          smoothNormals=True, invert=False):
    if nTilesMajor < 3: 
        exudyn.Print("WARNING: graphics.Torus: nTilesMajor < 3: setting nTilesMajor=3")
        nTilesMajor = 3
    if nTilesMinor < 3: 
        exudyn.Print("WARNING: graphics.Torus: nTilesMinor < 3: setting nTilesMinor=3")
        nTilesMinor = 3

    vertices = []
    normals = []
    triangles = []

    #create orthonormal basis for torus
    [ex,ey,ez] = ComputeOrthonormalBasisVectors(axis) #ex=axis
    A = np.vstack([ey,ez,ex]).T
    
    isOpen=False #open circle
    isOpen = (minorAngleEnd-minorAngleStart) < 2*np.pi-1e-10
    
    nTilesMinor1 = nTilesMinor+isOpen
    invertSign = (1.-2.*int(invert) )

    if minorAngleStart >= minorAngleEnd:
        raise ValueError('Torus: ensure that minorAngleStart < minorAngleEnd !')

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

    data = {'type':'TriangleList', 
            'colors':np.array(colors).flatten(), 
            #'normals':np.array(normals).flatten(), #just don't add in case it shall not be smooth
            'points':np.array(vertices).flatten(), 
            'triangles':np.array(triangles).flatten()}
    if smoothNormals: 
        data['normals'] = np.array(normals).flatten()
    
    return data



#**function: generate graphics data for a planar Link between the two joint positions, having two axes
#**input:
#  p0: joint0 center position
#  p1: joint1 center position
#  axis0: direction of rotation axis at p0, if drawn as a cylinder; [0,0,0] otherwise
#  axis1: direction of rotation axis of p1, if drawn as a cylinder; [0,0,0] otherwise
#  radius: list of two radii [radius0, radius1], being the two radii of the joints drawn by a cylinder or sphere    
#  width: list of two widths [width0, width1], being the two widths of the joints drawn by a cylinder; ignored for sphere    
#  thickness: the thickness of the link (shaft) between the two joint positions; thickness in z-direction or diameter (cylinder)
#  color: provided as list of 4 RGBA values
#  nTiles: used to determine resolution of cylinder >=3; use larger values for finer resolution
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
def RigidLink(p0,p1,axis0=[0,0,0], axis1=[0,0,0], radius=[0.1,0.1], 
                          thickness=0.05, width=[0.05,0.05], color=[0.,0.,0.,1.], nTiles = 16):
    linkAxis = ebu.VSub(p1,p0)
    #linkAxis0 = ebu.Normalize(linkAxis)
    a0=list(axis0)
    a1=list(axis1)
    
    data0 = Cylinder(p0, linkAxis, 0.5*thickness, color, nTiles)
    data1 = {}
    data2 = {}

    if ebu.NormL2(axis0) == 0:
        data1 = Sphere(p0, radius[0], color, nTiles)
    else:
        a0=ebu.Normalize(a0)
        data1 = Cylinder(list(np.array(p0)-0.5*width[0]*np.array(a0)), 
                                     list(width[0]*np.array(a0)), 
                                     radius[0], color, nTiles)
        
    if ebu.NormL2(axis1) == 0:
        data2 = Sphere(p1, radius[1], color, nTiles)
    else:
        a1=ebu.Normalize(a1)
        data2 = Cylinder(list(np.array(p1)-0.5*width[1]*np.array(a1)), 
                                     list(width[1]*np.array(a1)), radius[1], color, nTiles)

    #now merge lists, including appropriate indices of triangle points!
    np0 = int(len(data0['points'])/3) #number of points of first point list ==> this is the offset for next list
    np1 = np0 + int(len(data1['points'])/3) #number of points of first point list ==> this is the offset for next list

    triangles = data0['triangles']
    trigs1 = np.array(data1['triangles'])
    trigs1 += np0
    triangles = np.append(triangles,trigs1)
    
    trigs2 = np.array(data2['triangles'])
    trigs2 += np1
    triangles = np.append(triangles,trigs2)
    
    points = np.concatenate((data0['points'], data1['points'], data2['points']))
    normals = np.concatenate((data0['normals'], data1['normals'], data2['normals']))
    colors = np.concatenate((data0['colors'], data1['colors'], data2['colors']))
    
    data = {'type':'TriangleList', 'colors':colors,
            'points':points, 'normals':normals, 'triangles':np.array(triangles)}
    return data


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#   unused argument yet: contourNormals: if provided as list of 2D vectors, they prescribe the normals to the contour for smooth visualization; otherwise, contour is drawn flat
#**function: generate graphics data for a solid of revolution with given 3D point and axis, 2D point list for contour, (optional)2D normals and color; 
#**input:
#  pAxis: axis point of one face of solid of revolution (3D list or np.array)
#  vAxis: vector representing the solid of revolution's axis (3D list or np.array)
#  contour: a list of 2D-points, specifying the contour (x=axis, y=radius), e.g.: [[0,0],[0,0.1],[1,0.1]]
#  color: provided as list of 4 RGBA values
#  nTiles: used to determine resolution of solid; use larger values for finer resolution
#  smoothContour: if True, the contour is made smooth by auto-computing normals to the contour
#  addEdges: True or number of edges along revolution mantle; for optimal drawing, nTiles shall be multiple addEdges
#  edgeColor: optional color for edges
#  addFaces: if False, no faces are added (only edges)
#  smoothingAngle: if angle between two edges is smaller than smoothingAngle, smoothing is applied
#  alternatingColor: add a second color, which enables to see the rotation of the solid
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
#**example:
##simple contour, using list of 2D points:
#contour=[[0,0.2],[0.3,0.2],[0.5,0.3],[0.7,0.4],[1,0.4],[1,0.]]
#rev1 = graphics.SolidOfRevolution(pAxis=[0,0.5,0], vAxis=[1,0,0], 
#                                     contour=contour, color=color.red,
#                                     alternatingColor=color.grey)
##draw torus:
#contour=[]
#r = 0.2 #small radius of torus
#R = 0.5 #big radius of torus
#nc = 16 #discretization of torus
#for i in range(nc+3): #+3 in order to remove boundary effects
#    contour+=[[r*cos(i/nc*pi*2),R+r*sin(i/nc*pi*2)]]
#
##use smoothContour to make torus looking smooth
#rev2 = graphics.SolidOfRevolution(pAxis=[0,0.5,0], vAxis=[1,0,0], 
#                                     contour=contour, color=color.red, 
#                                     nTiles = 64, smoothContour=True)
def SolidOfRevolution(pAxis, vAxis, contour, color=[0.,0.,0.,1.], nTiles = 16, smoothContour = False, 
                      addEdges = False, edgeColor=color.black, addFaces=True, smoothingAngle=2*np.pi, **kwargs):  
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

    points = []
    normals = []
    colors = []
    nT2 = int(nTiles/2)
    nf = graphicsDataNormalsFactor #factor for normals (inwards/outwards)

    for j in range(len(contour)-1):
        pc0 = np.array(contour[j])
        pc1 = np.array(contour[j+1])
        points0 = []
        points1 = []
        normals0 = []
        normals1 = []
        for i in range(nTiles):
            phi = i*2*pi/nTiles
            x0 = pc0[1]*sin(phi)
            y0 = pc0[1]*cos(phi)
            vv0 = x0*n1 + y0*n2

            x1 = pc1[1]*sin(phi)
            y1 = pc1[1]*cos(phi)
            vv1 = x1*n1 + y1*n2

            pz0 = p0 + vv0 + pc0[0]*v
            pz1 = p0 + vv1 + pc1[0]*v
            points0 += list(pz0)
            points1 += list(pz1)

            #vc = pc1-pc0
            #nc = [-vc[1],vc[0]]
            nc0 = contourNormals[j]
            nUnit0 = ebu.Normalize(nf*nc0[1]*sin(phi)*n1 + nf*nc0[1]*cos(phi)*n2+nf*nc0[0]*v)
            nUnit1 = nUnit0
            if smoothContour:
                #nc1 = contourNormals[j+1]
                nc1 = contourNormalsNext[j]
                nUnit1 = ebu.Normalize(nf*nc1[1]*sin(phi)*n1 + nf*nc1[1]*cos(phi)*n2+nf*nc1[0]*v)

            normals0 = normals0 + nUnit0
            normals1 = normals1 + nUnit1

        cList = list(color)*nT2 + list(color2)*(nTiles-nT2)
        colors += cList+cList
        points += points0 + points1
        normals += normals0 + normals1
    
    triangles = []
    n = nTiles
    #circumference:
    if addFaces:
        for j in range(len(contour)-1):
            k = j*2*n
            for i in range(nTiles):
                if i < nTiles-1:
                    triangles += [i+k,n+i+k,n+i+k+1]
                    triangles += [i+k,n+i+k+1,i+1+k]
                else:
                    triangles += [i+k,n+i+k,n+k]
                    triangles += [i+k,n+k,k]

    #triangle normals point inwards to object ...
    data = {'type':'TriangleList', 'colors':np.array(colors),
            'points':np.array(points), 'normals':np.array(normals), 'triangles':np.array(triangles)}


    if addEdges > 0:
        data['edgeColor'] = np.array(edgeColor)
        edges = []

        cntEdges = 0        
        nSteps = nTiles
        if type(addEdges) != bool and addEdges > 0:
            cntEdges = int(addEdges)
            nSteps = int(nTiles/cntEdges)
        
        hEdges = []
        for j in range(cntEdges):
            hEdges += [[]]

        for j in range(len(contour)-1):
            k = j*2*n
            for i in range(nTiles):
                edges += [i+k, (i+1)%nTiles+k]
                if i%nSteps==0:
                    j=int(i/nSteps)
                    if j < cntEdges:
                        hEdges[j] += [i+k, i+k+n]

        for j in range(cntEdges):
            edges += hEdges[j]

        data['edges'] = np.array(edges)

    return data


#**function: generate graphics data for an arrow with given origin, axis, shaft radius, optional size factors for head and color; nTiles gives the number of tiles (minimum=3)
#**input:
#  pAxis: axis point of the origin (base) of the arrow (3D list or np.array)
#  vAxis: vector representing the vector pointing from the origin to the tip (head) of the error (3D list or np.array)
#  radius: positive value representing radius of shaft cylinder
#  headFactor: positive value representing the ratio between head's radius and the shaft radius
#  headStretch: positive value representing the ratio between the head's radius and the head's length
#  color: provided as list of 4 RGBA values
#  nTiles: used to determine resolution of arrow (of revolution object) >=3; use larger values for finer resolution
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
def Arrow(pAxis, vAxis, radius, color=[0.,0.,0.,1.], headFactor = 2, headStretch = 4, nTiles = 12):  
    L = ebu.NormL2(vAxis)
    rHead = radius * headFactor
    xHead = L - headStretch*rHead
    contour=[[0,0],[0,radius],[xHead,radius],[xHead,rHead],[L,0]]
    return SolidOfRevolution(pAxis=pAxis, vAxis=vAxis, contour=contour, color=color, nTiles=nTiles)

#**function: generate graphics data for three arrows representing an orthogonal basis with point of origin, shaft radius, optional size factors for head and colors; nTiles gives the number of tiles (minimum=3)
#**input:
#  origin: point of the origin of the base (3D list or np.array)
#  rotationMatrix: optional transformation, which rotates the basis vectors
#  length: positive value representing lengths of arrows for basis
#  colors: provided as list of 3 colors (list of 4 RGBA values)
#  headFactor: positive value representing the ratio between head's radius and the shaft radius
#  headStretch: positive value representing the ratio between the head's radius and the head's length
#  nTiles: used to determine resolution of arrows of basis (of revolution object) >=3; use larger values for finer resolution
#  radius: positive value representing radius of arrows; default: radius = 0.01*length
#  labels: a list of 3 strings written to the three axes (X, Y, Z); in this case, the result is returned as list of GraphicsData!
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
def Basis(origin=[0,0,0], rotationMatrix = np.eye(3), length = 1, colors=[color.red, color.green, color.blue], 
                      headFactor = 2, headStretch = 4, nTiles = 12, **kwargs):  
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

#**function: generate graphics data for frame (similar to Basis), showing three arrows representing an orthogonal basis for the homogeneous transformation HT; optional shaft radius, optional size factors for head and colors; nTiles gives the number of tiles (minimum=3)
#**input:
#  HT: homogeneous transformation representing frame
#  length: positive value representing lengths of arrows for basis
#  colors: provided as list of 3 colors (list of 4 RGBA values)
#  headFactor: positive value representing the ratio between head's radius and the shaft radius
#  headStretch: positive value representing the ratio between the head's radius and the head's length
#  nTiles: used to determine resolution of arrows of basis (of revolution object) >=3; use larger values for finer resolution
#  radius: positive value representing radius of arrows; default: radius = 0.01*length
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
def Frame(HT=np.eye(4), length = 1, colors=[color.red, color.green, color.blue], 
                      headFactor = 2, headStretch = 4, nTiles = 12, **kwargs):  
    radius = 0.01*length
    if 'radius' in kwargs:
        radius = kwargs['radius']

    
    A = HT2rotationMatrix(HT)
    origin = HT2translation(HT)
    
    g1 = Arrow(origin,A@[length,0,0],radius, colors[0], headFactor, headStretch, nTiles)
    g2 = Arrow(origin,A@[0,length,0],radius, colors[1], headFactor, headStretch, nTiles)
    g3 = Arrow(origin,A@[0,0,length],radius, colors[2], headFactor, headStretch, nTiles)

    return MergeTriangleLists(MergeTriangleLists(g1,g2),g3)


#**function: generate graphics data for simple quad with option for checkerboard pattern;
#  points are arranged counter-clock-wise, e.g.: p0=[0,0,0], p1=[1,0,0], p2=[1,1,0], p3=[0,1,0]
#**input: 
#  pList: list of 4 quad points [[x0,y0,z0],[x1,y1,z1],...]
#  color: provided as list of 4 RGBA values
#  alternatingColor: second color; if defined, a checkerboard pattern (default: 10x10) is drawn with color and alternatingColor
#  nTiles: number of tiles for checkerboard pattern (default: 10)
#  nTilesY: if defined, use number of tiles in y-direction different from x-direction (=nTiles)
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
#**example:
#plane = graphics.Quad([[-8, 0, -8],[ 8, 0, -8,],[ 8, 0, 8],[-8, 0, 8]], 
#                         color.darkgrey, nTiles=8, 
#                         alternatingColor=color.lightgrey)
#oGround=mbs.AddObject(ObjectGround(referencePosition=[0,0,0],
#                      visualization=VObjectGround(graphicsData=[plane])))
def Quad(pList, color=[0.,0.,0.,1.], **kwargs): 
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


#**function: function to generate checkerboard background;
#  points are arranged counter-clock-wise, e.g.: 
#**input: 
#  point: midpoint of pattern provided as list or np.array
#  normal: normal to plane provided as list or np.array
#  size: dimension of first side length of quad
#  size2: dimension of second side length of quad
#  color: provided as list of 4 RGBA values
#  alternatingColor: second color; if defined, a checkerboard pattern (default: 10x10) is drawn with color and alternatingColor
#  nTiles: number of tiles for checkerboard pattern in first direction
#  nTiles2: number of tiles for checkerboard pattern in second direction; default: nTiles
#  materialIndex: use special graphics material for both colors
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
#**example:
#plane = graphics.CheckerBoard(normal=[0,0,1], size=5)
#oGround=mbs.AddObject(ObjectGround(referencePosition=[0,0,0],
#                      visualization=VObjectGround(graphicsData=[plane])))
def CheckerBoard(point=[0,0,0], normal=[0,0,1], size = 1,
                             color=color.lightgrey, alternatingColor=color.lightgrey2, nTiles=10, **kwargs):
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
#**function: create graphicsData for solid extrusion based on 2D points and segments; by default, the extrusion is performed in z-direction;
#            additional transformations are possible to translate and rotate the extruded body;
#**input:
#  vertices: list of pairs of coordinates of vertices in mesh [x,y], see ComputeTriangularMesh(...)
#  segments: list of segments, which are pairs of node numbers [i,j], defining the boundary of the mesh;
#            the ordering of the nodes is such that left triangle = inside, right triangle = outside; see ComputeTriangularMesh(...)
#  height:   height of extruded object
#  rot:      rotation matrix, which the whole extruded object point coordinates are multiplied with before adding offset
#  pOff:     3D offset vector added to all extruded coordinates (both planes); the z-coordinate of the extrusion object obtains 0 for the base plane, z=height for the top plane
#  relRot: rotation matrix for transformation of top (second) plane of extrusion object
#  relOff: 3D offset vector added top (second) plane of extrusion object; the z-coordinate is added to height, which is the base z-value
#  color: provided as list of 4 RGBA values
#  smoothNormals: if True, algorithm tries to smoothen normals at vertices and normals are added; creates more points; if False, triangle normals are used internally 
#  addEdges: if True or 1, edges at bottom/top are included in the GraphicsData dictionary; if 2, also mantle edges are included
#  edgeColor: optional color for edges
#  addFaces: if False, no faces are added (only edges)
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
#**example:
# #simple block with cutout
# g = graphics.SolidExtrusion(vertices=[[-0.4,-0.4], [0.4,-0.4], [ 0.4,0.4], [0.1,0.4],
#                                       [0.1,  0.2], [-0.1,0.2], [-0.1,0.4], [-0.4,0.4]],
#                            segments=[[0,1], [1,2], [2,3], [3,4], [4,5], [5,6], [6,7], [7,0]],
#                            pOff = [0,2,-1], height=1.5,
#                            color=graphics.color.steelblue, addEdges=2)        
#
# oGround=mbs.CreateGround(graphicsDataList=[g])
def SolidExtrusion(vertices, segments, height, 
                   rot = np.diag([1,1,1]), pOff = [0,0,0], 
                   relRot = np.diag([1,1,1]), relOff = [0,0,0], 
                   color = [0,0,0,1], smoothNormals = False, 
                   addEdges = False, edgeColor=color.black, addFaces=True):
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


#**function: generate graphics data for an extrusion solid linking two circles by their external tangents in a plane; the shape is extruded along axisCylinder with height equal to its norm; nTiles controls circle tessellation
#**input:
#  point0: center of the first circle and base point of the extrusion (3D list or np.array)
#  point1: a point whose projection into the plane through point0 with normal axisCylinder defines the direction to the second circle center (3D list or np.array)
#  axisCylinder: vector normal to the circle plane and extrusion direction; extrusion height is ||axisCylinder|| (3D list or np.array)
#  radius0: radius of the first circle (positive float)
#  radius1: radius of the second circle (positive float)
#  radiusInner0: if > 0, radius of bore of the first circle
#  radiusInner1: if > 0, radius of bore of the second circle
#  nTiles: tiling used for a full circle (>=3); partial arcs are sampled proportionally
#  color: provided as list of 4 RGBA values
#  addEdges: if True, edges are added in TriangleList of GraphicsData; if addEdges is integer, additional int(addEdges) lines are added on the extrusion
#  edgeColor: optional color for edges
#  addFaces: if False, no faces are added (only edges)
#  smoothNormals: if True, algorithm tries to smoothen normals at vertices and normals are added; creates more points; if False, triangle normals are used internally 
#  kwargs: forwarded to graphics.SolidExtrusion
#**output: graphicsData dictionary, to be used in visualization of Exudyn objects
#**example:
# g = graphics.LinkedCylinders(point0=[0,0,0], point1=[0.8,0.2,0.4], axisCylinder=[0,0,1.2],
#                               radius0=0.25, radius1=0.15, nTiles=48,
#                               color=graphics.color.steelblue, addEdges=2)
# oGround=mbs.CreateGround(graphicsDataList=[g])
def LinkedCylinders(point0, point1, axisCylinder, radius0, radius1,
                    radiusInner0=0, radiusInner1=0, nTiles=32, color=[0,0,0,1],
                    addEdges=0, edgeColor=color.black, addFaces=True, smoothNormals=True,
                    **kwargs):

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
#**function: generate graphics for ball bearing rings, in particular for inner and outer rings; note that base parameters are identical as in function GetBallBearingData, assuming that the dictionary of the latter function is used as input for BallBearingRings
#**input:
#  innerGrooveTorusRadius: major radius of torus for inner groove
#  outerGrooveTorusRadius: major radius of torus for outer groove
#  nTilesRings: circumferential tiling of rings
#  nTilesGrooves: tiling of grooves
#  colorCage: cage RGBA color
#  colorInnerRing: inner ring RGBA color
#  colorOuterRing: outer ring RGBA color
#**output: dictionary of graphics data containing 'innerRingGraphics', 'outerRingGraphics' and 'cageGraphics'; Note: graphics data is in the local bearing coordinate system, which should align with inner ring, outer ring and cage bodies!
#**example:
# import exudyn.graphics as graphics
# from machines import GetBallBearingData
# data = GetBallBearingData(axis=[0,0,1], outsideDiameter=0.080, 
#                           boreDiameter=0.050, width=0.010, nBalls=12)
# graphicsData = graphics.BallBearingRings(**data)
# #... graphicsData now contains graphics of rings
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


#**function: create graphics for involute gear, using data from machines.InvoluteGear
#**input:
#  involuteGear: an instance of the class machines.InvoluteGear, containing gear data
#  width: width of gear
#  centerPoint: used to shift the center point of the gear; if 0, the center is in the middle of the gear
#  rotationMatrix: the gear is constructed in the x-y plane, with the gear axis [0,0,1]; to get any other axis, provide the rotation matrix
#  helixAngleDeg: optional angle for helix gears in degree; note that this is only an approximation to real helical gear geometry!
#  radius: in case of internal gear, this is the outer radius; for regular gear, this is the bore radius
#  relativeAngleOffset: angular offset (about gear wheel axis) relative to the angle of one tooth and gap; 0.5 means that the tooth goes to the position of the gap
#  color: provided as list of 4 RGBA values
#  smoothNormals: if True, algorithm tries to smoothen normals at vertices and normals are added; creates more points; if False, triangle normals are used internally 
#  addEdges: if True or 1, edges at bottom/top are included in the GraphicsData dictionary; if 2, also mantle edges are included
#  edgeColor: optional color for edges
#  addFaces: if False, no faces are added (only edges)
#**output: single graphics data for gear
def InvoluteGear(involuteGear, width, 
                 centerPoint=np.zeros(3), rotationMatrix = np.eye(3), 
                 helixAngleDeg=0, radius=0, relativeAngleOffset=0, 
                 color=[0,0,0,1], nTilesCylinder=32, smoothNormals = False, addEdges = False, 
                 edgeColor=color.black, addFaces=True,
                 ):
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
    




#**function: create graphics for toothed rack
#**input:
#  module: the module in m; thus, m*pi represents the mid-distance of one tooth to the next one
#  width: width of gear
#  nTeeth: number of teeth used; this gives the length; if this is a float number, only part of the last root or tooth are drawn accordingly 
#  toothHeight: height of tooth from root to head
#  rackBaseHeight: height of rack below root
#  pressureAngleDeg: pressure angle in degree for tooth shape
#  centerPoint: used to shift the center point of the gear; if 0, the center is at the start point of the generated toothed rack (x=0,y=0), z=0 is in the middle of the rack
#  rotationMatrix: the gear is constructed in the x-y plane, with width along z-axis
#  color: provided as list of 4 RGBA values
#  smoothNormals: if True, algorithm tries to smoothen normals at vertices and normals are added; creates more points; if False, triangle normals are used internally 
#  addEdges: if True or 1, edges at bottom/top are included in the GraphicsData dictionary; if 2, also mantle edges are included
#  edgeColor: optional color for edges
#  addFaces: if False, no faces are added (only edges)
#**output: single graphics data for gear
def ToothedRack(module, nTeeth, width, toothHeight, rackBaseHeight,
                pressureAngleDeg=20,
                centerPoint=np.zeros(3), rotationMatrix = np.eye(3), 
                color=[0,0,0,1], nTilesCylinder=32, addEdges = False, 
                edgeColor=color.black, addFaces=True,
                ):
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


#**function: compute bounding box of single graphicsData
#**input: 
#  graphicsData: a single Exudyn GraphicsData object
#**output::list: [bmin, bmax]; tuple of np.array shape (3,), or (None, None) if no points.
def BoundingBoxSingle(graphicsData):

    gtype = graphicsData.get('type', None)
    if gtype is None:
        raise ValueError("BoundingBoxSingle Missing 'type' in graphicsData.")

    if gtype == 'TriangleList':
        pts = np.array(graphicsData.get('points', []), dtype=float)
        if pts.size == 0:
            return [None, None]
        pts = pts.reshape((-1, 3))
        return [pts.min(axis=0), pts.max(axis=0)]

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

#**function: compute bounding box of single GraphicsData or list of GraphicsData 
#**input: 
#  graphicsData: a single Exudyn GraphicsData object or list
#**output::list: [bmin, bmax]; tuple of np.array shape (3,), or (None, None) if no points.
def BoundingBox(graphicsData):
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
        raise ValueError(f"BoundingBox: graphicsData must be dict or list")


#**function: convert triangles and points as returned from graphics.ToPointsAndTrigs(...) to GraphicsData; additionally, normals and color(s) can be provided
#**input: 
#  points: list or np.array with np rows of 3 columns (floats) per point (with np points)
#  triangles: list or np.array with 3 int per triangle (0-based indices to triangles), giving a matrix with nt rows and 3 columns (with nt triangles)
#  color: provided as list of 4 RGBA values or single list of (np)*[4 RGBA values]
#  normals: if not None, they have to be provided per point (as matrix, list of lists or flattened) and will be added to returned GraphicsData
#**output: returns GraphicsData with type TriangleList
def FromPointsAndTrigs(points, triangles, color=[0.,0.,0.,1.], normals=None):
    pointList = np.array(points).flatten()
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
            'triangles':triangleList}
    if normals is not None: 
        data['normals'] = np.array(normals).flatten()
    return data



#************************************************
#**function: convert graphics data into list of points and list of triangle indices (triplets)
#**input: g contains a GraphicsData with type TriangleList
#**output: returns [points, triangles], with points as list of np.array with 3 floats per point and triangles as a list of np.array with 3 int per triangle (0-based indices to points)
def ToPointsAndTrigs(g):
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
#**function: transform a GraphicsData object in several ways: move, rotate, scale; furthermore, normals can be fixed and inverted, etc.
#**input:
#  g: graphicsData to be transformed
#  translation: 3D offset as list or numpy.array added to rotated points; if pOff=None, no translation is applied
#  rotation: 3D rotation matrix as list of lists or numpy.array with shape (3,3); if A is scaled by factor, e.g. using 0.001*np.eye(3), you can also scale the coordinates; if Aoff=None, no rotation is performed
#  scale: scaling of position coordinates
#  normalizeNormals: if True, normals are scaled such that length=1 (or zero for zero-normals)
#  invertTriangles: if True, it inverts the triangle orientation (changing vertex index 0 and 1)
#  invertNormals: if True, the direction of normal is flipped
#**output: returns new graphcsData object to be used for drawing in objects
#**notes: the rigid body transformation corresponds to HomogeneousTransformation(rotation, translation), transforming original coordinates v into vNew = translation + rotation @ v
def Transform(graphicsData, translation=None, rotation=None, scale=1,
              normalizeNormals=False, invertNormals=False, invertTriangles=False,
              warn=True):
    
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
        if invertTriangles:
            nTrigs=int(len(graphicsData['triangles'])/3)
            triangles = np.array(graphicsData['triangles']).reshape((nTrigs,3))
        
            if invertTriangles:
                for i, trig in enumerate(triangles):
                    t0 = trig[0]
                    trig[0]=trig[1]
                    trig[1] = t0
                gNew['triangles'] = triangles.flatten()
        else:
            gNew['triangles'] = np.array(graphicsData['triangles'])
            
        if 'edges' in graphicsData:
            gNew['edges'] = np.array(graphicsData['edges'])
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
#**function: add rigid body transformation and possible scaling to GraphicsData, using position offset (global) pOff (list or np.array) and rotation Aoff (transforms local to global coordinates; list of lists or np.array)
#**input:
#  g: graphicsData to be transformed
#  pOff: 3D offset as list or numpy.array added to rotated points
#  Aoff: 3D rotation matrix as list of lists or numpy.array with shape (3,3); if Aoff=None, no rotation is performed
#**output: returns new graphcsData object to be used for drawing in objects
#**notes: transformation corresponds to HomogeneousTransformation(Aoff, pOff), transforming original coordinates v into vNew = pOff + Aoff @ v
def Move(g, pOff, Aoff=None):
    return Transform(graphicsData=g, translation=pOff, rotation=Aoff)

#************************************************
#**function: merge 2 different graphics data with triangle lists
#**input: graphicsData dictionaries g1 and g2 obtained from GraphicsData functions
#**output: one graphicsData dictionary with single triangle lists and compatible points and normals, to be used in visualization of EXUDYN objects; edges are merged; edgeColor is taken from graphicsData g1
def MergeTriangleLists(g1,g2):
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
                'points': np.array(g1['points']), 'triangles': np.array(g1['triangles'])}

        data['normals'] = np.append(data['normals'],g2['normals'])
    else:
        data = {'type':'TriangleList', 'colors':np.array(g1['colors']),
                'points': np.array(g1['points']), 'triangles': np.array(g1['triangles'])}
    
    data['colors'] = np.append(data['colors'], g2['colors'])
    data['points'] = np.append(data['points'], g2['points'])

    # for p in g2['triangles']:
    #     data['triangles'] += [int(p + nPoints)] 
    data['triangles'] = np.append(data['triangles'], np.array(g2['triangles'])+nPoints ) #add nPoints offset to g2 for correct connectivity

    #copy and merge edges; edges can be available only in one triangle list
    if 'edges' in g1:
        data['edges'] = np.array(g1['edges'])
    if 'edges' in g2:
        edges2 = np.array(g2['edges'])
        if 'edges' not in data:
            data['edges'] = []
        else:
            edges2 += nPoints #add offset
        
        data['edges'] = np.append(data['edges'], edges2)
    if 'edgeColor' in g1:
        data['edgeColor'] = np.array(g1['edgeColor']) #only taken from g1, as there is only a single color
    elif 'edgeColor' in g2:
        data['edgeColor'] = np.array(g2['edgeColor']) #only taken from g2


    return data

#************************************************
#**function: invert triangle orientation and triangle normals (or only one of these tasks); can also check consistency of normals
#**input:
#  graphicsData: graphicsData as returned e.g. from graphics.Sphere
#  invertTriangles: if True, it inverts the triangle orientation (changing vertex index 0 and 1)
#  invertNormals: if True, the direction of normal is flipped
#**output: returns new graphicsData (copy) with modified triangles and normals
def InvertTriangles(graphicsData, invertTriangles=True, invertNormals=True):
    if graphicsData['type'] != 'TriangleList': 
        raise ValueError('InvertTriangles only works for graphicsData of TriangleList type')
    if 'normals' not in graphicsData and invertNormals:
        raise ValueError('InvertTriangles requires normals in TriangleList if invertNormals=True')

    gNew = {'type':'TriangleList'}
    gNew['points'] = np.array(graphicsData['points']) #copy
    gNew['colors'] = np.array(graphicsData['colors']) #copy
    gNew['triangles'] = np.array(graphicsData['triangles']) #copy

    nPoints=int(len(graphicsData['points'])/3)
    nTrigs=int(len(graphicsData['triangles'])/3)

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

#**function: check consistency of orientation of triangles and vertex (point) normals
#**input:
#  graphicsData: graphicsData as returned e.g. from graphics.Sphere
#**output: returns number of cases in which triangle normals and vertex normals are inconsistent (scalar product is negative)
def InconsistentTriangles(graphicsData):
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

#**function: convert NGsolve (surface) mesh into (surface) points and triangles; clearly, it requires to have ngsolve installed
#**input: 
#  mesh: a ngsolve mesh; having a geometry geo = OCCGeometry(...), mesh is returned from ngsolve.Mesh(geo.GenerateMesh(...))
#  ngMesh: a netgen mesh; having a geometry geo = OCCGeometry(...), ngMesh is returned from geo.GenerateMesh(...)
#  meshOrder: either 1 (linear, flat triangles) or 2 (quadratic, smooth triangles)
#  scale: additional scaling factor for geometry, as it is recommended to define netgen geometries in mm due to tolerances
#  addNormals: if True, it computes and adds normals
#  verbose: print debug information
#**output: [points, triangles] or if addNormals=True, [points, triangles, normals] for further usage in graphics.FromPointsAndTrigs(...)
#**example:
##assume having already a body of netgen OCCGeometry
#geo = OCCGeometry(body)
#ngMesh = geo.GenerateMesh(maxh=maxh)
##convert mesh into points, triangles and normals (with second-order elements!)
#[points, triangles, normals] = graphics.NGsolveMesh2PointsAndTrigs(mesh=ngMesh)
##convert into graphicsData
#gMesh = graphics.FromPointsAndTrigs( points, triangles, normals=normals,
#                                    color=graphics.color.red)
##use the mesh on a ground object
#mbs.CreateGround(graphicsDataList=[gMesh])
def NGsolveMesh2PointsAndTrigs(mesh=None, ngMesh=None, meshOrder=2, scale=1, addNormals=True, verbose=False):
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



#**function: generate graphics data from STL file (text format!) and use color for visualization; this function is slow, use stl binary files with FromSTLfile(...)
#**input:
#  fileName: string containing directory and filename of STL-file (in text / SCII format) to load
#  color: provided as list of 4 RGBA values
#  verbose: if True, useful information is provided during reading
#  invertNormals: if True, orientation of normals (usually pointing inwards in STL mesh) are inverted for compatibility in Exudyn
#  invertTriangles: if True, triangle orientation (based on local indices) is inverted for compatibility in Exudyn
#**output: creates graphicsData, inverting the STL graphics regarding normals and triangle orientations (interchanged 2nd and 3rd component of triangle index)
def FromSTLfileASCII(fileName, color=[0.,0.,0.,1.], verbose=False, invertNormals=True, invertTriangles=True): 
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
#**function: generate graphics data from any file that can be loaded with PyMeshLab (in particular .obj, .dae and .stl); either use defaultColor or given color in mesh.
#**input:
#  fileName: string containing directory and filename of geometry file
#  defaultColor: provided as list of 4 RGBA values; used only if meshlab cannot load valid color or if file does not include color (e.g., STL)
#  verbose: if True, some information is logged during file import
#  invertNormals: if True, orientation of normals (usually pointing inwards in STL mesh) are inverted for compatibility in Exudyn
#  invertTriangles: if True: triangle orientation (based on local indices) is inverted for compatibility in Exudyn
#  normalizeNormals: if True, normals are scaled such that length=1 (or zero for zero-normals)
#  useDefaultColor: if True: ignores colors of the loaded mesh and uses defaultColor
#**output::dict: graphicsData in Exudyn dictionary format
#**notes: requires pymeshlab to be installed (pip install pymeshlab); materials and textures are currently not considered in the import functionality!
def FromPyMeshlabFile(fileName, defaultColor=color.defaultBody,
                      invertNormals=False, invertTriangles=False, normalizeNormals=True,
                      useDefaultColor=False, verbose=False):
    try:
        import pymeshlab #pip install pymeshlab
    except:
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
#**function: generate graphics data from STL file, allowing text or binary format; requires numpy-stl to be installed; additionally can scale, rotate and translate
#**input:
#  fileName: string containing directory and filename of STL-file (in text / SCII format) to load
#  color: provided as list of 4 RGBA values
#  verbose: if True, useful information is provided during reading
#  density: if given and if verbose, mass, volume, inertia, etc. are computed
#  scale: point coordinates are transformed by scaling factor
#  invertNormals: if True, orientation of normals (usually pointing inwards in STL mesh) are inverted for compatibility in Exudyn
#  invertTriangles: if True, triangle orientation (based on local indices) is inverted for compatibility in Exudyn
#**output: creates graphicsData, inverting the STL graphics regarding normals and triangle orientations (interchanged 2nd and 3rd component of triangle index)
#**notes: the model is first scaled, then rotated, then the offset pOff is added; finally min, max, mass, volume, inertia, com are computed!
def FromSTLfile(fileName, color=[0.,0.,0.,1.], verbose=False, density=0., scale=1., Aoff=[], pOff=[], invertNormals=True, invertTriangles=True):
    try:
        from stl import mesh
    except:
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


#**function: compute and return GraphicsData with edges and smoothend normals for mesh consisting of points and triangles (e.g., as returned from GraphicsData2PointsAndTrigs); ignores stored normals
#  graphicsData: single GraphicsData object of type TriangleList; existing edges are ignored
#  edgeColor: optional color for edges
#  edgeAngle: angle above which edges are added to geometry
#  addEdges: if True, edges are added in TriangleList of GraphicsData 
#  smoothNormals: if True, algorithm tries to smoothen normals at vertices; otherwise, uses triangle normals
#  roundDigits: number of digits, relative to max dimensions of object, at which points are assumed to be equal; too small or too larger number of digits may cause artifacts
#  triangleColor: if triangleColor is set to a RGBA color, this color is used for the new triangle mesh throughout; otherwise, stored colors are unchanged
#**output: returns GraphicsData with added edges and smoothed normals
#**notes: this function is suitable for STL import; it assumes that all colors in graphicsData are the same and only takes the first color!
def AddEdgesAndSmoothenNormals(graphicsData, edgeColor = color.black, edgeAngle = 0.25*pi,
                               addEdges=True, smoothNormals=True, roundDigits=5, 
                               triangleColor = []):
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

#**function: export given graphics data (only type TriangleList allowed!) to STL ascii file using fileName
#**input:
#  graphicsData: a single GraphicsData dictionary with type='TriangleList', no list of GraphicsData
#  fileName: file name including (local) path to export STL file
#  solidName: optional name used in STL file
#  invertNormals: if True, orientation of normals (usually pointing inwards in STL mesh) are inverted for compatibility in Exudyn
#  invertTriangles: if True, triangle orientation (based on local indices) is inverted for compatibility in Exudyn
def ExportSTL(graphicsData, fileName, solidName='ExudynSolid', invertNormals=True, invertTriangles=True):
    if graphicsData['type'] != 'TriangleList':
        raise ValueError('ExportSTL: invalid graphics data type; only TriangleList allowed')
        
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
