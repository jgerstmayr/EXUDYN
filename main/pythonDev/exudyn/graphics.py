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
                                      HT2rotationMatrix, HT2translation
import exudyn.graphicsDataUtilities as gdu

from exudyn.advancedUtilities import IsEmptyList

#constants and fixed structures:
import numpy as np #LoadSolutionFile
import copy as copy #to be able to copy e.g. lists
from math import pi, sin, cos

graphicsDataNormalsFactor = 1. #this is a factor being either -1. [original normals pointing inside; until 2022-06-27], while +1. gives corrected normals pointing outside
graphicsDataSwitchTriangleOrder = False #this is the old ordering of triangles in some Sphere or Cylinder functions, causing computed normals to point inside

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#colors ...

#this is a pure structure with default values; user will see this similar as a sub-module: graphics.color.red
class color: pass

color.red = exudyn.graphicsDataUtilities.color4red
color.green = exudyn.graphicsDataUtilities.color4green
color.blue = exudyn.graphicsDataUtilities.color4blue

color.cyan = exudyn.graphicsDataUtilities.color4cyan
color.magenta = exudyn.graphicsDataUtilities.color4magenta
color.yellow = exudyn.graphicsDataUtilities.color4yellow

color.orange = exudyn.graphicsDataUtilities.color4orange
color.pink = exudyn.graphicsDataUtilities.color4pink
color.lawngreen = exudyn.graphicsDataUtilities.color4lawngreen

color.springgreen = exudyn.graphicsDataUtilities.color4springgreen
color.violet = exudyn.graphicsDataUtilities.color4violet
color.dodgerblue = exudyn.graphicsDataUtilities.color4dodgerblue

color.lightred = exudyn.graphicsDataUtilities.color4lightred
color.lightgreen = exudyn.graphicsDataUtilities.color4lightgreen
color.steelblue = exudyn.graphicsDataUtilities.color4steelblue
color.brown = exudyn.graphicsDataUtilities.color4brown

color.black = exudyn.graphicsDataUtilities.color4black
color.darkgrey = exudyn.graphicsDataUtilities.color4darkgrey
color.darkgrey2 = exudyn.graphicsDataUtilities.color4darkgrey2
color.grey = exudyn.graphicsDataUtilities.color4grey
color.lightgrey = exudyn.graphicsDataUtilities.color4lightgrey
color.lightgrey2 = exudyn.graphicsDataUtilities.color4lightgrey2
color.white = exudyn.graphicsDataUtilities.color4white

color.default = exudyn.graphicsDataUtilities.color4default

#a convenient list for creating automatic coloring of objects
colorList = exudyn.graphicsDataUtilities.color4list

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#basic geometrical objects
# Brick = exudyn.graphicsDataUtilities.GraphicsDataOrthoCubePoint
# Cuboid = exudyn.graphicsDataUtilities.GraphicsDataCube
# Sphere = exudyn.graphicsDataUtilities.GraphicsDataSphere
# Cylinder = exudyn.graphicsDataUtilities.GraphicsDataCylinder

# Lines = exudyn.graphicsDataUtilities.GraphicsDataLine
# Circle = exudyn.graphicsDataUtilities.GraphicsDataCircle
# Text = exudyn.graphicsDataUtilities.GraphicsDataText

# #advanced objects
# RigidLink = exudyn.graphicsDataUtilities.GraphicsDataRigidLink
# SolidOfRevolution = exudyn.graphicsDataUtilities.GraphicsDataSolidOfRevolution
# Arrow = exudyn.graphicsDataUtilities.GraphicsDataArrow
# Basis = exudyn.graphicsDataUtilities.GraphicsDataBasis
# Frame = exudyn.graphicsDataUtilities.GraphicsDataFrame
# Quad = exudyn.graphicsDataUtilities.GraphicsDataQuad
# CheckerBoard = exudyn.graphicsDataUtilities.GraphicsDataCheckerBoard
# SolidExtrusion = exudyn.graphicsDataUtilities.GraphicsDataSolidExtrusion

#import/export and transformations
# FromSTLfile = exudyn.graphicsDataUtilities.GraphicsDataFromSTLfile
# FromSTLfileASCII = exudyn.graphicsDataUtilities.GraphicsDataFromSTLfileTxt
# FromPointsAndTrigs = exudyn.graphicsDataUtilities.GraphicsDataFromPointsAndTrigs
# ToPointsAndTrigs = exudyn.graphicsDataUtilities.GraphicsData2PointsAndTrigs
# ExportSTL = exudyn.graphicsDataUtilities.ExportGraphicsData2STL

# Move = exudyn.graphicsDataUtilities.MoveGraphicsData
# MergeTriangleLists = exudyn.graphicsDataUtilities.MergeGraphicsDataTriangleList
# AddEdgesAndSmoothenNormals = exudyn.graphicsDataUtilities.AddEdgesAndSmoothenNormals


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: generate graphics data for a sphere with point p and radius
#**input:
#  point: center of sphere (3D list or np.array)
#  radius: positive value
#  color: provided as list of 4 RGBA values
#  nTiles: used to determine resolution of sphere >=3; use larger values for finer resolution
#  addEdges: True or number of edges along sphere shell (under development); for optimal drawing, nTiles shall be multiple of 4 or 8
#  edgeColor: optional color for edges
#  addFaces: if False, no faces are added (only edges)
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
def Sphere(point=[0,0,0], radius=0.1, color=[0.,0.,0.,1.], nTiles = 8, 
           addEdges = False, edgeColor=color.black, addFaces=True):
    if nTiles < 3: print("WARNING: Sphere: nTiles < 3: set nTiles=3")
    
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
        for iphi in range(nTiles):
            z = -r*cos(pi*i0/nTiles)    #runs from -r .. r (this is the coordinate of the axis of circles)
            phi = 2*pi*iphi/nTiles #angle
            fact = sin(pi*i0/nTiles)

            x = fact*r*sin(phi)
            y = fact*r*cos(phi)

            vv = x*e0 + y*e1 + z*e2
            points += list(p + vv)
            
            n = ebu.Normalize(vv) #2022-06-27: corrected to (vv) to point outwards
            #print(n)
            normals += n
            
            colors += color

    
    if addFaces:
        for i0 in range(nTiles):
            for iphi in range(nTiles):
                p0 = i0*nTiles+iphi
                p1 = (i0+1)*nTiles+iphi
                iphi1 = iphi + 1
                if iphi1 >= nTiles: 
                    iphi1 = 0
                p2 = i0*nTiles+iphi1
                p3 = (i0+1)*nTiles+iphi1
    
                if graphicsDataSwitchTriangleOrder:
                    triangles += [p0,p3,p1, p0,p2,p3]
                else:
                    triangles += [p0,p1,p3, p0,p3,p2]
            
    data = {'type':'TriangleList', 'colors':colors, 
            'normals':normals, 
            'points':points, 
            'triangles':triangles}
    
    if type(addEdges) == bool and addEdges == True:
        addEdges = 3

    if addEdges > 0:
        data['edgeColor'] = list(edgeColor)

        edges = []
        hEdges = [] #edges at half of iphi
        nt = 2
        if addEdges > 1:
            nt = 4
        if addEdges > 3:
            nt = 8
        for j in range(nt):
            hEdges += [[]]
        hTiles = int(nTiles/nt)
        # hLast = [None]*nt
        # hFirst = [None]*nt
        sTiles = max(addEdges-1,1) #non-negative
        nStep = int(nTiles/sTiles)
        
        for i0 in range(nTiles):
            for iphi in range(nTiles):
                p0 = i0*nTiles+iphi
                p1 = (i0+1)*nTiles+iphi
                if i0%nStep == 0:
                    iphi1 = iphi + 1
                    if iphi1 >= nTiles: 
                        iphi1 = 0
                    p2 = i0*nTiles+iphi1
                    if addEdges>1:
                        edges += [p0, p2]
                if hTiles != 0:
                    if iphi%hTiles == 0:
                        j = int(iphi/hTiles)
                        if j < nt:
                            hEdges[j] += [p0,p1]
                            # if hLast[j] == None:
                            #     hLast[j] = p0
                            #     hFirst[j] = p0
                            # else:
                            #     hEdges[j] += [hLast[j], p0]
                            #     hLast[j] = p0
        
        for j in range(nt):
            #print('j=',j, hEdges[j], ', hFirst=',hFirst)
            if nt%2 == 0: #close edges only for even nt
                hEdges[j] += [hEdges[j][-1], hEdges[(j+int(nt/2))%nt][-1]]
                
            edges += hEdges[j]

        data['edges'] = edges
    
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
    data = [0]*(len(pList)*3)
    for i, p in enumerate(pList):
        data[i*3:i*3+3] = list(p)
    dataRect = {'type':'Line', 'color': list(color), 'data':data}

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
    return {'type':'Circle', 'color': list(color), 'radius': radius, 'position':list(point)}


#************************************************
#**function: generate graphics data for a text drawn at a 3D position
#**input: 
#  point: position of text
#  text: string representing text
#  color: provided as list of 4 RGBA values
#**nodes: text size can be adjusted with visualizationSettings.general.textSize, which affects the text size (=font size) globally
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
def Text(point=[0,0,0], text='', color=[0.,0.,0.,1.]): 
    return {'type':'Text', 'color': list(color), 'text':text, 'position':list(point)}


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

    colors=[]
    for i in range(8):
        colors=colors+color

    points = []
    for p in pList:
        points += p
#    points = [xMin,yMin,zMin, xMax,yMin,zMin, xMax,yMax,zMin, xMin,yMax,zMin,
#              xMin,yMin,zMax, xMax,yMin,zMax, xMax,yMax,zMax, xMin,yMax,zMax]

    #1-based ... triangles = [1,3,2, 1,4,3, 5,6,7, 5,7,8, 1,2,5, 2,6,5, 2,3,6, 3,7,6, 3,4,7, 4,8,7, 4,1,8, 1,5,8 ]
    #triangles = [0,2,1, 0,3,2, 6,4,5, 6,7,4, 0,1,4, 1,5,4, 1,2,5, 2,6,5, 2,3,6, 3,7,6, 3,0,7, 0,4,7]

    trigList = [[0,2,1], [0,3,2], #
                [6,4,5], [6,7,4], #
                [0,1,4], [1,5,4], #
                [1,2,5], [2,6,5], #
                [2,3,6], [3,7,6], #
                [3,0,7], [0,4,7]] #
    triangles = []
    # print('addNormals=',addNormals)
    if not addNormals:
        for i in range(6):
            if faces[i]:
                for j in range(2):
                    if addFaces:
                        triangles += trigList[i*2+j]
        data = {'type':'TriangleList', 'colors': colors, 'points':points, 'triangles':triangles}
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
        
        data = {'type':'TriangleList', 'colors': color*cnt, 'normals':normals, 'points':points2, 'triangles':triangles}

    if addEdges:
        edges = [0,1, 1,2, 2,3, 3,0,
                 4,5, 5,6, 6,7, 7,4,
                 0,4, 1,5, 2,6, 3,7 ]
        
        data['edges'] = edges
        data['edgeColor'] = list(edgeColor)
        
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
    return Cuboid(pList, list(color), addNormals=addNormals, addEdges=addEdges, edgeColor=edgeColor, addFaces=addFaces)


#**function: generate graphics data forfor orthogonal 3D block with center point and size
#**input: 
#  centerPoint: center of cube as 3D list or np.array
#  size: size as 3D list or np.array
#  color: list of 4 RGBA values
#  addNormals: add face normals to triangle information
#  addEdges: if True, edges are added in TriangleList of GraphicsData 
#  edgeColor: optional color for edges
#  addFaces: if False, no faces are added (only edges)
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects; if addEdges=True, it returns a list of two dictionaries
def Brick(centerPoint=[0,0,0], size=[0.1,0.1,0.1], color=[0.,0.,0.,1.], addNormals=False, addEdges=False, edgeColor=color.black, addFaces=True): 
    
    xMin = centerPoint[0] - 0.5*size[0]
    yMin = centerPoint[1] - 0.5*size[1]
    zMin = centerPoint[2] - 0.5*size[2]
    xMax = centerPoint[0] + 0.5*size[0]
    yMax = centerPoint[1] + 0.5*size[1]
    zMax = centerPoint[2] + 0.5*size[2]

    gCube = BrickXYZ(xMin, yMin, zMin, xMax, yMax, zMax, color, 
                                  addNormals=addNormals, addEdges=addEdges, edgeColor=edgeColor, addFaces=addFaces)
    if addEdges:
        gCube['edgeColor'] = list(edgeColor)
        gCube['edges'] = [0,1, 1,2, 2,3, 3,0,  0,4, 1,5, 2,6, 3,7,  4,5, 5,6, 6,7, 7,4]
        #print('new2')
    return gCube


#**function: generate graphics data for a cylinder with given axis, radius and color; nTiles gives the number of tiles (minimum=3)
#**input:
#  pAxis: axis point of one face of cylinder (3D list or np.array)
#  vAxis: vector representing the cylinder's axis (3D list or np.array)
#  radius: positive value representing radius of cylinder
#  color: provided as list of 4 RGBA values
#  nTiles: used to determine resolution of cylinder >=3; use larger values for finer resolution
#  angleRange: given in rad, to draw only part of cylinder (halfcylinder, etc.); for full range use [0..2 * pi]
#  lastFace: if angleRange != [0,2*pi], then the faces of the open cylinder are shown with lastFace = True
#  cutPlain: only used for angleRange != [0,2*pi]; if True, a plane is cut through the part of the cylinder; if False, the cylinder becomes a cake shape ...
#  addEdges: if True, edges are added in TriangleList of GraphicsData; if addEdges is integer, additional int(addEdges) lines are added on the cylinder mantle
#  edgeColor: optional color for edges
#  addFaces: if False, no faces are added (only edges)
#  alternatingColor: if given, optionally another color in order to see rotation of solid; only works, if angleRange=[0,2*pi]
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
def Cylinder(pAxis=[0,0,0], vAxis=[0,0,1], radius=0.1, color=[0.,0.,0.,1.], nTiles = 16, 
                         angleRange=[0,2*pi], lastFace = True, cutPlain = True, 
                         addEdges=False, edgeColor=color.black,
                         addFaces=True, **kwargs):  

    if nTiles < 3: print("WARNING: GraphicsDataCylinder: nTiles < 3: set nTiles=3")
    
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
        color2 = kwargs['alternatingColor']

    colors=[]
    #for i in range(2*n+2*nTiles):
    #    colors += color
    n2 = int(nTiles/2)    
    for i in range(2):
        colors += color
    for j in range(4):
        for i in range(n2):
            colors += color
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
        points0 += pAxis + pAxis1 + p2 + p3 + pAxis + pAxis1 + p4 + p5
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

    # print('points len=', len(points0))
    # print('normals len=', len(normals0))
    if not addFaces:
        triangles = []

    #triangle normals point inwards to object ...
    data = {'type':'TriangleList', 'colors':colors, 
            'normals':normals0, 
            'points':points0, 'triangles':triangles}

    if addEdges:
        data['edgeColor'] = list(edgeColor)
        
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
        
        data['edges'] = edges

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
    triangles += list(trigs1)
    
    trigs2 = np.array(data2['triangles'])
    trigs2 += np1
    triangles += list(trigs2)
    
    points = data0['points'] + data1['points'] + data2['points']
    normals = data0['normals'] + data1['normals'] + data2['normals']
    colors = data0['colors'] + data1['colors'] + data2['colors']
    
    data = {'type':'TriangleList', 'colors':colors, 
            'normals':normals, 
            'points':points, 'triangles':triangles}
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
                                  addEdges = False, edgeColor=color.black, addFaces=True, **kwargs):  

    if len(contour) < 2: 
        raise ValueError("ERROR: SolidOfRevolution: contour must contain at least 2 points")
    if nTiles < 3: 
        print("WARNING: SolidOfRevolution: nTiles < 3: set nTiles=3")

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
    contourNormals += [contourNormals[-1]] #normal for last point same as previous

    if smoothContour:
        contourNormals2 = [contourNormals[0]]
        for j in range(len(contour)-1):
            ns = ebu.Normalize(np.array(contourNormals[j]) + np.array(contourNormals[j+1])) #not fully correct, but sufficient
            contourNormals2 += [list(ns)]
        contourNormals = contourNormals2

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
                nc1 = contourNormals[j+1]
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
                    triangles += [i+k,n+i+k+1,n+i+k]
                    triangles += [i+k,i+1+k,n+i+k+1]
                else:
                    triangles += [i+k,n+k,n+i+k]
                    triangles += [i+k,k,n+k]

    #triangle normals point inwards to object ...
    data = {'type':'TriangleList', 'colors':colors, 
            'normals':normals, 
            'points':points, 'triangles':triangles}


    if addEdges > 0:
        data['edgeColor'] = list(edgeColor)
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

        data['edges'] = edges

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
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
def Basis(origin=[0,0,0], rotationMatrix = np.eye(3), length = 1, colors=[color.red, color.green, color.blue], 
                      headFactor = 2, headStretch = 4, nTiles = 12, **kwargs):  
    radius = 0.01*length
    if 'radius' in kwargs:
        radius = kwargs['radius']

    A = np.array(rotationMatrix)
    g1 = Arrow(origin,A@[length,0,0],radius, colors[0], headFactor, headStretch, nTiles)
    g2 = Arrow(origin,A@[0,length,0],radius, colors[1], headFactor, headStretch, nTiles)
    g3 = Arrow(origin,A@[0,0,length],radius, colors[2], headFactor, headStretch, nTiles)

    return MergeTriangleLists(MergeTriangleLists(g1,g2),g3)

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

    data = {'type':'TriangleList', 'colors': colors, 'points':points, 'triangles':triangles}
    #print(data)
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

    [v,n1,n2] = ComputeOrthonormalBasisVectors(normal)
    p0=np.array(point)
    points = [list(p0-0.5*size*n1-0.5*size2*n2),
              list(p0+0.5*size*n1-0.5*size2*n2),
              list(p0+0.5*size*n1+0.5*size2*n2),
              list(p0-0.5*size*n1+0.5*size2*n2)]

    return Quad(points, color=list(color), alternatingColor=alternatingColor, 
                            nTiles=nTiles, nTilesY=nTiles2)

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: create graphicsData for solid extrusion based on 2D points and segments; by default, the extrusion is performed in z-direction;
#            additional transformations are possible to translate and rotate the extruded body;
#**input:
#  vertices: list of pairs of coordinates of vertices in mesh [x,y], see ComputeTriangularMesh(...)
#  segments: list of segments, which are pairs of node numbers [i,j], defining the boundary of the mesh;
#            the ordering of the nodes is such that left triangle = inside, right triangle = outside; see ComputeTriangularMesh(...)
#  height:   height of extruded object
#  rot:      rotation matrix, which the extruded object point coordinates are multiplied with before adding offset
#  pOff:     3D offset vector added to extruded coordinates; the z-coordinate of the extrusion object obtains 0 for the base plane, z=height for the top plane
#  smoothNormals: if True, algorithm tries to smoothen normals at vertices and normals are added; creates more points; if False, triangle normals are used internally 
#  addEdges: if True or 1, edges at bottom/top are included in the GraphicsData dictionary; if 2, also mantle edges are included
#  edgeColor: optional color for edges
#  addFaces: if False, no faces are added (only edges)
#**output: graphicsData dictionary, to be used in visualization of EXUDYN objects
def SolidExtrusion(vertices, segments, height, rot = np.diag([1,1,1]), pOff = [0,0,0], color = [0,0,0,1],
                               smoothNormals = False, addEdges = False, edgeColor=color.black, addFaces=True):
    n = len(vertices)
    n2 = n*2 #total number of vertices
    ns = len(segments)
    colors=[]
    for i in range(n2):
        colors+=color

    edges = []
    mantleEdges = (addEdges == 2)

    points = [[]]*n2
    for i in range(n):
        points[i] = [vertices[i][0],vertices[i][1],0]
    for i in range(n):
        points[i+n] = [vertices[i][0],vertices[i][1],height]

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
        for seg in segments:
            dirSeg = ebu.Normalize(np.array(vertices[seg[1]]) - np.array(vertices[seg[0]]))
            dirSeg3D = [dirSeg[1], -dirSeg[0], 0.] #this way points outwards ...
            pointNormals[seg[0]+2*n,:] += dirSeg3D
            pointNormals[seg[1]+2*n,:] += dirSeg3D
            pointNormals[seg[0]+3*n,:] += dirSeg3D
            pointNormals[seg[1]+3*n,:] += dirSeg3D
        
        for i in range(n2):
            lenSeg = ebu.NormL2(pointNormals[i,:])
            if lenSeg != 0.:
                pointNormals[i,:] = (1/lenSeg)*pointNormals[i,:]
            
        points2 = [[]]*n2
        for i in range(n):
            points2[i] = [vertices[i][0],vertices[i][1],0.]
            pointNormals[i+0*n,:] = [0.,0.,-1.]
            #pointNormals[i+2*n,:] = [0,0,-1]
        for i in range(n):
            points2[i+n] = [vertices[i][0],vertices[i][1],height]
            pointNormals[i+1*n,:] = [0.,0.,1.]
            #pointNormals[i+3*n,:] = [0,0,1]
        

    #transform points:
    pointsTransformed = []
    npRot = np.array(rot)
    npPoff = np.array(pOff)

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
        #print(list(trigs[i]))
        # trigList[i] = list(trigs[i])
        t = list(trigs[i])
        t.reverse()
        trigList[i] = copy.copy(t)
    for i in range(nt):
        t = list(trigs[i]+n)
        # t.reverse()
        trigList[i+nt] = copy.copy(t)
        
    #print("ns=",ns)
    #print("nt=",nt)
    off = n2*int(smoothNormals)
    for i in range(ns):
        trigList[2*nt+2*i  ] = [segments[i][0]+off,segments[i][1]+off,  segments[i][1]+n+off]
        trigList[2*nt+2*i+1] = [segments[i][0]+off,segments[i][1]+n+off,segments[i][0]+n+off]

        if mantleEdges:
            edges += [segments[i][0]+off,segments[i][0]+n+off]

    #print("trigList=",trigList)
    triangles = []
    if addFaces:
        for t in trigList:
            triangles += t
   
    data = {'type':'TriangleList', 'colors': colors, 'points':pointsTransformed, 'triangles':triangles}
    if addEdges:
        data['edgeColor'] = list(edgeColor)
        data['edges'] = edges

    if smoothNormals:
        data['normals'] = list(pointNormals.flatten())

    return data







#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#**function: convert triangles and points as returned from graphics.ToPointsAndTrigs(...) 
#**input: 
#  points: list of np.array with 3 floats per point 
#  triangles: list of np.array with 3 int per triangle (0-based indices to triangles)
#  color: provided as list of 4 RGBA values or single list of (number of points)*[4 RGBA values]
#**output: returns GraphicsData with type TriangleList
def FromPointsAndTrigs(points, triangles, color=[0.,0.,0.,1.]):
    pointList = list(np.array(points).flatten())
    triangleList = list(np.array(triangles).flatten())
    nPoints = int(len(pointList)/3)
    if len(color) == 4*nPoints:
        colorList = list(color) #without list() potential problem with mutable default value
    elif len(color) == 4:
        colorList = list(color)*nPoints
    else:
        print('number of points=', nPoints)
        print('number of trigs=', len(triangleList)/3)
        print('number of colors=', len(color))
        raise ValueError('FromPointsAndTrigs: color must have either 4 RGBA values or 4*(number of points) RGBA values as a list')
    data = {'type':'TriangleList', 
            'colors': colorList, 
            'points':pointList, 
            'triangles':triangleList}
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
#**function: add rigid body transformation to GraphicsData, using position offset (global) pOff (list or np.array) and rotation Aoff (transforms local to global coordinates; list of lists or np.array); see Aoff how to scale coordinates!
#**input:
#  g: graphicsData to be transformed
#  pOff: 3D offset as list or numpy.array added to rotated points
#  Aoff: 3D rotation matrix as list of lists or numpy.array with shape (3,3); if A is scaled by factor, e.g. using 0.001*np.eye(3), you can also scale the coordinates!!!
#**output: returns new graphcsData object to be used for drawing in objects
#**notes: transformation corresponds to HomogeneousTransformation(Aoff, pOff), transforming original coordinates v into vNew = pOff + Aoff @ v
def Move(g, pOff, Aoff):
    p0 = np.array(pOff)
    A0 = np.array(Aoff)
    
    if g['type'] == 'TriangleList': 
        gNew = {'type':'TriangleList'}
        gNew['colors'] = copy.copy(g['colors'])
        gNew['triangles'] = copy.copy(g['triangles'])
        if 'edges' in g:
            gNew['edges'] = copy.copy(g['edges'])
        if 'edgeColor' in g:
            gNew['edgeColor'] = copy.copy(g['edgeColor'])

        n=int(len(g['points'])/3)
        v0 = np.array(g['points'])
        v = np.kron(np.ones(n),p0) + (A0 @ v0.reshape((n,3)).T).T.flatten()
        
        gNew['points'] = list(v)
        if 'normals' in g:
            n0 = np.array(g['normals'])
            gNew['normals'] = list((A0 @ n0.reshape((n,3)).T).T.flatten() )
        
        # #original, slow:
        # for i in range(n):
        #     v = gNew['points'][i*3:i*3+3]
        #     v = p0 + A0 @ v
        #     gNew['points'][i*3:i*3+3] = list(v)
        # if 'normals' in gNew:
        #     n=int(len(g['normals'])/3)
        #     for i in range(n):
        #         v = gNew['normals'][i*3:i*3+3]
        #         v = A0 @ v
        #         gNew['normals'][i*3:i*3+3] = list(v)
    elif g['type'] == 'Line':
        gNew = copy.deepcopy(g)
        n=int(len(g['data'])/3)
        for i in range(n):
            v = gNew['data'][i*3:i*3+3]
            v = p0 + A0 @ v
            gNew['data'][i*3:i*3+3] = list(v)
    elif g['type'] == 'Text':
        gNew = copy.deepcopy(g)
        v = p0 + A0 @ gNew['position']
        gNew['position'] = list(v)
    elif g['type'] == 'Circle':
        gNew = copy.deepcopy(g)
        v = p0 + A0 @ gNew['position']
        gNew['position'] = list(v)
        if 'normal' in gNew:
            v = A0 @ gNew['normal']
            gNew['normal'] = list(v)
    else:
        raise ValueError('Move: unsupported graphics data type')
    return gNew

#************************************************
#**function: merge 2 different graphics data with triangle lists
#**input: graphicsData dictionaries g1 and g2 obtained from GraphicsData functions
#**output: one graphicsData dictionary with single triangle lists and compatible points and normals, to be used in visualization of EXUDYN objects; edges are merged; edgeColor is taken from graphicsData g1
def MergeTriangleLists(g1,g2):
    np = int(len(g1['points'])/3) #number of points
    useNormals = False
    if 'normals' in g1 and 'normals' in g2:
        useNormals = True

    if np*4 != len(g1['colors']):
        raise ValueError('MergeTriangleLists: incompatible colors and points in lists')

    if useNormals:
        if np*3 != len(g1['normals']):
            raise ValueError('MergeTriangleLists: incompatible normals and points in lists')
        data = {'type':'TriangleList', 'colors':copy.copy(g1['colors']), 'normals':copy.copy(g1['normals']), 
                'points': copy.copy(g1['points']), 'triangles': copy.copy(g1['triangles'])}

        data['normals'] += g2['normals']
    else:
        data = {'type':'TriangleList', 'colors':copy.copy(g1['colors']),
                'points': copy.copy(g1['points']), 'triangles': copy.copy(g1['triangles'])}
    
    data['colors'] += g2['colors']
    data['points'] += g2['points']

    #copy and merge edges; edges can be available only in one triangle list
    if 'edges' in g1:
        data['edges'] = copy.copy(g1['edges'])
    if 'edges' in g2:
        edges2 = copy.copy(g2['edges'])
        if 'edges' not in data:
            data['edges'] = []
        else:
            for i in range(len(edges2)):
                edges2[i] += np #add point offset
        
        data['edges'] += edges2
    if 'edgeColor' in g1:
        data['edgeColor'] = copy.copy(g1['edgeColor']) #only taken from g1
    elif 'edgeColor' in g2:
        data['edgeColor'] = copy.copy(g2['edgeColor']) #only taken from g1

    for p in g2['triangles']:
        data['triangles'] += [int(p + np)] #add point offset for correct connectivity

    return data


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
    if verbose: print("read STL file: "+fileName)

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
        print('large ascii STL file; switch to numpy-stl and binary format for faster loading!')

    while lineCnt < nLines and fileLines[lineCnt].strip().split()[0] != 'endsolid':
        if lineCnt%100000 == 0 and lineCnt !=0: 
            if verbose: print("  read line",lineCnt," / ", len(fileLines))

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
        print('FromSTLfile:')
        print('  max point=', list(data.max_))
        print('  min point=', list(data.min_))
        print('  STL points=', nPoints)
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
        print('  volume =', volume)
        print('  center of mass =', list(COM))
        print('  inertia =', list(inertia))
    
    # print('STL points3=', nPoints3)
    
    colors = color*nPoints
    #triangles = list(np.arange(0,nPoints))#wrong orientation ==> reverse
    if invertTriangles:
        triangles = list(np.arange(nPoints-1,-1,-1))#inverted sorting
    else:
        triangles = list(np.arange(0,nPoints))      #unmodified sorting of indices
    points = list(data.points.flatten())
    nf = 1.-2.*int(invertNormals) #+1 or -1 (inverted)
    normals = list(np.kron([nf,nf,nf],data.normals).flatten()) #normals must be per point

    dictGraphics = {'type':'TriangleList', 'colors':colors, 'normals':normals, 'points':points, 'triangles':triangles}
    if density == 0:
        return dictGraphics 
    else:
        return [dictGraphics, dictData]


#**function: compute and return GraphicsData with edges and smoothend normals for mesh consisting of points and triangles (e.g., as returned from GraphicsData2PointsAndTrigs)
#  graphicsData: single GraphicsData object of type TriangleList; existing edges are ignored
#  edgeColor: optional color for edges
#  edgeAngle: angle above which edges are added to geometry
#  roundDigits: number of digits, relative to max dimensions of object, at which points are assumed to be equal
#  smoothNormals: if True, algorithm tries to smoothen normals at vertices; otherwise, uses triangle normals
#  addEdges: if True, edges are added in TriangleList of GraphicsData 
#  triangleColor: if triangleColor is set to a RGBA color, this color is used for the new triangle mesh throughout
#**output: returns GraphicsData with added edges and smoothed normals
#**notes: this function is suitable for STL import; it assumes that all colors in graphicsData are the same and only takes the first color!
def AddEdgesAndSmoothenNormals(graphicsData, edgeColor = color.black, edgeAngle = 0.25*pi,
                           pointTolerance=5, addEdges=True, smoothNormals=True, roundDigits=5, 
                           triangleColor = []):
    from math import acos # ,sin, cos

    oldColors = copy.copy(graphicsData['colors']) #2022-12-06: accepts now all colors; graphicsData['colors'][0:4]    
    [points, trigs]=ToPointsAndTrigs(graphicsData)
    # [points, trigs]=RefineMesh(points, trigs)

    nPoints = len(points)
    nColors = int(len(oldColors)/4)

    triangleColorNew = list(triangleColor)

    if nColors != nPoints:
        print('WARNING: AddEdgesAndSmoothenNormals: found inconsistent colors; they must match the point list in graphics data')
        if triangleColorNew == []:
            triangleColorNew = graphicsData['colors'][0:4]

    if len(triangleColorNew) != 4 and len(triangleColorNew) != 0:
        triangleColorNew = [1,0,0,1]
        print('WARNING: AddEdgesAndSmoothenNormals: colors invalid; using default')

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
    # print('smoothen np=', nPoints)
    
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
                        #print('neighbours ', cntTrig, t)
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
        graphicsData2['edges'] = edges
        graphicsData2['edgeColor'] = list(edgeColor)

    if smoothNormals:
        graphicsData2['normals'] = list(np.array(pointNormals).flatten())
    
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
            
            f.write('facet normal '+str(-n[0]) + ' ' + str(-n[1]) + ' ' + str(-n[2]) + '\n') #normals inverted
            f.write('outer loop\n')
            f.write('vertex '+str(p[0][0]) + ' ' + str(p[0][1]) + ' ' + str(p[0][2]) + '\n')
            f.write('vertex '+str(p[2][0]) + ' ' + str(p[2][1]) + ' ' + str(p[2][2]) + '\n') #point index reversed!
            f.write('vertex '+str(p[1][0]) + ' ' + str(p[1][1]) + ' ' + str(p[1][2]) + '\n')
                
            f.write('endloop\n')
            f.write('endfacet\n')

        f.write('endsolid '+solidName+'\n')

            
    # data = {'type':'TriangleList', 'colors':colors, 
    #         'normals':normals, 
    #         'points':points, 'triangles':triangles}

