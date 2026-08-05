#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test for NGsolve interface with fem, using OCC Extrude() and Revolve()
#
# Author:   Johannes Gerstmayr 
# Date:     2026-02-02
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
from exudyn.FEM import HCBstaticModeSelection, FEMinterface, ObjectFFRFreducedOrderInterface

SC = exu.SystemContainer()
mbs = SC.AddSystem()

import numpy as np

import time

import netgen.occ as occ
from ngsolve import Mesh, Draw


width = 3
height = 2
length = 4
thickness = 0.08
arc = 0.5
bDim = 0.1
hBox = 0.8*height
arc2 = 0.4

#create frame
wp = occ.WorkPlane(occ.Axes(p=(0,0,0), n=occ.X, h=occ.Y)) #axis=x, plane: (h=Y, v=Z)
#wp.MoveTo(0,0).Line(0.5*width*2).Rotate(90).Line(height).Rotate(45).Line(0.5).Close()
wp.MoveTo(0,0).Line(0.5*width-arc).Arc(arc,90).Line(height-arc).Rotate(90).Line(thickness).\
        Rotate(90).Line(height-arc).Arc(arc-thickness,-90).Line(width-2*arc).Arc(arc-thickness,-90).\
        Line(height-arc).Rotate(90).Line(thickness).Rotate(90).Line(height-arc).\
        Arc(arc,90).Close()
    
frame = wp.Face().Extrude(length)
interfaceNameList = ['ground']
frame.faces.Min((0,0,1)).name = 'ground'

#create tube
rTube = (0.5*width-thickness-bDim*0.5)
rTubeInner = 0.5*rTube
lTube = length
wp1 = occ.WorkPlane(occ.Axes(p=(0,0,0), n=occ.Y, h=occ.Z)) #axis=X, plane: (h=X, v=Z)

#MoveTo(h,v): move cursor to position in workplane
#LineTo(h,v): create wire to to position in workplane
#Line(dx,dy): create wire in direction given
#Arc( radius, angleDegree): create arc in current direction with radius and angle
#ArcTo:  #destination h, v and destination tangent
#Rotate(angleDegree): change current direction relative to previous one (affects Line() and Arc() )
wp1.MoveTo( rTube, 0 )\
   .LineTo( rTube, lTube)\
   .LineTo( rTube-thickness, lTube)\
   .LineTo( rTube-thickness, thickness)\
   .Line( -thickness*1.5,0).Rotate(180)\
   .Arc( arc2, -90)\
   .LineTo( rTubeInner+thickness, lTube-thickness)\
   .LineTo( rTube-thickness*1.5, lTube-thickness)\
   .LineTo( rTube-thickness*1.5, lTube)\
   .LineTo( rTubeInner+thickness, lTube)\
   .LineTo( rTubeInner, lTube)\
   .LineTo( rTubeInner, arc2 )\
   .ArcTo(rTubeInner+arc2, 0, (0,-1))\
   .Close()


axis1 = occ.Axis((0,0,0),occ.X)
tube = wp1.Face().Revolve(axis1,360).Move((0,0,hBox))

#boxes between frame and tube:
vBox = [2*0.5*bDim,2*0.5*bDim,0.5*bDim]

boxList = []
for fy in [-1,1]:
    for fx in [0,0.5,1]:
        pBox = np.array((0.5*2*bDim+(length-2*bDim)*fx,fy*(-0.5*width+thickness+vBox[1]),hBox))
        box = occ.Box(tuple(pBox-vBox),
                      tuple(pBox+vBox))
        boxList.append(box)

geo = occ.OCCGeometry(frame+tube+boxList[0]+boxList[1]+boxList[2]+boxList[3]+boxList[4]+boxList[5])
print('meshing ...')
maxh=15

if False:
    import netgen.gui #this starts netgen gui; Press button "Visual" and activate "Auto-redraw after (sec)"; Then select "Mesh"


geoMesh = geo.GenerateMesh(maxh=maxh, 
                           curvaturesafety=2,
                           )

mesh = Mesh(geoMesh)


gFloor = graphics.CheckerBoard(point=[1,0,0.],size=8)
oGround = mbs.CreateGround(graphicsDataList=[gFloor])

if False: #just show mesh:
    [points, triangles, normals] = graphics.NGsolveMesh2PointsAndTrigs(mesh=mesh, 
                                                                       scale=1,
                                                                       meshOrder=2,
                                                                       addNormals=True,
                                                                       )
    color = graphics.color.steelblue
    meshColor=graphics.color.lawngreen[0:3]+[graphics.material.indexChrome]
                                         
    gMesh = graphics.FromPointsAndTrigs(points, triangles, normals=normals,
                                        color=meshColor)
    mbs.CreateGround(graphicsDataList=[gMesh])
    SC.visualizationSettings.view0.scene.showFaceEdges = True


rho = 2800
Emodulus = 8e11
nu = 0.3
femInterface = FEMinterface()
[bfM, bfK, fes] = femInterface.ImportMeshFromNGsolve(mesh, 
                                                     density=rho, youngsModulus=Emodulus, poissonsRatio=nu,
                                                     boundaryNamesList=interfaceNameList,
                                                     meshOrder=2
                                                     )
    
[boundaryNodesList, boundaryWeightsList] = femInterface.GetBoundaryNodeSetsAsLists()
femInterface.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryNodesList,
                                           nEigenModes=24,
                                           excludeRigidBodyMotion=True,
                                           boundaryNodesWeights=boundaryWeightsList,
                                           computationMode=HCBstaticModeSelection.RBE2)

print('eigenfrequencies (Hz):\n',femInterface.GetEigenFrequenciesHz(),sep='')

#invert boundary faces (BUG in surface creation in FEM?)
trigs = femInterface.surface[0]['Trigs']
femInterface.surface[0]['Trigs'][:,[0,2,1]] = trigs

cms = ObjectFFRFreducedOrderInterface(femInterface)

objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                              initialVelocity=[0,0,0], 
                                              initialAngularVelocity=[0,0,0],
                                              color=[0.9,0.9,0.9,1.],
                                              )

mbs.Assemble()


#%%
if True: #activate to animate modes
    from exudyn.interactive import AnimateModes

    SC.visualizationSettings.nodes.show = False
    SC.visualizationSettings.view0.scene.showFaceEdges = False
    SC.visualizationSettings.openGL.multiSampling = 4
    SC.visualizationSettings.openGL.light0.shadow = 0.2
    SC.visualizationSettings.openGL.light0.position=[2,2,10,1]
    SC.visualizationSettings.openGL.light0.diffuse = 0.4
    SC.visualizationSettings.openGL.lineWidth=2
    SC.visualizationSettings.openGL.lightModelAmbient = [0.6,0.6,0.6,1]
    SC.visualizationSettings.view0.window.renderWindowSize = [1400,1024]
    #SC.visualizationSettings.view0.camera.clippingPlaneNormal = [0,-1,0]
    
    
    nodeNumber = objFFRF['nGenericODE2'] #this is the node with the generalized coordinates

    SC.renderer.Start()              #start graphics visualization
    SC.renderer.SetModelView(zoom=2.011357,rotationVector=[-0.9999199,0.6224784,0.9588339],centerPoint=[0.7874073,1.334914,0])
    AnimateModes(SC, mbs, nodeNumber, period=0.1, 
                 showTime=False, scaleAmplitude=20,
                 runOnStart=True)
    import sys
    sys.exit()


SC.visualizationSettings.view0.window.renderWindowSize=[1200,800]
SC.visualizationSettings.general.autoFitScene=False

#SC.visualizationSettings.view0.scene.drawCoordinateSystem = False
SC.visualizationSettings.openGL.lineWidth = 2
#SC.visualizationSettings.openGL.light0.position = [-2.0, 4.0, 1.0, 1.0]
SC.visualizationSettings.loads.show = False
SC.visualizationSettings.openGL.light0.position=[2,2,10,1]

#raytracing options
SC.visualizationSettings.openGL.multiSampling = 2
SC.visualizationSettings.openGL.light0.shadow = 0.2
SC.visualizationSettings.openGL.light1.shadow = 0.2
SC.visualizationSettings.raytracer.numberOfThreads = 64

SC.visualizationSettings.view0.camera.useRaytracer = False #set True for raytracing
SC.visualizationSettings.raytracer.keepWindowActive= True
SC.visualizationSettings.raytracer.advanced.searchTreeFactor = 8


#visualize in Exudyn:
SC.renderer.Start()              #start graphics visualization

SC.renderer.SetModelView(zoom=2.011357,rotationVector=[-0.9999199,0.6224784,0.9588339],centerPoint=[0.7874073,1.334914,0])
SC.renderer.DoIdleTasks() #press space to continue

SC.renderer.Stop() #safely close rendering window!
    

