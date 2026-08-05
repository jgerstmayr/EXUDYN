#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test for NGsolve interface with fem, using OCC Extrude() and interface names
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
height = 1.5
length = 4
thickness = 0.1
arc = 0.5
maxh = 0.5

#simple solid of revolution geometry:
wp = occ.WorkPlane(occ.Axes(p=(0,0,0), n=occ.X, h=occ.Y))
#wp.MoveTo(0,0).Line(0.5*width*2).Rotate(90).Line(height).Rotate(45).Line(0.5).Close()
wp.MoveTo(0,0).Line(0.5*width-arc).Arc(arc,90).Line(height-arc).Rotate(90).Line(thickness).\
        Rotate(90).Line(height-arc).Arc(arc-thickness,-90).Line(width-2*arc).Arc(arc-thickness,-90).\
        Line(height-arc).Rotate(90).Line(thickness).Rotate(90).Line(height-arc).\
        Arc(arc,90).Close()
    
frame = wp.Face().Extrude(length)

bDim = 0.1
vBox = [0.5*2*bDim,0.5*bDim,0.5*bDim]

boxList = []
interfaceNameList = []
for fy in [-1,1]:
    for fx in [0,0.5,1]:
        name = 'box_y'+str(fy)+'_x'+str(fx)
        interfaceNameList.append(name)
        pBox = np.array((0.5*2*bDim+(length-2*bDim)*fx,fy*(-0.5*width+thickness+0.5*bDim),0.5*height))
        box = occ.Box(tuple(pBox-vBox),
                      tuple(pBox+vBox))
        if fy > 0:
            box.faces.Max((0, 1, 0)).name = name
        else:
            box.faces.Min((0, 1, 0)).name = name
        boxList.append(box)

geo = occ.OCCGeometry(frame+boxList[0]+boxList[1]+boxList[2]+boxList[3]+boxList[4]+boxList[5])
exu.Print('meshing ...')

if False:
    import netgen.gui #this starts netgen gui; Press button "Visual" and activate "Auto-redraw after (sec)"; Then select "Mesh"


geoMesh = geo.GenerateMesh(maxh=maxh, 
                           curvaturesafety=1.5, 
                           )

mesh = Mesh(geoMesh)


gFloor = graphics.CheckerBoard(point=[0,0,0.],size=8)
oGround = mbs.CreateGround(graphicsDataList=[gFloor])

#+++++++++++++++++++++++++++++++++++++++++++++
#create FEM
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
                                           nEigenModes=8,
                                           excludeRigidBodyMotion=True,
                                           boundaryNodesWeights=boundaryWeightsList,
                                           computationMode=HCBstaticModeSelection.RBE2)

exu.Print('eigenfrequencies (Hz):\n',femInterface.GetEigenFrequenciesHz(),sep='')

cms = ObjectFFRFreducedOrderInterface(femInterface)

objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                              initialVelocity=[0,0,0], 
                                              initialAngularVelocity=[0,0,0],
                                              color=[0.9,0.9,0.9,1.],
                                              )

mbs.Assemble()



#+++++++++++++++++++++++++++++++++++++++++++++
if True: #activate to animate modes
    from exudyn.interactive import AnimateModes
    SC.visualizationSettings.nodes.show = False
    SC.visualizationSettings.view0.scene.showFaceEdges = True
    SC.visualizationSettings.openGL.multiSampling=2
    SC.visualizationSettings.openGL.lineWidth=2
    SC.visualizationSettings.view0.window.renderWindowSize = [1600,1080]
    
    
    nodeNumber = objFFRF['nGenericODE2'] #this is the node with the generalized coordinates

    SC.renderer.Start()              #start graphics visualization
    SC.renderer.SetModelView(zoom=2.011357,rotationVector=[-0.9999199,0.6224784,0.9588339],centerPoint=[0.7874073,1.334914,0])
    AnimateModes(SC, mbs, nodeNumber, period=0.1, showTime=False, renderWindowText='Hurty-Craig-Bampton: 2 x 6 static modes and 8 eigenmodes\n',
                 runOnStart=True)
    import sys
    sys.exit()

#+++++++++++++++++++++++++++++++++++++++++++++

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
    

