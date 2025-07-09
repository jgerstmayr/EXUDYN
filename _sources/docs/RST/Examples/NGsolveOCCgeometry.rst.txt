
.. _examples-ngsolveoccgeometry:

*********************
NGsolveOCCgeometry.py
*********************

You can view and download this file on Github: `NGsolveOCCgeometry.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/NGsolveOCCgeometry.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for Hurty-Craig-Bampton modes using a simple flexible pendulum meshed with Netgen
   #
   # Author:   Johannes Gerstmayr 
   # Date:     2021-04-20
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   import exudyn as exu
   from exudyn.utilities import *
   import exudyn.graphics as graphics
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   import numpy as np
   
   import time
   
   from netgen.occ import *
   from ngsolve import Mesh, Draw
   
   coarsefact = 1
   #simple solid of revolution geometry:
   wp = WorkPlane(Axes(p=(0,0,0), n=Z, h=X))
   #wp.MoveTo(10,0).Line(18).Arc(5,180).Line(10).Close()
   wp.MoveTo(10,0).Line(18).Arc(2,90).Line(6).Arc(2,90).Line(18).Rotate(90).Line(2).Rotate(90).Line(10).Arc(3,-180).Line(10).Close()
   axis = Axis((0,0,0),Y)
   body = wp.Face().Revolve(axis,360)
   body.name='steel'
   body.faces.Min(Y).name='my_bc3'
   
   body2 = Cylinder(Pnt(15,0,0),-Y,4,5)
   body2.faces.Max(X).name='my_bc0'
   body2.faces.Min(X).name='my_bc1'
   body2.faces.Min(Y).name='my_bc2'
   #body2.faces[0].name='my_bc'
   body2.name='wood'
   
   #geo = OCCGeometry( p )
   maxh = 2.5*coarsefact #*0.125
   glued = Glue([body, body2]) #to combine different materials, use Glue
   
   geo = OCCGeometry(glued)
   print('meshing ...')
   geoMesh = geo.GenerateMesh(maxh=maxh, 
                                curvaturesafety=0.5/coarsefact,#*10, 
                                #segmentsperedge=12,
                                )
   
   mesh = Mesh(geoMesh)
   
   
   print('mats=',mesh.GetMaterials()) #shows materials
   print('bcs=',mesh.GetBoundaries()) #shows bc
   
   if True:
       import netgen.gui #this starts netgen gui; Press button "Visual" and activate "Auto-redraw after (sec)"; Then select "Mesh"
   
   
   [points, triangles, normals] = graphics.NGsolveMesh2PointsAndTrigs(mesh=mesh, 
                                                                      scale=0.01,
                                                                      meshOrder=2,
                                                                      addNormals=True,
                                                                      )
   color = color4steelblue
   meshColor=graphics.color.lawngreen[0:3]+[graphics.material.indexChrome]
                                        
   gMesh = graphics.FromPointsAndTrigs(points, triangles, normals=normals,
                                       color=meshColor)
   
   
   if False:
       #optionally save STL mesh from netgen:
       mesh.ngmesh.Export('solution/ngsolveOCCtest.stl','STL Format')
   
   if True:
       #save mesh via Exudyn and graphicsData
       graphics.ExportSTL(gMesh, 'solution/ngsolveOCCtest2.stl', invertNormals=False, invertTriangles=False)
   
       gMesh = graphics.FromSTLfileASCII('solution/ngsolveOCCtest2.stl', 
                                         invertNormals=False, invertTriangles=False,
                                         color=meshColor)
       gMesh = graphics.AddEdgesAndSmoothenNormals(gMesh, edgeAngle=0.35*pi)
   
   gFloor = graphics.CheckerBoard(point=[0,0,-0.5],size=2)
   mbs.CreateGround(graphicsDataList=[gMesh,
                                      gFloor])
       
   mbs.CreateMassPoint(physicsMass=1, show=False)
   
   mbs.Assemble()
   
   SC.visualizationSettings.window.renderWindowSize=[1200,800]
   SC.visualizationSettings.general.autoFitScene=False
   
   SC.visualizationSettings.general.drawCoordinateSystem = False
   SC.visualizationSettings.general.showSolverInformation = False
   SC.visualizationSettings.openGL.multiSampling=1
   SC.visualizationSettings.openGL.shadow = 0.2
   SC.visualizationSettings.openGL.lineWidth = 2
   #SC.visualizationSettings.openGL.light0position = [-2.0, 4.0, 1.0, 1.0]
   SC.visualizationSettings.loads.show = False
   SC.visualizationSettings.openGL.light0position=[2,-0.25,0.25,1]
   
   #raytracing options
   SC.visualizationSettings.exportImages.saveImageTimeOut = 200000
   SC.visualizationSettings.openGL.multiSampling = 1
   SC.visualizationSettings.openGL.enableLight1 = False
   SC.visualizationSettings.raytracer.numberOfThreads = 16 #number of threads!
   SC.visualizationSettings.raytracer.enable = False #set True for raytracing
   SC.visualizationSettings.raytracer.ambientLightColor = [0.5,0.5,0.5,1]
   SC.visualizationSettings.raytracer.backgroundColorReflections = [0.3,0.3,0.3,1]
   SC.visualizationSettings.raytracer.keepWindowActive= True
   SC.visualizationSettings.raytracer.searchTreeFactor = 8
   SC.visualizationSettings.raytracer.imageSizeFactor=2 #for faster rendering
   
   
   #visualize in Exudyn:
   SC.renderer.Start()              #start graphics visualization
   if 'renderState' in exu.sys: #reload last view
       SC.renderer.SetState(exu.sys['renderState'])
   
   #to run Exudyn and netgen in parallel (not recommended), we need to run an event loop
   while SC.renderer.IsActive():
       SC.renderer.DoIdleTasks(0) #press space to continue
       time.sleep(0.04)
       netgen.Redraw()
   
   SC.renderer.Stop() #safely close rendering window!
       
   


