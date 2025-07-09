
.. _examples-ngsolvegeometry:

******************
NGsolveGeometry.py
******************

You can view and download this file on Github: `NGsolveGeometry.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/NGsolveGeometry.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example to show how to create CSG-geometry in Netgen and import as STL into exudyn
   #
   # Author:   Johannes Gerstmayr 
   # Date:     2021-04-20
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.FEM import *
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   import numpy as np
   import time
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #netgen/meshing part:
   
   #geometrical parameters:
   L = 0.2         #Length of body
   sy = 0.04       #width of body  (Y)
   sz = 0.03       #height of body (Z)
   dy = 0.005      #additional distance Y
   r = 0.015       #radius of bolt
   R = 0.025       #outer radius
   x = 0.002
   meshH = 0.005   #0.01 is default, 0.002 gives 100000 nodes and is fairly converged; 
   curvaturesafety = 5
   
   axis = 0.15
   rotBasis = np.eye(3 )
   
   #steel: (not needed for drawing ...)
   rho = 7850
   Emodulus=2.1e11
   nu=0.3
   
   #test high flexibility
   Emodulus=2e8
   # nModes = 32
   
   
   #helper function for cylinder with netgen
   def CSGcylinder(p0,p1,r):
       v = VSub(p1,p0)
       v = Normalize(v)
       cyl = Cylinder(Pnt(p0[0],p0[1],p0[2]), Pnt(p1[0],p1[1],p1[2]), 
                      r) * Plane(Pnt(p0[0],p0[1],p0[2]), Vec(-v[0],-v[1],-v[2])) * Plane(Pnt(p1[0],p1[1],p1[2]), Vec(v[0],v[1],v[2])) 
       return cyl
   
   meshCreated = False
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   import ngsolve as ngs
   import netgen
   from netgen.meshing import *
   
   from netgen.geom2d import unit_square
   #import netgen.libngpy as libng
   from netgen.csg import *
   
   
   showCase = 'revolute'
   showCase = 'spheric'
   
   if showCase == 'revolute':
       rotBasis = RotationMatrixX(-0.5*pi)
       #++++++++++++++++++++++++++++++++++++++++++++++++++++
       #first body
       geo0 = CSGeometry()
       
       #plate
       block = OrthoBrick(Pnt(0, x, -0.5*sz),Pnt(L, sy-x, 0.5*sz))
       blockCyl = CSGcylinder(p0=[0,0,0], p1=[0,sy,0], r=R)
       bolt0 = CSGcylinder(p0=[0,sy,0], p1=[0,2*sy+dy,0], r=r)
       geo0.Add(block+blockCyl+bolt0)
       
       mesh0 = ngs.Mesh( geo0.GenerateMesh(maxh=meshH, curvaturesafety=curvaturesafety))
       mesh0.Curve(1)
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++
       #second body
       geo1 = CSGeometry()
       
       #plate
       block = OrthoBrick(Pnt(0, x, -0.5*sz),Pnt(L, sy-x, 0.5*sz))
       blockCyl = CSGcylinder(p0=[0,0,0], p1=[0,sy,0], r=R)
       hole = CSGcylinder(p0=[0,-sy*0.1,0], p1=[0,1.1*sy,0], r=r)
       geo1.Add(block+blockCyl-hole)
       
       mesh1 = ngs.Mesh( geo1.GenerateMesh(maxh=meshH, curvaturesafety=curvaturesafety))
       mesh1.Curve(1)
   
       if True:
           import netgen.gui #this starts netgen gui; Press button "Visual" and activate "Auto-redraw after (sec)"; Then select "Mesh"
           
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #import body with fem (this could be simplified in future ...)
       #body0
       fem0 = FEMinterface()
       fem0.ImportMeshFromNGsolve(mesh0, density=rho, youngsModulus=Emodulus, poissonsRatio=nu)
       graphics0 = graphics.FromPointsAndTrigs(fem0.GetNodePositionsAsArray(), fem0.GetSurfaceTriangles(), 
                                                  color=graphics.color.dodgerblue, )
       graphics0 = AddEdgesAndSmoothenNormals(graphics0)
       
       mbs.CreateRigidBody(referencePosition=[0,-sy*2-dy,0],
                           referenceRotationMatrix=RotationMatrixX(0),
                           inertia=InertiaCuboid(1000, [1,1,1]),
                           graphicsDataList=[graphics0])
       
       
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #body1
       fem1 = FEMinterface()
       fem1.ImportMeshFromNGsolve(mesh1, density=rho, youngsModulus=Emodulus, poissonsRatio=nu)
       graphics1 = graphics.FromPointsAndTrigs(fem1.GetNodePositionsAsArray(), fem1.GetSurfaceTriangles(), 
                                                  color=graphics.color.lightred)
       graphics1 = AddEdgesAndSmoothenNormals(graphics1)
       
       mbs.CreateRigidBody(referencePosition=[0,-sy,0],
                           referenceRotationMatrix=RotationMatrixY(1.2*pi),
                           inertia=InertiaCuboid(1000, [1,1,1]),
                           graphicsDataList=[graphics1])
   
   if showCase == 'spheric':
       #++++++++++++++++++++++++++++++++++++++++++++++++++++
       #first body
       geo0 = CSGeometry()
       meshH *= 0.5
       
       block = OrthoBrick(Pnt(0, -0.4*sy, -0.4*sz),Pnt(L, 0.4*sy, 0.4*sz))
       sphere = Sphere( Pnt(0, 0, 0), R*1.2)
       cutBlock = OrthoBrick(Pnt(-1, -sy, -sz),Pnt(-0.3*R, sy, sz))
       cutSphere = Sphere( Pnt(0, 0, 0), R)
       geo0.Add(block+sphere-cutBlock-cutSphere)
       
       mesh0 = ngs.Mesh( geo0.GenerateMesh(maxh=meshH, curvaturesafety=curvaturesafety))
       mesh0.Curve(1)
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++
       #second body
       geo1 = CSGeometry()
       
       #plate
       block = OrthoBrick(Pnt(0, -0.4*sz, -0.4*sz),Pnt(L, 0.4*sz, 0.4*sz))
       sphere = Sphere( Pnt(0, 0, 0), R)
       geo1.Add(block+sphere)
       
       mesh1 = ngs.Mesh( geo1.GenerateMesh(maxh=meshH, curvaturesafety=curvaturesafety))
       mesh1.Curve(1)
       
       if True:
           import netgen.gui #this starts netgen gui; Press button "Visual" and activate "Auto-redraw after (sec)"; Then select "Mesh"
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #import body with fem (this could be simplified in future ...)
       #body0
       fem0 = FEMinterface()
       fem0.ImportMeshFromNGsolve(mesh0, density=rho, youngsModulus=Emodulus, poissonsRatio=nu)
       graphics0 = graphics.FromPointsAndTrigs(fem0.GetNodePositionsAsArray(), fem0.GetSurfaceTriangles(), 
                                                  color=graphics.color.dodgerblue, )
       graphics0 = AddEdgesAndSmoothenNormals(graphics0)
       
       mbs.CreateRigidBody(referencePosition=[0,-sy,0],
                           referenceRotationMatrix=RotationMatrixX(0),
                           inertia=InertiaCuboid(1000, [1,1,1]),
                           graphicsDataList=[graphics0])
       
       
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #body1
       fem1 = FEMinterface()
       fem1.ImportMeshFromNGsolve(mesh1, density=rho, youngsModulus=Emodulus, poissonsRatio=nu)
       graphics1 = graphics.FromPointsAndTrigs(fem1.GetNodePositionsAsArray(), fem1.GetSurfaceTriangles(), 
                                                  color=graphics.color.lightred)
       graphics1 = AddEdgesAndSmoothenNormals(graphics1)
       
       mbs.CreateRigidBody(referencePosition=[0,-sy,0],
                           referenceRotationMatrix=RotationMatrixZ(-0.12*pi)@RotationMatrixY(1.15*pi),
                           inertia=InertiaCuboid(1000, [1,1,1]),
                           graphicsDataList=[graphics1])
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++
   #world basis
   gl=[]
   gl += [graphics.Text(point=rotBasis@[axis,0,0],text='X')]
   gl += [graphics.Text(point=rotBasis@[0,axis,0],text='Y')]
   gl += [graphics.Text(point=rotBasis@[0,0,axis],text='Z')]
   
   gl += [graphics.Basis(origin=[0,0,0], rotationMatrix=rotBasis, length=axis)]
   
   mbs.CreateGround(graphicsDataList=gl)
   
   mbs.Assemble()
   #%%++++++++++++++++++++++++++++++
   SC.visualizationSettings.openGL.polygonOffset = 0.1 #to draw edges clearly
   SC.visualizationSettings.openGL.lineWidth = 2
   SC.visualizationSettings.openGL.multiSampling = 4
   # SC.visualizationSettings.general.drawWorldBasis = True
   # SC.visualizationSettings.general.worldBasisSize = axis
   SC.visualizationSettings.general.drawCoordinateSystem = False
   SC.visualizationSettings.general.textSize = 16
   SC.visualizationSettings.window.renderWindowSize = [1600,1200]
   
   mbs.SolveDynamic()
   
   mbs.SolutionViewer()
   
   
   
   
   
   
   
   
   


