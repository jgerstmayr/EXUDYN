
.. _examples-ngsolvepistonengine:

**********************
NGsolvePistonEngine.py
**********************

You can view and download this file on Github: `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  generate a piston engine with finite element mesh 
   #           created with NGsolve and with variable number of pistons
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-06-12
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   import sys
   import exudyn as exu
   
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.rigidBodyUtilities import *
   from exudyn.FEM import *
   
   import time
   
   try: #if installed, may help to improve performance (depending on solver)
       import mkl
       mklThreads = 8
       mkl.set_num_threads(mklThreads)
       exu.Print('using',mklThreads,'mkl threads ...')
   except: pass
   
   from ngsolve import *
   from netgen.geom2d import unit_square
   
   #import netgen.libngpy as libng
   
   netgenDrawing = False #set true, to show geometry and mesh in NETGEN
   #if netgenDrawing, uncomment the following line and execute in external terminal, not in spyder (see preferences "Run"):
   #import netgen.gui
   
   from netgen.csg import *
   
   import numpy as np
   import timeit
   
   verbose = True
   meshSize = 0.005*4 #fast: 0.005*4 with order 1; standard:0.005; h=0.004 with meshOrder 2 gives 258k unknowns; fine: 0.0011: memory limit (96GB) for NGsolve; < 0.0015 makes problems with scipy eigensolver
   meshOrder = 1 #2 for stresses!
   showStresses = True #may take very long for large number of modes/nodes
   saveMesh = False
   
   #++++++++++++++++++++++++++++++++++++
   #helper functions (copied from EXUDYN):
   def RotationMatrixZ(angleRad):
       return np.array([ [np.cos(angleRad),-np.sin(angleRad), 0],
                         [np.sin(angleRad), np.cos(angleRad), 0],
                         [0,        0,        1] ]);
       
   def VAdd(v0, v1):
       if len(v0) != len(v1): print("ERROR in VAdd: incompatible vectors!")
       n = len(v0)
       v = [0]*n
       for i in range(n):
           v[i] = v0[i]+v1[i]
       return v
   
   def VSub(v0, v1):
       if len(v0) != len(v1): print("ERROR in VSub: incompatible vectors!")
       n = len(v0)
       v = [0]*n
       for i in range(n):
           v[i] = v0[i]-v1[i]
       return v
   
   def NormL2(vector):
       value = 0
       for x in vector:
           value += x**2
       return value**0.5
   
   def Normalize(v):
       v2=[0]*len(v)
   
       fact = NormL2(v)
       fact = 1./fact
       for i in range(len(v2)): 
           v2[i]=fact*v[i]
       return v2
   #++++++++++++++++++++++++++++++++++++
   startTotal = timeit.default_timer()
   #parameters
   
   #crank:
   b1 = 0.012 #width of journal bearing
   r1 = 0.012 #radius of journal bearing
   dk = 0.015 #crank arm width (z)
   bk = 0.032 #crank arm size (y)
   
   l3 = 0.030
   l4 = 0.040
   #l4x= 0.005 #offset of counterweight
   lk = 0.030 #l4*0.5+l3 #crank arm length (x)
   bm = 0.065
   dBevel = dk*0.5
   #shaft:
   r0 = 0.012 #0.012
   d0 = 0.020 #shaft length at left/right support
   d1 = 0.012 #shaft length at intermediate support
   
   #distance rings:
   db = 0.002          #width of distance ring
   rdb0 = r0+db        #total radius of distance ring, shaft
   rdb1 = r1+db        #total radius of distance ring, crank
   
   #conrod:
   bc = 0.024      #height of conrod
   dc = 0.012      #width of conrod
   lc = 0.080      #length of conrod (axis-axis)
   r1o= r1+0.006   #outer radius of conrod at crank joint
   r2 = 0.008      #radius of piston journal bearing
   r2o= r2+0.006   #outer radius of conrod at piston joint
   
   cylOffZ=0.010  #z-offset of cylinder cut out of conrod
   cylR = 0.008    #radius of cylinder cut out of conrod
   
   angC = 4*np.pi/180
   
   #piston:
   dpb = r2o-0.000   #axis inside piston
   r2p = r2o+0.004   #0.018
   lp = 0.034
   bp = 0.050
   lpAxis = dc+2*db
   lOffCut = 0.011 #offset for cutout of big cylinder
   
   #total length of one segment:
   lTotal = db+dk+db+b1+db+dk+db+d1
   
   #eps
   eps = 5e-4 #added to faces, to avoid CSG-problems
   
   #++++++++++++++++++++++++++++++++++++
   #points
   pLB = [0 ,0,-d0]
   p0B = [0 ,0,0]
   p1B = [0 ,0,db]
   #p2B = [0, 0,db+dk]
   p21B =[lk,0,db+dk]
   p31B = [lk,0,db+dk+db]
   p41B = [lk,0,db+dk+db+b1]
   p51B =[lk,0,db+dk+db+b1+db]
   p6B = [0 ,0,db+dk+db+b1+db+dk]
   p7B = [0 ,0,db+dk+db+b1+db+dk+db]
   p8B = [0 ,0,lTotal]
   
   def CSGcylinder(p0,p1,r):
       v = VSub(p1,p0)
       v = Normalize(v)
       cyl = Cylinder(Pnt(p0[0],p0[1],p0[2]), Pnt(p1[0],p1[1],p1[2]), 
                      r) * Plane(Pnt(p0[0],p0[1],p0[2]), Vec(-v[0],-v[1],-v[2])) * Plane(Pnt(p1[0],p1[1],p1[2]), Vec(v[0],v[1],v[2])) 
       return cyl
   
   def CSGcube(pCenter,size):
       s2 = [0.5*size[0],0.5*size[1],0.5*size[2]]
       p0 = VSub(pCenter,s2)
       p1 = VAdd(pCenter,s2)
       brick = OrthoBrick(Pnt(p0[0],p0[1],p0[2]),Pnt(p1[0],p1[1],p1[2]))
       return brick
   
   
   #transform points
   def TransformCrank(p, zOff, zRot):
       p2 = RotationMatrixZ(zRot) @ p
       pOff=[0,0,zOff]
       return VAdd(p2,pOff)
   
   #cube only in XY-plane, z infinite
   def CSGcubeXY(pCenter,sizeX,sizeY,ex,ey):
       #print("pCenter=",pCenter)
       pl1 = Plane(Pnt(pCenter[0]-0.5*sizeX*ex[0],pCenter[1]-0.5*sizeX*ex[1],0),Vec(-ex[0],-ex[1],-ex[2]))
       pl2 = Plane(Pnt(pCenter[0]+0.5*sizeX*ex[0],pCenter[1]+0.5*sizeX*ex[1],0),Vec( ex[0], ex[1], ex[2]))
   
       pl3 = Plane(Pnt(pCenter[0]-0.5*sizeY*ey[0],pCenter[1]-0.5*sizeY*ey[1],0),Vec(-ey[0],-ey[1],-ey[2]))
       pl4 = Plane(Pnt(pCenter[0]+0.5*sizeY*ey[0],pCenter[1]+0.5*sizeY*ey[1],0),Vec( ey[0], ey[1], ey[2]))
   
       return pl1*pl2*pl3*pl4
       
   
   #create one crank face at certain z-offset and rotation; side=1: left, side=-1: right
   def GetCrankFace(zOff, zRot, side=1):
       ex = RotationMatrixZ(zRot) @ [1,0,0]
       ey = RotationMatrixZ(zRot) @ [0,1,0]
       #print("zOff=",zOff, "zRot=", zRot, "side=", side,"ex=", ex)
       pLeft = [0,0,zOff]
       pRight = [0,0,zOff+dk]
       pMid = [0,0,zOff+0.5*dk]
   
       pcLeft=VAdd(pLeft,lk*ex)
       pcRight=VAdd(pRight,lk*ex)
       f=0.5**0.5
       cyl1pl = Plane(Pnt(pcLeft[0],pcLeft[1],pcLeft[2]+0.5*dk-side*dk),Vec(f*ex[0],f*ex[1],f*ex[2]-side*f))        
       cyl1 = Cylinder(Pnt(pcLeft[0],pcLeft[1],pcLeft[2]-1), Pnt(pcRight[0],pcRight[1],pcRight[2]+1), 0.5*bk)*cyl1pl
   
       #cone2 = Cylinder(Pnt(pcLeft[0],pcLeft[1],pcLeft[2]-1), Pnt(pcRight[0],pcRight[1],pcRight[2]+1), lk+l4)
       cone2 = Cone(Pnt(pcLeft[0],pcLeft[1],pcLeft[2]-side*dBevel+0.5*dk), Pnt(pcLeft[0],pcLeft[1],pcLeft[2]+side*dBevel+0.5*dk), lk+l4-1.5*dBevel, lk+l4-0.5*dBevel)
       cube1 = CSGcubeXY(VAdd(pMid,0.49*l3*ex),1.02*l3,bk,ex,ey) #make l3 a little longer, to avoid bad edges
       cube2 = CSGcubeXY(VAdd(pMid,-0.5*l4*ex),1.0*l4,bm,ex,ey)*cone2
   
       pc3a = VAdd(pLeft,0.*l3*ex+(0.5*bk+0.4*l3)*ey)
       cyl3a = Cylinder(Pnt(pc3a[0],pc3a[1],pc3a[2]-1), Pnt(pc3a[0],pc3a[1],pc3a[2]+1), 0.42*l3)
       pc3b = VAdd(pLeft,0.*l3*ex+(-0.5*bk-0.4*l3)*ey)
       cyl3b = Cylinder(Pnt(pc3b[0],pc3b[1],pc3b[2]-1), Pnt(pc3b[0],pc3b[1],pc3b[2]+1), 0.42*l3)
       #cube3a = (CSGcubeXY(VAdd(pMid,0.26*l3*ex+(0.5*bk+0.26*l3)*ey),0.5*l3,0.5*l3,ex,ey)-cyl3a)
       
       return ((cube1+cube2+cyl1)-(cyl3a+cyl3b))*Plane(Pnt(0,0,pLeft[2]),Vec(0,0,-1))*Plane(Pnt(0,0,pRight[2]),Vec(0,0,1))
       #return (cube1+cube2+cyl1)*Plane(Pnt(0,0,pLeft[2]),Vec(0,0,-1))*Plane(Pnt(0,0,pRight[2]),Vec(0,0,1))
   
   #generate one crank, rotated around z-axis in radiant
   def GenerateCrank(zOff, zRot):
       pL = TransformCrank(pLB,zOff, zRot)
       p0 = TransformCrank(p0B,zOff, zRot)
       p1 = TransformCrank(p1B,zOff, zRot)
   
       p21 = TransformCrank(p21B,zOff, zRot)
       p31 = TransformCrank(p31B,zOff, zRot)
       p41 = TransformCrank(p41B,zOff, zRot)
       p51 = TransformCrank(p51B,zOff, zRot)
   
       p6 = TransformCrank(p6B,zOff, zRot)
       p7 = TransformCrank(p7B,zOff, zRot)
       p8 = TransformCrank(p8B,zOff, zRot)
       
       crank0 = CSGcylinder(pL,[p0[0],p0[1],p0[2]+eps],r0)
       crank1 = CSGcylinder(p0,[p1[0],p1[1],p1[2]+eps],rdb0)
   
       #conrod bearing:
       crank3 = CSGcylinder([p21[0],p21[1],p21[2]-eps],p31,rdb1)
       crank7 = CSGcylinder(p31,p41,r1)
       crank8 = CSGcylinder(p41,[p51[0],p51[1],p51[2]+eps],rdb1)
       
       crank9 = CSGcylinder([p6[0],p6[1],p6[2]-eps],p7,rdb0)
       crank10 = CSGcylinder([p7[0],p7[1],p7[2]-eps],p8,r0)
   
       #return crank0+crank1+crank3+crank4+crank5+crank6+crank7+crank8+crank4b+crank5b+crank6b+crank9+crank10
       if zOff==0:#add first shaft
           crank1 = crank1+crank0
       return crank1+GetCrankFace(db+zOff,zRot,1)+crank3+crank7+crank8+GetCrankFace(db+2*db+dk+b1+zOff,zRot,-1)+crank10+crank9
   
   
   geoCrank = CSGeometry()
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #choose configuration for crankshaft:
   #crankConfig = [0] #1-piston
   #crankConfig = [np.pi/2] #1-piston
   #crankConfig = [0,np.pi] #2-piston
   #crankConfig = [0,np.pi*2./3.,2.*np.pi*2./3.] #3-piston
   #crankConfig = [0,np.pi,np.pi,0] #4-piston
   crankConfig = [0,np.pi*2./3.,2.*np.pi*2./3.,2.*np.pi*2./3.,np.pi*2./3.,0] #6-piston
   #crankConfig = crankConfig*2 #12-piston
   
   nPistons = len(crankConfig)
   
   crank = GenerateCrank(0, crankConfig[0])
   zPos = lTotal
   for i in range(len(crankConfig)-1):
       angle = crankConfig[i+1]
       crank += GenerateCrank(zPos, angle)
       zPos += lTotal
   
   # crank = (GenerateCrank(0, 0) + GenerateCrank(lTotal, np.pi*2./3.) + GenerateCrank(2*lTotal, np.pi*2.*2./3.)+
   #           GenerateCrank(3*lTotal, np.pi*2.*2./3.) + GenerateCrank(4*lTotal, np.pi*2./3.))
   
   geoCrank.Add(crank)
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #conrod model:
   def GenerateConrod(zOff):
       ey0 = [0,1,0] #top/bottom face vector of conrod
       ey1 = [0,-1,0]
   
       ex0 = [1,0,0] #top/bottom face vector of conrod
       ex1 = [1,0,0]
       
       ey0 = RotationMatrixZ(-angC)@ey0
       ey1 = RotationMatrixZ(angC)@ey1
       ex0 = RotationMatrixZ(-angC)@ex0
       ex1 = RotationMatrixZ(angC)@ex1
   
   
       pl1 = Plane(Pnt(0, 0.5*bc,0),Vec(ey0[0],ey0[1],ey0[2]))
       pl2 = Plane(Pnt(0,-0.5*bc,0),Vec(ey1[0],ey1[1],ey1[2]))
   
       pl3 = Plane(Pnt(-0.5*lc,0,0),Vec(-1,0,0))
       pl4 = Plane(Pnt( 0.5*lc,0,0),Vec( 1,0,0))
   
       pl5 = Plane(Pnt( 0,0,-0.5*dc+zOff),Vec( 0,0,-1))
       pl6 = Plane(Pnt( 0,0, 0.5*dc+zOff),Vec( 0,0, 1))
   
       
       cylC1 = Cylinder(Pnt(-0.5*lc,0,-1), Pnt(-0.5*lc,0,1), r1)
       #cylC1o = Cylinder(Pnt(-0.5*lc,0,-1), Pnt(-0.5*lc,0,1), r1o)
       cylC1o = Sphere(Pnt(-0.5*lc,0,zOff), r1o) #in fact is a sphere
   
       cylC2 = Cylinder(Pnt( 0.5*lc,0,-1), Pnt( 0.5*lc,0,1), r2)
       #cylC2o = Cylinder(Pnt(0.5*lc,0,-1), Pnt( 0.5*lc,0,1), r2o)
       cylC2o = Sphere(Pnt(0.5*lc,0,zOff), r2o) #in fact is a sphere
   
       cylSideA = (Cylinder(Pnt(-0.5*lc+r1o,0,cylOffZ+zOff), Pnt(0.5*lc-r2o,0,cylOffZ+zOff), cylR)*
                   Plane(Pnt(-0.5*lc+r1o-0.002,0,0),Vec(-1,0,0))*
                   Plane(Pnt( 0.5*lc-r2o+0.002,0,0),Vec( 1,0,0)))
   
       cylSideB = (Cylinder(Pnt(-0.5*lc+r1o,0,-cylOffZ+zOff), Pnt(0.5*lc-r2o,0,-cylOffZ+zOff), cylR)*
                   Plane(Pnt(-0.5*lc+r1o-0.002,0,0),Vec(-1,0,0))*
                   Plane(Pnt( 0.5*lc-r2o+0.002,0,0),Vec( 1,0,0)))
   
   
       return ((pl1*pl2*pl3*pl4+cylC1o+cylC2o)-cylC1-cylC2)*pl5*pl6-cylSideA-cylSideB
       #return pl1*pl2*pl3*pl4*pl5*pl6
   
   geoConrod = CSGeometry()
   conrod = GenerateConrod(0)#db+dk+db+0.5*b1
   geoConrod.Add(conrod)
   
   # if netgenDrawing: 
   #     Draw(geoCrank)
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #conrod model:
   def GeneratePiston(zOff):
       p0 = [-dpb,0,zOff]
       p1 = [-dpb+lp,0,zOff]
       cylPo   = CSGcylinder(p0, p1, 0.5*bp) #piston outside
       cylPaxis= CSGcylinder([0,0,-0.5*lpAxis-eps+zOff],     [0,0, 0.5*lpAxis+eps+zOff], r2) #piston axis
       cylPaxis0= CSGcylinder([0,0,-0.5*lpAxis-eps+zOff],    [0,0,-0.5*lpAxis+db+zOff], r2+db) #piston axis
       cylPaxis1= CSGcylinder([0,0, 0.5*lpAxis-db+zOff], [0,0, 0.5*lpAxis+eps+zOff], r2+db) #piston axis
       cylPin  = CSGcylinder([0,0,-0.5*lpAxis+zOff], [0,0, 0.5*lpAxis+zOff], r2p) #piston inner cutout
   
       #box = CSGcube([0,0,zOff], [dpb+r2p,2*(r2p),lpAxis])
       box = CSGcube([-0.5*dpb,0,zOff], [dpb,2*(r2p)-0.002,lpAxis-0.000])
   
       cylCut  = CSGcylinder([-(l4+l3+lOffCut),0,-bp+zOff], [-(l4+l3+lOffCut),0, bp+zOff], l4+l3) #piston inner cutout
   
       return (cylPo-box-cylCut-cylPin)+cylPaxis+cylPaxis0+cylPaxis1
   
   geoPiston = CSGeometry()
   piston = GeneratePiston(0)#db+dk+db+0.5*b1
   geoPiston.Add(piston)
   
   if verbose: print("Generate meshes ...")
   #do meshing, if geometry is successful
   if True:
       ngMeshCrank = geoCrank.GenerateMesh(maxh=meshSize)
       meshCrank = Mesh(ngMeshCrank)
       meshCrank.Curve(1)
       if netgenDrawing: 
           Draw(meshCrank)
       
       if False:
           #save mesh to file:
           meshCrank.ngmesh.Export('testData/crankshaft.mesh','Neutral Format')
   
   if True:
       ngMeshConrod = geoConrod.GenerateMesh(maxh=meshSize) #in videos 0.003
       meshConrod = Mesh(ngMeshConrod)
       meshConrod.Curve(1)
       if netgenDrawing: 
           Draw(meshConrod)
       if False:
           meshConrod.ngmesh.Export('testData/conrod.mesh','Neutral Format')
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   if True:
       ngMeshPiston = geoPiston.GenerateMesh(maxh=meshSize+0.001*0)
       meshPiston = Mesh(ngMeshPiston)
       meshPiston.Curve(1)
       if netgenDrawing: 
           Draw(meshPiston)
       if False:
           meshPiston.ngmesh.Export('testData/piston.mesh','Neutral Format')
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #here starts the EXUDYN part
   if True:
       SC = exu.SystemContainer()
       mbs = SC.AddSystem()
   
       #crankshaft and piston mechanical parameters:
       density = 7850
       youngsModulus = 2.1e11 *1e-1
       poissonsRatio = 0.3
       fRotorStart = 20 #initial revolutions per second, only crankshaft
   
       totalFEcoordinates = 0 #accumulated FE-mesh coordinates
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #import crankshaft mesh into EXUDYN FEMinterface
       femCrank = FEMinterface()
       eigenModesNGsolve=True
       nModes=8
   
       [bfM, bfK, fes] = femCrank.ImportMeshFromNGsolve(meshCrank, density, youngsModulus, poissonsRatio, 
                                                        verbose = True, meshOrder = meshOrder)
                             # computeEigenmodes=eigenModesNGsolve, excludeRigidBodyModes = 6,
                             # numberOfModes = nModes, maxEigensolveIterations=20)
   
       nModes = 20
       excludeRigidBodyModes = 6
       if verbose: print("number of coordinates crank =", femCrank.NumberOfCoordinates())
       if verbose: print("Compute eigenmodes crank ....")
   
       if not eigenModesNGsolve:
           startCrank = timeit.default_timer()
           femCrank.ComputeEigenmodes(nModes, excludeRigidBodyModes = excludeRigidBodyModes, useSparseSolver = True)
           stopCrank = timeit.default_timer()
           print("\ncrank eigen analysis time=", stopCrank-startCrank)
       else:
           start_time = time.time()
           femCrank.ComputeEigenmodesNGsolve(bfM, bfK, nModes=nModes, 
                                             excludeRigidBodyModes=excludeRigidBodyModes,  maxEigensolveIterations=20)
           print("NGsolve mode computation needed %.3f seconds" % (time.time() - start_time))
       
       totalFEcoordinates+=femCrank.NumberOfCoordinates()
       print("eigen freq. crank=", femCrank.GetEigenFrequenciesHz()[0:nModes])
   
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #compute stress modes:
       SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Displacement
       mat = KirchhoffMaterial(youngsModulus, poissonsRatio, density)
       varType = exu.OutputVariableType.DisplacementLocal
       #varType = exu.OutputVariableType.StrainLocal
       if showStresses:
           print("ComputePostProcessingModes femCrank ... ")
           start_time = time.time()
           varType = exu.OutputVariableType.StressLocal
           femCrank.ComputePostProcessingModesNGsolve(fes, material=mat, 
                                          outputVariableType=varType)
           print("--- %s seconds ---" % (time.time() - start_time))
       
       SC.visualizationSettings.contour.outputVariable = varType
       
       #print("Create CMS object and matrices ....")
       cmsCrank = ObjectFFRFreducedOrderInterface(femCrank)
       
       objFFRFcrank = cmsCrank.AddObjectFFRFreducedOrderWithUserFunctions(exu, mbs, 
                                                   positionRef=[0,0,0], 
                                                   eulerParametersRef=eulerParameters0, 
                                                   initialVelocity=[0,0,0], initialAngularVelocity=[0,0,1*fRotorStart*2*pi],
                                                   gravity = [0,-0*9.81,0],
                                                   color=[0.1,0.9,0.1,1.])
       mbs.SetObjectParameter(objFFRFcrank['oFFRFreducedOrder'],'VshowNodes',False)
   
   
       if False:#animate eigenmodes of crankshaft
           from exudyn.interactive import AnimateModes
           mbs.Assemble()
   
           SC.visualizationSettings.general.textSize = 16 #30 for cover figure
           SC.visualizationSettings.general.useGradientBackground = True
           SC.visualizationSettings.openGL.lineWidth = 2
           SC.visualizationSettings.openGL.showFaceEdges = True
           SC.visualizationSettings.openGL.showFaces = True
           SC.visualizationSettings.openGL.multiSampling = 4
           SC.visualizationSettings.nodes.show = False
           SC.visualizationSettings.window.renderWindowSize = [1600,1080]
   
           SC.visualizationSettings.contour.outputVariableComponent = 0
   
           SC.visualizationSettings.general.autoFitScene=False
   
           AnimateModes(SC, mbs, 1, period=0.2)
           exit()
   
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #import conrod and piston mesh into EXUDYN FEMinterface and compute eigenmodes
       nModes = 8
       excludeRigidBodyModes = 6
       femConrod = FEMinterface()
       # femConrod.ImportMeshFromNGsolve(meshConrod, density, youngsModulus, poissonsRatio, verbose = False)
       [bfM, bfK, fes] = femConrod.ImportMeshFromNGsolve(meshConrod, density, youngsModulus, poissonsRatio, 
                                                         verbose = False, meshOrder = meshOrder)
                             # computeEigenmodes=eigenModesNGsolve, excludeRigidBodyModes = 6,
                             # numberOfModes = nModes, maxEigensolveIterations=20)
       if verbose: print("number of coordinates conrod =", femConrod.NumberOfCoordinates())
       if verbose: print("Compute eigenmodes conrod ....")
   
       if not eigenModesNGsolve:
           femConrod.ComputeEigenmodes(nModes, excludeRigidBodyModes = excludeRigidBodyModes, useSparseSolver = True)
       else:
           femConrod.ComputeEigenmodesNGsolve(bfM, bfK, nModes=nModes, excludeRigidBodyModes=excludeRigidBodyModes)
   
       totalFEcoordinates+=femConrod.NumberOfCoordinates()
       if verbose: print("eigen freq. conrod=", femConrod.GetEigenFrequenciesHz()[0:nModes])
   
       if showStresses:
           print("ComputePostProcessingModes femConrod ... ")
           start_time = time.time()
           femConrod.ComputePostProcessingModesNGsolve(fes, material=mat, 
                                          outputVariableType=varType)
           print("--- %s seconds ---" % (time.time() - start_time))
   
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #import piston mesh into EXUDYN FEMinterface
       femPiston = FEMinterface()
       #femPiston.ImportMeshFromNGsolve(meshPiston, density, youngsModulus, poissonsRatio, verbose = False)
       [bfM, bfK, fes] = femPiston.ImportMeshFromNGsolve(meshPiston, density, youngsModulus, poissonsRatio, verbose = False, meshOrder = meshOrder)
       
       if verbose: print("number of coordinates piston =", femPiston.NumberOfCoordinates())
       if verbose: print("Compute eigenmodes piston ....")
   
       if not eigenModesNGsolve:
           femPiston.ComputeEigenmodes(nModes, excludeRigidBodyModes = excludeRigidBodyModes, useSparseSolver = True)
       else:
           femPiston.ComputeEigenmodesNGsolve(bfM, bfK, nModes=nModes, excludeRigidBodyModes=excludeRigidBodyModes)
   
       totalFEcoordinates+=femPiston.NumberOfCoordinates()
       if verbose: print("eigen freq. Piston=", femPiston.GetEigenFrequenciesHz()[0:nModes])
   
       if showStresses:
           print("ComputePostProcessingModes femPiston ... ")
           start_time = time.time()
           femPiston.ComputePostProcessingModesNGsolve(fes, material=mat, 
                                          outputVariableType=varType)
           print("--- %s seconds ---" % (time.time() - start_time))
   
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #import multiple conrods and pistons
   
       conrodsRotList = []
       conrodsPosList = []
       pistonsRotList = []
       pistonsPosList = []
   
       objFFRFconrodList=[]
       cmsConrodList=[]
       objFFRFpistonList=[]
       cmsPistonList=[]
       pkList = []
       pcList = []
       ppList = []
       zOffsetList = []
       for iCrank in range(len(crankConfig)):
           zOffset = db+dk+db + lTotal*iCrank #left end of conrod, for multiple conrods in a loop
           zOffsetList.append(zOffset)
           #compute crank (pK), conrod (pC) and piston position (pP) for any crank angle:
           phi = crankConfig[iCrank]
           pK = np.array([lk*np.cos(phi),lk*np.sin(phi),0])
           alpha=np.arcsin(pK[1]/lc)
           pC = pK + np.array([0.5*lc*np.cos(alpha),-0.5*lc*np.sin(alpha),0])
           pP = pK + np.array([lc*np.cos(alpha),-lc*np.sin(alpha),0])
           pkList.append(pK)
           pcList.append(pC)
           ppList.append(pP)
           #print("pK=",pK)
           #print("pC=",pC)
           #print("pP=",pP)
           
           eulerParametersInit = RotationMatrix2EulerParameters(RotationMatrixZ(-alpha))
           #pRef = [lk+0.5*lc,0,zOffset+0.5*b1] #0-degree
           pRef = pC + [0,0,zOffset+0.5*b1]
           conrodsRotList.append(RotationMatrixZ(-alpha).tolist())
           conrodsPosList.append(pRef.tolist())
           
           #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
           #import conrod CMS
           cmsConrod = ObjectFFRFreducedOrderInterface(femConrod)
           cmsConrodList.append(cmsConrod)
           objFFRFconrod = cmsConrod.AddObjectFFRFreducedOrderWithUserFunctions(exu, mbs, 
                                                       positionRef=pRef, 
                                                       eulerParametersRef=eulerParametersInit, 
                                                       initialVelocity=[0,0,0], 
                                                       initialAngularVelocity=[0,0,0*fRotorStart*2*pi],
                                                       gravity = [0,-0*9.81,0],
                                                       color=[0.1,0.9,0.1,1.])
           mbs.SetObjectParameter(objFFRFconrod['oFFRFreducedOrder'],'VshowNodes',False)
           objFFRFconrodList.append(objFFRFconrod)
       
           #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
           #import piston CMS
           cmsPiston = ObjectFFRFreducedOrderInterface(femPiston)
           cmsPistonList.append(cmsPiston)
   
           pRefPiston = pP+[0,0,zOffset+0.5*b1]
   
           pistonsRotList.append(RotationMatrixZ(0).tolist())
           pistonsPosList.append(pRefPiston.tolist())
   
           objFFRFpiston = cmsPiston.AddObjectFFRFreducedOrderWithUserFunctions(exu, mbs, 
                                                       positionRef=pRefPiston, 
                                                       eulerParametersRef=eulerParameters0, 
                                                       initialVelocity=[0,0,0], initialAngularVelocity=[0,0,0*fRotorStart*2*pi],
                                                       gravity = [0,-0*9.81,0],
                                                       color=[0.1,0.9,0.1,1.])
           mbs.SetObjectParameter(objFFRFpiston['oFFRFreducedOrder'],'VshowNodes',False)
           objFFRFpistonList.append(objFFRFpiston)
   
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       if True: #connect bodies:
           k = 1e6         #joint stiffness
           d = k*0.002     #joint damping
           nMarkerPerPiston = 10    #number of markers per crank/conrod/piston part
   
           genMarkerPos = [[0,0,-d0],[0,0,lTotal*nPistons]]
           genMarkerR   = [r0,r0]
           genMarkerFEM = [femCrank,femCrank]
           genMarkerObject = [objFFRFcrank,objFFRFcrank]
   
           for iCrank in range(len(crankConfig)):
               genMarkerPos += [pkList[iCrank]+[0,0,zOffsetList[iCrank]],pkList[iCrank]+[0,0,zOffsetList[iCrank]+b1],
                               [-0.5*lc,0,-0.5*dc],[-0.5*lc,0, 0.5*dc],[0.5*lc,0,-0.5*dc],[0.5*lc,0, 0.5*dc],
                               [0,0,-0.5*dc],[0,0,0.5*dc], [-dpb,0,0],[lp-dpb,0,0]]
               genMarkerR   += [r1,r1,
                               r1,r1,r2,r2,
                               r2,r2,0.5*bp,0.5*bp]
               genMarkerFEM += [femCrank,femCrank,
                               femConrod,femConrod,femConrod,femConrod,
                               femPiston,femPiston,femPiston,femPiston]
               genMarkerObject += [objFFRFcrank,objFFRFcrank,
                                  objFFRFconrodList[iCrank],objFFRFconrodList[iCrank],objFFRFconrodList[iCrank],objFFRFconrodList[iCrank],
                                  objFFRFpistonList[iCrank],objFFRFpistonList[iCrank],objFFRFpistonList[iCrank],objFFRFpistonList[iCrank]]
   
           markerList = []
           #generate markers for joints:
           for i in range(len(genMarkerPos)):
               p = genMarkerPos[i]
               nodeList=[]
               if p[2] != 0:
                   nodeList= genMarkerFEM[i].GetNodesOnCircle(p, [0,0,1], genMarkerR[i])
               else:
                   nodeList= genMarkerFEM[i].GetNodesOnCircle(p, [1,0,0], genMarkerR[i])
               #print("nodeList"+str(i)+":", nodeList)
               lenNodeList = len(nodeList)
               weights = np.array((1./lenNodeList)*np.ones(lenNodeList))
           
               markerList += [mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=genMarkerObject[i]['oFFRFreducedOrder'], 
                                                               meshNodeNumbers=np.array(nodeList), #these are the meshNodeNumbers
                                                               weightingFactors=weights))]
   
           oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
           
           mGroundPosLeft = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=genMarkerPos[0]))
           mGroundPosRight = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=genMarkerPos[1]))
           
   
           #joints for crankshaft/ground
           oSJleft = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mGroundPosLeft, markerList[0]],
                                               stiffness=[k,k,k], damping=[d,d,d]))
           oSJright = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mGroundPosRight, markerList[1]],
                                               stiffness=[k,k,k], damping=[d,d,d]))
   
           for iCrank in range(len(crankConfig)):
               mOff = nMarkerPerPiston*iCrank
               #joints for crankshaft/conrod:
               oJointCCleft = mbs.AddObject(CartesianSpringDamper(markerNumbers=[markerList[mOff+2], markerList[mOff+4]],
                                                   stiffness=[k,k,k], damping=[d,d,d]))
               oJointCCright= mbs.AddObject(CartesianSpringDamper(markerNumbers=[markerList[mOff+3], markerList[mOff+5]],
                                                   stiffness=[k,k,k], damping=[d,d,d]))
       
               #joints for conrod/piston:
               oJointCPleft = mbs.AddObject(CartesianSpringDamper(markerNumbers=[markerList[mOff+6], markerList[mOff+8]],
                                                   stiffness=[k,k,k], damping=[d,d,d]))
               oJointCPright= mbs.AddObject(CartesianSpringDamper(markerNumbers=[markerList[mOff+7], markerList[mOff+9]],
                                                   stiffness=[k,k,k], damping=[d,d,d]))
       
               mGroundPosPiston = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, 
                                                                   localPosition=[ppList[iCrank][0],0,zOffsetList[iCrank]+0.5*b1]))
               oJointPGleft = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mGroundPosPiston, markerList[mOff+10]],
                                                   stiffness=[0,k,k], damping=[0,d,d]))
               oJointPGright = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mGroundPosPiston, markerList[mOff+11]],
                                                   stiffness=[0,k,k], damping=[0,d,d]))
   
   
       stopTotal = timeit.default_timer()
       print("\ntotal elapsed time=", stopTotal-startTotal)
       mbs.Assemble()
   
       if saveMesh:
           #%%%
           dictMesh = {}
           [points, triangles, normals] = graphics.NGsolveMesh2PointsAndTrigs( ngMesh=ngMeshCrank)
           dictMesh['ngMeshCrank'] = {'points':points, 'triangles':triangles, 'normals':normals}
           [points, triangles, normals] = graphics.NGsolveMesh2PointsAndTrigs( ngMesh=ngMeshConrod)
           dictMesh['ngMeshConrod'] = {'points':points, 'triangles':triangles, 'normals':normals}
           [points, triangles, normals] = graphics.NGsolveMesh2PointsAndTrigs( ngMesh=ngMeshPiston)
           dictMesh['ngMeshPiston'] = {'points':points, 'triangles':triangles, 'normals':normals}
           dictMesh['conrodsRotList'] = conrodsRotList
           dictMesh['conrodsPosList'] = conrodsPosList
           dictMesh['pistonsRotList'] = pistonsRotList
           dictMesh['pistonsPosList'] = pistonsPosList
   
           fileName = 'testData/pistonEngineNGmesh.hdf5'
           SaveDictToHDF5(fileName, dictMesh)
   
   
       #%%now simulate model in exudyn:
       #%%+++++++++++++++++++++
       if True:
           print("totalFEcoordinates=",totalFEcoordinates)
           
           simulationSettings = exu.SimulationSettings()
           
           nodeDrawSize = 0.0005
           SC.visualizationSettings.general.textSize = 14 #30 for cover figure
           SC.visualizationSettings.general.useGradientBackground = True
           SC.visualizationSettings.openGL.lineWidth = 2
   
           SC.visualizationSettings.nodes.defaultSize = nodeDrawSize
           SC.visualizationSettings.nodes.drawNodesAsPoint = False
           SC.visualizationSettings.connectors.defaultSize = 2*nodeDrawSize
           SC.visualizationSettings.connectors.show = False
           
           SC.visualizationSettings.nodes.show = False
           SC.visualizationSettings.nodes.showBasis = True #of rigid body node of reference frame
           SC.visualizationSettings.nodes.basisSize = 0.12
           SC.visualizationSettings.bodies.deformationScaleFactor = 1 #use this factor to scale the deformation of modes
           
           SC.visualizationSettings.openGL.showFaceEdges = True
           SC.visualizationSettings.openGL.showFaces = True
           SC.visualizationSettings.openGL.multiSampling = 4
           
           SC.visualizationSettings.sensors.show = True
           SC.visualizationSettings.sensors.drawSimplified = False
           SC.visualizationSettings.sensors.defaultSize = 0.01
           SC.visualizationSettings.markers.drawSimplified = False
           SC.visualizationSettings.markers.show = False
           SC.visualizationSettings.markers.defaultSize = 0.01
           
           SC.visualizationSettings.loads.drawSimplified = False
           
           #SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Displacement
           SC.visualizationSettings.contour.outputVariableComponent = -1
           SC.visualizationSettings.contour.reduceRange = True
           #SC.visualizationSettings.contour.automaticRange = False
           #SC.visualizationSettings.contour.maxValue = 3e7
           # SC.visualizationSettings.contour.minValue = -0.0003
           # SC.visualizationSettings.contour.maxValue =  0.0003
           
           simulationSettings.solutionSettings.solutionInformation = "NGsolve/NETGEN engine test"
           
           h=0.05e-3
           tEnd = 2
           
           simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
           simulationSettings.timeIntegration.endTime = tEnd
           simulationSettings.solutionSettings.solutionWritePeriod = h*10 #writing already costs much time
           simulationSettings.timeIntegration.verboseMode = 1
           #simulationSettings.timeIntegration.verboseModeFile = 3
           simulationSettings.timeIntegration.newton.useModifiedNewton = True
           
           simulationSettings.solutionSettings.sensorsWritePeriod = h
           #simulationSettings.solutionSettings.coordinatesSolutionFileName = "solution/coordinatesSolution.txt"
           simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse #faster, because system size already quite large
           
           simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 #SHOULD work with 0.9 as well
           simulationSettings.displayStatistics = True
           #simulationSettings.displayComputationTime = True
           SC.visualizationSettings.general.autoFitScene = False #for reloading of renderState to work
           
           #create animation:
           if False:
               simulationSettings.solutionSettings.recordImagesInterval = 0.001
               SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
               SC.visualizationSettings.window.renderWindowSize=[1920,1080]
   
           SC.renderer.Start()
           if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
   
           SC.renderer.DoIdleTasks() #press space to continue
           
           simulate = True #set false to show last stored solution
           if simulate:
               mbs.SolveDynamic(simulationSettings)
           else:
               SC.visualizationSettings.general.autoFitScene = False
               sol = LoadSolutionFile('coordinatesSolution.txt')
               if False: #directly show animation
                   AnimateSolution(mbs, solution=sol, rowIncrement = 1, timeout=0.01, 
                                   createImages = False, runLoop = True)
               else: #interact with animation
                   
                   mbs.SolutionViewer(sol, rowIncrement=1, timeout=0.02)
   
   
           if False: #draw with matplotlib, export as pdf
               SC.visualizationSettings.exportImages.saveImageFormat = "TXT"
               SC.visualizationSettings.exportImages.saveImageAsTextTriangles=True
               SC.renderer.RedrawAndSaveImage() #uses default filename
               
               from exudyn.plot import LoadImage, PlotImage
   
               # plot 2D
               # data = LoadImage('images/frame00000.txt', trianglesAsLines=True)
               # PlotImage(data, HT=HomogeneousTransformation(RotationMatrixZ(0.5*pi)@RotationMatrixX(0.5*pi), [0,0,0]), 
               #           lineWidths=0.5, lineStyles='-', title='', closeAll=True, plot3D=False,
               #           fileName='images/test.pdf')
               
               data = LoadImage('images/frame00000.txt', trianglesAsLines=False)
               PlotImage(data, HT=HomogeneousTransformation(2.5*RotationMatrixZ(0.5*pi)@RotationMatrixY(-0.5*pi), [0,1,0.25]), 
                         lineWidths=0.5, lineStyles='-', triangleEdgeColors='black', triangleEdgeWidths=0.25, title='', closeAll=True, plot3D=True,
                         fileName='images/test3D.pdf')
                       
           SC.renderer.DoIdleTasks()
           SC.renderer.Stop() #safely close rendering window!
           lastRenderState = SC.renderer.GetState() #store model view for next simulation
       
   
   
       
       
       
       
   
   
   
   
   
   


