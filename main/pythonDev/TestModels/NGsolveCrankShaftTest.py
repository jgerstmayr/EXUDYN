#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  generate a piston engine with variable number of pistons
#
# Author:   Johannes Gerstmayr
# Date:     2020-06-12
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from ngsolve import *
from netgen.geom2d import unit_square

import netgen.libngpy as libng

from netgen.csg import *

import numpy as np

netgenDrawing = False #set true, to show geometry and mesh in NETGEN

if netgenDrawing:
    import netgen.gui

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
#parameters
#crank:
b1 = 0.020 #width of journal bearing
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
d0 = 0.030 #shaft length at left/right support
d1 = 0.015 #shaft length at intermediate support


#distance rings:
db = 0.002          #width of distance ring
rdb0 = r0+0.002     #total radius of distance ring, shaft
rdb1 = r1+0.002     #total radius of distance ring, crank

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


geo = CSGeometry()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#choose configuration:
crankConfig = [0,np.pi*2./3.,2.*np.pi*2./3.,2.*np.pi*2./3.,np.pi*2./3.,0] #6-piston
#crankConfig = [0] #1-piston
#crankConfig = [0,np.pi] #2-piston
#crankConfig = [0,np.pi,np.pi,0] #4-piston

crank = GenerateCrank(0, crankConfig[0])
zPos = lTotal
for i in range(len(crankConfig)-1):
    angle = crankConfig[i+1]
    crank += GenerateCrank(zPos, angle)
    zPos += lTotal

# crank = (GenerateCrank(0, 0) + GenerateCrank(lTotal, np.pi*2./3.) + GenerateCrank(2*lTotal, np.pi*2.*2./3.)+
#           GenerateCrank(3*lTotal, np.pi*2.*2./3.) + GenerateCrank(4*lTotal, np.pi*2./3.))

geo.Add(crank)

if netgenDrawing: Draw (geo)

#do meshing, if geometry is successful
if True:
    mesh = Mesh( geo.GenerateMesh(maxh=0.005))
    mesh.Curve(1)
    if netgenDrawing: Draw(mesh)
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #save mesh to file:
    #mesh.ngmesh.Export('crankshaft.txt','Neutral Format')

#here starts the EXUDYN part
if False:
    import sys
    #C:\DATA\cpp\EXUDYN_git\main\bin\WorkingRelease64P37
    sys.path.append('C:/DATA/cpp/EXUDYN_git/main/bin/WorkingRelease64P37') #for exudyn, itemInterface and from exudyn.utilities import *
    import exudyn as exu
    from exudyn.utilities import *
    from exudyn.FEM import *

    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    #crankshaft and piston mechanical parameters:
    density = 7850
    youngsModulus = 2.1e11*1e-3
    poissonsRatio = 0.3
    fRotorStart = 10 #initial revolutions per second

    #import mesh into EXUDYN FEMinterface
    fem = FEMinterface()
    fem.ImportMeshFromNGsolve(mesh, density, youngsModulus, poissonsRatio, verbose = True)

    nModes = 8
    excludeRigidBodyModes = 6
    print("Compute eigenmodes Scipy ....")

    fem.ComputeEigenmodes(nModes, excludeRigidBodyModes = excludeRigidBodyModes, useSparseSolver = True)
    print("eigen freq.=", fem.GetEigenFrequenciesHz()[0:nModes])
    
    print("Create CMS object and matrices ....")
    cms = ObjectFFRFreducedOrderInterface(fem)
    
    #user functions should be defined outside of class:
    def UFmassFFRFreducedOrder(mbs, t, itemIndex, qReduced, qReduced_t):
        return cms.UFmassFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)
    
    def UFforceFFRFreducedOrder(mbs, t, itemIndex, qReduced, qReduced_t):
        return cms.UFforceFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)
    
    objFFRF = cms.AddObjectFFRFreducedOrderWithUserFunctions(exu, mbs, positionRef=[0,0,0], eulerParametersRef=eulerParameters0, 
                                                  initialVelocity=[0,0,0], initialAngularVelocity=[0,0,fRotorStart*2*pi],
                                                  gravity = [0,-0*9.81,0],
                                                  UFforce=UFforceFFRFreducedOrder, UFmassMatrix=UFmassFFRFreducedOrder,
                                                  color=[0.1,0.9,0.1,1.])
    mbs.SetObjectParameter(objFFRF['oFFRFreducedOrder'],'VshowNodes',False)


    #now simulate model in exudyn:
    if True:
        mbs.Assemble()
        
        simulationSettings = exu.SimulationSettings()
        
        nodeDrawSize = 0.0005
        SC.visualizationSettings.nodes.defaultSize = nodeDrawSize
        SC.visualizationSettings.nodes.drawNodesAsPoint = False
        SC.visualizationSettings.connectors.defaultSize = 2*nodeDrawSize
        
        SC.visualizationSettings.nodes.show = True
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
        SC.visualizationSettings.markers.show = True
        SC.visualizationSettings.markers.defaultSize = 0.01
        
        SC.visualizationSettings.loads.drawSimplified = False
        
        SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
        SC.visualizationSettings.contour.outputVariableComponent = 2 #y-component
        # SC.visualizationSettings.contour.automaticRange = False
        # SC.visualizationSettings.contour.minValue = -0.0003
        # SC.visualizationSettings.contour.maxValue =  0.0003
        
        simulationSettings.solutionSettings.solutionInformation = "NGsolve/NETGEN engine test"
        
        h=5e-4
        tEnd = 10
        
        simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
        simulationSettings.timeIntegration.endTime = tEnd
        simulationSettings.solutionSettings.solutionWritePeriod = h
        simulationSettings.timeIntegration.verboseMode = 1
        #simulationSettings.timeIntegration.verboseModeFile = 3
        simulationSettings.timeIntegration.newton.useModifiedNewton = True
        
        simulationSettings.solutionSettings.sensorsWritePeriod = h
        #simulationSettings.solutionSettings.coordinatesSolutionFileName = "solution/coordinatesSolution.txt"
        
        simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 #SHOULD work with 0.9 as well
        simulationSettings.displayStatistics = True
        simulationSettings.displayComputationTime = True
        
        #create animation:
        #simulationSettings.solutionSettings.recordImagesInterval = 0.0002
        #SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
        
        exu.StartRenderer()
        lastRenderState = {'centerPoint': [-0.10326996445655823, -0.06658822298049927, 0.0],
                              'maxSceneSize': 0.10000000149011612,
                              'zoom': 0.11565303057432175,
                              'currentWindowSize': [1024, 768],
                              'modelRotation': np.array([[ 0.5591929 ,  0.        ,  0.82903755],
                                    [-0.37637517,  0.8910065 ,  0.25386825],
                                    [-0.73867786, -0.4539905 ,  0.49824452]])}
        SC.SetRenderState(lastRenderState) #load last model view
    
        mbs.WaitForUserToContinue() #press space to continue
        
        mbs.SolveDynamic(simulationSettings)
        
        SC.WaitForRenderEngineStopFlag()
        exu.StopRenderer() #safely close rendering window!
        lastRenderState = SC.GetRenderState() #store model view for next simulation
    












    
    # fes = VectorH1(mesh, order=2)
    # u = fes.TrialFunction()
    # v = fes.TestFunction()
    # a = BilinearForm(fes)
    # m = BilinearForm(fes)
    
    # def sigma(eps, mu, lam):
    #     return 2*mu*eps + lam*Trace(eps) * Id(eps.dims[0])
    
    # E, nu = 210e9, 0.2
    # mu  = E / 2 / (1+nu)
    # lam = E * nu / ((1+nu)*(1-2*nu))
    
    # rho = 4000
    
    # shift = 100 #shift mass matrix
    # nEig = 10 #find nEig eigenvalues
    
    # a += InnerProduct(sigma(Sym(Grad(u)),mu,lam), Sym(Grad(v)))*dx
    # a += shift*rho*u*v * dx
    
    # m += rho*u*v * dx
    
    # with TaskManager():
    #     print ("call assemble")
    #     a.Assemble()
    #     m.Assemble()
    
    #     #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #     #export to scipy sparse matrices:
    #     from scipy.sparse import csr_matrix
    #     K = csr_matrix( a.mat.CSR(), copy=True )
    #     M = csr_matrix( m.mat.CSR(), copy=True )
    #     print("K.shape=",K.shape)
    
    #     pre = a.mat.Inverse(inverse="sparsecholesky")
    #     print ("NGSolve pinvit ...")    
    #     evals, evecs = solvers.PINVIT(a.mat, m.mat, pre=pre, num=nEig, 
    #                                   maxit=2, printrates = True, GramSchmidt=True)
    
    # evals = [v-shift for v in evals]
    
    # print ("NGsolve evals = ", evals[6:nEig])
    
    # gfu = GridFunction(fes, multidim=len(evecs))
    # for i in range(len(evecs)):
    #     gfu.vecs[i].data = evecs[i]
    
    # # Draw (gfu, mesh, "u", sd=4)
    
    # gfu1 =  GridFunction(fes)
    # gfu1.vec.data = gfu.vecs[7]
    
    # fac = Parameter(0.02)
    # Draw (fac*gfu1, mesh, "animated")
    
    
    # #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # #compute eigenvalues and eigenmodes
    # calcEig = False
    # if calcEig:
    #     #from scipy.linalg import solve, eigh, eig #eigh for symmetric matrices, positive definite
    #     from scipy.sparse.linalg import eigsh #eigh for symmetric matrices, positive definite
    #     import numpy as np
    
    #     print ("scipy eigh...")
    #     [eigvals, eigvecs] = eigsh(A=K, k=nEig, M=M, which='SM') #this gives omega^2 ... squared eigen frequencies (rad/s)
    #     ev = np.sort(a=abs(eigvals))-shift
    #     print("\nScipy eigenvalues=",ev[6:nEig])
    #     # listEig = []
    #     # for i in range(nEig):
    #     #     listEig += [np.sqrt(ev[i])/(2*np.pi)]
    #     # print("\neigenvalues Scipy =", listEig)






