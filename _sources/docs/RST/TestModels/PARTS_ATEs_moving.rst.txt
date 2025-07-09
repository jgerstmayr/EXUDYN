
.. _testmodels-parts-ates-moving:

********************
PARTS_ATEs_moving.py
********************

You can view and download this file on Github: `PARTS_ATEs_moving.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/PARTS_ATEs_moving.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  example to simulate moving ATEs of PARTS
   #
   # Author:   Michael Pieber and Johannes Gerstmayr
   # Date:     2020-01-14
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   
   import numpy as np
   
   useGraphics = True #without test
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #you can erase the following lines and all exudynTestGlobals related operations if this is not intended to be used as TestModel:
   try: #only if called from test suite
       from modelUnitTests import exudynTestGlobals #for globally storing test results
       useGraphics = exudynTestGlobals.useGraphics
   except:
       class ExudynTestGlobals:
           pass
       exudynTestGlobals = ExudynTestGlobals()
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #plots
   if useGraphics: 
       import matplotlib.pyplot as plt
       import matplotlib.ticker as ticker
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   computeDynamic=True
   displaySimulation=True
   
   
   #Options, if False RevoluteJoint2D are used
   CartesianSpringDamperActive = False
   
   #Options for visualization
   frameList = True
   
   
   #Dimensions
   L1 = 38e-3 #mm
   L2 = 29e-3 #mm
   L3 = 8e-3 #mm
   L4 = 10e-3 #mm
   psi = np.arctan(L3/L1) #11.9Â°
   CE = L3/np.sin(psi) #38.8e-3 mm
   
   
   
   
   
   
   
   
   
   ## Visualization
   if frameList:
       fL = 0.01 #frame length
   else:
       fL = 0
   graphicsX = {'type':'Line', 'color':[0.8,0.1,0.1,1], 'data':[0,0,0, fL,0,0]}
   graphicsY = {'type':'Line', 'color':[0.1,0.8,0.1,1], 'data':[0,0,0, 0,fL,0]}
   graphicsZ = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[0,0,0, 0,0,fL]}
   frameList = [] #no frames
   #frameList = [graphicsX,graphicsY,graphicsZ]
   
   
   
   
   class CSixBarLinkage:
       def __init__(self, name = ''):
           self.name = name #needed?
   
   ## USER defined functions
   
   def To3D(p): #convert numpy 2D vector to 3D list
       return [p[0],p[1],0]   
   
   
   def RotZVec(angleRad,vec):
       Rz=np.array([ [np.cos(angleRad),-np.sin(angleRad)],
                         [np.sin(angleRad), np.cos(angleRad)] ])
       return np.dot(Rz,np.array(vec)); 
   
   
   def centreM(vec1, vec2): #Centre M of a line AB
       return (vec1+vec2)/2
   
   
   def userLoadDriveAle(t, load): #Add drive via ALE:
       if t < 1:
           return [-t*0.1,t*0.1,0 ]
       return [-1,1,0]
   
   
   
   def SixBarLinkage(P00,theta,alphaMean): #compute one six-bar linkage (P00: idealized rotation point, theta: rotaton of the six-bar linkage, angleMean: angle of the six-bar linkage (alpha, beta, gamma))
           
       #Parameters for RigidBody2D
       massRigid = 1e0
       inertiaRigid = 3.3e1
       
       #Parameters for CartesianSpringDamper
       if CartesianSpringDamperActive:
           k=[1e6,1e6,0]
           d=[1e2,1e2,0]
       
       #define the points/coordinates one six-bar linkage
       P01=[0,0]
       P1=[L1,0]
       P2=[L1+L2,0]
       P3=[L1,L3]
       P4=[L1+L2,L3]
       P5=[L1+CE*np.cos(alphaMean-psi),L3+CE*np.sin(alphaMean-psi)]
       P6=[L1+L2+CE*np.cos(alphaMean-psi),L3+CE*np.sin(alphaMean-psi)]
       P7=[L1+CE*np.cos(alphaMean-psi)+L2*np.cos(alphaMean),L3+CE*np.sin(alphaMean-psi)+L2*np.sin(alphaMean)]
       P8=[CE*np.cos(alphaMean-psi),CE*np.sin(alphaMean-psi)]
       P9=[CE*np.cos(alphaMean-psi)+L2*np.cos(alphaMean),CE*np.sin(alphaMean-psi)+L2*np.sin(alphaMean)]
       P10=[L1*np.cos(alphaMean),L1*np.sin(alphaMean)]
       P11=[(L1+L2)*np.cos(alphaMean),(L1+L2)*np.sin(alphaMean)]    
       
       PList=[P01,P1,P2,P3,P4,P5,P6,P7,P8,P9,P10,P11]
       
       #calculate vectors of the two long arms
       P53=list(np.array(PList[3])-np.array(PList[5]))
       P56=list(np.array(PList[6])-np.array(PList[5]))
       P57=list(np.array(PList[7])-np.array(PList[5]))
       P58=list(np.array(PList[8])-np.array(PList[5]))
   
       #displacement and rotation of all points P01-P11
       pT=[]
       for x in range(len(PList)):
           pT+=[list(RotZVec(theta,np.array(PList[x]))+P00)]
       
       #calculate centrers of mass of the rigid bodies
       M0=list(centreM(np.array(pT[4]), np.array(pT[3])))
       M2=list(centreM(np.array(pT[6]), np.array(pT[4])))
       M5=list(centreM(np.array(pT[7]), np.array(pT[9])))
       M7=list(centreM(np.array(pT[9]), np.array(pT[8])))
   
    
       #Draw++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       bLength = 0.5e-3    #y-dim of pendulum
       graphicsCE = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-CE/2,-bLength,0, CE/2,-bLength,0, CE/2,bLength,0, -CE/2,bLength,0, -CE/2,-bLength,0]}
       graphicsL2 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-L2/2,-bLength,0, L2/2,-bLength,0, L2/2,bLength,0, -L2/2,bLength,0, -L2/2,-bLength,0]}
        
       ad=list(RotZVec((alphaMean-psi),[-CE,-bLength]))
       bd=list(RotZVec((alphaMean-psi),[0,-bLength]))
       cd=list(RotZVec((alphaMean-psi),[0,bLength]))
       dd=list(RotZVec((alphaMean-psi),[-CE,bLength]))
       ed=list(RotZVec((alphaMean-psi),[-CE,-bLength]))    
       graphicsCE1 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':ad+[0]+ bd+[0]+ cd+[0]+ dd+[0]+ ed+[0]} #background
       
       ad=list(RotZVec((alphaMean-psi),[0,-bLength]))
       bd=list(RotZVec((alphaMean-psi),[L2*np.cos(psi),L2*np.sin(psi)-bLength]))
       cd=list(RotZVec((alphaMean-psi),[L2*np.cos(psi),L2*np.sin(psi)+bLength]))
       dd=list(RotZVec((alphaMean-psi),[0,bLength]))
       ed=list(RotZVec((alphaMean-psi),[0,-bLength]))  
       graphicsL21 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':ad+[0]+ bd+[0]+ cd+[0]+ dd+[0]+ ed+[0]} #background
       
       ad=list(RotZVec(0,[-CE*np.cos(psi),-CE*np.sin(psi)-bLength]))
       bd=list(RotZVec(0,[0,-bLength]))
       cd=list(RotZVec(0,[0,bLength]))
       dd=list(RotZVec(0,[-CE*np.cos(psi),-CE*np.sin(psi)+bLength]))
       ed=list(RotZVec(0,[-CE*np.cos(psi),-CE*np.sin(psi)-bLength]))    
       graphicsCE2 = {'type':'Line', 'color':[0.1,0.1,0.8,1],'data':ad+[0]+ bd+[0]+ cd+[0]+ dd+[0]+ ed+[0]}
   
       ad=list(RotZVec(0,[0,-bLength]))
       bd=list(RotZVec(0,[L2,-bLength]))
       cd=list(RotZVec(0,[L2,bLength]))
       dd=list(RotZVec(0,[0,bLength]))
       ed=list(RotZVec(0,[0,-bLength]))  
       graphicsL22 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':ad+[0]+ bd+[0]+ cd+[0]+ dd+[0]+ ed+[0]} #background
   
   
       #Define rigid bodies and markers+++++++++++++++++++++++++++++++++++++++++++
       #rigid body M0 with markers mR01,mR02
       nRigid0 = mbs.AddNode(Rigid2D(referenceCoordinates=M0+[theta], initialVelocities=[0.,0.,0.]));
       oRigid0 = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid0,visualization=VObjectRigidBody2D(graphicsData= [graphicsL2]+frameList)))
       mR01 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid0, localPosition=[-L2/2,0.,0.]))
       mR02 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid0, localPosition=[L2/2,0.,0.]))
   
       #rigid body M1 with markers mR1,mR2,mR15
       nRigid1 = mbs.AddNode(Rigid2D(referenceCoordinates=pT[5]+[0.+theta], initialVelocities=[0.,0.,0.]));
       oRigid1 = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid1,visualization=VObjectRigidBody2D(graphicsData= [graphicsCE1,graphicsL21]+frameList)))
       mR1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid1, localPosition=P53+[0.]))
       mR2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid1, localPosition=P57+[0.]))    
       mR15 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid1, localPosition=[0.,0.,0.]))
   
       #rigid body M2 with markers mR3,mR4     
       nRigid2 = mbs.AddNode(Rigid2D(referenceCoordinates=M2+[alphaMean-psi+theta], initialVelocities=[0.,0.,0.]));
       oRigid2 = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid2,visualization=VObjectRigidBody2D(graphicsData= [graphicsCE]+frameList)))
       mR3 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid2, localPosition=[-CE/2,0.,0]))
       mR4 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid2, localPosition=[CE/2,0.,0]))
   
       #rigid body M5 with markers mR9,mR10
       nRigid5 = mbs.AddNode(Rigid2D(referenceCoordinates=M5+[psi+theta], initialVelocities=[0.,0.,0.]));
       oRigid5 = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid5,visualization=VObjectRigidBody2D(graphicsData= [graphicsCE]+frameList)))
       mR9 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid5, localPosition=[-CE/2,0.,0]))
       mR10 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid5, localPosition=[CE/2,0.,0]))
   
       #rigid body M6 with markers mR11,mR12,mR65
       nRigid6 = mbs.AddNode(Rigid2D(referenceCoordinates=pT[5]+[0.+theta], initialVelocities=[0.,0.,0.]));
       oRigid6 = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid6,visualization=VObjectRigidBody2D(graphicsData= [graphicsCE2,graphicsL22]+frameList)))
       mR11 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid6, localPosition=P58+[0.]))
       mR12 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid6, localPosition=P56+[0.]))
       mR65 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid6, localPosition=[0.,0.,0.]))
   
       #rigid body M7 with markers mR13,mR14
       nRigid7 = mbs.AddNode(Rigid2D(referenceCoordinates=M7+[alphaMean+theta], initialVelocities=[0.,0.,0.]));
       oRigid7 = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid7,visualization=VObjectRigidBody2D(graphicsData= [graphicsL2]+frameList)))
       mR13 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid7, localPosition=[-L2/2,0.,0.]))
       mR14 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid7, localPosition=[L2/2,0.,0.]))
   
       #plug and socket for connection
       mC1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid0, localPosition=[-L2/2-L4,-L3,0.]))
       mC2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid7, localPosition=[-L2/2-L4,L3,0.]))
   
       #markers for prismatic joints
       mP02 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRigid0, localPosition=[L2/2,0.,0.]))
       mP14 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRigid7, localPosition=[L2/2,0.,0.]))
   
   
       #Define joints+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++        
       if CartesianSpringDamperActive:
           mbs.AddObject(CartesianSpringDamper(markerNumbers=[mR01,mR1],stiffness=k,damping=d))
           mbs.AddObject(CartesianSpringDamper(markerNumbers=[mR02,mR3],stiffness=k,damping=d))
                 
           mbs.AddObject(CartesianSpringDamper(markerNumbers=[mR10,mR2],stiffness=k,damping=d))
           mbs.AddObject(CartesianSpringDamper(markerNumbers=[mR14,mR9],stiffness=k,damping=d))
           mbs.AddObject(CartesianSpringDamper(markerNumbers=[mR13,mR11],stiffness=k,damping=d))
           
           mbs.AddObject(CartesianSpringDamper(markerNumbers=[mR15,mR65],stiffness=k,damping=d))
           mbs.AddObject(CartesianSpringDamper(markerNumbers=[mR4,mR12],stiffness=k,damping=d))
       else:
           mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR01,mR1]))
           mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR02,mR3]))
                  
           mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR10,mR2]))
           mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR14,mR9]))
           mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR13,mR11]))
       
           mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR15,mR65]))
           mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR4,mR12]))
       
       
       dictSixBarLinkages = {"markersForPrismaticJoints":[mP02,mP14],"markersForConnectors":[mC1,mC2],"markers":[mR1,mR3,mR9,mR01,mR02],"coordinatePoints":pT,"objects":[oRigid0,oRigid7],"nodes":[nRigid0,nRigid7]}    
   
       return dictSixBarLinkages
   
   
   
   def ATC(A,B,C):
       
       AB=(np.array(B)-np.array(A))
       BC=(np.array(C)-np.array(B))
       AC=(np.array(C)-np.array(A))
       
   #    offsetAngle=np.arcsin( np.dot(np.array(e1),AB)/(np.linalg.norm(np.array(e1)*np.linalg.norm(AB))))
       offsetAngle=np.arctan2(AB[1],AB[0])
   
       a=np.linalg.norm(np.array(C)-np.array(B))
       b=np.linalg.norm(np.array(C)-np.array(A))
       c=np.linalg.norm(np.array(B)-np.array(A))
       
       alpha=np.arccos((np.square(b)+np.square(c)-np.square(a))/(2*b*c))
       beta=np.arccos((np.square(a)+np.square(c)-np.square(b))/(2*a*c))
       gamma=np.arccos((np.square(a)+np.square(b)-np.square(c))/(2*a*b))
       
       s1=SixBarLinkage(A,offsetAngle,alpha)
       s2=SixBarLinkage(B,np.pi-beta+offsetAngle,beta)
       s3=SixBarLinkage(C,-(beta+gamma)+offsetAngle,gamma)  
       
       #markersForConnectors
       mFC=[s1["markersForConnectors"][0],s2["markersForConnectors"][1],s2["markersForConnectors"][0],s3["markersForConnectors"][1],s3["markersForConnectors"][0],s1["markersForConnectors"][1]]  
       
       #markersForPismaticJoints
       mFPJ=[s1["markersForPrismaticJoints"][0],s2["markersForPrismaticJoints"][1],s2["markersForPrismaticJoints"][0],s3["markersForPrismaticJoints"][1],s3["markersForPrismaticJoints"][0],s1["markersForPrismaticJoints"][1]]
   
       mbs.AddObject(PrismaticJoint2D(markerNumbers=[mFPJ[0],mFPJ[1]],axisMarker0=[1.,0.,0.],normalMarker1=[0.,-1.,0.], constrainRotation=True))
       mbs.AddObject(PrismaticJoint2D(markerNumbers=[mFPJ[2],mFPJ[3]],axisMarker0=[1.,0.,0.],normalMarker1=[0.,-1.,0.], constrainRotation=True))
       mbs.AddObject(PrismaticJoint2D(markerNumbers=[mFPJ[5],mFPJ[4]],axisMarker0=[1.,0.,0.],normalMarker1=[0.,-1.,0.], constrainRotation=True))
   
       La=a-2*L1-2*L2
       Lb=b-2*L1-2*L2
       Lc=c-2*L1-2*L2
       mbs.AddObject(SpringDamper(markerNumbers = [mFPJ[0],mFPJ[1]], stiffness = 1e5, damping=10e2, referenceLength=Lc)) 
       mbs.AddObject(SpringDamper(markerNumbers = [mFPJ[2],mFPJ[3]], stiffness = 1e5, damping=10e2, referenceLength=La))    
       mbs.AddObject(SpringDamper(markerNumbers = [mFPJ[4],mFPJ[5]], stiffness = 1e5, damping=10e2, referenceLength=Lb))      
   
       points=[s1["coordinatePoints"][0],s1["coordinatePoints"][3],s1["coordinatePoints"][4]]    
       nodes=[s1["nodes"][0],s1["nodes"][1],s2["nodes"][1],s3["nodes"][1]]
       objects=[s1["objects"][0],s1["objects"][1]]
       markers=[s1["markers"][0],s1["markers"][1],s1["markers"][2],s1["markers"][3],s1["markers"][4]]
       
       dictATE = {"markersForConnectors":mFC,"markersForPrismaticJoints":mFPJ,"coordinatePoints":points,"nodes":nodes,"objects":objects,"markers":markers} 
       return dictATE
   
   
   
   
   
   ## START main program
   
   #define start mesh   
   
   nx = 4
   ny = 2
   endTime = 0.05
   
   ###connection of ATCs
   kk=[1e8,1e8,0]
   dd=[1e2,1e2,0]
   
   
   topCon=[]
   bottomCon=[]
   for y in range(ny):
       for x in range(nx):
           A=[0.229*(x),0.229*(y)]
           B=[0.229*(x+1),0.229*(y)]
           C=[0.229*(x+1),0.229*(y+1)]
           D=[0.229*(x),0.229*(y+1)]    
       
       
           #create ATCs
           n1=ATC(A,B,C)
           n2=ATC(A,C,D)
           #n3=ATC(C,D,E)
           
           mbs.AddObject(CartesianSpringDamper(markerNumbers=[n1["markersForConnectors"][5],n2["markersForConnectors"][0]],stiffness=kk,damping=dd))
           mbs.AddObject(CartesianSpringDamper(markerNumbers=[n1["markersForConnectors"][4],n2["markersForConnectors"][1]],stiffness=kk,damping=dd))  
         
           if x == 0 and y == 0:
               nFirstATE=n1 #used only now first ATC for visualization purpose/boundary/force        
           if x == 0:
               topCon+=[n2["markersForConnectors"][3]]
               topCon+=[n2["markersForConnectors"][2]]
               bottomCon+=[n1["markersForConnectors"][0]]
               bottomCon+=[n1["markersForConnectors"][1]]
           if x > 0 and x < nx:
               c21=n2["markersForConnectors"][5]
               c22=n2["markersForConnectors"][4]
               
               mbs.AddObject(CartesianSpringDamper(markerNumbers=[c11alt,c21],stiffness=kk,damping=dd))
               mbs.AddObject(CartesianSpringDamper(markerNumbers=[c12alt,c22],stiffness=kk,damping=dd))
               
               topCon+=[n2["markersForConnectors"][3]]
               topCon+=[n2["markersForConnectors"][2]]
               bottomCon+=[n1["markersForConnectors"][0]]
               bottomCon+=[n1["markersForConnectors"][1]]
               
           c11alt=n1["markersForConnectors"][2]
           c12alt=n1["markersForConnectors"][3]
       
     
       if y > 0 and y < ny:
           for i in range(len(topCon)-(nx*2)):
               mbs.AddObject(CartesianSpringDamper(markerNumbers=[topCon[i],bottomCon[i+(nx*2)]],stiffness=kk,damping=dd))            
   
   
   
   n1=nFirstATE
   P0=n1["coordinatePoints"][0]
   #
   #
   ###connection of ATCs
   #kk=[1e4,1e4,0]
   #dd=[1e2,1e2,0]
   #
   #mbs.AddObject(CartesianSpringDamper(markerNumbers=[n2["markersForConnectors"][0],n1["markersForConnectors"][5]],stiffness=kk,damping=dd))
   #mbs.AddObject(CartesianSpringDamper(markerNumbers=[n2["markersForConnectors"][1],n1["markersForConnectors"][4]],stiffness=kk,damping=dd)) 
   ##
   
   
   ##loads
   #mbs.AddLoad(Force(markerNumber = n1["markersForConnectors"][4], loadVector=[1e2,1e2,0]))
   
   
   #Michael
   #mbs.AddLoad(Force(markerNumber = topCon[15], loadVector=[1e3,-1e3,0]))
   #mbs.AddLoad(Force(markerNumber = bottomCon[7], loadVector=[-1e3,-1e3,0]))
   
   
   #Johannes:
   def userLoad(mbs, t, loadVector):
       f=0.01+0.99*(1-np.cos((t/0.05)*np.pi)) #use small initial value for solver
       if t>8:
           f = 1
       return [f*loadVector[0], f*loadVector[1], f*loadVector[2]]
   
   fL = 1e3*10
   mbs.AddLoad(Force(markerNumber = topCon[15], loadVector=[fL,-fL,0], loadVectorUserFunction = userLoad))
   mbs.AddLoad(Force(markerNumber = bottomCon[7], loadVector=[-fL,-fL,0], loadVectorUserFunction = userLoad))
   
   #for i in range (8):
   #    mbs.AddLoad(Force(markerNumber = topCon[15-i], loadVector=[1e2,-1e2*0,0]))
   #for i in range (8):
   #    mbs.AddLoad(Force(markerNumber = bottomCon[i], loadVector=[-1e2,-1e2*0,0]))
   #
   #
   #
   #mbs.AddLoad(Force(markerNumber = bottomCon[7], loadVector=[1e2,1e2,0]))
   #mbs.AddLoad(Force(markerNumber = topCon[15], loadVector=[1e2,1e2,0]))
   
   
   
   #boundaries
   nGround=mbs.AddNode(PointGround(referenceCoordinates=P0+[0], visualization=VNodePointGround(show=False)))
   
   nRidgid0=n1["nodes"][0]
   ##prescribe rotation of link
   mCoordinateGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0)) #gives always 0 displacement
   
   mCoordinateRigid7x = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRidgid0, coordinate=0)) #angle of node of Rigid7
   mCoordinateRigid7y = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRidgid0, coordinate=1)) #angle of node of Rigid7
   mCoordinateRigid7phi = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRidgid0, coordinate=2)) #angle of node of Rigid7
   #
   #
   #JOH: oCoordAlphax = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround,mCoordinateRigid7x], offset = 0))
   oCoordAlphay = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround,mCoordinateRigid7y], offset = 0))
   #JOH: oCoordAlpha = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround,mCoordinateRigid7phi], offset = 0))
   
   
   
   nRidgid7=n1["nodes"][1]
   mCoordinateRigid7x = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRidgid7, coordinate=0)) #angle of node of Rigid7
   mCoordinateRigid7y = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRidgid7, coordinate=1)) #angle of node of Rigid7
   mCoordinateRigid7phi = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRidgid7, coordinate=2)) #angle of node of Rigid7
   
   
   #oCoordAlphax = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround,mCoordinateRigid7x], offset = 0))
   #JOH: oCoordAlphay = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround,mCoordinateRigid7y], offset = 0))
   #oCoordAlpha = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround,mCoordinateRigid7phi], offset = 0))
   
   #hack to add additional coordinate constraints:
   constrainLeftSide = True
   if constrainLeftSide:
       nn = mbs.systemData.NumberOfNodes()
       for i in range(nn):
           n0 = mbs.GetNode(i)
           if n0['nodeType'] == 'RigidBody2D':
               if np.round(n0['referenceCoordinates'][0],3) == 0.008: #constrain all rigidbody nodes with xref=0.008
                   mCC = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=i, coordinate=0)) #x-displacement
                   mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround,mCC]))
                   mCC = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=i, coordinate=1)) #y-displacement
                   mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround,mCC]))
   
   
   
   #mbs.AddObject(DistanceConstraint(markerNumbers=[n1["markersForPrismaticJoints"][0],n1["markersForPrismaticJoints"][1]], distance = 0.))
   #mbs.AddObject(DistanceConstraint(markerNumbers=[n1["markersForPrismaticJoints"][2],n1["markersForPrismaticJoints"][3]], distance = 0.))
   #mbs.AddObject(DistanceConstraint(markerNumbers=[n1["markersForPrismaticJoints"][4],n1["markersForPrismaticJoints"][5]], distance = 0.))
   
   
   mbs.Assemble() #creates initial configuration
   #exu.Print(mbs)
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   
   uList=[]
   phiList=[]
   
   
   T=0.002
   SC.visualizationSettings.connectors.defaultSize = T
   SC.visualizationSettings.bodies.defaultSize = [T, T, T]
   SC.visualizationSettings.nodes.defaultSize = 0.0025
   SC.visualizationSettings.markers.defaultSize = 0.005
   SC.visualizationSettings.loads.defaultSize = 0.005
   
   SC.visualizationSettings.nodes.show= False
   SC.visualizationSettings.markers.show= False
   
   SC.visualizationSettings.openGL.lineWidth=2 #maximum
   SC.visualizationSettings.openGL.lineSmooth=True
   SC.visualizationSettings.general.drawCoordinateSystem = False
   SC.visualizationSettings.window.renderWindowSize=[1600,1024]
   
   ##++++++++++++++++++++++++++++++
   ##ANIMATIONS
   ##make images for animations (requires FFMPEG):
   ##requires a subfolder 'images'
   #simulationSettings.solutionSettings.recordImagesInterval=endTime/200
   
   if useGraphics: #only start graphics once, but after background is set
       SC.renderer.Start()
       
       if displaySimulation:
           SC.renderer.DoIdleTasks()
   
   #SC.visualizationSettings.nodes.show = False
   
   simulationSettings.solutionSettings.solutionInformation = "PARTS_1Joint"
   
   
   if computeDynamic:
       simulationSettings.timeIntegration.numberOfSteps = 50
       simulationSettings.timeIntegration.endTime = endTime
       simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8 #10000
       simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-8
       simulationSettings.timeIntegration.verboseMode = 1
       #simulationSettings.timeIntegration.verboseModeFile = 1
       
       simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
       simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints =  simulationSettings.timeIntegration.generalizedAlpha.useNewmark
       simulationSettings.timeIntegration.newton.useModifiedNewton = False
       simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
       simulationSettings.timeIntegration.adaptiveStep = False #disable adaptive step reduction
       ##############################################################
       # IMPORTANT!!!!!!!!!
       simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse #sparse solver !!!!!!!!!!!!!!!
       ##############################################################
       simulationSettings.displayStatistics = True
           
       
       mbs.SolveDynamic(simulationSettings)
   
       
   if useGraphics: #only start graphics once, but after background is set
       if displaySimulation:
           SC.renderer.DoIdleTasks()
           
       SC.renderer.Stop() #safely close rendering window!
   
   nLast = mbs.systemData.NumberOfNodes()-1#just take last node-1 (last node is ground)
   
   uy=mbs.GetNodeOutput(nLast-1,exu.OutputVariableType.Position)[1] #y-coordinate of last node
   exu.Print("uy=", uy)
   exudynTestGlobals.testError = uy - (0.44656762760262225) #2020-01-16: 0.44656762760262225
   exudynTestGlobals.testResult = uy
   
   
   
   


