
.. _testmodels-objectffrftest:

*****************
objectFFRFTest.py
*****************

You can view and download this file on Github: `objectFFRFTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/objectFFRFTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for ObjectFFRF with C++ implementation user function for reduced order equations of motion
   # NOTE: this is a development file, with lots of unstructured code; just kept for consistency!
   #
   # Author:   Johannes Gerstmayr 
   # Date:     2020-05-13
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.FEM import *
   
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
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   import numpy as np
   
   #==> DO NOT USE THE APPROACH of this test, but use the FEMinterface; also use the more efficient ObjectFFRFreducedOrder !!!!
   
   nodeDrawSize = 0.0025
   
   testMode = 1 #0=MarkerGeneric, 1=MarkerSuperElement
   modeNames = ['FFRF_MG','FFRF']
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Use FEMinterface to import FEM model and create FFRFreducedOrder object
   fem = FEMinterface()
   inputFileName = 'testData/rotorDiscTest' #runTestSuite.py is at another directory
   
   nodes=fem.ImportFromAbaqusInputFile(inputFileName+'.inp', typeName='Instance', name='rotor-1')
   elements = np.array(fem.elements[0]['Hex8'])
   
   fem.ReadMassMatrixFromAbaqus(inputFileName+'MASS1.mtx')
   fem.ReadStiffnessMatrixFromAbaqus(inputFileName+'STIF1.mtx')
   fem.ScaleStiffnessMatrix(1e-2) #for larger deformations, stiffness is reduced to 1%
   
   massMatrix = fem.GetMassMatrix(sparse=False)
   stiffnessMatrix = fem.GetStiffnessMatrix(sparse=False)
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   exu.Print("nodes size=", nodes.shape)
   exu.Print("elements size=", elements.shape)
   
   minZ = min(nodes[:,2])
   maxZ = max(nodes[:,2])
   midZ = 0.5*(minZ+maxZ)
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #nLeft = (78-1)
   #nRight = (77-1)
   #nMid = (43-1)
   nLeft = -1
   nRight = -1
   nMid = -1
   nForce = -1 #40; node where fore is attached
   forceZ = 0.1
   
   #nTopRight = 12 #JG, excitation
   nForce = 12 #JG, excitation
   nTopMid = 9  #alternative: 103; JG, fixed node "rotation"
   
   unbalance = 0.1
   massMatrix[nTopMid*3+0,nTopMid*3+0] += unbalance
   massMatrix[nTopMid*3+1,nTopMid*3+1] += unbalance
   massMatrix[nTopMid*3+2,nTopMid*3+2] += unbalance
   
   #find output nodes:
   for i in range(len(nodes)):
       n = nodes[i]
       if abs(n[2] - minZ) < 1e-6 and abs(n[1]) < 1e-6 and abs(n[0]) < 1e-6: 
           nLeft = i
       if abs(n[2] - maxZ) < 1e-6 and abs(n[1]) < 1e-6 and abs(n[0]) < 1e-6: 
           nRight = i
       if abs(n[2] - midZ) < 1e-6 and abs(n[1]) < 1e-6 and abs(n[0]) < 1e-6: 
           nMid = i
       #if abs(n[2] - forceZ) < 1e-6 and abs(n[1]) < 1e-6 and abs(n[0]) < 1e-6: 
       #    nForce = i
   
   
   #exu.Print("nLeft=", nLeft, ", nRight=", nRight, ", nMid=", nMid, ", nForce=", nForce)
   
   #posX=0.15 #+/- x coordinate of nodes
   posLeft = nodes[nLeft]
   posRight = nodes[nRight]
   
   nNodes = len(nodes)
   nODE2 = nNodes*3
   exu.Print("nNodes=", nNodes, ", nODE2=", nODE2)
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
   calcEig = True
   if calcEig:
       from scipy.linalg import solve, eigh, eig #eigh for symmetric matrices, positive definite
   
       [eigvals, eigvecs] = eigh(stiffnessMatrix, massMatrix) #this gives omega^2 ... squared eigen frequencies (rad/s)
       ev = np.sort(a=abs(eigvals))
   
       listEig = []
       for i in range(18):
           listEig += [np.sqrt(ev[i])/(2*np.pi)]
       exu.Print("eigenvalues =", listEig)
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #eigenvalues of constrained system:
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
   calcEigConstrained = False
   if calcEigConstrained:
   
       constrainedCoordinates = [0]*nODE2
       constrainedCoordinates[nLeft*3+0] = 1  #X
       constrainedCoordinates[nLeft*3+1] = 1  #Y
       constrainedCoordinates[nLeft*3+2] = 1  #Z
       constrainedCoordinates[nRight*3+1] = 1 #Y
       constrainedCoordinates[nRight*3+2] = 1 #Z
   
       nConstrained = sum(constrainedCoordinates)    
       indexList = []
       cnt = 0
       for i in range(nODE2):
           if constrainedCoordinates[i] == 0:
               indexList+=[i]
   
       nODE2C = nODE2-nConstrained
       massMatrixC = np.zeros((nODE2C,nODE2C))
       stiffnessMatrixC = np.zeros((nODE2C,nODE2C))
   
       for i in range(nODE2C):
           for j in range(nODE2C):
               massMatrixC[i,j] = massMatrix[indexList[i],indexList[j]]
               stiffnessMatrixC[i,j] = stiffnessMatrix[indexList[i],indexList[j]]
   
       from scipy.linalg import solve, eigh, eig #eigh for symmetric matrices, positive definite
   
       [eigvals, eigvecs] = eigh(stiffnessMatrixC, massMatrixC) #this gives omega^2 ... squared eigen frequencies (rad/s)
       ev = np.sort(a=abs(eigvals))
   
       listEig = []
       for i in range(18):
           listEig += [np.sqrt(ev[i])/(2*np.pi)]
       exu.Print("eigenvalues of constrained system (Hz)=", listEig)
   
   
   #compute (3 x 3*n) skew matrix from (3*n) vector
   def ComputeSkewMatrix(v):
       n = int(len(v)/3) #number of nodes
       sm = np.zeros((3*n,3))
   
       for i in range(n):
           off = 3*i
           x=v[off+0]
           y=v[off+1]
           z=v[off+2]
           mLoc = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
           sm[off:off+3,:] = mLoc[:,:]
   
       return sm
       #Y0=np.array([[0,0,0],[0,0,1],[0,-1,0]])
       #Y1=np.array([[0,0,-1],[0,0,0],[1,0,0]])
       #Y2=np.array([[0,1,0],[-1,0,0],[0,0,0]])
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   nNodesFFRF = nNodes
   nODE2FFRF = nNodes*3
   
   #the following is only working for useFFRFobject = True; with False, it represents an old mode, deactivated with newer ObjectGenericODE2! 
   useFFRFobject = True #uses ObjectFFRF instead of ObjectGenericODE2 ==> mesh nodes are indexed from 0 .. n_meshNodes-1
   decFFRFobject = 0    #adapt node numbers if useFFRFobject=True
   if useFFRFobject: decFFRFobject = 1
   
   useFFRF = True
   if useFFRF:
       p0 = [0,0,midZ*0] #reference position
       v0 = [0,0,0] #initial translational velocity
       omega0 = [0,0,50*2*pi] #arbitrary initial angular velocity
       ep0 = np.array(eulerParameters0) #no rotation
       ep_t0 = AngularVelocity2EulerParameters_t(omega0, ep0)
       #adjust mass and stiffness matrices
       nODE2rigid = len(p0)+len(ep0)
       nODE2rot = len(ep0) #dimension of rotation parameters
       dim3D = len(p0)     #dimension of position 
       nODE2FFRF = nODE2rigid + nODE2
       nNodesFFRF = nNodes+1
   
       Knew = np.zeros((nODE2FFRF,nODE2FFRF))
       Mnew = np.zeros((nODE2FFRF,nODE2FFRF))
   
       FillInSubMatrix(stiffnessMatrix, Knew, nODE2rigid, nODE2rigid)
       FillInSubMatrix(massMatrix, Mnew, nODE2rigid, nODE2rigid)
      
       Dnew = 2e-4*Knew #add little bit of damping
       fNew = np.zeros(nODE2FFRF)
   
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #FFRF constant matrices:
       unit3D = np.eye(3)
       Phit = np.kron(np.ones(nNodes),unit3D).T
       PhitTM = Phit.T @ massMatrix
   
       Mtt = Phit.T @ massMatrix @ Phit
       Mnew[0:3,0:3] = Mtt
       totalMass = Mtt[0,0] #must be diagonal matrix with mass in diagonal
   
       xRef = nodes.flatten() #node reference values in single vector (can be added then to q[7:])
       xRefTilde = ComputeSkewMatrix(xRef) #rfTilde without q
   
       inertiaLocal = xRefTilde.T @ massMatrix @ xRefTilde
       if False:
           exu.Print("Phit=", Phit[0:6,:])
           exu.Print("PhitTM=", PhitTM[0:3,0:6])
           exu.Print("xRef=", xRef[0:6])
           exu.Print("xRefTilde=", xRefTilde[0:6,:])
   
           exu.Print("python inertiaLocal=", inertiaLocal)
           exu.Print("python totalMass=", totalMass)
           exu.Print("python Mtt=", Mtt)
   
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #compute gravity term
       g=np.array([0,-9.81,0]) #gravity vector
       fGravRigid = list(totalMass*g)+[0,0,0,0]
       #fGrav = np.array(fGravRigid + list((massMatrix @ Phit) @ g) ) #only local vector, without rotation
       #exu.Print("fGrav=",fGrav)
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   
   #    mbs.Reset()
   #background
   rect = [-0.3,-0.1,0.3,0.1] #xmin,ymin,xmax,ymax
   background = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
   oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
   mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=p0))
   #exu.Print("goundMarker=", mGround)
   
   nodeList = []
   nRB = -1
   
   if useFFRF:
       nRB = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=p0+list(ep0), 
                                         initialVelocities=v0+list(ep_t0)))
       nodeList += [nRB]
   
   
       #adjust node numbers:
       #in all cases, triglist is same; elements = elements  + 1 - decFFRFobject #increase node numbers, because of FFRFnode
   
       #boundary nodes not adjusted for old constraints in 
       nLeft += 1
       nRight += 1
       nMid += 1
       nForce += 1
       nTopMid += 1
   
   for node in nodes:
       n3 = mbs.AddNode(Point(referenceCoordinates = list(node), visualization=VNodePoint(show = not useFFRF))) #not useFFRF)))
       nodeList += [n3]
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   
   
   #exu.Print("nForce=", nForce)
   
   #conventional user function:
   def UFforce(mbs, t, itemIndex, q, q_t):
       force = np.zeros(nODE2FFRF)
       Avec = mbs.GetNodeOutput(nRB,  exu.OutputVariableType.RotationMatrix)
       A = Avec.reshape((3,3))
   
       #implementation for Euler Parameters (Glocal_t*theta_t=0)
       ep = np.array(q[dim3D:nODE2rigid]) + ep0 #add reference values, q are only the change w.r.t. reference values!
       G = EulerParameters2GLocal(ep)
       
       cF_t = np.array(q_t[nODE2rigid:])         #velocities of flexible coordinates
   
       rF = xRef + np.array(q[nODE2rigid:]) #nodal position
   
       omega3D = G @ np.array(q_t[dim3D:nODE2rigid])
       omega3Dtilde = Skew(omega3D)
       omega = np.array(list(omega3D)*nNodes)
       omegaTilde = np.kron(np.eye(nNodes),omega3Dtilde)
   
       #squared angul. vel. matrix:
       omega3Dtilde2 = Skew(omega3D) @ Skew(omega3D)
       omegaTilde2 = np.kron(np.eye(nNodes),omega3Dtilde2)
   
       if True: #for rotordynamics, we assume rigid body motion with constant ang. vel.
           #these 2 terms are computationally costly:
           rfTilde = ComputeSkewMatrix(rF) #rfTilde
           cF_tTilde = ComputeSkewMatrix(cF_t) 
   
           fTrans = A @ (omega3Dtilde @ PhitTM @ rfTilde @ omega3D + 2*PhitTM @ cF_tTilde @ omega3D)
           force[0:dim3D] = fTrans
   
           fRot = -G.T@(omega3Dtilde @ rfTilde.T @ massMatrix @ rfTilde @ omega3D + 
                           2*rfTilde.T @ massMatrix @ cF_tTilde @ omega3D)
           force[dim3D:nODE2rigid] = fRot
       
       fFlex = -massMatrix @ (omegaTilde2 @ rF + 2*(omegaTilde @ cF_t))
       force[nODE2rigid:] = fFlex
   
       #add gravity:
       if False:
           fGrav = np.array(fGravRigid + list(PhitTM.T @ (A.T @ g)) ) #only local vector, without rotation
           force += fGrav
   
   
       return force
   
   #ffrf mass matrix:
   def UFmassGenericODE2(mbs, t, itemIndex, q, q_t):
       Avec = mbs.GetNodeOutput(nRB,  exu.OutputVariableType.RotationMatrix)
       A = Avec.reshape((3,3))
       ep = q[dim3D:nODE2rigid] + ep0 #add reference values, q are only the change w.r.t. reference values!
       G = EulerParameters2GLocal(ep)
   
       rF = xRef + q[nODE2rigid:] #nodal position
       rfTilde = ComputeSkewMatrix(rF) #rfTilde
   
       #Mtr:
       Mtr = -A @ PhitTM @ rfTilde @ G
       Mnew[0:dim3D, dim3D:dim3D+nODE2rot] = Mtr
       Mnew[dim3D:dim3D+nODE2rot, 0:dim3D] = Mtr.T
       #Mtf:
       Mtf = A @ PhitTM
       Mnew[0:dim3D, nODE2rigid:] = Mtf
       Mnew[nODE2rigid:, 0:dim3D] = Mtf.T
       #Mrf:
       Mrf = -G.T @ rfTilde.T @ massMatrix
       Mnew[dim3D:dim3D+nODE2rot, nODE2rigid:] = Mrf
       Mnew[nODE2rigid:, dim3D:dim3D+nODE2rot] = Mrf.T
       #Mrr:
       Mnew[dim3D:dim3D+nODE2rot, dim3D:dim3D+nODE2rot] = -Mrf @ rfTilde @ G   #G.T @ rfTilde.T @ massMatrix @ rfTilde @ G
   
       #exu.Print(np.linalg.norm(rF))
       #omega3D = G @ q_t[dim3D:nODE2rigid]
       #exu.Print(omega3D)
   
       #exu.Print("Mtt Mtr Mtf=",Mnew[0:3,0:10].round(5))
       #exu.Print("Mrr=",Mnew[3:7,3:7].round(5))
       #exu.Print("Mff=",Mnew[7:10,7:13].round(5))
       #Mnew[:,:] = 0 #for testing
       return Mnew
   
   
   #convert elements to triangles for drawing:
   trigList = []
   for element in elements:
       trigList += ConvertHexToTrigs(element)
   trigList = np.array(trigList) 
   #exu.Print("trig list=", trigList)
   #exu.Print("trig list size=", trigList.shape)
   
   stiffnessMatrixFF = exu.MatrixContainer()
   stiffnessMatrixFF.SetWithDenseMatrix(stiffnessMatrix,useDenseMatrix=False)
   massMatrixFF = exu.MatrixContainer()
   massMatrixFF.SetWithDenseMatrix(massMatrix,useDenseMatrix=False)
   emptyMC = exu.MatrixContainer()
   
   #add generic body for FFRF-Object:
   if useFFRFobject:
       oGenericODE2 = mbs.AddObject(ObjectFFRF(nodeNumbers = nodeList, 
                                                       #massMatrixFF=Mnew, 
                                                       stiffnessMatrixFF=stiffnessMatrixFF, 
                                                       #dampingMatrixFF=Dnew, 
                                                       massMatrixFF=massMatrixFF,
                                                       #dampingMatrixFF=emptyMC,
                                                       #forceVector=fNew, #now is a global vector!
                                                       #forceUserFunction=UFforce,
                                                       #computeFFRFterms=True,
                                                       #massMatrixUserFunction=UFmassGenericODE2,
                                                       visualization=VObjectFFRF(triangleMesh = trigList, 
                                                                                 color=graphics.color.lightred,
                                                                                 showNodes = True)))
   else:
       oGenericODE2 = mbs.AddObject(ObjectGenericODE2(nodeNumbers = nodeList, 
                                                       massMatrix=Mnew, 
                                                       stiffnessMatrix=Knew, 
                                                       dampingMatrix=Dnew, 
                                                       forceVector=fNew, forceUserFunction=UFforce,
                                                       useFirstNodeAsReferenceFrame=True, #does not exist anymore
                                                       massMatrixUserFunction=UFmassGenericODE2,
                                                       visualization=VObjectGenericODE2(triangleMesh = trigList, 
                                                                                        color=graphics.color.lightred,
                                                                                        showNodes = True)))
   
   if nODE2rot == 4: #for euler parameters --> add body to constrain EP
       epsMass = 1e-3#needed, if not all ffrf terms are included
       #add rigid body to node for Euler Parameter constraint:
       nReferenceFrame = mbs.AddObject(ObjectRigidBody(nodeNumber=nRB, physicsMass=epsMass, physicsInertia=[epsMass,epsMass,epsMass,0,0,0])) 
   
   mRB = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nRB))
   #exu.Print("rigidNodeMarker=", mRB)
   
   
   #mbs.AddLoad(Torque(markerNumber=mRB, loadVector=[0,0,100*2*pi])) #add drive for reference frame
   
   if False: #OPTIONAL: lock rigid body motion of reference frame (for tests):
       mbs.AddObject(GenericJoint(markerNumbers=[mGround, mRB], constrainedAxes=[1,1,1, 1,1,0]))
   
   
   
   #ground point:
   nGroundLeft = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,minZ], visualization = VNodePointGround(show=False))) #ground node for coordinate constraint
   nGroundRight = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,maxZ], visualization = VNodePointGround(show=False))) #ground node for coordinate constraint
   
   mGroundLeft = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGroundLeft, coordinate=0)) #Ground node ==> no action
   mGroundRight = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGroundRight, coordinate=0)) #Ground node ==> no action
   
   mGroundPosLeft = mbs.AddMarker(MarkerNodePosition(nodeNumber = nGroundLeft)) #Ground node ==> no action
   mGroundPosRight = mbs.AddMarker(MarkerNodePosition(nodeNumber = nGroundRight)) #Ground node ==> no action
   
   #exu.Print("ground Node/Coordinate Markers =", mGroundLeft, "...", mGroundPosRight)
   
   #++++++++++++++++++++++++++++++++++++++++++
   #find nodes at left and right surface:
   nodeListLeft = []
   nodeListRight = []
   
   for i in range(len(nodes)):
       n = nodes[i] 
       if abs(n[2] - minZ) < 1e-6:
           nodeListLeft += [i+useFFRF] #add 1 for rigid body node, which is first node in GenericODE2 object
       elif abs(n[2] - maxZ) < 1e-6:
           nodeListRight += [i+useFFRF]
   
   #exu.Print("nodeListLeft =",nodeListLeft)
   #exu.Print("nodeListRight =",nodeListRight)
   
   lenLeft = len(nodeListLeft)
   lenRight = len(nodeListRight)
   weightsLeft = np.array((1./lenLeft)*np.ones(lenLeft))
   weightsRight = np.array((1./lenRight)*np.ones(lenRight))
   
   #exu.Print("nodeLeft =",nLeft)
   #exu.Print("nodeRight =",nRight)
   
   #lock FFRF reference frame:
   for i in range(3):
       mLeft = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nLeft, coordinate=i))
   #    exu.Print("mLeftCoord=", mLeft)
   
       mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundLeft,mLeft]))
       if i != 2: #exclude double constraint in z-direction (axis)
           mRight = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRight, coordinate=i))
   #        exu.Print("mRightCoord=", mRight)
           mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundRight,mRight]))
   
   #lock rotation (also needed in FFRF):
   mTopRight = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nTopMid, coordinate=0)) #x-coordinate of node with y-max
   #exu.Print("mTopRight=", mTopRight)
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundRight,mTopRight]))
   
   addSupports = True
   if addSupports:
       k = 2e8
       d = k*0.01
   
       useSpringDamper = True
   
       if testMode == 0:
           raise ValueError('does not exist any more')
           # mLeft = mbs.AddMarker(MarkerGenericBodyPosition(bodyNumber=oGenericODE2, 
           #                                                 nodeNumbers=nodeListLeft, 
           #                                                 weightingFactors=weightsLeft, 
           #                                                 useFirstNodeAsReferenceFrame=useFFRF))
           # mRight = mbs.AddMarker(MarkerGenericBodyPosition(bodyNumber=oGenericODE2, 
           #                                                 nodeNumbers=nodeListRight, 
           #                                                 weightingFactors=weightsRight,
           #                                                 useFirstNodeAsReferenceFrame=useFFRF))
       else:
           mLeft = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oGenericODE2, 
                                                           meshNodeNumbers=np.array(nodeListLeft)-1, #these are the meshNodeNumbers
                                                           weightingFactors=weightsLeft))
           mRight = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oGenericODE2, 
                                                           meshNodeNumbers=np.array(nodeListRight)-1, #these are the meshNodeNumbers 
                                                           weightingFactors=weightsRight))
   
       if useSpringDamper:
           oSJleft = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mLeft, mGroundPosLeft],
                                               stiffness=[k,k,k], damping=[d,d,d]))
           oSJright = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mRight,mGroundPosRight],
                                               stiffness=[k,k,0], damping=[d,d,d]))
       else:
           oSJleft = mbs.AddObject(SphericalJoint(markerNumbers=[mGroundPosLeft,mLeft], visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))
           oSJright= mbs.AddObject(SphericalJoint(markerNumbers=[mGroundPosRight,mRight], visualization=VObjectJointSpherical(jointRadius=nodeDrawSize)))
                                                       
   
   fileDir = 'solution/'
   sDisp=mbs.AddSensor(SensorSuperElement(bodyNumber=oGenericODE2, meshNodeNumber=nMid-1, #meshnode is -1
                            storeInternal=True,#fileName=fileDir+'nMidDisplacement'+modeNames[testMode]+'test.txt', 
                            outputVariableType = exu.OutputVariableType.Displacement))
   
   
   #exu.Print(mbs)
   mbs.Assemble()
   
   #exu.Print("ltg GenericODE2 left =", mbs.systemData.GetObjectLTGODE2(oSJleft))
   #exu.Print("ltg GenericODE2 right=", mbs.systemData.GetObjectLTGODE2(oSJright))
   
   simulationSettings = exu.SimulationSettings()
   
   SC.visualizationSettings.nodes.defaultSize = nodeDrawSize
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   SC.visualizationSettings.connectors.defaultSize = 2*nodeDrawSize
   
   SC.visualizationSettings.nodes.show = True
   SC.visualizationSettings.nodes.showBasis = True #of rigid body node of reference frame
   SC.visualizationSettings.nodes.basisSize = 0.12
   SC.visualizationSettings.bodies.deformationScaleFactor = 10
   
   SC.visualizationSettings.openGL.showFaceEdges = True
   SC.visualizationSettings.openGL.showFaces = True
   
   SC.visualizationSettings.sensors.show = True
   SC.visualizationSettings.sensors.drawSimplified = False
   SC.visualizationSettings.sensors.defaultSize = 0.01
   SC.visualizationSettings.markers.drawSimplified = False
   SC.visualizationSettings.markers.show = True
   SC.visualizationSettings.markers.defaultSize = 0.01
   
   SC.visualizationSettings.loads.drawSimplified = False
   
   SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Displacement
   SC.visualizationSettings.contour.outputVariableComponent = 2 #z-component
   
   simulationSettings.solutionSettings.solutionInformation = modeNames[testMode]
   simulationSettings.solutionSettings.writeSolutionToFile=False
   
   h=1e-4
   tEnd = 0.001
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = h
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   #simulationSettings.timeIntegration.newton.maxModifiedNewtonIterations = 10
   #simulationSettings.timeIntegration.newton.modifiedNewtonJacUpdatePerStep = True #this improves the FFRF simulation slightly
   
   simulationSettings.solutionSettings.sensorsWritePeriod = h
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 #SHOULD work with 0.9 as well
   #simulationSettings.displayStatistics = True
   #simulationSettings.displayComputationTime = True
   
   #create animation:
   #simulationSettings.solutionSettings.recordImagesInterval = 0.0002
   #SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
   
   if useGraphics:
       SC.renderer.Start()
       if 'lastRenderState' in vars():
           SC.renderer.SetState(lastRenderState) #load last model view
       
       SC.renderer.DoIdleTasks() #press space to continue
   
   mbs.SolveDynamic(simulationSettings)
   
   data = mbs.GetSensorStoredData(sDisp)
   #data = np.loadtxt(fileDir+'nMidDisplacement'+modeNames[testMode]+'test.txt', comments='#', delimiter=',')
   result = abs(data).sum()
   #pos = mbs.GetObjectOutputBody(objFFRF['oFFRFreducedOrder'],exu.OutputVariableType.Position, localPosition=[0,0,0])
   exu.Print('solution of ObjectFFRF=',result)
   
   exudynTestGlobals.testError = result - (0.0064600108120842666) #2022-02-20 (changed to internal sensor data); 2020-05-17 (tEnd=0.001, h=1e-4): 0.006445369560936511
   exudynTestGlobals.testResult = result#0.006460010812070858 
   
       
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
       lastRenderState = SC.renderer.GetState() #store model view for next simulation
   
   ##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
   #plot results
   cList=['r-','g-','b-','k-','c-','r:','g:','b:','k:','c:']
   if useGraphics:
       
       
       mbs.PlotSensor(sDisp, components=0, closeAll=True)
   
   


