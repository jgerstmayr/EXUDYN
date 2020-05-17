#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  support functions for finite element models
#
# Author:   Johannes Gerstmayr
# Date:     2020-03-10
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# Note that CSR matrices contain 3 numbers per row: [row, column, value]
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++import sys

#constants and fixed structures:
from itemInterface import *
from exudynUtilities import RoundMatrix, ComputeSkewMatrix, FillInSubMatrix
from exudynRigidBodyUtilities import AngularVelocity2EulerParameters_t, EulerParameters2GLocal, Skew
import numpy as np #LoadSolutionFile

#convert zero-based sparse matrix data to dense numpy matrix
#DEPRECTAED!!!!!!!!!!!!!!!!
def CompressedRowToDenseMatrix(sparseData):
    print("\n**************************\nWARNING: CompressedRowToDenseMatrix is deprecated!\n**************************\n")
    n = int(max(np.max(sparseData[:,0]),np.max(sparseData[:,1]))) #rows and columns are 1-based
    m = np.zeros((n,n))
    for row in sparseData:
        m[int(row[0])-1,int(row[1])-1] = row[2] #convert 1-based to 0-based
    return m

#convert zero-based sparse matrix data to dense numpy matrix
def CompressedRowSparseToDenseMatrix(sparseData):
    #does not work, if there are no entry in highest rows and columns
    n = int(max(np.max(sparseData[:,0]),np.max(sparseData[:,1])))+1 #rows and columns indices are 0-based ==> add 1 for size!
    m = np.zeros((n,n))
    for row in sparseData:
        m[int(row[0]),int(row[1])] = row[2]
    return m

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#compute diadic product of vector v and a 3D unit matrix =^= diadic(v,I_3x3); used for ObjectFFRF and CMS implementation
def VectorDiadicUnitMatrix3D(v):
    return np.kron(np.array(v), np.eye(3)).T

#print("diadicTest=", VectorDiadicUnitMatrix3D(np.array([1,2,3,4,5,6])))

#add entry to compressedRowSparse matrix, avoiding duplicates
#value is either added to existing entry (avoid duplicates) or a new entry is appended
def AddEntryToCompressedRowSparseArray(sparseData, row, column, value):
    n = len(sparseData[:,0])
    for i in range(n):
        if sparseData[i,0] == row and sparseData[i,1] == column:
            sparseData[i,2] += value
            return
        if sparseData[i,0] > row:
            np.insert(sparseData, i, np.array((row, column, value)), 0)
            return
    #insert at end of matrix:
    np.insert(sparseData, n, np.array((row, column, value)), 0)
    return

#compute rows and columns of a compressed sparse matrix and return as tuple: (rows,columns)
def CSRtoRowsAndColumns(sparseMatrixCSR):
    rows = sparseMatrixCSR[:,0].max()
    columns = sparseMatrixCSR[:,1].max()
    return (rows, columns)

#convert internal compressed CSR to scipy.sparse csr matrix
def CSRtoScipySparseCSR(sparseMatrixCSR):
    from scipy.sparse import csr_matrix
    X = csr_matrix((sparseMatrixCSR[:,2],(sparseMatrixCSR[:,0].astype(int),sparseMatrixCSR[:,1].astype(int))))
    #X.sum_duplicates() 
    return X

#read abaqus nodes information to numpy array
#typeName is Part or Instance; name is part's or instance's name
#if exportElement=False: returns np.array(nodes) with nodal coordinates
#if exportElements=True: returns a list [np.array(nodes), elementsDict, surfaceElementsDict] with information on types of elements
def ReadNodesFromAbaqusInp(fileName, typeName='Part', name='Part-1', exportElements=False):
    print("\n********************WARNING:\nFUNCTION ReadNodesFromAbaqusInp is deprecated; use FEMinterface!\n********************\n")
    fileLines = []
    try: #still close file if crashes
        file=open(fileName,'r') 
        fileLines = file.readlines()
    finally:
        file.close()
        
        
    print("read ", len(fileLines), "lines")
    
    startPart = False
    startReadNodes = False
    finishedReadNodes = False
    startReadElements = False
    finishedReadElements = False
    nodes = [] #store list of node values
    elements = [] #store list of elements with node numbers
    surfaceElements = [] #store list of surface elements (trigs, quads) with node numbers
    elementTypes = [] #string list of element types
    surfaceElementTypes = [] #string list of surface element types

    errorOccured = False
    lineCnt = 0
    for line in fileLines:
        #print("line", lineCnt, "=", line)
        lineCnt+=1
        if errorOccured:
            break
        
        if startReadNodes and not finishedReadNodes:
            if line[0] != '*': #check if nodes section has finished
                lineData = line.split(',') #split into values
                if len(lineData) != 4:
                    print("ERROR: Expected node number and 3 coordinates, line ", lineCnt)
                    errorOccured = True
                else:
                    v = []
                    for i in range(3):
                        v+=[float(lineData[i+1])]
                    nodes += [v] #add node data
            else:
                startReadNodes = False
                finishedReadNodes = True
    
        if startReadElements and not finishedReadElements:
            if line[0] != '*': #check if nodes section has finished
                lineData = line.split(',') #split into values
                if len(lineData) != 9:
                    print("ERROR: Expected element number and 8 indices for C3D8R, line=", lineCnt)
                    errorOccured = True
                else:
                    v = []
                    for i in range(8):
                        v+=[float(lineData[i+1])]
                    elements += [v] #add node data
                    elementTypes += ['Hex8'] #8-noded hexahedral
            else:
                startReadElements = False
                finishedReadElements = True
    
        if startPart and not startReadNodes and not finishedReadNodes:
            if line[0:5] == '*Node':
                startReadNodes = True
                startPart = False
            else:
                print("ERROR: Expected *Node after *Part, line=", lineCnt)
                errorOccured = True

        if finishedReadNodes and exportElements and not startReadElements:
            if line[0:8] == '*Element':
                startReadElements = True
                #check "type=C3D8R" in future

            
        if line[0:len(typeName)+1] == '*'+typeName:
            if not startPart:
                #print("ERROR: only one *Part section allowed, line=", lineCnt)
                #errorOccured = True
                
                lineInfo = line.split(',')
                #print(lineInfo)
                if len(lineInfo) != 3:
                    print("ERROR: invalid information for part/instance name, line=", lineCnt)
                    errorOccured = True
                else:
                    nameInfo = lineInfo[1].strip().split('=')
                    if nameInfo[0] != 'name':
                        print("ERROR: expected 'name=' in line=", lineCnt)
                        errorOccured = True
                    else:
                        if nameInfo[1] != name:
                            print("ERROR: expected name='" + name + "' in line=", lineCnt)
                            errorOccured = True
                        else:
                            startPart = True
    
    if not exportElements:
        return np.array(nodes)
    else:
        elementsDict = {'elements':np.array(elements), 'elementTypes':elementTypes}
        surfaceElementsDict = {'elements':np.array(surfaceElements), 'elementTypes':surfaceElementTypes}
        return [np.array(nodes), elementsDict, surfaceElementsDict]




#convert list of C3D8 Hex element with 8 nodes in nodeNumbers into triangle-List
def ConvertHexToTrigs(nodeNumbers):
    localList = [[0,1,2], [0,2,3], [6,5,4], [6,4,7], [0,4,1], [1,4,5], [1,5,2], [2,5,6], [2,6,3], [3,6,7], [3,7,0], [0,7,4]]

    trigList = []
    for trig in localList:
        ind = [nodeNumbers[trig[0]], nodeNumbers[trig[1]], nodeNumbers[trig[2]]]
        trigList += [ind]
    return trigList








#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++          ObjectFFRFreducedOrderTerms              ++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#compute terms necessary for ObjectFFRFreducedOrder
#class used internally in FEMinterface to compute ObjectFFRFreducedOrder dictionary
#this class holds all data for ObjectFFRFreducedOrder user functions
class ObjectFFRFreducedOrderInterface:
    def __init__(self, femInterface, roundMassMatrix = 1e-13, roundStiffNessMatrix = 1e-13):
 
        self.modeBasis = femInterface.modeBasis['matrix']
        nodeArray = femInterface.GetNodePositionsAsArray()
        self.trigList = femInterface.GetSurfaceTriangles()

        stiffnessMatrixCSR = CSRtoScipySparseCSR(femInterface.GetStiffnessMatrix(sparse=True))
        massMatrixCSR = CSRtoScipySparseCSR(femInterface.GetMassMatrix(sparse=True))

        #compute reduced mass and stiffness matrices, only flexible coordinates:
        self.massMatrixReduced = self.modeBasis.T @ massMatrixCSR @ self.modeBasis #LARGE MATRIX COMPUTATION
        self.stiffnessMatrixReduced = self.modeBasis.T @ stiffnessMatrixCSR @ self.modeBasis #LARGE MATRIX COMPUTATION
        RoundMatrix(self.massMatrixReduced, 1e-13*abs(self.massMatrixReduced).max()) #erase off-diagonal terms for higher efficiency ...
        RoundMatrix(self.stiffnessMatrixReduced, 1e-13*abs(self.stiffnessMatrixReduced).max())

        #new coordinates:
        #nODE2 = nNodes*3        #non reduced, full coordinates
        #nODE2reduced = nModes
        self.nModes = self.modeBasis.shape[1]                 #number of columns in self.modeBasis is the number of modes to consider
        self.nNodes = len(nodeArray)                #stored in nNodes x 3 np-array
        self.dim3D = len(nodeArray[0])              #dimension of position, assuming that one node exists ....
        self.nODE2rot = 4                           #dimension of rotation parameters; fixed to 4 for now!
        self.nODE2rigid = self.dim3D + self.nODE2rot
        self.nODE2FFRFreduced = self.nODE2rigid + self.nModes

        self.massMatrixFFRFreduced = np.zeros((self.nODE2FFRFreduced,self.nODE2FFRFreduced)) #create larger FFRF mass matrix

        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #FFRFreduced constant matrices:
        self.Phit = np.kron(np.ones(self.nNodes),np.eye(3)).T
        self.PhitTM = self.Phit.T @ massMatrixCSR #LARGE MATRIX COMPUTATION
        self.xRef = nodeArray.flatten()          #node reference values in single vector (can be added then to q[7:])
        self.xRefTilde = ComputeSkewMatrix(self.xRef) #rfTilde without q    

        #not needed, but may be interesting for checks:
        self.inertiaLocal = self.xRefTilde.T @ massMatrixCSR @ self.xRefTilde #LARGE MATRIX COMPUTATION
        #if False:
        #    print("Phit=", self.Phit[0:6,:])
        #    print("PhitTM=", self.PhitTM[0:3,0:6])
        #    print("xRef=", self.xRef[0:6])
        #    print("xRefTilde=", self.xRefTilde[0:6,:])


        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #prepare CMS matrices (some already given):
        #according to: A. Zwölfer, J. Gerstmayr. The nodal-based floating frame of reference formulation with modal reduction, Acta Mechanica, 2020, submitted.
        #\bar x = xRef
        #\tilde \bar x = xRefTilde
        #\bar Theta = inertiaLocal
        #\bar mPsiPsi = massMatrixReduced
        #Psi = self.modeBasis
        self.Mtt = self.PhitTM @ self.Phit
        self.totalMass = self.Mtt[0,0]                #Mtt must be diagonal matrix with mass in diagonal
        self.PsiTilde = ComputeSkewMatrix(self.modeBasis)
        self.chiU = 1./self.totalMass*(self.PhitTM @ self.xRef)                 #center of mass
        self.chiUtilde = ComputeSkewMatrix(self.chiU)
        self.mPsiTildePsi = self.PsiTilde.T @ massMatrixCSR @ self.modeBasis         #LARGE MATRIX COMPUTATION
        self.mPsiTildePsiTilde = self.PsiTilde.T @ massMatrixCSR @ self.PsiTilde#LARGE MATRIX COMPUTATION
        self.mPhitTPsi = self.Phit.T @ massMatrixCSR @ self.modeBasis                #LARGE MATRIX COMPUTATION
        self.mPhitTPsiTilde = self.Phit.T @ massMatrixCSR @ self.PsiTilde       #LARGE MATRIX COMPUTATION
        self.mXRefTildePsi = self.xRefTilde.T @ massMatrixCSR @ self.modeBasis       #LARGE MATRIX COMPUTATION
        self.mXRefTildePsiTilde = self.xRefTilde.T @ massMatrixCSR @ self.PsiTilde    #LARGE MATRIX COMPUTATION

        #fill already parts of the mass matrix in order to avoid copying this constant data during user function calls:
        self.massMatrixFFRFreduced[0:3,0:3] = self.Mtt
        #FillInSubMatrix(self.massMatrixReduced, self.massMatrixFFRFreduced, self.nODE2rigid, self.nODE2rigid)
        self.massMatrixFFRFreduced[self.nODE2rigid:,self.nODE2rigid:] = self.massMatrixReduced

    #add object to mbs as well as according nodes
    def AddObjectFFRFreducedOrderWithUserFunctions(self, exu, mbs, 
                                                  positionRef=[0,0,0], eulerParametersRef=[1,0,0,0], 
                                                  initialVelocity=[0,0,0], initialAngularVelocity=[0,0,0],
                                                  gravity=[0,0,0],
                                                  UFforce=0, UFmassMatrix=0,
                                                  color=[0.1,0.9,0.1,1.]):

        self.gravity = gravity

        #compute initial euler parameter velocities from angular velocity vector
        self.eulerParameters_t0 = AngularVelocity2EulerParameters_t(initialAngularVelocity, eulerParametersRef)
        self.eulerParameters0 = eulerParametersRef

        #rigid body node for ObjectFFRFreducedOrder
        self.nRigidBody = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=list(positionRef)+list(eulerParametersRef), 
                                            initialVelocities=list(initialVelocity)+list(self.eulerParameters_t0)))
        #generic node for modal coordinates in ObjectFFRFreducedOrder
        self.nGenericODE2 = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=self.nModes,
                                          referenceCoordinates=[0]*self.nModes,
                                          initialCoordinates=[0]*self.nModes,
                                          initialCoordinates_t=[0]*self.nModes))

        stiffnessMatrixMC = exu.MatrixContainer()
        stiffnessMatrixMC.SetWithDenseMatrix(self.stiffnessMatrixReduced,useDenseMatrix=False)
        massMatrixMC = exu.MatrixContainer()
        factMass = 1.
        if UFmassMatrix != 0:
            factMass = 0.
        massMatrixMC.SetWithDenseMatrix(factMass*self.massMatrixReduced,useDenseMatrix=False)
        emptyMC = exu.MatrixContainer()

        #add generic body for FFRF-Object:
        self.oFFRFreducedOrder = mbs.AddObject(ObjectFFRFreducedOrder(nodeNumbers = [self.nRigidBody, self.nGenericODE2], 
                                                            stiffnessMatrixReduced=stiffnessMatrixMC, 
                                                            massMatrixReduced=massMatrixMC,
                                                            modeBasis=self.modeBasis,
                                                            referencePositions = self.xRef,
                                                            forceUserFunction=UFforce,
                                                            massMatrixUserFunction=UFmassMatrix,
                                                            visualization=VObjectFFRF(triangleMesh = self.trigList, 
                                                                                        color=color,
                                                                                        showNodes = True)))
        dictReturn = {'nRigidBody':self.nRigidBody,
                      'nGenericODE2':self.nGenericODE2,
                      'oFFRFreducedOrder':self.oFFRFreducedOrder}

        if self.nODE2rot == 4: #for euler parameters --> add body to constrain EP
            epsMass = 1e-3#needed, if not all ffrf terms are included
            #add rigid body to node for Euler Parameter constraint:
            oAddedBody = mbs.AddObject(ObjectRigidBody(nodeNumber=self.nRigidBody, physicsMass=epsMass, 
                                                       physicsInertia=[epsMass,epsMass,epsMass,0,0,0])) 
            dictReturn['oAddedBody'] = oAddedBody

        return dictReturn



    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #CMS mass matrix user function; qReduced contains the rigid body node and the modal coordinates in one vector!
    def UFmassFFRFreducedOrder(self, exu, mbs, t, qReduced, qReduced_t):

        Avec = mbs.GetNodeOutput(self.nRigidBody,  exu.OutputVariableType.RotationMatrix)
        A = Avec.reshape((3,3))
        ep = np.array(qReduced[self.dim3D:self.nODE2rigid]) + np.array(self.eulerParameters0) #add reference values, q are only the change w.r.t. reference values!
        G = EulerParameters2GLocal(ep)

        zetaI = VectorDiadicUnitMatrix3D(qReduced[self.nODE2rigid:])
        zeta_tI = VectorDiadicUnitMatrix3D(qReduced_t[self.nODE2rigid:])

        #Mtt and Mff already filled into massMatrixFFRFreduced
        #Mtr:
        Mtr = -A @ (self.totalMass*self.chiUtilde + self.mPhitTPsiTilde @ zetaI) @ G
        self.massMatrixFFRFreduced[0:self.dim3D, self.dim3D:self.dim3D+self.nODE2rot] = Mtr
        self.massMatrixFFRFreduced[self.dim3D:self.dim3D+self.nODE2rot, 0:self.dim3D] = Mtr.T

        #Mtf:
        Mtf = A @ self.mPhitTPsi
        self.massMatrixFFRFreduced[0:self.dim3D, self.nODE2rigid:] = Mtf 
        self.massMatrixFFRFreduced[self.nODE2rigid:, 0:self.dim3D] = Mtf.T

        #Mrf:
        Mrf = -G.T @ (self.mXRefTildePsi + zetaI.T @ self.mPsiTildePsi)
        self.massMatrixFFRFreduced[self.dim3D:self.nODE2rigid, self.nODE2rigid:] = Mrf
        self.massMatrixFFRFreduced[self.nODE2rigid:, self.dim3D:self.nODE2rigid] = Mrf.T

        #Mrr:
        self.massMatrixFFRFreduced[self.dim3D:self.nODE2rigid, self.dim3D:self.nODE2rigid] = G.T@(self.inertiaLocal + self.mXRefTildePsiTilde @ zetaI + 
                                                                            zetaI.T @ self.mXRefTildePsiTilde.T + 
                                                                            zetaI.T @ self.mPsiTildePsiTilde @ zetaI)@G

        return self.massMatrixFFRFreduced

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #CMS force user function; qReduced contains the rigid body node and the modal coordinates in one vector!
    def UFforceFFRFreducedOrder(self, exu, mbs, t, qReduced, qReduced_t):
        force = np.zeros(self.nODE2FFRFreduced)

        Avec = mbs.GetNodeOutput(self.nRigidBody,  exu.OutputVariableType.RotationMatrix)
        A = Avec.reshape((3,3))
        ep = np.array(qReduced[self.dim3D:self.nODE2rigid]) + np.array(self.eulerParameters0) #add reference values, q are only the change w.r.t. reference values!
        G = EulerParameters2GLocal(ep)
        if len(ep) != 4: 
            print("ERROR: equations only implemented for Euler parameters case (terms missing for other formulations)"); exit()

        zetaI = VectorDiadicUnitMatrix3D(qReduced[self.nODE2rigid:])
        zeta_tI = VectorDiadicUnitMatrix3D(qReduced_t[self.nODE2rigid:])

        omega3D = G @ np.array(qReduced_t[self.dim3D:self.nODE2rigid])
        omega3Dtilde = Skew(omega3D)
        IZetadiadicOmega = np.kron(np.eye(self.nModes), np.array(omega3D)).T

        qReducedFF = np.array(qReduced[self.nODE2rigid:])
        qReduced_tFF = np.array(qReduced_t[self.nODE2rigid:])

        #CMS:
        force[0:self.dim3D] = (A @ omega3Dtilde @ (self.totalMass*self.chiUtilde + self.mPhitTPsiTilde @ zetaI) @ omega3D + 
                               2*A @ self.mPhitTPsiTilde @ zeta_tI @ omega3D) #identical to FFRF up to 1e-16
    
        force[self.dim3D:self.nODE2rigid] = (-G.T @ omega3Dtilde @ (self.inertiaLocal + self.mXRefTildePsiTilde @ zetaI + 
                                                                    zetaI.T @ self.mXRefTildePsiTilde.T + 
                                                                    zetaI.T @ self.mPsiTildePsiTilde @ zetaI ) @ omega3D - 
                                   2*G.T @ (self.mXRefTildePsiTilde @ zeta_tI + zetaI.T @ self.mPsiTildePsiTilde @ zeta_tI ) @ omega3D)  #identical to FFRF up to 1e-16
        
        force[self.nODE2rigid:] = (IZetadiadicOmega.T @ (self.mXRefTildePsiTilde.T + self.mPsiTildePsiTilde @ zetaI) @ omega3D + 
                                   2 * self.mPsiTildePsi.T @ zeta_tI @ omega3D) #identical to FFRF up to 1e-16

        #add gravity:
        if False:
            force[0:self.dim3D] += totalMass*np.array(self.gravity)
            force[self.nODE2rigid:] += self.mPhitTPsi.T @ (A.T @ g)

            #force[nODE2rigid:] += modeBasis.T @ PhitTM.T @ (A.T @ g)

        return force

        


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++   FEMinterface - finite element interface class   ++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#general interface to different FEM / mesh imports and export to EXUDYN functions
class FEMinterface:
    def __init__(self):
        self.nodes = {}                 # {'Position':[[x0,y0,z0],...], 'RigidBodyRxyz':[[x0,y0,z0],...],  },...]                     #dictionary of different node lists
        self.elements = []              # [{'Name':'identifier', 'Tet4':[[n0,n1,n2,n3],...], 'Hex8':[[n0,...,n7],...],  },...]        #there may be several element sets
        #self.massMatrix = {}           # {'Shape':[rows,columns], 'SparseCSR':[[r0,c0,value0],[r1,c1,value1], ... ],  }             #currently only in SparseCSR format allowed!
        self.massMatrix = np.zeros((0,0))    # np.array([[r0,c0,value0],[r1,c1,value1], ... ])                                #currently only in SparseCSR format allowed!
        self.stiffnessMatrix=np.zeros((0,0)) # np.array([[r0,c0,value0],[r1,c1,value1], ... ])                                #currently only in SparseCSR format allowed!
        self.surface = []               # [{'Name':'identifier', 'Trigs':[[n0,n1,n2],...], 'Quads':[[n0,...,n3],...],  },...]           #surface with faces
        self.nodeSets = []              # [{'Name':'identifier', 'NodeNumbers':[n_0,...,n_ns], 'NodeWeights':[w_0,...,w_ns]},...]     #for boundary conditions, etc.
        self.elementSets = []           # [{'Name':'identifier', 'ElementNumbers':[n_0,...,n_ns]},...]                                #for different volumes, etc.

        self.modeBasis = {}             # {'matrix':[[Psi_00,Psi_01, ..., Psi_0m],...,[Psi_n0,Psi_n1, ..., Psi_nm]],'type':'NormalModes'}
        self.eigenValues = []           # [ev0, ev1, ...]                                                                             #eigenvalues according to eigenvectors in mode basis
        self.ffrfReducedOrderTerms = () # 

        #some additional information, needed for checks and easier operation
        self.coordinatesPerNodeType = {'Position':3, 'Position2D':2, 'RigidBodyRxyz':6, 'RigidBodyEP':7} #number of coordinates for a certain node type

    #import Abaqus input file 
    #node numbers in elements are converted from 1-based indices to python's 0-based indices
    #return node numbers
    def ImportFromAbaqusInputFile(self, fileName, typeName='Part', name='Part-1'):
        fileLines = []
        print("read file name=", fileName)
        file=open(fileName,'r') 
        fileLines = file.readlines()
        file.close()
        
        print("read ", len(fileLines), "lines")
    
        startPart = False
        startReadNodes = False
        finishedReadNodes = False
        startReadElements = False
        finishedReadElements = False
        nodes = [] #store list of node values
        elements = [] #store list of elements with node numbers
        elementTypes = [] #string list of element types

        errorOccured = False
        lineCnt = 0
        for line in fileLines:
            #print("line", lineCnt, "=", line)
            lineCnt+=1
            if errorOccured:
                break
        
            if startReadNodes and not finishedReadNodes:
                if line[0] != '*': #check if nodes section has finished
                    lineData = line.split(',') #split into values
                    if len(lineData) != 4:
                        print("ERROR: Expected node number and 3 coordinates, line ", lineCnt)
                        errorOccured = True
                    else:
                        v = []
                        for i in range(3):
                            v+=[float(lineData[i+1])]
                        nodes += [v] #add node data
                else:
                    startReadNodes = False
                    finishedReadNodes = True
    
            if startReadElements and not finishedReadElements:
                if line[0] != '*': #check if nodes section has finished
                    lineData = line.split(',') #split into values
                    if len(lineData) != 9:
                        print("ERROR: Expected element number and 8 indices for C3D8R, line=", lineCnt)
                        errorOccured = True
                    else:
                        v = []
                        for i in range(8):
                            v+=[float(lineData[i+1])]
                        elements += [v] #add node data
                        elementTypes += ['Hex8'] #8-noded hexahedral
                else:
                    startReadElements = False
                    finishedReadElements = True
    
            if startPart and not startReadNodes and not finishedReadNodes:
                if line[0:5] == '*Node':
                    startReadNodes = True
                    startPart = False
                else:
                    print("ERROR: Expected *Node after *Part, line=", lineCnt)
                    errorOccured = True

            if finishedReadNodes and not startReadElements:
                if line[0:8] == '*Element':
                    startReadElements = True
                    #check "type=C3D8R" in future

            
            if line[0:len(typeName)+1] == '*'+typeName:
                if not startPart:
                    #print("ERROR: only one *Part section allowed, line=", lineCnt)
                    #errorOccured = True
                
                    lineInfo = line.split(',')
                    #print(lineInfo)
                    if len(lineInfo) != 3:
                        print("ERROR: invalid information for part/instance name, line=", lineCnt)
                        errorOccured = True
                    else:
                        nameInfo = lineInfo[1].strip().split('=')
                        if nameInfo[0] != 'name':
                            print("ERROR: expected 'name=' in line=", lineCnt)
                            errorOccured = True
                        else:
                            if nameInfo[1] != name:
                                print("ERROR: expected name='" + name + "' in line=", lineCnt)
                                errorOccured = True
                            else:
                                startPart = True

        self.nodes['Position'] = np.array(nodes)
        elements = np.array(elements)-1 #convert node indices from 1 to 0-based

        elementsDict = {'Name':name, 'Tet4':[], 'Hex8':[]}
        for i in range(len(elements)):
            if elementTypes[i] in elementsDict:
                elementsDict[elementTypes[i]] += [elements[i]]
            else:
                print("FEMinterface.ImportFromAbaqusInputFile: unknown elementType "+elementTypes[i]+", ignored")

        #convert elements to triangles for drawing:
        trigList = []
        for element in elements:
            trigList += ConvertHexToTrigs(element) #node numbers are already 0-based
        trigList = trigList
        self.surface += [{'Name':'meshSurface', 'Trigs':trigList}]    # [{'Name':'identifier', 'Trigs':[[n0,n1,n2],...], 'Quads':[[n0,...,n3],...],  },...]           #surface with faces

        return np.array(nodes)

    #read mass matrix from compressed row text format (exported from Abaqus)
    def ReadMassMatrixFromAbaqus(self, fileName, type='SparseRowColumnValue'):
        self.massMatrix = np.loadtxt(fileName)
        self.massMatrix[:,0] -= 1 #convert 1-based indices to 0-based indices
        self.massMatrix[:,1] -= 1

    #read stiffness matrix from compressed row text format (exported from Abaqus)
    def ReadStiffnessMatrixFromAbaqus(self, fileName, type='SparseRowColumnValue'):
        self.stiffnessMatrix = np.loadtxt(fileName)
        self.stiffnessMatrix[:,0] -= 1 #convert 1-based indices to 0-based indices
        self.stiffnessMatrix[:,1] -= 1

    #get sparse mass matrix in according format
    def GetMassMatrix(self, sparse=True):
        if sparse:
            return self.massMatrix
        else:
            return CompressedRowSparseToDenseMatrix(self.massMatrix)

    #get sparse stiffness matrix in according format
    def GetStiffnessMatrix(self, sparse=True):
        if sparse:
            return self.stiffnessMatrix
        else:
            return CompressedRowSparseToDenseMatrix(self.stiffnessMatrix)

    #scale (=multiply) mass matrix with factor
    def ScaleMassMatrix(self, factor):
        self.massMatrix[:,2] *= factor

    #scale (=multiply) stiffness matrix with factor
    def ScaleStiffnessMatrix(self, factor):
        self.stiffnessMatrix[:,2] *= factor

    #get total number of nodes
    def NumberOfNodes(self):
        nNodes = 0
        for nodeTypeName in self.nodes:
            nNodes += len(self.nodes[nodeTypeName])
        return nNodes

    #get node points as array; only possible, if there exists only one type of Position nodes
    def GetNodePositionsAsArray(self):
        if len(self.nodes) != 1:
            raise ValueError("ERROR: GetNodePositionsAsArray() only possible for one type of Position nodes!")

        nodeTypeName = list(self.nodes)[0]
        return self.nodes[nodeTypeName]

    #get number of total nodal coordinates
    def NumberOfCoordinates(self):
        nCoordinates = 0
        for nodeTypeName in self.nodes:
            nCoordinates += len(self.nodes[nodeTypeName]) * self.coordinatesPerNodeType[nodeTypeName]
        return nCoordinates

    #get node number for node at given point, e.g. p=[0.1,0.5,-0.2], using a tolerance (+/-) if coordinates are available only with reduced accuracy
    #if not found, it returns an invalid index
    def GetNodeAtPoint(self, point, tolerance = 1e-5, raiseException = True):
        cnt = 0
        for nodeTypeName in self.nodes:
            for nodePoint in self.nodes[nodeTypeName]:
                if abs(nodePoint - point).max() <= tolerance:
                    return cnt #this holds the node number
                cnt+=1
        
        if raiseException:
            raise ValueError("ERROR: GetNodeAtPoint: node point not found!")
        return -1 #invalid index, only if no exception raised

    #get node numbers in plane defined by point p and (normalized) normal vector n using a tolerance for the distance to the plane
    #if not found, it returns an empty list
    def GetNodesInPlane(self, point, normal,  tolerance = 1e-5):
        cnt = 0
        nodeList=[]
        for nodeTypeName in self.nodes:
            for nodePoint in self.nodes[nodeTypeName]:
                if abs(np.dot(nodePoint - point, normal)) <= tolerance:
                    nodeList += [cnt]
                cnt+=1
        return nodeList

    #return surface trigs as node number list (for drawing in EXUDYN)
    def GetSurfaceTriangles(self):
        #self.surface += [{'Name':'meshSurface', 'Trigs':trigList}]    # [{'Name':'identifier', 'Trigs':[[n0,n1,n2],...], 'Quads':[[n0,...,n3],...],  },...]           #surface with faces
        trigList = []
        for surface in self.surface:
            if 'Trigs' in surface:
                trigList += surface['Trigs']
        return trigList

    #modify stiffness matrix to add elastic support (joint, etc.) to a node; nodeNumber zero based (as everywhere in the code...)
    #springStiffness must have length according to the node size
    def AddElasticSupportAtNode(self, nodeNumber, springStiffness=[1e8,1e8,1e8]):
        if len(self.nodes) != 1:
            print("ERROR: AddElasticSupportAtNode: there must be exactly one list of nodes!")
        #nodeList = self.nodes(list(self.nodes)[0])
        nodeTypeName = list(self.nodes)[0]
        nodeSize = self.coordinatesPerNodeType[nodeTypeName]
        nCoordinate = nodeNumber * nodeSize
        #supports = []
        for i in range(nodeSize):
            #supports += [[nCoordinate+i,nCoordinate+i,springStiffness[i]]]
            AddEntryToCompressedRowSparseArray(self.stiffnessMatrix, nCoordinate+i,nCoordinate+i,springStiffness[i])
        #np.vstack((self.stiffnessMatrix, np.array(supports))) #append supports to sparse matrix

    #modify mass matrix by adding a mass to a certain node, modifying directly the mass matrix
    def AddNodeMass(self, nodeNumber, addedMass):
        if len(self.nodes) != 1:
            print("ERROR: AddElasticSupportAtNode: there must be exactly one list of nodes!")

        #nodeList = self.nodes(list(self.nodes)[0])
        nodeTypeName = list(self.nodes)[0]
        nodeSize = self.coordinatesPerNodeType[nodeTypeName]

        nCoordinate = nodeNumber * nodeSize
        supports = []
        for i in range(nodeSize):
            #supports += [[nCoordinate+i,nCoordinate+i,addedMass]]
            AddEntryToCompressedRowSparseArray(self.massMatrix, nCoordinate+i,nCoordinate+i,addedMass)
        #np.vstack((self.massMatrix, np.array(supports))) #append supports to sparse matrix

    #compute nModes smallest eigenvalues and eigenmodes from mass and stiffnessMatrix
    #store mode vector in modeBasis, but exclude 'excludeRigidBodyModes' rigid body modes from modeBasis
    #if excludeRigidBodyModes > 0, then the computed modes is nModes + excludeRigidBodyModes, from which excludeRigidBodyModes smallest eigenvalues are excluded
    def ComputeEigenmodes(self, nModes, excludeRigidBodyModes = 6, useSparseSolver = True):
        if not useSparseSolver:
            #unsorted, dense eigen vectors
            from scipy.linalg import solve, eigh, eig #eigh for symmetric matrices, positive definite

            K = self.GetStiffnessMatrix(sparse=False)
            M = self.GetMassMatrix(sparse=False)

            [eigVals, eigVecs] = eigh(K,M) #this gives omega^2 ... squared eigen frequencies (rad/s)
            self.modeBasis = {'matrix':eigVecs[:,excludeRigidBodyModes:excludeRigidBodyModes + nModes], 'type':'NormalNodes'}
            self.eigenValues = abs(eigVals)
        else:
            #sorted, sparse eigen vectors
            from scipy.sparse.linalg import eigsh #eigh for symmetric matrices, positive definite

            K = CSRtoScipySparseCSR(self.GetStiffnessMatrix(sparse=True))
            M = CSRtoScipySparseCSR(self.GetMassMatrix(sparse=True))
            n=self.NumberOfCoordinates()
            #optional, using shift-invert mode; DOES NOT WORK:
            #guess for smallest eigenvalue:
            #mMax = self.GetMassMatrix(sparse=True)[:,2].sum()/3 #take total mass
            #kMax = self.GetStiffnessMatrix(sparse=True)[:,2].max()   #assume only one node fixed
            #omegaMin = kMax/mMax*0.1 #factor 0.1 in order to make guess not too large
            #print("min freq=", np.sqrt(omegaMin)/(2*np.pi))
            #[eigVals, eigVecs] = eigsh(A=K, k=nModes+excludeRigidBodyModes, M=M, which='SM', sigma=omegaMin) #this gives omega^2 ... squared eigen frequencies (rad/s)

            #use "LM" (largest magnitude), but shift-inverted mode with sigma=0, to find the zero-eigenvalues:
            #see https://docs.scipy.org/doc/scipy/reference/tutorial/arpack.html
            [eigVals, eigVecs] = eigsh(A=K, k=nModes+excludeRigidBodyModes, M=M, 
                                       which='LM', sigma=0, mode='normal') #try modes 'normal','buckling' and 'cayley'
            
            #[eigVals, eigVecs] = eigsh(A=K, k=nModes+excludeRigidBodyModes, M=M, which='SM') #this gives omega^2 ... squared eigen frequencies (rad/s)

            self.modeBasis = {'matrix':eigVecs[:,excludeRigidBodyModes:excludeRigidBodyModes + nModes], 
                              'type':'NormalNodes'}
            self.eigenValues = abs(eigVals)

    #returns list of eigenvalues in Hz
    def GetEigenFrequenciesHz(self):
        return np.sqrt(self.eigenValues)/(2.*np.pi)

    #perform some consistency checks
    def CheckConsistency(self):
        nNodes = NumberOfNodes()
        #nNodes = len(self.nodes['Position']) #old

        if self.massMatrix.shape != (0,0):
            (rows,columns) = CSRtoRowsAndColumns(self.massMatrix)
            if rows != nNodes*3:
                print("ERROR: CheckConsistency: massMatrix rows different from nodes coordinates dimension")
            if columns != nNodes*3:
                print("ERROR: CheckConsistency: massMatrix columns different from nodes coordinates dimension")

        if self.stiffnessMatrix.shape != (0,0):
            (rows,columns) = CSRtoRowsAndColumns(self.stiffnessMatrix)
            if rows != nNodes*3:
                print("ERROR: CheckConsistency: stiffnessMatrix rows different from nodes coordinates dimension")
            if columns != nNodes*3:
                print("ERROR: CheckConsistency: stiffnessMatrix columns different from nodes coordinates dimension")

