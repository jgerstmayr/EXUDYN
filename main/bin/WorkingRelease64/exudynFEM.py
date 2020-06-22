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
from exudynUtilities import RoundMatrix, ComputeSkewMatrix, FillInSubMatrix, PlotLineCode
from exudynRigidBodyUtilities import AngularVelocity2EulerParameters_t, EulerParameters2GLocal, Skew
import numpy as np #LoadSolutionFile

#convert zero-based sparse matrix data to dense numpy matrix
#DEPRECTAED!!!!!!!!!!!!!!!!
def CompressedRowToDenseMatrix(sparseData):
    print("\n**************************\nWARNING: CompressedRowToDenseMatrix is deprecated!\n**************************\n")
    n = int(max(np.max(sparseData[:,0]),np.max(sparseData[:,1]))) #rows and columns are 1-based
    m = np.zeros((n,n))
    for row in sparseData:
        m[int(row[0])-1,int(row[1])-1] += row[2] #convert 1-based to 0-based; += for double entries
    return m

#**function: convert zero-based sparse matrix data to dense numpy matrix
#**input: 
#  sparseData: format (per row): [row, column, value] ==> converted into dense format
#**output: a dense matrix as np.array
def CompressedRowSparseToDenseMatrix(sparseData):
    #does not work, if there are no entry in highest rows and columns
    n = int(max(np.max(sparseData[:,0]),np.max(sparseData[:,1])))+1 #rows and columns indices are 0-based ==> add 1 for size!
    m = np.zeros((n,n))
    for row in sparseData:
        m[int(row[0]),int(row[1])] += row[2]  #+= for double entries
    return m

#**function: resort a sparse matrix (internal CSR format) with given sorting for rows and columns; changes matrix directly! used for ANSYS matrix import
def MapSparseMatrixIndices(matrix, sorting):
    for row in matrix:
        row[0] = sorting[int(row[0])]
        row[1] = sorting[int(row[1])]


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute diadic product of vector v and a 3D unit matrix = diadic(v,I$_{3x3}$); used for ObjectFFRF and CMS implementation
def VectorDiadicUnitMatrix3D(v):
    return np.kron(np.array(v), np.eye(3)).T

#print("diadicTest=", VectorDiadicUnitMatrix3D(np.array([1,2,3,4,5,6])))

#+++++++++++++++++++++++++++
#**function: compare cyclic two lists, reverse second list; return True, if any cyclic shifted lists are same, False otherwise
def CyclicCompareReversed(list1, list2):
    revList1 = np.flip(list1,0)
    for i in range(len(list2)):
        if (revList1 == np.roll(list2,i)).all(): return True
    return False


#**function: add entry to compressedRowSparse matrix, avoiding duplicates
#value is either added to existing entry (avoid duplicates) or a new entry is appended
def AddEntryToCompressedRowSparseArray(sparseData, row, column, value):
    n = len(sparseData[:,0])
    #print("AddEntryToCompressedRowSparseArray:",row,column,value, ", n=", n)
    for i in range(n):
        if int(sparseData[i,0]) == row and int(sparseData[i,1]) == column:
            sparseData[i,2] += value
            #print("AddEntryToCompressedRowSparseArray, value added")
            return sparseData
#        if int(sparseData[i,0]) > row:
#            np.insert(sparseData, i, np.array((row, column, value)), 0)
#            print("AddEntryToCompressedRowSparseArray, row inserted, i=",i)
#            return sparseData
    #insert at end of matrix:
    #print("AddEntryToCompressedRowSparseArray, row added at end")
    np.insert(sparseData, n, np.array((row, column, value)), 0)
    return sparseData

#**function: compute rows and columns of a compressed sparse matrix and return as tuple: (rows,columns)
def CSRtoRowsAndColumns(sparseMatrixCSR):
    rows = sparseMatrixCSR[:,0].max()
    columns = sparseMatrixCSR[:,1].max()
    return (rows, columns)

#**function: convert internal compressed CSR to scipy.sparse csr matrix
def CSRtoScipySparseCSR(sparseMatrixCSR):
    from scipy.sparse import csr_matrix
    X = csr_matrix((sparseMatrixCSR[:,2],(sparseMatrixCSR[:,0].astype(int),sparseMatrixCSR[:,1].astype(int))))
    #X.sum_duplicates() 
    return X


#**function: convert scipy.sparse csr matrix to internal compressed CSR 
def ScipySparseCSRtoCSR(scipyCSR):
    from scipy.sparse import csr_matrix
    data=scipyCSR.tocoo()
    sparseData = [data.row,data.col,data.data]
    return np.array(sparseData).T

#**function: resort indices of given CSR matrix in XXXYYYZZZ format to XYZXYZXYZ format; numberOfRows must be equal to columns
#needed for import from NGsolve
def ResortIndicesOfCSRmatrix(mXXYYZZ, numberOfRows):
    #compute resorting index array [0,1,2, 3,4,5, 6,7,8] ==> [0,3,6, 1,4,7, 2,5,8]
    if numberOfRows%3 != 0:
        raise ValueError("ResortIndicesOfCSRmatrix: numberOfRows must be multiple of 3")
    
    #compute transformation of indices:
    r = np.arange(numberOfRows).reshape((int(numberOfRows/3),3))
    r = r.T.flatten()
    
    nSparse = len(mXXYYZZ)
    for i in range(nSparse): #for loop is slow, but works ok for 100.000 DOF
        mXXYYZZ[i,0] = r[int(mXXYYZZ[i,0])]
        mXXYYZZ[i,1] = r[int(mXXYYZZ[i,1])]
    



#DEPRECATED!
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




#**function: convert list of Hex8/C3D8  element with 8 nodes in nodeNumbers into triangle-List
#also works for Hex20 elements, but does only take the corner nodes!
def ConvertHexToTrigs(nodeNumbers):
    localList = [[0,1,2], [0,2,3], [6,5,4], [6,4,7], [0,4,1], [1,4,5], [1,5,2], [2,5,6], [2,6,3], [3,6,7], [3,7,0], [0,7,4]]

    trigList = []
    for trig in localList:
        ind = [nodeNumbers[trig[0]], nodeNumbers[trig[1]], nodeNumbers[trig[2]]]
        trigList += [ind]
    return trigList



#**function: convert numpy.array dense matrix to (internal) compressed row format
def ConvertDenseToCompressedRowMatrix(denseMatrix):
    sparseMatrix = []
    (nRows,nColumns) = denseMatrix.shape
    for i in range(nRows):
        for j in range(nColumns):
            if denseMatrix[i,j] != 0.:
                sparseMatrix += [[i,j,denseMatrix[i,j]]]
    return np.array(sparseMatrix)




#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function:This function reads either the mass or stiffness matrix from an Ansys
#           Matrix Market Format (MMF). The corresponding matrix can either be exported 
#           as dense matrix or sparse matrix.
#
#**input: fileName of MMF file
#**output: internal compressed row sparse matrix (as (nrows x 3) numpy array)
#
#**author: Stefan Holzinger
#
# Note:
#   A MMF file can be created in Ansys by placing the following APDL code inside
#   the solution tree in Ansys Workbench:
#
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#   ! APDL code that exports sparse stiffnes and mass matrix in MMF format. If 
#   ! the dense matrix is needed, replace *SMAT with *DMAT in the following
#   ! APDL code.
#    
#   ! Export the stiffness matrix in MMF format
#   *SMAT,MatKD,D,IMPORT,FULL,file.full,STIFF
#   *EXPORT,MatKD,MMF,fileNameStiffnessMatrix,,,
#
#   ! Export the mass matrix in MMF format
#   *SMAT,MatMD,D,IMPORT,FULL,file.full,MASS
#   *EXPORT,MatMD,MMF,fileNameMassMatrix,,,
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#
# In case a lumped mass matrix is needed, place the following APDL Code inside 
# the Modal Analysis Tree:
#
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# ! APDL code to force Ansys to use a lumped mass formulation (if available for
# ! used elements)
#
# LUMPM, ON, , 0
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!    
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
def ReadMatrixFromAnsysMMF(fileName, verbose=False):
    if verbose: print("Start read matrix")
    # read file
    fileLines = []
    file=open(fileName,'r') 
    fileLines = file.readlines()
    file.close()
    
    # delete text infos
    offset = 0 #compute offset of comments and others
    while fileLines[offset][0] == '%':
        offset += 1
        #del fileLines[0]
    
    # delete size information in fileLines
    #del fileLines[0]
    offset += 1

    # put CSR data into this list
    dataList = []
    
    # read row and col indices as well as corresponding matrix values
    lineCnt = 0
    for line in fileLines:
        if lineCnt%500000 == 0 and lineCnt !=0: 
            if verbose: print("parse line",lineCnt," / ", len(fileLines))
        if lineCnt>=offset:
            rowStr = line.rsplit()
            
            row=[int(rowStr[0])-1, int(rowStr[1])-1, float(rowStr[2])] #convert to 0-based format
            dataList+=[row]
            if row[0] != row[1]: #Ansys only stores half of matrix==>add symmetric terms except diagonal terms!
                dataList+=[[row[1],row[0],row[2]]]
        lineCnt+=1
                    
    return np.array(dataList)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: read sorting vector for ANSYS mass and stiffness matrices and return sorting vector as np.array
#  the file contains sorting for nodes and applies this sorting to the DOF (assuming 3 DOF per node!)
#  the resulting sorted vector is already converted to 0-based indices
def ReadMatrixDOFmappingVectorFromAnsysTxt(fileName):
    fileLines = []
    file=open(fileName,'r') 
    fileLines = file.readlines()
    file.close()
    nLines = len(fileLines)

    #read leading comments
    lineOffset = 0
    while lineOffset < nLines and fileLines[lineOffset][0] == '%':
        lineOffset+=1

    #check header:
    rowStr = fileLines[lineOffset].rsplit()
    nDOF = int(rowStr[0])
    if int(rowStr[1]) != 1:
        raise ValueError("ReadMatrixDOFmappingVectorFromAnsysTxt: invalid value in line " + str(lineOffset+1) + ", expected 1 column")

    if int(rowStr[0]) != nLines-lineOffset-1:
        raise ValueError("ReadMatrixDOFmappingVectorFromAnsysTxt: number of lines do not match the number of DOF: nDOF="+str(nDOF)+", #data lines=", arg_str(nLines-lineOffset-1))

    lineOffset+=1

    #read now the mapping of the DOF line by line
    dataList = []
    lineCnt = 0
    for line in fileLines:
        if lineCnt>=lineOffset:
            dataList += [(int(line)-1)*3+0] #convert to 0-base and apply for x,y and z coordinate of node ...
            dataList += [(int(line)-1)*3+1] #convert to 0-base and apply for x,y and z coordinate of node ...
            dataList += [(int(line)-1)*3+2] #convert to 0-base and apply for x,y and z coordinate of node ...
        lineCnt+=1
                    
    return np.array(dataList)    


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: This function reads the nodal coordinates exported from Ansys.
#
#**input: fileName (file name ending must be .txt!)
#**output: nodal coordinates as numpy array
#
#**author: Stefan Holzinger
#
#**notes:
#   The nodal coordinates can be exported from Ansys by creating a named selection
#   of the body whos mesh should to exported by choosing its geometry. Next, 
#   create a second named selcetion by using a worksheet. Add the named selection
#   that was created first into the worksheet of the second named selection.
#   Inside the working sheet, choose 'convert' and convert the first created
#   named selection to 'mesh node' (Netzknoten in german) and click on generate
#   to create the second named selection. Next, right click on the second 
#   named selection tha was created and choose 'export' and save the nodal 
#   coordinates as .txt file.
#   
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def ReadNodalCoordinatesFromAnsysTxt(fileName, verbose=False):
    
    # read file
    fileLines = []
    file=open(fileName,'r') 
    fileLines = file.readlines()
    file.close()
            
    # delete first line
    del fileLines[0]
    
    # number of mesh nodes
    numberOfNodes = len(fileLines)
    
    # allocate memory
    # unsorted lists
    unsortedNodalCoordinates = [None]*(numberOfNodes)
    unsortedNodeNumberList   = [None]*(numberOfNodes)
    currentNodalCoordinates  = np.zeros(3)
    unsortedNodeDictList     = [None]*(numberOfNodes)
    # sorted lists
    #sortedNodalCoordinates = [None]*(numberOfNodes)
    sortedNodalCoordinates = np.zeros((numberOfNodes,3),dtype=float)
    sortedNodeNumberList   = [None]*(numberOfNodes)
    sortedNodeListDict     = [None]*(numberOfNodes)
    
    lineCtr = 0
    for line in fileLines:
        #if lineCtr%10000 == 0: print("read node",lineCtr)
        # remove '\n' from value at line end
        currentLine = line.rsplit()
            
        # add node index (one based) and corresponding nodal coordinates to node dict   
        currentNodalCoordinates = currentLine[1:]   
        for i in range(len(currentNodalCoordinates)): 
            currentNodalCoordinates[i] = float(currentNodalCoordinates[i].replace(",","."))   
        
        # add items to corresponding list
        unsortedNodeNumberList[lineCtr]   = int(currentLine[0])
        unsortedNodalCoordinates[lineCtr] = currentNodalCoordinates
        unsortedNodeDictList[lineCtr]     = {"nodeNr":int(currentLine[0]), "nodalCoordinates": currentNodalCoordinates}
            
        # increase line counter
        lineCtr += 1        
        
    
    # sort node list w.r.t. increasing node numbers
    sortedNodeListDict = sorted(unsortedNodeDictList, key = lambda i: i["nodeNr"])
    dictCtr = 0
    for dict_ in sortedNodeListDict:
        
        sortedNodalCoordinates[dictCtr] = sortedNodeListDict[dictCtr]["nodalCoordinates"]
        sortedNodeNumberList[dictCtr]   = sortedNodeListDict[dictCtr]["nodeNr"]
        dictCtr += 1
    
    return sortedNodalCoordinates





#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: This function reads the nodal coordinates exported from Ansys.
#
#**input: fileName (file name ending must be .txt!)
#**output: element connectivity as numpy array
#
#**author: Stefan Holzinger
#
#**notes:
#   The elements can be exported from Ansys by creating a named selection
#   of the body whos mesh should to exported by choosing its geometry. Next, 
#   create a second named selcetion by using a worksheet. Add the named selection
#   that was created first into the worksheet of the second named selection.
#   Inside the worksheet, choose 'convert' and convert the first created
#   named selection to 'mesh element' (Netzelement in german) and click on generate
#   to create the second named selection. Next, right click on the second 
#   named selection tha was created and choose 'export' and save the elements 
#   as .txt file.
#   
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def ReadElementsFromAnsysTxt(fileName, verbose=False):
    
    # read file
    fileLines = []
    file=open(fileName,'r') 
    fileLines = file.readlines()
    file.close()
                
    # delete first line ==> contains text
    del fileLines[0]
        
    # number of mesh nodes
    #numberOfElements = len(fileLines) #do not trust this, could be changed with one additional line
    numberOfElements = 0

    #check file consistency => first columns should contain element number (1-based)
    for i in range(len(fileLines)):
        line = fileLines[i]
        n = int(line.rsplit()[0])
        #print("element:", n)
        if n == i+1: 
            numberOfElements = n
        else:
            raise ValueError("ReadElementsFromAnsysTxt: invalid format in line "+str(i+2)+": expected element number, type and connectivity; be careful with empty lines")
                
    
    # allocate memory
    elementTypeList = [None]*(numberOfElements)
    elementConnectivityList = [None]*(numberOfElements)
    currentNodeIndexList = []
    
    for lineCtr in range(numberOfElements):
        if lineCtr%10000 == 0 and lineCtr !=0: 
            if verbose: print("read element",lineCtr," / ", numberOfElements)
        #if lineCtr%10000 == 0: print("read element",lineCtr)
        line = fileLines[lineCtr]
        
        # split current line at \n
        currentLine = line.rsplit()
        
        # get element type of current element
        elementTypeList[lineCtr] = currentLine[1]
        
        # get element connectivity of current element
        for node in currentLine[2:]:
            currentNodeIndexList.append( int(node)-1 )  # convert to zero based
        
        # add element connectivity of current element to elementConnectivityList 
        elementConnectivityList[lineCtr] = currentNodeIndexList 
        
        # clear current node index list (nodes that form the current element)
        currentNodeIndexList = []
        
    #elementsDict = {'Name':'elements', 'Tet4':[], 'Hex8':elementConnectivityList}
    elementsDict = {'Name':'elements'}

    if verbose: print("create element dictionaries...")
    for lineCtr in range(numberOfElements):
        line = fileLines[lineCtr]
        if not(elementTypeList[lineCtr] in elementsDict):
            elementsDict[elementTypeList[lineCtr]] = []
        elementsDict[elementTypeList[lineCtr]] += [elementConnectivityList[lineCtr]]

    return elementsDict












#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++          ObjectFFRF                               ++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**class: compute terms necessary for ObjectFFRF
#class used internally in FEMinterface to compute ObjectFFRF object 
#this class holds all data for ObjectFFRF user functions
class ObjectFFRFinterface:
    #**classFunction: initialize ObjectFFRFinterface with FEMinterface class
    #  initializes the ObjectFFRFinterface with nodes, modes, surface description and systemmatrices from FEMinterface
    #  data is then transfered to mbs object with classFunction AddObjectFFRF(...)
    def __init__(self, femInterface):
        self.modeBasis = femInterface.modeBasis['matrix']
        self.nodeArray = femInterface.GetNodePositionsAsArray()
        self.trigList = femInterface.GetSurfaceTriangles()

        #stiffnessMatrixCSR = CSRtoScipySparseCSR(femInterface.GetStiffnessMatrix(sparse=True))
        self.massMatrixCSR = CSRtoScipySparseCSR(femInterface.GetMassMatrix(sparse=True)) #for multiplications
        self.stiffnessMatrixSparse = femInterface.GetStiffnessMatrix(sparse=True)
        self.massMatrixSparse = femInterface.GetMassMatrix(sparse=True)

        #new coordinates:
        self.nNodes = len(self.nodeArray)                   #stored in nNodes x 3 np-array
        self.dim3D = len(self.nodeArray[0])                 #dimension of position, assuming that one node exists ....
        self.nODE2rot = 4                                   #dimension of rotation parameters; fixed to 4 for now!
        self.nODE2rigid = self.dim3D + self.nODE2rot
        self.nODE2FF = self.nNodes * self.dim3D
        self.nODE2FFRF = self.nODE2rigid + self.nODE2FF
        nNodesFFRF = self.nNodes+1                          #including rigid body node

        #self.massMatrixFFRF = np.zeros((self.nODE2FFRF,self.nODE2FFRF)) #create larger FFRF mass matrix
        #self.stiffnessMatrixFFRF = np.zeros((self.nODE2FFRF,self.nODE2FFRF)) #create larger FFRF mass matrix

        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #FFRFreduced constant matrices:
        self.Phit = np.kron(np.ones(self.nNodes),np.eye(3)).T
        self.PhitTM = self.Phit.T @ self.massMatrixCSR #LARGE MATRIX COMPUTATION
        self.xRef = self.nodeArray.flatten()                     #node reference values in single vector (can be added then to q[7:])
        self.xRefTilde = ComputeSkewMatrix(self.xRef) #rfTilde without q    

        #not needed, but may be interesting for checks:
        self.inertiaLocal = self.xRefTilde.T @ self.massMatrixCSR @ self.xRefTilde #LARGE MATRIX COMPUTATION


    #**classFunction: add according nodes, objects and constraints for FFRF object to MainSystem mbs
    #**input:
    #  exu: the exudyn module
    #  mbs: a MainSystem object
    #  positionRef: reference position of created ObjectFFRF (set in rigid body node underlying to ObjectFFRF)
    #  eulerParametersRef: reference euler parameters of created ObjectFFRF (set in rigid body node underlying to ObjectFFRF)
    #  initialVelocity: initial velocity of created ObjectFFRF (set in rigid body node underlying to ObjectFFRF)
    #  initialAngularVelocity: initial angular velocity of created ObjectFFRF (set in rigid body node underlying to ObjectFFRF)
    #  gravity: set [0,0,0] if no gravity shall be applied, or to the gravity vector otherwise
    #  constrainRigidBodyMotion: set True in order to add constraint (Tisserand frame) in order to suppress rigid motion of mesh nodes
    #  color: provided as list of 4 RGBA values
    #add object to mbs as well as according nodes
    def AddObjectFFRF(self, exu, mbs, 
                      positionRef=[0,0,0], eulerParametersRef=[1,0,0,0], 
                      initialVelocity=[0,0,0], initialAngularVelocity=[0,0,0],
                      gravity=[0,0,0],
                      constrainRigidBodyMotion=True, 
                      color=[0.1,0.9,0.1,1.]):

        self.gravity = gravity

        #compute initial euler parameter velocities from angular velocity vector
        self.eulerParameters_t0 = AngularVelocity2EulerParameters_t(initialAngularVelocity, eulerParametersRef)
        self.eulerParameters0 = eulerParametersRef

        #rigid body node for ObjectFFRF
        self.nRigidBody = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=list(positionRef)+list(eulerParametersRef), 
                                            initialVelocities=list(initialVelocity)+list(self.eulerParameters_t0)))

        self.nodeList = [] #list of nodenumbers in mbs
        for node in self.nodeArray:
            nMBS = mbs.AddNode(Point(referenceCoordinates = list(node), visualization=VNodePoint(show = False))) 
            self.nodeList += [nMBS]

        stiffnessMatrixMC = exu.MatrixContainer()
        #stiffnessMatrixMC.SetWithSparseMatrixCSR(self.nODE2FF, self.nODE2FF, self.stiffnessMatrixSparse,useDenseMatrix=False)
        stiffnessMatrixMC.SetWithDenseMatrix(CompressedRowSparseToDenseMatrix(self.stiffnessMatrixSparse),useDenseMatrix=False)

        massMatrixMC = exu.MatrixContainer()
        #massMatrixMC.SetWithSparseMatrixCSR(self.nODE2FF, self.nODE2FF, self.massMatrixSparse,useDenseMatrix=False)
        massMatrixMC.SetWithDenseMatrix(CompressedRowSparseToDenseMatrix(self.massMatrixSparse),useDenseMatrix=False)

        #add body for FFRF-Object:
        self.oFFRF = mbs.AddObject(ObjectFFRF(nodeNumbers = [self.nRigidBody] + self.nodeList, 
                                                            massMatrixFF=massMatrixMC,
                                                            stiffnessMatrixFF=stiffnessMatrixMC, 
                                                            #dampingMatrixFF=emptyMC,
                                                            #forceVector=fNew,
                                                            #forceUserFunction=UFforce,
                                                            #computeFFRFterms=True,
                                                            #massMatrixUserFunction=UFmassGenericODE2,
                                                            visualization=VObjectFFRF(triangleMesh = self.trigList, 
                                                                                      color=color,
                                                                                      showNodes = True)))


        self.oRigidBodyConstraint = -1
        if constrainRigidBodyMotion:
            mObjectCoordinates = mbs.AddMarker(MarkerObjectODE2Coordinates(objectNumber=self.oFFRF))
            nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0], visualization = VNodePointGround(show=False))) #ground node for coordinate constraint
            mGroundCoordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

            X1 = np.zeros((6, self.nODE2FFRF))
            X1rot = ComputeSkewMatrix(self.xRef).T @ self.massMatrixCSR
            X1[0:3,self.nODE2rigid:] = self.PhitTM
            X1[3:6,self.nODE2rigid:] = X1rot
            offset = np.zeros(6)
            offset[0:3] = self.PhitTM @ self.xRef #constrain current COM to reference COM

            #add constraint: X1*qObjectFFRF - offset = 0
            self.oRigidBodyConstraint = mbs.AddObject(CoordinateVectorConstraint(markerNumbers=[mGroundCoordinate, mObjectCoordinates], scalingMarker1 = X1, offset=offset))

        dictReturn = {'nRigidBody':self.nRigidBody,
                      'nodeList':self.nodeList,
                      'oFFRF':self.oFFRF,
                      'oRigidBodyConstraint':self.oRigidBodyConstraint}

        return dictReturn

    #**classFunction: optional forceUserFunction for ObjectFFRF (per default, this user function is ignored)
    def UFforce(self, exu, mbs, t, q, q_t):
        print("UFforce: not tested and not integrated in to FFRFinterface!")

        force = np.zeros(self.nODE2FFRF)
        Avec = mbs.GetNodeOutput(self.nRigidBody,  exu.OutputVariableType.RotationMatrix)
        A = Avec.reshape((3,3))

        #implementation for Euler Parameters (Glocal_t*theta_t=0)
        ep = np.array(q[self.dim3D:self.nODE2rigid]) + ep0 #add reference values, q are only the change w.r.t. reference values!
        G = EulerParameters2GLocal(ep)
    
        cF_t = np.array(q_t[self.nODE2rigid:])         #velocities of flexible coordinates

        rF = self.xRef + np.array(q[self.nODE2rigid:]) #nodal position

        omega3D = G @ np.array(q_t[self.dim3D:self.nODE2rigid])
        omega3Dtilde = Skew(omega3D)
        omega = np.array(list(omega3D)*self.nNodes)
        omegaTilde = np.kron(np.eye(nNodes),omega3Dtilde)

        #squared angul. vel. matrix:
        omega3Dtilde2 = Skew(omega3D) @ Skew(omega3D)
        omegaTilde2 = np.kron(np.eye(self.nNodes),omega3Dtilde2)

        #these 2 terms are computationally costly:
        rfTilde = ComputeSkewMatrix(rF) #rfTilde
        cF_tTilde = ComputeSkewMatrix(cF_t) 

        fTrans = A @ (omega3Dtilde @ self.PhitTM @ rfTilde @ omega3D + 2*self.PhitTM @ cF_tTilde @ omega3D)
        force[0:self.dim3D] = fTrans

        fRot = -G.T@(omega3Dtilde @ rfTilde.T @ self.massMatrixCSR @ rfTilde @ omega3D + 
                        2*rfTilde.T @ self.massMatrixCSR @ cF_tTilde @ omega3D)
        force[self.dim3D:self.nODE2rigid] = fRot
    
        fFlex = -self.massMatrixCSR @ (omegaTilde2 @ rF + 2*(omegaTilde @ cF_t))
        force[self.nODE2rigid:] = fFlex

        #add gravity:
        if False:
            fGrav = np.array(fGravRigid + list(self.PhitTM.T @ (A.T @ self.gravity)) ) #only local vector, without rotation
            force += fGrav

        return force

    #**classFunction: optional massMatrixUserFunction for ObjectFFRF (per default, this user function is ignored)
    def UFmassGenericODE2(self, exu, mbs, t, q, q_t):
        print("UFmassGenericODE2: not tested and not integrated into FFRFinterface!")
        Avec = mbs.GetNodeOutput(self.nRigidBody,  exu.OutputVariableType.RotationMatrix)
        A = Avec.reshape((3,3))
        ep = q[self.dim3D:self.nODE2rigid] + ep0 #add reference values, q are only the change w.r.t. reference values!
        G = EulerParameters2GLocal(ep)

        rF = self.xRef + q[self.nODE2rigid:] #nodal position
        rfTilde = ComputeSkewMatrix(rF) #rfTilde

        #Mtr:
        Mtr = -A @ self.PhitTM @ rfTilde @ G
        Mnew[0:self.dim3D, self.dim3D:self.dim3D+self.nODE2rot] = Mtr
        Mnew[self.dim3D:self.dim3D+self.nODE2rot, 0:self.dim3D] = Mtr.T
        #Mtf:
        Mtf = A @ self.PhitTM
        Mnew[0:self.dim3D, self.nODE2rigid:] = Mtf
        Mnew[self.nODE2rigid:, 0:self.dim3D] = Mtf.T
        #Mrf:
        Mrf = -G.T @ rfTilde.T @ self.massMatrixCSR
        Mnew[self.dim3D:self.dim3D+self.nODE2rot, self.nODE2rigid:] = Mrf
        Mnew[self.nODE2rigid:, self.dim3D:self.dim3D+self.nODE2rot] = Mrf.T
        #Mrr:
        Mnew[self.dim3D:self.dim3D+self.nODE2rot, self.dim3D:self.dim3D+self.nODE2rot] = -Mrf @ rfTilde @ G   #G.T @ rfTilde.T @ massMatrix @ rfTilde @ G

        return Mnew






#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++          ObjectFFRFreducedOrderTerms              ++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**class: compute terms necessary for ObjectFFRFreducedOrder
#  class used internally in FEMinterface to compute ObjectFFRFreducedOrder dictionary
#  this class holds all data for ObjectFFRFreducedOrder user functions
class ObjectFFRFreducedOrderInterface:
    #**classFunction: initialize ObjectFFRFreducedOrderInterface with FEMinterface class
    #  initializes the ObjectFFRFreducedOrderInterface with nodes, modes, surface description and reduced system matrices from FEMinterface
    #  data is then transfered to mbs object with classFunction AddObjectFFRFreducedOrderWithUserFunctions(...)
    #**input: 
    #  femInterface: must provide nodes, surfaceTriangles, modeBasis, massMatrix, stiffness
    #  roundMassMatrix: use this value to set entries of reduced mass matrix to zero which are below the treshold
    #  roundStiffNessMatrix: use this value to set entries of reduced stiffness matrix to zero which are below the treshold
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

    #**classFunction: add according nodes, objects and constraints for ObjectFFRFreducedOrder object to MainSystem mbs
    #**input:
    #  exu: the exudyn module
    #  mbs: a MainSystem object
    #  positionRef: reference position of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
    #  eulerParametersRef: reference euler parameters of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
    #  initialVelocity: initial velocity of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
    #  initialAngularVelocity: initial angular velocity of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
    #  gravity: set [0,0,0] if no gravity shall be applied, or to the gravity vector otherwise
    #  UFforce: provide a user function, which computes the quadratic velocity vector and applied forces; usually this function reads like:\\ \texttt{def UFforceFFRFreducedOrder(t, qReduced, qReduced\_t):\\ \phantom{XXXX}return cms.UFforceFFRFreducedOrder(exu, mbs, t, qReduced, qReduced\_t)}
    #  UFmassMatrix: provide a user function, which computes the quadratic velocity vector and applied forces; usually this function reads like:\\ \texttt{def UFmassFFRFreducedOrder(t, qReduced, qReduced\_t):\\  \phantom{XXXX}return cms.UFmassFFRFreducedOrder(exu, mbs, t, qReduced, qReduced\_t)}
    #  color: provided as list of 4 RGBA values
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

        #not needed any more; Euler parameter constraint included now in ObjectFFRFreducedOrder
        #if self.nODE2rot == 4: #for euler parameters --> add body to constrain EP
        #    epsMass = 1e-3#needed, if not all ffrf terms are included
        #    #add rigid body to node for Euler Parameter constraint:
        #    oAddedBody = mbs.AddObject(ObjectRigidBody(nodeNumber=self.nRigidBody, physicsMass=epsMass, 
        #                                               physicsInertia=[epsMass,epsMass,epsMass,0,0,0])) 
        #    dictReturn['oAddedBody'] = oAddedBody

        return dictReturn



    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: CMS mass matrix user function; qReduced and qReduced\_t contain the coordiantes of the rigid body node and the modal coordinates in one vector!
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
    #**classFunction: CMS force matrix user function; qReduced and qReduced\_t contain the coordiantes of the rigid body node and the modal coordinates in one vector!
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

        #add gravity (needs to be tested):
        if True:
            force[0:self.dim3D] += self.totalMass*np.array(self.gravity)
            force[self.dim3D:self.nODE2rigid] += G.T @ (Skew(self.chiU) @ (self.totalMass*A.T @ np.array(self.gravity)))
            force[self.nODE2rigid:] += self.mPhitTPsi.T @ (A.T @ np.array(self.gravity))

            #force[nODE2rigid:] += modeBasis.T @ PhitTM.T @ (A.T @ g)

        return force

        


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++   FEMinterface - finite element interface class   ++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**class: general interface to different FEM / mesh imports and export to EXUDYN functions
#         use this class to import meshes from different meshing or FEM programs (NETGEN/NGsolve, ABAQUS, ANSYS, ..) and store it in a unique format
#         do mesh operations, compute eigenmodes and reduced basis, etc.
#         load/store the data efficiently with LoadFromFile(...), SaveToFile(...)  if import functions are slow
#         export to EXUDYN objects
class FEMinterface:
    #**classFunction: initalize all data of the FEMinterface by, e.g., \texttt{fem = FEMinterface()}
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
        #self.ffrfReducedOrderTerms = () # 

        #some additional information, needed for checks and easier operation
        self.coordinatesPerNodeType = {'Position':3, 'Position2D':2, 'RigidBodyRxyz':6, 'RigidBodyEP':7} #number of coordinates for a certain node type


    #**classFunction: save all data (nodes, elements, ...) to a data filename; this function is much faster than the text-based import functions
    #**input: use filename without ending ==> ".npy" will be added
    def SaveToFile(self, fileName):
        with open(fileName, 'wb') as f:
            np.save(f, self.nodes, allow_pickle=True)
            np.save(f, self.elements, allow_pickle=True)
            np.save(f, self.massMatrix)
            np.save(f, self.stiffnessMatrix)
            np.save(f, self.surface, allow_pickle=True)
            np.save(f, self.nodeSets, allow_pickle=True)
            np.save(f, self.elementSets, allow_pickle=True)
            np.save(f, self.modeBasis, allow_pickle=True)
            np.save(f, self.eigenValues, allow_pickle=True)

    #**classFunction: load all data (nodes, elements, ...) from a data filename previously stored with SaveToFile(...). 
    #this function is much faster than the text-based import functions
    #**input: use filename without ending ==> ".npy" will be added
    def LoadFromFile(self, fileName):
        with open(fileName, 'rb') as f:
            self.nodes = np.load(f, allow_pickle=True).all()
            self.elements = list(np.load(f, allow_pickle=True))
            self.massMatrix = np.load(f)
            self.stiffnessMatrix = np.load(f)
            self.surface = list(np.load(f, allow_pickle=True))
            self.nodeSets =  list(np.load(f, allow_pickle=True))
            self.elementSets = list(np.load(f, allow_pickle=True))
            self.modeBasis = np.load(f, allow_pickle=True).all()
            self.eigenValues = list(np.load(f, allow_pickle=True))

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #ABAQUS import functions

#    #import Abaqus input file 
#    #node numbers in elements are converted from 1-based indices to python's 0-based indices
#    #return node numbers
#    def ImportFromAbaqusInputFileOLD(self, fileName, typeName='Part', name='Part-1'):
#        fileLines = []
#        print("read file name=", fileName)
#        file=open(fileName,'r') 
#        fileLines = file.readlines()
#        file.close()
#        
#        print("read ", len(fileLines), "lines")
#    
#        startPart = False
#        startReadNodes = False
#        finishedReadNodes = False
#        startReadElements = False
#        finishedReadElements = False
#        nodes = [] #store list of node values
#        elements = [] #store list of elements with node numbers
#        elementTypes = [] #string list of element types
#
#        errorOccured = False
#        lineCnt = 0
#        for line in fileLines:
#            #print("line", lineCnt, "=", line)
#            lineCnt+=1
#            if errorOccured:
#                break
#        
#            if startReadNodes and not finishedReadNodes:
#                if line[0] != '*': #check if nodes section has finished
#                    lineData = line.split(',') #split into values
#                    if len(lineData) != 4:
#                        print("ERROR: Expected node number and 3 coordinates, line ", lineCnt)
#                        errorOccured = True
#                    else:
#                        v = []
#                        for i in range(3):
#                            v+=[float(lineData[i+1])]
#                        nodes += [v] #add node data
#                else:
#                    startReadNodes = False
#                    finishedReadNodes = True
#    
#            if startReadElements and not finishedReadElements:
#                if line[0] != '*': #check if nodes section has finished
#                    lineData = line.split(',') #split into values
#                    if len(lineData) != 9:
#                        print("ERROR: Expected element number and 8 indices for C3D8R, line=", lineCnt)
#                        errorOccured = True
#                    else:
#                        v = []
#                        for i in range(8):
#                            v+=[float(lineData[i+1])]
#                        elements += [v] #add node data
#                        elementTypes += ['Hex8'] #8-noded hexahedral
#                else:
#                    startReadElements = False
#                    finishedReadElements = True
#    
#            if startPart and not startReadNodes and not finishedReadNodes:
#                if line[0:5] == '*Node':
#                    startReadNodes = True
#                    startPart = False
#                else:
#                    print("ERROR: Expected *Node after *Part, line=", lineCnt)
#                    errorOccured = True
#
#            if finishedReadNodes and not startReadElements:
#                if line[0:8] == '*Element':
#                    startReadElements = True
#                    #check "type=C3D8R" in future
#
#            
#            if line[0:len(typeName)+1] == '*'+typeName:
#                if not startPart:
#                    #print("ERROR: only one *Part section allowed, line=", lineCnt)
#                    #errorOccured = True
#                
#                    lineInfo = line.split(',')
#                    #print(lineInfo)
#                    if len(lineInfo) != 3:
#                        print("ERROR: invalid information for part/instance name, line=", lineCnt)
#                        errorOccured = True
#                    else:
#                        nameInfo = lineInfo[1].strip().split('=')
#                        if nameInfo[0] != 'name':
#                            print("ERROR: expected 'name=' in line=", lineCnt)
#                            errorOccured = True
#                        else:
#                            if nameInfo[1] != name:
#                                print("ERROR: expected name='" + name + "' in line=", lineCnt)
#                                errorOccured = True
#                            else:
#                                startPart = True
#
#        self.nodes['Position'] = np.array(nodes)
#        elements = np.array(elements)-1 #convert node indices from 1 to 0-based
#
#        elementsDict = {'Name':name, 'Tet4':[], 'Hex8':[]}
#        for i in range(len(elements)):
#            if elementTypes[i] in elementsDict:
#                elementsDict[elementTypes[i]] += [elements[i]]
#            else:
#                print("FEMinterface.ImportFromAbaqusInputFile: unknown elementType "+elementTypes[i]+", ignored")
#
#        self.elements += [elementsDict]
#
#        #convert elements to triangles for drawing:
#        trigList = []
#        for element in elements:
#            trigList += ConvertHexToTrigs(element) #node numbers are already 0-based
#        trigList = trigList
#        self.surface += [{'Name':'meshSurface', 'Trigs':trigList}]    # [{'Name':'identifier', 'Trigs':[[n0,n1,n2],...], 'Quads':[[n0,...,n3],...],  },...]           #surface with faces
#
#        return np.array(nodes)



    #**classFunction: import nodes and elements from Abaqus input file and create surface elements
    #node numbers in elements are converted from 1-based indices to python's 0-based indices
    #only works for Hex8, Hex20, Tet4 and Tet10 (C3D4, C3D8, C3D10, C3D20) elements
    #return node numbers as numpy array
    def ImportFromAbaqusInputFile(self, fileName, typeName='Part', name='Part-1', verbose=False):
        #def ImportABAQUS(fileName, typeName, name, verbose = False):
        fileLines = []
        if verbose: print("ImportFromAbaqusInputFile: read file name=", fileName)
        file=open(fileName,'r') 
        fileLines = file.readlines()
        file.close()
        
        if verbose: print("read", len(fileLines), "lines")
    
        lineCnt = 0          #current counter for lines, makes life simpler
        nLines = len(fileLines)
        typeNameFound = False
        elementsDict = {'Name':'elements'} #this is the destination for elements
        #+++++++++++++++++++++++++++++++++++++++++++++
        #find *Instance or *Part
        strTypeName = '*'+typeName
    
        while lineCnt < nLines and not typeNameFound:
            line = fileLines[lineCnt]
            if line[0:len(strTypeName)] == strTypeName:
                typeNameFound = True
                if verbose: print("found ", strTypeName, "in line", lineCnt)
                
            lineCnt += 1
    
        if not typeNameFound: raise ValueError("ImportFromAbaqusInputFile: did not find keyword '"+strTypeName+"'")
    
        #+++++++++++++++++++++++++++++++++++++++++++++
        #read *Node keyword
        if fileLines[lineCnt][0:5] == '*Node':
            lineCnt += 1
        else:
            raise ValueError("ImportFromAbaqusInputFile: expected *Node in line"+str(lineCnt+1)+', but received: '+fileLines[lineCnt])
        
        #+++++++++++++++++++++++++++++++++++++++++++++
        #read nodes:
        nodeReadFinished = False
        nodes=[]
        while lineCnt < nLines and not nodeReadFinished:
            line = fileLines[lineCnt]
        
            if line[0] != '*': #check if nodes section has finished
                lineData = line.split(',') #split into values
                if len(lineData) != 4:
                    raise ValueError("ImportFromAbaqusInputFile: Expected node number and 3 coordinates, line "+str(lineCnt))
                else:
                    v = []
                    for i in range(3):
                        v+=[float(lineData[i+1])] 
                    nodes += [v] #add node data
                    #if verbose: print("node=",v)
                lineCnt += 1#do not increase counter if * found
            else:
                nodeReadFinished = True

        if verbose: print("imported ", len(nodes), "nodes")
    
        #+++++++++++++++++++++++++++++++++++++++++++++
        #read *Element keyword
        #expect something like: *Element, type= C3D20 
        availableElementTypesNodes={'C3D20':20, 'C3D8':8}
        elementTypeConversion={'C3D20':'Hex20', 'C3D8':'Hex8', 'C3D4':'Tet4', 'C3D10':'Tet10'}
        elementSectionFound = False
        elementTypeName = ''
        while lineCnt < nLines and not elementSectionFound:
            line = fileLines[lineCnt]
            #print("now=", line)
            if line[0:len('*Element')] == '*Element':
                elementSectionFound = True
                elementTypeName=line.split(',')[1].split('=')[1].strip()
                if verbose: print("found *Element in line", lineCnt, 'element type=',elementTypeName)
                
                if not (elementTypeName in availableElementTypesNodes):
                    raise ValueError("ImportFromAbaqusInputFile: element type '"+elementTypeName+"' can not yet be imported")
                
            lineCnt += 1
    
        if not elementSectionFound: 
            raise ValueError("ImportFromAbaqusInputFile: did not find keyword *Element ")
    
        elementType= elementTypeConversion[elementTypeName]
        if not(elementTypeName in elementsDict):
            elementsDict[elementType] = []
    
        #+++++++++++++++++++++++++++++++++++++++++++++
        #read elements:
        elementReadFinished = False
        nElementNodes = availableElementTypesNodes[elementTypeName] #this is the expected number of nodes for element type
        elementCnt = 0
        while lineCnt < nLines and not elementReadFinished:
            line = fileLines[lineCnt]
            #print("read element line:",line)
        
            if line[0] != '*': #check if element section has finished
                lineStr = line.strip() #cut spaces at end in order to detect ',' at end
                while lineStr[-1] == ',' and lineCnt < nLines:
                    lineCnt += 1
                    line = fileLines[lineCnt]
                    if line[0] == '*': 
                        raise ValueError("ImportFromAbaqusInputFile: while reading elements, got invalid format of line "+str(lineCnt+1))
                    lineStr += line
                    #print("   extended line:",lineStr)
                    
                lineData = lineStr.strip().split(',') #split into values
                
                if len(lineData) != nElementNodes+1:
                    raise ValueError("ImportFromAbaqusInputFile: Expected element and "+str(nElementNodes)+" node numbers in line "+str(lineCnt))
                else:
                    v = []
                    for i in range(nElementNodes):
                        v+=[int(lineData[i+1])-1] #changed from float
                    elementsDict[elementType] += [v]
                    elementCnt += 1
            else:
                elementReadFinished = True
            lineCnt += 1
        
        if verbose: print("imported ", elementCnt, "elements")

        #FEMinterface:
        self.elements += [elementsDict]
        self.nodes['Position'] = np.array(nodes)


        #convert elements to triangles for drawing:
        trigList = []
        if 'Hex8' in elementsDict:
            for element in elementsDict['Hex8']:
                trigList += ConvertHexToTrigs(element) #node numbers are already 0-based
        if 'Hex20' in elementsDict:
            for element in elementsDict['Hex20']:
                trigList += ConvertHexToTrigs(element) #node numbers are already 0-based

        self.surface += [{'Name':'meshSurface', 'Trigs':trigList}]    # [{'Name':'identifier', 'Trigs':[[n0,n1,n2],...], 'Quads':[[n0,...,n3],...],  },...]           #surface with faces

        #VolumeToSurfaceElements(verbose) #create surface from imported elements

        return np.array(nodes)
        
        #return [np.array(nodes), elementsDict]




    #**classFunction: read mass matrix from compressed row text format (exported from Abaqus); in order to export system matrices, write the following lines in your Abaqus input file:
    #*STEP
    #*MATRIX GENERATE, STIFFNESS, MASS
    #*MATRIX OUTPUT, STIFFNESS, MASS, FORMAT=COORDINATE
    #*End Step
    def ReadMassMatrixFromAbaqus(self, fileName, type='SparseRowColumnValue'):
        self.massMatrix = np.loadtxt(fileName)
        self.massMatrix[:,0] -= 1 #convert 1-based indices to 0-based indices
        self.massMatrix[:,1] -= 1

    #**classFunction: read stiffness matrix from compressed row text format (exported from Abaqus)
    def ReadStiffnessMatrixFromAbaqus(self, fileName, type='SparseRowColumnValue'):
        self.stiffnessMatrix = np.loadtxt(fileName)
        self.stiffnessMatrix[:,0] -= 1 #convert 1-based indices to 0-based indices
        self.stiffnessMatrix[:,1] -= 1



    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: import mesh from NETGEN/NGsolve and setup mechanical problem
    #**notes: The interface to NETGEN/NGsolve has been created together with Joachim Schöberl, main developer 
    #  of NETGEN/NGsolve; Thank's a lot!
    #  download NGsolve at: https://ngsolve.org/
    #  NGsolve needs Python 3.7 (64bit) ==> use according EXUDYN version!
    #  note that node/element indices in the NGsolve mesh are 1-based and need to be converted to 0-base!
    #**input:
    #    mesh: a previously created \texttt{ngs.mesh} (NGsolve mesh, see examples)
    #    youngsModulus: Young's modulus used for mechanical model
    #    poissonsRatio: Poisson's ratio used for mechanical model
    #    density: density used for mechanical model
    #    verbose: set True to print out some status information
    def ImportMeshFromNGsolve(self, mesh, density, youngsModulus, poissonsRatio, verbose = False):
        import ngsolve as ngs
        meshOrder = 1#currently only order 1 possible, but will change!

        if verbose: print("NGsolve create mechanics FE space ...")
        fes = ngs.VectorH1(mesh, order=meshOrder)

        #create finite element spaces for mass matrix and stiffness matrix    
        u = fes.TrialFunction()
        v = fes.TestFunction()
        fesK = ngs.BilinearForm(fes)
        fesM = ngs.BilinearForm(fes)

        def sigma(eps, mu, lam):
            return 2*mu*eps + lam*ngs.Trace(eps) * ngs.Id(eps.dims[0])

        E = youngsModulus
        nu = poissonsRatio
        rho = density

        mu  = E / 2 / (1+nu) #Lame parameters
        lam = E * nu / ((1+nu)*(1-2*nu))

        #setup (linear) mechanical FE-space
        fesK += ngs.InnerProduct(sigma(ngs.Sym(ngs.Grad(u)),mu,lam), ngs.Sym(ngs.Grad(v)))*ngs.dx
        fesM += rho*u*v * ngs.dx

        with ngs.TaskManager():
            if verbose: print ("NGsolve assemble M and K")
            fesK.Assemble()
            fesM.Assemble()

        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #export to scipy sparse matrices:
        from scipy.sparse import csr_matrix
        if verbose: print ("NGsolve convert system matrices to scipy csr_matrix format")
        K = csr_matrix( fesK.mat.CSR(), copy=True )
        M = csr_matrix( fesM.mat.CSR(), copy=True )
        if verbose: print("K.shape=",K.shape)

        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #get node, element and surface list:
        nodeList=[]
        tetList=[]
        surfaceTriangleList=[] #for drawing
        NE = mesh.ne
        if verbose: print("number of tets=", NE)

        if meshOrder == 1:
            NP = len(mesh.vertices)
            if verbose: print("number of points=", NP)
            for n in mesh.vertices: 
                nodeList+=[list(n.point)]
        else:
            NP = len(ngmesh.Points())
            if verbose: print("number of points=", NP)
    
            for n in ngmesh.Points(): 
                nodeList+=[list(n)]

    
        #extract 3D elements from mesh:
        tets = mesh.ngmesh.Elements3D()
        cnt=0
        for el in tets:
            if len(el.vertices) == 4: #check if element is a tet
                tetIndices=[]
                for i in el.vertices:
                    tetIndices+=[int(i.nr)-1] #convert to 0-base
                tetList += [tetIndices]
            else:
                print("ERROR in ImportMeshFromNGsolve: invalid element in ngmesh, elementNr=", cnt, "linear tet elements required!")
            cnt+=1

        surface = mesh.ngmesh.Elements2D() #surface mesh
        for st in surface: 
            vertices = []
            for v in st.vertices:
                vertices += [v.nr-1] #convert to 0-based indices
            x=vertices[2] #flip vertices for correct orientation in EXUDYN
            vertices[2] = vertices[1]
            vertices[1] = x
            surfaceTriangleList += [vertices]

        nodes = np.array(nodeList)
        elements=np.array(tetList) #unused
        trigList = surfaceTriangleList

        #convert csr_matrix in NGsolve to exudyn sparse CSR np.array:
        M1 = ScipySparseCSRtoCSR(M)
        K1 = ScipySparseCSRtoCSR(K)

        nMK = M.shape[0] #get size of mass matrix; assume square matrix!
        #NGsolve sorts indices as x0x1x2...y0y1y2...z0z1z2...., but is needed as x0y0z0x1y1z1...
        ResortIndicesOfCSRmatrix(M1, nMK)
        ResortIndicesOfCSRmatrix(K1, nMK)

        self.nodes = {'Position':nodes}
        self.elements = [{'Name':'NGsolve','Tet4':elements}]
        self.massMatrix = M1 
        self.stiffnessMatrix = K1 
        self.surface = [{'Name':'meshSurface','Trigs':trigList}]


    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #general FEMinterface functionality


    #**classFunction: get sparse mass matrix in according format
    def GetMassMatrix(self, sparse=True):
        if sparse:
            return self.massMatrix
        else:
            return CompressedRowSparseToDenseMatrix(self.massMatrix)

    #**classFunction: get sparse stiffness matrix in according format
    def GetStiffnessMatrix(self, sparse=True):
        if sparse:
            return self.stiffnessMatrix
        else:
            return CompressedRowSparseToDenseMatrix(self.stiffnessMatrix)

    #**classFunction: get total number of nodes
    def NumberOfNodes(self):
        nNodes = 0
        for nodeTypeName in self.nodes:
            nNodes += len(self.nodes[nodeTypeName])
        return nNodes

    #**classFunction: get node points as array; only possible, if there exists only one type of Position nodes
    def GetNodePositionsAsArray(self):
        if len(self.nodes) != 1:
            raise ValueError("ERROR: GetNodePositionsAsArray() only possible for one type of Position nodes!")

        nodeTypeName = list(self.nodes)[0]
        return self.nodes[nodeTypeName]

    #**classFunction: get number of total nodal coordinates
    def NumberOfCoordinates(self):
        nCoordinates = 0
        for nodeTypeName in self.nodes:
            nCoordinates += len(self.nodes[nodeTypeName]) * self.coordinatesPerNodeType[nodeTypeName]
        return nCoordinates

    #**classFunction: get node number for node at given point, e.g. p=[0.1,0.5,-0.2], using a tolerance (+/-) if coordinates are available only with reduced accuracy
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

    #**classFunction: get node numbers in plane defined by point p and (normalized) normal vector n using a tolerance for the distance to the plane
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

    #**classFunction: get node numbers on a circle, by point p, (normalized) normal vector n (which is the axis of the circle) and radius r
    #using a tolerance for the distance to the plane
    #if not found, it returns an empty list
    def GetNodesOnCircle(self, point, normal, r, tolerance = 1e-5):
        cnt = 0
        nodeList=[]
        for nodeTypeName in self.nodes:
            for nodePoint in self.nodes[nodeTypeName]:
                if abs(np.dot(nodePoint - point, normal)) <= tolerance:
                    if abs(np.dot(nodePoint - point, nodePoint - point) - r**2) <= tolerance**2:
                        nodeList += [cnt]
                cnt+=1
        return nodeList

    #**classFunction: return surface trigs as node number list (for drawing in EXUDYN)
    def GetSurfaceTriangles(self):
        trigList = []
        for surface in self.surface:
            if 'Trigs' in surface:
                trigList += surface['Trigs']
        return trigList

    #**classFunction: generate surface elements from volume elements
    #stores the surface in self.surface
    #only works for one element list and one type ('Hex8') of elements
    def VolumeToSurfaceElements(self, verbose=False):
        if verbose: print("create surface from volume elements")
#        self.elements = []              # [{'Name':'identifier', 'Tet4':[[n0,n1,n2,n3],...], 'Hex8':[[n0,...,n7],...],  },...]        #there may be several element sets
#        self.surface = []               # [{'Name':'identifier', 'Trigs':[[n0,n1,n2],...], 'Quads':[[n0,...,n3],...],  },...]           #surface with faces
        nNodes = self.NumberOfNodes()
        hex8QuadIndices = [[0,1,2,3],[7,6,5,4],[0,4,5,1],[1,5,6,2],[2,6,7,3],[3,7,4,0]]
        hex20QuadIndices = [[0,1,2,3],[7,6,5,4],[0,4,5,1],[1,5,6,2],[2,6,7,3],[3,7,4,0]] #without midnodes, works for now
        tet4TrigIndices = [[0,1,2],[0,3,1],[1,3,2],[2,3,0]]
        tet10TrigIndices = [[0,1,2],[0,3,1],[1,3,2],[2,3,0]] #without midnodes, works for now
        
        elementTypes = ['Hex8','Hex20','Tet4','Tet10']

        element2IndexList = {'Hex8': hex8QuadIndices,
                             'Hex20':hex20QuadIndices,
                             'Tet4': tet4TrigIndices,
                             'Tet10':tet10TrigIndices}
        surfaceListTrigs = []
        surfaceListQuads = [] #list of surface quads, will be converted to trigs
        nodes2elements = [[]]*nNodes #element to node list

        #first store all elements linked to a certain node
        #print("build nodes to elements...")
        #print("nodes2elements=",nodes2elements)

        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #preprocess and generate nodes2elements list
        cnt = 0
        for elementDict in self.elements:
            for elementType in elementTypes:
                if elementType in elementDict:   #only implemented for Hex8
                    elementList = elementDict[elementType]
                    for element in elementList:
                        #print("  element",cnt,"=",element)
                        for i in element:
                            #print("    i=",i)
                            #nodes2elements[int(i)].append(int(cnt))
                            alist=list(nodes2elements[i])
                            alist.append(cnt)
                            nodes2elements[i] = alist
                            #print("nodes2elements[",i,"]=",nodes2elements[i])
                        cnt+=1

        
        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        for elementDict in self.elements:
            for elementType in elementTypes:
                if elementType in elementDict:   #only implemented for Hex8
                    elementList = elementDict[elementType]
                    currentIndexList = element2IndexList[elementType]
                    
                    #run through all elements:
                    nElements = len(elementList)
                    elementCnt = 0
                    cnt2=0
                    #now run over all elements:
                    for element in elementList:
                        if verbose and elementCnt%10000 == 0 and elementCnt>0:
                            print("process element ",elementCnt,"/",nElements)
                        for surface in currentIndexList:
                            #test a surface with nodes
                            actSurface = []
                            for i in surface:
                                actSurface += [element[i]]
                            lenSurface = len(actSurface) #3 or 4 nodes at surface
                            #print("  actsurface=",actSurface)
                            #find all potential candidates, which could be opposite
    
                            testElements = []
                            for nn in actSurface:
                                for elNum in nodes2elements[nn]:
                                    if not (elNum in testElements):
                                        testElements += [elNum]
                            
                            #print("  testElements=",testElements)
                            foundNeighbor = False
                            for el in testElements:
                                if el != elementCnt: #do not compare with itself!!!
                                    testElement = elementList[el]
                                    #print("    testElement=",testElement)
                                    for surface2 in currentIndexList:
                                        cnt2+=1
                                        #print("      surface2=",surface2)
                                        #test a surface2 with nodes
                                        testSurface = []
                                        for j in surface2:
                                            testSurface += [testElement[j]]
                                        #print("      testSurface=",testSurface)
                                        
                                        if len(set(testSurface).intersection(actSurface)) == lenSurface:
                                            #print("        found!")
                                            foundNeighbor = True
                                            break;
                            
                            if not foundNeighbor:
                                #print("      not found!")
                                if lenSurface == 4:
                                    surfaceListQuads += [actSurface]
                                else:
                                    surfaceListTrigs += [actSurface]
                        elementCnt += 1
        
        if verbose: print("surfaceListQuad length=",len(surfaceListQuad))
        for quad in surfaceListQuads:
            surfaceListTrigs += [[quad[0],quad[1],quad[2]]]
            surfaceListTrigs += [[quad[0],quad[2],quad[3]]]

        #print("surfaceListTrigs",surfaceListTrigs)
        #find if surface exists:
        surfaceExists = False
        for surf in self.surface:
            if surf['Name'] == 'meshSurface':
                surfaceExists = True
                surf['Trigs'] = surfaceListTrigs

        #otherwise add new surface
        if not surfaceExists:
            self.surface += [{'Name':'meshSurface', 'Trigs':surfaceListTrigs}]
                 
        
        
        
        
        
        
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #+++++++++++      COMPUTATIONAL FUNCTIONS            ++++++++++++++++++++++
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            
    #**classFunction: get gyroscopic matrix in according format; rotationAxis=[0,1,2] = [x,y,z]
    def GetGyroscopicMatrix(self, rotationAxis=2, sparse=True):
        if sparse:
            raise ValueError("GetGyroscopicMatrix: not implemented for sparse matrices!")
        else:
            #create gyroscopic terms
            nNodes = self.NumberOfNodes()
            if rotationAxis == 0:
                X=np.array([[ 0.,  0.,  0.],
                            [ 0.,  0., -1.],
                            [ 0.,  1.,  0.]])
            elif rotationAxis == 1:
                X=np.array([[ 0.,  0.,  1.],
                            [ 0.,  0.,  0.],
                            [-1.,  0.,  0.]])
            elif rotationAxis == 2:
                X=np.array([[ 0., -1.,  0.],
                            [ 1.,  0.,  0.],
                            [ 0.,  0.,  0.]])
            xBlock = np.kron(np.eye(nNodes), X) #create big block-diagonal matrix
            G=np.dot(xBlock,CompressedRowSparseToDenseMatrix(self.massMatrix))

            return G



    #**classFunction: scale (=multiply) mass matrix with factor
    def ScaleMassMatrix(self, factor):
        self.massMatrix[:,2] *= factor

    #**classFunction: scale (=multiply) stiffness matrix with factor
    def ScaleStiffnessMatrix(self, factor):
        self.stiffnessMatrix[:,2] *= factor

        
    #**classFunction: modify stiffness matrix to add elastic support (joint, etc.) to a node; nodeNumber zero based (as everywhere in the code...)
    #springStiffness must have length according to the node size
    def AddElasticSupportAtNode(self, nodeNumber, springStiffness=[1e8,1e8,1e8]):
        if len(self.nodes) != 1:
            print("ERROR: AddElasticSupportAtNode: there must be exactly one list of nodes!")
        #nodeList = self.nodes(list(self.nodes)[0])
        nodeTypeName = list(self.nodes)[0]
        nodeSize = self.coordinatesPerNodeType[nodeTypeName]
        nCoordinate = nodeNumber * nodeSize
        #print("AddElasticSupportAtNode, nCoordinate=", nCoordinate)
        
        #supports = []
        for i in range(nodeSize):
            #supports += [[nCoordinate+i,nCoordinate+i,springStiffness[i]]]
            self.stiffnessMatrix = AddEntryToCompressedRowSparseArray(self.stiffnessMatrix, nCoordinate+i,nCoordinate+i,springStiffness[i])
        #np.vstack((self.stiffnessMatrix, np.array(supports))) #append supports to sparse matrix

    #**classFunction: modify mass matrix by adding a mass to a certain node, modifying directly the mass matrix
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
            self.massMatrix = AddEntryToCompressedRowSparseArray(self.massMatrix, nCoordinate+i,nCoordinate+i,addedMass)
        #np.vstack((self.massMatrix, np.array(supports))) #append supports to sparse matrix

    #**classFunction: compute nModes smallest eigenvalues and eigenmodes from mass and stiffnessMatrix
    #store mode vector in modeBasis, but exclude a number of 'excludeRigidBodyModes' rigid body modes from modeBasis
    #if excludeRigidBodyModes > 0, then the computed modes is nModes + excludeRigidBodyModes, from which excludeRigidBodyModes smallest eigenvalues are excluded
    def ComputeEigenmodes(self, nModes, excludeRigidBodyModes = 0, useSparseSolver = True):
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
            #optional, using shift-invert mode; DOES NOT WORK:
            #guess for smallest eigenvalue:
            #n=self.NumberOfCoordinates()
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
            self.eigenValues = abs(eigVals[excludeRigidBodyModes:excludeRigidBodyModes + nModes])

    #**classFunction: return list of eigenvalues in Hz of previously computed eigenmodes
    def GetEigenFrequenciesHz(self):
        return np.sqrt(self.eigenValues)/(2.*np.pi)


    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: compute Campbell diagram for given mechanical system
    #create a first order system Axd + Bx = 0 with x= [q,qd]' and compute eigenvalues
    #takes mass M, stiffness K and gyroscopic matrix G from FEMinterface
    #currently only uses dense matrices, so it is limited to approx. 5000 unknowns!
    #**input:
    #  terminalFrequency: frequency in Hz, up to which the campbell diagram is computed
    #  nEigenfrequencies: gives the number of computed eigenfrequencies(modes), in addition to the rigid body mode 0
    #  frequencySteps: gives the number of increments (gives frequencySteps+1 total points in campbell diagram)
    #  rotationAxis:[0,1,2] = [x,y,z] provides rotation axis
    #  plotDiagram: if True, plots diagram for nEigenfrequencies befor terminating
    #  verbose: if True, shows progress of computation
    #**output: [listFrequencies, campbellFrequencies]
    #  listFrequencies: list of computed frequencies
    #  campbellFrequencies: array of campbell frequencies per eigenfrequency of system
    def ComputeCampbellDiagram(self, terminalFrequency, nEigenfrequencies=10, frequencySteps=25, 
                               rotationAxis=2, plotDiagram=False, verbose=False):
        from scipy.linalg import eig #eigh for symmetric matrices, positive definite
        
        #create gyroscopic terms
#        X=np.array([[ 0., -1.,  0.],
#                    [ 1.,  0.,  0.],
#                    [ 0.,  0.,  0.]])
#        xBlock = np.kron(np.eye(nNodes), X) #create big block-diagonal matrix
#        G=np.dot(xBlock,M)
        
        M = self.GetMassMatrix(sparse=False)
        K = self.GetStiffnessMatrix(sparse=False)
        G = self.GetGyroscopicMatrix(rotationAxis=2, sparse=False)

        nODE = self.NumberOfCoordinates()
        #nNodes = self.NumberOfNodes()
        B = np.block([[                    K, np.zeros((nODE,nODE))],
                      [np.zeros((nODE,nODE)), -M                   ]])
    
#        terminalFrequencyCampbell = 2*np.pi*225 #rad/s
        campbellFrequencies = []
        listFrequencies = []

        for val in range(frequencySteps+1):
            
            omega = val * terminalFrequency * 2*np.pi / frequencySteps
            if verbose:
                print("compute Campbell for frequency =", round(omega/(2*np.pi),3), " / ", terminalFrequency, '(Hz)')
            A = np.block([[omega * G, M                    ],
                          [        M, np.zeros((nODE,nODE))]])
    
        
            Amod = -np.dot(np.linalg.inv(A),B)
            #print("Amod =", Amod)
            [eigValues, eigVector] = eig(Amod) #this gives omega^2 ... squared eigen frequencies (rad/s)
        
            ev = np.sort(eigValues)
        
            listEigImag = []
            for i in range(len(ev)):
                v=abs(ev[i].imag/(2*np.pi))
                if not (v in listEigImag):
                    listEigImag += [v]

            listEigImag = np.sort(listEigImag)
        
            campbellFrequencies += [list(listEigImag[0:nEigenfrequencies+1])] #+1 for rigid body mode 0
            listFrequencies += [omega/(2*np.pi)]

        if plotDiagram:
            import matplotlib.pyplot as plt
            import matplotlib.ticker as ticker
        
            campbellFrequencies = np.array(campbellFrequencies)

            nPlotFrequencies = nEigenfrequencies
            if nEigenfrequencies > 27:
                nPlotFrequencies = 27
                print("only 28 eigenfrequencies can be plotted!")
            
            for i in range(nPlotFrequencies): #do not plot rigid body mode 0
                plt.plot(listFrequencies, campbellFrequencies[:,i+1], PlotLineCode(i+1), label='freq '+str(i))

            plt.plot(listFrequencies, listFrequencies, PlotLineCode(0), label='rotor speed (Hz)') 
            
            ax=plt.gca() # get current axes
            ax.grid(True, 'major', 'both')
            ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
            ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
            plt.xlabel("excitation / rotor frequency (Hz)")
            plt.ylabel("eigen frequency (Hz)")
            plt.tight_layout()
            plt.legend()
            plt.show() 

        return [listFrequencies, campbellFrequencies]

    #**classFunction: perform some consistency checks
    def CheckConsistency(self):
        nNodes = self.NumberOfNodes()
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
                

    
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Ansys    
    
    #**classFunction: read mass matrix from CSV format (exported from Ansys)
    def ReadMassMatrixFromAnsys(self, fileName, dofMappingVectorFile, sparse=True, verbose=False):
        if sparse:
            self.massMatrix = ReadMatrixFromAnsysMMF(fileName, verbose)
        else:
            M = np.loadtxt(fileName,delimiter=',')
            self.massMatrix = ConvertDenseToCompressedRowMatrix(M)

        sorting = ReadMatrixDOFmappingVectorFromAnsysTxt(dofMappingVectorFile)
        if len(sorting) < max(self.massMatrix[:,0]):
            raise ValueError("ReadMassMatrixFromAnsys: dofMappingVectorFile and matrix size do not fit")

        MapSparseMatrixIndices(self.massMatrix, sorting)

    #**classFunction: read stiffness matrix from CSV format (exported from Ansys)
    def ReadStiffnessMatrixFromAnsys(self, fileName, dofMappingVectorFile, sparse=True, verbose=False):
        if sparse:
            self.stiffnessMatrix = ReadMatrixFromAnsysMMF(fileName, verbose) 
        else:
            K = np.loadtxt(fileName,delimiter=',')
            self.stiffnessMatrix = ConvertDenseToCompressedRowMatrix(K)

        sorting = ReadMatrixDOFmappingVectorFromAnsysTxt(dofMappingVectorFile)
        if len(sorting) < max(self.stiffnessMatrix[:,0]):
            raise ValueError("ReadStiffnessMatrixFromAnsys: dofMappingVectorFile and matrix size do not fit")

        MapSparseMatrixIndices(self.stiffnessMatrix, sorting)
                    
    #**classFunction: read nodal coordinates (exported from Ansys as .txt-File)
    def ReadNodalCoordinatesFromAnsys(self, fileName, verbose=False):
        nodes = ReadNodalCoordinatesFromAnsysTxt(fileName, verbose)
        self.nodes['Position'] = np.array(nodes)
        
    #**classFunction: read elements (exported from Ansys as .txt-File)
    def ReadElementsFromAnsys(self, fileName, verbose=False):
        self.elements += [ReadElementsFromAnsysTxt(fileName, verbose)]
        self.VolumeToSurfaceElements() #generate surface elements

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
