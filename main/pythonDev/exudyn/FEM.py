#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Support functions and helper classes for import of meshes, finite element models (ABAQUS, ANSYS, NETGEN) and for generation of FFRF (floating frame of reference) objects.
#
# Author:   Johannes Gerstmayr; Stefan Holzinger (Abaqus and Ansys import utilities); Joachim Sch\"oberl (support for NGsolve import and eigen computations)
# Date:     2020-03-10 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# Notes: 	internal CSR matrix storage format contains 3 float numbers per row: [row, column, value], can be converted to scipy csr sparse matrices with function CSRtoScipySparseCSR(...)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#constants and fixed structures:
from exudyn.itemInterface import *
from exudyn.utilities import RoundMatrix, ComputeSkewMatrix, FillInSubMatrix, PlotLineCode, GetRigidBodyNode
from exudyn.rigidBodyUtilities import AngularVelocity2EulerParameters_t, EulerParameters2GLocal, RotationVector2GLocal, RotXYZ2GLocal, RotXYZ2GLocal_t, Skew
import numpy as np #LoadSolutionFile
from enum import Enum #for class HCBstaticModeSelection

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

#**function: resort indices of given NGsolve CSR matrix in XXXYYYZZZ format to XYZXYZXYZ format; numberOfRows must be equal to columns
#needed for import from NGsolve
def ResortIndicesOfCSRmatrix(mXXYYZZ, numberOfRows):
    #compute resorting index array [0,1,2, 3,4,5, 6,7,8] ==> [0,3,6, 1,4,7, 2,5,8]
    if numberOfRows%3 != 0:
        raise ValueError("ResortIndicesOfCSRmatrix: numberOfRows must be multiple of 3")
    
    #compute transformation of indices:
    r = np.arange(numberOfRows).reshape((int(numberOfRows/3),3))
    r = r.T.flatten()
    
    mXXYYZZ[:,0] = r[mXXYYZZ[:,0].astype(int)]
    mXXYYZZ[:,1] = r[mXXYYZZ[:,1].astype(int)]

    # nSparse = len(mXXYYZZ)
    # for i in range(nSparse): #for loop is slow, but works ok for 100.000 DOF
    #     mXXYYZZ[i,0] = r[int(mXXYYZZ[i,0])]
    #     mXXYYZZ[i,1] = r[int(mXXYYZZ[i,1])]
    
#**function: resort indices of given NGsolve vector in XXXYYYZZZ format to XYZXYZXYZ format
def ResortIndicesOfNGvector(vXXYYZZ):
    #compute resorting index array [0,1,2, 3,4,5, 6,7,8] ==> [0,3,6, 1,4,7, 2,5,8]
    v = np.array(vXXYYZZ)
    numberOfRows = len(v)
    if numberOfRows%3 != 0:
        raise ValueError("ResortIndicesOfNGvector: length must be multiple of 3")

    vNew = np.zeros(numberOfRows)

    #compute transformation of indices:
    r = np.arange(numberOfRows).reshape((int(numberOfRows/3),3))
    r = r.T.flatten()
    
    # for i in range(numberOfRows): #for loop is slow, but works ok for 100.000 DOF
    #     vNew[r[i]] = v[i]
    vNew[r[:]] = v[:]

    return vNew
    
#**function: resort indices of given Exudyun vector XYZXYZXYZ to NGsolve vector in XXXYYYZZZ format
def ResortIndicesExudyn2NGvector(vXYZXYZ):
    #compute resorting index array [0,1,2, 3,4,5, 6,7,8] ==> [0,3,6, 1,4,7, 2,5,8]
    v = np.array(vXYZXYZ)
    numberOfRows = len(v)
    if numberOfRows%3 != 0:
        raise ValueError("ResortIndicesOfNGvector: length must be multiple of 3")

    vNew = np.zeros(numberOfRows)

    #compute transformation of indices:
    r = np.arange(numberOfRows).reshape((int(numberOfRows/3),3))
    r = r.T.flatten()
    
    # for i in range(numberOfRows): #for loop is slow, but works ok for 100.000 DOF
    #     vNew[i] = v[r[i]]
    vNew[:] = v[r[:]]

    return vNew
    



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
#**notes:
#   A MMF file can be created in Ansys by placing the following APDL code inside
#   the solution tree in Ansys Workbench:
#
#   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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
#   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#
# In case a lumped mass matrix is needed, place the following APDL Code inside 
# the Modal Analysis Tree:
#
#   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#   ! APDL code to force Ansys to use a lumped mass formulation (if available for
#   ! used elements)
#   LUMPM, ON, , 0
#   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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





#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
#**class: material base class, e.g., for FiniteElement
class MaterialBaseClass:
    def __init__(self, youngsModulus, poissonsRatio, density):
        self.youngsModulus = youngsModulus
        self.poissonsRatio = poissonsRatio
        self.density = density

#**class: class for representation of Kirchhoff (linear elastic, 3D and 2D) material
#**notes: use planeStress=False for plane strain
class KirchhoffMaterial(MaterialBaseClass):
    def __init__(self, youngsModulus, poissonsRatio, density = 0, planeStress = True):
        super().__init__(youngsModulus, poissonsRatio, density)
        self.planeStress = planeStress

        Em = self.youngsModulus
        nu = self.poissonsRatio

        lam = nu*Em / ((1 + nu) * (1 - 2*nu))
        mu = Em / (2*(1 + nu))
        self.elasticityTensor = np.array([[lam + 2*mu, lam, lam, 0, 0, 0],
                              [lam, lam + 2*mu, lam, 0, 0, 0],
                              [lam, lam, lam + 2*mu, 0, 0, 0],
                              [0, 0, 0, mu, 0, 0],
                              [0, 0, 0, 0, mu, 0],
                              [0, 0, 0, 0, 0, mu]])

        if self.planeStress:
            self.elasticityTensor2D = Em/(1-nu**2)*np.array([[1, nu, 0],
                                                            [nu, 1, 0],
                                                            [0, 0, (1-nu)/2]])
        else:
            self.elasticityTensor2D = np.array([[lam + 2*mu, lam, 0],
                                                [lam, lam + 2*mu, 0],
                                                [0, 0, mu]])


    #**classFunction: convert strain tensor into stress tensor using elasticity tensor
    def Strain2Stress(self, strain):
        E = strain
        strainVector = np.array([  E[0,0],   E[1,1],   E[2,2],
                                 2*E[1,2], 2*E[0,2], 2*E[0,1]])
        SV = self.StrainVector2StressVector(strainVector)
        S = np.array([[SV[0], SV[5], SV[4]],
                      [SV[5], SV[1], SV[3]],
                      [SV[4], SV[3], SV[2]]])
        return S

    #**classFunction: convert strain vector into stress vector
    def StrainVector2StressVector(self, strainVector):
        return self.elasticityTensor @ strainVector

    #**classFunction: compute 2D stress vector from strain vector
    def StrainVector2StressVector2D(self, strainVector2D):
        #E=strain
        #strainVector2D = np.array([E[0,0], E[1,1], 2*E[0,1]])
        SV = self.elasticityTensor2D @ strainVector2D
        #S = np.array([[SV[0], SV[2]], [SV[2], SV[1]]])
        return SV

    #**classFunction: compute Lame parameters from internal Young's modulus and Poisson ratio
    #**output: return vector [mu, lam] of Lame parameters
    def LameParameters(self):
        E = self.youngsModulus
        nu = self.poissonsRatio
        mu  = E / 2 / (1+nu) #Lame parameters
        lam = E * nu / ((1+nu)*(1-2*nu))
        return [mu, lam]

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
#**class: finite element base class for lateron implementations of other finite elements
class FiniteElement:
    def __init__(self, material):
        self.material = material
    

#**class: simplistic 4-noded tetrahedral interface to compute strain/stress at nodal points
class Tet4(FiniteElement):
    def __init__(self, material):
        super().__init__(material)
    
    #return (per node) linearized strain, linearized stress, reference B-matrix and deformation gradient
    def ComputeMatrices(self, nodalReferenceCoordinates, nodalDisplacements):
        #following routines implemented according to implementation in AMFE (TU-Munich):
        X1, Y1, Z1, X2, Y2, Z2, X3, Y3, Z3, X4, Y4, Z4 = nodalReferenceCoordinates
        Umat = nodalDisplacements.reshape(-1, 3)
        Xmat = nodalReferenceCoordinates.reshape(-1, 3)

        det = (-X1*Y2*Z3 + X1*Y2*Z4 + X1*Y3*Z2 - X1*Y3*Z4 - X1*Y4*Z2 + X1*Y4*Z3 
              + X2*Y1*Z3 - X2*Y1*Z4 - X2*Y3*Z1 + X2*Y3*Z4 + X2*Y4*Z1 - X2*Y4*Z3 
              - X3*Y1*Z2 + X3*Y1*Z4 + X3*Y2*Z1 - X3*Y2*Z4 - X3*Y4*Z1 + X3*Y4*Z2 
              + X4*Y1*Z2 - X4*Y1*Z3 - X4*Y2*Z1 + X4*Y2*Z3 + X4*Y3*Z1 - X4*Y3*Z2)

        #compute B matrix for reference coordinates
        B0 = 1/det*np.array([
            [-Y2*Z3 + Y2*Z4 + Y3*Z2 - Y3*Z4 - Y4*Z2 + Y4*Z3,
              X2*Z3 - X2*Z4 - X3*Z2 + X3*Z4 + X4*Z2 - X4*Z3,
              -X2*Y3 + X2*Y4 + X3*Y2 - X3*Y4 - X4*Y2 + X4*Y3],
            [ Y1*Z3 - Y1*Z4 - Y3*Z1 + Y3*Z4 + Y4*Z1 - Y4*Z3,
              -X1*Z3 + X1*Z4 + X3*Z1 - X3*Z4 - X4*Z1 + X4*Z3,
              X1*Y3 - X1*Y4 - X3*Y1 + X3*Y4 + X4*Y1 - X4*Y3],
            [-Y1*Z2 + Y1*Z4 + Y2*Z1 - Y2*Z4 - Y4*Z1 + Y4*Z2,
              X1*Z2 - X1*Z4 - X2*Z1 + X2*Z4 + X4*Z1 - X4*Z2,
              -X1*Y2 + X1*Y4 + X2*Y1 - X2*Y4 - X4*Y1 + X4*Y2],
            [ Y1*Z2 - Y1*Z3 - Y2*Z1 + Y2*Z3 + Y3*Z1 - Y3*Z2,
              -X1*Z2 + X1*Z3 + X2*Z1 - X2*Z3 - X3*Z1 + X3*Z2,
              X1*Y2 - X1*Y3 - X2*Y1 + X2*Y3 + X3*Y1 - X3*Y2]])

        #displacement gradient:
        grad = Umat.T @ B0
        #deformation gradient:
        #F = grad + np.eye(3)

        #linearized strain:
        linE = 0.5*(grad + grad.T)
        strainVector = np.array([  linE[0,0],   linE[1,1],   linE[2,2],
                                 2*linE[1,2], 2*linE[0,2], 2*linE[0,1]])
        
        stressVector = self.material.StrainVector2StressVector(strainVector)
        
        # strainVector4 = np.ones((4,1)) @ np.array([[linE[0,0],   linE[1,1],   linE[2,2],
        #                                             2*linE[1,2], 2*linE[0,2], 2*linE[0,1]]])
        strainVector4 = np.ones((4,1)) @ np.array([strainVector])

        stressVector4 = np.ones((4,1)) @ np.array([stressVector])

        #strainvector per node:
        return [strainVector4, stressVector4, B0, grad]







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


    #**classFunction: add according nodes, objects and constraints for FFRF object to MainSystem mbs; only implemented for Euler parameters
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
                      massProportionalDamping = 0, stiffnessProportionalDamping = 0,
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

        if (massProportionalDamping != 0 or massProportionalDamping != 0):
            dampingMatrixMC = exu.MatrixContainer()
            dampingMatrixMC.SetWithDenseMatrix(massProportionalDamping*CompressedRowSparseToDenseMatrix(self.massMatrixSparse)+
                                               stiffnessProportionalDamping*CompressedRowSparseToDenseMatrix(self.stiffnessMatrixSparse),useDenseMatrix=False)
        else:
            dampingMatrixMC=[]
        
        #add body for FFRF-Object:
        self.oFFRF = mbs.AddObject(ObjectFFRF(nodeNumbers = [self.nRigidBody] + self.nodeList, 
                                                            massMatrixFF=massMatrixMC,
                                                            stiffnessMatrixFF=stiffnessMatrixMC, 
                                                            dampingMatrixFF=dampingMatrixMC,
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
            mGroundCoordinate = mbs.AddMarker(MarkerNodeCoordinates(nodeNumber = nGround)) #Ground node ==> no action

            X1 = np.zeros((6, self.nODE2FFRF))
            X1rot = ComputeSkewMatrix(self.xRef).T @ self.massMatrixCSR
            X1[0:3,self.nODE2rigid:] = self.PhitTM
            X1[3:6,self.nODE2rigid:] = X1rot
            offset = np.zeros(6)
            offset[0:3] = self.PhitTM @ self.xRef #constrain current COM to reference COM

            #add constraint: X1*qObjectFFRF - offset = 0
            self.oRigidBodyConstraint = mbs.AddObject(CoordinateVectorConstraint(markerNumbers=[mGroundCoordinate, mObjectCoordinates], 
                                                                                 scalingMarker1 = X1, offset=offset))

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



#**function: compute current (max, min, ...) value for chosen ObjectFFRFreducedOrder object (CMSobject) with exu.OutputVariableType. The function operates on nodal values. This is a helper function, which can be used to conveniently compute output quantities of the CMSobject efficiently and to use it in sensors
#**input: 
#  mbs: MainSystem of objectNumber
#  objectNumber: number of ObjectFFRFreducedOrder in mbs
#  outputVariableType: a exu.OutputVariableType out of [StressLocal, DisplacementLocal, VelocityLocal]
#  norm: string containing chosen norm to be computed, out of 'Mises', 'maxNorm', 'min', 'max'; 'max' will return maximum of all components (component wise), 'min' does same but for minimum; 'maxNorm' computes np.linalg.norm for every node and then takes maximum of all norms; Mises computes von-Mises stress for every node and then takes maximum of all nodes
#  nodeNumberList: list of mesh node numbers (from FEMinterface); if empty [], all nodes are used; otherwise, only given nodes are evaluated
#**output: return value or list of values according to chosen norm as np.array
def CMSObjectComputeNorm(mbs, objectNumber, outputVariableType, norm='max', nodeNumberList=[]):
    import exudyn as exu
    #get generic node number containing current coordinates:
    nGeneric = mbs.GetObjectParameter(objectNumber,'nodeNumbers')[1]
    #problem with rigid body coordinates:
    #c = mbs.GetObjectOutputBody(objectNumber,variableType=exu.OutputVariableType.Coordinates,localPosition=[0,0,0])
    #c_t = mbs.GetObjectOutputBody(objectNumber,variableType=exu.OutputVariableType.Coordinates_t,localPosition=[0,0,0])
    
    #see which outputvariable to compute:
    if (mbs.GetObjectParameter(objectNumber,'outputVariableTypeModeBasis') == outputVariableType):
        #get current coordinates:
        c = mbs.GetNodeOutput(nGeneric,variableType=exu.OutputVariableType.Coordinates)
        X = mbs.GetObjectParameter(objectNumber,'outputVariableModeBasis')
        values = np.zeros((X.shape[0], 6))
        #compute stresses in nodes
        nc = len(c)
        for i in range(len(c)):
            for j in range(6): #6 stress components
                values[:,j] += X[:,6*i+j] * c[i] #
                
    elif outputVariableType == exu.OutputVariableType.DisplacementLocal:
        c = mbs.GetNodeOutput(nGeneric,variableType=exu.OutputVariableType.Coordinates)
        X = mbs.GetObjectParameter(objectNumber,'modeBasis')
        nn = int(X.shape[0]/3)
        values = (X@c).reshape((nn,3))
    elif outputVariableType == exu.OutputVariableType.VelocityLocal:
        c_t = mbs.GetNodeOutput(nGeneric,variableType=exu.OutputVariableType.Coordinates_t)
        X = mbs.GetObjectParameter(objectNumber,'modeBasis')
        nn = int(X.shape[0]/3)
        values = (X@c_t).reshape((nn,3))
    #elif outputVariableType == exu.OutputVariableType.AccelerationLocal:
    #    c_tt = mbs.GetNodeOutput(nGeneric,variableType=exu.OutputVariableType.Coordinates_tt)
    #    X = mbs.GetObjectParameter(objectNumber,'modeBasis')
    #    nn = int(X.shape[0]/3)
    #    values = (X@c_tt).reshape((nn,3))
    else:
        raise ValueError("CMSObjectComputeNorm: illegal outputVariableType")

    #restrict evaluation to given nodes
    if nodeNumberList != []:
        values = values[nodeNumberList]

    if norm == 'max':
        return np.max(values, axis=0)
    elif norm == 'min':
        return np.min(values, axis=0)
    elif norm == 'maxNorm':
        return np.max(np.linalg.norm(values,axis=1))
    elif norm == 'Mises':
        from exudyn.physics import VonMisesStress
        if outputVariableType != exu.OutputVariableType.StressLocal:
            raise ValueError("CMSObjectComputeMaximum: norm = 'Mises' only possible for outputVariableType == StressLocal")
        return np.max(VonMisesStress(values))

    raise ValueError("CMSObjectComputeMaximum: unknown norm")
    return 0


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
    #  roundStiffnessMatrix: use this value to set entries of reduced stiffness matrix to zero which are below the treshold
    def __init__(self, femInterface, rigidBodyNodeType = 'NodeType.RotationEulerParameters',
                 roundMassMatrix = 1e-13, roundStiffnessMatrix = 1e-13):
 
        self.femInterface = femInterface
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
        
        self.rigidBodyNodeType = rigidBodyNodeType
        if str(self.rigidBodyNodeType) == 'NodeType.RotationEulerParameters':
            self.nODE2rot = 4                       #Euler parameters
        else:
            self.nODE2rot = 3
        
        self.nODE2rigid = self.dim3D + self.nODE2rot
        self.nODE2FFRFreduced = self.nODE2rigid + self.nModes

        self.massMatrixFFRFreduced = np.zeros((self.nODE2FFRFreduced,self.nODE2FFRFreduced)) #create larger FFRF mass matrix

        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #FFRFreduced constant matrices:
        self.Phit = np.kron(np.ones(self.nNodes),np.eye(3)).T
        self.PhitTM = self.Phit.T @ massMatrixCSR #LARGE MATRIX COMPUTATION
        self.xRef = nodeArray.flatten()          #node reference values in single vector (can be added then to q[7:])
        self.xRefTilde = ComputeSkewMatrix(self.xRef) #rfTilde without q    
        self.inertiaLocal = self.xRefTilde.T @ massMatrixCSR @ self.xRefTilde #LARGE MATRIX COMPUTATION

        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #prepare CMS matrices (some already given):
        #according to: A. Zw\"olfer, J. Gerstmayr. The nodal-based floating frame of reference formulation with modal reduction, Acta Mechanica, 2020, submitted.
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

    #unused, because done separately in FEMinterface:  computeOutputVariableModeBasis: provide exudyn.OutputVariableType for postprocessing (set to 0 if unused); currently this is only available for linear tetrahedral elements and for exudyn.OutputVariableType = StrainLocal or StressLocal


    #**classFunction: add according nodes, objects and constraints for ObjectFFRFreducedOrder object to MainSystem mbs; use this function with userfunctions=0 in order to use internal C++ functionality, which is approx. 10x faster; implementation of userfunctions also available for rotation vector (Lie group formulation), which needs further testing
    #**input:
    #  exu: the exudyn module
    #  mbs: a MainSystem object
    #  positionRef: reference position of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
    #  initialVelocity: initial velocity of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
    #  rotationMatrixRef: reference rotation of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder); if [], it becomes the unit matrix
    #  initialAngularVelocity: initial angular velocity of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
    #  eulerParametersRef: DEPRECATED, use rotationParametersRef or rotationMatrixRef in future: reference euler parameters of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
    #  gravity: ONLY available if user functions are applied; otherwise use LoadMassProportional and add to ObjectFFRFreducedOrder; set [0,0,0] if no gravity shall be applied, or to the gravity vector otherwise
    #  UFforce: (OPTIONAL, computation is slower) provide a user function, which computes the quadratic velocity vector and applied forces; usually this function reads like:\\ \texttt{def UFforceFFRFreducedOrder(mbs, t, qReduced, qReduced\_t):\\ \phantom{XXXX}return cms.UFforceFFRFreducedOrder(exu, mbs, t, qReduced, qReduced\_t)}
    #  UFmassMatrix: (OPTIONAL, computation is slower) provide a user function, which computes the quadratic velocity vector and applied forces; usually this function reads like:\\ \texttt{def UFmassFFRFreducedOrder(mbs, t, qReduced, qReduced\_t):\\  \phantom{XXXX}return cms.UFmassFFRFreducedOrder(exu, mbs, t, qReduced, qReduced\_t)}
    #  massProportionalDamping: Rayleigh damping factor for mass proportional damping (multiplied with reduced mass matrix), added to floating frame/modal coordinates only
    #  stiffnessProportionalDamping: Rayleigh damping factor for stiffness proportional damping, added to floating frame/modal coordinates only (multiplied with reduced stiffness matrix)
    #  color: provided as list of 4 RGBA values
    def AddObjectFFRFreducedOrderWithUserFunctions(self, exu, mbs, 
                                                  positionRef=[0,0,0], 
                                                  initialVelocity=[0,0,0], 
                                                  rotationMatrixRef=[], 
                                                  initialAngularVelocity=[0,0,0],
                                                  gravity=[0,0,0],
                                                  UFforce=0, UFmassMatrix=0,
                                                  massProportionalDamping = 0, stiffnessProportionalDamping = 0,
                                                  color=[0.1,0.9,0.1,1.],
                                                  eulerParametersRef=[]):

        #check chosen rotation parameterization:
        if len(rotationMatrixRef) == 0 and len(eulerParametersRef) == 0:
            rotationMatrixRef=np.diag([1,1,1])
        elif len(rotationMatrixRef) != 0 and len(eulerParametersRef) != 0:
            raise ValueError('AddObjectFFRFreducedOrderWithUserFunctions: rotationMatrixRef or eulerParametersRef must be zero')
        
        if len(eulerParametersRef) != 0:
            if str(self.rigidBodyNodeType) != 'NodeType.RotationEulerParameters':
                raise ValueError('AddObjectFFRFreducedOrderWithUserFunctions: inconsistent reference rotation parameters and rigidBodyNodeType')
            #compute initial euler parameter velocities from angular velocity vector
            self.rotationParameters_t0 = AngularVelocity2EulerParameters_t(initialAngularVelocity, eulerParametersRef)
            self.rotationParameters0 = eulerParametersRef
    
            #rigid body node for ObjectFFRFreducedOrder
            self.nRigidBody = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=list(positionRef)+list(eulerParametersRef), 
                                                initialVelocities=list(initialVelocity)+list(self.rotationParameters_t0)))
            self.rigidBodyNodeType = exu.NodeType.RotationEulerParameters
        else:
            #compute initial rotation parameter velocities from angular velocity vector
            nodeItem = GetRigidBodyNode(nodeType=self.rigidBodyNodeType, position=positionRef, 
                                        velocity=initialVelocity, rotationMatrix=rotationMatrixRef, 
                                        angularVelocity=initialAngularVelocity)
            self.nRigidBody = mbs.AddNode(nodeItem)
                        
            self.rotationParameters0 = nodeItem.referenceCoordinates[3:]
            self.rotationParameters_t0 = nodeItem.initialVelocities[3:]

        #print("self.rotationParameters0 =",self.rotationParameters0 )
        #print("self.rotationParameters_t0 =",self.rotationParameters_t0 )

        self.gravity = gravity
        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #check if postProcessingModes exist
        outputVariableModeBasis = []
        outputVariableTypeModeBasis = 0
        #FEM: self.postProcessingModes = {}   # {'matrix':<matrix containing stress components (xx,yy,zz,yz,xz,xy) in each column, rows are for every mesh node>,'outputVariableType':exudyn.OutputVariableType.StressLocal}

        if 'matrix' in self.femInterface.postProcessingModes:
            #check FEMinterface if modes exist:
            if str(self.femInterface.postProcessingModes['outputVariableType']) == 'OutputVariableType.StressLocal':
                outputVariableTypeModeBasis = exu.OutputVariableType.StressLocal
            elif str(self.femInterface.postProcessingModes['outputVariableType']) == 'OutputVariableType.StrainLocal':
                outputVariableTypeModeBasis = exu.OutputVariableType.StrainLocal
            else:
                raise ValueError('AddObjectFFRFreducedOrderWithUserFunctions(...): invalid outputVariableType in postProcessingModes')

            outputVariableModeBasis  = self.femInterface.postProcessingModes['matrix']
            
        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #generic node for modal coordinates in ObjectFFRFreducedOrder
        self.nGenericODE2 = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=self.nModes,
                                          referenceCoordinates=[0]*self.nModes,
                                          initialCoordinates=[0]*self.nModes,
                                          initialCoordinates_t=[0]*self.nModes))

        stiffnessMatrixMC = exu.MatrixContainer()
        stiffnessMatrixMC.SetWithDenseMatrix(self.stiffnessMatrixReduced,useDenseMatrix=False)

        self.dampingMatrixReduced = massProportionalDamping*self.massMatrixReduced+stiffnessProportionalDamping*self.stiffnessMatrixReduced
        if (massProportionalDamping != 0 or stiffnessProportionalDamping != 0):
            dampingMatrixMC = exu.MatrixContainer()
            dampingMatrixMC.SetWithDenseMatrix(self.dampingMatrixReduced, useDenseMatrix=False)
        else:
            dampingMatrixMC=[]

        massMatrixMC = exu.MatrixContainer()
        
        #not needed, as not included in C++ computation until full CMS functionality
        #factMass = 1.
        #if UFmassMatrix != 0:
        #    factMass = 0.
        #massMatrixMC.SetWithDenseMatrix(factMass*self.massMatrixReduced,useDenseMatrix=False)
        
        massMatrixMC.SetWithDenseMatrix(self.massMatrixReduced,useDenseMatrix=False)
        emptyMC = exu.MatrixContainer()

        #add generic body for FFRF-Object:
        if UFmassMatrix == 0 or UFforce == 0:
            
            if np.array(gravity) @ np.array(gravity) != 0.:
                raise ValueError("AddObjectFFRFreducedOrderWithUserFunctions: C++ version only implemented for gravity=[0,0,0]; set both user functions or use LoadMassProportional")

            self.oFFRFreducedOrder = mbs.AddObject(ObjectFFRFreducedOrder(nodeNumbers = [self.nRigidBody, self.nGenericODE2], 
                                                                stiffnessMatrixReduced=stiffnessMatrixMC, 
                                                                massMatrixReduced=massMatrixMC,
                                                                dampingMatrixReduced=dampingMatrixMC,
                                                                modeBasis=self.modeBasis,
                                                                referencePositions = self.xRef,
                                                                physicsMass=self.totalMass,
                                                                physicsInertia=self.inertiaLocal,
                                                                physicsCenterOfMass=self.chiU,
                                                                mPsiTildePsi = self.mPsiTildePsi,
                                                                mPsiTildePsiTilde = self.mPsiTildePsiTilde,
                                                                mPhitTPsi = self.mPhitTPsi,
                                                                mPhitTPsiTilde = self.mPhitTPsiTilde,
                                                                mXRefTildePsi = self.mXRefTildePsi,
                                                                mXRefTildePsiTilde = self.mXRefTildePsiTilde,
                                                                outputVariableModeBasis = outputVariableModeBasis,
                                                                outputVariableTypeModeBasis = outputVariableTypeModeBasis,
                                                                #
                                                                forceUserFunction=UFforce,
                                                                massMatrixUserFunction=UFmassMatrix,
                                                                computeFFRFterms=True, #only compute user function, no internal components ...
                                                                visualization=VObjectFFRF(triangleMesh = self.trigList, 
                                                                                            color=color,
                                                                                            showNodes = True)))

        else:
            self.oFFRFreducedOrder = mbs.AddObject(ObjectFFRFreducedOrder(nodeNumbers = [self.nRigidBody, self.nGenericODE2], 
                                                                stiffnessMatrixReduced=stiffnessMatrixMC, 
                                                                massMatrixReduced=massMatrixMC,
                                                                dampingMatrixReduced=dampingMatrixMC,
                                                                modeBasis=self.modeBasis,
                                                                referencePositions = self.xRef,
                                                                forceUserFunction=UFforce,
                                                                massMatrixUserFunction=UFmassMatrix,
                                                                computeFFRFterms=False, #only compaute user function, no internal components ...
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
        #compute rotation parameters including reference values:
        rp = np.array(qReduced[self.dim3D:self.nODE2rigid]) + np.array(self.rotationParameters0) #add reference values, q are only the change w.r.t. reference values!
        if self.rigidBodyNodeType == exu.NodeType.RotationEulerParameters:
            G = EulerParameters2GLocal(rp)
        elif self.rigidBodyNodeType == exu.NodeType.RotationRotationVector:
            G = RotationVector2GLocal(rp)
        elif self.rigidBodyNodeType ==exu.NodeType.RotationRxyz:
            G = RotXYZ2GLocal(rp)
        else:
            raise ValueError('UFmassFFRFreducedOrder: rotation parameterization not implemented')
        

        zetaI = VectorDiadicUnitMatrix3D(qReduced[self.nODE2rigid:])
        #zeta_tI = VectorDiadicUnitMatrix3D(qReduced_t[self.nODE2rigid:]) #not needed

        #print("Mtt=",self.massMatrixFFRFreduced[0:3,0:3])
        #print("Mff=",self.massMatrixFFRFreduced[self.nODE2rigid:,self.nODE2rigid:])
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

        #print("Mtr=",Mtr)
        #print("Mtf=",Mtf)
        #print("Mrf=",Mrf)
        #print("Mrr=",self.massMatrixFFRFreduced[self.dim3D:self.nODE2rigid, self.dim3D:self.nODE2rigid])

        return self.massMatrixFFRFreduced

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: CMS force matrix user function; qReduced and qReduced\_t contain the coordiantes of the rigid body node and the modal coordinates in one vector!
    def UFforceFFRFreducedOrder(self, exu, mbs, t, qReduced, qReduced_t):
        force = np.zeros(self.nODE2FFRFreduced)

        Avec = mbs.GetNodeOutput(self.nRigidBody,  exu.OutputVariableType.RotationMatrix)
        A = Avec.reshape((3,3))

        #compute rotation parameters including reference values:
        rp = np.array(qReduced[self.dim3D:self.nODE2rigid]) + np.array(self.rotationParameters0) #add reference values, q are only the change w.r.t. reference values!
        # if len(ep) != 4: 
        #     print("ERROR: equations only implemented for Euler parameters case (terms missing for other formulations)"); exit()

        if self.rigidBodyNodeType == exu.NodeType.RotationEulerParameters:
            G = EulerParameters2GLocal(rp)
        elif self.rigidBodyNodeType == exu.NodeType.RotationRotationVector:
            G = RotationVector2GLocal(rp)
        elif self.rigidBodyNodeType ==exu.NodeType.RotationRxyz:
            G = RotXYZ2GLocal(rp)
            #time derivatives:
            rp_t = np.array(qReduced_t[self.dim3D:self.nODE2rigid])
            G_t = RotXYZ2GLocal_t(rp, rp_t)
            G_tRp_t = G_t @ rp_t
        else:
            raise ValueError('UFforceFFRFreducedOrder: rotation parameterization not implemented')

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
                                   2 * self.mPsiTildePsi.T @ zeta_tI @ omega3D #identical to FFRF up to 1e-16
                                   - self.stiffnessMatrixReduced @ qReducedFF  #stiffness term added to user function, for better distinguishing with internal FFRF
                                   - self.dampingMatrixReduced @ qReduced_tFF) #damping term added to user function, for better distinguishing with internal FFRF

        #print("fQV=", force)
        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #additional terms for Euler angles:
        if self.rigidBodyNodeType == exu.NodeType.RotationRxyz:
            force[0:self.dim3D] += A @ (self.totalMass*self.chiUtilde + self.mPhitTPsiTilde @ zetaI) @ G_tRp_t
            
            force[self.dim3D:self.nODE2rigid] += -G.T @ (self.inertiaLocal + self.mXRefTildePsiTilde @ zetaI + 
                                                         zetaI.T @ self.mXRefTildePsiTilde.T + 
                                                         zetaI.T @ self.mPsiTildePsiTilde @ zetaI ) @ G_tRp_t
            
            force[self.nODE2rigid:] += (self.mXRefTildePsi.T + self.mPsiTildePsi.T @ zetaI) @ G_tRp_t

        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #add gravity (needs to be tested):
        if True:
            force[0:self.dim3D] += self.totalMass*np.array(self.gravity)
            force[self.dim3D:self.nODE2rigid] += G.T @ (Skew(self.chiU) @ (self.totalMass*A.T @ np.array(self.gravity)))
            force[self.nODE2rigid:] += self.mPhitTPsi.T @ (A.T @ np.array(self.gravity))

            #force[nODE2rigid:] += modeBasis.T @ PhitTM.T @ (A.T @ g)

        return force

    #**classFunction: add according nodes, objects and constraints for ObjectFFRFreducedOrder object to MainSystem mbs; use this function in order to use internal C++ functionality, which is approx. 10x faster than AddObjectFFRFreducedOrderWithUserFunctions(...)
    #**input:
    #  exu: the exudyn module
    #  mbs: a MainSystem object
    #  positionRef: reference position of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
    #  initialVelocity: initial velocity of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
    #  rotationMatrixRef: reference rotation of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder); if [], it becomes the unit matrix
    #  initialAngularVelocity: initial angular velocity of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
    #  massProportionalDamping: Rayleigh damping factor for mass proportional damping, added to floating frame/modal coordinates only
    #  stiffnessProportionalDamping: Rayleigh damping factor for stiffness proportional damping, added to floating frame/modal coordinates only
    #  color: provided as list of 4 RGBA values
    def AddObjectFFRFreducedOrder(self, mbs, 
                                  positionRef=[0,0,0], initialVelocity=[0,0,0], 
                                  rotationMatrixRef=[], initialAngularVelocity=[0,0,0],
                                  massProportionalDamping = 0, stiffnessProportionalDamping = 0,
                                  color=[0.1,0.9,0.1,1.]):
        import exudyn as exu
        return self.AddObjectFFRFreducedOrderWithUserFunctions(exu=exu, mbs=mbs, 
                                                  positionRef=positionRef, initialVelocity=initialVelocity, rotationMatrixRef=rotationMatrixRef, 
                                                  initialAngularVelocity=initialAngularVelocity,
                                                  massProportionalDamping = massProportionalDamping, stiffnessProportionalDamping = stiffnessProportionalDamping,
                                                  color=color)
    
        

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**class: helper calss for function ComputeHurtyCraigBamptonModes, declaring some computation options. It offers the following options:\\
# - allBoundaryNodes:     compute a single static mode for every boundary coordinate\\
# - RBE2:                 static modes only for rigid body motion at boundary nodes\\
# - RBE3:                 [yet NOT AVAILABLE] averaged rigid body interfaces; for future implementations\\
# - noStaticModes:        do not compute static modes, only eigen modes (not recommended; usually only for tests)
class HCBstaticModeSelection(Enum):
    allBoundaryNodes = 1    #compute a single static mode for every boundary coordinate; if this is used, 6 constraints need to be added to the ObjectFFRFreducedOrder, otherwise there is additional rigid body motion!
    RBE2 = 2                #(recommended) static modes computation only for rigid body motion at boundary nodes
    noStaticModes = 3       #do not compute static modes, only eigen modes (not really recommended; usually only for tests)
    


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++   FEMinterface - finite element interface class   ++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**class: general interface to different FEM / mesh imports and export to EXUDYN functions
#         use this class to import meshes from different meshing or FEM programs (NETGEN/NGsolve, ABAQUS, ANSYS, ..) and store it in a unique format
#         do mesh operations, compute eigenmodes and reduced basis, etc.
#         load/store the data efficiently with LoadFromFile(...), SaveToFile(...)  if import functions are slow
#         export to EXUDYN objects
class FEMinterface:
    #**classFunction: initalize all data of the FEMinterface by, e.g., \texttt{fem = FEMinterface()}
    #**example:
    # #**** this is not an example, just a description for internal variables ****
    # #default values for member variables stored internally in FEMinterface fem and typical structure:
    # fem.nodes = {}                 # {'Position':[[x0,y0,z0],...], 'RigidBodyRxyz':[[x0,y0,z0],...],  },...]                     #dictionary of different node lists
    # fem.elements = []              # [{'Name':'identifier', 'Tet4':[[n0,n1,n2,n3],...], 'Hex8':[[n0,...,n7],...],  },...]        #there may be several element sets
    # fem.massMatrix = np.zeros((0,0))    # np.array([[r0,c0,value0],[r1,c1,value1], ... ])                                #currently only in SparseCSR format allowed!
    # fem.stiffnessMatrix=np.zeros((0,0)) # np.array([[r0,c0,value0],[r1,c1,value1], ... ])                                #currently only in SparseCSR format allowed!
    # fem.surface = []               # [{'Name':'identifier', 'Trigs':[[n0,n1,n2],...], 'Quads':[[n0,...,n3],...],  },...]           #surface with faces
    # fem.nodeSets = []              # [{'Name':'identifier', 'NodeNumbers':[n_0,...,n_ns], 'NodeWeights':[w_0,...,w_ns]},...]     #for boundary conditions, etc.
    # fem.elementSets = []           # [{'Name':'identifier', 'ElementNumbers':[n_0,...,n_ns]},...]                                #for different volumes, etc.
    # fem.modeBasis = {}             # {'matrix':[[Psi_00,Psi_01, ..., Psi_0m],...,[Psi_n0,Psi_n1, ..., Psi_nm]],'type':'NormalModes'} #'NormalModes' are eigenmodes, 'HCBmodes' are Craig-Bampton modes including static modes
    # fem.eigenValues = []           # [ev0, ev1, ...]                                                                             #eigenvalues according to eigenvectors in mode basis
    # fem.postProcessingModes = {}   # {'matrix':<matrix containing stress components (xx,yy,zz,yz,xz,xy) in each column, rows are for every mesh node>,'outputVariableType':exudyn.OutputVariableType.StressLocal}
    def __init__(self):
        self.nodes = {}                 # {'Position':[[x0,y0,z0],...], 'RigidBodyRxyz':[[x0,y0,z0],...],  },...]                     #dictionary of different node lists
        self.elements = []              # [{'Name':'identifier', 'Tet4':[[n0,n1,n2,n3],...], 'Hex8':[[n0,...,n7],...],  },...]        #there may be several element sets
        self.massMatrix = np.zeros((0,0))    # np.array([[r0,c0,value0],[r1,c1,value1], ... ])                                #currently only in SparseCSR format allowed!
        self.stiffnessMatrix=np.zeros((0,0)) # np.array([[r0,c0,value0],[r1,c1,value1], ... ])                                #currently only in SparseCSR format allowed!
        self.surface = []               # [{'Name':'identifier', 'Trigs':[[n0,n1,n2],...], 'Quads':[[n0,...,n3],...],  },...]           #surface with faces
        self.nodeSets = []              # [{'Name':'identifier', 'NodeNumbers':[n_0,...,n_ns], 'NodeWeights':[w_0,...,w_ns]},...]     #for boundary conditions, etc.
        self.elementSets = []           # [{'Name':'identifier', 'ElementNumbers':[n_0,...,n_ns]},...]                                #for different volumes, etc.

        self.modeBasis = {}             # {'matrix':[[Psi_00,Psi_01, ..., Psi_0m],...,[Psi_n0,Psi_n1, ..., Psi_nm]],'type':'NormalModes'}
        self.eigenValues = []           # [ev0, ev1, ...]                                                                             #eigenvalues according to eigenvectors in mode basis
        self.postProcessingModes = {}   # {'matrix':<matrix containing stress components (xx,yy,zz,yz,xz,xy) in each column, rows are for every mesh node>,'outputVariableType':exudyn.OutputVariableType.StressLocal}
        #self.massMatrix = {}           # {'Shape':[rows,columns], 'SparseCSR':[[r0,c0,value0],[r1,c1,value1], ... ],  }             #currently only in SparseCSR format allowed!

        #some additional information, needed for checks and easier operation
        self.coordinatesPerNodeType = {'Position':3, 'Position2D':2, 'RigidBodyRxyz':6, 'RigidBodyEP':7} #number of coordinates for a certain node type


    #**classFunction: save all data (nodes, elements, ...) to a data filename; this function is much faster than the text-based import functions
    #**input: use filename without ending ==> ".npy" will be added
    def SaveToFile(self, fileName):
        fileExtension = ''
        if len(fileName) < 4 or fileName[-4:]!='.npy':
            fileExtension = '.npy'
        with open(fileName+fileExtension, 'wb') as f:
            np.save(f, self.nodes, allow_pickle=True)
            np.save(f, self.elements, allow_pickle=True)
            np.save(f, self.massMatrix)
            np.save(f, self.stiffnessMatrix)
            np.save(f, self.surface, allow_pickle=True)
            np.save(f, self.nodeSets, allow_pickle=True)
            np.save(f, self.elementSets, allow_pickle=True)
            np.save(f, self.modeBasis, allow_pickle=True)
            np.save(f, self.eigenValues, allow_pickle=True)
            np.save(f, self.postProcessingModes, allow_pickle=True)

    #**classFunction: load all data (nodes, elements, ...) from a data filename previously stored with SaveToFile(...). 
    #this function is much faster than the text-based import functions
    #**input: use filename without ending ==> ".npy" will be added
    def LoadFromFile(self, fileName):
        fileExtension = ''
        if len(fileName) < 4 or fileName[-4:]!='.npy':
            fileExtension = '.npy'
        with open(fileName+fileExtension, 'rb') as f:
            self.nodes = np.load(f, allow_pickle=True).all()
            self.elements = list(np.load(f, allow_pickle=True))
            self.massMatrix = np.load(f)
            self.stiffnessMatrix = np.load(f)
            self.surface = list(np.load(f, allow_pickle=True))
            self.nodeSets =  list(np.load(f, allow_pickle=True))
            self.elementSets = list(np.load(f, allow_pickle=True))
            self.modeBasis = np.load(f, allow_pickle=True).all()
            self.eigenValues = list(np.load(f, allow_pickle=True))
            self.postProcessingModes = np.load(f, allow_pickle=True).all()

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #ABAQUS import functions

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



    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: import mesh from NETGEN/NGsolve and setup mechanical problem
    #**notes: The interface to NETGEN/NGsolve has been created together with Joachim Sch\"oberl, main developer 
    #  of NETGEN/NGsolve; Thank's a lot!
    #  download NGsolve at: https://ngsolve.org/
    #  NGsolve needs Python 3.7 (64bit) ==> use according EXUDYN version!
    #  note that node/element indices in the NGsolve mesh are 1-based and need to be converted to 0-base!
    #**input:
    #    mesh: a previously created \texttt{ngs.mesh} (NGsolve mesh, see examples)
    #    youngsModulus: Young's modulus used for mechanical model
    #    poissonsRatio: Poisson's ratio used for mechanical model
    #    density: density used for mechanical model
    #    meshOrder: use 1 for linear elements and 2 for second order elements (recommended to use 2 for much higher accuracy!)
    #    verbose: set True to print out some status information
    #**notes: setting ngsolve.SetNumThreads(nt) you can select the number of treads that are used for assemble or other functionality with NGsolve functionality 
    #**output: creates according nodes, elements, in FEM and returns [bfM, bfK, fes] which are the (mass matrix M, stiffness matrix K) bilinear forms and the finite element space fes
    #**author: Johannes Gerstmayr, Joachim Sch\"oberl
    def ImportMeshFromNGsolve(self, mesh, density, youngsModulus, poissonsRatio, verbose = False, 
                              computeEigenmodes = False, meshOrder = 1, **kwargs):
        #OLD, DELETE 2022-01-01:
        #    computeEigenmodes: set True to use NGsolve for eigenmode computation instead of ComputeEigenmodes
        #    numberOfModes: if computeEigenmodes==True: number of eigen modes computed with NGsolve; default=10
        #    maxEigensolveIterations: if computeEigenmodes==True: maximum number of iterations for iterative eigensolver; default=40
        #    excludeRigidBodyModes: if computeEigenmodes==True: if rigid body modes are expected (in case of free-free modes), then this number specifies the number of eigenmodes to be excluded in the stored basis (usually 6 modes in 3D)
        import ngsolve as ngs
        if meshOrder < 1 or meshOrder > 2:
            raise ValueError('mesh order > 1 or mesh order < 2 not supported!')
            
        if meshOrder == 2:
            mesh.ngmesh.SecondOrder()

        if verbose: print("NGsolve create mechanics FE space ...")
        if meshOrder == 1:
            fes = ngs.VectorH1(mesh, order=meshOrder) #add interleaved = True to get xyzxyz sorting
        else:
            fes = ngs.NodalFESpace(mesh, order=meshOrder)**3
            
        #create finite element spaces for mass matrix and stiffness matrix    
        u = fes.TrialFunction()
        v = fes.TestFunction()
        bfK = ngs.BilinearForm(fes)
        bfM = ngs.BilinearForm(fes)

        def sigma(eps, mu, lam):
            return 2*mu*eps + lam*ngs.Trace(eps) * ngs.Id(eps.dims[0])

        E = youngsModulus
        nu = poissonsRatio
        rho = density

        mu  = E / 2 / (1+nu) #Lame parameters
        lam = E * nu / ((1+nu)*(1-2*nu))

        #setup (linear) mechanical FE-space
        bfK += ngs.InnerProduct(sigma(ngs.Sym(ngs.Grad(u)),mu,lam), ngs.Sym(ngs.Grad(v)))*ngs.dx
        bfM += rho*u*v * ngs.dx

        with ngs.TaskManager():
            if verbose: print ("NGsolve assemble M and K")
            bfK.Assemble()
            bfM.Assemble()

        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #export to scipy sparse matrices:
        from scipy.sparse import csr_matrix
        if verbose: print ("NGsolve convert system matrices to scipy csr_matrix format")
        K = csr_matrix( bfK.mat.CSR(), copy=True )
        M = csr_matrix( bfM.mat.CSR(), copy=True )
        if verbose: print("K.shape=",K.shape)

        #convert csr_matrix in NGsolve to exudyn sparse CSR np.array:
        M1 = ScipySparseCSRtoCSR(M)
        K1 = ScipySparseCSRtoCSR(K)

        nMK = M.shape[0] #get size of mass matrix; assume square matrix!
        #NGsolve sorts indices as x0x1x2...y0y1y2...z0z1z2...., but is needed as x0y0z0x1y1z1...
        ResortIndicesOfCSRmatrix(M1, nMK)
        ResortIndicesOfCSRmatrix(K1, nMK)


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
            NP = len(mesh.ngmesh.Points())
            if verbose: print("number of points=", NP)
    
            for n in mesh.ngmesh.Points(): 
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

        #function to flip surface elements:
        def Flip3D(v):
            return [v[0], v[2], v[1]]
        
        surface = mesh.ngmesh.Elements2D() #surface mesh
        if meshOrder == 1:
            for st in surface: 
                vertices = []
                for v in st.vertices: #st.points gives all nodes (for order>1), vertices only vertex points (always 4 per tet)
                    vertices += [v.nr-1] #convert to 0-based indices
                if len(vertices) != 3:
                    raise ValueError('ImportMeshFromNGsolve: expected linear 3-node surface elements')

                surfaceTriangleList += [Flip3D(vertices)]
        else: #order 2
            for st in surface: 
                w = []
                for v in st.points: #st.points gives all nodes (for order>1), vertices only vertex points (always 4 per tet)
                    w += [v.nr-1] #convert to 0-based indices
                if len(w) != 6:
                    raise ValueError('ImportMeshFromNGsolve: expected second order 6-node surface elements')

                surfaceTriangleList += [Flip3D([w[0],w[5],w[4]])]
                surfaceTriangleList += [Flip3D([w[5],w[1],w[3]])]
                surfaceTriangleList += [Flip3D([w[5],w[3],w[4]])]
                surfaceTriangleList += [Flip3D([w[4],w[3],w[2]])]

        nodes = np.array(nodeList)
        elements=np.array(tetList) #unused
        trigList = surfaceTriangleList

        self.nodes = {'Position':nodes}
        self.elements = [{'Name':'NGsolve','Tet4':elements}]
        self.massMatrix = M1 
        self.stiffnessMatrix = K1 
        self.surface = [{'Name':'meshSurface','Trigs':trigList}]

        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
        if computeEigenmodes:
            print('**********\nWARNING!**********\nNGsolve eigenmode computation deprecated: USE FEM.ComputeEigenmodesNGsolve(...)')
            if verbose: print ("NGsolve: compute eigenmodes")
            excludeRigidBodyModes = 0
            if 'excludeRigidBodyModes' in kwargs:
                excludeRigidBodyModes = kwargs['excludeRigidBodyModes']

            nModes = 10
            if 'numberOfModes' in kwargs: 
                nModes = kwargs['numberOfModes'] 
            maxIt = 40
            if 'maxEigensolveIterations' in kwargs: 
                maxIt = kwargs['maxEigensolveIterations']

            from ngsolve.eigenvalues import PINVIT

            with ngs.TaskManager():
                KM = bfK.mat.CreateMatrix()
                KM.AsVector().data = bfK.mat.AsVector() + 1e6* bfM.mat.AsVector()
            
                inv = KM.Inverse(inverse='sparsecholesky')
                res = PINVIT(bfK.mat, bfM.mat, inv, num=nModes+excludeRigidBodyModes, maxit=maxIt, \
                                printrates=verbose, GramSchmidt=True)

            #self.res = res
            nDOF = K.shape[0]
            eigVecs = np.zeros((nDOF, nModes))
            for i in range(nModes):
                #eigVecs[:,i] = np.array(res[1][excludeRigidBodyModes+i])
                eigVecs[:,i] = ResortIndicesOfNGvector(np.array(res[1][excludeRigidBodyModes+i]))
            
            self.modeBasis = {'matrix':eigVecs, 'type':'NormalModes'}
            self.eigenValues = np.abs(res[0][excludeRigidBodyModes:excludeRigidBodyModes + nModes])
                             
            if verbose: print ("eigenfrequencies (Hz) =",(0.5/np.pi)*np.sqrt(np.abs(res[0][excludeRigidBodyModes:excludeRigidBodyModes + nModes])))

        return [bfM, bfK, fes]


    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: compute nModes smallest eigenvalues and eigenmodes from mass and stiffnessMatrix; store mode vectors in modeBasis, but exclude a number of 'excludeRigidBodyModes' rigid body modes from modeBasis; uses scipy for solution of generalized eigenvalue problem
    #**input: 
    #  nModes: prescribe the number of modes to be computed; total computed modes are  (nModes+excludeRigidBodyModes), but only nModes with smallest absolute eigenvalues are considered and stored
    #  excludeRigidBodyModes: if rigid body modes are expected (in case of free-free modes), then this number specifies the number of eigenmodes to be excluded in the stored basis (usually 6 modes in 3D)
    #  maxEigensolveIterations: maximum number of iterations for iterative eigensolver; default=40
    #  verbose: if True, output some relevant information during solving
    #**output: eigenmodes are stored internally in FEMinterface as 'modeBasis' and eigenvalues as 'eigenValues'
    #**author: Johannes Gerstmayr, Joachim Sch\"oberl
    def ComputeEigenmodesNGsolve(self, bfM, bfK,
                                 nModes, 
                                 maxEigensolveIterations = 40,
                                 excludeRigidBodyModes = 0,
                                 verbose = False):
        import ngsolve as ngs

        maxIt = maxEigensolveIterations

        from ngsolve.eigenvalues import PINVIT

        with ngs.TaskManager():
            KM = bfK.mat.CreateMatrix()
            KM.AsVector().data = bfK.mat.AsVector() + 1e6* bfM.mat.AsVector()
        
            inv = KM.Inverse(inverse='sparsecholesky')
            res = PINVIT(bfK.mat, bfM.mat, inv, num=nModes+excludeRigidBodyModes, maxit=maxIt, \
                            printrates=verbose, GramSchmidt=True)

        #nDOF = K.shape[0]
        nDOF = bfK.space.ndof
        eigVecs = np.zeros((nDOF, nModes))
        for i in range(nModes):
            #eigVecs[:,i] = np.array(res[1][excludeRigidBodyModes+i])
            eigVecs[:,i] = ResortIndicesOfNGvector(np.array(res[1][excludeRigidBodyModes+i]))
        
        self.modeBasis = {'matrix':eigVecs, 'type':'NormalModes'}
        self.eigenValues = np.abs(res[0][excludeRigidBodyModes:excludeRigidBodyModes + nModes])
                         
        if verbose: print ("eigenfrequencies (Hz) =",(0.5/np.pi)*np.sqrt(np.abs(res[0][excludeRigidBodyModes:excludeRigidBodyModes + nModes])))

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: compute static  and eigen modes based on Hurty-Craig-Bampton, for details see theory part \refSection{sec:theory:CMS}. This function uses internal computational functionality of NGsolve and is often much faster than the scipy variant
    #**input:
    #  bfM: bilinearform for mass matrix as retured in ImportMeshFromNGsolve(...)
    #  bfK: bilinearform for stiffness matrix as retured in ImportMeshFromNGsolve(...)
    #  boundaryNodesList: [nodeList0, nodeList1, ...] a list of node lists, each of them representing a set of 'Position' nodes for which a rigid body interface (displacement/rotation and force/torque) is created; NOTE THAT boundary nodes may not overlap between the different node lists (no duplicated node indices!)
    #  nEigenModes: number of eigen modes in addition to static modes (may be zero for RBE2 computationMode); eigen modes are computed for the case where all rigid body motions at boundaries are fixed; only smallest nEigenModes absolute eigenvalues are considered
    #  maxEigensolveIterations: maximum number of iterations for iterative eigensolver; default=40
    #  verbose: if True, output some relevant information during solving
    #**output: stores computed modes in self.modeBasis and abs(eigenvalues) in self.eigenValues
    #**author: Johannes Gerstmayr, Joachim Sch\"oberl
    def ComputeHurtyCraigBamptonModesNGsolve(self,
                                      bfM, bfK,
                                      boundaryNodesList,
                                      nEigenModes, 
                                      maxEigensolveIterations = 40,
                                      verbose = False):

        import ngsolve as ngs
        import time
        start_time = time.time()
        #from scipy.linalg import solve, eigh, eig #eigh for symmetric matrices, positive definite
        
        nNodeLists = len(boundaryNodesList)
        #sizes of internal and boundary nodes:
        M = bfM.mat
        K = bfK.mat
        n = K.height
        nNodes = int(n/3)
        nodesPos = self.GetNodePositionsAsArray()

        bndDOFsAll = ngs.BitArray(n) #fill in all boundaries later
        bndDOFsAll[:] = False        
        
        #+++++++++++++++++++++++++++
        #compute static rigid body modes:
        addRotationModes = 1
        rbSize = 3 + 3*addRotationModes #size of rigid body coordinates (3 for translation, 6 for translation+rotation)
        nbRBE2 = (nNodeLists-1)*rbSize #number of chosen static modes, 6 DOF per rigid body interface; exclude first rigid body boundary in order to suppress rigid body motion of static modes
        DOFeig = np.arange(nbRBE2,nbRBE2+nEigenModes) #for final mapping of eigenmode coordinates

        modeBasis = np.zeros((n, nbRBE2+nEigenModes))

        #all rigid body modes
        mvAll = ngs.MultiVector(K.CreateColVector(),0)
        
        #1 rigid body modes
        mvRB = ngs.MultiVector(K.CreateColVector(),6)
        for v in mvRB: v[:]=0 #initialize!

        #rigid body displacements
        for i in range(3):
            mvRB[i][nNodes*i:nNodes*(i+1)] = 1
                        
        #create list of mappings between average rigid body motion and boundary DOF matrix
        rigidBodyMappings = [[]]*nNodeLists #list of mappings
        cntBoundary = 0 #counter for boundaryNodeLists / number of interfaces
        for cntBoundary, boundaryNodes in enumerate(boundaryNodesList):
            nbn = len(boundaryNodes)
            
            bndDOFs = ngs.BitArray(n)
            bndDOFs[:] = False
    
            DOFb = np.zeros(nbn*3, dtype=np.int)
            for i in range(len(boundaryNodes)):
                node = boundaryNodes[i]
                #DOFb[i*3:i*3+3] = [node*3,node*3+1,node*3+2] #interleaved xyzxyz
                DOFb[i*3:i*3+3] = [node,node+nNodes,node+2*nNodes] #xxxx yyyy zzzz
                
            for i, d in enumerate(DOFb):
                bndDOFs[d] = True

            bndDOFsAll |= bndDOFs
            
            #compute midpoints of boundary nodes:
            p0 = np.zeros(3)
            for node in boundaryNodes:
                p0 += nodesPos[node]
            
            p0 = p0*(1./len(boundaryNodes))

            #compute rigid body modes for this boundary:
            for i in range(3): #iterate about 3 rotation axes
                rot = np.zeros(3) #rotation vector, unit rotation
                rot[i] = 1
                rotTilde = Skew(rot)
                for bj in boundaryNodes:
                    p = nodesPos[bj]-p0
                    qRot = rotTilde@p

                    for k in range(3):
                        mvRB[i+3][bj+k*nNodes] = qRot[k]
            
            proj = ngs.Projector(bndDOFs, True)
            for v in mvRB:
                hv = v.CreateVector()
                hv.data = proj * v
                mvAll.Append(hv)
        
        if verbose: print('solve...')
        KiiInv = K.Inverse(~bndDOFsAll,inverse='sparsecholesky')
        if verbose: print('...ready')

        #compute static modes for all rigid body boundaries
        mvAll.data = mvAll - KiiInv @ K * mvAll
            
        for i in range(len(mvAll)-1*rbSize):
            modeBasis[:,i] = ResortIndicesOfNGvector(mvAll[i+1*rbSize])
   
        #++++++++++++++++++++++++++++++++++++++
        #compute modes for free inner nodes:    
        from ngsolve.eigenvalues import PINVIT

        if nEigenModes != 0:
            if verbose: print('compute eigenvectors of inner nodes...')
            maxIt = maxEigensolveIterations
            with ngs.TaskManager():#pajetrace=10**8):
                #with shift strategy, but not necessary for inner nodes (no rigid-body-modes)
                # KM = bfK.mat.CreateMatrix()
                # KM.AsVector().data = bfK.mat.AsVector() + 1e6* bfM.mat.AsVector()
                # KMinv = KM.Inverse(~bndDOFsAll,inverse='sparsecholesky')
                KMinv = KiiInv
                res = PINVIT(bfK.mat, bfM.mat, KMinv, 
                             num=nEigenModes, maxit=maxIt, \
                             printrates=verbose, GramSchmidt=True)
    
            if verbose: print('...ready')
    
            for i in range(nEigenModes):
                modeBasis[:,i+nbRBE2] = ResortIndicesOfNGvector(np.array(res[1][i]))

            self.eigenValues = np.abs(res[0][0:nEigenModes])
        else:
            self.eigenValues = np.array([])

        self.modeBasis = {'matrix':modeBasis, 'type':'HCBmodes'}
                         
        if verbose: 
            print ("eigenfrequencies (Hz) =",(0.5/np.pi)*np.sqrt(np.abs(self.eigenValues)))
            print("HCB NGsolve modes needed %.3f seconds" % (time.time() - start_time))



    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: compute special stress or strain modes in order to enable visualization of stresses and strains in ObjectFFRFreducedOrder; takes a NGsolve fes as input and uses internal NGsolve methods to efficiently compute stresses or strains
    #**input: 
    #  fes: finite element space as retured in ImportMeshFromNGsolve(...)
    #  material: specify material properties for computation of stresses, using a material class, e.g. material = KirchhoffMaterial(Emodulus, nu, rho); not needed for strains (material = 0)
    #  outputVariableType: specify either exudyn.OutputVariableType.StressLocal or exudyn.OutputVariableType.StrainLocal as the desired output variables
    #**notes: This function is implemented in Python and rather slow for larger meshes; for NGsolve / Netgen meshes, see the according ComputePostProcessingModesNGsolve function, which is usually much faster
    #**output: post processing modes are stored in FEMinterface in local variable postProcessingModes as a dictionary, where 'matrix' represents the modes and 'outputVariableType' stores the type of mode as a OutputVariableType
    #**author: Johannes Gerstmayr, Joachim Sch\"oberl
    def ComputePostProcessingModesNGsolve(self, fes, material = 0, 
                                          outputVariableType = 'OutputVariableType.StressLocal', 
                                          verbose = False):

        import ngsolve as ngs
        #++++++++++++++++++++++++++++++++++++
        #special ngsolve functions which are put into expression trees:
        def Strain2Stress(eps, mu, lam):
            return 2*mu*eps + lam*ngs.Trace(eps) * ngs.Id(eps.dims[0])
    
        def Strain2Strain(eps, mu, lam):
            return eps
        #++++++++++++++++++++++++++++++++++++
        #print("t=",outputVariableType)
        if str(outputVariableType) == 'OutputVariableType.StressLocal':
            computeStrains = False
            StressFunction = Strain2Stress
        elif str(outputVariableType) == 'OutputVariableType.StrainLocal':
            computeStrains = True
            StressFunction = Strain2Strain
        else:
            raise ValueError('ComputePostProcessingModes invoked with invalid outputVariableType')

        # if material == 0:
        #     material=KirchhoffMaterial(1, 0, 1)
        #     if not computeStrains:
        #         raise ValueError('ComputePostProcessingModes: if material=0, outputVariableType must be StrainLocal')

        # nNodes = int(fes.ndof/3)
        # modeBasis = self.modeBasis['matrix']
        # [mu, lam] = material.LameParameters()
                
        # nModeVectors = modeBasis.shape[1]
        # stressModesMatrix = np.zeros((nNodes,6*nModeVectors))
    
        # fesStress = ngs.MatrixValued(ngs.H1(fes.mesh, order=fes.globalorder), symmetric=True)
        # #order of stresses (per node) = xx,xy,xz,yy,yz,zz (in order xx xx xx xy xy xy...)
        # gfStress = ngs.GridFunction(fesStress)
        # gfu = ngs.GridFunction(fes)
    
        # #map ngsolve stress components xx,xy,yy,xz,yz,zz to Exudyn xx,yy,zz,yz,xz,xy
        # ngsStressMap = [0,2,5,4,3,1] #stressModeExu[i] = stressModeNGS[ngsStressMap[i]]
    
        # with ngs.TaskManager():
        #     for i in range(nModeVectors):
        #         if verbose: print('compute stress mode ', i, 'of', nModeVectors)
        #         v = modeBasis[:,i]
        #         gfu.vec.FV()[:] = ResortIndicesExudyn2NGvector(v)
        #         # print(StressFunction(ngs.Sym(ngs.Grad(gfu)), mu, lam))
        #         # t1 = time.time()
        #         # gfStress.Interpolate(StressFunction(ngs.Sym(ngs.Grad(gfu)), mu, lam))
        #         gfStress.Interpolate(StressFunction(ngs.Sym(ngs.Grad(gfu)), mu, lam).Compile())
        #         #gfStress.Set(StressFunction(ngs.Sym(ngs.Grad(gfu)), mu, lam))
        #         # print(time.time() - t1)
                
        #         sv = gfStress.vec.FV()
        #         for j in range(6):
        #             stressModesMatrix[:,i*6+j] = sv[ngsStressMap[j]*nNodes:(ngsStressMap[j]+1)*nNodes]
        
        # self.postProcessingModes = {'matrix': stressModesMatrix, 
        #                            'outputVariableType': outputVariableType}
        if material == 0:
            material=KirchhoffMaterial(1, 0, 1)
            if not computeStrains:
                raise ValueError('ComputePostProcessingModes: if material=0, outputVariableType must be StrainLocal')

        nNodes = int(fes.ndof/3)
        modeBasis = self.modeBasis['matrix']
        [mu, lam] = material.LameParameters()
                
        nModeVectors = modeBasis.shape[1]
        stressModesMatrix = np.zeros((nNodes,6*nModeVectors))
    
        meshOrder = fes.components[0].globalorder
        if verbose: print('ORDER of fes=',meshOrder)
        # meshOrder = 2
        if meshOrder == 1:
            fesStress = ngs.MatrixValued(ngs.H1(fes.mesh, order=meshOrder), symmetric=True)
        else:
            fesStress = ngs.MatrixValued(ngs.NodalFESpace(fes.mesh, order=meshOrder), symmetric=True)

        #order of stresses (per node) = xx,xy,xz,yy,yz,zz (in order xx xx xx xy xy xy...)
        gfStress = ngs.GridFunction(fesStress)
        gfu = ngs.GridFunction(fes)
    
        #map ngsolve stress components xx,xy,yy,xz,yz,zz to Exudyn xx,yy,zz,yz,xz,xy
        ngsStressMap = [0,2,5,4,3,1] #stressModeExu[i] = stressModeNGS[ngsStressMap[i]]
    
        with ngs.TaskManager():
            for i in range(nModeVectors):
                if verbose: print('compute stress mode ', i, 'of', nModeVectors)
                v = modeBasis[:,i]
                gfu.vec.FV()[:] = ResortIndicesExudyn2NGvector(v)
                #gfStress.Interpolate(StressFunction(ngs.Sym(ngs.Grad(gfu)), mu, lam).Compile())
                gfStress.Set(StressFunction(ngs.Sym(ngs.Grad(gfu)), mu, lam))
                
                sv = gfStress.vec.FV()
                for j in range(6):
                    stressModesMatrix[:,i*6+j] = sv[ngsStressMap[j]*nNodes:(ngsStressMap[j]+1)*nNodes]
        
        self.postProcessingModes = {'matrix': stressModesMatrix, 
                                    'outputVariableType': outputVariableType}
        




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
    #**notes: in order to obtain a list of certain node positions, see example
    #**example:
    #p=GetNodePositionsAsArray(self)[42] #get node 42 position
    #nodeList=[1,13,42]
    #pArray=GetNodePositionsAsArray(self)[nodeList] #get np.array with positions of node indices
    def GetNodePositionsAsArray(self):
        if len(self.nodes) != 1:
            raise ValueError("ERROR: GetNodePositionsAsArray() only possible for one type of Position nodes!")

        nodeTypeName = list(self.nodes)[0]
        return self.nodes[nodeTypeName]

    #**classFunction: get mean (average) position of nodes defined by list of node numbers
    def GetNodePositionsMean(self,nodeNumberList):
        if len(self.nodes) != 1:
            raise ValueError("ERROR: GetNodesMeanPositions(...) only possible for one type of Position nodes!")

        return np.mean(self.GetNodePositionsAsArray()[nodeNumberList], axis=0)

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
        if len(self.nodes) != 1 or 'Position' not in self.nodes:
            raise ValueError('FEMinterface.GetNodeAtPoint: only Position type nodes allowed')
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
        if len(self.nodes) != 1 or 'Position' not in self.nodes:
            raise ValueError('FEMinterface.GetNodesInPlane: only Position type nodes allowed')
        for nodeTypeName in self.nodes:
            for nodePoint in self.nodes[nodeTypeName]:
                if abs(np.dot(nodePoint - point, normal)) <= tolerance:
                    nodeList += [cnt]
                cnt+=1
        return nodeList

    #**classFunction: get node numbers in cube, given by pMin and pMax, containing the minimum and maximum x, y, and z coordinates
    #**example:
    #nList = GetNodesInCube([-1,-0.2,0],[1,0.5,0.5])
    #**output: returns list of nodes; if no nodes found, return an empty list
    def GetNodesInCube(self, pMin, pMax):
        cnt = 0
        nodeList=[]
        if len(self.nodes) != 1 or 'Position' not in self.nodes:
            raise ValueError('FEMinterface.GetNodesInCube: only Position type nodes allowed')
        for nodeTypeName in self.nodes:
            for nodePoint in self.nodes[nodeTypeName]:
                if (nodePoint[0] >= pMin[0] and nodePoint[0] <= pMax[0] and
                    nodePoint[1] >= pMin[1] and nodePoint[1] <= pMax[1] and
                    nodePoint[2] >= pMin[2] and nodePoint[2] <= pMax[2]):
                    nodeList += [cnt]
                cnt+=1
        return nodeList

    #**classFunction: get node numbers lying on line defined by points p1 and p2 and tolerance, which is accepted for points slightly outside the surface
    def GetNodesOnLine(self, p1, p2, tolerance=1e-5):
        return self.GetNodesOnCylinder(self, p1, p2, radius=0, tolerance=1e-5)

    #**classFunction: get node numbers lying on cylinder surface; cylinder defined by cylinder axes (points p1 and p2), 
    #  cylinder radius and tolerance, which is accepted for points slightly outside the surface
    #  if not found, it returns an empty list
    def GetNodesOnCylinder(self, p1, p2, radius, tolerance=1e-5):
        cnt = 0
        v0 = np.array(p2) - np.array(p1)
        lAxis = np.linalg.norm(v0)
        if lAxis != 0:
            v0 = v0/lAxis
            
        nodeList=[]
        if len(self.nodes) != 1 or 'Position' not in self.nodes:
            raise ValueError('FEMinterface.GetNodesOnCylinder: only Position type nodes allowed')

        for nodeTypeName in self.nodes:
            for nodePoint in self.nodes[nodeTypeName]:
                p = np.array(nodePoint)
                v1 = p-p1
                s = v0 @ v1
                
                if s <= lAxis+tolerance and s >= -tolerance:
                    pp = p1 + s*v0 #projected point
                    r = np.linalg.norm(p-pp) #shortest distance to axis
                    if abs(r-radius) <= tolerance:
                        nodeList += [cnt]
                cnt+=1
        return nodeList

    #**classFunction: get node numbers lying on a circle, by point p, (normalized) normal vector n (which is the axis of the circle) and radius r
    #using a tolerance for the distance to the plane
    #if not found, it returns an empty list
    def GetNodesOnCircle(self, point, normal, r, tolerance = 1e-5):
        cnt = 0
        nodeList=[]
        if len(self.nodes) != 1 or 'Position' not in self.nodes:
            raise ValueError('FEMinterface.GetNodesOnCylinder: only Position type nodes allowed')
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
            
        if sparse:
            from scipy import sparse
            xBlock = sparse.kron(sparse.eye(nNodes), X) #create big block-diagonal matrix
            G=np.dot(xBlock,CSRtoScipySparseCSR(self.massMatrix))
        else:
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

    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: create GenericODE2 object out of (linear) FEM model; uses always the sparse matrix mode, independent of the solver settings; this model can be directly used inside the multibody system as a static or dynamic FEM subsystem undergoing small deformations; computation is several magnitudes slower than ObjectFFRFreducedOrder
    #**input:
    #  mbs: multibody system to which the GenericODE2 is added
    #**output: return list [oGenericODE2, nodeList] containing object number of GenericODE2 as well as the list of mbs node numbers of all NodePoint nodes
    def CreateLinearFEMObjectGenericODE2(self, mbs, color=[0.9,0.4,0.4,1.]):
        import exudyn as exu
        femNodes = fem.GetNodePositionsAsArray()
        
        #add nodes:
        allNodeList = [] #create node list
        for node in femNodes:
            allNodeList += [mbs.AddNode(NodePoint(referenceCoordinates=node))]
        
        nRows = fem.NumberOfCoordinates()
        Mcsr = exu.MatrixContainer()
        Mcsr.SetWithSparseMatrixCSR(nRows,nRows,fem.GetMassMatrix(sparse=True), useDenseMatrix=False)
        # Mcsr.SetWithDenseMatrix(fem.GetMassMatrix(sparse=False), useDenseMatrix=True)
        Kcsr = exu.MatrixContainer()
        Kcsr.SetWithSparseMatrixCSR(nRows,nRows,fem.GetStiffnessMatrix(sparse=True), useDenseMatrix=False)
        # Kcsr.SetWithDenseMatrix(fem.GetStiffnessMatrix(sparse=False), useDenseMatrix=True)
        
        #now add generic body built from FEM model with mass and stiffness matrix (optional damping could be added):
        oGenericODE2 = mbs.AddObject(ObjectGenericODE2(nodeNumbers = allNodeList, 
                                                        massMatrix=Mcsr, 
                                                        stiffnessMatrix=Kcsr,
                                                        #forceVector=np.zeros(nRows), 
                                                        #forceUserFunction=UFforce,
                                                        visualization=VObjectGenericODE2(triangleMesh = fem.GetSurfaceTriangles(), color=color)
                                                        ))
        return [oGenericODE2, allNodeList]
        
    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: import mesh from NETGEN/NGsolve and setup mechanical problem
    #**classFunction: create GenericODE2 object fully nonlinear FEM model using NGsolve; uses always the sparse matrix mode, independent of the solver settings; this model can be directly used inside the multibody system as a static or dynamic nonlinear FEM subsystem undergoing large deformations; computation is several magnitudes slower than ObjectFFRFreducedOrder
    #**input:
    #    mbs: multibody system to which the GenericODE2 is added
    #    mesh: a previously created \texttt{ngs.mesh} (NGsolve mesh, see examples)
    #    youngsModulus: Young's modulus used for mechanical model
    #    poissonsRatio: Poisson's ratio used for mechanical model
    #    density: density used for mechanical model
    #    meshOrder: use 1 for linear elements and 2 for second order elements (recommended to use 2 for much higher accuracy!)
    #**output: return list [oGenericODE2, nodeList] containing object number of GenericODE2 as well as the list of mbs node numbers of all NodePoint nodes
    #**notes: The interface to NETGEN/NGsolve has been created together with Joachim Sch\"oberl, main developer 
    #  of NETGEN/NGsolve; Thank's a lot!
    #  download NGsolve at: https://ngsolve.org/
    #  NGsolve needs Python 3.7 (64bit) ==> use according EXUDYN version!
    #  note that node/element indices in the NGsolve mesh are 1-based and need to be converted to 0-base!
    #**author: Johannes Gerstmayr, Joachim Sch\"oberl
    def CreateNonlinearFEMObjectGenericODE2NGsolve(self, mbs, mesh, 
                                                   density, youngsModulus, poissonsRatio, 
                                                   meshOrder=1, color=[0.9,0.4,0.4,1.]):
        import ngsolve as ngs
        import exudyn as exu
        from scipy.sparse import csr_matrix
        
        if meshOrder < 1 or meshOrder > 2:
            raise ValueError('mesh order > 1 or mesh order < 2 not supported!')
            
        if meshOrder == 2:
            mesh.ngmesh.SecondOrder()

        nu = poissonsRatio
        mu  = youngsModulus / 2 / (1+nu)
        lam = youngsModulus * nu / ((1+nu)*(1-2*nu))
        
        #compute Green-Lagrange (nonlinear) strain tensor from displacement field
        def GLstrain(u):
            #F = ngs.Id(3) + ngs.Grad(u)
            #return 0.5*(F.trans * F - ngs.Id(3))
            return ngs.Sym(ngs.Grad(u)) + 0.5*(ngs.Grad(u).trans * ngs.Grad(u))
        
        #compute 2nd PK-stress from strain
        def Strain2Stress(eps, mu, lam):
            return 2*mu*eps + lam*ngs.Trace(eps) * ngs.Id(eps.dims[0])
        
        #deformation energy acc. to St. Venant-Kirchhoff material
        def DeformationEnergy (GLstrain, mu, lam):
            #return 0.5*lam*(ngs.Trace(GLstrain))**2 + mu * ngs.Trace(GLstrain*GLstrain) #2*mu/lam*Det(C)**(-lam/2/mu)-1)
            return 0.5*ngs.Trace(Strain2Stress(GLstrain,mu,lam).trans*GLstrain)
        
        #do not add boundary conditions here, otherwise stiffness matrix cannot be exported!
        fes = ngs.NodalFESpace(mesh, order=meshOrder)**3
        uu = fes.TrialFunction()
        v = fes.TestFunction()
    
        a = ngs.BilinearForm(fes)
        a += ngs.Variation(DeformationEnergy(GLstrain(uu), mu, lam).Compile()*ngs.dx)
        #linear:
        #a += ngs.InnerProduct(Strain2Stress(ngs.Sym(ngs.Grad(uu)),mu,lam), ngs.Sym(ngs.Grad(v)))*ngs.dx
        
        #define grid function to work with in nonlinear solver:
        u = ngs.GridFunction(fes)
        u.vec[:] = 0
            
        fem=FEMinterface()
        #this is mainly needed for triangleMesh, but also creates the linearized matrices, but ok:
        fem.ImportMeshFromNGsolve(mesh, density=density, 
                                  youngsModulus=youngsModulus, poissonsRatio=poissonsRatio, 
                                  meshOrder=meshOrder)

        #create nodes:
        femNodes = fem.GetNodePositionsAsArray()
        
        #add nodes:
        allNodeList = [] #create node list
        for node in femNodes:
            allNodeList += [mbs.AddNode(NodePoint(referenceCoordinates=node))]
        
        nRows = fem.NumberOfCoordinates()
        Mcsr = exu.MatrixContainer()
        Mcsr.SetWithSparseMatrixCSR(nRows,nRows,fem.GetMassMatrix(sparse=True), useDenseMatrix=False)
        Kcsr = exu.MatrixContainer()
        Kcsr.SetWithDenseMatrix(fem.GetStiffnessMatrix(sparse=True), useDenseMatrix=False)
        
        res = u.vec.CreateVector() #temporary vector
        
        #compute RHS of FEM object
        def UFforce(mbs, t, itemIndex, q, q_t):
            u.vec[:] = ResortIndicesExudyn2NGvector(np.array(q))
            a.Apply(u.vec, res)
            resNonlin = ResortIndicesOfNGvector(-res.FV().NumPy())
            return resNonlin

        #jacobian function for FEM object:
        #put some of the following functions into PyMatrixContainer for higher efficiency
        def UFjacobian(mbs, t, itemNumber, q, q_t, fODE2, fODE2_t):
            u.vec[:] = ResortIndicesExudyn2NGvector(np.array(q))
            a.AssembleLinearization(u.vec)
            Knonlinear = fODE2*csr_matrix( a.mat.CSR())#, copy=True )
            Kexu = ScipySparseCSRtoCSR(Knonlinear)
            nMK = Knonlinear.shape[0] #get size of mass matrix; assume square matrix!
            ResortIndicesOfCSRmatrix(Kexu, nMK)
            MCK = exu.MatrixContainer()
            MCK.SetWithSparseMatrixCSR(nMK,nMK,Kexu)
            return MCK
        
        #now add generic body built from FEM model with mass and stiffness matrix (optional damping could be added):
        oGenericODE2 = mbs.AddObject(ObjectGenericODE2(nodeNumbers = allNodeList, 
                                                        massMatrix=Mcsr, 
                                                        forceUserFunction=UFforce,
                                                        jacobianUserFunction=UFjacobian,
                                                        visualization=VObjectGenericODE2(triangleMesh = fem.GetSurfaceTriangles(), color=color)
                                                        ))

        return [oGenericODE2, allNodeList]
        
    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: compute nModes smallest eigenvalues and eigenmodes from mass and stiffnessMatrix; store mode vectors in modeBasis, but exclude a number of 'excludeRigidBodyModes' rigid body modes from modeBasis; uses scipy for solution of generalized eigenvalue problem
    #**input: 
    #  nModes: prescribe the number of modes to be computed; total computed modes are  (nModes+excludeRigidBodyModes), but only nModes with smallest absolute eigenvalues are considered and stored
    #  excludeRigidBodyModes: if rigid body modes are expected (in case of free-free modes), then this number specifies the number of eigenmodes to be excluded in the stored basis (usually 6 modes in 3D)
    #  useSparseSolver: for larger systems, the sparse solver needs to be used, which iteratively solves the problem and uses a random number generator (internally in ARPACK): therefore, results are not fully repeatable!!!
    #**notes: for NGsolve / Netgen meshes, see the according ComputeEigenmodesNGsolve function, which is usually much faster
    #**output: eigenmodes are stored internally in FEMinterface as 'modeBasis' and eigenvalues as 'eigenValues'
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
                              'type':'NormalModes'}
            self.eigenValues = abs(eigVals[excludeRigidBodyModes:excludeRigidBodyModes + nModes])

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: compute eigenmodes, using a set of boundary nodes that are all fixed; very similar to ComputeEigenmodes, but with additional definition of (fixed) boundary nodes.
    #**input: 
    #  boundaryNodes: a list of boundary node indices, refering to 'Position' type nodes in FEMinterface; all coordinates of these nodes are fixed for the computation of the modes
    #  nEigenModes: prescribe the number of modes to be computed; only nEigenModes with smallest abs(eigenvalues) are considered and stored
    #  useSparseSolver: [yet NOT IMPLEMENTED] for larger systems, the sparse solver needs to be used, which iteratively solves the problem and uses a random number generator (internally in ARPACK): therefore, results are not fully repeatable!!!
    #**output: eigenmodes are stored internally in FEMinterface as 'modeBasis' and eigenvalues as 'eigenValues'
    def ComputeEigenModesWithBoundaryNodes(self, 
                                      boundaryNodes,
                                      nEigenModes, 
                                      useSparseSolver = True):
        if not useSparseSolver:
            #unsorted, dense eigen vectors
            from scipy.linalg import solve, eigh, eig #eigh for symmetric matrices, positive definite
    
            K = self.GetStiffnessMatrix(sparse=False)
            M = self.GetMassMatrix(sparse=False)
            
            if len(boundaryNodes) != 0:
                #sizes of internal and boundary nodes:
                n = len(M)
                nb = len(boundaryNodes)*3
                ni = n-nb
        
                #compute indices for internal and boundary DOF/coordinates:
                DOFb = []
                for node in boundaryNodes:
                    DOFb += [node*3,node*3+1,node*3+2]
                
                DOFb = np.array(DOFb)
                DOFb.sort()
                DOFi = np.arange(n)
                DOFi = np.delete(DOFi, DOFb)
                DOFstatic = np.arange(nb) #for final mapping of boundary coordinates
                DOFeig = np.arange(nb,nb+nEigenModes) #for final mapping of eigenmode coordinates
                
                print("n=", n, ", nb=",nb, ", ni=", ni)
                #print("DOFb=", DOFb)
                #print("DOFi=", DOFi)
                
                #create mass and stiffness matrices with new indices:
                Mii = M[np.ix_(DOFi,DOFi)]
                Kii = K[np.ix_(DOFi,DOFi)]
                Kib = K[np.ix_(DOFi,DOFb)]
    
                print("solve eigenvalues...")
                [eigVals, eigVecs] = eigh(Kii,Mii) #this gives omega^2 ... squared eigen frequencies (rad/s)
    
                print("solve static modes...")
                KiiInvKib = -np.linalg.inv(Kii) @ Kib
    
                print("assemble matrices ...")
                if False:
                    modeBasis = np.zeros((n, nb+nEigenModes))
                    modeBasis[np.ix_(DOFi,DOFeig)] = eigVecs[:,:nEigenModes]
                    modeBasis[np.ix_(DOFb,DOFstatic)] = np.eye(nb)
                    modeBasis[np.ix_(DOFi,DOFstatic)] = KiiInvKib
                else:
                    modeBasis = np.zeros((n, nEigenModes))
                    DOFeig = np.arange(nEigenModes) #for final mapping of eigenmode coordinates
                    modeBasis[np.ix_(DOFi,DOFeig)] = eigVecs[:,:nEigenModes]
                
                #print(modeBasis.shape)
                #print(modeBasis.round(2))
               
                self.modeBasis = {'matrix':modeBasis, 'type':'NormalModes'}
                self.eigenValues = abs(eigVals)
    
            else:
                [eigVals, eigVecs] = eigh(K,M) #this gives omega^2 ... squared eigen frequencies (rad/s)
                self.modeBasis = {'matrix':eigVecs[:,excludeRigidBodyModes:excludeRigidBodyModes + nEigenModes], 'type':'NormalNodes'}
                self.eigenValues = abs(eigVals)
        else:
            raise ValueError("ComputeEigenModesWithBoundaryNodes: only implemented for dense mode")
    
    
    
    
    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: compute static  and eigen modes based on Hurty-Craig-Bampton, for details see theory part \refSection{sec:theory:CMS}. Note that this function may need significant time, depending on your hardware, but 50.000 nodes will require approx. 1-2 minutes and more nodes typically raise time more than linearly.
    #**input:
    #  boundaryNodesList: [nodeList0, nodeList1, ...] a list of node lists, each of them representing a set of 'Position' nodes for which a rigid body interface (displacement/rotation and force/torque) is created; NOTE THAT boundary nodes may not overlap between the different node lists (no duplicated node indices!)
    #  nEigenModes: number of eigen modes in addition to static modes (may be zero for RBE2 computationMode); eigen modes are computed for the case where all rigid body motions at boundaries are fixed; only smallest nEigenModes absolute eigenvalues are considered
    #  useSparseSolver: for more than approx.~500 nodes, it is recommended to use the sparse solver
    #  computationMode: see class HCBstaticModeSelection for available modes; select RBE2 as standard, which is both efficient and accurate and which uses rigid-body-interfaces (6 independent modes) per boundary
    #**notes: for NGsolve / Netgen meshes, see the according ComputeHurtyCraigBamptonModesNGsolve function, which is usually much faster
    #**output: stores computed modes in self.modeBasis and abs(eigenvalues) in self.eigenValues
    def ComputeHurtyCraigBamptonModes(self,
                                      boundaryNodesList,
                                      nEigenModes, 
                                      useSparseSolver = True,
                                      computationMode = HCBstaticModeSelection.RBE2):

        #only makes sense for RBE3 modes:  positionOnlyModes: provide empty list [] to compute rigid body interfaces for all boundary node lists, or a boolean list [False, False, True, ...] to indicate which modes only have 3 position but no rotation modes; only valid for computationMode = RBE2 
    
        #unsorted, dense eigen vectors
        from scipy.linalg import solve, eigh, eig #eigh for symmetric matrices, positive definite
        from scipy.linalg import block_diag
        import time #for some timers

        timerTreshold = 20000 #for more DOF than this number, show CPU times
    
        if useSparseSolver: 
            from scipy.sparse.linalg import eigsh #eigh for symmetric matrices, positive definite
            from scipy.sparse.linalg import factorized
    
            K = CSRtoScipySparseCSR(self.GetStiffnessMatrix(sparse=True))
            M = CSRtoScipySparseCSR(self.GetMassMatrix(sparse=True))
    
        else: #not recommended for more than 2000 nodes (6000 DOF)!
            K = self.GetStiffnessMatrix(sparse=False)
            M = self.GetMassMatrix(sparse=False)
            
        #implementation with RBE2 boundary mode
        if len(boundaryNodesList) != 0:
            
            n = M.shape[0] #size of mass and stiffness matrix; assume square matrix!
            nb = 0
            DOFb = []
            nNodeLists = len(boundaryNodesList)
            if 'Position' not in self.nodes or len(self.nodes) != 1:
                raise ValueError('ComputeHurtyCraigBamptonModes: nodes in FEMinterface must be of position type!')
            nodesPos = self.nodes['Position']
            boundaryNodesMidPoints = []
            
            #determine sizes and some parameters:
            for boundaryNodes in boundaryNodesList:
            #sizes of internal and boundary nodes:
                nb += len(boundaryNodes)*3
                #compute indices for internal and boundary DOF/coordinates:
                for node in boundaryNodes:
                    DOFb += [node*3,node*3+1,node*3+2]
    
                #compute midpoints of boundary nodes:
                p = np.zeros(3)
                for node in boundaryNodes:
                    p += nodesPos[node]
                
                boundaryNodesMidPoints += [p*(1./len(boundaryNodes))]
    
            #print("midpoints=",boundaryNodesMidPoints)
    
            ni = n-nb
    
            #compute boundary and internal DOF numbers:
            DOFb = np.array(DOFb)
            #DOFb.sort() #do not sort to keep consistency between sorting of nodes and DOFb
            DOFi = np.arange(n)
            DOFi = np.delete(DOFi, DOFb) #sorting not needed for DOFb
            
            #print("n=", n, ", nb=",nb, ", ni=", ni)
            #print("DOFb=", DOFb)
            #print("DOFi=", DOFi)
            
            #create mass and stiffness matrices with new indices:
            if useSparseSolver: 
                #A = B.tocsr()[np.array(list1),:].tocsc()[:,np.array(list2)] faster?
                #takes 0,042 seconds for 16000 nodes ...
                Mii = M[DOFi,:][:,DOFi] #these matrices are np.array (dense) or sparse ...
                Kii = K[DOFi,:][:,DOFi]
                Kib = K[DOFi,:][:,DOFb]
                
            else:
                #works also for sparse matrices, but computes dense matrices in between ...
                Mii = M[np.ix_(DOFi,DOFi)] 
                Kii = K[np.ix_(DOFi,DOFi)]
                Kib = K[np.ix_(DOFi,DOFb)]
            #Mii, Kii, Kib are now np.array (dense) or sparse ...
        
            if nEigenModes != 0:
                if n>timerTreshold: print("compute eigenvalues and eigenvectors... "); start_time = time.time()
                if useSparseSolver: 
                    #for details on solver settings, see FEMinterface.ComputeEigenmodes(...)
                    [eigVals, eigVecs] = eigsh(A=Kii, k=nEigenModes, M=Mii, 
                                               which='LM', sigma=0, mode='normal') #try modes 'normal','buckling' and 'cayley'
                else:
                    [eigVals, eigVecs] = eigh(Kii,Mii) #this gives omega^2 ... squared eigen frequencies (rad/s)
                if n>timerTreshold: print("   ... needed %.3f seconds" % (time.time() - start_time))
    
            #print("assemble matrices ...")

            if computationMode == HCBstaticModeSelection.allBoundaryNodes: #quite inefficient, because it 
                modeBasis = np.zeros((n, nb+nEigenModes))
                DOFstatic = np.arange(nb) #for final mapping of boundary coordinates
                if nEigenModes != 0:
                    DOFeig = np.arange(nb,nb+nEigenModes) #for final mapping of eigenmode coordinates
                    modeBasis[np.ix_(DOFi,DOFeig)] = eigVecs[:,:nEigenModes]
                
                modeBasis[np.ix_(DOFb,DOFstatic)] = np.eye(nb)
                if n>timerTreshold: print("factorize Kii... "); start_time = time.time()
                if useSparseSolver: 
                    invKii = factorized(Kii.tocsc()) #factorized expects csc format, otherwise warning
                    KiiInvKib = invKii(-Kib.toarray())
                else:
                    KiiInvKib = -np.linalg.inv(Kii) @ Kib
                if n>timerTreshold: print("   ... needed %.3f seconds" % (time.time() - start_time))
                modeBasis[np.ix_(DOFi,DOFstatic)] = KiiInvKib
    
            elif computationMode == HCBstaticModeSelection.RBE2:
                addRotationModes = 1
                rbSize = 3 + 3*addRotationModes #size of rigid body coordinates (3 for translation, 6 for translation+rotation)
                nbRBE2 = (nNodeLists-1)*rbSize #number of chosen static modes, 6 DOF per rigid body interface; exclude first rigid body boundary in order to suppress rigid body motion of static modes
    
                modeBasis = np.zeros((n, nbRBE2+nEigenModes))
                if nEigenModes != 0:
                    DOFeig = np.arange(nbRBE2,nbRBE2+nEigenModes) #for final mapping of eigenmode coordinates
                    modeBasis[np.ix_(DOFi,DOFeig)] = eigVecs[:,:nEigenModes]
                
                #create list of mappings between average rigid body motion and boundary DOF matrix
                rigidBodyMappings = [[]]*nNodeLists #list of mappings
                cntBoundary = 0 #counter for boundaryNodeLists / number of interfaces
                for boundaryNodes in boundaryNodesList:
                    nn = len(boundaryNodes)
                    #compute mapping from average displacement to displacement at boundary nodes:
                    T = np.kron(np.ones(nn),np.eye(3)).T #maps rigid body motion of interface to all boundary nodes
                    
                    #compute mapping from (averaged) rotation at boundary to displacement of boundary nodes:
                    if addRotationModes:
                        Trot = np.zeros((nn*3,3))
                        p0 = boundaryNodesMidPoints[cntBoundary]
                        for i in range(3): #iterate about 3 rotation axes
                            rot = np.zeros(3) #rotation vector, unit rotation
                            rot[i] = 1
                            rotTilde = Skew(rot)
                            for j in range(len(boundaryNodes)):
                                p = nodesPos[boundaryNodes[j]]-p0
                                qRot = rotTilde@p
                                Trot[j*3:j*3+3,i] = qRot
                        T = np.hstack((T,Trot))
                                
    
                    rigidBodyMappings[cntBoundary] = T
                    cntBoundary += 1
    
                
                Tall = block_diag(*rigidBodyMappings)  # '*' does unpacking of lists;
                Tall = Tall[:,rbSize:] #exclude first rigid body boundary in order to suppress rigid body motion of static modes
    
                DOFstatic = np.arange(nbRBE2) #for final mapping of boundary coordinates; 
                modeBasis[np.ix_(DOFb,DOFstatic)] = Tall
                if n>timerTreshold: print("factorize Kii... "); start_time = time.time()
                if useSparseSolver: 
                    invKii = factorized(Kii.tocsc()) #factorized expects csc format, otherwise warning
                    KiiInvKibTall = invKii(-(Kib @ Tall)) #(Kib @ Tall) gives already dense matrix; may be huge ...!
                else:
                    KiiInvKibTall = -np.linalg.inv(Kii) @ (Kib @ Tall)
                if n>timerTreshold: print("   ... needed %.3f seconds" % (time.time() - start_time))
    
                modeBasis[np.ix_(DOFi,DOFstatic)] = KiiInvKibTall #KiiInvKib @ Tall
    
                # modeBasis[np.ix_(DOFb,DOFstatic)] = np.eye(nb)
                # modeBasis[np.ix_(DOFi,DOFstatic)] = KiiInvKib
                
            elif computationMode == HCBstaticModeSelection.onlyEigenModes: #only eigen modes, e.g., for testing
                if nEigenModes != 0:
                    raise ValueError('ComputeHurtyCraigBamptonModes: in computationMode onlyEigenModes, nEigenModes must be != 0')

                modeBasis = np.zeros((n, nEigenModes))
                DOFeig = np.arange(nEigenModes) #for final mapping of eigenmode coordinates
                modeBasis[np.ix_(DOFi,DOFeig)] = eigVecs[:,:nEigenModes]
            
            #print(modeBasis.shape)
            #print(modeBasis.round(2))
           
            self.modeBasis = {'matrix':modeBasis, 'type':'HCBmodes'}
            if nEigenModes != 0:
                self.eigenValues = abs(eigVals)
            else:
                self.eigenValues = np.array([])
    
        else:
            [eigVals, eigVecs] = eigh(K,M) #this gives omega^2 ... squared eigen frequencies (rad/s)
            self.modeBasis = {'matrix':eigVecs[:,0:nEigenModes], 'type':'NormalModes'}
            self.eigenValues = abs(eigVals)



    #**classFunction: return list of eigenvalues in Hz of previously computed eigenmodes
    def GetEigenFrequenciesHz(self):
        return np.sqrt(self.eigenValues)/(2.*np.pi)

    #internal function for ComputePostProcessingModes, do not call from outside FEM
    def InternalComputePostprocessingMode(self,parameterList):
        #map list of parameters to internal variables (due to multiprocessing)
        iMode = parameterList[0]
        nodes = parameterList[1]
        modes = parameterList[2]
        elemList = parameterList[3]
        material = parameterList[4]
        computeStrains = parameterList[5]
        
        nNodes = len(nodes)
        nodesPerTet = 4
        elemRefCoords = np.zeros(nodesPerTet*3)
        displacements = np.zeros(nodesPerTet*3)
        stressModesCnt = np.zeros(nNodes) #store how many elements contribute to nodal stress (for averaging)

        stressModesMatrix = np.zeros((nNodes, 6)) #add up nodal stresses in this 6 modes for sigma_xx, sigma_yy, etc.
        for elem in elemList:
            for cnt in range(len(elem)):
                ind = elem[cnt]
                elemRefCoords[cnt*3:cnt*3+3] = nodes[ind,:]
                displacements[cnt*3:cnt*3+3] = modes[ind*3:ind*3+3,iMode]
                
            tet=Tet4(material) #all material is the same ...
            [Ev4, Sv4, B0, grad]=tet.ComputeMatrices(elemRefCoords, displacements)

            #now write stresses into stress modes
            for cnt in range(4): #number of nodes per element
                ind = elem[cnt]  #node index
                #if iMode == 0: #count how often nodes need to be averaged (count only for first mode)
                stressModesCnt[ind] += 1
                for j in range(6): #6 components
                    if computeStrains:
                        stressModesMatrix[ind,j] += Ev4[cnt, j]
                    else:
                        stressModesMatrix[ind,j] += Sv4[cnt, j]
                # for j in range(6): #6 components
                #     if computeStrains:
                #         stressModes[ind,6*iMode+j] += Ev4[cnt, j]
                #     else:
                #         stressModes[ind,6*iMode+j] += Sv4[cnt, j]
        nodeWarned = False
        for ind in range(nNodes):
            elPerNode = stressModesCnt[ind]
            if elPerNode == 0:
                if not nodeWarned:
                    nodeWarned = True
                    print('********\nWARNING:\n********\n Compute stress/strain modes: averaging of stress/strain at nodes failed, because node not connected to elements; this function only works for linear elements!')
                elPerNode = 1 #does not matter because no element attached, no stress computed
            stressModesMatrix[ind,:] *= 1/elPerNode

        return stressModesMatrix


    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: compute special stress or strain modes in order to enable visualization of stresses and strains in ObjectFFRFreducedOrder;
    #**input: 
    #  material: specify material properties for computation of stresses, using a material class, e.g. material = KirchhoffMaterial(Emodulus, nu, rho); not needed for strains
    #  outputVariableType: specify either exudyn.OutputVariableType.StressLocal or exudyn.OutputVariableType.StrainLocal as the desired output variables
    #  numberOfThreads: if numberOfThreads=1, it uses single threaded computation; if numberOfThreads>1, it uses the multiprocessing pools functionality, which requires that all code in your main file must be encapsulated within an if clause "if \_\_name\_\_ == '\_\_main\_\_':", see examples; if numberOfThreads==-1, it uses all threads/CPUs available
    #**notes: This function is implemented in Python and rather slow for larger meshes; for NGsolve / Netgen meshes, see the according ComputePostProcessingModesNGsolve function, which is usually much faster
    #**output: post processing modes are stored in FEMinterface in local variable postProcessingModes as a dictionary, where 'matrix' represents the modes and 'outputVariableType' stores the type of mode as a OutputVariableType
    def ComputePostProcessingModes(self, material=0, 
                                   outputVariableType='OutputVariableType.StressLocal',
                                   numberOfThreads=1):
        #import exudyn as exu #needed for outputVariableType

        if str(outputVariableType) == 'OutputVariableType.StressLocal':
            computeStrains = False
        elif str(outputVariableType) == 'OutputVariableType.StrainLocal':
            computeStrains = True
        else:
            raise ValueError('ComputePostProcessingModes invoked with invalid outputVariableType')
            
        nodes = self.nodes['Position']
        nNodes = len(nodes)
        if len(self.elements) != 1:
            raise ValueError('ComputePostProcessingModes(...): only implemented for FEMinterface with one list of elements')
        if 'Tet4' not in self.elements[0]:
            raise ValueError('ComputePostProcessingModes(...): only implemented for Tet4 elements')
        if 'matrix' not in self.modeBasis:
            raise ValueError('ComputePostProcessingModes(...): modeBasis needs to be computed in FEMinterface prior to calling ComputePostProcessingModes; use e.g. ComputeEigenmodes(...)')
            
        elemList = self.elements[0]['Tet4']
        modes = self.modeBasis['matrix']
        nModes = modes.shape[1]
        stressModes = np.zeros((nNodes, 6*nModes)) #add up nodal stresses

        if material == 0:
            material=KirchhoffMaterial(1, 0, 1)
            if not computeStrains:
                raise ValueError('ComputePostProcessingModes: if material=0, outputVariableType must be StrainLocal')

        showProgress = False
        if nModes*nNodes > 10000:
            showProgress = True
            #print("")

        #create vectorized input data for ComputePostprocessingMode
        vectorInput = [[]]*nModes
        for i in range(nModes):
            vectorInput[i] = [i,nodes, modes, elemList, material, computeStrains]
            

        useSingleThreading = True
        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        if numberOfThreads > 1 or numberOfThreads == -1:
            try:
                from multiprocessing import Pool, cpu_count #parallelization of computation
                if numberOfThreads == -1:
                    numberOfThreads = cpu_count() #cpu_count in fact gives number of threads ...
                useSingleThreading = False
            except:
                pass
            if not useSingleThreading:
                vectorInput = np.array(vectorInput)
                
                useTQDM = False
                if showProgress:
                    try:
                        import tqdm #progress bar
                        try: #_instances only available after first run!
                            tqdm.tqdm._instances.clear() #if open instances of tqdm, which leads to nasty newline
                        except:
                            pass
                        useTQDM = True
                        print("useTQDM")
                    except:
                        pass
                
                if useTQDM:
                    with Pool(processes=numberOfThreads) as p:
                        values = list(tqdm.tqdm(p.imap(self.InternalComputePostprocessingMode, vectorInput), 
                                                total=nModes))
                    print("", flush=True) #newline after tqdm progress bar output....
                else:
                    # simpler approach without tqdm:
                    with Pool(processes=numberOfThreads) as p:
                        values = p.map(self.InternalComputePostprocessingMode, vectorInput)

                for iMode in range(nModes):
                    stressModes[:,6*iMode:6*iMode+6] = values[iMode]
                
            
        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        if useSingleThreading:
            for iMode in range(nModes):
                stressModeMatrix = self.InternalComputePostprocessingMode(vectorInput[iMode])
                stressModes[:,6*iMode:6*iMode+6] = stressModeMatrix
                
                if showProgress:
                    print("\rComputePostProcessingModes: " + str(iMode/nModes*100) + str("%"),end='', flush=True)

            if showProgress:
                print("") #line break finally

        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        self.postProcessingModes = {'matrix': stressModes, 
                                    'outputVariableType': outputVariableType}
        
        #self.postProcessingModes = {}   # {'matrix':<matrix containing stress components (xx,yy,zz,yz,xz,xy) in each column, rows are for every mesh node>,'outputVariableType':exudyn.OutputVariableType.StressLocal}

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
    #  verbose: if True, shows progress of computation; if verbose=2, prints also eigenfrequencies
    #  useCorotationalFrame: if False, the classic rotor dynamics formulation for rotationally-symmetric rotors is used, where the rotor can be understood in a Lagrangian-Eulerian manner: the rotation is represented by an additional (Eulerian) velocity in rotation direction; if True, the corotational frame is used, which gives a factor 2 in the gyroscopic matrix and can be used for non-symmetric rotors as well
    #  useSparseSolver: for larger systems, the sparse solver needs to be used for creation of system matrices and for the eigenvalue solver (uses a random number generator internally in ARPACK, therefore, results are not fully repeatable!!!)
    #**output: [listFrequencies, campbellFrequencies]
    #  listFrequencies: list of computed frequencies
    #  campbellFrequencies: array of campbell frequencies per eigenfrequency of system
    def ComputeCampbellDiagram(self, terminalFrequency, nEigenfrequencies=10, frequencySteps=25, 
                               rotationAxis=2, plotDiagram=False, verbose=False, 
                               useCorotationalFrame=False, useSparseSolver=False):
        from scipy.linalg import eig #eigh for symmetric matrices, positive definite
        
        #create gyroscopic terms
#        X=np.array([[ 0., -1.,  0.],
#                    [ 1.,  0.,  0.],
#                    [ 0.,  0.,  0.]])
#        xBlock = np.kron(np.eye(nNodes), X) #create big block-diagonal matrix
#        G=np.dot(xBlock,M)
        
        if self.NumberOfCoordinates() > 1000:
            print('WARNING: ComputeCampbellDiagram(...): system has more than 1000 coordinates, set useSparseSolver=True')
            if self.NumberOfCoordinates() > 5000:
                raise ValueError('ComputeCampbellDiagram(...): system has more than 5000 coordinates, MUST set useSparseSolver=True')

        #dense matrix version:
        if not useSparseSolver:        
            M = self.GetMassMatrix(sparse=False)
            K = self.GetStiffnessMatrix(sparse=False)
            G = self.GetGyroscopicMatrix(rotationAxis=rotationAxis, sparse=False)
    
            #create system:
            #A*x_t + B*x=0
            nODE = self.NumberOfCoordinates()
            B = np.block([[                    K, np.zeros((nODE,nODE))],
                          [np.zeros((nODE,nODE)), -M                   ]])
        
            factorGyro = 1
            if useCorotationalFrame:
                factorGyro = 2 #this is the only difference to between fixed and corotational frame
    #        terminalFrequencyCampbell = 2*np.pi*225 #rad/s
            campbellFrequencies = []
            listFrequencies = []
    
            for val in range(frequencySteps+1):
                
                omega = val * terminalFrequency * 2*np.pi / frequencySteps
                if verbose:
                    print("compute Campbell for frequency =", round(omega/(2*np.pi),3), " / ", terminalFrequency, '(Hz)')
                A = np.block([[factorGyro*omega * G, M                    ],
                              [                   M, np.zeros((nODE,nODE))]])
        
            
                Amod = -np.dot(np.linalg.inv(A),B)
                #print("Amod =", Amod)
                [eigVals, eigVecs] = eig(Amod) #this gives omega^2 ... squared eigen frequencies (rad/s)
            
                ev = np.sort(eigVals)
            
                listEigAbs = []
                for i in range(len(ev)):
                    v=abs(ev[i].imag/(2*np.pi))
                    if not (v in listEigAbs):
                        listEigAbs += [v]
    
                listEigAbs = np.sort(listEigAbs)
                if verbose == 2:
                    print('  frequencies =',listEigAbs[0:nEigenfrequencies+1])
            
                campbellFrequencies += [list(listEigAbs[0:nEigenfrequencies+1])] #+1 for rigid body mode 0
                listFrequencies += [omega/(2*np.pi)]
        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #SPARSE version:
        else: 
            from scipy import sparse
            from scipy.sparse.linalg import factorized, eigs
            #[M, 0][q_tt] + [omega*G, 0] [q_t] = [0]
            #[0, 0][q_t ] + [       , K] [q  ] = [0]
 
            #M*d(q_t) + omega*G*q_t + K * q = 0
            #d(q) = q_t
            #d(q_t) = -Minv*(omega*G*q_t+K*q)
            
            #       [0      , I      ]
            #d(x) = [                ] * x
            #       [-Minv*K, -omega*Minv*G] 
 
            M = CSRtoScipySparseCSR(self.GetMassMatrix(sparse=True))
            K = CSRtoScipySparseCSR(self.GetStiffnessMatrix(sparse=True))
            G = self.GetGyroscopicMatrix(rotationAxis=rotationAxis, sparse=True) #already in scypi sparse format
    
            nODE = self.NumberOfCoordinates()

            # Minv = factorized(M.tocsc()) #factorized expects csc format, otherwise warning
            # MinvK = sparse.csr_matrix(Minv(-K.toarray())) #slow, but no other way right now
            # MinvG = sparse.csr_matrix(Minv((-G).toarray())) #this is for omega=1, multiplied with factor hereafter!

            #twice as large mass-matrix:
            M2 = sparse.bmat([[M   , None],
                              [None, M   ]]) 
            #M2inv = factorized(M2.tocsc()) 
        
            factorGyro = 1
            if useCorotationalFrame:
                factorGyro = 2 #this is the only difference to between fixed and corotational frame

            campbellFrequencies = []
            listFrequencies = []
    
            for val in range(frequencySteps+1):
                
                omega = val * terminalFrequency * 2*np.pi / frequencySteps
                if verbose:
                    print("compute Campbell for frequency =", round(omega/(2*np.pi),3), " / ", terminalFrequency, '(Hz)')
                # print('nEig:',nEigenfrequencies)
                # A = sparse.bmat([[None , sparse.eye(nODE)        ],
                #                  [MinvK, (factorGyro*omega)*MinvG]]) #in fact negative sign included in MinvK and MinvG!!!
                
                # [eigVals, eigVecs] = eigs(A=A, k=2*(nEigenfrequencies+1), #M=Mii, 
                #                           which='LM', sigma=0)
                A = sparse.bmat([[None , M        ],
                                  [-K, -(factorGyro*omega)*G]]) #in fact negative sign included in MinvK and MinvG!!!
                [eigVals, eigVecs] = eigs(A=A, k=2*(nEigenfrequencies+1), M=M2, #Minv=M2inv,
                                          which='LM', sigma=0)
            
                ev = np.sort(eigVals)
            
                listEigAbs = []
                for i in range(len(ev)):
                    v=abs(ev[i].imag/(2*np.pi))
                    if not (v in listEigAbs):
                        listEigAbs += [v]
    
                listEigAbs = np.sort(listEigAbs)
                if verbose == 2:
                    print('  frequencies =',listEigAbs[0:nEigenfrequencies+1])
            
                campbellFrequencies += [list(listEigAbs[0:nEigenfrequencies+1])] #+1 for rigid body mode 0
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
