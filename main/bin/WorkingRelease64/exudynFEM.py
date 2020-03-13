# Utility functions and structures for Exudyn
"""
Created on 2020-03-10

@author: Johannes Gerstmayr

contents: support functions for finite element models
"""
#constants and fixed structures:
import numpy as np #LoadSolutionFile

#convert sparse matrix data to dense numpy matrix
def CompressedRowToDenseMatrix(sparseData):
    n = int(np.max(sparseData[:,0])) #rows and columns are 1-based
    m = np.zeros((n,n))
    for row in sparseData:
        m[int(row[0])-1,int(row[1])-1] = row[2]
    return m

#read abaqus nodes information to numpy array
#typeName is Part or Instance; name is part's or instance's name
def ReadNodesFromAbaqusInp(fileName, typeName='Part', name='Part-1'):
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
    nodes = [] #store list of node values
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
                    print("ERROR: Expected *Node after *Part or *Instance, line=", lineCnt)
                    errorOccured = True
                else:
                    v = []
                    for i in range(3):
                        v+=[float(lineData[i+1])]
                    nodes += [v] #add node data
            else:
                startReadNodes = False
                finishedReadNodes = True
    
        if startPart and not startReadNodes and not finishedReadNodes:
            if line[0:5] == '*Node':
                startReadNodes = True
                startPart = False
            else:
                print("ERROR: Expected *Node after *Part, line=", lineCnt)
                errorOccured = True
            
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
    
    return np.array(nodes)

