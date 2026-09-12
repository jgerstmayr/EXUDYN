#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is a EXUDYN example
#
# Details:  The 3D movement of a point mass system is simulated.
#           The point masses are connected by spring-damper elements.
#           The system ist modelled in accordance with the Bachelor thesis of Thomas
#           Pfurtscheller (SS/WS19)
#
# Author:   Holzinger Stefan
# Date:     2019-10-14
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

import numpy as np

print('EXUDYN version='+exu.config.Version())

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Modelling of MBS starts here

# define number of nodes in mesh
numberOfNodesX = 6
numberOfNodesY = 4
numberOfNodes = numberOfNodesX*numberOfNodesY

# define undeformed spring length
springDamperElementLength = 1
springDamperDiagnoalElementLength = np.sqrt( springDamperElementLength*springDamperElementLength + springDamperElementLength*springDamperElementLength )

# define mass point mass and spring-damper properties
massMassPoint = 1/2 # each node has half mass, since elements have mass
elementStiffness = 1000
elementDamping = 5
activateElement = 1

# Flags 
clambRightSide = True                       # clamb right side of mesh. If false, only left side of system is clambed
useSpringDamperElementsX = True
useSpringDamperElementsY = True
useSpringDamperElementsDiagonal = True
useSpringDamperElementsOffDiagonal = True
createSolutionFile = False

# this part of the code is used for convergence analysis
# if hend < h0, step size will be reduced until this condition is false
h0 = 1.0e-5
hend = 1.0e-5
stepSizeReductionFactor = 2 # factor which is used to reduce time step size (increase number of steps)

stepSizeList = list()
stepSizeList.append(h0)
ctr1 = 0
while stepSizeList[ctr1] > hend:
    stepSizeList.append( stepSizeList[ctr1]/stepSizeReductionFactor )
    ctr1 = ctr1 + 1


# simulation settings
tEnd = 5.0 # s # simulate system until tend

# actual system definition starts here
for i in range( len(stepSizeList) ):
    
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    
    simulationSettings = exu.SimulationSettings() #takes currently set values or default values
    simulationSettings.solutionSettings.coordinatesSolutionFileName = 'BASpringDamperSystem h=' + str(stepSizeList[i]) + '.txt'
    
    print(int( tEnd/stepSizeList[i] ) )
    
    writeStep = 0.01
    simulationSettings.timeIntegration.numberOfSteps = int( tEnd/stepSizeList[i] )
    simulationSettings.timeIntegration.endTime = tEnd
    simulationSettings.solutionSettings.writeSolutionToFile = createSolutionFile
    simulationSettings.solutionSettings.solutionWritePeriod = writeStep
    simulationSettings.solutionSettings.outputPrecision = 16
    simulationSettings.displayComputationTime = True
    simulationSettings.timeIntegration.verboseMode = 1
    #simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
    #simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1           
    #simulationSettings.displayStatistics = True
    
    
    SC.visualizationSettings.bodies.showNumbers = True
    SC.visualizationSettings.nodes.defaultSize = 0.1
    SC.visualizationSettings.markers.defaultSize = 0.05
    SC.visualizationSettings.connectors.defaultSize = 0.1   
    
    
    
#######################  define nodes #########################################
    nodeCounter = 0     # zero based
    for i in range(numberOfNodesY):  
        for j in range(numberOfNodesX):
          
            nodeName = "node " + str(nodeCounter)
    
            # increase node counter 
            nodeCounter = nodeCounter + 1   
            
            if j == 0: 
                nodeDict = {"nodeType": "PointGround",
                            "referenceCoordinates": [0.0, i*springDamperElementLength, 0.0],
                            "name": nodeName}
     
            elif nodeCounter == (i+1)*numberOfNodesX and clambRightSide == True:
                nodeDict = {"nodeType": "PointGround",
                            "referenceCoordinates": [j*springDamperElementLength, i*springDamperElementLength, 0.0],
                            "name": nodeName}  
                
            else:
                nodeDict = {"nodeType": "Point",
                            "referenceCoordinates": [j*springDamperElementLength, i*springDamperElementLength, 0.0],
                            "initialCoordinates": [0.0, 0.0, 0.0],
                            "name": nodeName}
            
            # add node to mbs
            mbs.AddNode(nodeDict)
    
    
#######################  create objects #######################################        
    for i in range(numberOfNodes):
        
        nodeDict   = mbs.GetNode(i)
        nodeName   = nodeDict["name"]
        nodeNumber = mbs.GetNodeNumber(nodeName)
    
        massPointName = "mass point - " + nodeName
         
        objectDict = {"objectType": "MassPoint",
                      "physicsMass": massMassPoint,
                      "nodeNumber": nodeNumber,
                      "name": massPointName}
        
        mbs.AddObject(objectDict)
    
    
#######################  add markers ##########################################
    bodyMassMarkerName = list()
    nodePositionMarkerName = list()
    for i in range(numberOfNodes):
        
        nodeDict   = mbs.GetNode(i)
        nodeName   = nodeDict["name"]
        nodeNumber = mbs.GetNodeNumber(nodeName)
        
        bodyMassMarkerName.append("body mass marker - " + nodeName)
        nodePositionMarkerName.append("node position marker - " + nodeName)
        
        # body mass - used for garvitational load
        bodyMassMarkerDict = {"markerType": "BodyMass",
                              "bodyNumber": exu.ObjectIndex(nodeNumber), #nodeNumber=bodyNumber
                              "name": bodyMassMarkerName[i]}
        
        nodePositionMarkerDict = {"markerType": "NodePosition",
                                  "nodeNumber": nodeNumber,
                                  "name": nodePositionMarkerName[i]}
        
        mbs.AddMarker(bodyMassMarkerDict)
        mbs.AddMarker(nodePositionMarkerDict)
        
    
#######################  add loads ############################################
    for i in range( len(bodyMassMarkerName) ):
    
        markerNumberOfBodyMassMarker = mbs.GetMarkerNumber(bodyMassMarkerName[i])
        loadName = "gravity " + str(bodyMassMarkerName[i])
          
        loadDict = {"loadType": "MassProportional",
                    "markerNumber": markerNumberOfBodyMassMarker, 
                    "loadVector": [0.0, 0.0, -9.81],
                    "name": loadName}
                    
        mbs.AddLoad(loadDict)
    
    
#######################  add connectors #######################################
    ctr1 = 0
    elementCtr = 0;
    
    # spring-damper elements in x-direction
    if useSpringDamperElementsX == True:
    
        for i in range(numberOfNodesY): 
        
            for j in range(numberOfNodesX-1):
        
                markerNumberOfPositionMarkerL = mbs.GetMarkerNumber(nodePositionMarkerName[ctr1])
                markerNumberOfPositionMarkerR = mbs.GetMarkerNumber(nodePositionMarkerName[ctr1+1])
        
                ctr1 = ctr1 + 1
        
                springDamperElementName = "spring damper" + str(elementCtr)
             
                objectDict = {"objectType": "ConnectorSpringDamper",
                              "markerNumbers": [markerNumberOfPositionMarkerL, markerNumberOfPositionMarkerR],
                              "stiffness": elementStiffness,
                              "damping": elementDamping,
                              "force": 0,
                              "referenceLength": springDamperElementLength,
                              "activeConnector": activateElement,
                              "name": springDamperElementName,
                              "VdrawSize": 0.0}
            
                mbs.AddObject(objectDict)
        
                elementCtr = elementCtr + 1
        
            ctr1 = ctr1 + 1
       
    # spring-damper elements in y-direction
    if useSpringDamperElementsY == True:
        
        ctr1 = 0
        for i in range(numberOfNodesY-1): 
        
            for j in range(numberOfNodesX):
        
                markerNumberOfPositionMarkerL = mbs.GetMarkerNumber(nodePositionMarkerName[ctr1])
                markerNumberOfPositionMarkerR = mbs.GetMarkerNumber(nodePositionMarkerName[ctr1+numberOfNodesX])
        
                ctr1 = ctr1 + 1
        
                springDamperElementName = "spring damper" + str(elementCtr)
             
                objectDict = {"objectType": "ConnectorSpringDamper",
                              "markerNumbers": [markerNumberOfPositionMarkerL, markerNumberOfPositionMarkerR],
                              "stiffness": elementStiffness,
                              "damping": elementDamping,
                              "force": 0,
                              "referenceLength": springDamperElementLength,
                              "activeConnector": activateElement,
                              "name": springDamperElementName,
                              "VdrawSize": 0.0}
            
                mbs.AddObject(objectDict)
        
                elementCtr = elementCtr + 1
    
    
    # spring-damper elements in off-diagnoal-direction
    if useSpringDamperElementsOffDiagonal == True:
        
        ctr1 = 0
        for i in range(numberOfNodesY-1): #range(numberOfNodes):
        
            for j in range(numberOfNodesX-1):
        
                markerNumberOfPositionMarkerL = mbs.GetMarkerNumber(nodePositionMarkerName[ctr1])
                markerNumberOfPositionMarkerR = mbs.GetMarkerNumber(nodePositionMarkerName[ctr1+numberOfNodesX+1])
        
                ctr1 = ctr1 + 1
        
                springDamperElementName = "spring damper" + str(elementCtr)
             
                objectDict = {"objectType": "ConnectorSpringDamper",
                              "markerNumbers": [markerNumberOfPositionMarkerL, markerNumberOfPositionMarkerR],
                              "stiffness": elementStiffness,
                              "damping": elementDamping,
                              "force": 0,
                              "referenceLength": springDamperDiagnoalElementLength,
                              "activeConnector": activateElement,
                              "name": springDamperElementName,
                              "VdrawSize": 0.0}
            
                mbs.AddObject(objectDict)
        
                elementCtr = elementCtr + 1
        
            ctr1 = ctr1 + 1
    
    
    # spring-damper elements in diagnoal-direction
    if useSpringDamperElementsDiagonal == True:
        
        ctr1 = 0
        for i in range(numberOfNodesY-1): 
        
            for j in range(numberOfNodesX-1):
        
                markerNumberOfPositionMarkerL = mbs.GetMarkerNumber(nodePositionMarkerName[ctr1+1])
                markerNumberOfPositionMarkerR = mbs.GetMarkerNumber(nodePositionMarkerName[ctr1+numberOfNodesX])
        
                ctr1 = ctr1 + 1
        
                springDamperElementName = "spring damper" + str(elementCtr)
             
                objectDict = {"objectType": "ConnectorSpringDamper",
                              "markerNumbers": [markerNumberOfPositionMarkerL, markerNumberOfPositionMarkerR],
                              "stiffness": elementStiffness,
                              "damping": elementDamping,
                              "force": 0,
                              "referenceLength": springDamperDiagnoalElementLength,
                              "activeConnector": activateElement,
                              "name": springDamperElementName,
                              "VdrawSize": 0.0}
            
                mbs.AddObject(objectDict)
        
                elementCtr = elementCtr + 1
        
            ctr1 = ctr1 + 1
    

        
#######################  assemble and solve mbs ###############################
    mbs.Assemble()
    #print(mbs)
    
    
    SC.renderer.Start()
    mbs.SolveDynamic(simulationSettings, solverType =  exudyn.DynamicSolverType.ODE23)
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!


