#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test model for GeometricallyExactBeam2D, evaluating a grid of beams
#
# Model:    Planar model of beams arranged at grid (horizontal and vertical lines), rigidly connected;
#           The grid has a size of 16 elements in x-direction and 4 elements in y-direction.
#
# Author:   Johannes Gerstmayr
# Date:     2021-03-25
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# *clean example*
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

## import libraries
import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

import numpy as np
from math import sin, cos, pi

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

## set up system and define parameters
SC = exu.SystemContainer()
mbs = SC.AddSystem()

lElem = 0.5            # length of one finite element 
lElemY = lElem*1
E=2.1e11*0.1               # Steel; Young's modulus of beam element in N/m^2
rho=7800               # Steel; density of beam element in kg/m^3
b=0.1                  # width of rectangular beam element in m
h=0.1                 # height of rectangular beam element in m
A=b*h                  # cross sectional area of beam element in m^2
I=b*h**3/12            # second moment of area of beam element in m^4
nu = 0.3               # Poisson's ratio for steel
    
EI = E*I
EA = E*A
rhoA = rho*A
rhoI = rho*I
ks = 10*(1+nu)/(12+11*nu) # shear correction factor
G = E/(2*(1+nu))          # shear modulus
GA = ks*G*A               # shear stiffness of beam
    
nX = 16             #grid size x
nY = 4              #grid size y

## create nodes
nodeIndices = np.zeros((nX, nY), dtype=int)

for j in range(nY):
    for i in range(nX):
        pRef = [i*lElem,j*lElemY, 0] #zero angle; reference rotation not used! 
        
        ni=mbs.AddNode(NodeRigidBody2D(referenceCoordinates = pRef, 
                                       initialCoordinates = [0,0,0], 
                                       initialVelocities= [0,0,0]))
        nodeIndices[i,j]=int(ni)

## create elements in x-direction
for j in range(nY):
    for i in range(nX-1):
        n0 = nodeIndices[i, j]
        n1 = nodeIndices[i+1, j]
        oGeneric = mbs.AddObject(ObjectBeamGeometricallyExact2D(nodeNumbers = [n0,n1], 
                                                                physicsLength=lElem,
                                                                physicsMassPerLength=rhoA,
                                                                physicsCrossSectionInertia=rhoI,
                                                                physicsBendingStiffness=EI,
                                                                physicsAxialStiffness=EA,
                                                                physicsShearStiffness=GA,
                                                                includeReferenceRotations=False,
                                                                visualization=VObjectBeamGeometricallyExact2D(drawHeight = h)
                                                    ))

## create elements in y-direction
for j in range(nY-1):
    for i in range(int(nX/4)):
        n0 = nodeIndices[(i+1)*4-1, j]
        n1 = nodeIndices[(i+1)*4-1, j+1]
        mbs.AddObject(ObjectBeamGeometricallyExact2D(nodeNumbers = [n0,n1], 
                                                     physicsLength=lElemY,
                                                     physicsMassPerLength=rhoA,
                                                     physicsCrossSectionInertia=rhoI,
                                                     physicsBendingStiffness=EI,
                                                     physicsAxialStiffness=EA,
                                                     physicsShearStiffness=GA,
                                                     includeReferenceRotations=False,
                                                     visualization=VObjectBeamGeometricallyExact2D(drawHeight = h) ))
        
    

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
## create ground node and marker coordinate attached to ground node
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))    
mNCground = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))

## add markers and constraints for fixed supports
for j in range(nY):
    n0 = nodeIndices[0,j]
    mC0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=0))
    mC1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=1))
    mC2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=2))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mNCground, mC0]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mNCground, mC1]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mNCground, mC2]))

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #add tip load
    tipNodeMarker = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nodeIndices[-1,j]))
    mbs.AddLoad(Force(markerNumber = tipNodeMarker, loadVector = [0, -1e5, 0]))
    

## assemble    
mbs.Assemble()

## set up simulation settings
simulationSettings = exu.SimulationSettings()
    
tEnd = 0.1
steps = 100
simulationSettings.timeIntegration.numberOfSteps = steps
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.solutionSettings.writeSolutionToFile = False
#simulationSettings.timeIntegration.simulateInRealtime = True
#simulationSettings.timeIntegration.realtimeFactor = 0.1

#simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
    
simulationSettings.timeIntegration.newton.useModifiedNewton = True


simulationSettings.staticSolver.newton.maxIterations = 50
simulationSettings.staticSolver.numberOfLoadSteps = 10
# simulationSettings.displayComputationTime = True
# simulationSettings.displayStatistics = True


SC.visualizationSettings.nodes.defaultSize = 0.005
# SC.visualizationSettings.bodies.beams.crossSectionFilled = False
SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.ForceLocal    
SC.visualizationSettings.contour.outputVariableComponent = 0

## start graphics
if useGraphics:
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()

## start dynamic solver
mbs.SolveDynamic(simulationSettings)

## stop graphics        
if useGraphics:
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

## read and print solution
uLast = mbs.GetNodeOutput(nodeIndices[-1,-1], exu.OutputVariableType.Coordinates)
exu.Print("grid =",nodeIndices.shape,", uTip =", uLast[0:2])

exu.Print('solution of gridGeomExactBeam2D=',uLast[1]) #use y-coordinate

exudynTestGlobals.testError = uLast[1] - (-2.2115028353806547) 
exudynTestGlobals.testResult = uLast[1]



