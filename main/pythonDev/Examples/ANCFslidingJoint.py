#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  3D ANCF Cable element with sliding joint test
#
# Author:   Johannes Gerstmayr
# Date:     2026-03-02
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import ObjectANCFCable, VObjectANCFCable, InertiaCuboid, MarkerBodyRigid,\
                        NodeGenericData, ObjectJointSliding, MarkerBodyBeamShape
import exudyn.graphics as graphics #only import if it does not conflict
from exudyn.beams import GenerateStraightLineANCFCable

import numpy as np

#create an environment for mini example
SC = exu.SystemContainer()
mbs = SC.AddSystem()

L = 2
hg = 1 #ground offset y
rCable = 0.01
oGround=mbs.CreateGround(referencePosition= [0,0,0],
                         graphicsDataList=[graphics.CheckerBoard([0.5*L,-hg,0], normal=[0,1,0],size=3),
                                           graphics.Cylinder([0,0,0],[0,-hg,0],radius=rCable*2, color=graphics.color.orange),
                                           graphics.Cylinder([L,0,0],[0,-hg,0],radius=rCable*2, color=graphics.color.orange)
                                           ])

rhoA = 7800*rCable**2*np.pi
EA = 0.25*100000.
EI = 1

nCables = 1
for i in range(nCables):
    p0 = np.array([0,0,i*0.1])
    p1 = p0 + [L,0,0]

    cable = ObjectANCFCable(physicsMassPerLength=rhoA, 
                  physicsBendingStiffness = EI, 
                  physicsBendingDamping = EI*0.02,
                  physicsAxialStiffness=EA,
                  physicsAxialDamping=EA*0.02,
                  visualization=VObjectANCFCable(radius = rCable),
                  )

    ancf=GenerateStraightLineANCFCable(mbs=mbs,
                  positionOfNode0=p0, positionOfNode1=p1,
                  numberOfElements=48, #converged to 4 digits
                  cableTemplate=cable, #this defines the beam element properties
                  massProportionalLoad = [0,-9.81,0],
                  fixedConstraintsNode0 = [1,1,1, 0,1,1], #add constraints for pos and rot (r'_y,r'_z)
                  fixedConstraintsNode1 = [1,1,1, 0,1,1], #add constraints for pos and rot (r'_y,r'_z)
                  )
    #ancf=[cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList]

lElem = mbs.GetObject(ancf[1][0])['physicsLength']

slidingCoordinateInit = 0.1*L
initialLocalMarker = int(slidingCoordinateInit/lElem) #second element

hy = 0.4 #height of rigid body
sz = 5*rCable #z-displacement of rigid body
cubeLengths = [5*rCable,hy-2*rCable,10*rCable]
oRigid = mbs.CreateRigidBody(referencePosition=[slidingCoordinateInit, -0.5*hy,-sz],
                    inertia=InertiaCuboid(2*7800, cubeLengths),
                    gravity=[0,-9.81,0],
                    graphicsDataList=[graphics.Brick(size=cubeLengths, color=graphics.color.dodgerblue),
                                      graphics.Sphere([0,0.5*hy,sz], radius=2*rCable, color=graphics.color.red)]
                    )
mRigidTop = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRigid, localPosition=[0,0.5*hy,sz]))

addSlidingJoint = True
if addSlidingJoint:


    cableMarkerList = []#list of MarkerBodyBeamShape
    offsetList = []     #list of offsets counted from first cable element; needed in sliding joint
    offset = 0          #first cable element has offset 0
    for item in ancf[1]: #create markers for cable elements
        m = mbs.AddMarker(MarkerBodyBeamShape(bodyNumber = item))
        cableMarkerList += [m]
        offsetList += [offset]
        offset += lElem

    nodeDataSJ = mbs.AddNode(NodeGenericData(initialCoordinates=[initialLocalMarker,slidingCoordinateInit],numberOfDataCoordinates=2)) #initial index in cable list
    slidingJoint = mbs.AddObject(ObjectJointSliding(markerNumbers=[mRigidTop,cableMarkerList[initialLocalMarker]], 
                                                    constrainRotations=[0,0,0],
                                                    slidingMarkerNumbers=cableMarkerList, slidingMarkerOffsets=offsetList, 
                                                    nodeNumber=nodeDataSJ))


#assemble and solve system for default parameters
mbs.Assemble()

endTime=10
stepSize = 0.5e-3

simulationSettings = exu.SimulationSettings()

#simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.solutionSettings.solutionWritePeriod = 0.02 #data not used
simulationSettings.solutionSettings.binarySolutionFile = True
simulationSettings.solutionSettings.outputPrecision = 6 #float
simulationSettings.solutionSettings.sensorsWritePeriod = 0.002 #data not used
simulationSettings.timeIntegration.verboseMode = 1 #turn off, because of lots of output
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
simulationSettings.parallel.numberOfThreads = 1
simulationSettings.displayComputationTime = True
simulationSettings.displayStatistics = True

simulationSettings.timeIntegration.numberOfSteps = int(endTime/stepSize)
simulationSettings.timeIntegration.endTime = endTime
simulationSettings.timeIntegration.newton.useModifiedNewton = True

#simulationSettings.timeIntegration.simulateInRealtime = True
#simulationSettings.timeIntegration.realtimeFactor = 0.5

SC.visualizationSettings.openGL.multiSampling = 4

SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
SC.visualizationSettings.view0.window.renderWindowSize=[1200,1024]
SC.visualizationSettings.view0.camera.perspective = 0.5
SC.visualizationSettings.openGL.light0.shadow = 0.3
SC.visualizationSettings.openGL.light0.position = [2,10,2,1]
SC.visualizationSettings.nodes.show = False
SC.visualizationSettings.loads.show = False
SC.visualizationSettings.connectors.show = False
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++

SC.renderer.Start()
SC.renderer.SetModelView(zoom=1.335652,rotationVector=[0.7933899,0.5903195,0.2917689],centerPoint=[1.012,-0.349,0])
SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings)

SC.renderer.DoIdleTasks()
SC.renderer.Stop() #safely close rendering window!

mbs.SolutionViewer()
