#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  simple ANCF test
#
# Author:   Johannes Gerstmayr
# Date:     2022-03-04
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

import numpy as np
from math import sqrt, sin, cos

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

#%%++++++++++++++++++++++++++++++++++++++++
useGraphics = True
plotResults=False

tEnd = 0.5
h= 1e-3*0.1

SC = exu.SystemContainer()
mbs = SC.AddSystem()

damper = 0.01 #0.1: standard for parameter variation; 0.001: almost no damping, but solution is still oscillating at evaluation period

L=1.        #length of ANCF element in m    
rhoA=10     #beam + discrete masses
hBeam = 0.1

EA=1e5
EI=10*0.25
nElements = 8
lElem = L/nElements

# #additional bending and axial damping
bendingDamping=0.1*EI # for ALE Element
axialDamping=0 # for ALE Element

#generate coordinate marker
#nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
#mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create one beam template
cable = Cable2D(#physicsLength=L, 
                physicsMassPerLength=rhoA, 
                physicsBendingStiffness=EI, 
                physicsAxialStiffness=EA, 
                physicsBendingDamping=bendingDamping, 
                physicsAxialDamping=axialDamping, 
                # physicsUseCouplingTerms = True,
                # useReducedOrderIntegration = True, #faster
                visualization=VCable2D(drawHeight=hBeam)
                )

#alternative to mbs.AddObject(ALECable2D(...)) with nodes:
yOff = 0.5*hBeam
ancf=GenerateStraightLineANCFCable2D(mbs=mbs,
                positionOfNode0=[0,yOff,0], positionOfNode1=[L+0*sqrt(0.5),yOff+0*L*sqrt(0.5),0],
                numberOfElements=nElements,
                cableTemplate=cable, #this defines the beam element properties
                massProportionalLoad = [0,-9.81,0], #add larger gravity for larger deformation
                #fixedConstraintsNode0 = [1,1,1*0,1*0], #fixed
                #fixedConstraintsNode1 = [0,0,0,0]) #fixed
                )

ancfNodes = ancf[0]
ancfObjects = ancf[1]

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                visualization=VObjectGround(graphicsData=[graphics.CheckerBoard(size=2)])))
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))

mLeft = mbs.AddMarker(MarkerBodyPosition(bodyNumber=ancfObjects[0], localPosition=[0,-0.5*hBeam,0]))
mbs.AddObject(RevoluteJoint2D(markerNumbers=[mGround, mLeft] ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#sensorFileName = 'solution/beamTip.txt'
sTipNode = mbs.AddSensor(SensorNode(nodeNumber=ancfNodes[-1], storeInternal=True,
                            outputVariableType=exu.OutputVariableType.Position))
sPos = mbs.AddSensor(SensorBody(bodyNumber=ancfObjects[-1], storeInternal=True, localPosition=[lElem,-0.5*hBeam,0.],
                                outputVariableType=exu.OutputVariableType.Position))
sVel = mbs.AddSensor(SensorBody(bodyNumber=ancfObjects[-1], storeInternal=True, localPosition=[lElem,-0.5*hBeam,0.],
                                outputVariableType=exu.OutputVariableType.Velocity))
sAcc = mbs.AddSensor(SensorBody(bodyNumber=ancfObjects[-1], storeInternal=True, localPosition=[lElem,-0.5*hBeam,0.],
                                outputVariableType=exu.OutputVariableType.Acceleration))
sRot = mbs.AddSensor(SensorBody(bodyNumber=ancfObjects[-1], storeInternal=True, localPosition=[lElem,0.,0.],
                                outputVariableType=exu.OutputVariableType.Rotation))
sAngVel = mbs.AddSensor(SensorBody(bodyNumber=ancfObjects[-1], storeInternal=True, localPosition=[lElem,0.,0.],
                                outputVariableType=exu.OutputVariableType.AngularVelocity))
sAngAcc = mbs.AddSensor(SensorBody(bodyNumber=ancfObjects[-1], storeInternal=True, localPosition=[lElem,0.,0.],
                                outputVariableType=exu.OutputVariableType.AngularAcceleration))


mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.solutionSettings.sensorsWritePeriod = h
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
simulationSettings.timeIntegration.adaptiveStep = True #disable adaptive step reduction

simulationSettings.displayStatistics = True
SC.visualizationSettings.loads.show = False
SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.StrainLocal
#SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.CurvatureLocal
#SC.visualizationSettings.bodies.beams.axialTiling = 500
#SC.visualizationSettings.bodies.beams.crossSectionTiling = 8

if useGraphics:
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()

success = mbs.SolveDynamic(simulationSettings, 
                           exudyn.DynamicSolverType.TrapezoidalIndex2)

if useGraphics:
    SC.renderer.DoIdleTasks()
    #SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!        


#%%++++++++++++++++++
    if True:
        import matplotlib.pyplot as plt
        
        from exudyn.signalProcessing import FilterSensorOutput

        mbs.PlotSensor(sensorNumbers=[sRot, sAngVel, sAngAcc], components=[0,2,2], 
                   title='ang vel', closeAll=True,
                   markerStyles=['','x ','o '], lineStyles=['-','',''])
        dataRot = mbs.GetSensorStoredData(sRot)
        dataRotDiff = FilterSensorOutput(dataRot, filterWindow=5, polyOrder=3, derivative=1)
        plt.plot(dataRotDiff[:,0], dataRotDiff[:,1], 'c-', label='diff rot')

        dataAngVel = mbs.GetSensorStoredData(sAngVel)
        dataAngAcc = mbs.GetSensorStoredData(sAngAcc)
        dataAngVelDiff = FilterSensorOutput(dataRot, filterWindow=5, polyOrder=3, derivative=2)
        plt.plot(dataAngVelDiff[:,0], dataAngVelDiff[:,1], 'k-', label='diffdiff rot')
        dataAngVelDiff2 = FilterSensorOutput(dataAngVel, filterWindow=7, polyOrder=5, derivative=1)
        plt.plot(dataAngVelDiff2[:,0], dataAngVelDiff2[:,3], 'm-', label='diff ang vel')
        plt.legend()

        #+++++++++++++++++++
        mbs.PlotSensor(sensorNumbers=[sVel, sAcc], components=[1,1], 
                   title='velocities', 
                   markerStyles=['x ','o '], lineStyles=['',''])
        dataPos = mbs.GetSensorStoredData(sPos)
        dataPosDiff = FilterSensorOutput(dataPos, filterWindow=5, polyOrder=3, derivative=1)
        plt.plot(dataPosDiff[:,0], dataPosDiff[:,2], 'c-', label='diff pos')

        dataVel = mbs.GetSensorStoredData(sVel)
        dataAcc = mbs.GetSensorStoredData(sAcc)
        dataVelDiff = FilterSensorOutput(dataPos, filterWindow=5, polyOrder=3, derivative=2)
        plt.plot(dataVelDiff[:,0], dataVelDiff[:,2], 'k-', label='diffdiff pos')
        dataVelDiff2 = FilterSensorOutput(dataVel, filterWindow=7, polyOrder=5, derivative=1)
        plt.plot(dataVelDiff2[:,0], dataVelDiff2[:,2], 'm-', label='diff vel')
        plt.legend()

        exu.Print('angVel  = ', dataAngVel[-1,3])
        exu.Print('rotDiff = ', dataRotDiff[-1,1])

        exu.Print('angAcc     = ', dataAngAcc[-1,3])
        exu.Print('angVelDiff = ', dataAngVelDiff[-1,1])

        exu.Print('Vel     = ', dataVel[-1,2])
        exu.Print('rotDiff = ', dataPosDiff[-1,2])

        exu.Print('Acc     = ', dataAcc[-1,2])
        exu.Print('velDiff = ', dataVelDiff[-1,2])


