#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test model and example for exudyn.plot.PlotSensor, using several sensors and plotting results
#
# Author:   Johannes Gerstmayr
# Date:     2021-07-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

from math import sin, cos, pi
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

SC = exu.SystemContainer()
mbs = SC.AddSystem()
# useGraphics=False

#background
color = [0.1,0.1,0.8,1]
L = 0.4 #length of bodies
d = 0.1 #diameter of bodies

#create background, in order to have according zoom all
zz=2*L
#background0 = GraphicsDataRectangle(-zz,-zz,zz,zz,graphics.color.white)
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0])) #,visualization=VObjectGround(graphicsData= [background0])))
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oGround, localPosition=[0,0,0]))

p0 = [0.,0.,0] #reference position
vLoc = np.array([L,0,0]) #last to next joint

g = [0,-9.81*0,0]

inertia = InertiaCuboid(density=1000, sideLengths=[L,d,d])
#p0 += Alist[i] @ (0.5*vLoc)
p0 += (0.5*vLoc)

ep0 = eulerParameters0 #no rotation
graphicsBody = graphics.Brick([0,0,0], [L,d,d], graphics.color.steelblue)
oRB = mbs.CreateRigidBody(inertia=inertia, 
                          referencePosition=p0,
                          gravity=g,
                          graphicsDataList=[graphicsBody])
nRB= mbs.GetObject(oRB)['nodeNumber']

mPos0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB, localPosition = [-0.5*L,0,0]))
mPosLast = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB, localPosition = [0.5*L,0,0]))

oRJ = mbs.AddObject(ObjectJointRevoluteZ(markerNumbers = [mGround, mPos0], 
                                  visualization=VObjectJointRevoluteZ(axisRadius=0.5*d, axisLength=1.2*d) )) 
    
def UFload(mbs,t,load):
    return [0,load[1]*np.cos(t*2*np.pi),0]

lForce = mbs.AddLoad(LoadForceVector(markerNumber=mPosLast, loadVector=[0,2,0], loadVectorUserFunction=UFload))

filedir = 'solution/'
#these test are really written to files (most other tests use internal storage):
sLoad = mbs.AddSensor(SensorLoad(loadNumber=lForce, fileName=filedir+'plotSensorLoad.txt'))
sNode = mbs.AddSensor(SensorNode(nodeNumber=nRB, fileName=filedir+'plotSensorNode.txt', outputVariableType=exu.OutputVariableType.Coordinates))
sNode2 = mbs.AddSensor(SensorNode(nodeNumber=nRB, fileName=filedir+'plotSensorNodeRotation.txt', outputVariableType=exu.OutputVariableType.Rotation))
sBody = mbs.AddSensor(SensorBody(bodyNumber=oRB, fileName=filedir+'plotSensorBody.txt', localPosition=[0.5*L,0,0], outputVariableType=exu.OutputVariableType.Position))
sObject = mbs.AddSensor(SensorObject(objectNumber=oRJ, fileName=filedir+'plotSensorObject.txt', outputVariableType=exu.OutputVariableType.ForceLocal))
sMarker = mbs.AddSensor(SensorMarker(markerNumber=mPosLast, fileName=filedir+'plotSensorMarker.txt', outputVariableType=exu.OutputVariableType.Position))

mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 1
h=0.01  #use small step size to detext contact switching

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.solutionWritePeriod = 0.01
simulationSettings.solutionSettings.sensorsWritePeriod = 0.01
simulationSettings.timeIntegration.verboseMode = 1
# simulationSettings.timeIntegration.simulateInRealtime = True

# simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
# simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
simulationSettings.timeIntegration.newton.useModifiedNewton = True

# SC.visualizationSettings.nodes.show = True
# SC.visualizationSettings.nodes.drawNodesAsPoint  = False
# SC.visualizationSettings.nodes.showBasis = True
# SC.visualizationSettings.nodes.basisSize = 0.015
# SC.visualizationSettings.connectors.showJointAxes = True
 
SC.visualizationSettings.general.autoFitScene = False #use loaded render state
#useGraphics = False
if useGraphics:
    SC.renderer.Start()
    if 'renderState' in exu.sys:
        SC.renderer.SetState(exu.sys[ 'renderState' ])
    #SC.renderer.DoIdleTasks()
else:
    simulationSettings.solutionSettings.writeSolutionToFile = False

#mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)
mbs.SolveDynamic(simulationSettings, showHints=True)

#%%+++++++++++++++++++++++++++++
if useGraphics:
    #SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!


exudynTestGlobals.testError = 0
exudynTestGlobals.testResult = 1

import matplotlib.pyplot as plt


closeAll = not useGraphics
mbs.PlotSensor(sensorNumbers=sLoad, components=[0,1,2], closeAll=closeAll)
mbs.PlotSensor(sensorNumbers=sNode, components=[0,1,2,3,4,5,6], 
           yLabel='Coordinates with offset 1\nand scaled with $\\frac{1}{1000}$', 
           factors=1e-3, offsets=1,fontSize=12, closeAll=closeAll)
mbs.PlotSensor(sensorNumbers=sNode2, components=[0,1,2], closeAll=closeAll)
mbs.PlotSensor(sensorNumbers=sNode2, components=[0,1,2], closeAll=closeAll)
mbs.PlotSensor(sensorNumbers=[sBody]*3+[sMarker]*3, components=[0,1,2,0,1,2], 
           colorCodeOffset=3, newFigure=closeAll, fontSize=10, 
           yLabel='Rotation $\\alpha, \\beta, \\gamma$ and\n Position $x,y,z$', closeAll=closeAll)
mbs.PlotSensor(sensorNumbers=sObject, components=[0,1,2], title='Revolute joint forces', closeAll=closeAll)
mbs.PlotSensor(sensorNumbers=[sNode]*3+ [filedir+'plotSensorNode.txt']*3, components=[0,1,2]*2, closeAll=closeAll)

if closeAll:
    plt.close('all')

import os
for s in range(mbs.systemData.NumberOfSensors()):
    fileName=mbs.GetSensor(s)['fileName']
    exu.Print('remove file:', fileName)
    os.remove(fileName)

