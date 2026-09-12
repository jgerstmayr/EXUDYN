#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Example of double pendulum with Mass points: CoordinateVectorConstraint and GenericODE2
#
# Author:   Johannes Gerstmayr
# Date:     2022-03-17
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
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

withUserFunction = True

L = 0.8 #length of arm
mass = 2.5
g = 9.81

r = 0.05 #just for graphics
d = r/2

#add ground object and mass point:
sizeRect = 1.2*L*2
#graphicsBackground = GraphicsDataRectangle(-sizeRect,-sizeRect, sizeRect, 0.2*L, [1,1,1,1]) #for appropriate zoom
graphicsBackground = graphics.CheckerBoard(point=[0,-0.5*sizeRect,-2*r],size=sizeRect*1.8)

oGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0], 
                           visualization = VObjectGround(graphicsData = [graphicsBackground])))


graphicsSphere = graphics.Sphere(point=[0,0,0], radius=r, color=graphics.color.steelblue, nTiles = 16)

nR0 = mbs.AddNode(Point2D(referenceCoordinates=[L,0]))

mGround0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [0,0,0]))
mTip0 = mbs.AddMarker(MarkerNodePosition(nodeNumber=nR0))

if not withUserFunction: #with internal terms:
    oCD0 = mbs.AddObject(DistanceConstraint(markerNumbers=[mGround0, mTip0], distance=L))
else:
    #just for drawing, with inactive connector:
    oCD0 = mbs.AddObject(DistanceConstraint(markerNumbers=[mGround0, mTip0], distance=L, activeConnector=False))

#
mbs.AddLoad(Force(markerNumber = mTip0, loadVector = [0, -mass*g, 0])) 

fileNameDouble = 'solution/coordVecConstraintRefDouble.txt'
    
sPos0 = mbs.AddSensor(SensorNode(nodeNumber = nR0, storeInternal = True,
                                 outputVariableType=exu.OutputVariableType.Position))


graphicsSphere = graphics.Sphere(point=[0,0,0], radius=r, color=graphics.color.red, nTiles = 16)
nR1 = mbs.AddNode(Point2D(referenceCoordinates=[L*2,0]))

#instead of MassPoint2D, create one object ...
oGeneric = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[nR0, nR1], 
                                massMatrix=mass*np.eye(4)))

mTip1 = mbs.AddMarker(MarkerNodePosition(nodeNumber=nR1))

if not withUserFunction: #with internal terms:
    oCD1 = mbs.AddObject(DistanceConstraint(markerNumbers=[mTip0, mTip1], distance=L))
else:
    #just for drawing, with inactive connector:
    mbs.AddObject(DistanceConstraint(markerNumbers=[mTip0, mTip1], distance=L, activeConnector=False))

    nGround = mbs.AddNode(NodePointGround())
    mCoordsGround = mbs.AddMarker(MarkerNodeCoordinates(nodeNumber=nGround))

    mCoords0 = mbs.AddMarker(MarkerNodeCoordinates(nodeNumber=nR0))
    mCoords1 = mbs.AddMarker(MarkerNodeCoordinates(nodeNumber=nR1))

    mCoordsAll = mbs.AddMarker(MarkerObjectODE2Coordinates(objectNumber=oGeneric))
 
    def UFconstraint(mbs, t, itemNumber, q, q_t,velocityLevel):
        #print("q=", q, ", q_t=", q_t)
        return [np.sqrt(q[0]**2 + q[1]**2) - L,
                np.sqrt((q[2]-q[0])**2 + (q[3]-q[1])**2) - L]

    def UFjacobian(mbs, t, itemNumber, q, q_t,velocityLevel):
        #print("q=", q, ", q_t=", q_t)
        jac  = np.zeros((2,4))

        f0 = np.sqrt(q[0]**2 + q[1]**2)
        jac[0,0] = q[0]/f0
        jac[0,1] = q[1]/f0

        f1 = np.sqrt((q[2]-q[0])**2 + (q[3]-q[1])**2)
        jac[1,0] =-(q[2]-q[0])/f1
        jac[1,1] =-(q[3]-q[1])/f1
        jac[1,2] = (q[2]-q[0])/f1
        jac[1,3] = (q[3]-q[1])/f1
        return jac

        
    mbs.AddObject(CoordinateVectorConstraint(markerNumbers=[mCoordsGround, mCoordsAll],
                                             #markerNumbers=[mCoords0, mCoords1], #ALTERNATIVELY: with markers on nodes (but only works for max. 2 nodes!)
                                             scalingMarker0=np.zeros((2,4)), #needed to define number of algebraic equations; rows=nAE, cols=len(q) of mCoordsGround + mCoords0
                                             constraintUserFunction=UFconstraint,
                                             jacobianUserFunction=UFjacobian,
                                             visualization=VCoordinateVectorConstraint(show=False)))

 
#q
mbs.AddLoad(Force(markerNumber = mTip1, loadVector = [0, -mass*g, 0])) 

sPos1 = mbs.AddSensor(SensorNode(nodeNumber = nR1, storeInternal = True,
                                 #fileName=fileNameDouble,
                                 outputVariableType=exu.OutputVariableType.Position))



mbs.Assemble()

simulationSettings = exu.SimulationSettings()

#useGraphics=False
tEnd = 1
h = 1e-3
if useGraphics:
    tEnd = 1
    simulationSettings.timeIntegration.simulateInRealtime = True
    simulationSettings.timeIntegration.realtimeFactor = 1
    
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd

#simulationSettings.solutionSettings.solutionWritePeriod = h
simulationSettings.timeIntegration.verboseMode = 1
#simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8 #SHOULD work with 0.9 as well

SC.visualizationSettings.nodes.showBasis=True
SC.visualizationSettings.nodes.drawNodesAsPoint=False
SC.visualizationSettings.nodes.defaultSize=r

if useGraphics:
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings)

p0=mbs.GetNodeOutput(nR0, exu.OutputVariableType.Position)
exu.Print("p0=", list(p0))
u=sum(p0)

exu.Print('solution of coordinateVectorConstraint=',u)

exudynTestGlobals.testError = u - (-1.0825265797698322)
exudynTestGlobals.testResult = u


#%%++++++++++++++++++++++++++++
if useGraphics:
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

    from exudyn.plot import PlotSensorDefaults
    PlotSensorDefaults().fontSize = 12
    # PlotSensorDefaults().markerStyles=['x','o ','v ','^ ','s ']
    # mbs.PlotSensor([sPos0,sPos0,sPos1,sPos1], components=[0,1,0,1], closeAll=True)

    #if reference solution computed:
    mbs.PlotSensor([sPos0,sPos0,sPos1,sPos1,fileNameDouble], components=[0,1,0,1,1], closeAll=True, 
               markerStyles=['','','','','x'], lineStyles=['-','-','-','-',''])



