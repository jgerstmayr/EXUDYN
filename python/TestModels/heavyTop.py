#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Heavy top example
#           Refs.:  Terze, Z., Müller, A., Zlatar, D.: Singularity-free time integration of rotational quaternions using non-redundant ordinary differential equations. Multibody System Dynamics 38(3),201–225 (2016)
#
# Author:   Johannes Gerstmayr
# Date:     2020-02-02
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

#exu.Print('EXUDYN version='+exu.config.Version())

#background
#rect = [-0.1,-0.1,0.1,0.1] #xmin,ymin,xmax,ymax
#background0 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
color = [0.1,0.1,0.8,1]
r = 0.5 #radius
L = 1   #length


background0 = GraphicsDataRectangle(-L,-L,L,L,color)
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background0])))

#heavy top is fixed at [0,0,0] (COM of simulated body), but force is applied at [0,1,0] (COM of real top)
m = 15
Jxx=0.234375
Jyy=0.46875
Jzz=0.234375
#yS = 1 #distance from 

#vector to COM, where force is applied
rp = [0.,1.,0.]
#rpt = np.array(Skew(rp))
rpt = Skew(rp)
Fg = [0,0,-m*9.81]
#inertia tensor w.r.t. fixed point
JFP = np.diag([Jxx,Jyy,Jzz]) - m*np.dot(rpt,rpt)
#exu.Print(JFP)

omega0 = [0,150,-4.61538] #arbitrary initial angular velocity
p0 = [0,0,0] #reference position
v0 = [0.,0.,0.] #initial translational velocity

nodeTypeList = [exu.NodeType.RotationEulerParameters, exu.NodeType.RotationRxyz]

sAngVel=[]
sPos=[]
sCoords=[]
for nodeType in nodeTypeList:
    
    nRB = 0
    if nodeType == exu.NodeType.RotationEulerParameters:
        ep0 = eulerParameters0 #no rotation
        ep_t0 = AngularVelocity2EulerParameters_t(omega0, ep0)
        #exu.Print(ep_t0)
    
        nRB = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=p0+ep0, initialVelocities=v0+list(ep_t0)))
    else: #Rxyz
        rot0 = [0,0,0] #no rotation
        #omega0 = [10,0,0]
        rot_t0 = AngularVelocity2RotXYZ_t(omega0, rot0)
        #exu.Print('rot_t0=',rot_t0)
    
        nRB = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=p0+rot0, initialVelocities=v0+list(rot_t0)))
    
    oGraphics = graphics.BrickXYZ(-r/2,-L/2,-r/2, r/2,L/2,r/2, [0.1,0.1,0.8,1])
    oRB = mbs.AddObject(ObjectRigidBody(physicsMass=m, physicsInertia=[JFP[0][0], JFP[1][1], JFP[2][2], JFP[1][2], JFP[0][2], JFP[0][1]], 
                                        nodeNumber=nRB, visualization=VObjectRigidBody(graphicsData=[oGraphics])))
    
    mMassRB = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oRB, localPosition=[0,1,0])) #this is the real COM
    mbs.AddLoad(Force(markerNumber = mMassRB, loadVector=Fg)) 
    
    nPG=mbs.AddNode(PointGround(referenceCoordinates=[0,0,0])) #for coordinate constraint
    mCground = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nPG, coordinate=0)) #coordinate number does not matter
    
    mC0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRB, coordinate=0)) #ux
    mC1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRB, coordinate=1)) #uy
    mC2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRB, coordinate=2)) #uz
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mCground, mC0]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mCground, mC1]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mCground, mC2]))
    
    if useGraphics:
        sAdd = ''
        if nodeType == exu.NodeType.RotationRxyz:
            sAdd = 'Rxyz' #avoid that both sensor file names are identical
        #mbs.AddSensor(SensorNode(nodeNumber=nRB, storeInternal=True,#fileName='solution/sensorRotation'+sAdd+'.txt', outputVariableType=exu.OutputVariableType.Rotation))
        sAngVel+=[mbs.AddSensor(SensorNode(nodeNumber=nRB, storeInternal=True, #fileName='solution/sensorAngVelLocal'+sAdd+'.txt', 
                                           outputVariableType=exu.OutputVariableType.AngularVelocityLocal))]
        #mbs.AddSensor(SensorNode(nodeNumber=nRB, fileName='solution/sensorAngVel'+sAdd+'.txt', outputVariableType=exu.OutputVariableType.AngularVelocity))
        
        sPos+=[mbs.AddSensor(SensorBody(bodyNumber=oRB, storeInternal=True, #fileName='solution/sensorPosition'+sAdd+'.txt', 
                                        localPosition=rp, outputVariableType=exu.OutputVariableType.Position))]
        sCoords+=[mbs.AddSensor(SensorNode(nodeNumber=nRB, storeInternal=True, #fileName='solution/sensorCoordinates'+sAdd+'.txt', 
                                           outputVariableType=exu.OutputVariableType.Coordinates))]

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
mbs.Assemble()
#exu.Print(mbs)

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

fact = 2000
simulationSettings.timeIntegration.numberOfSteps = 1*fact
simulationSettings.timeIntegration.endTime = 0.0001*fact
#simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/fact
simulationSettings.solutionSettings.sensorsWritePeriod = simulationSettings.timeIntegration.endTime/fact

simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
#simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 

if useGraphics:
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings)

if useGraphics:
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

solTotal = mbs.systemData.GetODE2CoordinatesTotal()
u = 0
for i in range(4):
    u += abs(solTotal[3+i]); #Euler parameters
for i in range(3):
    u += abs(solTotal[7+3+i]); #Euler angles Rxyz

exu.Print('sol=',solTotal)

exu.Print('solution of heavy top =',u)
# EP ref solution MATLAB: at t=0.2
#  gen alpha (sigma=0.98, h=1e-4): -0.70813,0.43881,0.54593,0.089251 ==> abs sum=1.782121
#  RK4:                            -0.70828,0.43878,0.54573,0.0894   ==> abs sum=1.78219
#Exudyn: (index2)                  -1.70824157  0.43878143  0.54578152   0.08937154

#RotXYZ solution EXUDYN:           29.86975964,-0.7683481513,-1.002841906

exudynTestGlobals.testError = u - (33.42312575174431) #2020-02-04 added RigidRxyz: (33.423125751773306) 2020-02-03: (1.7821760506326125)
exudynTestGlobals.testResult = u



#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#compute exact solution:

if useGraphics:
    
    fileRef = '../../../docs/verification/HeavyTopSolution/HeavyTop_TimeEulerParameter_RK4.txt'
    mbs.PlotSensor(sCoords[0], components=[3,4,5,6], labels=['theta 0','theta 1','theta 2','theta 3'], 
               closeAll=True, offsets=[1.,0,0,0], yLabel='Euler parameters') #offsets for reference coords
    mbs.PlotSensor(fileRef, components=[0,1,2,3], labels=['theta 0 ref','theta 1 ref','theta 2 ref','theta 3 ref'], 
               colorCodeOffset=7, newFigure=False)
    
    mbs.PlotSensor(sAngVel[0], components=[0,1,2], labels=['omega X','omega Y','omega Z'])
    mbs.PlotSensor(sPos[0], components=[0,1,2])







