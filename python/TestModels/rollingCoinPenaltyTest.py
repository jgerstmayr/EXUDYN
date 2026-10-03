#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Rolling coin example; 
#           examine example of Rill, Schaeffer, Grundlagen und Methodik der Mehrkörpersimulation, 2010, page 59
#           Note that in comparison to the literature, we use the local x-axis for the local axis of the coin, z is the normal to the plane
#           mass and inertia do not influence the results, as long as mass and inertia of a infinitely small ring are used
#           gravity is set to [0,0,-9.81m/s^2] and the radius is 0.01m;
#           In this example, the penalty formulation is used, which additionally treats friction
#
# Author:   Johannes Gerstmayr
# Date:     2020-06-19
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics

import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()

phi0 = 5./180.*np.pi#initial nick angle of disc, 1 degree
g = [0,0,-9.81]     #gravity in m/s^2
m = 1               #mass in kg
r = 0.01            #radius of disc in m
w = 0.001           #width of disc in m, just for drawing
p0 = [r*np.sin(phi0),0,r*np.cos(phi0)+0.01] #origin of disc center point at reference, such that initial contact point is at [0,0,0]
initialRotation = RotationMatrixY(phi0)

omega0 = [40,0,0*1800/180*np.pi]                   #initial angular velocity around z-axis
v0 = Skew(omega0) @ initialRotation @ [0,0,r]   #initial angular velocity of center point
#v0 = [0,0,0]                                   #initial translational velocity
#exu.Print("v0=",v0)#," = ", [0,10*np.pi*r*np.sin(phi0),0])

#inertia for infinitely small ring:
inertiaRing = RigidBodyInertia(mass=1, inertiaTensor= np.diag([0.5*m*r**2, 0.25*m*r**2, 0.25*m*r**2]))
#print(inertiaRing)

#additional graphics for visualization of rotation:
graphicsBody = graphics.Brick(centerPoint=[0,0,0],size=[w*1.1,0.7*r,0.7*r], color=graphics.color.lightred)

dict0 = mbs.CreateRigidBody(referencePosition=p0,  
                            referenceRotationMatrix=initialRotation,  
                            initialVelocity=v0,  
                            initialAngularVelocity=omega0,  
                            inertia=inertiaRing,  
                            gravity=g,  
                            graphicsDataList=[graphicsBody],  
                            returnDict=True)  
[n0, b0] = [dict0['nodeNumber'], dict0['bodyNumber']]

#ground body and marker
gGround = graphics.Brick(centerPoint=[0,0,-0.001],size=[0.3,0.3,0.002], color=graphics.color.lightgrey)
oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[gGround])))
markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))

#markers for rigid body:
markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0,0,0]))

#rolling disc:
nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
oRolling=mbs.AddObject(ObjectConnectorRollingDiscPenalty(markerNumbers=[markerGround, markerBody0J0], nodeNumber = nGeneric,
                                              discRadius=r, dryFriction=[0.8,0.8], dryFrictionProportionalZone=1e-2, 
                                              rollingViscousFriction=0.2,
                                              contactStiffness=1e5, contactDamping=1e4,
                                              visualization=VObjectConnectorRollingDiscPenalty(discWidth=w, color=graphics.color.blue)))


#sensor for trace of contact point:
if not testIsActive:
    sTrail=mbs.AddSensor(SensorObject(objectNumber=oRolling, storeInternal=True,#fileName='solution/rollingDiscTrail.txt', 
                               outputVariableType = exu.OutputVariableType.Position))
    
    sTrailVel=mbs.AddSensor(SensorObject(objectNumber=oRolling, storeInternal=True,#fileName='solution/rollingDiscTrailVel.txt', 
                               outputVariableType = exu.OutputVariableType.Velocity))
    

    sAngVel=mbs.AddSensor(SensorBody(bodyNumber=b0, storeInternal=True,#fileName='solution/rollingDiscAngVel.txt', 
                               outputVariableType = exu.OutputVariableType.AngularVelocity))
    
    sPos=mbs.AddSensor(SensorBody(bodyNumber=b0, storeInternal=True,#fileName='solution/rollingDiscPos.txt', 
                               outputVariableType = exu.OutputVariableType.Position))
    
    sForce=mbs.AddSensor(SensorObject(objectNumber=oRolling, storeInternal=True,#fileName='solution/rollingDiscForceLocal.txt', 
                               outputVariableType = exu.OutputVariableType.ForceLocal))

mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 0.5
if not testIsActive:
    tEnd = 0.5

h=0.0001 

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
#simulationSettings.solution.file.writePeriod = 0.01
simulationSettings.solution.sensors.writePeriod = 0.0005
#simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.solution.file.write = False

simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True


SC.visualizationSettings.nodes.show = True
SC.visualizationSettings.nodes.drawNodesAsPoint  = False
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.nodes.basisSize = 0.015

if not testIsActive:
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings)

p0=mbs.GetObjectOutput(oRolling, exu.OutputVariableType.Position)
exu.Print('solution of rollingCoinPenaltyTest=',p0[0]) #use x-coordinate

exu.sys['testResult'] = p0[0]


if not testIsActive:
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

    ##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
    #plot results
    if True:
        
        
        mbs.PlotSensor(sTrail, componentsX=[0],components=[1], closeAll=True, title='wheel trail')

        mbs.PlotSensor(sPos, components=[0,1,2], title='wheel position')
        mbs.PlotSensor(sForce, components=[0,1,2], title='wheel force')

        mbs.PlotSensor(sAngVel, components=[0], title='wheel local angular velocity')

