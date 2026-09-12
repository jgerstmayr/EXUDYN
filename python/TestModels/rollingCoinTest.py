#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Rolling coin example; 
#           examine example of Rill, Schaeffer, Grundlagen und Methodik der Mehrkörpersimulation, 2010, page 59
#           Note that in comparison to the literature, we use the local x-axis for the local axis of the coin, z is the normal to the plane
#           mass and inertia do not influence the results, as long as mass and inertia of a infinitely small ring are used
#           gravity is set to [0,0,-9.81m/s^2] and the radius is 0.01m
#
# Author:   Johannes Gerstmayr
# Date:     2020-06-19
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

phi0 = 1./180.*np.pi#initial nick angle of disc, 1 degree
g = [0,0,-9.81]     #gravity in m/s^2
m = 1               #mass in kg
r = 0.01            #radius of disc in m
w = 0.001           #width of disc in m, just for drawing
p0 = [r*np.sin(phi0),0,r*np.cos(phi0)] #origin of disc center point at reference, such that initial contact point is at [0,0,0]
initialRotation = RotationMatrixY(phi0)

omega0 = [0,0,1800/180*np.pi]                   #initial angular velocity around z-axis
v0 = Skew(omega0) @ initialRotation @ [0,0,r]   #initial angular velocity of center point
#v0 = [0,0,0]                                   #initial translational velocity
#print("v0=",v0)#," = ", [0,10*np.pi*r*np.sin(phi0),0])

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
gGround = graphics.Brick(centerPoint=[0,0,-0.001],size=[0.12,0.12,0.002], color=graphics.color.lightgrey)
oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[gGround])))
markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))

#markers for rigid body:
markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0,0,0]))

#rolling disc:
oRolling=mbs.AddObject(ObjectJointRollingDisc(markerNumbers=[markerGround, markerBody0J0], 
                                              constrainedAxes=[1,1,1], discRadius=r,
                                              visualization=VObjectJointRollingDisc(discWidth=w,color=graphics.color.blue)))

sForce=mbs.AddSensor(SensorObject(objectNumber=oRolling, storeInternal=True,#fileName='solution/rollingDiscTrail.txt', 
                                  outputVariableType = exu.OutputVariableType.ForceLocal))



#sensor for trace of contact point:
if useGraphics:
    sTrail=mbs.AddSensor(SensorObject(objectNumber=oRolling, storeInternal=True,#fileName='solution/rollingDiscTrail.txt', 
                               outputVariableType = exu.OutputVariableType.Position))
    
    sVel=mbs.AddSensor(SensorObject(objectNumber=oRolling, storeInternal=True,#fileName='solution/rollingDiscTrailVel.txt', 
                               outputVariableType = exu.OutputVariableType.Velocity))
    


mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 0.5
if useGraphics:
    tEnd = 0.5

h=0.0005 #no visual differences for step sizes smaller than 0.0005

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
#simulationSettings.solutionSettings.solutionWritePeriod = 0.01
simulationSettings.solutionSettings.sensorsWritePeriod = 0.0005
simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.timeIntegration.verboseMode = 1
# simulationSettings.displayStatistics = True

simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True


SC.visualizationSettings.nodes.show = True
SC.visualizationSettings.nodes.drawNodesAsPoint  = False
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.nodes.basisSize = 0.015

if useGraphics:
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings)

p0=mbs.GetObjectOutput(oRolling, exu.OutputVariableType.Position)
force=mbs.GetSensorValues(sForce)
exu.Print('force in rollingCoinTest=',force) #use x-coordinate

u = p0[0] + 0.1*(force[0]+force[1]+force[2])
exu.Print('solution of rollingCoinTest=',u) #use x-coordinate

exudynTestGlobals.testError = u - (1.0634381189385853) #2024-04-29: added force #2020-06-20: 0.002004099927340136; 2020-06-19: 0.002004099760845168 #4s looks visually similar to Rill, but not exactly ...
exudynTestGlobals.testResult = u


if useGraphics:
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

    ##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
    #plot results
    if True:
        
        
        mbs.PlotSensor(sTrail, componentsX=[0],components=[1], closeAll=True, title='wheel trail')


        # import matplotlib.pyplot as plt
        # import matplotlib.ticker as ticker

        # if True:
        #     data = np.loadtxt('solution/rollingDiscTrail.txt', comments='#', delimiter=',') 
        #     plt.plot(data[:,1], data[:,2], 'r-',label='contact point trail') #x/y coordinates of trail
        # else:
        #     #show trail velocity computed numerically and from sensor:
        #     data = np.loadtxt('solution/rollingDiscTrail.txt', comments='#', delimiter=',') 
    
        #     nData = len(data)
        #     vVec = np.zeros((nData,2))
        #     dt = data[1,0]-data[0,0]
        #     for i in range(nData-1):
        #         vVec[i+1,0:2] = 1/dt*(data[i+1,1:3]-data[i,1:3])
    
        #     plt.plot(data[:,0], vVec[:,0], 'r-',label='contact point vel x') 
        #     plt.plot(data[:,0], vVec[:,1], 'k-',label='contact point vel y') 
        #     plt.plot(data[:,0], (vVec[:,0]**2+vVec[:,1]**2)**0.5, 'g-',label='|contact point vel|')
    
        #     trailVel = np.loadtxt('solution/rollingDiscTrailVel.txt', comments='#', delimiter=',') 
        #     plt.plot(data[:,0], trailVel[:,1], 'r--',label='trail vel x')
        #     plt.plot(data[:,0], trailVel[:,2], 'k--',label='trail vel y')
        #     plt.plot(data[:,0], trailVel[:,3], 'y--',label='trail vel z')
        #     plt.plot(data[:,0], (trailVel[:,1]**2+trailVel[:,2]**2)**0.5, 'b--',label='|trail vel|')

        # ax=plt.gca() # get current axes
        # ax.grid(True, 'major', 'both')
        # ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
        # ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
        # plt.tight_layout()
        # plt.legend()
        # plt.show() 
    

