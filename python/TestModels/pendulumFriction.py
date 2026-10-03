#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Mathematical pendulum with friction;
#           Remark: uses two friction models: CoordinateSpringDamper and CartesianSpringDamper
#
# Author:   Johannes Gerstmayr
# Date:     2019-12-26
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics
from exudyn.FEM import *

import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()


L = 0.8 #length of arm
mass = 2.5
g = 9.81

r = 0.05 #just for graphics
d = r/2

#add ground object and mass point:
graphicsBackground = graphics.Lines([[-1.2*L,-1.2*L,0], [1.2*L,-1.2*L,0], [1.2*L,0.2*L,0], [-1.2*L,0.2*L,0], [-1.2*L,-1.2*L,0]], color=[1,1,1,1]) #for appropriate zoom
oGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0], 
                           visualization = VObjectGround(graphicsData = [graphicsBackground])))


graphicsSphere = graphics.Sphere(point=[L/2,0,0], radius=r, color=[1.,0.2,0.2,1], nTiles = 16)
graphicsSphere2 = graphics.Sphere(point=[0,0,0], radius=r, color=graphics.color.steelblue, nTiles = 16)
graphicsLink = graphics.BrickXYZ(-L/2,-d/2,-d/2, L/2,d/2, d/2, [0.5,0.5,0.5,0.5])

inertia = InertiaCuboid(density=mass/(L*d*d), sideLengths=[L,d,d])
exu.Print("mass=",inertia.mass)

nR0 = mbs.AddNode(Rigid2D(referenceCoordinates=[L/2,0,0])) #body goes from [0,0,0] to [L,0,0]
oR0 = mbs.AddObject(RigidBody2D(nodeNumber=nR0, physicsMass = inertia.mass, physicsInertia=inertia.inertiaTensor[2][2], 
                                  visualization = VObjectRigidBody2D(graphicsData = [graphicsLink,graphicsSphere])))

#markers:
mGround0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [0,0,0]))
mR0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oR0, localPosition=[-L/2,0,0]))
mTip0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oR0, localPosition=[L/2,0,0]))
mNodeR0 = mbs.AddMarker(MarkerNodePosition(nodeNumber=nR0))
mR0COM = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oR0, localPosition=[0,0,0]))


oRJ0 = mbs.AddObject(RevoluteJoint2D(markerNumbers=[mGround0,mR0]))
#
mbs.AddLoad(Force(markerNumber = mNodeR0, loadVector = [0, -mass*g, 0])) 

zeroZoneFriction = 1e-3 #zero-zone for velocity in friction
fFriction = 1          #friction force (norm); acts against velocity
#user function for friction against velocity vector, including zeroZone
def UserFunctionSpringDamper(mbs, t, itemIndex, u, v, k, d, offset):
    vNorm = np.linalg.norm(v)
    f=[v[0],v[1],v[2]]
    if abs(vNorm) < offset[0]:
        f = ((offset[1]/offset[0])*np.array(f))
    else:
        f = ((offset[1]/vNorm)*np.array(f))
    return f

mbs.AddObject(CartesianSpringDamper(markerNumbers=[mGround0, mTip0], 
                                    offset=[zeroZoneFriction, fFriction, 0], 
                                    springForceUserFunction=UserFunctionSpringDamper))

if not testIsActive:
    sRot1 = mbs.AddSensor(SensorBody(bodyNumber = oR0, fileName='solution/pendulumFrictionRotation0.txt',
                             outputVariableType=exu.OutputVariableType.Rotation))

    sRot2 = mbs.AddSensor(SensorMarker(markerNumber = mR0COM, fileName='solution/pendulumFrictionRotation0marker.txt',
                               writeToFile = (not testIsActive),
                               outputVariableType=exu.OutputVariableType.Rotation))

sPos = mbs.AddSensor(SensorMarker(markerNumber = mR0COM, writeToFile = False,
                           outputVariableType=exu.OutputVariableType.Position))

mbs.Assemble()

simulationSettings = exu.SimulationSettings()

f = 4000
simulationSettings.timeIntegration.numberOfSteps = int(1*f)
simulationSettings.timeIntegration.endTime = 0.0001*f
simulationSettings.solution.file.writePeriod = simulationSettings.timeIntegration.endTime/5000
simulationSettings.solution.sensors.writePeriod = simulationSettings.timeIntegration.endTime/2000
#simulationSettings.show.computationTime = True
simulationSettings.timeIntegration.verboseMode = 1

#simulationSettings.timeIntegration.newton.useModifiedNewton = False
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = simulationSettings.timeIntegration.generalizedAlpha.useNewmark
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.60 #0.62 is approx. the limit

simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True
simulationSettings.solution.file.name= "solution/coordinatesSolution.txt"
simulationSettings.solution.file.write=False

#simulationSettings.show.statistics = True

SC.visualizationSettings.nodes.defaultSize = 0.05

if not testIsActive:
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings)

p0=mbs.GetObjectOutputBody(oR0, exu.OutputVariableType.Position, localPosition=[0,0,0])
exu.Print("p0=", p0)

p0 = mbs.GetSensorValues(sPos) #obtain values from marker
exu.Print("p0=", p0, '(marker)')
u=np.linalg.norm(p0)
exu.Print('solution of pendulumFriction=',u)

exu.sys['testResult'] = u


if not testIsActive:
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

    
    
    mbs.PlotSensor([sRot1, sRot2], components=[0,2], closeAll=True, markerStyles=['x','+'])

    # import matplotlib.pyplot as plt
    # import matplotlib.ticker as ticker
    
    # data = np.loadtxt('solution/pendulumFrictionRotation0.txt', comments='#', delimiter=',')
    # plt.plot(data[:,0], data[:,1], 'b-', label='rotation 0') #ccordinate 1 = rotation, scalar for ObjectRigidBody2D
    # data = np.loadtxt('solution/pendulumFrictionRotation0marker.txt', comments='#', delimiter=',')
    # plt.plot(data[:,0], data[:,3], 'r-', label='rotation 0') #ccordinate 3 = rotation, Z-coordinate because marker always 3D
    
    # ax=plt.gca() # get current axes
    # ax.grid(True, 'major', 'both')
    # ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
    # ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
    # plt.xlabel("time (s)")
    # plt.ylabel("angle (rad)")
    # plt.tight_layout() #better arrangement of plot
    # plt.legend()
    # plt.show() 
    

