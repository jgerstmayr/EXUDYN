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

import sys
sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
from exudyn.FEM import *
from exudyn.graphicsDataUtilities import *

from modelUnitTests import ExudynTestStructure, exudynTestGlobals
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()


L = 0.8 #length of arm
mass = 2.5
g = 9.81

r = 0.05 #just for graphics
d = r/2

#add ground object and mass point:
graphicsBackground = GraphicsDataRectangle(-1.2*L,-1.2*L, 1.2*L, 0.2*L, [1,1,1,1]) #for appropriate zoom
oGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0], 
                           visualization = VObjectGround(graphicsData = [graphicsBackground])))


graphicsSphere = GraphicsDataSphere(point=[L/2,0,0], radius=r, color=[1.,0.2,0.2,1], nTiles = 16)
graphicsSphere2 = GraphicsDataSphere(point=[0,0,0], radius=r, color=color4steelblue, nTiles = 16)
graphicsLink = GraphicsDataOrthoCube(-L/2,-d/2,-d/2, L/2,d/2, d/2, [0.5,0.5,0.5,0.5])

inertia = InertiaCuboid(density=mass/(L*d*d), sideLengths=[L,d,d])
exu.Print("mass=",inertia.mass)

nR0 = mbs.AddNode(Rigid2D(referenceCoordinates=[L/2,0,0])) #body goes from [0,0,0] to [L,0,0]
oR0 = mbs.AddObject(RigidBody2D(nodeNumber=nR0, physicsMass = inertia.mass, physicsInertia=inertia.inertiaTensor[2][2], 
                                  visualization = VObjectRigidBody2D(graphicsData = [graphicsLink,graphicsSphere])))
mGround0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [0,0,0]))
mR0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oR0, localPosition=[-L/2,0,0]))
mTip0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oR0, localPosition=[L/2,0,0]))
mNodeR0 = mbs.AddMarker(MarkerNodePosition(nodeNumber=nR0))
oRJ0 = mbs.AddObject(RevoluteJoint2D(markerNumbers=[mGround0,mR0]))
#
mbs.AddLoad(Force(markerNumber = mNodeR0, loadVector = [0, -mass*g, 0])) 

zeroZoneFriction = 1e-3 #zero-zone for velocity in friction
fFriction = 1          #friction force (norm); acts against velocity
#user function for friction against velocity vector, including zeroZone
def UserFunctionSpringDamper(mbs, t, u, v, k, d, offset):
    vNorm = NormL2(v)
    f=[v[0],v[1],v[2]]
    if abs(vNorm) < offset[0]:
        f = ScalarMult(offset[1]/offset[0], f)
    else:
        f = ScalarMult(offset[1]/vNorm, f)
    return f

mbs.AddObject(CartesianSpringDamper(markerNumbers=[mGround0, mTip0], 
                                    offset=[zeroZoneFriction, fFriction, 0], 
                                    springForceUserFunction=UserFunctionSpringDamper))

#mbs.AddSensor(SensorNode(nodeNumber = nR0, fileName='solution/pendulumFrictionRotation0.txt',
#                         outputVariableType=exu.OutputVariableType.Coordinates))
if exudynTestGlobals.useGraphics:
    mbs.AddSensor(SensorBody(bodyNumber = oR0, fileName='solution/pendulumFrictionRotation0.txt',
                             outputVariableType=exu.OutputVariableType.Rotation))


mbs.Assemble()

simulationSettings = exu.SimulationSettings()

f = 4000
simulationSettings.timeIntegration.numberOfSteps = int(1*f)
simulationSettings.timeIntegration.endTime = 0.0001*f
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/5000
simulationSettings.solutionSettings.sensorsWritePeriod = simulationSettings.timeIntegration.endTime/2000
#simulationSettings.displayComputationTime = True
simulationSettings.timeIntegration.verboseMode = 1

#simulationSettings.timeIntegration.newton.useModifiedNewton = False
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = simulationSettings.timeIntegration.generalizedAlpha.useNewmark
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.60 #0.62 is approx. the limit

simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True
simulationSettings.solutionSettings.coordinatesSolutionFileName= "coordinatesSolution.txt"

#simulationSettings.displayStatistics = True

SC.visualizationSettings.nodes.defaultSize = 0.05

if exudynTestGlobals.useGraphics:
    exu.StartRenderer()
    mbs.WaitForUserToContinue()

exu.SolveDynamic(mbs, simulationSettings)

p0=mbs.GetObjectOutputBody(oR0, exu.OutputVariableType.Position, localPosition=[0,0,0])
exu.Print("p0=", p0)
u=NormL2(p0)
exu.Print('solution of pendulumFriction=',u)

exudynTestGlobals.testError = u - (0.3999999877698205) #2020-04-22: 0.3999999877698205


if exudynTestGlobals.useGraphics:
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!

    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    
    data = np.loadtxt('solution/pendulumFrictionRotation0.txt', comments='#', delimiter=',')
    plt.plot(data[:,0], data[:,1], 'b-', label='rotation 0') #ccordinate 3 = rotation
    
    ax=plt.gca() # get current axes
    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
    plt.xlabel("time (s)")
    plt.ylabel("angle (rad)")
    plt.tight_layout() #better arrangement of plot
    plt.legend()
    plt.show() 
    

