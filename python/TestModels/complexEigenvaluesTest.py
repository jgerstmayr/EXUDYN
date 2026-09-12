#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test for computation of complex eigenvalues with ODE2 equations + algebraic joint constraints
#
# Author:   Michael Pieber, Johannes Gerstmayr
# Date:     2024-05-03
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
import numpy as np
import sys

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

u = 0
#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
SC = exu.SystemContainer()
mbs = SC.AddSystem()

L=0.5
mass = 1.6          #mass in kg
spring = 4000       #stiffness of spring-damper in N/m
damper = 8          #damping constant in N/(m/s)

omega0 = np.sqrt(spring/mass)     #eigen frequency of undamped system
dRel = damper/(2*np.sqrt(spring*mass)) #dimensionless damping
omega = omega0*np.sqrt(1-dRel**2) #eigen frequency of damped system
#note: (without offset!!!)
#decay = q_k+1 / q_k = np.exp(-2*pi/np.sqrt(1/dRel**2-1))
#D = 1/np.sqrt(4*pi**2/(np.log(decay)**2)+1)

exu.Print('dRel =',dRel, ', decay=',np.exp(-2*pi/np.sqrt(1/dRel**2-1)))

u0=-0.08            #initial displacement
v0=1                #initial velocity
f =80               #force on mass
x0=f/spring         #static displacement

exu.Print('resonance frequency = '+str(np.sqrt(spring/mass)))
exu.Print('static displacement = '+str(x0))

#node for 3D mass point:
n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0], 
                     initialCoordinates = [u0,0,0], 
                     initialVelocities= [v0,0,0]))

#ground node
nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))

#add mass point (this is a 3D object with 3 coordinates):
massPoint = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = n1))

#marker for ground (=fixed):
groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))

#marker for node coordinates to be used with spring-damper and constraint:
nodeMarker0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 0))
nodeMarker1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 1))
nodeMarker2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 2))

#spring-damper between two marker coordinates
oCSD = mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundMarker, nodeMarker0], 
                                          stiffness = spring, damping = damper)) 

if True: #False: 3 eigen values, True: only one eigenvalue
    oCC1 = mbs.AddObject(CoordinateConstraint(markerNumbers=[groundMarker, nodeMarker1]))
    oCC2 = mbs.AddObject(CoordinateConstraint(markerNumbers=[groundMarker, nodeMarker2]))

#add load:
mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker0, 
                                         load = f))

#add sensor:
sForce = mbs.AddSensor(SensorObject(objectNumber=oCSD, storeInternal=True,
                           outputVariableType=exu.OutputVariableType.Force))
sPos = mbs.AddSensor(SensorNode(nodeNumber=n1, storeInternal=True,
                           outputVariableType=exu.OutputVariableType.Position))

mbs.Assemble()

tEnd = 1     #end time of simulation
h = 0.001    #step size; leads to 1000 steps

simulationSettings = exu.SimulationSettings()
simulationSettings.solutionSettings.solutionWritePeriod = 5e-3 #output interval general
simulationSettings.solutionSettings.sensorsWritePeriod = 5e-3  #output interval of sensors
simulationSettings.timeIntegration.numberOfSteps = tEnd/h
simulationSettings.timeIntegration.endTime = tEnd

simulationSettings.timeIntegration.verboseMode = 1             #show some solver output
# simulationSettings.displayComputationTime = True               #show how fast

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1

#add some drawing parameters for this example
SC.visualizationSettings.nodes.drawNodesAsPoint=False
SC.visualizationSettings.nodes.defaultSize=0.1


#start solver:

if useGraphics:
    mbs.SolveDynamic(simulationSettings)
    mbs.PlotSensor(sPos, closeAll=True, title='linear mass-spring-damper')

[eigenValues, eVectors] = mbs.ComputeODE2Eigenvalues(
                                                  computeComplexEigenvalues=True,
                                                  useAbsoluteValues=False)
eigenFreqHz = abs(eigenValues[0].imag) / (2*np.pi)
eigenDRel = abs(eigenValues[0].real)
freqAnalytic = omega0*(-dRel+sqrt(1-dRel**2)*1j)

exu.Print('MSD analytical eigenvalues:',freqAnalytic)
exu.Print('MSD complex eigenvalues:',eigenValues)
exu.Print('MSD numerical eigenfrequency in Hz:',eigenFreqHz)
exu.Print('numerical damping dRel           :',eigenDRel/abs(eigenValues[0]))
exu.Print('*********************\n')

u += eigenFreqHz-omega0/(2*pi) + eigenDRel/abs(eigenValues[0])
# sys.exit()

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#rotating rigid body:
SC = exudyn.SystemContainer()
mbs = SC.AddSystem()

beamL=0.1 #in m
beamW=0.01
beamH=0.001
rho=5000 #kg/m**3

dRel = 0.0001
springL = 0.02 #in m
springK = 10   #in N/m

oGround = mbs.AddObject(ObjectGround())

inertiaCuboid=InertiaCuboid(density=rho,
                        sideLengths=[beamL,beamH,beamW])
thetaZZ=inertiaCuboid.Translated([-beamL/2,0,0]).Inertia()[2,2]

#damping:
springD = dRel * (2*np.sqrt(springK*beamL**2/thetaZZ)) #undamped!

bBeam = mbs.CreateRigidBody(inertia = inertiaCuboid,
                        referencePosition = [beamL*0.5,0,0],
                        #gravity = [0,-9.81*0,0],
                        graphicsDataList = [graphics.Brick(size=[beamL,beamH,beamW],
                        color=graphics.color.orange)])
mBeamRight = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bBeam, localPosition=[beamL*0.5,0,0]))

mbs.CreateGenericJoint(bodyNumbers= [oGround,bBeam], position= [0.,0.,0.], 
                       rotationMatrixAxes= np.eye(3), constrainedAxes= [1,1,1,1,1,0], 
                       axesRadius=0.001, axesLength= 0.01, color= graphics.color.default)

markerToConnect = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[beamL,-springL,0])) 

mbs.AddObject(CartesianSpringDamper(markerNumbers=[markerToConnect,mBeamRight],
                                    stiffness=[0,springK,0],
                                    damping=[0,springD,0],
                                    offset=[0,springL*0.9,0], #this induces damped oscillations
                                    visualization=VObjectConnectorCartesianSpringDamper(show=True,drawSize=0.01)
                                    ))    
sPos = mbs.AddSensor(SensorBody(bodyNumber=bBeam, storeInternal=True, localPosition=[beamL*0.5,0,0],
                           outputVariableType=exu.OutputVariableType.Position))

mbs.Assemble()
[eigenValues, eVectors] = mbs.ComputeODE2Eigenvalues(computeComplexEigenvalues=True,
                                                  useAbsoluteValues=False)

evNumerical = np.abs(eigenValues[0]) / (2*np.pi)  #abs(omega0*(-dRel+sqrt(1-dRel**2)*1j) ) gives omega0!!!
evNumericalDampedImag = np.abs(eigenValues[0].imag) / (2*np.pi) #this gives the damped case!
evNumericalDampedReal = np.abs(eigenValues[0].real) #this gives the damped case!

#use generalized eigenvalue solver to compare with undamped case
[eigenValuesGE, eVectorsGE] = mbs.ComputeODE2Eigenvalues(computeComplexEigenvalues=False,
                                                  useAbsoluteValues=True)


evAnalytical = np.sqrt( springK*beamL**2/thetaZZ ) / (2*np.pi)

u += (evAnalytical-abs(evNumerical))/evAnalytical
exu.Print('numerical eigenfrequency (in Hz) :',evNumerical)
exu.Print('numerical eigenfrequency damped  :',evNumericalDampedImag)
exu.Print('numerical damping dRel           :',evNumericalDampedReal/(evNumerical*2*pi))
exu.Print('numerical eigenfrequency GE      :',np.sqrt(eigenValuesGE[0])/(2*pi))
exu.Print('analytical eigenfrequency (in Hz):',evAnalytical)
exu.Print('error eigenvalues:', u, '\n*********************\n')

#decay measured: 1.130mm vs 0.777mm => decay= 1.454
#      ==> dRel measured  = 0.0595
#      ==> dRel numerical = 0.05999850003749906

tEnd = 1.5     #end time of simulation
h = 0.002    #step size; leads to 1000 steps

simulationSettings = exu.SimulationSettings()
simulationSettings.solutionSettings.sensorsWritePeriod = h  #output interval of sensors
simulationSettings.timeIntegration.numberOfSteps = tEnd/h
simulationSettings.timeIntegration.endTime = tEnd

if useGraphics:
    mbs.SolveDynamic(simulationSettings)
    mbs.PlotSensor(sPos,components=[1],title='bar with spring at tip')

#sys.exit()

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#mechanism
SC = exudyn.SystemContainer()
mbs = SC.AddSystem()

beamL=0.1 #in m
beamW=0.01
beamH=0.001
rho=5000 #kg/m**3
springK=1e3 #in N/m

oGround = mbs.AddObject(ObjectGround())

inertiaCuboid=InertiaCuboid(density=rho,
                        sideLengths=[beamL,beamH,beamW])

p0 = np.array([beamL*0.5,0,0])
b0 = mbs.CreateRigidBody(inertia = inertiaCuboid,
                         referencePosition = p0,
                         gravity = [0,-9.81,0],
                         graphicsDataList = [graphics.Brick(size=[beamL,beamH,beamW],
                         color=graphics.color.orange)])

R1 = RotationMatrixZ(-0.25*pi)@RotationMatrixY(0.25*pi)
p1 = 2*p0 + R1@p0
b1 = mbs.CreateRigidBody(inertia = inertiaCuboid,
                         referencePosition = p1,
                         referenceRotationMatrix = R1,
                         gravity = [0,-9.81,0],
                         graphicsDataList = [graphics.Brick(size=[beamL,beamH,beamW],
                         color=graphics.color.dodgerblue)])

mbs.CreateGenericJoint(bodyNumbers= [oGround,b0], position= [0.,0.,0.], 
                       constrainedAxes= [1,1,1,1,1,0], 
                       axesRadius=beamH*2, axesLength=beamW*1.05)

mB0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=p0))
mB1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=-p0))

mbs.AddObject(GenericJoint(markerNumbers=[mB1,mB0], constrainedAxes=[1,1,1, 1,0,0],
                           rotationMarker0=np.eye(3),
                           rotationMarker1=np.eye(3),
                           # rotationMarker1=R1.T,
                           visualization=VGenericJoint(axesRadius=beamH*2, axesLength=beamW*1.05)))

mbs.CreateCartesianSpringDamper(bodyOrNodeList=[b1, oGround],
                                localPosition0=p0,
                                localPosition1=2*p0 + R1@(2*p0),
                                stiffness=[springK]*3,
                                damping=[springK*1e-4]*3,
                                drawSize = beamW
                                )
# mbs.CreateGenericJoint(bodyNumbers= [b0, b1], position= 2*p0,
#                        constrainedAxes= [1,1,1,1,0,0], 
#                        axesRadius=beamH, axesLength=beamW)

sPos = mbs.AddSensor(SensorBody(bodyNumber=b1, localPosition=p0,
                                storeInternal=True, 
                                outputVariableType=exu.OutputVariableType.Displacement
                                ) )

mbs.Assemble()
SC.visualizationSettings.loads.show=False
SC.visualizationSettings.openGL.multiSampling=4
simulationSettings = exu.SimulationSettings()
simulationSettings.solutionSettings.sensorsWritePeriod = 1e-3
simulationSettings.timeIntegration.numberOfSteps=1000

#useAbsoluteValues is used to compare with GE, thus having double eigen values
[eigenValues, eVectors] = mbs.ComputeODE2Eigenvalues(computeComplexEigenvalues=True,
                                                  useAbsoluteValues=True)

evNumerical = eigenValues / (2*np.pi)
exu.Print('numerical eigenvalues in Hz:',evNumerical)
#compute with generalized eigenvalue solver without damping:
[eigenValues, eVectors] = mbs.ComputeODE2Eigenvalues(computeComplexEigenvalues=False,
                                                  useAbsoluteValues=True)

evNumerical = np.sqrt(eigenValues) / (2*np.pi)
exu.Print('numerical eigenvalues GE:',evNumerical)

if useGraphics:
    mbs.SolveDynamic(simulationSettings=simulationSettings)
    mbs.PlotSensor(sPos)
    
u += evNumerical[0]/100
exu.Print('result of computeODE2AEeigenvaluesTest2:', u)

exudynTestGlobals.testError = u - 0.38811732950413347 #should be zero
exudynTestGlobals.testResult = u




