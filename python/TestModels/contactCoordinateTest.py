#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test with ObjectContactCoordinate, which can be used to achieve accurate contact simulation
#           Uses step reduction to resolve contact switching point
#           Similar to postNewtonStepContactTest but with stiffer contact
#
# Author:   Johannes Gerstmayr
# Date:     2021-08-12
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

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

withUserFunction = False #compare to user function based test

#define parameters of mass point
L=0.5
r = 0.05
g=9.81
mass = 0.25         #mass in kg
spring = 20000*100        #stiffness of spring-damper in N/m
damper = 0.0001*spring          #damping constant in N/(m/s)
load0 = -mass*g     #in negative y-direction

doRefSol = False
tEnd = 0.25        #end time of simulation
h = 2e-3*0.1
if doRefSol:
    h=1e-5

#data coordinate: contains gap for ObjectContactCoordinate, for user function: 1=no contact, 0=contact
nData=mbs.AddNode(NodeGenericData(initialCoordinates=[1], numberOfDataCoordinates=1))

#node for 3D mass point:
n1=mbs.AddNode(Point(referenceCoordinates = [0,0,0],
                     initialCoordinates = [0,r+(L*0+0.05),0],
                     initialVelocities = [0,-0.1*0,0]))

#user function for spring force
def springForce(mbs, t, itemIndex, u, v, k, d, offset, mu, muPropZone):
    p = 0*L+u-r
    data = mbs.systemData.GetDataCoordinates()
    #exu.Print("p=", p, ", contact=", data[0])
    if data[0] == 0:
        return k*p + d*v
    else:
        return 0

def PostNewtonUserFunction(mbs, t):
    p0 = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position, configuration=exu.ConfigurationType.StartOfStep)[1] - r
    p = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)[1] - r
    #v0 = mbs.GetNodeOutput(n1, exu.OutputVariableType.Velocity, configuration=exu.ConfigurationType.StartOfStep)[1]
    #a0 = mbs.GetNodeOutput(n1, exu.OutputVariableType.Acceleration, configuration=exu.ConfigurationType.StartOfStep)[1]
    h = mbs.sys['dynamicSolver'].it.currentStepSize #grab current step size from dynamic solver
    data = mbs.systemData.GetDataCoordinates()
    data0 = mbs.systemData.GetDataCoordinates(configuration=exu.ConfigurationType.StartOfStep)

    #data[0] = 0 #no contact; 0 corresponds to the only one data coordinate in the system, attributed to contact
    recommendedStepSize = -1
    error = 0
    #check if previous assumption was wrong ==> set error, reduce step size and set new contact state
    if p < 0:
        if data0[0] == 1:
            error = abs(p)

            if (p0 > 0):
                recommendedStepSize = h*(abs(p0))/(abs(p0)+abs(p))
            else:
                recommendedStepSize = 0.25*h #simple alternative

            data[0] = 0 #contact

    else:
        if data0[0] == 0:
            error = abs(p)
            #recommendedStepSize = 1e-6 #simple alternative
            if (p0 > 0):
                recommendedStepSize = h*(abs(p0))/(abs(p0)+abs(p))
            else:
                recommendedStepSize = 0.25*h #simple alternative
            data[0] = 1 #contact off

    mbs.systemData.SetDataCoordinates(data)
    return [error,recommendedStepSize]

sMode = ""
if withUserFunction:
    mbs.SetPostNewtonUserFunction(PostNewtonUserFunction)
    sMode = "User"

#ground node
d=0.01
gGround = graphics.Brick([0,-d*0.5,0],[2*L,d,d],color=graphics.color.grey)
oGround=mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[gGround])))

nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))

#add mass point (this is a 3D object with 3 coordinates):
gSphere = graphics.Sphere([0,0,0], r, color=graphics.color.red, nTiles=20)
massPoint = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = n1,
                                    visualization=VMassPoint(graphicsData=[gSphere])))

#marker for ground (=fixed):
groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
#marker for springDamper for first (x-)coordinate:
nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 1)) #y-coordinate

#Spring-Damper between two marker coordinates
if withUserFunction:
    sensorFileName='solution/sensorPos'+sMode+'.txt'
    mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundMarker, nodeMarker], 
                                         stiffness = spring, damping = damper, 
                                         springForceUserFunction = springForce,
                                         visualization=VCoordinateSpringDamper(show=False))) 
else:
    sensorFileName=''
    mbs.AddObject(ObjectContactCoordinate(markerNumbers = [groundMarker, nodeMarker], 
                                          nodeNumber = nData,
                                          contactStiffness = spring, contactDamping = damper, 
                                          offset = r,
                                          visualization=VObjectContactCoordinate(show=False))) 

#add load:
loadC = mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker, 
                           load = load0))


if useGraphics:
    sPos = mbs.AddSensor(SensorNode(nodeNumber=n1, outputVariableType=exu.OutputVariableType.Position, 
                                    storeInternal=True,fileName=sensorFileName
                                    ))

mbs.Assemble()

simulationSettings = exu.SimulationSettings()
simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.solutionSettings.sensorsWritePeriod = 1e-10
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.minimumStepSize = 1e-10
simulationSettings.timeIntegration.stepInformation = 3 #do not show step increase

#important settings for contact:
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.newton.useModifiedNewton = False #True=does not work yet
simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-8 #this is the accepted penetration before reducing step size
if not withUserFunction:
    simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-3 #this is the accepted contact force error before reducing step size

simulationSettings.timeIntegration.discontinuous.maxIterations = 2 #1=immediately perform step reduction
simulationSettings.timeIntegration.discontinuous.ignoreMaxIterations = False #repeat step in case of failure
simulationSettings.timeIntegration.adaptiveStepRecoverySteps = 0 #number of steps to wait until step size is increased again
simulationSettings.timeIntegration.adaptiveStepIncrease = 10    #after successful step, increase again rapidly

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #for index 3 solver, this would be the best case

simulationSettings.displayStatistics = True
#simulationSettings.timeIntegration.simulateInRealtime = True

if useGraphics:
    SC.renderer.Start()              #start graphics visualization
    SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue

#start solver:
mbs.SolveDynamic(solverType=exu.DynamicSolverType.TrapezoidalIndex2, simulationSettings=simulationSettings) #chose index2, can handle adaptive steps
#mbs.SolveDynamic(solverType=exu.DynamicSolverType.RK67, simulationSettings=simulationSettings)

if useGraphics:
    SC.renderer.Stop()               #safely close rendering window!

u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
exu.Print('contactCoordinateTest=',u[1])

exudynTestGlobals.testError = u[1] - (0.055313199503736685) #2021-08-13: 0.055313199503736685 (may change significantly for other disc. solver strategies)
exudynTestGlobals.testResult = u[1]

#+++++++++++++++++++++++++++++++++++++++++++++++++++++

if useGraphics and True: #to run this, run model first with withUserFunction=True
    
    mbs.PlotSensor(sensorNumbers=[sPos, 'solution/sensorPosUser.txt'], components=1, 
               labels=['internal contact','user function'])

















