#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test with ObjectContactConvexRoll, which models a roll of a mechanum wheel or any other roll
#           which is described by a polynomial profile
#
# Author:   Peter Manzl
# Date:     2021-12-21
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


# create Ground and graphics: 
scb, g1, g2 = 0.5, 0.92, 0.72
graphGround = graphics.CheckerBoard(point= [0,0,0], normal= [0,0,1], size= scb, color= [g1, g1, g1, 1.0],
                                        alternatingColor= [g2, g2, g2, 1.0], nTiles= 12)
oGround = mbs.CreateGround(referencePosition = [0,0,0], 
                           graphicsDataList=[graphGround])
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround,localPosition = [0,0,0], name='Ground'))
nGround0=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))


poly = [-3.6, 0,  1.65e-02] # coefficients of the polynomial creating the rolling body
length = 0.1                # length of the roller
contour =  [[-length/2, 0]]  
x = np.linspace(start = - length/2, stop =length/2, num=51) 
for i in range(np.size(x)):
    contour+= [[x[i], np.polyval(poly, x[i])]]
contour += [ [length/2, 0]] # to create a closed contour
graphRoll = [graphics.SolidOfRevolution([0,0,0], [1,0,0], contour, graphics.color.lightred[0:3]+[1],
                                          alternatingColor=graphics.color.blue, nTiles = 32)]

InertiaRoll = InertiaCylinder(density=7800, length=length, outerRadius=3e-3, axis=0) 
bRoll = mbs.CreateRigidBody(inertia = InertiaRoll, 
                            referencePosition = [0,0,poly[-1]*1.2],  
                            referenceRotationMatrix =RotationMatrixY(np.pi/16),
                            initialAngularVelocity = RotationMatrixY(np.pi/16) @ np.array([-1000,0,0]),  # in Global coordinates
                            initialVelocity= [0,0,0],
                            gravity = [0,0,-9.81], 
                            graphicsDataList = graphRoll) 

nRoll = mbs.GetObject(bRoll)['nodeNumber']
        
nData = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
mRoll = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nRoll))
CConvexRoll = mbs.AddObject(ObjectContactConvexRoll(markerNumbers=[mGround, mRoll], 
                        nodeNumber=nData, contactStiffness=1e3, contactDamping=1, dynamicFriction = 0.9,
                       staticFrictionOffset = 0, viscousFriction=0, exponentialDecayStatic=1e-3, 
                       frictionProportionalZone=1e-4, rollLength=length, coefficientsHull=poly, 
                       visualization={'show': True, 'color': graphics.color.lightgreen}))

sBody = mbs.AddSensor(SensorBody(bodyNumber=bRoll, #fileName='PosRoller.txt',
                                 storeInternal=True,
                                 outputVariableType=exu.OutputVariableType.Position, visualization={'show': False})) 

mbs.Assemble()

h = 5e-4   #test
tEnd = 0.1 #test
#tEnd = 0.1*20 #for simulation
sims=exu.SimulationSettings()
sims.timeIntegration.generalizedAlpha.spectralRadius=0.7
sims.timeIntegration.endTime = tEnd
sims.timeIntegration.numberOfSteps = int(tEnd/h) #original: 1e-3, fails now in Newton
sims.timeIntegration.verboseMode = 0
sims.timeIntegration.stepInformation = 3 #do not show step reduction
sims.solutionSettings.coordinatesSolutionFileName = 'solution/coordinatesSolution.txt'
# sims.timeIntegration.newton.absoluteTolerance = 1e-8
# sims.timeIntegration.newton.relativeTolerance = 1e-6

if useGraphics: 
    sims.timeIntegration.verboseMode = 1
    sims.timeIntegration.stepInformation = 3+128+256
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()
mbs.SolveDynamic(sims)
if useGraphics: 
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

sol = mbs.systemData.GetODE2Coordinates()
exudynTestGlobals.testResult = np.sum(sol[:2])
exu.Print('result of ConvexContactTest=',exudynTestGlobals.testResult)
# %% 
if useGraphics: 
    #pos = np.loadtxt('PosRoller.txt', delimiter=',', comments='#')
    pos = mbs.GetSensorStoredData(sBody)
    exu.Print('End Pos: {}'.format(pos[-1,:]))
    
    
    mbs.PlotSensor(sBody,[0,1,2])
    
    
if useGraphics and False:
    SC.visualizationSettings.general.autoFitScene = False
    SC.visualizationSettings.general.graphicsUpdateInterval=0.02
    
    sol = LoadSolutionFile('solution/coordinatesSolution.txt', safeMode=True)#, maxRows=100)
    exu.Print('start SolutionViewer')
    mbs.SolutionViewer(sol)

