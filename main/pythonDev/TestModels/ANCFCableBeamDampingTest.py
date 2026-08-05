#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#
# Details:  This is a test script that compares the axial and bending damping
#           of the ANCF beam element with the ANCF 2D cable element.
# Authors:  Johannes Gerstmayr and Sebastian Weyrer
# Date:     2026-03-23
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import numpy as np
from exudyn.utilities import *
import exudyn.graphics as graphics

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
useGraphics = False

exu.Print('EXUDYN says hello with version', exu.config.Version())

#++++++++++++++++++++++++++++++++++++++++++++++++++
# set parameters and behavior of script
useSolutionViewer = False
recordImages = False
loadVectorList = [[0, -5, 0],
                  [-100, 0, 0]
                  ]
# cable parameters
l = 0.5                                             # [m] length of cable
massPerLength = 2                                   # [kg/m]
cableDiameter = 0.02                                # [m]
E = 1e8                                             # [N/m^2] E modulus of cable (in all directions!)

nElementsANCF2D = 4                                 # number of elements the ANCF 2D cable is made out of
nElementsANCFBeam = 8                               # number of elements the ANCF beam is made out of 

if useGraphics: #more accurate results
    nElementsANCF2D *= 4
    nElementsANCFBeam *= 4

beta = 0.02*0                                         # Rayleigh damping for bending
betaAxial = 0.0005*0                                  # Rayleigh damping for axial deformation

# automatically dependent parameters
lElement = l/nElementsANCFBeam                      # [m] length of one element (used in ANCF beam)
m = massPerLength*l                                 # [kg] full cable mass
cableRadius = cableDiameter/2                       # [m] radius of the cable
cableArea = np.pi*cableRadius**2                    # [m^2] cross section area of the cable
secondMomentOfInertia = np.pi*cableRadius**4/4      # [m^4] second moment of inertia (bending)
rho = massPerLength/cableArea                       # [kg/m^3] density of cable
rhoA = rho*cableArea                                # [kg/m] mass per unit length
rhoI = rho*secondMomentOfInertia                    # [kg*m] inertia per unit length
EA = E*cableArea                                    # [N] axial stiffness
EI = E*secondMomentOfInertia                        # [N*m^2] bending stiffness

# set up cross section data resulting from parameters (assemble matrices where needed) (X --> torsion, Y and Z --> bending)
sectionData = exu.BeamSection()
kPenalty = EA * 1 # penalty stiffness that is added to eliminate shear deformation
sectionData.stiffnessMatrix = np.diag([EA, kPenalty, kPenalty, kPenalty, EI, EI])               
sectionData.inertia = np.diag([0, rhoI, rhoI])
sectionData.massPerLength = rhoA
sectionData.dampingMatrix = np.diag([EA*betaAxial, 0, 0, 0, EI*beta, EI*beta])

# visualization of cable
nTiles = 18
sectionGeometry = exu.BeamSectionGeometry()
lp = exu.Vector2DList()
phi = 2*np.pi/nTiles
for i in range(nTiles):
    lp.Append([cableRadius*np.cos(i*phi), cableRadius*np.sin(i*phi)])
sectionGeometry.polygonalPoints = lp

#%% set up system container (once for all tests)
SC = exu.SystemContainer()

# general visualization settings
SC.visualizationSettings.nodes.show = True
SC.visualizationSettings.markers.show = True
SC.visualizationSettings.markers.drawSimplified = True
SC.visualizationSettings.view0.scene.drawCoordinateSystem = True
SC.visualizationSettings.bodies.beams.axialTiling = 1
# simulation settings
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = False
simulationSettings.displayComputationTime = True
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

if recordImages:
    simulationSettings.solutionSettings.recordImagesInterval = 0.1
mbs = SC.AddSystem()

solution = 0 #accumulated result for test suite

# iterate over the load vectors
for loadCase, loadVector in enumerate(loadVectorList):
    #%% set up general things for the test
    mbs.Reset() # reset mbs since we now make another test
    oGround = mbs.CreateGround(referencePosition=[0, 0, 0])

    #%% set up the ANCF 2D cable
    cableTemplate = Cable2D(physicsMassPerLength=rhoA,
                            physicsBendingStiffness=EI,
                            physicsAxialStiffness=EA,
                            physicsBendingDamping=beta*EI,
                            physicsAxialDamping=betaAxial*EA,
                            useReducedOrderIntegration=0,
                            visualization=VCable2D(drawHeight=cableDiameter))
    nCable2D, oCable2D, lCable2D, _, _ = GenerateStraightLineANCFCable2D(mbs, positionOfNode0=[0, 0, 0],
                                                                         positionOfNode1=[l, 0, 0],
                                                                         fixedConstraintsNode0=[1]*4,
                                                                         numberOfElements=nElementsANCF2D,
                                                                         cableTemplate=cableTemplate)
    mCable2D = mbs.AddMarker(MarkerNodePosition(nodeNumber=nCable2D[-1]))
    # add load to last node
    mbs.AddLoad(LoadForceVector(markerNumber=mCable2D,
                                loadVector=loadVector, bodyFixed=False))
    # add sensor to get position of last node
    sCable2D = mbs.AddSensor(SensorMarker(markerNumber=mCable2D,
                                          outputVariableType=exu.OutputVariableType.Displacement,
                                          storeInternal=True))

    #%% set up ANCF beam cable
    referenceOffset = [0, 0, 1]
    initialRotations = [0, 1, 0] + [0, 0, 1]
    mCableList = [] # this list holds the markers to which the discs can then be attached
    n0 = mbs.AddNode(NodePointSlope23(referenceCoordinates=referenceOffset + initialRotations))
    mCableList += [mbs.AddMarker(MarkerNodeRigid(nodeNumber=n0))]
    for k in range(nElementsANCFBeam):
        n1 = mbs.AddNode(NodePointSlope23(referenceCoordinates=[lElement*(k+1) + referenceOffset[0], referenceOffset[1], referenceOffset[2]] + initialRotations, visualization=VNodePointSlope23(show=True)))
        mCableList += [mbs.AddMarker(MarkerNodeRigid(nodeNumber=n1))]
        oBeam = mbs.AddObject(ObjectANCFBeam(nodeNumbers=[n0, n1],
                                             physicsLength=lElement, 
                                             sectionData=sectionData, #includes bending stiffness, axial stiffness, damping, etc.
                                             crossSectionPenaltyFactor=[1]*3,
                                             visualization=VANCFBeam(sectionGeometry=sectionGeometry,
                                                                     color=graphics.color.grey)))
        n0 = n1

    # fix cable to ground
    mbs.CreateGenericJoint(bodyNumbers=[oGround, mCableList[0]], show=True)
    # add load to last node
    mbs.AddLoad(LoadForceVector(markerNumber=mCableList[-1],
                                loadVector=loadVector, bodyFixed=False))
    # add sensor to get position of last node
    sBeam = mbs.AddSensor(SensorMarker(markerNumber=mCableList[-1],
                                       outputVariableType=exu.OutputVariableType.Displacement,
                                       storeInternal=True))

    #%% do simulation (implicit dynamic simulation) (WYSWYS - What You See is What You Simulate)
    mbs.Assemble()


    stepSize = 4e-3
    tEnd = 4
    if loadCase == 1:
        stepSize = 0.5e-3
        tEnd = 0.25
        
    if not useGraphics: #for test suite
        tEnd = 0.1

    #tEnd = 0.1
    
    simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
    simulationSettings.timeIntegration.endTime = tEnd
    simulationSettings.solutionSettings.writeSolutionToFile = useSolutionViewer
    simulationSettings.solutionSettings.sensorsWritePeriod = stepSize

    #++++++++++++++++++++++++++++++++++++++++++++++++++
    if useGraphics:
        SC.renderer.Start()
        SC.renderer.DoIdleTasks()
    mbs.SolveDynamic(simulationSettings=simulationSettings)
    if useGraphics:
        SC.renderer.DoIdleTasks()
        SC.renderer.Stop() #safely close rendering window!
    
    #++++++++++++++++++++++++++++++++++++++++++++++++++
    if useSolutionViewer and useGraphics:
        mbs.SolutionViewer()

    #%% plot displacement to compare results
    if useGraphics:
        [plt, fig, ax, line] = mbs.PlotSensor(sensorNumbers=[sCable2D, sBeam],
                       components=[1-loadCase]*2, 
                       title="Test Damping: load="+str(loadVector),
                       labels=['ANCF cable 2D','ANCF beam'], fontSize=12,
                       legendArgs=(0.65,0.78),
                       )

    #for test suite:
    solution += np.linalg.norm(mbs.GetSensorValues(sCable2D))
    solution += np.linalg.norm(mbs.GetSensorValues(sBeam))

exu.Print('ANCFCableBeamDampingTest: solution=', solution)

exudynTestGlobals.testResult = solution

