#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test model for GeometricallyExactBeam3D (UNDER DEVELOPMENT)
#
# Author:   Johannes Gerstmayr
# Date:     2022-05-03
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test model for GeometricallyExactBeam2D, cantilever beam with tip force and torque
#
# Author:   Johannes Gerstmayr
# Date:     2021-03-25
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *

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


L = 2 #length of beam

useEP = True
if useEP:
    NodeClass = NodeRigidBodyEP
    initialRotations = eulerParameters0
else:
    NodeClass = NodeRigidBodyRotVecLG
    initialRotations = [0,0,0]

sectionData = exu.BeamSection()
#print('section=',sectionData)

if True:
    nElements = 8 #4 works
    L = 1
    fTip = -4.05e5*0.5*0.25
    bodyFixedLoad = False
    # Material properties
    E = 2.1e11            # Young modulus
    nu = 0.3              # Poisson ratio
    G = E/(2*(1+nu))      # Shear modulus
    k = 6*(1+nu)/(7+6*nu) # Shear correction factor

    # Cross-section properties
    r = 0.025              # Radius of the beam cross-section
    A = np.pi*r**2         # Area of the beam cross-section
    I = (1/4)*np.pi*(r**4) # Second moment of area of the beam cross-section
    J = 2*I                # Polar moment of area of the beam cross-section
    sectionData.stiffnessMatrix = np.diag([E*A, k*G*A, k*G*A, G*J, E*I, E*I])
else:
    nElements = 5
    fTip = -10
    bodyFixedLoad = False
    sectionData.stiffnessMatrix = 100*np.eye(6)
    sectionData.stiffnessMatrix[0,0] = 1e4
    sectionData.stiffnessMatrix[1,1] = 1e4
    sectionData.stiffnessMatrix[2,2] = 1e4


EI = sectionData.stiffnessMatrix[5,5]
#linear solution:
uTip = fTip*L**3/(3*EI)
print('u linear=',uTip)

sectionData.inertia= 1*np.eye(3)
sectionData.massPerLength = 1e2

sectionGeometry = exu.BeamSectionGeometry()
w = 0.05 #half width of beam
u = 0.1*w
v = w-u #hight of I inner

#points, in positive rotation sense viewing in x-direction, points in [Y,Z]-plane
#points do not need to be closed!
lp = exu.Vector2DList()
if False:
    lp.Append([w,-w])
    lp.Append([w,w])
    lp.Append([-w,w])
    lp.Append([-w,-w])
else:
    lp.Append([-w,-w])
    lp.Append([-v,-w])
    lp.Append([-v,-u])
    lp.Append([ v,-u])
    lp.Append([ v,-w])
    lp.Append([ w,-w])

    lp.Append([ w, w])
    lp.Append([ v, w])
    lp.Append([ v, u])
    lp.Append([-v, u])
    lp.Append([-v, w])
    lp.Append([-w, w])
sectionGeometry.polygonalPoints = lp
#print('HERE\n',sectionGeometry.polygonalPoints)

lElem = L/nElements
n0 = mbs.AddNode(NodeClass(referenceCoordinates=[0,0,0]+initialRotations))
nInit = n0
for k in range(nElements):
    n1 = mbs.AddNode(NodeClass(referenceCoordinates=[lElem*(k+1),0,0]+initialRotations))

    oBeam = mbs.AddObject(ObjectBeamGeometricallyExact(nodeNumbers=[n0,n1], physicsLength = lElem, 
                                                         sectionData = sectionData,
                                                         visualization=VBeam3D(sectionGeometry=sectionGeometry)))
    n0 = n1

mTip = mbs.AddMarker(MarkerNodeRigid(nodeNumber = n1))
if False:
    mbs.AddLoad(Force(markerNumber=mTip, loadVector = [0,fTip,0], bodyFixed = bodyFixedLoad ))
elif True:
    #mbs.AddLoad(Force(markerNumber=mTip, loadVector = [0,fTip*0.1,fTip*0] ))
    mbs.AddLoad(Torque(markerNumber=mTip, loadVector = [0*fTip,0*0.2*fTip,1*fTip]))
else:
    mbs.AddLoad(Torque(markerNumber=mTip, loadVector = [3*fTip,0,0])) #static

nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
mnGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
 
for i in range(6):
    ii = i
    if i==3 and useEP:
        ii=6
    nm0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nInit, coordinate=ii))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mnGround, nm0]))

#nm1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=1))
#mbs.AddObject(CoordinateConstraint(markerNumbers=[mnGround, nm1]))
#nm2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=2))
#mbs.AddObject(CoordinateConstraint(markerNumbers=[mnGround, nm2]))


if False:
    lElem = L/nElements
    n2d0 = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0,0,0]))
    nInit = n2d0
    for k in range(nElements):
        n2d1 = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[lElem*(k+1),0,0]))

        oBeam = mbs.AddObject(ObjectBeamGeometricallyExact2D(nodeNumbers=[n2d0,n2d1], physicsLength = lElem,
                                                             physicsMassPerLength=sectionData.massPerLength,
                                                             physicsAxialStiffness=sectionData.stiffnessMatrix[0,0],
                                                             physicsBendingStiffness=EI,
                                                             physicsShearStiffness=sectionData.stiffnessMatrix[1,1],
                                                             physicsCrossSectionInertia=sectionData.inertia[2,2],
                                                             ))
        n2d0 = n2d1

    mTip = mbs.AddMarker(MarkerNodeRigid(nodeNumber = n2d1))
    mbs.AddLoad(Force(markerNumber=mTip, loadVector = [0,fTip,0], bodyFixed = bodyFixedLoad ))

    for i in range(3):
        nm0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nInit, coordinate=i))
        mbs.AddObject(CoordinateConstraint(markerNumbers=[mnGround, nm0]))


#print(mbs.GetObject(oBeam))

#print(mbs)
mbs.Assemble()

tEnd = 100     #end time of simulation
h = 0.01    #step size; leads to 1000 steps

simulationSettings = exu.SimulationSettings()
simulationSettings.solutionSettings.solutionWritePeriod = 5e-3  #output interval general
simulationSettings.solutionSettings.sensorsWritePeriod = 5e-3  #output interval of sensors
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h) #must be integer
simulationSettings.timeIntegration.endTime = tEnd
#simulationSettings.solutionSettings.solutionInformation = "This is the info\nNew line\n and another new line \n"
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
#simulationSettings.timeIntegration.simulateInRealtime=True
#simulationSettings.timeIntegration.realtimeFactor=0.1
simulationSettings.timeIntegration.verboseMode = 1
#simulationSettings.parallel.numberOfThreads = 4
# simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 1e-4
simulationSettings.timeIntegration.newton.numericalDifferentiation.forODE2 = True
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
#simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

simulationSettings.staticSolver.verboseMode = 1
simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 5e-5
#simulationSettings.staticSolver.newton.numericalDifferentiation.forODE2 = True
simulationSettings.staticSolver.newton.relativeTolerance = 1e-6
simulationSettings.staticSolver.numberOfLoadSteps = 1 #50


#add some drawing parameters for this example
SC.visualizationSettings.nodes.drawNodesAsPoint=False
SC.visualizationSettings.nodes.defaultSize=0.01

SC.visualizationSettings.bodies.beams.axialTiling = 50
SC.visualizationSettings.general.drawWorldBasis = True
SC.visualizationSettings.general.worldBasisSize = 0.1
SC.visualizationSettings.openGL.multiSampling = 4




    
if useGraphics:
    exu.StartRenderer()
    mbs.WaitForUserToContinue()

#exu.SolveStatic(mbs, simulationSettings)
exu.SolveDynamic(mbs, simulationSettings)
#exu.SolveDynamic(mbs, simulationSettings, solverType = exu.DynamicSolverType.RK44)
        

if useGraphics:
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!


##evaluate final (=current) output values
pTip = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
print('pTip=',pTip)
# pTip2D = mbs.GetNodeOutput(n2d1, exu.OutputVariableType.Position)
# print('pTip2D=',pTip2D)


# exudynTestGlobals.testError = uLast[1] - (-2.2115028353806547) 
# exudynTestGlobals.testResult = uLast[1]



