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

compute2D = True
compute3D = False


L = 2 #length of beam
w = 0.05 #half width of beam
u = 0.1*w
v = w-u #height of I inner
h = w #height Y

computeEigenmodes = True
sectionData = exu.BeamSection()
#print('section=',sectionData)

if True: #Nachbagauer, et al 2011
    nElements = 2
    L = 3
    bodyFixedLoad = False
    # Material properties
    if True: #eigenmodes
        h = 0.4
        w = 0.4
        E = 1e9            # Young modulus
        rho = 7850
    else:
        h = 0.5
        w = 0.1
        E = 2.07e11            # Young modulus
        rho = 1e2/(h*w)
    nu = 0.3              # Poisson ratio
    ks= 10*(1+nu)/(12+11*nu)
    G = E/(2*(1+nu))      # Shear modulus
    A=h*w

    fTip = 5e7*h**3

    # Cross-section properties
    I = w*h**3/12 # Second moment of area of the beam cross-section
    J = 2*I       # Polar moment of area of the beam cross-section
    sectionData.stiffnessMatrix = np.diag([E*A, G*A*ks, G*A*ks, G*J, E*I, E*I])
    print('stiffness=', np.diag(sectionData.stiffnessMatrix))
elif True:
    nElements = 16 #4 works
    L = 1
    fTip = -4.05e5*0.5*4 #*0.25
    bodyFixedLoad = True
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
    print('stiffness=', np.diag(sectionData.stiffnessMatrix))
else:
    nElements = 5
    fTip = -10
    bodyFixedLoad = False
    sectionData.stiffnessMatrix = 100*np.eye(6)
    sectionData.stiffnessMatrix[0,0] = 1e4
    sectionData.stiffnessMatrix[1,1] = 1e4
    sectionData.stiffnessMatrix[2,2] = 1e4
    A= 0.1
    rho=1000

rhoA = rho*A

EI = sectionData.stiffnessMatrix[5,5]
#linear solution:
uTip = fTip*L**3/(3*EI)
print('u linear=',uTip)

sectionData.inertia= rho*I*np.eye(3)
sectionData.massPerLength = rhoA

sectionGeometry = exu.BeamSectionGeometry()

#points, in positive rotation sense viewing in x-direction, points in [Y,Z]-plane
#points do not need to be closed!
lp = exu.Vector2DList()
if True:
    ff=0.2 #drawing
    lp.Append([h*ff,-w*ff])
    lp.Append([h*ff,w*ff])
    lp.Append([-h*ff,w*ff])
    lp.Append([-h*ff,-w*ff])
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
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
mnGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
 

eY=[0,1,0]
eZ=[0,0,1]
lElem = L/nElements
if compute3D:
    initialRotations = eY+eZ
    n0 = mbs.AddNode(NodePoint3DSlope23(referenceCoordinates=[0,0,0]+initialRotations))
    nInit = n0
    for k in range(nElements):
        n1 = mbs.AddNode(NodePoint3DSlope23(referenceCoordinates=[lElem*(k+1),0,0]+initialRotations))

        oBeam = mbs.AddObject(ObjectANCFBeam3D(nodeNumbers=[n0,n1], physicsLength = lElem, 
                                               testBeamRectangularSize = [h,w],
                                               sectionData = sectionData,
                                               crossSectionPenaltyFactor = [1,1,1],
                                               visualization=VBeam3D(sectionGeometry=sectionGeometry)))
        n0 = n1


    if not computeEigenmodes:
        mTip = mbs.AddMarker(MarkerNodePosition(nodeNumber = n1))
        mbs.AddLoad(Force(markerNumber=mTip, loadVector = [0,fTip*1,0], bodyFixed = bodyFixedLoad))
    
        #mTip = mbs.AddMarker(MarkerNodeRigid(nodeNumber = n1))
        #if True:
        #    mbs.AddLoad(Force(markerNumber=mTip, loadVector = [0,fTip,0], bodyFixed = bodyFixedLoad ))
        #elif True:
        #    mbs.AddLoad(Force(markerNumber=mTip, loadVector = [0,fTip,0] ))
        #    mbs.AddLoad(Torque(markerNumber=mTip, loadVector = [1*fTip,0.2*fTip,0]))
        #else:
        #    mbs.AddLoad(Torque(markerNumber=mTip, loadVector = [3*fTip,0,0])) #static
    
    
        for i in range(9):
            nm0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nInit, coordinate=i))
            mbs.AddObject(CoordinateConstraint(markerNumbers=[mnGround, nm0]))


if compute2D:
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

    if not computeEigenmodes:
        mTip = mbs.AddMarker(MarkerNodeRigid(nodeNumber = n2d1))
        mbs.AddLoad(Force(markerNumber=mTip, loadVector = [0,fTip,0], bodyFixed = bodyFixedLoad ))
    
        for i in range(3):
            nm0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nInit, coordinate=i))
            mbs.AddObject(CoordinateConstraint(markerNumbers=[mnGround, nm0]))


print('EA=', sectionData.stiffnessMatrix[0,0])
print('EI=', EI)

#print(mbs.GetObject(oBeam))

# print(mbs)
mbs.Assemble()

tEnd = 100     #end time of simulation
stepSize = 0.01*0.1    #step size; leads to 1000 steps

simulationSettings = exu.SimulationSettings()
simulationSettings.solutionSettings.solutionWritePeriod = 5e-3  #output interval general
simulationSettings.solutionSettings.sensorsWritePeriod = 5e-3  #output interval of sensors
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize) #must be integer
simulationSettings.timeIntegration.endTime = tEnd
#simulationSettings.solutionSettings.solutionInformation = "This is the info\nNew line\n and another new line \n"
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
#simulationSettings.timeIntegration.simulateInRealtime=True
#simulationSettings.timeIntegration.realtimeFactor=0.1

simulationSettings.timeIntegration.verboseMode = 1
#simulationSettings.parallel.numberOfThreads = 4
simulationSettings.timeIntegration.newton.useModifiedNewton = True
#simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1e0

simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 1e-4
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6

simulationSettings.displayComputationTime = True
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

simulationSettings.staticSolver.verboseMode = 1
#simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 5e-5
#simulationSettings.staticSolver.newton.numericalDifferentiation.forODE2 = True
#simulationSettings.staticSolver.newton.relativeTolerance = 1e-6
simulationSettings.staticSolver.numberOfLoadSteps = 10 #50


#add some drawing parameters for this example
SC.visualizationSettings.nodes.drawNodesAsPoint=False
SC.visualizationSettings.nodes.defaultSize=0.01

SC.visualizationSettings.bodies.beams.axialTiling = 50
SC.visualizationSettings.general.drawWorldBasis = True
SC.visualizationSettings.general.worldBasisSize = 0.1
SC.visualizationSettings.openGL.multiSampling = 4


# [M, K, D] = exu.solver.ComputeLinearizedSystem(mbs, simulationSettings, useSparseSolver=True)
# print('M=',M.round(1))

if useGraphics:
    exu.StartRenderer()


if computeEigenmodes:
    nModes = 3*(1+int(compute3D))
    nRigidModes = 3*(1+int(compute3D))
    if compute2D:
        constrainedCoordinates=[0,1,mbs.systemData.ODE2Size()-2]
    else:
        constrainedCoordinates=[0,1,2,5,mbs.systemData.ODE2Size()-8,mbs.systemData.ODE2Size()-7]
    
    # constrainedCoordinates=[]
        
    compeig=exu.ComputeODE2Eigenvalues(mbs, simulationSettings, useSparseSolver=False, 
                                numberOfEigenvalues= nRigidModes+nModes, 
                                constrainedCoordinates=constrainedCoordinates,
                                convert2Frequencies= False)
    
    print('eigvalues=',np.sqrt(compeig[0][nRigidModes:]))
        
    if False: #show modes:
        for i in range(nModes):
            iMode = nRigidModes+i
            mbs.systemData.SetODE2Coordinates(5*compeig[1][:,iMode], exudyn.ConfigurationType.Visualization)
            mbs.systemData.SetTime(np.sqrt(compeig[0][iMode]), exudyn.ConfigurationType.Visualization)
            mbs.SendRedrawSignal()
        
            mbs.WaitForUserToContinue()

else:
    exu.SolveStatic(mbs, simulationSettings)
    # exu.SolveDynamic(mbs, simulationSettings)
    #exu.SolveDynamic(mbs, simulationSettings, solverType = exu.DynamicSolverType.RK44)
    

if useGraphics:
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!


##evaluate final (=current) output values
if compute3D:
    pTip = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
    print('pTip=',pTip)
if compute2D:
     pTip2D = mbs.GetNodeOutput(n2d1, exu.OutputVariableType.Position)
     print('pTip2D=',pTip2D)


# exudynTestGlobals.testError = uLast[1] - (-2.2115028353806547) 
# exudynTestGlobals.testResult = uLast[1]



