#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test models ANCFThinPlate from 2007 paper Schwab, Gerstmayr, Meijaard, DETC2007-34754
#
# Author:   Johannes Gerstmayr
# Date:     2021-03-25
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

## import libaries
import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

import numpy as np
from math import sin, cos, pi

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
useGraphics = True
from exudyn.shells import ShellMesh
    
totalTestResults = 0

## set up mbs
SC = exu.SystemContainer()
mbs = SC.AddSystem()

## setup system container and mbs
SC = exu.SystemContainer()
mbs = SC.AddSystem()

testCases = {
             'tipForceStatic4mode0':4,#'tipForceStatic8':8,'tipForceStatic16':16,
             'tipTorqueStatic4mode0':4,#'tipTorqueStatic8':8,'tipTorqueStatic16':16,
             'tipForceStatic4mode1':4,#'tipForceStatic8':8,'tipForceStatic16':16,
             'tipTorqueStatic4mode1':4,#'tipTorqueStatic8':8,'tipTorqueStatic16':16,
             'eigenmodesFreeFree':4,
             'eigenmodesSimplySupported':4,
             }

for testCase, nFact in testCases.items():
    print('*** TEST: '+testCase+' ***')
    isStatic = ('static' in testCase.lower())
    isEigenmodes = ('eigenmodes' in testCase.lower())
    isDynamic = not isStatic and not isEigenmodes
    schwabMode = 1 if ('mode1' in testCase) else 0
    
    eigenmodesFreeFree = 1 if (isEigenmodes and ('freefree' in testCase.lower())) else 0
    eigenmodesSimplySupported = 1 if (isEigenmodes and ('simplysupported' in testCase.lower())) else 0
    
    mbs.Reset()
    ## define parameters for beams
    nx = nFact
    ny = nFact
    Lx = 1
    Ly = 1
    Em=2e11*0.1                 # steel
    rho=7800
    tPlate=0.02                 # thickness
    nu = 0.3               # Poisson's ratio

    p0 = np.array([0,0,0])
    dx = np.array([Lx,0,0])
    dy = np.array([0,Ly,0])

    plateMesh = ShellMesh(vertices=[p0,p0+dx,p0+dx+dy,p0+dy],
                          numberOfElementsX=nx,
                          numberOfElementsY=ny,
                          youngsModulus=Em, poissonsRatio=nu, density=rho, thickness=tPlate,
                          massProportionalDamping=5*0)

    plateMesh.CreateANCFThinPlateElements(mbs)

    oGround = mbs.CreateGround()

    #boundaries for eigenmodes
    if eigenmodesSimplySupported:
        for node in plateMesh.boundaryNodeNumbers['all']:
            mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node))
            mbs.CreateSphericalJoint(bodyNumbers=[oGround, mNode], jointRadius = 0.025)

    if False: #Dmitrochenko/Pogorelov 2003 test case
        for node in plateMesh.boundaryNodeNumbers['all']:
            mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node))
            mbs.CreateSphericalJoint(bodyNumbers=[oGround, mNode], jointRadius = 0.025)

        nNodes = len(plateMesh.nodeNumbers)
        a = Lx
        q0 = 216.5
        #q0 = 216.5*20000
        fPerNode =  q0 * (1/nNodes * tPlate * a * a) * (Em*tPlate**4) / a**4

        for node in plateMesh.nodeNumbers:
            mNode = mbs.AddMarker(MarkerNodePosition(nodeNumber=node))
            mbs.AddLoad(Force(markerNumber=mNode, loadVector = [0,0,-fPerNode]))

    #++++++++++++++++++++++++++++++++++++++
    #select test mode:
    testTipForce = ('tipForceStatic' in testCase)
    testTipTorque = ('tipTorqueStatic' in testCase)
    testAcceleration = False #nut used
    
    testSchwab = testTipForce or testTipTorque

    #add constraints
    if testSchwab:
        if schwabMode == 0: #just change coordinate system
            for node in plateMesh.boundaryNodeNumbers['bottom']:
                mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node))
                mbs.CreateGenericJoint(bodyNumbers=[oGround, mNode], axesRadius = 0.025, axesLength=0.03)
        else:
            for node in plateMesh.boundaryNodeNumbers['left']:
                mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node))
                mbs.CreateGenericJoint(bodyNumbers=[oGround, mNode], axesRadius = 0.025, axesLength=0.03)

    numberOfLoadSteps = 1
    Mtip = 0
    Ftip = 0
    Dplate = Em*tPlate**3/(12*(1-nu**2))
    if testSchwab:
        Ftip = 1 * testTipForce
        Mtip = 1 * testTipTorque

        if False:
            #large deforation
            Ftip = 10000 * testTipForce
            Mtip = Dplate*pi*2 * testTipTorque
            numberOfLoadSteps = 20

        M = (Lx/(2*nx) ) * Mtip
        F = (Lx/(2*nx) ) * Ftip
        Fsum = 0
        Msum = 0
        if schwabMode == 0:
            for node in plateMesh.boundaryNodeNumbers['top']:
                fact = 2
                if node in plateMesh.boundaryNodeNumbers['left']+plateMesh.boundaryNodeNumbers['right']:
                    fact = 1
                mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node))
                mbs.AddLoad(Torque(markerNumber=mNode, loadVector = [M*fact,0,0]))
                mbs.AddLoad(Force(markerNumber=mNode, loadVector = [0,0,-F*fact]))
                Fsum += F*fact
                Msum += M*fact
            sPosSchwab = mbs.AddSensor(SensorNode(nodeNumber=plateMesh.boundaryNodeNumbers['top'][0], storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
        else:
            for node in plateMesh.boundaryNodeNumbers['right']:
                fact = 2
                if node in plateMesh.boundaryNodeNumbers['bottom']+plateMesh.boundaryNodeNumbers['top']:
                    fact = 1
                mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node))
                mbs.AddLoad(Torque(markerNumber=mNode, loadVector = [0,M*fact,0]))
                mbs.AddLoad(Force(markerNumber=mNode, loadVector = [0,0,-F*fact]))
                Fsum += F*fact
                Msum += M*fact
            sPosSchwab = mbs.AddSensor(SensorNode(nodeNumber=plateMesh.boundaryNodeNumbers['top'][-1], storeInternal=True, outputVariableType=exu.OutputVariableType.Position))

        print('Fsum=',Fsum, ', Msum=',Msum,', D=',Dplate)

    if testAcceleration:
        g = 9.81
        for elem in plateMesh.elementNumbers:
            mMass = mbs.AddMarker(MarkerBodyMass(bodyNumber=elem))
            mbs.AddLoad(LoadMassProportional(markerNumber=mMass, loadVector = [0,0,-g]))

    # sPos = mbs.AddSensor(SensorNode(nodeNumber=plateMesh.boundaryNodeNumbers['top'][0], storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
    # sPos2 = mbs.AddSensor(SensorNode(nodeNumber=plateMesh.boundaryNodeNumbers['top'][-1], storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
    # sMid = mbs.AddSensor(SensorNode(nodeNumber=plateMesh.nodeNumbers[(nx+1)*(ny//2)+nx//2], storeInternal=True, outputVariableType=exu.OutputVariableType.Position))

    ## assemble system and define simulation settings
    mbs.Assemble()

    simulationSettings = exu.SimulationSettings()

    tEnd = 0.05*5
    stepSize = 0.01
    simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
    simulationSettings.timeIntegration.endTime = tEnd
    simulationSettings.timeIntegration.verboseMode = useGraphics
    simulationSettings.staticSolver.verboseMode = useGraphics
    simulationSettings.solutionSettings.solutionWritePeriod = 0.02
    simulationSettings.solutionSettings.writeSolutionToFile = True
    # simulationSettings.displayComputationTime = True

    simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
    simulationSettings.timeIntegration.newton.useModifiedNewton = True #for faster simulation

    simulationSettings.staticSolver.numberOfLoadSteps = numberOfLoadSteps
    # simulationSettings.staticSolver.newton.numericalDifferentiation.addReferenceCoordinatesToEpsilon = True
    # simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-9
    simulationSettings.staticSolver.newton.absoluteTolerance = 1e-6
    simulationSettings.staticSolver.newton.relativeTolerance = 1e-6
    simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-6 #for steel
    simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6 #for steel

    ## add some visualization settings
    SC.visualizationSettings.view0.window.renderWindowSize = [1600,1200]
    SC.visualizationSettings.nodes.defaultSize = 0.02
    SC.visualizationSettings.nodes.drawNodesAsPoint = False
    SC.visualizationSettings.bodies.beams.crossSectionFilled = True
    SC.visualizationSettings.bodies.beams.axialTiling = 8

    SC.visualizationSettings.openGL.light0.position = [5,5,10,1]
    #SC.visualizationSettings.openGL.lightModelAmbient = [0.6,0.6,0.6]
    SC.visualizationSettings.openGL.light0.diffuse = 0.4
    # SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.CurvatureLocal
    # SC.visualizationSettings.contour.outputVariableComponent = 0
    SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.CurvatureLocal
    SC.visualizationSettings.contour.outputVariableComponent = 0
    SC.visualizationSettings.sensors.defaultSize = 0.1
    SC.visualizationSettings.sensors.drawSimplified = False

    if isEigenmodes:
        [values, systemEigenVectors] = mbs.ComputeODE2Eigenvalues(useSparseSolver=eigenmodesFreeFree, 
                                                                  numberOfEigenvalues=32,
                                                                  #convert2Frequencies=True
                                                                  )
        eigenvaluesFact = pi**2/(Lx**2) * np.sqrt(Dplate/(rho*tPlate)) #for 
        values = np.array(np.sqrt(values))
        valuesPart = values[6*eigenmodesFreeFree:10+6*eigenmodesFreeFree]
        totalTestResults += 1e-3*np.sum(valuesPart/eigenvaluesFact) #fact to reduce size
        exu.Print(f'eigenfrequencies, Nx=Ny={nFact}:',valuesPart/eigenvaluesFact)
        #FreeFree: 
        #omega0=1.364614 * fact
        #omega1=1.985504 * fact
        #omega2=2.459086 * fact
        #SimplySupported:
        #omega0=(1**2 + 1**2) * fact
        #omega1=(2**2 + 1**2) * fact
        #omega2=(2**2 + 2**2) * fact
            

        SC.visualizationSettings.bodies.beams.axialTiling = 16 #32 does not run smooth!
        SC.visualizationSettings.openGL.multiSampling = 4
        SC.visualizationSettings.openGL.lineWidth = 3
        if useGraphics:
            from exudyn.interactive import AnimateModes
            AnimateModes(SC, mbs, nodeNumber=None, systemEigenVectors=systemEigenVectors, 
                         runOnStart=True,)
            # import sys
            # sys.exit()
    else:
    
        if useGraphics:
            SC.renderer.Start()
            SC.renderer.SetModelView(zoom=1.021832,rotationVector=[-0.1321511,-1.268006,-1.984913],centerPoint=[-0.1076974,-1.14091,0.635892])
            SC.renderer.DoIdleTasks()
    
        ## run dynamic simulation
        if isStatic:
            mbs.SolveStatic(simulationSettings)
        elif isDynamic:
            mbs.SolveDynamic(simulationSettings)
            #mbs.SolveDynamic(simulationSettings, solverType = exu.DynamicSolverType.ExplicitEuler)
        elif not isEigenmodes:
            raise ValueError('ANCFThinPlateTests: no valid mode')
            
        if useGraphics:
            SC.renderer.DoIdleTasks()
            SC.renderer.Stop()
    
        if testSchwab and isStatic:
            #Dplate = plateMesh.Dcurvature[0,0]
            wExact = Mtip*Ly**2/(2*Dplate) + Ftip*Ly**3 / (3*Dplate)
            deformationValue = mbs.GetSensorValues(sPosSchwab)[2]/wExact
            print('wExact  =',wExact , ', fact=', deformationValue )
            totalTestResults += abs(0.1*deformationValue)
    
        # print('tip pos =',list(mbs.GetSensorValues(sPos)) )
        # print('tip pos2=',list(mbs.GetSensorValues(sPos2)) )
        # print('mid pos =',list(mbs.GetSensorValues(sMid)) )
        #print('mid defl normalized=',mbs.GetSensorValues(sMid)[2]/tPlate )



exu.Print('\nsolution of ANCFThinPlateTests =',totalTestResults )

#exudynTestGlobals.testError = totalTestResults - (0.544141453130599) 
exudynTestGlobals.testResult = totalTestResults



