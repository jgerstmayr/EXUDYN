#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  The solver module provides interfaces to static, dynamic and eigenvalue solvers.
#           Most of the solvers are implemented inside the C++ core.
#
# Author:   Johannes Gerstmayr 
# Date:     2020-12-02
# Notes:    Solver functions are included directly in exudyn and can be used with exu.SolveStatic(...)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++import sys

#import is necessary, otherwise the solvers cannot be called
import exudyn

solverCheckMemoryAllocations = True
solverCheckMemoryAllocationsThreshold = 100000 #treshold for warning on too many news during solving

#**function: solves the static mbs problem using simulationSettings; check theDoc.pdf for MainSolverStatic for further details of the static solver
#**input:
#   mbs: the MainSystem containing the assembled system; note that mbs may be changed upon several runs of this function
#   simulationSettings: specific simulation settings used for computation of jacobian (e.g., sparse mode in static solver enables sparse computation)
#   updateInitialValues: if True, the results are written to initial values, such at a consecutive simulation uses the results of this simulation as the initial values of the next simulation
#   storeSolver: if True, the staticSolver object is stored in the mbs.sys dictionary as mbs.sys['staticSolver'] 
#**output: returns True, if successful, False if fails; if storeSolver = True, mbs.sys contains staticSolver, which allows to investigate solver problems (check theDoc.pdf section \refSection{sec:solverSubstructures} and the items described in \refSection{sec:MainSolverStatic})
#**example:
# import exudyn as exu
# from exudyn.itemInterface import *
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# #create simple system:
# ground = mbs.AddObject(ObjectGround())
# mbs.AddNode(NodePoint())
# body = mbs.AddObject(MassPoint(physicsMass=1, nodeNumber=0))
# m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=ground))
# m1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=body))
# mbs.AddObject(CartesianSpringDamper(markerNumbers=[m0,m1], stiffness=[100,100,100]))
# mbs.AddLoad(LoadForceVector(markerNumber=m1, loadVector=[10,10,10]))
# mbs.Assemble()
# sims = exu.SimulationSettings()
# sims.timeIntegration.endTime = 10
# success = exu.SolveStatic(mbs, sims, storeSolver = True)
# print("success =", success)
# print("iterations = ", mbs.sys['staticSolver'].it)
# print("pos=", mbs.GetObjectOutputBody(body,localPosition=[0,0,0], 
#       variableType=exu.OutputVariableType.Position))
def SolveStatic(mbs, simulationSettings = exudyn.SimulationSettings(), 
                updateInitialValues = False,
                storeSolver = True):
    staticSolver = exudyn.MainSolverStatic()
    
    success = staticSolver.SolveSystem(mbs, simulationSettings)
    
    if updateInitialValues:
        currentState = mbs.systemData.GetSystemState() #get current values
        mbs.systemData.SetSystemState(systemStateList=currentState, configuration = exudyn.ConfigurationType.Initial)

    if storeSolver:
        mbs.sys['staticSolver'] = staticSolver #copy solver structure to sys variable

    return success

#**function: solves the dynamic mbs problem using simulationSettings and solver type; check theDoc.pdf for MainSolverImplicitSecondOrder for further details of the dynamic solver
#**input:
#   mbs: the MainSystem containing the assembled system; note that mbs may be changed upon several runs of this function
#   simulationSettings: specific simulation settings
#   solverType: use exudyn.DynamicSolverType to set specific solver (default=generalized alpha)
#   updateInitialValues: if True, the results are written to initial values, such at a consecutive simulation uses the results of this simulation as the initial values of the next simulation
#   storeSolver: if True, the staticSolver object is stored in the mbs.sys dictionary as mbs.sys['staticSolver'] 
#   experimentalNewSolver: this allows to use the new solver; flag only used during development - will be removed in future!
#**output: returns True, if successful, False if fails; if storeSolver = True, mbs.sys contains staticSolver, which allows to investigate solver problems (check theDoc.pdf section \refSection{sec:solverSubstructures} and the items described in \refSection{sec:MainSolverStatic})
#**example:
# import exudyn as exu
# from exudyn.itemInterface import *
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# #create simple system:
# ground = mbs.AddObject(ObjectGround())
# mbs.AddNode(NodePoint())
# body = mbs.AddObject(MassPoint(physicsMass=1, nodeNumber=0))
# m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=ground))
# m1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=body))
# mbs.AddObject(CartesianSpringDamper(markerNumbers=[m0,m1], stiffness=[100,100,100]))
# mbs.AddLoad(LoadForceVector(markerNumber=m1, loadVector=[10,10,10]))
# mbs.Assemble()
# sims = exu.SimulationSettings()
# sims.timeIntegration.endTime = 10
# success = exu.SolveDynamic(mbs, sims, storeSolver = True)
# print("success =", success)
# print("iterations = ", mbs.sys['dynamicSolver'].it)
# print("pos=", mbs.GetObjectOutputBody(body,localPosition=[0,0,0], 
#       variableType=exu.OutputVariableType.Position))
def SolveDynamic(mbs,
                simulationSettings = exudyn.SimulationSettings(), 
                solverType = exudyn.DynamicSolverType.GeneralizedAlpha,
                updateInitialValues = False,
                storeSolver = True,
                ):

    if (solverType == exudyn.DynamicSolverType.TrapezoidalIndex2 or solverType == exudyn.DynamicSolverType.GeneralizedAlpha):
    
        dynamicSolver = exudyn.MainSolverImplicitSecondOrder()
        #if (experimentalNewSolver or #solver flag
        #    ('experimentalNewSolver' in exudyn.sys)): #flag set in test suite
        #    dynamicSolver.experimentalUseSolverNew = True #must be set at the very beginning when MainSolverImplicitSecondOrder() is initialized
    
        #store old settings:
        newmarkOld = simulationSettings.timeIntegration.generalizedAlpha.useNewmark
        index2Old = simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints

        if solverType == exudyn.DynamicSolverType.TrapezoidalIndex2:
            #manually override settings for integrator
            simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
            simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
    
        stat = exudyn.InfoStat(False)
        success = dynamicSolver.SolveSystem(mbs, simulationSettings)
        CheckSolverInfoStatistics(dynamicSolver.GetSolverName(), stat, dynamicSolver.it.newtonStepsCount) #now check if these statistics are ok

        #restore old settings:
        simulationSettings.timeIntegration.generalizedAlpha.useNewmark = newmarkOld
        simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = index2Old
    elif (solverType == exudyn.DynamicSolverType.ExplicitEuler or 
          solverType == exudyn.DynamicSolverType.ExplicitMidpoint or
          solverType == exudyn.DynamicSolverType.RK33 or
          solverType == exudyn.DynamicSolverType.RK44 or
          solverType == exudyn.DynamicSolverType.RK67 or
          solverType == exudyn.DynamicSolverType.ODE23 or
          solverType == exudyn.DynamicSolverType.DOPRI5
          ):
        simulationSettings.timeIntegration.explicitIntegration.dynamicSolverType = solverType
        dynamicSolver = exudyn.MainSolverExplicit()

        stat = exudyn.InfoStat(False)
        success = dynamicSolver.SolveSystem(mbs, simulationSettings)
        CheckSolverInfoStatistics(dynamicSolver.GetSolverName(), stat, dynamicSolver.it.currentStepIndex*dynamicSolver.GetNumberOfStages()) #now check if these statistics are ok
    else:
        raise ValueError("SolveDynamic: solver type not implemented: ", solverType)
    
    if updateInitialValues:
        currentState = mbs.systemData.GetSystemState() #get current values
        mbs.systemData.SetSystemState(systemStateList=currentState, 
                                      configuration = exudyn.ConfigurationType.Initial)
        mbs.systemData.SetODE2Coordinates_tt(coordinates = mbs.systemData.GetODE2Coordinates_tt(), 
                                             configuration = exudyn.ConfigurationType.Initial)

    if storeSolver:
        mbs.sys['dynamicSolver'] = dynamicSolver #copy solver structure to sys variable

    return success

#**function: compute eigenvalues for unconstrained ODE2 part of mbs, not considering the effects of algebraic constraints; the computation is done for the initial values of the mbs, independently of previous computations. If you would like to use the current state for the eigenvalue computation, you need to copy the current state to the initial state (using GetSystemState,SetSystemState, see \refSection{sec:mbs:systemData}).
#**input:    
#   mbs: the MainSystem containing the assembled system
#   simulationSettings: specific simulation settings used for computation of jacobian (e.g., sparse mode in static solver enables sparse computation)
#   useSparseSolver: if False (only for small systems), all eigenvalues are computed in dense mode (slow for large systems!); if True, only the numberOfEigenvalues are computed (numberOfEigenvalues must be set!); Currently, the matrices are exported only in DENSE MODE from mbs! NOTE that the sparsesolver accuracy is much less than the dense solver
#   numberOfEigenvalues: number of eigenvalues and eivenvectors to be computed; if numberOfEigenvalues==0, all eigenvalues will be computed (may be impossible for larger problems!)
#   convert2Frequencies: if True, the eigen values are converted into frequencies (Hz) and the output is [eigenFrequencies, eigenVectors]
#**output: [eigenValues, eigenVectors]; eigenValues being a numpy array of eigen values ($\omega_i^2$, being the squared eigen frequencies in ($\omega_i$ in rad/s)!), eigenVectors a numpy array containing the eigenvectors in every column
#**example:
#  #take any example from the Examples or TestModels folder, e.g., 'cartesianSpringDamper.py' and run it
#  #then execute the following commands in the console (or add it to the file):
#  [values, vectors] = exu.ComputeODE2Eigenvalues(mbs)
#  print("eigenvalues=", values)
#  #==>values contains the eigenvalues of the ODE2 part of the system in the current configuration
def ComputeODE2Eigenvalues(mbs, 
                           simulationSettings = exudyn.SimulationSettings(),
                           useSparseSolver = False, 
                           numberOfEigenvalues = 0,
                           setInitialValues = True,
                           convert2Frequencies = False):
    import numpy as np
    #use static solver, as it does not include factors from time integration (and no velocity derivatives) in the jacobian
    staticSolver = exudyn.MainSolverStatic()

    #initialize solver with initial values
    staticSolver.InitializeSolver(mbs, simulationSettings)

    staticSolver.ComputeMassMatrix(mbs)
    M = staticSolver.GetSystemMassMatrix()

    staticSolver.ComputeJacobianODE2RHS(mbs)    #compute ODE2 part of jacobian ==> stored internally in solver
    staticSolver.ComputeJacobianAE(mbs)         #compute algebraic part of jacobian (not needed here...)
    jacobian = staticSolver.GetSystemJacobian() #read out stored jacobian

    nODE2 = staticSolver.GetODE2size()

    #obtain ODE2 part from jacobian == stiffness matrix
    K = jacobian[0:nODE2,0:nODE2]

    if not useSparseSolver:
        from scipy.linalg import eigh  #eigh for symmetric matrices, positive definite; eig for standard eigen value problems
        [eigenValuesUnsorted, eigenVectors] = eigh(K, M) #this gives omega^2 ... squared eigen frequencies (rad/s)
        eigenValues = np.sort(a=abs(eigenValuesUnsorted)) #eigh returns unsorted eigenvalues...
        if numberOfEigenvalues > 0:
            eigenValues = eigenValues[0:numberOfEigenvalues]
            eigenVectors = eigenVectors[0:numberOfEigenvalues]
    else:
        if numberOfEigenvalues == 0: #compute all eigenvalues
            numberOfEigenvalues = nODE2

        from scipy.sparse.linalg import eigsh, csr_matrix #eigh for symmetric matrices, positive definite

        Kcsr = csr_matrix(K)
        Mcsr = csr_matrix(M)

        #use "LM" (largest magnitude), but shift-inverted mode with sigma=0, to find the zero-eigenvalues:
        #see https://docs.scipy.org/doc/scipy/reference/tutorial/arpack.html
        [eigenValues, eigenVectors] = eigsh(A=Kcsr, k=numberOfEigenvalues, M=Mcsr, 
                                   which='LM', sigma=0, mode='normal') 

        #sort eigenvalues
        eigenValues = np.sort(a=abs(eigenValues))

    if convert2Frequencies:
        eigenFrequencies = np.sqrt(eigenValues)/(2*np.pi)
        return [eigenFrequencies, eigenVectors]
    else:
        return [eigenValues, eigenVectors]
    
#**function: helper function for solvers to check e.g. if high number of memory allocations happened during simulation
#            This can happen, if large amount of sensors are attached and output is written in every time step
#**input: stat=exudyn.InfoStat() from previous step, numberOfEvaluations is a counter which is proportional to number of RHS evaluations in method
def CheckSolverInfoStatistics(solverName, infoStat, numberOfEvaluations):
    import numpy as np

    stat = np.array(exudyn.InfoStat(False)) - np.array(infoStat)

    newCnt = max(stat[0],stat[2],stat[4]) #array, vector, matrix new counts

    if newCnt > solverCheckMemoryAllocationsThreshold and newCnt >= numberOfEvaluations:
        exudyn.Print("WARNING: "+solverName+" detected large amount ("+str(newCnt)+") of memory allocations, which seem to occur in every time step; solver may be slow")

    #print("newcnt=", newCnt)
