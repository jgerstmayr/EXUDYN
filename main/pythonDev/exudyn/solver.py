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

#**function: helper function for unique error and helper messages
def SolverErrorMessage(solver, mbs, isStatic=False, 
                       showCausingObjects=True, showCausingNodes=True, showHints=True):
    s = ''
    s += '\n******************************\n'
    if isStatic:
        s += 'STATIC SOLVER FAILED:\n'
    else:        
        s += 'DYNAMIC SOLVER FAILED:\n'
    #print(solver.conv)
    massMatrixNotInvertible = solver.conv.massMatrixNotInvertible
    linearSolverFailed = solver.conv.linearSolverFailed
    causingRow = solver.conv.linearSolverCausingRow
    newtonFailed = solver.conv.stepReductionFailed or solver.conv.newtonSolutionDiverged

    

    if showHints:
        s += '  POSSIBLE REASONS for solver abort:\n'
        if linearSolverFailed:
            s += '  * unused node or no equations supplied for node\n'
            s += '  * redundant definition of constraints\n'
        if (solver.conv.stepReductionFailed and (not solver.conv.newtonSolutionDiverged)
            and solver.conv.residual < 1e-3): #indicates that solution is not very wrong
            s += '  * your tolerances (in newton) may be too tight, see solutions below\n'
            
        s += '  * inconsistent or inappropriate initial conditions\n'
        s += '  * system is nearly singular due to high (penalty) stiffness or singularities in your system\n'
        if isStatic:
            s += '  * static problem has unconstrained coordinates (that can move freely)\n'
            if newtonFailed:
                s += '  * the system is very nonlinear and thus requires smaller (load) steps\n'
        else:
            if newtonFailed:
                s += '  * too large step size, which prevents Newton to converge\n'
            if linearSolverFailed:
                s += '  * singular mass matrix (no mass/inertia assigned?)\n'

        s += '  -------------------\n'
        s += '  POSSIBLE SOLUTIONS:\n'
        s += '  * check your Python model!\n'
        s += '  * check the causing nodes, objects, connectors, etc.\n'
        s += '  * check your nodes, objects, connectors, and markers\n'
        s += '  * change solver and solver settings (e.g. change index 3/index 2 solver)\n'
        s += '  * try linearSolverSettings.ignoreSingularJacobian\n'
        s += '  * step-by-step remove constraints, until you find the causing item\n'
        s += '  * step-by-step remove objects and nodes, until you find the causing item\n'
        s += '  * check joint axes (using visualization), which may be incompatible\n'
        s += '  * use lower (penalty) stiffness factors\n'
        if newtonFailed:
            if isStatic:
                s += '  * adjust your tolerances: the newton solver may not be able to reach the relative or absolute tolerance, so increase them, e.g., staticSolver.newton.absoluteTolerance; if no loads are applied, usually absolute tolerances should be around 1e-5 to 1e-2\n'
            else:
                s += '  * adjust your tolerances: the newton solver may not be able to reach the relative or absolute tolerance, so increase them, e.g., timeIntegration.newton.absoluteTolerance; if no loads are applied, usually absolute tolerances should be around 1e-5 to 1e-2\n'
            s += '  * use smaller step size or load steps\n'
            s += '  * use adaptiveStep to reduce step size in static or dynamic simulation\n'
            s += '  * adjust the way you initialize your model and how to apply loads, etc.\n'
        s += '  * report error, clearly describing (a minimal description) of your problem, at reply.exudyn@gmail.com\n'
    else:
        s += "  use showHints=True to show helpful information\n"
    s += '******************************\n'
    
    nODE2 = solver.GetODE2size()
    nODE1 = solver.GetODE1size()
    nAE = solver.GetAEsize()
    nSys = nODE2+nODE1+nAE

    causingObjects = []
    if linearSolverFailed and causingRow >=0 and causingRow < nSys:
        s+="The causing system equation "+str(causingRow)+" belongs to a "
        if causingRow < nODE2:
            s+="ODE2 coordinate"
        elif causingRow < nODE2+nODE1:
            s+="ODE1 coordinate"
        else:
            s+="algebraic variable (Lagrange multiplier)"
        s+='\n'

        import numpy as np
        if showCausingObjects:
            for objectIndex in range(mbs.systemData.NumberOfObjects()):
                ltg = mbs.systemData.GetObjectLTGODE2(objectIndex)
                addObject = False
                if causingRow in ltg:
                    addObject = True
                ltg = mbs.systemData.GetObjectLTGODE1(objectIndex)
                if causingRow - (nODE2) in ltg:
                    addObject = True
                ltg = mbs.systemData.GetObjectLTGAE(objectIndex)
                if causingRow - (nODE2+nODE1) in ltg:
                    addObject = True

                if addObject and (objectIndex not in causingObjects):
                    causingObjects += [objectIndex]
            s+="Potential object number(s) causing linear solver to fail: "+str(causingObjects)+"\n"
            for i in causingObjects:
                oDict = mbs.GetObject(i)
                s += "    object "+str(i)+", name='"+oDict['name']+"', type="+ str(oDict['objectType']) +"\n"


    return s


#**function: solves the static mbs problem using simulationSettings; check theDoc.pdf for MainSolverStatic for further details of the static solver; NOTE that this function is directly available from exudyn (using exudyn.SolveStatic(...))
#**input:
#   mbs: the MainSystem containing the assembled system; note that mbs may be changed upon several runs of this function
#   simulationSettings: specific simulation settings out of exu.SimulationSettings(), as described in \refSection{sec:SolutionSettings}; use options for newton, discontinuous settings, etc., from staticSolver sub-items
#   updateInitialValues: if True, the results are written to initial values, such at a consecutive simulation uses the results of this simulation as the initial values of the next simulation
#   storeSolver: if True, the staticSolver object is stored in the mbs.sys dictionary as mbs.sys['staticSolver'], and simulationSettings are stored as mbs.sys['simulationSettings']
#**output: returns True, if successful, False if fails; if storeSolver = True, mbs.sys contains staticSolver, which allows to investigate solver problems (check theDoc.pdf \refSection{sec:solverSubstructures} and the items described in \refSection{sec:MainSolverStatic})
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
                storeSolver = True,
                showHints = False,
                showCausingItems = True,
                ):
    staticSolver = exudyn.MainSolverStatic()
    if storeSolver:
        mbs.sys['staticSolver'] = staticSolver #copy solver structure to sys variable
        mbs.sys['simulationSettings'] = simulationSettings #link to last simulation settings
    success = False
    try:
        success = staticSolver.SolveSystem(mbs, simulationSettings)
    except:
        pass
    finally:
        if not success:
            #resolved (delete):
            # exudyn.Print not shown in Spyder at this time (because of exception?)
            # print(SolverErrorMessage(staticSolver, mbs, isStatic=True, showCausingObjects=showCausingItems, 
            #                          showCausingNodes=showCausingItems, showHints=showHints), flush=True)
            exudyn.Print(SolverErrorMessage(staticSolver, mbs, isStatic=True, showCausingObjects=showCausingItems, 
                                     showCausingNodes=showCausingItems, showHints=showHints))
            raise ValueError("SolveStatic terminated due to errors")

        elif updateInitialValues:
            currentState = mbs.systemData.GetSystemState() #get current values
            mbs.systemData.SetSystemState(systemStateList=currentState, configuration = exudyn.ConfigurationType.Initial)


    return success

#**function: solves the dynamic mbs problem using simulationSettings and solver type; check theDoc.pdf for MainSolverImplicitSecondOrder for further details of the dynamic solver; NOTE that this function is directly available from exudyn (using exudyn.SolveDynamic(...))
#**input:
#   mbs: the MainSystem containing the assembled system; note that mbs may be changed upon several runs of this function
#   simulationSettings: specific simulation settings out of exu.SimulationSettings(), as described in \refSection{sec:SolutionSettings}; use options for newton, discontinuous settings, etc., from timeIntegration; therein, implicit second order solvers use settings from generalizedAlpha and explict solvers from explicitIntegration; be careful with settings, as the influence accuracy (step size!), convergence and performance (see special \refSection{secSpeedUp})
#   solverType: use exudyn.DynamicSolverType to set specific solver (default=generalized alpha)
#   updateInitialValues: if True, the results are written to initial values, such at a consecutive simulation uses the results of this simulation as the initial values of the next simulation
#   storeSolver: if True, the staticSolver object is stored in the mbs.sys dictionary as mbs.sys['staticSolver'], and simulationSettings are stored as mbs.sys['simulationSettings']
#   showHints: show additional hints, if solver fails
#   showCausingItems: if linear solver fails, this option helps to identify objects, etc. which are related to a singularity in the linearized system matrix
#**output: returns True, if successful, False if fails; if storeSolver = True, mbs.sys contains staticSolver, which allows to investigate solver problems (check theDoc.pdf \refSection{sec:solverSubstructures} and the items described in \refSection{sec:MainSolverStatic})
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
                showHints = False,
                showCausingItems = True,
                ):
    success = False
    if (solverType == exudyn.DynamicSolverType.TrapezoidalIndex2 or solverType == exudyn.DynamicSolverType.GeneralizedAlpha):
    
        dynamicSolver = exudyn.MainSolverImplicitSecondOrder()
        if storeSolver:
            mbs.sys['dynamicSolver'] = dynamicSolver #copy solver structure to sys variable
            mbs.sys['simulationSettings'] = simulationSettings #link to last simulation settings
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
        success = False
        try:
            success = dynamicSolver.SolveSystem(mbs, simulationSettings)
        except:
            pass
        finally:
            if not success:
                #resolved (delete):
                # print(SolverErrorMessage(dynamicSolver, mbs, isStatic=False, showCausingObjects=showCausingItems, 
                #                          showCausingNodes=showCausingItems, showHints=showHints), flush=True)
                exudyn.Print(SolverErrorMessage(dynamicSolver, mbs, isStatic=False, showCausingObjects=showCausingItems, 
                                         showCausingNodes=showCausingItems, showHints=showHints))
                raise ValueError("SolveDynamic terminated")
                
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
        if storeSolver:
            mbs.sys['dynamicSolver'] = dynamicSolver #copy solver structure to sys variable
            mbs.sys['simulationSettings'] = simulationSettings #link to last simulation settings

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

    return success

#**function: compute linearized system of equations for ODE2 part of mbs, not considering the effects of algebraic constraints
#**input:    
#   mbs: the MainSystem containing the assembled system
#   simulationSettings: specific simulation settings used for computation of jacobian (e.g., sparse mode in static solver enables sparse computation)
#   useSparseSolver: if False (only for small systems), all eigenvalues are computed in dense mode (slow for large systems!); if True, only the numberOfEigenvalues are computed (numberOfEigenvalues must be set!); Currently, the matrices are exported only in DENSE MODE from mbs! NOTE that the sparsesolver accuracy is much less than the dense solver
#**output: [M, K, D]; list containing numpy mass matrix M, stiffness matrix K and damping matrix D
#**example:
#  #take any example from the Examples or TestModels folder, e.g., 'cartesianSpringDamper.py' and run it
#  #then execute the following commands in the console (or add it to the file):
#  [M, K, D] = exu.ComputeLinearizedSystem(mbs)
#  print("M=", M)
#  print("K=", K)
def ComputeLinearizedSystem(mbs, 
                            simulationSettings = exudyn.SimulationSettings(),
                            useSparseSolver = False):
    import numpy as np
    #use static solver, as it does not include factors from time integration (and no velocity derivatives) in the jacobian
    staticSolver = exudyn.MainSolverStatic()

    #initialize solver with initial values
    staticSolver.InitializeSolver(mbs, simulationSettings)

    staticSolver.ComputeMassMatrix(mbs)
    M = staticSolver.GetSystemMassMatrix()

    staticSolver.ComputeJacobianODE2RHS(mbs)    #compute ODE2 part of jacobian ==> stored internally in solver
    #staticSolver.ComputeJacobianAE(mbs)         #compute algebraic part of jacobian (not needed here...)
    jacobian = staticSolver.GetSystemJacobian() #read out stored jacobian

    nODE2 = staticSolver.GetODE2size()

    #obtain ODE2 part from jacobian == stiffness matrix
    K = jacobian[0:nODE2,0:nODE2]

    staticSolver.ComputeJacobianODE2RHS(mbs, scalarFactor=0)    #reset jacobian
    staticSolver.ComputeJacobianODE2RHS_t(mbs)                  #compute ODE2_t part of jacobian ==> stored internally in solver
    jacobian_t = staticSolver.GetSystemJacobian() #read out stored jacobian
    
    #obtain ODE2_t part from jacobian == damping matrix
    D = jacobian_t[0:nODE2,0:nODE2]

    return [M, K, D]


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
