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
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#import is necessary, otherwise the solvers cannot be called
import numpy as np
import exudyn

solverCheckMemoryAllocations = True
solverCheckMemoryAllocationsThreshold = 100000 #treshold for warning on too many news during solving

#**function: (internal) helper function for unique error and helper messages
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
        s += '  * try linearSolverSettings.ignoreSingularJacobian and EigenDense solver type in case of redundant constraints\n'
        s += '  * for small systems (<1000 coordinates), try mbs. ComputeSystemDegreeOfFreedom() to check system\n'
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

    if massMatrixNotInvertible:
        s+='The mass matrix is not to invertable; check that every node/coordinate has appropriate mass!'

    return s


#**function: solves the static mbs problem using simulationSettings; check theDoc.pdf for MainSolverStatic for further details of the static solver; this function is also available in exudyn (using exudyn.SolveStatic(...))
#**input:
#   mbs: the MainSystem containing the assembled system; note that mbs may be changed upon several runs of this function
#   simulationSettings: specific simulation settings out of exu.SimulationSettings(), as described in \refSection{sec:SolutionSettings}; use options for newton, discontinuous settings, etc., from staticSolver sub-items
#   updateInitialValues: if True, the results are written to initial values, such at a consecutive simulation uses the results of this simulation as the initial values of the next simulation
#   storeSolver: if True, the staticSolver object is stored in the mbs.sys dictionary as mbs.sys['staticSolver'], and simulationSettings are stored as mbs.sys['simulationSettings']
#**output: bool; returns True, if successful, False if fails; if storeSolver = True, mbs.sys contains staticSolver, which allows to investigate solver problems (check theDoc.pdf \refSection{sec:solverSubstructures} and the items described in \refSection{sec:MainSolverStatic})
#**belongsTo: MainSystem
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
# simulationSettings = exu.SimulationSettings()
# simulationSettings.timeIntegration.endTime = 10
# success = mbs.SolveStatic(simulationSettings, storeSolver = True)
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

#**function: solves the dynamic mbs problem using simulationSettings and solver type; check theDoc.pdf for MainSolverImplicitSecondOrder for further details of the dynamic solver; this function is also available in exudyn (using exudyn.SolveDynamic(...))
#**input:
#   mbs: the MainSystem containing the assembled system; note that mbs may be changed upon several runs of this function
#   simulationSettings: specific simulation settings out of exu.SimulationSettings(), as described in \refSection{sec:SolutionSettings}; use options for newton, discontinuous settings, etc., from timeIntegration; therein, implicit second order solvers use settings from generalizedAlpha and explict solvers from explicitIntegration; be careful with settings, as the influence accuracy (step size!), convergence and performance (see special \refSection{sec:overview:basics:speedup})
#   solverType: use exudyn.DynamicSolverType to set specific solver (default=generalized alpha)
#   updateInitialValues: if True, the results are written to initial values, such at a consecutive simulation uses the results of this simulation as the initial values of the next simulation
#   storeSolver: if True, the staticSolver object is stored in the mbs.sys dictionary as mbs.sys['staticSolver'], and simulationSettings are stored as mbs.sys['simulationSettings']
#   showHints: show additional hints, if solver fails
#   showCausingItems: if linear solver fails, this option helps to identify objects, etc. which are related to a singularity in the linearized system matrix
#**output: bool; returns True, if successful, False if fails; if storeSolver = True, mbs.sys contains staticSolver, which allows to investigate solver problems (check theDoc.pdf \refSection{sec:solverSubstructures} and the items described in \refSection{sec:MainSolverStatic})
#**belongsTo: MainSystem
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
# #
# mbs.Assemble()
# simulationSettings = exu.SimulationSettings()
# simulationSettings.timeIntegration.endTime = 10
# success = mbs.SolveDynamic(simulationSettings, storeSolver = True)
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
            solverType == exudyn.DynamicSolverType.DOPRI5 or
            solverType == exudyn.DynamicSolverType.VelocityVerlet
            ):
        simulationSettings.timeIntegration.explicitIntegration.dynamicSolverType = solverType
        #print('solverType=', simulationSettings.timeIntegration.explicitIntegration.dynamicSolverType)
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

#**function: return success (True/False) and error message of solver after SolveSteps(...), SolveSystem(...), SolveDynamic(...) or SolveStatic(...) have been called. May also be set if other higher level functions called e.g. SolveSystem(...)
#**input: 
#  solverStructure: solver structure, as stored in mbs.sys or as created e.g. by exudyn.MainSolverExplicit()
#**output: [success, errorString], returns success=True or False and in case of no success, information is provided in errorString
#**example:
#  #assume MainSystem mbs, exu library and simulationSettings:
#  try:
#      mbs.SolveDynamic(simulationSettings)
#  except:
#      [success, msg] = exu.SolverSuccess(mbs.sys['dynamicSolver'])
#      print('success=',success)
#      print('error message=',msg)
#
#  #alternative:
#  solver=exu.MainSolverImplicitSecondOrder()
#  ...
#  [success, msg] = exu.SolverSuccess(solver)
def SolverSuccess(solverStructure):
    return [solverStructure.output.finishedSuccessfully, solverStructure.GetErrorString()]

#internal function: uniquely turn off all output and file writing - this would potentially erase user's output files ...
def DeactivateWritingOfSolvers(simulationSettings):
    store = {}
    store['verboseModeOld'] = simulationSettings.staticSolver.verboseMode
    simulationSettings.staticSolver.verboseMode = 0
    store['writeSolutionToFileOld'] = simulationSettings.solutionSettings.writeSolutionToFile
    simulationSettings.solutionSettings.writeSolutionToFile = False
    store['sensorsStoreAndWriteFilesOld'] = simulationSettings.solutionSettings.sensorsStoreAndWriteFiles
    simulationSettings.solutionSettings.sensorsStoreAndWriteFiles = False
    return store

#internal function for special solver functions; uniquely restores some settings
def RestoreSimulationSettings(simulationSettings, store):
    simulationSettings.staticSolver.verboseMode = store['verboseModeOld']
    simulationSettings.solutionSettings.writeSolutionToFile = store['writeSolutionToFileOld']
    simulationSettings.solutionSettings.sensorsStoreAndWriteFiles = store['sensorsStoreAndWriteFilesOld']
    

#**function: compute linearized system of equations for ODE2 part of mbs, not considering the effects of algebraic constraints; for computation of eigenvalues and advanced computation with constrained systems, see ComputeODE2Eigenvalues; the current implementation is also able to project into the constrained space, however, this currently does not generally work with non-holonomic systems
#**input:    
#   mbs: the MainSystem containing the assembled system
#   simulationSettings: specific simulation settings used for computation of jacobian (e.g., sparse mode in static solver enables sparse computation)
#   projectIntoConstraintNullspace: if False, algebraic equations (and constraint jacobian) are not considered for the linearized system; if True, the equations are projected into the nullspace of the constraints in the current configuration, using singular value decomposition; in the latter case, the returned list contains [M, K, D, C, N] where C is the constraint jacobian and N is the nullspace matrix (C and N may be an empty list, depending on the following flags)
#   singularValuesTolerance: tolerance used to distinguish between zero and nonzero singular values for algebraic constraints projection
#   returnConstraintJacobian: if True, the returned list contains [M, K, D, C, N] where C is the constraint jacobian and N is the nullspace matrix (may be empty)
#   returnConstraintNullspace: if True, the returned list contains [M, K, D, C, N] where C is the constraint jacobian (may be empty) and N is the nullspace matrix
#**output: [ArrayLike, ArrayLike, ArrayLike]; [M, K, D]; list containing numpy mass matrix M, stiffness matrix K and damping matrix D; for constraints, see options with arguments above, return values may change to [M, K, D, C, N]
#**notes: consider paper of Agundez, Vallejo, Freire, Mikkola, "The dependent coordinates in the linearization of constrained multibody systems: Handling and elimination", https://www.sciencedirect.com/science/article/pii/S0020740324000791
#**belongsTo: MainSystem
#**example:
# import exudyn as exu
# from exudyn.utilities import *
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# #
# b0 = mbs.CreateMassPoint(referencePosition = [2,0,0],
#                          initialVelocity = [2*0,5,0],
#                          physicsMass = 1, gravity = [0,-9.81,0],
#                          drawSize = 0.5, color=graphics.color.blue)
# #
# oGround = mbs.AddObject(ObjectGround())
# #add vertical spring
# oSD = mbs.CreateSpringDamper(bodyOrNodeList=[oGround, b0],
#                              localPosition0=[2,1,0],
#                              localPosition1=[0,0,0],
#                              stiffness=1e4, damping=1e2,
#                              drawSize=0.2)
# #
# mbs.Assemble()
# [M,K,D] = mbs.ComputeLinearizedSystem()
# exu.Print('M=\n',M,'\nK=\n',K,'\nD=\n',D) 
def ComputeLinearizedSystem(mbs, 
                            simulationSettings = exudyn.SimulationSettings(),
                            projectIntoConstraintNullspace = False,
                            singularValuesTolerance = 1e-12,
                            returnConstraintJacobian = False,
                            returnConstraintNullspace = False
                            ):

    #do not overide sensor files or solution files ...
    store = DeactivateWritingOfSolvers(simulationSettings)

    #use static solver, as it does not include factors from time integration (and no velocity derivatives) in the jacobian
    staticSolver = exudyn.MainSolverStatic()

    #initialize solver with initial values
    staticSolver.InitializeSolver(mbs, simulationSettings)

    nODE2 = staticSolver.GetODE2size()
    nODE1 = staticSolver.GetODE1size()
    nAE = staticSolver.GetAEsize()
    if nODE1 != 0:
        print('WARNING: ComputeLinearizedSystem: not implemented for ODE1 coordinates; results may be wrong')

    staticSolver.ComputeMassMatrix(mbs)
    Mode2 = staticSolver.GetSystemMassMatrix()[0:nODE2,0:nODE2]

    staticSolver.ComputeJacobianODE2RHS(mbs,scalarFactor_ODE2=-1,
                                        scalarFactor_ODE2_t=0, 
                                        scalarFactor_ODE1=0)    #compute ODE2 part of jacobian ==> stored internally in solver

    if nAE != 0 and (projectIntoConstraintNullspace or returnConstraintJacobian or returnConstraintNullspace):
        #compute AE part of jacobian if needed for constraint projection
        staticSolver.ComputeJacobianAE(mbs, scalarFactor_ODE2=1., scalarFactor_ODE2_t=0., 
                                       scalarFactor_ODE1=0., #could be 1 to include ODE1 part
                                       velocityLevel=False)          

    jacobian = staticSolver.GetSystemJacobian() #read out stored jacobian

    Kode2 = jacobian[0:nODE2,0:nODE2]    #obtain ODE2 part from jacobian == stiffness matrix

    staticSolver.ComputeJacobianODE2RHS(mbs,scalarFactor_ODE2=0,
                                        scalarFactor_ODE2_t=-1, 
                                        scalarFactor_ODE1=0)    #reset jacobian
    jacobian_t = staticSolver.GetSystemJacobian() #read out stored jacobian
    
    Dode2 = jacobian_t[0:nODE2,0:nODE2] #obtain ODE2_t part from jacobian == damping matrix

    staticSolver.FinalizeSolver(mbs, simulationSettings) #close files, etc.
    RestoreSimulationSettings(simulationSettings, store)

    #include constraints:
    listCN = []
    if nAE != 0 and (projectIntoConstraintNullspace or returnConstraintJacobian or returnConstraintNullspace):
        try:
            from scipy.linalg import svd
        except:
            raise ValueError('ComputeLinearizedSystem: missing scipy package; install with: pip install scipy')
        #use SVD to project equations into nullspace
        #constraint jacobian:
        C = jacobian[0:nODE2,nODE2+nODE1:]
        if returnConstraintJacobian:
            listCN = [C]
        else: listCN.append([])
        
        if projectIntoConstraintNullspace or returnConstraintNullspace:
            #compute SVD; D includes singular values
            [U,D,V] = svd(C)
            
            nnz = (abs(D) > singularValuesTolerance).sum() #size of constraints, often number of cols of C
    
            # print('K, D, M, C=',Kode2, Dode2, Mode2, C)
            nullspace = U[:,nnz:].T 

            if returnConstraintNullspace:
                listCN.append(nullspace)
            else: listCN.append([])

    if nAE != 0 and projectIntoConstraintNullspace:
        Knullspace = nullspace @ Kode2 @ nullspace.T
        Mnullspace = nullspace @ Mode2 @ nullspace.T
        Dnullspace = nullspace @ Dode2 @ nullspace.T
            
        return [Mnullspace, Knullspace, Dnullspace] + listCN
    else:
        return [Mode2, Kode2, Dode2] + listCN

#**function: compute eigenvalues for unconstrained ODE2 part of mbs, which represent the square of the eigenfrequencies (in radiant) of the undamped system; the computation may include constraints in case that ignoreAlgebraicEquations=False (however, this currently does not generally work with non-holonomic systems); for algebraic constraints, however, a dense singular value decomposition of the constraint jacobian is used for the nullspace projection; the computation is done for the initial values of the mbs, independently of previous computations. If you would like to use the current state for the eigenvalue computation, you need to copy the current state to the initial state (using GetSystemState, SetSystemState, see \refSection{sec:mbs:systemData}); note that mass and stiffness matrices are computed in dense mode so far, while eigenvalues are computed according to useSparseSolver.
#**input:    
#   mbs: the MainSystem containing the assembled system
#   simulationSettings: specific simulation settings used for computation of jacobian (e.g., sparse mode in static solver enables sparse computation)
#   useSparseSolver: if False (only for small systems), all eigenvalues are computed in dense mode (slow for large systems!); if True, only the numberOfEigenvalues are computed (numberOfEigenvalues must be set!); Currently, the matrices are exported only in DENSE MODE from mbs, which means that intermediate matrices may become huge for more than 5000 coordinates! NOTE that the sparsesolver accuracy is much less than the dense solver
#   numberOfEigenvalues: number of eigenvalues and eivenvectors to be computed; if numberOfEigenvalues==0, all eigenvalues will be computed (may be impossible for larger or sparse problems!)
#   constrainedCoordinates: if this list is non-empty (and there are no algebraic equations or ignoreAlgebraicEquations=True), the integer indices represent constrained coordinates of the system, which are fixed during eigenvalue/vector computation; according rows/columns of mass and stiffness matrices are erased; in this case, algebraic equations of the system are ignored
#   convert2Frequencies: if True, the square root is computed for eigenvalues, they are converted into frequencies (Hz), and the output is [eigenFrequencies, eigenVectors]
#   useAbsoluteValues: if True, abs(eigenvalues) is used, which avoids problems for small (close to zero) eigenvalues; needed, when converting to frequencies
#   computeComplexEigenvalues: if True, the system is converted into a system of first order differential equations, including damping terms; returned eigenvalues are complex and contain the 'damping' (=real) part and the eigenfrequency (=complex) part; for this case, set useAbsoluteValues=False (otherwise you will not get the complex values; values are unsorted, however!); also, convert2Frequencies must be False in this case! only implemented for dense solver
#   ignoreAlgebraicEquations: if True, algebraic equations (and constraint jacobian) are not considered for eigenvalue computation; otherwise, the solver tries to automatically project the system into the nullspace kernel of the constraint jacobian using a SVD; this gives eigenvalues of the constrained system; eigenvectors are not computed
#   singularValuesTolerance: tolerance used to distinguish between zero and nonzero singular values for algebraic constraints projection
#**output: [ArrayLike, ArrayLike]; [eigenValues, eigenVectors]; eigenValues being a numpy array of eigen values ($\omega_i^2$, being the squared eigen frequencies in ($\omega_i$ in rad/s)!), eigenVectors a numpy array containing the eigenvectors in every column
#**belongsTo: MainSystem
#**author: Johannes Gerstmayr, Michael Pieber
#**example:
#  #take any example from the Examples or TestModels folder, e.g., 'cartesianSpringDamper.py' and run it
#  #specific example:
# import exudyn as exu
# from exudyn.utilities import *
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# #
# b0 = mbs.CreateMassPoint(referencePosition = [2,0,0],
#                          physicsMass = 1, gravity = [0,-9.81,0],
#                          drawSize = 0.5, color=graphics.color.blue)
# #
# oGround = mbs.AddObject(ObjectGround())
# #add vertical spring
# oSD = mbs.CreateSpringDamper(bodyOrNodeList=[oGround, b0],
#                              localPosition0=[2,1,0],
#                              localPosition1=[0,0,0],
#                              stiffness=1e4, damping=1e2,
#                              drawSize=0.2)
# #
# mbs.Assemble()
# #
# [eigenvalues, eigenvectors] = mbs.ComputeODE2Eigenvalues()
# #==>eigenvalues contain the eigenvalues of the ODE2 part of the system in the current configuration
# #
# #compute eigenfrequencies in Hz (analytical: 100/2/pi Hz for y-direction):
# [eigenvaluesHz, ev] = mbs.ComputeODE2Eigenvalues(convert2Frequencies=True)
# #
# #compute complex eigenvalues:
# [eigenvaluesComplex, ev] = mbs.ComputeODE2Eigenvalues(computeComplexEigenvalues=True,
#                                                       useAbsoluteValues=False)
def ComputeODE2Eigenvalues(mbs, 
                           simulationSettings = exudyn.SimulationSettings(),
                           useSparseSolver = False, numberOfEigenvalues = 0, constrainedCoordinates=[],
                           convert2Frequencies = False, useAbsoluteValues = True, 
                           computeComplexEigenvalues = False,
                           ignoreAlgebraicEquations=False, singularValuesTolerance=1e-12):

    store = DeactivateWritingOfSolvers(simulationSettings)

    try:
        from scipy.linalg import eigh, eig, svd  #eigh for symmetric matrices, positive definite; eig for standard eigen value problems
        from scipy.sparse.linalg import eigsh #eigh for symmetric matrices, positive definite
        from scipy.sparse import csr_matrix
    except:
        raise ValueError('ComputeODE2Eigenvalues: missing scipy package; install with: pip install scipy')

    #use static solver, as it does not include factors from time integration (and no velocity derivatives) in the jacobian
    staticSolver = exudyn.MainSolverStatic()

    #initialize solver with initial values
    staticSolver.InitializeSolver(mbs, simulationSettings)

    nODE2 = staticSolver.GetODE2size()
    nODE1 = staticSolver.GetODE1size()
    nAE = staticSolver.GetAEsize()
    if nODE1 != 0:
        print('WARNING: ComputeODE2Eigenvalues: not implemented for ODE1 coordinates; results may be wrong')

    staticSolver.ComputeMassMatrix(mbs)
    Mode2 = staticSolver.GetSystemMassMatrix()

    #compute ODE2 part of jacobian ==> stored internally in solver
    staticSolver.ComputeJacobianODE2RHS(mbs,scalarFactor_ODE2=-1,
                                        scalarFactor_ODE2_t=0, 
                                        scalarFactor_ODE1=0)    #could be 1 to include ODE1 part
    if nAE != 0:
        #compute AE part of jacobian if needed for constraint projection
        staticSolver.ComputeJacobianAE(mbs, scalarFactor_ODE2=1., scalarFactor_ODE2_t=0., 
                                       scalarFactor_ODE1=0., #could be 1 to include ODE1 part
                                       velocityLevel=False)          
    
    jacobian = staticSolver.GetSystemJacobian() #read out stored jacobian; includes ODE2, ODE1 and nAE part

    #obtain ODE2 part from jacobian == stiffness matrix
    Kode2 = jacobian[0:nODE2,0:nODE2]

    if computeComplexEigenvalues:
        if convert2Frequencies:
            raise ValueError('ComputeODE2Eigenvalues: In case of computeComplexEigenvalues=True, you must set convert2Frequencies=False !')
            
        #compute damping matrix
        staticSolver.ComputeJacobianODE2RHS(mbs,scalarFactor_ODE2=0,
                                                scalarFactor_ODE2_t=-1, 
                                                scalarFactor_ODE1=0)    #could be 1 to include ODE1 part
        jacobianD = staticSolver.GetSystemJacobian() #read out stored jacobian; includes ODE2, ODE1 and nAE part
        Dode2 = jacobianD[0:nODE2,0:nODE2]

    remappingIndices = np.arange(nODE2) #maps new coordinates to original (full) ones
    if constrainedCoordinates != []:
        Mode2 = np.delete(np.delete(Mode2, constrainedCoordinates, 0), constrainedCoordinates, 1)
        Kode2 = np.delete(np.delete(Kode2, constrainedCoordinates, 0), constrainedCoordinates, 1)
        if computeComplexEigenvalues:
            Dode2 = np.delete(np.delete(Dode2, constrainedCoordinates, 0), constrainedCoordinates, 1)
        
        remappingIndices = np.delete(remappingIndices, constrainedCoordinates)

    staticSolver.FinalizeSolver(mbs, simulationSettings) #close files, etc.
    RestoreSimulationSettings(simulationSettings, store)

    if nAE != 0 and not ignoreAlgebraicEquations and constrainedCoordinates != []:
        raise ValueError('ComputeODE2Eigenvalues: in case of algebraic equations, either ignoreAlgebraicEquations=True or constrainedCoordinates=[]')


    if constrainedCoordinates != [] or nAE == 0 or ignoreAlgebraicEquations:
        if not useSparseSolver:
            if not computeComplexEigenvalues:
                [eigenValuesUnsorted, eigenVectors] = eigh(Kode2, Mode2) #this gives omega^2 ... squared eigen frequencies (rad/s)
            else:
                #complex eigenvalues
                B = np.block([[                  Kode2, Dode2     ],
                              [np.zeros((nODE2,nODE2)),-Mode2     ]])
            
                A = np.block([[np.zeros((nODE2,nODE2)), Mode2                    ],
                              [                  Mode2, np.zeros((nODE2,nODE2))]])                
                Amod = -np.dot(np.linalg.inv(A),B)
                [eigenValuesUnsorted, eigenVectors] = eig(Amod)
            
            if useAbsoluteValues:
                sortIndices = np.argsort(abs(eigenValuesUnsorted)) #get resorting index
                eigenValues = np.sort(a=abs(eigenValuesUnsorted)) #eigh returns unsorted eigenvalues...
            else:
                sortIndices = np.argsort(eigenValuesUnsorted) #get resorting index
                eigenValues = np.sort(a=eigenValuesUnsorted) #eigh returns unsorted eigenvalues...
            if numberOfEigenvalues > 0:
                eigenValues = eigenValues[0:numberOfEigenvalues]
                sortIndices = sortIndices[0:numberOfEigenvalues]
            eigenVectors = eigenVectors[:,sortIndices] #eigenvectors are given in columns!
        else:
            if computeComplexEigenvalues:
                raise ValueError('ComputeODE2Eigenvalues: computeComplexEigenvalues must be False with sparse solvers')
            if numberOfEigenvalues == 0: #compute all eigenvalues
                numberOfEigenvalues = nODE2
    
            Kcsr = csr_matrix(Kode2)
            Mcsr = csr_matrix(Mode2)
    
            #use "LM" (largest magnitude), but shift-inverted mode with sigma=0, to find the zero-eigenvalues:
            #see https://docs.scipy.org/doc/scipy/reference/tutorial/arpack.html
            [eigenValues, eigenVectors] = eigsh(A=Kcsr, k=numberOfEigenvalues, M=Mcsr, 
                                       which='LM', sigma=0, mode='normal') 
    
            #sort eigenvalues
            if useAbsoluteValues:
                sortIndices = np.argsort(abs(eigenValues)) #get resorting index
                eigenValues = np.sort(a=abs(eigenValues))
            else:
                sortIndices = np.argsort(eigenValues) #get resorting index
                eigenValues = np.sort(a=eigenValues)
            eigenVectors = eigenVectors[:,sortIndices] #eigenvectors are given in columns!

        eigenVectorsNew = np.zeros((nODE2,numberOfEigenvalues))
        if constrainedCoordinates != []:
            # print('remap=', remappingIndices)
            for i in range(numberOfEigenvalues):
                eigenVectorsNew[remappingIndices,i] = eigenVectors[:,i]
            eigenVectors = eigenVectorsNew
    else: #this includes general constraints and requires different solvers
                             
        if useSparseSolver:
            raise ValueError('ComputeODE2Eigenvalues: in case of algebraic equations and ignoreAlgebraicEquations=False, useSparseSolver must be False')
        #use SVD to project equations into nullspace
        #constraint jacobian:
        C = jacobian[0:nODE2,nODE2+nODE1:]
        
        #compute SVD; D includes singular values
        [U,D,V] = svd(C)
        
        nnz = (abs(D) > singularValuesTolerance).sum() #size of constraints, often number of cols of C
        
        nullspace = U[:,nnz:].T #U[nnz:]
        Knullspace = nullspace@Kode2@nullspace.T
        Mnullspace = nullspace@Mode2@nullspace.T
        nODE2ns = Knullspace.shape[0]
            
        if computeComplexEigenvalues:
            Dnullspace = nullspace@Dode2@nullspace.T
            #A*q_t + B*q=0 => q=[x,v]
            #[0  M] [x_t]   [K  D] [x]
            #[    ]*[   ] + [    ]*[ ] = 0
            #[I  0] [v_t]   [0 -I] [v]
            #complex eigenvalues
            B = np.block([[                 Knullspace,  Dnullspace                   ],
                          [np.zeros((nODE2ns,nODE2ns)), -Mnullspace                   ]])
        
            A = np.block([[np.zeros((nODE2ns,nODE2ns)), Mnullspace                 ],
                          [                 Mnullspace, np.zeros((nODE2ns,nODE2ns))]])                
            Amod = -np.dot(np.linalg.inv(A),B)
            [eigenValuesUnsorted, eigenVectorsReduced] = eig(Amod)
        else:
            [eigenValuesUnsorted, eigenVectorsReduced] = eigh(Knullspace,Mnullspace)
            
        if useAbsoluteValues:
            sortIndices = np.argsort(abs(eigenValuesUnsorted)) #get resorting index
            eigenValues = np.sort(a=abs(eigenValuesUnsorted)) #eigh returns unsorted eigenvalues...
        else:
            sortIndices = np.argsort(eigenValuesUnsorted) #get resorting index
            eigenValues = np.sort(a=eigenValuesUnsorted) #eigh returns unsorted eigenvalues...

        if numberOfEigenvalues > 0:
            eigenValues = eigenValues[0:numberOfEigenvalues]
            sortIndices = sortIndices[0:numberOfEigenvalues]
        eigenVectorsReduced = eigenVectorsReduced[:,sortIndices] #eigenvectors are given in columns!

        if computeComplexEigenvalues:
            eigenVectors = np.block([nullspace.T, nullspace.T]) @ eigenVectorsReduced
        else:
            eigenVectors = nullspace.T @ eigenVectorsReduced



    if convert2Frequencies: #not possible for computeComplexEigenvalues !
        eigenFrequencies = np.sqrt(eigenValues)/(2*np.pi)
        return [eigenFrequencies, eigenVectors]
    else:
        return [eigenValues, eigenVectors]

    

#**function: compute system DOF numerically, considering Gr{\"u}bler-Kutzbach formula as well as redundant constraints; uses numpy matrix rank or singular value decomposition of scipy (useSVD=True)
#**input:
#  mbs: MainSystem for which DOF shall be computed
#  simulationSettings: used e.g. for settings regarding numerical differentiation; default settings may be used in most cases
#  threshold: threshold factor for singular values which estimate the redundant constraints
#  useSVD: use singular value decomposition directly, also showing SVD values if verbose=True
#  verbose: if True, it will show the singular values and one may decide if the threshold shall be adapted
#**output: dict; returns dictionary with key words 'degreeOfFreedom', 'redundantConstraints', 'nODE2', 'nODE1', 'nAE', 'nPureAE', where: degreeOfFreedom = the system degree of freedom computed numerically, redundantConstraints=the number of redundant constraints, nODE2=number of ODE2 coordinates, nODE1=number of ODE1 coordinates, nAE=total number of constraints, nPureAE=number of constraints on algebraic variables (e.g., lambda=0) that are not coupled to ODE2 coordinates
#**notes: this approach could possibly fail with special constraints! Currently only works with dense matrices, thus it will be slow for larger systems
#**belongsTo: MainSystem
#**example:
# import exudyn as exu
# from exudyn.utilities import *
# import numpy as np
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# #
# b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, 
#                                                  sideLengths=[1,0.1,0.1]),
#                          referencePosition = [6,0,0],
#                          initialAngularVelocity = [0,8,0],
#                          gravity = [0,-9.81,0],
#                          graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1], 
#                                                                       color=graphics.color.orange)])
# oGround = mbs.AddObject(ObjectGround())
# mbs.CreateGenericJoint(bodyNumbers=[oGround, b0], position=[5.5,0,0],
#                        constrainedAxes=[1,1,1, 1,0,0],
#                        rotationMatrixAxes=RotationMatrixX(0.125*pi), #tilt axes
#                        useGlobalFrame=True, axesRadius=0.02, axesLength=0.2)
# #
# mbs.Assemble()
# dof = mbs.ComputeSystemDegreeOfFreedom(verbose=1)['degreeOfFreedom'] #print out details
def ComputeSystemDegreeOfFreedom(mbs, 
                simulationSettings = exudyn.SimulationSettings(),
                threshold = 1e-12, verbose=False, useSVD=False):
    #use static solver, as it does not include factors from time integration (and no velocity derivatives) in the jacobian
    
    store = DeactivateWritingOfSolvers(simulationSettings)

    staticSolver = exudyn.MainSolverStatic()
    #initialize solver with initial values
    staticSolver.InitializeSolver(mbs, simulationSettings)

    nODE2 = staticSolver.GetODE2size()
    nODE1 = staticSolver.GetODE1size()
    if nODE1 != 0:
        print('WARNING: ComputeSystemDegreeOfFreedom(...) not suitable in case of ODE1 coordinates; ODE1 coordinates will not count as additional DOF!')
    nODE12 = nODE2+nODE1
    nAE = staticSolver.GetAEsize()

    #reset jacobian    
    staticSolver.ComputeJacobianODE2RHS(mbs,scalarFactor_ODE2=-1,
                                        scalarFactor_ODE2_t=0, 
                                        scalarFactor_ODE1=0)    #compute ODE2 part of jacobian ==> stored internally in solver
    staticSolver.ComputeJacobianAE(mbs, scalarFactor_ODE2=1., scalarFactor_ODE2_t=0., scalarFactor_ODE1=1., velocityLevel=False)
    jacobian = staticSolver.GetSystemJacobian()#.round(20) #read out stored jacobian

    Code2 = jacobian[0:nODE2, nODE12:]    #C_q: constraint jacobian
    Cae = jacobian[nODE12:,nODE12:]       #C_lambda
    Call =  jacobian[:, nODE12:]
    
    staticSolver.FinalizeSolver(mbs, simulationSettings) #close files, etc.
    #restore old settings:
    RestoreSimulationSettings(simulationSettings, store)
    
    if verbose == 2:
        print('Code2=\n', Code2, sep='')
        print('Cae  =\n', Cae, sep='')

    if useSVD:
        try:
            from scipy.linalg import svdvals#, svd
        except:
            raise ValueError('ComputeSystemDegreeOfFreedom: missing scipy package; install with: pip install scipy')

    
    nRedundant = 0
    nPureAE = 0
    if nAE != 0:

        if useSVD:
            #s = svd(Call, compute_uv=False, full_matrices=False)#, lapack_driver='gesvd')
            s = svdvals(Call)
    
            if verbose:
                print('singular values ODE2+AE=',s)
    
            sAE = svdvals(Cae)
            #sAE = svd(Cae, compute_uv=False, full_matrices=False)#, lapack_driver='gesvd')
    
            if verbose:
                print('singular values pure AE=',sAE)
            
            nRedundant = (abs(s) <= threshold).sum() #counts True=1, False=0
            #these are constraints on AE coordinates, which do not count as constraints!
            nPureAE = (abs(sAE) > threshold).sum() #counts True=1, False=0
        else:
            nRedundant = nAE - np.linalg.matrix_rank(Call)
            nPureAE = np.linalg.matrix_rank(Cae)

    dof = nODE2-(nAE-nPureAE)+nRedundant
    if verbose:
        print('ODE2 coordinates          =',nODE2)
        print('total constraints         =',nAE)
        print('redundant constraints     =',nRedundant)
        print('pure algebraic constraints=',nPureAE)
        print('degree of freedom         =',dof)

    #return [dof, nRedundant, nODE2, nODE1, nAE, nPureAE] #old mode
    return {'degreeOfFreedom':dof, 
            'redundantConstraints':nRedundant, 
            'nODE2':nODE2, 'nODE1':nODE1, 
            'nAE':nAE, 'nPureAE':nPureAE}


    
#**function: helper function for solvers to check e.g. if high number of memory allocations happened during simulation
#            This can happen, if large amount of sensors are attached and output is written in every time step
#**input: stat=exudyn.InfoStat() from previous step, numberOfEvaluations is a counter which is proportional to number of RHS evaluations in method
def CheckSolverInfoStatistics(solverName, infoStat, numberOfEvaluations):
    stat = np.array(exudyn.InfoStat(False)) - np.array(infoStat)

    newCnt = max(stat[0],stat[2],stat[4]) #array, vector, matrix new counts

    if newCnt > solverCheckMemoryAllocationsThreshold and newCnt >= numberOfEvaluations:
        exudyn.Print("WARNING: "+solverName+" detected large amount ("+str(newCnt)+") of memory allocations, which seem to occur in every time step; this may be due to user functions; solver may be slow")

    #print("newcnt=", newCnt)

#%%+++++++++++++++++++
if __name__ == '__main__':
    import exudyn.demos as demos
    if True:
        [mbs, SC] = demos.Demo1(False)
        res = exudyn.ComputeSystemDegreeOfFreedom(mbs, verbose=True)
        print('Demo1 dof=',res['degreeOfFreedom'],'\n') #2 DOF (2D mass point freely moving)

    #takes some seconds to compute:
    if True:
        [mbs, SC] = demos.Demo2(False)
        res = exudyn.ComputeSystemDegreeOfFreedom(mbs)
        print('Demo2=',res,'\n') #12*3=36 (12 bodies with 12 spherical joints)
    
        res = exudyn.ComputeSystemDegreeOfFreedom(mbs, verbose=True, useSVD=True)
        print('Demo2 using SVD=',res) #12*3=36 (12 bodies with 12 spherical joints)


    #3D rigid-body with revolute joint and spring, same as in TestModels/ComputeODE2AEeigenvaluesTest.py
    #compared to analytical solution
    if True:
        from exudyn.utilities import *
        import numpy as np

        SC = exudyn.SystemContainer()
        mbs = SC.AddSystem()

        beamL=0.1 #in m
        beamW=0.01
        beamH=0.001
        rho=5000 #kg/m**3
        springL=0.02 #in m
        springK=1e1 #in N/m
    
        oGround = mbs.AddObject(ObjectGround())
    
        inertiaCuboid=InertiaCuboid(density=rho,
                                sideLengths=[beamL,beamH,beamW])
    
        bBeam = mbs.CreateRigidBody(inertia = inertiaCuboid,
                                referencePosition = [beamL*0.5,0,0],
                                gravity = [0,-9.81*0,0],
                                graphicsDataList = [exudyn.graphics.Brick(size=[beamL,beamH,beamW],
                                color=graphics.color.orange)])
        mBeamRight = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bBeam, localPosition=[beamL*0.5,0,0]))

        mbs.CreateGenericJoint(bodyNumbers= [oGround,bBeam], position= [0.,0.,0.], 
                                      rotationMatrixAxes= np.eye(3), constrainedAxes= [1,1,1,1,1,0], 
                                      axesRadius=0.001, axesLength= 0.01, color= exudyn.graphics.color.default)
    
        markerToConnect = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[beamL,-springL,0])) 
    
        mbs.AddObject(CartesianSpringDamper(markerNumbers=[markerToConnect,mBeamRight],
                                            stiffness=[0,springK,0],
                                            damping=[0,0,0],
                                            offset=[0,springL,0],
                                            visualization=VObjectConnectorCartesianSpringDamper(show=True,drawSize=0.01)
                                            ))    
        mbs.Assemble()
        [ew, ev] = mbs.ComputeODE2Eigenvalues()

        evNumerical = np.sqrt(ew[0]) / (2*np.pi)
        
        thetaZZ=inertiaCuboid.Translated([-beamL/2,0,0]).Inertia()[2,2]
        evAnalytical = np.sqrt( springK*beamL**2/thetaZZ ) / (2*np.pi)

        print('numerical eigenvalues in Hz:',evNumerical)
        print('analytical eigenvalues in Hz:',evAnalytical)
        print('error eigenvalues:', (evAnalytical-evNumerical)/evAnalytical)

