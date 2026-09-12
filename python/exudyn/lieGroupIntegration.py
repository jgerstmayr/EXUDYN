#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Lie group integration methods in Python; this is available for tests and research; 
#           these methods are integrated into the C++ kernel as a larger set of explicit Lie group methods 
#           and automatically used, if Lie group Nodes are used
#               
# Author:   Stefan Holzinger, Johannes Gerstmayr
# Date:     2020-09-11
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import exudyn as exu
import exudyn.lieGroupBasics as elg
import numpy as np


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#       LIE GROUP TIME INTEGRATION METHODS 
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


###############################################################################    
def ComputeStepWithRK1(ODE2RHS, v0, w0, h):
    w     = w0 + h*ODE2RHS(v0, w0)
    Omega = h*w
    v     = elg.CompositionRuleForRotationVectors(v0, Omega)
    return [v, w]

def ComputeStepWithRK1FromAcceleration(v0_t, v0, w0, h):
    w     = w0 + h*v0_t
    Omega = h*w
    v     = elg.CompositionRuleForRotationVectors(v0, Omega)
    return [v, w]

def ComputeStepWithRK4(ODE2RHS, v0, w0, h):
    
    # compute slope estimations
    k1 = h*ODE2RHS( v0, w0 )
    K1 = h*np.dot( elg.TExpSO3Inv(np.zeros(3)), w0 )
    
    k2 = h*ODE2RHS( elg.CompositionRuleForRotationVectors(v0, 0.5*K1), w0+0.5*k1 )
    K2 = h*np.dot( elg.TExpSO3Inv(0.5*K1), w0+0.5*k1 )
    
    k3 = h*ODE2RHS( elg.CompositionRuleForRotationVectors(v0, 0.5*K2), w0+0.5*k2 )
    K3 = h*np.dot( elg.TExpSO3Inv(0.5*K2), w0+0.5*k2 )
    
    #k4 = h*ODE2RHS( v0, w0+k3 )
    k4 = h*ODE2RHS( elg.CompositionRuleForRotationVectors(v0, K3), w0+k3 )
    K4 = h*np.dot( elg.TExpSO3Inv(K3), w0+k3 )
    
    # compute update
    w     = w0 + 1/6 * (k1 + 2*k2 + 2*k3 + k4)
    Omega = 1/6 * (K1 + 2*K2 + 2*K3 + K4)
    v     = elg.CompositionRuleForRotationVectors(v0, Omega)
    
    # return step solution
    return [v, w]

      
def RK_SolveEulersEOMWithProposedApproach(ODE2RHS, v0, w0, tEnd, numberOfSteps, solver):
    
    # choose solver
    if solver == "RK1":
        def ComputeStep(ODE2RHS, v0, w0, h):
            return ComputeStepWithRK1(ODE2RHS, v0, w0, h)
    elif solver == "RK4":
        def ComputeStep(ODE2RHS, v0, w0, h):
            return ComputeStepWithRK4(ODE2RHS, v0, w0, h)    
    else:
        exudyn.Print('requested solver is not available!!')
        
    # compute time step size
    h = tEnd/numberOfSteps
    
    # allocate memory
    t = np.zeros(numberOfSteps+1)
    v = np.zeros((numberOfSteps+1,3),dtype=float)
    w = np.zeros((numberOfSteps+1,3),dtype=float)
  
    # set initial conditions
    v[0] = v0
    w[0] = w0
    
    # time integration loop
    for i in range(numberOfSteps):
        
        # compute step i+1
        stepSolution = ComputeStep(ODE2RHS, v[i], w[i], h)
        
        # update solution data
        v[i+1] = stepSolution[0] # rotation vector update
        w[i+1] = stepSolution[1] # angular velocity update
        t[i+1] = (i+1)*h
        
    # solution data
    solutionData = {"time": t,
                    "rotationVector": v,
                    "angularVelocity": w                    
                    }
    
    # return solution data
    return solutionData 




#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Explicit Lie group integrator using solver user function
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#compute coordinates and LieGroup nodes lists for Explicit LieGroup RK
def LieGroupExplicitRKInitialize(mainSys):
    nObjects = mainSys.systemData.NumberOfObjects()
    nNodes = mainSys.systemData.NumberOfNodes()
#    lieGroupNodesList = []   #UNUSED #list of lie group nodes which need special treatment

    lieGroupODE2indices = []
    lieGroupReferenceRotations = [] #store reference values of nodes; need to be added to current values when computing rotation vectors
    constrainedToGroundCoordinatesList = [] #list of constrained (fixed) ODE2 coordinates
    constrainedCoordinatesList =[]  #list of pairs of constrained ODE2 coordinates
    

    for i in range(nNodes):
        d = mainSys.GetNode(i)
        if d['nodeType'] == 'RigidBodyRotVecLG':
#            lieGroupNodesList += [i]
            lieGroupODE2indices += [mainSys.GetNodeODE2Index(i)]
            lieGroupReferenceRotations += [d['referenceCoordinates'][3:6] ]
    
    for i in range(nObjects):
        d = mainSys.GetObject(i)
        if d['objectType'] == 'ConnectorCoordinate':
            if d['factorValue1'] != 1.: 
                exudyn.Print('ConnectorCoordinate.factorValue1 must be 1., otherwise connector constraint cannot be resolved!')
            elif d['offset'] != 0.: 
                exudyn.Print('ConnectorCoordinate.offset must be 0., otherwise connector constraint cannot be resolved!')
            elif d['activeConnector'] == True: #constrain only if connector is active!
                markers = d['markerNumbers']
                coords=[]
                for j in markers:
                    dm=mainSys.GetMarker(j)
                    if dm['markerType'] == 'NodeCoordinate': #only works for NodeCoordinate for now
                        nNode = dm['nodeNumber']
                        nCoordinate = dm['coordinate']
                        dn = mainSys.GetNode(nNode)
                        if dn['nodeType'] != 'PointGround':
                            coords+=[mainSys.GetNodeODE2Index(nNode)+nCoordinate]
                if len(coords) == 2:
                    constrainedCoordinatesList += [coords] #these global coordinates need to be fixed to each other
                else:
                    constrainedToGroundCoordinatesList += [coords[0]] #this global coordinate needs to be fixed

    mainSys.sys['lieGroupODE2indices'] = lieGroupODE2indices
    mainSys.sys['lieGroupReferenceRotations'] = lieGroupReferenceRotations
    
    mainSys.sys['constrainedToGroundCoordinatesList'] = constrainedToGroundCoordinatesList
    mainSys.sys['constrainedCoordinatesList'] = constrainedCoordinatesList
    
#compute accelerations for current state; used for explicit RK integrator
def ExplicitRKComputeSystemAcceleration(mainSolver, mainSys, nODE2):
    mainSolver.ComputeODE2RHS(mainSys)
    res = mainSolver.GetSystemResidual()
    Fode2 = res[0:nODE2]
    mainSolver.ComputeMassMatrix(mainSys)
    M = mainSolver.GetSystemMassMatrix()
    
    #adjust mass and force for constrained coordinates:
    #cCoords[0] is the relevant coordinate
    for cCoords in mainSys.sys['constrainedCoordinatesList']: #list of pairs of constrained ODE2 coordinates
        M[cCoords[0]][cCoords[0]] += M[cCoords[0]][cCoords[1]]
        M[cCoords[0]][cCoords[0]] += M[cCoords[1]][cCoords[0]]
        M[cCoords[0]][cCoords[0]] += M[cCoords[1]][cCoords[1]]

        Fode2[cCoords[0]] += Fode2[cCoords[1]]
    
    a0= np.linalg.solve(M,Fode2)[0:nODE2] #acceleration
    return a0

#computes global K for rotation vector update; used for explicit LieGroup RK integrator
#v0 is startOfStep global velocity vector
#Kprev is K of previous stage
def LieGroupComputeKstage(mainSys, u0, v0, h, nODE2, Kprev, kprev, factK):

    #K = np.zeros(nODE2)
    K = h*(v0[0:nODE2] + factK*kprev) #this is the correct displacement stage for non-LieGroup coordinates; LieGroup coordinates will be overwritten!
    for i in mainSys.sys['lieGroupODE2indices']:
        i1 = i+3 #start index of rotation
        i2 = i+6 #end index of rotation
        omega0 = v0[i1:i2]
        K0 = Kprev[i1:i2]
        k0 = kprev[i1:i2]
        K[i1:i2] = h*np.dot(elg.TExpSO3Inv(factK*K0), omega0 + factK*k0)
        
    return K

#apply coordinate constraints to updated system coordinates; used for explicit RK integrator
def ExplicitRKApplyCoordinateConstraints(mainSys):
    u = mainSys.systemData.GetODE2Coordinates()
    v = mainSys.systemData.GetODE2Coordinates_t()
    for i in mainSys.sys['constrainedToGroundCoordinatesList']: #list of constrained (fixed) ODE2 coordinates
        u[i] = 0 #offset could be considered here very easy
        v[i] = 0

    for cCoords in mainSys.sys['constrainedCoordinatesList']: #list of pairs of constrained ODE2 coordinates
        u[cCoords[1]] = u[cCoords[0]] #cCoords[0] coordinate has been considered in ODE2RHS computation and contains correct motion
        v[cCoords[1]] = v[cCoords[0]] 

    mainSys.systemData.SetODE2Coordinates(u)
    mainSys.systemData.SetODE2Coordinates_t(v)    


#update rotation vector and angular velocities for every Lie group node; used for explicit LieGroup RK integrator
def LieGroupUpdateStageSystemCoordinates(mainSys, u0, v0, nODE2, Kprev, kprev, factK):
#    u = mainSys.systemData.GetODE2Coordinates()
#    v = mainSys.systemData.GetODE2Coordinates_t()

    #RK stage update for non-LieGroup coordinates
    u = u0[0:nODE2] + factK * Kprev
    v = v0[0:nODE2] + factK * kprev

    #update rotation vector and angular velocities for every Lie group node: 
    cnt = 0
    for i in mainSys.sys['lieGroupODE2indices']:
        i1 = i+3 #start index of rotation
        i2 = i+6 #end index of rotation
        vecRef = mainSys.sys['lieGroupReferenceRotations'][cnt]
        vec0 = vecRef + u0[i1:i2]
        omega0 = v0[i1:i2]
        K = Kprev[i1:i2]
        k = kprev[i1:i2]
        u[i1:i2] = elg.CompositionRuleForRotationVectors(vec0, factK*K) - vecRef

        v[i1:i2] = omega0+factK*k #could be omitted
        cnt += 1

    mainSys.systemData.SetODE2Coordinates(u)
    mainSys.systemData.SetODE2Coordinates_t(v)    
    ExplicitRKApplyCoordinateConstraints(mainSys)

#user function for Newton to realize explicit RK4 solver with LieGroup integration
#USAGE:
#dynamicSolver = exu.MainSolverImplicitSecondOrder()
#dynamicSolver.SetUserFunctionNewton(mbs, UserFunctionNewtonLieGroupRK4)
#dynamicSolver.SolveSystem(mbs, simulationSettings)
def UserFunctionNewtonLieGroupRK4(mainSolver, mainSys, sims):

    h = mainSolver.it.currentStepSize
    tend = mainSys.systemData.GetTime() #end of step time
    t0 = mainSys.systemData.GetTime(exu.ConfigurationType.StartOfStep) #start of step time

    nODE2 = mainSolver.GetODE2size()
    #nAE = mainSolver.GetAEsize()
    #nSys = nODE2+nAE

    #+++++++++++++++++++++++++++++++++++++     
    u0 = mainSys.systemData.GetODE2Coordinates()
    #uRef=mainSys.systemData.GetODE2Coordinates(configuration = exu.ConfigurationType.Reference)
    v0 = mainSys.systemData.GetODE2Coordinates_t()
     
    #K0 = np.zeros(nODE2)
    mainSys.systemData.SetTime(t0)
    k1 = h*ExplicitRKComputeSystemAcceleration(mainSolver, mainSys, nODE2)#velocities
    #K1 = LieGroupComputeKstage(mainSys, u0, v0, h, nODE2, K0, K0, 0.)      #displacements; K0==k0; 
    K1 = h*v0[0:nODE2]

    mainSys.systemData.SetTime(t0 + 0.5*h)
    LieGroupUpdateStageSystemCoordinates(mainSys, u0, v0, nODE2, K1, k1, 0.5)
    k2 = h*ExplicitRKComputeSystemAcceleration(mainSolver, mainSys, nODE2)
    K2 = LieGroupComputeKstage(mainSys, u0, v0, h, nODE2, K1, k1, 0.5)
    
    mainSys.systemData.SetTime(t0 + 0.5*h)
    LieGroupUpdateStageSystemCoordinates(mainSys, u0, v0, nODE2, K2, k2, 0.5)
    k3 = h*ExplicitRKComputeSystemAcceleration(mainSolver, mainSys, nODE2)
    K3 = LieGroupComputeKstage(mainSys, u0, v0, h, nODE2, K2, k2, 0.5)
    
    mainSys.systemData.SetTime(t0 + h) #this is also the step end time
    LieGroupUpdateStageSystemCoordinates(mainSys, u0, v0, nODE2, K3, k3, 1.)
    k4 = h*ExplicitRKComputeSystemAcceleration(mainSolver, mainSys, nODE2)
    K4 = LieGroupComputeKstage(mainSys, u0, v0, h, nODE2, K3, k3, 1.)
    
    #++++++++++++++++++
    mainSys.systemData.SetTime(tend)
    # compute update for end of step
    #global update for velocities (same for all velocities!)
    vStep      = v0 + 1./6. * (k1 + 2*k2 + 2*k3 + k4)
    #incremental rotation vector, can be computed globally:
    deltaU     = 1./6. * (K1 + 2*K2 + 2*K3 + K4) #contains displacement updates and incremental velocity vectors
    
    uStep = u0 + deltaU #standard update for non-LieGroup nodes
    
    #compute final rotation vector updates based on deltaU:
    cnt = 0
    for i in mainSys.sys['lieGroupODE2indices']:
        i1 = i+3 #start index of rotation
        i2 = i+6 #end index of rotation
        vecRef = mainSys.sys['lieGroupReferenceRotations'][cnt]
        vec0 = vecRef + u0[i1:i2]
        #vec0 = u0[i1:i2]
        incrRotVec = deltaU[i1:i2]
        uStep[i1:i2] = elg.CompositionRuleForRotationVectors(vec0, incrRotVec) - vecRef
        cnt += 1
        
    mainSys.systemData.SetODE2Coordinates(uStep)
    mainSys.systemData.SetODE2Coordinates_t(vStep)

    ExplicitRKApplyCoordinateConstraints(mainSys)

    return True


