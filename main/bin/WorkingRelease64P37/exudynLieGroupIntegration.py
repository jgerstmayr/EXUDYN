#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Lie group update formulas for rotation vector
#           Refs.:  Holzinger S., Gerstmayr J.: Time integration of rigid bodies modelled with three rotation parameters, Multibody System Dynamics (2020)
#
#           Notation according to reference above!
#           v ... rotation vector
#           w ... angular velocity (local frame)  
#           phi ... rotation angle
#           n ... rotation axis
#
# Author:   Stefan Holzinger
# Date:     2020-06-02
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
from numpy import linalg as LA
import math

import exudyn as exu
from exudynRigidBodyUtilities import Skew

###############################################################################
# rotation vector update formulas
def ComposeRotationVectors(v0, Omega):
    # This function composes two rotation vectors v0 and Omega 
    # Reuslt: v = v0 circ Omega
    
    # compute rotation angle and rotation Axis of rotation vector v0
    phi0 = LA.norm(v0)                               # rotation angle 
    n0   = ComputeRotationAxisFromRotationVector(v0) # rotation axis
    
    # compute rotation angle of rotation vector Omega
    deltaPhi = LA.norm(Omega)
    
    # compute rotation angle of resulting rotation vector 
    phi = 2*np.arccos(np.cos(phi0/2)*np.cos(deltaPhi/2) - 0.5*np.dot(n0,Omega)*(np.sin(phi0/2)*Sinc(deltaPhi/2)))
    
    # np.arccos(x) returns NaN in case that x>1 or x < -1, both leading to sin(phi/2) == 0
    if math.isnan( phi ):
        phi = 0.
        
    # compute rotation axis of resulting rotation vector v
    I = np.diag([1, 1, 1])
    n0Tilde = Skew(n0)
    sinPhi2 = np.sin(phi/2)
    if sinPhi2 == 0.:
        nBar = np.array([0.0, 0.0, 0.0])
#    elif phi == 2*np.pi:
#        nBar = np.array([0.0, 0.0, 0.0]) 
    else:
        x1 = ( (np.sin(phi0/2)*np.cos(deltaPhi/2))/sinPhi2 )*n0
        x2 = np.dot((Sinc(deltaPhi/2)/(2*sinPhi2))*(I*np.cos(phi0/2) + np.sin(phi0/2)*n0Tilde), Omega)
        nBar = x1 + x2
    
    # compute resulting rotation vector v
    v = phi*nBar
    
    # return resulting rotation vector 
    return v


# compute rotation axis from given rotation vector
def ComputeRotationAxisFromRotationVector(rotationVector):
    
    # compute rotation angle
    rotationAngle = LA.norm(rotationVector)
    
    # compute rotation axis
    if rotationAngle == 0.0:
        rotationAxis = np.array([0.0, 0.0, 0.0])
    else:
        rotationAxis = (1/rotationAngle)*rotationVector
    
    # return rotation axis 
    return rotationAxis


# sinc function according to paper
def Sinc(x):
    return np.sinc(x/np.pi)
#    if x == 0:
#        s = 1
#    else:
#        s = np.sin(x)/x
#    return s



## inverse tangent operator corresponding to exponential map
#def TSO3Inv(Omega):
#    I = np.diag([1, 1, 1])
#    incrementalRotationAngle = LA.norm(Omega)
#    if incrementalRotationAngle == 0:
#        Omega_t = I
#    else:
#        OmegaTilde = Skew(Omega)
#        omegaNorm = LA.norm(Omega)
#        gamma      = 0.5*omegaNorm*cot( 0.5*omegaNorm )
#        phiSquared = omegaNorm*omegaNorm
#        Omega_t = I + 0.5*OmegaTilde + ((1-gamma)/phiSquared)*np.dot(OmegaTilde,OmegaTilde)
#    return Omega_t
#
# inverse tangent operator corresponding to exponential map
def TSO3Inv(Omega):
    I = np.diag([1, 1, 1])
    incrementalRotationAngle = LA.norm(Omega)
    if incrementalRotationAngle == 0:
        Omega_t = I
    else:
        OmegaTilde = Skew(Omega)
        omegaNorm = LA.norm(Omega)
        #gamma      = 0.5*omegaNorm*cot( 0.5*omegaNorm )
        gamma1 = 0
        if omegaNorm < 1e-1: #approximate 1-x/tan(x)
            x=0.5*omegaNorm
            gamma1 = x**2/3+x**4/45+x**6*2/945+x**8/4725
        else:
            gamma1 = 1-0.5*omegaNorm*cot( 0.5*omegaNorm )
        phiSquared = omegaNorm*omegaNorm
        Omega_t = I + 0.5*OmegaTilde + ((gamma1)/phiSquared)*np.dot(OmegaTilde,OmegaTilde)
    return Omega_t

#cot(x) = 1/x-1/3*x-1/45*x**3-2/945*x**5
    
               
def cot(x):
    # Cotangent of angle in radians
    return 1/np.tan(x)


###############################################################################    
def ComputeStepWithRK1(ODE2RHS, v0, w0, h):
    w     = w0 + h*ODE2RHS(v0, w0)
    Omega = h*w
    v     = ComposeRotationVectors(v0, Omega)
    return [v, w]

def ComputeStepWithRK1FromAcceleration(v0_t, v0, w0, h):
    w     = w0 + h*v0_t
    Omega = h*w
    v     = ComposeRotationVectors(v0, Omega)
    return [v, w]

def ComputeStepWithRK4(ODE2RHS, v0, w0, h):
    
    # compute slope estimations
    k1 = h*ODE2RHS( v0, w0 )
    K1 = h*np.dot( TSO3Inv(np.zeros(3)), w0 )
    
    k2 = h*ODE2RHS( ComposeRotationVectors(v0, 0.5*K1), w0+0.5*k1 )
    K2 = h*np.dot( TSO3Inv(0.5*K1), w0+0.5*k1 )
    
    k3 = h*ODE2RHS( ComposeRotationVectors(v0, 0.5*K2), w0+0.5*k2 )
    K3 = h*np.dot( TSO3Inv(0.5*K2), w0+0.5*k2 )
    
    #k4 = h*ODE2RHS( v0, w0+k3 )
    k4 = h*ODE2RHS( ComposeRotationVectors(v0, K3), w0+k3 )
    K4 = h*np.dot( TSO3Inv(K3), w0+k3 )
    
    # compute update
    w     = w0 + 1/6 * (k1 + 2*k2 + 2*k3 + k4)
    Omega = 1/6 * (K1 + 2*K2 + 2*K3 + K4)
    v     = ComposeRotationVectors(v0, Omega)
    
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
        print('wished solver is not available!!')
        
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
                print('ConnectorCoordinate.factorValue1 must be 1., otherwise connector constraint cannot be resolved!')
            elif d['offset'] != 0.: 
                print('ConnectorCoordinate.offset must be 0., otherwise connector constraint cannot be resolved!')
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
#    print("constrained coordinates =",constrainedToGroundCoordinatesList)
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
#    print("Fode2=",Fode2)
#    print("M=",np.diag(M))
    
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
        K[i1:i2] = h*np.dot(TSO3Inv(factK*K0), omega0 + factK*k0)
        
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
        u[i1:i2] = ComposeRotationVectors(vec0, factK*K) - vecRef
        #print("k=",k, ",factK=", factK, ",omega0=", omega0)
        v[i1:i2] = omega0+factK*k
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
    #print("tstart0=", t0)

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
        uStep[i1:i2] = ComposeRotationVectors(vec0, incrRotVec) - vecRef
        cnt += 1
        
    mainSys.systemData.SetODE2Coordinates(uStep)
    mainSys.systemData.SetODE2Coordinates_t(vStep)

    ExplicitRKApplyCoordinateConstraints(mainSys)

    return True


