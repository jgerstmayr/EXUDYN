#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Data and information that support the findings of the article:
# M. Pieber, K. Ntarladima, R. Winkler and J. Gerstmayr, A Hybrid ALE 
# Formulation for the Investigation of the Stability of Pipes Conveying Fluid 
# and Axially Moving Beams. Journal of Computational and Nonlinear Dynamics
#
# Details:  Fig.6 The eigenvalues of the lowest four modes of the straight 
#           cantilevered pipe conveying fluid as a function of the 
#           dimensionless flow velocity u (indicated with integer 
#           numbers), with a moving mass factor mf=0.2.
#
#           Fig.7 The eigenvalues of the lowest four modes of the straight 
#           cantilevered pipe conveying fluid as a function of the 
#           dimensionless flow velocity u (indicated with integer 
#           numbers), with a moving mass factor mf=0.295.
#
#           Fig.9 Bifurcation diagrams of the dimensionless tip-end 
#           displacements of a straight cantilevered pipe, obtained by the 
#           present model, for mf=0.2 and mf=0.295.
#
#           Fig.10 The eigenvalues of the lowest four modes of the moving beam 
#           with a fixed-fixed boundary condition as a function of the flow 
#           velocity vE (indicated with integer numbers in m/s), with all 
#           mass attached to the beam (mfd=0).
#
#           Fig.11 Bifurcation diagram for the fixed-fixed beam as a function
#           of vE compared with reference solution of Wickert [5]. Furthermore,
#           reduced masses per length of the beam rA are also investigated
#           which lead to higher critical velocities.
#           
#           
#           
# Author:   M. Pieber, K. Ntarladima, R. Winkler and J. Gerstmayr
# Date:     2022-01-04
# Testet with exudyn version:  1.1.71
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can 
#           redistribute it and/or modify it under the terms of the Exudyn 
#           license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
from exudyn.basicUtilities import ClearWorkspace
ClearWorkspace()

import pickle as pl

from exudyn.itemInterface import *
from exudyn.FEM import *
from exudyn.graphicsDataUtilities import *

import exudyn as exu

import numpy as np
from scipy.linalg import eigh, eig
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

plt.clf()
plt.rcParams['text.usetex'] = True
SC = exu.SystemContainer()
mbs = SC.AddSystem()
fontSize=14



examplePipe=False #set True for cantilevered pipe conveying fluid
if examplePipe:
    example1=False  #set True for Fig.6 and tip displacement for Fig.9
    example2=False #set True for Fig.7 and tip displacement for Fig.9


exampleBeam=True #set True for axially moving beams
if exampleBeam:
    example1=True #set True for Fig.10 and Fig.11
    example2=False
    example3=False

## choose static and/or dynamic simulation
doStaticSimulation = True
doDynamicSimulation = True #switch between only static or static and dynamic simulation

## display options, False for faster computation
displayStaticSimulation=True 
displayDynamicSimulation=True
plotTipDisplacement=True
plotVALE=False 


eigenValues=False #set True for eigenvalues (Fig.6, Fig.7, Fig.10); False for max. displacement (Fig.9)


saveSolutions = False # save solutions to file \plots
loadfig9 = False # plot Fig.9




if eigenValues==True:
    doStaticSimulation = False
    doDynamicSimulation = False
    displayStaticSimulation=False 
    displayDynamicSimulation=False
    plotTipDisplacement=False    
    plotVALE=False
    plotAdditionalValues=True


if examplePipe:
    pinedpined=False
    fixedfixed=True
    
    useConstraints=True #constraints left side of the beam
    useConstraintsCC=False #constraints right side of the beam   

    useCoordinateSpringDamper=False
    spring=0.
    damper=0.
 
    if example1:
        movingMassFactor=0.2
    if example2:
        movingMassFactor=0.295

    #parameters of the example    
    PI=10000
    L = 1  #length of ANCF element in m 
    m = 10 #kg
    
    d=np.sqrt((8*L**2)/PI) #width of a cylindrical ANCF element in m
    A=np.pi*d**2/4         #cross sectional area of ANCF element in m^2
    I=np.pi*d**4/32        #second moment of area m^4
    
    mges = m  / L # kg/m
    EI=  m*L**2*L # Nm^2
    
    mf=mges*movingMassFactor 
    mbeam=(1-movingMassFactor)/movingMassFactor * mf
    
    E=EI/I
    EA=E*A
    
    nElements = 16 #elements
    
    rho=mges/(A) #density of ANCF element in kg/m^3
    rhoA = rho*A #(nElements)
    
    f=3*EI/L**2 #tip load applied to ANCF element in N   
    
    endTimevAle=2
    endTimeForce=6
    
    nrStepsInit=500 #nr of simulation steps
    simTime=15  #simulation time
    secEnd=3
    
    if example1: #vALEs=5 bis vA=9
        vALEs = 5 #offset to vALE
        nb=51
        vA=9
        if eigenValues:
            vALEs = 0 #offset to vALE
            rangeToPlot=16
            nb=500
            vA=16
            plotAdditionalValues=True 
                        
    if example2: #vALEs=7 bis vA=12
        vALEs = 5 #offset to vALE
        nb = 51
        vA = 9
        simTime=50  #simulation time for system’s steady state
        if eigenValues:
            vALEs = 0 #offset to vALE
            rangeToPlot=16
            nb=2500
            vA=19
            plotAdditionalValues=True
            
            
if exampleBeam:
    if example1:
        pinedpined=False
        fixedfixed=True
    if example2:
        pinedpined=True
        fixedfixed=False
    useConstraints=True #constraints left side of the beam
    useConstraintsCC=True #constraints right side of the beam 
    
    endTimevAle=2
    endTimeForce=6
    
    PI=10000
    d=np.sqrt(8/PI)           #width of cylindrical element in m
    A=np.pi*d**2/4            #cross sectional area of element in m^2   
    
    nElements = 16
    
    L=1 #length of ANCF element in m
    rhoA0 = 1 / L**2 #beam + discrete masses

    k1=100.
    vf=0.8
    
    #Fig.11 f_DM: 0, 0.5, 0.75, 0.9
    movingDiscreteMassFactor = 0 #this is the amount of moving discrete masses compared to total mass; movingDiscreteMassFactor = 0 #all mass attached to beam if no masses
    
    totalMass = L*rhoA0 #total mass of beam + discrete masses

    rhoA=rhoA0*(1 - movingDiscreteMassFactor)  #mass per unit length for beam
    EA=k1**2 
    EI=vf**2 * L**2

    f=3*EI/L**2     #tip load applied to ANCF element in N

    if example1:
        movingMassFactor = 1
        vALEs=0
        nb=3 #51
        vA=16 #8
        if eigenValues:
            nb=1000
            vA=15           
            rangeToPlot=vA+1
            plotAdditionalValues=False
     

    useCoordinateSpringDamper=True #use damping for every node
    spring=0.
    
    damper=1.5/nElements
            
    nrStepsInit=1000 #nr of simulation steps
    simTime=15 #simulation time
    secEnd=3


nEigenfrequencies = 4
listFrequencies = []
cnt=0

utipDisp=[]
tipDisp=[]
tipDispMean=[]

rect = [-2.5,-2,2.5,1] #xmin,ymin,xmax,ymax
background0 = GraphicsDataRectangle(0, -0.1, 1, 0.1, color=[0.,0.,0.,1.]) #background
background1 = GraphicsDataRectangle(0, -0.05, 0.25, 0.05, color=[0.,0.,0.,1.]) #background



for x in range(nb): 
    for path in range(1): 
        
        if path==1:
            f=-f
        
        mbs.Reset()
        
        vAx=(vA-vALEs)/(nb-1)*x
        
        if examplePipe:
            xx=1/(L*np.sqrt((mges*movingMassFactor)/EI))
        if exampleBeam:
            xx=1/np.sqrt(rhoA)
        
        vALE=(vALEs+vAx)*xx #*2.25
        print(vALE/xx)
        vALE2=vALE
        
        nrSteps=int(round(nrStepsInit+nrStepsInit*abs(vALE)/xx))*10 #nr of simulation steps changes with vALE
        
        #++++++++++++++++++++++++++++++++++++++
        oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background0,background1])))
        #ground node
        nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
        mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action
        
        nodeList=[]  #for nodes of cable
        cableList=[]
        
        #++++++++++++++++++++++++++++++++++++++
        #create nodes and finite elements (ANCFCable2D)
        nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
        
        lElem = L / nElements
        nodeList+=[nc0]
        
        
        if eigenValues:
            setVALE=vALE
        else:
            setVALE=0
            
        nALE = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=1, referenceCoordinates=[0], initialCoordinates=[0], initialCoordinates_t=[vALE]))
        mALE = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nALE, coordinate=0)) #ALE velocity
        oCC=mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mALE], offset=0, velocityLevel = False)) # False for static computation
        
    
        for i in range(nElements):
            nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
            nodeList+=[nLast]
            elem=mbs.AddObject(ALECable2D(physicsLength=lElem, 
                                          physicsMassPerLength=rhoA, 
                                          physicsBendingStiffness=EI, 
                                          physicsAxialStiffness=EA, 
                                          physicsBendingDamping=0, 
                                          physicsAxialDamping=0, 
                                          physicsMovingMassFactor=movingMassFactor, 
                                          nodeNumbers=[nodeList[i],nodeList[i+1],nALE]))
            cableList+=[elem]
            
            if useCoordinateSpringDamper:            
                mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nLast, coordinate=0))
                mbs.AddObject(CoordinateSpringDamper(markerNumbers = [mGround , mANCF0], stiffness = spring, damping = damper))
                
                mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nLast, coordinate=1))
                mbs.AddObject(CoordinateSpringDamper(markerNumbers = [mGround , mANCF0], stiffness = spring, damping = damper))
        
        if useConstraints:
            mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=0))
            mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=1))
            mANCF2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=3))
                
            mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
            mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
            if fixedfixed:
                mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2]))
            
            if useConstraintsCC:
                mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nLast, coordinate=0))
                mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nLast, coordinate=1))
                mANCF2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nLast, coordinate=3))
                    
                mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
                mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
                if fixedfixed:
                    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2]))
        
        if examplePipe:
            mANCFLast = mbs.AddMarker(MarkerNodePosition(nodeNumber=round(len(cableList))+2)) #force
            
        if exampleBeam: 
            mANCFLast = mbs.AddMarker(MarkerBodyPosition(bodyNumber=cableList[int(0.25*L/(L/nElements))],localPosition = [0.25*L-L/nElements*int(0.25*L/(L/nElements)),0,0]))
  
        
        nLoad=mbs.AddLoad(LoadForceVector(markerNumber=mANCFLast, loadVector=[0, 0, 0]))
        
        if eigenValues==False:
            if examplePipe:
                mbs.AddSensor(SensorBody(bodyNumber=cableList[round(len(cableList)-1)], localPosition=[lElem,0.,0.], fileName='displacementPoint.txt', 
                                            outputVariableType=exu.OutputVariableType.Position))
                
            if exampleBeam:
                mbs.AddSensor(SensorBody(bodyNumber=cableList[int(0.25*L/(L/nElements))], localPosition=[0.25*L-L/nElements*int(0.25*L/(L/nElements)),0,0], fileName='displacementPoint.txt', 
                                            outputVariableType=exu.OutputVariableType.Position))
            
            mbs.AddSensor(SensorObject(objectNumber=oCC, fileName='vALE.txt',
                                       outputVariableType=exu.OutputVariableType.Velocity)) 

        mbs.Assemble()
    
        simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        
        simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-9
        simulationSettings.staticSolver.verboseMode = 0
        simulationSettings.staticSolver.newton.relativeTolerance = 1e-5 #10000
        simulationSettings.staticSolver.newton.absoluteTolerance = 1e-5
        simulationSettings.staticSolver.newton.maxIterations = 50


        if displayStaticSimulation: 
            exu.StartRenderer()
            mbs.WaitForUserToContinue()        
            
        if doStaticSimulation:
            nLoadSteps = 10
            if eigenValues:
                nLoadSteps=1
            for loadSteps in range(nLoadSteps):
          
                modeStatic=True
                modeDynamic=False
                
                loadValue = f*((loadSteps+1)/nLoadSteps)

                mbs.SetObjectParameter(oCC, 'offset', vALE)
                mbs.SetNodeParameter(nALE, 'initialCoordinates_t', [vALE])
                mbs.SetObjectParameter(oCC, 'velocityLevel', False)
                mbs.SetLoadParameter(nLoad, 'loadVector', [0,-loadValue,0]) 
                
                exu.SolveStatic(mbs, simulationSettings)
            
                sol = mbs.systemData.GetSystemState()
                mbs.systemData.SetSystemState(sol, configuration=exu.ConfigurationType.Initial) #set initial conditions for next step
                
            
            if displayStaticSimulation:  
                mbs.WaitForUserToContinue()
                exu.StopRenderer() #safely close rendering window!

        uList=[]    
        if doDynamicSimulation: #switch between static and dynamic simulation
            def UserFunctionLoad(t, load):
                t0 = 0      #start time
                dT = endTimeForce      #end time
                f0 = f      #initial force
                f1 = 0      #final force
                loadValue = f0
                if modeStatic:
                    loadValue = 0
                if modeDynamic:
                    loadValue = 0 
                    if t>t0:
                        if t < t0+dT:
                            loadValue = f0 + (f1-f0) * 0.5*(1-np.cos((t-t0)/dT*np.pi)) #t=[0 .. 25]
                        else:
                            loadValue = 0
                return [0, -loadValue, 0] 
            def UserFunctionDriveRefLen1(t, u, v, k, d, fa):
                t0 = 0 #5      #start time
                dT = endTimevAle #10      #end time
                v0 = 0      #initial length
                v1 = vALE    #final length
                vDrive = v0
                if modeStatic:
                    vDrive=v0
                if modeDynamic:        
                    if t>t0:
                        if t < t0+dT:
                            vDrive = v0 + (v1-v0) * 0.5*(1-np.cos((t-t0)/dT*np.pi)) #t=[0 .. 25]
                        else:
                            vDrive = v1               
                return vDrive
            
            #user function called at beginning of every time step
            def MBSUserFunction(mbs, t):
                driveLen = UserFunctionDriveRefLen1(t, 0, 0, 0, 0, 0)#*factor
                load=UserFunctionLoad(t, 0)
                
                mbs.SetObjectParameter(oCC, 'offset', driveLen)
                mbs.SetNodeParameter(nALE, 'initialCoordinates_t', [driveLen])
                mbs.SetObjectParameter(oCC, 'velocityLevel', True)
                mbs.SetLoadParameter(nLoad, 'loadVector', load)  
                return True #succeeded
            
            mbs.SetPreStepUserFunction(MBSUserFunction)    

            modeStatic=False
            modeDynamic=True    
    
            if displayDynamicSimulation:
                exu.StartRenderer()
                mbs.WaitForUserToContinue()
        
            simulationSettings.timeIntegration.numberOfSteps = nrSteps
            simulationSettings.timeIntegration.endTime = simTime
            simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
            simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-6
            simulationSettings.timeIntegration.verboseMode = 0
            
            simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
            simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints =  simulationSettings.timeIntegration.generalizedAlpha.useNewmark
            simulationSettings.timeIntegration.newton.useModifiedNewton = True
            simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
            simulationSettings.timeIntegration.adaptiveStep = True #disable adaptive step reduction
            ##############################################################
            # IMPORTANT!!!!!!!!!
            simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse #sparse solver !!!!!!!!!!!!!!!
            ##############################################################
            simulationSettings.displayStatistics = False
            
            success = exu.SolveDynamic(mbs, simulationSettings)

            uList += [mbs.GetNodeOutput(nLast, exu.OutputVariableType.Position)]
            
            if displayDynamicSimulation:
                SC.WaitForRenderEngineStopFlag()
                exu.StopRenderer() #safely close rendering window!        
        
            data0 = np.loadtxt('displacementPoint.txt', comments='#', delimiter=',') 
            
            plt.close()
            if plotVALE:
                data1 = np.loadtxt('vALE.txt', comments='#', delimiter=',') 
                plt.figure()
                plt.plot(data1[:,0],data1[:,1])
                plt.xlabel(r'time in s',fontsize=fontSize)
                plt.ylabel(r'velocity vALE',fontsize=fontSize)
                plt.show()
                plt.pause(0.5)        
                
            
            
            utipDisp+=[vALE2/xx]
            
            Tper=data0[-1,0]-secEnd
            ab=np.where(data0[:,0]>Tper)[0][0]  
            
            tipDispMax=max(data0[ab:-1,2])
            tipDispMin=min(data0[ab:-1,2])
                       
            tipDisp+=[tipDispMax]
    
            utipDisp+=[vALE2/xx]
            tipDisp+=[tipDispMin]
    
            utipDispMean=(tipDispMax+tipDispMin)/2
            tipDispMean+=[utipDispMean]
            tipDispMean+=[utipDispMean]
            
            if plotTipDisplacement:
                plt.figure()
                plt.plot(data0[:,0],data0[:,2])
                plt.plot(data0[ab:,0],data0[ab:,2])
                plt.xlabel(r'time in s',fontsize=fontSize)
                plt.ylabel(r'max displacement at L',fontsize=fontSize)
                plt.show()
                plt.pause(1)
            

        if eigenValues:
            simulationSettings = exu.SimulationSettings() #takes currently set values or default values
            
            staticSolver = exu.MainSolverStatic()       
            staticSolver.InitializeSolver(mbs, simulationSettings)
            
            
            nODE2 = staticSolver.GetODE2size()
            #compute mass matrix        
            staticSolver.ComputeMassMatrix(mbs, 1)#simulationSettings)
            m = staticSolver.GetSystemMassMatrix()
            
            #compute stiffness matrix    
            staticSolver.ComputeJacobianODE2RHS(mbs, 1)
            staticSolver.ComputeJacobianAE(mbs, 1) #not necessary
            K = staticSolver.GetSystemJacobian()    
            K2 = -K[0:nODE2,0:nODE2]
        
            #compute damping matrix    
            staticSolver.ComputeJacobianODE2RHS(mbs, 0) #reset jacobian, add 0*K
            staticSolver.ComputeJacobianAE(mbs, 0) #not necessary
            staticSolver.ComputeJacobianODE2RHS_t(mbs, 1) #add D to jacobian
            D = staticSolver.GetSystemJacobian()    
            D2 = -D[0:nODE2,0:nODE2]
        
            deleteLineAndRow = [0,1,3,4]
            
            if pinedpined and exampleBeam:
                deleteLineAndRow = [0,1,4,m.shape[0]-4,m.shape[0]-3] #pined-pined for ALE-elements
            
            if fixedfixed and exampleBeam:  
                deleteLineAndRow = [0,1,3,4,m.shape[0]-4,m.shape[0]-3,m.shape[0]-1] #fixed-fixed for ALE-elements            
            
            def deleteRowAndCollumnOfMatrix(i, M):
                M=np.delete(M,i,0)
                M=np.delete(M,i,1)
                return M
        
            deleteLineAndRow = np.sort(deleteLineAndRow)[::-1]
            for i in deleteLineAndRow:
                K2=deleteRowAndCollumnOfMatrix(i,K2)
                D2=deleteRowAndCollumnOfMatrix(i,D2)
                m=deleteRowAndCollumnOfMatrix(i,m)
         
            # A=[0 I, -M^-1 K ; -M^-1 K, -M^-1 D]; (lambda I - A)v = 0       
            AA=np.vstack([np.hstack([np.dot(-np.linalg.inv(m),D2),np.dot(-np.linalg.inv(m),K2)]),np.hstack([np.identity(len(K2)),np.diag([0],len(K2)-1)])])
            ew, ewv = eig(AA)


            # sort calculate and plot ew    
            idx = np.argsort(abs(ew))[::1]
            ewSort = ew[idx]           
        
            listFrequencies+=[vALE/xx]       
            ewSort=list(ewSort)
        
            newSort=ewSort
            
            newSort=[]
            for j in range(len(ewSort)):
                if ewSort[j].imag >=0:
                    newSort+=[ewSort[j]]
        
            if x==0:   
                listFrequencies+=list(np.array(newSort[0:nEigenfrequencies]))        
                
            if x>0:
                for i in range(nEigenfrequencies):              
                    found=0
                    ra=0.01
                    
                    while found != 1:
                        for j in range(len(newSort)):
                            if found==0 and newSort[j].imag > listFrequencies[len(listFrequencies)-nEigenfrequencies-1].imag-ra and newSort[j].real > listFrequencies[len(listFrequencies)-nEigenfrequencies-1].real-ra and newSort[j].imag < listFrequencies[len(listFrequencies)-nEigenfrequencies-1].imag+ra and newSort[j].real < listFrequencies[len(listFrequencies)-nEigenfrequencies-1].real+ra:
                                listFrequencies += [newSort[j]]
                                found=1  
                        ra=ra+0.1


if eigenValues==False:
    fig_handle = plt.figure()
    plt.rc('font', size=fontSize) 
    plt.rc('axes', titlesize=fontSize)    # fontsize of the axes title
    plt.rc('axes', labelsize=fontSize)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=fontSize)   # fontsize of the tick labels
    plt.rc('ytick', labelsize=fontSize)   # fontsize of the tick labels
    plt.rc('legend', fontsize=fontSize)   # legend fontsize
    
    ax=plt.gca()
    
    ax.grid(True,'major','both')
    
    plt.plot(utipDisp[::2],tipDisp[::2],color='black')
    plt.plot(utipDisp[1::2],tipDisp[1::2],color='black')
    
    plt.tight_layout()
    plt.xlabel(r'dimensionless flow velocity $\bar{u}$',fontsize=fontSize)
    if examplePipe:
        if example1:
            plt.axis([vALEs, vA, -0.6, 0.6])
        if example2:
            plt.axis([vALEs, vA, -0.6, 0.6])            
            
        plt.ylabel(r'displacement amplitude x=1 in m',fontsize=fontSize)
        
    if exampleBeam:
        plt.xticks( [0, 2, 4, 6, 8, 10]    )
        plt.ylabel(r'displacement amplitude x=0.25 in m',fontsize=fontSize)
        if pinedpined:
            plt.axis([0, 10, -0.05, 0.05])
        if fixedfixed:
            plt.axis([0, 10, -0.03, 0.03])

    if saveSolutions:
        if example1:
            pl.dump(fig_handle, open('plots/example1Pipe.pkl', 'wb'))  
        if example2:
            pl.dump(fig_handle, open('plots/example2Pipe.pkl', 'wb'))  
        
    plt.show()
    if saveSolutions:
        if example1:
            plt.savefig('plots/example1Pipe.pdf',format='pdf')
        if example2:
            plt.savefig('plots/example2Pipe.pdf',format='pdf')

    print('tipDisp=',tipDisp)


if exampleBeam and eigenValues==False:
   
    
    if fixedfixed: #fixed-fixed analytical solution
        m=1
        x=0.25
        ux=[]
        for gamma in utipDisp[::2]:
            if (gamma**2.-1.)/(2.*m*np.pi)**2-vf**2 <= 0:
                ux+=[0]
                ux+=[0]
            else:
                uxS=2./k1*np.sqrt((gamma**2-1.)/(2.*m*np.pi)**2-vf**2)
                ux+=[uxS]
                ux+=[-uxS]
        plt.scatter(utipDisp,ux,color='blue', marker='o',label='Wickert')
        
        
        uxp=[]
        uxn=[]
        vx = np.linspace(0, vA, 100)
        for gamma in vx:
            if (gamma**2.-1.)/(2.*m*np.pi)**2-vf**2 <= 0:
                uxp+=[0]
                uxn+=[0]
            else:
                uxS=2./k1*np.sqrt((gamma**2-1.)/(2.*m*np.pi)**2-vf**2)
                uxp+=[uxS]
                uxn+=[-uxS]
                
        plt.plot(vx,uxp,color='blue')
        plt.plot(vx,uxn,color='blue')
        plt.legend()
    
    
if eigenValues:
    uList=[]
    eig1List=[]
    eig2List=[]
    eig3List=[]
    eig4List=[]
    
    uList=listFrequencies[0::5]
    eig1List=listFrequencies[1::5]
    eig2List=listFrequencies[2::5]
    eig3List=listFrequencies[3::5]
    eig4List=listFrequencies[4::5]
    
    plt.plot(np.array(eig1List).imag,np.array(eig1List).real)
    plt.plot(np.array(eig2List).imag,np.array(eig2List).real)
    plt.plot(np.array(eig3List).imag,np.array(eig3List).real)
    plt.plot(np.array(eig4List).imag,np.array(eig4List).real)
    
    offsetText=0
    fontSize = 14
    cnt=0
    for j in range(rangeToPlot):
        findFirst=0
        for i in range(len(uList)):
            if np.round(uList[i],1) >= j and findFirst==0:
                X = [eig1List[i].real]
                Y = [eig1List[i].imag]            
                plt.scatter(Y,X,color='black', marker='x')
                if i>0:
                    plt.annotate(r''+str(j),xy=(eig1List[i].imag,eig1List[i].real),xytext=(eig1List[i].imag+offsetText,eig1List[i].real+offsetText),fontsize=fontSize)
                findFirst=1
                
    cnt=0
    for j in range(rangeToPlot):
        findFirst=0
        for i in range(len(uList)):
            if np.round(uList[i],1) >= j and findFirst==0:
                X = [eig2List[i].real]
                Y = [eig2List[i].imag]            
                plt.scatter(Y,X,color='black', marker='x')
                if i>0:
                    plt.annotate(r''+str(j),xy=(eig2List[i].imag,eig2List[i].real+offsetText),xytext=(eig2List[i].imag,eig2List[i].real+offsetText),fontsize=fontSize)
                findFirst=1           
    
    cnt=0
    for j in range(rangeToPlot):
        findFirst=0
        for i in range(len(uList)):
            if np.round(uList[i],1) >= j and findFirst==0:
                X = [eig3List[i].real]
                Y = [eig3List[i].imag]            
                plt.scatter(Y,X,color='black', marker='x')
                if i>0:
                    plt.annotate(r''+str(j),xy=(eig3List[i].imag,eig3List[i].real+offsetText),xytext=(eig3List[i].imag,eig3List[i].real+offsetText),fontsize=fontSize)
                findFirst=1
    
    
    cnt=0
    for j in range(rangeToPlot):
        findFirst=0
        for i in range(len(uList)):
            if np.round(uList[i],1) >= j and findFirst==0:
                X = [eig4List[i].real]
                Y = [eig4List[i].imag]            
                plt.scatter(Y,X,color='black', marker='x')
                if i>0:
                    plt.annotate(r''+str(j),xy=(eig4List[i].imag,eig4List[i].real+offsetText),xytext=(eig4List[i].imag,eig4List[i].real+offsetText),fontsize=fontSize)
                findFirst=1  
    
    
    
    if plotAdditionalValues:
        if examplePipe==True:
            if example1:
                j=5.6
                findFirst=0
                for i in range(len(uList)):
                    if np.round(uList[i],1) >= j and findFirst==0:
                        X = [eig2List[i].real]
                        Y = [eig2List[i].imag]            
                        plt.scatter(Y,X,color='blue', marker='x')
                        plt.annotate(r''+str(j),xy=(eig2List[i].imag,eig2List[i].real+offsetText),xytext=(eig2List[i].imag,eig2List[i].real+offsetText),fontsize=fontSize)
                        findFirst=1
                
                j=15.5
                findFirst=0
                for i in range(len(uList)):
                    if np.round(uList[i],1) >= j and findFirst==0:
                        X = [eig4List[i].real]
                        Y = [eig4List[i].imag]            
                        plt.scatter(Y,X,color='black', marker='x')
                        plt.annotate(r''+str(j),xy=(eig4List[i].imag,eig4List[i].real+offsetText),xytext=(eig4List[i].imag,eig4List[i].real+offsetText),fontsize=fontSize)
                        findFirst=1
                           
            if example2:
                j=16
                findFirst=0
                for i in range(len(uList)):
                    if np.round(uList[i],1) >= j and findFirst==0:
                        X = [eig4List[i].real]
                        Y = [eig4List[i].imag]            
                        plt.scatter(Y,X,color='black', marker='x')
                        plt.annotate(r''+str(j),xy=(eig4List[i].imag,eig4List[i].real+offsetText),xytext=(eig4List[i].imag,eig4List[i].real+offsetText),fontsize=fontSize)
                        findFirst=1
                j=17
                findFirst=0
                for i in range(len(uList)):
                    if np.round(uList[i],1) >= j and findFirst==0:
                        X = [eig4List[i].real]
                        Y = [eig4List[i].imag]            
                        plt.scatter(Y,X,color='black', marker='x')
                        plt.annotate(r''+str(j),xy=(eig4List[i].imag,eig4List[i].real+offsetText),xytext=(eig4List[i].imag,eig4List[i].real+offsetText),fontsize=fontSize)
                        findFirst=1
        

        if exampleBeam==True:
                j=5.1
                findFirst=0
                if example1:
                    for i in range(len(uList)):
                        if np.round(uList[i],2) >= j and findFirst==0:
                            X = [eig1List[i].real]
                            Y = [eig1List[i].imag]            
                            plt.scatter(Y,X,color='blue', marker='x')
                            plt.annotate(r''+str(j),xy=(eig1List[i].imag,eig1List[i].real+offsetText),xytext=(eig1List[i].imag,eig1List[i].real+offsetText),fontsize=fontSize)
                            findFirst=1
                        
                        
                if example2:     
                    j=2.5
                    findFirst=0
                    for i in range(len(uList)):
                        if np.round(uList[i],2) >= j and findFirst==0:
                            X = [eig1List[i].real]
                            Y = [eig1List[i].imag]            
                            plt.scatter(Y,X,color='blue', marker='x')
                            plt.annotate(r''+str(j),xy=(eig1List[i].imag,eig1List[i].real+offsetText),xytext=(eig1List[i].imag,eig1List[i].real+offsetText),fontsize=fontSize)
                            findFirst=1                           
    
    ## plots
    plt.rc('font', size=fontSize) 
    plt.rc('axes', titlesize=fontSize)     # fontsize of the axes title
    plt.rc('axes', labelsize=fontSize)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=fontSize)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=fontSize)    # fontsize of the tick labels
    plt.rc('legend', fontsize=fontSize)    # legend fontsize
    
    ax=plt.gca()
    
    ax.grid(True,'major','both')
    plt.xticks( [0, 20, 40, 60, 80, 100, 120, 140,160])
    
        
    plt.tight_layout()
    plt.xlabel(r'Im($\omega$)',fontsize=fontSize)
    plt.ylabel(r'Re($\omega$)',fontsize=fontSize)
    if example2:
        plt.axis([0, 125, -35, 20])
    else:
        plt.axis([0, 125, -30, 20])
        
    if exampleBeam:
        if eigenValues:
            plt.axis([-0.1, 100, -20, 20])
        
    plt.show()    

    if saveSolutions:
        if example1 and examplePipe:
            plt.savefig('plots/example1PipeEigenvalues.pdf',format='pdf')
        if example2 and examplePipe:
            plt.savefig('plots/example2PipeEigenvalues.pdf',format='pdf')   
        if example1 and exampleBeam:
            plt.savefig('plots/example1BeamEigenvalues.pdf',format='pdf')      
    
if eigenValues: 
    omegaNum = np.zeros(nEigenfrequencies)
    for i in range(nEigenfrequencies):
        omegaNum[i]=abs(np.array(listFrequencies[i+1]))
        
    if examplePipe:   
        print('fixed-free f numerical =',omegaNum/(2*np.pi),' in Hz')
    if exampleBeam: 
        print('fixed-fixed (free-free) f numerical =',omegaNum/(2*np.pi),' in Hz')

        #analytical: bending eigenfrequency of free-free/fixed-fixed beam:
        beta = [4.730040744862704, 7.853204624095838, 10.99560783800167, 14.13716549125746, 17.27875965739948, 20.42035224562606, 23.56194490204046, 26.70353755550819, 29.84513020910325]
        omega = np.zeros(4)
        for i in range(4):
            omega[i] = ((beta[i]/L)**4 * (EI/rhoA))**0.5
        print('omega analytical fixed-fixed (free-free) =',omega/(2*np.pi),' in Hz')
    
    
    if examplePipe:
        #analytical: bending eigenfrequency of fixed-free beam:
        beta = [1.875, 4.694, 7.855, 10.99557429, 14.13716694, 17.27875959, 20.42035225, 23.5619449, 26.70353756]
        omega = np.zeros(4)
        for i in range(4):
            omega[i] = ((beta[i]/L)**4 * (EI/rhoA))**0.5
        
        print('fixed-free f analytical =',omega/(2*np.pi) ,' in Hz')    
    
    
    
    relErr=(omegaNum-omega)/omega*100
    print('relative error=',relErr,'in %')
    
    
if loadfig9==True:
    # Load figure from disk and display
    fig_handle = pl.load(open('plots/example1Pipe.pkl','rb'))
    fig_handle2 = pl.load(open('plots/example2Pipe.pkl','rb'))
    
    
    
    plt1=fig_handle.axes[0].lines[0].get_data()
    plt2=fig_handle.axes[0].lines[1].get_data()
    plt3=fig_handle2.axes[0].lines[0].get_data()
    plt4=fig_handle2.axes[0].lines[1].get_data()
    
    
    fontSize=14
    fig_handle3 = plt.figure()
    plt.rc('font', size=fontSize) 
    plt.rc('axes', titlesize=fontSize)    #fontsize of the axes title
    plt.rc('axes', labelsize=fontSize)    #fontsize of the x and y labels
    plt.rc('xtick', labelsize=fontSize)   #fontsize of the tick labels
    plt.rc('ytick', labelsize=fontSize)   #fontsize of the tick labels
    plt.rc('legend', fontsize=fontSize)   #legend fontsize
    
    
    ax=plt.gca()
    ax.grid(True,'major','both')
    
    plt.plot(plt1[0],plt1[1],color='black',label='$f_{M}=0.2$')
    plt.plot(plt2[0],plt2[1],color='black')
    
    plt.plot(plt3[0],plt3[1],'r--',label='$f_{M}=0.295$')
    plt.plot(plt4[0],plt4[1],'r--')
    plt.axis([5, 9, -0.6, 0.6])
    
    
    
    plt.tight_layout()
    plt.xlabel(r'dimensionless flow velocity $\bar{u}$',fontsize=fontSize)
    plt.ylabel(r'displacement amplitude at $x=L_B$ in m',fontsize=fontSize)
    
    plt.legend()
    plt.legend(loc='upper left')   
    if saveSolutions:
        plt.savefig('plots/example1u2Pipe.pdf',format='pdf')