#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Data and information that support the findings of the article:
# M. Pieber, K. Ntarladima, R. Winkler and J. Gerstmayr, A Hybrid ALE 
# Formulation for the Investigation of the Stability of Pipes Conveying Fluid 
# and Axially Moving Beams. Journal of Computational and Nonlinear Dynamics
#
# Details:  Fig.12 Absolute position error for increasing number of elements
#           for cantilever pipes conveying fluid with the dimensionless velocity
#           u = 9, a moving mass factor fM = 0.2 and fM = 0.295 and for
#           a moving beam with the velocity vE = 9 m/s with a moving mass
#           factor fM = 1.
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

import sys

from exudyn.itemInterface import *
from exudyn.FEM import *
from exudyn.graphicsDataUtilities import *
from exudyn.processing import ParameterVariation

import exudyn as exu

import numpy as np
from scipy.linalg import eigh, eig #eigh for symmetric matrices, positive definite
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


fontSize=14

saveFigure=False

def ParameterFunction(parameterSet):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    nElements=16
    if 'nElements' in parameterSet:
        nElements = parameterSet['nElements']

    nElements=int(nElements)

        
    vAx=9
    if 'vAx' in parameterSet:
        vAx = parameterSet['vAx']
    vA=int(vAx)
        
    dVar=0.1   
    
    mDMF=0
    if 'mDMF' in parameterSet:
        mDMF = parameterSet['mDMF']

    iCalc = 'Ref' #needed for parallel computation ==> output files are different for every computation
    if 'computationIndex' in parameterSet:
        iCalc = str(parameterSet['computationIndex'])
        
        
        

    examplePipe=False #Cantilevered pipe conveying fluid
    if examplePipe:
        example1=False
        example2=False
        example3=False
    
    
    exampleBeam=True #fixed-fixed example
    if exampleBeam:
        example1=True
        example2=False
        example3=False
    
    
    ## choose static and/or dynamic simulation
    doStaticSimulation = True
    doDynamicSimulation = True #switch between only static or static and dynamic simulation
    
    
    ## display options, False for faster computation
    displayStaticSimulation=False 
    displayDynamicSimulation=False
    plotTipDisplacement=False
    
    plotVALE=False 
    
    
    
    
    eigenValues=False #if True, set doStaticSimulation=False and doDynamicSimulation=False
    # plotAdditionalValues=False
    
    if eigenValues==True:
        doStaticSimulation = False
        doDynamicSimulation = False
        displayStaticSimulation=False 
        displayDynamicSimulation=False
        plotTipDisplacement=False    
        plotVALE=False
        plotAdditionalValues=True
    

    tEnd = 15   #simulation end time
    h = 4e-5    #4e-4: relAcc = 1e-6; step size    
    
    if examplePipe:
        pinedpined=False
        fixedfixed=True
        
        useConstraints=True #constraints left side of the beam
        useConstraintsCC=False #constraints right side of the beam   
    
    
        useCoordinateSpringDamper=False #use damping for every node use this for Yang Example
        spring=0.
        damper=0.
        
        
    
        #parameters of the example    
        PI=10000
        beta=0.2 #=movingMassFactor
        
        L=1.                   # length of ANCF element in m 
        ma=10
        EI=10                  # Nm^2 
        
        d=np.sqrt(8/PI)        # width of a cylindrical ANCF element in m
        A=np.pi*d**2/4         # cross sectional area of ANCF element in m^2
        I=np.pi*d**4/32
        
        E=EI/I
        EA=E*A
        
        rho=ma/(A*L)              # density of ANCF element in kg/m^3
        rhoA = rho*A
        
        f=3*EI/L**2*0.1   # tip load applied to ANCF element in N   
        
        endTimevAle=2
        endTimeForce=6
        

        nrStepsInit=500*20 # for convergence analysis*10 #nr of simulation steps
        simTime=100 #15*3      # for convergence analysis*10 #simulation time
        secEnd=3    

        
        if example1: #vALEs=5 bis vA=9
            movingMassFactor = 0.2
            vALEs = 5 #offset to vALE
            nb=51
            
            if eigenValues:
                vALEs = 0 #offset to vALE
                rangeToPlot=16
                nb=1000
                vA=16
                plotAdditionalValues=True 
                
        if example2: #vALEs=7 bis vA=12
            movingMassFactor = 0.295
            vALEs = 5 #6 offset to vALE
            nb=51
            vA=9 #8.5
            
            
            if eigenValues:
                vALEs = 0 #offset to vALE
                rangeToPlot=16
                nb=2500
                vA=9
                plotAdditionalValues=True     
                
        if example3:
            movingMassFactor = 1
            vALEs = 5 #offset to vALE
            nb=2
            vA=28
            vALEs = 0 #offset to vALE
            if eigenValues:
                vALEs = 0 #offset to vALE
                rangeToPlot=35+1
                nb=7000
                vA=32
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

        
        L=1.                   # length of ANCF element in m
        k1=100.
        vf=0.8
        
        movingDiscreteMassFactor = 0 #this is the amount of moving discrete masses compared to total mass; movingDiscreteMassFactor = 0 #all mass attached to beam if no masses
        
        rhoA0=1     #beam + discrete masses
        totalMass = L*rhoA0 #total mass of beam + discrete masses
    
        rhoA=rhoA0*(1 - movingDiscreteMassFactor)      #mass per unit length for beam;
        EA=k1**2
        EI=vf**2
    
        f=3*EI/L**2     # tip load applied to ANCF element in N
    
    
        if example1:
            movingMassFactor = 1
            vALEs=0
            nb=2     
            
            if eigenValues:
                nb=2000
                vA=15
                
                rangeToPlot=vA+1
                plotAdditionalValues=False
         
        
                
        
        
        useCoordinateSpringDamper=True #use damping for every node use this for Yang Example
        spring=0.
          
        damper=1.5/(nElements) *10
        # +axialer Dämpfung gleicher Parameter
        
        
        nrStepsInit=500 *20 # for convergence analysis*10 #nr of simulation steps
        simTime=15 #15*3      # for convergence analysis*10 #simulation time
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
    
    
    
    for x in range(1): 
        for path in range(1):  
            if path==1:
                f=-f
            
            mbs.Reset()
            
            if examplePipe:
                xx=1/np.sqrt((ma*movingMassFactor)/EI)
            if exampleBeam:
                xx=1
            
            vALEs=0
            # xx=1
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

        

            fileName = 'solution/viconPoint'+iCalc+'.txt'
            fileName2 = 'solution/vALE'+iCalc+'.txt'
            nLoad=mbs.AddLoad(LoadForceVector(markerNumber=mANCFLast, loadVector=[0, 0, 0]))
            # nLoad=mbs.AddLoad(LoadForceVector(markerNumber=mANCFLast, loadVector=[0, 0, 0], loadVectorUserFunction=UFLoad))
            
            if eigenValues==False:
                if examplePipe:
                    mbs.AddSensor(SensorBody(bodyNumber=cableList[round(len(cableList)-1)], localPosition=[lElem,0.,0.], fileName=fileName, 
                                                outputVariableType=exu.OutputVariableType.Position))
                    
                if exampleBeam:
                    mbs.AddSensor(SensorBody(bodyNumber=cableList[int(0.25*L/(L/nElements))], localPosition=[0.25*L-L/nElements*int(0.25*L/(L/nElements)),0,0], fileName=fileName, 
                                                outputVariableType=exu.OutputVariableType.Position))
                
                mbs.AddSensor(SensorObject(objectNumber=oCC, fileName=fileName2,
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
                    # print(mbs.GetNodeParameter(nALE, 'initialCoordinates_t'))
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
            
                simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
                simulationSettings.timeIntegration.endTime = tEnd
                
                simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6 #10000
                simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-6
                simulationSettings.timeIntegration.verboseMode = 0
                #simulationSettings.timeIntegration.newton.useNumericalDifferentiation = True
                #simulationSettings.timeIntegration.newton.numericalDifferentiation.doSystemWideDifferentiation = True
                
                simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
                simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints =  simulationSettings.timeIntegration.generalizedAlpha.useNewmark
                simulationSettings.timeIntegration.newton.useModifiedNewton = True
                # simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
                simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.9
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
             
                data0 = np.loadtxt(fileName, comments='#', delimiter=',') 
                
                plt.close()
                if plotVALE:
                    data1 = np.loadtxt(fileName2, comments='#', delimiter=',') 
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
        
                utipDispMean=(tipDispMax+tipDispMin)/2
                tipDispMean+=[utipDispMean]
                tipDispMean+=[utipDispMean]


    import os
    if iCalc != 'Ref':
        os.remove(fileName) #remove files in order to clean up
        
    del mbs
    del SC
    
    return tipDisp




#%%++++++++++++++++++++++++++++++++++++
#single run:
if False:
    useGraphics = True
    vAx = 9# 5.5#2 masses 7.5, 1 mass 8
    nElements=64
    
    plt.close("all")
    
    ParameterFunction({'vAx':vAx, 'nElements':nElements})
    # plt.close()
    plt.figure()
    
    ax=plt.gca()
    ax.grid(True,'major','both')
    
    data0 = np.loadtxt('solution/beamALEmidPointRef.txt', comments='#', delimiter=',') 
    plt.plot(data0[:,0],data0[:,2],'b-',label='midPointDeflection')
    data0 = np.loadtxt('solution/beamALEmidPointRef.txt', comments='#', delimiter=',') 
    plt.plot(data0[:,0],data0[:,2],'b-',label='vALE'+str(vAx))
    
    plt.tight_layout()
                
    plt.legend()
    plt.show()


#%%++++++++++++++++++++++++++++++++++++
#parameter variation    
if True and __name__ == "__main__": #check if this is performed in main thread for multiprocessing
    import time

    vA=18
    n = 2
    
    start_time = time.time()
    [pDict, values] = ParameterVariation(parameterFunction=ParameterFunction, 
                                         parameters = {
                                                        'vAx':[9], #(0,vA,n),
                                                         # 'dVar':(0.01,0.2,5),
                                                         # 'mDMF':[0,0.5,0.75,0.9],
                                                        'nElements':[2,4,8,16,32,64],
                                                        # 'nElements':[2],
                                                       },
                                         debugMode=True,
                                         addComputationIndex=True,
                                         useMultiProcessing=True,
                                         showProgress=True,
                                         resultsFile='solution/parVarvAxmDMF.txt'
                                         )
    
    print("--- %s seconds ---" % (time.time() - start_time))


    nElements=list(pDict['nElements'])
    
    nElements = [2,4,8,16,32,64]
    results02 = [
        0.4603260197,
        0.4570239464,
        0.4289781352,
        0.4270065973,
        0.4268557337,
        0.4269002223
        ]    
    
    results0295 = [
        0.3497998724,
        0.2605572851,
        0.286037834,
        0.2880151541,
        0.288161501,
        0.2881991974
     ]
    
    results1 = [-0.02391558295,
             -0.0240099967,
             -0.02393790976,
             -0.02393126617,
             -0.02393038536,
             -0.02393011034]

    

    refSolution02 = results02[-1]
    refSolution0295 = results0295[-1]
    refSolution1 = results1[-1]
    
    errorSegments02 =  abs( np.array(results02) - refSolution02)
    errorSegments0295 = abs( np.array(results0295) - refSolution0295)
    errorSegments1 = abs( np.array(results1) - refSolution1)
    
    
    import matplotlib.pyplot as plt  
    fig_handle = plt.figure()
    fontSize=14
    plt.rc('font', size=fontSize) 
    plt.rc('axes', titlesize=fontSize)     # fontsize of the axes title
    plt.rc('axes', labelsize=fontSize)     # fontsize of the x and y labels
    plt.rc('xtick', labelsize=fontSize)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=fontSize)    # fontsize of the tick labels
    plt.rc('legend', fontsize=fontSize)    # legend fontsize
    
    n1=1
    n2=5
    
    plt.plot(nElements[n1:n2],results02[n1:n2],label='mmf=0.2')
    plt.plot(nElements[n1:n2],results0295[n1:n2],label='mmf=0.295')
    plt.plot(nElements[n1:n2],errorSegments1[n1:n2],label='mmf=1')

    
    
    plt.xlabel('N elements',fontsize=fontSize)
    plt.ylabel('y',fontsize=fontSize)
    plt.grid()
    plt.show()
    plt.legend()
    
    #plot error
    plt.figure()
    plt.plot(nElements[n1:n2],errorSegments02[n1:n2],label='$f_M$=0.2',marker='x')
    plt.plot(nElements[n1:n2],errorSegments0295[n1:n2],label='$f_M$=0.295',marker='x')
    plt.plot(nElements[n1:n2],errorSegments1[n1:n2],label='$f_M$=1',marker='x')

    plt.xlabel('$n_b$ elements',fontsize=fontSize)
    plt.ylabel('absolute error',fontsize=fontSize)
    plt.grid()
    plt.show()
    plt.legend()

    plt.semilogy(basey=10)
    
    plt.semilogx(basex=2)

    if saveFigure:
        plt.savefig('plots/convergenceElement.pdf',format='pdf')



