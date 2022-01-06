#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Data and information that support the findings of the article:
# M. Pieber, K. Ntarladima, R. Winkler and J. Gerstmayr, A Hybrid ALE 
# Formulation for the Investigation of the Stability of Pipes Conveying Fluid 
# and Axially Moving Beams. Journal of Computational and Nonlinear Dynamics
#
# Details:  ANCF ALE with moving continuous and discrete masses
#
# Author:   Johannes Gerstmayr, Michael Pieber
# Date:     2022-01-04
# Testet with exudyn version:  1.1.71
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from exudyn.basicUtilities import ClearWorkspace
ClearWorkspace()

from exudyn.basicUtilities import *
import exudyn as exu
from exudyn.processing import ParameterVariation


import os


from exudyn.itemInterface import *
from exudyn.FEM import *
from exudyn.graphicsDataUtilities import *
from exudyn.utilities import *

import numpy as np
from scipy.linalg import eigh, eig #eigh for symmetric matrices, positive definite
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

#plt.clear('all')
#plt.rcParams['text.usetex'] = True #slows down figures


fontSize=14

#%%++++++++++++++++++++++++++++++++++++++++
useGraphics = False
plotResults=False
tEvaluate=12        #6; time for evaluation of maximum
tTerminateForce=4  #4 ; time when static force is set to zero
tEnd = tTerminateForce*2+tEvaluate   #simulation end time
oCCvALE = 0 #coordinate constraint object for vALE

rect = [-2.5,-2,2.5,1] #xmin,ymin,xmax,ymax
background0 = GraphicsDataRectangle(0, -0.1, 1, 0.1, color=[0.,0.,0.,1.]) #background
background1 = GraphicsDataRectangle(0, -0.05, 0.25, 0.05, color=[0.,0.,0.,1.]) #background


#user function called at beginning of every time step
def MBSUserFunction(mbs, t):
    vALE = SmoothStep(t, 0, tTerminateForce*0.5, 0, mbs.variables['setVALE'])#*factor
    #print(vALE,t,setVALE)
    
    mbs.SetObjectParameter(oCCvALE, 'offset', vALE)
    mbs.SetObjectParameter(oCCvALE, 'velocityLevel', True)

    # for cc in mbs.variables['ccMassList']:
    #     mbs.SetObjectParameter(cc, 'offset', vALE)
    #     mbs.SetObjectParameter(cc, 'velocityLevel', True)

    if True:
        nALE = mbs.variables['nALE']
        sALE = mbs.GetNodeOutput(nALE, exu.OutputVariableType.Coordinates)
        #print("s=",sALE)
        cnt = 0
        for sjl in mbs.variables['slidingJointList']:
            sj = sjl[0] #object ALE sliding joint
            #print(t, ": sj=", mbs.GetObjectOutput(sj, exu.OutputVariableType.SlidingCoordinate))
            #print(mbs.GetObject(sj))
 
            #check if mass shall be started:
            #print("sALE=",sALE, ", sOff=", sjl[3])
            if sALE+1e-10>=sjl[3]:
                if not mbs.GetObjectParameter(sj, 'activeConnector'): #only for first time!
                    #print("t=", t, "switch on connector ",cnt, ", offset=",-sjl[3])
                    mbs.SetObjectParameter(sj, 'slidingOffset', -sjl[3])
                    mbs.SetObjectParameter(sj, 'activeConnector', True)

           
            sOff = mbs.GetObjectParameter(sj, 'slidingOffset') #with this value, it is set to x=0
            s = mbs.GetObjectOutput(sj, exu.OutputVariableType.SlidingCoordinate)
            # print(str(cnt)+": t=", t, ", s=", s, ", sOff"+"=",sOff)

            # nData = mbs.GetObjectParameter(sj, 'nodeNumbers')[0] #node 0 is the GenericNode having the sliding offset
            #print("data=", dataCoords[sjl[2]])
            #print("data=", dataCoords)
            #print("sOff=", dataCoords[nData])
            sEnd = mbs.variables['L']
            if s > sEnd:
                #adjust sliding offset:
                oldS = mbs.GetObjectParameter(sj, 'slidingOffset')#mbs.variables['L']) #with this value, it is set to x=0
                mbs.SetObjectParameter(sj, 'slidingOffset', sOff-s)#mbs.variables['L']) #with this value, it is set to x=0
                #print(cnt,": oldS=", oldS, ", newS=", sOff-s)
                #print("vALE=",vALE)

                #adjust sliding marker:
                if True:
                    dataCoords = mbs.systemData.GetDataCoordinates()
                    dataCoords[sjl[2]] = 0 #reset the slidingMarkerOffset (index to sliding marker)
                    mbs.systemData.SetDataCoordinates(dataCoords) #reset current local sliding marker index
                    mbs.systemData.SetDataCoordinates(dataCoords, exu.ConfigurationType.StartOfStep) #reset current local sliding marker index
                    currentMarkers = mbs.GetObjectParameter(sj, 'markerNumbers') #[1]==cable marker, needs to be reset
                    slidingMarkerNumbers = mbs.GetObjectParameter(sj, 'slidingMarkerNumbers') #[1]==cable marker, needs to be reset
                    currentMarkers[1] = exu.MarkerIndex(slidingMarkerNumbers[0]) #go to first marker
                    mbs.SetObjectParameter(sj, 'markerNumbers',currentMarkers)
                    #print("current marker=", mbs.GetObjectParameter(sj, 'markerNumbers'))
                    #print("slidingMarkerNumbers=", slidingMarkerNumbers)
                    mbs.AssembleLTGLists()

                #reset sliding markers to start value:
                # mSliding = mbs.GetObjectParameter(sj, 'slidingMarkerNumbers')[0] #first sliding marker
                # markerNumbers = mbs.GetObjectParameter(sj, 'markerNumbers') #first sliding marker
                # markerNumbers[0] = exu.MarkerIndex(mSliding)
                # mbs.SetObjectParameter(sj, 'markerNumbers', markerNumbers) #type needs to be marker index

                #reset x-coordinate of mass:
                nNode = sjl[1] #node number of sliding mass
                indexODE2 = mbs.GetNodeODE2Index(nNode)

                ODE2coords = mbs.systemData.GetODE2Coordinates()
                ODE2coords_t = mbs.systemData.GetODE2Coordinates_t()
                ODE2coords_tt = mbs.systemData.GetODE2Coordinates_tt()
                #print(str(cnt)+": t=", t, ", s=", s, ", sOff"+"=",sOff)
                #print("  x"+str(cnt)+"=", ODE2coords[indexODE2])
                ODE2coords[indexODE2+0] = 0
                ODE2coords[indexODE2+1] = 0
                #ODE2coords_t[indexODE2+0] = 0 #should stay constant
                ODE2coords_t[indexODE2+1] = 0
                ODE2coords_tt[indexODE2+0] = 0
                ODE2coords_tt[indexODE2+1] = 0

                mbs.systemData.SetODE2Coordinates(ODE2coords)
                mbs.systemData.SetODE2Coordinates(ODE2coords, exu.ConfigurationType.StartOfStep)
                mbs.systemData.SetODE2Coordinates_t(ODE2coords_t)
                mbs.systemData.SetODE2Coordinates_t(ODE2coords_t, exu.ConfigurationType.StartOfStep)
                mbs.systemData.SetODE2Coordinates_tt(ODE2coords_tt)
                mbs.systemData.SetODE2Coordinates_tt(ODE2coords_tt, exu.ConfigurationType.StartOfStep)
                #ODE2coords_tt2 = mbs.systemData.GetODE2Coordinates_tt()
                #print("ode2_tt=",ODE2coords_tt2[indexODE2:indexODE2+1])

            cnt+=1        
    return True #succeeded


#smooth function for loading in y-direction
def UFLoad(mbs, t, load):
    #without SolveStatic: return [0, SmoothStep(t,0,0.5*tTerminateForce,0,1)*SmoothStep(t, 0.5*tTerminateForce, tTerminateForce, load[1], 0), 0]
    return [0, SmoothStep(t, 0.5*tTerminateForce, tTerminateForce, load[1], 0), 0]


#build parameterized model for parameter variation
def BuildModel(parameterSet):
#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# if True:
#     parameterSet={}
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    #++++++++++++++++++++++++++++++++++
    #initialize variables        
    vALE0=0
    if 'vALE0' in parameterSet:
        vALE0 = parameterSet['vALE0']

    iCalc = 'Ref' #needed for parallel computation ==> output files are different for every computation
    if 'computationIndex' in parameterSet:
        iCalc = str(parameterSet['computationIndex'])
        
    nElements = 9 #16 
    if 'nElements' in parameterSet:
        nElements = int(parameterSet['nElements'])

    damper=0.1 #0.1: standard for parameter variation; 0.001: almost no damping, but solution is still oscillating at evaluation period
    if 'damping' in parameterSet:
        damper = (parameterSet['damping'])

    nMasses = 0 #12
    if 'nMasses' in parameterSet:
        nm = int(parameterSet['nMasses'])
        massList=[0,1,2,3,4,8,12,16,20,24,30]
        nMasses=massList[nm]
    nMasses = 16    

    movingDiscreteMassFactor = 0.9 #this is the amount of moving discrete masses compared to total mass
    if nMasses == 0:
        movingDiscreteMassFactor = 0 #all mass attached to beam if no masses
    
    if 'movingDiscreteMassFactor' in parameterSet:
        movingDiscreteMassFactor = parameterSet['movingDiscreteMassFactor']
    #++++++++++++++++++++++++++++++++++
    #movingDiscreteMassFactor = 0.25
    
       
    k1=100.
    vf=0.8

    L=1.        #length of ANCF element in m    
    mbs.variables['L'] = L
    P=1
    rhoA0=1     #beam + discrete masses
    totalMass = L*rhoA0 #total mass of beam + discrete masses

    rhoA=rhoA0*(1 - movingDiscreteMassFactor)      #mass per unit length for beam;
    EA=k1**2
    EI=vf**2
    
    f=3*EI/L**2     # tip load applied to ANCF element in N

    movingMassFactor = 1 #factor for beam;1=axially moving beam, <1: pipe
    mbs.variables['setVALE'] = vALE0   #start value for vALE
    # nb=21       #number of steps
    # vA=10       #final value for vALE

    useCoordinateSpringDamper=True #use damping for every node use this for Yang Example
    

    h = 4e-4    #4e-4: relAcc = 1e-6; step size
    if 'h' in parameterSet:
        h = parameterSet['h']
    
    # #additional bending and axial damping
    bendingDamping=0 # for ALE Element
    axialDamping=0 # for ALE Element
    
    #generate coordinate marker
    nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
    mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action
    
    #++++++++++++++++++++++++++++++++++++++++
    #create ALE node
    #print("setVALE=",mbs.variables['setVALE'])
    nALE = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=1, referenceCoordinates=[0], 
                                       initialCoordinates=[0], initialCoordinates_t=[0* mbs.variables['setVALE']]))
    mALE = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nALE, coordinate=0)) #ALE velocity
    mbs.variables['nALE'] = nALE
    
    if useGraphics:
        mbs.variables['sALEpos'] = mbs.AddSensor(SensorNode(nodeNumber=nALE, fileName='solution/nodeALEpos.txt',
                                 outputVariableType=exu.OutputVariableType.Coordinates))
        mbs.variables['sALEvel'] = mbs.AddSensor(SensorNode(nodeNumber=nALE, fileName='solution/nodeALEvel.txt',
                                 outputVariableType=exu.OutputVariableType.Coordinates_t))

    #constraint to prescribe ALE velocity; will be set in PreStepUserFunction
    oCCvALE=mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mALE], offset=0, 
                                               velocityLevel = False, 
                                               visualization=VCoordinateConstraint(show=False))) # False for static computation
    
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #create one beam template
    cable = ALECable2D(#physicsLength=L, 
                        physicsMassPerLength=rhoA, 
                        physicsBendingStiffness=EI, 
                        physicsAxialStiffness=EA, 
                        physicsBendingDamping=bendingDamping, 
                        physicsAxialDamping=axialDamping, 
                        physicsMovingMassFactor=movingMassFactor, 
                        nodeNumbers=[0,0,nALE]
                        )
    
    ancf=GenerateStraightLineANCFCable2D(mbs=mbs,
                    positionOfNode0=[0,0,0], positionOfNode1=[L,0,0],
                    numberOfElements=nElements,
                    cableTemplate=cable, #this defines the beam element properties
                    massProportionalLoad = [0,-9.81*0,0], #optionally add gravity
                    fixedConstraintsNode0 = [1,1,0,1], #fixed
                    fixedConstraintsNode1 = [1,1,0,1]) #fixed

    ancfNodes = ancf[0]
    ancfObjects = ancf[1]
    for oCC in ancf[4]:
        mbs.SetObjectParameter(oCC,'VdrawSize',0.005)
    

    if useCoordinateSpringDamper:            
        for node in ancfNodes:
            mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = node, coordinate=0))
            mbs.AddObject(CoordinateSpringDamper(markerNumbers = [mGround , mANCF0], 
                                                 stiffness = 0, damping = 1*damper,
                                                 visualization=VCoordinateSpringDamper(show=False)))
            
            mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = node, coordinate=1))
            mbs.AddObject(CoordinateSpringDamper(markerNumbers = [mGround, mANCF0], 
                                                 stiffness = 0, damping = damper,
                                                 visualization=VCoordinateSpringDamper(show=False)))
        
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    midNode = ancfObjects[int(0.25*L/(L/nElements))] #gives correct result for odd node numbers / even nElements
    midNode2 = ancfNodes[int(len(ancfNodes)/2)] #gives correct result for odd node numbers / even nElements
    useMidSpanLoad = True
    if useMidSpanLoad:
        mANCFmid = mbs.AddMarker(MarkerBodyPosition(bodyNumber=midNode,localPosition = [0.25*L-L/nElements*int(0.25*L/(L/nElements)),0,0])) 
        nLoad=mbs.AddLoad(LoadForceVector(markerNumber=mANCFmid, loadVector=[0, -f, 0], 
                                          loadVectorUserFunction=UFLoad
                                          ))

    addForce=False
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    nMassList = []  #mass nodes
    oMassList = []  #mass objects
    #mbs.variables['ccMassList'] = [] #coordinate constraints objects
    mbs.variables['slidingJointList'] = []
    mbs.variables['massSensors'] = []
    if nMasses != 0:
        nDataIndex = 0 #data coordinate index for ALESlidingJoint
        mass = totalMass*movingDiscreteMassFactor / nMasses
        sizeMass = 0.02
        for i in range(nMasses):
            nMass = mbs.AddNode(NodePoint2D(referenceCoordinates=[0,0], initialVelocities=[0*mbs.variables['setVALE'],0]))
            g = GraphicsDataSphere([0,0,0], radius=sizeMass*0.5, color=color4red)
            oMass = mbs.AddObject(ObjectMassPoint2D(physicsMass=mass, nodeNumber=nMass, 
                                                    visualization=VMassPoint2D(graphicsData=[g])))
    
            mMassX = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nMass, coordinate=0)) 
            mMassY = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nMass, coordinate=1)) 
            
            if addForce:
                mLoadMarker = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass)) 
                mbs.AddLoad(LoadForceVector(markerNumber=mLoadMarker, loadVector=[0, -10, 0]))
            
            
            if useGraphics:
                sMass = mbs.AddSensor(SensorNode(nodeNumber=nMass, fileName = 'solution/nodeMass'+str(i)+'.txt',
                                                 outputVariableType=exu.OutputVariableType.Position))            
                mbs.variables['massSensors'] += [sMass]
            #constraint to prescribe ALE velocity; will be set in PreStepUserFunction
            # mbs.variables['ccMassList']+=[mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mMassX], offset=0, 
            #                                            velocityLevel = False, # False for static computation
            #                                            visualization=VCoordinateConstraint(show=False)))] 
            # mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mMassY], offset=0, 
            #                                            visualization=VCoordinateConstraint(show=False)))
            sStart = i*L/nMasses
            mMass = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oMass))
            oSJ = GenerateAleSlidingJoint(mbs,ancfObjects, mMass, nALE,
                                localMarkerIndexOfStartCable=0, AleSlidingOffset=0,
                                activeConnector=True)
            #print("oSJ=",oSJ)
            #list contains: sliding joint object, sliding mass node number, data coordinate node number,
            #               parameter sALE at which every node is started
            mbs.variables['slidingJointList'] += [[oSJ[0], nMass, nDataIndex, sStart]]
            #print(mbs.GetObject(oSJ[0]))
            mbs.SetObjectParameter(oSJ[0],'VdrawSize',sizeMass*1.2)
            #print("sj=",mbs.variables['slidingJointList'][-1])
            nDataIndex += 1 #one coordinate added per sliding joint

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    sensorFileName = 'solution/beamALEmidPoint'+iCalc+'.txt'
    mbs.AddSensor(SensorBody(bodyNumber=midNode,localPosition=[0.25*L-L/nElements*int(0.25*L/(L/nElements)),0,0], fileName=sensorFileName, 
                                outputVariableType=exu.OutputVariableType.Displacement))
    

    mbs.Assemble()
    # print(mbs)
    #mbs.systemData.Info()

    simulationSettings = exu.SimulationSettings() #takes currently set values or default values
    if useGraphics:
        verboseMode = 1
    else:
        verboseMode = 0

    
    # mbs.SetObjectParameter(oCC, 'offsetUserFunction', OffsetUF) 
    # mbs.SetNodeParameter(nALE, 'initialCoordinates_t', [vALE])
    # mbs.SetObjectParameter(oCC, 'offset', vALE)
    # mbs.SetLoadParameter(nLoad, 'loadVector', [0, 0, 0])  
    
    simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
    simulationSettings.timeIntegration.endTime = tEnd
    simulationSettings.solutionSettings.writeSolutionToFile = False
    simulationSettings.solutionSettings.sensorsWritePeriod = 0.004
    #simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6 #10000
    simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-8 #default:1e-10
    simulationSettings.timeIntegration.verboseMode = verboseMode
    simulationSettings.staticSolver.verboseMode = verboseMode
    
    simulationSettings.timeIntegration.newton.useModifiedNewton = True
    # simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
    simulationSettings.timeIntegration.adaptiveStep = True #disable adaptive step reduction
    ##############################################################
    # IMPORTANT!!!!!!!!!
    #simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse #sparse solver !!!!!!!!!!!!!!!
    ##############################################################
    simulationSettings.displayStatistics = True
    
          
    if useGraphics:
        exu.StartRenderer()
        mbs.WaitForUserToContinue()

    simulationSettings.staticSolver.numberOfLoadSteps=10

    success = exu.SolveStatic(mbs, simulationSettings, updateInitialValues=True)
    # if useGraphics:
    #     mbs.WaitForUserToContinue()

    for sjl in mbs.variables['slidingJointList']:
        mbs.SetObjectParameter(sjl[0], 'activeConnector', False)

    #mbs.SetObjectParameter(oCCvALE,'velocityLevel',True)
    mbs.SetPreStepUserFunction(MBSUserFunction)     #only for dynamic!

    #simulationSettings.timeIntegration.simulateInRealtime = True
    #simulationSettings.timeIntegration.realtimeFactor=1
    success = exu.SolveDynamic(mbs, simulationSettings, exudyn.DynamicSolverType.TrapezoidalIndex2)
    #success = exu.SolveDynamic(mbs, simulationSettings)

    if useGraphics:
        SC.WaitForRenderEngineStopFlag()
        exu.StopRenderer() #safely close rendering window!        

        from exudyn.plot import PlotSensor
        plt.figure("masses-x")
        PlotSensor(mbs, sensorNumbers=mbs.variables['massSensors'],components=[0]*nMasses)
        plt.figure("masses-y")
        PlotSensor(mbs, sensorNumbers=mbs.variables['massSensors'],components=[1]*nMasses)
        plt.figure("ALE pos")
        PlotSensor(mbs, sensorNumbers=mbs.variables['sALEpos'],components=0)
        exu.variables['mbs'] = mbs
        

    data0 = np.loadtxt(sensorFileName, comments='#', delimiter=',') 
    n = len(data0)
    nStart = int(n*(tEnd - 0.25*tEvaluate)/tEnd)
    #amp = max(data0[nStart:n,2]) - min(data0[nStart:n,2]) #oscillation amplitude
    #amp = abs(0.5*(max(data0[nStart:n,2]) + min(data0[nStart:n,2])))
    amp = (max(abs(data0[nStart:n,2])))
    if useGraphics:
        print("amp=", amp)

#%%++++++++++++++++++++++++++++++++++++
    if True: #delete files; does not work for parallel, consecutive operation
        if iCalc != 'Ref':
            os.remove(sensorFileName) #remove files in order to clean up
            while(os.path.exists(sensorFileName)): #wait until file is really deleted -> usually some delay
                sleep(0.001) #not nice, but there is no other way than that

    del mbs
    del SC
    return amp

#%%++++++++++++++++++++++++++++++++++++
#single run:
if True:
    useGraphics = True
    vALE0 = 7# 5.5#2 masses 7.5, 1 mass 8
    plt.close("all")
    BuildModel({'vALE0':vALE0, 'nMasses':7})
    # plt.close()
    plt.figure()
    
    ax=plt.gca()
    ax.grid(True,'major','both')
    
    data0 = np.loadtxt('solution/beamALEmidPointRef.txt', comments='#', delimiter=',') 
    plt.plot(data0[:,0],data0[:,2],'b-',label='midPointDeflection')
    data0 = np.loadtxt('solution/beamALEmidPointRef.txt', comments='#', delimiter=',') 
    plt.plot(data0[:,0],data0[:,2],'b-',label='vALE'+str(vALE0))
    
    plt.tight_layout()
                
    plt.legend()
    plt.show()
    #plt.plot(data0[:,0],data0[:,2])
    #plt.show()


#%%++++++++++++++++++++++++++++++++++++
#parameter variation    
if False and __name__ == "__main__": #check if this is performed in main thread for multiprocessing
    start_time = time.time()
    n = 320+1 #320+1 for fine computation
    [pDict, values] = ParameterVariation(parameterFunction=BuildModel, 
                                         parameters = {'vALE0':(0,8,n), 
                                                       #'h':(2e-4,1e-3,2),
                                                       #'damping':(1e-1,5e-1,3),
                                                       #'nElements':(8,16,2),
                                                       'nMasses':(0,7,8), #0,7,8 for fine computation
                                                       #'movingDiscreteMassFactor':(0.1,1,10),
                                                       },
                                         debugMode=False,
                                         #useLogSpace=True,
                                         addComputationIndex=True,
                                         useMultiProcessing=True,
                                         numberOfThreads=4,
                                         showProgress=True,
                                         #resultsFile='solution/parvarForce4Eval12discreteMass0.9.txt'
                                         resultsFile='solution/parameterVariation.txt'
                                         )

    print("--- %s seconds ---" % (time.time() - start_time))


    fig = plt.figure()
    ax=fig.gca() # get current axes

    ax.plot(pDict['vALE0'], values, 'b-', label='vALE') 
    ax.set_ylabel('max amplitude (m)')
    ax.set_xlabel('vALE (m/s)')


    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 

    # if xLogScale:
    #     ax.set_xscale('log')
    # if yLogScale:
    #     ax.set_yscale('log')
        
    plt.tight_layout()
    plt.legend()

    plt.show() 






