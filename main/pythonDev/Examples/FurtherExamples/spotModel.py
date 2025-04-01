#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  This file shows setup of model for quad-legged spot using URDF file; spotData.h5 has to be in same folder!
#
# Author:   Johannes Gerstmayr, Janik Thentie
# 
# Date:      2025-03-10
#
# Notes:     requires pip install stable-baselines3[extra]
# 
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
#from exudyn.rigidBodyUtilities import *
from exudyn.robotics import *
from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration, ProfilePTP
from exudyn.robotics.utilities import GetRoboticsToolboxInternalModel, LoadURDFrobot, GetURDFrobotData

import numpy as np
from numpy import linalg as LA
from math import pi
import sys

#define function to create model which can be called from outside
def GetModel(addShoulderContact = False):
    #%%+++++++++++++++++++
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    
    useNewStaticContact = True #set false, if not available
    
    g = [0,0,-9.81]
    #%% contact
    k = 10e4*0.5
    d = 0.01*k
    frictionCoeff = 0.8 #0.5
    ss = 1
    
    planeL = 16
    #zContact = -0.7+0.01214 #contact at beginning, straight legs
    zContact = -0.7+0.01214+6.45586829e-02 #contact at beginning, leg = [0,0.2*pi,-0.3*pi]
    p0 = [0,0,zContact]
    mbs.variables['zContact'] = zContact
    
    
    markerList = []
    radiusList = []
    
    
    gContact = mbs.AddGeneralContact()
    gContact.sphereSphereContact = False
    #gContact.verboseMode = 1
    gContact.SetFrictionPairings(frictionCoeff*np.eye(1))
    gContact.SetSearchTreeCellSize(numberOfCells=[ss,ss,1])
    stFact = 0.3 #smaller grid as we do not walk that far
    gContact.SetSearchTreeBox(pMin=np.array([-0.5*planeL*stFact,-0.5*planeL*stFact,-1]),
                               pMax=np.array([0.5*planeL*stFact,0.5*planeL*stFact,0.1]))
    #gContact.frictionVelocityPenalty = 1e3 #10
    gContact.frictionProportionalZone = 0.01 #0.001
    
    #%% ground
    gGroundSimple = graphics.CheckerBoard(p0,size=planeL, nTiles=1) #only 1 tile for efficiency
    gGround = graphics.CheckerBoard(p0,size=planeL, nTiles=16)
    objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0],
                                              visualization=VObjectGround(graphicsData=[gGround])))
    
    
    nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0] ))
    mGround = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGround))
    
    [meshPoints, meshTrigs] = graphics.ToPointsAndTrigs(gGroundSimple)
    #[meshPoints, meshTrigs] = RefineMesh(meshPoints, meshTrigs) #just to have more triangles on floor
    gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mGround, 
                                        contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,
                                        pointList=meshPoints,  triangleList=meshTrigs,
                                        staticTriangles=useNewStaticContact, 
                                        )
    # gContact.isActive = False
    
    
    #%%+++++++++++++++++++++++++++++++
    URDF_visualisation = True # showing Bricks like interia  or  URDF-Model
    onlyShowModel = False #True: only graphics, no simulation
    listTrajectory = []
    oKT = None
    nKT = None
    saveHDF5 = True        #use this to save robot data, allowing to load directly from there
    loadFromHDF5 = True    #faster load; does not require roboticstoolbox, pymeshlab, etc.
    
    modelName = 'robot'
    modelCnt = 0
    #for modelCnt, modelName in enumerate(robotModels):
    from exudyn.advancedUtilities import LoadDictFromHDF5
    robotDataDict = LoadDictFromHDF5('spotData.h5')
    
    
    offsetPosition = [0,0,0]
    
    
    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    linkList = robotDataDict['linkList']
    numberOfJoints = robotDataDict['numberOfJoints']

    if len(linkList) != numberOfJoints:
        #if this does not work for robots, probably some weird configuration or branches/multiple arms
        raise ValueError('robot creation: inconsistent data!')
    
    
    graphicsBaseList = []#graphics.Sphere(radius=0.01, color=graphics.color.grey)] #robotDataDict['graphicsBaseList']
    graphicsToolList = robotDataDict['graphicsToolList']
    
    #control parameters, per joint:
    fact = 0.5*1e3 #make control soft, as we like to avoid heavy oscillatory motion
    Pcontrol = np.array([fact]*numberOfJoints)  # [hip_y, hip_x, knee_x, ....]
    Dcontrol = np.array([0.05*fact]*numberOfJoints)
     
    legsInit = [0,36*pi/180,-54*pi/180.]
    mbs.variables['legsInit'] = legsInit
    # referenceConfiguration = [18]*0
    referenceConfiguration = [0]*6 + legsInit*4
    mbsRobot = Robot(gravity=g,
                  base = RobotBase(HT = HTtranslate(offsetPosition)),                  
                  referenceConfiguration = referenceConfiguration) #referenceConfiguration created with 0s automatically
    
    #++++++++++++++++++++++++++
    #according numbering of links (0,1,2,...)
    linkNumbers={'None':-1}
    linkNumbers['base']= 5
    linkNumbers['base_link']= 5
    
    # Floating Base Definition
    floatingBaseLinks = [
        {'name': 'base_x', 'jointType': 'Px'},
        {'name': 'base_y', 'jointType': 'Py'},
        {'name': 'base_z', 'jointType': 'Pz'},
        {'name': 'base_roll', 'jointType': 'Rx'},
        {'name': 'base_pitch', 'jointType': 'Ry'},  
        {'name': 'base_yaw', 'jointType': 'Rz'},    
    ]
    for cnt, link in enumerate(floatingBaseLinks):
        linkNumbers[link['name']] = cnt
        
    for cnt, link in enumerate(linkList):
        linkNumbers[link['name']] = cnt + 6
        
    #++++++++++++++++++++++++++
    #Floating Base
    mass_base = 20
    cube = [0.7,0.4,0.12]
    graphic_base = [graphics.Brick(centerPoint=[0,0,0],size=cube,color=graphics.color.red)]
    inertia_base = InertiaCuboid(density=2000,sideLengths=cube).InertiaCOM()
        
    if URDF_visualisation:
        graphic_base = robotDataDict['graphicsBaseList']   
    
       
    for cnt, baseLink in enumerate(floatingBaseLinks):
        vis = VRobotLink(jointWidth=0, jointRadius=0, linkWidth=0, showCOM=False, showMBSjoint=False) if cnt < 5 else VRobotLink(graphicsData = graphic_base)
        # if cnt < 5:
        #     vis = VRobotLink(showMBSjoint=False)
        mbsRobot.AddLink(RobotLink(
                        mass= 0 if cnt < 5 else mass_base,  # 0 mass for floating links
                        parent=-1 if cnt == 0 else linkNumbers[floatingBaseLinks[cnt - 1]['name']],
                        COM=[0, 0, 0],
                        inertia=0*InertiaSphere(0.1, 0.01).InertiaCOM() if cnt < 5 else inertia_base,
                        preHT=HTtranslate([0, 0, 0]),  # No offset
                        jointType=baseLink['jointType'],
                        PDcontrol=(0, 0),
                        visualization= vis
                    ))
    
    
    
    #++++++++++++++++++++++++++
    #load model    
    # print('*** load model '+modelName+', nJoints=',numberOfJoints)
    for cnt, link in enumerate(linkList):
        jointType = None
        HT = link['preHT']

        if len(link['graphicsDataList']) == 0: #no graphics loaded (no urdf, no pymeshlab)
            visualization=VRobotLink(linkColor=graphics.colorList[cnt])
            
        elif URDF_visualisation:            
            visualization=VRobotLink(graphicsData =link['graphicsDataList'],
                                     showMBSjoint=False,
                                     jointWidth=0.12,jointRadius=0.01)
        mass = link['mass']
        inertia = link['inertiaCOM']
        
        if mass == 0: #checking mass in URDF
            # if cnt == 0 and mass == 0:
            #     print('no mass found in URDF,  used own values')            
            
             
            if 'hip' in link['name']:
                mass = 0.5
                cube = [0.05,0.2,0.05]
                graphic = [graphics.Brick(centerPoint=[0,0,0],size=cube,color=graphics.color.red)]
                inertia = InertiaCuboid(density=2000,sideLengths=cube).InertiaCOM()
                if not URDF_visualisation:
                    visualization=VRobotLink(graphicsData = graphic,
                                             showMBSjoint=False,
                                             jointWidth=0.12,jointRadius=0.01)
            else:
                mass = 2
                cube = [0.05,0.05,0.35]
                com = [0,0,-0.35/2]
                link['com'] = com
                graphic = [graphics.Brick(centerPoint=com,size=cube,color=graphics.color.red)]
                inertia = InertiaCuboid(density=3000,sideLengths=[0.05,0.05,0.4]).InertiaCOM()
                if not URDF_visualisation:
                    visualization=VRobotLink(graphicsData = graphic,
                                             showMBSjoint=False,
                                             jointWidth=0.12,jointRadius=0.01)
        else:
             print('mass',cnt,'=',mass)
           
        mbsRobot.AddLink(RobotLink(mass=mass,
                                   parent = linkNumbers[link['parentName']],
                                   COM=link['com'], 
                                   inertia=inertia,
                                   preHT=link['preHT'],
                                   jointType=link['jointType'],
                                   PDcontrol=(Pcontrol[cnt], Dcontrol[cnt]),
                                   visualization=visualization
                                   ))
    
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #configurations and trajectory
    q0 = np.zeros(numberOfJoints+6) #zero angle configuration +6 Floating Base
    q1 = np.zeros(numberOfJoints+6) 

    #test simple motion
    #position fpr PD-Control:
    #ordering of legs:
    #front left, front right, back left, back right
    q0 = [0,0,0, 0,0,0] + list(np.array(legsInit))*4
    #q1 = [0,0,0, 0,0,0] + list(np.array(leg))*4
    # q1 = [0,0,0, 0,0,0, 0,0.1*pi,-0.1*pi, 0,0,0, 0,0,0, 0,0,0]
    # q1 = [0,0,0, 0,0,0, 0,0,0, 0,0.1*pi,-0.1*pi, 0,0,0, 0,0,0]
    # q1 = [0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0.1*pi,-0.1*pi, 0,0,0]
    q1 = [0,0,0, 0,0,0, 0.25*pi,0,0,-0.25*pi,0,0,0.25*pi,0,0,-0.25*pi,0,0,]
    q2 = [0,0,0, 0,0,0, 0,0.5*pi,-0.9*pi,0,0.5*pi,-0.9*pi,0,0.5*pi,-0.9*pi,0,0.5*pi,-0.9*pi] # [[0]*6, hip_y, hip_x, knee_x, ....]
    q3 = [0,0,0, 0,0,0, 0,0.5*pi,-0.9*pi,0,0.5*pi,-0.9*pi,0,0.5*pi,-0.9*pi,0,0.5*pi,-0.9*pi] # [[0]*6, hip_y, hip_x, knee_x, ....]
    q4 = [0,0,0,0,0,0,0,0.5*pi,-0.9*pi,0,0.5*pi,-0.9*pi,0,0,0,0,0,0]

    # q1 = [0,0,0,0,0,0,0,0.4*pi,-0.5*pi,0,0,0,0,0,0,0,0,0] # [[0]*6, hip_y, hip_x, knee_x, ....]
    # q2 = q0
    
    
    #trajectory generated with optimal acceleration profiles:
    trajectory = Trajectory(initialCoordinates=q0, initialTime=0)
    trajectory.Add(ProfileConstantAcceleration(q0,0.3))
    trajectory.Add(ProfileConstantAcceleration(q1,1))
    trajectory.Add(ProfileConstantAcceleration(q2,1))
    trajectory.Add(ProfileConstantAcceleration(q0,0.7))
    trajectory.Add(ProfileConstantAcceleration(q3,0.7))
    trajectory.Add(ProfileConstantAcceleration(q4,0.7))
    trajectory.Add(ProfileConstantAcceleration(q0,0.4))
    
    
    
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #build mbs mbsRobot model:
    robotDict = mbsRobot.CreateKinematicTree(mbs)
    
    oKT = robotDict['objectKinematicTree']
    nKT = robotDict['nodeGeneric']
        
    #+++++ MAREKER for Contact++++
    for i in ['fl.hip','fr.hip','hl.hip','hr.hip']:     
        offy = 0.1
        if 'l' not in i:
            offy *= -1
        m = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oKT, linkNumber=linkNumbers[i], 
                                                   localPosition=[0,offy,0.005], visualization=VMarkerKinematicTreeRigid()))    
        if addShoulderContact: #set False for efficiency
            gContact.AddSphereWithMarker(m, radius=0.08, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,)

    legMarkers = []            
    for i in ['fl.lleg','fr.lleg','hl.lleg','hr.lleg']:     
        r = 0.05 #0.035
        mbs.variables['legRadius'] = r #radius of leg contact sphere, relative to marker
        m = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oKT, linkNumber=linkNumbers[i], 
                                                   localPosition=[0,0.,-0.370+r], visualization=VMarkerKinematicTreeRigid()))    
        gContact.AddSphereWithMarker(m, radius=r, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,)
        legMarkers += [m]

    mbs.variables['legMarkers'] = legMarkers

    #add sensors (only last sensor is kept)
    sJointRot = mbs.AddSensor(SensorBody(bodyNumber=oKT, 
                                         storeInternal=True, 
                                         outputVariableType=exu.OutputVariableType.Coordinates))
    
    sTorques = mbs.AddSensor(SensorBody(bodyNumber=oKT, storeInternal=True, 
                                          outputVariableType=exu.OutputVariableType.Force))
            
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #user function which is called only once per step, prescribe trajectory
    
    
    mbs.Assemble()
    mbs.variables['trajectory'] = trajectory
    mbs.variables['mbsRobot'] = mbsRobot
    return mbs,SC,oKT,nKT





#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
if __name__=='__main__':
    mbs,SC,oKT,nKT = GetModel()

    maxVel = 0
    cntPrint = 0
    def PreStepUF(mbs, t):
        global maxVel, cntPrint
        #check coordinates
        if False:
            q_t = mbs.GetNodeOutput(nKT, variableType=exu.OutputVariableType.Coordinates_t)
            q = mbs.GetNodeOutput(nKT, variableType=exu.OutputVariableType.Coordinates)
            if False:
                #print('t=',round(t,3),',q=',np.round(q[:6],5))
                # print('t=',round(t,3),',q=',np.round(q[:6],5))
    
                # if int(t*10000)%100 == 0:
                    # print('t=',round(t,3),',q=',np.round(q[:6],3))
                print('t=',round(t,3),',q_t=',np.round(q_t,2))
            maxVel = max(maxVel,np.max(np.abs(q_t[6:])))
            if t>5:
                print('q=',q)

        cntPrint += 1
        if cntPrint % 100 == 0:
            legRadius = mbs.variables['legRadius']
            legMarkers = mbs.variables['legMarkers']
            zContact = mbs.variables['zContact']+0.05
            # print('t=',round(t,2),',fl=',np.round(mbs.GetMarkerOutput(markerNumber=legMarkers[0], variableType=exu.OutputVariableType.Position)[2]-zContact,4),end='')
            # print(',fr=',np.round(mbs.GetMarkerOutput(markerNumber=legMarkers[1], variableType=exu.OutputVariableType.Position)[2]-zContact,4),end='')
            # print(',bl=',np.round(mbs.GetMarkerOutput(markerNumber=legMarkers[2], variableType=exu.OutputVariableType.Position)[2]-zContact,4),end='')
            # print(',br=',np.round(mbs.GetMarkerOutput(markerNumber=legMarkers[3], variableType=exu.OutputVariableType.Position)[2]-zContact,4))


        if True:
            #set position and velocity in kinematic tree joints:
            [u,v,a] = mbs.variables['trajectory'].Evaluate(t)
            #note: here we would need to add the gear ratios to u and v, if they shall represent motor angles!!!
            mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector', u)
            mbs.SetObjectParameter(oKT, 'jointVelocityOffsetVector', v)           


        return True

    mbs.SetPreStepUserFunction(PreStepUF)
    #simulate
       
    
    SC.visualizationSettings.connectors.showJointAxes = True
    SC.visualizationSettings.connectors.jointAxesLength = 0.02
    SC.visualizationSettings.connectors.jointAxesRadius = 0.002
    
    SC.visualizationSettings.nodes.showBasis = True
    SC.visualizationSettings.nodes.basisSize = 0.1
    SC.visualizationSettings.loads.show = False
    SC.visualizationSettings.bodies.kinematicTree.frameSize=0.12
    SC.visualizationSettings.openGL.multiSampling=4
        
    tEnd = 6
    stepSize = 0.0002 #could be larger!
    
    #mbs.WaitForUserToContinue()
    simulationSettings = exu.SimulationSettings() #takes currently set values or default values
    
    simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
    simulationSettings.timeIntegration.endTime = tEnd
    simulationSettings.solutionSettings.solutionWritePeriod = 0.04
    simulationSettings.solutionSettings.sensorsWritePeriod = 0.005
    simulationSettings.solutionSettings.binarySolutionFile = True
    simulationSettings.solutionSettings.writeSolutionToFile = True
    #simulationSettings.displayComputationTime = True
    # simulationSettings.timeIntegration.simulateInRealtime = True #DON'T do that!
    simulationSettings.timeIntegration.explicitIntegration.computeEndOfStepAccelerations = False
    simulationSettings.timeIntegration.explicitIntegration.computeMassMatrixInversePerBody = True
    
    simulationSettings.timeIntegration.verboseMode = 1
    # simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
        
    SC.visualizationSettings.contact.showSpheres = False
    SC.visualizationSettings.general.drawWorldBasis = False
    SC.visualizationSettings.general.autoFitScene=False
    SC.visualizationSettings.window.renderWindowSize=[1920,1200]
    SC.visualizationSettings.openGL.perspective=1
    SC.visualizationSettings.openGL.shadow=0.25
    SC.visualizationSettings.openGL.light0ambient = 0.5
    SC.visualizationSettings.openGL.light0diffuse = 0.5
    SC.visualizationSettings.openGL.light0position = [2,-4,8,0]
    
    
    SC.visualizationSettings.bodies.kinematicTree.showJointFrames = False

    useGraphics = True
    
    if useGraphics:
        exu.StartRenderer()
        if 'renderState' in exu.sys:
            SC.SetRenderState(exu.sys['renderState'])
        mbs.WaitForUserToContinue()
        
    mbs.SolveDynamic(simulationSettings, 
                       # solverType=exu.DynamicSolverType.TrapezoidalIndex2,
                       solverType=exu.DynamicSolverType.ExplicitEuler,
                       # solverType=exu.DynamicSolverType.VelocityVerlet,
                     showHints=True)

    print('max joint vel=',maxVel) 
    
    if useGraphics:
        SC.visualizationSettings.general.autoFitScene = False
        exu.StopRenderer()
    
    if True and simulationSettings.solutionSettings.writeSolutionToFile: #set True to show animation after simulation
        mbs.SolutionViewer()
    
    
    #%%++++++++++++++++++++++++++++
    if False and not onlyShowModel:
        mbs.PlotSensor(sensorNumbers=sJointRot, components=np.arange(numberOfJoints), 
                        title='joint angles', closeAll=True)
        mbs.PlotSensor(sensorNumbers=sTorques, components=np.arange(numberOfJoints), 
                        title='joint torques')
