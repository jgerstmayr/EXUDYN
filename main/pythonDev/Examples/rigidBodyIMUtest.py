#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Rigid body example, which generates test data for IMU (local angular velocity and accelerations)
#
# Author:   Johannes Gerstmayr
# Date:     2020-11-13
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
from exudyn.plot import PlotSensor
from math import sin, cos, pi

SC = exu.SystemContainer()
mbs = SC.AddSystem()

print('EXUDYN version='+exu.GetVersionString())

#background
#rect = [-0.1,-0.1,0.1,0.1] #xmin,ymin,xmax,ymax
#background0 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
color = [0.1,0.1,0.8,1]
s = 0.25 #size of cube
sx = s #x-size
zz = 1.25*s #max size
cPosZ = 0.1 #offset of constraint in z-direction

# background0 = GraphicsDataRectangle(-zz,-zz,zz,zz,color)
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))#, visualization=VObjectGround(graphicsData= [background0])))
# mPosLast = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oGround, 
#                                             localPosition=[-2*sx,0,cPosZ]))



omega0 = [0,0,0] #arbitrary initial angular velocity
ep0 = eulerParameters0 #no rotation

ep_t0 = AngularVelocity2EulerParameters_t(omega0, ep0)

p0 = [0,0,0] #reference position
v0 = [0,0,0] #initial translational velocity

nGyro = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=p0+ep0, 
                                  initialVelocities=v0+list(ep_t0)))
Ixx = 0.2
Iyy = 0.2
Izz = 0.2
oGraphics = GraphicsDataOrthoCube(-sx,-s,-s, sx,s,s, [0.8,0.1,0.1,1])
oGyro = mbs.AddObject(ObjectRigidBody(physicsMass=2, 
                                    physicsInertia=[Ixx,Iyy,Izz,0,0,0], 
                                    nodeNumber=nGyro, 
                                    visualization=VObjectRigidBody(graphicsData=[oGraphics])))

# mMassRB = mbs.AddMarker(MarkerBodyMass(bodyNumber = oRB))
# mbs.AddLoad(Gravity(markerNumber = mMassRB, loadVector=[0.,-9.81,0.])) #gravity in negative z-direction

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#add torque to rotate body around one axis; M=Ixx*omega_t
angleXX = 0.5*2*pi #amount of rotation caused by torque
angleYY = 0.25*2*pi #amount of rotation caused by torque
angleZZ = 0.25*2*pi #amount of rotation caused by torque
def UFtorque1(t, load):
    fx = 0    
    fy = 0    
    fz = 0    
    if t <= 0.5:
        fx = Ixx*angleXX*4 #factor 4 because of integration of accelerations
    elif t <= 1:
        fx = -Ixx*angleXX*4
    elif t <= 1.5:
        fz = Izz*angleZZ*4 #factor 4 because of integration of accelerations
    elif t <= 2:
        fz = -Izz*angleZZ*4
    elif t <= 2.5:
        fy = Iyy*angleYY*4 #factor 4 because of integration of accelerations
    elif t <= 3:
        fy = -Iyy*angleYY*4
    return [fx,fy,fz]
    
mCenterRB = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oGyro, localPosition = [0.,0.,0.]))
mbs.AddLoad(Torque(markerNumber = mCenterRB, 
                   loadVector=[0,0,0],
                   bodyFixed=True, #use local coordinates for torque
                   loadVectorUserFunction = UFtorque1)) #gravity in negative z-direction

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#sensors
#all sensors placed at localPosition=[0,0,0]
sOmegaLocal = mbs.AddSensor(SensorBody(bodyNumber=oGyro, fileName='solution/angularVelocityLocalIMU.txt',
                          outputVariableType=exu.OutputVariableType.AngularVelocityLocal))
sRotation = mbs.AddSensor(SensorBody(bodyNumber=oGyro, fileName='solution/rotationIMU.txt',
                          outputVariableType=exu.OutputVariableType.Rotation))
sOmega = mbs.AddSensor(SensorBody(bodyNumber=oGyro, fileName='solution/angularVelocityGlobalIMU.txt',
                          outputVariableType=exu.OutputVariableType.AngularVelocity))
sAcc = mbs.AddSensor(SensorBody(bodyNumber=oGyro, fileName='solution/accelerationGlobalIMU.txt',
                                localPosition = [0.05,0.05,0.05],
                          outputVariableType=exu.OutputVariableType.Acceleration))
sRot = mbs.AddSensor(SensorBody(bodyNumber=oGyro, fileName='solution/rotationMatrix.txt',
                          outputVariableType=exu.OutputVariableType.RotationMatrix))

#acceleration sensor: gives global coordinates!
#sAcceleration = mbs.AddSensor(SensorBody(bodyNumber=oGyro, 
#                                         fileName='solution/accelerationGlobalIMU.txt',
#                                         localPosition=[0.05,0.05,0], #measure off-axis accerlations...
#                                         outputVariableType=exu.OutputVariableType.Acceleration))
#

mbs.Assemble()
#print(mbs)

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 3
h = 1e-5
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.solutionWritePeriod = 2e-3
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.useModifiedNewton = True

#use Newmark index2, no damping
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True

simulationSettings.solutionSettings.solutionInformation = "rigid body tests"
SC.visualizationSettings.loads.show = False
SC.visualizationSettings.nodes.showBasis=True
SC.visualizationSettings.nodes.basisSize=1.25
SC.visualizationSettings.nodes.show=True

SC.visualizationSettings.general.autoFitScene = 0
exu.StartRenderer()
SC.SetRenderState({'centerPoint': [0.0, 0.0, 0.0],
 'maxSceneSize': 1.0,
 'zoom': 2.0,
 'currentWindowSize': [1024, 768],
 'modelRotation': np.array([[0,0,1],
        [1,0,0],
        [0,1,0]])}) #load last model view

#mbs.WaitForUserToContinue()
SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)

#SC.WaitForRenderEngineStopFlag()
exu.StopRenderer() #safely close rendering window!

if False:
    import matplotlib.pyplot as plt
    plt.close("all")
    PlotSensor(mbs, [sOmegaLocal]*3, components = [0,1,2])
    plt.figure()
    PlotSensor(mbs, [sRotation]*3, components = [0,1,2])

#+++++++++++++++++++++++++++++++++++++++++++++++++++++

if True:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    plt.close("all")
    ax=plt.gca() # get current axes

    dataRot = np.loadtxt('solution/rotationMatrix.txt', comments='#', delimiter=',')
    dataAcc = np.loadtxt('solution/accelerationGlobalIMU.txt', comments='#', delimiter=',')

    n = len(dataAcc)
    accLocal = np.zeros((n,4))
    accLocal[:,0] = dataAcc[:,0]
    rotMat = np.zeros((n,3,3))
    for i in range(n):
        rotMat[i,:,:] = dataRot[i,1:10].reshape(3,3)
        accLocal[i,1:4] = rotMat[i,:,:] @ dataAcc[i,1:4]

    #plot 3 components of global accelerations
    plt.plot(accLocal[:,0], accLocal[:,1], 'r-', label='acc0 local') 
    plt.plot(accLocal[:,0], accLocal[:,2], 'g-', label='acc1 local') 
    plt.plot(accLocal[:,0], accLocal[:,3], 'b-', label='acc2 local') 
    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
    plt.tight_layout()
    plt.legend()

    plt.figure()
    #plot 3 components of local accelerations
    plt.plot(dataAcc[:,0], dataAcc[:,1], 'r-', label='acc0 global') 
    plt.plot(dataAcc[:,0], dataAcc[:,2], 'g-', label='acc1 global') 
    plt.plot(dataAcc[:,0], dataAcc[:,3], 'b-', label='acc2 global') 
    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
    plt.tight_layout()
    plt.legend()

    plt.figure()
    PlotSensor(mbs, [sRotation]*3, components = [0,1,2])
    
    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
    plt.tight_layout()
    plt.show() 
