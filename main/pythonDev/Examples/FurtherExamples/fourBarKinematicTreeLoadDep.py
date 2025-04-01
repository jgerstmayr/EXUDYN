#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A four bar mechanism with controlled torque at support of first bar;
#           Uses the KinematicTree with a closing-loop constraint;
#           Note: added the according dependencies for the sensors in order to improve Newton convergence
#
# Author:   Lukas Gerhold
# Edited:   Johannes Gerstmayr
# Date:     2024-05-06
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import exudyn as exu
from exudyn.utilities import * #includes graphicsDataUtilities and rigidBodyUtilities
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

L1 = 0.5
L2 = 1
L3 = 1
w = 0.05

ground = mbs.CreateGround(referencePosition=[L2,L1-L3,0])

J1 = InertiaCuboid(density=1000, sideLengths=[w,L1,w])
J1 = J1.Translated([0.5*L1,0,0])
com1 = J1.com

J2 = InertiaCuboid(density=1000, sideLengths=[L2,w,w])
J2 = J2.Translated([0.5*L2,0,0])
com2 = J2.com

J3 = InertiaCuboid(density=1000, sideLengths=[w,L3,w])
J3 = J3.Translated([0.5*L3,0,0])
com3 = J3.com

# "Schiefe" Gravitation, damit sich was bewegt

gravity3D = [0,-9.81,0]

n = 3 #number of links in kinematic tree

linkMasses = []
linkCOMs = exu.Vector3DList()
linkInertiasCOM = exu.Matrix3DList()

jointTransformations = exu.Matrix3DList()
jointOffsets = exu.Vector3DList()

for i in range(n):
    
    A = np.eye(3)
    
    if i == 0:
        
        A = RotXYZ2RotationMatrix([0,0,0.5*pi])
        
        v = np.array([0,0,0])
        
        linkMasses += [J1.Mass()]
        linkCOMs.Append(J1.COM())
        linkInertiasCOM.Append(J1.InertiaCOM())
        
    if i == 1:
        
        A = RotXYZ2RotationMatrix([0,0,-0.5*pi])
        
        v = np.array([L1,0,0])
        
        linkMasses += [J2.Mass()]
        linkCOMs.Append(J2.COM())
        linkInertiasCOM.Append(J2.InertiaCOM())
        
    if i == 2:
        
        A = RotXYZ2RotationMatrix([0,0,-0.5*pi])
        
        v = np.array([L2,0,0])
        
        linkMasses += [J3.Mass()]
        linkCOMs.Append(J3.COM())
        linkInertiasCOM.Append(J3.InertiaCOM())
    
    jointTransformations.Append(A)
    jointOffsets.Append(v)

gLink1 =  GraphicsDataOrthoCubePoint(centerPoint= [0.5*L1,0,0], size= [L1,w,w], color= color4dodgerblue)
gLink2 =  GraphicsDataOrthoCubePoint(centerPoint= [0.5*L2,0,0], size= [L2,w,w], color= color4dodgerblue)
gLink3 =  GraphicsDataOrthoCubePoint(centerPoint= [0.5*L3,0,0], size= [L3,w,w], color= color4dodgerblue)
gJoint = GraphicsDataCylinder([0,0,-2.5*w], [0,0,5*w], 0.8*w, color=color4grey)
gList = [[gJoint,gLink1],[gJoint,gLink2],[gJoint,gLink3]] #one list per link; add joint first, then it will be visible with transparency setting

nGeneric = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0.]*n,
                                       initialCoordinates=[0.]*n,
                                       initialCoordinates_t=[0.]*n,
                                       numberOfODE2Coordinates=n))

oKT = mbs.AddObject(ObjectKinematicTree(nodeNumber=nGeneric, jointTypes=[exu.JointType.RevoluteZ]*n, linkParents=np.arange(n)-1,
                                  jointTransformations=jointTransformations, jointOffsets=jointOffsets, 
                                  linkInertiasCOM=linkInertiasCOM, linkCOMs=linkCOMs, linkMasses=linkMasses, 
                                  baseOffset = [0.,0.,0.], gravity=gravity3D, 
                                  #jointForceVector=[0.]*n,
                                  visualization=VObjectKinematicTree(graphicsDataList = gList)))

# Marker am Ende und am Anfang (am Ende und Anfang des "Ground-Balkens")

markerAtB3 = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oKT,
                                                    linkNumber=2,
                                                    localPosition=[L3,0,0]))

markerAtGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=ground,
                                               localPosition=[0,0,0]))

# Gelenk zum "Ground-Balken"

oJoint = mbs.AddObject(GenericJoint(markerNumbers=[markerAtB3,markerAtGround],
                                           constrainedAxes=[1,1,0, 0,0,0]))

sOmega = mbs.AddSensor(SensorKinematicTree(objectNumber=oKT,
                                  linkNumber=0,
                                  outputVariableType=exu.OutputVariableType.AngularVelocity,
                                  storeInternal=True))

#user function for torque
def UF(mbs, t, load):
    #simple control: desired - current value
    load[2] = 100*(20 - mbs.GetSensorValues(sOmega)[2]) 
    return load
    
markerAtB0 = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oKT,
                                                    linkNumber=0,
                                                    localPosition=[0,0,0]))

lTorque = mbs.AddLoad(Torque(markerNumber=markerAtB0,loadVector=[0,0,-5],
            loadVectorUserFunction=UF))

mbs.Assemble()

#this has to be done after Assemble() to improve convergence:
#grab ODE2 coordinates of KT and add as dependencies for load:
coordsKT = mbs.systemData.GetObjectLTGODE2(oKT)
mbs.systemData.AddODE2LoadDependencies(lTorque, coordsKT)


tEnd = 2
h = 0.005

simulationSettings = exu.SimulationSettings()
simulationSettings.solutionSettings.writeSolutionToFile=False
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h) #must be integer
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.computeLoadsJacobian=2 #add load ODE2 and ODE2_t dependencies!
# simulationSettings.timeIntegration.simulateInRealtime = True
simulationSettings.displayStatistics = True

SC.visualizationSettings.bodies.kinematicTree.frameSize = 0.25
SC.visualizationSettings.bodies.kinematicTree.showJointFrames = True
SC.visualizationSettings.general.drawWorldBasis = True
SC.visualizationSettings.general.worldBasisSize = 2
SC.visualizationSettings.openGL.multiSampling = 4

exu.StartRenderer()
mbs.WaitForUserToContinue()

mbs.SolveDynamic(simulationSettings)

SC.WaitForRenderEngineStopFlag()
exu.StopRenderer()

#draw angular velocity
mbs.PlotSensor(sOmega, components=[2])

