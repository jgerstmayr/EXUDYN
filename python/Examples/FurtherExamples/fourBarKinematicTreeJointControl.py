#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A four bar mechanism with kinematic tree with joint control in kinematic tree
#
# Author:   Felix Schmid
# Date:     2024-05-06
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *

import numpy as np

useGraphics = True #without test

SC = exu.SystemContainer()
mbs = SC.AddSystem()


L1 = 0.5 #length of links
L2 = 1
L3 = 1
L = [L1,L2,L3]
w = 0.1 #width of links
ground = mbs.CreateGround(referencePosition=[L2,L1-L3,0])

gravity3D = [0,-9.81,0]

n=3 #3number of coordinates

linkMasses = []
linkCOMs = exu.Vector3DList()
linkInertiasCOM=exu.Matrix3DList()

jointTransformations=exu.Matrix3DList()
jointOffsets = exu.Vector3DList()
for i in range(n):
    # Erstellen von rotierten Achsen und Verschiebungen
    if i == 0:
        v = np.array([0, 0, 0])
        A = RotXYZ2RotationMatrix([0, 0, 0.5 * pi])


    if i == 1:
        v = np.array([L1, 0, 0])
        A = RotXYZ2RotationMatrix([0, 0, -0.5 * pi])

    if i == 2:
        v = np.array([L2, 0, 0])
        A = RotXYZ2RotationMatrix([0, 0, -0.5 * pi])
        
    J = InertiaCuboid(density=1000, sideLengths=[L[i],w,w]) #w.r.t. reference center of mass
    J = J.Translated([0.5*L[i],0,0])
    linkCOMs.Append(J.COM())
    Jcom = J

    #now add joint/link to lists:
    jointTransformations.Append(A)
    jointOffsets.Append(v)

    linkMasses += [Jcom.Mass()]
    linkInertiasCOM.Append(Jcom.InertiaCOM())
    

#create per-link graphics:
gLink1 =  GraphicsDataOrthoCubePoint(centerPoint= [0.5*L1,0,0], size= [L1,w,w], color= color4red)
gLink2 =  GraphicsDataOrthoCubePoint(centerPoint= [0.5*L2,0,0], size= [L2,w,w], color= color4dodgerblue)
gLink3 =  GraphicsDataOrthoCubePoint(centerPoint= [0.5*L3,0,0], size= [L3,w,w], color= color4green)
gJoint = GraphicsDataCylinder([0,0,-1.25*w], [0,0,2.5*w], 0.4*w, color=color4grey)
gList = [[gJoint,gLink1],[gJoint,gLink2],[gJoint,gLink3]] #one list per link; add joint first, then it will be visible with transparency setting

#create node for unknowns of KinematicTree
nGeneric = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0.]*n,
                                       initialCoordinates=[0.]*n,
                                       initialCoordinates_t=[0.]*n,
                                       numberOfODE2Coordinates=n))

#create KinematicTree
tree = mbs.AddObject(ObjectKinematicTree(nodeNumber=nGeneric, jointTypes=[exu.JointType.RevoluteZ]*n, 
                                         linkParents=np.arange(n)-1,
                                  jointTransformations=jointTransformations, 
                                  jointOffsets=jointOffsets, 
                                  linkInertiasCOM=linkInertiasCOM, linkCOMs=linkCOMs, 
                                  linkMasses=linkMasses, 
                                  baseOffset = [0.,0.,0.], gravity=gravity3D, 
                                  jointVelocityOffsetVector=[5,0,0],
                                  jointDControlVector=[20,0,0],
                                  visualization=VObjectKinematicTree(graphicsDataList = gList)))




# Federdämpfer hinzufügen
mLink3 = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=tree,linkNumber=n-1,
                                        localPosition = [L3,0,0])) # Marker am Ende des 3. Balkens



mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=ground,
                                        localPosition = [0,0,0])) # Marker auf dem Boden

#+++++++++++++++++++++++++++++++++++++
#FederDämpfer auf den Boden 1. Versuch
#++++++++++++++++++++++++++++++++++++

# k=10e5
# mbs.AddObject(RigidBodySpringDamper(markerNumbers = [mGround, mLink3],
#                                     stiffness = np.diag([k,k,k, 0,0,0]),
#                                     damping = np.diag([0,k*0.01,0, 0,0,0]),
#                                     offset = [0,0,0, 0,0,0]))

#+++++++++++++++++++++++++++++++++++++
#GenericJoint auf den Boden 2. Versuch
#+++++++++++++++++++++++++++++++++++++

GroundJoint = mbs.AddObject(GenericJoint(markerNumbers=[mLink3, mGround],
                                         constrainedAxes=[1,1,0,0,0,0]))

sOmega = mbs.AddSensor(SensorKinematicTree(objectNumber=tree, linkNumber=0, outputVariableType=exu.OutputVariableType.AngularVelocity, storeInternal=True))


mbs.Assemble()

tEnd=10
h=0.005

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h) #must be integer
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.simulateInRealtime =False


SC.visualizationSettings.general.drawWorldBasis = True

#solve dynamic problem with default parameters

mbs.SolveDynamic(simulationSettings) 

mbs.SolutionViewer()

mbs.PlotSensor(sOmega,components=[2])

SC.WaitForRenderEngineStopFlag()
exu.StopRenderer() #safely close rendering window!