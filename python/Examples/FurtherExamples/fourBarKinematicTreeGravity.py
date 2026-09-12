#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A four bar mechanism under gravity (with some additional disturbance in x-direction);
#           Uses the KinematicTree with a closing-loop constraint;
#
# Author:   Luca Weber
# Date:     2024-05-06
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import numpy as np

# create empty system
sc = exu.SystemContainer() # sc = systems container
mbs = sc.AddSystem() # mbs = multibodysystem

L1 = 0.5
L2 = 1
L3 = 1
w = 0.05

ground = mbs.CreateGround(referencePosition=[L2,L1-L3,0])

J1 = InertiaCuboid(density=1000, sideLengths=[L1,w,w]) #w.r.t. reference center of mass
J1 = J1.Translated([0.5*L1,0,0])
com1 = J1.com

J2 = InertiaCuboid(density=1000, sideLengths=[L2,w,w]) #w.r.t. reference center of mass
J2 = J2.Translated([0.5*L2,0,0])
com2 = J2.com

J3 = InertiaCuboid(density=1000, sideLengths=[L3,w,w]) #w.r.t. reference center of mass
J3 = J3.Translated([0.5*L3,0,0])
com3 = J3.com

#use some additional force in x-direction to initiate motion
gravity = [0.5,-9.81,0]

linkMasses = []
linkCOMs = exu.Vector3DList()
linkInertiasCOM=exu.Matrix3DList()

jointTransformations=exu.Matrix3DList()
jointOffsets = exu.Vector3DList()

n = 3

for i in range(n):
    A=np.eye(3)
    
    if i == 0:
        A=RotXYZ2RotationMatrix([0,0,0.5*pi])
        v = np.array([0,0,0])
        
        #graphics
        gLink1 =  GraphicsDataOrthoCubePoint(centerPoint= [0.5*L1,0,0], size= [L1,w,w], color=color4dodgerblue)
        gJoint1 = GraphicsDataCylinder([0,0,-1.25*w], [0,0,2.5*w], 0.4*w, color=color4grey)
        
        linkMasses += [J1.Mass()]
        linkCOMs.Append(J1.COM())
        linkInertiasCOM.Append(J1.InertiaCOM())
        
    if i == 1:
        A=RotXYZ2RotationMatrix([0,0,-0.5*pi])
        v = np.array([L1,0,0])
        
        #graphics
        gLink2 =  GraphicsDataOrthoCubePoint(centerPoint= [0.5*L2,0,0], size= [L2,w,w], color=color4dodgerblue)
        gJoint2 = GraphicsDataCylinder([0,0,-1.25*w], [0,0,2.5*w], 0.4*w, color=color4grey)
        
        linkMasses += [J2.Mass()]
        linkCOMs.Append(J2.COM())
        linkInertiasCOM.Append(J2.InertiaCOM())
        
    if i == 2:
        A=RotXYZ2RotationMatrix([0,0,-0.5*pi])
        v = np.array([L2,0,0])
        
        #graphics
        gLink3 =  GraphicsDataOrthoCubePoint(centerPoint= [0.5*L3,0,0], size= [L3,w,w], color=color4dodgerblue)
        gJoint3 = GraphicsDataCylinder([0,0,-1.25*w], [0,0,2.5*w], 0.4*w, color=color4grey)
        # gLink4 =  GraphicsDataOrthoCubePoint(centerPoint= [0,0,0], size= [0,0,0], color=color4dodgerblue) # nur für Joint4
        # gJoint4 = GraphicsDataCylinder([0,0,-1.25*w], [0,0,2.5*w], 0.4*w, color=color4grey)
        
        linkMasses += [J3.Mass()]
        linkCOMs.Append(J3.COM())
        linkInertiasCOM.Append(J3.InertiaCOM())

    #now add joint/link to lists:
    jointTransformations.Append(A)
    jointOffsets.Append(v)

#gList = [[gJoint1,gLink1],[gJoint2,gLink2],[gJoint3,gLink3],[gJoint4,gLink4]] #one list per link; add joint first, then it will be visible with transparency setting
gList = [[gJoint1,gLink1],[gJoint2,gLink2],[gJoint3,gLink3]] #one list per link; add joint first, then it will be visible with transparency setting

#create node for unknowns of KinematicTree
nGeneric = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0.]*n,
                                       initialCoordinates=[0.]*n,
                                       initialCoordinates_t=[0.]*n,
                                       numberOfODE2Coordinates=n))

#create KinematicTree
kt = mbs.AddObject(ObjectKinematicTree(nodeNumber=nGeneric, jointTypes=[exu.JointType.RevoluteZ]*n, linkParents=np.arange(n)-1,
                                  jointTransformations=jointTransformations, jointOffsets=jointOffsets,
                                  linkInertiasCOM=linkInertiasCOM, linkCOMs=linkCOMs, linkMasses=linkMasses,
                                  baseOffset = [0.,0.,0.], gravity=gravity,
                                  #jointForceVector=[0.]*n,
                                  visualization=VObjectKinematicTree(graphicsDataList = gList)))

markerBody = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=kt, linkNumber=2, localPosition=[L3,0,0]))
markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=ground,localPosition = [0,0,0])) # [0,0,0], weil ground schon verschoben ist

lastjoint = mbs.AddObject(GenericJoint(markerNumbers=[markerGround,markerBody], constrainedAxes=[1,1,0,0,0,0]))

tEnd = 10     #end time of simulation
h = 0.005    #step size

# finalize system and assemble equations
mbs.Assemble()

mbs.ComputeSystemDegreeOfFreedom(verbose=True)

simulationSettings = exu.SimulationSettings()
simulationSettings.solutionSettings.writeSolutionToFile=False
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h) #must be integer
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.simulateInRealtime = True

sc.visualizationSettings.general.drawWorldBasis = True

exu.StartRenderer()
mbs.WaitForUserToContinue()

mbs.SolveDynamic(simulationSettings)

sc.WaitForRenderEngineStopFlag()
exu.StopRenderer()