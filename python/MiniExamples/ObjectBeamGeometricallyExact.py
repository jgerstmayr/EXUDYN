#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectBeamGeometricallyExact
# 
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
# 
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics

import numpy as np

#create an environment for mini example
SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))

#a cantilever of four 3D geometrically exact beam elements on Euler parameter nodes, loaded at the tip
L = 1; nElements = 4; EI = 100; GA = 1e4; F = -0.1
section = exu.BeamSection()
section.stiffnessMatrix = np.diag([1e5, GA, GA, 80, EI, EI]) #EA, GA_y, GA_z, GJ, EI_y, EI_z
section.inertia = np.diag([0.02, 0.01, 0.01])
section.massPerLength = 1
nodes = [mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[L*i/nElements,0,0]+eulerParameters0))
         for i in range(nElements+1)]
for i in range(nElements):
    mbs.AddObject(ObjectBeamGeometricallyExact(nodeNumbers=[nodes[i],nodes[i+1]], physicsLength=L/nElements,
                                               sectionData=section))
mbs.AddObject(GenericJoint(markerNumbers=[mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGround)),
                                          mbs.AddMarker(MarkerNodeRigid(nodeNumber=nodes[0]))])) #clamped
mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=nodes[-1])), loadVector=[0,F,0]))
mbs.Assemble()
mbs.SolveStatic()
#Timoshenko beam: F*L^3/(3*EI) + F*L/GA = -0.3433e-3, approached with more elements
exu.sys['testResult'] = mbs.GetNodeOutput(nodes[-1], exu.OutputVariableType.Displacement)[1]*1000

exu.Print("example for ObjectBeamGeometricallyExact completed, test result =", exu.sys['testResult'])

