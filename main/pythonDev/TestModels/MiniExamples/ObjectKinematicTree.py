#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectKinematicTree
# 
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
# 
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import sys
sys.path.append('../TestModels')
sys.path.append('../../TestModels') #for direct run in directory

import exudyn as exu
from exudyn.utilities import *

from modelUnitTests import ExudynTestStructure, exudynTestGlobals
import numpy as np

#create an environment for mini example
SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))

#build 1R mechanism (pendulum)
L = 1 #length of link
RBinertia = InertiaCuboid(1000, [L,0.1*L,0.1*L])
inertiaLinkCOM = RBinertia.InertiaCOM() #KinematicTree requires COM inertia
linkCOM = np.array([0.5*L,0.,0.]) #if COM=0, gravity does not act on pendulum!

offsetsList = exu.Vector3DList([[0,0,0]])
rotList = exu.Matrix3DList([np.eye(3)])
linkCOMs=exu.Vector3DList([linkCOM])
linkInertiasCOM=exu.Matrix3DList([inertiaLinkCOM])


nGeneric = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0.],initialCoordinates=[0.],
                                       initialCoordinates_t=[0.],numberOfODE2Coordinates=1))

oKT = mbs.AddObject(ObjectKinematicTree(nodeNumber=nGeneric, jointTypes=[exu.JointType.RevoluteZ], linkParents=[-1],
                                  jointTransformations=rotList, jointOffsets=offsetsList, linkInertiasCOM=linkInertiasCOM,
                                  linkCOMs=linkCOMs, linkMasses=[RBinertia.mass], 
                                  baseOffset = [0.5,0.,0.], gravity=[0.,-9.81,0.]))

#assemble and solve system for default parameters
mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values
simulationSettings.timeIntegration.numberOfSteps = 1000 #gives very accurate results
mbs.SolveDynamic(simulationSettings , solverType=exu.DynamicSolverType.RK67) #highly accurate!

#check final value of angle:
q0 = mbs.GetNodeOutput(nGeneric, exu.OutputVariableType.Coordinates)
#exu.Print(q0)
exudynTestGlobals.testResult = q0 #-3.134018551808591; RigidBody2D with 2e6 time steps gives: -3.134018551809384

exu.Print("example for ObjectKinematicTree completed, test result =", exudynTestGlobals.testResult)

