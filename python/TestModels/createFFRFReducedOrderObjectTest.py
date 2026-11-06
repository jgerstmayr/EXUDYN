#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test for MainSystem.CreateFFRFReducedOrderObject (#3164): a flexible pendulum from the mesh of
#           testData/netgenTestMesh, the rigid body node selected by rigidBodyNodeType, its hinge at the
#           SuperElementRigid marker of the left face. The marker takes a small offset and, by
#           superElementRigidMarkersLocalHTs, a rotation which turns the free axis of the joint from the marker
#           x-axis to the global z-axis; without this rotation the beam would only twist about its own axis.
#           The function refuses illegal rigid body node types, offsets and transformations.
#           The result is the tip displacement at the end of the simulation, plus 1 if every illegal call was refused.
#
# Author:   Sebastian Weyrer
# Date:     2026-10-30
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.FEM import *
from exudyn import cMath #the math functions of the C++ code, the same bits on every platform
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#%% parameters of the beam
#beam of length L along the x-axis, cross section 2*a x 2*a, midpoint of the left face at the origin
L = 1
a = 0.1
nModes = 4

fem = FEMinterface()
fem.LoadFromFile(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'testData', 'netgenTestMesh'), mode='NPZ')

nodesLeft = fem.GetNodesInPlane([0,0,0], [-1,0,0])
weightsLeft = fem.GetNodeWeightsFromSurfaceAreas(nodesLeft)
nTip = fem.GetNodeAtPoint([L,-a,-a])

#the mesh has no named boundaries, so the boundary is added here; CreateFFRFReducedOrderObject creates one marker for it
fem.nodeSets = [{'Name':'left', 'NodeNumbers':np.array(nodesLeft), 'NodeWeights':np.array(weightsLeft), 'type':'boundary'}]

#the dense solver, because the sparse solver starts from a random vector and gives non-repeatable results
fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=[nodesLeft],
                                  nEigenModes=nModes,
                                  useSparseSolver=False,
                                  boundaryNodesWeights=[weightsLeft])

#the frame of the hinge: the rotation maps the marker x-axis to the z-axis of the body; the area weights give the
#midpoint of the left face, which is the origin, so the hinge is at the offset
A = np.array([[0.,0.,-1.],
              [0.,1., 0.],
              [1.,0., 0.]])
offset = np.array([0.02,0.,0.])

simulationSettings = exu.SimulationSettings()
h = 1e-3
tEnd = 0.2
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
simulationSettings.solution.sensors.writePeriod = h
simulationSettings.solution.file.write = not testIsActive

SC.visualizationSettings.nodes.show = False

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
def RunPendulum(**createArguments):
    """create the pendulum with createArguments for CreateFFRFReducedOrderObject, simulate it and return the norm of
    the tip displacement at the end"""
    mbs.Reset()
    createDict = mbs.CreateFFRFReducedOrderObject(name='pendulum', femInterface=fem,
                                                  gravity=[0,-9.81,0], **createArguments)
    oFFRF = createDict['FFRFReducedOrderObjectDict']['oFFRFreducedOrder']
    mLeft = createDict['pendulum:left']

    oGround = mbs.CreateGround()
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localHT=exu.HT(rotation=A, translation=offset)))
    mbs.AddObject(GenericJoint(markerNumbers=[mGround, mLeft], constrainedAxes=[1,1,1,0,1,1],
                               visualization=VGenericJoint(axesRadius=0.1*a, axesLength=0.5*a)))

    sTip = mbs.AddSensor(SensorSuperElement(bodyNumber=oFFRF, meshNodeNumber=nTip, storeInternal=True,
                                            outputVariableType=exu.OutputVariableType.Displacement))
    mbs.Assemble()

    if not testIsActive:
        SC.renderer.Start()
        SC.renderer.RestoreSavedState() #load last model view
        SC.renderer.DoIdleTasks() #press space to continue

    mbs.SolveDynamic(simulationSettings=simulationSettings)

    if not testIsActive:
        SC.renderer.DoIdleTasks()
        SC.renderer.Stop() #safely close rendering window!

    uTip = mbs.GetSensorValues(sTip)
    exu.Print('tip displacement=', uTip)
    return cMath.norm(uTip)

#%% the illegal arguments, each one refused before anything is added to mbs
illegalArguments = [{'rigidBodyNodeType':'NodeType.notANodeType'},
                    {'rigidBodyNodeType':'NodeType.Position'},                                         #not a rotation node
                    {'superElementRigidMarkersOffsets':'offset'},
                    {'superElementRigidMarkersOffsets':[offset, offset]},                              #two offsets for one marker
                    {'superElementRigidMarkersOffsets':[[0.,0.]]},
                    {'superElementRigidMarkersLocalHTs':exu.HT()},                                     #not a list
                    {'superElementRigidMarkersLocalHTs':[np.eye(3)]},                                  #not 4x4
                    {'superElementRigidMarkersLocalHTs':[exu.HT(), exu.HT()]},                         #two transformations for one marker
                    {'superElementRigidMarkersOffsets':[offset],
                     'superElementRigidMarkersLocalHTs':[exu.HT(rotation=A, translation=2*offset)]},   #disagree
                   ]

allRefused = True
if not exu.__useExudynFast: #the fast module does not check arguments
    for arguments in illegalArguments:
        try:
            RunPendulum(**arguments)
            allRefused = False
            exu.Print('NOT refused:', arguments)
        except ValueError as e:
            exu.Print('refused:', e)

#%% the reference run
result = RunPendulum(rigidBodyNodeType='NodeType.RotationRxyz',
                     superElementRigidMarkersOffsets=[offset],
                     superElementRigidMarkersLocalHTs=[exu.HT(rotation=A, translation=offset)])
result += float(allRefused)

exu.Print('solution of createFFRFReducedOrderObjectTest=', result)

exu.sys['testResult'] = result
