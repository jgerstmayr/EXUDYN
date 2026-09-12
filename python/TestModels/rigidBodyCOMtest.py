#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test rigid body formulation for different center of mass (COM)
#
# Author:   Johannes Gerstmayr
# Date:     2020-04-22
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
from exudyn.FEM import *

import numpy as np

useGraphics = True #without test
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#you can erase the following lines and all exudynTestGlobals related operations if this is not intended to be used as TestModel:
try: #only if called from test suite
    from modelUnitTests import exudynTestGlobals #for globally storing test results
    useGraphics = exudynTestGlobals.useGraphics
except:
    class ExudynTestGlobals:
        pass
    exudynTestGlobals = ExudynTestGlobals()
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

SC = exu.SystemContainer()
mbs = SC.AddSystem()

nBodies = 2
color = [0.1,0.1,0.8,1]
s = 0.1 #width of cube
sx = 3*s #length of cube/body
cPosZ = 0. #offset of constraint in z-direction
zz = sx * (nBodies+1)*2 #max size of background

background0 = GraphicsDataRectangle(-zz,-zz,zz,2.5*sx,color)
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], 
                                   visualization=VObjectGround(graphicsData= [background0])))

m=25
inertia=np.array([[10,1,2],
                  [ 1,7,3],
                  [ 2,3,6]])

nodeList=[]
objectList=[]
for case in range(2):
    nRB=-1
    if case == 0:
        com=[0,0,0]
    else:
        #com=[0.4,0.6,1.3]
        com=[0.4,0.22,-0.35]
    zOff = 0.5*case*0

    RBinertia = RigidBodyInertia(mass=m, inertiaTensor=inertia)
    #exu.Print("RBinertia orig =", RBinertia)
    RBinertia = RBinertia.Translated(com) #this includes the correct terms in inertia

    if NormL2(RBinertia.com) != 0 and i==1:
        exu.Print("AddRigidBody COM=", RBinertia.com)
        exu.Print("inertia6D=", RBinertia.GetInertia6D())
    #exu.Print("RBinertia trans=", RBinertia)
    #exu.Print("inertia6D=", RBinertia.GetInertia6D())
    #exu.Print("inertia.com=", RBinertia.com)
    oRBlast = oGround

    #create a chain of bodies:
    for i in range(nBodies):
        omega0 = [0,0,0] #arbitrary initial angular velocity
        
        #Rotxyz:
        #ep0 = [0,0,0]
        #ep_t0 = [0,0,0]

        p0 = VSub([i*2*sx+sx,0.,zOff],com) #reference position
        v0 = [0.,0.,0.] #initial translational velocity

        color=[0.8,0.1,0.1,1]
        if case==0:
            color=[0.1,0.1,0.8,1]

        oGraphics = GraphicsDataOrthoCubeLines(-sx+com[0],-s+com[1],-s+com[2], sx+com[0],s+com[1],s+com[2], color)
        d=0.02
        oGraphicsCOM = GraphicsDataOrthoCubeLines(-d+com[0],-d+com[1],-d+com[2], d+com[0],d+com[1],d+com[2], [0.1,0.8,0.1,1])

        rDict = mbs.CreateRigidBody(inertia=RBinertia, 
                                  referencePosition=p0, 
                                  initialVelocity=v0,initialAngularVelocity=omega0, 
                                  gravity=[0.,-9.81,0.],
                                  graphicsDataList=[oGraphics,oGraphicsCOM],returnDict=True)
        oRB = rDict['bodyNumber']
        nRB = rDict['nodeNumber']

        val=0
        if i==0: val=1
        mbs.CreateGenericJoint(bodyNumbers=[oRB, oRBlast], position=VAdd([-sx,0.,0],com), 
                               constrainedAxes=[1,1,1, val,val,0], useGlobalFrame=False)

        #for next chain body
        oRBlast = oRB

    sCoords=mbs.AddSensor(SensorNode(nodeNumber=nRB, storeInternal=True,#fileName="solution/sensor"+str(case)+".txt", 
                             outputVariableType=exu.OutputVariableType.Coordinates))
    nodeList += [nRB]
    objectList += [oRB]

mbs.Assemble()
#exu.Print(mbs)

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

fact = 100
simulationSettings.timeIntegration.numberOfSteps = 1*fact
simulationSettings.timeIntegration.endTime = 0.01*fact
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/1000
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.solutionSettings.writeSolutionToFile = False

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 

simulationSettings.solutionSettings.solutionInformation = "rigid body tests"
SC.visualizationSettings.nodes.defaultSize = 0.025
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.showBasis = True

#simulationSettings.displayComputationTime = True
#simulationSettings.displayStatistics = True


if useGraphics:
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings)



p0=mbs.GetObjectOutputBody(objectList[0], exu.OutputVariableType.Displacement, mbs.GetObject(objectList[0])['physicsCenterOfMass'])
#exu.Print("p0=", p0)
p1=mbs.GetObjectOutputBody(objectList[1], exu.OutputVariableType.Displacement, mbs.GetObject(objectList[1])['physicsCenterOfMass'])
#exu.Print("p1=", p1)

#exu.Print("p0-p1=", p0-p1)
#convergence of two formulations (difference due to time integration):
#h=0.001:  p0-p1= [ 2.89037808e-06 -4.38559926e-07  4.83240595e-07] #similar results for Rxyz parameterization
#h=0.0001: p0-p1= [ 2.88781241e-08 -4.40013365e-09  5.24721844e-09]
#h=0.00001:p0-p1= [ 2.64592348e-10 -5.90557048e-11  4.66975986e-10]

#+++++++++++++++++++++++++++++++++++++++++++++
u=NormL2(p0) + NormL2(p1)
exu.Print('solution of rigidBodyCOMtest=',u)

exudynTestGlobals.testError = u - (3.409431467726293) #2020-04-22: 3.409431467726293
exudynTestGlobals.testResult = u


if useGraphics:
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!





