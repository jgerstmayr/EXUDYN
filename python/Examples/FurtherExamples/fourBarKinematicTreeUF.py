#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A four bar mechanism under gravity (with some additional disturbance in x-direction);
#           Uses the KinematicTree with a closing-loop constraint;
#
# Author:   Leonard Schaum
# Date:     2024-05-07
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import exudyn as exu
from exudyn.utilities import *

import numpy as np

from exudyn.robotics import *

SC = exu.SystemContainer()
mbs = SC.AddSystem()


gGround =  GraphicsDataCheckerBoard(point= [0,0,-2], size = 12)
objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0],
                                          visualization=VObjectGround(graphicsData=[gGround])))
markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=objectGround,
                                             localPosition=[0,0,0]))

# Systemkonstanten 
L1, L2, L3 = 0.5, 1.0, 1.0
w = 0.05  # Breite der Glieder
density = 1000  # Dichte der Glieder
gravity = [0, -9.81, 0]  # Schwerkraft
#Antriebsmoment (durch UserFunction angepasst)
torque = 1


graphicsBaseList = [GraphicsDataOrthoCubePoint(size=[0.75*w, 0.75*w, 0.75*w], color=color4grey)] #rail

linkSystem = Robot(gravity=gravity,
              base = RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList)),
              referenceConfiguration = []) #referenceConfiguration created with 0s automatically


linksList = []


Jlink1 = (InertiaCuboid(density=density, sideLengths=[w, L1, w]))
Jlink2 = (InertiaCuboid(density=density, sideLengths=[w, L2, w]))
Jlink3 = (InertiaCuboid(density=density, sideLengths=[w, L3, w]))
Jlink4 = (InertiaCuboid(density=density, sideLengths=[1, 1 , 1]))



Jlink = Jlink1
Jlink = Jlink.Translated([0,0.5*L1,0])
preHT = HT0()

link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                 jointType='Rz', preHT=preHT, 
                 #PDcontrol=(pControl*0, dControl*0),
                 visualization=VRobotLink(linkColor=color4blue))
linkSystem.AddLink(link)
linksList += [copy(link)]



Jlink = Jlink2
Jlink = Jlink.Translated([0,0.5*L2,0])
preHT = HT0()
preHT = HTtranslateY(L1)

link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                 jointType='Rz', preHT=preHT, 
                 #PDcontrol=(pControl*0, dControl*0),
                 visualization=VRobotLink(linkColor=color4blue))
linkSystem.AddLink(link)
linksList += [copy(link)]



Jlink = Jlink3
Jlink = Jlink.Translated([0,0.5*L3,0])
preHT = HT0()
preHT = HTtranslateY(L2)

link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                 jointType='Rz', preHT=preHT, 
                 #PDcontrol=(pControl*0, dControl*0),
                 visualization=VRobotLink(linkColor=color4blue))
linkSystem.AddLink(link)
linksList += [copy(link)]


#4. Rotationsgelenk mit später fixiertem Würfel als Arm
Jlink = Jlink4
Jlink = Jlink.Translated([0,0,0])
preHT = HT0()
preHT = HTtranslateY(L3)

link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                  jointType='Rz', preHT=preHT, 
                  #PDcontrol=(pControl*0, dControl*0),
                  visualization=VRobotLink(linkColor=color4blue))
linkSystem.AddLink(link)
linksList += [copy(link)]


#Startkonfiguration
linkSystem.referenceConfiguration[0] = (2*pi/360) * 0  
linkSystem.referenceConfiguration[1] = (2*pi/360) * 90 
linkSystem.referenceConfiguration[2] = (2*pi/360) * 90 
    



dKT = linkSystem.CreateKinematicTree(mbs)
#dKT = linkSystem.CreateRedundantCoordinateMBS(mbs, baseMarker=markerGround)
oKT = dKT['objectKinematicTree']


#Fixierung des letzten Glieds mit generic Joint an Marker
TCPMarker = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oKT,
                                                   linkNumber=3,
                                                   localPosition=[-1,-0.5,0]))
mbs.AddObject(GenericJoint(markerNumbers=[markerGround, TCPMarker],
                           constrainedAxes=[1,1,0,0,0,1],
                           visualization=VObjectJointGeneric(axesRadius=0.2*w,
                                                             axesLength=1.4*w)))

#User Function for torque of -20 Nm
def UF(mbs, t, load):
    load[2] = 100*(-20-mbs.GetSensorValues(sOmega)[2])
    return load

 # Create Torque to drive the kinematics and Sensor
AntriebMarker = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oKT,
                                                       linkNumber=0,
                                                       localPosition=[0,0,0]))
mbs.AddLoad(Torque(markerNumber=AntriebMarker,
                   loadVector=[0,0,torque],
                   loadVectorUserFunction=UF))
sOmega = mbs.AddSensor(SensorMarker(markerNumber=AntriebMarker,
                                    outputVariableType=exu.OutputVariableType.AngularVelocityLocal,
                                    storeInternal=True))



mbs.Assemble()

#mbs.DrawSystemGraph(useItemTypes=True)

mbs.ComputeSystemDegreeOfFreedom(verbose=True)



SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.general.drawWorldBasis = True

simulationSettings = exu.SimulationSettings()
simulationSettings.linearSolverSettings.ignoreSingularJacobian = True
simulationSettings.linearSolverType = exu.LinearSolverType.EigenDense
#simulationSettings.timeIntegration.simulateInRealtime = True
simulationSettings.timeIntegration.endTime = 2
simulationSettings.timeIntegration.numberOfSteps = 2000
#simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.displayStatistics = True

#solve dynamic problem with default parameters
exu.StartRenderer()
mbs.WaitForUserToContinue()

mbs.SolveDynamic(simulationSettings)

SC.WaitForRenderEngineStopFlag()

mbs.PlotSensor(sensorNumbers=sOmega,components=[2])

exu.StopRenderer() #safely close rendering window!

#visualize solution
# mbs.SolutionViewer()




