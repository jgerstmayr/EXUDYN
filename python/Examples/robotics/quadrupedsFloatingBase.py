#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ANYmal B and Boston Dynamics Spot side by side in one model, each a kinematic tree on a floating base by Robot.CreateKinematicTree (#3359, #3360),
#           standing on the ground on contact spheres at their feet by their PD control, then lowering the body and rising again. ANYmal comes from
#           exudyn.robotics.models; Spot needs spotData.h5 of the repository CommonData_git beside this one, without it only ANYmal is simulated. The values of the mass of Spot are assumed, as its URDF has none.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-09
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
from exudyn.robotics import Robot, RobotBase, RobotLink, VRobotBase, VRobotLink
from exudyn.robotics.models import QuadrupedANYmalB, LinkDict2Robot
from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration
from exudyn.advancedUtilities import LoadDictFromHDF5
import numpy as np
import os

SC = exu.SystemContainer()
mbs = SC.AddSystem()

g = [0, 0, -9.81]
k, d, friction = 5e4, 500., 0.8
planeSize = 8.

#%%+++ the ground for GeneralContact, at z = 0
gContact = mbs.AddGeneralContact()
gContact.sphereSphereContact = False
gContact.SetFrictionPairings(friction*np.eye(1))
gContact.SetSearchTreeCellSize(numberOfCells=[1, 1, 1])
gContact.SetSearchTreeBox(pMin=[-0.5*planeSize, -0.5*planeSize, -1], pMax=[0.5*planeSize, 0.5*planeSize, 1.5])
gContact.frictionProportionalZone = 0.01

plane = graphics.CheckerBoard([0, 0, 0], size=planeSize, nTiles=1)
mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[graphics.CheckerBoard([0, 0, 0], size=planeSize, nTiles=16)])))
groundMarker = mbs.AddMarker(MarkerNodeRigid(nodeNumber=mbs.AddNode(NodePointGround())))
(points, triangles) = graphics.ToPointsAndTrigs(plane)
gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=groundMarker, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,
                                    pointList=points, triangleList=triangles, staticTriangles=True)

def AddFeet(treeObject, feet, radius):
    """a contact sphere at each foot; feet: pairs of the link number and the center of the sphere in the frame of the link"""
    for (linkNumber, position) in feet:
        marker = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=treeObject, linkNumber=linkNumber, localPosition=position))
        gContact.AddSphereWithMarker(marker, radius=radius, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)

def ControlProfile(robot, q1):
    """hold, go to q1 and back"""
    trajectory = Trajectory(initialCoordinates=robot.q0, initialTime=0)
    trajectory.Add(ProfileConstantAcceleration(list(robot.q0), 0.5))
    trajectory.Add(ProfileConstantAcceleration(list(q1), 0.5))
    trajectory.Add(ProfileConstantAcceleration(list(robot.q0), 0.5))
    return trajectory

robots = []

#%%+++ ANYmal
model = QuadrupedANYmalB()
legsDown = [-0.1, 0.5, -0.9, 0.1, 0.5, -0.9, -0.1, -0.5, 0.9, 0.1, -0.5, 0.9] #knees bent, feet below the hips
model['Pcontrol'] = np.array([1000.]*12)
model['Dcontrol'] = np.array([30.]*12)
anymal = LinkDict2Robot(model)
anymal.q0 = legsDown
footRadius = 0.03
names = [link['name'] for link in model['links']]
for (i, linkDict) in enumerate(model['links']): #each link as a capsule to the joint of its child, or to the foot
    children = [other for other in model['links'] if other['parent'] == i]
    end = list(children[0]['preHT'].translation) if children else list(model['feet'][linkDict['name']])
    link = anymal.links[i]
    link.visualization = VRobotLink(graphicsData=[graphics.Capsule(pAxis=[0, 0, 0], vAxis=end, radius=0.025, color=link.visualization.linkColor)], showCOM=False)

def HeightOfBase(robot, model, shank, position, radius):
    """the height of the base for which the first foot just touches the ground, by the kinematics at q0"""
    frames = {-1: np.eye(4)}
    for (i, link) in enumerate(model['links']):
        T = np.eye(4)
        T = frames[link.get('parent', i - 1)] @ np.array(link['preHT'].HT44() if hasattr(link['preHT'], 'HT44') else link['preHT'])
        q = robot.q0[i]
        (c, s, axis) = (np.cos(q), np.sin(q), link['jointType'][1])
        J = np.eye(4)
        if axis == 'x': J[1:3, 1:3] = [[c, -s], [s, c]]
        elif axis == 'y': J[0, 0], J[0, 2], J[2, 0], J[2, 2] = c, s, -s, c
        else: J[:2, :2] = [[c, -s], [s, c]]
        frames[i] = T @ J
    return radius - min((frames[names.index(s)][:3, :3] @ np.array(p) + frames[names.index(s)][:3, 3])[2] for (s, p) in zip(shank, position))

feetAnymal = list(model['feet'].items())
z = HeightOfBase(anymal, model, [f[0] for f in feetAnymal], [f[1] for f in feetAnymal], footRadius)
anymal.base = RobotBase(HT=exu.HT(translation=[-0.8, 0, z + 0.01]), visualization=VRobotBase(graphicsData=[graphics.Brick(size=[0.56, 0.24, 0.16], color=graphics.color.red)]))
base = model['baseInertia']
treeA = anymal.CreateKinematicTree(mbs, floatingBase=RigidBodyInertia(mass=base['mass'], inertiaTensor=base['inertia'], com=base['COM'], inertiaTensorAtCOM=True))
AddFeet(treeA['objectKinematicTree'], [(names.index(n), p) for (n, p) in feetAnymal], footRadius)
qLow = np.array(legsDown)*1.4
robots += [(anymal, treeA, ControlProfile(anymal, qLow))]

#%%+++ Spot
spotFile = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', 'CommonData_git', 'data', 'Exudyn', 'robotics', 'RobotHDF5', 'spotData.h5')
if os.path.isfile(spotFile):
    data = LoadDictFromHDF5(spotFile)
    legs = [0, 0.63, -0.94]*4
    spot = Robot(gravity=g, base=RobotBase(HT=exu.HT(translation=[0.8, 0, 0.]), visualization=VRobotBase(graphicsData=list(data['graphicsBaseList']))))
    linkNumbers = {'None': -1, 'base': -1, 'base_link': -1}
    for (i, link) in enumerate(data['linkList']):
        linkNumbers[link['name']] = i
        (mass, length) = (0.5, 0.05) if 'hip' in link['name'] else (2., 0.2)
        spot.AddLink(RobotLink(mass=mass, COM=[0, 0, -length], inertia=InertiaCuboid(density=2000, sideLengths=[0.05, 0.05, 0.3]).InertiaCOM(),
                               preHT=link['preHT'], jointType=link['jointType'], parent=linkNumbers[link['parentName']], PDcontrol=(500., 25.),
                               visualization=VRobotLink(graphicsData=link['graphicsDataList'], showMBSjoint=False)))
    spot.q0 = legs
    footLinks = ['fl.lleg', 'fr.lleg', 'hl.lleg', 'hr.lleg']
    spotModel = {'links': [{'parent': linkNumbers[l['parentName']], 'preHT': l['preHT'], 'jointType': l['jointType']} for l in data['linkList']]}
    names = [l['name'] for l in data['linkList']]
    r = 0.05
    position = [0, 0, -0.370 + r]
    z = HeightOfBase(spot, spotModel, footLinks, [position]*4, r)
    spot.base.HT = exu.HT(translation=[0.8, 0, z + 0.01])
    inertia = InertiaCuboid(density=2000, sideLengths=[0.7, 0.2, 0.12])
    treeS = spot.CreateKinematicTree(mbs, floatingBase=RigidBodyInertia(mass=20., inertiaTensor=inertia.InertiaCOM()*20./inertia.Mass()))
    AddFeet(treeS['objectKinematicTree'], [(linkNumbers[n], position) for n in footLinks], r)
    robots += [(spot, treeS, ControlProfile(spot, np.array(legs)*1.3))]
else:
    exu.Print('spotData.h5 is not there: only ANYmal is simulated')

sensors = [mbs.AddSensor(SensorNode(nodeNumber=tree['nodeBase'], storeInternal=True, outputVariableType=exu.OutputVariableType.Position)) for (robot, tree, trajectory) in robots]

def PreStepUserFunction(mbs, t):
    for (robot, tree, trajectory) in robots:
        (q, q_t, unused) = trajectory.Evaluate(t)
        mbs.SetObjectParameter(tree['objectKinematicTree'], 'jointPositionOffsetVector', q)
        mbs.SetObjectParameter(tree['objectKinematicTree'], 'jointVelocityOffsetVector', q_t)
    return True

mbs.SetPreStepUserFunction(PreStepUserFunction)
mbs.Assemble()

tEnd = 2.
stepSize = 1e-4
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.explicit.computeEndOfStepAccelerations = False
simulationSettings.solution.file.writePeriod = 0.02
simulationSettings.solution.sensors.writePeriod = 0.01
SC.visualizationSettings.view0.scene.drawWorldBasis = True
SC.visualizationSettings.openGL.light0.shadow = 0.3
SC.visualizationSettings.bodies.kinematicTree.showJointFrames = False
SC.visualizationSettings.openGL.advanced.initialModelRotation = RotationMatrixZ(0.5) @ RotationMatrixX(0.5*np.pi - 0.3)

mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitMidpoint, showHints=False)
for sensor in sensors:
    heights = mbs.GetSensorStoredData(sensor)[:, 3]
    exu.Print('height of the base: start', heights[0], 'min', heights.min(), 'end', heights[-1])
mbs.SolutionViewer()
