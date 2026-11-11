#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ObjectJointHelicalZ (#3421, #3422), the helical (screw) joint: (1) a rigid body on a screw about the x-axis
#           of the ground (the joint frame turned by the localHT of the markers) under a torque and an axial force,
#           through two and a half turns, against the analytic solution phi = (M + F h) t^2 / (2 (Jxx + m h^2)) at
#           index 3 and index 2, for a right- and a left-handed thread; the continuous angle of the output variable
#           Rotation and the multiplier of the coupling against F - m h phi_tt; (2) the same screw as ObjectJointUser with the
#           constraint of userJointTest.py (#3100); (3) the pitch zero against ObjectJointRevoluteZ, the coupling free
#           against the cylindrical ObjectJointGeneric; (4) a static solve: an axial spring against the screw under a
#           torque, phi = M/(k h^2); (5) mbs.CreateHelicalJoint (#3423), activeConnector=False and what the joint refuses.
#           Run alone, it draws the screw and opens the SolutionViewer.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-11
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics
from exudyn import cMath #the math functions of the C++ code, the same bits on every platform
import numpy as np
testIsActive = exu.sys.get('testIsActive', False) #run alone: the renderer, the solution file and the SolutionViewer

SC = exu.SystemContainer()

endTime = 1.
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 500
simulationSettings.timeIntegration.endTime = endTime
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-12
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-12
simulationSettings.solution.file.write = False

inertia = InertiaCuboid(density=1000, sideLengths=[0.2, 0.1, 0.1])
m = inertia.Mass()
Jxx = inertia.Inertia()[0, 0]
pitch = 0.05
torque = 0.25 #about 15.4 rad at t=1, two and a half turns
axialForce = 0.1
Rzx = RotationMatrixY(0.5*np.pi) #turns the joint axis z into the global x

def AddScrewBody(mbs):
    node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0, 0, 0] + list(eulerParameters0)))
    body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=m, inertia=inertia.GetInertia6D(),
                                         visualization=VObjectRigidBody(graphicsData=[graphics.Brick(size=[0.2, 0.1, 0.1], color=graphics.color.steelblue, addEdges=True)])))
    mbs.AddLoad(LoadTorqueVector(markerNumber=mbs.AddMarker(MarkerBodyRigid(bodyNumber=body)), loadVector=[torque, 0, 0]))
    mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerBodyRigid(bodyNumber=body)), loadVector=[axialForce, 0, 0]))
    return node, body

def Solve(mbs, index2=False, show=False):
    mbs.Assemble()
    simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = index2 #velocity level, with Newmark
    simulationSettings.timeIntegration.generalizedAlpha.useNewmark = index2
    if show and not testIsActive:
        simulationSettings.solution.file.write = True
        SC.renderer.Start()
    mbs.SolveDynamic(simulationSettings)
    if show and not testIsActive:
        SC.renderer.Stop()
        mbs.SolutionViewer() #it reads the solution file of this run
        simulationSettings.solution.file.write = False
    simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
    simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(1) the helical joint against the analytic solution
def SolveHelical(h, index2, show=False):
    mbs = SC.AddSystem()
    node, body = AddScrewBody(mbs)
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=mbs.AddObject(ObjectGround()), localHT=exu.HT(rotation=Rzx)))
    mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localHT=exu.HT(rotation=Rzx)))
    joint = mbs.AddObject(ObjectJointHelicalZ(markerNumbers=[mGround, mBody], pitch=h,
                                              visualization=VHelicalJointZ(axisRadius=0.02, axisLength=0.6)))
    Solve(mbs, index2, show)
    return mbs, node, joint

differenceAnalytic = 0
u = 0
for h in [pitch, -pitch]:
    phiAnalytic = (torque + axialForce*h)*endTime**2/(2*(Jxx + m*h**2))
    phi_tt = 2*phiAnalytic/endTime**2
    for index2 in [False, True]:
        mbs, node, joint = SolveHelical(h, index2, show=(h > 0 and not index2))
        A = np.array(mbs.GetNodeOutput(node, exu.OutputVariableType.RotationMatrix)).reshape(3, 3)
        x = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[0]
        angle = mbs.GetObjectOutput(joint, exu.OutputVariableType.Rotation)[2] #continuous: two and a half turns
        axial = mbs.GetObjectOutput(joint, exu.OutputVariableType.ForceLocal)[2]
        differences = [np.max(np.abs(A - RotationMatrixX(phiAnalytic))), abs(x - h*phiAnalytic), abs(angle - phiAnalytic)]
        if not index2: #the multiplier at index 3; index 2 has the multipliers of the velocity level
            differences += [abs(axial - (axialForce - m*h*phi_tt))]
        differenceAnalytic = max(differenceAnalytic, max(differences))
        u += angle + x + 0.1*axial
        exu.Print('helical joint h =', h, ', index 2' if index2 else ', index 3', ': angle', angle, 'analytic', phiAnalytic,
                  ', axial multiplier', axial, 'analytic', axialForce - m*h*phi_tt, ', largest difference', max(differences))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(2) the same screw as ObjectJointUser with the constraint of userJointTest.py (#3100), about the x-axis of the ground
def UFscrew(mbs, t, itemNumber, HT0, HT1, parameters):
    relative = HT0.Relative(HT1)
    (Arel, d) = (relative.rotation, relative.translation)
    phi = d[0]/parameters[0]
    return [Arel[0, 1], Arel[0, 2], d[1], d[2], parameters[0]*(Arel[2, 1]*cMath.cos(phi) - Arel[1, 1]*cMath.sin(phi))]

def UFscrewJacobian(mbs, t, itemNumber, HT0, HT1, parameters):
    A0, A1 = HT0.rotation, HT1.rotation
    r = HT1.translation - HT0.translation
    Arel = A0.T @ A1
    h = parameters[0]
    phi = (A0.T @ r)[0]/h
    jacobian = np.zeros((5, 12))
    def AddArel(row, i, j, factor):
        g = factor*np.cross(A1[:, j], A0[:, i])
        jacobian[row, 3:6] -= g
        jacobian[row, 9:12] += g
    def AddD(row, k, factor):
        jacobian[row, 0:3] -= factor*A0[:, k]
        jacobian[row, 6:9] += factor*A0[:, k]
        jacobian[row, 3:6] += factor*(A0.T @ Skew(r))[k]
    AddArel(0, 0, 1, 1.)
    AddArel(1, 0, 2, 1.)
    AddD(2, 1, 1.)
    AddD(3, 2, 1.)
    AddArel(4, 2, 1, h*cMath.cos(phi))
    AddArel(4, 1, 1, -h*cMath.sin(phi))
    AddD(4, 0, -Arel[2, 1]*cMath.sin(phi) - Arel[1, 1]*cMath.cos(phi))
    return jacobian

mbsUser = SC.AddSystem()
nodeUser, bodyUser = AddScrewBody(mbsUser)
mbsUser.AddObject(ObjectJointUser(markerNumbers=[mbsUser.AddMarker(MarkerBodyRigid(bodyNumber=mbsUser.AddObject(ObjectGround()))),
                                                 mbsUser.AddMarker(MarkerBodyRigid(bodyNumber=bodyUser))],
                                  numberOfConstraints=5, parameters=[pitch], constraintUserFunction=UFscrew,
                                  jacobianUserFunction=UFscrewJacobian))
Solve(mbsUser)
mbs, node, joint = SolveHelical(pitch, False)
differenceUser = (np.max(np.abs(np.array(mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates))
                                - np.array(mbsUser.GetNodeOutput(nodeUser, exu.OutputVariableType.Coordinates)))))
exu.Print('helical joint against ObjectJointUser: largest difference of the coordinates', differenceUser)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(3) pitch zero against ObjectJointRevoluteZ, the coupling free against the cylindrical ObjectJointGeneric
def SolveReduced(jointType):
    mbs = SC.AddSystem()
    node, body = AddScrewBody(mbs)
    mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localPosition=[0.1, 0.05, 0])), loadVector=[0, 0.2, 0.3]))
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=mbs.AddObject(ObjectGround()), localHT=exu.HT(rotation=Rzx)))
    mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localHT=exu.HT(rotation=Rzx)))
    if jointType == 'helical0':
        mbs.AddObject(ObjectJointHelicalZ(markerNumbers=[mGround, mBody], pitch=0))
    elif jointType == 'revolute':
        mbs.AddObject(ObjectJointRevoluteZ(markerNumbers=[mGround, mBody]))
    elif jointType == 'helicalCylindrical':
        mbs.AddObject(ObjectJointHelicalZ(markerNumbers=[mGround, mBody], pitch=pitch, constrainedAxes=[1,1,0,1,1]))
    elif jointType == 'genericCylindrical':
        mbs.AddObject(ObjectJointGeneric(markerNumbers=[mGround, mBody], constrainedAxes=[1,1,0,1,1,0]))
    Solve(mbs)
    return np.array(mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates))

differenceRevolute = np.max(np.abs(SolveReduced('helical0') - SolveReduced('revolute')))
qCylindrical = SolveReduced('helicalCylindrical')
differenceCylindrical = np.max(np.abs(qCylindrical - SolveReduced('genericCylindrical')))
exu.Print('helical joint, pitch 0 against ObjectJointRevoluteZ:', differenceRevolute,
          ', coupling free against the cylindrical ObjectJointGeneric:', differenceCylindrical)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(4) static: an axial spring of stiffness k against the screw under the torque M, phi = M/(k h^2)
mbs = SC.AddSystem()
oGround = mbs.CreateGround()
bNut = mbs.CreateRigidBody(inertia=inertia, referencePosition=[0, 0, 0])
oHelical = mbs.CreateHelicalJoint(itemNumbers=[oGround, bNut], position=[0, 0, 0], axis=[1, 0, 0], pitch=pitch,
                                  axisRadius=0.02, axisLength=0.6)
k = 400.
mbs.CreateCartesianSpringDamper(itemNumbers=[oGround, bNut], localPosition0=[0, 0, 0], localPosition1=[0, 0, 0], stiffness=[k, 0, 0], damping=[0, 0, 0])
mbs.CreateTorque(bodyNumber=bNut, loadVector=[torque, 0, 0])
mbs.Assemble()
staticSettings = exu.SimulationSettings()
staticSettings.staticSolver.numberOfLoadSteps = 10
staticSettings.staticSolver.newton.relativeTolerance = 1e-12
staticSettings.staticSolver.newton.absoluteTolerance = 1e-12
mbs.SolveStatic(staticSettings)
angleStatic = mbs.GetObjectOutput(oHelical, exu.OutputVariableType.Rotation)[2]
differenceStatic = abs(angleStatic - torque/(k*pitch**2))
exu.Print('helical joint, static: angle', angleStatic, 'analytic', torque/(k*pitch**2), ', difference', differenceStatic)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(5) activeConnector=False and what the joint refuses
mbs = SC.AddSystem()
oGround = mbs.CreateGround()
bNut = mbs.CreateRigidBody(inertia=inertia, referencePosition=[0, 0, 0])
oInactive = mbs.CreateHelicalJoint(itemNumbers=[oGround, bNut], position=[0, 0, 0], axis=[1, 0, 0], pitch=pitch)
mbs.SetObjectParameter(oInactive, 'activeConnector', False)
mbs.CreateTorque(bodyNumber=bNut, loadVector=[torque, 0, 0])
mbs.Assemble()
simulationSettings.timeIntegration.numberOfSteps = 10
simulationSettings.timeIntegration.endTime = 0.1
mbs.SolveDynamic(simulationSettings)
inactiveForce = np.max(np.abs(mbs.GetObjectOutput(oInactive, exu.OutputVariableType.ForceLocal)))
exu.Print('helical joint, activeConnector=False: largest multiplier', inactiveForce)

refused = 0
mbs = SC.AddSystem()
oGround = mbs.CreateGround()
bNut = mbs.CreateRigidBody(inertia=inertia, referencePosition=[0, 0, 0])
mbs.CreateHelicalJoint(itemNumbers=[oGround, bNut], position=[0, 0, 0], axis=[1, 0, 0], pitch=pitch, constrainedAxes=[1,1,2,1,1])
try:
    mbs.Assemble()
except Exception as e:
    refused += 1
    exu.Print('refused as expected:', str(e).split('\n')[0][:120])
try:
    mbs.CreateHelicalJoint(itemNumbers=[oGround, mbs.AddMarker(MarkerNodePosition(nodeNumber=0))], position=[], axis=[1, 0, 0], pitch=pitch)
except Exception as e:
    refused += 1
    exu.Print('refused as expected:', str(e).split('\n')[0][:120])

#the analytic and the comparisons are exact to the time integration error / the Newton tolerance; the sum carries them
exu.Print('helicalJointTest: analytic', differenceAnalytic, ', user joint', differenceUser, ', revolute', differenceRevolute,
          ', cylindrical', differenceCylindrical, ', static', differenceStatic, ', inactive', inactiveForce, ', refused', refused)
u += 0.01*np.sum(qCylindrical) + angleStatic + refused
if max(differenceUser, differenceRevolute, differenceCylindrical, differenceStatic, inactiveForce) > 1e-8 or differenceAnalytic > 5e-3:
    u += 1000 #a failure shows in the result
exu.Print('solution of helicalJointTest=', u)
exu.sys['testResult'] = u
