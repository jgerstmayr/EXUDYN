#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The blend times chosen by Trajectory, the check of the blended velocities and the blending in Cartesian
#           space (#3283): blendTimeWithNext='auto' takes the deceleration time of a profile of constant acceleration
#           followed by one in the same direction, the velocity then staying constant, and less than half the duration
#           of two quintic profiles, whose summed velocities would exceed their peak velocity; with maxVelocities of Trajectory, a blend time whose summed velocities exceed the limits is
#           refused; two straight lines of the tool of the Staubli TX2-90L at a right angle, blended with
#           blendSpace='cartesian', keep the tool in the plane of the lines and its rotation, with velocities and
#           accelerations that are the derivatives of the coordinates, while the blending of the joints lets the tool
#           leave the plane. The result is the number of checks that pass plus the blend times and the closest
#           approach of the tool to the corner.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-06
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.robotics.models import ManipulatorStaubliTX2_90L, LinkDict2Robot
from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration, ProfilePolynomial5, ProfileCartesianLinear
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)
checks = []

#'auto': the deceleration time when the next profile moves on in the same direction at the same speed
auto = Trajectory([0., 0.])
auto.Add(ProfileConstantAcceleration([1., 0.5], 1.), blendTimeWithNext='auto')
auto.Add(ProfileConstantAcceleration([2., 1.], 1.))
checks += [abs(auto.blendTimes[0] - 0.5) < 1e-12 and abs(auto.Evaluate(0.75)[1][0] - 2.) < 1e-12]
#two quintic profiles: the overlap is shorter than half their durations, as the sum would exceed their peak velocity
faster = Trajectory([0.])
faster.Add(ProfilePolynomial5([1.], 1.), blendTimeWithNext='auto')
faster.Add(ProfilePolynomial5([2.], 1.))
vMax = max(abs(faster.Evaluate(t)[1][0]) for t in np.linspace(0, faster.GetTimes()[-1], 401))
checks += [0 < faster.blendTimes[0] < 0.5 and vMax <= 1.875*(1 + 1e-6)]

#a blend time whose velocities exceed maxVelocities is refused, a shorter one accepted
for (blendTime, accepted) in [(1.0, False), (0.5, True)]:
    try:
        limited = Trajectory([0.], maxVelocities=[2.2])
        limited.Add(ProfileConstantAcceleration([1.], 1.), blendTimeWithNext=blendTime)
        limited.Add(ProfileConstantAcceleration([2.], 2.))
        checks += [accepted]
    except ValueError:
        checks += [not accepted]

#two lines of the tool at a right angle, blended in Cartesian and in joint space
robot = LinkDict2Robot(ManipulatorStaubliTX2_90L())
chain = robot.GetKinematicChain()
qA = np.array([-0.5, 0.45, 1.6, 0., 1.1, 0.])
HTA = chain.ToolHT(qA)
pA = np.array(HTA.translation)
deviations = {}
for blendSpace in ['joint', 'cartesian']:
    trajectory = Trajectory(qA)
    trajectory.Add(ProfileCartesianLinear(robot, exu.HT(rotation=HTA.rotation, translation=pA + [0, 0.4, 0]), 0.8,
                                          timeLaw='constantAcceleration'), blendTimeWithNext=0.3, blendSpace=blendSpace)
    trajectory.Add(ProfileCartesianLinear(robot, exu.HT(rotation=HTA.rotation, translation=pA + [-0.3, 0.4, 0]), 0.8,
                                          timeLaw='constantAcceleration'))
    tools = [chain.ToolHT(trajectory.Evaluate(t)[0]) for t in np.linspace(0, trajectory.GetTimes()[-1], 101)]
    deviations[blendSpace] = max(max(abs(H.translation[2] - pA[2]) for H in tools),
                                 max(np.max(np.abs(np.array(H.rotation) - np.array(HTA.rotation))) for H in tools))
    if blendSpace == 'cartesian':
        (t, dt) = (0.7, 1e-5) #in the overlap, 0.5 to 0.8
        [q, q_t, q_tt] = trajectory.Evaluate(t)
        checks += [np.max(np.abs((trajectory.Evaluate(t + dt)[0] - trajectory.Evaluate(t - dt)[0])/(2*dt) - q_t)) < 1e-7
                   and np.max(np.abs((trajectory.Evaluate(t + dt)[1] - trajectory.Evaluate(t - dt)[1])/(2*dt) - q_tt)) < 1e-5]
        blending = trajectory.GetBlendings(chain)[0]
        checks += [np.max(np.abs(trajectory.Evaluate(trajectory.GetTimes()[-1])[0] - trajectory[-1].GetFinalCoordinates())) < 1e-12]
exu.Print('deviation from the plane and the rotation:', deviations)
checks += [deviations['cartesian'] < 1e-12 and deviations['joint'] > 1e-6]

u = sum(checks) + auto.blendTimes[0] + faster.blendTimes[0] + blending['toolMinDistance']
exu.Print('checks:', checks)
exu.Print('solution of trajectoryBlendingCartesianTest=', u)
exu.sys['testResult'] = u
