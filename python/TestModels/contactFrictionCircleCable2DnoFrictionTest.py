#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ObjectContactFrictionCircleCable2D without a friction model - frictionStiffness and
#           frictionVelocityPenalty zero - has no tangential force, whatever the friction coefficient and
#           whatever slip state the data node starts with (#1290). A cable lies on a fixed circle, in contact
#           from the start, its data node initialized to slip; the cable is pulled along its axis. The motion
#           with frictionCoefficient 0.5 equals the one with 0, and the tangential force is zero in every step.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

def Run(frictionCoefficient):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.AddObject(ObjectGround())
    length, nElements = 1., 4
    (b, h, E, rho) = (0.01, 0.01, 2e9, 1000)
    nodes = [mbs.AddNode(Point2DS1(referenceCoordinates=[length/nElements*i,0,1,0])) for i in range(nElements+1)]
    cables = [mbs.AddObject(Cable2D(length=length/nElements, massPerLength=rho*b*h, bendingStiffness=E*b*h**3/12,
                                    axialStiffness=E*b*h, nodeNumbers=[nodes[i], nodes[i+1]])) for i in range(nElements)]
    #gravity, and a pull along the cable at its end
    for (i, node) in enumerate(nodes):
        mbs.AddLoad(Force(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=node)),
                          loadVector=[1.*(i == nElements), -9.81*rho*b*h*length/nElements, 0]))

    radius = 0.3
    mCircle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0.5,-radius-0.001,0]))
    nSegments = 4
    for cable in cables:
        mCable = mbs.AddMarker(MarkerBodyCable2DShape(bodyNumber=cable, numberOfSegments=nSegments))
        #in contact from the start, and in slip: a state that has no meaning without a friction model
        data = mbs.AddNode(NodeGenericData(initialCoordinates=[-0.001]*nSegments + [1]*nSegments + [0]*nSegments,
                                           numberOfDataCoordinates=3*nSegments))
        mbs.AddObject(ObjectContactFrictionCircleCable2D(markerNumbers=[mCircle, mCable], nodeNumber=data,
                                                         numberOfContactSegments=nSegments, contactStiffness=1e4,
                                                         contactDamping=10, frictionStiffness=0, frictionVelocityPenalty=0,
                                                         frictionCoefficient=frictionCoefficient, circleRadius=radius))
    mbs.Assemble()

    mbs.variables['tangential'] = 0.
    def PostStep(mbs, t):
        for i in range(mbs.systemData.NumberOfObjects()):
            if mbs.GetObject(i)['objectType'] == 'ContactFrictionCircleCable2D':
                force = mbs.GetObjectOutput(i, exu.OutputVariableType.ForceLocal)
                mbs.variables['tangential'] = max(mbs.variables['tangential'], np.max(np.abs(force[0::2])))
        return True
    mbs.SetPostStepUserFunction(PostStep)

    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 100
    simulationSettings.timeIntegration.endTime = 0.1
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.solution.file.write = False
    mbs.SolveDynamic(simulationSettings)
    return (mbs.systemData.GetODE2Coordinates(), mbs.variables['tangential'])

(q0, tangential0) = Run(0.)
(q1, tangential1) = Run(0.5)
difference = np.linalg.norm(q1 - q0)
exu.Print('contact without friction: tangential forces', tangential0, tangential1, ', difference of the motion', difference)

u = np.sum(q0) + 1e3*(difference + tangential0 + tangential1)
exu.Print('solution of contactFrictionCircleCable2DnoFrictionTest=', u)

exu.sys['testResult'] = u
