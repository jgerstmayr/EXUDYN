#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  3D rigid body implemented by user function and compared to C++ implementation;
#           Test model for 3D rigid body with Euler parameters modeled with GenericODE2 and CoordinateVectorConstraint;
#           One of the challenges of the example is the inclusion of the Euler parameter constraint
#
# Author:   Johannes Gerstmayr
# Date:     2021-06-28
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

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



zz = 1 #max size
s = 0.1 #size of cube
sx = 3*s #x-size

background0 = GraphicsDataRectangle(-zz,-zz,zz,zz,graphics.color.white)
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], 
                                   visualization=VObjectGround(graphicsData= [background0])))
mPosLast = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oGround, 
                                            localPosition=[-2*sx,0,0]))

omega0 = [0,50.,20] #arbitrary initial angular velocity
ep0 = eulerParameters0 #no rotation

ep_t0 = AngularVelocity2EulerParameters_t(omega0, ep0)

p0 = [0.,0.,0] #reference position
p1 = [s*5,0.,0] #reference position
v0 = [0.2,0.,0.] #initial translational velocity

nRB = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=p1+ep0, 
                                  initialVelocities=v0+list(ep_t0)))

mass = 2
inertia6D = [6,1,6,0,1,0]
g = 9.81

oGraphics = graphics.Brick(centerPoint=[0,0,0], size=[sx,s,s], color=graphics.color.red)
oRB = mbs.AddObject(ObjectRigidBody(physicsMass=mass, 
                                    physicsInertia=inertia6D, 
                                    nodeNumber=nRB, 
                                    visualization=VObjectRigidBody(graphicsData=[oGraphics])))

mMassRB = mbs.AddMarker(MarkerBodyMass(bodyNumber = oRB))
mbs.AddLoad(Gravity(markerNumber = mMassRB, loadVector=[0.,-g,0.])) #gravity in negative z-direction


if True: #rigid body as user function
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #node for mass point:
    useDummyObject = False #set true for an alternative way: use dummy rigid body to realize constraint
    qRef2 = np.array(p0+ep0)
    nRB2 = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=np.array(p0+ep0), #reference coordinates for node2
                                      initialVelocities=v0+list(ep_t0),
                                      addConstraintEquation=useDummyObject)) #do not add algebraic variable here!

    #dummy object, replacement for constraint by using a rigid body with zero mass:
    if useDummyObject:
        oRB2 = mbs.AddObject(ObjectRigidBody(physicsMass=mass*0, 
                                            physicsInertia=np.array(inertia6D)*0, 
                                            nodeNumber=nRB2, 
                                            visualization=VObjectRigidBody(graphicsData=[oGraphics])))

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #equations of motion for rigid body, with COM=[0,0,0]
    M=np.diag([mass,mass,mass])            #translatoric part of mass matrix
    J = Inertia6D2InertiaTensor(inertia6D) #local inertia tensor
    MRB = np.zeros((7,7))
    exu.Print("M =",M)
    exu.Print("J =",J)
    fG = np.array([0,-g*mass,0]+[0]*4)

    def UFgenericODE2(mbs, t, itemIndex, q, q_t):
        f = np.copy(fG)
        #slower, but without global variable: qRef2 = mbs.GetNodeParameter(mbs.GetObjectParameter(itemIndex,'nodeNumbers')[0], 'referenceCoordinates')
        q2 = np.array(q) + qRef2 #q only contains 'change', reference coordinates must be added

        qEP = q2[3:7] #Euler parameters for node
        qEP_t = q_t[3:7] #time derivative of Euler parameters for node
        G = EulerParameters2GLocal(qEP)
        omega = G @ qEP_t

        f[3:7] += -G.T @ Skew(omega) @ J @ omega
        return f
        #exu.Print("t =", t, ", f =", f)

    def UFmassGenericODE2(mbs, t, itemIndex, q, q_t):
        #slower, but without global variable: qRef2 = mbs.GetNodeParameter(mbs.GetObjectParameter(itemIndex,'nodeNumbers')[0], 'referenceCoordinates')
        q2 = np.array(q) + qRef2 #q only contains 'change', reference coordinates must be added
        qEP = q2[3:7] #Euler parameters for node
        G = EulerParameters2GLocal(qEP)

        MRB[0:3,0:3] = M            #translational part
        MRB[3:7,3:7] = G.T @ J @ G  #rotational part
        return MRB

    #add visualization for rigid body: note that transformation from local to global coordinates needs to be done as well
    def UFgraphics(mbs, itemNumber):
        n = mbs.GetObjectParameter(itemNumber, 'nodeNumbers')[0]
        p0 = mbs.GetNodeOutput(nodeNumber=n, variableType=exu.OutputVariableType.Position, configuration=exu.ConfigurationType.Visualization)
        A = mbs.GetNodeOutput(nodeNumber=n, variableType=exu.OutputVariableType.RotationMatrix, configuration=exu.ConfigurationType.Visualization)
        
        A0 = np.reshape(A, (3,3))
        graphics1 = graphics.Move(oGraphics, p0, A0)
        return [graphics1]

    mbs.AddObject(ObjectGenericODE2(nodeNumbers = [nRB2], 
                                    forceUserFunction=UFgenericODE2, massMatrixUserFunction=UFmassGenericODE2,
                                    visualization=VObjectGenericODE2(graphicsDataUserFunction=UFgraphics)))

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #add Euler parameter constraint
    if not useDummyObject:
        nG = mbs.AddNode(NodePointGround(visualization=VNodePointGround(show=False)))
        mNodeGround = mbs.AddMarker(MarkerNodeCoordinates(nodeNumber=nG))
        mRB2 = mbs.AddMarker(MarkerNodeCoordinates(nodeNumber=nRB2))

        #q0^2+q1^2+q2^2+q3^2 - 1 = 0
        mbs.AddObject(CoordinateVectorConstraint(markerNumbers=[mNodeGround, mRB2],
                                                 scalingMarker0=[], scalingMarker1=[],
                                                 quadraticTermMarker0=[], quadraticTermMarker1=np.array([[0,0,0,1,1,1,1]]),
                                                 offset=[1],
                                                 visualization=VCoordinateVectorConstraint(show=False)))
#end: user function for rigid body
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


mbs.Assemble()
exu.Print(mbs)

simulationSettings = exu.SimulationSettings()

#useGraphics=False
tEnd = 0.05
h = 1e-3
if useGraphics:
    tEnd = 1

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
#simulationSettings.solutionSettings.solutionWritePeriod = h
simulationSettings.timeIntegration.verboseMode = 1
#simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8 #SHOULD work with 0.9 as well

SC.visualizationSettings.nodes.showBasis=True

if useGraphics:
    SC.renderer.Start()

mbs.SolveDynamic(simulationSettings)


u0 = mbs.GetNodeOutput(nRB, exu.OutputVariableType.Displacement)
rot0 = mbs.GetNodeOutput(nRB, exu.OutputVariableType.Rotation)
exu.Print('u0=',p0,', rot0=', rot0)

u1 = mbs.GetNodeOutput(nRB2, exu.OutputVariableType.Displacement)
rot1 = mbs.GetNodeOutput(nRB2, exu.OutputVariableType.Rotation)
exu.Print('u1=',p1,', rot1=', rot1)

result = (abs(u1+u0)+abs(rot1+rot0)).sum()
exu.Print('solution of rigidBodyAsUserFunctionTest=',result)

exudynTestGlobals.testError = result - (8.950865271552146) #2020-06-28: 8.950865271552146
exudynTestGlobals.testResult = result

if useGraphics:
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

