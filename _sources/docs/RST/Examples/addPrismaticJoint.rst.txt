
.. _examples-addprismaticjoint:

********************
addPrismaticJoint.py
********************

You can view and download this file on Github: `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/addPrismaticJoint.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Create a chain of bodies connected with prismatic joints; Example for CreatePrismaticJoint utility function
   #
   # Model:    N-link chain of rigid bodies connected by prismatic joints
   #
   # Author:   Johannes Gerstmayr 
   # Date:     2021-07-02
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   # *clean example*
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   ## import libaries
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   from math import sin, cos, pi
   import numpy as np
   
   ## set up mbs
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   ## define overall parameters
   L = 0.4 #length of bodies
   d = 0.1 #diameter of bodies
   p0 = [0.,0.,0] #reference position
   vLoc = np.array([L,0,0]) #last to next joint
   g = [0,-9.81,0]
   
   ## create ground with marker
   oGround=mbs.AddObject(ObjectGround(referencePosition= [-0.5*L,0,0])) 
   mPosLast = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oGround, localPosition=[0,0,0]))
   bodyLast = oGround
   
   ## set up rotation matrices for relative rotation of joints
   A0 = np.eye(3)
   Alast = A0 #previous marker
   
   A0 = RotationMatrixX(0)
   A1 = RotationMatrixY(0.5*pi)
   A2 = RotationMatrixZ(0.5*pi)
   A3 = RotationMatrixX(-0.5*pi)
   Alist=[A0,A1,A2,A3]
   
   ## set up list of vectors defining axes
   v0=[0,0,1]
   v1=[1,1,1]
   v2=[1,0,0]
   v3=[0,0,1]
   axisList=[v0,v1,v2,v3]
   
   ## loop to create a chain of 4 bodies under gravity connected with prismatic joints
   for i in range(4):
       ### create inertia for block with dimensions [L,d,d]
       inertia = InertiaCuboid(density=1000, sideLengths=[L,d,d])
       ### create graphics for body as block with (body-fixed) centerPoint, size and color
       graphicsBody = graphics.Brick(centerPoint=[0,0,0], size=[0.96*L,d,d], color=graphics.color.steelblue)
   
       ### create and add rigid body to mbs
       p0 += Alist[i] @ (0.5*vLoc)
       oRB = mbs.CreateRigidBody(inertia=inertia, 
                                 referencePosition=p0,
                                 referenceRotationMatrix=Alist[i],
                                 gravity=g,
                                 graphicsDataList=[graphicsBody])
       nRB= mbs.GetObject(oRB)['nodeNumber']
   
       body0 = bodyLast
       body1 = oRB
       ### retrieve reference position for simpler definition of global joint position
       point = mbs.GetObjectOutputBody(oRB,exu.OutputVariableType.Position,
                                       localPosition=[-0.5*L,0,0],
                                       configuration=exu.ConfigurationType.Reference)
       axis = axisList[i]
       ### set up prismatic joint between two bodies, at global position and with global axis
       mbs.CreatePrismaticJoint(bodyNumbers=[body0, body1], position=point, axis=axis, 
                                useGlobalFrame=True, axisRadius=0.6*d, axisLength=1.2*d)
   
       bodyLast = oRB
       
       p0 += Alist[i] @ (0.5*vLoc)
       Alast = Alist[i]
   
   ## assemble and set up simulation settings
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 2
   h=0.001  #use small step size to detext contact switching
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = 0.005
   #simulationSettings.timeIntegration.simulateInRealtime = True
   simulationSettings.timeIntegration.realtimeFactor = 0.5
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
   simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   SC.visualizationSettings.nodes.show = True
   SC.visualizationSettings.nodes.drawNodesAsPoint  = False
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.015
   SC.visualizationSettings.connectors.showJointAxes = True
   
   #for snapshot:
   SC.visualizationSettings.openGL.multiSampling=4
   SC.visualizationSettings.openGL.lineWidth=2
   
   SC.visualizationSettings.general.autoFitScene = False #use loaded render state
   useGraphics = True
   if useGraphics:
       simulationSettings.displayComputationTime = True
       simulationSettings.displayStatistics = True
       SC.renderer.Start()
       ## reload previous render configuration
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys[ 'renderState' ])
   else:
       simulationSettings.solutionSettings.writeSolutionToFile = False
   
   ## start solver
   mbs.SolveDynamic(simulationSettings, showHints=True)
   
   ## visualization
   mbs.SolutionViewer(runMode=2, runOnStart=True)
   
   ## evaluate some results
   u0 = mbs.GetNodeOutput(nRB, exu.OutputVariableType.Displacement)
   rot0 = mbs.GetNodeOutput(nRB, exu.OutputVariableType.Rotation)
   exu.Print('u0=',u0,', rot0=', rot0)
   
   result = (abs(u0)+abs(rot0)).sum()
   exu.Print('solution of addPrismaticJoint=',result)
   
   


