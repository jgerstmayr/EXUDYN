
.. _examples-mouseinteractionexample:

**************************
mouseInteractionExample.py
**************************

You can view and download this file on Github: `mouseInteractionExample.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/mouseInteractionExample.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details: This is the first interactive mouse motion example; 
   #          use your mouse to drag the end of the chain;
   #          models a chain of 3D rigid bodies connected with revolute joints;
   #          may require RESTART of python kernel to work properly!
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-12-06
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.graphicsDataUtilities import *
   
   from math import sin, cos, pi
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   activateWithKeyPress = True #activate mouse drag with keypress 'D'
   
   color = [0.1,0.1,0.8,1]
   
   sx = 0.25
   sy = 0.1
   sz = 0.1
   
   nBodies = 16
   
   Ltot = nBodies*sx
   sc = 4 #size of coordinate system
   coordSys0 = graphics.Cylinder([0,0,0], [sz,0,0], 0.01, color=graphics.color.red)
   coordSys1 = graphics.Cylinder([0,0,0], [0,sz,0], 0.01, color=graphics.color.green)
   coordSys2 = graphics.Cylinder([0,0,0], [0,0,sz], 0.01, color=graphics.color.blue)
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], 
                                      visualization=VObjectGround(graphicsData= [coordSys0, coordSys1, coordSys2])))
   
   m=1 #mass
   density = 2000
   
   nodeList=[]
   objectList=[]
   
   nRB=-1
   com=[0,0,0]
    
   RBinertia = InertiaCuboid(density=density, sideLengths=[sx,sy,sz])
   
   mPosGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oGround, 
                                               localPosition=[0,0,0]))
   mPosLast  = mPosGround
   
   deltaAngleZ = 0.05*pi/nBodies
   p0 = np.array([0,0,0])
   
   #create a chain of bodies:
   for i in range(nBodies):
       
       A = RotationMatrixZ(i*deltaAngleZ)
       deltaRot2 = RotationMatrixZ(0.5*deltaAngleZ)
       p0 = p0 + A@[0.5*sx,0,0]
   
       omega0 = [0,0,0] #arbitrary initial angular velocity
       v0 = [0.,0.,0.] #initial translational velocity
   
       color=[1,0.1,0.1,1]
   
       oGraphics = graphics.BrickXYZ(-sx*0.5,-sy*0.5,-sz*0.5, sx*0.5, sy*0.5, sz*0.5, color)
    
       dictRB = mbs.CreateRigidBody(
                     inertia=RBinertia, 
                     referencePosition=p0, 
                     referenceRotationMatrix=A,
                     initialVelocity=v0,
                     initialAngularVelocity=omega0,
                     gravity=[0., 0., -9.81],
                     graphicsDataList=[oGraphics],
                     returnDict=True)
       [nRB, oRB] = [dictRB['nodeNumber'], dictRB['bodyNumber']]
   
       nodeList += [nRB]
       objectList += [oRB]
       mPos = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB, localPosition = [-0.5*sx,0,0]))
       lastDeltaRot2 = deltaRot2
       if i == 0:
            lastDeltaRot2 = RotationMatrixZ(0)
           
       mbs.AddObject(GenericJoint(markerNumbers = [mPos, mPosLast], 
                                  constrainedAxes=[1,1,1, 1,0,1],
                                  rotationMarker0=lastDeltaRot2.T,
                                  rotationMarker1=lastDeltaRot2,
                                  visualization=VGenericJoint(axesRadius = 0.5*sy, axesLength=1.1*sz)))
   
       #marker for next chain body
       mPosLast = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB, localPosition = [0.5*sx,0,0]))
   
       #add damping to bodies:
       mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers = [mPosGround, mPosLast],
                                                          damping = [40]*3,
                                                          visualization=VCartesianSpringDamper(show=False)))
       
       p0 = p0 + A@[0.5*sx,0,0]
   
   #activate by keypress 'D':
   mbs.variables['activateMouseDrag'] = True
   if activateWithKeyPress:
       mbs.variables['activateMouseDrag'] = False
       
   def UFmouseDrag0(mbs, t, itemIndex, u, v, k, d, offset): #changed 2023-01-21:, mu, muPropZone):
       if not mbs.variables['activateMouseDrag'] == True:
           return 0
       p = SC.GetCurrentMouseCoordinates(True)
       p = SC.renderer.GetState()['openGLcoordinates']
       #print("u=",u)
       return k*(Ltot-0.5*sx+u-p[0]) + d*v
   
   def UFmouseDrag1(mbs, t, itemIndex, u, v, k, d, offset): #changed 2023-01-21:, mu, muPropZone):
       if not mbs.variables['activateMouseDrag'] == True:
           return 0
       p = SC.GetCurrentMouseCoordinates(True)
       return k*(u-p[1]) + d*v
   
   def UFmouseDrag2(mbs, t, itemIndex, u, v, k, d, offset): #changed 2023-01-21:, mu, muPropZone):
       if not mbs.variables['activateMouseDrag'] == True:
           return 0
       p = SC.GetCurrentMouseCoordinates(True)
       return k*(u-p[1]) + d*v
   
   if True:
       nGround = mbs.AddNode(NodePointGround())
       mGroundCoordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate = 0))
       k=5.*1e4
       mCoord0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodeList[-1], coordinate = 0))
       mCoord1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodeList[-1], coordinate = 1))
       mCoord2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodeList[-1], coordinate = 2))
       
       mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mGroundCoordinate, mCoord0], 
                                            stiffness=k, damping=0.01*k, 
                                            springForceUserFunction=UFmouseDrag0,
                                            visualization=VCoordinateSpringDamper(show=False),
                                           ))
       mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mGroundCoordinate, mCoord2], 
                                            stiffness=k, damping=0.01*k, 
                                            springForceUserFunction=UFmouseDrag2,
                                            visualization=VCoordinateSpringDamper(show=False),
                                           ))
   
   mbs.Assemble()
   #exu.Print(mbs)
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 10000
   h = 0.001
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.writeSolutionToFile = False
   simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/1000
   simulationSettings.timeIntegration.verboseMode = 1
   
   #good for interactive examples, as it is independent of CPU power ...
   simulationSettings.timeIntegration.simulateInRealtime = True
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   
   simulationSettings.timeIntegration.newton.absoluteTolerance = 1e2 #if no force acts
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 #0.6 works well 
   
   simulationSettings.solutionSettings.solutionInformation = "mouse interaction example: press 'D' to (de-)activate mouse drag, F2 to switch key functionality"
   
   #+++++++++++++++++++++++++++++++++++
   #these options are not necessary:
   SC.visualizationSettings.nodes.defaultSize = 0.025
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   SC.visualizationSettings.nodes.showBasis = False
   
   SC.visualizationSettings.openGL.light0position = [0.2,0.2,10,0]
   
   SC.visualizationSettings.openGL.light1position = [1,1,-10,0]
   SC.visualizationSettings.openGL.light1ambient= 0. #0.25
   SC.visualizationSettings.openGL.light1diffuse= 0.5 #0.4
   SC.visualizationSettings.openGL.light1specular= 0.6 #0.4
   SC.visualizationSettings.openGL.enableLight1 = True
   
   SC.visualizationSettings.openGL.lightModelTwoSide= True
   
   SC.visualizationSettings.general.drawWorldBasis= True
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.01
   
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.openGL.lineWidth = 2
   SC.visualizationSettings.window.ignoreKeys = True #otherwise keyPressUserFunction not called!
   SC.visualizationSettings.general.useMultiThreadedRendering = True
   
   useGraphics = True
   if useGraphics:
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys['renderState'])
       else:
           renderState = {'centerPoint': [-0.33064934611320496,
                            -0.5762133598327637,
                            0.41875001788139343],
                           'maxSceneSize': 1.75,
                           'zoom': 4.042552471160889,
                           'currentWindowSize': [1024, 768],
                           'modelRotation': [[1.0, 0.0, 0.0],
                            [0.0, -4.371138828673793e-08, -1.0],
                            [0.0, 1.0, -4.371138828673793e-08]],
                           'mouseCoordinates': [713.0, 379.0],
                           'openGLcoordinates': [1.7853742130100727, -0.5235759578645229]}
           SC.renderer.SetState(renderState)
           SC.renderer.SetState(renderState)
       SC.renderer.DoIdleTasks()
   
   #+++++++++++++++++++++++++++++++++++
   #react on key press, in development state:
   #causes crash at termination of python code ...
   def UFkeyPress(key, action, mods):
       #print('key:',key)
       if chr(key) == 'D' and action == 1: #use capital letters for comparison!!! action 1 == press
           mbs.variables['activateMouseDrag'] = not mbs.variables['activateMouseDrag']
       
   if activateWithKeyPress:
       SC.visualizationSettings.window.keyPressUserFunction = UFkeyPress
   
   
   mbs.SolveDynamic(simulationSettings)
   SC.visualizationSettings.window.ResetKeyPressUserFunction()
   
   if useGraphics:
       SC.renderer.Stop() #safely close rendering window!
   
   
   
   


