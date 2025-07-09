
.. _examples-cmsexamplecourse:

*******************
CMSexampleCourse.py
*******************

You can view and download this file on Github: `CMSexampleCourse.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/CMSexampleCourse.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Create flexible multibody system
   #
   # Author:   Johannes Gerstmayr 
   # Date:     2021-09-10
   # Python version: Python 3.7, 64bits, Anaconda3  + ngsolve + webgui_jupyter_widgets
   # Jupyter:  requires upgrade of scipy and uninstall and install tk (tkinter)
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.FEM import *
   import numpy as np
   from math import sqrt, sin, cos, pi
   
   doMeshing = True #set false if mesh shall be loaded
   useHCBmodes = True #Hurty-Craig-Bampton modes
   computeStresses = False #takes some time
   loadStresses = False #set True only if already computed previously; may lead to severe problems if wrong modes are loaded!!!!
   
   fileName = 'testData/FMBStest1' #for load/save of FEM data
   
   
   # # Parameter Definition
   # ![example geometry](exampleBody.png "Geometry")
   
   #flexible body dimensions:
   femInterface = FEMinterface()
   #geometrical parameters
   ri = 0.010 #radius of hole/bolt
   ra = 0.016 #outer radius
   t  = 0.020 #part thickness, bolt length
   f  = 0.004 #radius of part rounding
   f2 = 0.002 #radius of bolt rounding
   L = 0.250  #distance hole/bolt
   
   hp = ra*sqrt(0.5)-f*(1-sqrt(0.5)) #height of bar
   d1 = (ra+1*f)*sqrt(0.5) #length of rounded part
   L1 = L-2*d1 #length of rectangular section
   
   #number of eigenmodes
   nModes = 8
   
   rho = 1000
   Emodulus=1e8
   nu=0.3
   
   
   # # Create FEM mesh in Netgen
   
   if doMeshing: #needs netgen/ngsolve to be installed to compute mesh, see e.g.: https://github.com/NGSolve/ngsolve/releases
   
       from netgen.occ import *
       #from ngsolve.webgui import Draw #in Jupyter
       from ngsolve import Mesh, Draw
   
       #create part geometry
       wp = WorkPlane(gp_Ax3(p=(0,0,-1.5*t), n=Z, h=X))
   
       wp.Rotate(90).MoveTo(-ra,0).Arc(ra,-135).Arc(f,45).Line(0.5*L1).Line(0.5*L1).Arc(f,45).Arc(ra,-135)
       wp.Arc(ra,-135).Arc(f,45).Line(L1).Arc(f,45).Arc(ra,-135)
       wp.Close().Reverse()
   
       p1 = wp.Face().Extrude(t)
       
       #bolt:
       wp2 = WorkPlane(Axes(p=(0,0,-0.5*t), n=X, h=Y))
       f22 = np.sqrt(2)*f2
       #wp2.MoveTo(0,0).LineTo(ri,0).LineTo(ri,t).LineTo(0,t).LineTo(0,0) #without rounding
       wp2.MoveTo(0,0).Line(ri+f2).Rotate(180).Arc(f2,-90).Line(t-2*f2).Rotate(45).Line(f22).Rotate(45).Line(ri-f2).Rotate(90).Line(t).Close() #with rounding+chamfer
       axis2 = Axis((0,0,0),Z)
       p2 = wp2.Face().Revolve(axis2,360)
   
       #hole:
       wp3 = WorkPlane(Axes(p=(L,0,-1.5*t), n=X, h=Y))
       #wp2.MoveTo(0,0).LineTo(ri,0).LineTo(ri,t).LineTo(0,t).LineTo(0,0) #without rounding
       wp3.MoveTo(0,0).Line(ri+f2).Rotate(135).Line(f22).Rotate(-45).Line(t-2*f2).Rotate(-45).Line(f22).Rotate(135).Line(ri+f2).Rotate(90).Line(t).Close() #with rounding
       axis3 = Axis((L,0,0),Z)
       p3 = wp3.Face().Revolve(axis3,360)
       
       p1 = p1 + p2
       p1 = p1 - p3
   
       #for geometry check:
       #     box = Box((0,0,0), (ri,ri,ri)) #show (0,0,0)
       #     p1 = p1+ box
       geo = OCCGeometry( p1 )
   
       #Jupyter, webgui, draw geometry
       #NEEDS: pip install webgui_jupyter_widgets
       from netgen.webgui import Draw as DrawGeo
       #DrawGeo(geo.shape)
   
       #generate mesh:
       from ngsolve.webgui import Draw
       mesh = Mesh(geo.GenerateMesh(maxh=1.5*f2))
       #Jupyter, webgui, draw mesh
       Draw(mesh)
   
   
   
   # # Import mesh into Exudyn
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   if doMeshing: #needs netgen/ngsolve to be installed to compute mesh, see e.g.: https://github.com/NGSolve/ngsolve/releases
       #save FEM mesh
       femInterface.ImportMeshFromNGsolve(mesh, density=rho, youngsModulus=Emodulus, poissonsRatio=nu)
       femInterface.SaveToFile(fileName)
   else:
       femInterface.LoadFromFile(fileName)
   print("number of nodes = ", femInterface.NumberOfNodes())
   
   
   # In[12]:
   
   
   femInterface.ComputeEigenmodes(nModes, excludeRigidBodyModes = 6, useSparseSolver = True)
   if False: #activate to animate modes
       from exudyn.interactive import AnimateModes
       mbs.Reset()
       cms = ObjectFFRFreducedOrderInterface(femInterface)
   
       objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                                     initialVelocity=[0,0,0], 
                                                     initialAngularVelocity=[0,0,0],
                                                     color=[0.1,0.9,0.1,1.],
                                                     )
       mbs.Assemble()
       SC.visualizationSettings.nodes.show = False
       SC.visualizationSettings.openGL.showFaceEdges = True
       SC.visualizationSettings.openGL.multiSampling=4
       SC.visualizationSettings.openGL.lineWidth=2
       SC.visualizationSettings.window.renderWindowSize = [1600,1080]
   
       #%%+++++++++++++++++++++++++++++++++++++++
       SC.visualizationSettings.general.autoFitScene = False #otherwise, model may be difficult to be moved
   
       nodeNumber = objFFRF['nGenericODE2'] #this is the node with the generalized coordinates
       AnimateModes(SC, mbs, nodeNumber, period=0.1, 
                    scaleAmplitude = 0.02,
                    showTime=False, renderWindowText='Show modes\n',
                    runOnStart=True)
   
   
   # # Define interfaces
   addSensors = True
   pLeft = [0,0,0] #midpoint of bolt
   pRight = [L,0,-t] #midpoint of hole
   pMid = [0.5*L,hp,-0.5*t] #midpoint of bar
   pTip = [L+ra,0,-0.5*t] #midpoint of bar
   
   #%%  
   if addSensors:
       nMid = femInterface.GetNodeAtPoint(pMid, tolerance=1e-2) #tip node (do not use midpoint, as this may not be a mesh node ...)
       print("pMid=",pMid,", nMid=",nMid)
       nTip = femInterface.GetNodeAtPoint(pTip, tolerance=1e-2) #tip node (do not use midpoint, as this may not be a mesh node ...)
       print("pTip=",pTip,", nTip=",nTip)
   
   tV = np.array([0,0,0.5*t])
   nodesLeft = femInterface.GetNodesOnCylinder(pLeft-tV, pLeft+tV, ri)
   # print('nodesLeft=',nodesLeft)
   nodesRight = femInterface.GetNodesOnCylinder(pRight-tV, pRight+tV, ri)
   # print('nodesRight=',nodesRight)
   
   lenNodesLeft = len(nodesLeft)
   weightsNodesLeft = np.array((1./lenNodesLeft)*np.ones(lenNodesLeft))
   
   lenNodesRight = len(nodesRight)
   weightsNodesRight = np.array((1./lenNodesRight)*np.ones(lenNodesRight))
   
   boundaryList = [nodesLeft, nodesRight] #second boudary (right plane) not needed ...
   
   
   # # Compute eigenmodes
   #remark: ComputeEigenmodes requires upgrade of scipy (python -m pip install --upgrade scipy) as compared to Anaconda installation...
   import time
   
   print("compute modes... ")
   start_time = time.time()
   
   if useHCBmodes:
       femInterface.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryList, 
                                     nEigenModes=nModes, 
                                     useSparseSolver=True,
                                     computationMode = HCBstaticModeSelection.RBE2)
   else:
       femInterface.ComputeEigenmodes(nModes, excludeRigidBodyModes = 6, useSparseSolver = True)
   
   print("computation of modes needed %.3f seconds" % (time.time() - start_time))
   print("eigen freq.=", femInterface.GetEigenFrequenciesHz())
   
   
   
   
   # # Compute stresses
   
   femModesName = fileName+'modes'
   if useHCBmodes:
       femModesName+='HCB'
   
   varType = exu.OutputVariableType.StressLocal
   
   if computeStresses:
       mat = KirchhoffMaterial(Emodulus, nu, rho)
       #varType = exu.OutputVariableType.StrainLocal
       print("ComputePostProcessingModes ... (may take a while)")
       start_time = time.time()
       femInterface.ComputePostProcessingModes(material=mat, 
                                      outputVariableType=varType)
       print("   ... needed %.3f seconds" % (time.time() - start_time))
       SC.visualizationSettings.contour.reduceRange=False
       SC.visualizationSettings.contour.outputVariable = varType
       SC.visualizationSettings.contour.outputVariableComponent = 0 #x-component
       #save modes + stresses
       femInterface.SaveToFile(femModesName)
   else:
       if loadStresses:
           femInterface.LoadFromFile(femModesName)
           SC.visualizationSettings.contour.outputVariable = varType
           SC.visualizationSettings.contour.outputVariableComponent = 0 #x-component
   
   
   # # Setup flexible body in exudyn
   cms = ObjectFFRFreducedOrderInterface(femInterface)
   
   objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                                 initialVelocity=[0,0,0], 
                                                 initialAngularVelocity=[0,0,0],
                                                 color=[0.1,0.9,0.1,1.],
                                                 )
   
   
   # # Visualize modes
   if False:
       from exudyn.interactive import AnimateModes
       mbs.Assemble()
       SC.visualizationSettings.nodes.show = False
       SC.visualizationSettings.openGL.showFaceEdges = True
       SC.visualizationSettings.openGL.multiSampling=4
       #SC.visualizationSettings.window.renderWindowSize = [1600,1080]
       SC.visualizationSettings.general.autoFitScene = False #otherwise, model may be difficult to be moved
   
       nodeNumber = objFFRF['nGenericODE2'] #this is the node with the generalized coordinates
       AnimateModes(SC, mbs, nodeNumber, scaleAmplitude = 0.1, runOnStart = True)
   
   
   # # Add gravity
   
   # In[11]:
   
   
   #add gravity (not necessary if user functions used)
   oFFRF = objFFRF['oFFRFreducedOrder']
   mBody = mbs.AddMarker(MarkerBodyMass(bodyNumber=oFFRF))
   mbs.AddLoad(LoadMassProportional(markerNumber=mBody, loadVector= [0,-9.81,0]))
   
   
   # # Add joint constraint
   
   # In[12]:
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add markers and joints
   
   #mRB = mbs.AddMarker(MarkerNodeRigid(nodeNumber=objFFRF['nRigidBody']))
   oGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0]))
   mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oGround, 
                                           localPosition = pLeft))
   
   #add marker
   #NOTE: offset is added in order to compensate for small errors in average node position
   #      because mesh is not fully symmetric and the average node position does not match 
   #      the desired (body-fixed) joint position [0,0,0]; if not used, a small initial jump
   #      happens during simulation when the body moves to the constrained position
   mLeft = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                 meshNodeNumbers=np.array(nodesLeft), #these are the meshNodeNumbers
                                                 weightingFactors=weightsNodesLeft,
                                                 offset=-femInterface.GetNodePositionsMean(nodesLeft),
                                                ))
   oJoint = mbs.AddObject(GenericJoint(markerNumbers=[mGround, mLeft], 
                               constrainedAxes = [1,1,1,1,1,0],
                               visualization=VGenericJoint(axesRadius=0.05*ri, axesLength=1.5*t)))
   # oJoint = mbs.AddObject(RevoluteJointZ(markerNumbers=[mGround, mLeft],
   #                               visualization=VRevoluteJointZ(axisRadius=0.05*ri, axisLength=1.5*t)))
   
   if False: #if this is used, remove offset in MarkerSuperElementRigid above
       #alternative to offset above: compensate joint offset by computation of current displacement in joint: (if not done in MarkerSuperElementRigid)
       mbs.Assemble() #initialize system to compute joint offset
       jointOffset = mbs.GetObjectOutput(oJoint,exu.OutputVariableType.DisplacementLocal)
       print('jointOffset=',jointOffset)
   
       mbs.SetMarkerParameter(mLeft.GetIndex(), 'offset', list(-jointOffset)) #compensate offset; mLeft.GetIndex() because of BUG751
   
       #now check new offset:
       mbs.Assemble() #initialize system to compute joint offset
       jointOffset = mbs.GetObjectOutput(oJoint,exu.OutputVariableType.DisplacementLocal)
       print('jointOffset=',jointOffset)
   
   
   # # Add sensors
   fileDir = 'solution/'
   if addSensors:
       sMidDispl = mbs.AddSensor(SensorSuperElement(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                 meshNodeNumber=nMid, #meshnode number!
                                 fileName=fileDir+'uMid'+str(nModes)+'modes.txt', 
                                 outputVariableType = exu.OutputVariableType.Displacement))
       sTipDispl = mbs.AddSensor(SensorSuperElement(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                 meshNodeNumber=nTip, #meshnode number!
                                 fileName=fileDir+'uTip'+str(nModes)+'modes.txt', 
                                 outputVariableType = exu.OutputVariableType.Displacement))
   
   
   # # Set up visualization
   # (not needed)
   nodeDrawSize = 0.0025 #for joint drawing
   SC.visualizationSettings.nodes.defaultSize = nodeDrawSize
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   SC.visualizationSettings.connectors.defaultSize = nodeDrawSize
   
   SC.visualizationSettings.nodes.show = False
   SC.visualizationSettings.nodes.showBasis = True #of rigid body node of reference frame
   SC.visualizationSettings.nodes.basisSize = t*4
   SC.visualizationSettings.bodies.deformationScaleFactor = 1 #use this factor to scale the deformation of modes
   
   SC.visualizationSettings.openGL.showFaceEdges = True
   SC.visualizationSettings.openGL.showFaces = True
   
   SC.visualizationSettings.sensors.show = True
   SC.visualizationSettings.sensors.drawSimplified = False
   SC.visualizationSettings.sensors.defaultSize = nodeDrawSize*2
   SC.visualizationSettings.markers.drawSimplified = False
   SC.visualizationSettings.markers.show = False
   SC.visualizationSettings.markers.defaultSize = nodeDrawSize*2
   
   SC.visualizationSettings.loads.drawSimplified = False
   SC.visualizationSettings.loads.defaultSize = t*3
   SC.visualizationSettings.loads.defaultRadius = 0.05*t
   
   SC.visualizationSettings.window.renderWindowSize=[1280,720]
   SC.visualizationSettings.openGL.multiSampling = 4
   
   #create animation:
   # simulationSettings.solutionSettings.recordImagesInterval = 0.005
   # SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
   
   
   # # Set up simulation
   mbs.Assemble() #initialize bodies, assemble system; necessary to simulate
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.solutionInformation = "ObjectFFRFreducedOrder test"
   
   h=1e-3
   tEnd = 2
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = h
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.timeIntegration.verboseModeFile = 3
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   simulationSettings.solutionSettings.sensorsWritePeriod = h
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
   #simulationSettings.displayStatistics = True
   simulationSettings.displayComputationTime = True
   
   
   # # Start renderer and Simulate
   
   lifeVisualization = True
   
   if lifeVisualization:
       SC.visualizationSettings.general.autoFitScene=False #if reloaded view settings
       SC.renderer.Start()
       if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
       SC.renderer.DoIdleTasks() #press space to continue
           
   mbs.SolveDynamic(#solverType=exu.DynamicSolverType.TrapezoidalIndex2, 
                     simulationSettings=simulationSettings)
               
   if addSensors:
       uTip = mbs.GetSensorValues(sMidDispl)
       print("nModes=", nModes, ", mid displacement=", uTip)
   
   if lifeVisualization:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   
   # # 3D rendering of FMBS
   if False: #use this to reload the solution and use SolutionViewer
       SC.visualizationSettings.general.autoFitScene=False #if reloaded view settings
   
       
       mbs.SolutionViewer() #can also be entered in IPython ...
   
   
   # # Plot sensor
   
   if addSensors:
       
       mbs.PlotSensor(sensorNumbers=[sMidDispl,sMidDispl,sMidDispl], components=[0,1,2])
   
   
   


