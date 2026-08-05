#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test for sliding joint and beam attached to FEM mesh
#
# Author:   Johannes Gerstmayr 
# Date:     2026-02-02
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
from exudyn.FEM import HCBstaticModeSelection, FEMinterface, ObjectFFRFreducedOrderInterface, KirchhoffMaterial
from exudyn.beams import GenerateStraightLineANCFCable

SC = exu.SystemContainer()
mbs = SC.AddSystem()

import numpy as np

import time

import netgen.occ as occ
from ngsolve import Mesh, Draw



gravity=[0,0,-9.81]
width = 3
height = 1.5
length = 4
thickness = 0.1
arc = 0.5
bDim = 0.08
bDimX = 2*bDim
rCable = bDim*0.6

carrierWidth = 0.5
carrierHeight = 0.1
carrierLength = width-2*thickness-0.4*bDim
carrierT = 0.05*carrierWidth


maxh=2*thickness #*1 gives fine solution, but takes 5 minutes to simulate
maxhC=0.5*thickness
curvaturesafety=1.5
endTime=3
stepSize = 2e-3


rho = 2800
Emodulus = 8e11
nu = 0.3
materialAlu = KirchhoffMaterial(Emodulus, nu, rho)

interfaceNameList = ['ground']

if True:
    #%%++++++++++++++++++++++++++++++++++++++++++++++++
    #extrusion geometry for frame:
    wp = occ.WorkPlane(occ.Axes(p=(0,0,0), n=occ.X, h=occ.Y))
    #wp.MoveTo(0,0).Line(0.5*width*2).Rotate(90).Line(height).Rotate(45).Line(0.5).Close()
    wp.MoveTo(0,0).Line(0.5*width-arc).Arc(arc,90).Line(height-arc).Rotate(90).Line(thickness).\
            Rotate(90).Line(height-arc).Arc(arc-thickness,-90).Line(width-2*arc).Arc(arc-thickness,-90).\
            Line(height-arc).Rotate(90).Line(thickness).Rotate(90).Line(height-arc).\
            Arc(arc,90).Close()
        
    frame = wp.Face().Extrude(length)
    frame.faces.Min((0,0,1)).name='ground'
    
    vBox = [0.5*bDimX,0.5*bDim,0.5*bDim]
    
    boxList = []
    for fy in [-1,1]:
        for fx in [0,0.5,1]:
            name = 'box_y'+str(fy)+'_x'+str(fx)
            interfaceNameList.append(name)
            pBox = np.array((0.5*bDimX+(length-bDimX)*fx,
                             fy*(-0.5*width+thickness+0.5*bDim),
                             0.5*height))
            box = occ.Box(tuple(pBox-vBox),
                          tuple(pBox+vBox))
            if fy > 0:
                box.faces.Max((0, 1, 0)).name = name
            else:
                box.faces.Min((0, 1, 0)).name = name
            boxList.append(box)
    
    geo = occ.OCCGeometry(frame+boxList[0]+boxList[1]+boxList[2]+boxList[3]+boxList[4]+boxList[5])
    print('meshing ...')
    
    if False:
        import netgen.gui #this starts netgen gui; Press button "Visual" and activate "Auto-redraw after (sec)"; Then select "Mesh"
    
    
    geoMesh = geo.GenerateMesh(maxh=maxh, 
                               curvaturesafety=curvaturesafety, 
                               )
    
    mesh = Mesh(geoMesh)
    
    femInterface = FEMinterface()
    [bfM, bfK, fes] = femInterface.ImportMeshFromNGsolve(mesh, 
                                                         density=rho, youngsModulus=Emodulus, poissonsRatio=nu,
                                                         boundaryNamesList=interfaceNameList,
                                                         meshOrder=2
                                                         )
    print('nNodes=', femInterface.NumberOfNodes())

    [boundaryNodesList, boundaryWeightsList] = femInterface.GetBoundaryNodeSetsAsLists()
    femInterface.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryNodesList,
                                               nEigenModes=8,
                                               excludeRigidBodyMotion=True,
                                               boundaryNodesWeights=boundaryWeightsList,
                                               computationMode=HCBstaticModeSelection.RBE2)
    
    print('eigenfrequencies frame (Hz):\n',np.round(femInterface.GetEigenFrequenciesHz(),2),sep='')
    femInterface.ComputePostProcessingModesNGsolve(fes, materialAlu)
    
    
    createFFRFObjectDict0 = mbs.CreateFFRFReducedOrderObject(name='frame',
                                                            femInterface=femInterface,
                                                            stiffnessProportionalDamping=5e-4,
                                                            gravity=gravity)
    mFrameGround = createFFRFObjectDict0['frame:ground']
    mFrameBoxL = []
    mFrameBoxR = []
    mFrameBoxL.append(createFFRFObjectDict0['frame:box_y-1_x0'])
    mFrameBoxL.append(createFFRFObjectDict0['frame:box_y-1_x0.5'])
    mFrameBoxL.append(createFFRFObjectDict0['frame:box_y-1_x1'])
    mFrameBoxR.append(createFFRFObjectDict0['frame:box_y1_x0'])
    mFrameBoxR.append(createFFRFObjectDict0['frame:box_y1_x0.5'])
    mFrameBoxR.append(createFFRFObjectDict0['frame:box_y1_x1'])
    
    mFrameBox = [mFrameBoxL, mFrameBoxR]
    
    #%%++++++++++++++++++++++++++++++++++++++++++++++++
    
    gFloor = graphics.CheckerBoard(point=[0,0,0.],size=8)
    oGround = mbs.CreateGround(graphicsDataList=[gFloor])
    
    mbs.CreateGenericJoint(bodyNumbers=[mFrameGround,oGround])

#%%++++++++++++++++++++++++++++++++++++++++++++++++
interfaceNameListC = ['attachment']

wp = occ.WorkPlane(occ.Axes(p=(0,0,0), n=-occ.Y, h=occ.X))
wp.MoveTo(0,0).LineTo(carrierT,0)\
              .LineTo(carrierT,carrierHeight-carrierT)\
              .LineTo(carrierWidth-carrierT,carrierHeight-carrierT)\
              .LineTo(carrierWidth-carrierT,0)\
              .LineTo(carrierWidth,0)\
              .LineTo(carrierWidth,carrierHeight)\
              .LineTo(0           ,carrierHeight)\
              .Close()
    
carrier = wp.Face().Extrude(carrierLength).Move((-0.5*carrierWidth,0.5*carrierLength,0))
#frame.faces.Min((0,0,1)).name='ground'
# carrier.faces.Max((0,0,1)).name='attachment'
# carrier.faces.Min((0,0,1)).name='base'

pFront = np.array([0.,-carrierLength*0.5+carrierT*0.5,carrierHeight*0.5])
fBox = np.array([0.5*carrierWidth,0.5*carrierT,0.5*carrierHeight])
frontL = occ.Box(tuple(pFront-fBox),
                 tuple(pFront+fBox))
pFront[1] *= -1
frontR = occ.Box(tuple(pFront-fBox),
                 tuple(pFront+fBox))

hAttach = carrierHeight*0.25*4
wAttach = carrierWidth*0.8

pAttach = np.array([carrierWidth*0.,carrierLength*0.,carrierHeight+hAttach*0.5])
aBox = np.array([0.5*wAttach,0.5*wAttach,0.5*hAttach])

attachment = occ.Box(tuple(pAttach-aBox),
                     tuple(pAttach+aBox))

attachment.faces.Max((0,0,1)).name='attachment'

#connection to sliders:
sDim = 1.6*bDim
sBoxes = []
for ix in [-1,1]:
    for iy in [-1,1]:
        sBox = np.array([0.5*bDim,0.5*sDim,0.5*sDim])
        psBox = np.array([ix*(carrierWidth*0.5-bDim*0.5),iy*(carrierLength*0.5-sDim*0.5),-sDim*0.5])
        #print('psBox=',psBox)
        box = occ.Box(tuple(psBox-sBox),
                      tuple(psBox+sBox))
        
        cyl = occ.Cylinder(tuple(psBox-[0.5*bDim,0,0]), (1,0,0), rCable, bDim)
        iName = 'joint_X'+str(ix)+'_Y'+str(iy)
        cyl.faces[0].name = iName
        interfaceNameListC += [iName]

        sBoxes.append(box-cyl)


geoC = occ.OCCGeometry(carrier+attachment+frontL+frontR
                       +sBoxes[0]+sBoxes[1]+sBoxes[2]+sBoxes[3])

geoMeshC = geoC.GenerateMesh(maxh=maxhC, 
                             curvaturesafety=curvaturesafety, 
                            )

meshC = Mesh(geoMeshC)

femInterfaceC = FEMinterface()
[bfM, bfK, fes] = femInterfaceC.ImportMeshFromNGsolve(meshC, 
                                                     density=rho, youngsModulus=Emodulus, poissonsRatio=nu,
                                                     boundaryNamesList=interfaceNameListC,
                                                     meshOrder=2
                                                     )
print('nNodesC=', femInterfaceC.NumberOfNodes())
[boundaryNodesListC, boundaryWeightsListC] = femInterfaceC.GetBoundaryNodeSetsAsLists()
femInterfaceC.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryNodesListC,
                                            nEigenModes=8,
                                            excludeRigidBodyMotion=True,
                                            boundaryNodesWeights=boundaryWeightsListC,
                                            computationMode=HCBstaticModeSelection.RBE2)

femInterfaceC.ComputePostProcessingModesNGsolve(fes, materialAlu)

print('eigenfrequencies carrier (Hz):\n',np.round(femInterfaceC.GetEigenFrequenciesHz(),2),sep='')


createFFRFObjectDictC = mbs.CreateFFRFReducedOrderObject(name='carrier',
                                                         referencePosition=[carrierWidth*0.5+bDim,
                                                                            carrierLength*0.,
                                                                            0.5*height+0.5*sDim],
                                                         #initialVelocity=[1,0,0],
                                                         femInterface=femInterfaceC,
                                                         stiffnessProportionalDamping=1e-4,
                                                         gravity=gravity)
mCarrierAttachment = createFFRFObjectDictC['carrier:attachment']
mCarrierSlidersLeft = [createFFRFObjectDictC['carrier:joint_X-1_Y-1'],
                       createFFRFObjectDictC['carrier:joint_X1_Y-1']]
mCarrierSlidersRight = [createFFRFObjectDictC['carrier:joint_X-1_Y1'],
                       createFFRFObjectDictC['carrier:joint_X1_Y1']]


if False: #activate to animate modes
    mbs.Assemble()
    from exudyn.interactive import AnimateModes

    SC.visualizationSettings.nodes.show = False
    #SC.visualizationSettings.view0.scene.showFaceEdges = True
    SC.visualizationSettings.openGL.multiSampling=2
    SC.visualizationSettings.openGL.lineWidth=2
    SC.visualizationSettings.view0.window.renderWindowSize = [1600,1080]
    
    objFFRFC = createFFRFObjectDictC['FFRFReducedOrderObjectDict']
    nodeNumber = objFFRFC['nGenericODE2'] #this is the node with the generalized coordinates

    SC.renderer.Start()              #start graphics visualization
    SC.renderer.SetModelView(zoom=2.011357,rotationVector=[-0.9999199,0.6224784,0.9588339],centerPoint=[0.7874073,1.334914,0])
    AnimateModes(SC, mbs, nodeNumber, period=0.1, showTime=False, 
                 renderWindowText='Eigenmodes visualization\n',
                 runOnStart=True)
    import sys
    sys.exit()


#%%++++++++++++++++++++++++++++++++++++++++++++++++
#add beams for rails:

rhoA = 7800*rCable**2*np.pi

fact = 1 #1e-2 #test with softer beam
EA = rCable**2*np.pi*Emodulus*fact
EI = rCable**4*np.pi/4*Emodulus*fact

yBeam = (-0.5*width+thickness+1*bDim)

ancfList = []
for iy in [-1,1]:
    p0 = np.array([0.5*bDimX,iy*yBeam,0.5*height])
    p1 = p0 + [length-bDimX,0,0]

    cable = ObjectANCFCable(physicsMassPerLength=rhoA, 
                  physicsBendingStiffness = EI, 
                  physicsBendingDamping = EI*0.001,
                  physicsAxialStiffness=EA,
                  physicsAxialDamping=EA*0.0001,
                  visualization=VObjectANCFCable(radius = rCable),
                  )

    ancf=GenerateStraightLineANCFCable(mbs=mbs,
                  positionOfNode0=p0, positionOfNode1=p1,
                  numberOfElements=16, #converged to 4 digits
                  cableTemplate=cable, #this defines the beam element properties
                  massProportionalLoad = gravity,
                  #fixedConstraintsNode0 = [1,1,1, 0,1,1], #add constraints for pos and rot (r'_y,r'_z)
                  #fixedConstraintsNode1 = [1,1,1, 0,1,1], #add constraints for pos and rot (r'_y,r'_z)
                  )
    #ancf=[cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList]
    
    mBoxList = mFrameBox[0] if iy==-1 else mFrameBox[1]
    nANCFnodes = len(ancf[0])
    for i, marker in enumerate(mBoxList):
        nANCF = ancf[0][i*(nANCFnodes-1)//2]
        mANCF = mbs.AddMarker(MarkerNodePosition(nodeNumber=nANCF))
        # mbs.CreateCartesianSpringDamper(bodyNumbers=[marker, nANCF],
        #                                 stiffness=[1e5]*3,
        #                                 damping=[2e3]*3)
        mbs.AddObject(SphericalJoint(markerNumbers=[marker, mANCF],
                                     visualization=VSphericalJoint(jointRadius=rCable*1.2)))
        
    ancfList.append(ancf)



#%%++++++++++++++++++++++++++++++++++++++++++++++++
#sliding joint:
sMarkerZ = 0.5*height+0.5*sDim-sDim*0.5 #ideal Z-position
sMarkerY = (carrierLength*0.5-sDim*0.5)

addSlidingJoint = True
if addSlidingJoint:
    for iANCF, markerList in enumerate([mCarrierSlidersRight,mCarrierSlidersLeft]):
        ancf = ancfList[iANCF]
        lElem = mbs.GetObject(ancf[1][0])['physicsLength']

        for i, mSlider in enumerate(markerList):
            pMarker = mbs.GetMarkerOutput(mSlider,exu.OutputVariableType.Position, 
                                          exu.ConfigurationType.Reference)
            #print('pMarker=',pMarker)
            if pMarker[1] < 0: 
                offsetY = -(sMarkerY - abs(pMarker[1]))
            else:
                offsetY = (sMarkerY - abs(pMarker[1]))
            offsetZ = sMarkerZ- pMarker[2]
            mbs.SetMarkerParameter(mSlider, 'offset', [0,offsetY,offsetZ])
            pMarkerCorr = mbs.GetMarkerOutput(mSlider,exu.OutputVariableType.Position, 
                                          exu.ConfigurationType.Reference)
            print('pMarkerCorr=',pMarkerCorr)
            
            slidingCoordinateInit = pMarker[0] #X-coordinate
            initialLocalMarker = int(slidingCoordinateInit/lElem) #second element
            
        
            cableMarkerList = []#list of MarkerBodyBeamShape
            offsetList = []     #list of offsets counted from first cable element; needed in sliding joint
            offset = 0          #first cable element has offset 0
            for item in ancf[1]: #create markers for cable elements
                m = mbs.AddMarker(MarkerBodyBeamShape(bodyNumber = item))
                cableMarkerList += [m]
                offsetList += [offset]
                offset += lElem
        
            nodeDataSJ = mbs.AddNode(NodeGenericData(initialCoordinates=[initialLocalMarker,slidingCoordinateInit],numberOfDataCoordinates=2)) #initial index in cable list
            slidingJoint = mbs.AddObject(ObjectJointSliding(markerNumbers=[mSlider,cableMarkerList[initialLocalMarker]], 
                                                            constrainRotations=[0,0,0],
                                                            slidingMarkerNumbers=cableMarkerList, slidingMarkerOffsets=offsetList, 
                                                            nodeNumber=nodeDataSJ))

#++++++++++++++++++++++++++++++++++++++++++++++++
#driving force and measurement:

def UFspring(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset):
    xDesired = SmoothStep(t, 0.2, 1.2, 0, length-carrierWidth-bDim*4)
    k=1e5
    d=5e3
    forceX = (displacement[0]-xDesired)*k + velocity[0]*d
    return [forceX,0,0]

oGroundCSD = mbs.CreateGround(referencePosition=mbs.GetMarkerOutput(mCarrierAttachment,
                                                                    exu.OutputVariableType.Position, 
                                                                    exu.ConfigurationType.Reference) )

oCSD = mbs.CreateCartesianSpringDamper(bodyNumbers=[oGroundCSD, mCarrierAttachment],
                                       springForceUserFunction=UFspring,
                                       show=False)

mbs.AddLoad(LoadForceVector(markerNumber=mCarrierAttachment, loadVector=[0,0,-2000*9.81]))

sPos = mbs.AddSensor(SensorMarker(markerNumber=mCarrierAttachment, storeInternal=True,
                                  outputVariableType=exu.OutputVariableType.Displacement))

#++++++++++++++++++++++++++++++++++++++++++++++++
mbs.Assemble()

SC.visualizationSettings.nodes.show = False
SC.visualizationSettings.markers.show = True
SC.visualizationSettings.markers.drawSimplified = False
SC.visualizationSettings.markers.defaultSize = 0.01

if False: #activate to animate modes
    mbs.Assemble()
    from exudyn.interactive import AnimateModes

    #SC.visualizationSettings.view0.scene.showFaceEdges = True
    SC.visualizationSettings.openGL.multiSampling=2
    SC.visualizationSettings.openGL.lineWidth=2
    SC.visualizationSettings.view0.window.renderWindowSize = [1600,1080]
    
    objFFRF = createFFRFObjectDict0['FFRFReducedOrderObjectDict']
    nodeNumber = objFFRF['nGenericODE2'] #this is the node with the generalized coordinates

    SC.renderer.Start()              #start graphics visualization
    SC.renderer.SetModelView(zoom=2.011357,rotationVector=[-0.9999199,0.6224784,0.9588339],centerPoint=[0.7874073,1.334914,0])
    AnimateModes(SC, mbs, nodeNumber, period=0.1, showTime=False, 
                 renderWindowText='Eigenmodes visualization\n',
                 runOnStart=True)
    import sys
    sys.exit()


SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.StressLocal
SC.visualizationSettings.contour.outputVariableComponent = -1

SC.visualizationSettings.view0.window.renderWindowSize=[1200,800]
SC.visualizationSettings.general.autoFitScene=False

#SC.visualizationSettings.view0.scene.drawCoordinateSystem = False
SC.visualizationSettings.openGL.lineWidth = 1
SC.visualizationSettings.loads.show = False
SC.visualizationSettings.openGL.light0.position=[2,2,10,1]

SC.visualizationSettings.openGL.lightModelAmbient = [0.6,0.6,0.6,1]
SC.visualizationSettings.openGL.multiSampling = 2
SC.visualizationSettings.openGL.light0.shadow = 0.2
SC.visualizationSettings.openGL.light1.shadow = 0.2
SC.visualizationSettings.raytracer.numberOfThreads = 64

SC.visualizationSettings.view0.camera.useRaytracer = False #set True for raytracing
SC.visualizationSettings.raytracer.keepWindowActive= True
SC.visualizationSettings.raytracer.advanced.searchTreeFactor = 8

simulationSettings = exu.SimulationSettings()

#simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.solutionSettings.solutionWritePeriod = 0.02 #data not used
simulationSettings.solutionSettings.sensorsWritePeriod = stepSize
simulationSettings.timeIntegration.verboseMode = 1 #turn off, because of lots of output
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
# simulationSettings.parallel.numberOfThreads = 4
#simulationSettings.displayComputationTime = True
#simulationSettings.displayStatistics = True

simulationSettings.timeIntegration.numberOfSteps = int(endTime/stepSize)
simulationSettings.timeIntegration.endTime = endTime
simulationSettings.timeIntegration.newton.useModifiedNewton = True


#visualize in Exudyn:
SC.renderer.Start()              #start graphics visualization

SC.renderer.SetModelView(zoom=2.011357,rotationVector=[-0.9999199,0.6224784,0.9588339],centerPoint=[0.7874073,1.334914,0])
SC.renderer.DoIdleTasks() #press space to continue
mbs.SolveDynamic(simulationSettings)

#SC.renderer.Stop() #safely close rendering window!
    
mbs.PlotSensor(sPos, components=[0])
mbs.PlotSensor(sPos, components=[2])

mbs.SolutionViewer()
