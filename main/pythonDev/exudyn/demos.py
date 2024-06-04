#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  The demos library includes basic demos which are available directly after installation;
#           For advanced demos, see main/pythonDev/Examples and main/pythonDev/TestModels
#
# Authors:  Johannes Gerstmayr
# Date:     2023-01-12
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import exudyn as exu

def DemoInfo():
    print('\n************************************')
    print('for advanced demos github page:')
    print('https://github.com/jgerstmayr/EXUDYN')
    print('look under main/pythonDev/Examples')
    print('and main/pythonDev/TestModels')
    print('************************************\n')
    

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: very simple demo to show that exudyn is correctly installed; does not require graphics; similar to Examples/myFirstExample.py
def Demo1(showAll = True):
    if showAll:
        print('start demo1: verify that exudyn is running')
    import exudyn as exu               #EXUDYN package including C++ core part
    import exudyn.itemInterface as eii #conversion of data to exudyn dictionaries
    
    SC = exu.SystemContainer()         #container of systems
    mbs = SC.AddSystem()               #add a new system to work with
    
    nMP = mbs.AddNode(eii.NodePoint2D(referenceCoordinates=[0,0]))
    mbs.AddObject(eii.ObjectMassPoint2D(physicsMass=10, nodeNumber=nMP ))
    mMP = mbs.AddMarker(eii.MarkerNodePosition(nodeNumber = nMP))
    mbs.AddLoad(eii.Force(markerNumber = mMP, loadVector=[0.001,0,0]))
    
    mbs.Assemble()                     #assemble system and solve
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.verboseMode=1 #provide some output
    simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/demo1.txt'

    mbs.SolveDynamic(simulationSettings)
    if showAll:
        print('results can be found in local directory: solution/demo1.txt')
    
        DemoInfo()
    
    return [mbs, SC]

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: advanced demo, showing that graphics is available; similar to Examples/rigid3Dexample.py
def Demo2(showAll = True):
    import exudyn as exu
    import exudyn.itemInterface as eii #conversion of data to exudyn dictionaries
    from exudyn.utilities import eulerParameters0, AngularVelocity2EulerParameters_t
    import exudyn.graphics as graphics
    
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    
    if showAll:
        print('EXUDYN version='+exu.GetVersionString())
    
    #%%+++++++++++++++++++++++++++++++++++
    #background
    color = [0.1,0.1,0.8,1]
    zz = 2  #max size
    s = 0.1 #size of cube
    sx = 3*s #x-size
    cPosZ = 0.1 #offset of constraint in z-direction, to get more arbitrary motion

    background0 = graphics.CheckerBoard(point=[0,-2*zz,-0.5*zz],size=8*zz, size2=6.4*zz, nTiles2=8)
    oGround=mbs.AddObject(eii.ObjectGround(referencePosition= [0,0,0], 
                                       visualization=eii.VObjectGround(graphicsData= [background0])))
    mPosLast = mbs.AddMarker(eii.MarkerBodyPosition(bodyNumber = oGround, 
                                                localPosition=[0,0,cPosZ]))
    
    #%%+++++++++++++++++++++++++++++++++++
    #create a chain of 6 bodies:
    for i in range(12):
        #print("Build Object", i)
        ep0 = eulerParameters0 #no rotation
        p0 = [sx+i*2*sx,0.,0] #reference position
    
        nRB = mbs.AddNode(eii.NodeRigidBodyEP(referenceCoordinates=p0+ep0))
        oGraphics = graphics.Brick(size=[1.8*sx, 2*s, 2*s], color= graphics.color.dodgerblue, addEdges=True)
        oGraphicsJoint = graphics.Sphere(point=[-sx,0,cPosZ], radius = 0.6*s, color=graphics.color.darkgrey, 
                                            nTiles=24)
        oRB = mbs.AddObject(eii.ObjectRigidBody(physicsMass=2, 
                                            physicsInertia=[6,1,6,0,0,0], 
                                            nodeNumber=nRB, 
                                            visualization=eii.VObjectRigidBody(graphicsData=[oGraphics, oGraphicsJoint])))
    
        mMassRB = mbs.AddMarker(eii.MarkerBodyMass(bodyNumber = oRB))
        mbs.AddLoad(eii.Gravity(markerNumber = mMassRB, loadVector=[0.,-9.81,0.])) #gravity in negative z-direction
    
        mPos = mbs.AddMarker(eii.MarkerBodyPosition(bodyNumber = oRB, localPosition = [-sx,0.,cPosZ]))
        mbs.AddObject(eii.SphericalJoint(markerNumbers = [mPosLast, mPos]))
        mPosLast = mbs.AddMarker(eii.MarkerBodyPosition(bodyNumber = oRB, localPosition = [sx,0.,cPosZ]))
    
    #%%+++++++++++++++++++++++++++++++++++
    mbs.Assemble()
    # print(mbs)
    
    simulationSettings = exu.SimulationSettings() #takes currently set values or default values
    
    fact = 2000*(1+9*showAll) #10000
    simulationSettings.timeIntegration.numberOfSteps = 1*fact
    simulationSettings.timeIntegration.endTime = 0.001*fact*0.5*4
    simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/fact*20
    simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/chain.txt'
    simulationSettings.timeIntegration.verboseMode = 1
    simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

    simulationSettings.timeIntegration.newton.useModifiedNewton = True
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 
    
    SC.visualizationSettings.general.renderWindowString = "rigid body chain: press 'V' for drawing settings and 'Q' to stop"
    SC.visualizationSettings.nodes.defaultSize = 0.05
    SC.visualizationSettings.general.graphicsUpdateInterval = 0.02

    SC.visualizationSettings.openGL.multiSampling = 4
    SC.visualizationSettings.openGL.lineWidth = 2
    
    SC.visualizationSettings.openGL.shadow = 0.3
    SC.visualizationSettings.openGL.light0position = [4,4,10,0]
    
    if showAll:
        exu.StartRenderer()
        mbs.WaitForUserToContinue()
    
    simulationSettings.timeIntegration.numberOfSteps = 1*fact
    simulationSettings.timeIntegration.endTime = 0.001*fact*0.5*4
    mbs.SolveDynamic(simulationSettings)
    
    if showAll:
        SC.WaitForRenderEngineStopFlag()
        exu.StopRenderer() #safely close rendering window!

    if showAll:
        input("Press Enter to start SolutionViewer...")
    
        from exudyn.interactive import SolutionViewer
        mbs.SolutionViewer()

        DemoInfo()

    return [mbs, SC]


#%%++++++++++++++++++++++++
#testing of demos
if __name__ == '__main__':
    
    Demo1()
    input("Press Enter for Demo2 ...")
    
    Demo2()
    