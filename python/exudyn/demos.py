#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  The demos library includes basic demos which are available directly after installation;
#           For advanced demos, see python/Examples and python/TestModels
#
# Authors:  Johannes Gerstmayr
# Date:     2023-01-12
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os

import exudyn

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'demoSolutionDirectory', 'DemoSolutionFile', 'DemoInfo', 'Demo1', 'Demo2',
    ]

#where a demo is allowed to leave files: a demo is run to see that Exudyn works, from whatever
#directory the user happens to be in, and it used to create a solution/ directory there - inside
#this repository that is an untracked directory beside the sources (#2620). tmp/ is what this
#repository ignores, and it says what the files are.
demoSolutionDirectory = 'tmp/solution'


def DemoSolutionFile(name):
    """The solution file of a demo, in a directory that is created if it does not exist.

    Args:
        name: the file name, e.g. 'demo1.txt'

    Returns:
        the path the demo writes to
    """
    if not os.path.isdir(demoSolutionDirectory):
        os.makedirs(demoSolutionDirectory)
    return demoSolutionDirectory + '/' + name

def DemoInfo():
    exudyn.Print('\n************************************')
    exudyn.Print('for advanced demos github page:')
    exudyn.Print('https://github.com/jgerstmayr/EXUDYN')
    exudyn.Print('look under python/Examples')
    exudyn.Print('and python/TestModels')
    exudyn.Print('************************************\n')
    

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def Demo1(showAll = True):
    """very simple demo to show that exudyn is correctly installed; does not require graphics; similar to Examples/myFirstExample.py
    """
    if showAll:
        exudyn.Print('start demo1: verify that exudyn is running')
    import exudyn.itemInterface as eii #conversion of data to exudyn dictionaries
    
    SC = exudyn.SystemContainer()         #container of systems
    mbs = SC.AddSystem()               #add a new system to work with
    
    nMP = mbs.AddNode(eii.NodePoint2D(referenceCoordinates=[0,0]))
    mbs.AddObject(eii.ObjectMassPoint2D(mass=10, nodeNumber=nMP ))
    mMP = mbs.AddMarker(eii.MarkerNodePosition(nodeNumber = nMP))
    mbs.AddLoad(eii.Force(markerNumber = mMP, loadVector=[0.001,0,0]))
    
    mbs.Assemble()                     #assemble system and solve
    simulationSettings = exudyn.SimulationSettings()
    simulationSettings.timeIntegration.verboseMode=1 #provide some output
    simulationSettings.solution.file.name = DemoSolutionFile('demo1.txt')

    mbs.SolveDynamic(simulationSettings)
    if showAll:
        exudyn.Print('results can be found in local directory: ' + DemoSolutionFile('demo1.txt'))
    
        DemoInfo()
    
    return [mbs, SC]

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def Demo2(showAll = True):
    """advanced demo, showing that graphics is available; similar to Examples/rigid3Dexample.py
    """
    import exudyn.itemInterface as eii #conversion of data to exudyn dictionaries
    from exudyn.utilities import eulerParameters0
    import exudyn.graphics as graphics
    
    SC = exudyn.SystemContainer()
    mbs = SC.AddSystem()
    
    if showAll:
        exudyn.Print('EXUDYN version='+exudyn.config.Version())
    
    #%%+++++++++++++++++++++++++++++++++++
    #background
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
        #exudyn.Print("Build Object", i)
        ep0 = eulerParameters0 #no rotation
        p0 = [sx+i*2*sx,0.,0] #reference position
    
        nRB = mbs.AddNode(eii.NodeRigidBodyEP(referenceCoordinates=p0+ep0))
        oGraphics = graphics.Brick(size=[1.8*sx, 2*s, 2*s], color= graphics.color.dodgerblue, addEdges=True)
        oGraphicsJoint = graphics.Sphere(point=[-sx,0,cPosZ], radius = 0.6*s, color=graphics.color.darkgrey, 
                                            nTiles=24)
        oRB = mbs.AddObject(eii.ObjectRigidBody(mass=2, 
                                            inertia=[6,1,6,0,0,0], 
                                            nodeNumber=nRB, 
                                            visualization=eii.VObjectRigidBody(graphicsData=[oGraphics, oGraphicsJoint])))
    
        mMassRB = mbs.AddMarker(eii.MarkerBodyMass(bodyNumber = oRB))
        mbs.AddLoad(eii.Gravity(markerNumber = mMassRB, loadVector=[0.,-9.81,0.])) #gravity in negative z-direction
    
        mPos = mbs.AddMarker(eii.MarkerBodyPosition(bodyNumber = oRB, localPosition = [-sx,0.,cPosZ]))
        mbs.AddObject(eii.SphericalJoint(markerNumbers = [mPosLast, mPos]))
        mPosLast = mbs.AddMarker(eii.MarkerBodyPosition(bodyNumber = oRB, localPosition = [sx,0.,cPosZ]))
    
    #%%+++++++++++++++++++++++++++++++++++
    mbs.Assemble()
    # exudyn.Print(mbs)
    
    simulationSettings = exudyn.SimulationSettings() #takes currently set values or default values
    
    fact = 200*(1+99*showAll) #10000
    simulationSettings.timeIntegration.numberOfSteps = 1*fact
    simulationSettings.timeIntegration.endTime = 0.001*fact*0.5*4
    simulationSettings.solution.file.writePeriod = simulationSettings.timeIntegration.endTime/fact*20
    if showAll:
        simulationSettings.solution.file.name = DemoSolutionFile('chain.txt')
    simulationSettings.timeIntegration.verboseMode = int(showAll)
    simulationSettings.linearSolver.solverType = exudyn.LinearSolverType.EigenSparse

    simulationSettings.timeIntegration.newton.useModifiedNewton = True
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 
    
    SC.visualizationSettings.general.renderWindowString = "rigid body chain: press 'V' for drawing settings and 'Q' to stop"
    SC.visualizationSettings.nodes.defaultSize = 0.05
    SC.visualizationSettings.general.graphicsUpdateInterval = 0.02

    SC.visualizationSettings.openGL.multiSampling = 4
    SC.visualizationSettings.openGL.lineWidth = 2
    
    SC.visualizationSettings.openGL.light0.shadow = 0.3
    SC.visualizationSettings.openGL.light0.position = [4,4,10,0]
    
    if showAll:
        SC.renderer.Start()
        SC.renderer.DoIdleTasks()
    
    simulationSettings.timeIntegration.numberOfSteps = 1*fact
    simulationSettings.timeIntegration.endTime = 0.001*fact*0.5*4
    mbs.SolveDynamic(simulationSettings)
    
    if showAll:
        SC.renderer.DoIdleTasks()
        SC.renderer.Stop() #safely close rendering window!

    if showAll:
        input("Press Enter to start SolutionViewer...")
    
        # from exudyn.interactive import SolutionViewer
        mbs.SolutionViewer()

        DemoInfo()

    return [mbs, SC]


#%%++++++++++++++++++++++++
#testing of demos
if __name__ == '__main__':
    
    Demo1()
    input("Press Enter for Demo2 ...")
    
    Demo2()
    