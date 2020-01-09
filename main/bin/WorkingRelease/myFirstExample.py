
from itemInterface import *     #conversion of data to exudyn dictionaries
import exudyn as exu           #C++ EXUDYN library
SC = exu.SystemContainer()       #container of systems
mbs = SC.AddSystem()            #add a new system to work with

nMP = mbs.AddNode(NodePoint2D(referenceCoordinates=[0,0]))
mbs.AddObject(ObjectMassPoint2D(physicsMass=10, nodeNumber=nMP ))
mMP = mbs.AddMarker(MarkerNodePosition(nodeNumber = nMP))
mbs.AddLoad(Force(markerNumber = mMP, loadVector=[0.001,0,0]))

mbs.Assemble()                  #assemble system and solve
simulationSettings = exu.SimulationSettings()
SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)

