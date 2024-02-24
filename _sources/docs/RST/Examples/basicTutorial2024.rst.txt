
.. _examples-basictutorial2024:

********************
basicTutorial2024.py
********************

You can view and download this file on Github: `basicTutorial2024.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/basicTutorial2024.py>`_

.. code-block:: python
   :linenos:

   #++++++++++++++++++++++++++++++++
   #author: Johannes Gerstmayr
   #data:   2024-02-15
   #++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import *
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #++++++++++++++++++++++++++++++++++
   
   oGround = mbs.CreateGround() #[0,0,0]
   
   oMass = mbs.CreateMassPoint(name='HeavyMass',
                               referencePosition=[2,0,0],
                               physicsMass=12,
                               gravity=[0,-9.81,0],
                               drawSize=0.2,
                               color=color4red)
   
   oSD = mbs.CreateSpringDamper(bodyList=[oGround, oMass],
                                stiffness=500,
                                damping=10,
                                drawSize=0.1)
   
   mbs.Assemble()
   
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   SC.visualizationSettings.general.drawWorldBasis = True
   
   tEnd = 10
   stepSize = 0.02 #smaller gives more accurate results
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   
   mbs.SolveDynamic(simulationSettings=simulationSettings)
   
   mbs.SolutionViewer()
   


