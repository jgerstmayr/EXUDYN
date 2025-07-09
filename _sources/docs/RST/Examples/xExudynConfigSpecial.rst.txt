
.. _examples-xexudynconfigspecial:

***********************
xExudynConfigSpecial.py
***********************

You can view and download this file on Github: `xExudynConfigSpecial.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/xExudynConfigSpecial.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  A test for exudyn variables and functions, config, special, ...
   #           Mainly to check if something crashes 
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-05-22
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu               #EXUDYN package including C++ core part
   from exudyn.utilities import * 
   import time
   
   exu.Help()
   
   #++++++++++++++++++++++++++++++++++++++++
   SC = exu.SystemContainer()         #container of systems
   mbs = SC.AddSystem()               #add a new system to work with
   
   nMP = mbs.AddNode(NodePoint2D(referenceCoordinates=[0,0]))
   mbs.AddObject(ObjectMassPoint2D(physicsMass=10, nodeNumber=nMP ))
   mMP = mbs.AddMarker(MarkerNodePosition(nodeNumber = nMP))
   mbs.AddLoad(Force(markerNumber = mMP, loadVector=[0.001,0,0]))
   
   mbs.Assemble()                     #assemble system and solve
   
   #++++++++++++++++++++++++++++++++++++++++
   #test some special exudyn member variables and functions:
   exu.Print('EXUDYN version='+exu.config.Version())
   exu.Print('EXUDYN version with details='+exu.config.Version(addDetails=True))
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #store old settings
   outputPrecision = exu.config.outputPrecision
   suppressWarnings = exu.config.suppressWarnings
   timeout = exu.special.solver.timeout
   throwErrorWithCtrlC = exu.special.solver.throwErrorWithCtrlC
   eigenFullPivotLUsolverDebugLevel = exu.experimental.eigenFullPivotLUsolverDebugLevel
   markerSuperElementRigidTexpSO3 = exu.experimental.markerSuperElementRigidTexpSO3
   
   printToConsole = exu.config.printToConsole 
   printFileName = exu.config.printFileName
   printToFile = exu.config.printToFile
   printToFileAppend = exu.config.printToFileAppend
   printFlushAlways = exu.config.printFlushAlways
   printDelayMilliSeconds = exu.config.printDelayMilliSeconds
   linalgOutputFormatPython = exu.config.linalgOutputFormatPython
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   exu.config.outputPrecision = 4
   exu.config.suppressWarnings = True
   exu.config.printFlushAlways = True
   exu.config.printDelayMilliSeconds = 10
   
   exu.special.solver.timeout=2
   exu.special.solver.throwErrorWithCtrlC=0
   exu.special.exceptions.dictionaryVersionMismatch=False
   exu.special.exceptions.dictionaryNonCopyable=False
   exu.experimental.eigenFullPivotLUsolverDebugLevel=True
   exu.experimental.markerSuperElementRigidTexpSO3=False
   
   exu.Print(exu.config)
   exu.Print(exu.special)
   exu.Print(exu.experimental)
   exu.Print(exu.special.exceptions)
   exu.special.InfoStat()
   
   
   #during test, we have to store current behavior
   
   exu.SetWriteToFile('solution/test.txt')
   exu.config.printToConsole = True
   exu.Print('\n*****\nThis should be visible in testExamples: test:',42,{'a':'b'},end="\n*****\n", sep=" - ", flush=True)
   exu.config.printToConsole = printToConsole
   #exu.SetWriteToFile('',False)
   
   for i in range(10):
       exu.Print('\rtest flush=True',42+i,end=" *", sep=":", flush=True)
       time.sleep(0.01)
   exu.Print('\n')
   for i in range(10):
       exu.Print('\rtest flush=False:',i,end=" *", flush=False)
       time.sleep(0.01)
   exu.Print('\n')
   exu.Print('\rtest flush=False with end="\\n":',i,sep='',end="\n", flush=False)
   
   exu.Print('\nend')
   
   
   
   #++++++++++++++++++++++++++++++++++++++++
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.timeIntegration.verboseMode=3 #provide detailed output
   simulationSettings.timeIntegration.endTime=0.001
   simulationSettings.timeIntegration.numberOfSteps=1
   mbs.SolveDynamic(simulationSettings)
   exu.config.linalgOutputFormatPython = False
   mbs.SolveDynamic(simulationSettings) #now writing matlab format
   
   #run demo
   exu.demos.Demo1()
   exu.Print(exu.graphics.color.red)
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #store old settings
   exu.config.outputPrecision = outputPrecision
   exu.config.suppressWarnings = suppressWarnings
   exu.special.solver.timeout = timeout
   exu.special.solver.throwErrorWithCtrlC = throwErrorWithCtrlC
   exu.experimental.eigenFullPivotLUsolverDebugLevel = eigenFullPivotLUsolverDebugLevel
   exu.experimental.markerSuperElementRigidTexpSO3 = markerSuperElementRigidTexpSO3
   exu.SetWriteToFile(printFileName, flagWriteToFile=printToFile, flagAppend=True)
   exu.config.printFlushAlways = printFlushAlways
   exu.config.printDelayMilliSeconds = printDelayMilliSeconds
   exu.config.linalgOutputFormatPython = linalgOutputFormatPython
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   exu.Print('*** Within runExampleTests.py: This text should only be visible in file, not in console ***')


