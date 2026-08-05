
.. _testmodels-allexudynmodulestest:

***********************
allExudynModulesTest.py
***********************

You can view and download this file on Github: `allExudynModulesTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/allExudynModulesTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for import of all exudyn modules, like utilities, etc.; 
   #           done to ensure that they do not contain any major error due to docstring conversion
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-06-14
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   
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
   
   from pathlib import Path
   import importlib.util
   import sys
   import os
   
   # Set the directory path ('.' for current directory)
   exudynPath = os.path.dirname(exu.__file__)
   directory = Path(exudynPath)
   
   # Get all files ending in .py recursively
   allModules = list(directory.rglob('*.py')) #list of file objects
   
   testSolution = 1
   
   exu.Print('first module location=',str(allModules[0]))
   
   excludeModules = ['__init__.py', #fail
                     'resultsMonitor.py', #can only be called from command line with args
                     'rosInterface.py',   #rospy usually missing
                     #'artificialIntelligence.py'
                     ]
   
   try:
       import stable_baselines3
       #ok
   except:
       exu.Print('stable-baselines3 not available: skip artificialIntelligence.py')
       excludeModules.append('artificialIntelligence.py')
   
   #test all modules
   for moduleCheck in allModules: #moduleCheck converts to string as path+fileName
       if moduleCheck.name in excludeModules:
           continue
       exu.Print('check module:',moduleCheck.name)
       try:
           spec = importlib.util.spec_from_file_location(moduleCheck.stem, moduleCheck)
   
           testModule = importlib.util.module_from_spec(spec)
           
           sys.modules[moduleCheck.stem] = testModule #moduleCheck.name is the filename with ending, moduleCheck.stem is without ending
           
           spec.loader.exec_module(testModule) #now execute module
       
       except FileNotFoundError:
           testSolution = 0
           exu.Print(f"Error: The module {moduleCheck} does not exist.")
       except SyntaxError as e:
           testSolution = 0
           exu.Print(f"Error: module {moduleCheck} has invalid Python syntax: {e}")
       except Exception as e:
           testSolution = 0
           exu.Print(f"Error: Failed to execute {moduleCheck}: {e}")
   
   if testSolution==1:
       exu.Print('\n*** all imports successful! ***\n')
   #also add test for demos
   try:
       exu.Print('test demo1')
       exu.demos.Demo1()
       exu.Print('test demo2')
       exu.demos.Demo2(showAll=False)
   except Exception as e:
       exu.Print(f"Error: Demos failed: {e}")
       
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   exu.Print('\nsolution of allExudynModulesTest (should be 1)=',testSolution) 
   exudynTestGlobals.testResult = testSolution
   


