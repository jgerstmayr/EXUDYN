
.. _examples-testhdf5loadsave:

*******************
testHDF5loadSave.py
*******************

You can view and download this file on Github: `testHDF5loadSave.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/testHDF5loadSave.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  A test for HDF5 load and save functionality
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-06-08
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu               #EXUDYN package including C++ core part
   from exudyn.utilities import * 
   import exudyn.graphics as graphics
   from exudyn.advancedUtilities import * 
   
   #Recursively compare two dictionaries for equality.
   def CompareDicts(dict1, dict2):
       if dict1.keys() != dict2.keys():
           return False
       for key in dict1:
           val1, val2 = dict1[key], dict2[key]
           if isinstance(val1, dict) and isinstance(val2, dict):
               if not CompareDicts(val1, val2):
                   return False
           elif isinstance(val1, list) and isinstance(val2, list):
               if len(val1) != len(val2):
                   return False
               for item1, item2 in zip(val1, val2):
                   if isinstance(item1, dict) and isinstance(item2, dict):
                       if not CompareDicts(item1, item2):
                           return False
                   elif isinstance(item1, np.ndarray) and isinstance(item2, np.ndarray):
                       if not np.array_equal(item1, item2):
                           return False
                   elif item1 != item2:
                       return False
           elif isinstance(val1, np.ndarray) and isinstance(val2, np.ndarray):
               if not np.array_equal(val1, val2):
                   return False
           elif val1 != val2:
               return False
       return True
   
   gChecker=graphics.CheckerBoard(nTiles=1)
   gSphere=graphics.Sphere(nTiles=3)
   
   test = {
       'a': [np.array([1.0, 2.0, 3.0]), [3, 4, 5]],
       'b': {'x': 10, 'y': 20.5, 'z': [np.array([1, 2]), 'hello']},
       'c': [True, False, None],
       'd': 'This is a test string',
       'e': np.array([[1, 2], [3, 4]]),
       'f': [np.array([5.5, 6.5]), {'nested': 'value', 'list': [1,2,np.array([1,2]),{'g':[1.,2.,'message']}]}],
       'h': [gChecker,gSphere],
   }
   
   fileName = 'testData/test.hdf5'
   SaveDictToHDF5(fileName, test)
   test2 = LoadDictFromHDF5(fileName)
   
   if CompareDicts(test, test2):
       exu.Print('TestHDF5loadSave: test successful!')
   else:
       #this is used in Examples and will just show that example failed
       raise ValueError('***********\nTestHDF5loadSave: failed!\n***********\n')
   
   
   


