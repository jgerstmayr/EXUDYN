
.. _testmodels-matrixcontainertest:

**********************
matrixContainerTest.py
**********************

You can view and download this file on Github: `matrixContainerTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/matrixContainerTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Simple test for MatrixContainer, similar as Examples given in Docu
   #
   # Author:   Johannes Gerstmayr
   # Date:     2024-10-09
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
   
   import numpy as np
   from scipy.sparse import csr_matrix
   from exudyn import MatrixContainer
   
   u = 0 #result
   mc = MatrixContainer()
   
   #Create MatrixContainer with dense matrix:
   #matrix can be initialized with a dense matrix, using a numpy array or list of lists , e.g.:
   matrix = np.eye(3)
   mcDense1 = MatrixContainer(matrix)
   u+= mcDense1.Convert2DenseMatrix()[0,1]
   mcDense2 = MatrixContainer([[1,2],[3,4]])
   
   u+= mcDense1.Convert2DenseMatrix()[1,0]
   mcSparse = MatrixContainer(csr_matrix(np.array([[1,5],[0,7]])))
   u+= mcSparse.Convert2DenseMatrix()[0,1]
   
   #Set with dense pyArray (a numpy array): 
   pyArray = np.array(matrix)
   mc.SetWithDenseMatrix(pyArray, useDenseMatrix = True)
   u+= mc.Convert2DenseMatrix()[1,0]
   
   #Set empty matrix:
   mc.SetWithDenseMatrix([[]], useDenseMatrix = True)
   
   #Set with list of lists, stored as sparse matrix:
   mc.SetWithDenseMatrix([[1,2],[3,4]], useDenseMatrix = False)
   u+= mc.Convert2DenseMatrix()[1,0]
   
   #Set with sparse triplets (list of lists or numpy array):
   mc.SetWithSparseMatrix([[0,0,13.3],[1,1,4.2],[1,2,42.]], 
                          numberOfRows=2, numberOfColumns=3, 
                          useDenseMatrix=True)
   u+= mc.Convert2DenseMatrix()[1,2]
   
   exu.Print(mc)
   #gives dense matrix:
   #[[13.3  0.   0. ]
   # [ 0.   4.2 42. ]]
   
   #Set with scipy matrix:
   #WARNING: only use csr_matrix
   #         csc_matrix would basically run, but gives the transposed!!!
   spmat = csr_matrix(matrix) 
   mc.SetWithSparseMatrix(spmat) #takes rows and column format automatically
   u+= mc.Convert2DenseMatrix()[1,1]
   
   #initialize and add triplets later on
   mc.Initialize(3,3,useDenseMatrix=False)
   mc.AddSparseMatrix(spmat, factor=1)
   #can also add smaller matrix
   mc.AddSparseMatrix(csr_matrix(np.eye(2)), factor=0.5)
   exu.Print('mc=',mc.Convert2DenseMatrix())
   u+= mc.Convert2DenseMatrix()[0,0]
   
   #set matrix with smaller sparse matrix than the final dimensions
   mc.SetWithSparseMatrix(csr_matrix(np.array([[1,2],[3,4]])), numberOfRows=4, numberOfColumns=2)
   exu.Print('mc=',mc.Convert2DenseMatrix())
   u+= mc.Convert2DenseMatrix()[1,1]
   
   
   result = u
   exu.Print('solution of matrixContainerTest=',result)
   
   exudynTestGlobals.testError = (result - (56.5))
   exudynTestGlobals.testResult = result


