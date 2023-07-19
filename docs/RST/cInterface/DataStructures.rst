

.. _sec-cinterface-datastructures:

***************
Data structures
***************


This section describes a set of special data structures which are used in the Python-C++ interface, 
such as a MatrixContainer for dense/sparse matrices or a list of 3D vectors. 
Note that there are many native data types, such as lists, dicts and numpy arrays (e.g. 3D vectors), 
which are not described here as they are native to Pybind11, but can be passed as arguments when appropriate.

.. _sec-matrixcontainer:


MatrixContainer
===============

The MatrixContainer is a versatile representation for dense and sparse matrices.

.. code-block:: python
   :linenos:
   
   #Create empty MatrixContainer:
   from exudyn import MatrixContainer
   mc = MatrixContainer()
   
   #Create MatrixContainer with dense matrix:
   #matrix can be a list of lists or a numpy array, e.g.:
   matrix = np.eye(6)
   mc = MatrixContainer(matrix)
   
   #Set with dense pyArray (a numpy array): 
   pyArray = np.array(matrix)
   mc.SetWithDenseMatrix(pyArray, useDenseMatrix = True)
   
   #Set empty matrix:
   mc.SetWithDenseMatrix([]], bool useDenseMatrix = True)
   
   #Set with list of lists, stored as sparse matrix:
   mc.SetWithDenseMatrix([[1,2],[3,4]], bool useDenseMatrix = False)
   
   #Set with sparse CSR matrix:
   mc.SetWithSparseMatrixCSR(2,3,[[0,0,13.3],[1,1,4.2],[1,2,42.]], useDenseMatrix=True)
   
   print(mc)
   #gives dense matrix:
   #[[13.3  0.   0. ]
   # [ 0.   4.2 42. ]]

\ The class **MatrixContainer** has the following **functions and structures**:

* | **SetWithDenseMatrix**\ (\ *pyArray*\ , \ *useDenseMatrix*\  = False): 
  | set MatrixContainer with dense numpy array of size (n x m); array (=matrix) contains values and matrix size information; if useDenseMatrix=True, matrix will be stored internally as dense matrix, otherwise it will be converted and stored as sparse matrix (which may speed up computations for larger problems)
* | **SetWithSparseMatrixCSR**\ (\ *numberOfRowsInit*\ , \ *numberOfColumnsInit*\ , \ *pyArrayCSR*\ , \ *useDenseMatrix*\  = True): 
  | set with sparse CSR matrix format: numpy array 'pyArrayCSR' contains sparse triplet (row, col, value) per row; numberOfRows and numberOfColumns given extra; if useDenseMatrix=True, matrix will be converted and stored internally as dense matrix, otherwise it will be stored as sparse matrix
* | **GetPythonObject**\ (): 
  | convert MatrixContainer to numpy array (dense) or dictionary (sparse): containing nr. of rows, nr. of columns, numpy matrix with sparse triplets
* | **Convert2DenseMatrix**\ (): 
  | convert MatrixContainer to dense numpy array (SLOW and may fail for too large sparse matrices)
* | **UseDenseMatrix**\ (): 
  | returns True if dense matrix is used, otherwise False
* | **\_\_repr\_\_()**\ : 
  | return the string representation of the MatrixContainer




Vector3DList
============

The Vector3DList is used to represent lists of 3D vectors. This is used to transfer such lists from Python to C++. 

Usage: 
+  Create empty \ ``Vector3DList``\  with \ ``x = Vector3DList()``\  
+  Create \ ``Vector3DList``\  with list of numpy arrays:
\ ``x = Vector3DList([ numpy.array([1.,2.,3.]), numpy.array([4.,5.,6.]) ])``\ 
+  Create \ ``Vector3DList``\  with list of lists \ ``x = Vector3DList([[1.,2.,3.], [4.,5.,6.]])``\ 
+  Append item: \ ``x.Append([0.,2.,4.])``\ 
+  Convert into list of numpy arrays: \ ``x.GetPythonObject()``\ 



\ The class **Vector3DList** has the following **functions and structures**:

* | **Append**\ (\ *pyArray*\ ): 
  | add single array or list to Vector3DList; array or list must have appropriate dimension!
* | **GetPythonObject**\ (): 
  | convert Vector3DList into (copied) list of numpy arrays
* | **len(data)**\ : 
  | return length of the Vector3DList, using len(data) where data is the Vector3DList
* | **data[index]= ...**\ : 
  | set list item 'index' with data, write: data[index] = ...
* | **... = data[index]**\ : 
  | get copy of list item with 'index' as vector
* | **\_\_copy\_\_**\ (): 
  | copy method to be used for copy.copy(...); in fact does already deep copy
* | **\_\_deepcopy\_\_**\ (): 
  | deepcopy method to be used for copy.copy(...)
* | **\_\_repr\_\_()**\ : 
  | return the string representation of the Vector3DList data, e.g.: print(data)




Vector2DList
============

The Vector2DList is used to represent lists of 2D vectors. This is used to transfer such lists from Python to C++. 

Usage: 
+  Create empty \ ``Vector2DList``\  with \ ``x = Vector2DList()``\  
+  Create \ ``Vector2DList``\  with list of numpy arrays:
\ ``x = Vector2DList([ numpy.array([1.,2.]), numpy.array([4.,5.]) ])``\ 
+  Create \ ``Vector2DList``\  with list of lists \ ``x = Vector2DList([[1.,2.], [4.,5.]])``\ 
+  Append item: \ ``x.Append([0.,2.])``\ 
+  Convert into list of numpy arrays: \ ``x.GetPythonObject()``\ 
+  similar to Vector3DList !



\ The class **Vector2DList** has the following **functions and structures**:

* | **Append**\ (\ *pyArray*\ ): 
  | add single array or list to Vector2DList; array or list must have appropriate dimension!
* | **GetPythonObject**\ (): 
  | convert Vector2DList into (copied) list of numpy arrays
* | **len(data)**\ : 
  | return length of the Vector2DList, using len(data) where data is the Vector2DList
* | **data[index]= ...**\ : 
  | set list item 'index' with data, write: data[index] = ...
* | **... = data[index]**\ : 
  | get copy of list item with 'index' as vector
* | **\_\_copy\_\_**\ (): 
  | copy method to be used for copy.copy(...); in fact does already deep copy
* | **\_\_deepcopy\_\_**\ (): 
  | deepcopy method to be used for copy.copy(...)
* | **\_\_repr\_\_()**\ : 
  | return the string representation of the Vector2DList data, e.g.: print(data)




Vector6DList
============

The Vector6DList is used to represent lists of 6D vectors. This is used to transfer such lists from Python to C++. 

Usage: 
+  Create empty \ ``Vector6DList``\  with \ ``x = Vector6DList()``\  
+  Convert into list of numpy arrays: \ ``x.GetPythonObject()``\ 
+  similar to Vector3DList !



\ The class **Vector6DList** has the following **functions and structures**:

* | **Append**\ (\ *pyArray*\ ): 
  | add single array or list to Vector6DList; array or list must have appropriate dimension!
* | **GetPythonObject**\ (): 
  | convert Vector6DList into (copied) list of numpy arrays
* | **len(data)**\ : 
  | return length of the Vector6DList, using len(data) where data is the Vector6DList
* | **data[index]= ...**\ : 
  | set list item 'index' with data, write: data[index] = ...
* | **... = data[index]**\ : 
  | get copy of list item with 'index' as vector
* | **\_\_copy\_\_**\ (): 
  | copy method to be used for copy.copy(...); in fact does already deep copy
* | **\_\_deepcopy\_\_**\ (): 
  | deepcopy method to be used for copy.copy(...)
* | **\_\_repr\_\_()**\ : 
  | return the string representation of the Vector6DList data, e.g.: print(data)




Matrix3DList
============

The Matrix3DList is used to represent lists of 3D Matrices. . This is used to transfer such lists from Python to C++. 

Usage: 
+  Create empty \ ``Matrix3DList``\  with \ ``x = Matrix3DList()``\  
+  Create \ ``Matrix3DList``\  with list of numpy arrays:
\ ``x = Matrix3DList([ numpy.eye(3), numpy.array([[1.,2.,3.],[4.,5.,6.],[7.,8.,9.]]) ])``\ 
+  Append item: \ ``x.Append(numpy.eye(3))``\ 
+  Convert into list of numpy arrays: \ ``x.GetPythonObject()``\ 
+  similar to Vector3DList !



\ The class **Matrix3DList** has the following **functions and structures**:

* | **Append**\ (\ *pyArray*\ ): 
  | add single 3D array or list of lists to Matrix3DList; array or lists must have appropriate dimension!
* | **GetPythonObject**\ (): 
  | convert Matrix3DList into (copied) list of 3x3 numpy arrays
* | **len(data)**\ : 
  | return length of the Matrix3DList, using len(data) where data is the Matrix3DList
* | **data[index]= ...**\ : 
  | set list item 'index' with matrix, write: data[index] = ...
* | **... = data[index]**\ : 
  | get copy of list item with 'index' as matrix
* | **\_\_repr\_\_()**\ : 
  | return the string representation of the Matrix3DList data, e.g.: print(data)




Matrix6DList
============

The Matrix6DList is used to represent lists of 6D Matrices. . This is used to transfer such lists from Python to C++. 

Usage: 
+  Create empty \ ``Matrix6DList``\  with \ ``x = Matrix6DList()``\  
+  Create \ ``Matrix6DList``\  with list of numpy arrays:
\ ``x = Matrix6DList([ numpy.eye(6), 2*numpy.eye(6) ])``\ 
+  Append item: \ ``x.Append(numpy.eye(6))``\ 
+  Convert into list of numpy arrays: \ ``x.GetPythonObject()``\ 
+  similar to Matrix3DList !



\ The class **Matrix6DList** has the following **functions and structures**:

* | **Append**\ (\ *pyArray*\ ): 
  | add single 6D array or list of lists to Matrix6DList; array or lists must have appropriate dimension!
* | **GetPythonObject**\ (): 
  | convert Matrix6DList into (copied) list of 6x6 numpy arrays
* | **len(data)**\ : 
  | return length of the Matrix6DList, using len(data) where data is the Matrix6DList
* | **data[index]= ...**\ : 
  | set list item 'index' with matrix, write: data[index] = ...
* | **... = data[index]**\ : 
  | get copy of list item with 'index' as matrix
* | **\_\_repr\_\_()**\ : 
  | return the string representation of the Matrix6DList data, e.g.: print(data)



