

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

The MatrixContainer is a versatile representation for dense and sparse matrices. NOTE: if the MatrixContainer is constructed from a numpy array or a list of lists, both representing a dense matrix, it will go into dense mode; if it is initialized with a scipy sparse csr matrix, it will go into sparse mode. Examples:

.. code-block:: python
   :linenos:
   
   #Create empty MatrixContainer:
   from scipy.sparse import csr_matrix
   from exudyn import MatrixContainer
   mc = MatrixContainer() #empty matrix, dense mode
   
   #Create MatrixContainer with dense matrix:
   #container can be initialized with a dense matrix, using list of lists or a numpy array, e.g.:
   matrix = np.eye(3)
   #stores matrices internally in dense mode:
   mcDense1 = MatrixContainer(matrix)
   mcDense2 = MatrixContainer([[1,2],[3,4]])
   
   #container can be initialized with a scipy csr sparse matrix, then being stored as sparse matrix
   mcSparse = MatrixContainer(csr_matrix(matrix))
   
   #Set with dense pyArray (a numpy array): 
   pyArray = np.array(matrix)
   mc.SetWithDenseMatrix(pyArray, useDenseMatrix = True)
   
   #Set empty matrix:
   mc.SetWithDenseMatrix([[]], useDenseMatrix = True)
   
   #Set with list of lists, stored as sparse matrix:
   mc.SetWithDenseMatrix([[1,2],[3,4]], useDenseMatrix = False)
   
   #Set with sparse triplets (list of lists or numpy array):
   mc.SetWithSparseMatrix([[0,0,13.3],[1,1,4.2],[1,2,42.]], 
                          numberOfRows=2, numberOfColumns=3, 
                          useDenseMatrix=True)
   
   print(mc)
   #gives dense matrix:
   #[[13.3  0.   0. ]
   # [ 0.   4.2 42. ]]
   
   #Set with scipy matrix:
   #WARNING: only use csr_matrix
   #         csc_matrix would basically run, but gives the transposed!!!
   spmat = csr_matrix(matrix) 
   mc.SetWithSparseMatrix(spmat) #takes rows and column format automatically
   
   #initialize and add triplets later on
   mc.Initialize(3,3,useDenseMatrix=False)
   mc.AddSparseMatrix(spmat, factor=1)
   #can also add smaller matrix
   mc.AddSparseMatrix(csr_matrix(np.eye(2)), factor=0.5)
   print('mc8=',mc)
   

\ The class **MatrixContainer** has the following **functions and structures**:

* | **Initialize**\ (\ *numberOfRows*\ , \ *numberOfColumns*\ , \ *useDenseMatrix*\  = True): 
  | initialize MatrixContainer with number of rows and columns and set dense/sparse mode
* | **SetWithDenseMatrix**\ (\ *pyArray*\ , \ *useDenseMatrix*\  = False, \ *factor*\  = 1.): 
  | set MatrixContainer with dense numpy array of size (n x m); array (=matrix) contains values and matrix size information; if useDenseMatrix=True, matrix will be stored internally as dense matrix, otherwise it will be converted and stored as sparse matrix (which may speed up computations for larger problems); pyArray is multiplied with given factor
* | **SetWithSparseMatrix**\ (\ *sparseMatrix*\ , \ *numberOfRows*\  = invalid (-1), \ *numberOfColumns*\  = invalid (-1), \ *useDenseMatrix*\  = False, \ *factor*\  = 1.): 
  | set with scipy sparse csr_matrix (NOT: csc_matrix!) or with internal sparse triplet format (denoted as CSR): 'sparseMatrix' either contains a scipy matrix create with csr_matrix or a list of lists of sparse triplets (row, col, value) or the list of lists converted into numpy array; numberOfRowsInit and numberOfColumnsInit denote the size of the matrices, which are ignored in case of a scipy sparse matrix; if useDenseMatrix=True, matrix will be converted and stored internally as dense matrix, otherwise it will be stored as sparse matrix triplets; the values of sparseMatrix are multiplied with the given factor before storing
* | **AddSparseMatrix**\ (\ *sparseMatrix*\ , \ *factor*\  = 1.): 
  | add scipy sparse csr_matrix with factor to already initilized MatrixContainer; sparseMatrix must contain according scipy csr format, otherwise the behavior is undefined! This function allows to efficiently add submatrices to the MatrixContainer
* | **GetPythonObject**\ (): 
  | convert MatrixContainer to numpy array (dense) or dictionary (sparse): containing nr. of rows, nr. of columns, numpy matrix with sparse triplets
* | **Convert2DenseMatrix**\ (): 
  | convert MatrixContainer to dense numpy array (SLOW and may fail for too large sparse matrices)
* | **UseDenseMatrix**\ (): 
  | returns True if dense matrix is used, otherwise False
* | **SetAllZero**\ (): 
  | Set all values to zero; dense mode: set all matrix entries to zero (slow); sparse mode: set number of triplets to zero (fast)
* | **SetWithSparseMatrixCSR**\ (\ *numberOfRowsInit*\ , \ *numberOfColumnsInit*\ , \ *pyArrayCSR*\ , \ *useDenseMatrix*\  = False, \ *factor*\  = 1.): 
  | DEPRECATED: set with sparse CSR matrix format: numpy array 'pyArrayCSR' contains sparse triplet (row, col, value) per row; numberOfRows and numberOfColumns given extra; if useDenseMatrix=True, matrix will be converted and stored internally as dense matrix, otherwise it will be stored as sparse matrix; the values of pyArrayCSR are multiplied by the given factor
* | **\_\_repr\_\_()**\ : 
  | return the string representation of the MatrixContainer




.. _sec-graphicsmateriallist:


GraphicsMaterialList
====================

The GraphicsMaterialList contains the list of materials (material properties) for visualization; currently, only the raytracer uses materials. Materials can be accessed via the variable materials in renderer of SystemContainer.

.. code-block:: python
   :linenos:
   
   #access material 0:
   mat0 = SC.renderer.materials[0]
   #convert into dictionary for easier processing:
   matDict = mat0.GetDictionary()
   matDict['alpha'] = 0.5
   #change material (e.g. using a data base):
   mat0.SetDictionary(matDict)
   #or directly update material
   mat0.name = 'new name'
   mat0.emission = [0.8,0.6,0.]
   
   #update material in renderer:
   SC.renderer.materials.Set(0,mat0)
   #update material directly with dictionary:
   SC.renderer.materials.Set(0,matDict)
   
   #create new material:
   mat10 = SC.renderer.materials.New()
   mat10.reflectivity = 0.8
   SC.renderer.materials.Append(mat10) #returns index of mat10
   
   #10 default graphics materials in Exudyn
   #listed here with default color and some properties:
   #note the increased computational costs for reflection & transparency
   #the material names are as follows:
   SC.renderer.materials[0].name == "default"    #steel blue
   SC.renderer.materials[1].name == "matt"       #green
   SC.renderer.materials[2].name == "steel"      #grey (reflection)
   SC.renderer.materials[3].name == "plastic"    #red (reflection)
   SC.renderer.materials[4].name == "chrome"     #light grey (reflection)
   SC.renderer.materials[5].name == "shiny"      #orange (reflection)
   SC.renderer.materials[6].name == "transparent"#(transparency,slight refraction)
   SC.renderer.materials[7].name == "glass"      #light grey (reflection,transparency,refraction)
   SC.renderer.materials[8].name == "mirror"     #light grey (reflection)
   SC.renderer.materials[9].name == "emission"   #light yellow
   

\ The class **GraphicsMaterialList** has the following **functions and structures**:

* | **Reset**\ (): 
  | reset materials to 10 default materials
* | **Append**\ (\ *material*\ ): 
  | add single material as dict or VSettingsMaterial to list; returns index of newly added material
* | **New**\ (): 
  | Get new default material, which can be modified or appended to materials list
* | **Set**\ (\ *indexOrName*\ , \ *material*\ ): 
  | set material with index 'materialIndex' as dict or VSettingsMaterial
* | **Get**\ (\ *indexOrName*\ ): 
  | get material with index 'materialIndex' as VSettingsMaterial
* | **GetDict**\ (\ *indexOrName*\ ): 
  | get material with index 'materialIndex' as dict
* | **len(data)**\ : 
  | return length of the Vector3DList, using len(data) where data is the Vector3DList
* | **... = data[index]**\ : 
  | get reference access of material with 'index' as VSettingsMaterial
* | **\_\_repr\_\_()**\ : 
  | return the string representation of the GraphicsMaterialList




Vector3DList
============

The Vector3DList is used to represent lists of 3D vectors. This is used to transfer such lists from Python to C++. 

Usage:

+  Create empty \ ``Vector3DList``\  with \ ``x = Vector3DList()``\  
+  Create \ ``Vector3DList``\  with list of numpy arrays:\ ``x = Vector3DList([ numpy.array([1.,2.,3.]), numpy.array([4.,5.,6.]) ])``\ 
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



