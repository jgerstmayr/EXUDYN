

.. _sec-item-objectgenericode1:

ObjectGenericODE1
=================

A system of \ :math:`n`\  ODE1, having a system matrix, a rhs vector, but mostly it will use a user function to describe special ODE1 systems. It is based on NodeGenericODE1 nodes. NOTE that all matrices, vectors, etc. must have the same dimensions \ :math:`n`\  or \ :math:`(n \times n)`\ , or they must be empty \ :math:`(0 \times 0)`\ , using [] in Python.
 



The item \ **ObjectGenericODE1**\  with type = 'GenericODE1' has the following parameters:

 

* | **name** [type = String, default = '']:
  | objects's unique name
* | **nodeNumbers** [type = ArrayNodeIndex, default = []]:
  | node numbers which provide the coordinates for the object (consecutively as provided in this list)
* | **systemMatrix** [type = NumpyMatrix, default = Matrix[]]:
  | system matrix (state space matrix) of first order ODE
* | **rhsVector** [type = NumpyVector, default = []]:
  | a constant rhs vector (e.g., for constant input)
* | **rhsUserFunction** [type = PyFunctionVectorMbsScalarIndexVector, default =  0]:
  | A Python user function which computes the right-hand-side (rhs) of the first order ODE; see description below
* | **coordinateIndexPerNode** [type = ArrayIndex, default = []]:
  | this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed
* | **tempCoordinates** [type = NumpyVector, default = []]:
  | temporary vector containing coordinates
* | **tempCoordinates_t** [type = NumpyVector, default = []]:
  | temporary vector containing velocity coordinates



The item VObjectGenericODE1 has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


