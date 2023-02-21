

.. _sec-item-objectgenericode1:

ObjectGenericODE1
=================

A system of \ :math:`n`\  ODE1, having a system matrix, a rhs vector, but mostly it will use a user function to describe special ODE1 systems. It is based on NodeGenericODE1 nodes. NOTE that all matrices, vectors, etc. must have the same dimensions \ :math:`n`\  or \ :math:`(n \times n)`\ , or they must be empty \ :math:`(0 \times 0)`\ , using [] in Python.

\ **Additional information for ObjectGenericODE1**\ :

* | The Object has the following types = \ ``MultiNoded``\ 
* | Requested node type: read detailed information of item


The item \ **ObjectGenericODE1**\  with type = 'GenericODE1' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **nodeNumbers** [\ :math:`\mathbf{n}_n = [n_0,\,\ldots,\,n_n]\tp`\ , type = ArrayNodeIndex, default = []]:
  | node numbers which provide the coordinates for the object (consecutively as provided in this list)
* | **systemMatrix** [\ :math:`{\mathbf{A}} \in \Rcal^{n \times n}`\ , type = NumpyMatrix, default = Matrix[]]:
  | system matrix (state space matrix) of first order ODE
* | **rhsVector** [\ :math:`{\mathbf{f}} \in \Rcal^{n}`\ , type = NumpyVector, default = []]:
  | a constant rhs vector (e.g., for constant input)
* | **rhsUserFunction** [\ :math:`{\mathbf{f}}_{user} \in \Rcal^{n}`\ , type = PyFunctionVectorMbsScalarIndexVector, default =  0]:
  | A Python user function which computes the right-hand-side (rhs) of the first order ODE; see description below
* | **coordinateIndexPerNode** [type = ArrayIndex, default = []]:
  | this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed
* | **tempCoordinates** [\ :math:`{\mathbf{c}}_{temp} \in \Rcal^{n}`\ , type = NumpyVector, default = []]:
  | temporary vector containing coordinates
* | **tempCoordinates_t** [\ :math:`\dot {\mathbf{c}}_{temp} \in \Rcal^{n}`\ , type = NumpyVector, default = []]:
  | temporary vector containing velocity coordinates
* | **visualization** [type = VObjectGenericODE1]:
  | parameters for visualization of item



The item VObjectGenericODE1 has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Coordinates``\ : 
  | all ODE1 coordinates
* | ``Coordinates\_t``\ : 
  | all ODE1 velocity coordinates




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


