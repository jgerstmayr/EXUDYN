

.. _sec-item-objectgenericode2:

ObjectGenericODE2
=================

A system of \ :math:`n`\  second order ordinary differential equations (ODE2), having a mass matrix, damping/gyroscopic matrix, stiffness matrix and generalized forces. It can combine generic nodes, or node points. User functions can be used to compute mass matrix and generalized forces depending on given coordinates. NOTE that all matrices, vectors, etc. must have the same dimensions \ :math:`n`\  or \ :math:`(n \times n)`\ , or they must be empty \ :math:`(0 \times 0)`\ , except for the mass matrix which always needs to have dimensions \ :math:`(n \times n)`\ .
 



The item \ **ObjectGenericODE2**\  with type = 'GenericODE2' has the following parameters:

 

* | **name** [type = String, default = '']:
  | objects's unique name
* | **nodeNumbers** [type = ArrayNodeIndex, default = []]:
  | node numbers which provide the coordinates for the object (consecutively as provided in this list)
* | **massMatrix** [type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | mass matrix of object as MatrixContainer (or numpy array / list of lists)
* | **stiffnessMatrix** [type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | stiffness matrix of object as MatrixContainer (or numpy array / list of lists); NOTE that (dense/sparse triplets) format must agree with dampingMatrix and jacobianUserFunction
* | **dampingMatrix** [type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | damping matrix of object as MatrixContainer (or numpy array / list of lists); NOTE that (dense/sparse triplets) format must agree with stiffnessMatrix and jacobianUserFunction
* | **forceVector** [type = NumpyVector, default = []]:
  | generalized force vector added to RHS
* | **forceUserFunction** [type = PyFunctionVectorMbsScalarIndex2Vector, default =  0]:
  | A Python user function which computes the generalized user force vector for the ODE2 equations; see description below
* | **massMatrixUserFunction** [type = PyFunctionMatrixContainerMbsScalarIndex2Vector, default =  0]:
  | A Python user function which computes the mass matrix instead of the constant mass matrix given in \ :math:`{\mathbf{M}}`\ ; return numpy array or MatrixContainer; see description below
* | **jacobianUserFunction** [type = PyFunctionMatrixContainerMbsScalarIndex2Vector2Scalar, default =  0]:
  | A Python user function which computes the jacobian, i.e., the derivative of the left-hand-side object equation w.r.t.\ the coordinates (times \ :math:`f_ODE2`\ ) and w.r.t.\ the velocities (times \ :math:`f_ODE2_t`\ ). Terms on the RHS must be subtracted from the LHS equation; the respective terms for the stiffness matrix and damping matrix are automatically added; see description below
* | **coordinateIndexPerNode** [type = ArrayIndex, default = []]:
  | this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed
* | **tempCoordinates** [type = NumpyVector, default = []]:
  | temporary vector containing coordinates
* | **tempCoordinates_t** [type = NumpyVector, default = []]:
  | temporary vector containing velocity coordinates
* | **tempCoordinates_tt** [type = NumpyVector, default = []]:
  | temporary vector containing acceleration coordinates



The item VObjectGenericODE2 has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.], size = 4]:
  | RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
* | **triangleMesh** [type = NumpyMatrixI, default = MatrixI[]]:
  | a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
* | **showNodes** [type = Bool, default = False]:
  | set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'
* | **graphicsDataUserFunction** [type = PyFunctionGraphicsData, default =  0]:
  | A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics data is draw in global coordinates; it can be used to implement user element visualization, e.g., beam elements or simple mechanical systems; note that this user function may significantly slow down visualization




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


