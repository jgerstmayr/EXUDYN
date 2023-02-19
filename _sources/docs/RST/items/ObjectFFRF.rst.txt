

.. _sec-item-objectffrf:

ObjectFFRF
==========

This object is used to represent equations modelled by the FFRF. It contains a RigidBodyNode (always node 0) and a list of other nodes representing the finite element nodes used in the FFRF. Note that temporary matrices and vectors are subject of change in future.
 



The item \ **ObjectFFRF**\  with type = 'FFRF' has the following parameters:

 

* | **name** [type = String, default = '']:
  | objects's unique name
* | **nodeNumbers** [type = ArrayNodeIndex, default = []]:
  | node numbers which provide the coordinates for the object (consecutively as provided in this list); the \ :math:`(n_\mathrmnf+1)`\  nodes represent the nodes of the FE mesh (except for node 0); the global nodal position needs to be reconstructed from the rigid-body motion of the reference frame
* | **massMatrixFF** [type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | body-fixed and ONLY flexible coordinates part of mass matrix of object given in Python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format
* | **stiffnessMatrixFF** [type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | body-fixed and ONLY flexible coordinates part of stiffness matrix of object in Python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format
* | **dampingMatrixFF** [type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | body-fixed and ONLY flexible coordinates part of damping matrix of object in Python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format
* | **forceVector** [type = NumpyVector, default = []]:
  | generalized, force vector added to RHS; the rigid body part \ :math:`{\mathbf{f}}_r`\  is directly applied to rigid body coordinates while the flexible part \ :math:`{\mathbf{f}}\indf`\  is transformed from global to local coordinates; note that this force vector only allows to add gravity forces for bodies with COM at the origin of the reference frame
* | **forceUserFunction** [type = PyFunctionVectorMbsScalarIndex2Vector, default =  0]:
  | A Python user function which computes the generalized user force vector for the ODE2 equations; note the different coordinate systems for rigid body and flexible part; The function args are mbs, time, objectNumber, coordinates q (without reference values) and coordinate velocities q_t; see description below
* | **massMatrixUserFunction** [type = PyFunctionMatrixMbsScalarIndex2Vector, default =  0]:
  | A Python user function which computes the TOTAL mass matrix (including reference node) and adds the local constant mass matrix; note the different coordinate systems as described in the FFRF mass matrix; see description below
* | **computeFFRFterms** [type = Bool, default = True]:
  | flag decides whether the standard FFRF terms are computed; use this flag for user-defined definition of FFRF terms in mass matrix and quadratic velocity vector
* | **coordinateIndexPerNode** [type = ArrayIndex, default = []]:
  | this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed
* | **objectIsInitialized** [type = Bool, default = False]:
  | ALWAYS set to False! flag used to correctly initialize all FFRF matrices; as soon as this flag is False, internal (constant) FFRF matrices are recomputed during Assemble()
* | **physicsMass** [type = UReal, default = 0.]:
  | total mass [SI:kg] of FFRF object, auto-computed from mass matrix \ :math:`\LUb{\mathbf{M}}`\ 
* | **physicsInertia** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | inertia tensor [SI:kgm\ :math:`^2`\ ] of rigid body w.r.t. to the reference point of the body, auto-computed from the mass matrix \ :math:`\LUb{\mathbf{M}}`\ 
* | **physicsCenterOfMass** [type = Vector3D, default = [0.,0.,0.], size = 3]:
  | local position of center of mass (COM); auto-computed from mass matrix \ :math:`\LUb{\mathbf{M}}`\ 
* | **PHItTM** [type = NumpyMatrix, default = Matrix[]]:
  | projector matrix; may be removed in future
* | **referencePositions** [type = NumpyVector, default = []]:
  | vector containing the reference positions of all flexible nodes
* | **tempVector** [type = NumpyVector, default = []]:
  | temporary vector
* | **tempCoordinates** [type = NumpyVector, default = []]:
  | temporary vector containing coordinates
* | **tempCoordinates_t** [type = NumpyVector, default = []]:
  | temporary vector containing velocity coordinates
* | **tempRefPosSkew** [type = NumpyMatrix, default = Matrix[]]:
  | temporary matrix with skew symmetric local (deformed) node positions
* | **tempVelSkew** [type = NumpyMatrix, default = Matrix[]]:
  | temporary matrix with skew symmetric local node velocities



The item VObjectFFRF has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown; use visualizationSettings.bodies.deformationScaleFactor to draw scaled (local) deformations; the reference frame node is shown with additional letters RF
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.], size = 4]:
  | RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
* | **triangleMesh** [type = NumpyMatrixI, default = MatrixI[]]:
  | a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
* | **showNodes** [type = Bool, default = False]:
  | set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


