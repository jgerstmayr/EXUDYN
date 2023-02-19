

.. _sec-item-objectffrfreducedorder:

ObjectFFRFreducedOrder
======================

This object is used to represent modally reduced flexible bodies using the FFRF and the CMS. It can be used to model real-life mechanical systems imported from finite element codes or Python tools such as NETGEN/NGsolve, see the \ ``FEMinterface``\  in Section :ref:`sec-fem-feminterface---init--`\ . It contains a RigidBodyNode (always node 0) and a NodeGenericODE2 representing the modal coordinates. Currently, equations must be defined within user functions, which are available in the FEM module, see class \ ``ObjectFFRFreducedOrderInterface``\ , especially the user functions \ ``UFmassFFRFreducedOrder``\  and \ ``UFforceFFRFreducedOrder``\ , Section :ref:`sec-fem-objectffrfreducedorderinterface-addobjectffrfreducedorderwithuserfunctions`\ .
 



The item \ **ObjectFFRFreducedOrder**\  with type = 'FFRFreducedOrder' has the following parameters:

 

* | **name** [type = String, default = '']:
  | objects's unique name
* | **nodeNumbers** [type = ArrayNodeIndex, default = []]:
  | node numbers of rigid body node and NodeGenericODE2 for modal coordinates; the global nodal position needs to be reconstructed from the rigid-body motion of the reference frame, the modal coordinates and the mode basis
* | **massMatrixReduced** [type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | body-fixed and ONLY flexible coordinates part of reduced mass matrix; provided as MatrixContainer(sparse/dense matrix)
* | **stiffnessMatrixReduced** [type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | body-fixed and ONLY flexible coordinates part of reduced stiffness matrix; provided as MatrixContainer(sparse/dense matrix)
* | **dampingMatrixReduced** [type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | body-fixed and ONLY flexible coordinates part of reduced damping matrix; provided as MatrixContainer(sparse/dense matrix)
* | **forceUserFunction** [type = PyFunctionVectorMbsScalarIndex2Vector, default =  0]:
  | A Python user function which computes the generalized user force vector for the ODE2 equations; see description below
* | **massMatrixUserFunction** [type = PyFunctionMatrixMbsScalarIndex2Vector, default =  0]:
  | A Python user function which computes the TOTAL mass matrix (including reference node) and adds the local constant mass matrix; see description below
* | **computeFFRFterms** [type = Bool, default = True]:
  | flag decides whether the standard FFRF/CMS terms are computed; use this flag for user-defined definition of FFRF terms in mass matrix and quadratic velocity vector
* | **modeBasis** [type = NumpyMatrix, default = Matrix[]]:
  | mode basis, which transforms reduced coordinates to (full) nodal coordinates, written as a single vector \ :math:`[u_x,n_0,\,u_y,n_0,\,u_z,n_0,\,\ldots,\,u_x,n_n,\,u_y,n_n,\,u_z,n_n]\tp`\ 
* | **outputVariableModeBasis** [type = NumpyMatrix, default = Matrix[]]:
  | mode basis, which transforms reduced coordinates to output variables per mode and per node; \ :math:`s_OV`\  is the size of the output variable, e.g., 6 for stress modes (\ :math:`S_xx,...,S_xy`\ )
* | **outputVariableTypeModeBasis** [type = OutputVariableType, default = OutputVariableType::_None]:
  | this must be the output variable type of the outputVariableModeBasis, e.g. exu.OutputVariableType.Stress
* | **referencePositions** [type = NumpyVector, default = []]:
  | vector containing the reference positions of all flexible nodes, needed for graphics
* | **objectIsInitialized** [type = Bool, default = False]:
  | ALWAYS set to False! flag used to correctly initialize all FFRF matrices; as soon as this flag is False, some internal (constant) FFRF matrices are recomputed during Assemble()
* | **physicsMass** [type = UReal, default = 0.]:
  | total mass [SI:kg] of FFRFreducedOrder object
* | **physicsInertia** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | inertia tensor [SI:kgm\ :math:`^2`\ ] of rigid body w.r.t. to the reference point of the body
* | **physicsCenterOfMass** [type = Vector3D, default = [0.,0.,0.], size = 3]:
  | local position of center of mass (COM)
* | **mPsiTildePsi** [type = NumpyMatrix, default = Matrix[]]:
  | special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface
* | **mPsiTildePsiTilde** [type = NumpyMatrix, default = Matrix[]]:
  | special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface
* | **mPhitTPsi** [type = NumpyMatrix, default = Matrix[]]:
  | special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface
* | **mPhitTPsiTilde** [type = NumpyMatrix, default = Matrix[]]:
  | special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface
* | **mXRefTildePsi** [type = NumpyMatrix, default = Matrix[]]:
  | special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface
* | **mXRefTildePsiTilde** [type = NumpyMatrix, default = Matrix[]]:
  | special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface
* | **physicsCenterOfMassTilde** [type = Matrix3D, default = [[0,0,0], [0,0,0], [0,0,0]]]:
  | tilde matrix from local position of COM; autocomputed during initialization
* | **tempUserFunctionForce** [type = NumpyVector, default = []]:
  | temporary vector for UF force



The item VObjectFFRFreducedOrder has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown; use visualizationSettings.bodies.deformationScaleFactor to draw scaled (local) deformations; the reference frame node is shown with additional letters RF
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.], size = 4]:
  | RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
* | **triangleMesh** [type = NumpyMatrixI, default = MatrixI[]]:
  | a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
* | **showNodes** [type = Bool, default = False]:
  | set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


