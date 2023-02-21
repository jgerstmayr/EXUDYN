

.. _sec-item-objectffrfreducedorder:

ObjectFFRFreducedOrder
======================

This object is used to represent modally reduced flexible bodies using the FFRF and the CMS. It can be used to model real-life mechanical systems imported from finite element codes or Python tools such as NETGEN/NGsolve, see the \ ``FEMinterface``\  in Section :ref:`sec-fem-feminterface---init--`\ . It contains a RigidBodyNode (always node 0) and a NodeGenericODE2 representing the modal coordinates. Currently, equations must be defined within user functions, which are available in the FEM module, see class \ ``ObjectFFRFreducedOrderInterface``\ , especially the user functions \ ``UFmassFFRFreducedOrder``\  and \ ``UFforceFFRFreducedOrder``\ , Section :ref:`sec-fem-objectffrfreducedorderinterface-addobjectffrfreducedorderwithuserfunctions`\ .

Authors: Gerstmayr Johannes, Zwölfer Andreas

\ **Additional information for ObjectFFRFreducedOrder**\ :

* | The Object has the following types = \ ``Body``\ , \ ``MultiNoded``\ , \ ``SuperElement``\ 
* | Requested node type: read detailed information of item
* | \ **Short name**\  for Python = \ ``CMSobject``\ 
* | \ **Short name**\  for Python visualization object = \ ``VCMSobject``\ 


The item \ **ObjectFFRFreducedOrder**\  with type = 'FFRFreducedOrder' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **nodeNumbers** [\ :math:`\mathbf{n} = [n_0,\,n_1]\tp`\ , type = ArrayNodeIndex, default = []]:
  | node numbers of rigid body node and NodeGenericODE2 for modal coordinates; the global nodal position needs to be reconstructed from the rigid-body motion of the reference frame, the modal coordinates and the mode basis
* | **massMatrixReduced** [\ :math:`{\mathbf{M}}\indred \in \Rcal^{n_m \times n_m}`\ , type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | body-fixed and ONLY flexible coordinates part of reduced mass matrix; provided as MatrixContainer(sparse/dense matrix)
* | **stiffnessMatrixReduced** [\ :math:`{\mathbf{K}}\indred \in \Rcal^{n_m \times n_m}`\ , type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | body-fixed and ONLY flexible coordinates part of reduced stiffness matrix; provided as MatrixContainer(sparse/dense matrix)
* | **dampingMatrixReduced** [\ :math:`{\mathbf{D}}\indred \in \Rcal^{n_m \times n_m}`\ , type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | body-fixed and ONLY flexible coordinates part of reduced damping matrix; provided as MatrixContainer(sparse/dense matrix)
* | **forceUserFunction** [\ :math:`{\mathbf{f}}\induser \in \Rcal^{n_{ODE2}}`\ , type = PyFunctionVectorMbsScalarIndex2Vector, default =  0]:
  | A Python user function which computes the generalized user force vector for the ODE2 equations; see description below
* | **massMatrixUserFunction** [\ :math:`{\mathbf{M}}\induser \in \Rcal^{n_{ODE2}\times n_{ODE2}}`\ , type = PyFunctionMatrixMbsScalarIndex2Vector, default =  0]:
  | A Python user function which computes the TOTAL mass matrix (including reference node) and adds the local constant mass matrix; see description below
* | **computeFFRFterms** [type = Bool, default = True]:
  | flag decides whether the standard FFRF/CMS terms are computed; use this flag for user-defined definition of FFRF terms in mass matrix and quadratic velocity vector
* | **modeBasis** [\ :math:`\LU{b}{\tPsi} \in \Rcal^{n\indf \times n_{m}}`\ , type = NumpyMatrix, default = Matrix[]]:
  | mode basis, which transforms reduced coordinates to (full) nodal coordinates, written as a single vector \ :math:`[u_{x,n_0},\,u_{y,n_0},\,u_{z,n_0},\,\ldots,\,u_{x,n_n},\,u_{y,n_n},\,u_{z,n_n}]\tp`\ 
* | **outputVariableModeBasis** [\ :math:`\LU{b}{\tPsi}_{OV} \in \Rcal^{n_n \times (n_{m}\cdot s_{OV})}`\ , type = NumpyMatrix, default = Matrix[]]:
  | mode basis, which transforms reduced coordinates to output variables per mode and per node; \ :math:`s_{OV}`\  is the size of the output variable, e.g., 6 for stress modes (\ :math:`S_{xx},...,S_{xy}`\ )
* | **outputVariableTypeModeBasis** [type = OutputVariableType, default = OutputVariableType::_None]:
  | this must be the output variable type of the outputVariableModeBasis, e.g. exu.OutputVariableType.Stress
* | **referencePositions** [\ :math:`\LU{b}{{\mathbf{x}}}\cRef \in \Rcal^{n\indf}`\ , type = NumpyVector, default = []]:
  | vector containing the reference positions of all flexible nodes, needed for graphics
* | **objectIsInitialized** [type = Bool, default = False]:
  | ALWAYS set to False! flag used to correctly initialize all FFRF matrices; as soon as this flag is False, some internal (constant) FFRF matrices are recomputed during Assemble()
* | **physicsMass** [\ :math:`m`\ , type = UReal, default = 0.]:
  | total mass [SI:kg] of FFRFreducedOrder object
* | **physicsInertia** [\ :math:`{\mathbf{J}}_r \in \Rcal^{3 \times 3}`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | inertia tensor [SI:kgm\ :math:`^2`\ ] of rigid body w.r.t. to the reference point of the body
* | **physicsCenterOfMass** [\ :math:`\LU{b}{{\mathbf{b}}}_{COM}`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
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
* | **physicsCenterOfMassTilde** [\ :math:`\LU{b}{\tilde {\mathbf{b}}}_{COM}`\ , type = Matrix3D, default = [[0,0,0], [0,0,0], [0,0,0]]]:
  | tilde matrix from local position of COM; autocomputed during initialization
* | **tempUserFunctionForce** [\ :math:`{\mathbf{f}}_{temp} \in \Rcal^{n_{ODE2}}`\ , type = NumpyVector, default = []]:
  | temporary vector for UF force
* | **visualization** [type = VObjectFFRFreducedOrder]:
  | parameters for visualization of item



The item VObjectFFRFreducedOrder has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown; use visualizationSettings.bodies.deformationScaleFactor to draw scaled (local) deformations; the reference frame node is shown with additional letters RF
* | **color** [type = Float4, size = 4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
* | **triangleMesh** [type = NumpyMatrixI, default = MatrixI[]]:
  | a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
* | **showNodes** [type = Bool, default = False]:
  | set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Coordinates``\ : 
  | all ODE2 coordinates
* | ``Coordinates\_t``\ : 
  | all ODE2 velocity coordinates
* | ``Force``\ : 
  | generalized forces for all coordinates (residual of all forces except mass*accleration; corresponds to ComputeODE2LHS)




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


