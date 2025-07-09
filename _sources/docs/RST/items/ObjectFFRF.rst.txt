

.. _sec-item-objectffrf:

ObjectFFRF
==========

This object is used to represent equations modelled by the \ :ref:`FFRF <FFRF>`\ . It contains a RigidBodyNode (always node 0) and a list of other nodes representing the finite element nodes used in the \ :ref:`FFRF <FFRF>`\ . Note that temporary matrices and vectors are subject of change in future. Usually you SHOULD NOT USE THIS OBJECT - use the much more efficient ObjectFFRFreducedOrder object with modal reduction instead.

Authors: Gerstmayr Johannes, Zwölfer Andreas

\ **Additional information for ObjectFFRF**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Body``\ , \ ``MultiNoded``\ , \ ``SuperElement``\ 
* | Requested \ ``Node``\  type: read detailed information of item


The item \ **ObjectFFRF**\  with type = 'FFRF' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **nodeNumbers** [\ :math:`\mathbf{n}\indf = [n_0,\,\ldots,\,n_{n_\mathrm{nf}}]\tp`\ , type = ArrayNodeIndex, default = []]:
  | node numbers which provide the coordinates for the object (consecutively as provided in this list); the \ :math:`(n_\mathrm{nf}+1)`\  nodes represent the nodes of the FE mesh (except for node 0); the global nodal position needs to be reconstructed from the rigid-body motion of the reference frame
* | **massMatrixFF** [\ :math:`\LU{b}{{\mathbf{M}}} \in \Rcal^{n\indf \times n\indf}`\ , type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | body-fixed and ONLY flexible coordinates part of mass matrix of object given in Python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format
* | **stiffnessMatrixFF** [\ :math:`\LU{b}{{\mathbf{K}}} \in \Rcal^{n\indf \times n\indf}`\ , type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | body-fixed and ONLY flexible coordinates part of stiffness matrix of object in Python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format
* | **dampingMatrixFF** [\ :math:`\LU{b}{{\mathbf{D}}} \in \Rcal^{n\indf \times n\indf}`\ , type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | body-fixed and ONLY flexible coordinates part of damping matrix of object in Python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format
* | **forceVector** [\ :math:`\LU{0}{{\mathbf{f}}} = [\LU{0}{{\mathbf{f}}\indr},\; \LU{0}{{\mathbf{f}}\indf}]\tp \in \Rcal^{n_c}`\ , type = NumpyVector, default = []]:
  | generalized, force vector added to RHS; the rigid body part \ :math:`{\mathbf{f}}_r`\  is directly applied to rigid body coordinates while the flexible part \ :math:`{\mathbf{f}}\indf`\  is transformed from global to local coordinates; note that this force vector only allows to add gravity forces for bodies with \ :ref:`COM <COM>`\  at the origin of the reference frame
* | **forceUserFunction** [\ :math:`{\mathbf{f}}_{user} =  [\LU{0}{{\mathbf{f}}_{\mathrm{r},user}},\; \LU{b}{{\mathbf{f}}_{\mathrm{f},user}}]\tp \in \Rcal^{n_c}`\ , type = PyFunctionVectorMbsScalarIndex2Vector, default =  0]:
  | A Python user function which computes the generalized user force vector for the \ :ref:`ODE2 <ODE2>`\  equations; note the different coordinate systems for rigid body and flexible part; The function args are mbs, time, objectNumber, coordinates q (without reference values) and coordinate velocities q_t; see description below
* | **massMatrixUserFunction** [\ :math:`{\mathbf{M}}_{user} \in \Rcal^{n_c\times n_c}`\ , type = PyFunctionMatrixMbsScalarIndex2Vector, default =  0]:
  | A Python user function which computes the TOTAL mass matrix (including reference node) and adds the local constant mass matrix; note the different coordinate systems as described in the \ :ref:`FFRF <FFRF>`\  mass matrix; see description below
* | **computeFFRFterms** [type = Bool, default = True]:
  | flag decides whether the standard \ :ref:`FFRF <FFRF>`\  terms are computed; use this flag for user-defined definition of \ :ref:`FFRF <FFRF>`\  terms in mass matrix and quadratic velocity vector
* | **coordinateIndexPerNode** [type = ArrayIndex, default = []]:
  | this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed
* | **objectIsInitialized** [type = Bool, default = False]:
  | ALWAYS set to False! flag used to correctly initialize all \ :ref:`FFRF <FFRF>`\  matrices; as soon as this flag is False, internal (constant) \ :ref:`FFRF <FFRF>`\  matrices are recomputed during Assemble()
* | **physicsMass** [\ :math:`m`\ , type = UReal, default = 0.]:
  | total mass [SI:kg] of \ :ref:`FFRF <FFRF>`\  object, auto-computed from mass matrix \ :math:`\LU{b}{{\mathbf{M}}}`\ 
* | **physicsInertia** [\ :math:`J_r \in \Rcal^{3 \times 3}`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | inertia tensor [SI:kgm\ :math:`^2`\ ] of rigid body w.r.t. to the reference point of the body, auto-computed from the mass matrix \ :math:`\LU{b}{{\mathbf{M}}}`\ 
* | **physicsCenterOfMass** [\ :math:`\LU{b}{{\mathbf{b}}}_{COM}`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | local position of center of mass (\ :ref:`COM <COM>`\ ); auto-computed from mass matrix \ :math:`\LU{b}{{\mathbf{M}}}`\ 
* | **PHItTM** [\ :math:`\tPhi\indt\tp \in \Rcal^{n\indf \times 3}`\ , type = NumpyMatrix, default = Matrix[]]:
  | projector matrix; may be removed in future
* | **referencePositions** [\ :math:`{\mathbf{x}}\cRef \in \Rcal^{n\indf}`\ , type = NumpyVector, default = []]:
  | vector containing the reference positions of all flexible nodes
* | **tempVector** [\ :math:`{\mathbf{v}}_{temp} \in \Rcal^{n\indf}`\ , type = NumpyVector, default = []]:
  | temporary vector
* | **tempCoordinates** [\ :math:`{\mathbf{c}}_{temp} \in \Rcal^{n\indf}`\ , type = NumpyVector, default = []]:
  | temporary vector containing coordinates
* | **tempCoordinates_t** [\ :math:`\dot {\mathbf{c}}_{temp} \in \Rcal^{n\indf}`\ , type = NumpyVector, default = []]:
  | temporary vector containing velocity coordinates
* | **tempRefPosSkew** [\ :math:`\tilde{\mathbf{p}}\indf \in \Rcal^{n\indf \times 3}`\ , type = NumpyMatrix, default = Matrix[]]:
  | temporary matrix with skew symmetric local (deformed) node positions
* | **tempVelSkew** [\ :math:`\dot{\tilde{\mathbf{c}}}\indf \in \Rcal^{n\indf \times 3}`\ , type = NumpyMatrix, default = Matrix[]]:
  | temporary matrix with skew symmetric local node velocities
* | **visualization** [type = VObjectFFRF]:
  | parameters for visualization of item



The item VObjectFFRF has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown; use visualizationSettings.bodies.deformationScaleFactor to draw scaled (local) deformations; the reference frame node is shown with additional letters RF
* | **color** [type = Float4, size = 4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
* | **triangleMesh** [type = NumpyMatrixI, default = MatrixI[]]:
  | a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
* | **showNodes** [type = Bool, default = False]:
  | set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'


----------

.. _description-objectffrf:

DESCRIPTION of ObjectFFRF
-------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Coordinates``\ : 
  | all \ :ref:`ODE2 <ODE2>`\  coordinates
* | ``Coordinates\_t``\ : 
  | all \ :ref:`ODE2 <ODE2>`\  velocity coordinates
* | ``Coordinates\_tt``\ : 
  | all \ :ref:`ODE2 <ODE2>`\  acceleration coordinates
* | ``Force``\ : 
  | generalized forces for all coordinates (residual of all forces except mass*accleration; corresponds to ComputeODE2LHS)



Additional output variables for superelement node access
--------------------------------------------------------

Functions like \ ``GetObjectOutputSuperElement(...)``\ , see Section :ref:`sec-mainsystem-object`\ , 
or \ ``SensorSuperElement``\ , see Section :ref:`sec-mainsystem-sensor`\ , directly access special output variables
(\ ``OutputVariableType``\ ) of the mesh nodes \ :math:`n_i`\  of the superelement.
Additionally, the contour drawing of the object can make use the \ ``OutputVariableType``\  of the meshnodes.

.. _sec-objectffrf-superelementoutput:


Super element output variables
------------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | super element output variables
     - | symbol
     - | description
   * - | Position
     - | \ :math:`\LU{0}{{\mathbf{p}}}\cConfig(n_i) = \LU{0}{\pRef\cConfig} + \LU{0b}{\Rot}\cConfig \LU{b}{{\mathbf{p}}}\cConfig(n_i)`\ 
     - | global position of mesh node \ :math:`n_i`\  including rigid body motion and flexible deformation
   * - | Displacement
     - | \ :math:`\LU{0}{{\mathbf{c}}}\cConfig(n_i) = \LU{0}{{\mathbf{p}}\cConfig(n_i)} - \LU{0}{{\mathbf{p}}\cRef(n_i)}`\ 
     - | global displacement of mesh node \ :math:`n_i`\  including rigid body motion and flexible deformation
   * - | Velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}\cConfig(n_i) = \LU{0}{\dot \pRef\cConfig} + \LU{0b}{\Rot}\cConfig (\LU{b}{\dot {\mathbf{q}}\indf}\cConfig(n_i) + \LU{b}{\tomega}\cConfig \times \LU{b}{{\mathbf{p}}}\cConfig(n_i))`\ 
     - | global velocity of mesh node \ :math:`n_i`\  including rigid body motion and flexible deformation
   * - | Acceleration
     - | \ :math:`\begin{array}{l} \LU{0}{{\mathbf{a}}}\cConfig(n_i) = \LU{0}{\ddot \pRef\cConfig}\cConfig \\
                          + \LU{0b}{\Rot}\cConfig \LU{b}{\ddot {\mathbf{q}}\indf}\cConfig(n_i) \\
                          + 2\LU{0}{\tomega}\cConfig \times \LU{0b}{\Rot}\cConfig \LU{b}{\dot {\mathbf{q}}\indf}\cConfig(n_i) \\
                          + \LU{0}{\talpha}\cConfig \times \LU{0}{{\mathbf{p}}}\cConfig(n_i) \\
                          + \LU{0}{\tomega}\cConfig \times (\LU{0}{\tomega}\cConfig \times \LU{0}{{\mathbf{p}}}\cConfig(n_i)) \end{array}`\ 
     - | global acceleration of mesh
                          node \ :math:`n_i`\  including rigid body motion and flexible deformation; note that \ :math:`\LU{0}{{\mathbf{p}}}\cConfig(n_i) = \LU{0b}{\Rot} \LU{b}{{\mathbf{p}}}\cConfig(n_i)`\ 
   * - | DisplacementLocal
     - | \ :math:`\LU{b}{{\mathbf{d}}}\cConfig(n_i) = \LU{b}{{\mathbf{p}}}\cConfig(n_i) - \LU{b}{{\mathbf{x}}}\cRef(n_i)`\ 
     - | local displacement of mesh node \ :math:`n_i`\ , representing the flexible deformation within the body frame; note that \ :math:`\LU{0}{{\mathbf{u}}}\cConfig \neq \LU{0b}{\Rot}\LU{b}{{\mathbf{d}}}\cConfig`\  !
   * - | VelocityLocal
     - | \ :math:`\LU{b}{\dot {\mathbf{q}}\indf}\cConfig(n_i)`\ 
     - | local velocity of mesh node \ :math:`n_i`\ , representing the rate of flexible deformation within the body frame


Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | object coordinates
     - | \ :math:`{\mathbf{q}} = [{\mathbf{q}}\indt\tp,\;{\mathbf{q}}\indr\tp,\;{\mathbf{q}}\indf\tp]\tp`\ 
     - | object coordinates
   * - | rigid body coordinates
     - | \ :math:`{\mathbf{q}}\indrigid = [{\mathbf{q}}\indt\tp,\;{\mathbf{q}}\indr\tp]\tp =  [q_0,\,q_1,\,q_2,\,\psi_0,\,\psi_1,\,\psi_2,\,\psi_3]\tp`\ 
     - | rigid body coordinates in case of Euler parameters
   * - | reference frame (rigid body) position
     - | \ :math:`\LU{0}{\pRef\cConfig} = \LU{0}{{\mathbf{q}}_\mathrm{t,config}}+\LU{0}{{\mathbf{q}}_\mathrm{t,ref}}`\ 
     - | global position of underlying rigid body node \ :math:`n_0`\  which defines the reference frame origin
   * - | reference frame (rigid body) orientation
     - | \ :math:`\LU{0b}{\Rot(\ttheta)}\cConfig`\ 
     - | transformation matrix for transformation of local (reference frame) to global coordinates, given by underlying rigid body node \ :math:`n_0`\ 
   * - | local nodal position
     - | \ :math:`\LU{b}{{\mathbf{p}}^{(i)}} = \LU{b}{{\mathbf{x}}^{(i)}}\cRef + \LU{b}{{\mathbf{q}}\indf^{(i)}}`\ 
     - | vector of body-fixed (local) position of node \ :math:`(i)`\ , including flexible part
   * - | local nodal positions
     - | \ :math:`\LU{b}{{\mathbf{p}}} = \LU{b}{{\mathbf{x}}}\cRef + \LU{b}{{\mathbf{q}}\indf}`\ 
     - | vector of all body-fixed (local) nodal positions including flexible part
   * - | rotation coordinates
     - | \ :math:`\ttheta\cCur = [\psi_0,\,\psi_1,\,\psi_2,\,\psi_3]\tp\cRef + [\psi_0,\,\psi_1,\,\psi_2,\,\psi_3]\cCur\tp`\ 
     - | rigid body coordinates in case of Euler parameters
   * - | flexible coordinates
     - | \ :math:`\LU{b}{{\mathbf{q}}\indf}`\ 
     - | flexible, body-fixed coordinates
   * - | transformation of flexible coordinates
     - | \ :math:`\LU{0b}{{\mathbf{A}}_{bd}} = \mathrm{diag}([\LU{0b}{{\mathbf{A}}},\;\ldots,\;\LU{0b}{{\mathbf{A}}})`\ 
     - | block diagonal transformation matrix, which transforms all flexible coordinates from local to global coordinates

The derivations follow Zwölfer and Gerstmayr  with only small modifications in the notation.

Nodal coordinates
-----------------

Consider an object with \ :math:`n = 1 + n_\mathrm{nf}`\  nodes, \ :math:`n_\mathrm{nf}`\  being the number of 'flexible' nodes and one additional node is the rigid body node for the reference frame.
The list if node numbers is \ :math:`[n_0,\,\ldots,\,n_{n_\mathrm{nf}}]`\  and the according numbers of 
nodal coordinates are \ :math:`[n_{c_0},\,\ldots,\,n_{c_n}]`\ , where \ :math:`n_0`\  denotes the rigid body node.
This gives \ :math:`n_c`\  total nodal coordinates, 

.. math::

   n_c = \sum_{i=0}^{n_\mathrm{nf}} n_{c_i} ,


whereof the number of flexible coordinates is

.. math::

   n\indf = 3 \cdot n_\mathrm{nf} .



The total number of equations (=coordinates) of the object is \ :math:`n_c`\ .
The first node \ :math:`n_0`\  represents the rigid body motion of the underlying reference frame with \ :math:`n_{c\indr} = n_{c_0}`\  coordinates \ (e.g., 
\ :math:`n_{c\indr}=6`\  coordinates for Euler angles and \ :math:`n_{c\indr}=7`\  coordinates in case of Euler parameters; currently only the Euler parameter
case is implemented.). 


Kinematics
----------

We assume a finite element mesh with 
The kinematics of the \ :ref:`FFRF <FFRF>`\  is based on a splitting of 
translational (\ :math:`{\mathbf{c}}_t \in \Rcal^{n\indf}`\ ), rotational (\ :math:`{\mathbf{c}}\indr \in \Rcal^{n\indf}`\ ) and flexible (\ :math:`{\mathbf{c}}\indf \in \Rcal^{n\indf}`\ ) nodal displacements, 

.. math::
   :label: eq-objectffrf-coordinatessplitting

   \LU{0}{{\mathbf{c}}} = \LU{0}{{\mathbf{c}}\indt} + \LU{0}{{\mathbf{c}}\indr} + \LU{0}{{\mathbf{c}}\indf} .


which are written in global coordinates in Eq. :eq:`eq-objectffrf-coordinatessplitting`\  but will be transformed to other coordinates later on.

In the present formulation of \ ``ObjectFFRF``\ , we use the following set of object coordinates (unknowns)

.. math::

   {\mathbf{q}} = \left[\LU{0}{{\mathbf{q}}\indt\tp} \;\; \ttheta\tp \;\; \LU{b}{{\mathbf{q}}\indf\tp} \right]\tp \in \Rcal^{n_c}


with \ :math:`\LU{0}{{\mathbf{q}}}\indt \in \Rcal^{3}`\ , \ :math:`\ttheta \in \Rcal^{4}`\  and \ :math:`\LU{b}{{\mathbf{q}}}\indf \in \Rcal^{n\indf}`\ .
Note that parts of the coordinates \ :math:`{\mathbf{q}}`\  can be already interpreted in specific coordinate systems, which is therefore added.

With the relations 

.. math::
   :label: eq-objectffrf-phit

   \tPhi\indt &=& \left[\mathbf{I}_{3 \times 3} ,\; \ldots ,\; \mathbf{I}_{3 \times 3} \right]\tp \in \Rcal^{n\indf \times 3} ,\\
   \LU{0}{{\mathbf{c}}\indt} &=& \tPhi\indt \LU{0}{{\mathbf{q}}\indt} ,\\
   \LU{0}{{\mathbf{c}}\indr} &=& \left(\LU{0b}{{\mathbf{A}}_{bd}} - {\mathbf{I}}_{bd}\right) \LU{b}{{\mathbf{x}}\cRef} ,\\
   \LU{0}{{\mathbf{c}}\indf} &=& \LU{0b}{{\mathbf{A}}_{bd}} \LU{b}{{\mathbf{q}}\indf} , \mathrm{and}\\
   {\mathbf{I}}_{bd} &=& \mathrm{diag}(\mathbf{I}_{3 \times 3}, \; \ldots ,\; \mathbf{I}_{3 \times 3}) \in \Rcal^{n\indf \times n\indf}  ,


we obtain the total relation of (global) nodal displacements to the object coordinates

.. math::

   \LU{0}{{\mathbf{c}}} = \tPhi\indt \LU{0}{{\mathbf{q}}\indt} + \left(\LU{0b}{{\mathbf{A}}_{bd}} - {\mathbf{I}}_{bd}\right) \LU{b}{{\mathbf{x}}\cRef} + \LU{0b}{{\mathbf{A}}_{bd}} \LU{b}{{\mathbf{q}}\indf} .


On velocity level, we have

.. math::

   \LU{0}{\dot {\mathbf{c}}} = {\mathbf{L}} \dot {\mathbf{q}} ,


with the matrix \ :math:`{\mathbf{L}} \in \Rcal^{n\indf \times n_c}`\ 

.. math::

   {\mathbf{L}} = \left[\tPhi\indt ,\;\; -\LU{0b}{{\mathbf{A}}_{bd}} \LU{b}{\tilde {\mathbf{p}}} \LU{b}{{\mathbf{G}}} ,\;\; \LU{0b}{{\mathbf{A}}_{bd}} \right]


with the rotation parameters specific matrix \ :math:`\LU{b}{{\mathbf{G}}}`\ , implicitly defined in the rigid body node by the relation \ :math:`\LU{b}{\tomega} = \LU{b}{{\mathbf{G}}} \dot \ttheta`\ 
and the body-fixed nodal position vector (for node \ :math:`i`\ )

.. math::

   \LU{b}{{\mathbf{p}}} = \LU{b}{{\mathbf{x}}\cRef} + \LU{b}{{\mathbf{q}}\indf}, \quad \LU{b}{{\mathbf{p}}^{(i)}} = \LU{b}{{\mathbf{x}}^{(i)}\cRef} + \LU{b}{{\mathbf{q}}_{\mathrm{f},i}^{(i)}}


and the special tilde matrix for vectors \ :math:`{\mathbf{p}} \in \Rcal^{3 {n_\mathrm{nf}}}`\ , 

.. math::
   :label: eq-objectffrf-specialtilde

   \LU{b}{\tilde {\mathbf{p}}} = \vr{\LU{b}{\tilde{\mathbf{p}}^{(i)}}}{\vdots}{\LU{b}{\tilde{\mathbf{p}}^{(i)}}} \in \Rcal^{3{n_\mathrm{nf}} \times 3} .


with the tilde operator for a \ :math:`{\mathbf{p}}^{(i)} \in \Rcal^{3}`\  defined in the common notations section.

Equations of motion
-------------------

We use the Lagrange equations extended for constraint \ :math:`{\mathbf{g}}`\ ,

.. math::

   \frac{d}{dt} \left( \frac{\partial T}{\partial \dot {\mathbf{q}}\tp} \right) - \frac{\partial T}{\partial {\mathbf{q}}\tp} + \frac{\partial V}{\partial {\mathbf{q}}\tp} + \frac{\partial \tlambda\tp {\mathbf{g}}}{\partial {\mathbf{q}}\tp} = \frac{\partial W}{\partial {\mathbf{q}}\tp}


with the quantities

.. math::

   T(\LU{0}{\dot {\mathbf{c}}({\mathbf{q}}, \dot {\mathbf{q}})}) &=& \frac{1}{2}\LU{0}{\dot {\mathbf{c}}\tp} \LU{0}{{\mathbf{M}}}  \LU{0}{\dot {\mathbf{c}}} = \frac{1}{2}\LU{0}{\dot {\mathbf{c}}\tp} \LU{0b}{{\mathbf{A}}_{bd}} \LU{b}{{\mathbf{M}}} \LU{0b}{{\mathbf{A}}_{bd}}\tp  \LU{0}{\dot {\mathbf{c}}} = \frac{1}{2}\LU{0}{\dot {\mathbf{c}}\tp} \LU{b}{{\mathbf{M}}}  \LU{0}{\dot {\mathbf{c}}}\\
   V(\LU{0}{{\mathbf{q}}\indf}) &=& \frac{1}{2}\LU{b}{{\mathbf{q}}\indf\tp} \LU{b}{{\mathbf{K}}}  \LU{b}{{\mathbf{q}}\indf}  \\
   \delta W(\LU{0}{{\mathbf{c}}({\mathbf{q}})},t) &=& \LU{b}{\delta {\mathbf{c}} \tp} {\mathbf{f}}  \\
   {\mathbf{g}}({\mathbf{q}}, t) &=& \Null  \\



Note that \ :math:`\LU{b}{{\mathbf{M}}}`\  and \ :math:`\LU{b}{{\mathbf{K}}}`\  are the conventional finite element mass an stiffness 
matrices defined in the body frame.

Elementary differentiation rules of the Lagrange equations lead to

.. math::
   :label: eq-objectffrf-leq

   {\mathbf{L}}\tp {\mathbf{M}} {\mathbf{L}} \ddot {\mathbf{q}} + {\mathbf{L}}\tp {\mathbf{M}} \dot {\mathbf{L}} \dot {\mathbf{q}} + \hat {\mathbf{K}} {\mathbf{q}} + \frac{\partial {\mathbf{g}}}{\partial {\mathbf{q}}\tp} \tlambda = {\mathbf{L}}\tp {\mathbf{f}}


with \ :math:`{\mathbf{M}} = \LU{b}{{\mathbf{M}}}`\  and \ :math:`\hat {\mathbf{K}}`\  becoming obvious in Eq. :eq:`eq-objectffrf-eom`\ . 
Note that Eq. :eq:`eq-objectffrf-leq`\  is given in global coordinates for the translational part, in terms of rotation parameters
for the rotation part and in body-fixed coordinates for the flexible part of the equations.

In case that \ ``computeFFRFterms = True``\ , Eqs. :eq:`eq-objectffrf-leq`\  can be transformed into the equations of motion,

.. math::
   :label: eq-objectffrf-eom

   \left({\mathbf{M}}_{user}(mbs, t, i_N, {\mathbf{q}},\dot {\mathbf{q}}) + \mr{{\mathbf{M}}\indtt}{{\mathbf{M}}\indtr}{{\mathbf{M}}\indtf} {}{{\mathbf{M}}\indrr}{{\mathbf{M}}\indrf} {\mathrm{sym.}}{}{\LU{b}{{\mathbf{M}}}} \right) \ddot {\mathbf{q}} + \mr{0}{0}{0} {0}{0}{0} {0}{0}{\LU{b}{{\mathbf{D}}}} \dot {\mathbf{q}} + \mr{0}{0}{0} {0}{0}{0} {0}{0}{\LU{b}{{\mathbf{K}}}} {\mathbf{q}} = {\mathbf{f}}_{v}({\mathbf{q}},\dot {\mathbf{q}}) + \vp{{\mathbf{f}}\indr}{\LURU{0b}{{\mathbf{A}}}{bd}{\mathrm{T}} {\mathbf{f}}\indf} + {\mathbf{f}}_{user}(mbs, t, i_N, {\mathbf{q}}, \dot {\mathbf{q}})


in which \ ``iN``\  represents the itemNumber (=objectNumber of ObjectFFRF in mbs) in the user function.
The mass terms are given as

.. math::

   {\mathbf{M}}\indtt &=& \tPhi\indt\tp \LU{b}{{\mathbf{M}}} \tPhi\indt,\\
   {\mathbf{M}}\indtr &=& -\LU{0b}{\Rot} \tPhi\indt\tp \LU{b}{{\mathbf{M}}} \LU{b}{\tilde {\mathbf{p}}} \LU{b}{{\mathbf{G}}} ,\\
   {\mathbf{M}}\indtf &=& \LU{0b}{\Rot} \tPhi\indt\tp \LU{b}{{\mathbf{M}}} ,\\
   {\mathbf{M}}\indrr &=& \LU{b}{{\mathbf{G}}}\tp \LU{b}{\tilde {\mathbf{p}}\tp} \LU{b}{{\mathbf{M}}} \LU{b}{\tilde {\mathbf{p}}} \LU{b}{{\mathbf{G}}} ,\\
   {\mathbf{M}}\indrf &=& - \LU{b}{{\mathbf{G}}}\tp \LU{b}{\tilde {\mathbf{p}}\tp} \LU{b}{{\mathbf{M}}} .


In case that \ ``computeFFRFterms = False``\ , the mass terms \ :math:`{\mathbf{M}}\indtt, {\mathbf{M}}\indtr, {\mathbf{M}}\indtf, {\mathbf{M}}\indrr, 
{\mathbf{M}}\indrf, \LU{b}{{\mathbf{M}}}`\  in Eq. :eq:`eq-objectffrf-eom`\  are set to zero (and not computed) and
the quadratic velocity vector \ :math:`{\mathbf{f}}_{v} = \Null`\ .
Note that the user functions \ :math:`{\mathbf{f}}_{user}(mbs, t, i_N, {\mathbf{q}},\dot {\mathbf{q}})`\  and \ :math:`{\mathbf{M}}_{user}(mbs, t, i_N, {\mathbf{q}},\dot {\mathbf{q}})`\  may be empty (=0). 
The detailed equations of motion for this element can be found in .

The quadratic velocity vector follows as

.. math::

   {\mathbf{f}}_{v}({\mathbf{q}},\dot {\mathbf{q}}) = \vr {-\LU{0b}{\Rot} \tPhi\indt\tp \LU{b}{{\mathbf{M}}}\left( \omegaBDtilde \omegaBDtilde \LU{b}{{\mathbf{p}}} + 2 \omegaBDtilde \LU{b}{\dot {\mathbf{q}}}\indf - \LU{b}{\tilde {\mathbf{p}}} \LU{b}{\dot {\mathbf{G}}} \dot \ttheta \right)} {\LU{b}{{\mathbf{G}}}\tp \LU{b}{\tilde {\mathbf{p}}\tp} \LU{b}{{\mathbf{M}}} \left( \omegaBDtilde \omegaBDtilde \LU{b}{{\mathbf{p}}} + 2 \omegaBDtilde \LU{b}{\dot {\mathbf{q}}}\indf - \LU{b}{\tilde {\mathbf{p}}} \LU{b}{\dot {\mathbf{G}}} \dot \ttheta \right)} {-\LU{b}{{\mathbf{M}}} \left( \omegaBDtilde \omegaBDtilde \LU{b}{{\mathbf{p}}} + 2 \omegaBDtilde \LU{b}{\dot {\mathbf{q}}}\indf - \LU{b}{\tilde {\mathbf{p}}} \LU{b}{\dot {\mathbf{G}}} \dot \ttheta \right)}


with the special matrix

.. math::

   \omegaBDtilde = \mathrm{diag}\left(\LU{b}{\tilde \tomega_\mathrm{bd}}, \; \ldots ,\; \LU{b}{\tilde \tomega_\mathrm{bd}}  \right) \in \Rcal^{n\indf \times n\indf}


CoordinateLoads are added for each \ :ref:`ODE2 <ODE2>`\  coordinate on the RHS of the latter equation. 

If the rigid body node is using Euler parameters \ :math:`\ttheta = [\theta_0,\,\theta_1,\,\theta_2,\,\theta_3]\tp`\ , an \ **additional constraint**\  (constraint nr.\ 0) is 
added automatically for the Euler parameter norm, reading

.. math::

   1 - \sum_{i=0}^{3} \theta_i^2 = 0.



In order to suppress the rigid body motion of the mesh nodes, you should apply a ObjectConnectorCoordinateVector object with the following constraint
equations which impose constraints of a so-called Tisserand frame, giving 3 constraints for the position of the center of mass

.. math::

   \Phi\indt\tp \LU{b}{{\mathbf{M}}} {\mathbf{q}}\indf = 0


and 3 constraints for the rotation,

.. math::

   \tilde{\mathbf{x}}_{f}\tp \LU{b}{{\mathbf{M}}} {\mathbf{q}}\indf = 0



--------

\ **Userfunction**\ : ``forceUserFunction(mbs, t, itemNumber, q, q_t)`` 


A user function, which computes a force vector depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments /  return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs to which object belongs
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``q``\ 
     - | Vector \ :math:`\in \Rcal^n_c`\ 
     - | object coordinates (nodal displacement coordinates of rigid body and mesh nodes) in current configuration, without reference values
   * - | \ ``q_t``\ 
     - | Vector \ :math:`\in \Rcal^n_c`\ 
     - | object velocity coordinates (time derivative of \ ``q``\ ) in current configuration
   * - | \returnValue
     - | Vector \ :math:`\in \Rcal^{n_c}`\ 
     - | returns force vector for object


--------

\ **Userfunction**\ : ``massMatrixUserFunction(mbs, t, itemNumber, q, q_t)`` 


A user function, which computes a mass matrix depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments /  return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs to which object belongs
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``q``\ 
     - | Vector \ :math:`\in \Rcal^n_c`\ 
     - | object coordinates (nodal displacement coordinates of rigid body and mesh nodes) in current configuration, without reference values
   * - | \ ``q_t``\ 
     - | Vector \ :math:`\in \Rcal^n_c`\ 
     - | object velocity coordinates (time derivative of \ ``q``\ ) in current configuration
   * - | \returnValue
     - | NumpyMatrix \ :math:`\in \Rcal^{n_c \times n_c}`\ 
     - | returns mass matrix for object




Relevant Examples and TestModels with weblink:

    \ `objectFFRFTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFTest.py>`_\  (TestModels/), \ `objectFFRFTest2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFTest2.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


