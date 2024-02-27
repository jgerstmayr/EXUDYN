

.. _sec-item-objectffrfreducedorder:

ObjectFFRFreducedOrder
======================

This object is used to represent modally reduced flexible bodies using the \ :ref:`FFRF <FFRF>`\  and the \ :ref:`CMS <CMS>`\ . It can be used to model real-life mechanical systems imported from finite element codes or Python tools such as NETGEN/NGsolve, see the \ ``FEMinterface``\  in Section :ref:`sec-fem-feminterface---init--`\ . It contains a RigidBodyNode (always node 0) and a NodeGenericODE2 representing the modal coordinates. Currently, equations must be defined within user functions, which are available in the FEM module, see class \ ``ObjectFFRFreducedOrderInterface``\ , especially the user functions \ ``UFmassFFRFreducedOrder``\  and \ ``UFforceFFRFreducedOrder``\ , Section :ref:`sec-fem-objectffrfreducedorderinterface-addobjectffrfreducedorderwithuserfunctions`\ .

Authors: Gerstmayr Johannes, Zwölfer Andreas

\ **Additional information for ObjectFFRFreducedOrder**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Body``\ , \ ``MultiNoded``\ , \ ``SuperElement``\ 
* | Requested \ ``Node``\  type: read detailed information of item
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
  | A Python user function which computes the generalized user force vector for the \ :ref:`ODE2 <ODE2>`\  equations; see description below
* | **massMatrixUserFunction** [\ :math:`{\mathbf{M}}\induser \in \Rcal^{n_{ODE2}\times n_{ODE2}}`\ , type = PyFunctionMatrixMbsScalarIndex2Vector, default =  0]:
  | A Python user function which computes the TOTAL mass matrix (including reference node) and adds the local constant mass matrix; see description below
* | **computeFFRFterms** [type = Bool, default = True]:
  | flag decides whether the standard \ :ref:`FFRF <FFRF>`\ /\ :ref:`CMS <CMS>`\  terms are computed; use this flag for user-defined definition of \ :ref:`FFRF <FFRF>`\  terms in mass matrix and quadratic velocity vector
* | **modeBasis** [\ :math:`\LU{b}{\tPsi} \in \Rcal^{n\indf \times n_{m}}`\ , type = NumpyMatrix, default = Matrix[]]:
  | mode basis, which transforms reduced coordinates to (full) nodal coordinates, written as a single vector \ :math:`[u_{x,n_0},\,u_{y,n_0},\,u_{z,n_0},\,\ldots,\,u_{x,n_n},\,u_{y,n_n},\,u_{z,n_n}]\tp`\ 
* | **outputVariableModeBasis** [\ :math:`\LU{b}{\tPsi}_{OV} \in \Rcal^{n_n \times (n_{m}\cdot s_{OV})}`\ , type = NumpyMatrix, default = Matrix[]]:
  | mode basis, which transforms reduced coordinates to output variables per mode and per node; \ :math:`s_{OV}`\  is the size of the output variable, e.g., 6 for stress modes (\ :math:`S_{xx},...,S_{xy}`\ )
* | **outputVariableTypeModeBasis** [type = OutputVariableType, default = OutputVariableType::_None]:
  | this must be the output variable type of the outputVariableModeBasis, e.g. exu.OutputVariableType.Stress
* | **referencePositions** [\ :math:`\LU{b}{{\mathbf{x}}}\cRef \in \Rcal^{n\indf}`\ , type = NumpyVector, default = []]:
  | vector containing the reference positions of all flexible nodes, needed for graphics
* | **objectIsInitialized** [type = Bool, default = False]:
  | ALWAYS set to False! flag used to correctly initialize all \ :ref:`FFRF <FFRF>`\  matrices; as soon as this flag is False, some internal (constant) \ :ref:`FFRF <FFRF>`\  matrices are recomputed during Assemble()
* | **physicsMass** [\ :math:`m`\ , type = UReal, default = 0.]:
  | total mass [SI:kg] of FFRFreducedOrder object
* | **physicsInertia** [\ :math:`{\mathbf{J}}_r \in \Rcal^{3 \times 3}`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | inertia tensor [SI:kgm\ :math:`^2`\ ] of rigid body w.r.t. to the reference point of the body
* | **physicsCenterOfMass** [\ :math:`\LU{b}{{\mathbf{b}}}_{COM}`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | local position of center of mass (\ :ref:`COM <COM>`\ )
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
  | tilde matrix from local position of \ :ref:`COM <COM>`\ ; autocomputed during initialization
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


----------

.. _description-objectffrfreducedorder:

DESCRIPTION of ObjectFFRFreducedOrder
-------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Coordinates``\ : 
  | all \ :ref:`ODE2 <ODE2>`\  coordinates
* | ``Coordinates\_t``\ : 
  | all \ :ref:`ODE2 <ODE2>`\  velocity coordinates
* | ``Force``\ : 
  | generalized forces for all coordinates (residual of all forces except mass*accleration; corresponds to ComputeODE2LHS)



.. _sec-objectffrfreducedorder-superelementoutput:


Super element output variables
------------------------------

Functions like \ ``GetObjectOutputSuperElement(...)``\ , see Section :ref:`sec-mainsystem-object`\ , 
or \ ``SensorSuperElement``\ , see Section :ref:`sec-mainsystem-sensor`\ , directly access special output variables
(\ ``OutputVariableType``\ ) of the mesh nodes of the superelement.
Additionally, the contour drawing of the object can make use the \ ``OutputVariableType``\  of the meshnodes.

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | super element output variables
     - | symbol
     - | description
   * - | DisplacementLocal (mesh node \ :math:`i`\ )
     - | \ :math:`\LU{b}{{\mathbf{u}}\indf^{(i)}} = \left( \LU{b}{\tPsi} \tzeta\right)_{3\cdot i \ldots 3\cdot i+2}= \vr{\LU{b}{{\mathbf{q}}_{\mathrm{f},i\cdot 3}}}{\LU{b}{{\mathbf{q}}_{\mathrm{f},i\cdot 3+1}}}{\LU{b}{{\mathbf{q}}_{\mathrm{f},i\cdot 3+2}}}`\ 
     - | local nodal mesh displacement in reference (body) frame, measuring only flexible part of displacement
   * - | VelocityLocal (mesh node \ :math:`(i)`\ )
     - | \ :math:`\LU{b}{\dot {\mathbf{u}}_\mathrm{f}^{(i)}} = \left( \LU{b}{\tPsi} \dot \tzeta\right)_{3\cdot i \ldots 3\cdot i+2}`\ 
     - | local nodal mesh velocity in reference (body) frame, only for flexible part of displacement
   * - | Displacement (mesh node \ :math:`(i)`\ )
     - | \ :math:`\LU{0}{{\mathbf{u}}\cConfig^{(i)}} = \LU{0}{{\mathbf{q}}_{\mathrm{t,config}}} + \LU{0b}{{\mathbf{A}}_\mathrm{config}} \LU{b}{{\mathbf{p}}_\mathrm{f,config}^{(i)}} - (\LU{0}{{\mathbf{q}}_{\mathrm{t,ref}}} + \LU{0b}{{\mathbf{A}}_{ref}} \LU{b}{{\mathbf{x}}\cRef^{(i)}})`\ 
     - | nodal mesh displacement in global coordinates
   * - | Position (mesh node \ :math:`(i)`\ )
     - | \ :math:`\LU{0}{{\mathbf{p}}^{(i)}} = \LU{0}{\pRef} + \LU{0b}{{\mathbf{A}}} \LU{b}{{\mathbf{p}}\indf^{(i)}}`\ 
     - | nodal mesh position in global coordinates
   * - | Velocity (mesh node \ :math:`(i)`\ )
     - | \ :math:`\LU{0}{\dot {\mathbf{u}}^{(i)}} = \LU{0}{\dot {\mathbf{q}}\indt} + \LU{0b}{{\mathbf{A}}} (\LU{b}{\dot {\mathbf{u}}\indf^{(i)}} + \LU{b}{\tilde \tomega} \LU{b}{{\mathbf{p}}\indf^{(i)}})`\ 
     - | nodal mesh velocity in global coordinates
   * - | Acceleration (mesh node \ :math:`(i)`\ )
     - | \ :math:`\LU{0}{{\mathbf{a}}^{(i)}} = \LU{0}{\ddot {\mathbf{q}}\indt} + 
                                                    \LU{0b}{\Rot} \LU{b}{\ddot {\mathbf{u}}\indf^{(i)}} + 
                                                    2\LU{0}{\tomega} \times \LU{0b}{\Rot} \LU{b}{\dot {\mathbf{u}}\indf^{(i)}} +
                                                    \LU{0}{\talpha} \times \LU{0}{{\mathbf{p}}\indf^{(i)}} + 
                                                    \LU{0}{\tomega} \times (\LU{0}{\tomega} \times \LU{0}{{\mathbf{p}}\indf^{(i)}})`\ 
     - | global acceleration of mesh 
                                                    node \ :math:`n_i`\  including rigid body motion and flexible deformation; note that \ :math:`\LU{0}{{\mathbf{x}}}(n_i) = \LU{0b}{\Rot} \LU{b}{{\mathbf{x}}}(n_i)`\ 
   * - | StressLocal (mesh node \ :math:`(i)`\ )
     - | \ :math:`\LU{b}{\tsigma^{(i)}} = (\LU{b}{\tPsi_{OV}} \tzeta)_{3\cdot i \ldots 3\cdot i+5}`\ 
     - | linearized stress components of mesh node \ :math:`(i)`\  in reference frame; \ :math:`\tsigma=[\sigma_{xx},\,\sigma_{yy},\,\sigma_{zz},\,\sigma_{yz},\,\sigma_{xz},\,\sigma_{xy}]\tp`\ ; ONLY available, if \ :math:`\LU{b}{\tPsi}_{OV}`\  is provided and \ ``outputVariableTypeModeBasis== exu.OutputVariableType.StressLocal``\ 
   * - | StrainLocal (mesh node \ :math:`(i)`\ )
     - | \ :math:`\LU{b}{\teps^{(i)}} = (\LU{b}{\tPsi}_{OV} \tzeta)_{3\cdot i \ldots 3\cdot i+5}`\ 
     - | linearized strain components of mesh node \ :math:`(i)`\  in reference frame; \ :math:`\teps=[\varepsilon_{xx},\,\varepsilon_{yy},\,\varepsilon_{zz},\,\varepsilon_{yz},\,\varepsilon_{xz},\,\varepsilon_{xy}]\tp`\ ; ONLY available, if \ :math:`\LU{b}{\tPsi}_{OV}`\  is provided and \ ``outputVariableTypeModeBasis== exu.OutputVariableType.StrainLocal``\ 


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | reference frame
     - | \ :math:`b`\ 
     - | the body-fixed / local frame is always denoted by \ :math:`b`\ 
   * - | number of rigid body coordinates
     - | \ :math:`n\indrigid`\ 
     - | number of rigid body node coordinates: 6 in case of Euler angles (not fully available for ObjectFFRFreducedOrder) and 7 in case of Euler parameters
   * - | number of flexible / mesh coordinates
     - | \ :math:`n\indf = 3 \cdot n_n`\ 
     - | with number of nodes \ :math:`n_n`\ ; relevant for visualization
   * - | number of modal coordinates
     - | \ :math:`n_m \ll n\indf`\ 
     - | the number of reduced or modal coordinates, computed from number of columns given in \ ``modeBasis``\ 
   * - | total number object coordinates
     - | \ :math:`n_{ODE2} = n_m + n_{rigid}`\ 
     - | 
   * - | reference frame origin
     - | \ :math:`\LU{0}{\pRef} = \LU{0}{{\mathbf{q}}_{\mathrm{t}}} + \LU{0}{{\mathbf{q}}_{\mathrm{t,ref}}}`\ 
     - | reference frame position (origin)
   * - | reference frame rotation
     - | \ :math:`\ttheta\cConfig = \ttheta\cConfig + \ttheta_{ref}`\ 
     - | reference frame rotation parameters in any configuration except reference
   * - | reference frame orientation
     - | \ :math:`\LU{0b}{\Rot}\cConfig = \LU{0b}{\Rot}\cConfig(\ttheta\cConfig)`\ 
     - | transformation matrix for transformation of local (reference frame) to global coordinates, given by underlying rigid body node \ :math:`n_0`\ 
   * - | local vector of flexible coordinates
     - | \ :math:`\LU{b}{{\mathbf{q}}\indf} = \LU{b}{\tPsi} \tzeta`\ 
     - | represents mesh displacements; vector of alternating x,y, an z coordinates of local (in body frame) mesh displacements reconstructed from modal coordinates \ :math:`\tzeta`\ ; only evaluated for selected node points (e.g., sensors) during computation; corresponds to same vector in \ ``ObjectFFRF``\ 
   * - | local nodal positions
     - | \ :math:`\LU{b}{{\mathbf{p}}\indf} = \LU{b}{{\mathbf{q}}\indf} + \LU{b}{{\mathbf{x}}\cRef}`\ 
     - | vector of all body-fixed nodal positions including flexible part; only evaluated for selected node points during computation
   * - | local position of node (i)
     - | \ :math:`\LU{b}{{\mathbf{p}}\indf^{(i)}} = \LU{b}{{\mathbf{u}}\indf^{(i)}} + \LU{b}{{\mathbf{x}}^{(i)}\cRef} = \vr{\LU{b}{{\mathbf{q}}_{\mathrm{f},i\cdot 3}}}{\LU{b}{{\mathbf{q}}_{\mathrm{f},i\cdot 3+1}}}{\LU{b}{{\mathbf{q}}_{\mathrm{f},i\cdot 3+2}}} + \vr{\LU{b}{{\mathbf{x}}_{\mathrm{ref},i\cdot 3}}}{\LU{b}{{\mathbf{x}}_{\mathrm{ref},i\cdot 3+1}}}{\LU{b}{{\mathbf{x}}_{\mathrm{ref},i\cdot 3+2}}}`\ 
     - | body-fixed, deformed nodal mesh position (including flexible part)
   * - | vector of modal coordinates
     - | \ :math:`\tzeta = [\zeta_0,\,\ldots,\zeta_{n_m-1}]\tp`\ 
     - | vector of modal or reduced coordinates; these coordinates can either represent amplitudes of eigenmodes, static modes or general modes, depending on your mode basis
   * - | coordinate vector
     - | \ :math:`{\mathbf{q}} = [\LU{0}{{\mathbf{q}}\indt},\,\tpsi,\,\tzeta]`\ 
     - | vector of object coordinates; \ :math:`{\mathbf{q}}\indt`\  and \ :math:`\tpsi`\  are the translation and rotation part of displacements of the reference frame, provided by the rigid body node (node number 0)
   * - | flexible coordinates transformation matrix
     - | \ :math:`\LU{0b}{{\mathbf{A}}_{bd}} = \mathrm{diag}([\LU{0b}{{\mathbf{A}}},\;\ldots,\;\LU{0b}{{\mathbf{A}}}])`\ 
     - | block diagonal transformation matrix, which transforms all flexible coordinates from local to global coordinates


Modal reduction and reduced inertia matrices
--------------------------------------------

The formulation is based on the EOM of \ ``ObjectFFRF``\ , \ **also regarding parts of notation**\  
and some input parameters, Section :ref:`sec-item-objectffrf`\ , and 
can be found in Zwölfer and Gerstmayr  with only small modifications in the notation.
The notation of kinematics quantities follows the floating frame of reference idea with
quantities given in the tables above and sketched in \ :numref:`fig-objectffrfreducedorder-mesh`\ .


.. _fig-objectffrfreducedorder-mesh:
.. figure:: ../../theDoc/figures/ObjectFFRFsketch.png
   :width: 400

   Floating frame of reference with exemplary position of a mesh node *i* 


                   
The reduced order \ :ref:`FFRF <FFRF>`\  formulation is based on an approximation of flexible coordinates \ :math:`\LU{b}{{\mathbf{q}}\indf}`\  
by means of a reduction or mode basis \ :math:`\LU{b}{\tPsi}`\  (\ ``modeBasis``\ ) and the the modal coordinates \ :math:`\tzeta`\ ,

.. math::

   \LU{b}{{\mathbf{q}}\indf} \approx \LU{b}{\tPsi} \tzeta


The mode basis \ :math:`\LU{b}{\tPsi}`\  contains so-called mode shape vectors in its columns, which may be computed from eigen analysis, static computation or more advanced techniques, 
see the helper functions in module \ ``exudyn.FEM``\ , within the class \text{FEMinterface}.
To compute eigen modes, use \ ``FEMinterface.ComputeEigenmodes(...)``\  or
\ ``FEMinterface.ComputeHurtyCraigBamptonModes(...)``\ . For details on model order reduction and component mode synthesis, see Section :ref:`sec-theory-cms`\ .
In many applications, \ :math:`n_m`\  typically ranges between 10 and 50, but also beyond -- depending on the desired accuracy of the model.

The \ ``ObjectFFRF``\  coordinates and Eqs. :eq:`eq-objectffrf-eom`\ \ (this is not done for user functions and \ ``forceVector``\ ) can be reduced by the matrix \ :math:`{\mathbf{H}} \in \Rcal^{(n\indf+n\indrigid) \times n_{ODE2}}`\ ,

.. math::

   {\mathbf{q}}_{FFRF} = \vr{{\mathbf{q}}\indt}{\ttheta}{\LU{b}{{\mathbf{q}}\indf}} = \mr{\mathbf{I}_{3 \times 3}}{\Null}{\Null} {\Null}{{\mathbf{I}}\indr}{\Null} {\Null}{\Null}{\LU{b}{\tPsi}} \vr{{\mathbf{q}}\indt}{\ttheta}{\tzeta} = {\mathbf{H}} \, {\mathbf{q}}


with the \ :math:`4\times 4`\  identity matrix \ :math:`{\mathbf{I}}\indr`\  in case of Euler parameters and the reduced coordinates \ :math:`{\mathbf{q}}`\ .

The reduced equations follow from the reduction of system matrices in Eqs. :eq:`eq-objectffrf-eom`\ ,

.. math::

   {\mathbf{K}}\indred &=& \LU{b}{\tPsi}\tp \LU{b}{{\mathbf{K}}} \LU{b}{\tPsi} , \\
   {\mathbf{M}}\indred &=& \LU{b}{\tPsi}\tp \LU{b}{{\mathbf{M}}} \LU{b}{\tPsi} , \\



the computation of rigid body inertia

.. math::

   \LU{b}{\tTheta}\indu &=& \LUX{b}{\tilde {\mathbf{x}}}{\cRef\tp} \LU{b}{{\mathbf{M}}} \LU{b}{\tilde {\mathbf{x}}\cRef}\\



the center of mass (and according tilde matrix), using \ :math:`\tPhi\indt`\  from Eq. :eq:`eq-objectffrf-phit`\ ,

.. math::

   \LU{b}{\tchi}\indu &=& \frac{1}{m} \tPhi\tp\indt \LU{b}{{\mathbf{M}}} \LU{b}{{\mathbf{x}}\cRef}\\
   \LU{b}{\tilde \tchi\indu} &=& \frac{1}{m} \tPhi\tp\indt \LU{b}{{\mathbf{M}}} \LU{b}{\tilde {\mathbf{x}}\cRef}\\


 
and seven inertia-like matrices ,

.. math::

   {\mathbf{M}}_{AB} = {\mathbf{A}}\tp \LU{b}{{\mathbf{M}}} {\mathbf{B}}, \quad \mathrm{using} \quad {\mathbf{A}}{\mathbf{B}} \in \left[\tPsi\tPsi ,\; \widetilde{\tPsi}\tPsi,\; \widetilde{\tPsi}\widetilde{\tPsi},\; \tPhi\indt\tPsi,\; \tPhi\indt\widetilde{\tPsi},\; \tilde{\mathbf{x}}\cRef\tPsi,\; \tilde{\mathbf{x}}\cRef\widetilde{\tPsi}\right]


Note that the special tilde operator for vectors \ :math:`{\mathbf{p}} \in \Rcal^{n_f}`\  of Eq. :eq:`eq-objectffrf-specialtilde`\  is frequently used.



Equations of motion
-------------------

Equations of motion, in case that \ ``computeFFRFterms = True``\ :

.. math::

   \left({\mathbf{M}}_{user}(mbs, t,{\mathbf{q}},\dot {\mathbf{q}}) + \mr{{\mathbf{M}}\indtt}{{\mathbf{M}}\indtr}{{\mathbf{M}}\indtf} {}{{\mathbf{M}}\indrr}{{\mathbf{M}}\indrf} {\mathrm{sym.}}{}{{\mathbf{M}}\indff} \right) \ddot {\mathbf{q}} + \mr{0}{0}{0} {0}{0}{0} {0}{0}{{\mathbf{D}}\indff} \dot {\mathbf{q}} + \mr{0}{0}{0} {0}{0}{0} {0}{0}{{\mathbf{K}}\indff} {\mathbf{q}} = &&\\
   \nonumber {\mathbf{f}}_v({\mathbf{q}},\dot {\mathbf{q}}) + {\mathbf{f}}_{user}(mbs, t,{\mathbf{q}},\dot {\mathbf{q}}) &&


\ (NOTE that currently the internal (C++) computed terms are zero,

.. math::

   \mr{{\mathbf{M}}\indtt}{{\mathbf{M}}\indtr}{{\mathbf{M}}\indtf} {}{{\mathbf{M}}\indrr}{{\mathbf{M}}\indrf} {\mathrm{sym.}}{}{{\mathbf{M}}\indff} = \Null \quad \mathrm{and} \quad {\mathbf{f}}_v({\mathbf{q}},\dot {\mathbf{q}}) = \Null ,


but they are implemented in predefined user functions, see \ ``FEM.py``\ , Section :ref:`sec-fem-objectffrfreducedorderinterface-addobjectffrfreducedorderwithuserfunctions`\ . In near future, these terms will be implemented in C++ and replace the user functions.)
Note that in case of Euler parameters for the parameterization of rotations for the reference frame, the Euler parameter constraint equation is added automatically by this object.
The single terms of the mass matrix are defined as

.. math::

   {\mathbf{M}}\indtt &=& m \mathbf{I}_{3 \times 3} \\
   {\mathbf{M}}\indtr &=& -\LU{0b}{\Rot} \left[ m \LU{b}{\tilde \tchi\indu} + {\mathbf{M}}_{\Phi\indt\!{\widetilde\Psi}} \left( \tzeta \otimes {\mathbf{I}} \right)  \right] \LU{b}{{\mathbf{G}}}\\
   {\mathbf{M}}\indtf &=& \LU{0b}{\Rot} {\mathbf{M}}_{\Phi\indt\!\Psi} \\
   {\mathbf{M}}\indrr &=& \LU{b}{{\mathbf{G}}\tp} \left[\LU{b}{\tTheta}\indu + {\mathbf{M}}_{\tilde {\mathbf{x}}\cRef{\widetilde\Psi}} \left( \tzeta \otimes {\mathbf{I}} \right) + \left( \tzeta \otimes {\mathbf{I}} \right)\tp {\mathbf{M}}_{\tilde {\mathbf{x}}\cRef{\widetilde\Psi}}\tp + \left( \tzeta \otimes {\mathbf{I}} \right)\tp {\mathbf{M}}_{{\widetilde\Psi}{\widetilde\Psi}}\left( \tzeta \otimes {\mathbf{I}} \right) \right] \LU{b}{{\mathbf{G}}}\\
   {\mathbf{M}}\indrf &=& -\LU{b}{{\mathbf{G}}\tp} \left[ {\mathbf{M}}_{\tilde {\mathbf{x}}\cRef\Psi} + \left( \tzeta \otimes {\mathbf{I}} \right)\tp {\mathbf{M}}_{{\widetilde\Psi}\Psi}  \right] \\
   {\mathbf{M}}\indff &=& {\mathbf{M}}_{\Psi\Psi}


with the Kronecker product\ (In Python numpy module this is computed by \ ``numpy.kron(zeta, Im).T``\ ),

.. math::

   \tzeta \otimes {\mathbf{I}} = \vr{\zeta_0 {\mathbf{I}}}{\vdots}{\zeta_{m-1} {\mathbf{I}}}


The quadratic velocity vector \ :math:`{\mathbf{f}}_v({\mathbf{q}},\dot {\mathbf{q}}) = \left[ {\mathbf{f}}_{v\mathrm{t}}\tp,\; {\mathbf{f}}_{v\mathrm{r}}\tp,\; {\mathbf{f}}_{v\mathrm{f}}\tp \right]\tp`\  reads

.. math::

   {\mathbf{f}}_{v\mathrm{t}} &=& \LU{0b}{\Rot} \LU{b}{\tilde \tomega}\left[ m \LU{b}{\tilde \tchi\indu} + {\mathbf{M}}_{\Phi\indt\!{\widetilde\Psi}} \left( \tzeta \otimes {\mathbf{I}} \right)  \right] \LU{b}{\tomega} + 2 \LU{0b}{\Rot} {\mathbf{M}}_{\Phi\indt\!{\widetilde\Psi}} \left( \dot \tzeta \otimes {\mathbf{I}} \right)  \LU{b}{\tomega} \nonumber \\
   && + \LU{0b}{\Rot} \left[ m \LU{b}{\tilde \tchi\indu} + {\mathbf{M}}_{\Phi\indt\!{\widetilde\Psi}} \left( \tzeta \otimes {\mathbf{I}} \right)  \right] \LU{b}{\dot {\mathbf{G}}} \dot \ttheta , \\
   {\mathbf{f}}_{v\mathrm{r}} &=& -\LU{b}{{\mathbf{G}}\tp} \LU{b}{\tilde \tomega} \left[\LU{b}{\tTheta}\indu + {\mathbf{M}}_{\tilde {\mathbf{x}}\cRef{\widetilde\Psi}} \left( \tzeta \otimes {\mathbf{I}} \right) + \left( \tzeta \otimes {\mathbf{I}} \right)\tp {\mathbf{M}}_{\tilde {\mathbf{x}}\cRef{\widetilde\Psi}}\tp + \left( \tzeta \otimes {\mathbf{I}} \right)\tp {\mathbf{M}}_{{\widetilde\Psi}{\widetilde\Psi}}\left( \tzeta \otimes {\mathbf{I}} \right) \right]\LU{b}{\tomega} \nonumber \\
   && -2 \LU{b}{{\mathbf{G}}\tp} \left[ {\mathbf{M}}_{\tilde {\mathbf{x}}\cRef{\widetilde\Psi}} \left( \dot \tzeta \otimes {\mathbf{I}} \right) + \left( \tzeta \otimes {\mathbf{I}} \right)\tp {\mathbf{M}}_{{\widetilde\Psi}{\widetilde\Psi}}\left( \dot \tzeta \otimes {\mathbf{I}} \right) \right] \LU{b}{\tomega} \nonumber \\
   && -\LU{b}{{\mathbf{G}}\tp}\left[\LU{b}{\tTheta}\indu + {\mathbf{M}}_{\tilde {\mathbf{x}}\cRef{\widetilde\Psi}} \left( \tzeta \otimes {\mathbf{I}} \right) + \left( \tzeta \otimes {\mathbf{I}} \right)\tp {\mathbf{M}}_{\tilde {\mathbf{x}}\cRef{\widetilde\Psi}}\tp + \left( \tzeta \otimes {\mathbf{I}} \right)\tp {\mathbf{M}}_{{\widetilde\Psi}{\widetilde\Psi}}\left( \tzeta \otimes {\mathbf{I}} \right) \right] \LU{b}{\dot {\mathbf{G}}} \dot \ttheta , \\
   {\mathbf{f}}_{v\mathrm{f}} &=& \left( {\mathbf{I}}_\zeta \otimes \LU{b}{\tomega} \right)\tp \left[ {\mathbf{M}}_{\tilde{\mathbf{x}}\cRef{\widetilde\Psi}}\tp + {\mathbf{M}}_{{\widetilde\Psi}{\widetilde\Psi}}\left( \tzeta \otimes {\mathbf{I}} \right) \right] \LU{b}{\tomega} +2 {\mathbf{M}}_{{\widetilde\Psi}{\Psi}}\tp\left( \dot\tzeta \otimes {\mathbf{I}} \right) \LU{b}{\tomega} \nonumber \\
   && + \left[ {\mathbf{M}}_{\tilde{\mathbf{x}}\cRef{\Psi}}\tp + {\mathbf{M}}_{{\widetilde\Psi}{\Psi}}\tp\left( \tzeta \otimes {\mathbf{I}} \right) \right] \LU{b}{\dot {\mathbf{G}}} \dot \ttheta .


Note that terms including \ :math:`\LU{b}{\dot {\mathbf{G}}} \dot \ttheta`\  vanish in case of Euler parameters or in case that \ :math:`\LU{b}{\dot {\mathbf{G}}} = \Null`\ ,
and we use another Kronecker product with the unit matrix \ :math:`{\mathbf{I}}_\zeta \in \Rcal^{n_m \times n_m}`\ ,

.. math::

   {\mathbf{I}}_\zeta \otimes \LU{b}{\tomega} = \mr{\LU{b}{\tomega}}{}{} {}{\ddots}{} {}{}{\LU{b}{\tomega}} \in \Rcal^{3n_m \times n_m}




In case that \ ``computeFFRFterms = False``\ , the mass terms \ :math:`{\mathbf{M}}\indtt \ldots {\mathbf{M}}\indff`\  are zero (not computed) and
the quadratic velocity vector \ :math:`{\mathbf{f}}_Q = \Null`\ .
Note that the user functions \ :math:`{\mathbf{f}}_{user}(mbs, t,{\mathbf{q}},\dot {\mathbf{q}})`\  and 
\ :math:`{\mathbf{M}}_{user}(mbs, t,{\mathbf{q}},\dot {\mathbf{q}})`\  may be empty (=0). 
The detailed equations of motion for this element can be found in .


Position Jacobian
-----------------

For joints and loads, the position jacobian of a node is needed in order to compute forces applied to averaged displacements and 
rotations at nodes.
Recall that the modal coordinates \ :math:`\tzeta`\  are transformed to node coordinates by means of the mode basis  \ :math:`\LU{b}{\tPsi}`\ ,

.. math::

   \LU{b}{{\mathbf{q}}\indf} = \LU{b}{\tPsi} \tzeta .


The local displacements \ :math:`\LU{b}{{\mathbf{u}}\indf^{(i)}}`\  of a specific node \ :math:`i`\  can be reconstructed in this way by means of

.. math::

   \LU{b}{{\mathbf{u}}\indf^{(i)}} = \vr{\LU{b}{{\mathbf{q}}_{\mathrm{f},i\cdot 3}}}{\LU{b}{{\mathbf{q}}_{\mathrm{f},i\cdot 3+1}}}{\LU{b}{{\mathbf{q}}_{\mathrm{f},i\cdot 3+2}}} ,


and the global position of a node, see tables above, reads

.. math::

   \LU{0}{{\mathbf{p}}^{(i)}} = \LU{0}{{\mathbf{p}}\indt} + \LU{0b}{{\mathbf{A}}} \left( \LU{b}{{\mathbf{u}}\indf^{(i)}} + \LU{b}{{\mathbf{x}}^{(i)}\cRef} \right)


Thus, the jacobian of the global position reads

.. math::

   \LU{0}{{\mathbf{J}}_\mathrm{pos}^{(i)}} = \frac{\partial \LU{0}{{\mathbf{p}}^{(i)}}}{\partial [{\mathbf{q}}\indt, \;\ttheta, \;\tzeta]} = \left[\mathbf{I}_{3 \times 3}, \; -\LU{0b}{\Rot} \left(\LU{b}{\tilde{\mathbf{u}}\indf^{(i)}} + \LU{b}{\tilde{\mathbf{x}}^{(i)}\cRef} \right) \LU{b}{{\mathbf{G}}},\; \LU{0b}{\Rot} \vr{\LU{b}{\tPsi_{r=3i}\tp}}{\LU{b}{\tPsi_{r=3i+1}\tp}}{\LU{b}{\tPsi_{r=3i+2}\tp}}\right] ,


in which \ :math:`\LU{b}{\tPsi_{r=...}}`\  represents the row \ :math:`r`\  of the mode basis (matrix) \ :math:`\LU{b}{\Psi}`\ , and
the matrix 

.. math::

   \vr{\LU{b}{\tPsi_{r=3i}\tp}}{\LU{b}{\tPsi_{r=3i+1}\tp}}{\LU{b}{\tPsi_{r=3i+2}\tp}} \in \Rcal^{3 \times n_m}


Furthermore, the jacobian of the local position reads

.. math::

   \LU{b}{{\mathbf{J}}_\mathrm{pos}^{(i)}} = \frac{\partial \LU{b}{{\mathbf{p}}\indf^{(i)}}}{\partial [{\mathbf{q}}\indt, \;\ttheta, \;\tzeta]} = \left[\Null, \; \Null, \; \vr{\LU{b}{\tPsi_{r=3i}\tp}}{\LU{b}{\tPsi_{r=3i+1}\tp}}{\LU{b}{\tPsi_{r=3i+2}\tp}}\right] ,


which is used in \ ``MarkerSuperElementRigid``\ .



Joints and Loads
----------------

Use special \ ``MarkerSuperElementPosition``\  to apply forces, SpringDampers or spherical joints. This marker can be attached to a single node of the underlying
mesh or to a set of nodes, which is then averaged, see the according marker description.

Use special \ ``MarkerSuperElementRigid``\  to apply torques or special joints (e.g., \ ``JointGeneric``\ ). 
This marker must be attached to a set of nodes which can represent rigid body motion. The rigid body motion is then averaged for all of these nodes,
see the according marker description.

For application of mass proportional loads (gravity), you can use conventional MarkerBodyMass.
However, \ **do not use**\  \ ``MarkerBodyPosition``\  or \ ``MarkerBodyRigid``\  for ObjectFFRFreducedOrder, unless wanted, because it only attaches to the floating
frame. This means, that a force to a \ ``MarkerBodyPosition``\  would only be applied to the (rigid) floating frame, but not onto the deformable body and
results depend strongly on the choice of the reference frame (or the underlying mode shapes).

CoordinateLoads are added for each \ :ref:`ODE2 <ODE2>`\  coordinate on the RHS of the equations of motion. 



--------

\ **Userfunction**\ : ``forceUserFunction(mbs, t, itemNumber, q, q_t)`` 


A user function, which computes a force vector depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.
Note that itemNumber represents the index of the ObjectFFRFreducedOrder object in mbs, which can be used to retrieve additional data from the object through
\ ``mbs.GetObjectParameter(itemNumber, ...)``\ , see the according description of \ ``GetObjectParameter``\ .

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
     - | Vector \ :math:`\in \Rcal^n_{ODE2}`\ 
     - | \ :ref:`FFRF <FFRF>`\  object coordinates (rigid body coordinates and reduced coordinates in a list) in current configuration, without reference values
   * - | \ ``q_t``\ 
     - | Vector \ :math:`\in \Rcal^n_{ODE2}`\ 
     - | object velocity coordinates (time derivatives of \ ``q``\ ) in current configuration
   * - | \returnValue
     - | Vector \ :math:`\in \Rcal^{n_{ODE2}}`\ 
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
     - | Vector \ :math:`\in \Rcal^n_{ODE2}`\ 
     - | \ :ref:`FFRF <FFRF>`\  object coordinates (rigid body coordinates and reduced coordinates in a list) in current configuration, without reference values
   * - | \ ``q_t``\ 
     - | Vector \ :math:`\in \Rcal^n_{ODE2}`\ 
     - | object velocity coordinates (time derivatives of \ ``q``\ ) in current configuration
   * - | \returnValue
     - | NumpyMatrix \ :math:`\in \Rcal^{n_{ODE2} \times n_{ODE2}}`\ 
     - | returns mass matrix for object




Relevant Examples and TestModels with weblink:

    \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Examples/), \ `objectFFRFreducedOrderNetgen.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/objectFFRFreducedOrderNetgen.py>`_\  (Examples/), \ `NGsolveCrankShaftTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/NGsolveCrankShaftTest.py>`_\  (TestModels/), \ `objectFFRFreducedOrderAccelerations.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderAccelerations.py>`_\  (TestModels/), \ `objectFFRFreducedOrderShowModes.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderShowModes.py>`_\  (TestModels/), \ `objectFFRFreducedOrderStressModesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderStressModesTest.py>`_\  (TestModels/), \ `superElementRigidJointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/superElementRigidJointTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


