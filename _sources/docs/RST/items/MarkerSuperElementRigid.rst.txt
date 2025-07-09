

.. _sec-item-markersuperelementrigid:

MarkerSuperElementRigid
=======================

A position and orientation (rigid-body) marker attached to a SuperElement, such as ObjectFFRF, ObjectGenericODE2 and ObjectFFRFreducedOrder (for which it may be inefficient). The marker acts on the mesh nodes, not on the underlying nodes of the object. Note that in contrast to the MarkerSuperElementPosition, this marker needs a set of interface nodes which are not aligned at one line, such that these node points can represent a rigid body motion. Note that definitions of marker positions are slightly different from MarkerSuperElementPosition.

\ **Additional information for MarkerSuperElementRigid**\ :

* | This \ ``Marker``\  has/provides the following types = \ ``Object``\ , \ ``Body``\ , \ ``Position``\ , \ ``Orientation``\ 


The item \ **MarkerSuperElementRigid**\  with type = 'SuperElementRigid' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **bodyNumber** [\ :math:`n_b`\ , type = ObjectIndex, default = invalid (-1)]:
  | body number to which marker is attached to
* | **offset** [\ :math:`\LU{r}{{\mathbf{o}}_{ref}}`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | local marker SuperElement reference position offset used to correct the center point of the marker, which is computed from the weighted average of reference node positions (which may have some offset to the desired joint position). Note that this offset shall be small and larger offsets can cause instability in simulation models (better to have symmetric meshes at joints).
* | **meshNodeNumbers** [\ :math:`[k_0,\,\ldots,\,k_{n_m-1}]\tp`\ , type = ArrayIndex, default = []]:
  | a list of \ :math:`n_m`\  mesh node numbers of superelement (=interface nodes) which are used to compute the body-fixed marker position and orientation; the related nodes must provide 3D position information, such as NodePoint, NodePoint2D, NodeRigidBody[..]; in order to retrieve the global node number, the generic body needs to convert local into global node numbers
* | **weightingFactors** [\ :math:`[w_{0},\,\ldots,\,w_{n_m-1}]\tp`\ , type = Vector, default = []]:
  | a list of \ :math:`n_m`\  weighting factors per node to compute the final local position and orientation; these factors could be based on surface integrals of the constrained mesh faces
* | **useAlternativeApproach** [type = Bool, default = True]:
  | this flag switches between two versions for the computation of the rotation and angular velocity of the marker; alternative approach uses skew symmetric matrix of reference position; follows the inertia concept
* | **rotationsExponentialMap** [type = Index, default = 2]:
  | Experimental flag (2 is the correct value and will be used in future, removing this flag): This value switches different behavior for computation of rotations and angular velocities: 0 uses linearized rotations and angular velocities, 1 uses the exponential map for rotations but linear angular velocities, 2 uses the exponential map for rotations and the according tangent map for angular velocities
* | **visualization** [type = VMarkerSuperElementRigid]:
  | parameters for visualization of item



The item VMarkerSuperElementRigid has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **showMarkerNodes** [type = Bool, default = True]:
  | set true, if all nodes are shown (similar to marker, but with less intensity)


----------

.. _description-markersuperelementrigid:

DESCRIPTION of MarkerSuperElementRigid
--------------------------------------
\ **Definition of marker quantities**\ :

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | number of mesh nodes
     - | \ :math:`n_m`\ 
     - | size of \ ``meshNodeNumbers``\  and \ ``weightingFactors``\  which are marked; this must not be the number of mesh nodes in the marked object
   * - | mesh node number
     - | \ :math:`i = k_i`\ 
     - | abbreviation, runs over all marker mesh nodes
   * - | mesh node local displacement
     - | \ :math:`\LU{r}{{\mathbf{u}}^{(i)}}`\ 
     - | current local (within reference frame \ :math:`r`\ ) displacement of mesh node \ :math:`k_i`\  in object \ :math:`n_b`\ 
   * - | mesh node local position
     - | \ :math:`\LU{r}{{\mathbf{p}}^{(i)}} = \LU{r}{{\mathbf{x}}^{(i)}\cRef} + \LU{r}{{\mathbf{u}}^{(i)}}`\ 
     - | current local (within reference frame \ :math:`r`\ , which is the body frame \ :math:`b`\  ,e.g., in \ ``ObjectFFRFreducedOrder``\ ) position of mesh node \ :math:`k_i`\  in object \ :math:`n_b`\ 
   * - | mesh node local reference position
     - | \ :math:`\LU{r}{{\mathbf{x}}^{(i)}\cRef}`\ 
     - | local (within reference frame \ :math:`r`\ ) reference position of mesh node \ :math:`k_i`\  in object \ :math:`n_b`\ , see e.g.\ \ ``ObjectFFRFreducedOrder``\ 
   * - | averaged local reference position
     - | \ :math:`\LU{r}{{\mathbf{x}}^\mathrm{avg}\cRef} = \sum_i w_i \LU{r}{{\mathbf{x}}^{(i)}\cRef}`\ 
     - | midpoint reference position of marker; averaged local reference positions of all mesh nodes \ :math:`k_i`\ , 
          using weighting for averaging; may not coincide with center point of your idealized joint surface (e.g., midpoint of cylinder), see \ :numref:`fig-markersuperelementrigid-sketch`\ 
   * - | marker centered mesh node local reference position
     - | \ :math:`\LU{r}{{\mathbf{p}}^{(i)}\cRef} = \LU{r}{{\mathbf{x}}^{(i)}\cRef}- \LU{r}{{\mathbf{x}}^\mathrm{avg}\cRef}`\ 
     - | local reference position of mesh node \ :math:`k_i`\  relative to the center position of marker
   * - | mesh node local velocity
     - | \ :math:`\LU{r}{{\mathbf{v}}^{(i)}}`\ 
     - | current local (within reference frame \ :math:`r`\ ) velocity of mesh node \ :math:`k_i`\  in object \ :math:`n_b`\ 
   * - | super element reference point
     - | \ :math:`\LU{0}{{\mathbf{p}}}_r`\  (\ :math:`=\LU{0}{{\mathbf{p}}}\indt`\  in \ ``ObjectFFRFreduced- Order``\ )
     - | current position (origin) of super element's floating frame (r), which is zero, if the object does not provide a reference frame (such as GenericODE2)
   * - | super element rotation matrix
     - | \ :math:`\LU{0r}{\Rot}`\ 
     - | current rigid body transformation matrix of super element's floating frame (r), which is the identity matrix, if the object does not provide a reference frame (such as GenericODE2)
   * - | super element angular velocity
     - | \ :math:`\LU{r}{\tomega_r}`\ 
     - | current local angular velocity of super element's floating frame (r), which is zero, if the object does not provide a reference frame (such as GenericODE2)

   * - | marker position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m} \!=\! \LU{0}{{\mathbf{p}}}_r + \LU{0r}{\Rot} \left(\LU{r}{{\mathbf{o}}\cRef}\! +\! \sum_i w_i \cdot \LU{r}{{\mathbf{p}}^{(i)}} \right)`\ 
     - | 
          current global position which is provided by marker; note offset \ :math:`\LU{r}{{\mathbf{o}}\cRef}`\  added, if used as a correction of marker mesh nodes
   * - | marker velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m} = \LU{0}{\dot {\mathbf{p}}}_r`\  \ :math:`+ \LU{0r}{\Rot} \left( \LU{r}{\tilde \tomega_r} \left(\LU{r}{{\mathbf{o}}\cRef}\! +\! \sum_i w_i \cdot \LU{r}{{\mathbf{p}}^{(i)}} \right) + \right.`\ 
                                              \ :math:`\left. \sum_i (w_i \cdot \LU{r}{\dot {\mathbf{u}}^{(i)}}) \right)`\ 
     - | current global velocity which is provided by marker
   * - | marker rotation matrix
     - | \ :math:`\LU{0r}{\Rot}_{m} = \LU{0r}{\Rot} \cdot \mathbf{exp}(\LU{r}{\ttheta}_{m})`\ 
     - | current rotation matrix, which transforms the local marker coordinates and adds the rigid body transformation of floating frames \ :math:`\LU{0r}{\Rot}`\ ; uses exponential map for SO3, assumes that \ :math:`\ttheta`\  represents a rotation vector
   * - | marker local rotation
     - | \ :math:`\LU{r}{\ttheta}_{m}`\ 
     - | current local linearized rotations (rotation vector); for the computation, see below for the standard and alternative approach
   * - | marker local angular velocity
     - | \ :math:`\LU{r}{\tomega}_{m}`\ 
     - | local angular velocity due to mesh node velocity only; for the computation, see below for the standard and alternative approach
   * - | marker global angular velocity
     - | \ :math:`\LU{0}{\tomega}_{m} = \LU{0}{\tomega_{r}} + \LU{0r}{\Rot} \LU{r}{\tomega}_{m}`\ 
     - | current global angular velocity



Marker background
-----------------

The marker allows to realize a multi-point constraint (assuming that the marker is used in a joint constraint), 
connecting to averaged nodal displacements and rotations (also known as RBE3 in NASTRAN), see e.g.\ . 
However, using Craig-Bampton RBE2 modes, will create RBE2 multi-point constraints for \ ``ObjectFFRFreducedOrder``\  objects.

For more information on the various quantities and their coordinate systems, see table above and \ :numref:`fig-markersuperelementrigid-sketch`\ .


.. _fig-markersuperelementrigid-sketch:
.. figure:: ../../theDoc/figures/MarkerSuperElementRigid.png
   :width: 400

   Sketch of marker nodes, exemplary node \ :math:`i`\ , reference coordinates and marker coordinate system; note the difference of the center of the marker 'surface' (rectangle) marked with the red cross, and the averaged of the averaged local reference position.


Marker quantities
-----------------

The marker provides a 'position' jacobian, which is the derivative of the global marker velocity w.r.t.\ the 
object velocity coordinates \ :math:`\dot {\mathbf{q}}_{n_b}`\ ,

.. math::

   \LU{0}{{\mathbf{J}}_{m,pos}} = \frac{\partial \LU{0}{{\mathbf{v}}}_{m}}{\dot {\mathbf{q}}_{n_b}} .


In case of \ ``ObjectGenericODE2``\ , assuming pure displacement based nodes,
the jacobian will consist of zeros and unit matrices \ :math:`{\mathbf{I}}`\  ,

.. math::

   \LU{0}{{\mathbf{J}}_{m,pos}^{GenericODE2}} = \frac{\partial \LU{0}{{\mathbf{v}}}_{m}}{\dot {\mathbf{q}}_{n_b}} = \left[ \Null,\; \ldots,\; \Null,\; w_0 {\mathbf{I}},\; \Null,\; \ldots,\; \Null,\; w_1 {\mathbf{I}},\; \Null,\; \ldots,\; \Null \right],


in which the \ :math:`{\mathbf{I}}`\  matrices are placed at the according indices of marker nodes.

In case of \ ``ObjectFFRFreducedOrder``\ , this jacobian is computed as weighted sum 
of the position jacobians, see \ ``ObjectFFRFreducedOrder``\ ,

.. math::

   \LU{0}{{\mathbf{J}}_{m,pos}^{FFRFreduced}} = \frac{\partial \LU{0}{{\mathbf{v}}}_{m}}{\dot {\mathbf{q}}_{n_b}} = \sum_i w_i \LU{0}{{\mathbf{J}}^{(i)}_\mathrm{pos}} = \left[{\mathbf{I}}, \; -\LU{0r}{\Rot} \left(\LU{r}{{\mathbf{o}}\cRef} + \sum_i \LU{r}{{\mathbf{p}}^{(i)}} \right) \LU{r}{{\mathbf{G}}},\; \sum_i w_i \LU{0r}{\Rot} \vr{\LU{r}{\tPsi_{r=3i}\tp}}{\LU{r}{\tPsi_{r=3i+1}\tp}}{\LU{r}{\tPsi_{r=3i+2}\tp}} \right] .


In \ ``ObjectFFRFreducedOrder``\ , the jacobian usually affects all reduced coordinates.


Standard approach for computation of rotation (\ ``useAlternativeApproach = False``\ )
--------------------------------------------------------------------------------------

As compared to \ ``MarkerSuperElementPosition``\ , \ ``MarkerSuperElementRigid``\  also links the marker to the orientation of 
the set of nodes provided. For this reason, the check performed in \ ``mbs.assemble()``\  will take care that the nodes are capable
to describe rotations.
The first approach, here called as a standard, follows the idea that displacements contribute to rotation are weighted by their quadratic distance, 
cf.\ , and gives the (small rotation) rotation vector

.. math::

   \LU{r}{\ttheta}_{m} = \frac{\sum_i w_i \LU{r}{{\mathbf{p}}_{ref}^{(i)}} \times \LU{r}{{\mathbf{u}}^{(i)}}}{\sum_i w_i |\LU{r}{{\mathbf{p}}_{ref}^{(i)}}|^2}


Note that \ :math:`{\mathbf{p}}_{ref}^{(i)}`\  is not the reference position in the \ ``ObjectFFRFreducedOrder``\  object, but it is relative to the midpoint reference position
all marker nodes, given in \ :math:`\LU{r}{{\mathbf{x}}^\mathrm{avg}\cRef}`\ .
Accordingly, the marker local angular velocity can be calculated as

.. math::

   \LU{r}{\tomega}_{m} = \LU{r}{\dot \ttheta}_{m} = \frac{\sum_i w_i \LU{r}{\tilde {\mathbf{p}}_{ref}^{(i)}} \LU{r}{{\mathbf{v}}_i}}{\sum_i w_i |\LU{r}{{\mathbf{p}}_{ref}^{(i)}}|^2}


The marker also provides a `rotation' jacobian, which is the derivative of the marker angular velocity \ :math:`\LU{0}{\tomega}_{m}`\  w.r.t.\ the 
object velocity coordinates \ :math:`\dot {\mathbf{q}}_{n_b}`\ ,

.. math::

   \LU{0}{{\mathbf{J}}_{m,rot}} = \frac{\partial \LU{0}{\tomega}_{m}}{\partial \dot {\mathbf{q}}_{n_b}} = \frac{\partial \LU{0r}{\Rot}(\LU{r}{\tomega_{r}} + \LU{r}{\tomega}_{m})}{\partial \dot {\mathbf{q}}_{n_b}} = \LU{0r}{\Rot} \left(\frac{\partial \LU{r}{\tomega}_{r}}{\partial \dot {\mathbf{q}}_{n_b}} + \frac{\sum_i w_i \LU{r}{\tilde {\mathbf{p}}_{ref}^{(i)}} \LU{r}{{\mathbf{J}}_{pos}^{(i)}}}{\sum_i w_i |\LU{r}{{\mathbf{p}}_{ref}^{(i)}}|^2} \right)


In case of \ ``ObjectFFRFreducedOrder``\ , this jacobian is computed as

.. math::
   :label: eq-markersuperelementrigid-jacrotstandard

   \LU{0}{{\mathbf{J}}_{m,rot}^{FFRFreduced}} = \left[\Null,\; \LU{0r}{\Rot} \LU{r}{{\mathbf{G}}_{local}},\; \LU{0r}{\Rot} \frac{\sum_i w_i \LU{r}{\tilde {\mathbf{p}}_{ref}^{(i)}} \LU{r}{{\mathbf{J}}_{pos,f}^{(i)}}}{\sum_i w_i |\LU{r}{{\mathbf{p}}_{ref}^{(i)}}|^2} \right]


in which you should know that

+  we used \ :math:`\frac{\partial \LU{r}{\tomega_{r}} }{\partial \dot \ttheta_r} = \LU{r}{{\mathbf{G}}_{local}}`\ , 
+  \ :math:`\ttheta_{r}`\  represent the rotation parameters for the rigid body node of \ ``ObjectFFRFreducedOrder``\ ,
+  \ :math:`\LU{r}{{\mathbf{J}}_{pos,f}^{(i)}}`\  is the \ **local**\  jacobian, which only includes the flexible part of the local 
        jacobian for a single mesh node, \ :math:`\LU{r}{{\mathbf{J}}_{pos}^{(i)}}`\  (note the small \ :math:`r`\  on the upper left), 
        as defined in \ ``ObjectFFRFreducedOrder``\ .
 
For further quantities also consult the according description in \ ``ObjectFFRFreducedOrder``\ .



Alternative computation of rotation (\ ``useAlternativeApproach = True``\ )
---------------------------------------------------------------------------

Note that this approach is \ **still under development**\  and needs further validation. 
However, tests show that this model is superior to the standard approach, as it improves the averaging of motion w.r.t.\ rotations
at the marker nodes.

In the alternative approach, the weighting matrix \ :math:`{\mathbf{W}}`\  
has the interpretation of an inertia tensor built from nodes using weights equal to node masses.
In such an interpretation, the 'local angular momentum' w.r.t.\ the marker (averaged) position can be computed as 

.. math::
   :label: eq-markersuperelementrigid-omegaandwm

   {\mathbf{W}} \LU{r}{\tomega}_{m} = \sum_i w_i \LU{r}{\tilde {\mathbf{p}}_{ref}^{(i)}} \left(\LU{r}{{\mathbf{v}}^{(i)}} - \LU{r}{{\mathbf{v}}^\mathrm{avg}}\right)= -\sum_i  \left( w_i \LU{r}{\tilde {\mathbf{p}}_{ref}^{(i)}} \LU{r}{\tilde {\mathbf{p}}_{ref}^{(i)}} \right) \LU{r}{\tomega}_{m}


which implicitly defines the weighting matrix \ :math:`{\mathbf{W}}`\ , which must be invertable (but it is only a \ :math:`3 \times 3`\  matrix!),

.. math::

   {\mathbf{W}} = -\sum_i  w_i \LU{r}{\tilde {\mathbf{p}}_{ref}^{(i)}} \LU{r}{\tilde {\mathbf{p}}_{ref}^{(i)}}


Furthermore, we need to introduce the averaged velocity of the marker averaged reference position, using \ :math:`\LU{r}{\dot {\mathbf{u}}^{(i)}} = \LU{r}{{\mathbf{v}}^{(i)}}`\ , which is defined as

.. math::

   \LU{r}{{\mathbf{v}}^\mathrm{avg}} = \sum_i  w_i \LU{r}{{\mathbf{v}}^{(i)}} ,


similar to the averaged local reference position \ :math:`\LU{r}{{\mathbf{x}}^\mathrm{avg}\cRef}`\  given in the table above, see also \ :numref:`fig-markersuperelementrigid-sketch`\ .

In the alternative approach, thus the marker local rotations read

.. math::

   \LU{r}{\ttheta}_{m,alt} = {\mathbf{W}}^{-1} \sum_i w_i \LU{r}{\tilde {\mathbf{p}}_{ref}^{(i)}} \left( \LU{r}{{\mathbf{u}}^{(i)}} - \LU{r}{{\mathbf{x}}^\mathrm{avg}\cRef} \right) ,


and the marker local angular velocity is defined as

.. math::

   \LU{r}{\tomega}_{m,alt} = {\mathbf{W}}^{-1} \sum_i w_i \LU{r}{\tilde {\mathbf{p}}_{ref}^{(i)}} \left( \LU{r}{{\mathbf{v}}^{(i)}} - \LU{r}{{\mathbf{v}}^\mathrm{avg}} \right) .


Note that, the average velocity \ :math:`\LU{r}{{\mathbf{v}}^\mathrm{avg}}`\  would cancel out in a symmetric mesh, but would cause spurious 
angular velocities in unsymmetric (w.r.t.\ the axis of rotation) distribition of mesh nodes. 
This could even lead to spurious rotations or angular velocities in pure translatoric motion.

In the alternative mode, the Jacobian for the rotation / angular velocity is defined as

.. math::

   \LU{0}{{\mathbf{J}}_{m,rot,alt}} = \frac{\partial \LU{0}{\tomega}_{m}}{\partial \dot {\mathbf{q}}_{n_b}} = \frac{\partial \LU{0r}{\Rot}(\LU{r}{\tomega_{r}} + \LU{r}{\tomega}_{m})}{\partial \dot {\mathbf{q}}_{n_b}} = \LU{0r}{\Rot} \left(\frac{\partial \LU{r}{\tomega}_{r}}{\partial \dot {\mathbf{q}}_{n_b}}  + {\mathbf{W}}^{-1} \sum_i w_i \LU{r}{\tilde {\mathbf{p}}_{ref}^{(i)}} \LU{r}{{\mathbf{J}}_{pos}^{(i)}}\right)


In case of \ ``ObjectFFRFreducedOrder``\ , this jacobian is computed as

.. math::

   \LU{0}{{\mathbf{J}}_{m,rot,alt}^{FFRFreduced}} = \left[\Null,\; \LU{0r}{\Rot} \LU{r}{{\mathbf{G}}_{local}},\; \LU{0r}{\Rot} {\mathbf{W}}^{-1} \sum_i w_i \LU{r}{\tilde {\mathbf{p}}_{ref}^{(i)}} \LU{r}{{\mathbf{J}}_{pos,f}^{(i)}} \right]


see also the descriptions given after Eq. :eq:`eq-markersuperelementrigid-jacrotstandard`\  in the 'standard' approach.


\ **EXAMPLE for marker on body 4, mesh nodes 10,11,12,13**\ :

\ ``MarkerSuperElementRigid(bodyNumber = 4, meshNodeNumber = [10, 11, 12, 13], weightingFactors = [0.25, 0.25, 0.25, 0.25], referencePosition=[0,0,0])``\ 


For detailed examples, see \ ``TestModels``\ .


Relevant Examples and TestModels with weblink:

    \ `CMSexampleCourse.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/CMSexampleCourse.py>`_\  (Examples/), \ `netgenSTLtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/netgenSTLtest.py>`_\  (Examples/), \ `NGsolveCMStutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCMStutorial.py>`_\  (Examples/), \ `NGsolveCraigBampton.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCraigBampton.py>`_\  (Examples/), \ `NGsolveFFRF.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveFFRF.py>`_\  (Examples/), \ `NGsolveLinearFEM.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveLinearFEM.py>`_\  (Examples/), \ `NGsolveModalAnalysis.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveModalAnalysis.py>`_\  (Examples/), \ `ObjectFFRFconvergenceTestBeam.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ObjectFFRFconvergenceTestBeam.py>`_\  (Examples/), \ `ObjectFFRFconvergenceTestHinge.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ObjectFFRFconvergenceTestHinge.py>`_\  (Examples/), \ `pendulumVerify.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulumVerify.py>`_\  (Examples/), \ `serialRobotFlexible.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotFlexible.py>`_\  (Examples/), \ `abaqusImportTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/abaqusImportTest.py>`_\  (TestModels/), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TestModels/), \ `linearFEMgenericODE2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/linearFEMgenericODE2.py>`_\  (TestModels/), \ `NGsolveCMStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/NGsolveCMStest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


