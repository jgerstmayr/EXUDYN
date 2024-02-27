Frames, rotations and coordinate systems
========================================

In this section, we introduce frames, which provide reference points and rotations attached to the ground or to bodies, thus providing a bases for coordinate systems.


Reference points and reference frames
-------------------------------------

In contrast to the reference coordinates (which may include reference position and rotations), the reference point of a body, marker or joint is available in any configuration (current, initial, etc.). 
The reference point and orientation (or reference frame) of a rigid body coincides with the position and orientation of the underlying node.
The local position of a point of the body is expressed relative to the body's reference frame, which may be different from the body's center of mass.
The reference frame is also used for \ :ref:`FFRF <FFRF>`\  objects. In most cases, reference points are denoted by \ :math:`\pRef`\ .


Coordinate systems of frames
----------------------------

Many considerations and computations in kinematics can be formulated coordinate-free.
However, ultimately local positions are given by the user in different coordinates from results given in global (world) coordinates.

For this reason, we use left upper indices in symbols to indicate underlying frames of points or vectors, e.g.,
\ :math:`\LU{0}{{\mathbf{u}}}`\  represents a displacement vector in the global (world) coordinate system \ :math:`0`\ , while 
\ :math:`\LU{m1}{{\mathbf{u}}}`\  represents the displacement vector in marker 1 (\ :math:`m1`\ ) coordinates. 

Typical coordinate systems (frames) are:

+  \ :math:`\LU{0}{{\mathbf{u}}}`\  \ :math:`\ldots`\  global or world coordinates
+  \ :math:`\LU{b}{{\mathbf{u}}}`\  \ :math:`\ldots`\  body-fixed, local coordinates
+  \ :math:`\LU{m0}{{\mathbf{u}}}`\  \ :math:`\ldots`\  local coordinates of (the body or node of) marker \ :math:`m0`\ 
+  \ :math:`\LU{m1}{{\mathbf{u}}}`\  \ :math:`\ldots`\  local coordinates of (the body or node of) marker \ :math:`m1`\ 
+  \ :math:`\LU{J0}{{\mathbf{u}}}`\  \ :math:`\ldots`\  local coordinates of joint \ :math:`0`\ , related to marker \ :math:`m0`\ 
+  \ :math:`\LU{J1}{{\mathbf{u}}}`\  \ :math:`\ldots`\  local coordinates of joint \ :math:`1`\ , related to marker \ :math:`m1`\ 

To transform the local coordinates \ :math:`\LU{m0}{{\mathbf{u}}}`\  of marker 0 into global coordinates \ :math:`\LU{0}{{\mathbf{x}}}`\ , we use the relation

.. math::
   :label: eq-theory-coordinatesystems-rot

   \LU{0}{{\mathbf{u}}} = \LU{0,m0}{\Rot} \LU{m0}{{\mathbf{u}}}


in which \ :math:`\LU{0,m0}{\Rot}`\  is the transformation matrix of (the body or node of) the underlying marker 0.

In particular, note that any transformation, e.g., of the angular velocity in body (b) and world coordinates,

.. math::

   \LU{0}{\tomega} = \LU{0,b}{\Rot} \LU{b}{\tomega}


only transforms the vector from one frame to the other, but does not change the vector \ :math:`\tomega`\  itself.




Homogeneous transformations
---------------------------

In order to considere rotations and translations in frames, homogeneous transformations are used.
An arbitrary vector \ :math:`\LU{i}{{\mathbf{r}}}`\  in coordinates of frame \ :math:`\mathcal{F}_i`\  can be expressed relative to \ :math:`\mathcal{F}_j`\  with the vector \ :math:`\LU{j}{{\mathbf{p}}_i}`\ , pointing from \ :math:`O_j`\  to \ :math:`O_i`\  and a rotation matrix according to Eq. :eq:`eq-theory-coordinatesystems-rot`\ :

.. math::
   :label: eq-theory-ht-positionrelation

   \LU{j}{{\mathbf{r}}} = \LU{ji}{{\mathbf{R}}} \LU{i}{{\mathbf{r}}} + \LU{j}{{\mathbf{p}}_i}


Eq. :eq:`eq-theory-ht-positionrelation`\  can be written as a linear mapping,

.. math::

   \vp{\LU{j}{{\mathbf{r}}}}{1} = \mp{\LU{ji}{{\mathbf{R}}}}{\LU{j}{{\mathbf{p}}_i}}{\Null^T}{1} \vp{\LU{i}{{\mathbf{r}}}}{1}, \quad


in which the matrix

.. math::

   \LU{ji}{{\mathbf{T}}} = \mp{\LU{ji}{{\mathbf{R}}}}{\LU{j}{{\mathbf{p}}_i}}{\Null^T}{1}


is referred to as the \ **homogeneous transformation matrix**\ , and \ :math:`\LU{j}{\hat {\mathbf{r}}} = [\LU{j}{{\mathbf{r}}} \;\; 1]^T`\  is the homogeneous vector corresponding to \ :math:`\LU{j}{{\mathbf{r}}}`\ .

Analogous to the rotation matrix, \ :math:`\LU{ji}{{\mathbf{T}}}`\  has an inverse, which follows as

.. math::

   \LURU{ji}{{\mathbf{T}}}{}{-1} = \LU{ij}{{\mathbf{T}}} = \mp{\LURU{ji}{{\mathbf{R}}}{}{T}} {-\LURU{ji}{{\mathbf{R}}}{}{T} \LU{j}{{\mathbf{p}}_i}}{\Null^T}{1} \vp{\LU{j}{{\mathbf{r}}}}{1} .


Sequential application of homogeneous transformations simplifies to

.. math::

   \LU{ki}{{\mathbf{T}}} = \LU{kj}{{\mathbf{T}}} \LU{ji}{{\mathbf{T}}} .


We could also actively move vectors, by assuming that the homogeneous transformation matrix \ :math:`\LU{i0,i1}{{\mathbf{T}}}`\  transforms coordinates within the same frame \ :math:`\mathcal{F}_i`\ , between two (time) steps 0 and 1,

.. math::

   \LU{i1}{\hat {\mathbf{r}}} = \LU{i1,i0}{{\mathbf{T}}} \LU{i0}{\hat {\mathbf{r}}}


which advances the vector \ :math:`\LU{i0}{\hat {\mathbf{r}}}`\  in time.

Note that an efficient implementation would only include \ :math:`{\mathbf{R}}`\  and \ :math:`{\mathbf{p}}`\ , without necessarily computing \ :math:`4 \times 4`\  matrices.



.. _fig-theory-HT-changeOfFrame:
.. figure:: ../theDoc/figures/theoryRotationsHTchangeOfFrame.png
   :width: 320

   Homogeneous transformation from \ :math:`\mathcal{F}_i`\  to \ :math:`\mathcal{F}_j`\  using the relation \ :math:`\LU{j}{\hat{\mathbf{r}} } = \LU{ji}{{\mathbf{T}}} \LU{i}{\hat{\mathbf{r}} }`\ 




All homogeneous transformations form the \ **Special Euclidean Group SE(3)**\  -- the motion group -- for describing rigid body movements in 3D. Therefore, homogeneous transformations also meet the criteria for groups (closure, associativity, existence of neutral and inverse elements).

Elementary rotations about the \ :math:`{\mathbf{e}}_z`\ -axis and translations along the \ :math:`{\mathbf{e}}_x`\ -axis, for example, are given by:


.. math::

   \mathbf{Rot}({\mathbf{e}}_z, \theta) = \vfour{\cos \theta ,\; -\sin \theta ,\; 0 ,\; 0}{\sin \theta ,\; \cos \theta ,\; 0 ,\; 0}{0 ,\; 0 ,\; 1 ,\; 0}{0 ,\; 0 ,\; 0 ,\; 1}, \quad \mathbf{Trans}({\mathbf{e}}_x, d) = \vfour{1 ,\; 0 ,\; 0 ,\; d}{0 ,\; 1 ,\; 0 ,\; 0}{0 ,\; 0 ,\; 1 ,\; 0}{0 ,\; 0 ,\; 0 ,\; 1}




Arbitrary composite transformations can be constructed from elementary transformations. In general, the following applies:

+  Pre-multiplication (\ **Pre-Multiplication**\ ): Transformation in the global coordinate system
+  Post-multiplication (\ **Post-Multiplication**\ ): Transformation in the rotated coordinate system

By exploiting the tools of so-called Lie groups, rigid body movements can be interpolated using matrix logarithm (\ ``LogSE3(...)``\ ) and matrix exponential function (\ ``ExpSE3(...)``\ ).




Rotations
---------


In this section, we discuss in particular rotations, as already introduced for coordinate systems and homogeneous transformations.
Rotations can be represented by transformation matrices, leading to a linear transformation

.. math::

   \LU{0}{{\mathbf{r}}} = \LU{0,1}{\Rot} \LU{1}{{\mathbf{r}}}


which transforms the vector \ :math:`\LU{1}{{\mathbf{r}}}`\  accordingly.
However, rotations are inherently nonlinear. First, rotation parametrizations -- except for the components of the rotation matrix itself -- are highly nonlinear functions of rotation parameters.
Second, subsequent rotations couple in a multiplicative (i.e., nonlinear) way. Therefore rotations have to be treated differently from translations.



Derivation of the transformation matrix from coordinate transformations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


The vector \ :math:`{\mathbf{a}}`\  can be represented in coordinates of frame \ :math:`\mathcal{F}_1`\  with basis vectors \ :math:`({\mathbf{e}}_{x1},\,{\mathbf{e}}_{y1},\,{\mathbf{e}}_{z1})`\  and of frame \ :math:`\mathcal{F}_2`\  with basis vectors \ :math:`({\mathbf{e}}_{x2},\,{\mathbf{e}}_{y2},\,{\mathbf{e}}_{z2})`\ ,

.. math::
   :label: eq-theory-rotations-trans1

   {\mathbf{a}} = \LUR{1}{a}{x} {\mathbf{e}}_{x1} + \LUR{1}{a}{y} {\mathbf{e}}_{y1} +\LUR{1}{a}{z} {\mathbf{e}}_{z1} = \LUR{2}{a}{x} {\mathbf{e}}_{x2} + \LUR{2}{a}{y} {\mathbf{e}}_{y2} +\LUR{2}{a}{z} {\mathbf{e}}_{z2} .


Multiplying Eq. :eq:`eq-theory-rotations-trans1`\  with the basis vectors \ :math:`({\mathbf{e}}_{x1},\,{\mathbf{e}}_{y1},\,{\mathbf{e}}_{z1})`\  consecutively, we obtain three relations,

.. math::

   \LUR{1}{a}{x} &=& \LUR{2}{a}{x} {\mathbf{e}}_{x1}^T{\mathbf{e}}_{x2} + \LUR{2}{a}{y} {\mathbf{e}}_{x1}^T{\mathbf{e}}_{y2} +\LUR{2}{a}{z} {\mathbf{e}}_{x1}^T{\mathbf{e}}_{z2} , \nonumber\\
   \LUR{1}{a}{y} &=& \LUR{2}{a}{x} {\mathbf{e}}_{y1}^T{\mathbf{e}}_{x2} + \LUR{2}{a}{y} {\mathbf{e}}_{y1}^T{\mathbf{e}}_{y2} +\LUR{2}{a}{z} {\mathbf{e}}_{y1}^T{\mathbf{e}}_{z2} , \nonumber\\
   \LUR{1}{a}{z} &=& \LUR{2}{a}{x} {\mathbf{e}}_{z1}^T{\mathbf{e}}_{x2} + \LUR{2}{a}{y} {\mathbf{e}}_{z1}^T{\mathbf{e}}_{y2} +\LUR{2}{a}{z} {\mathbf{e}}_{z1}^T{\mathbf{e}}_{z2} .


where we apply the orthonormality relationships \ :math:`{\mathbf{e}}^T_{x1} {\mathbf{e}}_{x1} = 1`\ , \ :math:`\ldots`\  as well as \ :math:`{\mathbf{e}}^T_{x1} {\mathbf{e}}_{y1} = 0`\ , \ :math:`{\mathbf{e}}^T_{x1} {\mathbf{e}}_{z1} = 0`\ , \ :math:`\ldots`\  througout.

We can represent the result as linear transformation

.. math::

   \vr{\LUR{1}{a}{x}}{\LUR{1}{a}{y}}{\LUR{1}{a}{z}} = \mr{{\mathbf{e}}_{x1}^T{\mathbf{e}}_{x2}}{{\mathbf{e}}_{x1}^T{\mathbf{e}}_{y2}}{{\mathbf{e}}_{x1}^T{\mathbf{e}}_{z2}} {{\mathbf{e}}_{y1}^T{\mathbf{e}}_{x2}}{{\mathbf{e}}_{y1}^T{\mathbf{e}}_{y2}}{{\mathbf{e}}_{y1}^T{\mathbf{e}}_{z2}} {{\mathbf{e}}_{z1}^T{\mathbf{e}}_{x2}}{{\mathbf{e}}_{z1}^T{\mathbf{e}}_{y2}}{{\mathbf{e}}_{z1}^T{\mathbf{e}}_{z2}} \vr{\LUR{2}{a}{x}}{\LUR{2}{a}{y}}{\LUR{2}{a}{z}}


or in short

.. math::

   \LU{1}{{\mathbf{a}}} = \LU{12}{\Rot} \LU{2}{{\mathbf{a}}} .


The \ **Transformationsmatrix**\  \ :math:`\LU{12}{\Rot}`\  transforms coordinates of vector \ :math:`{\mathbf{a}}`\  from \ :math:`\mathcal{F}_2`\  into \ :math:`\mathcal{F}_1`\ .
We observe that the columns of \ :math:`\LU{12}{\Rot}`\  contain unit vectors \ :math:`{\mathbf{e}}_{x2}`\ , \ :math:`{\mathbf{e}}_{y2}`\ , and \ :math:`{\mathbf{e}}_{z2}`\  represented in frame \ :math:`\mathcal{F}_1`\ ,
and that the rows can be identified as the unit vectors \ :math:`{\mathbf{e}}_{x1}^T`\ , \ :math:`{\mathbf{e}}_{y1}^T`\ , and \ :math:`{\mathbf{e}}_{z1}^T`\  represented in frame \ :math:`\mathcal{F}_2`\ :

.. math::

   \LU{12}{\Rot} = \left[ \LUR{1}{{\mathbf{e}}}{x2} \quad \LUR{1}{{\mathbf{e}}}{y2} \quad \LUR{1}{{\mathbf{e}}}{z2}\right] = \vr{\LURU{2}{{\mathbf{e}}}{x1}{\mathrm{T}}}{\LURU{2}{{\mathbf{e}}}{y1}{\mathrm{T}}}{\LURU{2}{{\mathbf{e}}}{z1}{\mathrm{T}}} .


Note that:

+  Columns and rows of \ :math:`\LU{12}{\Rot}`\  represent an orthonormal right-hand system. 
+  Because the determinant \ :math:`\det(\LU{12}{\Rot})= + 1`\ , we call \ :math:`\LU{12}{\Rot}`\  to be proper-orthogonal.
+  Rotations conserve length, \ :math:`|{\mathbf{A}} {\mathbf{x}}| = |{\mathbf{x}}|`\ .
+  For the same reason, rotations conserve angles.
+  During transformations, we observe that reading left upper indices from left to right, indices are changed only by transformation matrices (which are not transposed): \ :math:`\LU{2}{{\mathbf{a}}} = \LU{21}{\Rot} \LU{1}{{\mathbf{a}}}`\ 

We note the following rules:

.. math::

   \LU{21}{\Rot} &=& \LURU{12}{\Rot}{}{-1} ,\nonumber \\
   \LURU{12}{\Rot}{}{-1} &=& \LURU{12}{\Rot}{}{T} ,\nonumber \\
   \LU{12}{\Rot} \LURU{12}{\Rot}{}{T} &=& {\mathbf{I}} , \nonumber \\
   \LURU{12}{\Rot}{}{T} \LU{12}{\Rot} &=& {\mathbf{I}} .





Elementary rotations
^^^^^^^^^^^^^^^^^^^^


Elementary rotations are rotations about a single axis, using one of the orthogonal basis vectors.
Consider basis \ :math:`({\mathbf{e}}_{x1},\,{\mathbf{e}}_{y1},\,{\mathbf{e}}_{z1})`\  and a rotation around axis \ :math:`{\mathbf{e}}_{x1}`\  with angle \ :math:`\varphi_1`\  in positive rotation sense, obtaining a rotated frame \ :math:`({\mathbf{e}}_{x2},\,{\mathbf{e}}_{y2},\,{\mathbf{e}}_{z2})`\ , see \ :numref:`fig-theory-rotations-elementaryx`\ .
The rotation matrix for this case reads:

.. math::

   \LU{12}{\Rot} = \left[ \LUR{1}{{\mathbf{e}}}{x2} \;\; \LUR{1}{{\mathbf{e}}}{y2} \;\; \LUR{1}{{\mathbf{e}}}{z2} \right] = \mr{1}{0}{0}{0}{c \varphi_1}{-s \varphi_1}{0}{s \varphi_1}{c \varphi_1}.


In this case, the coordinates of a vector \ :math:`{\mathbf{r}}`\  are transformed according to

.. math::

   \LU{1}{{\mathbf{r}}} = \LU{12}{\Rot} \LU{2}{{\mathbf{r}}}





.. _fig-theory-rotations-elementaryx:
.. figure:: ../theDoc/figures/elementaryRotationX.png
   :width: 320

   Elementary rotation around axis \ :math:`\mathbf{ x}_1`\ .


In a second example, a rotation with angle \ :math:`\varphi_2`\  around \ :math:`{\mathbf{e}}_{y2}`\  is performed to transform from basis \ :math:`({\mathbf{e}}_{x2},\,{\mathbf{e}}_{y2},\,{\mathbf{e}}_{z2})`\  into \ :math:`({\mathbf{e}}_{x3},\,{\mathbf{e}}_{y3},\,{\mathbf{e}}_{z3})`\ , see \ :numref:`fig-theory-rotations-elementaryy`\ .
The rotation matrix for this case reads:

.. math::

   \LU{23}{\Rot} = \left[ \LUR{2}{{\mathbf{e}}}{x3} \;\; \LUR{2}{{\mathbf{e}}}{y3} \;\; \LUR{2}{{\mathbf{e}}}{z3} \right] = \mr{c \varphi_2}{0}{s \varphi_2}{0}{1}{0}{-s \varphi_2}{0}{c \varphi_2}.




.. _fig-theory-rotations-elementaryy:
.. figure:: ../theDoc/figures/elementaryRotationY.png
   :width: 320

   Elementary rotation around axis \ :math:`\mathbf{ y}_2`\ .


Finally, a rotation around \ :math:`{\mathbf{e}}_{z3}`\  with angle \ :math:`\varphi_3`\  would give similarly,

.. math::

   \LU{34}{\Rot} = \left[ \LUR{3}{{\mathbf{e}}}{x4} \;\; \LUR{3}{{\mathbf{e}}}{y4} \;\; \LUR{3}{{\mathbf{e}}}{z4} \right] = \mr{c \varphi_3}{-s \varphi_3}{0} {s \varphi_3}{c \varphi_3}{0} {0}{0}{1} .






Linearized rotations
^^^^^^^^^^^^^^^^^^^^


In this case we can interpret the small rotations as a constant angular velocity over small time \ :math:`\Delta t`\ , \ :math:`{\mathbf{t}}arphi = \Delta t \cdot \tomega`\ , which results in

.. math::

   {\mathbf{r}}_1^\prime= {\mathbf{r}}_1 + \Delta t \cdot (\tomega \times {\mathbf{r}}_1) = ({\mathbf{I}} + \tilde {\mathbf{t}}arphi) {\mathbf{r}}_1


Leading to the linearized rotation matrix

.. math::

   \Rot_\mathrm{lin} = {\mathbf{I}} - \tilde {\mathbf{t}}arphi



Alternatively, using linearized rotations \ :math:`{\mathbf{t}}arphi = [\varphi_1,\;\varphi_2,\;\varphi_3]\tp`\ , with \ :math:`|{\mathbf{t}}arphi| \ll 1`\ , we can approximate \ :math:`\sin \varphi_1`\  as:

.. math::

   \sin \varphi_1 = \varphi_1 - \frac{\varphi_1^3}{3!} + \frac{\varphi_1^5}{5!} - ... \approx \varphi_1


Similarly, we can approximate \ :math:`\cos \varphi_1 \approx 1`\ . This immediately gives a linearization for elementary rotations -- where we observe that contributions for each axis can be added.

The axis-angle representation (see later) leads to the same result by linearization of the Rodrigues formula,

.. math::

   \Rot({\mathbf{u}}, \varphi) \approx {\mathbf{I}} + \tilde {\mathbf{u}} \varphi, \quad \Rot\tp \approx {\mathbf{I}} - \tilde {\mathbf{u}} \varphi


in which \ :math:`\varphi`\  represents the infinitesimal angle and \ :math:`{\mathbf{u}}`\  is the rotation axis.

\ **Note**\ : whenever you would like to work with linearized rotations, or if you do not know the sequence of small rotations, still do not use the linearized formula as it gives immediately rotation matrices without strict orthogonality and determinant of 1. In order to avoid computational problems, use for example \ ``RotationVector2RotationMatrix(phi)``\  to compute a consistent rotation matrix based on the matrix exponential.



Active and passive rotations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is important to distinguish two fundamentally different concepts of rotations.
There is the \ **active rotation**\ , which can be represented by a coordinate-free relation,

.. math::
   :label: eq-theory-rotations-active

   {\mathbf{v}}^\prime = \Rot {\mathbf{v}}


in which a vector \ :math:`{\mathbf{v}}`\  is actively rotated into a new configuration \ :math:`{\mathbf{v}}^\prime`\  using a rotation tensor \ :math:`\Rot`\ .
Eq. :eq:`eq-theory-rotations-active`\  can be represented in any coordinate system, but keeping it fixed.

Furthermore, we denote a \ **passive rotation**\  as a coordinate transformation, as used many times before,

.. math::

   \LU{1}{{\mathbf{v}}} = \LU{12}{\Rot} \LU{2}{{\mathbf{v}}}


in which the vector \ :math:`{\mathbf{v}}`\  is not changed at all, but only its coordinates are represented in two different coordinate systems.

While passive rotations (and homogeneous transformations) are used, e.g., to compute current positions of body-attached points through several relative coordinate systems, active rotations can represent the rotations from one time step to the next one.


Successive rotations
^^^^^^^^^^^^^^^^^^^^

The successive application of rotation matrices is non-commutative. An exception is the planar case, i.e., all rotations occur around the same axis.
Specifically, we see that for successive rotations, the following must be considered:

.. math::

   {\mathbf{A}}_2 {\mathbf{A}}_1 {\mathbf{v}} \neq {\mathbf{A}}_1 {\mathbf{A}}_2 {\mathbf{v}} .


As an example, we consider in Figure  :ref:`fig-theory-rotations-successive`\  the different order of rotations of a block.


.. _fig-theory-rotations-successive:
.. figure:: ../theDoc/figures/RotationsSequences.png
   :width: 500

   Successive rotations are not commutative.



In \ :numref:`fig-theory-rotations-successive`\ a, the block shown is first rotated 90° about the \ :math:`\mathbf{e}_2`\  axis and then 90° about the \ :math:`\mathbf{e}_3`\  axis. In Figure  :ref:`fig-theory-rotations-successive`\ b, the same rotations are applied in reverse order, i.e., the block is first rotated 90° about the \ :math:`\mathbf{e}_3`\  axis and then 90° about the \ :math:`\mathbf{e}_2`\  axis. It is immediately apparent that the resulting orientation of the block is different in both cases.

Finally, it should be noted that there are two different types of successive rotations.
In the first variant, also known as the \ **single-frame method**\ , the same reference frame is chosen for each rotation. This variant is best understood in the active rotation of vectors, where these rotations always take place, for example, in the global reference system.

In the second variant, also known as the \ **multi-frame method**\ , a new reference frame created by the preceding rotation is used for each rotation. This variant can be illustrated by applications in robotics (e.g., articulated arm robots), where (passive) rotations of the reference systems of the robot's respective arms occur, with each rotation taking place in the reference system of the preceding arm.



Rotation tensor: axis-angle representation
------------------------------------------

In the following, we will elaborate the rotation tensor, inherently linked to the axis-angle representation.
Note that the rotation axis times the angle is denoted as rotation vector throughout.

Considering Euler's theorem on rotations, we assume a transformation

.. math::

   {\mathbf{r}}_0 \ra {\mathbf{r}}(t)


which purely follows from a rotation. This transformation thus can be represented by a rotation axis \ :math:`{\mathbf{u}}`\  and an angle \ :math:`\varphi(t)`\ ,

.. math::

   ({\mathbf{u}}(t), \, \varphi(t))


both of them given in a coordinate-free manner.
We may therefore conclude

.. math::

   {\mathbf{r}}(t)=\Rot(t) \, {\mathbf{r}}_0 \qquad \text{with} \qquad \Rot(t)=\Rot({\mathbf{u}}(t),\varphi(t))




.. _fig-theory-rotations-angleaxis:
.. figure:: ../theDoc/figures/RotationAxisAngle.png
   :width: 260

   Rotation of a vector \ :math:`\mathbf{ r}_0`\  by means of the angle-axis tuple \ :math:`(\mathbf{ u}(t), \, \varphi(t))`\ .



Using \ :numref:`fig-theory-rotations-angleaxis`\ , we may now consider relations of the two frames \ :math:`({\mathbf{e}}_{x0},\,{\mathbf{e}}_{y0},\,{\mathbf{e}}_{z0})`\  and \ :math:`({\mathbf{e}}_{x1},\,{\mathbf{e}}_{y1},\,{\mathbf{e}}_{z1})`\ , solely defined by the angle-axis \ :math:`({\mathbf{u}}(t), \, \varphi(t))`\  relation.


.. _fig-theory-rotations-axisanglederivation:
.. figure:: ../theDoc/figures/RotationAxisAngleDerivation.png
   :width: 320

   Relations for derivation of rotation tensor and Rodrigues' formula.



According to \ :numref:`fig-theory-rotations-axisanglederivation`\ , we may split the vector \ :math:`{\mathbf{r}}`\ , which has the length \ :math:`r = \vert {\mathbf{r}} \vert`\ , into

.. math::

   {\mathbf{r}}= c_1 {\mathbf{e}}_1 + c_2 {\mathbf{e}}_2 + c_3 {\mathbf{e}}_3


with the unit vectors given as

.. math::

   {\mathbf{e}}_1  =  \frac{{\mathbf{r}}_0 - {\mathbf{u}}({\mathbf{u}}^{\mathrm{T}} {\mathbf{r}}_0)}{r \sin \delta},  \quad {\mathbf{e}}_2  =  \frac{\tilde{{\mathbf{u}}} {\mathbf{r}}_0}{r \sin \delta}, \quad \mathrm{and} \quad {\mathbf{e}}_3  =   {\mathbf{u}} .


The coefficients can be calculated as follows,

.. math::

   c_1 = r \sin \delta \cos \varphi, \quad c_2 = r \sin \delta \sin \varphi, \quad \mathrm{and} \quad c_3 = r \cos \delta


Putting everything together gives

.. math::

   {\mathbf{r}} = \cos \varphi \, {\mathbf{r}}_0 + \sin \varphi \, \tilde{{\mathbf{u}}} \, {\mathbf{r}}_0 + (1 - \cos \varphi) \, {\mathbf{u}} \, ({\mathbf{u}}\tp \, {\mathbf{r}}_0)


From the relation \ :math:`{\mathbf{r}} = \Rot({\mathbf{u}} ,\varphi) {\mathbf{r}}_0`\ , we can reinterpret the \ **rotation tensor**\  \ :math:`\Rot`\ 

.. math::

   \Rot({\mathbf{u}} ,\varphi) = \cos \varphi \, {\mathbf{I}} + \sin \varphi \, \tilde{{\mathbf{u}}} + (1-\cos \varphi) \, {\mathbf{u}} \, {\mathbf{u}}\tp


With the additional relations

.. math::

   {\mathbf{u}} \, {\mathbf{u}}\tp=\tilde{\mathbf{u}} \, \tilde{\mathbf{u}} +({\mathbf{u}}\tp {\mathbf{u}}) {\mathbf{I}} \quad \mathrm{and} \quad {\mathbf{u}}\tp {\mathbf{u}} = 1


we finally obtain

.. math::

   \Rot({\mathbf{u}} ,\varphi) =  {\mathbf{I}} + \sin \varphi \, \tilde{{\mathbf{u}}} + (1-\cos \varphi) \,  \tilde{{\mathbf{u}}} \, \tilde{{\mathbf{u}}}


which is also known as the \ **Rodrigues formula**\  for rotations. 
While this formula is coordinate-free, it may be represented in any coordinate system, such as

.. math::

   \LU{0}{{\mathbf{r}}}(t)=\LU{0}{\Rot}(t) \, \LUR{0}{{\mathbf{r}}}{0} \qquad \text{with} \qquad \LU{0}{\Rot}(t)=\Rot(\LU{0}{{\mathbf{u}}}(t),\varphi(t))

    


Rotation vector
^^^^^^^^^^^^^^^

Using the rotation vector \ :math:`{\mathbf{v}}_\mathrm{rot} = \varphi\cdot {\mathbf{u}}`\  and \ :math:`\varphi = |{\mathbf{v}}_\mathrm{rot}|`\ , another representation of the rotation tensor follows as

.. math::

   \Rot({\mathbf{u}} ,\varphi) =  {\mathbf{I}} + \frac{\sin \varphi}{\varphi} \tilde{\mathbf{v}}_\mathrm{rot} + \frac{(1-\cos \varphi)}{\varphi^2} \,  \tilde{\mathbf{v}}_\mathrm{rot} \, \tilde{\mathbf{v}}_\mathrm{rot}


Note, that we inherently assume that \ :math:`\varphi \in [0,\pi]`\ .
In Exudyn, there are the following \ **functions and items related to the rotation vector**\  and the axis-angle representation:

+  \ ``NodeRigidBodyRotVecLG``\ : A 3D rigid body node based on rotation vector and Lie group methods; can be used for explicit integration methods, not leading to singularities when integrating with Lie-group methods
+  \ ``RotationVector2RotationMatrix``\ : computes the rotation matrix from a given rotation vector \ :math:`{\mathbf{v}}_\mathrm{rot}`\ 
+  \ ``RotationMatrix2RotationVector``\ : computes the rotation vector \ :math:`{\mathbf{v}}_\mathrm{rot}`\  from a given rotation matrix, based on a reconstruction of Euler parameters
+  \ ``ComputeRotationAxisFromRotationVector``\ : computes the rotation axis \ :math:`{\mathbf{u}}`\  from the rotation vector by using \ :math:`\varphi = |{\mathbf{v}}_\mathrm{rot}|`\ 

Note the following \ **properties of the rotation tensor**\ :

+  The axis-angle representations \ :math:`({\mathbf{u}}, \varphi)`\  and \ :math:`(-{\mathbf{u}}, -\varphi)`\  represent the same tensor \ :math:`\Rot({\mathbf{u}},\varphi)=\Rot(-{\mathbf{u}},-\varphi)`\ 
+  The rotation tensor \ :math:`\Rot`\  is orthogonal, i.e., \ :math:`\Rot\tp \, \Rot = {\mathbf{E}}`\ . 
+  The inverse rotation tensor \ :math:`\Rot^{-1} = \Rot\tp`\  is represented by the identical axis-angle tuples \ :math:`({\mathbf{u}}, -\varphi)`\  and \ :math:`(-{\mathbf{u}}, \varphi)`\ , leading to \ :math:`{\mathbf{r}}_0=\Rot({\mathbf{u}}, -\varphi) {\mathbf{r}}`\ .
+  We furthermore not the identities: \ :math:`\Rot({\mathbf{u}}, -\varphi) \equiv \Rot(-{\mathbf{u}}, \varphi) \equiv \Rot\tp ({\mathbf{u}}, \varphi)`\ 
+  The rotation axis is invariant to the rotation: \ :math:`\Rot({\mathbf{u}}, \varphi) {\mathbf{u}} = {\mathbf{u}}`\ 




Euler angles: Tait-Bryan angles
-------------------------------

In the following section, we discuss rotation matrices built by Tait-Bryan angles, which are consecutive \ :math:`xyz`\ -rotations.
Note that there are many other representations of Euler angles, however, they are not implemented or used in Exudyn.
The output variable \ ``Rotation``\  gives Tait-Bryan angles, which is why it is important to define their properties.



.. _fig-theory-rotations-taitbryanangles:
.. figure:: ../theDoc/figures/theoryRotationsTaitBryanAngles.png
   :width: 240

   Definition of Tait-Bryan angles.



The transformation given by Tait-Bryan angles \ :math:`(\alpha,\beta,\gamma)`\  follows from three successive rotations,

.. math::

   \LU{0}{{\mathbf{r}}} = \LU{01}{\Rot}(\alpha) \, \LU{12}{\Rot}(\beta) \, \LU{23}{\Rot}(\gamma) \, \LU{3}{{\mathbf{r}}} \quad \mathrm{and} \quad \LU{0}{{\mathbf{r}}} = \LU{03}{\Rot}(\alpha,\beta,\gamma) \, \LU{3}{{\mathbf{r}}}


The \ **Tait-Bryan rotation matrix**\  is thus defined as

.. math::

   \LU{03}{\Rot}(\alpha,\beta,\gamma) = \mr{\co\beta \,\co\gamma}{-\co\beta\,\si\gamma}{\si\beta} {\co \alpha \,\si \gamma + \si \alpha \,\si \beta \,\co \gamma}{\co \alpha \,\co\gamma-\si\alpha\,\si\beta\,\si\gamma}{-\si\alpha\,\co\beta} {\si\alpha\,\si\gamma-\co\alpha\,\si\beta\,\co\gamma}{\si\alpha\,\co\gamma+\co\alpha\,\si\beta\,\si\gamma}{\co\alpha\,\co\beta}


While not fully obvious from the latter equations, we note that there is a singularity at \ :math:`|\beta| = \frac{\pi}{2}`\ , which causes any computations to stall whenever getting close enough to this value.

It is also possible to reconstruct Tait-Bryan angles from a given rotation matrix.
Assume, we have the rotation matrix \ :math:`\LU{03}{\Rot}=(A_{ij})`\ .
There are two solutions for \ :math:`\beta`\  which follow from

.. math::

   \cos \beta = \pm \sqrt{1-A_{13}^2}, \quad \mathrm{and} \quad \sin \beta = A_{13}


For beta fulfilling \ :math:`|\beta| \ne \frac{\pi}{2}`\ , we can uniquely compute \ :math:`\gamma`\  and \ :math:`\alpha`\ ,

.. math::

   \cos \gamma = \frac{A_{11}}{\cos \beta}, \quad \sin \gamma = - \frac{A_{12}}{\cos \beta}, \quad \cos \alpha = \frac{A_{33}}{\cos \beta}, \quad \mathrm{and} \quad \sin \alpha = - \frac{A_{23}}{\cos \beta} .


Note that improvements are possible using the \ :math:`\mathrm{atan2}()`\  function.
We can finally resolve non-uniqueness by restricting \ :math:`\beta`\  within the range \ :math:`-\frac{\pi}{2} < \beta < \frac{\pi}{2}`\ .


Tait-Bryan angles: angular velocity vector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Angular velocities are essential for the formulation of equations of motion of rigid bodies, for constraints and for evaluation.
Therefore, basic relations are shown for Tait-Bryan angles, in particular the velocity transformation matrix \ :math:`{\mathbf{G}}_{TB}`\ .

The angular velocity vector \ :math:`\omega`\  can either be computed from the basic relation \ :math:`\dot \Rot = \tilde \omega \Rot`\ , which however involves many trigonometric terms, or from partial angular velocities, being part of the co-rotated rotation axes.
With the latter approach, we see that

.. math::

   \tomega_{30}=\dot{\alpha} {\mathbf{e}}_{x0}+\dot{\beta} {\mathbf{e}}_{y1}+\dot{\gamma} {\mathbf{e}}_{z2} \quad \text{or} \quad \tomega_{30}=\underbrace{[{\mathbf{e}}_{x0} \quad {\mathbf{e}}_{y1} \quad {\mathbf{e}}_{z2}]}_{{\mathbf{G}}_{TB}} \vr{\dot{\alpha}}{\dot{\beta}}{\dot{\gamma}}


We can now evaluate this equation in frame \ :math:`\mathcal{F}_0`\  with \ :math:`\tbeta=[\alpha\;\beta\;\gamma]\tp`\ , written as

.. math::

   \LUR{0}{\tomega}{30}=\underbrace{[\LUR{0}{{\mathbf{e}}}{x0} \quad \LUR{0}{{\mathbf{e}}}{y1} \quad \LUR{0}{{\mathbf{e}}}{z2}]}_{\LUR{0}{{\mathbf{G}}}{TB}} \vr{\dot{\alpha}}{\dot{\beta}}{\dot{\gamma}} = \LUR{0}{{\mathbf{G}}}{TB}(\tbeta) \dot{\tbeta} .


Evaluating the consecutive rotations, we find the respective unit vectors as

.. math::

   \LUR{0}{{\mathbf{e}}}{x0} = \vr{1}{0}{0}, \quad \LUR{0}{{\mathbf{e}}}{y1} = \vr{0}{\text{c} \alpha}{\text{s} \alpha}, \quad \mathrm{and} \quad \LUR{0}{{\mathbf{e}}}{z2} = \vr{\text{s} \beta}{-\text{s} \alpha \text{c} \beta}{\text{c} \alpha \text{c} \beta} .


which results in the relations for the (global) angular velocity vector

.. math::
   :label: eq-theory-rotations-taitbryang

   \vr{\LUR{0}{\omega}{30x}}{\LUR{0}{\omega}{30y}}{\LUR{0}{\omega}{30z}} = \mr{1}{0} {\text{s} \beta} {0}{\text{c} \alpha} {-\text{s} \alpha \text{c} \beta} {0}{\text{s} \alpha} {\text{c} \alpha \text{c} \beta} \vr{\dot{\alpha}}{\dot{\beta}}{\dot{\gamma}}


The matrix \ :math:`\LUR{0}{{\mathbf{G}}}{TB}`\  is used to compute partial derivatives of the angular velocity vector w.r.t. rotations.
However, it is also used to project torques and angular velocities into a space given by \ :math:`{\mathbf{G}}_{TB}^\mathrm{T}`\  (which is indeed not equivalent to \ :math:`{\mathbf{G}}_{TB}^{-1}`\ , but gives a symmetric mass matrix -- see the equations of motion for the rigid body).

The inverse relation between \ :math:`\dot \tbeta`\  and \ :math:`\LUR{0}{\tomega}{30}`\  gives

.. math::
   :label: eq-theory-rotations-taitbryanginv

   \vr{\dot{\alpha}}{\dot{\beta}}{\dot{\gamma}} = \frac{1}{\text{c} \beta} \mr{\text{c} \beta}{\text{s} \alpha \, \text{s} \beta}{-\text{c} \alpha \, \text{s} \beta}{0}{\text{c} \alpha \, \text{c} \beta}{\text{s} \alpha \text{c} \beta}{0}{-\text{s} \alpha}{\text{c} \alpha}  \vr{\LUR{0}{\omega}{30x}}{\LUR{0}{\omega}{30y}}{\LUR{0}{\omega}{30z}}


or in compact form

.. math::

   \dot{\tbeta} = \LURU{0}{{\mathbf{G}}}{TB}{-1}(\tbeta)  \LUR{0}{\tomega}{30}


We again see that for the case \ :math:`\vert \beta \vert = \pi/2`\  the matrix\ :math:`\LUR{0}{{\mathbf{G}}}{TB}`\  becomes singular, and Eq. :eq:`eq-theory-rotations-taitbryanginv`\  cannot be resolved for \ :math:`\dot{\tbeta}`\ !

Finally, we also provide the relations of \ :math:`\tomega_{30}`\  in body-fixed (local) coordinates \ :math:`\mathcal{F}_3`\ ,
which are frequently used in the implementation,

.. math::

   \vr{\LUR{3}{\omega}{30x}}{\LUR{3}{\omega}{30y}}{\LUR{3}{\omega}{30z}} = \mr{\text{c} \beta \, \text{c} \gamma}{\text{s} \gamma}{0}{-\text{c} \beta \, \text{s} \gamma}{\text{c} \gamma}{0}{\text{s} \beta}{0}{1} \vr{\dot{\alpha}}{\dot{\beta}}{\dot{\gamma}} ,


which reads in compact form (\ :math:`\LUR{3}{{\mathbf{G}}}{TB} = \LUR{b}{{\mathbf{G}}}{TB}`\ ),

.. math::

   \LUR{3}{\tomega}{30} =  \LUR{3}{{\mathbf{G}}}{TB}(\tbeta) \dot{\tbeta}



In Exudyn, there are the following \ **functions and items related to Tait-Bryan angles**\ :

+  \ ``NodeRigidBodyRxyz``\ : A 3D rigid body node based on Tait-Bryan angles
+  \ ``RotXYZ2RotationMatrix``\ : computes the rotation matrix from given Tait-Bryan angles
+  \ ``RotationMatrix2RotXYZ``\ : computes Tait-Bryan angles from a given rotation matrix
+  \ ``RotXYZ2G``\ : computes the matrix \ :math:`\LU{0}{{\mathbf{G}}}`\  relating time derivatives of Tait-Bryan angles to the (global) angular velocity vector
+  \ ``RotXYZ2Glocal``\ : computes the matrix \ :math:`\LU{b}{{\mathbf{G}}}`\  relating time derivatives of Tait-Bryan angles to the (local, body-fixed) angular velocity vector





Euler parameters and unit quaternions
-------------------------------------


As one of the most important forms of rotational parameters for multibody systems, the 4 Euler parameters are introduced.
They can be mathematically represented as unit quaternions and provide particularly simple expressions and equations of motion, albeit at the expense of an additional constraint.

The Euler parameters are defined as

.. math::
   :label: eq-theory-rotations-eulerparametersdef

   \underline{{\mathbf{p}}}=\vp{p_{\mathrm{s}}}{{\mathbf{p}}}=\vp{\cos \frac \varphi 2}{{\mathbf{u}} \, \sin \frac\varphi 2} \quad \text{bzw.} \quad \underline{{\mathbf{p}}}=\vfour{p_{\mathrm{s}}}{p_x}{p_y}{p_z}=\vfour{\cos \frac\varphi 2}{u_x \, \sin \frac\varphi 2}{u_y \, \sin \frac\varphi 2}{u_z \, \sin \frac\varphi 2}


Here, \ :math:`p_s=\cos \frac\varphi 2`\  is the scalar part and \ :math:`{\mathbf{p}} = {\mathbf{u}} \, \sin \frac\varphi 2`\  is the vector part of the Euler parameters \ :math:`\underline{{\mathbf{p}}}`\ .
The four Euler parameters \ :math:`\underline{{\mathbf{p}}}`\  are subject to the normalization condition

.. math::

   \phi(\underline{{\mathbf{p}}}) \equiv p_s^2+p_x^2+p_y^2+p_z^2-1=0 \quad \mathrm{or} \quad  p_s^2 +{\mathbf{p}}\tp \, {\mathbf{p}} -1 = 0 .


First, the following trigonometric transformations are introduced,

.. math::

   \cos \varphi = 2 \cos^2\frac{\varphi}{2} -1, \quad \sin \varphi = 2 \sin \frac \varphi 2 \cos \frac \varphi 2, \quad 1-\cos \varphi = 2 \sin^2 \frac{\varphi}{2} .


When these are substituted into the rotation tensor using Eq. :eq:`eq-theory-rotations-eulerparametersdef`\ , it follows

.. math::

   \Rot(\underline{{\mathbf{p}}})=(2 p_{\mathrm{s}}^2-1) \, {\mathbf{E}} + 2 p_{\mathrm{s}} \, \tilde{{\mathbf{p}}} + 2 \, {\mathbf{p}} \, {\mathbf{p}}\tp .



Euler parameters can initially be expressed in the global reference frame \ :math:`\mathcal{F}_0`\ ,

.. math::

   \LU{0}{\underline{{\mathbf{p}}}}=\vp{p_{\mathrm{s}}}{\LU{0}{{\mathbf{p}}}}=\vp{\cos \frac\varphi 2}{\LU{0}{{\mathbf{u}}} \, \sin \frac{\varphi} {2}} .


Thus, the coordinate representation of the rotation tensor is given by

.. math::
   :label: eq-theory-rotations-eulerparametersrot

   \Rot(\LU{0}{\underline{{\mathbf{p}}}}) = 2 \LU{0}{\mr{\ps^2+p_x^2-\frac 1 2}{p_x p_y-\ps p_z}{p_x p_z+\ps p_y} {p_x p_y+\ps p_z}{\ps^2+p_y^2-\frac 1 2}{p_y p_z-\ps p_x} {p_x p_z-\ps p_y}{p_y p_z+\ps p_x}{\ps^2+p_z^2-\frac{1}{2} }} = \LU{01}{\Rot}


Note that the axis of rotation is invariant to the rotation, \ :math:`\LU{0}{{\mathbf{u}}} = \LU{1}{{\mathbf{u}}}`\ , thus \ :math:`\LU{0}{\underline{{\mathbf{p}}}} = \LU{1}{\underline{{\mathbf{p}}}}`\ 
Due to the ambiguity \ :math:`\Rot(\underline{{\mathbf{p}}}) = \Rot(-\underline{{\mathbf{p}}})`\ , the scalar part of the Euler parameters is usually normalized, i.e., \ :math:`p_s \ge 0`\ .

The careful reader may observe that matrix \ :math:`\Rot`\  in Eq. :eq:`eq-theory-rotations-eulerparametersrot`\  is not identical to the implementation in Exudyn.
This is true, because there is always an alternative formulation for the diagonal terms by adding the normalization condition times a factor.


Quaternion operations
^^^^^^^^^^^^^^^^^^^^^

Calculations with Euler parameters are efficiently performed through the rules of quaternion algebra.
Active perspective as vector rotation: The rotation of vector \ :math:`{\mathbf{r}}_0`\  into \ :math:`{\mathbf{r}}(t)`\ ,

.. math::
   :label: eq-theory-rotations-quaternionrotation

   \LU{0}{{\mathbf{r}}}(t)=\LU{0}{\Rot}(t) \, \LUR{0}{{\mathbf{r}}}{0} \quad \text{mit} \quad \LU{0}{\Rot}(t)= \Rot(\LU{0}{{\mathbf{u}}}(t), \varphi (t))


is formulated using the quaternion \ :math:`\LU{0}{\underline{{\mathbf{p}}}}=\underline{{\mathbf{p}}}(\LU{0}{{\mathbf{u}}}, \varphi)`\  and its conjugate \ :math:`\myoverline{\underline{{\mathbf{p}}}}`\  with the help of the double quaternion product (operator \ :math:`\circ`\ ),

.. math::

   \LU{0}{\underline{{\mathbf{r}}}}(t) = \LU{0}{\underline{{\mathbf{p}}}}(t) \, \circ \, \LUR{0}{\underline{{\mathbf{r}}}}{0} \, \circ \, \LU{0}{\underline{\myoverline{{\mathbf{p}}}}}(t) ,


or in short form,

.. math::

   \vp{0}{\LU{0}{{\mathbf{r}}}(t)}=\vp{p_{\mathrm{s}}(t)}{\LU{0}{{\mathbf{p}}}(t)} \, \circ \, \vp{0}{\LUR{0}{{\mathbf{r}}}{0}} \, \circ \, \vp{p_{\mathrm{s}}(t)}{-\LU{0}{{\mathbf{p}}}(t)} .


With the multiplication rule for quaternions, the scalar part yields \ :math:`0=0`\  and the vector part the vector rotation \ :eq:`eq-theory-rotations-quaternionrotation`\ . 
For multiple rotations, we have

.. math::

   \LUR{0}{\underline{{\mathbf{r}}}}{2}=\LUR{0}{\underline{{\mathbf{p}}}}{2} \circ \LUR{0}{\underline{{\mathbf{p}}}}{1} \circ \LUR{0}{\underline{{\mathbf{r}}}}{0} \circ  \LUR{0}{\myoverline{\underline{{\mathbf{p}}}}}{1} \circ \LUR{0}{\myoverline{\underline{{\mathbf{p}}}}}{2} .


Given two quaternions

.. math::

   \underline{{\mathbf{a}}} = a_\mathrm{s} + \mathrm{i}\, a_x + \mathrm{j}\, a_y + \mathrm{k}\, a_z, \quad \mathrm{and} \quad \underline{{\mathbf{b}}} = b_\mathrm{s} + \mathrm{i}\, b_x + \mathrm{j}\, b_y + \mathrm{k}\, b_z ,


we provide two important \ **rules of quaternion algebra**\ , which is the sum,

.. math::

   \underline{{\mathbf{c}}} = \underline{{\mathbf{a}}} + \underline{{\mathbf{b}}} \quad \ra \quad \vp{c_\mathrm{s}}{{\mathbf{c}}} = \vp{a_\mathrm{s}}{{\mathbf{a}}} + \vp{b_\mathrm{s}}{{\mathbf{b}}} = \vp{a_\mathrm{s} + b_\mathrm{s}}{{\mathbf{a}} + {\mathbf{b}}} ,


and the multiplication

.. math::

   \underline{{\mathbf{c}}} = \underline{{\mathbf{a}}} \circ \underline{{\mathbf{b}}} \neq \underline{{\mathbf{b}}} \circ \underline{{\mathbf{a}}} \quad \ra \quad \vp{c_\mathrm{s}}{{\mathbf{c}}} = \vp{a_\mathrm{s}}{{\mathbf{a}}} \circ \vp{b_\mathrm{s}}{{\mathbf{b}}} = \vp{a_\mathrm{s} b_\mathrm{s} - {\mathbf{a}}^\mathrm{T} {\mathbf{b}}}{a_\mathrm{s}{\mathbf{b}} + b_\mathrm{s}{\mathbf{a}} + \tilde {\mathbf{a}} {\mathbf{b}}} .





Angular velocity vector
^^^^^^^^^^^^^^^^^^^^^^^

To derive the angular velocity from Euler parameters, there are both the possibilities of deriving the rotation matrix with respect to time and directly deriving the angular velocity from Euler parameters.

For an infinitesimal rotation \ :math:`\dd \psi`\ , the quaternion follows

.. math::

   \underline{{\mathbf{p}}}({\mathbf{e}}, \dd \psi) = \vp{\cos \frac{\dd \psi}{2}}{{\mathbf{e}} \sin \frac{\dd\psi}{2}} \approx \vp{1}{{\mathbf{e}} \frac{\dd\psi}{2}}


from which we conclude that from \ :math:`\underline{{\mathbf{p}}}({\mathbf{e}}, \dd \psi) \, \circ \, \underline{{\mathbf{p}}}(t) = \underline{{\mathbf{p}}}(t) + \dd\underline{{\mathbf{p}}}`\  and thus \ :math:`\dd\underline{{\mathbf{p}}} = \vp{0}{{\mathbf{e}} \frac{\dd\psi}{2}}`\ .
Dividing by \ :math:`\dd t`\  and with the angular velocity \ :math:`\tomega_{10} = {\mathbf{e}} \, \dot \psi`\ , it follows

.. math::

   \vp{\dot \ps}{\LLdot{0}{{\mathbf{p}}}{}} = \frac{1}{2} \vp{0}{\tomega_{10} }  \circ \vp{\ps}{{\mathbf{p}}}


or in short form

.. math::

   \LU{0}{\underline{\dot {\mathbf{p}}}} = \frac{1}{2} \underline{\tomega}_{10}  \circ \underline{{\mathbf{p}}}


With the quaternion product in matrix notation, it follows

.. math::

   \vp{\dot \ps}{\LU{0}{\dot{\mathbf{p}}}{}} = \frac{1}{2} \mp{\ps}{-{\mathbf{p}}^\mathrm{T}}{{\mathbf{p}}}{\ps {\mathbf{E}} - \tilde {\mathbf{p}}} \vp{0}{\tomega_{10} }


or with the matrix \ :math:`\underline{{\mathbf{P}}}`\  it can be written as

.. math::

   \LU{0}{\underline{\dot {\mathbf{p}}}}{} = \frac{1}{2} {\underline{{\mathbf{P}}}}({\mathbf{p}}) \underline{\tomega}_{10}


This form can be represented in global coordinates due to \ :math:`{\underline{{\mathbf{P}}}}^{-1} = {\underline{{\mathbf{P}}}}\tp`\  as

.. math::

   \vp{0}{\LU{0}{\tomega}_{10} } = 2 \mp{\ps}{\LU{0}{{\mathbf{p}}}^\mathrm{T}}{-\LU{0}{{\mathbf{p}}}}{\ps {\mathbf{E}} + \LU{0}{\tilde {\mathbf{p}}}} \, \vp{\dot \ps}{\LLdot{0}{{\mathbf{p}}}{}}


Thus, the velocity transformation follows to

.. math::

   \LUR{0}{{\mathbf{G}}}{EP} = \left[-2\LU{0}{{\mathbf{p}}}, \; 2\ps {\mathbf{E}} + 2\LU{0}{\tilde {\mathbf{p}}}\right]


as a matrix with 3 rows and 4 columns. With \ :math:`\LUR{0}{{\mathbf{G}}}{EP}`\ , the angular velocity vector can be conveniently calculated,

.. math::

   \LU{0}{\tomega}_{10} = \left[-2\LU{0}{{\mathbf{p}}}, \; 2\ps {\mathbf{E}} + 2\LU{0}{\tilde {\mathbf{p}}}\right] \, \LU{0}{\underline{\dot {\mathbf{p}}}}



In Exudyn, there are the following \ **functions and items related to Euler parameters**\ :

+  \ ``NodeRigidBodyEP``\ : A 3D rigid body node based on Euler parameters
+  \ ``EulerParameters2RotationMatrix``\ : computes the rotation matrix from given Euler parameters
+  \ ``RotationMatrix2EulerParameters``\ : computes Euler parameters from a given rotation matrix
+  \ ``EulerParameters2G``\ : computes the matrix \ :math:`\LUR{0}{{\mathbf{G}}}{EP}`\  relating time derivatives of Euler parameters to the (global) angular velocity vector
+  \ ``EulerParameters2Glocal``\ : computes the matrix \ :math:`\LUR{b}{{\mathbf{G}}}{EP}`\  relating time derivatives of Euler parameters to the (local, body-fixed) angular velocity vector




