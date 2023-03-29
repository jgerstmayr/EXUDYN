.. _sec-theory:


***********************
Theory and formulations
***********************


This section includes some material on formulations and general theoretical backgrounds for Exudyn .
Note that the formulation of nodes, objects, markers, etc. is presented right at the subsection of each object, see Section :ref:`sec-item-reference-manual`\ .
Furthermore, the general notations are described in Section :ref:`sec-generalnotation`\  and an overview on the system assembly and system equations of motion is given in Section :ref:`sec-generalnotation`\ , solvers are described in Section :ref:`sec-solvers`\ .

Reference and current coordinates
---------------------------------

An important fact on the coordinates used in Exudyn is upon the \ **additive**\ \ (This additive splitting is also used for rotations: therefore, only the sum of reference and current (or visualization) coordinates has a geometrical meaning, while the parts are only used within the solver for incrementing.) splitting of quantities (e.g. position, rotation parameters, etc.) into \ **reference**\  and \ **current**\  (initial/visualization/...) coordinates.
The current position vector of a point node is computed from the reference position plus the current displacment, reading

.. math::

   {\mathbf{p}}\cCur = {\mathbf{p}}\cRef + {\mathbf{u}}\cCur


In the same way rotation parameters are computed from,

.. math::

   \ttheta\cCur = \ttheta\cRef + \tpsi\cCur


which are based on reference quantities plus displacements or changes. Note that these changes are additive, even for rotation parameters. Needless to say, \ :math:`\tpsi\cCur`\  do not represent rotation parameters, while \ :math:`\ttheta\cRef`\  should be chosen such that they represent the orientation of a node in reference configuration.
The necessity for reference coordinates originates from finite elements, which usually split nodal position into displacements and reference position.
However, we also use the reference position here in order to define joints, e.g., using the utility function \ ``AddRevoluteJoint(...)``\ .

Against to the splitting of positions, displacements and velocities (and most other quantities) are not having this reference part!


Reference point
---------------

In contrast to the reference position or reference coordinates, the reference point is mainly used for objects, e.g., rigid bodies.
The reference point is the position of the underlying (rigid body) node, while we can compute the position of any point on the body (or on the mass point).
The reference point is also the origin of the co-rotating (reference) frame with the body-fixed coordinate system.

The same concept is also used for \ :ref:`FFRF <FFRF>`\  objects. In most cases, reference points are denoted by \ :math:`\pRef`\ .


Coordinate Systems
------------------

The left indices provide information about the coordinate system, e.g.,

.. math::

   \LU{0}{{\mathbf{u}}}


is the displacement vector in the global (inertial) coordinate systme \ :math:`0`\ , while 

.. math::

   \LU{m1}{{\mathbf{u}}}


represents the displacement vector in marker 1 (\ :math:`m1`\ ) coordinates. Typical coordinate systems:

+  \ :math:`\LU{0}{{\mathbf{u}}}`\  \ :math:`\ldots`\  global coordinates
+  \ :math:`\LU{b}{{\mathbf{u}}}`\  \ :math:`\ldots`\  body-fixed, local coordinates
+  \ :math:`\LU{m0}{{\mathbf{u}}}`\  \ :math:`\ldots`\  local coordinates of (the body or node of) marker \ :math:`m0`\ 
+  \ :math:`\LU{m1}{{\mathbf{u}}}`\  \ :math:`\ldots`\  local coordinates of (the body or node of) marker \ :math:`m1`\ 
+  \ :math:`\LU{J0}{{\mathbf{u}}}`\  \ :math:`\ldots`\  local coordinates of joint \ :math:`0`\ , related to marker \ :math:`m0`\ 
+  \ :math:`\LU{J1}{{\mathbf{u}}}`\  \ :math:`\ldots`\  local coordinates of joint \ :math:`1`\ , related to marker \ :math:`m1`\ 

To transform the local coordinates \ :math:`\LU{m0}{{\mathbf{u}}}`\  of marker 0 into global coordinates \ :math:`\LU{0}{{\mathbf{x}}}`\ , we use

.. math::

   \LU{0}{{\mathbf{u}}} = \LU{0,m0}{\Rot} \LU{m0}{{\mathbf{u}}}


in which \ :math:`\LU{0,m0}{\Rot}`\  is the transformation matrix of (the body or node of) the underlying marker 0.


.. _sec-integrationpoints:


Integration Points
------------------

For several tasks, especially for finite elements and contact, different integration rules are used, which are summarized here.
The interval of all integration rules is \ :math:`\in [-1,1]`\ , thus giving a total sum for integration weights of 2.
The points \ :math:`\xi_{ip}`\  and weights \ :math:`w_{ip}`\  for Gauss rules read:

The following table collects some typical \ **input parameters**\  for nodes, objects and markers: 

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | \ **type/order**\ 
     - |  \ **point 0**\ 
     - |  \ **point 1**\ 
     - |  \ **point 2**\ 
     - |  \ **point 3**\  
   * - | Gauss 1
     - |  0
     - | 
     - | 
     - | 
   * - | Gauss 3
     - |  \ :math:`-\sqrt{1 / 3}`\ 
     - |  \ :math:`\sqrt{1 / 3}`\ 
     - | 
     - | 
   * - | Gauss 5
     - |  \ :math:`-\sqrt{3 / 5}`\ 
     - |  0
     - |  \ :math:`\sqrt{3 / 5}`\ 
     - | 
   * - | Gauss 7
     - |  \ :math:`-\sqrt{3 / 7 + \sqrt{120} / 35}`\ 
     - |  \ :math:`-\sqrt{3 / 7 - \sqrt{120} / 35}`\ 
     - |  \ :math:`\sqrt{3 / 7 - \sqrt{120} / 35}`\ 
     - |  \ :math:`\sqrt{3 / 7 + \sqrt{120} / 35}`\  
   * - | 
     - | 
     - | 
     - | 
     - | 
   * - | \ **type/order**\ 
     - |  \ **weight 0**\ 
     - |  \ **weight 1**\ 
     - |  \ **weight 2**\ 
     - |  \ **weight 3**\  
   * - | Gauss 1
     - |  2
     - | 
     - | 
     - | 
   * - | Gauss 3
     - |  1
     - |  1
     - | 
     - | 
   * - | Gauss 5
     - |  \ :math:`5 / 9`\ 
     - |  \ :math:`8 / 9`\ 
     - |  \ :math:`5 / 9`\ 
     - |  
   * - | Gauss 7
     - |  \ :math:`1 / 2 - 5 / (3 \sqrt{120})`\ 
     - |  \ :math:`1 / 2 + 5 / (3*\sqrt{120})`\ 
     - |  \ :math:`1 / 2 + 5 / (3*\sqrt{120})`\ 
     - |  \ :math:`1 / 2 - 5 / (3*\sqrt{120})`\  


The points \ :math:`\xi_{ip}`\  and weights \ :math:`w_{ip}`\  for Lobatto rules read: 

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | \ **type/order**\ 
     - |  \ **point 0**\ 
     - |  \ **point 1**\ 
     - |  \ **point 2**\ 
     - |  \ **point 3**\ 
   * - | Lobatto 1
     - |  -1
     - |  1
     - | 
     - | 
   * - | Lobatto 3
     - |  -1
     - |  0
     - |  1
     - | 
   * - | Lobatto 5
     - |  -1
     - |  \ :math:`-\sqrt{1/5}`\ 
     - |  \ :math:`\sqrt{1/5}`\ 
     - |  1
   * - | 
     - | 
     - | 
     - | 
     - | 
   * - | \ **type/order**\ 
     - |  \ **weight 0**\ 
     - |  \ **weight 1**\ 
     - |  \ **weight 2**\ 
     - |  \ **weight 3**\  
   * - | Lobatto 1
     - |  1
     - |  1
     - | 
     - | 
   * - | Lobatto 3
     - |  \ :math:`1/3`\ 
     - |  \ :math:`4/3`\ 
     - |  \ :math:`1/3`\ 
     - | 
   * - | Lobatto 5
     - |  \ :math:`1/6`\ 
     - |  \ :math:`5/6`\ 
     - |  \ :math:`5/6`\ 
     - |  \ :math:`1/6`\ 

Further integration rules can be found in the C++ code of Exudyn , see file \ ``BasicLinalg.h``\ .
\ **For further information on this topic read**\ : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_
