

.. _sec-item-markerbodiesrelativerotationcoordinate:

MarkerBodiesRelativeRotationCoordinate
======================================

A coordinate-based Marker attached to two rigid bodies or beams which computes the relative rotation between the bodies according to the given axis; this marker can be used together with coordinate-based constraints and connectors (e.g., CoordinateSpringDamper and CoordinateConstraint). Note that it is assumed that the two bodies can only rotate about the given axis (e.g., constrained by a revolute joint) -- otherwise results may be unexpected. Note that this approach is not compatible with FFRF-based flexible bodies and currently requires and intermediate rigid body.

\ **Additional information for MarkerBodiesRelativeRotationCoordinate**\ :

* | This \ ``Marker``\  has/provides the following types = \ ``Node``\ , \ ``Object``\ , \ ``Body``\ , \ ``Position``\ , \ ``Orientation``\ , \ ``Coordinate``\ 


The item \ **MarkerBodiesRelativeRotationCoordinate**\  with type = 'BodiesRelativeRotationCoordinate' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **bodyNumbers** [\ :math:`[b_0,b_1]\tp`\ , type = ArrayObjectIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of body numbers for which relative coordinate is computed
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number of NodeGenericData with 1 coordinate which contains previous angle for continuation of angles (initialize accordingly if needed); if node is not supplied, angles will have jump outside \ :math:`\pm \pi`\ 
* | **localPosition0** [\ :math:`\LU{m_0}{{\mathbf{p}}}_0`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | local position on body 0; i.e. local (body-fixed) position where position is measured and force is applied to
* | **localPosition1** [\ :math:`\LU{m_1}{{\mathbf{p}}}_1`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | local position on body 1; i.e. local (body-fixed) position where position is measured and force is applied to
* | **axis0** [\ :math:`\LU{m_0}{{\mathbf{a}}}_0`\ , type = Vector3D, size = 3, default = [1.,0.,0.]]:
  | axis defined in body 0, along which the relative rotation is measured
* | **offset** [\ :math:`x_\mathrm{off}`\ , type = Real, default = 0.]:
  | rotation offset [SI:1] subtracted from the measured rotation; can be used to change the zero rotation
* | **visualization** [type = VMarkerBodiesRelativeRotationCoordinate]:
  | parameters for visualization of item



The item VMarkerBodiesRelativeRotationCoordinate has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-markerbodiesrelativerotationcoordinate:

DESCRIPTION of MarkerBodiesRelativeRotationCoordinate
-----------------------------------------------------
The marker consists of two bodies, body \ :math:`b_0`\  and body \ :math:`b_1`\  with respective global marker positions \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\  and \ :math:`\LU{0}{{\mathbf{p}}}_{m1}`\ ,
depending on local positions \ :math:`\LU{m_0}{{\mathbf{p}}}_0`\  and \ :math:`\LU{m_1}{{\mathbf{p}}}_1`\ , 
and marker orientations \ :math:`\LU{0,m_0}{\Rot}_{m0}`\  and \ :math:`\LU{0,m_1}{\Rot}_{m1}`\ .
From the given axis \ ``axis0``\ , we compute an orthonormal basis (orthonormal to axis0) relative to marker \ :math:`m_0`\ ,

.. math::

   \LU{m_0,b}{\Rot}_b = \left[\LU{m_0}{{\mathbf{x}}}_b, \LU{m_0}{{\mathbf{y}}}_b, \LU{m_0}{{\mathbf{a}}}_0\right]


The relative rotation marker computes the relative rotation according to

.. math::

   \LU{m_0,m_1}{\Rot}_{rel} = \LU{m_0,0}{\Rot}_{m0} \LU{0,m_1}{\Rot}_{m1}


This relative rotation, which represents a rotation about axis \ :math:`\LU{m_0}{{\mathbf{a}}}_0`\  is then transformed into the orthonormal basis,

.. math::

   \LU{b}{\Rot}_{rel} = \LU{b,m_0}{\Rot}_b \LU{m_0,m_1}{\Rot}_{rel} \LU{m_0,b}{\Rot}_b


and contains the desired rotation about the z-axis, which can be extracted as

.. math::

   \varphi = \mathrm{atan2}(\LU{b}{\Rot}_{rel}[1,0],\LU{b}{\Rot}_{rel}[0,0]) - x_\mathrm{off}


The global axis is computed as 

.. math::

   \LU{0}{{\mathbf{a}}}_0 = \LU{0,m_0}{\Rot}_{m0} \LU{m_0}{{\mathbf{a}}}_0

    
Using the angular velocities at the two bodies, \ :math:`\LU{m_0}{\tomega_0}`\  and  \ :math:`\LU{m_1}{\tomega_1}`\ , the relative angular velocity, which may be used in coordinate spring-dampers or for velocity-level constraints, 
is simply computed as

.. math::

   \dot \varphi = \LU{0}{{\mathbf{a}}}_0\tp \left( \LU{0,m_1}{\Rot}_{m1} \LU{m_1}{\tomega_1} - \LU{0,m_0}{\Rot}_{m0} \LU{m_0}{\tomega_0} \right)


Jacobians are computed according to the relative rotation velocity.
Using this approach, coordinate constraints can be added to mechanisms to purely add internal drives, not affecting global momenta.
Furthermore, coupling to a relative translation can be used to create advanced mechanisms and gears.


Relevant Examples and TestModels with weblink:

    \ `involuteGearGraphics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/involuteGearGraphics.py>`_\  (Examples/), \ `relativeRotationTranslationMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/relativeRotationTranslationMechanism.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


