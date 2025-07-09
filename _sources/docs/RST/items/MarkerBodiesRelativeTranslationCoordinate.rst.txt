

.. _sec-item-markerbodiesrelativetranslationcoordinate:

MarkerBodiesRelativeTranslationCoordinate
=========================================

A coordinate-based Marker attached to two rigid bodies or beams which computes the relative translation between the bodies according to the given axis; this marker can be used together with coordinate-based constraints and connectors (e.g., CoordinateSpringDamper and CoordinateConstraint). Note that it is assumed that the two bodies can only move along the given axis (e.g., constrained by a prismatic joint) -- otherwise results may be unexpected. Note that this approach is not compatible with FFRF-based flexible bodies and currently requires and intermediate rigid body.

\ **Additional information for MarkerBodiesRelativeTranslationCoordinate**\ :

* | This \ ``Marker``\  has/provides the following types = \ ``Object``\ , \ ``Body``\ , \ ``Position``\ , \ ``Orientation``\ , \ ``Coordinate``\ 


The item \ **MarkerBodiesRelativeTranslationCoordinate**\  with type = 'BodiesRelativeTranslationCoordinate' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **bodyNumbers** [\ :math:`[b_0,b_1]\tp`\ , type = ArrayObjectIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of body numbers for which relative coordinate is computed
* | **localPosition0** [\ :math:`\LU{m_0}{{\mathbf{p}}}_0`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | local position on body 0; i.e. local (body-fixed) position where position is measured and force is applied to
* | **localPosition1** [\ :math:`\LU{m_1}{{\mathbf{p}}}_1`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | local position on body 1; i.e. local (body-fixed) position where position is measured and force is applied to
* | **axis0** [\ :math:`\LU{m_0}{{\mathbf{a}}}_0`\ , type = Vector3D, size = 3, default = [1.,0.,0.]]:
  | axis defined in body 0, along which the relative translation is measured
* | **offset** [\ :math:`x_\mathrm{off}`\ , type = Real, default = 0.]:
  | translation offset [SI:m] subtracted from the translation; can be used to change the zero position
* | **visualization** [type = VMarkerBodiesRelativeTranslationCoordinate]:
  | parameters for visualization of item



The item VMarkerBodiesRelativeTranslationCoordinate has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-markerbodiesrelativetranslationcoordinate:

DESCRIPTION of MarkerBodiesRelativeTranslationCoordinate
--------------------------------------------------------
The marker consists of two bodies, body \ :math:`b_0`\  and body \ :math:`b_1`\  with respective global marker positions \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\  and \ :math:`\LU{0}{{\mathbf{p}}}_{m1}`\ ,
depending on local positions \ :math:`\LU{m_0}{{\mathbf{p}}}_0`\  and \ :math:`\LU{m_1}{{\mathbf{p}}}_1`\ , 
and marker orientations \ :math:`\LU{0,m_0}{\Rot}_{m0}`\  and \ :math:`\LU{0,m_1}{\Rot}_{m1}`\ .
The global axis is computed as 

.. math::

   \LU{0}{{\mathbf{a}}}_0 = \LU{0,m_0}{\Rot}_{m0} \LU{m_0}{{\mathbf{a}}}_0


The relative translation marker computes the relative translation from the equation

.. math::

   t =  \LU{0}{{\mathbf{a}}}_0\tp ({\mathbf{p}}_{m1} - {\mathbf{p}}_{m0}) - x_\mathrm{off}


The translational velocity, which may be used in coordinate spring-dampers or for velocity-level constraints, is computed as

.. math::

   \dot t = \LU{0}{{\mathbf{a}}}_0\tp (\dot {\mathbf{p}}_{m1} - \dot {\mathbf{p}}_{m0}) + \LU{0}{\dot {\mathbf{a}}}_0\tp ({\mathbf{p}}_{m1} - {\mathbf{p}}_{m0})


Jacobians are computed according to the relative translational velocity, ignoring the \ :math:`\dot {\mathbf{a}}_0`\  part.
Using this approach, coordinate constraints can be added to mechanisms to purely add internal drives, not affecting global momenta.
Furthermore, coupling to a relative rotation marker MarkerBodiesRelativeRotationCoordinate can be used to 
create advanced mechanisms and gears.


Relevant Examples and TestModels with weblink:

    \ `involuteGearGraphics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/involuteGearGraphics.py>`_\  (Examples/), \ `relativeRotationTranslationMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/relativeRotationTranslationMechanism.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


