


.. _sec-beamsectiongeometry:

BeamSectionGeometry
-------------------

Data structure for definition of 2D and 3D beam (cross) section geometrical properties. Used for visualization and contact.

BeamSectionGeometry has the following items:

* | **crossSectionRadiusY** [type = UReal, default = 0.]:
  | \ :math:`c_Y\,`\  [SI:m] \ :math:`Y`\  radius for circular cross section
* | **crossSectionRadiusZ** [type = UReal, default = 0.]:
  | \ :math:`c_Z\,`\  [SI:m] \ :math:`Z`\  radius for circular cross section
* | **crossSectionType** [type = CrossSectionType, default = CrossSectionType::Polygon]:
  | \tabnewline Type of cross section: Polygon, Circular, etc.
* | **polygonalPoints** [type = Vector2DList]:
  | \ :math:`{\mathbf{p}}_pg\,`\  [SI: (m,m) ] list of polygonal (\ :math:`Y,Z`\ ) points in local beam cross section coordinates, defined in positive rotation direction

