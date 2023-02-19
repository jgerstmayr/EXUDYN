

.. _sec-item-objectbeamgeometricallyexact:

ObjectBeamGeometricallyExact
============================

OBJECT UNDER CONSTRUCTION: A 3D geometrically exact beam finite element, currently using two 3D rigid body nodes. The localPosition \ :math:`x`\  of the beam ranges from \ :math:`-L/2`\  (at node 0) to \ :math:`L/2`\  (at node 1). The axial coordinate is \ :math:`x`\  (first coordinate) and the cross section is spanned by local \ :math:`y`\ /\ :math:`z`\  axes.
 



The item \ **ObjectBeamGeometricallyExact**\  with type = 'BeamGeometricallyExact' has the following parameters:

 

* | **name** [type = String, default = '']:
  | objects's unique name
* | **nodeNumbers** [type = NodeIndex2, default = [invalid [-1], invalid [-1]]]:
  | two node numbers for beam element
* | **physicsLength** [type = PReal, default = 0.]:
  | [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives \ :math:`\rho A L`\ ; must be positive
* | **sectionData** [type = BeamSection, default = BeamSection()]:
  | data as given by exudyn.BeamSection(), defining inertial, stiffness and damping parameters of beam section.



The item VObjectBeamGeometricallyExact has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown; geometry is defined by sectionGeometry
* | **sectionGeometry** [type = BeamSectionGeometry, default =  BeamSectionGeometry()]:
  | defines cross section shape used for visualization and contact
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA color of the object; if R==-1, use default color




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


