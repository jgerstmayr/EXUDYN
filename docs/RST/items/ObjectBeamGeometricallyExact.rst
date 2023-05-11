

.. _sec-item-objectbeamgeometricallyexact:

ObjectBeamGeometricallyExact
============================

OBJECT UNDER CONSTRUCTION: A 3D geometrically exact beam finite element, currently using two 3D rigid body nodes. The localPosition \ :math:`x`\  of the beam ranges from \ :math:`-L/2`\  (at node 0) to \ :math:`L/2`\  (at node 1). The axial coordinate is \ :math:`x`\  (first coordinate) and the cross section is spanned by local \ :math:`y`\ /\ :math:`z`\  axes.

\ **Additional information for ObjectBeamGeometricallyExact**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Body``\ , \ ``MultiNoded``\ 
* | Requested \ ``Node``\  type = \ ``Position``\  + \ ``Orientation``\ 
* | \ **Short name**\  for Python = \ ``Beam3D``\ 
* | \ **Short name**\  for Python visualization object = \ ``VBeam3D``\ 


The item \ **ObjectBeamGeometricallyExact**\  with type = 'BeamGeometricallyExact' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **nodeNumbers** [type = NodeIndex2, default = [invalid [-1], invalid [-1]]]:
  | two node numbers for beam element
* | **physicsLength** [\ :math:`L`\ , type = PReal, default = 0.]:
  | [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives \ :math:`\rho A L`\ ; must be positive
* | **sectionData** [type = BeamSection, default = BeamSection()]:
  | data as given by exudyn.BeamSection(), defining inertial, stiffness and damping parameters of beam section.
* | **visualization** [type = VObjectBeamGeometricallyExact]:
  | parameters for visualization of item



The item VObjectBeamGeometricallyExact has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown; geometry is defined by sectionGeometry
* | **sectionGeometry** [type = BeamSectionGeometry, default =  BeamSectionGeometry()]:
  | defines cross section shape used for visualization and contact
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA color of the object; if R==-1, use default color


----------

.. _description-objectbeamgeometricallyexact:

DESCRIPTION of ObjectBeamGeometricallyExact
-------------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : 
  | global position vector of local axis (1) and cross section (2) position
* | ``Displacement``\ : 
  | global displacement vector of local axis (1) and cross section (2) position
* | ``Velocity``\ : 
  | global velocity vector of local axis (1) and cross section (2) position
* | ``Rotation``\ : 
  | 3D Tait-Bryan rotation components, containing rotation around \ :math:`z`\ -axis only
* | ``StrainLocal``\ : 
  | 6 strain components, containing only axial (\ :math:`xx`\ ) and shear strain (\ :math:`xy`\ )
* | ``CurvatureLocal``\ : 
  | 3D vector of curvature, containing only curvature w.r.t. \ :math:`z`\ -axis


Detailed description coming later.


Relevant Examples and TestModels with weblink:

    \ `geometricallyExactBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/geometricallyExactBeamTest.py>`_\  (TestModels/), \ `rightAngleFrame.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rightAngleFrame.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


