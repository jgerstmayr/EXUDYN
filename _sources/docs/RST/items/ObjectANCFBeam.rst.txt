

.. _sec-item-objectancfbeam:

ObjectANCFBeam
==============

OBJECT UNDER CONSTRUCTION: A 3D beam finite element based on the absolute nodal coordinate formulation, using two nodes. The localPosition \ :math:`x`\  of the beam ranges from \ :math:`-L/2`\  (at node 0) to \ :math:`L/2`\  (at node 1). The axial coordinate is \ :math:`x`\  (first coordinate) and the cross section is spanned by local \ :math:`y`\ /\ :math:`z`\  axes; assuming dimensions \ :math:`w_y`\  and \ :math:`w_z`\  in cross section, the local position range is \ :math:`\in [[-L/2,L/2],\, [-wy/2,wy/2],\, [-wz/2,wz/2] ]`\ .

\ **Additional information for ObjectANCFBeam**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Body``\ , \ ``MultiNoded``\ 
* | Requested \ ``Node``\  type = \ ``Position``\  + \ ``Orientation``\ 
* | \ **Short name**\  for Python = \ ``ANCFBeam``\ 
* | \ **Short name**\  for Python visualization object = \ ``VANCFBeam``\ 


The item \ **ObjectANCFBeam**\  with type = 'ANCFBeam' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **nodeNumbers** [type = NodeIndex2, default = [invalid [-1], invalid [-1]]]:
  | two node numbers for beam element
* | **physicsLength** [\ :math:`L`\ , type = PReal, default = 0.]:
  | [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives \ :math:`\rho A L`\ ; must be positive
* | **sectionData** [type = BeamSection, default = BeamSection()]:
  | data as given by exudyn.BeamSection(), defining inertial, stiffness and damping parameters of beam section.
* | **crossSectionPenaltyFactor** [\ :math:`f_{cs} = [f_{yy},\,f_{zz},\,f_{yz}]\tp`\ , type = Vector3D, default = [1.,1.,1.]]:
  | [SI:1] additional penalty factors for cross section deformation, which are in total \ :math:`k_{cs} = [f_{yy}\cdot k_{yy},\, f_{zz}\cdot k_{zz},\, f_{yz}\cdot k_{yz}]\tp`\ 
* | **visualization** [type = VObjectANCFBeam]:
  | parameters for visualization of item



The item VObjectANCFBeam has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown; geometry is defined by sectionGeometry
* | **sectionGeometry** [type = BeamSectionGeometry, default =  BeamSectionGeometry()]:
  | defines cross section shape used for visualization and contact
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA color of the object; if R==-1, use default color


----------

.. _description-objectancfbeam:

DESCRIPTION of ObjectANCFBeam
-----------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : 
  | global position vector of local position vector
* | ``Displacement``\ : 
  | global displacement vector of local position vector
* | ``Velocity``\ : 
  | global velocity vector of local position vector
* | ``VelocityLocal``\ : 
  | global velocity vector of local position vector
* | ``AngularVelocity``\ : 
  | global angular velocity vector of local (axis) position vector
* | ``AngularVelocityLocal``\ : 
  | local angular velocity vector of local (axis) position vector
* | ``Acceleration``\ : 
  | global acceleration vector of local position vector
* | ``Rotation``\ : 
  | 3D Tait-Bryan rotation components of cross section rotation
* | ``RotationMatrix``\ : 
  | rotation matrix of cross section rotation as 9D vector


Detailed description coming later.


Relevant Examples and TestModels with weblink:

    \ `ANCFBeamEigTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamEigTest.py>`_\  (TestModels/), \ `ANCFBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamTest.py>`_\  (TestModels/), \ `geometricallyExactBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/geometricallyExactBeamTest.py>`_\  (TestModels/), \ `rightAngleFrame.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rightAngleFrame.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


