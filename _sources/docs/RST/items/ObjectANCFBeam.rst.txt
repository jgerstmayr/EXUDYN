

.. _sec-item-objectancfbeam:

ObjectANCFBeam
==============

OBJECT UNDER CONSTRUCTION: A 3D beam finite element based on the absolute nodal coordinate formulation, using two . The localPosition \ :math:`x`\  of the beam ranges from \ :math:`-L/2`\  (at node 0) to \ :math:`L/2`\  (at node 1). The axial coordinate is \ :math:`x`\  (first coordinate) and the cross section is spanned by local \ :math:`y`\ /\ :math:`z`\  axes; assuming dimensions \ :math:`w_y`\  and \ :math:`w_z`\  in cross section, the local position range is \ :math:`\in [[-L/2,L/2],\, [-wy/2,wy/2],\, [-wz/2,wz/2] ]`\ .

\ **Additional information for ObjectANCFBeam**\ :

* | The Object has the following types = \ ``Body``\ , \ ``MultiNoded``\ 
* | Requested node type = \ ``Position``\  + \ ``Orientation``\ 
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
* | **testBeamRectangularSize** [\ :math:`[h_y,\, h_z]`\ , type = Vector2D, default = [-1.,-1.]]:
  | [SI:m] test dimensions for mass matrix and other terms using standard rectangular cross section
* | **visualization** [type = VObjectANCFBeam]:
  | parameters for visualization of item



The item VObjectANCFBeam has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown; geometry is defined by sectionGeometry
* | **sectionGeometry** [type = BeamSectionGeometry, default =  BeamSectionGeometry()]:
  | defines cross section shape used for visualization and contact
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA color of the object; if R==-1, use default color



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




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


