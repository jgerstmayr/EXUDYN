

.. _sec-item-objectkinematictree:

ObjectKinematicTree
===================

A special object to represent open kinematic trees using minimum coordinate formulation (NOT FULLY TESTED!). The kinematic tree is defined by lists of joint types, parents, inertia parameters (w.r.t. COM), etc.\ per link (body) and given joint (pre) transformations from the previous joint. Every joint / link is defined by the position and orientation of the previous joint and a coordinate transformation (incl.\ translation) from the previous link's to this link's joint coordinates. The joint can be combined with a marker, which allows to attach connectors as well as joints to represent closed loop mechanisms. Efficient models can be created by using tree structures in combination with constraints and very long chains should be avoided and replaced by (smaller) jointed chains if possible. The class Robot from exudyn.robotics can also be used to create kinematic trees, which are then exported as KinematicTree or as redundant multibody system. Use specialized settings in VisualizationSettings.bodies.kinematicTree for showing joint frames and other properties.

\ **Additional information for ObjectKinematicTree**\ :

* | The Object has the following types = \ ``Body``\ , \ ``MultiNoded``\ , \ ``SuperElement``\ 
* | Requested node type = \ ``GenericODE2``\ 
* | \ **Short name**\  for Python = \ ``KinematicTree``\ 
* | \ **Short name**\  for Python visualization object = \ ``VKinematicTree``\ 


The item \ **ObjectKinematicTree**\  with type = 'KinematicTree' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **nodeNumber** [\ :math:`n_0 \in \Ncal^n`\ , type = NodeIndex, default = invalid (-1)]:
  | node number (type NodeIndex) of GenericODE2 node containing the coordinates for the kinematic tree; \ :math:`n`\  being the number of minimum coordinates
* | **gravity** [\ :math:`\LU{0}{{\mathbf{g}}} \in \Rcal^{3}`\ , type = Vector3D, default = [0.,0.,0.]]:
  | gravity vector in inertial coordinates; used to simply apply gravity as LoadMassProportional is not available for KinematicTree
* | **baseOffset** [\ :math:`\LU{0}{{\mathbf{p}}_b} \in \Rcal^{3}`\ , type = Vector3D, default = [0.,0.,0.]]:
  | offset vector for base, in global coordinates
* | **jointTypes** [\ :math:`{\mathbf{j}}_T \in \Ncal^{n}`\ , type = JointTypeList, default = []]:
  | joint types of kinematic Tree joints; must be always set
* | **linkParents** [\ :math:`{\mathbf{i}}_p = [p_0,\, p_1,\, \ldots] \in \Ncal^{n}`\ , type = ArrayIndex, default = []]:
  | index of parent joint/link; if no parent exists, the value is \ :math:`-1`\ ; by default, \ :math:`p_0=-1`\  because the \ :math:`i`\ th parent index must always fulfill \ :math:`p_i<i`\ ; must be always set
* | **jointTransformations** [\ :math:`{\mathbf{T}} = [\LU{p_0,j_0}{{\mathbf{T}}_0},\, \LU{p_1,j_1}{{\mathbf{T}}_1},\, \ldots ] \in [\Rcal^{3 \times 3}, ...]`\ , type = Matrix3DList, default = []]:
  | list of constant joint transformations from parent joint coordinates \ :math:`p_0`\  to this joint coordinates \ :math:`j_0`\ ; if no parent exists (\ :math:`-1`\ ), the base coordinate system \ :math:`0`\  is used; must be always set
* | **jointOffsets** [\ :math:`{\mathbf{V}} = [\LU{p_0}{o_0},\, \LU{p_1}{o_1},\, \ldots ] \in [\Rcal^{3}, ...]`\ , type = Vector3DList, default = []]:
  | list of constant joint offsets from parent joint to this joint; \ :math:`p_0`\ , \ :math:`p_1`\ , \ :math:`\ldots`\  denote the parent coordinate systems; if no parent exists (\ :math:`-1`\ ), the base coordinate system \ :math:`0`\  is used; must be always set
* | **linkInertiasCOM** [\ :math:`{\mathbf{J}}_{COM} = [\LU{j_0}{{\mathbf{J}}_0},\, \LU{j_1}{{\mathbf{J}}_1},\, \ldots ] \in [\Rcal^{3 \times 3}, ...]`\ , type = Matrix3DList, default = []]:
  | list of link inertia tensors w.r.t.\ COM in joint/link \ :math:`j_i`\  coordinates; must be always set
* | **linkCOMs** [\ :math:`{\mathbf{C}} = [\LU{j_0}{{\mathbf{c}}_0},\, \LU{j_1}{{\mathbf{c}}_1},\, \ldots ] \in [\Rcal^{3}, ...]`\ , type = Vector3DList, default = []]:
  | list of vectors for center of mass (COM) in joint/link \ :math:`j_i`\  coordinates; must be always set
* | **linkMasses** [\ :math:`{\mathbf{m}} \in \Rcal^{n}`\ , type = Vector, default = []]:
  | masses of links; must be always set
* | **linkForces** [\ :math:`\LU{0}{{\mathbf{F}}} \in [\Rcal^{3}, ...]`\ , type = Vector3DList, default = []]:
  | list of 3D force vectors per link in global coordinates acting on joint frame origin; use force-torque couple to realize off-origin forces; defaults to empty list \ :math:`[]`\ , adding no forces
* | **linkTorques** [\ :math:`\LU{0}{{\mathbf{F}}_\tau} \in [\Rcal^{3}, ...]`\ , type = Vector3DList, default = []]:
  | list of 3D torque vectors per link in global coordinates; defaults to empty list \ :math:`[]`\ , adding no torques
* | **jointForceVector** [\ :math:`{\mathbf{f}} \in \Rcal^{n}`\ , type = Vector, default = []]:
  | generalized force vector per coordinate added to RHS of EOM; represents a torque around the axis of rotation in revolute joints and a force in prismatic joints; for a revolute joint \ :math:`i`\ , the torque \ :math:`f[i]`\  acts positive (w.r.t.\ rotation axis) on link \ :math:`i`\  and negative on parent link \ :math:`p_i`\ ; must be either empty list/array \ :math:`[]`\  (default) or have size \ :math:`n`\ 
* | **jointPositionOffsetVector** [\ :math:`{\mathbf{u}}_o \in \Rcal^{n}`\ , type = Vector, default = []]:
  | offset for joint coordinates used in P(D) control; acts in positive joint direction similar to jointForceVector; should be modified, e.g., in preStepUserFunction; must be either empty list/array \ :math:`[]`\  (default) or have size \ :math:`n`\ 
* | **jointVelocityOffsetVector** [\ :math:`{\mathbf{v}}_o \in \Rcal^{n}`\ , type = Vector, default = []]:
  | velocity offset for joint coordinates used in (P)D control; acts in positive joint direction similar to jointForceVector; should be modified, e.g., in preStepUserFunction; must be either empty list/array \ :math:`[]`\  (default) or have size \ :math:`n`\ 
* | **jointPControlVector** [\ :math:`{\mathbf{P}} \in \Rcal^{n}`\ , type = Vector, default = []]:
  | proportional (P) control values per joint (multiplied with position error between joint value and offset \ :math:`{\mathbf{u}}_o`\ ); note that more complicated control laws must be implemented with user functions; must be either empty list/array \ :math:`[]`\  (default) or have size \ :math:`n`\ 
* | **jointDControlVector** [\ :math:`{\mathbf{D}} \in \Rcal^{n}`\ , type = Vector, default = []]:
  | derivative (D) control values per joint (multiplied with velocity error between joint velocity and velocity offset \ :math:`{\mathbf{v}}_o`\ ); note that more complicated control laws must be implemented with user functions; must be either empty list/array \ :math:`[]`\  (default) or have size \ :math:`n`\ 
* | **forceUserFunction** [\ :math:`{\mathbf{f}}_{user} \in \Rcal^{n}`\ , type = PyFunctionVectorMbsScalarIndex2Vector, default =  0]:
  | A Python user function which computes the generalized force vector on RHS with identical action as jointForceVector; see description below
* | **visualization** [type = VObjectKinematicTree]:
  | parameters for visualization of item



The item VObjectKinematicTree has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **showLinks** [type = Bool, default = True]:
  | set true, if links shall be shown; if graphicsDataList is empty, a standard drawing for links is used (drawing a cylinder from previous joint or base to next joint; size relative to frame size in KinematicTree visualization settings); else graphicsDataList are used per link; NOTE visualization of joint and COM frames can be modified via visualizationSettings.bodies.kinematicTree
* | **showJoints** [type = Bool, default = True]:
  | set true, if joints shall be shown; if graphicsDataList is empty, a standard drawing for joints is used (drawing a cylinder for revolute joints; size relative to frame size in KinematicTree visualization settings)
* | **color** [type = Float4, size = 4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
* | **graphicsDataList** [type = BodyGraphicsDataList]:
  | Structure contains data for link/joint visualization; data is defined as list of BodyGraphicdData where every BodyGraphicdData corresponds to one link/joint; must either be emtpy list or length must agree with number of links



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Coordinates``\ : 
  | all ODE2 joint coordinates, including reference values; these are the minimal coordinates of the object
* | ``Coordinates\_t``\ : 
  | all ODE2 velocity coordinates
* | ``Coordinates\_tt``\ : 
  | all ODE2 acceleration coordinates
* | ``Force``\ : 
  | generalized forces for all coordinates (residual of all forces except mass*accleration; corresponds to ComputeODE2LHS)




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


