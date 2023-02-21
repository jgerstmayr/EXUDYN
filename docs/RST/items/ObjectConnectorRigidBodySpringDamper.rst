

.. _sec-item-objectconnectorrigidbodyspringdamper:

ObjectConnectorRigidBodySpringDamper
====================================

An 3D spring-damper element acting on relative displacements and relative rotations of two rigid body (position+orientation) markers; connects to (position+orientation)-based markers and represents a penalty-based rigid joint (or prismatic, revolute, etc.)

\ **Additional information for ObjectConnectorRigidBodySpringDamper**\ :

* | The Object has the following types = \ ``Connector``\ 
* | Requested marker type = \ ``Position``\  + \ ``Orientation``\ 
* | Requested node type = \ ``GenericData``\ 
* | \ **Short name**\  for Python = \ ``RigidBodySpringDamper``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRigidBodySpringDamper``\ 


The item \ **ObjectConnectorRigidBodySpringDamper**\  with type = 'ConnectorRigidBodySpringDamper' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **nodeNumber** [\ :math:`n_d`\ , type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData (size depends on application) for dataCoordinates for user functions (e.g., implementing contact/friction user function)
* | **stiffness** [type = Matrix6D, default = np.zeros((6,6))]:
  | stiffness [SI:N/m or Nm/rad] of translational, torsional and coupled springs; act against relative displacements in x, y, and z-direction as well as the relative angles (calculated as Euler angles); in the simplest case, the first 3 diagonal values correspond to the local stiffness in x,y,z direction and the last 3 diagonal values correspond to the rotational stiffness around x,y and z axis
* | **damping** [type = Matrix6D, default = np.zeros((6,6))]:
  | damping [SI:N/(m/s) or Nm/(rad/s)] of translational, torsional and coupled dampers; very similar to stiffness, however, the rotational velocity is computed from the angular velocity vector
* | **rotationMarker0** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker 0; stiffness, damping, etc. components are measured in local coordinates relative to rotationMarker0
* | **rotationMarker1** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker 1; stiffness, damping, etc. components are measured in local coordinates relative to rotationMarker1
* | **offset** [type = Vector6D, default = [0.,0.,0.,0.,0.,0.]]:
  | translational and rotational offset considered in the spring force calculation
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **springForceTorqueUserFunction** [\ :math:`\mathrm{UF} \in \Rcal^6`\ , type = PyFunctionVector6DmbsScalarIndex4Vector3D2Matrix6D2Matrix3DVector6D, default =  0]:
  | A Python function which computes the 6D force-torque vector (3D force + 3D torque) between the two rigid body markers, if activeConnector=True; see description below
* | **postNewtonStepUserFunction** [\ :math:`\mathrm{UF}_{PN} \in \Rcal`\ , type = PyFunctionVectorMbsScalarIndex4VectorVector3D2Matrix6D2Matrix3DVector6D, default =  0]:
  | A Python function which computes the error of the PostNewtonStep; see description below
* | **visualization** [type = VObjectConnectorRigidBodySpringDamper]:
  | parameters for visualization of item



The item VObjectConnectorRigidBodySpringDamper has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``DisplacementLocal``\ : \ :math:`\LU{J0}{\Delta{\mathbf{p}}}`\ 
  | relative displacement in local joint0 coordinates
* | ``VelocityLocal``\ : \ :math:`\LU{J0}{\Delta{\mathbf{v}}}`\ 
  | relative translational velocity in local joint0 coordinates
* | ``Rotation``\ : \ :math:`\LU{J0}{\ttheta}= [\theta_0,\theta_1,\theta_2]\tp`\ 
  | relative rotation parameters (Tait Bryan Rxyz); these are the angles used for calculation of joint torques (e.g. if cX is the diagonal rotational stiffness, the moment for axis X reads mX=cX*phiX, etc.)
* | ``AngularVelocityLocal``\ : \ :math:`\LU{J0}{\Delta\tomega}`\ 
  | relative angular velocity in local joint0 coordinates
* | ``ForceLocal``\ : \ :math:`\LU{J0}{{\mathbf{f}}}`\ 
  | joint force in local joint0 coordinates
* | ``TorqueLocal``\ : \ :math:`\LU{J0}{{\mathbf{m}}}`\ 
  | joint torque in in local joint0 coordinates




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


