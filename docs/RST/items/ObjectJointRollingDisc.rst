

.. _sec-item-objectjointrollingdisc:

ObjectJointRollingDisc
======================

A joint representing a rolling rigid disc (marker 1) on a flat surface (marker 0, ground body) in global \ :math:`x`\ -\ :math:`y`\  plane. The contraint is based on an idealized rolling formulation with no slip. The contraints works for discs as long as the disc axis and the plane normal vector are not parallel. It must be assured that the disc has contact to ground in the initial configuration (adjust z-position of body accordingly). The ground body can be a rigid body which is moving. In this case, the flat surface is assumed to be in the \ :math:`x`\ -\ :math:`y`\ -plane at \ :math:`z=0`\ . Note that the rolling body must have the reference point at the center of the disc. NOTE: the cases of normal other than \ :math:`z`\ -direction, wheel axis other than \ :math:`x`\ -axis and moving ground body needs to be tested further, check your results!
 



The item \ **ObjectJointRollingDisc**\  with type = 'JointRollingDisc' has the following parameters:

 

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ], size =  2]:
  | list of markers used in connector; \ :math:`m0`\  represents the ground and \ :math:`m1`\  represents the rolling body, which has its reference point (=local position [0,0,0]) at the disc center point
* | **constrainedAxes** [type = ArrayIndex, default = [1,1,1], size = 3]:
  | flag, which determines which constraints are active, in which \ :math:`j_0,j_1`\  represent the tangential motion and \ :math:`j_2`\  represents the normal (contact) direction; currently the constraints are given in global coordinates, but will be changed into local \ :math:`J1`\  coordinates in future
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **discRadius** [type = PReal, default = 0]:
  | defines the disc radius
* | **discAxis** [type = Vector3D, default = [1,0,0]]:
  | axis of disc defined in marker \ :math:`m1`\  frame
* | **planeNormal** [type = Vector3D, default = [0,0,1]]:
  | normal to the contact / rolling plane defined in marker \ :math:`m0`\  coordinates



The item VObjectJointRollingDisc has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **discWidth** [type = float, default = 0.1]:
  | width of disc for drawing
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


