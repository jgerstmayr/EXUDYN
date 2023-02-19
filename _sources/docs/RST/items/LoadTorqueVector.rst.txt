

.. _sec-item-loadtorquevector:

LoadTorqueVector
================

Load with (3D) torque vector; attached to rigidbody-based marker.
 



The item \ **LoadTorqueVector**\  with type = 'TorqueVector' has the following parameters:

 

* | **name** [type = String, default = '']:
  | load's unique name
* | **markerNumber** [type = MarkerIndex, default = invalid (-1)]:
  | marker's number to which load is applied
* | **loadVector** [type = Vector3D, default = [0.,0.,0.]]:
  | vector-valued load [SI:N]; in case of a user function, this vector is ignored
* | **bodyFixed** [type = Bool, default = False]:
  | if bodyFixed is true, the load is defined in body-fixed (local) coordinates, leading to a follower torque; if false: global coordinates are used
* | **loadVectorUserFunction** [type = PyFunctionVector3DmbsScalarVector3D, default =  0]:
  | A Python function which defines the time-dependent load and replaces loadVector; see description below; see also notes on loadFactor and drawing in LoadForceVector! Example for Python function: def f(mbs, t, loadVector): return [loadVector[0]*np.sin(t*10*2*3.1415),0,0]



The item VLoadTorqueVector has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


