

.. _sec-item-loadmassproportional:

LoadMassProportional
====================

Load attached to MarkerBodyMass marker, applying a 3D vector load (e.g. the vector [0,-g,0] is used to apply gravitational loading of size g in negative y-direction).
 



The item \ **LoadMassProportional**\  with type = 'MassProportional' has the following parameters:

 

* | **name** [type = String, default = '']:
  | load's unique name
* | **markerNumber** [type = MarkerIndex, default = invalid (-1)]:
  | marker's number to which load is applied
* | **loadVector** [type = Vector3D, default = [0.,0.,0.]]:
  | vector-valued load [SI:N/kg = m/s\ :math:`^2`\ ]; typically, this will be the gravity vector in global coordinates; in case of a user function, this v is ignored
* | **loadVectorUserFunction** [type = PyFunctionVector3DmbsScalarVector3D, default =  0]:
  | A Python function which defines the time-dependent load; see description below; see also notes on loadFactor and drawing in LoadForceVector!



The item VLoadMassProportional has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


