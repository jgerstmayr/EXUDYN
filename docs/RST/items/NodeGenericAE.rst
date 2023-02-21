

.. _sec-item-nodegenericae:

NodeGenericAE
=============

A node containing a number of AE variables; use e.g. linear state space systems. Note that referenceCoordinates and initialCoordinates must be initialized, because no default values exist.

The item \ **NodeGenericAE**\  with type = 'GenericAE' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [\ :math:`{\mathbf{y}}\cRef = [y_0,\,\ldots,\,y_{nc}]\tp\cRef`\ , type = Vector, default = []]:
  | generic reference coordinates of node; must be consistent with numberOfAECoordinates
* | **initialCoordinates** [\ :math:`{\mathbf{y}}\cIni = [y_0,\,\ldots,\,y_{nc}]\tp\cIni`\ , type = Vector, default = []]:
  | initial displacement coordinates; must be consistent with numberOfAECoordinates
* | **numberOfAECoordinates** [\ :math:`n_c`\ , type = PInt, default = 0]:
  | number of generic AE coordinates
* | **visualization** [type = VNodeGenericAE]:
  | parameters for visualization of item



The item VNodeGenericAE has the following parameters:

* | **show** [type = Bool, default = False]:
  | set true, if item is shown in visualization and false if it is not shown



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Coordinates``\ : \ :math:`{\mathbf{y}}\cConfig = [y_0,\,\ldots,\,y_{nc}]\tp\cConfig`\ 
  | AE coordinates vector of node




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


