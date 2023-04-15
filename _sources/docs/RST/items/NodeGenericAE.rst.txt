

.. _sec-item-nodegenericae:

NodeGenericAE
=============

A node containing a number of \ :ref:`AE <AE>`\  variables; use e.g. linear state space systems. Note that referenceCoordinates and initialCoordinates must be initialized, because no default values exist.

The item \ **NodeGenericAE**\  with type = 'GenericAE' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [\ :math:`{\mathbf{y}}\cRef = [y_0,\,\ldots,\,y_{nc}]\tp\cRef`\ , type = Vector, default = []]:
  | generic reference coordinates of node; must be consistent with numberOfAECoordinates
* | **initialCoordinates** [\ :math:`{\mathbf{y}}\cIni = [y_0,\,\ldots,\,y_{nc}]\tp\cIni`\ , type = Vector, default = []]:
  | initial displacement coordinates; must be consistent with numberOfAECoordinates
* | **numberOfAECoordinates** [\ :math:`n_c`\ , type = PInt, default = 0]:
  | number of generic \ :ref:`AE <AE>`\  coordinates
* | **visualization** [type = VNodeGenericAE]:
  | parameters for visualization of item



The item VNodeGenericAE has the following parameters:

* | **show** [type = Bool, default = False]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-nodegenericae:

DESCRIPTION of NodeGenericAE
----------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Coordinates``\ : \ :math:`{\mathbf{y}}\cConfig = [y_0,\,\ldots,\,y_{nc}]\tp\cConfig`\ 
  | \ :ref:`AE <AE>`\  coordinates vector of node




\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


