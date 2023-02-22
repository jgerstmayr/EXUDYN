

.. _sec-item-nodegenericdata:

NodeGenericData
===============

A node containing a number of data (history) variables; use e.g. for contact (active set), friction or plasticity (history variable).

\ **Additional information for NodeGenericData**\ :

* | The Node has the following types = \ ``GenericData``\ 


The item \ **NodeGenericData**\  with type = 'GenericData' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **initialCoordinates** [\ :math:`{\mathbf{x}}\cIni = [x_0,\,\ldots,\,x_{n_c}]\tp\cIni`\ , type = Vector, default = []]:
  | initial data coordinates
* | **numberOfDataCoordinates** [\ :math:`n_c`\ , type = UInt, default = 0]:
  | number of generic data coordinates (history variables)
* | **visualization** [type = VNodeGenericData]:
  | parameters for visualization of item



The item VNodeGenericData has the following parameters:

* | **show** [type = Bool, default = False]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-nodegenericdata:

DESCRIPTION of NodeGenericData
------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Coordinates``\ : \ :math:`{\mathbf{x}}\cConfig = [x_0,\,\ldots,\,x_{nc}]\tp\cConfig`\ 
  | data coordinates (history variables) vector of node




\ **The web version may not be complete. For details, always consider the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


