

.. _sec-item-nodegenericode1:

NodeGenericODE1
===============

A node containing a number of ODE1 variables; use e.g. linear state space systems. Note that referenceCoordinates and initialCoordinates must be initialized, because no default values exist.

The item \ **NodeGenericODE1**\  with type = 'GenericODE1' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [\ :math:`{\mathbf{y}}\cRef = [y_0,\,\ldots,\,y_{nc}]\tp\cRef`\ , type = Vector, default = []]:
  | generic reference coordinates of node; must be consistent with numberOfODE1Coordinates
* | **initialCoordinates** [\ :math:`{\mathbf{y}}\cIni = [y_0,\,\ldots,\,y_{nc}]\tp\cIni`\ , type = Vector, default = []]:
  | initial displacement coordinates; must be consistent with numberOfODE1Coordinates
* | **numberOfODE1Coordinates** [\ :math:`n_c`\ , type = PInt, default = 0]:
  | number of generic ODE1 coordinates
* | **visualization** [type = VNodeGenericODE1]:
  | parameters for visualization of item



The item VNodeGenericODE1 has the following parameters:

* | **show** [type = Bool, default = False]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-nodegenericode1:

DESCRIPTION of NodeGenericODE1
------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Coordinates``\ : \ :math:`{\mathbf{y}}\cConfig = [y_0,\,\ldots,\,y_{nc}]\tp\cConfig`\ 
  | ODE1 coordinates vector of node
* | ``Coordinates\_t``\ : \ :math:`\dot {\mathbf{y}}\cConfig = [\dot y_0,\,\ldots,\,\dot y_{nc}]\tp\cConfig`\ 
  | ODE1 velocity coordinates vector of node




\ **The web version may not be complete. For details, always consider the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


