

.. _sec-item-nodegenericode2:

NodeGenericODE2
===============

A node containing a number of \ :ref:`ODE2 <ODE2>`\  variables; use e.g. for scalar dynamic equations (Mass1D) or for the ALECable element. Note that referenceCoordinates and all initialCoordinates(_t) must be initialized, because no default values exist.

\ **Additional information for NodeGenericODE2**\ :

* | The Node has the following types = \ ``GenericODE2``\ 


The item \ **NodeGenericODE2**\  with type = 'GenericODE2' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [\ :math:`{\mathbf{q}}\cRef = [q_0,\,\ldots,\,q_{nc}]\tp\cRef`\ , type = Vector, default = []]:
  | generic reference coordinates of node; must be consistent with numberOfODE2Coordinates
* | **initialCoordinates** [\ :math:`{\mathbf{q}}\cIni = [q_0,\,\ldots,\,q_{nc}]\tp\cIni`\ , type = Vector, default = []]:
  | initial displacement coordinates; must be consistent with numberOfODE2Coordinates
* | **initialCoordinates_t** [\ :math:`\dot {\mathbf{q}}\cIni = [\dot q_0,\,\ldots,\,\dot q_{n_c}]\tp\cIni`\ , type = Vector, default = []]:
  | initial velocity coordinates; must be consistent with numberOfODE2Coordinates
* | **numberOfODE2Coordinates** [\ :math:`n_c`\ , type = PInt, default = 0]:
  | number of generic \ :ref:`ODE2 <ODE2>`\  coordinates
* | **visualization** [type = VNodeGenericODE2]:
  | parameters for visualization of item



The item VNodeGenericODE2 has the following parameters:

* | **show** [type = Bool, default = False]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-nodegenericode2:

DESCRIPTION of NodeGenericODE2
------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Coordinates``\ : \ :math:`{\mathbf{q}}\cConfig = [q_0,\,\ldots,\,q_{nc}]\tp\cConfig`\ 
  | coordinates vector of node
* | ``Coordinates\_t``\ : \ :math:`\dot {\mathbf{q}}\cConfig = [\dot q_0,\,\ldots,\,\dot q_{nc}]\tp\cConfig`\ 
  | velocity coordinates vector of node
* | ``Coordinates\_tt``\ : \ :math:`\ddot {\mathbf{q}}\cConfig = [\ddot q_0,\,\ldots,\,\ddot q_{nc}]\tp\cConfig`\ 
  | acceleration coordinates vector of node




\ **The web version may not be complete. For details, always consider the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


