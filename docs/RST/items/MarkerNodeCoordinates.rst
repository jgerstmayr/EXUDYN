

.. _sec-item-markernodecoordinates:

MarkerNodeCoordinates
=====================

A node-Marker attached to all \ :ref:`ODE2 <ODE2>`\  coordinates of a node; IN CONTRAST to MarkerNodeCoordinate, the marker coordinates INCLUDE the reference values! for \ :ref:`ODE1 <ODE1>`\  coordinates use MarkerNodeODE1Coordinates (under development).

\ **Additional information for MarkerNodeCoordinates**\ :

* | This \ ``Marker``\  has/provides the following types = \ ``Node``\ , \ ``Coordinate``\ 


The item \ **MarkerNodeCoordinates**\  with type = 'NodeCoordinates' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number to which marker is attached to
* | **visualization** [type = VMarkerNodeCoordinates]:
  | parameters for visualization of item



The item VMarkerNodeCoordinates has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-markernodecoordinates:

DESCRIPTION of MarkerNodeCoordinates
------------------------------------

Relevant Examples and TestModels with weblink:

    \ `coordinateVectorConstraint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateVectorConstraint.py>`_\  (TestModels/), \ `coordinateVectorConstraintGenericODE2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateVectorConstraintGenericODE2.py>`_\  (TestModels/), \ `rigidBodyAsUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyAsUserFunctionTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


