

.. _sec-item-loadcoordinate:

LoadCoordinate
==============

Load with scalar value, which is attached to a coordinate-based marker; the load can be used e.g. to apply a force to a single axis of a body, a nodal coordinate of a finite element  or a torque to the rotatory DOF of a rigid body.

\ **Additional information for LoadCoordinate**\ :

* | Requested \ ``Marker``\  type = \ ``Coordinate``\ 


The item \ **LoadCoordinate**\  with type = 'Coordinate' has the following parameters:

* | **name** [type = String, default = '']:
  | load's unique name
* | **markerNumber** [type = MarkerIndex, default = invalid (-1)]:
  | marker's number to which load is applied
* | **load** [\ :math:`f`\ , type = Real, default = 0.]:
  | scalar load [SI:N]; in case of a user function, this value is ignored
* | **loadUserFunction** [\ :math:`\mathrm{UF} \in \Rcal`\ , type = PyFunctionMbsScalar2, default =  0]:
  | A Python function which defines the time-dependent load and replaces the load; see description below; see also notes on loadFactor and drawing in LoadForceVector!
* | **visualization** [type = VLoadCoordinate]:
  | parameters for visualization of item



The item VLoadCoordinate has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-loadcoordinate:

DESCRIPTION of LoadCoordinate
-----------------------------

Details
-------

The scalar \ ``load``\  is applied on a coordinate defined by a Marker of type 'Coordinate', e.g., \ ``MarkerNodeCoordinate``\ .
This can be used to create simple 1D problems, or to simply apply a translational force on a Node or even a torque
on a rotation coordinate (but take care for its meaning).

--------

\ **Userfunction**\ : ``loadUserFunction(mbs, t, load)`` 


A user function, which computes the scalar load depending on time and the object's \ ``load``\  parameter.

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments / return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs to which load belongs
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs 
   * - | \ ``load``\ 
     - | Real
     - | \ :math:`{\mathbf{b}}`\  copied from object; WARNING: this parameter does not work in combination with static computation, as it is changed by the solver over step time
   * - | \returnValue
     - | Real
     - | computed load


--------

\ **User function example**\ :



.. code-block:: python

    from math import sin, cos, pi
    #this example uses the object's stored parameter load to compute a time-dependent load
    def UFload(mbs, t, load): 
        return load*sin(10*(2*pi)*t)

    n0=mbs.AddNode(Point())
    nodeMarker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0,coordinate=0))
    mbs.AddLoad(LoadCoordinate(markerNumber = markerCoordinate,
                               load = 10,
                               loadUserFunction = UFload))




Relevant Examples and TestModels with weblink:

    \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Examples/), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Examples/), \ `ComputeSensitivitiesExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ComputeSensitivitiesExample.py>`_\  (Examples/), \ `coordinateSpringDamper.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/coordinateSpringDamper.py>`_\  (Examples/), \ `geneticOptimizationSliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/geneticOptimizationSliderCrank.py>`_\  (Examples/), \ `lavalRotor2Dtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/lavalRotor2Dtest.py>`_\  (Examples/), \ `massSpringFrictionInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/massSpringFrictionInteractive.py>`_\  (Examples/), \ `minimizeExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/minimizeExample.py>`_\  (Examples/), \ `nMassOscillator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillator.py>`_\  (Examples/), \ `nMassOscillatorEigenmodes.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorEigenmodes.py>`_\  (Examples/), \ `nMassOscillatorInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorInteractive.py>`_\  (Examples/), \ `openAIgymInterfaceTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymInterfaceTest.py>`_\  (Examples/), \ `ANCFslidingAndALEjointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFslidingAndALEjointTest.py>`_\  (TestModels/), \ `contactCoordinateTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/contactCoordinateTest.py>`_\  (TestModels/), \ `coordinateSpringDamperExt.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateSpringDamperExt.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


