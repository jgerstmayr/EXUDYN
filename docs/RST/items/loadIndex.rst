====
Load
====

A Load applies a (usually constant) force, torque, mass-proportional or generalized load onto Nodes or Objects via Markers. The requested \ ``Marker``\  types need to be provided by the used \text{Marker}. The marker may provide more types than requested. For non-constant loads, use either a \ ``load...UserFunction``\  or change the load in every step by means of a \ ``preStepUserFunction``\  in the \ ``MainSystem``\  (mbs).

.. toctree::
   :maxdepth: 2

   LoadForceVector.rst
   LoadTorqueVector.rst
   LoadMassProportional.rst
   LoadCoordinate.rst

