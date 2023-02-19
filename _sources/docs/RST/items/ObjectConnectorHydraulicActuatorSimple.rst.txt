

.. _sec-item-objectconnectorhydraulicactuatorsimple:

ObjectConnectorHydraulicActuatorSimple
======================================

A basic hydraulic actuator with pressure build up equations. The actuator follows a valve input value, which results in a in- or outflow of fluid depending on the pressure difference. Valve values can be prescribed by user functions (not yet available) or with the MainSystem PreStepUserFunction(...).
 



The item \ **ObjectConnectorHydraulicActuatorSimple**\  with type = 'ConnectorHydraulicActuatorSimple' has the following parameters:

 

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **nodeNumbers** [type = ArrayNodeIndex, default = []]:
  | node number of GenericODE1 node for hydraulic components (reference values for this node must be zero); data node may be added in future
* | **offsetLength** [type = UReal, default = 0.]:
  | offset length [SI:m] of cylinder, representing minimal distance between the two bushings at stroke=0
* | **strokeLength** [type = PReal, default = 0.]:
  | stroke length [SI:m] of cylinder, representing maximum extension relative to \ :math:`L_o`\ ; the measured distance between the markers is \ :math:`L_s+L_o`\ 
* | **chamberCrossSection0** [type = PReal, default = 0.]:
  | cross section [SI:m\ :math:`^2`\ ] of chamber (inner cylinder) at piston head (nut) side (0)
* | **chamberCrossSection1** [type = PReal, default = 0.]:
  | cross section [SI:m\ :math:`^2`\ ] of chamber at piston rod side (1); usually smaller than chamberCrossSection0
* | **referenceVolume0** [type = PReal, default = 0.]:
  | chamber reference volume [SI:m\ :math:`^3`\ ] at piston head (nut) side (0) for stroke length zero
* | **referenceVolume1** [type = PReal, default = 0.]:
  | chamber reference volume [SI:m\ :math:`^2`\ ] at piston rod side (1) for stroke length zero
* | **valveOpening0** [type = Real, default = 0.]:
  | relative opening of valve \ :math:`[-1 \ldots 1]`\  [SI:1] at piston head (nut) side (0); positive value is valve opening towards system pressure, negative value is valve opening towards tank pressure; zero means closed valve
* | **valveOpening1** [type = Real, default = 0.]:
  | relative opening of valve \ :math:`[-1 \ldots 1]`\  [SI:1] at piston rod side (1); positive value is valve opening towards system pressure, negative value is valve opening towards tank pressure; zero means closed valve
* | **actuatorDamping** [type = UReal, default = 0.]:
  | damping [SI:N/(m\ :math:`\,`\ s)] of hydraulic actuator (against actuator axial velocity)
* | **oilBulkModulus** [type = PReal, default = 0.]:
  | bulk modulus of oil [SI:N/(m\ :math:`^2`\ )]
* | **nominalFlow** [type = PReal, default = 0.]:
  | nominal flow of oil through valve [SI:m\ :math:`^3`\ /s]
* | **systemPressure** [type = Real, default = 0.]:
  | system pressure [SI:N/(m\ :math:`^2`\ )]
* | **tankPressure** [type = Real, default = 0.]:
  | tank pressure [SI:N/(m\ :math:`^2`\ )]
* | **useChamberVolumeChange** [type = Bool, default = False]:
  | if True, the pressure build up equations include the change of oil stiffness due to change of chamber volume
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint



The item VObjectConnectorHydraulicActuatorSimple has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **cylinderRadius** [type = float, default = 0.05]:
  | radius for drawing of cylinder
* | **rodRadius** [type = float, default = 0.03]:
  | radius for drawing of rod
* | **pistonRadius** [type = float, default = 0.04]:
  | radius for drawing of piston (if drawn transparent)
* | **pistonLength** [type = float, default = 0.001]:
  | radius for drawing of piston (if drawn transparent)
* | **rodMountRadius** [type = float, default = 0.0]:
  | radius for drawing of rod mount sphere
* | **baseMountRadius** [type = float, default = 0.0]:
  | radius for drawing of base mount sphere
* | **baseMountLength** [type = float, default = 0.0]:
  | radius for drawing of base mount sphere
* | **colorCylinder** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA cylinder color; if R==-1, use default connector color
* | **colorPiston** [type = Float4, default = [0.8,0.8,0.8,1.]]:
  | RGBA piston color




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


