

.. _sec-item-objectconnectorhydraulicactuatorsimple:

ObjectConnectorHydraulicActuatorSimple
======================================

A basic hydraulic actuator with pressure build up equations. The actuator follows a valve input value, which results in a in- or outflow of fluid depending on the pressure difference. Valve values can be prescribed by user functions (not yet available) or with the MainSystem PreStepUserFunction(...).

\ **Additional information for ObjectConnectorHydraulicActuatorSimple**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\ 
* | Requested \ ``Node``\  type: read detailed information of item
* | \ **Short name**\  for Python = \ ``HydraulicActuatorSimple``\ 
* | \ **Short name**\  for Python visualization object = \ ``VHydraulicActuatorSimple``\ 


The item \ **ObjectConnectorHydraulicActuatorSimple**\  with type = 'ConnectorHydraulicActuatorSimple' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **nodeNumbers** [\ :math:`\mathbf{n}_n = [n_{ODE1}]\tp`\ , type = ArrayNodeIndex, default = []]:
  | currently a list with one node number of NodeGenericODE1 for 2 hydraulic pressures (reference values for this node must be zero); data node may be added in future for switching
* | **offsetLength** [\ :math:`L_o`\ , type = UReal, default = 0.]:
  | offset length [SI:m] of cylinder, representing minimal distance between the two bushings at stroke=0
* | **strokeLength** [\ :math:`L_s`\ , type = PReal, default = 0.]:
  | stroke length [SI:m] of cylinder, representing maximum extension relative to \ :math:`L_o`\ ; the measured distance between the markers is \ :math:`L_s+L_o`\ 
* | **chamberCrossSection0** [\ :math:`A_0`\ , type = PReal, default = 0.]:
  | cross section [SI:m\ :math:`^2`\ ] of chamber (inner cylinder) at piston head (nut) side (0)
* | **chamberCrossSection1** [\ :math:`A_1`\ , type = PReal, default = 0.]:
  | cross section [SI:m\ :math:`^2`\ ] of chamber at piston rod side (1); usually smaller than chamberCrossSection0
* | **hoseVolume0** [\ :math:`V_{h,0}`\ , type = PReal, default = 0.]:
  | hose volume [SI:m\ :math:`^3`\ ] at piston head (nut) side (0); as the effective bulk modulus would go to infinity at stroke length zero, the hose volume must be greater than zero
* | **hoseVolume1** [\ :math:`V_{h,1}`\ , type = PReal, default = 0.]:
  | hose volume [SI:m\ :math:`^3`\ ] at piston rod side (1); as the effective bulk modulus would go to infinity at max. stroke length, the hose volume must be greater than zero
* | **valveOpening0** [\ :math:`A_{v,0}`\ , type = Real, default = 0.]:
  | relative opening of valve \ :math:`[-1 \ldots 1]`\  [SI:1] at piston head (nut) side (0); positive value is valve opening towards system pressure, negative value is valve opening towards tank pressure; zero means closed valve
* | **valveOpening1** [\ :math:`A_{v,1}`\ , type = Real, default = 0.]:
  | relative opening of valve \ :math:`[-1 \ldots 1]`\  [SI:1] at piston rod side (1); positive value is valve opening towards system pressure, negative value is valve opening towards tank pressure; zero means closed valve
* | **actuatorDamping** [\ :math:`d_{HA}`\ , type = UReal, default = 0.]:
  | damping [SI:N/(m\ :math:`\,`\ s)] of hydraulic actuator (against actuator axial velocity)
* | **oilBulkModulus** [\ :math:`K_{oil}`\ , type = PReal, default = 0.]:
  | bulk modulus of oil [SI:N/(m\ :math:`^2`\ )]
* | **cylinderBulkModulus** [\ :math:`K_{cyl}`\ , type = UReal, default = 0.]:
  | bulk modulus of cylinder [SI:N/(m\ :math:`^2`\ )]; in fact, this is value represents the effect of the cylinder stiffness on the effective bulk modulus
* | **hoseBulkModulus** [\ :math:`K_{hose}`\ , type = UReal, default = 0.]:
  | bulk modulus of hose [SI:N/(m\ :math:`^2`\ )]; in fact, this is value represents the effect of the hose stiffness on the effective bulk modulus
* | **nominalFlow** [\ :math:`Q_n`\ , type = PReal, default = 0.]:
  | nominal flow of oil through valve [SI:m\ :math:`^3`\ /s]
* | **systemPressure** [\ :math:`p_s`\ , type = Real, default = 0.]:
  | system pressure [SI:N/(m\ :math:`^2`\ )]
* | **tankPressure** [\ :math:`p_t`\ , type = Real, default = 0.]:
  | tank pressure [SI:N/(m\ :math:`^2`\ )]
* | **useChamberVolumeChange** [type = Bool, default = False]:
  | if True, the pressure build up equations include the change of oil stiffness due to change of chamber volume
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectConnectorHydraulicActuatorSimple]:
  | parameters for visualization of item



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


----------

.. _description-objectconnectorhydraulicactuatorsimple:

DESCRIPTION of ObjectConnectorHydraulicActuatorSimple
-----------------------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Distance``\ : \ :math:`L = |\Delta\! \LU{0}{{\mathbf{p}}}|`\ 
  | distance between both marker points (usually the actuator bushings); current actuator length
* | ``Displacement``\ : 
  | relative displacement between both marker points
* | ``Velocity``\ : \ :math:`\Delta\! \LU{0}{{\mathbf{v}}}`\ 
  | relative velocity between both points
* | ``VelocityLocal``\ : \ :math:`\dot L`\ 
  | actuator velocity, the derivative of actuator length
* | ``Force``\ : 
  | force in actuator resulting as the difference of both pressures times according cross sections



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | marker m0 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\ 
     - | current global position which is provided by marker m0
   * - | marker m1 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m1}`\ 
     - | 
   * - | marker m0 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | current global velocity which is provided by marker m0
   * - | marker m1 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m1}`\ 
     - | 
   * - | Displacement
     - | \ :math:`\Delta\! \LU{0}{{\mathbf{p}}}`\ =\ :math:`\LU{0}{{\mathbf{p}}}_{m1} - \LU{0}{{\mathbf{p}}}_{m0}`\ 
     - | The relative vector between marker points, stored as Displacement in output variables
   * - | current actuator length
     - | \ :math:`L`\ =\ :math:`|\Delta\! \LU{0}{{\mathbf{p}}}|`\ 
     - | stored as Distance in output variables
   * - | time derivative of actuator length
     - | \ :math:`\dot L`\ =\ :math:`\Delta\! \LU{0}{{\mathbf{v}}}\tp {\mathbf{v}}_{f}`\ 
     - | 
   * - | Velocity
     - | \ :math:`\Delta\! \LU{0}{{\mathbf{v}}}`\ =\ :math:`\LU{0}{{\mathbf{v}}}_{m1} - \LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | The vectorial relative velocity
   * - | Force
     - | \ :math:`{\mathbf{f}}`\ 
     - | see below


Connector forces
----------------

The unit vector in force direction reads (raises SysError if \ :math:`L=0`\ ),

.. math::

   {\mathbf{v}}_{f} = \frac{1}{L} \Delta\! \LU{0}{{\mathbf{p}}}


The simple double-acting hydraulic actuator has two pressure chambers, one being denoted with 0 at the
piston head (nut) and the other at the piston rod side denoted with 1. The pressure \ :math:`p_0`\  acts at the piston head at area \ :math:`A_0`\ , 
while the pressure \ :math:`p_1`\  counteracts on the opposite side with (usually smaller) area \ :math:`A_1`\ .
If \ ``activeConnector = True``\ , the scalar actuator force (tension = positive) is computed as

.. math::

   f_{HA} = -p_0 \cdot A_0 + p_1 \cdot A_1 + v \cdot d_HA


where \ :math:`v`\  represents the actuator velocitiy and \ :math:`d_HA`\  is the viscous damping coefficient.

The vector of the actuator force applied at both markers finally reads

.. math::

   {\mathbf{f}} = f_{HA}{\mathbf{v}}_{f}


The virtual work of the connector force is computed from the virtual displacement 

.. math::

   \delta \Delta\! \LU{0}{{\mathbf{p}}} = \delta \LU{0}{{\mathbf{p}}}_{m1} - \delta \LU{0}{{\mathbf{p}}}_{m0} ,


and the virtual work (not the transposed version here, because the resulting generalized forces shall be a column vector),

.. math::

   \delta W_{HA} = {\mathbf{f}} \delta \Delta\! \LU{0}{{\mathbf{p}}} .

    

Pressure build up equations
---------------------------

The hydraulics model consists of a double-acting piston. It follows the paper of  
except for the friction and the additional valve, which are not available here.

The hydraulic actuator contains internal states, namely pressures \ :math:`p_0`\  and \ :math:`p_1`\ .
The \ :ref:`ODE1 <ODE1>`\  for pressures follows for the the case of laminar flow, based on system and tank pressure,
valve positions as well as the actuator velocity and position (only for change of volume).

The distance between the two marker points, which are usually the bushings or clevis mounts of the hydraulic cylinder, is
denoted as \ :math:`L`\ . The stroke length \ :math:`s \in [0, L_s]`\  is defined as

.. math::

   s = L - L_o


such that at zero stroke, the actuator length is \ :math:`L_o`\ . The stroke velocity (positive value means extension) reads

.. math::

   \dot s = \Delta\! \LU{0}{{\mathbf{v}}\tp} {\mathbf{v}}_{f}



If \ ``useChamberVolumeChange == True``\ , the volume change due to stroke change will be considered for the
volume related to the stiffness of the fluid.
The cylinder volumes in chambers 0 and 1 are then

.. math::

   V_{0,cur} = V_{h,0} + A_0 \cdot s, \quad V_{1,cur} = V_{h,1} + A_1 \cdot (L_s - s)


The effective bulk modulus for chamber \ :math:`k \in {0,1}`\  is computed as follows,

.. math::
   :label: eq-hydraulicactuator-effbulkmodulus

   K_{k,eff} = \frac{1}{ \frac{1}{K_{oil}} + \frac{V_{k,cur} - V_{h,k}}{V_{k,cur} \cdot K_{cyl}} + \frac{V_{h,k}}{V_{k,cur} \cdot K_{hose}} },


where we use a slightly different approach from  when computing the volume for the cylinder bulk modulus term for \ :math:`k=1`\ .

Note that in case of \ :math:`K_{cyl}=0`\  and/or \ :math:`K_{hose}=0`\ , the according fractions in Eq. :eq:`eq-hydraulicactuator-effbulkmodulus`\   
are set to zero (which other wise would give infinity).

Otherwise, if \ ``useChamberVolumeChange == False``\ , \ :math:`V_{0,cur}=V_{h,0}`\ , \ :math:`V_{1,cur}=V_{h,1}`\  and \ :math:`K_{k,eff} = K_{oil}`\  for chambers \ :math:`k \in {0,1}`\ .

The pressure equations (explicit \ :ref:`ODE1 <ODE1>`\ ) have the structure

.. math::

   \vp{\dot p_0}{\dot p_1} = \vp{f_0(p_0, s, \dot s)}{f_1(p_1, s, \dot s)}


and follow for different cases and chambers / valves \ :math:`k=\{0,1\}`\ , based on the simple model where 

+  \ :math:`A_{v,k} = 0`\ : valve k closed
+  \ :math:`A_{v,k} > 0`\ : valve k opened towards system pressure (pump)
+  \ :math:`A_{v,k} < 0`\ : valve k opened towards tank pressure

Thus, the following equations are used\ (while it would happen rarely in regular operation, the arguments of the square roots could become negative; 
thus, in the implementation we use \ :math:`\mathrm{sqrts}(x) = \mathrm{sign}(x) \cdot \sqrt{\mathrm{abs}(x)}`\ .):

.. math::

   \dot p_0 = \frac{K_{0,eff}}{V_{0,cur}} \left( -A_0 \cdot \dot s + A_{v,0} \cdot Q_n \cdot \mathrm{sqrts}(p_s - p_0)  \right)  \quad \mathrm{if} \quad \mathrm A_{v,0} \ge 0



.. math::

   \dot p_0 = \frac{K_{0,eff}}{V_{0,cur}} \left( -A_0 \cdot \dot s + A_{v,0} \cdot Q_n \cdot \mathrm{sqrts}(p_0 - p_t)  \right)  \quad \mathrm{if} \quad \mathrm A_{v,0} < 0



.. math::

   \dot p_1 = \frac{K_{1,eff}}{V_{1,cur}} \left(  A_1 \cdot \dot s + A_{v,1} \cdot Q_n \cdot \mathrm{sqrts}(p_s - p_1)  \right)  \quad \mathrm{if} \quad \mathrm A_{v,1} \ge 0



.. math::

   \dot p_1 = \frac{K_{1,eff}}{V_{1,cur}} \left(  A_1 \cdot \dot s + A_{v,1} \cdot Q_n \cdot \mathrm{sqrts}(p_1 - p_t)  \right)  \quad \mathrm{if} \quad \mathrm A_{v,1} < 0





Relevant Examples and TestModels with weblink:

    \ `HydraulicActuator2Arms.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/HydraulicActuator2Arms.py>`_\  (Examples/), \ `HydraulicActuatorStaticInitialization.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/HydraulicActuatorStaticInitialization.py>`_\  (Examples/), \ `hydraulicActuatorSimpleTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/hydraulicActuatorSimpleTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


