

.. _sec-item-objectconnectorcoordinatespringdamperext:

ObjectConnectorCoordinateSpringDamperExt
========================================

A 1D (scalar) spring-damper element acting on single \ :ref:`ODE2 <ODE2>`\  coordinates; same as ObjectConnectorCoordinateSpringDamper but with extended features, such as limit stop and improved friction; has different user function interface and additional data node as compared to ObjectConnectorCoordinateSpringDamper, but otherwise behaves very similar. The CoordinateSpringDamperExt is very useful for a single axis of a robot or similar machine modelled with a KinematicTree, as it can add friction and limits based on physical properties. It is highly recommended, to use the bristle model for friction with frictionProportionalZone=0 in case of implicit integrators (GeneralizedAlpha) as it converges better.

\ **Additional information for ObjectConnectorCoordinateSpringDamperExt**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``Coordinate``\ 
* | Requested \ ``Node``\  type = \ ``GenericData``\ 
* | \ **Short name**\  for Python = \ ``CoordinateSpringDamperExt``\ 
* | \ **Short name**\  for Python visualization object = \ ``VCoordinateSpringDamperExt``\ 


The item \ **ObjectConnectorCoordinateSpringDamperExt**\  with type = 'ConnectorCoordinateSpringDamperExt' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData for 3 data coordinates (friction mode, last sticking position, limit stop state), see description for details; must exist in case of bristle friction model or limit stops
* | **stiffness** [\ :math:`k`\ , type = Real, default = 0.]:
  | stiffness [SI:N/m] of spring; acts against relative value of coordinates
* | **damping** [\ :math:`d`\ , type = Real, default = 0.]:
  | damping [SI:N/(m s)] of damper; acts against relative velocity of coordinates
* | **offset** [\ :math:`x_\mathrm{off}`\ , type = Real, default = 0.]:
  | offset between two coordinates (reference length of springs), see equation; it can be used to represent the pre-scribed drive coordinate
* | **velocityOffset** [\ :math:`v_\mathrm{off}`\ , type = Real, default = 0.]:
  | offset between two coordinates; used to model D-control of a drive, where damping is not acting against prescribed velocity
* | **factor0** [\ :math:`f_0`\ , type = Real, default = 1.]:
  | marker 0 coordinate is multiplied with factor0
* | **factor1** [\ :math:`f_1`\ , type = Real, default = 1.]:
  | marker 1 coordinate is multiplied with factor1
* | **fDynamicFriction** [\ :math:`f_{\mu,\mathrm{d}}`\ , type = UReal, default = 0.]:
  | dynamic (viscous) friction force [SI:N] against relative velocity when sliding; assuming a normal force \ :math:`f_N`\ , the friction force can be interpreted as \ :math:`f_\mu = \mu f_N`\ 
* | **fStaticFrictionOffset** [\ :math:`f_{\mu,\mathrm{so}}`\ , type = UReal, default = 0.]:
  | static (dry) friction offset force [SI:N]; assuming a normal force \ :math:`f_N`\ , the friction force is limited by \ :math:`f_\mu \le (\mu_{so} + \mu_d) f_N = f_{\mu_d} + f_{\mu_{so}}`\ 
* | **stickingStiffness** [\ :math:`k_\mu`\ , type = UReal, default = 0.]:
  | stiffness of bristles in sticking case  [SI:N/m]
* | **stickingDamping** [\ :math:`d_\mu`\ , type = UReal, default = 0.]:
  | damping of bristles in sticking case  [SI:N/(m/s)]
* | **exponentialDecayStatic** [\ :math:`v_\mathrm{exp}`\ , type = PReal, default = 1.e-3]:
  | relative velocity for exponential decay of static friction offset force [SI:m/s] against relative velocity; at \ :math:`\Delta v = v_\mathrm{exp}`\ , the static friction offset force is reduced to 36.8\%
* | **fViscousFriction** [\ :math:`f_{\mu,\mathrm{v}}`\ , type = Real, default = 0.]:
  | viscous friction force part [SI:N/(m s)], acting against relative velocity in sliding case
* | **frictionProportionalZone** [\ :math:`v_\mathrm{reg}`\ , type = UReal, default = 0.]:
  | if non-zero, a regularized Stribeck model is used, regularizing friction force around zero velocity - leading to zero friction force in case of zero velocity; this does not require a data node at all; if zero, the bristle model is used, which requires a data node which contains previous friction state and last sticking position
* | **limitStopsUpper** [\ :math:`s_\mathrm{upper}`\ , type = Real, default = 0.]:
  | upper (maximum) value [SI:m] of coordinate before limit is activated; defined relative to the two marker coordinates
* | **limitStopsLower** [\ :math:`s_\mathrm{lower}`\ , type = Real, default = 0.]:
  | lower (minimum) value [SI:m] of coordinate before limit is activated; defined relative to the two marker coordinates
* | **limitStopsStiffness** [\ :math:`k_\mathrm{limits}`\ , type = UReal, default = 0.]:
  | stiffness [SI:N/m] of limit stop (contact stiffness); following a linear contact model
* | **limitStopsDamping** [\ :math:`d_\mathrm{limits}`\ , type = UReal, default = 0.]:
  | damping [SI:N/(m/s)] of limit stop (contact damping); following a linear contact model
* | **useLimitStops** [type = bool, default = False]:
  | if True, limit stops are considered and parameters must be set accordingly; furthermore, the NodeGenericData must have 3 data coordinates
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **springForceUserFunction** [\ :math:`\mathrm{UF} \in \Rcal`\ , type = PyFunctionMbsScalarIndexScalar11, default =  0]:
  | A Python function which defines the spring force with 8 parameters, see equations section / see description below
* | **visualization** [type = VObjectConnectorCoordinateSpringDamperExt]:
  | parameters for visualization of item



The item VObjectConnectorCoordinateSpringDamperExt has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectconnectorcoordinatespringdamperext:

DESCRIPTION of ObjectConnectorCoordinateSpringDamperExt
-------------------------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Displacement``\ : \ :math:`\Delta q`\ 
  | relative scalar displacement of marker coordinates
* | ``Velocity``\ : \ :math:`\Delta v`\ 
  | difference of scalar marker velocity coordinates
* | ``Force``\ : \ :math:`f_{SD}`\ 
  | scalar spring force



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | marker m0 coordinate
     - | \ :math:`q_{m0}`\ 
     - | current displacement coordinate which is provided by marker m0; does NOT include reference coordinate!
   * - | marker m1 coordinate
     - | \ :math:`q_{m1}`\ 
     - | 
   * - | marker m0 velocity coordinate
     - | \ :math:`v_{m0}`\ 
     - | current velocity coordinate which is provided by marker m0
   * - | marker m1 velocity coordinate
     - | \ :math:`v_{m1}`\ 
     - | 


Connector forces
----------------

Displacement between marker m0 to marker m1 coordinates (does NOT include reference coordinates),

.. math::

   q= f_1 \cdot q_{m1} - f_0 \cdot q_{m0}


and relative velocity,

.. math::

   v= f_1 \cdot v_{m1} - f_0 \cdot v_{m0}


The friction force is computed from given friction 'force' parameters, as there is no normal force in this model.
This means, that \ ``fDynamicFriction``\  represents \ :math:`\mu_d \cdot F_N`\  in which \ :math:`\mu_d`\  is the friction parameter and 
\ :math:`F_N`\  is an according normal force.

The friction force is computed for different cases:

+  CASE 1: \ ``frictionProportionalZone != 0``\  (\ :math:`v_\mathrm{reg} \neq 0`\ ): 

  This case works well for explicit integrators and represents simplified friction. It is suited best, e.g., for drives if considered
  for a specific velocity, but not for the velocity=0 (at which no friction force is produced).
  If \ :math:`f_{\mu,\mathrm{d}} > 0`\  or \ :math:`f_{\mu,\mathrm{so}} > 0`\  or \ :math:`f_{\mu,\mathrm{v}} != 0`\ , the Stribeck friction model is used, with
  
.. math::

   f_\mathrm{friction} = \begin{cases} (f_{\mu,\mathrm{d}} + f_{\mu,\mathrm{so}}) \frac{\Delta v}{v_\mathrm{reg}}, \quad \mathrm{if} \quad |v| <= v_\mathrm{reg} \quad \mathrm{and} \quad v_\mathrm{reg} \neq 0 \\ \mathrm{Sign}(v)\left(f_{\mu,\mathrm{d}} + f_{\mu,\mathrm{so}} \mathrm{e}^{-(|v|-v_{reg})/v_{exp}} + f_{\mu,\mathrm{v}} (|v|-v_\mathrm{reg}) \right), \quad \mathrm{else} \end{cases}


This case does not use a PostNewton iteration (which may be advantageous in constant step size explicit integration, 
but may be problematic in implicit integration).

+  CASE 2: \ ``frictionProportionalZone != 0``\  (or \ ``useLimitStops=True``\ ): 

  This case is perfectly suited for implicit integration, as it includes special switching variables that help to 
  avoid numerical problems due to switching (e.g., between stick and slip) during a Newton iteration. 
  In this case, a so-called bristle model is used, which requires the nodeNumber (data node) to be defined by a GenericDataNode, 
  which must contain 3 data variables. In case of sticking, the sticking force results from a spring-damper model with 
  parameters \ :math:`k_\mathrm{limits}`\  and \ :math:`d_\mathrm{limits}`\ , which resolves sticking very well. The last sticking position
  is tracked, which allows to change between stick and slip; however, transition means a reduction of accuracy and
  requires additional computation of system Jacobians and Newton or discontinuous iterations.
  This case includes a PostNewton iteration to switch between stick and slip.

In CASE 2, the GenericDataNode has the 3 data variables (friction mode, last sticking position, limit stop state):

+ [0:] friction mode  \ :math:`d_{\mu}`\ : 
     \ :math:`d_{\mu}=0`\ : stick, 
     \ :math:`d_{\mu}=\pm f_\mathrm{slip}`\ : slip (in according positive or negative direction); \ :math:`f_\mathrm{slip}`\  representing the slipping force
+ [1:] last sticking position  \ :math:`x_{lsp}`\ : contains relative coordinate \ :math:`q`\  at last sticking position; in the sticking case, any deviation from that position leads to an additional bristle force  

      
.. math::

   f_\mathrm{friction}^* = (q-x_{lsp}) \cdot k_\mathrm{\mu} + v \cdot d_\mathrm{\mu}


+ [2:] limit stop state \ :math:`d_{ls}`\ : \ :math:`d_{ls} = 0`\ : no limit reached (no contact, \ :math:`d_{ls}<0`\ : limitStopsLower surpassed, \ :math:`d_{ls}>0`\ : limitStopsUpper surpassed; \ :math:`|d_{ls}|`\  contains the penetration value of the soft contact model

Initialization of the GenericDataNode should be done such that the initial state (e.g. stick) is already set within this variable.
Not doing so may change results (as the solver assumes that the model is already slipping) and requires additional iterations.
NOTE, that in particular, if \ :math:`d_{\mu}`\  is initilized with 0 (stick) and \ :math:`x_{lsp}`\  (last sticking position) differs largely
from the current \ :math:`q`\ , a large initial force may result. 

The contact force \ :math:`f_\mathrm{contact}`\  is computed if limit stops are reached. 
The contact is represented by a spring-damper, which is activated as soon as the limit is reached and deactivated, if the limit is left again.
Contact forces are computed from stiffness \ :math:`k_\mathrm{limits}`\  and damping \ :math:`d_\mathrm{limits}`\ , penetration into stop and velocity,

.. math::

   f_\mathrm{contact} = \begin{cases} k_\mathrm{limits} \cdot (q-s_\mathrm{upper}) +  d_\mathrm{limits} \cdot v \quad \mathrm{if} \quad q > s_\mathrm{upper}\\ k_\mathrm{limits} \cdot (q-s_\mathrm{lower}) +  d_\mathrm{limits} \cdot v \quad \mathrm{if} \quad q < s_\mathrm{lower} \end{cases}


NOTE: while a combination of friction and limit stop is possible, it may be wanted to put a friction with 
\ ``frictionProportionalZone != 0``\  and a limit stop into two different objects, as the combined behavior 
would switch to a PostNewton method for the regularized friction model.

If \ ``activeConnector = True``\ , the scalar spring force vector is computed as

.. math::

   f_{SD} = k \cdot \left( q - x_\mathrm{off} \right) + d \cdot \left( v - v_\mathrm{off} \right) + f_\mathrm{friction} + f_\mathrm{contact}


If the springForceUserFunction \ :math:`\mathrm{UF}`\  is defined, \ :math:`{\mathbf{f}}_{SD}`\  instead becomes (\ :math:`t`\  is current time)

.. math::

   f_{SD} = \mathrm{UF}(mbs, t, i_N, q, v, k, d, x_\mathrm{off}, v_\mathrm{off}, f_{\mu,\mathrm{d}}, f_{\mu,\mathrm{so}}, v_\mathrm{exp}, f_{\mu,\mathrm{v}}, v_\mathrm{reg})


and \ ``iN``\  represents the itemNumber (=objectNumber).

The virtual work of the connector force is computed from the virtual displacement 

.. math::

   \delta q = f_1 \cdot \delta q_{m1} - f_0 \cdot \delta q_{m0} ,


and the virtual work results as

.. math::

   \delta W_{SD} = f_{SD} \cdot \delta q = f_{SD} \cdot \left( f_1 \cdot \delta q_{m1} - f_0 \cdot \delta q_{m0} \right) .


The generalized (elastic) forces thus read for the markers \ :math:`m0`\  and \ :math:`m1`\ ,

.. math::

   {\mathbf{Q}}_{SD, m0} = -f_{SD} \cdot f_0 \cdot {\mathbf{J}}_{coord,m0} , \quad {\mathbf{Q}}_{SD, m1} = f_{SD} \cdot f_1 \cdot {\mathbf{J}}_{coord,m1} ,


in which \ :math:`{\mathbf{J}}_{coord,m0}`\  and \ :math:`{\mathbf{J}}_{coord,m1}`\  represent the coordinate Jacobians of the respective markers.
As can be seen in generalized force \ :math:`{\mathbf{Q}}`\ , the factors \ :math:`f_0`\  and \ :math:`f_1`\  are added accordingly which increase the 
force on 'slower' coordinates for certain gear ratios.

If \ ``activeConnector = False``\ , \ :math:`f_{SD}`\  is set to zero.

--------

\ **Userfunction**\ : ``springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset, velocityOffset, 
fDynamicFriction, fStaticFrictionOffset, exponentialDecayStatic, fViscousFriction, frictionProportionalZone)`` 


A user function, which computes the scalar spring force depending on time, object variables (displacement, velocity) 
and several object parameters.
Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
\ ``mbs.GetObjectParameter(itemNumber, ...)``\ , see the according description of \ ``GetObjectParameter``\ .

Only a subset of object variables is passed to the function using the current values of the CoordinateSpringDamperExt object.
For parameters that are not passed via the user function interface, use mbs.GetObject(itemNumber) or, e.g.,
mbs.GetObjectParameter(itemNumber, 'limitStopsUpper') to obtain these parameters inside the user function.

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments / return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs in which underlying item is defined
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs 
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number \ :math:`i_N`\  of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``displacement``\ 
     - | Real
     - | \ :math:`\Delta q`\ 
   * - | \ ``velocity``\ 
     - | Real
     - | \ :math:`\Delta v`\ 
   * - | \ ``stiffness``\ 
     - | Real
     - | copied from object
   * - | \ ``damping``\ 
     - | Real
     - | copied from object
   * - | \ ``offset``\ 
     - | Real
     - | copied from object
   * - | \ ``velocityOffset``\ 
     - | Real
     - | copied from object
   * - | \ ``fDynamicFriction``\ 
     - | Real
     - | copied from object
   * - | \ ``fStaticFrictionOffset``\ 
     - | Real
     - | copied from object
   * - | \ ``exponentialDecayStatic``\ 
     - | Real
     - | copied from object
   * - | \ ``fViscousFriction``\ 
     - | Real
     - | copied from object
   * - | \ ``frictionProportionalZone``\ 
     - | Real
     - | copied from object, also called regularization velocity or regVel
   * - | \returnValue
     - | Real
     - | scalar value of computed force


--------

\ **User function example**\ :



.. code-block:: python

    #see also mini example! 
    #For further parameters, use mbs.GetObject(itemNumber) or 
    #  e.g. mbs.GetObjectParameter(itemNumber, 'limitStopsUpper')
    def UFforce(mbs, t, itemNumber, u, v, k, d, offset, vOffset, muDynamic, myStaticOffset, muExpVel, muViscous, muRegVel):
        return k*(u-offset) + d*v




Relevant Examples and TestModels with weblink:

    \ `coordinateSpringDamper.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/coordinateSpringDamper.py>`_\  (Examples/), \ `involuteGearGraphics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/involuteGearGraphics.py>`_\  (Examples/), \ `lugreFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/lugreFrictionTest.py>`_\  (Examples/), \ `massSpringFrictionInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/massSpringFrictionInteractive.py>`_\  (Examples/), \ `coordinateSpringDamperExt.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateSpringDamperExt.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


