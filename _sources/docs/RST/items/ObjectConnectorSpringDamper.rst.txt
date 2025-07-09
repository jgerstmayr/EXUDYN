

.. _sec-item-objectconnectorspringdamper:

ObjectConnectorSpringDamper
===========================

An simple spring-damper element with additional force; connects to position-based markers.

\ **Additional information for ObjectConnectorSpringDamper**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``SpringDamper``\ 
* | \ **Short name**\  for Python visualization object = \ ``VSpringDamper``\ 


The item \ **ObjectConnectorSpringDamper**\  with type = 'ConnectorSpringDamper' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **referenceLength** [\ :math:`L_0`\ , type = UReal, default = 0.]:
  | reference length [SI:m] of spring
* | **stiffness** [\ :math:`k`\ , type = UReal, default = 0.]:
  | stiffness [SI:N/m] of spring; force acts against (length-initialLength)
* | **damping** [\ :math:`d`\ , type = UReal, default = 0.]:
  | damping [SI:N/(m s)] of damper; force acts against d/dt(length)
* | **force** [\ :math:`f_{a}`\ , type = Real, default = 0.]:
  | added constant force [SI:N] of spring; scalar force; f=1 is equivalent to reducing initialLength by 1/stiffness; f > 0: tension; f < 0: compression; can be used to model actuator force
* | **velocityOffset** [\ :math:`\dot L_0`\ , type = Real, default = 0.]:
  | velocity offset [SI:m/s] of damper, being equivalent to time change of reference length
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **springForceUserFunction** [\ :math:`\mathrm{UF} \in \Rcal`\ , type = PyFunctionMbsScalarIndexScalar5, default =  0]:
  | A Python function which defines the spring force with parameters; the Python function will only be evaluated, if activeConnector is true, otherwise the SpringDamper is inactive; see description below
* | **visualization** [type = VObjectConnectorSpringDamper]:
  | parameters for visualization of item



The item VObjectConnectorSpringDamper has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectconnectorspringdamper:

DESCRIPTION of ObjectConnectorSpringDamper
------------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Distance``\ : 
  | distance between both points
* | ``Displacement``\ : 
  | relative displacement between both points
* | ``Velocity``\ : 
  | relative velocity between both points
* | ``Force``\ : \ :math:`{\mathbf{f}}`\ 
  | 3D spring-damper force vector
* | ``ForceLocal``\ : \ :math:`f_{SD}`\ 
  | scalar spring-damper force



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
   * - | time derivative of distance
     - | \ :math:`\dot L`\ 
     - | \ :math:`\Delta\! \LU{0}{{\mathbf{v}}}\tp {\mathbf{v}}_{f}`\ 


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | output variables
     - | symbol
     - | formula
   * - | Displacement
     - | \ :math:`\Delta\! \LU{0}{{\mathbf{p}}}`\ 
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m1} - \LU{0}{{\mathbf{p}}}_{m0}`\ 
   * - | Velocity
     - | \ :math:`\Delta\! \LU{0}{{\mathbf{v}}}`\ 
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m1} - \LU{0}{{\mathbf{v}}}_{m0}`\ 
   * - | Distance
     - | \ :math:`L`\ 
     - | \ :math:`|\Delta\! \LU{0}{{\mathbf{p}}}|`\ 
   * - | Force
     - | \ :math:`{\mathbf{f}}`\ 
     - | see below


Connector forces
----------------

The unit vector in force direction reads (raises SysError if \ :math:`L=0`\ ),

.. math::

   {\mathbf{v}}_{f} = \frac{1}{L} \Delta\! \LU{0}{{\mathbf{p}}}


If \ ``activeConnector = True``\ , the scalar spring force is computed as

.. math::

   f_{SD} = k\cdot(L-L_0) + d \cdot(\dot L -\dot L_0)+ f_{a}


If the springForceUserFunction \ :math:`\mathrm{UF}`\  is defined, \ :math:`{\mathbf{f}}`\  instead becomes (\ :math:`t`\  is current time)

.. math::

   f_{SD} = \mathrm{UF}(mbs, t, i_N, L-L_0, \dot L - \dot L_0, k, d, f_{a})


and \ ``iN``\  represents the itemNumber (=objectNumber). Note that, if \ ``activeConnector = False``\ , \ :math:`f_{SD}`\  is set to zero.

The vector of the spring-damper force applied at both markers finally reads

.. math::

   {\mathbf{f}} = f_{SD}{\mathbf{v}}_{f}


The virtual work of the connector force is computed from the virtual displacement 

.. math::

   \delta \Delta\! \LU{0}{{\mathbf{p}}} = \delta \LU{0}{{\mathbf{p}}}_{m1} - \delta \LU{0}{{\mathbf{p}}}_{m0} ,


and the virtual work (note the transposed version here, because the resulting generalized forces shall be a column vector),

.. math::

   \delta W_{SD} = {\mathbf{f}} \delta \Delta\! \LU{0}{{\mathbf{p}}} = \left( k\cdot(L-L_0) + d \cdot (\dot L - \dot L_0) + f_{a} \right) \left(\delta \LU{0}{{\mathbf{p}}}_{m1} - \delta \LU{0}{{\mathbf{p}}}_{m0} \right)\tp {\mathbf{v}}_{f} .


The generalized (elastic) forces thus result from

.. math::

   {\mathbf{Q}}_{SD} = \frac{\partial \LU{0}{{\mathbf{p}}}}{\partial {\mathbf{q}}_{SD}\tp} {\mathbf{f}} ,


and read for the markers \ :math:`m0`\  and \ :math:`m1`\ ,

.. math::

   {\mathbf{Q}}_{SD, m0} = -\left( k\cdot(L-L_0) + d \cdot (\dot L - \dot L_0) + f_{a} \right) {\mathbf{J}}_{pos,m0}\tp {\mathbf{v}}_{f} , \quad {\mathbf{Q}}_{SD, m1} = \left( k\cdot(L-L_0) + d \cdot (\dot L - \dot L_0)+ f_{a} \right) {\mathbf{J}}_{pos,m1}\tp {\mathbf{v}}_{f} ,


where \ :math:`{\mathbf{J}}_{pos,m1}`\  represents the derivative of marker \ :math:`m1`\  w.r.t.\ its associated coordinates \ :math:`{\mathbf{q}}_{m1}`\ , analogously \ :math:`{\mathbf{J}}_{pos,m0}`\ .

Connector Jacobian
------------------

The position-level jacobian for the connector, involving all coordinates associated with markers \ :math:`m0`\  and \ :math:`m1`\ , follows from 

.. math::

   {\mathbf{J}}_{SD} = \mp{\frac{\partial {\mathbf{Q}}_{SD, m0}}{\partial {\mathbf{q}}_{m0}} }{\frac{\partial {\mathbf{Q}}_{SD, m0}}{\partial {\mathbf{q}}_{m1}}} {\frac{\partial {\mathbf{Q}}_{SD, m0}}{\partial {\mathbf{q}}_{m1}} }{\frac{\partial {\mathbf{Q}}_{SD, m1}}{\partial {\mathbf{q}}_{m1}}}


and the velocity level jacobian reads

.. math::

   {\mathbf{J}}_{SD,t} = \mp{\frac{\partial {\mathbf{Q}}_{SD, m0}}{\partial \dot {\mathbf{q}}_{m0}} }{\frac{\partial {\mathbf{Q}}_{SD, m0}}{\partial \dot {\mathbf{q}}_{m1}}} {\frac{\partial {\mathbf{Q}}_{SD, m0}}{\partial \dot {\mathbf{q}}_{m1}} }{\frac{\partial {\mathbf{Q}}_{SD, m1}}{\partial \dot {\mathbf{q}}_{m1}}}


The sub-Jacobians follow from

.. math::

   \frac{\partial {\mathbf{Q}}_{SD, m0}}{\partial {\mathbf{q}}_{m0}} = -\frac{\partial {\mathbf{J}}_{pos,m0}\tp }{\partial {\mathbf{q}}_{m0}} {\mathbf{v}}_{f} \left( k\cdot(L-L_0) + d \cdot(\dot L - \dot L_0) + f_{a} \right) -{\mathbf{J}}_{pos,m0}\tp \frac{\partial {\mathbf{v}}_{f} \left( k\cdot(L-L_0) + d \cdot(\dot L - \dot L_0) + f_{a} \right)   }{\partial {\mathbf{q}}_{m0}}


in which the term \ :math:`\frac{\partial {\mathbf{J}}_{pos,m0}\tp }{\partial {\mathbf{q}}_{m0}}`\  is computed from a special function provided by markers, that
compute the derivative of the marker jacobian times a constant vector, in this case the spring force \ :math:`{\mathbf{f}}`\ ; this jacobian term is usually less  
dominant, but is included in the numerical as well as the analytical derivatives, see the general jacobian computation information.

The other term, which is the dominant term, is computed as (dependence of velocity term on position coordinates and \ :math:`\dot L_0`\  term neglected),

.. math::

   \frac{\partial {\mathbf{Q}}_{SD, m0}}{\partial {\mathbf{q}}_{m0}} &=& -{\mathbf{J}}_{pos,m0}\tp \frac{\partial {\mathbf{v}}_{f} \left( k\cdot(L-L_0) + d \cdot(\dot L - \dot L_0) + f_{a} \right)   }{\partial {\mathbf{q}}_{m0}} \nonumber \\
   &=& -{\mathbf{J}}_{pos,m0}\tp \frac{\partial  \left( k\cdot \left( \Delta\! \LU{0}{{\mathbf{p}}} - L_0 {\mathbf{v}}_{f} \right)+ {\mathbf{v}}_{f} \left(d \cdot {\mathbf{v}}_{f}\tp \Delta\! \LU{0}{{\mathbf{v}}}  + f_{a} \right) \right)   }{\partial {\mathbf{q}}_{m0}} \nonumber \\
   &\approx& {\mathbf{J}}_{pos,m0}\tp \left(k\cdot {\mathbf{I}} - k  \frac{L_0}{L}\left({\mathbf{I}} - \LU{0}{{\mathbf{v}}_{f}} \otimes \LU{0}{{\mathbf{v}}_{f}} \right)  +\frac{1}{L}\left({\mathbf{I}} - \LU{0}{{\mathbf{v}}_{f}} \otimes \LU{0}{{\mathbf{v}}_{f}} \right) \left(d \cdot {\mathbf{v}}_{f}\tp \Delta\! \LU{0}{{\mathbf{v}}}  + f_{a} \right) \right. \nonumber \\
   &&\left. + d \LU{0}{{\mathbf{v}}_{f}} \otimes \left(\frac{1}{L}\left({\mathbf{I}} - \LU{0}{{\mathbf{v}}_{f}} \otimes \LU{0}{{\mathbf{v}}_{f}} \right) \LU{0}{{\mathbf{v}}_{f}} \right) \right) \LU{0}{{\mathbf{J}}_{pos,m0}}


Alternatively (again \ :math:`\dot L_0`\  term neglected):

.. math::

   \frac{\partial {\mathbf{Q}}_{SD, m0}}{\partial {\mathbf{q}}_{m0}} &=& -{\mathbf{J}}_{pos,m0}\tp \frac{\partial {\mathbf{v}}_{f} \left( k\cdot(L-L_0) + d \cdot(\dot L - \dot L_0) + f_{a} \right)   }{\partial {\mathbf{q}}_{m0}} \nonumber \\
   &=& {\mathbf{J}}_{pos,m0}\tp \frac{1}{L}\left({\mathbf{I}} - \LU{0}{{\mathbf{v}}_{f}} \otimes \LU{0}{{\mathbf{v}}_{f}} \right) \left( k\cdot(L-L_0) + d \cdot(\dot L - \dot L_0) + f_{a} \right) {\mathbf{J}}_{pos,m0} \nonumber \\
   && +{\mathbf{J}}_{pos,m0}\tp \LU{0}{{\mathbf{v}}_{f}} \otimes \left( k\cdot \LU{0}{{\mathbf{v}}_{f}} + d \cdot\Delta\! \LU{0}{{\mathbf{v}}} \frac{1}{L}\left({\mathbf{I}} - \LU{0}{{\mathbf{v}}_{f}} \otimes \LU{0}{{\mathbf{v}}_{f}} \right) \right) {\mathbf{J}}_{pos,m0} - d {\mathbf{J}}_{pos,m0}\tp \LU{0}{{\mathbf{v}}_{f}} \otimes \LU{0}{{\mathbf{v}}_{f}} \frac{\partial \Delta\! \LU{0}{{\mathbf{v}}}}{\partial {\mathbf{q}}_{m0}}  \nonumber \\
   &=& {\mathbf{J}}_{pos,m0}\tp \left(\frac{f_{SD}}{L}\left({\mathbf{I}} - \LU{0}{{\mathbf{v}}_{f}} \otimes \LU{0}{{\mathbf{v}}_{f}} \right) + k \LU{0}{{\mathbf{v}}_{f}} \otimes \LU{0}{{\mathbf{v}}_{f}} + \frac{d}{L} \left(\LU{0}{{\mathbf{v}}_{f}} \otimes \Delta\! \LU{0}{{\mathbf{v}}}\right) \cdot \left({\mathbf{I}} - \LU{0}{{\mathbf{v}}_{f}} \otimes \LU{0}{{\mathbf{v}}_{f}} \right) + ...! \right) {\mathbf{J}}_{pos,m0}


Noting that \ :math:`\frac{\partial {\mathbf{v}}_{f} }{\partial {\mathbf{q}}_{m0}} = 
-\frac{1}{L}\left({\mathbf{I}} - \LU{0}{{\mathbf{v}}_{f}} \otimes \LU{0}{{\mathbf{v}}_{f}} \right) \LU{0}{{\mathbf{J}}_{pos,m0}}`\  and 
\ :math:`\frac{\partial {\mathbf{v}}_{f} }{\partial {\mathbf{q}}_{m1}} = 
\frac{1}{L}\left({\mathbf{I}} - \LU{0}{{\mathbf{v}}_{f}} \otimes \LU{0}{{\mathbf{v}}_{f}} \right) \LU{0}{{\mathbf{J}}_{pos,m1}}`\ .
The Jacobian w.r.t.\ velocity coordinates follows as

.. math::

   \frac{\partial {\mathbf{Q}}_{SD, m0}}{\partial \dot {\mathbf{q}}_{m0}} &=& -{\mathbf{J}}_{pos,m0}\tp \frac{\partial {\mathbf{v}}_{f} \left( k\cdot(L-L_0) + d \cdot(\dot L - \dot L_0) + f_{a} \right)   }{\partial \dot {\mathbf{q}}_{m0}} \nonumber \\
   &=& {\mathbf{J}}_{pos,m0}\tp \left(d {\mathbf{v}}_{f} \otimes {\mathbf{v}}_{f} \right) \LU{0}{{\mathbf{J}}_{pos,m0}}


Note that in case that \ :math:`L=0`\ , the term \ :math:`\frac{1}{L} \left({\mathbf{I}} - \LU{0}{{\mathbf{v}}_{f}} \otimes \LU{0}{{\mathbf{v}}_{f}} \right)`\  is replaced
by the unit matrix, in order to avoid zero (singular) jacobian; this is a workaround and should only occur in exceptional cases.

The term \ :math:`\frac{\partial \Delta\! \LU{0}{{\mathbf{v}}}}{\partial {\mathbf{q}}_{m0}}`\ , which is important for large damping, yields

.. math::

   \frac{\partial \Delta\! \LU{0}{{\mathbf{v}}}}{\partial {\mathbf{q}}_{m0}} = \frac{\partial {\mathbf{J}}_{pos,m0} \dot {\mathbf{q}}_{m0}}{\partial {\mathbf{q}}_{m0}}= \frac{\partial {\mathbf{J}}_{pos,m0} }{\partial {\mathbf{q}}_{m0}} \dot {\mathbf{q}}_{m0}


The latter term is currently neglected.

Jacobians for markers \ :math:`m1`\  and mixed \ :math:`m0`\ /\ :math:`m1`\  terms follow analogously.

--------

\ **Userfunction**\ : ``springForceUserFunction(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force)`` 


A user function, which computes the spring force depending on time, object variables (deltaL, deltaL_t) and 
object parameters (stiffness, damping, force).
The object variables are provided to the function using the current values of the SpringDamper object.
Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
\ ``mbs.GetObjectParameter(itemNumber, ...)``\ , see the according description of \ ``GetObjectParameter``\ .

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments /  return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs to which object belongs
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number \ :math:`i_N`\  of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``deltaL``\ 
     - | Real
     - | \ :math:`L-L_0`\ , spring elongation
   * - | \ ``deltaL_t``\ 
     - | Real
     - | \ :math:`(\dot L - \dot L_0)`\ , spring velocity, including offset
   * - | \ ``stiffness``\ 
     - | Real
     - | copied from object
   * - | \ ``damping``\ 
     - | Real
     - | copied from object
   * - | \ ``force``\ 
     - | Real
     - | copied from object; constant force
   * - | \returnValue
     - | Real
     - | scalar value of computed spring force


--------

\ **User function example**\ :



.. code-block:: python

    #define nonlinear force
    def UFforce(mbs, t, itemNumber, u, v, k, d, F0): 
        return k*u + d*v + F0
    #markerNumbers taken from mini example
    mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[m0,m1],
                                              referenceLength = 1, 
                                              stiffness = 100, damping = 1,
                                              springForceUserFunction = UFforce))

 



.. _miniexample-objectconnectorspringdamper:

MINI EXAMPLE for ObjectConnectorSpringDamper
--------------------------------------------


.. code-block:: python
   :linenos:

   node = mbs.AddNode(NodePoint(referenceCoordinates = [1.05,0,0]))
   oMassPoint = mbs.AddObject(MassPoint(nodeNumber = node, physicsMass=1))
   
   m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0]))
   m1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oMassPoint, localPosition=[0,0,0]))
   
   mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[m0,m1],
                                             referenceLength = 1, #shorter than initial distance
                                             stiffness = 100,
                                             damping = 1))
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   mbs.SolveDynamic()
   
   #check result at default integration time
   exudynTestGlobals.testResult = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[0]

Relevant Examples and TestModels with weblink:

    \ `HydraulicsUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/HydraulicsUserFunction.py>`_\  (Examples/), \ `SpringDamperMassUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/SpringDamperMassUserFunction.py>`_\  (Examples/), \ `basicTutorial2024.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/basicTutorial2024.py>`_\  (Examples/), \ `camFollowerExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/camFollowerExample.py>`_\  (Examples/), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Examples/), \ `contactCurveWithLongCurve.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/contactCurveWithLongCurve.py>`_\  (Examples/), \ `springDamperTutorialNew.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/springDamperTutorialNew.py>`_\  (Examples/), \ `springMassFriction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/springMassFriction.py>`_\  (Examples/), \ `symbolicUserFunctionMasses.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/symbolicUserFunctionMasses.py>`_\  (Examples/), \ `tutorialNeuralNetwork.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/tutorialNeuralNetwork.py>`_\  (Examples/), \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Examples/), \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Examples/), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TestModels/), \ `loadUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/loadUserFunctionTest.py>`_\  (TestModels/), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


