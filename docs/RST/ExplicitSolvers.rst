.. _sec-explicitsolver:


Explicit solvers
================

Explicit solvers are in general only applicable for systems without constraints (i.e., no joints!). However, some solvers accept simple \ ``CoordinateConstraint``\ , e.g., fixing coordinates to the ground.
Nevertheless, for constraint-free systems, e.g., with penalty constraints, can be solved for very high order and with great efficiency.
A list of explicit solvers is available, see Section :ref:`sec-dynamicsolvertype`\ , for an overview of all implicit and explicit solvers.

The solution vector \ :math:`\txi`\  (denoted as \ :math:`y`\  in the literature ), which is defined as

.. math::

   \txi = [{\mathbf{q}}\tp \;\; \dot {\mathbf{q}}\tp \;\; {\mathbf{y}}\tp ]\tp


and which includes \ :ref:`ODE2 <ODE2>`\  coordinates and velocities and \ :ref:`ODE1 <ODE1>`\  coordinates. All coordinates are computed without reference values.

The \ :ref:`ODE1 <ODE1>`\  and \ :ref:`ODE2 <ODE2>`\  equations of Eq. :eq:`eq-systemeom`\ , with \ :math:`\tlambda=0`\ , are written in explicit form and converted to first order equations,

.. math::
   :label: eq-systemeom

   \dot {\mathbf{q}} &=& \vel \nonumber \\
   \dot \vel & = &{\mathbf{M}}^{-1} {\mathbf{f}}_\SO({\mathbf{q}}, \vel, t) \nonumber \\
   \dot {\mathbf{y}} & = &{\mathbf{f}}_\FO({\mathbf{y}}, t) \\



The system first order differential equations for explicit solvers thus read

.. math::

   \dot \txi = {\mathbf{f}}_e (\txi, t)



.. _sec-rungekuttamethod:


Explicit Runge-Kutta method
---------------------------

Explicit time integration methods seek the solution \ :math:`\txi_{t+h}`\  at time \ :math:`t+h`\  for given initial value \ :math:`\txi_{t}`\  (at the beginning of one step \ :math:`t`\  or at the beginning of the simulation, \ :math:`t=0`\ ),

.. math::

   \txi_{t+h} = \txi_{t} + \Delta \txi.


For any given Runge-Kutta method, the integration of one step with step size \ :math:`h`\  is performed by an approximation

.. math::
   :label: s-stage-quadrature

   \Delta \txi = \int _{t}^{t+h}{\mathbf{f}}_e(\tau ,\txi(\tau ))d\tau \approx h\left[b_{1} {\mathbf{f}}_e(t,\txi(t))+b_{2} {\mathbf{f}}_e(t+c_{2} h,\txi(t+c_{2} h))+ \ldots +b_{s} {\mathbf{f}}_e(t+\txi_{s} h,u(t+\txi_{s} h))\right]


in which \ :math:`t + c_{i}h`\  is the time for stage \ :math:`i`\  and \ :math:`b_i`\  the according weight given in the integration formula.
Stages are within one step (therefor called one-step-methods), where \ :math:`c_i=0`\  represents the beginning of the step and \ :math:`c_i=1`\  the end.
Note that \ :math:`c_{1}= 0`\  for explicit integration formulas.

The unknown solution vectors \ :math:`\txi`\  at the stages are abbreviated by

.. math::

   {\mathbf{g}}_{i} \approx \txi(t+c_{i} h)


and computed by explicit integration (quadrature) formulas of lower order (\ :math:`g_i`\  not to be mixed up with algebraic equations!),

.. math::
   :label: eq-expl-rk-stages

   \begin{array}{l} {{\mathbf{g}}_{1} =\txi_t} \\ {{\mathbf{g}}_{2} =\txi_t+ha_{21} {\mathbf{f}}_e(t,{\mathbf{g}}_{1} )} \\ {{\mathbf{g}}_{3} =\txi_t+h\left[a_{31} {\mathbf{f}}_e(t,{\mathbf{g}}_{1} )+a_{32} {\mathbf{f}}_e(t+c_{2} h,{\mathbf{g}}_{2} )\right]} \\ {{\rm \; \; \; \; \; \; }\vdots } \\ {{\mathbf{g}}_{s} =\txi_t+h\left[a_{s1} {\mathbf{f}}_e(t,{\mathbf{g}}_{1} )+a_{s2} {\mathbf{f}}_e(t+c_{2} h,{\mathbf{g}}_{2} )+ \ldots +a_{s,s-1} {\mathbf{f}}_e(t+c_{s-1} h,{\mathbf{g}}_{s-1} )\right]} \end{array}


After all vectors \ :math:`{\mathbf{g}}_i`\  have been consecutively evaluated, the step is updated by Eq. :eq:`s-stage-quadrature`\ .


For some exemplary tableaus of explicit and impliciti Runge-Kutta methods, see theDoc.pdf!



Automatic step size control
---------------------------

Advanced solvers, such as \ ``ODE23``\  and \ ``DOPRI5``\ , include automatic step size control\ (activated with
\ ``timeIntegration.automaticStepSize = True``\  in simulationSettings).

We estimate the error of a time step with current step size \ :math:`h`\  by
using an embedded Runge-Kutta formula, which includes two approximations \ :eq:`s-stage-quadrature`\  of order \ :math:`p`\  and \ :math:`\hat p = p-1`\ , which is obtained by using two different integration formulas with common coefficients \ :math:`c_i`\ , but two sets of weights \ :math:`b_i`\  and \ :math:`\hat b_i`\ , leading to two approximations \ :math:`\txi`\  and \ :math:`\hat \txi`\ . These so-called embedded Runge-Kutta formulas are widely used, for details see Hairer et al. . 

The according apporximations \ :math:`\txi`\  and \ :math:`\hat \txi`\  are used to estimate an error 

.. math::

   e_j=|\xi_j- \hat \xi_j|


for every component \ :math:`j`\  of the solution vector \ :math:`\txi`\ .
A scaling is used for every component of the solution vector, evaluating at the beginning (\ :math:`0`\ ) and end (\ :math:`1`\ ) of the time step:

.. math::

   s_j = a_{tol} + r_{tol} \cdot \mathrm{max}(|\xi_{0j}|, |\xi_{1j}|)


Then the relative, scaled, scalar error for the step, which needs to fulfill \ :math:`err \le 1`\ , is computed as 

.. math::

   err = \sqrt{\frac 1 n \sum_{j=1}^n \left( \frac{\xi_{1j} - \hat \xi_{1j}}{s_j} \right)^2}


The optimal step size then reads

.. math::

   h_{opt} = h \cdot \left(\frac{1}{err} \right)^{(1/(q+1))}


Currently we use the suggested step size as

.. math::

   h_{new} = \mathrm{min}\left(h_{max}, \mathrm{min}\left(h \cdot f_{maxInc},  \mathrm{max}(h_{min}, f_{sfty} \cdot h_{opt}) \right) \right)


With the maximum step size \ :math:`h_{max} = \frac{t_{start} - t_{end}}{n_{steps}}`\  and the minimum step size \ :math:`h_{min}`\ , given in the \ ``timeIntegration``\  
\ ``simulationSettings``\ .
The factor \ :math:`f_{maxInc}`\  limits the increase of the current step size \ :math:`h`\ , the factor \ :math:`f_{sfty}`\  is a safety factor for limiting the chosen step size relative to the optimal one in order to avoid frequent step rejections. 
If \ :math:`h_{new} \le h`\ , the current step is accepted, otherwise the step is recomputed with \ :math:`h_{new}`\ .
For more details, see Hairer et al. .


Stability limit
---------------

Note that there are hard limitations for every explicit integration method regarding the step size. Especially for stiff systems (basically with high stiffness parameters and small masses, but also with restrictions to damping), the \ **step size**\  \ :math:`h`\  \ **has an upper limit**\ : \ :math:`h < h_{lim}`\ . Above that limit the method is inherently unstable, which needs to be considered both for constant and automatic step size selection.


Explicit Lie group integrators
------------------------------

All explicit solvers including the automatic step size solvers (DOPRI5, ODE23) have been equiped with Lie group integration functionality, see Holzinger et al. .

Basically, the integration formulas, see Section :ref:`sec-rungekuttamethod`\  are extended for special rotation parameters.
Lie group integration is currently only available for \ ``NodeRigidBodyRotVecLG``\  used in \ ``ObjectRigidBody``\  (3D rigid body). 
\ ``FFRFreducedOrder``\  will be extended to such nodes in the near future.
To get Lie group integrators running with rigid body models, all 3D node types need to be set to \ ``NodeRigidBodyRotVecLG``\  and 
set \ ``explicitIntegration.useLieGroupIntegration = True``\ .


Constraints with explicit solvers
---------------------------------

Explicit solvers generally do not solve for algebraic constraints, except for very simple \ ``CoordinateConstraint``\ . 
All connectors having the additional \ ``type=Constraint``\ , see the according object in Section :ref:`sec-item-objectconnectorspringdamper`\ ff., 
are in general not solvable by explicit solvers. 
Currently, only \ ``CoordinateConstraint``\  with one coordinate fixed to ground can be accounted for, 
if \ ``explicitIntegration.eliminateConstraints == True``\ . 
However, this offers the great flexibility to compute finite elements (imported meshes or ANCF beams) to be (partially) fixed to ground.
A \ ``CoordinateConstraint``\  that fixes a coordinate with index \ :math:`j`\  to ground leads to the simple algebraic \ :ref:`ODE2 <ODE2>`\  equation

.. math::

   g_j({\mathbf{q}}) = 0 \quad \Leftrightarrow \quad  q_j = 0


which can be solved by the implemented explicit solvers by just setting \ :math:`q_j = 0`\  previously to every computation and \ :math:`\dot q_j = 0`\  after every \ :ref:`RHS <RHS>`\  evaluation.

NOTE that, if \ ``explicitIntegration.eliminateConstraints == False``\ , constraints are ignored by the explicit solver (and all algebraic variables are set to zero). This may be wanted (e.g. to investigate the free motion of bodies), but in general leads to wrong and meaningless solution.

