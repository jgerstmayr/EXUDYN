Dynamics: Mechanical principles
===============================

Following kinematics, dynamics comes into play, which involves the study of the forces and torques that cause the motion, bridging the gap between the motion observed and the reasons behind it.
In this section, we shortly recap mechanical principles, which are used throughout for the simulation of multibody systems.


Newton's basic principles
-------------------------

In the study of multibody systems, Newton's basic principles lay the foundational framework. At the heart of these principles are three laws that govern the motion of bodies:

\ **1. Newton's first law (law of inertia)**\ : A body remains at rest, or in uniform motion in a straight line, unless acted upon by a force \ :math:`{\mathbf{f}}`\ , expressed as:

.. math::

   {\mathbf{f}} = 0 \implies \frac{\dd {\mathbf{v}}}{\dd t} = 0


where \ :math:`{\mathbf{f}}`\  is the net force applied to the body, and \ :math:`\frac{d{\mathbf{v}}}{dt}`\  is the acceleration of the body.

\ **2. Newton's second law (law of motion)**\ : The sum of the forces F acting on a point mass equals the change in momentum of this mass, 

.. math::

   {\mathbf{f}}= \frac{\dd\,{\mathbf{J}}}{\dd t} = \dot {\mathbf{J}}


where \ :math:`{\mathbf{J}}`\  is the (linear, translational) momentum of the mass \ :math:`m`\ . Assuming a constant mass of the body (which is not true for some cases such as rockets), we find that the acceleration \ :math:`{\mathbf{a}}`\  of an object is directly proportional to the net force \ :math:`{\mathbf{f}}`\  acting upon it and inversely proportional to its mass, given by the equation:

.. math::
   :label: eq-theory-newton-fma

   {\mathbf{f}}= m {\mathbf{a}}


where \ :math:`m`\  is the mass of the object.
  
\ **3. Newton's third law (action and reaction)**\ : For every action, there is an equal and opposite reaction. This principle is fundamental in analyzing the interactions within multibody systems and can be summarized as:

.. math::

   {\mathbf{f}}_{12} = -{\mathbf{f}}_{21}


in which we observe \ :math:`{\mathbf{f}}_{12}`\  as the force exerted by body 1 on body 2, and \ :math:`{\mathbf{f}}_{21}`\  as the force exerted by body 2 on body 1.

These principles are pivotal in understanding and modeling the dynamics of multibody systems, providing the basis for further exploration and analysis in this field.
Newton's principles represent linear (translational) motion, but may be transformed to rotations by using angular momentum, as well. This leads to Euler's equations, see the description of the \ ``ObjectRigidBody``\ .


The Lagrange-d'Alembert principle
---------------------------------

For a constant mass \ :math:`m`\ , it follows from Newton's second law that

.. math::
   :label: eq-newton-dalembert

   \sum {\mathbf{f}}  = m {\mathbf{a}} ,


Eq. :eq:`eq-newton-dalembert`\  can also be written in the form

.. math::

   \sum {\mathbf{f}} - m {\mathbf{a}} = \Null


The vector \ :math:`(- m {\mathbf{a}} )`\  is referred to as the inertia force, since it can apparently be balanced to zero in the sense of a generalized equilibrium. This equilibrium is called dynamic equilibrium, in analogy to statics.

In \ **D'Alembert's Principle**\ , the inertia force \ :math:`{\mathbf{f}}_i = -m {\mathbf{a}}`\  is introduced, and the sum of the forces is written as

.. math::

   {\mathbf{f}}_e + {\mathbf{f}}_c + {\mathbf{f}}_i = \mathbf{0}


where applied forces \ :math:`{\mathbf{f}}_e`\  and constraint forces \ :math:`{\mathbf{f}}_c`\  are distinguished.

A key property of the constraint forces is that the virtual work done by constraint forces always vanishes,

.. math::
   :label: eq-virt-arb-zwangskraefte

   {\mathbf{f}}_c \cdot \delta {\mathbf{r}} = 0 .


where here \ :math:`\delta {\mathbf{r}}`\  is the virtual displacement associated with the constraint force.


Generalized Principle of Virtual Work
-------------------------------------

With the generalized principle of virtual work, it follows

.. math::

   \left({\mathbf{f}}_e + {\mathbf{f}}_i \right) \cdot \delta {\mathbf{r}} = 0 \quad \text{or} \quad


Thus, we obtain the \ **Lagrange-d'Alembert**\  principle as

.. math::

   \delta W_e + \delta W_T = 0 \quad \text{or} \quad


with the virtual work \ :math:`W_e`\  of applied forces and \ :math:`W_i`\  of inertia forces,

.. math::

   \delta W_e = {\mathbf{f}}_e \cdot \delta {\mathbf{r}} \quad \mathrm{and} \quad \delta W_T = {\mathbf{f}}_i \cdot \delta {\mathbf{r}} .


For a system of \ :math:`n`\  mass points, it follows

.. math::

   \sum_{i=0}^{n-1} \left({\mathbf{f}}_{e,i} \cdot \delta {\mathbf{r}}_i -  m_i {\mathbf{a}}_i \cdot \delta {\mathbf{r}}_i \right) = 0


It is ultimately characterized by the fact that, in comparison to D'Alembert's principle, constraint forces do not appear.

Note that this principle is used in Exudyn at many places to derive equations for rigid and flexible bodies, as well as for connectors, such as spring-dampers.


Virtual displacements
---------------------

The virtual displacement \ :math:`\delta {\mathbf{r}}_j`\  or the variation of any quantity can be written in terms of variation of the underlying coordinates, see Section :ref:`sec-theory-generalized-coordinates`\ ,

.. math::
   :label: eq-theory-virtual-displacement

   \delta {\mathbf{r}}_j = \sum_{i=1}^n \frac{\partial {\mathbf{r}}_j}{\partial  q_i} \delta q_i .




Generalized Forces
------------------

To derive the Lagrangian equations, we first consider the virtual work done by \ :math:`N`\  forces on \ :math:`N`\  mass points

.. math::

   \delta W = \sum_{j=1}^N {\mathbf{f}}_j \cdot \delta {\mathbf{r}}_j .


Here, \ :math:`{\mathbf{f}}_j`\  represents the force on, and \ :math:`\delta {\mathbf{r}}_j`\  represents the displacement of, the \ :math:`j`\ th mass point.

Using Eq. :eq:`eq-theory-virtual-displacement`\ , the virtual work can be expressed as

.. math::

   \delta W = \sum_{j=1}^N {\mathbf{f}}_j \cdot \sum_{i=1}^n \frac{\partial {\mathbf{r}}_j}{\partial  q_i} \delta q_i = \sum_{i=1}^n \left(\sum_{j=1}^N {\mathbf{f}}_j \cdot \frac{\partial {\mathbf{r}}_j}{\partial  q_i} \right) \delta q_i .


In the process of translating the virtual work of forces or moments, we identify what are called generalized forces \ :math:`Q_i`\ ,

.. math::
   :label: eq-theory-generalized-forces

   Q_i = \sum_{j=1}^N {\mathbf{f}}_j \cdot \frac{\partial {\mathbf{r}}_j}{\partial  q_i} .


Consequently, the virtual work can also be written as,

.. math::

   \delta W = \sum_{i=1}^n Q_i \, \delta q_i .





Lagrange's Equations of Motion
------------------------------

We define the kinetic energy as \ :math:`T`\ , which for \ :math:`N`\  mass points is given by

.. math::

   T = \frac{1}{2} \sum_{j=1}^N  m_j {\mathbf{v}}_j^2


Using the kinetic energy \ :math:`T`\ , the following \ **Lagrangian equations**\  can be written,

.. math::

   \frac{\mathrm{d}}{\mathrm{d}t} \frac{\partial T}{\partial \dot q_i} - \frac{\partial T}{\partial q_i} = Q_i, \quad i=1,2,\ldots,n .


These are valid under the assumption of minimal coordinates with holonomic constraints, resulting in the fact that all virtual displacements \ :math:`\delta q_i`\  are independent of each other.

For forces that possess a potential \ :math:`V`\ , i.e. the variation of the potential reads

.. math::

   \delta V = \sum_{i=1}^n \frac{\partial V}{\partial q_i}\delta q_i = - \sum_{i=1}^n Q_i^{(V)} \delta q_i ,


and assuming that \ :math:`Q_i`\  now only includes forces \ **without a potential**\ , the Lagrange equations can be rewritten as follows,

.. math::

   \frac{\mathrm{d}}{\mathrm{d}t} \frac{\partial T}{\partial \dot q_i} - \frac{\partial T}{\partial q_i} = Q_i + Q_i^{(V)}= Q_i - \frac{\partial V}{\partial q_i}, \quad i=1,2,\ldots,n .


With the \ **definition**\  \ :math:`L=T-V`\  and the relationship \ :math:`\frac{\partial V}{\partial \dot q_i} = 0`\ , a common form of Lagrange's equations is obtained:

.. math::

   \frac{\mathrm{d}}{\mathrm{d}t} \frac{\partial L}{\partial \dot q_i} - \frac{\partial L}{\partial q_i} = Q_i, \quad i=1,2,\ldots,n .


This equation is also used sometimes to derive equations of motion for rigid or flexible bodies in Exudyn.


Multibody formulations: redundant and minimal coordinates
---------------------------------------------------------

In multibody system dynamics, we primarily distinguish between two formulations:

+  \ **redundant coordinates**\ 
+  \ **minimal coordinates**\ 

Formulations based on minimal coordinates appeared naturally and early, in principle every model in dynamics, 
such as a 1D motion of a mass point according to Newton's laws, see Eq. :eq:`eq-theory-newton-fma`\ , Euler's equations, or a single DOF mass-spring-damper model. 
The \ **advantages of a minimal-coordinates**\  formulation are:

+  coordinates represent the degrees of freedom
+  direct access to relevant coordinates
+  direct application of forces to the coordinates
+  application of any class of solvers, as long as equations are non-stiff

The \ **disadvantages**\  are related to the \ **additional efforts to derive the equations of motion**\ , which has to be done for every different system,
and the general restriction to open-loop systems, making it difficult to be applied to closed-loop systems (but not always impossible). 
There are many modifications, which allow closed-loop systems, e.g., via augmented Lagrangians or similar approaches.

A \ **redundant-coordinates**\  formulation has the following \ **advantages**\ :

+  being \ **extremely versatile**\  and allowing practically any combination of (closed-loop) constraints
+  easy extension to flexible bodies, sliding constraints, etc.
+  multibody systems can be created easily by \ **combining a set of bodies and constraints**\  (\ :math:`\ra`\  user friendly)

\ **Disadvantages**\  are clearly the \ **resulting index 3**\  (position-level) constraints, 
for which only very few implicit solvers (time integration) exist, and it requires always matrix factorization during solving.
A further problem is that this formulation may potentially lead to redundant constraints, which need to be treated specially with solvers,
and that the degrees of freedom are less obvious and that relevant \ **(e.g., joint) coordinates are not directly accessible**\  on the equations level.

It is left to the user, which formulation to chose when working with multibody systems. 
In Exudyn, there is the option to create tree-like rigid body systems
using the \ ``ObjectKinematicTree``\ , which allows to use minimal coordinates for open-loop systems. In general, Exudyn is a redundant 
multibody dynamics formulation for reasons of simpler assembly of equations of motion.

As an example, we investigate the equations of motion for a \ **mathematical pendulum**\  with mass \ :math:`m`\ , distance \ :math:`L`\  between support and mass,
under gravity \ :math:`g`\ .

In the minimal coordinates formulation, see \ :numref:`fig-theory-formulations-pendulum`\ , we define the angle \ :math:`\varphi`\ , being zero in the horizontal configuration,

.. math::

   m L \ddot \varphi = m g \cos \varphi


leading to one \ :math:`2^\mathrm{nd}`\  order ordinary differential equation (ODE2), using the minimal coordinate \ :math:`\varphi`\ .


.. _fig-theory-formulations-pendulum:
.. figure:: ../theDoc/figures/pendulum.png
   :width: 200

   Mathematical pendulum with minimal coordinate \ :math:`\varphi`\ .



With redundant coordinates, see \ :numref:`fig-theory-formulations-pendulumconstraint`\ , we may introduce two Cartesian coordinates (\ :math:`x`\ , \ :math:`y`\ ), to define the location of the mass in 
the plane, leading to two differential equations for the mass point,

.. math::
   :label: eq-theory-pendulum-redundantcoords

   m \ddot x = f_x, \quad m \ddot y = f_y


together with a constraint equation 

.. math::
   :label: eq-theory-pendulum-redundantconstraint

   c(x,y) = \sqrt{x^2 + y^2} - L = 0




.. _fig-theory-formulations-pendulumconstraint:
.. figure:: ../theDoc/figures/pendulumConstraint.png
   :width: 200

   Mathematical pendulum with redundant coordinates (\ :math:`x`\ , \ :math:`y`\ ).



Note that Eq. :eq:`eq-theory-pendulum-redundantcoords`\  and Eq. :eq:`eq-theory-pendulum-redundantconstraint`\  include 3 equations for only two unknowns (\ :math:`x`\ , \ :math:`y`\ ). 
Therefore, we need to introduce an additional unknown Lagrange multiplier \ :math:`\lambda`\  for the redundant multibody formulation,
which represents a force in direction of the pendulum's string, exactly such that the length \ :math:`L`\  is preserved.
The factor \ :math:`\lambda`\  has to be projected with the constraint derivatives (Jacobian) onto the coordinates (\ :math:`x`\ , \ :math:`y`\ ), giving the forces

.. math::

   f_x = -\frac{\partial c}{\partial x} \lambda, \quad f_y = m g - \frac{\partial c}{\partial y} \lambda .


Therefore, we can rewrite Eq. :eq:`eq-theory-pendulum-redundantcoords`\  as

.. math::
   :label: eq-theory-pendulum-redundantfinal

   m \ddot x + \frac{\partial c}{\partial x} \lambda = 0, \quad m \ddot y + \frac{\partial c}{\partial y} \lambda = m g


with the algebraic constraint, written in a computationally more efficient form,

.. math::
   :label: eq-theory-pendulum-redundantconstraintfinal

   c(x,y) = x^2 + y^2 - L^2 = 0


The equations of motion are formed by both Eq. :eq:`eq-theory-pendulum-redundantfinal`\  and Eq. :eq:`eq-theory-pendulum-redundantconstraintfinal`\ ,
which are \ **differential-algebraic equations (DAEs) of index 3**\  and therefore much harder to solve than ordinary differential equations
in case of minimal coordinates.

In Exudyn, we therefore use either the generalized-\ :math:`\alpha`\  solver, or an implicit trapezoidal rule in combination with an index 2
reduction, see the section on solvers for more details.

