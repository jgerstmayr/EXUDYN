.. _sec-equationlhsrhs:


LHS-RHS naming conventions in EXUDYN
====================================

The general idea of the Exudyn is to have objects, which provide equations (\ :ref:`ODE2 <ODE2>`\ , \ :ref:`ODE1 <ODE1>`\ , \ :ref:`AE <AE>`\ ).
The solver then assembles these equations and solves the static or dynamic problem.
The system structure and solver are similar but much more advanced and modular as earlier solvers by the main developer .

Functions and variables contain the abbreviations \ :ref:`LHS <LHS>`\  and \ :ref:`RHS <RHS>`\ , sometimes lower-case, in order
to distinguish if terms are computed at the \ :ref:`LHS <LHS>`\  or \ :ref:`RHS <RHS>`\ .

The objects have the following \ :ref:`LHS <LHS>`\ --\ :ref:`RHS <RHS>`\  conventions:

+  the acceleration term, e.g., \ :math:`m \cdot \ddot q`\  is always positive on the \ :ref:`LHS <LHS>`\ 
+  objects, connectors, etc., use \ :ref:`LHS <LHS>`\  conventions for most terms: mass, stiffness matrix, elastic forces, damping, etc., are computed at \ :ref:`LHS <LHS>`\  of the object equation
+  object forces are written at the \ :ref:`RHS <RHS>`\  of the object equation
+  in case of constraint or connector equations, there is no \ :ref:`LHS <LHS>`\  or \ :ref:`RHS <RHS>`\ , as there is no acceleration term. 

Therefore, the computation function evaluates the term as given in the description of the object, adding it to the \ :ref:`LHS <LHS>`\ .
Object equations may read, e.g., for one coordinate \ :math:`q`\ , mass \ :math:`m`\ , damping coefficient \ :math:`d`\ , stiffness \ :math:`k`\  and applied force \ :math:`f`\ ,

.. math::

   \underbrace{m \cdot \ddot q + d \cdot \dot q + k \cdot q}_{LHS} = \underbrace{f}_{RHS}

 
In this case, the C++ function \ ``ComputeODE2LHS(const Vector\& ode2Lhs)``\  will compute the term
\ :math:`d \cdot \dot q + k \cdot q`\  with positive sign. Note that the acceleration term \ :math:`m \cdot \ddot q`\  is computed separately, as it 
is computed from mass matrix and acceleration.

However, system quantities (e.g. within the solver) are always written on \ :ref:`RHS <RHS>`\ \ (except for the acceleration \ :math:`\times`\  mass matrix and constraint reaction forces, see Eq. :eq:`eq-system-eom`\ ): 

.. math::

   \underbrace{M_{sys} \cdot \ddot q_{sys}}_{LHS} = \underbrace{f_{sys}}_{RHS} \,.


In the case of the object equation

.. math::

   m \cdot \ddot q + d \cdot \dot q + k \cdot q = f \, ,

 
the \ :ref:`RHS <RHS>`\  term becomes \ :math:`f_{sys} = -(d \cdot \dot q + k \cdot q) + f`\  and it is computed by the C++ function \ ``ComputeSystemODE2RHS``\ .
This means, that during computation, terms which appear at the \ :ref:`LHS <LHS>`\  of the object are transferred to the \ :ref:`RHS <RHS>`\  of the system equation.
This enables a simpler setup of equations for the solver.
