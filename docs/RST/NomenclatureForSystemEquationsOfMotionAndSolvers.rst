.. _sec-nomenclatureeom:


Nomenclature for system equations of motion and solvers
=======================================================

Using the basic notation for coordinates in Section :ref:`sec-generalnotation`\ , we use the following quantities and symbols for equations of motion and solvers:

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | \ **quantity**\ 
     - | \ **symbol**\ 
     - | \ **description**\  
   * - | number of \ :ref:`ODE2 <ODE2>`\  coordinates
     - | \ :math:`n`\ 
     - | \ :ref:`ODE2 <ODE2>`\   
   * - | number of \ :ref:`ODE1 <ODE1>`\  coordinates
     - | \ :math:`n_\FO`\ 
     - | \ :ref:`ODE1 <ODE1>`\   
   * - | number of \ :ref:`AE <AE>`\  coordinates
     - | \ :math:`m`\ 
     - | \ :ref:`AE <AE>`\   
   * - | number of system coordinates
     - | \ :math:`n_{\SYS}`\ 
     - | \SYSN 
   * - | \ :ref:`ODE2 <ODE2>`\  coordinates
     - | \ :math:`{\mathbf{q}} = [q_0,\, \ldots,\, q_{n_q}]\tp`\ 
     - | \ :ref:`ODE2 <ODE2>`\ , displacement-based coordinates (could also be rotation or deformation coordinates) 
   * - | \ :ref:`ODE2 <ODE2>`\  velocities
     - | \ :math:`\vel = \dot {\mathbf{q}} = [\dot q_0,\, \ldots,\, \dot q_{n_q}]\tp`\ 
     - | \ :ref:`ODE2 <ODE2>`\  velocity coordinates 
   * - | \ :ref:`ODE2 <ODE2>`\  accelerations
     - | \ :math:`\ddot {\mathbf{q}} = [\ddot q_0,\, \ldots,\, \ddot q_{n_q}]\tp`\ 
     - | \ :ref:`ODE2 <ODE2>`\  acceleration coordinates 
   * - | \ :ref:`ODE1 <ODE1>`\  coordinates
     - | \ :math:`{\mathbf{y}} = [y_0,\, \ldots,\, y_{n_y}]\tp`\ 
     - | vector of \ :math:`n_y`\  coordinates for \ :ref:`ODE1 <ODE1>`\  
   * - | \ :ref:`ODE1 <ODE1>`\  velocities
     - | \ :math:`\dot {\mathbf{y}} = [\dot y_0,\, \ldots,\, \dot y_{n_y}]\tp`\ 
     - | vector of \ :math:`n`\  velocities for \ :ref:`ODE1 <ODE1>`\  
   * - | \ :ref:`ODE2 <ODE2>`\  Lagrange multipliers
     - | \ :math:`\tlambda = [\lambda_0,\, \ldots,\, \lambda_m]\tp`\ 
     - | vector of \ :math:`m`\  Lagrange multipliers (=algebraic coordinates), representing the linear factors (often forces or torques) to fulfill the algebraic equations; for \ :ref:`ODE1 <ODE1>`\  and \ :ref:`ODE2 <ODE2>`\  coordinates 
   * - | data coordinates
     - | \ :math:`{\mathbf{x}} = [x_0,\, \ldots,\, x_l]\tp`\ 
     - | vector of \ :math:`l`\  data coordinates in any configuration 
   * - | \ :ref:`RHS <RHS>`\  \ :ref:`ODE2 <ODE2>`\ 
     - | \ :math:`{\mathbf{f}}_\SO\in \Rcal^{n_q}`\ 
     - | right-hand-side of \ :ref:`ODE2 <ODE2>`\  equations; (all terms except mass matrix \ :math:`\times`\  acceleration and joint reaction forces) 
   * - | \ :ref:`RHS <RHS>`\  \ :ref:`ODE1 <ODE1>`\ 
     - | \ :math:`{\mathbf{f}}_\SO\in \Rcal^{n_y}`\ 
     - | right-hand-side of \ :ref:`ODE1 <ODE1>`\  equations 
   * - | \ :ref:`AE <AE>`\ 
     - | \ :math:`{\mathbf{g}}\in \Rcal^{m}`\ 
     - | algebraic equations 
   * - | mass matrix
     - | \ :math:`{\mathbf{M}}\in \Rcal^{n_q \times n_q}`\ 
     - | mass matrix, only for \ :ref:`ODE2 <ODE2>`\  equations 
   * - | (tangent) stiffness matrix
     - | \ :math:`{\mathbf{K}}\in \Rcal^{n_q \times n_q}`\ 
     - | includes all derivatives of \ :math:`{\mathbf{f}}_\SO`\  w.r.t. \ :math:`{\mathbf{q}}`\  
   * - | damping/gyroscopic matrix
     - | \ :math:`{\mathbf{D}}\in \Rcal^{n_q \times n_q}`\ 
     - | includes all derivatives of \ :math:`{\mathbf{f}}_\SO`\  w.r.t. \ :math:`\vel`\  
   * - | step size
     - | \ :math:`h`\ 
     - | current step size in time integration method 
   * - | residual
     - | \ :math:`{\mathbf{r}}_\SO \in \Rcal^{n_q}`\ , \ :math:`{\mathbf{r}}_\FO \in \Rcal^{n_y}`\ , \ :math:`{\mathbf{r}}_\AE \in \Rcal^{m}`\ 
     - | residuals for each type of coordinates within static/time integration -- depends on method 
   * - | system residual
     - | \ :math:`{\mathbf{r}}\in \Rcal^{n_s}`\ 
     - | system residual -- depends on method 
   * - | system coordinates
     - | \ :math:`\txi`\ 
     - | system coordinates and unknowns for solver; definition depends on solver 
   * - | Jacobian
     - | \ :math:`{\mathbf{J}}\in \Rcal^{n_s \times n_s}`\ 
     - | system Jacobian -- depends on method 






