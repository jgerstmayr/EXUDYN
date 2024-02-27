System assembly
===============

Assembling equations of motion is done within the C++ class \ ``CSystem``\ , see the file \ ``CSystem.cpp``\ .
The general idea is to assemble, i.e. to sum up, (parts of) residuals attributed by different objects. The summation process is based on coordinate indices to which the single equations belong to.
Let's assume that we have two simple \ ``ObjectMass1D``\  objects, with object indices \ :math:`o0`\  and \ :math:`o1`\  and having mass \ :math:`m_0`\  and \ :math:`m_1`\ . They are connected to nodes of type \ ``Node1D``\  \ :math:`n0`\  and \ :math:`n1`\ , with global coordinate indices \ :math:`c0`\  and \ :math:`c1`\ .
The partial object residuals, which are fully independent equations, read

.. math::

   m_0 \cdot \ddot q_{c0} &=& RHS_{c0} , \\
   m_1 \cdot \ddot q_{c1} &=& RHS_{c1} ,


where \ :math:`RHS_{c0}`\  and \ :math:`RHS_{c1}`\  the right-hand-side of the respective equations/coordinates. They represent forces, e.g., from \ ``LoadCoordinate``\  items (which directly are applied to coordinates of nodes), say \ :math:`f_{c0}`\  and \ :math:`f_{c1}`\ , that are in case also summed up on the right hand side.
Let us for now assume that 

.. math::

   RHS_{c0} = f_{c0} \quad \mathrm{and} \quad RHS_{c1} = f_{c1} .



Now we add another \ ``ObjectMass1D``\  object with object index \ :math:`o2`\ , having mass \ :math:`m_2`\ , but letting the object \ * again*\  use node \ :math:`n0`\  with coordinate \ :math:`c0`\ .
In this case, the total object residuals read

.. math::

   (m_0+m_2) \cdot \ddot q_{c0} &=& RHS_{c0} , \\
   m_1 \cdot \ddot q_{c1} &=& RHS_{c1} .

 
It is clear, that now the mass in the first equation is increased due to the fact that two objects contribute to the same coordinate. The same would happen, if several loads are applied to the same coordinate.

Finally, if we add a \ ``CoordinateSpringDamper``\ , assuming a spring \ :math:`k`\  between coordinates \ :math:`c0`\  and \ :math:`c1`\ , the \ :ref:`RHS <RHS>`\  of equations related to \ :math:`c0`\  and \ :math:`c1`\  is now augmented to

.. math::

   RHS_{c0} &=& f_{c0} + k \cdot (q_{c1} - q_{c0}) , \\
   RHS_{c1} &=& f_{c1} + k \cdot (q_{c0} - q_{c1}) .


The system of equation would therefore read

.. math::

   (m_0+m_2) \cdot \ddot q_{c0} &=& f_{c0} + k \cdot (q_{c1} - q_{c0}) , \\
   m_1 \cdot \ddot q_{c1}  &=& f_{c1} + k \cdot (q_{c0} - q_{c1}) .


It should be noted, that all (components of) residuals ('equations') are summed up for the according coordinates, and also all contributions to the mass matrix. 
Only constraint equations, which are related to Lagrange parameters always get their 'own' Lagrange multipliers, which are automatically assigned by the system and therefore independent for every constraint.

