.. _sec-overview-ltgmapping:


Mapping between local and global coordinate indices
===================================================

The \ :ref:`LTG <LTG>`\ -index-mappings\ (local-to-global coordinate index mappings containing transformation from local object coordinate indices to global (system) coordinate indices; this is different for \ **coordinate transformations**\ !) between local coordinate \ **indices**\ , on node or object level, and global (=system) coordinate \ **indices**\  follows the following rules:

+  \ :ref:`LTG <LTG>`\ -index-mappings are computed during \ ``mbs.Assemble()``\  and are not available before.
+  Nodes own a global index which relates the local coordinates to global (system) coordinate. E.g., for a \ :ref:`ODE2 <ODE2>`\  node with node number \ ``i``\ , this index can be obtained via the function \ ``mbs.GetNodeODE2Index(i)``\ .
+  The order of global coordinates is simply following the node numbering. If we add three nodes \ ``NodePoint``\ , the system will contain 9 coordinates, where the first triple (starting index 0) belongs to node 0, the second triple (starting index 3) belongs to node 1 and the third triple (starting index 6) belongs to node 2. After \ ``mbs.Assemble()``\ , you can access the system coordinates via \ ``mbs.systemData.GetODE2Coordinates()``\ , which returns a numpy array with 9 coordinates, containing the initial values provided in \ ``NodePoint``\  (default: zero).
+  Objects have their own \ :ref:`LTG <LTG>`\ -index-mappings for their respective coordinate types. The \ :ref:`ODE2 <ODE2>`\  coordinates of an object \ ``j``\  can be retrieved via \ ``mbs.systemData.GetObjectLTGODE2(j)``\ . For a body, these are the global \ :ref:`ODE2 <ODE2>`\  coordinates representing the body; for a connector, these are the coordinates to which the connector is linked (usually coordinates of two bodies); for a ground object, the \ :ref:`LTG <LTG>`\ -index-mapping is empty; see also Section :ref:`sec-systemdata-objectltg`\ .
+  Constraints create algebraic variables (Lagrange multipliers) automatically. For a constraint with object number \ ``k``\ , the global index to algebraic variables (of \ :ref:`AE <AE>`\ -type) can be accessed via \ ``mbs.systemData.GetObjectLTGAE(k)``\ .


