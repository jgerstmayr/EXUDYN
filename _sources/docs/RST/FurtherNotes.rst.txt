Further notes
=============


.. _sec-install-notes-goals:


Goals of Exudyn
---------------

After the first development phase (2019-2023), it

+  is a moderately large \ (wheels have only sizes of 2MB on Windows and 4MB on Linux, without fast Exudyn options) multibody library, which can be easily linked to other projects,
+  contains basic multibody rigid bodies, flexible bodies, joints, contact, etc.,
+  includes a large Python utility library for convenient building and post processing of models,
+  allows to efficiently simulate small scale systems (compute \ :math:`100\,000`\ s of time steps per second for systems with \ :math:`n_{DOF}<10`\ ),
+  allows to efficiently simulate medium scaled systems for problems with \ :math:`n_{DOF} < 1\,000\,000`\ ,
+  is a safe and widely accessible module for Python,
+  allows to add user defined objects and solvers in C++,
+  allows to add user defined objects and solvers in Python,
+  allows multi-threaded parallel computing,
+  includes Lie group integration,
+  includes interfaces for robotics and ROS,
+  includes interfaces for reinforcement learning (stable-baselines3), pytorch and artificial intelligence,
+  includes kinematical trees with minimal coordinates.

Future goals (2024-2026) are:

+  add specific and advanced connectors/constraints (extended wheels, contact, control connector)
+  automatic step size selection for second order solvers (planned 2025),
+  add 3D beams and plates (first attempts exist; planned 2024),
+  export equations (planned, 2025),
+  add GPU support (planned, 2025).

For solved issues (and new features), see section 'Issues and Bugs', Section :ref:`sec-issuetracker`\ .
For specific open issues, see \ ``trackerlog.html``\  -- a document only intended for developers!
