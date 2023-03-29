Further notes
=============


.. _sec-install-notes-goals:


Goals of Exudyn
---------------

After the first development phase (2019-2021), it

+  is a moderately large (2MB on windows!) multibody library, which can be easily linked to other projects,
+  contains basic multibody rigid bodies, flexible bodies, joints, contact, etc.,
+  includes a large Python utility library for convenient building and post processing of models,
+  allows to efficiently simulate small scale systems (compute \ :math:`100\,000`\ s of time steps per second for systems with \ :math:`n_{DOF}<10`\ ),
+  allows to efficiently simulate medium scaled systems for problems with \ :math:`n_{DOF} < 1\,000\,000`\ ,
+  is a safe and widely accessible module for Python,
+  allows to add user defined objects and solvers in C++,
+  allows to add user defined objects and solvers in Python.

Future goals (2022-2024) are:

+  add more multi-threaded parallel computing techniques (DONE, Q2 2022),
+  add vectorization,
+  add specific and advanced connectors/constraints (extended wheels, contact, control connector)
+  kinematical trees with minimal coordinates (DONE, Q1 2022),
+  automatic step size selection for second order solvers (planned, 2023),
+  deeper integration of Lie groups (Q3 2022),
+  more interfaces for robotics (DONE, Q1 2022),
+  add 3D beams (first attempts exist; planned, Q1 2023),
+  export equations (planned, 2024)

For solved issues (and new features), see section 'Issues and Bugs', Section :ref:`sec-issuetracker`\ .
For specific open issues, see \ ``trackerlog.html``\  -- a document only intended for developers!
