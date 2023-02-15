
.. _sec-pythonutilityfunctions:

========================
Python Utility Functions
========================

This chapter describes in every subsection the functions and classes of the utility modules. 
These modules help to create multibody systems with the EXUDYN core module. Functions are implemented in Python and can be easily changed, extended and also verified by the user. **Check the source code** by entering these functions in Sypder and pressing ``CTRL + left mouse button``\ . These Python functions are much slower than the functions available in the C++ core. Some matrix computations with larger matrices implemented in numpy and scipy, however, are parallelized and therefore very efficient.

Note that in general functions accept lists and numpy arrays. If not, an error will occur, which is easily tracked.
Furthermore, angles are generally provided in radian (\ :math:`2\pi`\  equals \ :math:`360\,^o`\ ) and no units are used for distances, but it is recommended to use SI units (m, kg, s) throughout.

Functions have been implemented, if not otherwise mentioned, by Johannes Gerstmayr.
