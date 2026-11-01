#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  exudyn.symbolics, the Python parts built on exudyn.symbolic, the symbolic module of C++ (#3165):
#             kinematics   the rotation and kinematics functions for symbolic user functions;
#             codeExport   symbolic expressions and recordings written as code (numpy, sympy, exudyn.symbolic);
#             userItems    UserItemCreator, the user items with the interfaces of the built-in items;
#             systemModel  the equations of motion of a system as symbolic programs: SymbolicSystem, NumpySystem,
#                          SystemVariables;
#             solvers      the explicit and the generalized-alpha solver on these systems;
#             utilities    functions of the time for symbolic user functions: SampleTime, Noise (#3175).
#           The modules are imported one by one, e.g. from exudyn.symbolics.systemModel import SymbolicSystem;
#           systemModel and solvers need scipy.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-29 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#public API of this module: none, the modules are imported one by one; kept complete by tools/checkAll.py (#2444)
__all__ = []
