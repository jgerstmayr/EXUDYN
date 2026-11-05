# Internals

How parts of Exudyn work inside: written for whoever changes them, after the user manual has said what they do.

| document | contents |
|---|---|
| [symbolic.md](symbolic.md) | exudyn.symbolic inside: the recording, symbolic user functions, the flat program, the copies for the threads, Optimize, the export, exudyn.symbolics.kinematics |
| [symbolicSystem.md](symbolicSystem.md) | the symbolic system inside: the representation, what is assembled, the composed system, the Jacobians, the interface of the solvers, the solvers |
| [userFunctions.md](userFunctions.md) | the C++/Python boundary of the user functions: the stored function, the path of a call, the threads, the cost |
| [userItems.md](userItems.md) | what each kind of item gives the solver, and how the user items are built |
| [systemUtilities.md](systemUtilities.md) | exudyn.systemUtilities: SystemQuantity, the energy, the constraint drift, the system graph |
| [symmetricBasis.md](symmetricBasis.md) | the frame of two slope vectors, its rotation Jacobian and the variants measured |
| [systemJacobians.md](systemJacobians.md) | the system Jacobians: the local Jacobians of the items, the connector interface, the parallel assembly, the checks |
| [scope.md](scope.md) | the scopes of the render window: the settings, one list of primitives for OpenGL and the raytracer, the static graphics, the locks |

```{toctree}
:hidden:
:maxdepth: 2

/docs/dev/internals/symbolic
/docs/dev/internals/symbolicSystem
/docs/dev/internals/userFunctions
/docs/dev/internals/userItems
/docs/dev/internals/systemUtilities
/docs/dev/internals/symmetricBasis
/docs/dev/internals/systemJacobians
/docs/dev/internals/scope
```
