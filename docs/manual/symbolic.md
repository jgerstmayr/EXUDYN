(sec-symbolic)=
# Symbolic system and user functions

The items and solvers of Exudyn are C++. Four parts let a model be written in Python and still be computed fast or
be taken outside Exudyn:

- the **symbolic system**: the equations of motion of a system of user items as symbolic expressions, which can be
  evaluated, differentiated, solved and exported;
- **symbolic user functions**: a Python user function recorded once and then computed in C++;
- **user items**: bodies, nodes, markers, connectors and joints whose behaviour is written in Python, with the same
  interface to the system as the built-in items;
- the **symbolic solvers**: the explicit and the generalized-alpha solver of Exudyn written in Python, for the
  symbolic system.

```{toctree}
:maxdepth: 2

symbolicSystem
symbolicUserFunctions
userItems
symbolicSolvers
```
