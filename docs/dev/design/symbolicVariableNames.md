# Names of symbolic variables: evaluation (revision2026c step RG12.73, #3208)

Maintainer 2026-11-02: `esym.Real('a', 12.)` and `esym.Real('a', 13.)` are two variables of one name. Reading an
equation, one takes `m` at three places as one quantity while they may be different ones. `Diff` takes the
variable, not its name, so the computation is right, but the text is confusing and future uses may go wrong. A
global scope of names - `symbolic.globals`, each name once, `esym.Real('a', 12., addToScope=False)`, a reset and a
switch - would remove this; evaluate it, with options and recommendations.

## 1. What is (probed 2026-11-02)

- A named variable is an object; its name is a label. `Real('a', 12.)` and `Real('a', 13.)` are two variables:
  `a0*a1 + a0` prints `((a * a) + a)` and evaluates to 168; `Diff` by `a0` and by `a1` differ.
- Where a name identifies a variable, two of one name are already refused:
  - `esym.Function`: "two inputs are named 'a'"; a named variable of the result that is no input must be the one of
    `esym.variables` of its name, else "depends on the variable 'a', which is no input and not in esym.variables".
  - `esym.VariableSet.Add`: "variable name already exists"; this holds for `esym.variables` too.
  - `SymbolicSystem(mbs, parameters=...)`: "two different variables are named ..." among the parameters (#3152).
- Not refused: an input of `esym.Function` named as a variable of `esym.variables` that the result also reads -
  `Function(a0*esym.variables.Get('a'), [a0])` with `a0` named `a` is accepted, two meanings of `a` in one function.
- `esym.variables` is the module-wide `VariableSet` that recordings read at every call (symbolicUserFunctions.md):
  its members are global inputs of the recorded functions, not a register of names.
- The library creates named variables of the same names many times: every recording names its arguments after the
  Python arguments (`t`, `q`, ...); `SymbolicSystem` names its coordinates at each build, again after
  `SetParameterValues`; `exudyn.shells` creates `x`, `y`, `z` at each call. Scripts run again in one interpreter
  (Spyder, notebooks) and pytest runs many models in one process.

## 2. Options

| | what | changes for scripts | effort | remark |
|---|---|---|---|---|
| **A** checks where names identify | refuse two variables of one name wherever they meet: in one `esym.Function` (inputs and the variables of `esym.variables` it reads), one `VariableSet`, one `SymbolicSystem` and its parameters, one export of a flat program | none for valid models; the gap of section 1 raises | small: one check in `esym.Function` | most of it exists |
| **B** a global scope of names (the maintainer's proposal) | `esym.Real(name, value)` registers the name in a module-wide scope; a second variable of the name raises; `addToScope=False` exempts one, a reset and a switch restore the old behaviour | a script run a second time in one interpreter raises at its first variable unless the scope is reset; tests in one process likewise | medium | every internal creation (recordings, SymbolicSystem, shells) needs `addToScope=False`; the scope holds every variable alive unless by weak references; it must not be `esym.variables`, whose members recorded functions read as inputs - that would turn the check of A into acceptance |
| **B'** interning | `esym.Real(name, value)` returns the variable of that name if it exists with the same value and raises if the value differs | a second run works; a changed value raises | medium | global state as B; the value of a parameter changes by `SetValue`, not by a new `Real` |
| **C** scopes as objects | names are unique within a scope the user creates: `params = esym.VariableSet(); m = params.Add('m', 13.5)` (Add returns the variable), the parameters of `SymbolicSystem` and of `UserItemCreator` (symbolicParameters.md) | none; the scope is explicit | small | the parameters of a system are one scope already (P1 of symbolicParameters.md) |

## 3. Recommendation

- **A**, with the one missing check: `esym.Function` refuses an input whose name is also that of a variable of
  `esym.variables` it reads. Every place where a name identifies a variable then refuses two of one name, and the
  computations, which take the variables, stay as they are.
- **C** for the symbolic parameters: one scope per system - the `ParameterBindings` of the creator (symbolicParameters.md
  P1) holds one variable per name, as SymbolicSystem checks now; a `VariableSet` the user fills is accepted as the
  scope.
- **B** not as the default: the second run of a script, the tests in one process and the library's own variables
  would each need a reset or an exemption, and a refused second `Real('m', ...)` protects only against what A and C
  catch where it matters. If wanted, B as a switch that is off by default, `esym.SetUniqueNames(True)`, for scripts that
  want the check at creation.
- Readability of a printed expression with two variables of one name stays as it is under A and C; a variable keeps
  its name.

## 4. To be decided

- **N1** A: the missing check in `esym.Function`. Proposed.
- **N2** C: the parameters of a system as its scope of names, a `VariableSet` accepted as that scope. Proposed, with
  RG12.67.2.
- **N3** B: a global scope of names - not as the default; as a switch off by default if wanted.
