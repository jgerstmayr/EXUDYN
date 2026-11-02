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

## 3. The maintainer's input of 2026-11-02 (second round)

- The parameter bindings are in the symbolic layer (symbolicParameters.md), so there is no scope in MainSystem yet. A
  scope of the model would be valuable, e.g. `mbs.special.symbolicVariables` or a better place: its variables the
  only ones a symbolic user function may read, bound to the mbs so that they are not lost, belonging to the model,
  exported with it (dict, pickle), easy to check, new with every new mbs, cleared by `mbs.Reset()`. Close to option C,
  but with MainSystem, because the regular symbolic user functions read such variables as well, not only the user
  items.
- Each named `esym.Real(name, value)` could require a scope; inside a function a named variable makes no sense.
  Without a scope it could go to `esym.variables` - a change of the behaviour, to be documented.
- The old behaviour of `esym.Real(name, value)` could come back with a special deprecation flag.
- `esym.Real(name, value)` could take the variable of the name if it exists (B'), so that a script also runs the
  second time.
- Symbolic was used rarely, as there were few uses for it: a good long-term solution is wanted, a complete change
  rather than big compromises.

## 4. Packages

Three packages, each complete in itself, instead of the single decisions N1-N3 of the first round.

### Package 1: the variables belong to a scope, the model has its own (recommended)

1. **Every named variable is in exactly one scope**, a `VariableSet`; a name is one variable in its scope. Unnamed
   values (`esym.Real(2.)`) and expressions belong to no scope.
2. **The model's scope**: `mbs.symbolicVariables`, a `VariableSet` held by MainSystem, created with the mbs, emptied
   by `mbs.Reset()`. A script run again in one interpreter, a notebook cell, the tests in one process each get a new
   mbs and so a new scope - no conflict between runs.
3. **Creation**: `esym.Real(name, value, scope=mbs)` gives the variable of that name in the model's scope - a new one,
   or the existing one with its value set (B' within the scope, so a cell run twice works); `scope` also takes a
   `VariableSet`. Without `scope`, a named `esym.Real` goes to `esym.variables` the same way. `VariableSet.Add` stays
   strict (a second name raises), for code that builds a scope once.
4. **What a recording reads**: a symbolic user function reads named variables only from its arguments and from the
   scope of its mbs; another named variable - one of `esym.variables` among them - is refused at the recording, with
   the message how to create it in the model's scope. `esym.Function`, without mbs, reads its inputs and
   `esym.variables`, as now. Creating a named variable while recording raises.
5. **The symbolic parameters** are the variables of `mbs.symbolicVariables`: SymbolicSystem takes them from there,
   the bindings of the user items (symbolicParameters.md) point to them, `SetParameterValues` sets them. The scope of
   the system and that of the model are one.
6. **The library's own names** - the arguments of a recording, the coordinates of SymbolicSystem, `x`, `y`, `z` of
   `exudyn.shells` - are created in scopes of their own (a local `VariableSet` per build), not in `esym.variables`.
7. **Kept with the model**: a flat program names the variables of the scope it reads; a model saved as a whole (not
   yet available) carries the scope with its values.
8. **Old behaviour by a flag**: `exudyn.special.symbolicLegacyVariables = True` - deprecated, with a warning, removed
   later - makes a named `esym.Real` a free variable as before and lets symbolic user functions read `esym.variables`.

Changes for scripts: a symbolic user function that reads `esym.variables` creates its variables with `scope=mbs`
instead; scripts that create two variables of one name in one scope get one variable. Effort: medium - the scope in
MainSystem (C++ `Symbolic::VariableSet`, pybind, Reset), the check at the recording (the walk of #3198), `scope` of
`esym.Real`, the library's local scopes, the flag, the documentation and test models.

### Package 2: one global scope with interning

Rules 1, 3 (without the model's scope), 6 and 8 of package 1: every named `esym.Real` is in `esym.variables`, a second
one of the name is the same variable with the new value; symbolic user functions read `esym.variables` as now; the
missing check of `esym.Function` (section 1) is not needed, as one name is one variable. Smaller (no change of
MainSystem), but two models in one process share their variables silently - the tests in one process, two systems
in one script, a parameter variation in threads -, and nothing ties the variables to the model they belong to.

### Package 3: checks where names identify (the first recommendation)

The missing check of `esym.Function` (N1) and the parameters of a system as its scope of names (N2); `esym.Real`
unchanged, no model scope. Smallest; two variables of one name remain possible and print alike, which is what the
maintainer wants to avoid.

## 5. Decided and implemented

- **N4** Package 1, the scope `mbs.symbolicVariables` (maintainer 2026-11-02). Implemented in RG12.73.2 (#3208):
  `esym.Real/Vector/Matrix(name, value, scope=None)`; a name of another kind or size replaces the variable (the tests
  in one process and the reference notebook reused names of different kinds); without recording a named Real is a
  number as before; the flag is `exudyn.special.deprecations.symbolicVariablesWithoutScope`, its removal #3231.

N1-N3 of the first round are answered by the package: package 1 makes N1 unnecessary (one name is one variable in a
scope), replaces N2 by the model's scope and answers N3 with scopes instead of one global one.
