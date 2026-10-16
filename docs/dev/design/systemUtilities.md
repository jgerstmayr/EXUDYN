# exudyn.systemUtilities: the functions of a whole system - concept

revision2026c step RG4.25.3 (#2997). The functions that take a `MainSystem` and describe the system as a whole - its
energy, the drift of its constraints, its graph, its degrees of freedom - are spread over `advancedUtilities.py`,
`solver.py` and `misc/mainSystemExtensions.py`, and the two of RG4.25 are built differently: the energy is a Python
class, the drift a C++ check of the solver with a Python function and a sensor function. This concept puts them into
one library with one pattern.

Decided by the maintainer (2026-10-16): all system-related functionality in one library file; a base class
`SystemQuantity` with derived classes for the energy and the drift, "exactly how it should work"; `DrawSystemGraph`
split into `ComputeSystemGraph` and the drawing, `mainSystemExtensions` only imports and wraps; the interface of 1.13
may change for the functions introduced with 1.12/1.13, as users do not use them yet; `utilities.py` imports the new
library, so that what came from it before 1.11.0 stays where it was.

## 1. The name

**`exudyn.systemUtilities`**, file `python/exudyn/systemUtilities.py`. It sits beside `basicUtilities`,
`advancedUtilities` and `rigidBodyUtilities`; "system" is what the documentation calls the `MainSystem` (`mbs`)
throughout. `mainSystemUtilities` would repeat the C++ class name and is longer; `mainSystemExtensions` stays the module
that attaches functions to `MainSystem`.

## 2. What belongs to it

A function belongs here if its argument is the whole system and its result describes the system - not one item, not
the creation of items, not a run of a solver:

| function | now in | since | in systemUtilities |
|---|---|---|---|
| `SystemEnergy` | advancedUtilities | 1.12.236 (#2202) | `SystemEnergy(SystemQuantity)` |
| `LoadPotentialEnergy` | advancedUtilities | 1.12.236 | kept, used by `SystemEnergy` |
| `CreateLoadEnergySensor` | advancedUtilities | 1.12.236 | removed: `SystemEnergy(mbs, items=[load]).AddSensor(components=['loads'])` |
| `SensorUFSystemConstraintDrift` | advancedUtilities | 1.13.92 (#565) | removed: `SystemConstraintDrift(mbs).SensorFunction` |
| `ComputeConstraintResiduals` | solver.py, `mbs.` | 1.13.92 (#565) | moved; `mbs.ComputeConstraintResiduals` stays |
| `ItemODE2Coordinates`, `NumericalJacobian` | advancedUtilities | 1.12.217 (#2779) | moved: the state of the system, for tests of items |
| `ComputeSystemDegreeOfFreedom` | solver.py, `exu.`, `mbs.` | before 1.11 | moved; solver.py imports it, so `exudyn.solver.` and `exu.` keep it |
| `DrawSystemGraph` | mainSystemExtensions, `mbs.` | before 1.11 | split: `ComputeSystemGraph`, `DrawSystemGraph`; `mbs.DrawSystemGraph` a thin wrapper with the same arguments |

Not included: `ComputeLinearizedSystem` and `ComputeODE2Eigenvalues` (they set up a solver and stay in solver.py),
`FindObjectIndex`/`FindNodeIndex` (they search Python variables), the `Create...` functions, `PlotSensor`.

## 3. SystemQuantity

```python
class SystemQuantity:
    """a quantity of the whole system with named components - an energy, a drift - from the contributions of its items"""
    names = []                                  #the components, fixed by the derived class

    def __init__(self, mbs, items=None, skipUnavailable=True):
        #collects the contributing items once (the system must be assembled): all, or those of items (a list of
        #ObjectIndex, NodeIndex, LoadIndex); items that have the quantity but not with their parameters (a user function)
        #go into .unavailable with the reason, or raise if skipUnavailable=False
    def Refresh(self):                          #collects again, after the system changed
    def Compute(self, configuration=exu.ConfigurationType.Current) -> np.ndarray:
        #one value per component, in the order of names
    def ComputeItems(self, configuration=exu.ConfigurationType.Current) -> dict:
        #the contribution of each item: {item index: np.ndarray}
    def SensorFunction(self, mbs, t, sensorNumbers, factors, configuration) -> list:
        #a sensorUserFunction: Compute(configuration)
    def AddSensor(self, components=None, storeInternal=True, writeToFile=False, fileName='', name='') -> exu.SensorIndex:
        #a SensorUserFunction recording the components (all, or those named)
    unavailable                                 #[(item, type name, what, reason)]
```

- **One evaluation path**: `Compute` sums or reduces the item contributions that `ComputeItems` gives; a derived class
  implements only `_CollectItems` and `_ItemContribution(item, configuration)` and, where the reduction is not a sum,
  `_Reduce`.
- **Items, not output variables of the system**: the contributions come from what the C++ items provide
  (`GetObjectOutputBody(..., KineticEnergy)`, `ComputeItem(..., ConstraintDrift)`), so a user item (ObjectUserBody)
  takes part as soon as it provides the output.
- **Configuration**: as the output functions take it; the kinetic energy from the mass matrix exists in the current
  configuration only, which `Compute` says when asked for another.
- **Sensors**: `AddSensor` adds one `SensorUserFunction` with the bound `SensorFunction`; the object must stay alive
  with the model, which `mbs.variables` keeps (the same as the symbolic user functions now).

### SystemEnergy

`names = ['kinetic', 'potential', 'loads', 'total']`. The items: objects with `KineticEnergy` or `PotentialEnergy`
among their output variables (as `mbs.Inspect` lists them), loads with a potential (`LoadPotentialEnergy`); `total`
the sum of the three. As the present class computes it; the interface changes from `ComputeSystemEnergies()` to
`Compute()`.

### SystemConstraintDrift

`names = ['positionDrift', 'velocityResidual']`. The items: the constraints and the nodes with algebraic equations
(Euler parameters), from `mbs.ComputeItem(item)` - what applies to the item. `ComputeItems` gives the residual vectors
(`ComputeItemType.ConstraintDrift` and `AlgebraicEquations_t`), `Compute` the largest absolute values (the reduction is
a maximum, not a sum). An item without a velocity level (ObjectJointSliding) is in `.unavailable` for that component,
its value NaN.

The solver check of RG4.25.1 stays in C++ - it runs in every step without Python - and uses the same definition
(`ComputeItemType.ConstraintDrift`: position level, velocity-level constraints left out); the manual says this once for
both.

### Later quantities in the same form

The linear and angular momentum of the bodies (needs an output variable of the bodies, not available now), the work of
the loads, the power of the dampers.

## 4. The system graph

`ComputeSystemGraph(mbs, showLoads=True, showSensors=True, useItemNames=False, useItemTypes=False, addItemData=False,
addAnnotations=False) -> networkx.Graph`: the nodes (one per item, with the attributes item type, number, name,
label, color, and the item data on request) and the edges (object - node, marker - object or node, connector -
marker, load - marker, sensor - item). `DrawSystemGraph(mbs, ...)`: `ComputeSystemGraph`, the layout and the matplotlib
drawing, with the present arguments and return value. networkx and matplotlib are imported in the two functions only
(optional dependencies, rule 6). The graph alone serves an analysis without drawing - connected parts, items without
connection - which now needs `showGraph=False` and the return value.

## 5. Modules and imports

- `systemUtilities` imports `exudyn`, `numpy`, `exudyn.itemInterface`; never `utilities` (circular).
- `mainSystemExtensions` imports `ComputeSystemGraph`, `DrawSystemGraph` and attaches `mbs.DrawSystemGraph`;
  `ComputeConstraintResiduals` and `ComputeSystemDegreeOfFreedom` are attached with `@extends(exudyn.MainSystem)` in
  `systemUtilities` itself, as solver.py does now.
- `utilities.py` adds `systemUtilities` to its star imports and to `__all__`; `advancedUtilities` drops the moved
  functions (they came with 1.12), `solver.py` imports `ComputeSystemDegreeOfFreedom` back.
- the generated page `docs/generated/pythonUtilities/systemUtilities.md` comes with the module; the manual gets one
  section on system quantities (energy, drift, sensors), and the section on the drift in solver.md refers to it.

## 6. Tests

`energiesFlexibleBodiesTest.py` and `constraintDriftTest.py` move to the new classes (their values stay); a test model
for `ComputeSystemGraph` (the number of nodes and edges of a small model, no drawing); the pytest of the moved
functions follow them.

## 7. Steps

1. The module with `SystemQuantity`, `SystemEnergy`, `SystemConstraintDrift`, the moved functions, the imports;
   the two test models; `SensorUFSystemConstraintDrift` and `CreateLoadEnergySensor` removed.
2. `ComputeSystemGraph` and `DrawSystemGraph`, the wrapper in `mainSystemExtensions`; a test model.
3. The manual section; #2997 resolved with step 1.
