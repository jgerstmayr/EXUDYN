# exudyn.systemUtilities: the functions of a whole system

revision2026c step RG4.25.3 (#2997). The functions that take a `MainSystem` and describe the system as a whole - its
energy, the drift of its constraints, its graph - are in one module, `python/exudyn/systemUtilities.py`, and the
quantities of the system follow one pattern, `SystemQuantity`. The user documentation is the section *Quantities of
the whole system* of the manual (`docs/manual/introductionBasics.md`, `sec-systemquantities`).

Decided by the maintainer (2026-10-16): all system-related functionality in one library file; a base class
`SystemQuantity` with derived classes for the energy and the drift; `DrawSystemGraph` split into `ComputeSystemGraph`
and the drawing; the interface of the functions introduced with 1.12/1.13 may change, as users do not use them yet;
`utilities.py` imports the new library, so that what came from it before 1.11.0 is there again. Functions that need a
solver stay in `solver.py` - `ComputeSystemDegreeOfFreedom` (the Jacobian of the constraints),
`ComputeLinearizedSystem`, `ComputeODE2Eigenvalues`.

## 1. The name

**`exudyn.systemUtilities`**, beside `basicUtilities`, `advancedUtilities` and `rigidBodyUtilities`; "system" is what
the documentation calls the `MainSystem` (`mbs`) throughout. `mainSystemUtilities` would repeat the C++ class name;
`mainSystemExtensions` stays the module of the `Create...` functions.

## 2. What belongs to it

A function belongs here if its argument is the whole system and its result describes the system - not one item, not
the creation of items, not a run of a solver:

| function | before | since | in systemUtilities |
|---|---|---|---|
| `SystemEnergy` | advancedUtilities | 1.12.236 (#2202) | `SystemEnergy(SystemQuantity)`; `ComputeSystemEnergies()` is `Compute()` |
| `LoadPotentialEnergy` | advancedUtilities | 1.12.236 | moved, used by `SystemEnergy` |
| `CreateLoadEnergySensor` | advancedUtilities | 1.12.236 | removed: `SystemEnergy(mbs, items=[load]).AddSensor(components=['loads'])` |
| `SensorUFSystemConstraintDrift` | advancedUtilities | 1.13.92 (#565) | removed: `SystemConstraintDrift(mbs).AddSensor()` |
| `ComputeConstraintResiduals` | solver.py, `mbs.` | 1.13.92 (#565) | moved; `mbs.ComputeConstraintResiduals` stays |
| `NumericalJacobian` | advancedUtilities | 1.12.217 (#2779) | moved: it varies the state of the system; the counterpart of `mbs.ComputeItem` for tests of items, in particular user items (arguments `systemCoordinates`, `velocityLevel`, `epsilon=1e-8` since #3004) |
| `DrawSystemGraph` | mainSystemExtensions, `mbs.` | before 1.11 | split: `ComputeSystemGraph`, `DrawSystemGraph`; `mbs.DrawSystemGraph` with the same arguments and return value |

Removed before the move (#3004): `ItemODE2Coordinates` - the indices of the ODE2 coordinates of an item are
`mbs.systemData.GetObjectLTGODE2` and `GetNodeLTGODE2` (C++); markers have no coordinates of their own.

Not included: the functions that need a solver (above), `FindObjectIndex`/`FindNodeIndex` (they search Python
variables), the `Create...` functions, `PlotSensor`.

## 3. SystemQuantity

```python
class SystemQuantity:
    names = []                                  #the components, fixed by the derived class
    def __init__(self, mbs, items=None, skipUnavailable=True)
    def Refresh(self)                           #collects the items again, after the system changed
    def Compute(self, configuration=exu.ConfigurationType.Current) -> np.ndarray      #one value per component
    def ComputeItems(self, configuration=exu.ConfigurationType.Current) -> list       #[(item, np.ndarray)]
    def SensorFunction(self, mbs, t, sensorNumbers, factors, configuration) -> list   #a sensorUserFunction
    def AddSensor(self, components=None, storeInternal=True, writeToFile=False, fileName='', name='') -> exu.SensorIndex
    items                                       #the contributing items
    unavailable                                 #[(item, type name, component, reason)]
```

- **Collected once**: the constructor (the system must be assembled) collects the contributing items - all, or those
  of `items`, a list of ObjectIndex, NodeIndex, LoadIndex; items that have the quantity but not with their parameters
  (a user function) go into `.unavailable` with the reason, or raise with `skipUnavailable=False`.
- **One evaluation path**: a derived class implements `_CollectItems` and `_ItemContribution(item, configuration)`,
  which returns one value per component; `ComputeItems` lists them, `Compute` reduces them with `_Reduce` - the sum,
  or for the drift the maximum. `ComputeItems` returns a list of pairs, because the item indices of Exudyn compare
  and hash by identity, not by value.
- **Items, not output variables of the system**: the contributions come from what the C++ items provide
  (`GetObjectOutputBody(..., KineticEnergy)`, `ComputeItem(..., ConstraintDrift)`), so a user item (ObjectUserBody)
  takes part as soon as it provides the output.
- **Configuration**: as the output functions take it; the kinetic energy from the mass matrix exists in the current
  configuration only, and the item says so when asked for another.
- **Sensors**: `AddSensor` adds one `SensorUserFunction` with the bound `SensorFunction` (or a function of the chosen
  components); the sensor holds the reference to the object. `mbs.Assemble()` after it, as for every added item.

### SystemEnergy

`names = ['kinetic', 'potential', 'loads', 'total']`. The items: objects with `KineticEnergy` or `PotentialEnergy`
among their output variables (as `mbs.Inspect` lists them), loads with a potential (`LoadPotentialEnergy`); `total`
the sum of the three, also per item.

### SystemConstraintDrift

`names = ['positionDrift', 'velocityResidual']`. The items: the objects and nodes for which `mbs.ComputeItem(item)`
lists `ComputeItemType.ConstraintDrift` - constraints and nodes with algebraic equations (Euler parameters). The
contribution of an item is the largest absolute value of its `ConstraintDrift` and of its `AlgebraicEquations_t`; the
reduction is the maximum. An item without a velocity level (ObjectJointSliding) is in `.unavailable` for
`velocityResidual`, whose value is then NaN. The residual vectors themselves come from `mbs.ComputeConstraintResiduals`
and `mbs.ComputeItem`.

The solver check of RG4.25.1 stays in C++ - it runs in every step without Python - and uses the same definition
(`ComputeItemType.ConstraintDrift`: position level, velocity-level constraints left out).

### Later quantities in the same form

The linear and angular momentum of the bodies (needs an output variable of the bodies, not available now), the work of
the loads, the power of the dampers.

## 4. The system graph

`ComputeSystemGraph(mbs, showLoads=True, showSensors=True, useItemNames=False, useItemTypes=False,
addItemTypeNames=True, multiLine=False, addItemData=False, addAnnotations=False) -> networkx.Graph`: one graph node
per item, named as `DrawSystemGraph` labels it, with the attributes `basicType` and `color` (and `type`, `ID`, `name`,
`annotations` on request), and the edges object - node, marker - object or node, connector - marker, load - marker,
sensor - object or node, each with its `color`. `DrawSystemGraph(mbs, ...)`: `ComputeSystemGraph`, the layout and the
matplotlib drawing, with the arguments and the return value it had before; the graphs of both are the same as those
of 1.13.6 (checked for the label options). networkx and matplotlib are imported in the functions only (optional
dependencies, rule 6).

## 5. Modules and imports

- `systemUtilities` imports `exudyn`, `numpy`, `exudyn.itemInterface` and the extension registry; never `utilities`
  (circular).
- `ComputeConstraintResiduals` and `DrawSystemGraph` are attached with `@extends(exudyn.MainSystem)` in
  `systemUtilities`; `mainSystemExtensions` imports the module before its `install()`, so `mbs.` has them after
  `import exudyn`.
- `utilities.py` star-imports `systemUtilities` and adds its `__all__`; `DrawSystemGraph` comes with
  `from exudyn.utilities import *` again, as before 1.11 (`tools/checkUserScripts.py` no longer reports it).
- the generated page `docs/generated/pythonUtilities/systemUtilities.md` (`utilityDocsModel.filesParsed`).

## 6. Tests

`energiesTest.py` and `constraintDriftTest.py` use the new classes, with unchanged results; `systemUtilitiesTest.py`
tests the selection of items, the contributions, the components of a sensor, `.unavailable`, the drift against
`mbs.ComputeConstraintResiduals`, and `ComputeSystemGraph` where networkx is installed (not in `venvExuP313`; the
per-version environments `venvP312` and `venvP313` have it).
