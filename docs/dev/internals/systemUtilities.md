# exudyn.systemUtilities: the functions of a whole system

The functions that take a `MainSystem` and describe the system as a whole - its energy, the drift of its constraints,
its graph - are in one module, `python/exudyn/systemUtilities.py`, and the quantities of the system follow one
pattern, `SystemQuantity` (#2997). The user documentation is the section {ref}`sec-systemquantities` of the manual and
the generated page of the module.

## 1. What belongs to it

A function belongs here if its argument is the whole system and its result describes the system - not one item, not
the creation of items, not a run of a solver:

| function | what |
|---|---|
| `SystemEnergy(SystemQuantity)` | kinetic, potential and load energy of the items |
| `SystemConstraintDrift(SystemQuantity)` | the drift of the constraints at position and velocity level |
| `LoadPotentialEnergy` | the potential of a load, used by `SystemEnergy` |
| `ComputeConstraintResiduals` | the residuals of the constraints; also `mbs.ComputeConstraintResiduals` |
| `NumericalJacobian` | varies the state of the system: the counterpart of `mbs.ComputeItem` for tests of items, in particular user items (`systemCoordinates`, `velocityLevel`, `epsilon=1e-8`, #3004) |
| `ComputeSystemGraph`, `DrawSystemGraph` | the graph of the items, and its drawing; also `mbs.DrawSystemGraph` |

Not in it: the functions that need a solver - `ComputeSystemDegreeOfFreedom` (the Jacobian of the constraints),
`ComputeLinearizedSystem`, `ComputeODE2Eigenvalues` stay in `solver.py` -, `FindObjectIndex`/`FindNodeIndex` (they
search Python variables), the `Create...` functions (`mainSystemExtensions`), `PlotSensor`. The indices of the ODE2
coordinates of an item are `mbs.systemData.GetObjectLTGODE2` and `GetNodeLTGODE2`.

## 2. SystemQuantity

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

**SystemEnergy**: `names = ['kinetic', 'potential', 'loads', 'total']`. The items: objects with `KineticEnergy` or
`PotentialEnergy` among their output variables (as `mbs.Inspect` lists them), loads with a potential
(`LoadPotentialEnergy`); `total` the sum of the three, also per item.

**SystemConstraintDrift**: `names = ['positionDrift', 'velocityResidual']`. The items: the objects and nodes for which
`mbs.ComputeItem(item)` lists `ComputeItemType.ConstraintDrift` - constraints and nodes with algebraic equations (Euler
parameters). The contribution of an item is the largest absolute value of its `ConstraintDrift` and of its
`AlgebraicEquations_t`; the reduction is the maximum. An item without a velocity level (ObjectJointSliding) is in
`.unavailable` for `velocityResidual`, whose value is then NaN. The check of the solver runs in C++ in every step and
uses the same definition (`ComputeItemType.ConstraintDrift`: position level, velocity-level constraints left out).

## 3. The system graph

`ComputeSystemGraph(mbs, showLoads=True, showSensors=True, useItemNames=False, useItemTypes=False,
addItemTypeNames=True, multiLine=False, addItemData=False, addAnnotations=False) -> networkx.Graph`: one graph node
per item, named as `DrawSystemGraph` labels it, with the attributes `basicType` and `color` (and `type`, `ID`, `name`,
`annotations` on request), and the edges object - node, marker - object or node, connector - marker, load - marker,
sensor - object or node, each with its `color`. `DrawSystemGraph(mbs, ...)`: `ComputeSystemGraph`, the layout and the
matplotlib drawing. networkx and matplotlib are imported in the functions only (optional dependencies).

## 4. Modules and imports

- `systemUtilities` imports `exudyn`, `numpy`, `exudyn.itemInterface` and the extension registry; never `utilities`
  (circular).
- `ComputeConstraintResiduals` and `DrawSystemGraph` are attached with `@extends(exudyn.MainSystem)`;
  `mainSystemExtensions` imports the module before its `install()`, so `mbs.` has them after `import exudyn`.
- `utilities.py` star-imports `systemUtilities` and adds its `__all__`, so `from exudyn.utilities import *` gives
  `DrawSystemGraph` as before 1.11.
- The generated page `docs/generated/pythonUtilities/systemUtilities.md` (`utilityDocsModel.filesParsed`).

## 5. Decisions (maintainer 2026-10-16)

| decision | why |
|---|---|
| all functions of the whole system in one module, `exudyn.systemUtilities` | "system" is what the documentation calls the `MainSystem`; `mainSystemUtilities` would repeat the C++ class name |
| a base class `SystemQuantity` with derived classes for the energy and the drift | one way to collect, compute, reduce and sense |
| `DrawSystemGraph` split into `ComputeSystemGraph` and the drawing | the graph without matplotlib, testable |
| the functions that need a solver stay in `solver.py` | they run a solver |

## 6. Tests

`energiesTest.py` and `constraintDriftTest.py` use the classes; `systemUtilitiesTest.py` tests the selection of items,
the contributions, the components of a sensor, `.unavailable`, the drift against `mbs.ComputeConstraintResiduals`, and
`ComputeSystemGraph` where networkx is installed (not in `venvExuP313`; `venvP312` and `venvP313` have it).
