# Sensor documentation (development document)

*Temporary, revision2026b step RG13.4.5 (#2721); what is common to every kind is in
[itemDefinitionsDev.md](itemDefinitionsDev.md). 8 sensors in `definitions/itemDefsSensors.py`.*

The maintainer, 2026-09-27: sensors only read output variables that are already defined, so
*"everything is 100% clear"* - except the local position on a body. They need **no equations**, a
more detailed description where one is needed, and **a general sensor section** before the first
sensor, to which every sensor refers for the general behaviour.

## 1. What a reader needs to know about a sensor

Sensors are written by hand even in scripts that use Create functions for everything else
(`mbs.AddSensor(SensorBody(...))` in the rigid body tutorial). The questions:

1. **what can it measure** - the output variables of the item it is attached to, which are on the
   item's page, not on the sensor's;
2. **where** - for a body sensor the local position $\pLocB$, in body coordinates; for a superelement
   the mesh node; for a kinematic tree the link and the local position in the link's frame;
3. **in which configuration** - current, initial, reference, visualization;
4. **where the values go** - a file, `storeInternal` and `mbs.GetSensorStoredData`, `PlotSensor`;
   how often (`sensorsWritePeriod`), what a line of the file holds.

Questions 3 and 4 are the same for every sensor.

## 2. What the pages say today

| | sensors | |
|---|---|---|
| without any text beyond the class description | 7 of 8 | all but `SensorUserFunction` |
| class description | 35 to 75 words | and **the same sentences in all of them**: *"The sensor measures ... and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file."* |
| with a MiniExample, a figure | 0, 0 | |

The page of `SensorBody` is, after the parameter tables, an empty *DESCRIPTION* heading and the list
of examples. What it shows of its own is one sentence: *"As a difference to SensorObject, the body
sensor needs a local position at which the sensor is attached to"*.

Textual findings:

- The repeated sentences (file format, SensorUserFunction) are the general section - in eight class
  descriptions today, so that a reader of the index sees them eight times and a reader of one page
  once, without the rest of what is general.
- `SensorBody` says it measures *"OutputVariableBody"*, `SensorSuperElement` *"OutputVariableSuperElement"*,
  `SensorKinematicTree` *"OutputVariableKinematicTree"* - names of C++ enums a Python user never sees.
  What a user needs is *"the output variables of the body, see the body's page"* and a link.
- `SensorMarker` lists what it can measure per kind of marker - the one sensor whose measured values
  are not on another page, because markers have no output variables (markerDefinitionsDev §4).
- Where the local position of `SensorBody` is taken from - the body's reference point, which for
  `ObjectRigidBody` may or may not be the centre of mass - is the maintainer's *"only the local
  position needs to be interpreted"*; it is not said.

## 3. The ideal sensor page

Short; no equations:

| section | contents |
|---|---|
| class description | **one** sentence of its own: what it is attached to |
| **Attached to** | the item, and what locates the point: local position (and in which frame, from which reference point), mesh node, link |
| **Measures** | *the output variables of the item it is attached to*, with a link to the kind's list; for `SensorMarker` the table of what each marker provides; for `SensorUserFunction` the user function |
| **Details** | only where there are any: the kinematic tree's link frame, the superelement's mesh node and the modes, the load sensor's value in which frame |
| **MiniExample** | written; the same few lines for all, with a different item |

## 4. The general sensor section

Before the first sensor, instead of the paragraph of the index page and the repeated sentences:

- **what a sensor is**: it reads an output variable of a node, object, body, marker or load at the
  times the solver writes output, and does not act on the system;
- **output variables**: where the list for each item is (on the item's page), `OutputVariableType`,
  what `configuration` means;
- **where values go**: the file (`fileName`, `writeToFile`, the format of a line - `[time,
  sensorValue[0], ...]` - and the header), `storeInternal` and `mbs.GetSensorStoredData`,
  `mbs.GetSensorValues`, `sensorsWritePeriod`; the files of a run are in `solution/` by default
  (#2718);
- **plotting**: `mbs.PlotSensor`, and the results monitor;
- **transforming values**: `SensorUserFunction` - once, not in eight class descriptions;
- **visualization**: sensor traces (`visualizationSettings.sensors.traces`).

## 5. Open for the maintainer

- Shorten the eight class descriptions to their own sentence once the general section exists - the
  same step, so that nothing is lost in between.
