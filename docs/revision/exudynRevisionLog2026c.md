# Exudyn Revision Log 2026c

What was done and found in the steps of [`exudynRevisionPlan2026c.md`](exudynRevisionPlan2026c.md): one entry per
closed step, with the anchor of its number (`<a id="rg10-30"></a>`), a heading `### RG10.30 — title (date, #issue)`,
and what the work turned out to be - the measurements, the surprises, what was decided against. Details that the issue
already holds are not repeated.

The logs of the closed plans, [`exudynRevisionLog2026b.md`](exudynRevisionLog2026b.md) and
[`exudynRevisionLog2026.md`](exudynRevisionLog2026.md), are not appended to any more. **A closed entry is never
edited**; a fact that turns out wrong is corrected in [`exudynRevisionInfo2026.md`](exudynRevisionInfo2026.md), with a
dated note here.

<a id="rg10-30"></a>
### RG10.30 — the plan 2026c and planManager.py (2026-10-10, #2929)

*(Maintainer 2026-10-10: "The next step is to close the revision plan b, as it also got messy ... keep the short
information in the plan, move the rest to the logs - open a new RevisionPlan2026c ... the nomenclature should be
**OPEN_RG2.13.4** / **CLOSED_RG3.1.4** ... create a local python code in docs/revision/planManager.py ... when the tool is
ready, move the open steps of revision2026b to revision2026c, possibly keeping the numbering up to RG11".)*

- **Plan 2026b closed.** It had grown to 2419 lines: 447 blocks (groups, steps, sub-steps), a few steps open in name
  with all their work done, and one number given twice (two different steps `RG2.6`). It is now one line per step -
  status, title, issues and where an open step went - 519 lines. The full text of every step as it stood is in
  [its log](exudynRevisionLog2026b.md#plan-at-closing), followed by the introductions of the groups and the last
  section *Next steps recommended*. The anchors `rgX-Y` of the plan stay, because the first plan links to them.
- **Closed at the closing**, open in the text but done or decided:
  - RG1.1 and RG1.4, the 1.13 release;
  - RG2.4, the manual GUI check, done on all three platforms;
  - RG4.15 and RG4.19, all their sub-steps done or decided;
  - RG4.17, #2763 closed;
  - RG6.14, confirmed on Ubuntu;
  - RG9.6.3 (#2768) and RG12.29 (#2203);
  - RG16.10.5, RG16.12 and RG16.13;
  - the sub-steps of steps already DONE.
- **Plan 2026c** has four sections: how to, next steps, open steps (generated), groups (a generated table, then the
  groups). A group or step is a heading `## **OPEN_RG4** ...` / `### **OPEN_RG4.21** ...`, whose level follows from the
  number; the marker appears only there. The title says what the step does. Groups RG1 to RG11 keep their numbers,
  and so does every step carried over.
- **Renumbered or regrouped:**
  - the second RG2.6 of 2026b (three old test models) is RG2.9;
  - what was left of groups 12 to 18 is in RG11:
    - RG12.44 to RG12.46 → RG11.4;
    - RG13.9 → RG11.5;
    - RG14.3 → RG11.6;
    - RG15 (#2746) → RG11.7;
    - RG18.1 → RG11.8 (sub-steps renumbered 1 to 17);
    - RG18.2 → RG11.9.
- **"Not decided to be resolved"**, a table at the end of 2026b, is now a sentence in the introduction of the group each
  item belongs to (RG4, RG6, RG9, RG11).
- **`docs/revision/planManager.py`** - `check`, `update`, `list`, `show`, `add`, `set`:
  - It checks:
    - the four sections once each and in order;
    - every heading of the groups section is a group or step of the right level;
    - numbers unique, parents present, siblings ascending;
    - a closed step has no open one below it, and a group is open exactly while a step of it is;
    - markers only in headings;
    - every `RGx.y` in the text exists, unless it is cited as "revision2026b step ...";
    - the next steps name only open steps;
    - the generated parts are current.
  - It warns:
    - for an open step whose issues are all closed (an issue cited as "from #707" or "since #2869" is context, not work);
    - for a closed step longer than about 12 lines.
  - `add` gives the next free number, also past the numbers revision2026b gave.
  - Every writing command rewrites the generated parts and checks.
- Tried on a copy:
  - `add RG4` gave RG4.27, because 2026b used up to RG4.26;
  - opening an empty group and citing a missing step were both refused.

<a id="rg3-45"></a>
### RG3.45 — the user documentation after the 1.13 review (2026-10-10, #2932-#2937)

*(Maintainer 2026-10-10, a list collected after the release.)*

- **#2932**: four references in *Getting started* were written as LaTeX `\url{...}`, which the PDF printed as text.
  They are now Markdown links. `checkMathMacros.py` refuses `\url{` and `\href{` in text, as it refuses a LaTeX
  accent (#2908).
- **#2933**: *Goals of Exudyn*.
  - The first development phase (2019-2023) says "allows to add user functions for items and solvers in Python"
    instead of user-defined objects and solvers in C++ and Python.
  - A **second development phase (2024-2026)** names what it brought:
    - specific connectors and contact;
    - 3D beams and plates;
    - symbolic user functions;
    - `exu.HT` with energies, `Inspect` and `ComputeItem`;
    - the 2026 revision: tested wheels for all four platforms, Markdown documentation, notebooks.
  - The **future goals (2027-2029)**: Python and C++ user elements, advanced contact formulations, automatic step
    size selection for second order solvers, export of the equations, GPU support.
- **#2934**: the raytracer draws item texts; the note that it does not is gone from `introductionAdvanced.md`.
- **#2935**: the tetrahedron of the graphics notebook has its fourth point off the corner (`[2.1,0.2,0.6]`), so it no
  longer looks like a corner of a cube, and `openGL.light0.shadow = 0.3` gives the shapes their depth. The notebook
  was run again.
- **#2936**: the Twitter link is gone from `README.rst`, the only place it was.
- **#2937**: the six images of `README.rst` are in two rows of three (a line block), with widths given in pixels and
  the second row smaller. GitLab showed them too large; how GitLab renders the widths can only be seen there, and the
  maintainer adjusts them.

<a id="rg6-18"></a>
### RG6.18 — the SolutionViewer: the PreStepUserFunction, and the space key (2026-10-10, #2930, #2931)

- **#2930**:
  - `SolutionViewer(..., callPreStepUserFunction=False)`. With True, the `PreStepUserFunction` of the system is called
    after `SetSolutionState` with the time of the shown row (column 0 of the solution) before the step is drawn, also
    when the viewer wraps around to row 0.
  - The function is read with `mbs.GetPreStepUserFunction()`, which gives `{'function': ..., 'type': 'Python'}` or
    `0`. A symbolic user function is not called.
  - Example `python/Examples/solutionViewerPreStepUserFunction.py`: a ground brick with its frame moves along a screw,
    its `referenceHT` set as an `exu.HT` in the `PreStepUserFunction`, beside a pendulum. Checked without a window:
    the call with the time of row 50 (t = 1) puts the ground at z = 0.0625. On screen only the maintainer can check it.
- **#2931**:
  - Tk's class binding of a button runs its command on the space key when the button has the focus. On macOS the Run
    button has it, so the space key toggled twice: once through the button, once through the binding of the window.
    On Windows the focus stays on the window, which is why it worked there.
  - `InteractiveDialog.OnSpaceKey` toggles only when the event did not come from the Run button, then calls
    `userSpaceKey(dialog, stopped)`.
  - The SolutionViewer uses `userSpaceKey` to set the mode: the space key starts in *Continuous run* and stops in
    *Static*. To be checked on macOS.

<a id="rg10-29-1"></a>
### RG10.29.1, RG10.29.2 — the stubs, and large files (2026-10-10, #2915, #2916)

- **#2915**:
  - `createStubFiles.py`, which setup.py runs in every build, wrote `__init__.pyi` and `symbolic.pyi` every time, with
    the line endings of the platform. The Linux build in Docker runs on the Windows checkout, so the stubs had LF
    endings afterwards, and git shows a working copy with only LF as modified (measured: the same content, LF instead
    of CRLF, gives `M`). The tree was dirty, and `exudev release --tag` refused to tag - twice at 1.13.
  - A stub is now written only when its text changes, compared in text mode, so CRLF and LF compare equal.
- **#2916**:
  - `*.synctex` is ignored beside `*.synctex.gz`.
  - The pre-commit hook refuses a new file above 6 MB: build output, or deliberate - then `--no-verify`. The largest
    tracked files are matrices of an example of up to 10 MB.
  - Tested with a staged 7 MB file: refused, with its name and size.

<a id="rg10-31"></a>
### RG10.31.1 to RG10.31.4 — exudev scripts --fix rewrites what it can decide (2026-10-10, #2938)

*(Maintainer 2026-10-10: "The exudev scripts checker reports, but does not fix problems yet (at least as far as
possible). In some cases, it would be probably possible to automatically fix ... Ask for decisions in the chat.")*

- `exudev scripts` had no `--fix` at all. `tools/checkUserScripts.py --fix` rewrote only the renamed settings, item
  parameters and Create arguments (#2813, #2814, #2863). The driver now passes `--fix`.
- `--fix` now also writes:
  - the imports a star import no longer provides (`import numpy as np`, `from math import sin`, ...), the imports of
    the names that moved to another module, and `import exudyn.<submodule>`. Each import line goes after the star
    import from exudyn, marked `#added by exudev scripts --fix`, and only if the script has neither the line nor the
    name;
  - the removed vector helpers:
    - `NormL2`, `VSum`, `VMult`, `Vec2Tilde`, `Tilde2Vec`, `eye2D`, `eye3D` renamed;
    - `VAdd(a, b)` → `(np.array(a) + np.array(b))`, `ScalarMult(s, v)` → `s*np.array(v)`, `DiagonalMatrix(n, value=x)`
      → `x*np.eye(n)`;
  - the `GraphicsData...` aliases → `graphics....`, and `color4red` → `graphics.color.red`;
  - `DrawSystemGraph(mbs, ...)` → `mbs.DrawSystemGraph(...)`, and the same for the other functions that became
    methods of the MainSystem;
  - the deprecated module functions:
    - `exu.SolveDynamic(mbs, s)` → `mbs.SolveDynamic(s)`, and `SolveStatic`, `ComputeODE2Eigenvalues`;
    - `exu.StartRenderer()` → `SC.renderer.Start()` and the other renderer functions, if the script has exactly one
      SystemContainer;
    - `exu.SetOutputPrecision(8)` → `exu.config.outputPrecision = 8` and the other settings functions;
    - `exu.InfoStat` → `exu.special.InfoStat`;
  - the deprecated SystemContainer methods: `SC.GetRenderState()` → `SC.renderer.GetState()` and the others, and
    `mbs.WaitForUserToContinue()` → `SC.renderer.DoIdleTasks()`;
  - the old default solution file `'coordinatesSolution.txt'` → `'solution/coordinatesSolution.txt'`.
- A call is rewritten from the source of its arguments (`ast.get_source_segment`). One with `*` or `**` arguments, or
  keywords where positions are expected, is left reported.
- Replacements that overlap - a helper inside the argument of another - are done in up to three passes. A fixed source
  that does not parse raises before anything is written.
- **A bug of the renames of #2863**: a deprecated argument whose replacement takes another value was renamed with its
  value. `eulerParametersRef=ep` became `rotationMatrixRef=ep`: Euler parameters given as a rotation matrix. Now
  `rotationMatrixRef=EulerParameters2RotationMatrix(ep)`, from the table `argumentValueFixes`.
- Measured on copies of `python/Examples` and `python/TestModels` (378 scripts): 56 findings before `--fix`, 8 after.
  The rest:
  - `MecanumXYphi2WheelVelocities`, `BrickXYZ`, `rotationMarker0` - deprecated with another interface;
  - the deliberate deprecation tests.
- A script with every kind of finding, fixed: 26 fixes, nothing left to report, and it runs until `DrawSystemGraph`,
  which needs networkx. Three tests in `test_checkUserScripts.py`.
- Open, by decision of the maintainer (RG10.31.5 to RG10.31.8):
  - files written beside the script;
  - statements and arguments without effect;
  - library functions whose arguments change (`BrickXYZ` → `Brick`);
  - in place or with a backup.

<a id="rg10-31-5"></a>
### RG10.31.5 to RG10.31.8 — the decisions on what --fix leaves open (2026-10-10, #2939)

*(Maintainer 2026-10-10, asked in the chat:)*

| question | decision |
|---|---|
| files written without a directory | report only |
| statements and arguments that have no effect | "basically delete them", but keep `SC.AttachToRenderEngine` and `SC.DetachFromRenderEngine`, which should do something |
| deprecated library functions whose arguments change | rewrite `BrickXYZ` |
| how `--fix` writes | in place, with a `.bak` |

- Deleted by `--fix`:
  - a standalone assignment to a dummy setting (`openGL.light0ambient`) or a removed one (`interactive.openVR`), with
    its line; `pass` if it was the only statement of its block; nothing if it shares a line;
  - a removed argument (`rBoundingSphere`), with its comma;
  - a visualization parameter of an item that draws nothing - `VNode1D(show=False)` and the key `'Vshow'` / `'VdrawSize'`
    of an item dictionary.
- `AttachToRenderEngine`, `DetachFromRenderEngine` and `renderer.Detach` stay as they are, and stay reported.
- `graphics.BrickXYZ(x0, y0, z0, x1, y1, z1, ...)` → `graphics.Brick(centerPoint=[...], size=[...], ...)`: computed
  where the corners are numbers, written as expressions where they are not; the other arguments are kept.
- The original of a fixed script is kept once as `<script>.py.bak`; an existing `.bak` is not overwritten.
- On the copies of the examples and test models: 6 findings left, all deprecations without a one-to-one replacement
  (`MecanumXYphi2WheelVelocities`, `rotationMarker0/1`). Two more tests.

<a id="rg6-18-3"></a>
### RG6.18.3 — the first space key of a running SolutionViewer (2026-10-10, #2940)

*(Maintainer 2026-10-10: "it does not work at the first time if it starts already in the 'run' mode but at 'Static'.
Then it should first switch to 'continuous run' first, without switching to stop.")*

- The SolutionViewer starts running (`runOnStart=True`) in *Static* (`runMode=2`). The space key of #2931 toggled first
  and then set the mode, so the first press stopped the running viewer.
- `InteractiveDialog(userSpaceKey=F)` now calls `F(dialog)` **instead of** starting or stopping, and `F` calls
  `dialog.StartSimulation()` where it starts or stops. The SolutionViewer:
  - running in *Static* or *One cycle*: switch to *Continuous run*, keep running;
  - otherwise: start in *Continuous run*, or stop in *Static*.
- The double toggle on macOS is now handled where it starts: the Run button gets its own `<space>` binding. It comes
  before Tk's class binding of the button and ends with `'break'`, so the key acts once - also where the button has the
  focus. To be checked on screen.

<a id="rg10-31-9"></a>
### RG10.31.9 — GraphicsDataRectangle and GraphicsDataOrthoCubeLines as graphics.Brick (2026-10-10, #2941)

*(Maintainer 2026-10-10: "if GraphicsDataRectangle stays in a script, it is currently not found. But taking the first 4
args, it could be easily converted into a graphics.Brick command, setting Z-size to 0 addFaces=False and
addEdges=True.")*

- Both were reported, as deprecated library functions and, with `from exudyn.utilities import *`, as names that star
  import no longer provides; neither was fixed. Checked with the three ways a script reaches them: the star import of
  `exudyn.utilities`, the star import of `exudyn.graphicsDataUtilities`, and `gdu.GraphicsDataRectangle`.
- `--fix` now writes:
  - `GraphicsDataRectangle(x0, y0, x1, y1, color)` → `graphics.Brick(centerPoint=[cx, cy, 0], size=[w, h, 0],
    addFaces=False, addEdges=True, edgeColor=color)`;
  - `GraphicsDataOrthoCubeLines(x0, y0, z0, x1, y1, z1, color)` the same with its z.
- The rewrite shares the corner arithmetic of `BrickXYZ`: numbers where the corners are numbers, expressions where not.
  `import exudyn.graphics as graphics` is added if missing.
- A brick with size 0 in z and no faces gives 8 points and 12 edges (4 of them of length 0), and it draws as the
  rectangle; checked as ground graphics. One more test.

<a id="rg10-31-10"></a>
### RG10.31.10 — what the exit code of exudev scripts --check means (2026-10-10, #2942)

*(Maintainer 2026-10-10: "when running the check script, it gives an error: .\exudev.bat scripts --check
..\..\teaching\Innsbruck\Maschinendynamik_WS2019\")*

- Measured: no exception. The run checked 79 scripts of the lecture (16 other .py files skipped) and found 504
  findings in 75 of them; with `--check` that is exit code 1, as designed. `conda run` then printed
  `ERROR conda.cli.main_run:execute(125): ... failed. (See above for error)`, and the summary said `FAILED`. Together
  they read as a crash of the tool.
- Now:
  - the checker ends with `--check: the scripts need changes (504 finding(s)) - exit code 1; --fix rewrites what can be
    rewritten`;
  - the summary of `exudev scripts` says `FAILED: scripts need changes`;
  - every exudev step that runs through `conda run` and fails says below conda's line that it only passes on the
    exit code.

<a id="rg10-31-11"></a>
### RG10.31.11 — conda's line gone from the scripts check, and its exit code kept (2026-10-10, #2943)

*(Maintainer 2026-10-10, after RG10.31.10: the output of `exudev scripts --check` on the lecture still ends with
`ERROR conda.cli.main_run ... failed`.)*

- The explaining note of #2942 stays for every other exudev step. The scripts step instead reads its output through
  `runner.WithoutCondaError`, a line filter that passes every line except conda's. Its `tailLength = 0`, so a failure
  does not repeat the last lines; `RunFiltered` takes `tailLength` from the filter.
- **A bug of #2942**: `RunSteps` counted a step as failed only for the verdict `'FAILED'`. With
  `'FAILED: scripts need changes'`, `exudev scripts --check` ended with exit code 0. Every verdict that starts with
  `FAILED` now sets exit code 1. Measured on the lecture: exit code 1, the output without conda's line.

<a id="rg10-31-12"></a>
### RG10.31.12 — AddRigidBody to mbs.CreateRigidBody (2026-10-10, #2944)

*(Maintainer 2026-10-10: "many of my old scripts contain AddRigidBody, which shall be replaced in CreateRigidBody. The
only thing are the return values ... get the node number from the body with mbs.GetObject(objectIndex)['nodeNumber'].
Also the args are slightly different.")*

- `AddRigidBody(mainSys, inertia, nodeType, position, velocity, rotationMatrix, rotationParameters, angularVelocity,
  gravity, graphicsDataList)`, positional or by keyword, becomes `mainSys.CreateRigidBody(inertia=..., nodeType=...,
  referencePosition=..., initialVelocity=..., referenceRotationMatrix=..., initialAngularVelocity=..., gravity=...,
  graphicsDataList=...)`. Empty `rotationMatrix=[]` and `rotationParameters=[]` are left out.
- `rotationParameters` becomes a rotation matrix of the node type: `EulerParameters2RotationMatrix`,
  `RotXYZ2RotationMatrix`, `RotationVector2RotationMatrix`. A node type that is not written as a literal leaves the
  call reported.
- The result, `[nodeNumber, bodyNumber]` before and the body now:
  - `[n, b] = AddRigidBody(...)` (or `n, b = ...`) → `b = mbs.CreateRigidBody(...)` and, in the next line with the same
    indentation, `n = mbs.GetObject(b)['nodeNumber']`; with `_` for one of them only the other;
  - `rb = AddRigidBody(...)` used only as `rb[0]` and `rb[1]` → `rb` is the body, `rb[0]` →
    `mbs.GetObject(rb)['nodeNumber']`, `rb[1]` → `rb`;
  - a call whose result is not used is replaced as it is;
  - any other use stays reported.
- A sample script with all forms runs after the fix. On copies of the two lecture folders - already fixed by the
  maintainer, with 74 `.bak` files -, 35 scripts held 75 calls of `AddRigidBody`; none is left after `--fix`. Their
  findings go from 179 to 54:
  - sensor files written beside the script (reported, by decision);
  - `rotationMarker0/1`;
  - `CSRtoScipySparseCSR`;
  - `AddRevoluteJoint`.

<a id="rg10-31-13"></a>
### RG10.31.13 — AddRevoluteJoint, AddPrismaticJoint, CSRtoScipySparseCSR (2026-10-10, #2945)

*(Maintainer 2026-10-10: "yes, do these 3 as well" - AddRevoluteJoint, CSRtoScipySparseCSR and rotationMarker0/1.)*

- `AddRevoluteJoint` / `AddPrismaticJoint(mbs, body0, body1, point, axis, useGlobalFrame, showJoint, axisRadius,
  axisLength)` → `mbs.CreateRevoluteJoint` / `CreatePrismaticJoint(itemNumbers=[body0, body1], position=point, axis=axis,
  useGlobalFrame=..., show=showJoint, ...)`.
  - Of the returned `[joint, marker0, marker1]` the joint stays; the markers are read with
    `mbs.GetObject(joint)['markerNumbers'][0/1]`, as new lines after `j, m0, m1 = ...` or in place of `p[1]`, `p[2]`.
  - The handling of the results is one function, `FixResultList`, shared with `AddRigidBody` (#2944).
  - A fixed sample with both joints runs.
- `CSRtoScipySparseCSR(fem.GetMassMatrix(sparse=True))` (and `GetStiffnessMatrix`) → the matrix itself, a scipy
  matrix since 1.9 - the form of all six uses in the lectures. For any other argument it stays reported:
  `SparseTripletsToScipySparseCSR` refuses a scipy matrix, which the old function passed through.
- **A false finding**: `AuswertungShell.py` of the lecture defines its own `CSRtoScipySparseCSR`, and the checker
  reported its calls as the deprecated library function. A function the script defines itself, called by its name, is
  no longer taken for the one of the library.
- `rotationMarker0/1` is not done: the rotation has to go into the localHT of the marker. Which marker that is, and
  whether it has a localHT, is known only when the script runs - see the question to the maintainer.

<a id="rg10-31-14"></a>
### RG10.31.14 — rotationMarker0/1, string node types, the removed HT; the lectures (2026-10-10, #2946)

*(Maintainer 2026-10-10: "If a rotationMarker is used, or in a GenericJoint, it MUST be a rigid body marker, so it can
always be put into localHT of the marker. Problems would occur, if the marker is used multiple times. So, I suggest to
try the fix and then you could check (and fix) these files manually.")*

- **`rotationMarker<k>=A`** of a joint or rigid spring-damper:
  - goes into the localHT of marker k where the script creates that marker as `MarkerBodyRigid`/`MarkerNodeRigid` -
    inline in `markerNumbers`, or in the assignment that reaches the item (the last before it in the same function);
  - conditions: between that assignment and the next one of the name, the item is its only use (a variable used again
    for the next marker is fine), and every name of `A` is set before the marker and not again until the item;
  - `localPosition=p` → `localHT=exu.HT(rotation=A, translation=p)`, an existing localHT → `exu.HT(old)*exu.HT(rotation=A)`;
  - a unit matrix `np.eye(3)` is just removed;
  - checked numerically: a sample with a GenericJoint and a TorsionalSpringDamper gives the same coordinates before
    and after, to 1e-10.
- **Found when the lecture scripts were run after `--fix`:**
  - `AddRigidBody` also took `nodeType=str(exu.NodeType.X)` and the string `'NodeType.X'`, and the rewrite passed them
    on to `CreateRigidBody`, which refuses them (`invalid nodeType`, PARSimulation). `--fix` now writes the NodeType
    itself, also in a `CreateRigidBody` an earlier `--fix` wrote - 26 files of the lectures repaired that way;
  - `rigidBodyUtilities.HT`, the shortcut removed by #2781, was unknown to the checker. `HT(A, r)` →
    `HomogeneousTransformation(A, r)`, and `from exudyn.rigidBodyUtilities import HT` → `... import
    HomogeneousTransformation as HT`; `exu.HT`, the class, is left alone.
- **The lectures** (`C:\DATA\teaching\Innsbruck`, outside the repository; each original kept as `.py.bak`), on the
  maintainer's request. `--fix` was run on `Maschinendynamik_WS2019` and `AngewandteRobotikBA2020`. The
  `rotationMarker` uses it could not move were done by hand:
  - `ExudynSimulation/MobileRobot.py`: the wheel axle marker gets `localHT=exu.HT(rotation=initialRotation, ...)`; it is
    also appended to a list that nothing reads;
  - `ExudynSimulation/PARSimulation.py`: the torsional spring-dampers of the robot joints use the joint markers. Since
    #2804 those carry the rotations in their localHT, so the joints' `rotationMarker0/1` are the unit matrix, and
    `rot0`/`rot1` are gone;
  - `Vorbereitung/ExudynModel.py`, `V2`, `V3`: the wheel motor (TorsionalSpringDamper) and the brake joint use the
    markers of the wheel's revolute joint, instead of the axle and wheel markers turned by the joint's
    `rotationMarker0/1`.

    A torsional spring-damper uses only the orientation of its markers, so this is the frame the scripts meant.
    Since `AddRevoluteJoint` keeps the rotation in the marker (#2804), the old form had turned the motor to the z-axis
    of the axle marker - a change of behaviour that came before this fix.
- **After the fixes:**
  - `AngewandteRobotikBA2020`: no finding left;
  - `Maschinendynamik_WS2019`: only sensor files written beside the script, reported by decision.
- **Run in copies** (`--run`, timeout 60 s):
  - every script that fails also fails as its original `.bak`, there earlier: none of the failures comes from a fix;
  - the failures that are left come from the scripts or the run, not the fixes:
    - APIs of Exudyn 1.0 (`SC.TimeIntegrationSolve`, `preStepPyExecute`, `dryFriction`);
    - data files that are not there;
    - `fig.gca(projection='3d')` of matplotlib < 3.6;
    - `useIndex2Constraints` without Newmark;
    - the solver timeout of 2 s of `--run` cutting real-time simulations short;
  - `PARSimulation`, `Example_HT` and `PAR_UE1_Loesung2025` run now.

<a id="rg10-29-3"></a>
### RG10.29.3, .4, .6-.9 — the release tooling after 1.13 (2026-10-10, #2922, #2923, #2925-#2928)

*(Maintainer 2026-10-10: "Also do RG10.29.")* RG10.29.5 (#2924, what the release notes of a minor release contain)
stays open: a decision of the maintainer.

- **#2922, a rehearsal as a release build**: `setup.py` builds the fast module for every Python version when
  `EXUDYN_RELEASE_BUILD=1`, also for a `.dev` version. `wheels.yml` has the input *build as a release* for *Run
  workflow*, passed to setup.py (into the manylinux container through `CIBW_ENVIRONMENT_PASS_LINUX`), and a weekly
  schedule (Monday 03:17 UTC). A schedule of GitHub always runs on the default branch, so the weekly run builds
  `master`, the last release - which is what catches a change on GitHub's side. Not run here: it needs a push.
- **#2923, the home directory in the logs**: `testRunnerTools.WithoutHomeDirectory` writes the home directory as
  `%USERPROFILE%` (Windows, either slash, any case) or `~`, a longer account name with the same beginning untouched; the
  three runners rewrite their log after closing it. Tests in `test_runnerLogs.py`.
- **#2925, the links of the landing pages**: `tools/checkLandingPages.py`, in `exudev generate --all-checks`. README.rst
  and the text of `[project.readme]` are parsed with docutils (a warning is a finding); a relative link must name a
  file of the repository and is not allowed at all in the PyPI text; a link to `github.com/jgerstmayr/EXUDYN/blob/...`
  or to `exudyn.readthedocs.io/en/<version>/<page>.html` must name an existing file or page source. No network. Both
  texts pass; the tests feed it the link of #2917 (`docs/theDoc/theDoc.pdf`).
- **#2926, the fast-module suite weekly**: the weekly routine is `exudev build --complete --fast`; in a development
  version it runs the fast-module suite and performance run on Python 3.13, the one version that has the module
  (`FastModuleVersions`), instead of failing on 3.10. WORKFLOW.md says that a reference value that moves is
  re-recorded in `AVX2ReferenceSolutionUpdate()` in the same commit, or the commit says that the fast value stayed.
- **#2927**: dependency group `release` with `twine>=6.1` and `pkginfo>=1.12`; RELEASE.md names it.
- **#2928, Python 3.15**: the condition of the issue holds - cibuildwheel 4.3.0 (2026-10-05) builds cp315 without the
  prerelease switch (its prerelease gate is cp316 now), and numpy 2.5.3, scipy 1.18.1 and matplotlib 3.11.2 have cp315
  wheels (PyPI, queried 2026-10-10). `wheels.yml` builds cp310-cp315 with cibuildwheel 4.3.0 (24 jobs), the GitLab
  matrix has cp315, `exudev`'s version matrix is P310-P315, the classifiers, README.rst, the PyPI text and the
  developer documentation say 3.10-3.15. cibuildwheel 4 repairs Windows wheels with delvewheel by default;
  `repair-wheel-command = ""` keeps the Windows wheels as cibuildwheel 3 built them. The local manylinux image has no
  cp315 (`/opt/python`: cp38-cp314); RELEASE.md names the `docker pull` among the traps. The first CI run with 4.3.0
  is the test of the cibuildwheel upgrade.

<a id="rg4-1-3-1"></a>
### RG4.1.3.1, RG4.1.3.2 — the math library of each platform, and portable functions (2026-10-10, #2947, #2948)

*(Maintainer 2026-10-10: "Start with OPEN_RG4.1.3 ... using the exudyn.experimental add a function that evaluates, and
I could run on the different platforms ... before changing a lot in the code, make some experiments. substeps would be
1) evidence of differences 2) performance tests of overloads (and checking the results) 3) use the new functions in
exudyn, possibly with a macro constant as with the Real and other definitions in Exudyn.")*

**The tool.** `exudyn.experimental.EvaluateMathFunction(name, x, y, implementation, repetitions)` evaluates a function
in the compiled module (`src/Utilities/MathFunctions.h`), one call per value in a loop that is not vectorized, as the
code calls it; `python/testing/mathFunctionsPlatform.py` evaluates all of them on fixed arguments (5000 per range, the
ranges the code uses), writes the values to `python/logs/tmp/mathFunctions_<platform>_<module>_<implementation>.npz`,
counts the values that are not correctly rounded (mpmath, 120 bits, if installed), compares two files (`--compare`)
and times std against portable (`--timing`). **The arguments must be the same bits everywhere**: the first version
made them with `10**rng.uniform(...)`, and numpy's power differs between the platforms too - sqrt then "differed" in
142 values. They are now integers of 53 bits times 2^-53 and single numpy operations, and the files carry them, so a
comparison of different arguments is refused.

**1. Evidence** (Windows 11 / MSVC 14.43 / UCRT, and Ubuntu 24.04 in WSL / gcc / glibc 2.39; cp313, regular module):

| function | values | not correctly rounded: Windows | Linux | Windows ≠ Linux | max ULP |
|---|---|---|---|---|---|
| sin | 25000 | 640 | 23 | 631 | 1 |
| cos | 25000 | 603 | 34 | 603 | 1 |
| tan | 25000 | 899 | 49 | 910 | 1 |
| asin | 20000 | 2858 | 9 | 2855 | 1 |
| acos | 20000 | 434 | 14 | 442 | 1 |
| atan | 20000 | 570 | 13 | 575 | 1 |
| atan2 | 20000 | 18 | 22 | 28 | 1 |
| exp | 20000 | 99 | 11 | 100 | 1 |
| log | 15000 | 11 | 7 | 16 | 1 |
| pow | 20000 | 5 | 19 | 18 | 1 |
| sinh | 20000 | 1403 | 4813 | 5101 | 2 |
| cosh | 20000 | 1651 | 2374 | 2611 | 1 |
| tanh | 20000 | 561 | 4367 | 4298 | 2 |
| cbrt | 15000 | 4403 | 7775 | 8309 | 3 |
| fmod, sqrt | 25000 | 0 | 0 | 0 | 0 |

- Every transcendental function differs between the two platforms, in 0.1 % (atan2) to 55 % (cbrt) of the arguments, by
  one ULP and up to three; sqrt and fmod, which IEEE 754 makes exact or correctly rounded, do not.
- **The UCRT of Windows depends on the CPU**: it has an FMA3 code path, chosen at run time. With it switched off
  (`_set_FMA3_enable(0)`, implementation `stdNoFMA3`) sin, cos, tan, asin, acos, atan, exp, sinh, cosh and tanh differ in
  up to 3 % of the values. Two Windows computers, one without FMA3 (before Haswell), can give different results.
- Vectorized loops (`stdLoop`) and the fast module (AVX2) give the same values as the scalar regular module, on both
  platforms.

**2. Portable functions.** `src/Utilities/PortableMath.cpp`, `EXUmath::Portable::Sin`, ...: the double precision
functions of musl libc 1.2.5 (MIT; fdlibm/FreeBSD msun, and Arm's optimized routines for exp, log, pow), downloaded
from git.musl-libc.org and ported by a script (C99 to C++, names, one namespace per file; algorithms and constants
unchanged). musl's `__FP_FAST_FMA` paths, which gcc takes on ARM64, are switched off - otherwise ARM would compute
differently. Results:
- **The same bits on Windows x64 and Linux x86-64**, in all 16 x 20000 values, and the same in the fast module (AVX2,
  /arch:AVX2 without contraction). `test_mathFunctionsPlatform.py` holds a fingerprint of them; macOS ARM is not
  measured yet - the test and the script run there.
- Accuracy like the libraries: at most one ULP from the correctly rounded value (two for sinh and tanh); exp, log and
  pow are correctly rounded in all but 8-17 of 15000-20000 values.
- Time per evaluation (ns, the fastest of 5 x 20 runs):

  | | sin | cos | tan | asin | atan | atan2 | exp | log | pow | sinh | tanh | cbrt |
  |---|---|---|---|---|---|---|---|---|---|---|---|---|
  | Windows UCRT | 4.8 | 4.8 | 5.6 | 3.3 | 4.3 | 8.7 | 1.8 | 2.6 | 7.8 | 6.4 | 5.0 | 10.4 |
  | Windows portable | 7.5 | 8.0 | 9.1 | 2.6 | 3.6 | 11.8 | 1.7 | 2.6 | 9.0 | 8.8 | 7.2 | 3.8 |
  | Linux glibc | 8.2 | 8.5 | 9.8 | 3.1 | 5.6 | 12.0 | 2.3 | 2.2 | 6.4 | 8.2 | 6.9 | 6.5 |
  | Linux portable | 6.9 | 7.0 | 8.2 | 2.6 | 3.9 | 10.7 | 1.6 | 2.3 | 7.7 | 6.7 | 7.1 | 3.6 |

  On Windows the portable sin/cos/tan take 1.6 times the time of the UCRT, a few ns per call; on Linux they are faster
  than glibc. A model spends far more per step elsewhere; step 3 measures what it costs there.
- Not ported: fmod and sqrt, which are exact or correctly rounded everywhere, and functions Exudyn does not call.

<a id="rg4-1-3-3"></a>
### RG4.1.3.3 — the math calls through EXUmath, and the suite with portable functions (2026-10-10, #2949)

- **The switch**: `EXUmath::Sin`, ..., `Cbrt` in `MathFunctions.h` call `std::sin`, ... or, with `EXUDYN_PORTABLE_MATH`
  (a commented `#define` in `BasicDefinitions.h`, like `DoublePrecision`; for an experiment
  `EXUDYN_EXTRA_COMPILE_ARGS=-DEXUDYN_PORTABLE_MATH`), `EXUmath::Portable`. A double goes to these; any other type -
  `AutoDiff`, a symbolic real - to its own overload through argument dependent lookup (a template enabled for
  non-arithmetic types only, so that `Pow(x, 2)` with an int still takes the double version). `GetPlatformString()`, and
  so `exudyn.config.Version(True)`, says `PORTABLEMATH`. `CODING_STYLE.md` §1 says to use them.
- **The calls**: 247 in 33 files of `src/` routed by a script that skips comments, strings and declarations
  (`Real sin(`, `AutoDiff<...> sin(`, `static SReal sin(`); `src/Graphics`, `src/Tests` and the generated files keep
  the library. **With the switch off the results are bit-identical**: 267 of 267 RESULT values of the suite (test
  models and mini examples) equal those before.
- **With the switch on** (Windows; Linux in WSL, glibc 2.39, numpy 2.2.4 there and 2.4.6 on Windows), RESULT values
  of the suite:
  - Windows, library against portable: 56 of 267 differ, 17 beyond their tolerance - the contact models by 1e-5..2e-4
    (`generalContactFrictionTests` 2.1e-4, `sphereTriangleTest2` 3e-5, `generalContactCylinderTest` 1.3e-5): the math
    library alone moves them as far as Linux does.
  - Windows against Linux: 83 differ with the library, 46 with the portable functions on both.
  - The models of `UnresolvedOnLinux()` and `UnresolvedOnMacOS()` (relative difference Windows/Linux):

    | model | library | portable |
    |---|---|---|
    | generalContactFrictionTests | 1.9e-04 | **0** |
    | sphereTriangleTest2 | 2.4e-05 | **0** |
    | rigidBodySpringDamperIntrinsic | 8.2e-11 | **0** |
    | generalContactImplicit1 | 8.4e-11 | **0** |
    | computeItemTest | 2.3e-11 | **0** |
    | geometricallyExactBeamRightAngleFrame | 1.3e-16 | **0** |
    | generalContactCylinderTest | 1.5e-06 | 8.7e-06 |
    | createSphereQuadContact | 1.1e-07 | 1.1e-07 |
    | createSphereTriangleContact | 3.0e-10 | 3.0e-10 |
    | ANCFbeltDrive | 1.0e-10 | 1.0e-10 |
    | ANCFgeneralContactCircle | 3.2e-12 | 3.2e-12 |
    | objectFFRFreducedOrderTest | 7.0e-13 | 7.0e-13 |
    | coordinateSpringDamperExt, rollingDiscTangentialForces, contactSphereSphereTest, sliderCrank3Dbenchmark, ANCFcontactCircleTest, connectorGravityTest, ANCFslidingAndALEjointTest | 0 | 0 |

  - **The math library is the cause of the large contact differences**: with the same functions on both platforms
    the friction model, the sphere-triangle contact and four more agree to the last bit.
  - `generalContactCylinderTest` differs with the switch as well: another cause.
  - The ones with the same difference with and without the switch do not depend on the C++ math: their values come
    from the Python side (the Create functions, numpy 2.2 against 2.4, cf. #2502). A Linux environment with numpy 2.4
    decides it.
  - The last row agrees in WSL also with the library; GitLab (manylinux_2_28, its own glibc) and macOS see them differ
    - consistent with different versions of libm.
- **Time**: the performance tests, twice each on Windows, without other load: library 49.6 s / 51.8 s, portable
  50.4 s / 50.0 s - no difference beyond the noise of ±5 %.
- **Not done here** (RG4.1.3.4, the maintainer's decision): the switch on by default. It moves 56 results, so every
  reference set is recorded again, and macOS ARM has to be measured first.

<a id="rg6-18-4"></a>
### RG6.18.4 — a solution file without coordinates (2026-10-10, #2950)

*(Maintainer 2026-10-10, with `tmp/ex02b_exudyn_bewegtes_KS.py`: the SolutionViewer reads the file only after a
`CreateMassPoint` is added, otherwise it is one column and not loaded as a matrix.)* A model of ground objects moved by
a PreStepUserFunction has no coordinates; its solution file holds the time only, and `np.loadtxt` returned a vector,
so `np.size(data, 1)` failed. `LoadSolutionFile` reads with `ndmin=2` - also a file with a single row. The binary
file was read correctly already. Test model `solutionFileWithoutCoordinatesTest.py`: text and binary file, 11 rows of
one column, `SetSolutionState` for every row.

<a id="rg6-8-9"></a>
### RG6.8.9 — the raytracer cut the lines in front of the focal plane (2026-10-10, #2951)

*(Maintainer 2026-10-10, same script: the raytracer shows only parts of the sensor traces, which OpenGL shows fully;
also at the last step without the SolutionViewer.)*
- The lines - sensor traces, line graphics, the basis lines of the items - are rasterized after the raytracing and
  tested against its depth buffer and a near plane. The near plane was `-projectionNearMin` (-0.1) for every perspective
  view, which is right for the camera-centric view, whose eye is at the origin; in the default model-centric view the
  eye is at `zoom/perspective` in front of the focal plane, so every line point nearer to the camera than the focal
  plane was dropped. In the example, the screw trace rises above the ground: in a view from above all of it vanished,
  in an inclined view the parts in front of the focal plane.
- The near plane is now `projectionNearMin` in front of the eye of either view, and the depth of a line is interpolated
  from the distance to that eye (it used `zoom/perspective` also in the camera-centric view). Triangles were not
  affected: they are raytraced from the eye.
- Measured: rendered offline at perspective 0, 0.1, 0.3, 0.5: before, no trace at 0.1-0.5 in the top view; after, the
  traces and triads are complete. pytest `testRaytracerTracesInPerspective`: the red pixels of a trace above the
  focal plane with perspective 0.5 are at least half of those of the orthographic image.
- `raytracerNOGLFWtest.py` (perspective 1) changes from 0.28161678591179 to 0.28013531271596: 625 pixels, the edges of
  the three bricks nearest to the camera, which were missing and are now drawn (compared image by image).

<a id="rg4-1-3-5"></a>
### RG4.1.3.5 — the native and the portable functions timed in the experimental function (2026-10-10, #2952)

*(Maintainer 2026-10-10: "I think it is required to measure the performance of these functions for the native and the
modified (musl lib) variant directly, within the experimental function ... if we only need a few functions from musl
lib, we finally should only include these ones directly in the exudyn code (mentioning the source).")*
- `EvaluateMathFunction(name, x, y, implementation, repetitions)` returns the time per evaluation, and times inside
  the compiled module: `std` is the library, `portable` the musl functions, and the new `exudyn` is `EXUmath::Sin`,
  ... - what the C++ code calls, the library or, with `EXUDYN_PORTABLE_MATH`, the portable functions.
  `mathFunctionsPlatform.py --timing` prints the three (the fastest of 5 x 20 evaluations of the check's arguments).
- Windows, ns per evaluation, regular module (the fast module within 0.3 ns of it):

  | | sin | cos | tan | asin | acos | atan | atan2 | exp | log | pow | sinh | cosh | tanh | cbrt |
  |---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
  | std | 4.9 | 4.8 | 5.6 | 3.2 | 3.0 | 4.6 | 8.5 | 1.7 | 2.5 | 7.6 | 5.8 | 4.7 | 5.0 | 10.1 |
  | portable | 7.8 | 8.1 | 9.0 | 2.5 | 2.6 | 3.8 | 11.5 | 1.7 | 2.5 | 8.7 | 7.7 | 3.7 | 6.9 | 3.8 |
  | exudyn, switch off | 4.8 | 4.8 | 5.6 | 3.2 | 3.0 | 5.1 | 8.4 | 1.8 | 2.5 | 7.6 | 5.7 | 4.8 | 5.0 | 10.3 |
  | exudyn, switch on (sin to exp) | 8.1 | 8.2 | | | | | 11.1 | 1.7 | | 8.6 | | | | |

  The wrapper costs nothing (exudyn = std with the switch off, = portable with it on). The Linux numbers are in the
  entry of RG4.1.3.1: there the portable functions are faster than glibc.
- **Only what Exudyn needs is in the code**: `PortableMath.cpp` holds the 14 functions `src/` calls through `EXUmath`
  (all of them are called, from 2 call sites for acos to 77 for sin), the kernels they share (`__sin`, `__cos`,
  `__tan`, `__rem_pio2`, `__rem_pio2_large`, `__expo2`, `expm1` for the hyperbolic functions), the tables of exp, log
  and pow, and musl's error helpers; its header names musl 1.2.5, the files and the three licences.

<a id="rg10-29-5"></a>
### RG10.29.5 — the release notes are three lines (2026-10-10, #2924)

*(Maintainer 2026-10-10: "only short notes, refering to the issues and revision section, but no details and even no
summary (I would agree with three lines of text on the changes). The changes are obvious from the issues.")*
`WriteReleaseNotes` writes `dist/RELEASE_NOTES.md` - the body of the GitHub release and the message of the tag - as a
title and three lines: the revisions chapter for what changes for a model script, the issue tracker for the issues
resolved since the previous tag (`git tag --sort=-v:refname`, e.g. "since 1.13.6"), and the install command. It no
longer cuts the section of `CHANGELOG.md`, which after a minor bump holds only the issues since the bump.

<a id="rg11-8-5"></a>
### RG11.8.5 — sparse system matrices for Python (2026-10-10, #1529)

*(Maintainer 2026-10-10: "RG11.8.5: I follow your proposal.")*
- `GetSystemJacobian(sparse=False)` and `GetSystemMassMatrix(sparse=False)` of the static, implicit and explicit
  solvers: with `sparse=True` a `scipy.sparse.csr_matrix`, built in C++ from the entries that are not zero
  (`GeneralMatrix::GetSparseEntries`: the built Eigen matrix, or its triplets before it is built - a position may come
  twice, `csr_matrix` adds them up) - no dense matrix on the way, which needed n² memory (20 GB for 50 000
  coordinates). With a dense solver, `sparse=True` converts its matrix. scipy is imported only then; without it the
  error says so. The return type of the definitions is `PyObject` (`py::object`, stub `Any`).
- `ComputeODE2Eigenvalues(useSparseSolver=True)` computes and exports the matrices sparse (the solver switched to
  `EigenSparse` for the computation) whenever the eigenvalues go to `eigsh`: without algebraic equations, with
  `constrainedCoordinates` or with `ignoreAlgebraicEquations`; the nullspace projection and the complex eigenvalues
  stay dense. `ComputeLinearizedSystem(..., useSparseMatrices=True)` returns M, K, D sparse.
- Test model `sparseSystemMatricesTest.py`: a chain of 30 mass points, sparse = dense for both solver types and the
  linearized system (88 entries of K instead of 8100), the first five eigenvalues of `eigsh` equal to `eigh`.

<a id="rg4-1-3-6"></a>
### RG4.1.3.6 — the namespace EM, and the check (2026-10-10, #2953)

*(Maintainer 2026-10-10: "Use a separate small EM "Exudyn Mathfunctions" namespace. In files where several such
functions are used, the functions are used without the namespace ... In sparse usage (lets say up to 5 such functions in
a .cpp file), EM::Sin is good to go ... Also in some basic math, kinematics and lie group header files, keep the
namespace. In item code implementations, readability might be improved when using it without the namespace. Certainly,
the mentioned check needs to be added".)*
- `namespace EM` holds the 14 functions and nothing else, so `using namespace EM;` brings in only these; the
  portable functions, the evaluator and the rest stay in `EXUmath`.
- The 247 calls: six `.cpp` files with more than five calls (`CObjectRigidBody2D` 22, `CObjectBeamGeometricallyExact2D`
  16, `CNodeRigidBodyRxyz` 10, `CNodePoint2DSlope1`, `CObjectANCFCable2D`, `CObjectRigidBody` 6 each) say
  `using namespace EM;` after their includes and call `Sin(...)`; the other `.cpp` files and all headers write
  `EM::Sin(...)`. The suite gives the same 269 results to the last bit.
- `tools/checkMathFunctions.py`, in `exudev generate --all-checks`: a call of `sin`, `std::sin`, `::sin`, ... in `src/`
  (not in `src/Graphics`, `src/Tests` and the two files that implement them) is refused, as is `using namespace EM;`
  in a header. Comments, strings, declarations of overloads (`static SReal sin(`) and class functions (`SR::sin`) are
  not calls. Its first run found nothing the routing of #2949 had missed. pytest `test_checkMathFunctions.py`.
- `CODING_STYLE.md` §1 states the rule.

<a id="rg4-1-3-4"></a>
### RG4.1.3.4 — the portable math functions by default (2026-10-10, #2954, #2955)

*(Maintainer 2026-10-10: "I will have the Mac on Monday again, but we are good to go to switch to EM::... functions, as
we see that at least it resolves linux problems, right?")*
- `BasicDefinitions.h` defines `EXUDYN_PORTABLE_MATH` unless `EXUDYN_LIBRARY_MATH` is set (by the compiler, e.g.
  `EXUDYN_EXTRA_COMPILE_ARGS=-DEXUDYN_LIBRARY_MATH`), which gives the library functions to compare with. Both modules
  build with it; `config.Version(True)` says `PORTABLEMATH`.
- **Recorded again on Windows**: 53 values of the test models and mini examples (the three tests whose result is an error
  against 0 keep 0), the 16 entries of `AVX2ReferenceSolutionUpdate()` from the `_fast` log, and `perfRigidPendulum`
  (rel. 2.1e-6, a long implicit run; the other performance results stay within their tolerance). Both suites and the
  performance tests pass. The largest moves:

  | model | rel. | reference before | portable |
  |---|---|---|---|
  | generalContactFrictionTests.py | 2.1e-04 | 12.022654145378834 | 12.025130873085953 |
  | sphereTriangleTest2.py | 3.0e-05 | 4.35616383223589 | 4.356293935400209 |
  | generalContactCylinderTest.py | 1.3e-05 | 12.24658398056691 | 12.246428561485022 |
  | driveTrainTest.py | 1.2e-08 | -9.26985560534277e-08 | -9.269855494320467e-08 |
  | kinematicTreeAndMBStest.py | 3.3e-09 | 263.88120463802585 | 263.88120375754687 |
  | geometricallyExactBeamRightAngleFrame.py | 7.0e-10 | 3.306181713660017 | 3.306181715978533 |
  | generalContactImplicit1.py | 5.2e-10 | 0.7758155402165082 | 0.7758155406175806 |
  | rigidBodySpringDamperIntrinsic.py | 7.6e-11 | 0.5472368462870515 | 0.5472368463287056 |
  | computeItemTest.py | 5.5e-11 | 71.26280786574209 | 71.26280786962762 |
  | fourBarMechanismIftomm.py | 2.8e-11 | 0.1721665271838613 | 0.17216652718868367 |
  | connectorRigidBodySpringDamperTest.py | 2.0e-11 | 0.18276224743555652 | 0.18276224743194636 |
  | ballBearingTest.py | 1.4e-11 | 0.037852414033278825 | 0.03785241403274696 |
  | serialRobotTest.py | 1.6e-12 | 0.7681856909844541 | 0.7681856909831933 |
  | geometricallyExactBeam2DquadraticTest.py | 8.9e-13 | 0.7426300926712416 | 0.7426300926718995 |

  The other 39 move by less than 1e-12; all are in the git history of `runTestSuiteRefSol.py`.
- **Linux, measured**: manylinux_2_28 in docker (cp313, numpy 2.5.3, scipy 1.15.2 - what GitLab runs) and WSL (glibc
  2.39, numpy 2.2.4): 3 models differ from Windows, both times the same three - `generalContactCylinderTest`
  (abs. 1.1e-4, rel. 8.7e-6) and the Create* contacts `createSphereQuadContact` and `createSphereTriangleContact`.
  `UnresolvedOnLinux()` keeps these three and loses the other nine; WORKFLOW.md §8 says so.
- **macOS / Linux ARM**: `UnresolvedOnMacOS()` is unchanged until the maintainer measures on macOS ARM (Monday).
- **#2955, found on the way**: the local manylinux build repaired and tested the alphabetically first wheel of
  `dist/manylinux` - `exudyn-1.13.3` - instead of the one it had built, and reported twelve failures of that old
  module. `buildManylinux.sh` selects both wheels by the version of `version.txt`. GitLab starts from a clean
  checkout and was not affected.
- **The build of #2953 (d6c8a5f7) did not compile on Linux**: gcc looks up a name in a template where the template
  stands, and two files had templates before their `using namespace EM;`. The directive is now after the first
  `#include` of the six files (CODING_STYLE.md §1); MSVC (`/Zc:twoPhase-`) had accepted it.
