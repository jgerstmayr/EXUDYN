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

<a id="rg11-10-1"></a>
### RG11.10.1 — the kinematics functions of rigidBodyUtilities compared (2026-10-10, #2957)
*(Maintainer 2026-10-10: the kinematics functions of rigidBodyUtilities - rotation matrices, Gram-Schmidt, basis
vectors, skew matrices, Euler parameters - come from C++ through pybind, defined in the definition files; not the
homogeneous transformations, T66 and RigidBodyInertia. "Start with RG11.10.1 and put the comparison on
tmp/compareRBU.md")*
- `tmp/compareRBU.md` (not tracked): 29 functions in five tables - the files using each, what it does now, the C++
  function, the interface wanted - and nine decisions D1-D9 with a recommendation each. `RigidBodyMath.h` and
  `BasicLinalg.h` have 22 of them; missing in C++: `AngularVelocity2EulerParameters_t`, `AngularVelocity2RotXYZ_t`,
  `RotXYZ2EulerParameters`, `RotationMatrix2RotZYZ`, `RotationMatrix2D`, `RotationVector2G`/`GLocal`, the 4x4 case of
  `Skew2Vec`, the matrix case of `ComputeSkewMatrix`.
- Found: `RotationVector2RotationMatrix` computes `1-cos(phi)` where the node `NodeRigidBodyRotVecLG` computes
  `2 sin^2(phi/2)`; `RotationMatrix2EulerParameters` normalizes, `RotationMatrix2EP` does not;
  `RotationVector2GLocal` names its argument `eulerParameters`; `AngularVelocity2RotXYZ_t` and `ComputeSkewMatrix`
  print instead of raising. The decisions are taken in RG11.10.4.

<a id="rg11-10-2"></a>
### RG11.10.2 and RG11.10.3 — exudyn.cMath (2026-10-10, #2958, #2959)
- **The pattern of a C++ submodule** (for `cUtilities` the same in RG11.10.4): a definition file of its own,
  `definitions/pybindCMath.py`, with `pb` (the page) and `cMathModule` (the functions). `pybindEmitter.py` replays it:
  the C++ bindings into `pybind_manual_classes.h`, inside a block in which `m` is the compiled submodule
  `exudyn._cMath`; the page `docs/generated/cInterface/CMath.md`; the stub `stubCMath.pyi`, which
  `createStubFiles.py` writes to `python/exudyn/cMath.pyi` (tier 1 in `regenerate.py`). `python/exudyn/cMath.py`
  takes the compiled functions and adds `pi`; `import exudyn` imports it. The compiled submodule has a leading
  underscore so that the star import of `__init__.py` does not put it where the Python module belongs.
- **cMath**: `sin`, ..., `cbrt` (the 14 functions of `EM`) and `sqrt`; a float gives a float, a list or an array an
  array of the same shape (`py::vectorize`); `atan2(y, x)` as in `math`.
- **Measured** (Windows, `mathFunctionsPlatform.py --timing`, ns per value): on arrays as fast as numpy (sin 5.2 vs
  4.9, exp 1.8 vs 1.8, cbrt 4.0 vs 7.7, tanh 6.5 vs 3.7). One call per float: 200 ns, `math.sin` 34 ns - the call of a
  pybind function costs 155 ns. Through `py::vectorize` a float took 530 ns; a float overload registered before the
  vectorized one takes it at once.
- `python/testing/test_cMath.py`: the bits of all 15 functions equal `EvaluateMathFunction(..., 'exudyn')` on the
  arguments of the platform check, as array and one float at a time; shapes, lists, broadcasting; `__all__`, `pi` and
  the stub agree.

<a id="rg11-10-4"></a>
### RG11.10.4 to RG11.10.8 — cUtilities, and cMath in the Python library and the test models (2026-10-11, #2956, #2960-#2964)
*(Maintainer 2026-10-11, on tmp/compareRBU.md: D1 cUtilities returns a (3,3) array; D2 keep the normalization; D3 the
function of NodeRigidBodyRotVecLG; D4 raise; D5 RotationVector2G/GLocal stay Python; D6 Skew also for 3n vectors and
3n x m matrices, Skew2Vec 3x3 and 4x4; D7 an issue; D8 flip=False; D9 ExpSO3 and LogSO3 in cUtilities. On speed: cMath
is for initial values and test models, users may take math and numpy as before.)*
- **cUtilities** (#2960): 28 functions in `src/Pymodules/PyCUtilities.h`, declared in `definitions/pybindCUtilities.py`
  like cMath; `pybindEmitter.py` and `createStubFiles.py` take the list `cSubmodules` of `definitions/pybindTypes.py`.
  New in `RigidBodyMath.h`: `AngularVelocity2RotXYZ_t`, `RotXYZ2EP`, `RotationMatrix2RotZYZ`. Against the Python
  functions, 300 random inputs: equal to rel. 1.2e-15 (Skew, EP2G, EP2Glocal, EP2RotationMatrix bit-identical).
  `ComputeOrthonormalBasisVectors` and `GramSchmidt` return the vectors as ROWS of a (3,3) array, so that
  `[v0, n1, n2] = ...` and `np.array(...)` keep working; no caller had to change. `AngularVelocity2RotXYZ_t` raises
  for |cos(rot[1])| < 1e-14 - cos(pi/2) is 6e-17 in floats, so the old test `== 0` never fired. Test model
  `cUtilitiesTest.py`: orthogonality, inverses, G matrices against central differences, the cross product, what is
  refused; its result sums the values of all functions.
- **rigidBodyUtilities** (#2961): the kinematics functions call cUtilities, docstrings say so; `**(1/3)` of the
  inertia is `cbrt`. The suite passed without a new reference.
- **The library** (#2962): `tools/checkPythonMath.py` found 405 places; rewritten by a script (`from math import` ->
  `from exudyn.cMath import`, `np.sin(` -> `cMath.sin(`, `x**0.5` -> `cMath.sqrt(x)`), the constants of
  `basicUtilities` by hand (`math.sqrt`, correctly rounded). `np.linalg.norm(x)` with one argument sums in an order of
  the BLAS library (the 1 ULP of #2956): `cMath.norm`, in Python, `math.fsum` of the squares. `lieGroupBasics.ExpSO3`
  and `LogSO3` call cUtilities.
- **Test models** (#2963): 88 changed. Recorded again on Windows: 24 references (largest: `generalContactCylinderTest`
  rel. 3.9e-5, `ANCFbeltDrive` rel. 8.9e-6, the others below 5e-12) and 7 of the fast module. WSL (glibc 2.39, numpy
  2.2.4, the build of this state): all 177 test models agree; the three of `UnresolvedOnLinux()` to the last bit, so
  the list is empty (#2956). The manylinux docker was not run. `python/Examples` stay as they are.
- **The check** (#2964), in `--all-checks`: imports and calls of math and numpy functions, single-argument
  `np.linalg.norm`, `**` with an exponent that is not whole; `#checkPythonMath: allowed` on a line exempts it (a
  `symbolic.Real` in `symbolicUserFunctionTest.py`). 310 whole exponents (`x**2`) are reported only.

<a id="rg11-8-1"></a>
### RG11.8.1 and RG11.8.4 — override in src/Linalg, and Lie group nodes in the static solver (2026-10-11, #123, #1493)
*(Maintainer 2026-10-11: RG11.8.1 now, the libraries identified first; RG11.8.4 can be implemented at once, reusing the
checks of the dynamic solvers where it makes sense; the other sub-steps of RG11.8 evaluated first.)*
- **How the places were found**: every source of `sources.json` compiled syntax-only by gcc 13.3 in WSL with
  `-Wsuggest-override` (the pybind11 headers of venvExuP313, which venvP313 in WSL does not have). A build with
  `EXUDYN_EXTRA_COMPILE_ARGS=-Wsuggest-override` reports nothing, because setup.py reuses the object files when only the
  extra arguments change. `override` was added after the parameter list and `const`, in the style the files already
  use (`virtual ... override`); the second scan finds none in src/Linalg.
- **The rest of src** (#2967): 455 places in 112 files, most of them 4 in every generated item header.
- **#1493**: `CSolverBase::SolveSteps` refuses, for the static solver, a node of type `LieGroupWithDirectUpdate`
  (NodeRigidBodyRotVecLG), with `NotImplementedFeatureError`; the dynamic solvers sort such nodes into lists, which the
  static solver does not need, so only the loop over the node types is the same. `ComputeLinearizedSystem` uses the
  static solver without `SolveSteps` and is not refused. geometricallyExactBeamTest.py said in a comment that the
  static case does not work with NodeRigidBodyRotVecLG. The node's description says so now.

<a id="rg11-8-2"></a>
### RG11.8.2, RG11.8.3 and the page Module functions (2026-10-11, #990, #1061, #2968)
*(Maintainer 2026-10-11: RG11.8.2 as suggested, CreateCoordinateConstraint with includeReferenceCoordinates=[bool,
bool], both False; RG11.8.3 one short table, `returns a reference` in typewriter font; cMath and cUtilities into the
module page, ordered config, variables and sys, special, cMath, cUtilities, experimental, with "Module functions" at
the level of SystemContainer.)*
- **#990**: `includeReferenceCoordinate` of MarkerNodeCoordinate and MarkerNodeODE1Coordinate adds the reference value of
  the coordinate in both code paths of the marker - `ComputeMarkerData` and the connector interface (#2745); a ground
  node gives its reference coordinate (0 if it has none for that index). MarkerNodeCoordinates includes the reference
  values already, so the evaluation's plural flag was not needed. Test model `markerNodeCoordinateReferenceTest.py`:
  a mass point held at the absolute position 2.5, and a spring of length 2 between a ground node at 1 and a mass point
  at 3, both exact to 1e-12.
- **#1061**: the eight rows were measured in Python (`is` and changing the returned object) before writing them. The
  tag comes from the binding: `pybindEmitter.MarkReferences` adds it to a function with
  `py::return_value_policy::reference(_internal)` and to the member a `CppCode` binds with it; the scalar members of
  GeneralContact (`isActive`, `verboseMode`) are copied by Python and get none. Members that pybind returns by reference
  without an explicit policy - `SC.visualizationSettings` - are in the table, not tagged.
- **#2968**: `pybindModule.py` has the sections in the new order, in two NoStub blocks (config, then special and
  experimental, with variables and sys between them); the new steering call `CSubmodulesHere` replays the definition
  files of cSubmodules there, with a Replay of their own. The page title is "Module functions"; `CMath.md` and
  `CUtilities.md` are no longer written and were removed; README.rst links the new place.
- **Tests of cMath and cUtilities** (maintainer's question): `python/testing/test_cMath.py` (bits against
  `EvaluateMathFunction`, shapes, norm, both stubs), `python/TestModels/cUtilitiesTest.py`, and
  `test_checkPythonMath.py` for the check - no new step needed.

<a id="rg11-8-6"></a>
### RG11.8.6, RG11.8.7, MarkerNodeCoordinates and the page Exudyn module functions (2026-10-11, #1614, #1912, #2969, #2970)
*(Maintainer 2026-10-11: FactorizeNew - the temporaries as members of GeneralMatrixEXUdense, with the option for
clean-up, because for n < 20, which is frequent, an allocation per factorization counts; coSimulationTest.py as
proposed; MarkerNodeCoordinates gets the parameter too, default True; the chapter is 'Python-C++ interface', the page
'Exudyn module functions', with short examples of special, cMath and cUtilities.)*
- **#1614, the evaluation**: `GeneralMatrixEXUdense::FactorizeNew` kept `static ResizableMatrix m` and
  `static ArrayIndex rows` for `InvertSpecial` with `LinearSolverType::EXUdense`, the default: the n x n temporary of
  the largest system stayed allocated until Python ended, and two solvers in two threads shared it. FactorizeNew is
  called only on the three dense matrices of `SolverLocalData` - `systemJacobian` in the Newton step
  (CSolverBase.cpp:1509) and the implicit solver (CSolverImplicitSecondOrder.cpp:202), `systemMassMatrix` in the
  explicit solver (CSolverExplicit.cpp:113, 666) - and in the C++ unit tests. `SimulationSettings.cleanUpMemory`
  replaces these matrices (`SolverLocalData::CleanUpMemory`), which frees whatever they hold.
- **#1614, the change**: `inverseTemporary` and `rowSwapsTemporary` are members of `GeneralMatrixEXUdense`: allocated at
  the first factorization of a size and reused, freed with the matrix. One temporary per dense matrix instead of one
  shared by all; CObjectKinematicTree has its own already.
- **#1912**: `coSimulationTest.py` - m0 = 1 on k0 = 100 (generalized-alpha), m1 = 0.5 on k1 = 50 to m0 (RK44,
  `MainSolverExplicit.SolveSteps` in the PreStepUserFunction of mbs0, with the offset of the spring set to the position
  of m0 and the spring force given back as a LoadCoordinate). Against one system (a tenth of the step size): error
  2.50e-4 with h = 1e-3, 1.25e-4 with h/2, ratio 1.996 - first order, as a staggered coupling is; the test fails if the
  ratio leaves [1.8, 2.2].
- **#2969**: `MarkerNodeCoordinates.includeReferenceCoordinates`, default True, so every model stays the same; the test
  model ties the displacements of two planar mass points at x = 4 and x = 7 with it.
- **#2970**: the title comes from `markdownPageTitles` (pybindEmitter.py) and from DefPyStartClass of the module
  (autoGenerateHelper.py), which DropRepeatedTitle then drops; the chapter heading is written by pybindEmitter itself
  (the AddDocu in pybindGeneralInformation.py is reset by ResetMarkdown), so both are 'Python-C++ interface' now. The
  notebook part `module` (generalInformation.ipynb) shows `config.Version`, `special.solver.timeout`,
  `special.userInterface.suppressPlots`, `cMath.sin`/`cos`/`norm` and `cUtilities.RotationMatrixZ`/
  `RotationMatrix2EulerParameters`, run by tools/runNotebooks.py.

<a id="rg7-1"></a>
### RG7.1 — the plan of the Python user items (2026-10-12, #2971)
*(Maintainer 2026-10-11: before the C++ user items; user node, body, marker, constraint, joint, connector, no user
load; a documentation section per kind with an example. 2026-10-12: the design document under docs/dev/design with
its measurement script; four user types; the connector gets its markers as Python objects; the ANCF cable minimal;
symbolic in a group of its own (RG12) that blocks RG7.8; the overhead of the user functions checked, so that the user
items can still be dropped; go with the user rigid body.)*
- The design, with the decisions and the measurements, is
  [docs/dev/design/pythonUserItems.md](../dev/design/pythonUserItems.md); `measureUserFunctions.py` next to it. Two
  rounds: the first concept was revised by the maintainer's decisions (ObjectGenericODE2 is a superelement, so the user
  body is a new item treated as a finite element; nodes second; the user connector calls marker functions; user types
  for output variables, nodes and markers with names; SensorUserFunction with itemNumbers).
- Measured: a call into Python costs 0.16 us more than C++ for floats, 0.72 us with small numpy arrays, a symbolic
  user function 0.04 us. The code of a user function is mostly generated; by hand about 8 lines of evaluation wrapper
  per function plus the call site and the declaration.
- Steps RG7.2-RG7.9 (#2972-#2979), RG9.7 for the general markers on flexible bodies (#2980), and the new group RG12
  Symbolic with RG12.1-RG12.5 (#2981-#2985).

<a id="rg11-9"></a>
### RG11.8.1.3, RG11.8.13 and RG11.9 — override in all of src, exudyn.shells, the velocity offsets (2026-10-12, #829, #1189, #1631, #2349, #2967)
*(Maintainer 2026-10-12: RG11.8.1 without interaction, itemHeaderEmitter writes override; velocityOffset in the
Cartesian, rigid-body and coordinate spring-dampers, not in ObjectContactFrictionCircleCable2D; #2349 probably done.)*
- **#2967**: itemHeaderEmitter writes `override` for the accessors of the C and Visualization classes in every Main
  class and for the getters of color, triangle mesh and showNodes that VisualizationObjectSuperElement declares pure
  virtual (`overriddenGetters`); structureHeaderEmitter for `Print` of a structure with a parent and for the access
  functions that MainSolverBase declares virtual (`overriddenPyAccess`). By hand: 45 declarations in CNode.h,
  CObjectConnector.h, CObjectBody.h, VisualizationObject.h, VisualizationSystemContainer.h, CSolverExplicit.h,
  CSolverImplicitSecondOrder.h, CObjectANCFCable2DBase.h and Stdoutput.h. The syntax-only scan with gcc 13.3 reports
  none in src now, and setup.py adds `-Wsuggest-override` to the unix flags, so a new one is seen.
- **#2349**: partly done before - the docstrings of ShellMesh's constructor and of four functions had Args and
  Returns. The file header described how the file came about (copied from a test script, the differences to it); it
  now says what the module is. Comments that compared with that script are gone; CreateANCFThinPlateElements,
  AddSphericalJointToGround, AddClampToGround and the nine SurfaceMap functions have Args and Returns.
- **#829, #1631**: `velocityOffset` of CoordinateSpringDamper (scalar), CartesianSpringDamper (3 components) and
  RigidBodySpringDamper (6, in joint 0 coordinates), subtracted from the relative velocity in the damper force. A user
  function gets it as each item treats its offset: CartesianSpringDamper subtracts both from the arguments,
  CoordinateSpringDamper and RigidBodySpringDamper neither, the user function reads them with GetObjectParameter.
  Test model springDamperVelocityOffsetTest.py: dampers without springs drive the masses to the velocity offsets,
  against v(t) = vOff (1 - exp(-d t/m)) to 1e-4 (generalized-alpha, 1000 steps). #1189 closed: not for
  ObjectContactFrictionCircleCable2D.

<a id="rg7-3-1"></a>
### RG7.3.1 — ObjectUserBody, the user rigid body (2026-10-12, #2986)
*(Maintainer 2026-10-12: go with the first steps to make a user rigid body; the user items may still be dropped, so the
code they add is measured.)*
- **The item**: ObjectUserBody (definitions/itemDefsObjects.py, src/ImplObjects/CObjectUserBody.cpp) on any number of
  ODE2 nodes. User functions: `massMatrixUserFunction` (required, checked at Assemble), `forceUserFunction`,
  `positionUserFunction`, `positionJacobianUserFunction`, `rotationMatrixUserFunction`, `rotationJacobianUserFunction`
  (the three matrix functions share one def), and `graphicsDataUserFunction`. Every one gets the TOTAL coordinates
  (reference plus displacement), as a finite element computes a point; the coordinates are lists, as for
  ObjectGenericODE2. C++ forms the velocity and the angular velocity from the Jacobians, the displacement from the
  reference configuration, `GetJacobianTransposedTimesVectorDerivative` by central differences of the Jacobian user
  functions, and the Jacobian of the equations numerically. The access function types derive from the declared access
  functions: MarkerBodyPosition and MarkerBodyRigid; a function a marker needs and the body was not given raises a
  ValueError with its name when it is evaluated - checked in the test model.
- **The renderer thread** may not call Python (VisualizationSystem::UpdateGraphicsData). The access functions in the
  visualization configuration ask `PyGILState_Check()`: without the GIL they read src/System/VisualizationAccessCache.h,
  which registers a local position it does not know; `VisualizationObjectUserBody::CallUserFunction`, called in the
  computation thread when the renderer requests the user function drawing (its HasUserFunction is always true), fills
  the cache and draws the graphics of the user function. A marker or connector is drawn a frame late; velocities and
  displacements in the renderer thread are zero.
- **Test model** userRigidBodyTest.py: a body on NodeRigidBodyRxyz with M = diag(m I, G^T J G) and the quadratic
  velocity terms G^T (J G_t theta_t + omega x J omega), written with exudyn.cUtilities, beside ObjectRigidBody, both
  under gravity, a CartesianSpringDamper at a point off the center of mass and a LoadTorqueVector on MarkerBodyRigid;
  position, displacement, velocity, rotation matrix, angular velocities and kinetic energy agree to 4.5e-14 after
  500 generalized-alpha steps. Mini example: a mass point as a user body.
- **Code added** (design 7.1): by hand 500 lines of C++ (CObjectUserBody.cpp 304, VisualizationAccessCache.h 97,
  evaluation wrappers 53, CallUserFunction 20, the consistency check 27) and 276 lines of definition; generated 493
  lines of headers plus the factory, the Python interface and the page. Nothing in the existing items changed.

<a id="rg7-3-2"></a>
### RG7.3.2 and RG7.3.3 — the ANCF cable and the tetrahedron as user elements, RG7.3 closed (2026-10-12, #2973, #2987, #2988)
*(Maintainer 2026-10-12: the ANCF cable 2D minimal; the cable and the linear tetrahedron now, then the user types.)*
- Both are test models only: ObjectUserBody needed no change, so the C++ code of RG7.3 is that of RG7.3.1.
- **#2987** userANCFCable2DTest.py: the element in 60 lines of Python - cubic Hermite shape functions as 2 x 8
  matrices, the consistent mass matrix (Gauss order 7), the axial strain |r'| - 1 (order 9) and the curvature
  (r' x r'')/|r'|^2 (order 5), as ObjectANCFCable2D integrates them, position and position Jacobian. A cantilever of four
  elements, clamped by coordinate constraints, under a tip force on MarkerBodyPosition: positions and velocities agree
  with ObjectANCFCable2D to 7e-14 after 200 steps (1.5 s, most of it the Python forces under the numerical Jacobian).
- **#2988** userTetrahedronTest.py: Saint Venant-Kirchhoff (Green strain, P = F S), consistent mass rho V/20 (1 + delta),
  position at the coordinates of the unit tetrahedron. A unit cube of six tetrahedra on its diagonal in uniaxial
  tension, held by the fewest constraints and loaded by the consistent forces of its face: the static solution is the
  exact homogeneous one, a E (a^2 - 1)/2 = P with the lateral stretch from S22 = 0, to 2e-16 at the nodes and at a point
  inside each element; a cube in uniform motion has the kinetic energy 1/2 rho V v^2 to 1e-13.
- **#2973** resolved with these: the three examples of design 5.1 (rigid body, cable, tetrahedron) work.

<a id="rg7-2"></a>
### RG7.2 and RG7.2.1 — the common pattern of the user items, the user types (2026-10-12, #2972, #2989)
*(Maintainer 2026-10-12: four user types are enough; the alias works, `exu.OutputVariableType.xyz =
exu.OutputVariableType.Position`; the user types after the cable and the tetrahedron.)*
- **#2972**: the pattern is written down as built, design section 5.9 (docs/dev/design/pythonUserItems.md): names
  `<what>UserFunction`, arguments `mbs, t, itemNumber`, then the subject, then `q, q_t` as lists with the reference
  values; checked sizes; wrappers in evaluateUserFunctions.cpp; PyError for a missing function; serial computation and
  the renderer cache; conditional output variables. Section 5.6 says what the user types are now, 7.1 what they cost.
- **#2989**, the enumerations: `UserType0`-`UserType3` declared in definitions/outputVariableTypes.py (bits 36-39) and
  definitions/enumTypes.py (Node::Type bits 19-22, Marker::Type bits 23-26, EndOfEnumList moved from 22 to 27; it is
  used nowhere). The generated string functions call `UserTypes::Name(kind, index)` of the new header
  src/Main/UserTypes.h (no pybind11, as the generated headers include it), which returns the registered name or
  UserType0.
- **exu.special.userTypes** (Experimental.h, Pybind_manual_classes.cpp, documented in pybindModule.py): `SetName(type,
  name)` checks that the value is a user type, that the name is a Python name and not yet one of the enumeration,
  removes the previous alias, sets the new one on the class and gives the class a `__str__` that writes the name;
  `GetName`, `Reset`. `.name` of the value stays UserType0 (pybind11 cannot rename it).
- **ObjectUserBody.outputVariableUserFunction**(mbs, t, itemNumber, outputVariableType, localPosition, q, q_t): the
  body provides UserType0-3 only when it is given. For that, `ItemOutputVariable` takes a `condition` (a C++ expression),
  which itemHeaderEmitter puts into GetOutputVariableTypes; the annotation `OutputVariableType` was added to the user
  function defs, and the symbolic user functions receive it as the Real of its value (SymbolicUtilities.h).
- **Test model** userTypesTest.py: names, aliases, `str()`, a rename, what SetName refuses, a SensorBody with the
  named user type - its file header says `#OutputVariableType = BendingStrain`, the values are those of the user
  function - the message of a body without the function names the type, and Reset leaves the plain user types for the
  models that run after it.
- **A fix of RG7.3.1 (#2986)**: with the new build, the test suite crashed with a heap corruption (0xc0000374) some
  models after userRigidBodyTest.py, not in a run of the model alone. EvaluateMatrix MOVED the matrix of the user
  function into `value`, which is the ResizableMatrix of the marker temporaries: the move exchanged the memory but not
  the allocated size the ResizableMatrix keeps, so the next resize wrote past the buffer. Found by leaving out the user
  models one by one; `value` is now copied, and the user output variables are copied likewise.

<a id="rg11-8-16"></a>
### RG11.8.12, RG11.8.16 and RG11.8.18 — graphics data errors, the message of a failed solve, Inspect instead of try (2026-10-13, #1977, #2304, #2305, #2991)
*(Maintainer 2026-10-13: both fixes yes; SystemEnergy should ask mbs.Inspect, and look for the same construct
elsewhere; the output of userRigidBodyTest.py "missing rotationMatrixUserFunction refused: 1" asked about.)*
- **#1977** (solver.py): `_SolverFailed` builds the block of `SolverErrorMessage` once. A solve that stops by itself
  raises `SolverError('SolveDynamic terminated: ' + the solver's error string + block)`; an exception from inside the
  solve keeps its type and message and gets the block as a note (`add_note`, Python 3.11; printed on 3.10). Nothing
  is printed any more, as errors go into the exception (CODING_STYLE 10.6). The FAQ example of the old printed
  output is replaced by the current one. Test model solverFailureMessageTest.py: Newton failure with and without
  showHints, a user function error with its note, a singular static system.
- **#2304, #2305** (VisualizationSystemContainer.cpp): every error of `PyWriteBodyGraphicsDataList` goes through
  `GraphicsDataError`, which puts the item - set by a `GraphicsDataContext` in AddObject, ModifyObject,
  SetObjectParameter, the five `CallUserFunction` and per link of KinematicTree - and the entry `graphicsData[i]`
  before the message. Not-a-number and infinity are refused in Line, Lines, Circle (position, radius), Text and the
  radii of Spheres, as they were in TriangleList and the points of Spheres. An error in a graphicsDataUserFunction does
  not stop the visualization, it is written with the same text. Test model graphicsDataErrorsTest.py.
- **#2991**: `SystemEnergy` takes body or not from `mbs.Inspect(object, InspectType.ObjectType)` and the energies
  from `InspectType.OutputVariables`, which lists an energy only where it can be computed; only for an energy not
  listed it reads it once, to report the reason of a NotImplementedError in `.unavailable`. ComputeSystemEnergies uses
  the stored kind instead of trying the body function first. A search of python/exudyn for a try/except around an
  output function found one more: `CreateDistanceSensor` tried the position of its marker, it now asks Inspect.
  energiesTest.py gives the same output.
- **userRigidBodyTest.py**: "refused: 1" was the check passing - a rigid marker on a user body without
  rotationMatrixUserFunction must be refused; the line now says so. The block "DYNAMIC SOLVER FAILED" it printed is
  gone with #1977.

<a id="rg7-7"></a>
### RG7.7, RG11.8.17 and the concept of RG11.8.8 — item numbers for SensorUserFunction, the user function timer, the symmetric basis (2026-10-13, #2007, #2228, #2977)
*(Maintainer 2026-10-13: RG7.7 now; RG11.8.17 in an appropriate way, the timers cost little against a Python call;
RG11.8.8 a concept: the average frame of two slopes for NodePointSlope12 and other nodes, with the derivative,
efficient, a Python version in docs/dev/design first.)*
- **#2977**: `SensorUserFunction.itemNumbers` (an ArrayIndex of any kind of item); `CSensorUserFunction::
  EvaluateUserFunction` passes it instead of sensorNumbers when given, and `MainSensorUserFunction::
  CheckPreAssembleConsistency` refuses both. The argument of the user function is called `numbers` now - positional,
  so existing functions work. Test model sensorUserFunctionItemsTest.py: the kinetic energy of a mass point, a rigid
  body and a 1D mass against the exact value (4e-8: the generalized-alpha method dissipates a little of the rotation),
  and the refusal.
- **#2007**: `UserFunctionTimer` (ExceptionsTemplates.h): `UserFunctionExceptionHandling`, through which every user
  function is called, times the outermost call (a user function may call another) while the solver has set it
  active - `timer.Reset` with `show.computationTime`; thread_local, so worker threads are not counted, and not in the
  fast module. `CSolverTimer.userFunctions` (definitions/structureDefsSolverData.py) is read at the end of the solve
  and listed as `(userFunctions) = x% (within the timers above)`, not counted in the sum, as it overlaps ODE2RHS,
  Python, ... In a spring-damper with a Python force, 2000 steps: 33 % of the measured time. Test model
  userFunctionTimerTest.py checks the flags only.
- **#2228, concept**: docs/dev/design/symmetricBasis.md and symmetricBasis.py. The basis $e_1, e_2 = (\hat u \pm \hat
  v)/\sqrt 2$ of the normalized sum and difference of the unit slopes is orthonormal, symmetric, independent of the
  slope lengths, and equal to the rotation of the polar decomposition (checked to round-off); its rotation Jacobian
  in closed form from dot products, checked against the numerical derivative. Found on the way: NodePointSlope12 takes
  its rotation from Gram-Schmidt and its angular velocity from a least squares fit, which agree for a rigid motion only
  - up to 130 % apart for deforming slopes; the derivative of its rotation Jacobian is not implemented. Proposed: C++
  templated, the derivative by AutoDiff, then the node; the step stays open for the decision.

<a id="rg11-8-8"></a>
### RG11.8.8 — the symmetric basis of two slopes, for the slope nodes, the ANCF beam and the ANCF plate (2026-10-13, #2228)
*(Maintainer 2026-10-13: the symmetric basis, combined; the Jacobian needs a few divisions more, not measured; in C++
as a template for automatic differentiation; used in ObjectANCFThinPlate and NodePointSlope12, and in ObjectANCFBeam
and NodePointSlope23 instead of the Gram-Schmidt functions of AccessFunctionsAD.h; the ANCF beam tested again, against
the geometrically exact beam.)*
- **The functions** (src/Linalg/RigidBodyMath.h): `SymmetricBasis(a, b, e1, e2, e3)` in the combined form of design
  section 5 (4 square roots), `SymmetricBasisRotationJacobian(a, b, G)` in closed form (dot products, 3 x 6), both
  templated on the number type; bound as `exudyn.cUtilities.SymmetricBasis`, `SymmetricBasisRotationJacobian` and
  called by `exudyn.rigidBodyUtilities` under the same names. They agree with the Python reference
  (docs/dev/design/symmetricBasis.py) to 2e-13 on random pairs; cUtilitiesTest.py checks orthonormality, symmetry,
  length independence, the polar rotation (SVD), G against the numerical derivative, and the refusal of parallel or
  zero vectors.
- **AccessFunctionsAD.h**: `SlopesRotation<SlopesFrame::XY or YZ>` places e1, e2, e3 as the columns of the node or
  element ([e1, e2, e3] for the slopes x, y; [e3, e1, e2] for y, z); `SlopesRotationJacobian` and
  `SlopesAngularVelocity` use the closed-form G; `SlopesRotationJacobianTTimesTorqueDerivative` differentiates G^T
  torque by AutoDiff with 6 directions - single level, where the Gram-Schmidt version needed nested AutoDiff of the
  rotation matrix. The three `EXUmath::OrthogonalBasisFromVectorsXY/ZY` of BasicLinalg.h had no other user and are
  removed.
- **NodePointSlope12**: rotation matrix, angular velocity, rotation Jacobian, `GetRotationMatrix_t` (= skew(omega) A)
  and the until now missing `GetRotationJacobianTTimesVector_q` - all of one frame; before, Gram-Schmidt for the
  rotation and a least squares angular velocity. **NodePointSlope23** the same code. **ObjectANCFBeam**
  `GetRotationMatrix` takes the symmetric basis; its rotation Jacobian and derivative went through AccessFunctionsAD
  already. **ObjectANCFThinPlate** gets `GetRotationMatrix`, `GetAngularVelocity` (was not implemented),
  `GetAngularVelocityLocal`, `GetRotationJacobian`, `GetJacobianTransposedTimesVectorDerivative` for a torque, and the
  output variables RotationMatrix, AngularVelocity, AngularVelocityLocal: the slopes at a mid-surface point through the
  inverse Jacobian of the reference geometry, as GetSlopes; MarkerBodyRigid now acts on the plate.
- **References**: ANCFBeamTest.py +2.6e-10, ANCFCableBeamDampingTest.py +2.3e-11, the mini examples NodePointSlope12
  and ObjectANCFThinPlate 1.1e-5 (0.016 %: the clamping GenericJoints act on the symmetric frame).
- **ANCFBeamGeometricallyExactComparisonTest.py** (new): the cantilever of ANCFBeamTest case 2 with 8 elements of each
  kind. Static general bending: tip displacement within 8.8e-5, rotation matrix within 3.7e-6. A large twist and
  bending torque through MarkerBodyRigid on the last element: within 7.7e-3 and 8.1e-3; refining from 8 to 32 elements
  the displacement difference falls quadratically (7.7e-3, 1.9e-3, 4.8e-4), the rotation linearly. The cross section
  penalty factor matters: with 10 the slopes shear by 0.2 degrees at that twist, and the symmetric frame and a
  Gram-Schmidt frame differ from the rigid cross section of the geometrically exact beam by the whole or half of it;
  with 100 both agree with each other and with the geometrically exact beam; the test uses 100. Dynamic: the frame and
  angular velocity of the element end are those of the end node, exactly.
- **ANCFThinPlateRotationTest.py** (new): a torque at a corner of a clamped strip through MarkerNodeRigid and through
  MarkerBodyRigid on the element corner gives the same solution to the bit; the frame of the element at the corner is
  that of the node, its third axis the normal; dynamic, the angular velocity of the element frame is the time
  derivative of its rotation matrix (1.8e-10).

<a id="rg11-8-19"></a>
### RG11.8.19, RG4.20.5.1, RG11.8.14 and RG11.8.15 — the strains of the ANCF beam, the slope scaling of ShellMesh, the mecanum test, a container without renderer (2026-10-14, #948, #2226, #2259, #2992)
*(Maintainer 2026-10-14: do the follow-up on the cross section penalty, #2226, #2259, #948.)*
- **#2992, ObjectANCFBeam**: after #2228 the node and element rotation were the symmetric basis, but the strains
  (twist, curvatures, the two shear strains) were measured in the frame of the 2013 formulation (`GetBasisI`, the
  z-slope leading). A torque on the node then acted on a frame that the strains did not see as rotated, and the cross
  section at the loaded node sheared against the penalty only: 0.00044, 0.00088, 0.0018 degrees for 8, 16, 32
  elements, i.e. 1/(penalty x element length). `CrossSectionFrame` (the symmetric basis, [e3, e1, e2]) replaces
  `GetBasisI`/`GetBasisI_xI`; the curvature is kappa = A^T G [slopeY'; slopeZ'], G the closed-form rotation Jacobian.
  The shear falls to about 1e-10 degrees; ANCFBeamGeometricallyExactComparisonTest: the static rotation difference to
  the geometrically exact beam 1.4e-7 instead of 3.7e-6. References: ANCFBeamTest +2.4e-5 (the Princeton beam of
  ANCFBeamTest moves by about 1e-6 relative), ANCFCableBeamDampingTest +1.5e-11, energiesFlexibleBodiesTest +2.8e-10,
  the comparison test -3.2e-9.
- **#2226, ShellMesh**: the derivative lengths of the mapped geometry (chained through the bilinear map of the
  vertices) improve the chord lengths only 3 times on a 30-degree element (radius error 1.95e-4 against 5.8e-4): a cubic
  with the exact tangent lengths is not the best cubic of an arc. `EdgeSlopeScaling` takes per element edge the length s
  for which the cubic passes through the mapped midpoint of the edge, s = 8 (m - (p0+p1)/2).(t0-t1)/|t0-t1|^2 - for an
  arc 4 R tan(angle/4) - and the derivative length on straight edges, without a mapping, or when s leaves [0.5, 2]
  times the derivative length. Radius error 3.5e-7; flat meshes keep the chord lengths (1.1e-16). The plate tests are
  unchanged (flat or mapped without curvature along an edge).
- **#2259**: `SystemContainer(attachToRenderer=False)`; such a container can be attached later by SC.renderer.Attach().
  Destroying it does not detach the renderer either (DetachVisualizationSystem only detaches the container that is
  linked). InverseKinematicsNumerical uses it unless useRenderer. Test model systemContainerComputeOnlyTest.py, through
  exu.special.currentRendererSystemContainer. It found #2993: InverseKinematicsNumerical fails for links without
  PDcontrol (raised, RG11.8.20).
- **#948**: mecanumWheelRollingDiscTest follows a Trajectory (forward 4 m, sideways 4 m, half a turn, forward) with
  TorsionalSpringDamper motors (stiffness 0, damping 500, `velocityOffset` set in a PreStepUserFunction), as
  mobileMecanumWheelRobotWithLidar.py. The result cannot stay the same: before, the torque was a load on the wheel only
  and the velocity a step. After 20 s the car is 1 cm from the trajectory; the test runs 4 s (3.890 of 4 m).

<a id="rg11-8-21"></a>
### RG11.8.21 and RG4.20.5.2 — the clipboard of the dialogs, the rotation issues of the plate closed (2026-10-14, #2218, #2219, #2994)
*(Maintainer 2026-10-14: a line copied by "copy line" in the sim or vis dialog can be pasted only while the dialog
is open, on macOS and Windows, from "exudyn dialog vis" and from the V key; close #2218 and #2219 if resolved.)*
- **The cause**: tkinter's clipboard is served by Tk itself - it answers when another program pastes - so the text
  lives only as long as the Tk window. The `update()` after `clipboard_append` (added for this before) is not enough:
  reproduced on Windows with a process that goes on after the window is destroyed, as the renderer does, the
  clipboard is then empty; when the process ends right after the window, Windows keeps the text, which is why a
  short test looked fine.
- **The fix**: `GUI.CopyToClipboard(widget, text)` gives the text to the system - Windows: `OpenClipboard`,
  `SetClipboardData(CF_UNICODETEXT)` through ctypes, the memory then belongs to the system; macOS: `pbcopy` (with a
  UTF-8 locale); Linux: `wl-copy`, `xclip` or `xsel` if installed - and falls back to tkinter's clipboard. Used by
  "copy line" and by "copy all" of the code window. Checked on Windows: the text, with a non-ASCII character, is
  still in the clipboard after the window is destroyed in a running process and after the process ends; tkinter's
  way gives an empty clipboard in the same run. test_guiValues.py: the command per platform, and the fallback.
  macOS and Linux are not checked on a machine here.
- **#2218, #2219**: done with #2228 (RG11.8.8): ObjectANCFThinPlate has GetRotationMatrix, the angular velocity, the
  rotation Jacobian and its derivative (MarkerBodyRigid acts on it); NodePointSlope12 has the symmetric basis,
  symmetric in both slopes, with the consistent angular velocity, rotation Jacobian and the formerly missing
  GetRotationJacobianTTimesVector_q. Closed as resolved.

<a id="rg11-8-9"></a>
### RG11.8.9 — the initial conditions checked against the constraints, the projection of the initial velocities (2026-10-14, #2234)
*(Maintainer 2026-10-14: option A, the warning, in a try block, since the velocity level can fail - then the warning
names the flag that switches the check off, and the projection; option B, the projection, as proposed; the position
level too; check how often it occurs in the tests, ideally automatically, with values stored by the solver; #565, the
drift, follows later and is not touched.)*
- **The check** (CSolverImplicitSecondOrderTimeInt::CheckInitialConstraints, before the initial accelerations): the
  algebraic equations at position level and at velocity level, `ComputeAlgebraicEquations(..., false/true)`, each in a
  try block; the largest residual against `initialConstraintsTolerance` (default 1e-8) times 1 + the largest
  coordinate, respectively velocity coordinate. Velocity-level constraints (`UsesVelocityLevel`: rolling disc,
  CoordinateConstraint with velocityLevel) give their velocity equation at both levels and are left out of the
  position check - without that, a prescribed speed of 8 pi appeared as a position residual of 25.1. The warning names
  the item ("object 7"), the residual and the limit, the remedies, and that
  `generalizedAlpha.checkInitialConstraints = False` switches it off. Skipped for a run that continues from a restart
  file. If an evaluation fails (ObjectJointSliding and ObjectJointSliding2D have no velocity level), the warning says so,
  names the flag and the projection, and the simulation continues with what was evaluated.
- **The projection** (`generalizedAlpha.projectInitialVelocities`, default False): [M C_q^T; C_q 0][dv; mu] = [0; -g_v],
  the change of least kinetic energy; the matrix is the one of the initial accelerations (factored once, reused by
  them - FactorizeInitialSystemJacobian, extracted from the initial accelerations), the velocity residual is evaluated
  again afterwards. A successful projection does not warn; the position level is not corrected.
- **Stored values**: `solver.output.initialConstraintsResidualPosition`, `initialConstraintsResidualVelocity` (-1 if
  not computed), `initialVelocitiesProjected`; module-wide `exu.special.solver.initialConstraintsViolations`, the number
  of such time integrations. runTestSuite.py resets it per test model and mini example (also in parallel runs, through
  the result line of the worker) and lists the models at the end of the log.
- **How often** (suite of 2026-10-14): 15 of 192 test models, 32 time integrations (14 of them createItemNumbersTest); all at velocity level except
  sliderCrank3Dtest.py, whose GenericJoint slider-conrod constrains the rotation about z while the conrod starts
  rotated by 0.248 rad. Typical: one body of a mechanism gets an initial velocity, the others not (fourBarMechanismTest
  4.8, sliderCrank3Dbenchmark 0.48), a prescribed speed against an initial speed 0 (ballBearingTest 8 pi - its solver
  reduces the step size in the first steps -, coordinateSpringDamperExt, createFunctionsTest), a rolling disc without
  matching initial rotation (createRollingDiscTest), rigidBodyAsUserFunctionTest 725. Mini examples:
  MarkerBodiesRelativeTranslationCoordinate (position 0.3); ObjectJointSliding, ObjectJointSliding2D,
  MarkerBodyCable2DCoordinates, MarkerBodyBeamShape cannot be checked at velocity level. No reference value changed;
  the models are not changed here.
- **Test model initialConstraintsCheckTest.py**: a pendulum on a distance constraint. A radial initial velocity 0.5:
  residual 0.5, the constraint force of the first steps 28666 instead of 7.9; with the projection the solution is
  that of the consistent start to the bit; a distance 1% too long: position residual 0.01; the check off: -1, no
  count. Manual: docs/manual/solver.md, section *Consistent initial conditions*.
- **For #565** (the drift during the run, later): the evaluation of both levels and the naming of the item
  (`AlgebraicEquationItem`) are what a drift inspector needs at the end of a step; the counter in special.solver and
  the per-model list of the runner can take a drift count the same way.

<a id="rg4-1-3-8"></a>
### RG4.1.3.8 (open), RG7.3.4 and RG12.3.1 (planned) — Eigen on arm64 without a patch, the review of the user body, the user function boundary (2026-10-15, #2990, #2995, #2996)
*(Maintainer 2026-10-15: realize the Mac agent's fix without patching the Eigen sources - outside or by compiler
flags, then now, otherwise only propose; review ObjectUserBody and ObjectGenericODE2 into a new sub-step; the
allocations of the StdVector interface into an open step with a first sub-step to pre-analyze.)*
- **#2990**: the Mac agent found the rest of the difference in `EIGEN_HAS_SINGLE_INSTRUCTION_MADD`, which Eigen 3.3.7
  defines unconditionally on NEON (`#ifndef ... #define`, so no `-D` can prevent it), on SSE only under `__FMA__`.
  In GeneralBlockPanelKernel.h it selects mr = 3 packets instead of the default 2 (double on NEON: 6 instead of 4)
  and `pmadd` instead of `pmul` + `padd`. Eigen/Core includes, in this order, the NEON packet math (line 401),
  DenseBase.h (435) and the product kernels (491); DenseBase includes the user header named by
  `EIGEN_DENSEBASE_PLUGIN` inside the class. That header, `src/Linalg/EigenPluginNoMadd.h`, contains only the
  `#undef`, and LinearSolver.h - the only file that includes Eigen - sets the plugin on `__aarch64__`. Eigen stays
  unchanged, the configuration is in one place with the `__ARM_FEATURE_FMA` handling of 03c90435. Checked on x86
  in WSL with g++ `-mavx2 -mfma`, where Eigen's AVX header defines the macro the same way: mr for double 12 without,
  8 with the plugin, through the include path of the build (`-Isrc -Iinclude`), and SparseLU solves. Windows and
  Linux x86-64 do not compile the branch. tmp/MacOS/MacAgentRequest5.md asks the Mac agent for the suite without its
  header patch. ARCHITECTURE.md lists the configuration beside the vendored libraries. The option of a newer Eigen
  stays open; the fast module (AVX2 + FMA, mr 12) could take the same plugin to give the bits of the regular module,
  not done, as it is meant to be fast.
- **#2995** (RG7.3.4, RG7.3 reopened for it): the review of ObjectUserBody and ObjectGenericODE2, 16 points in the
  plan step, to be decided before the change.
- **#2996** (RG12.3.1, under the design of the user function interfaces): the pre-analysis of the allocations at the
  boundary, related to #1864 and #1910.

<a id="rg4-1-3-8-closed"></a>
### RG4.1.3.8 closed — the Mac suite passes with the Eigen plugin (2026-10-15, #2990)
*(Maintainer 2026-10-15: the Mac reports the whole test suite passing - close the issue.)* The macOS ARM run of
b8507905, with the vendored Eigen unchanged, passes all test models and mini examples. `UnresolvedOnMacOS()` is kept
as it is: which of its entries now match Windows to the bit needs the Mac log, which is not in tmp/MacOS. The text of
the plan step while it was open:

#2990. The macOS ARM run of 1.13.75 fails in six test models (createSphereQuadContact, createSphereTriangleContact,
sphereTriangleTest2, sliderCrank3Dbenchmark, and the listed ANCFgeneralContactCircle and
geometricallyExactBeamRightAngleFrame), all solved with `LinearSolverType.EigenSparse`. The dumps of the Mac agent
(tmp/MacOS/MacAgentResponse, `dumpRun.py`) against the same script on Windows: the inputs - initial and reference
coordinates, every item, the numbers numpy computes in sliderCrank3Dbenchmark - are the same bits, the explicit solve
of sphereTriangleTest2 too; the implicit solves part by one ulp in some step. Eigen's NEON packets fuse a*b+c
(`vfmaq_f64` under `__ARM_FEATURE_FMA`) whatever `-ffp-contract=off` says. LinearSolver.h undefines the macro around
the Eigen includes; done on Windows (no change there), open until the Mac run confirms; then `UnresolvedOnMacOS()`
shrinks. The same holds for the fast module, whose `-mfma` makes Eigen's AVX packets fuse (`__FMA__`) - a separate
decision, as that module is meant to be fast.

2026-10-13, the Mac run of the change (tmp/MacOS/MacAgentResponse2): clang's arm_neon.h needs the macro, so it is
included before the macro is undefined. With that, sliderCrank3Dbenchmark is bit-identical with Windows and
ANCFgeneralContactCircle passes; 7 of the 8 entries of `UnresolvedOnMacOS()` pass. Four models still differ:
createSphereQuadContact, createSphereTriangleContact and sphereTriangleTest2 (EigenSparse; from step 13, resp. after step
50, by one ulp in 32 of 49 coordinates at once) and geometricallyExactBeamRightAngleFrame (static, EigenSparse). Next:
the same dumps with `LinearSolverType.EXUdense` on both platforms (`tmp/MacOS/dumpRun2.py`, `compareDumps2.py`, the
Windows dumps in `WindowsReference2.zip`) - identical then means the rest is in Eigen too.

2026-10-15, the Mac agent's result (tmp/MacOS/MacOSeigenFixTestSuite.md): EXUdense is bit-identical; the rest is
`EIGEN_HAS_SINGLE_INSTRUCTION_MADD`, which Eigen 3.3.7 defines on NEON unconditionally (on SSE only with `__FMA__`). It
sets the block height of the product kernel (mr = 3 instead of 2 packets) and a fused multiply-add in it, and with them
the order of the sums in SparseLU; with the macro off, all four models match Windows in every step and the whole suite
passes on the Mac (patched in the vendored header, not committed). Done without a change of Eigen: LinearSolver.h sets
Eigen's documented `EIGEN_DENSEBASE_PLUGIN` on arm64 to `Linalg/EigenPluginNoMadd.h`, which only undefines the macro;
Eigen/Core includes the plugin inside DenseBase, after the NEON packet math and before the product kernels. Checked on
x86 with `-mavx2 -mfma`, where Eigen's AVX header defines the macro the same way: gebp mr 12 -> 8 for double, SparseLU
solves. Open until the Mac suite confirms it without the local patch; then `UnresolvedOnMacOS()` loses the four.

<a id="rg4-25"></a>
### RG4.25.1 and RG4.25.2 — the drift of the constraints, in the solver and in Python (2026-10-15, #565)
*(Maintainer 2026-10-15: monitoring in a solver function at FinishStep if checkConstraintDrift, default False;
constraintDriftTolerance in the units of the constraint, stop above it with the remedies in the exception; check
period 20 steps; warning hard-wired at 0.001 of the tolerance; output: the largest residual and the item; warn once
and a summary at the end; SolveDynamic gets index2CheckConstraints for TrapezoidalIndex2; no counter in
exu.special; measure first, then the tolerance; a Python function like ComputeLinearizedSystem instead of a
systemData function; no projection, it destabilizes the index 2 solver; sensor user functions, energy later; the drift
as output variable of the joints. After the measurement: 1e-5 and 1e-8, TrapezoidalIndex2 only, mention the drift in
the tests, reduce the step size where it stops.)*
- **The solver** (CSolverImplicitSecondOrderTimeInt): `ConstraintResidual` evaluates the algebraic equations at
  position or velocity level, at position level without the equations of velocity-level constraints (they give their
  velocity equations at both levels); used by the initial check of #2234 and by `CheckConstraintDrift`, called in the
  overridden `FinishStep` every `constraintDriftCheckPeriod` steps. Above 0.001 `constraintDriftTolerance` one warning
  per run with item and time, above the tolerance an `ExudynSolverError` with the remedies (step size, index 3,
  tolerance, the flag); `FinalizeSolver` reports the largest drift if a warning came. `output.constraintDriftMaximum`
  (-1 if not checked), `constraintDriftItem`, `constraintDriftTime`. The two free static helpers of #2234
  (`MaxAbsAndIndex`, `AlgebraicEquationItem`) became members.
- **Measured** (suite of 2026-10-15, the check forced on for every dynamic solve with an unreachable tolerance, in the
  installed solver.py only): 407 index 3 solves - ordinary joints at most 6.6e-9 (the Newton tolerance), the sliding
  joints (ObjectJointSliding, ObjectJointSliding2D) 0.00995 and SlidingJoint2DTest, connectorFrameOutputsTest 5e-5 to
  9e-5: their equations include forces (the force along the slide is zero), not only positions. 24 index 2 solves:
  laserScanner 1.5e-6, rollingCoin 2.5e-6, carRollingDisc 2.5e-6, SliderCrank2D 3.3e-6, mecanumWheel 5.9e-6 (all
  Euler parameters of nodes except SliderCrank2D), rotatingTable 1.9e-5, rigidBodyCOM 2.6e-4, ANCFslidingAndALEjoint
  0.017 (the ALE joint), sliderCrank3Dtest 0.248 (its initial positions, #2234). A pendulum with h = 2 ms drifts 1e-5
  in 2 s. Hence 1e-5; of the TrapezoidalIndex2 models only rotatingTableTest exceeds it - its step size is halved
  (drift 4.8e-6, new reference); the index 2 test models note their drift at the solver setting.
- **Python**: `mbs.ComputeConstraintResiduals(velocityLevel=False)` - per item with `mbs.ComputeItem`, without a
  solver: a solver's InitializeSolver copies the initial state into the current one, so it could not be used during a
  simulation; returns the residuals and the ObjectIndex or NodeIndex of each entry. `ComputeItemType.ConstraintDrift`
  (position level, zero for velocity-level constraints) and `AlgebraicEquations_t`, both and `AlgebraicEquations` also
  for nodes with algebraic equations (Euler parameters). `SensorUFSystemConstraintDrift` (advancedUtilities): [largest
  drift, largest velocity-level residual], NaN for the latter where an item has no velocity level.
- **The output variable**: `ConstraintEquation` existed for CoordinateConstraint and CoordinateVectorConstraint; it is
  now computed for every constraint in `GetConnectorOutputVariable` (CObjectConstraint::GetOutputVariableConstraintEquation,
  the connector interface or the marker data) and listed for ObjectConnectorDistance and the ten joints. The two
  former implementations passed the velocity flag in the place of the item number.
- **Test model constraintDriftTest.py**: pendulum and free rigid body with Euler parameters, TrapezoidalIndex2:
  drift 9.8e-6; the residual function, ComputeItem, the output variable and the sensor agree to the bit; the flag off
  gives -1; a tolerance below the drift stops the solver with the message; index 3 checked: 2.8e-13.
- **Not done**: the energies as sensor functions (#2997, RG4.25.3); a projection (decided against).

<a id="rg12-3-1"></a>
### RG12.3.1 — the user function boundary analysed; two Linalg bugs found (2026-10-15, #2996, #2998, #2999)
*(Maintainer 2026-10-15: continue with the pre-analysis; Python user functions run only serially, no GIL problem;
show the matrix problem; there should be no visualization cache in ObjectUserBody, the user function graphics is
computed in the main thread.)*
- **The analysis**: docs/dev/design/userFunctionBoundary.md with its script userFunctionBoundary.py. The path of a
  `forceUserFunction` call counted from the code: `Vector` to `StdVector` (2), the `std::function` by value (2),
  symbolic: the lambda and `EvaluateStdVector` by value (2), `ResizableConstVector(const std::vector<T>)` by value
  (2, +2 above 7), the result `ResizableConstVector` -> `StdVector` -> `Vector` with a by-value constructor (3), no
  move assignment of `Vector` (1). Measured with the solver timer userFunctions: symbolic 0.31 microseconds with
  vectors, 0.035 to 0.050 with scalar arguments; Python 0.45 (n = 1) to 29 (n = 1000) for returning a prepared list,
  0.25 plus 25 ns per value; a Python function receives lists, numpy results are slower than lists. Options A
  (const references), B (persistent buffers), C (a direct symbolic path into Exudyn vectors), D (numpy views for
  Python), E (move assignment of Vector).
- **#2998**: the measurement crashed (0xC0000374) for a symbolic forceUserFunction with 10 coordinates, deterministic
  from n = 8: `ResizableConstVectorBase::SetNumberOfItems`, when the size does not grow, sets `data = &localData[0]`
  also for a vector on the heap; the second call writes 10 values into 7, the destructor deletes the local buffer.
  AddressSanitizer (WSL, g++, the Linalg headers only): stack-buffer-overflow at the second SetAll. Raised as a bug,
  RG12.3.2; not fixed in this step.
- **#2999, the matrix problem** (RG7.3.4 point 16): `MatrixBase::operator=(MatrixBase&&)` swaps data, rows and
  columns; through a `Matrix&` to a `ResizableMatrix`, `allocatedSize` keeps the old capacity. AddressSanitizer: a
  ResizableMatrix of 6 x 6 gets a moved Matrix of 3 x 1, `SetNumberOfRowsAndColumns(6, 6)` stays within the old
  capacity, `SetAll` writes 36 values into the 24-byte buffer - heap-buffer-overflow. On a `LinkedDataMatrix` the
  temporary would delete the linked memory. Vectors are not affected: `VectorBase` has no move assignment.
- **The renderer and ObjectUserBody** (RG7.3.4 point 17): the graphicsDataUserFunction runs in the main thread
  (PostProcessData::ProcessUserFunctionDrawing on request of the renderer); the render loop itself calls
  UpdateGraphicsData, which draws markers, connectors and sensors and asks the bodies for positions in the
  visualization configuration - for a user body a Python call from that thread, which the cache avoids. Recorded as a
  decision for RG7.3.4.

<a id="rg12-3-2"></a>
### RG12.3.2 and RG7.3.4.1 — the ResizableConstVector and the assignments of the matrices and vectors (2026-10-15, #2998, #2999)
*(Maintainer 2026-10-15: first fix the matrix problem with the move assignment and the ResizableConstVector #2998.)*
- **#2998, ResizableConstVector** (src/Linalg/ResizableVector.h): one rule now - the data is on the heap if and only if
  maxNumberOfItems > maxLocalSize (7), and then maxNumberOfItems is its size; `AllocateMemory` records it in both
  branches (before, the constructors with a size or a std::vector above 7 left it 0: the heap memory leaked);
  `MaxNumberOfItems()` is the memory the vector has, the local 7 or the heap size; `SetNumberOfItems` allocates only
  if the size does not fit and leaves the data where it is otherwise. The constructor with a value called `SetAll`
  without `this->` (GCC refuses it; it was never used).
- **#2999, the assignments**: `MatrixBase::operator=(MatrixBase&&)` exchanges the memory only if both matrices own
  exactly their rows x columns (a virtual `OwnsExactMemory`, false for ResizableMatrix and LinkedDataMatrix), and copies
  otherwise. Found while testing it: the GENERATED copy assignments of ResizableMatrix and ResizableVector (the class
  declares a copy constructor and a template operator=, neither suppresses it) copied the allocated size of the other
  object together with the values - a ResizableMatrix of 4 assigned from one of 1 x 1 within 36 believed to have 36
  (AddressSanitizer: heap-buffer-overflow), the same for ResizableVector and with it ResizableVectorParallel. Both now
  have copy assignments that copy the values through the base and keep their own capacity; ResizableMatrix a move
  assignment that swaps the memory with the capacity. `EvaluateMatrix` of ObjectUserBody moves its result again.
- **Tests**: src/Tests/AllVectorUnitTests.h (ResizableVector copy assignment; ResizableConstVector set again above 7,
  shrink, grow, the constructors) and AllMatrixVariantsUnitTests.h (a Matrix moved into a ResizableMatrix and into a
  LinkedDataMatrix through Matrix&, copy and move assignment of ResizableMatrix, the move between two Matrix objects
  still exchanging memory). Run in WSL with g++ and AddressSanitizer (the vector and matrix-variant lists): all pass,
  no sanitizer report; with the old headers the vector tests fail and AddressSanitizer reports the invalid free.
  Test model symbolicVectorUserFunctionTest.py: ObjectGenericODE2 with 10 coordinates, symbolic forceUserFunction
  equal to the Python one to the bit; it crashed before. userFunctionBoundary.md has the symbolic column above 7 now:
  0.45 microseconds at 10, 0.67 at 100, 4.2 at 1000.

<a id="rg6-8-8"></a>
### RG6.8.8, RG12.3.3, RG4.25.3 (proposal) and RG4.1 — the near plane, the user function interface, energy and drift (2026-10-16, #2194, #2204, #2996, #2997)
*(Maintainer 2026-10-16: the user items - evaluate the Python and symbolic user functions again; which vector and
matrix classes, what must they do, such that the interface is smoothest for Python and symbolic, ideally without
allocation on the Exudyn side, symbolic first; a proposal for the linalg components, the symbolic interface, helpers,
and the user items, into userFunctionBoundary.md. Is RG4.1 resolved? RG4.25: energy and drift homogenized, two
classes with similar functionality at system level. RG6.8.8 with the raytracer, fixes that match OpenGL, proposals
for the shadows and camera settings; not much investment before OpenGL 3.3/4.1.)*
- **RG6.8.8, the near plane (#2204)**: measured with the raytracer, whose lines are cut at the same near plane as
  OpenGL's triangles: the sensor trace of testRaytracerTracesInPerspective scaled to 1/100 disappeared completely in
  the perspective, model-centric view - the eye is zoom/perspective = 0.084 from the focal plane, inside the fixed near
  plane 0.1 (`RenderState::GetProjectionNearMin`). Now `min(0.1, 0.001 maxSceneSize)`: scenes of 100 and larger
  unchanged, the ratio of far to near plane about 1000 for the depth buffer. OpenGL (GlfwRenderer::SetProjection) and
  the raytracer take it from the same function, so the fix is the same in both. Scale 1, 0.01, 0.001: trace and sphere
  visible in both projections (with `minSceneSize` and `boundingBoxZoomAllOffset` scaled, see #3001). pytest
  testASmallModelStaysVisibleInPerspective. **#2194**: `camera.nearFarPlaneOffset` with its flag exists; its
  description said 0.01 for the camera-centric near plane, which was 0.1, and is corrected.
- **Lights and shadows** (scratch script, 12 raytracer images): a directional or positional global light keeps its
  shadows on the model when the model is rotated or panned; a camera-frame light moves them; perspective
  model-centric is consistent. The light transformations of the raytracer (global: R p - w c) and of the OpenGL
  shadow volumes (camera frame: p R + (1 - dir) c R) are the same transformation in the two directions. OpenGL itself
  cannot be rendered here without a window; nothing found that needs a fix now.
- **Found, for decision** (#3000, RG6.8.10): the camera-centric view - field of view atan(zoom perspective), with zoom a
  length (1 degree for a model of 1 cm, 84 degrees for 1 m), ZoomAll does not place the eye, the far plane does not
  follow it; the raytracer clips no triangles. (#3001, RG6.8.11): ZoomAll of small scenes is limited by two absolute
  settings. Both left for the OpenGL 3.3/4.1 renderer or a decision.
- **RG12.3.3**: the user functions again - 42 parameters, 28 with a symbolic version; the symbolic evaluation itself
  returns every node by value. userFunctionBoundary.md sections 5 to 11: requirements R1-R7, the linalg components
  (views as arguments, resizable results, copy from numpy in one pass, numpy views), a generated `Evaluate` per
  signature that dispatches on the kind, linked named variables and node buffers for symbolic, numpy views for
  Python (a change from lists, with a switch or 2.0), the points of RG7.3.4 decided by it; steps in section 11.
- **RG4.25.3**: proposal of `SystemQuantity` with `SystemEnergy` and `SystemConstraintDrift` in the plan step.
- **RG4.1**: Linux resolved; macOS needs a log of a full run (tmp/MacOS/MacAgentRequest6.md).

<a id="rg6-8-10"></a>
### RG6.8.10, RG6.8.11, RG4.1 closed, RG4.25.3 concept — the camera-centric view, small scenes, macOS, system utilities (2026-10-16, #3000, #3001)
*(Maintainer 2026-10-16: the successful macOS run is tmp/MacOS/testSuiteLog_V1.13.90.dev1_darwin-ARM-64bit-P3.13.txt;
the space key of the SolutionViewer works on macOS. #3001: absolute by intention, as a point has no size; minSceneSize
could be smaller if the scene were computed more accurately; boundingBoxZoomAllOffset 10 times smaller; a short section
for tiny or huge models. #3000: fix it, also for the raytracer. RG4.25.3: all system functions in one library,
DrawSystemGraph split, a good name; the concept into design/systemUtilities.md.)*
- **RG4.1 closed**: the macOS log of 1.13.90 - all 193 test models and 94 mini examples within their tolerance; the
  seven models of `UnresolvedOnMacOS()` that had macOS-only differences give the Windows values (error 0, one 1e-16),
  the largest differences elsewhere 1e-14 to 1.2e-12. The list keeps objectFFRFreducedOrderTest.py for cp314 (numpy's
  eigenvalue solver). RG4.1.2 (the mechanism: Eigen's fused multiply-add on NEON) and RG4.1.3 (the math library)
  closed with it. RG6.18.2 notes the space key on macOS.
- **#3000**: `RenderState::ComputePerspectiveFrustum` gives the eye on the view axis, the near and far plane and the
  angle of view, used by `GlfwRenderer::SetProjection` and by the raytracer (rays, lines, texts, and now the clipping
  of its triangles at the near and far plane of primary rays, which OpenGL always did). Model-centric: unchanged (the
  same values as before). Camera-centric: tan(angle/2) = perspective zoom / (maxSceneSize/2) instead of
  perspective zoom - the zoom, a length, made a model of 1 cm look through 1 degree and one of 1 m through 84; the far
  plane at |translation| + zMaxSceneFactor maxSceneSize from the eye instead of zMaxSceneFactor maxSceneSize; ZoomAll
  sets `camera.cameraPosition` (a setting; the rotation pivots about the camera, so its position decides) in front of
  the center of the scene at maxSceneSize/(2 perspective) - before, it stayed in the scene and showed nothing. The
  camera-centric inverse projection of the raytracer's lines used the model-centric formula; corrected. Checked with
  12 raytracer images of light, rotation, pan and both views: the camera-centric images show the scene as the
  model-centric ones. pytest testTheCameraCentricViewShowsTheScene (scale 1 and 0.01).
- **#3001**: the box of the scene counts spheres, circles, lines, triangles with their size; points drawn in pixels and
  texts have none - that is what minSceneSize is for. `minSceneSize` 0.1 -> 0.01, `boundingBoxZoomAllOffset`
  0.01 -> 0.001; descriptions and GUI.md section *Tiny and huge models*.
- **RG4.25.3**: docs/dev/design/systemUtilities.md - `exudyn.systemUtilities`; what moves (SystemEnergy,
  LoadPotentialEnergy, ComputeConstraintResiduals, ItemODE2Coordinates, NumericalJacobian, ComputeSystemDegreeOfFreedom,
  DrawSystemGraph) and what is removed (CreateLoadEnergySensor, SensorUFSystemConstraintDrift - both since 1.12/1.13);
  `SystemQuantity` with `Compute`, `ComputeItems`, `SensorFunction`, `AddSensor`, `items`, `unavailable`;
  `SystemEnergy`, `SystemConstraintDrift`; `ComputeSystemGraph` and `DrawSystemGraph`; imports for compatibility.

<a id="rg13"></a>
### RG13 added, RG12.3.3 revised — total coordinates as a group, Vector& in the user function interface (2026-10-16, #1910, #3002)
*(Maintainer 2026-10-16: why not `const Vector& q` instead of `LinkedDataVector`, as it includes the linked vector and
is more flexible - adjust if agreed. Total coordinates will be stored in the solver in the future, so their efficiency
is less a question for the current proposal; a new group for total coordinates in system data, solvers and item
functions, starting with an evaluation of how to do it efficiently; the solvers act on coordinates relative to the
reference, total coordinates are an additional end product - a vector addition and cache memory, mostly relevant for
the explicit solvers; after extended performance evaluations with more models, solvers and configurations.)*
- **userFunctionBoundary.md**: agreed - `Vector` is `VectorBase<Real>`, the base of `ResizableVector` and
  `LinkedDataVector`, so `const Vector&` takes all three without a conversion; only memory that is no `VectorBase`
  (`std::vector`, a pointer, `ConstSizeVector`, `SlimVector`) needs a `LinkedDataVector` around it. The results become
  `Vector&` as well: `SetNumberOfItems` is virtual, so a `ResizableVector` keeps its capacity and a `LinkedDataVector`
  of the right size takes the result in place. Matrices the same with `Matrix`. Point 13 of RG7.3.4 waits for RG13.
- **RG13**, a new group: RG13.1 the performance baseline over more models, solvers and configurations (#3002), RG13.2
  the evaluation of how the total coordinates are provided (#1910, kept open as an idea since 2024).

<a id="rg12-3-4"></a>
### RG12.3.4-RG12.3.7 and RG7.10 planned — the decisions on the user function interface (2026-10-16, #2996, #3003)
*(Maintainer 2026-10-16: numpy for q in the Python user functions - it was a flaw that it was not; a switch in
exudyn.special.deprecations for the old behaviour only if it causes no significant overhead with the planned changes;
nobody should have concatenated lists, so numpy in any case. No visualization cache in ObjectUserBody - it would not
work with the SolutionViewer; items that depend on Python user items cannot be drawn in the visualization thread: a
list computed in Assemble that marks them, or a second function besides HasUserFunction that evaluates the
dependencies, in a separate step; a mechanism that detects a Python (not symbolic) user function called in the
visualization thread, a global flag. The order of the steps was generally good; integrate the decisions.)*
- userFunctionBoundary.md: section 9 decided (numpy, the switch only if free), point 17 decided (no cache, RG7.10),
  section 11 the order: RG12.3.4 (const references), RG12.3.5 (generated Evaluate, symbolic node buffers), RG12.3.6
  (numpy), RG7.10 (drawing in the main thread, detection), RG7.3.4 (the user items without the cache), RG12.3.7
  (matrices).
- RG7.10 a new step of RG7 (#3003), as the maintainer asked; the four sub-steps of RG12.3 carry #2996.

<a id="rg4-25-3-numjac"></a>
### RG4.25.3 — NumericalJacobian revised, ItemODE2Coordinates removed (2026-10-16, #3004)
*(Maintainer 2026-10-16: both functions need to be revised, probably removed. ItemODE2Coordinates exists for objects in
mbs.systemData - is it required for other items, and if so, why not in C++? NumericalJacobian: velocityLevel instead of
velocities, systemCoordinates instead of coordinates, epsilon about 1e-8 as elsewhere in Exudyn. Both do not meet the
docstring conventions; if purely internal, a place like misc.internal. Revise before continuing with systemUtilities.)*
- **ItemODE2Coordinates removed**: for objects it was `GetObjectLTGODE2`, for nodes `GetNodeLTGODE2` - both C++ already;
  for markers it raised. The callers (test_accessFunctionsAllBodies.py, computeItemTest.py) use
  `mbs.systemData.GetObjectLTGODE2`.
- **NumericalJacobian(mbs, function, systemCoordinates, velocityLevel=False, epsilon=1e-8)**: the docstring with Args,
  Returns, Example. Kept public, not internal: `mbs.ComputeItem` is public for testing and debugging a user's item, and
  NumericalJacobian is what it is compared with (the ComputeItem description and the 1.13 revisions say so); it moves to
  systemUtilities with step 1 of the concept.
- **epsilon**: 1e-8 as the numerical differentiation of the solvers, absolute here. With central differences the
  round-off is about 1e-16/epsilon = 1e-8, which is above the relative tolerance 1e-7 of
  test_accessFunctionsAllBodies.py at some of its random states; the test passes `epsilon=1e-6` (truncation about 1e-12),
  with a comment. computeItemTest.py unchanged (71.26280786962762).

<a id="rg4-25-3"></a>
### RG4.25.3 — exudyn.systemUtilities (2026-10-16, #2997)
*(Maintainer 2026-10-16: ComputeSystemDegreeOfFreedom also requires the solver, so it stays there - remove it from the
systemUtilities plan; implement systemUtilities.md as far as it can be done now and adapt the docs.)*
- **python/exudyn/systemUtilities.py**: `SystemQuantity` (items collected once, `Refresh`, `Compute`, `ComputeItems`,
  `SensorFunction`, `AddSensor(components=...)`, `.items`, `.unavailable`); `SystemEnergy` (kinetic, potential, loads,
  total) and `SystemConstraintDrift` (positionDrift, velocityResidual; the maximum over the items, NaN where an item
  has no velocity level); `LoadPotentialEnergy`, `NumericalJacobian` from advancedUtilities;
  `ComputeConstraintResiduals` from solver.py; `ComputeSystemGraph` and `DrawSystemGraph` from mainSystemExtensions,
  both `@extends` in the module, which mainSystemExtensions imports before `install()`.
- `ComputeItems` returns a list of (item, values), not a dict as the concept said: `ObjectIndex(3) == ObjectIndex(3)`
  is False and their hashes differ, so item indices cannot be keys.
- `CreateLoadEnergySensor` and `SensorUFSystemConstraintDrift` removed (1.12/1.13, as decided); `utilities.py`
  star-imports the module - `DrawSystemGraph` comes with `from exudyn.utilities import *` again, as before 1.11, and
  `tools/checkUserScripts.py` no longer reports it (its test uses `CreateDistanceSensor` instead).
- `DrawSystemGraph` against the one of 1.13.6 (venvP313, which has networkx) on a model with joint, spring, loads and
  sensors, for six combinations of the label options, with and without drawing: the same graph nodes in the same
  order, the same edges and edge colors, the same node data (only the order of the keys differs).
- Tests: energiesTest.py, constraintDriftTest.py, computeItemTest.py, mainSystemExtensionsTests.py give their
  reference values unchanged; new systemUtilitiesTest.py (the graph part runs only where networkx is installed - not
  in venvExuP313).
- Docs: introductionBasics.md section *Quantities of the whole system* (`sec-systemquantities`), solver.md drift
  section, revisions.md (after 1.13), the references in definitions and MainSystem.cpp; design/systemUtilities.md
  describes the module as it is. RG4.25 closed with it.

<a id="rg12-3-4-done"></a>
### RG12.3.4 — the vectors of the user functions by const reference, the comparison with 1.13.6 (2026-10-16, #3005)
*(Maintainer 2026-10-16: the 6 steps of userFunctionBoundary.md section 11 are all good to go; whenever something
advances for the user functions, a performance comparison of a couple of typical user functions with 1.13.6, possibly
in the performance tests; the performance should rather improve, or at least not drop; for Python and symbolic user
functions, so that both effects are seen.)*
- **definitions/definitionTypes.py**: `annotationToCppArgument` - a `Vector` argument of a user function is
  `const StdVector&` in its std::function; the return stays `StdVector`. The generated templates and members change with
  it; `itemInterfaceEmitter` reads `const StdVector&` as `StdVector`, so `userFunctionArgsDict` (and itemInterface.py)
  is unchanged.
- **SymbolicUtilities.h**: the nine `Evaluate...` templates take `const Args&...`; the generated lambdas pass their
  references through.
- **Linalg**: the constructors of VectorBase, ResizableVectorBase, ResizableConstVectorBase, ResizableVectorParallelBase
  and SlimVectorBase from `std::vector` take a const reference; `VectorBase::operator=(VectorBase&&)` exchanges memory
  between two plain Vectors (`GetType() == VectorType::Vector`) and copies otherwise - the rule of #2999; unit test
  in AllVectorUnitTests.h (plain, resizable through a `Vector&`, linked, from a resizable).
- **evaluateUserFunctions.cpp**: the seven `force = (Vector)(...)` are `force.CopyFrom(...)` - no temporary Vector.
- **The measurement** (maintainer's request): `python/PerformanceModels/perfUserFunctions.py` - spring (scalars),
  cartesian (3D vectors in and out), load (3D), generic6/10/100 (ObjectGenericODE2 force) as Python and symbolic
  functions computing the same - in the performance suite (group small, 12 single runs with reference values, the
  Python and symbolic results identical); `docs/dev/design/userFunctionPerformance.py` runs each case in a process of
  its own in the current version and in a reference package directory (a copy of site-packages/exudyn of 1.13.6 from
  venvP313), subtracts the same model without user function, and prints a table: userFunctionBoundary.md section 12.
  The symbolic generic10/generic100 crash in 1.13.6 (#2998), hence generic6.
- **Result**: symbolic vector functions 35-45 % faster than before the step (generic6 0.40 -> 0.22 us per call against
  1.13.6, generic100 1.10 -> 0.69), Exudyn allocations per call 2.5 -> 1.25 (10 -> 8.75 above 7 items); Python vector
  functions unchanged (the lists and the Python body dominate); scalar and 3D ones unchanged. `spring` symbolic is about
  10 % slower than 1.13.6 before and after the step: `u**3` is the portable power since #2954.
- Gates: build, regeneration, suite with the C++ unit tests, pytest, performance suite (all 37 single runs).

<a id="rg12-3-5"></a>
### RG12.3.5 — symbolic vector user functions without allocations (2026-10-16, #3006, #3007)
*(Maintainer 2026-10-16: the 6 steps of section 11 are all good to go; every advance measured against 1.13.6, for
Python and symbolic user functions.)*
- **Node buffers** (Symbolic.h, SymbolicVector.h): `VectorExpressionBase::EvaluateRef()` returns the value in the memory
  of the node - a mutable `ResizableConstVector buffer`, kept between the evaluations -, `Evaluate()` copies it; every
  vector node (the list of SReal, +, -, scalar*vector, matrix*vector and its transpose, unary + and -, MultComponents)
  evaluates into its buffer from the references of its operands; the scalar nodes of vectors (scalar product, NormL2,
  component, == and !=) read their operands by reference. A node used twice in one expression is evaluated twice with
  the same arguments into the same buffer, which gives the same values.
- **Arguments and result** (SymbolicUtilities.h): `processArgument` for `StdVector` and for Exudyn vectors copies into
  the memory of the named vector (no temporary `ResizableConstVector`); the returns of 2D/3D/6D/StdVector read the
  result by reference; `EvaluateIntoVector(result, ...)` copies the result into `result`.
- **The call**: `PythonUserFunctionBase` keeps the pointer of the symbolic function its `pyObject` holds (set in
  `SetPythonUserFunction`, reset in `SetPythonObject`); `EvaluateUserFunctionIntoVector(userFunction, result, mbs,
  args...)` evaluates a symbolic function directly and a Python one through its std::function, converting Exudyn vector
  arguments with `UserFunctionArgument`. One template for all signatures instead of a generated `Evaluate` per
  signature - the same effect with less generated code. ObjectGenericODE2, ObjectGenericODE1, ObjectFFRF,
  ObjectFFRFreducedOrder and ObjectKinematicTree pass their coordinates as Exudyn vectors (definitions: `const Vector&`
  in `EvaluateUserFunctionForce`/`RHS`) and keep the force in the new member `tempUserFunctionForce` (no interface).
- **Not done, deliberately**: the linked named variables (R6) - a copy of the arguments costs one pass and no
  allocation, while a link would change the type every vector node reads; ObjectUserBody and
  ObjectConnectorCoordinateVector (RG7.3.4), the matrices (RG12.3.7).
- **Measured** (userFunctionPerformance.py against 1.13.6, userFunctionBoundary.md section 12): symbolic generic6
  0.407 -> 0.061 us per call, generic10 0.069, generic100 0.306 (after RG12.3.4: 0.385, 0.692); load 0.71, cartesian 0.84
  of 1.13.6; no Exudyn allocation per call in any run; Python unchanged.
- **Test**: symbolicVectorUserFunctionTest.py with a second force of every kind of vector node against its Python
  version (difference 3.5e-18; the same with 1.13.6 for 6 coordinates). Writing it found #3007: `q[0]` of a vector
  argument cannot be recorded (1.13.6 too) - RG12.3.8.

<a id="rg12-3-6"></a>
### RG12.3.6, RG12.3.8 — numpy arguments for Python user functions, components in symbolic ones, the symbolic user rigid body (2026-10-16, #3007, #3008, #3009)
*(Maintainer 2026-10-16: do RG12.3.6 and fix #3007; build the userRigidBody example also with symbolic user functions,
as far as possible - both modes in the same test model; completely possible at the end of the user function and
symbolic extensions.)*
- **#3007**: `VectorExpressionOperatorBracket::Evaluate` gives 0 while a function is recorded and the vector has no size
  (the arguments are recorded with size 0); out of range otherwise raises as before. `q[0]` in a symbolic vector user
  function works; symbolicVectorUserFunctionTest.py uses it (new reference 0.5094476793074225).
- **#3008, the Python path**: `PythonUserFunctionBase::SetPythonObject` makes the std::function with
  `PythonUserFunctionToStdFunction` (SymbolicUtilities.h) instead of `py::cast<UFT>`: a `StdVector` argument becomes a
  read-only numpy view (`EPyUtils::ToNumpyView`, base None: no copy, not writeable), every other argument is cast as
  pybind11 casts it; the result goes through `PythonUserFunctionResult`, a `StdVector` from a 1D numpy array in one
  pass (`EPyUtils::CopyFromPython`); a `PythonFunctionHandle` releases the function under the GIL, as pybind11's own
  wrapper does. `EvaluateUserFunctionIntoVector` calls a Python function directly with views of the Exudyn vectors
  and copies the result into the item's vector - no `std::vector` on that path.
- **The switch**: `exudyn.special.deprecations.userFunctionListArguments` (False) gives lists; one bool test per vector
  argument, so it is free, as the maintainer required; its use is reported with PyDeprecated ('settings', removed 2031).
- **Not changed**: the fixed-size vectors (Vector3D, Vector6D) and matrices stay lists, as section 9 decided only for
  the run-time sized ones.
- **Measured** (section 12): Python generic6/10/100 2.26/2.25/2.35 us per call against 2.99/3.35/10.48 in 1.13.6; scalar
  and 3D unchanged; a list comprehension over `q` is slower at small n, as numpy scalars are slower to iterate.
- **userRigidBodyTest.py**: a third body, ObjectUserBody with a symbolic forceUserFunction and positionUserFunction;
  the force and the position are written once with components (MatVec, Cross, GLocal, the rotation matrix) and run
  with `exudyn.cMath` as Python functions (checked against the matrix versions: 5.6e-17) and with `exudyn.symbolic`
  as recording; the Python and the symbolic user body agree to 6.5e-14, both agree with ObjectRigidBody; the mass
  matrix, rotation and Jacobians return matrices and stay Python until RG12.3.7. The reference value is unchanged.
- **Docs**: manual section *User functions* (`sec-userfunctions`, introductionBasics.md); every user function with a
  `Vector` argument links it from its reference page (itemDocsEmitter); revisions.md.
- **Found**: the examples suite (runTestExamples.py, not run for #565) stops in three examples at the drift check of
  TrapezoidalIndex2 - #3009, RG4.26, for decision; rendererNOGLFWexample.py fails as in 1.13.6 (an image file).

<a id="rg12-5"></a>
### RG12.3.9, RG4.26, RG12.5, RG12.1 — numpy for the fixed sizes, the examples at the drift check, the symbolic tests and status (2026-10-16, #2981, #2985, #3009, #3010)
*(Maintainer 2026-10-16: vectors of size 3 and 6 staying lists was a misunderstanding - if there are no major
boundaries, they become numpy arrays too. The two tutorialRigidBody examples use the generalized-alpha solver, as the
drift is large and users would be misled; the ALE example gets a smaller step size, possibly a smaller tEnd, to match
the limit and keep the duration small; rendererNOGLFWexample.py - check and fix. Then RG12.5 and RG12.1.)*
- **#3010**: built first as asked - `std::array<Real, n>` and `std::array<std::array<Real, n>, m>` as read-only numpy
  views - and measured with a CartesianSpringDamper user function (five 3D arguments), microseconds per call:

  | user function | numpy arrays | lists |
  |---|---:|---:|
  | returns a constant (the conversion only) | 0.58 | 0.53 |
  | written with components, `k[0]*u[0] + ...` | 1.40 | 0.66 |
  | written with numpy operations, `k*u + d*v` | 1.72 | - |

  The conversion costs the same since the views are made with the numpy C interface (`EPyUtils::NumpyView`,
  `PyArray_NewFromDescr` without a base and without the writeable flag; with `py::array_t` it was 1.0); but Python
  computes with numpy scalars and with arrays of three items two to three times slower than with floats and lists, and
  the existing user functions of fixed-size vectors are written with components. That is a major boundary in the sense
  of the request, so the fixed sizes stay lists and the decision is the maintainer's; switching is two branches in
  `PythonUserFunctionArgument`. Kept from it: the C interface views (the run-time sized vectors gain about 10 %:
  generic6 2.06 instead of 2.26 us), a fixed-size result from a numpy array in one pass, and
  fourBarKinematicTreeLoadDep.py and fourBarKinematicTreeUF.py, which wrote `load[2] = ...` into their argument and
  return a new list now. The use of `userFunctionListArguments` is reported once per session (it was reported at every
  argument, 4 us per call).
- **#3009**: tutorialRigidBody(Create).ipynb solve with `GeneralizedAlpha`; the Create tutorial says what index 2 does
  and that the solver stops at its drift for this model; outputs stored with `exudev notebooks`. The ALE example: the
  drift of its moving-mass sliding joints (JointALEMoving2D) under index 2 is proportional to the step size - 1.8e-3
  over the full run at h = 4e-4, 9.3e-4 at 1e-4, 4.7e-4 at 5e-5 (measured with tEnd = 2), and largest in the first
  second - so 1e-5 would need h of about 1e-6, 2e7 steps; a shorter tEnd does not help either. The example keeps the
  step size and solver of its article and sets `constraintDriftTolerance = 5e-3` with the reason. rendererNOGLFWexample.py
  wrote `images/mergedImageBallContact.jpg` into a directory nothing creates (failed in 1.13.6 too); it writes into
  `OutputFilePath(...)` and creates the directory; removed from `KnownExampleFailures`.
- **RG12.5** (#2985): symbolicUserFunctionSignaturesTest.py generates, for each of the 30 symbolic signatures of
  `userFunctionArgsDict`, a function that uses every argument, computes it with `exudyn.cMath` and records it with
  `exudyn.symbolic`, and compares `UserFunction.Evaluate` with Python: equal to the last bit. Four signatures failed
  before: `postNewtonFunction` (a StdVector2D result: `ArgTypeIsVector` lacked it), `constraintUserFunction` of
  CoordinateVector (a `bool` argument), `outputVariableUserFunction` of ObjectUserBody (an `OutputVariableType`
  argument) - both recorded as `esym.Real` now -, and `springForceTorqueUserFunction` evaluated from Python (matrix
  arguments: `PyEvaluateUF` dispatches on what was recorded). Part (2) solves seven items with Python and symbolic
  versions of one function: CartesianSpringDamper, CoordinateSpringDamper, TorsionalSpringDamper, ObjectGenericODE1,
  LoadTorqueVector, LoadMassProportional, the offset of a CoordinateConstraint - equal.
- **RG12.1** (#2981): docs/dev/design/symbolicStatus.md. Found while writing it: an item set with `SetObjectParameter`
  holds its symbolic function, so the "keep it alive" note of CreateSymbolicUserFunction is stricter than needed.
- The examples suite after the changes: only NGsolveOCCboundaries2.py failed, inside NGsolve (a local heap of 3.2 GB in
  the parallel run); alone it runs.

<a id="rg12-3-10"></a>
### RG12.3.10 — fixed-size vectors and matrices as numpy arrays, dot and @ for symbolic (2026-10-17, #3011)
*(Maintainer 2026-10-17: no problem if the access to single elements is slower - with numpy, most problems would be
solved in vectorized form; really do the switch, and adapt the scripts that can now take the vector form. With
symbolic, a vectorized form should work as well; the multiplication operators of exudyn.symbolic have a different
meaning, so introduce a .dot operator to enable equal scripts for Python and symbolic, or switch to @ in symbolic later.)*
- `PythonUserFunctionArgument` gives `std::array<Real, n>` and `std::array<std::array<Real, n>, m>` (row-wise,
  static_assert) as read-only numpy views again, through `EPyUtils::ToNumpyView(data, rows, columns)`.
- **Scripts in vectorized form**: ROSTurtle.py `(v - offset)*d`; chatGPTupdate2.py, createFunctionsTest.py,
  taskmanagerTest.py, mainSystemExtensionsTests.py and PARTS_ATEs_moving.py without `np.array(loadVector)` or with
  `f*loadVector`; pythonUserFunctionArgumentsTest.py `k*(u - offset) + d*v`, `stiffness @ np.hstack(...)`;
  perfUserFunctions.py has vectorized Python versions of the Cartesian spring-damper and the load (`np.asarray`, so
  that they also run with the lists of 1.13.6 for the comparison). The results of the test models are unchanged. Left
  with components: the functions that are recorded symbolically too (tutorialSymbolic, cartesianSpringDamperUserFunction,
  the symbolic tests), where `k*u` would be the scalar product.
- **Symbolic**: `esym.Vector.dot`, `Vector @ Vector`, `Vector @ Matrix`, `Matrix.dot`, `Matrix @ Vector`, `Matrix @
  Matrix`, `numpyArray @ esym.Vector/Matrix` (`__rmatmul__`, a 2D array as constant matrix); `__array_ufunc__ = None` on
  Real, Vector and Matrix, so numpy returns NotImplemented and Python calls the symbolic operator - checked:
  `K @ q` records `([[2,0,0],...] * q)`, `np.float64(2.)*x` gives `(2 * x)`, which numpy before turned into a number
  through `__float__`. `*` of two symbolic vectors stays the scalar product: changing it to the product of the items
  would silently change existing symbolic functions; with `dot`/`@` the scalar product is written alike for both, and
  the question of `*` is open.
- **Measured** (userFunctionPerformance.py, the same script for both versions): cartesian Python 2.86 against 3.67 us in
  1.13.6, load 1.21 against 1.37, generic6 2.05 against 2.96, generic100 2.20 against 10.46; symbolic unchanged.

<a id="rg7-10"></a>
### RG7.10 — the items that depend on user functions drawn in the main thread (2026-10-17, #3003)
*(Maintainer 2026-10-16: no visualization cache in ObjectUserBody, it would not work with the SolutionViewer; items that
depend on Python user items cannot be drawn in the visualization thread: a list computed in Assemble that marks them,
in a separate step; a global flag that detects a Python (not symbolic) user function called in the visualization
thread. 2026-10-17: do RG7.10 and RG7.3.4.)*
- **The lists** (VisualizationSystem::UpdateMainThreadItems): computed in the render update when the system has changed
  (`systemHasChanged`, which Assemble and every change of an item set) - the same moment as an Assemble, without a
  second place that must know about the visualization. An object with `AccessFunctionsCallUserFunctions()` (a new
  virtual of CObject, true for ObjectUserBody), the markers on such an object (`Marker::Body` or `Object`), the
  connectors with such a marker, the loads on such a marker and every load with a user function, the sensors on such an
  object, marker or load. Symbolic user functions are included: one is evaluated by one thread at a time, and the
  computation thread evaluates it during the step.
- **The drawing**: `UpdateGraphicsData` (render thread) requests the main thread as for the graphics user functions when
  a list has an item, and skips the listed items; `PostProcessData::ProcessUserFunctionDrawing` (main thread) calls
  `UpdateMainThreadGraphics` after the graphics user functions, with the show flags of each kind; the marker frame is
  drawn by `DrawMarker` in both. Single-threaded rendering and `GetGraphicsData` go the same way in one thread.
- **The detection**: `SetExudynMainThread` at the import of the module records the thread; `IsExudynMainThread` is
  asked by every Python user function (`PythonUserFunctionToStdFunction`) and by ObjectUserBody before it evaluates a
  user function; a call from another thread sets an atomic flag, which ProcessUserFunctionDrawing reports once as a
  warning. ObjectUserBody returns zero or the unit matrix in another thread, as before with an empty cache.
- **Removed**: `VisualizationAccessCache` from ObjectUserBody (member, `UpdateVisualizationCache`); the header
  `src/System/VisualizationAccessCache.h` is unused now - to be deleted after approval (a tracked file).
- **Loads**: `loads.drawWithUserFunction` evaluates the user function wherever the load is drawn in the main thread -
  a Python user function is drawn in multithreaded rendering too (before: only symbolic, or single-threaded).
- **Found and fixed**: the MainSystem argument of a Python user function was passed as a const reference, which pybind11
  copies - and MainSystem cannot be copied - when its Python object no longer exists (a model built in a function that
  returned only the SystemContainer): "return_value_policy = copy, but type is non-copyable". It is passed by
  reference now (`PythonUserFunctionArgument`).
- **Test**: test_graphicsData.py testTheItemsOnAUserBodyAreDrawnOnceAndWhereTheyAre - a marker frame, a Cartesian
  spring-damper, a load with a user function and a sensor on an ObjectUserBody give the same drawing elements, per
  item and at the same points, as on an ObjectRigidBody. The thread itself cannot be tested without a window.

<a id="rg7-3-4"></a>
### RG7.3.4 — ObjectUserBody and ObjectGenericODE2 with the conventions of Exudyn (2026-10-17, #2995)
*(Maintainer 2026-10-15: static functions, EvaluateMatrix with an integer switch, StdVector used internally;
2026-10-17: do RG7.10 and RG7.3.4.)* The points of the review:
- (1) the free static functions are gone: `ConfigurationTime` is `CSystemData::GetConfigurationTime`, `CurrentMatrix`
  is the body of `GetPositionJacobian`/`GetRotationJacobian`, `MatrixTimesVelocity` the member `JacobianTimesVelocity`.
- (2) `EvaluateMatrix(which)` is `EvaluatePositionJacobian`, `EvaluateRotationMatrix` (into a Matrix3D),
  `EvaluateRotationJacobian`, over one member template `EvaluateMatrixUserFunction(function, name, columns, ...)`.
- (3) no StdVector inside: `CObjectBody::GatherCoordinates(q, q_t, configuration, addReference)` gathers into local
  `ResizableConstVector`s - no allocation up to 7 coordinates, and no member buffer that a nested call (a user function
  that asks for an output of the same body) could overwrite while Python holds a view of it; the user functions get
  numpy views of them through the new `EvaluateUserFunction(userFunction, mbs, args...)` (SymbolicUtilities.h), which
  calls a Python function directly and a symbolic one through its std::function, and `EvaluateUserFunctionIntoVector`
  for the force and the output variables. `PythonUserFunctionArgument` views a SlimVector (the local position) too;
  `processArgument` copies one into a symbolic named vector.
- (4) `GetODE2Size` of ObjectUserBody keeps its loop over the nodes: a body has few nodes, and a value kept from
  Assemble would be stale for the checks before the next Assemble.
- (5) `CObject::GetObjectNumber`/`SetObjectNumber`, set for every object at the start of `CSystem::Assemble`; the search
  of ObjectUserBody and its mutable member are gone.
- (6) the derivative of J^T f uses the rule of `newton.numericalDifferentiation` with its defaults (relativeEpsilon 1e-7,
  minimumCoordinateSize 1e-2) - the settings of the solver do not reach a marker; userRigidBodyTest.py changes by 1e-14
  relative (13.475755021376248, before ...353), from the Newton iterations.
- (7) the output Coordinates gathers once, without the reference; `GetDisplacement` keeps two calls of the position user
  function, as the reference position depends on the local position.
- (8) `CheckResultSize` in ObjectUserBody and the same message in ObjectGenericODE2: "ObjectGenericODE2:
  forceUserFunction returned a vector of size 2, but 4 is needed" - a `PyError` (ValueError) in both;
  pythonUserFunctionArgumentsTest.py checks it.
- (9) the commented-out code of ObjectGenericODE2 removed.
- (10) `ComputeJacobianODE2_ODE2`: the user Jacobian, or the stiffness matrix, or the damping matrix as the first matrix
  (`CopyOrAddTripletsWithFactor`), the others added by `AddMatrixWithFactor`, which checks that the format is the one of
  the Jacobian - before, the stiffness matrix was checked also without a jacobianUserFunction and the damping matrix
  not at all.
- (11), (12) `GetOutputVariableBody` of ObjectGenericODE2 computes into local vectors, only what the variable needs.
- (13) waits for RG13 (total coordinates).
- (14) both fetch the MainSystem from their system data; the GenericODE2 evaluations no longer take it as argument.
- (15) `GatherCoordinates` serves ObjectUserBody and ObjectGenericODE2 (`ComputeObjectCoordinates` is a call of it);
  ObjectKinematicTree, ObjectFFRF and ObjectGenericODE1 keep their own, as their coordinates are not those of the nodes
  alone (joint coordinates, the reference frame, ODE1 nodes).
- (16), (17) done before (#2999, #3003).
- Measured: generic6 and generic100 of perfUserFunctions as after RG12.3.10 (Python 2.04/2.12 us, symbolic 0.058/0.25);
  ObjectUserBody did not exist in 1.13.6.

<a id="rg9-8"></a>
### RG9.8 — the object numbers owned by CSystemData (2026-10-17, #3012)
*(Maintainer 2026-10-17: set the number when the item is added, as functions of the reference configuration work
before Assemble; only DeleteObject changes numbers; integrate it into CSystemData so that no number can be wrong.)*
- Checked: the list `CSystemData::cObjects` was changed in two places only - `Append` in MainObjectFactory (AddObject)
  and `Remove` in `MainSystem::PyDeleteObject`; `Reset` empties it. Nothing sorts or moves objects.
- `CSystemData::AddCObject(object)` appends and sets the number, `DeleteCObject(number)` deletes, removes and renumbers
  the objects after it; the non-const `GetCObjects()` is gone, so no other code can change the list (the build needed
  no other change); `CObject::SetObjectNumber` and the number are private, CSystemData is a friend. The renumbering of
  #2995 in `CSystem::Assemble` is gone.
- The reference configuration works before Assemble already (`RaiseIfNotConsistentNorReference`); with the number set
  at AddObject, a user function of ObjectUserBody called from `GetObjectOutputBody(..., Reference)` before Assemble
  gets its object number, not -1.
- Found and fixed in DeleteObject: a marker on several bodies (MarkerBodiesRelativeTranslationCoordinate/
  RotationCoordinate) was renumbered by comparing its first body for each of them; a deleted object between its two
  bodies left the second one unchanged.
- deleteItemsTest.py checks the number before Assemble, after a DeleteObject, and both bodies of a relative marker
  around a deleted object; its result is unchanged.

<a id="rg9-9"></a>
### RG9.9 — the own number out of the arguments of the item functions (2026-10-17, #3013)
*(Maintainer 2026-10-17: remove the object number from the arguments where it is not needed any more; evaluate first,
propose sub-steps, resolve what needs no decision.)*

**Evaluation (RG9.9.1).** Since #3012 every object has its number from AddObject on; the argument repeated it:

| where | functions | argument | done |
|---|---|---|---|
| CObject, CObjectBody, every object | ComputeODE2LHS, ComputeODE1RHS, ComputeJacobianODE2_ODE2, ComputeMassMatrix, GetOutputVariable, GetOutputVariableBody, ComputeKineticEnergyFromMassMatrix, the item helpers (ComputeODE2LHStemplate, ComputeMassMatrixAndODE2LHS, AddJointForces, EvaluateUserFunctionForce/MassMatrix/Jacobian/RHS, ComputeAxialForce/BendingMomentLocalUserFunction) | objectNumber, itemIndex | RG9.9.2/.3 |
| CObjectConnector, every connector and constraint | ComputeODE2LHS/ODE1RHS/ComputeJacobianODE2_ODE2 with MarkerDataStructure, ComputeJacobianForce6D, ComputeJacobianODE2_ODE2generic, GetOutputVariableConnector, GetOutputVariableConstraintEquation, PostNewtonStep, ComputeAlgebraicEquations, ComputeJacobianAE, the connector interface (`...Connector`, the L2 chains `Connector...Markers`, `Constraint...Markers`, `SeedConstraintEquations...`, GetConnectorOutputVariable), ComputeConnectorForce*/ComputeConstraintEquations*(Diff), ComputeSpringForce*, ComputeSpringTorque, ComputeConnectorProperties, ComputeReevingGeometry, EvaluateUserFunctionForce/Offset/Offset_t/Constraint/Jacobian/PostNewtonStep | objectNumber, itemIndex | RG9.9.2/.3 |
| CSystem | ComputeObjectODE2LHS, ComputeObjectODE1RHS (take the object pointer and its number) | objectNumber | RG9.9.2 |
| MainObject | GetOutputVariable, GetOutputVariableConnector, GetOutputVariableBody | objectNumber | RG9.9.2 |
| CSensor, the graphics helpers | the calls of the object outputs | parameters.objectNumber, itemNumber | RG9.9.2 |
| visualization of all kinds of items | UpdateGraphics, CallUserFunction, DrawMarker, GetMarkerPositionOrientation | itemNumber | RG9.9.4, proposed |
| kept | CSystem::ComputeObjectJacobianODE2/AlgebraicEquations/ReactionForces (the number selects the object); MainSystem and MainSystemData (the Python interface); the markers and sensors (the number of the object they refer to); the arguments of the Python user functions (the user's interface) | | |

**RG9.9.2/.3.** A script removed the parameter from the signatures of the listed functions, in `src/` and in the
definitions, and the argument at its position from their calls, where it was an item number; the compiler named the
uses left in the bodies, which became `GetObjectNumber()` (`connector.GetObjectNumber()` in the free functions of
the connector interface). The definitions had two identical entries of the library function ComputeAlgebraicEquations
of constraints (they differed by a space before the argument); one is gone. Every call passed the own number (the loop
index of the objects, the number of the MainObject, the object of a sensor), so nothing computes differently; the test
suite passes unchanged.

<a id="rg7-10-1"></a>
### RG7.10.1 — VisualizationAccessCache.h deleted (2026-10-17, #3014)
*(Maintainer 2026-10-17: yes, delete it.)* `src/System/VisualizationAccessCache.h` and its entries in
`msvc/cppsrc.vcxproj(.filters)` are gone; docs/dev/design/pythonUserItems.md 5.2 said the renderer reads the cache and
the item number is searched - it now says the body is drawn in the main thread (#3003) and the number is the object's
own (#3012).

<a id="rg12-3-11"></a>
### RG12.3.11 — the operators of exudyn.symbolic are those of numpy (2026-10-17, #3015)
*(Maintainer 2026-10-17: deactivate `*` between two symbolic vectors, the scalar product via `@`; add numpy's multiply
so that user functions are cross-compatible; `*` raises and names dot and multiply; an issue for `*` as in numpy in two
to three years; a deprecation flag for the scalar product, off by default; check the compatibility of numpy and symbolic
in general - no discrepancies, a smaller set of features; important missing features as issue and step.)*
- **The comparison** (scratchpad script, then symbolicNumpyCompatibilityTest.py): 60 expressions with numpy arrays and
  symbolic operands. Before: 5 gave a different result (`a * b`, `M * M`, `M * a`, `a * M`, `a == b`), 2 lost the
  expression silently (`math.sin(y)`, `float(y)` - and `np.array([s, y])` looked lost but is an array of `esym.Real`),
  about 30 were refused, among them every numpy ufunc (`np.multiply`, `np.sin`, a numpy array plus or times a symbolic
  vector), `a / s`, `2**y` (the third `__pow__` overload was bound as `__pow__` instead of `__rpow__`), `abs(y)`,
  `a[-1]`, `sum(a)` and `np.dot(a, b)` (the index raised InternalError, so the vector was no sequence).
- **Refused now** with a TypeError that names the operation to use: `*` of two vectors, of two matrices, of a matrix
  and a vector (`SymbolicStarProduct` in Symbolic.cpp), `==` and `!=` of two vectors. `exudyn.special.deprecations.
  symbolicStarProduct = True` restores the scalar and matrix products of `*`, with a DeprecationWarning once.
- **Added**: `Vector.multiply` (also with a list or numpy array); `__array_ufunc__` implemented instead of `None`
  (`SymbolicArrayUfunc`): numpy arrays and numpy scalars become symbolic vectors, matrices and floats (an object array
  keeps its `esym.Real` items), then `add`, `subtract`, `multiply` (of two vectors: `multiply`), `divide`, `matmul`,
  `power`, `negative`, `positive`, `absolute`, the math functions of `esym` under numpy's names (`arcsin` is `asin`, ...),
  `arctan2`, `maximum`, `minimum`; anything else is NotImplemented, and numpy raises a TypeError. `Vector / scalar`,
  `2**x`, `abs(x)`; `v[i]` raises IndexError outside and counts negative indices from the end; `__iter__`.
- **Run-time size**: a recorded `StdVector` argument had size 0, so `len(q)` was 0 and numpy, `sum()` and a loop saw an
  empty vector; `q * npArray` failed. CreateSymbolicUserFunction records `q`, `q_t` of ObjectGenericODE2, ObjectGenericODE1,
  ObjectUserBody, ObjectKinematicTree, ObjectFFRF and ObjectFFRFreducedOrder with the number of coordinates of their
  nodes (`_ObjectCoordinatesSize`, from `GetNodeOutput(..., Coordinates, Reference)`, which works before Assemble);
  where the size stays unknown, `len()` and `iter()` raise a TypeError instead of seeing 0. A vector result that is a
  numpy array of `esym.Real` (`np.array([fx, fy])`, `np.cross`) is taken as `esym.Vector`.
- The expression strings print the products as `@` and the product of the components as `.multiply(...)`.
- **Changed**: symbolicModuleTest.py (`v@w`, `M@N`, `M@v`, `v@M`, `np.multiply` in both cases; the comparisons of
  vectors by components with `AllEqual`), symbolicVectorUserFunctionTest.py (`q@q_t`), the notebook
  reference/symbolic.ipynb; their results are unchanged. All other test models, the five examples with symbolic user
  functions and perfUserFunctions.py run unchanged.
- **Open as steps**: RG12.6 (#3016, `*` as numpy in 2029), RG12.7 (#3017, `float()` while recording, a decision),
  RG12.8 (#3018, `A.T`, rows, slices, component-wise functions).

<a id="rg12-7"></a>
### RG12.7 — float() of a symbolic expression refused while recording (2026-10-17, #3017)
*(Maintainer 2026-10-17: yes, raise an error.)*
- `esym.Real.__float__` raises a TypeError while recording (`SReal::recordExpressions`) for a Real that holds an
  expression or a named variable; a number converts as before, and so does an expression after
  `esym.SetRecording(False)`. The error is a plain Python TypeError, not PyError, as pybind11 may try a conversion to
  float when it resolves an overload, which must not stop a renderer.
- Refused now: `math.sin(x)`, `float(x)`, `np.array([x], dtype=float)`, and the component of a vector without
  expression set to an expression (`v[0] = x + 1`, `v[0] += x`), which evaluated it before.
- Found by the test suite: shells.py called its vertex mapping with `esym.Real` and evaluated the result with
  `float()`; SurfaceMap.ConeFrustum and Cylinder convert their arguments with `float()` and got constants, so the
  slopes came from differences. Now a mapping that raises the TypeError is evaluated with numbers and the slopes by
  differences - the same values as before; `_Number` evaluates a result that is symbolic. symbolicModuleTest.py sets
  the components of its vector with `Evaluate(...)`. Results unchanged.
- **An operation on numbers alone gives a number, also while recording**: `esym.sign(v[0])*10` in a Python user
  function (cartesianSpringDamperUserFunction.py, the documented way of writing one function for both) built an
  expression of constants, which the conversion to float now refused. `SReal::RecordExpressions(operands...)` - recording
  on and an operand with an expression or a named variable - replaces `recordExpressions` in the 44 operators and
  functions of `esym.Real`; an expression without variables was a constant anyway, so nothing records differently, the
  trees are smaller and the printed expressions show the folded numbers.
- symbolicNumpyCompatibilityTest.py checks the three conversions among the refused operations, and that esym functions
  of numbers give a float.

<a id="rg2-10"></a>
### RG2.10 — the threads of the numerical libraries in the parallel examples run (2026-10-17, #3019)
*(Maintainer 2026-10-17: probably like the earlier problem with scipy, check it; then: add a step and resolve it.)*
- NGsolveOCCboundaries2.py failed in the parallel run of 1.13.104 and 1.13.111 with "Could not allocate localheap,
  heapsize = 3200000000", and passed alone. Measured with psutil (the memory a process reserves from Windows; the
  working set stays near 1.3 GB), on 32 threads, 66 GB RAM and 41 GB pagefile: `import ngsolve` 8 GB (MKL and OpenMP
  reserve per thread), `Assemble` in the TaskManager 3.2 GB more (100 MB per thread), the example 17 GB; `import exudyn`
  5 MB, `import numpy, scipy` 1.6 GB. The examples run in 8 processes in alphabetical order, so the NGsolve examples
  run at the same time and exceed the commit limit. Nothing of Exudyn and no dense matrix is involved.
- `testRunnerTools.ParallelProcessEnvironment(numberOfProcesses)`: a copy of the environment with OMP_NUM_THREADS,
  MKL_NUM_THREADS, OPENBLAS_NUM_THREADS and NGS_NUM_THREADS = cores / processes (4 here), unless set already;
  RunExamplesInParallel gives it to every example process. Only the child processes get it - the runner and the shell
  keep their environment, so there is nothing to set back. With 4 threads the example reserves 6.3 GB and takes the
  same 14 s; the threads of Exudyn's own solvers are set by their settings and do not change.
- The parallel run of the test suite keeps the threads: its results are compared to 5e-14, and the number of BLAS
  threads may change the last bits of a large product.
- pytest test_runnerLogs.py::testTheParallelProcessesShareTheThreadsOfTheLibraries.

<a id="rg12-8"></a>
### RG12.8 — the transposed matrix, rows, slices and component-wise functions in exudyn.symbolic (2026-10-17, #3018)
*(Maintainer 2026-10-17: do RG12.8.)*
- `MatrixExpressionTransposed` (SymbolicMatrix.h) and `SymbolicRealMatrix::Transposed`; `A.T` and `A.transpose()`;
  a row `A[i]` (also `A[-1]`) is a vector of the components `A[i, j]`.
- `v[a:b]` (any Python slice) is a vector of the components; `v + s`, `s + v`, `v - s`, `s - v` with a float or an
  `esym.Real` add to each component; `v + list`, `list + v`, `v - list`, `list - v`.
- numpy's functions of one argument applied to a vector apply to each component (`np.sin(v)`, `np.abs(v)`, ...), in
  `SymbolicArrayUfunc` through `SymbolicVectorMap`; the functions of matrices, `npMatrix * v` and slices of matrices are
  refused, as not needed so far.
- These need the size of the vector while recording; a vector argument without size raises the TypeError of #3015.
- symbolicNumpyCompatibilityTest.py: seven more expressions agree with numpy.

<a id="rg12-3-7-2"></a>
### RG12.3.7.2 — symbolic user functions with matrix results, dense (2026-10-17, #3020)
*(Maintainer 2026-10-17: plan the sub-steps first; a sparse path with the pattern fixed when the tree is built, dense
first is acceptable in a format that passes into the sparse mode, as MatrixContainer does.)* The evaluation is the
table of RG12.3.7 in the plan.
- **Generator**: `definitionTypes.userFunctionMatrixResults` marks the user function defs annotated `-> MatrixContainer`
  or `-> NumpyMatrix`; `FitsSymbolicUF` lets them be symbolic, so that PySymbolicUserFunctionSet.h has a branch for the
  nine functions (`EvaluatePyObject`, `EvaluateNumpyMatrix`).
- **SymbolicUtilities.h**: a recorded `esym.Matrix` result (`SetMatrixType`; the return type py::object of a
  MatrixContainer result or NumpyMatrix); `EvaluateIntoMatrix(Matrix&)` and `EvaluateIntoMatrixContainer(container,
  ltg)` - dense now, the ltg list is the parameter for the sparse results of RG12.3.7.4; `PyEvaluateUF` returns a numpy
  array for a matrix. Fixed on the way: `EvaluateNumpyMatrix` cast a ResizableMatrix to `py::array_t` by its
  constructor, which gave an array of a wrong shape - it uses `EPyUtils::ToPython` (only reached now, as no matrix result
  could be symbolic); `MatrixExpressionOperatorMultMatrixMatrix::EvaluateComponent` compared the inner size with a bool
  and refused every component of a matrix product, `(A @ B)[i, j]`.
- **The items** (evaluateUserFunctions.cpp): `EvaluateUserFunctionIntoMatrixContainer` - a symbolic function directly,
  a Python one through `PyMatrixContainer` and `CopyOrAddTriplets` as before - for ObjectGenericODE2 (mass, Jacobian),
  ObjectUserBody (mass) and ObjectConnectorCoordinateVector (Jacobian); `EvaluateUserFunctionIntoMatrix` for the
  Jacobians and the rotation of ObjectUserBody; ObjectFFRF and ObjectFFRFreducedOrder evaluate a symbolic mass matrix
  directly.
- **CreateSymbolicUserFunction** takes a list of lists or a 2D numpy array of symbolic items as matrix result.
- **Tests**: userRigidBodyTest.py - the user body with all six user functions symbolic, the matrices with `@` and
  `.T`, agrees with the Python body to 7.8e-14 (result unchanged); symbolicUserFunctionSignaturesTest.py evaluates the
  nine signatures with matrix results (39 signatures) and solves an ObjectGenericODE2 with mass matrix and Jacobian
  functions of the coordinates in both modes (identical results).
- **Measured** (ObjectGenericODE2, 6 coordinates, mass matrix and force user functions, 20000 steps of
  ExplicitMidpoint): Python 0.468 s, symbolic 0.051 s, the same result; the matrix nodes still allocate (RG12.3.7.3).

<a id="rg11-8-22"></a>
### RG11.8.22 — the issue store of the tests (2026-10-17, #3023)
*(Maintainer 2026-10-17: the test should not create thousands of files.)*
- The fixture `tracker` of test_issueTracker.py copied the whole store (128 open, 1167 closed files, 6 archives) for
  each of its about 60 tests; under pytest -n 8 the setup failed once with WinError 1450. A session fixture
  `storeTemplate` builds the store of the tests once per process: the open issues, the archives, the 50 newest closed
  issues in closed/ (a test that changes a closed issue takes a recent one, which lies there as in the real store),
  the older closed issues written into the archives of their years - the same issues and the same closed count, in
  about 190 files; each test copies that.
- testTheStoreIsConsistent checks the real store read-only and the store of the tests, and that both have the same
  closed count. test_exudev.py's html test reads the real store instead of a copy, as it writes no issue.

<a id="rg10-32"></a>
### RG10.32 — deferred issues (2026-10-17, #3024)
*(Maintainer 2026-10-17: RG12.6 is a delayed issue; the step can be closed; a field for it as proposed.)*
- issueStore: the field `deferredUntil` (YYYY-MM-DD); CheckStore refuses another form and the field on a closed issue.
- issueTracker: `IsDeferred(issue)` - open and the date not reached - and `DeferIssue(number, date, reason)`, which
  sets the field and the deadline and writes the reason into the working remarks; `exudev issue defer`.
- `exudev issue list --open` and `triage` leave deferred issues out and name them apart; planManager prints each
  deferred issue with the steps that cite it (`deferred: #3016 until 2029-01-01 (RG12.6)`).
- #3016 is deferred until 2029-01-01. pytest: three tests in test_issueTracker.py.

<a id="rg12-3-7-3"></a>
### RG12.3.7.3 to .5 — symbolic matrix nodes without allocation, sparse matrix results, one copy of a numpy result (2026-10-17, #3021, #3022, #1864)
*(Maintainer 2026-10-17: do the next steps as suggested.)*
- **#3021**: `MatrixExpressionBase` has `EvaluateRef()` into a `mutable ResizableMatrix buffer` per node and
  `Evaluate()` copies from it, as the vector nodes since #3006; every matrix node (the constant and named matrices,
  the matrix of components, +, -, scalar times matrix, unary minus and plus, the product, the transposed) evaluates
  into its buffer, the products of matrix and vector read the matrix by reference, and the symbolic function evaluates
  its matrix result by reference into the item. Measured with `exu.special.InfoStat`: an ObjectGenericODE2 with a
  mass matrix `G.T @ J @ G + 0.1*G - 0.1*G` and a force with `np.multiply` and `@` adds the same counts for 100 and
  1000 steps (pytest test_symbolicAllocations.py).
- **#3022**: `MatrixExpressionSReal::StructuralNonZeros` - the components that are not a constant ExpressionReal
  with value 0 (a named variable is never a structural zero); the symbolic function keeps this pattern when the result
  is recorded (`SetMatrixType`), and `EvaluateIntoMatrixContainer` appends the triplets of the pattern with the ltg
  indices, as `CopyOrAddTriplets` does with a sparse Python result; a matrix without constant zeros, a matrix of
  operations and an item without ltg list (the Jacobian of ObjectConnectorCoordinateVector) stay dense. Both system
  matrices take triplets (the dense one adds them). No separate `esym.SparseMatrix` type: the components are the
  natural way to write a sparse matrix, and the pattern follows from them. symbolicSparseMatrixTest.py: a chain of
  30 masses on ObjectGenericODE2, with mass matrix and Jacobian of components, Python (dense numpy) and symbolic
  (sparse), EigenSparse and EXUdense - the four solutions agree to 7e-17. With 300 masses: 0.26 s symbolic, 3.6 s
  with the Python functions, which build the dense lists.
- **#1864**: a numpy matrix returned by a Python function is copied once into the dense matrix of the item's
  MatrixContainer (`EPyUtils::FromPython`, which keeps the memory when the size fits) instead of into a
  PyMatrixContainer and from there into the item; other results (MatrixContainer, scipy sparse, lists) take the
  PyMatrixContainer path as before. ObjectGenericODE2 with a 6 x 6 mass matrix function: 0.444 s against 0.502 s of
  1.13.6 (whole solution); the table is in userFunctionBoundary.md section 12. The original idea of #1864, linking the
  numpy memory, is not needed for this.
- The evaluation of RG12.3.7 (2026-10-17), as it stood in the plan:

The matrices at the boundary (evaluation 2026-10-17, maintainer: a sparse path for GenericODE2 and the user body, with
the sparse pattern fixed when the tree is built; dense first is fine, in the format that passes into the sparse mode,
as MatrixContainer does):

| item, user function | matrix | Python now | symbolic now |
|---|---|---|---|
| ObjectGenericODE2 massMatrixUserFunction, jacobianUserFunction | result n x n, MatrixContainer | numpy, list of lists, scipy sparse or MatrixContainer, one copy into the item's MatrixContainer, added with `CopyOrAddTriplets` and the ltg list | not possible (py::object result) |
| ObjectUserBody massMatrixUserFunction | result n x n, MatrixContainer | the same | not possible |
| ObjectUserBody positionJacobianUserFunction, rotationJacobianUserFunction, rotationMatrixUserFunction | result 3 x n, 3 x 3, dense | numpy, copied into a Matrix (`NumPy2Matrix`) | not possible |
| ObjectFFRF, ObjectFFRFreducedOrder massMatrixUserFunction | result n x n, dense | numpy, copied | not possible (NumpyMatrix result) |
| ObjectConnectorCoordinateVector jacobianUserFunction | result, MatrixContainer, dense only | numpy, copied | not possible |
| ObjectConnectorRigidBodySpringDamper springForceTorqueUserFunction, postNewtonStepUserFunction | arguments stiffness, damping 6 x 6, rotJ0, rotJ1 3 x 3 | read-only numpy views (#3011) | named matrices (#2985) |

The arguments are done; what is left are the results, and the expressions of matrices (RG12.8 for `A.T`).

<a id="rg7-3-5"></a>
### RG7.3.5 — userRigidBodyTest.py in a loop over the three kinds of body (2026-10-17, #3025)
*(Maintainer 2026-10-17: the three cases computed one after the other, with show.computationTime and verboseMode on.)*
- `Solve(kind)` builds the ground, the body and its loads in a system of their own for 'standard' (ObjectRigidBody),
  'python' and 'symbolic' (ObjectUserBody) and solves it with `show.computationTime` and `verboseMode = 1`; the
  comparison of the outputs is unchanged, and so is the result (13.475755021376248).
- 500 implicit steps on this machine: ObjectRigidBody 0.005 s, Python user functions 0.26 s (97 % in the user
  functions), symbolic user functions 0.11 s (95 %). The symbolic body is still 20 times the standard one: per Newton
  iteration the Cartesian spring-damper evaluates position, rotation and both Jacobians of the body, and the
  derivative of J^T f evaluates the Jacobians 4 n times by differences (#2995) - a place for analytic derivatives
  later.

<a id="rg6-12-1"></a>
### RG6.12.1 — the ruler (2026-10-18, #2244)
*(Maintainer 2026-10-18: the ruler makes sense in rotated views as well, but only with perspective 0; key F5 or F6,
as F4 is often used for other actions.)*
- `AddRuler` and `RulerTickSpacing` in Raytracing.cpp, called from `GetGraphicsDataStatic`, so OpenGL and the raytracer
  draw it: a line at the bottom and the left border, minor ticks of 5 pixels every fifth of the major distance, major
  ticks of 10 pixels (times the display scaling) with labels in the small font; the major distance is 1, 2 or 5 times a
  power of 10 and at least 80 pixels; no label within 2.5 font sizes of the corner, where the two scales meet.
- In an axis-aligned view without tracked marker the labels are the coordinates of the view
  (`PixelsToOpenGLCoordinates2D`, as the mouse coordinates of F3), and with F3 on the mouse position is marked green
  on both scales; in a rotated view the labels are the distance from the left or bottom end. Nothing with perspective
  > 0.
- `window.showRuler` (default False), key F5 (GlfwClient.cpp), the key table exudyn.misc.keyBindings and from it the
  help dialog and GUI.md.
- pytest testTheRulerHasTicksAtNiceDistances (test_graphicsRegression.py): raytraced 400 x 300 images at three zooms
  over five orders of magnitude, 10 to 25 ticks across the width, and the ruler in a rotated view.
- parameterConversionTestReference.txt records the new setting (written with `recordReference = True`).

<a id="rg7-11"></a>
### RG7.11 — ObjectUserBody reuses the results of its access-function user functions (2026-10-18, #3028)
*(Maintainer 2026-10-18: make the symbolic user body faster - a step and an issue, now.)*
- **Measured first** (userRigidBodyTest.py, 500 implicit steps, the Python user functions counted): 4173 calls of
  forceUserFunction, i.e. evaluations of the right-hand side, and per evaluation 4.8 calls of positionJacobianUserFunction
  and 1.9 of each of positionUserFunction, rotationMatrixUserFunction and rotationJacobianUserFunction - the
  Cartesian spring-damper, the force and the torque marker on the body ask for the same values. The timers put 92 % of
  the time into ODE2RHS and 2 % into the Jacobian. **Correction** of the log of RG7.3.5: the derivative of J^T f by
  differences is not where the time goes; an analytic derivative would not help this model.
- `UserFunctionResultCache<TValue>` (CObjectBody.h): the last two results by time, local position and coordinates
  (compared exactly); `CObjectUserBody` keeps one for the position, the position Jacobian, the rotation matrix and the
  rotation Jacobian (`mutable`, as the body is evaluated serially in the main thread, #3003). A nested call - a user
  function that asks for an output of its own body - stores after it has computed, so it cannot see a half-written
  entry. `ParametersHaveChanged` (a new user function, any parameter) and `PreAssemble` clear the results.
- With the two entries 45 % of the calls remain (simulated in Python first). userRigidBodyTest.py: Python user
  functions 0.26 s to 0.18 s, symbolic 0.106 s to 0.068 s; ObjectRigidBody 0.005 s; the results are bit-identical. The
  test checks that a new positionUserFunction is used and not the stored result.
- The detailed description of ObjectUserBody says that its user functions must be functions of their arguments while
  time and coordinates stay the same.

<a id="rg6-12-2"></a>
### RG6.12.2 — orienting the triangles of imported geometry (2026-10-18, #2281)
*(Maintainer 2026-10-18: do it as described in the concept, in particular as an option of the STL import, as STL files
often have wrong orientations.)*
- graphics.py: `_TriangleOrientation` merges the points by position (rounded to 1e-9 of the bounding box, or a given
  tolerance), forms the half edges in numpy and pairs them with `np.unique`; `_OrientTrianglesAnalysis` flood-fills each
  connected component, flipping a neighbour that runs the shared edge the same way, turns a closed component with
  negative signed volume, and keeps the majority orientation of an open one. Degenerate triangles (two corners at one
  point, as at the poles of a sphere) have no orientation and are left out of the topology - before, the poles showed
  as non-manifold edges. `triangles6` are oriented by their corners (the mid-side nodes swap as in InvertTriangles).
- `OrientTriangles(graphicsData, outward=True, normals='recompute'|'remove'|'keep', tolerance=None,
  returnReport=False)`; the report: triangles, flipped, components, closedComponents, invertedComponents,
  inconsistentEdges, openEdges, nonManifoldEdges, degenerateTriangles. `CheckTriangleOrientation` returns the report of
  what OrientTriangles would do, with `consistent`.
- `orientTriangles=False` in FromSTLfile, FromSTLfileASCII and FromPointsAndTrigs (normals then recomputed, or left to
  the renderer where none were given). Not when graphics is added to a system, as decided in the concept.
- Measured: 65 536 triangles (a sphere) 0.28 s for the check.
- Checked with it: every primitive of exudyn.graphics in the list of test_graphicsRegression.py is consistent and
  outward (Basis and Frame report non-manifold edges where their arrows meet, InvertTriangles is inward by intention).
- orientTrianglesTest.py: a brick, a cylinder and a sphere with 40 % of their triangles flipped are oriented back to
  the original; two bricks, one inverted, end outward; an open checkerboard keeps its majority; three triangles on an
  edge are reported; an STL file with flipped triangles is repaired by FromSTLfileASCII(orientTriangles=True), with
  outward normals; seven primitives are consistent.

<a id="rg6-11-1"></a>
### RG6.11.1 — the textures, decided (2026-10-18, #2155, #2154)
*(Maintainer 2026-10-18: SC.renderer.AddTexture with the interpretation as a new enumeration type, for completion and
Python/C++ consistency; flags following what is usual in OpenGL, beside the interpretation, so that GraphicsData may
override them in the future, not now; a way for 6-node triangles; graphics.Quad and possibly graphics.Cylinder with a
texture, or general geometry without giving the coordinates; a flag in the OpenGL settings to switch textures off; the
rest and the example, preferably with graphics.Quad, good to go; new specific things as sub-steps and issues: #3029 to
#3034.)* The proposal as it was decided (maintainer 2026-10-17: textures defined globally, objects refer to them; a unique string name in
Python, a number in C++; RGB, RGBA with a meaning of A, grey; flags for later extensions; GraphicsData adds only the
texture coordinates). Today no scene texture exists; the font is one GL texture per character from the bitmap
compiled into characterBitmap.cpp (GlfwClientExtended.cpp:511), and the raytracer image is shown as a texture.
- **The store**: one `TextureStore` in the VisualizationSystemContainer, shared by all systems and views: entries
  {name, id, width, height, channels 1/3/4, bytes (uint8, row by row), flags, version}. Python:
  `SC.renderer.AddTexture(name, image, interpretation=...)` with a numpy array (H x W, H x W x 3, H x W x 4, uint8 or
  float in [0,1]) returns the id; `RemoveTexture(name)`, `GetTextureNames()`; adding a name again replaces the image
  (its version moves, the renderer uploads it again). Names starting with `exudyn.` are reserved (the icons below).
- **Flags** (a bit field, extensible): filter nearest/linear, wrap repeat/clamp, the meaning of A (opacity, mask with
  a threshold, or unused), how the texture combines with the color of the triangle (replace, or multiply - a grey
  texture times the color gives shading), mipmaps later.
- **GraphicsData**: the `TriangleList` dict gets `'texture': name` and `'textureCoordinates'` (u, v per point); C++
  resolves the name to the id once, when the graphics data is set. The textured triangles go into their own array
  (`GLTexturedTriangle`: the triangle, the texture id, 3 x (u, v)), so that untextured triangles keep their size and
  path; the renderer draws them sorted by texture, one bind per texture.
- **OpenGL**: textures are uploaded in the render thread when their version changed (`glTexImage2D`, 1.x: sizes that
  are not powers of 2 need the extension, which current drivers have; else padded); OpenGL 3.3/4.1 later uses the same
  store with sampler objects. **Raytracer**: samples the bytes with the barycentric u, v it computes already
  (Raytracing.cpp:95), nearest or bilinear by the flag - a raytraced image shows the textures as well.
- **Steps after the decision**: the store and the Python API with a test of names and replacement; the textured
  triangles in OpenGL; in the raytracer with a graphics regression image; an example (an image on a plane, a
  checkerboard by texture instead of triangles).

<a id="rg6-11-2"></a>
### RG6.11.2 to .4 — the texture store, textured triangle lists, OpenGL (2026-10-18, #2154, #2155, #3029, #3030, #3032)
- **Store** (`src/Graphics/TextureStore.h`, the instance in VisualizationSystemContainer.cpp): entries by number with
  name, size, channels, interpretation, flag bits, bytes and a version from one counter; a number is never given to
  another name, a removed entry keeps its name and number, so the graphics data keeps the numbers it resolved; a spin
  lock against the render thread. Different from the proposal: one store for the module rather than a member of the
  VisualizationSystemContainer (same effect: shared by all containers and views), and `AddTexture` returns None - the
  number is C++ only.
- **AddTexture**: `py::array::ensure`, so nested lists work; uint8 copied, any other numbers as floats in [0, 1]
  (else ValueError); 1, 3 or 4 channels converted to those of the interpretation (Grey 1, RGB 3, RGBA/RGBAMask 4: grey
  replicated, A=255 added, A dropped for RGB; colors refused for Grey); the rows stored from the bottom of the image so
  that v=0 is its bottom; empty and `exudyn.` names refused. The default interpretation is RGBA, which takes every image.
- **Enums**: `TextureInterpretation` plain values; `TextureFlags` bits as an enum class without a string function (the
  generated bit string function only compiles for the plain enums in a namespace); pybind enums have no `|`, so the
  flags are a list. The stub default of an empty `std::vector<...>()` is shown as `[]` (autoGenerateHelper.py).
- **Data**: `GLTexturedTriangle : GLTriangle` with textureID and three `Float2`, in `glTexturedTriangles` of
  GraphicsData and BodyGraphicsData - untextured triangles keep their 128 bytes. Parsing resolves the name (unknown or
  removed: ValueError, add the texture first); `textureCoordinates` without `texture` and the reverse are refused.
  6-node textured triangles: the four sub-triangles (0,3,5), (3,1,4), (5,4,2), (3,4,5) with the node normals - they
  lose their curvature, as decided. `AppendMovedTriangles` (template) moves both kinds in AddBodyGraphicsData(Colored);
  with contour colors the textured ones are plain triangles. Bounding box, flush, the export of the body graphics (one
  TriangleList per texture) and `GetGraphicsData()['texturedTriangles']` (items, points, normals, colors,
  textureCoordinates, texture names).
- **OpenGL**: `UpdateSceneTextures` after `glfwMakeContextCurrent` in Render - the views share one context group;
  `GL_LUMINANCE`/`GL_RGB`/`GL_RGBA`, unpack alignment 1, `GL_NEAREST`/`GL_LINEAR`, `GL_REPEAT`/`GL_CLAMP_TO_EDGE`;
  deleted when the renderer stops, uploaded again on the next start. `DrawTriangles` is templated on the triangle type;
  the textured kind binds when the texture changes from one triangle to the next (no sorting by texture - with
  depthSorting the order is the depth), `GL_MODULATE` or `GL_REPLACE`, alpha test at 0.5 for RGBAMask; highlight,
  selection and edge mode draw them plain. They cast shadows.
- **Not verified on a screen**: no window may open from here; the OpenGL path compiles and the data paths are tested
  (textureTest.py, 103.0); a raytraced image with textured triangles is produced, plain. The maintainer checks the look
  with the example of RG6.11.7, or with the quad of textureTest.py and `SC.renderer.Start()`.
- Left for later sub-steps: the raytracer (RG6.11.5), the graphics functions keeping `texture` when they merge or move
  triangle lists and giving coordinates (RG6.11.6), the example (RG6.11.7).

<a id="rg6-11-5"></a>
### RG6.11.5, .6, .9 and .8 in part — textures in the raytracer and in exudyn.graphics (2026-10-18, #3031, #3034, #3035, #3036)
*(Maintainer 2026-10-18: the OpenGL test shows the red and black checkerboard texture, the ruler works in OpenGL; do
RG6.11.5 and .6; textures in OpenGameArt_textures_nosync, a test with hardwood_small.jpg; a common loader under
exudyn.graphics; graphics.CheckerBoard by default with a texture generated on the fly, useTexture=True.)*
- **Raytracer** (#3034): the textured triangles go into the triangle list of the raytracer as before; beside it
  `triangleTextures` (per triangle -1 or an index into `texturedTriangles`, allocated only when a textured triangle
  exists, filled with -1 for those before) and `textures`, a copy of each texture used (`TextureStore::GetCopy`), so the
  threads sample without the lock and Python may replace a texture meanwhile. `IntersectRayWithItem` calls
  `ApplyTexture` after a hit: the barycentric u, v of the hit interpolate the coordinates, `Texture::Sample` (in
  TextureStore.h, nearest or bilinear between pixel centers, repeat or clamp - as OpenGL); Replace sets the color,
  else it is multiplied; RGBA multiplies the alpha; RGBAMask below 0.5 clears the hit, so the ray passes and the cut-out
  casts no shadow. The raytracer lights a replaced color (OpenGL does not with GL_REPLACE) - left as it is.
- **exudyn.graphics** (#3031): `Quad(texture, textureTiles)` - one quad of two triangles, coordinates (0,0) to the tiles;
  `color=None` is black, white with a texture. `ProjectTexture(graphicsData, texture, projection, point, rotationMatrix,
  size, tiles, color=white)`: planar keeps the points; box, cylindrical and spherical give each triangle points of its
  own (appended, so edges keep their indices), box chooses per triangle the axis nearest to its normal and orients the
  image not mirrored and upright on the sides; cylindrical puts the triangles facing along the axis planar on the
  faces; a triangle across the seam gets its angles on one side, a point on the axis the angle of the others.
  `Cylinder(texture, textureTiles)` = the cylinder, then ProjectTexture cylindrical with right-handed axes, the image
  once across the diameter on the faces. `_rowColumns` knows `textureCoordinates`; Transform and InvertTriangles copy
  the keys; MergeTriangleLists refuses two textures and gives a list without texture the coordinates nan - C++ draws a
  triangle with a nan coordinate without the texture (inf still refused). This was forced by the examples and
  laserScannerTest, which merge bricks into a CheckerBoard.
- **CheckerBoard** (#3036): the 2x2 image [[alternate, color], [color, alternate]] (first row at the top, so the tile
  at p0 has color), Nearest, RGB or RGBA if a color is transparent, tiles nTiles/2; the module function `_AddTexture`
  (hand-bound in Pybind_manual_classes.cpp, `PyAddTexture` with `allowReserved`) adds it without a SystemContainer -
  no defaults there, because the enums are registered after the manual classes (the module failed to import with
  them). Colors outside [0, 1] (a material in A) give the triangles. orientTrianglesTest and
  generalContactCylinderTest use `useTexture=False`: the first needs the triangles, the second makes its contact mesh of
  them.
- **ImageFromFile** (#3035): Pillow, converting other modes to L, RGB or RGBA; else matplotlib (PNG as floats, scaled);
  else ImportError naming both; pillow is in the extra [common] of pyproject.toml (checkExtras). Tested with a PNG
  written by matplotlib. hardwood_small.jpg is not in the repository:
  the folder has no license file and the JPEG no license metadata, so it waits for the maintainer (RG6.11.8 open).
- graphicsRegression.py: `texturedTriangles` is a kind of the fingerprint, with the texture coordinates and the names;
  the references recorded again - the CheckerBoard of the cases is now two textured triangles, and the raytraced image
  with showFaceEdges shows the edges of the two triangles instead of those of the tiles.
- textureGraphicsTest.py (3684.0): raytraced quadrants of a 2x2 texture, showTextures=False, the mask over a green quad,
  the coordinates of Quad, Cylinder (no seam triangle, tiles, faces), box, planar, spherical, Move/Merge/Invert, a
  merged list without texture drawn plain, CheckerBoard with texture against the triangles (< 5 % of the pixels
  differ), ImageFromFile; the sums of coordinates rounded to 1e-6, since the angles come from numpy.

<a id="rg6-11-7"></a>
### RG6.11.7 and RG6.11.8 — the textures example and the wood image (2026-10-18, #3033, #3035)
*(Maintainer 2026-10-18: hardwood.jpg, of which hardwood_small.jpg is built, is CC0 - add it to the testData; the
RG6.11.7 example, not a test, with a wooden floor and a cylinder with a painted pattern like colors of sin/cos, with
raytracer settings to render nicely.)*
- `python/Examples/testData/hardwood_small.jpg` (64 kB, 1024 x 1024): once in the repository, beside the other data of
  the examples; textureGraphicsTest.py reads it as `../Examples/testData/...`, as the runners change into the
  directory of the model. Its check is not part of the result (JPEG decoders may differ in the last levels): shape,
  uint8, the mean color brown, and a raytraced quad brown.
- `texturesExample.py`: the floor `graphics.Quad` 4 x 4 m with the wood twice in each direction, a brick with the wood by
  `ProjectTexture(..., 'box')`, a cylinder (r 0.4 m, 1 m) with a 512 x 256 pattern of red, green and blue from
  `cMath.sin`/`cos` of whole periods around (no seam) on a revolute joint, a quarter turn per second; light0
  directional with shadow 0.4 and lightRadius 0.15, ambient 0.5; `useRaytracer=True` (default): raytracer with 8
  threads, multiSampling 2, 7 light radius variations, keepWindowActive; SetModelView for the start view. A raytraced
  800 x 600 image of the scene was looked at; written without deprecated names (run with -W error::DeprecationWarning).
- Found by the examples run: the commit of RG6.11.5 (d48ebc4d) put `ImageFromFile` between the decorator `@_ReturnsRows`
  and `FromSTLfileASCII`, so the STL import returned flat lists and the image loader went through `_Rows`; the
  decorator is back on FromSTLfileASCII. The example reads the image as `../Examples/testData/...`, as the other
  examples with data do, since the runners do not start in the directory of the example.

<a id="rg6-11-10"></a>
### RG6.11.10 — the bit enumerations combine with | in Python (2026-10-18, #3037)
*(Maintainer 2026-10-18: "Python enums can't be OR-ed" - but this is widely used, e.g. for the output variables; adjust
TextureFlags to match the other enums. Measured: no enum of Exudyn had | in Python - `exu.NodeType.Position |
exu.NodeType.Orientation` raised TypeError; combined bits appeared as lists of names (requestedMarkerTypes) or plain
ints (selectionLeftMouseItemTypes=31). Decision by question: | for the enumerations of bits.)*
- `py::arithmetic` does not do it: pybind11 2.13 gives a scoped enum (enum class: OutputVariableType, TextureFlags,
  LinearSolverType, CrossSectionType) only comparisons, no bitwise operators (they exist for unscoped enums only).
  So `pybindTypes.EnumBitOperators()` emits `__or__`, `__ror__`, `__and__`, `__rand__` as `py::int_(a) | py::int_(b)`,
  for every enum with `bits=True` and OutputVariableType; they go after the values, as `.def` returns a `class_`
  that has no `.value`. The result is an int, as in C++. The stubs get the four operators, the documentation a sentence.
- `SC.renderer.AddTexture(..., flags)` and the internal `_AddTexture` take `Index flags` (an enum value converts via
  `__index__`), default 0; bits outside Nearest|Clamp|Replace raise ValueError, a list TypeError (introduced after
  1.11, so no deprecation). CheckerBoard, textureTest.py (two checks added) and textureGraphicsTest.py pass single
  values or |. The empty-`std::vector` default rule in autoGenerateHelper.py is gone with the list.

<a id="rg12-2"></a>
### RG12.2 and RG12.2.1, RG12.4.1 proposed — the user function interfaces measured (2026-10-18, #2982, #3038, #3039, #2984)
- **The table**: `docs/dev/design/userFunctionInterfaces.py` reads every user function def of the item definitions
  (member `userFunction`, annotations as written) and the three of the MainSystem, classifies what crosses the
  boundary, and takes the symbolic ones from `PySymbolicUserFunctionSet.h`: 45 user functions, 24 signatures, 39
  symbolic; without: the 5 graphics user functions and sensorUserFunction. Section 1 of userFunctionBoundary.md had 42
  and 28 (before the matrix results #3020); corrected there with the date.
- **Measured** (userFunctionBoundary.md section 13): perfUserFunctions.py gains rigid6 (RigidBodySpringDamper with
  NodeRigidBodyRxyz, explicit without constraints), ode1x6 (GenericODE1; 'none' uses systemMatrix, the rate 2 so that
  the result is not 0 after 4 s), mass6 (GenericODE2 mass matrix, 'none' the constant mass) and preStep (once per step:
  `callsPerStep`, also used by userFunctionPerformance.py); runTestSuiteRefSol.py has the 8 new timings and the new
  total. The Python cartesian/load functions are numpy-vectorized since #3011, which is why they are slower than the
  lists of section 12 in both versions - checked against 1.13.6 with the same functions, not a regression.
- **#3038** found while writing preStep: `return True` cannot be recorded; `CreateSymbolicUserFunction` turns a bool,
  int or float result into `exudyn.symbolic.Real`; symbolicUserFunctionSignaturesTest.py part (3), result unchanged.
- **#3039** (idea, RG12.2.2): constant matrix arguments copied at every call.
- **RG12.4.1**: the proposal in the plan; the fact behind it: `mbs` is passed as itself while recording.

<a id="rg6-19-2"></a>
### RG6.19.2 — zoom to the selected item (2026-10-18, #3027)
- `GraphicsData::ComputeMaxScene(..., onlyItemID)` takes only the elements an item drew (all kinds, the textured
  triangles included); `RenderState::ComputeZoomToItem` adds them over the graphics data in the rotation of the view
  (with a tracked marker, as ComputeMaxSceneSize), centers x and y, and takes the zoom from the box with the factor and
  offset of ZoomAll - the bounding-box branch of ComputeZoomAll is now `ZoomForBox`, used by both; the scene size and
  bounding box stay, so the clipping planes do not change; a camera-centric perspective view moves its camera as
  ZoomAll does (#3000).
- CTRL+A (plain A is ZoomAll): the item of `state->mouseSelectionItemID/Type/MbsNumber`, which the left mouse click
  sets and keeps after the highlight timeout; without a selection a message. `SC.renderer.ZoomToItem(itemIndex,
  mbsNumber=0, viewID=0)`: with an active renderer a request in RenderViewDataVSC (`zoomToItemRequest`, read with
  the zoom-all request in the update loop), else at once on the graphics data built as for the raytracer; False if the
  item draws nothing. keyBindings.py has CTRL+A.
- zoomToItemTest.py: a 10 m plate and two small bodies; the far body centered within 5 cm and the zoom below 0.2 of
  ZoomAll, scene size unchanged; node (with nodes.showBasis - a rigid-body node drawn as point has no graphics data),
  marker and load near it; a hidden node returns False and leaves the view; an int and a wrong mbsNumber refused.

<a id="rg7-4-1"></a>
### RG7.4.1 — NodeUser (2026-10-18, #2974)
- Definition `NodeUser` (itemDefsNodes.py) with the pattern of ObjectUserBody (design 5.9): `q` with the reference
  values, read-only arrays; `positionUserFunction` required (CheckPreAssembleConsistency, also the sizes of the three
  coordinate vectors); `positionJacobianUserFunction`, `rotationMatrixUserFunction`, `rotationJacobianUserFunction`
  optional. Type Position + Orientation, declared statically as ObjectUserBody does: a rigid marker on a node without
  rotation function raises when evaluated, and the rotation outputs (and HomogeneousTransformation, which the
  generator now adds under the condition of RotationMatrix) are conditional.
- `src/ImplNodes/CNodeUser.cpp` (new source; vcxproj, filters, sources.json): positions, velocities `J q_t`,
  accelerations - with a given J, `J q_tt + J_t q_t` with `J_t q_t` by central differences of J along q_t; without, the
  second derivative of the position along q_t (h = 1e-4 relative), as first differences of a numerical J amplified its
  rounding to 2.5e-4 in the test; angular velocities, `d(J_rot^T v)/dq` by central differences. The Evaluate functions
  in evaluateUserFunctions.cpp: caches as #3028 (local position 0), numerical position Jacobian, numerical rotation
  Jacobian from the axial vector of `dR/dq_k R^T`.
- A node needs the MainSystem and the time: `CNode::SetCSystemData` (virtual, empty) is called by the generated
  registration for every node, NodeUser keeps it; the node number is searched in the node list at the call, as
  DeleteNode renumbers. `CNode::AccessFunctionsCallUserFunctions` makes VisualizationSystem draw the node, the
  objects on it, the markers and sensors on it in the main thread (mainThreadNodes).
- The generator adds symbolic versions of the four user functions (symbolicUserFunctionSignaturesTest.py: 43
  signatures, all agree); a symbolic NodeUser function through CreateSymbolicUserFunction with a NodeIndex is not
  tried yet.
- userNodeTest.py (30.459...): a point with numerical and given Jacobian against NodePoint (1.3e-8), a rigid-body node
  in Rxyz coordinates with the rigid-body mass matrix in ObjectGenericODE2, RigidBodySpringDamper on MarkerNodeRigid and
  a torque, rotation Jacobian given and numerical, against NodeRigidBodyRxyz (4.4e-9); refused: no position function,
  wrong size, rotation without function, wrong Jacobian size; the node in the graphics data. Also: test_itemCompatibility
  arguments, the drawing settings, parameterConversionTest reference, mini example and its graphics reference, the
  pydoclint baseline (DOC108 of the Protocol arguments, as ObjectUserBody).

<a id="rg7-4-2"></a>
### RG7.4.2 — the fully parametrized ANCF beam on NodeUser (2026-10-19, #2974)
- userANCFBeam3DTest.py: nodes NodeUser(12): r, r_x, r_y, r_z; position and its Jacobian given, the rotation from
  the gradients, its Jacobian by the central differences of NodeUser. The element ObjectUserBody(24): cubic Hermite
  along the axis for r and r_x, linear for r_y and r_z (Shabana's fully parametrized element), constant mass matrix,
  forces f_a = -sum_p w_p F S dN_a/dX with F, the Green-Lagrange strain and the second Piola-Kirchhoff stress of
  St.Venant-Kirchhoff at 4 x 2 x 2 Gauss points, vectorized with einsum (the whole test 0.5 s). nu = 0 avoids the
  Poisson locking of the linear cross section.
- Tip force -0.1 (E = 1e6, square 0.1, L = 1): 2, 4, 8 elements 6.3 %, 1.7 %, 0.50 % from Timoshenko (-0.004024), the
  shear locking of the element vanishing with the mesh.
- Tip torque on MarkerNodeRigid: with the frame of r_x and r_y the tip rotated 5.7 % more than M L/(E I) - the torque
  acted on the tangent of the axis; with the frame of the cross section (r_z and r_y orthonormalized) it acts on the
  gradients of the cross section, as the stress of pure bending, and the rotation is M L/(E I) to 1e-6 for 2 to 8
  elements. The test uses this frame and says why.
- The result is the sum of three deflections and the angle, to the tolerance of Newton: `exu.sys['testTolerance'] = 1e-9`.

<a id="rg6-19-1"></a>
### RG6.19.1 — the icon bar (2026-10-19, #3026)
*(Maintainer 2026-10-19: start with the implementation as planned; the order of the icons with SHIFT and CTRL as in
the plan; `interactive.iconBar.scaling`; about 19 icons, 1200 pixels high. The list gives 20: quit, 2 zooms, 4
views, ruler, mouse coordinates, lines, edges, faces, transparency, update speed and 6 item kinds.)*
- **Icons** (src/Graphics/IconBar.h/.cpp, compiled in every build): a canvas of 64 x 64 RGBA floats with
  anti-aliased shapes from the signed distance of the pixel center (lines, polylines as one shape, arcs, discs,
  polygons by the even-odd rule, arrows), blended over; one drawing per icon, in ink, blue, light blue and red.
  `IconBar::AddIconTextures` puts them into the TextureStore as 'exudyn.icon.<name>' (RGBA, Clamp) when the
  renderer creates its font textures; OpenGL uploads them with the scene textures. `exudyn._compiledModule.
  _IconBarImages` gives the images to Python, for test_iconBar.py and to look at them - looked at as a sheet.
- **One action per key and icon**, simpler than the planned ExecuteAction refactoring of key_callback: an icon sends
  its key to key_callback (ESCAPE, A, CTRL+A, CTRL+1 to 7 with SHIFT, F5, F3, 1 to 5, N, B, C, M, L, S with CTRL for
  SHIFT), so a key and its icon do the same by construction; lines, edges (face and mesh edges), faces (face and mesh
  faces) and transparency have no key of their own and switch the settings of the view. A consequence: F2 (ignore
  keys) also stops the icons.
- **The bar** (GlfwClientExtended.cpp, `DrawIconBar` after `SaveImage`, so not in saved or retrieved images): pixel
  coordinates of the frame buffer, the mouse converted from window coordinates (retina); folds in when the window is
  hovered, no mouse button is pressed and the mouse is within `foldDistance` x display scaling of the right border,
  stays while the mouse is over it, slides over 0.15 s with a refresh signal per frame; icon 54 px x scaling x display
  scaling with 4 px gaps, made smaller to fit the height. Pressed toggles on light blue, the hovered icon grey; texts
  over the icons with the bitmap font - the axes of the views (swapped while CTRL is held), the number of the update
  interval, # for shown item numbers; the tooltip of the hovered icon left of the bar. A mouse press on the bar runs
  the action (left button) and, with its release, does not reach the selection and the rotation.
- Settings `interactive.iconBar.show` (True), `scaling` (1), `foldDistance` (20 px) - a new structure VSettingsIconBar.
  Its class is in stubtestBaseline.txt as every VSettings class (pybind metaclass, written by checkPython.py
  --stubs --write); parameterConversionTest reference with its three settings.
- **Not seen on a screen** (no window may open here): RG6.19.3 is the check; the positions of the texts over the icons
  (the bitmap font drawn with a flipped y axis) are the most likely to need a correction.
- The proposal as it was decided:

For a decision. Facts: the renderer is OpenGL 1.x (immediate mode, display lists); `key_callback`
(GlfwClient.cpp:204-922) performs each action inline; the mouse callbacks select items on click and rotate or move on
drag (961-1188); the display scaling is `state->displayScaling` (1887).
- **Actions**: the bodies of the key cases move into one function `ExecuteAction(RendererAction)`; a key and an icon
  call the same action, so nothing is done twice and the help lists both. Not every key gets an icon.
- **The icons** (about 30, in groups): visualization settings, command window, help; views XY, YZ, XZ, isometric;
  show/hide nodes, bodies, connectors, markers, loads, sensors and their numbers (a click on the small number field of
  the icon); zoom in, out, all, to the selection (RG6.19.2); ruler (RG6.12); mouse coordinates; lines, edges, faces;
  global transparency; graphics update speed (cycles through the five intervals); quit. A toggle shows its state
  (pressed or not).
- **Drawing the icons**: each icon is a small C++ function on a 64 x 64 canvas using a few primitives - line, polyline,
  circle, filled rectangle, filled triangle and a glyph of the font bitmap that is compiled in already - which a small
  rasterizer writes into an RGBA buffer (the raytracer has RasterizeLine and RasterizeCharacter, Raytracing.cpp:578,
  746). At startup all icons are drawn once and stored in the texture store of RG6.11.1 under reserved names
  (`exudyn.icon.zoomAll`); no files, no data beyond the code. In OpenGL 3.3/4.1 the same textures are drawn as quads
  with a shader. Until RG6.11 exists, the icon textures are created directly as GL textures, as the font is.
- **The bar**: drawn in pixel coordinates after the overlay (`glOrtho(0, width, 0, height)`, as `DrawImageRGBA`
  does), by one function `DrawIconBar`; it is not part of the raytraced or saved images. It folds in from the right
  border when the mouse comes within `interactive.iconBar.foldDistance` pixels (default 20) without a button pressed,
  slides in over about 0.15 s and folds out when the mouse leaves it; it fills the height of the window, the icon size
  is `interactive.iconBar.size` times the font size times the display scaling, shrunk to fit the height; a click on an
  icon is taken before selection and rotation; the name and the key of an icon appear beside it on hover.
  `interactive.iconBar.show` (default True) switches it off.
- **Steps after the decision**: ExecuteAction with the keys unchanged (the GUI tests pass); the icon rasterizer with a
  pytest of a few icons as images; the bar and its mouse handling; the icons; the documentation of the keys and icons
  in GUI.md.

<a id="rg6-19-3"></a>
### RG6.19.3 — the icon bar checked on the screen and adapted (2026-10-19, #3026)
*(Maintainer 2026-10-19: works well; the bar shall stay a little longer, 3 times longer than now; zoom to the
selection is a region selected by two mouse clicks - a misunderstanding; the bar smaller, scaling 0.7 by default.)*
- After the mouse left the bar, it stays `iconBarHoldTime` = 0.3 s before it folds out over 0.15 s: 0.45 s, 3 times the
  fold time it had.
- The icon `zoomRegion` (was zoomSelection, a magnifier over a dashed frame): the next two left clicks outside the bar
  are the corners of a region, drawn as a rectangle from the first corner to the mouse; the region fills the window -
  its center becomes the center of the view, the zoom shrinks by the larger of its relative width and height
  (`IconBarZoomToRegion`); a right click or the icon again cancels; the clicks do not reach the selection or the
  movement of the model. CTRL+A stays the zoom to the selected item (RG6.19.2), without an icon.
- `interactive.iconBar.scaling` = 0.7 by default: 38 px icons at display scaling 1, about 830 px for the bar.

<a id="rg6-19-4"></a>
### RG6.19.4 — the icon bar after the second check on the screen (2026-10-19, #3040)
*(Maintainer 2026-10-19, with a screenshot: keep CTRL+A and zoom to a region also with SHIFT + click and drag; the
tooltip more to the left, so that it does not overlap the bar, its background about 15 % up and 10 % higher, and
scaled with the monitor scaling and general.displayScaleFactor; iconBar.show to view.window.showIconBar, the other two
to general, plus general.iconBarTimeout in milliseconds, no deprecation; edges a rectangle split into 8 triangles,
faces a cube with a front and two side faces, lines the cube's edges; the standard update interval into the list.)*
- **Settings.** The structure VSettingsIconBar is gone: `view0.window.showIconBar` (True, per view),
  `general.iconBarScaling` (0.7), `general.iconBarFoldDistance` (20 px), `general.iconBarTimeout` (300 ms, the hold
  time that was the constant `iconBarHoldTime`). Introduced after 1.11, so renamed without deprecation.
- **SHIFT+drag** (`IconBarMouseButton`, `regionStep` 3): a left press with exactly SHIFT outside the bar starts the
  rectangle, drawn as for the icon; the release zooms with `IconBarZoomToRegion`; less than 3 px is a click and does
  nothing. It works with the bar switched off. SHIFT with the left button had no use in the render window. A row in `keyBindings.mouseBindings`, so the help text and the generated table have it.
- **Tooltip.** The width was estimated with 0.6 times the font size per character, the font advances by
  `characterWidth/fontSize` = 47/64 = 0.73: the text ran into the bar. It is now measured with the font's advance,
  and drawn in `window.globalFontSize` times `GetFontScaling` (monitor scaling times displayScaleFactor) instead of a
  fraction of the icon size; the background is 1.76 font sizes high (was 1.6), its center 0.24 up.
- **Icons.** `edges`: the outline and a 2 x 2 grid with one diagonal per cell; `faces`: a cube with front, top and
  right faces (`DrawCube`), without outline - the icon of the bodies has one and other proportions; `lines`: the
  edges of the same cube, the three hidden ones thin and grey.
- **Update interval.** The 0.1 s default is the second key interval; the "1" of the screenshot is what
  `IconBarUpdateInterval` answered for 0.02 s and for every interval that is not one of the keys. Now
  `IconBarUpdateIntervals` sorts the interval of the model, taken when the bar is drawn first after `StartRenderer`,
  into the key intervals; the icon cycles through that list and sets the interval itself (keys 1 to 5 cannot reach the
  model's), shows the number in the list (none for another value) and its tooltip lists the intervals.

<a id="rg6-19-5"></a>
### RG6.19.5 — the tooltips of the icon bar in the size of the version string (2026-10-19, #3041)
*(Maintainer 2026-10-19: the tooltips follow the scaling - good -, but are quite large; they shall have the
size of the version string, which has a smaller factor, still scaled as now.)*
- The version string is drawn with `globalFontSize*fontSmallFactor` (0.75, GlfwClientBase.h), and so is the
  tooltip now, times `GetFontScaling` (monitor scaling times displayScaleFactor) as before.

<a id="rg6-19-6"></a>
### RG6.19.6 — the store windows of the settings dialog (2026-10-19, #3042)
*(Maintainer 2026-10-19: the button 'store' of the store settings window becomes 'store in config.json'
with a hint, because the positions are stored in the same local file; the one-line descriptions of the
store settings and store positions windows get line breaks.)*
- `ShowCodeLines` wraps the description with `textwrap.fill(..., 80)`, the width of its code area, and
  takes `confirm` as a list of (text, function, tooltip): the buttons sit in a frame of their own, each
  with a tooltip.
- Both store windows have **store in config.json**; its tooltip names the file, the settings file of this
  computer and account, which keeps the window positions as well as the settings.

<a id="rg6-19-7"></a>
### RG6.19.7 — stored configurations CFG0 to CFG3 (2026-10-19, #3043)
*(Maintainer 2026-10-19, a user request: four configurations CFG0 to CFG3 to switch the whole settings
between, in ~/.exudyn/config.json as a list storedSettings of dictionaries with the difference to the
default, created as empty dictionaries, which equal a reset; an icon switches to one, CTRL and the icon
store into it; buttons -> CFG0 ... with hints and a UTF-8 arrow in store settings. Asked: all four
stored, none fixed; a script stores with GUI.StoreVisualizationSettings(CFG=None|0..3), None the regular
section - so that CFG0 can be set right before renderer.Start(); switching gives the defaults plus the
stored difference.)*
- **overrideSettings**: section `storedSettings` (in `sectionNames`, not applied at import);
  `numberOfStoredSettings` = 4; `ChangedVisualizationSettings(vs)` - the storable difference to the
  defaults, which `Store(SC)` used inline and now calls; `StoredSettings()` reads the FILE, so that a
  configuration stored by another script is the one switched to; `StoreStoredSettings(index, values)`
  writes all four; `ApplyStoredSettings(vs, index)` sets every leaf whose value string differs from the
  default back to it (not `interactive.highlightItemType`) and then applies the configuration through
  `ApplyVisualizationSettings(..., record=False)`, so that `Applied()` keeps describing the import.
  A user function of the settings is not a leaf of the dictionary and is not touched.
- **GUI**: `StoreVisualizationSettings(SC=None, CFG=None)` and `ApplyStoredVisualizationSettings(CFG,
  SC=None)`, the renderer's container when SC is None; the switch sends a redraw signal.
- **Icons** config0 to config3 at the end of the bar (24 icons): three sliders, the bar writes the
  number. A click queues `ApplyStoredVisualizationSettings(CFG=n)`, CTRL and a click
  `StoreVisualizationSettings(CFG=n)`, as a string for the Python thread (`PyQueueExecutableString`,
  through `__import__`, so nothing is added to the namespace of the model), as the dialogs do.
- **store settings** window: buttons → CFG0 to → CFG3 (U+2192) with tooltips.
- Tests in test_userSettings.py (the file of the pytest's own): all four written, defaults plus the
  difference, an empty one resets, the index checked, the GUI functions; test_iconBar.py with 24 icons.

<a id="rg6-19-8"></a>
### RG6.19.8 — a shadow icon, the CFG icons as buttons (2026-10-19, #3044)
*(Maintainer 2026-10-19: add a shadow switch 0.25 / 0; CFG0-3 with an underlay rectangle slightly smaller than the
icon, so that it looks like a settings button, and the text CFG0 ... instead of the number, which fits at the current
text size; the hints of the CFG buttons much too long, about half.)*
- Icon `shadow` after `transparency` (25 icons): a ball over its shadow. On when a light that is on casts a shadow
  (`IconBarShadowIsOn`); a click sets `openGL.light0.shadow` to 0.25 and switches light0 on, or sets the shadow of
  the four lights to 0.
- config0 to config3: a light rectangle with rounded corners at 4 to 60 of the 64 pixels and three sliders in its
  lower part; the bar writes CFG0 ... centered over the upper part, in dark blue. The text over an icon is placed by
  the advance of the font (characterWidth/fontSize = 0.73 of its size), it was 0.62. test_iconBar.py allows the
  buttons 90 % coverage.
- Hints: the tooltips of the icons "switch to the stored configuration CFG0 (CTRL: store settings and view as CFG0)",
  the buttons of the store windows "store these settings as configuration CFG0 in config.json; the icon CFG0 switches
  to them" - about half of before.

<a id="rg6-19-9"></a>
### RG6.19.9 — the stored configurations with the model view (2026-10-19, #3045)
*(Maintainer 2026-10-19: CFG0-3 do not store the model view, but should; the GUI storing function gets a flag for the
settings and for the model view; CTRL+CFGx stores both; store settings stores the settings into CFGx, store model view
the model view; the dictionaries of both are empty by default and only override what is there - an empty model view
does not change the current one -, so that a user can have a view, settings, or both.)*
- A configuration is `{'visualizationSettings': {...}, 'modelView': {...}}`; `overrideSettings.modelViewKeys`
  centerPoint, rotationCenterPoint, zoom, modelRotation - the entries of the render state that SetState takes and
  that make the view; `ModelView(renderState)` gives them as plain lists. `StoreStoredSettings(index,
  visualizationSettings=None, modelView=None)` replaces the parts given and keeps the others.
- `ApplyStoredSettings(vs, index, renderer=None)`: settings only if the configuration stores some (then the
  defaults plus them, as before), the view through `renderer.SetState` only if it stores one; returns (number of
  settings, view set). An empty configuration is no longer a reset - the maintainer's rule now, an empty part changes
  nothing.
- `GUI.StoreVisualizationSettings(SC, CFG, storeSettings=True, storeModelView=False)` - the model view only into a
  configuration, else a ValueError; `ApplyStoredVisualizationSettings(CFG, SC)` returns the tuple. CTRL and an icon
  call it with both flags; the window of **store model view** has the buttons → CFG0 ... for the view.
- An entry of the earlier form, a flat dictionary of paths, is read as empty; it existed for a few hours.
- Tests: the two parts, the view switched with the settings kept (test_aConfigurationSwitchesTheModelView), the GUI
  flags.

<a id="rg12-2-3"></a>
### RG12.2.3 — the memory allocations of a symbolic ObjectUserBody (2026-10-19, #3046)
*(Maintainer 2026-10-19: userRigidBodyTest.py with 10000 time steps still reports 613350 memory allocations for the
body with symbolic user functions, ODE2RHS 83 %, user functions 91 %; what is the cause - a step still to do, or
another limitation?)*
- Measured with `exu.special.InfoStat` around `SolveDynamic`, 100 and 200 steps, full Newton, the body alone with
  each user function symbolic in turn (scratch script): 49 matrix and 4 vector allocations per step with Python user
  functions, 71 with the symbolic positionJacobianUserFunction; ObjectRigidBody 0.
- Cause 1: `MatrixBase::CopyFrom`, `SetMatrix` and `operator=` called `ResizeMatrix`, which frees and allocates also
  at the same size; a symbolic matrix result is copied with `CopyFrom` into the Matrix of the caller. Now
  `SetNumberOfRowsAndColumns`, which reallocates only for another size (src/Linalg/Matrix.h) - the rule of Vector
  already. 71 -> 49.
- Cause 2: ObjectUserBody's local `Matrix jacobian`, `Matrix rotation`, `Vector plus(n), minus(n)` in
  GetJacobianTransposedTimesVectorDerivative, JacobianTimesVelocity and EvaluateRotationMatrix; now the mutable
  buffers bufferJacobian, bufferRotation, bufferPlus, bufferMinus. Each is written by a copy after the user function
  returned, so a user function that asks the body again (an output) does not change what its caller reads. 49 -> 33.
- Cause 3: `UserFunctionResultCache<ResizableMatrix>::Store` took `const TValue&`, so a Matrix was converted into a
  temporary ResizableMatrix at every store; now a template that copies into the entry (CObjectBody.h). 33 -> 0, for
  Python and symbolic user functions alike.
- The time did not change: 10000 steps take 1.86 s (symbolic), 4.35 s (Python), 0.12 s (ObjectRigidBody). The cost is
  the evaluation itself: the force expression is a text of 9456 characters, one evaluation about 5 us from Python
  (mass matrix 2.7, position 1.4, position Jacobian 4.4), and an evaluation of the right-hand side calls the force and,
  for three markers, positions, velocities and Jacobians. A shared node is evaluated at every use - planned as
  RG12.2.4 (#3047).

<a id="rg12-2-4"></a>
### RG12.2.4 — closed: no shared-node values in the symbolic evaluation (2026-10-19, #3047)
*(Maintainer 2026-10-19: totally against the proposal and against any hashing of values; this is a design decision
and needs to be treated carefully. Many things are recomputed, in particular when markers are evaluated twice, and
node and body functions overlap, like the G matrix; but in parallel computation it is often better to recompute
values than to load them from memory, and intermediate storage destroys the flexibility and transparency of the
approach and leads to problems that are very hard to detect - the time spent on them is usually much longer than the
time saved. A workaround is the combined function of the rigid-body marker, which computes all values at once and is
faster than the single position, velocity, rotation matrix functions; possibly a cause of the lower speed here, not
checked. The symbolic rigid body is a demo and test case and is left as it is.)*
- #3047 closed as won't fix; the decision is D18 of the info document.
- *(Maintainer 2026-10-20, after the question what "node" meant - a node of the expression tree, not a node item:
  #3047 stays closed; a new step evaluates the options first - an automatic value of a tree node for one evaluation,
  an algorithmic form with statements, intermediate variables stored by hand, others -, without the need for ultimate
  performance now and with the recording of the C++ items in view, so that too many changes are counter-productive;
  the result caches of #3028 are removed; D18 covers the items, the core and the visualization, and storing values
  is used rarely, each case discussed with and approved by the maintainer.)* RG12.2.5 (#3048) evaluates the
  options, RG7.11.1 (#3049) removes the caches. The buffers of #3046 reuse memory only and store no value that a
  later evaluation reads.
- `ObjectUserBody` uses the generic `CObjectBody::ComputeRigidBodyMarkerData`, which calls GetPosition, GetVelocity,
  GetRotationMatrix and GetAngularVelocityLocal one by one; a combined function, as ObjectRigidBody has, is not planned.

<a id="rg7-11-1"></a>
### RG7.11.1 — the result caches of ObjectUserBody and NodeUser removed (2026-10-20, #3049)
*(Maintainer 2026-10-20: remove #3028; D18 relates to the Exudyn items, the CSystem core and the visualization, and
storing values shall be used rarely, each case discussed with and approved by the maintainer.)*
- `UserFunctionResultCache` (CObjectBody.h), the four caches of each item, `ClearUserFunctionResults`, and the
  `ParametersHaveChanged` and `PreAssemble` of ObjectUserBody that cleared them are gone; the user functions are
  called for every request. The descriptions of both items no longer say that the user functions must be functions
  of their arguments. NodeUser's rotation matrix goes through a buffer `bufferRotation` (memory only), as
  ObjectUserBody's, so that no allocation per call comes back.
- userRigidBodyTest.py keeps the check that a new position user function is used at once; its header no longer
  names the caches. The results of the test models are unchanged.
- userRigidBodyTest.py with 10000 steps, as RG12.2.3: Python user functions 4.35 -> 10.9 s, symbolic 1.86 -> 4.13 s,
  ObjectRigidBody 0.12 s; no memory allocation per step.

<a id="rg6-19-10"></a>
### RG6.19.10 — the CFG buttons and the bar with the raytracer (2026-10-20, #3050)
*(Maintainer 2026-10-20, with a screenshot of the CFG buttons: CFG shall be written lower, the underlay rectangle
slightly larger and without border; with the raytracer the bar also fades in, and as the raytracer is slow it shall
show completely with no delay.)*
- config0 to config3: the rounded rectangle at 1 to 63 of the 64 pixels, radius 9, without the outline; the text
  CFG0 ... has its baseline at 0.42 of the icon height (was 1.1 font sizes, so that it stood on the upper edge),
  above the three sliders. test_iconBar.py allows these buttons 97 % coverage.
- With `camera.useRaytracer` of the view, `bar.visible` jumps to its target instead of sliding over 0.15 s; the time the
  bar stays after the mouse left it is the same.

<a id="rg6-19-11"></a>
### RG6.19.11 — the icon for the visualization settings (2026-10-20, #3051)
*(Maintainer 2026-10-20: the icon bar looks good; add an icon to open the visualization settings.)*
- Icon `settings` after `sensors` and before the CFG buttons (26 icons): a gear wheel of 8 teeth with a light hub; it
  sends the key V. GUI.md lists it.

<a id="rg6-8-12"></a>
### RG6.8.12 — a switch and a measurement for the split cache (2026-10-20, #3052)
*(Maintainer 2026-10-20: does triangles6SplitCache make sense at all? In flexible bodies the shapes deform, so the cache
will not act; pure rigid body transformations could be done via OpenGL, which is not done as display lists are not
widely used. Are there measurements in common situations? If not, an experimental flag to check the influence; the
maintainer tries a FEM example with quadratic triangles or many cylinders.)*
- The measurement there is (revision2026b RG6.7.2.3): a partial sphere of 90000 triangles6 through the raytracer
  takes 139 ms against 79 ms for the same 360000 flat triangles, so the split costs about 60 ms per image (0.7 us per
  element). Nothing was measured in the OpenGL renderer and nothing for common models. The cache acts only for frames
  without a new graphics update - turning the view of a paused or finished simulation -, as every graphics update
  raises the data version; graphics.Cylinder and Tube are made of 6-node triangles, so many cylinders are a case.
- `exu.experimental.cacheCurvedTriangleSplit` (True): False splits in every frame. `curvedTriangleSplitCount` and
  `curvedTriangleSplitTime` add up the splits of a non-empty list and their seconds in the OpenGL renderer
  (`EXUvis::SplitTriangles6Cached`); set both to 0 before a measurement. The raytracer splits per image anyway.
- Usage: run the model with the renderer, rotate the view for some seconds after the simulation, print the count and
  the time with and without the cache; the time per frame without the cache is what the cache saves per frame.

<a id="rg12-2-5"></a>
### RG12.2.5 — the options for intermediate values in symbolic expressions (2026-10-20, #3048)
*(Maintainer 2026-10-20: do RG12.2.5 now - find a good way to extend or optimize the symbolic functions; expressions
might require a scope for variables; the current variable sets may already be a way to store intermediate values as in
algorithms; and for the most important objects - rigid bodies, Cable2D, connectors and joints, without FFRF and
complex features - would it be feasible to record the expressions of a mass matrix or residual, or would that require
re-implementing everything as a symbolic tree? This could influence the way to go.)*
- The evaluation and the recommendation are in docs/dev/design/symbolicIntermediateValues.md; the counts come from
  docs/dev/design/symbolicGraphSize.py, which records the component functions of userRigidBodyTest.py with a counting
  type: force 122 nodes, 1171 operations as a tree (9.6 times), position 48 and 71.
- Read from the code: SetValue of a variable during recording runs once and is not recorded, so a recorded function
  writes nothing; a VariableSet is global state written from Python between calls. The vector and matrix nodes have
  per-node buffers of the current evaluation, but a shared vector node is still computed twice.
- The AD templates: connectors and joints on `MarkerRigid<TReal>` (#2745), `ComputeODE2LHStemplate` of the ANCF
  elements, the position and rotation of ObjectRigidBody (#2744); ComputeMassMatrix and ComputeODE2LHS of the rigid
  body and the G matrices of the nodes are `Real` only. Branches on values such as `if (springLength2 != 0.)` are the
  review a recording needs. `SReal(ExpressionBase*)` evaluates the new node's subtree at once, which a large recording
  would have to replace by the value from the operands.
- Recommendation: the flat evaluation of scalar expressions first, then vectors and matrices; a feasibility step for
  the recording number type with SpringDamper and ANCFCable2D. Nothing implemented; the maintainer decides.

<a id="rg6-8-12-done"></a>
### RG6.8.12 — the split cache removed (2026-10-20, #3052)
*(Maintainer 2026-10-20: tested with graphicsCurvedShapes, the resolution and the triangle refinement increased; the
speed looked the same with and without the cache, so the split cache and the experimental flag can be removed.)*
- Removed: `triangles6SplitCache`, `triangles6EdgesCache`, `Triangles6SplitKey` and the data version of GraphicsData,
  which only the cache read; `EXUvis::SplitTriangles6Cached`; `exu.experimental.cacheCurvedTriangleSplit`,
  `curvedTriangleSplitCount` and `curvedTriangleSplitTime`.
- The OpenGL renderer splits the 6-node triangles of each GraphicsData in every frame with `EXUvis::SplitTriangles6`
  into `GlfwRenderer::triangles6Split` and `triangles6Edges`, buffers of the render thread as `lines3Split` - the
  faces, the normals and the shadow volumes of that GraphicsData read them in the same pass. The raytracer splits per
  image as before.

<a id="rg12-2-6"></a>
### RG12.2.6 — the flat evaluation of the scalar expressions (2026-10-20, #3053)
*(Maintainer 2026-10-20, on RG12.2.5: implement the flat evaluation for scalar expressions next, vectors and matrices
afterwards, with the proposed tests; add issues and steps; allow export or conversion to text of the compiled lists.
The C++ recording is not started now.)*
- **The operand interface** of `ExpressionBase` (Symbolic.h): `NumberOfOperands`, `GetOperand`, `EvaluateFromOperands`,
  `ToStringFromOperands`; the defaults make a node a leaf evaluated by `Evaluate()`. The 39 classes with one or two
  scalar operands (arithmetic, comparisons, the functions, pow, atan2, mod, min, max) override them; the bodies were
  generated from each class's own `Evaluate()` and `ToString()`, the operand calls replaced by the values, so that both
  compute with the same expression. IfThenElse stays a leaf - its branches are evaluated lazily, as a tree -, as do the
  components of vector and matrix expressions (RG12.2.7).
- **FlatProgram**: `Build(roots)` sorts the graph depth first without recursion, a map from node to slot used only
  while building; operation k writes slot k from the slots of its operands; `Evaluate()` runs the list; the slots are
  overwritten by every evaluation (D18). `ToString` writes `tK = op(...)` per operation with operands, leaves written
  into their users (`cos(q[1])`), then the results.
- **SymbolicFunction**: `BuildFlatProgram()` after the return type is set - a scalar, a vector of components
  (VectorExpressionSReal) or a matrix of components (MatrixExpressionSReal); a program of leaves only is dropped (the
  tree is as fast). The evaluation functions, including the sparse matrix result (#3022), use it when
  `flatEvaluation` (True); `Delete()` clears it. Python: `UserFunction.flatEvaluation`, `UserFunction.GetFlatProgram()`,
  declared in definitions/pybindSymbolic.py for the stub and the reference.
- **Tests** (test_symbolicFlatEvaluation.py): bit-identical results of both evaluations for a vector force with shared
  values on several levels, a matrix of components with zeros, a scalar with IfThenElse, the functions and the
  comparisons, 25 to 50 random arguments each; a solution of 200 steps through ObjectGenericODE2 bit-identical; the
  text - one line per operation, `cos(q[1])` once, the result names. The symbolic test models keep their results.
- **Measured**: the force of userRigidBodyTest.py, 122 operations, 5.5 -> 1.5 us per call from Python (with about 1 us
  of call overhead); the position 1.45 -> 1.3 us. The mass matrix and the Jacobians are matrix products and stay trees
  until RG12.2.7. userRigidBodyTest.py with 10000 steps, the symbolic body alone, both runs twice: 2.70 s (tree) ->
  2.19 s (flat). (The 4.13 s of RG7.11.1 were measured on a loaded machine; the tree evaluation now takes 2.70 s.)

<a id="rg12-2-7"></a>
### RG12.2.7 — the flat evaluation of the vector and matrix operations (2026-10-20, #3054)
*(Maintainer 2026-10-20, on RG12.2.5: the vectors and matrices after the scalar expressions, with the same tests and
the text export.)*
- `FlatNode` (the kind - number, vector, matrix - and the node) and `FlatValue` (the number, or a pointer to the vector
  or matrix an operand's node evaluated into) in Symbolic.h; `GetOperand` returns a FlatNode and `EvaluateFromOperands`
  takes FlatValues for all three kinds. VectorExpressionBase and MatrixExpressionBase have the interface with the
  default of a leaf (`EvaluateRef()`). The 13 classes of SymbolicVector.h and 8 of SymbolicMatrix.h with operands got
  it generated from their own `EvaluateRef` / `Evaluate` and `ToString` - the scalar ones there too (dot product,
  norm, the brackets, the comparisons of vectors). The component of a matrix reads the operand's value instead of
  `EvaluateComponent`: the same sum in the same order. The vector and matrix of scalar components
  (VectorExpressionSReal, MatrixExpressionSReal) take their components as operands; the accessors of RG12.2.6 are gone.
- FlatProgram: operations of any kind with any number of operands (a list of operand slots), slots of FlatValue; the
  result is the root of the return value - a vector or matrix result is the node's own memory, returned without copy;
  the sparse matrix result reads its pattern from it. A component of an argument, `q[1]`, is an operation but written
  inline in the text and not counted (`InlineInFlatText`).
- Tests (test_symbolicFlatEvaluation.py): two more functions - matrix products with a shared matrix G, vector
  operations with a matrix, a dot product, a norm and components of computed vectors - bit-identical to the tree; the
  text: the vector root as `[t.., t.., t..]`, G one operation used by `.T` and `@`.
- Measured: per call from Python, flat against tree: force 1.8 / 5.8 us, mass matrix (G.T J G) 1.95 / 2.7, position
  Jacobian 2.2 / 4.3, position 1.5 / 1.45. userRigidBodyTest.py with 10000 steps, the symbolic body alone, twice each:
  2.72 s (tree) -> 0.75 s (flat); ObjectRigidBody 0.12 s.

<a id="rg7-12-1"></a>
### RG7.12.1 — the parameters of the user items, evaluated (2026-10-21, #3056)
*(Maintainer 2026-10-20: the ObjectUserBody, and future user items, has no parameters - they are in the symbolic user
functions; is that the planned and optimal way, think of a user beam with different parameters per element. Suggestion:
an arbitrary-length parameter vector of doubles, which also allows bools and indices, available in the user functions
(changing the interfaces, which are not public yet); the user defines the meaning, and a Python mapping in both
directions, map2dict(parameter vector) -> dict and map2parameters(dict) -> vector, with lists in the dict, keeps the
symbolic quantities in the recorded functions, so no overhead; zero length is the present implementation; the mapping
defined in the script with the user functions, recommended as the way to go, or integrated into the user item as a
Python function only used at definition. Evaluate and recommend in the first sub-step. Also: the GPU tests of
recorded Python symbolic user functions as a step in RG5, done by someone else - RG5.5, #3055.)*
- Measured (scratch script): one recorded forceUserFunction assigned to two ObjectUserBody gives both the same motion;
  recording the force of userRigidBodyTest.py takes 0.2 ms. userANCFBeam3DTest.py creates the functions per element
  length by a closure.
- The evaluation and the recommendation are pythonUserItems.md section 5.10: `parameters` last in every user
  function, a named symbolic vector while recording, so one recording serves all items of a kind and SetObjectParameter
  acts without recording again; its components are read at each call, inline in the list of operations as `q[1]`; structural
  parameters (sizes, Gauss points, Python branches) cannot come from it in a symbolic function and stay Python
  constants; the mapping as a Python helper `ParameterLayout` (`ToVector` checking names and sizes, `ToDict` for numpy
  and symbolic vectors) next to the user functions, not as an item parameter, which C++ never calls and which would put
  a Python object into the item's dict. Nothing implemented.

<a id="rg7-12-2"></a>
### RG7.12.2 — the interfaces of the user items that last (2026-10-22, #3056)
*(Maintainer 2026-10-21: rigid bodies need total, finite elements relative coordinates and the reference ones for
drawing - add the current and the reference configuration to the arguments, possibly qRef, q and q_t, in the
visualization from its configuration; the visualization should have a parameter vector of its own, and the
graphicsDataUserFunction gets both; graphics data mapped by a second visualization user function on local positions,
possibly nonlinear, or stay with graphicsDataUserFunctions producing meshes, also symbolic; symbolic functions need
some access to the visualization settings, for appearance and contour colors; check the user item tests, the rigid
body and a beam element for what is missing; put the considerations into the plan of the Python user items, and choose
interfaces that allow later extensions without changing them.)*
- A comparison of ObjectUserBody and NodeUser with ObjectRigidBody, ObjectANCFCable2D and the geometrically exact beams,
  read from the definitions and the C++ (a research agent, checked against the sources quoted): no
  `DisplacementMassIntegral_q` (MarkerBodyMass, LoadMassProportional refuse the user body; the rigid-body test applies
  gravity as LoadForceVector); no Acceleration, AngularAcceleration, Rotation, VelocityLocal, PotentialEnergy, beam
  outputs only as UserType0-3; mass matrix never constant, Jacobian numerical only; no AE, data or ODE1 coordinates;
  `UpdateGraphics` empty, `graphicsDataUserFunction(mbs, itemNumber)` in global coordinates without configuration, no
  visualization setting read; cable markers and GeneralContact cables check the C++ type; NodeUser reports
  Position+Orientation also without rotation user function. The tests carry reference data in globals, closures and a
  dict by item number.
- The considerations and recommendations are pythonUserItems.md section 5.11 (A-F). Nothing implemented.

<a id="rg7-12-3"></a>
### RG7.12.3 — qRef, q, q_t and parameters in every user function (2026-10-22, #3056)
*(Maintainer 2026-10-22: the revision looks better. Decisions: a) implement RG7.12 with qRef, q, q_t, parameters in every
user function; b) q_tt in the outputVariableUserFunction; c) further things like data can only be accessed via
MainSystem functions, probably added to symbolic in the future. Remarks: the outputVariableUserFunction is not limited
to UserType quantities - IfThenElse allows as many output variables as needed, so beam outputs and energy are possible;
a constant mass matrix with a local matrix storage and a flag as in the ANCF elements; algebraic equations, data
coordinates, ODE1 not for now; visualizationParameters, q_tt, data and what else is missing should be accessible via
mbs and itemNumber, at least for visualization - a placeholder now, or be sure that they can be accessed via mbs.)*
- Definitions: ObjectUserBody and NodeUser get `parameters` (vector, default empty); their user functions are
  `(mbs, t, itemNumber, [localPosition | outputVariableType, localPosition], qRef, q, q_t, [q_tt], parameters)`; the
  graphicsDataUserFunction of ObjectUserBody has its own signature `(mbs, t, itemNumber, qRef, q, q_t, parameters)`
  with the visualization configuration (it shared the one of ObjectGenericODE2, `(mbs, itemNumber)`). The std::function
  types, the symbolic signatures and the Protocols are generated from the definitions.
- C++: CObjectUserBody `GatherUserFunctionCoordinates` (q, q_t of the configuration, qRef of the reference
  configuration) and `GatherAccelerations` (zero in the reference configuration); the Evaluate functions pass qRef and
  q_t beside the varied q; CNodeUser `Velocities` (zero in the reference configuration, where GetCoordinateVector_t
  throws); VisualizationObjectUserBody::CallUserFunction gathers the visualization configuration and calls through
  `EvaluateUserFunction` (numpy views).
- Python: `ParameterLayout(entries)` with `Size`, `ToVector(dict)` (names and sizes checked) and `ToDict(vector)` (numpy
  and symbolic vectors; a symbolic vector argument has no size while recording, #3007); ConvertFunctionToSymbolic
  records `qRef`, `q_tt` with the size of `q` and `parameters` with the length of the item's parameters.
- Visualization parameters (remark 4): not added as an argument. A future `visualizationParameters` of
  VObjectUserBody is reachable through `mbs.GetObjectParameter(itemNumber, 'V...')`, as `'Vshow'` is today, which the
  graphicsDataUserFunction - Python only, it returns graphics data - can read; a later mapping function is a new user
  function with a signature of its own. Data coordinates and q_tt elsewhere: through MainSystem functions (decision c).
- The test models with user items (userRigidBodyTest, userANCFCable2DTest, userANCFBeam3DTest, userTetrahedronTest,
  userNodeTest, userTypesTest, deleteItemsTest) and test_itemCompatibility take the new arguments; their results are
  unchanged. New userParametersTest.py: two mass-spring oscillators, one recorded symbolic force and mass matrix for
  both parameter sets, bit-equal to the Python functions, 2e-7 from the exact solution; a stiffness changed with
  SetObjectParameter after recording acts; ParameterLayout refuses missing, unknown and wrongly sized entries.
- The build: `exudev build` fails at link (LNK1181 gdi32.lib) since the Windows SDK 10.0.26100.0 found by setuptools
  through Visual Studio 18 BuildTools has no `um` libraries; built with the vcvars64 of Visual Studio 2022 and SDK
  10.0.22621.0 and DISTUTILS_USE_SDK=1 (a scratch batch file) - the machine's toolchain, not the code.

<a id="rg7-12-4"></a>
### RG7.12.4 — the output variables of the user body (2026-10-22, #3057)
*(Maintainer 2026-10-22: why does the outputVariableUserFunction only accept UserType quantities? A misunderstanding: the
IfThenElse construct allows as many output variables as required, so beam outputs and energy are possible.)*
- Correct: the function gets `outputVariableType`, a symbolic function as a number, and IfThenElse selects per type; the
  limit was the list of output variables the body declared (UserType0-3). Now, with the function given, the body
  declares DisplacementLocal, VelocityLocal, Acceleration(Local), Rotation, AngularAcceleration(Local), Director1-3,
  Force(Local), Torque(Local), StrainLocal, StressLocal, CurvatureLocal, PotentialEnergy and UserType0-3 - all types
  it does not compute itself, except those that belong to connectors and joints - and routes them to the function. A
  type the function does not provide: it returns an empty list, which raises with the type named. The restriction of
  symbolic functions: one result size for all types (IfThenElse per component), as the recording fixes the size.
- IfThenElse takes symbolic Reals only: a constant branch is `esym.Real(0.)`.
- userOutputVariablesTest.py: a mass on a spring with strain, potential energy and acceleration (from q_tt), Python
  and symbolic (with IfThenElse), against x0 cos(omega t) and each other; SensorBody records PotentialEnergy; Rotation,
  which the Python function does not provide, is refused.

<a id="rg7-12-5"></a>
### RG7.12.5 — the constant mass matrix of the user body (2026-10-22, #3058)
*(Maintainer 2026-10-22: constant mass matrix - good idea, add that; in particular useful for some finite elements, with a
local matrix storage and a flag as in the ANCF elements.)*
- `constantMassMatrix` (False): `HasConstantMassMatrix` returns it; `ComputeMassMatrix` evaluates the
  massMatrixUserFunction once into `storedMassMatrix` with the local indices 0..n-1 - dense or sparse, as the function
  returns it - and adds it with the item's ltg on every call (`CopyOrAddTriplets`). `ParametersHaveChanged` and
  `PreAssemble` set `massMatrixComputed` false. A stored value, approved by the maintainer under D18, as
  `massMatrixComputed` / `precomputedMassMatrix` of the ANCF elements.
- userParametersTest.py: with `constantMassMatrix` the Python mass matrix function is called twice for two bodies in a
  solution of 5000 steps, and the solution is bit-equal; its result unchanged.

<a id="rg11-8-23"></a>
### RG11.8.23 — the Windows SDK of the build (2026-10-22, #3059)
*(Maintainer 2026-10-22: after a mandatory Windows update the build fails; VS 2026 uninstalled, VS 2022 updated, exudev
build still fails - please fix.)*
- The link failed with LNK1181 on `gdi32.lib`: `Windows Kits/10/Lib/10.0.26100.0/um/x64` was changed on 2026-10-21
  04:32 and has 478 libraries without `gdi32.lib`, `opengl32.lib` and `shell32.lib`; 10.0.22621.0 and 10.0.19041.0 are
  complete. setuptools runs vcvarsall.bat without an SDK version, which takes the newest.
- `commands.WindowsSdkEnvironment`: on Windows, if the newest SDK lacks one of the libraries setup.py links
  (`windowsSdkLibraries`), the environment of `vcvarsall.bat x64 <newest complete>` (found by vswhere) with
  `DISTUTILS_USE_SDK` and `MSSdk`, which setuptools then uses as it is; a printed line names both SDKs. Nothing when
  the newest is complete, on other platforms or when the caller set `DISTUTILS_USE_SDK`. test_exudev.py checks the
  choice on a fake SDK directory.
- Repairing 10.0.26100.0 (Visual Studio Installer, individual components) makes the build take it again.

<a id="rg7-12-6"></a>
### RG7.12.6 — q of the user functions: the displacement coordinates (2026-10-22, #3060)
*(Maintainer 2026-10-22: q should be the displacement coordinates, not the total coordinates, as internally in the C++
code, so results are also more comparable; for rigid bodies total coordinates would be preferable, but for the
time-consuming cases like finite elements the displacements are often equally relevant, and symbolic expressions can
easily add both. Remark 4 of the same day: the access of symbolic functions to the visualization settings stays a path
for later, without an issue - pythonUserItems.md 5.11 D.)*
- Estimate given before the change: no objection. Every C++ item computes with the displacements and reads the
  reference from its node; ObjectGenericODE2 gives its user functions the displacements as well, so all user functions
  are alike. Cost: a rigid body or a large-deformation element adds `qRef + q`, n additions per call - in a recorded
  symbolic function n nodes more, against the tens to hundreds of a mass matrix or a force; an element with linear
  strains saves the subtraction. The numerical derivatives inside the items (the derivative of J^T f in
  ObjectUserBody, of J_rot^T m and the acceleration in NodeUser) scale their step with the coordinates as
  `newton.numericalDifferentiation` does with its default `addReferenceCoordinatesToEpsilon=False`, i.e. now with the
  displacements; userRigidBodyTest.py moves by 1.4e-13 (13.475755021376248 -> 13.475755021376385), all other results
  unchanged.
- C++: `CObjectUserBody::GatherUserFunctionCoordinates` and the graphicsDataUserFunction call `GatherCoordinates`
  without the reference; `CNodeUser::Displacements` (zero in the reference configuration) replaces `TotalCoordinates`;
  the output variable CoordinatesTotal of NodeUser stays reference plus displacement.
- The descriptions of both items and their mini examples (`qRef + q`), the docstring of `ParameterLayout`, the test
  models deleteItemsTest, userANCFBeam3DTest, userANCFCable2DTest, userNodeTest, userOutputVariablesTest,
  userParametersTest, userRigidBodyTest, userTetrahedronTest, userTypesTest, and test_graphicsData /
  test_itemCompatibility; pythonUserItems.md 5.9 and 5.11 (A, C, D, F with the decisions of the day),
  userFunctionBoundary.md (13), revisions.md.
- Not checked: the AVX2 reference of userRigidBodyTest in `AVX2ReferenceSolutionUpdate()` (fast module not built).

<a id="rg7-12-7"></a>
### RG7.12.7 — one-coordinate nodes in the symbolic recording; the cost of the arguments (2026-10-22, #3061)
*(Maintainer 2026-10-22: update pythonUserItems.md, which still mentions total coordinates, and continue with the
planned changes of the user body.)*
- The decided changes of the user body were all done (RG7.12.3-RG7.12.6). What 5.10 left open was the measured cost of
  the extra arguments. measureUserFunctions.py got two variants: one ObjectUserBody per mass point with a Python force
  `(mbs, t, itemNumber, qRef, q, q_t, parameters)` and a stored mass matrix, and the same with one symbolic force
  recorded for all 100 bodies. Two runs, 1.13.166.dev1: Python 1.175 / 1.162 s, 0.97 us per call more than the C++
  spring and 0.33 us more than ObjectGenericODE2 per mass point `(mbs, t, itemNumber, q, q_t)` (0.840 / 0.832 s) - two
  more numpy views and the gathering of the coordinates by the item; symbolic 0.251 s, 0.06 us per call more than C++.
- The symbolic variant failed: `_ObjectCoordinatesSize` took `len()` of the node output Coordinates, a float for a
  node with one coordinate (NodeGenericODE2 of size 1), so CreateSymbolicUserFunction raised a TypeError for every item
  with such a node. Now `np.size`. userParametersTest.py records a body on a NodeGenericODE2 of one coordinate and
  compares with the Python function; its result unchanged.
- pythonUserItems.md 5.10 and 5.11 rewritten to say what is - the parameters, `ParameterLayout`, the arguments with
  `q` the displacements, no visualizationParameters argument, symbolic access to the visualization settings a later
  path - with the measurement, also in the table of section 7; the gaps of E kept as the list for later.

<a id="rg7-12-8"></a>
### RG7.12.8 — the total coordinates as qTotal (2026-10-22, #3062)
*(Maintainer 2026-10-22: looking at the user body examples, almost all write q + qRef, as they need total coordinates;
the other case seems rare, and future changes in C++ may switch to total coordinates for performance. Revising the last
decision: use total coordinates in the user item user functions, but denote them as qTotal to make that clear; for
complex functions they can be assigned to q inside the function to keep expressions short, but the interface shall be
clear.)*
- The C++ and the test models of #3060 reverted (`git revert --no-commit` of that commit, the plan, log, tracker and
  version files kept): `GatherUserFunctionCoordinates` and the graphicsDataUserFunction gather with the reference,
  `CNodeUser::TotalCoordinates` again; userRigidBodyTest.py has its earlier result 13.475755021376248 again.
- The argument `q` of every user function of ObjectUserBody and NodeUser is `qTotal` - in the defs of the definition
  files, hence in the Protocols, the argument tables and the symbolic signatures; ConvertFunctionToSymbolic sizes
  `qTotal` like `q` (by the name in the definition). The user function itself may call it anything, as the recording
  passes the arguments by the names of the user's function.
- The user functions of the mini examples, the test models (deleteItemsTest, userANCFBeam3DTest, userANCFCable2DTest,
  userNodeTest, userOutputVariablesTest, userParametersTest, userRigidBodyTest, userTetrahedronTest, userTypesTest),
  test_graphicsData, test_itemCompatibility and measureUserFunctions.py take `qTotal`; a function that uses it more than
  twice starts with `q = qTotal`. Results unchanged.
- pythonUserItems.md 5.9, 5.10, 5.11 A and F, userFunctionBoundary.md (13), revisions.md and the ParameterLayout
  docstring say `qTotal`.

<a id="rg7-13"></a>
### RG7.13 — the mass integral of the user body (2026-10-23, #3063)
*(Maintainer 2026-10-23: do RG7.9, the mass-integral access, RG7.5 and the graphics data drawn with the
positionUserFunction.)*
- The access function behind MarkerBodyMass is `CObjectBody::GetMassWeightedPositionJacobian(Matrix&)`, 3 x n,
  int(rho J_pos dV); MarkerBodyMass puts it into the position Jacobian of its marker data, and the load of type
  ForcePerMass (LoadMassProportional) adds its transpose times the load vector (CSystem::ComputeODE2SingleLoad). The
  generator derives DisplacementMassIntegral_q from `ItemFunctionDef('GetMassWeightedPositionJacobian')`.
- ObjectUserBody: `massIntegralUserFunction(mbs, t, itemNumber, qRef, qTotal, q_t, parameters)` returns the 3 x n
  matrix, checked for its size; without it, a MarkerBodyMass or a load on it raises a ValueError that names the
  function. `centerOfMass` (local position, default [0,0,0]) is what GetLocalCenterOfMass gives, where MarkerBodyMass
  and LoadMassProportional are drawn.
- Not changed: Assemble does not check that the body of a MarkerBodyMass provides DisplacementMassIntegral_q
  (CSystem.cpp near the Position/Orientation checks); for the user body the evaluation reports it.
- userMassIntegralTest.py: (1) a cantilever of four user ANCF cable elements under gravity, the mass integral the
  integral of the shape functions times rhoA, against ObjectANCFCable2D: 5.4e-14; (2) a mass point as a user body
  against MassPoint, with a Python and a symbolic mass integral: equal, and the exact parabola; (3) without the
  function, refused.
- pythonUserItems.md: the line of the table in 5.11 E had been written in a wrong encoding by a PowerShell edit of
  #3062 (`∫ρ` as `âˆ«Ï`); restored.

<a id="rg7-14"></a>
### RG7.14 — graphics data of the user body, mapped by the positionUserFunction (2026-10-23, #3064)
*(Maintainer 2026-10-23: the graphics data list which is drawn using the transformation from positionUserFunction,
together with a "show" visualization flag.)*
- VObjectUserBody gets `graphicsData` (TBodyGraphicsData, as VObjectRigidBody); the flag is the `show` the item already
  has, which switches the whole body - no second flag. `VisualizationObjectUserBody::UpdateGraphics`, which runs in
  the main thread for the user body (#3003), gathers the coordinates of the visualization configuration once and maps
  every point with `EvaluatePosition` - one call of the positionUserFunction per point, no point kept between redraws.
- `EXUvis::AddBodyGraphicsDataMapped(bodyGraphicsData, graphicsData, map, itemID)` (VisualizationPrimitives.cpp): lines,
  circles, spheres, lines3 and texts by their points (radii kept, texts copied), triangles and textured triangles with
  the normals computed from the mapped corners (flat - the triangle list keeps no connectivity), triangles6 with
  `hasNormals=false`, so that the split computes the normals of the mapped geometry. The item number is drawn at the
  mapped local origin with `bodies.showNumbers`.
- userGraphicsDataTest.py: (1) a rigid body as user body draws a brick, a sphere and lines where ObjectRigidBody draws
  the same graphics data, points within 6e-8 and normals within 2e-7 (float); (2) a bent user ANCF cable element maps a
  line along its axis to the positions the body gives (2e-8) and a strip across it with normals in the plane of
  bending, normal to the axis; (3) a symbolic positionUserFunction draws the same; (4) `show=False` draws nothing.

<a id="rg7-5-1"></a>
### RG7.5.1 — ObjectConnectorUser and exudyn.ConnectorMarker (2026-10-23, #2975)
*(Maintainer 2026-10-23: do RG7.9, the mass-integral access, RG7.5 and the graphics data drawn with the
positionUserFunction.)*
- The connector: any number of markers of any type (`RequestedNumberOfMarkers` 0, no requested type), `parameters`,
  `activeConnector`, `forceUserFunction(mbs, t, itemNumber, markers, parameters) -> None`. Through the legacy path
  `ComputeODE2LHS(ode2Lhs, markerData)`, as ObjectConnectorReevingSystemSprings - the connector interface of #2745 has
  two markers. The block of each marker in ode2Lhs is the length of its LTG (`ComputeMarkerODE2LTGarray`), as CSystem
  composes the coordinates of a connector. `GetAvailableJacobians` ODE2_ODE2 + ODE2_ODE2_t without `_function`: the
  Jacobian is numerical, the user function called for each perturbed coordinate. HasUserFunction: serial.
- `exudyn.ConnectorMarker` (src/Pymodules/PyConnectorMarker.h, bound in pybindDataStructures.py, no constructor from
  Python): created per marker and call by py::cast of a copy - owned by Python, so a script that keeps one holds an
  object, not a dangling pointer - and invalidated after the call, also when the function raises; then every function
  raises ("valid during that call only"). Reads: GetPosition/GetVelocity (Position), GetRotationMatrix/
  GetAngularVelocity/GetAngularVelocityLocal (Orientation), GetCoordinates(_t) (Coordinate(s)), GetBodyPosition/
  GetBodyVelocity(localPosition) (Body: the body of the marker, through CObjectBody::GetPosition/GetVelocity), the
  numbers. Applies: ApplyForce (J_pos^T f), ApplyTorque (J_rot^T m), ApplyBodyForce(force, localPosition) (the body's
  GetPositionJacobian at localPosition), ApplyCoordinateForces (J^T f); a force acts as a load, i.e. it is subtracted
  from the left-hand side. A function the marker does not provide raises with the marker and its type.
- Generators: the annotations `ConnectorMarkerList` (py::object, not symbolic) and `None` (void) for the def.
- Drawing: nothing, the number at the first marker with `connectors.showNumbers`. No output variables.
- userConnectorTest.py: (1) a Cartesian spring-damper between two mass points against CartesianSpringDamper: 1.6e-15;
  (2) a rigid body with a rotational damper (ApplyTorque) and a spring at a body point (ApplyBodyForce) against
  RigidBodySpringDamper and CartesianSpringDamper: 7.8e-13; (3) a ball resting on a pinned/roller ANCF cable of four
  elements, the contact at the closest point of the closest element (golden section, then Newton on the normality -
  a coarser point made the numerical Jacobian too noisy for the static solver), static with the whole weight from the
  start (`useLoadFactor=False`, as a part of the weight lifts the ball off): the contact carries the weight to 1e-12,
  the cable deflects as under a point load (3.4e-6 of 0.081); (4) ApplyTorque on a position marker and a ConnectorMarker
  used after the call are refused. The ball above a node touched both elements first and was counted twice - the
  contact takes the closest element only.
- Not done: the user markers - split off as #3065 (RG7.5.2), with the question what they add.

<a id="rg7-14-1"></a>
### RG7.14.1 — graphics of the user body test models (2026-10-23, #3067)
*(Maintainer 2026-10-23: when you added the graphics data that is mapped with the positionUserFunction, also add some
useful graphics to the existing user object body examples (beams, rigid body, ...) and adapt the test such that it runs
with the graphics in case it is not executed in test mode, and also writing the solution file in that case, so the
solution viewer will open afterwards to check the graphics.)*
- graphicsData in local coordinates, mapped by the positionUserFunction: userRigidBodyTest a brick of the inertia
  cuboid with its frame (shown for the symbolic body, so it draws without Python); userANCFBeam3DTest each element as
  four bricks of the cross section (shown for 8 elements, static); userTetrahedronTest the four faces of each
  tetrahedron, shrunk to 90 % about its center so that the six are seen; userANCFCable2DTest and userMassIntegralTest
  the axis of each element as 8 line segments (the positionUserFunction of the 2D cable gives the axis only);
  userParametersTest and userOutputVariablesTest the mass points as spheres.
- Run alone (`exu.sys['testIsActive']` not set), the solve that is shown writes the solution file, runs with the
  renderer and then opens the SolutionViewer, as the other test models do; in a test run nothing changes - all seven
  results are the same, also run alone with the windows suppressed.

<a id="rg7-9"></a>
### RG7.9 — the manual page of the user items (2026-10-23, #2979)
*(Maintainer 2026-10-23: do RG7.9, the mass-integral access, RG7.5 and the graphics data drawn with the
positionUserFunction.)*
- How each kind of item works conceptually was written meanwhile as the general pages of the item reference
  (revision2026b step RG13.5, definitions/itemKindDefinitions.py: nodeGeneral, objectBodyGeneral, ...). The new page
  docs/manual/userItems.md links them instead of repeating them and covers what is particular to user items: the
  table of user items (ObjectUserBody, NodeUser, ObjectConnectorUser, the user functions of the built-in items), the
  arguments `(mbs, t, itemNumber, [...], qRef, qTotal, q_t, parameters)`, ParameterLayout, symbolic functions with one
  recording per kind and their structural limit, the measured cost, what a user body provides (equations,
  constantMassMatrix, the body interface, the mass integral, output variables, graphicsData), the ConnectorMarker of
  ObjectConnectorUser, an example - two mass points of one kind with parameters, gravity through the mass integral,
  spheres, one symbolic recording; run before it was written - and the test models as complete examples.
- In the manual after "Advanced topics" (index.md, pdfIndex.md); linked from the user functions of the basics.

<a id="rg7-12-9"></a>
### RG7.12.9 — the parameters in the user rigid body and the user ANCF cable (2026-10-23, #3068)
*(Maintainer 2026-10-23: I see that the user objects include the parameters, but do not use them. So please, add the
parameters at least to the interface in user rigid body and user ANCFCable2D.)*
- userRigidBodyTest.py: `ParameterLayout([('mass', 1), ('inertia', 6)])`, the inertia in Exudyn's order
  [Jxx, Jyy, Jzz, Jyz, Jxz, Jxy] and turned into the tensor by `InertiaTensor`, which serves numbers and symbolic
  components; the Python mass matrix and force and the symbolic mass matrix and force read them, ForceComponents gets
  the tensor as an argument; every user body of the model, also the one of the refusal, gets `bodyParameters`.
- userANCFCable2DTest.py: `ParameterLayout([('length', 1), ('rhoA', 1), ('EA', 1), ('EI', 1)])`; the shape functions
  and the Gauss rules take the length; the mass matrix is computed by its user function from the parameters, once per
  element with `constantMassMatrix=True`, instead of a global matrix of the script.
- Both results are bit-equal to their references (13.475755021376248, 7.797666638067246).

<a id="rg12-47-1"></a>
### RG12.47.1 — rotation utilities for symbolic user items: the analysis (2026-10-23, #3069)
*(Maintainer 2026-10-23: regarding cUtilities / rigidBodyUtilities: make a selected list of functions that are useful
for symbolic user items; add symbolicUtilities.py in exudyn/config; each function shall have the same name and the same
args, except for "functions" as last arg, e.g. RotationVector2GLocal(rotationVector, functions=exudyn.symbolic),
RotXYZ2RotationMatrix(rot, functions=exudyn.symbolic); exudyn.symbolic could be replaced with exudyn.cMath, numpy or
math in most cases as well; also check what would fail or what is missing first; this should simplify the example
scripts when using functions=exudyn.symbolic; adapt the test scripts; add a step and issue, the first substep the
analysis and recommendation - do the analysis now.)*
- docs/dev/design/symbolicUtilities.md: the 30 functions of cUtilities and what rigidBodyUtilities adds; the math
  names the four modules share (sin ... pow; not abs/sign/min/max in math and cMath, no pi in exudyn.symbolic);
  measured: a function written once equals cUtilities with math, cMath and numpy and recorded with exudyn.symbolic
  (0.0); cMath and math refuse symbolic values; numpy's functions record them; IfThenElse needs symbolic branches,
  ignores the 0/0 of the unused one; no cross product of symbolic vectors; numeric cost 0.63 us (cUtilities) against
  1.2-2.4 us.
- Recommended: `exudyn.symbolicUtilities` (exudyn.config is the module of the runtime settings - to confirm), 21
  forward functions (Skew, Cross, Inertia6D2InertiaTensor, RotationMatrixX/Y/Z, RotationMatrix2D, the RotXYZ,
  Euler-parameter and rotation-vector functions with their G matrices and derivatives), no inverse conversions;
  `functions=exudyn.symbolic` last, the helpers _Matrix, _IfThenElse, _Pi; a test model against cUtilities; then
  userRigidBodyTest.py and userANCFBeam3DTest.py use it. Missing in exudyn.symbolic, each an issue if wanted: cross,
  pi, IfThenElse with a number.

<a id="rg12-47-2"></a>
### RG12.47.2 and RG12.47.3 — exudyn.symbolicUtilities and its use in userRigidBodyTest.py (2026-10-23, #3069)
*(Maintainer 2026-10-23, on the analysis: functions=math or cMath would also work, using functions.sin inside the
function; it is about implementing e.g. RotationMatrixX for symbolic, numpy and exudyn.cMath, the result a numpy matrix,
only for symbolic a symbolic matrix; cMath is only cosmetic, cUtilities would be faster. python/exudyn/symbolicUtilities.py:
yes. The gaps of exudyn.symbolic get issues: yes. The list: as proposed.)*
- python/exudyn/symbolicUtilities.py: the 21 functions of the analysis with the names and arguments of cUtilities /
  rigidBodyUtilities and `functions=exudyn.symbolic` last; `_Matrix` / `_Vector` return exudyn.symbolic.Matrix / Vector
  for exudyn.symbolic and float numpy arrays otherwise; RotationVector2RotationMatrix uses the limits 1 and 1/2 of its
  factors below an angle of 1e-12, by IfThenElse for a symbolic angle. In the utility documentation
  (utilityDocsModel.py), the module list of the introduction and the manual page of the user items.
- python/TestModels/symbolicUtilitiesTest.py (reference 7.024076141931356): every function at 5 random arguments with
  numpy, math and cMath and recorded once of named symbolic variables, evaluated after SetValue / SetVector, against
  cUtilities and rigidBodyUtilities: largest difference 6.7e-16; the result types; the rotation vector at length 0 and
  1e-13: difference 0.
- userRigidBodyTest.py: the symbolic user functions with su.RotXYZ2GLocal, RotXYZ2GLocal_t, RotXYZ2RotationMatrix, Skew,
  Cross and Inertia6D2InertiaTensor on symbolic vectors with @ and .T; its own component functions are gone (282 to 237
  lines). ForceTorques and Position, with functions=cMath, equal the Python user functions (difference 0.0); the result
  13.475755021376248 is unchanged. userANCFBeam3DTest.py and the other user item scripts have no rotation functions.
- Found while testing: slicing a symbolic vector gives a symbolic vector (the analysis said a list; corrected in the
  design document); cUtilities.RotationVector2RotationMatrix([7, 0, 0]) is wrong (#3073, RG12.47.7). Raised as asked:
  #3070, #3071, #3072 (RG12.47.4-6).

<a id="rg12-47-4"></a>
### RG12.47.4 to RG12.47.7 — cross, pi, IfThenElse with numbers in exudyn.symbolic; RotationVector2RotationMatrix beyond 2 pi (2026-10-24, #3070-#3073)
*(Maintainer 2026-10-24: then do the open RG12.47 steps.)*
- src/Linalg/Symbolic.cpp: `cross(a, b)` in the module, of two symbolic vectors or one of them a list / numpy array,
  recorded as the three component expressions; a vector of another size than 3 raises a ValueError (#3070).
  `m.attr("pi") = EXUstd::pi` (#3071). IfThenElse bound for the eight combinations of SReal and Real of the
  condition and the branches - the template took numbers already (#3072). Documented in definitions/pybindSymbolic.py
  with their stubs.
- src/Linalg/RigidBodyMath.h: RotationVector2RotationMatrix reduced the angle by fmod(angle, 2 pi) and kept the
  vector, so the factors sin(a)/a and 2 sin(a/2)^2/a^2 belonged to another angle; the reduction is removed (#3073).
  The Lie group nodes call the function as well; their test models give the same results (rotations below 2 pi).
- Tests: symbolicModuleTest.py (cross of the three kinds, recorded and after SetVector, the refusal; pi; IfThenElse
  with numbers, also recorded), cUtilitiesTest.py (rotation vectors of length 7 and 13 against RotationMatrixX and
  ExpSO3); results unchanged.

<a id="rg12-48"></a>
### RG12.48 — the mathematical functions of exudyn.symbolic, Diff tested, atan2 Diff; the TestModels sorted (2026-10-24, #3074-#3077)
*(Maintainer 2026-10-24: the symbolic part lists isfinite(x): according to specification of C++ std::isfinite and so
on - write it shorter; the interface and the behaviour are as std::, but they do not call std:: any more, they are
reimplemented with musl - clarify. Diff says (UNTESTED!) - if untested, add tests and remove UNTESTED. The models in
TestModels are not sorted alphabetically, as in Examples.)*
- Checked in src/Linalg/Symbolic.h what computes each function: exp, log, pow, sin, cos, tan, asin, acos, atan, atan2,
  sinh, cosh, tanh call EM:: (musl, as exudyn.cMath); sqrt, fabs, round, ceil, floor, fmod, min, max, isfinite std::
  (exact); asinh, acosh, atanh std:: of the platform (not in cMath). definitions/pybindSymbolic.py: one paragraph with
  the names and these three groups instead of 20 entries "according to specification of C++ std::..."; the functions
  keep their stubs (addDocu=False), with one line each; atan2, which was bound but missing from the documentation and
  the stubs, documented; sign said 1 for x>1, now x>0 (#3074).
- Diff was tested since #2479 for x^3, sin cos, exp/x, sqrt and pow. symbolicModuleTest.py now checks 32 expressions -
  every operator and function with a derivative, IfThenElse in both branches, atan2 in either argument - against a
  central difference, and the 14 functions and comparisons without one raise "not possible"; UNTESTED removed (#3075).
  Found: atan2(y, x).Diff returned (y x' - x y')/(x^2+y^2), the negative; corrected (#3076). The test result is
  unchanged (the checks count into cntWrong).
- tools/generators/examplesDocsEmitter.py: the TestModels sorted as the Examples, case-insensitive (#3077).
- Seen, not changed: a Diff that is not possible raises exudyn.InternalError, not a ValueError.

<a id="rg5-6"></a>
### RG5.6.1 and RG5.6.2 — the machine-readable performance log and the performance history (2026-10-24, #3078, #3079)
*(Maintainer 2026-10-24: we do not have a good tracking of performance, neither human- nor machine-readable.
performanceLogs could include a machine-readable section - general data like the testing machine and its load during
the test, then the timing for each single test, even with subtimers - as JSON section or a separate file. From that an
exudev feature could create a data summary for many Exudyn versions, and from that a table and plot of the main test
timings over time; the summary, where tests evolve and disappear, stored in the repo, and a new performance test added
not for every micro version but when expecting changes, before and after performance improvements. The adapted suite
could be run for older versions, not older than 1.11, in a tolerant mode recording the compatible models; older
results are in the text of the logs of 1.9.0, 1.10.0, ... Compare the fast module, and in the regular module the
timers per model, which slow them down a little but show effects much more clearly.)*
- The JSON goes INTO the log, after a line `#JSON`, as runMiniExamplePerformance.py does (#2745) - one file per run, and
  one parser for both. It is printed by exu.Print before the C++ writer closes the log: a Python append was overtaken
  by the writer's buffer. Contents: version, module, AVX2, build date; machine id (EXUDYN_MACHINE_ID), CPU, system,
  Python, numpy; the date; the CPU load before and after (mean and maximum of the threads); every test file with group,
  wall time, result and whether it passed; every single run with solver time, result, error, passed and the solver
  sub-timers; the totals of the groups.
- testRunnerTools.AddTiming records the sub-timers that are not zero when timer.useTimer is set
  (solverSubTimerNames). perfConnectorInterface, perfAccessFunctionsAD, perfUserFunctions and perfLargeMassSpringChain
  (which had it off) now set show.computationTime = True, as the six others did: all 45 runs of the regular module
  have sub-timers, the fast module none (compiled away). The total was 45.6 s regular, 36.5 s fast (load before 0.6 %).
- python/testing/performanceHistory.py: `add LOG ...` reads a log - the JSON section, or for an older log the CPU TIME
  of each passed test file and, since #2460, the table of the single runs - and adds or replaces the entry of its series
  (machine, system, Python, module) and version in python/logs/performance/performanceHistory.json; `report` writes
  performanceHistory.md, per series the test files and the single runs over the versions (missing tests empty), the
  sub-timers of the two newest runs with timers, versions with a CPU load above 20 % marked, and a plot of the file
  times (matplotlib, Agg). The machine of a log without EXUDYN_MACHINE_ID is its processor string, in old and new logs
  alike. exudev: `perf --history-add LOG ...` (paths relative to python/logs/performance) and `perf --history-report`.
- Seeded with the Windows Python 3.13 logs of this machine: 1.9.0, 1.10.0, 1.11.0, 1.13.6 (regular and fast, load 35 %
  before the regular run) and 1.13.183.dev1 (regular and fast, new runs). python/testing/test_performanceHistory.py.
- Open (RG5.6.3, #3080): the tolerant mode for older versions needs environments with exudyn 1.11 ... from PyPI.

<a id="rg5-6-3"></a>
### RG5.6.3 — the tolerant mode for older versions, the history filled (2026-10-24, #3080)
*(Maintainer 2026-10-24: I put exudyn-1.12.0.dev1-cp313-cp313-win_amd64.whl into tmp and you can install 1.11.0 via pip
install. Run the tests in venvP313. The other results shall be taken from the performance results (text files) in
logs/performance. If you see important versions to test, many 3.13 wheels are in dist. Only the regular module.)*
- The first tolerant run with 1.11.0 failed in every model: the models use the names of 1.12.256 and later (`mass`,
  `itemNumbers`, `simulationSettings.solution.file.write`, `show.computationTime`, `linearSolver.solverType`,
  `timeIntegration.explicit`). python/testing/performanceCompatibility.py, used by `--tolerant` for a version before
  1.12.256: the item classes and Create functions of the installed version (38) are wrapped to take the new argument
  names, read from the docstrings of the repository's itemInterface.py and mainSystemExtensions.py; the settings
  paths of the models are replaced in their source by the old ones (a table of 15, from the deprecated entries of
  structureDefsSimulationSettings.py). OutputFilePath, missing before #2500, falls back to the given name.
- 1.11.0 (pip, PyPI) and 1.12.0.dev1 (the wheel in tmp) in venvP313: 8 of 10 models run and give today's results
  bit-equal (FINISHED SUCCESSFUL), which also checks the translation; perfUserFunctions (symbolic user functions with
  vectors, later) and perfAccessFunctionsAD (raises in SolveDynamic) are skipped, the runs before the exception kept.
  Totals of the 8 files: 45.3 s (1.11.0); 1.13.183: 45.6 s for all 10. venvP313 has the current build again.
- The history: all Windows Python 3.13 logs of python/logs/performance read from their text (1.9.0, 1.10.0, 1.11.0,
  1.13.6 regular and fast, 1.13.42 ... 1.13.174, 15 versions), the JSON of 1.13.183 regular and fast, and the two
  tolerant runs, which replace the text entries of 1.11.0; the Linux logs of 1.10.0 and 1.11.35 as a series of their
  own: 24 entries in 3 series. No dist wheel was run: the text logs cover the 1.13 versions.
- The dev logs of 1.13.42-1.13.174 scatter by up to 30 % (13.7-15.4 s for perfLargeMassSpringChain between versions
  without a change in it); several were taken with load and without a load record that the report could mark.

*2026-10-24, RG5.6.4 (#3081; maintainer: as the data is recorded now, remove performanceCompatibility.py again):*
removed, with its use in runPerformanceTests.py, its test in test_performanceHistory.py and the paragraph of WORKFLOW.md;
`--tolerant` stays. The entries of 1.11.0 and 1.12.0.dev1 in performanceHistory.json are kept.

<a id="rg7-5-2"></a>
### RG7.5.2, RG7.6 and RG7.8.1-2 — MarkerUser, ObjectJointUser, the symbolic user items in parallel analysed (2026-10-24, #3065, #2976, #2978)
*(Maintainer 2026-10-24: do the open RG7 tasks now.)*
- RG7.5.2, the decision: what a user marker adds beyond ObjectConnectorUser (reaches any point of its markers' bodies)
  and ObjectUserBody (defines its local positions) is a point of SEVERAL bodies where the built-in connectors, joints,
  loads and sensors act. MarkerUser (itemDefsMarkers.py, CMarkerUser.cpp): `bodyNumbers`, `parameters`,
  positionUserFunction, positionJacobianUserFunction, rotationMatrixUserFunction, rotationJacobianUserFunction with
  `(mbs, t, itemNumber, qRef, qTotal, q_t, parameters)`; the coordinates of the nodes of the bodies body by body (the
  order of ComputeMarkerODE2LTGarray for a multi-object marker, CObjectBody::GatherCoordinates); the Jacobians by central
  differences as NodeUser; GetType has Orientation only with a rotationMatrixUserFunction (ItemTypes got an
  `implementation` for types that depend on parameters), so a joint on a marker without it is refused at Assemble; no
  JacobianDerivativeAvailable, so connectors on it compute their Jacobian numerically; CMarker::AccessFunctionsCallUserFunctions
  draws it and what depends on it in the main thread (VisualizationSystem). Found while testing: with the numerical
  Jacobians a Newton tolerance of 1e-12 is not reached in every step, the step size is reduced and the result differs
  by 1e-4; with the default tolerances the variants agree to 1e-10 - said on the manual page.
- RG7.6: ObjectJointUser (itemDefsObjects.py, CObjectJointUser.cpp) on the connector interface of rigid markers (#2745):
  ComputeConstraintEquationsRigid calls the constraintUserFunction with the positions and rotation matrices, at velocity
  level C [v0, omega0, v1, omega1] + dc/dt by central differences in t; ComputeConstraintEquationsRigidDiff takes the
  increments of each AutoDiff direction from the seeded markers (skew(dtheta) = dA A^T) and multiplies them with
  C = dc/d[p0, theta0, p1, theta1] from the jacobianUserFunction or 24 central differences; numberOfConstraints 1-6,
  checked at Assemble with the constraintUserFunction; outputs Position, Displacement, Force (the multipliers),
  ConstraintEquation.
- userJointTest.py (-1.8272301913592162): a screw joint (5 equations) against phi = (M + F h) t^2/(2 (Jxx + m h^2)),
  2e-7 at index 3, 9e-8 at index 2 (Newmark); a spherical joint without and with jacobianUserFunction against
  ObjectJointSpherical, 1.6e-11; the refusals. userMarkerTest.py (-0.3569348653534026): the midpoint of two mass points
  with spring and force against two mass points with half of each, 8e-11, the SensorMarker; a pendulum on a rigid
  MarkerUser written with exudyn.symbolicUtilities and numpy against MarkerBodyRigid, 3e-10; the refusals. Mini examples
  of both items.
- RG7.8: the graphics part is #3064. The parallel part needs the symbolic engine first: PySymbolicUserFunction copies the
  arguments into its named variables (argRealList), FlatProgram keeps the values in `slots`, and vector and matrix
  expressions return references to buffers inside their nodes; one recording serves every item of a kind (#3056), so
  two threads would overwrite each other. Planned as RG7.8.3.

<a id="rg11-11"></a>
### RG11.8.26 and RG11.11.1 — the design notes out of the docs; proposed issues (2026-10-24, #3082, #3083)
*(Maintainer 2026-10-24: the docs/dev/design documents went into the docs - not intended, they contain planning, not
the current state; mention them once in the development section, like the plans and logs of the revision folder. And:
contributors should be able to propose issues; they cannot raise them, as that interferes with the numbers; a raise
folder, whose issues the maintainer's next raise raises first, with a command like "exudev issue add-proposed" that
also works alone; a similar mechanism for merging, as versions depend on the resolved dates - suggest one.)*
- conf.py excludes `docs/dev/design/*`; docs/dev/README.md lists the folder in one row (a GitHub link, as the revision
  files) instead of seven rows and seven toctree entries. Nothing published linked them.
- issueStore.ProposedDirectory(); issueTracker.ProposeIssue (checked as RaiseIssue, without a number; file
  `<YYYYMMDD-HHMMSS>-<author>-<title>.json`), ProposedIssues, AddProposedIssues (in the order of the names, the proposer
  as author, "(proposed DATE by NAME)" appended, the file removed); RaiseIssue calls AddProposedIssues first
  (addProposedFirst). exudev: `issue propose` and `issue add-proposed`. test_issueTracker.py
  testProposedIssuesAreRaisedFirst.
- The merging mechanism is written as a proposal in docs/dev/design/contributions.md section 3 - proposed resolutions,
  `apply-proposed` after a merge in the maintainer's sequence, a check of the contribution's changed files - and waits
  for the decision (RG11.11.2).

<a id="rg11-11-2"></a>
### RG11.11.2 — proposed resolutions, apply-proposed, the check of a contribution (2026-10-25, #3083)
*(Maintainer 2026-10-25: you mentioned "the maintainer as resolvedAuthor" - the original author who proposed to raise or
resolve the issue shall appear as author of the raised or resolved issue, he/she did the work and this is the main
thing. Resolution date = date of merge, otherwise it would not work. apply-proposed stays a deliberate command: there
should be a warning when resolving an issue saying that there are proposed ones and how to apply.)*
- issueTracker: ProposeResolution (an open issue by number, or a proposed issue by its file name; file
  `issues/proposed/resolutions/<date>-<author>-<issue>.json`), ProposedResolutions, ApplyProposed (AddProposedIssuesByName,
  then the resolutions in ascending number with ResolveIssue(author = the proposer), the date of now; one that is no
  longer open is reported and kept), ProposalsWaitingNote, printed by every ResolveIssue while proposals wait
  (noteProposals). The raised proposal keeps its proposer as author (RG11.11.1).
- exudev: `issue propose-resolution`, `issue apply-proposed`. tools/checkIssues.py `--contribution BASE` with the list
  contributionForbidden; checked against the last commit, which lists the version files it changed.
- Tests in test_issueTracker.py. WORKFLOW.md, CONTRIBUTING.md, the exudev README and docs/dev/design/contributions.md
  describe it. A GitHub workflow step for pull requests is not added (.github/workflows/ - to ask).

<a id="rg11-11-3"></a>
### RG11.11.3 — the check of a contribution as a GitHub workflow (2026-10-25, #3084)
*(Maintainer 2026-10-25: yes, add a workflow step under .github/workflows/; I assume that it is described in the docs.)*
- `.github/workflows/contribution.yml`: on `pull_request`, only when the head repository is not this one (a fork - the
  maintainer's own branches resolve issues); checkout with the full history, Python 3.x, `python tools/checkIssues.py
  --contribution origin/<base>`; the actions at the majors of the other two workflows. checkIssues.py needs only the
  standard library.
- Described in CONTRIBUTING.md, WORKFLOW.md (the paragraph of proposed resolutions), the CI table of RELEASE.md and
  docs/dev/design/contributions.md section 3.4.
- Not run on GitHub from here (no push); the check itself is tested by test_issueTracker.py.

<a id="rg12-49"></a>
### RG12.49.1 — simplification and merging of the flat program, evaluated (2026-10-25, #3085)
*(Maintainer 2026-10-25: would a basic simplify (x*0, etc.) help, in particular when computing the flat program? a+0 and
a*0 often appear. Pure numbers could also be optimized, like sin(Real(13)). If we add sin(q0) two times, they would
become independent leaves; compare leaves up to a certain depth - write a concept and whether it impacts the current
implementation a lot. Is cos(rot[2]) written twice efficient with the flat evaluation?)*
- Measured with GetFlatProgram() of 1.13.190: the matrix of the question 11 operations, 3 duplicates (8 with the values
  shared by hand); a numpy object product of a constant 3x3 matrix 16 operations, 7 trivial, 3 duplicates; as a Python
  sum 19, 10, 4; userRigidBodyTest.py 314 operations, 0 trivial, 63 duplicates. `esym.sin(esym.Real(13))*q[2]` is
  recorded as `(0.420167 * q[2])`: an operation of numbers is computed while recording.
- Proposal in docs/dev/design/symbolicFlatProgramOptimization.md: in FlatProgram::Build, a key of class and operand
  slots (the operands are merged first, so one level compares any depth), numbers by bits, variables and arguments by
  pointer, commutative operands sorted - the same bits as the tree; the exact rules and `0*x` → 0 (which loses a nan
  of x), a switch to turn it off. No change to the expression classes beyond two virtual functions with defaults.
  Waiting for the decision: where, and `0*x`.

<a id="rg12-50"></a>
### RG12.50.1 — the flat program of expressions, the export and the symbolic model of a system, evaluated (2026-10-25, #3086)
*(Maintainer 2026-10-25: GetFlatProgram() - could it be available for symbolic expressions, or would we need functions
in symbolic more generally? Export the evaluation tree, for the symbolic code of a model of user items - a symbolic
system assembly in symbolicUtilities.py, interesting for control people; as text or another format; to sympy?)*
- docs/dev/design/symbolicExport.md: FlatProgram::Build takes any roots, so `esym.GetFlatProgram(expressions)` is a
  binding; the export as a Python structure (operations with names and operand slots, leaves with values or names),
  writers to Python source and to sympy (optional), a round-trip test; `SymbolicSystem(mbs)` renames the argument
  leaves by the local-to-global maps and adds mass matrices, forces and constraints; the marker Jacobians of the
  connectors are computed in C++ and exudyn.symbolic has no derivative as an expression, so the connectors go through
  sympy. Sub-steps RG12.50.2-5; waiting for the decision.

<a id="rg12-50-1-refined"></a>
### RG12.50.1, refined — one structure behind every output (2026-10-25, #3086)
*(Maintainer 2026-10-25: how would esym.GetFlatProgram(expressions, names) work, with examples - input and output? Why
not make the text executable, ToString(format='plain' / 'exudyn.symbolic' / 'numpy' / 'sympy'), and would the
dependencies (q, q_t) need a separate export? How does GetFlatProgramData() describe the tree - are there indices of
the nodes? ToString of the data and of an expression should work the same, with little overhead; a common structure
from which another code (C++, ...) is written in Python without Exudyn - if sympy offers that, a conversion to sympy is
enough. Update the concept, list the decisions and recommendations.)*
- docs/dev/design/symbolicExport.md rewritten: the recording is a graph; the structure is the flat program, entry k =
  node k = tK of the text, operands by number and always earlier; one Python writer with a flat and a nested layout
  writes every format, and reproduces str() and GetFlatProgram() of today as the test; the arguments and results are
  part of the structure (the full signature of a user function, the named variables of expressions, VariableSet
  variables as keyword parameters); esym.GetFlatProgram/GetFlatProgramData(expressions, arguments, resultNames) with
  examples; the four formats; the functions whose meaning differs (mod = fmod, round half away from zero, comparisons
  as 0/1, IfThenElse); sympy as the common structure for C, C++, Fortran, Rust, Julia, Matlab, LaTeX, lambdify, cse and
  codegen, with what it changes (simplifies and reorders while building, optional); nine decisions with
  recommendations. Plan sub-steps RG12.50.2-3 renamed to match.

<a id="rg12-49-1-revised"></a>
### RG12.49.1, revised — the optimization on the expressions (2026-10-25, #3085)
*(Maintainer 2026-10-25: the optimization could belong to the symbolic expressions, not only to the flat program - a
function expression = symbolic.optimize(expression, options) or simplify, generating a new tree, with levels: exact and
unproblematic simplifications (level 1), the rest (level 2); merging of duplicates, mergeDuplicates=True, so that q[3]
is one single leaf; would mergeDuplicates > 1 be a depth of comparison, only in the flat program - is
sin(q[1]+3)*cos(q[2]-q[0]) written twice evaluated once today? Defaults level=1, mergeDuplicates=True. Update the
design file, revised recommendations and decisions.)*
- Measured: `sin(q[1]+3)*cos(q[2]-q[0])` written twice is 11 operations in the flat program, 6 needed; the flat program
  merges by pointer only, at no depth; every `q[1]` is a new component node with a new index leaf.
- docs/dev/design/symbolicFlatProgramOptimization.md revised: `esym.Optimize(expressions, level=1, mergeDuplicates=True)`
  builds a new graph, sharing what does not change; one pass from the leaves up with a key of class and new operand
  pointers (commutative operands sorted, numbers by bits, variables and arguments by pointer), so merging compares any
  depth and a depth parameter is not needed - a bool; the leaves merge; level 1 exact (`1*x`, `x**1`, `x**0`, numbers,
  IfThenElse of a number, components of a vector number, and `x+0` with only the sign of a zero), level 2 `0*x`, `0/x`,
  `x-x`, `x/x`, combined numbers; CreateSymbolicUserFunction(optimize=...) with the same defaults; the flat program
  unchanged; eight decisions with recommendations. Plan RG12.49 and RG12.49.2 renamed.

<a id="rg12-50-2"></a>
### RG12.50.2-3 — the flat program as a Python structure and as code (2026-10-25, #3087, #3088)
*(Maintainer 2026-10-25: esym.GetFlatProgram(e) shall have flatten=True, which keeps the expressions unflattened,
still convertible; format 'plain' shall be called 'Python', 'exudyn.symbolic' 'symbolic'; symbolicExport.py into
misc/, later also misc/symbolicSystem.py and misc/symbolicUserItems.py; the speed of the translation through the
structure evaluated, and compatible to go back to C++; the symbolic model in a separate design file, later; the rest of
the recommendations accepted; start with RG12.50.2 and RG12.50.3.)*
- C++: `FlatName()` in the 63 expression classes with operands (empty for the leaves); IfThenElse with FlatName and its operands
  (Condition, IfTrue, IfFalse); `FlatProgram::ToString(resultNames, flatten)`; in Symbolic.cpp the class
  FlatProgramData (the traversal of FlatProgram::Build, IfThenElse with three operands, leaves as number or variable
  with value, text and size), FlatProgramDataOfExpressions, FlatProgramDataOfUserFunction (arguments from the stored
  dictionary, argNameList from CreateSymbolicUserFunction, the named variables that are no arguments as variables),
  FlatProgramOfExpressions (the format Python by C++), SymbolicToString; bindings esym.GetFlatProgramData,
  esym.GetFlatProgram, UserFunction.GetFlatProgram(format, flatten), UserFunction.GetFlatProgramData,
  ToString(format) of Real, Vector and Matrix; the definitions in pybindSymbolic.py.
- python/exudyn/misc/symbolicExport.py: Write (formats Python, symbolic, numpy, sympy; flat and nested), ToFunction,
  ToSympy; the functions where numpy and sympy differ written as Exudyn computes them (mod = fmod, sympy.trunc being
  the truncation of polynomials; round half away from zero; comparisons 0/1; IfThenElse; Not; isfinite refused by
  sympy); the variables of exudyn.symbolic.variables read with esym.variables.Get by the format symbolic, which keeps the
  signature that CreateSymbolicUserFunction checks, keyword arguments otherwise. In the reference manual
  (utilityDocsModel), sympy in the extra [tests] of pyproject.toml (checkExtras).
- python/TestModels/symbolicExportTest.py: three recorded user functions (every scalar function and operator; the
  vector and the matrix operations) and two expressions: the writer's Python equals the C++ text flat and nested, numpy
  within 1.1e-16 relative of the recording at random arguments, the format symbolic recorded again gives the same flat
  program, sympy within 3.9e-16 (sympy 1.14, installed into venvExuP313 from PyPI for it), isfinite refused, an
  argument missing refused; reference 11450.057870734236.
- Speed (docs/dev/design/symbolicExport.md section 5): C++ text 3.8-4.8 M entries/s, structure 1.1-1.4 M/s, Write
  1.1-1.9 M/s. Found: IfThenElse a leaf of the flat program (#3089, RG12.49.3); deep recordings overflow the stack
  (#3090, RG12.49.4). Unary + of vectors and matrices and == of vectors cannot be recorded from Python, so the writer
  has them untested.
- Docs: introductionBasics.md (a recording as code), revisions.md, the design note rewritten to what is built.

<a id="rg12-49-2"></a>
### RG12.49.2-3 — esym.Optimize, IfThenElse in the flat program (2026-10-25, #3091, #3089)
*(Maintainer 2026-10-25: do the next recommended steps; RG12.49.2 as recommended in the design file.)*
- C++: `NewWithOperands()` in each of the 63 operation classes (one line, generated by a script from the
  constructors; VectorExpressionSReal, MatrixExpressionSReal and IfThenElse by hand); class Optimizer in Symbolic.cpp:
  one pass without recursion, a map from old to new nodes, keys of class and new operands (+, *, ==, != sorted), numbers
  and number vectors and matrices by bits, named variables by pointer; the rules of symbolicFlatProgramOptimization.md
  section 3.2 at level 1 and 2; the created nodes that are not used are freed (each held once more while the list is
  freed in reverse order); a number created is counted as the others (ExpressionReal counts its first user in its
  constructor - found by the count of the nodes); esym.Optimize(expressions, level, mergeDuplicates), recording on
  while it runs.
- CreateSymbolicUserFunction and ConvertFunctionToSymbolic: optimize=True (default; a dict: the arguments of Optimize;
  False: as recorded); argNameList kept.
- IfThenElse (#3089): NumberOfOperands 3, EvaluateFromOperands selecting by the condition (as the tree, a condition
  other than 0 is true), ToStringFromOperands as its ToString; the special case in FlatProgramData removed; the
  writer's Python text now equals the C++ text also for symbolicExportTest.py's IfThenElse function.
- symbolicOptimizeTest.py: the operation counts per level (rotation matrix 11/8/8/8, duplicated sin*cos 11/6/6/6, numpy
  product 16/13/12/3), the values bit for bit at level 0 and 1, the rules at small cases as text, a nan kept at level 1
  and lost at level 2, a list merged across its results, no node left over, CreateSymbolicUserFunction with
  optimize False/True/level 2 (41/29/20 operations, the same values); reference 212.46417414604238. Found: a named
  esym.Matrix is never freed (#3092, RG12.49.5), not caused by Optimize; the test creates its matrix before it counts.
- All 215 test models give their references with optimize on by default; userRigidBodyTest.py 314 -> 227 operations;
  a rigid-body force with RotXYZ2GLocal written three times, 200000 explicit steps: 0.31 -> 0.15 s.
- Docs: introductionBasics.md, revisions.md, pybindSymbolic.py (Optimize), the design notes.

<a id="rg12-49-4"></a>
### RG12.49.4 — deep recordings, evaluated (2026-10-25, #3090)
*(Maintainer 2026-10-25: do they occur today? can we estimate the depth that causes the stack overflow? can we measure
the depth? do the evaluation first, then suggest a way to go or a fix.)*
- Today: the depth (the longest path from the result to a leaf, from GetFlatProgramData) of every recorded user
  function of the 16 test models and examples with CreateSymbolicUserFunction: at most 44 (symbolicExportTest.py, a sum
  of 40 terms in a loop), userRigidBodyTest.py 17, symbolicUserFunctionMasses.py (2544 functions) 7. Deep recordings come
  from loops in Python that accumulate - a sum over many terms is as deep as its number of terms.
- Measured (a chain x = sin(x*z + c)*z, 4 nodes per step; each in its own process, the stack of python.exe 2 000 000
  bytes): the recording and Evaluate work up to about 33000 nested nodes and fail at 34000, Diff at 28000, str() at 9000;
  GetFlatProgram, GetFlatProgramData and Optimize are iterative and are limited by the recording only. About 60 bytes of
  stack per level for Evaluate, 70 for Diff, 220 for str().
- The recording itself is quadratic in the depth: SReal(ExpressionBase*) evaluates the whole new tree for its value,
  so a chain of 1000/2000/4000/8000 steps records in 0.07/0.24/1.0/4.6 s, a sum of 8000 terms in 0.2 s; and it is the
  recording that overflows the stack first.
- Proposal: (1) the value of a new SReal from the values of its operands, which the operators have, instead of
  evaluating the tree: linear recording and no recursion while recording; (2) Destroy without recursion (an explicit
  stack), so that deleting a deep recording cannot crash; (3) str() written by the flat program in the nested layout,
  which is iterative; (4) Evaluate and Diff of the tree (flatEvaluation=False, esym.Real.Evaluate) check the depth, which
  the flat program has when it is built, and raise a ValueError above 10000 instead of crashing; (5)
  esym.GetDepth(expressions) for the user. With (1)-(3) only the tree evaluation keeps a limit, and it is reported.

<a id="rg7-8-4"></a>
### RG7.8.4 — the multithreaded ODE2 residual and user functions (2026-10-25, #3093)
*(Found while analysing RG7.8.3, maintainer 2026-10-25: do the next recommended steps.)*
- CSystem::ComputeSystemODE2RHS: `nItems` is the number of listComputeObjectODE2LhsNoUF, but the parallel loop took
  `listComputeObjectODE2Lhs[j]`, the list of all objects. With an object with a user function among the first nItems,
  it was computed in a worker thread and once more in the serial loop of the user functions, and as many objects
  without user functions at the end of the list were not computed at all. Measured: 100 mass points with
  spring-dampers, the first with a symbolic springForceUserFunction, 4 threads, ExplicitEuler 100 steps: the last mass
  point at 0.0495 (its spring left out) against 0.0424613 with 1 thread; with a Python user function the run hangs (a
  Python call from a worker thread). Fix: `listComputeObjectODE2LhsNoUF[j]`. The other parallel loops (mass matrix,
  loads, algebraic equations, projected reaction forces) take their NoUF lists.
- python/TestModels/multithreadingUserFunctionTest.py: the chain with a Python user function at the first and a symbolic
  one at the 50th spring-damper, 4 threads against 1 thread: difference 0; reference 0.04949999999999993.
- RG7.8.3 analysed (plan text): A the evaluation with a context per thread, B the parallel lists; not started.

<a id="rg12-52"></a>
### RG12.52 — the introduction of exudyn.symbolic (2026-10-25, #3094)
*(Maintainer 2026-10-25: regarding symbolic, in either case, the introduction section to Python-C++ interface/Symbolic
has to be improved significantly, giving an overview and the background of the capabilities.)*
- The introduction of docs/generated/cInterface/Symbolic.md, written in definitions/pybindSymbolic.py: what
  exudyn.symbolic is for and what a symbolic user function gains (5 to 44 times the Python function, no Python at the
  call, drawn by the render thread, written out as code); what is recorded (the types, named variables, nodes, numbers
  computed at once, numpy's meaning, the refusals, the portable functions, symbolicUtilities); symbolic user functions
  (CreateSymbolicUserFunction, keep it alive, what is supported, the user items); how a recording is evaluated (flat
  program, Optimize, GetFlatProgram, flatEvaluation, Evaluate, Diff a number); a recording as code (the formats,
  symbolicExport); what to know (a Python if decided once while recording, IfThenElse; loops unrolled; only
  exudyn.symbolic and numpy recorded; not yet in the parallel loops; the depth limit; verify against the Python function).
- Checked while writing it: a Python `if` on a symbolic comparison does not raise while recording but takes the branch
  of the values of the arguments at the recording, which are zero.
- A list in a pybind description is a Markdown list; checkDefinitions refuses `\bi ... \ei` there.

<a id="rg7-8-3"></a>
### RG7.8.3, RG7.8.5 — symbolic user functions in the parallel loops (2026-10-25, #2978, #3095)
*(Maintainer 2026-10-25: then do (if possible now): RG7.8.3, parts A and B.)*
- Part A, done as a copy per thread instead of a context: SymbolicFunction::PrepareThreads(n) copies the recording for
  the threads 1 ... n-1 in the order of the flat program - each operation new by NewWithOperands (#3085), each argument
  a new named leaf of the same name and value, the numbers and the other variables shared, as an evaluation only reads
  them - and builds the flat program of the copy; an evaluation takes the copy of ExuThreading::TaskManager::GetThreadId(),
  the main thread the recording. The context through the evaluation of about 30 classes is not needed: the operations
  of a copy keep their vectors and matrices, as the recording does. A recording can be copied if every operation of its
  flat program without operands is a number, a vector, a matrix or a variable; a constant result, or a leaf that is no
  argument, is read only. processArgument writes into the leaves of the copy; an argument the recording does not use
  has none.
- Part B: UserFunctionBase, the base of PythonUserFunctionBase<UFT>, with the kind and the symbolic function;
  ComputableInThreads(): none, or a symbolic function that can be copied, set through SetPythonUserFunction (a
  symbolic function set only as std::function is not), and no Python object argument (ObjectConnectorUser).
  GetUserFunctions(list) of every item with user functions, generated by itemHeaderEmitter.py from its parameters of
  type PyFunction, in CNode, CObject, CMarker, CLoad, CSensor. CSystem::PreComputeItemLists puts into the serial lists
  the items with a user function that is not computable in the threads, and those reading such an item - the objects on
  such a node, the markers on such a node or object, the connectors and loads on such a marker -, the rule of the
  drawing in the main thread (#3003); before, connectors on an ObjectUserBody, NodeUser or MarkerUser were computed in
  the threads whatever their user functions. CSystem::UpdateUserFunctionsForThreads(), at the start of the six
  functions with a parallel loop: the lists again when a user function was set since (UserFunctionAssignmentCount(),
  e.g. in a preStepUserFunction), the copies when there are more threads than prepared.
- Found by forcing 4 threads: ObjectUserBody wrote its Jacobian, the rotation matrix and the vectors of the numerical
  derivative into members (#3046), NodeUser its rotation matrix (#3049); two connectors on one body in two threads
  crashed about 1 run in 7. They are static thread_local in the functions now, with the memory kept per thread; 25
  runs without a crash.
- #3095: PostNewtonStep ran listDiscontinuousIteration in the threads without regard to the user functions; 4
  ObjectConnectorRigidBodySpringDamper with a Python postNewtonStepUserFunction, 4 threads, hung on the build before
  (no end after 60 s). The loop runs in the main thread if one of its connectors is in the main thread
  (discontinuousIterationInMainThread).
- Measured, each 4 threads against 1, results bit for bit equal: 4000 mass points with spring-dampers, one symbolic
  springForceUserFunction for all, RK44 200 steps, 1.30 -> 0.77 s (without user functions 0.93 -> 0.33 s, one Python
  function among them 1.03 -> 0.55 s); 200 user rigid bodies with the symbolic functions of userRigidBodyTest.py on
  spring-dampers, RK44 200 steps, 0.77 -> 0.31 s. The user ANCF cable of userANCFCable2DTest.py has Python user
  functions only and was not measured.
- The test suite with every dynamic solve in 4 threads and the lower limits 1 (a wrapper of solver._SolveDynamic):
  217 test models and 98 mini examples pass.
- python/TestModels/symbolicUserFunctionThreadsTest.py: a chain with one symbolic function for all spring-dampers and
  one Python one, at t = 0.02 a connector set to a Python function and one to a second symbolic function by a
  preStepUserFunction; 10 user bodies with symbolic functions (a constant mass matrix) and spring-dampers between them;
  a rigid body with a Python postNewtonStepUserFunction; 4 threads against 1: difference 0; reference 860.0074721246586.
- Docs: pybindSymbolic.py (What to know), userItems.md, revisions.md.

<a id="rg12-49-5"></a>
### RG12.49.5 — the count of a named symbolic matrix (2026-10-25, #3092)
*(Maintainer 2026-10-25: then do RG12.49.5.)*
- Not a leak: the node of `esym.Matrix('M', ...)` is deleted with the matrix; SymbolicRealMatrix(name, array) counted
  `newCount += 1 + (exprMatrix == 0)`, 2 for one node, as exprMatrix is still nullptr there. Now NewCount()++, as the
  other constructors. The description of #3092 (a reference too many) was wrong; its release note says what it was.
- symbolicOptimizeTest.py creates the named matrix after the first count of the nodes and deletes it before the last;
  reference unchanged.

<a id="rg11-12"></a>
### RG11.12 — the design notes of the closed steps distributed (2026-10-25, #3097)
*(Maintainer 2026-10-25: which of the information goes into the docs; the files merged to the implemented status, no
plan, no status before, no recommendations, decisions can stay; docs/dev/internals/: yes; closed notes deleted after
merging: move to tmp/delete/; symbolic as a chapter or a subchapter, without duplicating the reference page.)*
- The eleven notes of docs/dev/design were all of closed steps. Merged, each into the state as implemented with its
  decisions and the measurements that support them, without the analyses of options, the proposals, the steps and the
  earlier states:
  - docs/dev/internals/symbolic.md from symbolicStatus, symbolicIntermediateValues, symbolicFlatProgramOptimization,
    symbolicExport and symbolicUtilities, plus the copies for the threads of RG7.8.3; three statements of the notes were
    out of date and are written as they are now: esym.cross (#3070), esym.pi (#3071), numbers as branches of IfThenElse
    (#3072);
  - docs/dev/internals/userFunctions.md from userFunctionBoundary (the path of a call, the threads, the interfaces, the
    performance against 1.13.6 of section 13); the counts of the interfaces from userFunctionInterfaces.py today (57
    user functions, 50 with a symbolic version, 31 signatures, against 45, 39, 24 at 1.13.141);
  - docs/dev/internals/userItems.md from pythonUserItems (what each kind gives the solver, the user items, the common
    pattern, the parameters, the interfaces, the cost, the decisions);
  - docs/dev/internals/systemUtilities.md and symmetricBasis.md, their notes without the history;
  - contributions: WORKFLOW.md had the mechanism; added the rule for a second branch of the maintainer.
- The notes are in tmp/delete/design/ (untracked) and removed from the repository; docs/dev/design/ stays for the
  notes of open steps, excluded from the published documentation (#3082). The links of the plan point to the pages that
  have the content now; the log keeps the names of the notes, which git has.
- The scripts to tools/benchmarks/: measureUserFunctions.py, userFunctionBoundary.py, userFunctionInterfaces.py,
  userFunctionPerformance.py, symmetricBasis.py, symmetricBasisBenchmark.cpp, with their paths; userFunctionInterfaces.py
  and symmetricBasis.py run. symbolicGraphSize.py went to tmp/delete/ instead: it parsed functions of
  userRigidBodyTest.py that are gone since #3069, and it measured graph against tree for the flat evaluation, which is
  built.
- docs/manual/symbolicUserFunctions.md, a chapter between the advanced introduction and the user items: a first
  symbolic user function with its check (the example run: the recording equals the Python function exactly), writing a
  function that records, vectors, matrices and the user items, speed and threads, a recording as code. The overview,
  the types and "what to know" stay on the reference page, which the chapter links; the three bullets on symbolic user
  functions of the user functions section of introductionBasics.md moved into the chapter, one sentence and the link
  remain there.
- WORKFLOW.md §8.4: a design note lives while its step is open; when it closes, the user part goes into the manual, the
  inside into docs/dev/internals/, the history into the log, the note leaves the repository and its scripts go to
  tools/benchmarks/.

<a id="rg12-49-6"></a>
### RG12.49.6 — bool() of a symbolic value refuses while recording (2026-10-25, #3096)
*(Maintainer 2026-10-25: at recording, "if x>0" would take the current value of x, which is meaningless; this should
only work if x is not symbolic; "if y>0" still works if y is a regular Python value, decided at recording. Can be done
now.)*
- Found first: esym.Real had no `__bool__` and no `__len__`, so Python took every symbolic value as true - `bool(x > 1)`
  with x = 0 was True, and so was `bool(esym.Real(0.))` with recording off. A Python `if` in a recorded function took
  the first branch for every call, not the branch of the values at recording, as the log of RG12.52 says (note
  2026-10-25: that statement was not measured and is wrong; the reference page, the manual and internals/symbolic.md
  are corrected).
- `__bool__` of esym.Real (Symbolic.cpp, next to `__float__`): while recording, an expression or a named variable raises
  a TypeError that names esym.IfThenElse and x.Evaluate(); otherwise the value, not zero. `and`, `or`, `not` go through
  it as well.
- Checked: the test suite, the pytest files and the 4 examples with exudyn.symbolic pass - none of them reads bool() of
  a symbolic value while recording. symbolicModuleTest.py: if, and, or, not raise; a number and an unrecorded comparison
  are conditions; the reference is unchanged.
- Docs: pybindSymbolic.py (What to know, `__bool__`), symbolicUserFunctions.md, internals/symbolic.md, revisions.md.

<a id="rg12-53"></a>
### RG12.53 — algorithmic symbolic functions, written down (2026-10-25)
*(Maintainer 2026-10-25: SetValue during recording is dangerous anyway; the intended assignments in algorithms would be
different - the more algorithmic view of functions, where in future one could use a.Assign(a+1), which would allow an
algorithmic approach, including for-loops. Add this as an RG step in symbolic, optional, no open issue now.)*
- Written down as the optional step RG12.53, without an issue.

<a id="rg12-49-4-2"></a>
### RG12.49.4 — deep recordings without a stack overflow (2026-10-25, #3090)
*(Maintainer 2026-10-25: everything recommended is good to go; SetValue during recording is dangerous anyway - so the
suggestion plus an exception, if not needed in our current cases. The algorithmic view with Assign is RG12.53.)*
- The value of a new expression (Symbolic.cpp, RecordNode): `recordedValue` and `recordedDepth` in the three node
  bases - a number, or a pointer to the vector or matrix the node evaluated into -; a new holder computes its node by
  EvaluateFromOperands from the recorded values of its operands, a leaf has its present value. A node made by Optimize
  or for a thread is recorded when first needed, with its operands, by an explicit stack. NumberOfItems, NumberOfRows,
  NumberOfColumns read the recorded value (they evaluated the tree); the in-place operators keep the value as well.
  The values are those of the tree evaluation at the time, except after SetValue of a variable between recording an
  operand and its user: then the stored value is from the operands, and Evaluate() gives the present one.
- DeleteNode: the 109 places `x->Destroy(); delete x; deleteCount++` replaced; a thread-local list of nodes to delete,
  drained by the first caller. The counts of new and deleted nodes stay balanced (symbolicModuleTest.py,
  symbolicOptimizeTest.py, the new test).
- str() of Real, Vector and Matrix: FlatProgram::ToString with flatten = false, the same text; quadratic in the depth,
  as each operation copies the text of its operands (3000 steps of the chain, 12000 operations, 0.4 s).
- The tree evaluation: Evaluate and Diff from Python of the three types, and flatEvaluation = False, raise a ValueError
  above 10000 operations (maximumTreeDepth; about 60 and 70 bytes per level, below the 1 MB stack of a Windows thread).
- esym.GetDepth(expressions); esym._SetRecordingFunction, set by CreateSymbolicUserFunction around the call of the
  function (with try/finally, which also restores SetRecording on an error): SetValue, SetVector, SetMatrix and a
  component of a named variable raise meanwhile. No recording of the test models and examples sets a value.
- Measured (the chain x = sin(x*z + 0.1)*z of the evaluation): 1000/4000/16000/60000 steps record in
  0.003/0.013/0.049/0.18 s (before 0.07/1.0/- /-, a crash above about 8500 steps), deleted in 0.006 s at 60000 steps;
  a symbolic user function of 5000 steps (20000 operations) created in 0.04 s, its flat evaluation the bits of the loop
  with cMath.
- python/TestModels/symbolicDeepRecordingTest.py: 200000 operations recorded, measured, deleted, the nodes balanced;
  str of 12000; Evaluate and Diff refused above, equal to cMath below; the value from the operands; a size error at
  recording; a user function of 20000 operations, flatEvaluation = False refused; SetValue while recording refused.
  Reference 41.235299378685376. The test suite, the pytest files and the 4 examples with exudyn.symbolic pass.
- RG12.49 closed with it.

<a id="rg12-4-1"></a>
### RG12.4.1 — item outputs in symbolic user functions (2026-10-25, #2984)
*(Maintainer 2026-10-25: do RG12.4.1 now - the proposal as written.)*
- Recording: CreateSymbolicUserFunction passes `_SymbolicMainSystem(mbs)` (advancedUtilities.py) as the mbs argument.
  GetNodeOutput, GetObjectOutput, GetObjectOutputBody, GetMarkerOutput and GetSensorValues call the real function once,
  for the size of the output - in the requested configuration, else in the reference configuration, else an error that
  asks for mbs.Assemble() before the recording -, and return `esym._ItemOutput(...)`: an esym.Vector, its component 0 for
  a scalar output; an output of two dimensions (HomogeneousTransformation, exu.HT) is refused. `variables` gives
  mbs.variables; every other attribute raises an AttributeError, also setting one.
- Evaluation: `VectorExpressionItemOutput` (SymbolicVector.h, evaluated in Symbolic.cpp), a vector leaf with the kind,
  item number, OutputVariableType, ConfigurationType and local position; it evaluates into its buffer by
  CNode::GetOutputVariable, CObject::GetOutputVariable (a connector through ComputeMarkerDataStructure and
  GetConnectorOutputVariable, as MainSystem::PyGetObjectOutputVariable), CObjectBody::GetOutputVariableBody,
  CMarker::GetOutputVariable or CSensor::GetSensorValues, without Python and without the checks the recording did.
  SymbolicFunction collects these leaves from its flat program; ItemOutputScope in each of the 12 evaluation templates
  sets the MainSystem of the call into them and takes it back, also on an error, and refuses a second evaluation of the
  same function inside the first (an item reading its own output). Without a MainSystem the leaf has the value of the
  recording; UserFunction.Evaluate takes it from its mbs argument and requires it. The leaves are no leaves of values,
  so such a recording is not copied for the threads and its item is computed in the main thread.
- Measured, symbolicItemOutputTest.py: a mass point pulled by three loads that read a node, a body point, a marker, a
  sensor and the length of a spring-damper, 500 steps: Python functions 0.069 s, symbolic 0.0074 s; the same result
  to the bit, also with 4 threads. Found while writing it: GetObjectOutput has outputs for connectors and objects such as
  GenericODE2, not for bodies (GetObjectOutputBody).
- The test suite, the pytest files and the 4 examples with CreateSymbolicUserFunction pass: no recording used another
  attribute of mbs.
- Docs: symbolicUserFunctions.md (Reading other items), pybindSymbolic.py, internals/symbolic.md and userFunctions.md,
  the docstring of CreateSymbolicUserFunction, revisions.md. RG12.4 closed with it.

<a id="rg12-2-2"></a>
### RG12.2.2 — the fixed-size arguments of symbolic user functions copied in one pass (2026-10-25, #3039, #2930)
*(Maintainer 2026-10-25: #2930 tested, everything looks good now. RG12.2.2: do the measurement; for larger matrices this
would matter in the future; if it does not extend the code significantly and the speedup is about 10 %, it is worth
doing.)*
- Not the race of RG7.8.3: the argument leaves exist per thread (each thread copy has its own), and each call writes
  those of its thread. The arguments of springForceTorqueUserFunction of ObjectConnectorRigidBodySpringDamper are
  copied three times: Matrix6D -> std::array by the item, through the std::function by value, into the named leaf by
  processArgument; only the last is the symbolic code.
- Measured with tools/benchmarks/userFunctionPerformance.py (rigid6, RK44, fastest of 9), against 1.13.185 from venvP313
  as reference: 1.13.185 0.571, 1.13.207 0.576-0.587 us per call (no change by today's steps; the table of 1.13.141
  has 0.407, an other state of the machine). processArgument skipping the matrices after their first call (a
  measurement build, not kept): 0.449-0.463 us, so the copy by components - SetNumberOfRowsAndColumns and 36 range
  checked matrix(i, j) per 6x6 matrix, an initializer list per 3x3 - cost about 0.125 us, 21 % of the call.
- Done: processArgumentMatrix copies a std::array matrix by one std::copy into the memory of the leaf, both by rows
  (static_assert on the contiguity), resized only if the size differs; the same for std::array vectors (StdVector2D,
  3D, 6D: one template instead of the initializer lists). rigid6 0.580 -> 0.428 us (0.74 of 1.13.185), cartesian
  0.38 -> 0.31, load unchanged; results bit for bit. About 20 lines less.
- Not done: copying only on a change (option B) or reading in place (option C) would save the remaining ~0.03 us, about
  6 %, below the 10 % of the decision; B needs a change counter of the item at every call site and gains nothing for a
  recording shared by several items, C a matrix view through the flat evaluation. For large matrix arguments, C is the
  one to take, with the conversion of the item.
- The test suite passes.

<a id="rg7-15-1"></a>
### RG7.15, RG7.15.1, RG7.15.2 — the review of the user items; the SolutionViewer of their test models, ObjectJointUser a joint (2026-10-25, #3098, #3099)
*(Maintainer 2026-10-25, the review of ObjectConnectorUser - its arguments, a NodeGenericData and its data coordinates
for any kind of contact, a postNewtonStepUserFunction, a sphere-sphere contact as test -, of ObjectJointUser - the screw
joint of userJointTest.py does not work beyond 180 degrees; nowhere in Exudyn is the constraint Jacobian approximated by
numerical differentiation, as it leads to severe artifacts, so a jacobianUserFunction is always required and a missing
one fails in Assemble; check what the other user items need at least -, of MarkerUser - it is a body marker, MarkerBodyUser
with a parameter markerType -; physicsMass in symbolicItemOutputTest.py; solution.file.write = False before
mbs.SolutionViewer() in several test models, so the viewer refuses; ObjectJointUser listed under Constraint.)*
- Written down as RG7.15 with six sub-steps, one issue each (#3098 to #3103).
- RG7.15.1: userANCFBeam3DTest, userANCFCable2DTest, userJointTest, userMarkerTest, userMassIntegralTest,
  userOutputVariablesTest, userParametersTest, userRigidBodyTest, userTetrahedronTest: `mbs.SolutionViewer()` before
  `simulationSettings.solution.file.write = False` - the viewer reads the solution file of the run and refuses when
  writing is off. The results are unchanged.
- RG7.15.2: ObjectJointUser has objectType Joint: its page moved from the constraint index to the joint index.

<a id="rg7-15-3"></a>
### RG7.15.3, RG7.15.4 — the screw joint beyond 180 degrees; the Jacobian of ObjectJointUser required (2026-10-25, #3100, #3101)
- The cause of #3100 is the constraint, not the Jacobian: `d[0] - h*atan2(Arel[2,1], Arel[1,1])` jumps by 2 pi h when
  the angle passes 180 degrees, so beyond half a turn there is no continuous solution. Measured with a torque that
  turns the body further: the atan2 version follows the analytic angle exactly up to x/h = pi and stops there (the
  solver reduces the step to its minimum).
- Written periodic, `h*(Arel[2,1]*cos(phi) - Arel[1,1]*sin(phi))` with phi = d[0]/h, which is h*sin(angle - phi):
  smooth for any number of turns, the same derivative as before at the solution. With its analytic Jacobian (rows of
  Arel[i,j] by (a1_j x a0_i).(theta1 - theta0), of d = A0^T (p1 - p0) by A0^T and A0^T skew(p1 - p0) theta0; checked
  against central differences of the rotations to 1e-10) it follows the analytic solution through 2.5 turns: angle
  15.2990 against 15.3 at index 3 (1.2e-3 in the rotation matrix with 500 steps, the time integration), 15.299994 at
  index 2.
- The numerical Jacobian of CObjectJointUser equals the analytic one to 7e-11 for this constraint, but with it the
  solver failed at t = 0.24 with the Newton tolerances 1e-12 of the test (passed with 1e-8): the accuracy of central
  differences limits what Newton reaches, as userItems.md said for MarkerUser.
- #3101: CheckPreAssembleConsistency of ObjectJointUser requires the jacobianUserFunction of an active joint, with the
  reason; ComputeJacobianIncrements calls only it (the central differences removed); dc/dt at velocity level stays a
  central difference in t, it is no Jacobian. The definition, its mini example (with UFjacobian), userItems.md and
  revisions.md say so.
- userJointTest.py: the screw periodic with UFscrewJacobian over 2.5 turns (torque 0.25), compared by the rotation
  matrix of the analytic angle and the translation; the pendulum with the given Jacobian only; a missing
  jacobianUserFunction among the refusals. Reference 28.411748466345937 (before -1.8272301913592162).
- What the other user items need at least, as they are: ObjectUserBody the massMatrixUserFunction (forces, positions,
  rotations and their Jacobians optional; the Jacobians of positions and rotations by central differences if not
  given); NodeUser the positionUserFunction (its Jacobians by central differences if not given); MarkerUser the
  positionUserFunction (likewise); ObjectConnectorUser nothing (no forces without forceUserFunction). A joint on a
  NodeUser, an ObjectUserBody or a MarkerUser without their Jacobian functions gets a constraint Jacobian that contains
  central differences; left for the maintainer's decision.
- Found while writing this step: a Bash heredoc turns a double backslash into one; an edit script wrote a vertical tab
  instead of \v into userItems.md, and an earlier one a backspace instead of \b into the log of RG12.52 (`\bi ... \ei`);
  both repaired, the files of the session searched for control characters.
- The test suite passes.

<a id="rg7-15-5"></a>
### RG7.15.5 — ObjectConnectorUser with data coordinates and a postNewtonStepUserFunction (2026-10-26, #3102)
- `nodeNumber`, a NodeGenericData as in ObjectConnectorRigidBodySpringDamper (GetNodeNumber, SetNodeNumber,
  GetNumberOfNodes 1 with a node, requested type GenericData). `forceUserFunction(mbs, t, itemNumber, markers,
  dataCoordinates, parameters)`: the data coordinates of the current configuration, empty without node - the signature
  changes without deprecation, the item is not released yet.
- `postNewtonStepUserFunction(mbs, t, itemNumber, markers, dataCoordinates, dataCoordinatesStartOfStep, parameters)`
  returns a tuple, `(dataCoordinates, discontinuousError)` or `(dataCoordinates, discontinuousError,
  recommendedStepSize)`, instead of the one vector [error, step size, data...] of RigidBodySpringDamper: the
  annotation PostNewtonResult (py::object in C++, tuple in Python), read in EvaluateUserFunctionPostNewtonStep, with a
  TypeError for another result and a ValueError for data of the wrong size. The start of step is passed because a
  contact sets the impact velocity back to it during the iterations, as CObjectContactSphereSphere does.
  CObjectConnectorUser::PostNewtonStep stores the data, gives error and step size to the solver and updates the
  Jacobian when the error is not zero; HasDiscontinuousIteration with the function. The markers of the call read only:
  PyConnectorMarker without ode2Lhs refuses Apply... ("in the postNewtonStepUserFunction the markers are for reading");
  the markers of the post-Newton step are computed without Jacobians, so a force would have been dropped silently.
- CheckPreAssembleConsistency: a postNewtonStepUserFunction needs nodeNumber.
- python/TestModels/userConnectorContactTest.py: the test of ObjectContactSphereSphere (a sphere between two fixed ones,
  linear stiffness 1e6, impact model 2, restitution 0.7, 0.5 s) with the user connector, data [gap, impact velocity]:
  the speeds after the impacts 10, 7.083057, 5.009789, 3.519707 as the built-in contact to 9e-16, the same end position;
  the refusals (no data node, a force in the post-Newton step, data of the wrong size). Reference 26.270810078732666.
- parameterConversionTestReference.txt: the two new parameters of ObjectConnectorUser recorded, as for the other items.
  userFunctionInterfaces.py: 58 user functions, 50 with a symbolic version, 32 signatures (userFunctions.md).
- Docs: the definition (Data coordinates and the post-Newton step, mini example), userItems.md, revisions.md.

<a id="rg7-15-6"></a>
### RG7.15.6 and RG7.15.7 — MarkerBodyUser with providedMarkerType; ObjectUserBody without mass matrix (2026-10-26, #3103, #3104)
- Maintainer 2026-10-26: the proposal of RG7.15.6 accepted; the marker type Coordinates and the user shapes (polynomial
  curve, polynomial shell surface) became RG7.16 (#3106), the Jacobians by central differences of NodeUser,
  ObjectUserBody and MarkerBodyUser RG7.17 (#3107, tbd, kept for now, perhaps behind an exudyn.special flag), the
  derivatives of symbolic user functions by esym.Diff RG12.54 (#3108), a requested marker type of ObjectConnectorUser
  and ObjectJointUser RG7.15.8 (#3105); the massMatrixUserFunction of ObjectUserBody not required (RG7.15.7).
- MarkerUser is MarkerBodyUser, type name BodyUser: CMarkerUser.cpp renamed by git mv to CMarkerBodyUser.cpp, the three
  generated headers, the generated page and the mini example of the old name removed, the vcxproj, its filters,
  sources.json, the includes and the reference of the mini example renamed. The item is not released, so no deprecation.
- The parameter is **providedMarkerType**, not markerType: markerType is the key of the type name in the dictionary of
  every marker ('markerType': 'BodyUser'), and the generated GetDictionary wrote both into the same key. Default
  exu.MarkerType.Position; exu.MarkerType.Position | exu.MarkerType.Orientation for a rigid marker. GetType returns Body +
  Object + providedMarkerType instead of deciding by the rotationMatrixUserFunction. CheckPreAssembleConsistency: Position
  with a rotationMatrixUserFunction refused, Position | Orientation without it refused, any other value refused.
- The parameter type MarkerType of the generators (definitionTypes.TMarkerType, typeModel: Marker::Type): written by
  `(Marker::Type)py::cast<Index>(...)`, so that the enum and the int that | gives both work; read back as exu.MarkerType,
  which shows a combination as `<MarkerType.???: 96>` and converts by int(). Default Marker::Position, in Python
  exudyn.MarkerType.Position. RG7.15.8 uses the same type.
- userMarkerTest.py: the pendulum marker with Position | Orientation, the refusals 2 -> 5 and the type read back
  (providedMarkerType 96, markerType 'BodyUser'); result unchanged, -0.3569348653534026. A joint on a wrong marker is
  refused by the object check before the marker check, so the marker refusals are tested without the joint.
- ObjectUserBody: Assemble no longer requires the massMatrixUserFunction; EvaluateUserFunctionMassMatrix raises "...
  massMatrixUserFunction is not given, but the mass matrix is needed - by a dynamic solver or an output such as the
  kinetic energy". userTetrahedronTest.py: the patch test without mass matrix gives the same static solution (difference
  0.0), SolveDynamic raises; result unchanged.
- parameterConversionTestReference.txt re-recorded (the new name, the outcome row of MarkerType).
- Docs: the definition (overall description, Marker type section, the table), userItems.md (manual and internals, there
  also the signatures of ObjectConnectorUser and the required Jacobian of ObjectJointUser of #3101/#3102 brought up to
  date), revisions.md, symbolicUserFunctions.md.
- The test suite passes (221 test models, 98 mini examples); pytest -n 8 passes. Its first run failed for the graphics
  test of the mini example MarkerBodyUser.py, which writes a missing reference and fails once: the new reference is
  byte-identical to the one of MarkerUser.py, which is removed.

<a id="rg12-51-1"></a>
### RG12.51.1 — The design of the symbolic system models (2026-10-26, #3109)
- docs/dev/design/symbolicSystem.md, from the maintainer's points of 2026-10-26 and the section 4 of the first
  symbolicExport.md (commit 9790d8c5): the goals export, GPU/CPU evaluation and a check by a Python solver; what exists
  (flat programs and their writers, Optimize, one recording for many objects, the LTG maps, the solver API in Python,
  esym.Diff as a recursion that returns numbers, MatrixContainer triplets).
- The design: per object a flat program, the objects of one recording as a kind with tables of parameters and LTG
  indices; the residual M q_tt - f per object as the basic term; global matrices only as triplets.
- Own considerations: explicit integration from residuals still needs M^-1 f - a constant mass matrix factored once, or
  the matrix-free product M a = r(q, q_t, a) - r(q, q_t, 0) with conjugate gradients; the Jacobians best by symbolic
  differentiation into a new flat program (one function per node class next to NewWithOperands, then Optimize), which is
  exportable and also gives the marker Jacobians without sympy; forward mode with n tangents per slot (the "local
  stack") as the runtime form; esym.Diff per coordinate as the fallback for the Python solver.
- The solver: generalized-alpha in the form Exudyn uses, the settings of SimulationSettings.timeIntegration, checked
  term by term against the solver API and in time against SolveDynamic. Proposed in exudyn/misc/symbolicSystem.py,
  not systemUtilities.py, whose rule keeps the solver functions in solver.py - decision 3.
- Five sub-steps and four decisions proposed; RG12.51.2 waits for them.

<a id="rg7-15-8"></a>
### RG7.15.8 — requestedMarkerType of ObjectConnectorUser (2026-10-26, #3105)
- ObjectConnectorUser.requestedMarkerType, of the parameter type MarkerType of #3103, default exu.MarkerType._None
  (markers of any type, as before); GetRequestedMarkerType returns it, so the check of CSystem refuses at Assemble a
  marker that lacks one of the types ("... contains marker with invalid type 'NodePosition', but expected marker type
  'PositionOrientation'"). The default Marker::_None is written as exudyn.MarkerType._None in Python.
- ObjectJointUser gets no such parameter: it computes on the rigid connector interface (ItemConnectorInterface('Rigid'))
  and requests Position | Orientation; no other value would work.
- userConnectorTest.py: the rigid connector requests Position | Orientation, a position marker under that request is
  refused; refusals 2 -> 3, result unchanged. parameterConversionTestReference.txt re-recorded.
- Docs: the definition (the paragraph in "The markers in the user function"), userItems.md.

<a id="rg12-51-2"></a>
### RG12.51.2 — SymbolicSystem of user bodies and generic ODE2 objects (2026-10-26, #3110)
- The decisions of the design (maintainer 2026-10-26) recorded in docs/dev/design/symbolicSystem.md §7/§8 and in the
  plan: RG12.51.3-.6, RG12.54.1-.2 and RG12.55 (#3111, the user items with the interfaces of the built-in items, before
  RG12.51.4).
- python/exudyn/misc/symbolicSystem.py: SymbolicSystem(mbs, evaluator='symbolic' | 'numpy') after Assemble. The objects
  are grouped in kinds by object type and the text of the flat programs of their recordings; a SymbolicKind holds the
  recordings, the object numbers, the LTG table (objects x local coordinates) and the parameters. ObjectUserBody with
  symbolic massMatrixUserFunction (required) and forceUserFunction; ObjectGenericODE2 with M, K, D, the force vector and
  symbolic user functions; objects without coordinates are skipped; a Python user function, other objects with
  coordinates and loads are refused with the list. The evaluator 'symbolic' calls the recording (Evaluate), 'numpy' the
  function symbolicExport.ToFunction writes from it, once per kind.
- MassMatrixTriplets (rows, columns, values of the nonzero local entries), MassMatrix(sparse=False | True, scipy
  csr_matrix, an ImportError naming scipy otherwise), Forces (the ODE2 part of the right-hand side: f + f_user - K q -
  D q_t for GenericODE2), Residual (M q_tt - f, object by object without the global matrix).
- python/TestModels/symbolicSystemTest.py: 3 kinds - two user bodies with one recording (nonlinear spring, damping),
  one with a mass matrix depending on q and a force depending on t, a GenericODE2 with a symbolic force -, 11 ODE2
  coordinates; at 3 random states mass matrix, forces and residual by both evaluators and the sparse mass matrix equal
  ComputeMassMatrix/ComputeODE2RHS of MainSolverImplicitSecondOrder to 4.4e-16; the refusals. Reference 3.
- The user functions of the test are recorded only, so they use exudyn.symbolic functions (esym.sin): exudyn.cMath
  takes no symbolic arguments, and checkPythonMath refuses np.sin.
- Docs: utilityDocsModel.py lists the module (its page generated), revisions.md, internals/symbolic.md. A docstring
  section "Attributes:" is not one of the parsed sections (Args, Returns, Note, Example): written as Note.
- The test suite passes (222 test models, 98 mini examples), the docs build; pytest -n 8 passes except the flaky
  browser test of test_issueTracker.py (a timeout), which passes alone.

<a id="rg12-54-1"></a>
### RG12.54.1 — The Jacobian of the residual column by column by esym.Diff (2026-10-26, #3108)
- SymbolicSystem.ResidualJacobian(q, q_t, q_tt, t, factor_q=1, factor_q_t=0, factor_q_tt=0, sparse=False): factor_q
  dr/dq + factor_q_t dr/dq_t + factor_q_tt dr/dq_tt, as dense array or scipy sparse matrix; factor_q_tt alone gives
  the mass matrix, a Newton step combines the three. For GenericODE2 the constant K, D, M are added to the derivatives
  of its user functions.
- SymbolicKind.ResidualExpressions(): the residual of a kind, once, as one esym.Real per component of named variables
  (q, q_t, q_tt, t, and qRef and the parameters of a user body); per object the values are set by SetValue and one
  Diff per entry gives each column.
- Why the wrapper _Scalar: the 'symbolic' code of a recording gives esym.Vector and esym.Matrix results, and a
  component of a vector or matrix expression has no Diff ("Symbolic::operator[]::Diff(vector, vector): not
  implemented" - the vector and matrix expressions have no derivatives); the 'numpy' code with esym.Real components
  fails where an esym.Real multiplies a numpy constant, which gives an esym.Matrix. The numpy code with object arrays of
  _Scalar, a wrapper that returns NotImplemented for numpy arrays and implements the functions numpy calls on objects
  (sin, arctan2, ...) by those of exudyn.symbolic, gives each component as its own esym.Real.
- symbolicSystemTest.py: the Jacobian of each derivative and of a combination against central differences of the
  residual, 6.0e-9 (the error of the differences); against ComputeJacobianODE2RHS of the solver at q_tt = 0 - which is
  the Jacobian of f = -r, and computed by central differences for the user bodies - 4.7e-6 at entries of 400, checked
  relative to 1e-6. Result unchanged, 3.
- The answer to the question of 2026-10-26 on esym.sin in the test: np.sin of an esym.Real does work - numpy calls the
  method sin() of an object, and esym.Real has one -, while exudyn.cMath.sin takes numbers and numpy arrays only;
  checkPythonMath refuses np.sin in test models (#2964, platform-dependent for numbers), so a recorded function writes
  esym.sin, which records and also takes numbers.

<a id="rg12-55-1"></a>
### RG12.55.1 and RG12.56.1 — The design of the user items and of the explicit solvers (2026-10-26, #3111, #3112)
- docs/dev/design/symbolicUserItems.md: what exists (ObjectUserBody, the built-in markers on it, ObjectJointUser and
  ObjectConnectorCoordinateVector symbolic, ObjectConnectorUser not - its markers are Python objects); connectors and
  contacts therefore as ObjectUserBody without mass on the nodes of both bodies; the connectors need mbs at creation,
  so the interface proposed is the Create functions of mbs with the same signatures on a creator object
  (UserItemCreator(mbs), or mbs itself), Python user functions first and one recording per kind by Record(); the table
  of the items of the maintainer's list with what each becomes and the arguments supported and refused, the further
  items for a good representation (planar bodies, bushings, coordinate constraints, generic joint, sensors unchanged),
  the order (ground, mass point, spring-damper, force first, for the explicit solvers), five decisions. RG12.55.2-.5
  planned.
- docs/dev/design/symbolicSystem.md §9: the interface SystemEquations of every solver - nODE2, InitialState, Forces,
  MassMatrix, IsMassMatrixConstant, Residual and ResidualJacobian with defaults (central differences only as fallback
  for numpy systems) -, SymbolicSystem and NumpySystem on it, one result dict. §10: explicit Euler, RK44 and velocity
  Verlet as src/Solver/CSolverExplicit.cpp computes them (the first-order system with the Butcher tableau; Verlet with
  the force at the half-step velocity), the constant step size of startTime, endTime, numberOfSteps, the mass matrix
  factored once when constant, the settings without meaning refused when not default, the check against SolveDynamic
  with the same solver type and a NumpySystem. RG12.56 (#3112, new, maintainer 2026-10-26) with .2 and .3 planned;
  RG12.51.3 after it. §5 says how option B is implemented.

<a id="rg12-51-7"></a>
### RG12.51.7 — SymbolicSystem on SymbolicSystemBase, exudyn.symbolic only; the planning of 2026-10-26 (2026-10-26, #3113)
- The maintainer's direction (2026-10-26): the system is extracted from mbs, only if every item is compatible, and
  reuses its data; a computed state goes back into mbs, where the renderer, the sensors and the outputs read it - only
  the solve is new; it evaluates by exudyn.symbolic, numpy and other formats come from an export of the whole system;
  what exudyn.symbolic lacks is added there, not worked around in SymbolicSystem; a base class for the solvers.
- python/exudyn/misc/symbolicSystem.py: SymbolicSystemBase (nODE2, InitialState, IsMassMatrixConstant, Forces,
  MassMatrixTriplets, MassMatrix, Residual, ResidualJacobian by central differences of Residual); SymbolicSystem(mbs)
  on it, without the evaluator argument and the numpy code of symbolicExport; InitialState from the Initial
  configuration, IsMassMatrixConstant from constantMassMatrix and the GenericODE2 objects, SetSystemState(q, q_t, q_tt,
  t, configuration); refused as well: recordings that read mbs (an item output, 'mbs.' in the flat program), ODE1
  coordinates, algebraic equations, data coordinates. The wrapper _Scalar of RG12.54.1 is removed, so the Jacobian is
  central differences until #3115. scipy is imported at the start of the module, with a clear ImportError.
- symbolicSystemTest.py: one system; the sparse mass matrix in every state; the Jacobian by q_tt against the mass
  matrix 2.3e-9, against the solver's (also central differences for user bodies) 4.7e-6 at entries of 400; the initial
  state, the state written back and read by GetNodeOutput (6e-17); IsMassMatrixConstant False; three refusals. Result
  unchanged, 3.
- The gaps of exudyn.symbolic, in docs/dev/design/symbolicSystem.md §11 with the proposals, each a new step (maintainer
  2026-10-26): RG12.57 cMath with symbolic arguments (#3114); RG12.58 Diff of vector and matrix expressions by a seed
  (variable, component), a derivative buffer per node next to its evaluation buffer, UserFunction.EvaluateJacobian
  (#3115); RG12.59 the forward derivative of a flat program in one pass, in one memory block per program sized at the
  first evaluation (#3116); RG12.60 a recording called with symbolic arguments (#3117). RG12.51.8 the built-in loads
  in the system (#3118). §3 the system model as extracted from mbs and written back, §9 SymbolicSystemBase instead of
  SystemEquations.
- docs/dev/design/symbolicUserItems.md: the decisions on the user items (the Create signatures on UserItemCreator,
  a small set first, a Python graphicsDataUserFunction for the drawing), and §2 why a symbolic ObjectConnectorUser is
  not possible now - its ConnectorMarker objects are no symbolic arguments and its forces are side effects, not a
  result - and the extension: a second user function of the marker states that returns the forces, applied through
  the marker Jacobians; two markers as ObjectJointUser proposed, any number stacked as the alternative. RG12.55.6
  (#3119), open for that decision.

<a id="rg12-56-2"></a>
### RG12.56.2 and RG12.56.3 — NumpySystem and the explicit solvers SolveExplicit (2026-10-26, #3112)
- The maintainer's addition (2026-10-26): an explicit integrator only pays if the inverse of the mass matrix is taken
  per body (single-noded bodies), as computeMassMatrixInversePerBody does in Exudyn; otherwise the system mass matrix
  is solved in every stage, the speedup is gone and the GPU hard to use.
- python/exudyn/misc/symbolicSystem.py: SymbolicSystemBase.MassMatrixBlocks (one block of all coordinates), in
  SymbolicSystem per kind the LTG table and the stack of the local mass matrices; NumpySystem(massMatrix, forces,
  initialState, massMatrixBlocks), a constant matrix (numpy or scipy sparse) or a function; SolveExplicit(system,
  simulationSettings, solverType): explicit Euler, RK44 and velocity Verlet with the Butcher tableaus and the order of
  operations of CSolverExplicit, the steps of startTime, endTime, numberOfSteps, the accelerations by the inverse per
  body (batched numpy.linalg.inv per kind, the blocks checked to share no coordinate) or by scipy splu, once for a
  constant mass matrix (reuseConstantMassMatrix), computeEndOfStepAccelerations, a Lie group node refused with
  useLieGroupIntegration; the result dict at solution.file.writePeriod as CSolverBase writes it; for a SymbolicSystem
  the output through an Exudyn MainSolverExplicit - InitializeSolver, the initial state and per step SetSystemState
  and FinishStep, run as explicit Euler without the inverse per body (ObjectUserBody refuses it in Exudyn, #3120).
- symbolicExplicitSolverTest.py: two user bodies of one kind, one with a variable mass matrix, an ObjectGenericODE2; the
  three methods with and without the inverse per body against SolveDynamic, 400 steps: 4.2e-15 relative; a sensor
  written through mbs against that of SolveDynamic 5.1e-14 (the initial row needed the call of FinishStep at the start
  time); the same equations as NumpySystem 8.5e-17; the numbers of force and mass matrix evaluations as in Exudyn
  (800 for explicit Euler with end-of-step accelerations, 2000 for RK44, 401 for velocity Verlet). Result 1.
- docs/dev/design/symbolicSystem.md §10 the inverse per body, the settings and the check as done; §12 an outlook,
  asked for by the maintainer: one step of all objects in parallel on the GPU by levels of the flat programs, with
  no-ops and uniform operation layers, and related work from memory (levelized gate-level simulation, GPU genetic
  programming, Cartesian genetic programming, level scheduling, wavefront path tracing), not checked online.
- docs/dev/design/symbolicUserItems.md §2: decision (a) and the name markerForceUserFunction; the answer to the
  question whether the ConnectorMarker form is needed - its one use is a force at a computed point of a flexible body
  (userConnectorTest.py part 3), ObjectConnectorUser is not released yet; proposed: the returning function becomes the
  forceUserFunction, the other form kept under another name or removed, for the maintainer.

<a id="rg12-58"></a>
### RG12.57 and RG12.58 — cMath with symbolic arguments; Diff of vectors and matrices (2026-10-26, #3114, #3115)
- definitions/pybindCMath.py: per function of exudyn.cMath an overload with Symbolic::SReal calling SReal::sin<SReal>
  and so on - all but cbrt; atan2 and pow also (SReal, Real) and (Real, SReal) -, registered after the float overload,
  so a float costs nothing more; src/Pymodules/Pybind_manual_classes.cpp includes Linalg/Symbolic.h.
- src/Linalg/Symbolic.h: struct DiffSeed (variable, component); Diff(const DiffSeed&) instead of
  Diff(ExpressionNamedReal*); VectorExpressionBase and MatrixExpressionBase get DiffRef(seed) and a diffBuffer next to
  the evaluation buffer (sized at the first call and kept, no heap of their own), a default that raises naming the
  operation. SymbolicVector.h, SymbolicMatrix.h: DiffRef of the named vector and matrix (unit component), the vectors
  and matrices of scalars, +, -, scalar times, matrix @ vector, vector @ matrix, multiply, unary minus and plus, matrix @
  matrix, transposed; Diff of the dot product, NormL2 (raises at zero), the components; an item output raises.
- Python: Real.Diff(vector, component), Vector.Diff(var), Vector.Diff(vector, component), Matrix.Diff likewise,
  UserFunction.EvaluateJacobian(argumentName, *args) - a vector, matrix or array (rows x columns x argument) -,
  documented in definitions/pybindSymbolic.py. SymbolicSystem.ResidualJacobian by EvaluateJacobian of the force and
  mass matrix recordings by the arguments at the positions of q and q_t (their names from GetFlatProgramData).
- symbolicVectorDiffTest.py: 18 expressions by x and the components of q against central differences 1.9e-10;
  EvaluateJacobian of a force and a mass matrix recording 1.1e-10; four refusals; cMath of symbolic arguments, recorded
  and evaluated, equal to the numbers. symbolicSystemTest.py: the Jacobian by Diff against central differences 6.0e-9,
  result unchanged, 3. symbolicExplicitSolverTest.py writes its numpy system with cMath.sin.

<a id="rg12-55-6"></a>
### RG12.55.6, RG12.56.4 and RG12.51.9 — ObjectConnectorUser with the forces of the marker states; the user body inverted per body; connectors in the symbolic system (2026-10-26, #3119, #3120, #3121)
- The maintainer's decisions (2026-10-26): two markers with named arguments, as ObjectJointUser; the returning function
  becomes the forceUserFunction and the ConnectorMarker form is kept under another name for forces at computed points
  of flexible bodies; then the inverse per body of ObjectUserBody, then the explicit solver with a relevant system.
- definitions/itemDefsObjects.py, ObjectConnectorUser: forceUserFunction(mbs, t, itemNumber, position0, velocity0,
  rotationMatrix0, angularVelocity0, position1, velocity1, rotationMatrix1, angularVelocity1, dataCoordinates,
  parameters) returning [f0, f1], or [f0, m0, f1, m1] with Orientation in requestedMarkerType - the rotations then of
  the markers, else unit matrices and zeros; the former form as markerAccessUserFunction; GetRequestedMarkerType adds
  Position for a forceUserFunction; a graphicsDataUserFunction (Python, global coordinates) with HasUserFunction and
  CallUserFunction of the visualization, written out in the definition as the library has no entry for connectors.
  C++: EvaluateUserFunctionForce applies -J^T f by the position and rotation Jacobians of the markers, a result of the
  wrong size raises naming the sizes; EvaluateUserFunctionMarkerAccess is the former function; Assemble refuses both
  functions and a forceUserFunction of other than two markers. ConnectorMarker, its documentation and errors name the
  markerAccessUserFunction. The description: the forces of the two markers first, the access to the markers second,
  the drawing; the mini example with the new form.
- userConnectorTest.py: the spring-damper as forceUserFunction and as markerAccessUserFunction against
  CartesianSpringDamper 1.6e-15; a rotational damper and a spring of a rigid body as forceUserFunction with torques
  against RigidBodySpringDamper 1.4e-13; three new refusals (both functions, one marker, three values); result
  unchanged. userConnectorContactTest.py: the contact force by the forceUserFunction, the switch by the
  postNewtonStepUserFunction reading the markers; result unchanged. docs/manual/userItems.md, internals/userItems.md.
- src/ImplObjects/CObjectUserBody.cpp: with computeInverse the local mass matrix - stored if constant - into a dense
  matrix and inverted, a singular one raises (#3120).
- exudyn/misc/symbolicSystem.py: ObjectConnectorUser with a symbolic forceUserFunction on MarkerNodePosition of
  NodePoint (coordinates) or NodePointGround and MarkerBodyPosition of ObjectGround (fixed, the reference position of
  the marker); a kind per pattern of markers; the forces scattered, MassMatrixBlocks without connectors, the Jacobian
  by EvaluateJacobian of the recording by position0/1 and velocity0/1; a markerAccessUserFunction, a
  postNewtonStepUserFunction, Orientation and other markers refused; an inactive connector left out (#3121).
- symbolicExplicitSolverTest.py rewritten on a net of 4 x 4 mass points, the upper corners fixed, 24 spring-dampers,
  gravity, written with cMath.sqrt and recorded: the three methods with and without the inverse per body against
  SolveDynamic with the same settings 2.6e-14; the sensor 6.9e-16; the mass matrix in every stage equal; the user items
  against MassPoint and ObjectConnectorSpringDamper 1.1e-16; the net as NumpySystem 8.3e-17; the Jacobian of the
  residual by Diff against central differences 7.3e-11 relative to the stiffness. For the record, 200 steps of RK44:
  0.29 s SolveExplicit on the symbolic system, 0.12 s on the NumpySystem, 0.02 s SolveDynamic. Result 1.

<a id="rg10-33"></a>
### RG10.33 — exudev clean and the output in the root (2026-10-27, #3122)
*(Maintainer 2026-10-27: .pytest_cache, .ruff_cache, .mypy_cache, the PDF build, build, bin and solution in the root -
are they needed, and can exudev clean remove them; _build is needed for the HTML documentation, tmp is intended.)*
- What each is: `.pytest_cache`, `.ruff_cache`, `.mypy_cache` - the caches of pytest, ruff and of the stubtest of
  `tools/checkPython.py --stubs` (21 MB), each ignoring itself by its own .gitignore, written again by the next run;
  `_buildpdf` - the LaTeX build of `exudev docs --pdf` (62 MB), which copies the PDF out of it; `solution/` - a
  coordinatesSolution.txt of 2026-10-27 07:35 (14 ODE2, 2 AE, 100 steps) from a model run with the root as working
  directory and no EXUDYN_OUTPUTDIRECTORY; `bin/` - OutDir of msvc/cppsrc.vcxproj, the Debug and Release module of
  Visual Studio (187 MB), with its IntDir `msvc/x64/` (2.0 GB of object files); `build/` - only the flag stamp
  exudynBuildFlags.txt of setup.py, the empty linux directories of the WSL build and an asanpdb.pdb of 2026-09-17.
- tools/exudev/commands.py `Clean`: by default also the three caches, `_buildpdf` and `solution/`; `--vs` the Visual
  Studio output `bin/` and `msvc/x64/` (a full rebuild in Visual Studio afterwards); `--all` includes `--vs`. Never
  removed: `_build`, `build/` itself, `tmp/`. `build --clean` and `build --complete` pass `vs=False`.
- tools/exudev/README.md, docs/dev/BUILD.md. `exudev clean -n` lists the five directories; with `--vs` also bin and
  msvc/x64. pytest test_exudev.py passes.

<a id="rg10-33-1"></a>
### RG10.33.1 — the pytest cache in build/ (2026-10-27, #3123)
*(Maintainer 2026-10-27: yes, the pytest cache under build/.)*
- pyproject.toml `[tool.pytest.ini_options]`: `cache_dir = 'build/.pytest_cache'`, relative to the root directory of
  pytest, which is the repository root by pyproject.toml. exudev clean removes `build/.pytest_cache`, and still a
  `.pytest_cache` in the root from older runs. docs/dev/BUILD.md.

<a id="rg10-33-2"></a>
### RG10.33.2 — the ruff cache in build/ (2026-10-27, #3124)
*(Maintainer 2026-10-27: move the ruff cache into build; the solution directory of the root: see first whether it
comes back.)*
- pyproject.toml `[tool.ruff]`: `cache-dir = "build/.ruff_cache"`, for the editor and a ruff run by hand;
  tools/checkPython.py runs ruff with `--no-cache`. A `ruff check` wrote build/.ruff_cache and nothing in the root.
  exudev clean removes `build/.ruff_cache`, and still a `.ruff_cache` in the root from older runs. docs/dev/BUILD.md.

<a id="rg12-51-8"></a>
### RG12.51.8 — the built-in loads in the symbolic system (2026-10-27, #3118)
*(Maintainer 2026-10-27: then do three of the next RG12 steps - this one, RG12.55.2 and RG12.51.3.)*
- exudyn/misc/symbolicSystem.py: `SymbolicLoad` (load number, type, LTG indices, value, the recording of its user
  function, the user body it acts through); `SymbolicSystem.loads`. LoadForceVector, LoadTorqueVector,
  LoadMassProportional and LoadCoordinate, the value constant or by a symbolic loadVectorUserFunction /
  loadUserFunction of the time:
  - on MarkerNodePosition of a NodePoint (force) and MarkerNodeCoordinate (LoadCoordinate) the value at the LTG
    indices of the marker, as the Jacobian of these markers is the unit matrix;
  - on MarkerBodyPosition, MarkerBodyRigid and MarkerBodyMass of an ObjectUserBody $\Jm\tp \vv$ with $\Jm$ of its
    symbolic positionJacobianUserFunction, rotationJacobianUserFunction or massIntegralUserFunction at the local
    position of the marker; `bodyFixed` rotates $\vv$ by its rotationMatrixUserFunction, as CSystem.cpp applies loads;
  - the Jacobian of $-\fv$ by `EvaluateJacobian` of these functions, $d(\Jm\tp \Rot \vv) = d\Jm\tp \Rot\vv + \Jm\tp
    d\Rot\,\vv$; a load on a node or coordinate depends on the time only;
  - a load on the ground acts on nothing and is left out; other loads, a torque or bodyFixed on a point node, other
    markers and bodies are refused with the reason, a Python user function of the load or the body raises.
- The ObjectIndex of a marker is passed to the recordings as int, as `Evaluate` takes numbers only.
- symbolicSystemLoadsTest.py: a user rigid body on NodeRigidBodyRxyz (all functions symbolic, symbolicUtilities) under
  gravity by MarkerBodyMass, a force of sin(10t) off the center of mass, a body-fixed force and torque and a constant
  torque on MarkerBodyRigid; a user mass point with a force on MarkerNodePosition and a LoadCoordinate of cos(5t); a
  force on the ground. SolveExplicit RK44 against SolveDynamic 1.5e-15, the Jacobian of the residual (factor_q 1,
  factor_q_t 0.5) against central differences 1.7e-10, two refusals. Result 1.
- docs/dev/design/symbolicSystem.md §3, docs/manual/revisions.md.

<a id="rg12-55-2"></a>
### RG12.55.2 — UserItemCreator: ground, mass point, spring-damper, force (2026-10-27, #3125)
*(Maintainer 2026-10-27: three of the next RG12 steps; decisions of 2026-10-26: the Create signatures on
UserItemCreator(mbs), a small set first.)*
- python/exudyn/misc/symbolicUserItems.py, `UserItemCreator(mbs)`; its Create functions have the arguments and defaults
  of those of mbs, the deprecated ones included, which raise when given:
  - `CreateGround`: mbs.CreateGround, the built-in ground has no coordinates;
  - `CreateMassPoint`: the NodePoint and the gravity load (MarkerBodyMass, LoadMassProportional) of mbs.CreateMassPoint,
    an ObjectUserBody with `parameters=[mass]`, `constantMassMatrix`, the mass, mass integral, position (node position
    plus local position, as MassPoint), rotation and Jacobian functions; `create2D` and `physicsMass` refused;
  - `CreateSpringDamper`: the markers of mbs.CreateSpringDamper (GetMarkersPosRot), an ObjectConnectorUser with
    `parameters=[k, d, L0, f, v0]` and the forceUserFunction of ObjectConnectorSpringDamper written with cMath; at the
    center of a user mass point the marker is on its node, which SymbolicSystem takes until RG12.51.4; drawn by a
    graphicsDataUserFunction, a line, or a cylinder of diameter drawSize; `springForceUserFunction` refused;
  - `CreateForce`: mbs.CreateForce; a Python loadVectorUserFunction is recorded by Record.
  - `Record()`: each user function of a kind recorded once and set in every item of the kind; `recordings` keeps
    them alive.
- symbolicUserItemsTest.py: one function builds a chain of three mass points on spring-dampers from the ground,
  gravity, a force of sin(10t), with `creator = mbs` and `creator = UserItemCreator(mbs)`: the Python user items and the
  recorded ones against the built-in items 5.6e-16 / 6.7e-16 (generalized-alpha) and 1.4e-17 / 2.2e-16 (RK44); the
  recorded system as SymbolicSystem (3 kinds, 4 loads), SolveExplicit against SolveDynamic 1.3e-15; the four
  signatures equal by inspect; two refusals. Result 1.
- tools/generators/utilityDocsModel.py: the module's page; docs/dev/design/symbolicUserItems.md §3 and §4
  (itemNumbers instead of the deprecated bodyNumbers), docs/manual/revisions.md.

<a id="rg12-51-3"></a>
### RG12.51.3 — the generalized-alpha solver in Python (2026-10-27, #3126)
*(Maintainer 2026-10-27: three of the next RG12 steps.)*
- exudyn/misc/symbolicSystem.py `SolveGeneralizedAlpha(system, simulationSettings)`, as
  src/Solver/CSolverImplicitSecondOrder.cpp computes it (Arnold and Bruls 2007, Algorithm 1):
  - the parameters of PreInitializeSolverSpecific: alphaM, alphaF, gamma, beta of the spectral radius; with
    `useNewmark` alphaM = alphaF = 0.5 and beta, gamma of the settings; beta' and gamma';
  - initial accelerations M^-1 f with `computeInitialAccelerations`, else zero; the algorithmic accelerations start
    equal to them;
  - the predictor of ComputeNewtonUpdate(initial), the accelerations set to zero; Newton with the increment of
    `ResidualJacobian(q, q_t, q_tt, t, 1, gamma', beta')` by scipy's splu: q -= d, q_t -= gamma' d, q_tt -= beta' d;
    the convergence of CSolverBase::Newton - the residual relative to the initial one, or absolute, divided by
    sqrt(n) with `weightTolerancePerCoordinate`; `useModifiedNewton` keeps the Jacobian over the iterations of a step;
    after convergence the algorithmic accelerations as FinalizeNewton;
  - constant step size: a step that does not converge raises; `useIndex2Constraints` and Lie group nodes are refused.
  - The output of SolveExplicit moved into one class `_Output` of the stored results and the output through mbs, which
    both solvers use; SolveExplicit computes as before (its test unchanged).
- The Jacobian contains d(M q_tt)/dq, which Exudyn leaves out: the iterates differ, the solution not.
- symbolicGeneralizedAlphaTest.py: a chain of five recorded user mass points on spring-dampers of k = 1e5 (made with
  UserItemCreator), gravity, a force of sin(20t), h = 1e-3, about 2/omega_max; relativeTolerance 1e-10. Against
  SolveDynamic: the trapezoidal rule (rho = 1), rho = 0.6, Newmark, without initial accelerations, modified Newton -
  q 1.1e-15 to 1.1e-11, q_t to 1.2e-11, q_tt to 1.1e-10 relative, the sensor 1.0e-12; 2 Newton iterations per step,
  modified Newton 1 Jacobian per step. Result 1.
- docs/dev/design/symbolicSystem.md §6 and §7, docs/manual/revisions.md.

<a id="rg10-33-3"></a>
### RG10.33.3 — the output of the models pytest runs in build/pytestOutput (2026-10-28, #3127)
*(Maintainer 2026-10-28: solution/: raise an issue and fix now.)*
- `solution/coordinatesSolution.txt` came back in the root during `pytest -n 8 python/testing`. Each test file run
  alone from the root: test_connectorInterface, test_accessFunctionsAD, test_specialBeams and test_graphicsData write
  it - a solver initialized or run with the default `solution.file.write`.
- python/testing/conftest.py `pytest_configure`: EXUDYN_OUTPUTDIRECTORY = `build/pytestOutput/<worker id>` (`main`
  without pytest-xdist) unless it is set already; the controller of `-n` sets nothing, because its workers inherit
  its environment and would all write into one directory. It runs before the test modules import exudyn.
- An output directory refuses an absolute file name, and changes the config defaults: a fixture `noOutputDirectory`
  (exudyn.config.outputDirectory empty for one test) in the 7 tests that needed it - test_exceptions (3, the solver
  file in tmp_path), test_graphicsData (2, PlotImage into tmp_path), test_userSettings (2, the stored config
  settings).
- exudev clean removes `build/pytestOutput`; docs/dev/BUILD.md, tools/exudev/README.md.
- `pytest -n 8 python/testing`: 1427 passed, no `solution/` in the root, the files in build/pytestOutput/gw*.

<a id="rg12-55-3"></a>
### RG12.55.3 — the user rigid body and torsional spring-damper in UserItemCreator (2026-10-28, #3128)
*(Maintainer 2026-10-28: do the suggested RG12 steps now; when building a test, also measure the CPU time of the
symbolic and the internal integration.)*
- exudyn/misc/symbolicUserItems.py `CreateRigidBody`: ObjectUserBody on NodeRigidBodyRxyz, the node as
  mbs.CreateRigidBody computes it (reference and initial rotation, the angular velocity as Tait-Bryan rates), the
  parameters [mass, inertia about the node (6), centerOfMass (3)]; the user functions written once with
  exudyn.symbolicUtilities and the functions module of the argument - cMath for the numbers in Exudyn, exudyn.symbolic
  while recorded: the mass matrix of CObjectRigidBody::ComputeMassMatrix (G^T J G, the coupling -m A skew(c) G of a
  center of mass off the node), the forces of ComputeODE2LHS on the right-hand side, the position, rotation, their
  Jacobians, the mass integral; gravity by MarkerBodyMass and LoadMassProportional as the built-in function. Refused:
  every nodeType but RotationRxyz - the default Euler parameters need their algebraic equation, the rotation vector
  node is a Lie group node -, graphicsDataUserFunction, create2D.
- `CreateTorsionalSpringDamper`: the MarkerBodyRigid of the built-in function (the frame of the joint, its z-axis the
  axis), ObjectConnectorUser requesting Position | Orientation, the torque of CObjectConnectorTorsionalSpringDamper -
  the z Tait-Bryan angle of A0^T A1 by atan2, the z-component of A0^T (omega1 - omega0) -; drawn as a line or cylinder
  along the axis. The torques act on the markers, as the forces of the spring-damper: +tau A0 e_z on marker 0, the
  sign checked against the built-in item (the opposite sign drove the chain unstable). Refused: unlimitedRotations
  True, its default - the turns need a data coordinate -, springTorqueUserFunction, markers in itemNumbers.
- python/TestModels/symbolicUserRigidBodiesTest.py: three bars with the center of mass off the node, each end held by
  three spring-dampers of length 0.1 (a spring of length zero has no direction) and three torsional spring-dampers
  (global x, y, local [0,1,1]) - a first model with one torsional spring per joint left the spin about the bar axis
  free and became chaotic, so that step size and Newton iterates decided the result -; gravity, a force of
  sin(10 t). Built-in against Python and recorded user items: RK44 1.9e-14, generalized-alpha 5.5e-13 relative; the
  arguments of both Create functions equal to those of mbs; nodeType and unlimitedRotations refused. Result 1.
- The times of SolveDynamic, 250 steps (printed, not judged): RK44 built-in 0.016 s, Python user items 1.75 s, recorded
  0.11 s; generalized-alpha built-in 0.03 s, recorded 0.22 s.
- docs/dev/design/symbolicUserItems.md §3/§4, docs/manual/revisions.md.

<a id="rg12-51-4-1"></a>
### RG12.51.4.1 — connectors on markers of user bodies and of the ground, with rotations (2026-10-28, #3129)
*(Maintainer 2026-10-28: do the suggested RG12 steps now; when building a test, also measure the CPU time of the
symbolic and the internal integration - "I expect that the symbolic version performs well already".)* A sub-step of
RG12.51.4, whose joints and constraints need the joints of RG12.55.4 first.
- exudyn/misc/symbolicSystem.py `_ConnectorMarkers`: per marker a dict - 'fixed' (a point ground node, a marker on the
  ground: its reference position and, for Orientation, rotation), 'point' (NodePosition of a NodePoint), 'body'
  (MarkerBodyPosition or MarkerBodyRigid of an ObjectUserBody: local position, the rotation of localHT, the recordings
  of positionUserFunction and positionJacobianUserFunction, with Orientation also rotationMatrixUserFunction and
  rotationJacobianUserFunction - a missing one refuses the connector). The requestedMarkerType Orientation is no
  longer refused; the kind key adds the marker types and the flat programs of the body functions.
- `_ConnectorStates`: the arguments of the forceUserFunction - p by positionUserFunction, v = J_pos q_t, A = A_body
  A_marker, omega = J_rot q_t; unit rotations and zero angular velocities without Orientation, as Exudyn - and the
  Jacobians of each marker; the forces J_pos^T f + J_rot^T m, the torques after the force of each marker (stride 6).
- `_ConnectorJacobian`: dF/dq = sum over markers of dF/dp J_pos + dF/dv (dJ_pos/dq q_t) + dF/dA dA/dq + dF/domega
  (dJ_rot/dq q_t), dF/dq_t = dF/dv J_pos + dF/domega J_rot, by EvaluateJacobian of the force by each marker state - the
  derivative by a matrix argument is (result x 9), row by row, which the check confirms - and of the body functions by
  qTotal; then J_pos^T dF + J_rot^T dM and the derivatives of the Jacobians times the forces.
- python/TestModels/symbolicUserRigidBodiesTest.py (of RG12.55.3) extended: the recorded chain as SymbolicSystem
  (3 user bodies, 4 kinds of connectors, 4 loads); against SolveDynamic SolveExplicit RK44 1.0e-15 (250 steps),
  SolveGeneralizedAlpha 3.2e-11 (50 steps, relativeTolerance 1e-10); the Jacobian of the residual 9.3e-11 to central
  differences. Result 1.
- **The times** (wall clock, one thread; 250 steps unless noted): SolveDynamic RK44 built-in items 0.014 s, recorded
  user items 0.11 s, Python user items 1.7 s; SolveExplicit on the SymbolicSystem 0.96 s. Generalized-alpha: built-in
  0.021 s, recorded 0.21 s; 50 steps: SolveDynamic recorded 0.041 s, SolveGeneralizedAlpha 1.30 s. Measured parts: one
  Forces 0.58 ms - 120 calls of Evaluate at about 3 us each, the rest the Python around them -, Exudyn's right-hand
  side with the recorded items about 0.11 ms; one ResidualJacobian 19.6 ms, one EvaluateJacobian by a matrix argument
  29 us. So the symbolic system is not yet fast: about 8 times Exudyn with the same recordings for RK44, 30 times for
  generalized-alpha, the cost being the per-object Python calls and the tree derivatives, not the arithmetic.
- symbolicExplicitSolverTest.py read the markers of a connector kind as (indices, position) pairs; it reads the dicts.
- docs/dev/design/symbolicSystem.md §2 and §7, symbolicUserItems.md §2/§3, docs/manual/revisions.md.

<a id="rg12-51-10"></a>
### RG12.51.10 — the equations of motion of the system as symbolic expressions and flat program (2026-10-28, #3130)
*(Maintainer 2026-10-28: "I did not see how I would be able to get the symbolic equations for the system"; two
examples, a simple one of 2 equations and one of 6-10 coordinates with couplings, the symbolic equations and the flat
program; extensions as a (sub)step.)*
- exudyn/misc/symbolicSystem.py `SymbolicSystem.Equations(format='symbolic', optimize=True)`: the variables q, q_t,
  q_tt (named exudyn.symbolic vectors, the initial state as values) and t, or sympy symbols q0, ..., t; each recording
  called with symbolic arguments through `exudyn.misc.symbolicExport.ToFunction(recording.GetFlatProgramData(),
  format)` - which records the same graph with the arguments replaced (S3 of the design, here in Python) -; the
  parameters, reference coordinates and constant matrices as numbers. Composed as the numbers: user bodies (mass matrix
  and forces), ObjectGenericODE2 (M, K, D, f and user functions), ObjectConnectorUser on their marker kinematics (fixed,
  point node, user body with the rotations) with J_pos^T f + J_rot^T m, the loads (constant or of a user function, on
  nodes, coordinates and through the body functions, bodyFixed by the rotation). Returns a dict of the variables and
  'massMatrix', 'forces', 'residual' (M q_tt - f); with optimize, esym.Optimize level 2 removes 0*x, 1*x, x+0 of the
  assembly. `GetFlatProgram(format='Python', flatten=True)`: the residual as function Residual(q, q_t, q_tt, t), as text
  or as the code of the formats of symbolicExport. `_Backend` holds the two types; `_Sum` starts a sum without 0 +.
- python/Examples/symbolicSystemSpringDamper.py: two masses, ObjectGenericODE2 with M, K, D and a symbolic
  forceUserFunction (a hardening spring, a force of the time): the residual as expression and flat program (24
  operations), M and f by sympy, the residual in LaTeX; the numpy code equal to SymbolicSystem.Residual (0.0); RK44 of
  SolveExplicit and SolveDynamic.
- python/Examples/symbolicSystemRigidBodyPendulum.py: of UserItemCreator a bar on Tait-Bryan angles (center of mass off
  the node) held by three spring-dampers and a torsional spring-damper, a mass point on a spring-damper at its end,
  gravity, a force of the time - 9 coordinates; the flat program of 474 operations, by sympy the coupling block
  M[0:3,3:6] and M[3,3], the count of operations of f (sympy.simplify of the forces took 184 s, so the example does not
  simplify them); the numpy code equal to the residual to 2e-13; SolveGeneralizedAlpha equal to SolveDynamic. 2 s.
- python/TestModels/symbolicSystemEquationsTest.py: both kinds of system, the second with a body-fixed force and a
  LoadCoordinate of a user function as well; at 3 random states each, the mass matrix, forces and residual of the numpy
  code of the flat program and of sympy.lambdify of the sympy expressions against MassMatrix, Forces and Residual:
  7.5e-15 relative. sympy is optional: without it the sympy part is left out. Result 1.
- Measured: Equations 0.009 s, GetFlatProgram 0.006 s, Equations(format='sympy') 0.15 s for the 9 coordinates.
- RG12.60 (#3117) stays open: the system no longer needs it; whether a C++ copy by NewWithOperands is still wanted
  is the maintainer's question.
- docs/dev/design/symbolicSystem.md §11 S3, docs/manual/revisions.md.

<a id="rg12-51-11"></a>
### RG12.51.11 — the forces and the mass matrix by one recording of the composed system (2026-10-28, #3131)
*(Maintainer 2026-10-28: "why the symbolic system (Python solver) for the explicit integrator case is so slow - it does
not include Diff() ... the symbolic evaluation is fast and there is a huge overhead in the Exudyn internal solver ... Is
the evaluation only done for each scalar equation separately? Can you improve that or does it require some extension to
vectorized evaluation?")*
- The cause: not per scalar equation, but per object, marker and load. On the chain of symbolicUserRigidBodiesTest.py
  (3 rigid bodies, 18 connectors, 4 loads, 18 coordinates) one Forces made about 120 calls of Evaluate - each about 3 us,
  mostly the call from Python with its arguments - and the Python around them: 0.59 ms. Evaluating the composed
  expressions of Equations as one tree was slower (3.8 ms: the tree walk evaluates shared subexpressions again), their
  numpy code 0.28 ms; recorded as a user function, evaluated by its flat program: 14 us. No vectorized evaluation was
  needed.
- exudyn/misc/symbolicSystem.py: `SymbolicSystem(mbs, composeSystem=True)` and `_Compose`: the forces and the entries of
  the mass matrix at the rows and columns of the local mass matrices, recorded by CreateSymbolicUserFunction as functions
  (mbs, t, itemNumber, q, q_t) of ObjectGenericODE2 from `_EquationLists` - the composition of Equations moved there, so
  that Equations and the recordings share it -; `composed` holds them. Forces, MassMatrixTriplets and Residual (by
  np.bincount of the triplets) use them; MassMatrixBlocks and ResidualJacobian stay per object. `Equations(variables=...)`
  takes given variables, the arguments of the function being recorded.
- `_MassMatrixSolver`: up to 200 coordinates a dense LU (scipy.linalg.lu_factor) - forming the sparse matrix and its
  checks cost 100 us per stage, more than the evaluation.
- Measured, the chain, 250 steps (wall clock, one thread): SolveExplicit RK44 0.96 s before, 0.049 s now (0.054 s with
  the inverse per body), explicit Euler 0.020 s; SolveDynamic RK44 0.115 s with the same recorded user items, 0.014 s
  with the built-in items. Per evaluation: forces 14-17 us by the recording against 88 us of ComputeODE2RHS of Exudyn
  with the user items; what is left in SolveExplicit is the Python loop and the LU of the 18 x 18 matrix. As an
  experiment, the composed recordings as forceUserFunction and massMatrixUserFunction of one ObjectGenericODE2 in Exudyn
  ran the 250 steps in 0.020 s, equal to 2e-16.
- SolveGeneralizedAlpha is unchanged (0.96 s for 50 steps): its Jacobian is per object by Diff of the trees (RG12.59).
- symbolicSystemEquationsTest.py: the references are those of SymbolicSystem(mbs, composeSystem=False), and the composed
  Forces, MassMatrix and Residual are compared too; 7.5e-15. The symbolic test models unchanged in their results.
- docs/dev/design/symbolicSystem.md §11, docs/manual/revisions.md.

<a id="rg12-51-12"></a>
### RG12.51.12 — Equations: flags for the results, the mass matrices of the bodies, no residual (2026-10-28, #3132)
*(Maintainer 2026-10-28: "system.Equations() returns a couple of terms, some of them not needed; add flags for that;
further: it is not clear what forces means (it would differ if we have a massInversePerBody or not); the
massInversePerBody-option is the most important for further tests, I believe; the residual seems to be less relevant
... if no objection, remove it.")*
- exudyn/misc/symbolicSystem.py `Equations(format, optimize, variables, massMatrix=True, forces=True,
  massMatrixBlocks=False)`: the results asked for; 'massMatrixBlocks' the mass matrices of the bodies per kind,
  (ltg, list of matrices), as MassMatrixBlocks() gives them. The docstring says what f is: the whole right-hand side of
  M q_tt = f - the quadratic velocity terms of the bodies, the connectors, the loads -, the same with the global mass
  matrix and with the inverse per body, which only solves differently. The residual and the variable q_tt removed;
  variables=(q, q_t, t). `_EquationLists` also gives the local mass matrices.
- The composed system records the entries of all local mass matrices as a third recording, so that MassMatrixBlocks is
  one call too: SolveExplicit RK44 with the inverse per body on the chain of symbolicUserRigidBodiesTest.py 0.054 s ->
  0.043 s, explicit Euler 0.018 s; the einsum of the inverse per body is one numpy call per kind, not per acceleration.
- `GetFlatProgram(format, flatten, results=['forces', 'massMatrix'])`: a function Equations(q, q_t, t).
- symbolicSystemEquationsTest.py: mass matrix, forces and the mass matrices of the bodies, 7.5e-15; the two examples
  print M and f instead of the residual.
- docs/dev/design/symbolicSystem.md §11, docs/manual/revisions.md.

<a id="rg12-51-13"></a>
### RG12.51.13 — ODE1 and algebraic coordinates in the interface and the solvers of the symbolic system (2026-10-28, #3134)
*(Maintainer 2026-10-28: "There is only the ODE2 in the symbolicSystem so far, but we also have to include the ODE1 and
AE case, such there is no gap in the future; adapt the symbolic system & solver and check the function interfaces;
There will be a final object that may carry a coupling of all kinds of coordinates (which the internal solvers are
currently not capable of - they also need to be extended)".)*
- exudyn/misc/symbolicSystem.py `SymbolicSystemBase`: nODE1, nAE (zero by default), InitialStateODE1, Forces(q, q_t, t,
  y), ODE1RightHandSide(q, q_t, y, t), AlgebraicEquations(q, q_t, y, lam, t, velocityLevel), ReactionForces(q, q_t, y,
  lam, t); Residual(..., y) and ResidualJacobian(..., y); SystemResidual = [M q_tt - f + C_q^T lambda, g - y_t, c] as
  ComputeNewtonResidual of CSolverImplicitSecondOrder; SystemJacobian by columns of the Newton unknowns, the ODE2 block
  by ResidualJacobian and the reaction forces, the ODE1 and algebraic rows and columns by central differences. Every
  equation may depend on every kind of coordinate. SymbolicSystem: y accepted and not used.
- `NumpySystem(..., ode1RightHandSide, initialStateODE1, constraints, constraintJacobian, numberOfConstraints)`;
  forces(q, q_t, t, y) with ODE1 coordinates.
- `SolveExplicit`: the ODE1 coordinates in the Runge-Kutta stages (CSolverExplicit), y_t zero at the start and g at the
  end of each step; algebraic equations refused, velocity Verlet with ODE1 refused.
- `SolveGeneralizedAlpha` read from CSolverImplicitSecondOrder.cpp: the ODE1 coordinates by the trapezoidal rule as
  ComputeNewtonUpdate does it (y += h/2 y_t, y_t = 0, then y -= Delta, y_t -= 2/h Delta), the Jacobian columns of the
  ODE1 unknowns d/dy + 2/h d/dy_t; the multipliers kept from the step before; the scaling of useScaling (ODE2 rows times
  beta h^2, algebraic columns divided, lambda -= Delta/(beta h^2)), the convergence on the unscaled residual of all
  equations divided by sqrt(nSys); the initial accelerations of FactorizeInitialSystemJacobian, [M C_q^T; C_q 0] with
  a unit matrix for ODE1 and -(C_q q_t)_q q_t, C_q by central differences, the multipliers with
  storeInitialAlgebraicCoordinates. Without ODE1 and algebraic coordinates the sparse path of #3126 is unchanged.
  `_InitialAccelerations`.
- python/TestModels/symbolicSystemODE1AETest.py: three masses with K and D, one ODE1 coordinate y_t = -a y + b q0 +
  sin(5 t) acting on mass 0 by -c y, a constraint q2 - q1 = 0; in Exudyn ObjectGenericODE2 and ObjectGenericODE1 with
  Python user functions that read each other's node, ObjectConnectorCoordinate. SolveGeneralizedAlpha against
  SolveDynamic with and without the constraint, rho = 0.6, the trapezoidal rule, Newmark, without initial accelerations:
  1.1e-10 to 1.9e-10 relative (relativeTolerance 1e-10); SolveExplicit RK44 and explicit Euler without the constraint
  5.9e-15 and 6.5e-15; the constraint refused by SolveExplicit. Result 1.
- Exudyn's ODE1 velocities start at zero in both solvers, so that the first trapezoidal step uses y_t0 = 0, not g at
  the start; the symbolic solvers do the same, to compare.
- SymbolicSystem still refuses items with ODE1 and algebraic coordinates (joints RG12.51.4, ODE1 items later).
- docs/dev/design/symbolicSystem.md §6, docs/manual/revisions.md.

<a id="rg12-59"></a>
### RG12.59 — the forward derivative of a flat program in one pass, in a block of the program (2026-10-28, #3116)
*(Maintainer 2026-10-28: "Then do the suggested RG12.59: the one-pass derivative in C++.")*
- Before: EvaluateJacobian walked the tree once per component of the argument, and every rule evaluated its operands
  again as trees - shared subexpressions many times: 454 us for the 6 columns of a rigid body force whose evaluation
  takes 3 us.
- src/Linalg/Symbolic.h `FlatProgram`: `Differentiate(variable, n, scalarVariable)`, `ResultDerivative(i)`; members
  `derivatives` (the block: per slot its components times n), `derivativeOffsets`, `derivativeRules`,
  `operandDerivatives`; Clear() drops the rules. src/Linalg/Symbolic.cpp: `DerivativeRule`, `FindDerivativeRule` (once
  per program, by FlatName() and the kinds of the operands), `Differentiate`: the variable a unit matrix; rules for +, -,
  *, /, pos, neg, sin ... atanh (with the domains of their Diff), pow (a constant exponent by b a^(b-1) da also for a
  base <= 0 - the tree's Diff is NaN there, #3135 -, else as Diff), atan2, IfThenElse (the branch of the condition),
  zero for the comparisons, Not, isfinite, sign, round, ceil, floor (the conditions are slots of the program and were
  differentiated otherwise, which raises), the dot product, NormL2, the components of vectors and matrices, the vector
  of components, vector +, -, scalar times, A v, A^T v (vector@matrix), pos, neg, multiply, the matrix of components,
  matrix +, -, scalar times, pos, neg, A B, A^T; every other node (abs, min, max, mod, item outputs, leaves) by its Diff
  or DiffRef per component, as before. No allocation after the block has its size; a thread copy has its own program.
- src/Linalg/SymbolicUtilities.h `PyEvaluateJacobian`: with UseFlatProgram() Evaluate, Differentiate and the result
  block; else the tree, as before (flatEvaluation = False).
- python/exudyn/misc/symbolicSystem.py `SymbolicSystem.ResidualJacobian`: up to _denseMassMatrixSize coordinates the
  Jacobian of the composed recordings - forces by q and q_t, d(M q_tt)/dq by the mass matrix entries, M - in two
  calls; above, object by object as before (the composed Jacobian is dense).
- Measured (the chain of symbolicUserRigidBodiesTest.py): EvaluateJacobian of the rigid body force by qTotal 454 -> 6.1
  us, its mass matrix 243 -> 6.6 us, a torsional spring-damper force by a rotation matrix 29 -> 4.5 us, by a position
  12 -> 4 us; one pass and tree equal to the last bit there. ResidualJacobian per object 19.6 -> 9.4 ms, composed
  further down; SolveGeneralizedAlpha, 50 steps, 0.96 s -> 0.027 s (SolveDynamic with the same recordings 0.039 s;
  3.2e-11 to it as before).
- python/TestModels/symbolicOnePassJacobianTest.py: a force and a mass matrix recording with every operation that has
  a rule, at 4 random states, by qTotal and q_t: one pass against the tree 3e-18, against central differences 5.9e-10,
  no NaN; a third recording with pow of a negative base: the tree gives NaN (80 entries), the one pass the derivative.
  Result 1. The first version of the test compared NaN and passed silently, and squares x**2 of negative coordinates
  made the tree NaN everywhere - the test compares only finite entries and counts the NaN now.
- #3135 raised for the tree's pow, plan step RG12.54.3.
- docs/dev/design/symbolicSystem.md §11 S2, docs/manual/revisions.md.

<a id="rg11-8-27"></a>
### RG11.8.27 — the test of store positions presses the store button (2026-10-28, #3133)
*(Maintainer 2026-10-28: "Fix #3135 and #3133; => do them first.")*
- python/testing/test_guiValues.py `testStorePositionsStoresEveryOpenWindow`: `confirm[0][1]()` - ShowCodeLines takes
  confirm as a list of (text, function, tooltip) since #3041; the test indexed the old single tuple and raised
  IndexError once Tk was available to the run. Passes with Tk.

<a id="rg12-54-3"></a>
### RG12.54.3 — Diff of pow for a base <= 0 with a constant exponent (2026-10-28, #3135)
*(Maintainer 2026-10-28: "Fix #3135 and #3133; => do them first.")*
- src/Linalg/Symbolic.h `ExpressionPower::Diff`: the derivatives of base and exponent first; where the exponent's is
  zero, b a^(b-1) da without the log term, also for a <= 0 (x**2 of a negative x); NaN for a <= 0 only where the
  exponent depends on the variable, as before.
- src/Linalg/Symbolic.cpp `FlatProgram::Differentiate`, RealPow: the same per column - before, a constant exponent was
  decided over all columns, so that an exponent depending on one column gave NaN in every column for a <= 0.
- python/TestModels/symbolicOnePassJacobianTest.py: the recording with pow of a negative base is compared against the
  tree like the others; no NaN in either.
- python/TestModels/symbolicModuleTest.py checked NaN for Diff of pow(x, 4) at x <= 0, the old answer: now 4 x^3 at every
  x, and pow(x, x) - an exponent depending on x - NaN for x <= 0 and x^x (log x + 1) else; the result is unchanged.

<a id="rg14-1"></a>
### RG14.1 — the design of the uniform evaluation on the CPU and the GPU (2026-10-28, #3136)
*(Maintainer 2026-10-28: "Regarding GPU/CUDA - Uniform operations: build a separate design .md file for that and move
the existing part from symbolicSystem.md to that; add a new RG for that task and if some related tasks are given in
RG12, close them there and add to the new RG (with a planning step, the Python-CPU/Numpy implementation step, and the
Python-CUDA implementation step as suggested with CuPy and the limitations that results won't be identical, like
sin(x))".)*
- docs/dev/design/uniformEvaluation.md: §1 one kernel per kind (Numba) and one step by levels; §2 levels and the block
  of numbers and inputs; §3 the two ways to uniform operations and one array of all values; §4 a step and ensembles;
  §5 the experiment with its measurements (RG14.2.1); §6 the limits - not the bits of Exudyn on the GPU (CUDA's sin,
  cos, sqrt, fused multiply-adds, atomic sums), double precision on consumer GPUs, small systems only as ensembles,
  branches; §7 the steps; §8 related work.
- Moved from docs/dev/design/symbolicSystem.md: the Numba paragraph of §4 and §12 (the outlook by levels), replaced by
  links; §7 says RG12.51.6 moved.
- Plan: group RG14 with RG14.1 (this), RG14.2 numpy on the CPU (#3137) with RG14.2.1 the experiment and RG14.2.2 into
  exudyn.misc, RG14.3 CuPy (#3138), RG14.4 Numba per kind (#3139, was RG12.51.6, closed there).

<a id="rg14-2-1"></a>
### RG14.2.1 — the levels tried on the rigid-body example in Python (2026-10-28, #3137 stays open)
*(Maintainer 2026-10-28: "I would like to start with e.g. a Python-based version that builds the vector operations and
applies it to one of the current example systems - so to see how that works and to make it more transparent (it does
not need to be very general in this case); this Python experiment would rather belong to dev/design as well, before
putting it to the exudyn library.")*
- docs/dev/design/uniformEvaluationExperiment.py [nBars] [ensembleSize]: the system of
  python/Examples/symbolicSystemRigidBodyPendulum.py as a chain of nBars bars; `Scalarize` writes the flat programs of
  the composed forces and mass matrices of the bodies (GetFlatProgramData) as one graph of scalar operations (common
  nodes reused, numbers combined, x*1, x+0, 0*x dropped); schedules by levels (as soon and as late as possible), by a
  cycle of all kinds, by a cycle of four kinds and the rest, and one array of all values; one numpy ufunc per layer and
  kind over arrays of nodes x states; the numbers and inputs a block read by every layer.
- Measured (one core): 1 bar 523 operations in 29 levels, 10 bars 7732 in 33. Entries per operation: by levels 3.4,
  as late as possible 2.5 and 2.9, a cycle of all 9 kinds 19.9 and 19.1, a cycle of 4 kinds + the rest 11.2 and 10.8,
  one array 1. Equal to SymbolicSystem.Forces and MassMatrixBlocks: 0 (1 bar) and 1.1e-16 relative (10 bars). Per state:
  SymbolicSystem 11 and 57 us; one array 162 and 315 us for one state, 1.1 and 29 us each for 1000 states; as late as
  possible 4.2 and 83 us each for 1000 states. RK44, 200 steps, an ensemble of 100 states: member 0 against
  SolveExplicit 1.3e-15 and 2.2e-15; 0.21 s and 1.5 s for the ensemble, 0.028 s and 0.10 s for one state by
  SolveExplicit.
- First version: the inputs t, q, q_t carried by copies through all levels (1376 copies for the bar); in the block
  1272; as late as possible 775. Ten bars with h = 1 ms are unstable for RK44 also in SolveExplicit: h = 1 ms/nBars.
- What it shows (design §5): the values are Exudyn's on the CPU; copies dominate the levelled form; the fixed cycle
  costs a factor of 11 to 20 for these programs, dominated by * and +; one array of all values is fastest on the CPU and
  needs no copies.

<a id="rg12-55-4"></a>
### RG12.55.4 — the joints of UserItemCreator (2026-10-28, #3140)
*(Maintainer 2026-10-28, on the proposal to make SymbolicSystem take ODE1 items and joints, RG12.51.4 which needs
RG12.55.4: "=> yes".)*
- exudyn/misc/symbolicUserItems.py: CreateRevoluteJoint, CreatePrismaticJoint, CreateSphericalJoint,
  CreateDistanceConstraint with the arguments of mbs. Each an ObjectJointUser on MarkerBodyRigid of the two bodies, the
  markers those of mbs.Create...Joint - their rotation the joint frame (revolute: z the axis, prismatic: x the axis), as
  `_JointMarkers` computes it from JointPreCheckCalcBodyMarkers; the spherical joint and the distance constraint on
  rigid markers without rotation, the distance constraint by GetMarkersPosRot(useRigidMarker=True), so also on a user
  mass point (its rotationMatrixUserFunction is the unit matrix).
- The equations as src/ImplObjects/CObjectJointRevoluteZ.cpp, CObjectJointPrismaticX.cpp, CObjectJointSpherical.cpp,
  CObjectConnectorDistance.cpp at position level; the jacobianUserFunction by the marker translations and global
  rotation increments: -I and I for a relative position, u x v and v x u for a scalar product of an axis u of marker 0
  and v of marker 1, u x d for an axis u of marker 0 times the relative position d, -/+ d/|d| for the distance. Python
  functions with cMath, recorded by Record, one recording per kind of joint.
- Refused: markers in itemNumbers; constrainedAxes other than [1, 1, 1] (ObjectJointUser has no equation lambda = 0);
  nodes in the distance constraint, distance 0. Not drawn: ObjectJointUser draws nothing; the drawing arguments are
  accepted and not used.
- python/TestModels/symbolicUserJointsTest.py: a double pendulum on revolute joints with tilted axes, a body on a
  tilted prismatic joint with a spring-damper, a spinning bar on a spherical joint, a mass point on a distance constraint
  to it, gravity; initial velocities that fulfill the constraints. The built-in joints, ObjectJointUser with Python and
  with recorded functions, SolveDynamic (generalized-alpha, 200 steps, relativeTolerance 1e-10): 2.1e-12 and 7.3e-13
  relative. Result 1.
- docs/dev/design/symbolicUserItems.md table, docs/manual/revisions.md.

<a id="rg12-51-4-2"></a>
### RG12.51.4.2 — ObjectJointUser in the symbolic system (2026-10-28, #3141)
*(Maintainer 2026-10-28, on the proposal to make SymbolicSystem take ODE1 items and joints: "=> yes".)*
- exudyn/misc/symbolicSystem.py `SymbolicSystem`: ObjectJointUser taken by `_AddJoint` into `jointKinds`, grouped as the
  connectors (the flat programs of both recordings and the pattern of the markers); `_ConnectorMarkers(..., orientation=True)`
  for its rigid markers; refused if not active or not symbolic. nAE from mbs, checked against the equations of the
  joints (other items with algebraic equations stay refused). `AlgebraicEquations` (position level only) by the
  constraintUserFunction with the marker positions and rotation matrices; `ReactionForces` C_q^T lambda: C^T lambda of
  the recorded jacobianUserFunction - forces and torques on the markers -, mapped by J_pos^T and J_rot^T of each marker.
  `__repr__` lists the joints. `_Output`: with algebraic coordinates MainSolverImplicitSecondOrder writes the output -
  the explicit one refuses constraints at InitializeSolver -, and lambda is written into mbs.
- python/TestModels/symbolicSystemJointsTest.py, the system of symbolicUserJointsTest.py recorded: the equations in the
  initial state 6.1e-16; the reaction forces against central differences of lambda^T c(q) 2.4e-11 relative;
  SolveGeneralizedAlpha against SolveDynamic, 100 steps: 1.1e-8 at relativeTolerance 1e-10, 1.1e-11 at 1e-13 - both stop
  at other iterates, as the constraint blocks of SystemJacobian are central differences; without initial accelerations
  the same 1.1e-8. Times 4.2 s against 0.048 s. Result 1.
- Open: exact constraint blocks (the Jacobian of the constraint recording by the one pass through the marker
  kinematics), the constraints in Equations(), ODE1 items.
- docs/dev/design/symbolicSystem.md §6, docs/manual/revisions.md.

<a id="rg12-61"></a>
### RG12.61 — the counts of the nodes of expressions and recordings (2026-10-28, #3142)
*(Maintainer 2026-10-28: "In order to understand the complexity of symbolic expressions, I like to have a method that
allows to count the nodes (a more deep evaluation could count every single kind of expression ...); I see this feature
in the expressions themselves, but also for the flat program => add an RG step, with a recommendation"; then "Do ...
RG12.61 (as recommended) now".)*
- src/Linalg/Symbolic.cpp: `StatisticsOfNodes` builds the FlatProgram of the roots - the traversal without recursion
  that the evaluation uses - and counts in its order: nodes, leaves, operations, the nodes used by more than one
  operation (`shared`), the depth and the tree size per node from those of its operands (a float: 2^61 - 1 for a chain
  x = x*x of 60 steps), `flatOperations` as `FlatProgram::NumberOfComputations` - the lines tK = ... of
  GetFlatProgram() -; with detailed per FlatName of the operations and number, variable or itemOutput of the leaves,
  and per kind. A result that is a number counts as one leaf. Nothing is counted at evaluation.
- Python: `esym.GetStatistics(expressions, detailed=False)` for one or a list of expressions together, and
  `GetStatistics(detailed=False)` of esym.Real, Vector, Matrix and UserFunction; definitions/pybindSymbolic.py.
- python/TestModels/symbolicStatisticsTest.py: y = sin(x)*x and z = y + x counted by hand (4 nodes, 3 operations, 1
  shared, depth 3, tree 10); the chain of 61 nodes and a tree of 2^61 - 1; A @ q + sin(q[0])*q with its kinds and
  the component; Optimize of (x*1 + 0)*cos(x + 1) + cos(x + 1), 13 nodes to 6; a recorded spring force, 17 nodes of
  which 9 operations, flatOperations equal to the lines of its flat program.
- docs/manual/symbolicUserFunctions.md, docs/manual/revisions.md.

<a id="rg12-51-17"></a>
### RG12.51.17 — lambda_ instead of lam in exudyn.misc.symbolicSystem (2026-10-28, #3148)
*(Maintainer 2026-10-28, while RG12.51.14 to .16 were being done: "don't use lam for a name in function args, variables
or docs - use lambda. Like always, avoid abbreviations. check the symbolicSystem.* files in particular.")*
- `lambda` is a keyword of Python and cannot name an argument or a variable; the arguments and variables are `lambda_`,
  the trailing underscore of PEP 8 for such a name, and the text says lambda: `AlgebraicEquations`, `ReactionForces`,
  `SystemResidual`, `SystemJacobian` of SymbolicSystemBase, NumpySystem and SymbolicSystem, the `constraints` function of
  NumpySystem, `_Output.Step`, the solvers; `lamInitial` is `lambdaInitial`.
- The other abbreviations of python/exudyn/misc/symbolicSystem.py: the locals `n1`, `n2`, `nSys` of SystemJacobian and
  SolveGeneralizedAlpha are `nODE1`, `nODE2`, `nSystem`. Kept: `nAE` (Exudyn's name, as `nODE2`), `ltg`, `Jpos`, `Jrot`
  (Exudyn's names), `J` and `dq` (the mathematical symbols).
- docs/dev/design/symbolicSystem.md (the signatures, the named vector `lambda` of section 12.2),
  python/TestModels/symbolicSystemJointsTest.py and symbolicSystemODE1AETest.py; their results unchanged.

<a id="rg12-51-14"></a>
### RG12.51.14 — ObjectGenericODE1 in SymbolicSystem (2026-10-28, #3144)
*(Maintainer 2026-10-28, on the proposal to make SymbolicSystem take ODE1 items and joints: "=> yes"; then "Do
RG12.51.14, .15 and .16 as well as RG12.61 (as recommended) now".)*
- python/exudyn/misc/symbolicSystem.py: `nODE1` from systemData; ObjectGenericODE1 grouped in `ode1Kinds` by its
  recorded rhsUserFunction (or none), with its systemMatrix and rhsVector and the LTG map of GetObjectLTGODE1; a Python
  rhsUserFunction raises as every Python user function; ODE1 coordinates of other items are refused.
  `ODE1RightHandSide(q, q_t, y, t)` is A y + f + f_user(mbs, t, itemNumber, y) per object, as
  CObjectGenericODE1::ComputeODE1RHS, plus the LoadCoordinate on MarkerNodeODE1Coordinate (`ode1Loads`, constant or
  with a symbolic loadUserFunction); `ODE1Jacobian(y, t)` exact from A and EvaluateJacobian of the recording;
  `InitialStateODE1` from the initial configuration. `_Output.Step` writes y and y_t into mbs.
- Not taken: the ODE1 equations in `Equations()` and `GetFlatProgram()`; `SystemJacobian` keeps the ODE1 columns by
  central differences (RG12.51.15 makes it exact).
- python/TestModels/symbolicSystemODE1Test.py: two Van der Pol oscillators of one recording (one kind of two objects),
  a linear system driven by a LoadCoordinate with a recorded user function, a mass on a nonlinear spring as
  ObjectGenericODE2. ODE1RightHandSide equal to ComputeODE1RHS of the solver (0.0), ODE1Jacobian against central
  differences 2.6e-10; SolveGeneralizedAlpha 8.3e-14, RK44 6.5e-15, explicit Euler 6.5e-15 relative to SolveDynamic,
  the ODE1 coordinates written into mbs; a Python rhsUserFunction refused.
- docs/dev/design/symbolicSystem.md sections 3, 6 and 12.1; docs/manual/revisions.md.

<a id="rg12-51-15"></a>
### RG12.51.15 — the exact Jacobian of SymbolicSystem with joints and ODE1 objects (2026-10-28, #3145)
*(Maintainer 2026-10-28: "Do RG12.51.14, .15 and .16 as well as RG12.61 (as recommended) now".)*
- python/exudyn/misc/symbolicSystem.py: `SymbolicSystem.SystemJacobian` overrides the one of the base class, which takes
  every block but the ODE2 one by central differences. Per joint `_JointJacobian`: C_q = sum over the markers of the
  columns of the recorded jacobianUserFunction times J_pos and J_rot; the derivative of C_q^T lambda by q from
  EvaluateJacobian of the Jacobian recording by the marker positions and rotation matrices (arguments 3 + 2k, 4 + 2k),
  contracted with lambda and composed with J_pos and dA/dq of the user body, plus dJ_pos/dq and dJ_rot/dq of the body
  times the forces and torques C^T lambda - as `_ConnectorJacobian` (#3129). The blocks: ODE2 x ODE2 ResidualJacobian
  plus factor_q times that derivative, ODE2 x AE C_q^T, AE x ODE2 factor_q C_q, ODE1 x ODE1 factor_y ODE1Jacobian -
  factor_y_t I (the rows g - y_t). velocityLevel raises, as AlgebraicEquations does.
- Measured on symbolicSystemJointsTest.py (33 unknowns, 100 steps): against the central differences of the base class
  1.6e-12 (factor_q only) and 1.2e-13 relative (the factors of a Newton step); SolveGeneralizedAlpha 4.2 s -> 0.44 s,
  SolveDynamic 0.059 s.
- **Correction of the log of RG12.51.4.2** (2026-10-28): it gave the central differences of the constraint blocks as
  the reason why SolveGeneralizedAlpha and SolveDynamic differ by 1.1e-8 at relativeTolerance 1e-10. They differ by
  the same 1.1e-8 with the exact blocks, and by 1.0e-8 also without the derivative of the reaction forces: the
  iterates differ by the ODE2 block, where SolveGeneralizedAlpha has d(M q_tt)/dq and Exudyn not (section 6 of the
  design note). The test keeps the tolerance 1e-13 (1.1e-11), with the reason corrected in its comment.
- symbolicSystemJointsTest.py checks the exact Jacobian against the central differences (1.2e-13).
- docs/dev/design/symbolicSystem.md sections 6 and 12.1, docs/manual/revisions.md.

<a id="rg12-51-16"></a>
### RG12.51.16 — the constraints of the joints in Equations and GetFlatProgram (2026-10-28, #3146)
*(Maintainer 2026-10-28: "Do RG12.51.14, .15 and .16 as well as RG12.61 (as recommended) now".)*
- python/exudyn/misc/symbolicSystem.py: `Equations(..., algebraicEquations=False, reactionForces=False)`: the results
  'algebraicEquations' (nAE, of q and t) and 'reactionForces' (nODE2, of q, t and the algebraic coordinates), the
  variable of the algebraic coordinates 'lambda_' - a named vector of exudyn.symbolic, the sympy symbols lambda_0,
  ... -, or given as fourth of `variables`. `_JointEquations` composes them as AlgebraicEquations and ReactionForces do:
  the marker states by `_MarkerStateExpressions`, the recorded constraintUserFunction, C^T lambda of the recorded
  jacobianUserFunction mapped by `_AddMarkerForces` - both helpers taken out of `_ConnectorEquations`, which uses them
  now. `GetFlatProgram(results=[...])` takes the four results and refuses others; with 'reactionForces' the function
  is Equations(q, q_t, t, lambda_). The name is lambda_, as lambda is a keyword of Python and the exported code uses
  the variable names as arguments.
- python/TestModels/symbolicSystemJointsTest.py: both results as numpy code of GetFlatProgram and as sympy expressions
  equal to AlgebraicEquations and ReactionForces, 4.1e-15 relative.
- Open: the ODE1 equations in Equations().
- docs/dev/design/symbolicSystem.md sections 6 and 12.1, docs/manual/revisions.md.

<a id="rg12-51-18"></a>
### RG12.51.18 — z for the algebraic coordinates in exudyn.misc.symbolicSystem (2026-10-28, #3150)
*(Maintainer 2026-10-28: "lambda_ is not very readable: z would be the logical quantity ... for the formulas in the
docs, we keep lambda but in the symbolic system and in user items, I believe that z is a good alternative. Anyway, it
needs to be written in the docstring for args what it represents. In the nomenclature, then mention that z is used in
Python code if lambda is not possible. Add this as substep, to switch to z consistently.")*
- python/exudyn/misc/symbolicSystem.py: `z` for `lambda_` of RG12.51.17 (#3148) in every argument and variable,
  `zInitial`, `zJoint`, `zComponents`; the variable of `Equations(reactionForces=True)` and its key 'z', the result 'z'
  of SolveGeneralizedAlpha (was 'lambda'), `Equations(q, q_t, t, z)` of GetFlatProgram; the formulas of the docstrings
  in z (`C_q^T z`), each argument "z: the algebraic coordinates, the Lagrange multipliers lambda of the
  documentation", the class docstring of SymbolicSystemBase says why.
- symbolicUserItems.py has no algebraic coordinates as arguments or variables.
- docs/manual/notation.md, the row Lagrange multipliers: `z` in Python code where `lambda` is a keyword; the design
  note, revisions.md and the two test models.

<a id="rg12-65-1"></a>
### RG12.65.1 — the old planning comments of Symbolic.cpp removed (2026-10-28, #3154)
*(Maintainer 2026-10-28: "By the way: I saw some strange old planning comments in symbolic.cpp - you can remove
that.")*
- src/Linalg/Symbolic.cpp: the comment "plan:" before the flat program section, 113 lines - about 60 items marked DONE,
  and open notes: the matrix operations (done since), mbs.symbolic for item outputs (#2984 done, the system variables
  RG12.63), assignments and loops (RG12.53), an EXUDYN_MINIMAL_COMPILATION flag and SReal for Real in general (not
  pursued, no step). No code changed.
- The same commit: steps RG12.63 (#3151, the system variables), RG12.64 (#3152, named esym.Real in the parameters,
  checked to work with the variables kept in Python beside the numbers of the C++ items), RG12.65 (#3153, the table of
  operations, with its stages .2 to .5); docs/dev/design/symbolicSystem.md section 12.7 with the refined parameters
  and the revised R1; docs/dev/design/symbolicUserItems.md section 6, the possible C++ items with numerical and
  symbolic parameters.

<a id="rg12-62"></a>
### RG12.62 — recordings with declared inputs, a VariableSet as scope (2026-10-29, #3149)
*(Maintainer 2026-10-28: "R2 is definitely accepted and shall be done"; on the scope: "would it help to assign the
variable set (or something better) to the named variables, so they would have a local scope? This would also enable
the function args".)*
- src/Linalg/Symbolic.h, `VariableSet`: besides the named Reals (SReal, which C++ reads) the named vectors and
  matrices as their Python objects, and the order in which the names were added; a name is unique in its set, not in
  the module. `GetNames` gives that order (was the order of an unordered_map), `NumberOfItems`, `Reset`, `ToString`
  over all kinds.
- src/Linalg/Symbolic.cpp, the Python interface: `Add(name, value)` - a number gives a named Real, a list or 1D array a
  named Vector, a 2D array a named Matrix - and `Add(variable)` of a named Real, Vector or Matrix return the variable
  (returned None); `Get`, `Set`, `[]` for all kinds. `esym.Function(result, inputs, name='Function')`: the inputs a
  list of named variables or a VariableSet (its variables in their order), names unique, every other named variable of
  the result refused unless it is the variable of esym.variables, which the function reads at every call; a
  PySymbolicUserFunction set up by SetupUserFunction from the dict of a recording, with the inputs as its arguments
  after the mbs placeholder that every user function has and `withoutMainSystem`, so that GetFlatProgramData and the
  export leave mbs out: `Evaluate(*inputs)`, `EvaluateJacobian(name, *inputs)`, `GetFlatProgram(format)` (def
  F(q, t, K)), `GetStatistics`. The errors are ExudynValueError (a ValueError).
- python/exudyn/misc/symbolicSystem.py: the composed system (#3131) by esym.Function of the inputs q, q_t, t in a
  VariableSet, optimized by esym.Optimize as CreateSymbolicUserFunction did, instead of the borrowed signature (mbs, t,
  itemNumber, q, q_t) of ObjectGenericODE2; the results of all symbolic system test models unchanged, the experiment of
  RG14.2.1 too.
- Not done of the issue text: CreateSymbolicUserFunction builds no VariableSet of the signature - the argument
  variables of each recording are its own nodes already, and the scope matters only where names meet, in Function and
  in the export.
- python/TestModels/symbolicFunctionTest.py; definitions/pybindSymbolic.py (Function, the VariableSet functions, whose
  NumberOfItems was described as "Return True, if variable name exists"), docs/manual/symbolicUserFunctions.md,
  docs/manual/revisions.md, the design note section 12.7.

<a id="rg12-51-19"></a>
### RG12.51.19 — the names of the equations in exudyn.misc.symbolicSystem (2026-10-29, #3155)
*(Maintainer 2026-10-28: "forces: I always distinguish between RHS and LHS forces, for a good reason (see the docs);
M*a + LHSforce = RHSforce; keep that to avoid sign-issues and consistency with the internal functionality.
ReactionForces: if only for ODE2, say so in the function name"; then "do ... the §12.5 renames (ODE2RHS,
ODE2ReactionForcesLHS, ...)".)*
- python/exudyn/misc/symbolicSystem.py: `Forces` -> `ODE2RHS`, `ODE1RightHandSide` -> `ODE1RHS`, `ReactionForces` ->
  `ODE2ReactionForcesLHS`, `Residual` -> `ODE2Residual`, `ResidualJacobian` -> `ODE2ResidualJacobian`, in
  SymbolicSystemBase, NumpySystem and SymbolicSystem and in the solvers; `ODE1ReactionTermsLHS(q, q_t, y, z, t)`, the
  term C_y^T z of the ODE1 equations, new in the base class, zero there, subtracted in the ODE1 rows of SystemResidual
  when there are algebraic coordinates. `Equations(..., ODE2RHS=True, ..., ODE2ReactionForcesLHS=False)` and its
  results 'ODE2RHS', 'ODE2ReactionForcesLHS' (were forces/'forces', reactionForces/'reactionForces'), the same names in
  GetFlatProgram(results) and the key of the composed recording; NumpySystem(massMatrix, ode2RHS, ..., ode1RHS, ...)
  (were forces, ode1RightHandSide). The module is in no release, so no deprecation.
- Kept: SystemResidual, SystemJacobian, AlgebraicEquations, MassMatrix, MassMatrixTriplets, MassMatrixBlocks (the
  names of the solver functions of Exudyn); the internal `_LocalForces`, `_LoadForces` of the items' forces;
  `_LocalODE1RightHandSide` is `_LocalODE1RHS`. Open: the letters of solver.md in the docstrings.
- The test models of the symbolic system, the examples symbolicSystemSpringDamper.py and
  symbolicSystemRigidBodyPendulum.py, docs/dev/design/uniformEvaluationExperiment.py, the design notes (section 12.5
  now the table of the names), docs/manual/revisions.md; all results unchanged.

<a id="rg12-51-20"></a>
### RG12.51.20 — the side of the residual in its name (2026-10-29, #3156)
*(Maintainer 2026-10-29: "ODE2Residual and ODE2ResidualJacobian should contain RHS or LHS (whatever they are) - so like
ODE2ResidualRHS. Otherwise, the sign is unclear.")*
- python/exudyn/misc/symbolicSystem.py: `ODE2Residual` -> `ODE2ResidualLHS`, `ODE2ResidualJacobian` ->
  `ODE2ResidualLHSJacobian`, in SymbolicSystemBase, SymbolicSystem and the solvers: the residual is M q_tt - f, all terms
  on the left-hand side, LHS minus RHS, as the docstrings say. The local function of the Newton loop of
  SolveGeneralizedAlpha, which is SystemResidual for a system with ODE1 or algebraic coordinates, is `NewtonResidual`.
  SystemResidual keeps the name of the solver function of Exudyn (its ODE1 rows are RHS minus LHS, as there).
- The test models of the symbolic system, the design note section 12.5, docs/manual/revisions.md; results unchanged.

<a id="rg12-65-2"></a>
### RG12.65.2 — stage 1 of the table of operations: SymbolicProgram (2026-10-29, #3157)
*(Maintainer 2026-10-28: "it is good to go with the first transition step"; 2026-10-29: "Do the next 3 steps you
mentioned, with the new source files. Regarding the symbolic table: if for efficiency or implementation reasons, it
would give slightly different results (so not bit for bit), it would also be ok ... for now this is a good idea to
check the full equivalence.")*
- New src/Linalg/SymbolicTable.h and SymbolicTable.cpp, in msvc/cppsrc.vcxproj (and .filters) and sources.json - the
  'all' list by gen_sources.py, the 'minimal' list by hand, as Symbolic.cpp needs it: `OperationCode` (Leaf, Node and
  the 40 scalar operations), the table `operationTable` - per code the name of its node (FlatName), the number of scalar
  operands and the rule of its forward derivative -, `OperationCodeOf` (a node of kind Real whose operands are all
  Real and whose name and number of operands are a row of the table; a leaf; else Node), and `SymbolicProgram`: built
  from a FlatProgram, its operations in the same order and slots, evaluated by one switch on the codes - the scalar
  operations read the slots of their operands and compute as the EvaluateFromOperands of their nodes, with the same
  functions (EM::Sin, std::fmod, ...) -, the leaves and the operations of vectors and matrices by their nodes; its
  Differentiate as FlatProgram::Differentiate, the rules of the scalar codes from the table.
- Moved from Symbolic.cpp to SymbolicTable.cpp: `DerivativeRule`, `FindDerivativeRule`, `SlotComponents` and the body
  of the rules as `ForwardDerivative(rule, node, value, ...)`, which FlatProgram::Differentiate now calls per
  operation; the code of the rules unchanged.
- SymbolicUtilities.h: SymbolicFunction and its thread copies keep a SymbolicProgram beside the FlatProgram, built with
  it; the evaluation of the result (number, vector, matrix, the sparse matrix result) and PyEvaluateJacobian use it when
  `symbolicProgramEvaluation` is set. `esym.SetTableEvaluation(flag)` and `esym.GetTableEvaluation()`; default True,
  as both paths gave the same bits on everything measured; False is the fallback.
- Checked: python/testing/test_symbolicTableEvaluation.py - every scalar operation of the table, its value and its
  derivative by both arguments, at three points (mod, min and max, which have no derivative, by value); vector and
  matrix operations with scalar operands; and the 35 test models that record symbolic user functions or use the
  symbolic system, each run with False and with True, their output (without the lines of times) and their result
  equal. The same bits everywhere, so no tolerance was needed.
- Measured (scratchpad timeTable.py): ObjectGenericODE2 with 20 coordinates and a recorded forceUserFunction of 8
  terms of sin, cos and exp per coordinate, 20000 steps: RK44 0.80 s -> 0.33 s, generalized-alpha 0.49 s -> 0.21 s.
- docs: the evaluation paragraph of the symbolic module (definitions/pybindSymbolic.py), revisions.md, the design note
  section 12.7.

<a id="rg12-63"></a>
### RG12.63 — the system variables and the marker states as their expressions (2026-10-29, #3151)
*(Maintainer 2026-10-28: "option 'C' is highly welcome"; "good to go as a new step"; 2026-10-29: "Do the next 3 steps
you mentioned".)*
- python/exudyn/misc/symbolicSystem.py, `SystemVariables(system)`: `variables`, an exudyn.symbolic.VariableSet of the
  named vectors q, q_t (initial state), y (initial ODE1 state), y_t, z (zeros) and the real t, in this order; `q`, ...,
  `t` its variables; `MarkerPosition`, `MarkerVelocity` (without the rotation functions of the body, so a marker on a
  mass point body needs only its position functions), `MarkerRotationMatrix`, `MarkerAngularVelocity` (with them):
  expressions of the system variables; `Function(result, name)`, esym.Function of the set, whose EvaluateJacobian
  gives the Jacobian of an expression of marker states in one pass; `Equations(**options)` of the system in the
  variables q, q_t, t, z. A marker the system does not take raises a ValueError naming it.
- In Python and not in the C++ module as `esym.SystemVariables(mbs)`, as the design note had it: the composition of a
  marker - a point node, the recordings of a user body, the ground - is in symbolicSystem.py. Data coordinates are
  left out, as SymbolicSystem refuses them.
- One composition: `_MarkerData(markerNumber, orientation)` (the dict of a marker, from the loop of
  `_ConnectorMarkers`, which now calls it; the type of the marker checked before its output is read) and
  `_MarkerState(marker, indices, ...)` (the state of one marker, from `_MarkerStateExpressions`, which now calls it
  per marker); the recording call `_Caller(format)` from `_EquationLists`. The expressions are built by the same
  operations in the same order, the results of the test models unchanged.
- python/TestModels/symbolicSystemVariablesTest.py: two user rigid bodies, a mass point, a spring-damper, a torsional
  spring-damper, a revolute joint; at a random state the positions, velocities, rotation matrices and angular
  velocities of 6 markers equal GetMarkerOutput (difference 0.0), the Jacobian of a distance by q against central
  differences 4e-11, the length of the spring 0.0, Equations() in the set equal to Equations() of the system.
- Open: `_ConnectorJacobian` and `_JointJacobian` compose their Jacobians by hand; by the set they would be the
  one-pass derivative of the composed functions - a change of the rounding, to be measured first (design note 12.7).

<a id="rg12-64"></a>
### RG12.64 — named exudyn.symbolic.Real as parameters of the user items (2026-10-29, #3152)
*(Maintainer 2026-10-28: "parameters ... could be a vector of named esym.Real ... recorded, exactly as they are defined
by the user, such as mass m1=5kg ... if m1 is shared across several elements, it would be fine ... differentiation ...
w.r.t. these quantities"; 2026-10-29: "Do the next 3 steps you mentioned".)*
- python/exudyn/misc/symbolicUserItems.py: `_Numbers` gives the values for the item; `UserItemCreator.parameterVariables`
  keeps, by the int of the item number, the parameters of an item with a named esym.Real among them: CreateMassPoint
  (mass), CreateSpringDamper (stiffness, damping, referenceLength, force, velocityOffset), CreateTorsionalSpringDamper
  (stiffness, damping, offset, velocityOffset, torque). Not the inertia of CreateRigidBody nor the joints.
- python/exudyn/misc/symbolicSystem.py: `SymbolicSystem(mbs, composeSystem, parameters)`; `_ParameterVariables` checks
  the number of the parameters, that each value is that of the item and that a name is one variable, and keeps
  `parameterVariables`; `_ParameterVector` gives Equations the variables and numbers of an item - for the user bodies,
  the markers on them, the loads on them, the connectors and the joints -, sympy the symbol of the name
  (`_Backend.Parameter`); the composed system of the evaluation keeps the numbers (`_Backend(...,
  parameterVariables=False)`). `Equations()['parameters']` lists the variables; `GetFlatProgram` takes them as
  arguments after q, q_t, t (and z). `SetParameterValues(values)`: the values of the variables, the numbers of the
  items by SetObjectParameter, the system built again from mbs.
- python/TestModels/symbolicSystemParametersTest.py: two mass points of mass m1, a spring of stiffness k; the forces
  and the mass matrix of Equations() equal those of the system (0.0), their derivatives by m1 and k by one
  EvaluateJacobian against central differences by SetParameterValues (1.6e-10), the numpy code with m1 and k as
  arguments (0.0), a value that is not that of the item refused.
- docs: revisions.md, the design note section 12.7, the Details of both modules.

<a id="rg12-65-3"></a>
### RG12.65.3 — stage 2 of the table: Optimize, the export and the count (2026-10-29, #3160)
*(Maintainer 2026-10-29: "do RG12.65.3 now".)*
- src/Linalg/SymbolicTable.h: the column `commutative` of the table (+, *, ==, !=); `ScalarOperation(code, x)`, the
  value of a scalar operation from its operands x(i), a template inlined into its users - the switch of
  SymbolicProgram::Evaluate moved there; `OperationCodeOfNode(node)` (the kinds of the operands read from the node),
  `IsTableCode`, `OperationName(node)`.
- src/Linalg/Symbolic.cpp, Optimizer: a constant scalar operation is folded by ScalarOperation (an operation of its
  node, as the component of a vector of numbers, by its node as before); the rules of *, +, -, /, pow, pos, neg and
  IfThenElse select by the code instead of the C++ class (`Is<ExpressionOperatorMul>`); the key of an operation is
  its code for the table and its class otherwise, and the commutative operations are those of the column. The rules of
  the vector and matrix operations and of the components stay with their classes, which have no codes yet.
- FlatProgramData (the export) and StatisticsName (the count) take the name of the table; GetStatistics gives
  `tableOperations`, the operations of the flat program computed by the table; symbolicStatisticsTest.py checks it.
- All test models unchanged, among them symbolicOptimizeTest.py and symbolicExportTest.py to the bit.

<a id="rg12-55-7"></a>
### RG12.55.7 — the drawing of ObjectJointUser and of the joints of UserItemCreator (2026-10-29, #3147, #3161)
*(Maintainer 2026-10-29: "do RG12.55.7: as mentioned, drawing could be realized in postprocessing with the native C++
objects (when they exist); so a simplified graphics representation of joints and connectors is sufficient".)*
- definitions/itemDefsObjects.py, ObjectJointUser: the visualization parameter `graphicsDataUserFunction(mbs,
  itemNumber)`, returning graphics data in global coordinates, with HasUserFunction and CallUserFunction as
  ObjectConnectorUser (#3119); src/Graphics/VisualizationUserFunctions.cpp, VisualizationObjectJointUser::CallUserFunction.
- #3161, found here: PostProcessData::ProcessUserFunctionDrawing called the graphics user functions of the bodies only
  (`!item->IsConnector()`, and only with bodies.show), so the graphicsDataUserFunction of ObjectConnectorUser was never
  called and the spring-dampers of UserItemCreator were not drawn - GetGraphicsData had no element of them. Now each
  item with a graphics user function is drawn with bodies.show or, for a connector, with connectors.show.
- python/exudyn/misc/symbolicUserItems.py, `_JointGraphics`: the revolute joint a cylinder along the z-axis of marker 0
  (axisRadius, axisLength), the prismatic joint along its x-axis, the spherical joint a sphere (jointRadius), the
  distance constraint a line or, with drawSize > 0, a cylinder between the markers; color, grey by default; the markers
  read in the visualization configuration. The arguments were accepted and not used.
- python/TestModels/symbolicUserItemsDrawingTest.py: the six connectors and joints in GetGraphicsData, none with
  connectors.show = False, the line of the torsional spring-damper moves with its body.
- python/TestModels/parameterConversionTestReference.txt rewritten (recordReference): the new parameter
  VgraphicsDataUserFunction of ObjectJointUser in two rows, as for ObjectConnectorUser.

<a id="rg12-51-21"></a>
### RG12.51.21 — the ODE1 equations in Equations() and GetFlatProgram() (2026-10-29, #3162)
*(Maintainer 2026-10-29: "add the ODE1 equations in Equations()".)*
- python/exudyn/misc/symbolicSystem.py: `_ODE1Equations(backend, y, t)`, the right-hand side g of the ODE1 equations as
  expressions, as ODE1RHS computes it: per ObjectGenericODE1 A y + f and its recorded rhsUserFunction, the
  LoadCoordinate on ODE1 coordinates with or without its recorded loadUserFunction. `Equations(..., ODE1RHS=True)`:
  the result 'ODE1RHS' and the variable 'y' (the initial ODE1 state; or the fifth of `variables`);
  `GetFlatProgram(results=[..., 'ODE1RHS'])` writes Equations(q, q_t, t[, z], y, ...); `SystemVariables.Equations`
  passes y.
- python/TestModels/symbolicSystemODE1Test.py: the expressions and the numpy code equal ODE1RHS (0.0).

<a id="rg12-68"></a>
### RG12.68 — the package exudyn.symbolics (2026-10-29, #3165)
*(Maintainer 2026-10-29: "give the symbolic Python parts more space ... create a symbolic directory"; on the names:
exudyn.symbolics "shorter and clear that it would take everything (including the utilities)"; system.py and export.py
replaced by specific names; "symbolicUtilities.py was not part of 1.13.6 ... still ok to move".)*
- python/exudyn/symbolics/ (git mv, the history kept): `kinematics.py` (was exudyn/symbolicUtilities.py),
  `codeExport.py` (misc/symbolicExport.py), `userItems.py` (misc/symbolicUserItems.py), `systemModel.py` and
  `solvers.py` - misc/symbolicSystem.py split: SymbolicSystemBase, NumpySystem, SymbolicKind, SymbolicLoad,
  SymbolicSystem, SystemVariables and `_denseMassMatrixSize` in systemModel, explicitSolverTypes, SolveExplicit,
  SolveGeneralizedAlpha and their helpers in solvers, which imports systemModel. `__init__.py` imports nothing - the
  modules are imported one by one, as systemModel and solvers need scipy - and lists them.
- No deprecation: none of the modules was in a release (1.13.6).
- References: setup.py (the package), Symbolic.cpp (the import of codeExport by GetFlatProgram, comments),
  definitions/pybindSymbolic.py, tools/generators/utilityDocsModel.py (the pages symbolics.*; the four old generated
  pages removed), pyproject.toml, the test models, the examples, docs/dev (README, internals/symbolic.md, the design
  notes) and docs/manual; the imports of misc.symbolicSystem split into systemModel and solvers. The closed plan steps
  and log entries keep the old names, as they were then.

<a id="rg9-10-1"></a>
### RG9.10.1 — ObjectGenericDAE, the design note (2026-10-29, #3143)
*(Maintainer 2026-10-29: "RG9.10 is good to go now"; the step says: first a design note with the equations, the C++
class and the changes of the solvers.)*
- docs/dev/design/genericDAE.md: the equations with Exudyn's LHS/RHS convention, the parameters (the names of
  ObjectGenericODE2 and ObjectGenericODE1 and the new coupling and constraint matrices), the user functions with
  (mbs, t, itemNumber, q, q_t, y, ..., parameters), the C++ class as a body with several nodes, ODE1 coordinates and
  algebraic equations of its own.
- Read in the C++: CSystem::JacobianODE2RHS adds the block ODE2-ODE1 only numerically and only in the branch of a
  numerical ODE2 Jacobian; CSystem::NumericalJacobianODE1RHS is the only Jacobian of the ODE1 rows; CObject has no
  analytic ODE1 Jacobian. CSolverImplicitSecondOrder::ComputeNewtonResidual has the ODE1 rows g - y_t without
  C_y^T lambda, while CSystem::JacobianAE puts the transposed AE_ODE1 block into the ODE1 rows of the Jacobian, and
  ComputeODE2ProjectedReactionForces raises for AE_ODE1; no item gives AE_ODE1, so no present model is affected.
- Sub-steps RG9.10.2 to .5: the solver, the item, the symbolic user functions, SymbolicSystem. Decisions asked: the
  analytic ODE1 Jacobians (recommended) or numerical blocks as interim; one jacobianUserFunction of all blocks.

<a id="rg9-10-2"></a>
### RG9.10.2 — the solver for ObjectGenericDAE: analytic ODE1 Jacobians, C_y^T lambda (2026-10-29, #3167)
*(Maintainer 2026-10-29: "yes, Analytic Jacobians; new C++ functions (only required in case that ODE1 is nonzero, so
little overhead)"; "jacobianUserFunction for all blocks: ... one for all".)*
- src/System/CObject.h: `ComputeJacobianODE1RHS(jacobianODE1, jacobianODE2, jacobianODE2_t)` - the ODE1 RHS by the
  local ODE1 coordinates, and by the ODE2 coordinates and velocities if ODE1_ODE2_function and ODE1_ODE2_t_function are
  set - and `ComputeJacobianODE2_ODE1(jacobian)` - the ODE2 LHS by the local ODE1 coordinates -, called for an object
  that sets ODE1_ODE1_function or ODE2_ODE1_function (the flags existed, unused).
- src/Main/CSystem.cpp: `NumericalJacobianODE1RHS` renamed `JacobianODE1RHS`, with the analytic branch per object
  (connectors stay numerical); CSystem.h had an empty `JacobianODE1RHS` "used in future", which
  MainSolverBase::ComputeJacobianODE1RHS - `solver.ComputeJacobianODE1RHS` in Python - called, so it added nothing;
  removed. `JacobianODE2RHS`: the coupling block ODE2-ODE1 also for an object with an analytic ODE2 Jacobian, its own or
  numerical; before, such an object got none. `ComputeODE1ProjectedReactionTerms(reactionForces, ode1ReactionTerms,
  factor)` over the new list `listObjectProjectedReactionTermsODE1` (objects with algebraic equations, ODE1 coordinates
  and AE_ODE1); `ComputeODE2ProjectedReactionForces` no longer raises for AE_ODE1. `JacobianAE`: the block AE_ODE1 at
  the AE rows and ODE1 columns, its transpose at the ODE1 rows and AE columns - they were at the ODE1 rows and ODE2
  columns, the factors of the solver were overwritten by 1, and objects with ODE1 coordinates raised; no item had them.
- Solvers: CSolverImplicitSecondOrder subtracts C_y^T lambda in the ODE1 residual (g - y_t - C_y^T lambda, as
  SymbolicSystemBase.SystemResidual, #3155) and passes factorODE1_AE = -scalAE; CSolverStatic adds +C_y^T lambda, as
  its ODE2 rows do, and leaves it out with constrainODE1Coordinates.
- ObjectGenericODE1 (definitions/itemFunctions.py, itemDefsObjects.py, CObjectGenericODE1.cpp): ODE1_ODE1_function
  without rhsUserFunction, the Jacobian its system matrix.
- python/TestModels/genericODE1JacobianTest.py: the Jacobian equals A exactly (by the user function 3.3e-8), the
  implicit solution the same to 1.3e-11, one Newton step per step with both. genericODE1duplicateNodeTest.py now gives
  its equations by a rhsUserFunction, so it still checks the numerical Jacobian of #1424; its result unchanged.
- The part C_y^T lambda has no item yet to check it; the test models of ObjectGenericDAE do (RG9.10.3).
- taskmanagerTest.py: its ObjectGenericODE1 now with the analytic Jacobian, the result -0.23406814273662005 (before
  -0.23406814272950313, 7e-12). The description of ObjectGenericODE1 says which Jacobian the solvers take.
- docs/dev/design/genericDAE.md sections 3 and 4.

<a id="rg12-51-4-3"></a>
### RG12.51.4.3 — the Jacobians of connectors and joints by the one-pass derivative (2026-10-29, #3166)
*(Maintainer 2026-10-29: "yes, but they could also be *nodes* with local q and q_t ... no problem if some of these
expressions are not bit-identical"; "for body markers, probably better per body and for node markers, probably better
per node. If 'always nodes' is easier to implement, that is also ok ... ltg-maps should exist for both cases".)*
- python/exudyn/symbolics/systemModel.py: `SymbolicSystem(mbs, ..., itemJacobians='chainRule'|'onePass')`, kept by
  SetParameterValues. `_ItemFunction(kind, i, joint)`: per connector one `esym.Function` of its generalized forces
  (J_pos^T f + J_rot^T m of its markers), per joint of [c, C_q^T z]; the variables `q<g>`, `q_t<g>` per body or node g
  of its markers - their local indices into the item's ltg, so the columns go to the system by kind.ltg -, one variable
  of all coordinates if the groups overlap (a node marker on a node of a body marker), `z` (joint) and `t`; the
  expressions by `_MarkerStateExpressions` and `_AddMarkerForces`, as Equations(); built at the first use, cached
  per item. `_ItemJacobians`: one EvaluateJacobian per variable q<g> (and q_t<g> for the damping);
  `_ConnectorJacobianOnePass`, `_JointJacobianOnePass` return what `_ConnectorJacobian`, `_JointJacobian` return.
- Dependencies checked: ObjectConnectorHydraulicActuatorSimple depends on ODE1 coordinates (its forces on the
  pressures, its ODE1 RHS on q and q_t); joints on z (c_i = z_i on free axes and for inactive joints in the C++
  joints; C_q^T z); connectors may get data coordinates. SymbolicSystem takes ObjectConnectorUser and ObjectJointUser
  only, which have none of the first and the reaction forces as z; ODE1 and data coordinates would be further
  variables of the same Function.
- python/TestModels/symbolicSystemItemJacobiansTest.py: rigid bodies on revolute, prismatic and spherical joints, a
  distance constraint, spring-dampers between bodies, mass points and the ground, a torsional spring-damper. Jacobian
  of the system onePass against chainRule 1.2e-19 relative; per evaluation 0.49 ms against 2.8 ms (the first, which
  builds the functions, 14.5 ms); SolveGeneralizedAlpha 0.15 s against 0.24 s, the solutions equal to 7e-15.
- The default stays 'chainRule' until the maintainer decides; with 'onePass' as the default, `_ConnectorJacobian`
  and `_JointJacobian` (about 120 lines) could go.
- docs/dev/design/symbolicSystem.md section 12.7, revisions.md.

<a id="rg12-65-6"></a>
### RG12.65.6 — the derivative as new expressions: esym.Jacobian (2026-10-29, #3163)
*(Maintainer 2026-10-29: "Then do your suggested 1., 2. and 3. steps"; RG12.65.6 independent of stage 3 since option A.)*
- src/Linalg/SymbolicTable.cpp: `SymbolicJacobian(expression, variable)` - the flat program of the expression, evaluated
  once for the sizes; per operation and per component j of the variable its derivative as a new expression,
  `SymbolicDerivativeOfOperation(rule, ...)`, by the rule `FindDerivativeRule` gives - the rules of `ForwardDerivative`,
  which computes their numbers: the scalar functions, IfThenElse, pow (the term of the exponent only if it depends on
  the variable), atan2, dot, norm, components, the vector and matrix sums, scalings, products, the products of
  components and the transpose, as operations of the same kind on the derivatives. A zero derivative is kept as zero,
  not as a node, so the cost follows the dependencies. A leaf: the unit vector of the variable, else its constant Diff.
  An operation without a rule - abs, min, max, mod, which the one-pass derivative takes by its Fallback - raises. The result: a
  Real; a Vector of the derivatives by the components; for a Vector a Vector (by a Real) or the Jacobian Matrix of
  the components. `FlatProgram::SlotValue(k)` in Symbolic.h.
- Symbolic.cpp, definitions/pybindSymbolic.py: `esym.Jacobian(expression, variable, optimize=True)`, Optimize level 1.
- python/TestModels/symbolicJacobianExpressionsTest.py: all rules, against EvaluateJacobian 1.5e-16 (by the vector) and
  0 (by a Real); the Jacobian of the Jacobian of a scalar equal to its exact Hessian (1.4e-17); abs raises; a number
  has the zero Jacobian. Sizes and times: the function 69 flat operations, its Jacobian 280; one evaluation of the
  Jacobian program 2.7 us against 4.4 us of EvaluateJacobian (3 components).
- The Jacobian of a recorded user function as its own program, for the export of the Newton matrix and the GPU, uses
  this on the recording's expressions; it is not wired into the items yet (RG14, RG12.51.5).
- New sub-step RG12.65.7 (#3168): sparse matrices in the table (maintainer 2026-10-29).
- docs/dev/design/symbolicSystem.md sections 2 and 12.7, the module description, revisions.md.

<a id="rg12-51-4-4"></a>
### RG12.51.4.4 — the one-pass Jacobians the only method (2026-10-29, #3169)
*(Maintainer 2026-10-29: "RG12.51.4.3: yes, implement your recommendation with onePass and remove the chain-rule code".)*
- python/exudyn/symbolics/systemModel.py: the argument `itemJacobians` of SymbolicSystem removed (introduced by #3166,
  not released, no deprecation); the chain-rule `_ConnectorJacobian` and `_JointJacobian` removed, the one-pass
  versions take their names; `_ConnectorStates`, `_JointArguments`, `_RecordingJacobian` and `_BodyJacobian` stay for
  the equations and the loads.
- python/TestModels/symbolicSystemItemJacobiansTest.py: the Jacobian against the central differences of
  SymbolicSystemBase (1.0e-13), SolveGeneralizedAlpha against SolveDynamic (5.1e-12); symbolicSystemJointsTest.py
  (1.2e-13, 1.1e-11) and symbolicUserRigidBodiesTest.py (9.1e-11) unchanged in their results.
- docs/dev/design/symbolicSystem.md, revisions.md.

<a id="rg9-10-3"></a>
### RG9.10.3 — ObjectGenericDAE, the item with its matrices and the Python user functions (2026-10-30, #3170)
*(Maintainer 2026-10-29: "Then also do RG9.10.3 and RG12.65.5".)*
- definitions/itemDefsObjects.py: ObjectGenericDAE, a CObjectBody (Body + MultiNoded) on `nodeNumbersODE2`,
  `nodeNumbersODE1` and `nodeNumbersAE` - the NodeGenericAE carry the multipliers, as CSystem gives a body algebraic
  coordinates only through its nodes (the design had numberOfAlgebraicEquations, coordinates of the object; that
  would need a change of the assembly) -; the dense matrices massMatrix, dampingMatrix, stiffnessMatrix,
  couplingMatrixODE1 (E), systemMatrix (A), couplingMatrixODE2 (B), couplingMatrixODE2_t (B_t), constraintMatrixODE2
  (C_q), constraintMatrixODE1 (C_y), the vectors forceVector, rhsVector, constraintVector, `parameters`;
  forceUserFunction and rhsUserFunction (mbs, t, itemNumber, q, q_t, y, parameters). definitions/itemFunctions.py:
  ComputeJacobianODE1RHS for every object.
- src/ImplObjects/CObjectGenericDAE.cpp (new; msvc project, filters, sources.json): the node lists, the sizes, the
  coordinates, the mass matrix, the ODE2 LHS K q + D q_t + E y - f - f_u, the ODE1 RHS A y + B q + B_t q_t + g + g_u,
  the algebraic equations C_q q + C_y y - c0 (index 2: C_q q_t + C_y y_t), the Jacobians of the matrices
  (ComputeJacobianODE2_ODE2, ComputeJacobianODE2_ODE1, ComputeJacobianODE1RHS, ComputeJacobianAE); GetAvailableJacobians:
  a block with a user function without its _function flag, so numerical; the output variables Coordinates and
  Coordinates_t by GetObjectOutputBody, as for every body. evaluateUserFunctions.cpp: EvaluateUserFunction;
  checkPreAssembleConsistenciesObjects.cpp: the node groups of the lists and the sizes of all matrices and vectors.
- Found while checking against SolveGeneralizedAlpha: the initial accelerations of CSolverImplicitSecondOrder took
  the block AE_ODE1 into the ODE1 rows of the initial system (since #3167 at its place), which made the start differ
  from SolveGeneralizedAlpha and the solution by O(h) (7e-3 at 100 steps, 4.7e-4 at 1600); the initial system keeps the
  ODE1 rows a unit matrix (factors 0 for the ODE1 blocks in FactorizeInitialSystemJacobian). The warning of
  NumericalConstraintJacobianTimesVector for AE_ODE1 removed: that block does not enter C_q v.
- python/TestModels/genericDAETest.py: the ODE2 part against ObjectGenericODE2 1.4e-16, the ODE1 part against
  ObjectGenericODE1 0, the constraint against a CoordinateConstraint 4.2e-17, the coupled system with all matrices
  against NumpySystem with C_y^T z by SolveGeneralizedAlpha 2.3e-13, with the user functions 2.1e-13 - the first
  check of the term C_y^T lambda of #3167.
- The constraintUserFunction, massMatrixUserFunction and jacobianUserFunction of the design wait for RG9.10.4, as
  their Jacobians must come from the one-pass derivative, not from differences (#3101).
- Reference values: symbolicUserFunctionSignaturesTest.py +2 (the two signatures), parameterConversionTestReference.txt
  rewritten by recordReference (9 rows of the new item); the graphics reference of the mini example written.
- docs/dev/design/genericDAE.md section 2, revisions.md.

<a id="rg12-65-5"></a>
### RG12.65.5 — stage 4 of the table: the old flat evaluation and the tree derivative removed (2026-10-30, #3171)
*(Maintainer 2026-10-29: "Then also do RG9.10.3 and RG12.65.5"; earlier: "the old code stays during the transition as
fallback".)*
- src/Linalg/Symbolic.h: FlatProgram keeps the operations, their operands and the results - its `slots`, `Evaluate`,
  `Differentiate`, `Result*` and the members of the derivative removed; the tree derivative removed: `Diff(const
  DiffSeed&)` and `DiffRef(const DiffSeed&)` of 71 node classes in Symbolic.h, SymbolicVector.h and SymbolicMatrix.h
  (by a script that removes each function with its body and checked by the diff), their `diffBuffer`s and `DiffSeed`;
  `SReal::DiffSReal` is now the one-pass derivative. About 650 lines less.
- src/Linalg/SymbolicTable.h/.cpp: the global `symbolicProgramEvaluation` removed; the rules: `Fallback` is now
  `NoDerivative` - zero if no operand depends on the variable, else it raises "the operation 'mod' has no derivative"
  (before, the Diff of mod, min and max raised always) -, a leaf `Zero` (the variable `Variable`), `abs` the new rule
  `RealAbs` (nan at 0, as its Diff), also in esym.Jacobian; `ForwardDerivative` and `SymbolicProgram::Differentiate`
  without the arguments of the fallback; `OnePassDerivative(root, variable, n, derivatives)`, `SlotValue`,
  `ResultComponents`. esym.Jacobian takes its sizes from SymbolicProgram.
- src/Linalg/SymbolicUtilities.h: evaluation by SymbolicProgram only; EvaluateJacobian always by the table (also with
  flatEvaluation = False), a result without operations by the unit matrix or zero. Symbolic.cpp: SetTableEvaluation and
  GetTableEvaluation removed (#3157, not released); Diff of Real, Vector and Matrix by `OnePassDerivative`.
- python/testing/test_symbolicTableEvaluation.py: the table against the tree evaluation (the same bits), the one-pass
  derivative against central differences, esym.Jacobian and Diff, mod/min/max raising only where needed.
- src/Linalg/symbolicCppDemo.h and src/Tests/SymbolicUnitTests.h keep DiffSReal.
- The tree evaluation (Evaluate(), flatEvaluation = False) stays, as the design has it.
- docs: the module description (definitions/pybindSymbolic.py), revisions.md, internals/symbolic.md, design note 12.7.

<a id="rg11-10-9"></a>
### RG11.10.9 and RG11.10.13 — EP2RotationMatrix_t removed; Transformation66 replaced, KinematicsBasics.h removed (2026-10-31, #2965, #3172)
*(Maintainer 2026-10-31: "EP2RotationMatrix_t: I think it is not used, so remove it from C++; EP_t2Glocal_t and EP_t2G_t
are identical maps for EP, so they are not required in cUtilities. Then close the issue." - "First, the typedef shall be
removed and directly replaced by the HomogeneousTransformation and a HomogeneousTransformationList ... Then the file
KinematicsBasics.h shall be removed. ... Transformation66 is also used in definition files".)*
- RG11.10.9: `RigidBodyMath::EP2RotationMatrix_t` and its template had no caller; removed. `EP_t2G_t` and
  `EP_t2Glocal_t` are `EP2G` and `EP2Glocal` evaluated at ep_t (G is linear in the Euler parameters), so cUtilities does
  not get them; in C++ they stay, CNodeRigidBodyEP uses them. #2965 closed as not needed.
- RG11.10.13: `typedef ResizableArray<HomogeneousTransformation> HomogeneousTransformationList` in
  src/Linalg/HomogeneousTransformation.h; CObjectKinematicTree.cpp (12 uses), definitions/itemDefsObjects.py (the two
  temporary lists of ObjectKinematicTree, the arguments of two functions, the include of the header now
  RigidBodyMath.h), definitions/definitionTypes.py (`THomogeneousTransformationList`, its default rendering) and
  test_defaultValueRenderings.py on the new names; the includes of KinematicsBasics.h in VisualizationItemHelpers.h,
  CObjectANCFBeam.cpp, CObjectBeamGeometricallyExact.cpp, MainObjectFactory.cpp became RigidBodyMath.h (the one file it
  included), in CObjectConnectorRigidBodySpringDamper.cpp it was redundant; `PyTransformation66List` of PyMatrixVector.h
  was never bound and is removed; KinematicsBasics.h removed with its entries of msvc/cppsrc.vcxproj and .filters. The
  generated CObjectKinematicTree.h follows by regeneration. The closed plans and logs keep the old name.

<a id="rg11-8-10"></a>
### RG11.8.10 — model-centric and camera-centric views in the manual (2026-10-31, #2247)
*(Maintainer 2026-10-31: "Complete the OPEN_RG11.8.10.")*
- docs/manual/GUI.md, chapter The model view: a section `sec-renderer-modelcameracentric` with a table of the two views -
  what moves (modelRotation and centerPoint move the model; cameraPosition and modelRotation the camera), what is visible
  (near and far plane at 2 maxSceneSize zMaxSceneFactor around the focal plane; at min(0.1, 0.001 maxSceneSize) from
  the eye), the perspective (the eye at zoom/perspective; the zoom of a lens), ZoomAll (sets cameraPosition in a
  camera-centric perspective view) and trackMarker (cameraPosition relative to the tracked position, in model
  coordinates: `translationMV += rotationMV*cameraPosition` before the marker orientation is applied) - read from
  ComputePerspectiveFrustum, ComputeZoomAll and GetRotationTranslationFWithMarker in VisualizationSystemContainerBase.h;
  nearFarPlaneOffset, the raytracer (#3000) and the shadows (#2308); an example of a camera following a vehicle. The
  paragraph of the section Tiny and huge models on the perspective of both views moved into it.
- definitions/structureDefsVisualizationSettings.py: the description of modelCentricView links the section.

<a id="rg12-51-22"></a>
### RG12.51.22 — the Python solvers write the state into mbs by references (2026-10-31, #3173)
*(Maintainer 2026-10-31: "The Step(...) function copies the current states into the system states in every time step.
There is mbs.systemData.GetSystemStateDict(...,reference=True) ... I wonder if copying in Step() can be avoided; or we
could update only regarding the solution write period and regarding a given updating scheme. Certainly, one could miss
some states in the internal solver functions, which needs to be checked".)*
- Measured (scratchpad, mass points, 20000 calls): SetODE2Coordinates, _t, _tt and SetTime 3.2 us for 30, 18 us for
  300 and 168 us for 3000 ODE2 coordinates - the setters take a std::vector, converted element by element -; writing
  the arrays of `GetSystemStateDict(reference=True)` in place 0.8, 0.8 and 1.6 us; FinishStep of MainSolverExplicit
  with nothing to write 0.45 to 0.65 us.
- Which states FinishStep reads: the current state and time, for the solution file (writePeriod), the sensors
  (sensorsWritePeriod, also storeInternal), the renderer (copied into the visualization state at its update), the
  image recording, and the state after the solve. The recordings of SymbolicSystem do not read mbs (refused by
  `_Recording`), so nothing else needs the state during the steps. Writing only at the due steps would have to repeat
  the rules of FinishStep in Python (two periods, the renderer, the last step) for at most the 0.8 us that remain:
  not done.
- python/exudyn/symbolics/solvers.py: `_Output` takes the references after InitializeSolver and writes q, q_t, q_tt,
  y, y_t and z into them, the time by SetTime; results of the 10 test models with SolveExplicit or
  SolveGeneralizedAlpha unchanged (test suite).
- definitions/pybindGeneralInformation.py, section Copying and referencing C++ objects: GetSystemStateDict(reference=True)
  and GetODE2Coordinates(copy=False), valid until the next Assemble, unchecked writes, the time not among them.
- Seen: the setters of the state (SetODE2Coordinates and the others, MainSystemData.h) convert through std::vector;
  taking a numpy array directly would make them about as fast - not changed here.

<a id="rg12-69-1"></a>
### RG12.69.1 — exudyn.HT for symbolic arguments: the analysis (2026-10-31, #3174); RG12.65.7 proposal, RG12.65.8, RG12.70
*(Maintainer 2026-10-31: "it would be good to have that features directly available in symbolic ... similar for
exudyn.HT? ... A clear design decision would be whether HT transformations are immediately mapped to the underlying
linear algebra transformations, or there could be native HT operations and functions ... I ask for a new step and a
first substep with an analysis, done right now." - "RG12.65.7: make a proposal first." - "codes for vector and matrix
operations ... add a RG step and describe there a little bit more clearly". - The functions of time: "Make a suitable
suggestion.")*
- docs/dev/design/symbolicHT.md: what exists (exudyn.HT, the 4x4 and T66 functions of rigidBodyUtilities,
  cUtilities, symbolics.kinematics, the cMath dispatch), the interfaces of ObjectConnectorUser and ObjectJointUser
  now, the two ways - A mapped at once to esym.Matrix and esym.Vector, B a native kind with codes - with their cost
  in the table, the derivatives, Optimize, the export and RG14; recommended A as a class with the method names of
  exudyn.HT (A1 Python, A2 exudyn.HT dispatching); the twist [v, omega], global at the marker origin, and its
  operations with HT; the functions in groups; the new signatures (both items unreleased, #2975, #2976); five
  decisions.
- RG12.65.7: the proposal for sparse matrices in docs/dev/design/symbolicSystem.md section 12.7 (declared, compressed
  rows, six codes, the derivative on the pattern, equal to the bit, triplets, after RG12.65.8); #3168 extended.
- RG12.65.8 (#3176): the 23 classes of vector and matrix operations, their buffers, why the codes - threads, GPU,
  export -, in the same section; "node classes" are the classes of the nodes of the expression graph.
- RG12.70 (#3175): the suggestion for SampleTime, Signal and Noise in the step itself.
- RG12.51.4 closed: all its sub-steps are closed; what is left of it has steps of its own.

<a id="rg12-65-8"></a>
### RG12.65.8 — the vector and matrix operations as codes of the table (2026-10-31, #3176); RG11.11.4 (#3177); RG12.69 revised
*(Maintainer 2026-10-31: "raise an issue for the apply-proposed and resolve it. Then do RG11.10.13, RG12.65.8"; on
exudyn.HT: "we are still in the evaluation stage ... make a final revision of symbolicHT.md with some decisions left to
the open questions".)*
- RG11.11.4 (#3177): the issue store of test_issueTracker.py copies proposed/ without its files; the waiting proposal
  of #3164 had made testProposedResolutionsAreAppliedAfterTheMerge fail. #3164 itself applied by the maintainer, committed.
- RG11.10.13 was already closed (#3172, commit 75551cc3); RG11.10 closed, all its sub-steps are.
- RG12.65.8: OperationCode has 23 codes more (Dot, Norm, VectorComponent, VectorEqual, VectorNotEqual, MatrixComponent;
  VectorOfComponents ... VectorMultiply; MatrixOfComponents ... MatrixTransposed); the row has the kind of the result
  and the kinds of the operands, the code is found by the key kind-name-operands ("*" for the components).
  ArrayOperation mirrors the EvaluateFromOperands of each node (the loops of MultMatrixVectorTemplate,
  MultMatrixTransposedVectorTemplate, the dot product of VectorBase); ArrayResultSize checks the sizes at the layout
  with the messages of the nodes. SymbolicProgram: SlotValue instead of FlatValue, one array `memory`, `Layout()` when
  a leaf changes its size, the leaves read by Evaluate()/EvaluateRef(), the results copied into a vector or matrix on
  reading. ForwardDerivative and esym.Jacobian on SlotValue; FindDerivativeRule and SlotComponents removed; Optimize
  folds a Real of constant vectors by ArrayOperation. Build raises for an operation without code.
- Checked: the suite (244 + 99) passes, symbolicStatisticsTest.py with tableOperations 5 instead of 1; pytest 1452
  passed; the values of the three recordings of symbolicOnePassJacobianTest.py equal the tree to the bit in 20 random
  states; measured against the previous build: 2.40 instead of 2.61 us (5x5 mass matrix), 2.26 instead of 2.35 us
  (force of vectors and matrices) per evaluation.
- RG12.69: symbolicHT.md revised after the decisions (mapped, the class first, the twist global at the marker origin,
  the signatures): the carrier of HT0 and twist0 (section 4: two leaves per HT made by the recording, one 6-vector per
  twist; no C++ class), the reduced interface of exudyn.HT (six functions), the minimal group for the joints and
  connectors of the Create functions, four open questions; RG12.69.2-.5 added. Found: exudyn.HT.__mul__ converts any
  other operand into a vector, so that exu.HT * symbolic HT raises instead of reaching __rmul__ - part of RG12.69.3.


<a id="rg12-69-2"></a>
### RG12.69.2 — exudyn.HT without the six functions (2026-10-31, #3181); the decisions on HT, RG12.70 revised
*(Maintainer 2026-10-31: on symbolicHT.md section 8 "all six"; the leaf names yes, "for the twist, it would be highly
valuable to have a clear interface (ideally twist.translation and twist.rotation) ... suggest something appropriate";
A2 "add this to the steps and do it as planned after A1"; the twist functions "in C++ and in symbolics.kinematics for
now", an issue without step to check which are needed; on RG12.70 "currently, we only need Signal", "Signal is not the
correct name"; "Then do the suggested RG12.69.2-.4 steps".)*
- The decisions in symbolicHT.md sections 5, 7 and 8: the twist an object with `velocity` and `angularVelocity`
  (`exudyn.Twist` in C++, `kinematics.Twist`), recorded as two leaves `twist0_velocity`, `twist0_angularVelocity`;
  the names of the output variables rather than translation and rotation, which are the parts of a frame.
  RG12.69.6 (A2) added; #3178 raised without step.
- RG12.70 rewritten (#3175 retitled and its description replaced): `esym.LookupTable(values, step, start, outside,
  interpolation)` called as `table(x)`, required and optional features separated; SampleTime and Noise in a new
  module of exudyn.symbolics; RG12.70.1 (#3180) the SampleSensor, later.
- RG11.6.1 (#3179): localPosition of the rigid body markers and rotationMarker0/1 documented as deprecated, later.
- RG12.69.2: `Get`, `SetIdentity`, `SetTranslation`, `Invert`, `RotateVector`, `RotateVectorTransposed` out of
  pybindDataStructures.py and their Py helpers out of PyHomogeneousTransformation.h (the C++ methods stay, MarkerData.h
  uses them); homogeneousTransformationTest.py checks `Relative` instead of the rotation of a vector (the same result
  value), the notebook HT.ipynb reads both parts by the properties; revisions.md.

<a id="rg12-69-3"></a>
### RG12.69.3 — the symbolic HT and Twist, exudyn.Twist and the twist functions (2026-11-01, #3182)
- C++: `Twist` (velocity, angularVelocity, global) and `RigidBodyMath::PointVelocity`, `LocalTwist`, `RelativeTwist`
  in HomogeneousTransformation.h; `PyTwist` (exudyn.Twist: constructor, the two properties, ==, copies, repr) in
  PyHomogeneousTransformation.h, bound in pybindDataStructures.py; the three functions in exudyn.cUtilities
  (pybindCUtilities.py, PyCUtilities.h). `PyHT::Multiply` and `MatMul` return NotImplemented for an operand that is no
  HT, list, tuple or numpy array: `exu.HT(...) * X()` reaches `X.__rmul__`, `H*'x'` still raises a TypeError.
- Python: `kinematics.HT` (rotation, translation; `*`, `@`, `__rmul__`/`__rmatmul__` with an exudyn.HT as constant,
  `Inverse`, `Relative`), `kinematics.Twist`, and the three functions, which call exudyn.cUtilities when all arguments
  are exudyn.HT and exudyn.Twist. A constant operand is made symbolic when the other one is (`_MatMul`, `_Plus`), as
  `esym.Matrix @ numpy` is not bound.
- The cUtilities stub imports HT and Twist from exudyn when its annotations name them (createStubFiles.py).
- symbolicHTTest.py: 10 random frames and twists, every operation and function against exudyn.HT and cUtilities -
  the same bits (largest difference 0.0); RelativeTwist against central differences of HT0.Relative(HT1) moved by the
  twists (1.7e-10, 2.3e-11), esym.Jacobian of the relative translation by a rotation vector (5.2e-11).
- Checked: the suite passes, pytest 1455 passed.

<a id="rg12-69-4"></a>
### RG12.69.4 — the user connectors and joints on HT and twist (2026-11-01, #3183)
- definitions: `ObjectConnectorUser_forceUserFunction(mbs, t, itemNumber, HT0, twist0, HT1, twist1, dataCoordinates,
  parameters)`, `ObjectJointUser_constraintUserFunction` and `_jacobianUserFunction(mbs, t, itemNumber, HT0, HT1,
  parameters)`, their argument texts, descriptions and examples; the annotations `HT` and `Twist` (definitionTypes.py:
  classes for the definition files, `HomogeneousTransformation`/`Twist` in C++, passed by const reference;
  userFunctionModel.annotationToPython).
- C++: the connector builds a `HomogeneousTransformation` and a `Twist` per marker (without Orientation the rotation by
  SetTranslation, flagged as none); the joint the HT of its markers; `ToStdMatrix3D` removed. SymbolicUtilities.h:
  `processArgument` of an HT writes its two leaves, of a Twist its two; `PythonUserFunctionArgument` passes a PyHT and
  a PyTwist.
- The recording (ConvertFunctionToSymbolic): an argument of type HomogeneousTransformation becomes the leaves
  `<name>_rotation` (3x3) and `<name>_translation` in a kinematics.HT, a Twist the leaves `<name>_velocity` and
  `<name>_angularVelocity` in a kinematics.Twist; argList, argTypeList and argNameList hold the leaves, so that the
  C++ setup, the threads and GetFlatProgramData need no change.
- userItems: the spring-damper by `HT1.translation - HT0.translation` and the twist velocities, the torsional
  spring-damper by `HT0.Relative(HT1).rotation` and `RelativeTwist`, the joints by the axes of the rotations and the
  global difference of the positions (the multipliers stay global, as for the built-in joints); `_Functions` and
  `_Column` stay. SymbolicSystem: the marker states reordered into the leaves (`_ConnectorLeaves`, `_JointLeaves`).
- Test models: userConnectorTest, userJointTest (the screw by `HT0.Relative(HT1)`), userConnectorContactTest,
  symbolicExplicitSolverTest on the new arguments, all with their reference values; symbolicUserFunctionSignaturesTest
  generates the HT and Twist arguments and evaluates the recording with their leaves (53 signatures, as before);
  test_userFunctions.py knows the two C++ types.
- Measured, numeric path (a Python spring of two mass points, explicit Euler, one call per step): 13.4 instead of
  8.7 us per step with the build before; creating an exu.HT or exu.Twist costs about 0.4 us, reading a property 0.5 us
  (a copy into a new numpy array). Views in the properties would save a part; not done.
- Checked: the suite passes, pytest 1455 passed, the documentation builds; the export of a recorded connector names
  the leaves HT0_rotation, HT0_translation, twist0_velocity, twist0_angularVelocity.

<a id="rg12-69-7"></a>
### RG12.69.7 — the parts of exudyn.HT and exudyn.Twist as views (2026-11-01, #3184); the decisions on RG12.69 and RG12.70
*(Maintainer 2026-11-01: the twist keeps velocity/angularVelocity - yes; LookupTable - yes; symbolics.utilities - yes;
the extra 4.7 us per Python call - add views.)*
- The decisions written into symbolicHT.md and RG12.70; #3175 extended.
- `EPyUtils::NumpyViewOf` (PybindUtilities.h): the read-only view of NumpyView with the Python object as its base;
  `HomogeneousTransformation::GetRotationDataPointer`; `PyHT::RotationView`, `TranslationView`, `PyTwist::VectorView`,
  bound as the getters of the four properties.
- Measured as in RG12.69.4: 11.7 instead of 13.4 us per step (8.7 with the former arguments); a property 0.3 instead of
  0.5 us; what is left is creating the two HT and two Twist objects per call.
- A view follows a later change of its object (`H.translation = ...`) and refuses writing (`H.rotation[0,0] = 1`
  raises), so that a script cannot change an HT through a copy it believes it has. No use in the repository wrote into a
  part; suite and pytest pass.

<a id="rg12-69-5"></a>
### RG12.69.5 — the rotation and Lie group functions of kinematics.HT (2026-11-01, #3185)
*(Maintainer 2026-11-01: "do the next steps as suggested", RG12.69.5 first.)*
- Methods of kinematics.HT; for numeric parts they call exudyn.HT (the same bits), for symbolic parts:
  - GetRotationVector: s the skew part of A, angle = atan2(|s|, (trace - 1)/2), the vector angle/|s| s, the factor
    by 1 + |s|^2/6 below |s| = 1e-6 when cos > 0. Not the Euler-parameter route of LogSO3Vector (four branches of the
    largest component); the price: near pi the factor grows as 1/|s| - an error of about 1e-16/(pi - angle), and at pi
    the vector is not unique. Documented in the method and the manual.
  - GetRxyz: the formula of RotationMatrix2RotXYZTemplate with IfThenElse at |cos(rotY)| < 1e-14.
  - LogSE3: U = beta p - Omega x p / 2 + gamma Omega (Omega . p), the transposed TExpSO3Inv written out, its series
    below phi = 0.02; SetExpSE3: U - t1 Omega x U + t2 Omega x (Omega x U), the transposed TExpSO3, its series below 0.01;
    LogR3xSO3, SetExpR3xSO3: the translation and the rotation vector each on its own.
- symbolicHTTest.py: against exudyn.HT for 4 random directions at angles 1, 1e-3, 1e-9, 0 and 3: 1.1e-14; esym.Jacobian
  of GetRotationVector(RotationVector2RotationMatrix(v)) the unit matrix at v = [0.3, 0.1, -0.2] and at v = 0, where the
  limit is taken (2.2e-16).

<a id="rg12-69-6"></a>
### RG12.69.6 — exudyn.HT and exudyn.Twist of symbolic arguments (2026-11-01, #3186)
- A pybind class cannot return another type from its constructor; `python/exudyn/__init__.py` sets `HT.__new__` and
  `Twist.__new__`: if an argument has a symbolic part - an esym Real, Vector, Matrix, a list holding one, or a class of
  symbolics.kinematics - `kinematics.HT.FromSymbolicArguments` / `Twist.FromSymbolicArguments` (the rotation by
  EulerParameters2RotationMatrix, RotXYZ2RotationMatrix, RotationVector2RotationMatrix), else the old `__new__`. Python
  skips `__init__` for an object of another class. The C++ creates its HT and Twist by pybind's cast, which calls no
  `__new__`: the user functions do not pay it.
- `PyHT::MultiplyOther`: `exu.HT * esym.Vector` gives the point transformed by the symbolic HT of the exu.HT.
- Measured: exu.HT(translation=[1,2,3]) from Python 1.87 instead of 1.17 us (exu.HT() 0.70); the scan was written with
  loops, not any() of generators, which cost 2.8 us.
- symbolicHTTest.py: exu.HT of symbolic Rxyz, of a symbolic matrix and vector, exu.Twist of a symbolic component, the
  types and the values (0.0 against exudyn.HT); exu.HT and exu.Twist of numbers stay the classes of numbers.

<a id="rg12-55-5"></a>
### RG12.55.5 — the sphere-sphere contact of UserItemCreator (2026-11-01, #3187); #3188 raised (RG12.55.8)
- `_UFsphereSphereContactForce` on HT0, twist0, HT1, twist1: gap |p1 - p0| - r0 - r1, normal from 0 to 1, force
  -k g - d n.(v1 - v0) on sphere 1 while g < 0, by `IfThenElse` when recorded, a Python if for numbers; parameters
  [r0, r1, k, d], numbers or named Reals. `CreateSphereSphereContact` with every argument of mbs.CreateSphereSphereContact;
  refused: isHollowSphere1, friction, the exponent, plasticity, adhesion, restitution, impact models, a data node,
  activeConnector=False. Drawn as two spheres. A kind of its own in Record, which SymbolicSystem takes as a connector.
- The force at one state equals the built-in one to the bit (ball penetrating with a relative velocity, one explicit
  step of 1e-7, the built-in with a negative gap in its data).
- A ball under gravity on two fixed spheres, 0.8 s: RK44 converges for both; without damping user and built-in agree to
  4e-6 at 16000 steps, the recorded equals the Python one to 1e-14, SymbolicSystem by SolveExplicit equals SolveDynamic.
  With d = 20 the two converge to positions 2.5e-4 apart: the damping force -d dg/dt pulls the spheres together as they
  separate at small overlap, and a switch at the end of a step by the data node and a switch within the step by the
  gap give different motions there. With generalized-alpha the user contact does not converge with the step size
  (7e-4 between 4000 and 32000 steps with damping, 2.4e-5 without; the built-in: 2e-5), also with full Newton and a
  tolerance of 1e-12. Raised as #3188 (RG12.55.8); the test model compares by RK44 and documents it.

<a id="rg9-10-4"></a>
### RG9.10.4 — the one-pass Jacobians of the recorded user functions of ObjectGenericDAE (2026-11-01, #3189)
- `PySymbolicUserFunction::EvaluateJacobianAdd(jacobian, argument, factor, mbs, args...)` (SymbolicUtilities.h): the
  arguments set as for an evaluation, the program of the recording or of the copy of the thread evaluated and
  differentiated by the leaf of the argument (Differentiate of the table, #3171), factor times the derivative added; a
  constant result adds nothing, a result that is the leaf the unit matrix. The first use of the one-pass derivative by a
  C++ item.
- CObjectGenericDAE: `AddUserFunctionJacobian` (declared in itemDefsObjects.py, in evaluateUserFunctions.cpp); the
  Jacobian functions add -factor df_u/dq, -factor df_u/dq_t, -df_u/dy, +dg_u/dy, +dg_u/dq, +dg_u/dq_t; GetAvailableJacobians
  sets the analytic flags unless the user function is a Python one. The arguments q, q_t, y are 2, 3, 4 after mbs.
- genericDAESymbolicTest.py: two ODE2 and one ODE1 coordinate, a cubic spring, damping by q, couplings both ways; the
  Jacobian of ComputeJacobianODE2RHS and ComputeJacobianODE1RHS with factors 1, 0.5, 1: recorded against numerical
  3.1e-6 (largest entry 70); 500 steps generalized-alpha with full Newton: the solutions agree to 9.6e-13, 862 Newton
  steps each.
- Not in this step: massMatrixUserFunction, constraintUserFunction, jacobianUserFunction of the design - RG9.10.6.

<a id="rg9-10-5"></a>
### RG9.10.5 — SymbolicSystem takes ObjectGenericDAE (2026-11-01, #3190)
- `SymbolicSystem.daeKinds`: the ObjectGenericDAE of the same recordings and numbers of coordinates, per object its
  matrices by the symbols of genericDAE.md (empty if not given), its parameters and the global indices of its ODE1
  and algebraic coordinates; a Python user function raises, as for the other items.
- The ODE2 forces f + f_u - K q - D q_t - E y and the mass matrix go into ODE2RHS, MassMatrix, MassMatrixBlocks (after
  the bodies), ODE2ResidualLHS and its Jacobian; an object with ODE2 and ODE1 coordinates makes the ODE2 equations
  depend on y, so these functions then need y and the composed system has the input y after t. ODE1RHS adds A y + B q
  + B_t q_t + g + g_u, ODE1Jacobian (now with q, q_t) A + dg_u/dy; AlgebraicEquations C_q q + C_y y - c_0;
  ODE2ReactionForcesLHS C_q^T z; ODE1ReactionTermsLHS, which SymbolicSystem did not have, C_y^T z. SystemJacobian adds
  the coupling blocks (`_DAECouplings`) by the matrices and EvaluateJacobian of the recordings at the arguments q, q_t,
  y (3, 4, 5 with mbs). The velocity level is refused, as for the joints.
- Equations(): the same terms as expressions, y whenever the system has an ObjectGenericDAE with ODE1 coordinates, the
  new result ODE1ReactionTermsLHS; GetFlatProgram takes it and adds z and y to the arguments when used.
- symbolicSystemGenericDAETest.py, the coupled model of genericDAETest.py with recorded user functions plus an object on
  ODE1 coordinates only: SolveGeneralizedAlpha against SolveDynamic 4.3e-14 (relative), SystemJacobian against central
  differences 6.2e-12, composed against object by object 0.0, the numpy export against the functions 0.0.

<a id="rg9-10-6"></a>
### RG9.10.6 — the massMatrixUserFunction, constraintUserFunction and jacobianUserFunction of ObjectGenericDAE (2026-11-01, #3191)
- `massMatrixUserFunction(mbs, t, itemNumber, q, q_t, y, parameters)` instead of the massMatrix, dense or sparse; the
  object then has no constant mass matrix, and the massMatrix is not required.
- `constraintUserFunction(mbs, t, itemNumber, q, q_t, y, z, velocityLevel, parameters)` added to the algebraic
  equations; its Jacobian blocks AE_ODE2, AE_ODE2_t, AE_ODE1 and AE_AE by EvaluateJacobianAdd of the recording
  (arguments q 2, q_t 3, y 4, z 5), or by the jacobianUserFunction; a Python one without it is refused before the
  assembly, as the Jacobian of the algebraic equations is not numerical (#3101). The reaction terms follow from the
  Jacobian as for every body with algebraic equations.
- `jacobianUserFunction(mbs, t, itemNumber, q, q_t, y, z, factorODE2, factorODE2_t, factorODE1, parameters)`: the local
  Jacobian (n2 + n1 + nAE) of the terms of all user functions, -f_u in the ODE2 rows, g_u in the ODE1 rows, c_u in the
  algebraic rows; the object calls it with the factors of each block it needs (1, 0, 1 and 0, 1, 0 for the ODE1 and the
  algebraic rows), and it replaces the numerical Jacobian of a Python force or rhs user function.
- SymbolicSystem: the recorded mass matrix in the forces, the composed system and Equations(), d(M q_tt)/dq in the
  Jacobian; refused with ODE1 coordinates, as its MassMatrix does not take y. The recorded constraint as one
  esym.Function per object of c_u and its reaction terms (dc_u/dq)^T z and (dc_u/dy)^T z by esym.Jacobian, so that
  SystemJacobian is exact, also in the ODE2 and ODE1 blocks; in Equations() the reaction terms by the Jacobian of the
  variables q and y (TransposedJacobianTimes of the backend, symbolic and sympy). `_InitialAccelerations` of the
  solvers has the block dc/dz in the algebraic rows, as CSolverImplicitSecondOrder (default factorAE_AE 1): without it
  the solutions with a compliance differed by 6e-9.
- genericDAEUserFunctionsTest.py: a Cartesian pendulum with damper and a lag on an ODE1 coordinate; Python with the
  jacobianUserFunction and recorded, both against MassPoint2D, DistanceConstraint, CartesianSpringDamper and
  ObjectGenericODE1 1.4e-13, 600 Newton steps each. SymbolicSystem, the constraint of q, y and z (a compliance
  -1e-4 z): SolveGeneralizedAlpha against SolveDynamic 6.7e-13; the mass matrix function without ODE1: 1.5e-13;
  SystemJacobian against differences 1.8e-11; the numpy export 0.0.
- Found: a compliance +1e-4 z (a negative stiffness) and a term 0.01 x q_t at position level diverge in both solvers,
  in C++ and in Python at the same time; the test uses -1e-4 z and no q_t.

<a id="rg12-70-2"></a>
### RG12.70.2 — esym.LookupTable and exudyn.symbolics.utilities (2026-11-01, #3192)
- C++ (Symbolic.h): `LookupTableData` - values, step, start, outside zero, hold or periodic (the length
  len(values)*step, the last interval to the first value) -, `Value(x, derivative)`, the slope for the derivative;
  `ExpressionLookupTable`, one operand and a shared_ptr to the data, FlatName LookupTable or LookupTableDerivative;
  `SReal::LookupTable`; `PyLookupTable`, bound as esym.LookupTable with `__call__`, `Derivative`, values, step, start,
  outside, len. interpolation other than 'linear' is refused, as the options of the decision are not done.
- The table of operations (SymbolicTable): the codes LookupTable (rule RealLookupTable, the slope times the derivative of
  the operand) and LookupTableDerivative (rule Zero, piecewise constant), computed by `LookupTableOperation` from the
  data of the node, which the program keeps as the node of its step - the constant operand of the decision -;
  esym.Jacobian gives the derivative as LookupTableDerivative of the same data.
- Optimize: the key of a lookup includes its data, so that two tables at the same argument are not merged; a lookup at
  a number is folded. GetFlatProgramData writes the data as 'table' of the operation; codeExport writes it as
  esym.LookupTable(...)(x) (symbolic) and by a function _LookupTable in the program (numpy: the same bits; sympy: a
  Piecewise, to 2e-14).
- `exudyn.symbolics.utilities`: `SampleTime(t, period, offset=0., functions=exudyn.symbolic)` = offset +
  period*floor((t - offset)/period + 1e-12); `NoiseTable(step, length, seed, amplitude, distribution, outside)`, a
  LookupTable of round(length/step) values of numpy.random.default_rng(seed), uniform in [-amplitude, amplitude] or
  normal, cached per arguments in a dict, so that a Python user function does not create it in every call;
  `Noise(t, ...)` its value at t. The decided signature Noise(t, ...) kept, the table available by NoiseTable.
- symbolicLookupTableTest.py: the three modes against numpy.interp 3.6e-15; the derivative in one pass and by
  esym.Jacobian against the slopes 0.0; two tables at one argument after Optimize; the numpy export 0.0; a spring of
  ObjectGenericODE2 by its tabulated characteristic curve with a load of SampleTime and Noise, recorded against Python
  0.0; SampleTime at the instants of 1 ms, the noise table cached and bounded.

<a id="rg12-55-9"></a>
### RG12.55.9 — the Cartesian and rigid-body spring-dampers, the coordinate constraint and the torque of UserItemCreator (2026-11-01, #3193)
- `CreateCartesianSpringDamper`: ObjectConnectorUser on the position markers of mbs, f = k (p1 - p0 - offset) + d (v1 -
  v0) per global component, +f on marker 0; a node of a user mass point instead of a body marker, as the spring-damper.
- `CreateRigidBodySpringDamper`: ObjectConnectorUser on the rigid markers of GetMarkersPosRot, the joint rotations MR0,
  MR1 of mbs as 18 of the 96 parameters (K and D row by row, offset, MR0, MR1), a kind per formulation. Tait-Bryan: the
  frame of marker 0 times MR0, u the relative translation and GetRxyz of the relative rotation; intrinsic: u = LogSE3
  of the relative frame, the joint frame the rotation of H0 * SetExpSE3(u/2), the torque of the offset of the force
  (Aj (-u/2)) x f on both markers, as CObjectConnectorRigidBodySpringDamper. Written once with kinematics.HT for
  numbers and expressions; the user functions of the contact refused (springForceTorqueUserFunction,
  postNewtonStepUserFunction), markers in itemNumbers too.
- `CreateCoordinateConstraint`: ObjectGenericDAE on the nodes of the two coordinates with a NodeGenericAE,
  constraintMatrixODE2 [-1, factor1] at the coordinates, constraintVector offset (minus the reference values for
  includeReferenceCoordinates); the coordinate of a body counted over its nodes as mbs does. For it, the massMatrix of
  ObjectGenericDAE may be empty, a zero mass: the check before the assembly is removed, and SymbolicSystem keeps such
  an object out of MassMatrixBlocks and adds no mass entries (its kind key holds whether M is empty).
- `CreateTorque` as CreateForce; Record records the loadVectorUserFunction of each by its load type.
- symbolicUserConnectorsTest.py: two user rigid bodies with two rigid-body spring-dampers (full K with a coupling,
  offsets, turned joint frames, useGlobalFrame False), two Cartesian spring-dampers, a coordinate constraint to a mass
  point and a body-fixed torque of a user function: Python and recorded user items against the built-in ones 5.5e-13
  (intrinsic) and 2.4e-12 (Tait-Bryan); SymbolicSystem by SolveGeneralizedAlpha against SolveDynamic 5.7e-12; 100
  steps, 4.9 s.

<a id="rg12-66-1"></a>
### RG12.66.1 — the post-Newton function of ObjectConnectorUser on the frames and twists (2026-11-01, #3194)
- The signature of the forceUserFunction plus dataCoordinatesStartOfStep; the result the vector [discontinuousError,
  recommendedStepSize, new data coordinates], checked for its size 2 + n_d. Replaced without deprecation (maintainer:
  ObjectConnectorUser came after 1.13.6). `ComputeFramesAndTwists` builds the frames and twists for both functions.
  It needs two markers; one marker is refused with its message.
- Recorded: CreateSymbolicUserFunction of a switch by IfThenElse gives the same solution as the Python function (a
  mass point on a unilateral spring, 200 steps).
- userConnectorContactTest.py on the new signature: the result unchanged (26.270810078732666); the refusal of a force
  applied in the function is gone with the markers.

<a id="rg12-66-2"></a>
### RG12.66.2 — data nodes and a post-Newton function of ObjectGenericDAE (2026-11-01, #3195)
- `nodeNumbersData`, the fourth list of nodes, NodeGenericData; every user function gets x after y - after z for the
  constraint and the Jacobian functions - and the recorded ones keep the positions of q, q_t, y, z (2 to 5 after mbs).
  `ComputeDataCoordinates`, called where the user functions are evaluated. Without deprecation: ObjectGenericDAE came
  after 1.13.6.
- `postNewtonStepUserFunction(mbs, t, itemNumber, q, q_t, y, z, x, x0, parameters)` -> [error, recommendedStepSize,
  x]; it needs data nodes. The system had the post-Newton step for connectors only (listDiscontinuousIteration with
  their markers): `CObject::HasPostNewtonStepBody` and `PostNewtonStepBody`, the list `listPostNewtonBodies` of
  CSystemData, called serially in CSystem::PostNewtonStep after the connectors.
- Found: the PreStepUserFunction gets the time at the end of the step it precedes, with the state at its start; the
  reference of the test takes t - h as the sampling time.
- genericDAEDataTest.py: a mass on a spring with a PD controller sampled at 0.01 s and held, its force and sample
  time the two data coordinates; the post-Newton step samples at the end of a step that reaches the instant, with
  error 0, so the force acts from the next step. Python 0.0 and recorded 1.3e-14 against ObjectGenericODE2 with the
  controller in a PreStepUserFunction. SymbolicSystem still refuses data coordinates, until RG12.66.3.

<a id="rg12-66-3"></a>
### RG12.66.3 — SymbolicSystem with data coordinates and the post-Newton step (2026-11-01, #3196)
- SymbolicSystemBase: nData, data (the data coordinates, constant in a step, which every equation reads - a member
  instead of an argument of every function, as the solvers set it once per iteration), InitialStateData(),
  PostNewton(q, q_t, y, z, x, x0, t) -> (x, error, recommendedStepSize).
- SymbolicSystem: the data indices ltgData of each ObjectConnectorUser and ObjectGenericDAE (GetObjectLTGData), the
  data refused only if other items have some; the recorded postNewtonStepUserFunction a recording of the kind (in its
  key with the number of data coordinates); PostNewton evaluates them as CSystem::PostNewtonStep - the largest error,
  the smallest recommended step size >= 0 -, the connectors with the marker states at the data of the iteration.
  The data in the numerical arguments, the item functions of the connectors and the constraint functions of the DAE
  (a variable x, not differentiated), the composed system (an input x after t and y) and Equations() (a variable x,
  variables[5]); postNewton=True gives the results [error, recommendedStepSize, data] item after item, of x and x0.
- The solvers, as CSolverBase::DiscontinuousIteration: the start of the step kept (and aAlgorithmic), after the step
  the post-Newton step; above discontinuous.iterationTolerance the step again from its start with the new data, up to
  maxIterations, then an error unless ignoreMaxIterations; the recommended step size not used (constant step size).
  The data written into mbs with the state.
- symbolicSystemDataTest.py: a ball on a unilateral spring switched by its recorded post-Newton function with error,
  and a DAE with a held controller, error 0: generalized-alpha 7.7e-14 and RK44 3.3e-16 against SolveDynamic, 1004
  discontinuous iterations as Exudyn's solvers; the exported post-Newton functions against PostNewton 0.0.

<a id="rg12-55-8"></a>
### RG12.55.8 — the user sphere contact switched by a data node; the restitution model (2026-11-01, #3188, #3197)
- `CreateSphereSphereContact`: a NodeGenericData [gap, impact velocity] from dataInitialCoordinates[0] and [2] (the
  tangent velocity and the plastic deformation must be 0); the force while the gap of the data is negative, with
  impactModel 1 (Hunt-Crossley) or 2 (Gonthier, Carvalho-Martins) the term lambda delta dg/dt, lambda = impactFactor
  k / max(minimumImpactVelocity, impact velocity), impactFactor of the restitution coefficient as in
  CObjectContactSphereSphere; the post-Newton step as CObjectContactSphereSphere::PostNewtonStep - the gap into the
  data, the error |k g| and the recommended step size |g0/(dg/dt)| of a switch, the impact velocity when the step
  begins open and ends in contact, else that of the start of the step. Two kinds, impactModel 0 and 1/2. The
  conditions by `_If`, esym.IfThenElse while recorded and a Python if for numbers, as a symbolic number in a numpy
  product would not convert.
- The cause of #3188 confirmed: switched within the Newton iterations by the gap of the state, the user contact had
  another solution with damping and did not converge with generalized-alpha; switched by the data, it equals the
  built-in contact: generalized-alpha 2.2e-14 (impactModel 0 with d = 20) and 2.3e-14 (model 2, e = 0.7), RK44 2.2e-14
  and 6.3e-14; SymbolicSystem by SolveGeneralizedAlpha 1.7e-13, by SolveExplicit 0.0 against SolveDynamic
  (useRecommendedStepSize False, as the Python solvers keep the step size).
- Convergence with the step size, generalized-alpha, the position after 0.8 s between 2000, 4000, ... 32000 steps:
  model 0 1.2e-2, 5.7e-3, 1.4e-3, 2.2e-4; model 2 4.3e-3, 1.1e-4, 1.2e-4, 3.4e-5; with the recommended step size
  1.4e-3, 1.5e-3, 7.2e-4, 3.3e-5 (model 0). The in-step switch had stayed at 7e-4.
- symbolicUserContactTest.py rewritten on it: both models, both solvers, Python, recorded and SymbolicSystem; 5 s.

<a id="rg12-65-4"></a>
### RG12.65.4 — the program of a recording: measured; esym.Function without the export structure (2026-11-01, #3198)
- Decided 2026-10-29 (option A): what remained was at most the operation code stored in the node instead of its lookup
  by name in SymbolicProgram::Build. Measured first, an expression e = 0.999 e + sin(q[i]) of n terms:

  | operations | recording | esym.Function before | after | first evaluation | evaluation |
  |---|---|---|---|---|---|
  | 3000 | 3.3 ms | 7.4 ms | 1.3 ms | 0.07 ms | 0.017 ms |
  | 30000 | 33 ms | 57 ms | 16 ms | 0.33 ms | 0.17 ms |
  | 150000 | 157 ms | 351 ms | 128 ms | 2.2 ms | 1.1 ms |

  GetFlatProgramData of the same expression alone took 287 ms: esym.Function built its py::dict per node only to find
  the named variables. `NamedVariablesOf` finds them by a traversal of the nodes. The build of the program, the rest,
  is less than the recording and done once; the code in the node is not worth a change of every expression class.
- esym.Function is what SymbolicSystem builds per item and for the composed system, which is faster by the same.

<a id="rg12-65-7-1"></a>
### RG12.65.7.1 — SparseMatrix in Python on the scalar operations of the entries (2026-11-01, #3199)
- The proposal (symbolicSystem.md section 12.7) waits for a decision on a slot kind of the table. First measured
  whether it is needed: a product written as the scalar operations of the nonzero entries, each row summed in the
  order of the columns, against the dense esym.Matrix product of a band matrix whose entries depend on the variables:

  | rows | entries | table operations sparse / dense | evaluation sparse / dense | Jacobian (one pass) sparse / dense |
  |---|---|---|---|---|
  | 20 | 58 | 561 / 408 | 3.0 / 3.3 us | 15 / 21 us |
  | 100 | 298 | 2881 / 2088 | 12.7 / 39.3 us | 162 / 816 us |

  The values and the one-pass derivatives equal the dense ones to the bit (the dense sum adds exact zeros);
  esym.Jacobian builds other sums, 4.4e-16. The dense product is one operation in the count but n^2 multiplications.
- `SparseMatrix(rows, columns, triplets)` in exudyn.symbolics.utilities - numbers or esym.Real, equal entries added,
  the pattern fixed -, `FromMatrix` of rows or a numpy array (the entries that are not the number 0), `@` a vector,
  `T`, `s*A`, `A + B` on the union, `A[i, j]`, `ToDense`, `Triplets`, `shape`, `nnz`; for numbers numpy arrays.
- symbolicSparseMatrixProductTest.py: the band matrices, the operations on numbers, FromMatrix, the numpy export, and a chain
  of 10 springs of ObjectGenericODE2 with a stiffness of the coordinates as SparseMatrix, recorded against Python 0.0.
- Not done: the slot kind of the table (compressed rows in the memory of the program, one code per product) and the
  sparse results of recordings for the system matrices; recommended only when a model - FFRF reduced order, shells -
  needs more than the scalar operations give.

<a id="rg12-70"></a>
### RG12.70 — the decisions of the step, moved from the plan when it closed (2026-11-01, #3175)
The text of the step as it stood in the plan; the step is implemented by RG12.70.2 (#3192), and the held sensor of
RG12.70.1 is the idea #3180.

#3175 (maintainer 2026-10-31: with ODE1, control, thermal and electro-mechanical problems can be reproduced, time-discrete problems not fully; a function giving the discrete time from the time; data signals at a sampling time with fast access to the interpolated value - closest, linear, quadratic - and numerical differentiation; a time-dependent randomizer, repeatable for each time; make a suitable suggestion. Revised 2026-10-31: only the signal is needed in exudyn.symbolic; the discrete time a Python function of a new module of exudyn.symbolics, as there will be more that fits neither kinematics, userItems nor systemModel; a hold needs data coordinates, so it is done by hand now and later by a sensor (RG12.70.1); the noise made with the signal from a random signal, at the cost of some memory; the signal repeated after a period as an option, zero outside by default; the three features proposed because they are structurally different and needed at some point, so better included early - but the definition chosen with care, what is required separated from what is an option, nothing that blocks CUDA, some features left out for now; "Signal" is not the name, as it may also interpolate a bending moment over the curvature: find a name, for an argument that is the time, a system coordinate or an item parameter).

Decided 2026-11-01 (maintainer: the name LookupTable, the module exudyn.symbolics.utilities):

- **`esym.LookupTable(values, step, start=0., outside='zero', interpolation='linear')`**, an object holding the data, and
  `table(x)` its value at the argument x - a time, a coordinate or a parameter, any `esym.Real`. The table and its call
  are two things: the data is stored once, copied once to the GPU, and called at several arguments (the derivative of
  a characteristic curve, two channels at the same time). The name says what it is, for time as for a curve; numpy
  calls the operation `interp`, Simulink and Modelica a lookup table (`1-D Lookup Table`, `CombiTable1D`).
- **Required now:** one channel; equal steps, the index `floor((x - start)/step)` in O(1); linear interpolation; outside
  the table `'zero'` (default), `'hold'` (the end values) or `'periodic'` (repeated with the length of the table); the
  derivative is that of the interpolant, piecewise constant for linear, which the one-pass derivative and the Jacobian
  give without a rule of their own. One code of the table of operations with a constant array as operand - a kind the
  table of RG12.65 gets for this -, a fixed kernel for RG14.
- **Options, kept possible by the interface and done when needed:** several channels (`values` a matrix, the result a
  vector); given arguments of unequal steps (`arguments=[...]`, bisection in O(log n), divergent on the GPU, therefore
  not the default); `'closest'` (piecewise constant, derivative 0) and `'quadratic'` (through the three nearest values,
  a continuous derivative).
- **In a new module of exudyn.symbolics** for what is neither kinematics, user items nor system models - proposed
  `exudyn.symbolics.utilities` (decided):
  - `SampleTime(t, period, offset=0., functions=exu.symbolic)` = offset + period*floor((t - offset)/period), with a
    tolerance of about 1e-12 period against rounding at the sampling instants; a function of time held between the
    samples is f(SampleTime(t)). Holding a state - read at the instants, kept in between - needs data coordinates:
    by hand now, by the sensor of RG12.70.1 later.
  - `Noise(t, step, length, seed=0, amplitude=1., distribution='uniform', outside='periodic')`: a `LookupTable` of
    `length/step` random values of `numpy.random.default_rng(seed)` - the same for the same seed in every run, step size,
    Newton iteration and order of evaluation; normal distribution by `distribution='normal'`; the memory of the table is
    the cost.

<a id="rg9-11-1"></a>
### RG9.11.1 — Design the revision of the Jacobians (2026-11-01, #3201, #3202)
- Maintainer 2026-11-01: the Jacobian computation of CSystem has many switches, because some objects are numerical
  and some analytic; every object should provide a Jacobian, analytically or locally by automatic or numerical
  differentiation, the markers having the interfaces for it; the assembly then distinguishes only the system-wide
  numerical Jacobian (all couplings, serial, for comparison) and the local Jacobians, in parallel for the objects
  without Python functions; connectors whose user functions read further coordinates (sensors) stay out of the fast
  path. Only ODE2-ODE2, AE-AE and ODE2-AE for speed; the other blocks rare, serial, readable, complete; in line with
  the ODE1-ODE2 couplings of #3143. CSystem.cpp split into CSystemAssemble.cpp, CSystem.cpp and CSystemJacobians.cpp.
- Existing issues checked: #1203 and #1202 (parallel ODE2 and AE Jacobians) and #858 (cost per item) were RG5.3 and
  are now RG9.11.5; #1548 (ODE1 loads, RG4.23) is part of RG9.11.7; #988 (sparse constraint Jacobian derivative),
  #3107 (central differences of the user items) and #2884 (the joints on the relative transformation) are related and
  stay where they are. None covered the revision as a whole: #3201 raised, and #3202 for the split.
- Found in the code: four functions decide per object between five ways (analytic, connector interface, marker data,
  numerical by CSystem with a Lie group variant, system-wide); the ODE1 coupling of the ODE2 rows in two code paths;
  no parallel Jacobian, `multithreadedLowerLimitJacobians` unused; of the 34 connectors and constraints, 12 connectors
  are on the ODE2 connector interface and 9 joints on the AE one. The solver chapter still says that the ODE1-ODE2
  blocks vanish.
- docs/dev/design/systemJacobians.md: the blocks and their current computation, the problems, one virtual
  `ComputeLocalJacobian` with a request and a numerical default in CObject (serial, as it perturbs the global
  state), the parallel and the serial list, the fast and the serial blocks, a verification test against the
  system-wide numerical Jacobian, the split; sub-steps RG9.11.2 to RG9.11.7 and the decisions J1 to J4.

<a id="rg12-55-11"></a>
### RG12.55.11 — One kind per Create function of UserItemCreator; IfThenElse of the utilities (2026-11-01, #3203)
- Maintainer 2026-11-01: why separate paths for rigidBodySpringDamper/Intrinsic and sphereSphereContact/Restitution -
  one user function, the switch done by the "fixed" settings at recording; and the idea of an IfThenElse that records
  only the active branch when its condition is a number, to be checked before an issue.
- Checked: the parameters of a recording are always symbolic (one recording per kind, read at each call), so a
  switch on a parameter records both branches, and the program computes both (the code RealIfThenElse picks one of two
  computed slots; Optimize folds only a constant condition). A condition is a number at recording only when it is a
  Python constant - a setting of the Create function bound in the function. So the type of the condition is the right
  criterion, and it is what the private `_If(condition, a, b, functions)` of the user items did by the module of the
  functions: `exudyn.symbolics.utilities.IfThenElse(condition, ifTrue, ifFalse)` replaces it.
- The contact as one function with the restitution term always - exact for impactModel 0, as its impact factor is 0 -
  costs 42 instead of 30 operations of the table for impactModel 0. Therefore both variants are fixed at recording:
  `_SphereSphereContactFunctions(restitution)` and `_RigidBodySpringDamperFunctions(intrinsic)` make the functions of
  a variant (one implementation, the variant a Python constant of a closure), `UserItemCreator._variants` keeps the
  functions of each item of these kinds, and Record records once per kind and variant, grouped by the identity of the
  functions. `creator.items` has the kinds `sphereSphereContact` and `rigidBodySpringDamper` only.
- symbolicUserContactTest.py and symbolicUserConnectorsTest.py give the same results to the bit; the latter also has
  one spring-damper of each formulation in one system (6.340335689918898, two checks more). symbolicLookupTableTest.py
  checks IfThenElse: numbers, a symbolic condition decided at evaluation, fewer operations for a constant condition
  (11.007450360798327).

<a id="rg12-67-1"></a>
### RG12.67.1 — Design the symbolic parameters of every item (2026-11-01, #3159)
- Maintainer 2026-11-01: continue with RG12.67, whose plan text asks for a design first.
- What #3152 covers: named variables for the scalar arguments of three Create functions, kept per object beside the
  numbers of its `parameters` vector, used by `Equations()`; the composed system takes the numbers and is built again
  by `SetParameterValues`. Not covered: the inertia of the rigid body (computed by the numeric `RigidBodyInertia`),
  the marker positions and frames of joints and connectors, load vectors, the matrices of ObjectGenericODE2 and
  ObjectGenericDAE, expressions of parameters.
- docs/dev/design/symbolicParameters.md: one parameter set of named esym.Real per system; bindings of slots (item
  type, number, field, component) to expressions of the parameters, the items keeping the numbers and checked as in
  #3152; structure stays a Python constant (#3203); where the bindings come from (UserItemCreator, by hand, symbolic
  inertia functions); Equations() with every bound slot; the parameters as an input of the composed system, so that
  new values need no rebuild and dr/dp comes with dr/dq; forward sensitivities of the solution in the Python solvers.
  Sub-steps RG12.67.2 to RG12.67.5 and the decisions P1 to P5.

<a id="rg12-60-1"></a>
### RG12.60.1 — The composition in Python measured on chains of rigid bodies (2026-11-01, #3117)
- Maintainer 2026-10-28: keep RG12.60 open - first check whether the Python code performs well enough for real-life
  problems, and whether other cases need the C++. 2026-11-01: continue with RG12.60.
- The Python path: `SymbolicSystem` and `Equations()` call each recording with symbolic arguments through its code of
  exudyn.symbolics.codeExport (`_Caller`: `ToFunction(recording.GetFlatProgramData(), 'symbolic')`, written once per
  recording, then one call per item); a C++ copy of the nodes with the arguments substituted would replace these calls.
- A chain of n rigid bodies of UserItemCreator (NodeRigidBodyRxyz), linked by a revolute joint or by three
  spring-dampers and a torsional spring-damper each, recorded; the time in the generated functions (`_F`) by cProfile,
  the wall times without it:

  | system | coordinates | SymbolicSystem | of it (profiled): calls of the recordings / Optimize / esym.Function | Equations() | SolveGeneralizedAlpha, 100 steps | SolveDynamic, 100 steps |
  |---|---|---|---|---|---|---|
  | 10 bodies, joints | 60 | 0.015 s | 0.001 s / - / - | 0.011 s | 0.62 s | 0.08 s |
  | 50 bodies, springs | 300 | 0.21 s | 0.013 s / - / - | 0.23 s | 3.9 s | 0.50 s |
  | 200 bodies, joints | 1200 | 0.26 s | 0.017 / 0.089 / 0.015 s of 0.165 s | 1.46 s | 59.5 s | 15.7 s |
  | 200 bodies, springs | 1200 | 0.86 s | 0.053 / 0.320 / 0.046 s of 0.618 s | 2.06 s | 15.5 s | 3.6 s |

- The substitution in Python is about 10 % of building the system, Optimize in C++ about half; the build is below 2 %
  of 100 steps of the Python solver and below a third of 100 steps of SolveDynamic. No case was found that needs the
  C++: every recording has its flat program, which codeExport writes, LookupTable included (#3192); a recording
  called inside another one being recorded works the same way.
- Proposed: close #3117 - the C++ copy is not needed -, and raise it again if a model shows otherwise. Seen on the way:
  `Equations()` with the default `massMatrix=True` builds the dense global mass matrix as esym.Matrix, 1.4 million
  entries at 1200 coordinates, which is most of its 2 s; `massMatrixBlocks` is the form for large systems.

<a id="rg9-11-decisions"></a>
### RG9.11 — the decisions on the design of the Jacobians (2026-11-01, #3201)
- Maintainer 2026-11-01: J1 yes, but for connectors and joints automatic differentiation should always be possible,
  so their parallel computation is no issue, and a local numerical Jacobian differentiates the marker quantities, not
  the system coordinates; the numerical backup is only for the CObjects that are not connectors. J2 yes. J3 as
  recommended; sensor couplings would need the dependencies (node, object -> node) -> marker -> (load, sensor) and
  those of the user functions (straightforward for symbolic ones) - a possible step of low priority, interesting as it
  keeps the Newton convergence. J4: forAE, forODE2 and forODE2Connectors deprecated, doing nothing; only the system-wide
  differentiation stays; jacobianConnectorDerivative removed too if it costs more code than it saves time. The ODE1
  blocks may go into the parallel line if that unifies the code. Continue with #3202, the comparison of the Jacobians
  (not broad - the test suite is the real test) and the first implementations of RG9.11.4.
- Checked for J1: all joints have analytic constraint Jacobians. Numerical by CSystem are ObjectConnectorUser, the
  eight contacts, ReevingSystemSprings, RollingDiscPenalty (an interface Jacobian exists, not declared),
  HydraulicActuatorSimple, CoordinateSpringDamperExt with stops or friction, the intrinsic RigidBodySpringDamper and
  every spring-damper with a Python user function: sub-step RG9.11.8. Design section 7; sub-steps RG9.11.9 (J4) and
  RG9.11.10 (J3) added.

<a id="rg9-11-2"></a>
### RG9.11.2 — CSystem.cpp split into three files (2026-11-01, #3202)
- A script split src/Main/CSystem.cpp (4113 lines) at its top-level definitions, each with the comment lines before
  it, by name: CSystemAssemble.cpp gets Assemble, CheckSystemIntegrity, AssembleCoordinates, AssembleLTGLists,
  AssembleObjectLTGLists, the three static helpers of the user functions in threads, UpdateUserFunctionsForThreads,
  PreComputeItemLists, the Lie group lists and AssembleInitializeSystemCoordinates/AssembleSystemInitialize;
  CSystemJacobians.cpp gets JacobianODE2Loads, ComputeObjectJacobianODE2, JacobianODE2RHS, JacobianODE1RHS,
  NumericalJacobianAE, ComputeObjectJacobianAE, JacobianAE and the four constraint Jacobian derivative and times
  vector functions; the rest stays in CSystem.cpp, in its order. The sorted lines of the three files equal those of
  the old file plus the two copied headers.
- Changed: `ComputeObjectODE1RHS` is no longer `inline` (JacobianODE1RHS in the other file calls it);
  CSystemJacobians.cpp declares `extern Index TScomputeGeneralContact`, the timer registered in CSystem.cpp. The two
  files are in msvc/cppsrc.vcxproj and its filters, and in both lists of sources.json (gen_sources.py --check OK).

<a id="rg9-11-3"></a>
### RG9.11.3 — The local Jacobians against the system-wide numerical one (2026-11-01, #3201)
- python/testing/test_systemJacobians.py, a pytest as it compares code paths: models RigidBodies (two bodies on
  Tait-Bryan angles, spring-damper, Cartesian, both rigid-body spring-dampers, torsional), PointsAndContact (mass
  points, ContactSphereSphere in contact, ConnectorCoordinateSpringDamper, spring-damper), Cable (four
  ObjectANCFCable2D with mass-proportional loads, a Cartesian spring-damper on a node), GenericDAE (ODE2 and ODE1
  coupled by matrices) and Joints (revolute, spherical, prismatic, generic, distance, coordinate constraint).
- At the initial coordinates turned by 0.01 cos(i) and zero velocities, with the dense solver: JacobianODE2RHS with the
  factors (1, 0, 0), (0, 1, 0), (0, 0, 1) - the loads included (computeLoadsJacobian=2) - against
  doSystemWideDifferentiation; JacobianODE1RHS of the DAE against the same; JacobianAE of the joints against forAE.
  All 19 agree within 1e-4 of the largest entry; the largest difference, 1.06e-5, is the velocity derivative of the
  cable's stiff spring, whose numerical step is that of the coordinates.

<a id="rg9-11-4-1"></a>
### RG9.11.4.1 — One function for the local Jacobian of the ODE2 rows (2026-11-01, #3201)
- JacobianODE2RHS decided per object in its loop between the object's own Jacobian and a numerical one, and computed
  the ODE1 block of the ODE2 rows in both branches, once after the analytic block and once after the numerical one.
- Now `CSystem::ComputeLocalJacobianODE2(temp, numDiff, j, factorODE2, factorODE2_t, factorODE1, local)` computes the
  blocks of object j into the temporary data - by q and q_t the object's own (ComputeObjectJacobianODE2, in
  temp.jacobianODE2Container) or numerical (Lie group nodes apart, in temp.localJacobian); by y the object's own
  (ComputeJacobianODE2_ODE1) or numerical, one lambda for both cases, in the new temp.localJacobianODE1 (tempMatrix
  cannot hold it, the markers use it while the LHS is evaluated) - and `LocalJacobianODE2` (CSystem.h) says which
  blocks it computed; `AddLocalJacobianODE2` adds them to the system Jacobian in the order of before. The loop of
  JacobianODE2RHS calls the two; the system-wide branch, the loads and the general contact are unchanged.
- The test suite passes, pytest 1485 passed (test_systemJacobians.py included). Computing and adding are separate,
  which is the unit of the parallel assembly (RG9.11.5): compute per thread, add serially.

<a id="rg9-11-4-2"></a>
### RG9.11.4.2 — The ODE1 rows on the pattern of the ODE2 rows (2026-11-01, #3201)
- `ComputeLocalJacobianODE1` computes the blocks of the ODE1 rows of an object - by y and the coupling by q and q_t,
  the object's own (ComputeJacobianODE1RHS, not for connectors) or numerical - and `AddLocalJacobianODE1` adds them in
  the order of before; JacobianODE1RHS is a loop over the two. The numerical coupling block has its own matrix
  `temp.localJacobianODE2`, as `localJacobian` still holds the ODE1 block when it is computed.
- `LocalJacobian` (CSystem.h, was `LocalJacobianODE2`) describes the blocks of both kinds of rows and where they are,
  and has `deferred` for the parallel assembly: `ComputeLocalJacobianODE2(..., analyticOnly=true)` returns false with
  `deferred` before it would differentiate numerically. `temp.deferredObjects` for the objects a thread leaves to the
  serial part.
- The test suite passes; test_systemJacobians.py and test_connectorInterface.py 67 passed.

<a id="rg9-11-4-3"></a>
### RG9.11.4.3 — The virtual ComputeLocalJacobianODE2 of CObject (2026-11-01, #3201)
- `CObject::ComputeLocalJacobianODE2(systemData, temp, numDiff, factorODE2, factorODE2_t, ltgODE2)`: the object's own
  local Jacobian of its ODE2 rows into temp.jacobianODE2Container, false if it has none for these settings. The default
  (CObjectBody.cpp, for every object that is not a connector) calls ComputeJacobianODE2_ODE2;
  `CObjectConnector::ComputeLocalJacobianODE2` (CObjectConnector.cpp) is the branch that was in CSystem, moved
  unchanged: forODE2Connectors, the Jacobian derivatives of the markers, the connector interface, else the marker
  data. CSystem::ComputeObjectJacobianODE2 checks only the settings and the declared Jacobian functions and calls it.
- Different from decision J2 as written (the numerical default in CObject): the numerical differentiation stays a
  function of CSystem. It perturbs the system coordinates and needs the Lie group composition of CSystem
  (ComputeLieGroupNodeCompositionEps); in CObject it would need the same access. It is what an object returning
  false gets, and it is serial (RG9.11.5). Moving it is possible later through a small interface of CSystem, if wanted.
- The test suite passes; test_systemJacobians.py, test_connectorInterface.py, test_accessFunctionsAD.py 73 passed.

<a id="rg9-11-5-1"></a>
### RG9.11.5.1 — The ODE2 rows of the Jacobian in parallel (2026-11-01, #1203, #3201)
- JacobianODE2RHS: with more than one thread and at least `parallel.multithreadedLowerLimitJacobians` objects without
  Python functions (listComputeObjectODE2LhsNoUF, the list of the parallel residual), a ParallelFor computes each
  object's local Jacobian into the TemporaryComputationData of its thread with `analyticOnly=true` and appends it as
  triplets of the system Jacobian (`AppendLocalJacobianODE2`: the nonzero entries of the dense blocks, or the triplets
  of the sparse container); an object that needs a numerical block - which perturbs the system state - returns
  `deferred` and goes to the thread's `deferredObjects`. Then serially: the triplets of all threads into the system
  matrix, the deferred objects and the objects with Python functions by ComputeLocalJacobianODE2/AddLocalJacobianODE2.
  With one thread or below the limit the serial loop of before runs, so the results of the serial runs are unchanged.
- Measured (Windows, EigenSparse, ComputeJacobianODE2RHS): an ANCF cable (ObjectANCFCable, analytic) of n elements with
  a Cartesian spring-damper (connector interface) over every second node:

  | n | 1 thread | 2 | 4 | 8 |
  |---|---|---|---|---|
  | 200 | 2.07 ms | 1.05 ms | 0.53 ms | 0.50 ms |
  | 2000 | 20.9 ms | 10.9 ms | 9.1 ms | 4.6 ms |

  The difference to the serial Jacobian is the rounding of another order of the sums, 3e-12 of the largest entry.
- test_systemJacobians.py compares the parallel with the serial assembly for the five models, dense and sparse, to
  1e-14 (10 cases more, 29 in all); the test suite passes, pytest 1495 passed.
- Open (RG9.11.5.2): the AE rows (#1202) and the split of the tasks by a cost per item (#858).

<a id="rg9-11-8"></a>
### RG9.11.8 — The local numerical Jacobian by the marker kinematics, tried and rejected (2026-11-01, #3201)
- Implemented on the local branch `rg9-11-8-kinematicsJacobian` (one commit, not merged): `ItemConnectorInterface(...,
  jacobian='numerical')` generates ComputeJacobianODE2Connector with `ConnectorJacobianODE2RigidMarkersNumerical` or
  `...CoordinateMarkersNumerical` - the connector's force of L1 differentiated by forward differences of the marker
  kinematics (position, global rotation increment by ExpSO3, velocity, global angular velocity; markers without
  orientation by position and velocity), the derivatives as DValues of the AutoDiff forces, chained by the code of
  the automatic Jacobian (moved into `ChainConnectorJacobianRigid`). It does not touch the system state, so a thread
  could compute it. Declared for RollingDiscPenalty, ContactConvexRoll, ContactCoordinate, ContactSphereSphere,
  ContactSphereTorus and ContactSphereTriangle.
- test_systemJacobians.py and test_connectorInterface.py passed (77), but 13 test models and one mini example changed,
  and the Newton convergence degraded where velocities depend on rotating frames and friction acts:

  | test model | Newton iterations before / after | Jacobians before / after |
  |---|---|---|
  | rotatingTableTest.py | 26785 / 1083889 | 71 / 678647 |
  | ballBearingTest.py | 3057 / 3669 | 531 / 907 |
  | createSphereQuadContact.py | 50384 / 50488 | 834 / 882 |
  | createSphereQuadContact2.py, createSphereTriangleContact.py, sphereTriangleTest2.py | equal within 6 | equal |

- The cause: the chain J^T K J with K by the marker kinematics lacks the derivative of the marker velocities by the
  coordinates (v = J(q) q_t, omega of a rotating frame), which the numerical differentiation by the system coordinates
  contains; the automatic Jacobians of the connector interface neglect it as well (test_connectorInterface.py compares
  them at zero velocities), which is harmless for spring-dampers and not for friction.
- Reverted on v2-dev. Options for the decision: (a) the markers provide dv/dq (the derivative of J q_t), so that the
  local Jacobians - automatic and numerical - are complete; (b) the contacts and the rolling disc stay numerical by
  CSystem, serially, and only connectors without velocity-dependent friction move.
