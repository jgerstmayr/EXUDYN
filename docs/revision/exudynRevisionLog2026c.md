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

<a id="rg9-11-8-1"></a>
### RG9.11.8.1 — dv/dq evaluated: the markers, the connectors, the effect on the test suite (2026-11-01, #3201, #3204)
- Maintainer 2026-11-01 on RG9.11.8: "I would prefer (a) in the long term; if I understand correctly, dv/dq is only
  computed when requested. For ConnectorSpringDamper or for normal (frictionless) contact, it wouldn't be needed.
  [...] I think this has to be decided by the connector, which marker jacobians are requested." - "prior to
  RG9.11.8, there should be an evaluation step [...] For the convex roll, a numerical jacobian in serial mode would
  not hurt a lot. [...] I believe this issue explains some convergence issues in the past."
- The evaluation is design section 8. How it was measured: a scratch runner executed every model of
  runTestSuiteRefSol.TestExamplesReferenceSolution in its own interpreter (8 at a time, the UI suppressed), wrapped
  exudyn.solver.SolveDynamic/SolveStatic (also as MainSystem methods) to sum `it.newtonStepsCount` and
  `it.newtonJacobiCount` of every solve, and set one flag of `numericalDifferentiation` in the settings passed in:
  as is, `forODE2Connectors=True`, `jacobianConnectorDerivative=False`; a fourth run took the timers of the solver
  (`displayComputationTime`). The runs are deterministic (two runs each of three models, equal counts).
- forODE2Connectors=True against as is, the 13 of 136 implicit models with connectors whose counts differ (Newton
  iterations / Jacobians, as is -> numerical; the relative change of the test result):

  | model | Newton | Jacobians | result |
  |---|---|---|---|
  | coordinateSpringDamperExt.py | 4780 -> 1409 | 4780 -> 1409 | 2e-2 |
  | mainSystemUserFunctionsTest.py | 8000 -> 9727 | 4 -> 4 | 1e-7 |
  | rotatingTableTest.py | 26785 -> 26377 | 71 -> 73 | 8e-10 |
  | userConnectorTest.py | 6961 -> 7161 | 45 -> 45 | 8e-15 |
  | symbolicUserConnectorsTest.py | 2309 -> 2187 | 2309 -> 2187 | 2e-15 |
  | mecanumWheelRollingDiscTest.py | 22601 -> 22617 | 39 -> 39 | 3e-11 |
  | userNodeTest.py | 5045 -> 5050 | 24 -> 24 | 8e-14 |
  | ANCFgeneralContactCircle.py, ANCFbeltDrive.py, symbolicUserRigidBodiesTest.py, springDamperVelocityOffsetTest.py, markerNodeCoordinateReferenceTest.py | within 1 | within 1 | <5e-11 |
  | solverFailureMessageTest.py | 19 -> 5 | | fails by design |

- coordinateSpringDamperExt.py: the Jacobians at the initial state differ by 1.3e5 in the rows of the gear pair
  (RotationalMass1D, ConnectorCoordinateSpringDamperExt with factor1 = -rad0/rad1, no friction): its analytic
  Jacobian is stiffness*factorODE2 + damping*factorODE2_t chained with the marker Jacobians, without factor0 and
  factor1, while ComputeConnectorForcesCoordinate has them. Raised as #3204 (BUG), to RG9.11.8.3.
- jacobianConnectorDerivative=False, 20 models differ: objectFFRFTest 54 -> 69, objectFFRFreducedOrderTest 799 -> 840,
  objectFFRFTest2 183 -> 190, objectFFRFreducedOrderAccelerations 74 -> 80, symbolicUserConnectorsTest 2309 -> 2372,
  userMarkerTest 11378 -> 10878, userNodeTest 5045 -> 4974, computeItemTest 265 -> 268, the others equal or within 4.
- The share of totalJacobian in the timer total of the contact models: ConvexContactTest 69 %,
  contactSphereTorusMomentumTest 68 %, rollingCoinPenaltyTest 68 %, contactComparisonTest 50 %, ballBearingTest 42 %,
  symbolicUserContactTest 41 %, carRollingDiscTest 24 %, restartFileTest 19 %, createSphereTriangleContact 19 %,
  createSphereQuadContact 17 %, sphereTriangleTest2 16 %, createSphereQuadContact2 11 %, rotatingTableTest 1 %,
  mecanumWheelRollingDiscTest 0 %.
- The markers, from src/ImplMarkers and the access functions of the bodies: every contact and rolling disc of the test
  suite uses MarkerBodyRigid on a RigidBody (CreateRigidBody, Euler parameters) or MarkerNodeRigid on an Euler
  parameter node. RigidBody with Euler parameters or Tait-Bryan angles, RigidBody2D, ANCFCable2D, ANCFBeam,
  ANCFThinPlate and the slope nodes form their Jacobian derivative by the nested AD of AccessFunctionsAD
  (`JacobianTransposedTimesVectorDerivative`), whose inner derivatives are the second derivatives dv/dq needs.

<a id="rg9-11-9"></a>
### RG9.11.9 — forAE, forODE2 and forODE2Connectors deprecated (2026-11-01, #3205)
- Maintainer 2026-11-01: "You could also do RG9.11.9 now, which then simplifies some paths in the further
  implementation and the full numerical fallback is still there." Design J4: deprecated, no effect.
- definitions/structureDefsSimulationSettings.py: NumericalDifferentiationSettings has `dummyBool` (SFNoPybind), and
  forAE, forODE2, forODE2Connectors are SFDeprecated members redirected to it (Deprecated('1.13.304', 2031)); the old
  rename forODE2connectors as well. The warning says "is deprecated and has no effect!" - structureHeaderEmitter.py
  recognized only openGL.dummy/dummyBool, now any `dummy`/`dummyBool`; structureModel.py writes "has no effect" in the
  list of changes of the structure page, deprecationsEmitter.py "(no effect)" (also for the five visualization
  settings with a dummy), checkUserScripts.py removes the assignment (it recognized `.dummy` only, not `dummyBool`).
- C++: ComputeObjectJacobianODE2 without forODE2, CObjectConnector::ComputeLocalJacobianODE2 without
  forODE2Connectors; JacobianAE differentiates numerically under doSystemWideDifferentiation. NumericalJacobianAE wrote
  into the dense matrix and refused a sparse one; it collects the nonzero entries as triplets and adds them by
  AddSparseTriplets, the same values for the dense matrix (the AE rows and columns are zero before), and works for
  EigenSparse.
- Models and tests: the `forODE2 = False` lines of four examples, a performance model and four general contact test
  models removed (the default), forAE=True of ANCFmovingRigidbody.py removed; geometricallyExactBeamJacobianTest.py,
  test_accessFunctionsAD.py, test_connectorInterface.py, test_jacobianAEvelocityLevel.py and test_systemJacobians.py
  compare with doSystemWideDifferentiation (test_systemJacobians: the constraint Jacobian dense and sparse);
  simulationSettingsRenamesTest.py without forODE2connectors (49 renames), test_checkDeprecations.py leaves out the
  redirects without effect; parameterConversionTestReference.txt rerecorded (the three names gone from the lists of
  numericalDifferentiation); tools/ci/stubtestBaseline.txt lists the three deprecated properties.
- Two test models set forODE2Connectors=True "to be compatible with old solutions" and now use the analytic connector
  Jacobians: scissorPrismaticRevolute2D.py 27.202556489044277 -> 26.722556541316987 (its result adds 1e-2 times the
  Newton iterations: 48 fewer), SpringDamperMesh.py its static reference -0.44056224799446486 -> -0.44056224908289804
  (Newton tolerance 1e-6). The AVX2 reference of scissorPrismaticRevolute2D (AVX2ReferenceSolutionUpdate) is stale
  until the fast-module logs are rerecorded.
- jacobianConnectorDerivative measured in RG9.11.8.1: it stays.
- Test suite passed, pytest passed, `exudev generate --all-checks` and `exudev docs` ok.

<a id="rg12-67-1-revised"></a>
### RG12.67 — the design of the symbolic parameters revised (2026-11-01, #3159)
- Maintainer 2026-11-01 on symbolicParameters.md: "what you propose: the binding of item parameters to expressions;
  this seems to be systematic (not human readable, but ok)." - "Can I define a parameter as esym.Real('m1', 13.5) and
  write it into the user item parameter vector, so the user function recording would recognize that automatically?
  that a parameter binding / slot is added? This would be a suitable way." - "the 'parameters' vector in the user
  items: would it help to store it as vector of esym.Real [...] it would be readable from the items?" - "'RigidBodyInertia'
  could be adapted to also take symbolic values (working with both types)."
- Probed: `float()` of an `esym.Real` raises (the recording guard), so an `esym.Real` in an item dict fails today and
  no model depends on a conversion; `InertiaCuboid(rho, [a, b, c])` with esym.Real fails on the shape check of an
  esym.Matrix, `InertiaCylinder` and `InertiaSphere` on an `if` of a symbolic condition.
- Design revised: section 2.3 (bindings written into the items at Add/Modify/Set, stored in mbs.parameterBindings,
  read back by symbolic=True; RigidBodyInertia for both types), section 2.7 (options A explicit, B written into the
  items, C symbolic values in C++; B proposed), sub-steps 2 and 3, decisions P1 and P3 revised, P6 new.

<a id="rg9-11-8-2"></a>
<a id="rg9-11-8-3"></a>
### RG9.11.8.2 and RG9.11.8.3 — dv/dq of the markers; the contacts and the rolling disc on the connector interface (2026-11-02, #3201, #3204)
- What the code does: design section 8.5. The two sub-steps are one commit, as the markers' function is tested only
  through the connectors that request it.
- Not by AccessFunctionsAD: its functors are switched off by default (`accessFunctionsByAD`) and normalize the Euler
  parameters, while the markers and the system use them unnormalized; the pure functions of RigidBodyMath with forward
  differences match the forms the markers give.
- Debugging (scratch scripts on rollingCoinPenaltyTest.py after 0.3 s, local against doSystemWideDifferentiation):
  - Tait-Bryan angles matched at once, 1e-8 in both blocks, dv/dq included.
  - Euler parameters: first 450 % in the q block and 9 % in the q_t block. The q_t block was rank 1: a directional
    derivative of the residual (mbs ComputeODE2RHS) confirmed the system-wide one, and the cause was the perturbed
    local angular velocity `A^T omega` (A^T A - I of 1e-5 against a step of 1e-7 relative); fixed by adding only the
    increment. Then dv/dq in the forms of the markers (Glocal) instead of `G theta_t`. Left: 1 % in the q block,
    0.5 % at zero velocities - the direction along e (RG9.11.8.5).
- Newton iterations / Jacobians of all test models, before (these connectors numerical by CSystem) and after:

  | model | Newton | Jacobians |
  |---|---|---|
  | rotatingTableTest.py | 26785 / 26740 | 71 / 73 |
  | ballBearingTest.py, carRollingDiscTest.py, createSphereQuadContact2.py, createSphereTriangleContact.py | equal | equal |
  | createSphereQuadContact.py, sphereTriangleTest2.py, restartFileTest.py | within 6 | equal |
  | ConvexContactTest.py | 4330 / 4361 | 4330 / 4361 |
  | contactComparisonTest.py | 220466 / 218603 | 220466 / 218603 |
  | rollingCoinPenaltyTest.py (Euler parameters) | 9568 / 11351 | 9568 / 11351 |
  | contactSphereTorusMomentumTest.py (Euler parameters) | 12297 / 14487 | 12297 / 14487 |
  | coordinateSpringDamperExt.py (#3204) | 4780 / 1411 | 4780 / 1411 |

- New references (runTestSuiteRefSol.py, with the old values in the comments): ballBearingTest, carRollingDiscTest,
  ConvexContactTest, coordinateSpringDamperExt (2 %: friction with stick-slip follows another path with the correct
  Jacobian), createSphereQuadContact, createSphereQuadContact2, createSphereTriangleContact, symbolicUserContactTest,
  restartFileTest (rounded to 6 digits, -2.93176 -> -2.931759), contactSphereTorusMomentumTest, rollingCoinPenaltyTest,
  rotatingTableTest, sphereTriangleTest2, and the mini example ObjectConnectorRollingDiscPenalty. The AVX2 references
  of these models are stale until the fast-module logs are rerecorded.
- test_systemJacobians.py: a model of a rolling disc and two rigid spheres in contact, both with friction, on Tait-Bryan
  angles, compared with the system-wide Jacobian also at velocities, and parallel against serial (37 cases).
- Test suite passed, pytest passed, `exudev generate --all-checks` ok (17 descriptions checked again: none states how
  the Jacobian is computed).

<a id="rg9-11-6"></a>
### RG9.11.6 — The algebraic blocks on the interface of the local Jacobians (2026-11-02, #3201)
- `CObject::ComputeLocalJacobianAE(systemData, temp, usesVelocityLevel, filledJacobians)`: the default (CObjectBody.cpp)
  is ComputeJacobianAE of a body with algebraic equations - it throws for an object that is neither body nor
  constraint, as CSystem did -, `CObjectConstraint::ComputeLocalJacobianAE` (CObjectConnector.cpp) C_q on the connector
  interface plus ComputeJacobianAE_AE, else the marker data and ComputeJacobianAE. CSystem::ComputeObjectJacobianAE,
  used by JacobianAE, the reaction forces, the constraint Jacobian derivative and mbs.ComputeItem, calls it.
- `CSystem::AddLocalJacobianAE(temp, ltgAE, ltgODE2, ltgODE1, usesVelocityLevel, filledJacobians, factors, jacobianGM)`
  with `JacobianAEFactors`: the code of the loop of JacobianAE, unchanged. The nodes with algebraic equations (the Euler
  parameter constraint) are added by the same function with local-to-global lists of their coordinates instead of
  AddSubmatrixWithFactor at offsets; a node block by ODE2_t is accepted now, ODE1 and AE blocks still raise.
- The units of the parallel AE rows (RG9.11.5.2) are these compute and add functions, as for the ODE2 rows.
- The same results: the test suite passes at its tolerance, pytest passed. test_systemJacobians.py: a model of two
  rigid bodies on Euler parameters for the node constraint, dense and sparse (44 cases); the joints stay on Tait-Bryan
  angles, as their rotation Jacobians, like those of the connectors, have no direction along the Euler parameters (the
  first try with joints on Euler parameters differed by 10 %, RG9.11.8.5).

<a id="rg9-11-8-5"></a>
### RG9.11.8.5 — The direction along the Euler parameters (2026-11-02, #3201, #3206)
- The question of the maintainer, whether the Euler parameter constraint of the nodes is considered: it is, in the
  blocks of the nodes with algebraic equations. The direction along e matters for another reason: the predictor of the
  time integration leaves the unit sphere, so the first Newton increment of every step has a part along e, and the
  residual depends on e there (`A - I` quadratic in e). Measured on rollingCoinPenaltyTest: at index 3 no drift and
  still 12036 / 9654 iterations local / system-wide, the first contraction 3e-3 / 4e-5; at index 2 (as the model runs)
  |e|^2 - 1 = 7e-7 after 5000 steps. The difference was the radial column, `(J_loc - J_sys)[:, e] ~ e^T`.
- `CMarker::GetRadialKinematics`, `CObjectBody::GetRadialKinematics`, `RigidBodyNodeRadialKinematics` (CObjectBody.cpp),
  `MarkerTemp::radial_q`; MarkerBodyPosition, MarkerBodyRigid, MarkerNodeRigid and ObjectRigidBody declare it
  (definitions, itemFunctions.py).
- CObjectConnector.cpp: AddRadialDerivativePosition and AddRadialDerivativeRigid after the chains of the automatic and
  numerical connector Jacobians; ConstraintJacobianRigidMarkers and ConstraintJacobianPositionMarkers write the part
  along e into `temp.localJacobianAE_ODE2radial` (RadialJacobianAE), which CSystem::ComputeObjectJacobianAE clears and
  AddLocalJacobianAE adds to the rows of the equations only.
- CObjectRigidBody::GetJacobianTransposedTimesVectorDerivative: `Glocal^T (p x 2 (A - I)^T f) e^T/(e^T e)`; without it
  the spring-dampers on position markers of Euler parameter bodies differed by 1e-3 of the largest entry in the rows of
  the rotation parameters.
- First try: the part along e in C_q for both blocks; 37 test models changed and restartFileTest and userJointTest did
  not converge - also with doSystemWideDifferentiation, which showed #3206: NumericalJacobianAE got the factor 1 for
  the block of the reaction forces (JacobianAE did not pass factorODE2_AE, factorODE1_AE, factorAE_AE) and used dC/dq^T,
  which has the part along e that the reaction forces of the residual do not. Fixed: the factors are passed, the block
  is the reaction forces of the unit multipliers, the velocity-level rows need no list any more.
- test_systemJacobians.py: the coordinates of the Euler parameter nodes back on the unit sphere; RigidBodiesEP, JointsEP
  and ContactsEP (the ODE2 blocks, the constraint Jacobian, the contacts at velocities), 63 cases.
  test_connectorInterface.py: C_q of JacobianAE against finite differences without the projection on the tangent
  space, and the reaction forces against the block of the reaction forces.
- Newton statistics (scratchpad newtonStats.py, all test models): 1167234 / 1154574 iterations against RG9.11.8.3, 25
  models fewer, 11 more; each of the latter with the same count as with doSystemWideDifferentiation (sliderCrank3Dtest
  2000 / 3172, carRollingDiscTest 584 / 696, userJointTest Jacobians 868 / 1285). 38 test models and 3 mini examples
  with new references, the old values in the comments.

<a id="rg9-11-8-4"></a>
### RG9.11.8.4 — The hydraulic actuator and the intrinsic rigid-body spring-damper (2026-11-02, #3201)
- Decided by the maintainer: HydraulicActuatorSimple and the intrinsic RigidBodySpringDamper get a local Jacobian;
  ReevingSystemSprings, the cable contacts and the Python user functions stay numerical by CSystem, serially.
- CObjectConnectorHydraulicActuatorSimple: ComputeActuatorForce a template (the length by the square root of a
  nonzero square, as in the spring-damper), ComputeConnectorForcePositionDiff, ComputeJacobianODE2_ODE1 analytic;
  GetAvailableJacobians with ODE2_ODE2_function, ODE2_ODE2_t_function and ODE2_ODE1_function.
  `CObject::ComputeJacobianODE2_ODE1` gets the system data and the temporary data (ObjectGenericDAE ignores them).
- CObjectConnectorRigidBodySpringDamper: its own ComputeJacobianODE2Connector (`ItemConnectorInterface('Rigid',
  jacobian=False)` plus the function), numerical by the marker kinematics for the intrinsic formulation, automatic
  otherwise; GetAvailableJacobians with the function flags unless a user function defines the force. The automatic
  derivatives of ExpSE3 and LogSE3 are not defined at zero (the norm of the rotation vector).
- test_systemJacobians.py: a model of a hydraulic actuator lifting an Euler parameter body, valves open, chamber volume
  change on, in the ODE2 and ODE1 comparisons; RigidBodies has had an intrinsic one, now on the local Jacobian (71
  cases). Newton statistics: the same for both test models, symbolicUserConnectorsTest 2309 / 2368.

<a id="rg9-11-5-2"></a>
### RG9.11.5.2 — The AE rows in parallel (2026-11-02, #1202, #858)
- CSystemJacobians.cpp: `LocalJacobianAEBlocks` (a template over the blocks of the local AE Jacobian, given to a
  functor), AddLocalJacobianAE and the new AppendLocalJacobianAE on it; `ComputeNodeJacobianAE` (the node block and
  its local-to-global lists in `temp.nodeLtgAE`, `temp.nodeLtgODE2`); JacobianAE in parallel above
  multithreadedLowerLimitJacobians over objectsConstraintWithAENoUF and nodesODE2WithAE, then objectsBodyWithAE and
  objectsConstraintWithAEUF serially; the serial path as before.
- The reaction forces are computed in parallel through ComputeObjectJacobianAE, so the constraints'
  local Jacobians are thread-safe.
- Measured (scratchpad benchAE.py, a chain of rigid bodies with alternating revolute and spherical joints, sparse):
  2000 joints 2.44 / 1.26 / 0.65 / 0.34 ms for 1 / 2 / 4 / 8 threads; 200: 0.24 / 0.04; 20: 0.03 / 0.01. The ODE2 rows
  of the same model, 1.6 ms, stay serial: the rigid bodies on Euler parameters differentiate numerically.
- threadsSuite.py (the test suite with 4 threads and every lower limit 1, multithreadedLowerLimitJacobians added): all
  353 test models and mini examples ok. test_systemJacobians.py: the constraint Jacobian in parallel against the
  serial one for Joints, JointsEP and EulerParameters, dense and sparse (77 cases).
- The cost per item (#858): not implemented, see design section 9; RG9.11.5.3 asks for the decision.

<a id="rg12-67-design-2"></a>
### RG12.67 — The design of the symbolic parameters revised again (2026-11-02, #3159, #3207)
- The maintainer's arguments of 2026-11-02, not yet a decision: items to be symbolic become user items, whose
  parameters carry the values (user loads and Python objects for markers and the generic objects when needed); named
  variables, expressions possible; the bookkeeping in the symbolic layer, not in mbs, as MainSystem and the C++ do not
  know SymbolicSystem; one systematic function of UserItemCreator that adds a user item; _massPointNodes not needed; no
  workaround for the loads now.
- symbolicParameters.md: 2.2 the slots are the parameters of the user items, `ParameterBindings` in exudyn.symbolics
  with one variable per name; 2.3 rewritten (option D, `UserItemCreator._AddUserItem`, `creator.parameterBindings`,
  the inertia, the frames of the user joints, `Keep` by hand); 2.4 SymbolicSystem with `creator.parameterBindings`;
  2.6 and 2.7 with D proposed and B dropped; section 3 and the decisions P1, P6 revised, P7 new.
- #3207 raised: the connectors on user mass points through body markers, `_massPointNodes` removed; SymbolicSystem
  takes position markers of user bodies (SymbolicSystem._MarkerData), so nothing else is needed. In RG12.67.2.
- Plan: RG12.67.2 and RG12.67.3 revised.

<a id="rg12-73-1"></a>
### RG12.73.1 — The names of symbolic variables evaluated (2026-11-02, #3208)
- New step RG12.73, as the maintainer asked for a new step for the evaluation.
- Probed: two `Real('a', ...)` are two variables, `((a * a) + a)` evaluates to 168, Diff by each differs;
  `esym.Function` refuses two inputs named `a` and a named variable of the result that is neither an input nor the one
  of `esym.variables`; `VariableSet.Add` refuses a second name; `SymbolicSystem` refuses two parameters of one name.
  Accepted: `Function(a0*esym.variables.Get('a'), [a0])` with `a0` named `a`. `esym.variables` is read by the
  recordings at every call; `VariableSet.Add` returns the variable.
- The library's own named variables: recordings name their arguments, SymbolicSystem its coordinates at every build
  (`_Backend.Real`, `Variable`), `exudyn.shells` creates `x`, `y`, `z` per call.
- docs/dev/design/symbolicVariableNames.md: options A, B, B', C; recommendation A with the missing check and C for the
  parameters of a system, B not as the default; decisions N1-N3. RG12.73.2 implements what is decided.

<a id="rg9-11-11"></a>
### RG9.11.11 — The rigid body's own ODE2 Jacobian (2026-11-02, #3209, #858)
- #858 closed as superseded (maintainer), RG9.11.5.3 and RG9.11.5 closed.
- CObjectRigidBody.cpp: `RigidBodyODE2LHSTemplate` (the terms of ComputeODE2LHS for Real and AutoDiff, the Glocal and
  Glocal_t formulas of RigidBodyMath written out for the number type, A unnormalized for Euler parameters, of
  RigidBodyRotationMatrixAD for Tait-Bryan angles), `ComputeJacobianODE2_ODE2` with `AutoDiff<8, Real>`,
  GetAvailableJacobians moved from the definitions into the cpp, with the function flags on Euler parameters and
  Tait-Bryan angles.
- test_systemJacobians.py: FreeBodies (Euler parameters and Tait-Bryan angles, with and without a center of mass) at
  velocities against the system-wide Jacobian; the parallel ODE2 comparison at velocities, as at zero velocity the
  ODE2 Jacobian of the joint models is now exactly zero (it was numerical noise); 84 cases.
- benchAE.py: the ODE2 Jacobian of 2000 bodies 1.57 ms before, 0.39 / 0.20 / 0.11 / 0.06 ms for 1 / 2 / 4 / 8 threads.
- Newton statistics: the same counts except sliderCrank3Dbenchmark 33733 / 33734 and userJointTest 17455 / 17448;
  22 test models with new references, 6e-14 to 7e-9.

<a id="rg9-11-7"></a>
### RG9.11.7, RG9.11.12, RG9.11.8 — The loads on ODE1 coordinates, the 2D rigid body, the plan of the Jacobians (2026-11-02, #1548, #3210)
- Maintainer 2026-11-02: RG9.11.8 closed; ObjectFFRFreducedOrder, ObjectKinematicTree and ObjectRigidBody2D should have
  an efficient Jacobian (none of them had one: all three declared ODE2_ODE2 without a function), ObjectANCFBeam may be
  involved, ObjectFFRF and ObjectALEANCFCable2D are for research only; the pressure equations of the hydraulic actuator
  analytic; one issue each for MarkerSuperElementRigid (not touched now, to be revised), MarkerKinematicTreeRigid
  (earlier, for convergence and performance) and the Lie group nodes; RG9.11.7 now. Issues #3210-#3217, steps
  RG9.11.12-RG9.11.17.
- CObjectRigidBody2D.cpp `ComputeJacobianODE2_ODE2` in closed form, GetAvailableJacobians with the function flags when
  the center of mass is off the node; test_systemJacobians.py FreeBodies with two such bodies.
- CSystem: `ComputeODE1SingleLoad` (ComputeODE1Loads a loop over it), `JacobianODE1Loads` in JacobianODE1RHS with
  `computeLoadsJacobian` (a new argument, also of MainSolverBase.ComputeJacobianODE1RHS, passed by the implicit and
  static solvers), the ODE1 columns of JacobianODE2Loads; `MainSystemData::PyAddODE1LoadDependencies`,
  `mbs.systemData.AddODE1LoadDependencies`; temp.tempIndex5.
- test_systemJacobians.py ODE1Loads (a load on an ODE1 and one on an ODE2 coordinate, both reading ODE1 and ODE2
  coordinates and a velocity): the blocks differ by 50-90 % without the load Jacobian, 3e-8 with it; models may
  declare dependencies after Assemble. python/TestModels/ode1LoadsJacobianTest.py: 566 / 387 Newton iterations
  without / with, the results within 2e-7.
- The derivative terms d(M q_tt)/dq and d(C_q^T lambda)/dq are in no Jacobian, also not the system-wide one; an
  evaluation of their own (RG9.11.18). The solver chapter of the manual is RG3.48 (#3223).
- Test suite 255 + 99 passed; 86 cases of test_systemJacobians.py.

<a id="rg12-74"></a>
### RG12.74 — The users of the data coordinates (2026-11-02, #3218, #3219, #3220)
- Maintainer 2026-11-02: add the users of the data coordinates named after RG12.66 - the turns of the torsional
  spring-damper, friction of the sphere contact, the adaptive step size of the Python solvers - as a step and resolve
  it now.
- userItems.py, the turns (RG12.74.1): `_TorsionalSpringDamperFunctions(unlimitedRotations)`, the angle of atan2 plus
  2 pi round((d - angle)/2 pi), d the angle of the last step in a NodeGenericData, which the post-Newton function sets
  (CObjectConnectorTorsionalSpringDamper::PostNewtonStep); esym.round while recorded, round() for numbers; a variant of
  the kind, the default unlimitedRotations=True no longer refused. symbolicUserRigidBodiesTest refuses
  springTorqueUserFunction instead. symbolicUserTorsionalTurnsTest: a free disc at 20 rad/s turns by 3.5 rad; Python
  and recorded against built-in 1.2e-10 (generalized-alpha) and 0 (RK44), SymbolicSystem against SolveDynamic the
  same, without the turns 26 rad/s off.
- userItems.py, friction (RG12.74.2): `_SphereSphereContactFunctions(restitution, friction)`; with friction rigid
  markers (as the built-in Create function), the parameters plus [dynamicFriction, frictionProportionalZone], the data
  [gap, impact velocity, tangential speed], the force of CObjectContactSphereSphere::ComputeContactForces - the
  velocities at the contact point with omega x (r + g/2) n, -mu N vT/zone while the speed of the data is below the zone,
  else -mu N vT/|vT| - with the torques about the centers, and in the post-Newton step the speed (the rotations only in
  contact) and the error |mu N/zone (|vT| - zone)| of a switch. symbolicUserContactFrictionTest: a ball sliding at 1 m/s
  rolls at 5/7 of it after 0.1 s, slip 2e-7; against built-in 1.4e-12, SymbolicSystem against SolveDynamic 5e-13.
- solvers.py, the step size (RG12.74.3): `_StepSize` with Next, Reduce and Accept as UpdateCurrentTime, ReduceStepSize
  and IncreaseStepSize of the implicit and the explicit solver (min(0.75 h, h_rec), else adaptiveStepDecrease or 1/2,
  minimumStepSize; adaptiveStepRecoverySteps steps of at most adaptiveStepRecoveryIterations Newton and discontinuous
  iterations, then adaptiveStepIncrease or 2, up to the nominal step); `_PostNewton` returns None for a rejected step
  and puts the data back to the start of the step; t accumulates as in CSolverBase with adaptiveStep; the statistics
  give rejectedSteps; the output with the step size of the step. symbolicSolversAdaptiveStepTest: a bouncing ball with
  useRecommendedStepSize, 511 / 516 steps with 4 rejected (generalized-alpha / RK44), against SolveDynamic 7e-12 / 0;
  with a constant step 6e-4 / 4e-3 off. Not: a step size reduction when Newton does not converge, which still raises.
- The existing symbolic test models give the same results.

<a id="rg3-49"></a>
### RG3.49 — The internal functions out of the reference pages (2026-11-02, #3224)
- tools/generators/utilityDocsModel.py `_IsInternal`: functions, classes and methods whose names start with an
  underscore, not the special methods such as `__init__`, are not parsed, so neither the pages nor the keyword lists of
  confHelperPyUtilities.py have them; 1120 lines fewer in 15 pages (symbolics.systemModel 37 functions).

<a id="rg3-46-1"></a>
### RG3.46.1, RG3.46.2, RG3.48 — genericDAE.md, symbolicHT.md and systemJacobians.md into the manual (2026-11-02, #3221, #3223)
- genericDAE.md: its equations, items and Jacobians were in the description of ObjectGenericDAE already; added the
  section Solvers (implicit and static solvers all equations, explicit solvers without algebraic coordinates, the
  q_t-dependent constraint and the compliance, SymbolicSystem); the handling of C_y^T lambda by the solvers into the
  system equations of solver.md; the design's symbols in systemModel.py now refer to the description.
- symbolicHT.md: HT and Twist have their reference pages (pybindDataStructures.py, the HT notebook), the symbolic
  classes and the twist functions are in symbolicUserFunctions.md and userItems.md; added the reason why a recording
  holds matrix and vector operations, not HT products. Nothing for the internals.
- systemJacobians.md: solver.md is "Solvers and system equations" with the section The system equations - the system
  equations of motion moved there, how the items assemble them, the system Jacobian: the blocks, the local Jacobian of
  each kind of item, the items differentiated numerically by the system, parallel and serial, the options of
  numericalDifferentiation, the loads with AddODE2LoadDependencies and AddODE1LoadDependencies, what the Jacobian leaves
  out, the Euler parameter direction -; the Newton section no longer says that J_SO_FO and J_FO_SO vanish. The inside
  - the functions, the connector interface with dv/dq and the direction along e, the parallel assembly, the
  measurements behind the decisions - in the new docs/dev/internals/systemJacobians.md (WORKFLOW.md §8.4).
  checkHeadings.py takes Jacobian as a name.
- The three design files removed; git keeps them (their last version is in d1b3e6dd).

<a id="rg3-46-4"></a>
### RG3.46.4, RG3.46.5 — ObjectGenericDAE with the Jacobians of its algebraic equations; no design files cited (2026-11-02, #3227, #3221)
- The equations of ObjectGenericDAE with C_q^* = C_q + dc_u/dq and C_y^* = C_y + dc_u/dy in the reaction terms - what
  CObjectGenericDAE::ComputeJacobianAE gives the solver -, and the constant terms f_0, g_0 beside c_0 (forceVector,
  rhsVector).
- The docstrings of SolveExplicit, SolveGeneralizedAlpha and the header of userItems.py without the citations of
  docs/dev/design (maintainer 2026-11-02: the design files are temporary; docs/dev/internals rarely, with a note that it
  is on GitHub). No other citation in python/exudyn, definitions and docs/manual.

<a id="rg6-20-7"></a>
### RG6.20.7 — Lines, texts and points in the coordinates of the graph of a scope (2026-11-04, #3237)
- Maintainer's decisions on the proposal of design scope.md section 7: polylines, outside the automatic range, named
  lines in the legend, one list with the view and the scope in each item (each scope walks through it; many items in
  many scopes are rare), the functions on `SC.renderer.scopes`; defaults color [0,0,0,1], width -1, front False,
  name ''; points radius 4; texts size 1, front True.
- `MainRendererScopes` (Python `RendererScopes`, `SC.renderer.scopes`): SetGraphics (all items checked before the list
  is replaced), AddGraphics (a dict or a list), GetGraphics (all keys of the type), ClearGraphics(view, scope). Each
  key is checked; a key the type does not have, a wrong type, a view outside 0-3, a scope outside 0-2, a line of one
  point, width, size or radius not > 0 raise with the key named.
- The list `ScopePlot::StaticGraphics` lives in VisualizationSystemContainer under an atomic flag; BuildScopes copies
  the items of its view under it. AddScope draws the items with front False after the grid, the sensor lines, then
  the items with front True, clipped to the plot area; a point is a circle of 16 lines, a text only if its position
  is inside; named lines follow the sensor lines in the legend. A scope with static items and no sensor is drawn.
- scopeTest.py: line, point and text on the raytracer image, clipped outside the zoom, other view and scope not drawn,
  GetGraphics, ClearGraphics, four refusals. scopeExample.py: the circle of the first mass and the reach of the second
  as named lines in the path plot, the pivot as a point and a text, the start height in scope0. Checked by eye on
  the raytracer image; OpenGL draws the same primitives.

<a id="rg6-20-10"></a>
### RG6.20.10 — The legend of a scope with the sensor name and the component; the background; the sensors setting (2026-11-04, #3248)
- Maintainer (testing in OpenGL: "works really fine"): the legend wrote "mass0, PositionY"; a sensor name is unique
  and the user names it by what it measures, so the legend writes the name and only the component: `m0.Pos-X`
  (different from PlotSensor, but shorter); over another sensor `m0.Pos-Y over m0.Pos-X`. A sensor of one component
  writes its name only; a component without a letter (not X, Y, Z or a tensor component) as `name[3]`.
- `backgroundColor` default [0.95, 0.95, 1, 0.9]; the description of `sensors` with the examples `[[0,1],[2,3]]` and
  `[[1,1,1,0,0]]`; scopeExample.py names its sensors m0.Pos, m1.Pos, m1.Vel. Checked on the raytracer image.

<a id="rg6-20-11"></a>
### RG6.20.11 — The settings dialog refuses what a list or matrix setting refuses (2026-11-04, #3249)
- Maintainer: an illegal input in the sensors field raised an exception or opened an empty error window. CheckType
  checked only fixed sizes: an IndexMatrix of open size passed with rows of different length or floats, and
  `len()` of a number raised in CheckType itself; SetDictionary then raised in the tkinter callback.
- CheckType checks lists (of numbers; of integers for the Index types) and matrices (a list of rows of equal length)
  by their elements. OnCellCommit sets the value on the settings in a try; what they refuse is put back in the tree
  and the settings, the undo entry dropped, and the message shown with the description of the setting, the editor
  staying open (ESCAPE keeps the original value). The other inputs: enums are picked from a combo box, numbers and
  file names were checked already; the item dialog of the right mouse button (EditDictionary) gives its values to
  ModifyObject, which checks them.
- test_guiValues.py: three accepted and six refused values.

<a id="rg6-20-12"></a>
### RG6.20.12 — The look of all scopes in sensors.scopes; location, minMaxX, minMaxY (2026-11-04, #3251)
- Maintainer, after testing: axesColor, backgroundColor, frameColor and lineWidth per scope are settings nobody
  changes per scope; no font size; region [xmin, ymin, xmax, ymax] not intuitive; horizontalZoom and verticalZoom
  apart in the sorted lists.
- `VSettingsScopes` at `visualizationSettings.sensors.scopes` (the maintainer's first choice): frameColor,
  backgroundColor, axesColor, lineWidth, fontSizeFactor (0.75, the former fontSmallFactor, times
  window.globalFontSize of the view and the display scaling, so it still differs per window). `VSettingsScope`:
  `location` [[xmin, ymin], [xmax, ymax]] (the maintainer suggested scopeLocation; inside scope0 the prefix
  repeats, so `location`), `minMaxX`, `minMaxY`. Renamed without deprecation, introduced after 1.13.0.
- The settings type `StdArray22F` (a 2x2 float matrix, MatrixFloat [2,2] in the dictionaries and the dialog), as
  StdArray33F: definitionTypes, typeModel, itemModel, the print of the header emitter; a CppValue default, which
  the documentation of memberDefaults now renders. scopeTest.py, scopeExample.py, the GUI chapter; checked on the
  raytracer image with fontSizeFactor 1 and a changed location.
- The maintainer checked #3249 in the GUI: a wrong sensors value shows the message.

<a id="rg6-20-13"></a>
### RG6.20.13 — xResolution 1, the values over another sensor, the default scopes a third of the height; the internals page (2026-11-05, #3252)
- Maintainer: xResolution 1 is by far sufficient (and faster); the default scopes a third of the height each, so that
  the three default scopes fill the window; the values of a plot over another sensor were not defined - every value,
  and every (1/xResolution)-th below 1. Before, they were thinned to about xResolution per pixel of the width.
- scopeTest.py (the regions of the image in thirds), scopeExample.py (scope2 at its default place), the GUI chapter.
- docs/dev/design/scope.md (the request, the questions, the proposals) replaced by docs/dev/internals/scope.md, the
  current state only: the settings, ScopePlot, the renderers, the static graphics, the locks, the tests.
- The maintainer checked #3249 in the GUI (a wrong sensors value: the message). EditLoose of the scratch edit tool
  ate the leading spaces after a replaced text ending with a newline (definitions: one line of indentation) and a
  space in GUI.md; both fixed, and the tool.

<a id="rg6-20-9"></a>
### RG6.20.9 — mbs.Reset() and item changes while the renderer runs: a lock of the system (2026-11-04, #3239)
- `PostProcessData::accessSystem` (an atomic flag per system) with `accessSystemOwner` (a nested lock in the same
  thread passes, so that AddNode in interactive mode, which assembles, and the raytracer inside the graphics update
  need no counting) and `SystemAccessGuard` (RAII; also on exceptions). It is not released by PostProcessData::Reset,
  which MainSystem::Reset calls while holding it.
- Main thread: MainSystem::Reset, AddMainNode/Object/Marker/Load/Sensor, PyDelete* and PyModify* of the five kinds,
  Assemble, PySet*Parameter for names starting with V. Not the other Set*Parameter: a pre-step function sets them
  every step, and they are values in place - with the lock the solver would wait for every frame of the renderer.
  While waiting, the main thread computes the user function drawings that the render thread requests while it holds
  the lock (it waits up to 200 ms for them); within ProcessUserFunctionDrawing the main thread does not lock.
- Render thread: VisualizationSystem::UpdateGraphicsData after the visualization state is taken, the traces
  (GetSensorsPositionsVectorsLists) and the scopes (ScopePlot::BuildScopes, all systems in order; the main thread
  holds at most one, so no deadlock).
- rendererItemChangesTest.py (new): 20 runs of Reset, build, solve, add and delete a sensor, modify a connector, set
  graphics, assemble, raytracer image; equal results. In the suite no window opens, so the lock is not contended there;
  run directly, the renderer draws meanwhile - to be run by the maintainer once (no window from here).

<a id="rg11-4-1"></a>
### RG11.4.1 — GetDictionary and SetDictionary of the renderer and exudyn.config (2026-11-05, #2075)
- exudyn.config had them already (#2685, GetDictionary, SetDictionary, GetDefaults); the renderer gets them:
  'materials' (GraphicsMaterialList.GetDict of each), 'scopeGraphics' (scopes.GetGraphics), 'states' (GetState of
  each enabled view, by its number). SetDictionary sets what is given - materials beyond the existing ones appended,
  the states by SetState without waiting for the renderer - and refuses an unknown key. Textures are not in it (images).
- GraphicsMaterialList.GetDict raised "Unregistered type SlimVectorBase<float,3>": the Float3 members are given as
  lists now.
- rendererDictionaryTest.py (new).

<a id="rg11-8-24"></a>
### RG11.8.24 — PrintToLog of the quiet test runs no longer recurses when a model switches printToConsole on (2026-11-05, #3066)
- PrintToLog (testRunnerTools.py) keeps the stream that redirect_stdout replaced; what arrives while it writes - exu.Print
  printing to sys.stdout with printToConsole set - goes to that stream instead of to exu.Print again.
- test_runnerLogs.py runs the case in its own process (the old code ended the process by a stack overflow): fails on
  the old code, passes now.

<a id="rg20-2-5"></a>
### RG20.2.5 — The error estimate of the order of the method for SolveImplicitRungeKutta (2026-11-04, #3250)
- Maintainer's questions (fourth answers): the unknowns per step - LobattoIIIA4 2 n + 2 m (A_2, A_3; Z_2, Z_3 for
  the two end equations c = 0 and C_q v + c_t = 0, no mu), Radau3 the same, LobattoIIIC4-GGL 3 n + 6 m (the first
  stage of IIIC is implicit too; Z_i and mu_i at each); step doubling only as the reference; the trapezoidal formulas
  evaluated without a solve, one step h^3 E and two substeps 2 (h/2)^3 E: their extrapolation is Simpson's rule, which
  is q1 of IIIA, so q1 - S = 0 identically - the error of order 4 has another constant that the three values of a
  step cannot see. Design advancedSolvers.md section 3.2.
- Measured against the true local error (a reference of 64 steps from the same start), N = 25 to 200 constant steps
  on 1 s, the middle stage recovered from the ends (V_2 = (6 (q1 - q0)/h - v0 - v1)/4): the trapezoid of one step
  overestimates by 850 to 73000, growing as 1/h^2; the fourth difference of the stage velocities of the previous
  and this step (Simpson's error) is 0.33 (oscillator) and 0.58 (pendulum) of it for all N; of the stage
  accelerations 0.8-1.06 without constraints, 12-690 (1/h^2) on the pendulum, as its stage accelerations are of order
  2; a fifth difference of the velocities for e_v 1.7-15, growing as 1/h. Radau3 with the cubic through four values:
  0.54-0.86.
- `_QuadratureError(points, b, stageValues)`: the integral of the interpolating polynomial by the weights of its
  Vandermonde system, against the quadrature of the method; the points of the previous step that coincide with a
  stage of this one are dropped. The first step by the trapezoidal rule (order 3), then the order len(points); e_v only
  without constraints.
- Automatic steps, tolerances 1e-4/1e-6/1e-8 on 2 s: 4 to 23 times fewer steps (pendulum, IIIA: 24, 52, 120 against
  128, 598, 2753), the global error of q a few times the tolerance; slider crank 1e-6 in 12 steps, 7e-7 (68, 4e-10).
  symbolicLobatto4Test.py: the slider crank, and the pendulum with the three methods at 1e-5 and 1e-7, the error below
  10 steps times the tolerance and the steps growing with the local order (IIIA 6.6, IIIC 4.4, Radau 4.3).

<a id="rg20-5-4"></a>
### RG20.5.4 — Exudyn's explicit solver: the eliminated coordinates fixed in the RK stages, with the value of their equation (2026-11-04, #3240)
- CSolverExplicitTimeInt kept a list of the eliminated coordinates and set them to 0 at the end of the step only;
  the stage accelerations of these coordinates were not zeroed, so the following stages moved them: RK44 of order 1
  on the chain of symbolicExplicitConstraintTest.py. The offset, factor1 and the reference values of the markers were
  ignored.
- `EliminatedCoordinate` (coordinate, constraint, the marker of the node, the value of the ground marker, the
  reference value of the node marker with includeReferenceCoordinate) and `FixEliminatedCoordinates(t, q, q_t, q_tt)`:
  the value of factor1 value1 - value0 - offset = 0 (offsetUserFunction at t), its derivative (offsetUserFunction_t,
  else 0), the acceleration 0; at each stage before the accelerations, the stage accelerations after them, the end of
  the step, the second approximation of the error estimate, and VelocityVerlet. An inactive constraint leaves its
  coordinate free; velocityLevel=True raises (notImplementedError), factor1 = 0 with the node at marker 1 raises.
- Result: Exudyn's RK44 equals SolveExplicit of the SymbolicSystem to 3e-17 (500 steps), also with the offset; DOPRI5
  (automatic) 3e-14 from a fine solution, ODE23 5e-9. symbolicExplicitConstraintTest.py compares them directly.
- distanceSensor.py (an ANCF cable clamped by CoordinateConstraints, RK44) changes from 1.86776431077868 to
  1.7491018054920742; with 4 times the steps the new result changes by 6e-10, so the old one was the error of the
  moving clamp in the stages.

<a id="rg20-2-7"></a>
### RG20.2.7 — ODE1 and data coordinates in SolveImplicitRungeKutta; errorEstimate in both solvers (2026-11-05, #3253)
- Maintainer: two estimators in each integrator, switched by a flag: the simple one of one step, and the accurate one
  of two steps, which needs the stage velocities of the previous step (not kept by the C++ solvers yet, resolvable).
  `errorEstimate='twoSteps'` / `'oneStep'`: the default of SolveImplicitRungeKutta 'twoSteps' (#3250), of
  SolveTrapezoidalGGL 'oneStep' (as before).
- SolveTrapezoidalGGL 'twoSteps': the trapezoidal rule against the parabola through the velocities at the start of the
  previous step and at both ends of this step (the accelerations without constraints; with them the velocity error
  by the mean accelerations, as 'oneStep' - with q only, the whipping chain took 79 steps and 1.6e-3 instead of 231 and
  2.1e-4). Ball 1e-6: 417 steps, 8.7e-5 against 419, 9.0e-5; chain 1e-5: 230 steps, 2.4e-4 against 231, 2.1e-4 - for
  a method of order 2 both estimates are of its order, so the steps are the same.
- SolveImplicitRungeKutta: the ODE1 velocities of the implicit stages as unknowns, Y_i = y0 + h sum A_ij Yd_j, the
  ODE1 rows of SystemResidual scaled by h, their Jacobian blocks from SystemJacobian by y and y_t; the data
  coordinates by _PostNewton in a loop around the step as SolveTrapezoidalGGL (a rejected step resets system.data).
  ODE1 beside the pendulum: order 4.0, 3.9, 3.0; the bouncing ball with a data coordinate (1000 steps): 4e-4 from
  generalized-alpha with 20000 steps (generalized-alpha with 1000: 7e-4); 'oneStep' 295 steps against 31.
- symbolicLobatto4Test.py renamed to symbolicImplicitRungeKuttaTest.py (maintainer's approval), with the ODE1, data
  and errorEstimate cases; symbolicTrapezoidalGGLStepSizeTest.py with 'twoSteps' on the ball.

<a id="rg19-6-1"></a>
### RG19.6.1 — exudyn.cUtilities.KinematicChain: forward kinematics, Jacobian, static torques in C++ (2026-11-05, #3259)
- Decision 10 of the design (maintainer 2026-11-05): the kinematic chain as a class of cUtilities, each function with
  a Python version in exudyn.robotics, cross-referenced.
- src/Pymodules/PyKinematicChain.h, bound in definitions/pybindCUtilities.py (the first class of a C submodule: the
  stub writer of the submodules removed the @overload lines without their indentation, so the methods of the class were
  indented twice - pybindEmitter.py removes the whole line now): KinematicChain(jointTypes, preHTs, localHTs, parents,
  masses, COMs, baseHT, toolHT, gravity), NumberOfLinks, JointHT, LinkHT, ToolHT, Jacobian(q, linkIndex, localPosition)
  (6 x n, the point in the link frame), StaticTorques(q) in one backward pass over the links (mass and first moment
  summed per subtree). Robot.GetKinematicChain() creates it; Robot.StaticTorques is the same O(n) algorithm in Python
  now (before a Jacobian per link, O(n^2)), the docstrings of JointHT, LinkHT, Jacobian and StaticTorques name the C++
  version.
- Measured (design section 2.1): Jacobian and StaticTorques 1.0 us against 23 and 130 us in Python, LinkHT 1.7 us
  against 17; the kinematic tree step of the Puma560 with the static compensation 14.9 us instead of 145 us.
  kinematicChainTest.py: the chain against Robot for the Puma560, UR5, Panda (with base and tool) and a tree with
  prismatic joints, to 1.4e-14, and the static torques against J^T m g summed.

### RG20.6.2 — the structure of the C++ solvers recommended (2026-11-05)
- Maintainer: LobattoIIIA4 (constraints at the end), LobattoIIIC4-GGL and Radau3 are the candidates for C++ next to
  TrapezoidalGGL, the automatic step size for TrapezoidalGGL and one Lobatto method, the other tested variants not; no
  Python interfaces of the new solvers (the pre- and post-step functions and the symbolic solvers serve); the class
  structure wanted.
- Design cppSolverStructure.md section 8: one class CSolverImplicitRungeKutta for the four methods, the method as data
  (tableau, constraint form, order; TrapezoidalGGL is the 2-stage Lobatto IIIA with GGL), overriding Newton() as the
  explicit solver does, so the step loop, the discontinuous iteration, the output and the restart stay in CSolverBase
  and nothing of the existing solvers changes; a stage Newton with its own unknowns and matrix (simplified Newton on M,
  K, D, C_q of the start of the step, H left out); StepErrorEstimate (oneStep, twoSteps) and a StepSizeController
  shared with ODE23 and DOPRI5 (RG20.8.2); four DynamicSolverType values and timeIntegration.errorEstimate.
- The automatic step size is a function of the tableau: restricting it to two methods saves no code; recommended for
  all four, tested on TrapezoidalGGL and LobattoIIIA4.
- Cases named for the maintainer: velocity-level and nonholonomic constraints (the Python solvers refuse them, the C++
  models use them often - rolling); Lie group nodes (refuse first or the exponential map in the stages); the initial
  accelerations; the dense output for RG20.4.2.

### RG20.9.1 — a library of test models and a solver study (2026-11-05, #3257, #3258)
- Maintainer: a set of models relevant for tests - mass oscillator, n-pendulum on distance constraints, a rigid body
  mechanism with joints and one with spring-dampers, the IFToMM slider crank, a kinematic tree, a 3D beam, an
  FFRFreducedOrder of a stored FEM, a GenericDAE with nonlinearities - in one file of TestModels with the names and a
  function returning the mbs of model i, fast settings, usable for solver tests and other things; the solver tests run
  in a more sophisticated way (the settings of a reference, varied steps, timeouts); test models may later use it.
- python/TestModels/modelLibrary.py: modelNames, GetModel(index or name, SC=None) -> assembled mbs,
  mbs.variables['modelLibrary'] (name, description, endTime, stepSize), SimulationSettings(mbs, stepSize, endTime),
  Result(mbs) (ODE2 and ODE1 coordinates). Each model runs in 1 to 31 ms with generalized-alpha; the FFRF model computes
  its 4 Hurty-Craig-Bampton modes of netgenTestMesh.npz in 0.17 s. The slider crank is a compact copy of
  sliderCrank3Dbenchmark.py, which stays self-contained as the benchmark file. modelLibraryTest.py (0.47 s) runs each
  model with its step and half of it (the state changes by less than 1e-2 of its size); modelLibrary.py is in
  DeliberatelyNotRun.
- python/testing/solverStudy.py: the reference by RK67 where it runs (the unconstrained models), else generalized-alpha,
  halving the step until two results agree to 1e-8 and extrapolating the last two by Richardson; then each solver with
  h, h/2, ... (ODE23, DOPRI5 with the tolerances 1e-3, 1e-4, ...), error against the reference, observed order, steps,
  time; a run beyond the time limit stopped by a PostStepUserFunction returning False.
- Found by the first study: generalized-alpha and TrapezoidalIndex2 of order 2 on all models but two; the 3D beam
  reduces its step (223 instead of 200 steps at h = 1e-3; its axial stiffness), the observed order irregular; the
  GenericDAE model gives its ODE1 coordinate (the lag) with order 1 - #3258, to be analyzed; TrapezoidalIndex2 fails on
  it because its constraintUserFunction ignores velocityLevel (a property of the model); the explicit solvers refuse all
  constrained models, as they should.

### RG2.9.3 — sliderCrank3Dbenchmark on the IFToMM model of 2026 (2026-11-05, #2326)
- Maintainer: the file of tmp/IftommSlidercrank2026 (made with Exudyn 1.10: Lie group nodes, AddRigidBody, joints by
  markers in the body frames, 5 s with the step 1e-4, the results file of the benchmark, timings of 10 runs) replaces
  the test model, cleaned up, the reference value computed again; directly the settings of the file, as test 0.5 s.
- Cleaned: CreateRigidBody with NodeType.RotationRotationVector (the same NodeRigidBodyRotVecLG), sensors stored
  internally, PlotSensor instead of the figures for the paper, the results file (time, slider position, crank angle
  every 0.01 s) written through OutputFilePath, doTimings kept (10 runs without graphics and sensors); sys.exudynFast
  dropped (the module is chosen by EXUDYN_MODULE now).
- Against the results file of 1.10: the slider position differs by at most 8.9e-7 m, the crank angle by 3.5e-5 rad
  over the 5 s (54.3 rad), at 0.5 s by 2e-7 rad. The test runs 0.32 s; its value 5.7911915103432765. The entry of the
  AVX2 list was removed: the model is new, its fast-module value is recorded from the next '_fast' log if it differs.

### RG19.2.3 — the URDF import in exudyn.robotics.urdfImport (2026-11-05, #3256)
- Decision 1 of the design (maintainer 2026-11-05): GetRoboticsToolboxInternalModel, LoadURDFrobot and
  GetURDFrobotData moved unchanged from robotics/utilities.py to robotics/urdfImport.py; utilities.py keeps AddLidar
  and the three names as deprecated wrappers; spotModel.py and serialRobotURDF.py import from urdfImport. The revision
  of the import itself is RG19.3.

### RG19.2.2 — the problems found in the robotics module fixed (2026-11-05, #3255)
- Maintainer: fix the problems of design section 2.1; the tool frame by the recommendation (the last link frame) also
  in the IK; the Panda of models.py named FrankaEmikaPanda; the Puma inertias by suitable values, said in the model;
  RTB 1.5.0 in venvP313rtb. Decided as well: urdfImport.py, future.py removed after the C++ IK, the cUtilities
  kinematic chain with Python versions of each function (design section 4.1).
- The tool frame: the marker of InverseKinematicsNumerical and the tool graphics of CreateKinematicTree on
  localHT * tool.HT of the last link (the redundant model had it there already); the two examples and the test model
  that wrote the goal as JointHT(q)[-1] * tool.HT use LinkHT - for the Puma560 and the test model's UR5 goal the
  values change only where the last localHT is not the identity. UR5 from 0.5 rad off: 35 of 40 found, before none.
- InterpolateHTs takes max(minSteps, ...): SolveSafe interpolates now, 12 of 40 random poses instead of 8 to 10, at 6
  to 8 ms - far apart poses need the LM IK of RG19.6, not more steps.
- The Panda against the RTB 1.5 Panda (modified DH): LinkDict2Robot builds a robot of dhMode modDHKK (preHT and localHT
  of ModDHKK2HT, the inertia parameters as given); the forward kinematics agree to 1e-15, the gravity torques to
  1e-15 once the COM of link 7 is moved into the flange frame (Gaz et al. give it in frame 7, the dictionary merges
  the flange offset 0.107 into the last link, as the RTB does). Its standard DH parameters were wrong (alpha of joints
  1 and 2 with the opposite sign, a = 0.088 instead of 0.0825 for joints 3 and 4; equal only at q = 0, 0.52 m apart at
  a general q) and agree now; the tool is the Franka hand of the RTB (0.103, -45 deg about z) instead of 0.11. The
  deprecated LinkDictModDHKK2Robot read the base and tool dicts as HT - fixed as well.
- Puma560: link 0 had I = diag(0, 0.35, 0), only the moment about its joint axis, the one that acts - now
  diag(0.2, 0.35, 0.2); link 2 had I_zz = 0.0125 with 0.066 + 0.0125 < 0.086 - now 0.025; said in the docstring. The
  gravity torques agree with the RTB's Puma560 (mass and COM unchanged).

### RG19.1.3, RG19.2.1 — the robotics module measured; the commented-out code removed (2026-11-05, #3254)
- Maintainer: start with the evaluation and the cleanup; the analytic 6R IK, and possibly other functions, also as C++
  in exudyn.cUtilities for efficiency.
- Measured (design robotics.md section 2.1): LinkHT/JointHT 17 us, Jacobian 23 us, StaticTorques 114 us per call for
  6 links; a kinematic tree step of the Puma560 9.8 us, 145 us with StaticTorques in the PreStepUserFunction - the
  Python kinematics in the control loop costs 15 times the step. InverseKinematicsNumerical: 0.1 to 0.5 ms per Solve,
  SolveSafe 8 to 10 of 40 random poses.
- Found: the IK puts the tool on the last joint frame, LinkHT and Jacobian on the last link frame (UR5: 0.079 m apart,
  no pose found; with the marker on localHT * tool.HT 35 of 40) - question 9; InterpolateHTs uses min instead of
  max (always one step); future.py's IK functions call functions exudyn.robotics no longer has; LinkDict2Robot
  refuses modDHKK, so the Panda of models.py cannot be made a Robot; the Puma560 inertias of links 0 and 2 violate the
  triangle inequality; venvP310rtb has RTB 1.1.1, not 1.5. Recommended: a C++ kinematic chain in cUtilities (forward
  kinematics, Jacobian, static torques in one pass, analytic IK for the spherical wrist and the UR geometry, LM IK),
  shared with ObjectKinematicTree; questions 9 and 10 added.
- #3254: 220 lines of commented-out code and the dead branches (if 0, if 1, if False) removed from all robotics files
  except future.py; ruff finds the same 64 items before and after.

<a id="rg19-6-2"></a>
### RG19.6.2 — the analytic inverse kinematics in KinematicChain and Robot; future.py removed (2026-11-05, #3260)
- Maintainer 2026-11-05: the IK of future.py for UR5 and 3R into C++, revised, with the 6R spherical wrist; then
  future.py removed (decision 1).
- The algorithm (C++ KinematicChain.InverseKinematicsAnalytic, Python Robot.InverseKinematicsAnalytic, the same steps):
  the chain as a product of exponentials T(q) = E_1(q_1) ... E_n(q_n) T(0), the axes and their points from the joint
  frames at q = 0, so that the DH convention, the base and the tool do not matter. The arm: q_1 puts the target point
  into the plane normal to the parallel axes 2 and 3 in which links 2 and 3 move it (a cos + b sin = e, 2 solutions),
  q_3 by the distance from axis 2 (subproblem 3), q_2 by subproblem 1. Spherical wrist: the wrist center (axes 4 and
  5 crossing, axis 6 through it) by the arm, then q_4, q_5 by subproblem 2 on omega_6 and q_6 by subproblem 1. UR: q_1
  for the crossing point of the axes 5 and 6, q_5 from the component of axis 6 along the parallel axes (independent of
  q_2 + q_3 + q_4), q_6 and the sum q_2 + q_3 + q_4 by subproblem 1, then the arm for a point of axis 4. Each solution
  checked by the forward kinematics (pose error relative to the size of the robot < 1e-8), angles in [-pi, pi) or
  nearest to qReference and sorted by the distance from it; a free angle of a singular configuration from qReference.
- future.py: ComputeIK3R, ComputeIKPuma560 and ComputeIKUR took the robot dictionaries with standard DH parameters
  only, printed, returned NaN rows and unchecked solutions; MakeCorkeRobot moved to urdfImport.py (without its print,
  the docstring argument corrected); the module and its baseline entries (pydoclint, ruff, stubtest) removed.
- Measured (design section 2.1): C++ 2.9 / 8.2 / 7.3 us (3R / Puma560 / UR5), Python 0.36 / 2.5 / 1.6 ms; 8 solutions
  for the Puma560, 2 to 8 for the UR5 at random poses - as many as 300 numerical solutions from random starts find.
- analyticInverseKinematicsTest.py: the 3R, the Puma560 of standard and modified DH parameters and the UR5 with base
  and tool, C++ against Python (5.5e-13), the pose of every solution, q among the solutions sorted by the distance from
  qReference (two elbow solutions of a nearly stretched arm can lie closer to the reference than q), out of reach,
  the singular wrist, the Panda refused.

<a id="rg19-5-1"></a>
### RG19.5.1 — Staubli TX2-90L, quintic and Cartesian linear profiles, SetControl and PreStepControl (2026-11-05, #3262)
- Maintainer 2026-11-05: the decisions 2 to 7 of the design (section 4.2); for a demo the Staubli TX2-90L with
  joint-space trajectories (polynomials of degree 5, constant accelerations) and a continuous path by the analytic IK,
  simple control without compensation.
- models.py: dhMode 'jointFrames' of LinkDict2Robot (per link the joint type and the translation of its joint frame,
  as a URDF); ManipulatorStaubliTX2_90L from tx2_90l.urdf (kinematics, joint limits, maximum velocities), estimated
  masses and inertias, PD values; a spherical wrist, 8 IK solutions.
- motion.py: ProfilePolynomial5; ProfileCartesianLinear (the path by InterpolateSO3, the IK nearest to the last
  evaluation, the velocities by the Jacobian, the accelerations by a difference in time; the end of the path followed
  in 50 steps at the construction, which checks the reach); the time laws polynomial5 and constantAcceleration.
- roboticsCore.py: SetControl, PreStepControl; CreateKinematicTree keeps its dict in the robot.
- The local data set copied to tmp/URDF (staubli, spot, mars rover, anymal, kuka).
- robotTrajectoryProfilesTest.py; Examples/staubliTX2trajectories.py (4.5 s, 4500 steps of generalized-alpha).

<a id="rg19-3-1"></a>
### RG19.3.1 — the meshes of a URDF as the graphics of a Robot, without the RTB (2026-11-05, #3263)
- Maintainer 2026-11-05: the Staubli with the graphics of its URDF, working with and without the data; the mesh
  inverted into the orientation of Exudyn and smoothened, as options of the loader.
- urdfImport.py: _ReadSTL (binary and text STL with numpy; numpy-stl not needed), LoadURDFgraphics (xml.etree; the
  options invertTriangles, orientTriangles, smoothNormals, smoothingAngle, addEdges), AddURDFgraphics (copies the
  link visualization, which a default argument may share; the joint and COM graphics off).
- models.py: 'urdfGraphics' of ManipulatorStaubliTX2_90L (file relative to the data set, base, links, source link);
  staubliTX2trajectories.py with the meshes from tmp/URDF or the simple graphics and a message.
- First attempt with the orientation and the normals of FromSTLfile copied (both inverted): the maintainer saw it
  inside out in the renderer (fine in the raytracer); the normals of the files are not used now.
- test_urdfGraphics.py: a URDF and binary and text STL of a brick written by the test; origin, scale, color, the
  orientation outward and inside out without the inversion, the links of a Robot.

<a id="rg19-3-2"></a>
### RG19.3.2, RG19.6.3, RG19.2.4 — one STL reader, the arm configuration, BuildFromDictionary removed (2026-11-05, #3264-#3268)
- Maintainer 2026-11-05: fuse the numpy STL reader with FromSTLfile, ideally without numpy-stl (the mass and inertia from
  the mesh), useNumpySTL to force it; elbow up/down etc. in the analytic IK, as a preference or by a function on all
  solutions; BuildFromDictionary removed; a step and issue for saving a Robot (HDF5) and for images in dicts; a
  proposal for the blending of PTP motions first.
- #3264: graphics._ReadSTL (moved from urdfImport) and _MassPropertiesOfTriangles (tetrahedra with the origin, the sign
  of the orientation removed); FromSTLfile reads with it, the normals from the triangles (numpy-stl's were not
  normalized), scale of three factors, the verbose volume only with density (it raised a NameError without);
  useNumpySTL=True the old path. Against numpy-stl in venvP312 on an irregular octahedron: 1e-8 relative, the float32
  of numpy-stl. stlFileImportTest.py: a brick written binary and text, scaled, rotated, moved.
- Seen: graphics.Brick is counterclockwise outward (positive volume), and FromSTLfile inverts the STL triangles by
  default; the test writes its STL inverted for it. Open question 11 of the design.
- #3265: ArmConfiguration in C++ and Python; first with W a point of axis 4 for the UR, where the shoulder did not tell
  the two q1 apart (4 of 20 poses): W is the point of the axes 5 and 6 for both 6R geometries, the elbow measured to
  axis 4 for the UR. The 3R has wrist 0, so 0 means any in configuration, as None. analyticInverseKinematicsTest.py
  checks the configurations (8 more checks, 27.2).
- #3266: BuildFromDictionary, buildFromDictionaryWarned and the two commented calls in the test models removed.
- #3267 (RG19.3.3) and #3268 (RG11.4.5) raised, open. Blending: design section 6.1, step RG19.5.2.

<a id="rg19-6-4"></a>
### RG19.6.4 — the Levenberg-Marquardt IK in KinematicChain and Robot (2026-11-05, #3269)
- Decision 6 of the design (2026-11-05): InverseKinematicsNumerical replaced by the LM IK, the class kept for a
  deprecation period, calling it.
- KinematicChain.InverseKinematicsNumerical (C++: the Jacobian of the tool point in the loop, a Gaussian elimination
  of the n x n system) and Robot.InverseKinematicsNumerical (Python); the class InverseKinematicsNumerical deprecated
  (on __init__), Solve and SolveSafe (10 restarts) call the chain, InterpolateHTs kept.
- First with the damping E + lambda; lambda alone converged more often and faster (design section 2.1). Restarts by a
  fixed sequence, the same in C++ and Python, so that both give the same q.
- numericalInverseKinematicsTest.py; systemContainerComputeOnlyTest.py, InverseKinematicsNumericalExample.py and
  serialRobotInverseKinematics.py on the chain (the latter ends at the same joint angles as before).

<a id="rg19-4-1"></a>
### RG19.4.1 — q0, friction and end stops in both models of Robot, the control and the sensors of the redundant model (2026-11-05, #3270)
- Decisions 2 and 3 of the design (2026-11-05): friction and end stops per link by a CoordinateSpringDamperExt on the
  joint coordinate, the node coordinate of the tree and a relative coordinate marker between the bodies of the
  redundant model; Robot(..., enableEndStops=False); the global friction by a function of Robot (SetJointFriction).
- Friction and stops in two Ext objects per joint: one with both is refused (the stops switch it to the bristle
  model). SetEndStops sets stiffness and damping for all links with limits; a limit without a stiffness raises when
  the stops are enabled.
- MarkerBodiesRelativeRotationCoordinate measured between the body frames, which is the joint angle only if preHT and
  localHT turn about the joint axis; localRotation0/1 added (C++), the joint frames in the bodies.
- Found and fixed in CreateRedundantCoordinateMBS: the TorsionalSpringDamper acted about z for 'Rx'/'Ry' joints, the
  first joint ignored the translation of preHT (the TX2-90L failed); the spring-dampers start at q0.
- q0: the initial coordinates of the tree node, the initial pose of the bodies (initialRotationMatrix in the body
  frame), the initial offsets of the PD control.
- PreStepControl and AddSensors act on the model created last; the sensors give one vector over the joints
  (SensorBody of the tree, SensorUserFunction otherwise).
- robotFrictionEndStopsTest.py: the TX2-90L and a robot with a prismatic joint, both models agree to 2e-6.

<a id="rg19-3-3"></a>
### RG19.3.3 — a Robot saved and loaded as a dict and HDF5 file (2026-11-05, #3267)
- Requested by the maintainer (2026-11-05): a Robot, e.g. the TX2-90L built from joint frames with the meshes of its
  URDF, could not be stored.
- Robot.GetDictionary: the parameters (gravity, reference configuration, q0, end stops, friction regularization), base
  and tool (HT, graphics data) and per link all RobotLink data with its visualization; Robot.SetDictionary rebuilds the
  links by AddLink. The data of a created model (SetControl, item numbers) are not included.
- SaveDictToHDF5 stored a list element by element, one dataset per number: a mesh of 1e5 coordinates was not
  practicable. A list of only int or only float values is now one dataset ('numericList'), loaded back as a list.
- The TX2-90L with its URDF meshes: 7 MB, saved in 0.2 s, loaded in 0.02 s, the same dict after the round trip.
- robotDictionaryTest.py (the HDF5 part needs h5py and scipy; the result does not depend on it).

<a id="rg11-4-5"></a>
### RG11.4.5 — the textures as dicts (2026-11-05, #3268)
- Requested by the maintainer (2026-11-05): no image to dict and back, so that neither a Robot nor an mbs dict could
  carry its textures.
- SC.renderer.GetTexture(name): 'name', 'image' as AddTexture takes it (uint8, (height, width) for Grey and (height,
  width, channels) otherwise, the first row at the top - the store holds the rows from the bottom), 'interpretation'
  by its name (a string, so that the dict saves as a file; the enum is accepted back as well) and 'flags' as int.
- 'textures', the list of all textures, in renderer.GetDictionary and SC.GetDictionary; their SetDictionary adds them
  (SC: before the systems, whose graphics name them). The mbs dict has none: the textures are shared by all systems.
- Robot.GetDictionary has the textures its graphics data name, SetDictionary adds them (by the module functions
  _GetTexture and _SetTexture, as the robot has no SystemContainer).
- Seen: SC.GetDictionary calls the deprecated SC.GetRenderState and warns (raised as an issue).
- Found by robotDictionaryTest.py, whose tool graphics with a texture appeared in the next robot test models of the
  suite: Robot(base=RobotBase(), tool=RobotTool()) and the visualization defaults of RobotBase and RobotTool were
  objects shared by all robots; now None, a new object each.

<a id="rg19-5-3"></a>
### RG19.5.3 — circular, screw and spline paths, the arm configuration kept, the jumps in Trajectory (2026-11-05, #3272)
- The request (2026-11-04): continuous paths - linear, screw including circular, spline - in a common way, all mixed
  in Trajectory, jumps detected and raised; design section 6.2: the configuration of #3265 kept on Cartesian paths.
- _ProfileCartesian: the common code of ProfileCartesianLinear (#3262); a profile gives its path as the frame of the
  tool and the derivatives of its position and rotation by the path parameter. Circular: the circle through start, via
  point and end, the rotation interpolated as for the line; screw: the tool frame as a rigid body about an axis, with a
  translation along it; spline: natural cubic splines on the chord length of the positions and of the rotation vectors
  relative to the start (piecewise interpolated rotations, tried first, gave a jump of the angular velocity at the via
  frames: 21 % between the joint velocities and the difference of the coordinates, 4e-6 with the spline).
- keepConfiguration: on a 6R robot with a spherical wrist all 8 configurations reach every pose, so keeping one never
  fails on reach; it fails as a jump - through the singular wrist the kept solution turns q4 and q6 by pi. The path
  is followed in 200 steps when the profile is added, and a step of more than 0.5 rad raises; without
  keepConfiguration the nearest solution passes, changing the configuration.
- Trajectory.Add evaluates a new profile at its start against the end of the trajectory (jumpTolerance = 1e-6).
- robotCartesianPathsTest.py. Open in RG19.5: blending (RG19.5.2), a Cartesian blend.

<a id="rg19-3-4"></a>
### RG19.3.4 — the format version of the Robot dict, GetGlobalToolHT, the TX2-90L saved in its example (2026-11-05, #3273)
- Maintainer 2026-11-05: a version number of the saved Robot, starting with 1, raised by 1 only if the stored format
  changes between two releases (it stays until the next release); the tool frame in global coordinates as
  GetGlobalToolHT; the TX2-90L of staubliTX2trajectories.py stored with the HDF5 functions in Examples/solution.
- robotDictionaryVersion = 1 in roboticsCore.py, 'dictionaryVersion' in the dict; SetDictionary refuses a later
  version and has the place for the conversions of earlier ones.
- GetGlobalToolHT(q) = LinkHT(q)[-1] * tool.HT in Python; the docstring points to the chain (some 100000 ToolHT per
  second, measured by the maintainer).
- The example saves the robot when the URDF data set is found and loads it from the file otherwise.

<a id="rg19-5-2"></a>
### RG19.5.2 — blending of the profiles of Trajectory, the example staubliTX2blending.py (2026-11-05, #3274)
- Maintainer 2026-11-05: the superposition of design section 6.1 agreed; the name blendTimeWithNext (the blending
  at the end of this profile and the start of the next) instead of blendTime; the distance at which blending occurs;
  Evaluate and EvaluateCoordinate with and without blending (enableBlending=True); a demo of the TX2-90L with a
  gripper, an L-shaped path in a horizontal plane with a cup in the middle, PTP and CP profiles mixed, twice (without,
  then with blending), the two paths and the points drawn.
- The profiles keep their times one after the other; with blending, profile k is evaluated at t plus the blend times
  of the profiles before it, and the trajectory is the start plus the displacements of the profiles that have
  started. Any profile works, also the Cartesian ones (their joint coordinates are superposed, so the tool leaves the
  line in the blending).
- GetTimes(enableBlending=True) gives the blended times (the starts of later profiles move); Add uses the unblended
  ones internally. A blend time longer than this or the next profile raises.
- GetBlendings: per blending the times, the via coordinates, the distance at which it starts and the closest
  approach; with a KinematicChain the same for the tool. Two constant-acceleration profiles blended by the
  deceleration time: the axis that moves on keeps its velocity; the TX2 tool starts 0.18 m before the via point and
  passes it at 0.036 m.
- staubliTX2blending.py: 3.7 s for the task without and 2.8 s with blendings of 0.3 s; trajectoryBlendingTest.py.

<a id="rg19-3-5"></a>
### RG19.3.5 — STL files keep their orientation, open meshes oriented by their volume, back faces in the raytracer (2026-11-05, #3275)
- Maintainer 2026-11-05 (image of the TX2-90L in OpenGL): the triangles or normals of some links flipped, links 3, 5
  and 6 right; the normals should point outward; a check during the STL import, e.g. by the volume; the back faces
  visible in the raytracer as in OpenGL; open question 11 checked in the GLFW client.
- OpenGL keeps glFrontFace at GL_CCW (no call in GlfwClient.cpp), the GraphicsData docs say counterclockwise seen from
  outside, graphics.Brick is so, and the 7 STL files of the TX2 have positive signed volumes with their file normals
  agreeing - STL files are outward as Exudyn. FromSTLfile inverted them by default; OrientTriangles turned the closed
  parts (links 3, 5, 6) outward again, the open ones (links 1, 2, base: 6 to 18 open edges) stayed inverted.
- Defaults of FromSTLfile, FromSTLfileASCII, ExportSTL and LoadURDFgraphics: no inversion; LoadURDFgraphics inverts
  the normals with the triangles when asked. OrientTriangles(outward=True): an open part whose signed volume about its
  center is at least 5 % of that of a sphere of its area (the TX2 meshes: 18 % to 79 %) is oriented by its volume;
  the result is now the same whether the file is inverted or not. checkOrientation: the cheap test of the whole mesh.
- raytracer.advanced.backFaceColor (alpha > 0): the hits from the back (hitFromBack) in a flat color. The TX2 with its
  meshes raytraced from below: 11 of 300000 pixels back faces (at edges) with the new defaults, 15201 with the
  triangles inverted and not oriented.
- raytracerBackFaceTest.py; stlFileImportTest.py and test_urdfGraphics.py write the STL outward.

<a id="rg19-4-2"></a>
### RG19.4.2 — the inverse dynamics, the dynamic compensation of the control (2026-11-05, #3276)
- Maintainer 2026-11-05: next steps 2 and 3 of the report (this one the dynamic compensation of RG19.4).
- Decision 4 of the design put the dynamic compensation in the kinematic tree; the chain is used instead, with the
  same data and a new argument inertias: KinematicChain.InverseDynamics by the recursive Newton-Euler algorithm in
  global vectors (the base accelerated by -g), Robot.InverseDynamics the same in Python, 1.4 to 1.8 us in C++.
- Checked against M q_tt + the left-hand side of ObjectKinematicTree (mass matrix of the solver, OutputVariable Force)
  for Rx/Ry/Rz, prismatic joints and the Puma560 to 1e-10 relative; at q_t = q_tt = 0 the negative of StaticTorques.
- SetControl(compensation='dynamic'): the drive of the inverse dynamics with the q, q_t, q_tt of the trajectory, in
  the tree as jointForceVector, in the redundant model as -torque of the spring-dampers. The TX2-90L on a polynomial
  of degree 5 in 0.6 s: tracking error 2.3e-3 rad with 'static', 1.2e-4 rad with 'dynamic', the same in both models.
- robotInverseDynamicsTest.py.

<a id="rg19-7-1"></a>
### RG19.7.1 — configurations, initial configuration and friction of the models (2026-11-05, #3277)
- Maintainer 2026-11-05: next step 3 of the report (RG19.7: models.py with initial configurations and friction).
- From the RTB 1.5 (venvP313rtb, no network): the configs of the DH Puma560 (qr, qz, qn, qs), UR5 (qr, qz) and Panda
  (qr, qz); the friction of the Puma560 (B, Tc, G per link at the motor; at the joint B G^2 and the mean of |Tc| times
  |G|: viscous 5.8, 9.5, 4.0, 0.41, 0.43, 0.22 Nms/rad, Coulomb 26, 10.6, 6.4, 1.07, 0.85, 0.55 Nm); the UR5 and the
  Panda have none there. The forward kinematics agree with the RTB (Puma560: the RTB's base is 0.67183 m higher).
- The TX2-90L: qz upright, qr = [0, 0.5, 1.4, 0, pi - 1.9, 0] with the tool pointing down.
- LinkDict2Robot(initialConfiguration=None, friction=False): the defaults keep the robots of the existing scripts as
  they were (q0 = reference, no friction); 'model' takes the model's initialConfiguration (qr for all four).
- Seen, open: the joint limits of the Puma560 of models.py (Corke, Visual Control) differ from the RTB's; the model
  Manipulator4Rsimple has 3 links.
- robotModelsTest.py.

<a id="rg11-13"></a>
### RG11.13 — the developer documentation in groups, the how-to notes under Misc (2026-11-05, #3278)
- Maintainer 2026-11-05: internals at the end, a group for workflow and release, git under getting started, building
  and running into the build page; the how-to notes of the developer front page under Misc, without duplicates and
  history.
- docs/dev/README.md: one table in the order of the toctree, which has 12 entries instead of 22; new pages
  docs/dev/DEVELOPMENT.md (the repository layout, toctree WORKFLOW, RELEASE, GUI_MANUAL_CHECK) and
  docs/dev/internals/README.md (the eight internals pages); GIT under GETTING_STARTED; the editors, the driver and the
  C++ entry points into BUILD.md.
- docs/howTo/misc.md publishes the five maintainer notes (no longer in exclude_patterns of conf.py, #2585 reversed by
  the maintainer); index.md and pdfIndex.md list it. Removed: the origins of the build quirks, the Anaconda 2020.02
  and MKL notes (history), the CC/CXX entry and the scipy pin (in BUILD.md and condaEnvironments.md), the
  documentation commands of condaEnvironments.md (in sphinxDocs.md); numpy-stl described as the optional reader.

<a id="rg19-8-1"></a>
### RG19.8.1 — the serial robot examples on the revised interfaces; Puma560 limits, Manipulator4Rsimple (2026-11-05, #3279)
- Maintainer 2026-11-05: RG19.8 now; the Puma560 joint limits as in the RTB (more up to date); Manipulator4Rsimple
  removed with the cleanup (outdated, 3 links).
- Rewritten on LinkDict2Robot(ManipulatorPuma560(), robotClass=...), SetControl/PreStepControl, AddSensors, PlotSensor:
  serialRobotKinematicTree.py (both models by a flag; equal to 1e-11 at the end; the default solver, as
  TrapezoidalIndex2 exceeds the drift tolerance with the redundant model), serialRobotTSD.py (redundant model, rotations
  up to 3 pi), serialRobotFlexible.py (control and sensors; result equal to before, 37.2276725), 
  serialRobotKinematicTreeDigging.py (PreStepControl with the time of the cycle), humanRobotInteraction.py (two robots
  in one PreStepUserFunction; equal to before), serialRobotInteractiveLimits.py (the limits as end stops of
  SetEndStops instead of a spring torque user function; not run interactively here), InverseKinematicsNumericalExample.py
  (UR5 of models.py, numerical and analytic IK).
- Kept: serialRobotTest.py and movingGroundRobotTest.py test the deprecated joint torque loads of
  CreateRedundantCoordinateMBS; ROSMobileManipulator.py has a state machine with brakes, not a trajectory.
- Puma560 jointLimits of the RTB 1.5 (160, -45..225, -225..45, -110..170, 100, 266 degrees); robotModelsTest.py checks
  them instead of the removed model (result unchanged).

<a id="rg19-1-2"></a>
### RG19.1.2 — the URDF import against the RTB 1.5 and the local data set (2026-11-05, #3280)
- Exudyn 1.13.390 installed into venvP313rtb (it had 1.13.6); no network: the RTB models that robot_descriptions
  downloads (UR3/5/10, Panda, KinovaGen3, Jaco, Fetch, PR2) were not loaded.
- Found: GetRoboticsToolboxInternalModel and LoadURDFrobot failed with RTB 1.5 (roboticstoolbox.tools.xacro and
  Robot.URDF_read removed; the models are URDFRobot classes reading by URDF_file/xacrodoc; urdf_string empty). Ported:
  the model class, robot.urdf_filepath, URDF_file with the package of the file, the directories next to it and below it
  registered (ANYmal uses package://meshes). GetURDFrobotData: the visual origins from the RTB links (scene.T, local),
  the static frames by fkine(end=link), the UR offset special case removed; links without joint moved into the joint
  link above (preHT composed, parentName the joint link, graphics and mass/COM/inertia added by Steiner).
- Check (scratchpad script; per model: Robot of the linkList against RTB fkine(end=link) at 5 random q, RTB rne
  against Robot.InverseDynamics for serial robots with masses, the inertia, flips, limits, meshes):
  - equal (fk 1e-16): Puma560, AL5D (rne 3e-16), px100/px150/rx150/rx200/vx300/vx300s/wx200/wx250/wx250s (branched by
    the gripper fingers), KUKA kr5_arc (rne 8e-17), Staubli tx2_60, tx2_90l, tx90 (rne 2e-16), tx60l (rne 6e-16),
    ANYmal, Spot; before the fixed links were composed the Interbotix robots were off by 0.6-1.7 m;
  - wrong: flipped joints (LBR link_4; KUKA kr3/kr6/kr10/kr150 links 1, 4, 6; Mars rover 11 joints): not implemented;
  - inertia: m = 0 (Puma560, LBR, KUKA except kr5_arc, TX2, Spot), not positive definite (AL5D).
- serialRobotURDF.py: the Puma560 (no download), Robot.SetControl and one PreStepUserFunction for all robots, the
  invalid lightModelAmbient of 3 values fixed; run with Puma560, LBR, vx300, px150, AL5D.

<a id="rg19-9"></a>
### RG19.9 — the chapter Robotics (2026-11-06, #3281)
- Maintainer 2026-11-05: a top-level chapter between the advanced topics and the symbolic system, the features and
  workflows, an image of a nice model, a notebook of the robot example; ROS integrated, its features said untested.
- docs/manual/robotics.md: the modules, a robot, the kinematics without a model, the two models, trajectories, control
  and sensors with a typical script, the URDF import (with the result of RG19.1.2), ROS, the examples.
- docs/figures/robotStaubliTX2.png: the TX2-90L with its URDF meshes, the gripper, the cup and the two tool paths of
  staubliTX2blending.py, raytraced by ShowImage (1600 x 1200); the modelRotation of ShowImage is the transpose of
  Rx(-1.15) Rz(-2.3) for z up.
- python/Notebooks/tutorialRobotics.ipynb: Puma560 in qn (qr is upright, where a Cartesian line soon leaves the
  workspace), analytic IK (8 solutions), a PTP blended into a Cartesian line, the kinematic tree with
  compensation='dynamic' (largest deviation 5e-5 rad; with friction=True 0.069 rad, as the friction is not
  compensated, so the notebook leaves the friction out), sensors and plots; its page is in the chapter's toctree.
- The ROS section of the advanced topics moved into the chapter; index.md and pdfIndex.md list the chapter.

<a id="rg11-4-6"></a>
### RG11.4.6 — SC.GetDictionary without deprecation warnings (2026-11-06, #3271)
- SC.GetDictionary called PyGetRenderState and SC.SetDictionary PySetRenderState, the deprecated Python functions,
  which warned the user of calls they had not made; both call renderer.GetState/SetState now (waitForRendererFullStartup
  false, as before). rendererDictionaryTest.py checks that a GetDictionary/SetDictionary pair warns of nothing (the
  SetRenderState warning appeared before the fix; result unchanged, 11).

<a id="rg19-3-6"></a>
### RG19.3.6 — flipped joints, inertia check and correction, mass from meshes, textures (2026-11-06, #3282)
- Maintainer 2026-11-05: after RG19.1.2, the RG19.3 items inertia check, textures from .dae/.obj, flipped joints.
- Flipped joint (RTB isflip: the link turns by -q): F = rotation by pi about an axis normal to the joint axis
  (diag(1,-1,-1) for z); preHT' = preHT F, COM and inertia transformed by F, the visuals and the links above which the
  next ones hang prefixed by F; the joint coordinate stays that of the URDF; linkDict 'flipHT' = F, the URDF link frame
  is the Exudyn frame times F. Check (RTB 1.5): LBR, KUKA kr3/kr6/kr10/kr150 and the Mars rover now equal to 1e-16;
  the Mars rover (flips and fixed links, masses): total mass 24.53 and the global COM equal to 1e-16.
- Inertia: _InertiaProblems (mass > 0, symmetric, positive definite, triangle inequality) per link in
  'inertiaProblems', printed with verbose > 0; correctInertia=True raises the principal moments as little as possible,
  a zero tensor first gets the shape of the meshes scaled to the mass (AL5D: four links with zero inertia, all
  physical after it); meshDensity > 0 gives a link without mass that of the solid of its meshes (TX2-90L with 2700:
  47, 36, 21, 17, 1.7, 0.07 kg; open meshes give estimates).
- Textures: a mesh with wedge texture coordinates and a texture file (pymeshlab) gets one point per triangle corner,
  the texture coordinates of the mesh, white color, and its first texture added by _SetTexture under the file name;
  ANYmal raytraced shows its textures, the logo readable (not mirrored, so the coordinates need no flip).
- test_urdfRobotData.py: a stub of the RTB interface (no toolbox needed) with a flipped joint and fixed links; the
  inertia helpers.

<a id="rg19-5-4"></a>
### RG19.5.4 — blending: 'auto', the velocity check, Cartesian space (2026-11-06, #3283)
- Maintainer 2026-11-05: the blending extras suggested at the end of the last report.
- 'auto' (resolved when the next profile is added): the largest of 20 steps below min(acceleration phase of both;
  half the duration for profiles without phases) whose blended velocities (51 samples) stay within the limit per
  coordinate - the larger of the peak velocities of the two profiles and maxVelocities of Trajectory, so that a profile
  that exceeds maxVelocities by itself (ProfileConstantAcceleration ignores them) is not blamed on the blending.
  Two profiles of constant acceleration always give the full phase (the summed velocities are linear in the overlap,
  their maximum at its ends); two quintic profiles of 1 s give 0.45 s.
- A numeric blend time is checked when maxVelocities are given (ValueError with the ratio).
- blendSpace='cartesian' (both profiles Cartesian, checked at the next Add): the Cartesian profiles carry ToolMotion(t)
  (frame, velocity, angular velocity from the path and the time law); in the overlap p = p_k + p_k1 - p_k1(start),
  R = A R_k with A = R_k1 R_k1(start)^T, v = v_k + v_k1, omega = omega_k1 + A omega_k; the joints by the analytic IK
  in the configuration of profile k nearest to the joint superposition, q_t by the Jacobian, q_tt by central
  difference. TX2-90L, two lines at a right angle: in the plane and rotation to 9e-16 (joint blending: 8e-5); the
  velocity and acceleration equal to the difference quotients.
- staubliTX2blending.py blends its line-line corners in Cartesian space; trajectoryBlendingCartesianTest.py.

<a id="rg19-8-2"></a>
### RG19.8.2 — the examples runner gives each example its __file__ (2026-11-06, #3284)
- Maintainer 2026-11-06: a full examples run (suggested at the end of the last report).
- `exudev examples` (parallel, venvExuP313): 2 of 196 failed, staubliTX2blending.py and staubliTX2trajectories.py, with
  NameError `__file__`: they find the local URDF data relative to their file, and the runner executes the source by
  exec without `__file__` - failing in every examples run since #3263, unnoticed as the test suite does not run examples.
- Fix: the worker script of the parallel run sets `__file__` to the absolute path of the example before the exec; the
  serial run sets it in its globals for each example and restores its own afterwards. Parallel run after the fix: all
  196 pass (46 s); the TX2 examples load their meshes and save the HDF5 file.
- The serial run (`--serial`, kept for the debugger): spotModel.py and publications/sliderCrankCMSacme.py fail, as
  they read their data relative to the working directory, which only the parallel run sets to the folder of the example
  (#2757); not changed here. Its examples share the working directory: rendererNOGLFWexample.py wrote
  python/TestModels/images/ (removed).

<a id="rg19-3-7"></a>
### RG19.3.7 — a small URDF robot in the repository, stored as HDF5; UR, Panda, Kinova checked (2026-11-06, #3285)
- Maintainer 2026-11-06: the URDF check of UR, Panda and Kinova with network access; the rest of RG19.3 - robots with
  inertias from their meshes and textures saved as HDF5, a small test URDF in the repository.
- Check (scratchpad urdfCheck.py, RTB 1.5 in venvP313rtb; robot_descriptions cloned example-robot-data, ros2_kortex and
  kinova-ros from GitHub): UR3, UR5, UR10 forward kinematics 1e-16/2e-16, rne 1.3e-16, 8.2e-17; KinovaGen3 fk 2e-16,
  rne 2.1e-16; Jaco (10 joints with the fingers, a tree) fk 1e-16; Panda (bundled with the RTB, not downloaded) fk
  1e-16, without masses. The UR offset of the old import ("no offset for UR5 ?WHY?") is gone: it was a symptom of the
  fixed links that #3280 merges. Panda was wrongly said to be downloaded in the urdfImport docstring, the example
  serialRobotURDF.py and revisions.md; corrected.
- python/TestModels/testData/urdf/testRobot (2.0 kB of meshes): testRobot.urdf with base_link (STL brick),
  joint1 Rz, link1 (2 kg, principal moments 0.0066, 0.00125, 0.0086 violating the triangle inequality, STL brick),
  a fixed link flange (rpy 0.4 about x), joint2 with axis 0 -1 0 (flipped), link2 without mass with arm.obj (a brick
  0.2 x 0.05 x 0.05 with arm.mtl and a 16 x 16 arm.png of three colored areas), a fixed tool0. arm.obj is force-added,
  as .gitignore ignores *.obj (compiler output), as objImportTest.obj.
- testRobot.h5 (56 kB): Robot.GetDictionary of the import with meshDensity=1000, correctInertia=True, PD control and the
  tool frame of tool0 (flipHT times the relative frame of the RTB); written by test_urdfRobotData.py
  (TestRobotFromURDF) with EXUDYN_RECORD_URDF_REFERENCE=1, compared to 1e-12 otherwise; skipped without the toolbox,
  pymeshlab, h5py, scipy. Run in venvP313rtb with the pytest of venvExuP313 appended to sys.path (no pytest there):
  3 passed.
- urdfRobotHDF5Test.py (test suite, needs h5py): the brick mass 0.5 kg, COM (-0.1, 0, 0) in the flipped frame and
  inertia; the corrected moments of link1 (largest kept, sum of the two smaller equal to it); the frames of link2 and
  the tool against the URDF chain computed by hand (rotation times flipHT); the texture image and the 36 corners with
  texture coordinates; held by PD control at q0 for 0.2 s. Result 11.896535862956235.
- urdfImport: the joint axis by ET.kind instead of ET.axis, deprecated since RTB 1.4, which warned in every import;
  the stub of test_urdfRobotData.py likewise.

<a id="rg20-4-2"></a>
### RG20.4.2 — the C++ solvers write at the multiples of the write periods, interpolated within the step (2026-11-06, #3286)
- Maintainer 2026-11-06: continue with the open solver tasks RG20.4, RG20.8, RG20.7.
- `CSolverBase::FinishStep` calls `WriteOutputWithinStep` before the writers of the end of the step: for each multiple
  of `solution.file.writePeriod` and `solution.sensors.writePeriod` (the next ones are `output.lastSolutionWritten`,
  `lastSensorsWritten`, as before) more than 1e-10 before the end of the step, `SetStateWithinStep` sets the current
  state to the interpolation between `startOfStepState` and the end of the step, copied to `outputEndOfStepState`: q by
  the cubic Hermite polynomial of q and q_t, q_t its derivative, q_tt, ODE1, ODE1_t and AE linear, the data coordinates
  of the end; the ODE2 coordinates of nodes of type `LieGroupWithDirectUpdate` linear (their velocities are not the
  derivatives of the coordinates). The writers write and advance their time by the period; the end is restored. Not
  for the static solver, a write period of 0 (every step) or the initial state. Same rule as the Python solvers (#3241).
- Test suite: 4 of 291 changed, all by the output times: compareFullModifiedNewton.py (solution file period 1e-4,
  step 0.02: 20001 rows instead of 101, the summed difference 0.000201 -> 0.0391) and geneticOptimizationTest.py
  (sensor period 2e-3, step 0.01: 0.101 -> 0.225), userConnectorContactTest.py (4e-12) and
  explicitSolversPostNewtonTest.py (8e-7), whose steps are reduced at the contacts. The AVX2 reference of
  explicitSolversPostNewtonTest.py (fast module, not built here) is to be re-recorded at the next fast-module run.
- solverOutputWithinStepTest.py (new): a mass on a spring with the period 0.0137 against 37 and 74 steps - the
  written times are the multiples and the end, the error of the output converges with order 4.03 (RK44) and 1.99
  (generalized-alpha, spectral radius 1); a period equal to the step gives the steps; DOPRI5 with the automatic step:
  the sensor and the solution file (twice the period) at the multiples, error 2.4e-8; a rigid body with a Lie group
  node: the angle at the multiples to 1e-14.
- The descriptions of `solution.file.writePeriod` and `solution.sensors.writePeriod` say how they are written; design
  notes advancedSolvers.md 3.4 and cppSolverStructure.md sections 5 and 8.6 updated.

<a id="rg20-8-2"></a>
### RG20.8.2 — the PI step size controller in ODE23 and DOPRI5, the error scale by absolute values (2026-11-06, #3245)
- Maintainer 2026-11-06: the open solver tasks RG20.4, RG20.8, RG20.7.
- `CSolverExplicitTimeInt` has a `StepSizeController` (src/Solver/StepSizeController.h, the PI controller of
  CSolverImplicitRungeKutta, #3261), initialized in PostInitializeSolverSpecific with stepSizeSafety,
  stepSizeMaxIncrease, minimumStepSize and the maximum step; `Control(err, h, orderMethod)` decides acceptance
  (err <= 1, as before) and the next or the repeated step size, replacing the elementary rule; the warning at the
  minimum step as before. The scale of each error is atol + rtol max(|q_second|, |q_start|) for the positions, the
  velocities and the ODE1 coordinates (#3245: without the absolute values the scale went through zero at
  q = -atol/rtol). The reduction after a failed Newton or a post-Newton step stays the base class path (design
  cppSolverStructure.md 8.4), so RG20.8 is closed with it.
- Test suite: 3 of 292 changed: solverExplicitODE1ODE2test.py 3.377 -> 2.070 (the summed errors of DOPRI5 at the
  tolerances 1 to 1e-6, smaller), explicitLieGroupIntegratorTest.py 0.16164 -> 0.16192, explicitSolversPostNewtonTest.py
  4.5635 -> 4.5577 (DOPRI5 and ODE23 with contact). Their AVX2 references (fast module, not built here) are to be
  re-recorded at the next fast-module run.
- explicitStepSizeControlTest.py (new): a mass on a spring about x = -1 and about x = +1 takes the same steps with
  ODE23 (57, 1 rejected) and DOPRI5 (21, 1 rejected), the end positions mirror each other to 1e-12, the errors 0.013
  and 0.004 at the tolerances 1e-3.
- solver.md (sec-explicitsolver) describes the PI controller; its old text had acceptance and rejection the wrong way
  round.

<a id="rg20-10-5"></a>
### RG20.10.5 - VelocityVerlet checked (2026-11-08, #3377)
- Against RK67 at 64 times the steps, 4 refinements: parabola exact (1e-12), undamped oscillator order 2.00, damped oscillator order 1.00 to 1.02, free rigid body with rotation vector node order 1.00 (the gyroscopic terms depend on the velocity, as in the damped case). The Lie group update of the positions is in the step. Warning and TEST phase mark removed; the texts of the solver page as asked by the maintainer.
    
<a id="rg5-9-11"></a>
### RG5.9.11, RG5.4.4, RG5.8.2 — the near-zero entries of the sparse Newton Jacobian, ObjectFFRF.sparseMassMatrix, no total coordinates (2026-11-12, #3448, #3449, #1910, #2182)

*(Maintainer 2026-11-12: the near-zero values of the sparse matrices get a threshold; the switch of the sparse mass
matrix goes into ObjectFFRF with a fitting name; #2182 closed; RG5.8.2 not done, closed.)*

**RG5.9.11 (#3448)**: the Newton Jacobian at the end of a run of the models of solverBaseline.py (scratch nearZero.py):
of the nonzeros, below 1e-12 times the larger of the row and the column maximum - rigidBodyJoints50 568 of 3324,
rigidBodySpringDampers50 1487 of 4405, contactSpheres 1428 of 2184, FFRFreducedOrder 28 of 170, the beams and the small
models none; absolute magnitudes down to 1e-46. `simulationSettings.linearSolver.dropThreshold` (default 0) leaves out
an entry at most dropThreshold times the smaller of the largest magnitudes of its row and its column -
`GeneralMatrixEigenSparse::DropSmallEntries` after `setFromTriplets` (Eigen `prune`, O(nnz)), set by the solver for
the Newton Jacobian only, not for the mass matrix or other sparse matrices. Measured (scratch dropBench.py, EigenSparse,
best of 3; 0 / 1e-14 / 1e-12 / 1e-10):

| model | Newton | time [s] | Newton iterations | difference of the coordinates |
|---|---|---|---|---|
| rigidBodySpringDampers50 | full | 0.686 / 0.597 / 0.569 / 0.558 | 714 for all | 2e-15 |
| rigidBodyJoints50 | full | 0.213 / 0.260 / 0.248 / 0.253 | 720 for all | 4e-15 |
| contactSpheres | full | 0.153 / 0.157 / 0.148 / 0.147 | 1003 for all | 5e-18 |
| beam3D60 | full | 0.344 / 0.349 / 0.355 / 0.358 | 400 for all | 6e-14 |
| all of them | modified | equal within 3 % (joints50 up to +8 %) | equal | up to 2e-13 |

The factorization of the spring-damper chain falls from 0.266 to 0.150 s, that of the revolute chain rises from 0.136
to 0.163 s with fewer nonzeros: COLAMD orders the thinner pattern worse there. No gain in general, so the default
stays 0 (only exact zeros, as before). Test model linearSolverDropThresholdTest.py (20 bodies, full Newton: the same
Newton iterations, coordinates equal to 2e-15).

**RG5.4.4 (#3449)**: `ObjectFFRF.sparseMassMatrix` (default True) replaces `exu.special.solver.superElementSparseMassMatrix`
of #2183, removed; objectFFRFsparseMassMatrixTest.py sets the parameter (result unchanged).

**RG5.8.2 (#1910)** closed, won't fix; **#2182** closed, won't fix (docs/dev/design/superElementSparse.md).
parameterConversionTestReference.txt recorded again (the two new parameters).

<a id="rg5-8-1"></a>
### RG5.8.1, RG5.1.2, RG5.1.4, RG5.2, RG5.4, RG5.11.3, RG5.9.10, RG5.10 — the solver baseline, the vectors, the fixed-size matrices, the sparse mass matrix of ObjectFFRF (2026-11-12, #3002, #142, #2183, #3445, #3446)

*(Maintainer 2026-11-12: settings belong in simulationSettings or visualizationSettings, exudyn.special for experimental
flags and settings outside the solver routines - written into CLAUDE.md and CODING_STYLE.md as the common way, not as a
hard rule; raytracer.numberOfThreads = 0 for the number of logical processors, as default; RG5.10 halted - Python user
functions are only one of its problems, and fully decoupled systems can use Python's multiprocessing; do RG5.8.1, RG5.1
(with an option to switch back to the present Vector and ResizableVector, by typedefs), an evaluation of RG5.2 (fixed
rows and columns instead of ConstSizeMatrix), RG5.4 with a fallback.)*

**RG5.11.3 (#3445)**: `raytracer.numberOfThreads` minimum 0, default 0 = `std::thread::hardware_concurrency()`.
**RG5.9.10 (#3446)**: CODING_STYLE.md §1 and CLAUDE.md, a section "The common way - not hard rules".

**RG5.8.1, the solver baseline (#3002)**: solverBaseline.py repaired - the first run by the timer (2026-11-10) failed in
480 of 540 cases: an error of the solver since fixed, the drift check that SolveDynamic switches on for
TrapezoidalIndex2 (now `index2CheckConstraints=False`: time, not accuracy, is measured), explicit solvers on models with
constraints (now skipped); the coordinates counted, the summary with the ratios dense / full Newton / h/4 / 4 threads,
the module in the file name. Run 2026-11-12 on both modules: 420 cases each, none failed, 804 s each;
docs/dev/design/solverBaseline.md. The fast module 5-25 % faster; 4 threads slower up to 42 coordinates (1.02-1.70),
faster from about 300 (0.48-0.86); EigenDense 2.6-18 times slower for the chains of 400-650 coordinates; full Newton
1.2-8.6 times; for RG5.8.2: a vector addition of all coordinates per evaluation is 0.06 % of a step of the cheapest
case (contactSpheres with RK44).

**RG5.1.2 and RG5.1.4, not done (#142)**: the allocations per step of the ten performance models
(`exudyn.special.InfoStat` around each SolveDynamic, linalgBenchmark.md §3.10): vectors 0.00-0.48 per step, all in the
initialization of a solve; so merging ResizableVector into Vector has nothing to save, and no switch was built.
`operator[]` is inline (defined in the class); its cost is the range check of the regular module, which the pointer
loops of L1-L3 avoid in the kernels. generalContactSpheresPerf allocates about 100 arrays per step (GeneralContact).

**RG5.2, evaluated**: docs/dev/design/linalgFixedSize.md (an agent's read-only analysis, checked): ConstSizeMatrix fixes
only the capacity; after L1-L6 the 3x3 products are as fast as a fixed-size prototype; the gaps of compile-time sizes:
6x6 products 2.2 times, 6x6 times vector 1.7, J^T K J for 3 x 6 1.7, short ConstSizeVector operations 3-4; Eigen's fixed
sizes are slower than the prototype with MSVC. Recommended: profile ODE2RHS of the rigid-body chains first, then
`SlimMatrix<T, rows, columns>`, Matrix6D first, Matrix3D behind a compile switch.

**RG5.4, first part (#2183)**: docs/dev/design/superElementSparse.md (analysis by an agent, checked): the system already
takes sparse element matrices; #2182 gains nothing (the reduced matrices are already triplets, the element matrix is
small); the cost is in ObjectFFRF's dense mass matrix and numerical Jacobian, and in the markers on reduced-order bodies
(#2184); the Python side transports triplets already, no change of the initialization needed. Done:
`MatrixContainer::AppendSubmatrixAsTriplets`, `AppendTripletsWithOffset`; ObjectFFRF's mass matrix as triplets with
`exu.special.solver.superElementSparseMassMatrix` (default True, experimental) as the fallback; the dense path without
the copy of Mff; `ComputeItem(..., MassMatrix)` for sparse mass matrices. NGsolve block, 20 steps: mass matrix 8-20
times faster (1525 nodes: 1.571 -> 0.080 s), total 6-13 % (24.07 -> 22.53 s), coordinates equal to 1e-16. Test model
objectFFRFsparseMassMatrixTest.py.

<a id="rg5-9-9"></a>
### RG5.9.9, RG6.25, RG5.11.2 — the substitution by rows, hideMbs, the scaling of the raytracer (2026-11-11, #3441, #3442, #3439)

*(Maintainer 2026-11-11: "For RG5.10, I like to see a more detailed concept here in the chat, before you continue.
Write that concept first, then do the suggested steps 1. - 3." The concept of RG5.10 was given in the chat and waits
for the decisions on the name, the behaviour after an error and the file names; RG5.10 is not continued.)*

**RG5.9.9, the substitution by rows (#3441)**:
- `SparseLURows<Ordering>` in ParallelSubstitution.h/.cpp (explicitly instantiated for COLAMD and the nested
  dissection): `factorize` invalidates the rows, `ExtractRows()` copies L and U into rows, `SolveRows` substitutes
  serially; `SparseLUParallel` derives from it. `GeneralMatrixEigenSparse::solver` is a `SparseLURows<COLAMDOrdering>`,
  the rows are extracted after each successful factorization if `exu.special.solver.sparseSolveByRows` (default True).
- Measured (scratch rowsStudy.py, each model in its own process, the faster of two runs): the 5 performance models with
  EigenSparse 22.21 -> 21.68 s, perf3DRigidBodies 1.401 -> 1.146 s (-18 %), perfConnectorInterface -5 %; 52 test
  models 21.23 -> 20.64 s, gridGeomExactBeam2D -11 %, the contact models -6 %; +4 to +9 % only for models of a few
  milliseconds. The study harness failed for ACFtest.py and linearFEMgenericODE2.py (working directory, `__file__`)
  with both settings; sphereTriangleTest.py diverged with Eigen's solve in that run - it is excluded from the suite as
  unstable (it changes with round-off).

**RG6.25, view[0-3].scene.hideMbs (#3442)**, as recommended in the step: `SystemHiddenInView(viewID, system, settings)`
in VisualizationSystemContainerBase.h, used in the draw loop of the renderer (faces, edges, lines, points, texts,
shadows and the picking follow), in the zoom of a view (`GraphicsData::ComputeMaxScene` with the hidden systems,
`ComputeZoomToItem`), the sensor traces (`SensorTraceLines` with the view) and the raytracer of the view.
viewHideMbsTest.py (new): raytracer images with hideMbs = [1], [0, 1], [] and [5] against containers holding only the
shown systems.

**RG5.11.2, the scaling of the raytracer (#3439)** (scratch rtScale.py: the scene of raytracerNOGLFWtest.py at
1200 x 1000, the fastest of three images, 16 cores / 32 logical processors):

| threads | load balancing, 12 tiles/thread | without, 12 tiles/thread | load balancing, 48 tiles/thread |
|---:|---:|---:|---:|
| 1 | 4.660 | 4.667 | 4.631 |
| 2 | 2.382 | 2.422 | 2.337 |
| 4 | 1.245 | 1.467 | 1.222 |
| 8 | 0.664 | 0.820 | 0.657 |
| 16 | 0.437 | 0.585 | 0.419 |
| 32 | 0.372 | 0.471 | 0.359 |
| 64 | 0.368 | 0.421 | 0.358 |
| 128 | 0.366 | 0.497 | 0.364 |

- With load balancing the raytracer scales to the logical processors (12.6 times on 32) and gains nothing beyond.
  Without it, each thread renders a fixed band of tiles; the best time is then at 64 threads, where the oversubscription
  balances a little - the maintainer's observation. The raytracer took the flag from
  `exudyn.special.solver.multiThreadingLoadBalancing`, which every solver sets from its `parallel.useLoadBalancing`.
- Changed: `TaskManager::AcquireWorkers` takes a load balancing mode (1/0, or -1 for the flag); the raytracer always
  balances: afterwards 0.369 s on 32 threads with the flag off as well (4: 1.262, 8: 0.685, 16: 0.444, 64: 0.367 s).
- For Exudyn in general: a `ParallelFor` without load balancing splits statically and suffers from unequal work per
  item. The default `raytracer.numberOfThreads = 8` gives 0.664 s here against 0.37 s with 32 - a candidate for a
  default from the number of processors.

<a id="rg5-9-7"></a>
### RG5.9.7 — the parallel substitution with a nested-dissection ordering, a prototype (2026-11-11, #3432, #3441)

*(Maintainer 2026-11-11: "RG5.9.7, the parallel back-substitution: => try that".)*

Built as `exu.special.solver.parallelSubstitution` (experimental, default False), with
`parallelSubstitutionPivotThreshold` (1e-3) and the counter `parallelSubstitutionSerial`; details and the table in
sparseSolvers.md section 4.1:
- `src/Linalg/ParallelSubstitution.h/.cpp` (new; vcxproj, filters, sources.json by gen_sources.py): `NestedDissection`
  (breadth-first level separators, components split into halves, a depth of about two subtrees per thread),
  `NestedDissectionOrdering` (the ordering functor of Eigen), `SparseLUParallel` (Eigen's `SparseLU` with it: the
  factors copied into rows after each factorization, the tree checked, the substitution depth by depth in a
  `ParallelFor`). `GeneralMatrixEigenSparse::FactorizeNew` and `Solve` use it if the flag is set; a failed
  factorization falls back to SparseLU.
- On the way, from scratch studies (ndTree.py, ndLift.py: SuperLU with partial pivoting):
  - the bound of 18-60 of section 4 assumed no row pivoting; with Eigen's partial pivoting the subtrees were coupled
    (thousands of crossing dependencies). Unknowns with a zero diagonal (multipliers, and the first Euler parameter at
    the reference rotation) are lifted into the highest tree node of their neighbours with a nonzero diagonal and
    ordered last there, and the diagonal pivot threshold is 1e-3: no crossing left on the chains, fill 16 % above
    COLAMD for the revolute chain. A first version lifted along lifted neighbours and carried the Euler parameters of
    half a chain into the root (3520 unknowns there instead of 21).
  - the pattern of the Jacobian changes, as Exudyn drops exact zeros: an ordering of the pattern seen was crossed by
    new entries in 77 of 80 factorizations - also with the union of the patterns and on the squared graph; keeping
    the zeros in the assembly gave a constant pattern but a factorization 1.8 times slower. The solver now passes the
    coordinates of each object (`GeneralMatrix::SetStructuralGroups`, from the local-to-global indices of ODE2, ODE1
    and AE) and the ordering works on the pattern joined with their cliques: one ordering per simulation; when the
    pattern changes only Eigen's elimination tree is renewed.
- Measured (scratch psubBench.py, the fastest of three runs): the solves are 1.8-4.4 times faster than with SparseLU
  on 8 threads, but 1.6-2.8 of that comes from the rows alone (one thread; Eigen's supernodal solve is slow for small
  supernodes) and only 0.9-1.7 from the threads (a solve takes 10-60 microseconds); the factorization is as fast as
  with COLAMD on one thread and slower on 8 threads (up to 70 % for 100 bodies, 7-16 % for 1000). In total, on 8
  threads: modified Newton on chains of 1000 and 10000 bodies 9-13 % faster, full Newton on 10000 mass points 14 %;
  100 bodies and full Newton on 1000 bodies slower. Experimental, off.
- parallelSubstitutionTest.py (new): revolute chain and mass point chain, modified and full Newton, 1 and 4 threads,
  against EigenSparse, no serial fallback; performanceErrors.md, revisions.md.
- The row solve alone is the larger and thread-independent gain: proposed as RG5.9.9 (#3441), the serial
  substitution by the rows of the factors for every SparseLU factorization, to be measured before it becomes the
  default.

<a id="rg5-9-8"></a>
### RG5.9.8, RG5.10.1, RG5.11 — persistent threads, what the systems of a SystemContainer share, the raytracer steps (2026-11-11, #3437, #3438, #3439, #3440)

*(Maintainer 2026-11-11, on the next steps: no special tricks for PARDISO - the option is valuable as it is, also where
it is slower, and it is expected to pay for very large systems such as the coming 4- and 10-node tetrahedra (not a
step); RG5.9.7 to be tried; the RG5.10 pre-analysis now; instead of sleeping workers by default, a function of
exudyn.special.solver that starts a number of workers that run permanently, for many runs of small systems, with the
solvers still taking the threads of simulationSettings, which wins with a message where the two disagree - "do that if
everything is clear as suggested, or propose a better idea"; a step for the raytracer, which should run in parallel to
the solver from the renderer thread, and one for its strange scaling - best with 64-128 threads on 16 cores.)*

**RG5.9.8, persistent threads (#3437)**, as suggested, with one adjustment: the persistent workers sleep while no
computation uses them (the condition variable of RG5.9.3), as spinning workers would occupy the cores between the
solves, while Python runs:
- `TaskManager` (`src/Utilities/Threading.h/.cpp`): a parked task manager - detached from `task_manager`, so that
  `GetNumThreads()` is 1 and code outside a computation (mbs functions called from Python) runs serially; the static
  `isRunning` stays true for its workers. `SetPersistentWorkers(n)`, `PersistentWorkers()`,
  `AcquireWorkers(n, takeOver, message, ticket)` and `ReleaseWorkers(acquired, ticket)`, all under one mutex.
  Acquire: the persistent workers if their number agrees; otherwise they are stopped and a task manager of its own runs
  (`message` says so), or the computation is serial; with `takeOver` (the solvers) threads left running by another
  computation are stopped, as before. Release: the persistent workers are parked again, threads of its own stop, and the
  persistent ones start again; a ticket makes sure that a solver destroyed late (#2237) does not give back threads
  another computation has taken over.
- The solvers (`CSolverBase::InitializeSolverData`, `StopThreadsAndCloseFiles`), the raytracer and
  `RunLinalgBenchmark` take and give back their threads through it; before, each called `SetNumThreads`,
  `EnterTaskManager` and `ExitTaskManager`, and two task managers would have shared the static state.
- `exu.special.solver.StartPersistentThreads(n)` (n >= 2; ValueError otherwise), `StopPersistentThreads()`,
  `PersistentThreads()`; an `atexit` hook stops them, so that no worker waits on a condition variable while Python
  ends. A solver with another `parallel.numberOfThreads` (also 1) warns: "numberOfThreads = 4 differs from the 8
  persistent threads ...; this computation uses 4 thread(s), and the persistent threads start again after it".
- Measured (a chain of mass points, EigenSparse, per solve, scratch persist2.py): 200 bodies, 1 step - 0.44 / 0.63 /
  1.10 ms on 4 / 8 / 16 threads of its own, 0.26 / 0.24 / 0.30 ms persistent; 20 steps 2.03 / 2.11 / 2.49 against
  1.82 / 1.30 / 1.67 ms; 2000 bodies, 20 steps 26.1 / 18.2 / 14.8 against 25.0 / 17.6 / 13.7 ms - the start and stop of
  the threads cost 0.2-0.8 ms per solve. The results equal those of one thread.
- persistentThreadsTest.py (new); performanceErrors.md, revisions.md, sparseSolvers.md section 5.

**RG5.10.1, the pre-analysis (#3440)**: docs/dev/design/systemContainerParallel.md, from a read of the code. The
decisive fact: the solve never releases the GIL (`SolveSystem` has no `gil_scoped_release`; the only release is in
`MainRenderer::GetGraphicsData`), so solves on Python threads run one after another today. 13 kinds of shared state
to be made per solver or locked (the one task manager and its global thread count, thread id 0 meaning the main
thread for the symbolic recordings, the suspension of workers, the `pySpecial` counters, the global timers, a
`static` matrix in `MatrixBase::Invert`, `pout`/`outputBuffer`, `PyError`/`PyWarning` with the global error flag, the
Python user functions, the idle operations with the renderer queues and `PyCheckSignals`, the default file names, the
settings object that `SolveDynamic` changes); safe: the per-system render lock, per-solver data, `thread_local` scratch
data, PARDISO (serialized). The natural start: one thread per system (numberOfThreads = 1 in each), thread-local
thread counts and ids. RG5.10 stays open for the design.

**RG5.11 (approved by the maintainer)**, with RG5.11.1 (#3438, the raytracer with threads of its own while a solver
runs, from the renderer thread) and RG5.11.2 (#3439, its scaling). A first candidate for the scaling, seen while
reading the code: without load balancing a `ParallelFor` runs one task per thread with a static split, and the
raytracer takes `multiThreadingLoadBalancing` from the last solver; more threads then mean finer tiles and a better
balance.

<a id="rg5-9-2"></a>
### RG5.9.2-RG5.9.6 — the PARDISO study, sleeping workers, LinearSolverType.PardisoMKL, reuseAnalyzedPattern measured (2026-11-11, #3428, #3430, #3431, #3434)

*(Maintainer 2026-11-11: the sleeping workers cost something for very small thread tasks, and how long they sleep must
be decided, as in NGsolve's TaskManager - at least a setting to switch them on and off, possibly in exudyn.special for
such linear algebra settings during testing; with a constant step size the Jacobian hardly changes, so the
back-substitutions are many and a parallel one makes sense; RG5.9.5 closed, as realistic models (a ropeway, a complex
machine) have no decoupled components - a user who has them splits them into systems of one SystemContainer, whose
parallel solution is a step of its own; "Do the other RG5.9.2, RG5.9.3 and RG5.9.4 now".)*

**RG5.9.2, the study (#3430)** - details in sparseSolvers.md section 3:
- 18 Jacobians exported after two steps (ODE chains and a grid; index-3 chains of 10-1000 bodies on revolute, spherical
  and generic joints, 1000 pendulums, 30 four-bar loops; four of them at index 2), 14 settings, every PARDISO call in
  its own process: PARDISO crashed with an access violation in several settings on the chains of 1000 bodies.
- The cause is the setting, not the matrix: METIS (the default ordering) with the parallel or the two-level
  factorization is wrong or fails on the larger constrained chains (`iparm[23] = 10`: error -15, NaN or the crash at
  1000 bodies; the default on 8 threads: 12 of 18; `iparm[23] = 1` on 8 threads: wrong at 100 bodies); minimum degree
  with the two-level factorization on 8 threads gave $10^{27}$ even for the spring chain. **Minimum degree with the
  one-level factorization solved all 18 on 1, 8 and 16 threads** to SuperLU's residual (the chain of 1000 bodies
  $9\cdot10^{-8}$, condition $2\cdot10^{11}$).
- Refuted: the format (the compressed columns as the rows of the transposed, `iparm[11] = 2`, give the same residuals),
  the numerical noise (dropping it changed nothing that failed), the zero diagonal of the multipliers (matching is on
  by default; index 2 behaves alike). Without matching and scaling 13 of 18. Pivot perturbation and refinement changed
  nothing.
- The ordering also decides the back-substitution: nested dissection gives chains 40-100 levels instead of 1000-4500,
  speedup bounds 18-60 at twice the fill (sparseSolvers.md section 4); after the maintainer's remark this is RG5.9.7
  (#3432), open.

**RG5.9.3, sleeping workers (#3428)**:
- `TaskManager::SuspendWorkers()` / `ResumeWorkers()` (empty stubs since 2022) built with a flag and a condition
  variable: a worker in its spin loop waits on it; `CreateJob` resumes first, `StopWorkers` wakes them to end.
  `FactorizeNew` of the sparse matrix suspends them through an RAII guard if
  `exu.special.solver.suspendWorkersDuringFactorization` (default False) and the matrix has at least
  `suspendWorkersMinimumSize` unknowns (default 1000); the `#define LinearSolverSuspendWorkers false` and its commented
  calls removed.
- 8 threads, EigenSparse, 100 steps, full Newton: spring chain 2.14 s (CPU 17 s) -> 1.89 s (6.9 s), revolute chain 1000
  1.52 s (12 s) -> 1.31 s (4.3 s), revolute chain 100 0.133 -> 0.114 s; modified Newton within the scatter. Off by
  default, as asked.

**RG5.9.4, `LinearSolverType.PardisoMKL` (#3431)** - sparseSolvers.md section 2:
- `src/Linalg/PardisoMKL.h/.cpp` (new, in the vcxproj, sources.json 'all' and 'minimal'), `GeneralMatrixEigenSparse`
  with `UsePardiso`, `FactorizePardiso`, `PardisoFallBack` and the solve with the residual check; the solver type
  accepted in `InitializeSolverPreChecks` (a `ValueError` with the installation hint if MKL is missing) and
  `InitializeSolverData`; settings and `PardisoInfo()` in `exu.special.solver`; setup.py links `-ldl` on Linux.
- Three bugs found on the way, each one a wrong solution with no error:
  - a pattern compared exactly made PARDISO analyze again in 66 of 100 steps (Exudyn drops exact zeros, which come and
    go): the analysis keeps the union of the patterns seen;
  - phases 11, 22 and -1 got pointers to one `double` as b and x, which PARDISO may use as vectors: arrays of size n;
  - **PARDISO of MKL 2025.0 on Windows changed values the caller kept in registers**: the Newton iteration lost its
    initial residual (`relTol = -1.6e+38`, then `inf`), 75 instead of 40 iterations on a chain of 10 mass points, 1103
    instead of 198 on 1000 rigid bodies, while every solve checked to $10^{-16}$; not the integer arguments (guards), not
    MXCSR or the x87 control word, also with `MKL_THREADING_LAYER=SEQUENTIAL`, not with TBB (other threads). Every MKL
    call now runs on a persistent thread of its own (`PardisoLibrary::RunOnThread`), and the results equal EigenSparse
    to $10^{-13}$.
- Measured (sparseSolvers.md section 2.4): PARDISO pays only for full Newton on large systems with several threads -
  1000 steps of the spring chain of 1000 bodies 15.3 s against 18.5 s (EigenSparse, 8 threads, sleeping workers) and
  36.9 s (1 thread), the revolute chain 16.8 against 18.6 and 21.7 s; with modified Newton it is slower, as its solve
  is 2-4 times slower than Eigen's. The type stays experimental.
- pardisoSolverTest.py (new): spring and revolute chain against EigenSparse, the forced fallback, `pardisoFallback =
  False`, the error without MKL, the sleeping workers on 4 threads; the same test result with and without MKL.
- docs/manual/performanceErrors.md (both switches), revisions.md, linalgBenchmark.md (a pointer to the result).

**RG5.9.5** closed, not done (maintainer, above); RG5.10 (#3433) solves the systems of one SystemContainer in parallel,
open.

**RG5.9.6, `reuseAnalyzedPattern` (#3434)**, measured as the maintainer asked (each model in its own process with
`exu.SimulationSettings` patched, the faster of two runs; scratch reuseStudy.py): the 5 performance models with sparse
solvers 23.12 s with and without (results equal to 2e-12), the 53 test models with EigenSparse 20.73 -> 20.40 s
(-1.6 %), at most -13 % (PARTS_ATEs_moving.py), -8 to -11 % for the geometrically exact beams and the 2D mechanisms.
As expected by the maintainer, too little for a default; the flag stays False.

<a id="rg6-24-5"></a>
### RG6.24.5 — the reference manual page of exudyn.machines (2026-11-11, #3429)

- Maintainer 2026-11-11: "exudyn.machines has no generated documentation page => please add one".
- `machines.py` added to `filesParsed` of tools/generators/utilityDocsModel.py; the page
  docs/generated/pythonUtilities/machines.md has the involute functions, MetricThreadSizes, MetricThread, the ball
  bearing functions and the class InvoluteGear, with the examples and test models that use them; it is listed in
  utilitiesIndex.md.
- The three docstrings of the involute had `\\tan` in a raw string, which reached the page as a line break of LaTeX
  followed by "tan"; now one backslash. The module header names what it has (gears, bearings, threads).
- Still not parsed, unchanged here: cMath.py, cUtilities.py, itemInterface.py and lieGroupIntegration.py (commented
  out in the list).

<a id="rg6-24"></a>
### RG6.24 — Screw threads, hexagon prisms, bolts and nuts in exudyn.graphics (2026-11-11, #3425)

#3425 (maintainer 2026-11-11: a visualization of screws for the helical joint, similar to `SolidOfRevolution`: single- and multi-start threads of a given pitch as a function in `exudyn.graphics`; the thread profile as a line segment normalized to the pitch (and the height), with the basic profiles 'triangle' (simplified metric), 'metric' and 'trapezoid' by name; the thread running out over a length at both ends into a cylinder, with lists of shaft diameters before and after the thread as in `SolidOfRevolution`, so that a taper or the shaft itself is drawn as well; internal threads with an outer cylinder of a given radius; hexagonal heads, simplified as an extruded hexagon. Recommendation and questions first; the implementation after the maintainer checked them).

**Recommendation (to be decided before the implementation):**
- **Two functions**: `graphics.ScrewThread(...)` for external and internal threads, and `graphics.HexagonPrism(...)` for heads and nuts. A screw is the thread plus a hexagon prism; the example of the step shows a bolt with nut on the helical joint. A `Bolt`/`Nut` convenience function is left out until it is needed.
- **The surface is one radius function** $r(\varphi, s)$ on a helical grid: with the lead $L = n_{starts} P$, a point at angle $\varphi$ and helical coordinate $s$ lies at $z = s + L\varphi/(2\pi)$ and radius $r = r_{root} \pm h\,f(u)$, $u = (s \bmod P)/P$, where $f$ is the normalized profile (+ external, - internal). The grid lines of constant $s$ are the breakpoints of the profile, so crests, roots and flanks are exact (sharp edges by doubled points with their own normals), multi-start comes from $L = n_{starts}P$ without extra code, and left-hand threads from the sign of $L$. The mesh is closed and made of plain triangles (`FromPointsAndTrigs`), $n_{Tiles}$ around times the points of the profile per pitch.
- **The profile**: a list (or numpy array) of points $[u, v]$, $u \in [0,1]$ along one pitch, $v \in [0,1]$ from the root (0) to the crest (1), piecewise linear and periodic ($v(0) = v(1)$); a vertical flank is two points of the same $u$, an overhanging flank (decreasing $u$) is refused. The height $h$ is absolute, by default from the pitch for the named profiles, so the flank angle follows from $h$ and $P$:
  - `'triangle'`: the sharp 60 degree V, $h = 0.866P$;
  - `'metric'`: ISO 68-1 with the flat crest $P/8$ and the root $P/4$ (rounded root drawn flat), $h = 0.6134P$ external, $0.5413P$ internal;
  - `'trapezoid'`: ISO 2904 (Tr), 30 degrees, $h = 0.5P$ (+ clearance), crest and root flats $0.366P$;
  - possibly `'square'` and `'buttress'` (ISO 3030, 3/30 degrees) as they cost one line each.
- **Run-out, chamfers and shafts by an envelope**: the thread radius is clamped between `rMin(z)` and `rMax(z)`, two piecewise linear contours given as lists of `[z, r]` (as the contour of `SolidOfRevolution`). A machined run-out is `rMin` rising from the root to the major diameter over `runOutLength`; a chamfer at the tip is `rMax` falling. `contourBefore` and `contourAfter` (lists of `[z, r]`, z relative to the start and the end of the thread) are drawn as solids of revolution and meet the thread seamlessly, because the envelope ends at their radius. A plain `runOutLength=[l0, l1]` builds the envelope for the common case, so the lists are only needed for a shaft or a taper. The clamped edges do not lie on grid lines, so a chamfer is faceted at coarse `nTiles` - acceptable for the visualization.
- **Internal thread**: the same surface with the profile turned over (crest at the minor diameter) and the normals inward; `outerDiameter` gives the outer cylinder, the end faces are rings between the thread and the outer cylinder.
- **HexagonPrism**: `pAxis, vAxis, widthAcrossFlats, height`, optional `boreDiameter` (a nut, by `SolidExtrusion` with an inner circle) and a 30 degree chamfer of the edges as an option later (the intersection with a cone, by the same clamp).
- **Parameters of ScrewThread**: `pAxis, vAxis, length, majorDiameter, pitch, profile='metric', height=None, starts=1, rightHanded=True, internal=False, outerDiameter=None, runOutLength=[0,0], contourBefore=None, contourAfter=None, rMin=None, rMax=None, color, nTiles=32, alternatingColor=None, addEdges=False`. Size: an M10 thread of 50 mm with 32 tiles and 8 profile points is about 17000 quads, 34000 triangles.
- **Test model** with the bounding boxes, the number of triangles and a fingerprint of the points (graphics regression), the closedness of the mesh, and the helical joint drawn with a bolt and a nut whose rotation matches the pitch.

**Questions:**
1. Two functions `ScrewThread` and `HexagonPrism`, or also `Bolt`/`Nut` that assemble head, shaft and thread?
2. The height of the profile absolute (default from the pitch per named profile), or relative to the pitch?
3. The envelope by `rMin`/`rMax` contours (general) plus `runOutLength` (simple) - or only the simple parameters?
4. A small table of ISO coarse threads (`'M10'` -> 10 mm, pitch 1.5 mm), or always diameter and pitch?
5. Should `ObjectJointHelicalZ` draw a thread itself (C++, `VObjectJointHelicalZ`, with the pitch of the joint), or does the thread stay graphics data of the bodies, the joint drawing its axis and helix as now?

Sub-steps when decided: (1) `ScrewThread` with the named profiles and the run-out, external and internal, (2) the envelope and the contours before and after, (3) `HexagonPrism`, (4) the test model and an example with the helical joint, the graphics regression references.

**Decisions (maintainer 2026-11-11):** (1) ScrewThread, Bolt and Nut - ideal for all use cases; a Bolt with shaft length zero draws only the head, the HexagonPrism; (2) the profile depends on the base diameter: simplest is to start a screw at the outer diameter and let the profile go inside from 0 to 1 (or the other way round); if the base is neither the outer nor the inner diameter, absolute profiles are simpler to understand - the recommended way left to Claude; (3) the typical envelope at the end of a screw, a chamfer on the thread, would be doable - otherwise the suggestion, but only one way; (4) the ISO table from M2 to about M60, the data and the handling function in exudyn.machines, split from the graphics; (5) as now: the bodies draw the ScrewThread, the joint its axis and helix.

**Resulting design:** the profile is measured from the base diameter inward for an external thread (the major diameter, v = 0 the crest, v = 1 the root at the depth h) and outward for an internal one (the minor diameter); h absolute with a default per named profile. One way for the ends: run-out and chamfer lengths at both ends (parameters), built internally as the clamp of the thread radius; the shaft before and after as contours. exudyn.machines.MetricThread('M10') gives the diameter, pitch and the hexagon sizes (ISO 261/262, 4014, 4032) in metres.

**Implementation** (RG6.24.1-RG6.24.4):

- `python/exudyn/graphics.py`: `ScrewThread` - the profile as the bolt's surface, the depth measured from the major
  diameter (v = 0) to the root (v = 1), an internal thread the same surface with the material outside, so that a nut
  of the same nominal diameter fits (`Nut` adds a clearance of 0.02 pitch against coinciding surfaces); the surface on
  a grid of the angle and the helical coordinate s, $z = s + \text{lead}\,\varphi/(2\pi)$, one strip of points per
  straight segment of the profile, so the corners stay sharp; the depth bounded by chamfers, run-outs and cut ends as a
  clamp, z clamped to the thread, so that the strips beyond an end collapse onto the end circle; normals by central
  differences of the surface. End faces as discs (external) or rings to `outerDiameter` (internal). `HexagonPrism`,
  `Bolt` (head by ISO 4014, shank, thread with a run-out of two pitches into the shank and a chamfer of the depth at
  the tip), `Nut` (countersinks of the depth); the helpers `_ThreadProfile`, `_Annulus`, `_CylinderMantle`,
  `_MeshToGraphics`. A cut end without a chamfer shows a short ramp of the profile on the face (documented).
- `python/exudyn/machines.py`: `MetricThread`, `MetricThreadSizes`, the table of 26 sizes.
- Checked by rendering with the raytracer (scratch screwImage.py): bolt and nut, metric, trapezoid with two starts,
  square left-handed, internal with an outer cylinder.
- `python/TestModels/screwGraphicsTest.py` (the name checked free), reference 0.9068896323640016; the nut on the helical
  joint differs from the analytic solution by 4.6e-5, 1.2e-5, 2.9e-6 at 1000, 2000, 4000 steps (second order, first
  run with 1e-8 as tolerance). `docs/manual/revisions.md`.

<a id="rg5-9-1"></a>
### RG5.9.1 — MKL PARDISO at run time, the threads of the factorization, the back-substitution: design (2026-11-11, #3427)
*(Maintainer 2026-11-11: MKL installed e.g. by pip and available to the C++ of Exudyn through its API, not in the wheel,
considered for the performance gains; the design with the interfaces and settings described clearly; the problems of
PARDISO - possibly the triplet format, or other causes - determined in a planned study, resolvable at least for some
classes of problems; the solver checked at least at one of the first steps; a separate issue for the threads, which
cannot be passed to the solvers - Exudyn's threads idle or stopped for a multithreaded factorization, ideally the solver
takes the existing threads; whether the serial back-substitution, done much more often than the factorization, can be
done by hand at least for Eigen. Start with the design, evaluation and recommendation step and an issue.)*

- `docs/dev/design/sparseSolvers.md` (new): §1 the shares of factorization and back-substitution with modified and full
  Newton (scratch solveShare.py), the numerical noise in the Jacobians (46 % / 67 % of the stored entries below
  1e-14 of their row maximum) and the condition numbers (scratch cond.py); §2 the run-time loading of `mkl_rt`, the
  solver type `PardisoMKL`, the settings, the matrix passed as the CSR of the transpose, the residual check and the
  fallback; §3 the study H1-H6 - Ruiz equilibration tried on the chain of 1000 bodies, no improvement (the entries
  span 1e-48 to 2), condition 2e11; §4 the level sets of the triangular factors (scratch levels.py): chains about 3
  unknowns per level, a 32 x 32 grid 16 per level over 190 levels, independent pendulums 3250 - no level-scheduled
  solve; §5 the TaskManager's spinning workers, the options sleep / stop / components / MKL's threads; §6 the steps.
- Plan: RG5.9 with RG5.9.1-RG5.9.6; #3428 raised for the threads (RG5.9.3, RG5.9.5); `linalgBenchmark.md` §3.9 links
  the note.
- RG6.24: the maintainer's decisions on the screw graphics and the resulting design; sub-steps RG6.24.1-RG6.24.4.

<a id="rg5-1-12"></a>
### RG5.1.12 — Eigen 5.0.1 and MKL PARDISO from Python, evaluated (2026-11-11, #3426)
*(Maintainer 2026-11-11: the current eigen-5.0.1.zip is in tmp - try whether it helps; check the bugs mentioned on the
GitLab page as far as they are relevant; consider MKL for the factorization directly through Python, not through the
C++ - does this work; evaluate first and add the results to linalgBenchmark.md before changing Exudyn.)*

- Eigen 5.0.1: a scratch worktree of HEAD with `include/Eigen` replaced, built with `exudev build --fast --no-install`,
  the wheel in a scratch venv (`--system-site-packages`, `pip install --ignore-installed`; without it pip kept the
  installed 1.13.559 of the base environment - first run). Builds without a source change and without a warning from
  Eigen. `RunLinalgBenchmark` in both modules and versions (scratch benchRun.py, eigenCmp.py), the factorization of the
  models of RG5.1.11 (scratch mklEval.py), the test suite in both modules: 10 of 333 test models differ by round-off.
- Network (announced): libeigen.gitlab.io, its release page of 5.0, the GitLab API for the issues of `release::5.0.1`
  and the open confirmed bugs since 2025-09. Relevant were only #2982/#2984 (alignment, fixed in 5.0.1), #3230
  (`ComplexEigenSolver` eigenvectors; Exudyn uses eigenvalues only), #3086/#3119 (performance of small sizes).
- MKL: `mkl_rt.2.dll` of `venvExuP313` by ctypes, PARDISO on the Jacobians from `GetSystemJacobian(sparse=True)`. The
  default parallel factorization gave a residual of 1e3 on the revolute chain of 100 bodies with error 0 (44 perturbed
  pivots; first run); `iparm[23] = 10` solved it, but the chain of 1000 bodies failed with every tried setting. The Python
  side has `SetUserFunctionNewton`, but no setter of the Newton increment.
- Then `linearSolver.reuseAnalyzedPattern` (exists, off by default): 20-31 % shorter factorizations where the number of
  nonzeros stays; the spring chain changes it, because the assembly leaves out exact zeros.
- `docs/dev/design/linalgBenchmark.md` §3.8, §3.9, G6, G7 and §7. The worktree and the scratch venv removed.
- Raised #3425 for the screw graphics (RG6.24, open: the recommendation and the questions in the plan).

<a id="rg21-6-9"></a>
### RG21.6.9 — pathPlanningUR5.py: the floors at the height of the bases, the wall lower and transparent (2026-11-11, #3424)
*(Maintainer 2026-11-11: the robot with the many obstacles and the objects seem to fly, possibly the ground is too low;
the wall of the first robot lower and transparent, so that the robots and the trajectories are seen from one side;
otherwise it works fine now - the final adjustments.)*

- The cause: both floors had their top at z = -0.1, the UR5 base mesh begins at z = 0 (-0.003 to 0.021) and so do the
  obstacles standing on the floor.
- `Floor(size, color)`: the floor drawn with its top at z = 0, the box of the collision check 0.03 m lower - the capsule
  of the shoulder (link 0, radius 0.0825) reaches 4 mm below the base, so a floor at z = 0 is nearer than the margin at
  every configuration and `PlanToPose` finds no free goal (first run). The table is no longer one of the boxes of the
  scenes; the boxes take an optional color.
- The wall of the scene `wall`: 0.8 m instead of 1.6 m high, standing on the floor, `lightgrey` with alpha 0.5.
- The scene `window` with seed 6: with the floor 0.07 m higher in the collision check, seed 1 found no path within
  20000 points (seeds 2 and 5 neither; 3, 4, 6, 7 did, 6 the fastest, 0.29 s).
- Run without windows: smallest distances during the simulation 0.0111 (wall), 0.0110 (shelf), 0.0113 (window) at the
  margin 0.01, 0.0218 for the second robot at 0.02; the warnings of `ProfileJointPath` in the window scene are #3411.

<a id="rg5-1-11"></a>
### RG5.1.11 — the share of FinalizeMatrix in the Newton step (2026-11-11, #3420)
*(Maintainer 2026-11-11: do RG5.1.11.)*

- `CSolverBase::Newton`: the two parts of `timer.factorization`, `FinalizeMatrix` and `FactorizeNew`, as the global
  timers *finalize matrix* and *factorize* (`show.globalTimers`), always registered and kept in the regular module
  (`STARTGLOBALTIMERmain`; the registration without `addAlways` left the index at 0, the timer of the bounding boxes of
  the contact - first run).
- Measured (scratch finalizeShare.py, tables in `docs/dev/design/linalgBenchmark.md` §3.7): a revolute chain of 10-1000
  rigid bodies, a RigidBodySpringDamper chain, a mass-point chain, EigenSparse, 100 steps. `FinalizeMatrix` is 2-7 % of
  the solver time with full Newton, below 2 % with modified Newton; the factorization by `SparseLU` 53-58 % of the
  revolute chain, the Jacobian 58-66 % of the spring chain (about 10 us per RigidBodySpringDamper and Jacobian).
- Decision recorded in the design note: a pattern mode of `GeneralMatrix` (G1) gains at most these percent - not
  now; the levers are the sparse factorization and the Jacobian of RigidBodySpringDamper.

<a id="rg9-14-2"></a>
### RG9.14.2 — ObjectJointHelicalZ (2026-11-11, #3421)
*(Maintainer 2026-11-11: do RG9.14.2 ObjectJointHelicalZ with RG9.14.3 and RG9.14.4, with issues and steps; the point
on a helix (RG9.14.5 of the analysis) and the following are not further considered - a general curve joint, probably
along a 3rd or 5th order curve in space with an option for a screw motion, comes later, related also to the curve
markers on deformable bodies; when ObjectJointHelicalZ is done, close the design file, move the revised file to
internals keeping the abandoned proposal of ObjectJointPointOnHelixZ; check whether other design files can move to
internals or need decisions.)*

- `definitions/itemDefsObjects.py`: ObjectJointHelicalZ (`HelicalJointZ`, type name `JointHelicalZ`), the parameters
  `pitch`, `constrainedAxes` (5), `activeConnector`, visualization `axisRadius`, `axisLength`, `color`; 5 algebraic
  equations; the description with the equations, the table of the reduced joints and the MiniExample (a nut on a
  vertical screw, $-Fht^2/(2(J_{zz}+mh^2))$ exactly).
- `src/ImplObjects/CObjectJointHelicalZ.cpp`: the translations along $\mathbf{x}_0$, $\mathbf{y}_0$, the coupling
  $d_z - h\tilde\varphi = -h\,\mathrm{atan2}(\ldots)$ with the angle on the branch nearest $d_z/h$, the tilts
  $\mathbf{z}_0\cdot\mathbf{y}_1$, $\mathbf{z}_0\cdot\mathbf{x}_1$; analytic rows, the velocity level as rows times twists;
  $\lambda_i$ on a free equation with `ComputeJacobianAE_AE`; the outputs (`Rotation` z the continuous angle,
  `TorqueLocal` z $-h\lambda_2$); the drawing with the thread by `DrawSpring`. `CheckPreAssembleConsistency` for
  `constrainedAxes`; the include of the factory (`MainObjectFactory.cpp`); vcxproj, filters, sources.json.
- `docs/dev/design/helicalJoint.md` closed: `docs/dev/internals/helicalJoint.md` as built, with the point on a helix as
  not built and its analysis kept; `docs/manual/revisions.md`.

<a id="rg9-14-3"></a>
### RG9.14.3 — helicalJointTest.py (2026-11-11, #3422)

- The screw about the x-axis of the ground (the joint frame turned by the `localHT` of the markers), torque and axial
  force, 2.5 turns, $h = \pm 0.05$, index 3 and 2: the angle and the position against the analytic solution, 1.2e-3 at
  500 steps (the integration error); the continuous angle of `Rotation`; the multiplier of the coupling
  $\lambda_2 = F - mh\ddot\varphi$ (the sign of the multipliers, $\mathbf{C}_q^T\boldsymbol{\lambda}$ on the left).
- Against ObjectJointUser with the screw of userJointTest.py (#3100): 1.3e-14; pitch 0 against ObjectJointRevoluteZ and
  the free coupling against the cylindrical ObjectJointGeneric: the same bits; static, an axial spring against the
  torque, $M/(kh^2)$ to 1e-16; `activeConnector=False`; the refusals of an entry 2 and of a position marker.

<a id="rg9-14-4"></a>
### RG9.14.4 — mbs.CreateHelicalJoint (2026-11-11, #3423)

- `python/exudyn/misc/mainSystemExtensions.py`: as CreateRevoluteJoint, with `pitch` and `constrainedAxes`; a marker
  given in `itemNumbers` gets a copy turned by the joint frame, one without `localHT` is refused (the joint has no
  `rotationMarker0/1`). Registered in `__all__`, `autoGenerateHelper.py` and `utilityDocsModel.py`; used by the static
  case of the test.
- The design notes of closed steps went to internals (WORKFLOW.md §8.4): contactPrimitives.md (RG9.12, RG9.13) and
  robotPlanning.md (RG21) as `docs/dev/internals/` pages, flatArray.md (RG11.14) as §5 of
  `docs/dev/internals/systemJacobians.md` - written by subagents from the notes and checked against the code.

<a id="rg5-1-8"></a>
### RG5.1.8 — local sums over data pointers: matrix times vector, MultAdd and +=/-=, J^T K J (2026-11-11, #3417)
*(Maintainer 2026-11-11: start with RG5.1.8 and do until RG5.1.10; the performance suite before and after.)*

- `MultMatrixVectorTemplate` (`Matrix.h`): the sum of a row in a local of the result's type, written once, the row
  stride hoisted. `VectorBase::MultAdd`, `operator+=`, `operator-=` (`Vector.h`) and `ConstSizeVectorBase` `+=`, `-=`,
  `MultAdd` (`ConstSizeVector.h`): the size and the data pointers in locals - MSVC has no type-based alias analysis and
  reloaded `numberOfItems` and `data` after every store, so the loops were not vectorized.
  `MultMatrixTransposedMatrixTemplate`: rank-1 updates row by row over the data pointers; the Add variant keeps a local
  sum per entry (`result + sum` unchanged), over the data pointers.
- Every sum keeps its order: all 438 results of the test suite in the regular module are the same bits as before.
  The fast module fails `rollingDiscRollingFrictionTest.py` (new today, #926) by 1.5e-13 - the same value at HEAD
  before the change (stash, fast build, rerun); entered in `AVX2ReferenceSolutionUpdate`.
- `RunLinalgBenchmark`, regular / fast, before -> after (ns): `Vector MultAdd` n = 100 64 / 29 -> 13 / 7, n = 500
  348 / 179 -> 58 / 27; `Vector +=` 3-4 times from n = 100; matrix times vector and the dense solve 2-2.5 times at
  n = 6-12, 4.5 at 24, 6-7.5 from 48 (now 0.19 ns per multiply-add, Eigen 0.05-0.1); $\Jm\tp\Km\Jm$ 1.5 times at n = 6,
  2.3 at 12, 3.2-3.5 at 24; `ConstSizeMatrix<36> * ConstSizeVector<6>` 2 times. Tables in
  `docs/dev/design/linalgBenchmark.md` §3.6.

<a id="rg5-1-9"></a>
### RG5.1.9 — SlimVector index loops, Matrix3D products unrolled (2026-11-11, #3418)

- `SlimVector.h`: the scalar constructor, `+= -= *= /=` (also with `VectorBase`), binary and unary `+ -`, the scalar
  products and the dot product as index loops over `dataSize` instead of range-based loops over iterators; the cross
  product written into the data instead of constructed from an initializer list (with its size check).
- `ConstSizeMatrix.h`: `Matrix3D * Matrix3D` (the `dataSize == 9` case of the generic product, by `if constexpr`, when
  all sizes are 3) and `Matrix3D * Vector3D`, `Vector3D * Matrix3D` unrolled over the data pointers, each sum from 0
  in the order before.
- Benchmark, regular / fast: `SlimVector<3> +` 5.5 / 5.7 -> 2.8 / 2.7 ns (Eigen 0.8), the cross product 10.1 / 10.0 ->
  4.4 / 4.3 (Eigen 4.2), `Matrix3D * Matrix3D` 21.3 / 17.9 -> 7.6 / 8.3 (prototype 8.1), `HT * v` 8.4 -> 6.1;
  `Matrix3D * Vector3D` in the regular module 7.2 -> 6.5 (the two size checks remain). Results the same bits.
- Not done: the replacement of `A.GetTransposed() * v` by `v * A` in the item code (64 places; the same bits, a change of
  callers) - left as part of L5 in the design note.

<a id="rg5-1-10"></a>
### RG5.1.10 — the triplets of AddSubmatrix from the data pointers (2026-11-11, #3419)

- `LinearSolver.cpp`: one template `AppendSubmatrixTriplets<transposed>` for `AddSubmatrix`, `AddSubmatrixTransposed`
  and their `WithFactor` variants: the sizes of `LTGrows`/`LTGcolumns` checked once, values and indices read from the
  data pointers, the triplet array enlarged once per call; the order of the triplets and `factor*value` as before
  (`1*value` is the same bits as the former branch without the factor).
- Benchmark: 10 objects 4135 / 1481 -> 1364 / 1407 ns, 1000 objects 350 / 159 -> 145 / 150 us - the regular module as
  fast as the fast one; `setFromTriplets` is now 86 % of the sparse assembly (G1, RG5.1.11).
- Performance tests (`exudev perf`, `--fast`, the same machine before and after RG5.1.8-RG5.1.10): regular 49.06 ->
  43.69 s (-11 %), fast 40.87 -> 35.51 s (-13 %); perfObjectFFRFreducedOrder -34 % / -15 %, perfUserFunctions -23 % /
  -30 %, perfLargeMassSpringChain -9 % / -10 %, perfConnectorInterface -11 % / -15 %; all results unchanged.

<a id="rg14-6-2"></a>
### RG14.6.2 — CommonTangentOf2Circles: the length of nB, the convergence (2026-11-11, #3412)

- `Geometry.h`: `nBlen = nB.GetL2Norm()` (was the length of nA); the function returns `lastError <= tol`, i.e. true if
  the residual of the returned `rA`, `rB` is below the tolerance, instead of `nIt != maxIt` (a convergence in the 8th
  iteration was a failure, 8 iterations without convergence a success). The issue spoke of a second Newton loop of the
  same pattern in the file; there is none.
- `reevingSystemTangentTest.py` (new): two sheaves on the ground, the axis of the second tilted towards the first; the
  rope length (`Distance`) against the tangent computed in the test by another parametrization (the line normal to
  both radius vectors, Newton with a difference Jacobian): 3.025965077828625 both, before the fix 3.0083055912100445.
- `reevingSystemSpringsTest.py`: 2.2155575717433043 -> 2.2156643963423805 - its sheaves rotate, so their axes are not
  exactly parallel.

<a id="rg14-6-3"></a>
### RG14.6.3 — three text errors (2026-11-11, #3413)

- ObjectJointSpherical: `constrainedAxes` describes three translation axes (it spoke of rotation axes 3-5); its
  `markerNumbers` text was the one of the rolling disc (coin, ground marker) - the plain text of the other joints.
- The stray `v` in $\LU{0,J1}{\Rot}$ of ObjectJointGeneric, and the same in ObjectJointPrismaticX.
- `docs/dev/internals/userItems.md` §7, CreateRigidBody: on NodeRigidBodyEP (the default) or NodeRigidBodyRxyz, the
  Euler parameter constraint as algebraic equation (#3238); only Lie group nodes refused.

<a id="rg5-1-7"></a>
### RG5.1.7 — the linear algebra benchmark for small systems, regular and fast module, Eigen as reference (2026-11-11, #3414)
*(Maintainer 2026-11-11: RG5.1 has to be done again, also with the fast module, so that both cases are known - a
requirement for every repeated run. The linear algebra matters most for small systems, the local residuals and
Jacobians of the objects; the largest gain is expected where the sizes are known and loops can be unrolled and
vectorized; long vectors occur only in the time integration, matrix times vector only in the dense solver; focus below
500, possibly on the assembly of the SparseTriplets handed to Eigen; Eigen's linear algebra as reference; local
optimizations within the linalg classes first, global ones such as merging Vector second.)*

- The survey of the computation functions (`src/ImplObjects`, `ImplMarkers`, `ImplNodes`, `System`, `Main`): the rigid
  body and its markers on `Matrix3D` (hand-optimized for the EP and Lie group nodes), the connectors on $\Jm\tp\fv$ and
  $\Jm_i\tp\frac{\partial\fv}{\partial\qv}\Jm_k$ with `ResizableMatrix` (n = 6 ... 24) and `AutoDiff<12>`, the beams on
  `ConstSizeVector`/`ConstSizeMatrix` of run-time size, the numerical local Jacobian column by column, the assembly by
  `AddSubmatrix` into triplets and `setFromTriplets`, the dense solver by `InvertSpecial` and the inverse times the
  residual.
- `RunLinalgBenchmark` extended (`group`, `library` in each entry): vectors 3 to 500 and across the threading limit,
  Eigen `VectorXd` and `Matrix<n,1>`; `Matrix3D`, 6x6 against Eigen; $\Jm\tp\fv$ and $\Jm\tp\Km\Jm$ for n = 6, 7, 12, 14,
  24 against Eigen dynamic and fixed; `AutoDiff<12>`; matrix times vector, `InvertSpecial` and the solve against
  `PartialPivLU`; 12 x 12 local Jacobians of 10, 100, 1000 objects into `GeneralMatrixEigenSparse` with and without
  `FinalizeMatrix`, against adding into a known pattern; `LinalgBenchmarkFixedMatrix`, a matrix of compile-time size with
  plain loops, as the prototype. 228 entries; the fast module built by `exudev build --fast`.
- Eigen 3.3.7 with MSVC is slow for some small fixed products - `Matrix3d * Matrix3d` 41 ns, 7 ns by `lazyProduct`;
  $\Jm\tp\Km\Jm$ for n = 6, 7 420-740 ns -, confirmed by a standalone program with the options of `setup.py`, with and
  without `/arch:AVX2` (scratch eigencheck.cpp); for small fixed products the prototype is the reference.
- Results and causes in `docs/dev/design/linalgBenchmark.md`; the main ones (fast module): below 12 elements every
  vector operation costs the call, 2.2-2.8 ns, compile-time sizes 0.7-0.9 ns; the range checks are not the cost of the
  small kernels, but 2.8 times of `AddSubmatrix`; `MultMatrixVectorTemplate` adds into `result[i]` instead of a local
  sum - 8 times Eigen at n = 24, 27 times at 500, the solve of the dense solver; `VectorBase::MultAdd` 0.36 ns per
  element against 0.08 of `ResizableVectorParallel`; `MultMatrixTransposedMatrixTemplate` 2.6 times the prototype;
  `SlimVector<3> +` 7 times Eigen; `setFromTriplets` 77 % of the sparse assembly, a known pattern 12 times faster;
  `InvertSpecial` as fast as Eigen's LU up to n = 200; 16 threads at the limit of 20000 elements 2.5 times slower than
  one.
- Recommendations: local L1-L6 (the three loops, `SlimVector` and `Matrix3D`, the triplets from the data pointer) as
  RG5.1.8-RG5.1.10; global G1-G6, first the measurement of the share of `FinalizeMatrix` in the Newton step (RG5.1.11)
  before a pattern mode of `GeneralMatrix`; compile-time sizes (RG5.2), the merge (RG5.1.2), the threading limit, the
  dense solver, Eigen 3.4 after.
- `test_linalgBenchmark.py`: the groups and libraries. RG5.1 states the requirement of both modules.

<a id="rg9-14-1"></a>
### RG9.14.1 — helical joints: the analysis (2026-11-11, #3415)
*(Maintainer 2026-11-11: a helical/screw joint as a regular C++ joint like GenericJoint, which also works for a motion
on a screw curve with a radial offset to the axis, and so on a circle; the parts lockable separately; check the
constraint formulation and the options, one or two joints, a good name; a new step, its first sub-step the analysis
with a design file.)*

- `docs/dev/design/helicalJoint.md` (written by a subagent from the joints on the relative frame, the connector
  interface and the user screw of `userJointTest.py`, #3100; reviewed): **two joints**, as the two sets of equations
  share no row - the turn angle from the rotation in the screw, from the position in the point joint - and the radius
  is meaningless in the one and required in the other.
- **ObjectJointHelicalZ**: 4 equations of `JointRelativeFrameEquations` and the coupling
  $g_c = d_z - h\tilde\varphi$, $\tilde\varphi$ the angle on the branch nearest $d_z/h$, i.e.
  $-h\,\mathrm{atan2}(\sin(\varphi - d_z/h), \cos(\varphi - d_z/h))$: no state (the translation counts the turns), one
  root (the periodic form of #3100 also vanishes half a turn off), $h = 0$ the revolute joint, $\lambda_c$ the axial
  force; 5 lockable equations (coupling off: cylindrical; $h = 0$ with the radial ones off: planar). With the frame
  following, a radius only moves marker 1, so the rigid joint has none.
- **ObjectJointPointOnHelixZ**: $\rho - r = 0$, $d_z - h\tilde\theta = 0$ with $\theta = \mathrm{atan2}(d_y, d_x)$; $r > 0$;
  $h = 0$ the circle; [radial, path] lockable (cylinder, helicoid); a fixed orientation by an added ObjectJointGeneric.
  Two rigid markers in the first version, as the connector interface has no mixed kind.
- Tests and sub-steps proposed in the note; they wait for the maintainer's decision. The analysis found two text errors
  of the joint descriptions (#3413).

<a id="rg14-6-1"></a>
### RG14.6.1 — built-in items as symbolic user items: the evaluation (2026-11-11, #3200)
*(Maintainer 2026-11-11: user items, evaluate first, in a new design file userItems2.md to be merged into
internals/userItems.md when finished; beyond #3200 the contact primitives sphere-triangle, cylinder-triangle,
cylinder-cylinder, cylinder-sphere, the 3D geometrically exact beam, the kinematic tree - challenging, but very valuable
- and the rolling disc penalty connector; a proposal for each with limitations, approaches and simplifications.)*

- `docs/dev/design/userItems2.md` (written by a subagent from userItems.py, the internals and design notes of the
  symbolic system and the uniform evaluation, and the C++ of each item; reviewed): the rules a recording must satisfy
  (fixed structure and variants, branches as selects with guarded denominators, iterations unrolled, no exceptions, no
  mbs, the Jacobians, the comparison), the interfaces at hand, one section per item, five interface extensions and the
  order.
- Without extension (LOW-MEDIUM): ObjectConnectorRollingDiscPenalty (ObjectConnectorUser on two rigid markers, a data
  node of 3, the post-Newton step) first; ObjectJointGeneric as ObjectJointUser of the constrained axes, a variant per
  pattern; one contact law of the user contacts, then sphere-cylinder and sphere-triangle by closed forms,
  cylinder-triangle on the infinite plane and capsule-capsule with a fixed 4 points; the geometrically exact beam as
  ObjectUserBody on two rigid nodes with a symbolic inverse tangent operator of SE(3).
- With extensions: the hydraulic actuator (E1, ODE1 coordinates of ObjectConnectorUser), the reeving system (E2, a
  forceUserFunction of n markers), the kinematic tree (E3, MarkerBodyUser recorded and assembled by SymbolicSystem;
  the most valuable), the reduced-order FFRF body (E3, E5 matrix leaves of the parameters, last). Finite cylinder
  contacts and cylinder-cylinder depend on GJK/EPA, not recordable as built - not now.
- Found: `CommonTangentOf2Circles` normalizes nB by the length of nA and miscounts its Newton iterations (#3412); a
  latent use of an unfilled vector in `CObjectKinematicTree::AddJointForces` (not active today, noted in the design
  file); a stale row of userItems.md (#3413).

<a id="rg21-6-8"></a>
### RG21.6.8 — pathPlanningUR5.py: a closed pocket, a narrower plate over the turn, the part as a sphere (2026-11-11, #3416)
*(Maintainer 2026-11-11: the model looks very good already; the top plate hides the scene too much - a quarter as wide
in y, shorter in x with a smaller max(x); the hole of the red blocks closed, the blocks at max/min y shortened in y; the
red part on the tool looks strange and touches the obstacles - a sphere, smaller or with twice the distance to the
obstacles, in particular at the hole and at the target.)*

- The plate 1.3 x 0.2 m (before 1.8 x 0.8 m), x from -0.9 to 0.4 m (before 0.9 m). At its old place (y 0.45-0.65) the
  arm turned beside it, at y 0.37; the plate is over the turn now (y 0.28-0.48), the lamp at its edge: 225 sampled
  joint frames of the trajectory within its footprint are beneath it, none above.
- The pocket: the blocks in front and behind over the full width, the side blocks 0.1 m thick and 0.36 m long between
  them, closed at the corners; 0.40 m between the side blocks - at 0.35 m and twice the margin there is no free
  configuration at the pick.
- The part: a sphere of radius 0.02 m, 0.03 m below the tool, a collision shape of the last link of a second planning
  robot (`collisionShapes` with an HT in the frame of the last link), so that the planner keeps it away from the
  obstacles; the second robot keeps twice the margin, 0.02 m, also at the pick and above the column (the place 0.03 m
  higher). Smallest distance in the simulation 0.0216 m; 87 waypoints, 8 after the shortcuts, seed 2 kept.
- The warning of #3411 is printed by this example as well.

<a id="rg5-1-3"></a>
### RG5.1.3 — CollectCurrentNodeMarkerData of NodeRigidBodyRotVecLG: the rotation matrix once (2026-11-11, #3409)

- The function collects, for every marker on a rigid body in every evaluation, $\Gm$, $\Gm_{loc}$, position, velocity,
  $\Am$ and the local angular velocity. It called the six getters: `GetG` and `GetRotationMatrix` each computed $\Am$
  from the rotation vector, `GetGlocal` copied the identity, and each getter read the coordinates through a
  `LinkedDataVector` of its configuration.
- Now it reads reference, current and velocity coordinates by pointer, computes $\Am$ once by
  `RotationVector2RotationMatrix` with the same sums as `GetRotationParameters`, and sets $\Gm = \Am$,
  $\Gm_{loc} = \Im$; the local angular velocity is the rotational velocity coordinates, as `CNodeRigidBodyEP` does.
- Measured (scratch script rotVecTiming.py): a chain of 1000 rigid bodies, each joined to the last by a
  `RigidBodySpringDamper` on two `MarkerBodyRigid`, RK44 with Lie group integration and the mass matrix inverted per
  body, 200 steps, best of 3: **0.725 s before, 0.571 s after (-21 %)**, the sum of the coordinates
  9.1621499927033 before and after; the same chain on Tait-Bryan nodes takes 0.75 s. (With the dense mass matrix of
  6000 coordinates the run takes 79 s, the inverse dominates.)
- Gates: see RG5.1.6; the test suite gives the same results (bitwise, all references unchanged).

<a id="rg5-1-6"></a>
### RG5.1.6 — exudyn.special.RunLinalgBenchmark: the linear algebra micro-benchmark of every build (2026-11-11, #3410)

- `src/Linalg/linalgBenchmark.h` (header-only, included by `Pybind_manual_classes.cpp`, the pattern of
  `symbolicCppDemo.h`): `RunLinalgBenchmark(minimumTime, numberOfThreads)` times each operation in batches doubling
  until `minimumTime` has passed and returns operation, size, threads, ns per call and a check sum, which keeps the
  compiler from removing the work. `exudyn.special.RunLinalgBenchmark` returns the list of dictionaries.
- The operations: `Vector` +=, *= scalar and `MultAdd` at 3, 10, 100, 1000, 10000 and 1, 10, 50 times
  `ResizableVectorParallelThreadingLimit` (20000); `ResizableVectorParallel` +=, `MultAdd` and `CopyFrom` at the same
  sizes, with `numberOfThreads` > 1 also multithreaded from the limit on (the task manager entered and left as the
  solvers do); `SlimVector<3>` + and cross product; `Matrix3D` times vector and matrix; `ConstSizeMatrix<36>` (6 x 6)
  times vector and matrix; `HomogeneousTransformation` times vector, times transformation and inverse, with and without
  rotation; `Matrix` times `Vector` at 10, 100, 1000.
- First results (regular module, this laptop, `RunLinalgBenchmark(0.02, 4)`), ns per call:

  | operation | 1000 | 20000 | 1000000 | 1000000, 4 threads |
  |---|---|---|---|---|
  | `Vector +=` | 236 | 4857 | 253165 | - |
  | `ResizableVectorParallel +=` | 115 | 2649 | 143491 | 51976 |
  | `Vector MultAdd` | 373 | 7933 | 396059 | - |
  | `ResizableVectorParallel MultAdd` | 274 | 5702 | 294597 | 75336 |

  `Matrix3D * Matrix3D` 21.7, `Matrix3D * Vector3D` 7.3, 6 x 6 times vector 31.7 and times matrix 118, HT times
  vector 8.3, HT inverse 8.6; `Matrix * Vector` 65 ns at n = 10, 14.3 us at 100 (1.4 ns per multiply-add), 2.05 ms
  at 1000 (2.0 ns).
- What it shows for the open steps: `Vector` is 1.4 to 2 times slower than `ResizableVectorParallel` single-threaded
  from 100 elements on, for the same operation - the case of RG5.1.2 (merge) and RG5.1.4 (operator[]); 4 threads
  gain 1.8 times at the limit and 2.8 times at 10^6; the dense matrix-vector product runs at about one multiply-add
  per 1.4 to 2 ns, against 0.1 to 0.4 ns per element of the vector operations - a candidate for RG5.2.
- `python/testing/test_linalgBenchmark.py`: the entries and their keys, single- and multithreaded, the threads
  released afterwards. The table of runs in `WORKFLOW.md` names it.
- Gates: build ok; regeneration: the binding in `pybindModule.py`, the stubs and `Exudyn.md`; test suite and pytest
  in the gate run of RG5.1.3 and RG5.1.6.

*Note 2026-11-11:* the machine of these results is a desktop, AMD Ryzen 9 9950X (16 cores), not a laptop. The full
tables, a second run with 4 and 16 threads and what they say for RG5.1.2, RG5.1.4 and RG5.2 are in
docs/dev/design/linalgBenchmark.md.

<a id="rg9-6-2"></a>
### RG9.6.2 — the static_asserts of the required member functions; ObjectANCFBeam and ObjectBeamGeometricallyExact2D completed (2026-11-11, #2866, #3406)

- `RequiredMemberFunctions(definition)` in `tools/generators/itemHeaderEmitter.py` holds the table of RG9.6.1 and
  gives, per item, the function, the base class whose default raises, and the declaration that requires it: the node
  type (Position, Orientation, their 2D variants, RigidBody), the marker type (Position, Orientation, Coordinate(s),
  JacobianDerivativeAvailable, HasPostNewton), the access function types of a body (`TranslationalVelocity_qt`,
  `AngularVelocity_qt`, `DisplacementMassIntegral_q`, `SuperElement` on `CObjectSuperElement`; not for
  `OwnMarkersOnly`) and the output variables (`GetOutputVariable`, `...Body` or `...Connector` by the main parent).
- Each generated `C<Item>.h` ends with
  `static_assert(!std::is_same<decltype(&CItem::F), decltype(&CBase::F)>::value, "...")` per required function:
  `&CItem::F` has the type of a pointer to a member of the class that declares `F`, so the assertion fails exactly when
  neither the item nor a hand-written parent overrides the default. 353 assertions in 96 headers; the message names
  the item, the declaration and the function.
- The first build failed for exactly the two items of #3406 and for no other: `ObjectANCFBeam` gets
  `GetLocalCenterOfMass` (the middle of the beam axis, the origin of its local frame, x from -L/2 to L/2),
  `ObjectBeamGeometricallyExact2D` gets `GetAngularVelocityLocal` (equal to the global one, as it rotates about z
  only). Before, a `MarkerBodyMass` on the ANCF beam, resp. a `MarkerBodyRigid` on the 2D beam under a connector that
  reads the local angular velocity, raised "illegal call".
- `beamMarkerAccessTest.py` (new): a pendulum of four 2D beams on a `RigidBodySpringDamper` (which reads the local
  angular velocity of its markers) under gravity, and the `MarkerBodyMass` of an ANCF beam drawn at the center of mass;
  result 3.671828577706509. `test_itemRequiredMemberFunctions.py` checks the table on the definitions (7 tests).
- `checkDescriptions.py` leaves the assertions out of the fingerprint of a header, as they repeat its declarations;
  the fingerprints of the other 100 items stay. `NEW_ITEM.md` §4 says what the assertion checks and where its table is.
- Gates: build ok; regeneration: the 96 headers, the two items, the pages; test suite 331 + 107 successful; pytest
  1795 passed; docs ok.

<a id="rg21-6-7"></a>
### RG21.6.7 — pathPlanningUR5.py: pick and place with a second robot among obstacles of all convex shapes (2026-11-11, #3408)
*(Maintainer 2026-11-11: extend pathPlanningUR5 by a second robot with a much more complicated task and more obstacles -
"virtually" picking something inside a hole, like between two boxes, in front, and placing it on a high table or column
behind, so that it turns by 180 degrees, with an obstacle at a higher position to pass beneath; other collision objects
than boxes, if possible.)*

- `ConvexShape` has Sphere, Capsule, Cylinder, Box, Triangle, Points (convex hull) and Cone, all usable as obstacles of
  the collision model; the example uses all but the triangle. `Obstacle(kind, position, color, rotation, **parameters)`
  gives the ConvexShape, its HT and its drawing (Brick, Cylinder, Capsule, Sphere, SolidOfRevolution for the cone,
  FromPointsAndTrigs for the pyramid) at one place, so that drawing and collision check cannot differ.
- The scene, found by trial in a scratch script (pickPlace.py): a pocket of four boxes in front (0.55 m), open at the
  top only - the wrist needs 0.35 m between the side blocks and the tool turned by 90 degrees about z, at a height of
  0.15 m (deeper, or narrower, has no free configuration); the column a cylinder of 0.45 m behind; a plate at 0.7 m
  over the side +y and a wall at the side -y, so that the arm turns via +y beneath the plate (at 0.65 m the arm passes
  above the plate, at 0.55 m no path is found); under the plate a lamp (capsule and sphere), a cone and a pyramid.
  Over seeds 1-6, RRT-Connect finds the path in about 1 s with 80-160 waypoints, 7-8 after shortcuts; the highest link
  origin beneath the plate 0.56-0.59 m, except seed 6 (0.81 m, over the edge of the plate); seed 2 is used (5.0 s of
  motion).
- Pick and place are poses of the tool; `FreeConfiguration` takes the free solution of
  `KinematicChain.InverseKinematicsAnalytic` nearest to a reference (the arm in front, resp. turned by pi), and
  `PlanJointPath` plans between them. The part is a cube below the tool, drawn by a `graphicsDataUserFunction` of a
  ground object from the joint coordinates of the drawn state, so also in the SolutionViewer.
- The example is restructured: obstacles, scenes, robots (`CreateRobot`: meshes or capsules, base at an offset), the
  planning in the frame of the base by one planning robot for both, trajectories, one model with both kinematic trees,
  their traces and the smallest distance each. Without a window, all three scenes of the first robot with the second:
  both at the end of their path (2e-15 / 3e-10), smallest distances 0.0110-0.0117 m for the margin 0.01 m. The view is
  for the maintainer to check; the figure of the robotics chapter shows the first robot only.

<a id="rg2-12-4"></a>
### RG2.12.4 — robotPathPlanningTest.py shows the planned paths (2026-11-11, #3407)
*(Maintainer 2026-11-11: robotPathPlanningTest.py has no visualization; run as in Spyder it shall show what happens,
with the graphics when testIsActive is False; the planned path on a ground object; possibly three robots in one scene
with their collision objects, paths and robots shifted on the ground; more structure and comments.)*

- The checks are in sections (#%%+++: robot, obstacles and collision model; planning; the trajectory; the goal as pose
  of the tool; what the planner refuses), with a comment per check group and docstrings for the helpers; wall and
  table are given by size and position, so that the scene reuses them. The checks are the same, the result 24.
- Run directly: three UR5 at y = -2.5, 0, 2.5 m follow the path of RRT-Connect (47 waypoints), the shortened path
  (10) and the path to the tool pose, each by `TrajectoryFromPath` and `SetControl(..., compensation='static')` on its
  own kinematic tree, the base moved by `robot.base.HT`; per robot its table and (transparent) wall, the tool path of
  the trajectory as a line in the color of the scene, the waypoints as spheres (blue the start, red the goal), a label
  with the number of waypoints. The meshes come from ur5Data.h5 of CommonData_git, otherwise the collision capsules
  are drawn (as pathPlanningUR5.py). Without a window: all three at the goal (joint error 9e-16, 7e-16, 5e-10),
  1.6 s of simulation. The view is for the maintainer to check.

<a id="rg2-9-4"></a>
### RG2.9.4, RG4.24, RG9.6.1, RG5.1.1 — gravity on ObjectFFRF, the rolling friction of a tilted disc, the table of required member functions, the benchmark of HomogeneousTransformation (2026-11-11, #3404, #926, #2866, #3405)
*(Maintainer 2026-11-11: do the next steps up to RG5.1.1 - RG2.9.4, RG4.24, RG9.6.1, RG5.1.1.)*

- **RG2.9.4** (#3404): the way to apply the argument `gravity` of `ObjectFFRFinterface.AddObjectFFRF` was the
  `LoadMassProportional` on a `MarkerBodyMass`, as `AddObjectFFRFreducedOrder` does. Measured against L^T f of the paper
  (objectFFRFforceVectorTest.py): off by 41 of 113-134, in both cases. Two causes in `CObjectFFRF`:
  `InitializeObject` computed `centerOfMass` from `referencePositions` before filling them, so it was always 0; and
  `GetMassWeightedPositionJacobian` gave [m I, -A m chi~ G, 0] - no flexible columns and the undeformed center of mass.
  It is now Phi_t^T M L = [m I, -A PHItTM rf~ G, A PHItTM], the first three rows of the mass matrix, computed as
  `ComputeMassMatrix` computes them; centerOfMass is computed after the reference positions. The load equals L^T f to
  1e-10 (two more checks in objectFFRFforceVectorTest.py, result 12). `AddObjectFFRF` adds the load if gravity is not
  zero and returns it as `loadGravity`. The description of the marker interfaces of ObjectFFRF said the flexible
  coordinates get no share of a MarkerBodyMass; it says what it does now. `ObjectFFRFreducedOrder` takes the
  undeformed center of mass `chiU` in the rotational part as well (the reduced coordinates do not enter it) - not
  changed here.
- **RG4.24** (#926): `rollingViscousFriction` acted against the velocity of the disc center relative to the ground
  point, in the plane; for an upright disc that is the velocity of the trail, for a tilted disc that turns about the
  plane normal it is not (the coin of the test, tilted 0.4 rad, rolling at 1 m/s and turning at 3 rad/s: trail
  1.0 m/s, center 0.77 m/s). The force now takes the velocity of the trail, the time derivative of the contact point
  (which the spin about the disc axis does not move) - the function of the output `Velocity`, now one static function
  for both; in the plane, at the center of the disc, and its moment about marker 0 is added to the ground (it was
  missing). rollingDiscRollingFrictionTest.py: the trail velocity against a central difference of the contact point
  (tilted and upright), the force -mu_r |f_n| v_r from the right-hand side of the solver, the upright disc unchanged,
  and the tilted coin rolling 1 s slowed from 0.77 to 0.48 m/s. The description has the formula. The test models
  with rolling friction (carRollingDiscTest.py, upright wheels) keep their results.
- **RG9.6.1** (#2866): which declaration of a definition requires which member function - the functions the system
  calls for it, whose base class default throws ("illegal call to ..."):

  | kind | declaration | required member functions (throwing default in) |
  |---|---|---|
  | node | output variables | `GetOutputVariable` (CNode) |
  | node | `Position` or `Position2D` | `GetPosition`, `GetVelocity`, `GetPositionJacobian` (CNodeODE2) |
  | node | `Orientation` or `Orientation2D` | `GetRotationMatrix`, `GetAngularVelocity`, `GetAngularVelocityLocal`, `GetRotationJacobian` (CNodeODE2) |
  | node | `RigidBody`, 3D | `GetNumberOfRotationCoordinates`, `GetG`, `GetGlocal`, `GetG_t`, `GetGlocal_t` (CNodeRigidBody) |
  | node | AE coordinates (Euler parameters) | `ComputeAlgebraicEquations`, `ComputeJacobianAE` (CNodeODE2) |
  | object | output variables | `GetOutputVariableBody` / `GetOutputVariableConnector` / `GetOutputVariable` (CObjectBody, CObjectConnector, CObject) |
  | body | `TranslationalVelocity_qt` (`GetPositionJacobian`) | `GetPosition`, `GetVelocity`, `GetDisplacement` (CObjectBody) |
  | body | `AngularVelocity_qt` (`GetRotationJacobian`) | `GetRotationMatrix`, `GetAngularVelocity`, `GetAngularVelocityLocal` (CObjectBody) |
  | body | `DisplacementMassIntegral_q` (`GetMassWeightedPositionJacobian`) | `GetLocalCenterOfMass` (CObjectBody; CMarkerBodyMass::GetPosition) |
  | body | `SuperElement` | `GetNumberOfMeshNodes`, `GetMeshNode`, `GetMeshNodeLocalPosition`/`Velocity`, `GetMeshNodePosition`/`Velocity`, `GetAccessFunctionSuperElement`, `GetOutputVariableTypesSuperElement`, `GetOutputVariableSuperElement` (CObjectBody) |
  | body | ODE2 coordinates | `ComputeMassMatrix`, `HasConstantMassMatrix`, `ComputeODE2LHS` (CObjectBody, CObject) |
  | object | Jacobian flags (`GetAvailableJacobians`) | `ComputeJacobianODE2_ODE2`, `ComputeJacobianAE`, ... per flag (CObject) |
  | connector | penalty, not on the connector interface | `ComputeODE2LHS(..., markerData)`, `IsPenaltyConnector` (CObjectConnector) |
  | connector | constraint, not on the connector interface | `ComputeAlgebraicEquations(..., markerData)`, `ComputeJacobianAE(..., markerData)` (CObjectConnector) |
  | marker | `Position` | `GetPosition`, `GetVelocity`, `ComputeMarkerData` (CMarker) |
  | marker | `Orientation` | `GetRotationMatrix`, `GetAngularVelocity`, `GetAngularVelocityLocal` (CMarker) |
  | marker | `Coordinate`/`Coordinates` | `ComputeMarkerData` (CMarker) |
  | marker | `JacobianDerivativeAvailable` | `ComputeMarkerDataJacobianDerivative` (CMarker) |
  | marker | `HasPostNewton` | `PostNewtonStep` (CMarker) |
  | load | vector / scalar (`IsVector`) | `GetLoadVector` / `GetLoadValue`, `GetRequestedMarkerType`, `GetMarkerNumber` (CLoad) |

  Not required by a declaration: `GetAcceleration`, `GetAngularAcceleration` (only the output variables that a node
  declares and computes itself), the G matrices of `NodeRigidBody2D` (not a CNodeRigidBody), the body rows for an
  object with `OwnMarkersOnly` (ObjectGenericODE2, ObjectKinematicTree: the general body markers are refused).
  A scan of the generated headers and their hand-written parents with the rows for nodes, bodies and markers (not yet
  the connectors, Jacobian flags and loads, whose declarations the emitter reads from the definitions) finds two
  items that declare and inherit the throwing default - #3406, for RG9.6.2: `ObjectBeamGeometricallyExact2D`
  (`AngularVelocity_qt` without `GetAngularVelocityLocal`: a RigidBodySpringDamper on its MarkerBodyRigid stops with
  "illegal call to CObjectBody::GetAngularVelocityLocal", reproduced) and `ObjectANCFBeam` (`DisplacementMassIntegral_q`
  without `GetLocalCenterOfMass`). RG9.6.2 writes the table into itemHeaderEmitter.py, from the declarations of the
  definitions.
- **RG5.1.1** (#3405): no benchmark of RG5.1 exists yet, so a standalone one: htBenchmark.cpp including
  `src/Linalg/HomogeneousTransformation.h`, compiled with the options of setup.py (/O2 /Ot /Oi /GL /fp:precise), once
  as it is and once from a copy with `homogeneousTransformationUseIdentityFlag = false`; arrays of 1024
  transformations (with rotation, translation only, mixed at random), 20000 repetitions, the best of 5; ns per
  operation, three runs each, stable to 2 %:

  | operation | flag on: rotation | translation | mixed | flag off: any |
  |---|---|---|---|---|
  | H*v | 2.39 | 1.34 | 1.87 | 2.42 |
  | H1*H2 | 3.67 | 2.26 | 4.96 | 3.52 |
  | H^-1 (GetInverse) | 3.16 | 7.70 | 4.93 | 2.48 |
  | H^-1, Invert with locals | 1.00 | 0.99 | 1.00 | |

  The flag saves 44 % (H*v) and 36 % (H1*H2) for a translation, is neutral for H*v with rotation and costs 4 % for
  H1*H2 with rotation; alternating at random it costs 41 % in H1*H2 (branch prediction) - a pattern the markers and
  joints do not have, as an item keeps its kind of frame. Kept. `Invert` wrote `p = -RotateVector(p)`: RotateVector
  reads p through a reference while p is written - with the flag the copy of GetInverse plus that took 7.7 ns for a
  translation, three times the flag off. Written with the translation read into locals first and the translation case
  as a negation: 1.0 ns in all cases, the same arithmetic (the check sums equal); applied. The full test suite is the
  check that results are bitwise the same.

<a id="rg2-8"></a>
### RG2.8, RG2.9.1, RG2.9.2, RG6.18.5 — a converged cable run, the flyball governor with the Lie group integrator, the forces of ObjectFFRF against the paper, Make mp4 (2026-11-10, #2914, #559, #591, #3403)
*(Maintainer 2026-11-10: close RG2.12 and RG6.8; try RG2.8; do RG2.9.1 and RG2.9.2, the paper probably the one of 2019 in
tmp/papers; Make mp4 of the SolutionViewer opens the folder images while the frames go elsewhere, and says completed
without a video.)*

- **RG2.12, RG6.8** closed: their sub-steps were done.
- **RG2.8** (#2914): the cable of perfAccessFunctionsAD was 1 m of 200 elements (0.005 m each, bending stiffness 1,
  axial 1000) with the initial velocity 0.1 sin(i), noise from node to node. The Newton iteration did not converge
  in many steps: with a fixed step the solver stopped ("reduction of step size reached minimum step size"); the
  adaptive step reduced it, so every Newton setting gave another sequence of steps (0.004663 modified, 0.005112 full,
  0.005388 with relativeTolerance 1e-10). Measured (h = 1e-3, 200 steps, the sum of |q|):

  | cable | modified | full | fixed step | h/2 | h/4 |
  |---|---|---|---|---|---|
  | 1 m, sin(i) | 0.004663 | 0.005112 | fails | | |
  | 10 m, sin(i) | 0.296643 | 0.296643 | 0.296643 | 0.397277 | |
  | 10 m, half sine | 0.1431328338 | 0.1431328337 | 0.1431328338 | 0.142810 | 0.142733 |

  The run is the last: 10 m (elements of 0.05 m), the initial velocity 0.1 sin(pi i/200). relativeTolerance 1e-9 gives
  0.14306 (modified) and 0.14304 (full) with some reduced steps; 1e-10 is below what the residual reaches and halves the
  step thousands of times (0.1428 / 0.1447, 14 s) - a step size effect, not the Newton iteration. Reference
  0.14313283378374353, 0.108 s instead of 0.57 s; runPerformanceTests.py: all 45 runs pass, 47.2 s. The entry of
  AVX2ReferenceSolutionUpdate (drift 8.4e-7 of the old cable) is removed; no fast module is installed here, the
  release run of `--fast-module` decides whether the new one needs an entry (RG20.7.9).
- **RG2.9.1** (#559): stiffFlyballGovernor.py tests the Python Lie group RK4 of lieGroupIntegration on 20 steps;
  flyballGovernorLieGroupTest.py takes the compliant flyball governor of stiffFlyballGovernor2.py on rotation vector
  nodes with RK44 and `explicit.useLieGroupIntegration` for 0.2 s: h = 5e-5 within 5.0e-7 of the same model on
  Tait-Bryan nodes at h = 1.25e-5 (no Lie group; Euler parameters are refused by the explicit solver for their
  algebraic equation), 1.7e-8 at h = 2.5e-5 (ratio 29, order 4); the Python Lie group RK4 the same as the C++ one to
  6.6e-14 over 0.01 s. h = 1e-4 is unstable (the torsion of the rods, about 14000 rad/s). The slider sinks from 0.5 to
  0.355 m. 1.3 s.
- **RG2.9.2** (#591): Zwoelfer and Gerstmayr 2019 (tmp/papers), the nodal-based FFRF: L = [Phi_t, -A_bd rf~ G, A_bd]
  (14.25), the mass matrix L^T M L (14.84), the applied forces L^T f (14.88). objectFFRFforceVectorTest.py: 8 nodes
  with lumped masses and bars between all of them, Euler parameters, translated, rotated, deformed (0.01), at rest;
  the mass matrix and the right-hand side of the solver. Agree to 1e-10: the mass matrix with L^T M L, gravity by a
  LoadForceVector per mesh node (MarkerSuperElementPosition) with L^T M Phi_t g. The forceVector [m g, 0, M Phi_t g]
  gives L^T f without its rotational part -G^T rf~^T A_bd^T M Phi_t g: for the center of mass off the origin that part
  is 71.1, for the center of mass at the origin only the deformation gives it (0.256). This is what the description of
  forceVector says ("only ... for bodies with COM at the origin of the reference frame"); exact gravity is the nodal
  loads. Found: `ObjectFFRFinterface.AddObjectFFRF(gravity=...)` of exudyn.FEM stores the gravity and never applies
  it - #3404, RG2.9.4. (The 2021 and 2022 papers were not needed.)
- **RG6.18.5** (#3403): the button Make mp4 opened InteractiveImages2Video with the fixed working directory `images`
  while 'Record frames' writes to `exportImages.saveImageFileName` with `exudyn.config.outputDirectory`; with a wrong
  folder ConvertImages2Video printed the error of ffmpeg and returned, and the dialog said "complete". Now
  `ExportedImagesLocation(visualizationSettings)` gives [directory, pattern] as the renderer writes them
  (`OutputFilePath`, `.png`/`.tga`), the dialog starts there (absolute path); ConvertImages2Video checks the first image
  before ffmpeg (FileNotFoundError, ValueError for a pattern without a number field), raises RuntimeError with the last
  line of ffmpeg instead of printing, and returns the path of the video, which the dialog shows. GUI.md: 'Record
  frames' (the text said 'Record animation') and where the images go. imagesToVideoTest.py: the locations, the
  refusals, and with ffmpeg three PNG frames written without an image library to a video (1639 bytes here).

<a id="rg19-16"></a>
### RG6.22.1, RG19.16, RG11.4.7 — the column of WindowControls, the interactive robot on the controls, primitives and the order of the links in the URDF import (2026-11-10, #3402, #3362, #3393)
*(Maintainer 2026-11-10: WindowControls with top=, the example simpler with it; the update rate of the graphics stays
low by default; RG19.16 with a URDF visualization of a nicer robot (Staeubli or UR5); RG11.4.7.)*

- **RG6.22.1** (#3402): the maintainer found the elements of renderWindowControls.py over the texts of the renderer at
  the top left; `WindowControls` began its column at 0.01 of the window height. It begins at `top`, 0.25 by default;
  the example gives `top=0.3` and the column size instead of a location per element, and keeps the maintainer's
  `graphicsUpdateInterval = 0.025`. rendererControlsTest.py checks the new start.
- **RG19.16** (#3362): serialRobotInteractiveLimits.py is the TX2-90L of `robotics.models` with the meshes of its
  URDF (`AddURDFgraphics`, the capsules without CommonData_git), on the kinematic tree with gravity and
  `SetControl(..., compensation='static')`. The trajectory of SetControl is a small class whose `Evaluate(t)` moves
  the commanded angles towards the slider values at half the maximum joint velocities of the model, so a jump of a
  slider is no step of the PD control. `WindowControls`: a slider per joint over its joint limits, end stops at
  the limits; the toggle 'mouse drag', the push button 'reset'. The drag: at the first step after the toggle the
  mouse (`GetState()['openGLcoordinates']`, which needs no running renderer) and the tool frame are stored; then
  the tool goes to its frame plus the mouse motion turned by `modelRotation` (the mapping of the former example),
  the joint targets from `KinematicChain.InverseKinematicsAnalytic` with the arm configuration of the current
  target, the first solution within the joint limits; the sliders follow by `SetState` without their functions.
  The pose of the tool is `general.renderWindowString`. InteractiveDialog and the redundant coordinates are gone.
  Without a window the script sets the slider of joint 1 to 0.5 and lifts the tool by 0.2 m at t = 1 s: reached,
  the tool at z = 0.839 m (0.639 m before). The view, the drag direction and the feel are for the maintainer to check.
- **RG11.4.7** (#3393): `GetURDFrobotData` draws the visual boxes (`Brick` of the size), cylinders (`Cylinder` along z,
  centered) and spheres of the toolbox (`stype` cuboid, cylinder, sphere) in the color of the URDF, placed as the
  meshes; a mesh without pymeshlab is still left out. `baseLinkNames`: the links fixed to the base. The links are
  taken in a stable order with each parent before its children: the toolbox keeps the order of the file when the
  joints have explicit indices, and a fixed link before its joint link then lost its mass and graphics to the base;
  `jointNumber` stays the index of the toolbox, a loop of parents is a ValueError. Checked: a stub robot with a
  child before its parent and the three primitives (test_urdfRobotData.py); a URDF with the three primitives through
  the toolbox 1.5 (venvP313rtb, the local wheel installed there); test_urdfRobotData.py and
  test_robotModelsAgainstURDF.py with the toolbox all pass, testRobot.h5 unchanged (its URDF has meshes only).

<a id="rg6-22"></a>
### RG6.22, RG6.23, RG21.6.6 — buttons and sliders in the render window, the zoom of the scopes, the notebook scripts in Tier 2 (2026-11-10, #3400, #3401, #3399)
*(Maintainer 2026-11-10: RG6.22 as recommended, except: the renderer thread does not take the GIL, it queues the
event, which the main thread executes; a visualization setting in general puts the icon bar and the buttons and sliders
into the images, default False, the scopes have their own switch. RG6.23 good to go. RG21.6.6.)*

- **The elements** (#3400): `src/Graphics/RendererControls.h` - `Control` (type, view, location `[[x0, y0], [x1, y1]]`
  relative from the top left, text, fontSize, color, backgroundColor, toggle, state, minMax, format) and `ControlList`
  (lock, items, next ID, the element the mouse holds, the user function as `std::function<void(Index, std::vector<Real>)>`),
  one list per SystemContainer in `VisualizationSystemContainer::rendererControls`, cleared by `SC.Reset()`. The slider
  label takes a Python format, of which `{}` and `{:.Nf}`, `{:.Ne}`, `{:.Ng}` are converted to printf (`ConvertFormat`);
  another format is refused when the slider is added, as the label is written in the renderer thread.
- **Drawing and mouse** (GlfwClientExtended.cpp): `DrawControls` as the icon bar in pixels of the frame buffer, the
  background with its alpha value, darker while pressed or switched on, a track and a knob for a slider, the label
  centered. The mouse: icon bar, then the elements (`ControlsMouseButton`, `ControlsCursor`, `ControlsScroll`), then the
  scopes, then the view; any mouse button over an element is taken, so a click never selects or rotates through it.
  A push button sends 1 on release over it and stays 0, a toggle button switches; a slider sends each new value while
  dragged, the wheel 1/100 of the range (CTRL 1/1000).
- **The callback**: `PyQueueRendererControl(id, state)` in rendererPythonInterface.cpp, a queue as the key press
  function has; a value of the same element still waiting is replaced, so a dragged slider does not pile up calls.
  `PyProcessRendererControlQueue` in `PyProcessExecuteQueue` calls the user function in the main thread; an error in it
  is a warning.
- **Python**: `SC.renderer.AddButton`, `AddSlider` (keyword arguments; checked through one dict conversion that
  `SetDictionary` uses too), `SetControlState(id, data, callUserFunction=False)`, `GetControlState`, `GetControls`,
  `DeleteControl`, `ClearControls`, `SetControlUserFunction`; `GetDictionary()['controls']` with the IDs.
  `visualizationSettings.general.iconBarAndControlsInImages`: the elements and the icon bar are drawn before the frame
  is saved. `exudyn.interactive.WindowControls`: one function per element, elements without location in a column at
  the left. rendererControlsTest.py (the API without a window), renderWindowControls.py (a mass on a spring-damper,
  sliders for stiffness and damping, a gravity toggle, a push button, a scope), GUI.md section *Buttons and sliders*.
  The robot model of RG19.16 is not yet moved onto them (its own step).
- **Not seen on the screen**: no run here opens a window (rule 11), so the drawing and the mouse are checked by the
  maintainer with renderWindowControls.py.
- **The zoom of the scopes** (#3401): `RenderState::scopeViews`, per scope the override `[xMin, xMax, yMin, yMax]`, the
  settings it began with (location, minMaxX, minMaxY: a change resets it), and the ranges and the plot area of the last
  drawing, which `ScopePlot::BuildScopes`/`AddScope` write (also for the raytracer); `AddScope` takes the vertical
  range of the override. Idle: a system paused, or no new state for 0.3 s (`StateUpdateCounter`, the sum of the update
  counters of the systems); a new state resets the override, which covers the solver and the solution viewer alike.
  The wheel zooms by 0.85 per step about the mouse (SHIFT: x only, CTRL: y only), a drag of more than 2 pixels moves the
  lines, a double click (0.4 s, 5 pixels) resets. GUI.md, the mouse table of keyBindings.py.
- **RG21.6.6** (#3399): `python/Examples/notebooks` is in `tier2Paths` of regenerate.py: a changed script is a
  warning, a new one not in git a hard gate, as for the pages of the notebooks; checked with a changed script.

<a id="rg4-27"></a>
### RG4.27 — the contacts switch by the gap; the damping at the end of a contact documented (2026-11-10, #904)
*(Maintainer 2026-11-10: close #904 as won't fix; the documentation says that the damping may give negative contact
forces, which keeps the linear spring-damper behaviour between the two bodies, with the restitution-based contact law
as the alternative; in the general section, referred to by the damping parameters.)*

How the contacts switched when it was decided (2026-11-09):

| contact | switched by | the damping may pull at the end of a contact |
|---|---|---|
| ObjectContactCoordinate, ObjectContactSphereSphere, ObjectContactSphereTorus, ObjectContactSphereTriangle, ObjectContactSphereCylinder | the gap of the last post-Newton step | yes |
| ObjectContactCylinderCylinder, ObjectContactCylinderTriangle, ObjectContactConvexConvex, ObjectContactPrimitiveConvex (the contact points) | the gap per contact point | yes |
| ObjectContactConvexRoll, ObjectConnectorRollingDiscPenalty | the gap | yes |
| ObjectContactCircleCable2D, ObjectContactFrictionCircleCable2D | the gap per segment | yes |
| ObjectContactCurveCircles | the gap **and** a pressing normal force (`gap < 0 and fNormal < 0` in its post-Newton step) | no |
| GeneralContact, sphere-sphere and sphere-triangle | `keepContactWhilePenetrating`: True by the gap, **False (the default)** by a pressing force; a sphere-triangle contact with an explicit solver always by the gap | only with True |
| GeneralContact, ANCF cable and other contacts | the gap | yes |

Nothing changes in the code. The general section of the contacts (itemKindDefinitions.py, *Damping and the end of a
contact*, `sec-contact-damping`) explains the pulling force, why the contact stays active while the shapes
penetrate, the impact models HuntCrossley and GonthierCarvalhoMartins as the alternative, and the two exceptions;
the `contactDamping` of 14 contacts refers to it, that of ObjectContactCurveCircles says that it switches off where
its force pulls. ObjectJournalBearing has no such end of a contact. `keepContactWhilePenetrating` of GeneralContact
already said it.

<a id="rg11-6-3"></a>
### RG11.6.3, RG11.6.4 — the 3D joints on their relative frame with analytic derivatives, the documentation of the constraint Jacobians (2026-11-10, #2884, #845)
*(Maintainer 2026-11-10: the decisions of RG11.6.2 as proposed, and the analytic Jacobians - planned as a remark for
RG5 - now too.)*

- **One implementation** (#2884): `src/System/JointRelativeFrame.h` has the 6 equations of a joint on the relative
  frame of J1 in J0 for a set of constrained axes - the translations, global if all three are constrained, else
  $\tv_{i0}\tp(\pv_1-\pv_0)$; the rotations as rigid (or the alternative triple products), revolute or universal joint -
  with their rows by the 12 variations [dp0, dtheta0, dp1, dtheta1]: $\tv_{i0}\tp\dv$ gives $\pm\tv_{i0}$ and
  $\tv_{i0}\times\dv$, an axes product $\tv_{i0}\tp\tv_{j1}$ gives $\pm\tv_{i0}\times\tv_{j1}$, the triple product
  $a\tp(b\times c)$ (a, c of J0) gives $\pm b\times(c\times a)$, as it is invariant under a common rotation. The
  velocity level is the same rows times the twists of the markers, which replaces the three hand-written time
  derivatives. ObjectJointGeneric uses it with its axes and the offset of the user function, JointRevoluteZ with
  [1,1,1,1,1,0], JointPrismaticX with [0,1,1,1,1,1], keeping the rows of the constrained axes
  (`JointConstrainedEquations`): the three joints are 230 lines shorter, the header has the equations once.
- **The interface**: `CObjectConstraint::ComputeConstraintEquationsRigidDerivatives` (false by default) returns the
  equations with a `RigidMarkersDerivatives` matrix (MarkerData.h); `ConstraintEquationsDerivativesRigid` replaces
  `SeedConstraintEquationsRigid` and gives the rows for the Jacobian and the reaction forces, analytic or copied from
  the automatic differentiation of `ComputeConstraintEquationsRigidDiff` (JointPrismatic2D, JointUser). The three
  joints no longer have `ComputeConstraintEquationsRigidDiff` and the template.
- **First try, not kept**: the analytic rows written into the AutoDiff numbers of the existing `RigidDiff` path. The
  seeding and copying of the AutoDiff frames stayed, and the gain was small (reaction forces of the RevoluteZ chain
  0.39 -> 0.31 s); the Real interface halves them.
- **Measured** (scratchpad jointBench.py: 400 Euler parameter bodies in a chain, 200 steps of 1 ms,
  generalized-alpha, EigenSparse, best of 3), automatic -> analytic, total (reaction forces, AE Jacobian):

  | joint | modified Newton | full Newton |
  |---|---|---|
  | JointRevoluteZ | 1.058 -> 0.853 s (0.388 -> 0.190, 0.014 -> 0.011) | 6.02 -> 5.81 s (0.239 -> 0.119, 0.438 -> 0.338) |
  | JointGeneric revolute | 1.003 -> 0.871 s (0.330 -> 0.198) | 6.13 -> 5.98 s (0.213 -> 0.130, 0.472 -> 0.400) |
  | JointGeneric universal | 1.082 -> 0.999 s (0.214 -> 0.129) | 3.91 -> 3.82 s (0.168 -> 0.104, 0.355 -> 0.295) |
  | JointPrismaticX | 0.184 -> 0.145 s (0.095 -> 0.057) | 0.523 -> 0.428 s (0.097 -> 0.057, 0.113 -> 0.094) |

  The same Newton iterations and coordinates to 12 digits. What remains of the reaction forces and the Jacobian is the
  kinematics and Jacobians of the markers and their projection; with full Newton the factorization dominates.
- **A correction**: the velocity level of the alternative constraints of ObjectJointGeneric used the time derivatives
  of the standard ones; it is now the derivative of the alternative constraints.
- **Checks**: test_connectorInterface.py compares C_q with finite differences of the equations (RevoluteZ, PrismaticX
  with rotated marker frames, Generic rigid, revolute, universal with a free translation, spherical, rotated frames,
  the alternative constraints on Rxyz nodes, slope nodes) and C_q^T lambda with the reaction forces.
- **#845**: solver.md, {ref}`sec-constraintjacobian`: C_q chained from the derivatives by the marker kinematics and
  the marker Jacobians, the reaction forces per marker without C_q, the rows of the 3D joints, which constraints use
  automatic differentiation and which compute their Jacobian themselves, and how to check it with ComputeItem and
  NumericalJacobian; systemJacobians.md the interface and the measurement.

<a id="rg21-6-3"></a>
### RG21.6.3 to RG21.6.5 — the trajectory along a path, the colors of the UR5, the figures (2026-11-10, #3397, #3398)
*(Maintainer 2026-11-09: the trajectory of PlanToPose, ShortcutPath and TrajectoryFromPath has many intermediate points,
brakes and accelerates a lot, and its blending cannot be adjusted; a larger blending for parts of almost the same
direction; the UR5 has no colors; a raytraced view of the navigation (texture three times finer, shadows, perspective,
no checkerboard) and of the UR5 in the docs; more details on navigation and planning; more comments in the notebooks.)*

- **ProfileJointPath** (#3397, motion.py): straight edges, each corner rounded by a quadratic Bezier curve from
  blendRadius/2 times the shorter edge before to after the waypoint; the path parameter is scaled per piece so that
  |dq/dp| = 1 at its ends (otherwise the joint velocities jump where a line meets a curve - they did in the first
  version); the velocity along the path by a time-optimal parametrization on numberOfPoints points per piece: the
  largest (dp/dt)^2 from the velocity limits and, by bisection, from |q_pp x + q_p pDDot| <= aMax; a forward pass with
  the largest and a backward pass with the smallest constant acceleration per interval which suits the limits at both
  of its ends, solved as linear inequalities in the unknown end. Measured on the three scenes of pathPlanningUR5.py
  (limits 1.5 rad/s, 4 rad/s^2): durations 4.73 -> 2.77 s (wall), 2.19 -> 1.78 s (shelf), 2.54 -> 1.98 s (window) for
  blendRadius 1 (0.5: 2.77, 1.88, 2.17 s); the old blended PTP stopped seven times at the wall and exceeded the
  acceleration limit at the shelf (6.3 rad/s^2); the new one never stops on the way. Its accelerations exceed the
  limit by 1/numberOfPoints roughly: 10 % at 20 points, 4 % at 40, 1.5 % at 100 (the default), 0.5 % at 200; 0.3 s to
  compute at 100. Tried and not taken: a uniform cubic B-spline with the waypoints as control points (subdivided
  where it collides) - slower than the rounded corners at the same limits. TrajectoryFromPath(blendRadius=1) halves
  the rounding until the check is free, then runs the path itself; robotPathPlanningTest.py checks the limits, the
  continuity of the velocities, no stop on the way, and the stop at a corner without rounding.
- **Collada colors** (#3398, urdfImport.py): pymeshlab merges the parts of a .dae file and drops their materials;
  ReadColladaMesh reads the triangles with the diffuse colors of their materials (nodes, unit, up axis, polylists),
  identical in geometry to pymeshlab on the seven meshes of the UR5 (bounding boxes to 1e-6); corners of equal point,
  normal and color are merged. GetURDFrobotData uses it for a .dae mesh without texture. ur5Data.h5 of CommonData_git
  written again by createUR5Data.py in venvP313rtb (3.3 MB, before 2.6 MB).
- **Figures**: docs/figures/robotNavigation.png and robotPathPlanningUR5.png, raytraced without a window;
  mobileRobotsNavigation.py with the distance map at 1/3 of the planning cell (mapCellSize), no checkerboard, a
  perspective of 0.5, a directional light with shadows, seen obliquely. robotics.md: the collision check, RRT-Connect,
  the shortcuts, PlanToPose, TrajectoryFromPath, the grid, A*, the SE(2) planner and the controllers of the mobile
  robots in more detail. tutorialRobotics.ipynb with more explanation of the trajectories, the control and the
  sensors (by a subagent, checked); both notebooks run again.

<a id="rg6-21"></a>
### RG6.21 — the icon bar at the start; RG11.5.1, RG4.27, RG6.22, RG6.23 (2026-11-10, #3396, #3395, #904)
*(Maintainer 2026-11-09: the icon bar open for 5 seconds at the start; plan the buttons and sliders and the zoom of
the scopes first; the impact models by their names in the contact descriptions; a step to decide #904.)*

- **Icon bar** (#3396): `visualizationSettings.general.iconBarStartTime` (ms, 5000); `IconBarState.startShownUntil`
  is set at the first frame of a window and reset when the renderer starts; until then the bar is shown as if the
  mouse were at it. It is drawn after the frame is saved, so saved and raytracer images do not have it. Not seen on
  screen here (no window may open); to be checked by the maintainer.
- **Contact descriptions** (#3395): the cases of ObjectContactSphereSphere are `ContactImpactModel.Linear` (the
  adhesive elasto-plastic model, linear without adhesion and plasticity), `HuntCrossley`,
  `GonthierCarvalhoMartins`; the other sphere contacts name them and link [ContactImpactModel](#sec-contactimpactmodel);
  ObjectContactCoordinate named them already.
- **RG4.27** (#904): read in the post-Newton steps and in GeneralContact: every contact item switches by the gap,
  except ObjectContactCurveCircles (gap and pressing force) and GeneralContact with `keepContactWhilePenetrating =
  False`, its default (sphere-sphere and sphere-triangle by the pressing force, `CContact.h` `ContactActs`).
- **RG6.22, RG6.23**: the recommendations are in the plan.

<a id="rg11-4-4-2"></a>
### RG11.4.4.2 — the exudyn lists and MatrixContainer in HDF5 (2026-11-09, #1892)
*(Maintainer 2026-11-09: do the HDF5 part, close the rest.)*

`SaveDictToHDF5` stores `Vector2DList`, `Vector3DList`, `Vector6DList`, `Matrix3DList`, `Matrix6DList` as one array
and a `MatrixContainer` as its dense array or as its triplets with the size, each with its type name as `datatype`,
at the top level and in lists; `LoadDictFromHDF5` rebuilds them by their constructors (`_SaveExudynContainerToHDF5`,
`_LoadExudynContainerFromHDF5` in advancedUtilities.py). hdf5ExudynTypesTest.py: all types, an empty list, a sparse
matrix in a list, gzip. #782 (NGsolve) closed as not wanted; RG11.4.4 closed.

The developer note on the C++ solvers moved from docs/dev/design/cppSolverStructure.md to
docs/dev/internals/solvers.md and describes the solvers as they are (maintainer 2026-11-09). RG9.9 closed (RG9.9.5).

<a id="rg9-9-5"></a>
### RG9.9.5 — what is left of the own item number in the arguments (2026-11-09, #3013)
*(Maintainer 2026-11-09: evaluate first.)*

**What is left.** The computation has no own number left in any argument: RG9.9.2 and RG9.9.3 removed it from the objects, connectors, constraints, CSystem, MainObject and the sensors. What still takes a number:

| where | argument | count | own number? |
|---|---|---|---|
| `UpdateGraphics` of the visualization items | itemNumber | 58 objects, 12 nodes, 14 markers, 3 loads, 7 sensors, ~80 files and the generated headers | yes, but the visualization items do not store it; the drawing loop passes its index |
| `CallUserFunction`, `DrawMarker`, `VisualizationItemHelpers.h` | itemNumber | a few | as above |
| `CSystem::ComputeObjectJacobianODE2/AlgebraicEquations/ReactionForces`, `AssembleObjectLTGLists`, `CSystemData::DeleteCObject` | objectNumber | 5 | no: the number selects the object |
| `MainSystemData::PyGetObjectLocalToGlobal...`, the `Raise...` checks, `GeneralContact` | objectNumber, itemIndex | ~10 | no: the Python interface and other items |
| `CObjectGenericDAE::SetNodeNumber`, `CSensor/CMarker::SetObjectNumber/SetNodeNumber` | nodeNumber, objectNumber | setters | no: the item they refer to |
| two commented-out lines (`CObjectALEANCFCable2D.cpp`, `CObjectANCFBeam.cpp`) | objectNumber | 2 | dead comments |

So the only remaining own number is the drawing. RG9.9.4 decided it stays there: nodes and markers stay thin, and the number lives in the loop of `VisualizationSystem.cpp`. Removing it would need a number (or a pointer to the computational item) in every visualization item. That costs memory per item, and the visualization only gains a shorter signature in about 95 functions.

**Recommendation:** close RG9.9 as done, after deleting the two dead comments. If the drawing is to lose the argument after all, the groups would be: (1) the generator and the base classes of the five kinds, (2) the bodies and finite elements, (3) the connectors and joints, (4) nodes and markers, (5) loads and sensors. Each group is one build and the graphics regression test (`test_graphicsRegression.py`). I do not recommend it.

<a id="rg11-4-4-1"></a>
### RG11.4.4.1 — numpy, scipy and HDF5 for MatrixContainer and the lists (2026-11-09, #782, #1892)
*(Maintainer 2026-11-09: evaluate first.)*

**numpy and scipy are there.** `exu.MatrixContainer(m)` and `SetWithDenseMatrix` take a numpy array, the constructor and `SetWithSparseMatrixCSR` a scipy CSR matrix (#1863), and every item parameter of type MatrixContainer takes both. `GetPythonObject()` returns a numpy array for a dense matrix and a dict of triplets for a sparse one. Returning a scipy matrix there would make scipy a dependency of that call; `scipy.sparse.csr_matrix((v, (r, c)))` of the triplets is one line for the user. Nothing to do.

**NGsolve (#782):** not wanted; a conversion through scipy is what `exudyn.FEM` already does. Close #782.

**HDF5 (#1892):** `SaveDictToHDF5` refuses `Vector3DList` and `MatrixContainer` today (measured: "unsupported type"), while a scipy CSR matrix works. Storing the five list types (`Vector2DList`, `Vector3DList`, `Vector6DList`, `Matrix3DList`, `Matrix6DList`) as one array, and a `MatrixContainer` as its dense array or its triplets, each with its type name as `datatype`, and rebuilding them in `LoadDictFromHDF5` by their constructors is about 30 lines in `advancedUtilities.py` and one test model.

**Recommendation:** do #1892 as RG11.4.4.2 (small), close #782, and then close RG11.4.4.

<a id="rg11-6-2"></a>
### RG11.6.2 — the joints on homogeneous transformations (2026-11-09, #2884, #845)
*(Maintainer 2026-11-09: evaluate first.)*

**What is there.** Since revision2026b step RG14.2, the joints use the marker interface. Each joint writes its equations once, as a template `ComputeConstraintEquationsTemplate` over `MarkerRigid<TReal>` or `MarkerPosition<TReal>`. Their Jacobians come from automatic differentiation of that template (the `...Diff` variants with `DReal`), so there is no hand-written Jacobian left in `ObjectJointGeneric`, `JointRevoluteZ`, `JointPrismaticX` and `JointSpherical`. The rotation equations are already entries of the relative rotation: `vRot0 * A1.column(k)` is an entry of $\Am_0\tp\Am_1$, and the position equations are $\pv_1-\pv_0$, or $\Am_0\tp(\pv_1-\pv_0)$ when only some translations are constrained. So the joints effectively compute on $\Hm_0^{-1}\Hm_1$; #2884 is mostly realized, without the name.

**What is left of #2884 and #845:**

1. **One function of the relative transformation for the three 3D joints.** `JointRevoluteZ` and `JointPrismaticX` repeat the equations of `ObjectJointGeneric` for a fixed set of constrained axes (about 45 lines each). A shared template `RelativeFrameConstraints(H0, H1, constrainedAxes, ...)` would leave one implementation. The results should stay identical, and the test suite checks that. Small to medium.
2. **Analytic Jacobians by the relative twist** instead of automatic differentiation. This is purely a performance question: one Jacobian per Newton iteration and joint. It belongs to RG5, after a measurement of the share of the joint Jacobians in a model with many joints. Not now.
3. **The 2D joints, the sliding joints and the rolling disc** have their own geometry (curves, contact). A common relative-frame function does not fit them; they stay.
4. **#845, the documentation of the Jacobians:** the joint pages and the theory part say how the Jacobian is obtained: the automatic differentiation of the equations above, the projection by the marker Jacobians ($\Cm_\qv = \partial\gv/\partial\Hm \cdot \Jm_{marker}$), and which joints have analytic Jacobians. About one section.

**Recommendation:** do 1 and 4 as RG11.6.3 and RG11.6.4 (one issue each), move 2 as a remark to RG5, and then close RG11.6 and #2884.

<a id="rg20-8-4"></a>
### RG20.8.4 — The reduction after a failed post-Newton step in the common controller (2026-11-09, #3394)

`StepSizeController::ReducedStepSize` holds the rule of a step reduced outside of the error control (a recommended
step size times at most 0.75, else `adaptiveStepDecrease`, no growth afterwards), used by generalized-alpha, the
implicit Runge-Kutta solvers and ODE23/DOPRI5. The explicit solvers did not set the no-growth flag and used a fixed
0.5. With this, RG20.8 is complete: the controllers of RG20.8.1 and RG20.8.2 already covered the rest.

<a id="rg20-7-13"></a>
### RG20.7.13 — ObjectFFRFreducedOrder without allocations per Jacobian (2026-11-09, #3351)

The two local vectors of the quadratic velocity terms in `ComputeJacobianODE2_ODE2` are members now;
`FFRFreducedOrder` is in `test_implicitRungeKuttaAllocations.py`.

<a id="rg11-4-2"></a>
### RG11.4.2 — The visualization of an item as its class or a dict (2026-11-09, #2342)

The generated item classes take `visualization=None`, its `V` class or a dict with only the non-default parameters,
through `_ItemVisualization`, which replaces `CopyDictLevel1`; `item.visualization` is always the `V` class, an unknown
parameter is a `ValueError` naming the item. `itemVisualizationArgumentTest.py`.

<a id="rg21-6-1"></a>
### RG21.6.1, RG21.6.2, RG19.14.5 - the examples of path planning, navigation and quadrupeds (2026-11-09, #3390, #3391, #3392)
- Maintainer 2026-11-09: the motion of pathPlanningUR5 goes to an intermediate point far off the goal; check it; sensor traces, the planned trajectory with spheres at the end of each part, two more challenging scenes, shadow and the URDF graphics; the distance map and the planned trajectory in mobileRobotsNavigation, in the renderer and in matplotlib; ANYmal is drawn with only parts of its legs.
- The detour was real: with waypoints of RRT-Connect only, the base joint went past the goal (1.56 for the goal 0.8) and came back. `ShortcutPath` now connects random points anywhere on the path and then leaves out the waypoints that are not needed: the base joint moves monotonically for seeds 1 to 3. The tool still moves up and towards the base, which the wall requires: the stretched arm cannot turn past it. More waypoints (10 instead of 5) and about 0.5 s longer motion.
- A real error found by the test: `Trajectory.FromWaypoints` used each joint at its largest acceleration (`ProfilePTP(syncAccTimes=False)`), so that the motion between two waypoints was not their straight line in joint space, and left it by up to 3 mm of clearance in the arm. `syncAccTimes=True` is now implemented and used by `FromWaypoints`: the smallest distance in the simulation is 11 mm for the margin of 10 mm in all three scenes (2 mm before at the wall). The check of the blends uses a four times finer sampling and the shortcuts keep 1 mm more than the margin.
- Three scenes in one file, chosen by `scene`: wall, shelf (two compartments), window (0.36 x 0.35 m); window fails for seed 2 with 20000 points, as narrow passages are hard for RRT. The pairs left out of the collision check are found in one free configuration of the arm instead of qStart, which fails the shelf scene otherwise.
- The URDF of the UR5 has .dae visual meshes, which `AddURDFgraphics` skips (STL only): `RobotHDF5/ur5Data.h5` of CommonData_git (2.6 MB), written by `createUR5Data.py`, holds them; without the file the capsules are drawn. `showCollisionShapes` draws the capsules over the meshes, as no visualization setting draws the shapes of a CollisionModel.
- mobileRobotsNavigation: the distance map of the grid as texture on the floor, with the region not available to the larger robot and lines every 0.5 m; planned paths with spheres at the waypoints, sensor traces, a matplotlib figure.
- quadrupedsFloatingBase: the default graphics of a link ends at the joint of its child, so that the shanks had none, and the base of ANYmal had no graphics; each link is now a capsule to its child or foot, the base a box. Checked by raytracer images, without a window, for all three examples.

<a id="rg20-4-6"></a>
### RG20.4.6 - the rotations of Lie group nodes in the output within the step (2026-11-09, #3389)
- `SetStateWithinStep`: the 3 displacements of a Lie group node by the cubic Hermite polynomial; `R = R0 Exp(psi)` with psi the cubic Hermite polynomial of `Log(R0^T R1)`, the derivatives `h omega0` and `h T^-1(Log) omega1`, the coordinates by `CompositionRotationVector` from the start, as the integrator, `omega = T(psi) psi_x/h`.
- lieGroupOutputWithinStepTest.py: RK44 against 32 times smaller steps: rotation order 4.07, angular velocity 3.19 (the derivative of the polynomial), position exact for the parabola; 20 rad/s passing 2 pi several times: rotation matrix to 7e-15.

<a id="rg11-5"></a>
### RG11.5 - single items in the reference manual (2026-11-09, #625, #626, #627, #987, #2325)
- ObjectRigidBody: figure of the frames, the reference point and the COM (docs/figures/ObjectRigidBodyFrames). ObjectJointGeneric: tables of the free and locked axes for 2 and for 1 constrained rotations, from `CObjectJointGenericFreeRotAxis` and `LockedRotAxis`. ObjectALEANCFCable2D: short, as asked: the added terms, the output, the paper. ObjectContactFrictionCircleCable2D stays as it is: it has description, equations and figure.

<a id="rg19-15-3"></a>
### RG19.15.3 - the joint torque loads deprecated, compression of HDF5 files (2026-11-09, #3386, #3387)
- createJointTorqueLoads (default None, no loads) and jointLoadUserFunctionList report their use by DeprecatedArgument, expiry 2027; the 14 scripts passing createJointTorqueLoads=False no longer do; #3361 deferred until 2027-01-01. robotDeprecatedJointLoadsTest.py. SaveDictToHDF5 and Robot.SaveToHDF5 with compression='gzip': articulatedDummy.h5 from 13.5 to 4.0 MB. RG19.14.4 (#3388) keeps the real training run of Spot for later.
    
<a id="rg19-13-1"></a>
### RG19.13.1 - densityForInertiaEstimation and CompleteInertiasFromMeshes (2026-11-09, #3357)
- GetURDFrobotData(densityForInertiaEstimation=None) replaces meshDensity, for links without any inertial data; correctInertia is replaced by urdfImport.CompleteInertiasFromMeshes(robotData, density), a function of its own: mass without a positive definite inertia gets that of the solid of the meshes scaled to the mass, no mass the density, then the triangle inequality. testRobot.h5 is unchanged by the new path. urdfCompleteInertiasTest.py.
    
<a id="rg19-11-2"></a>
### RG19.11.2 - the robot examples in Examples/robotics (2026-11-09, #3355)
- 15 examples moved by git mv (not ROSMassPoint.py, kinematicTree*.py and the openAIgym examples, which are no robots); the relative paths to CommonData_git one level longer. The pages of the examples and the lists of relevant examples of the documentation now also read Examples/robotics, so that the moved examples keep their pages and the four other ones of the folder get theirs.
    
<a id="rg19-10-2"></a>
### RG19.10.2 - the articulated dummy as a stored Robot (2026-11-09, #3353)
- createArticulatedDummy.py writes articulatedDummy.h5 (13.5 MB) in CommonData_git: the 7 joints of arm and hand, the rest of the body as graphics of the base. Corrected at this occasion: the center of mass of the hand was relative to the shoulder instead of the wrist, and the mass of hand and fingers was counted twice, in the lower arm and in the hand. humanRobotInteraction.py loads it; the examples runner skips it only without CommonData_git.
    
<a id="rg21-6"></a>
### RG21.6 - examples and notebook of path planning (2026-11-09, #3385)
- pathPlanningUR5.py: a goal given by a pose of the tool, a margin of 0.01, the smallest distance during the simulation 0.0099. mobileRobotsNavigation.py: a mecanum-wheeled robot and a differential drive through a room, both reach their goals; kinematic trees of x, y, phi and the wheels under PD, the contact of the wheels with the ground is not modeled. tutorialPathPlanning.ipynb; a section of the robotics chapter. Left out of the design: ShapesFromLinks, AddToMbs (the shapes as contact items), triangles of URDF meshes as shapes.

<a id="rg21-5"></a>
### RG21.5 - occupancy grid, A*, RRT in SE(2), pure pursuit (2026-11-09, #3384)
- navigation.py: OccupancyGrid (rectangles, circles, convex shapes in a range of heights, DistanceTransform from the occupied cells with a free neighbor in numpy, Inflate), PlanGridPath with line of sight, PlanSE2Path on a chain Px, Py, Rz, so that it reuses PlanJointPath and the C++ check, PathWithHeadings, PurePursuit, DifferentialDriveWheelVelocities. robotNavigationTest.py: A* around a wall 13.47 against about 13.28 known, 0.05 cells; pure pursuit deviates 0.05 at a corner. ConvexShape.BoundingRadius bound.

<a id="rg21-4"></a>
### RG21.4 - RRT-Connect, ShortcutPath, PlanToPose, Trajectory.FromWaypoints (2026-11-09, #3383)
- navigation.py on CollisionModel. UR5 around a wall: 47 waypoints, 4 after 200 shortcuts, 38 ms. TrajectoryFromPath checks the blended motion and halves the blend times until it is free, without blending it is the path. robotPathPlanningTest.py.

<a id="rg21-3"></a>
### RG21.3 - collision geometries of a URDF, up to two capsules per link (2026-11-09, #3382)
- GetURDFrobotData(collision=True) replaces collisionMeshes; keys 'collision' of the links, 'collisionBase'; ReduceURDFrobotData moves them; RobotFromURDFdata(collisionMeshesAs=). FitCapsulesToMesh (assignment by the distance to the axes; the two capsules are taken if the volume of their union is smaller; the points are sampled on long triangles, as a hull alone leaves out the middle of a straight link); the volumes of the union relative to one capsule: UR5 forearm 0.86, upper arm 0.89, Panda link 7 0.67, TX2 link 6 0.79, Spot lower leg 0.63; a link stays at one capsule only if the second brings nothing. UR5, Panda, TX2 and Spot (CommonData_git) re-fitted from the raw meshes. RobotBase.collisionShapes; CollisionModel.ExcludeCollidingPairs, as the capsules of the base and link 2 of the UR5 overlap at any configuration. The capsule of the URDF extensions is not read, the toolbox does not know it.
    
<a id="rg19-14-3"></a>
### RG19.14.3 - spotReinforcementLearning.py on the floating base (2026-11-09, #3381)
- GetSpotModel in the example, the state with position and rotation vector of the base; the 12 joints are set by PD. Checked without the packages gym and stable_baselines3 (not installed; stubbed): the environment stands, 60 RL steps; learning not run. spotModel.py removed.
    
<a id="rg21-2"></a>
### RG21.2 - the collision shapes of RobotLink and Robot, cUtilities.CollisionModel (2026-11-09, #3380)
- RobotLink.collisionShapes (the capsules of robotics.models or tuples of ConvexShape and HT), Robot.GetCollisionModel(obstacles, baseShapes); C++ class PyCollisionModel on EGeometry::ConvexDistance: MinimumDistance, IsFree, IsEdgeFree, with a bounding sphere filter. 2.1 us per MinimumDistance of the UR5 with 2 obstacles at its ready configuration. robotCollisionModelTest.py. ShapesFromLinks of the design is left out for now.
    
<a id="rg19-14-2"></a>
### RG19.14.2 - ANYmal and Spot on the floating base in one model (2026-11-09, #3360)
- quadrupedsFloatingBase.py: both as trees of 12 joints on floating bases, feet on spheres of GeneralContact with explicit integration, standing and lowering the body; Spot only if the data of CommonData_git are there, its masses assumed. The Mars rover is not added (no license). spotModel.py with its 6 dummy joints stays as it is.
    
<a id="rg19-14-1"></a>
### RG19.14.1 - a floating base for Robot (2026-11-09, #3359)
- CreateKinematicTree(mbs, floatingBase=RigidBodyInertia): a rigid body with a Lie group node at base.HT with the graphics of the base, baseNodeNumber of the tree. robotFloatingBaseTest.py: ANYmal in free fall follows the parabola to 1e-12, legs held. QuadrupedANYmalB has 'baseInertia' and 'feet'.
    
<a id="RG20-8-3"></a>
### RG20.8.3 - the automatic step size with Lie group nodes (2026-11-09, #3313)
- Cause: the error estimate of the first step, which has no previous step, compared the positions with the integral of the velocities. The rotation coordinates of Lie group nodes are composed, so that term did not shrink beyond O(h): with a rod with a reference rotation the estimate was 1e5 times the tolerance at h = 1e-3, proportional to h, and the first step went to the minimum 1e-8. There the rounding of the position constraints divided by h^2 gave a velocity error of about 1e-8 which stayed for the whole run. The rotation coordinates are left out of this check.
    - sliderCrank3D, LobattoIIIA4: end error 2.7e-11 at the tolerances 1e-7 and 1e-9 with 201 and 203 steps, as constant steps of the same number; the smallest step 2e-4 instead of 1e-8; Radau3, Radau5 and LobattoIIIC4GGL the same. automaticStepLieGroupTest.py.
    
<a id="rg20-4-5"></a>
### RG20.4.5 - the output within the step by the interpolation of the solver (2026-11-09, #3379)
- DOPRI5 by its dense output of order 4 (Shampine 1986, the coefficients of ntrp45), RK67 by the quintic Hermite polynomial with the end accelerations (the first stage as the acceleration at the start).
    - Decided: the collocation polynomial for Radau and Lobatto. Measured on an oscillator and a constrained pendulum at constant steps: the collocation polynomial gives order 4 for Radau5 (error 3.9e-10 at 400 steps), the quintic Hermite polynomial order 5 (1.3e-11), and for LobattoIIIA4 and Radau3 the same order and error as the collocation polynomial; velocities of LobattoIIIC4GGL order 4 against 3. The quintic polynomial therefore also for these; the collocation polynomial only for the ODE1 coordinates. TrapezoidalGGL stays with the cubic polynomial. Lie group nodes stay linear.
    
<a id="rg11-14-3"></a>
### RG11.14.3 - the C++ unit-test build (2026-11-09, #3378)
- Symbolic.h includes pybind11, which it uses. round, floor and the comparisons have the rule Zero in the operation table (#3116); the unit test expected an exception, the test now says what the table does.
    
<a id="rg19-17-4"></a>
### RG19.17.4 - UR10, LBR iiwa 14 and ANYmal B in robotics.models (2026-11-08, #3375)
- Generated by RobotModelDictFromData. UR10 URDF downloaded from example-robot-data 4.0.9, its inertias are plausible and taken as they are. The iiwa URDF has no inertia: from the collision meshes with one density, 29.5 kg with the base. ANYmal B a tree of 12 joints on a fixed base, parent passed by LinkDict2Robot. Spot and the Mars rover have no license file and are not in models.py (maintainer); spotData.h5 moved to CommonData_git, the Mars rover not added. URDF files of the three in CommonData_git, without meshes.
    
<a id="rg19-17-3"></a>
### RG19.17.3 - collision shapes for UR5, Panda, TX2-90L, Spot, the check of the hull (2026-11-08, #3374)
- FitCapsuleToPoints: a capsule per link containing all points; the test checks containment on the binary STL meshes. ConvexHullOfMesh and collisionMeshes=True of GetURDFrobotData give the hulls with the warning below 0.98 (UR5: 0.645 to 0.945). Spot: spotCollisionShapes.json in CommonData_git, from the visual meshes as its URDF has no collision elements.
    
<a id="rg19-17-2"></a>
### RG19.17.2 - robotics.models on the URDF, RobotModelDictFromData, test_robotModelsAgainstURDF.py (2026-11-08, #3373)
- UR5 and Panda by default in the URDF frames, the former models with useDH=True; the configuration qr of the UR5 is [0, 0, 0, 0, pi/2, 0] now, as the URDF base is turned by pi against the DH base (the same tool pose, 0.3 mm apart in z: 0.089159 against 0.089459 in the DH table). The UR5 inertias and centers of mass from its collision meshes, scaled to the masses of the URDF (the URDF has inertias up to 100 times too large at the wrists); Panda without the hand, gripper='hand' merges it. The joint limits of the TX2-90L are the exact ones of the URDF now. All examples and the test suite pass; reference values unchanged.
    
<a id="rg11-14-2"></a>
### RG11.14.2 - the class FlatArray and the connector interface on it (2026-11-08, #3368)
- src/Utilities/FlatArray.h as designed; the virtual functions of CObjectConnector, their generated declarations, implementations and templates of the items take FlatArray for markers, forces and torques; 16 loops over a hardcoded 2 replaced by loops over the arrays. A local array passed to a template function needs the explicit FlatArray, as a template argument is not deduced through a conversion (6 places).
    - Speed, best of 7 runs of contactPrimitivesTableTest (test length), revoluteJointPrismaticJointTest, springDamperUserFunctionTest. Default module before 2.884 / 0.0262 / 0.136 s, after 2.886 / 0.0268 / 0.135 s; fast module before 3.69 / 0.0219 / 0.124 s, after 3.69 / 0.0221 / 0.127 s; within the spread of the runs.
    - The unit test of FlatArray passes in a unit-test build, with the repair of #3378 applied locally; the build itself is step RG11.14.3.
    
<a id="rg9-13-4"></a>
### RG9.13.4 - contact shapes transparent by default (2026-11-08, #3376)
- The five colors of contact.color with alpha 0.8; said on the general page of the contacts. No test result changed.
    
<a id="rg19-17-1"></a>
### RG19.17.1 - urdfImport.ReduceURDFrobotData (2026-11-08, #3372)
- Locked joints merge the link into the joint link above (mass, center of mass, inertia by the parallel axis theorem, graphics moved, children attached there), removed links go with all below them; the names are those of joints or links, GetURDFrobotData now returns 'jointName'.
    - reduceURDFrobotDataTest.py: mass, center of mass and inertia of the robot equal to link-by-link sums, without the toolbox. Checked on panda.urdf in venvP313rtb: 9 joints reduced to 7, 16.82 kg kept, the tool at panda_hand as fkine gives it.
    - Measured for RG19.17.2: Panda URDF and models.py have the same masses and inertias (link 7 apart, which carries the hand in the URDF); the UR5 URDF has inertia traces of 0.44 (wrist 1 and 2, 1.2 kg) and 0.068 (wrist 3, 0.19 kg) against 0.0041 and 0.0003 in models.py, and 0.47 against 0.012 on the upper arm. Open for the maintainer.
    
<a id="rg19-13-3"></a>
### RG19.13.3 - 0.4 kg of link 5 moved to link 6 of the TX2-90L (2026-11-08, #3371)
- The total of 117 kg unchanged; three test references.
    
<a id="rg20-10-4"></a>
### RG20.10.4 - recommendations for the dynamic solvers (2026-11-08, #3370)
- The recommendations of the maintainer; VelocityVerlet said to be still under development, the explicit solvers without constraints.
    
<a id="rg20-10-3"></a>
### RG20.10.3 - the multipliers of the reference at acceleration level, the study extended (2026-11-08, #3369)
- Measured on the end states at h0/512: the multipliers at acceleration level of LobattoIIIC4GGL, LobattoIIIA4 and Radau5 agree to 1e-12 (sliderCrank3D, FFRFreducedOrder), their own multipliers differ by 1e-3; no floor in any solver down to h0/128 (Radau5 order 5, LobattoIIIC4GGL order 4 on FFRFreducedOrder). tmp/ImplicitRK/accLevelReport.txt.
    - No C++ change: MultipliersFromAccelerationLevel in the study script sets the end state as initial state and reads the initial-acceleration solve.
    - LobattoIIIC4GGL stays the reference and a solver (#3314 closed); the figures show the solvers' own multipliers, with the V at small steps.
    - VelocityVerlet has order 1 on both unconstrained models, as their forces depend on the velocities.
    - 12 PNG replaced by SVG (HTML) and PDF (LaTeX) by the wildcard of the figure directive.
    
<a id="rg9-13-3"></a>
### RG9.13.3 - graphics.Capsule (2026-11-08, #3366)
- The arguments of graphics.Cylinder without those of partial cylinders; the volume of the surface within 2 % at 64 tiles, length 0 gives a sphere.
    
<a id="rg9-13-2"></a>
### RG9.13.2 - isCapsule0 and isCapsule1 of ObjectContactCylinderCylinder (2026-11-08, #3365)
- Two capsules by the closest points of their segments and the side points; a capsule on a cap by the two ends of its segment over the cap.
    - A capsule together with isHollowCylinder1 is refused for both shapes, simpler than the recommendation of the design note (only the hollow one).
    
<a id="rg9-13-1"></a>
### RG9.13.1 - ObjectContactPrimitiveConvex (2026-11-08, #3364)
- Enum ContactPrimitiveType; a face of a polytope is clipped as the union of its coplanar triangles (Face::Polygon); the polytope class and its drawing moved to ContactPrimitivePoints.h.
    - capsulePrimitiveContactTest.py: sphere, capsule, cylinder, cone on a box rest on 1, 2, 3, 3 points at the exact heights.
    
<a id="rg11-14-1"></a>
### RG11.14.1 - analysis of the raw pointers of the connector interface (2026-11-08, #3367)
- 13 virtual functions of CObjectConnector, 33 generated declarations of the force functions, 18 implementation files and
  about 15 places in CObjectConnector.cpp use raw pointers without the size; the hardcoded 2 is in 29 lines.
- Compared: SlimArray with 2 items, std::span, FlatArray; recommended FlatArray<T> passed by value, FlatArray<const T> for
  what is read, range checked like the other classes.

<a id="rg19-15-1"></a>
### RG19.15.1 - the two test models on Robot.SetControl (2026-11-08, #3363)
- About 130 lines fewer in each file. The result changes from 0.76819 to -0.77167 (serialRobotTest): the sign of the joint
  torque of AddSensors, and the compensation at the coordinates of the trajectory instead of the measured ones.
- The friction note of RobotFromURDFdata names the release of the Robotics Toolbox (decision 18 of the design note).

<a id="rg19-13-2"></a>
### RG19.13.2 - the inertias of ManipulatorStaubliTX2_90L (2026-11-08, #3358)
- The manual gives no inertias, only the total of 117 kg (taken with the base). The visual meshes enclose 0.05757 m^3 with
  the base, so the density is 2032 kg/m^3; base 24.9 kg, links 35.4, 26.8, 15.7, 12.9, 1.24, 0.06 kg (arm 92.1 kg). The
  convex collision meshes would give 1724 kg/m^3 and overestimate the slender parts.
- All six fulfil `_InertiaProblems`. The inertias are numbers in models.py, so the model needs no files.
- New references: robotInverseDynamicsTest, robotDictionaryTest, robotFrictionEndStopsTest; the sum of their checks is unchanged.

<a id="rg19-12-1"></a>
### RG19.12.1 - RobotFromURDFdata, Robot.SaveToHDF5 and LoadFromHDF5 (2026-11-08, #3356)
- `RobotFromURDFdata(robotData, gravity, baseHT, toolHT, PDcontrol, urdfFile, enableEndStops)`: replaces the loops of
  serialRobotURDF.py and of `TestRobotFromURDF`; the stored testRobot.h5 is still equal to what the function creates
  (run in venvP313rtb, which has no pytest: the test function called directly).
- `GetURDFrobotData(toolLinkName=...)` gives 'toolHT', the calculation of the test moved into the import.
- The Robotics Toolbox does not read `<dynamics damping friction>`: link.B and link.Tc stay at their defaults, against
  decision 3 of the design note. The function reads these elements from `urdfFile` (a URDF, not xacro).

<a id="rg19-11-1"></a>
### RG19.11.1 - the Spot examples in Examples/robotics (2026-11-08, #3354)
- git mv of the two files and of the data; spotModel.py opens `data/spotData.h5` beside its file, and runs from another
  working directory (checked). The examples runner finds subfolders by itself.

<a id="rg19-10-1"></a>
### RG19.10.1 - the URDF files of the examples in CommonData_git (2026-11-08, #3352)
- `CommonData_git/data/Exudyn/robotics/URDF`: staubli/staubli_tx2_90_support moved from tmp/URDF (4.7 MB), ur_description
  (UR5 only, 6.7 MB) and panda_description (11 MB) copied from example-robot-data 4.0.9 in the cache of robot_descriptions;
  67 files and a README with the sources. CommonData_git is not committed (the maintainer's repository).
- The mesh names of the two URDF files of example-robot-data are `package://<folder>/...` there, so that LoadURDFrobot
  finds them with the folder as package path.
- staubliTX2trajectories.py and staubliTX2blending.py run with the meshes; serialRobotURDF.py reads UR5 and Panda from
  there if it is present (checked in venvP313rtb: UR5 with 6 joints, the Panda with 9, its fingers included).

<a id="rg20-10-1"></a>
### RG20.10.1 — Radau5, Radau IIA with three stages (2026-11-08, #3318)
- C++: DynamicSolverType.Radau5 (16), the tableau of Radau IIA with c = ((4 - sqrt 6)/10, (4 + sqrt 6)/10, 1); the
  generic solver of the tableau (maxStages 3 suffices; the two-step estimate takes 6 points). Python:
  'Radau5' of SolveImplicitRungeKutta, the tableau RadauIIA5; solver.py lists it among the implicit Runge-Kutta solvers.
- The constraints 'end' were written for two implicit stages: the multipliers of 3 stages with 2 blocks of equations,
  a singular Jacobian on the pendulum. A third block, three variants measured (nPendulum, rigidBodyJoints,
  sliderCrank3D, constant steps n = 25..200, reference LobattoIIIA4 with 3200 steps):
  A, the position constraints at the middle stage: order 4 in q and v; B, the position constraints at all three
  stages (Hairer's index-3 Radau IIA): q order 5, v order 3; D, the velocity constraints at the middle stage: q and v
  order 5.0 (the slider crank at the reference accuracy 1e-12 from 50 steps). Taken D, in C++ and Python; the special
  rows (velocity-level constraints, multiplier equations) evaluated at that stage with its multipliers.
- Orders: C++ implicitRungeKuttaSolversTest 5.01 and 5.0 (Radau5 with 5 h, at h it is at the Newton tolerance);
  Python on the pendulum q 5.01, v 4.97, a and z 3.09, ODE1 coordinates 5.02; massOscillator without constraints 5.0.
- Automatic steps (tolerance 1e-10, start 1/10 of the end time): the most accurate with the fewest steps on
  massOscillator (3.3e-11, 311), nPendulum (1.3e-8, 188), rigidBodyJoints (1.1e-9, 110), sliderCrank3D with Lie group
  nodes (8.1e-11, 58; LobattoIIIA4 1.4e-7 with 105, #3313).
- CPU per step (2000 constant steps) relative to generalized-alpha: Radau5 4.4, 9.3, 7.2; LobattoIIIA4 2.6, 3.3, 2.6;
  LobattoIIIC4GGL 7.4, 19.4, 15.4 (nPendulum, rigidBodyJoints, sliderCrank3D): half the cost of LobattoIIIC4GGL.
- Tests: Radau5 in implicitRungeKuttaSolversTest.py (its reference now Radau5 with h/8), implicitRungeKuttaLieGroupTest.py,
  symbolicImplicitRungeKuttaTest.py and test_implicitRungeKuttaAllocations.py (no allocation per step). solver.md,
  symbolicSolvers.md, revisions.md.

<a id="rg20-7-12"></a>
### RG20.7.12 — no allocations per step with rigid body spring-dampers (2026-11-08, #3312)
- Measured with exu.special.InfoStat (difference of 200 and 400 steps): 10 vectors per step with TrapezoidalGGL,
  LobattoIIIA4 and Radau3 on rigidBodySpringDampers, none with generalized-alpha; per evaluation of the system
  Jacobian (MainSolverImplicitSecondOrder.ComputeJacobianODE2RHS) 5, with any factors - the Runge-Kutta solvers
  evaluate it twice per step (positions, velocities), generalized-alpha with modified Newton seldom.
- 1 for a spring-damper on the ground, 3 for two: one per marker on a body. The cause: the local ResizableVector
  radial_q in the derivative of J^T f of ObjectRigidBody (the Euler parameter direction of #3201), allocated at its
  first size in every call. Now a ResizableConstVector, which holds the 7 coordinates of a rigid node locally. 0 per
  step for the three solvers; 0 per Jacobian on all models of the library except FFRFreducedOrder, 2 per Jacobian:
  raised as #3351.
- test_implicitRungeKuttaAllocations.py counts the vectors too and includes rigidBodySpringDampers.

<a id="rg9-12-30"></a>
### RG9.12.30 — the showcase and the table test draw their contact forces (2026-11-07, #3350)
- connectors.show = False removed from contactPrimitivesShowcase.py and from the viewer part of
  contactPrimitivesTableTest.py; the generic joint of the table is not drawn (CreateGenericJoint(show=False)).
- Rendered with interactive.ShowImage: the arrow of the bearing is now drawn. The arrows of the other contacts are drawn
  but hidden: as specified, their heads meet at the contact point, so the arrow of the force on marker 0 lies in the
  body of marker 1 and the other in the body of marker 0 - inside opaque bodies. Left to the maintainer: transparent
  bodies, or the arrows from the contact point outwards.

<a id="rg9-12-24"></a>
### RG9.12.24 — the contacts as a group of the reference manual (2026-11-07, #3344)
- The groups of the objects are the objectType of the definitions, used only by the documentation (itemDocsEmitter,
  itemDocumentationReport; the C++ CObjectType is separate): ObjectTypeContact = 'Contact' (new), objectClassNames and
  kindPlural of itemDocsEmitter. The contact type the maintainer remembered was the section "Contact connectors" of the
  kind Connector in itemKindDefinitions.py.
- The 14 ObjectContact... items, ObjectConnectorRollingDiscPenalty (named a contact connector by that section) and
  ObjectJournalBearing (its theory is in theoryContact.md) are of the type Contact: the pages objectContactIndex and
  objectContactGeneral; the section of the Connector page is now a pointer, the Contact page has the principle, the
  data node and the active set (the journal bearing needs none), the Create functions.
- Drawing: contacts are drawn only with connectors.show, as every connector (VisualizationSystem.cpp:205) - said on the
  Contact page. contactPrimitivesShowcase.py and contactPrimitivesTableTest.py set connectors.show = False and so hide
  their arrows: raised as #3350.

<a id="rg9-12-23"></a>
### RG9.12.23 — all primitive contacts on a shaken table (2026-11-07, #3343)
- contactPrimitivesTableTest.py (new): a table of 1 x 0.7 m with borders 0.15 m high, moved by CreateGenericJoint with
  an offsetUserFunction: a sin^2 pulse of 0.2 s, 5 cm up, 2 cm sideways, 0.12 rad about x, then at rest. On it two
  spheres, two cylinders lying along y, two cones standing, a cube and an icosahedron; 52 contact items, all by the
  Create functions of #3342: the planes of top and borders for spheres (CreateSphereTriangleContact with
  infinitePlane, #3347), cylinders and cones, the boxes of top and borders for the polytopes; the pairs sphere-sphere,
  sphere-cylinder (4), cylinder-cylinder, cone-cone, cylinder-cone as ConeCone with equal radii (4), cube-icosahedron.
- It found two bugs at once: CreateSphereTriangleContact refused the list of points and had no infinitePlane (#3347),
  and the icosahedron landing flat stopped the solver in the first step (#3349).
- Checks: the solver finishes; every body inside the borders and above the top; the bodies jump (0.17 m); run directly
  for 2 s, the largest velocity below 0.05 m/s (0.027: spheres and cylinders roll on, nothing resists rolling). The same
  with 750 and 1500 steps per second. As a test the first 0.4 s, 3 s, with the tolerance 1e-7 of the model
  (exu.sys['testTolerance']): impacts make the last digits depend on the platform; two runs gave the same digits.
- checkInitialConstraints off: the offset user function of the generic joint has no velocity level.

<a id="rg9-12-29"></a>
### RG9.12.29 — ObjectContactConvexConvex: the points in contact in the first blocks (2026-11-07, #3349)
- Found with the test of the shaken table: the solver failed in the first step (t = 0.001); by switching the groups of
  contacts off, the polytopes against the boxes of the table; the cube alone rests at every start, the icosahedron
  placed or dropped flat (0, 1e-4) fails, started 1e-5 deep it runs.
- Cause: all 12 points of the icosahedron are over the face of the box; Candidates keeps the 8 deepest; the 3 points of
  each layer have equal heights, so which of them are kept changes from one Newton iteration to the next; CopyTo gave
  the kept points to the data blocks in the order of their ids (#3331, so that a point keeps its block), and the 3
  points in contact moved to other blocks: the gap of a block switched sign each iteration.
- Now deepest first, equal gaps by id: the points in contact take the first blocks whichever others are kept; two points
  in contact may swap blocks, which changes no contact state. The icosahedron of the experiment runs from 0, 1e-4 and
  1e-2 above the slab, and inside the borders.
- convexConvexContactTest.py: an icosahedron dropped flat from 1 mm rests at its inradius - m g/(3k) (to 1e-15); with the
  old order the SolverError of the minimum step size (checked by building without the fix). The cubes are unchanged.

<a id="rg9-12-28"></a>
### RG9.12.28 — the Create functions of the primitive contacts in __all__ (2026-11-07, #3348)
- tools/checkAll.py: __all__ of misc/mainSystemExtensions.py lacked the six Create functions of #3342; written with
  checkAll.py --write (31 names). The generate gate had stopped at checkAll in the runs of #3342 and #3347; its short
  output was misread as passing (the gates are now read in full).

<a id="rg9-12-27"></a>
### RG9.12.27 — CreateSphereTriangleContact with a list and infinitePlane (2026-11-07, #3347)
- Found writing the test of the shaken table: CreateSphereTriangleContact passed trianglePoints to the item as given,
  and ObjectContactSphereTriangle refuses a list (ExudynTypeError), although the docstring promised a list or a numpy
  array; and infinitePlane of the item (#3323) was no argument.
- trianglePoints through TrianglePointsArgument of #3342 (a Vector3DList); infinitePlane after localPosition1, so that
  the positions of the earlier arguments stay; its check; "the quad" of the docstring is "the triangle".
- The test went into createPrimitiveContactsTest.py, with the other Create functions of the contacts (not into the
  older createSphereTriangleContact.py): a ball on the plane of a triangle given as a list, 3 m outside the triangle,
  at r - m g/k.

<a id="rg9-12-22"></a>
### RG9.12.22 — Create functions of the primitive contacts (2026-11-07, #3342)
- mbs.CreateSphereCylinderContact, CreateCylinderCylinderContact, CreateCylinderTriangleContact, CreateConeConeContact,
  CreateConeTriangleContact, CreateConvexConvexContact (the design proposed the first three and the convex one; the cones
  came later, RG9.12.11). Each adds the markers, a data node of the size of the item (points times 3, times 4 with
  contactPlasticityRatio > 0, apart at the start) and the item; arguments as CreateSphereSphereContact: itemNumbers
  (body, node or marker), the law, impactModel, the plastic model, contactForceUserFunction, activeConnector, show, color.
- A cylinder or a cone is placed by localPosition and localAxis on its item (a rigid marker whose z axis is the axis:
  the columns n1, n2, axis of ComputeOrthonormalBasis); a marker is used as it is, with the axis [0,0,1]. Triangle points
  relative to localPosition1; polytope points relative to localPosition0/1 (added to the points: the marker stays at the
  body origin), e.g. from graphics.ToPointsAndTrigs.
- Shared helpers (not public): AxisHT, ShapeMarker, TrianglePointsArgument (a Vector3DList; the item refuses a list),
  CheckPrimitiveContact, AddPrimitiveContact. RaiseTypeError raises a ValueError, also for a radius of 0.
- createPrimitiveContactsTest.py (new): six bodies at rest at their closed-form heights (to 1e-11; the shaft in the ring
  on its 2 rims, the cones on 3 rim points, the cube on 4 corners), the data node sizes 24 and 32, two refused
  arguments. The six docstring examples run and come to rest where expected.
- The drawing texts of the ten contact items name the force arrows of #3339; theoryContact.md lists the Create
  functions; the design document says they are done.

<a id="rg9-12-21"></a>
### RG9.12.21 — contactPrimitivesShowcase.py revised (2026-11-07, #3341)
- Six lanes on a checkerboard floor, each drawn: the ball over a lying cylinder onto the floor; the roller down a
  drawn ramp (a finite CylinderTriangle, the plane continued below the floor) onto the floor; the cone in its circle;
  an icosahedron (12 points, ConvexHull of the script: faces of coplanar points as fans, outward) tumbling down a
  wedge of 31 degrees (at 19 degrees it stuck: tan < mu) into a cube, 4 ConvexConvex contacts; a ring (inner radius
  0.12) swinging on a fixed shaft of radius 0.03 on two posts (with 3 cm clearance it hardly moved); a rotor of 0.1 m
  in a bearing with 5 mm clearance, the viscosity from the closed form of the short bearing for eps = 0.9, released
  below the center: it settles at the closed-form attitude (0.0016, -0.0042).
- Found on the way: the cube sinking at rest (#3338, RG9.12.18), the rotor turning the other way about the axis of
  marker 1, which is along -y (#3345, RG9.12.25), the resting icosahedron (#3346, RG9.12.26).
- The lane of the polytopes failed the discontinuous iteration also with these fixes, with 1500 steps alone and with
  3000 steps together with the ring: the icosahedron slides at about 1 mm/s, the default frictionProportionalZone, and
  switches between sticking and sliding in each iteration; with 1e-4 or 1e-2 it runs. The example uses 0.01; with it
  all lanes run with 1500, 2000, 3000, 4000 and 6000 steps to the same end.
- Contact forces drawn (#3339), 0.5 m per 100 N; light from above with shadow 0.4 and a light radius for soft shadows;
  useRaytracer switches the SolutionViewer to the raytracer, without the texts. The view z up as in the other examples
  (RotationMatrixZ @ RotationMatrixX, checked with interactive.ShowImage, which takes the initial settings; the window
  itself was not opened).
- Time: 3.9 s for the script, 2000 steps (the old one 5.8 s with 4000 steps, most of it EPA at its iteration limit);
  the lane of the polytopes is 2.3 of it.
- Left from RG9.12.20 (#3340): the stubtest baseline lacked VSettingsContactShow, VSettingsContactColor (the metaclass,
  as every structure) and the six deprecated members of VSettingsContact (as VSettingsGeneral.drawWorldBasis); written
  here. The gates had run the stub check before the build, against the module installed before the change.
<a id="rg9-12-25"></a>
### RG9.12.25 — ObjectJournalBearing for both directions of rotation (2026-11-07, #3345)
- Found with the revised contactPrimitivesShowcase.py: the bearing axis is the local z axis of marker 1, there along
  -y, the rotor turns at +50 rad/s about y, so omega = -50 about the axis; the rotor drifted to the clearance.
- The forces of Flores et al. (2009) assume the film in the gap converging in the direction of rotation, w = omega -
  2 gammaDot > 0. With w < 0 the wedge term of Fr, -K 2 eps^2 w/(1-eps^2)^2 (long: -K eps^2 w/((2+eps^2)(1-eps^2))),
  pushes outwards.
- First tried the exact mirror (|w| and the whole Ft reversed): the squeeze term of Ft then jumps where w changes its
  sign, which it does at the start from the center (gammaDot = e_t.u_t/e large for small e); even the journal turning
  forward failed the Newton iteration at t = 0.004 (relative tolerance 1e-10 of the test).
- Taken: the wedge term of Fr with |w|, continuous; Ft unchanged, whose wedge term changes its sign with w. The steady
  equilibrium is the exact mirror; transients differ in the squeeze term of Ft only.
- journalBearingTest.py: a short and a long journal turning the other way, at the mirrored eccentricities to 6.4e-10 m
  (c = 1e-4; checked to 1e-4 c); before the fix 1.8e-4 m off. The forward journals changed by 1e-6 in eps (their start
  transient has w < 0 for a moment); the description of the item says it, checkDescriptions marked.

<a id="rg9-12-26"></a>
### RG9.12.26 — EPA refined where it stops before it converges (2026-11-07, #3346)
- Found with the revised contactPrimitivesShowcase.py: an icosahedron resting on a face on the slab made the
  discontinuous iteration fail at the minimum step size (lane 3 alone with 1500 steps at t = 2.297; all lanes with
  3000 steps at 1.87). cUtilities.Distance near that pose: 2 of 5000 normals 0.0115 rad off -z. The guard of #3338 stops
  EPA at a face that is not one of the Minkowski difference; ObjectContactConvexConvex takes the face points only for a
  normal within 1e-6 (cos) of a face, else one closest point.
- ConvexEPA::converged: true only when the support in the normal is on the face (the tolerance); ConvexDistanceCores
  then refines a result that did not converge as the cylinders (shape 1 moved out along the normal, GJK on the
  separated shapes, the smallest support bound kept). 0 of 20000 off; the resting cube to 3e-10.
- convexConvexContactTest.py: 5000 poses around the one of the showcase, with its slab points in their order (the order
  decides the ties of the support of a box: with the box of the test centered the poses did not fail); the normal to
  1e-16, without the fix 6.7e-5 (checked by building without it).
- The failure of the showcase lane itself remained: it is the friction, see RG9.12.21.

<a id="rg9-12-20"></a>
### RG9.12.20 — contact.show and contact.color (2026-11-07, #3340)
- VSettingsContactShow and VSettingsContactColor (new), the members show and color of VSettingsContact, each with
  triangles, spheres, tori, cylinders, cones (the defaults and colors of the former members).
- The members of 1.13.6 (git f172eb2c4, version.txt 1.13.6) - showTriangles, showSpheres, showTori, colorTriangles,
  colorSpheres, colorTori - forward as deprecated (Deprecated('1.13.454', 2031), the path contact.show.spheres etc. from
  the top structure); showCylinders, showCones, colorCylinders, colorCones came after 1.13 and are removed.
- The rest of contact (search tree, bounding boxes, tilings, the contact forces) stays: those are not per shape.
- Renamed in the C++ (the contact items, CContact.cpp), the drawingSettings and descriptions of itemDefsObjects.py, 22
  models and tests of python/, the design document and the section after 1.13 of revisions.md.
- parameterConversionTestReference.txt re-recorded (contact.show, contact.color); checkDescriptions marked for the four
  sphere contacts.

<a id="rg9-12-19"></a>
### RG9.12.19 — the contact forces drawn as two arrows (2026-11-07, #3339)
- EXUvis::DrawContactForce: the force on marker 0 red (0.9, 0.1, 0.1), its negative green (0.1, 0.75, 0.1), each
  contactForcesFactor times the force long, the heads at the contact point; the radius contactForcesRadius (new, 2 mm),
  at most a tenth of the length, so that DrawArrow (head 7.5 radii) keeps a shaft.
- VisualizationItemHelpers.h: VisualizationMarkerData (position, rotation, velocities of the markers in the
  visualization configuration; position markers with the unit matrix), DrawContactForces (showContactForces and no
  user function of the force: the render thread cannot call a Python function, #3003, and a symbolic one has no copy
  of its recording for that thread).
- ContactPoints::DrawForces for CylinderCylinder, CylinderTriangle, ConeCone, ConeTriangle and ConvexConvex: the points
  of the visualization configuration, PointForce with the contact state from the gaps (contactFromData=false) and the
  visualization coordinates of the data node. SphereSphere, SphereTriangle, SphereTorus, SphereCylinder by their
  ComputeConnectorProperties at the point of their Position output; the journal bearing at the journal surface
  towards the eccentricity.
- Sign: forces[k] of ComputeConnectorForceRigid are in the sign of the left-hand side (#2745), forces[1] = fVec, so fVec
  is the physical force on marker 0 in all of them (first drawn with the opposite sign, found by the test).
- Drawn only for items with visualization show=True, as the shapes (#3326); the description of showContactForces says so.
- test_graphicsData.py: two spheres 1 cm deep (100 N, 0.1 m arrows, the red one above the contact point), a cube on a
  box with 4 arrows of each color.

<a id="rg9-12-18"></a>
### RG9.12.18 — EPA keeps the origin inside its polytope (2026-11-07, #3338)
- Found in contactPrimitivesShowcase.py: the cube resting flat on the box (ObjectContactConvexConvex, 0.08 mm deep)
  sank 2.2 cm at t = 0.471 s and bounced. The Distance was -0 with the normal +z instead of -z; the face points were
  then measured against the bottom face of the box (gaps -0.2).
- Reproduced with cUtilities.Distance: 439 of 20000 poses near rest (turned about z, tilted by 1e-9, 0.02-0.2 mm deep).
  EPA ran into its 128 iterations: on the nearly flat side of the Minkowski difference it added points of 1e-9 height
  (above its tolerance of 1e-10 scale), grew to 210 faces, until a thin new face turned over (minD = -dBefore), the
  origin lay outside the polytope, and the face of the most negative distance was returned with the depth 0.
- ConvexEPA: AddPoint refuses a point that is a vertex already and, while expanding (keepOrigin), a new face with the
  origin outside (n.a < -tolerance); EPA stops at the last valid face. It starts from the tetrahedron of GJK when GJK
  ends with one that is not flat (volume > 1e-6 of the greedy one) - alone this did not help, it keeps the start
  inside. 0 of 20000 fail, the depth to 9e-10 (the tilt); the showcase solves in 1.4 s instead of 5.2 s.
- convexConvexContactTest.py: 2000 resting poses, depth and normal to 1e-8 (+1 in the result); no other test changed.

<a id="rg9-12-8"></a>
### RG9.12.8 — the theory section and the example of the primitive contacts; RG9.12 closed (2026-11-07, #3337)
- docs/manual/theoryContact.md, section "Contact between primitive shapes" (label sec-contact-primitives), before the
  ANCF section: the table of the ten items with their options, the distance (GJK, EPA, the refinement of curved sides,
  cUtilities.Distance, the output Distance when inactive), the contact points of lines, faces, hollow shapes and
  polytopes, the law with ContactImpactModel and the elasto-plastic option, the data node and the active set, the
  contact force user function, the journal bearing with the load capacity of the short bearing and its whirl.
- python/Examples/contactPrimitivesShowcase.py: a ball over a cylinder onto the floor, a roller down an inclined plane,
  a cone rolling in a circle, a cube tipping onto a box, a shaft falling into a ring, a rotor in a journal bearing;
  checked by its final positions (all at rest or rolling as expected) and the graphics data.
- Not in this step: the Create functions of the design (CreateSphereCylinderContact, ...), and the plastic option of
  SphereTriangle and SphereCylinder (#3336).

<a id="rg9-12-9"></a>
### RG9.12.9 — the plastic overlap and the adhesion of the contact points (2026-11-07, #3336)
- The Edinburgh adhesive elasto-plastic model of ObjectContactSphereSphere (loading with k_c, unloading and reloading
  with K2 = k_c/(1 - lambda_P) from the plastic overlap, adhesion -k_adh delta^n_adh, a constant pull-off force, with
  impactModel Linear) as the option ContactPlasticity of ContactPrimitiveForce.h, used by the contact points
  (ContactPrimitivePoints.h) of CylinderCylinder, CylinderTriangle, ConeCone, ConeTriangle and ConvexConvex, with the
  four parameters of SphereSphere. With contactPlasticityRatio > 0 the blocks are 4 long (GetDataVariablesSize), the
  fourth the plastic overlap, updated in the post Newton step as in SphereSphere: lambda_P times the deepest penetration
  of the current contact, kept from the start of the step, 0 apart.
- SphereTriangle and SphereCylinder keep their own data (one point, 4 and 3 coordinates); they are not changed here.
- contactPlasticityTest.py (new): an elastic cube rests m g/(4k) deep to 1e-17; a plastic one with a pull-off force at
  lambda_P delta_max + (m g/4 + f0)/K2 to 1e-17 with the overlap of the data node; the sensor of the Distance, written
  at the step size, gives delta_max 2e-4 lower than the overlap - the deepest instant lies in a step reduced at the
  impact, between the written times.

<a id="rg9-12-13"></a>
### RG9.12.13 — ObjectJournalBearing (2026-11-07, #3335)
- A separate item, as decided (design 8.4, 8.6): journal (marker 0) and bearing (marker 1, its local z axis the axis),
  no data node. The eccentricity normal to the axis, its rate (radial rate and the rate of the attitude angle) and
  omega, the sum of the angular velocities of journal and bearing about the axis. The film force in the frame of the
  eccentricity after Flores, Ambrosio, Claro, Lankarani, Koshy (2009): the short bearing (Ocvirk) and the long bearing
  (Sommerfeld, half film) with their squeeze terms; the formulas are written from that paper and checked here against
  the classical steady forms - the load capacity and attitude angle of the short bearing, the steady radial and
  tangential force of the long bearing; the squeeze terms are not checked by a closed form. epsilon is limited by
  maximumEccentricityRatio (0.99); beyond the clearance a linear penalty contact. The force acts on the line through
  the journal center; no friction torque of the film. JournalBearingModel {Short, Long} as the enumeration of #3327
  (ints accepted); the generator converts both enumerations by one branch. forceUserFunction(mbs, t, itemNumber,
  eccentricity, eccentricity_t, angularVelocity, axis) replaces model and contact; its example is the short bearing.
- Finite bearings (correction functions, Bastani and de Queiroz) are not built in; they are the "further models" the
  user function adds, or a later bearingModel.
- journalBearingTest.py (new): the film force at the closed-form equilibrium of the short bearing (eps 0.8613 for
  m g = 98.1 N, attitude 0.4339) is [0, m g, 0] to 1e-12; floating journals: a rigid rotor at eps below about 0.8 whirled
  (oil whip: a first run at 1000 rad/s ended at eps 1.8), so the speeds give eps 0.86 (short) and 0.93 (long); a small
  whirl decays slowly, so the floating journals are checked to 1e-3 of the load; the user function against the built-in
  short bearing 1e-17; the dry journal m g/k beyond the clearance to 1e-16.

<a id="rg9-12-12"></a>
### RG9.12.12 — the contact inside a hollow cylinder and a hollow cone (2026-11-07, #3334)
- isHollowCylinder1 of ObjectContactCylinderCylinder and isHollowCone1 of ObjectContactConeCone, as decided in the
  design 8.6; the hollow shape keeps its radii, its length and two open ends. The points (HollowPoints): on each rim of
  the inner shape - or, where the inner shape leaves the hollow one, at its end plane (a long shaft through a ring
  rests on the ring's edges) - the point in the direction of the eccentricity of the rim and the points at +-120
  degrees, so a centered shape (a cone in its seat) rests on all sides; the distance of a point of a side line from
  the axis is largest at its ends, so the rims carry the contact. The gap (R - rho)/sqrt(1 + R'^2) is normal to the
  wall; the normal is that of the wall. Up to 6 points: the cylinder contacts have room for 8, and the data node needs
  24 coordinates with the flag (GetDataVariablesSize; the post Newton step loops over the blocks of the data node).
- The check at Assemble: cylindersRadii[0] <= cylindersRadii[1]; for cones the smaller radius of the inner cone not
  above the larger of the seat. The hollow shape is drawn as a tube 0.2 of its radii thick; DrawCylinder scales the
  inner radius with the outer one, for the conical tube.
- hollowContactTest.py (new): the shaft in the bore and through the ring, R - r + m g/(2k) below the axis to 1e-17;
  the cone in the seat of the same taper m g (1 + R'^2)/(6 k R'^2) below the fit to 1e-17; the refusal of a shaft
  larger than the bore.

<a id="rg7-15-9"></a>
### RG7.15.9 — the drawing of CreateCartesianSpringDamper of UserItemCreator (2026-11-07, #3333)
- _JointGraphics of userItems.py read the RotationMatrix of marker 0 for every shape; a Cartesian spring-damper on a
  mass point has a position marker, and the exception ended the user-function drawing of the whole system: only the
  lines drawn before remained (GetGraphicsData: 2 lines, none of the 24 triangles of the bodies). Now the rotation is
  read only for the revolute and prismatic shapes: 115 lines, 24 triangles, 1 sphere.
- Found while probing: a change in python/exudyn is in effect only after the build installs the package - the first
  probes ran the old userItems.py.
- symbolicUserConnectorsTest.py: bodies and mass point with graphics, the SolutionViewer of the Python user items when
  run directly (result unchanged); test_graphicsData.py checks the drawing.

<a id="rg9-12-11"></a>
### RG9.12.11 — truncated cones: ObjectContactConeCone and ObjectContactConeTriangle (2026-11-07, #3332)
- ConvexShapeType::Cone (radius at -L/2, radius1 at +L/2, one may be 0): support of the farther rim, the closest point
  in the trapezoid of the half-plane, the refinement of curved sides as the cylinder; cUtilities.ConvexShape('Cone',
  radius, length, radius1=...). The Cylinder struct of the cylinder contacts carries the radius of each end, so the
  face points (deepest rim point, end of the side line, two rim points; none at a tip) serve the cones as well.
- Cone on cone at the side: along the side line of cone 0 the signed distance to cone 1 (exact, by the trapezoid);
  convex along the line, so its minimum by golden section and the ends of the penetration by bisection, the minimum
  carrying the penetration beyond the line of the ends. Found while making it run (the lying cone on a cylinder
  failed at the first contact, Newton stalled at a residual of 1e-2..1e-5): (1) the half-plane from the closest point
  of EPA is not exact (noise 1e-6 N); (2) from the closest points of the two axes it jumps where the axes intersect -
  a rotation of 1e-8 gave 5300 N; now through the point of the axis of cone 1 closest to the center of cone 0, smooth;
  (3) the zero gap of the ends switched by +-zeroGap, a jump of 1e-5 N inside Newton: now a continuous offset
  gap + zeroGap, also for the cylinder sides (cylinderContactTest 9.8466099132801 -> ...2201).
- coneContactTest.py (new, the SolutionViewer shows five cones): Distance against cUtilities.Distance 2.6e-16;
  standing, upside down on a triangle, on a fixed cone - L/2 - m g/(3k) to 1e-17; lying on a side line on the plane and
  on a cylinder - the line sinks at the center by m g (x1^2 + x2^2)/(k (x2 - x1)^2), the ends' penetrations balancing
  the moment of the weight (my first expectation m g/(2k) was 5e-6 off) - to 1e-8. DrawCylinder(radiusEnd) draws them
  (#3330), contact.showCones and colorCones.

<a id="rg9-12-6"></a>
### RG9.12.6 and RG9.12.7 — ObjectContactConvexConvex with the contact points of faces (2026-11-07, #3331)
- points0, triangles0, points1, triangles1 as numpy arrays (NumpyMatrix, NumpyMatrixI); CheckPreAssembleConsistency:
  rows of 3, indices in range, triangles with area, every point inside every triangle plane within 1e-8 of the size
  (convex, outward normals). GJK and EPA on ConvexShapeType::Points reading the rows of the matrix without a copy
  (ConvexShape.pointRows).
- The points: the face of shape 1 whose outward normal is -n (triangles with this normal within 1e-6), the points of
  shape 0 over it (projected into one of its triangles); the face of shape 0 with the normal n, the points of shape 1;
  coincident points of both skipped; of more, the 8 deepest, ordered by point index so that a point keeps its data
  block (24 coordinates); else the closest point. The shared code of the points (force, generalized forces, outputs,
  post Newton) moved into ContactPrimitivePoints.h, used by the cylinder, cone and convex contacts.
- convexConvexContactTest.py (new, five cubes): Distance against cUtilities.Distance 5.6e-17; a cube on a box, dropped
  tilted, on a small box: 0.3 - m g/(4k) to 1e-16; sliding 0.169895 against v0^2/(2 mu g) = 0.169895; the user
  function against the law 1.5e-13. My first file name, convexContactTest.py, overwrote the tracked
  ConvexContactTest.py on the case-insensitive file system; restored from git.

<a id="rg2-12-2"></a>
### RG2.12.2 — the test models created from 2026-09-20 show their simulation (2026-11-07, #3322)
- Maintainer 2026-11-07: the models created from 2026-09-20; the point is the animation in the SolutionViewer; a
  model of several cases names the case in renderWindowString or is left out. Evaluated in
  docs/revision/testModelViewerReview.md (184 models: 41 + 7 to show, 75 + 47 to leave, 14 shown already);
  approved with "do the suggested next steps".
- Done by a subagent, checked here by the full test suite: 43 models changed, each run in test mode before and after
  (testResult identical to the last digit) and run directly with the SolutionViewer replaced by a probe that loads
  the solution file and counts the shapes of GetGraphicsData (bodies draw something in every one).
- Beyond the recipe: constraintDriftTest, initialConstraintsCheckTest and ANCFBeamGeometricallyExactComparisonTest
  create one SystemContainer per case, so that the viewer does not draw all cases on top of each other; some models
  write the solution only for the shown case; Mass1D got a referencePosition in springDamperVelocityOffsetTest and
  coSimulationTest so the masses are drawn apart; contactSphereTorusMomentumTest and generalContactTriangleMomentumTest
  a file.writePeriod for more frames. coSimulationTest shows the one-system reference (the co-simulation writes only
  mbs0).
- Skipped: ANCFThinPlateRotationTest (10 ms, 9 mm on 1 m), implicitRungeKuttaLieGroupTest (5 ms, the subject is the
  Newton iterations), multithreadingUserFunctionTest and symbolicUserFunctionThreadsTest (long chains moving
  centimeters, the subject is the threads), symbolicUserConnectorsTest (put back: the visualization user function of
  CreateCartesianSpringDamper of the UserItemCreator raises "Invalid OutputVariableType 'RotationMatrix'" for a mass
  point marker - raised as its own issue).

<a id="rg9-12-15"></a>
### RG9.12.15 — the impact model as the enumeration type ContactImpactModel (2026-11-07, #3327)
- The open question (a number still accepted?) was not answered; decided as the least change for users: the
  parameter takes exu.ContactImpactModel or the integer 0, 1, 2, converted in C++ by py::cast<Index> as
  providedMarkerType of MarkerBodyUser takes exu.MarkerType or an int (#3103); GetObjectParameter and GetObject give
  the enumeration. So nothing is deprecated, and the many models with impactModel=2 run unchanged.
- definitions/enumTypes.py: ContactImpactModel, Linear 0, HuntCrossley 1, GonthierCarvalhoMartins 2;
  definitionTypes.py: TContactImpactModel with the default ContactImpactModel::Linear; itemHeaderEmitter.py: the
  conversion. The 7 contact items (ContactCoordinate, SphereSphere, SphereTorus, SphereTriangle, SphereCylinder,
  CylinderCylinder, CylinderTriangle) with one description; the C++ compares with the names; the checks at Assemble
  still refuse a value outside 0..2 (checked: SetObjectParameter(o, 'impactModel', 5) is refused at Assemble).
- CreateSphereSphereContact and the two other Create functions with impactModel: default ContactImpactModel.Linear,
  accept the enumeration or the int. contactSphereSphereTest.py uses the name; parameterConversionTestReference
  re-recorded.

<a id="rg9-12-17"></a>
### RG9.12.17 — the faces of the cylinders and cones drawn in C++ (2026-11-07, #3330)
- Maintainer 2026-11-07: the cylinders of sphereCylinderContactTest.py have flipped faces; use the existing cylinder
  function; a cone drawing for the cone contacts, as an extension of the C++ cylinder. The item already used
  EXUvis::DrawCylinder - the function itself is wrong: from GetGraphicsData, all 64 triangles of a contact cylinder
  wound inward while their normals pointed outward (graphics.Cylinder: 128 of 128 outward). The basis of
  ComputeOrthogonalBasisVectors is right-handed and the points run clockwise about the axis; DrawCone the same. Every
  C++ cylinder and cone was affected: spring-dampers, the hydraulic actuator, the rolling disc, arrows, frame axes.
- Both functions add each triangle with the winding of its outward normal (points 1 and 2 exchanged), the points and
  normals unchanged, so that angleRange keeps its meaning; the graphics regression fingerprints did not change.
- DrawCylinder(..., radiusEnd): the radius at pAxis0 + vAxis, a truncated cone (0: pointed, without end face), the
  smooth normals tilted by (radius - radiusEnd)/L along the axis; for the cone contacts of RG9.12.11.
- test_graphicsData.py: the triangles of a contact cylinder and of the 3D axes of a node wind as their normals point.

<a id="rg9-12-5"></a>
### RG9.12.5 — ObjectContactCylinderCylinder and ObjectContactCylinderTriangle, with contact points (2026-11-07, #3329)
- First with one contact point, as the design said: crossed cylinders, a rim on a side and a tilted cylinder on a
  plane without friction ran; a roller on the plane or the triangle (with and without friction, tilted or not), a disc
  cap on a lying cylinder, two parallel cylinders failed within 0.2 s ("discontinuous iteration was not successful"):
  the deepest point jumps between the ends of the line. Asked; the maintainer chose the contact points now.
- Up to 4 points, a block of 3 data coordinates each (12); src/ImplObjects/CObjectContactCylinderCylinder.cpp holds
  both items on one point generator (namespace ContactCylinder): a cylinder on a face by the deepest rim point, the
  other end of its side line (clipped to the face), two rim points at +-120 degrees (MuJoCo's plane-cylinder scheme),
  rim points beyond the face moved to its border along the cap radius; side on side by the closest axis point and the
  ends of the interval of penetration, the closest point carrying only the penetration beyond the line of the ends.
  Three things the probes found: (1) HomogeneousTransformation::GetRotation() returns a copy, so the frames given to
  GJK were the identity - parallel cylinders came out 0.3 deep; the frame is now built from the marker. (2) The ends
  of a penetration interval and the closest point of parallel sides have the gap 0 by construction, and its rounding
  switched the contact state in every PostNewton; a gap within 1e-10 (r0 + r1) of zero counts as apart. (3) A
  standing cylinder offset over the border of a cap: the radial direction of the points jumped from the marker's x
  axis to the tilt direction at the first tilt, and Newton diverged; it now goes over continuously below a tilt of
  1e-3 rad (share eps^2/(eps + tilt)), and the Distance of the plane is the exact support point, not the first point.
- Probes, all with k = 1e5: disc cap on side 0.25 - m g/(2k), crossed 0.3 - m g/k, parallel 0.3 - m g/(2k), stacked
  caps 0.3 - m g/(3k), skew 0.3 rad one point, parallel rolling and sliding with friction, roller on plane and on
  triangle 0.1 - m g/(2k) and 2/3 v0, offset stacked caps (tips over, runs).
- ConvexShape keeps the 3 points of a triangle in a fixed array (no allocation in the items); PyConvexShape fills it.
  ContactPrimitiveForce.h said "force on marker 1"; it is the force on marker 0 (#3328).
- cylinderContactTest.py (new, the SolutionViewer shows six bodies): Distance of deactivated contacts against
  cUtilities.Distance 0.0 and against the plane 6e-17; the roller 0.66602 (2/3 v0 within the regularized friction),
  heights within 6e-9 of r - m g/(2k), L/2 - m g/(3k) (standing on a triangle, and a tilted drop), r0 + r1 - m g/k,
  r0 + r1 - m g/(2k); the user function against the law 4.3e-12. symbolicUserFunctionSignaturesTest 2 more
  signatures (60.295 -> 62.295); parameterConversionTestReference re-recorded.

<a id="rg9-12-16"></a>
### RG9.12.16 — the sign of the output Torque of the sphere contacts (2026-11-07, #3328)
- Found while setting the conventions of RG9.12.5: a ball sliding in +x on a triangle, a large sphere and a large
  cylinder, one step of 1e-5 s - Force [-2.943, 0, 9.81] (friction -x, normal +z: the force on the ball, marker 0),
  the ball's angular velocity +y, but Torque -0.2942 about y. The output was ((-(r + g/2)) n0) x f, the
  contribution to the left-hand side that ComputeConnectorForceRigid gives for marker 0; the torque on marker 0 is
  ((r + g/2) n0) x f. The simulation uses ComputeConnectorForceRigid and was right.
- Corrected in the four items; the description of SphereCylinder named the forces on the two markers the other way
  round, corrected. sphereCylinderContactTest.py: the sliding ball has a Torque sensor, +0.4902 about y at t = 0.02
  against mu m g r = 0.4905 (old build: -0.4902); 2 checks more (6.614 -> 8.614).

<a id="rg2-12-3"></a>
### RG2.12.3 — the contact items of the contact test models drawn (2026-11-07, #3326)
- Maintainer 2026-11-07: contactDistanceOutputTest.py showed nothing, sphereCylinderContactTest.py the ball but no
  cylinder. The contact.show... settings draw only the contact items whose visualization has show=True, and the
  sphere contacts have show=False by default; checked with SC.renderer.GetGraphicsData(): show=True adds the sphere
  and the 64 triangles of the cylinder, the settings alone add nothing.
- contactDistanceOutputTest.py, sphereCylinderContactTest.py, sphereTrianglePlaneTest.py, contactUserFunctionTest.py:
  the contact items with visualization show=True; their results unchanged. The descriptions of contact.showSpheres,
  showTriangles, showTori, showCylinders and the rule in WORKFLOW.md name the condition.
- The all-checks of exudev generate had stopped at checkDescriptions since RG9.12.4 (the three sphere contacts
  changed by the Distance output, #3324 and #3325; their descriptions were updated then, now marked), and behind it at
  the stubs (cUtilities.ConvexShape, a pybind class, into the stubtest backlog as KinematicChain) and pydoclint (the
  DOC108 of the two new user function parameters, deliberate per #2664; baseline regenerated). All 19 pass.

<a id="rg9-12-14"></a>
### RG9.12.14 — the contact force user function; the type names of two sphere contacts (2026-11-07, #3324, #3325)
- contactForceUserFunction(mbs, t, itemNumber, gap, gap_t, normal, tangentialVelocity, impactVelocity) of
  ObjectContactSphereTriangle and ObjectContactSphereCylinder, the two items on the shared law. Changed against the
  design (section 8.5): the normal is an argument and the result is the global force on marker 0 - the convention
  of the output Force, checked: a ball at rest gives +m g - instead of [normal, tangential 1, tangential 2], which
  would need tangential axes the items do not have; dataCoordinates left out, the impact velocity is the one value
  of them a law needs. Called in contact as the data node says; the friction part (for the torque) is the tangential
  part of the result; dynamicFriction > 0 still switches on the rotation of marker 0 and the friction torque, as it
  needs a rigid marker 0.
- The symbolic version came from the generator without further code (PySymbolicUserFunctionSet.h, one member for
  the shared signature); symbolicUserFunctionSignaturesTest.py counts 2 more signatures (58.295 -> 60.295).
- #3325 found on the way: CreateSymbolicUserFunction looked up ObjectContactSphereSphere for the triangle -
  GetTypeName of ObjectContactSphereTriangle and SphereTorus returned ContactSphereSphere, copied from the
  sphere-sphere item; mbs.GetObject(i)['objectType'] was wrong for both, and a dictionary of such an item added
  again created a sphere-sphere contact. Corrected in the definitions.
- contactUserFunctionTest.py (new, with the SolutionViewer): six balls dropped with a horizontal velocity onto
  cylinders and triangles with friction; the Python user function of the built-in law against the law: 3e-12 on
  the cylinder, 2.8e-5 on the triangle - the built-in law takes the stick/slip regime of the data node, the user
  function the current one, which differ in the steps where it switches -, the symbolic against the Python user
  function 1.5e-13; the ball rolls at 0.714 = 5/7 v0.

<a id="rg9-12-10"></a>
### RG9.12.10 — infinitePlane of ObjectContactSphereTriangle (2026-11-07, #3323)
- Maintainer 2026-11-07: the decisions of contactPrimitives.md section 8.6; the clear steps first (8.1 and 8.5 with
  the first items).
- The parameter infinitePlane (False): the plane of the triangle with its normal by the order of the points
  (EGeometry::ComputeTriangleNormal, as the drawing), the signed distance of the center, the normal from the sphere
  to the plane fixed to minus the triangle normal, so that a center behind the plane is pushed back to the front;
  includeEdges ignored. One cross product per evaluation instead of the point-triangle distance.
- sphereTrianglePlaneTest.py (new, with the SolutionViewer): the distances of deactivated contacts beside the
  triangle (to the plane 0.15, to the edge 0.392) and behind the plane (-0.15, normal -z), exact; two balls sliding
  over the edge of the triangle with friction: on the plane the ball rolls on at r - m g/k, on the triangle it falls
  off. parameterConversionTestReference.txt recorded.

<a id="rg9-12-4"></a>
### RG9.12.4 — ObjectContactSphereCylinder (2026-11-07, #3320)
- The definition as ObjectContactSphereTriangle: sphere at marker 0, the cylinder centered at marker 1 along its
  local z axis (`cylinderRadius`, `cylinderLength`), the contact law, friction and impact models, the numerical
  Jacobian of the rigid connector interface; 3 data coordinates (gap, tangential velocity, impact velocity) as
  decided in contactPrimitives.md section 3.3, without the unused fourth one of the sphere contacts.
- The geometry by EGeometry::DistanceSphereCylinder (#3317): the regions side, cap, rim and inside; the normal from
  the sphere towards the cylinder, also for a center inside, where the vector to the closest point would point the
  wrong way.
- The contact law moved out of ObjectContactSphereTriangle into src/ImplObjects/ContactPrimitiveForce.h, a template
  on the parameter structure, used by both; SphereTriangle keeps its member as a call of it - the suite unchanged.
- contact.showCylinders and contact.colorCylinders (visualization settings) for the drawing.
- MiniExample (a ball at rest on a lying cylinder, 0.3 - m g/k); sphereCylinderContactTest.py (new): the Distance
  of a deactivated contact against cUtilities.Distance on a tilted cylinder for side, cap, rim and inside (exact),
  at rest on cap and side (1e-8), a ball sliding along the top with friction 0.5 rolls at 5/7 v0 (vx 0.71399,
  omega r 0.71469). parameterConversionTestReference.txt recorded with the new item.
- Not in this step: a Create function (RG9.12.8 with the examples), the contact force user function (RG9.12.14).

<a id="rg9-12-3"></a>
### RG9.12.3 — the output variable Distance of the sphere contacts (2026-11-07, #3319)
- OutputVariableType.Distance exists (bit 0, the connectors' distance); ObjectContactSphereSphere, SphereTriangle
  and SphereTorus give their gap under it, the same value as DisplacementLocal. GetOutputVariableConnector of the
  three computes the geometry independently of activeConnector (only the forces and the data node skip it), so the
  distance of a deactivated contact is there without a change.
- contactDistanceOutputTest.py (new): a ball falls past an inactive sphere, triangle and torus contact (the torus
  contact is for a sphere inside its tube, so its distance is negative there) onto an active sphere: the distances
  against closed forms (1e-16), the fall unaffected by the inactive contacts (1.6e-15), at rest Distance equal to
  DisplacementLocal, -m g/k.

<a id="rg9-12-2"></a>
### RG9.12.2 — the distance kernels of convex shapes: GJK, EPA, exudyn.cUtilities.Distance (2026-11-07, #3317)
- Maintainer 2026-11-07: the design of contactPrimitives.md decided (#3315), the RG9.12 steps first.
- src/Linalg/GeometryConvex.h (new): ConvexShape (Sphere, Capsule, Cylinder, Box, Triangle, Points) with support
  functions of the core - the sphere a point and the capsule a segment, their radius a margin -; GJK with the closest
  point of a simplex by its Voronoi regions (Ericson 5.1.5), EPA from the simplex and the supports along the axes
  (the polytope of the largest tetrahedron, a new point removes the faces it sees connected to the one it sees most,
  rejected if a new face would be degenerate); a flat Minkowski difference (two cores without volume) gives the
  depth 0 and its plane normal. The analytic sphere-cylinder distance as check and fast path.
- Measured on random pairs of all 36 combinations: separated distances to rounding (1e-14) for every pair except
  those with a cylinder. GJK stalls on the side of a cylinder - its points are combinations of the two rims at
  slightly different angles, and the gain of a step is quadratic in the gap - leaving errors of 4e-8 at distances of
  0.08; alternating projections of the closest points onto the cores (closed forms per shape, GJK for a point set)
  give the rounding. EPA on a cylinder ended at 128 iterations with depth errors up to 0.15; with 1000 iterations
  the polytope broke up (faces that see a point within the tolerance left holes) and the run did not end - hence the
  connected visible set and the rejection of degenerate faces. A penetration with a cylinder is refined by moving
  shape 1 out by (d + delta) n: the closest point p of C = A - B to T with its normal m is a point of the boundary,
  p.m the support value of C in m and so an upper bound of the depth; the smallest bound kept, converging linearly
  with a factor of about 0.3.
- Remaining: with a cylinder, within about 1e-6 of contact (separated or penetrating) the distance to about 1e-7 of
  the size, as the alternating projections converge slowly there; the normal at the side of a cylinder to 1e-7.
  Fine for the prototypes; the contact items of a cylinder (RG9.12.4/.5) will show whether it matters.
- 15 us per call on average. exudyn.cUtilities.ConvexShape and Distance (src/Pymodules/PyConvexShape.h, bound in
  definitions/pybindCUtilities.py); convexDistanceTest.py (new): closed forms (boxes, crossed capsules, parallel and
  crossed cylinders, sphere-triangle), the analytic sphere-cylinder against GJK/EPA (4e-16), random pairs moved to
  contact (3e-8) and 0.01 apart (1.5e-12), refusals.

<a id="rg20-4-4"></a>
### RG20.4.4 and RG20.9.12 — the interpolated output on the solver page, the study figures revised; LobattoIIIC4GGL evaluated (2026-11-06, #3310, #3311, #3312, #3313)
- Maintainer 2026-11-06: no information on the interpolated output of RG20.4.2 found; no real advantage of
  LobattoIIIC4GGL seen - keep it?; why the multipliers of rigidBodySpringDampers do not converge, also with
  generalized-alpha; the efficiency figure split into constant and automatic steps, with larger ranges.
- #3310: solver.md, section Output of the solution and the sensors (sec-solver-output) in General solver structure:
  the write times, the Hermite polynomial of the ODE2 coordinates, the linear interpolation of the others and of Lie
  group nodes, the data coordinates of the end, automatic steps, the static solver, the symbolic solvers.
- #3311, the multipliers: the algebraic coordinates of rigidBodySpringDampers are the multipliers of the three Euler
  parameter constraints, and they are zero in the exact solution - the mass matrix G^T J G and the forces G^T (...)
  have no component along theta (G theta = 0), so the row theta^T of the equations of motion gives 2 |theta|^2 lambda
  = 0. The reference was 0 to 1e-12, divided by max(norm, 1e-14): 1 to 1e4 for every solver. A group whose
  reference has an L2 norm below 1e-10 is left out of the figures now (in Plot, the stored data unchanged).
- #3311, the efficiency: efficiency_models.png with constant steps, efficiency_adaptive.png (new) with the tolerances
  1e-2 ... 1e-10, both 2x2 and 4.2 x 3.4 in per subplot; the automatic-step runs computed again. The global error at
  the end lies 10 to 200 times above the tolerance; at 1e-2 to 1e-4 the largest step (a tenth of the time) limits.
- Found with it: LobattoIIIA4 on sliderCrank3D stays at an error of 1e-6 to 1e-7 for the tolerances 1e-5 to 1e-10
  (21 to 106 steps), while 106 constant steps give 1e-10 - #3313 (RG20.8.3).
- LobattoIIIC4GGL, measured (velocities against LobattoIIIC4GGL with h0/64, constant steps): rigidBodySpringDampers
  (stiffness 1e5, the stiff period about 16 ms) at 16 h0 = 0.032 s: LobattoIIIA4 1.0e-5, LobattoIIIC4GGL 5.8e-5,
  Radau3 1.2e-3, generalized-alpha 8.8e-2; FFRFreducedOrder at 4 h0 to 8 h0: LobattoIIIC4GGL 2 to 3 times more
  accurate than LobattoIIIA4, at h0 equal; CPU time 2.5 to 3 times that of LobattoIIIA4. Its L-stability does not pay
  off on these models: LobattoIIIA4 is not damped but stays accurate at steps above the stiff period. What it adds:
  the multipliers without the step-to-step oscillation of LobattoIIIA4 (the reference of the study) and the
  constraints at every stage. Radau3 is L-stable too, at a third of its cost. Recommendation to the maintainer:
  remove it from SolveDynamic (unreleased, no deprecation), the reference of the study by Radau3 at small steps.
- Found with it: about 10 memory allocations per step of the Runge-Kutta solvers on rigidBodySpringDampers (16000
  steps, the warning of the solver), none with generalized-alpha - #3312 (RG20.7.12).

<a id="rg20-9"></a>
### RG20.9 closed, the fast-module checkpoint; RG11.8.20 superseded; the open robotics tasks (2026-11-06, #2993)
- Maintainer 2026-11-06: which RG19 tasks are open, the design note revised; why RG20.9 is still open.
- RG20.9 was left open for the checkpoint of #3296 at the completion of a group; its eleven sub-steps were closed.
  Closed now, with the checkpoint run at V1.13.427: the fast module built (2 min 14 s); its suite 9 of 294 failed,
  all rounding or stale entries - modelLibraryTest and rollingCoinTest kept AVX2 references from before #3304 and
  #303 (their fast results equal the regular references to 1.2e-13 and 1.3e-14; rollingCoinTest needs no entry any
  more), the others drift by 5.6e-14 to 2.8e-11 against their entries. recordAVX2References.py --write: 10 entries
  updated, rollingCoinTest removed; the fast suite (also --parallel) and the regular suite pass. Performance tests
  pass with both modules: 47.9 s regular (46.0 s at V1.13.418), 39.0 s fast.
- RG11.8.20 (#2993) closed as superseded by #3269: the deprecated InverseKinematicsNumerical builds a KinematicChain
  and runs its LM IK; a Puma560 of models.py built without PDcontrol reaches its tool HT to 2e-13 and keeps its
  gravity.
- docs/dev/design/robotics.md: section 1 checked against the code - every task still applies; spotModel.py still loads
  spotData.h5 relative to the working directory (fixed in one line by the pattern of #3303, independent of task 1),
  humanRobotInteraction.py still reads an absolute path; which tasks depend on task 1, an order for when the group
  resumes, #2993 in the table of what is done. RG19 stays paused.

<a id="rg20-9-11"></a>
### RG20.9.11 — the ODE2 velocities in the convergence figures (2026-11-06, #3309)
- Maintainer 2026-11-06: the ODE2 velocities also in the convergence plots, arranged 4x1 or 2x2.
- solverConvergenceFigures.py: the group ODE2_t (GetODE2Coordinates_t) with its own Richardson reference; the
  subplots of a model in one row (three for the constrained models, two for kinematicTree), 2x2 for genericDAE, the
  only model with all four groups, with its title in the space tight_layout leaves; the mean figure 2x2. A run computes
  a stored model again whose reference lacks a group.
- The whole study again (25 min). The velocities converge with the orders of the coordinates between h0 and h0/4:
  generalized-alpha and TrapezoidalGGL 2.0 to 2.06, Radau3 2.82 to 3.02, LobattoIIIA4 4.0 to 4.29, except on
  sliderCrank3D, whose Runge-Kutta errors are at the floor. The mean efficiency: LobattoIIIA4 at 2 h0 has a 600 times
  smaller error than generalized-alpha at h0 (before 500; the CPU times of a new run) for 1.55 times its CPU time.
  solver.md says so.

<a id="rg20-9-10"></a>
### RG20.9.10 — the stage Jacobian by Euler parameters against finite differences (2026-11-06, #3308)
- Maintainer 2026-11-06: do RG20.9.10.
- No defect. The Jacobian of an item by Euler parameters is the derivative along the rotations, -A p~ G for a point p
  of the body: G theta = 0, so it has no component along the parameters themselves. The rotation matrix of
  EP2RotationMatrix is quadratic in the parameters and changes along them (by 2 (A - I) p per unit length). The
  finite differences of CheckJacobian perturb one parameter, mostly along theta for theta0 near 1, and contain that
  component. Checked with numpy at a rotation of 0.3 about z: the two Jacobians times the projector I - theta theta^T
  agree to rounding; along theta the G form gives 0, the quadratic form (-0.047, 0.114, 0); in the column of theta0
  the entries are (0.032, -0.053) against (-0.015, 0.060), and at small angles they are the negatives - the entries
  of the issue (rigidBodyJoints -6.26574e-5 against 6.26575e-5, FFRFreducedOrder -1.74388e-5 against 1.74388e-5).
- Left as it is: the constraint of the Euler parameters (its row 2 theta^T) fixes the increments along theta, which
  are driven by its residual alone and vanish after the first iteration; the reaction forces C_q^T lambda use the same
  G, so the matrix stays the transposed pair of the constraint rows. LobattoIIIA4 needs 1.9 Newton iterations per
  step on rigidBodyJoints with the step 1e-5. Generalized-alpha uses the same Jacobians.
- A comment in CSolverImplicitRungeKutta::CheckJacobian names the expected differences.

<a id="rg20-9-9"></a>
### RG20.9.9 — the Newton floor of LobattoIIIA4 and Radau3 at small steps (2026-11-06, #3307, #3308)
- Maintainer 2026-11-06: do RG20.9.9.
- Located on FFRFreducedOrder with the step 3.125e-5 (verbose 3, the largest residual per block): the stage
  equations of motion of stage 1 stay at 8e-11 and jump by factors of 2 between iterations, stage 2 is at 1e-13, the
  position constraints at the end at 1.1e-16 - one rounding of the coordinates, which the row scale 1/h^2 makes 1e-7
  in the scaled system. Each iteration therefore moves the stage accelerations by about eps |q| / h^2, and the
  Jacobian of the end of the step, used at all stages, misfits stage 1 by O(h) (the check of verbose 4: differences
  of 7e-4 in A1/A1 at h, 3.5e-4 at h/2): a residual of eps |q| / h, the floor doubling as the step halves. The
  differentiation epsilon changes nothing (1e-5 and 1e-9). Generalized-alpha has the same rows but starts from a
  residual 1e4 times larger, so its relative tolerance ends the iteration above the floor; the good predictor of the
  Runge-Kutta solvers (the accelerations of the start) makes the relative tolerance demand 3.5e-12.
- The fix: in ComputeResidual a position constraint residual (the rows at the end, and the position rows of GGL
  that are not velocity-level or equations of a multiplier) below 16 eps max|q_ref + q0| is zero. Considered and
  left: the Jacobian per stage (the mass matrix and C_q at each stage, the cost of a Jacobian times the stages, and
  the increments would remain), and a test of stagnation in the Newton loop (a heuristic on the contraction).
- Without the fix, with the step 1e-5 and the Newton tolerances 1e-10 / 1e-12, LobattoIIIA4 and Radau3 fail in the
  first steps on nPendulum, rigidBodyJoints and sliderCrank3D (1.6e-5); LobattoIIIC4GGL and TrapezoidalGGL run. With
  the fix all run with the constant number of steps, on FFRFreducedOrder down to 7.8e-6. genericDAE ran in both
  builds over 0.1 s, so it is not in the test.
- implicitRungeKuttaSmallStepTest.py (new): nPendulum and rigidBodyJoints, 1e-5, 0.02 s, both solvers, 8004 steps.
  No reference of the suite changed.
- The study: the automatic-step runs again (--adaptive); LobattoIIIA4 on sliderCrank3D with the tolerance 1e-8 had
  failed and runs (43 steps); efficiency_models.png changed in that point.
- #3308 raised (RG20.9.10): the Jacobian check shows single entries of the constraint rows by the acceleration of the
  first Euler parameter with the opposite sign, which do not shrink with the step (rigidBodyJoints, FFRFreducedOrder).

<a id="rg20-9-7"></a>
### RG20.9.7 and RG20.9.8 — the convergence and efficiency figures, the solver page (2026-11-06, #3305, #3306)
- Maintainer 2026-11-06: the description of the new solvers in the solver section, specifics and error estimators;
  log-log convergence plots of all solvers on the library models, few explicit, a selection for the implicit ones, a
  mean of the errors normalized by the L2 norm per group of coordinates, the groups ODE2, ODE1 and AE in subplots,
  about 15 figures; a typical efficiency diagram, accuracy against CPU time.
- python/testing/solverConvergenceFigures.py (maintainer tool, not in the suite): 8 models (beam3D left out - its
  errors stay between 1e-6 and 1e-3 without converging smoothly at these steps), steps 2 h0 ... h0/16, tolerances
  1e-3 ... 1e-8 for the automatic step size (the largest step a tenth of the run, the first h0), Newton tolerances
  1e-10; errors at the end per group relative to the L2 norm of the group of the reference; CPU time the minimum of
  three runs; the data in docs/figures/solverStudy/solverStudy.json, --replot and --adaptive. About 25 minutes.
- The reference: first LobattoIIIA4 at h0/128 with Richardson - the multipliers of all solvers then stalled at 8e-6
  on genericDAE: LobattoIIIA4 does not damp the multipliers, an oscillation from step to step that does not vanish
  with the step; LobattoIIIC4GGL (L-stable) is the reference now, RK67 without constraints. LobattoIIIA4 and Radau3
  stop at small steps on FFRFreducedOrder with a growing Newton floor (#3307, RG20.9.9).
- Found on the way: #3304 (RG20.9.6). The first study ran with the largest step of the automatic step size equal to
  h0, so the adaptive runs could not take larger steps; now a tenth of the end time.
- Results (docs/manual/solver.md, section Convergence and efficiency): orders between h0 and h0/4 - generalized-alpha
  1.95-2.0, TrapezoidalGGL 1.95-2.0, LobattoIIIA4 3.99-4.07 (4.36 near the floor), Radau3 2.87-3.01, multipliers 2;
  CPU per step against generalized-alpha 2 (TrapezoidalGGL), 2.9 (LobattoIIIA4), 3.0 (Radau3), 9.0 (LobattoIIIC4GGL);
  LobattoIIIA4 at 2 h0 has a 500 times smaller error than generalized-alpha at h0 for 1.6 times its CPU time.
- solver.md: the overview table of the dynamic solvers, the section Implicit Runge-Kutta solvers (stage equations,
  tableaus, the constraints at the end against GGL, unknowns per step, error estimates oneStep and twoSteps and why
  one step cannot estimate order 4), the section Convergence and efficiency with the 11 figures.

<a id="rg20-9-5"></a>
### RG20.9.5 and RG20.9.6 — the restart of the modified Newton method with ODE1 coordinates; the velocity constraint of genericDAE (2026-11-06, #3300, #3304)
- Maintainer 2026-11-06: do RG20.9.5; the description of the new solvers and the convergence figures (RG20.9.7,
  RG20.9.8).
- #3300, the cause: with relativeTolerance 1e-12 the modified Newton method of generalized-alpha restarts a step 14
  times in 500 steps of genericDAE (bad contractivity). The restart (CSolverBase::Newton) set the coordinates back to
  the start of the step but not the ODE1 derivatives, assuming the predictor did - and the predictor adds h/2 times the
  current ODE1Coords_t, which were those of the rejected iteration: an error of about h^2/2 y_tt per restart. Now the
  restart restores ODE1Coords_t from the start of the step (not in the static solver). Modified against full Newton
  at h = 1e-3: 3.6e-6 before, 5e-15 now; generalizedAlphaODE1OrderTest.py checks it (3.6e-14).
- #3304, found with the convergence figures: LobattoIIIA4 and Radau3 diverged on genericDAE within 0.05 s at every
  step (the Newton residual growing tenfold per step), TrapezoidalIndex2 failed at h0. The constraintUserFunction of
  the model ignored velocityLevel and gave the position constraint also as velocity constraint; a mass point on a
  DistanceConstraint with an ObjectGenericODE1 lag ran with all solvers, which located it. The user function returns
  2 (x0 + q0) q_t0 + 2 (y0 + q1) q_t1 for velocityLevel now; all six implicit solvers run.
- References: modelLibraryTest (both changes), generalizedAlphaODE1OrderTest (the new check).

<a id="rg2-11"></a>
### RG2.11 — the examples and test models find their data beside their file (2026-11-06, #3303)
- Found by grep for 'testData' on code lines: 16 test models (abaqusImportTest, compareAbaqusAnsysRotorEigenfrequencies,
  createFFRFReducedOrderObjectTest, linearFEMgenericODE2, NGsolveCMStest, the five objectFFRFreducedOrder*,
  objectFFRFTest, objectFFRFTest2, objectGenericODE2Test, superElementRigidJointTest, textureGraphicsTest,
  urdfRobotHDF5Test), 4 examples (netgenSTLtest, pymeshlabFileImport, texturesExample,
  publications/sliderCrankCMSacme) and perfObjectFFRFreducedOrder read their input by paths relative to the working
  directory - 'testData/...', or sideways '../Examples/testData/...' and '../TestModels/testData/...' so that they
  worked from the runners' directories; objectFFRFreducedOrderStressModesTest chose between two paths by testIsActive.
- Now os.path.join(os.path.dirname(os.path.abspath(__file__)), 'testData', ...). The runners give __file__: the test
  suite (ExecModel), the examples (#3284) and now runPerformanceTests.py, which set it for each model and restores its
  own. Left: NGsolvePistonEngine.py exports meshes to testData/ inside "if False", pendulumVerify.py defines an unused
  fileName, the notebook CMSexampleCourseJupyter.ipynb (run in its directory).
- Checked: the 16 test models, the performance model and two examples run with the working directory in the
  scratchpad, results unchanged; the suite, pytest, all 196 examples and the performance tests pass.
  NGsolveCMStest wrote its round trip to OutputFilePath(fileName + '2'), which refuses an absolute name: now
  OutputFilePath('testData/netgenTestMesh2'), the place it had.
- The performance tests (the checkpoint of #3299) found perfRigidPendulum off by 5.4e-10 relative since #303 (RG4.21):
  the differences of fourth order summed in another order, as the six test models there; recorded again.

<a id="rg5-1-5"></a>
### RG5.1.5 — LinkedDataVector cannot be copied (2026-11-06, #3302)
- LinkedDataVectorBase had the implicit copy constructor: it calls VectorBase(const VectorBase&), which allocates and
  copies (and counts a linkedDataVectorCast with runtime checks), and ~LinkedDataVectorBase sets data to nullptr, so
  the copy was never freed - an allocation and a leak per copy, the case of ObjectKinematicTree in #3291.
- Probe: the copy constructor declared deleted; the regular and the fast module (exudev build --fast) compile, so no
  copy is left in the code. Kept deleted, so that the compiler refuses every new copy. The suite and pytest pass.

<a id="rg20-9-2"></a>
### RG20.9.2 — the test models that repeat a model of the library (2026-11-06, #3301)
- Compared each model of modelLibrary.py with the test models of the same kind:
  - sliderCrank3D is sliderCrank3Dbenchmark.py, whose purpose is the benchmark itself (results file, timings, 5 s run
    directly): taking it from the library would hide the benchmark model; sliderCrank3Dtest.py is another variant
    (Euler parameters, other inertia of the crank, index 2, sensors).
  - genericDAE is one of the variants of genericDAEUserFunctionsTest.py (Python and recorded user functions, the mass
    matrix function, the coupled constraint, against built-in items); FFRFreducedOrder the pendulum of
    createFFRFReducedOrderObjectTest.py, which tests the arguments and refusals of CreateFFRFReducedOrderObject.
  - kinematicTree: kinematicTreeTest.py (five links, ObjectKinematicTree directly) and the other kinematic tree tests
    vary the links; nPendulum, rigidBodyJoints/SpringDampers, massOscillator, beam3D have no test model with the
    same model (MathematicalPendulumTest, energiesTest, restartFileTest, the beam tests use other parameters and
    items).
- Decided (#3301): a test model is changed only if its model is the library model itself; none is. Nothing changed.

<a id="rg20-9-3"></a>
### RG20.9.3 — the order of an ODE1 coordinate coupled to ODE2 coordinates in generalized-alpha (2026-11-06, #3258)
- Cause: the ODE1 coordinates are integrated by the trapezoidal rule y1 = y0 + h/2 (y_t0 + y_t1), and y_t0 of the first
  step was zero - the initial accelerations set only the ODE2 part, the ODE1 rows of their system are a unit matrix.
  The first step lost h/2 g(y0): for the lag of genericDAE (g(y0) = 2.82, decaying with exp(-t/0.2)) 0.116 h at 0.5 s,
  as measured (1.15e-4 at h = 1e-3), order 1.
- CSolverImplicitSecondOrderTimeInt::InitializeSolverInitialConditions sets ODE1Coords_t = F_ODE1 - C_y^T lambda0
  (lambda0 of the initial accelerations if computed, also when storeInitialAlgebraicCoordinates is False, else zero);
  exudyn.symbolics.solvers.SolveGeneralizedAlpha the same, so that it keeps agreeing with SolveDynamic (it had the
  zero start as well, "as in Exudyn").
- Order now 2 for both: the ratios of the differences between the steps 4e-3, 2e-3, 1e-3 are 3.998 to 4.000 for x, y
  and the ODE1 coordinate (spectral radius 0.6 and 1): generalizedAlphaODE1OrderTest.py.
- With modified Newton and relativeTolerance 1e-12 the ODE1 coordinate stayed erratic at 1e-6 (3.6e-6 off the full
  Newton method at h = 1e-3, the ODE2 coordinates 1e-14): #3300, RG20.9.5.
- References: 9 test models with ODE1 coordinates (ode1LoadsJacobianTest 8.8e-3 absolute, genericODE1JacobianTest
  1.3e-2, taskmanagerTest 1.2e-2, ...) and the mini example ObjectGenericDAE (0.0482 -> 0.0464, a short lag); the
  comparisons of SolveGeneralizedAlpha against SolveDynamic went from 1e-3 to 1e-13 (symbolicSystemGenericDAETest,
  genericDAEUserFunctionsTest, symbolicSystemODE1Test, symbolicSystemODE1AETest 1e-10). solver.md, revisions.md.

<a id="rg4-21"></a>
### RG4.21 — the initial accelerations with the constraints at velocity level and those depending on the time (2026-11-06, #303, #574)
- Maintainer 2026-11-06: do the suggested steps 1-5 (RG4.21, RG20.9.3, RG20.9.2, the copy of LinkedDataVector, the
  examples that depend on the working directory).
- Found: the right-hand side of the constraint rows had only (C_q v)_q v (#3289), and only with initial velocities.
  The velocity-level constraints (ObjectJointRollingDisc, CoordinateConstraint and CoordinateVectorConstraint with
  velocityLevel) were left out of it, so their rows read C_v a = 0 instead of C_v a = -(C_q v + C_t); the terms by the
  time (2 C_qt v + C_tt, C_t) were left out everywhere, so a prescribed motion from rest started without acceleration.
- CSystem::ComputeConstraintVelocityTerm now gives all terms: (C_q v)_q v as before; the others from the residuals
  G(sigma, tau) = g(q0 + sigma v, v, t0 + tau) (ComputeAlgebraicEquations at position level, which a velocity-level
  constraint answers with its velocity equation) by central differences of fourth order, 25 evaluations at the
  start: velocity rows G_sigma + G_tau, position rows 2 G_sigmatau + G_tautau. The differences in tau are taken of
  G - G(sigma, 0), so they are exactly zero for a constraint that does not depend on the time: no flag needed (#303
  asked for one), the rows of a velocity-level object from ComputeObjectJacobianAE (usesVelocityLevel). The step in
  tau is 1e-3 of endTime - startTime, at most 1e-3; the user functions are evaluated up to two steps before the start
  time (documented in solver.md). Called also without initial velocities.
- initialAccelerationsTest.py: a prescribed motion A(1-cos(omega t)) from rest, at position level (error 3e-11) and by
  offsetUserFunction_t at velocity level (1e-10); a tilted coin rolling on a plane: the accelerations after one step of
  h differ from the initial ones by 0.089 and 0.0089 for h = 1e-3 and 1e-4, i.e. O(h).
- References: rollingCoinTest 4.7e-3, createRollingDiscTest 4.7e-5, rollingDiscTangentialForces 1.4e-9 (C_q v of the
  rolling discs), genericJointUserFunctionTest 2.5e-11 (the offset by the time); ANCFbeltDrive, ballBearingTest,
  contactCurveExample, createFunctionsTest, computeItemTest, homogeneousTransformationParameterTest 1e-13 to 3e-9 (the
  same differences summed in another order). solver.md, the descriptions of CoordinateConstraint,
  CoordinateVectorConstraint, GenericJoint and computeInitialAccelerations no longer say the time is left out.

<a id="rg20-7-11"></a>
### RG20.7.11 — the performance tests with the regular module at the checkpoint of a completed group (2026-11-06, #3299)
- Maintainer 2026-11-06: yes to the proposal after RG20.7.9 - the performance tests are not in the gates, so
  perfRigidPendulum failed for one commit after #3289 unnoticed.
- WORKFLOW.md section 8 and the row "fast module" of section 5, the how-to of the plan: the checkpoint of #3296 runs
  `exudev perf` with the regular module as well. Run now: 10 performance tests successful, 46.0 s.

<a id="rg20-7-7"></a>
### RG20.7.7 to RG20.7.10 — user checks in every module, the fast module tested regularly, its performance references, the browser test (2026-11-06, #3295 to #3298)
- Maintainer 2026-11-06: do RG20.7.7 (the user checks the fast module compiles out), RG20.7.8 (the fast module
  tested regularly - not in every commit, but at least when the development of an RG or a significant step such as
  the symbolic solvers is completed), the performance references with the fast module, the flaky browser test.
- RG20.7.7 (#3295). tools/errorTriage.py: 2307 call sites, CHECKandTHROW 774 (208 with a user audience by its text
  rules, 122 unplaced), CHECKandTHROWstring 568 (339 user) - all compiled out of exudynCPPfast. Instead of rewriting
  them one by one: ReleaseAssert.h keeps in the fast module every CHECKandTHROW with an exception class (since #2528 a
  class says that a user can trigger the check; 184 sites) and every CHECKandTHROWstring (an unconditional throw,
  reached only on the error path); it leaves out CHECKandTHROW without a class (the invariants, the range checks of
  Vector and Matrix, 388 of them in src/Linalg) and CHECKandTHROWcond. CSystem::ComputeODE2SingleLoad declared two
  flags of a classed check only without __FAST_EXUDYN_LINALG - declared always now. Then the user checks without a
  class got one: 59 of the 152 user or unplaced sites of errorTriage outside src/Autogenerated (HT and Twist,
  ComputeItem, the HT output variable, the markers that provide no velocity, degenerate ANCF plates and slopes, the
  user functions of ObjectGenericDAE and ObjectConnectorUser, the indices of GeneralContact, NaN in the raytracer,
  ...); the rest are invariants (view numbers of the renderer, sizes in CSystem, temporaries); the 49 in
  src/Autogenerated are the local node numbers of GetNodeNumber and the connector interface, invariants as well. In
  the symbolic module, which errorTriage counts as internal by its path, 75 checks got a class (indices into named
  vectors and matrices, integer indices, names, esym.LookupTable, SetValue/SetVector, Diff, the user function
  arguments, item numbers); 8 left as invariants. CODING_STYLE.md section 10.1 says what the fast module keeps.
- Both suites pass (295 and 292 models, 99 mini examples); the performance tests with the fast module 36.4 s
  (regular 45.2 s; reference 39.5 s) - the kept checks cost nothing measurable.
- RG20.7.8 (#3296): WORKFLOW.md section 8 and the table of section 5, and the how-to of the plan: the fast module
  is tested when a group or a significant step is completed and before a release - test suite (serial and
  --parallel) and performance tests. tools/recordAVX2References.py records AVX2ReferenceSolutionUpdate from the
  newest _fast log (the script of #3294); checked: it reproduces the 46 entries of #3294 from the current log.
- RG20.7.9 (#3297): runPerformanceTests.py --fast-module: the AVX2 entry perfAccessFunctionsAD:cable-n200-implicit
  equals the fast result. perfRigidPendulum failed in both modules, 2.4735499208275304 instead of 2.473544662086979
  (5.3e-6 relative) - since #3289: a Rigid2D pendulum whose initial velocities do not fulfil the joint, a long
  implicit run, the term of the initial velocities by the central difference of fourth order instead of the forward
  difference; recorded again. The performance suite is not part of the gates, so the commit of #3289 did not see it.
- RG20.7.10 (#3298): DumpDom of test_issueTracker.py tries the headless browser three times for 60 s, each with its
  own profile, then skips the test with the reason, as for a browser that prints no DOM.

<a id="rg20-7-6"></a>
### RG20.7.6 — the fast module: user checks compiled out, the AVX2 references (2026-11-06, #3293, #3294)
- Maintainer 2026-11-06: fix the related problems; the AVX2 references were to be recorded again after RG20.7.2.
- exudynCPPfast built here (exudev build --fast, 2m15s). Its suite stopped at test model 152, symbolicFunctionTest,
  with a segmentation fault: esym.Function(t, [t + 1]) refused an input that is no named variable by CHECKandTHROW,
  which __FAST_EXUDYN_LINALG compiles out, and then dereferenced the null node. Run in parallel (each model in its
  own interpreter): 47 models and 2 mini examples failed - 10 not by rounding: symbolicItemOutputTest,
  symbolicDeepRecordingTest, symbolicVectorDiffTest, symbolicJacobianExpressionsTest, symbolicModuleTest (refusals
  that did not raise), symbolicVariableScopeTest (a TypeError after a missing refusal), symbolicSystemTest (access
  violation: GetNodeOutput of a system not assembled, whose refusal in RaiseIfNotConsistentNorReference was
  CHECKandTHROWstring), createFFRFReducedOrderObjectTest and modelLibraryTest (ImportError 'type Real is already
  registered': netgenTestMesh.npz holds a pickled exudyn.exudynCPP.OutputVariableType, whose unpickling imported the
  regular module next to the fast one).
- Fixed (#3293): the user checks the tests reach by PyError (CODING_STYLE.md section 10.1) - esym.Function (4), the
  sizes of the operands of every vector and matrix operation ('inconsistent', 26 in SymbolicVector.h,
  SymbolicMatrix.h, SymbolicTable.cpp), Diff of a component and of an unnamed variable, the derivative of an operation
  without one (2), Evaluate without the MainSystem, EvaluateJacobian (2), a named variable set or created while
  recording (2), a variable outside mbs.symbolicVariables; the five MainSystemData::RaiseIf... by ThrowPyErrorType,
  which throws without printing, as CHECKandTHROWstring did. exudyn/__init__.py: with the fast module loaded,
  sys.modules['exudyn.exudynCPP'] is that module. About 100 CHECKandTHROW of the symbolic module remain, not reached by
  a test; whether they check user input is not audited.
- Then the 40 models and 2 mini examples left differed from the regular values by 6e-14 to 4.8e-3 - stale entries of
  AVX2ReferenceSolutionUpdate (last recorded 2026-10-10) or none. Recorded again (#3294) from the log by a script:
  an entry for every model and mini example whose fast result differs from the regular reference by more than its
  tolerance, ordered by the difference - 46 entries, 12 new, 9 removed (their fast and regular values agree now); the
  entry of runPerformanceTests kept, not run here. The fast suite passes, in parallel and serially (292 models, 99
  mini examples), the regular one as well.

<a id="rg20-7-5"></a>
### RG20.7.5 / RG20.9.4 — ObjectKinematicTree without allocations, modelLibrary.py without deprecated arguments (2026-11-06, #3291, #3292)
- Maintainer 2026-11-06: fix the related problems.
- Measured with exu.special.InfoStat on the kinematic tree of modelLibrary.py, 100 against 200 steps: 28.9 vectors per
  step with generalized-alpha, 28.6 TrapezoidalIndex2, 26.6 LobattoIIIA4, 10 RK44, 4 ExplicitEuler, each one a
  LinkedDataVector copied into a Vector (linkedDataVectorCast_counts the same); per call of mbs.ComputeItem one for
  ODE2LHS and MassMatrix, 1 + n for each Jacobian - one per call of ComputeMassMatrixAndODE2LHS. The joint velocities
  were linked by LinkedDataVector q_t(qGiven_t ? LinkedDataVector(*qGiven_t) : node->GetCurrentCoordinateVector_t()),
  which MSVC copies through the copy constructor of VectorBase. Fixed (#3291): q and q_t declared empty and linked by
  LinkDataTo; 0 per step in all five solvers. The same pattern is nowhere else in src (a conditional in a constructor
  of a LinkedDataVector only for indices). test_kinematicTreeAllocations.py (generalized-alpha, RK44, LobattoIIIA4).
- modelLibrary.py (#3292): itemNumbers for bodyNumbers (4 calls), mass for physicsMass (2); modelLibraryTest gives
  the same result and no DeprecationWarning (run with -W error::DeprecationWarning). The other test models with
  bodyNumbers in a Create function are libraryDeprecationTest and createItemNumbersTest, which test the old name, and
  the rolling discs, whose bodyNumbers is not deprecated.

<a id="rg20-7-4"></a>
### RG20.7.4 — the order on the spatial slider crank: the initial accelerations, the stages of Lie group nodes (2026-11-06, #3289, #3290)
- Maintainer 2026-11-06: do the open steps, fix the related problems.
- Measured on sliderCrank3D of modelLibrary.py to t = 0.2, h = 2e-3 to 2.5e-4, against LobattoIIIC4GGL with h/64,
  Newton tolerances 1e-12: with Euler parameters LobattoIIIA4 2.0, LobattoIIIC4GGL 4, Radau3 3, TrapezoidalGGL 2;
  with Lie group nodes all four 2. The two methods without a stage at the start (IIIC, Radau) were right with Euler
  parameters, so LobattoIIIA4 had a cause of its own: its first stage takes the accelerations of the start of the step.
- The initial accelerations against the derivative of the velocities of a fine run (LobattoIIIC4GGL, 20 steps per
  1e-3 s, one-sided difference of second order): with Rxyz right to 1e-4 (the difference), with Euler parameters and
  Lie group nodes wrong by up to 3.4 of 12.8 (slider crank) and on a single body spinning about a spherical joint away
  from its center. Causes: C_q v of ComputeConstraintJacobianTimesVector left out temp.localJacobianAE_ODE2radial,
  the part along the Euler parameters which the rows of [M C_q^T; C_q 0] contain since #3201 (the derivative of
  (J_rad v) along the motion, r |v|^2, was missing); with Lie group nodes (C_q v)_q was differentiated by adding to
  the rotation vector and multiplied by the angular velocities, which is not the derivative along the motion.
- Fixed (#3289): CSystem::ComputeConstraintVelocityTerm, (C_q v)_q v as the central difference of fourth order of C_q v
  at the positions moved by +-eps v and +-2 eps v, Lie group nodes by CompositionRule with the increment, C_q v with the
  radial part; eps v 1e-4 of the largest reference plus current coordinate (at least minimumCoordinateSize). Replaces
  NumericalConstraintJacobianDerivative (dense only) and ComputeConstraintJacobianDerivative (sparse), the matrix
  (C_q v)_q is not formed any more. A central difference of second order with 1e-5 gave 2e-10 on the pendulum of
  initialAccelerationsTest, the fourth order its exact value. With it the accelerations agree with the fine run for all
  three node types; LobattoIIIA4 with Euler parameters: order 4 (errors 6.3e-10, 4.0e-11, 2.7e-12).
- Lie group nodes: the stage rotation was q0 composed with h sum_j A_ij omega_j - the increment of the other
  coordinates; the rotation vector of the motion with body angular velocity omega satisfies Delta' = TExpSO3Inv(Delta)
  omega, so the increment differs by terms of h^2 (order 2). Fixed (#3290): Delta_i = h sum_j A_ij TExpSO3Inv(Delta_j)
  omega_j (Runge-Kutta-Munthe-Kaas, as CSolverExplicit), by a fixed-point iteration from the linear increment, at most
  10 iterations, until the change is below 1e-15. The Jacobian keeps the tangent operator of the linear increment.
  After it, Lie group nodes: LobattoIIIA4 3.99/3.85, LobattoIIIC4GGL 3.96, Radau3 2.97, TrapezoidalGGL 2.00 - as Euler
  parameters; implicitRungeKuttaLieGroupTest: Lie group against Euler parameters 2e-16 to 2e-15 for the order-4
  methods, 2.0 Newton iterations per step (the sum of the test 13 instead of 15).
- Test suite: initialAccelerationsTest.py extended by the rigid body on a spherical joint with Euler parameters, Lie
  group node and Rxyz (the accelerations of its center equal); 8 references changed by 1e-11 to 6e-9 relative
  (generalized-alpha starts from the initial accelerations; the other node types by the central instead of the forward
  difference), modelLibraryTest by 3e-7 (the slider crank with Lie group nodes), SwitchingConstraintsTest its constant
  by 5.8e-10. The AVX2 references of the changed models are re-recorded with the fast module.

<a id="rg20-7-3"></a>
### RG20.7.3 — no allocations per step in the implicit Runge-Kutta solvers, timeIntegration.errorEstimate (2026-11-06, #3287)
- Maintainer 2026-11-06: the open solver tasks RG20.4, RG20.8, RG20.7.
- Measured by exu.special.InfoStat, 100 against 200 steps (nPendulum, rigidBodyJoints, kinematicTree of
  modelLibrary.py): TrapezoidalGGL and LobattoIIIA4 created 6 to 10 arrays per step, and with the dense solver 3
  matrices per step; generalized-alpha none. The arrays: the local SparseTripletVector of the stage Jacobian; the
  matrices: GetSparseEntries of the dense matrix, by default a dense copy (GetEXUdenseMatrix) of jq, jv and the mass
  matrix per Jacobian. Not counted by InfoStat, also per Jacobian: the std::vectors of the entries and the two
  unordered_maps of each AddProduct.
- Changed: GeneralMatrixEXUdense::GetSparseEntries reads its matrix without a copy; the entries, the stage triplets
  (SetNumberOfItems(0) keeps the memory) and the work of AddProduct are members of CSolverImplicitRungeKutta;
  AddProduct by the right block compressed by rows and the products as (key, value) sorted and summed, instead of the
  hash maps. After it: 0 arrays and 0 matrices per step for all four methods, dense and sparse (+-1 when a work array
  grows). The kinematic tree creates 17 to 29 vectors per step in every solver, generalized-alpha as well - a matter
  of ObjectKinematicTree, not of these solvers (not changed here).
- `timeIntegration.errorEstimate` (TString, default 'default'): 'oneStep', 'twoSteps', 'default' (twoSteps for the
  Runge-Kutta methods, oneStep for TrapezoidalGGL, as before); another value raises a ValueError at the start.
- Found while testing it: the oneStep estimate - the difference to the trapezoidal rule of the first and the last
  stage - is zero for TrapezoidalGGL, which is that rule: its automatic step size (the default) always took the
  maximum step (the mass on a spring with the tolerances 1e-6: 21 steps, error 0.024). Now as SolveTrapezoidalGGL:
  the position error h^2/12 (a1 - a0) and the velocity error h^3/12 a_tt by the second difference of the mean
  accelerations (v1 - v0)/h of this and the last two accepted steps (also with twoSteps when there are constraints):
  692 steps, error 6.1e-5; twoSteps 696 steps, 5.9e-5. No test model used the automatic step of TrapezoidalGGL in C++.
- test_implicitRungeKuttaAllocations.py (new pytest): the arrays and matrices of 100 and 400 steps differ by at most
  2, for the four methods, dense and sparse, and with the automatic step. implicitRungeKuttaErrorEstimateTest.py (new):
  default equal to twoSteps (LobattoIIIA4) and oneStep (TrapezoidalGGL), the two estimates different, the errors below
  1e-3, an invalid value refused. parameterConversionTestReference.txt: the new setting recorded.
- Seen, not changed: modelLibrary.py calls CreateRevoluteJoint with the deprecated bodyNumbers.

<a id="rg20-7-2"></a>
### RG20.7.2 — Lie group nodes in the implicit Runge-Kutta solvers; the composition of small rotations (2026-11-06, #3288)
- Maintainer 2026-11-06: the open solver tasks RG20.4, RG20.8, RG20.7.
- Reproduced on sliderCrank3D of modelLibrary.py (NodeRigidBodyRotVecLG), to t = 0.2: LobattoIIIA4 and Radau3 fail at
  h = 6.25e-5 (Newton cycles at 2.5e-10 in the first step), TrapezoidalGGL and LobattoIIIC4GGL run; with Euler
  parameters or Rxyz all converge to 1e-12 in 2-3 iterations. Not the Jacobian: a fresh Jacobian in each iteration and
  the tangent operator (added first) changed nothing. The residual per block (new, verbose 3) put the floor in the
  ODE2 equations of the middle stage; the stage Jacobian against finite differences (new, verbose 4, before the
  factorization, perturbations 1e-3 as the constraints at the end change by h^2 times them) showed the position
  constraint at the end not depending on a stage acceleration at all: the composed rotation of the slider did not
  change.
- Cause: EXUlie::CompositionRotationVector took the composed angle from x = cos(w/2) as pi - 2 atan2(x, sqrt(1 - x^2));
  for w below about 1e-8 rad, x rounds to 1 and the angle to 0 - rotation increments of a few 1e-12 rad, as Newton
  corrects at small steps, were lost. Fixed: rho/2, the vector part of the quaternion product, has the norm sin(w/2),
  so w = 2 atan2(|rho|/2, x), equal in exact arithmetic on [0, 2 pi] and accurate for small angles (pi - 2 atan2(x, s)
  also cancels against pi, 1e-4 relative at w = 1e-12). The same in CompositionRuleForRotationVectors of
  lieGroupBasics.py, whose abs() workaround (#2494) is not needed any more; composing pi n with itself is now exact to
  1e-15 for any axis (it was 4e-8 for n = [0, 0, 1]).
- The stage Jacobian multiplies the rotation columns of jq of Lie group nodes by TExpSO3 of the rotation increment of
  the last stage, as generalized-alpha with lieGroupAddTangentOperator (ApplyTangentOperator, on the sparse entries).
- After it, sliderCrank3D to t = 0.2: all four methods run at 1e-3, 2.5e-4, 6.25e-5; LobattoIIIA4 at 6.25e-5 in 3
  iterations per step. Orders against LobattoIIIA4 with h/32 (h = 2e-3, 1e-3, 5e-4): 2.0 for all four methods and for
  generalized-alpha, with Lie group nodes and with Euler parameters alike - not of the Lie group nodes; RG20.7.4 added.
- Test suite: 10 test models with Lie group nodes changed by their rounding: generalContactFrictionTests by 4.6e-4 and
  generalContactCylinderTest by 3.6e-4 absolute (the models whose AVX2 references already differ by 7.5e-3 and
  4.2e-5), sphereTriangleTest2 by 4.3e-5, the other seven by 2e-8 or less. Performance tests: perfConnectorInterface (the explicit
  runs with Lie group nodes) by 1.5e-9 and 2e-10. The examples run: all 196 pass. Their AVX2
  references (fast module, not built here) are to be re-recorded at the next fast-module run.
- implicitRungeKuttaLieGroupTest.py (new): the slider crank at h = 6.25e-5 to t = 0.005, Lie group nodes against Euler
  parameters to 1e-10 with at most 3 iterations per step for the four methods; the composition keeps 1e-12 rad.

<a id="rg20-7-1"></a>
### RG20.7.1 — the implicit Runge-Kutta solvers in C++: TrapezoidalGGL, LobattoIIIA4, LobattoIIIC4GGL, Radau3 (2026-11-05, #3261)
- Maintainer 2026-11-05: the one class CSolverImplicitRungeKutta and the step size control for all four are agreed;
  the C++ needs the cases the prototypes refuse (velocity-level constraints, Lie group nodes).
- Implemented as design section 8.5a: a component of the implicit second-order solver (no new Main class), the
  method as data (GetMethod), the stage Newton of the Python _SolveRungeKutta with the Jacobian at the last stage and
  H left out, the error estimates of one and of two steps, StepSizeController (the PI controller of _ErrorStepSize).
- Two errors found on the way: the first Jacobian of a step was taken at the stage state of the previous step (at
  the first step zero vectors - Euler parameters p = 0, a singular or wrong matrix, apparently random failures); the
  residual norm was that of the scaled rows (h^2 times the forces), so that Newton stopped after one iteration at
  small steps - now the unscaled norm, and the rows scaled to entries of order 1.
- Orders measured (section 8.5a): 2, 4, 4, 3 on four models with constraints and Euler parameters; the automatic step
  size works (mass on a spring, tolerance 1e-8: 180 steps from a maximum step of 0.05).
- Open: Lie group nodes at small steps (RG20.7.2); per-step allocations of the triplets, the setting
  timeIntegration.errorEstimate (RG20.7.3); ODE23 and DOPRI5 on StepSizeController (RG20.8.2).
- implicitRungeKuttaSolversTest.py: the orders on the pendulum and the rigid body chain, the automatic step size.

<a id="rg20-2-8"></a>
### RG20.2.8 — index 3 at the stages alone, measured (2026-11-05)
- Maintainer: why not c(Q_i) = 0 at both implicit stages of Radau IIA or Lobatto (or one multiplier for both), index 3
  directly, which would spare the velocity constraints of the user items in C++? Measured with a scratch variant of
  _SolveRungeKutta (pendulum 25/50/100 steps, slider crank 16/32): Radau IIA orders q/v/z 3/2/1, Lobatto IIIC 4/2/1,
  the velocity constraint 4e-4 to 6e-4 (100 steps) - the theory of Hairer, Lubich, Roche; Lobatto IIIA 2/2/no
  convergence; one shared multiplier the same or failing (IIIA). The velocities projected after the step: v of the
  full order, z of order 1. The end form needs only c, C_q and c_t (C_q v + c_t, c_t numerical); without the term
  H = (C_q v)_q in its Jacobian the results are the same with 3 % to 27 % more Newton iterations. Design
  advancedSolvers.md section 3.2; no code change.

<a id="rg20-6-1"></a>
### RG20.6.1 — The C++ solvers as they are (2026-11-05)
- docs/dev/design/cppSolverStructure.md: the files and sizes, the classes (CSolverBase with the static, the implicit
  second-order and the explicit solver; the Main classes and the user-function variant for Python), the step loop
  (InitializeStep, DiscontinuousIteration around Newton and PostNewton, FinishStep, reduction and increase of the
  step), the generic Newton on nSys unknowns, the two step size rules (the explicit error control inside Newton(), the
  adaptive step of the implicit solver), output and restart, the Python layer, and the facts that matter for RG20.7:
  more unknowns than nSys (mu, stages), Jacobians per stage, the stages of the previous step not kept, output at the
  end of the step. Maintainer: analysis of the present state only; the recommendations after the symbolic solvers
  are final (RG20.6.2).

<a id="rg20-5-3"></a>
### RG20.5.3 — Euler parameters in the symbolic system and the projection in SolveExplicit (2026-11-04, #3238)
- **UserItemCreator.CreateRigidBody** on a `NodeRigidBodyEP` (now the default, as `mbs.CreateRigidBody`; Rxyz stays):
  the node as `mbs.CreateRigidBody` computes it, the user body with the variant `_rigidBodyEPFunctions` of the kind
  rigidBody - mass matrix [[m I, -m A Skew(com) G], [.., G^T J G]] with GLocal(p) (3 x 4), the forces -G^T (omega x J
  omega) and -A (omega x (U x omega)) (GLocal(p_t) p_t = 0), the access functions with GLocal; the algebraic equation
  is the node's (CNodeRigidBodyEP, p^T p - 1). A heavy top on a spherical joint agrees with the built-in rigid body
  in mbs.SolveDynamic to 1e-13.
- **SymbolicSystem**: `eulerParameterNodes` (the nodes of type RigidBodyEP with an algebraic equation); in
  AlgebraicEquations p^T p - 1 (p reference plus current), in ODE2ReactionForcesLHS 2 p z, in the exact SystemJacobian
  2 z I, 2 p and 2 p^T (against central differences 1e-13); the count of the algebraic equations includes them.
  Equations() refuses them in algebraicEquations and ODE2ReactionForcesLHS for now. SolveGeneralizedAlpha agrees
  with mbs.SolveDynamic to 1e-12; the solvers of advancedSolvers take them as any constraint (LobattoIIIA4 of order
  4.4, TrapezoidalGGL 2 on a body on a spring).
- **SolveExplicit**: the equations of the Euler parameter nodes are not eliminated; with them the accelerations come
  from [M C^T; C 0] [a; z] = [f; -2 p_t^T p_t], C = 2 p^T, on the free coordinates (dense, also with
  computeMassMatrixInversePerBody, the mass matrix of Euler parameters being singular), and after each step p is
  projected onto |p| = 1 and p_t onto p^T p_t = 0 (`_ProjectEulerParameters`); the other algebraic equations as before
  (`_FixedCoordinates` on the rows given). A body on a spring-damper by RK44: order 3.96, the norm kept to 1e-16.
- symbolicEulerParametersTest.py (new, 1.4 s).

<a id="rg20-2-6"></a>
### RG20.2.6 — SolveImplicitRungeKutta: LobattoIIIA4, LobattoIIIC4-GGL, Radau3; the formulas of the error estimates (2026-11-04, #3247)
- Maintainer (third answers): IIIA-end as `LobattoIIIA4`, IIIC-GGL as `LobattoIIIC4-GGL`, and Radau3 - three methods
  with two implicit stages; IIIC-end not kept. `SolveImplicitRungeKutta(system, settings, method)` and
  `implicitRungeKuttaMethods` replace `SolveLobatto4` and `lobatto4Variants` (introduced today, not released);
  `_SolveLobatto` is `_SolveRungeKutta`, the tableaus `_rungeKuttaTableaus` with Radau IIA (c = 1/3, 1).
- Radau3 measured on the pendulum (50/100/200 steps): with the constraints at the end of order 3.0 in q and v, 2.0 in
  a and z; with GGL the same, 20 % smaller errors; kept with the constraints at the end (fewer unknowns, no mu). The
  stiff spring at h omega = 70: Radau3 damps the fast mode and takes 0.09 of the energy of the pendulum in 2 s. Its
  embedded estimate uses the start, q0 + h/2 (v0 + v1), v0 + h/2 (a0 + a1).
- The error estimates (design section 3.2, maintainer: the formulas, one trapezoidal step or two substeps): both are
  the same second difference D = V_1 - 2 V_2 + V_3 of the stage velocities (S - T1 = -h/3 D, S - T2 = -h/12 D,
  Simpson S = (4 T2 - T1)/3); no combination of the three stage values vanishes for quadratics, so an order-4
  estimate needs values beyond the step - the candidates of RG20.2.5 (step doubling as the reference, the fourth
  divided difference over two steps, IIIA against IIIC, a defect). The design file revised to the current state:
  what exists, the formulas, the measured table of all constraint forms, the decisions of the three answers, the
  results and the status of the steps.
- symbolicLobatto4Test.py on the new names, Radau3 included (the file keeps its name).

<a id="rg20-2-3"></a>
### RG20.2.3, RG20.2.4 — Lobatto4 with the constraints at the end of the step, its automatic step size; the chain from horizontal (2026-11-04, #3246)
- **The chain from horizontal** (maintainer: start at horizontal or at a right angle): the free end reaches 7.5 m/s
  at 0.46 s. At 0.5 s TrapezoidalGGL needs 99, 231, 514 automatic steps (tolerances 1e-4, 1e-5, 1e-6) for 1.8e-3,
  2.1e-4, 3.5e-5; constant steps 100, 200, 400 give 1.5e-3, 3.7e-4, 9.1e-5 - as good, with an error following the
  tolerance. The start near upright of RG20.1.4 is the exception, documented (maintainer: document only).
  symbolicTrapezoidalGGLStepSizeTest.py starts horizontally.
- **The constraints at the end** (maintainer: one of the 3-stage Lobatto methods worked in HOTINT with index 3 without
  GGL, possibly with the constraints at the end only). Measured on the pendulum (25/50/100 steps, scratch solver):
  - one multiplier constant over the step, c(q1) = 0: IIIA, IIIB, IIIC, IIIA-IIIB and IIIB-IIIA all of order 4 in q,
    2 in v, 1.8 in z, the velocity constraint drifting by 4e-4 at 100 steps; the multiplier of a stage linear between
    the previous and the new end value diverges;
  - a multiplier per stage after the start, the start that of the previous end, c(q1) = 0 and C_q(q1) v1 = 0: IIIA
    4.0/4.0/2.1 (q/v/z), IIIC 3.7/4.1/2.8, IIIA-IIIB 4.0/4.0/2.1, both constraints at 1e-15.
  The second is in `_SolveLobatto` as constraints='end' (rewritten with 'GGL': one layout of the unknowns, the stage
  accelerations of the implicit stages and the multipliers of the stages after the start, the end constraints at
  their rows); `lobatto4Variants` 'IIIA-end' (default), 'IIIC-end', 'IIIA-GGL', 'IIIC-GGL', plain 'IIIC' dropped.
  Slider crank (8/16/32 steps): IIIA-end 4.5/4.0, IIIC-end 4.1/4.2, in 0.5 s against 0.9 and 1.4 s of IIIA-GGL and
  IIIC-GGL. Stiff spring (h omega = 70): IIIC-GGL damps the fast mode (energy -0.5), IIIC-end gains 0.28 there and
  damps at h omega = 14, IIIA-end keeps the energy to 8e-5 at h omega = 14.
- **The automatic step size** (RG20.2.3, maintainer: the embedded solution first): the trapezoidal solution of the
  first and the last stage, q0 + h/2 (V_1 + V_3), v0 + h/2 (A_1 + A_3), its difference to the end of the step as the
  error of order h^3 in the PI controller. It works (1 to 16 rejections), but estimates the order-2 error: Lobatto4
  takes about the steps of the trapezoidal rule, with errors far below the tolerance - ball at 1e-6, 1e-8, 1e-10: 433,
  1662, 7457 steps, errors 3e-8, 6e-9, 4e-11 (TrapezoidalGGL 419 steps, 9e-5); slider crank at 1e-6: 68 steps, 4e-10,
  where 16 constant steps give 1e-8. RG20.2.5 added for an order-4 estimate.
- symbolicLobatto4Test.py: the four variants, IIIA-end on the slider crank with constant and automatic steps,
  IIIA-end keeping the energy of the stiff spring.

<a id="rg20-2-2"></a>
### RG20.2.2 — Lobatto4 with constant steps: the variants measured, IIIC-GGL chosen (2026-11-04, #3244)
- `_SolveLobatto` of exudyn/symbolics/advancedSolvers.py: one Newton method for the stages of a stiffly accurate
  tableau, Q_i = q0 + h c_i v0 + h^2 (A^2)_ij A_j, V_i = v0 + h A_ij A_j, the stage equations of motion and
  constraints at the implicit stages (a first row of zeros: the stage is the start, IIIA), with GGL also C_q V_i +
  c_t = 0 and Q_i - C_q^T mu_i; the Jacobian per stage from two SystemJacobian calls (by q and by v) and the mass
  matrix. Check: its 2-stage IIIA-GGL equals SolveTrapezoidalGGL to 5e-16. `SolveLobatto4(system, settings,
  variant)` with `lobatto4Variants`; ODE1 and data coordinates refused for now.
- **Measured** (pendulum, 25/50 steps against 400, Newton to 1e-13):

  | variant | q | v | a, z | velocity constraint |
  |---|---|---|---|---|
  | IIIA-GGL (2 implicit stages) | 4.0 | 4.0 | 2.1 | 1e-14 |
  | IIIC-GGL (3) | 3.6 | 4.1 | 2.0, errors 25x those of IIIA-GGL | 1e-14 |
  | IIIC, constraints at all stages (3) | 4.1 | 2.1 | 1.0 | 2e-3 |
  | IIIA index 3 without GGL (2) | 2.0 | 2.0 | does not converge | 1e-4 |

  Slider crank (8/16 steps against mbs.SolveDynamic with 20000 steps): IIIA-GGL 3.97, IIIC-GGL 4.2 in q and v, IIIC
  4.0 in q and 1.9 in v; Lobatto4 with 32 steps 1e-8, TrapezoidalGGL with 64 steps 4e-5. A pendulum with a mass on a
  stiff linear spring (omega = 1414/s) at h = 0.05: IIIC-GGL damps the fast mode (its energy 0.5 removed, the slow
  motion kept), IIIA-GGL and plain IIIC do not converge at h omega = 70; at h = 0.01 IIIA-GGL keeps the energy to
  1e-3, TrapezoidalGGL loses 4.5 of it.
- **Chosen**: IIIC-GGL as the default - order 4 in q and v and L-stable; the maintainer's expectation (IIIC with the
  constraints at all stages) holds for q but not for v, which needs the velocity constraints. IIIA-GGL stays as the
  variant without damping and of smaller acceleration errors, IIIC for the record.
- symbolicLobatto4Test.py (new, 9 s).

<a id="rg20-1-4"></a>
### RG20.1.4, RG20.8.1 — The step size of TrapezoidalGGL on a whipping chain and a bouncing ball; the velocity error; the PI controller (2026-11-04, #3243, #3245)
- **Models** (maintainer: a falling chain of about 10 elements, fixed on one side, the other nearly vertical up, the
  whip effect; a contact sphere): a chain of 10 mass points (NumpySystem, 20 coordinates, 10 distance constraints)
  from 80 degrees, its free end at 7.9 m/s at 0.7 s, against Exudyn's mass points and ObjectConnectorDistance with
  generalized-alpha (spectral radius 0.9 - 1 is unstable with index 3 at small steps; 80000 and 320000 steps agree to
  1e-6); a ball on the ground with Hertz contact k p^1.5 and Hunt-Crossley damping (two contacts of 10 ms in 1.5 s)
  against scipy DOP853 at 1e-13.
- **Ball**: the controlled step needs 419 steps for the error 9e-5, constant steps 4000 for 1e-3 and 16000 for 5e-5.
- **Chain**: the controlled step is worse than a constant one: at 0.7 s 393 steps give 0.17, 200 constant steps
  9e-3. The error is already at 0.4 s, before the whip, 60 times that of constant steps of the same mean size: the
  start near the upright position is unstable, its errors grow exponentially, and the controller takes large steps
  there as the local error is small. A local error controller does not see the sensitivity; tighter tolerances help.
- **Velocity error**: by the second difference of the end accelerations it is spoiled with constraints by their
  alternation (the slider crank then took 3243 steps at 1e-7 with the PI controller); by the second difference of the
  mean accelerations (v1 - v0)/h of the last three steps it is not (1195 steps, error 3e-5, 2 rejections) and equals
  the former on the unconstrained ball. Kept: positions by h^2/12 (a1 - a0) and velocities by the mean accelerations.
- **PI controller** (RG20.8.1, Gustafsson: h safety (1/e)^(0.7/k) e_p^(0.4/k), after a rejection no growth): the
  rejections drop from up to 65 (I controller) to 2-12 on all three models at equal accuracy.
- Newton: a diverging iteration (NaN at a first step of the maximum size on the chain) counts as not converged, so
  that the step is reduced.
- Found: the C++ error norm of ODE23/DOPRI5 uses max(q_new, q_start) without absolute values - #3245, RG20.8.2.
- symbolicTrapezoidalGGLStepSizeTest.py (new, 13 s); symbolicTrapezoidalGGLTest.py: 140 and 675 steps instead of 74
  and 343 at the same tolerances, with 3.5 times smaller errors (the velocities are in the estimate).

<a id="rg20-1-2"></a>
### RG20.1.2, RG20.1.3 — SolveTrapezoidalGGL with constant steps and with the step size by the local error (2026-11-04, #3242)
- **The module** exudyn/symbolics/advancedSolvers.py (new, decision 7: the new solvers in a module of their own), on
  the helpers of solvers.py (`_Output`, `_StepSize`, `_PostNewton`, `_InitialAccelerations`), in the utility docs.
- **The method**: v1 = v0 + h/2 (a0 + a1), q1 = q0 + h/2 (v0 + v1) - C_q^T mu, M a1 = f - C_q^T z1, c(q1) = 0,
  C_q v1 + c_t = 0 - Newmark with beta = 1/4, gamma = 1/2 plus the velocity constraints and the second multiplier mu.
  Newton for (a1, y1, z1, mu), the ODE2 rows scaled by h^2/4 and the z columns by its inverse; the mu columns
  -J_q C_q^T from a second SystemJacobian by q; the velocity rows with (C_q v)_q by a difference of C_q along v.
  C_q is taken row by row from ODE2ReactionForcesLHS with unit multipliers, exact for joints and ObjectGenericDAE;
  C_q^T mu in the positions at the previous iterate. ODE1 coordinates by the trapezoidal rule, their initial
  velocities from the ODE1 equations; consistent initial accelerations and z. Algebraic equations of q_t, y or z are
  refused (random variations, as `_FixedCoordinates`).
- **On the way**: with C_q of the last Jacobian in the residual, the velocity constraint ended at 1e-6 and the
  accelerations and z of a fine reference stagnated at 3e-3 - the reason for the exact C_q in the residual; then the
  constraints hold to 1e-15 on both levels and a and z converge with order 2. Without (C_q v)_q the Newton method
  needed 3 to 5 iterations per step on the slider crank, with it 1.3 to 3.
- **Measured, constant steps**: pendulum (NumpySystem) and slider crank (two rigid bodies on revolute joints, the
  slider a mass point on a spherical joint with its height fixed; SymbolicSystem of UserItemCreator) against
  mbs.SolveDynamic with 5000 steps: order 2.00 to 2.02 in q, v, a, z; at an equal step 25 % more accurate in q than
  generalized-alpha with spectral radius 0.8.
- **The step size** (RG20.1.3, decision 1 and 3): the error h^2/12 (a1 - a0) = h^3/12 q_ttt of the positions, the
  norm and the rule of CSolverExplicit (root mean square of the errors over atol + rtol max(|q0|, |q1|), h_opt = h
  (1/err)^(1/3), safety, maximum increase, rejected if h_opt < h); `_ErrorStepSize`, derived from `_StepSize`, also
  reduces the step after a failed Newton iteration or post-Newton step. Measured on the pendulum: the tolerances
  1e-5 and 1e-7 give 74 and 343 steps, errors 8.4e-4 and 3.5e-5.
- **Found**: at an equal number of steps the controlled step is no better than constant steps - equal on the free
  pendulum, 4 times the error with a force pulse of width 0.01, 10 times on the slider crank. The estimate does not
  see the local error h^3/12 a_tt of the velocities, which dominates the global error. Tried and not kept: a_tt by the
  second difference of the end accelerations of two steps - spoiled by the undamped odd-even oscillation of the
  trapezoidal accelerations (the error alternated 0.6, 1.5, 0.8 between steps, 26 rejections); the differences of the
  mean accelerations (v1 - v0)/h of three steps, which are free of it - fewer rejections, but no better per step. The I
  controller rejects 5 to 20 % of the steps where the error grows. Variable steps themselves are of order 2 (steps
  alternating 1/300 and 1/600 against 1/400). RG20.1.4 added for the velocity error and a model of changing time
  scales; RG20.8 for a PI controller.
- symbolicTrapezoidalGGLTest.py (new, 7 s): the orders, the constraints, against SolveGeneralizedAlpha, ODE1
  coordinates, the automatic step size with two tolerances, the refusal of an equation of the velocities.

<a id="rg20-4-3"></a>
### RG20.4.3 — The output of the symbolic solvers at the exact write times, interpolated within the step (2026-11-04, #3241)
- `_Output` of exudyn/symbolics/solvers.py keeps the state of the previous step and writes the result at
  tStart + k writePeriod (a counter, not a sum) and at the end; a write time within a step takes `_Interpolate`: q by
  the cubic Hermite polynomial of q and q_t at both ends, q_t its derivative, q_tt, y, y_t and z linear. A write time
  within 1e-10 of the end of a step writes the step as computed, so a period that is a multiple of the step gives the
  states of the steps, as before.
- Through mbs: the write times within the step are the next ones of the C++ writers, `output.lastSolutionWritten` and
  `lastSensorsWritten`; for each, the interpolated state into mbs, then `WriteCoordinatesToFile` and the new binding
  `WriteSensorsToFile` (MainSolverBase.h, the three solver classes of structureDefsSolvers.py), each of which writes if
  its time is reached and advances it by its period; `FinishStep` at the end of the step as before - the post-step
  user function, the renderer and the console output stay once per step. The data coordinates within the step are
  those of its end.
- Measured: no result of the test suite changed (its write periods are multiples of the steps or the models read the
  last values). An oscillator by RK44 with 37 and 74 steps and the period 0.0137: the interpolated output converges
  with order 4.03, the order of RK44.
- symbolicSolverOutputTest.py (new): the write times, the order of the interpolated output, the states of the steps,
  the sensor and the solution file of a SymbolicSystem at their own periods, equal to the result.

<a id="rg20-5-2"></a>
### RG20.5.2 — SolveExplicit eliminates fixed coordinates; the decisions of RG20 (2026-11-04, #3238, #3240)
- **Decisions** (maintainer 2026-11-04 on the six questions of docs/dev/design/advancedSolvers.md): the acceleration
  difference for TrapezoidalGGL; `Lobatto4`, the maintainer expecting Lobatto IIIC with the constraints at all stages
  for index 3 (to be compared with IIIA and GGL in RG20.2.2; IIIC has three implicit stages, IIIA two), the embedded
  trapezoidal and the IIIA/IIIC error estimates, Radau3 kept in mind; the existing absolute and relative tolerances for
  all adaptive solvers; Python first for the output within the step and the Euler parameters, the new solvers in a
  module of their own, C++ when settled; the elimination as in C++; a common step size controller - RG20.8 added,
  RG20.2 and its sub-steps renamed, RG20.5.4 added for #3240.
- **The elimination** (exudyn/symbolics/solvers.py `_FixedCoordinates`): the constraint Jacobian by central
  differences at the initial state; an equation is a fixed coordinate if its row has one nonzero entry and the
  equation does not change with a random variation of q (beyond the linear part), q_t, y, z and t; the value of the
  coordinate from the equation, so an offset and a factor are kept; stages with zero velocity and acceleration of the
  fixed coordinates; a mass matrix that couples a fixed coordinate (or is not constant) solved densely without its
  rows and columns. eliminateConstraints=False warns and ignores the equations, as Exudyn.
- **Found**: Exudyn's explicit solver zeroes the fixed coordinates at the end of the step only; its stages evaluate the
  forces with them moved, so RK44 with eliminated coordinates converges with order 1 (the chain of the test: 500,
  5000, 50000 steps differ by 4e-5 and 4e-6 at mass 1), while SolveExplicit is converged at 500 steps. Explicit Euler
  agrees to 1e-10. #3240 raised. The finding also means that the elimination of the C++ solver cannot be the
  reference of the RK44 result: the test compares RK44 against 2000 steps of itself and, to 2e-6, against 50000 steps
  of Exudyn's.
- **Euler parameters** (RG20.5.3): SymbolicSystem refuses NodeRigidBodyEP, as its norm constraint is an algebraic
  equation of a node; the projection in SolveExplicit needs the node in SymbolicSystem first.
- symbolicExplicitConstraintTest.py (new): a chain of three mass points of UserItemCreator, three coordinates fixed by
  CreateCoordinateConstraint, against the built-in chain; an offset; a NumpySystem with a coupled mass matrix against
  the system without the fixed coordinate; the refusal of a constraint between two coordinates.

<a id="rg6-20-6"></a>
### RG6.20.6 follow-up — scopeTest.py without deprecated arguments, run directly with the renderer; scopeExample.py (2026-11-04, #3237)
- Maintainer: scopeTest.py raised deprecation messages and opened no renderer when run in Spyder. It used `physicsMass`,
  `bodyNumbers` and `solutionSettings.writeSolutionToFile`; now `mass`, `itemNumbers`, `solution.file.write`. Run
  directly (not in the test suite), it shows the model with the scopes of the automatic mode for 10 s in real time.
- scopeExample.py (new): a double pendulum of two mass points on distance constraints, scope0 the heights over the
  last 5 s, scope1 all components of a velocity, scope2 the paths of both masses (y over x, last 3 s); live during the
  simulation and in the solution viewer. Without a window it is not run in real time (0.1 s).
- The legend of a line over another sensor names the x-component as well: "mass1, PositionY over PositionX".

<a id="rg6-20-1"></a>
### RG6.20.1-.6, RG6.20.8 — The Scope: the decisions, the settings, ScopePlot, OpenGL and the raytracer, the lock of the stored sensor data (2026-11-04, #3237, #3239)
- **Decisions** (maintainer 2026-11-04, on the eleven questions of the first evaluation; docs/dev/design/scope.md
  section 2): three fixed scopes per view; the settings in `view<i>.window.scope<j>` (the maintainer named
  `view<i>.scopes.scope<j>` as the alternative; the place of the request was taken); the sensors as n x 5 integers
  with the mbs number, n x 2 accepted; xRange dropped; 16 colors x 3 styles close to PlotLineCode, no markers; no key,
  the icon and the settings, with an automatic mode; the component names in C++; a lock instead of reading unlocked;
  Reset to be checked; the extras later on SC.renderer, a point a circle with the line width; axesColor
  [0.1, 0.1, 0.1, 1]; the message once per run, 30 s in the window.
- **Reset while the renderer runs** (decision 9): SC.Reset() and deleting a SystemContainer detach the renderer first,
  which stops the loop and joins the thread; mbs.Reset() and item changes do not synchronize with the render thread -
  #3239 raised, RG6.20.9.
- **Settings** (RG6.20.2): VSettingsScope with sensors, region, frameColor, backgroundColor, axesColor, lineWidth,
  useGrid, verticalZoom, horizontalZoom, timeHistory, showLegend, xLabel, yLabel, xResolution; `window.showScopes` and
  scope0..2 (memberDefaults for the regions of scope1 and scope2). The generator got the types `Float2` (the zooms)
  and `MatrixI` (the sensors, a list of lists in Python, rows of equal length or ValueError): typeModel,
  structureHeaderEmitter (converted list members), itemModel.Size, EPyUtils::FromPython/ToPythonMember for MatrixI.
  `componentY=-1` draws all components, as the automatic mode. parameterConversionTestReference.txt rewritten: only
  the new settings added; the stubtest backlog has exudyn.VSettingsScope beside the other settings classes.
- **ScopePlot** (RG6.20.3, src/Graphics/ScopePlot.h/.cpp): one ordered list of rectangles, lines with their width and
  texts in pixels of the window - ordered, so that the legend box is drawn over the lines. The values up to the
  visualization time and of the last timeHistory seconds, decimated to xResolution values per pixel by the minimum
  and maximum of each bin in their order, with one value on each side of a horizontalZoom; over another sensor every
  k-th row. Ticks of 1, 2, 5 x 10^k with at least two in the range (a scope of 50 pixels showed only the tick 0),
  sub-ticks 5 (4 for a spacing of 2); a small scope leaves out the axis labels; the legend box at 60 % of the
  background alpha, as it covered the lines of the default 1/5 height.
- **Drawing** (RG6.20.4, .5): OpenGL in GlfwRenderer::RenderScopes after the overlay and before SaveImage, lines of
  one width in one glBegin; the raytracer in Raytracer::DrawScopes into its image before the scaling down - the
  rectangles blended, the lines by RasterizeLine as static objects, the texts by DrawString. RasterizeLine draws
  nothing for a line shorter than a pixel (steps = 0), which dropped the flat parts of every plotted line; such a line
  is drawn one pixel long by DrawScopes, RasterizeLine itself is unchanged. The raytracer image of a scope takes 20 ms
  at 800 x 600. The icon "scope" after the sensors; the message (no such mbs or sensor, no storeInternal, rows not of
  2 or 5) through ShowMessage for 30 s and PrintDelayed, once per start of the renderer; the raytracer prints it.
  **The OpenGL drawing is not seen yet**: no window was opened; the raytracer image was checked.
- **Lock** (RG6.20.8): CSensor::LockInternalStorage/ReleaseInternalStorage, an atomic flag in a member that a copy of
  a sensor does not share; taken by CSolverBase when it resets and appends the rows, by the traces, by ScopePlot and
  by MainSensor::GetInternalStorage. The type check of the triad sensor moved before the lock, as it may raise.
- **Test** (RG6.20.6): scopeTest.py renders a mass on a spring with the model hidden, through the raytracer at
  480 x 360: no scope without showScopes, the automatic mode, a sensor without storeInternal and a verticalZoom beside
  the values without colored pixels, the velocity over the position; test_iconBar.py with the icon. The GUI chapter
  has the section Scopes; revisions.md the entry.

<a id="rg6-20-rg19-rg20"></a>
### RG6.20, RG19, RG20 — The Scope, the robotics revision and the advanced solvers planned and evaluated; closings (2026-11-04, #3237, #3238)
- **Closings** (maintainer 2026-11-04): #2223 (Diff of vectors and matrices, superseded by #3115, #3163), #1104 (a general
  generic object, superseded by ObjectGenericDAE #3170), #1722 (exudyn.Parameter, to be done differently), #2267 and
  RG11.8.11 (the URDF import is revised as a whole in RG19), #3106 and RG7.16 (contact and flexible interfaces in C++
  first; MarkerNodeUser first, RG7.18 with #3236); RG12.65 closed with #3153 resolved, RG12.65.7 and .7.2 closed as
  steps, #3168 stays open. RG7.15 stays open (to be extended later), #1904 stays.
- **Numbers**: the maintainer asked for RG15 (robotics) and RG16 (advanced solvers); revision2026b has the groups RG15 to
  RG18, and step numbers are permanent (planManager.py takes the next free number), so they are RG19 and RG20.
- **RG6.20, the Scope** (#3237): docs/dev/design/scope.md with the request as given, what exists - the sensor traces
  read the internal storage up to the visualization time, the icon bar is drawn after the image is saved, the
  raytracer has lines and text in pixels but no filled triangles, S and CTRL+S are bound -, one list of 2D primitives
  for both renderers, decimation by min/max per pixel column, nice-number ticks; eleven open questions (RG6.20.1),
  among them the default of axesColors, equal to the background in the request.
- **RG19, robotics** (no issue yet): docs/dev/design/robotics.md - 5170 lines in 9 files, about 140 lines of
  commented code, what examples and test models use (ProfileConstantAcceleration 90 times, RobotLink 60, Robot 28,
  CreateKinematicTree 19, ...), the serial robot scripts each with their own PreStepUserFunction, files and
  matplotlib; the URDF import with its special cases (the UR offset, flipped joints not implemented, no inertia check,
  no textures); the analytic IK of future.py unused. Recommended: urdfImport.py, exudyn.HT, q0, one control
  description with friction and end stops for both models, SetControl and PreStepControl, AddSensors, the
  compensation from the kinematic tree, trajectories of joint and Cartesian segments with one Evaluate, analytic IK
  (Pieper) and Levenberg-Marquardt; eight open questions. The check against RTB 1.5 waits for the maintainer's
  environment (RG19.1.2).
- **RG20, advanced solvers** (#3238 for RG20.5 only, as requested): docs/dev/design/advancedSolvers.md - the Python
  solvers have no error control (the step only reduced on a rejected post-Newton step), the C++ explicit solver
  eliminates only coordinate constraints to the ground, the symbolic SolveExplicit refuses algebraic equations, the
  output is written at the step times. Evaluated: TrapezoidalGGL (the name fits) with the error estimate h^2/12
  |a_{n+1} - a_n|; Lobatto with 3 stages, where "directly stable for index 3" holds for the IIIA-IIIB pair rather than
  IIIA alone, a name to decide; the adjoint on TrapezoidalGGL with constant steps, missing the derivatives of the
  constraints by the parameters; cubic Hermite output; Euler parameters by projection; what else is missing; a first
  look at the C++ solver structure. Six open questions.

<a id="rg7-17"></a>
### RG7.17 — The Jacobian functions of the user items required where a connector uses them, no central differences (2026-11-04, #3107)
- Maintainer 2026-11-04: require the position and rotation Jacobian functions only when a joint or connector uses
  the item through a marker, so that a new beam with markers only on its nodes works without them; no central
  differences as backup.
- CSystemAssemble.cpp, `MissingUserJacobianFunction`: for each marker of a connector (joints and contacts are
  connectors), a MarkerBodyUser, a body marker with Position on an ObjectUserBody or a node marker on a NodeUser must
  have its positionJacobianUserFunction, and with a connector that requests the orientation its
  rotationJacobianUserFunction; else a ModelError at Assemble naming the item. MarkerBodyMass and loads are not
  checked: a load or a sensor that needs a missing function raises when it is evaluated, as ObjectUserBody did before.
- The central differences of the position and rotation Jacobians of NodeUser and MarkerBodyUser removed (the
  derivatives of the given Jacobians, J_t q_t and d(J_rot^T v)/dq, stay central differences of the Jacobian
  functions); the acceleration of a NodeUser by its Jacobian only. The velocity of a NodeUser without
  positionJacobianUserFunction raises.
- Test models: userNodeTest.py and userMarkerTest.py without the "numerical" variants, which are now refusals at
  Assemble (the results unchanged to the bit); userANCFBeam3DTest.py computes the rotation Jacobian of its slope node
  in the script by central differences (5.6e-10 relative, within its tolerance 1e-9); the mini examples of NodeUser
  and MarkerBodyUser with their Jacobian functions (MarkerBodyUser 7e-12, the refsol updated); test_graphicsData.py
  with the position Jacobian of its user body.

<a id="rg12-55-13"></a>
### RG12.55.13, RG12.55 — The difference of twists; the Jacobian of a user joint from its constraints (2026-11-03, #3235)
- Maintainer 2026-11-03: the proposals are good to go.
- `Twist operator+/-` in C++, `__add__`/`__sub__` of exudyn.Twist (NotImplemented for another type, so that
  `exudyn.Twist - kinematics.Twist` reaches `__rsub__`), and of kinematics.Twist; the spring-damper, the Cartesian and
  the rigid-body spring-damper of UserItemCreator on them, the last by `LocalTwist(HT(Aj), twist1 - twist0)`.
- `JointJacobianUserFunction(constraintUserFunction, numberOfParameters=None)` in exudyn.symbolics.userItems: the
  constraint function called once with the varied frames of a VariableSet (R0, p0, R1, p1, parameters, variations),
  `esym.Jacobian` by the 12 variations, one esym.Function, written by codeExport ('numpy' for numbers, 'symbolic' for
  a recording, there `Optimize(level=2)` for the zero variations). First with `(I + skew(theta)) @ R` as matrix
  product: 175 / 353 operations for the revolute / prismatic joint against 34 / 85 by hand; the rotation by its
  nonzero components: 55 / 112, the distance constraint 32 / 23, the spherical joint 0 / 0. The numbers equal the
  hand-written Jacobians to 6e-17; symbolicUserJointsTest.py unchanged (recorded 4.3385e-13, the same digits),
  symbolicSystemItemJacobiansTest.py 5.0e-12 as before.
- A recording for a kind (Record of UserItemCreator) has `parameters` of run-time size, without len(): the number of
  parameters is an argument, 0 for the three joints, 1 for the distance constraint.
- The four hand-written jacobianUserFunction, `_JacobianRows` and `_Unit` removed.
- symbolicJointJacobianTest.py (new): the screw joint of userJointTest.py by its constraints alone, its Jacobian within
  7.4e-11 of central differences of the varied frames; the angle 15.2990331116233 with the Python and the recorded
  functions (9e-14 apart), the angle of the hand-written Jacobian in userJointTest.py to 3e-14; the difference of
  twists, numbers and symbolic.
- RG12.55 closed with all its sub-steps.

<a id="rg12-67-6"></a>
### RG12.67.6, RG12.73.3, RG12.67, RG12.73 — The designs of the symbolic parameters and of the names of variables into the documentation (2026-11-03, #3159)
- What is implemented of docs/dev/design/symbolicParameters.md and symbolicVariableNames.md is in the manual and the
  internals; both notes removed. New in the documentation: in the manual (symbolicSystem.md) reading the bindings,
  `ParameterBindings.Keep`, the numbers in the items, structure as a constant of the recording, what holds numbers;
  (symbolicUserFunctions.md) a name given again with another kind or size replaces the variable. In the internals
  (symbolic.md) the library's own variables in scopes of their own and why a scope per model; (symbolicSystem.md §7)
  bindings neither in MainSystem nor in the C++ items, the rest symbolic as user items, no forward sensitivities;
  (userItems.md §8) the parameters of the user items kept by `_AddUserItem`, RigidBodyInertia for both.
- Not taken over, as not built: the options B, B', C and packages 2 and 3 of the names (only the reason against a global
  scope), the options A-C of the source of the bindings (only the reasons against B and C), the forward sensitivities.
- docs/dev/design/symbolicSystem.md: section 12.4 (the parameters as an input, P1) removed, implemented by RG12.67.4;
  12.3 (q relative, the reference values as variables) stays as an open recommendation.
- RG12.67 and RG12.73 closed with all their sub-steps; #3159 resolved (#3208 was resolved with RG12.73.2).

<a id="rg12-72"></a>
### RG12.72 — The user items tested by the examples of the Create functions (2026-11-03, no issue)
- Maintainer 2026-11-03: do RG12.72. The "mini examples built with the Create functions" are the Examples of the
  docstrings of the Create functions of mbs (exudyn/misc/mainSystemExtensions.py); the MiniExamples of the items are
  generated per item from the definitions and use AddObject, one of 102 a Create function.
- symbolicUserItemsCreateExamplesTest.py (new): for each Create function of UserItemCreator the Example is taken from
  the docstring (inspect.getdoc), run as it is and with `creator = UserItemCreator(mbs)` for `mbs.Create...`,
  `creator.Record()` before `mbs.Assemble()`; both with `nodeType=exu.NodeType.RotationRxyz` for CreateRigidBody (the
  node of the user rigid body; the examples have the default Euler parameters), `['nodeNumber']` read as
  `['nodeNumbers'][0]` for the user mass point. Compared: the positions and velocities of the point and rigid nodes
  at the end (1000 steps, 2 s); printed: the time of SolveDynamic of both.
- Results: CartesianSpringDamper 4.0e-10, the others 0 to 6.6e-12; the time of the user items / built-in items, in s:
  MassPoint 0.002 / 0.002, Force 0.002 / 0.002, CartesianSpringDamper 0.004 / 0.003, SphereSphereContact 0.005 /
  0.003, CoordinateConstraint 0.008 / 0.004, SpringDamper 0.011 / 0.004, Torque 0.011 / 0.003, PrismaticJoint 0.019 /
  0.005, RigidBody 0.020 / 0.006, RigidBodySpringDamper 0.055 / 0.012, RevoluteJoint 0.064 / 0.013, SphericalJoint
  0.072 / 0.011, TorsionalSpringDamper 0.073 / 0.013: the joints and the connectors on rigid markers cost the most.
- CreateDistanceConstraint: its example constrains a node, which the user item refuses ("a node or a marker is not
  supported"); the test expects exactly this refusal, so that a new one is seen. CreateGround has no simulation.
- The examples of CreateRigidBodySpringDamper and CreateTorsionalSpringDamper ("coming later") and of
  CreateSphereSphereContact (none) written, in the form of the others: a bar on a bushing, a bar on a revolute joint
  with a torsional spring-damper, a mass point falling onto a sphere fixed to the ground.

<a id="rg12-55-10"></a>
### RG12.55.10, RG12.55.12 — Which HT and twist functions the user items and the C++ items use (2026-11-03, #3178)
- Counted by scripts over the force, constraint and post-Newton functions of exudyn/symbolics/userItems.py (spring-
  damper, torsional, Cartesian and rigid-body spring-damper, sphere contact, revolute, prismatic, spherical joint,
  distance constraint) and over src/ImplObjects/CObjectJoint*, CObjectConnector*, CObjectContact* (34 files),
  src/System/CObjectConnector.cpp, src/ImplMarkers (19) and the rigid nodes (4), comments removed.
- **The user items** use the parts of HT and Twist directly: translation 15, rotation 16, velocity 10, angular
  velocity 5; the difference of the translations p1 - p0 10 times and of the velocities v1 - v0 5 times; a column of a
  rotation (an axis) 17, cross products 11, scalar products 24 - the joints. Of the methods and functions:
  `HT.Relative` 2 (torsional, rigid-body spring-damper), the composition `HT(A0 R0, p0)` 2, `GetRxyz`, `LogSE3`,
  `SetExpSE3` once (rigid-body spring-damper), `RelativeTwist` once (torsional, its angular part); `PointVelocity` and
  `LocalTwist` never - the sphere contact computes omega x r with a global offset by hand, the rigid-body spring-damper
  A^T (v1 - v0) and A^T (omega1 - omega0) by hand. The four joints write their jacobianUserFunction by hand, by rows
  of [p0, theta0, p1, theta1] (u x v, -u, d x u).
- **The C++ connectors and joints** compute on the marker data in the same terms: the joint frame A0 rotationMarker0
  10 times in 4 files (generic, revolute, prismatic, rigid-body spring-damper); A0^T (p1 - p0) 15 in 5; A0^T A1 6 in
  6; A0^T (v1 - v0) 6 in 4; A0^T (A1 w1 - A0 w0) 5 in 5; A w_local 22 in 7; the Tait-Bryan angles of the relative
  rotation 5 in 5; A skew(w) 4 in 2; ExpSE3 once; the HT class's PointVelocity, LocalTwist, RelativeTwist never. The
  relative velocity of the C++ items is A0^T (v1 - v0), not RelativeTwist (which subtracts omega0 x (p1 - p0)); the
  rigid-body spring-damper of the user items follows the C++. The Jacobians come from the connector interface:
  `ComputeConstraintEquationsRigidDiff` and `ComputeConnectorForceRigidDiff`, the equations or forces of the
  MarkerRigid frames and twists differentiated by AutoDiff by the 6 variations of each marker and chained with the
  marker Jacobians in CObjectConnector.cpp (5 joints, 12 connectors and contacts) - the counterpart of what the user
  joints write by hand.
- **The rigid-body coordinates**: each rigid node has GetPositionJacobian, GetRotationJacobian (A G_local for Rxyz, EP,
  RotVecLG) and GetRotationJacobianTTimesVector_q / GetGlocalTv_q, the derivative of J_rot^T v (3 each in the nodes
  Rxyz, EP, RotVecLG, 2 in CObjectRigidBody); the markers compose them with their local position, J_pos - A r~ J_rot,
  and 14 markers implement ComputeMarkerDataJacobianDerivative, each by itself; G, G_local and their time derivatives
  18 times in 3 nodes. The user rigid body writes the same: [I, -A r~ G], [0, A G_local].
- **Proposed** (sub-step RG12.55.13, for decision): stay - HT with rotation, translation, composition, Inverse,
  Relative, GetRxyz, LogSE3, SetExpSE3; Twist with velocity and angular velocity; RelativeTwist and PointVelocity
  (the public functions of cUtilities). New: the difference of two twists, `twist1 - twist0`, so that A0^T (v1 - v0)
  is `LocalTwist(HT0, twist1 - twist0)`; the Jacobian of a user joint by the variations of its marker frames, from
  its constraint function by esym.Jacobian (A -> (I + skew(theta)) A, p -> p + dp, at zero), the symbolic counterpart
  of the connector interface, so that a user joint is written by its constraints alone and the four hand-written
  jacobianUserFunction go. Not as HT methods: the axes of a rotation, scalar and cross products (matrix operations).
  For C++, a candidate, not proposed now: one function of the point Jacobian of a frame composition and its
  derivative for the 14 marker Jacobian derivatives.
- No issue was raised for RG12.55.13: two proposed issues of a contributor wait in issues/proposed with their proposed
  resolutions, and every raise would raise them before `exudev issue apply-proposed`.

<a id="rg12-67-4"></a>
### RG12.67.4 — The parameters as an input of the composed system; RG12.67 closed (2026-11-03, #3159)
- Maintainer 2026-11-03: do RG12.67.4 now; RG12.67.3.2 (the symbolic frames) closed without workarounds - user
  markers should resolve it -, RG12.67.5 (forward sensitivities) abandoned: beside adjoint methods, numerical
  sensitivities are simpler and not much slower; sensitivities may come with advanced solvers.
- `SymbolicSystem._ParameterBackend(variables)`: with parameter bindings the input `p` of the values of the parameters
  (the order of `parameterNames`, those of `ParameterBindings.Parameters()`) is added to the inputs of the composed
  system and of the item functions (`_ItemFunction` of the connectors and joints, `_DAEConstraintFunction`); the
  bound expressions in the components of p by one `esym.Function` of all bound expressions in the named variables,
  written as a Python function by codeExport and called with `p[k]` (the substitution of RG12.60). `_ParameterVector`
  takes these lists from the backend.
- `SetParameterValues`: the variables, `ParameterBindings.Apply`, the values of p, and the numbers of the tables of the
  evaluation item by item - the user bodies, the connectors, joints and DAE objects, the markers on user bodies, the
  loads on user bodies (a gravity load reads the mass of its body) -; no rebuild: 0.04 ms against 6.1 ms of building
  the system of symbolicParameterJacobianTest.py.
- `ODE2RHSParameterJacobian(q, q_t, t, y)` (nODE2 x np) and `MassMatrixParameterJacobian(q, q_t, t)` (np x nODE2 x
  nODE2), each one `EvaluateJacobian('p', ...)` of a composed recording; they need the composed system and parameters.
  The algebraic equations have no derivative by p (not needed now).
- symbolicParameterJacobianTest.py (new): a user rigid body of the inertia of rho and L on a revolute joint, two
  spring-dampers of k and k/2, a mass point of m; after SetParameterValues the forces, mass matrix, residual Jacobian,
  constraints and reaction forces of the composed and the item-by-item system equal those of systems built anew to
  the bit (0.0), the recordings the same objects; the derivatives by the four parameters within 1.5e-10 (relative) of
  central differences. symbolicSystemParametersTest.py and symbolicRigidBodyInertiaTest.py unchanged.

<a id="rg12-67-3-1"></a>
### RG12.67.3.1 — The symbolic inertia and the symbolic arguments of the Create functions (2026-11-02, #3159)
- `RigidBodyInertia` and the shapes (InertiaCuboid, RodX, MassPoint, Sphere, HollowSphere, Cylinder), Translated,
  Rotated, Transformed, +, InertiaCOM and SetWithCOMinertia with symbolic values (decision P3): numpy arrays of
  esym.Real components, the products component by component - numpy's operations with an esym.Real operand give an
  esym.Matrix (#3015), which has no shape -; the checks (triangle inequality, symmetry, com = 0 for Rotated, the
  volume of a sphere) and GetGraphics on the present values. The numbers path is unchanged (float arrays), the test
  suite unchanged.
- UserItemCreator.CreateRigidBody puts the expressions into parameterBindings, centerOfMass of the user body the
  numbers. The other Create functions take expressions where their arguments are parameters of the user item: probed
  with the Cartesian, torsional and rigid-body spring-dampers (vectors and matrices of expressions), the sphere
  contact (radii, stiffness), the spring-damper and the distance constraint (whose check of the distance 0 now takes
  the value); the forces of Equations() equal the system's.
- symbolicRigidBodyInertiaTest.py: the inertia of expressions equals the one of numbers (0.0); a rigid body with the
  inertia of rho and L on a spring-damper, Equations() against the system 3.6e-15, the derivatives by rho and L
  within 1.2e-11 (relative) of central differences by SetParameterValues.
- Not done (sub-step RG12.67.3.2): the frames of the joints and connectors are the local frames of their markers,
  numbers.

<a id="rg9-11"></a>
### RG9.11.13.1, RG9.11.13.2, RG9.11 — The floating base and the Jacobian of ObjectKinematicTree; the revision of the system Jacobians closed (2026-11-02, #1549, #3211, #3201)
- **Floating base** (#1549): `baseNodeNumber` (a 3D rigid body node, the invalid index without), node 1 of the tree,
  its coordinates after the joint coordinates; `BaseMotion`, the root links in the base by `baseOffset` (also in the
  dynamics now, where it is needed with a moving base); the forces with the bias acceleration of the base and the
  forces of the root links on its coordinates by the transposed motion subspace (A f, G_local^T n); the mass matrix
  with the composite inertia of the tree in the base and the couplings; the marker Jacobians with the columns of the
  base; `CMarker::HasJacobianDerivative` (virtual, the type by default) false for MarkerKinematicTreeRigid on a tree
  with a floating base, so the connectors on it are differentiated by the system; the cache with the coordinates of
  both nodes; the requested node types checked in CheckPreAssembleConsistency. kinematicTreeFloatingBaseTest.py: a
  base body with two revolute links and a spring-damper on the tip against three rigid bodies with revolute joints,
  on the Lie group node, Euler parameters and Tait-Bryan angles: 3e-6 at 1600 steps of generalized-alpha, 1.9e-7 at
  6400 (the error of the integration, 16 times less for 4 times the steps); RK44 on the Lie group node 3.6e-10 from a
  fine reference (without the spring-damper).
- **The Jacobian** (#3211), measured first (the performance model is the chain of the measurements of RG9.11.13.3,
  no PerformanceModels file): the system's numerical Jacobian cost 2n evaluations of the tree anyway, the gain is in
  the d(M q_tt)/dq of a long chain. `ComputeJacobianODE2_ODE2` by forward differences of the inverse dynamics
  (`ComputeMassMatrixAndODE2LHS` with given coordinates and accelerations) in thread-local temporaries, in the parallel
  assembly; `jacobianMassMatrixDerivative` True by default (maintainer 2026-11-02: the derivatives of M(q) q_tt are
  not to be neglected). Generalized-alpha, full Newton, a spring-damper per link, without / with the term: 20 links
  664 / 571 iterations, 60 links 1591 / 317 iterations, 6.07 / 1.18 s, with four threads 4.27 / 0.46 s. The 60-link
  chain is ill-conditioned: its end coordinate moves by 3 % with relativeTolerance 1e-8 or 1e-10, with and without
  the term. With a floating base or a forceUserFunction the Jacobian of the system; an analytic derivative of the
  recursion is not done (the measured gain is the term). test_systemJacobians.py: the tree of the comparisons
  without the term, and a test that the difference of the two Jacobians is d(M q_tt)/dq of the mass matrix.
- Five test models change below 2e-10 (the Newton iterates); parameterConversionTestReference.txt with the two new
  parameters.
- **RG9.11 closed** (#3201): every object provides its local Jacobian - analytic, automatic, on the connector
  interface or differentiated by the system by its own coordinates -, assembled in parallel for the ODE2 and the
  algebraic rows. Measured per step of the revision: an ANCF cable of 2000 elements 4.6 instead of 20.9 ms per
  Jacobian with 8 threads; 2000 joints 0.34 instead of 2.44 ms; 2000 rigid bodies 0.39 instead of 1.57 ms, 0.06 ms with
  8 threads; perfObjectFFRFreducedOrder with 24 modes 6.85 instead of 13.4 s; a tree of six links with spring-dampers
  and contacts 0.38 instead of 1.08 s; HydraulicActuator2Arms 1.00 instead of 1.23 s; 200 bodies on Lie group nodes
  1.24 instead of 1.96 s; 60 links of a tree 1.18 instead of 6.07 s. The performance tests (runPerformanceTests.py,
  where the Jacobians are a small part): total 48.6 s at 1.13.331 against 46.3 s at 1.13.309 and 45.2 s at 1.13.190,
  the files within their scatter except generalContactSpheresPerf, 7.46 s at 1.13.190 and 9.68 s at 1.13.309 (9.3 s
  standalone now, the ODE2RHS of the contact 3.96 / 4.86 s with one thread) - a slowdown before this group's last
  steps, raised as an issue. Left open with their issues, as steps closed: the ANCFBeam (#3213, low priority) and
  MarkerSuperElementRigid (#3215, to be revised).

<a id="rg3-46-3"></a>
### RG3.46.3 — symbolicSystem.md and symbolicUserItems.md into the manual and the internals; itemPrecomputation.md (2026-11-02, #3221)
- New docs/dev/internals/symbolicSystem.md: the representation (a flat program per kind, the LTG tables), what is
  extracted from mbs and how a state goes back, the table of the items assembled, the composed system and its
  measurement, the parameters, the Jacobians, the interface SymbolicSystemBase, the solvers and their measurements,
  the decisions; in the toctree of docs/dev/README.md. The chapter Symbolic system of the manual got a paragraph on the
  representation (with a note that the internals are on GitHub) and the parameters as expressions.
- symbolicUserItems.md into docs/dev/internals/userItems.md section 7 (UserItemCreator, the table of the items, the
  forceUserFunction of the marker states and why not the ConnectorMarker form) and its decisions into section 8, the
  section 6 of the design (a C++ item with symbolic parameters, not planned) among them; the design file removed.
- symbolicSystem.md reduced to the sections cited by uniformEvaluation.md, cudaRoadmap.md (both of a colleague, not
  touched) and symbolicParameters.md: 5, 10, 12.3, 12.4 and the codes of the vector and matrix operations of 12.7
  with the proposal for sparse matrices (#3168); 179 of 758 lines.
- itemPrecomputation.md (RG9.11.13.3): the options not chosen and the measurement into
  docs/dev/internals/userFunctions.md, the design file removed (maintainer 2026-11-02).
- git keeps the removed files; their last version is in 459fd578.

<a id="rg12-73-2"></a>
### RG12.73.2, RG12.67.2 — The scope of the model for the named symbolic variables; the bindings of the parameters of the user items (2026-11-02, #3208, #3159, #3207)
- Decided (maintainer 2026-11-02): package 1 of symbolicVariableNames.md with `mbs.symbolicVariables`.
- `MainSystem::symbolicVariables`, a Python object (an `esym.VariableSet`) created when first used, so that MainSystem
  needs no header of the symbolic module; `Reset` sets it to None, `GetDictionary`/`SetDictionary` keep the names and
  values; `mbs.symbolicVariables` a read-only property.
- `esym.Real(name, value, scope=None)`, `esym.Vector` and `esym.Matrix` likewise: the variable of the name in the
  scope (None: `esym.variables`, a MainSystem, a VariableSet), a new one or the one there with the value set. Found by
  the test suite: a name of another kind or size is replaced, not refused - symbolicUtilitiesTest, symbolicModuleTest
  and the reference notebook symbolic.ipynb use one name for a Real and a Vector -; with recording off a named Real is
  a number, as before (symbolicModuleTest). Creating a named variable while a user function is recorded raises.
- `_CheckVariablesOfRecording` in ConvertFunctionToSymbolic: every named variable of a recording is an argument or the
  variable of its name in `mbs.symbolicVariables`, the same node (`NamedVariablesOf`). The arguments of a recording
  are created in a VariableSet of their own; so are the library's names in shells.py and the backend of
  SymbolicSystem.Equations. `_SymbolicMainSystem.symbolicVariables` for the recording; codeExport's format symbolic
  reads `mbs.symbolicVariables` in a user function.
- `exudyn.special.deprecations.symbolicVariablesWithoutScope` restores the old variables; #3231 removes it later.
- The named variables of `esym.variables` now live as long as the set: symbolicOptimizeTest (live nodes) uses a scope of
  its own, symbolicModuleTest resets `esym.variables` before its counters; symbolicExportTest and symbolicFunctionTest
  adapted.
- `ParameterBindings` (userItems.py): per item the list of numbers and expressions, `Keep`, `Parameters` (the
  variables of `mbs.symbolicVariables` in them, each once, `_VariablesNotInScope`), `Check`, `Values`, `Apply`;
  `UserItemCreator._AddUserItem(kind, item, parameters, variant)` in every Create function; `parameterBindings`
  instead of `parameterVariables`; `_massPointNodes` removed (#3207) - no test result changed. SymbolicSystem takes the
  bindings (`parameterBindings`), expressions as parameters, sympy by codeExport.ToSympy.
- Test models: symbolicVariableScopeTest.py (new), symbolicSystemParametersTest.py with `stiffness=k/2 + 0.1*m1`: the
  derivatives by m1 and k within 7e-11 of central differences by SetParameterValues.

<a id="rg9-11-13-3"></a>
### RG9.11.13.3 — The frames of ObjectKinematicTree in a cache validated by its coordinates, with CSystem::PreComputeItems (2026-11-02, #3229)
- As decided (maintainer 2026-11-02, design docs/dev/design/itemPrecomputation.md section 3): options C and D together,
  no thread buffers; a failed comparison inside a parallel loop raises; the comparison always with the velocities and
  accelerations; only ObjectKinematicTree. Sub-step 1 (thread buffers) not done, 4 (the object's own LHS and mass
  matrix on the cache) evaluated without a gain: they need the placements in the parents and the bias accelerations
  with gravity, once per tree.
- `CObjectKinematicTree::PreCompute` with `treeCacheCoordinates` (4n numbers) and `treeCacheValid`, reset by
  `ParametersHaveChanged` and at Assemble; `GetTreeTransformations` for the Get...KinematicTree functions (the other
  configurations in thread-local temporaries, the visualization in its members); the chain of a link thread-local.
- `CObject::HasPreCompute`/`PreCompute`, `cSystemData.listPreComputeObjects` (with a marker on the object),
  `CSystem::PreComputeItems` in the six system functions instead of UpdateUserFunctionsForThreads, and before the
  parallel parts of JacobianODE2RHS and JacobianAE and the Jacobian of the general contacts. The deferral of
  RG9.11.14 removed.
- `ExuThreading::TaskManager::JobIsRunning`; `CreateJob` threw an exception of its own part before waiting for the
  other threads, and an exception of a worker thread was lost if it came later - it now waits, then throws. The
  constructors of `ExuThreading::Exception` were `inline` in Threading.cpp, so no other file could throw it.
- Measured (RK44, 2000 steps, a CartesianSpringDamper per link), before / after: 6 links 0.131 / 0.052 s, 20 links 1.22
  / 0.265 s, 60 links 11.0 / 2.13 s; the tree alone 0.026, 0.148, 1.38 s. Generalized-alpha, full Newton, 200 steps, 20
  links: 664 iterations, 0.165 s with one thread, 0.126 s with four. 60 links take some 30 Newton iterations per step,
  with the local as with the system-wide Jacobian (1591 / 1589 in 50 steps), so not a matter of the cache.
- New test model kinematicTreeMarkerCacheTest.py: six links with a spring-damper each and a spherical joint, one and
  four threads with the parallel limits at 1 the same within 1e-14; SetObjectParameter of baseOffset moves the markers.

<a id="rg11-6-1"></a>
### RG11.6.1 — localPosition of the rigid body markers and rotationMarker0/1 documented as deprecated (2026-11-02, #3179)
- The descriptions of rotationMarker0/1 of ObjectConnectorRigidBodySpringDamper, ObjectConnectorTorsionalSpringDamper,
  ObjectJointGeneric, ObjectJointRevoluteZ and ObjectJointPrismaticX and of localPosition of MarkerBodyRigid and
  MarkerKinematicTreeRigid say that they are deprecated and will be removed, pointing to localHT; no Deprecated(...)
  in the definitions, so no warning (maintainer 2026-10-31). The offset of MarkerSuperElementRigid, also a part of its
  localHT, is not touched (MarkerSuperElementRigid will be revised, RG9.11.17.3).

<a id="rg9-11-17-2"></a>
### RG9.11.17.2 — The Lie group node in the local Jacobians (2026-11-02, #3217)
- The only Lie group node is NodeRigidBodyRotVecLG; NodeRigidBodyRotVecDataLG, named in #3217, does not exist.
- RigidBodyNodeVelocity_q on the node: the velocity coordinates of the rotation are omega (local), the composition is
  local, A Exp(delta) (CompositionRotationVector), so the angular velocity and the velocity of a point have the
  columns -A omega~ and -A (omega x p)~; MarkerNodeRigid, MarkerBodyRigid and MarkerBodyPosition on such bodies give
  dv/dq, so the contacts with friction on them are on the connector interface.
- CObjectRigidBody::ComputeJacobianODE2_ODE2LieGroup in closed form (the description of the function); the function
  flags in GetAvailableJacobians, so the bodies are in the parallel assembly.
- Found by the new test: JacobianODE2Loads differentiated by x + eps also on the Lie group nodes, the system-wide
  Jacobian by the composition; the mass proportional load of a body with a center of mass differed by 0.34 (relative).
  It now uses SetNumDiffObjectLieGroup, the same values elsewhere.
- test_systemJacobians: FreeBodiesLieGroup (with and without a center of mass) and ContactsLieGroup, also at
  velocities; within the tolerance.
- Measured: the test models with Lie group nodes (createSphereQuadContact, createSphereTriangleContact,
  generalContactImplicit2, sliderCrank3Dbenchmark) the same Newton iterations, up to 3 % faster; a chain of 200 bodies
  with spherical joints, full Newton, 200 steps: 1.96 s / 1.24 s (1269 / 1262 iterations), 0.83 s with 4 threads; the
  test suite unchanged.

<a id="rg9-11-10"></a>
### RG9.11.10 — Evaluated: the dependencies outside the local-to-global lists (2026-11-02, #3201)
- Where they come from: the C++ items see their coordinates through their nodes and markers, which are in their
  local-to-global lists - also the connectors, whose lists are those of their markers; the symbolic user functions get
  only their arguments (a recording cannot read a sensor or another item), so their recordings have no hidden
  dependency; loads declare theirs with AddODE2LoadDependencies and AddODE1LoadDependencies (#1548). What remains are
  the Python user functions of objects that read mbs - GetSensorValues, Get...Output, the coordinates of other items.
- Measured: a mass point on a spring-damper whose springForceUserFunction adds gain times the position of another mass
  point, read by a sensor (full Newton, 1000 steps): 1999 to 2000 Newton iterations with the local Jacobians, 1000 to
  1010 with doSystemWideDifferentiation, for gains 1e2 to 1e4; the same results, the same time for this small model.
- Options: (a) declared, mbs.systemData.AddObjectODE2Dependencies(objectNumber, coordinates) as for the loads, the
  columns added to the numerical differentiation of the object, which a Python user function has anyway - small and
  exact, the user knows what the function reads; (b) the same declared by sensors, the coordinates of the sensor's
  item looked up; (c) found automatically by evaluating the user functions once with the reads of mbs recorded - not
  robust: reads depend on branches and on time, and the MainSystem functions are C++ methods; (d) from the recordings -
  nothing to find, see above.
- Decision proposed: (a), with (b) as a convenience, when a model needs it; doSystemWideDifferentiation is the remedy
  now and the manual says so with the measurement. No step or issue opened (maintainer 2026-11-02).

<a id="rg9-11-18"></a>
### RG9.11.18 — The derivative of the reaction forces as an option of the Newton Jacobian (2026-11-02, #3230)
- d(M q_tt)/dq: not in the Jacobians (the solver adds M, the system-wide numerical Jacobian differentiates the right-hand
  side only); zero for the bodies with a constant mass matrix; its effect is measured by #3228 (maintainer: low
  priority, no step).
- newton.numericalDifferentiation.constraintReactionDerivative (False): CSystem::JacobianODE2ReactionForces at the end
  of JacobianODE2RHS, the reaction forces of each object with algebraic equations (ComputeObjectReactionForces) and of
  the nodes with algebraic equations at the current Lagrange multipliers, differentiated by their own coordinates
  (SetNumDiffObjectLieGroup) and velocities, serially, with -factorODE2 and -factorODE2_t; so also in the static solver
  and with doSystemWideDifferentiation.
- test_systemJacobians: test_theReactionDerivativeIsTheDerivativeOfTheReactionForces (Joints, JointsEP,
  EulerParameters, Lagrange multipliers 100 sin(1 + i)) against central differences of C_q^T lambda from the ODE2-AE
  block; the harness sets the Lagrange multipliers.
- Measured, full / modified Newton, off / on (Newton iterations, solver time): fourBarMechanismIftomm 1058 / 802 it,
  0.023 / 0.031 s (full), 2739 / 2719 (modified); bricardMechanism 1301 / 1302, 0.11 / 0.23 s; sliderCrank3Dbenchmark
  8190 / 8186, 0.22 / 0.57 s; serialRobotTest, rigidBodyCOMtest, sphericalJointTest, kinematicTreeAndMBStest,
  superElementRigidJointTest, revoluteJointPrismaticJointTest within 3 iterations, 1.4 to 2.6 times the time with full
  Newton, unchanged with modified Newton; the results within the Newton tolerance. Off by default, as decided.
- The parallel, analytic alternative (the marker Jacobian derivative for joints whose reaction forces are a marker
  Jacobian times a constant direction) is not done; with these measurements it is not worth its code now.

<a id="rg9-11-14"></a>
### RG9.11.14 — The ODE2 Jacobian of ObjectFFRFreducedOrder (2026-11-02, #3212, #3229)
- CObjectFFRFreducedOrder::ComputeQuadraticVelocityVector(A, Glocal, omega, zeta, zeta_t), moved out of ComputeODE2LHS
  unchanged (the unused G_t terms of the rotation parameter velocities removed); ComputeJacobianODE2_ODE2 with K and D,
  the quadratic velocity terms by zeta and zeta_t analytic, by the rotation parameters and their velocities by forward
  differences of the vector at the rotation and G of EP2RotationMatrix/EP2Glocal or RotXYZ2RotationMatrix/
  RotXYZ2GlocalTemplate; GetAvailableJacobians with the functions on Euler parameters and Tait-Bryan angles without a
  forceUserFunction; the parameter numericalJacobian (False), the switch back to the numerical Jacobian of the system.
- test_systemJacobians: FFRFreducedOrder and FFRFreducedOrderRxyz (the netgen test mesh, four Hurty-Craig-Bampton
  modes, turning, on a spring at its left face), also at velocities; the object's block within 1e-8.
- Measured with numericalJacobian True / False on perfObjectFFRFreducedOrder (0.2 s, 2000 steps): 8 modes 0.35 / 0.32 s
  (modified Newton), 1.13 / 0.92 s (full); 24 modes 2.40 / 1.63 s, 13.40 / 6.85 s; the same Newton iterations (4039 /
  4049, 5771 / 5767 with full Newton). The maintainer's simplification of the quadratic velocity terms is not needed.
- parameterConversionTestReference.txt rewritten for numericalJacobian.
- #3229: a full pytest run failed once in test_theParallelODE2JacobianIsTheSerialOne[KinematicTree], the race of the
  connectors on the tree's markers since RG9.11.17.1; CSystem::ComputeLocalJacobianODE2 defers connectors on
  MarkerKinematicTreeRigid in the parallel assembly to the serial part (five runs pass). The AE rows and the residual
  still compute them in parallel; #3229 stays open.

<a id="rg9-11-17-1"></a>
### RG9.11.17.1 — MarkerKinematicTreeRigid with dv/dq and the Jacobian derivative (2026-11-02, #3216)
- CObjectKinematicTree: GetLinkChain, JacobianColumnDerivative (column i of the position and rotation Jacobians by the
  coordinate of joint j of the chain: a revolute joint j above i turns the columns about a_j, a prismatic one leaves
  them, a joint at or below i moves the point, which changes a_i x (p - p_i) of a revolute i), and from it
  GetVelocity_qKinematicTree (3 x n, times q_t) and GetJacobianTransposedTimesVectorDerivativeKinematicTree (n x n).
- MarkerKinematicTreeRigid: ComputeMarkerDataJacobianDerivative instead of the error, GetVelocity_q, the types
  JacobianDerivativeAvailable and JacobianDerivativeNonZero; the description of its Jacobians without "not
  implemented".
- test_systemJacobians: model KinematicTree (revolute, prismatic, revolute and a branch; a Cartesian and a rigid-body
  spring-damper and a sphere contact with friction on the markers, the spring-dampers without damping, whose dv/dq the
  connector interface leaves out); within 2e-8 of the system-wide Jacobian, also at velocities; with
  jacobianConnectorDerivative = False 0.14.
- Measured in one environment (a chain of six revolute links, a Cartesian spring-damper on each, 2000 steps), old /
  new: without contacts 13351 / 13352 Newton iterations and 0.24 / 0.22 s (modified Newton), 4000 / 4000 and 0.71 /
  0.18 s (full); with a sphere contact with friction on each link 15298 / 15588 and 0.39 / 0.36 s, 4269 / 4271 and
  1.08 / 0.38 s; the test suite unchanged.
- Found: the marker functions of the tree write its mutable temporaries (jointTransformationsTemp, links,
  tempArrayIndex), so connectors on its markers race in parallel threads, as in the residual before; #3229 raised.

### RG9.11.16 — The Jacobian of the pressure equations of ConnectorHydraulicActuatorSimple, analytic (2026-11-02, #3214)
- CObjectConnector::ComputeJacobianODE1RHSConnector (false by default), called by CSystem::ComputeLocalJacobianODE1
  for connectors with ODE1_ODE1_function; the hydraulic actuator gives p_i,t = k_i(s) (a_i s_t + Q_i(p_i)) by the
  pressures, by the stroke s (k_i = K_i/V_i by AutoDiff<1> of ComputeBulkModulusPerVolume, which ComputeODE1RHS uses as
  well) and by its velocity, chained with the position Jacobians, the direction along the Euler parameters and dv/dq of
  the markers; ComputeValveFlow with the derivative of the square root limited at 1e-8 times the system pressure.
- test_systemJacobians: the ODE1 rows also at velocities (dv/dq); Hydraulic within 4e-7 of the system-wide numerical
  Jacobian, block by block.
- Measured in one environment, old / new: hydraulicActuatorSimpleTest 962 / 962 Newton iterations (modified Newton),
  445 / 445 (full); HydraulicActuator2Arms 177292 / 177200 (modified), 57290 / 57293 and 1.23 / 1.00 s (full);
  HydraulicActuatorStaticInitialization 3743 / 3743, 1394 / 1394; the results within the Newton tolerance; the test
  suite unchanged.

<a id="rg3-47"></a>
### RG3.47 — The chapter Symbolic system and user functions (2026-11-02, #3222, #3225, #3226)
- docs/manual/symbolic.md, the chapter at the place of symbolicUserFunctions and userItems in index.md and
  pdfIndex.md, with symbolicSystem.md (what SymbolicSystem is, what works - a small part of the C++ features, extended
  continuously, for experimentation, teaching and the export of simpler models, easily patched in Python -, how to build
  it with UserItemCreator, what it gives, the notebook), symbolicUserFunctions.md, userItems.md (a new section on
  UserItemCreator) and symbolicSolvers.md (the two solvers, the settings they take, the replicate whose solution is
  written into mbs). The manual named neither UserItemCreator nor SymbolicSystem before.
- python/Notebooks/tutorialSymbolicSystem.ipynb (#3225): two mass points on spring-dampers by UserItemCreator, the
  equations as sympy expressions, the forces at the initial state, SolveGeneralizedAlpha with the solution file, the
  solution viewer and PlotSensor on mbs, against SolveDynamic 7e-10, the flat program; its page and its example script
  generated by notebookEmitter.
- The header of systemModel.py shortened to the use of the module, without the list of issues.
- #3226 raised and resolved: runNotebooks.py sets EXUDYN_NO_USER_SETTINGS, so that the stored outputs do not carry the
  note of the maintainer's config.json (HT.ipynb had it).
- Not yet: symbolicSystem.md and symbolicUserItems.md into the chapter and the internals (RG3.46.3).
