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
- A list in a pybind description is a Markdown list; checkDefinitions refuses `i ... \ei` there.

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
