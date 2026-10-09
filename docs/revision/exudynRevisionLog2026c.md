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
