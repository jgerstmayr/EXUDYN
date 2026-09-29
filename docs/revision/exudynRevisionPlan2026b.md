# Exudyn Revision Plan 2026b

The continuation of the 2026 revision. The first plan
([`exudynRevisionPlan2026.md`](exudynRevisionPlan2026.md), log
[`exudynRevisionLog2026.md`](exudynRevisionLog2026.md)) completed as **1.12** and is now a record;
what it had not finished is here, together with everything that comes next.

Facts, decisions, invariants and the material for the documentation stay in the **shared** info
document [`exudynRevisionInfo2026.md`](exudynRevisionInfo2026.md) - there is one set of facts about
this repository, not two. What was done and found in THIS plan goes into
[`exudynRevisionLog2026b.md`](exudynRevisionLog2026b.md).

## Groups instead of phases, and how they are numbered

The first plan was a sequence with an end: phases R0 to R11, worked roughly in order, finishing at
1.12. What is left is not a sequence - a bug in contact friction, a rendering revision and a
plugin ABI are not stages of one march, they are **themes that run in parallel for years**. So
this plan is organised by group.

- **Numbers are permanent**: `RG<group>.<step>`, sub-steps `RG8.3.1`, further splits `RG8.3.1a`.
  A number is given when the step is planned and never changes; steps are appended to their group.
- **Cite them** as "revision2026b group RG1" and "revision2026b step RG8.3.1" - and a step of the
  FIRST plan is always cited as "revision2026 step R4.3", never as a bare number: a bare
  "step 3.4" could be R3.4 of the finished plan or RG3.4 of this one, and the prefix is what
  keeps them apart.
- **In code** a comment cites the **issue number** (`#2411`), as before: comments live for years,
  issue numbers are stable references and plan numbers are not.
- A done step keeps one line here - status, date, outcome, link to the log; an open step keeps its
  full text.
- **A new group or a new top-level step is proposed to the maintainer before it is written down**;
  sub-steps may be added directly.

The last section, **Next steps recommended**, is the answer to *what now*: the open steps and
what the current work raised, with the numbers to look them up by. It is updated from time to
time and copies nothing.


Each step that came from the first plan says so: *(revision2026 step R9.1)*. The finished plan
carries the same table from its side, so a citation from either direction resolves.

## The groups

The open steps of a group are the ones without **DONE**; there is no count here, because a
hand-maintained one is wrong the next day (the rule of RG3.9).

| group | what belongs in it |
|---|---|
| **RG1** Release and publication | getting a release out and onto GitHub and PyPI |
| **RG2** Testing and verification | what is not tested, and who tests it before a release |
| **RG3** Docs | what the documentation still gets wrong or does not say |
| **RG4** Implementation problems and bugs | real, reproducible problems that need a plan rather than a fix |
| **RG5** Performance | measurement first, then the code that is actually hot |
| **RG6** Graphics and rendering | the renderer, the settings dialogs, and the rendering revision it is heading for |
| **RG7** Python user items | items whose behaviour is written in Python |
| **RG8** Compiled C++ user items | plugins: user items compiled against the shipped headers |
| **RG9** Structural core improvements | the architecture of the core, where a change touches everything |
| **RG10** Tooling and process | exudev, the issue tracker, the generators, CI |
| **RG11** Misc | what has no group yet; three of a kind become a group |
| **RG12** Python interface | the shape of the Python API itself: deprecation, settings, what a script sees |
| **RG13** Item documentation | a full documentation and a MiniExample for every item |
| **RG14** Marker values computed where they are used | connectors, constraints and loads compute their marker values themselves; for automatic differentiation |
| **RG15** Objects computing from given coordinates | bodies and finite elements take their coordinates as arguments; for automatic differentiation |

## RG1 — Release and publication

What it takes to put a version in front of people, and the state of that question today: **1.12
completes the 2026 revision and stays internal** - no PyPI wheels, no GitHub release, no
promotion (info document D16) - and **1.13 is the first public release**. GitHub `master` is
frozen at 1.11.0 until then, which `tools/hooks/pre-push` enforces; the colleagues work on the
internal GitLab meanwhile.

<a id="rg1-1"></a>
**RG1.1** *(group RG1; revision2026 step R1.8)* **At the 1.13 release: fast-forward `master`, push once with tags.** Ordinary push; clones, permalinks and
    issue references stay valid; GitHub renders the layout change as renames.

<a id="rg1-2"></a>
**RG1.2** *(group RG1; revision2026 step R1.9)* Retroactively tag past releases where the commits can be identified.

<a id="rg1-3"></a>
**RG1.3** *(group RG1, when there is material; revision2026 step R1.10)* **Second internal GitLab repository for development-only
    Python.** Models that never make it to `Examples`, one-off study scripts, and internal
    experiments live there rather than in the public tree. Not created yet — do it when there is
    something to put in it, not before. Note the consequence for revision2026 step R1.4: with a destination that
    is a *repository*, the sibling-directory and nested-repo shapes stop being the answer, and
    `experimental/` in `.gitignore` is only a safety net for work in progress.

<a id="rg1-4"></a>
**RG1.4** *(group RG1; maintainer decision 2026-09-22)* **The 1.13 release.** The first public
    release after the revision, and the only one that is announced. What has to be true before it
    is built:

    - the **integration round** of the colleagues is through (RG2.2), and what it found is either
      fixed or recorded as an issue with a decision;
    - **macOS wheels** exist and pass - the machine is expected around 2026-10-20;
    - the documentation has been **read by somebody who did not write it**;
    - the **manual GUI check** (RG2.4, `docs/dev/GUI_MANUAL_CHECK.md`) is done on Windows, Ubuntu and
      macOS;
    - `exudev release` runs clean: its readiness step (revision2026 step R8.2) checks the issue
      store, the published pages, a clean working tree and a free tag, and `--tag` writes the
      annotated tag with `dist/RELEASE_NOTES.md` as its message.

    Then: `exudev issue mode --release`, `exudev issue bump --minor` (1.13 needs a name - the
    release names are jazz legends in alphabetical order, and `releases.json` plans none beyond
    1.14), the release build over every Python version, the wheels to PyPI, the tag and the
    release notes to GitHub, and `master` fast-forwarded (RG1.1).


## RG2 — Testing and verification

The suite is 116 test models, 23 mini examples, 171 examples and the pytest set, run on five
Python versions and three platforms (revision2026 phase R5). This group is about what that does
NOT cover, and about the testing that no suite can do.

<a id="rg2-1"></a>
**RG2.1** *(group RG2, added 2026-09-20; revision2026 step R5.18.9)* **The drawing code is
    exercised by exactly one test model** (#2562).

    **The step was written on a wrong premise and is corrected here** (maintainer, 2026-09-22):
    it said that *nothing* in the test suite ever calls `UpdateGraphics`. In fact
    `python/TestModels/raytracerNOGLFWtest.py` runs in the suite against a reference checksum and
    calls `SC.renderer.RedrawAndGetImage(useRaytracer=True)`, which goes through
    `MainRenderer::RedrawAndGetImage` to `VSC.UpdateGraphicsDataNow()` and
    `VSC.UpdateGraphicsData()` — so the `UpdateGraphics` of every visible item **is** executed,
    and its result enters a compared number.

    What is true is narrower, and is what remains of this step: **one model, one set of
    visualization settings, one checksum**. A checksum can only say "different"; it cannot say
    *what* changed, and it changes on any visualization change whether or not the change was
    wanted. The work that follows from it is RG2.3 (the suite) and RG6.3 (the API it needs).

<a id="rg2-2"></a>
**RG2.2** *(group RG2; maintainer decision 2026-09-22)* **The integration round before 1.13.**
    The colleagues at the institute integrate their own work against the current version on the
    internal GitLab and report what breaks. This is the testing that the suite cannot do: real
    models, written by people who did not write the change, on machines that are not the
    maintainer's.

    Every finding becomes an issue (`exudev issue raise`), so that the round leaves a record
    rather than a memory. The round is what RG1.4 waits for.


<a id="rg2-4"></a>
**RG2.4** *(group RG2; maintainer 2026-09-29)* **A manual check of the render window and the dialogs
    before a release** (#2748). The tests check what the renderer draws; nothing checks that a person can
    use the window and the dialogs - keys, mouse, tkinter windows, focus, fonts, placement. The check
    list is [docs/dev/GUI_MANUAL_CHECK.md](../dev/GUI_MANUAL_CHECK.md), about one hour per platform, with
    `python/testing/guiManualCheckModel.py`, which has an item of every kind and waits for the person.
    **The list and the model DONE 2026-09-29** — [log](exudynRevisionLog2026b.md#rg2-4); **the checks
    themselves** are done once per release on Windows, Ubuntu and macOS, and RG1.4 waits for them.

<a id="rg2-3"></a>
**RG2.3** *(group RG2; maintainer 2026-09-22)* **A graphics regression suite** (#2582). RG2.1
    leaves one model, one setting and one checksum. What is wanted: several models against
    several visualization settings - show and hide of nodes, markers, loads and sensors,
    different colours and text settings - compared as **low-resolution reference images** that a
    human can also look at, or as **counts taken from the graphics data** (triangles, lines,
    texts per item), or both: the counts say what changed, the images say whether it still looks
    right. Depends on RG6.3.

    Open in the tracker for this group besides these: **#2498** (nothing checks that an item type
    provides the member functions it must), **#2511** (the ROS examples were last run in 2023).

    - **RG2.3.1** **DONE 2026-09-27** (#2700) — [log](exudynRevisionLog2026b.md#rg2-3-1) — the
      maintainer: *"yes do that and totally remove the .txt graphics export"*. Built as
      `SC.renderer.GetGraphicsData()`; the TXT export, its four settings and `LoadImage` are gone.
      *(sub-step of RG2.3, #2582; measured 2026-09-27 on the maintainer's request:
      "check a specific older renderer export function which exports txt with the drawing elements -
      this could be json in future ... also used for testing")* **the drawing elements as data, and
      what the existing export cannot do.**

      **What exists.** `SC.visualizationSettings.exportImages.saveImageFormat = 'TXT'` writes the
      scene as text instead of as an image - `GlfwRenderer::SaveSceneToFile`,
      `src/Graphics/GlfwClient.cpp` - with `saveImageAsTextLines`, `...Circles`, `...Triangles` and
      `...Texts` selecting what goes in. The format is comment-marked sections: `#COLOR` then RGBA,
      `#LINE` then a polyline of `x, y, z` triplets, `#TRIANGLE` then three points, `#END` at the
      end. `exudyn.plot.LoadImage` reads it back into lists and `PlotImage` draws it with matplotlib;
      `NGsolveCraigBampton.py` and `NGsolvePistonEngine.py` are the only users, both behind an `if`.

      **The blocking fact, and it is the reason this is a sub-step and not a use of what is there**:
      `MainRenderer::RedrawAndSaveImage` calls `RendererInActiveError`, so the export needs the
      **running OpenGL renderer** - a window on somebody's screen. A test cannot ask for it:
      `EXUDYN_SUPPRESS_UI_WINDOW_OPEN=1` means there is no renderer at all.

      **The path that does work already exists.** `RedrawAndGetImage(useRaytracer=True)` runs with the
      renderer **inactive**: `MainRenderer` updates the post-processing data of every system, then
      `VSC.UpdateGraphicsDataNow()` and `VSC.UpdateGraphicsData()`, and hands the graphics data to the
      software renderer. `raytracerNOGLFWtest.py` does exactly this inside the test suite, so
      **building the graphics data without a window is proven, and only the serialization is missing**.

      **What the text format loses** - each of these is a reason to write the data rather than to
      extend the text:

      | lost | why it matters for a test |
      |---|---|
      | **`itemID`**, which **every** primitive carries (`GLLine`, `GLSphere`, `GLCircleXY`, `GLText`, `GLTriangle`) and `Index2ItemID` decodes into item type and index | it is the difference between *something changed* and *ObjectRigidBody 7 draws 12 triangles fewer*; it is the one field a graphics regression suite cannot do without |
      | **spheres**, not exported at all | `graphics.Sphere` and every node drawn as a sphere are invisible to the export |
      | **texts**, `PrintDelayed("SageImage: Text export not yet implemented!")` - and the message says SageImage | the text settings are among the things RG2.3 wants to vary |
      | triangle **normals** and the per-vertex colours; a line's `color2` | a shading or a contour-colour change would not show |
      | circles become polylines whose vertex count comes from `general.circleTiling` | a display setting leaks into the data, so the reference changes when the setting does |
      | no version, no counts, no fingerprint of the settings the scene was drawn with | a reference file cannot say what it is a reference *of* |

      **The suggestion, in one sentence**: a **JSON export of the graphics data, taken with the
      renderer inactive**, as the oracle of the suite - the counts RG2.3 asks for fall out of it
      (`len` per primitive per `itemID`), and the low-resolution reference images stay the human half
      of the comparison, from `RedrawAndGetImage(True)`, which the suite can already produce.

      Shape, to be decided with the maintainer:

      - **where it lives**: `VisualizationSystemContainer`, beside `UpdateGraphicsData`, so it is
        reachable with the renderer inactive; the GLFW path keeps TXT for what it is used for today.
      - **what Python sees**: `SC.renderer.GetGraphicsData(viewID=0)` returning a **dict** is the
        useful primitive - a test compares numbers and does not want a file - with
        `SaveGraphicsData(fileName)` writing the same thing as JSON for a reference that a human reads
        and `git diff` shows.
      - **what goes in**: the five primitive lists with every field, `itemID` **decoded** to
        `(itemType, itemIndex)` because a raw `Index` is not a stable thing to compare, plus a header
        with a format version, the per-primitive counts, and the settings that were in force.
      - **what stays out**: nothing derived from the window - no zoom, no model view, no screen size -
        or the data is not comparable between machines, which is the mistake the pixel checksum makes.
      - **the tolerance question**: a float coordinate compared exactly will fail across compilers, so
        the reference wants the counts per item exact and the coordinates rounded, and the rounding is
        a decision (the TXT export writes 8 digits of a `float`).
      - **whether `LoadImage`/`PlotImage` follow**: they read the TXT today; reading JSON as well is
        small, and it gives the suite's reference files a viewer for free.

      **What this replaces**: the only drawing test today is `raytracerNOGLFWtest.py`, one pixel
      checksum of one model, removed from the reference set on macOS because the offscreen path
      crashes there since 1.11.0. A count per item is portable in a way a checksum of pixels is not.

    - **RG2.3.2** *(sub-step of RG2.3; found in RG2.3.1)* **DONE 2026-09-27** —
      [log](exudynRevisionLog2026b.md#rg2-3-2) — **`PlotImage(plot3D=True)` fails with every
      current matplotlib** (#2701): `fig.gca(projection='3d')` was removed in matplotlib 3.6 -
      measured with 3.11.0, `TypeError` - and the 3D mode is the only one that draws triangles. Two
      slips in the same function: the 2D branch adds `p0[0]` to y and z, and the 3D branch ends a
      segment at `z[j]` instead of `z[j+1]`; neither shows with the default `HT`. Small, with a test
      that draws in 3D.

    - **RG2.3.3** *(sub-step of RG2.3; maintainer 2026-09-27)* **The graphics regression test**
      (#2704). *"File graphicdata test as step. It could include also metrics for positions, colors -
      mean/min/max - so the content is also checked."* Built **in sub-steps, each of them a test that
      runs**, so that whether the approach carries is seen after the first one and not after the last.

      **Decided (maintainer, 2026-09-27)**:

      | question | decision |
      |---|---|
      | tolerance of the metrics | **1e-5**, relative |
      | granularity | **per item** for a model of up to 32 items, **per item type** (nodes, objects, markers, loads, sensors) above |
      | references | `python/testing/graphicsReferences/`, one JSON file per case |

      **The fingerprint** of a case, from `SC.renderer.GetGraphicsData()`, per item or item type and
      per kind of element: the **number** of lines, spheres, circles, texts and triangles exactly; the
      **min, max and mean** of points (per coordinate), colours (per channel), radii and normals with
      the tolerance; texts as their strings. A change shows in `git diff` as *"object 3: triangles 12
      -> 10"*. The references are written by the test when asked, as `parameterConversionTest.py`
      does.

      **What it cannot see, measured 2026-09-27**: **sensor traces** are drawn by
      `GlfwRenderer::RenderSensorTraces` directly in OpenGL and are not in the graphics data at all; and
      the **raytracer does not draw `glSpheres`**, which `GetGraphicsData()` does return. Both are
      recorded, not worked around.

      - **RG2.3.3.1** **DONE 2026-09-27** — [log](exudynRevisionLog2026b.md#rg2-3-3-1) - **the
        machinery and the first cases** - the fingerprint, the comparison, the
        references, and the `graphics.*` functions: one `ObjectGround` per function (Sphere,
        Cylinder, Brick, Tube, Torus, Arrow, Basis, Frame, RigidLink, SolidOfRevolution,
        SolidExtrusion, Quad, CheckerBoard, Circle, Lines, Text, the gear parts, FromPointsAndTrigs and
        a small STL written by the test, and the transforms Move, Transform, MergeTriangleLists,
        InvertTriangles, AddEdgesAndSmoothenNormals). No solver. **This is the feasibility test**:
        size of the references, how stable the metrics are, how long it takes.
      - **RG2.3.3.2** **DONE 2026-09-27** — [log](exudynRevisionLog2026b.md#rg2-3-3-2) - **the
        settings on one representative model** - not every setting on every model, which grows too
        fast: one model carrying what the most used settings change - the basic edge and face
        features, show/hide of nodes, markers, loads and sensors, `showNumbers`, the tilings.
        `deformationScaleFactor` and the contour settings are **later**, they are special. Done as 28
        variants of one model of 21 items, each stored as what it changes; **the `view0.scene`
        settings - faces, face edges, lines, transparency - are not in the graphics data**: OpenGL
        applies them when it draws, so this test cannot see them, and the raytracer images of
        RG2.3.3.4 are where they can be seen.
      - **RG2.3.3.3** **DONE 2026-09-27** — [log](exudynRevisionLog2026b.md#rg2-3-3-3) - **special
        cases as manual examples** - graphics user functions, and whatever else needs a model of its
        own; sensor traces only if they become part of the graphics data. Done for the graphics user
        functions of a ground and a rigid body and a load user function, at the start and after half
        a second. **It found a bug on its first run** (#2726): what a graphics user function draws
        carried the object number where the item ID belongs, and was attributed to a wrong system.
      - **RG2.3.3.4** **DONE 2026-09-27** — [log](exudynRevisionLog2026b.md#rg2-3-3-4) - **the
        raytracer** - `RedrawAndGetImage(True)` at a very low resolution, about 100 x 100, which is
        the part that sees transparency, materials and lighting. Slower, so probably **a small test
        set that always runs and a larger one that does not**; decided after measuring the first
        results, not before. **Measured: 1 to 12 ms per image, identical from run to run** - so one
        set, always run: the representative model with the default, transparent faces, face edges
        and no faces, as PNG references of 0.2 to 1.1 KB. The tolerance across platforms is a guess
        until the first linux run.
      - **RG2.3.3.5** (#2751; #2704 resolved with .1 to .4) **every item, through its MiniExample** - the
        MiniExamples exist since RG13.6. It depended on the group the maintainer
        announced on 2026-09-27: **a MiniExample for every item**, together with the missing
        documentation and examples of all items (a revision group of its own, *"like RG13"* - not
        written yet). The test takes each MiniExample, **injects** a small graphics into its bodies -
        `SetObjectParameter(..., 'VgraphicsData', ...)` at the end and `Assemble()` again - so that
        every item draws triangles, lines and a text with little data, and compares it at the initial
        state and after a few steps (moving items must move, the ground must not). That covers the
        items systematically and without a model per item written for the test, and **the same run can
        write the image of each item** for its documentation page. Nodes without objects do not
        simulate, so nodes, markers, loads and sensors are covered there as well, inside their
        MiniExamples. It grows with that group, one item at a time.

      When GraphicsData gets its sphere and curved triangles (RG6.7, #2709), the test grows with it.

      **The maintainer on RG2.3.3.1 (2026-09-27)**: *"The described approach sounds good - good to
      go."* On the 32-item limit: *"why not put more graphics into the same item? otherwise, the test
      could be split among two files."* Both work, and they differ in what a difference names: several
      graphics in one item give one line for the item, so a new function goes into the item of **its
      family** - one for the transforms, one for the machine parts - where the family still says what
      changed; a case that outgrows 32 families goes into a second case, in the same file or another.

    - **RG2.3.4** **DONE 2026-09-27** (#2706) — [log](exudynRevisionLog2026b.md#rg2-3-4) —
      **`PlotImage` in 3D shows the triangles**: its limits come from everything drawn, not from the
      lines alone.

    - **RG2.3.5** **DONE 2026-09-27** (#2711) — [log](exudynRevisionLog2026b.md#rg2-3-5) —
      **`PlotImage` saves into `exudyn.config.outputDirectory`**, like every other output of a run.

## RG3 — Docs

The documentation is Markdown, built with Sphinx and published for every release since
revision2026 steps R7.1 and R7.2, and what the revision changed is carried into it. What belongs
here is what is still wrong, still missing, or newly wrong because something changed.

Its starting list is [`documentationImpact2026.md`](documentationImpact2026.md), written in
revision2026 step R7.2.2: section C of that file says which feature is documented where, and the
gaps it names are the first candidates. The maintainer's own findings go here as steps.

*No steps yet.*


<a id="rg3-1"></a>
**RG3.1** **DONE 2026-09-22** (#2584) — [log](exudynRevisionLog2026b.md#rg3-1) —
    **The section structure of the user manual was wrong.** The conversion of revision2026
    step R7.1.5 left the `toctree` of `docs/manual/introduction.md` below its last section,
    so three chapters appeared as sub-pages of *"Mapping between local and global coordinate
    indices"*. The order the maintainer asked for is in place, and *C++ Code* is a short
    section of *Advanced topics* that points at the developer documentation.

<a id="rg3-2"></a>
**RG3.2** **DONE 2026-09-22** (#2585) — [log](exudynRevisionLog2026b.md#rg3-2) —
    **The internal how-to notes left the published documentation.** `docs/howTo/` held two
    kinds of note - what a user needs and what only a maintainer needs - and the second kind
    is now excluded in `conf.py`, where `docs/revision/*` already was, and mentioned in one
    line with a link. It stays in git and is simply not a page.

<a id="rg3-3"></a>
**RG3.3** **DONE 2026-09-22** (#2586) — [log](exudynRevisionLog2026b.md#rg3-3) —
    **Is there a PDF, and should there be?** There was none since decision D8 ended it with
    the LaTeX sources. **Answered 2026-09-22 (D17): yes** - `exudev docs --pdf`, release
    only, everything except the source listings of the examples and test models, and the
    issue history is in, because a reader who searches it gets the reason for each change
    with it. 1159 pages. The three defects it uncovered are RG3.6, RG3.7 and RG3.8.


    - **RG3.3.2** **DONE 2026-09-27** (#2707) — [log](exudynRevisionLog2026b.md#rg3-3-2) — **the
      first page of the PDF shows the logo once**, and as wide as the piston engine below it.

<a id="rg3-3-1"></a>
**RG3.3.1** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-3-1) —
    **`exudev docs --pdf` needed Perl and did not say so** (#2658). The LaTeX run was `latexmk`,
    which is a Perl script: it succeeded in Git Bash and failed in PowerShell on the same machine,
    because Git for Windows ships a perl in its `usr/bin` that the one puts on PATH and the other
    does not. The three things `latexmk` automates - run the engine, build the index, run the engine
    again until the cross-references stop moving - are done by `commands.BuildDocumentationPdf` now,
    with the engine and `makeindex` that every TeX installation brings. **Verified with every
    directory holding a `perl.exe` removed from PATH**: 1103 pages, two passes, 54 s.

<a id="rg3-4"></a>
**RG3.4** **DONE 2026-09-22** (#2587) — [log](exudynRevisionLog2026b.md#rg3-4) — *(group RG3; maintainer 2026-09-22)* **The revisions chapter says where the details
    are** (#2587). It is deliberately short, and it should end by pointing at the developer
    documentation: the revision is recorded in full in a plan and a log, and there are **two**
    of them now - revision2026, finished and completed as 1.12, and revision2026b, continuing.

<a id="rg3-5"></a>
**RG3.5** **DONE 2026-09-22** (#2550) — [log](exudynRevisionLog2026b.md#rg3-5) — *(group RG3; maintainer 2026-09-22)* **The citations point nowhere**
    (#2550). The chapters cite in running text — "see Zwölfer and Gerstmayr
    [ZwoelferGerstmayr2021]" — which the LaTeX build resolved and nothing resolved after
    it: the keys were printed and linked to nothing. `docs/bibliographyDoc.bib` survived the
    LaTeX build (revision2026 step R7.1.7 moved it to `docs/`) and holds every key that is used.

    A generated references page, and the citations become links to it without a single source
    text being edited.

<a id="rg3-6"></a>
**RG3.6** **DONE 2026-09-22** (#2592) — [log](exudynRevisionLog2026b.md#rg3-6) —
    *(group RG3; found in RG3.3)* **A generated settings page shows a table with no rows.**
    `structureDocsEmitter.py` writes the table header before the loop that writes the rows, and
    every row of a structure can be skipped - `VSettingsWindowDeprecated` has nothing left that
    is not deprecated. An empty box on the published page, and the reason `sphinx -b latex`
    aborted.

<a id="rg3-7"></a>
**RG3.7** **DONE 2026-09-22** (#2593) — [log](exudynRevisionLog2026b.md#rg3-7) —
    *(group RG3; found in RG3.3)* **Display math opened at the end of a text line swallows the
    text.** 45 blocks in three documents: the sentence is typeset as the formula and the formula
    is shown as raw LaTeX in a code block. Two causes - the list conversion of
    `latexToMarkdown.py` joins an item into one line, and `theoryContact.md` was written that way
    by the conversion of revision2026 step R7.1.5.

<a id="rg3-8"></a>
**RG3.8** *(group RG3; found in RG3.3, extended by the maintainer 2026-09-23)* **The
    unreferenced figures are the trace of figures the conversion lost** (#2594). Fourteen `.pdf`
    and `.eps` files in `docs/figures/` are referenced by nothing; copies are in
    `tmp/unusedFigures/` and nothing has left version control.

    **The maintainer copied the 1.11.0 documentation to `tmp/docs`**, which answers where they
    came from: **every one of the thirteen names appears in the old `.tex` chapters**
    (`theory.tex`, `itemDefinition.tex`, `tutorial.tex`, `solver.tex`, `GUI.tex`). They are not
    leftovers, they are figures the documentation used to show.

    An audit of `tmp/docs/theDoc/*.tex` against the current sources: **60 figures in the old
    chapters, 44 in the new ones.** Most of the difference is not a loss — the HCB and
    free-free mode series are 29 single images that the Markdown replaced with four montages, and
    the singles are still in `docs/figures/modesHinge/`. What IS lost is small and specific:

    | figure | state |
    |---|---|
    | `generalContactANCF2Dcircle` | `.pdf` only, no png twin — the figure AND its caption are gone from the contact theory, where `theory.tex` explained the cable/circle intersection with it |
    | `generalContactSpheres` | `.pdf` only — the only "references" in the current tree are a test model that happens to carry the same name |
    | `ObjectJointALEmoving2D` | `.pdf` only — the item page of `ObjectJointALEMoving2D` has no figure |
    | `intro2.jpg` | exists, referenced by nothing |

    The other eleven `.pdf`/`.eps` are the **vector originals of png twins that the documentation
    does use**. So two questions, and they are different:

    - **the four lost figures** have to come back into the Markdown, with their captions;
    - **the format**: the maintainer asks whether to convert them to SVG. What decides it is the
      PDF of RG3.3: the LaTeX builder **cannot include SVG** — that is exactly the error the
      README badges produced (*"a suitable image for latex builder not found:
      ['image/svg+xml']"*). Sphinx solves it with image candidates: an image written as
      `figures/name.*` picks `.svg` for the html build and `.pdf` for the LaTeX one. So the
      answer is not one format but a pair — **SVG for the browser, PDF for the PDF, one name**
      — and photographs and screenshots stay raster. The vector originals then stop being
      unreferenced and become the source they always were.

<a id="rg3-8-1"></a>
**RG3.8.1** **DONE 2026-09-24** (#2594) — [log](exudynRevisionLog2026b.md#rg3-8-1) —
    **The three contact-friction figures are vector** *(maintainer, 2026-09-24, who drew the
    SVGs)*. `ContactFrictionCircleCable2D`, `...stickingPos` and `...normals` are written as
    `docs/figures/<name>.*` in `definitions/itemDefsObjects.py`, so Sphinx picks the **SVG for
    the browser and the PDF for the LaTeX build** from one name - the pair the step asked for.
    Which further figures are worth an SVG is measured in the log.

<a id="rg3-8-2"></a>
**RG3.8.2** **DONE 2026-09-25** (#2594, #2650) — [log](exudynRevisionLog2026b.md#rg3-8-2) —
    **Three more figures are vector, two became text, and what they replaced is out of the tree**
    *(maintainer, 2026-09-25, who drew the SVGs)*. `pendulum`, `pendulumConstraint` and
    `RotationsSequences` are `.*` candidates; the two `kinematicTree` images were **screenshots of
    a LaTeX algorithm** and are the algorithms themselves now, written out from the old `.tex`;
    `intro2.jpg` is the title picture of the PDF again. Eight obsolete files left the tree, and
    the four sub-tutorials are no longer nested under the first one (#2650).

<a id="rg3-8-3"></a>
**RG3.8.3** **DONE 2026-09-25** (#2651) — [log](exudynRevisionLog2026b.md#rg3-8-3) —
    **Three item pictures were in the repository and on no page** *(maintainer, 2026-09-25)*.
    `RevoluteJointZ2`, `SphericalJoint` and `UniversalJoint` show exactly one item each and are
    now in the descriptions of `ObjectJointRevoluteZ`, `ObjectJointSpherical` and
    `ObjectJointGeneric`. `TutorialRigidBody1.png` belonged to a tutorial that no longer exists
    and is gone.

<a id="rg3-8-4"></a>
**RG3.8.4** **DONE 2026-09-26** (#2594) — [log](exudynRevisionLog2026b.md#rg3-8-4) —
    **The four lost figures are three, and they are back.** `generalContactSpheres` and
    `generalContactANCF2Dcircle` are in `docs/manual/theoryContact.md` with the captions they had in
    `theory.tex`, and `ObjectJointALEmoving2D` is in the description of its item, where
    `itemDefinition.tex` had it inside an `\ignoreRST{...}` that the conversion honoured. Each is a
    `.*` candidate, so the browser gets the SVG the maintainer drew and the PDF the vector original -
    checked in `_buildpdf/latex/exudynDocumentation.tex`, which includes all three as `.pdf`.

<a id="rg3-8-5"></a>
**RG3.8.5** *(from RG3.8; measured 2026-09-26)* **The seventeen vector originals whose png the
    documentation uses.** `CommonTangents3D`, `ConvexRolling`, `ObjectFFRFsketch`,
    `SphereSphereContact` and thirteen more exist as `.png` **and** as `.pdf` or `.eps`, and every
    reference names the `.png`. Writing them as `.*` would give the PDF the vector original at no
    cost, which is what RG3.8 wanted.

    **Why it is not done in passing**: the risk is that a `.pdf` twin is *not* the same picture as
    its `.png` - they were exported at different times over ten years - and the failure is silent,
    because the HTML shows one and the PDF the other and nobody compares two builds. So the step is
    **one comparison per pair first**, and only the pairs that match are switched. Until then the
    `.png` is what both builds show, which is at least the same thing twice.


<a id="rg3-9"></a>
**RG3.9** **DONE 2026-09-23** (#2598) — [log](exudynRevisionLog2026b.md#rg3-9) —
    **Three corrections to the landing pages and the developer chapters.** That Exudyn is
    developed heavily with Claude Code since 1.11.0 is said at the top of `README.rst` and
    of `pdfIndex.md`; the hand-counted numbers of examples, test models and pages are gone
    (a number that is not generated is wrong the next day); and the seven developer
    documents are nested under *Exudyn developer documentation* instead of beside it.

<a id="rg3-10"></a>
**RG3.10** **DONE 2026-09-24** (#2599) — [log](exudynRevisionLog2026b.md#rg3-10) —
    **`CHANGELOG.md` and the issue tracker page hold the same list twice.** They do not any more:
    the changelog is the **current release** with a table of every release above it (2440 lines
    to **124**), and the tracker page is *"Resolved issues and resolved bugs **before version
    1.12**"* (9153 to **8923**). Every issue is published, in exactly one place.

<a id="rg3-10-1"></a>
**RG3.10.1** **DONE 2026-09-24** (#2637) — [log](exudynRevisionLog2026b.md#rg3-10-1) —
    **The changelog and the tracker page printed the same issue in two formats**
    *(maintainer, 2026-09-24)*. RG3.10 split the two lists but left the two renderings: the
    changelog had a type badge and no dates, the tracker page had dates and no type. One
    `IssueEntry()` prints both - the type, then the priority and the effort as badges, then
    who raised and resolved it, then the sub-list of the tracker. The changelog sentence that
    called the other page "the full issue tracker" was wrong and is gone.

<a id="rg3-11"></a>
**RG3.11** **DONE 2026-09-23** (#2611) — [log](exudynRevisionLog2026b.md#rg3-11) —
    **"The C++ core" pointed at the repository instead of at the documentation.** The
    section in *Advanced topics* stays - it is short and it answers a question users ask -
    but it sent the reader to GitHub URLs of pages that RG3.2 publishes. It links to the
    developer documentation now, and says what a link cannot: that for a deeper
    understanding of the core there is no way around the GitHub project itself.

<a id="rg3-12"></a>
**RG3.12** **DONE 2026-09-24** (#2646) — [log](exudynRevisionLog2026b.md#rg3-12-1) — *(group RG3; maintainer 2026-09-24)* **Building from source and the development
    workflow are told three times and never from the start** (#2646). The maintainer read the
    two chapters end to end and the faults are structural, not wording. Measured:

    - **the order is wrong.** `gettingStarted.md` ends with the toctree that nests *Installation
      instructions*, so a reader meets *Run a simple example in Python* **before** being told how
      to install anything;
    - **three places describe the build** and disagree. `gettingStartedInstall.md` has *Build and
      install under Windows / Mac OS X / Ubuntu* (~170 lines, still *"go to `main` of your cloned
      github folder"* although the `main/` level went in revision2026 step R3.1, still Ubuntu
      18.04 with Python 3.6 and a `USE_GLFW_GRAPHICS` define in `BasicDefinitions.h` that no
      longer exists); `docs/howTo/buildFromSource.md` is the current reference at 151 lines and
      the maintainer *"finally found"* it; `docs/dev/README.md` has a third, short version;
    - **nothing says how to get the code.** There is no `git clone` in the documentation, no
      choice between ssh and https, and no page on branches, commit messages, pull, push and
      merge - which an external contributor has to be told and which is what keeps the internal
      workflow consistent;
    - **`WORKFLOW.md` is 845 lines** whose sections run 0, 1, 2, **0a**, 2a, 2b, 3, 4, 5, 6: the
      one-time setup of a clone stands after versioning. Section 0 starts from an environment
      that already exists, without saying where it comes from;
    - **Visual Studio 2022 is called the primary development environment**, which is half true.
      It is the mixed Python/native debugger. The everyday work - Python, the definition files,
      the documentation, Claude Code - happens in **VS Code**, which is where the co-developers
      will be.

    **The recommended shape**, which is what the sub-steps build. The rule behind it: *a fact is
    written once and linked to*, and the **user manual tells a user how to install**, while
    **building from source belongs to the developer documentation**.

    | page | what it holds |
    |---|---|
    | `docs/manual/gettingStarted.md` | what Exudyn is, the goals, the thanks - and the toctree **before** the example |
    | `docs/manual/gettingStartedInstall.md` | requirements, pip, a specific wheel, troubleshooting, uninstall. **One short section** *Build from source* saying when a user needs it and linking to the developer page; no recipe |
    | **new** `docs/dev/GETTING_STARTED.md` | clone over https or ssh, create the environment, run `tools/setupLocalWorkspace.py`, build once, run the tests once - the step-by-step an engineer needs, absorbing `WORKFLOW.md` §0a |
    | **new** `docs/dev/BUILD.md` | the **one** build reference: a platform-independent part first, then Windows, Linux, macOS, then "the build works and the import does not", debugging and cleaning up. Absorbs `docs/howTo/buildFromSource.md` **and** the three sections of the user manual |
    | **new** `docs/dev/GIT.md` | branch, commit message, pull, push, merge, and what a contribution must provide - the command line form, because that is what a VS Code user types |
    | `docs/dev/WORKFLOW.md` | what is left once the setup and the git part have moved out: the issue tracker, versioning, CI, the gates, committing - renumbered in the order it is done |
    | `docs/howTo/buildFromSource.md` | **deleted**; `condaEnvironments.md` stays and is linked from the new setup page |

    Sub-steps:

    - **RG3.12.1** **DONE 2026-09-24** — [log](exudynRevisionLog2026b.md#rg3-12-1) — the order in the user manual, and the LaTeX relicts;
    - **RG3.12.2** **DONE 2026-09-24** — [log](exudynRevisionLog2026b.md#rg3-12-2) — `docs/dev/BUILD.md`: one build reference, the how-to note deleted, the manual reduced to a pointer;
    - **RG3.12.3** **DONE 2026-09-24** — [log](exudynRevisionLog2026b.md#rg3-12-3) — `docs/dev/GETTING_STARTED.md`: clone, environment, workspace, first build, first test run;
    - **RG3.12.4** **DONE 2026-09-24** — [log](exudynRevisionLog2026b.md#rg3-12-4) — `docs/dev/GIT.md`: the git workflow, for co-developers and for contributors;
    - **RG3.12.5** **DONE 2026-09-24** — [log](exudynRevisionLog2026b.md#rg3-12-5) — `WORKFLOW.md` restructured into the order the work is done, and shortened by what moved out;
    - **RG3.12.6** **DONE 2026-09-24** — [log](exudynRevisionLog2026b.md#rg3-12-6) — the editors: VS Code for the everyday work, Visual Studio 2022 for mixed
      Python/native debugging, in the invariants of the info document, in `CLAUDE.md`, in the
      developer README and in the user manual.


<a id="rg3-13"></a>
**RG3.13** **DONE 2026-09-24** (#2648) — [log](exudynRevisionLog2026b.md#rg3-13) — *(group RG3; maintainer 2026-09-24)* **The tree told the reader about the revision
    instead of about itself** (#2646). The rule is now written down - `CLAUDE.md` 6a and
    `CODING_STYLE.md` §6, *documentation says what IS, not what it was* - and this step applies it
    to what is already there. The maintainer's example, `python/TestModels/GraphicsDataTest.py`:

    > *"GraphicsDataTest, one of the ten small tests that lived in `python/testing/modelUnitTests.py`
    > from 2019 until revision2026b step RG10.6.5 made each of them an ordinary test model."*

    A reader of that page - the test models **are** documentation pages - wants to know what the
    model computes. Measured 2026-09-24: **887 mentions of `revision2026` in 257 files** outside
    `docs/revision/`, which is where they belong. Not all of them are wrong, so the step is a
    sweep with a rule, not a replace:

    | where | mentions | what to do |
    |---|---|---|
    | `python/TestModels` (24 files), `python/PerformanceModels` (3) | 36 | **published pages**: rewrite the `Details:` header to say what the model does; the issue number stays, the step number goes |
    | `definitions/` (27 files) | 39 | **published**: the item and settings descriptions are the reference manual |
    | `docs/manual` (2 files left), `docs/dev` (6) | ~60 | `docs/dev` may keep a step reference where it is about the plan itself; a manual page may not |
    | `src/`, `tools/`, `setup.py`, `conf.py` | ~750 | **not published**: the rule there is the older one - a comment cites the issue, not the step - so this is a cheaper pass, and a comment that explains a measurement may keep its step |

    What must not be lost: the **issue number**, which is a link a reader can follow, and any
    sentence that carries a *measurement* or a *decision*. What goes: the name a thing had before,
    when it changed, and which plan step changed it.


<a id="rg3-13-1"></a>
**RG3.13.1** *(group RG3; from RG3.13, 2026-09-24)* **DONE 2026-09-27** —
    [log](exudynRevisionLog2026b.md#rg3-13-1) — **235 references to the plan are left in
    comments, each inside a sentence** (#2649). By the time it was done they were 330 in 177 files -
    the work since RG3.13 had added its own - and they are 0. 652 of the 887 were parentheticals or appended
    clauses and went by rule, keeping the issue number where there was one. The rest read like
    *"step R4.3 is moving outputs from the old generators to separate emitters"* - a sentence has
    to be written for each, which a pattern cannot do. **None is in a published page**: they are
    comments in `src/`, `tools/` and `python/`, so this is tidiness rather than a defect, and it
    is work for a session with nothing better to do.

    - **RG3.13.2** *(sub-step; found in RG3.13.1)* **DONE 2026-09-27** —
      [log](exudynRevisionLog2026b.md#rg3-13-2) — **88 bare step numbers** (#2703): the same rule
      broken without the plan's name - `step R6.3.8`, `the rule RG6.2.11 wrote down`, `dies with the
      LaTeX branch in R7.1.7` - in 39 files. RG3.13.1 counted `revision2026` and did not see them. The
      same three passes and the same proof apply.

<a id="rg3-14"></a>
**RG3.14** **DONE 2026-09-26** (#2655) **The item and settings descriptions are written
    in LaTeX** (#2655). `definitions/` is the source of the reference manual, and a developer who
    writes an item description there writes LaTeX: the published pages are Markdown and are
    correct, so this is not a defect in the output - it is that the input is a language nobody
    writing an item description should have to know, and that nothing checks it.

    Measured 2026-09-25 over the string constants of `definitions/*.py`:

    | construct | count | what it is |
    |---|---|---|
    | `$...$`, `\be..\ee`, `\bea..\eea` | 2897 / 372 / 45 | **the math - already native Markdown** (`dollarmath`), 166 macros declared to MathJax and to LaTeX by `conf.py` |
    | `\rowTable` / `\startTable` | 709 / 86 | the tables, in **7 header kinds**, two of which differ only in a space |
    | `\hac` / `\ac` | 160 / 17 | an abbreviation, linked to `docs/generated/abbreviations.md` |
    | `\mysubsubsubsection(label)` | 127 / 6 | the only heading level an item uses; `\mysubsection(label)` appears 4 times, in the four **structure** files |
    | `\refSection` / `\eq` / `\eqs` / `\eqref` / `\fig` / `\ref` / `\label` | 69 / 37 / 3 / 2 / 16 / 17 / 55 | the references |
    | `\userFunction` / `\returnValue` / `\userFunctionExample` | 35 / 35 / 17 | a user function: signature, argument table, example |
    | `\onlyRST` / `\ignoreRST` | 11 / 13 | the two switches |
    | `\texttt` | 725 | inline code |
    | everything else | 87 distinct macros, 2684 occurrences in all; **32 of them occur at most twice** | |

    The decision (maintainer, 2026-09-25): **the description stays one text field, and it becomes
    MyST Markdown.** The math stays LaTeX, because that is what Markdown's math *is*. What goes is
    the structural LaTeX: the headings, the references, the tables, the user function blocks and
    the two switches. What replaces it is a **small documented set of `NAME:argument` macros** that
    the converter expands, and **data in the definition dict** wherever the text was a table. When
    the step is done, **a backslash outside math is an error** - see RG3.14.7 - so the writer is
    not tempted to reach for the rest of LaTeX.

    **Which strings are converted.** Every description text of `definitions/` reaches the same
    function, `latexToMarkdown.ConvertText`, and all of them are in scope; measured by the keyword
    each literal is passed to:

    | file group | keyword | strings | reached through |
    |---|---|---|---|
    | `itemDefs*.py` | `equations` | 64 | `itemDocsEmitter.WriteFile` → `ConvertText`, then `NormalizeHeadings` in `itemDocsEmitter.WriteMarkdownPages` |
    | `itemDefs*.py` | `classDescription` | 31 | the same |
    | `itemDefs*.py`, `itemFunctions.py` | `description` of `ItemParameter` / `ItemFunction` | 52 | `autoGenerateHelper.PyLatexRST.ItemInterfaceWriteRow` → `ConvertText`, one table cell |
    | `structureDefs*.py` | `description`, `classDescription`, `sectionText` | 43 | `structureDocsEmitter.StructureDocs` and `PyLatexRST.SystemStructuresWriteDefRow` |
    | `pybind*.py` | `description` | 6 with macros, and **163 non-raw strings with a backslash** | `pybindEmitter` replays the calls onto `PyLatexRST`, whose `AddDocu`, `AddDocuList`, `DefPyFunctionAccess` and `Table3WriteRow` call `ConvertText` |
    | `outputVariableDescriptions.py`, `outputVariableTypes.py`, `enumTypes.py`, `definitionTypes.py` | - | **none** | they carry math only, or no text at all - the output variable table is **already** generated from data, which is the shape RG3.14.4 gives the others |

    `miniExample` is Python, not prose, and is not converted. The same `ConvertText` also serves
    the docstrings of `python/exudyn/` through `utilityDocsEmitter`; those are **not** in this step.

    **Every sub-step ends by writing its rules into one place**, the new *"Writing a description"*
    section of [`definitions/README.md`](../../definitions/README.md) (RG3.14.8). No rule is
    copied anywhere else: the header of each definition file, `CLAUDE.md` and this step all
    *point* at it.


    - **RG3.14.1** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-1) - **the abbreviations**: `ABRV:ODE2` in place of the seven
      LaTeX spellings (`\hac`, `\hacs`, `\acf`, `\acl`, `\acs`, `\acp`, `\ac`), which
      `latexToMarkdown.ConvertInline` rendered identically - so they were one macro under seven
      names. 178 occurrences, and none needs an argument the macro cannot carry. The list is
      already a Python dict, `abbreviations` in `tools/generators/examplesDocsEmitter.py`, written
      out by its `WriteAbbreviations`, so the new `tools/checkDefinitions.py` **checks the key** and
      names the file and the line of a wrong one.
    - **RG3.14.2** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-2) - **the heading levels, defined and checked.** An item page is
      `# <file>` / `## <item>` / `### DESCRIPTION of <item>`, and a `\mysubsubsubsection` becomes
      `####`; the depth is implicit in the macro name (`latexToMarkdown.ConvertSections`) and
      `latexToMarkdown.NormalizeHeadings` silently repairs whatever does not fit. In Markdown the
      writer writes `#### Equations`, and the emitter that places the page -
      `itemDocsEmitter.WriteMarkdownPages` and `structureDocsEmitter.StructureDocs` - checks the
      level against the page it is placed in. The check has something to find: the 133 headings
      carry **49 distinct titles**, and two pairs differ only in case or spacing -
      *Connector forces* (11) against *Connector Forces* (3), *Post Newton Step* (3) against
      *PostNewtonStep* (1). A documented set of the recurring ones - *Definition of quantities*
      (37), *Equations of motion* (12), *Connector forces* (14), *Connector constraint equations*
      (9), *Geometric relations* (8), *Details* (4) - with free titles allowed below them.
    - **RG3.14.3** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-3) - **the references, in native Markdown.** `[](#sec:itemGround)` in place of
      `\refSection{sec:itemGround}`, `[](#eq:ObjectGround:position)` in place of `\eq{...}`,
      `[](#fig:ObjectSphereSphereContact)` in place of `\fig{...}`, and `$$...$$ (eq:name)` in
      place of `\be ... \label{eq:name} ... \ee`. All of these live in
      `latexToMarkdown.ConvertInline` (with `RefLabel` and `autoGenerateHelper.MarkdownLabel` for
      the label spelling) and `latexToMarkdown.ConvertDisplayMath`. **Probed 2026-09-25** against
      Sphinx 9.1.0 / myst-parser 5.1.0 with this project's `myst_enable_extensions`: a target
      written `(sec:name)=` **on a heading**, an equation label, and a `{figure}` with `:name:` all
      three resolve from another page, with and without link text, and the text defaults to the
      heading or the caption. The 55 `\label`s are **45 equations, 10 figures and nothing else**,
      and all 9 section labels are written as `\mysub...sectionlabel`, i.e. on their heading - so
      every reference in `definitions/` can become a native link. A citation needs **no macro at all**: `conf.py`
      appends a Markdown link definition for every key of the bibliography, so
      `[ZwoelferGerstmayr2021]` written directly is already a link, and all 31 `\cite` calls are
      single-key. So the answer to the maintainer's question is the strongest one available:
      **nothing stays a backslash command outside math.**

      - **RG3.14.3.1** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-3-1) - **the display math.** The 372 `\be .. \ee` and
        45 `\bea .. \eea` blocks become `$$ ... $$` and `$$ \begin{aligned} ... \end{aligned} $$`,
        which is what `latexToMarkdown.ConvertDisplayMath` already writes, and the **45 equation
        labels inside them** become the MyST form `$$ ... $$ (eq-name)`. The mathematics itself does
        not change: `\be` and `\ee` are Exudyn's own delimiters, not LaTeX's, and `$$` is what
        Markdown's display math is. With them go `\eqComma` and `\eqDot`, which the converter
        already spells out, and `\nonumber`, which numbers nothing in an `aligned` block.
      - **RG3.14.3.2** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-3-2) -
        **a reference to an equation is the role, not a link.** RG3.14.3 made all 144 references
        native, and 42 of them point at an equation: those resolve in the HTML and **left the PDF as
        undefined references**, because the LaTeX writer gives a link to an equation the anchor
        `<document>:equation-<label>` while it labels the equation `equation:<document>:<label>`.
        Only the PDF said so, which is why RG3.3.1 had to come first. `checkDefinitions` rejects a
        link whose target is an equation label.
    - **RG3.14.4** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-4) - **a table
      is a Markdown table.** The 48 tables that are not a user function's argument list are pipe
      tables, written where they stand in the text; the 34 argument tables belong to RG3.14.5 and
      four more `\startTable`s are commented out and already dead. The pages are byte-identical.

      **The dict was measured and not used.** `quantities=[Quantity(name, symbol, description)]` was
      the plan for the 39 *"Definition of quantities"* tables, but **32 of the 39 open the
      `equations` text and 7 do not** - three items carry a second one, and one sits under the
      sentence that introduces it. A field in the dict says what a table holds and not where it goes,
      so those seven would move on the page or the text would need a placement marker: more
      machinery than the table. A pipe table says both, in one place, in native Markdown. Making the
      **symbol column** data, so that a checker can say whether every symbol is a declared math
      macro, is worth a step of its own and is not this one.
    - **RG3.14.5** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-5) - **a user function block loses its LaTeX** - and the typed Python signature was measured and not used, because 35 of the 228 argument rows say the *size* of an argument as a formula, and a formula does not render inside a code block. The arguments stay a table, now a pipe table; the signature is written as the Python it is, so the `\_` escapes are gone either way. Originally planned as: In place of
      `\userFunction{forceUserFunction(mbs, t, itemNumber, q, q\_t)}` followed by a table and a
      `\returnValue` row:

      ```python
      userFunctions=[UserFunction(r'''
          def forceUserFunction(mbs: MainSystem, t: Real, itemNumber: Index,
                                q: 'Vector $\in \Rcal^{n_{ODE2}}$',
                                q_t: 'Vector $\in \Rcal^{n_{ODE2}}$') -> 'Vector6D':
              """compute the force ... """
          ''')]
      ```

      The signature carries the names, the order and the types; the docstring carries the prose;
      the per-argument description comes from the docstring's parameter lines. The measurement says
      this fits: 34 of the 35 signatures are a single line, and **15 of them contain `\_` escapes
      that exist only because the text is LaTeX** - `q\_t` is `q_t` in Python. The 231 argument
      rows use **26 distinct types**, 14 of them plain names (`Real` 98, `MainSystem` 35,
      `Index` 29, `Vector3D` 16, `BodyGraphicsData` 5, ...). The 17 `\userFunctionExample` blocks
      become ordinary fenced code (`latexToMarkdown.ConvertListings` loses them).

      **Duplication between items is intended and stays** (maintainer, 2026-09-25): two items can
      share a signature and mean different things by it, and the argument and return descriptions
      are what say so. What the step does remove is duplication **inside** one item: a user
      function is documented once per item.
    - **RG3.14.6** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-6) - **the two RST switches go.** `latexToMarkdown.ResolveRSTSwitches` keeps what
      `\onlyRST` holds and **drops what `\ignoreRST` holds**, and `grep includegraphics
      docs/generated/` finds nothing: all 13 `\ignoreRST` blocks are **dead text that reaches no
      builder**, because the PDF is built by Sphinx from the same Markdown since RG3.3 (D17). Ten
      of the 11 `\onlyRST` blocks hold an RST `.. figure::` that a MyST `{image}` says in three
      lines. So: **delete the 13, unwrap the 11, and delete the two macros with
      `ResolveRSTSwitches`, `DropLatexFigures`, `ConvertRSTFigures`, `ConvertRSTImages` and
      `LatexRSTFigure`.** The one pair worth a decision is `ObjectKinematicTree`, where the
      `\ignoreRST` twin holds the two LaTeX `algorithm` environments; RG3.8.2 already put the
      algorithms into the text as numbered lists, so **there is no case left for keeping even
      one**, and the step may delete all 24.
    - **RG3.14.7** **DONE 2026-09-25** - **the tail and the gate.** 1005 backslash commands in 23 names are left in a
      description outside mathematics, a comment and a code block: **807 in `itemDefs*`, 190 in
      `pybind*`, 8 in `structureDefs*`**. A first attempt converted them in one pass and moved 738
      lines of the pages, so the step is split - the maintainer's advice, 2026-09-25: *"why not try
      step-by-step or by defs-classes, with some global replacement of latex elements where it would
      always work first"*. The order is by how much a macro interacts with the line structure, which
      is where every failure so far has come from.

      - **RG3.14.7.1** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-7-1) - the
        macros that are a single element and touch no line: `\codeName` (11), `\vspace` (20),
        `\noindent` (17), `\paragraph` (15), `\footnote` (13), and one each of `\text`,
        `\mysmall`, `\textdegree`, `\phantom`, `\exuUrl`, plus the 55 escaped spaces that
        `\codeName\ ` needed before a comma.
      - **RG3.14.7.2** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-7-2) - `\texttt{x}` to `` `x` ``: 706 of the 1005, one to one, no line structure.
      - **RG3.14.7.3** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-7-3) - the bold forms: `\bf` (45) in its `{\bf x}` shape and `\mybold` (29).
      - **RG3.14.7.4** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-7-4) - the **lists**: `\bi`/`\ei` (19/19), `\item` (84), `\bn`/`\en` (4/4).
        This is the one that rewrites lines, and `ConvertLists` indents a block inside an item by two
        while `DedentOutsideCode` keeps that indentation inside a fence - so it is done alone, with
        the byte-identical comparison read per file.
      - **RG3.14.7.5** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-7-5) - what is not a `\name`: the **41 LaTeX line breaks** `\\` (21 in
        `itemDefs`, 14 in `pybind`, 6 in `structureDefs`), the 6 `\tabnewline` of the pybind tables,
        the **67 `%%RSTCOMPATIBLE` markers** - 65 of which have nothing after them at all - and the
        six `\n` in pybind descriptions that reach the page as two characters, because the old
        parser turned a literal `\n` into a newline for the item files and never for these. A
        Markdown hard break is two trailing spaces, which `StripComments` removes, and 191 lines
        already end in two or more spaces by accident - so this sub-step decides what a line break in
        a description *is*, and that decision is why it comes last.
      - **RG3.14.7.6** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-7-6) - **the gate**: a `\name` in a description, outside mathematics, a comment
        and a code block, is an error with the file, the line and the name - in
        `tools/checkDefinitions.py`, beside the other five rules.
        `latexToMarkdown.ReportUnknown` has been dead code all along and goes with it.
    - **RG3.14.8** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-8) - **the one place the rules are written.** A new section of
      `definitions/README.md`, which is published (`docs/dev/README.md` lists it) and today says
      how a *member* is written but nothing about the description text. Its skeleton is written
      **first** and each sub-step adds its own paragraph, so that no sub-step is done before its
      rule is readable. Then, and this is the point of the step: the **header comment of every
      `definitions/*.py` file** says *read `definitions/README.md` before writing a description*,
      and **`CLAUDE.md` says the same** in its hard rules - a session that writes plain Markdown
      where a `ABRV:` macro is meant, or LaTeX where the check will reject it, is a session that
      did not read one file.
    - **RG3.14.9** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-9) - **a description that carries math is an `r'...'` literal, and that is
      checked.** Measured 2026-09-25 over `definitions/*.py`: **1128 string literals carry a `$` or
      a backslash macro; 940 are already `r'...'`, 188 are not, and 163 of those 188 already hold a
      doubled backslash** - `'\\item'`, `'  \\item Create \\texttt{Vector3DList}'`, `' \\\\ \\\\
      Usage: \\bi'`, almost all in the `pybind*.py` files. So the escaping is already being paid
      by hand, and one `\n` or `\t` written by accident is a bug nobody sees. The check is a file
      check and needs no new machinery: `ast` gives each literal its line and column, and the
      source at that column carries the prefix - the measurement script for this step is the check.
      The 163 become raw literals and readable; the other 25 hold a `$` and no backslash and are
      converted for the rule's sake.

    - **RG3.14.10** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-10) - **the
      citations, and the field that was called `latexText`.** The 31 `\cite{key}` calls are
      `[key]`, which is what they already rendered as, so the pages do not move; a citation is
      given the marker of RG3.14.11 and checked against the bibliography;
      without a marker nothing could be told apart, because 1228 bracketed tokens in `definitions/`
      are not citations and twelve of the 100 keys are not shaped like one. And `StructureDefinition.latexText`, which is neither LaTeX nor written in
      it, is `sectionText`: the heading and the paragraph that open a group of structures.

    - **RG3.14.11** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-14-11) - **the
      citation marker, and the last of `theDoc.pdf`.** A citation is `[CITE:Key]`, which the
      converter turns into the `[Key]` that `conf.py` resolves: the pages do not move, and the
      marker is what makes three checks possible - a key the bibliography does not have, a key
      written without the marker, and a bracketed word that is nearly a key. A `[CITE:` left in a
      generated page is a conversion that did not happen. `\refSection` in a docstring names its
      section instead of the literal `theDoc.pdf`, and the eight `\refSection{...}` that stood in
      the issue archive are plain section names (**#2656** is the five hand-written docstrings that
      still say `theDoc.pdf` in their own text).

    - **RG3.14.12** **DONE 2026-09-26** — [log](exudynRevisionLog2026b.md#rg3-14-12) - the five
      `theDoc.pdf` references are gone, and the name is explained once in the revisions chapter.
      Smaller than the step assumed: both section targets already resolved, so only the words in
      front of them were wrong. The same docstrings said `staticSolver` where `SolveDynamic`
      stores `dynamicSolver`.
    - **RG3.14.13** **DONE 2026-09-26** — [log](exudynRevisionLog2026b.md#rg3-14-13) - the LaTeX
      machinery of `autoGenerateHelper.py`, audited by reachability and not by reading: **eight
      names reachable from nothing**, 185 lines, and the regeneration a no-op afterwards. What is
      alive is alive for a reason and is named in the log. The audit found what the step could not
      know: **745 LaTeX escapes in the generated Markdown**, which is RG3.14.14.
    - **RG3.14.14** **DONE 2026-09-26** — [log](exudynRevisionLog2026b.md#rg3-14-14) - the LaTeX
      escapes that were left in the generated Markdown. **115 of them, not 745**: the other 638 are
      the tracker log, where escaping issue text is the rule of #2545 and is correct. Five producers
      removed, 38 subscripts that had never rendered, and two utility functions that had no
      *Relevant Examples* list because the search looked for a name with a backslash in it.

    **What would not work today**, and is either solved inside the step or stated as its boundary:

    - **A target that sits on neither a heading, an equation nor a named figure cannot be reached by
      `[](#name)`.** Probed 2026-09-25: a target on a paragraph, on a list item, as an inline
      `{#anchor}` and as a raw `<a id>` all four warn `myst.xref_missing`, and only
      `` {ref}`text <name>` `` resolves them - with the text, because `` {ref}`name` `` alone warns
      *"A title or caption not found"*. There is exactly one such place, and it is the reason
      RG3.14.1 keeps a macro: the abbreviation list, section `sec:listofabbreviations` in
      `docs/generated/abbreviations.md`, where each entry is a bare target above a paragraph -
      `(ODE2)=` above `**ODE2**: second order ordinary differential equations`. `ABRV:ODE2`
      therefore expands to `{ref}`ODE2 <ODE2>``, which is what `\hac` does today, and **the writer
      never types the role**. Giving every abbreviation its own heading would make the native form
      work and is not worth 90 headings in a list.
    - **An implicit heading anchor is same-page only.** `myst_heading_anchors = 3` generates a slug
      per heading, and `[](#a-sub-heading)` from another file warns. Every target that is
      referenced across pages has to be written as `(name)=`, which is what `\label` does today,
      so nothing is lost - but the check of RG3.14.7 cannot be *"no labels"*.
    - **A type that is a shape cannot be a Python annotation.** 44 of the 231 argument rows have a
      type cell like `Vector $\in \Rcal^{n_{ODE2}}$`. A string annotation carries it -
      `q: 'Vector $\in \Rcal^{n_{ODE2}}$'` - and the generator reads the annotation as a string
      rather than evaluating it. The block is therefore parsed with `ast`, not executed.
    - **The user function signature is declared in three places and this step unifies none of
      them**: the documentation block, the member's type (`PyFunctionGraphicsData`, and the
      `std::function<...>` it maps to in `definitions/definitionTypes.py`) and the stub files.
      Making the documented signature the source of all three is a step of its own and belongs to
      RG12, not here.
    - **The math macros stay LaTeX**, and that is the decision, not a limitation: MyST's math *is*
      LaTeX, `conf.py` declares the 166 macros to MathJax and to the LaTeX preamble, and
      `tools/checkMathMacros.py` already checks that every one used is known. Rewriting 2897 inline
      formulas would buy nothing.
    - **The hand-written chapters of `docs/manual/` are not in scope**, nor are the docstrings of
      `python/exudyn/`. `ConvertText` keeps its LaTeX branch for them; what this step adds is a
      check that `definitions/` no longer uses it.

    The gate is the ordinary one, with one addition: the **generated pages must not change** except
    where a table gains a column or a heading is corrected, so the step is done in passes with
    `git diff docs/generated/` read after each.


<a id="rg3-15"></a>
**RG3.15** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-15) — **The chapters of the user manual** (#2657, #2661).
    The table of contents is decided and is written out at the end of this document, in *The chapters
    of the user manual, as decided for #2657 and #2662*. In short: **Renderer, graphics and
    visualization** becomes the one visualization chapter, in four sections - *The renderer window*,
    *The model view*, *Images, animations and the solution viewer*, *How to add graphics* - built
    from the eleven visualization sections that sit in *Exudyn basics* today, the five that are
    already in the chapter and the one in *Advanced topics*. **Performance, errors and solver
    failures** becomes a chapter of its own with the three sections of *Exudyn basics* that belong
    together and are not basics. *Advanced topics* takes what is internals or reference - the
    graphics pipeline, raytracing, the `GraphicsData` reference - and, with #2661, the command line
    and the results monitor. *Exudyn basics* keeps one visualization section, *Seeing the model*,
    which points at the chapter.

    Every section that moves keeps its target, so that the references to it keep working; that is
    what makes this a move and not a rewrite, and it is the thing the gate has to prove.

<a id="rg3-16"></a>
**RG3.16** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-16) — **Every heading is sentence case** (#2662).
    *"This is a heading"*, everywhere. Measured 2026-09-25: 116 of the 189 headings of
    `docs/manual/` and `index.md` already are, and about fifteen are Title Case and should not be -
    *Installation and Getting Started*, *Exudyn Basics*, *Generating Animations*, *Generalized
    Forces*, *Lagrange's Equations of Motion* and the like. What stays capitalised is a proper noun
    (*Runge-Kutta*, *Newmark*, *Hurty-Craig-Bampton*, *Tait-Bryan*) and a name the code spells with
    a capital (`GraphicsData: Line`, *Items: Nodes, Objects, ...*), so the checker that finds them
    needs a list of those names and reports the rest.

<a id="rg3-17"></a>
**RG3.17** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg3-17) — **A comment in a description is an HTML comment**
    (#2663). 513 lines of `definitions/` are nothing but a `%` comment and 7 more carry one after
    text; they become `<!-- ... -->`, which the converter removes so that it does not reach the page.
    **`%` survives inside mathematics and nowhere else**: MathJax and LaTeX both honour it there, so
    it needs no handling at all - two of the 513 are a commented-out continuation line inside a
    multi-line formula, which is exactly the case that must keep working. The reason to change the
    rest is in `StripComments`: it runs **before** the mathematics is protected, so a `%` anywhere
    truncates the rest of its line whatever that line is.


<a id="rg3-18"></a>
**RG3.18** **DONE 2026-09-26** (#2660) — [log](exudynRevisionLog2026b.md#rg3-18) —
    **The pages of the Python-C++ interface repeat their own title, and it costs the MainSystem
    extensions their place in the table of contents** (#2660).

    Six pages open with their own name twice - *12.3 SystemContainer* / *12.3.1 SystemContainer*, and
    the same for `Renderer`, `MainSystem`, `SystemData`, `Symbolic` and `GeneralContact`. The title
    comes from `markdownPageTitles` in `pybindEmitter.WriteMarkdownPages` and the section from the
    class's own `DefPyStartClass`, and for these six the two are the same word.

    **The second half of the issue is the same defect seen from the table of contents** (maintainer,
    2026-09-25): there is no entry for *MainSystem extensions (create)* nor for *MainSystem
    extensions (general)*. Measured: both are **level-3** headings of
    `docs/generated/cInterface/MainSystem.md`, and they are at level 3 rather than 2 *because* the
    redundant section takes a level - the page title is 1, the repeated name is 2, the extension
    sections are 3. Nested through the cInterface index into the main `toctree`, both of which say
    `:maxdepth: 3`, level 3 on that page falls past the limit. **Removing the repetition moves them
    to level 2 and they appear**, which is why the two halves are one step.

    What has to be arranged: the repeated section **carries the target** a reference points at
    (`sec:mainsystem:pythonextensions` and its kind), so the label moves to the page title before the
    heading goes - the same rule that made RG3.15 a move and not a rewrite, and the strict build is
    what proves it.

    The item pages of the reference manual have the same shape - `# ObjectGround` then
    `## ObjectGround` - and are done in the same step. There the second heading carries
    `sec:item:<Item>`, which **every** item reference in the documentation uses, so it is the more
    delicate half of the two.

    **Done with one helper for both**, `latexToMarkdown.DropRepeatedTitle`: it removes a first
    heading that only repeats the page title and **returns its label**, which the emitters write
    above the `# ` title. Nothing lifts the headings that follow by hand - `NormalizeHeadings`
    rebuilds every level from the nesting it walks, so a heading that is gone takes its level with
    it, which is exactly what the table of contents needed.


<a id="rg3-28"></a>
**RG3.28** *(group RG3; from RG13.7, 2026-09-29)* **DONE 2026-09-29** — [log](exudynRevisionLog2026b.md#rg3-28) —
    **The developer documentation named paths of the old `main/` directory** (#2743): `WORKFLOW.md`,
    the layout block of `docs/dev/README.md` and a how-to note; the layout block also lost its
    hand-kept counts.

## RG4 — Implementation problems and bugs

Problems that are real, reproducible, and too deep to fix in passing. They are recorded here
rather than worked around silently, so the debt stays visible and each item can be closed on
evidence. An ordinary bug goes into the issue tracker and is fixed; a step appears here when the
fix needs a plan of its own.

Open in the tracker for this group: **#2423** (every C++ user error inspects the Python source to
find its file and line, on every raise).

<a id="rg3-21"></a>
**RG3.21** *(group RG3; maintainer 2026-09-24, clarified on their request 2026-09-27)* **DONE 2026-09-27** — [log](exudynRevisionLog2026b.md#rg3-21) — **The pages that
    still describe the state before a step that is done** (#2673). The list was made by measurement -
    496 identifiers, 129 settings paths, 11 deprecated call forms - and everything it found was fixed in
    one commit; the two scripts are the cheap way to ask again.

    **The reason**: six weeks of revision2026 and revision2026b changed behaviour that `docs/manual/`
    pages describe, and a page is corrected only when somebody walks past it - so some of them still
    describe how Exudyn worked in August.

    **What is in scope, because "which exact audit" is a fair question**: `docs/manual/` only.
    `docs/generated/` is rewritten from `definitions/` and the docstrings and cannot be stale;
    `docs/dev/` and `docs/howTo/` describe the workflow rather than the behaviour, and go with the step
    that changes the workflow. The subjects are the ones the closed issues name:

    | subject | what changed |
    |---|---|
    | the renderer keys and the dialogs | RG6.2.x: the settings dialog, the find, the fonts, the columns, what is stored |
    | the output directory | `exudyn.config.outputDirectory`, and which files follow it |
    | the star imports and `exudyn.config` | the deprecated module-level functions |
    | the command line | `python -m exudyn ...` |
    | the results monitor | it exists, and what it stores |
    | the override settings | `~/.exudyn/config.json`, which changes what a script does |
    | the deprecated names | what is on its way out, and by when |
    | the PlotSensor defaults | `None` means the default, and the file can set them |

    **The method, and why it is cheap**: `docs/manual/revisions.md` is the list of what changed for a
    user, and every closed issue of the two revisions carries its release note. So this is a comparison
    of two lists against the pages, not a re-reading of the manual.

    **The first deliverable is the LIST** - per page, what it claims against what the code does - and
    nothing else can be planned before it exists. It decides whether the rest is one commit or five.

<a id="rg3-22"></a>
**RG3.22** *(group RG3; maintainer 2026-09-25)* **DONE 2026-09-27** — [log](exudynRevisionLog2026b.md#rg3-22) — **The simulation settings section says how to look a
    setting up** (#2659). It explains the substructures and how to assign values, and says nothing
    about *finding* one. `python -m exudyn dialogs sim` opens the same tree the renderer's V key
    opens, with no model and no renderer, and it is the fastest way to answer "what is this setting
    called" - two sentences and a line of code in `docs/manual/` where the section is.

<a id="rg4-1"></a>
**RG3.19** *(group RG3; maintainer 2026-09-26)* **DONE 2026-09-26** — [log](exudynRevisionLog2026b.md#rg3-19) - **the arguments of
    a documented function are one per line, with the name in code** (#2665). The `Args:` block of a
    docstring was joined into one paragraph by `utilityDocsModel.Tags2Markdown`, so a function of ten
    arguments was one wall of text and the names were in the body font - in the Python utility
    functions and in the MainSystem extensions of the Python-C++ command interface, in the HTML and
    in the PDF, because both are built from the same Markdown. The maintainer reported it with a
    screenshot of the old RST pages, which had it right.

**RG4.1** *(group RG4; revision2026 step R10.1)* **Resolve the Windows/Linux differences in contact and friction models.** Measured 2026-09-10
    on manylinux_2_28 / cp313 / numpy 2.4.6, against the Windows reference values (Linux tolerance
    `3e-11`), relative error:

    | test | relative error |
    |---|---|
    | `coordinateSpringDamperExt.py` | 3.4e-11 |
    | `rigidBodySpringDamperIntrinsic.py` | 1.9e-10 |
    | `rollingDiscTangentialForces.py` | 1.5e-09 |
    | `contactSphereSphereTest.py` | 6.2e-09 |
    | `sphereTriangleTest2.py` | 1.7e-05 |
    | `generalContactCylinderTest.py` | 2.2e-05 |
    | `generalContactFrictionTests.py` | 4.9e-04 |
    | **`sphereTriangleTest.py`** | **1.6e+04** |

    These are **reproducible**, which distinguishes them from the non-deterministic tests of
    fact 24: they are not chaos, they are a difference with a cause that has not been found. The
    spread suggests more than one cause — four sit just above a very tight tolerance and look like
    ordinary floating-point divergence, three are 1e-5..1e-3 and are plausibly contact-state
    decisions taken differently, and **`sphereTriangleTest.py` is in another category entirely**:
    reference 3.8226, Linux 59370.97. Four orders of magnitude is a divergence or a blow-up, not
    an accuracy difference, and it should be looked at first and separately.

    Held in `UnresolvedOnLinux()` in `runTestSuiteRefSol.py`, excluded from the exit code **on
    Linux only** — the reference values are the Windows ones, and Windows must keep passing them
    (verified: 106/106 with the list active). Marked `L` in the per-test overview so a reader sees
    why a failure did not fail the run. **The list should shrink; every entry removed is a real
    fix.**

    Note what this nearly cost: a file-name based sensitive-test list would have swept
    `sphereTriangleTest.py` in with the chaotic contact tests, excluded it from the exit code
    permanently, and hidden a four-order-of-magnitude divergence behind a policy decision. That is
    the argument for populating these lists from measurement, restated as a concrete near miss.

    **macOS, measured 2026-09-26** by the maintainer on macOS ARM / Python 3.13 / V1.12.68.dev1
    (`tmp/testSuiteLog_V1.12.68.dev1_darwin-ARM-64bit-P3.13.txt`): **fifteen** differ - fourteen test
    models and one mini example. Nine of them are the Linux ones, so the two platforms fail mostly
    the same models and macOS is not a separate phenomenon; five are macOS only, and the mini
    example is the first one anywhere:

    | test | relative error | also on Linux |
    |---|---|---|
    | `generalContactFrictionTests.py` | 1.4e-04 | yes |
    | `generalContactCylinderTest.py` | 2.3e-05 | yes |
    | `sphereTriangleTest2.py` | 1.8e-05 | yes |
    | `ANCFbeltDrive.py` | 8.9e-06 | **no** |
    | `ANCFcontactCircleTest.py` | 1.5e-07 | **no** |
    | `connectorGravityTest.py` | 3.9e-13 of 1.0e+06 | **no** |
    | `contactSphereSphereTest.py` | 6.2e-09 | yes |
    | `ANCFslidingAndALEjointTest.py` | 1.4e-09 | **no** |
    | `generalContactImplicit1.py` | 3.9e-08 | yes |
    | `rollingDiscTangentialForces.py` | 4.0e-09 | yes |
    | `coordinateSpringDamperExt.py` | 2.1e-10 | yes |
    | `sliderCrank3Dbenchmark.py` | 2.9e-10 | yes |
    | `rigidBodySpringDamperIntrinsic.py` | 6.6e-11 | yes |
    | `ANCFgeneralContactCircle.py` | 7.8e-11 | **no** |
    | `ObjectConnectorRigidBodySpringDamper.py` (mini example) | 2.5e-09 | **no** |

    **Nothing is four orders of magnitude out**, which is the difference from the Linux table above:
    `sphereTriangleTest.py` - the one that mattered there - is not among them. The largest is 1.4e-04
    on a friction model, the same family that is unresolved on Linux, and the five macOS-only ones
    are ANCF contact and sliding models plus a gravity connector whose *absolute* difference is
    3.9e-07 on a value of a million. So the reading is: the same unexplained platform arithmetic,
    on a few more models, and no new category.

    - **RG4.1.1** **DONE 2026-09-26** — [log](exudynRevisionLog2026b.md#rg4-1-1) - `UnresolvedOnMacOS()`
      beside `UnresolvedOnLinux()`, applied on darwin, so the suite exits 0 there as it does on Linux.
      A **mini example** can be a known platform difference too, which nothing allowed for before.
    - **RG4.1.2** — the five macOS-only models. They are not the same question as the Linux nine:
      four are ANCF contact/sliding and one is a gravity connector, and none of them appears on
      Linux at all. Worth one look at whether they share a mechanism before being folded into the
      general question.

<a id="rg4-6"></a>
**RG4.6** *(group RG4; from RG4.5, 2026-09-26)* **DONE 2026-09-28** —
    [log](exudynRevisionLog2026b.md#rg4-6) — decided for **a binding a user can use as well**,
    `SC.renderer.StopSimulation(forceQuit=True)`, which does what closing the render window does.
    **A test hook for `forceQuitSimulation`** (#2674).
    #2616 fixed the behaviour - quitting the renderer **before** a simulation starts raised where
    quitting **during** it did not - and nothing can test it: the flag is set by the renderer thread
    from a key press or a closed window, and there is no binding for it. The fix is therefore checked
    by hand and stays checked by hand. The decision the step takes is **which of the two**: a
    binding a user could also use (on `mbs.systemData` or the renderer) or a hook that exists only
    for the test.

<a id="rg4-7"></a>
**RG4.7** *(group RG4; from #2423)* **CLOSED 2026-09-29, done by revision2026 step R6.3.5** (a49d501f):
    `PyGetCurrentFileInformation` reads `f_code.co_filename` and `f_lineno` of the frame and no longer
    calls `inspect.getframeinfo`. **Every C++ user error inspects the Python source for its file
    and line.** `PyError` and `PyWarning` call `PyGetCurrentFileInformation`
    (`src/Main/Stdoutput.cpp:259`), which calls `inspect.getframeinfo`: that resolves the module by
    scanning `sys.modules` and then reads the source file. The cost grows with the number of
    imported modules, and `parameterConversionTest.py` pays it about 38000 times - which is also the
    measurement that found it. A raised error is not a hot path, but a *probe* is, and the test model
    that probes every parameter of every item is the one place both meet.

<a id="rg4-8"></a>
**RG4.8** *(group RG4; maintainer 2026-09-28)* **`ObjectBeamGeometricallyExact` (3D): analyse the
    implementation** (#2730). *"The 3D GeometricallyExactBeam has some defects and is still under
    development. So, you would find some errors in the implementation - so keep this item open and
    add a step and issue to RG4."* The analysis says what the element computes, where it departs from
    the formulation, and what is missing; its reference page (RG13.5.2) waits for it. **The rule this
    sets**: an item found with a larger defect while RG13 documents it gets an RG4 step for a deeper
    analysis, not a fix inside RG13.

    **The material** (maintainer, 2026-09-29): a colleague's comparison of the element with an own
    SE(3) beam after Sonneville, Cardona & Brüls (2014), done with Claude (`tmp/beam_element_comparison_vs_exudyn.pdf`,
    the code not included). Its findings, from the source and from runs:
    - the same as the literature: the SE(3) interpolation; `TExpSE3`/`TExpSE3Inv` (the active form after
      Hante 2022 equals Sonneville's to machine precision; the commented-out one loses accuracy at small
      angles); the elastic forces and their Jacobian, checked against the running element;
    - different: the **mass matrix** - lumped, block-diagonal per node, the rotational block from each
      node's `G_local` - where Sonneville's Eq. (81) is consistent and couples the nodes; the
      **gyroscopic terms** per node, $\tomega \times (\Jm \tomega)$, not integrated over the element
      (Eq. 78-82); **no Jacobian of the mass matrix and the gyroscopic terms**, and the elastic Jacobian
      misses the $\Gm_{local,q}$ and $\Rot\tp_q$ chain rule terms (marked *MISSING* in the source);
      **gravity** by midpoint shape functions, without the nodal torques; the **velocity** of a point
      interpolated linearly (*"not consistent with position"*); **no body markers**
      (`GetAccessFunctionBody` not implemented);
    - measured: static large deformation agrees to $5\cdot10^{-9}$ m; a flexible pendulum of 10 elements
      released from horizontal follows `ObjectBeamGeometricallyExact2D` and the SE(3) reference, while
      the 3D element departs visibly after $t = 0.5$ s.

    The open issues of the element are sub-steps here, each **inspected first** - some may be solved:
    - **RG4.8.1** reproduce the comparison with a script of our own: the element fixed at both nodes at a
      prescribed configuration, forces and Jacobian from the joint reactions; kept in `tmp/`;
    - **RG4.8.2** a planar dynamic test: the flexible pendulum of the comparison with
      `ObjectBeamGeometricallyExact2D`, the 3D element and `ObjectANCFCable2D` - the test model that shows
      the dynamic difference;
    - **RG4.8.3** a 3D test against the literature: the right-angle frame (L-shape) with its published
      response (#1499);
    - **RG4.8.4** the mass matrix and the gyroscopic (quadratic velocity) terms - consistent after
      Sonneville Eq. (78)-(82), or lumped with the correct terms (#1273);
    - **RG4.8.5** the Jacobian: the missing $\Gm_{local,q}$ and $\Rot\tp_q$ terms, and the Jacobian of
      the mass and gyroscopic terms (#1550, #1100);
    - **RG4.8.6** the reference configuration in the residual and the Jacobian - a pre-curved element,
      *"h0 must contain the reference configuration"* in the source (#1494);
    - **RG4.8.7** distributed loads with their nodal torques, a consistent velocity field, body markers -
      `GetAccessFunctionBody` throws before its switch, while the element declares four access functions
      (found in RG9.3.1);
    - **RG4.8.8** whether #736 (*"include GeomExactBeam3D as provided by Jan Tomec"*) is this element
      or superseded by it;
    - **RG4.8.9** then the reference page (RG13.5.2) and the MiniExample (RG13.6).

<a id="rg4-14"></a>
**RG4.14** *(group RG4; 2026-09-29)* **`ObjectBeamGeometricallyExact2D`: a test of the 3-node element**
    (#2208) - the only open issue of the planar element.

<a id="rg4-15"></a>
**RG4.15** *(group RG4; maintainer 2026-09-29)* **The open bugs and fixes before 1.13.** The maintainer: *"Before
    the upcoming release, we definitely should try to resolve the open BUGs"*, and the urgent FIX issues.
    Checked 2026-09-29, each against the code or with a run - see the [log](exudynRevisionLog2026b.md#rg4-15).
    Closed as resolved or no longer applying: #738, #1048, #1772, #1846 (duplicate of #1845), #1889.
    The graphics ones are RG6.8. What remains, in the order proposed:
    - **RG4.15.1** **DONE 2026-09-29** (#2749) — one drop, three contact objects:
      the test model `contactComparisonTest.py`, the check in compensation for #738;
    - **RG4.15.2** (#2750) `ObjectContactCoordinate` gets the contact law of `ObjectContactSphereSphere` -
      `contactStiffnessExponent`, `restitutionCoefficient`, `impactModel`, `minimumImpactVelocity` -, and the
      comparison test extends to them; its release step size is also the one difference the test found;
    - **RG4.15.3** (#830) the explicit solvers do no post Newton step - contact and switching items are
      not updated: a warning at the start of an explicit solve with such items, or the update after each
      step;
    - **RG4.15.4** (#2127) `ObjectContactSphereTorus`: momentum conservation - a free ball in a free ring,
      the sum of the torques on both bodies must vanish;
    - **RG4.15.5** (#1639) a repeated `mbs.SolveDynamic` with `ObjectFFRFreducedOrder` diverges -
      reproduced by solving `objectFFRFreducedOrderTest.py` twice;
    - **RG4.15.6** (#1888) `mbs.GetDictionary()` works with a symbolic user function, but
      `mbs.SetDictionary()` of that dictionary fails (*"Unable to cast ... symbolic.UserFunction"*);
    - **RG4.15.7** (#1424) the numerical ODE1 Jacobian with a connector whose two markers are on the same
      object - the duplicate coordinates, as for ODE2 (`CSystem.cpp` says *"ODE1 needs to be checked as
      well"*);
    - **RG4.15.8** (#1848, #1947) `GeneralContact`: implicit sphere-triangle contact and its friction against
      `ObjectContactSphereSphere` - the drop of RG4.15.1 as a fourth case.

    After 1.13, not urgent: #1845 (`ComputePostProcessingModes` with threads), #1565
    (`InitializeFromRestartFile`), #2109 (DOPRI5 step size at discontinuities), #2326 (the slider crank
    benchmark after the revised IFToMM model).

<a id="rg4-9"></a>
**RG4.9** *(group RG4; from RG13.5.3, 2026-09-28)* **DONE 2026-09-29** —
    [log](exudynRevisionLog2026b.md#rg4-9) — **The cable and beam shape markers accept any
    body** (#2731). `MarkerBodyCable2DShape`, `MarkerBodyCable2DCoordinates` and `MarkerBodyBeamShape`
    have no consistency check of their body; `Assemble()` accepts them on a rigid body, and computing
    the marker data then treats it as an ANCF cable - an `InternalError` with range checks, undefined
    behaviour in the fast module. The analysis: which bodies each marker can serve, a
    `CheckPreAssembleConsistency` that says so, and whether `MarkerBodyBeamShape` - described as for
    *"a 3D beam finite element"* - should serve other beams than `ObjectANCFCable`.

    **And the relative-coordinate markers** (maintainer, 2026-09-28): `MarkerBodiesRelativeTranslation-`
    and `...RotationCoordinate` declare `Position` and `Orientation` besides `Coordinate`, so their
    pages list 28 connectors, joints and loads that could use them, a spring-damper among them. To be
    checked here whether such a combination computes anything meaningful; if not, they are declared as
    coordinate markers only.

<a id="rg4-10"></a>
**RG4.10** *(group RG4; from RG13.5.2.3, 2026-09-28)* **DONE 2026-09-29** —
    [log](exudynRevisionLog2026b.md#rg4-10) — **`ObjectGenericODE2` and `ObjectKinematicTree`
    admit the general body markers, which then fail** (#2734). Both declare `TranslationalVelocity_qt`
    and `AngularVelocity_qt` so that their own markers pass `CSystem::CheckSystemIntegrity`; the same
    declaration admits `MarkerBodyPosition`, `MarkerBodyRigid` and `MarkerBodyMass`, whose computation
    calls `GetAccessFunctionBody`, which raises *"not available"* for both. The declaration carries
    `bodyMarkers=False` since RG13.5.2.3, which the item pages use; the step lets the integrity check use
    it as well, so that `Assemble()` refuses the combination.

<a id="rg4-11"></a>
**RG4.11** *(group RG4; from RG13.5.2.4, 2026-09-28)* **DONE 2026-09-29** —
    [log](exudynRevisionLog2026b.md#rg4-10) — **`ObjectContactCoordinate` ignores
    `activeConnector`, and its output variable `Distance` raises** (#2735). **Extended in RG13.5.2.5**:
    four objects declare output variables whose `GetOutputVariableConnector` raises *"not implemented"* -
    `ObjectContactCoordinate` and `ObjectContactCircleCable2D` (`Distance`), `ObjectJointRevolute2D`
    (`Displacement`, `Rotation`), `ObjectJointPrismatic2D` (`Distance`, `Rotation`); implement them or
    declare none. `ComputeODE2LHS` computes the
    contact force without looking at `activeConnector`; `GetOutputVariableTypes` declares `Distance`
    while `GetOutputVariableConnector` raises *"not implemented"*, so a sensor asking for it passes
    `Assemble()` and fails in the simulation. Small; an RG4 step by the maintainer's rule for defects
    found while documenting.

<a id="rg4-12"></a>
**RG4.12** *(group RG4; from RG13.6.1, 2026-09-29)* **`NodeGenericAE` cannot be used** (#2736): it
    provides only `GenericAE`, no object requests that type, no node marker attaches to it, and no
    example, test model or module of the package uses it - a node with algebraic coordinates and no
    object to write their equations. Either an object takes it (its description names linear state
    space systems) or it is deprecated. Its page has no MiniExample until then.

    **Needs the maintainer's decision** (2026-09-29): there is no mechanism to deprecate an item class
    (RG12.2 is for parameters), and making it usable needs an object that writes algebraic equations for
    its coordinates - e.g. a generic algebraic object with a residual user function, or an extension of
    `ObjectGenericODE1`/`ObjectGenericODE2` that takes an AE node. Neither is a fix.

<a id="rg4-13"></a>
**RG4.13** *(group RG4; from RG13.5.3.1, 2026-09-29)* **DONE 2026-09-29** —
    [log](exudynRevisionLog2026b.md#rg4-13) — **`ObjectKinematicTree`: the position Jacobian of
    a prismatic joint rotates the axis twice** (#2740). `CObjectKinematicTree::ComputeJacobian` sets the
    column to `rotJoint*axis` where `axis` is already global. Measured: a force along a prismatic axis
    behind a revolute joint at 90 degrees moves nothing. Wrong generalized forces for every marker,
    load and connector behind a rotated prismatic joint. One line and a test model; **high priority**.

<a id="rg4-2"></a>
**RG4.2** **DONE 2026-09-26** (#2413) — [log](exudynRevisionLog2026b.md#rg4-2) —
    **`ObjectContactConvexRoll.pContact` is a computed value that Python reads**, which is what the
    maintainer decided on 2026-09-26: *"make pContact same as the variables in FFRF that are computed
    internally and can be read by the Python interface"* - and **not** a data variable, which is what
    this step said from revision2026 step R10.2 until then. The step closes with the measurement that
    `pContact` already was one, and with `rBoundingSphere` beside it, which was not.

<a id="rg4-3"></a>
**RG4.3** *(group RG4; revision2026 step R10.3)* **Explicit integration cost** (#2398, #2400). With the default dense linear solver
    an explicit step on a chain of point masses costs O(N^2) (168 ms per step at N=2000; 400 times
    faster with `EigenSparse`), and `computeMassMatrixInversePerBody` changes nothing unless a
    sparse solver is selected as well. At least warn at large N; better, avoid the global solve
    in explicit integration where the flag makes it unnecessary.


<a id="rg4-4"></a>
**RG4.4** **DONE 2026-09-23** (#2603) — [log](exudynRevisionLog2026b.md#rg4-4) —
    **Two lines of Python segfault the process.** The top settings class calls `Init(this)` in
    its constructor, so a structure Python builds links itself; it defines a copy constructor and
    a copy assignment that re-link the **copy**, which is the answer to the question the step
    left open; and every deprecated forwarding checks its backlink and raises instead of
    dereferencing `nullptr`. All 93 deprecated members of a standalone
    `exu.VisualizationSettings()` are read and written in a test.

<a id="rg4-5"></a>
**RG4.5** **DONE 2026-09-23** (#2616) — [log](exudynRevisionLog2026b.md#rg4-5) —
    **Quitting the renderer before a simulation started raised, quitting during it did
    not.** `CSolverBase::SolveSystem` returned `false` when `forceQuitSimulation` was
    already set, and `SolveDynamic` reads `false` as a failure, so a user who closed the
    render window while the script waited got *DYNAMIC SOLVER FAILED* and a traceback. It
    returns `true` now.

    **Left open deliberately**: `forceQuitSimulation` is set by `GlfwClient.cpp` alone and
    has no Python binding, so the path cannot be reached without a window and has no test.
    A binding or a test hook would make "stopped" against "failed" testable; it is worth a
    step of its own if the maintainer wants it.

## RG5 — Performance

Measurement first, then the code that is actually hot. revision2026 step R2.16 measured the linear
algebra and revision2026 step R5.11 built the fast-module test path; what is missing is a benchmark that is
maintained rather than written once, and the vectorization work it would guide.

<a id="rg5-1"></a>
**RG5.1** *(group RG5, before RG5.2; maintainer decision 2026-09-16; revised 2026-09-17; revision2026 step R11.2)* **A maintained
    micro-benchmark for the linear algebra, inside Exudyn** (#2397, from revision2026 step R2.16).

    *The starting point named by the original text is gone.* This step used to say "replace the
    dead sweep in `PyTest()` (`src/Pymodules/pythonTests.cpp`)"; that file was **deleted** in step
    revision2026 step R5.4.11 (#2484) as outdated and misleading. Nothing is replaced, then - this step **writes** the
    benchmark. What the deleted code was is still worth knowing, because it says what not to
    repeat: it sat inside comment blocks and `if (0)`, `exu.Test()` was not even bound in a release
    build (`EXUDYN_RELEASE`), and it timed hand-written loops rather than the vector code the
    solver actually runs. It can be read in commit `ad54a93` if anyone wants it.

    Write a benchmark that is compiled into **every** build and runs the **real** operations -
    `Vector`/`ResizableVectorParallel` add, subtract, scale and `MultAdd`, `SlimVector`/`Matrix3D`
    products, `ConstSizeMatrix` and matrix-vector products - over a size sweep that crosses
    `ResizableVectorParallelThreadingLimit`, single- and multithreaded. Exposed as
    `exudyn.special.RunLinalgBenchmark()` (flags for sizes, repeats, which groups), so the user
    interface grows by one function; `exu.special` already exists (`PySpecial` in
    `src/Main/Experimental.h`, bound from `definitions/pybindModule.py`). This also lets a user
    measure their own CPU: which build flags and how many threads make sense there. The tool-level
    benchmark `tools/benchmarks/avx2Benchmark.py` stays as the solver-level counterpart.

    The pattern to follow is `src/Linalg/symbolicCppDemo.h` from revision2026 step R5.4.12: named functions, one
    topic each, compiled by being included, and a header that says what it is for.

<a id="rg5-2"></a>
**RG5.2** *(group RG5, after RG5.1; revision2026 step R11.3)* **Make the hot linear algebra vectorizable.** Revision2026 step R2.16 measured that the
    solver time of long-vector models sits in `ODE2RHS` (73-91 % of the explicit runs), i.e. in
    per-object 3x3 and short-vector work, not in the long-vector loops that AVX2 accelerates, and
    `ConstSizeMatrix` carries its size at runtime, so the compiler cannot unroll it. Candidates:
    compile-time sizes where the size is known, more use of homogeneous transformations in the
    rigid-body kinematics, and the object loop of `ODE2RHS` itself. Steered by the benchmark of
    RG5.1; a compile-flag decision alone (revision2026 step R2.16) cannot achieve this.


## RG6 — Graphics and rendering

The renderer is GLFW and OpenGL, it runs in its own thread, and **nothing in the test suite draws
anything** (RG2.1) - which is why a rendering revision is worth planning rather than improvising.
This group is that revision and what has to happen before it can start.

<a id="rg6-1"></a>
**RG6.1** **DONE 2026-09-24** (#2645) — [log](exudynRevisionLog2026b.md#rg6-1) —
    **OpenVR is removed**, which is what the rendering revision was waiting for. The
    interface and its header, every `__EXUDYN_USE_OPENVR` guard, the `--openvr` flag and
    `-lopenvr_api`, the vendored SDK and the prebuilt library, the settings under
    `interactive.openVR`, the `openVR` entry of the render state, the example and the manual
    section are gone: **2.1 MB in 13 tracked files**. A user who needs it stays on 1.11, and
    the changelog and the revisions chapter say so.

<a id="rg6-2"></a>
**RG6.2** **DONE 2026-09-23** (#2591), **reopened and closed again the same day for RG6.2.12
    to RG6.2.15** — [log](exudynRevisionLog2026b.md#rg6-2) —
    **The settings dialogs, and the shape of `GUI.py`.** The seven complaints the maintainer
    named on 2026-09-22 - a restricted table, no type hints, no font scaling off macOS,
    columns that cannot be adjusted, descriptions behind a special key, no inline editing,
    unhandy combo boxes - are answered by RG6.2.1 to RG6.2.10, and RG6.2.5 (a second front
    end) is dropped. The review that found the cause of each of them, and the three findings
    that shaped the sub-steps - the dialog is not specific to `visualizationSettings`, 220 of
    the 775 lines of `rendererPythonInterface.cpp` were Python, and no test touched any of
    it - are in the [review](exudynRevisionLog2026b.md#rg6-2-review). RG6.2.11 to RG6.2.25
    are what using the dialog produced afterwards.

<a id="rg6-2-1"></a>
**RG6.2.1** **DONE 2026-09-22** (#2595) — [log](exudynRevisionLog2026b.md#rg6-2-1) — **The dialogs leave the C++.** The help dialog, the command window, the quit
    question and the right-mouse dialog become functions of `exudyn.misc.GUI`, and
    `rendererPythonInterface.cpp` keeps one call each — which is what it already does for the
    settings dialog. The window setup that the C++ assembles by string concatenation today
    (`-topmost`, `-alpha` from `visualizationSettings.dialogs`) becomes one helper on the Python
    side, where those settings are readable anyway. **No behaviour changes**; what changes is that
    220 lines of Python become Python: ruff reads them, the stub check reads them, and a person
    editing them gets a syntax error instead of a runtime one.

<a id="rg6-2-2"></a>
**RG6.2.2** **DONE 2026-09-23** (#2596) — [log](exudynRevisionLog2026b.md#rg6-2-2) — **The layer under the widgets gets tests.** `ConvertString2Value`,
    `ConvertValue2String`, `CheckType` and `GetComboBoxListsDict` decide what a typed value
    becomes, and no test calls them. They need no window, so pytest can: every type the settings
    structures actually use (20 of them, `bool` to `VectorFloat`), the round trip value -> string
    -> value, and the rejection of a wrong one.

<a id="rg6-2-3"></a>
**RG6.2.3** **DONE 2026-09-23** (#2597, #2601) — [log](exudynRevisionLog2026b.md#rg6-2-3)
    **The six small complaints.** The column widths (`tree.column(...)` was never called),
    the enum lists built from the module instead of the hard-coded three, the type shown in
    the table and named in the error message, and the description in a tooltip rather than
    behind the key `h`. `dialogs.fontScaling` is RG6.2.3.1, the only one that leaves Python.
    The three defects RG6.2.2 found went with it: `CheckType` had no branch for an enum, `:`
    was not a valid file name character, and a value that passed `CheckType` but failed
    `ConvertString2Value` was dropped with a `print()`.

<a id="rg6-2-3-1"></a>
**RG6.2.3.1** **DONE 2026-09-23** (#2602) — [log](exudynRevisionLog2026b.md#rg6-2-3-1) — **One `dialogs.fontScaling` for every platform.** `if not IsApple(): fontFactor = 1`
    — off macOS the font factor is forced to 1 and only the row height follows the display
    scaling, so the maintainer cannot make the dialog readable on Linux. The setting is called
    `dialogs.fontScalingMacOS`, so the fix is a rename with a **deprecation** — the mechanism of
    RG12.1, `Deprecated(since, expires)` in `definitions/structureDefsVisualizationSettings.py`.
    It is the only part of RG6.2.3 that leaves Python: the definitions regenerate the C++ settings
    headers, so it needs a build and it can break one.

<a id="rg6-2-4"></a>
**RG6.2.4** **DONE 2026-09-23** (#2604) — [log](exudynRevisionLog2026b.md#rg6-2-4) — **Inline editing** — the one real rewrite: the value is edited in the cell
    instead of in a separate field at the bottom of the window that swaps with a combo box by
    z-order. The bottom row became the **line that sets the selected item**, with a copy button
    (maintainer, 2026-09-23) — which is what RG12.3 produces for the whole structure.

<a id="rg6-2-5"></a>
**RG6.2.5** **DROPPED 2026-09-23** *(maintainer)* — **A second front end.** It was planned
    as a question, not as work: whether another toolkit is wanted once RG6.2.1-RG6.2.4 have shown
    what the interface between the data and the widgets is. The answer is no. tkinter runs
    everywhere and needs no installation, which is the reason it was kept in the first place, and
    RG6.2.8 to RG6.2.10 have made it do what was asked of it. What the step would have needed is
    ready in any case — `GetDictionaryWithTypeInfo()` is bound for every settings structure, and
    everything below the widgets is module level functions on dictionaries since RG6.2.8 — so a
    second front end remains possible without this step standing open.

<a id="rg6-2-6"></a>
**RG6.2.6** **DONE 2026-09-23** (#2591) — [log](exudynRevisionLog2026b.md#rg6-2-6) — **The key bindings are written down three times**: `GlfwClient.cpp` implements them,
    `docs/manual/GUI.md` tabulates them in 64 rows, and the help dialog prints its own 55-line
    text. Two of the three are prose that nothing keeps in step with the first. One source — a
    table in Python — could feed both the dialog and a generated page, the way `definitions/`
    feeds the reference manual (rule 10). Raised here because RG6.2.1 moves the third copy
    without fixing the duplication.

<a id="rg6-2-7"></a>
**RG6.2.7** **DONE 2026-09-23** (#2591) — [log](exudynRevisionLog2026b.md#rg6-2-7) —
    **`GUI.py` is cleaned up, last** *(maintainer, 2026-09-23)*: the module every other
    sub-step edits, tidied when the shape had settled - 55 lines of commented-out code, the
    dead `#EXAMPLE` dictionary, 10 bare `print()` calls, the font and scaling setup
    duplicated between the two dialogs, and `treeEditOpenItems`, module level mutable state
    that made the open folders outlive the dialog and every SystemContainer in the process.

<a id="rg6-2-8"></a>
**RG6.2.8** **DONE 2026-09-23** (#2605) — [log](exudynRevisionLog2026b.md#rg6-2-8) —
    **The bottom row reads like code, and says each thing once** *(maintainer, 2026-09-23,
    after trying RG6.2.4)*: the pastable line in a smaller fixed font with a box of its own,
    the label that repeated the pop-up description gone, `copy` renamed to say what it
    copies, and the type and size kept - they are what the pop-up does not say.

<a id="rg6-2-9"></a>
**RG6.2.9** **DONE 2026-09-23** (#2606) — [log](exudynRevisionLog2026b.md#rg6-2-9) —
    **A changed value is visible, and every change can be copied at once** *(maintainer,
    2026-09-23)*. Nothing marked the rows a user had edited, so after ten edits in four
    folders the ten could not be found again. A changed row is coloured, and a second button
    copies all changes as the lines that set them. It also answers the question RG12.3
    shares: changed against the values the dialog opened with, and against the defaults -
    both, in two views.

<a id="rg6-2-10"></a>
**RG6.2.10** **DONE 2026-09-23** (#2607) — [log](exudynRevisionLog2026b.md#rg6-2-10) —
    **Find a setting** *(maintainer, 2026-09-23)*. Several hundred values in a tree of
    folders, and the only route to one was knowing its folder. CTRL-F, names before
    descriptions, a drop-down of the hits as dotted paths, Return to the first and F3 to the
    next. The two rejected shapes - filtering the tree, a separate result window - and why,
    are in the log.

<a id="rg6-2-11"></a>
**RG6.2.11** **DONE 2026-09-23** (#2608) — [log](exudynRevisionLog2026b.md#rg6-2-11) —
    **The catalogue of optional features, decided.** The maintainer went through it on
    2026-09-23: reset, undo and the *changed only* view were already built; load and save to
    a file, units in the description and apply-while-open are answered with no or with "that
    is already the behaviour"; the same dialog for `simulationSettings` survives as RG6.2.18.
    **Remember the window** stays undecided, and the log carries the rule that would make it
    safe - restore the size always, the position only when it still lies inside the virtual
    desktop - so that the step, if it is ever taken, starts from it.

<a id="rg6-2-12"></a>
**RG6.2.12** **DONE 2026-09-23** (#2612) — [log](exudynRevisionLog2026b.md#rg6-2-12) —
    **59 untouched settings were called changed, and a folded folder hid a change**
    *(maintainer, 2026-09-23, from demo 2)*. The reference was `exu.VisualizationSettings()`,
    and a `SystemContainer` initialised 59 of those settings - the lights and the raytracer
    materials - so a user who had changed nothing saw every one of them reported. The
    reference is the state a user starts from, and a folder is marked when its subtree holds
    a change. (RG6.2.20 later made those 59 defaults of the structure itself.)

<a id="rg6-2-13"></a>
**RG6.2.13** **DONE 2026-09-23** (#2613) — [log](exudynRevisionLog2026b.md#rg6-2-13) — **The find bar needs no button, and says nothing when it is idle** (#2613)
    *(maintainer, 2026-09-23)*. The search runs while the text is typed, so the **find** button is
    removed; and the drop-down of the hits looks like something to click before anything has been
    searched for, so it is **greyed out** until there is a search text.

<a id="rg6-2-14"></a>
**RG6.2.14** **DONE 2026-09-23** (#2614) — [log](exudynRevisionLog2026b.md#rg6-2-14) —
    **Reset, revert, undo, close - and the windows stay in front** *(maintainer,
    2026-09-23)*. A second button row: *diff to default* and *this session* on the left,
    reset, revert, undo and close on the right, each with a tooltip; the tooltips of the tree
    open after 0.5 seconds because they were in the way while the mouse crossed it.

<a id="rg6-2-15"></a>
**RG6.2.15** **DONE 2026-09-23** (#2615) — [log](exudynRevisionLog2026b.md#rg6-2-15) — **A settings folder has a description, and nothing shows it** (#2615)
    *(maintainer, 2026-09-23)*. Every settings structure carries `classDescription` in
    `definitions/— ` *"General settings for visualization that influence all windows, default
    values, autofit, multithreading, etc."* — and it reaches the reference manual and the C++
    header, but **not `GetDictionaryWithTypeInfo`**: only the leaves have a description there, so
    the dialog has nothing to show when the mouse is over a folder. The emitter puts the class
    description into the dictionary under a **reserved key**, the way `itemIdentifier` is
    reserved, and the tooltip shows it for a folder.

<a id="rg6-2-16"></a>
**RG6.2.16** **DONE 2026-09-23** (#2621) — [log](exudynRevisionLog2026b.md#rg6-2-16) —
    **The window with the changes was invisible** *(maintainer, 2026-09-23)*. *"diff to default"
    and "this session" show nothing.* The content was right — a probe that builds the dialog
    without mapping a window finds the changes and raises nothing — so the window never became
    visible: the dialog is topmost and the `Toplevel` opened at the same place behind it, which
    looks exactly like a button that does nothing. Two more from the same message: *this session*
    is called **changes since start**, and the rows at the top and the bottom span the columns of
    the tree only, so that no button sits under its vertical scroll bar.

<a id="rg6-2-17"></a>
**RG6.2.17** **DONE 2026-09-23** (#2623) — [log](exudynRevisionLog2026b.md#rg6-2-17) —
    **Opening the dialog re-pointed `exudyn.sys` at a throw-away container.** Found while
    chasing RG6.2.16 and the cause of it: constructing an `exudyn.SystemContainer()` replaces
    `exudyn.sys['currentRendererSystemContainer']`, and RG6.2.12 created one to read the
    defaults - so from the moment the dialog opened, the redraw signal went to the throw-away
    container instead of to the renderer. The entry is saved and restored around it.

<a id="rg6-2-18"></a>
**RG6.2.18** **DONE 2026-09-24** (#2624) — [log](exudynRevisionLog2026b.md#rg6-2-18) —
    **The same dialog for `simulationSettings`** — and for the visualization settings and the
    key bindings, from the command line the maintainer proposed:
    **`python -m exudyn dialogs vis | sim | help`**. The command dialog is deliberately not among
    them.

<a id="rg6-2-19"></a>
**RG6.2.19** **DONE 2026-09-23** (#2625) — [log](exudynRevisionLog2026b.md#rg6-2-19) —
    **Opening the settings dialog closed the render window** *(maintainer, 2026-09-23)*.
    `MainSystemContainer()` attaches to the render engine in its constructor and detaches in
    its destructor, so the temporary container of RG6.2.12 took the window away from the
    container that owns it. The dialog creates no container any more.

<a id="rg6-2-20"></a>
**RG6.2.20** **DONE 2026-09-23** (#2626) — [log](exudynRevisionLog2026b.md#rg6-2-20) —
    **The defaults of the lights and the raytracer materials are hidden in C++ constructors.**
    They are defaults of the **structure** now: `StructureParameter` gained `memberDefaults`, the
    89 values are in `definitions/structureDefsVisualizationSettings.py`, the generated
    constructors carry them, the C++ that set them afterwards is gone, and the reference says
    what `material1` and `light2` start from. Every one of the 89 was compared against the C++ it
    replaces and is identical.

<a id="rg6-2-21"></a>
**RG6.2.21** **DONE 2026-09-23** (#2627) — [log](exudynRevisionLog2026b.md#rg6-2-21) —
    **The dialog stopped asking, and undo goes back one whole state** *(maintainer,
    2026-09-23)*. Reset and revert each asked a yes/no question nobody had asked for -
    *"we can always revert to initial settings and there is undo"* - and undo went back one
    value and was switched off by exactly the two clicks one would want to undo. Both
    questions are gone and the dialog keeps a stack of whole states.

<a id="rg6-2-22"></a>
**RG6.2.22** **DONE 2026-09-23** (#2630) — [log](exudynRevisionLog2026b.md#rg6-2-22) —
    **A double click on a bool no longer toggled it** *(maintainer, 2026-09-23)*. It used to
    switch `True`/`False`, and the cause is the cell editor of RG6.2.4 (#2604): the tree binds
    `<ButtonRelease-1>` to the editor, and for a `bool` that editor is a **Combobox placed over
    the value cell**, so the second click of a double click landed on the combobox and the
    `<Double-1>` binding on the tree never fired. The toggle code itself was untouched and
    unreachable. On a bool row the cell edit is **scheduled** now and a double click cancels the
    job; every other type keeps the editor that opens at once.

<a id="rg6-2-23"></a>
**RG6.2.23** **DONE 2026-09-23** (#2631) — [log](exudynRevisionLog2026b.md#rg6-2-23) —
    **`dialogs.fontScaling` only worked at 0** *(maintainer, 2026-09-23)*: *"1.0 gives a
    larger font, but much too small row height and smaller column width"*. The row height and
    the column width were computed from the font scaling, which is not what decides how large
    a glyph comes out - the point-to-pixel conversion follows the tk scaling of the display.
    Both are measured from the font now, by `DialogRowMetrics`.

<a id="rg6-2-24"></a>
**RG6.2.24** **DONE 2026-09-24** (#2634) — [log](exudynRevisionLog2026b.md#rg6-2-24) —
    **The dialogs opened from the command line were larger and blurred** *(maintainer,
    2026-09-24)*. Two causes, both from the missing renderer: the process was not **DPI aware**
    — GLFW makes it so when the render window opens, and without it Windows draws tkinter at 96
    dpi and stretches the bitmap — and `GetExudynDisplayScaling()` read the scaling from the
    renderer's state and returned **1** when there is none. The process makes itself DPI aware
    before the first window, and tkinter is asked for the scaling when no renderer can be.

<a id="rg6-2-25"></a>
**RG6.2.25** **DONE 2026-09-24** (#2635) — [log](exudynRevisionLog2026b.md#rg6-2-25) —
    **The combo box of an enum repeated the type name in every entry** *(maintainer,
    2026-09-24)*. `contour.outputVariable` offered `OutputVariableType.Displacement` and 32
    more, all beginning with the same 19 characters, in a box as wide as the value column -
    and the type is in the column beside it. The list shows the entry without its type; the
    settings structure and the generated code line keep the full name.

<a id="rg6-2-25-1"></a>
**RG6.2.25.1** **DONE 2026-09-24** (#2640) — [log](exudynRevisionLog2026b.md#rg6-2-25-1) —
    **The value cell showed the type name again as soon as the combo box collapsed**
    *(maintainer, 2026-09-24)*. RG6.2.25 shortened the list and left the cell, which is the
    same problem one step later. The short name is **the** value string of an enum now -
    `ConvertValue2String` produces it, so the cell, the marking of a changed value and the
    comparison of `ChangedSettings` all speak it - and `ValueLiteral` puts the type back,
    because the generated Python is the only place that needs the full name.

<a id="rg6-2-26"></a>
**RG6.2.26** **DONE 2026-09-24** (#2639) — [log](exudynRevisionLog2026b.md#rg6-2-26) —
    **The tooltips were invisible while the dialog is topmost** *(maintainer, 2026-09-24)*, in
    the renderer and from the command line alike; turning `dialogs.alwaysTopmost` off and
    reopening made them work, which named the cause. A tooltip is a `Toplevel` of the dialog
    and had no topmost flag, and on Windows a topmost window is always above one that is not -
    the mechanism that hid the window of the changes in #2621. The tooltip window carries the
    flag now, so the dialog keeps the topmost it needs to block the render window.

<a id="rg6-2-28"></a>
**RG6.2.28** **DONE 2026-09-23** (by RG6.2.20, #2626) — [log](exudynRevisionLog2026b.md#rg6-2-28) —
    **The lights and the raytracer materials are defaults of the structure.** Proposed on 2026-09-26
    as the blocker of RG12.9 to RG12.11, on the strength of the RG6.2.12 log; the maintainer read
    the C++ and said it looked done already, and it is. **RG6.2.20 did it three days earlier**: the
    89 assignments moved out of the C++ constructor into
    `definitions/structureDefsVisualizationSettings.py`, `containerInitialisedSettings` is empty,
    `DefaultSettingsDictionary` takes the plain constructor, and
    `testASystemContainerInitialisesNothingBeyondTheDefaults` requires the difference to stay empty.

    **Measured on 2026-09-26 before anything was changed**: `exu.VisualizationSettings()` and
    `SystemContainer().visualizationSettings` differ in **0 of 470** settings. #2678 is closed as
    obsolete, and **nothing blocks RG12.9 to RG12.11**.

    The lesson is worth the two lines: the plan said "the container initialises 59 settings"
    because a log entry from eight steps earlier said so, and a log entry is what was true **when
    it was written**. A premise that decides the order of three steps is measured, not read.

<a id="rg6-3"></a>
**RG6.3** *(group RG6; maintainer 2026-09-22)* **CLOSED 2026-09-27, superseded** (#2583) - the
    maintainer: replaced by the new ways to go. The headless call is `SC.renderer.GetGraphicsData()`
    (#2700), which returns the data itself; the low-resolution images are RG2.3.3.4. The step as it
    was: **The renderer extraction functions are not shaped
    for testing** (#2583). `RedrawAndGetImage()` and `GetRenderState()` exist and are what a
    graphics test has to build on, but they were written for interactive use: the image comes
    back at full resolution, nothing returns a **summary** of the graphics data without
    rendering, and the raytracer path and the GLFW path differ in what they update. RG2.3 needs
    a documented headless call that updates the graphics data and returns counts, and an image
    call that takes a resolution.

<a id="rg6-4"></a>
**RG6.4** **DONE 2026-09-23** (#2609) — [log](exudynRevisionLog2026b.md#rg6-4) —
    **The light and shadow descriptions say things that are no longer true.** All three faults
    are fixed: the members of a light read *"of this light"* and the mapping to
    `GL_LIGHT0`-`GL_LIGHT3` is said once at `enable`; the claim that `light0` is the light with
    shadows is gone, and `shadow` says that every light casts one; the directional-light
    approximation moved from `position` to `shadow` and names **no factor**. Two deprecated
    setting names in the hand-written manual went with them.

<a id="rg6-5"></a>
**RG6.5** **DONE 2026-09-24** (#2633) — [log](exudynRevisionLog2026b.md#rg6-5) —
    **Restoring the saved render state took two lines in 82 places** *(maintainer,
    2026-09-24)*. `SC.renderer.Stop()` saves the state of every open view in `exudyn.sys`,
    and every model that wanted the previous view back repeated the same `if 'renderState'
    in exu.sys:` guard. **`SC.renderer.RestoreSavedState()`** does it and returns `False`
    when nothing has been saved - the first run of a script, which is what the guard was for.
    82 occurrences in 85 files, in five variants of which four still used the deprecated
    `SC.SetRenderState`, are one call each.

<a id="rg6-6"></a>
**RG6.6** **DONE 2026-09-24** (#2643) — [log](exudynRevisionLog2026b.md#rg6-6) —
    **macOS: the settings dialog aborted the process** *(maintainer, 2026-09-24, first
    graphics test on macOS)*. The single-threaded renderer - which macOS always is - polls
    events and runs the queued Python inside `DoIdleTasks()`, and the settings dialog calls
    `DoIdleTasks(0)` on every change. A second event pump inside the first is fatal there,
    because `glfwPollEvents()` runs the **shared** Cocoa run loop, which redraws the tkinter
    dialog and calls back into Python. `GlfwRenderer::idleOperationDepth` counts the idle
    operations on the stack and only the outermost pumps; a nested one renders and returns.

<a id="rg6-7"></a>
**RG6.7** *(group RG6; maintainer 2026-09-27)* **GraphicsData gets a Sphere and a
    CurvedTriangleList** (#2709). Bigger than it sounds, because every consumer of the graphics data
    has to follow - even the minimal implementation with temporary workarounds: the GraphicsData
    classes and their dictionary, the OpenGL renderer, the raytracer, the pybind interfaces,
    `SC.renderer.GetGraphicsData()`, the documentation, and the graphics regression test (RG2.3.3).

    **A limitation to resolve with it**: spheres are already special. The OpenGL renderer treats the
    spheres of nodes separately, because there can be very many of them; the **raytracer does not draw
    `glSpheres` at all**; `GetGraphicsData()` does return them (measured 2026-09-27). A Sphere that is
    fully part of GraphicsData has to be drawn the same way by all three.

    - **RG6.7.1** *(the preliminary sub-step)* **what the sphere can do, and what the curved triangle
      is** (#2710). The geometry is the decision that matters: ideally a curved element that is smooth
      with continuous tangents **not only at its nodes but along its boundaries**, so that a curved
      surface made of many of them has no visible edges. Quads are acceptable if they are better and
      also work degenerated to a triangle. How many and which nodes the element has belongs to the
      decision. The deliverable is a short comparison of candidates for the maintainer, each with what
      it costs in the OpenGL renderer, the raytracer and `GetGraphicsData()`.

<a id="rg6-8"></a>
**RG6.8** *(group RG6; maintainer 2026-09-29)* **The graphics fixes before 1.13** - *"many are graphics
    related; still, some may be solvable or you could suggest a simple test"*. With the test each can
    have:
    - **RG6.8.1** (#1813) marker positions in `AnimateModes` with deformation scaling 0 - **headless**:
      the marker positions in `SC.renderer.GetGraphicsData()` against the reference positions;
    - **RG6.8.2** (#2309) `ZoomAll` ignores a `trackMarker` - **headless**: the render state after
      `ZoomAll` with a moving tracked marker, the marker in the view;
    - **RG6.8.3** (#2321) meshes from NGsolve give triangles of the wrong orientation - with ngsolve
      (optional package): the normals of `fem.GetSurfaceTriangles()` against the outward normals;
    - **RG6.8.4** (#2308) erratic shadows with `modelCentricView=False` and lights in the camera frame -
      a small raytracer image against a reference, as in RG2.3.3.4;
    - **RG6.8.5** (#2140, #2236) Linux: crashes when the renderer closes and with the SolutionViewer; the
      time in the renderer initialized wrong - the manual check (RG2.4) S7, Q1, Q2 on Ubuntu, plus a
      script that starts and stops the renderer twenty times;
    - **RG6.8.6** (#2237, #2350) macOS: PlotSensor in Spyder; `raytracerNOGLFWtest.py`, excluded on macOS
      since 1.11.0 because offscreen `RedrawAndGetImage` crashes - when the macOS machine is there
      (around 2026-10-20), the manual check P1 in Spyder and the test model without its exclusion.

## RG7 — Python user items

Items whose behaviour is written in Python. Today that means user functions on existing items -
`ObjectGenericODE2` with a user-supplied right-hand side, user loads, user sensors - which is
powerful and has a cost: every evaluation crosses the C++/Python boundary.

The subject of this group is how far that can go: what a **whole item defined in Python** would
look like, with the same validated dict interface as a built-in one, and where the line runs
against RG8, which solves the same problem by compiling. The two are alternatives for the same
user, and deciding which to recommend for which case belongs here.

*No steps yet.*


## RG8 — Compiled C++ user items

*(revision2026 phase R9.)* Additive; nothing earlier changes. Two shipped variants remain after
revision2026 step R2.10, so a plugin is bound to the one it was built against (`use_AVX2` changes
`exuMemoryAlignment`); RG8.2 turns that from silent corruption into a refusal.

**Founding policy**: a plugin author builds exudyn from source once, then builds the plugin with
the same toolchain. Compiler, CRT and flags are then identical by construction, so no C-ABI shim
is needed - plugins inherit from `CObject` directly.

<a id="rg8-1"></a>
**RG8.1** *(revision2026 step R9.1)* Make the registry cross-binary. `MainObjectFactory.h:97` holds its singleton in a
    function-local static inside a header-only class template, so every binary gets a private
    copy. Move the storage into one exported accessor in a single translation unit. The dispatch
    path needs no change.

<a id="rg8-2"></a>
**RG8.2** *(revision2026 step R9.2)* Define an ABI fingerprint checked at registration: exudyn version, active macro set, compiler
    id and version, `__cplusplus`, and on MSVC `_ITERATOR_DEBUG_LEVEL` plus the CRT model. Add
    `sizeof` canaries for `Vector`, `CObject`, `std::string`, `std::function`. Refuse with a
    message naming the mismatch. **The Debug/Release CRT case matters most** — the registry holds
    `std::map<std::string, std::function<...>>`, and debugging a new object in VS is exactly what
    a plugin author will do.

<a id="rg8-3"></a>
**RG8.3** *(revision2026 step R9.3)* Commit a reference plugin subdirectory built on every commit: minimum compile, a trivial
    registered object, a handshake assertion, and a test that adds it to a system and solves.
    **This is the synchronisation mechanism** — interface drift breaks your build, not a user's.

<a id="rg8-4"></a>
**RG8.4** *(revision2026 step R9.4)* Ship the plugin headers in the wheel; add `exudyn.get_include()` (numpy/pybind11 convention).
    Headers only: `Linalg/`, `Utilities/`, item base classes, plugin interface header.

<a id="rg8-5"></a>
**RG8.5** *(revision2026 step R9.5)* Change duplicate handling in `RegisterClass` from `CHECKandTHROWstring` to a collected,
    reported error. Otherwise two third parties choosing the same name abort `import exudyn`.

<a id="rg8-6"></a>
**RG8.6** *(revision2026 step R9.6)* Discovery at import: `~/.exudyn/plugins/` plus `EXUDYN_PLUGIN_PATH`, and
    `importlib.metadata` entry points for pip-installed plugins. Read a manifest *before* loading
    the library. Load each in isolation; a broken plugin must never break `import exudyn`. Never
    put the plugin directory inside the installed package.

<a id="rg8-7"></a>
**RG8.7** *(revision2026 step R9.7)* Record loaded plugins and versions in the solver log and solution file header. A script
    yielding different results on a colleague's machine because of a library in their home
    directory is a reproducibility hazard.

<a id="rg8-8"></a>
**RG8.8** *(revision2026 step R9.8)* Extend the revision2026 step R4.3 emitter to scaffold a plugin from a user's definition file — C++ skeleton,
    Python dict-building class, `.pyi` stub — emitted into the user's package.

<a id="rg8-9"></a>
**RG8.9** *(revision2026 step R9.9)* Document three constraints: plugins are never unloaded or reloaded (a rebuilt plugin needs a
    kernel restart in Spyder/Jupyter); plugin authors build from source; the Python
    dict-builder class stays an explicit `from myplugin import ObjectMyThing` rather than being
    injected into `exudyn.itemInterface`, so every script says where its item types came from.


## RG9 — Structural core improvements

The architecture of the core, where one change touches every item. Nothing here is a weekend: the
triage of revision2026 step R8.5.2 classified seven open issues as HUGE (above 40 hours of human
work) and they are this group's backlog -

- **#354** automatic differentiation for objects,
- **#1104** the fully generic object,
- **#1722** `exudyn.Parameter` for every item parameter,
- **#88** the data-dependency architecture,
- **#1821** a kinematics solver and **#1822** an inverse-dynamics solver,
- **#1956** a representative figure for each of the 109 items (the one that is documentation, and
  is really RG3).

A step appears here when one of them has been thought through far enough to be planned. This is
also where **2.0** comes from: the major number is reserved for this work, not for the 2026
revision (info document D15).


<a id="rg9-1"></a>
**RG9.1** **DONE 2026-09-23** (#2622) — [log](exudynRevisionLog2026b.md#rg9-1) —
    **The item sources stop paying for pybind11.** **52 of 52** sources in `src/ImplObjects/`
    reached pybind11 before, **19** after — and those 19 for a reason of their own, a user
    function, a `PyMatrixContainer`, a numpy array or `ExceptionsTemplates.h`, not through the
    graphics headers. **The build time did not change**: 57.1 s before, 58.0 s after, on a clean
    build of the same machine, so the gain is in the structure and not in the clock.

<a id="rg9-2"></a>
**RG9.2** **DONE 2026-09-23** (#2628) — [log](exudynRevisionLog2026b.md#rg9-2) —
    **Fourteen item sources included an exception header they do not use, and paid pybind11
    for it.** `src/Utilities/ExceptionsTemplates.h` was included by 17 sources in
    `src/ImplObjects/` and used by one; it includes pybind11, which was the only route to it
    for eight of them. Item sources reaching pybind11: **19 to 11**, and the build time again
    did not move. The build then found what the include had hidden - 24 generated headers
    free-riding on it for the `namespace py` alias - and a second defect (#2629):
    `itemHeaderEmitter.py` wrote generated headers in the **locale** encoding.

<a id="rg9-3"></a>
**RG9.3** *(group RG9; maintainer 2026-09-29)* **Access functions as single functions of the objects**
    (#2744). `GetAccessFunctionBody(AccessFunctionType, localPosition, Matrix& value)` serves every access
    type through one function and a switch, and carries workarounds - the vector of
    `JacobianTtimesVector_q` travels in the output matrix, `OwnMarkersOnly` (RG4.10) says what a
    declaration cannot. Single functions per access type, with interfaces that say what they take and
    return, avoid them.
    - **RG9.3.1** **DONE 2026-09-29, for the maintainer's decision** — the evaluation, in
      `tmp/evalRG9_3_accessFunctions.md` (not kept in the repository): which objects provide which
      access functions today, which markers and loads call them, and what the best interface is for each.
      Proposed: one virtual function per access type, and the flags derived from the functions a
      definition declares (with RG9.3.2); before RG14;
    - **RG9.3.2** a check that the access function flags an object declares (`ItemAccessFunctionTypes`)
      and the functions its definition declares agree - possibly by deriving the flags from the
      functions;
    - **RG9.3.3** the migration, object by object.

## RG10 — Tooling and process

The machinery a maintainer uses: `exudev` (revision2026 step R5.18), the issue tracker and its
JSON store (revision2026 steps R8.3 to R8.5), the generators (revision2026 step R4.3), the checks of the commit gate, and the CI. It
works; this group carries what it still lacks.

Open in the tracker for this group: **#2541** (`exudyn.config` and `exudyn.special` are in no stub
file, so an editor cannot complete them).

<a id="rg10-1"></a>
**RG10.1** *(group RG10; maintainer request 2026-09-15; revision2026 step R8.6)* **DONE 2026-09-27**
    (#2712) — [log](exudynRevisionLog2026b.md#rg10-1) — `exudev scripts <folder>`, a maintainer tool
    for now, as the maintainer decided for teaching; whether it later ships in the package is open.
    **Checker for user scripts after the 1.12 API changes.** Teaching folders and user projects hold Exudyn scripts written
    against 1.x. A static checker (parses, never runs) reports per file and line: names the script
    uses but no longer gets from a star import (`np`, `sin`, `graphics`, ...; revision2026 step R4.22.3), removed
    names with their replacement (revision2026 step R4.22.1, revision2026 step R4.22.2), and submodules used without their import, with the
    import line to add. The name lists come from the modules' `__all__` and from the table
    [API changes for the 1.12 release notes](exudynRevisionInfo2026.md#api-changes-v2), which is
    complete since the revision closed. Decide then whether it ships in the
    package (users run it) or stays in `tools/`. It is the mechanical half of the
    {ref}`revisions chapter <sec-revisions>`: the chapter tells a user what to do, the checker
    finds the places. Worth having before the 1.13 release (RG1.4), which is when users meet
    the changes.

    - **RG10.1.1** *(maintainer 2026-09-27)* **running the scripts too** (#2713): copy them into a
      local space and execute them as the examples are run, with a timeout. A script is checked
      first for paths that do not travel - absolute ones such as `C:\`, relative ones such as `../`
      or `..\` - because a copied script with such a path reads or writes elsewhere, or fails for a
      reason that is not the Exudyn version.
    - **RG10.1.2** **DONE 2026-09-27** — [log](exudynRevisionLog2026b.md#rg10-1-2) - **the
      repository's own scripts** (#2714): the checker reports 54 findings in 17 of
      the 342 examples, test models and mini examples. **Four are real breaks** in scripts that no
      suite runs - `NGsolveGeometry.py`, `humanRobotInteraction.py` and `stlFileImport.py` call
      `AddEdgesAndSmoothenNormals` without `graphics.`, `nMassOscillatorEigenmodes.py` uses
      `graphics` without importing it - and the rest are deprecated forms that still work
      (`exu.StartRenderer`, `general.drawWorldBasis`, `exu.SolveDynamic`, ...). The examples are
      what users copy.


<a id="rg10-2"></a>
**RG10.2** **DONE 2026-09-23** (#2600) — [log](exudynRevisionLog2026b.md#rg10-2) —
    **The issue table of `exudev issue serve` did not say what its columns are, and left out
    the priority** *(maintainer, 2026-09-23)*. It has a header row with the meaning of the
    effort values within reach, it shows the priority, and the effort tag carries its word -
    `LOW EFF` - because `LOW` and `HIGH` are values of both fields and two bare tags in one
    row cannot be told apart. That rule is what RG3.10.1 later took for both pages.

<a id="rg10-2-1"></a>
**RG10.2.1** **DONE 2026-09-24** (#2636) — [log](exudynRevisionLog2026b.md#rg10-2-1) —
    **The search of `exudev issue serve` missed most fields, and the list stopped at 400**
    *(maintainer, 2026-09-24)*. The text search read four fields, so neither author could be
    searched for, and the listing sent the newest 400 of 2,637 rows with no paging, which put
    everything older than about #2240 out of reach. The search reads every field; the listing sends
    what matched.

<a id="rg10-2-2"></a>
**RG10.2.2** **DONE 2026-09-24** (#2641) — [log](exudynRevisionLog2026b.md#rg10-2-2) —
    **A search for digits found every field except the issue number** *(maintainer,
    2026-09-24)*: `249` listed the issues that name it in their text and two whose
    `resolvedInVersion` is `0.1.249`, and missed **#2497**, which is what it was typed for.
    RG10.2.1 had left the number as a separate exact test. It is a substring like every other
    field now, so `249` finds #249 and #2490 to #2499.

<a id="rg10-3"></a>
**RG10.3** **DONE 2026-09-23** (#2617) — [log](exudynRevisionLog2026b.md#rg10-3) — *(group
    RG10; maintainer 2026-09-23)* **`exudev` does not say how long a step took.** The batch
    scripts it replaced printed the build time, and it is read: a build that suddenly takes twice
    as long is the first sign that a header dependency grew. Every step is timed and the summary
    prints it, with the total.

<a id="rg10-4"></a>
**RG10.4** **DONE 2026-09-23** (#2618) — [log](exudynRevisionLog2026b.md#rg10-4) — *(group
    RG10; maintainer 2026-09-23)* **`src/pythonGenerator/` holds one file and should not exist.**
    Everything of the old generator directory moved to `tools/generators/` in revision2026 step
    R4.3 except `exudynVersion.py`, which locates the repository root and reads `version.txt`. It
    moves to `tools/generators/`, where the other build-time helpers live and are already in
    `MANIFEST.in`, and the directory is deleted.

<a id="rg10-5"></a>
**RG10.5** **DONE 2026-09-23** (#2619) — [log](exudynRevisionLog2026b.md#rg10-5) — *(group
    RG10; maintainer 2026-09-23)* **VS Code cannot follow a C++ include.** *"include errors
    detected - update your include paths"*, and *"cannot open source file ../Eigen/Sparse"*: the
    vendored headers are reached through subdirectories of `include/`, and nothing tells the
    C/C++ extension about them. `.vscode/` is git-ignored, so the fix is a committed template
    that `tools/setupLocalWorkspace.py` copies, as for `exudyn.sln` and `python/pytest.py`.

<a id="rg10-6"></a>
**RG10.6** **DONE 2026-09-24** (#2632) — [log](exudynRevisionLog2026b.md#rg10-6) —
    **The TestModels imported the test suite to find out whether they are being tested.**
    Nine lines in every model became one, `testIsActive = exu.sys.get('testIsActive',
    False)`, and with them went the second way of running a test. The sub-steps, each with
    its own log entry:

    - **RG10.6.1** — [log](exudynRevisionLog2026b.md#rg10-6-1) — the channel: the runners write
      `exu.sys['testIsActive']`, read `exu.sys['testResult']` and honour
      `exu.sys['testTolerance']`, in the in-process runner, in the parallel worker and for
      the mini examples.
    - **RG10.6.2** — one model, `bricardMechanism.py`, with an identical number.
    - **RG10.6.3** — [log](exudynRevisionLog2026b.md#rg10-6-3) — all 129 models, the 98
      hard-coded `testError = result - <number>` lines gone, every one of the 139 results
      identical to the run before the sweep.
    - **RG10.6.4** — the 24 mini examples, which are generated, so the change is in
      `tools/generators/miniExampleEmitter.py` and in the `miniExample` bodies.
    - **RG10.6.5** — [log](exudynRevisionLog2026b.md#rg10-6-5) — `modelUnitTests.py` and
      `runUnitTests.py` are deleted; their ten test functions are test models.
    - **RG10.6.6** — [log](exudynRevisionLog2026b.md#rg10-6-6) — the documentation, which was a
      gap and not a correction: `docs/dev/WORKFLOW.md` says what a test model looks like.
    - **RG10.6.7** — [log](exudynRevisionLog2026b.md#rg10-6-7) — the one hidden tolerance:
      `kinematicTreeAndMBStest.py` states `exu.sys['testTolerance']` instead of multiplying
      its result by 1e-7. The only reference solution that moved.
    - **RG10.6.8** — [log](exudynRevisionLog2026b.md#rg10-6-8) — the seven performance models,
      `AddTiming` into `exu.sys['testTimings']`, and `ExudynTestStructure` deleted.

<a id="rg10-7"></a>
**RG10.7** **DONE 2026-09-24** (#2638) — [log](exudynRevisionLog2026b.md#rg10-7) —
    **The plan carried the full text of the steps that are finished** *(maintainer,
    2026-09-24)*: 971 of its 1391 lines, against its own rule that a done step keeps status,
    date, outcome and a link. A step that ended in *"the original text follows"* is cut there -
    that text is the issue as it was raised, and the tracker has it - and the rest were
    rewritten to the outcome. 1391 lines to 939, with no anchor, step number or group heading
    lost. The review of `GUI.py` was the one piece of analysis that lived only here and is now
    a [log entry](exudynRevisionLog2026b.md#rg6-2-review).

<a id="rg10-7-1"></a>
**RG10.7.1** **DONE 2026-09-24** (#2642) — [log](exudynRevisionLog2026b.md#rg10-7-1) —
    **The plan did not say what to do next** *(maintainer, 2026-09-24)*. The open steps are
    spread over twelve groups and were read by scrolling. The last section, **Next steps
    recommended**, names them with their issue and a short title, lists what the current work
    raised without making it a step, and recommends an order with the reason for it. It copies
    nothing and is updated from time to time.

<a id="rg10-8"></a>
**RG10.8** **DONE 2026-09-24** (#2644) — [log](exudynRevisionLog2026b.md#rg10-8) —
    **`exudev` is needed on linux and macOS too** *(maintainer, 2026-09-24)*. Measured by
    planning every command under WSL: most of it was already portable, and **three** places were
    not - `clean` matched only the Windows build directories, `docs --open` called `xdg-open`,
    which macOS does not have, and `linux` drove the manylinux container through `wsl -e`, which
    on linux there is nothing to go through. All three are the platform's own now, the macOS
    case of the manylinux image is refused with a reason, and `test_exudev.py` pins the three
    from any platform.

<a id="rg10-9"></a>
**RG10.9** **DONE 2026-09-24** (#2647) — [log](exudynRevisionLog2026b.md#rg10-9) —
    **`regenerated_files` failed on linux and could not fail on Windows** *(maintainer supplied
    the GitLab log of 1.12.61)*. The generator wrote `pybind_modules.h` while the repository has
    `Pybind_modules.h`: one file on Windows, two on linux, where the real header was **never
    regenerated**. The name is one spelling now, a declared output is checked **case-exactly** on
    every platform, and a generator may create a file it declares instead of printing
    *"illegal file"* and writing nothing.


<a id="rg10-10"></a>
**RG10.10** **DONE 2026-09-25** (#2653) — [log](exudynRevisionLog2026b.md#rg10-10) —
    **The header of `definitionLoader.py` read like the file was dead** *(maintainer question,
    2026-09-25)*. It is live - three generators stop working without it - and what made it look
    dead was a header that opened with what the old parser produced and ended with two promises
    about its own removal. It says what it is and who uses it. `itemModel.LegacyItems()`, found
    while checking, really was dead and is gone.

<a id="rg10-11"></a>
**RG10.11** *(group RG10; from #2541; numbered RG10.2 by mistake until 2026-09-27, when that number
    was already taken)* **DONE 2026-09-28** — [log](exudynRevisionLog2026b.md#rg10-11) —
    **`exudyn.config` and `exudyn.special` reach a stub file.**
    `exudyn.config` is the run-time settings object - `outputDirectory`, `printToConsole`,
    `suppressWarnings`, `precision` - and `exudyn.special` holds the rarely needed corners. Neither
    the objects nor their C++ classes appear in `python/exudyn/__init__.pyi`, so no editor completes
    `exudyn.config.outputDirectory` and no checker knows it exists. The stub is generated
    (`tools/checkPython.py --stubs`), so this is a question of what the generator is told about the
    two members rather than of writing a stub by hand.

<a id="rg10-12"></a>
**RG10.12** *(group RG10; maintainer 2026-09-29)* **DONE 2026-09-29** — [log](exudynRevisionLog2026b.md#rg10-12) —
    **The GitLab job `check_docstrings` passes, and the gates run pydoclint** (#2747): four docstrings
    fixed, the deliberate `DOC108` of the typed user function parameters in the baseline, and
    `pydoclint` a stage of `exudev generate --all-checks`.

## RG11 — Misc

What belongs to no group yet. Three of a kind here are a reason to propose a group of their own.

<a id="rg11-1"></a>
**RG11.1** **DONE 2026-09-24** (#2610) — [log](exudynRevisionLog2026b.md#rg11-1) —
    **The results monitor runs beside the simulation, or it is redundant.** Evaluated, and the
    recommendation is a **second process**: the file is already the protocol, the command line
    already exists, and it is a handful of lines around
    `subprocess.Popen([sys.executable, '-m', 'exudyn', 'monitor', ...])`. One premise of the step
    was wrong and is corrected below — `PlotSensor` cannot follow a growing file, so the
    in-script call is **not** redundant today. Building it is **RG11.3**, proposed and not
    created.

<a id="rg11-3"></a>
**RG11.3** **DONE 2026-09-26** (#2670) — [log](exudynRevisionLog2026b.md#rg11-3) — **The results monitor beside
    a running simulation.** RG11.1 evaluated the four ways and recommends a **second process**:
    the solution file is already the protocol, `python -m exudyn monitor` already exists, nothing
    is shared so no backend, GIL or thread-safety question arises, and it is a handful of lines
    around `subprocess.Popen([sys.executable, '-m', 'exudyn', 'monitor', fileName, ...])` that
    returns the handle. `MonitorResults` stays as it is for the case where blocking is wanted.
    The two open questions are **answered**: the child is **left running**, because the point of a
    monitor on a short simulation is that the plot is still there when it ends, and the returned
    `subprocess.Popen` is the handle for a script that wants it gone; and `SolutionViewer` is **not**
    served by the same call - it needs the renderer and the system, not a file, so it has nothing to
    gain from a second process that can only read what was written.

    - **RG11.3.1** **DONE 2026-09-27** (#2672) — [log](exudynRevisionLog2026b.md#rg11-3-1) - **the
      monitor waits for the file.** `WaitForData` waits for the file, the header and the first row;
      the caller's existence test, which said *"file not found"* before any waiting could begin, now
      applies only to `--once`, which plots what exists and returns. `waitTimeout` keeps its meaning
      and gains the file: 0 waits without limit, N gives up after N seconds - and what is waited for
      is announced, naming the file, because waiting forever for a file that never appears is what a
      typo looks like. Reported twice, three days apart.

      The fix is to move the existence test into the waiting loop, so that `--wait 0` waits for the
      file to **appear** and not only for its first row. What `--wait N` should mean for a file that
      never appears is the one decision: the same timeout, or a separate one, because waiting for a
      file that a typo made impossible is a different mistake from waiting for a slow solver. Until
      it is done, the docstring of `StartResultsMonitor` and `docs/manual/resultsMonitor.md` promise
      something that is not true; that is recorded in the issue rather than by weakening the text,
      because the text says what the function is **for**.

<a id="rg11-2"></a>
**RG11.2** **DONE 2026-09-23** (#2620) — [log](exudynRevisionLog2026b.md#rg11-2) —
    **The demos wrote a `solution/` directory into whatever directory they were started in.**
    `python -m exudyn demo 2` created one beside the sources of this repository - untracked,
    unignored, and nearly committed by accident. They write to `tmp/solution/` now, and
    `solution/` is in `.gitignore` so that an older installed version cannot leave one.

## RG12 — Python interface

*(Group proposed by the maintainer, 2026-09-22.)* The shape of the Python API itself, as opposed
to what it computes: how a parameter is named, what happens when a name changes, what a user can
find out about the settings of a model. It is the group a user notices most and reads least about.

<a id="rg12-1"></a>
**RG12.1** *(group RG12; maintainer 2026-09-22)* **`simulationSettings` gets the deprecation
    mechanism** (#2588). `visualizationSettings` has it: a member is marked `Deprecated(since,
    expires)` in the definitions - 93 members carry it today - and a user who sets the old name
    is told the new one instead of being ignored. `simulationSettings` uses none of it, although
    it is the same generator and the same structure machinery, so a renamed solver setting
    simply disappears.

<a id="rg12-2"></a>
**RG12.2** *(group RG12; maintainer 2026-09-22)* **Item parameters can be deprecated** (#2589).
    The case that actually hurts: an item parameter is renamed and every script that used the
    old name stops working, with no message that says what to write instead. Two levels are
    possible - the generated classes of `itemInterface.py`, which is one place and covers what a
    script writes, or the `Get`/`Set` functions of the items themselves, which also covers
    `mbs.GetObjectParameter`. If it reaches the C++ side, **the deprecated names are searched
    last**, so that the common case pays nothing.

<a id="rg12-3"></a>
**RG12.3** **DONE 2026-09-24** (#2590) — [log](exudynRevisionLog2026b.md#rg12-3) —
    **What did this model actually change?** `ChangedSettings`, `ChangedSettingsCode` and
    `PrintChangedSettings` answer it, for `visualizationSettings` and `simulationSettings` alike,
    in the new **`exudyn.misc.settingsUtilities`** — which is the window-free half of
    `exudyn.misc.GUI`, moved out because that module imports tkinter at module scope and a model
    script therefore could not use any of it. A test starts a fresh interpreter and requires that
    importing the new module pulls in **no tkinter**. The solution file needs no C++ change:
    `solutionSettings.solutionInformation` is written into its header and takes the block as it
    is.

Open in the tracker for this group: **#2497** (59 bare `except:` remain in the shipped
package).

<a id="rg12-4"></a>
**RG12.4** *(group RG12; maintainer 2026-09-25)* **DONE 2026-09-26 except RG12.4.7** (#2664, resolved) —
    **A user function is one typed Python function, and
    everything else is generated from it** (#2664). **After RG3.14** - the descriptions have to be
    Markdown first, because this step makes the documentation block an output rather than a text.

    Traced for `ObjectGenericODE2.forceUserFunction` on 2026-09-25, the same signature is stated in
    **five** places and checked in none:

    | where | what it says |
    |---|---|
    | `definitions/itemDefsObjects.py`, the `ItemParameter` | `type=TPyFunctionVectorMbsScalarIndex2Vector` |
    | `definitions/definitionTypes.py` | that type as `std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector)>` |
    | `definitions/itemDefsObjects.py`, the prose | `forceUserFunction(mbs, t, itemNumber, q, q_t)` and an argument table with its own type column |
    | `python/exudyn/itemInterface.py`, `userFunctionArgsDict` | `[[types], ['mbs','arg0','arg1','arg2','arg3'], ['StdVector']]` |
    | `src/System/evaluateUserFunctions.cpp` | the call, by hand |

    **The registry already exists and is half filled in**: `userFunctionArgsDict` is generated, is
    shipped, and `advancedUtilities.py` builds the symbolic function interface out of it - with
    `arg0` for `t` and `StdVector` for a numpy array. That is the thing to fix, and the rest follows
    from it.

    **The source is an ordinary Python function**, not a string - the maintainer, 2026-09-25:
    *"I wanted it ... not to be given in a string, but defined in the Python code ... because this
    avoids problems in the definition itself and immediately becomes Python"*. It stands in the
    definition file immediately above the `definitions.append(...)` it belongs to, and is passed to
    its parameter by object:

    ```python
    def ObjectGenericODE2_forceUserFunction(mbs: MainSystem, t: Real, itemNumber: Index,
                                           q: np.ndarray, q_t: np.ndarray) -> np.ndarray:
        r"""compute the generalized user force vector for the ODE2 equations

        Args:
            t: current time
            q: generalized coordinates, $\qv \in \Rcal^{n_{ODE2}}$
        Returns:
            the force vector, $\fv_{user} \in \Rcal^{n_{ODE2}}$
        """

    ... ItemParameter(..., pythonName='forceUserFunction',
                      userFunction=ObjectGenericODE2_forceUserFunction)
    ```

    The `def` is named `<Item>_<parameter>` because one definition file holds 35 of them and four
    are called `forceUserFunction`; **the name the documentation prints is the parameter's**
    **`pythonName`**, so a page still reads `forceUserFunction(mbs, t, itemNumber, q, q_t)`. The
    annotation types - `Real`, `Index`, `MainSystem`, `np.ndarray` and five more - are ordinary
    Python names in `definitions/definitionTypes.py`, so a definition file stays importable and
    readable in an editor. Nothing is executed: the function object is used only to find its source,
    which is read with `ast`, so an annotation is reported **as it is written**.

    Note what moved: the **size of an argument is a formula in its description**, not part of its
    type. That is the maintainer's own correction of the idea and it is what makes the whole thing
    possible - RG3.14.5 could not put the arguments into a code block because 35 of the 228 rows said
    the size as a formula, and a formula does not render inside one. Typed as `np.ndarray` and
    described as $\qv \in \Rcal^{n_{ODE2}}$, both halves are in the right place.

    - **RG12.4.1** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg12-4-1) - the vocabulary: the annotation types and `userFunction=` in `definitions/definitionTypes.py`, and the
      reader that turns one into names, Python types, the docstring and the `Args:`/`Returns:` lines.
      `ast` only: the block is parsed, never run, so a type may be a name that does not exist at
      generation time.
    - **RG12.4.2** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg12-4-2) - one function,
      end to end: `ObjectGround.graphicsDataUserFunction`. Its block is generated from the def,
      `userFunctionArgsDict` carries the real argument names, and the page is unchanged apart
      from one blank line that was inside the example's code fence. The trial item is not the
      `ObjectGenericODE2.forceUserFunction` this step first named: that item has four blocks and
      one shared example, which is a reordering question and not a mechanism question.
    - **RG12.4.3** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg12-4-3) - the check:
      the number of arguments, each argument's type, the return type and that the docstring
      describes every argument, all against the `std::function` that
      `definitionTypes.userFunctionSignatures` maps the parameter's type to. A size is not
      compared, because a size is not in the type. `tools/checkDefinitions.py` reports a finding
      with the file and the line, and the generator refuses to emit.
    - **RG12.4.4** **DONE 2026-09-25** — [log](exudynRevisionLog2026b.md#rg12-4-4) - a `Protocol`
      per user function in `itemInterface.py`, generated from the def, **and** the parameter of
      the item class annotated with it - `Union[ObjectGroundGraphicsDataUserFunction, int]`,
      because 0 is the value that means no user function. Without the annotation an editor has
      nothing to complete at the place where a user writes the function.
    - **RG12.4.5** **DONE 2026-09-26** — the remaining signatures, in the order of the item files; **23 distinct
      signatures under 17 names in 35 blocks**, so two thirds of the work is naming arguments that
      are already written down in the prose. A block is converted by a script that refuses what it
      does not recognise; what it refuses is done by hand.
        - **RG12.4.5.1** **DONE 2026-09-26** — [log](exudynRevisionLog2026b.md#rg12-4-5-1) - the
          vocabulary of sizes (`Vector6D`, `Matrix3D`, `Array`, and a size that is not fixed as the
          leading `$\in ...$` of the argument's description), and the five items of
          `itemDefsLoads.py` and `itemDefsSensors.py`: six user functions of 23.
        - **RG12.4.5.2** **DONE 2026-09-26** — [log](exudynRevisionLog2026b.md#rg12-4-5-2) - the
          connectors of `itemDefsObjects.py`: eight items, ten user functions, of which two
          documented arguments that do not exist and one was written over two lines.
          `ObjectConnectorRigidBodySpringDamper` is **not** among them - its second block does not
          list its arguments at all - and goes with RG12.4.5.3.
        - **RG12.4.5.3** **DONE 2026-09-26** — [log](exudynRevisionLog2026b.md#rg12-4-5-3) - the
          bodies, the joint and the rigid body spring damper: **all 34 user functions are defs**.
        - **RG12.4.5.4** **DONE 2026-09-26** - the gate: `tools/checkDefinitions.py` reports a
          parameter of a `PyFunction...` type that carries no def, so a new user function cannot be
          written as prose again.
    - **RG12.4.6** **DONE 2026-09-26** — [log](exudynRevisionLog2026b.md#rg12-4-6) - what became
      redundant: the argument table is generated for every user function, the `\_` escapes are
      gone with RG3.14, and `advancedUtilities`' hand-built `F(...)` message now also names the
      generated `Protocol`, which is carried in `userFunctionArgsDict` as a fourth entry.
    - **RG12.4.7** *(maintainer, 2026-09-25)* — **the `TPyFunction...` group type disappears from a
      definition**. *"Can the types like `TPyFunctionMbsScalarIndexScalar5` then also be eliminated?
      They are common interfaces also for the C++-side, but if the automatic mechanisms allow to also
      generate the appropriate function signatures ... then it would be removed. One could just define
      every different user function in `PySymbolicUserFunctionSet.h`, instead of having the current
      groups."*

      **It can, and the def is the source that makes it possible.** Measured 2026-09-25: 24 group
      names in `definitionTypes.userFunctionSignatures`, named after their signature
      (`MbsScalarIndexScalar5` = `Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real)`), used
      **53 times** in `definitions/` - four items share `MbsScalarIndexScalar5`, four share
      `VectorMbsScalarIndex2Vector`, and nine names are used once. The group is a **lookup key, not a
      C++ type**: the generated item header declares the member as the `std::function<...>` text, so
      nothing in C++ ever names the group except `PySymbolicUserFunctionSet.h`, which declares one
      member per group and dispatches an if-chain of (item, user function) onto it.

      The def already states the whole signature - once RG12.4.5 gives every argument a **sized**
      annotation (`Vector6D`, `Matrix3D`, `Vector`, `ArrayIndex`), `cppToAnnotation` is
      one-to-one and the `std::function<...>` is **derivable from the annotations**. Then `type=` says
      nothing the def does not.

        - **RG12.4.7.1** — **the proof, changing no output**: derive the `std::function<...>` from
          each def's annotations and compare it with
          `definitionTypes.userFunctionSignatures[type]`. Every converted user function must agree,
          and the comparison joins `CheckAgainstCpp`, which today reads the mapping in one direction
          only. Nothing is removed while the two disagree anywhere, and a disagreement is a finding
          with a file and a line, not a broken build.
        - **RG12.4.7.2** — **the removal**: `type=` goes from a user function parameter, the emitters
          take the signature from the def, and the group name becomes an emitted detail. Where the
          generated C++ wants a name it is `<Item><Parameter>` - the name the `Protocol` already has -
          so `PySymbolicUserFunctionSet.h` can declare one member per user function instead of one per
          group, which is what the maintainer asked for. Whether it *should* is a separate question:
          identical signatures collapsing into one member is deduplication, and deduplication of a
          generated file is cheap to keep and cheap to drop.

      **The restriction the maintainer names is real and it is in the symbolic set.** A symbolic user
      function is evaluated through `EvaluateBool`, `EvaluateReal`, `EvaluateStdVector`,
      `EvaluateStdVector2D`, `EvaluateStdVector3D` and `EvaluateStdVector6D` - **six** return shapes,
      and `PySymbolicUserFunctionSet.h` dispatches **15** (item, user function) pairs of the 35.
      There is no `Evaluate` for `py::object` (graphics data, `MatrixContainer`), for `NumpyMatrix`,
      or for `StdArrayIndex`/`ConfigurationType` arguments. So one member per user function does not
      make every user function symbolic: a member can only bind to an `Evaluate` that exists. The step
      therefore makes the restriction **stated** - the generator says which user functions have no
      symbolic path, in one place - instead of leaving it to be discovered at the call.


<a id="rg12-5"></a>
**RG12.5** *(group RG12; maintainer 2026-09-26)* **DONE 2026-09-28** (#2666) — **User settings that persist between runs: one
    `~/.exudyn` file, and what may be in it** (#2666). The results monitor introduced
    `~/.exudyn/resultsMonitor.json` (`resultsMonitor.SettingsFileName`) without a decision about what
    such a directory is *for*. The maintainer: *"this is basically good and could be used for other
    things as well (store window positions, dialog sizes, even fontscaling, etc. in a systematic
    manner) ... mostly I would see overrides for anything in visualizationSettings - except special
    types - and exudyn.config (like config.outputDirectory)"*.

    **What is settled**: one file rather than one per tool; it needs documentation; and because it
    changes what a script does when it is present, it belongs in `docs/manual/revisions.md`. A note
    is printed on the first import when the stored settings are **not empty**, because a stored
    setting makes a run less reproducible and the user must be able to see that from the output.

    **What is open, and is what the sub-steps decide.** Each of these is a real fork, not a detail:

    - **What may be overridden.** `visualizationSettings` (excluding the types that are not a plain
      value - a `BodyGraphicsData`, a user function, a container) and parts of `exudyn.config` such
      as `outputDirectory`. A whitelist by type is checkable; a free-form dictionary is not.
    - **Who reads it.** Either `python/exudyn/__init__.py` reads the JSON and writes the values into
      the module through the existing dict interface - Python only, no C++ change, and the values are
      in place before a script can look at them - or C++ reads it with
      `py::module_::import("json")`, which puts the file into the core and its failure modes with it.
      The first is the smaller change and is the recommendation to argue against.
    - **When it is applied**, and whether a script can ask what came from the file rather than from
      the defaults. Without that, a bug report about a setting is not reproducible by the reader.
    - **Whether the results monitor's own file is folded in** or kept beside it. Folding it in is the
      point of "one file"; keeping it is less work and leaves the monitor standalone.
    - **The dialog settings that drive it**: `storeDialogPositions` (position and size) and
      `storeDialogSettings` (font size, columns, opened trees) in `visualizationSettings`, so that
      storing is something a user switches on rather than something that happens.

    Related: **RG6.2.11** (#2608) is a second file storing overall window states, and this step
    should decide whether that is the same file.

    - **RG12.5.1** **DONE 2026-09-26** — [log](exudynRevisionLog2026b.md#rg12-5-1) - the file, the
      two sections that are read, the note, and the switch. `~/.exudyn/config.json` with `config`
      and `visualizationSettings`; `python/exudyn/settings.py`; applied at import and at every
      `SystemContainer`; `exudyn.settings.Print()` says what came from it; nothing writes it by
      itself; `EXUDYN_NO_USER_SETTINGS=1` ignores it and **the four test runners set that
      variable**, so a stored setting can never move a test result.

      Three of the four open questions are answered by it: **what may be overridden** (plain values
      and lists of them; anything else is refused with a message), **who reads it** (Python, in
      `__init__.py`, so the C++ core is untouched and the values are in place before a script can
      look at them) and **when a script can ask** (`Applied()`, `Ignored()`, `Print()`).
    - **RG12.5.2** **DONE 2026-09-28** — [log](exudynRevisionLog2026b.md#rg12-5-2) — **the types
      that are not plain.** An enum - `OutputVariableType`,
      `ItemType` - is stored honestly as its name, and `settingsUtilities` already converts between
      the two (`ConvertString2Value`, `EnumFullName`). 2 of the 466 visualization settings are
      enums, which is why they were left out of .1 rather than guessed at.

      The measurement and the decision are in the
      [log](exudynRevisionLog2026b.md#decisions-2026-09-29).
    - **RG12.5.3** **DONE 2026-09-26** — [log](exudynRevisionLog2026b.md#rg12-5-3) - the dialogs
      section, which is **RG6.2.29** (#2675) built: `visualizationSettings.dialogs.
      storeDialogPositions` (new, default False), the `"dialogs"` section of the file, and the rule
      of RG6.2.11 - the size always, the position only when the window would still be reachable.
    - **RG12.5.4** **DONE 2026-09-26** (by RG12.10, #2684) - **folding in
      `~/.exudyn/resultsMonitor.json`.** Done without the migration the step planned: the maintainer
      deleted the file on 2026-09-26 and decided it shall not be used again, so the monitor reads its
      section of the one file and the old name disappears everywhere.

<a id="rg12-6"></a>
**RG12.6** **DONE 2026-09-26** (#2667) — [log](exudynRevisionLog2026b.md#rg12-6) —
    **The columns of a settings dialog are relative and configurable**. `misc/GUI.py` gives the tree four fixed widths - 325, 188, 113 and 420
    pixels, multiplied by the dialog scaling - so a long name is cut off on every screen.
    `visualizationSettings.dialogs` gets `columnWidthName`, `columnWidthValue` and `columnWidthType`,
    each a fraction in 0..1 of the dialog width, and the **description column takes what is left**,
    which is what makes three numbers enough. The minimum widths stay, because a column of zero
    width is not a configuration a user means.

<a id="rg12-7"></a>
**RG12.7** **DONE 2026-09-26** (#2668) — [log](exudynRevisionLog2026b.md#rg12-6) —
    **The mouse wheel changes the font size of a dialog**. About 10% per notch, up and down. Every metric of the dialog already follows the font -
    `DialogFontSize`, `DialogRowMetrics`, `textHeightFactor` - so the work is to rebuild the tree at
    the new size and to keep the scroll position. **Which modifier** is the open question: the wheel
    alone scrolls the tree, so it is `Ctrl` + wheel unless the maintainer prefers otherwise, and on
    macOS that is a different event name than on Windows and X11.

<a id="rg12-8"></a>
**RG12.8** *(group RG12; maintainer 2026-09-26)* **DONE 2026-09-26** — [log](exudynRevisionLog2026b.md#rg12-8) -
    **One test for all user functions at once** (#2671). RG12.4 made one def the source of four
    generated things - the documentation block, the entry of `userFunctionArgsDict`, the `Protocol`,
    and the check against the C++ `std::function`. The generators compare them **while they run**;
    nothing compared what was **shipped**, and the item test models exercise a handful of user
    functions rather than the set. `python/testing/test_userFunctions.py` reads the installed package
    only, so it fails when a generated file is stale, when a `Protocol` is missing from `__all__`, or
    when an argument was renamed in one place and not the other. It runs no simulation: a model per
    user function is what the test models are, and would be testing the solver.

<a id="rg12-9"></a>
**RG12.9** **DONE 2026-09-26** (#2679) — [log](exudynRevisionLog2026b.md#rg12-9) —
    **The override settings live in `exudyn.special.overrideSettings`**, a dictionary that
    `import exudyn` fills once from `~/.exudyn/config.json` and that both Python and the C++ core
    read, and the module that reads and writes the file is **`exudyn.misc.overrideSettings`**
    (`exudyn.settings` is gone - it is internal, and a user reaches the values through
    `exudyn.special.overrideSettings`).

    **The decided `py::dict` carrier, with the lifetime caveat handled the other way round.** The
    step said "a `py::dict` member of `PySpecial`"; the member would have put pybind11 into
    `Main/Experimental.h`, which **eight** translation units include, two of them in `Linalg` and
    `Utilities`. The dictionary is therefore `EPyUtils::OverrideSettings()` in
    `Pybind_manual_classes.cpp` - allocated once on the first access, during module import while the
    interpreter and the GIL are there, and **never freed on purpose**, which is the caveat the step
    itself named: a global that releases a Python reference after finalization crashes the process.
    From Python it is what was asked for, `exu.special.overrideSettings`, read-only so that it cannot
    be replaced by something that is not a dictionary, and `__repr__` says how many sections are in
    it.

    **What changed beyond the move**: every function that took `settings=None` now means *the store*
    and not a second read of the file, so the dialogs and the settings can no longer disagree within
    a run - `DialogGeometry` re-opened the file on every call. `Store` and `StoreDialogGeometry`
    write the file **and** the store, for the same reason. Three new tests, 29 in the file.

<a id="rg12-10"></a>
**RG12.10** **DONE 2026-09-26** (#2684) — [log](exudynRevisionLog2026b.md#rg12-10) —
    **The workflow of the override settings**, as the maintainer wrote it out, in six steps that are
    now also the documentation.

    **A stored `visualizationSetting` reaches every structure that is created** - the one a
    `SystemContainer` builds and one built by `exu.VisualizationSettings()`, which got nothing before
    - through two subclasses that `import exudyn` installs *only* when the file holds such a section.

    **The trap the subclass sets is closed in the same step**, and it is the part worth remembering:
    `DefaultSettingsDictionary` called `type(structure)()`, so on an instance of the subclass it
    returned the **override** as its own default - measured at `multiSampling: 4` where the default is
    1 - which would have broken every "diff to default", the dialog's marking and `Store(SC)` without
    a word. `settingsUtilities.CompiledSettingsClass(structure)` walks to the class the compiled
    module defines, and the defaults are the defaults again.

    **The results monitor has no file of its own.** The maintainer deleted
    `~/.exudyn/resultsMonitor.json` on 2026-09-26 - it had existed for a few hours - and decided
    there is no migration: the monitor reads and writes the `resultsMonitor` section of the one file,
    `SettingsFileName` is gone, and the old name is gone from the documentation.
    **`overrideSettings.StoreSection(name, values)` is the one writer of a section**, which the
    dialogs, the monitor and `Store` all go through.


    - **RG12.10.1** **DONE 2026-09-27** (#2705) — [log](exudynRevisionLog2026b.md#rg12-10-1) — **the
      import note is one short line, and the file can switch it off.** *"NOTE: 8 visualizationSettings
      read from ~/.exudyn/config.json"* - a count of 0 is not printed - and
      `"suppressOverrideSettingsWarning": true` in the file keeps it quiet.

<a id="rg12-11"></a>
**RG12.11** **DONE 2026-09-26** (#2685) — [log](exudynRevisionLog2026b.md#rg12-11) —
    **Storing the override settings**, all three parts of it.

    **`exudyn.config` has a dictionary interface and its defaults** - `GetDictionary()`,
    `SetDictionary(d)`, `GetDefaults()` - which is what the maintainer asked for. The defaults are
    **snapshotted during module import**, because they cannot be constructed: `ExudynConfig` is a
    facade over globals, so a second one reports the current values (measured: 12 after
    `outputPrecision = 12`). What Exudyn starts with **is** the default, taken while it still is, so
    nothing is written down twice and nothing can drift. The settings are listed once, with the three
    read-only ones marked, and `Main/Config.h` stays free of pybind11 - eight translation units
    include it.

    **`Store(config=...)` stores what differs from those defaults.** It used to store every value that
    was not `''`, `0` or `False`, so it stored `printToConsole` from every run and `outputPrecision`
    because 6 is not 0 - settings a user never touched.

    **The dialog has a "store settings" button**: the `visualizationSettings` that differ from the
    defaults and the dialog's own size and position, nothing else, and it **shows exactly what it will
    write** with a store/cancel pair first, because one click reaches the home directory. It stores
    the geometry whether or not `storeDialogPositions` is on - that flag is about remembering on
    closing, this is a user asking.

    **"diff to default" stays a difference to the real default**, and what the file already stores is
    named again under a comment line saying so, as decided. The grouping is
    `GUI.SplitStoredFromChanged`, a function, so it is tested without opening a window.

<a id="rg12-12"></a>
**RG12.12** **DONE 2026-09-27** (#2588 family; no issue of its own) — [log](exudynRevisionLog2026b.md#rg12-12) —
    **PlotSensor takes its defaults from the override settings.**

    **The defaults**: the eleven arguments that shadowed `PlotSensorDefaults()` now default to `None`,
    and `None` is what asks for the default. They used to be compared against *the original literal
    default* - `if fontSize == 16` - so passing that value on purpose could not be told from not
    passing it, which `PlotSensorDefaults()` documented as a wart of its own: *"BUT PlotSensor(...,
    fontSize=16) will use fontSize=12, BECAUSE 16 is the original default value!!!"*. That sentence is
    gone from the docstring because the behaviour is gone, and the mutable default arguments
    (`colors=[]`, `sizeInches=[6.4,4.8]`) went with it.

    **The file**: a `plotSensor` section of `~/.exudyn/config.json` sets any of those defaults, read
    when `exudyn.plot` is imported; a name that is not a default is reported rather than invented.

    **The window positions**, by the decision recorded here: **by their sequence**, the counter reset
    by `closeAll=True`, because plot windows have no unique title. Only the **position** - the size of
    a plot is `sizeInches`, which is already a default the file can set - and only when
    `PlotSensorDefaults().storeWindowPositions` is True, which is off, as every "store where I left it"
    in Exudyn is. It reuses what the dialogs use: the `dialogs` section, `DialogGeometry`, the
    reachability rule and `StoreGeometryString`, and it is written for the tkinter and Qt backends and
    silent on anything else. **No test opens a plot window**, so that half is contract, not
    measurement.

<a id="rg3-23"></a>
**RG3.23** **DONE 2026-09-26** (#2680) — [log](exudynRevisionLog2026b.md#rg3-23) —
    **The override settings are documented where the module is.** The text of
    `docs/manual/userSettings.md` is now a section of the Exudyn module page,
    *Settings that persist between runs* (`sec-overridesettings`), written as `pb.AddDocu(...)` in
    `definitions/pybindModule.py`; `exu.special.overrideSettings` is a documented data member beside
    `exu.sys` and `exu.variables`; and the manual page keeps its place in the table of contents as a
    pointer to it.

    **The environment variables have a list**, `sec-environmentvariables`, and it was needed: of the
    **six** the package reads, three - `EXUDYN_NO_USER_SETTINGS`, `EXUDYN_CONFIG_FILE` and
    `EXUDYN_IMPORT_VERBOSE` - were documented nowhere at all.

    **The troubleshooting hint** is in `performanceErrors.md`, *Behaviour that is not in your
    script*: deleting `~/.exudyn` returns everything to the defaults, and
    `EXUDYN_NO_USER_SETTINGS=1` answers the question without deleting anything.

    Two rules were learnt against the gate rather than from the README: inline code in a description
    is a **backtick span** and `\texttt{}` is rejected outside mathematics - the README said the
    opposite and now says what is checked - and a sub-heading of an `AddDocu` section is level
    **4**.

    **Still open, and not part of this step**: `exu.config` and `exu.special` are documented by
    `pb.DefLatexDataAccess(...)` written by hand while the settings structures have a generator.
    The maintainer: *"Ideally, the access to structures would be handled and documented both via the
    same mechanism ... but I don't know if this needs an improvement right now."*

<a id="rg3-24"></a>
**RG3.24** *(group RG3; maintainer 2026-09-26)* **DONE 2026-09-27** — [log](exudynRevisionLog2026b.md#rg3-24) — **The generator API still says "Latex"** (#2681).
    Done in three passes, each gated by a regeneration that changed nothing: the declaration API and
    its class, the local names, and the dead LaTeX the reading found. **RG3.24.4 is the one question
    left** and it is the maintainer's.
    *"DefLatexDataAccess and similar Latex commands need to be just renamed consistently. Search for
    latex and see where it still makes sense, rename where clear (Markdown) or suggest options when
    unclear."*

    Measured the same day over `tools/generators/` and `definitions/`: **28 files** carry the word.
    The big ones are the pybind declaration API, which every definition file uses -
    **`DefLatexDataAccess` (64 uses)**, `DefLatexStartTable` (32), `DefLatexFinishTable` (27),
    `DefLatexOperator` (17), `DefLatexStartClass` (13) - and they write **Markdown** and have done
    since RG3.14. Then `PyLatexRST` (18), which writes Python, a stub and Markdown and neither LaTeX
    nor RST, and the small ones: `latexSymbol`, `moduleNameLatex`, `sLatexObjectClass`, `latexStr`.

    **Three kinds, and only the third needs a decision.** A name that describes **Markdown output**
    is renamed by rule (`DefLatexDataAccess` to `DefDataAccess`, `PyLatexRST` to something that says
    what it writes). A name that describes **real LaTeX** stays: `Str2Doxygen` writes C++ comments
    and `latexToMarkdown.py` is named after what it converts **from**. The third kind is
    `latexSymbol` and its family - the `$...$` symbol of an item parameter, which **is** LaTeX inside
    Markdown - where `mathSymbol` says what it is and touching it moves 12 uses in three emitters.
    Options go to the maintainer with the list.

    **RG3.24.1/.2 corrected one sentence of this step**: `Str2Latex` and `GetTypesStringLatex` were
    named here as functions that feed real LaTeX and therefore keep their names. They do not - see
    the audit below - so they are a question of *removal*, not of renaming.

    A rename of 64 call sites in the definition files is a large diff that changes no output, so it
    wants the same gate as RG3.14.13: **the regeneration is a no-op**, or the rename was not one.

    - **RG3.24.1** **DONE 2026-09-26** — [log](exudynRevisionLog2026b.md#rg3-24-1) - **where the
      legacy string helpers are still called.** Asked for by the maintainer: *"I still believe that
      functions like Str2Latex and in particular DefaultValue2Python would now be replaced by the new
      generators and definitions mechanisms."* Nine helpers, 60 call sites, all in
      `tools/generators/`:

      | helper | call sites | what it is given |
      |---|---|---|
      | `Str2Latex` | 21 | a type name, a size, a default value, a python name, an argument name, a description |
      | `Str2Doxygen` | 17 | the text of a C++ `//!` comment |
      | `ExtractLatexSymbol` | 9 | a description that begins with `$...$` |
      | `GetTypesStringLatex` | 8 | the `Node::Position`-style requested types of an item |
      | `DefaultValue2Python` | 5 | the C++ literal of a default value |
      | `Latex2RSTlabel` | 4 | a section label |
      | `RemoveSpacesTabs` | 3 | a type string |
      | `SplitString`, `CutLinesFromString` | 0 | nothing - imported by `itemDocsEmitter` and never called |

      `PyLatexRST` (18) and `NormalizeHeadings` (10) are counted with them in the earlier survey and
      are **not** legacy: the first is the writer every emitter uses, the second is the Markdown rule
      of RG3.14. They belong to the renaming, not here.

    - **RG3.24.2** **DONE 2026-09-26** — [log](exudynRevisionLog2026b.md#rg3-24-2) - **what each of
      them still does**, measured by giving every call site its real inputs (the generator runs each
      stage as a subprocess, so a wrapper would have seen nothing):

      | helper | verdict |
      |---|---|
      | `Str2Latex(s)`, plain | **a no-op, provably**: 0 of 3720 type names, sizes, python names and descriptions are changed. All it does now is `{` to `\{`, into Markdown, where it would be wrong if it ever fired |
      | `Str2Latex(s, isDefaultValue=True)` | the **only** source of the printed default of 990 item and 263 settings parameters; not LaTeX at all, a C++-to-Python converter with a different rounding than `DefaultValue2Python` (`np.zeros((6,6))` against `IIDiagMatrix(...)`) |
      | `DefaultValue2Python` | the same conversion for the interface and the stubs, 1001 of 2837 inputs changed; see below |
      | `GetTypesStringLatex` | **correct, and RG3.24.3 corrected this row**: it writes `\texttt{...}` into text that goes through the LaTeX-to-Markdown converter, which turns it into a backtick span. Only the **name** is wrong, which is RG3.24 |
      | `Str2Doxygen` | **correct and stays**: it escapes for a C++ comment, which is what it says |
      | `ExtractLatexSymbol` | **stays**, and is the `latexSymbol` question of RG3.24 - it splits a real `$...$` off a description |
      | `Latex2RSTlabel`, `RemoveSpacesTabs` | small, correct, badly named (`Latex2RSTlabel` makes a **MyST** label) |
      | `SplitString`, `CutLinesFromString` | **dead**, and the import in `itemDocsEmitter` is the only thing that keeps them |

    - **RG3.24.3** **DONE 2026-09-26** (#2682) — [log](exudynRevisionLog2026b.md#rg3-24-3) -
      **the default values stopped making a round trip through a C++ literal string.** Option A was
      built, and the audit's own premise turned out to be too pessimistic: `CppValue` has carried
      `ToPython()` and `ToDocument()` since it was written and **nothing had ever called them**, so
      for 142 of the 402 the renderings were already there to be asked for.

      `definitionTypes.py` gained **`PythonLiteral(value, typeName)`** and
      **`DocumentLiteral(value, typeName)`** beside `CppLiteral`: a `CppValue` is asked, a plain
      number, flag or string is its own Python value, and C++ source text is translated by named
      tables - `emptyContainerValues` (17 entries), `namedDefaultValues` (7), `innerDefaultValues`
      (1) - plus one rule for a braced initializer and one for an enum value. **An expression no
      rule covers raises `UnknownDefaultValue`**, which stops the generator and names the table to
      extend; that is the whole difference from guessing. No regular expressions, because
      `checkDefinitions` reads every string literal of a definition file and a backslash followed by
      a letter is a LaTeX command to it - which is right for a description, so the patterns are
      plain string logic and the f-suffix rule is written out.

      **`DefaultValue2Python`, `Str2Latex`, `SplitString` and `CutLinesFromString` are gone** - 201
      lines - and with them the 14 `Str2Latex` calls that RG3.24.2 measured at 0 changes out of
      3720 inputs, and the emitter workaround *"don't do this for file names, because 'f' is
      erased!"*. Removing them changed the generated output by **nothing**, which is what those
      measurements predicted.

      **What it repaired in the published documentation**, all of it found by writing the tables out:

      | in the pages | was | is |
      |---|---|---|
      | 85 cells | `[ invalid [-1], invalid [-1] ]` | `[ invalid (-1), invalid (-1) ]` |
      | 30 cells | `Matrix[]`, `PyMatrixContainer[]`, `MatrixI[]` | `[]` |
      | 1 cell | `[Matrix3DF[3,3,1.,0.,0., 0.,1.,0., 0.,0.,1.]]` | `[[1.,0.,0.], [0.,1.,0.], [0.,0.,1.]]` |
      | the signature of `ObjectContactConvexRoll` | `coefficientsHull =  []` | `coefficientsHull = []` |

      **74 lines in 47 generated files, and every one of them is in that table** - the gate the step
      asked for, run before and after the removals. **17 new tests**
      (`python/testing/test_defaultValueRenderings.py`), each naming the defect it forbids, and one
      that renders all 1376 defaults so that a missing rule is a test failure and not a surprise
      during a release.

      **One thing was deliberately not done**: the three `CppValue` constants carried a *document*
      wording that disagrees with what is published - `'invalid index'` against `invalid (-1)`, and
      prose for the default colour against `[-1.,-1.,-1.,-1.]`. The published wording won and the
      constants were corrected to it: this step removes corruption, it does not re-word the manual.
      Whether the default column should read `exudyn.InvalidIndex()` instead of `invalid (-1)` is a
      decision, and it is a small one now that there is one place to make it.

    - **RG3.24.4** *(the question RG3.24 reserved; maintainer's choice)* **DONE 2026-09-27** (#2699)
      — [log](exudynRevisionLog2026b.md#rg3-24-4) — **what the `latexSymbol` family should be
      called.** The maintainer chose **B**, `mathSymbol`. `latexSymbol` (12 uses in `itemDocsEmitter`, `itemHeaderEmitter`,
      `itemInterfaceEmitter` and `typesEmitter`), `ExtractLatexSymbol` (`itemModel`, 9), and inside it
      `stringLatexSymbol` and `addLatexSign`. It is the `$...$` that a parameter description may open
      with - `$\theta$ rotation angle` - which the emitters split off and put into its own column of
      the parameter table.

      **Why it was not renamed by rule**: the thing it names *is* LaTeX, so `latexSymbol` is not
      wrong the way `DefLatexDataAccess` was wrong. The options:

      | option | what it costs | what it buys |
      |---|---|---|
      | **A** leave it | nothing | the name says the markup, which is true |
      | **B** `mathSymbol`, `ExtractMathSymbol`, `mathSymbolString`, `mathSign` *(recommended)* | one mechanical rename, 24 occurrences in five files, gated by a no-op regeneration | the name says what it *is* - the symbol of the parameter - and the markup stays the converter's business |
      | **C** rename the function by what it does, `SplitLeadingMathSymbol(description)`, and leave the variables | the same rename plus a docstring | the function name says that only a **leading** `$...$` is split off, which is the rule nothing states today and which its error message does not name either |

      B and C are compatible: C is B plus a better name for the function.

<a id="rg12-13"></a>
**RG12.13** **DONE 2026-09-26** (#2686) — [log](exudynRevisionLog2026b.md#rg12-13) —
    **A stored dialog geometry is used.** `RestoreWindowGeometry` asked
    `visualizationSettings.dialogs.storeDialogPositions` first and returned early, so what the store
    button of RG12.11 wrote was never read back - measured: the window was asked for the default
    `900x700` while `1122x1751+7+14` was stored. **The flag now decides only what its name says**,
    whether a dialog stores *itself* when it closes, which is where `RememberWindowGeometry` still
    asks it.

    `StoreDialogPositions(settingsStructure=None)` **asks the structure being edited first**, because
    `GetRendererSystemContainer()` is None whenever no container is attached to a running renderer: for
    `python -m exudyn dialogs`, and for any script before `renderer.Start()`, the flag was False
    however it was set, and such a dialog could never store itself either.

    **And a stored size is cut down to the current screen**, for the reason the position is checked at
    all: a settings dialog is taller than it is wide and its buttons are in the bottom row, so a size
    stored on a larger or a rotated monitor put the close button off the screen. The maintainer's own
    file - 1751 pixels high - is exactly that case on a 1234-pixel screen. The reachability rule for
    the position is unchanged.

    - **RG12.13.1** *(parked, 2026-09-26)* **the error that could not be reproduced.** The maintainer
      reported that opening the visualization settings with a `config.json` holding a `dialogs` entry
      *"reports an error"*. Their exact file, through the load path and through the same dialog built
      in a withdrawn window, raises **nothing**, and every reader of the section returns what it
      should. They have since **deleted the file** - *"as it might have been in an invalid state"* -
      so there is nothing left to chase. It stays here so that a recurrence is recognised rather than
      investigated from the beginning; what is needed then is the text of the error.
    - **RG12.13.2** **DONE 2026-09-26** (#2690) **a version in the settings file.** *"there should be a version in
      the config file, as we may change the structure or anything in the future, and only a version
      can help to decide whether or how an older file can be used."* The structure changed twice in
      one day - the `resultsMonitor` section was folded in, the `dialogs` section was added - so the
      case is real and not hypothetical.

      **Which version is the decision, and it matters more than it looks**: the micro version is
      derived from the count of resolved issues, so an exact match on the full version
      (`1.12.95.dev1`) throws a user's settings away **on every issue that is resolved**, and on every
      patch release.

      **The maintainer chose option C, and said how small it should be**: *"just add a version
      number 1 for now. As soon as exudyn was released (so no earlier than that makes sense for users
      out there) AND that we changed behavior of the config.json, we can increment the version just
      using 2. Very simple, no deep tech; similar as in FEM. But we need a version in the long term,
      so that we know whether a user stores a very old file that is not readable any more."*

      `overrideSettings.fileFormatVersion = 1` is written into every file by `Save`, always the
      current one whatever the file said, and `Load` **ignores** a file that does not carry exactly
      that number, with one note naming both versions and saying to store the settings again. It is
      never a section and never reaches `exudyn.special.overrideSettings`.

      **And nothing remembers the format that was not versioned** (maintainer, 2026-09-26): *"the
      config file was just alive a few hours, we don't track something like that in the memory of the
      code."* The rule is the rule - a file carries the number or it is not read - and the code says
      that and no more.

<a id="rg12-14"></a>
**RG12.14** **DONE 2026-09-26** (#2687) — [log](exudynRevisionLog2026b.md#rg12-14) —
    **The override settings can be read again**: `overrideSettings.Reload()`. A file edited while a
    session runs, or stored by another session, takes effect without restarting the interpreter -
    which is the Spyder case, where the kernel stays and a second `import exudyn` does nothing.

    The store is emptied first, so a section removed from the file is gone from it; `config` is
    applied; and the `visualizationSettings` reach every structure created afterwards, **including the
    case where the file had none at import** - then the reload is what installs the wrapped
    constructors. **A reload does not undo**: what already reached `exudyn.config`, and a structure
    that already exists, keep what they were given, and that is documented where the reload is.

    The step shrank twice before it was built, both times because the maintainer looked: `StoreSection`
    already updates the store, and the wrappers close over that same dictionary - so what looked like
    "it does not store" was RG12.13.

<a id="rg12-15"></a>
**RG12.15** **DONE 2026-09-27** (#2688) — [log](exudynRevisionLog2026b.md#rg12-15) —
    **A script can place a dialog, and it is written down.** `StoreDialogGeometry(name, size,
    position)` with the dialog's title, in the Exudyn module page beside the file it writes; the
    render window with its own two settings, including what its position means; and the footnote the
    maintainer asked for - writing into `exudyn.special.overrideSettings` works, holds for one run and
    does not touch the file, and is not the recommended way because nothing checks what is put there.

<a id="rg12-16"></a>
**RG12.16** **DONE 2026-09-26** (#2689) — [log](exudynRevisionLog2026b.md#rg12-16) —
    **The render window and the SolutionViewer remember their size and position**, as the settings
    dialogs do since RG12.5.3.

    - **RG12.16.1** **DONE 2026-09-26** **the render window can be placed.** `view*.window.renderWindowPosition` and
      `view*.window.useRenderWindowPosition`, ordinary settings, so the settings file, `Store(SC)` and
      the store button carry them with nothing added, one set per view. `GlfwClient.cpp` calls
      `glfwSetWindowPos` when the flag is on - it never called it at all.

      **The `(-1,-1)` sentinel is what it uses**, after a detour: it was built with a
      `useRenderWindowPosition` flag instead, because the settings dialog refuses a negative
      `IndexArray` - the type the position shares with the size - and that rule is the only guard
      there is, since the C++ accepts a negative sensor number *and* a negative window size. The
      maintainer then **measured the render window itself** (2026-09-26): a GLFW window position is
      always positive, *"so this means that we CAN take the negative values (any of both)"*, and
      asked for the flag to go. It did. The dialog still cannot type a negative value there, which is
      named in `knownRoundTripGaps` with the reason: a user **sets** a position in the dialog, which is
      positive, and unsets it in the file or from a script.

      They also measured what the position means: it is the position of the **OpenGL area**, not of the
      title bar, so a value below about 50 hides part of the title bar and 0 hides it completely -
      *"this works, as there is still the escape button"* - which is a way to have a view without one.
      The description says so.
    - **RG12.16.2** **DONE 2026-09-26** **it remembers where it was.** `view*.window.storeRenderWindowGeometry`, off by
      default, writes the size and the position back into the settings when the window closes, so that
      storing the settings keeps a render window where it was left. Off by default for the reason
      `dialogs.storeDialogPositions` exists: a settings structure that changes by itself would make
      *diff to default* report a window position after every run.
    - **RG12.16.3** **DONE 2026-09-26** **the SolutionViewer, and two more for free.** Its window is an
      `InteractiveDialog` - and so are the mode shapes and an interactive simulation - so all three
      restore and store themselves under their own title, through the same
      `RestoreWindowGeometry`/`StoreWindowGeometry` and the same `dialogs` section as the settings
      dialogs. `RestoreWindowGeometry` leaves the size to the layout when nothing is stored, which a
      dialog that sizes itself from its widgets needs.

<a id="rg12-19"></a>
**RG12.19** **DONE 2026-09-27** (#2693) — [log](exudynRevisionLog2026b.md#rg12-19) —
    **Two buttons: one for the settings, one for the positions**, each showing what it will write
    before it writes it. The **render window** geometry rides along in the settings button, by the
    maintainer's decision: *"I opt to store it in the config file in the visualizationSettings, because
    it is the straightforward way and becomes now natural, because it is only stored if it differs from
    default."*

<a id="rg12-20"></a>
**RG12.20** **DONE 2026-09-27** (#2694) — [log](exudynRevisionLog2026b.md#rg12-20) —
    **Where the render window is, and what happens when the file and the session disagree.**

    The maintainer pointed at the code: *"there is already SetRenderStateScreenSize in GlfwClient.cpp
    and it only needs to be copied or extended to size AND position ... follow the trace of the
    state->currentWindowSize, to add a currentWindowPosition to the RenderState, also making it
    read/write in the MainRenderer::Get/SetState."* That is what was done, and the trace was exactly
    as described.

    **No window-move callback was needed**: the size is refreshed on every `Render`, so the position is
    asked for in the same place - one `glfwGetWindowPos` next to a redraw - and there is one place where
    the state learns about the window instead of two. `SetState` writes the position **and**
    `view*.window.renderWindowPosition`, as it has always done for the size, which is the maintainer's
    *"otherwise a re-open would not have the just stored positions"*.

    **The conflict is said once.** `SC.renderer.Start()` compares the `visualizationSettings` section of
    the file with what the session has, names both values when they differ, and is silent when they
    agree. It runs **before** the `suppressRenderer` guard, because the disagreement is between the file
    and the session whether or not a window opens - and with no settings file it returns at once, which
    is every test run.

<a id="rg3-25"></a>
**RG3.25** *(group RG3; raised 2026-09-26)* **DONE 2026-09-27** (#2683) —
    [log](exudynRevisionLog2026b.md#rg3-25) — **a TAB instead of a backslash put `exttt{...}` on three
    pages of the Symbolic manual.** Three descriptions fixed to a backtick span, four typos with them,
    and `checkDefinitions` rejects a TAB in a description, which is the part that was worth deciding:
    the LaTeX rule looks for the backslash, and the TAB had eaten it.

<a id="rg3-26"></a>
**RG3.26** *(group RG3; maintainer 2026-09-27)* **DONE 2026-09-27** —
    [log](exudynRevisionLog2026b.md#rg3-26) — the maintainer chose **option B**, the check.
    **`index.md` and `pdfIndex.md` are two hand-written
    tables of contents that must agree** (#2697). *"Maybe this is necessary, but it is really brittle
    and requires a clear indication to sync the toctrees ... The rest should be practically identical,
    if possible."*

    **Measured, and it is more than remembered**: 25 toctree entries against 23. Only in `index.md`:
    `README`, **`docs/manual/performanceErrors`**, the examples index and the test-models index. Only in
    `pdfIndex.md`: `docs/manual/commandLine` and `docs/manual/resultsMonitor`, which in the HTML are
    nested under `introductionAdvanced` instead. **And the order of the shared entries differs.** So the
    examples and the front page are the intended differences; a whole chapter missing from the PDF, the
    different nesting and the different order are not, and nothing says so when they drift again.

    - **Option A**: generate `pdfIndex.md` from `index.md` with a declared list of exclusions. One file
      is then the truth and the other a build product - and it needs a rule for the front page, which is
      the one part that really differs.
    - **Option B (cheapest, and it catches drift tomorrow)**: keep both and add a check to `tools/` that
      compares the two entry lists against a **declared** difference, the way `checkAll` compares
      `__all__` against what a module defines. A new page in one and not the other then fails a gate
      instead of being noticed months later.
    - **Option C**: accept the difference and say so at the top of both files. The least work and the
      least protection.

    - **RG3.26.1** **DONE 2026-09-27** (#2702) — [log](exudynRevisionLog2026b.md#rg3-26-1) — **the
      part that is a defect, not a decision.** The drift has a date: RG3.15 restructured the user
      manual in `index.md` on 2026-09-25 and `pdfIndex.md` was not changed with it. `pdfIndex.md` now
      takes the user-manual order of `index.md`; what differs is what is meant to - `README`, the
      examples and test models, the front page - and **the choice among A, B and C is still open**.

<a id="rg3-27"></a>
**RG3.27** *(group RG3; maintainer 2026-09-27)* **DONE 2026-09-27** (#2708) —
    [log](exudynRevisionLog2026b.md#rg3-27) — **The mass-spring-damper tutorial comes first.**

<a id="rg12-21"></a>
**RG12.21** **DONE 2026-09-27** (#2695) — [log](exudynRevisionLog2026b.md#rg12-21) — **`python -m exudyn info` prints the home directory** - and the command exists to be pasted into an issue, so it carries an account name with it.
    The home directory is shown as `%USERPROFILE%` or `~`, which is what a reader would type anyway, and
    `--showPaths` gives the real ones for a problem that is about a path.

<a id="rg12-22"></a>
**RG12.22** **DONE 2026-09-27** (#2696) — [log](exudynRevisionLog2026b.md#rg12-22) — **The results monitor took the focus and came to the front
    on every update** - *"so one cannot use the control panel"*. The cause is
    `plt.pause`, which calls `show(block=False)` every time, and TkAgg's `show()` does `deiconify()` and
    `lift()`. `canvas.start_event_loop` waits and processes events and does nothing else. The *"except
    optionally alwaysOnTop"* half is a monitor setting of that name, default False, stored in the
    `resultsMonitor` section like the rest, with `--always-on-top` on the command line.

<a id="rg12-23"></a>
**RG12.23** *(group RG12; maintainer 2026-09-27)* **DONE 2026-09-27** — [log](exudynRevisionLog2026b.md#rg12-23) — **The plot windows cannot be stored while the renderer
    is still open** (#2698). RG12.12 places a plot window where the one of the same sequence number was
    left, and stores it when it closes - but the maintainer is pointing at the *moment*: a user arranges
    several windows and wants to store them together, and the settings dialog, which is where storing
    happens for everything else, is usually gone by then because the renderer has stopped.

    Their design, and it is the right shape: a **function** a script or a dialog can call - *store where
    the plot windows are now* - which needs a list of the live figures. *"possibly the matplotlib figures
    need to be stored in an internal list - either in plot.py or in exudyn.sys, cleared when doing
    closeAll; the figure references should get invalid on closing ... and thus a function could then try
    to grab the current figure's positions and sizes and store them, allowing to reuse the size in the
    next PlotSensor commands."* With it, the sizes become reusable too, which the per-window close
    handler cannot do.


<a id="rg12-24"></a>
**RG12.24** *(group RG12; maintainer 2026-09-27)* **DONE 2026-09-27** (#2718) —
    [log](exudynRevisionLog2026b.md#rg12-24) — **The files a run writes by default go into
    `solution/`**, and the scripts that write beside themselves are found. The solution file, the
    solver information and the restart file default to `solution/...`; `exudev scripts` reports a file
    a script names without a directory and the default solution file read back by its old name; the
    repository's scripts follow, and neither the examples nor the test models write a file beside a
    script any more. The other way the maintainer named - the output directory set in Spyder - is
    documented beside the environment variables.

<a id="rg12-25"></a>
**RG12.25** *(group RG12; maintainer 2026-09-27)* **DONE 2026-09-27** (#2719) —
    [log](exudynRevisionLog2026b.md#rg12-25) — **store positions stores every open window**: the
    settings dialog, the other interactive dialogs (the SolutionViewer), the PlotSensor windows, and the
    render window where it is, which becomes `view0.window.renderWindowSize/Position` in the file and
    in the dialog.

<a id="rg12-26"></a>
**RG12.26** *(group RG12; maintainer 2026-09-27)* **DONE 2026-09-27** (#2720) —
    [log](exudynRevisionLog2026b.md#rg12-26) — **the SolutionViewer**: its sliders and the Run
    button follow the width of the window, `windowSize=[w, h]` gives it a size, it is stored with the
    other windows, and the label *t = 1.0* is gone.

<a id="rg12-27"></a>
**RG12.27** *(group RG12; maintainer 2026-09-27)* **DONE 2026-09-27** (#2722) —
    [log](exudynRevisionLog2026b.md#rg12-27) — **store positions with Qt plot windows, and the
    SolutionViewer when it is narrow**: the plot windows of Spyder's Qt backend are listed and stored
    (a `QRect` was taken for tkinter's geometry string, and the button failed before it showed
    anything); the button and label columns of a dialog keep their width, and only the slider column
    gives when the window is narrower than the dialog asks for.

<a id="rg12-28"></a>
**RG12.28** *(group RG12; maintainer 2026-09-27)* **DONE 2026-09-27** (#2723) —
    [log](exudynRevisionLog2026b.md#rg12-28) — **PlotSensor opens at the stored size**: the default
    `sizeInches` no longer resizes a window that was given its stored size; a `sizeInches` given in
    the script still wins.

## RG13 — Item documentation

*(Group created by the maintainer, 2026-09-27.)* **Every item gets a full documentation and a
MiniExample** - node, object, marker, load and sensor - which is more than fits into RG3, and **one of
the most important steps before Exudyn 1.13**: the reference manual of the items is what users read
most, and it is generated from `definitions/itemDefs*.py`, so what a definition does not carry, no
page shows.

What depends on it: the graphics regression test takes every item through its MiniExample
(RG2.3.3.5), and the image of each item on its page can be written by the same run.

<a id="rg13-1"></a>
**RG13.1** *(group RG13; maintainer 2026-09-27)* **DONE 2026-09-27** —
    [log](exudynRevisionLog2026b.md#rg13-1) — the table is
    [itemDocumentationState.md](itemDocumentationState.md), written by
    `tools/itemDocumentationReport.py`. **The state of the documentation, item by item**
    (#2715). Per item, measured from its definition and not estimated: the class description, the
    description of its equations, which parameters and output variables are described and which are
    not, whether it has a MiniExample and whether that one runs, an image, the examples and test
    models that use it. The deliverable is the table, and what it says about the kinds of items.

<a id="rg13-2"></a>
**RG13.2** *(group RG13; maintainer 2026-09-27)* **CLOSED 2026-09-28** - done by RG13.4 and RG13.5
    (maintainer's decision). **What the ideal documentation of an item contains**
    (#2716), per item type - node, object, marker, load, sensor - and per kind of object - body,
    connector, constraint, and the other object types. From that and the table of RG13.1: **a
    detailed plan that makes it work** for every item, as further steps of this group.

    The maintainer's direction - what *equations* means per kind, and a general section per kind -
    is in the [log](exudynRevisionLog2026b.md#decisions-2026-09-29).

<a id="rg13-3"></a>
**RG13.3** *(group RG13; maintainer 2026-09-27)* **Each description synchronized once with its
    implementation, and the definition says so** (#2717). *"each item's description needs to be
    one-time manually synched with the implementation; then gets a checked in the definitions file
    (or any better way for that)."*

    **The better way, proposed**: a flag that is set once stays true while the code moves on. The
    mark records **what** was checked as well as who and when - a fingerprint of the implementation it
    was checked against, the item's `src/Impl<Kind>s/C<Item>.cpp` and its generated header - so that
    `checkDefinitions` can report *"the implementation of ObjectJointRevoluteZ changed since its
    description was checked"*. The mark then means what a reader assumes it means, and a change to an
    item's C++ brings its description back into view in the commit that made it.

    **Decided (maintainer, 2026-09-27)**: *"the fingerprint is cool! yes, should be added. With a
    simple way to compute a new fingerprint after a change"* - the check reports the old and the new
    fingerprint, and after the description has been looked at again, the new one replaces the old
    one: one command, not a hand edit of a hash.

<a id="rg13-4"></a>
**RG13.4** *(group RG13; maintainer 2026-09-27)* **DONE 2026-09-29** —
    [log](exudynRevisionLog2026b.md#rg13-4) — **The development documents per item type** (#2721)
    - *what the documentation of a node, object, marker, load and sensor must contain*, evaluated on
    the text: from the tutorials and the Create functions - *"most model scripts now use Create
    functions, so the core functionality is hidden, which means that one has to build a combined view
    of a script and the Create functions in the background"* - and from the text the generator writes
    around the generated information (*"This Node has/provides the following types = Position"* could
    say which markers that allows, and could be generated). Nodes: how their coordinates are
    interpreted, made systematic. Objects in groups: rigid bodies, flexible bodies (nonlinear finite
    elements), connectors - which act on two or more markers, define the force from the kinematics and
    apply it through what the markers provide. Sensors, loads, markers: a general section before the
    first item, which every item refers to.

    **Written first as temporary documents** `docs/revision/<itemType>DefinitionsDev.md`; when the
    group is done, they become the section of the developer documentation that says what
    documentation an item needs, what it contains and how it is structured, and are removed. They
    are the input of RG13.2's plan.

    - **RG13.4.0** to **RG13.4.5** **DONE 2026-09-28** - the documents on what is common, and on
      nodes, objects, markers, loads and sensors; the maintainer's approval of each is in the
      [log](exudynRevisionLog2026b.md#decisions-2026-09-29).
    - **RG13.4.6** **DONE 2026-09-29** — [log](exudynRevisionLog2026b.md#rg13-4-6) - folded into
      [definitions/README.md](../../definitions/README.md) §*The page of an item*, and removed.

<a id="rg13-5"></a>
**RG13.5** *(group RG13; maintainer 2026-09-27)* **The documentation of the items, written by the
    documents of RG13.4** (#2725) - **DONE 2026-09-28 except `ObjectBeamGeometricallyExact`, which
    waits for RG4.8**. *"start a new step RG13.5, which adds according documentation for
    these types, again adding 13.5.1 for nodes, .2 for objects, ..."*. A kind is written when its
    document of RG13.4 is agreed: nodes, loads and sensors now, objects and markers after them.

    - **RG13.5.0** **the frame the pages are written into**, first (maintainer's decision,
      2026-09-27):
      - **RG13.5.0.1** **DONE 2026-09-27** (#2724) — [log](exudynRevisionLog2026b.md#rg13-5-0-1) -
        the fields of an item: `classDescription` is `overallDescription` - the brief text, used for
        the class, the docstring and the paragraph under the heading - and `equations` is
        `detailedDescription`, the full text after the generated part of the page. Items only; the
        structures keep `classDescription`.
      - **RG13.5.0.2** **DONE 2026-09-28** — [log](exudynRevisionLog2026b.md#rg13-5-0-2) - the
        general section of each kind of item, in a new definitions file
        `definitions/itemKindDefinitions.py` (maintainer's decision), with an `overallDescription` -
        today's paragraph of the index page - and a `detailedDescription`, written by .1 to .5.
      - **RG13.5.0.3** **DONE 2026-09-28** — [log](exudynRevisionLog2026b.md#rg13-5-0-3) - the
        generated frame of every item page: *Interface* instead of *Additional information*, the
        types in words (which markers, nodes, connectors and loads fit), the Python names on one
        line, headings of their own for parameters, output variables and the detailed description;
        `requestedNodeTypes` declared for the node markers. Generating the C++ check of the node
        markers from that declaration is #2727.
      - **RG13.5.0.4** **DONE 2026-09-28** (#2727) — [log](exudynRevisionLog2026b.md#rg13-5-0-4) -
        **the declared node types and the C++ check agree, and a test says so.** Every node marker
        with a declaration is attached to every node; what `Assemble()` accepts must be what
        `requestedNodeTypes` says (48 combinations, all agree), and a node marker that measures a
        position or an orientation without a declaration fails. The C++ check is not generated
        from the declaration: it is one rule on the marker type bits for all markers, not one per
        marker, and the test is what keeps the two from drifting apart.
      - **RG13.5.0.5** **DONE 2026-09-28** (#2729) — [log](exudynRevisionLog2026b.md#rg13-5-0-5) -
        **every item on a new page of the PDF** (maintainer): a raw LaTeX `\clearpage` opens every
        generated item page; the HTML build ignores it.
      - **RG13.5.0.7** **DONE 2026-09-29** (#2739) — [log](exudynRevisionLog2026b.md#rg13-5-0-7) -
        **the general section of a kind is a page of its own** (maintainer): *Node* -> *General info
        for all nodes* with its sub-sections -> `NodePoint`, `NodePoint2D`, ... as siblings; no
        *Items* heading.
      - **RG13.5.0.8** **DONE 2026-09-29** (#2741) — [log](exudynRevisionLog2026b.md#rg13-5-0-8) -
        **parameter tables** (maintainer): the symbol opens the description, `(symbol: $\fv$)`, instead
        of following the name; the columns of the parameter, output variable and marker tables have
        fixed widths in the PDF.
      - **RG13.5.0.6** **DONE 2026-09-29** (#2737) — [log](exudynRevisionLog2026b.md#rg13-5-0-6) -
        **the Create functions and the examples of a page, declared** (maintainer): a line
        **Simpler** before the parameters names the Create functions that add an object or load
        (`createFunctions`), and mentions of them in the texts are kept to that line; the basic items
        and all sensors name their examples (`examples`), the others keep the search by name.
    - **RG13.5.1** **DONE 2026-09-28** — [log](exudynRevisionLog2026b.md#rg13-5-1) - nodes - the
      table of coordinates, the frame and interpretation, the action on the equations of motion,
      constraints, singularities; the slopes of the slope nodes. Found on the way: the Euler parameter
      constraint is the node's, not the object's, and the default slopes of two nodes were parallel
      (#2728, corrected on the maintainer's decision the same day).
    - **RG13.5.2** objects, by group (maintainer 2026-09-28), bodies first:
      - **RG13.5.2.1** **DONE 2026-09-28** — [log](exudynRevisionLog2026b.md#rg13-5-2-1) - bodies -
        rigid bodies, mass points, 1D masses, ground; with the general section of the bodies: marker
        interfaces and the approach to the Jacobians.
      - **RG13.5.2.2** **DONE 2026-09-28** — [log](exudynRevisionLog2026b.md#rg13-5-2-2) - flexible
        bodies - the nonlinear finite elements; `ObjectBeamGeometricallyExact` after RG4.8. Found on
        the way: `ObjectANCFCable` declared an angular velocity it does not have (#2733, corrected).
      - **RG13.5.2.3** **DONE 2026-09-28** — [log](exudynRevisionLog2026b.md#rg13-5-2-3) - super
        elements - FFRF, reduced order FFRF, generic ODE2, kinematic tree. Found on the way: two of them
        admit body markers that fail (RG4.10, #2734).
      - **RG13.5.2.4** **DONE 2026-09-28** — [log](exudynRevisionLog2026b.md#rg13-5-2-4) - connectors -
        spring-dampers, contact, penalty joints. Found on the way: two defects of
        `ObjectContactCoordinate` (RG4.11, #2735).
      - **RG13.5.2.5** **DONE 2026-09-28** — [log](exudynRevisionLog2026b.md#rg13-5-2-5) - constraints
        and joints.
      - **RG13.5.2.6** **DONE 2026-09-28** — [log](exudynRevisionLog2026b.md#rg13-5-2-5) - the general
        objects - `ObjectGenericODE1`.
    - **RG13.5.3** **DONE 2026-09-28** — [log](exudynRevisionLog2026b.md#rg13-5-3) - markers - the
      general marker section with the generated table of all markers, and **Marker quantities** and
      **Jacobians** for the 13 markers that had little or no text; the five with long texts of their
      own (the two relative-coordinate markers, the two superelement markers, the kinematic tree
      marker) keep them. Found on the way: the shape markers accept any body (RG4.9, #2731).
      - **RG13.5.3.1** **DONE 2026-09-29** (#2738) — [log](exudynRevisionLog2026b.md#rg13-5-3-1) -
        the HTML comments of `MarkerSuperElementRigid` removed, what was valid of them restored as
        text; `MarkerKinematicTreeRigid` written from the C++. Found on the way: the prismatic joint
        Jacobian of `ObjectKinematicTree` (RG4.13, #2740).
    - **RG13.5.4** **DONE 2026-09-28** — [log](exudynRevisionLog2026b.md#rg13-5-4) - loads - the
      load, its frame, the generalized forces with the transformation.
    - **RG13.5.5** **DONE 2026-09-28** — [log](exudynRevisionLog2026b.md#rg13-5-5) - sensors - the
      general sensor section, and each sensor with what is its own.

<a id="rg13-6"></a>
**RG13.6** *(group RG13; maintainer 2026-09-28)* **A MiniExample for every item** (#2732) - the goal
    RG13 was created with: the short script under *Mini example* on the page of every item, run by the
    test suite. 74 of the 97 items have none (RG13.1). RG2.3.3.5 takes every item through its
    MiniExample and waits for this. It starts with nodes, markers, loads and sensors, whose examples
    are short, and follows the pages of RG13.5 kind by kind.

    **What a MiniExample is**: [definitions/README.md](../../definitions/README.md) §*The mini
    example*.

    - **RG13.6.1** **DONE 2026-09-29** — [log](exudynRevisionLog2026b.md#rg13-6-1) - nodes (15 of
      16; `NodeGenericAE` waits for RG4.12, #2736).
    - **RG13.6.2** to **RG13.6.4** **DONE 2026-09-29** — [log](exudynRevisionLog2026b.md#rg13-6-2) -
      markers, loads, sensors: every one has a MiniExample.
    - **RG13.6.5** **DONE 2026-09-29** — [log](exudynRevisionLog2026b.md#rg13-6-5) - objects: all but
      three have a MiniExample.
    - **RG13.6.6** *(maintainer 2026-09-29: later)* `ObjectFFRF` and `ObjectFFRFreducedOrder` get their
      MiniExamples when tetrahedral finite elements are part of Exudyn itself; until then a mesh comes from
      a file or from NGsolve, and their pages name a complete model instead (`NGsolveFFRF.py`,
      `NGsolveCMStutorial.py`, `objectFFRFreducedOrderTest.py`). `ObjectBeamGeometricallyExact` waits for
      RG4.8.

<a id="rg13-7"></a>
**RG13.7** *(group RG13; maintainer 2026-09-29)* **DONE 2026-09-29** — [log](exudynRevisionLog2026b.md#rg13-7) —
    **How to set up a new item** (#2742): one developer page, `docs/dev/NEW_ITEM.md` - the definition,
    which files the generator writes for a new class name, the C++ to write by kind, the checks, what to
    run; it refers to `definitions/README.md`, `ARCHITECTURE.md` and `WORKFLOW.md` for the details.

## RG14 — Marker values computed where they are used

*(Group created by the maintainer, 2026-09-29.)* Today every connector, joint, constraint and load gets
a `MarkerData` for each of its markers, computed before it is called - positions, orientations,
velocities and full Jacobians, whether it needs them or not. The alternative: the connector or load
computes the marker values itself, with a small temporary per marker. **The big advantage is automatic
differentiation**, which then sees the whole computation from the coordinates to the force. It shapes
how future items and the user elements (RG7, RG8) are written, so it is decided first, even if it is not
done.

<a id="rg14-1"></a>
**RG14.1** *(group RG14; maintainer 2026-09-29)* **DONE 2026-09-29, for the maintainer's decision** —
    in `tmp/evalRG14_1_markerData.md` (not kept in the repository). Proposed: connectors and loads call
    marker functions with a compact temporary, after RG9.3, loads first; GeneralContact keeps its
    precomputation; the AD benefit needs the body side of RG15. **The evaluation** (#2745): what `MarkerData` holds
    and costs today, who computes and who reads which part of it (connectors, constraints, loads,
    `GeneralContact`), and the options - a smaller temporary per marker, marker functions a connector
    calls, what automatic differentiation needs from them. The result is a proposal for the maintainer:
    whether, and which option.

<a id="rg14-2"></a>
**RG14.2** *(group RG14; after RG14.1 is decided)* **The migration**: a function in `CObjectConnector`
    and `CLoad` that does what the precomputation does today, so that nothing changes; then each
    connector and load changed to compute its marker values itself, one at a time, the test suite
    unchanged after each.

## RG15 — Objects computing from given coordinates

*(Group created by the maintainer, 2026-09-29.)* A body or finite element reads its coordinates from its
nodes inside `ComputeODE2LHS` and the mass matrix. If it got the coordinates - displacements and
velocities - as arguments instead, automatic differentiation of an object would be simple. Unlike RG14
this is **a real performance question** with more cases: objects with one node (`ObjectMassPoint`,
`ObjectRigidBody`, ...) can keep linked data without copying, while finite elements and other multi-node
objects would get their coordinates from the interface.

<a id="rg15-1"></a>
**RG15.1** *(group RG15; maintainer 2026-09-29)* **DONE 2026-09-29, for the maintainer's decision** —
    in `tmp/evalRG15_1_objectCoordinates.md` (not kept in the repository). Found: five finite elements
    already gather their coordinates and use automatic differentiation for the Jacobian. Proposed: that
    pattern as the standard, the rotation parametrizations as templated functions for the rigid bodies,
    decided together with RG14. **The evaluation** (#2746): how the objects read their
    coordinates today, what passing them would cost (measured, RG5), which kinds of objects there are -
    one node with linked data, several nodes, super elements, the kinematic tree -, and what automatic
    differentiation needs. The result is a proposal for the maintainer, including whether RG14 and RG15
    are one interface.

## Next steps recommended

*A reading of the groups above, updated from time to time. It is **not** a second place where
work is planned: a step keeps its full text in its group, and this section carries its number, its
issue and a short title only. The open issues that are not steps are in the tracker, which
`exudev issue serve` reads.*

### Still open

| step | issue | what it is |
|---|---|---|
| RG1.1 | - | at 1.13: fast-forward `master`, push once with tags |
| RG1.2 | - | retroactive tags for the past releases whose commits can be identified |
| RG1.3 | - | a second internal repository for development-only Python |
| RG1.4 | - | **the 1.13 release** - the first public one after the revision |
| RG2.1 | #2562 | test the drawing code, which one test model covers today |
| RG2.2 | - | the integration round of the institute before 1.13 |
| RG2.3.3.5 | #2582, #2751 | the graphics regression test: every item through its MiniExample - the last open part of RG2.3, waits for RG13.6 |
| RG3.8.5 | #2594 | the seventeen vector originals whose png the documentation uses |
| RG4.1 | - | the Windows/Linux differences in contact and friction; RG4.1.2 the five macOS-only models |
| RG4.3 | #2398, #2400 | bring down the cost of an explicit integration step |
| RG4.8 | #2730, #736, #1100, #1273, #1494, #1499, #1550 | `ObjectBeamGeometricallyExact` (3D): the comparison reproduced, 2D and 3D tests, mass matrix, gyroscopic terms, Jacobian, reference configuration |
| RG4.14 | #2208 | `ObjectBeamGeometricallyExact2D`: a test of the 3-node element |
| RG2.4 | #2748 | the manual GUI check, per release and platform (list and model done) |
| RG4.12 | #2736 | `NodeGenericAE` cannot be used: no object, marker or script takes it - **deprecate, or an object for it?** |
| RG5.1 | - | a maintained micro-benchmark of the linear algebra, inside Exudyn (from #2397) |
| RG5.2 | - | make the hot linear algebra vectorizable |
| RG4.15 | #2750, #830, #2127, #1639, #1888, #1424, #1848, #1947 | the open bugs and fixes before 1.13 |
| RG6.8 | #1813, #2309, #2321, #2308, #2140, #2236, #2237, #2350 | the graphics fixes before 1.13, each with a test |
| RG6.7 | #2709, #2710 | GraphicsData gets a Sphere and a curved triangle list; RG6.7.1 evaluates the geometry first |
| RG8.1 to RG8.9 | - | the plugin ABI: registry, fingerprint, reference plugin, headers, discovery |
| RG9.3 | #2744 | access functions as single functions of the objects; evaluation first |
| RG10.1.1 | #2713 | exudev scripts also runs the scripts, in a local copy with a timeout, after a check for paths |
| RG12.1 | #2588 | `simulationSettings` gets the deprecation mechanism |
| RG12.2 | #2589 | let an item parameter be deprecated and renamed |
| RG12.4.7 | - | the `TPyFunction...` group type disappears from a definition (#2664 was resolved without it) |
| RG14.1 | #2745 | evaluation: connectors and loads compute their marker values themselves |
| RG15.1 | #2746 | evaluation: objects compute from coordinates passed in |
| RG13.3 | #2717 | each description synchronized once with its implementation, recorded with a fingerprint |
| RG13.5.2 | #2725 | the page of `ObjectBeamGeometricallyExact`, after RG4.8 |
| RG13.6.6 | #2732 | MiniExamples of `ObjectFFRF` and `ObjectFFRFreducedOrder`, once tetrahedral elements are part of Exudyn |

Open in the tracker without a step: #2498 (nothing checks that an item type provides the member
functions it must) and #2511 (the ROS examples were last run in 2023), both named in RG2.

### Raised by the current work, and not yet a step

Each of these is written down where it was found; none is planned, and the maintainer decides
whether it becomes a step.

| where | issue | what it is |
|---|---|---|

*Empty since 2026-09-29: #2608 was done by RG6.2.11. The decision on the chapters of the user manual
(#2657, #2662), which stood below, is carried out and is in the
[log](exudynRevisionLog2026b.md#decisions-2026-09-29).*

### Recommended next

The title of each says what the step **does**; the sentence after it says why it comes here.

1. **Run the integration round of the institute, then release 1.13** (RG2.2, RG1.4). It is
   the only item on this page that needs **other people's time**, so it starts before the
   rest is ready, not after.
2. **Write a MiniExample for every item** (RG13.6, #2732). The goal RG13 was created for, and
   the graphics regression test of every item (RG2.3.3.5) waits for it.
3. **Resolve the item defects the documentation found** (RG4.8 to RG4.11). They are on the pages
   of the items now, which is where users meet them; the 3D geometrically exact beam has no page
   until RG4.8 is done.
4. **Give `simulationSettings` the deprecation mechanism** (RG12.1, #2588). It is the one
   `visualizationSettings` already has, and RG12.2 (#2589) cannot start until both have it.
5. **Place or drop the figures that no page references** (RG3.8.5, #2594). Small, and it is
   published documentation that is visibly wrong.
