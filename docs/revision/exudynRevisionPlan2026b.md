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
| **RG16** Homogeneous transformations | `exu.HT` and the frames of items, markers and joints as one transformation |
| **RG17** Notebooks | tutorials and examples as notebooks |

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
    **The GraphicsData features drawn since 1.12** are part of it (maintainer 2026-10-01; row K13, model
    `python/Examples/graphicsCurvedShapes.py`): the type `Spheres` (also `graphics.Sphere`), the 6-node triangles
    (`triangles6`) and their curved face edges, the quadratic edges (`edges3`) and lines (`Lines` with `shape`
    `'quadratic'`), the two tiling settings `openGL.advanced.curvedTriangleTilingAngle`/`curvedTriangleMaxTiling`
    changed in the running window, and all of it in the raytracer - what the graphics tests can only check as data.
    **The list and the model DONE 2026-09-29** — [log](exudynRevisionLog2026b.md#rg2-4); **the checks
    themselves** are done once per release on Windows, Ubuntu and macOS, and RG1.4 waits for them.

<a id="rg2-5"></a>
**RG2.5** **DONE 2026-10-04** (#2832) — [log](exudynRevisionLog2026b.md#rg2-5) *(group RG2; feedback of a colleague installing Exudyn, forwarded by the maintainer 2026-10-04: "The
test-suite needs matplotlib to be installed; otherwise, it fails. Should it be like that?")* **The test suite without
matplotlib**: a test model that fails for a missing package of the `[tests]` extra is skipped and listed with
`pip install exudyn[tests]`, `exudyn.misc.resultsMonitor` imports matplotlib only if it is there.

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
      - **RG2.3.3.5** **DONE 2026-09-30** — [log](exudynRevisionLog2026b.md#rg2-3-3-5) (#2751; #2704 resolved
        with .1 to .4) **every item, through its MiniExample** - `test_graphicsMiniExamples.py`, all 93; the
        images per item for the documentation are not written yet. The plan was - the
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

      - **RG2.3.3.6** *(maintainer 2026-09-30)* **DONE 2026-10-01** (#2765, #2764) — [log](exudynRevisionLog2026b.md#rg2-3-3-6) -
        **the MiniExample graphics test sees more**, and it found a drawing bug on its first run: a 3D view
        instead of the x-y plane and `view0.scene.drawWorldBasis = True`, so that a wrong transformation in
        the drawing shows as a displaced or turned item against the world basis - which only works where the
        MiniExample's positions and orientations are not all zero, so some MiniExamples get a reference
        position or rotation (their test results move and are re-recorded). The references stay as the
        test writes them.
      - **RG2.3.3.7** **CLOSED 2026-10-04**, restarted as RG3.31 (#2830) *(maintainer 2026-10-04: "most of them are too
        difficult to do them automatically")* *(maintainer 2026-09-30)* **an image per item for its page**: 800 x 600, generated
        automatically by the raytracer, the white border cropped, a 3D view; selected by hand where the image
        fits the description - the others stay without one for now. The definition of the item names the
        file (a field such as `image='itemImages/ObjectRigidBody.png'`), stored in `docs/figures/itemImages/`.
        The evaluation run of 2026-09-30 (`tmp/miniExampleImages/`) showed what the MiniExamples need first:
        the raytracer draws no spheres (RG6.7.3), most bodies have no graphics of their own, and the node
        frames and load arrows dominate.
      - **RG2.3.3.8** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg2-3-3-8) *(maintainer 2026-10-04: "Do the
        suggested next 3 steps")* *(found 2026-10-02)* **the raytracer can hang in `RedrawAndGetImage`** (#2776) - it was
        a failed multithreaded solve that left its worker threads running: twice the full
        pytest run hung in `testRaytracerImages`, all workers idle - a wait, not a loop; two later runs and a stress run
        of 8 processes did not. Candidates: the start of the `TaskManager` for the raytracer's `ParallelFor`, or a
        `TaskManager` left running by a previous test in the same worker. To find before a release; until then a hung
        test run is this, not a new failure.

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
gaps it names are the first candidates. The maintainer's own findings go here as steps. *(The steps are in number
order since 2026-10-04; the cleanup of 2026-10-03 had left some of them among RG12.)*

<a id="rg3-1"></a>
**RG3.1** **DONE 2026-09-22** (#2584) — [log](exudynRevisionLog2026b.md#rg3-1) · [plan text](exudynRevisionLog2026b.md#plan-rg3-1) — The section structure of the user manual was wrong.

<a id="rg3-2"></a>
**RG3.2** **DONE 2026-09-22** (#2585) — [log](exudynRevisionLog2026b.md#rg3-2) · [plan text](exudynRevisionLog2026b.md#plan-rg3-2) — The internal how-to notes left the published documentation.

<a id="rg3-3"></a>
**RG3.3** **DONE 2026-09-22** (#2586) — [log](exudynRevisionLog2026b.md#rg3-3) · [plan text](exudynRevisionLog2026b.md#plan-rg3-3) — Is there a PDF, and should there be?

<a id="rg3-3-1"></a>
**RG3.3.1** **DONE 2026-09-25** (#2658) — [log](exudynRevisionLog2026b.md#rg3-3-1) · [plan text](exudynRevisionLog2026b.md#plan-rg3-3-1) — `exudev docs --pdf` needed Perl and did not say so.

<a id="rg3-4"></a>
**RG3.4** **DONE 2026-09-22** (#2587) — [log](exudynRevisionLog2026b.md#rg3-4) · [plan text](exudynRevisionLog2026b.md#plan-rg3-4) — The revisions chapter says where the details are.

<a id="rg3-5"></a>
**RG3.5** **DONE 2026-09-22** (#2550) — [log](exudynRevisionLog2026b.md#rg3-5) · [plan text](exudynRevisionLog2026b.md#plan-rg3-5) — The citations point nowhere.

<a id="rg3-6"></a>
**RG3.6** **DONE 2026-09-22** (#2592) — [log](exudynRevisionLog2026b.md#rg3-6) · [plan text](exudynRevisionLog2026b.md#plan-rg3-6) — A generated settings page shows a table with no rows.

<a id="rg3-7"></a>
**RG3.7** **DONE 2026-09-22** (#2593) — [log](exudynRevisionLog2026b.md#rg3-7) · [plan text](exudynRevisionLog2026b.md#plan-rg3-7) — Display math opened at the end of a text line swallows the text.

<a id="rg3-8"></a>
**RG3.8** **DONE 2026-10-03** (#2594) — [log](exudynRevisionLog2026b.md#rg3-8-1) · [plan text](exudynRevisionLog2026b.md#plan-rg3-8) — The unreferenced figures are the trace of figures the conversion lost. (all sub-steps done; the figures are back and vector where an original exists)

<a id="rg3-9"></a>
**RG3.9** **DONE 2026-09-23** (#2598) — [log](exudynRevisionLog2026b.md#rg3-9) · [plan text](exudynRevisionLog2026b.md#plan-rg3-9) — Three corrections to the landing pages and the developer chapters.

<a id="rg3-10"></a>
**RG3.10** **DONE 2026-09-24** (#2599) — [log](exudynRevisionLog2026b.md#rg3-10) · [plan text](exudynRevisionLog2026b.md#plan-rg3-10) — `CHANGELOG.md` and the issue tracker page hold the same list twice.

<a id="rg3-10-1"></a>
**RG3.10.1** **DONE 2026-09-24** (#2637) — [log](exudynRevisionLog2026b.md#rg3-10-1) · [plan text](exudynRevisionLog2026b.md#plan-rg3-10-1) — The changelog and the tracker page printed the same issue in two formats.

<a id="rg3-11"></a>
**RG3.11** **DONE 2026-09-23** (#2611) — [log](exudynRevisionLog2026b.md#rg3-11) · [plan text](exudynRevisionLog2026b.md#plan-rg3-11) — "The C++ core" pointed at the repository instead of at the documentation.

<a id="rg3-12"></a>
**RG3.12** **DONE 2026-09-24** (#2646) — [log](exudynRevisionLog2026b.md#rg3-12-1) · [plan text](exudynRevisionLog2026b.md#plan-rg3-12) — Building from source and the development workflow are told three times and never from the start.

<a id="rg3-13"></a>
**RG3.13** **DONE 2026-09-24** (#2648, #2646) — [log](exudynRevisionLog2026b.md#rg3-13) · [plan text](exudynRevisionLog2026b.md#plan-rg3-13) — The tree told the reader about the revision instead of about itself.

<a id="rg3-13-1"></a>
**RG3.13.1** **DONE 2026-09-27** (#2649) — [log](exudynRevisionLog2026b.md#rg3-13-1) · [plan text](exudynRevisionLog2026b.md#plan-rg3-13-1) — 235 references to the plan are left in comments, each inside a sentence.

<a id="rg3-14"></a>
**RG3.14** **DONE 2026-09-26** (#2655) — [plan text](exudynRevisionLog2026b.md#plan-rg3-14) — The item and settings descriptions are written in LaTeX.

<a id="rg3-15"></a>
**RG3.15** **DONE 2026-09-25** (#2657, #2661, #2662) — [log](exudynRevisionLog2026b.md#rg3-15) · [plan text](exudynRevisionLog2026b.md#plan-rg3-15) — The chapters of the user manual.

<a id="rg3-16"></a>
**RG3.16** **DONE 2026-09-25** (#2662) — [log](exudynRevisionLog2026b.md#rg3-16) · [plan text](exudynRevisionLog2026b.md#plan-rg3-16) — Every heading is sentence case.

<a id="rg3-17"></a>
**RG3.17** **DONE 2026-09-25** (#2663) — [log](exudynRevisionLog2026b.md#rg3-17) · [plan text](exudynRevisionLog2026b.md#plan-rg3-17) — A comment in a description is an HTML comment.

<a id="rg3-18"></a>
**RG3.18** **DONE 2026-09-26** (#2660) — [log](exudynRevisionLog2026b.md#rg3-18) · [plan text](exudynRevisionLog2026b.md#plan-rg3-18) — The pages of the Python-C++ interface repeat their own title, and it costs the MainSystem extensions their place in the table of contents.

<a id="rg3-19"></a>
**RG3.19** **DONE 2026-09-26** (#2665) — [log](exudynRevisionLog2026b.md#rg3-19) · [plan text](exudynRevisionLog2026b.md#plan-rg3-19) — The arguments of a documented function are one per line, with the name in code.

<a id="rg3-21"></a>
**RG3.21** **DONE 2026-09-27** (#2673) — [log](exudynRevisionLog2026b.md#rg3-21) · [plan text](exudynRevisionLog2026b.md#plan-rg3-21) — The pages that still describe the state before a step that is done.

<a id="rg3-22"></a>
**RG3.22** **DONE 2026-09-27** (#2659) — [log](exudynRevisionLog2026b.md#rg3-22) · [plan text](exudynRevisionLog2026b.md#plan-rg3-22) — The simulation settings section says how to look a setting up.

<a id="rg3-23"></a>
**RG3.23** **DONE 2026-09-26** (#2680) — [log](exudynRevisionLog2026b.md#rg3-23) · [plan text](exudynRevisionLog2026b.md#plan-rg3-23) — The override settings are documented where the module is.

<a id="rg3-24"></a>
**RG3.24** **DONE 2026-09-27** (#2681) — [log](exudynRevisionLog2026b.md#rg3-24) · [plan text](exudynRevisionLog2026b.md#plan-rg3-24) — The generator API still says "Latex".

<a id="rg3-25"></a>
**RG3.25** **DONE 2026-09-27** (#2683) — [log](exudynRevisionLog2026b.md#rg3-25) · [plan text](exudynRevisionLog2026b.md#plan-rg3-25) — a TAB instead of a backslash put `exttt{...}` on three pages of the Symbolic manual.

<a id="rg3-26"></a>
**RG3.26** **DONE 2026-09-27** (#2697) — [log](exudynRevisionLog2026b.md#rg3-26) · [plan text](exudynRevisionLog2026b.md#plan-rg3-26) — `index.md` and `pdfIndex.md` are two hand-written tables of contents that must agree - the maintainer chose option B, the check.

<a id="rg3-27"></a>
**RG3.27** **DONE 2026-09-27** (#2708) — [log](exudynRevisionLog2026b.md#rg3-27) · [plan text](exudynRevisionLog2026b.md#plan-rg3-27) — The mass-spring-damper tutorial comes first.

<a id="rg3-28"></a>
**RG3.28** **DONE 2026-09-29** (#2743) — [log](exudynRevisionLog2026b.md#rg3-28) · [plan text](exudynRevisionLog2026b.md#plan-rg3-28) — The developer documentation named paths of the old `main/` directory.

<a id="rg3-29"></a>
**RG3.29** **DONE 2026-10-03** (#2808) — [log](exudynRevisionLog2026b.md#rg3-29) · [plan text](exudynRevisionLog2026b.md#plan-rg3-29) — The Python-C++ command interface in sections.

<a id="rg3-30"></a>
**RG3.30** **DONE 2026-10-03** (#2812) — [log](exudynRevisionLog2026b.md#rg3-30) · [plan text](exudynRevisionLog2026b.md#plan-rg3-30) — The flow charts as TikZ again.

<a id="rg3-31"></a>
**RG3.31** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg3-31) *(group RG3; maintainer 2026-10-04: "I think that most of them are too difficult to do them automatically
... check which items (mostly bodies, loads, joints) would make sense to have a representative image - and which ones
do not yet have one")* **Representative images for the item pages** (#2830), in place of RG2.3.3.7 (an image per item,
generated automatically from its MiniExample). A few images, made by hand-written scripts (one or two, or a test model
where one fits), for the items whose page gains from a picture.
    - **RG3.31.1** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg3-31-1) *the status and a list, for the
      maintainer's choice*: of the 97 item pages, five have a rendered image of the item (`ObjectRigidBody`,
      `ObjectJointGeneric`, `ObjectJointRevoluteZ`, `ObjectJointPrismaticX`, `ObjectJointSpherical`) and eight a sketch
      of its quantities (FFRF, rolling disc, convex roll, curve-circles, the circle-cable and sphere contacts, ALE moving joint,
      `MarkerSuperElementRigid`). **Proposed** - a picture says what the item is or does at one look:
        - bodies: `ObjectGround` (a checkerboard with a basis), `ObjectMassPoint` (a sphere on a spring), `ObjectRigidBody2D`,
          `ObjectKinematicTree` (a 3-link arm with its joint axes), `ObjectANCFCable2D` and `ObjectBeamGeometricallyExact`
          (a bent cantilever), `ObjectANCFThinPlate` (a deformed plate), `ObjectFFRFreducedOrder` (a flexible body with
          stresses, from an NGsolve example);
        - loads: `LoadForceVector` and `LoadTorqueVector` (arrows on a body), `LoadMassProportional` (gravity on a body);
        - connectors: `ObjectConnectorSpringDamper`, `ObjectConnectorCartesianSpringDamper`,
          `ObjectConnectorRigidBodySpringDamper`, `ObjectConnectorTorsionalSpringDamper`, `ObjectConnectorDistance`,
          `ObjectConnectorReevingSystemSprings` (a rope over sheaves), `ObjectConnectorRollingDiscPenalty` and
          `ObjectJointRollingDisc` (a wheel on the ground), `ObjectJointRevolute2D`, `ObjectJointSliding2D` (a mass sliding
          on a cable);
        - contacts: `ObjectContactSphereSphere` (beside the sketch it has);
        - markers: `MarkerBodyRigid` (a body with the marker frame, `localHT`), the only marker whose frame is worth a picture.
      **Not proposed**: the generic nodes and objects (`NodeGeneric*`, `ObjectGenericODE1/2`), the coordinate markers and
      constraints, `ObjectConnectorCoordinate*`, `ObjectContactCoordinate`, the sensors, `LoadCoordinate`, the 1D masses -
      what they do is a number, not a shape.
    - **DECIDED 2026-10-04** (maintainer): the list as proposed, `ObjectRigidBody2D` "same as ObjectRigidBody but in a
      planar view", `ObjectFFRFreducedOrder` from the FFRF tutorial; "for the raytracer use shadows on, a light position
      [,,,1] to have a positional light with lightRadiusVariations=21".
    - **RG3.31.2** **DONE 2026-10-04** after the choice: one or two scripts in `tools/itemImages/` (or a test model where one fits) that build
      each chosen item in a small scene and write its image with the raytracer (`SC.renderer.RedrawAndGetImage(
      useRaytracer=True)`, no window), 800 x 600, white border cropped; the **non-simplified** drawing modes (springs as
      helices, the basis vectors of markers and nodes as arrows, joints with their axes); view, light and material
      adjusted by hand, image by image;
    - **RG3.31.3** **DONE 2026-10-04** the images in `docs/figures/`, each named on the page of its item (`{image}` in the
      `detailedDescription`, as the joints do), and a look at every page.
    - **RG3.31.5** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg17-2-6) *(maintainer 2026-10-04)* the images
      of the rigid body and the joints move from the overall description (where they also went into the docstrings)
      to the field `image`; `ObjectJointRevoluteZ` shows only the joint between two bodies (`RevoluteJointZ2.png`
      renamed to `RevoluteJointZ.png`, the other removed); `ObjectRigidBody` gets an image of `itemImages.py`; the
      maintainer's settings of the images (1080 x 700, lightRadiusVariations 41, ...) kept and all images drawn
      again; `itemImages.py` sets the window flags itself, so that the FFRF scene opens no window also in a session
      that imported exudyn before.
    - **RG3.31.4** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg3-31-4) *(maintainer 2026-10-04: "fix
      RG3.31.4")* (#2837) a 3D beam without polygonal section geometry is drawn as a line with an orthonormal
      basis at every tile (`UpdateGraphicsBeam3D`, marked "temporary!" in the code), found with the image of
      `ObjectBeamGeometricallyExact`.

<a id="rg3-32"></a>
**RG3.32** **DONE 2026-10-04** (#2834) — [log](exudynRevisionLog2026b.md#rg3-32) *(group RG3; feedback of a colleague installing Exudyn, forwarded by the maintainer 2026-10-04:
"\begin{aligned} ended with \end{split} ... should be fixed globally")* **A display formula has no blank line**: the
converter removes the lines a LaTeX comment leaves inside `$$...$$`, `docs/manual/solver.md` lost seven blank lines, and
`checkMathMacros --check` refuses a blank line inside display math.

<a id="rg3-33"></a>
**RG3.33** **DONE 2026-10-04** (#2835) — [log](exudynRevisionLog2026b.md#rg3-32) *(group RG3; feedback of a colleague installing Exudyn, forwarded by the maintainer 2026-10-04: "reads
strange => remove")* **The start page says nothing about how the documentation is made**: the paragraph on the
hand-written table of contents left `index.md`; `docs/dev/README.md` names `index.md` in the repository layout.

<a id="rg3-34"></a>
**RG3.34** **DONE 2026-10-05** (#2852) — [log](exudynRevisionLog2026b.md#rg3-34) *(group RG3; maintainer 2026-10-05: "This
remark shall appear only once after the first code cell of a notebook ... smaller and directly under the cell ...
only show the filename")* **The pages of the Python-C++ interface name the notebook of an example once**: after its
first code cell on the page, as its file name, small and close under the cell.
    - **RG3.34.1** **DONE 2026-10-05** — [log](exudynRevisionLog2026b.md#rg3-34-1) *(maintainer 2026-10-05: "put the
      filename as typewriter ... and add the path")* the remark is the path of the notebook, in typewriter; the
      notebook snippets of the manual pages end with the same small line.

<a id="rg3-35"></a>
**RG3.35** **DONE 2026-10-05** (#2853, #2854, #2855) — [log](exudynRevisionLog2026b.md#rg3-35) *(group RG3; maintainer
2026-10-05: "Several issues in the PDF docs, many related to tables")* **The tables of the PDF**: every table in a
section breaks across pages instead of running into the footer, a table without widths of its own gets them from its
content (the notation tables, the definitions of quantities, the parameter tables of the items); the settings tables
lose the size column and get short headings and a wider name column; the acceleration row of the FFRF output
variables is one row again; the items index lists every renamed and deprecated item parameter.

<a id="rg3-36"></a>
**RG3.36** *(group RG3; maintainer 2026-10-05: "add 'A PDF check in the docs gate' as a step")* **The PDF is checked
    with the documentation** (#2856). `exudev docs` builds the html; the PDF (`exudev docs --pdf`, about 2.5 min) is
    a release artifact, so what RG3.35 fixed - tables running into the footer, a column one word wide, a table row
    broken over lines - showed only to a reader of the PDF. Proposal: `exudev docs --pdf --check`, run as a gate
    when the documentation changed (or before a release), that fails on a LaTeX error and on a new warning of the
    LaTeX log - overfull boxes, undefined references, missing figures - against a baseline kept in the repository,
    and reports the pages. Open: whether it runs with every docs gate (+2.5 min) or only before a commit that touches
    `docs/`, `definitions/` or `conf.py`.


## RG4 — Implementation problems and bugs

Problems that are real, reproducible, and too deep to fix in passing. They are recorded here
rather than worked around silently, so the debt stays visible and each item can be closed on
evidence. An ordinary bug goes into the issue tracker and is fixed; a step appears here when the
fix needs a plan of its own.

The steps are numbered in the order they were raised and stand here in the order of their numbers.

<a id="rg4-1"></a>
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
    - **RG4.1.3** *open (maintainer 2026-10-01)* — **the math library and uninitialized values**, two candidate
      causes to test. What is known (IEEE 754-2008/2019, the glibc manual *Errors in Math Functions*, the MSVC `/fp`
      and GCC/clang `-ffp-contract` documentation; from the standard literature, not re-fetched here):
      - **`sqrt` is the same everywhere**: IEEE 754 requires it correctly rounded, and x86-64 (`sqrtsd`) and ARM64
        (`fsqrt`) do that in hardware. It can only differ through a compiler that replaces it by an approximation
        (`-ffast-math`, reciprocal square root), which no Exudyn build uses.
      - **`atan2`, `sin`, `cos`, `exp`, `log`, `pow`, `acos`, `tanh` are not**: IEEE 754 only *recommends* correct
        rounding for them, and glibc's libm, the MSVC UCRT and Apple's libm are separate implementations that
        each guarantee about 1 ULP and differ in the last bit for some arguments. One ULP that decides a contact
        state or a friction branch is enough for differences like the 1e-5..1e-3 of the table above.
      - **FMA contraction**, closely related: clang contracts `a*b+c` into one rounded operation by default
        (`-ffp-contract=on`), and **ARM64 always has the instruction** - so the macOS ARM build contracts, while the
        Windows build does not (MSVC does not contract) and the Linux x86-64 baseline cannot (no FMA in the baseline
        ISA; `setup.py` adds `-ffp-contract=off` for the fast module only). A strong candidate for the five
        macOS-only models (RG4.1.2) - the first thing to try is `-ffp-contract=off` in the darwin options.
      - **the test for the math functions**, as proposed by the maintainer: route the transcendental calls through
        one header (`EXUmath::Atan2`, ...) with a switch to own portable implementations (fdlibm/openlibm-like, or a
        correctly rounded one), build both platforms with it, and rerun the suite; the entries of
        `UnresolvedOnLinux()` that vanish were caused by it. First count which of these functions the
        affected models actually reach (contact, friction regularization, rotation parameters).
      - **uninitialized values**: `SlimVectorBase`, `SlimArray`, `ConstSizeVectorBase` and `ConstSizeMatrixBase` have
        `= default` constructors, so `Vector3D v;` leaves the values uninitialized on **both** compilers (only a
        value-initialization `Vector3D v{}` zeroes them), and so do member variables without an initializer. What
        such a read returns is whatever the stack held, which differs between compilers, libraries and
        optimization - a classic platform difference. The test: a build switch that fills default-constructed
        linalg objects with NaN, and the suite run with it (a NaN in a result names a read before a write); on
        Linux additionally `valgrind --tool=memcheck` or clang's `-fsanitize=memory` on the affected models, and
        `-Wmaybe-uninitialized` at `-O2`.
      - not yet checked, a third candidate of the same kind: iteration order of unordered containers and
        non-stable sorts with equal keys, which differ between libstdc++, libc++ and the MSVC library.

<a id="rg4-2"></a>
**RG4.2** **DONE 2026-09-26** (#2413) — [log](exudynRevisionLog2026b.md#rg4-2) · [plan text](exudynRevisionLog2026b.md#plan-rg4-2) — `ObjectContactConvexRoll.pContact` is a computed value that Python reads.

<a id="rg4-3"></a>
**RG4.3** **DONE 2026-10-02** (#2398) — [log](exudynRevisionLog2026b.md#rg4-3) · [plan text](exudynRevisionLog2026b.md#plan-rg4-3) — Explicit integration cost.

<a id="rg4-4"></a>
**RG4.4** **DONE 2026-09-23** (#2603) — [log](exudynRevisionLog2026b.md#rg4-4) · [plan text](exudynRevisionLog2026b.md#plan-rg4-4) — Two lines of Python segfault the process.

<a id="rg4-5"></a>
**RG4.5** **DONE 2026-09-23** (#2616) — [log](exudynRevisionLog2026b.md#rg4-5) · [plan text](exudynRevisionLog2026b.md#plan-rg4-5) — Quitting the renderer before a simulation started raised, quitting during it did not.

<a id="rg4-6"></a>
**RG4.6** **DONE 2026-09-28** (#2674, #2616) — [log](exudynRevisionLog2026b.md#rg4-6) · [plan text](exudynRevisionLog2026b.md#plan-rg4-6) — A test hook for `forceQuitSimulation` - decided for a binding a user can use as well, `SC.renderer.StopSimulation(forceQuit=True)`.

<a id="rg4-7"></a>
**RG4.7** **CLOSED 2026-09-29, done by revision2026 step R6.3.5** (#2423) — [plan text](exudynRevisionLog2026b.md#plan-rg4-7) — Every C++ user error inspects the Python source for its file and line.

<a id="rg4-8"></a>
**RG4.8** **DONE 2026-09-30** (#2730) — [plan text](exudynRevisionLog2026b.md#plan-rg4-8) — `ObjectBeamGeometricallyExact` (3D): analyse the implementation.
    Two leftovers, decided by the maintainer on 2026-09-30:
    - **RG4.8.13** **DONE 2026-10-01** (#2762) — [log](exudynRevisionLog2026b.md#rg4-8-13) - `rightAngleFrame.py` (ANCF and this element, not run by the suite) stops near the
      buckling load with its load-driven settings: drive it by displacement instead - a coordinate constraint
      whose offset a user function prescribes;
    - **RG4.8.14** **DONE 2026-10-01** (#2761) — [log](exudynRevisionLog2026b.md#rg4-8-14) - the switch of the mass matrix stays, as a special setting and not an experimental one:
      `exu.special.beams.geometricallyExactLumpedMass` (default False, the consistent mass), so that tests
      can run both; `exu.experimental.beamGeometricallyExactConsistentMass` goes.

<a id="rg4-9"></a>
**RG4.9** **DONE 2026-09-29** (#2731) — [log](exudynRevisionLog2026b.md#rg4-9) · [plan text](exudynRevisionLog2026b.md#plan-rg4-9) — The cable and beam shape markers accept any body.

<a id="rg4-10"></a>
**RG4.10** **DONE 2026-09-29** (#2734) — [log](exudynRevisionLog2026b.md#rg4-10) · [plan text](exudynRevisionLog2026b.md#plan-rg4-10) — `ObjectGenericODE2` and `ObjectKinematicTree` admit the general body markers, which then fail.

<a id="rg4-11"></a>
**RG4.11** **DONE 2026-09-29** (#2735) — [log](exudynRevisionLog2026b.md#rg4-10) · [plan text](exudynRevisionLog2026b.md#plan-rg4-11) — `ObjectContactCoordinate` ignores `activeConnector`, and its output variable `Distance` raises.

<a id="rg4-12"></a>
**RG4.12** **CLOSED 2026-10-02** (maintainer: closed, so it stays visible; a case that needs it re-opens the topic with a
    new issue) *(group RG4; from RG13.6.1, 2026-09-29)* **`NodeGenericAE` cannot be used** (#2736): it
    provides only `GenericAE`, no object requests that type, no node marker attaches to it, and no
    example, test model or module of the package uses it - a node with algebraic coordinates and no
    object to write their equations. Either an object takes it (its description names linear state
    space systems) or it is deprecated. Its page has no MiniExample until then.

    **ON HOLD** (maintainer, 2026-09-29): the node stays, not deprecated - *"It will be used in the
    future"*. What it is for, as the maintainer describes it and as it would be built:
    - **(a) the owner of the Lagrange multipliers of a constraint.** Today a constraint's multipliers are
      AE coordinates allocated automatically, owned by no node, and nothing but the solver can address
      them. A constraint or joint gets an **optional** `NodeGenericAE` with as many coordinates as it has
      algebraic equations; if one is given, the object's AE coordinates are the node's, otherwise they are
      allocated as today - so every existing model stays as it is. The multipliers then become usable
      like any node coordinate: a sensor reads them, and another object can write a relation for them -
      the same as `ObjectJointGeneric` leaving a direction free, but said as $\lambda_2 = 0$;
    - **(b) the unknowns of purely algebraic equations** that an object writes for them: an implicit
      function, e.g. the geometry of a sliding joint on a complicated element, solved together with the
      system instead of in a local iteration. This needs an object that writes the residuals - a C++
      element with internal unknowns, or a generic algebraic object with a residual user function and a
      numerical Jacobian.

    Before either: the assembly must map a constraint's AE equations onto a node's coordinates (today it
    allocates them per object), a marker and a sensor must reach AE coordinates, and each solver must be
    checked for AE coordinates owned by a node (the explicit solvers do not take AE equations at all).
    (a) comes first: it has a test with a known answer - a joint gives the same result with and without
    the node. Its page has no MiniExample until then.

<a id="rg4-13"></a>
**RG4.13** **DONE 2026-09-29** (#2740) — [log](exudynRevisionLog2026b.md#rg4-13) · [plan text](exudynRevisionLog2026b.md#plan-rg4-13) — `ObjectKinematicTree`: the position Jacobian of a prismatic joint rotates the axis twice.

<a id="rg4-14"></a>
**RG4.14** **DONE 2026-09-30** (#2208) — [log](exudynRevisionLog2026b.md#rg4-14) · [plan text](exudynRevisionLog2026b.md#plan-rg4-14) — `ObjectBeamGeometricallyExact2D`: a test of the 3-node element.

<a id="rg4-15"></a>
**RG4.15** *(group RG4; maintainer 2026-09-29)* **The open bugs and fixes before 1.13.** The maintainer: *"Before
    the upcoming release, we definitely should try to resolve the open BUGs"*, and the urgent FIX issues.
    Checked 2026-09-29, each against the code or with a run - see the [log](exudynRevisionLog2026b.md#rg4-15).
    Closed as resolved or no longer applying: #738, #1048, #1772, #1846 (duplicate of #1845), #1889.
    The graphics ones are RG6.8. What remains, in the order proposed:
    - **RG4.15.1** **DONE 2026-09-29** (#2749) — one drop, three contact objects:
      the test model `contactComparisonTest.py`, the check in compensation for #738;
    - **RG4.15.2** **DONE 2026-09-29** (#2750) — [log](exudynRevisionLog2026b.md#rg4-15-2) — `ObjectContactCoordinate` gets the contact law of `ObjectContactSphereSphere` -
      `contactStiffnessExponent`, `restitutionCoefficient`, `impactModel`, `minimumImpactVelocity` -, and the
      comparison test extends to them; its release step size is also the one difference the test found;
    - **RG4.15.3** **DONE 2026-09-30** (#830) — [log](exudynRevisionLog2026b.md#rg4-15-3) — the explicit
      solvers do no post Newton step: a warning at the start of an explicit solve names the objects that are
      not updated - **wrong, and removed by RG4.16.1** (#2755): the explicit solvers do the step;
    - **RG4.15.4** **DONE 2026-09-30** (#2127) — [log](exudynRevisionLog2026b.md#rg4-15-3) —
      `ObjectContactSphereTorus`: momentum conservation - the torque of the normal force on the ring was
      missing without friction; test model `contactSphereTorusMomentumTest.py`;
    - **RG4.15.5** **CLOSED 2026-09-30** (#1639) a repeated `mbs.SolveDynamic` with `ObjectFFRFreducedOrder`
      diverges - not reproduced with `objectFFRFreducedOrderTest.py`, `superElementRigidJointTest.py` and
      `abaqusImportTest.py`, each solved again: identical results; old and probably fixed (maintainer);
    - **RG4.15.6** **DONE 2026-09-29** (#1888) — [log](exudynRevisionLog2026b.md#rg4-15-2) — `mbs.GetDictionary()` works with a symbolic user function, but
      `mbs.SetDictionary()` of that dictionary fails (*"Unable to cast ... symbolic.UserFunction"*);
    - **RG4.15.7** **DONE 2026-09-30** (#1424) — [log](exudynRevisionLog2026b.md#rg4-15-3) — the numerical
      ODE1 Jacobian with coordinates an object addresses twice: each column once, as for ODE2; test model
      `genericODE1duplicateNodeTest.py`;
    - **RG4.15.8** **DONE 2026-10-04** (#1848, #1947) — [log](exudynRevisionLog2026b.md#rg4-15-8) *(maintainer
      2026-10-04: "Do the next solo steps")* `GeneralContact`, implicit sphere-triangle contact, as the fourth case of
      the drop, and a sliding ball against the rigid-body solution: it agrees; the friction of the implicit solver
      now keeps the sliding/sticking state its active set gives, as `ObjectContactSphereSphere` does;
    - **RG4.15.9** **DONE 2026-10-05** — [log](exudynRevisionLog2026b.md#rg4-15-9) (#2848) `GeneralContact` gets the
      setting `keepContactWhilePenetrating`: a sphere-sphere or sphere-triangle contact then acts while the bodies
      penetrate, as the contact objects do; by default it acts only while its force presses, as before. The fifth
      case of `contactComparisonTest.py`.
    - **RG4.15.10** **DONE 2026-10-05** — [log](exudynRevisionLog2026b.md#rg4-15-10) (#2849) `GeneralContact`: the
      contact force of a sphere-triangle contact gives the triangle body its torque also without friction; test model
      `generalContactTriangleMomentumTest.py`.

    After 1.13, not urgent: #1845 (`ComputePostProcessingModes` with threads), #1565
    (`InitializeFromRestartFile`), #2326 (the slider crank benchmark after the revised IFToMM model);
    #2109 (DOPRI5 step size at discontinuities) goes with RG4.16.

<a id="rg4-16"></a>
**RG4.16** **DONE 2026-09-30** (#2754, #830, #2109) — [log](exudynRevisionLog2026b.md#rg4-16-1) · [plan text](exudynRevisionLog2026b.md#plan-rg4-16) — Explicit solvers and the states of the PostNewton step.


<a id="rg4-17"></a>
**RG4.17** *(group RG4; maintainer 2026-10-01, from RG4.8.13)* **`ObjectANCFBeam`: the Newton iteration stalls in the
    right-angle frame** (#2763). `rightAngleFrame.py` with `useGeometricallyExact = False`, driven by displacement:
    from load step 7 on, Newton stagnates at a relative error of 1e-7 to 3e-7 against the tolerance 1e-8 (with 1e-6:
    at 1e-6 to 2e-6), and the static solver stops at 3 % of the drive; the geometrically exact beam takes 4.7
    iterations per step. Linear convergence at a floor points at an **inconsistent Jacobian**: the first check is
    the analytic Jacobian of the element against a numerical one in a deformed, twisted state (as RG4.8.5 did for
    the geometrically exact beam), then the corner - a `GenericJoint` between two slope nodes, whose rotation the
    marker derives from the slopes.
    - **RG4.17.1** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg4-17-1) - **the corner: the rotation Jacobian of
      the slope nodes was not the derivative of their rotation** - found and made consistent (`NodePointSlope23`, and
      `ObjectANCFBeam` of RG9.3.7, through one set of functions); C_q of the corner joint now equals the numerical one.
      The stall is **not** gone: it remains with the numerical system Jacobian too, so it is no longer an
      inconsistent Jacobian.
    - **RG4.17.2** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg4-17-2) *(maintainer 2026-10-04: "Do the
      next solo steps")* **the stall analysed: no error in the model or the solver, but a very small region of Newton
      convergence of the ANCF frame** at $P \approx 0.31$ N ($t = 0.033$), the same for 8 or 16 elements, penalty
      factor 0.1 to 10, sparse or dense solver, 200 or 800 load steps; without the imperfection the run ends at $t=1$.
      The Jacobian is consistent in all blocks, the constrained tangent is positive definite (smallest eigenvalue
      0.0018 - no bifurcation), its condition 5.5e12 is that of the geometrically exact beam; a Newton iteration by hand
      with equilibrated dense solves, line search and iterative refinement stalls alike. A single cantilever (lateral
      buckling, torsion $L/GJ$) agrees with the geometrically exact beam.
    - **RG4.17.3** **DONE 2026-10-05** — [log](exudynRevisionLog2026b.md#rg4-19-decisions) *(maintainer 2026-10-05:
      "(a) document in the script and issue; and close the case")* `rightAngleFrame.py` keeps the geometrically exact
      beam, and its Details say where and why the ANCF variant stops; #2763 closed with the same text. Not taken:
      (b) a better predictor, (c) step reduction or arc length after a stall, (d) the strain measures of the ANCF beam.

<a id="rg4-18"></a>
**RG4.18** **DONE 2026-10-02** (#2790) — [log](exudynRevisionLog2026b.md#rg4-18) · [plan text](exudynRevisionLog2026b.md#plan-rg4-18) — A system without coordinates is solved.

<a id="rg4-19"></a>
**RG4.19** *(group RG4; maintainer 2026-10-04: "evaluate the currently open issues. In particular bugs, fix, check, etc.
    issues that would indicate that something should be corrected or resolved. List them as options in a new step")*
    **The open BUG, FIX and CHECK issues, evaluated: options.** 40 were open; each was checked against the code, a
    few with a run - see the [log](exudynRevisionLog2026b.md#rg4-19). Resolved as done in the meantime: #124, #209,
    #380, #390, #613, #1192, #1395, #1500. The maintainer's decisions of 2026-10-05 are in the
    [log](exudynRevisionLog2026b.md#rg4-19-decisions). The options, small first:
    - **RG4.19.1** **DONE 2026-10-05** — [log](exudynRevisionLog2026b.md#rg4-19-1) (#984) an inactive
      `RigidBodySpringDamper`, `LinearSpringDamper` and `TorsionalSpringDamper` reports no force or torque; test model
      `inactiveConnectorForceTest.py`;
    - **RG4.19.2** **DONE 2026-10-05** — [log](exudynRevisionLog2026b.md#rg4-19-1) (#1512) the `GeneralContact` of
      `AddGeneralContact` / `GetGeneralContact` keeps its system alive (`reference_internal`); the same question for
      a `MainSystem` and its `SystemContainer` is #2851;
    - **RG4.19.3** **DONE 2026-10-05** — [log](exudynRevisionLog2026b.md#rg4-19-1) (#121) every allocation of
      `ResizableArray` goes through one function that catches `bad_alloc`, as `Vector` and `Matrix` do.
    - **RG4.19.4** *(option, medium)* (#1337) a singular system Jacobian in `CSolverBase::Newton` is a `SysError`; with
      `adaptiveStep` it could be a failed step that is reduced. It also bears on RG4.17.3 (c).
    - **RG4.19.5** *(option, a run first)* (#1290) `ObjectContactFrictionCircleCable2D` shows tangential forces with
      all friction stiffness and damping zero.
    - **RG4.19.6** *(option, a run first)* (#692) the transposed `AE_ODE2_t` block of the system Jacobian for
      velocity-level constraints against a numerical one (a rolling disc).
    - **RG4.19.7** **DECIDED 2026-10-05** (#2130 closed) the contact objects stay as they are - in contact while the
      bodies penetrate, so that the damping may pull: that is what the restitution models of `impactModel` assume,
      and the linear spring-damper law is the same idea. `GeneralContact` gets a setting to do the same (RG4.15.9).
    - **RG4.19.8** **DECIDED 2026-10-05** (#1565 closed, superseded by #2850) `InitializeFromRestartFile` is no longer
      public; how a restart works is RG12.39.
    - **RG4.19.9** **DECIDED 2026-10-05** closed #171, #172, #173, #174, #436 (not worth a change), and #1677, #1682,
      #1683, #1684, #1956 (done by the revisions: `GetDictionary` of the system, the `Inspect`/`Compute` functions of
      `MainSystem`, the item pages); #1681 abandoned (a wrong name). Kept: #142, #591, #1167, #1247, #1740, #1776,
      #1910, and #1920 - a check to investigate before it becomes an extension.
    - **RG4.19.10** *(option, a check)* (#2851) a `MainSystem` does not keep its `SystemContainer` alive
      (`AddSystem` returns with `return_value_policy::reference`), found with RG4.19.2.
    - Not here, they need a Linux machine or a screen: #2204, #2205 (perspective), #2277, #2278 (GLFW on Linux) - RG6.8.

<a id="rg4-20"></a>
**RG4.20** *(group RG4; maintainer 2026-10-05: "a modification from an internal colleague the ANCFThinPlate element. He
    checked everything with the literature ... add a step and issue for that and, ideally, immediately integrate the
    changes")* **`ObjectANCFThinPlate` and `exudyn.shells`: the revision of a colleague, ported** (#2857, #2858,
    #2859). Michael Pieber revised the element in a working copy based on a state of 2026-06-01 and documented it for
    the port - description, patch, the changed files, a verification script and 19 unit tests, in
    `tmp/shells/changesANCFThinPlate2026/` (not in the repository). The port re-applies what the repository changed since
    that state: the renames of 1.12, the access functions, the energies, the outputs Director1/2.
    - **RG4.20.1** **DONE 2026-10-05** — [log](exudynRevisionLog2026b.md#rg4-20) (#2857) the element: material curvature
      measure, membrane and bending integrated at separate points (mode 0 Gauss 5 x 5; 1 and 2 Lobatto 3 x 3 / Gauss
      2 x 2), 12 thickness values and the stiffness from the local thickness, Kelvin-Voigt damping
      (`stiffnessProportionalDamping`, `bendingStiffnessProportionalDamping`), Jacobian by automatic differentiation of
      coordinates and velocities, outputs through the thickness; the consistency check takes 12 thickness values;
      test model `ANCFThinPlateRevisionTest.py`.
    - **RG4.20.2** **DONE 2026-10-05** — [log](exudynRevisionLog2026b.md#rg4-20) (#2858) `exudyn.shells`:
      `ANCFThinPlateBuilder`, the geometry maps, the constraint, load and hinge functions, ShellMesh with damping and a
      thickness function; the names in UpperCamelCase.
    - **RG4.20.3** *(open)* (#2859) the visualization: `drawNormal`, the postprocessed contour, `contourZeta`, the
      finer tiling.
    - **RG4.20.4** *(decisions, the port notes of the colleague)*: (a) note 4 - the first component of
      `CurvatureLocal` and `TorqueLocal` is negated, so that a plate strip along x has the sign of `ObjectANCFCable2D`
      but the opposite sign of a strip along y: one convention for all components? (b) note 10b - the thickness
      function of `ShellMesh` differentiates with respect to global x and y, the element reads the 12 values as
      gradients along its edges: convert, or keep the restriction to rectangular elements in the x-y plane (it is
      documented); (c) note 11 - the default integration mode is 0 for the element and `ShellMesh`, 1 for
      `ANCFThinPlateBuilder`; (d) notes 6, 7, 8 belong to RG4.20.3.

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
    - **RG5.1.1** *(maintainer 2026-10-03)* the products of `HomogeneousTransformation` - $\Hm\vv$, $\Hm_1\Hm_2$,
      $\Hm^{-1}$ - with and without the flag of no rotation (`HasNoRotation`, set by construction and, since #2810, for
      a unit matrix given from Python): whether skipping the rotation is a measurable gain, or the branch costs more
      than it saves (`homogeneousTransformationUseIdentityFlag` switches it off).

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
**RG6.1** **DONE 2026-09-24** (#2645) — [log](exudynRevisionLog2026b.md#rg6-1) · [plan text](exudynRevisionLog2026b.md#plan-rg6-1) — OpenVR is removed.

<a id="rg6-2"></a>
**RG6.2** **DONE 2026-09-23** (#2591) — [log](exudynRevisionLog2026b.md#rg6-2) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2) — The settings dialogs, and the shape of `GUI.py` (reopened and closed again the same day for RG6.2.12 to RG6.2.15).

<a id="rg6-2-1"></a>
**RG6.2.1** **DONE 2026-09-22** (#2595) — [log](exudynRevisionLog2026b.md#rg6-2-1) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-1) — The dialogs leave the C++.

<a id="rg6-2-2"></a>
**RG6.2.2** **DONE 2026-09-23** (#2596) — [log](exudynRevisionLog2026b.md#rg6-2-2) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-2) — The layer under the widgets gets tests.

<a id="rg6-2-3"></a>
**RG6.2.3** **DONE 2026-09-23** (#2597, #2601) — [log](exudynRevisionLog2026b.md#rg6-2-3) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-3) — The six small complaints.

<a id="rg6-2-3-1"></a>
**RG6.2.3.1** **DONE 2026-09-23** (#2602) — [log](exudynRevisionLog2026b.md#rg6-2-3-1) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-3-1) — One `dialogs.fontScaling` for every platform.

<a id="rg6-2-4"></a>
**RG6.2.4** **DONE 2026-09-23** (#2604) — [log](exudynRevisionLog2026b.md#rg6-2-4) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-4) — Inline editing.

<a id="rg6-2-5"></a>
**RG6.2.5** **DROPPED 2026-09-23** — [plan text](exudynRevisionLog2026b.md#plan-rg6-2-5) — A second front end.

<a id="rg6-2-6"></a>
**RG6.2.6** **DONE 2026-09-23** (#2591) — [log](exudynRevisionLog2026b.md#rg6-2-6) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-6) — The key bindings are written down three times.

<a id="rg6-2-7"></a>
**RG6.2.7** **DONE 2026-09-23** (#2591) — [log](exudynRevisionLog2026b.md#rg6-2-7) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-7) — `GUI.py` is cleaned up, last.

<a id="rg6-2-8"></a>
**RG6.2.8** **DONE 2026-09-23** (#2605) — [log](exudynRevisionLog2026b.md#rg6-2-8) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-8) — The bottom row reads like code, and says each thing once.

<a id="rg6-2-9"></a>
**RG6.2.9** **DONE 2026-09-23** (#2606) — [log](exudynRevisionLog2026b.md#rg6-2-9) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-9) — A changed value is visible, and every change can be copied at once.

<a id="rg6-2-10"></a>
**RG6.2.10** **DONE 2026-09-23** (#2607) — [log](exudynRevisionLog2026b.md#rg6-2-10) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-10) — Find a setting.

<a id="rg6-2-11"></a>
**RG6.2.11** **DONE 2026-09-23** (#2608) — [log](exudynRevisionLog2026b.md#rg6-2-11) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-11) — The catalogue of optional features, decided.

<a id="rg6-2-12"></a>
**RG6.2.12** **DONE 2026-09-23** (#2612) — [log](exudynRevisionLog2026b.md#rg6-2-12) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-12) — 59 untouched settings were called changed, and a folded folder hid a change.

<a id="rg6-2-13"></a>
**RG6.2.13** **DONE 2026-09-23** (#2613) — [log](exudynRevisionLog2026b.md#rg6-2-13) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-13) — The find bar needs no button, and says nothing when it is idle.

<a id="rg6-2-14"></a>
**RG6.2.14** **DONE 2026-09-23** (#2614) — [log](exudynRevisionLog2026b.md#rg6-2-14) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-14) — Reset, revert, undo, close - and the windows stay in front.

<a id="rg6-2-15"></a>
**RG6.2.15** **DONE 2026-09-23** (#2615) — [log](exudynRevisionLog2026b.md#rg6-2-15) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-15) — A settings folder has a description, and nothing shows it.

<a id="rg6-2-16"></a>
**RG6.2.16** **DONE 2026-09-23** (#2621) — [log](exudynRevisionLog2026b.md#rg6-2-16) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-16) — The window with the changes was invisible.

<a id="rg6-2-17"></a>
**RG6.2.17** **DONE 2026-09-23** (#2623) — [log](exudynRevisionLog2026b.md#rg6-2-17) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-17) — Opening the dialog re-pointed `exudyn.sys` at a throw-away container.

<a id="rg6-2-18"></a>
**RG6.2.18** **DONE 2026-09-24** (#2624) — [log](exudynRevisionLog2026b.md#rg6-2-18) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-18) — The same dialog for `simulationSettings`.

<a id="rg6-2-19"></a>
**RG6.2.19** **DONE 2026-09-23** (#2625) — [log](exudynRevisionLog2026b.md#rg6-2-19) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-19) — Opening the settings dialog closed the render window.

<a id="rg6-2-20"></a>
**RG6.2.20** **DONE 2026-09-23** (#2626) — [log](exudynRevisionLog2026b.md#rg6-2-20) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-20) — The defaults of the lights and the raytracer materials are hidden in C++ constructors.

<a id="rg6-2-21"></a>
**RG6.2.21** **DONE 2026-09-23** (#2627) — [log](exudynRevisionLog2026b.md#rg6-2-21) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-21) — The dialog stopped asking, and undo goes back one whole state.

<a id="rg6-2-22"></a>
**RG6.2.22** **DONE 2026-09-23** (#2630, #2604) — [log](exudynRevisionLog2026b.md#rg6-2-22) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-22) — A double click on a bool no longer toggled it.

<a id="rg6-2-23"></a>
**RG6.2.23** **DONE 2026-09-23** (#2631) — [log](exudynRevisionLog2026b.md#rg6-2-23) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-23) — `dialogs.fontScaling` only worked at 0.

<a id="rg6-2-24"></a>
**RG6.2.24** **DONE 2026-09-24** (#2634) — [log](exudynRevisionLog2026b.md#rg6-2-24) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-24) — The dialogs opened from the command line were larger and blurred.

<a id="rg6-2-25"></a>
**RG6.2.25** **DONE 2026-09-24** (#2635) — [log](exudynRevisionLog2026b.md#rg6-2-25) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-25) — The combo box of an enum repeated the type name in every entry.

<a id="rg6-2-25-1"></a>
**RG6.2.25.1** **DONE 2026-09-24** (#2640) — [log](exudynRevisionLog2026b.md#rg6-2-25-1) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-25-1) — The value cell showed the type name again as soon as the combo box collapsed.

<a id="rg6-2-26"></a>
**RG6.2.26** **DONE 2026-09-24** (#2639, #2621) — [log](exudynRevisionLog2026b.md#rg6-2-26) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-26) — The tooltips were invisible while the dialog is topmost.

<a id="rg6-2-28"></a>
**RG6.2.28** **DONE 2026-09-23** (#2626) — [log](exudynRevisionLog2026b.md#rg6-2-28) · [plan text](exudynRevisionLog2026b.md#plan-rg6-2-28) — The lights and the raytracer materials are defaults of the structure.

<a id="rg6-3"></a>
**RG6.3** **CLOSED 2026-09-27, superseded** (#2583, #2700) — [plan text](exudynRevisionLog2026b.md#plan-rg6-3) — The renderer extraction functions are not shaped for testing.

<a id="rg6-4"></a>
**RG6.4** **DONE 2026-09-23** (#2609) — [log](exudynRevisionLog2026b.md#rg6-4) · [plan text](exudynRevisionLog2026b.md#plan-rg6-4) — The light and shadow descriptions say things that are no longer true.

<a id="rg6-5"></a>
**RG6.5** **DONE 2026-09-24** (#2633) — [log](exudynRevisionLog2026b.md#rg6-5) · [plan text](exudynRevisionLog2026b.md#plan-rg6-5) — Restoring the saved render state took two lines in 82 places.

<a id="rg6-6"></a>
**RG6.6** **DONE 2026-09-24** (#2643) — [log](exudynRevisionLog2026b.md#rg6-6) · [plan text](exudynRevisionLog2026b.md#plan-rg6-6) — macOS: the settings dialog aborted the process.

<a id="rg6-7"></a>
**RG6.7** **DONE 2026-10-03** (#2709) — [log](exudynRevisionLog2026b.md#rg6-7-done) · [plan text](exudynRevisionLog2026b.md#plan-rg6-7) — GraphicsData gets a Sphere and a CurvedTriangleList.

<a id="rg6-9"></a>
**RG6.9** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg6-9) *(maintainer 2026-10-04; "then do RG6.9")* **Curves of connectors drawn with a tiling of their own, as watertight tubes** (#2839).
`ObjectConnectorReevingSystemSprings` draws its rope arcs with `general.cylinderTiling` segments, whatever the angle of
the arc; the spring windings of the spring-dampers use `connectors.springNumberOfWindings` and lines.
    - **RG6.9.1** a setting `connectors.curveTiling` - segments per full turn of a curve -, the number of segments of an
      arc in proportion to its angle; used by the rope of the reeving system and the windings of the spring-dampers
      (`ObjectConnectorSpringDamper`, `CartesianSpringDamper`, `CoordinateSpringDamper(Ext)`, `LinearSpringDamper`);
    - **RG6.9.2** a function of `EXUvis` that draws a watertight tube along a polyline (one ring of vertices per point,
      shared by the neighboring segments, normals from the ring), used by the reeving system and - with a new flag,
      e.g. `connectors.springDraw3D` - by the windings, which are lines today;
    - **RG6.9.3** the graphics regression references and the item images of the connectors drawn again.

<a id="rg6-8"></a>
**RG6.8** *(group RG6; maintainer 2026-09-29)* **The graphics fixes before 1.13** - *"many are graphics
    related; still, some may be solvable or you could suggest a simple test"*. With the test each can
    have:
    - **RG6.8.1** **DONE 2026-09-30** — [log](exudynRevisionLog2026b.md#rg6-8-1) - (#1813) marker positions in
      `AnimateModes` with deformation scaling 0 - **headless**: the marker positions in
      `SC.renderer.GetGraphicsData()` against the reference positions. The superelement markers now draw at the
      mesh nodes as the superelement draws them; `test_superElementMarkerGraphics.py`;
    - **RG6.8.2** **DONE 2026-09-30** — [log](exudynRevisionLog2026b.md#rg6-8-2) - (#2309) `ZoomAll` ignores a
      `trackMarker` - **headless**: the render state after `ZoomAll` with a tracked marker; `ZoomAll` now
      centers the scene with the tracking applied, position and orientation; `test_zoomAllTrackMarker.py`;
    - **RG6.8.3** **DONE 2026-09-30** — [log](exudynRevisionLog2026b.md#rg6-8-3) - (#2321) meshes from NGsolve
      give triangles of the wrong orientation - with ngsolve (optional package): the normals of
      `fem.GetSurfaceTriangles()` against the outward normals. `ImportMeshFromNGsolve` flipped the surface of
      NETGEN, which points outward already; `test_femSurfaceOrientation.py`;
    - **RG6.8.4** **DONE 2026-09-30** (#2308; to be seen on screen in the manual check, row K12) erratic shadows with `modelCentricView=False` and lights in the camera frame -
      a small raytracer image against a reference, as in RG2.3.3.4. **Analysed 2026-09-30**
      ([log](exudynRevisionLog2026b.md#rg6-8-4)): the shadows are OpenGL stencil shadow volumes, which the
      raytracer does not use, so its image cannot show the defect; the likely cause is the clipping of the
      volumes by the near and far planes of the camera-centric projection. **Changed 2026-09-30**
      ([log](exudynRevisionLog2026b.md#rg6-8-4-1)): depth clamping while the volumes are drawn. **DONE
      2026-09-30**: the maintainer checked it on screen - no artifacts any more;
    - **RG6.8.5** (#2140, #2236) Linux: crashes when the renderer closes and with the SolutionViewer; the
      time in the renderer initialized wrong - the manual check (RG2.4) S7, Q1, Q2 on Ubuntu, plus a
      script that starts and stops the renderer twenty times;
    - **RG6.8.6** (#2237, #2350) macOS: PlotSensor in Spyder; `raytracerNOGLFWtest.py`, excluded on macOS
      since 1.11.0 because offscreen `RedrawAndGetImage` crashes - when the macOS machine is there
      (around 2026-10-20), the manual check P1 in Spyder and the test model without its exclusion;
        - **RG6.8.6.1** **DONE 2026-10-05** — [log](exudynRevisionLog2026b.md#rg6-8-6-1) *(maintainer 2026-10-05:
          "investigate if there is something in the code that would explain, and fix it then")* (#2237) a solver
          that is destroyed stops only the threads it started: the garbage collector of a console that runs a
          script again destroyed the solver of the earlier run while the new one ran, and stopped its threads; the
          consistency flags of a system are initialized. Whether this was the crash is the check on macOS.
    - **RG6.8.7** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg6-8-7) *(maintainer 2026-10-04: "by
      converting them to lines, so only a small fix")* (#2844) the raytracer draws no `GraphicsData` of type `Circle` (and none of the circles the
      2D items draw), found by the `GraphicsData` example of RG17.5.

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
**RG9.1** **DONE 2026-09-23** (#2622) — [log](exudynRevisionLog2026b.md#rg9-1) · [plan text](exudynRevisionLog2026b.md#plan-rg9-1) — The item sources stop paying for pybind11.

<a id="rg9-2"></a>
**RG9.2** **DONE 2026-09-23** (#2628, #2629) — [log](exudynRevisionLog2026b.md#rg9-2) · [plan text](exudynRevisionLog2026b.md#plan-rg9-2) — Fourteen item sources included an exception header they do not use, and paid pybind11 for it.

<a id="rg9-3"></a>
**RG9.3** **DONE 2026-10-03** (#2744) — [log](exudynRevisionLog2026b.md#rg9-3-4) · [plan text](exudynRevisionLog2026b.md#plan-rg9-3) — Access functions as single functions of the objects.

<a id="rg9-4"></a>
**RG9.4** **DONE 2026-10-03** (#2202) — [log](exudynRevisionLog2026b.md#rg9-4-2) · [plan text](exudynRevisionLog2026b.md#plan-rg9-4) — Kinetic and potential energy as output variables.

<a id="rg9-5"></a>
**RG9.5** **DONE 2026-10-03** (#2779) — [log](exudynRevisionLog2026b.md#rg9-5) · [plan text](exudynRevisionLog2026b.md#plan-rg9-5) — `mbs.ComputeItem`: the computation functions of an item from Python. (RG9.5.1 to RG9.5.6 done, RG9.5.7 decided to stay as it is)

## RG10 — Tooling and process

The machinery a maintainer uses: `exudev` (revision2026 step R5.18), the issue tracker and its
JSON store (revision2026 steps R8.3 to R8.5), the generators (revision2026 step R4.3), the checks of the commit gate, and the CI. It
works; this group carries what it still lacks.

Open in the tracker for this group: **#2541** (`exudyn.config` and `exudyn.special` are in no stub
file, so an editor cannot complete them).

<a id="rg10-1"></a>
**RG10.1** **DONE 2026-09-27** (#2712) — [log](exudynRevisionLog2026b.md#rg10-1) · [plan text](exudynRevisionLog2026b.md#plan-rg10-1) — Checker for user scripts after the 1.12 API changes: exudev scripts, a maintainer tool.
    - **RG10.1.1** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg10-1-1) *(maintainer 2026-10-04: "Do the
      suggested next 3 steps")* (#2713) `exudev scripts --run` runs each script in a copy of its folder, without windows,
      with a timeout, after a check for paths outside the folder.

<a id="rg10-2"></a>
**RG10.2** **DONE 2026-09-23** (#2600) — [log](exudynRevisionLog2026b.md#rg10-2) · [plan text](exudynRevisionLog2026b.md#plan-rg10-2) — The issue table of `exudev issue serve` did not say what its columns are, and left out the priority.

<a id="rg10-2-1"></a>
**RG10.2.1** **DONE 2026-09-24** (#2636, #2240) — [log](exudynRevisionLog2026b.md#rg10-2-1) · [plan text](exudynRevisionLog2026b.md#plan-rg10-2-1) — The search of `exudev issue serve` missed most fields, and the list stopped at 400.

<a id="rg10-2-2"></a>
**RG10.2.2** **DONE 2026-09-24** (#2641, #2497, #249, #2490, #2499) — [log](exudynRevisionLog2026b.md#rg10-2-2) · [plan text](exudynRevisionLog2026b.md#plan-rg10-2-2) — A search for digits found every field except the issue number.

<a id="rg10-3"></a>
**RG10.3** **DONE 2026-09-23** (#2617) — [log](exudynRevisionLog2026b.md#rg10-3) · [plan text](exudynRevisionLog2026b.md#plan-rg10-3) — `exudev` does not say how long a step took.

<a id="rg10-4"></a>
**RG10.4** **DONE 2026-09-23** (#2618) — [log](exudynRevisionLog2026b.md#rg10-4) · [plan text](exudynRevisionLog2026b.md#plan-rg10-4) — `src/pythonGenerator/` holds one file and should not exist.

<a id="rg10-5"></a>
**RG10.5** **DONE 2026-09-23** (#2619) — [log](exudynRevisionLog2026b.md#rg10-5) · [plan text](exudynRevisionLog2026b.md#plan-rg10-5) — VS Code cannot follow a C++ include.

<a id="rg10-6"></a>
**RG10.6** **DONE 2026-09-24** (#2632) — [log](exudynRevisionLog2026b.md#rg10-6) · [plan text](exudynRevisionLog2026b.md#plan-rg10-6) — The TestModels imported the test suite to find out whether they are being tested.

<a id="rg10-7"></a>
**RG10.7** **DONE 2026-09-24** (#2638) — [log](exudynRevisionLog2026b.md#rg10-7) · [plan text](exudynRevisionLog2026b.md#plan-rg10-7) — The plan carried the full text of the steps that are finished.

<a id="rg10-7-1"></a>
**RG10.7.1** **DONE 2026-09-24** (#2642) — [log](exudynRevisionLog2026b.md#rg10-7-1) · [plan text](exudynRevisionLog2026b.md#plan-rg10-7-1) — The plan did not say what to do next.

<a id="rg10-8"></a>
**RG10.8** **DONE 2026-09-24** (#2644) — [log](exudynRevisionLog2026b.md#rg10-8) · [plan text](exudynRevisionLog2026b.md#plan-rg10-8) — `exudev` is needed on linux and macOS too.

<a id="rg10-9"></a>
**RG10.9** **DONE 2026-09-24** (#2647) — [log](exudynRevisionLog2026b.md#rg10-9) · [plan text](exudynRevisionLog2026b.md#plan-rg10-9) — `regenerated_files` failed on linux and could not fail on Windows.


<a id="rg10-10"></a>
**RG10.10** **DONE 2026-09-25** (#2653) — [log](exudynRevisionLog2026b.md#rg10-10) · [plan text](exudynRevisionLog2026b.md#plan-rg10-10) — The header of `definitionLoader.py` read like the file was dead.

<a id="rg10-11"></a>
**RG10.11** **DONE 2026-09-28** (#2541) — [log](exudynRevisionLog2026b.md#rg10-11) · [plan text](exudynRevisionLog2026b.md#plan-rg10-11) — `exudyn.config` and `exudyn.special` reach a stub file.

<a id="rg10-12"></a>
**RG10.12** **DONE 2026-09-29** (#2747) — [log](exudynRevisionLog2026b.md#rg10-12) · [plan text](exudynRevisionLog2026b.md#plan-rg10-12) — The GitLab job `check_docstrings` passes, and the gates run pydoclint.

<a id="rg10-13"></a>
**RG10.13** **DONE 2026-09-29** (#2752) — [log](exudynRevisionLog2026b.md#rg10-13) · [plan text](exudynRevisionLog2026b.md#plan-rg10-13) — `exudev issue plot`.

<a id="rg10-14"></a>
**RG10.14** **DONE 2026-09-30** (#2760) — [log](exudynRevisionLog2026b.md#rg10-14) · [plan text](exudynRevisionLog2026b.md#plan-rg10-14) — `exudev` runs the pytest files and the MiniExample performance run.

<a id="rg10-15"></a>
**RG10.15** **DONE 2026-10-04** (#2833) — [log](exudynRevisionLog2026b.md#rg2-5) *(group RG10; feedback of a colleague installing Exudyn, forwarded by the maintainer 2026-10-04:
"Only for a complete build, different venvs for different Python versions shall be required")* **`exudev env` probes the
environments that exist**: without `--py` or `--env`, the missing ones of the version matrix are named as needed only
for `build --complete`, instead of stopping the command.

## RG11 — Misc

What belongs to no group yet. Three of a kind here are a reason to propose a group of their own.

<a id="rg11-1"></a>
**RG11.1** **DONE 2026-09-24** (#2610) — [log](exudynRevisionLog2026b.md#rg11-1) · [plan text](exudynRevisionLog2026b.md#plan-rg11-1) — The results monitor runs beside the simulation, or it is redundant.

<a id="rg11-3"></a>
**RG11.3** **DONE 2026-09-26** (#2670) — [log](exudynRevisionLog2026b.md#rg11-3) · [plan text](exudynRevisionLog2026b.md#plan-rg11-3) — The results monitor beside a running simulation.

<a id="rg11-2"></a>
**RG11.2** **DONE 2026-09-23** (#2620) — [log](exudynRevisionLog2026b.md#rg11-2) · [plan text](exudynRevisionLog2026b.md#plan-rg11-2) — The demos wrote a `solution/` directory into whatever directory they were started in.

## RG12 — Python interface

*(Group proposed by the maintainer, 2026-09-22.)* The shape of the Python API itself, as opposed
to what it computes: how a parameter is named, what happens when a name changes, what a user can
find out about the settings of a model. It is the group a user notices most and reads least about.

<a id="rg12-1"></a>
**RG12.1** **DONE 2026-10-03** (#2588) — [log](exudynRevisionLog2026b.md#rg12-1) · [plan text](exudynRevisionLog2026b.md#plan-rg12-1) — `simulationSettings` gets the deprecation mechanism.

<a id="rg12-2"></a>
**RG12.2** **DONE 2026-10-03** (#2589) — [log](exudynRevisionLog2026b.md#rg12-2) · [plan text](exudynRevisionLog2026b.md#plan-rg12-2) — Item parameters can be deprecated.

<a id="rg12-3"></a>
**RG12.3** **DONE 2026-09-24** (#2590) — [log](exudynRevisionLog2026b.md#rg12-3) · [plan text](exudynRevisionLog2026b.md#plan-rg12-3) — What did this model actually change?

<a id="rg12-4"></a>
**RG12.4** **DONE 2026-09-26** (#2664) — [log](exudynRevisionLog2026b.md#rg12-4-1) · [plan text](exudynRevisionLog2026b.md#plan-rg12-4) — A user function is one typed Python function, and everything else is generated from it.

<a id="rg12-5"></a>
**RG12.5** **DONE 2026-09-28** (#2666) — [plan text](exudynRevisionLog2026b.md#plan-rg12-5) — User settings that persist between runs: one `~/.exudyn` file, and what may be in it.

<a id="rg12-6"></a>
**RG12.6** **DONE 2026-09-26** (#2667) — [log](exudynRevisionLog2026b.md#rg12-6) · [plan text](exudynRevisionLog2026b.md#plan-rg12-6) — The columns of a settings dialog are relative and configurable.

<a id="rg12-7"></a>
**RG12.7** **DONE 2026-09-26** (#2668) — [log](exudynRevisionLog2026b.md#rg12-6) · [plan text](exudynRevisionLog2026b.md#plan-rg12-7) — The mouse wheel changes the font size of a dialog.

<a id="rg12-8"></a>
**RG12.8** **DONE 2026-09-26** (#2671) — [log](exudynRevisionLog2026b.md#rg12-8) · [plan text](exudynRevisionLog2026b.md#plan-rg12-8) — One test for all user functions at once.

<a id="rg12-9"></a>
**RG12.9** **DONE 2026-09-26** (#2679) — [log](exudynRevisionLog2026b.md#rg12-9) · [plan text](exudynRevisionLog2026b.md#plan-rg12-9) — The override settings live in `exudyn.special.overrideSettings`.

<a id="rg12-10"></a>
**RG12.10** **DONE 2026-09-26** (#2684) — [log](exudynRevisionLog2026b.md#rg12-10) · [plan text](exudynRevisionLog2026b.md#plan-rg12-10) — The workflow of the override settings.

<a id="rg12-11"></a>
**RG12.11** **DONE 2026-09-26** (#2685) — [log](exudynRevisionLog2026b.md#rg12-11) · [plan text](exudynRevisionLog2026b.md#plan-rg12-11) — Storing the override settings.

<a id="rg12-12"></a>
**RG12.12** **DONE 2026-09-27** (#2588) — [log](exudynRevisionLog2026b.md#rg12-12) · [plan text](exudynRevisionLog2026b.md#plan-rg12-12) — PlotSensor takes its defaults from the override settings.

<a id="rg12-13"></a>
**RG12.13** **DONE 2026-09-26** (#2686) — [log](exudynRevisionLog2026b.md#rg12-13) · [plan text](exudynRevisionLog2026b.md#plan-rg12-13) — A stored dialog geometry is used.

<a id="rg12-14"></a>
**RG12.14** **DONE 2026-09-26** (#2687) — [log](exudynRevisionLog2026b.md#rg12-14) · [plan text](exudynRevisionLog2026b.md#plan-rg12-14) — The override settings can be read again.

<a id="rg12-15"></a>
**RG12.15** **DONE 2026-09-27** (#2688) — [log](exudynRevisionLog2026b.md#rg12-15) · [plan text](exudynRevisionLog2026b.md#plan-rg12-15) — A script can place a dialog, and it is written down.

<a id="rg12-16"></a>
**RG12.16** **DONE 2026-09-26** (#2689) — [log](exudynRevisionLog2026b.md#rg12-16) · [plan text](exudynRevisionLog2026b.md#plan-rg12-16) — The render window and the SolutionViewer remember their size and position.

<a id="rg12-19"></a>
**RG12.19** **DONE 2026-09-27** (#2693) — [log](exudynRevisionLog2026b.md#rg12-19) · [plan text](exudynRevisionLog2026b.md#plan-rg12-19) — Two buttons: one for the settings, one for the positions.

<a id="rg12-20"></a>
**RG12.20** **DONE 2026-09-27** (#2694) — [log](exudynRevisionLog2026b.md#rg12-20) · [plan text](exudynRevisionLog2026b.md#plan-rg12-20) — Where the render window is, and what happens when the file and the session disagree.

<a id="rg12-21"></a>
**RG12.21** **DONE 2026-09-27** (#2695) — [log](exudynRevisionLog2026b.md#rg12-21) · [plan text](exudynRevisionLog2026b.md#plan-rg12-21) — `python -m exudyn info` prints the home directory.

<a id="rg12-22"></a>
**RG12.22** **DONE 2026-09-27** (#2696) — [log](exudynRevisionLog2026b.md#rg12-22) · [plan text](exudynRevisionLog2026b.md#plan-rg12-22) — The results monitor took the focus and came to the front on every update.

<a id="rg12-23"></a>
**RG12.23** **DONE 2026-09-27** (#2698) — [log](exudynRevisionLog2026b.md#rg12-23) · [plan text](exudynRevisionLog2026b.md#plan-rg12-23) — The plot windows cannot be stored while the renderer is still open.


<a id="rg12-24"></a>
**RG12.24** **DONE 2026-09-27** (#2718) — [log](exudynRevisionLog2026b.md#rg12-24) · [plan text](exudynRevisionLog2026b.md#plan-rg12-24) — The files a run writes by default go into `solution/`.

<a id="rg12-25"></a>
**RG12.25** **DONE 2026-09-27** (#2719) — [log](exudynRevisionLog2026b.md#rg12-25) · [plan text](exudynRevisionLog2026b.md#plan-rg12-25) — Store positions stores every open window.

<a id="rg12-26"></a>
**RG12.26** **DONE 2026-09-27** (#2720) — [log](exudynRevisionLog2026b.md#rg12-26) · [plan text](exudynRevisionLog2026b.md#plan-rg12-26) — The SolutionViewer follows the width of its window, takes a size and is stored with the other windows.

<a id="rg12-27"></a>
**RG12.27** **DONE 2026-09-27** (#2722) — [log](exudynRevisionLog2026b.md#rg12-27) · [plan text](exudynRevisionLog2026b.md#plan-rg12-27) — Store positions with Qt plot windows, and the SolutionViewer when it is narrow.

<a id="rg12-28"></a>
**RG12.28** **DONE 2026-09-27** (#2723) — [log](exudynRevisionLog2026b.md#rg12-28) · [plan text](exudynRevisionLog2026b.md#plan-rg12-28) — PlotSensor opens at the stored size.

<a id="rg12-30"></a>
**RG12.30** **DONE 2026-09-30** (#2756, #2759, #2757) — [log](exudynRevisionLog2026b.md#rg12-30) · [plan text](exudynRevisionLog2026b.md#plan-rg12-30) — `exudyn.utilities` imports less.

<a id="rg12-29"></a>
**RG12.29** *(group RG12; maintainer 2026-09-30)* **What an item provides, asked from Python** (#2203).
    There is no way to ask an item which output variables it has, which node or marker types it
    requests or provides, or which access functions it offers: `mbs.GetObject()` returns the parameters
    and the type name, and a request for an output variable the item lacks answers with an error that
    names only the one asked for (checked 2026-09-30). The information exists in C++
    (`GetOutputVariableTypes`, `GetRequestedNodeType`, `GetRequestedMarkerType`, `GetType`,
    `GetAccessFunctionTypes`). #2203 proposes `mbs.Inspect(itemIndex, what, ...)`; a test could then
    loop over the output variables an object declares instead of a hand-kept list, and the generated
    item pages (RG13.5.0.3) show the same. Not implemented yet - the interface first: one function
    with a `what` argument or one function per question.

    **The maintainer (2026-09-30)**: one function with a `what`, where `what` is a **type** and not a
    string - importable, completed by an editor; *"make a good suggestion"*. **Proposed**:

    ```python
    mbs.Inspect(itemIndex, what=None)
    #itemIndex: an exu.ObjectIndex, NodeIndex, MarkerIndex, LoadIndex or SensorIndex - the typed index
    #           says the kind of item, so a plain int is refused with the hint to use the typed one
    #what:      a member of exu.InspectType, or None for a dict {InspectType.X: answer} of all that apply

    mbs.Inspect(oBody, exu.InspectType.OutputVariables)
    #-> [exu.OutputVariableType.Position, exu.OutputVariableType.Velocity, ...]
    mbs.Inspect(oSpring, exu.InspectType.RequestedMarkerTypes)
    #-> [exu.MarkerType.Position, exu.MarkerType.Position]   one entry per marker the connector takes
    ```

    `exu.InspectType` is a new enum, generated like the others (`definitions/`), and **every answer is a
    list of the enums Exudyn already exports** - `OutputVariableType`, `NodeType`, `MarkerType`,
    `ObjectType`, `AccessFunctionType` - never an integer bit mask and never a string:

    | `InspectType` | applies to | answers with |
    |---|---|---|
    | `OutputVariables` | object, node | `[OutputVariableType]` - with the parameters the item has now, so energies only where they can be computed (RG9.4) |
    | `ObjectType` | object | `[ObjectType]`: `Body`, `Connector`, `Constraint`, `SuperElement`, ... - the flags, one enum each |
    | `NodeType` | node | `[NodeType]` the node provides: `Position`, `Orientation`, `RotationEulerParameters`, ... |
    | `RequestedNodeTypes` | object, node marker | one `[NodeType]` per node it takes |
    | `MarkerType` | marker | `[MarkerType]` it provides |
    | `RequestedMarkerTypes` | connector, constraint, load | one `[MarkerType]` per marker it takes |
    | `AccessFunctions` | body | `[AccessFunctionType]` it offers - what decides which body markers it takes |

    A `what` that does not apply to the kind of item raises with the list of those that do. The flags that
    are combinations in C++ (`NodeType`, `MarkerType`, `ObjectType`, `AccessFunctionType`) are split into
    their single members, which is what a script compares against. A test then loops over
    `Inspect(item, InspectType.OutputVariables)` of every MiniExample instead of a hand-kept list, and the
    item pages of RG13.5.0.3 can take the same answers. **Confirmed by the maintainer, 2026-10-01.**
    **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg12-29) - `mbs.Inspect` and `exu.InspectType` as
    proposed; output variables of markers too; a sensor has nothing to inspect (`{}` for `what=None`); the
    potential energy listed only where `PotentialEnergyAvailable()`; the test model `inspectTest.py` and the pytest
    `test_inspectOutputVariables.py`, which reads every listed output variable of every MiniExample.
    - **RG12.29.1** **DONE 2026-10-01** (#2768) — [log](exudynRevisionLog2026b.md#rg12-29-1) - the output
      variables that items declared and could not compute, found by that test, all implemented: `NodePoint2D` (`RotationMatrix`, `Rotation`, `AngularVelocity(Local)`), `MarkerNodeODE1Coordinate`
      (`Coordinates_t`), `ObjectANCFThinPlate` (`Director2`), `ObjectANCFBeam` (`AngularVelocity(Local)`),
      `ObjectRotationalMass1D` (`AngularVelocityLocal`), and two that depend on the node - `ObjectRigidBody` on a
      Lie group node (the accelerations of rotation), `MarkerNodeRigid` on a node without angular velocity
      (`NodePointGround`); the plate's `Director1`, `ForceLocal` and `TorqueLocal` returned zeros as well;
    - **RG12.29.2** **DONE 2026-10-04** (#2817) — [log](exudynRevisionLog2026b.md#rg12-29-2) - the node types a
      **node marker** requests: the declaration `requestedNodeTypes` generates `GetRequestedNodeTypes()` of the Main
      class, `Assemble` checks it instead of the hand-written C++, and `Inspect` answers it - per node a list of
      requirements, each a list of alternatives.

<a id="rg12-31"></a>
**RG12.31** **DONE 2026-10-04** (#2802) — [plan text](exudynRevisionLog2026b.md#plan-rg12-31) — Settings and item parameters that could be renamed or restructured.

<a id="rg12-34"></a>
**RG12.34** **DONE 2026-10-04** (#2813) — [log](exudynRevisionLog2026b.md#rg12-34) · [plan text](exudynRevisionLog2026b.md#plan-rg12-34) — The simulation settings renamed and restructured as decided in RG12.31.
    - **RG12.34.7** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg12-34-7) *(maintainer 2026-10-04: "Do the
      suggested next 3 steps")* (#2847) `SetDictionary` of the simulation and visualization settings takes a dictionary of
      Exudyn 1.11: its old keys go to their new places, and the keys it does not give keep their values;
    - **RG12.34.8** **DONE 2026-10-04** (#2816, #2818) — [log](exudynRevisionLog2026b.md#rg12-34-8) - the binary
      solution file takes the size of its numbers from `solution.precision` (maintainer 2026-10-04); the debug print
      of `LoadBinarySolutionFile` removed.

<a id="rg12-36"></a>
**RG12.36** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg12-36) *(group RG12; maintainer 2026-10-04: "do this
step already now"; full Newton in the test models that used the default, with the comment "Just for the test; modified
Newton is usually faster"; the drift of the MiniExamples accepted)* **Two Newton structures, and the modified Newton by
default in the time integration** (#2815) - realized with `memberDefaults` of the one structure, no second one. `NewtonSettings` is one structure shared by `timeIntegration` and `staticSolver`; with
the forwarding of RG12.1 and RG12.34 two structures are possible, most of their members copied, so that each can have
its own defaults - above all `timeIntegration.newton.useModifiedNewton = True`, a large gain for the user. It changes
the results of many test models (iterations, step sizes): the test suite is evaluated again, model by model, before.

<a id="rg12-37"></a>
**RG12.37** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg12-37) *(group RG12; maintainer 2026-10-04: "If I
use spyder or VS Code and I click on CreateRigidBody, it directs me to the .pyi file. Is there a fix for that?"; then:
"ok, do (a)")* **Go to definition of `mbs.CreateRigidBody` reaches the Python
function** (#2825). The stub `exudyn/__init__.pyi` declares each function added to `MainSystem` (the 25 Create
functions, `SolveDynamic`, `PlotSensor`, `SolutionViewer`, ... - 32) as a method with a copy of its signature and
the first sentence of its docstring; the function is `MainSystemCreateRigidBody` in
`exudyn/misc/mainSystemExtensions.py`. **Tried 2026-10-04** on a copy of the package, with jedi 0.20 (Spyder) and
mypy (the gate's stubtest):
    - (a) in the stub, `CreateRigidBody = _MainSystemCreateRigidBody` with the import at its top: mypy takes the
      signature and the binding from the function itself (`mbs` dropped); jedi's go to definition lands on that line
      of the stub, which names the function, and its inference and signature help reach the source with the full
      docstring - one click more than wanted; the return annotations of today's stub (`-> ObjectIndex`) are lost
      unless the functions get them;
    - (b) `from exudyn.misc.mainSystemExtensions import MainSystemCreateRigidBody as CreateRigidBody` inside the class
      of the stub: jedi goes straight to the source; **mypy refuses it** ("Unsupported class scoped import") and types
      the method as `Any`;
    - (c) today's stub, its docstring naming the module and the function: nothing breaks, nothing is clickable.
    VS Code (Pylance) could not be tried here. **Recommended**: (a) with return annotations on the functions (the
    stub generator then writes one line per function instead of a copied signature). **Decided (a)**, the return
    annotations RG12.38.

<a id="rg12-38"></a>
**RG12.38** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg12-38) *(group RG12; maintainer 2026-10-04: "add a
step to add return annotations for the Create functions")*
**Return annotations for the functions added to `MainSystem`** (#2826). Since RG12.37 the stub assigns the functions
themselves, so their return types are those of the functions - unannotated today, while the copied stub had them
(`ObjectIndex`, `NodeIndex`, `Union[dict, ObjectIndex]` for `returnDict`, ...). The 25 Create functions and the others
(`SolveDynamic`, `SolveStatic`, `PlotSensor`, `SolutionViewer`, `ComputeLinearizedSystem`, `ComputeODE2Eigenvalues`,
`ComputeSystemDegreeOfFreedom`, `CreateDistanceSensor...`, `DrawSystemGraph`) get them, from the `Returns:` of their
docstrings; argument annotations only where they help a reader; a check that every function added to a class has one.

<a id="rg12-35"></a>
**RG12.35** **DONE 2026-10-04** (#2814) — [log](exudynRevisionLog2026b.md#rg12-35) · [plan text](exudynRevisionLog2026b.md#plan-rg12-35) — The item parameters renamed as decided in RG12.31.

<a id="rg12-32"></a>
**RG12.32** **DONE 2026-10-03** (#2804, #2805, #2806) — [log](exudynRevisionLog2026b.md#rg12-32) · [plan text](exudynRevisionLog2026b.md#plan-rg12-32) — A deprecation warns where the user wrote the deprecated name, once, and is counted.

<a id="rg12-33"></a>
**RG12.33** **DONE 2026-10-03** (#2807) — [log](exudynRevisionLog2026b.md#rg12-33) · [plan text](exudynRevisionLog2026b.md#plan-rg12-33) — Deprecations of the Python library, declared and checked.

<a id="rg12-39"></a>
**RG12.39** *(group RG12; maintainer 2026-10-05, from RG4.19.8)* **How a simulation continues from its restart file**
    (#2850). The restart file is written (`simulationSettings.solution.restart`: `write`, `name`, `writePeriod`), and
    a prototype that reads it back is `_InitializeFromRestartFile` in `basicUtilities.py`, not public since #1565 was
    closed. Before a function is written, evaluate how a restart works together with the model script: where the
    restart file comes in - for example the system detects that one is available and loads its state from it -,
    what else a restart needs (the time, the solver's state, sensors and files that continue), and what the user
    writes. Then a proposal, for the maintainer's decision.
    **Evaluated 2026-10-05** — [log](exudynRevisionLog2026b.md#rg12-39) *(maintainer 2026-10-05: "do RG12.39")*:
    no solver writes a restart file - `solution.restart.write=True` only warns - so the prototype reads a format that
    does not exist; the descriptions of the settings say so now. **Proposal, for decision** (options in the log):
    - **RG12.39.1** *(proposed)* the solvers write the restart file: one row - time, ODE2, ODE2_t, ODE2_tt, the
      algorithmic accelerations of the generalized-alpha method, ODE1, AE, data coordinates, the current step size -
      and a header with a fingerprint of the system (its numbers of items and coordinates, the solver type), every
      `writePeriod` and at the end; written to a temporary file and renamed, the previous one kept as `.bck`.
    - **RG12.39.2** *(proposed)* the script asks for it: `mbs.SolveDynamic(simulationSettings, restartFile='...')`
      (and `SolveStatic`) - after `Assemble`, the solver checks the fingerprint, sets the state and the start time
      from the file, and appends to the solution and sensor files; the end time stays the script's. The model -
      items, user functions - is the script's, which is why the restart is not a pickled `SystemContainer`.
    - **RG12.39.3** *(proposed, later)* `solution.restart.continueIfAvailable`: the same without changing the
      script - for a job on a cluster that is killed and started again with the same script; a file whose
      fingerprint does not fit is an error, not ignored.
    - Test: a run from 0 to 1 against a run from 0 to 0.5 and a restart to 1 - identical with the algorithmic
      accelerations stored, which is the reason they are in the file.

## RG13 — Item documentation

*(Group created by the maintainer, 2026-09-27.)* **Every item gets a full documentation and a
MiniExample** - node, object, marker, load and sensor - which is more than fits into RG3, and **one of
the most important steps before Exudyn 1.13**: the reference manual of the items is what users read
most, and it is generated from `definitions/itemDefs*.py`, so what a definition does not carry, no
page shows.

What depends on it: the graphics regression test takes every item through its MiniExample
(RG2.3.3.5), and the image of each item on its page can be written by the same run.

<a id="rg13-1"></a>
**RG13.1** **DONE 2026-09-27** (#2715) — [log](exudynRevisionLog2026b.md#rg13-1) · [plan text](exudynRevisionLog2026b.md#plan-rg13-1) — The state of the documentation, item by item.

<a id="rg13-2"></a>
**RG13.2** **CLOSED 2026-09-28** — [log](exudynRevisionLog2026b.md#decisions-2026-09-29) · [plan text](exudynRevisionLog2026b.md#plan-rg13-2) — What the ideal documentation of an item contains.

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
**RG13.4** **DONE 2026-09-29** (#2721) — [log](exudynRevisionLog2026b.md#rg13-4) · [plan text](exudynRevisionLog2026b.md#plan-rg13-4) — The development documents per item type.

<a id="rg13-5"></a>
**RG13.5** **DONE 2026-09-30** (#2725) — [log](exudynRevisionLog2026b.md#rg13-5-0-1) · [plan text](exudynRevisionLog2026b.md#plan-rg13-5) — The documentation of the items, written by the documents of RG13.4. (nodes, objects, markers, loads, sensors written; ObjectBeamGeometricallyExact by RG4.8.9)

<a id="rg13-6"></a>
**RG13.6** **DONE 2026-10-03** (#2732) — [log](exudynRevisionLog2026b.md#rg13-6-1) · [plan text](exudynRevisionLog2026b.md#plan-rg13-6) — A MiniExample for every item. (a MiniExample for every item but ObjectFFRF/ObjectFFRFreducedOrder (RG13.6.6, not decided))

<a id="rg13-7"></a>
**RG13.7** **DONE 2026-09-29** (#2742) — [log](exudynRevisionLog2026b.md#rg13-7) · [plan text](exudynRevisionLog2026b.md#plan-rg13-7) — How to set up a new item.

<a id="rg13-8"></a>
**RG13.8** **DONE 2026-10-04** *(maintainer 2026-10-04)* **How an item is drawn** (#2840): the drawing of the items is documented nowhere but in the
74 `UpdateGraphics` functions - which settings they use for size, tiling and color, what `drawSize = -1` means, what a
color of `[-1,-1,-1,-1]` takes. Proposed:
    - **RG13.8.1** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg13-8-1) *(maintainer 2026-10-04: "continue
      with 1-3")* an inventory, per item: the settings and the parameters its `UpdateGraphics` reads, its default
      size and color, what it draws (and what not, e.g. the windings of a spring or the axes of a joint); written as a
      table, and where a setting is misleading or does not fit (e.g. `general.cylinderTiling` for the arcs of a rope,
      RG6.9), raised;
    - **RG13.8.2** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg13-8-2) *(maintainer 2026-10-04: "Do next
      RG13.8 steps")* a section *Drawing* in the generated frame of each item page, from a declaration in the definition
      (e.g. `drawing=r'...'` and the settings it uses, which the emitter links to the settings page), so that the page
      and the code are checked against each other where possible;
    - **RG13.8.3** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg13-8-2) the general rules once per kind, in `itemKindDefinitions.py`: nodes, markers, loads and sensors are
      drawn alike within their kind (size, color, number), objects one by one;
    - **RG13.8.5** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg13-8-5) *(maintainer 2026-10-04: "the change
      is ok")* (#2846) `ObjectContactCurveCircles` draws the points of its curve with `contact.contactPointsDefaultSize`
      instead of the deprecated `connectors.contactPointsDefaultSize`;
    - **RG13.8.4** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg13-8-4) *(maintainer 2026-10-04: the
      decisions per finding, see the log)* (#2843) the findings of the inventory: parameters that are not read (`ObjectConnectorDistance.drawSize`),
      items with drawing parameters and no drawing (`ObjectConnectorGravity`, `ObjectConnectorCoordinateVector`, and
      `show` of 13 items that draw nothing), a description that names another setting than the code
      (`ObjectContactCurveCircles`), a view setting read into the graphics data (`ObjectANCFThinPlate`), the load drawn
      with its user function only without multithreaded rendering, the contour of the 3D beam line, the tiling of
      connectors and joints without a rule - per finding: draw it, remove the parameter, or say what it does.

## RG14 — Marker values computed where they are used

*(Group created by the maintainer, 2026-09-29.)* Today every connector, joint, constraint and load gets
a `MarkerData` for each of its markers, computed before it is called - positions, orientations,
velocities and full Jacobians, whether it needs them or not. The alternative: the connector or load
computes the marker values itself, with a small temporary per marker. **The big advantage is automatic
differentiation**, which then sees the whole computation from the coordinates to the force. It shapes
how future items and the user elements (RG7, RG8) are written, so it is decided first, even if it is not
done.

<a id="rg14-1"></a>
**RG14.1** **CLOSED 2026-10-01** — [plan text](exudynRevisionLog2026b.md#plan-rg14-1) — The evaluation.

<a id="rg14-2"></a>
**RG14.2** **DONE 2026-10-04** (#2745) — [log](exudynRevisionLog2026b.md#rg14-2-1) · [plan text](exudynRevisionLog2026b.md#plan-rg14-2) — The migration. (the migration done; RG14.2.9.4 and RG14.2.12 in the list of what is not decided)

<a id="rg14-3"></a>
**RG14.3** *(group RG14; maintainer 2026-10-01)* **Joints and their Jacobians on homogeneous transformations.**
    The derivatives of the kinematic equations of the joints are always of the same kind - relative position
    and rotation of two frames, projected on axes - and each joint writes them by hand today, with its own
    rotation Jacobians. With the rigid markers as homogeneous transformations (RG14.2.8), a joint's
    constraint equations become functions of $\Hm_0^{-1}\Hm_1$, and their Jacobians follow systematically from
    the relative twist - one implementation for `JointGeneric`, `JointRevoluteZ`, `JointPrismaticX`, the 2D
    joints and the rolling disc, instead of one each. Done when homogeneous transformations are integrated
    more deeply, after RG14.2.9 (constraints on L0/L1/L2); a step of its own because it replaces working
    code and needs the comparison of RG14.2.

## RG15 — Objects computing from given coordinates

*(Group created by the maintainer, 2026-09-29.)* A body or finite element reads its coordinates from its
nodes inside `ComputeODE2LHS` and the mass matrix. If it got the coordinates - displacements and
velocities - as arguments instead, automatic differentiation of an object would be simple. Unlike RG14
this is **a real performance question** with more cases: objects with one node (`ObjectMassPoint`,
`ObjectRigidBody`, ...) can keep linked data without copying, while finite elements and other multi-node
objects would get their coordinates from the interface.

<a id="rg15-1"></a>
**RG15.1** **DONE 2026-09-29** — [plan text](exudynRevisionLog2026b.md#plan-rg15-1) — The evaluation.

## RG16 — Homogeneous transformations

*(Group created by the maintainer, 2026-10-02.)* A rigid frame - position and rotation - is one homogeneous
transformation (HT). The C++ class exists (`HomogeneousTransformationBase` in `RigidBodyMath`, the frames of the
connector interface since RG14.2.8), Python has `rigidBodyUtilities.HomogeneousTransformation` on 4x4 numpy arrays,
and the items take a position and a rotation matrix. The group makes the C++ class fast and reachable from Python, and
then lets the rigid items take and give an HT. **Why now** (maintainer): the user items to come - in Python (RG7) and
in C++ (RG8) - shall meet the newer interfaces from the start, not deprecated ones; for robotics, the kinematic tree and
the joints an HT means fewer variables and one way of doing things.

<a id="rg16-1"></a>
**RG16.1** **DONE 2026-10-02** (#2780) — [log](exudynRevisionLog2026b.md#rg16-1) · [plan text](exudynRevisionLog2026b.md#plan-rg16-1) — The C++ class and its Python binding. (the class, its binding and the output variable)

<a id="rg16-2"></a>
**RG16.2** **DECIDED 2026-10-02** — [log](exudynRevisionLog2026b.md#rg16-2-decided) · [plan text](exudynRevisionLog2026b.md#plan-rg16-2) — The evaluation: HT in the user interface of the rigid items.

<a id="rg16-3"></a>
**RG16.3** **DONE 2026-10-03** — [log](exudynRevisionLog2026b.md#rg16-3-1) · [plan text](exudynRevisionLog2026b.md#plan-rg16-3) — The cases that break nothing for users.

<a id="rg16-4"></a>
**RG16.4** **DONE 2026-10-03** — [log](exudynRevisionLog2026b.md#rg16-4) · [plan text](exudynRevisionLog2026b.md#plan-rg16-4) — The further steps.

<a id="rg16-5"></a>
**RG16.5** **DONE 2026-10-03** — [plan text](exudynRevisionLog2026b.md#plan-rg16-5) — `localHT` in the rigid markers.

<a id="rg16-6"></a>
**RG16.6** **DECIDED 2026-10-03** (#2809) — [plan text](exudynRevisionLog2026b.md#plan-rg16-6) — What `exu.HT` offers, and how its parts are named.

<a id="rg16-7"></a>
**RG16.7** **DONE 2026-10-03** (#2810) — [log](exudynRevisionLog2026b.md#rg16-7) · [plan text](exudynRevisionLog2026b.md#plan-rg16-7) — `exu.HT` as decided in RG16.6.

<a id="rg16-8"></a>
**RG16.8** **DONE 2026-10-04** (#2819) — [log](exudynRevisionLog2026b.md#rg16-8) *(group RG16; maintainer 2026-10-04)*
**The logarithm and the exponential map in `exu.HT`**: `LogSE3()`, `SetExpSE3(v)`, `LogR3xSO3()`, `SetExpR3xSO3(v)`,
on the functions of the C++ core.

<a id="rg16-9"></a>
**RG16.9** **DONE 2026-10-04** (#2820) — [log](exudynRevisionLog2026b.md#rg16-9) *(group RG16; maintainer 2026-10-04)*
**The library builds the frames of markers with `exu.HT`, and no Create function passes `rotationMarker0/1`**: the
Create functions, `GetJointArgs`, `robotics.Robot.CreateRedundantCoordinateMBS`, the MiniExample of
`ObjectJointPrismaticX` and the advice of the deprecated `rotationMarker0/1`. The other uses of the HT functions of
`rigidBodyUtilities` in the library are RG16.10 (robotics) and RG16.11 (the rest).

<a id="rg16-10"></a>
**RG16.10** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg16-10) *(group RG16; maintainer
2026-10-04: "the robotics.Robot class (and the related classes) still use the Python HomogeneousTransformation"; on
RG16.10.1: "do as suggested. Could also be read/write indexing, if possible, but would fail on writing in the last
row")* **The robotics classes on `exu.HT`** (#2821). Since RG16.4.2 every HT a user gives
(`RobotBase(HT=)`, `RobotTool(HT=)`, `RobotLink(localHT=, preHT=)`) may be an `exu.HT` or a 4x4 matrix, and is stored
as a numpy array; the computations (`LinkHT`, `JointHT`, `COMHT`, `Jacobian`, the DH conversions `StdDH2HT`/`ModDHKK2HT`,
`dictJointType2HT`, `InverseKinematicsNumerical`) are numpy products with `HTtranslate`/`HTrotateX`/`HT0` (about 120
places in `roboticsCore`, `models`, `mobile`, `special`, `future`, `utilities`), and what they return are numpy
arrays, which scripts read as `HT[-1][0:3,3]` or with `HT2translation(HT[-1])`.
    - **RG16.10.1** **DECIDED 2026-10-04** as proposed, with indexing for writing: the robotics classes store and compute with `exu.HT`
      (products in C++), and `LinkHT`/`JointHT`/`COMHT` return lists of `exu.HT`. So that scripts keep working,
      `exu.HT` gets `__getitem__` on its 4x4 matrix (`H[0:3,3]`, `H[2][3]`; reading only), `H @ x` keeps working through
      `__array__`, and `HT2translation`/`HT2rotationMatrix` take an `exu.HT` already (RG16.4.2). The alternative: keep
      numpy outputs and use `exu.HT` inside only - less gain for the user, no break at all;
    - **RG16.10.2** **DONE 2026-10-04** `roboticsCore.py` on `exu.HT`, the forward kinematics and the Jacobian measured
      against today's (a robot of 6 and of 20 links);
    - **RG16.10.3** **DONE 2026-10-04** `models.py` (the robot definitions: DH parameters, `preHT`, tool frames), `mobile.py`, `special.py`,
      `future.py`, `utilities.py`;
    - **RG16.10.4** **DONE 2026-10-04** the scripts with the Robot class or the kinematic tree, each with its result unchanged: the examples
      `serialRobot*.py` (8), `humanRobotInteraction.py`, `InverseKinematicsNumericalExample.py`, `ROSMobileManipulator.py`,
      `mobileMecanumWheelRobotWithLidar.py`, `openAIgymNLink*.py`, `kinematicTreeAndMBS.py`, `kinematicTreePendulum.py`,
      `FurtherExamples/fourBarKinematicTree*.py`, `spotModel.py`, and the test models `serialRobotTest.py`,
      `movingGroundRobotTest.py`, `kinematicTreeAndMBStest.py`, `kinematicTreeConstraintTest.py`,
      `createKinematicTreeTest.py` (24 of the 42 scripts that used the HT functions of `rigidBodyUtilities`);
    - **RG16.10.5** the HT functions of `rigidBodyUtilities` (`HomogeneousTransformation`, `HTtranslate`, ...): kept as
      numpy helpers or deprecated with `exu.HT` as advice - **DECIDED 2026-10-04: kept** (maintainer), as numpy helpers; nothing of the library and of the
      scripts uses them now but `rigidBodyUtilities` itself, `graphics`, `plot`, `lieGroupBasics`, `robotics.mobile`,
      `special`, `future` and `utilities` (on numpy input from users) and the two `homogeneousTransformation*Test.py`.

<a id="rg16-11"></a>
**RG16.11** **DONE 2026-10-04** (#2822) — [log](exudynRevisionLog2026b.md#rg16-11) *(group RG16; maintainer 2026-10-04:
"I mean like in the solutionViewerTest.py. These are few examples or test models ... a couple of scripts are
definitely sufficient - don't make the scripts more complicated than necessary")* **Scripts that build rigid frames
use `exu.HT`**: the 18 scripts that used the HT functions of `rigidBodyUtilities` without the robotics classes did so
mostly for the `localHT` of a marker, one or two lines each; 14 of them give it with `exu.HT` now, every result unchanged
(`solutionViewerTest.py` with RG16.12.2). Not changed: the two `homogeneousTransformation*Test.py`, which compare
`exu.HT` with those functions on purpose, and the `HT` argument of `PlotImage` in two NGsolve examples (a numpy
function). The other 24 scripts use the robotics classes or the kinematic tree: RG16.10.4.

<a id="rg16-12"></a>
**RG16.12** *(group RG16; maintainer 2026-10-04: "a couple of examples and test models (like 4+4) should use
referenceHT / localHT, just to show how it works and for the tests")* **Examples and test models that show
`referenceHT`, `localHT` and `exu.HT`** (#2823).
    - **RG16.12.1** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg16-12-1) the new example
      `homogeneousTransformationInterpolation.py`: four rigid bodies, each held by a generic joint to a ground whose
      `referenceHT` a PreStepUserFunction sets with `InterpolateSO3` and `InterpolateSE3`, the solution stored every
      5 ms for the SolutionViewer (reworked after the maintainer's look, 2026-10-04: bodies, not grounds);
    - **RG16.12.2** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg16-12) `solutionViewerTest.py`: the chain
      of 100 bodies built from `exu.HT` products, the bodies with `referenceHT`, the joints on markers with `localHT`;
    - **RG16.12.3, .4** **DONE 2026-10-04 by RG16.11** — the examples `mouseInteractionExample.py` and
      `bicycleIftommBenchmarkMarkerBasedJoints.py` and twelve test models give `localHT` with `exu.HT` now; the
      parameters themselves are tested by `homogeneousTransformationParameterTest.py` (referenceHT of ground and bodies,
      localHT of the rigid markers, jointHTs) and `homogeneousTransformationTest.py` (the output variable and sensor);
    - **RG16.12.5** **DONE 2026-10-04 by RG17.2.7** — [log](exudynRevisionLog2026b.md#rg17-3) the notebook
      `tutorialRigidBodyCreate` replaces `rigidBodyTutorial3.py`: it names `referenceHT` and shows the frame of a body as
      `exu.HT`.

<a id="rg16-13"></a>
**RG16.13** *(group RG16; maintainer 2026-10-04: "ObjectKinematicTree still has only jointTransformations and
jointOffsets, but I believe that jointHTs would be much more convenient and could also boost the internal
computations (?)")* **DONE 2026-10-04** **`ObjectKinematicTree` and its `jointHTs`** (#2824). The parameter exists since RG16.4.1 (#2798):
`jointHTs` takes a list of `exu.HT` or 4x4 matrices (the type `HomogeneousTransformationList` of the definitions) and
is a view of the two stored lists `jointTransformations` and `jointOffsets`; `Robot.CreateKinematicTree` gives it.
    - **RG16.13.1** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg16-13) `mbs.CreateKinematicTree` passes
      `jointHTs`, `TreeLink.jointHT` is an `exu.HT` (given as `exu.HT` or 4x4 matrix), `exu.HT(H)` copies an HT, and
      the MiniExample of `ObjectKinematicTree` gives `jointHTs=[exu.HT()]`;
    - **RG16.13.2** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg16-13-2) whether storing one HT per joint
      in `CObjectKinematicTree` saves time: computing the joint transformations `XL` once instead of per evaluation
      changed nothing measurable on a tree of 6 and of 50 links. *(Corrected 2026-10-04, the maintainer: `Transformation66`
      is not a 6x6 matrix but the C++ HT, a typedef in `KinematicsBasics.h` - the tree already computes with HTs, through
      wrappers named after the Pluecker matrices; the work is RG16.13.5 to RG16.13.9.)*
    - **RG16.13.3** **DONE 2026-10-04** with RG16.10: `Robot.CreateKinematicTree` gives `jointHTs` of `exu.HT`;
    - **RG16.13.4** **DONE 2026-10-04** (#2827) `mbs.CreateKinematicTree` with automatic graphics failed for a link
      without offset (`UnboundLocalError: gLink`), found by the measurement of RG16.13.2.
    - **RG16.13.5** **DONE 2026-10-04** (#2828) — [log](exudynRevisionLog2026b.md#rg16-13-5) the unused 6x6 Pluecker
      matrices removed from `KinematicsBasics.h`: the `#ifndef USE_EFFICIENT_TRANSFORMATION66` branch (about 300 lines,
      never compiled, the switch always defined) and the switch itself; `Transformation66` is `HomogeneousTransformation`.
    - **RG16.13.6** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg16-13-6) *(maintainer 2026-10-04: "do as far as possible RG16.13.6-.9")* the functions that matter (#2829): `ComputeTreeTransformations` (positions,
      velocities and accelerations of all links: the output variables, markers and sensors) and
      `ComputeMassMatrixAndODE2LHS` (the composite rigid body algorithm and the recursive Newton-Euler terms), with
      `ComputeJacobian`, `AddExternalForces6D` and the `Get...KinematicTree` functions; what they use of
      `RigidBodyMath` today: `T66toRotationTranslationInverse` (12x), `T66Mult` (6x), `RotationTranslation2T66` (6x),
      `RotationTranslation2T66Inverse` (4x), `T66MultTransposed`, `T66MultInertia`, `MultT66SkewMotion` (2x each),
      `T66TransformInertia`, `T66SkewForce`, `T66MultTransposedInverse`, `T66MotionInverse`, `MultT66SkewForce`,
      `InertiaT66FromInertiaParameters` - written down per function: which transformation it needs, in which direction
      (Featherstone's `Xup` maps from the parent to the link, the inverse of the HT that places the link);
    - **RG16.13.7** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg16-13-6) two local implementations of those two functions in `CObjectKinematicTree.cpp` on the HT directly:
      positions and rotations with `HomogeneousTransformation` products, motion and force vectors as pairs of `Vector3D`
      (no `Vector6D`, no wrappers), selected by a flag of `exudyn.experimental` (a switch for the testing, not a
      setting), the old path as it is;
    - **RG16.13.8** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg16-13-6) the comparison: every test model and MiniExample of the kinematic tree with both paths (equal to
      round-off), and the time per evaluation on trees of 6 and of 50 links;
    - **RG16.13.9** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg16-13-9) if the HT path agrees and is not slower: it becomes the only one, the switch goes, and the T66
      functions of `KinematicsBasics.h` go with it - except what the 6D motion and force algebra still needs, possibly in a
      more suitable form;
    - **RG16.13.10** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg16-13-6) (#2845) the `forceUserFunction` of
      `ObjectKinematicTree` received an empty vector as `q_t`, found while moving the forces per joint into one function.

## RG17 — Notebooks

*(Group created by the maintainer, 2026-10-03.)* Tutorials, and some sections of examples, as notebooks
(Jupyter or similar) - a tutorial is read and run step by step, which a notebook shows and a script does not.

<a id="rg17-1"></a>
**RG17.1** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg17-1) *(group RG17; maintainer 2026-10-03:
"evaluation step first")* **Evaluate notebooks for tutorials and examples** (#2811), before anything is converted: what the documentation build needs (`myst-nb` or `nbsphinx`, executed
or stored output, the PDF), how a notebook is tested (`runTestExamples.py`, `nbval`, or a converted `.py`), the renderer
and `PlotSensor` inside a notebook (no window: images, or an interactive viewer), the size of the repository with stored
outputs, and which tutorials and example sections first. Ends with a recommendation for the maintainer. A first
candidate: `rigidBodyTutorial3.py` with `exu.HT` (RG16.12.5).
    **Recommendation** (the measurements in the log), **for the maintainer's decision** - the points marked (?):
    - **format**: Jupyter `.ipynb`, stored **without outputs** (a check in `exudev generate --all-checks` refuses a
      notebook with outputs; the one notebook in the repository, `CMSexampleCourseJupyter.ipynb`, holds 120 kB with 11
      outputs); one file per tutorial, no paired `.py` (one place, rule 10);
    - **place** (?): `python/Notebooks/`, beside `Examples/` - or the tutorial pages themselves as notebooks in
      `docs/manual/`, which would make the page and the script one file;
    - **documentation**: `myst-nb` (dev only), which extends the `myst_parser` the build uses already, the notebooks
      executed at build time with a cache (`nb_execution_mode = "cache"`, with the installed exudyn of `venvExuP313`),
      so that the pages show current outputs and the repository holds none; the PDF takes the same output (the LaTeX
      builder of myst-nb), checked once;
    - **test**: `runTestExamples.py` runs the code cells of each notebook as a script, read with `json` (no new
      dependency, the same environment flags and log as the examples); `nbval` is not needed;
    - **the scene in a notebook**: no window - `SC.renderer.RedrawAndGetImage(useRaytracer=True)` after `ZoomAll()` or
      `SetModelView`, shown with `matplotlib.pyplot.imshow` (400 x 300 in 0.02 s, measured); a small helper (?) in
      `exudyn.interactive` (`ShowImage(SC)`), and a sequence of such images for a motion; the OpenGL renderer and the
      SolutionViewer open their own window when the notebook runs locally, and are skipped in the documentation build;
      `PlotSensor` shows inline as matplotlib does;
    - **first**: `rigidBodyTutorial3` (RG16.12.5), then the spring-damper tutorial (the first one a reader meets, RG3.27);
      the others after the first two have been looked at.
    **Decided (maintainer 2026-10-04)**: `python/Notebooks/`; the tutorial pages of the documentation and the PDF are
    the *views* of the notebooks; the outputs are **stored** - stale outputs in the hand-written tutorials are what
    stored, regenerated outputs avoid; `exudyn.interactive.ShowImage(SC)` yes; 1-2 tutorials first, as extra tutorials,
    to compare; no tutorial twice - the example scripts are generated from the notebooks; the old tutorial files go after
    the cleanup, the three rigid body variants shrunk to one alternative; a suggestion for the other examples of the
    documentation and, most important, a systematic way for the examples of `definitions/pybind*.py`.

<a id="rg17-2"></a>
**RG17.2** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg17-2) *(group RG17; after the decision on RG17.1)*
**The notebook tooling and the first notebooks** (#2831), without `myst-nb` - a converter of our own, as for every other
page (the package `myst` that was installed is not `myst-nb` and is not needed; `nbformat` is not needed either, a
notebook is JSON):
    - **RG17.2.1** `exudyn.interactive.ShowImage(SC, size, modelRotation, zoomAll, show, fileName)`: the scene as an image
      of the raytracer, no window, shown with matplotlib;
    - **RG17.2.2** `tools/runNotebooks.py`: runs each notebook in an interpreter of its own (no window, files to a
      temporary directory) and stores the outputs in it - printed text, the value of the last expression, the figures as
      PNG; no Jupyter package needed;
    - **RG17.2.3** `tools/generators/notebookEmitter.py`, a stage of the regeneration: per notebook the page
      `docs/generated/notebooks/<name>.md` (code, stored outputs, images) and the example script
      `python/Examples/<name>.py` with a generated header - so `runTestExamples.py` runs every notebook; cells tagged
      `remove-cell` (Jupyter's cell tags) run but are not shown, `remove-input` shows only the output;
    - **RG17.2.4** the first two, as extra tutorials beside the old ones (`docs/manual/tutorial.md`):
      `tutorialSpringDamper.ipynb` (exact solution and Exudyn in one plot) and `tutorialRigidBody.ipynb` (frames with
      `exu.HT`, images of the reference and the final state);
    - **RG17.2.5** the systematic way for the examples of the reference manual, shown on one: `pb.AddDocuNotebook(path)`
      in `definitions/pybind*.py` instead of `pb.AddDocuCodeBlock(code)` - the code cells, the Markdown cells as text and
      the stored text outputs on the page; the example of `exu.HT` is `python/Notebooks/reference/HT.ipynb`.
    - **RG17.2.6** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg17-2-6) *(maintainer 2026-10-04)* every
      notebook names itself, `python/Notebooks/<name>.ipynb` in code font, in its first cell (the emitter and
      `AddDocuNotebook` refuse one that does not); plots stored at 12.8 x 6.4 inch (8:4) and `ShowImage` images at twice
      the size, both shown at half their pixels; the scripts in `python/Examples/notebooks/`;
    - **RG17.2.7** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg17-2-6) all tutorials as notebooks, with the
      text of the tutorial pages: `tutorialSpringDamper` (items) and `tutorialSpringDamperCreate`, `tutorialRigidBody`
      (markers and joints) and `tutorialRigidBodyCreate`, `tutorialFlexibleBeams`, `tutorialSymbolic`, `tutorialFFRF`;
      the old pages in a second table of contents until RG17.3 removes them.

<a id="rg17-3"></a>
**RG17.3** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg17-3) *(group RG17; after the maintainer has
compared the notebooks with the old tutorials; maintainer 2026-10-04: "the new RG17.3 tutorials are really good -
remove the old ones")* **The tutorials are the notebooks** (#2831): the old pages `docs/manual/tutorial*.md` removed, with their figures that no notebook
uses; the old scripts removed (*ask before deleting*): `springDamperTutorial.py`, `springDamperTutorialNew.py`,
`rigidBodyTutorial.py`, `rigidBodyTutorial2.py`, `rigidBodyTutorial3.py`, `rigidBodyTutorial3withMarkers.py` and
`beamTutorial.py` - the alternative built from nodes, objects and markers is the notebook `tutorialRigidBody`;
`exudev notebooks` to run them (the tool of RG17.2.2), and `runNotebooks.py --check` in the release checks: a notebook
whose code changed since its outputs were stored.

<a id="rg17-4"></a>
**RG17.4** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg17-4) *(group RG17; maintainer 2026-10-04: "most important: the definitions/pybind... examples ... a systematic way
would make a lot of sense")* **Every example of the reference manual a notebook** (#2831): the 36 `AddDocuCodeBlock`
of `definitions/pybind*.py` (General information 10, MainSystem 8, symbolic 8, data structures 3, one each in enums,
general contact, module, renderer, system container, system data, types) become notebooks in
`python/Notebooks/reference/`, one per class or section; what a snippet needs but does not show (a system with a body,
a solved model) is a cell tagged `remove-cell`; the text before a block that only introduces it moves into the notebook
as Markdown. Then: the reference notebooks run in the test suite (a pytest that executes their code cells), and the
examples cannot go stale. *(Maintainer 2026-10-04: "only if it works out straight-forward. If a couple of them would be
too complicated, you can also keep them in the old format. But a clean test for all is certainly an added value.")*
Of the 36, 24 are found by their `code="""..."""`: 11 in `pybindMainSystem.py`/`pybindSystemData.py`/
`pybindSystemContainer.py`/`pybindRenderer.py` are short models that can run as they are; the 10 of
`pybindGeneralInformation.py` are fragments of one introduction (imports, a system) and an error message - one
notebook with them as cells, the error kept as text. *(Maintainer 2026-10-04: "do RG17.4 as proposed")*
    - **RG17.4.1** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg17-4) 30 of the 34 code blocks are five
      notebooks of `python/Notebooks/reference/` - `generalInformation` (with the examples of `pybindModule.py` and
      `pybindEnums.py`), `mainSystem` (with `systemData` and `GeneralContact`), `systemContainer` (with the renderer and
      the materials), `symbolic`, `matrixContainer`; one notebook per definition file or class, each example a part
      (cells tagged `part-<name>`) that `AddDocuNotebook(path, part=...)` shows where the block was, followed by the
      name of the notebook; kept as code blocks: the three error messages of *Exceptions and error messages* and the
      copy example (RG17.4.3);
    - **RG17.4.2** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg17-4) the test:
      `python/testing/test_referenceNotebooks.py` runs every reference notebook (`runNotebooks.py --test`, nothing
      stored) and checks that every notebook stores the outputs of its current code; running them found the
      errors listed in the log, among them #2841 (fixed) and #2842;
    - **RG17.4.3** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg17-4-3) *(maintainer 2026-10-04: "continue
      with 1-3")* (#2842) `SC.AppendSystem` of a `MainSystem` that Python owns (a copy, or the system of
      another container) is deleted twice - an access violation at the end of the copy example of *Copying and
      referencing C++ objects*; the container shall delete only the systems it created and keep an appended one alive;
      then the copy example becomes a part of `generalInformation.ipynb`.

<a id="rg17-5"></a>
**RG17.5** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg17-5) *(group RG17; maintainer 2026-10-04: "a
suggestion for the other examples in the docs ... would probably require a part that is not shown"; on the proposal:
"sounds good. 70 is really a lot, so I ask to see if some of them can be fusioned into one example, with the text in
between or restructuring the text a little bit to fit into less notebooks. Julia can't be tested; graphicsData could go
into one example with all cases at the end, refering to that in the GraphicsData section")* **The snippets of the user
manual** (#2831): the manual pages hold
about 70 Python blocks besides the tutorials - `introductionAdvanced.md` 17, `GUI.md` 15, `userSettings.md` 6,
`performanceErrors.md` 4, `introductionBasics.md` 4, `gettingStartedFAQ.md` 4, `resultsMonitor.md` 2,
`gettingStartedExample.md` 2. Most are fragments (one call with a comment) that only make sense in a model; they become
notebooks of `python/Notebooks/snippets/`, one per page, with the model they need as `remove-cell`, and the page includes
the converted cell where the block stands (a MyST `{include}` of a fragment the emitter writes, or a directive of our
own). Blocks that are not code to run - the FAQ's error messages, a command line - stay as they are.
    - **RG17.5.1** **DONE 2026-10-04** — [log](exudynRevisionLog2026b.md#rg17-5) three notebooks of
      `python/Notebooks/snippets/` hold 25 of the 55 Python blocks: `graphics` (the five `GraphicsData` examples and the
      raytracing snippet as one model, shown once at the end of the section *GraphicsData*), `solving` (simulation
      settings, output, the renderer around the solver, the errors of a solve: 7 blocks in 6 parts), `visualization`
      (the renderer page: 12 blocks in 11 parts); the other 30 stay - the eleven of Julia, the outputs and console
      sessions, and the code that writes the user's settings or opens windows (see the log).

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
| RG2.3 | #2582 | the graphics regression suite |
| RG2.4 | - | the manual GUI check, once per release and platform (list and model done) |
| RG4.1 | - | the Windows/Linux differences in contact and friction; RG4.1.2 the five macOS-only models, RG4.1.3 the math library |
| RG4.15 | #2848, #2849 | the open bugs before 1.13: `GeneralContact`, the contact model of its implicit solver (decision) and the torque on triangle bodies |
| RG12.39 | #2850 | a restart from the restart file: how it works with a model script, then a proposal |
| RG3.36 | #2856 | the PDF checked with the documentation |
| RG4.20 | #2859 | ANCF thin plate of a colleague: the visualization (RG4.20.3) and four decisions (RG4.20.4) |
| RG4.19 | #692, #1290, #1337, #2851 | the open bugs and checks, evaluated: options and decisions (RG4.19.1-.9) |
| RG5.1 | - | a maintained micro-benchmark of the linear algebra, inside Exudyn (from #2397); RG5.1.1 the no-rotation flag of the HT |
| RG5.2 | - | make the hot linear algebra vectorizable |
| RG6.8 | #2140, #2236, #2237, #2350 | the graphics fixes before 1.13: Linux (RG6.8.5) and macOS (RG6.8.6), which wait for those machines |
| RG8.1 to RG8.9 | - | the plugin ABI: registry, fingerprint, reference plugin, headers, discovery |
| RG13.3 | #2717 | each description synchronized once with its implementation, recorded with a fingerprint |
| RG14.3 | #2745 | joints and their Jacobians on homogeneous transformations, after RG14.2.9 |
| RG15 | #2746 | objects computing from coordinates passed in: the work after the evaluation of RG15.1, not planned yet |

<a id="not-decided"></a>
### Not decided to be resolved

Tasks put off by a decision of the maintainer, or because there is no straightforward way, while the rest of their
step is done. They are kept here, out of the list above; their issue is resolved for what is done, or closed with the
reason, and a case that needs one of them opens a new issue that names the step.

| step | issue | what it is | why not now |
|---|---|---|---|
| RG4.12 | #2736, closed | `NodeGenericAE` as the owner of Lagrange multipliers or of linear state space systems | on hold (maintainer, 2026-09-29): "it will be used in the future" - a case re-opens it |
| RG6.7.1 | #2709, resolved | a raytracer for curved geometry (instead of the flat split) | not planned: the split is accurate as fine as the tiling angle asks |
| RG6.7.2.1 | #2709, resolved | the faces of Hex20 meshes curved (`VolumeToSurfaceElements`) | an 8-node face has no node on a diagonal, which a 6-node triangle needs; it would need points that are not mesh nodes |
| RG9.4.3.2 | #2202, resolved | energy of the contact and special objects | not now (maintainer, 2026-10-01); with friction it may be difficult |
| RG13.6.6 | #2732, closed | MiniExamples of `ObjectFFRF` and `ObjectFFRFreducedOrder` | later (maintainer, 2026-09-29): when tetrahedral elements are part of Exudyn |
| RG14.2.9.4 | #2745 | the term $\partial(\Cm_\qv\tp\lambdav)/\partial\qv$ in the Newton matrix of the joints | on hold (maintainer, 2026-10-02): the gain is limited to large reaction forces at large rotations |
| RG14.2.12 | #2745 | `GeneralContact` on the marker interface | measured, not now: no gain found that would pay for the change |
| RG16.3.4 | - | a translation in the `localHT` of `MarkerNodeRigid` | when a case needs it: the position Jacobian and its derivative with an offset in the node; `MarkerBodyRigid` has it |

Open in the tracker without a step: #2498 (nothing checks that an item type provides the member
functions it must) and #2511 (the ROS examples were last run in 2023), both named in RG2.

### Raised by the current work, and not yet a step

Each of these is written down where it was found; none is planned, and the maintainer decides
whether it becomes a step.

| where | issue | what it is |
|---|---|---|
| RG16.6.3 | #2809, resolved | `GetPosition2D`/the angle of a planar frame: considered, implemented only if a planar model needs it |

*#2608 was done by RG6.2.11. The decision on the chapters of the user manual
(#2657, #2662), which stood below, is carried out and is in the
[log](exudynRevisionLog2026b.md#decisions-2026-09-29).*

### Recommended next

The title of each says what the step **does**; the sentence after it says why it comes here.

1. **Run the integration round of the institute, then release 1.13** (RG2.2, RG1.4). It is the only item on this
   page that needs **other people's time**, so it starts before the rest is ready, not after.
2. **Do the manual GUI check on Windows** (RG2.4), with the curved GraphicsData (row K13) and the TikZ figures in
   the PDF. It is the last condition of 1.13 that one person can meet alone.
3. **Finish the steps that are nearly done**, each small and without a decision left: none left at the moment.
4. **Then the larger open steps of 1.13**: none in RG4 at the moment; RG4.19.4-.6 and .10 are open options.
