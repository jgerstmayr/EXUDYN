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
    **The GraphicsData features drawn since 1.12** are part of it (maintainer 2026-10-01; row K13, model
    `python/Examples/graphicsCurvedShapes.py`): the type `Spheres` (also `graphics.Sphere`), the 6-node triangles
    (`triangles6`) and their curved face edges, the quadratic edges (`edges3`) and lines (`Lines` with `shape`
    `'quadratic'`), the two tiling settings `openGL.advanced.curvedTriangleTilingAngle`/`curvedTriangleMaxTiling`
    changed in the running window, and all of it in the raytracer - what the graphics tests can only check as data.
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
      - **RG2.3.3.7** *(maintainer 2026-09-30)* **an image per item for its page**: 800 x 600, generated
        automatically by the raytracer, the white border cropped, a 3D view; selected by hand where the image
        fits the description - the others stay without one for now. The definition of the item names the
        file (a field such as `image='itemImages/ObjectRigidBody.png'`), stored in `docs/figures/itemImages/`.
        The evaluation run of 2026-09-30 (`tmp/miniExampleImages/`) showed what the MiniExamples need first:
        the raytracer draws no spheres (RG6.7.3), most bodies have no graphics of their own, and the node
        frames and load arrows dominate.
      - **RG2.3.3.8** *(found 2026-10-02)* **the raytracer can hang in `RedrawAndGetImage`** (#2776): twice the full
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
gaps it names are the first candidates. The maintainer's own findings go here as steps.

*No steps yet.*


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
**RG3.8.1** **DONE 2026-09-24** (#2594) — [log](exudynRevisionLog2026b.md#rg3-8-1) · [plan text](exudynRevisionLog2026b.md#plan-rg3-8-1) — The three contact-friction figures are vector.

<a id="rg3-8-2"></a>
**RG3.8.2** **DONE 2026-09-25** (#2594, #2650) — [log](exudynRevisionLog2026b.md#rg3-8-2) · [plan text](exudynRevisionLog2026b.md#plan-rg3-8-2) — Three more figures are vector, two became text, and what they replaced is out of the tree.

<a id="rg3-8-3"></a>
**RG3.8.3** **DONE 2026-09-25** (#2651) — [log](exudynRevisionLog2026b.md#rg3-8-3) · [plan text](exudynRevisionLog2026b.md#plan-rg3-8-3) — Three item pictures were in the repository and on no page.

<a id="rg3-8-4"></a>
**RG3.8.4** **DONE 2026-09-26** (#2594) — [log](exudynRevisionLog2026b.md#rg3-8-4) · [plan text](exudynRevisionLog2026b.md#plan-rg3-8-4) — The four lost figures are three, and they are back.

<a id="rg3-8-5"></a>
**RG3.8.5** **DONE 2026-10-03** — [log](exudynRevisionLog2026b.md#rg3-8-5) (#2594) *(from RG3.8; measured 2026-09-26)* **The seventeen vector originals whose png the
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


<a id="rg3-28"></a>
**RG3.28** **DONE 2026-09-29** (#2743) — [log](exudynRevisionLog2026b.md#rg3-28) · [plan text](exudynRevisionLog2026b.md#plan-rg3-28) — The developer documentation named paths of the old `main/` directory.

<a id="rg3-21"></a>
**RG3.21** **DONE 2026-09-27** (#2673) — [log](exudynRevisionLog2026b.md#rg3-21) · [plan text](exudynRevisionLog2026b.md#plan-rg3-21) — The pages that still describe the state before a step that is done.

<a id="rg3-22"></a>
**RG3.22** **DONE 2026-09-27** (#2659) — [log](exudynRevisionLog2026b.md#rg3-22) · [plan text](exudynRevisionLog2026b.md#plan-rg3-22) — The simulation settings section says how to look a setting up.

<a id="rg3-19"></a>
**RG3.19** **DONE 2026-09-26** (#2665) — [log](exudynRevisionLog2026b.md#rg3-19) · [plan text](exudynRevisionLog2026b.md#plan-rg3-19) — The arguments of a documented function are one per line, with the name in code.


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
**RG4.3** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg4-3) - a warning and the documentation, no change of the
    solver. *(group RG4; revision2026 step R10.3)* **Explicit integration cost** (#2398, #2400). With the default dense linear solver
    an explicit step on a chain of point masses costs O(N^2) (168 ms per step at N=2000; 400 times
    faster with `EigenSparse`), and `computeMassMatrixInversePerBody` changes nothing unless a
    sparse solver is selected as well. At least warn at large N; better, avoid the global solve
    in explicit integration where the flag makes it unnecessary.

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
    - **RG4.15.8** (#1848, #1947) `GeneralContact`: implicit sphere-triangle contact and its friction against
      `ObjectContactSphereSphere` - the drop of RG4.15.1 as a fourth case.

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
    - **RG4.17.2** *(open, needs no decision)* **the stall with a consistent Jacobian**: from load step 7 (3 % of the
      drive) the residual grows by a constant factor 1.41 per Newton iteration, with the analytic and with the
      system-wide numerical Jacobian alike; the condition number of the system Jacobian is 5e12 (the thin section,
      $w/h = 1/50$, with `crossSectionPenaltyFactor = [1,1,1]`). Next: the eigenvalues of the tangent stiffness at the
      stall against those of the geometrically exact beam - a zero or negative one is a bifurcation of the ANCF model
      (a cross-section mode), not a solver problem -, then the penalty factor and the number of elements.

<a id="rg4-18"></a>
**RG4.18** *(group RG4; maintainer 2026-10-02)* **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg4-18) - **A system
    without coordinates is solved** (#2790): nODE2 = nODE1 = 0 (and no algebraic equations) - at least the explicit
    solvers; check where it breaks.

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
**RG6.7** **DONE 2026-10-03** — [log](exudynRevisionLog2026b.md#rg6-7-done) *(group RG6; maintainer 2026-09-27)*
    **GraphicsData gets a Sphere and a CurvedTriangleList** (#2709). Bigger than it sounds, because every consumer of the graphics data
    has to follow - even the minimal implementation with temporary workarounds: the GraphicsData
    classes and their dictionary, the OpenGL renderer, the raytracer, the pybind interfaces,
    `SC.renderer.GetGraphicsData()`, the documentation, and the graphics regression test (RG2.3.3).

    **A limitation to resolve with it**: spheres are already special. The OpenGL renderer treats the
    spheres of nodes separately, because there can be very many of them; the **raytracer does not draw
    `glSpheres` at all**; `GetGraphicsData()` does return them (measured 2026-09-27). A Sphere that is
    fully part of GraphicsData has to be drawn the same way by all three.

    - **RG6.7.1** **DONE 2026-09-30, decided** — [log](exudynRevisionLog2026b.md#rg6-7-1) - what the sphere
      can do, and what the curved triangle is (#2710). **Decided (maintainer, 2026-09-30)**:
      - the element is the **6-node quadratic triangle with optional normals at its six nodes**;
      - it is **split into flat triangles when the graphics data is drawn**, the normals interpolated with
        the shape functions; the split is **adaptive**: the angle between the normals of a triangle (given or
        computed from the geometry), approximated cheaply by $|\nv_i \times \nv_j|$, against a threshold
        angle, with a maximum number of subdivisions per direction - two settings, **one global setting each**,
        in `openGL.advanced` although the raytracer reads them as well: `curvedTriangleTilingAngle` (degrees,
        default 3) and `curvedTriangleMaxTiling` (default 5), names to be confirmed by the implementation;
      - **the raytracer does the same**: the normals were never the problem, the flat shape of the sub-triangles
        is; a raytracer for curved geometry would be the real answer and is not planned;
      - **the sphere is a GraphicsData type, and the nodes are drawn with it** - a node is no separate graphics
        feature any more but gets a sphere shape; the order of drawing stays, so that with transparent faces
        the nodes are seen through the objects.
    - **RG6.7.2** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg6-7-2) - the 6-node triangle: the
      dictionary (`TriangleList` with six indices per triangle and optional normals per point), the adaptive split
      with the two settings, `GetGraphicsData()` returning the split, `exudyn.graphics` helpers
      (`NGsolveMesh2PointsAndTrigs(..., triangles6=True)`, `FromPointsAndTrigs` with six columns);
      - **RG6.7.2.1** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg6-7-2-1) - the superelements with six
        columns in `triangleMesh` (the FFRF bodies and the FEM surface of quadratic meshes, `FEMinterface`), whose points
        deform in every frame; and the contour colors on 6-node triangles (`AddBodyGraphicsDataColored` applied them to
        flat triangles only); the surfaces that `VolumeToSurfaceElements` builds for Tet10 (Abaqus imports) get their
        6-node triangles (2026-10-03, [log](exudynRevisionLog2026b.md#rg6-7-done)); a Hex20 face is drawn by its corners -
        its 8 nodes have none on the diagonals a split into triangles needs (*not decided to be resolved*, see the list
        below);
      - **RG6.7.2.2** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg6-7-2-2) - **the split when drawing**, as
        decided in RG6.7.1 (the first implementation split when the graphics data was built, a misunderstanding):
        `GraphicsData` keeps `glTriangles6` only; OpenGL splits per frame, the raytracer per image,
        `GetGraphicsData()` per call, each with the settings of that moment - a change of
        `curvedTriangleTilingAngle` shows at once. The edges (`showFaceEdges`) are the curved edges, not those of the
        split. Defaults **15°** (24 segments around a full cylinder) and at most **8** subdivisions;
      - **RG6.7.2.3** **DONE 2026-10-03, measured** — [log](exudynRevisionLog2026b.md#rg6-7-done) - the split cached per
        GraphicsData for OpenGL (`SplitTriangles6Cached`), 60 ms per frame for $9\cdot10^4$ triangles6 saved - the cost of the split per frame for large quadratic meshes (an NGsolve
        surface of $10^5$ triangles6 at 15°: up to 64 flat triangles each); if it shows, cache the split per
        GraphicsData, invalidated by the graphics update and by the two settings;
    - **RG6.7.3** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg6-7-3) - the sphere type in GraphicsData,
      drawn by OpenGL, the raytracer (a ray-sphere intersection in its search tree) and `GetGraphicsData()`; the
      nodes drawn through it, in the order of today;
      - **RG6.7.3.1** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg6-7-2-2) - the raytraced sphere fell
        apart into rings (maintainer's screenshot): $|\mathbf{o}-\mathbf{c}|^2-r^2$ cancels in float when the camera
        is far from a small sphere; now the distance of the center from the ray, which is stable;
    - **RG6.7.4** the graphics tests (RG2.3.3) and the documentation grow with both - **DONE with RG6.7.2 and
      RG6.7.3**: the cases `Sphere`, `Spheres` and `Triangles6` of `testEveryGraphicsFunction`, the manual
      (*GraphicsData: Spheres*, the key `triangles6` of *GraphicsData: TriangleList*).
    - **RG6.7.5** **DONE 2026-10-03** — [log](exudynRevisionLog2026b.md#rg6-7-done) *(proposed by the maintainer
      2026-10-01)* - **anisotropic tiling**: a cylinder patch is curved in
      one direction only, but the split subdivides both, $n^2$ triangles where $2n$ would do. The way: a number of
      subdivisions **per edge**, $n_{01}, n_{12}, n_{20}$, each from the angle between the normals of that edge's three
      nodes; the interior triangulated to match the three edge counts (rows of strips between the two most subdivided
      edges, the third edge's points joined by a fan). The counts must come from what two neighbours share - the
      edge's own nodes and their given normals, or, without normals, the edge's tangents at its ends - so that the
      shared edge is split alike on both sides and no cracks appear; the isotropic split of today has that problem
      already (one $n$ per triangle) and would lose it. A cylinder of 6-node triangles then needs $2n$ instead of
      $n^2$ flat triangles per element.
    - **RG6.7.6** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg6-7-6) - *decided (maintainer 2026-10-01): switched for 1.13* - **`TriangleList` and `Spheres` as $(n\times 3)$ arrays**: points,
      normals, triangles (and $(n\times 4)$ colors, $(n\times 6)$ triangles6) as rows, not flat lists. Measured
      2026-10-01: the C++ reader (`PyWriteBodyGraphicsDataList`) casts each key to a flat `std::vector<float>` and
      **rejects** a nested list or a 2D array today, so the flat form is all there is. Sub-steps:
      - **RG6.7.6.1** the reader accepts both - flat as now, and rows of 3 (4, 6), as list or 2D numpy array;
      - **RG6.7.6.2** the documentation (manual *GraphicsData*, the docstrings) shows only the rows;
      - **RG6.7.6.3** `exudyn.graphics` returns rows (`Brick`, `Cylinder`, `FromPointsAndTrigs`, `Transform`,
        `MergeTriangleLists`, ...) and reads both; `graphicsDataUtilities.py` likewise;
      - **RG6.7.6.4** the read-back (`mbs.GetObject(..., addGraphicsData=True)`, `GetBodyGraphicsDataList`) returns rows.
      - **RG6.7.6.5** the note in `docs/manual/revisions.md`: a script that indexes a returned list as flat
        (`g['points'][3*i+1]`) must reshape it, `np.array(g['points']).reshape(-1,3)` works for both forms.
      **Decided (maintainer, 2026-10-01)**: the returned form switches **already for 1.13**, not in 2.0. `Lines` and
      `edges3` read rows since RG6.7.7.2; `PyReadNumbers<T>` (`VisualizationSystemContainer.cpp`) is the reader the other
      keys get in RG6.7.6.1.
    - **RG6.7.7** *(maintainer 2026-10-01)* **Quadratic (3-node) lines and edges**, the line counterpart of the 6-node
      triangle. Today a `TriangleList`'s `edges` are point pairs, drawn as straight `GLLine`s, and `Lines` takes two
      points per line - a feature edge on a curved surface of 6-node triangles (the rim of a cylinder) can only be a
      chord. Sub-steps:
      **RG6.7.7.1 to RG6.7.7.5 DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg6-7-7).
      - **RG6.7.7.1** `TriangleList` gets the key **`edges3`**: three point indices per edge, `[p0, p1, m01, ...]` - the
        corners first, then the mid node, the order of `triangles6`; points and colors shared with the triangles,
        `edgeColor` as for `edges`. Drawn always, like `edges`; `showFaceEdges` keeps drawing the element edges of the
        6-node triangles.
      - **RG6.7.7.2** `Lines` gets the key **`shape`**: `'linear'` (the default, also when the key is missing; two points
        per line) or `'quadratic'` (three points per line, `[p0, p1, m01]` as for `edges3`; colors per point as now);
        later other shapes in the same way (splines). The points (and colors) as rows, $(2n\times3)$ / $(3n\times3)$
        and $(2n\times4)$ / $(3n\times4)$, with the flat lists still read - as for every GraphicsData (RG6.7.6).
      - **RG6.7.7.3** C++: **`GLLine3`** (three points, three colors, item) and a list `glLines3` in `BodyGraphicsData`
        and `GraphicsData`, kept as they are and split when drawn (OpenGL, raytracer as lines,
        `ComputeMaxScene`), by one function beside `SplitTriangle6`: the number of segments from the angle between the
        curve's end tangents $\tv_0 = 4\mv - 3\pv_0 - \pv_1$, $\tv_1 = 3\pv_1 + \pv_0 - 4\mv$ against
        `curvedTriangleTilingAngle`, at most `curvedTriangleMaxTiling` - a quarter circle at 15° gets 6 segments, as
        the edge of a neighbouring 6-node triangle does. The count depends only on the three points of the edge,
        which two neighbours share, so it is the edge rule RG6.7.5 needs as well. The curved element edges of the
        6-node triangles (RG6.7.2.2) use the same function.
      - **RG6.7.7.4** **`SC.renderer.GetGraphicsData(flatShapes=False)`**: by default the **native** shapes - the
        6-node triangles under `triangles6` and the quadratic lines under `lines3`, beside the flat `triangles` and
        `lines`; with `flatShapes=True` everything flat by **one fixed refinement** (each 6-node triangle into the 4
        triangles on its six nodes, each quadratic line into 2 lines), independent of the tiling settings. This
        replaces the split by the current settings that `GetGraphicsData()` returns since RG6.7.2.2, and makes the
        graphics references independent of `curvedTriangleTilingAngle`.
      - **RG6.7.7.5** the `exudyn.graphics` helpers keep `edges3` and the line shapes (`MergeTriangleLists`,
        `Transform`/`Move`, `InvertTriangles`); `Triangles6ToTriangles` turns an `edges3` into two `edges`;
        `graphics.Lines` gets `shape`.
      - the visual check: `python/Examples/graphicsCurvedShapes.py` (curved shapes beside the flat ones of `exudyn.graphics`,
        quadratic lines, spheres, a rotating body), row K13 of the manual GUI check (RG2.4);
      - **RG6.7.7.9** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg6-7-7-9) - the maintainer's look at
        `graphicsCurvedShapes.py`: element edges that could not be switched off, seams in the shading; `formatVersion`
        stays 1;
      - **RG6.7.7.8** **DONE 2026-10-02** (#2769) - `MergeTriangleLists` did not offset the `edges` of `g2` when `g1`
        had none; now as for `edges3`, test `testMergeOffsetsTheEdgesOfTheSecondList`.
      - **RG6.7.7.6** **the primitives on quadratic shapes** - `Cylinder`, `Tube`, `Torus`, `SolidOfRevolution`, the
        partial `Sphere`, `Arrow`, ... built from 6-node triangles with `edges3` on their rims. **In part DONE
        2026-10-02** — [log](exudynRevisionLog2026b.md#rg6-7-7-6): `Cylinder` (full, partial, hollow), `SolidOfRevolution`
        and with it `Arrow`, `Basis`, `Frame`, `RigidLink`, `BallBearingRings`, and `Torus`; the split with at least 2
        subdivisions for a curved element and the edges of a 6-node triangle in its tiling; then `Tube` and the `Sphere`
        with edges or between two latitudes. **The rest DONE 2026-10-03** — [log](exudynRevisionLog2026b.md#rg6-7-done):
        `LinkedCylinders` (arcs, tangents and bores) and the hollow `Sphere`; `SpheresToTriangleList` stays flat on
        purpose - it makes contact meshes - and so does `SolidExtrusion` of a polygon. *Compatibility of
        `nTiles`* (maintainer's question): `nTiles` keeps its meaning - **the number of flat segments around** - and
        the primitive uses $\lceil$`nTiles`/2$\rceil$ quadratic elements, each covering two of today's segments.
        For a script to never look coarser than today, the split of a *curved* 6-node triangle or 3-node line has
        **at least 2** subdivisions (one stays for a flat one, where the mid nodes add nothing); it gets more only
        where `curvedTriangleTilingAngle` asks for them. So a default cylinder (`nTiles=16`: 8 elements of 45°) shows
        24 segments at 15°, and a script with `nTiles=64` shows at least its 64. The data (points, triangles) shrink
        to about half, the drawn triangles never fall below today's.
      - **RG6.7.7.7** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg6-7-7-7) - the examples and test models
        with very large `nTiles` (chosen to hide the facets) are revised, most to about half the value, once RG6.7.7.6
        is in - checked by image, not by rule.
      - **RG6.7.7.11** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg6-7-7-11) - *(maintainer 2026-10-02)* **the
        frame of the rigid markers**: `visualizationSettings.markers.showBasis` and `basisSize` (the names of the
        nodes); simplified three RGB lines, else three arrows as the node basis with heads half as long (#2791) - the
        rigid markers get their own rotation with `localHT` (RG16.3.3).
      - **RG6.7.7.10** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg6-7-7-8) (written up there as RG6.7.7.8,
        a number taken by #2769) - *(found in RG6.7.7.7; needs no
        decision)* **single bright pixels of the raytracer on curved
        triangles at coarse tiling** (#2787): at `nTiles` 32 and below a few pixels of a `SolidOfRevolution` and a half
        sphere are white - rays between neighbouring curved triangles or their flat split; the split along a shared
        edge is to be checked for watertightness.

    <a id="rg6-7-sketch"></a>
    **The interface, sketched 2026-10-01** (for the maintainer; nothing implemented). What exists, read in the code:
    `GraphicsData` (C++, what is drawn) already has `glSpheres` - `GLSphere` = item, point, color, radius,
    resolution - and the nodes already go into it (`DrawNode` → `AddSphere`); OpenGL draws them, and
    `GetGraphicsData()` returns them as `spheres`. Missing are two links: the **body graphics** (`BodyGraphicsData`,
    the converted `VgraphicsData` of a body: lines, circles, texts, triangles - no spheres) and the **raytracer**,
    which intersects triangles only. The dictionary types today: `Line`, `Lines`, `Circle`, `Text`, `TriangleList`.

    *Python - the dictionaries.* One new type and one new key:

    ```python
    #spheres: arrays like 'Lines'; one sphere is n = 1
    {'type': 'Spheres',
     'points': [x0,y0,z0, x1,y1,z1, ...],      #centers, 3n floats
     'radii':  [r0, r1, ...] or r,             #n values, or one for all
     'colors': [R0,G0,B0,A0, ...] or [R,G,B,A],#n colors, or one for all
     'resolution': 8}                          #nTiles of today's graphics.Sphere (OpenGL; the raytracer is exact)

    #curved triangles: a key of 'TriangleList', so that one list can carry flat and curved triangles
    {'type': 'TriangleList', 'points': [...], 'colors': [...], 'normals': [...],   #normals optional, per point
     'triangles':   [i0,i1,i2, ...],                      #flat, as today
     'triangles6':  [c0,c1,c2, m01,m12,m20, ...],         #NEW: 6-node quadratic triangles
     'edges': [...]}                                       #as today
    #node order of a 6-node triangle: corners c0,c1,c2 counter-clockwise seen from outside, then the mid-side
    #nodes m01 (between c0 and c1), m12, m20 - the order of NETGEN/NGsolve second-order surface elements
    ```

    *Python - the helpers* (`exudyn.graphics`):

    | function | change |
    |---|---|
    | `Sphere(point, radius, color, nTiles, ...)` | returns `Spheres` with one point for a full sphere; with `addEdges`, a partial sphere (`majorAngleMin/Max`) or `innerRadius` it returns a `TriangleList` as today - those cannot be a `GLSphere` |
    | `Spheres(points, radii, colors, nTiles)` (new) | many spheres in one dictionary - particles, point clouds - one item instead of n |
    | `NGsolveMesh2PointsAndTrigs(..., meshOrder=2)` | returns `triangles6` for a second-order mesh instead of four flat triangles per element; the FEM surface of quadratic meshes the same |
    | `Move`, `Transform`, `MergeTriangleLists`, `BoundingBoxSingle`, `ToPointsAndTrigs`, STL export | learn `Spheres` (transform centers, scale radii) and `triangles6`; those that need flat triangles (STL, `ToPointsAndTrigs`) split with the same rule as the renderer |

    *C++ - where the new data goes.*

    ```cpp
    class BodyGraphicsData {            //the converted VgraphicsData of a body
        ResizableArray<GLLine> glLines; ResizableArray<GLCircleXY> glCirclesXY; ResizableArray<GLText> glTexts;
        ResizableArray<GLTriangle> glTriangles;
        ResizableArray<GLSphere> glSpheres;               //NEW: 'Spheres'
        ResizableArray<GLTriangle6> glTriangles6;         //NEW: 'triangles6', kept for the read-back and the split
    };
    class GLTriangle6 { Index itemID; std::array<Float3,6> points, normals; std::array<Float4,6> colors; bool hasNormals; };
    ```

    `GraphicsData` (what is drawn) gets **no** curved triangles: they are split into `GLTriangle`s when the body
    graphics are transformed into it, so the OpenGL renderer, the raytracer, `GetGraphicsData()` and the graphics
    fingerprints see flat triangles only, as decided. The split of a rigid body's graphics is the same in every frame:
    it is done once at the conversion and cached in `BodyGraphicsData`, redone when `curvedTriangleTilingAngle` or
    `curvedTriangleMaxTiling` changes; a superelement's `triangleMesh` with six columns deforms, so it is split in
    every frame. Spheres are transformed (center; radius unchanged, scaled only by an explicit scale) into
    `GraphicsData.glSpheres`, the list the nodes already use, in the same drawing order.

    | consumer | `Spheres` | `triangles6` |
    |---|---|---|
    | `PyWriteBodyGraphicsDataList` (dictionary → C++) | new branch | new key in `TriangleList` |
    | `PyGetBodyGraphicsDataList` (C++ → dictionary, `mbs.GetObject`) | new branch | the 6-node form, as given |
    | body graphics → `GraphicsData` (`UpdateGraphics` of the bodies) | transform into `glSpheres` | split (cached) into `glTriangles` |
    | OpenGL | nothing new | nothing new |
    | raytracer | **new**: ray-sphere intersection with the exact normal; the spheres in the search tree by their bounding boxes, after the triangles; shadows the same | nothing new |
    | `GetGraphicsData()` | nothing new (`spheres` exists) | the split triangles |
    | graphics tests (RG2.3.3) | spheres counted per item; references re-recorded where `graphics.Sphere` was a TriangleList | more triangles; references re-recorded |
    | settings | - | `openGL.advanced.curvedTriangleTilingAngle` (3°), `curvedTriangleMaxTiling` (5) |

    **Decided (maintainer, 2026-10-01)**: (a) one `Spheres` type with arrays; (b) `graphics.Sphere` returns it by
    default, but only for a whole sphere - not with edges, as a part of a sphere or hollow; scripts that treat the
    result as a `TriangleList` go through the helpers; (c) `triangles6` as a key of `TriangleList`; (d) **not** the
    cached split: `GraphicsData` (C++) gets its own structure for the 6-node triangle, which the `triangles6` of a
    `TriangleList` map to (the points duplicated per triangle, as for `GLTriangle` - the meshes are not kept), and
    `GLSphere`, which exists, is the internal structure of the sphere. The split into flat triangles happens where
    the 6-node triangles are drawn.

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
    - **RG6.8.4** (#2308) erratic shadows with `modelCentricView=False` and lights in the camera frame -
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
**RG9.1** **DONE 2026-09-23** (#2622) — [log](exudynRevisionLog2026b.md#rg9-1) · [plan text](exudynRevisionLog2026b.md#plan-rg9-1) — The item sources stop paying for pybind11.

<a id="rg9-2"></a>
**RG9.2** **DONE 2026-09-23** (#2628, #2629) — [log](exudynRevisionLog2026b.md#rg9-2) · [plan text](exudynRevisionLog2026b.md#plan-rg9-2) — Fourteen item sources included an exception header they do not use, and paid pybind11 for it.

<a id="rg9-3"></a>
**RG9.3** **DONE 2026-10-03** (#2744 resolved) *(group RG9; maintainer 2026-09-29)* **Access functions as single functions of the objects**
    (#2744). `GetAccessFunctionBody(AccessFunctionType, localPosition, Matrix& value)` serves every access
    type through one function and a switch, and carries workarounds - the vector of
    `JacobianTtimesVector_q` travels in the output matrix, `OwnMarkersOnly` (RG4.10) says what a
    declaration cannot. Single functions per access type, with interfaces that say what they take and
    return, avoid them.
    - **RG9.3.1** **DONE 2026-09-29, decided 2026-10-02** — the evaluation, in
      `tmp/evalRG9_3_accessFunctions.md` (not kept in the repository): which objects provide which
      access functions today, which markers and loads call them, and what the best interface is for each.
      Proposed: one virtual function per access type, and the flags derived from the functions a
      definition declares (with RG9.3.2); **decided (maintainer, 2026-10-02): as recommended** - option A (single
      virtual functions) and C (the flags from the definition), RG9.3.4 first; the question whether the Jacobians stay
      hand-written is evaluated after the split (RG9.3.5);
    - **RG9.3.2** **DONE 2026-10-02** with RG9.3.4.4 — a check that the access function flags an object declares
      (`ItemAccessFunctionTypes`) and the functions its definition declares agree - possibly by deriving the flags
      from the functions;
    - **RG9.3.3** the migration, object by object - realized in RG9.3.4.
    - **RG9.3.4** *(maintainer 2026-10-02)* **the split of `GetAccessFunctionBody`** — .1-.3 **DONE 2026-10-02**, .4 **DONE 2026-10-03** (first in
      part 2026-10-02) — [log](exudynRevisionLog2026b.md#rg9-3-4):
      - **RG9.3.4.1** the class of access functions: in `CObjectBody`, one virtual function per access type with an
        interface that says what it takes and returns - `GetPositionJacobian(localPosition, jacobian)` (3 x n),
        `GetRotationJacobian(localPosition, jacobian)`, `GetMassWeightedPositionJacobian(jacobian)`,
        `GetJacobianTransposedTimesVectorDerivative(localPosition, forceTorque, result)` (the vector as an argument, a
        return value for "zero"), `IsValidLocalPosition(localPosition, reason)` (the restrictions now found at run time
        - "on the axis", "at the center of mass" - checked at `Assemble()`); base implementations that raise
        *"<object> provides no <access>"*; documented in one place; `GetAccessFunctionBody` calls them meanwhile, so
        the callers do not change yet;
      - **RG9.3.4.2** the objects, one by one (17 objects and `MarkerBodyCable2DShape`): the switch split into the
        functions, compared with the old switch on the test models; the commented-out and dead cases removed
        (`ObjectRotationalMass1D`, `ObjectANCFCable`, `ObjectANCFBeam`); `ObjectBeamGeometricallyExact` provides what it
        declares or declares nothing (with RG4.8);
      - **RG9.3.4.3** the callers (the markers, the loads, `GeneralContact`) call the single functions;
        `GetAccessFunctionBody` and the input-through-output convention of `JacobianTtimesVector_q` go;
      - **RG9.3.4.4** = RG9.3.2: the flags derived from the functions a definition declares; `OwnMarkersOnly` becomes
        "declares none"; `SuperElementAlternativeRotationMode` moves to the marker; **done 2026-10-02: the check**
        (rule 7 of the definition validator: a body declares a type exactly if it provides its function). **The rest DONE
        2026-10-03** — [log](exudynRevisionLog2026b.md#rg9-3-4-4): the flags derived instead of declared, the own
        markers declared apart (`ownMarkers=`, `ownMarkerTypes=`), `OwnMarkersOnly` derived, and
        `SuperElementAlternativeRotationMode` an argument of `GetAccessFunctionSuperElement`;
    - **RG9.3.5** **DONE 2026-10-02** (evaluated with a switch; (a)-(c) decided by the maintainer and done, the switch
      removed) — [log](exudynRevisionLog2026b.md#rg9-3-5), [decision](exudynRevisionLog2026b.md#rg9-3-5-decided) - *(maintainer 2026-10-02; after RG9.3.4; "with a switch, so performance can
      be compared")* **evaluation: hand-written Jacobians or AD of a templated `GetPosition`**. To answer: what changes - a template cannot be virtual, so the object would provide a templated
      position function plus a virtual wrapper per number type (Real, the AD types of RG14), or the markers call
      object-specific templates; the impact on the implementation of each object (17), on the markers and on the
      definitions; what would be gained in performance (the Jacobian by AD costs a pass with n directions against a
      hand-written matrix today) and in code (the hand-written Jacobians and their derivatives
      `JacobianTtimesVector_q` disappear); and what RG15 (objects computing from given coordinates) changes about it.
      The result is a proposal, not a migration. **Done**: `exu.experimental.accessFunctionsByAD` (1: AD, 2: the general
      path with the hand-written functions) for `ObjectRigidBody` (Euler parameters, Tait-Bryan angles),
      `ObjectRigidBody2D` and `ObjectANCFCable2D`; the same results (Euler parameters: to the Newton tolerance); AD costs
      +40 % to +80 % solver time on chains of rigid bodies joined by connectors, +2 % on an ANCF cable; found #2774.
      **Proposed**: (a) the hand-written Jacobians stay for the bodies that are hot in connector-heavy models (rigid
      bodies, mass points) - their fast projections without forming a Jacobian matter more than AD; (b) AD provides
      what is missing or approximated today - the derivative of `J^T f` of the ANCF cables and beams (none, or taken
      as zero), and the access functions of new objects, from one templated position (with RG15); (c) a cheaper AD
      seeds only the coordinates the position is nonlinear in (the rotation parameters: 4 directions instead of 7);
      (d) the switch goes when (a)-(c) are decided.
    - **RG9.3.6** **DONE 2026-10-02** (1)-(3), (4) moved to RG9.3.7 — [log](exudynRevisionLog2026b.md#rg9-3-6) -
      *(maintainer 2026-10-02)* **the leftovers found in the evaluation and the split** (#2773): (1) the
      check of a marker on a body without rotation access reports the marker index as the object number (and spells
      *orienation*); (2) dead code - the `if (false)` branch of `CObjectANCFCable2DBase::GetPositionJacobian` (the exact
      derivative of the normal, equal to the version in use; `ObjectALEANCFCable2D` uses the exact one), the
      commented-out calls of `GetAccessFunctionBody` (relative coordinate markers, `CObjectRigidBody`,
      `VisualizationObject.h`), the incomplete commented-out block of `CObjectFFRFreducedOrder::GetMassWeightedPositionJacobian`,
      the double `SetAll` in `CObjectANCFBeam::GetMassWeightedPositionJacobian`; (3)
      `CMarkerKinematicTreeRigid::ComputeMarkerDataJacobianDerivative` raises unconditionally, the code after it is
      unreachable; (4) ideas: `ObjectRotationalMass1D` could provide the position Jacobian off its axis as
      `ObjectRigidBody2D` does, `ObjectANCFBeam` a rotation Jacobian from its slopes. The three findings of section 7
      of the evaluation (the beam that declared four types and provided none, the undeclared case of `ObjectANCFBeam`,
      the commented-out cases) are done in RG9.3.4.2.
    - **RG9.3.7** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg9-3-7) - *(maintainer 2026-10-02: the
      leftovers that need a decision go into a step of their own; done as proposed)* **two access
      functions a body could provide** (#2775): the position Jacobian of `ObjectRotationalMass1D` off its axis (it would
      depend on the angle, as for `ObjectRigidBody2D`; `Assemble()` refuses connector and load markers there today), and
      a rotation Jacobian of `ObjectANCFBeam` from its slopes (which slopes, which orthogonalization). For the
      maintainer's decision.
    - **RG9.3.8** **DONE 2026-10-02** (15 bodies; the rest in RG9.5.6) — [log](exudynRevisionLog2026b.md#rg9-5) -
      *(found in RG9.3.7 and RG4.17.1; needs no decision)* **every body's access functions against finite
      differences** (#2777): one pytest that builds each body type at deformed, rotated coordinates and checks, at
      several local positions, `GetPositionJacobian` and `GetRotationJacobian` against finite differences of the
      position and the rotation matrix, and `GetJacobianTransposedTimesVectorDerivative` against the numerical
      derivative of $\Jm^T\fv$. Both bugs of 2026-10-02 - the position Jacobian of `ObjectANCFBeam` sized for 8 of its
      18 coordinates, the rotation Jacobian of the slope nodes not the derivative of their rotation - were found by
      chance; this test would have found them. **After RG9.5** (maintainer 2026-10-02): the access functions are not
      reachable from Python today; with `mbs.ComputeItem` and its numerical-derivative helper this test is short.

<a id="rg9-4"></a>
**RG9.4** **DONE 2026-10-03** (#2202 resolved; RG9.4.3.2 is in the list
    [*not decided to be resolved*](#not-decided)) *(group RG9; maintainer 2026-09-30)* **Kinetic and potential energy as output variables**
    (#2202). `OutputVariableType.KineticEnergy` and `PotentialEnergy` exist (bits 32 and 33) and no item
    provides them. They are added where they make sense - rigid bodies, flexible bodies, superelements
    and connectors - and nowhere else; a test then checks the conservation of energy of a free
    oscillation, as the literature does for the beam benchmarks (RG4.8.11). One step per object type:
    - **RG9.4.1** **DECIDED 2026-09-30** (maintainer) - the convention: `PotentialEnergy` of an object is
      its **elastic** energy, zero in the reference configuration; only objects for which it is meaningful
      report `KineticEnergy` or `PotentialEnergy`. An object with a **user function** reports no energy; an
      object that cannot report it raises an exception **with the reason**. The energies are added only where
      the computation is straightforward and duplicates no larger code - for a beam from the existing
      `ComputeODE2LHS` functions, or a simple loop over the integration points (bending and axial strain
      energy; inefficient but simple is acceptable). The energy is one number for the item: **`localPosition`
      must be `[0,0,0]`**, so that nobody takes it for a quantity at a point. The inspection of RG12.29 lists
      the energies among the output variables only where they can be computed. **Loads are wanted as well**,
      at least constant and mass-proportional ones (the potential of the load through its marker's position)
      - planned in RG9.4.6 and RG9.4.7;
    - **RG9.4.2** **DONE 2026-10-01** (#2202, #2766) — [log](exudynRevisionLog2026b.md#rg9-4-2) - the simple objects first: `ObjectMassPoint`, `ObjectMassPoint2D`, `ObjectMass1D`,
      `ObjectRotationalMass1D`, `ObjectRigidBody`, `ObjectRigidBody2D` (kinetic), the linear spring-dampers
      (coordinate, Cartesian, torsional, linear; potential), and a **test model for energies** that shows
      the effect on several simple, independent mechanisms (a free oscillator, a pendulum on a spring, a
      rotating body), each with its conserved or dissipated total;
    - **RG9.4.3** the heavier objects: `ObjectConnectorRigidBodySpringDamper`, the ANCF cables and beams,
      `ObjectBeamGeometricallyExact(2D)`, the ALE cable - kinetic and elastic energy, by the rule above.
      **Kinetic energy DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg9-4-3) - for all of them from
      their mass matrix, and the potential energy of the rigid-body spring-damper.
      - **RG9.4.3.1** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg9-4-3-1) - the elastic energy of
        `ANCFCable2D`, `ALEANCFCable2D`, `ANCFCable`, `ANCFBeam`, `BeamGeometricallyExact(2D)` and `ANCFThinPlate`:
        the rule selection of each element moved into one function that the forces and the energy share, the
        strains are the ones of the forces; $\partial V/\partial\qv$ equals the elastic forces to $10^{-10}$; and the
        potential of `ObjectConnectorGravity`;
      - **RG9.4.3.2** **the objects without energy** - *not now* (maintainer, 2026-10-01: the special and contact
        objects need no energy right away, with friction it may be difficult); the list is kept here. What an
        object provides is answered by `mbs.Inspect(item, exu.InspectType.OutputVariables)` (RG12.29), which is
        where this list comes from (all MiniExamples, 2026-10-01); an energy added to an object removes it here:

        | objects | energy | why, or what would have to be done |
        |---|---|---|
        | `ContactCoordinate`, `ContactSphereSphere`, `ContactSphereTorus`, `ContactConvexRoll`, `ContactCurveCircles`, `ContactCircleCable2D`, `ContactFrictionCircleCable2D`, `ConnectorRollingDiscPenalty` | none yet | the integral of the normal penalty force over the penetration ($\frac{1}{2}k g^2$ linear, $\frac{2}{5}k g^{5/2}$ Hertz), zero without contact; friction and damping dissipate; one function per contact law, each with its own data states |
        | `ConnectorCoordinateSpringDamperExt` | none yet | the spring $\frac{1}{2}k(u-u_\mathrm{off})^2$ plus the bristle of the stick-slip friction $\frac{1}{2}k_b x_b^2$ in its data coordinate |
        | `ConnectorReevingSystemSprings` | none yet | the axial springs of the rope segments, $\sum\frac{1}{2}\frac{EA}{L}\Delta L^2$ with its own length bookkeeping |
        | `ConnectorHydraulicActuatorSimple` | none | the energy of the compressed oil is not elastic energy of the structure; first-order pressure states |
        | all constraints and joints (`ConnectorCoordinate`, `ConnectorCoordinateVector`, `ConnectorDistance`, `Joint...`) | none | ideal constraints do no work |
        | `ObjectGround`, `ObjectGenericODE1` | none | no motion; first-order coordinates |
        | the spring-dampers, `GenericODE2`, `FFRF`, `FFRFreducedOrder`, `KinematicTree`, `ANCFCable2D` **with a force user function** | none while the user function is set | the user function defines the force; `PotentialEnergyAvailable()` of the object, and `mbs.Inspect` does not list it |

    - **RG9.4.4** superelements: `ObjectFFRF`, `ObjectFFRFreducedOrder`, `ObjectGenericODE2`,
      `ObjectKinematicTree` - kinetic energy from the mass matrix, elastic energy from the stiffness matrix
      where the object has one. **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg9-4-3) - and
      **RG9.4.4.1** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg9-4-3-1) - the potential energy of
      `ObjectKinematicTree`: the springs of the P control, the constant joint forces and the built-in gravity;
    - **RG9.4.5** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg9-4-5) - what an object **should** provide
      against what it provides now. **Decided (maintainer, 2026-10-02)**: a rigid body reports a zero potential energy -
      it may get a built-in gravity later, and then its behavior does not change. So every body provides both,
      a connector the potential energy only;
    - **RG9.4.6** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg9-4-3) - `LoadPotentialEnergy` and
      `CreateLoadEnergySensor` in `exudyn.advancedUtilities`. The energy of a load, for constant and mass-proportional loads: the potential of the force
      through the position of its marker, computed by a user sensor (`LoadEnergyUserSensor`) - a load has no
      output variable today, only a sensor that reads its value;
    - **RG9.4.7** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg9-4-3) - `SystemEnergy` in
      `exudyn.advancedUtilities`. A utility class `SystemEnergy` (a user sensor): its `__init__` collects the objects that
      provide kinetic and potential energy - with a flag to skip those that should and do not, with the
      current parameters - and the loads, and `ComputeSystemEnergies` returns the totals, as long as there
      is no MainSystem function for the energy of the system.
    - **RG9.4.8** **DONE 2026-10-01** (#2767) — [log](exudynRevisionLog2026b.md#rg9-4-3-1) - the energy tests as
      **test models**, where users look for examples (maintainer, 2026-10-01): `test_energies.py` goes into
      `energiesTest.py`, a new `energiesFlexibleBodiesTest.py` keeps the energy of a free oscillation of each beam
      and plate element and of a kinematic tree (the conservation test this step asked for), and
      `test_connectorOutputVariables.py` became `connectorOutputVariablesTest.py`; the rule is in `CLAUDE.md` and
      `docs/dev/WORKFLOW.md` §5. **RG9.4.8.1** **CLOSED 2026-10-01** (maintainer): `test_specialBeams.py` and
      `test_computedParameters.py` stay in `python/testing/`.

<a id="rg9-5"></a>
**RG9.5** *(group RG9; maintainer 2026-10-02)* **`mbs.ComputeItem`: the computation functions of an item from Python**
    (#2779). Python reaches an item's computation only through its output variables (position, velocity, ...). The
    access functions of a body (`GetPositionJacobian`, `GetRotationJacobian`, `GetJacobianTransposedTimesVectorDerivative`),
    its `ComputeODE2LHS` and mass matrix, the equations and Jacobians of a constraint, the forces of a connector, the
    Jacobians of a node or marker are not reachable - which a test of them (RG9.3.8), the debugging of an item and a
    user who implements one all need. Proposed by the maintainer: an interface like `mbs.Inspect`,
    `mbs.ComputeItem(itemIndex, what, [optional parameters])`, computing **at the current state** (anything else is far
    more complicated); as many functions as possible defined once in a base class (`CObjectBody`, `CObjectConnector`,
    `CNodeODE2`, `CMarker`), each kind adding only its Python interface; and a small Python helper for the numerical
    derivative (of a position, a rotation matrix, a residual) that makes the comparison of a computed Jacobian a line.
    - **RG9.5.1** **DECIDED 2026-10-02** (maintainer: the name `mbs.ComputeItem`/`exu.ComputeItemType`, renamed from `ItemCompute` (#2783); the helper in `exudyn.advancedUtilities`; `what=None` the list of what applies to the item; one `vector` for force and torque) — [log](exudynRevisionLog2026b.md#rg9-5) -
      the evaluation and the proposal, for the maintainer's decisions: which functions per kind of item, the
      names of `what`, the arguments (local position, force/torque, factors), what is returned (numpy arrays, dicts), how
      the current state is set (`mbs.systemData`), what a call on an item that does not provide a function raises, and
      the place of the numerical-derivative helper;
    - **RG9.5.2** to **RG9.5.5** **DONE 2026-10-02** (#2779) - the access functions of the bodies (position and rotation Jacobians, the derivative of $\Jm^T\fv$, the
      mass-weighted position Jacobian, `IsValidLocalPosition`) and the helper; then RG9.3.8;
    - **RG9.5.3** the computation functions of the objects: `ComputeODE2LHS`, the mass matrix, the ODE2 Jacobians;
    - **RG9.5.4** connectors and constraints: the force on the interface, the algebraic equations, C_q, the reaction
      forces;
    - **RG9.5.5** nodes and markers: the node's position and rotation Jacobians and `GetRotationJacobianTTimesVector_q`,
      the marker's kinematics (L0) and its Jacobians.
    - **RG9.5.6** **DONE 2026-10-02** (the ODE2 Jacobian and all bodies; `IsValidLocalPosition` and the node Jacobians of
      algebraic equations left out, see the log) — [log](exudynRevisionLog2026b.md#rg9-5-6) - *(found in RG9.5; needs no
      decision)* **what `mbs.ComputeItem` does not compute yet** (#2782): the ODE2
      Jacobian of an object or connector (analytic where it has one, else numerical as the solver does),
      `IsValidLocalPosition`, the Jacobians of a node with algebraic equations; and the bodies the test of RG9.3.8 does
      not build yet - `ObjectFFRF`, `ObjectFFRFreducedOrder`, `ObjectKinematicTree`, `ObjectALEANCFCable2D`,
      `ObjectGenericODE2` -, and the derivative of the Lie group node by composed increments.
    - **RG9.5.7** *(found in RG9.5.6; maintainer 2026-10-02: the marker stays fixed to the beam, not co-moving with the
      axial displacement - co-moving makes sense only along a list of beams, as the sliding joints do; observed: the
      Jacobian agrees, the velocity of the marker does not - [log](exudynRevisionLog2026b.md#rg9-5-7); **DECIDED
      2026-10-03 (maintainer): left as it is** - the marker velocity keeps the Eulerian term; #2784 closed)*
      **`ObjectALEANCFCable2D`: what a marker on it is**
      (#2784): its position Jacobian has a zero column for the ALE coordinate (since #2786, before 8 columns, which failed
      every connector and load through a body marker after RG14.2.13), while its velocity
      output - the material velocity - depends on the ALE velocity; off the axis the Jacobian also differs from the
      derivative of the velocity output by 2 %. Either a marker is a material point (the Jacobian gets the ALE column, and
      forces act on the ALE coordinate), or a point fixed along the axis (its velocity is J q_t, without the ALE term);
      and which derivative of the normal is right off the axis.

## RG10 — Tooling and process

The machinery a maintainer uses: `exudev` (revision2026 step R5.18), the issue tracker and its
JSON store (revision2026 steps R8.3 to R8.5), the generators (revision2026 step R4.3), the checks of the commit gate, and the CI. It
works; this group carries what it still lacks.

Open in the tracker for this group: **#2541** (`exudyn.config` and `exudyn.special` are in no stub
file, so an editor cannot complete them).

<a id="rg10-1"></a>
**RG10.1** *(group RG10; maintainer request 2026-09-15; revision2026 step R8.6)* **DONE 2026-09-27**
    (#2712) — [log](exudynRevisionLog2026b.md#rg10-1) — `exudev scripts <folder>`, a maintainer tool
    for now, as the maintainer decided for teaching; **it stays a maintainer tool** (maintainer, 2026-09-30).
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
**RG10.14** *(group RG10; maintainer 2026-09-30)* **DONE 2026-09-30** (#2760) — [log](exudynRevisionLog2026b.md#rg10-14) —
    **`exudev` runs the pytest files and the MiniExample performance run**: `exudev pytest [--graphics]
    [--gate] [--record] [-k] [--processes]` and `exudev perf --mini [--full] [--processes] [--only] [--compare]`;
    `build --complete` runs the pytest files last.

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
**RG12.1** **DONE 2026-10-03** — [log](exudynRevisionLog2026b.md#rg12-1) *(group RG12; maintainer
    2026-09-22)* **`simulationSettings` gets the deprecation mechanism** (#2588). `visualizationSettings` has it: a member is marked `Deprecated(since,
    expires)` in the definitions - 93 members carry it today - and a user who sets the old name
    is told the new one instead of being ignored. `simulationSettings` uses none of it, although
    it is the same generator and the same structure machinery, so a renamed solver setting
    simply disappears.
    - **RG12.1.1** **DONE 2026-10-03** (#2800) — [log](exudynRevisionLog2026b.md#rg12-1-1) - *(maintainer
      2026-10-03)* `multithreadedLowerLimit...` instead of `multithreadedLimit...`, which said less; the old
      `multithreadedLLimit...` forward until 2031 (five years), not 2028.

<a id="rg12-2"></a>
**RG12.2** **DONE 2026-10-03** — [log](exudynRevisionLog2026b.md#rg12-2) *(group RG12; maintainer 2026-09-22)*
    **Item parameters can be deprecated** (#2589).
    The case that actually hurts: an item parameter is renamed and every script that used the
    old name stops working, with no message that says what to write instead. Two levels are
    possible - the generated classes of `itemInterface.py`, which is one place and covers what a
    script writes, or the `Get`/`Set` functions of the items themselves, which also covers
    `mbs.GetObjectParameter`. If it reaches the C++ side, **the deprecated names are searched
    last**, so that the common case pays nothing.

<a id="rg12-3"></a>
**RG12.3** **DONE 2026-09-24** (#2590) — [log](exudynRevisionLog2026b.md#rg12-3) · [plan text](exudynRevisionLog2026b.md#plan-rg12-3) — What did this model actually change?

<a id="rg12-4"></a>
**RG12.4** *(group RG12; maintainer 2026-09-25)* **DONE 2026-09-26, RG12.4.7 2026-10-03** (#2664, resolved; #2796, #2797) —
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
    - **RG12.4.7** **DONE 2026-10-03** — [log](exudynRevisionLog2026b.md#rg12-4-7) (#2796, #2797) *(maintainer,
      2026-09-25)* — **the `TPyFunction...` group type disappears from a definition**. *"Can the types like `TPyFunctionMbsScalarIndexScalar5` then also be eliminated?
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
          group, which is what the maintainer asked for. **Identical signatures collapse into one member**
          (maintainer, 2026-09-30).

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

<a id="rg3-23"></a>
**RG3.23** **DONE 2026-09-26** (#2680) — [log](exudynRevisionLog2026b.md#rg3-23) · [plan text](exudynRevisionLog2026b.md#plan-rg3-23) — The override settings are documented where the module is.

<a id="rg3-24"></a>
**RG3.24** **DONE 2026-09-27** (#2681) — [log](exudynRevisionLog2026b.md#rg3-24) · [plan text](exudynRevisionLog2026b.md#plan-rg3-24) — The generator API still says "Latex".

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
**RG12.14** **DONE 2026-09-26** (#2687) — [log](exudynRevisionLog2026b.md#rg12-14) · [plan text](exudynRevisionLog2026b.md#plan-rg12-14) — The override settings can be read again.

<a id="rg12-15"></a>
**RG12.15** **DONE 2026-09-27** (#2688) — [log](exudynRevisionLog2026b.md#rg12-15) · [plan text](exudynRevisionLog2026b.md#plan-rg12-15) — A script can place a dialog, and it is written down.

<a id="rg12-16"></a>
**RG12.16** **DONE 2026-09-26** (#2689) — [log](exudynRevisionLog2026b.md#rg12-16) · [plan text](exudynRevisionLog2026b.md#plan-rg12-16) — The render window and the SolutionViewer remember their size and position.

<a id="rg12-19"></a>
**RG12.19** **DONE 2026-09-27** (#2693) — [log](exudynRevisionLog2026b.md#rg12-19) —
    **Two buttons: one for the settings, one for the positions**, each showing what it will write
    before it writes it. The **render window** geometry rides along in the settings button, by the
    maintainer's decision: *"I opt to store it in the config file in the visualizationSettings, because
    it is the straightforward way and becomes now natural, because it is only stored if it differs from
    default."*

<a id="rg12-20"></a>
**RG12.20** **DONE 2026-09-27** (#2694) — [log](exudynRevisionLog2026b.md#rg12-20) · [plan text](exudynRevisionLog2026b.md#plan-rg12-20) — Where the render window is, and what happens when the file and the session disagree.

<a id="rg3-25"></a>
**RG3.25** **DONE 2026-09-27** (#2683) — [log](exudynRevisionLog2026b.md#rg3-25) · [plan text](exudynRevisionLog2026b.md#plan-rg3-25) — a TAB instead of a backslash put `exttt{...}` on three pages of the Symbolic manual.

<a id="rg3-26"></a>
**RG3.26** **DONE 2026-09-27** (#2697) — [log](exudynRevisionLog2026b.md#rg3-26) · [plan text](exudynRevisionLog2026b.md#plan-rg3-26) — `index.md` and `pdfIndex.md` are two hand-written tables of contents that must agree - the maintainer chose option B, the check.

<a id="rg3-27"></a>
**RG3.27** **DONE 2026-09-27** (#2708) — [log](exudynRevisionLog2026b.md#rg3-27) · [plan text](exudynRevisionLog2026b.md#plan-rg3-27) — The mass-spring-damper tutorial comes first.

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
    - **RG12.29.2** *open* - the node types a **node marker** requests: today checked in C++ as alternatives
      (`Position` or `Position2D`, ...), declared as `requestedNodeTypes` in the definitions for the documentation
      only (RG13.5.0.3); `Inspect` does not answer them yet - a list of alternatives per node would need its own
      form, or the C++ check generated from the declaration first.

<a id="rg12-31"></a>
**RG12.31** *(group RG12; maintainer 2026-10-03: "list them - I decide")* **Settings and item parameters that
could be renamed or restructured** (#2802). The candidates found by a pass over all members of `SimulationSettings`
and all parameters of the item definitions; each one is a decision of the maintainer, and each decided one becomes a
sub-step with its own issue, done with the deprecation of RG12.1/RG12.2 (old name forwarding, removal five years
after). *Status: a list, nothing decided.* Settings:
    - **RG12.31.1** two `outputPrecision`: the top one is for the console, `solutionSettings.outputPrecision` for the
      files - `consoleOutputPrecision`, or the top one into a small structure with the other console members
      (`displayComputationTime`, `displayStatistics`, `displayGlobalTimers`);
    - **RG12.31.2** `linearSolverType` beside `linearSolverSettings` - into it, as `linearSolverSettings.solverType`;
    - **RG12.31.3** `timeIntegration.simulateInRealtime`, `realtimeFactor`, `realtimeWaitMicroseconds` - a structure
      `timeIntegration.realtime` (`active`, `factor`, `waitMicroseconds`);
    - **RG12.31.4** `numericalDifferentiation.forODE2connectors` - `forODE2Connectors`, and
      `staticSolver.constrainODE1coordinates` - `constrainODE1Coordinates` (the capital of every other name);
    - **RG12.31.5** inside `newton`: `newtonResidualMode` - `residualMode`; `useNewtonSolver` (false = linear) -
      `useNewton` or `linear` with the opposite meaning;
    - **RG12.31.6** `solutionSettings`: three files with three patterns (`coordinatesSolutionFileName`,
      `solverInformationFileName`, `restartFileName`; `solutionWritePeriod`, `sensorsWritePeriod`,
      `restartWritePeriod`; `writeFileHeader`, `sensorsWriteFileHeader`), and `flushFilesDOF` (a number of
      coordinates) - substructures `solutionFile`, `sensorFiles`, `restartFile` with the same members; the largest
      one, and the one most scripts use;
    - **RG12.31.7** `explicitIntegration.dynamicSolverType` selects the explicit solver only - `explicitSolverType`.

  Item parameters:
    - **RG12.31.8** `ObjectJointGeneric.axesRadius/axesLength` against `axisRadius/axisLength` of
      `JointRevoluteZ`/`JointPrismaticX` (visualization) - one spelling;
    - **RG12.31.9** radii: `radiusSphere` (`ContactSphereTorus`, `ContactSphereTriangle`) - `sphereRadius`, as
      `circleRadius`, `discRadius`, `cylinderRadius`; `ContactConvexRoll.rBoundingSphere` - `boundingSphereRadius`;
    - **RG12.31.10** the friction of `ObjectConnectorCoordinateSpringDamperExt`: `fDynamicFriction`,
      `fStaticFrictionOffset`, `fViscousFriction` - without the `f`, as in `ContactConvexRoll`;
    - **RG12.31.11** `constrainRotation` (`JointPrismatic2D`, `JointSliding2D`) against `constrainRotations`
      (`JointSliding`, beside `constrainTranslations`) - one form;
    - **RG12.31.12** `ObjectConnectorCoordinate.factorValue1` against `factor0`/`factor1` of
      `CoordinateSpringDamperExt` - `factor1`;
    - **RG12.31.13** `ObjectConnectorRollingDiscPenalty` has `viscousFriction` and `rollingFrictionViscous` -
      `rollingViscousFriction`, if both stay;
    - **RG12.31.14** `MarkerSuperElementRigid.useAlternativeApproach` says nothing - `useAlternativeRotationMode`
      (the name of the C++ argument since RG9.3.4.4);
    - **RG12.31.15** the prefix `physics` (`physicsMass`, `physicsInertia`, `physicsAxialStiffness`, ... - 30
      parameters of bodies and finite elements) that connectors (`stiffness`, `damping`) and contacts do not
      have - drop it, or keep it as the mark of a body's physical data; touches most scripts, so for 2.0 if at all;
    - **RG12.31.16** flags in three patterns (`intrinsicFormulation`, `classicalFormulation`,
      `usePenaltyFormulation`, `useReducedOrderIntegration`) - `use...` for all, the lowest priority.

  Not listed: `localPosition` of the rigid markers beside `localHT` (its deprecation is RG14.2.15/RG16.5), the
  names of the internal members (`temp...`, RG14.2.16), and `axisMarker0` of `LinearSpringDamper`/`JointPrismatic2D`,
  an axis and not a rotation, which stays.

<a id="rg12-32"></a>
**RG12.32** *(group RG12; maintainer 2026-10-03: "think about the user"; **proposal, waits for the maintainer's
decision**)* **A deprecation warns where the user wrote the deprecated name** (#2804). The warning about
`rotationMarker0/1` (RG14.2.15) comes from `CheckPreAssembleConsistency`, so Python attributes it to the line of
`mbs.Assemble()`, and the message has to say which item - the user still searches for the line that wrote it.
*Proposal:*
    - **RG12.32.1** **one switch**, `exudyn.special.deprecations` (a small module, like `exudyn.special.userInterface`):
      `warnOnce = True` (default) - each deprecated name warns once per session, a set of the names already warned
      about; `warnOnce = False` - every use warns, for debugging a script; one function `Warn(key, message,
      stacklevel)` that all deprecations call, from Python and - through `PyDeprecated`, which imports it - from C++:
      the settings of RG12.1, the item renames of RG12.2, the renderer functions and the deprecated parameters. The
      one-per-session flag in C++ (RG14.2.15) goes. Changes RG12.1/RG12.2 from "every use" to "once per name", and
      their two test models count accordingly.
    - **RG12.32.2** **declared, not hand-written**: an item parameter that stays but is deprecated (no new name:
      `rotationMarker0/1`) gets `deprecated=Deprecated(since, expires, stays=True)` and its advice as
      `deprecatedAdvice`; RG12.2's renames keep theirs. The generators emit the warning **in the item class of
      `itemInterface.py`**, when the argument differs from the default, with the stacklevel of the user's call - so
      the warning names the line `ObjectJointGeneric(..., rotationMarker0=A)` - and in the generated
      `SetWithDictionary`/`SetParameter` for the dictionary and `SetObjectParameter` paths (one warning per
      call path, the switch decides). `CheckPreAssembleConsistency` of the five items goes back to what it was.
    - **RG12.32.3** **no warning from the library itself**: a `Create...` function given a `MarkerIndex` with a
      rotation adds a copy of that marker with the composed `localHT` (as `_MarkerWithRotation` of the robotics),
      instead of passing the rotation on in the deprecated parameter.
    - **RG12.32.4** **`exudev scripts` reports deprecated item parameters** (#2805): `checkUserScripts.py` reads
      the declarations of RG12.32.2 and RG12.2 from `definitions/` and reports a keyword of an item class
      (`ObjectJointGeneric(rotationMarker0=...)`, also through `GenericJoint`) and a key of an item dictionary
      (`'rotationMarker0':`) with the advice; and the removed `ObjectContactCurveCircles.rotationMarker0` (#2803) in
      its table of removed names. Independent of .1-.3 for the reading part.

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
**RG13.4** **DONE 2026-09-29** (#2721) — [log](exudynRevisionLog2026b.md#rg13-4) · [plan text](exudynRevisionLog2026b.md#plan-rg13-4) — The development documents per item type.

<a id="rg13-5"></a>
**RG13.5** *(group RG13; maintainer 2026-09-27)* **The documentation of the items, written by the
    documents of RG13.4** (#2725) - **DONE 2026-09-28**, `ObjectBeamGeometricallyExact` on 2026-09-30 by
    RG4.8.9. *"start a new step RG13.5, which adds according documentation for
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
        bodies - the nonlinear finite elements; `ObjectBeamGeometricallyExact` by RG4.8.9. Found on
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
      - **RG13.5.2.7** *(maintainer 2026-09-30)* **DONE 2026-09-30** — [log](exudynRevisionLog2026b.md#rg13-5-2-7) -
        the pages of `ObjectFFRF` and `ObjectFFRFreducedOrder` linked three example pages, which the PDF
        leaves out (#2758): the scripts are named and listed in the field `examples`, and
        `tools/checkDefinitions.py` rejects such a link.
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
    - **RG13.6.6** *(maintainer 2026-09-29: later; #2732 closed 2026-10-03, the step is in the list
      [*not decided to be resolved*](#not-decided))* `ObjectFFRF` and `ObjectFFRFreducedOrder` get their
      MiniExamples when tetrahedral finite elements are part of Exudyn itself; until then a mesh comes from
      a file or from NGsolve, and their pages name a complete model instead (`NGsolveFFRF.py`,
      `NGsolveCMStutorial.py`, `objectFFRFreducedOrderTest.py`). `ObjectBeamGeometricallyExact` has its
      MiniExample since RG4.8.9.

<a id="rg13-7"></a>
**RG13.7** **DONE 2026-09-29** (#2742) — [log](exudynRevisionLog2026b.md#rg13-7) · [plan text](exudynRevisionLog2026b.md#plan-rg13-7) — How to set up a new item.

## RG14 — Marker values computed where they are used

*(Group created by the maintainer, 2026-09-29.)* Today every connector, joint, constraint and load gets
a `MarkerData` for each of its markers, computed before it is called - positions, orientations,
velocities and full Jacobians, whether it needs them or not. The alternative: the connector or load
computes the marker values itself, with a small temporary per marker. **The big advantage is automatic
differentiation**, which then sees the whole computation from the coordinates to the force. It shapes
how future items and the user elements (RG7, RG8) are written, so it is decided first, even if it is not
done.

<a id="rg14-1"></a>
**RG14.1** *(group RG14; maintainer 2026-09-29)* **CLOSED 2026-10-01, superseded by RG14.2** (maintainer) —
    in `tmp/evalRG14_1_markerData.md` (not kept in the repository). Proposed: connectors and loads call
    marker functions with a compact temporary, after RG9.3, loads first; GeneralContact keeps its
    precomputation; the AD benefit needs the body side of RG15. **The evaluation** (#2745): what `MarkerData` holds
    and costs today, who computes and who reads which part of it (connectors, constraints, loads,
    `GeneralContact`), and the options - a smaller temporary per marker, marker functions a connector
    calls, what automatic differentiation needs from them. The result is a proposal for the maintainer:
    whether, and which option.

<a id="rg14-2"></a>
**RG14.2** *(group RG14; maintainer 2026-09-30, after reading RG14.1)* **The migration** (#2745). The
    maintainer's frame: a **fallback** to today's path until there is evidence that the new one computes
    the same; **no temporary data per item**, only per system and thread, fewer temporaries with generic
    names; **measure first**; two layers - the connector computes from a small structure without
    precomputed matrices, the system computes the marker data per marker kind (position, rigid,
    coordinate) and transports only what is needed, templated for automatic differentiation; a unified
    `ComputeConnectorForce`; homogeneous transformations for rigid markers, internally and later perhaps
    for the user; a pilot connector, then classes of connectors, each a step; `GeneralContact` compatible.
    The proposal is in `tmp/evalRG14_2_connectorInterface.md` (not kept in the repository): L0 marker
    kinematics (fixed size, templated, a homogeneous transformation for rigid markers), L1 a connector
    force as a pure function of them, L2 the system function per marker kind with the inner Jacobian by
    AD over the marker kinematics - which does not need RG15 -, a `Legacy` branch and an experimental
    switch as the fallback. Sub-steps, as proposed there:
    - **RG14.2.1** **DONE 2026-09-30** — [log](exudynRevisionLog2026b.md#rg14-2-1) - the MiniExample
      performance run `python/testing/runMiniExamplePerformance.py [--full] [--processes N]`: the generated
      MiniExamples with a dynamic solve appended, `miniExamplePerformanceTest` in the item definitions, ~2 s
      per example in full and a tenth in the regular run, in parallel on 80 % of the physical cores, the
      solver timers recorded, not in fast mode, `--compare` of two logs; the baseline log of the full run;
      **RG14.2.1.1** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg14-2-1-1) - the legacy path and
      the new one side by side in the performance run, `python/PerformanceModels/perfConnectorInterface.py`;
    - **RG14.2.2** the interface decided (the questions in section 9 of the evaluation). **Decided
      2026-09-30** (maintainer): one global experimental switch as the fallback; the field is
      `miniExamplePerformanceTest`; the regular performance run a tenth of the full one; loads **not**
      before the rigid-marker connectors; automatic differentiation is for later, not in the first
      migration. **Decided 2026-10-01** (maintainer) on the three open points: (a) the rigid marker keeps
      the mixed form - $\Hm$, $\vv$ global, $\tomega$ local - with a `BodyTwist` function, possibly a member
      of the homogeneous transformation; (b) automatic differentiation over separate vectors with a
      seeding helper, as proposed; (e) the second layer, one function per marker kind, **in `CSystem`**;
    - **RG14.2.3** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg14-2-3) - L0, the per-thread
      `MarkerTemp`, the marker functions and the dispatch with `Legacy` for every connector - results identical;
    - **RG14.2.4** the pilot `ObjectConnectorSpringDamper`, its inner Jacobian by AD, compared with the
      legacy path in results, Newton iterations and timers. **The force (ODE2) part DONE 2026-10-01** —
      [log](exudynRevisionLog2026b.md#rg14-2-3) - results identical, the right-hand side up to 20 % faster
      on rigid bodies; **RG14.2.4.1** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg14-2-4-1) - the
      inner Jacobian by AD with the seeding helper, in `CSystem` (`ComputeJacobianODE2PositionMarkers`), equal to
      the analytic legacy Jacobian to round-off, the same Newton iterations;
    - **RG14.2.5** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg14-2-5) - the other position-marker
      connectors: `CartesianSpringDamper` (force and Jacobian by AD), `ConnectorGravity` (force, and a Jacobian by AD
      where the legacy path differentiates numerically: the Jacobian 5× faster), `HydraulicActuatorSimple` (force; its
      Jacobian stays numerical, it couples to its ODE1 node); **RG14.2.6** **DONE 2026-10-01** —
      [log](exudynRevisionLog2026b.md#rg14-2-6) - the coordinate-marker connectors: L0 `MarkerCoordinate`, L2 with
      the Jacobian by AD, `CoordinateSpringDamper`; `CoordinateSpringDamperExt` (friction states, post Newton) and
      `ContactCoordinate` go with the contact connectors (RG14.2.10);
    - **RG14.2.7** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg14-2-7) - the loads.
    - **RG14.2.8** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg14-2-8) - the rigid-marker force
      connectors (`RigidBodySpringDamper`, `LinearSpringDamper`, `TorsionalSpringDamper`) on `MarkerRigid` with the
      frame as a homogeneous transformation, forces and torques per marker; their Jacobians stay numerical.
    - **RG14.2.8.1** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg14-2-8-1) - the connector Jacobian of
      the rigid-marker connectors by AD, **decided as proposed (maintainer 2026-10-01)**; the intrinsic formulation of
      the rigid-body spring-damper and the user functions stay numerical.
      - **RG14.2.8.1.1** **DONE 2026-10-01** (#2770) - `RotXYZGTv_qTemplate`, $\partial(\Gm\tp\vv)/\partial\qv$ for
        Tait-Bryan angles, had a wrong sign in entry (2,1); found by the comparison of RG14.2.8.1.
      *Today*: these three connectors declare no Jacobian function, so `CSystem::JacobianODE2RHS` differentiates the
      whole connector numerically - one evaluation of its right-hand side (marker data with Jacobians for both markers,
      the physics, the projection) per coordinate of both bodies, for the positions and again for the velocities: up
      to $2\times14+1$ evaluations per connector with Euler parameters. The analytic marker Jacobians and
      `ComputeMarkerDataJacobianDerivative` exist for `MarkerBodyRigid` and `MarkerNodeRigid`, but only the connectors
      with an analytic Jacobian use them (spring-damper, Cartesian, coordinate, gravity); the rigid ones do not.
      *Proposed*: the same chain as for position markers, with 12 directions - per marker 3 translations and 3
      rotation increments, the positions and rotations seeded with `factorODE2`, the velocities and angular
      velocities with `factorODE2_t` in the same directions. The rotation increment is global,
      $\Rot(\delta\thetav) = (\Im + \delta\tilde\thetav)\Rot$, so that it chains with the marker's rotation Jacobian
      ($\omegav = \Jm_{rot}\dot\qv$, global); the local angular velocity is formed in the AD type as
      $\Rot(\delta\thetav)\tp(\Rot\,\tomega_{local} + \delta\omegav)$. The connector returns force and torque per
      marker, so the inner Jacobian is four $6\times6$ blocks $\partial(\fv_i,\ttau_i)/\partial(\pv_k,\thetav_k)$, chained
      as $[\Jm_{pos,i};\Jm_{rot,i}]\tp \Km_{ik} [\Jm_{pos,k};\Jm_{rot,k}]$, plus `ComputeMarkerDataJacobianDerivative` per
      marker with **its own** force and torque (`ChainConnectorJacobian` generalized from "force on marker 1, reaction on
      marker 0" to a force and torque per marker). $\partial\vv/\partial\qv$ and $\partial\omegav/\partial\qv$ neglected,
      as for the position markers. `BodyTwist` (RG14.2.2 (a)) comes with it.
      *Needed*: the physics of the three connectors as templates; `RotationMatrix2RotXYZ` templated (today `Real`
      only), and for the intrinsic formulation `GetRelativeMotionTo`/`LogSO3`/`ExpSE3` checked for `AutoDiff`;
      `atan2` (and `asin`) added to `AutoDiff`, which has `sin`, `cos`, `atan`, `exp`, `sqrt` but not these. A
      user function keeps the numerical Jacobian.
      *Markers*: `MarkerBodyRigid` and `MarkerNodeRigid` (analytic Jacobian and its derivative); `MarkerKinematicTreeRigid`
      and `MarkerSuperElementRigid` have no Jacobian derivative and stay numerical, as for the analytic connectors today
      (unless `jacobianConnectorDerivative` is switched off).
      *Expected gain*, from the benchmark of RG14.2.8 (100 bodies, rigid-body spring-damper, generalized-alpha): the
      numerical Jacobian is 0.47 s of 0.75 s; one AD pass (about 5-15 force evaluations' worth) plus the analytic chain
      instead of up to 29 evaluations - a Jacobian 3-5× faster, the implicit total about 1.5-2× (the gravity connector,
      the same change on position markers, measured 5× on the Jacobian). Explicit solvers gain nothing. The Jacobian
      becomes exact instead of numerical; on Euler parameters it lacks the normalization direction, as for the
      gravity (+6 % Newton steps there). Checked against the numerical legacy Jacobian ($10^{-6}$) and in
      `perfConnectorInterface.py` with an implicit rigid run.
    - **RG14.2.9** constraints and joints on L0/L1/L2 - see [RG14.2.9 in detail](#rg14-2-9) below.
    - **RG14.2.10** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg14-2-10) - the contact connectors (with
      RG4.16): `ContactSphereSphere`, `ContactSphereTriangle`, `ContactSphereTorus`, `ContactConvexRoll`,
      `ConnectorRollingDiscPenalty`, `ContactCoordinate`, `ConnectorCoordinateSpringDamperExt`;
    - **RG14.2.11** **DECIDED 2026-10-02** (maintainer: *these items stay on the old path*) — [log](exudynRevisionLog2026b.md#rg14-2-11) -
      the special markers (shape, cable, many markers): the items that stay on the path of the marker data, and why;
      they keep that path as their own - since RG14.2.17 the default of the connector's functions of the interface;
      RG14.2.13 removes the switch, not the path;
    - **RG14.2.12** **MEASURED 2026-10-02, not now** — [log](exudynRevisionLog2026b.md#rg14-2-12) - `GeneralContact` on
      L0: the gain is about 2 %, it keeps its precomputation (as RG14.1 proposed); to be looked at again when the
      projection comes from the bodies (RG9.3.4);
    - **RG14.2.13** **DONE 2026-10-02** (the switch and the legacy functions; the output variables moved to RG14.2.18) —
      [log](exudynRevisionLog2026b.md#rg14-2-13) - output variables and sensors through the connector force, then the
      legacy switch and the unused temporaries removed; the path of the marker data stays for the items of RG14.2.11
      (decided).
    - **RG14.2.18** **DONE 2026-10-02** ((1) and (3) done, (2) and (4) stay, with reasons; maintainer: *do as
      suggested, keep overheads and duplication small*) — [log](exudynRevisionLog2026b.md#rg14-2-18) -
      *(found in RG14.2.13)* **what still goes through the marker data
      structure on the interface items**: (1) the output variables and sensors - `GetOutputVariableConnector` of every
      connector reads a `MarkerDataStructure`, which `CSensorObject::GetSensorValues` and `MainSystem::GetObjectOutput`
      allocate per call; the physics are shared with the interface, so it is the transport, not duplicated code - the
      output variables from the kinematics of the interface, and a per-thread structure for the rest; (2) `PostNewtonStep`
      of the contact and spring-damper connectors, the same; (3) `ContactSphereSphere` with friction and
      `ContactSphereTriangle` on markers without orientation keep their own `ComputeODE2LHS` - a mixed interface (one
      rigid, one position marker) would end that; (4) `ObjectConnectorCoordinate` at velocity level keeps its
      equations and Jacobian on the marker data - the interface at velocity level needs the Jacobian by the velocities
      (`AE_ODE2_t`).
    - **RG14.2.19** **DONE 2026-10-02** (no reader found: no declaration needed) — [log](exudynRevisionLog2026b.md#rg14-2-19) -
      *(found in RG14.2.18; #241, open since 2019, needs no decision)* **`PostNewtonStep` without the
      marker Jacobians**: `CSystem::PostNewtonStep` computes the marker data structure with `computeJacobian = true` for
      every connector with a discontinuous iteration, in every Newton iteration. 10 of the 16 implementations read no
      Jacobian (the contacts on spheres, triangles, tori, convex rolls, coordinates, the spring-dampers); the cable
      contacts, `ContactCurveCircles` and the sliding joints read their markers' `jacobian` (the shape functions of the
      cable markers) through `ComputeGap` and similar. A declared function of the connector - *its PostNewtonStep needs
      the Jacobians* - with the default true, false for the ten, and a measurement on a contact model (expected: the
      Jacobians of rigid body markers are a few % of a contact model's step).
    - **RG14.2.20** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg14-2-20) - *(found in RG14.2.18; needs no
      decision)* **a sensor's value vector without allocation** (#2778):
      after RG14.2.18 a sensor evaluation still allocates its value `Vector` once (4 M allocations for 200 sensors over
      20000 steps); a `ResizableVector` kept per sensor, or the value written into the storage directly.
    - **RG14.2.17** **DONE 2026-10-02** (maintainer: *yes, do the proposed way*) — [log](exudynRevisionLog2026b.md#rg14-2-17) -
      **the dispatch in the connector instead of in `CSystem`.** Today `CSystem` asks `GetConnectorInterface()` and switches on the enum to its
      L2 functions (`ComputeODE2LHS*Markers`, `ComputeJacobianODE2*Markers`, the constraint functions). Proposed: one
      virtual function per operation in `CObjectConnector` - the right-hand side, the Jacobian, the constraint equations
      and their Jacobian, the reaction forces - whose default is the path of the marker data, and which a connector on
      the interface overrides by calling the shared L2 chain of its kind of markers. The L2 chains move out of
      `CSystem.cpp` into one file of free functions (templates where the AD types need them), not into intermediate
      parent classes: a connector chooses its kind at run time (`ContactSphereSphere` on position or rigid markers),
      which a parent class cannot express, and the generator would need the parents in its headers. Gained: `CSystem`
      no longer knows the kinds of connectors, a new kind touches one file, and the enum is gone; not gained:
      performance (a virtual call replaces a switch plus a virtual call). Proposed to be done with RG14.2.13; done before
      it, so that RG14.2.13 only removes the switch.
    - **RG14.2.14** **DONE 2026-10-02** (.1-.4) *(maintainer 2026-10-01)* **`MarkerTemp` without `MarkerData`.** Why it holds one today: the L0/L1
      pair of a marker (`GetKinematicsRigid`/`AddGeneralizedForceTorque`, `GetODE2Size`/`AddGeneralizedForce`,
      `GetKinematicsCoordinate`/`AddGeneralizedForceCoordinate`) has a **default in `CMarker`** that calls the old
      `ComputeMarkerData` and keeps its Jacobians in `temp.markerData` between the two calls - so that every marker
      works on the new path before it is migrated. Measured 2026-10-01: own implementations exist for
      `MarkerBodyRigid` (through `CObjectBody::GetKinematicsRigid`/`AddForceTorque`, which needs no `MarkerData`),
      `MarkerBodyPosition`, `MarkerNodePosition` and `MarkerNodeCoordinate`; `MarkerNodeRigid`,
      `MarkerSuperElementRigid/Position`, `MarkerKinematicTreeRigid`, the beam/cable/relative-coordinate markers
      still go through the default. The clean form: every marker implements its pair, and what it keeps between
      the two calls is **its own small fixed-size state** (a node index and $\Gm$ as a `ConstSizeMatrix<3*4>` for a
      node marker; the body's $\Gm_{loc}$ for a body marker), so `MarkerTemp` becomes a small union-like buffer of
      fixed size instead of a `MarkerData` with two `ResizableMatrix`; the `MarkerData` of the fallback moves to
      the legacy path (a per-thread `MarkerDataStructure` that exists anyway) and disappears with it
      (RG14.2.13). Done marker by marker, together with RG14.2.11; nothing to gain from doing it before.
      *Sub-steps (2026-10-02; the inventory of the markers in the [log](exudynRevisionLog2026b.md#rg14-2-14-1)):*
      - **RG14.2.14.1** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg14-2-14-1) - `MarkerTemp` gets the
        fixed-size state of a rigid frame (rotation, G, G_local); `ObjectRigidBody` keeps it there instead of two
        `ResizableMatrix`; `MarkerNodeRigid` gets its own L0/L1 for the 3D rigid body nodes;
      - **RG14.2.14.2** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg14-2-14-2) - the coordinate markers
        that a coordinate connector takes: `MarkerNodeRotationCoordinate` on its own L0/L1; `MarkerNodeODE1Coordinate`,
        `MarkerNodeCoordinates` and the relative coordinate markers stay on the default (reasons in the log);
      - **RG14.2.14.3** **DONE 2026-10-02** (the superelement markers; the kinematic tree stays on the default) —
        [log](exudynRevisionLog2026b.md#rg14-2-14-3) - `MarkerSuperElementPosition`/`Rigid` and `MarkerKinematicTreeRigid` - their Jacobians are
        dense in many coordinates, their own L1 projects without forming them where possible;
      - **RG14.2.14.4** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg14-2-14-3) - the Jacobian chains (`ConnectorJacobianODE2*Markers`, `ConstraintJacobian*Markers`) take the
        marker Jacobians from a function of the marker (`GetJacobiansRigid`/`Position`) instead of `ComputeMarkerData`;
        then the `MarkerData` of `MarkerTemp` serves only the markers that keep the path of the marker data
        (RG14.2.11) and moves to a per-thread structure (`TemporaryMarkerDataStructure`).
    - **RG14.2.16** *(maintainer 2026-10-02)* **`TemporaryComputationData` smaller**: it holds 22 members per thread
      (measured 2026-10-02), most for the legacy path and its matrices: `markerDataStructure` (connectors, constraints and
      loads on the legacy path, `GeneralContact`), `localJacobianAE_ODE2/_ODE2_t/_ODE1/_AE` (the hand-written constraint
      Jacobians and the numerical one), `generalizedLoad`/`loadJacobian` (loads), `localJacobian`/`localJacobian_t`
      (numerical differentiation, the analytic legacy connectors, contact), `jacobianTemp`, `jacobianODE2Container`,
      `numericalJacobianf0/f1`, `tempIndex`-`tempIndex4`, `tempValue`/`tempValue2`, and `markerTemp[2]` of the new path.
      The goal: per thread only what the new path needs - the marker temporaries (after RG14.2.14 small and fixed in
      size), one local vector and one local matrix with generic names, the sparse buffers - and nothing kept for a
      single caller. Sub-steps when it starts: (1) the inventory as a table, member by member: which function uses it,
      on which path; (2) the members used only by the legacy path go with it (RG14.2.13); (3) the rest renamed to
      generic temporaries and shared. **Blocking**: the legacy path and its switch (RG14.2.13), which waits for the
      special markers (RG14.2.11), the contact connectors and `GeneralContact` (RG14.2.10, RG14.2.12) and the
      remaining constraints (`JointRollingDisc`, `ConnectorCoordinateVector`, the sliding joints); the numerical
      differentiation of objects and connectors, which stays; and the objects' own `ComputeODE2LHS`/mass matrix
      temporaries until RG15.
      - **RG14.2.16.1** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg14-2-16-1) - the inventory, member by
        member, grouped: no member serves the legacy path any more, so (2) has nothing left to remove; (3) proposed -
        the two matrices of one `GeneralContact` caller into its own temporaries, `tempIndex[4]`, `tempValue`/`2`
        named for `PostNewtonStep`.
      - **RG14.2.16.2** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg14-2-14-2) - (3) as proposed: the two
        matrices of `GeneralContact` are `tempMatrix`/`tempMatrix2`, the two numbers `postNewtonError`/`postNewtonStepSize`;
        the index arrays keep their generic names. 25 members remain, each with a user.
    - **RG14.2.15** **DONE 2026-10-03** — [log](exudynRevisionLog2026b.md#rg14-2-15) *(maintainer 2026-10-01; the
      parameter `localHT` in the rigid markers with RG16.3.3, #2795; the deprecation, once per session and removed in
      2031, the documentation and the examples with #2745)*
      **Markers with a rotation; the joints' `rotationMarker0/1` deprecated.** A rigid marker gets a local frame:
      **`localHT`** (a homogeneous transformation in the body) as the alternative to `localPosition`, which is
      deprecated later; the marker frame is then body frame × `localHT`, and the connector receives it as
      `MarkerRigid.frame` (RG14.2.8) - nothing changes at L1/L2. The joints and connectors with
      `rotationMarker0/1` (eight objects: `JointGeneric`, `JointRevoluteZ`, `JointPrismaticX`, the
      connectors `CartesianSpringDamper`, `RigidBodySpringDamper`, `LinearSpringDamper`, `TorsionalSpringDamper`
      and `ContactCurveCircles`) then have **two rotations in a row** - marker and
      connector - and the connector's ones are deprecated: internally a flag *rotation markers are not identity*,
      set at `CheckPreAssembleConsistency`, so that the extra products are computed only in that deprecated case.
      *(2026-10-02: the parameter and its deprecation are planned together with the homogeneous transformations of
      the user interface, RG16.5.)* Sub-steps when it starts: the parameter in the rigid markers (`MarkerBodyRigid`, `MarkerNodeRigid`,
      `MarkerSuperElementRigid`, `MarkerKinematicTreeRigid`) and their L0; the flag and the deprecation warning in
      the joints; the documentation and the examples moved to `localHT`. Until then, new marker and connector code
      takes the frame from `MarkerRigid` and does not add new uses of `rotationMarker0/1`.
      - **RG14.2.15.1** **DONE 2026-10-03** (#2801) — [log](exudynRevisionLog2026b.md#rg14-2-15) - found while
        moving the Create functions: `RigidBodySpringDamper` multiplied `rotationMarker0` from the left of the marker
        frame, all joints and its own page from the right.
      - **RG14.2.15.2** **DONE 2026-10-03** (#2803) — [log](exudynRevisionLog2026b.md#rg14-2-15-2) - *(maintainer
        2026-10-03: "what for?")* `ObjectContactCurveCircles.rotationMarker0` **removed without deprecation**: the
        contact never used it, only the drawing, and `Assemble()` refused any value but the unit matrix.

<a id="rg14-2-9"></a>
**RG14.2.9 in detail** *(proposed 2026-10-01; **decided as proposed by the maintainer, 2026-10-01**: (a) the term
$\partial(\Cm_\qv\tp\lambdav)/\partial\qv$ available and off, (b) and (c) as written)* - constraints and joints on the
connector interface.

*What the code does today.* 13 constraint objects (`CObjectConstraint`, derived from `CObjectConnector`), by the
marker kind they request:

| kind | constraints | lines of the .cpp |
|---|---|---|
| position | `ConnectorDistance`, `JointSpherical`, `JointRevolute2D` | 160, 179, 140 |
| position + orientation (rigid) | `JointGeneric`, `JointRevoluteZ`, `JointPrismaticX`, `JointPrismatic2D`, `JointRollingDisc` | 706, 382, 314, 219, 347 |
| coordinate | `ConnectorCoordinate`; vector: `ConnectorCoordinateVector` | 172, 310 |
| special (cable, ALE, sliding) | `JointSliding`, `JointSliding2D`, `JointALEMoving2D` | 397, 421, 375 |

Each writes its equations $\gv$ in `ComputeAlgebraicEquations(localAE, markerData, t, itemIndex, velocityLevel)` and its
Jacobian $\Cm_\qv$ **by hand** in `ComputeJacobianAE`, from the marker data with all Jacobians
(`ComputeMarkerDataStructure`). `CSystem` asks for them at four places: the equations (`ComputeAlgebraicEquations`),
the Newton matrix (`JacobianAE`: $\Cm_\qv$ and $\Cm_\qv\tp$), **the reaction forces in every right-hand side**
(`ComputeODE2ProjectedReactionForces`: the full local $\Cm_\qv$, $m\times n$, formed and multiplied with $\lambdav$), and
the initial accelerations ($(\Cm_\qv\dot\qv)_\qv$, numerical over `ComputeConstraintJacobianTimesVector`). The term
$\partial(\Cm_\qv\tp\lambdav)/\partial\qv$ is **not** in the Newton matrix today.

*Proposed.* The constraint's equations become a template of the marker kinematics only,
`ComputeConstraintEquations<TReal>(markers, t, itemIndex, velocityLevel, g)` - the joint's geometry, nothing else,
on `MarkerPosition`, `MarkerRigid` or `MarkerCoordinate`. `CSystem` does the rest, once per marker kind:
1. the equations: the template with `Real`;
2. $\Cm_\qv$ by AD over the marker kinematics (the directions of RG14.2.4.1 for position markers, of RG14.2.8.1 for
   rigid ones, chained with the marker Jacobians) - no hand-written `ComputeJacobianAE`;
3. the reaction forces without the $m\times n$ matrix: $\fv_k = (\partial\gv/\partial\pv_k)\tp\lambdav$,
   $\ttau_k = (\partial\gv/\partial\thetav_k)\tp\lambdav$ projected with `AddGeneralizedForce(Torque)` - $\Cm_\qv\tp\lambdav$
   is a connector force with $\lambdav$ as a parameter;
4. velocity-level constraints (`UsesVelocityLevel`, index 2) seed the velocity directions instead.

*Sub-steps*, each with the fallback switch and the comparison of residuals, $\Cm_\qv$ and reaction forces (to round-off,
both are analytic), solutions and timers:
- **RG14.2.9.1** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg14-2-9-1) - the interface in
  `CObjectConnector`/`CSystem` and the pilot `JointSpherical` (position markers), then `ConnectorDistance` and
  `JointRevolute2D`;
  - **RG14.2.9.1.1** **DONE 2026-10-01** (#2771) - `solver.ComputeAlgebraicEquations` and `ComputeODE1RHS` linked the
    residual with an end index where a count belongs, and added the equations to an uninitialized residual;
- **RG14.2.9.2** **DONE 2026-10-01** — [log](exudynRevisionLog2026b.md#rg14-2-9-2) - `ConnectorCoordinate` (coordinate markers);
- **RG14.2.9.3** the rigid joints `JointGeneric`, `JointRevoluteZ`, `JointPrismaticX`, `JointPrismatic2D` - after
  RG14.2.8.1, which brings the rotation directions; their equations are ported as they are (RG14.3 then rewrites
  them on $\Hm_0^{-1}\Hm_1$); **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg14-2-9-3),
  `JointGeneric`: [log](exudynRevisionLog2026b.md#rg14-2-9-3-generic);
  - **RG14.2.9.3.1** **DONE 2026-10-02** (#2772) - the hand-written Jacobian of `JointGeneric` ignored
    `alternativeConstraints`; on the new path the joint has the Jacobian of its own equations;
- **RG14.2.9.4** **ON HOLD (maintainer, 2026-10-02)**; it blocks no other step. *What it means*: the Newton matrix
  of a constrained system contains $\Cm_\qv\tp$ but not the derivative of the reaction forces $\Cm_\qv\tp\lambdav$ by the
  coordinates, $\partial(\Cm_\qv\tp\lambdav)/\partial\qv$ - the "geometric stiffness" of the joints. It is zero for linear
  constraints (coordinate constraints, the translations of joints with global equations) and small while the
  Lagrange multipliers are small; with large reaction forces at large rotations it would improve the convergence of
  Newton, and it changes no solution, only the iterations. *What it would need*: (1) the second derivatives of the
  equations - the templates of RG14.2.9 take a nested `AutoDiff<12, AutoDiff<12>>` as they are, at 144 directions per
  rigid joint, or the derivative of the reaction-force function $(\partial\gv/\partial\pv_k)\tp\lambdav$ by AD, which
  needs the same; (2) the derivative of the marker Jacobians with $\lambdav$ as the force -
  `ComputeMarkerDataJacobianDerivative`, as for the connectors (RG14.2.8.1); (3) a `newton` setting, off by default,
  and the chain into the ODE2-ODE2 block of the Newton matrix; (4) tests of convergence on large-rotation models.
  *Why on hold*: the gain is limited to models with large reaction forces at large rotations, the cost per joint is
  high (a 144-direction AD pass), and modified Newton usually hides the missing term; it may not be worth adding in
  general. Correction of the original proposal: it is not "almost free";
- `JointRollingDisc`, `ConnectorCoordinateVector` and the special joints go with RG14.2.11, or stay legacy.

*For the maintainer to decide*:
(a) whether $\partial(\Cm_\qv\tp\lambdav)/\partial\qv$ enters the Newton matrix - available almost free by AD of item 3
(the connector Jacobian of a force with $\lambdav$ fixed); it improves Newton convergence for large rotations but changes
results within the tolerance and needs re-recorded references; proposed: available, **off** by default
(a `newton` flag), so the port itself changes nothing;
(b) the split against RG14.3: RG14.2.9 moves the joints onto the interface with their equations unchanged; RG14.3
reformulates and unifies the equations - proposed as written here;
(c) the order: the rigid joints wait for RG14.2.8.1 - proposed.

*Expected gain.* The right-hand side no longer forms $\Cm_\qv$ for the reaction forces (today one full
`ComputeMarkerDataStructure` and `ComputeJacobianAE` per constraint and evaluation); the Newton matrix about as today.
The larger gain is the code: about 590 lines of hand-written `ComputeJacobianAE` in the eight ported constraints
(234 of them in `JointGeneric`) are replaced by one AD chain per marker kind - and with them the class of errors that a
hand-written $\Cm_\qv$ allows.

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
**RG15.1** *(group RG15; maintainer 2026-09-29)* **DONE 2026-09-29, for the maintainer's decision** —
    in `tmp/evalRG15_1_objectCoordinates.md` (not kept in the repository). Found: five finite elements
    already gather their coordinates and use automatic differentiation for the Jacobian. Proposed: that
    pattern as the standard, the rotation parametrizations as templated functions for the rigid bodies,
    decided together with RG14. **The evaluation** (#2746): how the objects read their
    coordinates today, what passing them would cost (measured, RG5), which kinds of objects there are -
    one node with linked data, several nodes, super elements, the kinematic tree -, and what automatic
    differentiation needs. The result is a proposal for the maintainer, including whether RG14 and RG15
    are one interface.

## RG16 — Homogeneous transformations

*(Group created by the maintainer, 2026-10-02.)* A rigid frame - position and rotation - is one homogeneous
transformation (HT). The C++ class exists (`HomogeneousTransformationBase` in `RigidBodyMath`, the frames of the
connector interface since RG14.2.8), Python has `rigidBodyUtilities.HomogeneousTransformation` on 4x4 numpy arrays,
and the items take a position and a rotation matrix. The group makes the C++ class fast and reachable from Python, and
then lets the rigid items take and give an HT. **Why now** (maintainer): the user items to come - in Python (RG7) and
in C++ (RG8) - shall meet the newer interfaces from the start, not deprecated ones; for robotics, the kinematic tree and
the joints an HT means fewer variables and one way of doing things.

<a id="rg16-1"></a>
**RG16.1** *(group RG16; maintainer 2026-10-02; a step of its own, independent of the interface steps)* **The C++
    class and its Python binding** (#2780).
    - **RG16.1.1** to **RG16.1.5** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg16-1),
      [RG16.1.5](exudynRevisionLog2026b.md#rg16-1-5) (the HT as an item parameter waits for RG16.2)
    - **RG16.1.7** **DONE 2026-10-02** - *(maintainer 2026-10-02)* `exu.HT` also takes the 16 values of a 4x4 matrix row
      by row, as a sensor stores the output variable (#2792): `exu.HT(values)`.
    - **RG16.1.6** **DONE 2026-10-02** — [log](exudynRevisionLog2026b.md#rg16-1-6) - *(maintainer 2026-10-02: "make
      this the standard way")* **the output variable `HomogeneousTransformation` as `exu.HT`** (#2789): the
      Get...Output functions return an `exu.HT`; a sensor stores the 16 values row by row.
    - **RG16.1.1** the class into a file of its own, out of the `exulie` namespace; it stores only the 12 numbers it
      needs (rotation and translation);
    - **RG16.1.2** performance for what is hot - $\Hm\vv$, $\Hm^{-1}$, $\Hm_1\Hm_2$, set (from $\Am$ and $\vv$, or from
      an HT) and get ($\Am$, $\vv$) -: fixed-size loops the compiler unrolls, and a flag *no rotation*, set when a frame
      is set without one, so that products skip the rotation; setting the flag must not cost (AVX2 latencies), and a
      global `constexpr` switches the behaviour, to measure both; the Lie group operators are not the hot cases;
    - **RG16.1.3** tests of the operations, and a measurement against today's class;
    - **RG16.1.4** the binding `exudyn.HT` with its operators, set and get, conversion from and to 4x4 arrays; a note
      in `rigidBodyUtilities.HomogeneousTransformation` on the faster C++ class;
    - **RG16.1.5** the HT used inside the rigid items (stored as in `ObjectGround` or `ObjectRigidBody`), and an output
      variable HT wherever position and rotation are available.

<a id="rg16-2"></a>
**RG16.2** *(group RG16; maintainer 2026-10-02)* **DECIDED 2026-10-02** (the decisions in the
    [log](exudynRevisionLog2026b.md#rg16-2-decided)) —
    [log](exudynRevisionLog2026b.md#rg16-2) - **The evaluation: HT in the user interface of the rigid items**
    (#2781), for the maintainer's decisions - unification, clarity, simplicity. The main cases: `ObjectRigidBody`,
    `ObjectGround`, the rigid body nodes, the `Marker...Rigid`. Can they take an HT instead of position and rotation, in
    a compatibility mode: both initialized with `None` (can the interface tell which of them a user set?), the default
    the zero position and the unit rotation, internally only the HT, and both still exported together with the HT? Or
    a global flag that switches back to position and rotation - possibly set automatically as soon as an item interface
    is given a position or a rotation matrix? The interface is not the performance question. What already exists (the
    `localHT` of RG14.2.15) is homogenized with it.

**RG16.3** **DONE 2026-10-03** except RG16.3.4 (in the list [*not decided to be resolved*](#not-decided); #2781
    resolved) *(group RG16)* **The cases that break nothing for users**, as decided in RG16.2 (2026-10-02): an HT
    parameter next to the position and rotation of today, both `None` for "not given", giving both raises, a 4x4 numpy
    array in the dictionary (an `exu.HT` accepted), only where it makes sense:
    - **RG16.3.1** **DONE 2026-10-03** — [log](exudynRevisionLog2026b.md#rg16-3-1) (#2793) `ObjectGround`: the HT is the internal storage; `referenceHT` next to `referencePosition` and
      `referenceRotation`, which the get/set interface keeps (and composes from the stored HT);
    - **RG16.3.2** **DONE 2026-10-03** — [log](exudynRevisionLog2026b.md#rg16-3-2) (#2794) `CreateRigidBody` (and `CreateGround`): `referenceHT` and `initialHT` - the transformation added to the
      reference -, translated into the node's reference and initial coordinates; rigid bodies and their nodes themselves
      take no HT (their state is their coordinates);
    - **RG16.3.3** **DONE 2026-10-03** — [log](exudynRevisionLog2026b.md#rg16-3-3) (#2795) the rigid body markers (`MarkerBodyRigid`, `MarkerNodeRigid`, `MarkerKinematicTreeRigid`): `localHT`,
      the marker frame = body or node frame x `localHT` (RG14.2.15, RG16.5); and `MarkerSuperElementRigid`
      *(maintainer 2026-10-02)*, where `localHT` replaces `offset` and adds a rotation - needed when the joints'
      `rotationMarker0/1` are deprecated, as a joint on a superelement then has no other place for its rotation.
    - **RG16.3.4** *(found in RG16.3.3)* a translation in the `localHT` of `MarkerNodeRigid` - today refused at
      `Assemble`, as the position Jacobian and the derivative of its transposed product need the offset in the node
      (the formulas of `CObjectRigidBody` for 3D rigid body nodes; for the slope nodes and `NodeRigidBody2D` to be
      decided) - when a case needs it.

**RG16.4** **DONE 2026-10-03** — [log](exudynRevisionLog2026b.md#rg16-4) *(group RG16)* **The further steps** - the
    mode switch, the kinematic tree and the robotics utilities on HT -, planned after RG16.3:
    - **RG16.4.1** (#2798) `ObjectKinematicTree.jointHTs`: the joint transformations and offsets as one list of HTs, a
      view of the two stored lists (a list of HTs in the generator, `htListOf`);
    - **RG16.4.2** (#2799) numpy reads an `exu.HT` as its 4x4 matrix (`__array__`); the HT functions of
      `rigidBodyUtilities` and the robotics classes take an `exu.HT`, `Robot.CreateKinematicTree` gives `jointHTs`;
    - **RG16.4.3** the mode switch: **not needed** - RG16.2 decided per item, with `None` for not given, no global mode.

<a id="rg16-5"></a>
**RG16.5** **DONE 2026-10-03 by RG16.3.3** (the decision of RG16.2: `localHT` and `localPosition` next to each other,
    `None` for not given, no break) *(group RG16; maintainer 2026-10-02)* **`localHT` in the rigid markers**, with RG14.2.15: a parameter
    `localHT`, by default `exudyn.HT0()` or `None` (to decide later between `localPosition` and `localHT`); where the
    change is made (the item interface); the break it would be - `localPosition` gone from `mbs.GetMarker()` and from
    the parameters -, handled by the deprecation of item parameters (RG12.2), or started together with it: for the
    maintainer's decision in RG16.2.

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
| RG4.1 | - | the Windows/Linux differences in contact and friction; RG4.1.2 the five macOS-only models |
| RG2.4 | #2748 | the manual GUI check, per release and platform (list and model done) |
| RG5.1 | - | a maintained micro-benchmark of the linear algebra, inside Exudyn (from #2397) |
| RG5.2 | - | make the hot linear algebra vectorizable |
| RG4.17 | #2763 | `ObjectANCFBeam`: Newton stalls in the right-angle frame - the inconsistent rotation Jacobian of the slope nodes fixed (RG4.17.1); the stall remains with a consistent Jacobian (RG4.17.2) |
| RG4.15 | #1848, #1947 | the open bugs and fixes before 1.13: `GeneralContact` against the sphere contact |
| RG6.8 | #2140, #2236, #2237, #2350 | the graphics fixes before 1.13: the Linux and macOS ones, which wait for those machines |
| RG8.1 to RG8.9 | - | the plugin ABI: registry, fingerprint, reference plugin, headers, discovery |
| RG10.1.1 | #2713 | exudev scripts also runs the scripts, in a local copy with a timeout, after a check for paths |
| RG12.31 | #2802 | settings and item parameters that could be renamed: a list for the maintainer's decision |
| RG14.3 | #2745 | joints and their Jacobians on homogeneous transformations, after RG14.2.9 |
| RG15.1 | #2746 | evaluation: objects compute from coordinates passed in |
| RG13.3 | #2717 | each description synchronized once with its implementation, recorded with a fingerprint |

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

*#2608 was done by RG6.2.11. The decision on the chapters of the user manual
(#2657, #2662), which stood below, is carried out and is in the
[log](exudynRevisionLog2026b.md#decisions-2026-09-29).*

### Recommended next

The title of each says what the step **does**; the sentence after it says why it comes here.

1. **Run the integration round of the institute, then release 1.13** (RG2.2, RG1.4). It is
   the only item on this page that needs **other people's time**, so it starts before the
   rest is ready, not after.
2. **Do the manual GUI check on Windows** (RG2.4, #2748), with the curved GraphicsData (row K13). It is
   the last condition of 1.13 that one person can meet alone.
3. **Decide the renames of RG12.31** (#2802): settings and item parameters with a name that says less than it
   could; each decided one is a small step on the deprecation mechanism of RG12.1/RG12.2.
