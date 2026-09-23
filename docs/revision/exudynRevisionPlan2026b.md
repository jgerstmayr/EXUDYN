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

Each step that came from the first plan says so: *(revision2026 step R9.1)*. The finished plan
carries the same table from its side, so a citation from either direction resolves.

## The groups

| group | what belongs in it | open steps today |
|---|---|---|
| **RG1** Release and publication | getting a release out and onto GitHub and PyPI | 4 |
| **RG2** Testing and verification | what is not tested, and who tests it before a release | 3 |
| **RG3** Docs | what the documentation still gets wrong or does not say | 11 |
| **RG4** Implementation problems and bugs | real, reproducible problems that need a plan rather than a fix | 5 |
| **RG5** Performance | measurement first, then the code that is actually hot | 2 |
| **RG6** Graphics and rendering | the renderer, the settings dialogs, and the rendering revision it is heading for | 4 |
| **RG7** Python user items | items whose behaviour is written in Python | - |
| **RG8** Compiled C++ user items | plugins: user items compiled against the shipped headers | 9 |
| **RG9** Structural core improvements | the architecture of the core, where a change touches everything | 1 |
| **RG10** Tooling and process | exudev, the issue tracker, the generators, CI | 5 |
| **RG11** Misc | what has no group yet; three of a kind become a group | 2 |
| **RG12** Python interface | the shape of the Python API itself: deprecation, settings, what a script sees | 3 |

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

## RG3 — Docs

The documentation is Markdown, built with Sphinx and published for every release since
revision2026 steps R7.1 and R7.2, and what the revision changed is carried into it. What belongs
here is what is still wrong, still missing, or newly wrong because something changed.

Its starting list is [`documentationImpact2026.md`](documentationImpact2026.md), written in
revision2026 step R7.2.2: section C of that file says which feature is documented where, and the
gaps it names are the first candidates. The maintainer's own findings go here as steps.

*No steps yet.*


<a id="rg3-1"></a>
**RG3.1** **DONE 2026-09-22** (#2584) — [log](exudynRevisionLog2026b.md#rg3-1) — *(group RG3; maintainer 2026-09-22)* **HIGH PRIORITY: the section structure of the user
    manual is wrong** (#2584). The conversion of revision2026 step R7.1.5 left the `toctree` of
    `docs/manual/introduction.md` **below its last section**, so *Exudyn Basics*, *Advanced
    topics* and *C++ Code* appear as sub-pages of *"Mapping between local and global coordinate
    indices"*. What the maintainer asks for:

    - **Installation and Getting Started before Overview on Exudyn** (it was before it);
    - **Exudyn Basics** and **Advanced topics** at the same level as *Overview on Exudyn*;
    - **"Mapping between local and global coordinate indices"** as the **last sub-section of
      "Items: Nodes, Objects, Loads, Markers, Sensors"**, where it belongs;
    - **C++ Code**: a short section in *Advanced topics* that points at the developer
      documentation, with the content moved there and given a name that says what it is - or
      split, if two names fit it better.

<a id="rg3-2"></a>
**RG3.2** **DONE 2026-09-22** (#2585) — [log](exudynRevisionLog2026b.md#rg3-2) — *(group RG3; maintainer 2026-09-22)* **The internal how-to notes leave the published
    documentation** (#2585). `docs/howTo/` holds two kinds of note: what a **user** needs
    (building from source, conda environments) and what only a maintainer needs (ffmpeg,
    matplotlib recipes, Visual Studio 2022, build quirks, what MSVC accepts where gcc does not).
    The second kind stays in the repository and is **mentioned in one line** with a link, rather
    than published.

    **Where the exclusion happens** is `exclude_patterns` in `conf.py` - the same list that keeps
    `docs/revision/*` (the plans, the logs, this file) and `.github/*` out of the build. A file
    that is excluded stays in git and is readable there; it is simply not a page.

<a id="rg3-3"></a>
**RG3.3** **DONE 2026-09-22** (#2586) — [log](exudynRevisionLog2026b.md#rg3-3) — *(group RG3; maintainer question 2026-09-22)* **Is there a PDF, and should there be?**
    (#2586). There is none: decision D8 ended the PDF with the LaTeX sources, because keeping it
    meant keeping a LaTeX toolchain and a second rendering of every page. A PDF **from the
    Markdown** is possible - `sphinx-build -b latex` renders MyST, and the math macros that
    `conf.py` declares to MathJax can generate the LaTeX preamble from the same list that
    `tools/checkMathMacros.py` already checks, which is the part that would otherwise be work.

    **Answered 2026-09-22 (D17): yes.** `exudev docs --pdf`, release only, everything except
    the source listings of the examples and test models — and the **issue history is in**,
    because a reader who searches the document, or feeds it to an AI tool, gets the reason for
    each change with it. 1159 pages. The three defects it uncovered are RG3.6, RG3.7 and RG3.8.

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

<a id="rg3-9"></a>
**RG3.9** **DONE 2026-09-23** (#2598) — [log](exudynRevisionLog2026b.md#rg3-9) —
    *(group RG3; maintainer 2026-09-23)* **Three corrections to the landing pages and the
    developer chapters.** (1) The landing pages say nothing about **how Exudyn is developed**:
    since 1.11.0 that is heavily with Anthropic's Claude Code — code, workflows, documentation,
    tests and examples — and it belongs at the top of `README.rst`, which is the GitHub landing
    page and the first page of the documentation, and of `pdfIndex.md`. (2) **No hand-counted
    numbers in published text**: `pdfIndex.md` counted the examples and the test models and the
    pages they would take, and a number that is not generated is wrong the next day. (3) In the
    PDF the seven **developer documents were chapters beside** *Exudyn developer documentation*
    rather than under it, because both tables of contents listed them as siblings;
    `docs/dev/README.md` carries them now, which nests them in the html sidebar as well.

<a id="rg3-10"></a>
**RG3.10** *(group RG3; maintainer 2026-09-23)* **`CHANGELOG.md` and the issue tracker page hold
    the same list twice** (#2599). Both are written by `issueTracker.py` from the same store,
    both list every resolved issue per release: the changelog is **2384 lines**, the tracker page
    **8987**, and about 2370 lines of the first are a shorter rendering of what the second says
    in full — the tracker page adds the author, the description and both dates, and it carries
    the open issues and the known bugs as well. In the PDF that is roughly 35 duplicated pages.
    It also explains what the maintainer noticed: releases 1.10 and older are one line per issue
    because those issues have a title and no release note, so there is nothing else to print.

    Decision: `CHANGELOG.md` becomes the **current release only** — which is what a reader of
      the GitHub page or of PyPI wants — and the full history lives in the tracker page, which
      the documentation publishes. In the docs (RTD, PDF) the current "Resolved issues and 
      resolved bugs" section becomes "Resolved issues and resolved bugs before version x.y.z" 
      or similar and contains only the earlier changes so there is no duplication - but still
      containing all issues for searching.

<a id="rg3-11"></a>
**RG3.11** **DONE 2026-09-23** (#2611) — [log](exudynRevisionLog2026b.md#rg3-11) — *(group RG3; maintainer 2026-09-23)* **"The C++ core" points at the repository
    instead of at the documentation** (#2611). The section in *Advanced topics* stays — it is
    short and it answers a question users ask — but it was written when the developer
    documentation lived only in the repository, and it still sends the reader to **GitHub URLs**
    of `docs/dev/ARCHITECTURE.md` and `docs/dev/CODING_STYLE.md`. Those are published pages of
    this documentation since RG3.2. It links to the **developer documentation** as a whole and to
    those two pages within it, and it says what the links cannot: that for a deeper understanding
    of the core, and for any low-level change, there is no way around visiting and studying the
    **GitHub project** itself.

## RG4 — Implementation problems and bugs

Problems that are real, reproducible, and too deep to fix in passing. They are recorded here
rather than worked around silently, so the debt stays visible and each item can be closed on
evidence. An ordinary bug goes into the issue tracker and is fixed; a step appears here when the
fix needs a plan of its own.

Open in the tracker for this group: **#2423** (every C++ user error inspects the Python source to
find its file and line, on every raise).

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

<a id="rg4-2"></a>
**RG4.2** *(group RG4; revision2026 step R10.2)* **`ObjectContactConvexRoll.pContact` becomes a data variable** (#2413). The
    computed contact point is stored in the parameter structure and read by the visualization, so
    it is neither system state nor configuration-dependent and keeps no history.

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
    `exu.VisualizationSettings()` are read and written in a test. The original text follows.

    *(group RG4; found in RG6.2.3.1, 2026-09-23)* **Two lines of Python segfault the
    process** (#2603):

    ```python
    import exudyn as exu
    exu.VisualizationSettings().general.drawWorldBasis     #exit code 139
    ```

    It is not that member: **every one of the 93 deprecated members** of `visualizationSettings`
    does it, on read and on write. A deprecated member forwards to its replacement through
    `backlink->view0.scene.drawWorldBasis`, the backlink of every sub-structure is set by
    `VisualizationSettings::Init(&settings)`, and that call happens in **exactly one place** `
    —VisualizationSystemContainer.h:153`, for the settings that belong to a `SystemContainer`.
    A `VisualizationSettings` that Python constructs on its own never gets `Init`, so every
    backlink stays `nullptr` and the first deprecated access dereferences it. Through
    `SC.visualizationSettings` everything works, which is why nobody has met it.

    It needs a plan rather than a one-liner, because a fix has to say what a **copy** of a
    settings structure means: a constructor that calls `Init(this)` is one line, and a copied
    object would then carry a backlink to the original. The same question arrives at
    `simulationSettings` with RG12.1, which gives it deprecated members for the first time.

<a id="rg4-5"></a>
**RG4.5** **DONE 2026-09-23** (#2616) — [log](exudynRevisionLog2026b.md#rg4-5) — *(group RG4;
    maintainer 2026-09-23)* **Quitting the renderer before a simulation starts raised, quitting
    during it did not** (#2616). `CSolverBase::SolveSystem` returned `false` when
    `forceQuitSimulation` was already set, and `SolveDynamic` reads `false` as a failure: the
    *DYNAMIC SOLVER FAILED* block and a `SolverError` traceback, for a user who simply closed the
    render window while the script waited. One step into the same simulation the stop is quiet,
    because `SolveSteps` returns `!conv.stepReductionFailed`. It returns `true` now.

    **What is left open here**, deliberately: `forceQuitSimulation` is set by `GlfwClient.cpp`
    alone and has **no Python binding**, so this path cannot be reached without a window and has
    no test — `mbs.SetRenderEngineStopFlag(True)` sets the *other* flag, `stopSimulation`, which
    `InitializeSolver` clears when a solve starts. A binding, or a test hook, would make the
    difference between "stopped" and "failed" testable at all; it is worth its own step if the
    maintainer wants it.

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
**RG6.1** *(group RG6, before the rendering revision; revision2026 step R11.1)* **Remove OpenVR.** It **blocks the rendering revision**, it
    is not testable in CI or by most users, and it carries a vendored SDK and a prebuilt binary.
    Scope: `src/Graphics/OpenVRinterface.cpp` and its header, every `__EXUDYN_USE_OPENVR`
    guard, the `--openvr` flag and `-lopenvr_api` in `setup.py`, `include/openVR/`, and
    `libs/openvr_api.dll` + `.lib`. Users needing OpenVR take Exudyn <= 1.11; say so in the
    release notes rather than leaving them to discover it. `docs/howTo/openVR.txt` was already
    removed with revision2026 step R3.6.


<a id="rg6-2"></a>
**RG6.2** **DONE 2026-09-23** (#2591), **reopened and closed again the same day for RG6.2.12 to RG6.2.15**,
    which is what the maintainer's first real use of the dialog produced — including a defect
    that only a running renderer shows. The other open work stands in RG6.2.11 (#2608, the
    optional features) and in RG12.3. — *(group RG6; maintainer 2026-09-22)* **The settings dialogs, and the shape of
    `GUI.py`** (#2591). It works, it runs everywhere and it needs no installation - tkinter -
    and that is the reason to keep it. What is wrong with it, in the maintainer's words: the
    table of the visualization settings is restricted; illegal input is caught but there are no
    type hints; the font size cannot be adjusted on Linux; the columns can hardly be adjusted; a
    description should appear in a pop-up rather than only with a special key; fields cannot be
    edited inline; combo boxes are unhandy. The dialogs are called from
    `rendererPythonInterface.cpp`, which executes Python inside C++ - that file belongs to the
    same review.

    Two things changed the ground under it: **revision2026 step R4.10 makes the parameter types
    available to Python**, so a dialog can know what it is editing; and the interface is generic
    enough that a **second front end** (Qt6, or a form that takes Qt5 and Qt6) would be a small
    overhead rather than a second GUI.

    It also shows what RG12.3 produces: the settings that differ from the defaults, as code to
    paste.

    **REVIEWED 2026-09-22.** `python/exudyn/misc/GUI.py`, 1017 lines, two dialog classes:
    `TkinterEditDictionaryWithTypeInfo` (the settings tree) and `TkinterEditDictionary` (a plain
    dictionary, used by right-mouse edit). Each of the seven complaints has a cause in the code,
    and most of them are small:

    | the complaint | what the code does | what it needs |
    |---|---|---|
    | the table is restricted | the tree has three columns, `Name`, `value`, `description`; **type and size are read and stored but never shown** (`self.typeStorage`, `self.sizeStorage`) | a type column, and the unit/range where the definition has one |
    | illegal input is caught, no type hints | `CheckType()` validates on commit and opens a `messagebox.showerror`; the type is known at that moment and is not in the message | show the expected type before the input, in the edit row and in the error |
    | the font cannot be adjusted on Linux | `if not IsApple(): fontFactor = 1` — the font factor is **forced to 1** off macOS and only the row height follows the display scaling; the setting is called `dialogs.fontScalingMacOS` | one `dialogs.fontScaling` for every platform, with `fontScalingMacOS` kept as a deprecated name (the mechanism of RG12.1) |
    | the columns can hardly be adjusted | **`tree.column(...)` is never called** — no width, no minwidth, no stretch, so every column keeps the tkinter default of 200 px and the description is cut | set the widths, let the description take the rest, remember what the user drags |
    | the description needs a key press | bound to the literal key `h`, shown in a modal `messagebox`; the column heading reads *"Description (press H to show)"* | a hover tooltip, and the full text in a wrapped area below the tree |
    | fields cannot be edited inline | the value is edited in a **separate `Entry`/`Combobox` at the bottom of the window**, and the two swap by z-order (`lower()`/`lift()`) | edit in the cell; the bottom row can stay as the place for the long description |
    | combo boxes are unhandy | one `Combobox` reused for every enum, values from `GetComboBoxListsDict()`, which **hard-codes three enum types** | build the list from the type name through `exu`, so that every enum gets a list |

    **The hard-coded three are a real gap, not only a smell**: `OutputVariableType`,
    `LinearSolverType` and `ItemType` are in the dict, and
    `timeIntegration.explicitIntegration.dynamicSolverType` is a `DynamicSolverType` — so it is
    edited as free text, where a typo is a silent wrong value.

    **The finding that changes the step**: the dialog is **not specific to
    `visualizationSettings`**. `GetDictionaryWithTypeInfo()` is generated for 100 structures and
    bound for `SimulationSettings` as well, with name, value, type, size and description for
    every leaf: **470 editable values in `visualizationSettings`, 152 in `simulationSettings`, and
    not one of them without a description**. `EditDictionaryWithTypeInfo(SC.simulationSettings)`
    is a call that nothing offers today. So "a settings dialog for the solver" is not a new
    dialog, and a second front end is a second *renderer* of the same data.

    **`rendererPythonInterface.cpp` is worse than "it executes Python inside C++"**: **220 of its
    775 lines ARE Python**, in six raw string literals. Only the settings dialog is a one-line
    call into `exudyn.misc.GUI`; the **help dialog (69 lines) and the command window (93 lines)
    are written in full inside the C++ file**, where ruff never sees them, the stub check never
    sees them, no test imports them, and one of them carries a leftover `\n";` inside a Python
    comment — which is what code looks like when nothing reads it. Moving those two into
    `exudyn.misc.GUI` beside the third, and leaving one call each in the C++, is the part of this
    step with the clearest boundary.

    **What no test touches**: `allExudynModulesTest.py` imports `GUI.py` because it imports every
    module of the package, and **nothing calls a single function of it**. A dialog needs a window,
    so the suite cannot; what *can* be tested without one is the layer underneath —
    `ConvertString2Value`, `ConvertValue2String`, `CheckType`, `GetComboBoxListsDict` — and that
    is worth doing first, because it is where a wrong value comes from.

    **The order, as sub-steps.** Each one stands on its own and none of them needs the next.

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
    — **The six small complaints.** The column widths (`tree.column(...)` was never called),
    the enum lists built from the module instead of the hard-coded three, the type shown in the
    table and named in the error message, and the description in a tooltip rather than behind the
    key `h`. **`dialogs.fontScaling` is RG6.2.3.1**, because it is the only one that leaves
    Python.

    **The three defects RG6.2.2 found are DONE 2026-09-23** (#2597) —
    [log](exudynRevisionLog2026b.md#rg6-2-3): `CheckType` had no branch for an enum type, so it
    rejected every enum value and only the combo box hid it; `:` was not one of its valid file
    name characters, so no absolute Windows path could be typed into a file name setting —
    including the shipped default `C:/openVRactionsManifest.json`; and a value that passed
    `CheckType` but failed `ConvertString2Value` was dropped with a `print()` to the console, so
    the dialog accepted an edit that never arrived. The enum lists are built from the module with
    them, which is what closed the last of the five.

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
**RG6.2.7** **DONE 2026-09-23** (#2591) — [log](exudynRevisionLog2026b.md#rg6-2-7) — **`GUI.py` is cleaned up, last** *(maintainer, 2026-09-23)*. The module is the one
    that every other sub-step edits, so the tidying belongs at the end, when the shape has
    settled and RG6.2.2 can say whether the tidying broke anything. What is there to do today,
    in 1315 lines: **55 lines of commented-out code**, the dead `#EXAMPLE` dictionary at the end,
    10 bare `print()` calls where a dialog swallows an error and prints it, the ~30 lines of
    font and scaling setup **duplicated** between `EditDictionaryWithTypeInfo` and
    `EditDictionary`, and `treeEditOpenItems— ` module level mutable state that the tree edits
    as a side effect, so which folders are open outlives the dialog and every SystemContainer in
    the process shares it.

<a id="rg6-2-8"></a>
**RG6.2.8** **DONE 2026-09-23** (#2605) — [log](exudynRevisionLog2026b.md#rg6-2-8) — **The bottom row reads like code, and says each thing once**
    *(maintainer, 2026-09-23, after trying RG6.2.4)*. Four small things, all in the row RG6.2.4
    introduced:

    - the pastable line uses the dialog font on the window background; it wants a **smaller,
      fixed font — the one of the cells — and a box or a background of its own**, so that it
      reads as code and is seen as copyable;
    - the label under it **repeats the description** that the pop-up of RG6.2.3 already shows in
      full; it goes;
    - **`copy` does not say what it copies**: it becomes `copy line` (or `copy last edit`), which
      is only worth doing because RG6.2.9 puts a second copy button beside it;
    - the type and size stay — they are what the pop-up does *not* say.

<a id="rg6-2-9"></a>
**RG6.2.9** **DONE 2026-09-23** (#2606) — [log](exudynRevisionLog2026b.md#rg6-2-9) — **A changed value is visible, and every change can be copied at once**
    *(maintainer, 2026-09-23)*. Nothing in the tree marks the rows a user has edited, so after
    ten edits in four folders the ten cannot be found again. A changed row is shown **boldface or
    in a colour** (a blue dark enough to read on the row background; a tkinter `Treeview` tag
    carries both), and a **second button** at the bottom copies **all** changes, as the lines that
    set them. That button is RG12.3 for one dialog session, and the two share one question that
    this step answers: *changed against what* — against the values the dialog opened with, or
    against the defaults. The first is what a user means while editing; the second is what makes
    a script reproduce the settings.

<a id="rg6-2-10"></a>
**RG6.2.10** **DONE 2026-09-23** (#2607) — [log](exudynRevisionLog2026b.md#rg6-2-10) — **Find a setting** *(maintainer, 2026-09-23)*. Several hundred values in a
    tree of folders, and the only route to one is knowing its folder. **CTRL-F and a find
    button**, matching **names first and descriptions second**, then jumping to the row: expand
    its folders, select it, scroll it into view. The form is decided in this step; the
    recommendation is **an entry plus a drop-down of the hits**, because it is the one shape that
    serves both ways of searching: as the user types, the drop-down lists the matches as their
    dotted paths (`general.textSize`), names before descriptions, description hits with a snippet
    of what matched; **Return** jumps to the first, **Return again** or **F3** to the next, and
    picking one from the drop-down jumps straight to it. Rejected alternatives, recorded so they
    are not re-proposed: *filtering the tree* to the hits (loses where a setting sits, and the
    tree is the map), and a *separate result window* (a third place to look, in a dialog that
    already has three).

<a id="rg6-2-11"></a>
**RG6.2.11** **DONE 2026-09-23** (#2608) — [log](exudynRevisionLog2026b.md#rg6-2-11) —
    **The catalogue of optional features, decided.** It was a list to pick from, and the
    maintainer went through it on 2026-09-23. Nothing of it is left open except one low priority
    step, which is why the catalogue closes:

    - **done in the meantime**: the **reset** and the **undo** (RG6.2.14), and the *"changed
      only" view*, which the two windows of RG6.2.9 are;
    - **no**: *load and save* the settings to a file — the code of RG6.2.9 is what a user keeps,
      and it goes into the script rather than into a second format nobody reads;
    - **no**: *units in the description* — even a position has no unit that Exudyn could name:
      the model's units are the user's implicit choice, so a unit in a description would be a
      guess printed as a fact;
    - **no** (already the behaviour): *apply while it is open* — every change is applied
      immediately and that stays;
    - **open, low priority: the same dialog for `simulationSettings`** — RG6.2.18;
    - **undecided: remember the window.** How it would work, and why it is not simply done —
      the geometry is one string (`tkWindow.geometry()` gives `WxH+X+Y`), and the three places it
      could live are a module variable in `exudyn.misc.GUI` (this process only, the shape of
      `treeEditLastOpenItems` from RG6.2.7), a member of `visualizationSettings.dialogs` (travels
      with the model and can be saved by a user's own script), or a file next to the user's
      configuration. **The danger is the position, not the size**: a window remembered on a second
      screen that is no longer attached opens where nobody can see it, and the same happens with a
      changed resolution or a docked laptop. What makes it safe is to restore the **size always**
      and the **position only when it still lies inside a screen** —
      `winfo_vrootwidth/height` give the whole virtual desktop, and a rectangle that is not fully
      inside it is dropped back to the default position. If the maintainer wants it, it becomes a
      step of its own with that rule written into it.

<a id="rg6-2-12"></a>
**RG6.2.12** **DONE 2026-09-23** (#2612) — [log](exudynRevisionLog2026b.md#rg6-2-12) — **59 untouched settings are called changed, and a folded folder hides a change**
    (#2612) *(maintainer, 2026-09-23, from demo 2)*. Two halves of one thing:

    - **the reference is wrong.** RG6.2.9 compares against `exu.VisualizationSettings()`, and a
      `SystemContainer` initialises **59** of those settings when it is created: the four lights
      and the ten raytracer materials, which are synced with the renderer. A user who changed
      nothing sees every light and every material reported as changed, which is exactly what the
      maintainer saw. The reference has to be the state a user **starts from**,
      `exu.SystemContainer().visualizationSettings— ` and this is a case that no test without a
      `SystemContainer` could have caught, which is the lesson worth keeping.
    - **a folder says nothing about its subtree.** With the tree folded, a changed value is
      invisible; a folder whose subtree holds a changed value is marked as well.

<a id="rg6-2-13"></a>
**RG6.2.13** **DONE 2026-09-23** (#2613) — [log](exudynRevisionLog2026b.md#rg6-2-13) — **The find bar needs no button, and says nothing when it is idle** (#2613)
    *(maintainer, 2026-09-23)*. The search runs while the text is typed, so the **find** button is
    removed; and the drop-down of the hits looks like something to click before anything has been
    searched for, so it is **greyed out** until there is a search text.

<a id="rg6-2-14"></a>
**RG6.2.14** **DONE 2026-09-23** (#2614) — [log](exudynRevisionLog2026b.md#rg6-2-14) — **Reset, revert, undo, close — and the windows stay in front** (#2614)
    *(maintainer, 2026-09-23)*. The bottom of the dialog gets a **second row**: *diff to default*
    and *this session* on the left, and on the right **reset** (to the defaults), **revert** (to
    the state the dialog opened with), **undo** (the last change, one step, greyed when there is
    nothing to undo — which is the undo of RG6.2.11) and **close** (what ESCAPE does). Every
    button says what it does in a **tooltip**, and the tooltips of the tree open after **0.5
    seconds**: they are in the way while the mouse crosses the tree, and since RG6.2.10 nobody has
    to sweep through the settings to find one. And the window showing the changes appeared
    **behind** the dialog the second time it was opened, because the dialog is topmost — which
    it must stay, since it blocks the render window.

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
    chasing RG6.2.16, and the cause of it: **constructing an `exudyn.SystemContainer()` replaces
    `exudyn.sys['currentRendererSystemContainer']`**, and RG6.2.12 creates one to read the
    defaults. From the moment the settings dialog opened, everything that asks for the renderer's
    container got the throw-away one — the window settings were read from it, and
    `UpdateSettingsStructure` sent the **redraw signal** to it instead of to the renderer. The
    entry is saved and restored around the construction, and `GetRendererSystemContainer` also
    catches the `RuntimeError` of a container that is gone.

<a id="rg6-2-18"></a>
**RG6.2.18** *(group RG6; maintainer 2026-09-23, from RG6.2.11)* **The same dialog for
    `simulationSettings`** (#2624), **low priority**. Everything below the widgets is ready:
    `GetDictionaryWithTypeInfo()` is bound for `SimulationSettings` as well, `SettingsPrefix`
    already writes `simulationSettings....` into the code line, and `DefaultSettingsDictionary`
    falls back to the constructor for a structure that is not on a `SystemContainer`. What is
    missing is a **way to open it** — a function in `exudyn.misc.GUI`, and the question whether
    the renderer should offer a key for it while a solver is running, where changing a solver
    setting mid-step is not the harmless thing that changing a colour is.

<a id="rg6-2-19"></a>
**RG6.2.19** **DONE 2026-09-23** (#2625) — [log](exudynRevisionLog2026b.md#rg6-2-19) —
    **Opening the settings dialog closed the render window** *(maintainer, 2026-09-23)*.
    `MainSystemContainer()` calls `AttachToRenderEngineInternal()` in its **constructor** and
    `Reset()— ` which calls `DetachFromRenderEngine— ` in its **destructor**. The temporary
    container that RG6.2.12 created to read the defaults therefore took the render window away
    from the container that owns it and handed it back to nothing. The dialog creates no
    container any more; the reference is the structure's own constructor again, and the window
    that lists the differences **says** which settings a `SystemContainer` initialises, instead
    of pretending they were changed. The dialog also gives up its `-topmost` while that window is
    open, which is the maintainer's own suggestion for the window that kept coming up behind it.

<a id="rg6-2-20"></a>
**RG6.2.20** **DONE 2026-09-23** (#2626) — [log](exudynRevisionLog2026b.md#rg6-2-20) —
    **The defaults of the lights and the raytracer materials are hidden in C++ constructors.**
    They are defaults of the **structure** now: `StructureParameter` gained `memberDefaults`, the
    89 values are in `definitions/structureDefsVisualizationSettings.py`, the generated
    constructors carry them, the C++ that set them afterwards is gone, and the reference says
    what `material1` and `light2` start from. Every one of the 89 was compared against the C++ it
    replaces and is identical. The original text of the step follows.

    *(group RG6; from RG6.2.19, 2026-09-23)* **The defaults of the lights and the
    raytracer materials are hidden in C++ constructors** (#2626). `exu.VisualizationSettings()`
    is not the state a user starts from: `VisualizationSystemContainer()` overrides nine light
    settings in its constructor (`light1`-`light3` diffuse, specular, enable and
    `light1.position`) and `MainGraphicsMaterialList::Reset()` fills the ten raytracer materials
    — about 59 values. That is the only reason the settings dialog cannot say what really
    differs from the defaults, and why it now carries a note instead. The values belong in
    `definitions/structureDefsVisualizationSettings.py` as the `defaultValue` of those members, so
    that the **generated** structure carries them and the constructor is the truth; the C++ lines
    then go. Nothing changes for a user: the container sets the same values today. The test of
    **Sharpened by the maintainer on 2026-09-23**, after testing *diff to default* in 1.12.34:
    the dialog compares against the defaults of the **struct type**, `VSettingsMaterial` and
    `VSettingsLight`, and a user never sees those. What the renderer uses are the defaults of
    `material0`, `material1`, ... and of `light1` to `light3`, set later in C++ and different in
    `diffuse`, `specular` and `enable`. So the comparison is not merely noisy: it is made against
    **a state that never exists**. The fix must therefore be the solid one and not a filter —
    the per-instance defaults become **the** defaults, so that the generated reference can state
    them too, which today it cannot: it prints the struct default while the renderer uses another
    value.

    RG6.2.19 says when this is done — it requires the difference to be **non-empty** and names
    the paths, so it fails the day the last one moves.

<a id="rg6-2-21"></a>
**RG6.2.21** **DONE 2026-09-23** (#2627) — [log](exudynRevisionLog2026b.md#rg6-2-21) —
    **The dialog stopped asking, and undo goes back one whole state** *(maintainer,
    2026-09-23)*. Two findings from using the second button row. **reset** and **revert** each
    asked a yes/no question that was never asked for: *"we can always revert to initial settings
    and there is undo, so no worries about one wrong button click"*. Both questions are gone.
    And **undo did not take a reset or a revert back** — it went back one value and was
    switched **off** by exactly the two clicks one would want to undo. The maintainer's own
    proposal is the fix: undo goes back to the **previous state**. The dialog keeps a stack of
    whole states — about 470 short strings per entry, nothing next to the redraw it triggers —
    pushed by a single edited value, by reset and by revert alike, so a chain of them comes back
    one by one.

<a id="rg6-3"></a>
**RG6.3** *(group RG6; maintainer 2026-09-22)* **The renderer extraction functions are not shaped
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
    setting names in the hand-written manual went with them. The original text follows.

    *(group RG6; maintainer 2026-09-23)* **The light and shadow descriptions say things
    that are no longer true** (#2609). The descriptions in
    `definitions/structureDefsVisualizationSettings.py` are what a user reads in the dialog, in
    the reference manual and in an editor tooltip, so they are the documentation of the lights.
    Three faults, all from the maintainer:

    - every member of a light repeats **"of GL_LIGHT0"** (`1`, `2`, `3`). Inside `light0` that is
      noise: it reads **"of this light"**, and the mapping — `light0` to `light3` are OpenGL's
      `GL_LIGHT0` to `GL_LIGHT3— ` is said **once**, at the `enable` flag of the light.
    - the remarks single out **light0 as the light that casts shadows**. That was a performance
      decision and it no longer holds: **every light can cast shadows**. Every such sentence is
      checked and adjusted.
    - *"approximates directional lights by enlarging the direction to 200 times maxSceneSize"*
      describes **shadows**, not a light, so it belongs to the shadow settings — and it must not
      name the factor, which has changed once already and will change again. Generated
      documentation that quotes a number no generator produced is wrong the day the number
      changes.

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

*No steps yet.*


<a id="rg9-1"></a>
**RG9.1** **DONE 2026-09-23** (#2622) — [log](exudynRevisionLog2026b.md#rg9-1) —
    **The item sources stop paying for pybind11.** **52 of 52** sources in `src/ImplObjects/`
    reached pybind11 before, **19** after — and those 19 for a reason of their own, a user
    function, a `PyMatrixContainer`, a numpy array or `ExceptionsTemplates.h`, not through the
    graphics headers. **The build time did not change**: 57.1 s before, 58.0 s after, on a clean
    build of the same machine, so the gain is in the structure and not in the clock. The original
    text of the step follows.

    *(group RG9; proposed 2026-09-23 at the maintainer's request, after RG6.2 and the item
    split of revision2026 step R11.4.4)* **The item sources stop paying for pybind11** (#2622).
    `src/Graphics/VisualizationItemHelpers.h` is included by the `C<Item>.cpp` files that draw
    something — and it includes `Graphics/VisualizationSystemContainer.h`, which includes
    pybind11. That is the dependency the split into per-item graphics functions was meant to
    avoid. **Measured on 2026-09-23**, not assumed:

    - the `py::` in `VisualizationSystemContainer.h` is **six free-function declarations**
      (`PyWriteBodyGraphicsDataList`, `PyGetBodyGraphicsDataList*`, ...), no class member and no
      template, and exactly **two** `.cpp` files call them. Moving them to a header of their own
      is small and carries no risk.
    - that alone changes nothing, because the header also has
      `#include "Main/CSystem.h"— ` the maintainer's own *"REMOVE: temporary"* line — and
      `CSystem.h` includes `Pymodules/PythonUserFunctions.h`, which includes pybind11.
    - **the experiment**: with that include commented out, the build fails **only** in
      `Graphics/VisualizationSystem.h:33-34`, which declares two members whose headers it never
      includes — `PostProcessData*` and `CSystemData*`. Both of those headers are **pybind-free**.

    So the order is: `VisualizationSystem.h` includes `Graphics/PostProcessData.h` and
    `Main/CSystemData.h` itself (it uses them as members and today free-rides on the container's
    include), `VisualizationSystemContainer.h` drops `Main/CSystem.h`, and the six declarations
    move to `Graphics/BodyGraphicsDataPython.h`. Then the item sources see no pybind11.

    **Acceptance is a measurement, not an opinion**: the build time before and after, which
    RG10.3 now prints — **57.9 s** for `exudyn build` on the maintainer's machine, 2026-09-23.
    `VisualizationSystem.h` is included widely enough that this deserves its own step rather than
    a drive-by edit.

## RG10 — Tooling and process

The machinery a maintainer uses: `exudev` (revision2026 step R5.18), the issue tracker and its
JSON store (revision2026 steps R8.3 to R8.5), the generators (revision2026 step R4.3), the checks of the commit gate, and the CI. It
works; this group carries what it still lacks.

Open in the tracker for this group: **#2541** (`exudyn.config` and `exudyn.special` are in no stub
file, so an editor cannot complete them).

<a id="rg10-1"></a>
**RG10.1** *(group RG10; maintainer request 2026-09-15; revision2026 step R8.6)* **Checker for user scripts
    after the 1.12 API changes.** Teaching folders and user projects hold Exudyn scripts written
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


<a id="rg10-2"></a>
**RG10.2** **DONE 2026-09-23** (#2600) — [log](exudynRevisionLog2026b.md#rg10-2) — *(group RG10; maintainer 2026-09-23)* **The issue table of `exudev issue serve` does
    not say what its columns are, and leaves out the priority** (#2600). Three things, all in
    `tools/issueTracker/issueServer.py`:

    - **no header row.** The list is `<table><tbody id="list">` and nothing else: the rows carry
      number, status, type, effort and title as bare tags, and a reader has to guess which tag is
      which. It needs a header — and the effort values need their meaning within reach, because
      `LOW` means *within 2 hours* and `HUGE` means *above 40 hours*, which no tag says.
    - **the priority is not shown**, although `/api/issues` already sends it in every row and the
      filter bar already filters on it. Only the row renderer leaves it out.
    - **effort and priority share their spelling**: `LOW` and `HIGH` are values of both, so two
      bare tags in one row cannot be told apart. The rule the maintainer gives: the **effort** tag
      carries the word — `LOW EFF`, `MEDIUM EFF`, `HIGH EFF`, `HUGE EFF` — and the **priority**
      tag stays plain — `LOW`, `NORMAL`, `HIGH`.

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

## RG11 — Misc

What belongs to no group yet. Three of a kind here are a reason to propose a group of their own.

<a id="rg11-1"></a>
**RG11.1** *(group RG11; maintainer 2026-09-23)* **The results monitor runs beside the
    simulation, or it is redundant** (#2610). `exudyn.misc.resultsMonitor` was a **command line**
    tool: a second terminal watched a solution file grow while the simulation wrote it, which is
    the whole point of a monitor. The documented in-script form

    ```python
    from exudyn.misc.resultsMonitor import MonitorResults
    MonitorResults('solution/genetic.txt', logY=True, updatePeriod=0.5)
    ```

    only earns its place if it does **not** block: if it returns when the window closes, it plots
    a finished file and `PlotSensor` already does that — a second way to do one thing, against
    rule 10. This step evaluates how it could run **beside** the simulation and recommends one
    way; the candidates and what each costs:

    - a **second thread** — cheapest to write, and matplotlib is not thread safe: the plot has
      to live on the main thread or in a backend that tolerates it;
    - a **second process** — no shared state, works with any backend, needs the file as the
      protocol (which it already is) and a way to end it with the script;
    - the **renderer's own loop** — there is already a GUI thread and a periodic callback, but
      it ties the monitor to a running renderer;
    - **drop the in-script call** and document the command line form, which is the honest outcome
      if none of the above is worth its complexity.

<a id="rg11-2"></a>
**RG11.2** **DONE 2026-09-23** (#2620) — [log](exudynRevisionLog2026b.md#rg11-2) — *(group
    RG11; maintainer 2026-09-23)* **The demos wrote a `solution/` directory into whatever
    directory they were started in.** `Demo1` and `Demo2` named `solution/demo1.txt` and
    `solution/chain.txt`, so `python -m exudyn demo 2` created a directory beside the sources of
    this repository — untracked, unignored, and nearly committed by accident. They write to
    `tmp/solution/` now, which is created if it is missing, and `solution/` is in `.gitignore` so
    that an older installed version cannot leave one lying around unnoticed.

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
**RG12.3** *(group RG12; maintainer 2026-09-22)* **What did this model actually change?** (#2590)
    Both settings structures have `GetDictionary()`, and a default instance is one call away
    (`exudyn.SimulationSettings()`, `exudyn.VisualizationSettings()`), but nothing subtracts the
    two. A helper in the utilities should print the difference **as Python code**, so that it can
    be pasted into a script and reproduces the settings - which is what makes a session in the
    visualization dialog reusable, and what RG6.2 should show for the current settings. Worth
    considering as additional information in solution and sensor files, where it would make a
    result reproducible.

    Open in the tracker for this group: **#2497** (59 bare `except:` remain in the shipped
    package).
