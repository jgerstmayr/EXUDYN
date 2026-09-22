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
| **RG3** Docs | what the documentation still gets wrong or does not say | 4 |
| **RG4** Implementation problems and bugs | real, reproducible problems that need a plan rather than a fix | 3 |
| **RG5** Performance | measurement first, then the code that is actually hot | 2 |
| **RG6** Graphics and rendering | the renderer, the settings dialogs, and the rendering revision it is heading for | 3 |
| **RG7** Python user items | items whose behaviour is written in Python | - |
| **RG8** Compiled C++ user items | plugins: user items compiled against the shipped headers | 9 |
| **RG9** Structural core improvements | the architecture of the core, where a change touches everything | - |
| **RG10** Tooling and process | exudev, the issue tracker, the generators, CI | 1 |
| **RG11** Misc | what has no group yet; three of a kind become a group |  - |
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
**RG3.3** *(group RG3; maintainer question 2026-09-22)* **Is there a PDF, and should there be?**
    (#2586). There is none: decision D8 ended the PDF with the LaTeX sources, because keeping it
    meant keeping a LaTeX toolchain and a second rendering of every page. A PDF **from the
    Markdown** is possible - `sphinx-build -b latex` renders MyST, and the math macros that
    `conf.py` declares to MathJax can generate the LaTeX preamble from the same list that
    `tools/checkMathMacros.py` already checks, which is the part that would otherwise be work.

    What this step decides: whether the PDF is wanted at all and for whom; and if it is, it is a
    **release-only build** (`exudev docs --pdf`), never part of the documentation gate, so that a
    missing LaTeX installation cannot stop an ordinary docs build.

<a id="rg3-4"></a>
**RG3.4** *(group RG3; maintainer 2026-09-22)* **The revisions chapter says where the details
    are** (#2587). It is deliberately short, and it should end by pointing at the developer
    documentation: the revision is recorded in full in a plan and a log, and there are **two**
    of them now - revision2026, finished and completed as 1.12, and revision2026b, continuing.

    Open in the tracker for this group besides these: **#2550** (citations such as
    `[ZwoelferGerstmayr2021]` are printed but resolve to nothing - the documentation needs a
    references page).

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
**RG6.2** *(group RG6; maintainer 2026-09-22)* **The settings dialogs, and the shape of
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
    overhead rather than a second GUI. The step is refined after a look at the current state.

    It also shows what RG12.3 produces: the settings that differ from the defaults, as code to
    paste.

<a id="rg6-3"></a>
**RG6.3** *(group RG6; maintainer 2026-09-22)* **The renderer extraction functions are not shaped
    for testing** (#2583). `RedrawAndGetImage()` and `GetRenderState()` exist and are what a
    graphics test has to build on, but they were written for interactive use: the image comes
    back at full resolution, nothing returns a **summary** of the graphics data without
    rendering, and the raytracer path and the GLFW path differ in what they update. RG2.3 needs
    a documented headless call that updates the graphics data and returns counts, and an image
    call that takes a resolution.

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


## RG11 — Misc

What belongs to no group yet. Three of a kind here are a reason to propose a group of their own.

*No steps yet.*

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
