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
**RG3.13.1** *(group RG3; from RG3.13, 2026-09-24)* **235 references to the plan are left in
    comments, each inside a sentence** (#2649). 652 of the 887 were parentheticals or appended
    clauses and went by rule, keeping the issue number where there was one. The rest read like
    *"step R4.3 is moving outputs from the old generators to separate emitters"* - a sentence has
    to be written for each, which a pattern cannot do. **None is in a published page**: they are
    comments in `src/`, `tools/` and `python/`, so this is tidiness rather than a defect, and it
    is work for a session with nothing better to do.

<a id="rg3-14"></a>
**RG3.14** *(group RG3; maintainer 2026-09-25)* **The item and settings descriptions are written
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
    - **RG3.14.7** - **the tail and the gate.** `\texttt{x}` is `` `x` ``, `\bf` is `**`,
      `\bi/\item/\ei` is a Markdown list - all in `latexToMarkdown.ConvertInline` and
      `ConvertLists`; the 32 macros that occur at most twice are rewritten one by one. Then
      `tools/generators/generate.py` gains the check that closes the step: **a backslash command in
      a `definitions/` description, outside math, is an error with the file, the line and the
      name** - `latexToMarkdown.ReportUnknown` already finds them and only prints, and
      `tools/checkMathMacros.py` already holds the line between a math macro and a structural one.
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

    - **RG3.14.12** *(from #2656; maintainer 2026-09-25)* **the `theDoc.pdf` references are resolved
      against real sections.** Five hand-written docstrings in the shipped package still send the
      reader to a document that has not existed since decision D8. The maintainer searched the built
      documentation: *"all of them can be clearly related to a section, but this has to be done with
      the context of each paragraph or section"* - so this is one judgement per reference, not a
      pattern. `MainSolverStatic`, for instance, **is** a section of the structures chapter and only
      needs a label to be referenceable. Two of the five also carry `[Section](#sec:solverSubstructures)`,
      whose target is spelled the LaTeX way and resolves to nothing.

      And the name itself gets one honest mention: the **revisions chapter** says that the
      documentation was a single PDF, `theDoc.pdf`, until this revision - a reader who meets the
      name in an old issue has to be able to find out what it was - and points at how the
      documentation is built now. That is the only place it is said (rule 6a).
    - **RG3.14.13** *(from RG3.14; maintainer 2026-09-25)* **what is left of the LaTeX conversion,
      and what of it goes.** `tools/generators/autoGenerateHelper.py` carries the old
      LaTeX-to-RST machinery - a conversion dict of some 90 entries, `ReplaceWords`, `Str2Latex`,
      `Latex2RSTlabel`, `GetTypesStringLatex`, the `sLatex`/`sRST` members of `PyLatexRST` and the
      table helpers that exist to lay out a LaTeX table - in an unsystematic state, because each
      revision step took what it needed and left the rest. Once RG3.14 has emptied `definitions/`
      of structural LaTeX, most of it has no caller. The step is a **census with a verdict per
      entry**: what is still reached, by whom, and what is deleted. The rule is the one that makes
      it safe - the regeneration must stay byte-identical - and the outcome is expected to be that
      the shapes worth keeping (the Markdown table helpers, `ImagePath`, the heading and label
      helpers) stay and the LaTeX branch goes.

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

<a id="rg6-2-27"></a>
**RG6.2.27** **DONE 2026-09-25** (#2654) — [log](exudynRevisionLog2026b.md#rg6-2-27) —
    **The command window lost the scope of the model** *(maintainer, 2026-09-25)*. Pressing X in
    the render window opens a window whose label promises the *"global scope of you Python
    model"*, and `mbs` was a `NameError` there: the dialog became a function of
    `exudyn.misc.GUI` in RG6.2.1, and `globals()` inside a module function is the module. It
    runs in `vars(__main__)` again - one dictionary, so an assignment also survives the command.

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


<a id="rg10-10"></a>
**RG10.10** **DONE 2026-09-25** (#2653) — [log](exudynRevisionLog2026b.md#rg10-10) —
    **The header of `definitionLoader.py` read like the file was dead** *(maintainer question,
    2026-09-25)*. It is live - three generators stop working without it - and what made it look
    dead was a header that opened with what the old parser produced and ended with two promises
    about its own removal. It says what it is and who uses it. `itemModel.LegacyItems()`, found
    while checking, really was dead and is gone.

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


<a id="rg10-7"></a>
**RG10.7** **DONE 2026-09-24** (#2638) — [log](exudynRevisionLog2026b.md#rg10-7) —
    **The plan carried the full text of the steps that are finished** *(maintainer,
    2026-09-24)*: 971 of its 1391 lines, against its own rule that a done step keeps status,
    date, outcome and a link. A step that ended in *"the original text follows"* is cut there -
    that text is the issue as it was raised, and the tracker has it - and the rest were
    rewritten to the outcome. 1391 lines to 939, with no anchor, step number or group heading
    lost. The review of `GUI.py` was the one piece of analysis that lived only here and is now
    a [log entry](exudynRevisionLog2026b.md#rg6-2-review).

## RG11 — Misc

What belongs to no group yet. Three of a kind here are a reason to propose a group of their own.

<a id="rg10-7-1"></a>
**RG10.7.1** **DONE 2026-09-24** (#2642) — [log](exudynRevisionLog2026b.md#rg10-7-1) —
    **The plan did not say what to do next** *(maintainer, 2026-09-24)*. The open steps are
    spread over twelve groups and were read by scrolling. The last section, **Next steps
    recommended**, names them with their issue and a short title, lists what the current work
    raised without making it a step, and recommends an order with the reason for it. It copies
    nothing and is updated from time to time.

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
**RG11.3** *(group RG11; proposed 2026-09-24 by RG11.1, not started)* **The results monitor beside
    a running simulation.** RG11.1 evaluated the four ways and recommends a **second process**:
    the solution file is already the protocol, `python -m exudyn monitor` already exists, nothing
    is shared so no backend, GIL or thread-safety question arises, and it is a handful of lines
    around `subprocess.Popen([sys.executable, '-m', 'exudyn', 'monitor', fileName, ...])` that
    returns the handle. `MonitorResults` stays as it is for the case where blocking is wanted.
    The open questions are the **lifetime** — whether the child is killed when the script ends
    or left for the user to close — and whether the same call should serve
    `SolutionViewer`.

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
| RG2.3 | #2582 | a graphics regression suite |
| RG3.8 | #2594 | place or drop the figures that no page references |
| RG3.13.1 | #2649 | write a sentence for the 235 plan references that are left in comments |
| RG3.14 | #2655 | the item descriptions become Markdown: `definitions/` is free of structural LaTeX; .7 closes the gate |
| RG3.14.12 | #2656 | the theDoc.pdf references resolved against real sections, and the name explained once |
| RG4.1 | - | resolve the Windows/linux differences in contact and friction |
| RG4.2 | #2413 | `ObjectContactConvexRoll.pContact` becomes a data variable |
| RG4.3 | #2398, #2400 | bring down the cost of an explicit integration step |
| RG5.1 | #2397 | build a micro-benchmark that is maintained, not written once |
| RG5.2 | - | make the hot linear algebra vectorizable |
| RG6.3 | #2583 | give the renderer a headless call that returns counts and an image at a given resolution |
| RG8.1 to RG8.9 | - | the plugin ABI: registry, fingerprint, reference plugin, headers, discovery |
| RG10.1 | - | a checker for user scripts after the 1.12 API changes |
| RG11.3 | - | run the results monitor in a second process beside the simulation |
| RG12.1 | #2588 | `simulationSettings` gets the deprecation mechanism |
| RG12.2 | #2589 | let an item parameter be deprecated and renamed |

### Raised by the current work, and not yet a step

Each of these is written down where it was found; none is planned, and the maintainer decides
whether it becomes a step.

| where | issue | what it is |
|---|---|---|
| maintainer, 2026-09-24 | - | the documentation of the steps that are done, where a page still describes the state before one of them |
| RG6.2.11 | #2608 | **remember the window** - undecided; the rule that makes it safe is in the log |
| RG4.5 | #2616 | a binding or a test hook for `forceQuitSimulation`, which no test can reach today |
| RG4 | #2423 | every C++ user error inspects the Python source to find its file and line |
| RG10 | #2541 | `exudyn.config` and `exudyn.special` are in no stub file |
| RG12 | #2497 | 59 bare `except:` remain in the shipped package |
| maintainer, 2026-09-25 | #2659 | the simulation settings section does not mention `python -m exudyn dialogs sim` |
| maintainer, 2026-09-25 | #2660 | six pages of the C++ interface repeat their own title as the first section |

### The chapters of the user manual, as decided for #2657 and #2662

*The maintainer's table of contents, 2026-09-25, answering the proposal above it. Measured the same
day: **eleven** visualization sections sit in `introductionBasics.md`, five in `GUI.md` and one in
`introductionAdvanced.md`.*

```
Exudyn
Installation and getting started
Overview on Exudyn
Exudyn basics
Renderer, graphics and visualization
    The renderer window
    The model view
    Images, animations and the solution viewer
    How to add graphics
Performance, errors and solver failures
    Errors: what Exudyn raises, and what to do about it
    Removing convergence problems and solver failures
    Performance and ways to speed up computations
Advanced topics
    ...
Notation
Theory
Solver
Python-C++ command interface
```

**Renderer, graphics and visualization** is `docs/manual/GUI.md`, in four sections built from what is
spread over three chapters today:

| section | gathers |
|---|---|
| **The renderer window** | starting and stopping it, mouse input (with the 6D mouse), keyboard input, the visualization settings dialog, the command and help windows - and a link to *Advanced topics* for the raytracer, which draws offline |
| **The model view** | the render state, storing and restoring a view, a camera that follows an object (today in *Advanced topics*) |
| **Images, animations and the solution viewer** | the solution viewer, saving images, software rendering, making an animation |
| **How to add graphics** | graphics user functions, colour, RGBA and transparency, character encoding - and a link to *Advanced topics* for the `GraphicsData` reference |

**Performance, errors and solver failures** is a chapter of its own, out of *Exudyn basics*: the
three sections are already written and already belong together, and none of them is a basic.

**`introductionBasics.md`** keeps one visualization section, *Seeing the model*: the lines that start
the renderer, and a link.

**`introductionAdvanced.md`** takes what is internals or reference rather than use - the **graphics
pipeline**, **raytracing** and the **`GraphicsData` reference** with its six sub-sections - and, with
#2661, *the command line* and *the results monitor*.

**Every heading is sentence case** (#2662): *"This is a heading"*. About fifteen are Title Case
today, and two of them are chapter titles the table of contents above already spells the new way -
*Installation and getting started*, *Exudyn basics*.

### Recommended next

The title of each says what the step **does**; the sentence after it says why it comes here.

1. **Run the integration round of the institute, then release 1.13** (RG2.2, RG1.4). It is
   the only item on this page that needs **other people's time**, so it starts before the
   rest is ready, not after.
2. **Give `simulationSettings` the deprecation mechanism** (RG12.1, #2588). It is the one
   `visualizationSettings` already has, and RG12.2 (#2589) cannot start until both have it.
3. **Build the graphics regression suite, with the headless renderer calls it needs**
   (RG2.3 with RG6.3, #2582 and #2583). They are one piece of work: the suite needs the call
   that updates the graphics data and returns counts, and that call has no other user.
4. **Place or drop the figures that no page references** (RG3.8, #2594). Small, and it is
   published documentation that is visibly wrong.
5. **Write the checker for user scripts after the 1.12 API changes** (RG10.1). The 1.13
   release is when users meet those changes, so it is worth having before RG1.4 lands.

