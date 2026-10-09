# Exudyn Revision Plan 2026b

**Closed on 2026-10-10.** Its open steps continue in
[`exudynRevisionPlan2026c.md`](exudynRevisionPlan2026c.md), most of them under the same number; each line
below says where. This plan keeps one line per step - status and title -; the full text of every step as it
stood at the closing is in the log, [The texts of plan 2026b at its closing](exudynRevisionLog2026b.md#plan-at-closing),
and what was done in each step in the log entries the texts link to.

Cite a step of this plan as "revision2026b step RG8.3.1", and a step of the first plan as "revision2026 step R4.3".

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
| **RG18** Resolving old issues | old issues of the tracker that are quick to resolve, or important and fit no other group |


## RG1 — Release and publication

- <a id="rg1-1"></a>**RG1.1** DONE 2026-10-09 — At the 1.13 release: fast-forward `master`, push once with tags. · [text](exudynRevisionLog2026b.md#closing-rg1-1)
- <a id="rg1-2"></a>**RG1.2** MOVED to revision2026c step RG1.2 — Retroactively tag past releases where the commits can be identified · [text](exudynRevisionLog2026b.md#closing-rg1-2)
- <a id="rg1-3"></a>**RG1.3** MOVED to revision2026c step RG1.3 — Second internal GitLab repository for development-only Python. · [text](exudynRevisionLog2026b.md#closing-rg1-3)
- <a id="rg1-4"></a>**RG1.4** DONE 2026-10-09 (1.13.6) — The 1.13 release. · [text](exudynRevisionLog2026b.md#closing-rg1-4)

## RG2 — Testing and verification

- <a id="rg2-1"></a>**RG2.1** DONE 2026-10-05 — The drawing code is exercised by exactly one test model (#2562) · [text](exudynRevisionLog2026b.md#closing-rg2-1)
- <a id="rg2-2"></a>**RG2.2** CLOSED 2026-10-08 — The integration round before 1.13. · [text](exudynRevisionLog2026b.md#closing-rg2-2)
- <a id="rg2-4"></a>**RG2.4** DONE 2026-10-09 (#2748) — A manual check of the render window and the dialogs before a release (#2748) · [text](exudynRevisionLog2026b.md#closing-rg2-4)
  - **RG2.4.1** DONE 2026-10-06 — what could be fixed without a screen: CTRL+keypad, F2 message, highlight time, sensor traces in the raytracer, `general.sphereTiling` for `graphics.Sphere` and the sph...
  - **RG2.4.2** DONE 2026-10-06 — confirmed on the Mac 2026-10-07 — (#2878) macOS: abort when a Tk dialog (SolutionViewer) is closed - glfwPollEvents dispatches the Tk event inside a Tk callback
  - **RG2.4.3** DONE 2026-10-06 — shadows of the GLSpheres in the OpenGL renderer, as a disc normal to the light (maintainer). (#2879)
  - **RG2.4.4** DONE 2026-10-06 — larger sizes and `openGL.lineWidth = 3` in the check model, the simplified-style switches as check V6a (#2880)
  - **RG2.4.5** DONE 2026-10-07 — Ubuntu: curved triangle tiling (K13) and recorded frames (S6) to be checked again (#2881)
    - **RG2.4.5.1** DONE 2026-10-07 — the cause of the missing frames, found by the GUI check agent on Ubuntu: `saveImageAsData` of a view was never initialized, and a nonzero value kept every recorded fra...
  - **RG2.4.6** DONE 2026-10-09 — the manual GUI check of 1.12.451 on macOS (2026-10-08)
    - **RG2.4.6.1** DONE 2026-10-09 (#2904) — R6: the properties dialog of a right click failed for a sensor - an empty value (`fileName` '') made `TkinterEditDictionary.GetDictionary` fail on every platform (#2904)
    - **RG2.4.6.2** DONE 2026-10-09 (#2905) — K5: real-time mode - a pause no longer counts, the simulation continues where it was (#2905)
    - **RG2.4.6.3** DONE 2026-10-09 (#2906) — S7: macOS - the render window and the dialog leave the screen when they are closed (#2906)
- <a id="rg2-5"></a>**RG2.5** DONE 2026-10-04 — The test suite without matplotlib (#2832) · [text](exudynRevisionLog2026b.md#closing-rg2-5)
- <a id="rg2-6"></a>**RG2.6** DONE 2026-10-06 — Each test model in a namespace of its own (#2875) · [text](exudynRevisionLog2026b.md#closing-rg2-6)
- <a id="rg2-3"></a>**RG2.3** DONE 2026-10-05 — A graphics regression suite (#2582) · [text](exudynRevisionLog2026b.md#closing-rg2-3)
  - **RG2.3.1** DONE 2026-09-27 — the maintainer: *"yes do that and totally remove the .txt graphics export"*. Built as `SC.renderer.GetGraphicsData()` (#2700)
  - **RG2.3.2** DONE (with its parent step) — `PlotImage(plot3D=True)` fails with every current matplotlib (#2701)
  - **RG2.3.3** DONE (with its parent step) — The graphics regression test (#2704)
    - **RG2.3.3.1** DONE 2026-09-27 — the machinery and the first cases
    - **RG2.3.3.2** DONE 2026-09-27 — the settings on one representative model
    - **RG2.3.3.3** DONE 2026-09-27 — special cases as manual examples
    - **RG2.3.3.4** DONE 2026-09-27 — the raytracer
    - **RG2.3.3.5** DONE 2026-09-30 — every item, through its MiniExample (#2751; #2704 resolved with .1 to .4)
    - **RG2.3.3.6** DONE (with its parent step) — the MiniExample graphics test sees more (#2765, #2764)
    - **RG2.3.3.7** CLOSED 2026-10-04 — an image per item for its page (#2830)
    - **RG2.3.3.8** DONE 2026-10-04 — the raytracer can hang in `RedrawAndGetImage` (#2776)
  - **RG2.3.4** DONE 2026-09-27 — `PlotImage` in 3D shows the triangles (#2706)
  - **RG2.3.5** DONE 2026-09-27 — `PlotImage` saves into `exudyn.config.outputDirectory` (#2711)
- <a id="rg2-6-b"></a>**RG2.6** MOVED to revision2026c step RG2.9 — Three old test models · [text](exudynRevisionLog2026b.md#closing-rg2-6-b)
  - **RG2.6.1** MOVED to revision2026c step RG2.9.1 — the Lie group integrator on the flybar governor (#559)
  - **RG2.6.2** MOVED to revision2026c step RG2.9.2 — `ObjectFFRF`: the force vector and the gravity forces against the paper (#591)
  - **RG2.6.3** MOVED to revision2026c step RG2.9.3 — `sliderCrank3Dbenchmark.py` adapted to the revised IFToMM model (#2326)
- <a id="rg2-7"></a>**RG2.7** DONE 2026-10-09 — The fast-module suite passes again (#2911, #2912) · [text](exudynRevisionLog2026b.md#closing-rg2-7)
- <a id="rg2-8"></a>**RG2.8** MOVED to revision2026c step RG2.8 — A converged cable run in `perfAccessFunctionsAD` (#2914) · [text](exudynRevisionLog2026b.md#closing-rg2-8)

## RG3 — Docs

- <a id="rg3-1"></a>**RG3.1** DONE 2026-09-22 — The section structure of the user manual was wrong (#2584) · [text](exudynRevisionLog2026b.md#closing-rg3-1)
- <a id="rg3-2"></a>**RG3.2** DONE 2026-09-22 — The internal how-to notes left the published documentation (#2585) · [text](exudynRevisionLog2026b.md#closing-rg3-2)
- <a id="rg3-3"></a>**RG3.3** DONE 2026-09-22 — Is there a PDF, and should there be? (#2586) · [text](exudynRevisionLog2026b.md#closing-rg3-3)
  - <a id="rg3-3-1"></a>**RG3.3.1** DONE 2026-09-25 — `exudev docs --pdf` needed Perl and did not say so (#2658) · [text](exudynRevisionLog2026b.md#closing-rg3-3-1)
- <a id="rg3-4"></a>**RG3.4** DONE 2026-09-22 — The revisions chapter says where the details are (#2587) · [text](exudynRevisionLog2026b.md#closing-rg3-4)
- <a id="rg3-5"></a>**RG3.5** DONE 2026-09-22 — The citations point nowhere (#2550) · [text](exudynRevisionLog2026b.md#closing-rg3-5)
- <a id="rg3-6"></a>**RG3.6** DONE 2026-09-22 — A generated settings page shows a table with no rows (#2592) · [text](exudynRevisionLog2026b.md#closing-rg3-6)
- <a id="rg3-7"></a>**RG3.7** DONE 2026-09-22 — Display math opened at the end of a text line swallows the text (#2593) · [text](exudynRevisionLog2026b.md#closing-rg3-7)
- <a id="rg3-8"></a>**RG3.8** DONE 2026-10-03 — The unreferenced figures are the trace of figures the conversion lost (#2594) · [text](exudynRevisionLog2026b.md#closing-rg3-8)
- <a id="rg3-9"></a>**RG3.9** DONE 2026-09-23 — Three corrections to the landing pages and the developer chapters (#2598) · [text](exudynRevisionLog2026b.md#closing-rg3-9)
- <a id="rg3-10"></a>**RG3.10** DONE 2026-09-24 — `CHANGELOG.md` and the issue tracker page hold the same list twice (#2599) · [text](exudynRevisionLog2026b.md#closing-rg3-10)
  - <a id="rg3-10-1"></a>**RG3.10.1** DONE 2026-09-24 — The changelog and the tracker page printed the same issue in two formats (#2637) · [text](exudynRevisionLog2026b.md#closing-rg3-10-1)
- <a id="rg3-11"></a>**RG3.11** DONE 2026-09-23 — "The C++ core" pointed at the repository instead of at the documentation (#2611) · [text](exudynRevisionLog2026b.md#closing-rg3-11)
- <a id="rg3-12"></a>**RG3.12** DONE 2026-09-24 — Building from source and the development workflow are told three times and never from the start (#2646) · [text](exudynRevisionLog2026b.md#closing-rg3-12)
- <a id="rg3-13"></a>**RG3.13** DONE 2026-09-24 — The tree told the reader about the revision instead of about itself (#2648, #2646) · [text](exudynRevisionLog2026b.md#closing-rg3-13)
  - <a id="rg3-13-1"></a>**RG3.13.1** DONE 2026-09-27 — 235 references to the plan are left in comments, each inside a sentence (#2649) · [text](exudynRevisionLog2026b.md#closing-rg3-13-1)
- <a id="rg3-14"></a>**RG3.14** DONE 2026-09-26 — The item and settings descriptions are written in LaTeX. (#2655) · [text](exudynRevisionLog2026b.md#closing-rg3-14)
- <a id="rg3-15"></a>**RG3.15** DONE 2026-09-25 — The chapters of the user manual (#2657, #2661, #2662) · [text](exudynRevisionLog2026b.md#closing-rg3-15)
- <a id="rg3-16"></a>**RG3.16** DONE 2026-09-25 — Every heading is sentence case (#2662) · [text](exudynRevisionLog2026b.md#closing-rg3-16)
- <a id="rg3-17"></a>**RG3.17** DONE 2026-09-25 — A comment in a description is an HTML comment (#2663) · [text](exudynRevisionLog2026b.md#closing-rg3-17)
- <a id="rg3-18"></a>**RG3.18** DONE 2026-09-26 — The pages of the Python-C++ interface repeat their own title, and it costs the MainSystem extensions their place in the table of contents (#2660) · [text](exudynRevisionLog2026b.md#closing-rg3-18)
- <a id="rg3-19"></a>**RG3.19** DONE 2026-09-26 — The arguments of a documented function are one per line, with the name in code (#2665) · [text](exudynRevisionLog2026b.md#closing-rg3-19)
- <a id="rg3-21"></a>**RG3.21** DONE 2026-09-27 — The pages that still describe the state before a step that is done (#2673) · [text](exudynRevisionLog2026b.md#closing-rg3-21)
- <a id="rg3-22"></a>**RG3.22** DONE 2026-09-27 — The simulation settings section says how to look a setting up (#2659) · [text](exudynRevisionLog2026b.md#closing-rg3-22)
- <a id="rg3-23"></a>**RG3.23** DONE 2026-09-26 — The override settings are documented where the module is (#2680) · [text](exudynRevisionLog2026b.md#closing-rg3-23)
- <a id="rg3-24"></a>**RG3.24** DONE 2026-09-27 — The generator API still says "Latex". (#2681) · [text](exudynRevisionLog2026b.md#closing-rg3-24)
- <a id="rg3-25"></a>**RG3.25** DONE 2026-09-27 — a TAB instead of a backslash put `exttt{...}` on three pages of the Symbolic manual (#2683) · [text](exudynRevisionLog2026b.md#closing-rg3-25)
- <a id="rg3-26"></a>**RG3.26** DONE 2026-09-27 — `index.md` and `pdfIndex.md` are two hand-written tables of contents that must agree - the maintainer chose option B, the check (#2697) · [text](exudynRevisionLog2026b.md#closing-rg3-26)
- <a id="rg3-27"></a>**RG3.27** DONE 2026-09-27 — The mass-spring-damper tutorial comes first (#2708) · [text](exudynRevisionLog2026b.md#closing-rg3-27)
- <a id="rg3-28"></a>**RG3.28** DONE 2026-09-29 — The developer documentation named paths of the old `main/` directory (#2743) · [text](exudynRevisionLog2026b.md#closing-rg3-28)
- <a id="rg3-29"></a>**RG3.29** DONE 2026-10-03 — The Python-C++ command interface in sections (#2808) · [text](exudynRevisionLog2026b.md#closing-rg3-29)
- <a id="rg3-30"></a>**RG3.30** DONE 2026-10-03 — The flow charts as TikZ again (#2812) · [text](exudynRevisionLog2026b.md#closing-rg3-30)
- <a id="rg3-31"></a>**RG3.31** DONE 2026-10-04 — Representative images for the item pages (#2830) · [text](exudynRevisionLog2026b.md#closing-rg3-31)
  - **RG3.31.1** DONE 2026-10-04 — *the status and a list, for the maintainer's choice*: of the 97 item pages, five have a rendered image of the item (`ObjectRigidBody`, `ObjectJointGeneric`, `ObjectJoi...
  - **RG3.31.2** DONE 2026-10-04 — after the choice: one or two scripts in `tools/itemImages/` (or a test model where one fits) that build each chosen item in a small scene and write its image with the...
  - **RG3.31.3** DONE 2026-10-04 — the images in `docs/figures/`, each named on the page of its item (`{image}` in the `detailedDescription`, as the joints do), and a look at every page.
  - **RG3.31.5** DONE 2026-10-04 — the images of the rigid body and the joints move from the overall description (where they also went into the docstrings) to the field `image`
  - **RG3.31.4** DONE 2026-10-04 — a 3D beam without polygonal section geometry is drawn as a line with an orthonormal basis at every tile (`UpdateGraphicsBeam3D`, marked "temporary!" in the code), foun...
- <a id="rg3-32"></a>**RG3.32** DONE 2026-10-04 — A display formula has no blank line (#2834) · [text](exudynRevisionLog2026b.md#closing-rg3-32)
- <a id="rg3-33"></a>**RG3.33** DONE 2026-10-04 — The start page says nothing about how the documentation is made (#2835) · [text](exudynRevisionLog2026b.md#closing-rg3-33)
- <a id="rg3-34"></a>**RG3.34** DONE 2026-10-05 — The pages of the Python-C++ interface name the notebook of an example once (#2852) · [text](exudynRevisionLog2026b.md#closing-rg3-34)
  - **RG3.34.1** DONE 2026-10-05 — the remark is the path of the notebook, in typewriter
- <a id="rg3-35"></a>**RG3.35** DONE 2026-10-05 — The tables of the PDF (#2853, #2854, #2855) · [text](exudynRevisionLog2026b.md#closing-rg3-35)
- <a id="rg3-38"></a>**RG3.38** DONE 2026-10-07 — No LaTeX source in the text of the PDF (#2883) · [text](exudynRevisionLog2026b.md#closing-rg3-38)
- <a id="rg3-37"></a>**RG3.37** DONE 2026-10-05 — A settings table names each item by its name (#2860) · [text](exudynRevisionLog2026b.md#closing-rg3-37)
- <a id="rg3-36"></a>**RG3.36** DONE 2026-10-05 — The PDF is checked with the documentation (#2856) · [text](exudynRevisionLog2026b.md#closing-rg3-36)
- <a id="rg3-39"></a>**RG3.39** CLOSED 2026-10-07 — How a model is built, and what an error says (#855, #886, #888): all three were in the manual already · [text](exudynRevisionLog2026b.md#closing-rg3-39)
- <a id="rg3-40"></a>**RG3.40** DONE 2026-10-07 — The getting-started page, checked command by command (#2888) · [text](exudynRevisionLog2026b.md#closing-rg3-40)
- <a id="rg3-41"></a>**RG3.41** DONE 2026-10-07 — ffmpeg installed with conda on every platform (#2889) · [text](exudynRevisionLog2026b.md#closing-rg3-41)
- <a id="rg3-42"></a>**RG3.42** DONE 2026-10-09 — LaTeX umlauts in the text (#2908) · [text](exudynRevisionLog2026b.md#closing-rg3-42)
- <a id="rg3-43"></a>**RG3.43** DONE 2026-10-09 — Return annotations in the signatures, and the index URL of pip (#2909, #2910) · [text](exudynRevisionLog2026b.md#closing-rg3-43)
- <a id="rg3-44"></a>**RG3.44** DONE 2026-10-09 — One landing page for GitHub and Read the Docs, a current one for PyPI (#2917) · [text](exudynRevisionLog2026b.md#closing-rg3-44)

## RG4 — Implementation problems and bugs

- <a id="rg4-1"></a>**RG4.1** MOVED to revision2026c step RG4.1 — Resolve the Windows/Linux differences in contact and friction models. · [text](exudynRevisionLog2026b.md#closing-rg4-1)
  - **RG4.1.1** DONE 2026-09-26 — `UnresolvedOnMacOS()` beside `UnresolvedOnLinux()`, applied on darwin, so the suite exits 0 there as it does on Linux
  - **RG4.1.2** MOVED to revision2026c step RG4.1.2 — the five macOS-only models
  - **RG4.1.4** DONE 2026-10-06 — to be confirmed on the Mac — (#2869) `-ffp-contract=off` for every unix build, not only the fast module - measured there: the models off the reference go from 8 to 3
  - **RG4.1.5** DONE 2026-10-06 — confirmed on the Mac 2026-10-07 — (#2876) the raytraced image of `RedrawAndGetImage` is no longer drawn into an OpenGL window that does not exist in the Python thread...
  - **RG4.1.6** DONE 2026-10-08 — the local `exudev build --complete` of 1.12.450: `energiesTest.py` differs by 5.9e-11 with numpy 2.2 (venvP310, as on Linux) - a tolerance factor instead of `Unresolve...
  - **RG4.1.3** MOVED to revision2026c step RG4.1.3 — the math library and uninitialized values
- <a id="rg4-2"></a>**RG4.2** DONE 2026-09-26 — `ObjectContactConvexRoll.pContact` is a computed value that Python reads (#2413) · [text](exudynRevisionLog2026b.md#closing-rg4-2)
- <a id="rg4-3"></a>**RG4.3** DONE 2026-10-02 — Explicit integration cost (#2398) · [text](exudynRevisionLog2026b.md#closing-rg4-3)
- <a id="rg4-4"></a>**RG4.4** DONE 2026-09-23 — Two lines of Python segfault the process (#2603) · [text](exudynRevisionLog2026b.md#closing-rg4-4)
- <a id="rg4-5"></a>**RG4.5** DONE 2026-09-23 — Quitting the renderer before a simulation started raised, quitting during it did not (#2616) · [text](exudynRevisionLog2026b.md#closing-rg4-5)
- <a id="rg4-6"></a>**RG4.6** DONE 2026-09-28 — A test hook for `forceQuitSimulation` - decided for a binding a user can use as well, `SC.renderer.StopSimulation(forceQuit=True)` (#2674, #2616) · [text](exudynRevisionLog2026b.md#closing-rg4-6)
- <a id="rg4-7"></a>**RG4.7** CLOSED 2026-09-29, done by revision2026 step R6.3.5 — Every C++ user error inspects the Python source for its file and line (#2423) · [text](exudynRevisionLog2026b.md#closing-rg4-7)
- <a id="rg4-8"></a>**RG4.8** DONE 2026-09-30 — `ObjectBeamGeometricallyExact` (3D): analyse the implementation (#2730) · [text](exudynRevisionLog2026b.md#closing-rg4-8)
  - **RG4.8.13** DONE 2026-10-01 — `rightAngleFrame.py` (ANCF and this element, not run by the suite) stops near the buckling load with its load-driven settings: drive it by displacement instead - a coo...
  - **RG4.8.14** DONE 2026-10-01 — the switch of the mass matrix stays, as a special setting and not an experimental one: `exu.special.beams.geometricallyExactLumpedMass` (default False, the consistent...
- <a id="rg4-9"></a>**RG4.9** DONE 2026-09-29 — The cable and beam shape markers accept any body (#2731) · [text](exudynRevisionLog2026b.md#closing-rg4-9)
- <a id="rg4-10"></a>**RG4.10** DONE 2026-09-29 — `ObjectGenericODE2` and `ObjectKinematicTree` admit the general body markers, which then fail (#2734) · [text](exudynRevisionLog2026b.md#closing-rg4-10)
- <a id="rg4-11"></a>**RG4.11** DONE 2026-09-29 — `ObjectContactCoordinate` ignores `activeConnector`, and its output variable `Distance` raises (#2735) · [text](exudynRevisionLog2026b.md#closing-rg4-11)
- <a id="rg4-12"></a>**RG4.12** CLOSED 2026-10-02 — (maintainer: closed, so it stays visible · [text](exudynRevisionLog2026b.md#closing-rg4-12)
- <a id="rg4-13"></a>**RG4.13** DONE 2026-09-29 — `ObjectKinematicTree`: the position Jacobian of a prismatic joint rotates the axis twice (#2740) · [text](exudynRevisionLog2026b.md#closing-rg4-13)
- <a id="rg4-14"></a>**RG4.14** DONE 2026-09-30 — `ObjectBeamGeometricallyExact2D`: a test of the 3-node element (#2208) · [text](exudynRevisionLog2026b.md#closing-rg4-14)
- <a id="rg4-15"></a>**RG4.15** DONE 2026-10-05 (all sub-steps) — The open bugs and fixes before 1.13. · [text](exudynRevisionLog2026b.md#closing-rg4-15)
  - **RG4.15.1** DONE 2026-09-29 — one drop, three contact objects: the test model `contactComparisonTest.py`, the check in compensation for #738
  - **RG4.15.2** DONE 2026-09-29 — `ObjectContactCoordinate` gets the contact law of `ObjectContactSphereSphere` - `contactStiffnessExponent`, `restitutionCoefficient`, `impactModel`, `minimumImpactVelo...
  - **RG4.15.3** DONE 2026-09-30 — the explicit solvers do no post Newton step: a warning at the start of an explicit solve names the objects that are not updated - **wrong, and removed by RG4.16.1** (#...
  - **RG4.15.4** DONE 2026-09-30 — `ObjectContactSphereTorus`: momentum conservation - the torque of the normal force on the ring was missing without friction (#2127)
  - **RG4.15.5** CLOSED 2026-09-30 — a repeated `mbs.SolveDynamic` with `ObjectFFRFreducedOrder` diverges - not reproduced with `objectFFRFreducedOrderTest.py`, `superElementRigidJointTest.py` and `abaqus...
  - **RG4.15.6** DONE 2026-09-29 — `mbs.GetDictionary()` works with a symbolic user function, but `mbs.SetDictionary()` of that dictionary fails (*"Unable to cast ... symbolic.UserFunction"*) (#1888)
  - **RG4.15.7** DONE 2026-09-30 — the numerical ODE1 Jacobian with coordinates an object addresses twice: each column once, as for ODE2 (#1424)
  - **RG4.15.8** DONE 2026-10-04 — `GeneralContact`, implicit sphere-triangle contact, as the fourth case of the drop, and a sliding ball against the rigid-body solution: it agrees (#1848, #1947)
  - **RG4.15.9** DONE 2026-10-05 — `GeneralContact` gets the setting `keepContactWhilePenetrating`: a sphere-sphere or sphere-triangle contact then acts while the bodies penetrate, as the contact object...
  - **RG4.15.10** DONE 2026-10-05 — `GeneralContact`: the contact force of a sphere-triangle contact gives the triangle body its torque also without friction (#2849)
- <a id="rg4-16"></a>**RG4.16** DONE 2026-09-30 — Explicit solvers and the states of the PostNewton step (#2754, #830, #2109) · [text](exudynRevisionLog2026b.md#closing-rg4-16)
- <a id="rg4-17"></a>**RG4.17** CLOSED (#2763) — `ObjectANCFBeam`: the Newton iteration stalls in the right-angle frame (#2763) · [text](exudynRevisionLog2026b.md#closing-rg4-17)
  - **RG4.17.1** DONE 2026-10-02 — the corner: the rotation Jacobian of the slope nodes was not the derivative of their rotation
  - **RG4.17.2** DONE 2026-10-04 — the stall analysed: no error in the model or the solver, but a very small region of Newton convergence of the ANCF frame
  - **RG4.17.3** DONE 2026-10-05 — `rightAngleFrame.py` keeps the geometrically exact beam, and its Details say where and why the ANCF variant stops
- <a id="rg4-18"></a>**RG4.18** DONE 2026-10-02 — A system without coordinates is solved (#2790) · [text](exudynRevisionLog2026b.md#closing-rg4-18)
- <a id="rg4-19"></a>**RG4.19** DONE 2026-10-07 (all sub-steps) — The open BUG, FIX and CHECK issues, evaluated: options. · [text](exudynRevisionLog2026b.md#closing-rg4-19)
  - **RG4.19.1** DONE 2026-10-05 — an inactive `RigidBodySpringDamper`, `LinearSpringDamper` and `TorsionalSpringDamper` reports no force or torque (#984)
  - **RG4.19.2** DONE 2026-10-05 — the `GeneralContact` of `AddGeneralContact` / `GetGeneralContact` keeps its system alive (`reference_internal`) (#1512)
  - **RG4.19.3** DONE 2026-10-05 — every allocation of `ResizableArray` goes through one function that catches `bad_alloc`, as `Vector` and `Matrix` do. (#121)
  - **RG4.19.4** DONE 2026-10-05 — with adaptive or automatic steps a singular system Jacobian is a failed step, which is reduced (#1337)
  - **RG4.19.5** DONE 2026-10-05 — `ObjectContactFrictionCircleCable2D` without a friction model applied the slip force of a data node that started in slip (#1290)
  - **RG4.19.6** DONE 2026-10-05 — the analytic transposed `AE_ODE2_t` block is right (#692)
  - **RG4.19.7** DECIDED 2026-10-05 — the contact objects stay as they are - in contact while the bodies penetrate, so that the damping may pull: that is what the restitution models of `impactModel` assume...
  - **RG4.19.8** DECIDED 2026-10-05 — `InitializeFromRestartFile` is no longer public (#1565 closed, superseded by #2850)
  - **RG4.19.9** DECIDED 2026-10-05 — closed #171, #172, #173, #174, #436 (not worth a change), and #1677, #1682, #1683, #1684, #1956 (done by the revisions: `GetDictionary` of the system, the `Inspect`/`C...
  - **RG4.19.10** DONE 2026-10-05 — `AddSystem` kept its container alive already (`keep_alive<0, 1>`), `GetSystem` did not and left a system in a deleted container (#2851)
  - **RG4.19.11** DONE 2026-10-05 — closed: #1845 (replaced by native C++ tests in Exudyn), #973 (the tests exist), #1167 and #1247 (symbolic user functions remove the round trip, C++ user functions will...
  - **RG4.19.12** DONE 2026-10-06 — after a step, `SlidingCoordinate` of JointSliding and JointSliding2D counts the last increment of the sliding coordinate twice: `PostNewtonStep` adds the algebraic inc...
  - **RG4.19.13** DONE 2026-10-06 — to be confirmed on Linux — (#2874) the restart file of an explicit solver held the never initialized `aAlgorithmic` (subnormal numbers, `-nan`), which glibc's `std::st...
  - **RG4.19.14** DONE 2026-10-07 — the expected warnings of the deprecation tests kept off the console (#2882)
- <a id="rg4-20"></a>**RG4.20** MOVED to revision2026c step RG4.20 — `ObjectANCFThinPlate` and `exudyn.shells`: the revision of a colleague, ported (#2857, #2858, #2859) · [text](exudynRevisionLog2026b.md#closing-rg4-20)
  - **RG4.20.1** DONE 2026-10-05 — the element: material curvature measure, membrane and bending integrated at separate points (mode 0 Gauss 5 x 5 (#2857)
  - **RG4.20.2** DONE 2026-10-05 — `exudyn.shells`: `ANCFThinPlateBuilder`, the geometry maps, the constraint, load and hinge functions, ShellMesh with damping and a thickness function (#2858)
  - **RG4.20.3** DONE 2026-10-05 — the visualization, as decided: no averaging and no members in `VisualizationSystem` (#2859)
  - **RG4.20.4** DONE 2026-10-05 — the decisions: one sign of `CurvatureLocal` and `TorqueLocal` (#2864)
  - **RG4.20.5** MOVED to revision2026c step RG4.20.5 — the open issues of the element: the rotation as an access function (#2218), the averaged rotation of `NodePointSlope12` symmetric in both slopes (#2219, with the basis...
- <a id="rg4-21"></a>**RG4.21** MOVED to revision2026c step RG4.21 — Constraints that depend on velocities or on time · [text](exudynRevisionLog2026b.md#closing-rg4-21)
- <a id="rg4-22"></a>**RG4.22** MOVED to revision2026c step RG4.22 — A 3D sliding joint · [text](exudynRevisionLog2026b.md#closing-rg4-22)
- <a id="rg4-23"></a>**RG4.23** MOVED to revision2026c step RG4.23 — The Jacobian of ODE1 loads (#1548) · [text](exudynRevisionLog2026b.md#closing-rg4-23)
- <a id="rg4-24"></a>**RG4.24** MOVED to revision2026c step RG4.24 — `ObjectRollingDiscPenalty`: viscous rolling friction for any axis (#926) · [text](exudynRevisionLog2026b.md#closing-rg4-24)
- <a id="rg4-25"></a>**RG4.25** DONE 2026-10-07 — `mbs.ComputeItem` cast every object to a body (#2892) · [text](exudynRevisionLog2026b.md#closing-rg4-25)
- <a id="rg4-26"></a>**RG4.26** DONE 2026-10-07 — `InverseKinematicsNumericalExample.py` (#2896) · [text](exudynRevisionLog2026b.md#closing-rg4-26)

## RG5 — Performance

- <a id="rg5-1"></a>**RG5.1** MOVED to revision2026c step RG5.1 — A maintained micro-benchmark for the linear algebra, inside Exudyn (#2397) · [text](exudynRevisionLog2026b.md#closing-rg5-1)
  - **RG5.1.1** MOVED to revision2026c step RG5.1.1 — the products of `HomogeneousTransformation` - $\Hm\vv$, $\Hm_1\Hm_2$, $\Hm^{-1}$ - with and without the flag of no rotation (`HasNoRotation`, set by construction and,...
  - **RG5.1.2** MOVED to revision2026c step RG5.1.2 — `ResizableVector` merged into `Vector`, `LinkedDataVector` a specialization of `Vector` without virtual calls - measured with the benchmark first, changed only for a g...
  - **RG5.1.3** MOVED to revision2026c step RG5.1.3 — `CollectCurrentNodeMarkerData` for `NodeRigidBodyRotVecLG`: the time it takes in a benchmark of rigid bodies, then the optimization.
  - **RG5.1.4** MOVED to revision2026c step RG5.1.4 — the inlining of `Vector::operator[]` and `ConstVector` (#142), template specializations of `SlimArray<T,2..4>` as for `SlimVector` (#161), and `CollectCurrentNodeData`...
- <a id="rg5-2"></a>**RG5.2** MOVED to revision2026c step RG5.2 — Make the hot linear algebra vectorizable. · [text](exudynRevisionLog2026b.md#closing-rg5-2)
- <a id="rg5-3"></a>**RG5.3** MOVED to revision2026c step RG5.3 — The Jacobians assembled in parallel (#1203) · [text](exudynRevisionLog2026b.md#closing-rg5-3)
- <a id="rg5-4"></a>**RG5.4** MOVED to revision2026c step RG5.4 — The superelements for larger meshes · [text](exudynRevisionLog2026b.md#closing-rg5-4)

## RG6 — Graphics and rendering

- <a id="rg6-1"></a>**RG6.1** DONE 2026-09-24 — OpenVR is removed (#2645) · [text](exudynRevisionLog2026b.md#closing-rg6-1)
- <a id="rg6-2"></a>**RG6.2** DONE 2026-09-23 — The settings dialogs, and the shape of `GUI.py` (reopened and closed again the same day for RG6.2.12 to RG6.2.15) (#2591) · [text](exudynRevisionLog2026b.md#closing-rg6-2)
  - <a id="rg6-2-1"></a>**RG6.2.1** DONE 2026-09-22 — The dialogs leave the C++. (#2595) · [text](exudynRevisionLog2026b.md#closing-rg6-2-1)
  - <a id="rg6-2-2"></a>**RG6.2.2** DONE 2026-09-23 — The layer under the widgets gets tests (#2596) · [text](exudynRevisionLog2026b.md#closing-rg6-2-2)
  - <a id="rg6-2-3"></a>**RG6.2.3** DONE 2026-09-23 — The six small complaints (#2597, #2601) · [text](exudynRevisionLog2026b.md#closing-rg6-2-3)
    - <a id="rg6-2-3-1"></a>**RG6.2.3.1** DONE 2026-09-23 — One `dialogs.fontScaling` for every platform (#2602) · [text](exudynRevisionLog2026b.md#closing-rg6-2-3-1)
  - <a id="rg6-2-4"></a>**RG6.2.4** DONE 2026-09-23 — Inline editing (#2604) · [text](exudynRevisionLog2026b.md#closing-rg6-2-4)
  - <a id="rg6-2-5"></a>**RG6.2.5** DROPPED 2026-09-23 — A second front end · [text](exudynRevisionLog2026b.md#closing-rg6-2-5)
  - <a id="rg6-2-6"></a>**RG6.2.6** DONE 2026-09-23 — The key bindings are written down three times (#2591) · [text](exudynRevisionLog2026b.md#closing-rg6-2-6)
  - <a id="rg6-2-7"></a>**RG6.2.7** DONE 2026-09-23 — `GUI.py` is cleaned up, last (#2591) · [text](exudynRevisionLog2026b.md#closing-rg6-2-7)
  - <a id="rg6-2-8"></a>**RG6.2.8** DONE 2026-09-23 — The bottom row reads like code, and says each thing once (#2605) · [text](exudynRevisionLog2026b.md#closing-rg6-2-8)
  - <a id="rg6-2-9"></a>**RG6.2.9** DONE 2026-09-23 — A changed value is visible, and every change can be copied at once (#2606) · [text](exudynRevisionLog2026b.md#closing-rg6-2-9)
  - <a id="rg6-2-10"></a>**RG6.2.10** DONE 2026-09-23 — Find a setting (#2607) · [text](exudynRevisionLog2026b.md#closing-rg6-2-10)
  - <a id="rg6-2-11"></a>**RG6.2.11** DONE 2026-09-23 — The catalogue of optional features, decided (#2608) · [text](exudynRevisionLog2026b.md#closing-rg6-2-11)
  - <a id="rg6-2-12"></a>**RG6.2.12** DONE 2026-09-23 — 59 untouched settings were called changed, and a folded folder hid a change (#2612) · [text](exudynRevisionLog2026b.md#closing-rg6-2-12)
  - <a id="rg6-2-13"></a>**RG6.2.13** DONE 2026-09-23 — The find bar needs no button, and says nothing when it is idle (#2613) · [text](exudynRevisionLog2026b.md#closing-rg6-2-13)
  - <a id="rg6-2-14"></a>**RG6.2.14** DONE 2026-09-23 — Reset, revert, undo, close - and the windows stay in front (#2614) · [text](exudynRevisionLog2026b.md#closing-rg6-2-14)
  - <a id="rg6-2-15"></a>**RG6.2.15** DONE 2026-09-23 — A settings folder has a description, and nothing shows it (#2615) · [text](exudynRevisionLog2026b.md#closing-rg6-2-15)
  - <a id="rg6-2-16"></a>**RG6.2.16** DONE 2026-09-23 — The window with the changes was invisible (#2621) · [text](exudynRevisionLog2026b.md#closing-rg6-2-16)
  - <a id="rg6-2-17"></a>**RG6.2.17** DONE 2026-09-23 — Opening the dialog re-pointed `exudyn.sys` at a throw-away container (#2623) · [text](exudynRevisionLog2026b.md#closing-rg6-2-17)
  - <a id="rg6-2-18"></a>**RG6.2.18** DONE 2026-09-24 — The same dialog for `simulationSettings` (#2624) · [text](exudynRevisionLog2026b.md#closing-rg6-2-18)
  - <a id="rg6-2-19"></a>**RG6.2.19** DONE 2026-09-23 — Opening the settings dialog closed the render window (#2625) · [text](exudynRevisionLog2026b.md#closing-rg6-2-19)
  - <a id="rg6-2-20"></a>**RG6.2.20** DONE 2026-09-23 — The defaults of the lights and the raytracer materials are hidden in C++ constructors (#2626) · [text](exudynRevisionLog2026b.md#closing-rg6-2-20)
  - <a id="rg6-2-21"></a>**RG6.2.21** DONE 2026-09-23 — The dialog stopped asking, and undo goes back one whole state (#2627) · [text](exudynRevisionLog2026b.md#closing-rg6-2-21)
  - <a id="rg6-2-22"></a>**RG6.2.22** DONE 2026-09-23 — A double click on a bool no longer toggled it (#2630, #2604) · [text](exudynRevisionLog2026b.md#closing-rg6-2-22)
  - <a id="rg6-2-23"></a>**RG6.2.23** DONE 2026-09-23 — `dialogs.fontScaling` only worked at 0 (#2631) · [text](exudynRevisionLog2026b.md#closing-rg6-2-23)
  - <a id="rg6-2-24"></a>**RG6.2.24** DONE 2026-09-24 — The dialogs opened from the command line were larger and blurred (#2634) · [text](exudynRevisionLog2026b.md#closing-rg6-2-24)
  - <a id="rg6-2-25"></a>**RG6.2.25** DONE 2026-09-24 — The combo box of an enum repeated the type name in every entry (#2635) · [text](exudynRevisionLog2026b.md#closing-rg6-2-25)
    - <a id="rg6-2-25-1"></a>**RG6.2.25.1** DONE 2026-09-24 — The value cell showed the type name again as soon as the combo box collapsed (#2640) · [text](exudynRevisionLog2026b.md#closing-rg6-2-25-1)
  - <a id="rg6-2-26"></a>**RG6.2.26** DONE 2026-09-24 — The tooltips were invisible while the dialog is topmost (#2639, #2621) · [text](exudynRevisionLog2026b.md#closing-rg6-2-26)
  - <a id="rg6-2-28"></a>**RG6.2.28** DONE 2026-09-23 — The lights and the raytracer materials are defaults of the structure (#2626) · [text](exudynRevisionLog2026b.md#closing-rg6-2-28)
- <a id="rg6-3"></a>**RG6.3** CLOSED 2026-09-27, superseded — The renderer extraction functions are not shaped for testing (#2583, #2700) · [text](exudynRevisionLog2026b.md#closing-rg6-3)
- <a id="rg6-4"></a>**RG6.4** DONE 2026-09-23 — The light and shadow descriptions say things that are no longer true (#2609) · [text](exudynRevisionLog2026b.md#closing-rg6-4)
- <a id="rg6-5"></a>**RG6.5** DONE 2026-09-24 — Restoring the saved render state took two lines in 82 places (#2633) · [text](exudynRevisionLog2026b.md#closing-rg6-5)
- <a id="rg6-6"></a>**RG6.6** DONE 2026-09-24 — macOS: the settings dialog aborted the process (#2643) · [text](exudynRevisionLog2026b.md#closing-rg6-6)
- <a id="rg6-7"></a>**RG6.7** DONE 2026-10-03 — GraphicsData gets a Sphere and a CurvedTriangleList (#2709) · [text](exudynRevisionLog2026b.md#closing-rg6-7)
- <a id="rg6-9"></a>**RG6.9** DONE 2026-10-04 — Curves of connectors drawn with a tiling of their own, as watertight tubes (#2839) · [text](exudynRevisionLog2026b.md#closing-rg6-9)
  - **RG6.9.1** DONE (with its parent step) — a setting `connectors.curveTiling` - segments per full turn of a curve -, the number of segments of an arc in proportion to its angle
  - **RG6.9.2** DONE (with its parent step) — a function of `EXUvis` that draws a watertight tube along a polyline (one ring of vertices per point, shared by the neighboring segments, normals from the ring), used...
  - **RG6.9.3** DONE (with its parent step) — the graphics regression references and the item images of the connectors drawn again
- <a id="rg6-8"></a>**RG6.8** MOVED to revision2026c step RG6.8 — The graphics fixes before 1.13 · [text](exudynRevisionLog2026b.md#closing-rg6-8)
  - **RG6.8.1** DONE 2026-09-30 — marker positions in `AnimateModes` with deformation scaling 0 - **headless**: the marker positions in `SC.renderer.GetGraphicsData()` against the reference positions (...
  - **RG6.8.2** DONE 2026-09-30 — headless (#2309)
  - **RG6.8.3** DONE 2026-09-30 — meshes from NGsolve give triangles of the wrong orientation - with ngsolve (optional package): the normals of `fem.GetSurfaceTriangles()` against the outward normals (...
  - **RG6.8.4** DONE 2026-09-30 — erratic shadows with `modelCentricView=False` and lights in the camera frame - a small raytracer image against a reference, as in RG2.3.3.4 (#2308; to be seen on scree...
  - **RG6.8.5** DONE 2026-10-08 — Linux: crashes when the renderer closes and with the SolutionViewer (#2140 resolved: "a larger amount of tests could not reproduce the linux failure", the maintainer;...
  - **RG6.8.6** DONE 2026-10-07 — macOS: PlotSensor in Spyder (#2237, #2350 closed: the maintainer's checks on macOS of 1.12.37x)
    - **RG6.8.6.1** DONE 2026-10-05 — a solver that is destroyed stops only the threads it started: the garbage collector of a console that runs a script again destroyed the solver of the earlier run while...
    - **RG6.8.6.2** DONE 2026-10-05 — the build on macOS (Apple clang 15) stopped in `CObjectJointGeneric.cpp`: `A0all.GetColumnVector<3>` on a matrix of the template type needs `.template`, which MSVC doe...
  - **RG6.8.7** DONE 2026-10-04 — the raytracer draws no `GraphicsData` of type `Circle` (and none of the circles the 2D items draw), found by the `GraphicsData` example of RG17.5. (#2844)
  - **RG6.8.8** MOVED to revision2026c step RG6.8.8 — the near and far planes: the visibility problems at a larger zoom (#2204, is it the limit 0.1?), and manual offsets of `zNear` and `zFar` in the settings (#2194)
- <a id="rg6-10"></a>**RG6.10** DONE 2026-10-05 — The model view as code (#2862) · [text](exudynRevisionLog2026b.md#closing-rg6-10)
  - <a id="rg6-10-1"></a>**RG6.10.1** DONE 2026-10-06 — store model view (#2871) · [text](exudynRevisionLog2026b.md#closing-rg6-10-1)
- <a id="rg6-11"></a>**RG6.11** MOVED to revision2026c step RG6.11 — Textures · [text](exudynRevisionLog2026b.md#closing-rg6-11)
- <a id="rg6-12"></a>**RG6.12** MOVED to revision2026c step RG6.12 — Two renderer tools · [text](exudynRevisionLog2026b.md#closing-rg6-12)
- <a id="rg6-13"></a>**RG6.13** DONE 2026-10-07 — The fonts of the tkinter dialogs on Linux (#2890) · [text](exudynRevisionLog2026b.md#closing-rg6-13)
- <a id="rg6-14"></a>**RG6.14** DONE 2026-10-08 (#2893) — Display scaling on Linux and the font of all dialogs (#2893, with #2277) · [text](exudynRevisionLog2026b.md#closing-rg6-14)
- <a id="rg6-15"></a>**RG6.15** DONE 2026-10-07 — One display scaling setting, and the dialog font for every widget (#2897) · [text](exudynRevisionLog2026b.md#closing-rg6-15)
- <a id="rg6-16"></a>**RG6.16** DONE 2026-10-07 — The scaling of the render window and of the dialogs, restarted (#2898) · [text](exudynRevisionLog2026b.md#closing-rg6-16)
- <a id="rg6-17"></a>**RG6.17** DONE 2026-10-07 — `bodies.shells.inPlaneTiling` (#2899) · [text](exudynRevisionLog2026b.md#closing-rg6-17)

## RG7 — Python user items


## RG8 — Compiled C++ user items

- <a id="rg8-1"></a>**RG8.1** MOVED to revision2026c step RG8.1 — Make the registry cross-binary · [text](exudynRevisionLog2026b.md#closing-rg8-1)
- <a id="rg8-2"></a>**RG8.2** MOVED to revision2026c step RG8.2 — Define an ABI fingerprint checked at registration: exudyn version, active macro set, compiler id and version, `__cplusplus`, and on MSVC `_ITERATOR_DEBUG_LEVEL` plus t... · [text](exudynRevisionLog2026b.md#closing-rg8-2)
- <a id="rg8-3"></a>**RG8.3** MOVED to revision2026c step RG8.3 — Commit a reference plugin subdirectory built on every commit: minimum compile, a trivial registered object, a handshake assertion, and a test that adds it to a system... · [text](exudynRevisionLog2026b.md#closing-rg8-3)
- <a id="rg8-4"></a>**RG8.4** MOVED to revision2026c step RG8.4 — Ship the plugin headers in the wheel · [text](exudynRevisionLog2026b.md#closing-rg8-4)
- <a id="rg8-5"></a>**RG8.5** MOVED to revision2026c step RG8.5 — Change duplicate handling in `RegisterClass` from `CHECKandTHROWstring` to a collected, reported error · [text](exudynRevisionLog2026b.md#closing-rg8-5)
- <a id="rg8-6"></a>**RG8.6** MOVED to revision2026c step RG8.6 — Discovery at import: `~/.exudyn/plugins/` plus `EXUDYN_PLUGIN_PATH`, and `importlib.metadata` entry points for pip-installed plugins · [text](exudynRevisionLog2026b.md#closing-rg8-6)
- <a id="rg8-7"></a>**RG8.7** MOVED to revision2026c step RG8.7 — Record loaded plugins and versions in the solver log and solution file header · [text](exudynRevisionLog2026b.md#closing-rg8-7)
- <a id="rg8-8"></a>**RG8.8** MOVED to revision2026c step RG8.8 — Extend the revision2026 step R4.3 emitter to scaffold a plugin from a user's definition file — C++ skeleton, Python dict-building class, `.pyi` stub — emitted into the... · [text](exudynRevisionLog2026b.md#closing-rg8-8)
- <a id="rg8-9"></a>**RG8.9** MOVED to revision2026c step RG8.9 — Document three constraints: plugins are never unloaded or reloaded (a rebuilt plugin needs a kernel restart in Spyder/Jupyter) · [text](exudynRevisionLog2026b.md#closing-rg8-9)
- <a id="rg8-10"></a>**RG8.10** MOVED to revision2026c step RG8.10 — Compiled user functions · [text](exudynRevisionLog2026b.md#closing-rg8-10)

## RG9 — Structural core improvements

- <a id="rg9-1"></a>**RG9.1** DONE 2026-09-23 — The item sources stop paying for pybind11 (#2622) · [text](exudynRevisionLog2026b.md#closing-rg9-1)
- <a id="rg9-2"></a>**RG9.2** DONE 2026-09-23 — Fourteen item sources included an exception header they do not use, and paid pybind11 for it (#2628, #2629) · [text](exudynRevisionLog2026b.md#closing-rg9-2)
- <a id="rg9-3"></a>**RG9.3** DONE 2026-10-03 — Access functions as single functions of the objects (#2744) · [text](exudynRevisionLog2026b.md#closing-rg9-3)
- <a id="rg9-4"></a>**RG9.4** DONE 2026-10-03 — Kinetic and potential energy as output variables (#2202) · [text](exudynRevisionLog2026b.md#closing-rg9-4)
- <a id="rg9-5"></a>**RG9.5** DONE 2026-10-03 — `mbs.ComputeItem`: the computation functions of an item from Python (#2779) · [text](exudynRevisionLog2026b.md#closing-rg9-5)
- <a id="rg9-6"></a>**RG9.6** MOVED to revision2026c step RG9.6 — The member functions an item must provide, checked when it compiles (#2866, #2498) · [text](exudynRevisionLog2026b.md#closing-rg9-6)
  - **RG9.6.1** MOVED to revision2026c step RG9.6.1 — the table: declaration (access function types, node type, output variables, Jacobian flags) to the functions it requires, from the checks of `CSystem.cpp` and the base...
  - **RG9.6.2** MOVED to revision2026c step RG9.6.2 — `itemHeaderEmitter.py` writes the `static_assert`s
  - **RG9.6.3** DONE 2026-10-01 (#2768) — the runtime half: a test model that reads every output variable an item declares, over the MiniExamples (`mbs.Inspect` does it for a model already, #2768).

## RG10 — Tooling and process

- <a id="rg10-1"></a>**RG10.1** DONE 2026-09-27 — Checker for user scripts after the 1.12 API changes: exudev scripts, a maintainer tool. (#2712) · [text](exudynRevisionLog2026b.md#closing-rg10-1)
  - **RG10.1.1** DONE 2026-10-04 — `exudev scripts --run` runs each script in a copy of its folder, without windows, with a timeout, after a check for paths outside the folder (#2713)
- <a id="rg10-2"></a>**RG10.2** DONE 2026-09-23 — The issue table of `exudev issue serve` did not say what its columns are, and left out the priority (#2600) · [text](exudynRevisionLog2026b.md#closing-rg10-2)
  - <a id="rg10-2-1"></a>**RG10.2.1** DONE 2026-09-24 — The search of `exudev issue serve` missed most fields, and the list stopped at 400 (#2636, #2240) · [text](exudynRevisionLog2026b.md#closing-rg10-2-1)
  - <a id="rg10-2-2"></a>**RG10.2.2** DONE 2026-09-24 — A search for digits found every field except the issue number (#2641, #2497, #249, #2490, #2499) · [text](exudynRevisionLog2026b.md#closing-rg10-2-2)
- <a id="rg10-3"></a>**RG10.3** DONE 2026-09-23 — `exudev` does not say how long a step took (#2617) · [text](exudynRevisionLog2026b.md#closing-rg10-3)
- <a id="rg10-4"></a>**RG10.4** DONE 2026-09-23 — `src/pythonGenerator/` holds one file and should not exist (#2618) · [text](exudynRevisionLog2026b.md#closing-rg10-4)
- <a id="rg10-5"></a>**RG10.5** DONE 2026-09-23 — VS Code cannot follow a C++ include (#2619) · [text](exudynRevisionLog2026b.md#closing-rg10-5)
- <a id="rg10-6"></a>**RG10.6** DONE 2026-09-24 — The TestModels imported the test suite to find out whether they are being tested (#2632) · [text](exudynRevisionLog2026b.md#closing-rg10-6)
- <a id="rg10-7"></a>**RG10.7** DONE 2026-09-24 — The plan carried the full text of the steps that are finished (#2638) · [text](exudynRevisionLog2026b.md#closing-rg10-7)
  - <a id="rg10-7-1"></a>**RG10.7.1** DONE 2026-09-24 — The plan did not say what to do next (#2642) · [text](exudynRevisionLog2026b.md#closing-rg10-7-1)
- <a id="rg10-8"></a>**RG10.8** DONE 2026-09-24 — `exudev` is needed on linux and macOS too (#2644) · [text](exudynRevisionLog2026b.md#closing-rg10-8)
- <a id="rg10-9"></a>**RG10.9** DONE 2026-09-24 — `regenerated_files` failed on linux and could not fail on Windows (#2647) · [text](exudynRevisionLog2026b.md#closing-rg10-9)
- <a id="rg10-10"></a>**RG10.10** DONE 2026-09-25 — The header of `definitionLoader.py` read like the file was dead (#2653) · [text](exudynRevisionLog2026b.md#closing-rg10-10)
- <a id="rg10-11"></a>**RG10.11** DONE 2026-09-28 — `exudyn.config` and `exudyn.special` reach a stub file (#2541) · [text](exudynRevisionLog2026b.md#closing-rg10-11)
- <a id="rg10-12"></a>**RG10.12** DONE 2026-09-29 — The GitLab job `check_docstrings` passes, and the gates run pydoclint (#2747) · [text](exudynRevisionLog2026b.md#closing-rg10-12)
- <a id="rg10-13"></a>**RG10.13** DONE 2026-09-29 — `exudev issue plot` (#2752) · [text](exudynRevisionLog2026b.md#closing-rg10-13)
- <a id="rg10-14"></a>**RG10.14** DONE 2026-09-30 — `exudev` runs the pytest files and the MiniExample performance run (#2760) · [text](exudynRevisionLog2026b.md#closing-rg10-14)
- <a id="rg10-15"></a>**RG10.15** DONE 2026-10-04 — `exudev env` probes the environments that exist (#2833) · [text](exudynRevisionLog2026b.md#closing-rg10-15)
- <a id="rg10-16"></a>**RG10.16** DONE 2026-10-05 — `exudev notebooks` runs (#2865) · [text](exudynRevisionLog2026b.md#closing-rg10-16)
- <a id="rg10-17"></a>**RG10.17** DONE 2026-10-06 — `exudev linux` asks WSL for the path without its shell (#2868) · [text](exudynRevisionLog2026b.md#closing-rg10-17)
- <a id="rg10-18"></a>**RG10.18** DONE 2026-10-07 — `exudev issue html` (#2885) · [text](exudynRevisionLog2026b.md#closing-rg10-18)
- <a id="rg10-19"></a>**RG10.19** DONE 2026-10-07 — The scipy pin of the `dev` group only where scipy 1.15.2 has wheels (#2886) · [text](exudynRevisionLog2026b.md#closing-rg10-19)
- <a id="rg10-20"></a>**RG10.20** DONE 2026-10-07 — `exudev --py 315` / `316` (#2891) · [text](exudynRevisionLog2026b.md#closing-rg10-20)
- <a id="rg10-21"></a>**RG10.21** DONE 2026-10-07 — A new generated page that git does not know fails the regeneration check (#2894) · [text](exudynRevisionLog2026b.md#closing-rg10-21)
- <a id="rg10-22"></a>**RG10.22** DONE 2026-10-07 — What a quiet run shows, and a log of a sane size (#2895) · [text](exudynRevisionLog2026b.md#closing-rg10-22)
- <a id="rg10-23"></a>**RG10.23** DONE 2026-10-08 — What the complete build still printed · [text](exudynRevisionLog2026b.md#closing-rg10-23)
  - **RG10.23.1** DONE (with its parent step) — the quiet build shows pip's message on conflicting packages as a note, with the conflicts (#2901)
  - **RG10.23.2** DONE (with its parent step) — the pytest files use the current helpers where they do not test a deprecation (#2902)
  - **RG10.23.3** DONE (with its parent step) — `spotReinforcementLearning.py` imports `spotModel`, the model of its folder (#2903)
- <a id="rg10-24"></a>**RG10.24** DONE 2026-10-09 — The test suite of the GitHub wheels can fail (#2907) · [text](exudynRevisionLog2026b.md#closing-rg10-24)
- <a id="rg10-25"></a>**RG10.25** DONE 2026-10-09 — exudev names the compiler output only when there is one (#2913) · [text](exudynRevisionLog2026b.md#closing-rg10-25)
- <a id="rg10-26"></a>**RG10.26** DONE 2026-10-09 — The GitHub wheels of Linux ARM and macOS (#2918) · [text](exudynRevisionLog2026b.md#closing-rg10-26)
- <a id="rg10-27"></a>**RG10.27** DONE 2026-10-09 — The module test without tkinter (#2920) · [text](exudynRevisionLog2026b.md#closing-rg10-27)
- <a id="rg10-28"></a>**RG10.28** DONE 2026-10-09 — How a release is made (#2921) · [text](exudynRevisionLog2026b.md#closing-rg10-28)
- <a id="rg10-29"></a>**RG10.29** MOVED to revision2026c step RG10.29 — What the 1.13 release left to do · [text](exudynRevisionLog2026b.md#closing-rg10-29)
  - **RG10.29.1** MOVED to revision2026c step RG10.29.1 — the release run dirties the working tree (the stubs, line endings only), so it does not tag (#2915)
  - **RG10.29.2** MOVED to revision2026c step RG10.29.2 — `*.synctex` ignored, and the commit gate refuses a new tracked file above a few MB (#2916)
  - **RG10.29.3** MOVED to revision2026c step RG10.29.3 — GitHub CI as a release build before the release: a `workflow_dispatch` input that builds the fast module for every version, and a weekly run on a `release/*` branch (#...
  - **RG10.29.4** MOVED to revision2026c step RG10.29.4 — the runners write the home directory as `%USERPROFILE%` / `~` into the logs (#2923)
  - **RG10.29.5** MOVED to revision2026c step RG10.29.5 — what the release notes of a minor release contain - a decision of the maintainer (#2924)
  - **RG10.29.6** MOVED to revision2026c step RG10.29.6 — a check of the links of `README.rst` and of the PyPI description (#2925)
  - **RG10.29.7** MOVED to revision2026c step RG10.29.7 — the fast-module suite in the weekly routine, and its reference set renewed with the regular one (#2926)
  - **RG10.29.8** MOVED to revision2026c step RG10.29.8 — a dependency group `release` with twine >= 6.1 (#2927)
  - **RG10.29.9** MOVED to revision2026c step RG10.29.9 — Python 3.15 in GitHub CI, the version matrix and the documentation. (#2928)

## RG11 — Misc

- <a id="rg11-1"></a>**RG11.1** DONE 2026-09-24 — The results monitor runs beside the simulation, or it is redundant (#2610) · [text](exudynRevisionLog2026b.md#closing-rg11-1)
- <a id="rg11-3"></a>**RG11.3** DONE 2026-09-26 — The results monitor beside a running simulation (#2670) · [text](exudynRevisionLog2026b.md#closing-rg11-3)
- <a id="rg11-2"></a>**RG11.2** DONE 2026-09-23 — The demos wrote a `solution/` directory into whatever directory they were started in. (#2620) · [text](exudynRevisionLog2026b.md#closing-rg11-2)

## RG12 — Python interface

- <a id="rg12-1"></a>**RG12.1** DONE 2026-10-03 — `simulationSettings` gets the deprecation mechanism (#2588) · [text](exudynRevisionLog2026b.md#closing-rg12-1)
- <a id="rg12-2"></a>**RG12.2** DONE 2026-10-03 — Item parameters can be deprecated (#2589) · [text](exudynRevisionLog2026b.md#closing-rg12-2)
- <a id="rg12-3"></a>**RG12.3** DONE 2026-09-24 — What did this model actually change? (#2590) · [text](exudynRevisionLog2026b.md#closing-rg12-3)
- <a id="rg12-4"></a>**RG12.4** DONE 2026-09-26 — A user function is one typed Python function, and everything else is generated from it (#2664) · [text](exudynRevisionLog2026b.md#closing-rg12-4)
- <a id="rg12-5"></a>**RG12.5** DONE 2026-09-28 — User settings that persist between runs: one `~/.exudyn` file, and what may be in it (#2666) · [text](exudynRevisionLog2026b.md#closing-rg12-5)
- <a id="rg12-6"></a>**RG12.6** DONE 2026-09-26 — The columns of a settings dialog are relative and configurable (#2667) · [text](exudynRevisionLog2026b.md#closing-rg12-6)
- <a id="rg12-7"></a>**RG12.7** DONE 2026-09-26 — The mouse wheel changes the font size of a dialog (#2668) · [text](exudynRevisionLog2026b.md#closing-rg12-7)
- <a id="rg12-8"></a>**RG12.8** DONE 2026-09-26 — One test for all user functions at once (#2671) · [text](exudynRevisionLog2026b.md#closing-rg12-8)
- <a id="rg12-9"></a>**RG12.9** DONE 2026-09-26 — The override settings live in `exudyn.special.overrideSettings` (#2679) · [text](exudynRevisionLog2026b.md#closing-rg12-9)
- <a id="rg12-10"></a>**RG12.10** DONE 2026-09-26 — The workflow of the override settings (#2684) · [text](exudynRevisionLog2026b.md#closing-rg12-10)
- <a id="rg12-11"></a>**RG12.11** DONE 2026-09-26 — Storing the override settings (#2685) · [text](exudynRevisionLog2026b.md#closing-rg12-11)
- <a id="rg12-12"></a>**RG12.12** DONE 2026-09-27 — PlotSensor takes its defaults from the override settings (#2588) · [text](exudynRevisionLog2026b.md#closing-rg12-12)
- <a id="rg12-13"></a>**RG12.13** DONE 2026-09-26 — A stored dialog geometry is used (#2686) · [text](exudynRevisionLog2026b.md#closing-rg12-13)
- <a id="rg12-14"></a>**RG12.14** DONE 2026-09-26 — The override settings can be read again (#2687) · [text](exudynRevisionLog2026b.md#closing-rg12-14)
- <a id="rg12-15"></a>**RG12.15** DONE 2026-09-27 — A script can place a dialog, and it is written down (#2688) · [text](exudynRevisionLog2026b.md#closing-rg12-15)
- <a id="rg12-16"></a>**RG12.16** DONE 2026-09-26 — The render window and the SolutionViewer remember their size and position (#2689) · [text](exudynRevisionLog2026b.md#closing-rg12-16)
- <a id="rg12-19"></a>**RG12.19** DONE 2026-09-27 — Two buttons: one for the settings, one for the positions (#2693) · [text](exudynRevisionLog2026b.md#closing-rg12-19)
- <a id="rg12-20"></a>**RG12.20** DONE 2026-09-27 — Where the render window is, and what happens when the file and the session disagree (#2694) · [text](exudynRevisionLog2026b.md#closing-rg12-20)
- <a id="rg12-21"></a>**RG12.21** DONE 2026-09-27 — `python -m exudyn info` prints the home directory (#2695) · [text](exudynRevisionLog2026b.md#closing-rg12-21)
- <a id="rg12-22"></a>**RG12.22** DONE 2026-09-27 — The results monitor took the focus and came to the front on every update (#2696) · [text](exudynRevisionLog2026b.md#closing-rg12-22)
- <a id="rg12-23"></a>**RG12.23** DONE 2026-09-27 — The plot windows cannot be stored while the renderer is still open (#2698) · [text](exudynRevisionLog2026b.md#closing-rg12-23)
- <a id="rg12-24"></a>**RG12.24** DONE 2026-09-27 — The files a run writes by default go into `solution/` (#2718) · [text](exudynRevisionLog2026b.md#closing-rg12-24)
- <a id="rg12-25"></a>**RG12.25** DONE 2026-09-27 — Store positions stores every open window (#2719) · [text](exudynRevisionLog2026b.md#closing-rg12-25)
- <a id="rg12-26"></a>**RG12.26** DONE 2026-09-27 — The SolutionViewer follows the width of its window, takes a size and is stored with the other windows (#2720) · [text](exudynRevisionLog2026b.md#closing-rg12-26)
- <a id="rg12-27"></a>**RG12.27** DONE 2026-09-27 — Store positions with Qt plot windows, and the SolutionViewer when it is narrow (#2722) · [text](exudynRevisionLog2026b.md#closing-rg12-27)
- <a id="rg12-28"></a>**RG12.28** DONE 2026-09-27 — PlotSensor opens at the stored size (#2723) · [text](exudynRevisionLog2026b.md#closing-rg12-28)
- <a id="rg12-30"></a>**RG12.30** DONE 2026-09-30 — `exudyn.utilities` imports less (#2756, #2759, #2757) · [text](exudynRevisionLog2026b.md#closing-rg12-30)
- <a id="rg12-29"></a>**RG12.29** DONE 2026-10-01 (#2203) — What an item provides, asked from Python (#2203) · [text](exudynRevisionLog2026b.md#closing-rg12-29)
  - **RG12.29.1** DONE 2026-10-01 — the output variables that items declared and could not compute, found by that test, all implemented: `NodePoint2D` (`RotationMatrix`, `Rotation`, `AngularVelocity(Loca...
  - **RG12.29.2** DONE 2026-10-04 — node marker (#2817)
- <a id="rg12-31"></a>**RG12.31** DONE 2026-10-04 — Settings and item parameters that could be renamed or restructured (#2802) · [text](exudynRevisionLog2026b.md#closing-rg12-31)
- <a id="rg12-34"></a>**RG12.34** DONE 2026-10-04 — The simulation settings renamed and restructured as decided in RG12.31. (#2813) · [text](exudynRevisionLog2026b.md#closing-rg12-34)
  - **RG12.34.7** DONE 2026-10-04 — `SetDictionary` of the simulation and visualization settings takes a dictionary of Exudyn 1.11: its old keys go to their new places, and the keys it does not give keep...
  - **RG12.34.8** DONE 2026-10-04 — the binary solution file takes the size of its numbers from `solution.precision` (maintainer 2026-10-04) (#2816, #2818)
- <a id="rg12-36"></a>**RG12.36** DONE 2026-10-04 — Two Newton structures, and the modified Newton by default in the time integration (#2815) · [text](exudynRevisionLog2026b.md#closing-rg12-36)
- <a id="rg12-37"></a>**RG12.37** DONE 2026-10-04 — Go to definition of `mbs.CreateRigidBody` reaches the Python function (#2825) · [text](exudynRevisionLog2026b.md#closing-rg12-37)
- <a id="rg12-38"></a>**RG12.38** DONE 2026-10-04 — Return annotations for the functions added to `MainSystem` (#2826) · [text](exudynRevisionLog2026b.md#closing-rg12-38)
- <a id="rg12-35"></a>**RG12.35** DONE 2026-10-04 — The item parameters renamed as decided in RG12.31 (#2814) · [text](exudynRevisionLog2026b.md#closing-rg12-35)
- <a id="rg12-32"></a>**RG12.32** DONE 2026-10-03 — A deprecation warns where the user wrote the deprecated name, once, and is counted (#2804, #2805, #2806) · [text](exudynRevisionLog2026b.md#closing-rg12-32)
- <a id="rg12-33"></a>**RG12.33** DONE 2026-10-03 — Deprecations of the Python library, declared and checked (#2807) · [text](exudynRevisionLog2026b.md#closing-rg12-33)
- <a id="rg12-40"></a>**RG12.40** DONE 2026-10-05 — The Create functions take bodies, nodes or markers in `itemNumbers` (#2863, #2337) · [text](exudynRevisionLog2026b.md#closing-rg12-40)
- <a id="rg12-43"></a>**RG12.43** DONE 2026-10-06 — The shipped scripts without deprecated names (#2873) · [text](exudynRevisionLog2026b.md#closing-rg12-43)
- <a id="rg12-42"></a>**RG12.42** DONE 2026-10-06 — Joints and connectors with consistent output variables: `HomogeneousTransformation`, the new `HomogeneousTransformationLocal`, `Displacement`, and the outputs the 2D j... · [text](exudynRevisionLog2026b.md#closing-rg12-42)
- <a id="rg12-41"></a>**RG12.41** DONE 2026-10-05 — `mbs.CreateLinearSpringDamper` (#1953, #1954) · [text](exudynRevisionLog2026b.md#closing-rg12-41)
- <a id="rg12-39"></a>**RG12.39** DONE 2026-10-05 — A dynamic simulation continues from its restart file: `solution.restart.write` and `solution.restart.continueIfAvailable` (#2850) · [text](exudynRevisionLog2026b.md#closing-rg12-39)
- <a id="rg12-44"></a>**RG12.44** MOVED to revision2026c step RG11.4 — Dictionaries of the newer structures, and the visualization of items · [text](exudynRevisionLog2026b.md#closing-rg12-44)
  - **RG12.44.1** MOVED to revision2026c step RG11.4.1 — `GetDictionary`/`SetDictionary` for `SC.renderer` and `exudyn.config` (#2075)
  - **RG12.44.2** MOVED to revision2026c step RG11.4.2 — `CopyDictLevel1` replaced: the `visualization` of an item given as its `VItem` class or as a dict (#2342)
- <a id="rg12-45"></a>**RG12.45** MOVED to revision2026c step RG11.4.3 — The URDF import · [text](exudynRevisionLog2026b.md#closing-rg12-45)
- <a id="rg12-46"></a>**RG12.46** MOVED to revision2026c step RG11.4.4 — `MatrixContainer` and the matrices of other packages (#1863) · [text](exudynRevisionLog2026b.md#closing-rg12-46)

## RG13 — Item documentation

- <a id="rg13-1"></a>**RG13.1** DONE 2026-09-27 — The state of the documentation, item by item (#2715) · [text](exudynRevisionLog2026b.md#closing-rg13-1)
- <a id="rg13-2"></a>**RG13.2** CLOSED 2026-09-28 — What the ideal documentation of an item contains · [text](exudynRevisionLog2026b.md#closing-rg13-2)
- <a id="rg13-3"></a>**RG13.3** DONE 2026-10-06 — Each description synchronized once with its implementation, and the definition says so (#2717) · [text](exudynRevisionLog2026b.md#closing-rg13-3)
  - **RG13.3.1** DONE 2026-10-05 — the workflow - `tools/checkDescriptions.py` (`--mark`, `--check` in `exudev generate --all-checks`), `definitions/descriptionChecks.json` - and the first group: the 16...
  - **RG13.3.6** DONE 2026-10-07 — the decisions carried out: ObjectContactCurveCircles computes its output variables and has friction (#2867)
- <a id="rg13-4"></a>**RG13.4** DONE 2026-09-29 — The development documents per item type (#2721) · [text](exudynRevisionLog2026b.md#closing-rg13-4)
- <a id="rg13-5"></a>**RG13.5** DONE 2026-09-30 — The documentation of the items, written by the documents of RG13.4 (#2725) · [text](exudynRevisionLog2026b.md#closing-rg13-5)
- <a id="rg13-6"></a>**RG13.6** DONE 2026-10-03 — A MiniExample for every item (#2732) · [text](exudynRevisionLog2026b.md#closing-rg13-6)
- <a id="rg13-7"></a>**RG13.7** DONE 2026-09-29 — How to set up a new item (#2742) · [text](exudynRevisionLog2026b.md#closing-rg13-7)
- <a id="rg13-8"></a>**RG13.8** DONE 2026-10-04 — How an item is drawn (#2840) · [text](exudynRevisionLog2026b.md#closing-rg13-8)
  - **RG13.8.1** DONE 2026-10-04 — an inventory, per item: the settings and the parameters its `UpdateGraphics` reads, its default size and color, what it draws (and what not, e.g
  - **RG13.8.2** DONE 2026-10-04 — a section *Drawing* in the generated frame of each item page, from a declaration in the definition (e.g
  - **RG13.8.3** DONE 2026-10-04 — the general rules once per kind, in `itemKindDefinitions.py`: nodes, markers, loads and sensors are drawn alike within their kind (size, color, number), objects one by...
  - **RG13.8.5** DONE 2026-10-04 — `ObjectContactCurveCircles` draws the points of its curve with `contact.contactPointsDefaultSize` instead of the deprecated `connectors.contactPointsDefaultSize` (#2846)
  - **RG13.8.4** DONE 2026-10-04 — the findings of the inventory: parameters that are not read (`ObjectConnectorDistance.drawSize`), items with drawing parameters and no drawing (`ObjectConnectorGravity...
- <a id="rg13-9"></a>**RG13.9** MOVED to revision2026c step RG11.5 — The old documentation issues of single items · [text](exudynRevisionLog2026b.md#closing-rg13-9)

## RG14 — Marker values computed where they are used

- <a id="rg14-1"></a>**RG14.1** CLOSED 2026-10-01 — The evaluation · [text](exudynRevisionLog2026b.md#closing-rg14-1)
- <a id="rg14-2"></a>**RG14.2** DONE 2026-10-04 — The migration (#2745) · [text](exudynRevisionLog2026b.md#closing-rg14-2)
- <a id="rg14-3"></a>**RG14.3** MOVED to revision2026c step RG11.6 — Joints and their Jacobians on homogeneous transformations. (#2884, successor of #2745) · [text](exudynRevisionLog2026b.md#closing-rg14-3)

## RG15 — Objects computing from given coordinates

- <a id="rg15-1"></a>**RG15.1** DONE 2026-09-29 — The evaluation. · [text](exudynRevisionLog2026b.md#closing-rg15-1)

## RG16 — Homogeneous transformations

- <a id="rg16-1"></a>**RG16.1** DONE 2026-10-02 — The C++ class and its Python binding (#2780) · [text](exudynRevisionLog2026b.md#closing-rg16-1)
- <a id="rg16-2"></a>**RG16.2** DECIDED 2026-10-02 — The evaluation: HT in the user interface of the rigid items · [text](exudynRevisionLog2026b.md#closing-rg16-2)
- <a id="rg16-3"></a>**RG16.3** DONE 2026-10-03 — The cases that break nothing for users · [text](exudynRevisionLog2026b.md#closing-rg16-3)
- <a id="rg16-4"></a>**RG16.4** DONE 2026-10-03 — The further steps · [text](exudynRevisionLog2026b.md#closing-rg16-4)
- <a id="rg16-5"></a>**RG16.5** DONE 2026-10-03 — `localHT` in the rigid markers · [text](exudynRevisionLog2026b.md#closing-rg16-5)
- <a id="rg16-6"></a>**RG16.6** DECIDED 2026-10-03 — What `exu.HT` offers, and how its parts are named (#2809) · [text](exudynRevisionLog2026b.md#closing-rg16-6)
- <a id="rg16-7"></a>**RG16.7** DONE 2026-10-03 — `exu.HT` as decided in RG16.6 (#2810) · [text](exudynRevisionLog2026b.md#closing-rg16-7)
- <a id="rg16-8"></a>**RG16.8** DONE 2026-10-04 — The logarithm and the exponential map in `exu.HT` (#2819) · [text](exudynRevisionLog2026b.md#closing-rg16-8)
- <a id="rg16-9"></a>**RG16.9** DONE 2026-10-04 — The library builds the frames of markers with `exu.HT`, and no Create function passes `rotationMarker0/1` (#2820) · [text](exudynRevisionLog2026b.md#closing-rg16-9)
- <a id="rg16-10"></a>**RG16.10** DONE 2026-10-04 — The robotics classes on `exu.HT` (#2821) · [text](exudynRevisionLog2026b.md#closing-rg16-10)
  - **RG16.10.1** DECIDED 2026-10-04 — as proposed, with indexing for writing: the robotics classes store and compute with `exu.HT` (products in C++), and `LinkHT`/`JointHT`/`COMHT` return lists of `exu.HT`
  - **RG16.10.2** DONE 2026-10-04 — `roboticsCore.py` on `exu.HT`, the forward kinematics and the Jacobian measured against today's (a robot of 6 and of 20 links)
  - **RG16.10.3** DONE 2026-10-04 — `models.py` (the robot definitions: DH parameters, `preHT`, tool frames), `mobile.py`, `special.py`, `future.py`, `utilities.py`
  - **RG16.10.4** DONE 2026-10-04 — the scripts with the Robot class or the kinematic tree, each with its result unchanged: the examples `serialRobot*.py` (8), `humanRobotInteraction.py`, `InverseKinemat...
  - **RG16.10.5** DECIDED 2026-10-04 — the HT functions of `rigidBodyUtilities` (`HomogeneousTransformation`, `HTtranslate`, ...): kept as numpy helpers or deprecated with `exu.HT` as advice - (maintainer),...
- <a id="rg16-11"></a>**RG16.11** DONE 2026-10-04 — Scripts that build rigid frames use `exu.HT` (#2822) · [text](exudynRevisionLog2026b.md#closing-rg16-11)
- <a id="rg16-12"></a>**RG16.12** DONE (#2823) — Examples and test models that show `referenceHT`, `localHT` and `exu.HT` (#2823) · [text](exudynRevisionLog2026b.md#closing-rg16-12)
  - **RG16.12.1** DONE 2026-10-04 — the new example `homogeneousTransformationInterpolation.py`: four rigid bodies, each held by a generic joint to a ground whose `referenceHT` a PreStepUserFunction sets...
  - **RG16.12.2** DONE 2026-10-04 — `solutionViewerTest.py`: the chain of 100 bodies built from `exu.HT` products, the bodies with `referenceHT`, the joints on markers with `localHT`
  - **RG16.12.5** DONE 2026-10-04 by RG17.2.7 — the notebook `tutorialRigidBodyCreate` replaces `rigidBodyTutorial3.py`: it names `referenceHT` and shows the frame of a body as `exu.HT`
- <a id="rg16-13"></a>**RG16.13** DONE 2026-10-04 (#2824) — `ObjectKinematicTree` and its `jointHTs` (#2824) · [text](exudynRevisionLog2026b.md#closing-rg16-13)
  - **RG16.13.1** DONE 2026-10-04 — `mbs.CreateKinematicTree` passes `jointHTs`, `TreeLink.jointHT` is an `exu.HT` (given as `exu.HT` or 4x4 matrix), `exu.HT(H)` copies an HT, and the MiniExample of `Obj...
  - **RG16.13.2** DONE 2026-10-04 — whether storing one HT per joint in `CObjectKinematicTree` saves time: computing the joint transformations `XL` once instead of per evaluation changed nothing measurab...
  - **RG16.13.3** DONE 2026-10-04 — with RG16.10: `Robot.CreateKinematicTree` gives `jointHTs` of `exu.HT`
  - **RG16.13.4** DONE 2026-10-04 — `mbs.CreateKinematicTree` with automatic graphics failed for a link without offset (`UnboundLocalError: gLink`), found by the measurement of RG16.13.2. (#2827)
  - **RG16.13.5** DONE 2026-10-04 — the unused 6x6 Pluecker matrices removed from `KinematicsBasics.h`: the `#ifndef USE_EFFICIENT_TRANSFORMATION66` branch (about 300 lines, never compiled, the switch al...
  - **RG16.13.6** DONE 2026-10-04 — the functions that matter (#2829): `ComputeTreeTransformations` (positions, velocities and accelerations of all links: the output variables, markers and sensors) and `...
  - **RG16.13.7** DONE 2026-10-04 — two local implementations of those two functions in `CObjectKinematicTree.cpp` on the HT directly: positions and rotations with `HomogeneousTransformation` products, m...
  - **RG16.13.8** DONE 2026-10-04 — the comparison: every test model and MiniExample of the kinematic tree with both paths (equal to round-off), and the time per evaluation on trees of 6 and of 50 links
  - **RG16.13.9** DONE 2026-10-04 — if the HT path agrees and is not slower: it becomes the only one, the switch goes, and the T66 functions of `KinematicsBasics.h` go with it - except what the 6D motion...
  - **RG16.13.10** DONE 2026-10-04 — the `forceUserFunction` of `ObjectKinematicTree` received an empty vector as `q_t`, found while moving the forces per joint into one function. (#2845)

## RG17 — Notebooks

- <a id="rg17-1"></a>**RG17.1** DONE 2026-10-04 — Evaluate notebooks for tutorials and examples (#2811) · [text](exudynRevisionLog2026b.md#closing-rg17-1)
- <a id="rg17-2"></a>**RG17.2** DONE 2026-10-04 — The notebook tooling and the first notebooks (#2831) · [text](exudynRevisionLog2026b.md#closing-rg17-2)
  - **RG17.2.1** DONE (with its parent step) — `exudyn.interactive.ShowImage(SC, size, modelRotation, zoomAll, show, fileName)`: the scene as an image of the raytracer, no window, shown with matplotlib
  - **RG17.2.2** DONE (with its parent step) — `tools/runNotebooks.py`: runs each notebook in an interpreter of its own (no window, files to a temporary directory) and stores the outputs in it - printed text, the v...
  - **RG17.2.3** DONE (with its parent step) — `tools/generators/notebookEmitter.py`, a stage of the regeneration: per notebook the page `docs/generated/notebooks/<name>.md` (code, stored outputs, images) and the e...
  - **RG17.2.4** DONE (with its parent step) — the first two, as extra tutorials beside the old ones (`docs/manual/tutorial.md`): `tutorialSpringDamper.ipynb` (exact solution and Exudyn in one plot) and `tutorialRi...
  - **RG17.2.5** DONE (with its parent step) — the systematic way for the examples of the reference manual, shown on one: `pb.AddDocuNotebook(path)` in `definitions/pybind*.py` instead of `pb.AddDocuCodeBlock(code)...
  - **RG17.2.6** DONE 2026-10-04 — every notebook names itself, `python/Notebooks/<name>.ipynb` in code font, in its first cell (the emitter and `AddDocuNotebook` refuse one that does not)
  - **RG17.2.7** DONE 2026-10-04 — all tutorials as notebooks, with the text of the tutorial pages: `tutorialSpringDamper` (items) and `tutorialSpringDamperCreate`, `tutorialRigidBody` (markers and join...
- <a id="rg17-3"></a>**RG17.3** DONE 2026-10-04 — The tutorials are the notebooks (#2831) · [text](exudynRevisionLog2026b.md#closing-rg17-3)
- <a id="rg17-4"></a>**RG17.4** DONE 2026-10-04 — Every example of the reference manual a notebook (#2831) · [text](exudynRevisionLog2026b.md#closing-rg17-4)
  - **RG17.4.1** DONE 2026-10-04 — 30 of the 34 code blocks are five notebooks of `python/Notebooks/reference/` - `generalInformation` (with the examples of `pybindModule.py` and `pybindEnums.py`), `mai...
  - **RG17.4.2** DONE 2026-10-04 — the test: `python/testing/test_referenceNotebooks.py` runs every reference notebook (`runNotebooks.py --test`, nothing stored) and checks that every notebook stores th...
  - **RG17.4.3** DONE 2026-10-04 — `SC.AppendSystem` of a `MainSystem` that Python owns (a copy, or the system of another container) is deleted twice - an access violation at the end of the copy example...
- <a id="rg17-5"></a>**RG17.5** DONE 2026-10-04 — **The snippets of the user · [text](exudynRevisionLog2026b.md#closing-rg17-5)
  - **RG17.5.1** DONE 2026-10-04 — three notebooks of `python/Notebooks/snippets/` hold 25 of the 55 Python blocks: `graphics` (the five `GraphicsData` examples and the raytracing snippet as one model,...

## RG18 — Resolving old issues

- <a id="rg18-1"></a>**RG18.1** MOVED to revision2026c step RG11.8 — The quick ones · [text](exudynRevisionLog2026b.md#closing-rg18-1)
  - **RG18.1.1** MOVED to revision2026c step RG11.8.1 — `override` on every virtual function of the derived classes in `src/Linalg` (#123)
  - **RG18.1.2** MOVED to revision2026c step RG11.8.2 — `MarkerNodeCoordinate`: a flag to include the reference value in the coordinate (#990)
  - **RG18.1.3** MOVED to revision2026c step RG11.8.3 — the manual: what is a copy and what a reference - `mbs`, `GetObject(...)`, the dicts, the settings structures (#1061)
  - **RG18.1.4** DONE 2026-10-07 — the version line of the solution and sensor file headers ends with the number of threads, e.g (#1450)
  - **RG18.1.5** MOVED to revision2026c step RG11.8.4 — the static solver refuses Lie group nodes with a message, if they cannot work there (checked first) (#1493)
  - **RG18.1.6** MOVED to revision2026c step RG11.8.5 — `GetSystemJacobian()` and `GetSystemMassMatrix()` of the solvers with `sparseTriplets=False`, returning sparse triplets for large systems (#1529)
  - **RG18.1.7** MOVED to revision2026c step RG11.8.6 — the static temporaries of `GeneralMatrixEXUdense::FactorizeNew` and of `Matrix.cpp` as members, freed when the solver finishes (#1614)
  - **RG18.1.8** MOVED to revision2026c step RG11.8.7 — an example of two mass-spring-dampers coupled, one system implicit and one explicit (#1912)
  - **RG18.1.9** MOVED to revision2026c step RG11.8.8 — the symmetric basis of two non-parallel vectors (the mid axis of the normalized vectors and the projections), in `rigidBodyUtilities` (#2228)
  - **RG18.1.10** MOVED to revision2026c step RG11.8.9 — a warning of the initial accelerations when the initial velocities do not fulfill the constraints, which gives oscillating constraint forces (#2234)
  - **RG18.1.11** MOVED to revision2026c step RG11.8.10 — the manual: model-centric and camera-centric views (#2247)
  - **RG18.1.12** MOVED to revision2026c step RG11.8.11 — `GetURDFrobotData`: the returned dict described key by key (#2267)
  - **RG18.1.13** MOVED to revision2026c step RG11.8.12 — `PyWriteBodyGraphicsDataList`: the item and the kind of data named in an error, and every point checked to be finite (#2304, #2305)
  - **RG18.1.14** MOVED to revision2026c step RG11.8.13 — the comments of `exudyn.shells` in the format of the other modules (#2349)
  - **RG18.1.15** MOVED to revision2026c step RG11.8.14 — `mecanumWheelRollingDiscTest.py` with the `Trajectory` class and `ObjectConnectorTorsionalSpringDamper`, same result (#948)
  - **RG18.1.16** MOVED to revision2026c step RG11.8.15 — `SystemContainer(...)` without the renderer, for a container used only to compute (as in the inverse kinematics) (#2259)
  - **RG18.1.17** MOVED to revision2026c step RG11.8.16 — the full message of a solver failure in the exception of `SolveStatic`/`SolveDynamic` (#1977)
  - **RG18.1.18** MOVED to revision2026c step RG11.8.17 — a timer of the Python user functions among the solver timers (#2007)
- <a id="rg18-2"></a>**RG18.2** MOVED to revision2026c step RG11.9 — The velocity offset in all spring-dampers (#829, #1631, #1189) · [text](exudynRevisionLog2026b.md#closing-rg18-2)
